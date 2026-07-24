# ORB-SLAM3 Remote Web Visualizer Design

**Status:** Approved design
**Date:** 2026-07-24
**Implementation status:** Deferred
**Target repository:** `ORB_SLAM3`
**Primary deployment:** x86-64 CPU-only host, remote access through a Tailscale tailnet

## 1. Purpose

Build an optional, read-only web visualizer for the customized ORB-SLAM3 fork. It will reproduce the useful visualization behavior of the Pangolin viewer while rendering remotely in a browser with Three.js.

The visualizer must show:

- The current camera pose and frustum.
- An interactive 3D sparse map.
- Keyframe frustums.
- The keyframe trajectory.
- Spanning-tree and loop-closure edges.
- A live camera trajectory trail.
- Tracking state and map statistics.
- One compressed, low-resolution camera image with tracked-feature overlays.
- Pangolin-like follow-camera, perspective, top-view, visibility, zoom, orbit, and pan behavior.

The visualizer is observational only. It must not expose reset, localization-mode, atlas-save, shutdown, shell, file-write, or other state-changing controls.

## 2. Goals

1. Add less than 1 ms p99 tracking-thread overhead for publishing visualization state.
2. Prevent slow networks and slow browser clients from blocking ORB-SLAM3.
3. Keep map access safe across tracking, local mapping, loop closing, reset, atlas load, and map merge.
4. Use bounded CPU, memory, and network resources.
5. Keep ORB-SLAM3 independent of frontend frameworks and web build tools.
6. Allow Pangolin and the web viewer to be enabled or disabled independently.
7. Make the wire protocol versioned so other viewers can be implemented later.
8. Support a sustained network limit of 20 Mbit/s while targeting less than 5 Mbit/s in normal use.
9. Preserve smooth browser interaction while point-map or image data is transferred.
10. Make the backend reusable for future semi-dense stereo point clouds without changing the browser transport architecture.

## 3. Non-goals

- Remote control of ORB-SLAM3.
- Navigation, planning, or robot control through the viewer.
- Browser-side SLAM or bundle adjustment.
- Streaming raw uncompressed camera frames.
- Reusing Pangolin OpenGL calls in the browser.
- Public Internet exposure without Tailscale.
- Video-quality 30 or 60 FPS camera streaming.
- Dense mesh rendering in the first implementation.
- ROS integration.
- Persisting visualization state as part of the ORB atlas.

## 4. Existing Code Constraints

### 4.1 Pangolin rendering

The current viewer creates a local OpenGL context and directly reads mutable ORB-SLAM3 objects. `MapDrawer` issues legacy immediate-mode calls such as `glBegin`, `glVertex3f`, and `glMultMatrixf`.

A browser cannot share that OpenGL context or access C++ pointers. WebGL also does not support the legacy immediate-mode API. The web viewer therefore reuses the scene data and visual semantics, not the rendering calls.

### 4.2 Existing value snapshots

The customized fork already exposes:

- `FrameSnapshot`
- `KeyframeSnapshot`
- `GraphSnapshot`
- `System::GetLastFrameSnapshot()`
- `System::GetGraphSnapshot()`

These are a suitable foundation because they do not expose `Frame*`, `KeyFrame*`, or `Map*` to consumers.

The existing snapshots do not include:

- Map-point positions.
- Visualization-specific point identifiers.
- Current image data.
- Tracked keypoint overlay data.
- Separate topology and geometry revisions.
- All Pangolin graph edge types.

### 4.3 Current lock behavior

`System::GetGraphSnapshot()` stabilizes the current map by locking reset, map-update, and atlas state. `Map::GetGraphSnapshotData()` then holds the map mutex while copying keyframes and taking keyframe locks.

This is safe but unsuitable for high-rate polling. A web client must never directly invoke it at rendering frequency.

### 4.4 Existing revision behavior

The current graph revision is primarily derived from `mnBigChangeIdx`. It does not identify every map-point insertion, map-point removal, keyframe insertion, or keyframe removal. Visualization needs more precise revision categories.

### 4.5 Current image drawing

`FrameDrawer::DrawFrame()` copies the source image and a complete `Frame`, then renders overlays with OpenCV. Calling it from the web backend would add unnecessary locking, memory copying, BGR conversion, drawing, and JPEG work.

The web path will publish a grayscale image and compact overlay vectors separately. The browser will draw the overlay.

## 5. Architectural Decision

Use an ORB-owned value snapshot source, an optional C++ web backend, two asynchronous WebSocket streams, and a Three.js frontend.

```text
┌──────────────────────── ORB-SLAM3 core ────────────────────────┐
│ Tracking              Local Mapping              Loop Closing  │
│     │                       │                           │       │
│     └──── value snapshots and bounded map-change events ─┘      │
└────────────────────────────┬────────────────────────────────────┘
                             │ nonblocking publication
┌────────────────────────────▼────────────────────────────────────┐
│ Optional WebViewer backend                                     │
│ map mirror → snapshot cache → encoder → client fanout          │
└────────────────┬──────────────────────────┬─────────────────────┘
                 │ realtime WebSocket       │ bulk WebSocket
                 ▼                          ▼
          pose / status             map / graph / JPEG
                 └──────────────┬───────────┘
                                ▼
                     Three.js browser frontend
```

The backend is an observer. Any backend error disables or degrades visualization without changing SLAM behavior.

## 6. Module Boundaries

### 6.1 `visualization_types`

A small ORB-SLAM3 core module containing only owned, value-based types:

- Frame state.
- Keyframe state.
- Graph edges.
- Sparse map points.
- Image metadata and immutable image storage.
- Feature overlay points.
- Epoch and revision values.

It contains no HTTP, WebSocket, JSON, Three.js, browser, or Tailscale concepts.

### 6.2 `orb_visualization_source`

The only module allowed to read ORB-SLAM3 internals. Its responsibilities are:

- Publish the latest frame state.
- Publish the latest eligible camera image and overlay.
- Publish bounded, value-only map-change events.
- Seed and recover an independent visualization mirror at safe map commit points.
- Convert mutable ORB state into immutable value snapshots.
- Track visualization epochs and revisions.

### 6.3 `web_viewer_protocol`

Defines:

- Wire header.
- Message types.
- Binary payload layouts.
- Epoch and revision semantics.
- Coordinate conventions.
- Protocol-version negotiation.

It does not contain socket or renderer code.

### 6.4 `web_viewer_backend`

An optional library containing:

- Boost.Asio/Beast HTTP and WebSocket service.
- Static frontend asset service.
- Snapshot scheduling.
- Binary encoding.
- JPEG encoding.
- Immutable payload caching.
- Rate limiting.
- Per-client bounded queues.
- Connection and heartbeat handling.

The initial implementation uses Boost.Asio and Boost.Beast. The protocol and snapshot source remain independent of this choice.

### 6.5 `web_viewer_frontend`

A TypeScript application using:

- Three.js.
- WebGL2 where available.
- `OrbitControls`.
- A Web Worker for binary point-map parsing.
- A 2D canvas for image feature overlays.

The frontend is built once into static assets. Node.js is a build-time dependency only. The robot runs the compiled assets without Node.js.

## 7. Core Snapshot Model

### 7.1 Revisions

Each active map exposes four monotonic values:

```text
epoch
topology_revision
geometry_revision
graph_revision
frame_sequence
```

Their meanings are:

- `epoch`: changes on full reset, active-map reset, atlas load, or a map transition that invalidates prior render identifiers.
- `topology_revision`: changes when a keyframe or map point is inserted, removed, or marked bad.
- `geometry_revision`: changes after an operation can change keyframe poses or map-point positions, including local BA, global BA, loop correction, scale correction, and map merge.
- `graph_revision`: changes when parent, loop, merge, covisibility, or inertial graph relationships change.
- `frame_sequence`: increments for each published tracking frame.

Revisions may skip values. Clients compare equality and ordering; they do not assume consecutive delivery.

### 7.2 Frame snapshot

The realtime frame snapshot contains:

```cpp
struct VisualizationFrameSnapshot {
    std::uint64_t epoch;
    std::uint64_t sequence;
    std::int64_t capture_timestamp_ns;
    std::uint64_t map_id;
    std::uint64_t reference_keyframe_id;
    int tracking_state;
    bool pose_valid;
    Sophus::SE3f T_world_camera;
    std::uint32_t tracked_keypoints;
    std::uint32_t tracked_map_points;
};
```

Publishing this snapshot must not allocate proportional to map size.

### 7.3 Image and overlay snapshot

The image snapshot contains:

```cpp
struct VisualizationFeature {
    float x;
    float y;
    std::uint8_t state;
};

struct VisualizationImageSnapshot {
    std::uint64_t epoch;
    std::uint64_t frame_sequence;
    std::int64_t capture_timestamp_ns;
    cv::Mat immutable_grayscale_image;
    std::vector<VisualizationFeature> features;
};
```

`immutable_grayscale_image` owns or shares an immutable reference-counted buffer. The backend may resize and encode it after the tracking call returns. ORB-SLAM3 never modifies a published buffer.

Feature states are:

```text
0 = untracked
1 = visual-odometry match
2 = map-point match
3 = outlier
```

Only states 1 through 3 need to be transmitted.

### 7.4 Map snapshot

The map snapshot contains:

```cpp
struct VisualizationMapPoint {
    std::uint64_t id;
    Eigen::Vector3f world_position;
    bool reference;
};

struct VisualizationKeyframe {
    std::uint64_t id;
    double timestamp;
    Sophus::SE3f T_world_camera;
    bool bad;
    bool has_parent;
    std::uint64_t parent_id;
    std::vector<std::uint64_t> loop_edges;
    std::vector<std::uint64_t> merge_edges;
};

struct VisualizationMapSnapshot {
    std::uint64_t epoch;
    std::uint64_t map_id;
    std::uint64_t topology_revision;
    std::uint64_t geometry_revision;
    std::uint64_t graph_revision;
    std::vector<VisualizationMapPoint> points;
    std::vector<VisualizationKeyframe> keyframes;
};
```

The first implementation includes parent, loop, and merge edges. Covisibility and inertial edges are added only if their commit-boundary publication satisfies the performance budget.

### 7.5 Visualization map mirror

Normal web operation does not repeatedly scan the mutable ORB map. `orb_visualization_source` publishes value-only changes into a bounded multi-producer queue, and the backend maintains an independent mirror:

```cpp
struct VisualizationMapEvent {
    std::uint64_t epoch;
    std::uint64_t topology_revision;
    std::uint64_t geometry_revision;
    std::uint64_t graph_revision;
    VisualizationEventType type;
    std::shared_ptr<const VisualizationEventPayload> payload;
};
```

Event categories are:

```text
POINTS_ADDED
POINTS_REMOVED
POINTS_UPDATED
KEYFRAMES_ADDED
KEYFRAMES_REMOVED
KEYFRAMES_UPDATED
GRAPH_UPDATED
MAP_REPLACED
EPOCH_RESET
```

Geometry-producing operations publish authoritative batches only at commit boundaries, not during individual optimizer iterations. A newer authoritative batch may replace an older unconsumed batch for the same map and event category.

The mirror owns standard value containers keyed by ORB IDs. It has no `MapPoint*`, `KeyFrame*`, `Map*`, or other non-owning ORB addresses. Browser snapshots and deltas are produced from this mirror, so client count and frontend refresh rate cannot create ORB lock pressure.

The mirror is seeded:

- With an empty map at new-map creation.
- From the loaded atlas after deserialization and before normal tracking begins.
- With an authoritative replacement batch at map-merge commit.

If the viewer is enabled, these seed operations are part of the existing map lifecycle while the map is already stabilized. The viewer cannot be enabled dynamically after `System` startup.

Topology records use fixed-size inline payloads in a preallocated ring; they do not allocate a shared payload for each new point. Geometry and replacement events reference immutable batch vectors because they are produced once per map commit rather than per optimizer iteration.

The event queue has a 16 MiB byte limit, including referenced batch payloads. Producers never wait for queue space. If a publication cannot fit:

1. Set an atomic `mirror_resync_required` flag.
2. Preserve frame-state publication.
3. Stop publishing map deltas from the stale mirror.
4. Produce one authoritative `MAP_REPLACED` batch at the next map commit boundary.
5. Resume deltas after that batch is applied.

This recovery path does not initiate an arbitrary full-map scan from a network or browser thread.

## 8. Snapshot Publication and Locking

### 8.1 Frame publication

At the end of `TrackMonocular`, `TrackStereo`, or `TrackRGBD`:

1. Build the small frame snapshot from values already computed by tracking.
2. Publish it with an atomic shared-pointer swap.
3. Do not perform JPEG encoding, network I/O, JSON encoding, point copying, or frontend-specific conversion.

C++11 `atomic_load` and `atomic_store` operations for `std::shared_ptr` are sufficient for immutable latest-value publication.

### 8.2 Image publication

Image publication is rate-gated before any additional image work. The default gate is 5 Hz.

When the gate opens:

1. Publish an immutable reference to the current grayscale input image.
2. Copy only compact feature coordinates and states.
3. Return to tracking.

Resize and JPEG encoding run on the backend encoder thread.

If a safe immutable reference cannot be published without an image copy, the implementation may copy the grayscale image. The measured tracking overhead must still satisfy the acceptance criteria.

### 8.3 Map publication

Map writers publish values they already own at stable commit boundaries:

- `Map::AddMapPoint` and `Map::EraseMapPoint` contribute point topology changes.
- `Map::AddKeyFrame` and `Map::EraseKeyFrame` contribute keyframe topology changes.
- Local mapping contributes an authoritative batch for points and keyframes changed by local BA.
- Loop closing contributes an authoritative geometry and graph batch after loop correction or global BA.
- Atlas load contributes one full seed before tracking workers begin.
- Map merge contributes one replacement batch before the merged map is exposed as the active visualization epoch.

Per-object topology events contain only IDs and the minimum required values. Optimizer publication is batched so it does not allocate or enqueue once per numerical iteration.

The publication API returns immediately. Failure marks the mirror for authoritative resynchronization at a later map commit boundary.

Event creation and map mutation must preserve the lock ordering already established by the safe snapshot code and its concurrency tests. Publication never performs encoding or network work while ORB locks are held.

### 8.4 Publication scheduling

Default scheduling:

```text
frame state:      publish every tracking frame
image snapshot:   maximum 5 Hz
map mirror drain: continuously, coalesced to maximum 20 published deltas/s
wire delta flush: maximum 2 Hz
full wire cache:  rebuild from the mirror every 10 seconds or after replacement
forced refresh:   after geometry replacement, reset, load, or merge
```

Full wire-cache rebuilds read only the backend-owned mirror. They do not scan ORB objects or acquire ORB map locks.

### 8.5 Snapshot cache

The backend stores:

- Latest frame payload.
- Latest image payload.
- Latest graph payload.
- Latest full point-map payload.
- A bounded delta history.

A new client is served exclusively from this cache. Client connection must not cause ORB map capture.

If the mirror is marked stale, the cache retains the last internally consistent full snapshot and exposes the stale state in statistics. It does not invent or partially apply map state.

## 9. Backend Processing Pipeline

### 9.1 Threads

The default backend uses:

1. One mirror/scheduling thread.
2. One encoding thread.
3. One asynchronous network I/O thread.

JPEG and large map encoding never run on the network I/O thread.

If JPEG profiling shows sustained encoder saturation, a second encoder thread may be enabled through configuration. Thread count remains bounded.

### 9.2 Queues

Queues are bounded by item count and bytes:

- Frame-state slot: one item, latest value wins.
- Image slot: one unencoded and one encoded item, latest value wins.
- Map-event queue: 16 MiB shared producer limit with authoritative-batch coalescing.
- Map-encode queue: one pending mirror snapshot; a newer snapshot replaces an unstarted older snapshot.
- Per-client realtime queue: maximum 256 KiB.
- Per-client bulk queue: maximum 4 MiB.

When the bulk queue reaches its limit:

1. Drop unsent image messages.
2. Coalesce map deltas into a future full snapshot.
3. Preserve the active full-snapshot transfer if it matches the current epoch.
4. Disconnect and require resynchronization if bounded recovery is impossible.

The backend never retains unbounded per-frame or per-client history.

### 9.3 Multi-client fanout

JPEG and map payloads are encoded once. Each client queue stores shared immutable references to the same buffers.

The default concurrent-client limit is four. A slow client has its own queue and cannot delay another client.

## 10. Transport Design

### 10.1 HTTP endpoints

```text
GET /                 compiled frontend
GET /assets/*         hashed static assets
GET /healthz          process and backend health summary
GET /config           public read-only viewer configuration
WS  /ws/realtime      latency-sensitive messages
WS  /ws/bulk          map, graph, and image messages
```

No state-changing HTTP methods or endpoints are provided.

### 10.2 Two WebSocket streams

`/ws/realtime` carries:

- Server hello.
- Frame poses.
- Tracking state.
- Statistics.
- Epoch changes.
- Heartbeats.
- Error notices.

`/ws/bulk` carries:

- Full graph state.
- Graph changes.
- Full point-map chunks.
- Point-map deltas.
- JPEG camera images.
- Feature overlays.

Splitting streams prevents a large ordered WebSocket message from placing a fresh pose behind it at the application layer. Both streams remain subject to the same physical network congestion, but realtime traffic has an independent send queue.

### 10.3 Wire header

Every binary message begins with a manually serialized 48-byte, little-endian header:

```text
offset  size  field
0       4     magic = 0x4f524257 ("ORBW")
4       2     protocol version = 0x0100
6       2     message type
8       4     flags
12      4     payload size in bytes
16      8     epoch
24      8     revision relevant to this message
32      8     sequence
40      8     capture timestamp in nanoseconds
```

Payload layouts are explicit and must not serialize native C++ structs directly because compiler padding and ABI layout are not stable.

### 10.4 Message types

Protocol version 1 defines:

```text
1   SERVER_HELLO
2   FRAME_STATE
3   STATISTICS
4   EPOCH_RESET
5   HEARTBEAT
6   ERROR_NOTICE
20  GRAPH_FULL
21  GRAPH_DELTA
30  POINTS_FULL_BEGIN
31  POINTS_FULL_CHUNK
32  POINTS_FULL_END
33  POINTS_DELTA
40  IMAGE_JPEG
41  FEATURE_OVERLAY
```

Unknown message types are ignored if their payload length is valid. A major protocol-version mismatch closes the connection with an explanatory error.

### 10.5 Full point encoding

The backend assigns session-local 32-bit render IDs to ORB map-point IDs. The mapping is reset when the visualization epoch changes.

Point chunks use local quantization:

```text
chunk origin: float32 x, y, z
quantization scale: float32 meters per unit
point count: uint32

per point:
render_id: uint32
x, y, z:  int16 relative to chunk origin
flags:    uint16
```

The default scale is 0.002 m. A chunk origin is selected so all encoded coordinates fit signed 16-bit range. Large maps are divided into multiple chunks.

Each point consumes 12 bytes. One hundred thousand points therefore require approximately 1.2 MB plus chunk headers. The format preserves 2 mm visualization resolution, which is finer than the displayed point size and typical sparse-map uncertainty.

### 10.6 Point deltas

A point delta contains three arrays:

- Additions with render ID, quantized position, and flags.
- Position updates with render ID and quantized position.
- Removals with render ID.

If the backend cannot construct a consistent delta from cached revisions, it sends a new full snapshot instead.

### 10.7 Images

JPEG messages contain binary JPEG bytes without Base64 encoding. Base64 would add approximately one-third overhead.

Default image settings:

```text
source:       left or monocular grayscale image
resolution:   maximum 640 × 400, aspect ratio preserved
rate:         5 FPS
JPEG quality: 55
```

The image and feature overlay use the same `frame_sequence` so the browser does not combine mismatched frames.

## 11. Bandwidth and Backpressure

### 11.1 Expected bandwidth

```text
frame pose and telemetry:     below 10 KiB/s
graph deltas:                 normally below 20 KiB/s
640 × 400 JPEG at 5 FPS:      approximately 0.6–3.2 Mbit/s
normal sparse-point deltas:   normally below 1 Mbit/s
100k-point full snapshot:     approximately 1.2 MB, occasional
```

The default steady-state target is below 5 Mbit/s per client.

### 11.2 Rate limiter

Each client uses a token bucket:

```text
steady rate: 6 Mbit/s
burst:       2 MiB
```

Realtime traffic is accounted separately and is always scheduled before bulk traffic.

### 11.3 Adaptive degradation

The backend measures:

- Socket send-queue bytes.
- WebSocket heartbeat round-trip time.
- Image acknowledgement age.
- Bulk snapshot progress.

When pressure rises, degradation occurs in this order:

1. Drop stale, unsent JPEG frames.
2. Reduce image rate from 5 FPS to 2 FPS.
3. Reduce JPEG quality from 55 to 40.
4. Reduce the long image dimension from 640 to 480 pixels.
5. Pause image transmission.
6. Delay point-map chunks.
7. Replace accumulated deltas with one future full resynchronization.

Frame poses, tracking state, and heartbeats continue unless the connection itself is unusable.

Recovery reverses these steps slowly after 10 seconds of stable queue and RTT measurements to avoid oscillation.

## 12. Latency Model

The viewer is not part of any control loop.

Target end-to-end age:

```text
frame pose:    below 100 ms typical, below 150 ms p95
camera image:  below 250 ms typical, below 400 ms p95
graph update:  below 2 seconds
point map:     below 5 seconds after a major correction
```

Every payload carries the original capture timestamp. The frontend displays current data age and marks the connection degraded when pose age exceeds 500 ms.

The browser renders the newest pose immediately. It may interpolate between the two newest poses for visual smoothness with a maximum 50 ms interpolation delay. It does not extrapolate robot state.

## 13. Frontend Rendering Design

### 13.1 Scene graph

The frontend uses a small, fixed number of render objects:

- One `THREE.Points` for normal map points.
- One `THREE.Points` for reference map points.
- One instanced frustum geometry for keyframes.
- One frustum for the current camera.
- One `THREE.LineSegments` object for graph edges.
- One `THREE.Line` for the corrected keyframe path.
- One `THREE.Line` for the recent live frame trail.
- One grid and axis helper.

It does not create one JavaScript or DOM object per map point.

### 13.2 GPU buffers

- Point positions and colors use typed arrays.
- Buffer capacity grows geometrically instead of reallocating for every delta.
- Updates mark only modified buffer ranges when practical.
- Deleted points are handled with a slot free-list and periodic compaction.
- Full snapshots build replacement arrays in a Web Worker and swap them on the render thread.
- Keyframe frustums use instancing to keep draw calls bounded.

Default limits:

```text
displayed sparse points: 250,000
keyframes:                20,000
live trail samples:      10,000
device pixel ratio:      min(devicePixelRatio, 1.5)
```

If the point limit is exceeded, deterministic ID-based sampling is used so points do not flicker between updates.

### 13.3 Render scheduling

The browser renders continuously only when:

- The user is orbiting, panning, or zooming.
- Follow-camera mode is active and new poses are arriving.
- New scene data requires animation.

When stationary, rendering is dirty-event driven. This reduces client CPU and GPU use.

### 13.4 Camera image

The browser decodes JPEG frames with `createImageBitmap()` when supported. It draws the bitmap into a 2D canvas and draws feature markers in a transparent overlay canvas.

No React or DOM node is created for individual features.

### 13.5 View controls

Version 1 provides:

- Orbit, pan, zoom.
- Reset view.
- Perspective view.
- Top view.
- Follow current camera.
- Follow gravity-aligned body orientation when inertial initialization is available.
- Show/hide sparse points.
- Show/hide keyframes.
- Show/hide corrected path.
- Show/hide live trail.
- Show/hide graph edges.
- Show/hide camera image.
- Adjustable visual point size stored only in browser preferences.

All controls modify browser rendering only.

## 14. Coordinate Conventions

Wire data remains in ORB-SLAM3 world coordinates.

The frontend applies one central right-handed conversion:

```text
(x, y, z)_three = (x, -y, -z)_orb
```

For rotations:

```text
C = diag(1, -1, -1)
R_three = C * R_orb * C^-1
t_three = C * t_orb
```

Frustum geometry is defined once in ORB camera convention and converted through the same transform. Coordinate conversion must not be duplicated across render components.

## 15. Connection Lifecycle

1. Browser loads static assets over HTTP or HTTPS.
2. Browser opens `/ws/realtime`.
3. Server sends `SERVER_HELLO` with instance identity, protocol version, epoch, and public configuration.
4. Browser opens `/ws/bulk`.
5. Server sends current cached graph and point-map snapshots.
6. Realtime poses continue while bulk synchronization is in progress.
7. Browser applies deltas only after the corresponding full snapshot completes.
8. On an epoch change, browser clears map-owned objects and requests cached resynchronization by reconnecting the bulk stream.
9. On network loss, browser reconnects with exponential backoff capped at 10 seconds.

A backend process restart changes the server instance identity. The frontend discards all prior state even if numeric epochs happen to match.

## 16. Tailscale Deployment and Security

Default binding:

```text
bind address: 127.0.0.1
port:         8080
```

Tailscale Serve proxies the loopback service to authenticated tailnet devices and supplies HTTPS. Direct public binding is disabled by default.

Security rules:

- No state-changing endpoints.
- No directory browsing.
- No arbitrary file paths.
- Static assets are served from a fixed compiled directory or embedded resource table.
- WebSocket origin is checked against the configured Tailscale hostname and loopback development origins.
- Payload sizes are validated before allocation.
- Client count and queue memory are capped.
- Protocol errors close only the offending connection.
- Viewer failure cannot terminate ORB-SLAM3.

## 17. Configuration

The ORB settings file gains a `WebViewer` section:

```yaml
WebViewer.Enabled: 0
WebViewer.BindAddress: "127.0.0.1"
WebViewer.Port: 8080
WebViewer.StaticRoot: "./web_viewer/dist"
WebViewer.MaxClients: 4
WebViewer.ImageEnabled: 1
WebViewer.ImageWidth: 640
WebViewer.ImageHeight: 400
WebViewer.ImageFps: 5
WebViewer.ImageJpegQuality: 55
WebViewer.MapDeltaFps: 2
WebViewer.FullCacheRefreshSeconds: 10
WebViewer.MapEventQueueBytes: 16777216
WebViewer.MaxPoints: 250000
WebViewer.ClientRateMbps: 6
WebViewer.ClientBurstBytes: 2097152
WebViewer.RealtimeQueueBytes: 262144
WebViewer.BulkQueueBytes: 4194304
```

Invalid web-viewer settings disable the web viewer and report a clear error. They do not prevent ORB-SLAM3 from starting unless the application explicitly marks visualization as required.

## 18. Error Handling

### Backend startup failure

Log the reason, mark WebViewer unhealthy, and continue SLAM without the web viewer.

### Map-event publication failure

Mark the visualization mirror stale and request one authoritative replacement batch at the next stable map commit. Continue frame and image publication.

### Encoding failure

Drop the affected image or map payload, retain the last good cached snapshot, and increment an error metric.

### Slow client

Apply adaptive degradation. Disconnect only that client if its bounded queue cannot recover.

### Malformed client data

The protocol is read-only, so client messages are limited to handshake and heartbeat acknowledgements. Invalid lengths, versions, or message types close that connection.

### Epoch mismatch

Discard queued deltas, clear render-ID mappings, and send the latest cached full snapshot.

## 19. Observability

The backend tracks:

- Connected clients.
- Map-event queue bytes and high-water mark.
- Map publication duration at commit boundaries.
- Mirror resynchronization count and stale duration.
- Snapshot point and keyframe counts.
- JPEG encode duration and size.
- Map encode duration and size.
- Per-client realtime and bulk queue bytes.
- Per-client estimated send rate.
- Heartbeat RTT.
- Dropped images.
- Coalesced deltas.
- Client disconnect reasons.

`/healthz` returns a compact read-only JSON summary. Detailed metrics are also available through the application logger without requiring a metrics database.

## 20. Performance Requirements

On the AMD Ryzen 7 5825U development host:

### ORB-SLAM3 impact

- Frame-state publication p99: less than 1 ms per tracking frame.
- Frame-state publication median: less than 0.2 ms.
- Individual topology-event publication p99: less than 0.1 ms.
- Map-event producers never wait for viewer queue space.
- Geometry batches are published only at existing map commit boundaries.
- No periodic web-viewer task scans the mutable ORB map during normal operation.
- Tracking FPS p50 and p99 may not regress by more than 2% with the viewer enabled and no clients.
- Tracking FPS p50 and p99 may not regress by more than 2% with one normal client or one throttled client.
- Dataset ATE must remain within the viewer-disabled run-to-run 95% confidence interval.

### Backend

- Steady CPU use: less than one CPU core with default settings.
- Typical backend memory: less than 100 MiB for a 250,000-point map and four clients.
- No memory growth during a two-hour reconnect and slow-client soak test.
- Default steady bandwidth: less than 5 Mbit/s per client.

### Frontend

- 60 FPS interaction target with 200,000 points on a contemporary laptop browser.
- 30 FPS minimum interaction with the configured 250,000-point limit.
- Full-map parsing occurs off the render thread.
- No main-thread pause longer than 50 ms during normal point deltas.

## 21. Validation Strategy

### 21.1 Unit tests

- Wire-header encoding and decoding.
- Payload bounds and malformed-message rejection.
- Epoch and revision ordering.
- Render-ID allocation and reset.
- Point quantization error and chunk-boundary behavior.
- Full snapshot and delta equivalence.
- Queue replacement and byte limits.
- Rate-limiter behavior.
- Coordinate conversion.

### 21.2 ORB concurrency tests

- Event publication during keyframe insertion.
- Event publication during keyframe culling.
- Event publication during map-point insertion and erasure.
- Authoritative geometry publication after local BA.
- Authoritative replacement publication after loop closure and map merge.
- Reset and atlas load while events are pending.
- Forced queue overflow followed by commit-boundary mirror recovery.
- Verification of established atlas, map-update, map, keyframe, and map-point lock ordering.

Existing snapshot test seams should be extended instead of creating production-only sleeps or timing assumptions.

### 21.3 Backend integration tests

- Browser connection before map initialization.
- Initial full synchronization.
- Live point and keyframe growth.
- Loop closure followed by geometry replacement.
- Lost tracking and successful relocalization.
- Server restart and client resynchronization.
- Four clients with one deliberately stalled.
- Malformed and oversized messages.

### 21.4 Performance tests

Use recorded datasets and a synthetic scene containing:

```text
250,000 map points
20,000 keyframes
10,000 path samples
four connected clients
```

Measure:

- Tracking frame-time distribution with viewer disabled.
- Viewer enabled with zero clients.
- One local client.
- One Tailscale client.
- One client limited to 20 Mbit/s.
- One slow client with added latency and packet loss.
- Four mixed-speed clients.

Linux traffic shaping should test 20 Mbit/s, 100 ms RTT, 1% packet loss, and short outages. Test results must include CPU, memory, queue sizes, dropped images, pose age, and SLAM frame timing.

### 21.5 Frontend tests

- Typed-array buffer growth and compaction.
- Web Worker snapshot parsing.
- Reconnect and epoch clearing.
- Follow-camera mode.
- All visibility toggles.
- Resize and high-DPI behavior.
- WebGL context loss and restoration.
- Chrome and Firefox on x86-64.

## 22. Delivery Sequence

### Milestone 1: scene foundation

- Visualization revisions.
- Nonblocking immutable map snapshot.
- Versioned protocol.
- Backend serving static assets.
- Browser rendering camera pose, keyframes, path, and sparse points.

### Milestone 2: realtime robustness

- Separate realtime and bulk WebSockets.
- Bounded queues.
- Cached full synchronization.
- Point deltas.
- Loop-closure geometry replacement.
- Reconnection and epoch handling.

### Milestone 3: camera panel

- Rate-gated immutable grayscale image snapshot.
- Off-thread JPEG encoding.
- Feature-overlay stream.
- Canvas overlay.
- Adaptive image quality and rate.

### Milestone 4: performance qualification

- Synthetic large-map benchmark.
- Dataset regression tests.
- Slow-client and Tailscale network tests.
- Publication-duration and queue-pressure instrumentation.
- Configuration tuning against acceptance criteria.

No milestone introduces remote state-changing commands.

## 23. Rejected Alternatives

### Server-side Pangolin video streaming

Rejected because it consumes more bandwidth, adds round-trip input latency, prevents true client-side 3D interaction, and requires graphics rendering on the robot.

### Compiling Pangolin to WebAssembly

Rejected because Pangolin windowing and legacy immediate-mode OpenGL are not browser-compatible. Porting them would cost more than implementing a small data-driven Three.js renderer.

### One WebSocket

Rejected because ordered bulk messages can delay realtime pose messages behind point-map or JPEG data.

### Full point-map transmission every second

Rejected because a 100,000-point map would consume a large fraction of the 20 Mbit/s link and repeatedly stall browser buffer replacement.

### Periodic full scans of the mutable ORB map

Rejected because copying all map points requires map stabilization and per-point position reads. Even nonblocking lock acquisition cannot bound the lock hold after a large scan begins. A commit-fed, value-only visualization mirror isolates web refresh and client behavior from ORB map locks.

### Calling `FrameDrawer::DrawFrame()` for web images

Rejected because it copies a complete frame, renders overlays on the backend, increases image entropy, and creates avoidable lock contention.

### Per-client ORB snapshots

Rejected because client behavior would directly determine ORB lock pressure and CPU cost. All clients must consume shared cached snapshots.

### WebRTC for the first camera stream

Rejected for the initial 5 FPS grayscale stream because JPEG over the bulk WebSocket is simpler, independently droppable, and fits the bandwidth budget. WebRTC can be reconsidered only if future requirements demand high-frame-rate video.

## 24. Future Compatibility

The protocol can later add distinct render layers for:

- Semi-dense stereo points.
- Occupancy voxels.
- Local navigation obstacles.
- Multiple cameras.
- IMU state and bias plots.
- Loop-closure diagnostics.

These additions use new message types and independent visibility layers. They do not change the read-only security model or allow the visualization path to influence SLAM.

## 25. Final Decision Summary

- Render remotely with Three.js, not Pangolin commands.
- Publish immutable frame snapshots and bounded map-change events from ORB-SLAM3.
- Maintain a backend-owned visualization mirror instead of polling the mutable map.
- Keep web code in an optional backend module.
- Protect SLAM with nonblocking capture and bounded queues.
- Cache and encode once for all clients.
- Separate realtime and bulk WebSockets.
- Send grayscale JPEG plus vector feature overlays.
- Use initial full state, deltas, and rare full resynchronization.
- Bind locally and expose through Tailscale Serve.
- Enforce measurable CPU, memory, latency, and bandwidth budgets before declaring the implementation complete.
