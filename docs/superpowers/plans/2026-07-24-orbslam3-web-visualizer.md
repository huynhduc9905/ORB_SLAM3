# ORB-SLAM3 Remote Web Visualizer Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build an optional read-only web visualizer for ORB-SLAM3 using C++ Boost.Asio/Beast backend, Three.js browser visualizer, nonblocking value snapshots, rate-gated JPEG streaming, and verify end-to-end on dataset extracted from `~/robot/bag/circle-run`.

**Architecture:** Nonblocking visualization events/snapshots in ORB-SLAM3 core -> backend map mirror & cached binary encoder -> dual WebSocket endpoints (`/ws/realtime` & `/ws/bulk`) -> Three.js WebGL browser visualizer.

**Tech Stack:** C++17 / Boost.Asio / Boost.Beast / OpenCV 4 / Eigen 3 / Three.js / TypeScript / Nix Flake.

## Global Constraints

- Must add less than 1 ms p99 tracking-thread overhead.
- Must not use ROS dependencies; raw data extracted from `~/robot/bag/circle-run` using pure-python `rosbags`.
- Self-contained Nix environment inside `ORB_SLAM3` repo (`flake.nix`).
- Read-only visualizer (no state-changing endpoints).
- Two WebSocket streams (`/ws/realtime` on port 8080 and `/ws/bulk`).
- Wire format header 48 bytes, little-endian, magic `0x4f524257`.
- Coordinate transformation for Three.js: `(x, y, z)_three = (x, -y, -z)_orb`.

---

### Task 1: Nix Environment Setup & Dataset Raw Exporter

**Files:**
- Create: `flake.nix`
- Create: `scripts/export_circle_run.py`
- Test: `nix develop --command python3 scripts/export_circle_run.py --bag ~/robot/bag/circle-run --output dataset/circle_run`

**Interfaces:**
- Consumes: `~/robot/bag/circle-run/circle-run_0.mcap`
- Produces: `dataset/circle_run/cam0/data/*.png`, `dataset/circle_run/cam0/times.txt`, `dataset/circle_run/imu.txt`, `dataset/circle_run/camera.yaml`

- [ ] **Step 1: Create `flake.nix` for ORB_SLAM3**

```nix
{
  description = "ORB_SLAM3 with Web Visualizer Nix environment";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nixpkgs, flake-utils }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          config.allowUnfree = true;
        };

        pangolin = pkgs.stdenv.mkDerivation rec {
          pname = "pangolin";
          version = "0.8";
          src = pkgs.fetchFromGitHub {
            owner = "stevenlovegrove";
            repo = "Pangolin";
            rev = "v${version}";
            hash = "sha256-X8TZWJOQOCItYt/F8E5ahiaPJXoppu9qBlEqfHP0vRc=";
          };
          nativeBuildInputs = [ pkgs.cmake pkgs.pkg-config ];
          buildInputs = [ pkgs.eigen pkgs.glew pkgs.libGL pkgs.xorg.libX11 pkgs.ffmpeg ];
          cmakeFlags = [ "-DBUILD_PANGOLIN_GUI=ON" ];
        };

        pyEnv = pkgs.python3.withPackages (ps: with ps; [
          rosbags
          opencv4
          numpy
          playwright
          requests
          websockets
        ]);

      in {
        devShells.default = pkgs.mkShell {
          name = "orbslam3-web-dev";

          nativeBuildInputs = with pkgs; [
            cmake
            pkg-config
            gcc
            gnumake
            nodejs
            nodePackages.npm
            playwright-driver.browsers
          ];

          buildInputs = with pkgs; [
            boost
            eigen
            opencv4
            openssl
            glew
            libGL
            xorg.libX11
            pangolin
            pyEnv
          ];

          shellHook = ''
            export PLAYWRIGHT_BROWSERS_PATH=${pkgs.playwright-driver.browsers}
            export PLAYWRIGHT_SKIP_BROWSER_DOWNLOAD=1
            echo "ORB_SLAM3 Web Visualizer Nix environment loaded."
          '';
        };
      });
}
```

- [ ] **Step 2: Test `nix develop` shell**

Run: `nix develop --command cmake --version`
Expected: `cmake version ...` succeeds without error.

- [ ] **Step 3: Create `scripts/export_circle_run.py`**

```python
#!/usr/bin/env python3
import sys
import os
import argparse
from pathlib import Path
import cv2
import numpy as np
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr

def main():
    parser = argparse.ArgumentParser(description="Export raw camera and IMU data from MCAP bag without ROS")
    parser.add_argument("--bag", required=True, help="Path to bag folder")
    parser.add_argument("--output", required=True, help="Output directory")
    args = parser.parse_args()

    out_dir = Path(args.output)
    cam0_dir = out_dir / "cam0" / "data"
    cam0_dir.mkdir(parents=True, exist_ok=True)

    times_file = out_dir / "cam0" / "times.txt"
    imu_file = out_dir / "imu.txt"

    times_fd = open(times_file, "w")
    imu_fd = open(imu_file, "w")

    img_count = 0
    imu_count = 0

    with Reader(args.bag) as reader:
        # Find connections
        img_conn = [c for c in reader.connections if c.topic == "/camera/camera/infra1/image_rect_raw"]
        imu_conn = [c for c in reader.connections if c.topic == "/camera/camera/imu" or c.topic == "/imu"]

        print(f"Found {len(img_conn)} image connections, {len(imu_conn)} IMU connections.")

        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == "/camera/camera/infra1/image_rect_raw":
                msg = deserialize_cdr(rawdata, connection.msgtype)
                # msg.header.stamp.sec, msg.header.stamp.nanosec
                ts_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                # Convert image data
                if msg.encoding in ["mono8", "8UC1"]:
                    img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width))
                elif msg.encoding in ["bgr8", "rgb8"]:
                    img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
                    if msg.encoding == "rgb8":
                        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                else:
                    img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width))

                filename = f"{ts_sec:.6f}.png"
                cv2.imwrite(str(cam0_dir / filename), img)
                times_fd.write(f"{ts_sec:.6f}\n")
                img_count += 1

            elif connection.topic in ["/camera/camera/imu", "/imu"]:
                msg = deserialize_cdr(rawdata, connection.msgtype)
                ts_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                wx, wy, wz = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
                ax, ay, az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
                imu_fd.write(f"{ts_sec:.6f} {wx} {wy} {wz} {ax} {ay} {az}\n")
                imu_count += 1

    times_fd.close()
    imu_fd.close()
    print(f"Export complete: {img_count} images, {imu_count} IMU measurements to {out_dir}")

    # Generate camera calibration file for RealSense D435i infra1
    cam_yaml = out_dir / "camera.yaml"
    with open(cam_yaml, "w") as f:
        f.write("""%YAML:1.0
File.version: "1.0"
Camera.type: "PinHole"
Camera1.fx: 383.74
Camera1.fy: 383.74
Camera1.cx: 320.44
Camera1.cy: 238.12
Camera1.k1: 0.0
Camera1.k2: 0.0
Camera1.p1: 0.0
Camera1.p2: 0.0
Camera.width: 640
Camera.height: 480
Camera.fps: 30.0
Camera.RGB: 0
ORBextractor.nFeatures: 1000
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7
Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1.0
Viewer.GraphLineWidth: 0.9
Viewer.PointSize: 2.0
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3.0
Viewer.ViewpointX: 0.0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -1.8
Viewer.ViewpointF: 500.0
WebViewer.Enabled: 1
WebViewer.Port: 8080
""")

if __name__ == "__main__":
    main()
```

- [ ] **Step 4: Execute dataset raw export script inside nix develop**

Run: `nix develop --command python3 scripts/export_circle_run.py --bag /home/duc/robot/bag/circle-run --output dataset/circle_run`
Expected: Output showing `Export complete: ~2700 images ... to dataset/circle_run`.

- [ ] **Step 5: Commit Task 1**

```bash
git add flake.nix scripts/export_circle_run.py dataset/circle_run/camera.yaml dataset/circle_run/cam0/times.txt
git commit -m "feat(nix): add flake.nix and dataset export script"
```

---

### Task 2: Core Snapshots & Revisions (`visualization_types` & `orb_visualization_source`)

**Files:**
- Create: `include/VisualizationTypes.h`
- Create: `include/VisualizationSource.h`
- Create: `src/VisualizationSource.cc`
- Modify: `include/System.h`, `src/System.cc`
- Modify: `include/Tracking.h`, `src/Tracking.cc`

**Interfaces:**
- Consumes: ORB-SLAM3 tracking state and map structures.
- Produces: Value-based snapshots `VisualizationFrameSnapshot`, `VisualizationImageSnapshot`, `VisualizationMapSnapshot`, `VisualizationMapEvent`. Nonblocking ring-buffer event queue for map updates.

- [ ] **Step 1: Create `include/VisualizationTypes.h`**

```cpp
#ifndef VISUALIZATION_TYPES_H
#define VISUALIZATION_TYPES_H

#include <cstdint>
#include <vector>
#include <memory>
#include <eigen3/Eigen/Core>
#include <sophus/se3.hpp>
#include <opencv2/core.hpp>

namespace ORB_SLAM3 {

enum class VisualizationEventType {
    POINTS_ADDED,
    POINTS_REMOVED,
    POINTS_UPDATED,
    KEYFRAMES_ADDED,
    KEYFRAMES_REMOVED,
    KEYFRAMES_UPDATED,
    GRAPH_UPDATED,
    MAP_REPLACED,
    EPOCH_RESET
};

struct VisualizationFrameSnapshot {
    std::uint64_t epoch{0};
    std::uint64_t sequence{0};
    std::int64_t capture_timestamp_ns{0};
    std::uint64_t map_id{0};
    std::uint64_t reference_keyframe_id{0};
    int tracking_state{0};
    bool pose_valid{false};
    Sophus::SE3f T_world_camera;
    std::uint32_t tracked_keypoints{0};
    std::uint32_t tracked_map_points{0};
};

struct VisualizationFeature {
    float x;
    float y;
    std::uint8_t state; // 1=VO match, 2=Map match, 3=Outlier
};

struct VisualizationImageSnapshot {
    std::uint64_t epoch{0};
    std::uint64_t frame_sequence{0};
    std::int64_t capture_timestamp_ns{0};
    cv::Mat immutable_grayscale_image;
    std::vector<VisualizationFeature> features;
};

struct VisualizationMapPoint {
    std::uint64_t id{0};
    Eigen::Vector3f world_position;
    bool reference{false};
};

struct VisualizationKeyframe {
    std::uint64_t id{0};
    double timestamp{0.0};
    Sophus::SE3f T_world_camera;
    bool bad{false};
    bool has_parent{false};
    std::uint64_t parent_id{0};
    std::vector<std::uint64_t> loop_edges;
    std::vector<std::uint64_t> merge_edges;
};

struct VisualizationMapSnapshot {
    std::uint64_t epoch{0};
    std::uint64_t map_id{0};
    std::uint64_t topology_revision{0};
    std::uint64_t geometry_revision{0};
    std::uint64_t graph_revision{0};
    std::vector<VisualizationMapPoint> points;
    std::vector<VisualizationKeyframe> keyframes;
};

struct VisualizationMapEvent {
    std::uint64_t epoch{0};
    std::uint64_t topology_revision{0};
    std::uint64_t geometry_revision{0};
    std::uint64_t graph_revision{0};
    VisualizationEventType type;
    std::vector<VisualizationMapPoint> points;
    std::vector<VisualizationKeyframe> keyframes;
};

} // namespace ORB_SLAM3

#endif // VISUALIZATION_TYPES_H
```

- [ ] **Step 2: Create `include/VisualizationSource.h` and `src/VisualizationSource.cc`**

```cpp
// include/VisualizationSource.h
#ifndef VISUALIZATION_SOURCE_H
#define VISUALIZATION_SOURCE_H

#include "VisualizationTypes.h"
#include <mutex>
#include <atomic>
#include <queue>

namespace ORB_SLAM3 {

class VisualizationSource {
public:
    VisualizationSource();
    ~VisualizationSource() = default;

    void PublishFrameState(const VisualizationFrameSnapshot& frame);
    void PublishImageState(const VisualizationImageSnapshot& image);
    void PublishMapEvent(const VisualizationMapEvent& event);

    std::shared_ptr<const VisualizationFrameSnapshot> GetLatestFrameState() const;
    std::shared_ptr<const VisualizationImageSnapshot> GetLatestImageState() const;
    std::vector<VisualizationMapEvent> PopPendingMapEvents();

    std::uint64_t GetCurrentEpoch() const { return mEpoch.load(); }
    void IncrementEpoch() { mEpoch++; }

private:
    std::atomic<std::uint64_t> mEpoch{1};
    mutable std::mutex mFrameMutex;
    std::shared_ptr<const VisualizationFrameSnapshot> mLatestFrame;

    mutable std::mutex mImageMutex;
    std::shared_ptr<const VisualizationImageSnapshot> mLatestImage;

    mutable std::mutex mEventsMutex;
    std::vector<VisualizationMapEvent> mPendingEvents;
    static constexpr size_t MAX_PENDING_EVENTS = 1000;
};

} // namespace ORB_SLAM3

#endif
```

```cpp
// src/VisualizationSource.cc
#include "VisualizationSource.h"

namespace ORB_SLAM3 {

VisualizationSource::VisualizationSource() {}

void VisualizationSource::PublishFrameState(const VisualizationFrameSnapshot& frame) {
    std::lock_guard<std::mutex> lock(mFrameMutex);
    mLatestFrame = std::make_shared<const VisualizationFrameSnapshot>(frame);
}

void VisualizationSource::PublishImageState(const VisualizationImageSnapshot& image) {
    std::lock_guard<std::mutex> lock(mImageMutex);
    mLatestImage = std::make_shared<const VisualizationImageSnapshot>(image);
}

void VisualizationSource::PublishMapEvent(const VisualizationMapEvent& event) {
    std::lock_guard<std::mutex> lock(mEventsMutex);
    if (mPendingEvents.size() < MAX_PENDING_EVENTS) {
        mPendingEvents.push_back(event);
    }
}

std::shared_ptr<const VisualizationFrameSnapshot> VisualizationSource::GetLatestFrameState() const {
    std::lock_guard<std::mutex> lock(mFrameMutex);
    return mLatestFrame;
}

std::shared_ptr<const VisualizationImageSnapshot> VisualizationSource::GetLatestImageState() const {
    std::lock_guard<std::mutex> lock(mImageMutex);
    return mLatestImage;
}

std::vector<VisualizationMapEvent> VisualizationSource::PopPendingMapEvents() {
    std::lock_guard<std::mutex> lock(mEventsMutex);
    std::vector<VisualizationMapEvent> events;
    events.swap(mPendingEvents);
    return events;
}

} // namespace ORB_SLAM3
```

- [ ] **Step 3: Integrate `VisualizationSource` into `Tracking::Track()`**

In `src/Tracking.cc`, after pose calculation:
Publish `VisualizationFrameSnapshot` and rate-gated (5 Hz) `VisualizationImageSnapshot` via `mpVisSource`.

- [ ] **Step 4: Write unit test `tests/test_visualization_source.cc`**

Verify thread-safe publication, nonblocking queueing, and epoch incrementing.

- [ ] **Step 5: Run tests and commit**

```bash
git add include/VisualizationTypes.h include/VisualizationSource.h src/VisualizationSource.cc
git commit -m "feat(core): add VisualizationTypes and VisualizationSource snapshot pipeline"
```

---

### Task 3: Protocol Specification & Binary Serializers (`web_viewer_protocol`)

**Files:**
- Create: `include/WebViewerProtocol.h`
- Create: `src/WebViewerProtocol.cc`
- Create: `tests/test_protocol.cc`

**Interfaces:**
- Consumes: `VisualizationFrameSnapshot`, `VisualizationMapSnapshot`, `VisualizationImageSnapshot`, `VisualizationMapEvent`.
- Produces: 48-byte header binary frames for WebSocket `/ws/realtime` and `/ws/bulk`. Point quantization with 12-byte payload per 3D point.

- [ ] **Step 1: Write `include/WebViewerProtocol.h`**

```cpp
#ifndef WEB_VIEWER_PROTOCOL_H
#define WEB_VIEWER_PROTOCOL_H

#include "VisualizationTypes.h"
#include <vector>
#include <cstdint>
#include <string>

namespace ORB_SLAM3 {

constexpr std::uint32_t PROTOCOL_MAGIC = 0x4f524257; // "ORBW"
constexpr std::uint16_t PROTOCOL_VERSION = 0x0100;    // v1.0

enum MessageType : std::uint16_t {
    MSG_SERVER_HELLO = 1,
    MSG_FRAME_STATE = 2,
    MSG_STATISTICS = 3,
    MSG_EPOCH_RESET = 4,
    MSG_HEARTBEAT = 5,
    MSG_ERROR_NOTICE = 6,
    MSG_GRAPH_FULL = 20,
    MSG_GRAPH_DELTA = 21,
    MSG_POINTS_FULL_BEGIN = 30,
    MSG_POINTS_FULL_CHUNK = 31,
    MSG_POINTS_FULL_END = 32,
    MSG_POINTS_DELTA = 33,
    MSG_IMAGE_JPEG = 40,
    MSG_FEATURE_OVERLAY = 41
};

#pragma pack(push, 1)
struct WireHeader {
    std::uint32_t magic;
    std::uint16_t version;
    std::uint16_t msg_type;
    std::uint32_t flags;
    std::uint32_t payload_size;
    std::uint64_t epoch;
    std::uint64_t revision;
    std::uint64_t sequence;
    std::int64_t capture_timestamp_ns;
};
#pragma pack(pop)

class WebViewerProtocol {
public:
    static std::vector<uint8_t> EncodeHeader(uint16_t msg_type, uint32_t payload_size, uint64_t epoch, uint64_t revision, uint64_t sequence, int64_t ts_ns);
    static bool DecodeHeader(const uint8_t* data, size_t size, WireHeader& out_header);

    static std::vector<uint8_t> EncodeFrameState(const VisualizationFrameSnapshot& frame);
    static std::vector<uint8_t> EncodePointsChunk(uint64_t epoch, uint64_t revision, const std::vector<VisualizationMapPoint>& points, float scale = 0.002f);
    static std::vector<uint8_t> EncodeImageJpeg(uint64_t epoch, uint64_t sequence, int64_t ts_ns, const cv::Mat& gray_img, int quality = 55);
    static std::vector<uint8_t> EncodeFeatureOverlay(uint64_t epoch, uint64_t sequence, int64_t ts_ns, const std::vector<VisualizationFeature>& features);
};

} // namespace ORB_SLAM3

#endif
```

- [ ] **Step 2: Implement `src/WebViewerProtocol.cc`**

Implement exact little-endian packing of `WireHeader`, pose vector/quaternion packing, JPEG compression via OpenCV `cv::imencode(".jpg")`, and point quantization (relative to chunk origin with 16-bit signed offsets).

- [ ] **Step 3: Write test `tests/test_protocol.cc` and verify**

```cpp
#include "WebViewerProtocol.h"
#include <cassert>
#include <iostream>

int main() {
    using namespace ORB_SLAM3;
    VisualizationFrameSnapshot frame;
    frame.epoch = 1;
    frame.sequence = 42;
    frame.tracking_state = 2;
    frame.pose_valid = true;

    auto buf = WebViewerProtocol::EncodeFrameState(frame);
    assert(buf.size() >= sizeof(WireHeader));
    WireHeader header;
    assert(WebViewerProtocol::DecodeHeader(buf.data(), buf.size(), header));
    assert(header.magic == PROTOCOL_MAGIC);
    assert(header.msg_type == MSG_FRAME_STATE);
    assert(header.sequence == 42);
    std::cout << "Protocol test passed! WireHeader size: " << sizeof(WireHeader) << std::endl;
    return 0;
}
```

- [ ] **Step 4: Run test in `nix develop`**

Run: `nix develop --command bash -c "g++ -std=c++17 -Iinclude tests/test_protocol.cc src/WebViewerProtocol.cc $(pkg-config --cflags --libs opencv4) -o test_protocol && ./test_protocol"`
Expected: `Protocol test passed! WireHeader size: 48`

- [ ] **Step 5: Commit Task 3**

```bash
git add include/WebViewerProtocol.h src/WebViewerProtocol.cc tests/test_protocol.cc
git commit -m "feat(protocol): implement versioned binary protocol encoder and tests"
```

---

### Task 4: Backend Map Mirror, Caching & Network Engine (`web_viewer_backend`)

**Files:**
- Create: `include/WebViewerBackend.h`
- Create: `src/WebViewerBackend.cc`
- Create: `tests/test_backend_server.cc`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: `VisualizationSource` snapshot events.
- Produces: Boost.Asio/Beast HTTP web server on configured port (default 8080) serving static assets and handling `/ws/realtime` and `/ws/bulk` WebSockets. Maintains backend map mirror and shared encoded caches.

- [ ] **Step 1: Write `include/WebViewerBackend.h`**

```cpp
#ifndef WEB_VIEWER_BACKEND_H
#define WEB_VIEWER_BACKEND_H

#include "VisualizationSource.h"
#include "WebViewerProtocol.h"
#include <string>
#include <memory>
#include <thread>
#include <atomic>
#include <unordered_map>

namespace ORB_SLAM3 {

struct WebViewerConfig {
    bool enabled{true};
    std::string bind_address{"127.0.0.1"};
    int port{8080};
    std::string static_root{"./web_viewer/dist"};
    int max_clients{4};
    int image_fps{5};
    int image_jpeg_quality{55};
};

class WebViewerBackend {
public:
    WebViewerBackend(std::shared_ptr<VisualizationSource> source, const WebViewerConfig& config);
    ~WebViewerBackend();

    bool Start();
    void Stop();
    bool IsRunning() const { return mRunning.load(); }

private:
    void NetworkThreadLoop();
    void MirrorWorkerLoop();

    std::shared_ptr<VisualizationSource> mpSource;
    WebViewerConfig mConfig;
    std::atomic<bool> mRunning{false};
    std::thread mNetworkThread;
    std::thread mMirrorThread;

    // Backend Map Mirror
    struct MirrorMapPoint {
        std::uint64_t id;
        Eigen::Vector3f pos;
    };
    std::unordered_map<std::uint64_t, MirrorMapPoint> mMirrorPoints;
};

} // namespace ORB_SLAM3

#endif
```

- [ ] **Step 2: Implement `src/WebViewerBackend.cc` using Boost.Beast HTTP/WebSocket**

Implement HTTP request router (`/`, `/assets/*`, `/healthz`, `/config`) and WebSocket acceptor (`/ws/realtime` and `/ws/bulk`).
Drain `VisualizationSource` map events into backend `mMirrorPoints`, update encoded cache, and broadcast to WebSocket clients.

- [ ] **Step 3: Update `CMakeLists.txt` to link Boost.Asio, Boost.Beast, and OpenSSL**

Add Boost components and web backend files to `add_library(${PROJECT_NAME} ...)` in `CMakeLists.txt`.

- [ ] **Step 4: Test backend server launch in `nix develop`**

Write `tests/test_backend_server.cc` launching `WebViewerBackend` on port 8080 and making HTTP GET `/healthz` request.

Run: `nix develop --command bash -c "curl -s http://127.0.0.1:8080/healthz"`
Expected: `{"status":"ok","clients":0}`

- [ ] **Step 5: Commit Task 4**

```bash
git add include/WebViewerBackend.h src/WebViewerBackend.cc tests/test_backend_server.cc CMakeLists.txt
git commit -m "feat(backend): implement Boost.Beast web server and map mirror engine"
```

---

### Task 5: Frontend Three.js TypeScript Application (`web_viewer_frontend`)

**Files:**
- Create: `web_viewer/package.json`
- Create: `web_viewer/tsconfig.json`
- Create: `web_viewer/vite.config.ts`
- Create: `web_viewer/index.html`
- Create: `web_viewer/src/main.ts`
- Create: `web_viewer/src/protocol.ts`
- Create: `web_viewer/src/renderer.ts`
- Create: `web_viewer/src/overlay.ts`
- Create: `web_viewer/src/controls.ts`

**Interfaces:**
- Consumes: `/ws/realtime` and `/ws/bulk` WebSocket streams.
- Produces: Interactive WebGL 3D view using Three.js (camera frustums, sparse point cloud `THREE.Points`, trajectory lines, view controls) and 2D canvas feature overlay.

- [ ] **Step 1: Create `web_viewer/package.json` and `vite.config.ts`**

```json
{
  "name": "orbslam3-web-viewer",
  "private": true,
  "version": "1.0.0",
  "type": "module",
  "scripts": {
    "dev": "vite",
    "build": "tsc && vite build"
  },
  "dependencies": {
    "three": "^0.160.0"
  },
  "devDependencies": {
    "@types/three": "^0.160.0",
    "typescript": "^5.3.0",
    "vite": "^5.0.0"
  }
}
```

- [ ] **Step 2: Implement protocol parser `web_viewer/src/protocol.ts`**

Parse 48-byte `WireHeader` (using DataView little-endian getters) and unpack camera pose, point chunks, and JPEG images.

- [ ] **Step 3: Implement 3D Renderer `web_viewer/src/renderer.ts`**

Apply coordinate transform `(x, y, z)_three = (x, -y, -z)_orb`.
Use `THREE.BufferGeometry` and `THREE.Points` for sparse map points and `THREE.Line` for trajectory trail.

- [ ] **Step 4: Implement Camera Canvas Overlay `web_viewer/src/overlay.ts`**

Render camera JPEG image to background canvas and draw green/yellow tracked feature points on transparent 2D overlay canvas.

- [ ] **Step 5: Build static frontend assets inside nix develop**

Run: `nix develop --command bash -c "cd web_viewer && npm install && npm run build"`
Expected: Output static assets generated in `web_viewer/dist/`.

- [ ] **Step 6: Commit Task 5**

```bash
git add web_viewer/
git commit -m "feat(frontend): add Three.js TypeScript web visualizer application"
```

---

### Task 6: System Integration & Settings Configuration

**Files:**
- Modify: `include/Settings.h`, `src/Settings.cc`
- Modify: `include/System.h`, `src/System.cc`
- Create: `Examples/Monocular/mono_web_runner.cc`

**Interfaces:**
- Reads `WebViewer.*` section from `camera.yaml` settings.
- Initializes `VisualizationSource` and `WebViewerBackend` when `WebViewer.Enabled: 1`.

- [ ] **Step 1: Update `Settings` class to parse `WebViewer` settings**

In `src/Settings.cc`, parse `WebViewer.Enabled`, `WebViewer.Port`, `WebViewer.BindAddress`, `WebViewer.StaticRoot`.

- [ ] **Step 2: Update `System` constructor to start WebViewer**

In `src/System.cc`:
```cpp
if (mSettings && mSettings->webViewerEnabled()) {
    mpVisSource = std::make_shared<VisualizationSource>();
    mpWebBackend = std::make_unique<WebViewerBackend>(mpVisSource, mSettings->getWebViewerConfig());
    mpWebBackend->Start();
}
```

- [ ] **Step 3: Create `Examples/Monocular/mono_web_runner.cc`**

Runner executable that accepts vocabulary file, camera settings file, dataset directory, and processes dataset frames in real time.

- [ ] **Step 4: Build entire project inside nix develop**

Run: `nix develop --command ./build.sh`
Expected: `libORB_SLAM3.so` and `mono_web_runner` build cleanly with 0 errors.

- [ ] **Step 5: Commit Task 6**

```bash
git add include/Settings.h src/Settings.cc include/System.h src/System.cc Examples/Monocular/mono_web_runner.cc
git commit -m "feat(system): integrate WebViewer backend initialization into System and Settings"
```

---

### Task 7: Automated Web UI Verification & Performance Checks

**Files:**
- Create: `scripts/verify_web_ui.py`
- Output Artifact: `docs/superpowers/artifacts/ui_verification.png`

**Interfaces:**
- Launches `mono_web_runner` on dataset `dataset/circle_run`.
- Launches Playwright headless browser pointing at `http://127.0.0.1:8080`.
- Verifies WebGL canvas element, checks `/healthz` metrics, takes screenshot, and validates performance.

- [ ] **Step 1: Write `scripts/verify_web_ui.py`**

```python
#!/usr/bin/env python3
import subprocess
import time
import requests
from playwright.sync_api import sync_playwright

def main():
    print("1. Checking server healthz...")
    resp = requests.get("http://127.0.0.1:8080/healthz")
    print("Healthz response:", resp.json())
    assert resp.status_code == 200

    print("2. Launching Playwright browser verification...")
    with sync_playwright() as p:
        browser = p.chromium.launch(headless=True)
        page = browser.new_page()
        page.goto("http://127.0.0.1:8080")
        page.wait_for_selector("#webgl-canvas", timeout=10000)

        time.sleep(3) # Wait for initial pose / map data

        screenshot_path = "docs/superpowers/artifacts/ui_verification.png"
        page.screenshot(path=screenshot_path)
        print(f"UI verification screenshot saved to {screenshot_path}")

        browser.close()

if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Run end-to-end verification inside nix develop**

Run: `nix develop --command python3 scripts/verify_web_ui.py`
Expected: Healthz returns `{"status":"ok"}` and screenshot saved to `docs/superpowers/artifacts/ui_verification.png`.

- [ ] **Step 3: Final Commit and Goal Handoff**

```bash
git add scripts/verify_web_ui.py docs/superpowers/artifacts/ui_verification.png
git commit -m "test(verify): add automated Web UI Playwright verification test"
```
