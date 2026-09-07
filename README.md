# ORB-SLAM3 (Hardened & Extended Fork)

A production-grade, hardened fork of **ORB-SLAM3 V1.0** featuring multi-session map persistence, pure relocalization, robust cross-map place recognition, memory and concurrency hardening, Three.js web visualization, and hermetic Nix flake integration.

---

## Current Status

- **Branch**: `master` (synchronized with PR #7 core stability patches and PR #8 map relocalization & extension).
- **Test Suite**: 100% pass rate on `orb_slam3_tests` (21/21 GoogleTest cases passing via CTest).
- **Build Toolchain**: Fully reproducible via Nix Flake (`flake.nix`), supporting modern GCC, CMake 4.x policy shims, OpenCV 4 with GTK3, and Pangolin 0.9.1.
- **Tested Hardware & Sensors**: Stereo (RealSense D435i), Monocular, Monocular-Inertial, Stereo-Inertial, and RGB-D.

---

## Key Fork Improvements

This fork retains upstream ORB-SLAM3 algorithmic accuracy while significantly expanding system robustness, embedding safety, and deployment workflows:

### 1. Multi-Session Mapping & Map Extension
- **Bidirectional Atlas Persistence**: Save (`--save-map <path>`) and load (`--load-map <path>`) multi-map Atlas files (`.osa`).
- **PostLoad Active Map Resolution**: Automatically binds and activates the most relevant map upon loading, allowing continuous mapping and keyframe expansion across sessions.
- **Seamless Extension**: Start a session with a pre-existing map, accumulate new keyframes and map points in newly explored areas, and serialize the extended Atlas back to disk.
- **Path Normalization & Overrides**: Automatic file extension normalization (`.osa` appended if omitted) and flexible environment variable overrides (`ORB_SLAM3_SAVE_ATLAS`, `ORB_SLAM3_LOAD_ATLAS`).

### 2. Pure Relocalization Mode
- **Zero Map Mutation**: Run tracking in localization-only mode (`--localize-only` or `ORB_SLAM3_LOCALIZE_ONLY=1`), disabling `LocalMapping` and freezing map points/keyframes for zero memory drift during repetitive routes.
- **Cross-Map Relocalization Fallback**: If the active map yields no relocalization candidates in a multi-map Atlas, the query automatically falls back across all inactive maps in the Atlas database.
- **Safe Dynamic Map Adoption**: When a match is found in an inactive map, the tracker synchronizes map pointers (`pCurrentMap`), safely transfers map update locks (`mMutexMapUpdate`), and adopts the target map seamlessly without race conditions or dropped frames.
- **Continuity Preservation**: Tracking retains state, velocity, and motion models upon successful relocalization in mapping mode instead of aborting the frame pipeline.

### 3. Concurrency, Memory & Lifecycle Hardening
- **Safe Thread Shutdown**: Background threads (`LocalMapping`, `LoopClosing`) employ timed joins with leak-on-deadlock safety guards, ensuring `System::Shutdown()` never indefinitely blocks host processes.
- **Thread-Safe KeyFrameDatabase**: Added mutex protection and caching for `queryScores` during candidate detection, eliminating data races and uninitialized score reads.
- **Bounded Tombstones**: Culled `KeyFrame` snapshot tombstones in `Map` are constrained to a strict FIFO bounded queue (max 1,000) to prevent unbounded memory growth during long-running sessions.
- **Atlas & Memory Cleanup**: Destructor `Atlas::~Atlas()` systematically releases bad maps, `Atlas::PreSave()` purges stale backup map lists, and raw pointers in `KeyFrame` constructors are strictly zero-initialized.
- **Algorithmic Bounds & Null Safety**:
  - Null pointer and culled keyframe guards added in `OptimizeEssentialGraph` and `CorrectLoop`.
  - Observation descriptor bounds checking in `ComputeDistinctiveDescriptors` and stereo `rightIndex`.
  - Removed unsynchronized `ChangeMap` invocations during localization mode switches.

### 4. Safe Value-Based Snapshots
- **No Leaked Pointers**: `System::GetLastFrameSnapshot()` and `System::GetGraphSnapshot()` expose frame tracking state and keyframe-graph topology without leaking raw internal ORB-SLAM3 object pointers.
- **Consistent Graph View**: Snapshots provide map identity, monotonic revision counters, keyframe poses, parent/loop edges, and culled tombstones so external ROS/web bridges retain a consistent graph view.
- **Multi-Lap Revisits**: Covisibility-connected loop candidates separated by at least 20 keyframe IDs remain eligible for geometric verification, enabling revisit loop closures on subsequent laps while filtering immediate neighbors.

### 5. Dual Visualization: Modern Web UI & Pangolin
- **Three.js Web Visualizer**: Integrated WebSocket backend streaming camera trajectory, keyframe frustums, and point clouds directly to a browser UI (`web_viewer/`).
- **Native Pangolin**: Maintained and isolated (Pangolin 0.9.1 via `nixpkgs-pango`) with real-time pacing, window lifecycle cleanup, and headless execution support.

---

## Quickstart with Nix (Recommended)

The easiest and most reproducible way to build and run the project is using the included Nix Flake:

### 1. Enter the Nix Development Shell
```bash
nix develop
```

This sets up a complete environment with GCC, CMake, OpenCV 4 (GTK3 enabled), Eigen3, Boost, OpenSSL, GTest, Node.js, and Pangolin.

### 2. Prepare Vocabulary and Build Thirdparty
```bash
# Extract pre-trained ORB vocabulary (gitignored)
tar -xf Vocabulary/ORBvoc.txt.tar.gz -C Vocabulary/

# Build Thirdparty dependencies (DBoW2, g2o, Sophus) and ORB-SLAM3
./build.sh
```

Alternatively, if Thirdparty libraries are already compiled:
```bash
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc) ORB_SLAM3 stereo_benchmark orb_slam3_tests
cd ..
```

### 3. Run the Automated Test Suite
```bash
ctest --test-dir build --output-on-failure
# Or execute GoogleTest suite directly (21 test cases across 5 suites):
./build/tests/orb_slam3_tests
```

---

## CLI Usage & Examples

The unified `stereo_benchmark` binary supports standard dataset execution, live visualization, map saving/loading, and relocalization.

> **Note**: Compiled runner executables reside in their respective `Examples/<Sensor>/` directories (e.g., `Examples/Stereo/stereo_benchmark`).

### Positional Syntax
```bash
./Examples/Stereo/stereo_benchmark \
  <path_to_vocabulary> \
  <path_to_settings> \
  <path_to_dataset> \
  [output_json] \
  [crash_report_dir] \
  [run_label] \
  [OPTIONS]
```

### Common Workflows

#### 1. Mapping and Saving Atlas
Build a new map from a dataset sequence and save the resulting Atlas:
```bash
./Examples/Stereo/stereo_benchmark \
  Vocabulary/ORBvoc.txt \
  Examples/Stereo/RealSense_D435i.yaml \
  dataset/session_1 \
  benchmark_run1.json /tmp/crashes run1 \
  --save-map /path/to/maps/area_map.osa \
  --viewer
```

#### 2. Extending an Existing Map
Load a previously saved map, continue mapping a new trajectory, and save the expanded map:
```bash
./Examples/Stereo/stereo_benchmark \
  Vocabulary/ORBvoc.txt \
  Examples/Stereo/RealSense_D435i.yaml \
  dataset/session_2 \
  benchmark_run2.json /tmp/crashes run2 \
  --load-map /path/to/maps/area_map.osa \
  --save-map /path/to/maps/extended_area_map.osa \
  --viewer
```

#### 3. Pure Relocalization Mode
Localize against a pre-built map without modifying or adding new map points:
```bash
./Examples/Stereo/stereo_benchmark \
  Vocabulary/ORBvoc.txt \
  Examples/Stereo/RealSense_D435i.yaml \
  dataset/reloc_test \
  benchmark_reloc.json /tmp/crashes reloc \
  --load-map /path/to/maps/extended_area_map.osa \
  --localize-only \
  --viewer
```

---

## CLI Flags & Environment Variables

| CLI Flag | Environment Variable | Description |
| :--- | :--- | :--- |
| `--save-map <path>` or `--save-map=<path>` | `ORB_SLAM3_SAVE_ATLAS` | Path to save the serialized Atlas (`.osa`) upon completion. |
| `--load-map <path>` or `--load-map=<path>` | `ORB_SLAM3_LOAD_ATLAS` | Path to an existing Atlas file (`.osa`) to load before processing. |
| `--localize-only` or `--loc-only` | `ORB_SLAM3_LOCALIZE_ONLY=1` or `ORB_SLAM3_LOCALIZATION_MODE=1` | Disables map building (`LocalMapping`); performs pure relocalization and tracking. |
| `--viewer` or `--pangolin` | `VIEWER=1` or `USE_PANGOLIN=1` | Enables the native Pangolin 3D viewer window. |

---

## Web Visualizer

A lightweight, modern web visualizer built on Vite and Three.js is provided in `web_viewer/`.

### 1. Launch C++ Backend Runner
```bash
./Examples/Stereo/stereo_web_runner \
  Vocabulary/ORBvoc.txt \
  Examples/Stereo/RealSense_D435i.yaml \
  path/to/sequence
```
*(The backend defaults to WebSocket port `8080`, or port `8085` if configured in the YAML settings via `WebViewer.Port: 8085`).*

### 2. Launch Web Frontend
```bash
cd web_viewer
npm install
npm run dev
```
Open `http://localhost:3000` in any modern browser to view the 3D trajectory and map point cloud in real time.

---

## Repository Structure

```text
ORB_SLAM3/
├── include/              # Public headers (System.h, Atlas.h, Tracking.h, etc.)
├── src/                  # Core SLAM implementation & stability hardening
├── Examples/             # Runners for Monocular, Stereo, RGB-D, and Inertial
│   └── Stereo/           # stereo_benchmark, stereo_web_runner, settings
├── web_viewer/           # Vite + Three.js browser visualizer
├── tests/                # GoogleTest suite (Atlas lifecycle, crash monitor, web mirror)
├── flake.nix             # Hermetic Nix development environment
└── CMakeLists.txt        # Top-level CMake build configuration
```

---

## License

This fork retains the original GNU General Public License v3.0 (GPLv3) from ORB-SLAM3. See [LICENSE](LICENSE) for details.
