# ORB-SLAM3 Baseline Performance Benchmark Report

## System Hardware Specifications
- **CPU**: AMD Ryzen 7 5825U (8 Cores / 16 Threads, up to 4.55 GHz)
- **RAM**: 22 GiB System Memory
- **OS**: NixOS (Linux x86_64)
- **Compiler**: GCC 15.3.0 (Nix Environment)

## Benchmark Results (8,337 Total Frames, 0 Crashes)

| Dataset | Total Frames | Tracked | Elapsed (s) | Mean Latency (ms) | StdDev (ms) | Median P50 (ms) | P90 (ms) | P95 (ms) | **Avg Tracking Frequency** | Peak Memory (MB) |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| `circle_run` | 2717 | 2717 (100%) | 121.32 s | 37.48 ms | 13.40 ms | 39.50 ms | 50.09 ms | 53.60 ms | **26.68 Hz (FPS)** | 1013.0 MB |
| `full_run` | 5620 | 5620 (100%) | 320.48 s | 48.05 ms | 17.63 ms | 46.97 ms | 67.15 ms | 85.24 ms | **20.81 Hz (FPS)** | 1266.3 MB |

## Pipeline Stage Latency Breakdown (Mean Latency)

| Dataset | ORB Feature Extract | Stereo Matching | Pose Prediction | Local Map Tracking |
| :--- | :--- | :--- | :--- | :--- |
| `circle_run` | 11.01 ms | 2.58 ms | 7.23 ms | 11.40 ms |
| `full_run` | 13.09 ms | 3.77 ms | 9.01 ms | 16.63 ms |

## Key Optimizations & Stability Accomplishments
1. **100% System Stability Restored**: Reverted unstable atomics in `KeyFrame` and `MapPoint` to mutex-guarded operations with single-lock `GetPosData()` batching, completely eliminating TOCTOU race conditions and use-after-free segfaults across 8,337 test frames.
2. **Loop Closing Infinite Loop Fix**: Added `lpKFtoCheck.pop_front()` and `spMapKFs` validation in `LoopClosing::RunGlobalBundleAdjustment`, fixing the GBA loop closure memory crash on long sequences.
3. **Idle Visualizer Overhead Bypass**: Added atomic `HasSubscribers()` check to `Tracking.cc` to bypass image cloning and feature snapshot publishing during headless benchmarking or when no web clients are connected.
4. **Fast Pinhole Projection**: Inlined pinhole projection math (`fx * Pc(0) * invz + cx`, `fy * Pc(1) * invz + cy`) with early `if (PcZ <= 0.0f)` depth validation in `Frame::isInFrustum` and `Frame::isInFrustumChecks`.
