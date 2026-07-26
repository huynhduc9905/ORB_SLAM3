# ORB-SLAM3 Performance Benchmark & Stage Profiling Report

**Hardware Baseline**: AMD Ryzen 7 5825U (8 Cores / 16 Threads, up to 4.55 GHz), 22 GiB RAM, NixOS Linux.

## Overall Optimization Progression

| Optimization Stage | `circle_run` FPS (Hz) | `circle_run` Mean Latency | `full_run` FPS (Hz) | `full_run` Mean Latency | `full_run` Total Time | Tracking Accuracy |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **0. Initial Single-Thread Baseline** | 33.50 Hz | 29.85 ms | 27.39 Hz | 36.50 ms | 232.66 s | 100% (5,620 / 5,620) |
| **1. OpenMP Parallelization & -O3** | 39.17 Hz | 25.53 ms | 32.52 Hz | 30.76 ms | 200.73 s | 100% (5,620 / 5,620) |
| **2. Hardware `POPCNT` Vectorization** | 39.83 Hz | 25.10 ms | 32.91 Hz | 30.38 ms | 198.62 s | 100% (5,620 / 5,620) |
| **3. Zero-Alloc Raw Pointer Patch Correlation** | 40.60 Hz | 24.63 ms | 34.31 Hz | 29.14 ms | 191.28 s | 100% (5,620 / 5,620) |
| **4. Zero-Clone Frame & Grid Allocation** | 40.74 Hz | 24.54 ms | 33.91 Hz | 29.49 ms | 193.38 s | 100% (5,620 / 5,620) |
| **5. High-Precision Stage Profiling (`REGISTER_TIMES`)** | **41.80 Hz** | **23.92 ms** | **35.01 Hz** | **28.56 ms** | **189.30 s** | **100% (5,620 / 5,620)** |

## Empirical Per-Function Stage Timing Breakdown (`full_run`)

| Sub-System / Function Stage | Execution Time (ms) | % Total Frame Time | Function Responsibilities & Notes |
| :--- | :---: | :---: | :--- |
| **1. ORB Feature Extraction** (`Frame::ExtractORB`) | **9.12 ms** | **31.9 %** | Multi-octave image pyramid construction, FAST corner detection grid, descriptor generation across left & right stereo images. |
| **2. Local Map Projection & BA** (`TrackLocalMap`) | **7.23 ms** | **25.3 %** | Local map point search (`SearchByProjection`), G2O non-linear pose optimization (`Optimizer::PoseOptimization`). |
| **3. Motion Model Prediction** (`TrackWithMotionModel`) | **3.35 ms** | **11.7 %** | Predict camera velocity, map point matching against prior frame. |
| **4. Epipolar Stereo Matching** (`ComputeStereoMatches`) | **1.06 ms** | **3.7 %** | Sub-pixel stereo keypoint matching (drastically reduced from >10 ms via raw-pointer patch correlation). |
| **5. System / Image Frame Overhead** | **7.80 ms** | **27.4 %** | Image decoding, matrix initialization, grid mapping. |
| **Total Frame Latency** | **28.56 ms** | **100.0 %** | **35.01 Hz (FPS) throughput** |

## Key Insights & Next Optimization Targets
1. **Primary Bottleneck**: ORB Feature Extraction consumes **9.12 ms** (31.9%) per frame. Optimizing FAST corner threshold checks or feature count tuning (e.g. 1,500 features instead of 2,000) will yield direct FPS gains toward 60 Hz.
2. **Secondary Bottleneck**: Local Map Projection & G2O BA consumes **7.23 ms** (25.3%). Multi-threading MapPoint projection searches across map points will further accelerate local tracking.
3. **Epipolar Stereo Match Optimization**: Stereo matching latency dropped to only **1.06 ms** (3.7% of frame time).
