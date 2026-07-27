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
| **5. High-Precision Stage Profiling (`REGISTER_TIMES`)** | 41.80 Hz | 23.92 ms | 35.01 Hz | 28.56 ms | 189.30 s | 100% (5,620 / 5,620) |
| **6. OpenMP Parallel Grid FAST & Local Map Matching** | 46.60 Hz | 21.46 ms | 37.68 Hz | 26.54 ms | 177.10 s | 100% (5,620 / 5,620) |
| **7. Lock-Free MapPoints, Zero-Alloc & Parallel Stereo Extraction** | 49.35 Hz | 20.26 ms | 39.99 Hz | 25.01 ms | 168.37 s | 100% (5,620 / 5,620) |
| **8. Fast PosInGrid & PoseOptimization Global Mutex Removal** | 50.76 Hz | 19.70 ms | 41.19 Hz | 24.28 ms | 164.20 s | 100% (5,620 / 5,620) |
| **9. 64-Bit Hardware POPCNT & Visualizer Update Guards** | **51.94 Hz** | **19.25 ms** | **43.77 Hz** | **22.85 ms** | **155.89 s** | **100% (5,620 / 5,620)** |

## Empirical Stage Timing Breakdown (`full_run` with 2,000 features fixed)

| Sub-System / Function Stage | Execution Time (ms) | % Total Frame Time | Optimizations Applied |
| :--- | :---: | :---: | :--- |
| **1. ORB Feature Extraction** (`Frame::ExtractORB`) | **6.21 ms** (down from 9.12 ms) | **27.2 %** | Concurrent stereo `std::thread`, `cv::Rect` submatrix ROI FAST detection, 64-bit POPCNT. |
| **2. Local Map Projection & BA** (`TrackLocalMap`) | **5.49 ms** (down from 7.23 ms) | **24.0 %** | Lock-free atomic visibility counters, zero-alloc observation callbacks, pose optimization lock removal. |
| **3. Motion Model Prediction** (`TrackWithMotionModel`) | **3.14 ms** | **13.7 %** | Camera velocity prediction and frame-to-frame feature projection matching. |
| **4. Epipolar Stereo Matching** (`ComputeStereoMatches`) | **1.04 ms** | **4.6 %** | Sub-pixel stereo keypoint matching (raw-pointer patch correlation & 64-bit hardware `POPCNT`). |
| **5. System / Image Frame Overhead** | **6.97 ms** | **30.5 %** | Guarded visualizer updates (`mpViewer`), fast float-to-int grid indexing. |
| **Total Frame Latency** | **22.85 ms** | **100.0 %** | **43.77 Hz (FPS)** mean throughput on `full_run` (`circle_run` reached **51.94 Hz** mean / **54.11 Hz** $P_{50}$) |
