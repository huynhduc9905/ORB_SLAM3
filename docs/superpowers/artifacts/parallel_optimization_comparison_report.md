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
| **7. Lock-Free MapPoints, Zero-Alloc & Parallel Stereo Extraction** | **49.35 Hz** | **20.26 ms** | **39.99 Hz** | **25.01 ms** | **168.37 s** | **100% (5,620 / 5,620)** |

## Empirical Stage Timing Breakdown (`full_run` with 2,000 features fixed)

| Sub-System / Function Stage | Execution Time (ms) | % Total Frame Time | Optimizations Applied |
| :--- | :---: | :---: | :--- |
| **1. ORB Feature Extraction** (`Frame::ExtractORB`) | **6.35 ms** (down from 9.12 ms) | **25.4 %** | Parallel grid cell FAST corner detection & concurrent stereo image extraction via `std::thread`. |
| **2. Local Map Projection & BA** (`TrackLocalMap`) | **6.04 ms** (down from 7.23 ms) | **24.2 %** | Lock-free atomic visibility counters (`std::atomic<int>`), zero-alloc observations, OpenMP projection matching. |
| **3. Motion Model Prediction** (`TrackWithMotionModel`) | **3.35 ms** | **13.4 %** | Camera velocity prediction and frame-to-frame feature projection matching. |
| **4. Epipolar Stereo Matching** (`ComputeStereoMatches`) | **1.06 ms** | **4.2 %** | Sub-pixel stereo keypoint matching (raw-pointer patch correlation & hardware `POPCNT`). |
| **5. System / Image Frame Overhead** | **8.21 ms** | **32.8 %** | Image decoding, matrix initialization, grid mapping. |
| **Total Frame Latency** | **25.01 ms** | **100.0 %** | **39.99 Hz (FPS) throughput** (`circle_run` reached **49.35 Hz** mean / **51.02 Hz** $P_{50}$) |
