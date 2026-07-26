# ORB-SLAM3 CPU Parallel Optimization Benchmark Report

## Executive Summary
By parallelizing the key bottlenecks in ORB-SLAM3 (`Frame::ComputeStereoMatches`, `ORBextractor::computeDescriptors`, `ORBextractor::computeOrientation`) using **OpenMP multi-threading** and enabling SIMD compilation flags (`-O3 -march=native -ffast-math -funroll-loops`), tracking throughput improved by up to **+18.7% in FPS** and reduced frame processing latency by **-15.7%** on the AMD Ryzen 7 5825U hardware across 8,337 total stereo IR frames.

---

## Hardware Environment
- **CPU**: AMD Ryzen 7 5825U (8 Cores / 16 Threads, up to 4.55 GHz)
- **RAM**: 22 GiB System Memory
- **OS**: NixOS (Linux x86_64)
- **Compiler**: GCC 15.3.0 (`-fopenmp -O3 -march=native -ffast-math`)

---

## Baseline vs. OpenMP Parallel Comparison Table

### 1. `circle_run` Dataset (2,717 Stereo IR Frames)
| Metric | Baseline | OpenMP Parallel | Delta / Improvement |
| :--- | :--- | :--- | :--- |
| **Average FPS** | 33.50 Hz | **39.17 Hz** | **+16.9% (+5.67 FPS)** |
| **Mean Frame Latency** | 29.85 ms | **25.53 ms** | **-14.5% (-4.32 ms)** |
| **Median ($P_{50}$) Latency** | 29.24 ms | **25.18 ms** | **-13.9% (-4.06 ms)** |
| **$P_{90}$ Latency** | 36.32 ms | **31.33 ms** | **-13.7% (-4.99 ms)** |
| **$P_{95}$ Latency** | 38.58 ms | **33.37 ms** | **-13.5% (-5.21 ms)** |
| **Total Sequence Duration**| 94.19 s | **82.74 s** | **Saved 11.45 s** |
| **Tracked Frames** | 2,717 / 2,717 (100%) | **2,717 / 2,717 (100%)** | 0% tracking loss |
| **Peak Memory (RSS)** | 982.6 MB | **951.4 MB** | -31.2 MB |

### 2. `full_run` Dataset (5,620 Stereo IR Frames)
| Metric | Baseline | OpenMP Parallel | Delta / Improvement |
| :--- | :--- | :--- | :--- |
| **Average FPS** | 27.39 Hz | **32.52 Hz** | **+18.7% (+5.13 FPS)** |
| **Mean Frame Latency** | 36.50 ms | **30.76 ms** | **-15.7% (-5.74 ms)** |
| **Median ($P_{50}$) Latency** | 35.52 ms | **30.43 ms** | **-14.3% (-5.09 ms)** |
| **$P_{90}$ Latency** | 46.41 ms | **38.55 ms** | **-16.9% (-7.86 ms)** |
| **$P_{95}$ Latency** | 49.96 ms | **41.42 ms** | **-17.1% (-8.54 ms)** |
| **Total Sequence Duration**| 232.66 s | **202.77 s** | **Saved 29.89 s** |
| **Tracked Frames** | 5,620 / 5,620 (100%) | **5,620 / 5,620 (100%)** | 0% tracking loss |
| **Peak Memory (RSS)** | 1,706.2 MB | **1,556.2 MB** | -150.0 MB |

---

## Code Optimizations Implemented

1. **Lock-Free Parallel Stereo Keypoint Matching (`src/Frame.cc`)**:
   ```cpp
   vector<pair<int, int>> vDistIdxTemp(N, pair<int,int>(-1, -1));
   #pragma omp parallel for schedule(dynamic, 16)
   for(int iL = 0; iL < N; iL++) {
       // Epipolar search & sliding window L1 correlation across 16 threads
   }
   ```
2. **Parallel ORB Descriptor Generation (`src/ORBextractor.cc`)**:
   ```cpp
   #pragma omp parallel for schedule(dynamic, 32)
   for (size_t i = 0; i < keypoints.size(); i++)
       computeOrbDescriptor(keypoints[i], image, &pattern[0], descriptors.ptr((int)i));
   ```
3. **Parallel Octave Orientation Calculation (`src/ORBextractor.cc`)**:
   ```cpp
   #pragma omp parallel for schedule(dynamic)
   for (int level = 0; level < nlevels; ++level)
       computeOrientation(mvImagePyramid[level], allKeypoints[level], umax);
   ```
