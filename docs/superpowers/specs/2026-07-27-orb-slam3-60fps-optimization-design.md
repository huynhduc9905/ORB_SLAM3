# ORB-SLAM3 60 FPS Performance Optimization Design

## Executive Summary
This document outlines the performance optimization design for ORB-SLAM3 on the `feature/web-visualizer` branch. The objective is to achieve stable **60 FPS** (mean per-frame tracking latency $\le 16.66\text{ ms}$) across benchmark datasets (`circle_run` and `full_run`) on the target environment.

---

## Baseline Performance & Targets

### Hardware Specifications
- **CPU**: AMD Ryzen 7 5825U (8 Cores / 16 Threads)
- **RAM**: 22 GiB
- **OS**: NixOS / Linux x86_64 (Nix Flake gcc 15.3.0 environment)

### Baseline Benchmarks (Measured)
| Dataset | Total Frames | Baseline Latency | Baseline FPS | Stage Breakdown (ORB / MapTrack / PosePred / Stereo) | Target Latency | Target FPS |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| `circle_run` | 2717 | 19.66 ms | 50.85 Hz | 6.46ms / 5.01ms / 2.67ms / 1.08ms | $\le 16.66\text{ ms}$ | $\ge 60.0\text{ Hz}$ |
| `full_run` | 5620 | 26.90 ms | 37.18 Hz | 7.83ms / 7.13ms / 3.04ms / 1.32ms | $\le 16.66\text{ ms}$ | $\ge 60.0\text{ Hz}$ |

To reach $\ge 60\text{ FPS}$, latency reductions of **~3.0 ms** for `circle_run` and **~10.2 ms** for `full_run` are required.

---

## System Architecture & Optimization Plan

### 1. Multi-Threaded Parallel ORB Feature Extractor
- **Location**: `src/ORBextractor.cc` (`ORBextractor::operator()`)
- **Problem**: Pyramid construction (`cv::GaussianBlur`), keypoint extraction (`ComputeKeyPointsOctTree`), and descriptor generation currently run sequentially across levels or with unoptimized level loops.
- **Design Solution**:
  - OpenMP parallelization over pyramid levels: `#pragma omp parallel for` across all levels ($0 \dots \text{nlevels}-1$).
  - Pre-allocate work buffers per pyramid level to eliminate intra-loop memory allocation contention.

### 2. Fast OctTree Memory Arena (`DistributeOctTree`)
- **Location**: `src/ORBextractor.cc` (`DistributeOctTree`)
- **Problem**: `DistributeOctTree` repeatedly performs dynamic heap allocations using `std::list<ExtractorNode>`, calls `push_back`, and erases nodes dynamically. For 8 pyramid levels per image (x2 for stereo), this causes massive memory fragmentation and heap churn.
- **Design Solution**:
  - Implement a thread-local static node pool / contiguous vector arena (`std::vector<ExtractorNode> nodeArena`).
  - Use index-based pointer tracking or reused vector slots instead of dynamic heap allocation and node destruction per frame.

### 3. Fast Contiguous Descriptor Buffer Memory Copy
- **Location**: `src/ORBextractor.cc` (`ComputeDescriptors`)
- **Problem**: Descriptors are copied row-by-row into `cv::Mat` output using `desc.row(i).copyTo(...)` which incurs OpenCV function call overhead per keypoint.
- **Design Solution**:
  - Direct contiguous block memory copy `std::memcpy(descriptors.ptr<uchar>(i), desc.ptr<uchar>(i), 32)` or direct writing into raw output buffer.

### 4. Vectorized Hamming Distance SIMD Acceleration
- **Location**: `src/ORBmatcher.cc` (`DescriptorDistance`)
- **Problem**: `DescriptorDistance` computes 256-bit Hamming distance using 32-bit `__builtin_popcount` loops.
- **Design Solution**:
  - Re-implement `DescriptorDistance` using 64-bit `__builtin_popcountll` on `uint64_t*` pointers (reducing loop count from 8 iterations to 4 per descriptor pair), or AVX2 `_mm256_popcnt` when available.

---

## Verification & Benchmarking Contract

1. **Compilation**: Clean build using Nix environment (`nix develop --command bash -c "cd build && make -j4"`).
2. **Benchmark Execution**: Run `python3 scripts/run_baseline_benchmark.py` across `circle_run` and `full_run`.
3. **Accuracy Verification**: Confirm 100% frame tracking without lost tracking or regression.
4. **Performance Verification**:
   - `circle_run` avg FPS $\ge 60.0$ (mean latency $\le 16.66\text{ ms}$).
   - `full_run` avg FPS $\ge 60.0$ (mean latency $\le 16.66\text{ ms}$).
