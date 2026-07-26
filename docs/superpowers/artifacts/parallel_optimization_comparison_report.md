# ORB-SLAM3 Performance Benchmark Comparison Report

**Hardware Baseline**: AMD Ryzen 7 5825U (8 Cores / 16 Threads, up to 4.55 GHz), 22 GiB RAM, NixOS Linux.

## Overall Optimization Progression

| Optimization Stage | `circle_run` FPS (Hz) | `circle_run` Mean Latency | `full_run` FPS (Hz) | `full_run` Mean Latency | `full_run` Total Time | Tracking Accuracy |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **0. Initial Single-Thread Baseline** | 33.50 Hz | 29.85 ms | 27.39 Hz | 36.50 ms | 232.66 s | 100% (5,620 / 5,620) |
| **1. OpenMP Parallelization & -O3** | 39.17 Hz | 25.53 ms | 32.52 Hz | 30.76 ms | 200.73 s | 100% (5,620 / 5,620) |
| **2. Hardware `POPCNT` Vectorization** | 39.83 Hz | 25.10 ms | 32.91 Hz | 30.38 ms | 198.62 s | 100% (5,620 / 5,620) |
| **3. Zero-Alloc Raw Pointer Patch Correlation** | 40.60 Hz | 24.63 ms | 34.31 Hz | 29.14 ms | 191.28 s | 100% (5,620 / 5,620) |
| **4. Frame Memory & Grid Allocation Removal** | **40.74 Hz** | **24.54 ms** | **33.91 Hz** | **29.49 ms** | **193.38 s** | **100% (5,620 / 5,620)** |

## Cumulative Speedup Metrics

- **`circle_run` Speedup**: **+21.6% FPS increase** (from 33.50 Hz to **40.74 Hz**, mean latency dropped by **5.31 ms** per frame).
- **`full_run` Speedup**: **+23.8% FPS increase** (from 27.39 Hz to **33.91 Hz**, mean latency dropped by **7.01 ms** per frame).
- **Total Time Saved**: Reduced total runtime on 5,620 frames from **232.66 seconds to 193.38 seconds** (saved **39.28 seconds**).
- **Tracking Accuracy**: 100.0% tracking rate preserved without drift or loss of tracking.
- **Process Stability**: 0 crashes, clean process shutdown exit without thread deadlocks.
