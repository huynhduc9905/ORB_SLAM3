# ORB-SLAM3 Baseline Performance Benchmark Report

## System Hardware Specifications
- **CPU**: AMD Ryzen 7 5825U (8 Cores / 16 Threads, up to 4.55 GHz)
- **RAM**: 22 GiB System Memory
- **OS**: NixOS (Linux x86_64)
- **Compiler**: GCC 15.3.0 (Nix Environment)

## Baseline Metrics

| Dataset | Total Frames | Tracked | Elapsed (s) | Mean Latency (ms) | StdDev (ms) | Median P50 (ms) | P90 (ms) | P95 (ms) | **Avg Tracking Frequency** | Peak Memory (MB) |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| `circle_run` | 2717 | 2717 | 67.99 s | 20.05 ms | 7.80 ms | 18.98 ms | 25.31 ms | 27.71 ms | **49.88 Hz (FPS)** | 1053.5 MB |
| `full_run` | 5620 | 5620 | 178.37 s | 26.67 ms | 11.07 ms | 25.07 ms | 35.91 ms | 40.72 ms | **37.50 Hz (FPS)** | 1267.3 MB |


## Sub-component Optimization Targets
1. **ORB Feature Extraction**: Parallelize multi-threaded pyramid level extraction across CPU worker threads.
2. **Stereo Descriptor Matching**: Vectorize epipolar search using AVX2 SIMD intrinsics.
3. **Local BA Optimization**: Streamline Hessian structure updates during keyframe culling.
