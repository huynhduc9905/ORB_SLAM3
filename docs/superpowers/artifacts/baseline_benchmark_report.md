# ORB-SLAM3 Baseline Performance Benchmark Report

## System Hardware Specifications
- **CPU**: AMD Ryzen 7 5825U (8 Cores / 16 Threads, up to 4.55 GHz)
- **RAM**: 22 GiB System Memory
- **OS**: NixOS (Linux x86_64)
- **Compiler**: GCC 15.3.0 (Nix Environment)

## Baseline Metrics

| Dataset | Total Frames | Tracked | Elapsed (s) | Mean Latency (ms) | StdDev (ms) | Median P50 (ms) | P90 (ms) | P95 (ms) | **Avg Tracking Frequency** | Peak Memory (MB) |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| `circle_run` | 2717 | 2717 | 65.56 s | 19.25 ms | 6.00 ms | 18.48 ms | 23.69 ms | 25.46 ms | **51.94 Hz (FPS)** | 992.8 MB |
| `full_run` | 5620 | 5620 | 155.89 s | 22.85 ms | 6.81 ms | 22.21 ms | 28.60 ms | 31.01 ms | **43.77 Hz (FPS)** | 1656.0 MB |


## Sub-component Optimization Targets
1. **ORB Feature Extraction**: Parallelize multi-threaded pyramid level extraction across CPU worker threads.
2. **Stereo Descriptor Matching**: Vectorize epipolar search using AVX2 SIMD intrinsics.
3. **Local BA Optimization**: Streamline Hessian structure updates during keyframe culling.
