# ORB-SLAM3 Baseline Performance Benchmark Report

## System Hardware Specifications
- **CPU**: AMD Ryzen 7 5825U (8 Cores / 16 Threads, up to 4.55 GHz)
- **RAM**: 22 GiB System Memory
- **OS**: NixOS (Linux x86_64)
- **Compiler**: GCC 15.3.0 (Nix Environment)

## Baseline Metrics

| Dataset | Total Frames | Tracked | Elapsed (s) | Mean Latency (ms) | StdDev (ms) | Median P50 (ms) | P90 (ms) | P95 (ms) | **Avg Tracking Frequency** | Peak Memory (MB) |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| `circle_run` | 2717 | 2717 | 82.51 s | 24.41 ms | 6.73 ms | 23.74 ms | 28.02 ms | 30.12 ms | **40.96 Hz (FPS)** | 1028.6 MB |
| `full_run` | 5620 | 5620 | 198.78 s | 28.39 ms | 5.16 ms | 28.23 ms | 33.72 ms | 35.65 ms | **35.22 Hz (FPS)** | 1173.1 MB |


## Sub-component Optimization Targets
1. **ORB Feature Extraction**: Parallelize multi-threaded pyramid level extraction across CPU worker threads.
2. **Stereo Descriptor Matching**: Vectorize epipolar search using AVX2 SIMD intrinsics.
3. **Local BA Optimization**: Streamline Hessian structure updates during keyframe culling.
