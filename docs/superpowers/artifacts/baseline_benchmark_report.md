# ORB-SLAM3 Baseline Performance Benchmark Report

## System Hardware Specifications
- **CPU**: AMD Ryzen 7 5825U (8 Cores / 16 Threads, up to 4.55 GHz)
- **RAM**: 22 GiB System Memory
- **OS**: NixOS (Linux x86_64)
- **Compiler**: GCC 15.3.0 (Nix Environment)

## Baseline Metrics

| Dataset | Total Frames | Tracked | Elapsed (s) | Mean Latency (ms) | StdDev (ms) | Median P50 (ms) | P90 (ms) | P95 (ms) | **Avg Tracking Frequency** | Peak Memory (MB) |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| `circle_run` | 2717 | 2717 | 68.27 s | 20.26 ms | 5.76 ms | 19.60 ms | 24.57 ms | 26.26 ms | **49.35 Hz (FPS)** | 999.1 MB |
| `full_run` | 5620 | 5620 | 168.37 s | 25.01 ms | 7.24 ms | 24.55 ms | 31.23 ms | 33.63 ms | **39.99 Hz (FPS)** | 1329.5 MB |


## Sub-component Optimization Targets
1. **ORB Feature Extraction**: Parallelize multi-threaded pyramid level extraction across CPU worker threads.
2. **Stereo Descriptor Matching**: Vectorize epipolar search using AVX2 SIMD intrinsics.
3. **Local BA Optimization**: Streamline Hessian structure updates during keyframe culling.
