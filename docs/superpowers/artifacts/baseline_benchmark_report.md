# ORB-SLAM3 Baseline Performance Benchmark Report

## System Hardware Specifications
- **CPU**: AMD Ryzen 7 5825U (8 Cores / 16 Threads, up to 4.55 GHz)
- **RAM**: 22 GiB System Memory
- **OS**: NixOS (Linux x86_64)
- **Compiler**: GCC 15.3.0 (Nix Environment)

## Baseline Metrics

| Dataset | Total Frames | Tracked | Elapsed (s) | Mean Latency (ms) | StdDev (ms) | Median P50 (ms) | P90 (ms) | P95 (ms) | **Avg Tracking Frequency** | Peak Memory (MB) |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| `full_run` | 5620 | 5620 | 152.65 s | 21.25 ms | 8.58 ms | 20.30 ms | 25.55 ms | 27.54 ms | **47.06 Hz (FPS)** | 1606.5 MB |


## Sub-component Optimization Targets
1. **ORB Feature Extraction**: Parallelize multi-threaded pyramid level extraction across CPU worker threads.
2. **Stereo Descriptor Matching**: Vectorize epipolar search using AVX2 SIMD intrinsics.
3. **Local BA Optimization**: Streamline Hessian structure updates during keyframe culling.
