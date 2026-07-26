# ORB-SLAM3 Headless Performance Benchmark Suite Design

## 1. Overview
This design document defines the architecture, instrumentation, metric collection, and reporting pipeline for the **ORB-SLAM3 Headless Performance Benchmark Suite**. The goal is to establish a deterministic, microsecond-accurate baseline on the AMD Ryzen 7 5825U hardware across stereo datasets (`dataset/full_run` and `dataset/circle_run`), enabling isolated evaluation of future algorithmic and parallelization optimizations.

## 2. Architecture & Components

```
+---------------------------------------------------------------------------------+
|                    stereo_benchmark (Headless C++ Executable)                   |
|                                                                                 |
|  +---------------------+   +---------------------+   +-----------------------+  |
|  |     Stereo Input    |   |     ORB-SLAM3       |   |  Timing & Resource    |  |
|  |   Dataset Reader    |-->|   Tracking Engine   |-->|   Instrumentation     |  |
|  | (full_run/circle)   |   |   (Headless Mode)   |   |    Collector          |  |
|  +---------------------+   +---------------------+   +-----------------------+  |
+----------------------------------------------------------|----------------------+
                                                           v
                                         +----------------------------------+
                                         |      Benchmark Report & JSON     |
                                         |  - baseline_results.json         |
                                         |  - baseline_report.md            |
                                         +----------------------------------+
```

### 2.1 Headless Benchmark Executable (`Examples/Stereo/stereo_benchmark.cc`)
- Runs without any GUI or web visualization server overhead.
- Loads raw stereo IR images sequentially into RAM or streams directly from disk with cached file handles to eliminate disk I/O bottlenecks.
- Disables artificial frame playback throttling (`sleep_for`), allowing ORB-SLAM3 to run at max algorithmic speed.

### 2.2 Fine-Grained Timing Instrumentation (`include/System.h` & `src/System.cc`)
High-resolution `std::chrono::steady_clock` timers measure:
1. **ORB Feature Extraction Time ($t_{\text{ORB}}$)**: Time spent extracting 2,000 keypoints and computing descriptors on Left and Right IR images.
2. **Stereo Matching Time ($t_{\text{Stereo}}$)**: Sub-pixel epipolar matching and disparity calculation (`ComputeStereoMatches`).
3. **Pose Tracking Time ($t_{\text{Pose}}$)**: Motion model prediction, map point projection (`SearchLocalPoints`), and g2o pose optimization (`PoseOptimization`).
4. **Total Frame Latency ($t_{\text{Total}}$)**: End-to-end duration of `SLAM.TrackStereo()`.
5. **Local Mapping Latency ($t_{\text{LBA}}$)**: Duration of Local Bundle Adjustment runs in the `LocalMapping` thread.

## 3. Metrics & Statistical Analysis

The benchmark collector aggregates measurements across multiple evaluation runs (3 benchmark passes) to compute robust statistics:

| Metric | Description | Target Unit |
| :--- | :--- | :--- |
| **Mean Latency ($\mu$)** | Average frame processing time | milliseconds (ms) |
| **Standard Deviation ($\sigma$)** | Variance in frame execution latency | milliseconds (ms) |
| **Median Latency ($P_{50}$)** | 50th percentile frame latency | milliseconds (ms) |
| **Tail Latencies ($P_{90}, P_{95}$)** | 90th & 95th percentile worst-case latencies | milliseconds (ms) |
| **Min / Max Latency** | Best and worst single-frame latencies | milliseconds (ms) |
| **Tracking Throughput (FPS)** | $1000.0 / \mu$ | Frames per second (Hz) |
| **Peak Memory Footprint** | Peak Resident Set Size (RSS) | Megabytes (MB) |
| **KeyFrame Count** | Total KeyFrames created by LocalMapping | count |
| **MapPoint Count** | Active 3D MapPoints retained in Atlas | count |

## 4. Benchmark Artifact Output Files

1. `docs/superpowers/artifacts/baseline_benchmark_results.json`:
   - Contains raw per-frame timing arrays, system hardware details, timestamp, dataset info, and statistical summaries for machine consumption.
2. `docs/superpowers/artifacts/baseline_benchmark_report.md`:
   - Formatted markdown report featuring hardware summary, component latency tables, throughput comparisons, and baseline target benchmarks.

## 5. Verification & Validation Plan
- Verify that `stereo_benchmark` compiles inside `nix develop`.
- Execute benchmark on `dataset/circle_run` (2,717 frames) and `dataset/full_run` (5,620 frames).
- Validate that output JSON and Markdown artifacts are generated correctly with non-zero statistical metrics.
