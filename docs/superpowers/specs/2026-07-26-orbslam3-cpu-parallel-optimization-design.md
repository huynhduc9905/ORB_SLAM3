# ORB-SLAM3 CPU Parallel Power Optimization Design

## 1. Overview
This document specifies the design for fully utilizing all 16 parallel CPU threads on the AMD Ryzen 7 5825U architecture. By introducing OpenMP multi-threading and SIMD vectorization to `Frame::ComputeStereoMatches()`, `ORBextractor::operator()`, and `ORBmatcher::SearchByProjection()`, ORB-SLAM3 tracking throughput will be significantly accelerated without compromising tracking accuracy.

## 2. Targeted Optimization Modules

### 2.1 OpenMP Build Configuration (`CMakeLists.txt`)
- Enable `-fopenmp` for C++ compilation.
- Add compiler optimization flags: `-O3 -march=native -ffast-math -funroll-loops`.

### 2.2 Parallel Stereo Keypoint Matching (`src/Frame.cc`)
- **Location**: `Frame::ComputeStereoMatches()`
- **Implementation**: Wrap the `for(int iL=0; iL<N; iL++)` loop with `#pragma omp parallel for schedule(dynamic, 16)`.
- **Thread Safety**: Each thread operates on an isolated keypoint index `iL` and writes to `mvuRight[iL]` and `mvDepth[iL]`, ensuring lock-free parallel execution.

### 2.3 Parallel ORB Feature Extraction (`src/ORBextractor.cc`)
- **Location**: `ORBextractor::ComputeKeyPointsOctTree()` and `ORBextractor::operator()`
- **Implementation**: Parallelize octave level FAST corner detection, cell node splitting, and descriptor computation across OpenMP worker threads.

### 2.4 Parallel Map Point Projection Search (`src/ORBmatcher.cc`)
- **Location**: `ORBmatcher::SearchByProjection()`
- **Implementation**: Parallelize loop over candidate map points with `#pragma omp parallel for schedule(dynamic, 16)`.

## 3. Baseline vs. Optimized Comparison Framework
- The automated benchmark runner `scripts/run_baseline_benchmark.py` will execute across `dataset/circle_run` and `dataset/full_run`.
- A comparison report (`docs/superpowers/artifacts/parallel_optimization_comparison_report.md`) will evaluate:
  - Baseline FPS vs. Parallel FPS
  - Mean latency reduction (%)
  - $P_{50}, P_{90}, P_{95}$ latency improvements
  - Total frame processing duration reduction (seconds saved)

## 4. Verification Plan
- Build inside `nix develop`.
- Execute `scripts/run_baseline_benchmark.py`.
- Assert 100% tracked frames (0% tracking loss) and FPS speedup across both datasets.
