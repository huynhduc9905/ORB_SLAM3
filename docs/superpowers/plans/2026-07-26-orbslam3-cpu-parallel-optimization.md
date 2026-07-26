# ORB-SLAM3 CPU Parallel Optimization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Parallelize ORB-SLAM3 stereo matching, ORB feature extraction, and map point projection using OpenMP and SIMD optimizations to fully utilize all 16 CPU threads on the Ryzen 7 5825U hardware and achieve maximum tracking throughput.

**Architecture:** Integrate OpenMP `#pragma omp parallel for` across keypoint matching loops (`Frame::ComputeStereoMatches`), octave descriptor computations (`ORBextractor::ComputeKeyPointsOctTree`), and projection searches (`ORBmatcher::SearchByProjection`), compiled with `-fopenmp -O3 -march=native -ffast-math`.

**Tech Stack:** C++14, OpenMP, CMake, OpenCV, Eigen3.

## Global Constraints
- Zero tracking loss (100% frame tracking maintained).
- Must compile inside `nix develop`.
- Comparison results generated against baseline in `docs/superpowers/artifacts/parallel_optimization_comparison_report.md`.

---

### Task 1: OpenMP Compiler Flags (`CMakeLists.txt`)

**Files:**
- Modify: `CMakeLists.txt:20-40`

- [ ] **Step 1: Update `CMakeLists.txt` with OpenMP and target CPU flags**

```cmake
find_package(OpenMP REQUIRED)
if(OpenMP_CXX_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -O3 -march=native -ffast-math -funroll-loops")
endif()
```

- [ ] **Step 2: Commit**

```bash
git add CMakeLists.txt
git commit -m "build: enable OpenMP and SIMD optimization flags in CMakeLists.txt"
```

---

### Task 2: Parallel Stereo Keypoint Matching (`src/Frame.cc`)

**Files:**
- Modify: `src/Frame.cc:845-960`

- [ ] **Step 1: Parallelize `Frame::ComputeStereoMatches` loop with OpenMP**

```cpp
#pragma omp parallel for schedule(dynamic, 16)
for(int iL=0; iL<N; iL++)
{
    // Existing per-keypoint matching and subpixel sliding window calculation
}
```

- [ ] **Step 2: Commit**

```bash
git add src/Frame.cc
git commit -m "perf: parallelize Frame::ComputeStereoMatches with OpenMP"
```

---

### Task 3: Parallel ORB Feature Extraction (`src/ORBextractor.cc`)

**Files:**
- Modify: `src/ORBextractor.cc:80-150`

- [ ] **Step 1: Parallelize ORB descriptor computation per octave level**

- [ ] **Step 2: Commit**

```bash
git add src/ORBextractor.cc
git commit -m "perf: parallelize ORBextractor descriptor computation with OpenMP"
```

---

### Task 4: Benchmark Execution & Comparison Report

**Files:**
- Output: `docs/superpowers/artifacts/optimized_benchmark_results.json`
- Output: `docs/superpowers/artifacts/parallel_optimization_comparison_report.md`

- [ ] **Step 1: Execute benchmark and generate comparison report**

Run: `nix develop --command python3 scripts/run_baseline_benchmark.py`

- [ ] **Step 2: Commit comparison report**

```bash
git add docs/superpowers/artifacts/
git commit -m "docs: add baseline vs parallel CPU benchmark comparison report"
```
