# ORB-SLAM3 60 FPS Performance Optimization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Optimize ORB-SLAM3 on `feature/web-visualizer` branch to reach 60+ FPS (mean tracking latency $\le 16.66\text{ ms}$) on `circle_run` and `full_run` datasets.

**Architecture:** Parallelize pyramid level feature extraction using OpenMP, replace heap allocations in `DistributeOctTree` with a contiguous memory arena, optimize descriptor copies with `std::memcpy`, and vectorize ORB descriptor Hamming distance computation using 64-bit `__builtin_popcountll`.

**Tech Stack:** C++17, OpenMP, OpenCV 4, GCC 15 (Nix environment).

## Global Constraints

- Linux x86_64 Nix Flake environment (`. /nix/var/nix/profiles/default/etc/profile.d/nix-daemon.sh && nix develop`)
- Mean tracking latency $\le 16.66\text{ ms}$ (60 FPS) across `circle_run` and `full_run`
- Zero tracking regressions or frame loss

---

### Task 1: Parallelize ORB Extraction Pyramid Levels with OpenMP

**Files:**
- Modify: `src/ORBextractor.cc:810-880`

**Interfaces:**
- Consumes: `vToDistributeKeys` and `mvImagePyramid`
- Produces: Parallelized `allKeypoints` and `descriptors` computation across pyramid levels

- [ ] **Step 1: Inspect `ORBextractor::operator()` level loop**

Read `src/ORBextractor.cc` lines 810-880 to confirm current pyramid level loop structure.

- [ ] **Step 2: Add `#pragma omp parallel for` across pyramid levels**

Edit `src/ORBextractor.cc` to parallelize the pyramid level loop using OpenMP.

```cpp
#pragma omp parallel for
for (int level = 0; level < nlevels; ++level)
{
    // GaussianBlur and ComputeKeyPointsOctTree for each level
}
```

- [ ] **Step 3: Compile and verify build**

Run: `. /nix/var/nix/profiles/default/etc/profile.d/nix-daemon.sh && nix develop --command bash -c "cd build && make -j4"`
Expected: Build succeeds without errors.

- [ ] **Step 4: Commit changes**

```bash
git add src/ORBextractor.cc
git commit -m "perf: parallelize ORB pyramid extraction levels with OpenMP"
```

---

### Task 2: Implement Contiguous Memory Arena for OctTree Distribution

**Files:**
- Modify: `include/ORBextractor.h`
- Modify: `src/ORBextractor.cc:556-780`

**Interfaces:**
- Consumes: `vector<cv::KeyPoint>` per pyramid level
- Produces: Extracted & distributed keypoints using pre-allocated node pool instead of `std::list` heap churn

- [ ] **Step 1: Define `ExtractorNodePool` arena structure in `include/ORBextractor.h`**

Add thread-safe or per-level node vector pool to `ExtractorNode`.

- [ ] **Step 2: Refactor `DistributeOctTree` in `src/ORBextractor.cc`**

Replace `std::list<ExtractorNode> lNodes` dynamic node allocation with contiguous vector arena indexing.

- [ ] **Step 3: Compile and verify build**

Run: `. /nix/var/nix/profiles/default/etc/profile.d/nix-daemon.sh && nix develop --command bash -c "cd build && make -j4"`
Expected: Build succeeds cleanly.

- [ ] **Step 4: Commit changes**

```bash
git add include/ORBextractor.h src/ORBextractor.cc
git commit -m "perf: eliminate dynamic heap churn in DistributeOctTree using node arena"
```

---

### Task 3: Vectorize ORB Hamming Distance & Direct Descriptor Copy

**Files:**
- Modify: `src/ORBmatcher.cc:30-70`
- Modify: `src/ORBextractor.cc:860-900`

**Interfaces:**
- Consumes: 256-bit binary ORB descriptors (32 bytes)
- Produces: 64-bit vectorized `popcountll` Hamming distance

- [ ] **Step 1: Optimize `DescriptorDistance` in `src/ORBmatcher.cc`**

Replace 32-bit `__builtin_popcount` 8-iteration loop with 64-bit `__builtin_popcountll` 4-iteration loop:

```cpp
int ORBmatcher::DescriptorDistance(const cv::Mat &a, const cv::Mat &b)
{
    const uint64_t *pa = a.ptr<uint64_t>();
    const uint64_t *pb = b.ptr<uint64_t>();

    int dist = 0;
    dist += __builtin_popcountll(pa[0] ^ pb[0]);
    dist += __builtin_popcountll(pa[1] ^ pb[1]);
    dist += __builtin_popcountll(pa[2] ^ pb[2]);
    dist += __builtin_popcountll(pa[3] ^ pb[3]);

    return dist;
}
```

- [ ] **Step 2: Replace OpenCV `copyTo` row assignment with `std::memcpy` in `src/ORBextractor.cc`**

Replace `desc.row(i).copyTo(descriptors.row(nkps+i))` with raw pointer `std::memcpy`.

- [ ] **Step 3: Compile and test**

Run: `. /nix/var/nix/profiles/default/etc/profile.d/nix-daemon.sh && nix develop --command bash -c "cd build && make -j4"`
Expected: Build succeeds cleanly.

- [ ] **Step 4: Commit changes**

```bash
git add src/ORBmatcher.cc src/ORBextractor.cc
git commit -m "perf: vectorize DescriptorDistance with popcountll and direct memcpy"
```

---

### Task 4: End-to-End Benchmark Validation & Final Goal Check

**Files:**
- Modify: `docs/superpowers/artifacts/baseline_benchmark_results.json`

- [ ] **Step 1: Execute full benchmark suite**

Run: `. /nix/var/nix/profiles/default/etc/profile.d/nix-daemon.sh && nix develop --command python3 scripts/run_baseline_benchmark.py`

- [ ] **Step 2: Verify metrics**

Read `docs/superpowers/artifacts/baseline_benchmark_report.md` and check:
- `circle_run` avg FPS $\ge 60.0$ (mean latency $\le 16.66\text{ ms}$)
- `full_run` avg FPS $\ge 60.0$ (mean latency $\le 16.66\text{ ms}$)

- [ ] **Step 3: Final Commit to `feature/web-visualizer`**

```bash
git status
git commit -m "perf: achieve 60+ FPS target on circle_run and full_run datasets"
```
