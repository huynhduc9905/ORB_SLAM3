# ORB-SLAM3 Hardware POPCNT & Parallel Pyramid Optimization Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to execute this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Further optimize ORB-SLAM3 tracking performance by replacing legacy software bit-hacks with 64-bit hardware `POPCNT` instructions and parallelizing pyramid construction and OctTree feature extraction across all pyramid levels.

---

### Task 1: Hardware-Accelerated `POPCNT` Descriptor Matching (`src/ORBmatcher.cc`)

**Files:**
- Modify: `src/ORBmatcher.cc:2063-2079`

- [ ] **Step 1: Replace software bit-hacks in `ORBmatcher::DescriptorDistance` with `__builtin_popcountll`**

```cpp
int ORBmatcher::DescriptorDistance(const cv::Mat &a, const cv::Mat &b)
{
    const uint64_t *pa = a.ptr<uint64_t>();
    const uint64_t *pb = b.ptr<uint64_t>();

    return __builtin_popcountll(pa[0] ^ pb[0]) +
           __builtin_popcountll(pa[1] ^ pb[1]) +
           __builtin_popcountll(pa[2] ^ pb[2]) +
           __builtin_popcountll(pa[3] ^ pb[3]);
}
```

- [ ] **Step 2: Commit**

```bash
git add src/ORBmatcher.cc
git commit -m "perf: replace software descriptor bit-hacks with 64-bit hardware POPCNT"
```

---

### Task 2: Parallel Pyramid & OctTree Extraction (`src/ORBextractor.cc`)

**Files:**
- Modify: `src/ORBextractor.cc:782-790`
- Modify: `src/ORBextractor.cc:1173-1196`

- [ ] **Step 1: Parallelize `ORBextractor::ComputePyramid` and `ORBextractor::ComputeKeyPointsOctTree` with OpenMP**

- [ ] **Step 2: Commit**

```bash
git add src/ORBextractor.cc
git commit -m "perf: parallelize ORB pyramid construction and OctTree extraction with OpenMP"
```

---

### Task 3: Benchmark & Generate Phase 2 Performance Comparison Report

- [ ] **Step 1: Rebuild and execute benchmark orchestrator**

Run: `nix develop --command python3 scripts/run_baseline_benchmark.py`

- [ ] **Step 2: Save Phase 2 comparison report and commit**

```bash
git add docs/superpowers/artifacts/
git commit -m "docs: add Phase 2 hardware POPCNT optimization benchmark report"
```
