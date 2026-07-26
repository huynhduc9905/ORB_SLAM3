# ORB-SLAM3 Phase 4 Zero-Allocation Patch Correlation & Parallel Pyramid Spec

## 1. Overview
This document specifies the design for Phase 4 optimizations:
1. Direct zero-allocation raw pointer $11 \times 11$ patch $L_1$ norm calculation in `Frame::ComputeStereoMatches()`.
2. OpenMP multi-threaded image pyramid downsampling across all 8 octave scale levels in `ORBextractor::ComputePyramid()`.
3. Execution safety via `stereo_benchmark` direct exit cleanup.

## 2. Target Performance Goal
- Target: Reach maximum FPS throughput (targeting 45–60 Hz) on AMD Ryzen 7 5825U hardware with 100% tracking accuracy.
