# ORB-SLAM3 Accuracy Restoration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove every accuracy-affecting regression introduced by the performance work on `feature/web-visualizer`, while keeping all crash fixes, the web visualizer, and the accuracy-neutral performance wins.

**Architecture:** Fix in place on `feature/web-visualizer` with surgical, individually-reviewable commits rather than reverting whole commits — the perf regressions and crash fixes are interleaved inside the same commits (`1f0110e`, `2994b4b`), so commit-level reverts would drop crash fixes. Upstream ORB-SLAM3 V1.0 (`master`) numerics are the reference: where this branch changed a numeric or match-selection behaviour without a benchmarked justification, restore the upstream behaviour. Introduce a GoogleTest harness (none exists today) so each fix is proven by a failing-then-passing test.

**Tech Stack:** C++17, CMake, OpenCV 4.6.0, Eigen3, g2o/DBoW2/Sophus (vendored), OpenMP, GoogleTest 1.17.0 (available at `/usr/include/gtest/gtest.h`).

## Global Constraints

- Branch: work on `feature/web-visualizer`. Do **not** rebase, reset, force-push, or rewrite history.
- Reference for correct behaviour: `git show master:<path>` (upstream ORB-SLAM3 V1.0).
- Never remove a crash fix, null guard, bounds check, or lifecycle fix in the course of reverting a perf change.
- Never revert the multi-lap loop-revisit change (`>=20` keyframe-ID gap) — it was reviewed as a legitimate improvement, gated by four independent geometric checks.
- Do not touch `web_viewer/`, `src/WebViewerBackend.cc`, `src/WebViewerProtocol.cc`, `src/VisualizationSource.cc`, or the `Examples/*web_runner*` files.
- Preserve these accuracy-neutral perf wins: 64-bit POPCNT / AVX2 `DescriptorDistance`, `MapPoint::CopyDescriptor` into stack buffers, `vIndices.reserve(64)` hoisting, `GetFeaturesInArea` out-parameter overload, per-level OpenMP in `ORBextractor`, batched `Map::AddMapPoints` locking, reused pyramid buffers (`mvTempPyramidBuffers`), and `mvImagePyramidBlurred`.
- Build type stays `Release`; build with `./build.sh` or `cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc)`.
- One task = one commit. Commit messages use `fix(accuracy):`, `fix(build):`, `test:`, or `chore:` prefixes.

---

## Verified Findings Being Fixed

Each was confirmed by reading the code at HEAD (`cb40ca1`), not just from review claims.

| # | Finding | Location | Verified how | Severity |
|---|---------|----------|--------------|----------|
| F1 | `-ffast-math` implies `-ffinite-math-only`; GCC may fold `std::isfinite`/`isnan` to constants, disabling 4 live guards — including the Sim3Solver NaN guard added by *this* branch as a crash fix | `CMakeLists.txt:27`; guards at `src/Sim3Solver.cc:375,403`, `src/TwoViewReconstruction.cc:835`, `src/Optimizer.cc:2981` | Read flags + grepped guards | Critical |
| F2 | Image pyramid resized from **full-res input** every level instead of cascaded from `level-1`; changes pixels at every level ≥2 → different FAST corners, descriptors, matches, poses | `src/ORBextractor.cc:1179` | Read current `ComputePyramid` in full | Critical |
| F3 | Frame-to-frame `SearchByProjection` reduction writes `mvpMapPoints[bestIdx]` **unconditionally** (last-wins overwrite) and increments `nmatches` for every candidate including overwritten ones → inflated `nmatches` returned to `TrackWithMotionModel`, which uses it as a tracking-success threshold; duplicate `rotHist` entries can also double-decrement `nmatches` | `src/ORBmatcher.cc:1891-1908`, decrement at `:1925-1928` | Read function + reduction | Critical |
| F4 | Local-map `SearchByProjection` defers writes, so the first-wins exclusion never fires *within* a call; a later map point landing on a claimed keypoint is dropped in the reduction instead of falling back to its second-best keypoint → lost matches | `src/ORBmatcher.cc:44-230` | Read function + reduction | Critical |
| F5 | The OpenMP pragmas that motivated F3/F4's deferred-write restructuring **no longer exist** (`grep -c "pragma omp" src/ORBmatcher.cc` = 0; was 2 at `1f0110e`, 0 from `2994b4b` on). The risky semantics buy zero performance today. | `src/ORBmatcher.cc` | Counted pragmas per commit | Critical (makes F3/F4 free to fix) |
| F6 | Visual `PoseOptimization` cut from 4 rounds × 10 iters (40) to 2 × 5 (10) — a 75% cut, halving outlier-reclassification passes. The IMU variant (`:5200`) and the other variant (`:4801`) were left at 40, so the frontends are now inconsistent. | `src/Optimizer.cc:1008,1011` | Read + grepped all `its[4]` | Important |
| F7 | `mMutexMapUpdate` scope narrowed from the whole `Track()` body to a 6-line check → tracking can observe a mid-update map, losing the frozen-map guarantee | `src/Tracking.cc:1887-1899` | Read current scope | Important |
| F8 | `MapPoint::mGlobalMutex` lock removed from `PoseOptimization` edge construction | `src/Optimizer.cc` (PoseOptimization) | Review + diff | Important |
| F9 | `mnVisible`/`mnFound` now written lock-free but remain plain `int` → formal data race vs `MapPoint::Replace()` | `src/MapPoint.cc:329-336`, `include/MapPoint.h:251-252` | Read decls | Important |
| F10 | `-O0 -g` in the general flags. Empirically **neutralized** in Release (`-O3 -DNDEBUG` from `CMAKE_CXX_FLAGS_RELEASE` is appended after and wins), but a latent hazard that silently disables optimization in any non-Release build, plus `-g` binary bloat | `CMakeLists.txt:12,13,27` | Reproduced flag order with a throwaway CMake project: `-Wall -O0 -g -fopenmp -O0 -g ... -O3 -DNDEBUG` | Important |
| F11 | `FORB::distance` non-AVX2 path casts `uint8_t*` → `uint64_t*` (strict-aliasing UB), unlike the `__builtin_memcpy` version used in `ORBmatcher.h` | `Thirdparty/DBoW2/DBoW2/FORB.cpp:99-104` | Read both paths | Minor |
| F12 | Committed ELF binaries `test_size_t`, `test_sad` tracked in git; `REGISTER_TIMES` profiling unconditionally enabled; hardcoded nix-store Boost include path | repo root, `CMakeLists.txt:33`, `CMakeLists.txt:409` | `git ls-files` + `file` | Minor |

**Explicitly assessed and NOT changed:** AVX2/POPCNT `DescriptorDistance` (bit-identical), per-level OpenMP (races checked, deterministic ordering preserved via row-major `vRowKeys`), multi-lap loop revisit (four geometric gates intact), GBA join-instead-of-detach lifecycle fix, double `pop_front` fix, KeyFrame OOB/`size_t`→`int` fixes, `LocalMapping::Release` `SetBadFlag` UAF fix, `MapPoint` observation `map`→`vector` (all accesses locked), `GetDescriptor()` without clone (safe: OpenCV 4.6 uses atomic refcounts), `KeyFrame::UpdateBestCovisibles` TOCTOU (traded a real deadlock for a benign staleness — keep).

---

## File Structure

**Created:**
- `tests/CMakeLists.txt` — test target wiring, guarded by `BUILD_TESTS` option.
- `tests/test_descriptor_distance.cc` — proves POPCNT/AVX2 paths are bit-identical to a reference popcount (F11 regression net).
- `tests/test_numeric_guards.cc` — proves `isfinite`/`isnan` guards survive the project's compile flags (F1).
- `tests/test_orb_pyramid.cc` — proves pyramid levels equal a cascaded reference (F2).
- `tests/test_match_bookkeeping.cc` — proves `nmatches` equals the number of populated slots and no slot is overwritten (F3, F4).

**Modified:**
- `CMakeLists.txt` — drop `-ffast-math`, restore `-O3`, drop `-g`, add `BUILD_TESTS`, make `REGISTER_TIMES` optional, add a fatal guard against `-ffast-math` creeping back.
- `src/ORBextractor.cc` — cascaded pyramid resize.
- `src/ORBmatcher.cc` — restore immediate-write match semantics in both `SearchByProjection` variants.
- `src/Optimizer.cc` — restore `PoseOptimization` iterations; restore `MapPoint::mGlobalMutex`.
- `src/Tracking.cc` — restore full-scope `mMutexMapUpdate`.
- `include/MapPoint.h`, `src/MapPoint.cc` — `std::atomic<int>` counters.
- `Thirdparty/DBoW2/DBoW2/FORB.cpp` — memcpy instead of aliasing cast.
- `.gitignore` — ignore stray build artifacts.

---

### Task 1: GoogleTest harness + descriptor-distance bit-identity test

Establishes the test harness and simultaneously locks in that the POPCNT/AVX2 optimization is bit-exact (so we can keep it with confidence).

**Files:**
- Create: `tests/CMakeLists.txt`
- Create: `tests/test_descriptor_distance.cc`
- Modify: `CMakeLists.txt` (append test wiring near the end, after `target_link_libraries(${PROJECT_NAME} ...)`)

**Interfaces:**
- Consumes: `ORB_SLAM3::ORBmatcher::DescriptorDistance(const uchar*, const uchar*)` (static, declared in `include/ORBmatcher.h`), `DBoW2::FORB::distance(const cv::Mat&, const cv::Mat&)`.
- Produces: a `orb_slam3_tests` CMake target runnable as `./build/tests/orb_slam3_tests`; later tasks add `.cc` files to the `SOURCES` list in `tests/CMakeLists.txt`.

- [ ] **Step 1: Write the failing test**

Create `tests/test_descriptor_distance.cc`:

```cpp
#include <gtest/gtest.h>
#include <opencv2/core.hpp>
#include <cstdint>
#include <random>

#include "ORBmatcher.h"
#include "Thirdparty/DBoW2/DBoW2/FORB.h"

namespace {

// Reference: naive per-bit Hamming distance over 32 bytes (256 bits).
int ReferenceHamming(const uint8_t* a, const uint8_t* b) {
    int dist = 0;
    for (int i = 0; i < 32; ++i) {
        uint8_t v = static_cast<uint8_t>(a[i] ^ b[i]);
        while (v) { dist += (v & 1u); v >>= 1; }
    }
    return dist;
}

TEST(DescriptorDistance, MatchesNaiveReferenceOnRandomDescriptors) {
    std::mt19937 rng(12345);
    std::uniform_int_distribution<int> byte_dist(0, 255);

    for (int trial = 0; trial < 2000; ++trial) {
        alignas(32) uint8_t a[32];
        alignas(32) uint8_t b[32];
        for (int i = 0; i < 32; ++i) {
            a[i] = static_cast<uint8_t>(byte_dist(rng));
            b[i] = static_cast<uint8_t>(byte_dist(rng));
        }
        EXPECT_EQ(ORB_SLAM3::ORBmatcher::DescriptorDistance(a, b),
                  ReferenceHamming(a, b)) << "trial " << trial;
    }
}

TEST(DescriptorDistance, HandlesIdenticalAndInvertedDescriptors) {
    alignas(32) uint8_t a[32];
    alignas(32) uint8_t b[32];
    for (int i = 0; i < 32; ++i) { a[i] = 0xA5; b[i] = 0xA5; }
    EXPECT_EQ(ORB_SLAM3::ORBmatcher::DescriptorDistance(a, b), 0);

    for (int i = 0; i < 32; ++i) { b[i] = 0x5A; }  // 0xA5 ^ 0x5A = 0xFF
    EXPECT_EQ(ORB_SLAM3::ORBmatcher::DescriptorDistance(a, b), 256);
}

// Guards F11: the DBoW2 fallback must agree with the reference too.
TEST(ForbDistance, MatchesNaiveReferenceOnUnalignedRows) {
    std::mt19937 rng(999);
    std::uniform_int_distribution<int> byte_dist(0, 255);

    // Two rows of a 3x32 CV_8U Mat; row 1 and row 2 exercise non-base offsets.
    cv::Mat descs(3, 32, CV_8U);
    for (int r = 0; r < descs.rows; ++r)
        for (int c = 0; c < descs.cols; ++c)
            descs.at<uint8_t>(r, c) = static_cast<uint8_t>(byte_dist(rng));

    for (int r = 0; r + 1 < descs.rows; ++r) {
        cv::Mat x = descs.row(r);
        cv::Mat y = descs.row(r + 1);
        EXPECT_EQ(DBoW2::FORB::distance(x, y),
                  ReferenceHamming(x.ptr<uint8_t>(), y.ptr<uint8_t>()))
            << "rows " << r << "," << r + 1;
    }
}

}  // namespace
```

Create `tests/CMakeLists.txt`:

```cmake
find_package(GTest REQUIRED)

set(TEST_SOURCES
    test_descriptor_distance.cc
)

add_executable(orb_slam3_tests ${TEST_SOURCES})

target_include_directories(orb_slam3_tests PRIVATE
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/include/CameraModels
    ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(orb_slam3_tests
    ${PROJECT_NAME}
    GTest::gtest
    GTest::gtest_main
)

add_test(NAME orb_slam3_tests COMMAND orb_slam3_tests)
```

- [ ] **Step 2: Wire tests into the top-level build**

In `CMakeLists.txt`, immediately after the `set(CMAKE_CXX_STANDARD_REQUIRED ON)` line (line 18), add:

```cmake
option(BUILD_TESTS "Build unit tests" ON)
```

At the very end of `CMakeLists.txt`, append:

```cmake
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
```

- [ ] **Step 3: Run to verify it builds and passes**

```bash
cd /data/ORB_SLAM3/build && cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5 && make -j$(nproc) orb_slam3_tests && ./tests/orb_slam3_tests --gtest_filter='DescriptorDistance*:ForbDistance*'
```

Expected: all 3 tests PASS. These assert existing correct behaviour, so passing is the expected outcome — this task's deliverable is the harness. If `ForbDistance` fails, that is F11 biting in practice: record it and fix it in Task 8 rather than here.

- [ ] **Step 4: Commit**

```bash
git add tests/CMakeLists.txt tests/test_descriptor_distance.cc CMakeLists.txt
git commit -m "test: add GoogleTest harness and DescriptorDistance bit-identity tests"
```

---

### Task 2: Remove `-ffast-math`, restore `-O3` (F1, F10)

**Files:**
- Modify: `CMakeLists.txt:12-13,27`
- Create: `tests/test_numeric_guards.cc`
- Modify: `tests/CMakeLists.txt` (add source)

**Interfaces:**
- Consumes: nothing from earlier tasks except the `orb_slam3_tests` target from Task 1.
- Produces: no new API. Guarantees the library is compiled without `-ffast-math`.

- [ ] **Step 1: Write the failing test**

Create `tests/test_numeric_guards.cc`. `volatile` prevents constant-folding at compile time, so this exercises the runtime behaviour the SLAM guards rely on:

```cpp
#include <gtest/gtest.h>
#include <Eigen/Core>
#include <cmath>
#include <limits>

namespace {

// ORB-SLAM3 relies on these guards at:
//   src/Sim3Solver.cc:375,403  (NaN guard added as a crash fix on this branch)
//   src/TwoViewReconstruction.cc:835  (monocular init triangulation validity)
//   src/Optimizer.cc:2981  (LocalInertialBA divergence check)
// -ffast-math implies -ffinite-math-only, under which the compiler is
// permitted to assume NaN/Inf never occur and fold these checks away.

TEST(NumericGuards, IsFiniteDetectsNaNAtRuntime) {
    volatile double zero = 0.0;
    volatile double nan_value = zero / zero;
    EXPECT_FALSE(std::isfinite(nan_value));
    EXPECT_TRUE(std::isnan(nan_value));
}

TEST(NumericGuards, IsFiniteDetectsInfinityAtRuntime) {
    volatile double one = 1.0;
    volatile double zero = 0.0;
    volatile double inf_value = one / zero;
    EXPECT_FALSE(std::isfinite(inf_value));
}

TEST(NumericGuards, EigenAllFiniteDetectsNaN) {
    volatile double zero = 0.0;
    Eigen::Vector3d v(1.0, 2.0, static_cast<double>(zero / zero));
    EXPECT_FALSE(v.allFinite());
}

// Mirrors the exact guard shape used in Sim3Solver.cc:375.
TEST(NumericGuards, Sim3StyleGuardRejectsDegenerateAxisAngle) {
    volatile float zero = 0.0f;
    Eigen::Vector3f vec(1.0f, 0.0f, static_cast<float>(zero / zero));
    float ang = static_cast<float>(zero / zero);
    float vecNorm = 1.0f;
    const bool rejected = !vec.allFinite() || !std::isfinite(ang) || vecNorm < 1e-9f;
    EXPECT_TRUE(rejected);
}

}  // namespace
```

Add `test_numeric_guards.cc` to `TEST_SOURCES` in `tests/CMakeLists.txt`:

```cmake
set(TEST_SOURCES
    test_descriptor_distance.cc
    test_numeric_guards.cc
)
```

- [ ] **Step 2: Run the test against the current `-ffast-math` build to record the baseline**

```bash
cd /data/ORB_SLAM3/build && cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5 && make -j$(nproc) orb_slam3_tests && ./tests/orb_slam3_tests --gtest_filter='NumericGuards*'
```

Record the result verbatim. Under `-ffast-math` these may pass or fail depending on GCC version and how aggressively it folds; `volatile` limits folding. **Either outcome is acceptable** — the test documents intent, and the authoritative protection is the CMake guard in Step 3. Do not skip this step: knowing whether the flag was actively breaking the guards on this toolchain is the difference between "latent hazard" and "live bug", and that belongs in the commit message.

- [ ] **Step 3: Fix the flags**

In `CMakeLists.txt`, replace lines 12-13:

```cmake
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")
```

Replace line 27 (inside the `if(OpenMP_CXX_FOUND)` block) — drop `-ffast-math`, `-O0`, and `-g`; keep OpenMP, `-march=native`, and `-funroll-loops`:

```cmake
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -march=native -funroll-loops")
```

Then, immediately after the closing `endif()` of the OpenMP block, add a guard so the flag cannot silently return:

```cmake
# ORB-SLAM3 relies on IEEE-754 NaN/Inf semantics for its numeric guards
# (Sim3Solver, TwoViewReconstruction, Optimizer). -ffast-math implies
# -ffinite-math-only, which permits the compiler to fold those checks away.
if(CMAKE_CXX_FLAGS MATCHES "ffast-math" OR CMAKE_CXX_FLAGS MATCHES "funsafe-math-optimizations" OR CMAKE_CXX_FLAGS MATCHES "ffinite-math-only")
    message(FATAL_ERROR "Fast/unsafe math flags break ORB-SLAM3 NaN guards. Remove them from CMAKE_CXX_FLAGS.")
endif()
```

- [ ] **Step 4: Rebuild fully and verify**

A flag change invalidates every object file, so rebuild from scratch:

```bash
cd /data/ORB_SLAM3 && rm -rf build && mkdir build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release -DBoost_INCLUDE_DIR=/nix/store/pb7333fnknqxbwr229aqdd2abfvi97yv-boost-1.89.0-dev/include -DCMAKE_POLICY_VERSION_MINIMUM=3.5 && make -j$(nproc) 2>&1 | tail -20
make -C /data/ORB_SLAM3/build orb_slam3_tests -j$(nproc) && ./tests/orb_slam3_tests
```

Expected: library and tests build; all `NumericGuards` tests PASS; `DescriptorDistance` tests still PASS. Confirm the flag is gone:

```bash
grep -c "ffast-math" /data/ORB_SLAM3/CMakeLists.txt   # expect 3 (only the guard's own MATCHES strings)
make -C /data/ORB_SLAM3/build VERBOSE=1 2>/dev/null | grep -m1 "ORBextractor.cc" | grep -o "\-O[0-3]\|ffast-math"  # expect -O3 only
```

- [ ] **Step 5: Commit**

```bash
git add CMakeLists.txt tests/CMakeLists.txt tests/test_numeric_guards.cc
git commit -m "fix(build): drop -ffast-math and restore -O3

-ffast-math implies -ffinite-math-only, permitting GCC to fold away the
std::isfinite/isnan guards at Sim3Solver.cc:375,403 (added on this branch
as a crash fix), TwoViewReconstruction.cc:835 and Optimizer.cc:2981.
Also restores -O3 in the general flags and drops leftover -O0 -g debug
flags. Adds a CMake guard against reintroduction plus runtime tests for
the guard shapes the SLAM code depends on."
```

---

### Task 3: Restore cascaded image pyramid (F2)

The single most impactful accuracy fix: it changes the pixels every ORB descriptor is computed from at pyramid levels ≥ 2.

**Files:**
- Modify: `src/ORBextractor.cc:1161-1183` (`ORBextractor::ComputePyramid`)
- Create: `tests/test_orb_pyramid.cc`
- Modify: `tests/CMakeLists.txt` (add source)

**Interfaces:**
- Consumes: `ORB_SLAM3::ORBextractor(int nfeatures, float scaleFactor, int nlevels, int iniThFAST, int minThFAST)`, its `operator()`, and the public `mvImagePyramid` / `mvScaleFactor` / `GetScaleFactor()` members declared in `include/ORBextractor.h`.
- Produces: no API change. `ComputePyramid` keeps its signature `void ComputePyramid(cv::Mat image)` and keeps using the reused `mvTempPyramidBuffers`.

- [ ] **Step 1: Write the failing test**

Create `tests/test_orb_pyramid.cc`:

```cpp
#include <gtest/gtest.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <random>
#include <vector>

#include "ORBextractor.h"

namespace {

constexpr int kNLevels = 8;
constexpr float kScaleFactor = 1.2f;

cv::Mat MakeTexturedImage(int rows, int cols) {
    // Deterministic high-frequency texture: downsampling method is only
    // distinguishable on content with energy near Nyquist.
    cv::Mat img(rows, cols, CV_8U);
    std::mt19937 rng(2024);
    std::uniform_int_distribution<int> noise(0, 60);
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            const int checker = ((r / 2 + c / 2) % 2) ? 200 : 40;
            img.at<uint8_t>(r, c) = cv::saturate_cast<uint8_t>(checker + noise(rng));
        }
    }
    return img;
}

// Independent reference implementation of upstream ORB-SLAM3 pyramid
// construction: each level is resized from the PREVIOUS level.
std::vector<cv::Mat> CascadedReferencePyramid(const cv::Mat& image) {
    std::vector<float> inv_scale(kNLevels, 1.0f);
    float scale = 1.0f;
    for (int i = 1; i < kNLevels; ++i) {
        scale *= kScaleFactor;
        inv_scale[i] = 1.0f / scale;
    }

    std::vector<cv::Mat> pyramid(kNLevels);
    pyramid[0] = image.clone();
    for (int level = 1; level < kNLevels; ++level) {
        const cv::Size sz(cvRound(static_cast<float>(image.cols) * inv_scale[level]),
                          cvRound(static_cast<float>(image.rows) * inv_scale[level]));
        cv::resize(pyramid[level - 1], pyramid[level], sz, 0, 0, cv::INTER_LINEAR);
    }
    return pyramid;
}

TEST(OrbPyramid, LevelsMatchCascadedDownsampling) {
    const cv::Mat image = MakeTexturedImage(480, 640);

    ORB_SLAM3::ORBextractor extractor(1000, kScaleFactor, kNLevels, 20, 7);
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    std::vector<int> lapping = {0, 0};
    extractor(image, cv::Mat(), keypoints, descriptors, lapping);

    const std::vector<cv::Mat> expected = CascadedReferencePyramid(image);

    for (int level = 0; level < kNLevels; ++level) {
        ASSERT_EQ(extractor.mvImagePyramid[level].size(), expected[level].size())
            << "level " << level;
        const cv::Mat diff = extractor.mvImagePyramid[level] != expected[level];
        EXPECT_EQ(cv::countNonZero(diff), 0)
            << "level " << level << " differs from cascaded reference; "
            << "pyramid must resize from level-1, not from the full-res input";
    }
}

// Proves the test above actually discriminates: resizing directly from the
// full-resolution input yields different pixels at coarse levels.
TEST(OrbPyramid, DirectResizeIsDistinguishableFromCascaded) {
    const cv::Mat image = MakeTexturedImage(480, 640);
    const std::vector<cv::Mat> cascaded = CascadedReferencePyramid(image);

    int differing_levels = 0;
    for (int level = 2; level < kNLevels; ++level) {
        cv::Mat direct;
        cv::resize(image, direct, cascaded[level].size(), 0, 0, cv::INTER_LINEAR);
        if (cv::countNonZero(direct != cascaded[level]) > 0) ++differing_levels;
    }
    EXPECT_GT(differing_levels, 0)
        << "if this fails the pyramid test cannot detect the regression";
}

}  // namespace
```

Add `test_orb_pyramid.cc` to `TEST_SOURCES` in `tests/CMakeLists.txt`.

- [ ] **Step 2: Run test to verify it fails**

```bash
cd /data/ORB_SLAM3/build && make -j$(nproc) orb_slam3_tests && ./tests/orb_slam3_tests --gtest_filter='OrbPyramid*'
```

Expected: `LevelsMatchCascadedDownsampling` FAILS with non-zero differing pixels at levels ≥ 2. `DirectResizeIsDistinguishableFromCascaded` PASSES.

- [ ] **Step 3: Fix `ComputePyramid`**

In `src/ORBextractor.cc`, in the `else` branch of `ComputePyramid` (currently line 1179), change the resize source from the full-resolution `image` to the previous pyramid level. The destination `mvImagePyramid[level]` is a view into `mvTempPyramidBuffers[level]` and the source is a view into `mvTempPyramidBuffers[level-1]`, so these are distinct allocations and the resize does not alias.

Replace:

```cpp
                resize(image, mvImagePyramid[level], sz, 0, 0, INTER_LINEAR);
```

with:

```cpp
                resize(mvImagePyramid[level-1], mvImagePyramid[level], sz, 0, 0, INTER_LINEAR);
```

Leave everything else in the function untouched — the buffer reuse (`mvTempPyramidBuffers`), the `Rect` view, and both `copyMakeBorder` calls stay exactly as they are. The loop already runs `level` ascending from 0, so `mvImagePyramid[level-1]` is fully populated before use.

- [ ] **Step 4: Run tests to verify they pass**

```bash
cd /data/ORB_SLAM3/build && make -j$(nproc) orb_slam3_tests && ./tests/orb_slam3_tests --gtest_filter='OrbPyramid*'
```

Expected: both tests PASS.

- [ ] **Step 5: Commit**

```bash
git add src/ORBextractor.cc tests/test_orb_pyramid.cc tests/CMakeLists.txt
git commit -m "fix(accuracy): resize pyramid from previous level, not full-res input

Building every pyramid level by resizing the full-resolution image skips
the progressive anti-aliasing of cascaded downsampling, changing pixel
content at all levels >=2 and therefore FAST corner locations, ORB
descriptors, matches and poses. Restores upstream cascaded construction
while keeping the reused pyramid buffers."
```

---

### Task 4: Restore immediate-write semantics in frame-to-frame `SearchByProjection` (F3)

Fixes the inflated `nmatches` that `TrackWithMotionModel` uses as a tracking-success threshold, plus the last-wins overwrite. Free of performance cost: the OpenMP pragma this restructuring existed to serve is gone (F5).

**Files:**
- Modify: `src/ORBmatcher.cc:1696-1910` (`SearchByProjection(Frame&, const Frame&, float, bool)`)
- Create: `tests/test_match_bookkeeping.cc`
- Modify: `tests/CMakeLists.txt` (add source)

**Interfaces:**
- Consumes: `ORB_SLAM3::ORBmatcher::DescriptorDistance` (Task 1).
- Produces: no signature change. `SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const float th, const bool bMono)` keeps returning `int`, but the return value now equals the number of slots actually populated.

- [ ] **Step 1: Write the failing test**

This is a pure bookkeeping-invariant test — it needs no dataset and no camera model, because it validates the reduction logic's arithmetic, which is where the bug lives. Create `tests/test_match_bookkeeping.cc`:

```cpp
#include <gtest/gtest.h>
#include <vector>

namespace {

// Models the reduction phase of SearchByProjection. Two source features can
// select the same destination keypoint; the buggy version writes both and
// counts both, so the returned count exceeds the number of populated slots.

struct MatchResult { int pMP; int bestIdx; };

int BuggyReduce(const std::vector<MatchResult>& results, std::vector<int>& slots) {
    int nmatches = 0;
    for (const auto& res : results) {
        if (res.bestIdx != -1) {
            slots[res.bestIdx] = res.pMP;   // unconditional: last-wins overwrite
            nmatches++;                      // counted even when overwriting
        }
    }
    return nmatches;
}

int FixedReduce(const std::vector<MatchResult>& results, std::vector<int>& slots) {
    int nmatches = 0;
    for (const auto& res : results) {
        if (res.bestIdx != -1 && slots[res.bestIdx] == 0) {
            slots[res.bestIdx] = res.pMP;
            nmatches++;
        }
    }
    return nmatches;
}

int CountPopulated(const std::vector<int>& slots) {
    int n = 0;
    for (int s : slots) if (s != 0) ++n;
    return n;
}

TEST(MatchBookkeeping, BuggyReductionInflatesMatchCount) {
    // Map points 11 and 22 both pick destination keypoint 3.
    const std::vector<MatchResult> results = {{11, 3}, {22, 3}, {33, 5}};
    std::vector<int> slots(8, 0);

    const int reported = BuggyReduce(results, slots);
    EXPECT_EQ(reported, 3);
    EXPECT_EQ(CountPopulated(slots), 2);
    EXPECT_NE(reported, CountPopulated(slots)) << "documents the bug";
    EXPECT_EQ(slots[3], 22) << "last-wins overwrite";
}

TEST(MatchBookkeeping, FixedReductionCountEqualsPopulatedSlots) {
    const std::vector<MatchResult> results = {{11, 3}, {22, 3}, {33, 5}};
    std::vector<int> slots(8, 0);

    const int reported = FixedReduce(results, slots);
    EXPECT_EQ(reported, CountPopulated(slots));
    EXPECT_EQ(reported, 2);
    EXPECT_EQ(slots[3], 11) << "first-wins, matching upstream";
}

}  // namespace
```

Add `test_match_bookkeeping.cc` to `TEST_SOURCES` in `tests/CMakeLists.txt`.

- [ ] **Step 2: Run to confirm the invariant is characterised**

```bash
cd /data/ORB_SLAM3/build && make -j$(nproc) orb_slam3_tests && ./tests/orb_slam3_tests --gtest_filter='MatchBookkeeping*'
```

Expected: both PASS. `BuggyReductionInflatesMatchCount` documents exactly what `ORBmatcher.cc:1891-1908` does today; `FixedReductionCountEqualsPopulatedSlots` specifies the required behaviour. This is the executable specification for Step 3.

- [ ] **Step 3: Restore immediate writes in the real function**

In `src/ORBmatcher.cc`, in `SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const float th, const bool bMono)`:

1. Delete the `struct MatchResult { ... };` declaration and the `vector<MatchResult> vResults(LastFrame.N, {nullptr, -1, -1, -1, -1});` line.
2. In the **left/monocular** branch, replace the deferred store:

```cpp
                if(bestDist<=TH_HIGH)
                {
                    vResults[i].pMP = pMP;
                    vResults[i].bestIdx = bestIdx2;
```

with an immediate write matching upstream:

```cpp
                if(bestDist<=TH_HIGH)
                {
                    CurrentFrame.mvpMapPoints[bestIdx2]=pMP;
                    nmatches++;
```

and change the orientation-histogram store in that same block from `vResults[i].bin = bin;` to `rotHist[bin].push_back(bestIdx2);`.

3. In the **right-camera** branch, replace:

```cpp
                    if(bestDist<=TH_HIGH)
                    {
                        vResults[i].pMP = pMP;
                        vResults[i].bestIdxRight = bestIdx2 + CurrentFrame.Nleft;
```

with:

```cpp
                    if(bestDist<=TH_HIGH)
                    {
                        CurrentFrame.mvpMapPoints[bestIdx2 + CurrentFrame.Nleft]=pMP;
                        nmatches++;
```

and change `vResults[i].binRight = bin;` to `rotHist[bin].push_back(bestIdx2 + CurrentFrame.Nleft);`.

4. Delete the entire sequential reduction loop (the block beginning `// Sequential reduction of candidate results into CurrentFrame and rotHist` through its closing brace, currently lines ~1890-1909).

Restoring immediate writes also re-arms the in-loop exclusion `if(CurrentFrame.mvpMapPoints[i2]) if(...Observations()>0) continue;`, so a later feature now naturally avoids a claimed keypoint instead of colliding with it. Cross-check the finished function against upstream:

```bash
git diff master -- src/ORBmatcher.cc | sed -n '/SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame/,/^@@/p' | head -60
```

The only remaining differences in this function should be `DescriptorDistance(dMP.data, pD)` taking raw pointers. There must be no `vResults` reference left:

```bash
grep -n "vResults\|MatchResult" src/ORBmatcher.cc   # expect no output
```

- [ ] **Step 4: Rebuild and verify**

```bash
cd /data/ORB_SLAM3/build && make -j$(nproc) 2>&1 | tail -5 && make -j$(nproc) orb_slam3_tests && ./tests/orb_slam3_tests
```

Expected: library builds clean; all tests PASS.

- [ ] **Step 5: Commit**

```bash
git add src/ORBmatcher.cc tests/test_match_bookkeeping.cc tests/CMakeLists.txt
git commit -m "fix(accuracy): restore immediate match writes in frame-to-frame SearchByProjection

The deferred reduction wrote mvpMapPoints unconditionally (last-wins
overwrite) and incremented nmatches for every candidate, inflating the
count TrackWithMotionModel uses as its tracking-success threshold, and
allowing duplicate rotHist entries to double-decrement it. The OpenMP
pragma this restructuring existed to serve was removed in 2994b4b, so
restoring upstream semantics costs no performance."
```

---

### Task 5: Restore immediate-write semantics in local-map `SearchByProjection` (F4)

Same root cause, different function. Here the deferred write also **loses** matches: a map point whose best keypoint was already claimed is dropped instead of falling back to its second-best. Safe optimizations in this function must be preserved.

**Files:**
- Modify: `src/ORBmatcher.cc:44-230` (`SearchByProjection(Frame&, const vector<MapPoint*>&, float, bool, float)`)

**Interfaces:**
- Consumes: `MapPoint::CopyDescriptor(uchar*)`, `Frame::GetFeaturesInArea(float, float, float, int, int, bool, std::vector<size_t>&)` out-parameter overload.
- Produces: no signature change; return value equals the number of slots populated.

- [ ] **Step 1: Confirm the current behaviour, then restore immediate writes**

Preserve all of: the `alignas(16) uchar MPdescriptor[32]` + `pMP->CopyDescriptor(...)` stack-buffer optimization, the hoisted `vIndices` / `vIndicesRight` with `reserve(64)`, and the out-parameter `GetFeaturesInArea`. Only the write timing changes.

1. Delete the `struct MatchCandidate { ... };` declaration and `vector<MatchCandidate> vMatches(vpMapPoints.size(), {nullptr, -1, -1, -1, -1});`.
2. Delete `MatchCandidate mc{pMP, -1, -1, -1, -1};` and the trailing `vMatches[iMP] = mc;`.
3. In the left-camera branch, replace the deferred store:

```cpp
                if(bestDist<=TH_HIGH)
                {
                    if(bestLevel!=bestLevel2 || bestDist<=mfNNratio*bestDist2){
                        mc.bestIdx = bestIdx;
                        if(F.Nleft != -1 && F.mvLeftToRightMatch[bestIdx] != -1){
                            mc.stereoIdx = F.mvLeftToRightMatch[bestIdx] + F.Nleft;
                        }
                    }
                }
```

with the upstream immediate form:

```cpp
                if(bestDist<=TH_HIGH)
                {
                    if(bestLevel!=bestLevel2 || bestDist<=mfNNratio*bestDist2){
                        F.mvpMapPoints[bestIdx]=pMP;

                        if(F.Nleft != -1 && F.mvLeftToRightMatch[bestIdx] != -1){ //Also match with the stereo observation at right camera
                            F.mvpMapPoints[F.mvLeftToRightMatch[bestIdx] + F.Nleft] = pMP;
                            nmatches++;
                            right++;
                        }

                        nmatches++;
                        left++;
                    }
                }
```

4. In the right-camera branch, replace:

```cpp
                    if(bestDist<=TH_HIGH)
                    {
                        if(bestLevel!=bestLevel2 || bestDist<=mfNNratio*bestDist2){
                            if(F.mvRightToLeftMatch[bestIdx] != -1){
                                mc.stereoIdxRight = F.mvRightToLeftMatch[bestIdx];
                            }
                            mc.bestIdxRight = bestIdx + F.Nleft;
                        }
                    }
```

with:

```cpp
                    if(bestDist<=TH_HIGH)
                    {
                        if(bestLevel!=bestLevel2 || bestDist<=mfNNratio*bestDist2){
                            if(F.mvRightToLeftMatch[bestIdx] != -1){
                                F.mvpMapPoints[F.mvRightToLeftMatch[bestIdx]] = pMP;
                                nmatches++;
                                left++;
                            }

                            F.mvpMapPoints[bestIdx + F.Nleft]=pMP;
                            nmatches++;
                            right++;
                        }
                    }
```

5. Delete the whole trailing reduction loop (`for(size_t iMP=0; iMP<vpMapPoints.size(); iMP++) { const auto& mc = vMatches[iMP]; ... }`), keeping the final `return nmatches;`.

Verify nothing is left over and compare against upstream:

```bash
grep -n "vMatches\|MatchCandidate" src/ORBmatcher.cc   # expect no output
git diff master -- src/ORBmatcher.cc | grep -c "^+.*CopyDescriptor"  # expect >=1: optimization preserved
```

- [ ] **Step 2: Rebuild and run the full test suite**

```bash
cd /data/ORB_SLAM3/build && make -j$(nproc) 2>&1 | tail -5 && make -j$(nproc) orb_slam3_tests && ./tests/orb_slam3_tests
```

Expected: builds clean, all tests PASS.

- [ ] **Step 3: Smoke-test that tracking still runs end to end**

```bash
ls /data/ORB_SLAM3/build/stereo_benchmark /data/ORB_SLAM3/Examples/Stereo/stereo_benchmark.cc
```

If a dataset is configured, run `scripts/run_baseline_benchmark.py` and confirm the run completes without assertion failures. If no dataset is present, note that end-to-end validation is deferred to Task 10 and say so explicitly in the commit message.

- [ ] **Step 4: Commit**

```bash
git add src/ORBmatcher.cc
git commit -m "fix(accuracy): restore immediate match writes in local-map SearchByProjection

Deferring writes to a reduction phase disabled the in-loop first-wins
exclusion, so a map point whose best keypoint was already claimed was
dropped entirely instead of falling back to its second-best candidate,
losing valid matches in feature-dense areas. Keeps the CopyDescriptor
stack buffer, hoisted vIndices reserve(64), and out-parameter
GetFeaturesInArea optimizations."
```

---

### Task 6: Restore visual `PoseOptimization` iteration budget (F6)

**Files:**
- Modify: `src/Optimizer.cc:1008,1011`

**Interfaces:**
- Consumes: nothing new.
- Produces: no API change.

- [ ] **Step 1: Restore the upstream budget**

`PoseOptimization` runs on every frame and determines both the tracked pose and the inlier/outlier classification. The branch cut it from 4 rounds × 10 iterations to 2 × 5 — a 75% reduction in total Levenberg-Marquardt iterations and, more consequentially, a halving of outlier-reclassification passes. The IMU variant at `:5200` and the variant at `:4801` were left at 4 × 10, so the frontends are currently inconsistent.

In `src/Optimizer.cc` line 1008:

```cpp
    const int its[4]={10,10,10,10};
```

and line 1011:

```cpp
    for(size_t it=0; it<4; it++)
```

Leave `chi2Mono` and `chi2Stereo` unchanged — they were already correct. Confirm all three sites now agree:

```bash
grep -n "its\[4\]" src/Optimizer.cc   # expect 10,10,10,10 at all three sites
grep -n "for(size_t it=0; it<" src/Optimizer.cc | head
```

Per YAGNI, do **not** add a configuration knob for this. If FPS must be recovered later, that is a separate, benchmarked change that reports its ATE cost alongside its speed gain.

- [ ] **Step 2: Rebuild and verify**

```bash
cd /data/ORB_SLAM3/build && make -j$(nproc) 2>&1 | tail -5 && ./tests/orb_slam3_tests
```

Expected: builds clean, all tests PASS.

- [ ] **Step 3: Commit**

```bash
git add src/Optimizer.cc
git commit -m "fix(accuracy): restore PoseOptimization to 4x10 iterations

Cutting the per-frame pose optimizer from 4 rounds x 10 iterations to
2 x 5 removed 75% of the LM iterations and halved the outlier
reclassification passes that progressively tighten the inlier set. It
also left the IMU variant at 4x10, making the visual and inertial
frontends inconsistent. Any future reduction must ship with measured
ATE numbers."
```

---

### Task 7: Restore map-update exclusion in tracking (F7, F8, F9)

These are concurrency-consistency restorations. Grouped because they share one verification strategy (ThreadSanitizer plus the concurrent stress test) and one behavioural claim: tracking must see a stable map for the duration of a `Track()` call.

**Files:**
- Modify: `src/Tracking.cc:1886-1899`
- Modify: `src/Optimizer.cc` (`PoseOptimization`, edge-construction loop)
- Modify: `include/MapPoint.h:251-252`
- Modify: `src/MapPoint.cc` (`IncreaseVisible`, `IncreaseFound`, and the `Replace`/`GetFoundRatio` readers)

**Interfaces:**
- Consumes: `Map::mMutexMapUpdate`, `MapPoint::mGlobalMutex`.
- Produces: `MapPoint::mnVisible` and `MapPoint::mnFound` become `std::atomic<int>`; `IncreaseVisible(int n)`, `IncreaseFound(int n)`, `GetFoundRatio()`, `GetFound()`, `GetVisible()` keep their existing signatures.

- [ ] **Step 1: Restore the full-scope map-update lock**

In `src/Tracking.cc`, the lock currently covers only the change-index check:

```cpp
    // Check map update status with scoped lock
    {
        unique_lock<mutex> lock(pCurrentMap->mMutexMapUpdate);
        mbMapUpdated = false;
        ...
    }
```

Upstream holds `mMutexMapUpdate` for the remainder of `Track()`, which is what guarantees tracking sees a frozen map while it matches, optimizes, and decides on keyframe insertion. Restore that by taking the lock un-scoped, exactly as upstream does:

```bash
git diff master -- src/Tracking.cc | sed -n '/mMutexMapUpdate/,+25p'
```

Replace the scoped block with the upstream form — a single `unique_lock` declared at that point whose lifetime extends to the end of the enclosing function scope:

```cpp
    // Get Map Mutex -> Map cannot be changed
    unique_lock<mutex> lock(pCurrentMap->mMutexMapUpdate);

    mbMapUpdated = false;

    int nCurMapChangeIndex = pCurrentMap->GetMapChangeIndex();
    int nMapChangeIndex = pCurrentMap->GetLastMapChange();
    if(nCurMapChangeIndex>nMapChangeIndex)
    {
        pCurrentMap->SetLastMapChange(nCurMapChangeIndex);
        mbMapUpdated = true;
    }
```

Confirm the lock is no longer released early and that no inner scope re-locks the same mutex (which would self-deadlock):

```bash
grep -n "mMutexMapUpdate" src/Tracking.cc
```

There must be exactly one acquisition in `Track()`. If a nested acquisition exists, remove the inner one.

- [ ] **Step 2: Restore `MapPoint::mGlobalMutex` in `PoseOptimization`**

In `src/Optimizer.cc`, `PoseOptimization`'s edge-construction loop reads `MapPoint` world positions. Upstream serializes that with a global lock. Restore it immediately before the loop over `pFrame->mvpMapPoints`:

```cpp
    {
    unique_lock<mutex> lock(MapPoint::mGlobalMutex);
```

closing the scope after the edge-construction loop ends, matching upstream:

```bash
git diff master -- src/Optimizer.cc | grep -n "mGlobalMutex"
```

- [ ] **Step 3: Make the tracking counters atomic**

In `include/MapPoint.h`, replace:

```cpp
     // Tracking counters
     int mnVisible;
     int mnFound;
```

with:

```cpp
     // Tracking counters (written lock-free from Tracking, read by
     // LocalMapping/LoopClosing via Replace and the culling heuristics)
     std::atomic<int> mnVisible;
     std::atomic<int> mnFound;
```

Ensure `#include <atomic>` is present in `include/MapPoint.h`. In `src/MapPoint.cc`, `IncreaseVisible`/`IncreaseFound` become plain `fetch_add`-equivalent compound assignments (`mnVisible += n;` works on `std::atomic<int>`), and any place that copies these into another `MapPoint` (notably `Replace`) must use `.load()` explicitly so the intent is visible. Serialization code that reads/writes these fields needs `.load()`/`.store()` too — the compiler will point out each site:

```bash
cd /data/ORB_SLAM3/build && make -j$(nproc) 2>&1 | grep -E "error|MapPoint.cc" | head -20
```

Fix each reported site by adding `.load()`; do not change any arithmetic.

- [ ] **Step 4: Verify with the full suite and a concurrency run**

```bash
cd /data/ORB_SLAM3/build && make -j$(nproc) 2>&1 | tail -5 && ./tests/orb_slam3_tests
```

Expected: builds clean, all tests PASS. Then exercise the concurrent path with the branch's own stress utility:

```bash
python3 /data/ORB_SLAM3/scripts/stress_test.py 2>&1 | tail -20
```

Expected: completes without crash, deadlock, or hang. **A hang here most likely means a nested `mMutexMapUpdate` acquisition from Step 1** — investigate with `gdb -p <pid>` and `thread apply all bt` rather than by narrowing the lock again.

Optionally build a ThreadSanitizer variant to check for remaining races. Note that TSan requires rebuilding the vendored thirdparty libraries and reports many pre-existing upstream races, so treat its output as informational:

```bash
cd /data/ORB_SLAM3 && cmake -S . -B build-tsan -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-fsanitize=thread -g" -DCMAKE_POLICY_VERSION_MINIMUM=3.5 && make -C build-tsan -j$(nproc) orb_slam3_tests 2>&1 | tail -5
```

- [ ] **Step 5: Commit**

```bash
git add src/Tracking.cc src/Optimizer.cc include/MapPoint.h src/MapPoint.cc
git commit -m "fix(accuracy): restore map-update exclusion during tracking

Narrowing mMutexMapUpdate to a 6-line change-index check let LocalMapping
cull and move map points while Track() was matching and optimizing
against them, losing the frozen-map guarantee the design assumes.
Also restores MapPoint::mGlobalMutex around PoseOptimization edge
construction and makes mnVisible/mnFound std::atomic<int>, since they are
now written lock-free but read under lock by MapPoint::Replace."
```

---

### Task 8: Fix `FORB::distance` strict aliasing (F11)

**Files:**
- Modify: `Thirdparty/DBoW2/DBoW2/FORB.cpp:98-105`

**Interfaces:**
- Consumes: the `ForbDistance` test from Task 1.
- Produces: no signature change.

- [ ] **Step 1: Replace the aliasing cast with memcpy**

The non-AVX2 fallback casts `uint8_t*` to `uint64_t*`, which is undefined behaviour under strict aliasing and assumes 8-byte alignment. `include/ORBmatcher.h` already solves this correctly with `__builtin_memcpy` into local buffers; mirror that. Replace:

```cpp
  const uint64_t *pa64 = (const uint64_t*)pa;
  const uint64_t *pb64 = (const uint64_t*)pb;
  return __builtin_popcountll(pa64[0] ^ pb64[0]) + 
         __builtin_popcountll(pa64[1] ^ pb64[1]) + 
         __builtin_popcountll(pa64[2] ^ pb64[2]) + 
         __builtin_popcountll(pa64[3] ^ pb64[3]);
```

with:

```cpp
  uint64_t va64[4], vb64[4];
  __builtin_memcpy(va64, pa, sizeof(va64));
  __builtin_memcpy(vb64, pb, sizeof(vb64));
  return __builtin_popcountll(va64[0] ^ vb64[0]) +
         __builtin_popcountll(va64[1] ^ vb64[1]) +
         __builtin_popcountll(va64[2] ^ vb64[2]) +
         __builtin_popcountll(va64[3] ^ vb64[3]);
```

The memcpy compiles to the same loads at `-O3`; it only removes the UB.

- [ ] **Step 2: Verify both code paths**

The AVX2 branch is selected on this machine (`-march=native`), so also compile the fallback explicitly to prove it is correct:

```bash
cd /data/ORB_SLAM3/build && make -j$(nproc) orb_slam3_tests && ./tests/orb_slam3_tests --gtest_filter='ForbDistance*'
cd /data/ORB_SLAM3 && cmake -S . -B build-noavx -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-mno-avx2" -DCMAKE_POLICY_VERSION_MINIMUM=3.5 > /dev/null && make -C build-noavx -j$(nproc) orb_slam3_tests 2>&1 | tail -3 && ./build-noavx/tests/orb_slam3_tests --gtest_filter='ForbDistance*:DescriptorDistance*'
```

Expected: PASS in both configurations. Then remove the scratch build dir: `rm -rf /data/ORB_SLAM3/build-noavx`.

- [ ] **Step 3: Commit**

```bash
git add Thirdparty/DBoW2/DBoW2/FORB.cpp
git commit -m "fix: avoid strict-aliasing UB in FORB::distance fallback

Mirrors the __builtin_memcpy approach already used in ORBmatcher.h
instead of casting uint8_t* to uint64_t*. Same codegen at -O3, no UB."
```

---

### Task 9: Repository hygiene (F12)

**Files:**
- Delete: `test_size_t`, `test_sad` (tracked ELF binaries)
- Modify: `.gitignore`, `CMakeLists.txt:33`

**Interfaces:** none.

- [ ] **Step 1: Remove tracked binaries and make profiling opt-in**

```bash
cd /data/ORB_SLAM3 && git rm --cached test_size_t test_sad && rm -f test_size_t test_sad
```

Append to `.gitignore`:

```
# Stray build artifacts
test_size_t
test_sad
build-tsan/
build-noavx/
```

`REGISTER_TIMES` is currently enabled unconditionally at `CMakeLists.txt:33`, which adds per-stage timing instrumentation to every production build. Make it opt-in — replace `add_definitions(-DREGISTER_TIMES)` with:

```cmake
option(REGISTER_TIMES "Enable per-stage timing instrumentation" OFF)
if(REGISTER_TIMES)
    add_definitions(-DREGISTER_TIMES)
endif()
```

Leave the hardcoded nix-store Boost include path at `CMakeLists.txt:409` alone for now — changing it risks breaking the local build, and it is a portability issue rather than a correctness one. Note it as follow-up.

- [ ] **Step 2: Verify the build still works with profiling off and on**

```bash
cd /data/ORB_SLAM3 && rm -rf build && mkdir build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release -DBoost_INCLUDE_DIR=/nix/store/pb7333fnknqxbwr229aqdd2abfvi97yv-boost-1.89.0-dev/include -DCMAKE_POLICY_VERSION_MINIMUM=3.5 && make -j$(nproc) 2>&1 | tail -5 && ./tests/orb_slam3_tests
```

Expected: builds clean with `REGISTER_TIMES` off; all tests PASS. If any `Examples/` target fails to compile because it references `vdIMUInteg_ms` or similar timing vectors unguarded, that is a pre-existing conditional-compilation gap — guard the offending reference with `#ifdef REGISTER_TIMES` rather than turning the definition back on.

- [ ] **Step 3: Commit**

```bash
git add -A .gitignore CMakeLists.txt
git commit -m "chore: untrack stray test binaries and make REGISTER_TIMES opt-in"
```

---

### Task 10: End-to-end accuracy validation against master

Unit tests prove the individual fixes. Only a dataset run proves the *system* accuracy is restored. This task is the one that substantiates any claim about ATE.

**Files:**
- Create: `docs/superpowers/artifacts/2026-08-06-accuracy-restoration-results.md`

**Interfaces:**
- Consumes: `Examples/Stereo/stereo_benchmark.cc` (built as `build/stereo_benchmark`), `scripts/run_baseline_benchmark.py`, `Vocabulary/ORBvoc.txt` (present).

- [ ] **Step 1: Obtain a dataset**

No SLAM dataset is currently present under `/data` (only OS images and the repo). EuRoC MH_01_easy is the standard choice: it has ground truth, it is stereo+IMU, and ORB-SLAM3's published numbers use it.

```bash
ls /data/ORB_SLAM3/Examples/Stereo/EuRoC.yaml   # config already in repo
```

**This step needs your confirmation before proceeding** — it downloads roughly 1.5 GB from an external host:

```bash
# Requires user approval before running.
mkdir -p /data/datasets/euroc && cd /data/datasets/euroc && \
  wget http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.zip && \
  unzip -q MH_01_easy.zip -d MH_01_easy
```

If the download is not permitted, stop here and report that system-level accuracy validation is blocked, listing exactly which unit tests did pass. Do not substitute a synthetic sequence and call it an ATE measurement.

- [ ] **Step 2: Measure master as the accuracy reference**

Use a separate worktree so this checkout's HEAD is never moved:

```bash
cd /data/ORB_SLAM3 && git worktree add /tmp/orbslam3-master master && cd /tmp/orbslam3-master && ./build.sh 2>&1 | tail -5
```

Run the stereo sequence and keep the trajectory:

```bash
cd /tmp/orbslam3-master && ./Examples/Stereo/stereo_euroc ./Vocabulary/ORBvoc.txt ./Examples/Stereo/EuRoC.yaml /data/datasets/euroc/MH_01_easy ./Examples/Stereo/EuRoC_TimeStamps/MH01.txt master_MH01
```

- [ ] **Step 3: Measure the fixed branch under identical conditions**

```bash
cd /data/ORB_SLAM3 && ./Examples/Stereo/stereo_euroc ./Vocabulary/ORBvoc.txt ./Examples/Stereo/EuRoC.yaml /data/datasets/euroc/MH_01_easy ./Examples/Stereo/EuRoC_TimeStamps/MH01.txt fixed_MH01
```

Run each configuration at least 3 times. ORB-SLAM3 is non-deterministic across runs (RANSAC seeding, thread interleaving), so a single run cannot distinguish a real regression from run-to-run variance. Report median and spread, not one number.

- [ ] **Step 4: Compute ATE and record results**

`evo` is the standard tool; install into a venv if absent:

```bash
python3 -m venv /tmp/evo-venv && /tmp/evo-venv/bin/pip install -q evo && \
/tmp/evo-venv/bin/evo_ape tum /data/datasets/euroc/MH_01_easy/mav0/state_groundtruth_estimate0/data.csv fixed_MH01_trajectory.txt -va
```

Write `docs/superpowers/artifacts/2026-08-06-accuracy-restoration-results.md` containing: the ATE RMSE median and range for master, for the branch before these fixes (`cb40ca1`, measurable via a third worktree if a before/after comparison is wanted), and for the branch after; plus tracking FPS for each. State plainly whether branch ATE is now within run-to-run variance of master.

The acceptance criterion is **branch ATE ≤ master ATE + run-to-run spread**. If the branch is still materially worse, do not declare the work complete: return to systematic-debugging and bisect the remaining difference across the commits in this plan.

- [ ] **Step 5: Clean up worktrees and commit results**

```bash
cd /data/ORB_SLAM3 && git worktree remove /tmp/orbslam3-master --force && rm -rf /tmp/evo-venv
git add docs/superpowers/artifacts/2026-08-06-accuracy-restoration-results.md
git commit -m "docs: record ATE comparison against master after accuracy restoration"
```

---

## Execution Order and Rationale

Tasks 1-3 first: the harness plus the pyramid fix, because F2 changes every descriptor in the system and therefore invalidates any measurement taken before it lands. Tasks 4-6 next: match bookkeeping and optimizer budget, each independently reviewable. Task 7 after those, because it is the highest-risk change (a lock-scope mistake deadlocks rather than degrades) and benefits from a known-good baseline. Tasks 8-9 are low-risk cleanup. Task 10 last, because it is the only step that can validate the whole set, and it must run on final code.

Request code review between tasks per superpowers:requesting-code-review, and run superpowers:verification-before-completion before claiming any task is done — every task above has an explicit expected-output step for exactly that purpose.

## Known Residual Items (not fixed by this plan)

Recorded so they are not mistaken for oversights:

- `KeyFrame::UpdateBestCovisibles` unlock/relock TOCTOU (`src/KeyFrame.cc:244-259`): deliberately kept. It replaced a real deadlock with benign staleness that self-corrects on the next call.
- `Frame` copy-constructor shallow copies of `mK`/`mDescriptors` and `imgLeft`/`imgRight` (`src/Frame.cc:57-58,1058-1059`): currently safe (those fields are never mutated post-construction, and grid queries never touch copied frames) but fragile. Worth a comment documenting the invariant.
- `-march=native` (pre-existing upstream in the Release flags): makes binaries non-portable and results non-reproducible across machines. Out of scope here.
- Hardcoded nix-store Boost include path at `CMakeLists.txt:409`.
- `MapPoint::GetDescriptor()` returning a non-cloned `cv::Mat`: safe on OpenCV 4.6.0 (atomic refcounts, verified installed version) but would be a race on OpenCV builds without atomic refcounting.
