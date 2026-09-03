# Concurrency, Memory Lifecycle & Tracking Stability Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Eliminate all remaining concurrency data races, memory leaks, visualization latency bottlenecks, and teardown hazards across ORB-SLAM3 core subsystems on `fix/core-stability-patches`.

**Architecture:** 
1. `KeyFrameDatabase`: Query-scoped local caching (`queryScores`) to eliminate concurrent mutations on shared `KeyFrame` instances.
2. `Tracking`: Keyframe-gated and map-change-driven dirty flags with rate-limiting for the web visualizer to drop lock acquisitions from 30 Hz to ~1–2 Hz while ensuring complete freshness.
3. `Map`: Auxiliary FIFO deque to strictly bound tombstone history to 1,000 entries by actual erasure time.
4. `System`: Timed liveness polling during thread join with leak-on-deadlock protection to eliminate shutdown Use-After-Free crashes.
5. `Toolchain`: Modernized GTest dependency resolution in `flake.nix` for `-DBUILD_TESTS=ON`.

**Tech Stack:** C++11/C++14, Nix Flakes, CMake, Google Test (GTest 1.17.0), POSIX Threads, OpenCV 4, Eigen 3.

**Spec:** [`docs/superpowers/specs/2026-09-03-concurrency-lifecycle-stability-design.md`](file:///home/duc/orb-playground/ORB_SLAM3/docs/superpowers/specs/2026-09-03-concurrency-lifecycle-stability-design.md)

## Global Constraints
- Do NOT run `nix flake update` (preserve pinned `flake.lock`).
- Preserve existing public C++ API signatures and snapshot data structures.
- All code must build cleanly with `-DBUILD_TESTS=ON` under `nix develop`.
- Adhere to the established lock ordering: `mMutexMap` -> `mMutexConnections` -> `mMutexFeatures`.

---

### Task 1: Nix Flake GTest Toolchain Integration

**Files:**
- Modify: `flake.nix:52-66`
- Test: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `pkgs.gtest` from existing `nixpkgs` input
- Produces: GTest include headers and CMake imported targets (`GTest::gtest`, `GTest::gtest_main`)

- [ ] **Step 1: Inspect flake.nix buildInputs and add pkgs.gtest**
  In `flake.nix`, add `gtest` to the `buildInputs` list in `mkShell`.
- [ ] **Step 2: Test CMake test configuration in Nix shell**
  Run: `nix develop --command bash -c "cd build && cmake -DBUILD_TESTS=ON .."`
  Verify that CMake locates GTest cleanly without errors.
- [ ] **Step 3: Commit toolchain change**
  `git commit -am "build(nix): add pkgs.gtest to dev shell buildInputs"`

---

### Task 2: KeyFrameDatabase Thread-Safe Local Score Cache

**Files:**
- Modify: `src/KeyFrameDatabase.cc:69-73, 660-715`
- Test: `tests/test_keyframe_database.cc` or unit test runner

**Interfaces:**
- Consumes: `pKF->mBowVec`, `pKFi->mBowVec`, `mpVoc->score()`
- Produces: `std::unordered_map<KeyFrame*, float> queryScores` (thread-local), 100% read-only access to `pKFi`/`pKF2`

- [ ] **Step 1: Add mutex protection to KeyFrameDatabase::clear()**
  In `src/KeyFrameDatabase.cc:69`, wrap `mvInvertedFile.clear()` and `resize()` with `unique_lock<mutex> lock(mMutex);`.
- [ ] **Step 2: Refactor DetectNBestCandidates Pass 1 to use local queryScores**
  Declare `std::unordered_map<KeyFrame*, float> queryScores;` inside `DetectNBestCandidates`.
  In lines 664–675, replace `pKFi->mPlaceRecognitionScore = si;` with `queryScores[pKFi] = si;`.
- [ ] **Step 3: Refactor DetectNBestCandidates Pass 2 covisibility accumulation**
  In lines 691–708:
  Look up `pKF2` in `queryScores`.
  If present: use `queryScores[pKF2]`.
  If absent: compute `float si = mpVoc->score(pKF->mBowVec, pKF2->mBowVec);`, insert `queryScores.emplace(pKF2, si);`, and use `si`.
  Delete lines 696–700 (`pKF2->mPlaceRecognitionScore = si; pKF2->mnPlaceRecognitionWords = minCommonWords + 1;`).
- [ ] **Step 4: Verify build and test compilation**
  Run: `nix develop --command make -j4 -C build ORB_SLAM3`
- [ ] **Step 5: Commit KeyFrameDatabase concurrency fixes**
  `git commit -am "fix(keyframedb): thread-safe queryScores cache and mutex in clear()"`

---

### Task 3: Map Strict FIFO Bounded Tombstones

**Files:**
- Modify: `include/Map.h:175-200`
- Modify: `src/Map.cc:130-155`
- Test: `tests/test_web_viewer_mirror.cc`

**Interfaces:**
- Consumes: `pKF->mnId`, `CaptureKeyframeSnapshot()`
- Produces: Strict FIFO deletion order capped at `kMaxErasedTombstones = 1000`

- [ ] **Step 1: Declare kMaxErasedTombstones and mErasedKeyframeOrder in include/Map.h**
  Add:
  ```cpp
  static constexpr size_t kMaxErasedTombstones = 1000;
  std::deque<std::uint64_t> mErasedKeyframeOrder;
  ```
- [ ] **Step 2: Update Map::EraseKeyFrame() in src/Map.cc**
  After inserting into `mErasedKeyframeSnapshots[pKF->mnId]`, push `pKF->mnId` to `mErasedKeyframeOrder`.
  While `mErasedKeyframeOrder.size() > kMaxErasedTombstones`:
  Pop front ID and erase that ID from `mErasedKeyframeSnapshots`.
- [ ] **Step 3: Update Map::AddKeyFrame() in src/Map.cc**
  If a re-inserted keyframe ID exists in `mErasedKeyframeSnapshots`, erase it from `mErasedKeyframeSnapshots`.
- [ ] **Step 4: Verify build and snapshot test**
  Run: `nix develop --command make -j4 -C build test_web_viewer_mirror`
- [ ] **Step 5: Commit Map FIFO tombstone bounding**
  `git commit -am "fix(map): enforce strict FIFO bounded tombstone queue (max 1000)"`

---

### Task 4: Tracking KeyFrame & Map-Change Visualizer Decoupling

**Files:**
- Modify: `include/Tracking.h:120-140, 240-270`
- Modify: `src/Tracking.cc:1890-1905, 2410-2440, 3320-3350`
- Test: `tests/test_web_viewer_mirror.cc`

**Interfaces:**
- Consumes: `pCurrentMap->GetMapChangeIndex()`, `Tracking::CreateNewKeyFrame()`
- Produces: Decoupled 30 FPS pose streaming with throttled full map points events

- [ ] **Step 1: Add dirty flag and rate-limit timestamp to include/Tracking.h**
  ```cpp
  public:
      void RequestMapVisualizationUpdate() noexcept {
          mbMapUpdatedForVisualizer.store(true, std::memory_order_relaxed);
      }
  private:
      std::atomic<bool> mbMapUpdatedForVisualizer{true};
      std::chrono::steady_clock::time_point mLastMapPubTime{};
  ```
- [ ] **Step 2: Trigger flag in Tracking::CreateNewKeyFrame() and Track()**
  In `Tracking::CreateNewKeyFrame()`, set `RequestMapVisualizationUpdate()`.
  In `Tracking::Track()` lines 1892–1898, when `nCurMapChangeIndex > nMapChangeIndex`, also call `RequestMapVisualizationUpdate()`.
- [ ] **Step 3: Gate full map points in Tracking::PublishVisualizationState()**
  In `src/Tracking.cc:2412-2437`:
  Always publish `VisualizationFrameSnapshot` and `VisualizationImageSnapshot` at 30 FPS.
  Check if `mbMapUpdatedForVisualizer.load(std::memory_order_relaxed)`:
  If true, check if at least 250ms elapsed since `mLastMapPubTime`.
  If so, set `mLastMapPubTime = now; mbMapUpdatedForVisualizer.store(false, std::memory_order_relaxed);` and execute `pMap->GetAllMapPoints()` publishing `POINTS_UPDATED`.
- [ ] **Step 4: Verify build**
  Run: `nix develop --command make -j4 -C build ORB_SLAM3`
- [ ] **Step 5: Commit visualizer decoupling**
  `git commit -am "perf(tracking): throttle visualizer map point sync to keyframe and map change events"`

---

### Task 5: System Safe Timed Teardown with Leak-on-Deadlock

**Files:**
- Modify: `src/System.cc:365-435`
- Test: `tests/test_web_viewer_mirror.cc`

**Interfaces:**
- Consumes: `subsystem->isFinished()`, `worker->joinable()`
- Produces: Safe timed teardown preventing Use-After-Free and `std::terminate()`

- [ ] **Step 1: Implement safeJoinWorker helper in src/System.cc**
  Implement template `safeJoinWorker` polling `subsystem->isFinished()` for up to 5.0 seconds.
  If timed out, log an error, call `worker->detach()`, and set a deadlocked flag.
- [ ] **Step 2: Guard heap destruction in System::Cleanup()**
  In `System::Cleanup(bool destroyResources)`:
  If `localMapperDeadlocked` is true, skip `delete mpLocalMapper;`.
  If `loopCloserDeadlocked` is true, skip `delete mpLoopCloser;`.
  If either is true, skip `delete mpAtlas; delete mpKeyFrameDatabase;` to prevent Use-After-Free.
- [ ] **Step 3: Verify build and shutdown tests**
  Run: `nix develop --command make -j4 -C build`
- [ ] **Step 4: Commit safe shutdown teardown**
  `git commit -am "fix(system): timed shutdown join with leak-on-deadlock safety guard"`

---

### Task 6: Full Regression & Test Suite Verification

**Files:**
- Test: All unit test binaries in `build/`
- Build: Full library and example targets

- [ ] **Step 1: Build test suite with -DBUILD_TESTS=ON**
  Run: `nix develop --command bash -c "cd build && cmake -DBUILD_TESTS=ON .. && make -j$(nproc)"`
- [ ] **Step 2: Run CTest test suite**
  Run: `nix develop --command bash -c "cd build && ctest --output-on-failure"`
- [ ] **Step 3: Verify live hardware branch rebase**
  Checkout `feature/live-d435i-stm32` and rebase onto `fix/core-stability-patches`.
  Build `stereo_d435i_live` and `stereo_inertial_d435i_stm32`.
