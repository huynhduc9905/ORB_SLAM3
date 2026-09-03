# Concurrency, Memory Lifecycle & Tracking Stability Design (Refined)

## 1. Overview & Context

This design resolves the concurrency, memory lifecycle, and real-time performance liabilities identified across the ORB-SLAM3 core engine on `fix/core-stability-patches`, incorporating refinements from dual independent subagent technical reviews:

1. **`KeyFrameDatabase` Unsynchronized Mutation:** In `DetectNBestCandidates()`, both `pKFi->mPlaceRecognitionScore` (line 671) and `pKF2->mPlaceRecognitionScore` (lines 696–700) are mutated with no locks held on the keyframes. Additionally, `KeyFrameDatabase::clear()` mutates `mvInvertedFile` without acquiring `mMutex`.
2. **`Tracking` Visualizer Jitter & Ghost Points:** In `PublishVisualizationState()`, `GetAllMapPoints()` copies and transforms all map points under `mMutexMap` on every camera frame (30 FPS) when web subscribers connect. However, gating updates *strictly* on keyframe creation starves the visualizer during stationary camera loop closures.
3. **`Map` Tombstone Eviction Order & Memory Leak:** In `Map::EraseKeyFrame()`, `mErasedKeyframeSnapshots` grows monotonically without bound on long runs. Using `map::begin()` evicts by lowest `mnId` rather than oldest deletion timestamp.
4. **`System::Cleanup()` Unbounded Join & Teardown UAF:** `worker->join()` can block indefinitely if a subsystem thread hangs during shutdown. Detaching a deadlocked thread while deleting heap subsystems causes use-after-free crashes.
5. **Nix Test Suite Toolchain:** `flake.nix` is missing `pkgs.gtest`, breaking `-DBUILD_TESTS=ON` CMake configuration.

---

## 2. Goals & Non-Goals

### Goals
- **100% Thread-Safe Place Recognition:** Zero concurrent writes to `KeyFrame` member variables during `DetectNBestCandidates()`, and mutex protection in `clear()`.
- **Zero-Jitter 30 FPS Tracking with Full Data Freshness:** Eliminate `mMutexMap` acquisition and large map point copying from normal 30 FPS tracking frames, while triggering map point updates on both keyframe creation AND map change index increments (loop closures / local BA fusion).
- **Strict FIFO Bounded Tombstone Footprint:** Cap `mErasedKeyframeSnapshots` to a fixed ceiling using an auxiliary FIFO order queue (`std::deque<uint64_t>`) so deletion order is preserved and memory remains bounded.
- **Graceful Shutdown & Leak-on-Deadlock Safety:** Ensure `System::Cleanup()` polls thread liveness with a 5s timeout. If deadlocked, detach the thread and skip deleting that subsystem and `mpAtlas` to guarantee zero Use-After-Free crashes.
- **Out-of-the-Box Test Compilation:** Add `pkgs.gtest` to `flake.nix` without bumping `flake.lock`.

### Non-Goals
- Altering the mathematical BoW scoring logic or place recognition criteria.
- Modifying the external WebSocket wire protocol or browser visualizer client.
- Redesigning the multi-camera / Atlas architecture.

---

## 3. Subsystem Detailed Designs

### 3.1. `KeyFrameDatabase`: Local Query-Scoped Score Cache & Mutex Protection

#### Problem
In `KeyFrameDatabase::DetectNBestCandidates()`:
- Line 671: `pKFi->mPlaceRecognitionScore = si;` mutates candidate keyframes without locks.
- Lines 696–700: `pKF2->mPlaceRecognitionScore = si;` mutates neighbor keyframes without locks.
- `KeyFrameDatabase::clear()` modifies `mvInvertedFile` without `mMutex`.

#### Solution
1. Use a local stack-allocated hash map inside `DetectNBestCandidates`:
   ```cpp
   std::unordered_map<KeyFrame*, float> queryScores;
   ```
2. In Pass 1 (lines 661–680), store computed scores in `queryScores`:
   ```cpp
   if(pKFi->mnPlaceRecognitionWords > minCommonWords) {
       nscores++;
       float si = mpVoc->score(pKF->mBowVec, pKFi->mBowVec);
       queryScores[pKFi] = si;
       lScoreAndMatch.push_back(make_pair(si, pKFi));
   }
   ```
3. In Pass 2 (covisibility neighbor accumulation, lines 691–708):
   ```cpp
   for(vector<KeyFrame*>::iterator vit = vpNeighs.begin(), vend = vpNeighs.end(); vit != vend; vit++) {
       KeyFrame* pKF2 = *vit;
       if(pKF2->mnPlaceRecognitionQuery != pKF->mnId)
           continue;

       auto itScore = queryScores.find(pKF2);
       float score2 = 0.f;
       if(itScore != queryScores.end()) {
           score2 = itScore->second;
       } else {
           score2 = mpVoc->score(pKF->mBowVec, pKF2->mBowVec);
           queryScores.emplace(pKF2, score2);
       }

       accScore += score2;
       if(score2 > bestScore) {
           pBestKF = pKF2;
           bestScore = score2;
       }
   }
   ```
4. Eliminate all writes to `pKF2->mPlaceRecognitionScore` and `pKF2->mnPlaceRecognitionWords`.
5. In `KeyFrameDatabase::clear()`:
   ```cpp
   void KeyFrameDatabase::clear() {
       unique_lock<mutex> lock(mMutex);
       mvInvertedFile.clear();
       mvInvertedFile.resize(mpVoc->size());
   }
   ```

---

### 3.2. `Tracking`: KeyFrame & Map-Change-Driven Visualizer Updates

#### Problem
Calling `pMap->GetAllMapPoints()` on every frame (30 Hz) under `mMutexMap` introduces severe latency jitter. However, gating updates *only* on `CreateNewKeyFrame()` starves the viewer when the camera is stationary during loop closing.

#### Solution
1. Add an atomic dirty flag and rate-limit timestamp to `Tracking`:
   ```cpp
   // Tracking.h
   std::atomic<bool> mbMapUpdatedForVisualizer{true};
   std::chrono::steady_clock::time_point mLastMapPubTime{};
   ```
2. Trigger the dirty flag from:
   - `Tracking::CreateNewKeyFrame()` on new keyframe generation.
   - `Tracking::Track()` when `pCurrentMap->GetMapChangeIndex()` increments:
     ```cpp
     int nCurMapChangeIndex = pCurrentMap->GetMapChangeIndex();
     int nMapChangeIndex = pCurrentMap->GetLastMapChange();
     if(nCurMapChangeIndex > nMapChangeIndex) {
         pCurrentMap->SetLastMapChange(nCurMapChangeIndex);
         mbMapUpdated = true;
         mbMapUpdatedForVisualizer.store(true, std::memory_order_relaxed);
     }
     ```
   - On state changes (`LOST` / `OK` / reset / merge).
3. In `Tracking::PublishVisualizationState()`:
   - Always publish `VisualizationFrameSnapshot` (camera pose) and `VisualizationImageSnapshot` (2D tracking features) at **30 FPS**.
   - Check if `mbMapUpdatedForVisualizer` is set AND at least 250ms have elapsed since `mLastMapPubTime`. If so, exchange the flag to false, update `mLastMapPubTime`, and publish `VisualizationEventType::POINTS_UPDATED`.
4. *Result:* Smooth 30 FPS camera tracking, full data freshness on loop closures, and zero map lock overhead during pure tracking frames.

---

### 3.3. `Map`: Strict FIFO Bounded Tombstone Queue

#### Problem
In `Map::EraseKeyFrame(KeyFrame* pKF)`, `mErasedKeyframeSnapshots[pKF->mnId]` retains all culled keyframes forever. Using `mErasedKeyframeSnapshots.erase(mErasedKeyframeSnapshots.begin())` evicts the lowest `KeyFrame::mnId`, not the oldest deletion.

#### Solution
1. In `include/Map.h`:
   ```cpp
   static constexpr size_t kMaxErasedTombstones = 1000;
   std::deque<std::uint64_t> mErasedKeyframeOrder;
   ```
2. In `src/Map.cc` `EraseKeyFrame()` (under existing `mMutexMap` lock):
   ```cpp
   mErasedKeyframeSnapshots[pKF->mnId] = CaptureKeyframeSnapshot(pKF, true, nullptr);
   mErasedKeyframeOrder.push_back(pKF->mnId);

   while(mErasedKeyframeOrder.size() > kMaxErasedTombstones) {
       const std::uint64_t oldestId = mErasedKeyframeOrder.front();
       mErasedKeyframeOrder.pop_front();
       mErasedKeyframeSnapshots.erase(oldestId);
   }
   ```
3. In `Map::AddKeyFrame()`: if an erased keyframe is re-added, remove its ID from `mErasedKeyframeSnapshots`.

---

### 3.4. `System::Cleanup()`: Liveness Polling & Leak-on-Deadlock Safety

#### Problem
`worker->join()` hangs if a worker deadlocks. Calling `worker->detach()` and deleting `mpLocalMapper`/`mpAtlas` causes catastrophic Use-After-Free crashes.

#### Solution
1. Implement a safe helper in `System.cc`:
   ```cpp
   template<typename SubsystemPtr>
   void safeJoinWorker(std::thread*& worker, SubsystemPtr subsystem, const char* name, bool& subsystemDeadlocked) {
       if(!worker) return;
       if(worker->joinable() && worker->get_id() != std::this_thread::get_id()) {
           const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
           while(subsystem && !subsystem->isFinished() && std::chrono::steady_clock::now() < deadline) {
               std::this_thread::sleep_for(std::chrono::milliseconds(20));
           }
           if(subsystem && !subsystem->isFinished()) {
               std::cerr << "CRITICAL ERROR: " << name << " thread did not exit within 5s shutdown timeout; possible deadlock." << std::endl;
               subsystemDeadlocked = true;
               worker->detach();
           } else {
               worker->join();
           }
       }
       delete worker;
       worker = nullptr;
   }
   ```
2. In `System::Cleanup(bool destroyResources)`:
   - Maintain flags `bool localMapperDeadlocked = false;` and `bool loopCloserDeadlocked = false;`.
   - If either thread deadlocks, log the error and **skip deleting `mpLocalMapper` / `mpLoopCloser` / `mpAtlas`** to prevent memory corruption.

---

### 3.5. Toolchain: Nix Flake GTest Support

#### Solution
In `flake.nix`: Add `pkgs.gtest` to `buildInputs` in `mkShell`. Do **not** run `nix flake update` (preserve pinned `flake.lock`).

---

## 4. Verification & Test Plan

1. **Compilation:** `nix develop --command make -j$(nproc)` builds all targets cleanly.
2. **Unit Tests:** `cmake -DBUILD_TESTS=ON .. && make -j4 && ctest --output-on-failure`.
3. **Concurrency Audit:** Verify zero mutations of `KeyFrame` fields in `DetectNBestCandidates()`.
4. **Visualizer Responsiveness:** Test stationary camera loop closure; verify map points refresh via `GetMapChangeIndex()`.
