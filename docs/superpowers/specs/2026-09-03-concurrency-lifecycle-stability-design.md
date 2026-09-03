# Concurrency, Memory Lifecycle & Tracking Stability Design

## 1. Overview & Context

This design resolves the remaining concurrency, memory lifecycle, and real-time performance liabilities identified across the ORB-SLAM3 core engine on `fix/core-stability-patches`:

1. **`KeyFrameDatabase` Unsynchronized Mutation:** In `DetectNBestCandidates()`, `pKF2->mPlaceRecognitionScore` and `pKF2->mnPlaceRecognitionWords` are mutated with no locks held.
2. **`Tracking` Visualizer Jitter:** In `PublishVisualizationState()`, `GetAllMapPoints()` copies and transforms all map points under `mMutexMap` on every camera frame (30 FPS) when web subscribers connect.
3. **`Map` Tombstone Memory Leak:** In `Map::EraseKeyFrame()`, `mErasedKeyframeSnapshots` grows monotonically without bound on long runs.
4. **`System::Cleanup()` Unbounded Join:** `worker->join()` can block indefinitely if a subsystem thread hangs during shutdown.
5. **Nix Test Suite Toolchain:** `flake.nix` is missing `pkgs.gtest`, breaking `-DBUILD_TESTS=ON` CMake configuration.

---

## 2. Goals & Non-Goals

### Goals
- **100% Thread-Safe Place Recognition:** Zero concurrent writes to `KeyFrame` member variables during `DetectNBestCandidates()`.
- **Zero-Jitter 30 FPS Tracking:** Eliminate `mMutexMap` acquisition and large map point copying from the 30 FPS tracking frame loop; decouple full map events to keyframe generation.
- **Bounded Tombstone Footprint:** Cap `mErasedKeyframeSnapshots` to a fixed ceiling so multi-hour continuous mapping does not leak memory.
- **Graceful Shutdown Diagnostics:** Ensure `System::Cleanup()` logs diagnostics if a worker thread fails to exit within a bounded timeout.
- **Out-of-the-Box Test Compilation:** Add `pkgs.gtest` to `flake.nix` to support `-DBUILD_TESTS=ON`.

### Non-Goals
- Altering the mathematical BoW scoring logic or visual place recognition criteria.
- Modifying the external WebSocket wire protocol or browser visualizer client.
- Redesigning the multi-camera / Atlas architecture.

---

## 3. Subsystem Detailed Designs

### 3.1. `KeyFrameDatabase`: Local Query-Scoped Score Cache

#### Problem
In `KeyFrameDatabase::DetectNBestCandidates()`, lines 696–700:
```cpp
if(pKF2->mnPlaceRecognitionWords <= minCommonWords) {
    float si = mpVoc->score(pKF->mBowVec, pKF2->mBowVec);
    pKF2->mPlaceRecognitionScore = si;
    pKF2->mnPlaceRecognitionWords = minCommonWords + 1;
}
accScore += pKF2->mPlaceRecognitionScore;
```
Writing to `pKF2->mPlaceRecognitionScore` and `pKF2->mnPlaceRecognitionWords` occurs with **no mutex held** on `pKF2` and after `mMutex` of `KeyFrameDatabase` was released. This is a data race.

#### Solution
Eliminate in-place mutation of `KeyFrame` objects. Use a local stack-allocated hash map:
```cpp
std::unordered_map<KeyFrame*, float> queryScores;
```
1. During the first scoring pass (`lKFsSharingWords` loop), store computed scores into `queryScores[pKFi] = si;`.
2. In the covisibility neighbor accumulation loop (`vpNeighs`):
   * Lookup `itScore = queryScores.find(pKF2);`
   * If found: `accScore += itScore->second;`
   * If not found:
     ```cpp
     const float si = mpVoc->score(pKF->mBowVec, pKF2->mBowVec);
     queryScores.emplace(pKF2, si);
     accScore += si;
     ```
3. Update `pBestKF` and `bestScore` using the score from `queryScores`.
4. Leave `pKF2->mPlaceRecognitionScore` and `pKF2->mnPlaceRecognitionWords` untouched.

*Thread Safety Proof:* `queryScores` is purely local to the calling thread. No shared `KeyFrame` state is modified.

---

### 3.2. `Tracking`: KeyFrame-Gated Map Event Publishing

#### Problem
In `Tracking::PublishVisualizationState()`, lines 2412–2437:
On **every frame** (30 Hz) when a web client is connected, the tracking thread calls `pMap->GetAllMapPoints()`, locking `mMutexMap` and copying thousands of map points into a temporary vector, then locks each point's `mMutexPos` to call `GetWorldPos()`. On maps with 20k–50k points, this causes tracking frame drops and lock contention against `LocalMapping`.

#### Solution
1. Introduce a boolean dirty flag in `Tracking`:
   ```cpp
   std::atomic<bool> mbMapUpdatedForVisualizer{true};
   ```
2. Set `mbMapUpdatedForVisualizer.store(true, std::memory_order_relaxed);`:
   - Inside `Tracking::CreateNewKeyFrame()` upon successfully inserting a new KeyFrame.
   - When tracking state changes to `LOST` / `OK` or after a map reset / merge.
3. In `Tracking::PublishVisualizationState()`:
   - Always publish `VisualizationFrameSnapshot` (camera pose) and `VisualizationImageSnapshot` (2D tracked features) at **30 FPS**.
   - Check `mbMapUpdatedForVisualizer.exchange(false)`:
     - Only execute `pMap->GetAllMapPoints()` and publish `VisualizationEventType::POINTS_UPDATED` when `mbMapUpdatedForVisualizer` was `true`.
4. *Result:* Full map updates scale with keyframe creation rate (~1–2 Hz) rather than video frame rate (30 Hz), reducing lock acquisitions by ~95% while keeping browser 3D point clouds fully synchronized.

---

### 3.3. `Map`: Bounded Tombstone Ring Buffer

#### Problem
In `Map::EraseKeyFrame(KeyFrame* pKF)`:
```cpp
mErasedKeyframeSnapshots[pKF->mnId] = CaptureKeyframeSnapshot(pKF, true, nullptr);
```
Every culled keyframe is permanently retained in `mErasedKeyframeSnapshots`. On multi-hour runs with thousands of keyframes culled by local mapping, memory grows monotonically.

#### Solution
Enforce an eviction limit:
```cpp
static constexpr size_t kMaxErasedTombstones = 250;
```
Inside `Map::EraseKeyFrame(KeyFrame* pKF)` (which already holds `mMutexMap`):
```cpp
mErasedKeyframeSnapshots[pKF->mnId] = CaptureKeyframeSnapshot(pKF, true, nullptr);
if (mErasedKeyframeSnapshots.size() > kMaxErasedTombstones) {
    // std::map is sorted by key (KeyFrame ID); erase the oldest tombstone
    mErasedKeyframeSnapshots.erase(mErasedKeyframeSnapshots.begin());
}
```
*Result:* Memory consumption is strictly bounded $O(1)$. External visualizers still receive tombstones for the latest 250 culled keyframes, which is more than sufficient for delta reconciliation.

---

### 3.4. `System::Cleanup()`: Join Timeout & Diagnostic Reporting

#### Problem
In `System::Cleanup()`, `worker->join()` blocks indefinitely if `LocalMapping` or `LoopClosing` deadlocks internally (e.g. spinning on `isStopped()`).

#### Solution
Before calling blocking `worker->join()`, check worker liveness or perform a bounded wait. If a worker thread fails to exit within 5.0 seconds after `RequestFinish()`:
1. Log an explicit diagnostic: `ERROR: Subsystem thread did not exit within 5s shutdown timeout; possible deadlock detected.`
2. Capture diagnostic logs if `CrashMonitor` is installed.
3. Proceed safely with teardown without hanging the terminal process.

---

### 3.5. Toolchain: Nix Flake GTest Support

#### Problem
Configuring CMake with `-DBUILD_TESTS=ON` fails with:
`Could NOT find GTest (missing: GTEST_LIBRARY GTEST_INCLUDE_DIR GTEST_MAIN_LIBRARY)`

#### Solution
In `flake.nix`:
Add `pkgs.gtest` to `buildInputs`. Update `flake.lock`.

---

## 4. Verification & Validation Strategy

1. **Compilation Check:** Run `nix develop --command make -j$(nproc)` to ensure clean build with zero warnings or errors.
2. **Test Suite Verification:** Run `cmake -DBUILD_TESTS=ON .. && make -j4 && ctest --output-on-failure` to verify unit and regression tests pass.
3. **Concurrency Race Check:** Verify that `KeyFrameDatabase::DetectNBestCandidates()` runs with thread-sanitizer clean semantics (no writes to `KeyFrame` fields).
4. **Visualizer Performance Benchmark:** Run a stereo sequence with web visualizer enabled; measure tracking loop frame rate to verify 30 FPS without lock spikes.
