# ORB-SLAM3

## Fork improvements

This fork retains the upstream ORB-SLAM3 V1.0 algorithms and adds the following improvements for library integration, long-running deployments, and loop-closure robustness:

- **Safe, value-based snapshots:** `System::GetLastFrameSnapshot()` and `System::GetGraphSnapshot()` expose frame state and keyframe-graph data without leaking ORB-SLAM3 object pointers. Graph snapshots include map identity, a revision number, keyframe poses, parent/loop edges, and tombstones for culled keyframes so external consumers can retain a consistent graph view.
- **Lifecycle and embedding safety:** construction failures and invalid settings/sensor API calls raise C++ exceptions instead of terminating the host process. `System` now performs ownership-safe shutdown and destruction, including cleanup of partially constructed instances and joining background workers in headless operation.
- **Snapshot concurrency safeguards:** snapshot collection stabilizes the active map while copying state, tracks reset/load epochs, and reports loop-closure graph changes only after their new edges are observable.
- **Multi-lap loop closure:** covisibility-connected candidates are no longer always rejected. Candidates separated by at least 20 keyframe IDs remain eligible for normal geometric verification, allowing revisits on later laps while filtering nearby trajectory neighbors.
- **Merged correctness fixes:** fixes prevent duplicate triangulation matches, skip map points already matched in the current projection frame, repair Sim3 matched-keyframe selection, keep covisibility connections bidirectional, harden place-recognition candidate selection/scoring, and make atlas pre-save iteration safe while observations or map points are removed.
- **Development-only test seams:** when compiled with `ORB_SLAM3_SNAPSHOT_TESTING`, optional hooks support deterministic tests for snapshot and keyframe-lock ordering behavior.
