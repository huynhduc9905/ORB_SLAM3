#ifndef LOOP_REVISIT_POLICY_H
#define LOOP_REVISIT_POLICY_H

// Local fork addition. Decides whether a covisibility-connected keyframe should
// be excluded as a loop-closure candidate. Original ORB-SLAM3 excludes every
// connected keyframe, which prevents loop closure when the robot re-drives a
// previously mapped area on a later lap (the revisited keyframes become
// covisibility-connected). We exclude a connected keyframe only when it is a
// recent trajectory neighbor, measured by keyframe-id gap; distant connected
// keyframes are genuine revisits and remain eligible for the normal geometric
// (Sim3 + coincidence) verification downstream.

namespace orb_slam3_wrapper_fork {

// Minimum keyframe-id gap for a covisibility-connected keyframe to still be
// eligible as a loop-closure candidate. Below this gap the connected keyframe
// is treated as an immediate trajectory neighbor and excluded (original
// behavior). At or above it, the keyframe is treated as a genuine revisit.
constexpr unsigned long kLoopRevisitMinKFGap = 20;

// Returns true when the connected candidate should be EXCLUDED as a loop
// candidate (recent trajectory neighbor); false when it should be ALLOWED
// (distant revisit, or a future/equal id that must not underflow).
inline bool shouldExcludeConnectedLoopCandidate(unsigned long current_id,
                                                unsigned long candidate_id) {
  if (current_id < candidate_id) {
    return false;
  }
  return (current_id - candidate_id) < kLoopRevisitMinKFGap;
}

}  // namespace orb_slam3_wrapper_fork

#endif  // LOOP_REVISIT_POLICY_H
