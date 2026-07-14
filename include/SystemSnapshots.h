/**
* This file is part of ORB-SLAM3.
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
* José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the
* terms of the GNU General Public License as published by the Free Software
* Foundation, either version 3 of the License, or (at your option) any later version.
*/

#ifndef SYSTEM_SNAPSHOTS_H
#define SYSTEM_SNAPSHOTS_H

#include <cstddef>
#include <cstdint>
#include <set>
#include <vector>

#include <sophus/se3.hpp>

namespace ORB_SLAM3
{

struct KeyframeSnapshot
{
    std::uint64_t id{};
    std::uint64_t map_id{};
    double timestamp{};
    Sophus::SE3f T_world_camera;
    bool bad{};
    bool has_parent{};
    std::uint64_t parent_id{};
    std::vector<std::uint64_t> loop_edge_ids;
};

struct FrameSnapshot
{
    double timestamp{};
    int tracking_state{};
    bool pose_valid{};
    std::uint64_t map_id{};
    std::uint64_t reference_keyframe_id{};
    Sophus::SE3f T_world_camera;
    Sophus::SE3f T_reference_camera_current_camera;
    std::size_t tracked_keypoints{};
};

struct GraphSnapshot
{
    std::uint64_t revision{};
    std::uint64_t active_map_id{};
    bool active_map_connected{};
    std::vector<KeyframeSnapshot> keyframes;
};

// Value-only bookkeeping shared by System and the snapshot tests. An epoch
// intentionally drops all prior connectivity when an atlas reset/load is requested.
class SnapshotGraphState
{
public:
    void BeginNewEpoch()
    {
        ++mEpoch;
        mHasLastMap = false;
        mHasRootMap = false;
        mConnectedMapIds.clear();
    }

    std::uint64_t Observe(std::uint64_t map_id, int big_change_index)
    {
        if(!mHasLastMap || map_id != mLastMapId || big_change_index != mLastBigChangeIndex)
            ++mRevision;
        mHasLastMap = true;
        mLastMapId = map_id;
        mLastBigChangeIndex = big_change_index;
        if(!mHasRootMap)
        {
            mHasRootMap = true;
            mConnectedMapIds.insert(map_id);
        }
        return mRevision;
    }

    void MarkConnected(std::uint64_t map_id) { mConnectedMapIds.insert(map_id); }
    bool IsConnected(std::uint64_t map_id) const
    {
        return mConnectedMapIds.count(map_id) != 0;
    }
    std::uint64_t Epoch() const { return mEpoch; }

private:
    std::uint64_t mEpoch{};
    std::uint64_t mRevision{};
    std::uint64_t mLastMapId{};
    int mLastBigChangeIndex{};
    bool mHasLastMap{};
    bool mHasRootMap{};
    std::set<std::uint64_t> mConnectedMapIds;
};

} // namespace ORB_SLAM3

#endif // SYSTEM_SNAPSHOTS_H
