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

class Map;

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

// Internal map-owned copy used to build a GraphSnapshot without retaining
// KeyFrame addresses after the Map lock is released.
struct MapGraphSnapshot
{
    std::uint64_t map_id{};
    int big_change_index{};
    std::vector<KeyframeSnapshot> keyframes;
    std::set<std::uint64_t> merge_edge_map_ids;
};

// Value-only bookkeeping shared by System and the snapshot tests. An epoch
// intentionally drops all prior connectivity when an atlas reset/load is requested.
class SnapshotGraphState
{
public:
    enum class EpochKind
    {
        FULL_RESET,
        ACTIVE_MAP_RESET,
        ATLAS_LOAD
    };

    void SetInitialRootMap(const Map& map)
    {
        mInitialRootMapIdentity = &map;
        mConnectFirstObservedMap = false;
    }

    void BeginNewEpoch(EpochKind kind)
    {
        ++mEpoch;
        mHasLastMap = false;
        mLastMapIdentity = nullptr;
        mInitialRootMapIdentity = nullptr;
        mConnectFirstObservedMap = kind == EpochKind::FULL_RESET;
        mConnectedMapIds.clear();
    }

    std::uint64_t Observe(const Map& map, std::uint64_t map_id, int big_change_index)
    {
        if(!mHasLastMap || &map != mLastMapIdentity || map_id != mLastMapId ||
           big_change_index != mLastBigChangeIndex)
            ++mRevision;
        mHasLastMap = true;
        mLastMapIdentity = &map;
        mLastMapId = map_id;
        mLastBigChangeIndex = big_change_index;
        if(mConnectFirstObservedMap || &map == mInitialRootMapIdentity)
        {
            mConnectedMapIds.insert(map_id);
            mConnectFirstObservedMap = false;
            mInitialRootMapIdentity = nullptr;
        }
        return mRevision;
    }

    void MarkConnected(const Map& map) { mConnectedMapIds.insert(MapId(map)); }
    bool IsConnected(const Map& map) const { return IsConnected(MapId(map)); }
    bool IsConnected(std::uint64_t map_id) const
    {
        return mConnectedMapIds.count(map_id) != 0;
    }
    std::uint64_t Epoch() const { return mEpoch; }

private:
    static std::uint64_t MapId(const Map& map);

    std::uint64_t mEpoch{};
    std::uint64_t mRevision{};
    std::uint64_t mLastMapId{};
    int mLastBigChangeIndex{};
    bool mHasLastMap{};
    bool mConnectFirstObservedMap{true};
    const Map* mLastMapIdentity{};
    const Map* mInitialRootMapIdentity{};
    std::set<std::uint64_t> mConnectedMapIds;
};

} // namespace ORB_SLAM3

#endif // SYSTEM_SNAPSHOTS_H
