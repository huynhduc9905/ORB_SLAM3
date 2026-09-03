#ifndef VISUALIZATION_TYPES_H
#define VISUALIZATION_TYPES_H

#include <cstdint>
#include <vector>
#include <memory>
#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <opencv2/core.hpp>

namespace ORB_SLAM3 {

enum class VisualizationEventType {
    POINTS_ADDED,
    POINTS_REMOVED,
    POINTS_UPDATED,
    KEYFRAMES_ADDED,
    KEYFRAMES_REMOVED,
    KEYFRAMES_UPDATED,
    GRAPH_UPDATED,
    MAP_REPLACED,
    EPOCH_RESET
};

struct VisualizationFrameSnapshot {
    std::uint64_t epoch{0};
    std::uint64_t sequence{0};
    std::int64_t capture_timestamp_ns{0};
    std::uint64_t map_id{0};
    std::uint64_t reference_keyframe_id{0};
    int tracking_state{0};
    bool pose_valid{false};
    Sophus::SE3f T_world_camera;
    std::uint32_t tracked_keypoints{0};
    std::uint32_t tracked_map_points{0};
};

struct VisualizationFeature {
    float x;
    float y;
    std::uint8_t state; // 1=VO match, 2=Map match, 3=Outlier
};

struct VisualizationImageSnapshot {
    std::uint64_t epoch{0};
    std::uint64_t frame_sequence{0};
    std::int64_t capture_timestamp_ns{0};
    cv::Mat immutable_grayscale_image;
    std::vector<VisualizationFeature> features;
};

struct VisualizationMapPoint {
    std::uint64_t id{0};
    Eigen::Vector3f world_position;
    bool reference{false};
};

struct VisualizationKeyframe {
    std::uint64_t id{0};
    double timestamp{0.0};
    Sophus::SE3f T_world_camera;
    bool bad{false};
    bool has_parent{false};
    std::uint64_t parent_id{0};
    std::vector<std::uint64_t> loop_edges;
    std::vector<std::uint64_t> merge_edges;
};

struct VisualizationMapSnapshot {
    std::uint64_t epoch{0};
    std::uint64_t map_id{0};
    std::uint64_t topology_revision{0};
    std::uint64_t geometry_revision{0};
    std::uint64_t graph_revision{0};
    std::vector<VisualizationMapPoint> points;
    std::vector<VisualizationKeyframe> keyframes;
};

struct VisualizationMapEvent {
    std::uint64_t epoch{0};
    std::uint64_t topology_revision{0};
    std::uint64_t geometry_revision{0};
    std::uint64_t graph_revision{0};
    VisualizationEventType type;
    // When true, `points` is the complete, authoritative set of live map
    // points (not an incremental delta). Consumers must replace their mirror
    // wholesale so that points which have since been culled or fused (e.g. by
    // loop-closure map fusion) are dropped rather than lingering as ghosts.
    bool full_snapshot{false};
    std::vector<VisualizationMapPoint> points;
    std::vector<VisualizationKeyframe> keyframes;
};

} // namespace ORB_SLAM3

#endif // VISUALIZATION_TYPES_H
