#include "WebViewerProtocol.h"
#include <cstring>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>

namespace ORB_SLAM3 {

std::vector<uint8_t> WebViewerProtocol::EncodeHeader(uint16_t msg_type, uint32_t payload_size, uint64_t epoch, uint64_t revision, uint64_t sequence, int64_t ts_ns) {
    WireHeader header;
    header.magic = PROTOCOL_MAGIC;
    header.version = PROTOCOL_VERSION;
    header.msg_type = msg_type;
    header.flags = 0;
    header.payload_size = payload_size;
    header.epoch = epoch;
    header.revision = revision;
    header.sequence = sequence;
    header.capture_timestamp_ns = ts_ns;

    std::vector<uint8_t> buf(sizeof(WireHeader));
    std::memcpy(buf.data(), &header, sizeof(WireHeader));
    return buf;
}

bool WebViewerProtocol::DecodeHeader(const uint8_t* data, size_t size, WireHeader& out_header) {
    if (size < sizeof(WireHeader)) return false;
    std::memcpy(&out_header, data, sizeof(WireHeader));
    return (out_header.magic == PROTOCOL_MAGIC);
}

std::vector<uint8_t> WebViewerProtocol::EncodeFrameState(const VisualizationFrameSnapshot& frame) {
    // Payload layout:
    // uint64_t map_id
    // uint64_t reference_keyframe_id
    // int32_t tracking_state
    // uint8_t pose_valid
    // float tx, ty, tz, qx, qy, qz, qw
    // uint32_t tracked_keypoints
    // uint32_t tracked_map_points

    Eigen::Vector3f t = frame.T_world_camera.translation();
    Eigen::Quaternionf q = frame.T_world_camera.unit_quaternion();

    uint32_t payload_size = 8 + 8 + 4 + 1 + (7 * 4) + 4 + 4;
    auto buf = EncodeHeader(MSG_FRAME_STATE, payload_size, frame.epoch, 0, frame.sequence, frame.capture_timestamp_ns);

    size_t offset = buf.size();
    buf.resize(offset + payload_size);
    uint8_t* ptr = buf.data() + offset;

    uint64_t map_id = frame.map_id;
    uint64_t ref_kf_id = frame.reference_keyframe_id;
    int32_t tr_state = frame.tracking_state;
    uint8_t pose_valid = frame.pose_valid ? 1 : 0;
    float tx = t.x(), ty = t.y(), tz = t.z();
    float qx = q.x(), qy = q.y(), qz = q.z(), qw = q.w();
    uint32_t tr_kp = frame.tracked_keypoints;
    uint32_t tr_mp = frame.tracked_map_points;

    std::memcpy(ptr, &map_id, 8); ptr += 8;
    std::memcpy(ptr, &ref_kf_id, 8); ptr += 8;
    std::memcpy(ptr, &tr_state, 4); ptr += 4;
    std::memcpy(ptr, &pose_valid, 1); ptr += 1;
    std::memcpy(ptr, &tx, 4); ptr += 4;
    std::memcpy(ptr, &ty, 4); ptr += 4;
    std::memcpy(ptr, &tz, 4); ptr += 4;
    std::memcpy(ptr, &qx, 4); ptr += 4;
    std::memcpy(ptr, &qy, 4); ptr += 4;
    std::memcpy(ptr, &qz, 4); ptr += 4;
    std::memcpy(ptr, &qw, 4); ptr += 4;
    std::memcpy(ptr, &tr_kp, 4); ptr += 4;
    std::memcpy(ptr, &tr_mp, 4); ptr += 4;

    return buf;
}

std::vector<uint8_t> WebViewerProtocol::EncodePointsChunk(uint64_t epoch, uint64_t revision, const std::vector<VisualizationMapPoint>& points, float scale) {
    if (points.empty()) {
        return EncodeHeader(MSG_POINTS_FULL_CHUNK, 0, epoch, revision, 0, 0);
    }

    // Calculate origin (mean)
    Eigen::Vector3f origin(0, 0, 0);
    for (const auto& p : points) {
        origin += p.world_position;
    }
    origin /= static_cast<float>(points.size());

    // Calculate max range from origin to determine dynamic scale factor
    float max_dist = 0.001f;
    for (const auto& p : points) {
        float dx = std::abs(p.world_position.x() - origin.x());
        float dy = std::abs(p.world_position.y() - origin.y());
        float dz = std::abs(p.world_position.z() - origin.z());
        max_dist = std::max({max_dist, dx, dy, dz});
    }
    float computed_scale = std::max(0.002f, max_dist / 32000.0f);

    uint32_t point_count = static_cast<uint32_t>(points.size());
    // Payload: origin (3x float), scale (float), point_count (uint32), per point: uint32 id, int16 dx, dy, dz, uint16 flags
    uint32_t payload_size = 12 + 4 + 4 + (point_count * 12);

    auto buf = EncodeHeader(MSG_POINTS_FULL_CHUNK, payload_size, epoch, revision, 0, 0);
    size_t offset = buf.size();
    buf.resize(offset + payload_size);
    uint8_t* ptr = buf.data() + offset;

    float ox = origin.x(), oy = origin.y(), oz = origin.z();
    std::memcpy(ptr, &ox, 4); ptr += 4;
    std::memcpy(ptr, &oy, 4); ptr += 4;
    std::memcpy(ptr, &oz, 4); ptr += 4;
    std::memcpy(ptr, &computed_scale, 4); ptr += 4;
    std::memcpy(ptr, &point_count, 4); ptr += 4;

    for (const auto& p : points) {
        uint32_t render_id = static_cast<uint32_t>(p.id);
        int16_t dx = static_cast<int16_t>(std::clamp((p.world_position.x() - ox) / computed_scale, -32768.0f, 32767.0f));
        int16_t dy = static_cast<int16_t>(std::clamp((p.world_position.y() - oy) / computed_scale, -32768.0f, 32767.0f));
        int16_t dz = static_cast<int16_t>(std::clamp((p.world_position.z() - oz) / computed_scale, -32768.0f, 32767.0f));
        uint16_t flags = p.reference ? 1 : 0;

        std::memcpy(ptr, &render_id, 4); ptr += 4;
        std::memcpy(ptr, &dx, 2); ptr += 2;
        std::memcpy(ptr, &dy, 2); ptr += 2;
        std::memcpy(ptr, &dz, 2); ptr += 2;
        std::memcpy(ptr, &flags, 2); ptr += 2;
    }

    return buf;
}

std::vector<uint8_t> WebViewerProtocol::EncodeImageJpeg(uint64_t epoch, uint64_t sequence, int64_t ts_ns, const cv::Mat& gray_img, int quality) {
    if (gray_img.empty() || gray_img.cols <= 0 || gray_img.rows <= 0) {
        return {};
    }

    try {
        cv::Mat resized;
        if (gray_img.cols > 640) {
            int target_w = 640;
            int target_h = gray_img.rows * target_w / gray_img.cols;
            if (target_h <= 0) target_h = 1;
            cv::resize(gray_img, resized, cv::Size(target_w, target_h));
        } else {
            resized = gray_img;
        }

        std::vector<uchar> jpeg_bytes;
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, quality};
        if (!cv::imencode(".jpg", resized, jpeg_bytes, params) || jpeg_bytes.empty()) {
            return {};
        }

        uint32_t payload_size = static_cast<uint32_t>(jpeg_bytes.size());
        auto buf = EncodeHeader(MSG_IMAGE_JPEG, payload_size, epoch, 0, sequence, ts_ns);
        size_t offset = buf.size();
        buf.resize(offset + payload_size);
        std::memcpy(buf.data() + offset, jpeg_bytes.data(), jpeg_bytes.size());

        return buf;
    } catch (const std::exception& e) {
        std::cerr << "[WebViewerProtocol] Exception encoding JPEG: " << e.what() << std::endl;
        return {};
    } catch (...) {
        std::cerr << "[WebViewerProtocol] Unknown exception encoding JPEG" << std::endl;
        return {};
    }
}

std::vector<uint8_t> WebViewerProtocol::EncodeFeatureOverlay(uint64_t epoch, uint64_t sequence, int64_t ts_ns, const std::vector<VisualizationFeature>& features) {
    uint32_t count = static_cast<uint32_t>(features.size());
    uint32_t payload_size = 4 + (count * 9); // count (uint32) + per feature: float x, float y, uint8 state

    auto buf = EncodeHeader(MSG_FEATURE_OVERLAY, payload_size, epoch, 0, sequence, ts_ns);
    size_t offset = buf.size();
    buf.resize(offset + payload_size);
    uint8_t* ptr = buf.data() + offset;

    std::memcpy(ptr, &count, 4); ptr += 4;
    for (const auto& feat : features) {
        float x = feat.x;
        float y = feat.y;
        uint8_t st = feat.state;
        std::memcpy(ptr, &x, 4); ptr += 4;
        std::memcpy(ptr, &y, 4); ptr += 4;
        std::memcpy(ptr, &st, 1); ptr += 1;
    }

    return buf;
}

} // namespace ORB_SLAM3
