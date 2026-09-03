#ifndef WEB_VIEWER_PROTOCOL_H
#define WEB_VIEWER_PROTOCOL_H

#include "VisualizationTypes.h"
#include <vector>
#include <cstdint>
#include <string>

namespace ORB_SLAM3 {

constexpr std::uint32_t PROTOCOL_MAGIC = 0x4f524257; // "ORBW"
constexpr std::uint16_t PROTOCOL_VERSION = 0x0100;    // v1.0

enum MessageType : std::uint16_t {
    MSG_SERVER_HELLO = 1,
    MSG_FRAME_STATE = 2,
    MSG_STATISTICS = 3,
    MSG_EPOCH_RESET = 4,
    MSG_HEARTBEAT = 5,
    MSG_ERROR_NOTICE = 6,
    MSG_GRAPH_FULL = 20,
    MSG_GRAPH_DELTA = 21,
    MSG_POINTS_FULL_BEGIN = 30,
    MSG_POINTS_FULL_CHUNK = 31,
    MSG_POINTS_FULL_END = 32,
    MSG_POINTS_DELTA = 33,
    MSG_IMAGE_JPEG = 40,
    MSG_FEATURE_OVERLAY = 41
};

#pragma pack(push, 1)
struct WireHeader {
    std::uint32_t magic;
    std::uint16_t version;
    std::uint16_t msg_type;
    std::uint32_t flags;
    std::uint32_t payload_size;
    std::uint64_t epoch;
    std::uint64_t revision;
    std::uint64_t sequence;
    std::int64_t capture_timestamp_ns;
};
#pragma pack(pop)

class WebViewerProtocol {
public:
    static std::vector<uint8_t> EncodeHeader(uint16_t msg_type, uint32_t payload_size, uint64_t epoch, uint64_t revision, uint64_t sequence, int64_t ts_ns);
    static bool DecodeHeader(const uint8_t* data, size_t size, WireHeader& out_header);

    static std::vector<uint8_t> EncodeFrameState(const VisualizationFrameSnapshot& frame);
    static std::vector<uint8_t> EncodePointsChunk(uint64_t epoch, uint64_t revision, const std::vector<VisualizationMapPoint>& points, float scale = 0.002f);
    static std::vector<uint8_t> EncodeImageJpeg(uint64_t epoch, uint64_t sequence, int64_t ts_ns, const cv::Mat& gray_img, int quality = 55);
    static std::vector<uint8_t> EncodeFeatureOverlay(uint64_t epoch, uint64_t sequence, int64_t ts_ns, const std::vector<VisualizationFeature>& features);
};

} // namespace ORB_SLAM3

#endif // WEB_VIEWER_PROTOCOL_H
