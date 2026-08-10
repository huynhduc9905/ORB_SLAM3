#ifndef WEB_VIEWER_BACKEND_H
#define WEB_VIEWER_BACKEND_H

#include "VisualizationSource.h"
#include "WebViewerProtocol.h"
#include <string>
#include <memory>
#include <thread>
#include <atomic>
#include <unordered_map>
#include <vector>
#include <mutex>

namespace ORB_SLAM3 {

struct WebViewerConfig {
    bool enabled{true};
    std::string bind_address{"0.0.0.0"};
    int port{8080};
    std::string static_root{"./web_viewer/dist"};
    int max_clients{4};
    int image_fps{5};
    int image_jpeg_quality{55};
};

class WebViewerBackendImpl;

class WebViewerBackend {
public:
    WebViewerBackend(std::shared_ptr<VisualizationSource> source, const WebViewerConfig& config);
    ~WebViewerBackend();

    bool Start();
    void Stop();
    bool IsRunning() const { return mRunning.load(); }

private:
    void MirrorWorkerLoop();

    std::shared_ptr<VisualizationSource> mpSource;
    WebViewerConfig mConfig;
    std::atomic<bool> mRunning{false};
    std::thread mMirrorThread;

    std::unique_ptr<WebViewerBackendImpl> mImpl;

    // Backend Map Mirror
    struct MirrorMapPoint {
        std::uint64_t id;
        Eigen::Vector3f pos;
        bool reference;
    };
    std::mutex mMirrorMutex;
    std::unordered_map<std::uint64_t, MirrorMapPoint> mMirrorPoints;
};

} // namespace ORB_SLAM3

#endif // WEB_VIEWER_BACKEND_H
