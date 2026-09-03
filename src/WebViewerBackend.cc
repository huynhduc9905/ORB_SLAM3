#include "WebViewerBackend.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cstring>
#include <filesystem>
#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <boost/beast/websocket.hpp>

namespace beast = boost::beast;
namespace http = beast::http;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = net::ip::tcp;

namespace ORB_SLAM3 {

namespace {

// Resolve an HTTP request target to a filesystem path that is provably inside
// static_root, or return empty if the request tries to escape (path traversal).
// The request target is attacker-controlled, so we strip any query/fragment,
// reject NUL bytes, then canonicalize and require the result to stay under the
// canonical static root. Without this, "GET /../../etc/passwd" would read
// arbitrary files the process can access -- especially dangerous now that the
// server can be bound to a non-loopback (e.g. tailnet) address.
std::string ResolveStaticPath(const std::string& static_root, std::string target) {
    if (target.find('\0') != std::string::npos) return {};
    // Drop query string / fragment.
    const auto qpos = target.find_first_of("?#");
    if (qpos != std::string::npos) target = target.substr(0, qpos);
    if (target.empty() || target == "/") target = "/index.html";

    namespace fs = std::filesystem;
    std::error_code ec;
    const fs::path root = fs::weakly_canonical(fs::path(static_root), ec);
    if (ec) return {};
    const fs::path candidate = fs::weakly_canonical(root / fs::path("." + target), ec);
    if (ec) return {};

    // Require candidate to be root itself or strictly under root.
    auto rit = root.begin();
    auto cit = candidate.begin();
    for (; rit != root.end(); ++rit, ++cit) {
        if (cit == candidate.end() || *cit != *rit) return {};
    }
    return candidate.string();
}

std::string MimeTypeFor(const std::string& path) {
    auto ends_with = [&](const char* ext) {
        const size_t n = std::strlen(ext);
        return path.size() >= n && path.compare(path.size() - n, n, ext) == 0;
    };
    if (ends_with(".html")) return "text/html";
    if (ends_with(".js"))   return "application/javascript";
    if (ends_with(".css"))  return "text/css";
    if (ends_with(".json")) return "application/json";
    if (ends_with(".svg"))  return "image/svg+xml";
    if (ends_with(".png"))  return "image/png";
    if (ends_with(".jpg") || ends_with(".jpeg")) return "image/jpeg";
    if (ends_with(".woff2")) return "font/woff2";
    if (ends_with(".wasm")) return "application/wasm";
    return "application/octet-stream";
}

} // namespace

class WebViewerBackendImpl {
public:
    WebViewerBackendImpl(std::shared_ptr<VisualizationSource> source, const WebViewerConfig& config)
        : mpSource(source), mConfig(config), mAcceptor(mIoc) {}

    void Start() {
        try {
            tcp::endpoint endpoint(net::ip::make_address(mConfig.bind_address), mConfig.port);
            mAcceptor.open(endpoint.protocol());
            mAcceptor.set_option(net::socket_base::reuse_address(true));
            mAcceptor.bind(endpoint);
            mAcceptor.listen();

            mRunning = true;
            mServerThread = std::thread([this]() { RunServer(); });
            std::cout << "[WebViewer] Server listening on " << mConfig.bind_address << ":" << mConfig.port << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "[WebViewer] Failed to start server: " << e.what() << std::endl;
        }
    }

    void Stop() {
        mRunning = false;
        boost::system::error_code ec;
        mAcceptor.close(ec);
        mIoc.stop();
        if (mServerThread.joinable()) {
            mServerThread.join();
        }
    }

    void BroadcastRealtime(const std::vector<uint8_t>& msg) {
        std::lock_guard<std::mutex> lock(mSocketsMutex);
        BroadcastAndPrune(mRealtimeSockets, msg);
    }

    void BroadcastBulk(const std::vector<uint8_t>& msg) {
        std::lock_guard<std::mutex> lock(mSocketsMutex);
        BroadcastAndPrune(mBulkSockets, msg);
    }

    bool HasClients() {
        std::lock_guard<std::mutex> lock(mSocketsMutex);
        return !mRealtimeSockets.empty() || !mBulkSockets.empty();
    }

private:
    // Broadcast to every socket; drop any that throw (client disconnected) so
    // the vectors don't grow without bound over a long-running deployment.
    // Caller must hold mSocketsMutex.
    static void BroadcastAndPrune(
            std::vector<std::shared_ptr<websocket::stream<tcp::socket>>>& sockets,
            const std::vector<uint8_t>& msg) {
        sockets.erase(
            std::remove_if(sockets.begin(), sockets.end(),
                [&](std::shared_ptr<websocket::stream<tcp::socket>>& ws) {
                    try {
                        ws->binary(true);
                        ws->write(net::buffer(msg));
                        return false; // keep
                    } catch (...) {
                        return true;  // dead -> prune
                    }
                }),
            sockets.end());
    }

    void RunServer() {
        while (mRunning) {
            try {
                tcp::socket socket(mIoc);
                boost::system::error_code ec;
                mAcceptor.accept(socket, ec);
                if (ec) break;

                std::thread([this, s = std::move(socket)]() mutable { HandleConnection(std::move(s)); }).detach();
            } catch (...) {
                break;
            }
        }
    }

    void HandleConnection(tcp::socket socket) {
        beast::flat_buffer buffer;
        http::request<http::string_body> req;
        boost::system::error_code ec;
        http::read(socket, buffer, req, ec);
        if (ec) return;

        if (websocket::is_upgrade(req)) {
            std::string target(req.target());
            auto ws = std::make_shared<websocket::stream<tcp::socket>>(std::move(socket));
            ws->accept(req, ec);
            if (ec) return;

            std::lock_guard<std::mutex> lock(mSocketsMutex);
            if (target == "/ws/realtime") {
                mRealtimeSockets.push_back(ws);
                std::cout << "[WebViewer] Realtime client connected." << std::endl;
                // Send hello
                auto hello = WebViewerProtocol::EncodeHeader(MSG_SERVER_HELLO, 0, mpSource->GetCurrentEpoch(), 0, 0, 0);
                ws->binary(true);
                ws->write(net::buffer(hello));
            } else if (target == "/ws/bulk") {
                mBulkSockets.push_back(ws);
                std::cout << "[WebViewer] Bulk client connected." << std::endl;
            }
        } else {
            // Serve HTTP
            std::string target(req.target());
            if (target == "/healthz") {
                http::response<http::string_body> res{http::status::ok, req.version()};
                res.set(http::field::content_type, "application/json");
                res.body() = "{\"status\":\"ok\",\"clients\":0}";
                res.prepare_payload();
                http::write(socket, res);
            } else if (target == "/config") {
                http::response<http::string_body> res{http::status::ok, req.version()};
                res.set(http::field::content_type, "application/json");
                res.body() = "{\"enabled\":true,\"port\":" + std::to_string(mConfig.port) + "}";
                res.prepare_payload();
                http::write(socket, res);
            } else {
                // Static file serving (path-traversal safe).
                std::string path = ResolveStaticPath(mConfig.static_root, target);
                std::ifstream ifs(path.empty() ? std::string() : path, std::ios::binary);
                if (!path.empty() && ifs) {
                    std::string body((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
                    http::response<http::string_body> res{http::status::ok, req.version()};
                    res.set(http::field::content_type, MimeTypeFor(path));
                    res.body() = body;
                    res.prepare_payload();
                    http::write(socket, res);
                } else {
                    http::response<http::string_body> res{http::status::not_found, req.version()};
                    res.body() = "404 Not Found";
                    res.prepare_payload();
                    http::write(socket, res);
                }
            }
        }
    }

    std::shared_ptr<VisualizationSource> mpSource;
    WebViewerConfig mConfig;
    net::io_context mIoc;
    tcp::acceptor mAcceptor;
    std::atomic<bool> mRunning{false};
    std::thread mServerThread;

    std::mutex mSocketsMutex;
    std::vector<std::shared_ptr<websocket::stream<tcp::socket>>> mRealtimeSockets;
    std::vector<std::shared_ptr<websocket::stream<tcp::socket>>> mBulkSockets;
};

WebViewerBackend::WebViewerBackend(std::shared_ptr<VisualizationSource> source, const WebViewerConfig& config)
    : mpSource(source), mConfig(config) {
    mImpl = std::make_unique<WebViewerBackendImpl>(mpSource, mConfig);
}

WebViewerBackend::~WebViewerBackend() {
    Stop();
}

bool WebViewerBackend::Start() {
    if (mRunning.exchange(true)) return true;
    mImpl->Start();
    mMirrorThread = std::thread([this]() { MirrorWorkerLoop(); });
    return true;
}

void WebViewerBackend::Stop() {
    if (!mRunning.exchange(false)) return;
    mImpl->Stop();
    if (mMirrorThread.joinable()) {
        mMirrorThread.join();
    }
}

void WebViewerBackend::ReconcileMirror(std::unordered_map<std::uint64_t, MirrorMapPoint>& mirror,
                                       const VisualizationMapEvent& ev) {
    switch (ev.type) {
        case VisualizationEventType::POINTS_ADDED:
        case VisualizationEventType::POINTS_UPDATED:
            // A full snapshot is authoritative: rebuild from scratch so points
            // culled or fused since the last snapshot (e.g. by loop-closure map
            // fusion) are dropped instead of lingering as ghost duplicates.
            if (ev.full_snapshot) {
                mirror.clear();
            }
            for (const auto& pt : ev.points) {
                mirror[pt.id] = MirrorMapPoint{pt.id, pt.world_position, pt.reference};
            }
            break;
        case VisualizationEventType::POINTS_REMOVED:
            for (const auto& pt : ev.points) {
                mirror.erase(pt.id);
            }
            break;
        default:
            break;
    }
}

void WebViewerBackend::MirrorWorkerLoop() {
    while (mRunning) {
        std::this_thread::sleep_for(std::chrono::milliseconds(33)); // ~30 Hz loop

        if (mpSource) {
            mpSource->SetHasSubscribers(mImpl->HasClients());
        }

        try {
            // 1. Frame state
            auto frame = mpSource->GetLatestFrameState();
            if (frame) {
                auto frame_buf = WebViewerProtocol::EncodeFrameState(*frame);
                mImpl->BroadcastRealtime(frame_buf);
            }

            // 2. Image state
            auto image = mpSource->GetLatestImageState();
            if (image && !image->immutable_grayscale_image.empty()) {
                auto img_buf = WebViewerProtocol::EncodeImageJpeg(image->epoch, image->frame_sequence, image->capture_timestamp_ns, image->immutable_grayscale_image, mConfig.image_jpeg_quality);
                if (!img_buf.empty()) {
                    mImpl->BroadcastBulk(img_buf);
                }

                if (!image->features.empty()) {
                    auto feat_buf = WebViewerProtocol::EncodeFeatureOverlay(image->epoch, image->frame_sequence, image->capture_timestamp_ns, image->features);
                    if (!feat_buf.empty()) {
                        mImpl->BroadcastBulk(feat_buf);
                    }
                }
            }

            // 3. Map events
            auto events = mpSource->PopPendingMapEvents();
            {
                std::lock_guard<std::mutex> lock(mMirrorMutex);
                for (const auto& ev : events) {
                    ReconcileMirror(mMirrorPoints, ev);
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "[WebViewerBackend] MirrorWorkerLoop exception: " << e.what() << std::endl;
        } catch (...) {
            std::cerr << "[WebViewerBackend] MirrorWorkerLoop unknown exception" << std::endl;
        }

        // Periodically broadcast point chunk
        std::vector<VisualizationMapPoint> points_list;
        {
            std::lock_guard<std::mutex> lock(mMirrorMutex);
            for (const auto& kv : mMirrorPoints) {
                points_list.push_back({kv.second.id, kv.second.pos, kv.second.reference});
            }
        }

        if (!points_list.empty()) {
            auto pts_buf = WebViewerProtocol::EncodePointsChunk(mpSource->GetCurrentEpoch(), 1, points_list);
            mImpl->BroadcastBulk(pts_buf);
        }
    }
}

} // namespace ORB_SLAM3
