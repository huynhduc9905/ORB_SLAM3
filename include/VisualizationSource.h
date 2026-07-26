#ifndef VISUALIZATION_SOURCE_H
#define VISUALIZATION_SOURCE_H

#include "VisualizationTypes.h"
#include <mutex>
#include <atomic>
#include <vector>
#include <memory>

namespace ORB_SLAM3 {

class VisualizationSource {
public:
    VisualizationSource();
    ~VisualizationSource() = default;

    void PublishFrameState(const VisualizationFrameSnapshot& frame);
    void PublishImageState(const VisualizationImageSnapshot& image);
    void PublishMapEvent(const VisualizationMapEvent& event);

    std::shared_ptr<const VisualizationFrameSnapshot> GetLatestFrameState() const;
    std::shared_ptr<const VisualizationImageSnapshot> GetLatestImageState() const;
    std::vector<VisualizationMapEvent> PopPendingMapEvents();

    std::uint64_t GetCurrentEpoch() const { return mEpoch.load(); }
    void IncrementEpoch() { mEpoch++; }

private:
    std::atomic<std::uint64_t> mEpoch{1};

    mutable std::mutex mFrameMutex;
    std::shared_ptr<const VisualizationFrameSnapshot> mLatestFrame;

    mutable std::mutex mImageMutex;
    std::shared_ptr<const VisualizationImageSnapshot> mLatestImage;

    mutable std::mutex mEventsMutex;
    std::vector<VisualizationMapEvent> mPendingEvents;
    static constexpr size_t MAX_PENDING_EVENTS = 1000;
};

} // namespace ORB_SLAM3

#endif // VISUALIZATION_SOURCE_H
