#include "VisualizationSource.h"

namespace ORB_SLAM3 {

VisualizationSource::VisualizationSource() {}

void VisualizationSource::PublishFrameState(const VisualizationFrameSnapshot& frame) {
    std::lock_guard<std::mutex> lock(mFrameMutex);
    mLatestFrame = std::make_shared<const VisualizationFrameSnapshot>(frame);
}

void VisualizationSource::PublishImageState(const VisualizationImageSnapshot& image) {
    std::lock_guard<std::mutex> lock(mImageMutex);
    mLatestImage = std::make_shared<const VisualizationImageSnapshot>(image);
}

void VisualizationSource::PublishMapEvent(const VisualizationMapEvent& event) {
    std::lock_guard<std::mutex> lock(mEventsMutex);
    if (mPendingEvents.size() < MAX_PENDING_EVENTS) {
        mPendingEvents.push_back(event);
    }
}

std::shared_ptr<const VisualizationFrameSnapshot> VisualizationSource::GetLatestFrameState() const {
    std::lock_guard<std::mutex> lock(mFrameMutex);
    return mLatestFrame;
}

std::shared_ptr<const VisualizationImageSnapshot> VisualizationSource::GetLatestImageState() const {
    std::lock_guard<std::mutex> lock(mImageMutex);
    return mLatestImage;
}

std::vector<VisualizationMapEvent> VisualizationSource::PopPendingMapEvents() {
    std::lock_guard<std::mutex> lock(mEventsMutex);
    std::vector<VisualizationMapEvent> events;
    events.swap(mPendingEvents);
    return events;
}

} // namespace ORB_SLAM3
