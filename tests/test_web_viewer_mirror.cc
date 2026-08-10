// Tests for the web-viewer backend point-mirror reconciliation.
//
// Regression target: after a loop closure, ORB-SLAM3 fuses/culls duplicate map
// points, so an object that was tracked twice collapses to one set of points.
// The backend used to only ever add/update points by id and never removed the
// culled ones, so the pre-closure duplicates lingered forever and the object
// appeared twice in the viewer. A full-snapshot event must replace the mirror
// wholesale so culled points disappear.

#include <gtest/gtest.h>

#include "WebViewerBackend.h"
#include "VisualizationTypes.h"

using namespace ORB_SLAM3;

namespace {

VisualizationMapPoint MakePoint(std::uint64_t id, float x, float y, float z) {
    VisualizationMapPoint p;
    p.id = id;
    p.world_position = Eigen::Vector3f(x, y, z);
    p.reference = false;
    return p;
}

VisualizationMapEvent FullSnapshot(std::vector<VisualizationMapPoint> pts) {
    VisualizationMapEvent ev;
    ev.type = VisualizationEventType::POINTS_UPDATED;
    ev.full_snapshot = true;
    ev.points = std::move(pts);
    return ev;
}

} // namespace

// The core regression: a full snapshot that no longer contains a previously
// seen point must drop that point (the loop-closure "one object shows two" bug).
TEST(WebViewerMirror, FullSnapshotDropsCulledPoints) {
    std::unordered_map<std::uint64_t, WebViewerBackend::MirrorMapPoint> mirror;

    // Before loop closure: object observed as two point sets (ids 1..4).
    WebViewerBackend::ReconcileMirror(mirror, FullSnapshot({
        MakePoint(1, 0, 0, 0), MakePoint(2, 1, 0, 0),
        MakePoint(3, 0.01f, 0, 0), MakePoint(4, 1.01f, 0, 0),
    }));
    ASSERT_EQ(mirror.size(), 4u);

    // After loop closure: points 3 and 4 were fused into 1 and 2 and culled;
    // SLAM now publishes only the surviving points, at corrected positions.
    WebViewerBackend::ReconcileMirror(mirror, FullSnapshot({
        MakePoint(1, 0, 0, 0), MakePoint(2, 1, 0, 0),
    }));

    EXPECT_EQ(mirror.size(), 2u);
    EXPECT_TRUE(mirror.count(1));
    EXPECT_TRUE(mirror.count(2));
    EXPECT_FALSE(mirror.count(3)) << "culled point 3 lingered as a ghost";
    EXPECT_FALSE(mirror.count(4)) << "culled point 4 lingered as a ghost";
}

// A full snapshot must reflect corrected positions for surviving points.
TEST(WebViewerMirror, FullSnapshotUpdatesPositions) {
    std::unordered_map<std::uint64_t, WebViewerBackend::MirrorMapPoint> mirror;

    WebViewerBackend::ReconcileMirror(mirror, FullSnapshot({MakePoint(7, 0, 0, 0)}));
    // Loop closure repositions the same map point.
    WebViewerBackend::ReconcileMirror(mirror, FullSnapshot({MakePoint(7, 5, 6, 7)}));

    ASSERT_EQ(mirror.size(), 1u);
    const auto& p = mirror.at(7);
    EXPECT_FLOAT_EQ(p.pos.x(), 5.0f);
    EXPECT_FLOAT_EQ(p.pos.y(), 6.0f);
    EXPECT_FLOAT_EQ(p.pos.z(), 7.0f);
}

// Non-snapshot (incremental) semantics must still work: add merges, remove erases.
TEST(WebViewerMirror, IncrementalAddAndRemove) {
    std::unordered_map<std::uint64_t, WebViewerBackend::MirrorMapPoint> mirror;

    WebViewerBackend::ReconcileMirror(mirror, FullSnapshot({MakePoint(1, 0, 0, 0)}));

    VisualizationMapEvent add;
    add.type = VisualizationEventType::POINTS_ADDED;
    add.full_snapshot = false;
    add.points = {MakePoint(2, 1, 1, 1)};
    WebViewerBackend::ReconcileMirror(mirror, add);
    EXPECT_EQ(mirror.size(), 2u);

    VisualizationMapEvent rem;
    rem.type = VisualizationEventType::POINTS_REMOVED;
    rem.points = {MakePoint(1, 0, 0, 0)};
    WebViewerBackend::ReconcileMirror(mirror, rem);
    EXPECT_EQ(mirror.size(), 1u);
    EXPECT_FALSE(mirror.count(1));
    EXPECT_TRUE(mirror.count(2));
}
