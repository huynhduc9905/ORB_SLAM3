#include <gtest/gtest.h>

#include "System.h"
#include "Atlas.h"
#include "Map.h"
#include "KeyFrame.h"

using namespace ORB_SLAM3;

namespace {

class TestableAtlas : public Atlas {
public:
    TestableAtlas() : Atlas() {}
    ~TestableAtlas() = default;

    void AddBackupMap(Map* pMap) {
        mvpBackupMaps.push_back(pMap);
    }

    const std::vector<Map*>& GetBackupMaps() const {
        return mvpBackupMaps;
    }

    void InsertMap(Map* pMap) {
        mspMaps.insert(pMap);
    }
};

} // namespace

TEST(SystemAtlas, NormalizeAtlasPath) {
    // Empty string stays empty
    EXPECT_EQ(System::NormalizeAtlasPath(""), "");

    // Relative path without extension gets ./ prefix and .osa suffix
    EXPECT_EQ(System::NormalizeAtlasPath("my_map"), "./my_map.osa");

    // Relative path with extension gets ./ prefix and avoids duplicate .osa
    EXPECT_EQ(System::NormalizeAtlasPath("my_map.osa"), "./my_map.osa");

    // Explicit ./ relative path without extension
    EXPECT_EQ(System::NormalizeAtlasPath("./my_map"), "./my_map.osa");

    // Explicit ./ relative path with extension avoids duplicate ./ and .osa
    EXPECT_EQ(System::NormalizeAtlasPath("./my_map.osa"), "./my_map.osa");

    // Subdirectory relative path
    EXPECT_EQ(System::NormalizeAtlasPath("subdir/map"), "./subdir/map.osa");
    EXPECT_EQ(System::NormalizeAtlasPath("subdir/map.osa"), "./subdir/map.osa");

    // Absolute path without extension preserves absolute path and appends .osa
    EXPECT_EQ(System::NormalizeAtlasPath("/tmp/saved_map"), "/tmp/saved_map.osa");

    // Absolute path with extension preserves exact absolute path
    EXPECT_EQ(System::NormalizeAtlasPath("/tmp/saved_map.osa"), "/tmp/saved_map.osa");
    EXPECT_EQ(System::NormalizeAtlasPath("/home/user/atlas/map_2026.osa"), "/home/user/atlas/map_2026.osa");
}

TEST(AtlasLifecycle, SetCurrentMapActivatesMap) {
    TestableAtlas atlas;
    EXPECT_NE(atlas.GetCurrentMap(), nullptr); // calls CreateNewMapNoLock() cleanly

    Map* pMap1 = new Map(10);
    Map* pMap2 = new Map(20);

    atlas.InsertMap(pMap1);
    atlas.InsertMap(pMap2);

    atlas.SetCurrentMap(pMap1);
    EXPECT_EQ(atlas.GetCurrentMap(), pMap1);
    EXPECT_TRUE(pMap1->IsInUse());

    atlas.ChangeMap(pMap2);
    EXPECT_EQ(atlas.GetCurrentMap(), pMap2);
    EXPECT_TRUE(pMap2->IsInUse());
    EXPECT_FALSE(pMap1->IsInUse());
}

TEST(AtlasLifecycle, PreSaveSanitizesBackupMaps) {
    TestableAtlas atlas;

    Map* pGoodMap = new Map(100);
    KeyFrame* pKF1 = new KeyFrame();
    pGoodMap->AddKeyFrame(pKF1);
    atlas.InsertMap(pGoodMap);

    Map* pEmptyMap = new Map(101);
    atlas.InsertMap(pEmptyMap);

    Map* pBadMap = new Map(102);
    KeyFrame* pKF2 = new KeyFrame();
    pBadMap->AddKeyFrame(pKF2);
    pBadMap->SetBad();
    atlas.InsertMap(pBadMap);

    atlas.PreSave();

    const auto& backupMaps = atlas.GetBackupMaps();
    EXPECT_EQ(backupMaps.size(), 1u);
    if (!backupMaps.empty()) {
        EXPECT_EQ(backupMaps[0], pGoodMap);
    }

    delete pKF1;
    delete pKF2;
    // Note: pGoodMap, pEmptyMap, and pBadMap are owned by atlas in mspMaps and cleaned up by Atlas::~Atlas()
}
