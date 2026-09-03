#include <gtest/gtest.h>

#include "System.h"
#include "Atlas.h"
#include "Map.h"
#include "KeyFrame.h"
#include "KeyFrameDatabase.h"
#include "Frame.h"
#include "ORBVocabulary.h"

#include <opencv2/core/core.hpp>
#include <vector>
#include <algorithm>

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

class TestableKeyFrameDatabase : public KeyFrameDatabase {
public:
    TestableKeyFrameDatabase() : KeyFrameDatabase() {}
    explicit TestableKeyFrameDatabase(const ORBVocabulary* pVoc, size_t minWords = 10)
        : KeyFrameDatabase(*pVoc) {
        mpVoc = pVoc;
        if (mvInvertedFile.size() < minWords) {
            mvInvertedFile.resize(minWords);
        }
    }
    explicit TestableKeyFrameDatabase(const ORBVocabulary& voc, size_t minWords = 10)
        : KeyFrameDatabase(voc) {
        mpVoc = &voc;
        if (mvInvertedFile.size() < minWords) {
            mvInvertedFile.resize(minWords);
        }
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

TEST(AtlasLifecycle, DetectRelocalizationCandidatesWithNullMapSearchesAll) {
    // 1. In-memory minimal synthetic vocabulary
    std::vector<std::vector<cv::Mat>> features(4);
    for (int i = 0; i < 4; ++i) {
        cv::Mat feat = cv::Mat::zeros(1, 32, CV_8U);
        feat.at<uint8_t>(0, 0) = static_cast<uint8_t>(i * 60 + 1);
        features[i].push_back(feat);
    }
    ORBVocabulary voc(4, 1);
    voc.create(features);

    // Create KeyFrameDatabase db(&voc)
    TestableKeyFrameDatabase db(&voc);

    // 2. Create two maps
    Map* pMap1 = new Map(1);
    Map* pMap2 = new Map(2);

    // 3. Create KeyFrames in Map 1 and Map 2
    KeyFrame* pKF1 = new KeyFrame();
    pKF1->mnId = 10;
    pKF1->UpdateMap(pMap1);
    pMap1->AddKeyFrame(pKF1);
    pKF1->SetKeyFrameDatabase(&db);
    pKF1->mBowVec.addWeight(0, 1.0);
    db.add(pKF1);

    KeyFrame* pKF2 = new KeyFrame();
    pKF2->mnId = 20;
    pKF2->UpdateMap(pMap2);
    pMap2->AddKeyFrame(pKF2);
    pKF2->SetKeyFrameDatabase(&db);
    pKF2->mBowVec.addWeight(0, 1.0);
    db.add(pKF2);

    // 4. Query Frame F sharing word 0 with the keyframes
    Frame F;
    F.mnId = 100;
    F.mBowVec.addWeight(0, 1.0);

    // Calling db.DetectRelocalizationCandidates(&F, nullptr) returns candidates from both maps
    std::vector<KeyFrame*> allCandidates = db.DetectRelocalizationCandidates(&F, nullptr);
    EXPECT_EQ(allCandidates.size(), 2u);
    bool foundKF1 = false;
    bool foundKF2 = false;
    for (KeyFrame* pKF : allCandidates) {
        if (pKF == pKF1) foundKF1 = true;
        if (pKF == pKF2) foundKF2 = true;
    }
    EXPECT_TRUE(foundKF1);
    EXPECT_TRUE(foundKF2);

    // Calling db.DetectRelocalizationCandidates(&F, pMap1) only returns candidates belonging to pMap1
    F.mnId = 101;
    std::vector<KeyFrame*> map1Candidates = db.DetectRelocalizationCandidates(&F, pMap1);
    EXPECT_EQ(map1Candidates.size(), 1u);
    if (!map1Candidates.empty()) {
        EXPECT_EQ(map1Candidates[0], pKF1);
    }

    // Calling db.DetectRelocalizationCandidates(&F, pMap2) only returns candidates belonging to pMap2
    F.mnId = 102;
    std::vector<KeyFrame*> map2Candidates = db.DetectRelocalizationCandidates(&F, pMap2);
    EXPECT_EQ(map2Candidates.size(), 1u);
    if (!map2Candidates.empty()) {
        EXPECT_EQ(map2Candidates[0], pKF2);
    }

    // 5. Test that bad maps are cleanly ignored
    pMap2->SetBad();
    EXPECT_TRUE(pMap2->IsBad());
    F.mnId = 103;
    std::vector<KeyFrame*> afterBadMapCandidates = db.DetectRelocalizationCandidates(&F, nullptr);
    EXPECT_EQ(afterBadMapCandidates.size(), 1u);
    if (!afterBadMapCandidates.empty()) {
        EXPECT_EQ(afterBadMapCandidates[0], pKF1);
    }

    F.mnId = 104;
    std::vector<KeyFrame*> badMapDirectCandidates = db.DetectRelocalizationCandidates(&F, pMap2);
    EXPECT_TRUE(badMapDirectCandidates.empty());

    // 6. Test that bad keyframes (pKFi->SetBadFlag()) are cleanly ignored
    KeyFrame* pKF3 = new KeyFrame();
    pKF3->mnId = 30; // Different from pMap1 initial keyframe (10)
    pKF3->UpdateMap(pMap1);
    pMap1->AddKeyFrame(pKF3);
    pKF3->SetKeyFrameDatabase(&db);
    pKF3->mBowVec.addWeight(0, 1.0);
    db.add(pKF3);

    F.mnId = 105;
    std::vector<KeyFrame*> beforeBadKFCandidates = db.DetectRelocalizationCandidates(&F, pMap1);
    EXPECT_EQ(beforeBadKFCandidates.size(), 2u);

    pKF3->SetBadFlag();
    EXPECT_TRUE(pKF3->isBad());

    F.mnId = 106;
    std::vector<KeyFrame*> afterBadKFCandidates = db.DetectRelocalizationCandidates(&F, pMap1);
    EXPECT_EQ(afterBadKFCandidates.size(), 1u);
    if (!afterBadKFCandidates.empty()) {
        EXPECT_EQ(afterBadKFCandidates[0], pKF1);
    }

    // Re-insert bad keyframe into DB to test that DetectRelocalizationCandidates
    // internal check 'if (pKFi->isBad()) continue;' cleanly ignores it even if present
    db.add(pKF3);
    F.mnId = 107;
    std::vector<KeyFrame*> withBadInDBCandidates = db.DetectRelocalizationCandidates(&F, nullptr);
    EXPECT_EQ(withBadInDBCandidates.size(), 1u);
    if (!withBadInDBCandidates.empty()) {
        EXPECT_EQ(withBadInDBCandidates[0], pKF1);
    }

    // 7. Clean resource cleanup
    db.clear();
    delete pKF1;
    delete pKF2;
    delete pKF3;
    delete pMap1;
    delete pMap2;
}

TEST(AtlasLifecycle, AtlasPostLoadSelectsActiveMap) {
    TestableAtlas atlas;
    Map* pInitialMap = atlas.GetCurrentMap();

    Map* pMap1 = new Map(10);
    Map* pMap2 = new Map(20);
    atlas.AddBackupMap(pMap1);
    atlas.AddBackupMap(pMap2);

    atlas.PostLoad();

    EXPECT_NE(atlas.GetCurrentMap(), nullptr);
    EXPECT_TRUE(atlas.GetCurrentMap() == pMap1 || atlas.GetCurrentMap() == pMap2);
    EXPECT_TRUE(atlas.GetCurrentMap()->IsInUse());

    delete pInitialMap;
}

