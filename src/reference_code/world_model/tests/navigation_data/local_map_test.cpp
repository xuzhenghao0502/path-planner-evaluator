#include <gtest/gtest.h>

#define private public
#include "navigation_data/local_map.h"


namespace gpal::pnc::planning {

class LocalMapTest : public ::testing::Test {
 protected:
  void SetUp() override {
    local_map_ = new LocalMap;
    road_seg_ = new LocalMapRoadSegment;
  }
  void TearDown() override {
    delete local_map_;
    delete road_seg_;
  }

  LocalMap* local_map_;
  LocalMapRoadSegment* road_seg_;
};

TEST_F(LocalMapTest, basicTest) {
  std::shared_ptr<LocalMapRoadSegment> road_seg_1 = std::make_shared<LocalMapRoadSegment>();
  road_seg_1->getMutableRoadSegmentId() = "1";
  road_seg_1->getMutablePoints().emplace_back(math::Vec3d(0.0, 0.0, 0.0));
  road_seg_1->getMutablePoints().emplace_back(math::Vec3d(0.0, 10.0, 0.0));
  local_map_->getMutableRoadSegmentMap()->emplace("1", road_seg_1);

  std::shared_ptr<LocalMapRoadSegment> road_seg_2 = std::make_shared<LocalMapRoadSegment>();
  road_seg_2->getMutableRoadSegmentId() = "2";
  road_seg_2->getMutablePoints().emplace_back(math::Vec3d(0.0, 10.0, 0.0));
  road_seg_2->getMutablePoints().emplace_back(math::Vec3d(10.0, 10.0, 0.0));
  local_map_->getMutableRoadSegmentMap()->emplace("2", road_seg_2);
  local_map_->getMutableMapErrorCode() = LocalMap::MapErrorCode::kOK;
  local_map_->getMutableEgoRoadSegmentId() = "1";

  auto result = local_map_->init();
  EXPECT_TRUE(result);
}

TEST_F(LocalMapTest, initTest) {
  road_seg_->getMutablePoints().emplace_back(math::Vec3d(0.0, 10.0, 0.0));
  auto result = road_seg_->init();
  EXPECT_FALSE(result);

  result = local_map_->init();
  EXPECT_FALSE(result);

  std::shared_ptr<LocalMapRoadSegment> road_seg_1 = std::make_shared<LocalMapRoadSegment>();
  road_seg_1->getMutableRoadSegmentId() = "1";
  local_map_->getMutableRoadSegmentMap()->emplace("1", road_seg_1);
  local_map_->getMutableMapErrorCode() = LocalMap::MapErrorCode::kOK;
  local_map_->getMutableEgoRoadSegmentId() = "1";
  result = local_map_->init();
  EXPECT_FALSE(result);

  delete local_map_;
  local_map_ = new LocalMap;
  math::AABoxKDTree2dParams params;
  result = local_map_->init(params);
  EXPECT_FALSE(result);
}

TEST_F(LocalMapTest, distanceFunctionTest) {
  math::Vec2d point(1.0, 1.0);
  double dist = road_seg_->DistanceSquareTo(point);
  EXPECT_EQ(dist, std::numeric_limits<double>::max());

  dist = road_seg_->DistanceSquareTo(point, 0.0);
  EXPECT_EQ(dist, std::numeric_limits<double>::max());

  road_seg_->getMutablePoints().emplace_back(math::Vec3d(0.0, 0.0, 0.0));
  road_seg_->getMutablePoints().emplace_back(math::Vec3d(5.0, 0.0, 0.0));
  road_seg_->getMutablePoints().emplace_back(math::Vec3d(10.0, 0.0, 0.0));
  auto result = road_seg_->init();
  EXPECT_TRUE(result);
  dist = road_seg_->DistanceSquareTo(point);
  EXPECT_EQ(dist, 1.0);

  road_seg_->getMutableRoadDirection() = planning::RoadDirection::kBothDirection;
  dist = road_seg_->DistanceSquareTo(point, 0.0);
  EXPECT_EQ(dist, 1.0);
}

TEST_F(LocalMapTest, isValidTest) {
  local_map_->getMutableMapErrorCode() = LocalMap::MapErrorCode::kOK;
  auto result = local_map_->isValid();
  EXPECT_FALSE(result);

  local_map_->getMutableEgoRoadSegmentId() = "1";
  result = local_map_->isValid();
  EXPECT_FALSE(result);
}

TEST_F(LocalMapTest, searchTest) {
  std::shared_ptr<LocalMapRoadSegment> road_seg_1 = std::make_shared<LocalMapRoadSegment>();
  road_seg_1->getMutableRoadSegmentId() = "1";
  road_seg_1->getMutablePoints().emplace_back(math::Vec3d(0.0, 0.0, 0.0));
  road_seg_1->getMutablePoints().emplace_back(math::Vec3d(0.0, 10.0, 0.0));
  local_map_->getMutableRoadSegmentMap()->emplace("1", road_seg_1);

  std::shared_ptr<LocalMapRoadSegment> road_seg_2 = std::make_shared<LocalMapRoadSegment>();
  road_seg_2->getMutableRoadSegmentId() = "2";
  road_seg_2->getMutablePoints().emplace_back(math::Vec3d(0.0, 10.0, 0.0));
  road_seg_2->getMutablePoints().emplace_back(math::Vec3d(10.0, 10.0, 0.0));
  local_map_->getMutableRoadSegmentMap()->emplace("2", road_seg_2);
  local_map_->getMutableMapErrorCode() = LocalMap::MapErrorCode::kOK;
  local_map_->getMutableEgoRoadSegmentId() = "1";
  math::AABoxKDTree2dParams param;
  param.max_leaf_size = 10;
  param.max_leaf_dimension = 5.0;
  local_map_->init(param);
  math::Arrow2d pose(5.0, 10.0, 0.0);
  auto nearest_road_id = local_map_->getNearestRoadSegmentId(pose);
  EXPECT_EQ(nearest_road_id, "2");

  math::Arrow2d pose2(std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), 0.0);
  nearest_road_id = local_map_->getNearestRoadSegmentId(pose2);
  EXPECT_EQ(nearest_road_id, "");

  delete local_map_;
  local_map_ = new LocalMap;
  LocalMapRoadSegment** nearest_road_seg = nullptr;
  auto result = local_map_->getNearestRoadSegment(pose, nearest_road_seg);
  EXPECT_FALSE(result);

  auto id = local_map_->getNearestRoadSegmentId(pose);
  EXPECT_EQ(id, "");
}

}  // namespace gpal::pnc::planning
