#include <base/singleton.h>
#include <fmt/chrono.h>
#include <gtest/gtest.h>

#include "base/log.h"

#include "config_manager/config_manager.h"

#define private public
#define protected public
#include "navigation_data/local_router.h"


namespace gpal::pnc::planning {

class LocalRouterTest : public ::testing::Test {
 protected:
  void SetUp() override {
    int argc = 0;
    char path[] = "policy_planner";
    char* argv[]{path};
    if (!Singleton<ConfigManager>::get_instance()->init(argc, argv)) {
      ERT_PLOG_I << "Failed to init config!";
    }
  }

  void buildInputData();

  LocalRouter local_router_;
};

void LocalRouterTest::buildInputData() {
  local_router_.reset();

  // std::vector<std::vector<LaneTopo>>
  auto lanes_topo = local_router_.getMutableLanesTopos();
  std::vector<LaneTopo> lane_topos;
  LaneTopo lane_topo_1;
  lane_topo_1.lane_id = "1";
  lane_topo_1.lane_segments.emplace_back(std::pair<std::string, float>{"1", 1.0});
  lane_topo_1.previous_lanes.clear();
  lane_topo_1.next_lanes.emplace_back(std::pair<std::string, std::string>{"2", "2"});
  lane_topo_1.next_lanes.emplace_back(std::pair<std::string, std::string>{"5", "5"});
  lane_topos.emplace_back(lane_topo_1);
  LaneTopo lane_topo_2;
  lane_topo_2.lane_id = "2";
  lane_topo_2.lane_segments.emplace_back(std::pair<std::string, float>{"2", 1.0});
  lane_topo_2.previous_lanes.emplace_back(std::pair<std::string, std::string>{"1", "1"});
  lane_topo_2.next_lanes.emplace_back(std::pair<std::string, std::string>{"3", "3"});
  lane_topos.emplace_back(lane_topo_2);
  LaneTopo lane_topo_3;
  lane_topo_3.lane_id = "3";
  lane_topo_3.lane_segments.emplace_back(std::pair<std::string, float>{"3", 1.0});
  lane_topo_3.previous_lanes.emplace_back(std::pair<std::string, std::string>{"2", "2"});
  lane_topo_3.previous_lanes.emplace_back(std::pair<std::string, std::string>{"5", "5"});
  lane_topo_3.next_lanes.clear();
  lane_topos.emplace_back(lane_topo_3);
  lanes_topo->emplace_back(lane_topos);
  LaneTopo lane_topo_4;
  lane_topo_4.lane_id = "4";
  lane_topo_4.lane_segments.emplace_back(std::pair<std::string, float>{"4", 1.0});
  lane_topo_4.previous_lanes.clear();
  lane_topo_4.next_lanes.clear();
  lane_topos.clear();
  lane_topos.emplace_back(lane_topo_4);
  lanes_topo->emplace_back(lane_topos);
  LaneTopo lane_topo_5;
  lane_topo_5.lane_id = "5";
  lane_topo_5.lane_segments.emplace_back(std::pair<std::string, float>{"5", 1.0});
  lane_topo_5.previous_lanes.clear();
  lane_topo_5.next_lanes.emplace_back(std::pair<std::string, std::string>{"3", "3"});
  lane_topos.emplace_back(lane_topo_5);

  // std::unordered_map<std::string, std::unordered_map<std::string, LocalRouteSegment>>*
  auto lanes_segments_map = local_router_.getMutableLanesSegmentsMap();
  std::unordered_map<std::string, planning::LocalRouteSegment> lane_segments_map;

  LocalRouteSegment seg_1;
  seg_1.setId("1");
  seg_1.setNavigationScore(1.0);
  seg_1.setNavigationLaneChangeDistance(577.0);
  seg_1.mutablePreviousSegmentIds()->clear();
  seg_1.mutableNextSegmentIds()->emplace_back("2");
  seg_1.mutableNextSegmentIds()->emplace_back("5");
  seg_1.setLeftNeighborId("4");
  seg_1.setRightNeighborId("");
  seg_1.setLeftBoundType(LocalRouteSegment::BoundaryType::DASHED);
  seg_1.setRightBoundType(LocalRouteSegment::BoundaryType::SOLID);
  seg_1.setDirection(DrivingDirection::kDirectionForwardOnly);
  for (int i = 0; i < 50; i++) {
    seg_1.mutableBoundaryPoints()->emplace_back(math::Vec3d(i, 1.875, 0.0), math::Vec3d(i, -1.875, 0.0));
    seg_1.mutableGuidePoints()->emplace_back(seg_1.id(), i, 0.0, 0.0, 0.0, 0.0, i, 1.875, 1.875, 2.0, 2.0);
  }
  seg_1.setAdcL(0.1);
  seg_1.setAdcS(5.0);
  seg_1.setAdcHeadingDiff(0.0);
  std::tuple<planning::LineSourceType, float, float> res_1{
      LineSourceType::LocalRouteReferenceLine_LineSource_LineSourceType_kTypePerception, 0.0, 49.0};
  seg_1.mutableSourceInfos()->emplace_back(std::move(res_1));
  std::get<0>(*seg_1.mutableRange()) = true;
  std::get<1>(*seg_1.mutableRange()) = seg_1.guidePoints().front().s();
  std::get<2>(*seg_1.mutableRange()) = seg_1.guidePoints().back().s();
  seg_1.mutableRangePoints()->first = seg_1.guidePoints().front();
  seg_1.mutableRangePoints()->second = seg_1.guidePoints().back();
  seg_1.mutableStopLine()->is_exist = true;
  seg_1.mutableStopLine()->id = "a";
  seg_1.mutableStopLine()->x = 5.0;
  seg_1.mutableStopLine()->s = 5.0;
  seg_1.mutableStopLine()->direction = DrivingDirection::kDirectionForwardOnly;
  seg_1.mutableGate()->is_exist = true;
  seg_1.mutableGate()->id = "a";
  seg_1.mutableGate()->x = 5.0;
  seg_1.mutableGate()->s = 5.0;
  seg_1.mutableSpeedLimit()->start_s = std::get<1>(seg_1.range());
  seg_1.mutableSpeedLimit()->end_s = std::get<2>(seg_1.range());
  seg_1.mutableSpeedLimit()->start_point = planning::math::Vec3d(
      seg_1.rangePoints().first.x(), seg_1.rangePoints().first.y(), seg_1.rangePoints().first.z());
  seg_1.mutableSpeedLimit()->end_point = planning::math::Vec3d(
      seg_1.rangePoints().second.x(), seg_1.rangePoints().second.y(), seg_1.rangePoints().second.z());
  seg_1.mutableSpeedLimit()->max_speed_limit = 20.0;
  PerceptionArea area;
  seg_1.mutableAreas()->emplace_back(area);

  LocalRouteSegment seg_2;
  seg_2.setId("2");
  seg_2.setNavigationScore(1.0);
  seg_2.setNavigationLaneChangeDistance(527.0);
  seg_2.mutablePreviousSegmentIds()->emplace_back("1");
  seg_2.mutableNextSegmentIds()->emplace_back("3");
  seg_2.setLeftNeighborId("");
  seg_2.setRightNeighborId("8");
  seg_2.setLeftBoundType(LocalRouteSegment::BoundaryType::PHYSICALLY_UNCROSSABLE);
  seg_2.setRightBoundType(LocalRouteSegment::BoundaryType::DASHED);
  seg_2.setDirection(DrivingDirection::kDirectionRightOnly);
  for (int i = 0; i < 50; i++) {
    seg_2.mutableBoundaryPoints()->emplace_back(math::Vec3d(i + 50, 1.875, 0.0), math::Vec3d(i + 50, -1.875, 0.0));
    seg_2.mutableGuidePoints()->emplace_back(seg_2.id(), i + 50, 0.0, 0.0, 0.0, 0.0, i + 50, 1.875, 1.875, 2.0, 2.0);
  }
  seg_2.setAdcL(0.2);
  seg_2.setAdcS(45.0);
  seg_2.setAdcHeadingDiff(0.0);
  std::tuple<planning::LineSourceType, float, float> res_2{
      LineSourceType::LocalRouteReferenceLine_LineSource_LineSourceType_kTypeVirtual, 0.0, 49.0};
  seg_2.mutableSourceInfos()->emplace_back(std::move(res_2));
  std::get<0>(*seg_2.mutableRange()) = true;
  std::get<1>(*seg_2.mutableRange()) = seg_2.guidePoints().front().s();
  std::get<2>(*seg_2.mutableRange()) = seg_2.guidePoints().back().s();
  seg_2.mutableRangePoints()->first = seg_2.guidePoints().front();
  seg_2.mutableRangePoints()->second = seg_2.guidePoints().back();
  seg_2.mutableStopLine()->is_exist = true;
  seg_2.mutableStopLine()->id = "b";
  seg_2.mutableStopLine()->x = 55.0;
  seg_2.mutableStopLine()->s = 5.0;
  seg_2.mutableStopLine()->direction = DrivingDirection::kDirectionRightOnly;
  seg_2.mutableGate()->is_exist = true;
  seg_2.mutableGate()->id = "b";
  seg_2.mutableGate()->x = 55.0;
  seg_2.mutableGate()->s = 5.0;
  seg_2.mutableSpeedLimit()->start_s = std::get<1>(seg_2.range());
  seg_2.mutableSpeedLimit()->end_s = std::get<2>(seg_2.range());
  seg_2.mutableSpeedLimit()->start_point = planning::math::Vec3d(
      seg_2.rangePoints().first.x(), seg_2.rangePoints().first.y(), seg_2.rangePoints().first.z());
  seg_2.mutableSpeedLimit()->end_point = planning::math::Vec3d(
      seg_2.rangePoints().second.x(), seg_2.rangePoints().second.y(), seg_2.rangePoints().second.z());
  seg_2.mutableSpeedLimit()->max_speed_limit = 20.0;

  LocalRouteSegment seg_3;
  seg_3.setId("3");
  seg_3.setNavigationScore(1.0);
  seg_3.setNavigationLaneChangeDistance(477.0);
  seg_3.mutablePreviousSegmentIds()->emplace_back("2");
  seg_3.mutablePreviousSegmentIds()->emplace_back("5");
  seg_3.mutableNextSegmentIds()->clear();
  seg_3.setLeftNeighborId("");
  seg_3.setRightNeighborId("");
  seg_3.setLeftBoundType(LocalRouteSegment::BoundaryType::SOLID);
  seg_3.setRightBoundType(LocalRouteSegment::BoundaryType::DASHED);
  seg_3.setDirection(DrivingDirection::kDirectionLeftOnly);
  for (int i = 0; i < 50; i++) {
    seg_3.mutableBoundaryPoints()->emplace_back(math::Vec3d(i + 100, 1.875, 0.0), math::Vec3d(i + 100, -1.875, 0.0));
    seg_3.mutableGuidePoints()->emplace_back(seg_3.id(), i + 100, 0.0, 0.0, 0.0, 0.0, i + 100, 1.875, 1.875, 2.0, 2.0);
  }
  seg_3.setAdcL(0.5);
  seg_3.setAdcS(95.0);
  seg_3.setAdcHeadingDiff(0.0);
  std::tuple<planning::LineSourceType, float, float> res_3{
      LineSourceType::LocalRouteReferenceLine_LineSource_LineSourceType_kTypeLocalMap, 0.0, 49.0};
  seg_3.mutableSourceInfos()->emplace_back(std::move(res_3));
  std::get<0>(*seg_3.mutableRange()) = true;
  std::get<1>(*seg_3.mutableRange()) = seg_3.guidePoints().front().s();
  std::get<2>(*seg_3.mutableRange()) = seg_3.guidePoints().back().s();
  seg_3.mutableRangePoints()->first = seg_3.guidePoints().front();
  seg_3.mutableRangePoints()->second = seg_3.guidePoints().back();
  seg_3.mutableStopLine()->is_exist = false;
  seg_3.mutableGate()->is_exist = false;
  seg_3.mutableSpeedLimit()->start_s = std::get<1>(seg_3.range());
  seg_3.mutableSpeedLimit()->end_s = std::get<2>(seg_3.range());
  seg_3.mutableSpeedLimit()->start_point = planning::math::Vec3d(
      seg_3.rangePoints().first.x(), seg_3.rangePoints().first.y(), seg_3.rangePoints().first.z());
  seg_3.mutableSpeedLimit()->end_point = planning::math::Vec3d(
      seg_3.rangePoints().second.x(), seg_3.rangePoints().second.y(), seg_3.rangePoints().second.z());
  seg_3.mutableSpeedLimit()->max_speed_limit = 10.0;

  LocalRouteSegment seg_4;
  seg_4.setId("4");
  seg_4.setNavigationScore(0.0);
  seg_4.setNavigationLaneChangeDistance(-1.0);
  seg_4.mutablePreviousSegmentIds()->clear();
  seg_4.mutableNextSegmentIds()->clear();
  seg_4.setLeftNeighborId("");
  seg_4.setRightNeighborId("1");
  seg_4.setLeftBoundType(LocalRouteSegment::BoundaryType::SOLID);
  seg_4.setRightBoundType(LocalRouteSegment::BoundaryType::DASHED);
  seg_4.setDirection(DrivingDirection::kDirectionUTurnOnly);
  for (int i = 0; i < 50; i++) {
    seg_4.mutableBoundaryPoints()->emplace_back(math::Vec3d(i, 5.625, 0.0), math::Vec3d(i, 1.875, 0.0));
    seg_4.mutableGuidePoints()->emplace_back(seg_4.id(), i, 3.75, 0.0, 0.0, 0.0, i, 1.875, 1.875, 2.0, 2.0);
  }
  seg_4.setAdcL(-3.65);
  seg_4.setAdcS(5.0);
  seg_4.setAdcHeadingDiff(0.0);
  std::tuple<planning::LineSourceType, float, float> res_4{
      LineSourceType::LocalRouteReferenceLine_LineSource_LineSourceType_kTypePerception, 0.0, 49.0};
  seg_4.mutableSourceInfos()->emplace_back(std::move(res_4));
  std::get<0>(*seg_4.mutableRange()) = true;
  std::get<1>(*seg_4.mutableRange()) = seg_4.guidePoints().front().s();
  std::get<2>(*seg_4.mutableRange()) = seg_4.guidePoints().back().s();
  seg_4.mutableRangePoints()->first = seg_4.guidePoints().front();
  seg_4.mutableRangePoints()->second = seg_4.guidePoints().back();
  seg_4.mutableStopLine()->is_exist = false;
  seg_4.mutableGate()->is_exist = false;
  seg_4.mutableSpeedLimit()->start_s = std::get<1>(seg_4.range());
  seg_4.mutableSpeedLimit()->end_s = std::get<2>(seg_4.range());
  seg_4.mutableSpeedLimit()->start_point = planning::math::Vec3d(
      seg_4.rangePoints().first.x(), seg_4.rangePoints().first.y(), seg_4.rangePoints().first.z());
  seg_4.mutableSpeedLimit()->end_point = planning::math::Vec3d(
      seg_4.rangePoints().second.x(), seg_4.rangePoints().second.y(), seg_4.rangePoints().second.z());
  seg_4.mutableSpeedLimit()->max_speed_limit = 40.0;

  LocalRouteSegment seg_5;
  seg_5.setId("5");
  seg_5.setNavigationScore(1.0);
  seg_5.setNavigationLaneChangeDistance(527.0);
  seg_5.mutablePreviousSegmentIds()->emplace_back("1");
  seg_5.mutableNextSegmentIds()->emplace_back("3");
  seg_5.setLeftNeighborId("");
  seg_5.setRightNeighborId("");
  seg_5.setLeftBoundType(LocalRouteSegment::BoundaryType::PHYSICALLY_UNCROSSABLE);
  seg_5.setRightBoundType(LocalRouteSegment::BoundaryType::DASHED);
  seg_5.setDirection(DrivingDirection::kDirectionRightOnly);
  for (int i = 0; i < 50; i++) {
    seg_5.mutableBoundaryPoints()->emplace_back(math::Vec3d(i + 50, 1.875, 0.0), math::Vec3d(i + 50, -1.875, 0.0));
    seg_5.mutableGuidePoints()->emplace_back(seg_5.id(), i + 50, 0.0, 0.0, 0.0, 0.0, i + 50, 1.875, 1.875, 2.0, 2.0);
  }
  seg_5.setAdcL(0.2);
  seg_5.setAdcS(45.0);
  seg_5.setAdcHeadingDiff(0.0);
  std::tuple<planning::LineSourceType, float, float> res_5{
      LineSourceType::LocalRouteReferenceLine_LineSource_LineSourceType_kTypeVirtual, 0.0, 49.0};
  seg_5.mutableSourceInfos()->emplace_back(std::move(res_5));
  std::get<0>(*seg_5.mutableRange()) = true;
  std::get<1>(*seg_5.mutableRange()) = seg_5.guidePoints().front().s();
  std::get<2>(*seg_5.mutableRange()) = seg_5.guidePoints().back().s();
  seg_5.mutableRangePoints()->first = seg_5.guidePoints().front();
  seg_5.mutableRangePoints()->second = seg_5.guidePoints().back();
  seg_5.mutableSpeedLimit()->start_s = std::get<1>(seg_5.range());
  seg_5.mutableSpeedLimit()->end_s = std::get<2>(seg_5.range());
  seg_5.mutableSpeedLimit()->start_point = planning::math::Vec3d(
      seg_5.rangePoints().first.x(), seg_5.rangePoints().first.y(), seg_5.rangePoints().first.z());
  seg_5.mutableSpeedLimit()->end_point = planning::math::Vec3d(
      seg_5.rangePoints().second.x(), seg_5.rangePoints().second.y(), seg_5.rangePoints().second.z());
  seg_5.mutableSpeedLimit()->max_speed_limit = 20.0;

  lane_segments_map.emplace(seg_1.id(), std::move(seg_1));
  lanes_segments_map->emplace(seg_1.id(), std::move(lane_segments_map));
  lane_segments_map.emplace(seg_2.id(), std::move(seg_2));
  lanes_segments_map->emplace(seg_2.id(), std::move(lane_segments_map));
  lane_segments_map.emplace(seg_3.id(), std::move(seg_3));
  lanes_segments_map->emplace(seg_3.id(), std::move(lane_segments_map));
  lane_segments_map.emplace(seg_4.id(), std::move(seg_4));
  lanes_segments_map->emplace(seg_4.id(), std::move(lane_segments_map));
  lane_segments_map.emplace(seg_5.id(), std::move(seg_5));
  lanes_segments_map->emplace(seg_5.id(), std::move(lane_segments_map));
  lane_segments_map.emplace("20", LocalRouteSegment());
  lanes_segments_map->emplace("20", std::move(lane_segments_map));
}

TEST_F(LocalRouterTest, processFuncTest) {
  buildInputData();
  local_router_.process();
  auto local_routes = local_router_.getLocalRoutes();
  EXPECT_TRUE(!local_routes.empty());

  for (auto local_route : local_routes) {
    auto mf_ranges = local_route.second.getMergeForkRangesFromSRange(0.0, 150.0, 10.0);
    auto m_ranges = local_route.second.getMergeRangesFromSRange(0.0, 150.0, 10.0);
    auto f_ranges = local_route.second.getForkRangesFromSRange(0.0, 150.0, 10.0);
    auto stoplines = local_route.second.getStopLinesFromSRange(0.0, 150.0, 10.0);
    auto gates = local_route.second.getGatesFromSRange(0.0, 150.0, 10.0);
    auto speed_limit = local_route.second.getSpeedLimitFromS(13.0);
    auto directions = local_route.second.getDirectionsFromSRange(0.0, 150.0, 10.0);
    auto direction = local_route.second.getDirectionFromS(13.0);
    auto source_infos = local_route.second.getSourceInfosFromSRange(0.0, 150.0, 10.0);
    auto source_info = local_route.second.getSourceInfoFromS(13.0);
    auto boundary_types = local_route.second.getBoundaryTypesFromSRange(0.0, 150.0, 10.0);
    auto boundary_type = local_route.second.getBoundaryTypeFromS(13.0);
    auto left_neighbor = local_route.second.getLeftNeighborLocalRouteBySRange(0.0, 150.0, 10.0);
    auto right_neighbor = local_route.second.getRightNeighborLocalRouteBySRange(0.0, 150.0, 10.0);
  }

  LocalRouteSegment seg_a;
  LocalRoutePoint pt_1, pt_2;
  auto is_match = local_router_.IsSegmentMatchMotionConstraints(seg_a);
  EXPECT_FALSE(is_match);
  pt_1.setX(20.0);
  seg_a.mutableGuidePoints()->emplace_back(pt_1);
  seg_a.mutableGuidePoints()->emplace_back(pt_2);
  is_match = local_router_.IsSegmentMatchMotionConstraints(seg_a);
  EXPECT_TRUE(is_match);
  pt_1.setX(5.0);
  seg_a.mutableGuidePoints()->clear();
  seg_a.mutableGuidePoints()->emplace_back(pt_1);
  seg_a.mutableGuidePoints()->emplace_back(pt_2);
  is_match = local_router_.IsSegmentMatchMotionConstraints(seg_a);
  EXPECT_FALSE(is_match);

  auto goal_seg = local_router_.getSegmentBySegmentId("1");
  EXPECT_TRUE(goal_seg.first);
  goal_seg = local_router_.getSegmentBySegmentId("100");
  EXPECT_FALSE(goal_seg.first);
  goal_seg = local_router_.getSegmentBySegmentId("0");
  EXPECT_FALSE(goal_seg.first);
  goal_seg = local_router_.getSegmentBySegmentId("");
  EXPECT_FALSE(goal_seg.first);

  auto goal_lane_id = local_router_.getLaneIdBySegmentId("1");
  EXPECT_TRUE(goal_lane_id.first);
  goal_lane_id = local_router_.getLaneIdBySegmentId("100");
  EXPECT_FALSE(goal_lane_id.first);
  goal_lane_id = local_router_.getLaneIdBySegmentId("0");
  EXPECT_FALSE(goal_lane_id.first);
  goal_lane_id = local_router_.getLaneIdBySegmentId("");
  EXPECT_FALSE(goal_lane_id.first);

  auto goal_local_routes_id = local_router_.getLocalRoutesIdBySegmentId("1");
  EXPECT_TRUE(goal_local_routes_id.first);
  goal_local_routes_id = local_router_.getLocalRoutesIdBySegmentId("100");
  EXPECT_FALSE(goal_local_routes_id.first);
  goal_local_routes_id = local_router_.getLocalRoutesIdBySegmentId("0");
  EXPECT_FALSE(goal_local_routes_id.first);
  goal_local_routes_id = local_router_.getLocalRoutesIdBySegmentId("");
  EXPECT_FALSE(goal_local_routes_id.first);

  auto goal_local_routes = local_router_.getLocalRoutesBySegmentId("1");
  EXPECT_TRUE(goal_local_routes.first);
  goal_local_routes = local_router_.getLocalRoutesBySegmentId("100");
  EXPECT_FALSE(goal_local_routes.first);
  goal_local_routes = local_router_.getLocalRoutesBySegmentId("0");
  EXPECT_FALSE(goal_local_routes.first);
  goal_local_routes = local_router_.getLocalRoutesBySegmentId("");
  EXPECT_FALSE(goal_local_routes.first);

  auto goal_segs_id = local_router_.getSegmentsIdByLaneId("1");
  EXPECT_TRUE(goal_segs_id.first);
  goal_segs_id = local_router_.getSegmentsIdByLaneId("100");
  EXPECT_FALSE(goal_segs_id.first);
  goal_segs_id = local_router_.getSegmentsIdByLaneId("0");
  EXPECT_FALSE(goal_segs_id.first);
  goal_segs_id = local_router_.getSegmentsIdByLaneId("");
  EXPECT_FALSE(goal_segs_id.first);

  auto goal_segs = local_router_.getSegmentsByLaneId("1");
  EXPECT_TRUE(goal_segs.first);
  goal_segs = local_router_.getSegmentsByLaneId("100");
  EXPECT_FALSE(goal_segs.first);
  goal_segs = local_router_.getSegmentsByLaneId("0");
  EXPECT_FALSE(goal_segs.first);
  goal_segs = local_router_.getSegmentsByLaneId("");
  EXPECT_FALSE(goal_segs.first);

  goal_local_routes_id = local_router_.getLocalRoutesIdByLaneId("1");
  EXPECT_TRUE(goal_local_routes_id.first);
  goal_local_routes_id = local_router_.getLocalRoutesIdByLaneId("100");
  EXPECT_FALSE(goal_local_routes_id.first);
  goal_local_routes_id = local_router_.getLocalRoutesIdByLaneId("0");
  EXPECT_FALSE(goal_local_routes_id.first);
  goal_local_routes_id = local_router_.getLocalRoutesIdByLaneId("");
  EXPECT_FALSE(goal_local_routes_id.first);

  auto goal_local_routes_string_id = local_router_.getLocalRoutesStringIdByLaneId("1");
  EXPECT_TRUE(goal_local_routes_string_id.first);
  goal_local_routes_string_id = local_router_.getLocalRoutesStringIdByLaneId("100");
  EXPECT_FALSE(goal_local_routes_string_id.first);
  goal_local_routes_string_id = local_router_.getLocalRoutesStringIdByLaneId("0");
  EXPECT_FALSE(goal_local_routes_string_id.first);
  goal_local_routes_string_id = local_router_.getLocalRoutesStringIdByLaneId("");
  EXPECT_FALSE(goal_local_routes_string_id.first);

  goal_local_routes = local_router_.getLocalRoutesByLaneId("1");
  EXPECT_TRUE(goal_local_routes.first);
  goal_local_routes = local_router_.getLocalRoutesByLaneId("100");
  EXPECT_FALSE(goal_local_routes.first);
  goal_local_routes = local_router_.getLocalRoutesByLaneId("0");
  EXPECT_FALSE(goal_local_routes.first);
  goal_local_routes = local_router_.getLocalRoutesByLaneId("");
  EXPECT_FALSE(goal_local_routes.first);

  auto goal_local_route = local_router_.getLocalRouteByLocalRouteId(1);
  EXPECT_TRUE(goal_local_route.first);
  goal_local_route = local_router_.getLocalRouteByLocalRouteId(100);
  EXPECT_FALSE(goal_local_route.first);
}


TEST_F(LocalRouterTest, RebuildBoundaryCrossRangesTest) {
  LocalRoute local_route;
  local_route.setId(1);
  
  std::vector<BoundaryCrossRange>* boundary_cross_ranges_0 = nullptr;
  auto res = local_route.RebuildBoundaryCrossRanges(boundary_cross_ranges_0);
  EXPECT_TRUE(!res);

  std::vector<BoundaryCrossRange> boundary_cross_ranges_1;
  res = local_route.RebuildBoundaryCrossRanges(&boundary_cross_ranges_1);
  EXPECT_TRUE(!res);

  auto local_routes_ = local_router_.getMutableLocalRoutes();
  local_routes_->emplace(local_route.id(), local_route);
  local_router_.rebuildLocalRoutes();
  EXPECT_TRUE(true);
}

TEST_F(LocalRouterTest, RebuildSpeedLimitsTest) {
  LocalRoute local_route;
  
  auto res = local_route.RebuildSpeedLimits();
  EXPECT_TRUE(!res);
}

TEST_F(LocalRouterTest, RebuildDirectionsTest) {
  LocalRoute local_route;
  
  auto res = local_route.RebuildDirections();
  EXPECT_TRUE(!res);

  auto directions = local_route.mutableDirections();
  directions->emplace_back(0.0, 10.0, DrivingDirection::kDirectionUnknown, math::Vec3d(0.0, 0.0, 0.0), math::Vec3d(10.0, 0.0, 0.0));
  directions->emplace_back(10.0, 20.0, DrivingDirection::kDirectionUnknown, math::Vec3d(10.0, 0.0, 0.0), math::Vec3d(20.0, 0.0, 0.0));
  res = local_route.RebuildDirections();
  EXPECT_TRUE(res);
}

TEST_F(LocalRouterTest, RebuildSourceInfosTest) {
  LocalRoute local_route;
  
  auto res = local_route.RebuildSourceInfos();
  EXPECT_TRUE(!res);

  auto source_infos = local_route.mutableSourceInfos();
  source_infos->emplace_back(LineSourceType::LocalRouteReferenceLine_LineSource_LineSourceType_kTypePerception, 0.0, 10.0);
  source_infos->emplace_back(LineSourceType::LocalRouteReferenceLine_LineSource_LineSourceType_kTypePerception, 10.0, 20.0);
  res = local_route.RebuildSourceInfos();
  EXPECT_TRUE(res);

  auto res1 = local_route.getSourceInfosFromSRange(25.0, 30.0);
  EXPECT_TRUE(res1.empty());
}

TEST_F(LocalRouterTest, getNeighborLocalRouteBySRangeTest) {
  LocalRoute local_route;

  std::vector<BoundaryCrossRange> boundary_cross_ranges;
  BoundaryCrossRange a, b;
  a.setBoundType(LocalRouteSegment::BoundaryType::DASHED);
  a.setBoundRange(std::make_tuple(true, 0.0, 10.0));
  a.setBoundRangePoints(std::make_pair(LocalRoutePoint("0", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0), LocalRoutePoint("0", 10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)));
  a.setSegmentsId({"a"});
  a.setSegmentsRange({std::make_tuple(true, 0.0, 10.0)});
  a.setSegmentsRangePoints({std::make_pair(LocalRoutePoint("0", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0), LocalRoutePoint("0", 10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))});
  a.setNeighborSegmentsId({std::make_pair("10", std::vector<int>{1})});
  boundary_cross_ranges.emplace_back(a);
  b.setBoundType(LocalRouteSegment::BoundaryType::DASHED);
  b.setBoundRange(std::make_tuple(true, 10.0, 20.0));
  b.setBoundRangePoints(std::make_pair(LocalRoutePoint("0", 10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0), LocalRoutePoint("0", 20.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)));
  b.setSegmentsId({"b"});
  b.setSegmentsRange({std::make_tuple(true, 10.0, 20.0)});
  b.setSegmentsRangePoints({std::make_pair(LocalRoutePoint("0", 10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0), LocalRoutePoint("0", 20.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))});
  b.setNeighborSegmentsId({std::make_pair("11", std::vector<int>{1})});
  boundary_cross_ranges.emplace_back(b);
  auto res = local_route.getNeighborLocalRouteBySRange(boundary_cross_ranges, 0.0, 20.0);

  EXPECT_TRUE(!res.empty());
}

TEST_F(LocalRouterTest, getMergeForkRangesFromSRangeTest) {
  LocalRoute local_route;
  auto merge_fork_ranges = local_route.mutableMergeForkRanges();
  MergeForkRange a, b, c;
  a.setS(10.0);
  b.setS(20.0);
  c.setS(30.0);
  merge_fork_ranges->emplace_back(a);
  merge_fork_ranges->emplace_back(b);
  merge_fork_ranges->emplace_back(c);
  auto res = local_route.getMergeForkRangesFromSRange(15.0, 25.0);
  EXPECT_TRUE(!res.empty());
}

TEST_F(LocalRouterTest, getMergeRangesFromSRangeTest) {
  LocalRoute local_route;
  auto merge_fork_ranges = local_route.mutableMergeForkRanges();
  MergeForkRange a, b, c;
  a.setS(10.0);
  a.setType(MergeForkRange::MergeForkType::MERGE);
  b.setS(20.0);
  b.setType(MergeForkRange::MergeForkType::MERGE);
  c.setS(30.0);
  c.setType(MergeForkRange::MergeForkType::MERGE);
  merge_fork_ranges->emplace_back(a);
  merge_fork_ranges->emplace_back(b);
  merge_fork_ranges->emplace_back(c);
  auto res = local_route.getMergeRangesFromSRange(15.0, 25.0);
  EXPECT_TRUE(!res.empty());
}

TEST_F(LocalRouterTest, getForkRangesFromSRangeTest) {
  LocalRoute local_route;
  auto merge_fork_ranges = local_route.mutableMergeForkRanges();
  MergeForkRange a, b, c;
  a.setS(10.0);
  a.setType(MergeForkRange::MergeForkType::FORK);
  b.setS(20.0);
  b.setType(MergeForkRange::MergeForkType::FORK);
  c.setS(30.0);
  c.setType(MergeForkRange::MergeForkType::FORK);
  merge_fork_ranges->emplace_back(a);
  merge_fork_ranges->emplace_back(b);
  merge_fork_ranges->emplace_back(c);
  auto res = local_route.getForkRangesFromSRange(15.0, 25.0);
  EXPECT_TRUE(!res.empty());
}

TEST_F(LocalRouterTest, getStopLinesFromSRangeTest) {
  LocalRoute local_route;
  auto stop_lines = local_route.mutableStopLines();
  StopLine a;
  a.s = 10.0;
  stop_lines->emplace_back(a);
  auto res = local_route.getStopLinesFromSRange(15.0, 25.0);
  EXPECT_TRUE(res.empty());
}

TEST_F(LocalRouterTest, getGatesFromSRangeTest) {
  LocalRoute local_route;
  auto gates = local_route.mutableGates();
  Gate a;
  a.s = 10.0;
  gates->emplace_back(a);
  auto res = local_route.getGatesFromSRange(15.0, 25.0);
  EXPECT_TRUE(res.empty());
}

TEST_F(LocalRouterTest, getSpeedLimitFromSTest) {
  LocalRoute local_route;
  auto speed_limits = local_route.mutableSpeedLimits();
  SpeedLimit a;
  a.start_s = 10.0;
  a.end_s = 20.0;
  speed_limits->emplace_back(a);
  auto res = local_route.getSpeedLimitFromS(5.0);
  EXPECT_TRUE(res.end_s < 1e-6);
}

TEST_F(LocalRouterTest, getDirectionsFromSRangeTest) {
  LocalRoute local_route;
  auto directions = local_route.mutableDirections();
  SegmentDirection a;
  a.start_s = 10.0;
  a.end_s = 20.0;
  directions->emplace_back(a);
  auto res = local_route.getDirectionsFromSRange(25.0, 30.0);
  EXPECT_TRUE(res.empty());

  auto res1 = local_route.getDirectionFromS(25.0);
  EXPECT_TRUE(res1.end_s < 1e-6);
}

TEST_F(LocalRouterTest, getBoundaryTypesFromSRangeTest) {
  LocalRoute local_route;
  auto boundary_types = local_route.mutableBoundaryTypes();
  SegmentBoundaryType a;
  a.start_s = 10.0;
  a.end_s = 20.0;
  boundary_types->emplace_back(a);
  auto res = local_route.getBoundaryTypesFromSRange(25.0, 30.0);
  EXPECT_TRUE(res.empty());

  auto res1 = local_route.getBoundaryTypeFromS(25.0);
  EXPECT_TRUE(res1.end_s < 1e-6);
}


TEST_F(LocalRouterTest, localRoutesSubsetRelationProcessTest) {
  LocalRoute local_route0, local_route1;
  local_route0.setId(1);
  auto valid_range0 = local_route0.mutableValidRange();
  std::get<0>(*valid_range0) = true;
  std::get<1>(*valid_range0) = 0.0;
  std::get<2>(*valid_range0) = 10.0;
  std::get<3>(*valid_range0).emplace_back("a");
  std::get<3>(*valid_range0).emplace_back("b");
  local_route1.setId(2);
  auto valid_range1 = local_route1.mutableValidRange();
  std::get<0>(*valid_range1) = true;
  std::get<1>(*valid_range1) = 0.0;
  std::get<2>(*valid_range1) = 10.0;
  std::get<3>(*valid_range1).emplace_back("b");
  
  auto local_routes_ = local_router_.getMutableLocalRoutes();
  local_routes_->emplace(local_route0.id(), local_route0);
  local_routes_->emplace(local_route1.id(), local_route1);
  local_router_.localRoutesSubsetRelationProcess();
  EXPECT_TRUE(true);
}

TEST_F(LocalRouterTest, judgeSegmentsNeighborRelationTest) {
  auto lanes_segments_map = local_router_.getMutableLanesSegmentsMap();
  std::unordered_map<std::string, LocalRouteSegment> lane_segs1, lane_segs2, lane_segs3, lane_segs4;
  LocalRouteSegment a, b, c, d;
  a.setLeftNeighborId("b");
  b.setLeftNeighborId("c");
  c.setLeftNeighborId("d");
  d.setLeftNeighborId("");
  a.setRightNeighborId("");
  b.setRightNeighborId("a");
  c.setRightNeighborId("b");
  d.setRightNeighborId("c");
  lane_segs1.emplace("a", a);
  lane_segs2.emplace("b", b);
  lane_segs3.emplace("c", c);
  lane_segs4.emplace("d", d);
  lanes_segments_map->emplace("1", lane_segs1);
  lanes_segments_map->emplace("2", lane_segs2);
  lanes_segments_map->emplace("3", lane_segs3);
  lanes_segments_map->emplace("4", lane_segs4);

  auto res = local_router_.judgeSegmentsNeighborRelation("a", {"d"});
  EXPECT_TRUE(std::get<0>(res) != 0);

  res = local_router_.judgeSegmentsNeighborRelation("d", {"a"});
  EXPECT_TRUE(std::get<0>(res) != 0);
}


TEST_F(LocalRouterTest, validPolicyBySpecifiedRangeTest) {
  LocalRoute local_route1, local_route2;
  local_route1.setId(1);
  auto valid_range1 = local_route1.mutableValidRange();
  std::get<0>(*valid_range1) = true;
  std::get<1>(*valid_range1) = 0.0;
  std::get<2>(*valid_range1) = 30.0;
  std::get<3>(*valid_range1).emplace_back("a");
  local_route1.setAdcS(1.0);
  auto lanes_segments1 = local_route1.mutableLanesSegments();
  LocalRouteSegment a;
  a.setId("a");
  a.setType(PerceptionLaneSegment::LaneSegmentType::kTypeVirtual);
  a.setDirection(DrivingDirection::kDirectionUTurnOnly);
  a.mutableGuidePoints()->emplace_back(LocalRoutePoint("a", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
  a.mutableGuidePoints()->emplace_back(LocalRoutePoint("a", 10.0, 0.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 0.0));
  lanes_segments1->emplace_back("1", std::vector<LocalRouteSegment>{a});

  local_route2.setId(2);
  auto valid_range2 = local_route2.mutableValidRange();
  std::get<0>(*valid_range2) = true;
  std::get<1>(*valid_range2) = 0.0;
  std::get<2>(*valid_range2) = 500.0;
  std::get<3>(*valid_range2).emplace_back("b");
  local_route2.setAdcS(300.0);
  auto lanes_segments2 = local_route2.mutableLanesSegments();
  LocalRouteSegment b, c;
  b.setId("b");
  b.mutableGuidePoints()->emplace_back(LocalRoutePoint("b", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
  b.mutableGuidePoints()->emplace_back(LocalRoutePoint("b", 100.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0));
  c.setId("c");
  c.mutableGuidePoints()->emplace_back(LocalRoutePoint("c", 100.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0));
  c.mutableGuidePoints()->emplace_back(LocalRoutePoint("c", 500.0, 0.0, 0.0, 0.0, 0.0, 500.0, 0.0, 0.0, 0.0, 0.0));
  lanes_segments2->emplace_back("2", std::vector<LocalRouteSegment>{b, c});

  auto local_routes_ = local_router_.getMutableLocalRoutes();
  local_routes_->emplace(local_route1.id(), local_route1);
  auto res = local_router_.validPolicyBySpecifiedRange(local_route1);
  EXPECT_TRUE(std::get<0>(res) == 0);

  local_routes_->emplace(local_route2.id(), local_route2);
  res = local_router_.validPolicyBySpecifiedRange(local_route2);
  EXPECT_TRUE(std::get<0>(res) != 0);

  res = local_router_.validPolicyByNavigationScore(local_route1);
  EXPECT_TRUE(std::get<0>(res) == 0);
}

TEST_F(LocalRouterTest, selectLocalRouteForSmoothTest) {
  LocalRoute local_route1;
  local_route1.setId(1);
  auto valid_range1 = local_route1.mutableValidRange();
  std::get<0>(*valid_range1) = true;
  std::get<1>(*valid_range1) = 0.0;
  std::get<2>(*valid_range1) = 100.0;
  std::get<3>(*valid_range1).emplace_back("a");
  local_route1.setAdcS(1.0);
  local_route1.setAdcHeadingDiff(M_PI);
  auto lanes_segments1 = local_route1.mutableLanesSegments();
  LocalRouteSegment b, c;
  b.setId("b");
  b.mutableGuidePoints()->emplace_back(LocalRoutePoint("b", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
  b.mutableGuidePoints()->emplace_back(LocalRoutePoint("b", 100.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0));
  lanes_segments1->emplace_back("1", std::vector<LocalRouteSegment>{b});

  auto local_routes_ = local_router_.getMutableLocalRoutes();
  local_routes_->emplace(local_route1.id(), local_route1);
  local_router_.selectLocalRouteForSmooth();
  EXPECT_TRUE(true);
}

TEST_F(LocalRouterTest, rebuildMergeForkRangesByLocalRouteIdTest) {
  LocalRoute local_route1;
  local_route1.setId(1);
  auto merge_fork_ranges = local_route1.mutableMergeForkRanges();
  MergeForkRange a;
  a.setS(10.0);
  a.setType(MergeForkRange::MergeForkType::FORK);
  a.mutableRelationIdsInfo()->emplace_back("a", "a1", std::vector<std::string>{"1"});
  merge_fork_ranges->emplace_back(a);
  auto lanes_segments1 = local_route1.mutableLanesSegments();
  LocalRouteSegment a1;
  lanes_segments1->emplace_back("a", std::vector<LocalRouteSegment>{a1});

  auto local_routes_ = local_router_.getMutableLocalRoutes();
  local_routes_->emplace(local_route1.id(), local_route1);
  local_router_.rebuildMergeForkRangesByLocalRouteId();
  EXPECT_TRUE(true);
}

TEST_F(LocalRouterTest, rebuildBoundaryCrossRangesByLocalRouteIdTest) {
  LocalRoute local_route1;
  local_route1.setId(1);
  
  BoundaryCrossRange a;
  a.setBoundType(LocalRouteSegment::BoundaryType::DASHED);
  a.setBoundRange(std::make_tuple(true, 0.0, 10.0));
  a.setBoundRangePoints(std::make_pair(LocalRoutePoint("0", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0), LocalRoutePoint("0", 10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)));
  a.setSegmentsId({"a"});
  a.setSegmentsRange({std::make_tuple(true, 0.0, 10.0)});
  a.setSegmentsRangePoints({std::make_pair(LocalRoutePoint("0", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0), LocalRoutePoint("0", 10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))});
  a.setNeighborSegmentsId({std::make_pair("a", std::vector<int>{1})});
  local_route1.mutableLeftBoundaryCrossRanges()->emplace_back(a);
  local_route1.mutableRightBoundaryCrossRanges()->emplace_back(a);
  auto lanes_segments1 = local_route1.mutableLanesSegments();
  LocalRouteSegment a1;
  a1.setId("a");
  lanes_segments1->emplace_back("a", std::vector<LocalRouteSegment>{a1});

  auto local_routes_ = local_router_.getMutableLocalRoutes();
  local_routes_->emplace(local_route1.id(), local_route1);
  local_router_.rebuildBoundaryCrossRangesByLocalRouteId();
  EXPECT_TRUE(true);
}

TEST_F(LocalRouterTest, generateUniqueLocalRouteIdTest) {
  LocalRouter local_router1, local_router2, local_router3;
  local_router1.reset();
  local_router2.reset();
  local_router3.reset();
  auto history_local_routes_id_vec1 = local_router1.mutableHistoryLocalRoutesIdVec();
  history_local_routes_id_vec1->emplace_back(false, 1, std::vector<std::string>{"a"});
  history_local_routes_id_vec1->emplace_back(false, 2, std::vector<std::string>{"b", "c"});
  LocalRoute local_route1;
  local_route1.mutableLanesSegments()->clear();
  auto res = local_router1.generateUniqueLocalRouteId(local_route1);
  std::cout << "res = "<<res<<std::endl;
  EXPECT_TRUE(true);

  auto history_local_routes_id_vec2 = local_router2.mutableHistoryLocalRoutesIdVec();
  history_local_routes_id_vec2->emplace_back(false, 1, std::vector<std::string>{"a"});
  history_local_routes_id_vec2->emplace_back(false, 2, std::vector<std::string>{"b", "c"});
  LocalRoute local_route2;
  auto lanes_segments1 = local_route2.mutableLanesSegments();
  LocalRouteSegment a1;
  a1.setId("a");
  lanes_segments1->emplace_back("a", std::vector<LocalRouteSegment>{a1});
  res = local_router2.generateUniqueLocalRouteId(local_route2);
  EXPECT_TRUE(true);

  auto history_local_routes_id_vec3 = local_router3.mutableHistoryLocalRoutesIdVec();
  history_local_routes_id_vec3->emplace_back(false, 1, std::vector<std::string>{"a"});
  history_local_routes_id_vec3->emplace_back(false, 2, std::vector<std::string>{"b", "c"});
  LocalRoute local_route3;
  auto lanes_segments2 = local_route3.mutableLanesSegments();
  LocalRouteSegment b1, d1;
  b1.setId("b");
  d1.setId("d");
  lanes_segments2->emplace_back("b", std::vector<LocalRouteSegment>{b1});
  lanes_segments2->emplace_back("d", std::vector<LocalRouteSegment>{d1});
  res = local_router3.generateUniqueLocalRouteId(local_route3);
  EXPECT_TRUE(true);
}

}  // namespace gpal::pnc::planning
