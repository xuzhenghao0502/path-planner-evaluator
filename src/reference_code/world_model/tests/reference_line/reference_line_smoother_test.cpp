#include <base/singleton.h>
#include <fmt/chrono.h>
#include <gtest/gtest.h>

#include "base/log.h"

#include "config_manager/config_manager.h"

#define private public
#include "reference_line/ocp_line_smoother.h"
#include "reference_line/ocp_reference_line_smoother.h"
#include "reference_line/reference_line_provider.h"


namespace gpal::pnc::planning {

class ReferenceLineSmootherTest : public ::testing::Test {
 protected:
  void SetUp() override {
    int argc = 0;
    char path[] = "policy_planner";
    char* argv[]{path};
    if (!Singleton<ConfigManager>::get_instance()->init(argc, argv)) {
      ERT_PLOG_I << "Failed to init config!";
    }
  }
};

TEST_F(ReferenceLineSmootherTest, SlicedRouteSmoothTest) {
  std::vector<RoutePoint> pts;
  std::shared_ptr<std::vector<RoutePoint>> stop_line_points = std::make_shared<std::vector<RoutePoint>>();
  std::shared_ptr<std::vector<SpeedLimit>> speed_limits = std::make_shared<std::vector<SpeedLimit>>();
  std::shared_ptr<std::vector<SegmentDirection>> segments_direction = std::make_shared<std::vector<SegmentDirection>>();
  std::shared_ptr<std::vector<std::tuple<bool, float, float, bool, bool>>> navigation_lane_change_ranges =
      std::make_shared<std::vector<std::tuple<bool, float, float, bool, bool>>>();
  std::shared_ptr<std::vector<std::tuple<bool, float, float>>> lane_follow_ranges =
      std::make_shared<std::vector<std::tuple<bool, float, float>>>();
  std::vector<ReferenceLine> raw_refs, smoothed_refs;
  ReferenceLineProvider reference_line_provider;

  // case 1
  std::shared_ptr<SlicedRoute> route_1 =
      std::make_shared<SlicedRoute>(pts, 0, "a", stop_line_points, speed_limits, segments_direction,
                                    navigation_lane_change_ranges, lane_follow_ranges);
  auto res = reference_line_provider.smoothReferenceLines(route_1, &smoothed_refs);
  EXPECT_FALSE(res);
  res = reference_line_provider.addRawReferenceLines(route_1, &raw_refs);
  EXPECT_FALSE(res);
  res = reference_line_provider.addRawReferenceLines(nullptr, &raw_refs);
  EXPECT_FALSE(res);

  // case 2
  RoutePoint pt;
  for (int i = 1; i < 50; i++) {
    pt.set_x(i);
    pt.setS(i);
    pts.emplace_back(pt);
  }
  navigation_lane_change_ranges->emplace_back(true, 0.0, 49.0, true, false);
  lane_follow_ranges->emplace_back(true, 0.0, 49.0);
  std::shared_ptr<SlicedRoute> route_2 =
      std::make_shared<SlicedRoute>(pts, 0, "a", stop_line_points, speed_limits, segments_direction,
                                    navigation_lane_change_ranges, lane_follow_ranges);
  route_2->setIsParkSlicedRoute(true);
  route_2->setIsEndSlicedRoute(true);
  res = reference_line_provider.smoothReferenceLines(route_2, &smoothed_refs);
  EXPECT_TRUE(res);

  // case 3
  res = reference_line_provider.addRawReferenceLines(route_2, &raw_refs);
  EXPECT_TRUE(res);

  // case 5
  reference_line_provider.setEnableSmoothRefLine(false);
  res = reference_line_provider.smoothReferenceLines(route_2, &smoothed_refs);
  EXPECT_TRUE(res);
}

TEST_F(ReferenceLineSmootherTest, PerceptionLaneSmoothTest) {
  std::vector<ReferenceLine> raw_refs, smoothed_refs;
  ReferenceLineProvider reference_line_provider;

  std::vector<PerceptionLane> lanes_1;
  PerceptionLane lane_1;
  lane_1.id = "a";
  LaneSegment seg_1;
  for (int i = 0; i < 3; i++) {
    seg_1.center_line.emplace_back(math::Vec3d(i, 0.0, 0.0));
    seg_1.offset.emplace_back(1.875);
  }
  lane_1.lane_segments.emplace_back(seg_1);
  lanes_1.emplace_back(lane_1);
  auto res = reference_line_provider.smoothReferenceLines(lanes_1, &smoothed_refs);
  EXPECT_FALSE(res);
  res = reference_line_provider.addRawReferenceLines(lanes_1, &raw_refs);
  EXPECT_TRUE(res);

  std::vector<PerceptionLane> lanes_2;
  PerceptionLane lane_2;
  lane_2.id = "a";
  LaneSegment seg_2;
  for (int i = 0; i < 50; i++) {
    seg_2.center_line.emplace_back(math::Vec3d(i, 0.0, 0.0));
    seg_2.offset.emplace_back(1.875);
  }
  lane_2.lane_segments.emplace_back(seg_2);
  lanes_2.emplace_back(lane_2);

  res = reference_line_provider.smoothReferenceLines(lanes_2, &smoothed_refs);
  EXPECT_TRUE(res);
  res = reference_line_provider.addRawReferenceLines(lanes_2, &raw_refs);
  EXPECT_TRUE(res);
  reference_line_provider.setEnableSmoothRefLine(false);
  res = reference_line_provider.smoothReferenceLines(lanes_2, &smoothed_refs);
  EXPECT_TRUE(res);
}

TEST_F(ReferenceLineSmootherTest, LocalRouteSmoothTest) {
  std::vector<ReferenceLine> raw_refs, smoothed_refs;
  ReferenceLineProvider reference_line_provider;

  std::map<int, LocalRoute> local_routes_1;
  LocalRoute local_route_1, local_route_2, local_route_3, local_route_4;

  // case 1
  local_route_1.setId(1);
  for (int i = 0; i < 500; i++) {
    LocalRoutePoint pt;
    pt.setX(i);
    pt.setS(i);
    pt.setLeftLaneWidth(1.875);
    pt.setRightLaneWidth(1.875);
    pt.setLeftRoadWidth(2.0);
    pt.setRightRoadWidth(2.0);
    local_route_1.mutableGuideLine()->emplace_back(pt);
  }
  local_route_1.setAdcS(150.0);
  local_route_1.setAdcL(0.1);
  local_route_1.setGlobalStartS(0.0);
  std::vector<std::string> tmp;
  local_route_1.setValidRange({true, 100.0, 450.0, tmp});
  local_route_1.mutableSpeedLimits()->emplace_back(100.0, 150.0, 20.0, 0.0, math::Vec3d(100.0, 0.0, 0.0),
                                                   math::Vec3d(150.0, 0.0, 0.0));
  local_route_1.mutableDirections()->emplace_back(100.0, 150.0, DrivingDirection::kDirectionForwardOnly,
                                                  math::Vec3d(100.0, 0.0, 0.0), math::Vec3d(150.0, 0.0, 0.0));
  local_route_1.mutableDirections()->emplace_back(150.0, 200.0, DrivingDirection::kDirectionLeftOnly,
                                                  math::Vec3d(150.0, 0.0, 0.0), math::Vec3d(200.0, 0.0, 0.0));
  local_route_1.mutableDirections()->emplace_back(200.0, 250.0, DrivingDirection::kDirectionRightOnly,
                                                  math::Vec3d(200.0, 0.0, 0.0), math::Vec3d(250.0, 0.0, 0.0));
  local_route_1.mutableDirections()->emplace_back(250.0, 300.0, DrivingDirection::kDirectionUTurnOnly,
                                                  math::Vec3d(250.0, 0.0, 0.0), math::Vec3d(300.0, 0.0, 0.0));
  local_route_1.mutableDirections()->emplace_back(300.0, 490.0, DrivingDirection::kDirectionInvalid,
                                                  math::Vec3d(300.0, 0.0, 0.0), math::Vec3d(490.0, 0.0, 0.0));
  local_route_1.mutableSourceInfos()->emplace_back(
      proto::LocalRouteReferenceLine_LineSource_LineSourceType_kTypePerception, 100.0, 350.0);
  local_route_1.mutableBoundaryTypes()->emplace_back(100.0, 150.0, LocalRouteSegment::BoundaryType::DASHED,
                                                     LocalRouteSegment::BoundaryType::SOLID,
                                                     math::Vec3d(100.0, 0.0, 0.0), math::Vec3d(150.0, 0.0, 0.0));
  local_route_1.mutableStopLines()->emplace_back("a", false, 150.0, 0.0, 0.0, 150.0, "0",
                                                 DrivingDirection::kDirectionInvalid);
  local_route_1.mutableGates()->emplace_back("a", proto::perception::Gate::GateStatus::Gate_GateStatus_kStatusUnknown, 150.0, 0.0, 0.0, 150.0,
                                             1.0);
  MergeForkRange mf;
  mf.setS(150.0);
  mf.setPoint(local_route_1.guideLine().at(150));
  local_route_1.mutableMergeForkRanges()->emplace_back(mf);
  std::vector<LocalRouteSegment> segs;
  LocalRouteSegment seg;
  seg.setRange({true, 100.0, 150.0});
  seg.setRangePoints({local_route_1.guideLine().at(100), local_route_1.guideLine().at(150)});
  seg.setNavigationScore(1.0);
  segs.emplace_back(seg);
  local_route_1.mutableLanesSegments()->emplace_back(std::make_pair(seg.id(), segs));
  BoundaryCrossRange left_boundary_cross_range;
  left_boundary_cross_range.setBoundType(LocalRouteSegment::BoundaryType::DASHED);
  left_boundary_cross_range.setBoundRange({true, 100.0, 150.0});
  left_boundary_cross_range.setBoundRangePoints({local_route_1.guideLine().at(100), local_route_1.guideLine().at(150)});
  left_boundary_cross_range.mutableSegmentsId()->emplace_back("a");
  left_boundary_cross_range.mutableSegmentsRange()->emplace_back(std::tuple<bool, float, float>{true, 100.0, 150.0});
  left_boundary_cross_range.mutableSegmentsRangePoints()->emplace_back(std::pair<LocalRoutePoint, LocalRoutePoint>{
      local_route_1.guideLine().at(100), local_route_1.guideLine().at(150)});
  left_boundary_cross_range.mutableNeighborSegmentsId()->emplace_back(
      std::pair<std::string, std::vector<int>>{"b", {2}});
  local_route_1.mutableLeftBoundaryCrossRanges()->emplace_back(left_boundary_cross_range);
  BoundaryCrossRange right_boundary_cross_range;
  right_boundary_cross_range.setBoundType(LocalRouteSegment::BoundaryType::DASHED);
  right_boundary_cross_range.setBoundRange({true, 100.0, 150.0});
  right_boundary_cross_range.setBoundRangePoints(
      {local_route_1.guideLine().at(100), local_route_1.guideLine().at(150)});
  right_boundary_cross_range.mutableSegmentsId()->emplace_back("a");
  right_boundary_cross_range.mutableSegmentsRange()->emplace_back(std::tuple<bool, float, float>{true, 100.0, 150.0});
  right_boundary_cross_range.mutableSegmentsRangePoints()->emplace_back(std::pair<LocalRoutePoint, LocalRoutePoint>{
      local_route_1.guideLine().at(100), local_route_1.guideLine().at(150)});
  right_boundary_cross_range.mutableNeighborSegmentsId()->emplace_back(
      std::pair<std::string, std::vector<int>>{"b", {2}});
  local_route_1.mutableRightBoundaryCrossRanges()->emplace_back(right_boundary_cross_range);
  local_routes_1[local_route_1.id()] = local_route_1;

  // case 2
  local_route_2 = local_route_1;
  local_route_2.setId(2);
  local_route_2.setValidRange({false, 0.0, 0.0, tmp});
  local_routes_1[local_route_2.id()] = local_route_2;

  // case 3
  local_route_3.setId(3);
  for (int i = 0; i < 50; i++) {
    LocalRoutePoint pt;
    pt.setX(i);
    pt.setS(i);
    pt.setLeftLaneWidth(1.875);
    pt.setRightLaneWidth(1.875);
    pt.setLeftRoadWidth(2.0);
    pt.setRightRoadWidth(2.0);
    local_route_3.mutableGuideLine()->emplace_back(pt);
  }
  local_route_3.setValidRange({true, 10.0, 30.0, tmp});
  local_routes_1[local_route_3.id()] = local_route_3;

  // case 4
  local_route_4.setId(4);
  for (int i = 0; i < 3; i++) {
    LocalRoutePoint pt;
    pt.setX(i);
    pt.setS(i);
    pt.setLeftLaneWidth(1.875);
    pt.setRightLaneWidth(1.875);
    pt.setLeftRoadWidth(2.0);
    pt.setRightRoadWidth(2.0);
    local_route_4.mutableGuideLine()->emplace_back(pt);
  }
  local_route_4.setValidRange({true, 0.0, 2.0, tmp});
  local_routes_1[local_route_4.id()] = local_route_4;

  auto res = reference_line_provider.smoothReferenceLines(local_routes_1, &smoothed_refs);
  EXPECT_TRUE(res);
  res = reference_line_provider.addRawReferenceLines(local_routes_1, &raw_refs);
  EXPECT_TRUE(res);
}

TEST_F(ReferenceLineSmootherTest, PathHistoryTest) {
  std::vector<ReferenceLine> raw_refs, smoothed_refs;
  ReferenceLineProvider reference_line_provider;

  MainPathHistory path_history_1;
  path_history_1.id = "a";
  for (int i = 0; i < 50; i++) {
    path_history_1.values.emplace_back(math::Vec3d(i, 0.0, 0.0));
  }
  auto res = reference_line_provider.smoothReferenceLines(path_history_1, &smoothed_refs);
  EXPECT_TRUE(res);

  MainPathHistory path_history_2;
  path_history_2.id = "a";
  for (int i = 0; i < 3; i++) {
    path_history_2.values.emplace_back(math::Vec3d(i, 0.0, 0.0));
  }
  res = reference_line_provider.smoothReferenceLines(path_history_2, &smoothed_refs);
  EXPECT_TRUE(res);

  MainPathHistory path_history_3;
  path_history_3.id = "a";
  for (int i = 0; i < 50; i++) {
    path_history_3.values.emplace_back(math::Vec3d(i, 0.0, 0.0));
    path_history_3.values.emplace_back(math::Vec3d(-i, 0.0, 0.0));
  }
  res = reference_line_provider.smoothReferenceLines(path_history_3, &smoothed_refs);
  EXPECT_TRUE(res);
}

TEST_F(ReferenceLineSmootherTest, ReferenceLineOcpsmoothTest) {
  ConfigManager* config_manager = Singleton<ConfigManager>::get_instance();
  ReferenceLineSmootherConfig smoother_config =
      config_manager->getConfig<ReferenceLineSmootherConfig>("OcpSmootherConfig");
  smoother_config.set_enable_data_recorder(true);
  smoother_config.set_enable_evaluation(true);
  std::unique_ptr<OcpReferenceLineSmoother> smoother = std::make_unique<OcpReferenceLineSmoother>(smoother_config);
  ReferenceLine smoothed_refs;

  // case 1
  std::vector<math::Vec3d> pts_1;
  for (int i = 0; i < 50; i++) {
    pts_1.emplace_back(math::Vec3d(i, 0.0, 0.0));
  }
  ReferenceLine raw_reference_line_1(pts_1);
  auto res = smoother->smooth(raw_reference_line_1, &smoothed_refs, true);
  EXPECT_TRUE(res);

  // case 2
  std::vector<math::Vec3d> pts_2;
  for (int i = 0; i < 50; i++) {
    pts_2.emplace_back(math::Vec3d(i, 0.0, 0.0));
    pts_2.emplace_back(math::Vec3d(-i, 0.0, 0.0));
  }
  ReferenceLine raw_reference_line_2(pts_2);
  res = smoother->smooth(raw_reference_line_2, &smoothed_refs, true);
  EXPECT_FALSE(res);
}

TEST_F(ReferenceLineSmootherTest, LineOcpsmoothTest) {
  ConfigManager* config_manager = Singleton<ConfigManager>::get_instance();

  // case 1
  ReferenceLineSmootherConfig smoother_config_1 =
      config_manager->getConfig<ReferenceLineSmootherConfig>("OcpSmootherConfig");
  std::unique_ptr<OcpLineSmoother> smoother_1 = std::make_unique<OcpLineSmoother>(smoother_config_1);

  std::vector<math::Vec3d> raw_line_1, res_line;
  for (int i = 0; i < 50; i++) {
    raw_line_1.emplace_back(math::Vec3d(i, 0.0, 0.0));
  }
  auto res = smoother_1->smooth(raw_line_1, &res_line);
  EXPECT_TRUE(res);

  // case 2
  ReferenceLineSmootherConfig smoother_config_2 =
      config_manager->getConfig<ReferenceLineSmootherConfig>("OcpSmootherConfig");
  smoother_config_2.set_enable_data_recorder(true);
  std::unique_ptr<OcpLineSmoother> smoother_2 = std::make_unique<OcpLineSmoother>(smoother_config_2);
  std::vector<math::Vec3d> raw_line_2;
  for (int i = 0; i < 50; i++) {
    raw_line_2.emplace_back(math::Vec3d(i, 0.0, 0.0));
    raw_line_2.emplace_back(math::Vec3d(-i, 0.0, 0.0));
  }
  res = smoother_2->smooth(raw_line_2, &res_line);
  EXPECT_FALSE(res);

  // case 3
  std::vector<math::Vec3d> raw_line_3;
  res = smoother_2->smooth(raw_line_3, &res_line);
  EXPECT_FALSE(res);
}
}  // namespace gpal::pnc::planning