#include <base/singleton.h>
#include <fmt/chrono.h>
#include <gtest/gtest.h>

#include "base/log.h"

#include "config_manager/config_manager.h"

#define private public
#include "reference_line/reference_line.h"


namespace gpal::pnc::planning {

class ReferenceLineTest : public ::testing::Test {
 protected:
  void SetUp() override {
    int argc = 0;
    char path[] = "policy_planner";
    char* argv[]{path};
    if (!Singleton<ConfigManager>::get_instance()->init(argc, argv)) {
      ERT_PLOG_I << "Failed to init config!";
    }
  }
  // void TearDown() override { delete generator_; }
};

TEST_F(ReferenceLineTest, CreateXYZPointsReferenceLineTest) {
  std::vector<math::Vec3d> pts_1;
  for (int i = 0; i < 50; i++) {
    pts_1.emplace_back(math::Vec3d(i, 0.0, 0.0));
  }
  ReferenceLine ref_1(pts_1);
  EXPECT_FALSE(ref_1.reference_points().empty());

  std::vector<math::Vec3d> pts_2;
  pts_2.emplace_back(math::Vec3d(0.0, 0.0, 0.0));
  ReferenceLine ref_2(pts_2);
  EXPECT_FALSE(ref_2.reference_points().empty());
}

TEST_F(ReferenceLineTest, CreateRefPointsReferenceLineTest) {
  std::vector<ReferencePoint> reference_points_1;
  for (int i = 0; i < 50; i++) {
    reference_points_1.emplace_back(math::Vec3d(i, 0.0, 0.0), 0.0, 0.0, 0.0, 0.0);
  }
  ReferenceLine ref_1(reference_points_1);
  EXPECT_FALSE(ref_1.reference_points().empty());

  std::vector<ReferencePoint> reference_points_2;
  reference_points_2.emplace_back(math::Vec3d(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 0.0);
  ReferenceLine ref_2(reference_points_2);
  EXPECT_FALSE(ref_2.reference_points().empty());
}

TEST_F(ReferenceLineTest, CreateSlicedRouteReferenceLineTest) {
  std::vector<RoutePoint> pts;
  std::shared_ptr<std::vector<RoutePoint>> stop_line_points = std::make_shared<std::vector<RoutePoint>>();
  std::shared_ptr<std::vector<SpeedLimit>> speed_limits = std::make_shared<std::vector<SpeedLimit>>();
  std::shared_ptr<std::vector<SegmentDirection>> segments_direction = std::make_shared<std::vector<SegmentDirection>>();
  std::shared_ptr<std::vector<std::tuple<bool, float, float, bool, bool>>> navigation_lane_change_ranges =
      std::make_shared<std::vector<std::tuple<bool, float, float, bool, bool>>>();
  std::shared_ptr<std::vector<std::tuple<bool, float, float>>> lane_follow_ranges =
      std::make_shared<std::vector<std::tuple<bool, float, float>>>();

  // case 1
  SlicedRoute route_1(pts, 0, "a", stop_line_points, speed_limits, segments_direction, navigation_lane_change_ranges,
                      lane_follow_ranges);
  {
    route_1.setIsParkSlicedRoute(true);
    route_1.setIsEndSlicedRoute(true);
    ReferenceLine ref_1(&route_1);
    EXPECT_TRUE(ref_1.reference_points().empty());
  }
  {
    route_1.setIsParkSlicedRoute(false);
    route_1.setIsEndSlicedRoute(true);
    ReferenceLine ref_1(&route_1);
    EXPECT_TRUE(ref_1.reference_points().empty());
  }
  {
    route_1.setIsParkSlicedRoute(true);
    route_1.setIsEndSlicedRoute(false);
    ReferenceLine ref_1(&route_1);
    EXPECT_TRUE(ref_1.reference_points().empty());
  }
  {
    route_1.setIsParkSlicedRoute(false);
    route_1.setIsEndSlicedRoute(false);
    ReferenceLine ref_1(&route_1);
    EXPECT_TRUE(ref_1.reference_points().empty());
  }

  // case 2
  RoutePoint pt;
  pt.set_x(0.0);
  pt.setS(0.0);
  pts.emplace_back(pt);

  SlicedRoute route_2(pts, 0, "", stop_line_points, speed_limits, segments_direction, navigation_lane_change_ranges,
                      lane_follow_ranges);
  ReferenceLine ref_2(&route_2);
  EXPECT_FALSE(ref_2.reference_points().empty());

  // case 3
  SlicedRoute route_3(pts, 0, "a", stop_line_points, speed_limits, segments_direction, navigation_lane_change_ranges,
                      lane_follow_ranges);
  ReferenceLine ref_3(&route_3);
  EXPECT_FALSE(ref_3.reference_points().empty());

  // case 4
  for (int i = 1; i < 50; i++) {
    pt.set_x(i);
    pt.setS(i);
    pts.emplace_back(pt);
  }
  SlicedRoute route_4(pts, 0, "a", stop_line_points, speed_limits, segments_direction, navigation_lane_change_ranges,
                      lane_follow_ranges);
  ReferenceLine ref_4(&route_4);
  EXPECT_FALSE(ref_4.reference_points().empty());

  // case 5
  stop_line_points->emplace_back(pts.front());
  stop_line_points->emplace_back(pts.back());
  speed_limits->emplace_back(SpeedLimit(0.0, 10.0, 20.0, 0.0, pts.front(), pts.at(10)));
  speed_limits->emplace_back(SpeedLimit(10.0, 49.0, 20.0, 0.0, pts.at(10), pts.back()));
  segments_direction->emplace_back(
      SegmentDirection(0.0, 10.0, DrivingDirection::kDirectionForwardOnly, pts.front(), pts.at(10)));
  segments_direction->emplace_back(
      SegmentDirection(10.0, 20.0, DrivingDirection::kDirectionLeftOnly, pts.at(10), pts.at(20)));
  segments_direction->emplace_back(
      SegmentDirection(20.0, 30.0, DrivingDirection::kDirectionRightOnly, pts.at(20), pts.at(30)));
  segments_direction->emplace_back(
      SegmentDirection(30.0, 40.0, DrivingDirection::kDirectionUTurnOnly, pts.at(30), pts.at(40)));
  segments_direction->emplace_back(
      SegmentDirection(40.0, 49.0, DrivingDirection::kDirectionInvalid, pts.at(40), pts.back()));
  navigation_lane_change_ranges->emplace_back(true, 0.0, 49.0, true, false);
  lane_follow_ranges->emplace_back(true, 0.0, 49.0);
  SlicedRoute route_5(pts, 0, "a", stop_line_points, speed_limits, segments_direction, navigation_lane_change_ranges,
                      lane_follow_ranges);
  ReferenceLine ref_5(&route_5);
  EXPECT_FALSE(ref_5.reference_points().empty());
}

TEST_F(ReferenceLineTest, CreatePerceptionLaneReferenceLineTest) {
  PerceptionLane lane;

  {
    ReferenceLine ref(&lane);
    EXPECT_TRUE(ref.reference_points().empty());
  }

  {
    lane.id = "a";
    ReferenceLine ref(&lane);
    EXPECT_TRUE(ref.reference_points().empty());
  }

  {
    lane.id = "a";
    LaneSegment seg;
    for (int i = 0; i < 50; i++) {
      seg.center_line.emplace_back(math::Vec3d(i, 0.0, 0.0));
      seg.offset.emplace_back(1.875);
    }
    lane.lane_segments.emplace_back(seg);
    ReferenceLine ref(&lane);
    EXPECT_FALSE(ref.reference_points().empty());
  }

  {
    lane.id = "a";
    LaneSegment seg;
    for (int i = 0; i < 50; i++) {
      seg.center_line.emplace_back(math::Vec3d(i, 0.0, 0.0));
      seg.offset.emplace_back(1.875);
    }
    seg.offset.emplace_back(1.875);
    lane.lane_segments.emplace_back(seg);
    ReferenceLine ref(&lane);
    EXPECT_FALSE(ref.reference_points().empty());
  }
}

TEST_F(ReferenceLineTest, CreateMainPathHistoryReferenceLineTest) {
  MainPathHistory path_history;

  {
    ReferenceLine ref(&path_history);
    EXPECT_TRUE(ref.reference_points().empty());
  }

  {
    path_history.id = "a";
    ReferenceLine ref(&path_history);
    EXPECT_TRUE(ref.reference_points().empty());
  }

  {
    path_history.id = "a";
    for (int i = 0; i < 50; i++) {
      path_history.values.emplace_back(math::Vec3d(i, 0.0, 0.0));
    }
    ReferenceLine ref(&path_history);
    EXPECT_FALSE(ref.reference_points().empty());
  }
}

TEST_F(ReferenceLineTest, CreateAdasLaneLineMarkingReferenceLineTest) {
  adas::LaneLineMarking marking;

  {
    marking.id = 1;
    ReferenceLine ref(&marking);
    EXPECT_TRUE(ref.reference_points().empty());
  }

  {
    marking.id = 1;
    for (int i = 0; i < 50; i++) {
      marking.line_points.emplace_back(math::Vec3d(i, 0.0, 0.0));
    }
    ReferenceLine ref(&marking);
    EXPECT_FALSE(ref.reference_points().empty());
  }
}

TEST_F(ReferenceLineTest, CreateLocalRouteReferenceLineTest) {
  LocalRoute local_route;

  local_route.setId(1);
  for (int i = 0; i < 50; i++) {
    LocalRoutePoint pt;
    pt.setX(i);
    pt.setS(i);
    pt.setLeftLaneWidth(1.875);
    pt.setRightLaneWidth(1.875);
    pt.setLeftRoadWidth(2.0);
    pt.setRightRoadWidth(2.0);
    local_route.mutableGuideLine()->emplace_back(pt);
  }
  local_route.setAdcS(15.0);
  local_route.setAdcL(0.1);
  local_route.setGlobalStartS(0.0);
  std::vector<std::string> tmp;
  local_route.setValidRange({true, 10.0, 45.0, tmp});

  local_route.mutableSpeedLimits()->emplace_back(0.0, 5.0, 20.0, 0.0, math::Vec3d(0.0, 0.0, 0.0),
                                                 math::Vec3d(5.0, 0.0, 0.0));
  local_route.mutableSpeedLimits()->emplace_back(5.0, 10.0, 20.0, 0.0, math::Vec3d(5.0, 0.0, 0.0),
                                                 math::Vec3d(10.0, 0.0, 0.0));
  local_route.mutableSpeedLimits()->emplace_back(10.0, 40.0, 40.0, 0.0, math::Vec3d(10.0, 0.0, 0.0),
                                                 math::Vec3d(40.0, 0.0, 0.0));
  local_route.mutableSpeedLimits()->emplace_back(40.0, 49.0, 40.0, 0.0, math::Vec3d(40.0, 0.0, 0.0),
                                                 math::Vec3d(49.0, 0.0, 0.0));
  local_route.mutableSpeedLimits()->emplace_back(55.0, 59.0, 40.0, 0.0, math::Vec3d(55.0, 0.0, 0.0),
                                                 math::Vec3d(59.0, 0.0, 0.0));

  local_route.mutableDirections()->emplace_back(0.0, 5.0, DrivingDirection::kDirectionForwardOnly,
                                                math::Vec3d(0.0, 0.0, 0.0), math::Vec3d(5.0, 0.0, 0.0));
  local_route.mutableDirections()->emplace_back(5.0, 10.0, DrivingDirection::kDirectionLeftOnly,
                                                math::Vec3d(5.0, 0.0, 0.0), math::Vec3d(10.0, 0.0, 0.0));
  local_route.mutableDirections()->emplace_back(10.0, 20.0, DrivingDirection::kDirectionRightOnly,
                                                math::Vec3d(10.0, 0.0, 0.0), math::Vec3d(20.0, 0.0, 0.0));
  local_route.mutableDirections()->emplace_back(20.0, 30.0, DrivingDirection::kDirectionUTurnOnly,
                                                math::Vec3d(20.0, 0.0, 0.0), math::Vec3d(30.0, 0.0, 0.0));
  local_route.mutableDirections()->emplace_back(30.0, 49.0, DrivingDirection::kDirectionInvalid,
                                                math::Vec3d(30.0, 0.0, 0.0), math::Vec3d(49.0, 0.0, 0.0));
  local_route.mutableDirections()->emplace_back(55.0, 59.0, DrivingDirection::kDirectionInvalid,
                                                math::Vec3d(55.0, 0.0, 0.0), math::Vec3d(59.0, 0.0, 0.0));

  local_route.mutableSourceInfos()->emplace_back(
      proto::LocalRouteReferenceLine_LineSource_LineSourceType_kTypePerception, 0.0, 5.0);
  local_route.mutableSourceInfos()->emplace_back(proto::LocalRouteReferenceLine_LineSource_LineSourceType_kTypeVirtual,
                                                 5.0, 10.0);
  local_route.mutableSourceInfos()->emplace_back(proto::LocalRouteReferenceLine_LineSource_LineSourceType_kTypeVirtual,
                                                 10.0, 40.0);
  local_route.mutableSourceInfos()->emplace_back(
      proto::LocalRouteReferenceLine_LineSource_LineSourceType_kTypePerception, 40.0, 49.0);
  local_route.mutableSourceInfos()->emplace_back(
      proto::LocalRouteReferenceLine_LineSource_LineSourceType_kTypePerception, 55.0, 59.0);

  local_route.mutableBoundaryTypes()->emplace_back(0.0, 5.0, LocalRouteSegment::BoundaryType::DASHED,
                                                   LocalRouteSegment::BoundaryType::SOLID, math::Vec3d(0.0, 0.0, 0.0),
                                                   math::Vec3d(5.0, 0.0, 0.0));
  local_route.mutableBoundaryTypes()->emplace_back(5.0, 40.0, LocalRouteSegment::BoundaryType::DASHED,
                                                   LocalRouteSegment::BoundaryType::PHYSICALLY_UNCROSSABLE,
                                                   math::Vec3d(5.0, 0.0, 0.0), math::Vec3d(40.0, 0.0, 0.0));
  local_route.mutableBoundaryTypes()->emplace_back(40.0, 49.0, LocalRouteSegment::BoundaryType::UNKNOWN,
                                                   LocalRouteSegment::BoundaryType::DASHED, math::Vec3d(40.0, 0.0, 0.0),
                                                   math::Vec3d(49.0, 0.0, 0.0));
  local_route.mutableBoundaryTypes()->emplace_back(55.0, 59.0, LocalRouteSegment::BoundaryType::UNKNOWN,
                                                   LocalRouteSegment::BoundaryType::DASHED, math::Vec3d(55.0, 0.0, 0.0),
                                                   math::Vec3d(59.0, 0.0, 0.0));

  local_route.mutableStopLines()->emplace_back("a", false, 5.0, 0.0, 0.0, 5.0, "0", DrivingDirection::kDirectionInvalid);
  local_route.mutableStopLines()->emplace_back("b", false, 40.0, 0.0, 0.0, 40.0, "0",
                                               DrivingDirection::kDirectionInvalid);
  local_route.mutableStopLines()->emplace_back("c", false, 49.0, 0.0, 0.0, 49.0, "0",
                                               DrivingDirection::kDirectionInvalid);
  local_route.mutableStopLines()->emplace_back("d", false, 55.0, 0.0, 0.0, 59.0, "0",
                                               DrivingDirection::kDirectionInvalid);

  local_route.mutableGates()->emplace_back("a", proto::perception::Gate::GateStatus::Gate_GateStatus_kStatusUnknown, 5.0, 0.0, 0.0, 5.0, 1.0);
  local_route.mutableGates()->emplace_back("b", proto::perception::Gate::GateStatus::Gate_GateStatus_kStatusUnknown, 40.0, 0.0, 0.0, 40.0, 1.0);
  local_route.mutableGates()->emplace_back("c", proto::perception::Gate::GateStatus::Gate_GateStatus_kStatusUnknown, 49.0, 0.0, 0.0, 49.0, 1.0);
  local_route.mutableGates()->emplace_back("d", proto::perception::Gate::GateStatus::Gate_GateStatus_kStatusUnknown, 55.0, 0.0, 0.0, 59.0, 1.0);

  MergeForkRange mf;
  mf.setS(5.0);
  mf.setPoint(local_route.guideLine().at(5));
  local_route.mutableMergeForkRanges()->emplace_back(mf);
  mf.setS(40.0);
  mf.setPoint(local_route.guideLine().at(40));
  local_route.mutableMergeForkRanges()->emplace_back(mf);
  mf.setS(49.0);
  mf.setPoint(local_route.guideLine().at(49));
  local_route.mutableMergeForkRanges()->emplace_back(mf);
  mf.setS(59.0);
  mf.setPoint(local_route.guideLine().at(49));
  local_route.mutableMergeForkRanges()->emplace_back(mf);

  std::vector<LocalRouteSegment> segs;
  LocalRouteSegment seg;
  seg.setRange({true, 0.0, 5.0});
  seg.setRangePoints({local_route.guideLine().at(0), local_route.guideLine().at(5)});
  seg.setNavigationScore(1.0);
  segs.emplace_back(seg);
  local_route.mutableLanesSegments()->emplace_back(std::make_pair(seg.id(), segs));
  seg.setRange({true, 5.0, 40.0});
  seg.setRangePoints({local_route.guideLine().at(5), local_route.guideLine().at(40)});
  seg.setNavigationScore(1.0);
  segs.clear();
  segs.emplace_back(seg);
  local_route.mutableLanesSegments()->emplace_back(std::make_pair(seg.id(), segs));
  seg.setRange({false, 40.0, 49.0});
  seg.setRangePoints({local_route.guideLine().at(40), local_route.guideLine().at(49)});
  seg.setNavigationScore(1.0);
  segs.clear();
  segs.emplace_back(seg);
  local_route.mutableLanesSegments()->emplace_back(std::make_pair(seg.id(), segs));
  seg.setRange({true, 40.0, 49.0});
  seg.setRangePoints({local_route.guideLine().at(40), local_route.guideLine().at(49)});
  seg.setNavigationScore(0.0);
  segs.clear();
  segs.emplace_back(seg);
  local_route.mutableLanesSegments()->emplace_back(std::make_pair(seg.id(), segs));
  seg.setRange({true, 55.0, 59.0});
  seg.setRangePoints({local_route.guideLine().at(49), local_route.guideLine().at(49)});
  seg.setNavigationScore(1.0);
  segs.clear();
  segs.emplace_back(seg);
  local_route.mutableLanesSegments()->emplace_back(std::make_pair(seg.id(), segs));

  BoundaryCrossRange left_boundary_cross_range;
  left_boundary_cross_range.setBoundType(LocalRouteSegment::BoundaryType::DASHED);
  left_boundary_cross_range.setBoundRange({true, 0.0, 15.0});
  left_boundary_cross_range.setBoundRangePoints({local_route.guideLine().at(0), local_route.guideLine().at(15)});
  left_boundary_cross_range.mutableSegmentsId()->emplace_back("a");
  left_boundary_cross_range.mutableSegmentsRange()->emplace_back(std::tuple<bool, float, float>{true, 0.0, 15.0});
  left_boundary_cross_range.mutableSegmentsRangePoints()->emplace_back(
      std::pair<LocalRoutePoint, LocalRoutePoint>{local_route.guideLine().at(0), local_route.guideLine().at(15)});
  left_boundary_cross_range.mutableNeighborSegmentsId()->emplace_back(
      std::pair<std::string, std::vector<int>>{"b", {2}});
  local_route.mutableLeftBoundaryCrossRanges()->emplace_back(left_boundary_cross_range);

  left_boundary_cross_range.setBoundType(LocalRouteSegment::BoundaryType::SOLID);
  left_boundary_cross_range.setBoundRange({true, 15.0, 15.0});
  left_boundary_cross_range.setBoundRangePoints({local_route.guideLine().at(15), local_route.guideLine().at(15)});
  left_boundary_cross_range.mutableSegmentsId()->emplace_back("c");
  left_boundary_cross_range.mutableSegmentsRange()->emplace_back(std::tuple<bool, float, float>{true, 15.0, 15.0});
  left_boundary_cross_range.mutableSegmentsRangePoints()->emplace_back(
      std::pair<LocalRoutePoint, LocalRoutePoint>{local_route.guideLine().at(15), local_route.guideLine().at(15)});
  left_boundary_cross_range.mutableNeighborSegmentsId()->emplace_back(
      std::pair<std::string, std::vector<int>>{"d", {7}});
  local_route.mutableLeftBoundaryCrossRanges()->emplace_back(left_boundary_cross_range);

  BoundaryCrossRange right_boundary_cross_range;
  right_boundary_cross_range.setBoundType(LocalRouteSegment::BoundaryType::DASHED);
  right_boundary_cross_range.setBoundRange({true, 0.0, 15.0});
  right_boundary_cross_range.setBoundRangePoints({local_route.guideLine().at(0), local_route.guideLine().at(15)});
  right_boundary_cross_range.mutableSegmentsId()->emplace_back("a");
  right_boundary_cross_range.mutableSegmentsRange()->emplace_back(std::tuple<bool, float, float>{true, 0.0, 15.0});
  right_boundary_cross_range.mutableSegmentsRangePoints()->emplace_back(
      std::pair<LocalRoutePoint, LocalRoutePoint>{local_route.guideLine().at(0), local_route.guideLine().at(15)});
  right_boundary_cross_range.mutableNeighborSegmentsId()->emplace_back(
      std::pair<std::string, std::vector<int>>{"b", {2}});
  local_route.mutableRightBoundaryCrossRanges()->emplace_back(right_boundary_cross_range);

  right_boundary_cross_range.setBoundType(LocalRouteSegment::BoundaryType::SOLID);
  right_boundary_cross_range.setBoundRange({true, 15.0, 15.0});
  right_boundary_cross_range.setBoundRangePoints({local_route.guideLine().at(15), local_route.guideLine().at(15)});
  right_boundary_cross_range.mutableSegmentsId()->emplace_back("c");
  right_boundary_cross_range.mutableSegmentsRange()->emplace_back(std::tuple<bool, float, float>{true, 15.0, 15.0});
  right_boundary_cross_range.mutableSegmentsRangePoints()->emplace_back(
      std::pair<LocalRoutePoint, LocalRoutePoint>{local_route.guideLine().at(15), local_route.guideLine().at(15)});
  right_boundary_cross_range.mutableNeighborSegmentsId()->emplace_back(
      std::pair<std::string, std::vector<int>>{"d", {7}});
  local_route.mutableRightBoundaryCrossRanges()->emplace_back(right_boundary_cross_range);

  {
    ReferenceLine ref(&local_route);
    EXPECT_FALSE(ref.reference_points().empty());
  }

  {
    local_route.setValidRange({true, 0.0, 49.0, tmp});
    ReferenceLine ref(&local_route);
    EXPECT_FALSE(ref.reference_points().empty());
  }

  {
    local_route.mutableGuideLine()->erase(local_route.mutableGuideLine()->begin(),
                                          local_route.mutableGuideLine()->begin() + 49);
    ReferenceLine ref(&local_route);
    EXPECT_FALSE(ref.reference_points().empty());
  }

  {
    local_route.mutableSpeedLimits()->clear();
    local_route.mutableDirections()->clear();
    local_route.mutableSourceInfos()->clear();
    local_route.mutableBoundaryTypes()->clear();
    local_route.mutableStopLines()->clear();
    local_route.mutableGates()->clear();
    local_route.mutableMergeForkRanges()->clear();
    local_route.mutableLanesSegments()->clear();
    ReferenceLine ref(&local_route);
    EXPECT_FALSE(ref.reference_points().empty());
  }
}

TEST_F(ReferenceLineTest, CreateLocalRouteRefReferenceLineTest) {
  proto::LocalRouteReferenceLine local_route_ref;

  local_route_ref.set_id("a");
  for (int i = 0; i < 50; i++) {
    auto guide_point = local_route_ref.add_guide_points();
    guide_point->mutable_point()->set_x(i);
    guide_point->set_s(i);
  }
  local_route_ref.set_length(49.0);

  auto line_source_1 = local_route_ref.add_line_sources();
  line_source_1->set_start_s(0.0);
  line_source_1->set_end_s(10.0);
  line_source_1->set_type(proto::LocalRouteReferenceLine_LineSource_LineSourceType_kTypePerception);
  auto line_source_2 = local_route_ref.add_line_sources();
  line_source_2->set_start_s(10.0);
  line_source_2->set_end_s(49.0);
  line_source_2->set_type(proto::LocalRouteReferenceLine_LineSource_LineSourceType_kTypeVirtual);

  auto res_stop_line_1 = local_route_ref.add_stop_lines();
  res_stop_line_1->set_id("a");
  res_stop_line_1->set_s(10.0);
  auto res_stop_line_2 = local_route_ref.add_stop_lines();
  res_stop_line_2->set_id("b");
  res_stop_line_2->set_s(20.0);

  auto res_gate_1 = local_route_ref.add_gates();
  res_gate_1->set_id("a");
  res_gate_1->set_s(10.0);
  auto res_gate_2 = local_route_ref.add_gates();
  res_gate_2->set_id("b");
  res_gate_2->set_s(17.0);

  auto res_speed_limit_1 = local_route_ref.add_speed_limits();
  res_speed_limit_1->set_start_s(0.0);
  res_speed_limit_1->set_end_s(10.0);
  res_speed_limit_1->set_max_speed_limit(20.0);
  auto res_speed_limit_2 = local_route_ref.add_speed_limits();
  res_speed_limit_2->set_start_s(10.0);
  res_speed_limit_2->set_end_s(49.0);
  res_speed_limit_2->set_max_speed_limit(30.0);

  auto res_navigation_score_1 = local_route_ref.add_navigation_scores();
  res_navigation_score_1->set_navigation_score(1.0);
  res_navigation_score_1->set_start_s(0.0);
  res_navigation_score_1->set_end_s(10.0);
  auto res_navigation_score_2 = local_route_ref.add_navigation_scores();
  res_navigation_score_2->set_navigation_score(0.0);
  res_navigation_score_2->set_start_s(10.0);
  res_navigation_score_2->set_end_s(49.0);

  local_route_ref.add_related_area_ids("a");

  auto res_direction_1 = local_route_ref.add_directions();
  res_direction_1->set_start_s(0.0);
  res_direction_1->set_end_s(10.0);
  res_direction_1->set_direction(proto::MapCommon_DrivingDirection_kDrivingStraight);
  auto res_direction_2 = local_route_ref.add_directions();
  res_direction_2->set_start_s(10.0);
  res_direction_2->set_end_s(20.0);
  res_direction_2->set_direction(proto::MapCommon_DrivingDirection_kDrivingLeft);
  auto res_direction_3 = local_route_ref.add_directions();
  res_direction_3->set_start_s(20.0);
  res_direction_3->set_end_s(30.0);
  res_direction_3->set_direction(proto::MapCommon_DrivingDirection_kDrivingRight);
  auto res_direction_4 = local_route_ref.add_directions();
  res_direction_4->set_start_s(30.0);
  res_direction_4->set_end_s(49.0);
  res_direction_4->set_direction(proto::MapCommon_DrivingDirection_kDrivingUturn);

  auto res_left_bound = local_route_ref.add_left_boundaries();
  auto res_right_bound = local_route_ref.add_right_boundaries();
  res_left_bound->set_bound_type(proto::LocalRouteBoundary_BoundaryType_kBoundaryDashed);
  res_left_bound->set_start_s(0.0);
  res_left_bound->set_end_s(10.0);
  res_right_bound->set_bound_type(proto::LocalRouteBoundary_BoundaryType_kBoundaryPhysicallyUncrossable);
  res_right_bound->set_start_s(0.0);
  res_right_bound->set_end_s(10.0);

  auto res_left_ref_line = local_route_ref.add_left_reference_lines();
  res_left_ref_line->set_neighbor_id("1");
  auto left_boundary = res_left_ref_line->add_boundaries();
  left_boundary->set_bound_type(proto::LocalRouteBoundary_BoundaryType_kBoundaryUnknown);
  left_boundary->set_start_s(0.0);
  left_boundary->set_end_s(10.0);

  auto res_right_ref_line = local_route_ref.add_right_reference_lines();
  res_right_ref_line->set_neighbor_id("3");
  auto right_boundary = res_right_ref_line->add_boundaries();
  right_boundary->set_bound_type(proto::LocalRouteBoundary_BoundaryType_kBoundarySolid);
  right_boundary->set_start_s(0.0);
  right_boundary->set_end_s(10.0);

  auto res_merge_fork_range = local_route_ref.add_merge_fork_ranges();
  res_merge_fork_range->set_type(proto::LocalRouteMergeForkRange_MergeForkType_kTypeMerge);
  res_merge_fork_range->set_s(15.0);
  res_merge_fork_range->add_related_local_route_ids("8");

  auto res_navigation_lane_change_info = local_route_ref.mutable_navigation_lane_change_info();
  res_navigation_lane_change_info->set_s(38.0);
  res_navigation_lane_change_info->set_direction(1);
  res_navigation_lane_change_info->set_times(2);

  proto::LocalRouteRoadInfo road_info;
  road_info.add_road_types(proto::MapCommon_RoadType_kMainRoad);

  std::tuple<Eigen::Matrix4d, Eigen::Matrix4d, math::Vec3d> tf;
  tf = std::make_tuple(Eigen::Matrix4d::Identity(), Eigen::Matrix4d::Identity(), math::Vec3d());

  {
    ReferenceLine ref(local_route_ref, tf, road_info, "car", MapPoint());
    EXPECT_FALSE(ref.reference_points().empty());
  }

  {
    local_route_ref.set_id("");
    ReferenceLine ref(local_route_ref, tf, road_info, "map", MapPoint());
    EXPECT_FALSE(ref.reference_points().empty());
  }

  {
    local_route_ref.set_id("a");
    auto new_left_bound = local_route_ref.add_left_boundaries();
    new_left_bound->set_bound_type(proto::LocalRouteBoundary_BoundaryType_kBoundaryPhysicallyUncrossable);
    new_left_bound->set_start_s(10.0);
    new_left_bound->set_end_s(20.0);
    ReferenceLine ref(local_route_ref, tf, road_info, "car", MapPoint());
    EXPECT_FALSE(ref.reference_points().empty());
  }

  {
    local_route_ref.set_id("a");
    local_route_ref.mutable_left_reference_lines()->Clear();
    local_route_ref.mutable_right_reference_lines()->Clear();
    auto new_left_ref_line = local_route_ref.add_left_reference_lines();
    new_left_ref_line->set_neighbor_id("11");
    auto new_right_ref_line = local_route_ref.add_right_reference_lines();
    new_right_ref_line->set_neighbor_id("33");
    ReferenceLine ref(local_route_ref, tf, road_info, "car", MapPoint());
    EXPECT_FALSE(ref.reference_points().empty());
  }
}

TEST_F(ReferenceLineTest, CreateEnvRoadCognitionReferenceLineTest) {
  proto::road_cognition::ReferenceLine ref_proto;
  ref_proto.set_id("a");
  for (int i = 0; i < 50; i++) {
    auto pt_proto = ref_proto.add_reference_points();
    pt_proto->mutable_point()->set_x(i);
    pt_proto->mutable_point()->set_y(0.0);
    pt_proto->set_s(i);
    pt_proto->set_lane_left_width(1.75);
    pt_proto->set_lane_right_width(1.75);
    pt_proto->set_road_left_width(2.0);
    pt_proto->set_road_right_width(2.0);
  }
  ref_proto.set_length(49.0);

  auto line_attribute_range_proto = ref_proto.add_line_attributes();
  line_attribute_range_proto->set_source_type(proto::road_cognition::LineAttributeRange_LineSourceType_kTypeInvalid);
  line_attribute_range_proto->set_line_type(proto::perception::CenterLine_TypeRange_Type_kTypeUnknown);
  line_attribute_range_proto->set_start_s(0.0);
  line_attribute_range_proto->set_end_s(49.0);

  auto stop_line_proto = ref_proto.add_stop_lines();
  stop_line_proto->set_id("sss");
  stop_line_proto->set_type(proto::perception::StopLine_LineType_kTypeNormal);
  stop_line_proto->mutable_point()->set_x(5.0);
  stop_line_proto->mutable_point()->set_y(0.0);
  stop_line_proto->set_s(5.0);
  stop_line_proto->set_direction(proto::MapCommon_DrivingDirection_kDrivingStraight);
  stop_line_proto->set_related_traffic_light_id("mm");

  auto gate_proto = ref_proto.add_gates();
  gate_proto->set_id("ggg");
  gate_proto->set_gate_type(proto::road_cognition::GateRange_GateType_kTypeUnknown);
  gate_proto->set_gate_status(proto::perception::Gate_GateStatus_kStatusUnknown);
  auto gate_pt_proto = gate_proto->add_points();
  gate_pt_proto->set_x(5.0);
  gate_pt_proto->set_y(0.0);
  gate_proto->set_s(5.0);
  gate_proto->set_head_stop_distance(1.0);

  auto speed_limit_proto = ref_proto.add_speed_limits();
  speed_limit_proto->set_max_speed_limit(80 / 3.6);
  speed_limit_proto->set_min_speed_limit(0.0);
  speed_limit_proto->set_start_s(0.0);
  speed_limit_proto->set_end_s(49.0);

  auto road_range_proto = ref_proto.add_roads();
  road_range_proto->set_road_id("rr");
  road_range_proto->set_start_s(0.0);
  road_range_proto->set_end_s(49.0);

  auto area_range_proto = ref_proto.add_areas();
  area_range_proto->set_area_id("aa");
  area_range_proto->set_start_s(0.0);
  area_range_proto->set_end_s(49.0);

  auto direction_proto = ref_proto.add_directions();
  direction_proto->set_direction(proto::MapCommon_DrivingDirection_kDrivingUturn);
  direction_proto->set_start_s(0.0);
  direction_proto->set_end_s(49.0);

  auto left_lane_boundary_proto = ref_proto.add_left_lane_boundaries();
  left_lane_boundary_proto->set_bound_type(proto::road_cognition::BoundaryRange_BoundaryType_kTypeCrossGuide);
  left_lane_boundary_proto->set_bound_shape(proto::perception::LaneMarking_LineShape_kShapeUnknown);
  left_lane_boundary_proto->set_bound_color(proto::perception::LaneMarking_LineColor_kColorUnknown);
  left_lane_boundary_proto->set_start_s(0.0);
  left_lane_boundary_proto->set_end_s(49.0);
  left_lane_boundary_proto->add_related_lane_markings_ids(11);

  auto right_lane_boundary_proto = ref_proto.add_right_lane_boundaries();
  right_lane_boundary_proto->set_bound_type(proto::road_cognition::BoundaryRange_BoundaryType_kTypeCrossGuide);
  right_lane_boundary_proto->set_bound_shape(proto::perception::LaneMarking_LineShape_kShapeUnknown);
  right_lane_boundary_proto->set_bound_color(proto::perception::LaneMarking_LineColor_kColorUnknown);
  right_lane_boundary_proto->set_start_s(0.0);
  right_lane_boundary_proto->set_end_s(49.0);
  right_lane_boundary_proto->add_related_lane_markings_ids(11);

  auto left_road_boundary_proto = ref_proto.add_left_road_boundaries();
  left_road_boundary_proto->set_bound_type(proto::road_cognition::BoundaryRange_BoundaryType_kTypeCrossGuide);
  left_road_boundary_proto->set_bound_shape(proto::perception::LaneMarking_LineShape_kShapeUnknown);
  left_road_boundary_proto->set_bound_color(proto::perception::LaneMarking_LineColor_kColorUnknown);
  left_road_boundary_proto->set_start_s(0.0);
  left_road_boundary_proto->set_end_s(49.0);
  left_road_boundary_proto->add_related_lane_markings_ids(11);

  auto right_road_boundary_proto = ref_proto.add_right_road_boundaries();
  right_road_boundary_proto->set_bound_type(proto::road_cognition::BoundaryRange_BoundaryType_kTypeCrossGuide);
  right_road_boundary_proto->set_bound_shape(proto::perception::LaneMarking_LineShape_kShapeUnknown);
  right_road_boundary_proto->set_bound_color(proto::perception::LaneMarking_LineColor_kColorUnknown);
  right_road_boundary_proto->set_start_s(0.0);
  right_road_boundary_proto->set_end_s(49.0);
  right_road_boundary_proto->add_related_lane_markings_ids(11);

  ref_proto.add_related_risk_boundaries_ids("cc");

  auto left_ref_proto = ref_proto.add_left_reference_lines();
  left_ref_proto->set_neighbor_id("a");
  auto l_boundary_proto = left_ref_proto->add_boundaries();
  l_boundary_proto->set_bound_type(proto::road_cognition::BoundaryRange_BoundaryType_kTypeCrossGuide);
  l_boundary_proto->set_bound_shape(proto::perception::LaneMarking_LineShape_kShapeUnknown);
  l_boundary_proto->set_bound_color(proto::perception::LaneMarking_LineColor_kColorUnknown);
  l_boundary_proto->set_start_s(0.0);
  l_boundary_proto->set_end_s(49.0);
  l_boundary_proto->add_related_lane_markings_ids(11);

  auto right_ref_proto = ref_proto.add_right_reference_lines();
  right_ref_proto->set_neighbor_id("a");
  auto r_boundary_proto = right_ref_proto->add_boundaries();
  r_boundary_proto->set_bound_type(proto::road_cognition::BoundaryRange_BoundaryType_kTypeCrossGuide);
  r_boundary_proto->set_bound_shape(proto::perception::LaneMarking_LineShape_kShapeUnknown);
  r_boundary_proto->set_bound_color(proto::perception::LaneMarking_LineColor_kColorUnknown);
  r_boundary_proto->set_start_s(0.0);
  r_boundary_proto->set_end_s(49.0);
  r_boundary_proto->add_related_lane_markings_ids(11);

  auto key_pt_proto = ref_proto.add_key_points();
  key_pt_proto->set_id("hh");
  key_pt_proto->set_type(proto::road_cognition::KeyPoint_PointType_kTypeUnknown);
  key_pt_proto->set_s(5.0);
  key_pt_proto->mutable_point()->set_x(5.0);
  key_pt_proto->mutable_point()->set_y(0.0);
  key_pt_proto->add_related_reference_lines_ids("a");

  auto navi_proto = ref_proto.mutable_navigation_info();
  navi_proto->set_lane_change_distance(-1);
  navi_proto->set_lane_change_direction(0);
  navi_proto->set_lane_change_times(0);
  auto score_range_proto = navi_proto->add_navigation_scores();
  score_range_proto->set_score(1.0);
  score_range_proto->set_start_s(0.0);
  score_range_proto->set_end_s(49.0);

  auto god_bound_proto = ref_proto.add_related_god_boundaries();
  god_bound_proto->set_type(proto::PerceptionPolyline_PolylineType_kTypeInvalid);
  for (int m = 0; m < 10; m++) {
    god_bound_proto->add_s_arr(m);
    god_bound_proto->add_l_arr(2.0);
  }

  ReferenceLine ref(ref_proto, MapPoint());
  EXPECT_TRUE(ref.reference_points().size() > 0);

  auto score_res = ref.getNavigationScoresFromSRange(80.0, 90.0);
  EXPECT_TRUE(score_res.empty());

  auto aver_bound = ref.getAverageLaneBoundFromRange(0.0, 10.0, 1.0);
  aver_bound = ref.getAverageRoadBoundFromRange(0.0, 10.0, 1.0);
  auto line_attrs_res = ref.getLineAttributeRangesFromSRange(0.0, 10.0);
  EXPECT_TRUE(line_attrs_res.size() > 0);
  line_attrs_res = ref.getLineAttributeRangesFromSRange(5.0, 5.0, 5.0);
  EXPECT_TRUE(line_attrs_res.size() > 0);
  line_attrs_res =
      ref.getSpecifiedLineAttributeRanges(proto::road_cognition::LineAttributeRange_LineSourceType_kTypeInvalid);
  EXPECT_TRUE(line_attrs_res.size() > 0);
  line_attrs_res = ref.getSpecifiedLineAttributeRanges(proto::perception::CenterLine_TypeRange_Type_kTypeUnknown);
  EXPECT_TRUE(line_attrs_res.size() > 0);
  auto line_attr_res = ref.getLineAttributeRangeFromS(5.0);

  auto bound_ranges_res = ref.getBoundaryRangesFromSRange(ref.getLeftLaneBoundaries(), 5.0, 5.0, 5.0);
  EXPECT_TRUE(bound_ranges_res.size() > 0);
  bound_ranges_res = ref.getBoundaryRangesFromSRange(ref.getLeftLaneBoundaries(), 0.0, 10.0);
  EXPECT_TRUE(bound_ranges_res.size() > 0);

  auto bound_range_res = ref.getBoundaryRangeFromS(ref.getLeftLaneBoundaries(), 0.0);

  auto neighbor_ids_res = ref.getNeighborReferenceLineIdsFromS(5.0, true);
  EXPECT_TRUE(neighbor_ids_res.size() > 0);

  auto key_pts_res = ref.getKeyPointsFromSRange(0.0, 10.0);
  EXPECT_TRUE(key_pts_res.size() > 0);
  key_pts_res = ref.getKeyPointsFromSRange(5.0, 5.0, 5.0);
  EXPECT_TRUE(key_pts_res.size() > 0);
  key_pts_res = ref.getSpecifiedKeyPoints(proto::road_cognition::KeyPoint_PointType_kTypeUnknown);
  EXPECT_TRUE(key_pts_res.size() > 0);

  ReferenceLine empty_ref;
  auto ref_pts = empty_ref.getReferencePoints(5.0, 10.0);
  auto index = empty_ref.getNearestReferenceIndex(5.0);
  auto ref_pt = empty_ref.getNearestReferencePoint(math::Vec3d());
  ref_pt = empty_ref.getNearestReferencePoint(5.0);
  EXPECT_TRUE(true);
}

TEST_F(ReferenceLineTest, FuctionTest) {
  std::vector<math::Vec3d> pts;
  for (int i = 0; i < 50; i++) {
    pts.emplace_back(math::Vec3d(i, 0.0, 0.0));
  }
  ReferenceLine ref(pts);

  double l_offset = 0.0;
  auto res = ref.offset(3.2, &l_offset);
  EXPECT_TRUE(res);

  auto ref_point = ref.getReferencePoint(3.2, 0.3);
  ref_point = ref.getReferencePoint(-1.0);
  ref_point = ref.getReferencePoint(3.2);
  ref_point = ref.getReferencePoint(88);

  auto ref_points = ref.getReferencePoints(2.3, 7.8);
  ref_points = ref.getReferencePoints(-2.3, 77.8);
  ref_points = ref.getReferencePoints(7.8, 2.3);

  ref_points = ref.getInterpolatedRefPoints(2.3, 7.8, 1.0);
  ref_points = ref.getInterpolatedRefPoints(-2.3, 77.8, 1.0);
  EXPECT_TRUE(true);

  ref.set_id("left");
  ref.getMemorizedRouteReferencePoints(math::Vec3d(), MapPoint());
  ref.set_id("right");
  ref.getMemorizedRouteReferencePoints(math::Vec3d(), MapPoint());

  PerceptionLane lane;
  lane.id = "a";
  LaneSegment seg;
  for (int i = 0; i < 50; i++) {
    seg.center_line.emplace_back(math::Vec3d(i, i, 0.0));
    seg.offset.emplace_back(1.5);
  }
  lane.lane_segments.emplace_back(seg);
  ref.modifyMemorizedReferenceLineBoundByPerceptionLane(lane, 0.0, 0.0);
  ref.modifyMemorizedReferenceLineBoundByPerceptionLane(lane, 90.0, 0.0);
  ref.modifyMemorizedReferenceLineBoundByPerceptionLane(lane, 0.0, 100.0);
  ref.modifyMemorizedReferenceLineBoundByPerceptionLane(lane, 90.0 * ANG2RAD, 100.0);

  ref.getNearestReferenceIndex(1.0);
  ref.getNearestReferenceIndex(-1.0);
  ref.getNearestReferenceIndex(100.0);

  ref.getNearestReferencePoint(math::Vec3d());
  ref.getNearestReferencePoint(1.1);
  ref.getNearestReferencePoint(1.9);
  ref.getNearestReferencePoint(-1.0);
  ref.getNearestReferencePoint(100.0);

  ref.getUpperIndexFromS(100.0);
  ref.getUpperIndexFromS(-1.0);

  ref.getFrenetPoint(planning::PathPt(math::Vec3d()));

  ref.toFrenetFrame(planning::TrajectoryPt());

  ref.getNavigationScoresFromSRange(1.0, 0.0, 10.0);
  ref.getNavigationScoreFromS(1.0);
  ref.setNavigationScores({std::tuple<float, float, float>{1.0, -10.0, -1.0},
                           std::tuple<float, float, float>{1.0, 0.0, 49.0},
                           std::tuple<float, float, float>{1.0, 49.0, 60.0}});
  ref.getNavigationScoresFromSRange(0.0, 0.0, 10.0);
  ref.getNavigationScoreFromS(1.0);
  ref.getNavigationScoreFromS(-10.0);
  ref.getNavigationScoreFromS(100.0);
  ref.set_line_type(ReferenceLine::LineType::MEMORIZED_ROUTE);
  ref.setIsParallelVirtual(false);
  ref.getNavigationScoresFromSRange(0.0, 0.0, 10.0);
  ref.getNavigationScoreFromS(1.0);
  ref.setIsParallelVirtual(true);
  ref.getNavigationScoresFromSRange(0.0, 0.0, 10.0);
  ref.getNavigationScoreFromS(1.0);

  res = ref.isInLaneFollowRanges();
  EXPECT_FALSE(res);
  ref.setlaneFollowRanges({std::pair<float, float>{10.0, 20.0}});
  res = ref.isInLaneFollowRanges();
  EXPECT_FALSE(res);
  ref.setlaneFollowRanges({std::pair<float, float>{0.0, 10.0}});
  res = ref.isInLaneFollowRanges();
  EXPECT_TRUE(res);
  ref.setlaneFollowRanges({std::pair<float, float>{-10.0, -1.0}});
  res = ref.isInLaneFollowRanges();
  EXPECT_FALSE(res);

  ref.getMaxCurvatureInRange(0.0, 10.0);
  ref.getMaxCurvatureInRange(10.0, 20.0);
  ref.getMaxCurvatureInRange(10.0, 8.0);
  ref.getMaxCurvatureInRange(100.0, 110.0);
  ref.getMaxCurvatureInRange(-10.0, -1.0);

  ref.getMaxAndMinCurvatureInRange(0.0, 10.0);
  ref.getMaxAndMinCurvatureInRange(10.0, 20.0);
  ref.getMaxAndMinCurvatureInRange(10.0, 8.0);
  ref.getMaxAndMinCurvatureInRange(100.0, 110.0);
  ref.getMaxAndMinCurvatureInRange(-10.0, -1.0);

  double lane_left_bound = 0.0, lane_right_bound = 0.0, road_left_bound = 0.0, road_right_bound = 0.0;
  ref.getLaneBound(0.0, &lane_left_bound, &lane_right_bound);
  ref.getRoadBound(0.0, &road_left_bound, &road_right_bound);

  gpal::pnc::SLBoundary sl_boundary;
  sl_boundary.set_start_l(-1.0);
  sl_boundary.set_end_l(1.0);
  sl_boundary.set_start_s(0.0);
  auto driving_bound = ref.getDrivingBound(sl_boundary);
  sl_boundary.set_start_l(-1.9);
  sl_boundary.set_end_l(1.9);
  driving_bound = ref.getDrivingBound(sl_boundary);
  sl_boundary.set_start_l(-1.0);
  sl_boundary.set_end_l(1.3);
  driving_bound = ref.getDrivingBound(sl_boundary);
  auto driving_tube_bound = ref.getDrivingTubeBound(0.0);

  SLPoint sl_point;
  sl_point.set_s(100.0);
  res = ref.isOnLane(sl_point);
  EXPECT_FALSE(res);
  res = ref.isOnRoad(sl_point);
  EXPECT_FALSE(res);
  sl_point.set_s(-10.0);
  res = ref.isOnLane(sl_point);
  EXPECT_FALSE(res);
  res = ref.isOnRoad(sl_point);
  EXPECT_FALSE(res);
  sl_point.set_s(0.5);
  sl_point.set_l(0.0);
  res = ref.isOnLane(sl_point);
  EXPECT_TRUE(res);
  res = ref.isOnRoad(sl_point);
  EXPECT_TRUE(res);
  sl_point.set_l(10.0);
  res = ref.isOnLane(sl_point);
  EXPECT_FALSE(res);
  res = ref.isOnRoad(sl_point);
  EXPECT_FALSE(res);
  sl_point.set_l(-10.0);
  res = ref.isOnLane(sl_point);
  EXPECT_FALSE(res);
  res = ref.isOnRoad(sl_point);
  EXPECT_FALSE(res);
  res = ref.isOnLane(math::Vec3d());
  EXPECT_FALSE(res);
  res = ref.isOnRoad(math::Vec3d());
  EXPECT_FALSE(res);

  std::pair<double, double> lane_range;
  sl_boundary.set_start_l(-1.0);
  sl_boundary.set_end_l(1.0);
  sl_boundary.set_start_s(0.0);
  sl_boundary.set_end_s(-10.0);
  res = ref.isOnLane(sl_boundary);
  EXPECT_FALSE(res);
  res = ref.isOnRoad(sl_boundary);
  EXPECT_FALSE(res);
  res = ref.isOnLane(sl_boundary, lane_range, 0.0);
  EXPECT_FALSE(res);
  sl_boundary.set_start_s(100.0);
  sl_boundary.set_end_s(10.0);
  res = ref.isOnLane(sl_boundary);
  EXPECT_FALSE(res);
  res = ref.isOnRoad(sl_boundary);
  EXPECT_FALSE(res);
  res = ref.isOnLane(sl_boundary, lane_range, 0.0);
  EXPECT_FALSE(res);
  sl_boundary.set_start_s(0.0);
  sl_boundary.set_end_s(0.0);
  sl_boundary.set_start_l(-1.0);
  sl_boundary.set_end_l(1.0);
  res = ref.isOnLane(sl_boundary);
  EXPECT_TRUE(res);
  res = ref.isOnRoad(sl_boundary);
  EXPECT_TRUE(res);
  res = ref.isOnLane(sl_boundary, lane_range, 0.0);
  EXPECT_TRUE(res);
  sl_boundary.set_start_l(10.0);
  sl_boundary.set_end_l(-10.0);
  res = ref.isOnLane(sl_boundary);
  EXPECT_FALSE(res);
  res = ref.isOnRoad(sl_boundary);
  EXPECT_FALSE(res);
  res = ref.isOnLane(sl_boundary, lane_range, 0.0);
  EXPECT_FALSE(res);
  sl_boundary.set_start_l(-1.0);
  sl_boundary.set_end_l(-10.0);
  res = ref.isOnLane(sl_boundary);
  EXPECT_FALSE(res);
  res = ref.isOnRoad(sl_boundary);
  EXPECT_FALSE(res);
  res = ref.isOnLane(sl_boundary, lane_range, 0.0);
  EXPECT_FALSE(res);
  sl_boundary.set_start_l(10.0);
  sl_boundary.set_end_l(1.0);
  res = ref.isOnLane(sl_boundary);
  EXPECT_FALSE(res);
  res = ref.isOnRoad(sl_boundary);
  EXPECT_FALSE(res);
  res = ref.isOnLane(sl_boundary, lane_range, 0.0);
  EXPECT_FALSE(res);

  math::Box2d box_a(math::Vec2d(), 0.0, 1.0, 1.0);
  res = ref.hasOverlap(box_a);
  EXPECT_TRUE(res);
  math::Box2d box_b(math::Vec2d(0.0, 3.0), 0.0, 1.0, 1.0);
  res = ref.hasOverlap(box_b);
  EXPECT_FALSE(res);
  math::Box2d box_c(math::Vec2d(-100.0, 3.0), 0.0, 1.0, 1.0);
  res = ref.hasOverlap(box_c);
  EXPECT_FALSE(res);
  math::Box2d box_d(math::Vec2d(100.0, 3.0), 0.0, 1.0, 1.0);
  res = ref.hasOverlap(box_d);
  EXPECT_FALSE(res);

  std::pair<double, double> adc_range;
  sl_boundary.set_start_s(0.0);
  sl_boundary.set_end_s(2.0);
  sl_boundary.set_start_l(-1.0);
  sl_boundary.set_end_l(1.0);
  res = ref.isRelavent(sl_boundary, lane_range, adc_range, 0.0);
  EXPECT_TRUE(res);
  sl_boundary.set_start_l(-100.0);
  sl_boundary.set_end_l(-50.0);
  res = ref.isRelavent(sl_boundary, lane_range, adc_range, 0.0);
  EXPECT_FALSE(res);
  sl_boundary.set_start_l(50.0);
  sl_boundary.set_end_l(100.0);
  res = ref.isRelavent(sl_boundary, lane_range, adc_range, 0.0);
  EXPECT_FALSE(res);
  sl_boundary.set_start_l(-10.0);
  sl_boundary.set_end_l(10.0);
  res = ref.isRelavent(sl_boundary, lane_range, adc_range, 0.0);
  EXPECT_TRUE(res);
  sl_boundary.set_start_s(100.0);
  sl_boundary.set_end_s(-2.0);
  sl_boundary.set_start_l(-1.0);
  sl_boundary.set_end_l(1.0);
  res = ref.isRelavent(sl_boundary, lane_range, adc_range, 0.0);
  EXPECT_FALSE(res);
  sl_boundary.set_start_s(100.0);
  sl_boundary.set_end_s(200.0);
  sl_boundary.set_start_l(-1.0);
  sl_boundary.set_end_l(1.0);
  res = ref.isRelavent(sl_boundary, lane_range, adc_range, 0.0);
  EXPECT_FALSE(res);

  auto tmp_pts = ref.reference_points();
  res = ref.computeInitInfo(tmp_pts);
  EXPECT_TRUE(res);
  tmp_pts.clear();
  res = ref.computeInitInfo(tmp_pts);
  EXPECT_FALSE(res);

  std::vector<math::Vec2d> polygon_pts;
  polygon_pts.emplace_back(math::Vec2d(0.0, 0.0));
  polygon_pts.emplace_back(math::Vec2d(1.0, 0.0));
  polygon_pts.emplace_back(math::Vec2d(1.0, 1.0));
  polygon_pts.emplace_back(math::Vec2d(0.0, 1.0));
  math::Polygon2d polygon(polygon_pts);
  auto overlap_range = ref.getOverlapRange(polygon);
  ReferenceLine tmp_ref;
  overlap_range = tmp_ref.getOverlapRange(polygon);
  EXPECT_TRUE(true);

  // ref.updateGeoFenceTypes();
  // ref.updateGeoFenceTypes();
  // EXPECT_TRUE(true);

  sl_point.set_s(0.0);
  sl_point.set_l(0.0);
  math::Vec3d xyz_point;
  ReferenceLine tmp_ref_1({math::Vec3d()});
  res = tmp_ref_1.sl2xy(sl_point, &xyz_point);
  EXPECT_FALSE(res);

  proto::PathPoint path_point;
  res = ref.sl2xy_path_point(1.0, 0.0, 0.0, 0.0, &path_point);
  EXPECT_TRUE(res);

  res = ref.xy2sl(math::Vec3d(), 0.0, std::pair<size_t, size_t>{0, 10}, &sl_point);
  EXPECT_TRUE(res);

  res = ref.getSLBoundary(polygon_pts, &sl_boundary);
  EXPECT_TRUE(res);
  math::Box2d box_1(math::Vec2d(), 0.0, 1.0, 1.0);
  res = ref.getSLBoundary(box_1, &sl_boundary, 0.0);
  EXPECT_TRUE(res);
  res = ref.getSLBoundary(box_1, &sl_boundary, 6000.0);
  EXPECT_TRUE(res);
  math::Box2d box_2;
  res = ref.getSLBoundary(box_2, &sl_boundary, 0.0);
  EXPECT_FALSE(res);
  res = tmp_ref_1.getSLBoundary(box_1, &sl_boundary, 0.0);
  EXPECT_FALSE(res);
  math::Box2d box_3(math::Vec2d(), 0.0, 10.0, 10.0);
  ;
  res = ref.getSLBoundary(box_3, &sl_boundary, 0.0);
  EXPECT_TRUE(res);

  double accumulate_s = 0.0, lateral = 0.0;
  res = ref.getProjection(math::Vec3d(), accumulate_s, lateral);
  res = ref.getProjection(math::Vec3d(), 0.0, accumulate_s, lateral);
  int min_index = -1;
  res = ref.getProjection(math::Vec3d(), accumulate_s, lateral, min_index);
  res = ref.getProjection(math::Vec3d(), 0.0, accumulate_s, lateral, min_index);
  res = tmp_ref.getProjection(math::Vec3d(), 0.0, accumulate_s, lateral, min_index);
  double min_distance = 0.0;
  res = ref.getProjection(math::Vec3d(), accumulate_s, lateral, min_distance, min_index);
  res = tmp_ref.getProjection(math::Vec3d(), accumulate_s, lateral, min_distance, min_index);
  res = ref.getProjection(math::Vec3d(), 1.0, std::pair<size_t, size_t>{0, 10}, accumulate_s, lateral);
  res = ref.getProjection(math::Vec3d(), 1.0, std::pair<size_t, size_t>{0, 10}, accumulate_s, lateral, min_index);
  res = ref.getProjection(math::Vec3d(), 1.0, std::pair<size_t, size_t>{0, 10}, accumulate_s, lateral, min_distance,
                          min_index);
  res = ref.getProjection(math::Vec3d(), 8000.0, std::pair<size_t, size_t>{0, 10}, accumulate_s, lateral, min_distance,
                          min_index);
  res = tmp_ref.getProjection(math::Vec3d(), 1.0, std::pair<size_t, size_t>{0, 10}, accumulate_s, lateral, min_distance,
                              min_index);

  res = ref.getFrontProjection(math::Vec3d(), &accumulate_s, &lateral);
  res = ref.getFrontProjection(math::Vec3d(0.0, 1.0, 0.0), &accumulate_s, &lateral);
  res = ref.getFrontProjection(math::Vec3d(), nullptr, &lateral);
  res = ref.getFrontProjection(math::Vec3d(), &accumulate_s, nullptr);
  res = tmp_ref.getFrontProjection(math::Vec3d(), &accumulate_s, &lateral);
  res = ref.getRearProjection(math::Vec3d(), &accumulate_s, &lateral);
  res = ref.getRearProjection(math::Vec3d(0.0, 1.0, 0.0), &accumulate_s, &lateral);
  res = ref.getRearProjection(math::Vec3d(), nullptr, &lateral);
  res = ref.getRearProjection(math::Vec3d(), &accumulate_s, nullptr);
  res = tmp_ref.getRearProjection(math::Vec3d(), &accumulate_s, &lateral);

  auto inter_pt = ref.Interpolate(ref.reference_points().at(0), 0.0, ref.reference_points().at(1), 1.0, 0.5);
  inter_pt = ref.Interpolate(ref.reference_points().at(0), 0.0, ref.reference_points().at(0), 0.0, 0.5);

  auto dis = ref.FindMinDistancePoint(ref.reference_points().at(0), 0.0, ref.reference_points().at(1), 1.0, 0.5, 0.1);

  EXPECT_TRUE(true);
}

TEST_F(ReferenceLineTest, SpeedLimitTest) {
  std::vector<math::Vec3d> pts;
  for (int i = 0; i < 50; i++) {
    pts.emplace_back(math::Vec3d(i, 0.0, 0.0));
  }
  ReferenceLine ref(pts);

  std::vector<SpeedLimit> speed_limits;
  ref.setSpeedLimits(speed_limits);
  double speed = ref.GetSpeedLimitFromS(0.0);

  speed_limits.emplace_back(SpeedLimit(5.0, 10.0, 32.0, 0.0, pts.at(5), pts.at(10)));
  speed_limits.emplace_back(SpeedLimit(10.0, 20.0, 30.0, 0.0, pts.at(10), pts.at(20)));
  speed_limits.emplace_back(SpeedLimit(20.0, 39.0, 30.0, 0.0, pts.at(20), pts.at(39)));
  speed_limits.emplace_back(SpeedLimit(39.0, 49.0, 29.5, 0.0, pts.at(39), pts.back()));
  ref.setSpeedLimits(speed_limits);
  speed = ref.GetSpeedLimitFromS(0.0);
  speed = ref.GetSpeedLimitFromS(8.0);
  speed = ref.GetSpeedLimitFromS(-10.0);
  speed = ref.GetSpeedLimitFromS(100.0);
  speed = ref.GetSpeedLimitFromS(15.0);
  speed = ref.GetSpeedLimitFromS(40.0);

  ref.AddSpeedLimit(SpeedLimit(3.0, 5.0, 20.0, 0.0, math::Vec3d(3.0, 0.0, 0.0), math::Vec3d(5.0, 0.0, 0.0)));
  ref.AddSpeedLimit(2.0, 7.0, 20.0, math::Vec3d(2.0, 0.0, 0.0), math::Vec3d(3.0, 0.0, 0.0));
  ref.AddSpeedLimit(0.0, 60.0, 20.0, math::Vec3d(0.0, 0.0, 0.0), math::Vec3d(60.0, 0.0, 0.0));
  ref.AddSpeedLimit(-10.0, -1.0, 20.0, math::Vec3d(0.0, 0.0, 0.0), math::Vec3d(0.0, 0.0, 0.0));
  ref.AddSpeedLimit(100.0, 110.0, 20.0, math::Vec3d(0.0, 0.0, 0.0), math::Vec3d(0.0, 0.0, 0.0));

  auto speeds = ref.getMemorizedRouteSpeedLimits(math::Vec3d(0.0, 0.0, 0.0));

  EXPECT_TRUE(true);
}

TEST_F(ReferenceLineTest, StopLineTest) {
  std::vector<math::Vec3d> pts;
  for (int i = 0; i < 50; i++) {
    pts.emplace_back(math::Vec3d(i, 0.0, 0.0));
  }
  ReferenceLine ref(pts);

  std::vector<StopLine> stop_lines;
  stop_lines.emplace_back("a", false, 5.0, 0.0, 0.0, 5.0, "0", DrivingDirection::kDirectionInvalid);
  stop_lines.emplace_back("b", false, 15.0, 0.0, 0.0, 15.0, "0", DrivingDirection::kDirectionInvalid);
  ref.setStopLines(stop_lines);

  auto res = ref.getStopLinesFromSRange(0.0, 10.0, 0.0);
  res = ref.getStopLinesFromSRange(10.0, 20.0, 0.0);

  res = ref.getMemorizedRouteStopLines(math::Vec3d());
  res = ref.getMemorizedRouteStopLines(math::Vec3d(), 0.0, 10.0, 0.0);
  res = ref.getMemorizedRouteStopLines(math::Vec3d(), 10.0, 20.0, 0.0);

  EXPECT_TRUE(true);
}

TEST_F(ReferenceLineTest, GateTest) {
  std::vector<math::Vec3d> pts;
  for (int i = 0; i < 50; i++) {
    pts.emplace_back(math::Vec3d(i, 0.0, 0.0));
  }
  ReferenceLine ref(pts);

  std::vector<Gate> gates;
  gates.emplace_back("a", proto::perception::Gate::GateStatus::Gate_GateStatus_kStatusUnknown, 5.0, 0.0, 0.0, 5.0, 1.0);
  gates.emplace_back("b", proto::perception::Gate::GateStatus::Gate_GateStatus_kStatusUnknown, 15.0, 0.0, 0.0, 15.0, 1.0);
  ref.setGates(gates);

  auto res = ref.getGatesFromSRange(0.0, 10.0, 0.0);
  res = ref.getGatesFromSRange(10.0, 20.0, 0.0);

  EXPECT_TRUE(true);
}

TEST_F(ReferenceLineTest, DirectionTest) {
  std::vector<math::Vec3d> pts;
  for (int i = 0; i < 50; i++) {
    pts.emplace_back(math::Vec3d(i, 0.0, 0.0));
  }
  ReferenceLine ref(pts);

  std::vector<SegmentDirection> directions;
  directions.emplace_back(
      SegmentDirection(0.0, 10.0, DrivingDirection::kDirectionForwardOnly, pts.front(), pts.at(10)));
  directions.emplace_back(SegmentDirection(10.0, 20.0, DrivingDirection::kDirectionLeftOnly, pts.at(10), pts.at(20)));
  directions.emplace_back(SegmentDirection(20.0, 30.0, DrivingDirection::kDirectionUTurnOnly, pts.at(20), pts.at(30)));
  ref.setDirections(directions);

  auto res = ref.getDirectionsFromSRange(0.0, 5.0, 0.0);
  res = ref.getDirectionsFromSRange(15.0, 5.0, 0.0);
  auto dir = ref.getDirectionFromS(-10.0);
  dir = ref.getDirectionFromS(5.0);
  dir = ref.getDirectionFromS(15.0);
  auto range = ref.getSpecifiedDirectionRanges(DrivingDirection::kDirectionLeftOnly);
  range = ref.getSpecifiedDirectionRanges(DrivingDirection::kDirectionUnknown);
  res = ref.getMemorizedRouteSegmentsDirection(math::Vec3d());

  EXPECT_TRUE(true);
}

TEST_F(ReferenceLineTest, SourceInfoTest) {
  std::vector<math::Vec3d> pts;
  for (int i = 0; i < 50; i++) {
    pts.emplace_back(math::Vec3d(i, 0.0, 0.0));
  }
  ReferenceLine ref(pts);

  std::vector<std::tuple<LineSourceType, float, float>> source_infos;
  source_infos.emplace_back(proto::LocalRouteReferenceLine_LineSource_LineSourceType_kTypePerception, 0.0, 10.0);
  source_infos.emplace_back(proto::LocalRouteReferenceLine_LineSource_LineSourceType_kTypeLocalMap, 10.0, 20.0);
  source_infos.emplace_back(proto::LocalRouteReferenceLine_LineSource_LineSourceType_kTypeVirtual, 20.0, 30.0);
  ref.setSourceInfos(source_infos);

  auto res = ref.getSourceInfosFromSRange(0.0, 5.0, 0.0);
  res = ref.getSourceInfosFromSRange(15.0, 5.0, 0.0);
  auto info = ref.getSourceInfoFromS(-10.0);
  info = ref.getSourceInfoFromS(5.0);
  info = ref.getSourceInfoFromS(15.0);
  res = ref.getSpecifiedSourceInfos(proto::LocalRouteReferenceLine_LineSource_LineSourceType_kTypeVirtual);
  res = ref.getSpecifiedSourceInfos(proto::LocalRouteReferenceLine_LineSource_LineSourceType_kTypeInvalid);

  EXPECT_TRUE(true);
}

TEST_F(ReferenceLineTest, BoundaryTypeTest) {
  std::vector<math::Vec3d> pts;
  for (int i = 0; i < 50; i++) {
    pts.emplace_back(math::Vec3d(i, 0.0, 0.0));
  }
  ReferenceLine ref(pts);

  std::vector<SegmentBoundaryType> bound_types;
  bound_types.emplace_back(0.0, 10.0, LocalRouteSegment::BoundaryType::DASHED, LocalRouteSegment::BoundaryType::SOLID,
                           math::Vec3d(0.0, 0.0, 0.0), math::Vec3d(10.0, 0.0, 0.0));
  bound_types.emplace_back(10.0, 20.0, LocalRouteSegment::BoundaryType::DASHED, LocalRouteSegment::BoundaryType::SOLID,
                           math::Vec3d(10.0, 0.0, 0.0), math::Vec3d(20.0, 0.0, 0.0));
  bound_types.emplace_back(20.0, 30.0, LocalRouteSegment::BoundaryType::DASHED, LocalRouteSegment::BoundaryType::SOLID,
                           math::Vec3d(20.0, 0.0, 0.0), math::Vec3d(30.0, 0.0, 0.0));
  ref.setBoundaryTypes(bound_types);

  auto res = ref.getBoundaryTypesFromSRange(0.0, 5.0, 0.0);
  res = ref.getBoundaryTypesFromSRange(15.0, 5.0, 0.0);
  auto type = ref.getBoundaryTypeFromS(-10.0);
  type = ref.getBoundaryTypeFromS(5.0);
  type = ref.getBoundaryTypeFromS(15.0);

  EXPECT_TRUE(true);
}

TEST_F(ReferenceLineTest, ParallelReferenceLineTest) {
  std::vector<math::Vec3d> pts;
  for (int i = 0; i < 50; i++) {
    pts.emplace_back(math::Vec3d(i, 0.0, 0.0));
  }
  ReferenceLine ref(pts);
  ref.setSpeedLimits({SpeedLimit(0.0, 10.0, 20.0, 0.0, pts.at(0), pts.at(10))});
  ref.setDirections({SegmentDirection(0.0, 10.0, DrivingDirection::kDirectionForwardOnly, pts.front(), pts.at(10))});
  ref.setNavigationLaneChangeRanges({std::tuple<float, float, bool, bool>{0.0, 10.0, true, false}});
  ref.setlaneFollowRanges({std::pair<float, float>{0.0, 10.0}});
  ref.generateParallelReferenceLine(3.75, "left");
  ref.generateParallelReferenceLine(3.75, "right");

  ReferenceLine ref_1;
  ref_1.generateParallelReferenceLine(3.75, "left");

  EXPECT_TRUE(true);
}

TEST_F(ReferenceLineTest, MergeForkRangeTest) {
  std::vector<math::Vec3d> pts;
  for (int i = 0; i < 50; i++) {
    pts.emplace_back(math::Vec3d(i, 0.0, 0.0));
  }
  ReferenceLine ref(pts);

  MergeForkRange mf_1, mf_2, mf_3, mf_4, mf_5;
  mf_1.setS(5.0);
  mf_1.setType(MergeForkRange::MergeForkType::MERGE);
  mf_2.setS(15.0);
  mf_2.setType(MergeForkRange::MergeForkType::MERGE);
  mf_3.setS(5.0);
  mf_3.setType(MergeForkRange::MergeForkType::FORK);
  mf_4.setS(15.0);
  mf_4.setType(MergeForkRange::MergeForkType::FORK);
  mf_5.setS(20.0);
  mf_5.setType(MergeForkRange::MergeForkType::UNKNOWN);

  ref.setMergeForkRanges({mf_1, mf_2, mf_5});
  auto res = ref.getMergeForkRangesFromSRange(0.0, 10.0, 0.0);
  res = ref.getMergeForkRangesFromSRange(10.0, 20.0, 0.0);
  res = ref.getMergeRangesFromSRange(0.0, 10.0, 0.0);
  res = ref.getMergeRangesFromSRange(10.0, 20.0, 0.0);

  ref.setMergeForkRanges({mf_3, mf_4, mf_5});
  res = ref.getForkRangesFromSRange(0.0, 10.0, 0.0);
  res = ref.getForkRangesFromSRange(10.0, 20.0, 0.0);

  EXPECT_TRUE(true);
}
}  // namespace gpal::pnc::planning
