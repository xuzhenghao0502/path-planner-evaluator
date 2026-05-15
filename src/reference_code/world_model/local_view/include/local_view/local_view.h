#pragma once

#include "Freespace.h"
#include "ParkingLot.h"
#include "adas_data/adas_road_marking.h"
#include "center_lines/road_structure.h"
#include "chassis.h"
#include "console.h"
#include "decision_data/decision_common.h"
#include "decision_data/polyline.h"
#include "gpal-interface/map_server/routing_info_sd.pb.h"
#include "localization.h"
#include "locator_compensator.h"
#include "memorized_route/memorized_route.h"
#include "navigation_data/env_road_cognition.h"
#include "navigation_data/local_map.h"
#include "navigation_data/local_router.h"
#include "navigation_data/navigation.h"
#include "navigation_data/perception_road_marking.h"
#include "navigation_data/perception_road_structure.h"
#include "obstacle/obstacle.h"
#include "proto/vehicle_state/vehicle_state.pb.h"
#include "scenario_odd.h"
#include "road_instance/env_road_instance.h"
#include "decision_result_proto.h"

namespace gpal::pnc::planning {

struct OdometrySenarioInfo {
  bool is_odometry_senario = true;
  float ego_env_road_cognition_lateral_error = std::numeric_limits<float>::max();
  float ego_env_road_cognition_heading_error = std::numeric_limits<float>::max();
  float ego_memorized_route_lateral_error = std::numeric_limits<float>::max();
  float ego_memorized_route_heading_error = std::numeric_limits<float>::max();

  void reset() {
    is_odometry_senario = true;
    ego_env_road_cognition_lateral_error = std::numeric_limits<float>::max();
    ego_env_road_cognition_heading_error = std::numeric_limits<float>::max();
    ego_memorized_route_lateral_error = std::numeric_limits<float>::max();
    ego_memorized_route_heading_error = std::numeric_limits<float>::max();
  }
};

class LocalView {
 public:
  LocalView();
  ~LocalView() = default;

  const std::shared_ptr<VehiclePoseCompensator>& getVehiclePoseCompensatorPtr() const {
    return vehicle_pose_compensator_;
  }
  std::shared_ptr<VehiclePoseCompensator> getMutableVehiclePoseCompensatorPtr() { return vehicle_pose_compensator_; }

  const std::shared_ptr<VehiclePoseCompensator>& getOdometryCompensatorPtr() const { return odometry_compensator_; }
  std::shared_ptr<VehiclePoseCompensator> getMutableOdometryCompensatorPtr() { return odometry_compensator_; }

  const std::shared_ptr<gpal::pnc::VehicleState>& getVehicleStatePtr() const { return vehicle_state_; }
  std::shared_ptr<gpal::pnc::VehicleState> getMutableVehicleStatePtr() { return vehicle_state_; }

  const std::shared_ptr<Localization>& getLocalizationPtr() const { return loc_; }
  std::shared_ptr<Localization> getMutableLocalizationPtr() { return loc_; }

  const std::shared_ptr<Localization>& getVehiclePosePtr() const { return vehicle_pose_; }
  std::shared_ptr<Localization> getMutableVehiclePosePtr() { return vehicle_pose_; }

  const std::shared_ptr<Localization>& getOdometryPtr() const { return odometry_; }
  std::shared_ptr<Localization> getMutableOdometryPtr() { return odometry_; }

  const std::shared_ptr<OdometrySenarioInfo>& getOdometrySenarioInfoPtr() const { return odometry_senario_info_; }
  std::shared_ptr<OdometrySenarioInfo> getMutableOdometrySenarioInfoPtr() { return odometry_senario_info_; }

  const std::shared_ptr<MemorizedRoute>& getMemorizedRoutePtr() const { return memorized_route_; }
  std::shared_ptr<MemorizedRoute> getMutableMemorizedRoutePtr() { return memorized_route_; }

  const std::shared_ptr<Chassis>& getChassisPtr() const { return chassis_; }
  std::shared_ptr<Chassis> getMutableChassisPtr() { return chassis_; }

  const std::shared_ptr<RoadStructure>& getRoadStructurePtr() const { return road_structure_; }
  std::shared_ptr<RoadStructure> getMutableRoadStructurePtr() { return road_structure_; }

  const std::shared_ptr<adas::AdasRoadMarking>& getAdasRoadMarkingPtr() const { return adas_road_marking_; }
  std::shared_ptr<adas::AdasRoadMarking> getMutableAdasRoadMarkingPtr() { return adas_road_marking_; }

  const std::shared_ptr<PerceptionRoadMarking>& getPerceptionRoadMarkingPtr() const { return road_marking_; }
  std::shared_ptr<PerceptionRoadMarking> getMutablePerceptionRoadMarkingPtr() { return road_marking_; }

  const std::shared_ptr<PerceptionRoadStructure>& getPerceptionRoadStructurePtr() const {
    return perception_road_structure_;
  }
  std::shared_ptr<PerceptionRoadStructure> getMutablePerceptionRoadStructurePtr() { return perception_road_structure_; }

  const std::shared_ptr<gpal::proto::RoutingInfoSd>& getRoutingInfoSdPtr() const { return routing_info_sd_; }
  std::shared_ptr<gpal::proto::RoutingInfoSd> getMutableRoutingInfoSdPtr() { return routing_info_sd_; }

  const std::shared_ptr<LocalMap>& getLocalMapPtr() const { return local_map_; }
  std::shared_ptr<LocalMap> getMutableLocalMapPtr() { return local_map_; }

  const std::shared_ptr<IndexedObstacles>& getIndexedObstaclesPtr() const { return obstacles_; }
  std::shared_ptr<IndexedObstacles> getMutableIndexedObstaclesPtr() { return obstacles_; }

  const std::shared_ptr<IndexedObstacles>& getEgoFrameIndexedObstaclesPtr() const { return ego_frame_obstacles_; }
  std::shared_ptr<IndexedObstacles> getMutableEgoFrameIndexedObstaclesPtr() { return ego_frame_obstacles_; }

  const std::shared_ptr<Freespace>& getFreespacePtr() const { return freespace_; }
  std::shared_ptr<Freespace> getMutableFreespacePtr() { return freespace_; }

  const std::shared_ptr<LocalRouter>& getLocalRouterPtr() const { return local_router_; }
  std::shared_ptr<LocalRouter> getMutableLocalRouterPtr() { return local_router_; }

  const std::shared_ptr<ParkingLot>& getParkLotPtr() const { return parklot_; }
  std::shared_ptr<ParkingLot> getMutableParkLotPtr() { return parklot_; }

  const std::shared_ptr<Console>& getConsolePtr() const { return console_; }
  std::shared_ptr<Console> getMutableConsolePtr() { return console_; }

  const std::shared_ptr<Console>& getAdasConsolePtr() const { return adas_console_; }
  std::shared_ptr<Console> getMutableAdasConsolePtr() { return adas_console_; }

  const std::shared_ptr<Navigation>& getNavigationPtr() const { return navigation_; }
  std::shared_ptr<Navigation> getMutableNavigationPtr() { return navigation_; }

  const std::shared_ptr<EnvRoadCognition>& getEnvRoadCognitionPtr() const { return env_road_cognition_; }
  std::shared_ptr<EnvRoadCognition> getMutableEnvRoadCognitionPtr() { return env_road_cognition_; }

  std::string DebugString() const { return debugString_; }

  void setPerceptionObstacleTimestamp(double timestamp) { perception_obstacle_timestamp_ = timestamp; }
  double getPerceptionObstacleTimestamp() const { return perception_obstacle_timestamp_; }

  const std::shared_ptr<PolylinesPerceptionBoundary>& getPolylinesPerceptionBoundaryPtr() const {
    return polylines_perception_boundary_;
  }
  std::shared_ptr<PolylinesPerceptionBoundary> getMutablePolylinesPerceptionBoundaryPtr() {
    return polylines_perception_boundary_;
  }
  void SetPolylinesPerceptionBoundary(
      const std::shared_ptr<PolylinesPerceptionBoundary>& polylines_perception_boundary) {
    polylines_perception_boundary_->set_forward_range(std::move(polylines_perception_boundary->forward_range()));
    polylines_perception_boundary_->set_backward_range(std::move(polylines_perception_boundary->backward_range()));
    polylines_perception_boundary_->set_left_range(std::move(polylines_perception_boundary->left_range()));
    polylines_perception_boundary_->set_right_range(std::move(polylines_perception_boundary->right_range()));
    polylines_perception_boundary_->set_fpoint_resolution(
        std::move(polylines_perception_boundary->fpoint_resolution()));
    polylines_perception_boundary_->mutable_polylines() = std::move(polylines_perception_boundary->polylines());
    polylines_perception_boundary_->mutable_polylines_within_one_lane_width() =
        std::move(polylines_perception_boundary->polylines_within_one_lane_width());
    polylines_perception_boundary_->mutable_polylines_within_two_lane_width() =
        std::move(polylines_perception_boundary->polylines_within_two_lane_width());
    polylines_perception_boundary_->mutable_polylines_outside_two_lane_width() =
        std::move(polylines_perception_boundary->polylines_outside_two_lane_width());
    polylines_perception_boundary_->set_status(std::move(polylines_perception_boundary->status()));
    polylines_perception_boundary_->set_base_relative_time(
        std::move(polylines_perception_boundary->get_base_relative_time()));
  }
  const std::shared_ptr<road_instance::EnvRoadInstance>& getEnvRoadInstancePtr() const { return road_instance_; }
  std::shared_ptr<road_instance::EnvRoadInstance> getMutableEnvRoadInstancePtr() { return road_instance_; }
  
  const std::shared_ptr<DecisionResultProto>& getDecisionResultProtoPtr() const { return decision_result_proto_; }
  std::shared_ptr<DecisionResultProto> getMutableDecisionResultProtoPtr() { return decision_result_proto_; }

 private:
  std::shared_ptr<VehiclePoseCompensator> vehicle_pose_compensator_ = nullptr;
  std::shared_ptr<VehiclePoseCompensator> odometry_compensator_ = nullptr;
  std::shared_ptr<Localization> loc_ = nullptr;
  std::shared_ptr<Localization> vehicle_pose_ = nullptr;
  std::shared_ptr<Localization> odometry_ = nullptr;
  std::shared_ptr<OdometrySenarioInfo> odometry_senario_info_ = nullptr;
  std::shared_ptr<MemorizedRoute> memorized_route_ = nullptr;
  std::shared_ptr<Chassis> chassis_ = nullptr;
  std::shared_ptr<RoadStructure> road_structure_ = nullptr;
  std::shared_ptr<adas::AdasRoadMarking> adas_road_marking_ = nullptr;
  std::shared_ptr<PerceptionRoadMarking> road_marking_ = nullptr;
  std::shared_ptr<PerceptionRoadStructure> perception_road_structure_ = nullptr;
  std::shared_ptr<gpal::proto::RoutingInfoSd> routing_info_sd_ = nullptr;
  std::shared_ptr<LocalMap> local_map_ = nullptr;
  std::shared_ptr<IndexedObstacles> obstacles_ = nullptr;
  std::shared_ptr<IndexedObstacles> ego_frame_obstacles_ = nullptr;
  std::shared_ptr<Freespace> freespace_ = nullptr;
  std::shared_ptr<ParkingLot> parklot_ = nullptr;
  std::shared_ptr<Console> console_ = nullptr;
  std::shared_ptr<Console> adas_console_ = nullptr;
  std::shared_ptr<Navigation> navigation_ = nullptr;
  std::shared_ptr<EnvRoadCognition> env_road_cognition_ = nullptr;
  std::shared_ptr<PolylinesPerceptionBoundary> polylines_perception_boundary_ = nullptr;
  std::shared_ptr<road_instance::EnvRoadInstance> road_instance_ = nullptr;
  std::shared_ptr<DecisionResultProto> decision_result_proto_ = nullptr;

  std::shared_ptr<gpal::pnc::VehicleState> vehicle_state_ = nullptr;
  std::shared_ptr<LocalRouter> local_router_ = nullptr;
  std::string debugString_ = "";

  double perception_obstacle_timestamp_ = -1.0;
};

}  // namespace gpal::pnc::planning
