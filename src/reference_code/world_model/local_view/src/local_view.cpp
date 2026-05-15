#include "local_view/local_view.h"

namespace gpal::pnc::planning {

LocalView::LocalView() {
  vehicle_pose_compensator_ = std::make_shared<VehiclePoseCompensator>();
  odometry_compensator_ = std::make_shared<VehiclePoseCompensator>();
  loc_ = std::make_shared<Localization>();
  vehicle_pose_ = std::make_shared<Localization>();
  odometry_ = std::make_shared<Localization>();
  odometry_senario_info_ = std::make_shared<OdometrySenarioInfo>();
  memorized_route_ = std::make_shared<MemorizedRoute>();
  chassis_ = std::make_shared<Chassis>();
  road_structure_ = std::make_shared<RoadStructure>();
  adas_road_marking_ = std::make_shared<adas::AdasRoadMarking>();
  obstacles_ = std::make_shared<IndexedObstacles>();
  ego_frame_obstacles_ = std::make_shared<IndexedObstacles>();
  vehicle_state_ = std::make_shared<gpal::pnc::VehicleState>();
  freespace_ = std::make_shared<Freespace>();
  local_router_ = std::make_shared<LocalRouter>();
  parklot_ = std::make_shared<ParkingLot>();
  console_ = std::make_shared<Console>();
  adas_console_ = std::make_shared<Console>();
  road_marking_ = std::make_shared<PerceptionRoadMarking>();
  perception_road_structure_ = std::make_shared<PerceptionRoadStructure>();
  routing_info_sd_ = std::make_shared<gpal::proto::RoutingInfoSd>();
  local_map_ = std::make_shared<LocalMap>();
  navigation_ = std::make_shared<Navigation>();
  env_road_cognition_ = std::make_shared<EnvRoadCognition>();
  polylines_perception_boundary_ = std::make_shared<PolylinesPerceptionBoundary>();
  road_instance_ = std::make_shared<road_instance::EnvRoadInstance>();
  decision_result_proto_ = std::make_shared<DecisionResultProto>();
}

}  // namespace gpal::pnc::planning
