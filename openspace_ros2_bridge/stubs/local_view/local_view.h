#pragma once

#include "local_view/Freespace.h"
#include "local_view/ParkingLot.h"
#include "local_view/chassis.h"
#include "local_view/console.h"
#include "local_view/decision_result_proto.h"
#include "local_view/localization.h"
#include "local_view/locator_compensator.h"
#include "local_view/scenario_odd.h"

#include "adas_data/adas_road_marking.h"
#include "center_lines/road_structure.h"
#include "decision_data/decision_common.h"
#include "decision_data/decision_result.h"
#include "decision_data/polyline.h"
#include "gpal-interface/map_server/routing_info_sd.pb.h"
#include "memorized_route/memorized_route.h"
#include "navigation_data/env_road_cognition.h"
#include "navigation_data/local_map.h"
#include "navigation_data/local_router.h"
#include "navigation_data/navigation.h"
#include "navigation_data/perception_road_marking.h"
#include "navigation_data/perception_road_structure.h"
#include "obstacle/obstacle.h"
#include "proto/vehicle_state/vehicle_state.pb.h"
#include "road_instance/env_road_instance.h"

#include <limits>
#include <memory>
#include <string>
#include <vector>

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
  LocalView() = default;
  ~LocalView() = default;

  const std::shared_ptr<VehiclePoseCompensator>& getVehiclePoseCompensatorPtr() const { return vehicle_pose_compensator_; }
  const std::shared_ptr<Freespace>& getFreespacePtr() const { return freespace_; }
  std::shared_ptr<Freespace> getMutableFreespacePtr() { return freespace_; }
  std::string DebugString() const { return ""; }

 private:
  std::shared_ptr<VehiclePoseCompensator> vehicle_pose_compensator_;
  std::shared_ptr<Freespace> freespace_;
};

}  // namespace gpal::pnc::planning
