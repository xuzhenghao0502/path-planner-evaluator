#include "local_view/scenario_odd.h"

namespace gpal::pnc::planning {

ScenarioOdd::ScenarioOdd() { Clear(); }

ScenarioOdd::~ScenarioOdd() {}

void ScenarioOdd::Clear() {
  acc_odd_.set_is_speed_domain_satisfied(false);
  acc_odd_.set_is_acc_odd_satisfied(false);

  lcc_odd_.set_is_ego_lane_center_line_exist(false);
  lcc_odd_.set_is_ego_lane_width_satisfied(false);
  lcc_odd_.set_is_driving_in_ego_lane(false);
  lcc_odd_.set_is_lcc_odd_satisfied(false);

  noa_odd_.set_is_navigation_route_set_ready(false);
  noa_odd_.set_is_in_highway_noa_area(false);
  noa_odd_.set_distance_to_highway_noa_disabled(-1);
  noa_odd_.set_route_id(-1);
  noa_odd_.set_is_navigation_route_need_replan(false);
  noa_odd_.set_is_route_goal_arrived(false);
  noa_odd_.set_is_noa_odd_satisfied(false);

  hd_function_odd_.set_is_hd_function_odd_satisfied(false);
  hd_function_odd_.set_is_hd_route_ready(false);
  hd_function_odd_.set_is_hd_ego_lane_exist(false);
  hd_function_odd_.set_is_hd_ego_lane_lateral_error_satisfied(false);
  hd_function_odd_.set_is_hd_ego_lane_heading_error_satisfied(false);

  target_function_state_ = proto::TaskResponse::kInvalidFunctionState;
}

}  // namespace gpal::pnc::planning
