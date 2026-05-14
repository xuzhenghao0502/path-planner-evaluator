#include "bound_parser/speed_wall_process.h"

namespace gpal::pnc::planning {
bool SpeedWallProcess::init(LongitudinalBoundParserProfile profile) {
  profile_ = profile;
  config_manager_ = Singleton<ConfigManager>::get_instance();
  CHECK_NOTNULL(config_manager_);
  vehicle_param_ = config_manager_->vehicle_config().vehicle_param();
  return true;
}

void SpeedWallProcess::getSpeedWall(const ReferenceLineInfo* reference_line_info, const Chassis* chassis,
                                    const unique_ptr<DataManager::VehicleInfo>& vehicle_info,
                                    const pair<bool, double>& destination_remain_dis_info,
                                    const DecisionResult& decision_result, std::vector<SpeedWall>* speed_walls) {
  speed_walls->clear();
  addTflSpeedWall(reference_line_info, chassis, vehicle_info, decision_result.getTrafficLightDecision(), speed_walls);
  addDecisionStopSpeedWall(vehicle_info, decision_result.getLongitudinalBoundaryDecision(), speed_walls);
  addDestinationSpeedWall(destination_remain_dis_info, speed_walls);
}

void SpeedWallProcess::addTflSpeedWall(const ReferenceLineInfo* reference_line_info, const Chassis* chassis,
                                       const unique_ptr<DataManager::VehicleInfo>& vehicle_info,
                                       const TrafficLightDecision& traffic_light_decision,
                                       std::vector<SpeedWall>* speed_walls) {
  if (reference_line_info == nullptr || !reference_line_info->isValid()) {
    return;
  }
  const float stop_buffer = 1.0;
  double vehicle_local_s = get<0>(vehicle_info->sl_info.first);
  auto stop_lines = reference_line_info->ref_line().getStopLinesFromSRange(vehicle_local_s);

  if (!stop_lines.empty() && traffic_light_decision.is_need_stop) {
    double stop_s = stop_lines.front().s - vehicle_local_s - vehicle_param_.front_edge_to_ego() - stop_buffer;

    double desired_acc_min = -2.5;
    double t_delay = 0.2;                            // 延迟
    double allowed_cross = vehicle_param_.length();  // 最大越线距离( 自车长)
    double ego_speed = chassis->Speed();             // 自车速度
    double stop_distance = ego_speed * t_delay - ego_speed * ego_speed / (2 * desired_acc_min);  // 制动距离
    double tfl_block_s = 300;
    double safe_distance = 1.0;
    double tfl_to_front_edge = fmax(stop_s, kMathEpsilon);
    if (profile_.allow_exceed_stop_line()) {
      if (stop_distance < allowed_cross + tfl_to_front_edge) {  // 越线范围内可以刹停
        tfl_block_s = fmax(tfl_to_front_edge, stop_distance);
        tfl_block_s = fmax(tfl_block_s, kMathEpsilon);
      } else if (tfl_to_front_edge > allowed_cross) {  // 距离红绿灯较远且车速较快
        tfl_block_s = tfl_to_front_edge;
      }
    } else {
      tfl_block_s = tfl_to_front_edge;
    }
    SpeedWall tfl_speed_wall;
    tfl_speed_wall.type = TFL_WALL;
    tfl_speed_wall.st_wall.clear();
    tfl_speed_wall.st_wall.emplace_back(math::Vec2d(0.0, tfl_block_s));
    tfl_speed_wall.st_wall.emplace_back(math::Vec2d(time_horizon_, tfl_block_s));
    tfl_speed_wall.stop_distance = safe_distance;
    speed_walls->emplace_back(tfl_speed_wall);
  }
}

void SpeedWallProcess::addDecisionStopSpeedWall(const unique_ptr<DataManager::VehicleInfo>& vehicle_info,
                                                const LongitudinalBoundDecision& longitudinal_bound_decision,
                                                std::vector<SpeedWall>* speed_walls) {
  for (auto& boundary_constraint : longitudinal_bound_decision) {
    if (boundary_constraint.type == WallType::LONG_JUNCTION_STOP
        || boundary_constraint.type == WallType::LONG_RSA_WALL) {
      double vehicle_local_s = get<0>(vehicle_info->sl_info.first);
      double decision_speed_limit = boundary_constraint.v;
      double stop_s = boundary_constraint.s - vehicle_local_s;

      SpeedWall decision_speed_wall;
      decision_speed_wall.type = JUNCTION_STOP;
      decision_speed_wall.st_wall.clear();
      double destination_block_s = fmax(stop_s, kMathEpsilon);
      decision_speed_wall.st_wall.emplace_back(math::Vec2d(0.0, destination_block_s));
      decision_speed_wall.st_wall.emplace_back(math::Vec2d(time_horizon_, destination_block_s));
      decision_speed_wall.stop_distance = 0.0;
      speed_walls->emplace_back(decision_speed_wall);
    }
    if (boundary_constraint.type == WallType::LONG_VIRTUAL_STOP_WALL) {
      double stop_s = boundary_constraint.s;
      SpeedWall decision_speed_wall;
      decision_speed_wall.type = VIRTUAL_LINE;
      decision_speed_wall.st_wall.clear();
      double virtual_stop_s = fmax(stop_s, kMathEpsilon);
      decision_speed_wall.st_wall.emplace_back(math::Vec2d(0.0, virtual_stop_s));
      decision_speed_wall.st_wall.emplace_back(math::Vec2d(time_horizon_, virtual_stop_s));
      decision_speed_wall.stop_distance = 0.0;
      speed_walls->emplace_back(decision_speed_wall);
    }
  }
}

void SpeedWallProcess::addDestinationSpeedWall(const pair<bool, double>& destination_remain_dis_info,
                                               std::vector<SpeedWall>* speed_walls) {
  bool consider_ref_end = destination_remain_dis_info.first;
  double ref_remain_dis = destination_remain_dis_info.second;
  if (!consider_ref_end) {
    return;
  }
  SpeedWall destination_speed_wall;
  destination_speed_wall.type = DESTINATION_WALL;
  destination_speed_wall.st_wall.clear();
  destination_speed_wall.st_wall.emplace_back(math::Vec2d(0.0, ref_remain_dis));
  destination_speed_wall.st_wall.emplace_back(math::Vec2d(time_horizon_, ref_remain_dis));
  destination_speed_wall.stop_distance = 0.0;
  speed_walls->emplace_back(destination_speed_wall);
}

}  // namespace gpal::pnc::planning