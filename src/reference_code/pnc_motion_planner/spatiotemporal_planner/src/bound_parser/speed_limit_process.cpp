#include "bound_parser/speed_limit_process.h"

namespace gpal::pnc::planning {

bool SpeedLimitProcess::init(LongitudinalBoundParserProfile profile) {
  profile_ = profile;
  config_manager_ = Singleton<ConfigManager>::get_instance();
  CHECK_NOTNULL(config_manager_);
  vehicle_param_ = config_manager_->vehicle_config().vehicle_param();
  return true;
}

void SpeedLimitProcess::reset() {
  speed_limits_.clear();
  map_speed_limit_.clear();
  curve_speed_limit_.clear();
  max_speed_limit_ = kMaxSpeedMS;
  last_steer_curve_acc_limit_ = false;
  init_v_ = 0.0;
  init_index_ = 0;

  is_turn_around_ = false;
  last_turn_state_ = 0;

  last_map_speed_limit_ = kMaxSpeedMS;
  last_console_speed_limit_ = kMaxSpeedMS;
  speed_limit_type_ = SpeedLimitType::kMap;
}

std::vector<SpatialSpeedLimit> SpeedLimitProcess::getSpeedLimit(
    const ReferenceLineInfo* reference_line_info, const math::IntervalData<ReferencePoint>& interval_reference_points,
    const Console* console, const Chassis* chassis, const unique_ptr<DataManager::VehicleInfo>& vehicle_info,
    const StageState& stage_state, const proto::road_cognition::ScenarioInfo& scenario_info, const DecisionResult& decision_result) {
  auto& reference_points = interval_reference_points.getOriginData();
  init_v_ = vehicle_info->start_point.v();
  init_index_ = interval_reference_points.getIndex(vehicle_info->start_point.path_pt().s());
  STLOG(D, "[SpeedLimitProcess::getSpeedLimit] init_index: ", init_index_, ", init_v: ", init_v_,
        ", reference_points size: ", reference_points.size());

  if (init_index_ > reference_points.size() - 1) {
    init_index_ = 0;
    STLOG(W, "[SpeedLimitProcess::getSpeedLimit] init_index is out of bounds, resetting to 0");
  }
  max_speed_limit_ = kMaxSpeedMS;
  speed_limits_.resize(reference_points.size());
  for (int i = 0; i < reference_points.size(); i++) {
    speed_limits_.at(i).s = reference_points.at(i).local_s();
    speed_limits_.at(i).speed_limit = kMaxSpeedMS;
    speed_limits_.at(i).id = "default";
  }
  //
  
  if(stage_state != StageState::CityNoaDrivingStage &&
      stage_state != StageState::HighwayNoaDrivingStage){
    // 非NOA状态，重置限速
    last_map_speed_limit_ = kMaxSpeedMS;
    last_console_speed_limit_ = kMaxSpeedMS;
    speed_limit_type_ = SpeedLimitType::kMap;
    getMapSpeedlimit(reference_line_info, reference_points);
    STLOG(D, "[SpeedLimitProcess::getNoaSpeedLimit] not NOA stage, reset speed limit");
  }else{
    getNoaSpeedLimit(reference_line_info, reference_points, console, stage_state);
  }

  getCurveSpeedLimit(reference_points);
  getDecisionSpeedLimit(chassis, decision_result, vehicle_info);

  caculateTurnFlag(reference_line_info, vehicle_info);
  getTurnSpeedLimit(reference_line_info, last_turn_state_, vehicle_info);

  getConstSpeedLimit(console, stage_state, scenario_info);
  smoothSpeedLimits();
  outCurveSpeedLimit(reference_line_info, chassis, vehicle_info);

  // for (auto& speed_limit : speed_limits_) {
  //   STLOG(D, "[SpeedLimitProcess::getSpeedLimit] SpeedLimit at s: ", speed_limit.s,
  //         ", speed_limit: ", speed_limit.speed_limit, ", id: ", speed_limit.id);
  // }

  return speed_limits_;
}

void SpeedLimitProcess::getMapSpeedlimit(const ReferenceLineInfo* reference_line_info,
                                         const std::vector<ReferencePoint>& reference_points) {
  if (reference_line_info == nullptr || !reference_line_info->isValid() || reference_points.empty()) {
    return;
  }
  map_speed_limit_.resize(reference_points.size());
  auto target_reference_line = reference_line_info->ref_line();
  for (int i = 0; i < reference_points.size(); i++) {
    double speed_limit = target_reference_line.GetSpeedLimitFromS(reference_points.at(i).local_s());
    map_speed_limit_.at(i).s = reference_points.at(i).local_s();
    if (speed_limit < kMathEpsilon) {
      map_speed_limit_.at(i).speed_limit = kMaxSpeedKMH;
      map_speed_limit_.at(i).id = "default";
    } else {
      map_speed_limit_.at(i).speed_limit = speed_limit;
      map_speed_limit_.at(i).id = "map";
    }

    if (speed_limits_.at(i).speed_limit > map_speed_limit_.at(i).speed_limit) {
      speed_limits_.at(i).speed_limit = map_speed_limit_.at(i).speed_limit;
      speed_limits_.at(i).id = map_speed_limit_.at(i).id;
    }
  }
  max_speed_limit_ = std::min(max_speed_limit_, map_speed_limit_.at(init_index_).speed_limit);
}

void SpeedLimitProcess::getNoaSpeedLimit(const ReferenceLineInfo* reference_line_info,
                                         const std::vector<ReferencePoint>& reference_points, const Console* console,
                                         const StageState& stage_state) {
  //get map speed limit
  if (reference_line_info == nullptr || !reference_line_info->isValid() || reference_points.empty()) {
    return;
  }
  bool map_speed_limit_changed = false;
  bool console_speed_limit_changed = false;
  auto target_reference_line = reference_line_info->ref_line();
  auto map_speed_limits = target_reference_line.getSpeedLimits();
  double current_reference_s = reference_points.at(init_index_).local_s();
  float current_map_speed_limit = kMaxSpeedMS;
  int current_map_speed_limit_index = -1;
  if( map_speed_limits.empty()){
    STLOG(W, "[SpeedLimitProcess::getNoaSpeedLimit] map_speed_limits is empty");
    return;
  }
  for (int i = 0; i < map_speed_limits.size(); i++) {
    // ERT_PLOG_W<<"map_speed_limits at i: "<<map_speed_limits.at(i).max_speed_limit<<" start_s: "<<map_speed_limits.at(i).start_s<<" end_s: "<<map_speed_limits.at(i).end_s;
    // [道路导航处理] 基于需求：NOA状态下，地图限速40km/h，需要调整为60km/h
    // if ((map_speed_limits.at(i).max_speed_limit - 40 * KMH_MS) < 0.1) {
    //   map_speed_limits.at(i).max_speed_limit = 60 * KMH_MS;
    // }
    if (map_speed_limits.at(i).start_s <= current_reference_s && map_speed_limits.at(i).end_s >= current_reference_s) {
      current_map_speed_limit = map_speed_limits.at(i).max_speed_limit;
      current_map_speed_limit_index = i;
      break;
    }
    // ERT_PLOG_W<<"recalculate map_speed_limits at i: "<<map_speed_limits.at(i).max_speed_limit<<" start_s: "<<map_speed_limits.at(i).start_s<<" end_s: "<<map_speed_limits.at(i).end_s;
  }
  if( abs(current_map_speed_limit - last_map_speed_limit_) > 0.1) {
    map_speed_limit_changed = true;
  }

  //get console speed limit
  float current_console_speed_limit = console->accDesiredSpeed();
  if( abs(current_console_speed_limit - last_console_speed_limit_) > 0.1) {
    console_speed_limit_changed = true;
  }
  if(map_speed_limit_changed){
    speed_limit_type_ = SpeedLimitType::kMap;
  } else if(console_speed_limit_changed){
    speed_limit_type_ = SpeedLimitType::kConsole;
  }

  // ERT_PLOG_W<<"current_map_speed_limit: "<<current_map_speed_limit<<" map_speed_limit_changed: "<<(int)map_speed_limit_changed
  // <<" current_console_speed_limit: "<<current_console_speed_limit<<" console_speed_limit_changed: "<<(int)console_speed_limit_changed
  // << " speed_limit_type_: "<<(int)speed_limit_type_;

  if(speed_limit_type_ == SpeedLimitType::kConsole){
    for(int i = 0; i<= current_map_speed_limit_index; i++){
      map_speed_limits.at(i).max_speed_limit= current_console_speed_limit;
    }
    max_speed_limit_ = fmin(max_speed_limit_, current_console_speed_limit);
  }else{
    max_speed_limit_ = fmin(max_speed_limit_, current_map_speed_limit);
  }

  map_speed_limit_.resize(reference_points.size());
  for (int i = 0; i < reference_points.size(); i++) {
    double s = reference_points.at(i).local_s();
    float speed_limit = getMapSpeedLimitFromS(map_speed_limits, s);
    map_speed_limit_.at(i).s = reference_points.at(i).local_s();
    if (speed_limit < kMathEpsilon) {
      map_speed_limit_.at(i).speed_limit = kMaxSpeedKMH;
      map_speed_limit_.at(i).id = "default";
    } else {
      map_speed_limit_.at(i).speed_limit = speed_limit;
      map_speed_limit_.at(i).id = "map";
    }

    if (speed_limits_.at(i).speed_limit > map_speed_limit_.at(i).speed_limit) {
      speed_limits_.at(i).speed_limit = map_speed_limit_.at(i).speed_limit;
      speed_limits_.at(i).id = map_speed_limit_.at(i).id;
    }
  }

  last_map_speed_limit_ = current_map_speed_limit;
  last_console_speed_limit_ = current_console_speed_limit;
}

float SpeedLimitProcess::getMapSpeedLimitFromS(std::vector<SpeedLimit> map_speed_limits, double s ){
    constexpr float acc = 1.0f;
  for (int i = 0; i < map_speed_limits.size(); ++i) {
    if (s >= map_speed_limits.at(i).start_s && s <= map_speed_limits.at(i).end_s) {
      if (i + 1 < map_speed_limits.size()) {
        bool need_decelerate = false;
        double min_map_speed_limit = map_speed_limits.at(i).max_speed_limit;
        double min_dis = map_speed_limits.at(i + 1).start_s;
        for(int j = i+1; j < map_speed_limits.size(); j++){
          if (min_map_speed_limit - map_speed_limits.at(j).max_speed_limit > 1.0) {
            need_decelerate = true;
            min_map_speed_limit = map_speed_limits.at(j).max_speed_limit;
            min_dis = map_speed_limits.at(j).start_s;
          }
        }
        if (need_decelerate) {
          // need decelerate
          float max_speed_limit = map_speed_limits.at(i).max_speed_limit;
          float min_speed_limit = min_map_speed_limit;
          float dis = min_dis - s;
          float speed_limit = std::sqrt(min_speed_limit * min_speed_limit + 2.0 * acc * dis);
          if (speed_limit >= max_speed_limit) {
            speed_limit = max_speed_limit;
          }
          return speed_limit;
        } else {
          return map_speed_limits.at(i).max_speed_limit;
        }
      } else {
        return map_speed_limits.at(i).max_speed_limit;
      }
    }
  }
  float speed_limit = kMaxSpeedMS;
  return speed_limit;
}

void SpeedLimitProcess::getCurveSpeedLimit(const std::vector<ReferencePoint>& reference_points) {
  if (reference_points.empty()) {
    return;
  }

  constexpr double kKappaThreshold = 0.02;
  constexpr double kEpsilon = 1e-4;
  curve_speed_limit_.resize(reference_points.size());
  std::vector<double> kappa_speed_limits;
  kappa_speed_limits.reserve(reference_points.size());

  std::vector<double> kappa_table = {0.001, 0.002, 0.004, 0.01, 0.02, 0.05, 0.1};
  std::vector<double> lat_a_limit = {
      profile_.lateral_acc_limit_for_radius_1000(), profile_.lateral_acc_limit_for_radius_500(),
      profile_.lateral_acc_limit_for_radius_250(),  profile_.lateral_acc_limit_for_radius_100(),
      profile_.lateral_acc_limit_for_radius_50(),   profile_.lateral_acc_limit_for_radius_20(),
      profile_.lateral_acc_limit_for_radius_10()};

  for (int i = 0; i < reference_points.size(); i++) {
    double kappa = std::clamp(reference_points.at(i).kappa(), -1.0, 1.0);
    if (std::abs(kappa) < kEpsilon) {
      kappa = kEpsilon * (kappa < 0 ? -1 : 1);
    }
    double kappa_lat_a_limit = math::TableLookUp1D(kappa_table, lat_a_limit, std::fabs(kappa));
    double speed_limit = std::sqrt(std::fabs(kappa_lat_a_limit / kappa));
    speed_limit = std::fmax(speed_limit, profile_.min_curve_speed_limit() * KMH_MS);

    kappa_speed_limits.emplace_back(speed_limit);
    curve_speed_limit_.at(i).s = reference_points.at(i).local_s();
    curve_speed_limit_.at(i).speed_limit = speed_limit;
    curve_speed_limit_.at(i).id = "curve";
  }

  // Backward speed-limits consideration
  for (int i = static_cast<int>(curve_speed_limit_.size()) - 2; i >= 0; i--) {
    float acc_speed_limit =
        sqrt(abs(2 * profile_.soft_curve_acc() * (curve_speed_limit_.at(i + 1).s - curve_speed_limit_.at(i).s)
                 + curve_speed_limit_.at(i + 1).speed_limit * curve_speed_limit_.at(i + 1).speed_limit));
    curve_speed_limit_.at(i).speed_limit = std::fmin(acc_speed_limit, curve_speed_limit_.at(i).speed_limit);
  }

  // Forward speed-limits consideration

  const std::vector<double> delta_speed_table = {0.0, 2.0, 3.0, 10.0, 20.0, 30.0};
  std::vector<double> min_decel_table = {0.3, 0.6, 0.8, 1.5, 2.0, 2.0};
  std::vector<double> virtual_decelerations;
  virtual_decelerations.reserve(reference_points.size());
  for (size_t i = 0UL; i < kappa_speed_limits.size(); ++i) {
    const double delta_v = init_v_ - kappa_speed_limits[i];
    const double min_decel = math::TableLookUp1D(delta_speed_table, min_decel_table, delta_v);
    virtual_decelerations.emplace_back(min_decel);
  }

  for (size_t i = 0UL; i < curve_speed_limit_.size(); i++) {
    double s_i = curve_speed_limit_.at(i).s;
    for (size_t j = i + 1UL; j < curve_speed_limit_.size(); j++) {
      double s_j = curve_speed_limit_.at(j).s;
      double kappa_speed_limit_j = kappa_speed_limits.at(j);
      double virtual_decel_j = virtual_decelerations.at(j);
      double kappa_speed_limit_from_j_to_i =
          sqrt(fmax(kappa_speed_limit_j * kappa_speed_limit_j + 2.0F * virtual_decel_j * (s_j - s_i), 0.01F));
      curve_speed_limit_.at(i).speed_limit = fmin(curve_speed_limit_.at(i).speed_limit, kappa_speed_limit_from_j_to_i);
    }
  }

  for (int i = 0; i < curve_speed_limit_.size(); i++) {
    if (speed_limits_.at(i).speed_limit > curve_speed_limit_.at(i).speed_limit) {
      speed_limits_.at(i).speed_limit = curve_speed_limit_.at(i).speed_limit;
      speed_limits_.at(i).id = curve_speed_limit_.at(i).id;
    }
  }
}

void SpeedLimitProcess::getDecisionSpeedLimit(const Chassis* chassis, const DecisionResult& decision_result,
                                              const unique_ptr<DataManager::VehicleInfo>& vehicle_info) {
  auto& boundary_decision = decision_result.getLongitudinalBoundaryDecision();
  for (auto& boundary_constraint : boundary_decision) {
    if (boundary_constraint.type == WallType::LONG_DECC_WALL ) {
      double ego_speed = chassis->Speed();
      double decision_speed_limit = max(boundary_constraint.v, 0.0);
      double  ego_s = vehicle_info->start_point.path_pt().s();
      double distance_to_decc_wall = max(boundary_constraint.s - ego_s, 0.0);
      double desired_acc = max((decision_speed_limit * decision_speed_limit - ego_speed * ego_speed) / (2 * distance_to_decc_wall), -2.0);
      if (desired_acc > -0.5) {
        continue;
      }
      for (int i = 0; i < speed_limits_.size(); i++) {
        if (speed_limits_.at(i).s < boundary_constraint.s) {
          double speed_limit =
              sqrt(decision_speed_limit * decision_speed_limit - 2 * desired_acc * (boundary_constraint.s - speed_limits_.at(i).s));
          if (speed_limits_.at(i).speed_limit > speed_limit) {
            speed_limits_.at(i).speed_limit = speed_limit;
            speed_limits_.at(i).id = "decision";
          }
        } else if (speed_limits_.at(i).s > boundary_constraint.s && speed_limits_.at(i).speed_limit > decision_speed_limit) {
          speed_limits_.at(i).speed_limit = decision_speed_limit;
          speed_limits_.at(i).id = "decison";
        }
      }
    }
  }
}

void SpeedLimitProcess::getTurnSpeedLimit(const ReferenceLineInfo* reference_line_info, const int& turn_state,
                                          const unique_ptr<DataManager::VehicleInfo>& vehicle_info) {
  if (reference_line_info == nullptr || !reference_line_info->isValid()) {
    return;
  }
  auto ego_s = vehicle_info->start_point.path_pt().s();
  auto stop_lines = reference_line_info->ref_line().getStopLinesFromSRange(ego_s, 200, 40);
  double turn_speed_limit = kMaxSpeedMS;
  double stop_line_dis = kNegativeInfinity;
  if (!stop_lines.empty()) {
    auto stop_line = stop_lines.front();
    if (stop_line.direction == DrivingDirection::kDirectionLeftOnly
        || stop_line.direction == DrivingDirection::kDirectionRightOnly) {
      stop_line_dis = stop_line.s;
      turn_speed_limit = profile_.turn_speed_limit() * KMH_MS;
    } else if (stop_line.direction == DrivingDirection::kDirectionUTurnOnly) {
      stop_line_dis = stop_line.s;
      turn_speed_limit = profile_.u_turn_speed_limit() * KMH_MS;
    } else if (stop_line.direction == DrivingDirection::kDirectionForwardOnly) {
      stop_line_dis = stop_line.s;
      turn_speed_limit = fmax(max_speed_limit_ - profile_.forward_speed_limit_buffer() * KMH_MS, 40.0 * KMH_MS);
    }
  } else if (turn_state == 2) {  // uturn state 确保uturn不提前提速
    stop_line_dis = ego_s;
    turn_speed_limit = profile_.u_turn_speed_limit() * KMH_MS;
  } else if (turn_state == 1 && profile_.enable_turn_accelerate_limit()) {  // left turn state 确保left turn不提前提速
    stop_line_dis = ego_s;
    turn_speed_limit = profile_.turn_speed_limit() * KMH_MS;
  } else {
    return;
  }
  for (int i = 0; i < speed_limits_.size(); i++) {
    if (speed_limits_.at(i).s < stop_line_dis) {
      double acc = 1.0;
      double speed_limit =
          sqrt(turn_speed_limit * turn_speed_limit + 2 * acc * (stop_line_dis - speed_limits_.at(i).s));
      if (speed_limits_.at(i).speed_limit > speed_limit) {
        speed_limits_.at(i).speed_limit = speed_limit;
        speed_limits_.at(i).id = "turn";
      }
    } else if (speed_limits_.at(i).s >= stop_line_dis && speed_limits_.at(i).speed_limit > turn_speed_limit) {
      speed_limits_.at(i).speed_limit = turn_speed_limit;
      speed_limits_.at(i).id = "turn";
    }
  }
}

void SpeedLimitProcess::getConstSpeedLimit(const Console* console, const StageState& stage_state,
                                           const proto::road_cognition::ScenarioInfo& scenario_info) {
  pair<string, double> min_const_speed_limit = make_pair("default", static_cast<double>(kMaxSpeedMS));
  auto driving_speed_limit = getDrivingSpeedLimit(console);
  if (min_const_speed_limit.second > driving_speed_limit.second) {
    min_const_speed_limit.first = driving_speed_limit.first;
    min_const_speed_limit.second = driving_speed_limit.second;
  }
  if (stage_state == StageState::LccDrivingStage ) {
    auto lcc_driving_speed_limit = getLccDrivingSpeedLimit(console);
    if (min_const_speed_limit.second > lcc_driving_speed_limit.second) {
      min_const_speed_limit.first = lcc_driving_speed_limit.first;
      min_const_speed_limit.second = lcc_driving_speed_limit.second;
    }
  }
  
  if (stage_state == StageState::HpaDrivingStage) {
    auto hpa_speed_limit = getHpaSpeedLimit();
    if (min_const_speed_limit.second > hpa_speed_limit.second) {
      min_const_speed_limit.first = hpa_speed_limit.first;
      min_const_speed_limit.second = hpa_speed_limit.second;
    }
  }

  auto scenario_speed_limit = getScenarioSpeedLimit(scenario_info);
  if (min_const_speed_limit.second > scenario_speed_limit.second) {
    min_const_speed_limit.first = scenario_speed_limit.first;
    min_const_speed_limit.second = scenario_speed_limit.second;
  }

  max_speed_limit_ = Min(max_speed_limit_, min_const_speed_limit.second);
  updateSpeedLimit(min_const_speed_limit.second, 0.0, min_const_speed_limit.first);
}

pair<string, double> SpeedLimitProcess::getDrivingSpeedLimit(const Console* console) {
  double speed_limit = kMaxSpeedMS;
  speed_limit = Min(profile_.max_speed_limit() * KMH_MS, console->speedLimit());
  return make_pair("driving", speed_limit);
}

pair<string, double> SpeedLimitProcess::getLccDrivingSpeedLimit(const Console* console) {
  double speed_limit = kMaxSpeedMS;
  speed_limit = Min(profile_.max_speed_limit() * KMH_MS, console->accDesiredSpeed());
  return make_pair("lcc_driving", speed_limit);
}

pair<string, double> SpeedLimitProcess::getHpaSpeedLimit(){
  double speed_limit = kMaxSpeedMS;
  speed_limit = Min(profile_.max_speed_limit() * KMH_MS, profile_.hpa_speed_limit() * KMH_MS);
  return make_pair("hpa_driving", speed_limit);
}

pair<string, double> SpeedLimitProcess::getScenarioSpeedLimit(
    const proto::road_cognition::ScenarioInfo& scenario_info) {
  double speed_limit = kMaxSpeedMS;
  for (auto junction_info : scenario_info.junctions_info()) {
    if (junction_info.position_relation()
        == proto::road_cognition::JunctionInfo_PositionRelationType_kInJunction) {
      speed_limit = fmax(max_speed_limit_ - profile_.forward_speed_limit_buffer() * KMH_MS, 40.0 * KMH_MS);
    }
  }
  return make_pair("junctions", speed_limit);
}

void SpeedLimitProcess::updateSpeedLimit(double speed_limit, double speed_limit_dis, string speed_limit_type) {
  for (int i = 0; i < speed_limits_.size(); i++) {
    if (speed_limits_.at(i).s >= speed_limit_dis && speed_limits_.at(i).speed_limit > speed_limit) {
      speed_limits_.at(i).speed_limit = speed_limit;
      speed_limits_.at(i).id = speed_limit_type;
    }
  }
}

void SpeedLimitProcess::smoothSpeedLimits() {
  // Generating a VSoftUpperBound-Curve starting at init_v_
  double a_lower_for_curvature_speed_limit = -4.0F;
  double a_lower_for_map_related_speed_limit = -2.0F;
  double acc_coeff_for_curvature_speed_limit = 0.2F;
  double decel_coeff_for_curvature_speed_limit = 0.6F;
  double acc_coeff_for_map_related_speed_limit = 1.0F;

  vector<double> overspeed_percent_table = {-0.01, 0.0, 0.1, 0.2, 0.3};
  vector<double> decel_coeff_for_map_table = {1.0, 0.3, 0.5, 0.7, 1.0};
  vector<double> max_virtual_acc_for_map_table = {-1.0F, -0.5F, -0.8F, -1.0F, -1.5F};

  // The acceleration-maximium for the virtual deceleration
  std::vector<double> delta_v_table_for_virtual_acc_max = {0.0001F, 0.5F, 1.0F};
  std::vector<double> virtual_acc_max_table = {-1.5F, -1.5F, -2.0F};

  STLOG(D, "[SpeedLimitProcess::smoothSpeedLimits] init_v: ", init_v_, ", init_index: ", init_index_,
        ", speed_limits_.size(): ", speed_limits_.size());

  if (speed_limits_.at(init_index_).speed_limit < init_v_) {
    double v_soft_upper = init_v_;
    speed_limits_.at(init_index_).speed_limit = v_soft_upper;
    for (size_t i = init_index_ + 1; i < speed_limits_.size(); i++) {
      bool over_curvature_speed_limit = (v_soft_upper > curve_speed_limit_.at(i).speed_limit);
      double delta_v = v_soft_upper - speed_limits_.at(i).speed_limit;
      double virtual_acc_max = math::TableLookUp1D(delta_v_table_for_virtual_acc_max, virtual_acc_max_table, delta_v);
      if (speed_limits_.at(i).id == "map" || speed_limits_.at(i).id == "driving") {
        double overspeed_percent = (v_soft_upper - speed_limits_.at(i).speed_limit) / (speed_limits_.at(i).speed_limit);
        acc_coeff_for_map_related_speed_limit =
            math::TableLookUp1D(overspeed_percent_table, decel_coeff_for_map_table, overspeed_percent);
        virtual_acc_max =
            math::TableLookUp1D(overspeed_percent_table, max_virtual_acc_for_map_table, overspeed_percent);
      }
      double virtual_acc = 0.0;
      // Calculate virtual_acc:
      if (over_curvature_speed_limit) {
        double virtual_acc_for_curvature_speed_limit =
            -decel_coeff_for_curvature_speed_limit * (v_soft_upper - curve_speed_limit_.at(i).speed_limit);
        virtual_acc_for_curvature_speed_limit =
            fmax(a_lower_for_curvature_speed_limit, virtual_acc_for_curvature_speed_limit);
        double virtual_acc_for_map_related_speed_limit =
            -acc_coeff_for_map_related_speed_limit * (v_soft_upper - speed_limits_.at(i).speed_limit);
        virtual_acc_for_map_related_speed_limit =
            fmax(virtual_acc_for_map_related_speed_limit, a_lower_for_map_related_speed_limit);
        virtual_acc = fmin(virtual_acc_for_curvature_speed_limit, virtual_acc_for_map_related_speed_limit);
      } else {
        double virtual_acc_for_map_related_speed_limit =
            -acc_coeff_for_map_related_speed_limit * (v_soft_upper - speed_limits_.at(i).speed_limit);
        virtual_acc_for_map_related_speed_limit =
            fmax(virtual_acc_for_map_related_speed_limit, a_lower_for_map_related_speed_limit);
        virtual_acc = virtual_acc_for_map_related_speed_limit;
      }
      if (delta_v >= 0.0001F) {
        virtual_acc = fmax(virtual_acc, virtual_acc_max);
        virtual_acc = fmin(virtual_acc, profile_.overspeed_min_acc_limit_threshold());
      }
      // Calculate the next v_soft_upper:
      double ds = speed_limits_.at(i).s - speed_limits_.at(i - 1).s;
      v_soft_upper = sqrt(fmax(v_soft_upper * v_soft_upper + 2.0F * virtual_acc * ds, 0.01F));
      if (v_soft_upper > speed_limits_.at(i).speed_limit) {
        speed_limits_.at(i).speed_limit = v_soft_upper;
      } else {
        break;
      }
    }
  } else {
    // double v_soft_upper = init_v_;
    // speed_limits_.at(init_index_).speed_limit = v_soft_upper;
    // double acc_coeff_fac = 1.0;
    // for (size_t i = init_index_ + 1; i < speed_limits_.size(); i++) {
    //   double acc_coeff = (speed_limits_.at(i).speed_limit < curve_speed_limit_.at(i).speed_limit - 0.3F)
    //                          ? acc_coeff_for_map_related_speed_limit * acc_coeff_fac
    //                          : acc_coeff_for_curvature_speed_limit * acc_coeff_fac;
    //   double virtual_acc = -acc_coeff * (v_soft_upper - speed_limits_.at(i).speed_limit);
    //   virtual_acc = fmin(virtual_acc, 1.0);
    //   double ds = speed_limits_.at(i).s - speed_limits_.at(i - 1).s;
    //   v_soft_upper = sqrt(fmax(v_soft_upper * v_soft_upper + 2.0F * virtual_acc * ds, 0.01F));
    //   if (v_soft_upper < speed_limits_.at(i).speed_limit) {
    //     speed_limits_.at(i).speed_limit = v_soft_upper;
    //   } else {
    //     break;
    //   }
    // }
  }
  for (size_t i = 0; i < speed_limits_.size(); i++) {
    speed_limits_.at(i).speed_limit = max(speed_limits_.at(i).speed_limit, 1.0);
  }
}

void SpeedLimitProcess::outCurveSpeedLimit(const ReferenceLineInfo* reference_line_info, const Chassis* chassis,
                                           const unique_ptr<DataManager::VehicleInfo>& vehicle_info) {
  if (reference_line_info == nullptr || !reference_line_info->isValid()) {
    return;
  }
  bool out_curve_acc_limit = false;
  const double cur_s = get<0>(vehicle_info->sl_info.first);
  const double max_kappa = reference_line_info->ref_line().getMaxCurvatureInRange(
      std::fmax(cur_s - profile_.out_curve_consider_back_distance(), 0.0), cur_s);
  if (std::fabs(max_kappa) >= profile_.out_curve_consider_curve_threshold()) {
    out_curve_acc_limit = true;
  }

  // need acc limit for out curve road
  if (out_curve_acc_limit) {
    const double steer_angle = chassis->SteeringAngle();
    const double steer_threshold_for_acc = profile_.steer_acc_limit_threshold();
    if (std::fabs(steer_angle) > steer_threshold_for_acc || last_steer_curve_acc_limit_) {
      std::vector<double> rear_acc_table = {1.5, 1.0, 0.8, 0.5, 0.3, 0.0};
      std::vector<double> ego_speed_table_kmh = {1.0, 5.0, 10.0, 15.0, 18.0, 25.0};
      const double ego_speed = chassis->Speed();
      const double acc_coeff_for_steer = math::TableLookUp1D(ego_speed_table_kmh, rear_acc_table, ego_speed * MS_KMH);
      double v_soft_upper = speed_limits_.at(init_index_).speed_limit;
      for (size_t i = init_index_ + 1; i < speed_limits_.size(); i++) {
        double ds = speed_limits_.at(i).s - speed_limits_.at(i - 1).s;
        v_soft_upper = sqrt(fmax(v_soft_upper * v_soft_upper + 2.0F * acc_coeff_for_steer * ds, 0.01F));
        if (v_soft_upper < speed_limits_.at(i).speed_limit) {
          speed_limits_.at(i).speed_limit = v_soft_upper;
        }
      }
    }

    if (std::fabs(steer_angle) > steer_threshold_for_acc) {
      last_steer_curve_acc_limit_ = true;
    }
    const double steer_threshold_for_release_acc_limit = profile_.release_steer_acc_limit_threshold();
    bool need_steer_release_acc = std::fabs(steer_angle) <= steer_threshold_for_release_acc_limit;
    if (need_steer_release_acc) {
      last_steer_curve_acc_limit_ = false;
    }
  } else {
    last_steer_curve_acc_limit_ = false;
  }
}

void SpeedLimitProcess::caculateTurnFlag(const ReferenceLineInfo* reference_line_info,
                                         const unique_ptr<DataManager::VehicleInfo>& vehicle_info) {
  if (reference_line_info == nullptr || !reference_line_info->isValid()) {
    return;
  }
  const float backward_thr = profile_.turn_back_range();
  const float forward_thr = profile_.turn_forward_range();
  auto ego_s = vehicle_info->start_point.path_pt().s();
  auto start_index = reference_line_info->ref_line().getNearestReferenceIndex(ego_s - backward_thr);
  auto end_index = reference_line_info->ref_line().getNearestReferenceIndex(ego_s + forward_thr);
  auto reference_points = reference_line_info->ref_line().reference_points();
  if (start_index >= end_index) {
    return;
  }
  int min_end_index = start_index;
  float start_heading = reference_points.at(start_index).heading();
  float min_cosheading = 1.f;
  for (int i = start_index + 1; i < end_index; ++i) {
    float cos_t = cosf(reference_points.at(i).heading() - start_heading);
    if (cos_t < min_cosheading) {
      min_cosheading = cos_t;
      min_end_index = i;
    }
  }

  if (last_turn_state_ == 0) {
    // 入弯
    if (min_cosheading < -0.95f) {
      is_turn_around_ = true;
      last_turn_state_ = 2;
    } else if (min_cosheading < 0.34f) {
      last_turn_state_ = 1;
    } else {
    }
  } else if (last_turn_state_ == 1) {
    if (min_cosheading < -0.95f) {  // normal turn to turn around
      is_turn_around_ = true;
      last_turn_state_ = 2;
    }
    if (min_cosheading > 0.98f) {
      last_turn_state_ = 0;
      is_turn_around_ = false;
    }
  } else {
    // 出弯
    if (min_cosheading > 0.98f) {
      last_turn_state_ = 0;
      is_turn_around_ = false;
    }
  }
}

}  // namespace gpal::pnc::planning