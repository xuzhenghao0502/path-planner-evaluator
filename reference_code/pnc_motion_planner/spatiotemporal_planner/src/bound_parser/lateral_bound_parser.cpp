/**
 * @file lateral_bound_parser.cpp
 * @brief 横向边界解析器实现
 */
#include "bound_parser/lateral_bound_parser.h"

#include "spatiotemporal_abstract_module.h"
namespace gpal::pnc::planning {

bool LateralBoundParser::init() {
  config_manager_ = Singleton<ConfigManager>::get_instance();
  CHECK_NOTNULL(config_manager_);
  return true;  // TODO: Implement actual initialization logic
}

void LateralBoundParser::reset() {}

std::string LateralBoundParser::id() const {
  return "LateralBoundParser";
}

bool LateralBoundParser::run(SpatiotemporalPlannerDataManager& data_manager) {
  // // 1. 更新边界解析器的配置和车辆信息
  // if (!update(data_manager)) {
  //   STLOG(E, "[LateralBoundParser::run] update failed");
  //   return false;
  // }

  // // 2. 初始化边界解析，基于决策边界和道路边界
  // if (!initBoundary(data_manager)) {
  //   STLOG(E, "[LateralBoundParser::run] initBoundary failed");
  //   return false;
  // }

  // // 3. 静态边界: 基于决策nudge
  // if (!calcBoundaryFromStaticObjects(data_manager)) {
  //   STLOG(E, "[LateralBoundParser::run] calcBoundaryFromStaticObjects failed");
  //   return false;
  // }

  // // 4. 基于场景，更新横向软边界soft_boundary.
  // if (!refineBoundaryUnderSpecialScene(data_manager)) {
  //   STLOG(E, "[LateralBoundParser::run] refineBoundaryUnderSpecialScene failed");
  //   return false;
  // }

  return true;
}

bool LateralBoundParser::update(SpatiotemporalPlannerDataManager& data_manager) {
  const auto& input_data = data_manager.inputData();
  const auto& config_info = data_manager.configInfo();
  const auto& vehicle_param = config_manager_->vehicle_config().vehicle_param();
  vehicle_info_ = input_data.vehicle_info.get();
  profile_ = config_info.lateral_path_bound_parser_profile.get();
  behavior_ = input_data.decision_result->getCurrFsmState();
  const auto frenet_s = input_data.vehicle_info->sl_info.first[0];
  double start_s = std::max(0.0, frenet_s - vehicle_param.rear_edge_to_ego());
  CHECK_NOTNULL(profile_);
  CHECK_NOTNULL(vehicle_info_);
  STLOG(D, "[LateralBoundParser::update] profile_->max_speed_limit(): ", profile_->max_speed_limit());
  STLOG(D, "[LateralBoundParser::update] profile_->max_range(): ", profile_->max_range());
  STLOG(D, "[LateralBoundParser::update] profile_->resolution(): ", profile_->resolution());

  object_type_lateral_distance_map_.clear();
  for (const auto& type_dist_pair : profile_->type_lateral_dist_pairs().type_lateral_dist_pair()) {
    object_type_lateral_distance_map_.emplace(type_dist_pair.obs_type(), type_dist_pair.obs_info());
  }

  l_offset_behavior_valid_ = false;
  if (behavior_ == FsmState::LEFT_HOLD || behavior_ == FsmState::RIGHT_HOLD || behavior_ == FsmState::LEFT_ATTEMPT
      || behavior_ == FsmState::RIGHT_ATTEMPT) {
    if (input_data.decision_result->getRefTrajInfo() != nullptr
        && !input_data.decision_result->getRefTrajInfo()->traj_points.empty()) {
      l_offset_behavior_valid_ = true;
    }
  }

  //寻找最近的跨线属性
  crossing_line_behavior_ = 0;  // 0 不跨线， 1 左侧跨线， 2 右侧跨线
  int temp_crossing_line_behavior = 0;
  float temp_crossing_line_behavior_start_s = 1e9;
  for (const auto& attri : input_data.target_ref_line_info->ref_line().getLaneMarkingCrossAttributeRanges()) {
    if (start_s > attri.after_cross_ref_end_s) {
      continue;
    }
    if (attri.after_cross_ref_start_s < temp_crossing_line_behavior_start_s) {
      temp_crossing_line_behavior = attri.is_left_cross ? 1 : 2;
      temp_crossing_line_behavior_start_s = attri.after_cross_ref_start_s;
    }
  }
  crossing_line_behavior_ = temp_crossing_line_behavior;
  return true;
}

bool LateralBoundParser::initBoundary(SpatiotemporalPlannerDataManager& data_manager) {
  const auto& vehicle_param = config_manager_->vehicle_config().vehicle_param();
  const auto& reference_line = data_manager.inputData().target_ref_line_info->ref_line();
  const auto& vehicle_info = data_manager.inputData().vehicle_info;
  const auto& decision_result = data_manager.inputData().decision_result;
  auto& boundary = data_manager.mutableBoundaryInfo().trajectory_boundary;
  auto& soft_boundary = boundary->mutableSoftLateralBound();
  auto& hard_boundary = boundary->mutableHardLateralBound();
  auto& hard_boundary_data = hard_boundary.mutableOriginData();
  auto& soft_boundary_data = soft_boundary.mutableOriginData();

  // 获取边界的基本信息
  // double start_s = boundary->start_s();
  const auto frenet_s = vehicle_info->sl_info.first[0];
  double start_s = std::max(0.0, frenet_s - vehicle_param.rear_edge_to_ego());
  double end_s = std::min(reference_line.length(), vehicle_info->frenet_end_s + vehicle_param.front_edge_to_ego());
  double delta_s = boundary->delta_s();
  size_t num_points = boundary->size();

  // 获取决策边界信息
  const auto& decision_boundaries = decision_result->getLateralBoundaryDecision();
  // 为每个采样点创建横向边界
  for (int i = 0; i < boundary->size(); ++i) {
    double curr_s = boundary->start_s() + i * boundary->delta_s();
    if (std::isinf(curr_s) || std::isnan(curr_s)) {
      STLOG(E, "[LateralBoundParser::initBoundary] invalid curr_s: ", curr_s);
      return false;
    }
    // 获取道路边界
    double road_left_bound = 0.0, road_right_bound = 0.0;
    reference_line.getRoadBound(curr_s, &road_left_bound, &road_right_bound);

    // 获取车道边界
    double lane_left_bound = 0.0, lane_right_bound = 0.0;
    reference_line.getLaneBound(curr_s, &lane_left_bound, &lane_right_bound);

    // 默认使用车道边界
    double hard_left_bound = road_left_bound, hard_right_bound = road_right_bound;
    double soft_left_bound = lane_left_bound, soft_right_bound = lane_right_bound;
    std::vector<double> speed_vec;
    std::vector<double> decision_bound_buffer_vec;
    for (const auto& ele : profile_->speed_decision_bound_buffer_map().elements()) {
      speed_vec.emplace_back(ele.speed());
      decision_bound_buffer_vec.emplace_back(ele.buffer());
    }
    double decision_hard_buffer = math::TableLookUp1D(speed_vec, decision_bound_buffer_vec,
                                                      data_manager.inputData().vehicle_info->sl_info.first[1]);
    double decision_soft_buffer = profile_->barrier_bound_extend_buffer();

    // 如果有决策边界，使用决策边界
    double search_radius = profile_->search_radius();  // 搜索半径
    double search_step = profile_->search_step();    // 搜索分辨率
    for (const auto& boundary : decision_boundaries) {
      if (boundary.type == LateralBoundaryType::LAT_HARD) {
        int search_count = 0;
        int max_search_count = static_cast<int>(2 * search_radius / search_step) + 1;
        double local_min_left = std::numeric_limits<double>::max();
        double local_max_right = std::numeric_limits<double>::lowest();
        BoundaryPointTypeInfo left_boundary_type = BoundaryPointTypeInfo::INVALID;
        BoundaryPointTypeInfo right_boundary_type = BoundaryPointTypeInfo::INVALID;
        for (double s = curr_s - search_radius; s <= curr_s + search_radius + 1e-6; s += search_step) {
          ++search_count;
          if (s < start_s || s > end_s) {
            continue;
          }
          auto [left, right, left_type, right_type] = Decision::LateralBoundConsInterpolate(s, boundary.points);
          if (left < local_min_left) {
            local_min_left = left;
            left_boundary_type = left_type;
          }
          if (right > local_max_right) {
            local_max_right = right;
            right_boundary_type = right_type;
          }
          if (search_count > max_search_count) {
            STLOG(W, "[LateralBoundParser::initBoundary] too many search count at curr_s: ", curr_s);
            break;
          }
        }
        auto decision_left_hard_buffer =
            right_boundary_type == BoundaryPointTypeInfo::PHYSICALLY_IMPASSABLE ? decision_hard_buffer : 0.0;
        auto decision_right_hard_buffer =
            left_boundary_type == BoundaryPointTypeInfo::PHYSICALLY_IMPASSABLE ? decision_hard_buffer : 0.0;
        hard_left_bound = std::min(hard_left_bound, local_min_left - decision_left_hard_buffer);
        hard_right_bound = std::max(hard_right_bound, local_max_right + decision_right_hard_buffer);
      }

      if (boundary.type == LateralBoundaryType::LAT_SOFT) {
        auto&& [left, right, l_type, r_type] = Decision::LateralBoundConsInterpolate(curr_s, boundary.points);
        bool left_is_physical = false;              // 是否为物理不可逾越边界
        bool right_is_physical = false;             // 是否为物理不可逾越边界
        bool right_bound_release_condition = false;  // 右边界释放条件
        bool left_bound_release_condition = false;   // 左边界释放条件

        left_is_physical =
            l_type == BoundaryPointTypeInfo::PHYSICALLY_IMPASSABLE || l_type == BoundaryPointTypeInfo::SOLID_LANE_LINE;
        right_is_physical =
            r_type == BoundaryPointTypeInfo::PHYSICALLY_IMPASSABLE || r_type == BoundaryPointTypeInfo::SOLID_LANE_LINE;
        right_bound_release_condition =
            behavior_ == FsmState::LEFT_CHANGE || behavior_ == FsmState::RIGHT_RETURN
            || (l_offset_behavior_valid_ && (behavior_ == FsmState::LEFT_HOLD || behavior_ == FsmState::RIGHT_ATTEMPT));

        left_bound_release_condition =
            behavior_ == FsmState::RIGHT_CHANGE || behavior_ == FsmState::LEFT_RETURN
            || (l_offset_behavior_valid_ && (behavior_ == FsmState::RIGHT_HOLD || behavior_ == FsmState::LEFT_ATTEMPT));

        if (right_bound_release_condition && !right_is_physical) {
          right = std::min(right + decision_soft_buffer,
                           vehicle_info_->curr_right_bound - profile_->max_lane_width_right());
        } else if (left_bound_release_condition && !left_is_physical) {
          left =
              std::max(left - decision_soft_buffer, vehicle_info_->curr_left_bound + profile_->max_lane_width_left());
        } else if (crossing_line_behavior_ == 1) {
          right = std::min(right + decision_soft_buffer,
                           vehicle_info_->curr_right_bound - profile_->max_lane_width_right());
        } else if (crossing_line_behavior_ == 2) {
          left =
              std::max(left - decision_soft_buffer, vehicle_info_->curr_left_bound + profile_->max_lane_width_left());
        }

        soft_left_bound = left;
        soft_right_bound = right;
      }
    }
    // 软边界主要是车道边界，避免软边界入侵车道中心线
    soft_left_bound = std::max(soft_left_bound, 0.5 * vehicle_param.width() + decision_soft_buffer);
    soft_right_bound = std::min(soft_right_bound, -0.5 * vehicle_param.width() - decision_soft_buffer);
    auto&& [hard_lower, hard_upper] = hard_boundary_data[i].mutableBounds();
    hard_lower = std::max(hard_lower, hard_right_bound);
    hard_upper = std::min(hard_upper, hard_left_bound);
    auto&& [soft_lower, soft_upper] = soft_boundary_data[i].mutableBounds();
    soft_lower = std::max(soft_lower, soft_right_bound);
    soft_upper = std::min(soft_upper, soft_left_bound);
  }

  if (!bound_parser_.reset(reference_line, start_s, end_s, profile_->resolution())) {
    return false;
  }

  return true;
}

bool LateralBoundParser::calcBoundaryFromStaticObjects(SpatiotemporalPlannerDataManager& data_manager) {
  const auto& vehicle_param = config_manager_->vehicle_config().vehicle_param();
  auto& boundary = data_manager.mutableBoundaryInfo().trajectory_boundary;
  auto& soft_boundary = boundary->mutableSoftLateralBound();
  auto& hard_boundary = boundary->mutableHardLateralBound();
  auto& static_objects = data_manager.objectsInfo().static_objects;
  const auto& decision_result = data_manager.inputData().decision_result;

  auto& hard_boundary_data = hard_boundary.mutableOriginData();
  auto& soft_boundary_data = soft_boundary.mutableOriginData();

  int hard_boundary_blocked_idx = -1;
  int soft_boundary_blocked_idx = -1;

  // 1.基于od速度及type信息，为决策静态nudge的od增加横向软、硬边界的buffer.
  std::vector<std::tuple<double, double, double>> hard_buffer, soft_buffer;
  std::vector<double> speed_vec;
  std::vector<double> speed_start_longit_bound_buffer_vec;
  std::vector<double> speed_end_longit_bound_buffer_vec;
  for (const auto& ele : profile_->speed_longit_bound_buffer_map().elements()) {
    speed_vec.emplace_back(ele.speed());
    speed_start_longit_bound_buffer_vec.emplace_back(ele.start_buffer());
    speed_end_longit_bound_buffer_vec.emplace_back(ele.end_buffer());
  }
  double start_longit_hard_buffer = profile_->barrier_start_long_buffer_for_obs();
  double end_longit_hard_buffer = profile_->barrier_end_long_buffer_for_obs();
  if (!speed_start_longit_bound_buffer_vec.empty() && !speed_vec.empty()) {
    start_longit_hard_buffer = math::TableLookUp1D(speed_vec, speed_start_longit_bound_buffer_vec,
                                                   data_manager.inputData().vehicle_info->sl_info.first[1]);
  }
  if (!speed_end_longit_bound_buffer_vec.empty() && !speed_vec.empty()) {
    end_longit_hard_buffer = math::TableLookUp1D(speed_vec, speed_end_longit_bound_buffer_vec,
                                                 data_manager.inputData().vehicle_info->sl_info.first[1]);
  }

  for (const auto& obstacle : static_objects) {
    const auto lateral_safe_buffer_pair =
        getObjectsLaterSafeBuffer(obstacle, data_manager.inputData().vehicle_info->sl_info.first[1]);
    hard_buffer.emplace_back(start_longit_hard_buffer, end_longit_hard_buffer, lateral_safe_buffer_pair.first);
    soft_buffer.emplace_back(profile_->soft_start_long_buffer_for_obs(), profile_->soft_end_long_buffer_for_obs(),
                             lateral_safe_buffer_pair.second);
  }

  // 2.基于决策nudge od的横向硬边界hard_boundary更新(车尾起始，已去除半车宽).
  //   1）基于od及buffer，在bound_parser_中生成沿参考线的ObstacleBoundaryInfo，包括:
  //   bound，right_bound_obstacles，left_bound_obstacles.
  bool hard_bound_info = bound_parser_.getBoundaryFromStaticObjects(static_objects, hard_buffer, hard_boundary_data);

  bool soft_bound_info = bound_parser_.getBoundaryFromStaticObjects(static_objects, soft_buffer, soft_boundary_data);

  //   3）更新横向边界
  const double half_width = 0.5 * vehicle_param.width();
  // 闸机边界ignore
  double ignore_start_s = 0.0, ignore_end_s = 0.0;
  bool ignore_boundary =
      ignoreBoundary(decision_result->getLongitudinalBoundaryDecision(), ignore_start_s, ignore_end_s);

  for (int i = 0; i < boundary->size(); ++i) {
    if (hard_boundary_blocked_idx < 0 && !updateBoundary(half_width, hard_boundary_data[i])) {
      STLOG(W, "[LateralBoundParser::calcBoundaryFromStaticObjects] hard boundary blocked at index ", i,
            ", s = ", hard_boundary_data[i].s());
      hard_boundary_blocked_idx = i;
      boundary->setBoundaryBlockS(hard_boundary_data[i].s() - vehicle_param.front_edge_to_ego());
      trimBoundary(hard_boundary_blocked_idx, half_width, hard_boundary_data);
    }
    if (soft_boundary_blocked_idx < 0 && !updateBoundary(half_width, soft_boundary_data[i])) {
      STLOG(W, "[LateralBoundParser::calcBoundaryFromStaticObjects] soft boundary blocked at index ", i,
            ", s = ", soft_boundary_data[i].s());
    }

    if (ignore_boundary) {
      int start_index = hard_boundary.getIndex(ignore_start_s);
      int end_index = hard_boundary.getIndex(ignore_end_s);
      trimBoundary(start_index, end_index, half_width, hard_boundary_data);
      trimBoundary(start_index, end_index, half_width, soft_boundary_data);
    }

    if (hard_boundary_blocked_idx > 0 && soft_boundary_blocked_idx > 0) {
      break;  // 如果已经找到阻塞的边界，则提前退出
    }
  }
  return true;
}

bool LateralBoundParser::refineBoundaryUnderSpecialScene(SpatiotemporalPlannerDataManager& data_manager) {
  // TODO 实现基于特殊场景的边界优化
  // 这里可能包括：
  // 1. 交叉口场景的边界调整
  // 2. 狭窄道路的边界优化
  // 3. 其他特殊场景处理
  return true;
}

std::pair<double, double> LateralBoundParser::getObjectsLaterSafeBuffer(
    const std::shared_ptr<Decision::DecisionObject>& obs, const double& adc_frenet_s_speed) {
  std::pair<double, double> safe_buffer_pair(0.0, 0.0);
  safe_buffer_pair.first = math::lerp(profile_->default_min_barrier_lateral_buffer_for_obs(), 0.0,
                                      profile_->default_max_barrier_lateral_buffer_for_obs(),
                                      profile_->max_speed_limit(), adc_frenet_s_speed * MS_KMH);
  safe_buffer_pair.first =
      std::min<double>(std::max<double>(safe_buffer_pair.first, profile_->default_min_barrier_lateral_buffer_for_obs()),
                       profile_->default_max_barrier_lateral_buffer_for_obs());

  safe_buffer_pair.second = math::lerp(profile_->default_min_soft_lateral_buffer_for_obs(), 0.0,
                                       profile_->default_max_soft_lateral_buffer_for_obs(), profile_->max_speed_limit(),
                                       adc_frenet_s_speed * MS_KMH);
  safe_buffer_pair.second =
      std::min<double>(std::max<double>(safe_buffer_pair.second, profile_->default_min_soft_lateral_buffer_for_obs()),
                       profile_->default_max_soft_lateral_buffer_for_obs());

  safe_buffer_pair = getObsLaterSafeBufferFromType(obs->type, safe_buffer_pair.first, safe_buffer_pair.second);

  return safe_buffer_pair;
}

std::pair<double, double> LateralBoundParser::getObsLaterSafeBufferFromType(
    const Decision::ObjectType& obs_type, const double& default_barrier_lateral_buffer,
    const double& default_soft_lateral_buffer) {
  std::pair<double, double> safe_buffer_pair(default_barrier_lateral_buffer, default_soft_lateral_buffer);
  if (object_type_lateral_distance_map_.empty()) {
    return safe_buffer_pair;
  }

  // PLOGI << "obs_type = " << static_cast<int>(obs_type);
  const auto& iter = object_type_lateral_distance_map_.find(static_cast<int>(obs_type));
  if (iter != object_type_lateral_distance_map_.end()) {
    safe_buffer_pair.first = iter->second.barrier_lateral_safe_distance();  // barrier
    safe_buffer_pair.second = iter->second.soft_lateral_safe_distance();    // soft
    // PLOGI << "find obs type: barrier_safe_dis = " << safe_buffer_pair.first
    //           << "  soft_safe_dis = " << safe_buffer_pair.second;
    return safe_buffer_pair;
  } else {
    // PLOGI << "failed to find obs type: barrier_safe_dis = " << safe_buffer_pair.first
    //           << "  soft_safe_dis = " << safe_buffer_pair.second;
    return safe_buffer_pair;
  }
}

bool LateralBoundParser::updateBoundary(const double& lat_buffer, Boundary& boundary) {
  const auto& vehicle_param = config_manager_->vehicle_config().vehicle_param();
  auto&& [lower, upper] = boundary.mutableBounds();
  const double& s = boundary.s();
  lower += lat_buffer;
  upper -= lat_buffer;
  if (s > vehicle_info_->sl_info.first[0] - vehicle_param.rear_edge_to_ego() - (1e-2)
      && s < vehicle_info_->sl_info.first[0] + vehicle_param.front_edge_to_ego() + (1e-2)) {
    lower = std::fmin(vehicle_info_->curr_right_bound, lower);
    upper = std::fmax(vehicle_info_->curr_left_bound, upper);
  }
  // 当边界交叉，小于自车s的边界重置为车道最大宽度，自车s以后的边界标记blocked
  if (lower > upper && s <= vehicle_info_->sl_info.first[0] - vehicle_param.rear_edge_to_ego() - (1e-2)) {
    lower = -profile_->max_road_width_right() + lat_buffer;
    upper = profile_->max_road_width_left() - lat_buffer;
  } else if (lower > upper) {
    return false;
  }

  return true;
}

void LateralBoundParser::trimBoundary(const int blocked_idx, const double& lat_buffer,
                                      std::vector<Boundary>& boundary) {
  if (blocked_idx < 0 || blocked_idx >= boundary.size()) {
    return;
  }
  for (size_t i = blocked_idx; i < boundary.size(); ++i) {
    auto&& [lower, upper] = boundary[i].mutableBounds();
    lower = -profile_->max_road_width_right() + lat_buffer;
    upper = profile_->max_road_width_left() - lat_buffer;
  }
}

void LateralBoundParser::trimBoundary(const int start_idx, const int end_idx, const double& lat_buffer,
                                      std::vector<Boundary>& boundary) {
  if (start_idx < 0 || start_idx >= boundary.size() || end_idx < 0 || end_idx >= boundary.size()) {
    return;
  }
  for (size_t i = start_idx; i < end_idx; ++i) {
    auto&& [lower, upper] = boundary[i].mutableBounds();
    lower = -profile_->max_road_width_right() + lat_buffer;
    upper = profile_->max_road_width_left() - lat_buffer;
  }
}

bool LateralBoundParser::ignoreBoundary(const LongitudinalBoundDecision& longitudinal_bound_decision,
                                        double& ignore_start_s, double& ignore_end_s) {
  const auto& vehicle_param = config_manager_->vehicle_config().vehicle_param();
  for (const auto& bound : longitudinal_bound_decision) {
    if (bound.type == WallType::LONG_RSA_WALL) {
      ignore_start_s = bound.s + vehicle_param.front_edge_to_ego() - profile_->bound_ignore_rear_buffer();
      ignore_end_s = bound.s + vehicle_param.front_edge_to_ego() + profile_->bound_ignore_front_buffer();
      return true;
    }
  }
  return false;
}

REGIST_MODULE(LateralBoundParser);
}  // namespace gpal::pnc::planning
