#include "openspace_path_planner/core/optimizer/openspace_ocp_bound_parser.h"

namespace gpal::pnc::planning {

namespace {

bool isInRange(const std::vector<std::pair<double, double>>& ranges, const double s) {
  if (ranges.empty()) {
    return false;
  }
  auto iter = std::find_if(ranges.begin(), ranges.end(), [s](const std::pair<double, double>& range) {
    return range.first <= s && s < range.second;
  });
  return iter != ranges.end();
}

bool isInRange(const std::vector<std::tuple<std::string, float, float, bool>> special_tags_range_, std::string tag,
               const double s) {
  if (special_tags_range_.empty()) {
    return false;
  }
  for (auto& [tag_, start_s, end_s, ego_inrange] : special_tags_range_) {
    if (tag_ == tag && start_s <= s && end_s > s) {
      return true;
    }
  }
  return false;
}

bool isInIgnoreRange(const std::vector<IgnoreRangeInfo>& ranges, const double s) {
  if (ranges.empty()) {
    return false;
  }
  auto iter = std::find_if(ranges.begin(), ranges.end(),
                           [s](const IgnoreRangeInfo& range) { return range.start_s <= s && s < range.end_s; });
  return iter != ranges.end();
}

}  // namespace

bool OpenspaceOcpBoundParser::init() {
  OPENSPACE_LOG(D, "[OpenspaceOcpBoundParser] init!");
  reset();
  config_manager_ = Singleton<ConfigManager>::get_instance();
  CHECK_NOTNULL(config_manager_);
  if (init_) {
    return true;
  }
  config_ = config_manager_->getConfig<PathBoundPointsConfig>("OpenspacePathBoundPointsConfig");
  auto path_bound_parser_config = config_manager_->getConfig<PathBoundParserConfig>("OpenspacePathBoundParserConfig");
  bound_parser_ = std::make_unique<PathBoundParser>(path_bound_parser_config);
  freespace_barrier_bound_filter_ = std::make_unique<PathBoundFilter>();
  freespace_soft_bound_filter_ = std::make_unique<PathBoundFilter>();
  static_obstacle_barrier_bound_filter_ = std::make_unique<PathBoundFilter>();
  static_obstacle_soft_bound_filter_ = std::make_unique<PathBoundFilter>();

  // genreate type-dist pairs, using unordered_map
  for (const auto& type_dist_pair : config_.type_lateral_dist_pairs().type_lateral_dist_pair()) {
    obs_type_lateral_distance_map_.emplace(type_dist_pair.obs_type(), type_dist_pair.obs_info());
  }
  init_ = true;
  return init_;
}

void OpenspaceOcpBoundParser::preProcess(const ReferenceLine& reference_line, const TrajectoryPt& init_point,
                                         const Eigen::Matrix4d& tf_ego_map) {
  OPENSPACE_LOG(D, "[OpenspaceOcpBoundParser] preProcess!");
  model_name_ = "";
  auto& vehicle_param = config_manager_->vehicle_config().vehicle_param();
  tf_ego_map_ = tf_ego_map;

  // adc_width_ = vehicle_param.width();
  adc_width_ = vehicle_param.width_without_rearview_mirror();
  adc_wheel_width_ = vehicle_param.width_wheel();

  adc_front_length_ = vehicle_param.front_edge_to_ego();
  adc_rear_length_ = vehicle_param.rear_edge_to_ego();

  const auto& target_ref = reference_line;
  planning_start_point_ = init_point;
  auto adc_sl_info = target_ref.toFrenetFrame(planning_start_point_);
  adc_frenet_s_ = std::fmin(adc_sl_info.first[0], target_ref.length());
  adc_frenet_l_ = adc_sl_info.second[0];
  adc_frenet_sd_ = std::max(kSpeedEpsilon, adc_sl_info.first[1]);
  adc_frenet_end_s_ = target_ref.length();

  path_blocked_idx_ = -1;
}

void OpenspaceOcpBoundParser::reset() {
  debug_status_.clear();
  debug_info_.clear();
  resetFilter();
  debug_info_ = "openspace info: ";
}

void OpenspaceOcpBoundParser::resetFilter() {
  if (freespace_barrier_bound_filter_) {
    freespace_barrier_bound_filter_->reset();
  }
  if (freespace_soft_bound_filter_) {
    freespace_soft_bound_filter_->reset();
  }
  if (static_obstacle_barrier_bound_filter_) {
    static_obstacle_barrier_bound_filter_->reset();
  }
  if (static_obstacle_soft_bound_filter_) {
    static_obstacle_soft_bound_filter_->reset();
  }
}

Status OpenspaceOcpBoundParser::generateBounds(const Freespace& freespace,
                                               const std::vector<ObstaclesLinesegment>& obstacles_linesegments,
                                               const ReferenceLine& reference_line, const bool consider_bound,
                                               PathBoundary& boundary, const PathPt::Direction direction) {
  // 1. Initialize the path boundaries with freespace.
  if (!initPathBoundary(reference_line, boundary, direction)) {
    const std::string msg = "[OpenspaceOcpBoundParser] Failed to initialize path boundaries.";
    OPENSPACE_LOG(E, msg);
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (consider_bound) {
    // 2. Fine-tune the barrier boundary based on freespace
    if (!getBoundaryFromFreespace(freespace, boundary.mutable_barrier_boundary(), boundary.mutable_soft_boundary(),
                                  boundary.mutable_blocking_freespace_info())) {
      const std::string msg =
          "[OpenspaceOcpBoundParser]Failed to decide fine tune the boundaries after "
          "taking into consideration freespace.";
      OPENSPACE_LOG(D, msg);
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }

    // 3. Fine-tune the barrier boundary based on static obstacles and nearby dynamic obstacles
    if (!getBoundaryFromStaticObstacles(reference_line, obstacles_linesegments, boundary.mutable_barrier_boundary(),
                                        boundary.mutable_soft_boundary(), boundary.mutable_blocking_obstacle_id())) {
      const std::string msg =
          "[OpenspaceOcpBoundParser] Failed to decide fine tune the boundaries after "
          "taking into consideration all static obstacles.";
      OPENSPACE_LOG(E, msg);
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
  }

  OPENSPACE_LOG(D, "[OpenspaceOcpBoundParser][generateBounds]: start_s = ", boundary.start_s(),
                "  end_s = ", boundary.end_s(), "  size = ", boundary.size(), "  delta_s = ", boundary.delta_s(),
                "  length = ", boundary.length());
  // for (int i = 0; i < boundary.barrier_boundary().size(); ++i) {
  //   OPENSPACE_LOG(D, "i = ", i);
  //   OPENSPACE_LOG(D, "   barrier: s = ", std::get<0>(boundary.barrier_boundary().at(i)),
  //                 "   l_min = ", std::get<1>(boundary.barrier_boundary().at(i)),
  //                 "   l_max = ", std::get<2>(boundary.barrier_boundary().at(i)));
  //   OPENSPACE_LOG(D, "   soft: s = ", std::get<0>(boundary.soft_boundary().at(i)),
  //                 "   l_min = ", std::get<1>(boundary.soft_boundary().at(i)),
  //                 "   l_max = ", std::get<2>(boundary.soft_boundary().at(i)));
  // }
  // TODO 4. Fine-tune the soft boundary considering dynamic obstacles

  // TODO 5. Lane keep only. Fine-tune the soft boundary considering risky obstacles.

  OPENSPACE_LOG(D, "[OpenspaceOcpBoundParser] Completed generating path boundaries ");
  return Status::OK();
}

bool OpenspaceOcpBoundParser::initPathBoundary(const ReferenceLine& reference_line, PathBoundary& boundary,
                                               const PathPt::Direction direction) {
  if (reference_line.reference_points().empty()) {
    return false;
  }
  OPENSPACE_LOG(D, "[OpenspaceOcpBoundParser][initPathBoundary] reference_line length = ", reference_line.length());
  double start_s = reference_line.reference_points().front().local_s();
  double delta_s = reference_line.length() / std::fmax((reference_line.reference_points().size() - 1), 1);
  size_t size = reference_line.reference_points().size();
  direction_ = direction;
  OPENSPACE_LOG(D, "[OpenspaceOcpBoundParser][initPathBoundary] start_s = ", start_s, " delta_s = ", delta_s,
                " size = ", size);
  boundary = PathBoundary(start_s, delta_s, size);
  boundary.set_label(model_name_);
  max_allowed_bounds_.clear();
  for (const auto& rp : reference_line.reference_points()) {
    max_allowed_bounds_.emplace_back(rp.local_s(), -config_.max_road_width_right(), config_.max_road_width_left());
  }

  double bound_start_s = std::max(0.0, adc_frenet_s_ - adc_rear_length_);
  double bound_end_s = std::min(reference_line.length(), adc_frenet_end_s_ + adc_front_length_);
  OPENSPACE_LOG(D, "[OpenspaceOcpBoundParser][initPathBoundary]: bound_start_s = ", bound_start_s,
                " bound_end_s = ", bound_end_s);
  OPENSPACE_LOG(D, "[OpenspaceOcpBoundParser][initPathBoundary]: reference_line_length = ", reference_line.length(),
                " adc_frenet_end_s_ = ", adc_frenet_end_s_, " adc_front_length_ = ", adc_front_length_);
  if (!bound_parser_->init(reference_line, tf_ego_map_, bound_start_s, bound_end_s)) {
    return false;
  }
  *boundary.mutable_barrier_boundary() = max_allowed_bounds_;
  *boundary.mutable_soft_boundary() = max_allowed_bounds_;
  return true;
}

bool OpenspaceOcpBoundParser::getBoundaryFromFreespace(
    const Freespace& freespace, std::vector<std::tuple<double, double, double>>* const barrier_boundary,
    std::vector<std::tuple<double, double, double>>* const soft_boundary,
    std::pair<bool, double>* block_freespace_info) {
  OPENSPACE_LOG(D, "[getBoundaryFromFreespace] getBoundaryFromFreespace start!");
  int path_blocked_idx = -1;
  std::vector<double> sample_offsets;
  for (auto& sample_pt : bound_parser_->sample_points()) {
    double offset = 0.0;
    auto [decision_bound, has_decision_bound] = decision_bounds_.interpolate(sample_pt.local_s());
    if (has_decision_bound) {
      auto& [s, lower, upper] = decision_bound;
      offset = 0.5 * (lower + upper);
    }
    sample_offsets.emplace_back(offset);
  }
  const auto barrier_soft_buffer_pair =
      getFsLaterSafeBuffer(config_.enable_lateral_buffer_use_obs_type(), config_.enable_lateral_buffer_use_obs_speed());
  auto raw_bound_info =
      bound_parser_->getBoundInfoFromFreespace(freespace, sample_offsets, barrier_soft_buffer_pair.first);
  auto barrier_bound_info = bound_parser_->getFreespaceBoundInfoWithLongiBuffer(
      raw_bound_info, config_.barrier_start_long_buffer_for_fs(), config_.barrier_end_long_buffer_for_fs());

  freespace_barrier_bound_filter_->update(barrier_bound_info.bound);
  for (int i = 0; i < barrier_boundary->size(); i++) {
    auto& [s, lower, upper] = barrier_boundary->at(i);
    auto [freespace_bound, has_bound] = freespace_barrier_bound_filter_->bound().interpolate(s);
    auto& [fbs, fs_lower, fs_upper] = freespace_bound;
    // extend freespace bound when out of map to avoid undetected hard boundary
    if (fbs < s) {
      break;
    } else {
      double w = adc_width_;
      if (isInRange(gate_ranges_, s)) {
        w = adc_wheel_width_;
      }
      double fs_buffer = barrier_soft_buffer_pair.first;
      if (!updateBoundary(0.5 * w + fs_buffer, fs_lower, fs_upper, barrier_boundary->at(i))) {
        path_blocked_idx = i;
        auto block_s = fbs - adc_frenet_s_;  // the distance to fs
        debug_status_.push_back(PathData::DebugStatusType::BLOCK_FS_BOUND);
        OPENSPACE_LOG(D, "[getBoundaryFromFreespace] blocking freespace s = ", block_s, "detected");
        break;
      }
    }
  }
  // trimPathBounds(path_blocked_idx, barrier_boundary);
  // trimPathBounds(path_blocked_idx, soft_boundary);
  // TODO extend fs with longitudinal buffer
  auto soft_bound_info = bound_parser_->getFreespaceBoundInfoWithLongiBuffer(
      raw_bound_info, config_.soft_start_long_buffer_for_fs(), config_.soft_end_long_buffer_for_fs());

  freespace_soft_bound_filter_->update(soft_bound_info.bound);
  for (int i = 0; i < soft_boundary->size(); i++) {
    auto& [s, lower, upper] = soft_boundary->at(i);
    auto [freespace_bound, has_bound] = freespace_soft_bound_filter_->bound().interpolate(s);
    auto& [fbs, fs_lower, fs_upper] = freespace_bound;
    // extend freespace bound when out of map to avoid undetected hard boundary
    if (fbs < s) {
      break;
    } else {
      double w = adc_width_;
      if (isInRange(gate_ranges_, s)) {
        w = adc_wheel_width_;
      }
      updateBoundary(0.5 * w + config_.soft_lateral_buffer_coefs_for_fs() * barrier_soft_buffer_pair.first, fs_lower,
                     fs_upper, soft_boundary->at(i), false);
    }
  }

  return true;
}

bool OpenspaceOcpBoundParser::getBoundaryFromStaticObstacles(
    const ReferenceLine& reference_line, const std::vector<ObstaclesLinesegment>& obstacles_linesegments,
    std::vector<std::tuple<double, double, double>>* const barrier_boundary,
    std::vector<std::tuple<double, double, double>>* const soft_boundary, std::string* blocking_obstacle_id) {
  std::vector<PolylineInput> parsed_polylines = generatePolylines(obstacles_linesegments, reference_line);
  OPENSPACE_LOG(D, "[getBoundaryFromStaticObstacles] select parsed_polylines size = ", parsed_polylines.size());
  std::vector<std::tuple<double, double, double>> barrier_buffer, soft_buffer;
  for (const auto& polyline : parsed_polylines) {
    // 注意：此处需要根据 polyline 的类型（如果需要）来确定不同的缓冲区。
    // std::get<0>(obstacles_linesegments[i]) 可以获取类型。
    // 为简化，这里暂时使用统一的默认值。
    double barrier_lat_buffer = config_.default_min_barrier_lateral_buffer_for_obs();
    double soft_lat_buffer = config_.default_min_soft_lateral_buffer_for_obs();

    barrier_buffer.emplace_back(config_.barrier_start_long_buffer_for_obs(), config_.barrier_end_long_buffer_for_obs(),
                                barrier_lat_buffer);
    soft_buffer.emplace_back(config_.soft_start_long_buffer_for_obs(), config_.soft_end_long_buffer_for_obs(),
                             soft_lat_buffer);
  }

  OPENSPACE_LOG(D, "[getBoundaryFromStaticObstacles] raw barrier boundary size = ", barrier_boundary->size());
  // for (auto& [s, lower, upper] : *barrier_boundary) {
  //   OPENSPACE_LOG(D, "[getBoundaryFromStaticObstacles] raw_barrier_boundary: s = ", s, "   lower = ", lower,
  //                 "   upper = ", upper);
  // }
  int path_blocked_idx = -1;
  auto barrier_bound_info = bound_parser_->getBoundaryFromPolyline(parsed_polylines, barrier_buffer);

  OPENSPACE_LOG(D, "[getBoundaryFromStaticObstacles] barrier bound size = ", barrier_bound_info.bound.size());
  // for (auto barrier_bound : barrier_bound_info.bound) {
  //   OPENSPACE_LOG(D, "[getBoundaryFromStaticObstacles] barrier_bound = ", std::get<0>(barrier_bound), " ; ",
  //                 std::get<1>(barrier_bound), " ; ", std::get<2>(barrier_bound));
  // }
  static_obstacle_barrier_bound_filter_->update(barrier_bound_info.bound);
  for (int i = 0; i < barrier_boundary->size(); i++) {
    auto& [s, lower, upper] = barrier_boundary->at(i);
    auto [od_bound, has_bound] = static_obstacle_barrier_bound_filter_->bound().interpolate(s);
    // extend freespace bound when out of map to avoid undetected hard boundary
    CHECK(has_bound);
    double w = adc_width_;
    if (isInRange(gate_ranges_, s)) {
      w = adc_wheel_width_;
    }
    auto& [ods, od_lower, od_upper] = od_bound;
    // OPENSPACE_LOG(D, "[getBoundaryFromStaticObstacles] interpolate_barrier_boundary: s = ", ods,
    //               "   lower = ", od_lower, "   upper = ", od_upper);
    if (!updateBoundary(0.5 * w, od_lower, od_upper, barrier_boundary->at(i))) {
      path_blocked_idx = static_cast<int>(i);
      OPENSPACE_LOG(D, "[getBoundaryFromStaticObstacles] path_blocked_idx = ", path_blocked_idx);
      debug_status_.push_back(PathData::DebugStatusType::BLOCK_STATIC_OBSTACLE);
      break;
    }
  }
  trimPathBounds(path_blocked_idx, barrier_boundary);
  trimPathBounds(path_blocked_idx, soft_boundary);

  // for (auto& [s, lower, upper] : *soft_boundary) {
  //   OPENSPACE_LOG(D, "[getBoundaryFromStaticObstacles] raw_soft_boundary: s = ", s, "   lower = ", lower,
  //                 "   upper = ", upper);
  // }
  auto soft_bound_info = bound_parser_->getBoundaryFromPolyline(parsed_polylines, soft_buffer);
  // for (auto soft_bound : soft_bound_info.bound) {
  //   OPENSPACE_LOG(D, "[getBoundaryFromStaticObstacles] raw_soft_bound = ", std::get<0>(soft_bound), " ; ",
  //                 std::get<1>(soft_bound), " ; ", std::get<2>(soft_bound));
  // }
  static_obstacle_soft_bound_filter_->update(soft_bound_info.bound);
  for (int i = 0; i < soft_boundary->size(); i++) {
    auto& [s, lower, upper] = soft_boundary->at(i);
    auto [od_bound, has_bound] = static_obstacle_soft_bound_filter_->bound().interpolate(s);
    // extend freespace bound when out of map to avoid undetected hard boundary
    CHECK(has_bound);
    double w = adc_width_;
    if (isInRange(gate_ranges_, s)) {
      w = adc_wheel_width_;
    }
    auto& [ods, od_lower, od_upper] = od_bound;
    // OPENSPACE_LOG(D, "[getBoundaryFromStaticObstacles] interpolate_soft_boundary: s = ", s, "   lower = ", od_lower,
    //               "   upper = ", od_upper);
    updateBoundary(0.5 * w, od_lower, od_upper, soft_boundary->at(i), false);
  }

  OPENSPACE_LOG(D, "[OpenspaceOcpBoundParser]getBoundaryFromStaticObstacles ");
  return true;
}

std::pair<double, double> OpenspaceOcpBoundParser::getFsLaterSafeBuffer(const bool use_obs_type,
                                                                        const bool use_obs_speed) {
  std::pair<double, double> safe_buffer_pair(config_.fs_info().barrier_lateral_safe_distance(),
                                             config_.fs_info().soft_lateral_safe_distance());
  if (use_obs_speed) {
    safe_buffer_pair = getFsLaterSafeBufferFromSpeed(config_.fs_info().barrier_lateral_safe_distance(),
                                                     config_.fs_info().soft_lateral_safe_distance());
  } else if (use_obs_type) {
    ;  // for future override
  }
  return safe_buffer_pair;
}

bool OpenspaceOcpBoundParser::updateBoundary(const double config_width, double right_bound, double left_bound,
                                             std::tuple<double, double, double>& boundary, const bool return_if_valid) {
  auto& [s, lower, upper] = boundary;
  // Update the right bound (l_min):
  double new_l_min = std::fmax(lower, right_bound + config_width);
  // Update the left bound (l_max):
  double new_l_max = std::fmin(upper, left_bound - config_width);

  // Check if Adc is blocked.
  // If blocked, don't update anything, return false.
  if (return_if_valid && new_l_min > new_l_max) {
    // SINFO("block info bound ({}, {}) to ({}, {})", lower, upper, new_l_min, new_l_max);
    return false;
  }
  // Otherwise, update path_boundaries and center_line; then return true.
  lower = new_l_min;
  upper = new_l_max;
  return true;
}

std::vector<std::shared_ptr<Decision::DecisionObject>> OpenspaceOcpBoundParser::generateStaticObstacles(
    const std::vector<std::shared_ptr<Decision::DecisionObject>>& all_obstacles, const ReferenceLine& reference_line) {
  std::vector<std::shared_ptr<Decision::DecisionObject>> ret;
  std::vector<ReferencePoint> sample_points;
  initSamplePoints(reference_line, &sample_points);
  // for(auto pt:reference_line.reference_points()){
  //   OPENSPACE_LOG(D, "pt.XY = ", pt.x(), ";", pt.y(), ";", pt.heading(), ";", pt.kappa());
  // }
  std::vector<math::Vec2d> sample_points_vec;
  for (const auto& sp : sample_points) {
    sample_points_vec.emplace_back(math::Vec2d(sp.x(), sp.y()));
  }
  // TODO
  static std::unordered_set<Decision::ObjectType> ovru_type_set{Decision::ObjectType::PEDESTRIAN,
                                                                Decision::ObjectType::VRU};
  for (const auto& obs : all_obstacles) {
    OPENSPACE_LOG(D, "[perception_obs] type = ", static_cast<int>(obs->type), " static = ", obs->is_static,
                  " id = ", obs->id);
    if (!obs->is_static || ovru_type_set.count(obs->type) != 0) continue;
    auto box = obs->cur_box;
    for (const auto& sp : sample_points_vec) {
      if (box.DistanceTo(sp) < 5.0) {
        for (const auto& point : box.GetAllCorners()) {
        }
        ret.emplace_back(obs);
        pnc::SLBoundary perception_sl;
        reference_line.getSLBoundary(ret.back()->cur_box.GetAllCorners(), &perception_sl);
        for (auto pt : ret.back()->cur_box.GetAllCorners()) {
          OPENSPACE_LOG(D, "[perception_obs]box.points : ", pt.x(), ";", pt.y());
        }
        for (auto pt : perception_sl.boundary_point()) {
          OPENSPACE_LOG(D, "[perception_obs] sl = ", pt.s(), ";", pt.l());
        }
        OPENSPACE_LOG(D, "[perception_obs] start2end_sl = ", perception_sl.start_s(), ";", perception_sl.end_s(), ";",
                      perception_sl.start_l(), ";", perception_sl.end_l());

        // auto perception_sl_start_point = reference_line.getReferencePoint(perception_sl.start_s());
        // auto perception_sl_end_point = reference_line.getReferencePoint(perception_sl.end_s());
        // std::pair<double, double> reference_line_direction = std::make_pair(
        //     std::cos(perception_sl_start_point.heading()), std::sin(perception_sl_start_point.heading()));
        // std::pair<double, double> reference_direction_check =
        //     std::make_pair(perception_sl_end_point.x() - perception_sl_start_point.x(),
        //                    perception_sl_end_point.y() - perception_sl_start_point.y());
        // if ((reference_line_direction.first * reference_direction_check.second -
        //      reference_line_direction.second * reference_direction_check.first) < 0) {
        //   perception_sl.set_start_l(-perception_sl.start_l());
        //   perception_sl.set_end_l(-perception_sl.end_l());
        // }
        OPENSPACE_LOG(D, "【fixed】[perception_obs] start2end_sl = ", perception_sl.start_s(), ";",
                      perception_sl.end_s(), ";", perception_sl.start_l(), ";", perception_sl.end_l());
        ret.back()->cur_sl_bound = std::move(perception_sl);
        // a cross_product b > 0 means counter clockwise ,b is left of a
        // a cross_product b < 0 means clockwise,b is right of a
        auto obs_match_point = reference_line.getReferencePoint(obs->cur_box.center_x(), obs->cur_box.center_y());
        std::pair<double, double> dir =
            std::make_pair(std::cos(obs_match_point.heading()), std::sin(obs_match_point.heading()));
        std::pair<double, double> check_dir = std::make_pair(obs->cur_box.center_x() - obs_match_point.x(),
                                                             obs->cur_box.center_y() - obs_match_point.y());
        double cross_product = dir.first * check_dir.second - dir.second * check_dir.first;
        if (cross_product > 0) {
          ret.back()->lat_od_tag = LateralOdTag::RIGHT_BYPASS;
        } else {
          ret.back()->lat_od_tag = LateralOdTag::LEFT_BYPASS;
        }

        break;
      }
    }
  }
  return ret;
}

bool OpenspaceOcpBoundParser::initSamplePoints(const ReferenceLine& reference_line,
                                               std::vector<ReferencePoint>* const sample_points) {
  sample_points->clear();
  const double resolution = 1.0;
  double begin_s = adc_frenet_s_;
  for (double curr_s = begin_s; curr_s < adc_frenet_end_s_; curr_s += resolution) {
    sample_points->emplace_back(reference_line.getReferencePoint(curr_s));
    sample_points->back().setLocalS(curr_s);
  }
  if (!sample_points->empty() && sample_points->back().local_s() != adc_frenet_end_s_) {
    sample_points->emplace_back(reference_line.getReferencePoint(adc_frenet_end_s_));
    sample_points->back().setLocalS(adc_frenet_end_s_);
  }
  return sample_points->size() > 0;
}

std::pair<double, double> OpenspaceOcpBoundParser::getObsLaterSafeBuffer(
    const std::shared_ptr<Decision::DecisionObject> obs, const bool use_obs_type, const bool use_obs_speed) {
  std::pair<double, double> safe_buffer_pair(
      config_.default_min_barrier_lateral_buffer_for_obs(),  // use obs defualt param
      config_.default_min_soft_lateral_buffer_for_obs());
  // not use obs type and speed
  if (use_obs_speed) {
    safe_buffer_pair = getObsLaterSafeBufferFromSpeed(obs, config_.default_min_barrier_lateral_buffer_for_obs(),
                                                      config_.default_min_soft_lateral_buffer_for_obs());
  } else if (use_obs_type) {
    safe_buffer_pair = getObsLaterSafeBufferFromType(obs->type, config_.default_min_barrier_lateral_buffer_for_obs(),
                                                     config_.default_min_soft_lateral_buffer_for_obs());
  } else {
    ;
  }
  return safe_buffer_pair;
}

std::pair<double, double> OpenspaceOcpBoundParser::getObsLaterSafeBufferFromType(
    const Decision::ObjectType& obs_type, const double default_barrier_lateral_buffer,
    const double default_soft_lateral_buffer) {
  std::pair<double, double> safe_buffer_pair(default_barrier_lateral_buffer, default_soft_lateral_buffer);
  if (obs_type_lateral_distance_map_.empty()) {
    return safe_buffer_pair;
  }
  const auto& iter = obs_type_lateral_distance_map_.find(static_cast<int>(obs_type));
  if (iter != obs_type_lateral_distance_map_.end()) {
    safe_buffer_pair.first = iter->second.barrier_lateral_safe_distance();  // barrier
    safe_buffer_pair.second = iter->second.soft_lateral_safe_distance();    // soft
    return safe_buffer_pair;
  } else {
    return safe_buffer_pair;
  }
}

std::pair<double, double> OpenspaceOcpBoundParser::getObsLaterSafeBufferFromSpeed(
    const std::shared_ptr<Decision::DecisionObject> obs, const double default_barrier_lateral_buffer,
    const double default_soft_lateral_buffer) {
  std::pair<double, double> safe_buffer_pair(default_barrier_lateral_buffer, default_soft_lateral_buffer);
  // get the coff of speed
  double coff_a = 0.0;
  double coff_b = 0.0;
  double coff_c_min = 0.0;  // TODO develop it in future
  double lateral_buffer = 0.0;
  auto elvLaterBuffer = [=](const double v_ego, const double v_obs, const double coff_a, const double coff_b,
                            const double coff_c_min) -> double {
    double lateral_buffer = coff_a * (v_ego - v_obs) * (v_ego - v_obs) + coff_b * v_ego + coff_c_min;
    // forbit the calibration is overvalue
    const double max_lateral_buffer = 3.0;
    const double min_lateral_buffer = 1e-6;
    lateral_buffer = std::min(max_lateral_buffer, std::max(min_lateral_buffer, lateral_buffer));
    return lateral_buffer;
  };
  const auto& iter = obs_type_lateral_distance_map_.find(static_cast<int>(obs->type));
  if (iter != obs_type_lateral_distance_map_.end()) {
    coff_a = iter->second.coff_a();
    coff_b = iter->second.coff_b();
    coff_c_min = iter->second.coff_c_min();
  }
  lateral_buffer = elvLaterBuffer(adc_frenet_sd_, obs->spd, coff_a, coff_b, coff_c_min);
  safe_buffer_pair.first = lateral_buffer;   // TODO develop it in future
  safe_buffer_pair.second = lateral_buffer;  //
  return safe_buffer_pair;
}

std::pair<double, double> OpenspaceOcpBoundParser::getFsLaterSafeBufferFromSpeed(
    const double default_barrier_lateral_buffer, const double default_soft_lateral_buffer) {
  std::pair<double, double> safe_buffer_pair(default_barrier_lateral_buffer, default_soft_lateral_buffer);
  double coff_a = 0.0;
  double coff_b = 0.0;
  double coff_c_min = default_barrier_lateral_buffer;  // TODO develop it in future
  double lateral_buffer = 0.0;
  auto elvLaterBuffer = [=](const double v_ego, const double v_obs, const double coff_a, const double coff_b,
                            const double coff_c_min) -> double {
    double lateral_buffer = coff_a * (v_ego - v_obs) * (v_ego - v_obs) + coff_b * v_ego + coff_c_min;
    const double max_lateral_buffer = 3.0;
    const double min_lateral_buffer = 0.1;
    lateral_buffer = std::min(max_lateral_buffer, std::max(min_lateral_buffer, lateral_buffer));
    return lateral_buffer;
  };
  if (config_.has_fs_info()) {
    coff_a = config_.fs_info().coff_a();
    coff_b = config_.fs_info().coff_b();
    coff_c_min = config_.fs_info().coff_c_min();
  }
  lateral_buffer = elvLaterBuffer(adc_frenet_sd_, 0.0, coff_a, coff_b, coff_c_min);
  safe_buffer_pair.first = lateral_buffer;
  safe_buffer_pair.second = lateral_buffer;

  return safe_buffer_pair;
}

void OpenspaceOcpBoundParser::trimPathBounds(const int path_blocked_idx,
                                             std::vector<std::tuple<double, double, double>>* const path_boundaries) {
  if (path_blocked_idx >= 0 && path_blocked_idx < path_boundaries->size() - 1) {
    if (path_blocked_idx == 0) {
      OPENSPACE_LOG(D, "Completely blocked. Cannot move at all");
    }
    path_blocked_idx_ = path_blocked_idx;
    path_boundaries->erase(path_boundaries->begin() + path_blocked_idx, path_boundaries->end());
  }
}

/**
 * @brief 将原始障碍物线段转换为带有SL边界和绕行标签的PolylineInput。
 * @details 此函数直接处理 ObstaclesLinesegment 输入，计算其几何中心以确定绕行方向，
 * 并调用参考线接口计算其SL边界，最终生成一个 PolylineInput 向量。
 * @param obstacles_linesegments 从混合A*传入的障碍物线段集合。
 * @param reference_line 当前的参考线。
 * @return 一个填充了完整信息的 PolylineInput 向量，可直接用于 PathBoundParser。
 */
std::vector<PolylineInput> OpenspaceOcpBoundParser::generatePolylines(
    const std::vector<ObstaclesLinesegment>& obstacles_linesegments, const ReferenceLine& reference_line) {
  std::vector<PolylineInput> result_polylines;

  // 用于为每个处理的线段生成唯一ID
  int segment_counter = 0;
  std::vector<ReferencePoint> sample_points;
  initSamplePoints(reference_line, &sample_points);
  // 外层循环：遍历所有障碍物 (每个障碍物是一个线段集合)
  for (const auto& obs_linesegment : obstacles_linesegments) {
    const auto& original_id = std::get<0>(obs_linesegment);
    const auto& segments = std::get<2>(obs_linesegment);
    // 内层循环：遍历当前障碍物的每一条线段
    for (const auto& segment : segments) {
      // 步骤1: 确定与当前线段相关的参考线S范围
      double start_s = reference_line.length();
      double end_s = 0.0;
      double min_dist = 5.0;                           // 用于寻找最近点的距离阈值
      constexpr double interest_distance_range = 5.0;  // 感兴趣的距离范围
      math::Vec2d closest_ref_point_to_segment;        // 记录离线段最近的那个参考点

      for (const auto& sp : sample_points) {
        math::Vec2d sample_point(sp.x(), sp.y());
        double dist_to_segment = segment.DistanceTo(sample_point);
        if (dist_to_segment < min_dist) {
          min_dist = dist_to_segment;
          closest_ref_point_to_segment = sample_point;
        }
        if (dist_to_segment < interest_distance_range) {
          start_s = std::min(start_s, sp.local_s());
          end_s = std::max(end_s, sp.local_s());
        }
      }

      // 如果没有任何参考点靠近此线段，或者S范围过小，则忽略此线段
      if (min_dist > interest_distance_range - 1e-2 || end_s - start_s < 1e-2) {
        continue;
      }
      // 步骤2: 计算垂足点(pedal_point)并确定L值，以此为依据打标签
      math::Vec2d pedal_point = calculatePedalPoint(closest_ref_point_to_segment, segment.start(), segment.end());
      SLPoint pedal_sl;
      if (!reference_line.xy2sl(math::Vec3d(pedal_point.x(), pedal_point.y(), 0.0), &pedal_sl)) {
        continue;
      }
      if (direction_ == PathPt::Direction::BACKWARD) {
        pedal_sl.set_l(-pedal_sl.l());  // 如果是逆向行驶，L值取反
      }

      // 步骤3: 计算该独立线段的SL边界并裁剪
      auto sl_boundary = std::make_unique<pnc::SLBoundary>();
      reference_line.getSLBoundary({segment.start(), segment.end()}, sl_boundary.get());

      // 步骤4: 创建并填充代表此单个线段的 PolylineInput
      PolylineInput polyline;
      polyline.id = original_id + "_" + std::to_string(segment_counter++);  // 创建唯一ID
      polyline.segments = {segment};                                        // 只包含当前这一条线段
      polyline.start_s = start_s;
      polyline.end_s = end_s;
      polyline.start_l = sl_boundary->start_l();
      polyline.end_l = sl_boundary->end_l();

      if (pedal_sl.l() > 0.0) {
        polyline.nudge_type = PolylineInput::NudgeType::RIGHT_BYPASS;  // 线段在左，向右绕
      } else {
        polyline.nudge_type = PolylineInput::NudgeType::LEFT_BYPASS;  // 线段在右，向左绕
      }

      result_polylines.push_back(std::move(polyline));
    }
  }
  return result_polylines;
}

math::Vec2d OpenspaceOcpBoundParser::calculatePedalPoint(const math::Vec2d point, const math::Vec2d& line_start,
                                                         const math::Vec2d& line_end) {
  math::Vec2d pedal_point;
  float inner_pro_se2sp = (line_end.x() - line_start.x()) * (point.x() - line_start.x()) +
                          (line_end.y() - line_start.y()) * (point.y() - line_start.y());
  float inner_pro_es2ep = (line_start.x() - line_end.x()) * (point.x() - line_end.x()) +
                          (line_start.y() - line_end.y()) * (point.y() - line_end.y());
  if (!(inner_pro_se2sp > 0)) {
    pedal_point = line_start;
  } else if (!(inner_pro_es2ep > 0)) {
    pedal_point = line_end;
  } else {
    float factor = 0.0;
    pedal_point = CalculatePedalPoint(point, line_start, line_end, factor);
  }
  return pedal_point;
}

}  // namespace gpal::pnc::planning