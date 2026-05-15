#include "openspace_path_planner/core/optimizer/openspace_path_optimizer.h"

namespace gpal::pnc::planning {

OpenspacePathOptimizer::~OpenspacePathOptimizer() = default;

void OpenspacePathOptimizer::clear() {
  optimizer_flu_path_.clear();
  optimizer_whole_flu_path_.clear();
  boundarys_vis_.clear();
  target_cal_discret_line_.clear();
  optimizer_index_ = 0;
  optimizer_status_ = OpenspaceStatus::DEFAULT;
  enable_scenario_optimizer_ = false;
  debug_info_ = "optimizer: ";
  is_fallback_attempt_ = false;

  OPENSPACE_LOG(I, "[PathOptimizer] optimizer clear!!");
}

OpenspacePathOptimizer::OpenspaceStatus OpenspacePathOptimizer::run(std::any& data) {
  auto optimizer_data = std::any_cast<std::shared_ptr<OpenspaceOptimizerData>>(data);
  const auto& name = optimizer_data->model_name_;
  auto optimizer_profiles = optimizer_data->optimizer_profiles_;
  if (optimizer_profiles == nullptr) {
    OPENSPACE_LOG(E, "[PathOptimizer][run]optimizer_profiles_ is nullptr");
    return optimizer_status_;
  }
  OPENSPACE_LOG(I, "[PathOptimizer][run]model_name = ", name);
  // preProcess
  OPENSPACE_LOG(D, "[PathOptimizer][run]target_cal_discret_line_.size = ", target_cal_discret_line_.size());
  if (target_cal_discret_line_.empty()) {
    vector<DiscretizedPath> target_partition_discret_line;
    DiscretizedPath target_whole_discret_line;
    pathPreProcess(optimizer_data->origin_path_, target_partition_discret_line, target_whole_discret_line);
    if (optimizer_profiles->is_segmented_optimization()) {
      target_cal_discret_line_.insert(target_cal_discret_line_.end(), target_partition_discret_line.begin(),
                                      target_partition_discret_line.end());
    } else {
      target_cal_discret_line_.push_back(target_whole_discret_line);
    }
    optimizer_status_ = OpenspaceStatus::INIT;
    optimizer_index_ = 0;
    is_fallback_attempt_ = false;
  }

  OPENSPACE_LOG(D, "[PathOptimizer][run]optimizer_data->obstacles_linesegments_.size = ",
                optimizer_data->obstacles_linesegments_->size());

  OPENSPACE_LOG(D, "[PathOptimizer][run]back target_cal_discret_line_.size = ", target_cal_discret_line_.size());

  // ============ 使用while循环处理优化 ============
  while (optimizer_index_ < target_cal_discret_line_.size()) {
    OPENSPACE_LOG(D, "[PathOptimizer][run] optimizer_index_ = ", optimizer_index_);
    DiscretizedPath origin_path = target_cal_discret_line_[optimizer_index_];
    DiscretizedPath pre_optimize_path;
    string model_name = name;
    // Reoptimizing模式的路径裁剪逻辑
    if (model_name == "reoptimizing") {
      // 1. 找到距离原点(0,0,0)最近点的索引
      auto it_closest = std::min_element(origin_path.begin(), origin_path.end(), [](const PathPt& a, const PathPt& b) {
        return std::hypot(a.x(), a.y()) < std::hypot(b.x(), b.y());
      });
      size_t closest_index = std::distance(origin_path.begin(), it_closest);

      // 2. 从最近点开始，向前查找第一个超过缓冲距离的点
      constexpr double forward_buffer = 0.5;
      const auto& ref_pt = origin_path[closest_index];  // 以最近点为参考

      auto it_start_node = std::find_if(origin_path.begin() + closest_index, origin_path.end(),
                                        [&ref_pt, forward_buffer](const PathPt& p) {
                                          return std::hypot(p.x() - ref_pt.x(), p.y() - ref_pt.y()) > forward_buffer;
                                        });

      // 3. 构建裁剪后的轨迹
      if (it_start_node != origin_path.end()) {
        pre_optimize_path.assign(it_start_node, origin_path.end());
      }

      // 4. 边缘情况处理：如果裁剪后轨迹为空，保留最后一点作为目标
      if (pre_optimize_path.empty() && !origin_path.empty()) {
        pre_optimize_path.push_back(origin_path.back());
      }

      // 5. 强制插入原点并重新计算里程
      PathPt ego_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      ego_pose.set_direction(origin_path.front().direction());
      pre_optimize_path.insert(pre_optimize_path.begin(), ego_pose);
    } else if (model_name == "refine") {
      pre_optimize_path = target_cal_discret_line_[optimizer_index_];
      // auto straight_path = straight_line_generator(optimizer_data->destination_point_, 2.0);
      // std::reverse(straight_path.begin(), straight_path.end());
      int clip_index = -1;
      double min_dist_sq = std::numeric_limits<double>::max();
      findNearestPointInTraj(pre_optimize_path, optimizer_data->destination_point_, clip_index, min_dist_sq);
      if (clip_index >= 0) {
        pre_optimize_path.resize(clip_index);
        pre_optimize_path.emplace_back(optimizer_data->destination_point_);
        // pre_optimize_path.insert(pre_optimize_path.end(), straight_path.begin(), straight_path.end());
      }

      // pre_optimize_path.back() = optimizer_data->destination_point_;

    } else {
      pre_optimize_path = target_cal_discret_line_[optimizer_index_];
    }
    ReCalculateLineLength(pre_optimize_path);
    calculateSteer(pre_optimize_path,
                   Singleton<ConfigManager>::get_instance()->vehicle_config().vehicle_param().wheel_base());
    // 打印最终轨迹信息
    OPENSPACE_LOG(D, "[ReOptimizer] Final segment size: ", pre_optimize_path.size());

    auto direction = pre_optimize_path.front().direction();

    // 场景优化器配置
    if (optimizer_profiles->enable_scenario_optimizer()) {
      if (optimizer_index_ + 1 == target_cal_discret_line_.size()) {
        piecewise_point_ = optimizer_data->piecewise_point_;
        setScenarioSRange();
        ocp_optimizer_.setScenarioTags(scenario_s_range_);
      }
    }

    // INIT状态：初始化优化器和边界
    if (optimizer_status_ == OpenspaceStatus::INIT) {
      OPENSPACE_LOG(D, "[PathOptimizer][run] Initializing optimizer and boundaries.");
      generateReferenceLine(optimizer_data->tf_ego_2_map_, pre_optimize_path, pre_optimize_path.front(), init_pt_map_,
                            reference_line_ego_, reference_line_map_);
      // 边界解析
      ocp_bound_parser_.init();
      ocp_bound_parser_.preProcess(reference_line_map_, init_pt_map_, optimizer_data->tf_map_2_ego_);
      ocp_bound_parser_.setModelName(model_name);
      ocp_bound_parser_.generateBounds(*optimizer_data->freespace_ptr_, *optimizer_data->obstacles_linesegments_,
                                       reference_line_map_, true, boundary_, direction);
      // print info
      PathData::BoundsVec3dWithId vis_bd;
      calOptimizerBounds(reference_line_map_, boundary_, vis_bd);
      boundarys_vis_.emplace_back(vis_bd);
      optimizer_data->boundarys_vis_ = boundarys_vis_;
      ocp_optimizer_.init();
      optimizer_status_ = OpenspaceStatus::WAITING;
      OPENSPACE_LOG(D, "[PathOptimizer][run] Initialization complete.");
      // ============ 继续下一次循环 ============
      continue;
    }

    OPENSPACE_LOG(D, "[PathOptimizer][run] optimizer_status_ = ", (int)optimizer_status_);

    // WAITING状态：执行优化
    if (optimizer_status_ == OpenspaceStatus::WAITING) {
      DiscretizedPath post_optimize_path;
      PathPt init_point = pre_optimize_path.front();

      if (is_fallback_attempt_) {
        model_name = "fallback";
        boundary_.set_label(model_name);
        OPENSPACE_LOG(W, "[PathOptimizer] This is a fallback attempt with relaxed boundaries.");
      }
      auto status = ocp_optimizer_.asyncProc(pre_optimize_path, init_point, boundary_, post_optimize_path);
      OPENSPACE_LOG(D, "[PathOptimizer][run] asyncProc status = ", (int)status);

      // ============ 如果优化器正在运行，立即返回，下一帧继续 ============
      if (status == OpenspaceOcpOptimizer::AsyncStatus::ASYNC_UNDERLOCKING) {
        return optimizer_status_;  // 这里必须return，因为异步优化还未完成
      }

      // 优化成功
      if (status == OpenspaceOcpOptimizer::AsyncStatus::ASYNC_SOLVED) {
        if (name == "refine") {
          auto straight_path =
              straight_line_generator(post_optimize_path.back(), optimizer_data->destination_straight_line_length_);
          post_optimize_path.insert(post_optimize_path.end(), straight_path.begin(), straight_path.end());
        }
        optimizer_whole_flu_path_.insert(optimizer_whole_flu_path_.end(), post_optimize_path.begin(),
                                         post_optimize_path.end());
        // 准备处理下一个路段
        debug_info_ += ocp_optimizer_.getDebugInfo() + ", ";
        optimizer_index_++;
        optimizer_status_ = OpenspaceStatus::INIT;
        is_fallback_attempt_ = false;  // 成功后，重置重试状态
        // ============ 继续循环处理下一段 ============
        continue;

      } else {
        // 优化失败
        if (is_fallback_attempt_) {
          // 如果这已经是第二次尝试（fallback），仍然失败了
          OPENSPACE_LOG(E, "[PathOptimizer] Fallback optimization attempt also failed. Using original path segment.");
          optimizer_whole_flu_path_.insert(optimizer_whole_flu_path_.end(), origin_path.begin(), origin_path.end());

          // 准备处理下一个路段
          debug_info_ += ocp_optimizer_.getDebugInfo() + ", ";
          optimizer_index_++;
          optimizer_status_ = OpenspaceStatus::INIT;
          is_fallback_attempt_ = false;  // 结束当前段，重置重试状态
          // ============ 不再return，继续循环处理下一段 ============
          continue;

        } else {
          // 如果这是第一次尝试失败
          OPENSPACE_LOG(W, "[PathOptimizer] Primary optimization attempt failed. Will try fallback immediately.");
          is_fallback_attempt_ = true;                   // 设置标志，立即进行二次尝试
          optimizer_status_ = OpenspaceStatus::WAITING;  // 保持WAITING状态
          // ============ 不再return到下一帧，而是立即重试 ============
          continue;  // 立即进入下一次循环进行fallback尝试
        }
      }
    }
    break;
  }

  // 所有轨迹段处理完成
  if (optimizer_index_ >= target_cal_discret_line_.size()) {
    optimizer_status_ = OpenspaceStatus::FINISH;
    pathPartition(optimizer_whole_flu_path_, optimizer_flu_path_, true);
    debug_info_ += "\n";
    optimizer_data->optimizer_flu_path_ = optimizer_flu_path_;
    optimizer_data->optimizer_whole_flu_path_ = optimizer_whole_flu_path_;
    optimizer_data->debug_info_ = debug_info_;
  }

  return optimizer_status_;
}

void OpenspacePathOptimizer::calOptimizerBound(const ReferenceLine& reference_line,
                                               const std::vector<std::tuple<double, double, double>>& boundary,
                                               std::vector<math::Vec3d>& bound_left,
                                               std::vector<math::Vec3d>& bound_right) {
  for (auto& bound_sl : boundary) {
    SLPoint sl;
    sl.set_s(std::get<0>(bound_sl));
    sl.set_l(std::get<1>(bound_sl));
    bound_right.emplace_back();
    reference_line.sl2xy(sl, &bound_right.back());
    sl.set_l(std::get<2>(bound_sl));
    bound_left.emplace_back();
    reference_line.sl2xy(sl, &bound_left.back());
  }
}

void OpenspacePathOptimizer::calOptimizerBounds(const ReferenceLine& reference_line, const PathBoundary& path_boundary,
                                                PathData::BoundsVec3dWithId& vis_bd) {
  calOptimizerBound(reference_line, path_boundary.barrier_boundary(), vis_bd.barrier_bound_left,
                    vis_bd.barrier_bound_right);
  calOptimizerBound(reference_line, path_boundary.soft_boundary(), vis_bd.soft_bound_left, vis_bd.soft_bound_right);
}

void OpenspacePathOptimizer::smoothPathPoints(const DiscretizedPath& path) {
  OPENSPACE_LOG(D, "[PathOptimizer][OptimizerPath] smoothPathPoints process path");
  PathPt temp_smooth_point;
  constexpr int p = 4;
  constexpr int max_iter_count = 5;

  smooth_path_.clear();
  BSpline2d::Coefs_t points_raw(2, path.size());
  for (size_t i = 0; i < path.size(); i++) {
    points_raw(0, i) = path[i].x();
    points_raw(1, i) = path[i].y();
  }
  auto [params_mat, points_mat] = b_spline::computeChordLengthParams(points_raw, 0.1, 1.0);
  std::shared_ptr<BSplineKnot> knot = std::make_shared<BSplineKnotClamp>(params_mat, p);

  int32_t iter_num = 0;
  auto bs = std::make_shared<BSpline2d>(p, knot, points_mat);
  auto bs_iter = bs;
  double cost = std::numeric_limits<double>::max();
  BSpline2dNormalLineSmoother solver(p);
  bool success = false;
  constexpr double piecewise_length = 1.0;
  constexpr int max_sample_count = 100;
  for (iter_num = 0; iter_num < max_iter_count; iter_num++) {
    /** Resample*/
    auto [states, params] = bs_iter->sample(piecewise_length, max_sample_count);

    if (states.size() <= 5) {
      smooth_status_ = SmoothStatus::FAILED;
      smooth_path_ = path;
      // SFIELD_INFO(openspace, "states.size() too less  {}", states.size());
      // SFIELD_INFO(openspace, "HA_star  solver result : false");
      return;
    }

    std::vector<std::tuple<double, double, double>> barrier_boundary;
    for (int i = 0; i < states.size(); i++) {
      barrier_boundary.emplace_back(params[i], -1.0, 1.0);
    }

    solver.init(states);

    // solver.setFrenetOptimalKernel(1.0, 1);
    solver.setOptimalKernel(0.01 / params.back(), 1);
    solver.setOptimalKernel(1.0, 2);
    solver.setOptimalKernel(10.0, 3);

    std::get<1>(barrier_boundary.front()) = std::max(-1e-6, std::get<1>(barrier_boundary.front()));
    std::get<2>(barrier_boundary.front()) = std::min(1e-6, std::get<2>(barrier_boundary.front()));
    std::get<1>(barrier_boundary.back()) = std::max(-1e-6, std::get<1>(barrier_boundary.back()));
    std::get<2>(barrier_boundary.back()) = std::min(1e-6, std::get<2>(barrier_boundary.back()));
    solver.setBarrierBounds(barrier_boundary);

    // /** Determine constraints*/
    solver.setFrontHeadingConstraint(path.front().theta());
    solver.setBackHeadingConstraint(path.back().theta());
    solver.setKappaBoundary(0.1);

    // if (iter_ratio > 1e-3) {
    // }
    // solver.setOffsetInequatityConstraint(1.0, 3);

    success = solver.solve();
    // SFIELD_INFO(openspace, "HA_star  solver result : {}", success);
    /** Check Result */
    if (success) {
      bs_iter = solver.getOptimizedBSpline2d();
      if (bs_iter->knot().length() > 10000) {
        OPENSPACE_LOG(E, iter_num + 1, "th Optimal success but has invalid results");
        success = false;
      }
    }
    if (!success) {
      OPENSPACE_LOG(E, iter_num + 1, "th Optimal failed");
      break;
    }
    bs = bs_iter;
    solver.reset();
  }
  if (!success) {
    smooth_path_ = path;
    OPENSPACE_LOG(E, "failed to smooth");
    smooth_status_ = SmoothStatus::FAILED;
    return;
  }
  smooth_status_ = SmoothStatus::SUCCESS;

  auto [states, params] = bs->sample(path.size());
  for (int i = 0; i < states.size(); i++) {
    auto& state = states[i];
    auto& s = params[i];
    temp_smooth_point.set_direction(path.front().direction());
    temp_smooth_point.set_x(state(0));
    temp_smooth_point.set_y(state(1));
    temp_smooth_point.set_theta(state(2));
    temp_smooth_point.set_kappa(state(3));
    temp_smooth_point.set_s(s);
    temp_smooth_point.set_dkappa(0);
    temp_smooth_point.set_ddkappa(0);
    smooth_path_.emplace_back(temp_smooth_point);
  }
}

void OpenspacePathOptimizer::pathPreProcess(const DiscretizedPath& orin_traj,
                                            vector<DiscretizedPath>& target_partition_discret_line,
                                            DiscretizedPath& target_discret_line) {
  if (orin_traj.empty()) {
    return;
  }
  vector<DiscretizedPath> partition_paths;
  pathPartition(orin_traj, partition_paths, false);
  // for (auto part_path : partition_paths) {
  //   for (auto pt : part_path) {
  //     OPENSPACE_LOG(D, "[PathOptimizer][PreProcess] 处理前： ", pt.x(), ", ", pt.y(), ", ", (int)pt.direction(), ",
  //     ",
  //                   pt.theta(), ", ", pt.kappa(), ", ", pt.s());
  //   }
  // }
  // TODO: 后续传入bound的参数
  double resolution = 0.2;
  DiscretizedPath init_path;
  PathPt* last_end_terminal_pt = nullptr;
  for (auto part_path : partition_paths) {
    DiscretizedPath path = DiscretizedPath(part_path);
    // 将上一段轨迹的终点插入为当前轨迹的起点
    if (last_end_terminal_pt != nullptr) {
      PathPt cur_start_terminal_pt = path.front();
      // cur_start_terminal_pt.set_x(last_end_terminal_pt->x());
      // cur_start_terminal_pt.set_y(last_end_terminal_pt->y());
      path.insert(path.begin(), cur_start_terminal_pt);
      // 重新初始化s
      ReCalculateLineLength(path);
    }
    int size = path.length() / resolution + 1;
    init_path.clear();
    for (int i = 0; i <= size; ++i) {
      init_path.push_back(path.evaluate(i * resolution));
    }
    if (init_path.back().s() - path.back().s() > 0.01) {
      init_path.push_back(
          math::interpolateUsingLinearApproximation(init_path.back(), path.back(), (size + 1) * resolution));
      init_path.back().set_direction(path.back().direction());
    }
    // for (auto path : init_path) {
    //   OPENSPACE_LOG(D, "[PathOptimizer][PreProcess] 处理后： ", path.x(), " ", path.y(), " ", (int)path.direction(),
    //                 " ", path.theta(), " ", path.kappa());
    // }
    target_partition_discret_line.push_back(init_path);

    last_end_terminal_pt = &init_path.back();
  }
  target_discret_line.clear();
  target_discret_line.emplace_back(target_partition_discret_line.front().front());
}

void OpenspacePathOptimizer::pathPartition(const DiscretizedPath& orin_traj, vector<DiscretizedPath>& partition_paths,
                                           bool only_partition) {
  if (orin_traj.empty()) {
    return;
  }
  DiscretizedPath partition_path;
  auto direction = orin_traj.begin()->direction();
  for (auto p : orin_traj) {
    if (direction != p.direction()) {
      if (!only_partition) {
        ReCalculateLineLength(partition_path);  // 初始化 s
        // loadKappa(partition_path);              // 初始化 kappa
      }
      partition_paths.push_back(partition_path);
      direction = p.direction();
      partition_path.clear();
      partition_path.push_back(p);
    } else {
      partition_path.push_back(p);
    }
  }
  if (!only_partition) {
    ReCalculateLineLength(partition_path);
    // loadKappa(partition_path);
  }
  partition_paths.push_back(partition_path);
}

void OpenspacePathOptimizer::loadKappa(DiscretizedPath& orin_traj) {
  CalculateLineCurvatureBaseOnThreePoint(orin_traj);
  for (auto& pt : orin_traj) {
    if (pt.direction() == PathPt::Direction::BACKWARD) {
      pt.set_kappa(-pt.kappa());
    }
  }
}

void OpenspacePathOptimizer::generateReferenceLine(const Eigen::Matrix4d& tf_ego_2_map, const DiscretizedPath& path_ego,
                                                   const PathPt& init_pt_ego, TrajectoryPt& init_pt_map,
                                                   ReferenceLine& reference_line_ego,
                                                   ReferenceLine& reference_line_map) {
  std::vector<ReferencePoint> ref_points_ego;
  std::vector<ReferencePoint> ref_points_map;
  for (auto pt : path_ego) {
    ref_points_ego.emplace_back(ReferencePoint(pt, 0.0, pt.theta(), pt.kappa(), pt.dkappa()));
    transfer::transformPoint(tf_ego_2_map, &pt);
    math::Vec3d map_rpy_vec3d(0.0, 0.0, pt.theta());
    transfer::transformRPY(tf_ego_2_map, &map_rpy_vec3d);
    pt.set_theta(map_rpy_vec3d.z());
    ref_points_map.emplace_back(ReferencePoint(pt, 0.0, pt.theta(), pt.kappa(), pt.dkappa()));
  }
  if (ref_points_ego.empty() || ref_points_map.empty()) {
    return;
  }
  reference_line_ego = ReferenceLine(ref_points_ego);
  reference_line_map = ReferenceLine(ref_points_map);

  PathPt temp_map_point = init_pt_ego;
  transfer::transformPoint(tf_ego_2_map, &temp_map_point);
  math::Vec3d map_rpy_vec3d(0.0, 0.0, temp_map_point.theta());
  transfer::transformRPY(tf_ego_2_map, &map_rpy_vec3d);
  temp_map_point.set_theta(map_rpy_vec3d.z());
  init_pt_map = TrajectoryPt(temp_map_point);
}

void OpenspacePathOptimizer::setScenarioSRange() {
  scenario_s_range_.clear();
  // 库位内场景,使用分段点区分
  double piecewise_offset = 0.0;
  auto piecewise_sl = reference_line_ego_.getFrenetPoint(piecewise_point_);
  double piecewise_start_s = std::min(std::max(piecewise_sl.s() + piecewise_offset, 0.0), reference_line_ego_.length());
  double piecewise_end_s = std::max(reference_line_ego_.length(), 0.0);
  scenario_s_range_.emplace_back(
      std::tuple<std::string, float, float, bool>{"Fine_Stop", piecewise_start_s, piecewise_end_s, true});
}

// 投影函数的实现
std::optional<PathProjectionResult> OpenspacePathOptimizer::findProjectionOnPath(const DiscretizedPath& path,
                                                                                 const PathPt& point_to_project) const {
  if (path.size() < 2) {
    return std::nullopt;
  }
  double min_dist_sq = std::numeric_limits<double>::max();
  PathProjectionResult result;
  for (size_t i = 0; i < path.size() - 1; ++i) {
    const auto& p1 = path[i];
    const auto& p2 = path[i + 1];
    const double seg_dx = p2.x() - p1.x();
    const double seg_dy = p2.y() - p1.y();
    const double seg_len_sq = seg_dx * seg_dx + seg_dy * seg_dy;
    if (seg_len_sq < 1e-6)
      continue;
    const double dx = point_to_project.x() - p1.x();
    const double dy = point_to_project.y() - p1.y();
    double t = (dx * seg_dx + dy * seg_dy) / seg_len_sq;
    PathPt closest_pt_on_segment;
    if (t < 0.0) {
      closest_pt_on_segment = p1;
    } else if (t > 1.0) {
      closest_pt_on_segment = p2;
    } else {
      closest_pt_on_segment.set_x(p1.x() + t * seg_dx);
      closest_pt_on_segment.set_y(p1.y() + t * seg_dy);
    }
    double dist_sq =
        std::hypot(point_to_project.x() - closest_pt_on_segment.x(), point_to_project.y() - closest_pt_on_segment.y());
    if (dist_sq < min_dist_sq) {
      min_dist_sq = dist_sq;
      result.closest_segment_start_index = i;
      result.projected_point = closest_pt_on_segment;
      double dist_on_seg = std::hypot(closest_pt_on_segment.x() - p1.x(), closest_pt_on_segment.y() - p1.y());
      result.distance_along_path = p1.s() + dist_on_seg;
    }
  }
  return result;
}

void OpenspacePathOptimizer::calculateSteer(DiscretizedPath& path, double wheelbase) {
  // 检查路径点是否为空
  if (path.empty()) {
    return;
  }
  // 直接遍历所有点，使用已有的kappa计算steer
  for (size_t i = 0; i < path.size(); ++i) {
    // 1. 从路径点直接获取曲率
    double kappa = path[i].kappa();

    // 2. 使用核心公式计算前轮转角
    double steer = std::atan(wheelbase * kappa);

    // 3. 将计算出的steer值更新到路径对象中
    path[i].set_front_steer(steer);
  }
}

}  // namespace gpal::pnc::planning