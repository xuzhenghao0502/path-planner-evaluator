#include "redundant_lateral_planner.h"

namespace gpal::pnc::planning {

bool RedundantLateralPlanner::init() {
  // 初始化横向规划器
  auto config_manager = Singleton<ConfigManager>::get_instance();
  planning_config_ = config_manager->getConfig<RedundantPlannerConfig>("RedundantPlannerConfig");
  if (!RealTimePathPlanner::init()) {
    RLOG(E, "[RedundantLateralPlanner] Initialization failed.");
    return false;
  }
  return true;
}

bool RedundantLateralPlanner::reset() {
  // 重置横向规划器状态
  return true;
}

bool RedundantLateralPlanner::processing(SnapShotData& snap_shot_data) {
  // Step 1: 公共预处理（两种模式均需执行）
  if (!preprocess(snap_shot_data)) {
    RLOG(E, "[RedundantLateralPlanner] Preprocessing failed.");
    return false;
  }

  // Step 2: 每帧刷新配置，支持动态调参
  planning_config_ =
      Singleton<ConfigManager>::get_instance()->getConfig<RedundantPlannerConfig>("RedundantPlannerConfig");

  // Step 3: 根据开关选择规划模式
  if (planning_config_.enable_unconstrained_lateral_mode()) {
    RLOG(I, "[RedundantLateralPlanner] Mode: UNCONSTRAINED (skip boundary generation and OCP optimization).");
    auto status = runOnceUnconstrained(snap_shot_data);
    return (status == PathData::StatusType::RUNNING);
  } else {
    RLOG(I, "[RedundantLateralPlanner] Mode: CONSTRAINED (full boundary + OCP pipeline).");
    auto status = runOnce(snap_shot_data);
    return (status == PathData::StatusType::RUNNING);
  }
}

// =============================================================================
// 无约束横向规划
// =============================================================================

PathData::StatusType RedundantLateralPlanner::runOnceUnconstrained(SnapShotData& snap_shot_data) {
  RLOG(I, "[RedundantLateralPlanner] ------------------Unconstrained Lateral Planning Start.------------------");
  auto t_start = std::chrono::steady_clock::now();

  const auto& target_reference_line_info = *snap_shot_data.target_ref_line_info_ptr;
  const auto& current_reference_line_info = *snap_shot_data.drive_ref_line_info_ptr;
  const auto& localization = *snap_shot_data.localization_ptr;
  const auto& chassis = *snap_shot_data.chassis_ptr;
  const auto& decision_result = *snap_shot_data.decision_result_ptr;
  const auto& prev_speed_data = snap_shot_data.prev_speed_data;
  const auto time_stamp = snap_shot_data.time_stamp;
  auto* path_data = snap_shot_data.path_data.get();

  // ──────────────────────────────────────────────────────────────
  // Step 1: 调用父类 preProcess()
  //   必须执行：generateUnconstrainedPath() 依赖父类 protected 成员
  //   （planning_start_point_、target_ref_line_info_、behavior_ 等），
  //   这些均由 preProcess() 赋值。
  // ──────────────────────────────────────────────────────────────
  preProcess(target_reference_line_info, current_reference_line_info, localization, chassis, decision_result,
             prev_speed_data);

  // ──────────────────────────────────────────────────────────────
  // Step 2: 确保 optimizer_.curr_profile_type_ 已初始化（双重保险）
  //   主路径：Step 3 的默认边界 label 为 "regular"，
  //           unconstrainedProc 内部 preProcess() 会从 label 正确赋值 curr_profile_type_。
  //   此处作为双重保险：若 profiles map 为空则提前退出。
  // ──────────────────────────────────────────────────────────────
  if (!optimizer_.ensureProfileType("regular")) {
    RLOG(E, "[RedundantLateralPlanner] [Unconstrained] OcpPathOptimizer profiles map is empty, cannot proceed.");
    return PathData::StatusType::FAILED;
  }

  // ──────────────────────────────────────────────────────────────
  // Step 3: 在内部 target_ref_line_info_ 写入默认边界
  //
  //   问题根因：generateUnconstrainedPath() 将 target_ref_line_info_->path_boundary()
  //   传给 optimizer_.unconstrainedProc()，OcpPathOptimizer::preProcess() 用它来
  //   初始化 N_、ds_、accumulated_s_。
  //   但内部 target_ref_line_info_ 是 RealTimePathPlanner::preProcess() 的深拷贝，
  //   path_boundary 始终为空（size=0）→ preProcess 因 "boundary too short" 失败。
  //
  //   修复：此处基于参考线参数构建默认边界，写入内部 target_ref_line_info_，
  //   保证每帧 unconstrainedProc 都能从有效数据初始化几何状态（N_/ds_/accumulated_s_），
  //   从而正确更新规划起点。
  // ──────────────────────────────────────────────────────────────
  {
    const double half_w = static_cast<double>(planning_config_.unconstrained_default_boundary_half_width());
    // 使用与 initPathBoundary 相同的 start_s / end_s 计算方式
    const double ref_len = target_ref_line_info_->ref_line().length();
    const double start_s = std::max(0.0, adc_frenet_s_ - adc_rear_length_);
    const double end_s   = std::min(ref_len, adc_frenet_end_s_ + adc_front_length_);
    const double length  = end_s - start_s;
    if (length > 1e-3) {
      const int seg_size = std::max(1, static_cast<int>(std::round(length / kPathResolution)));
      const double delta  = length / seg_size;
      const int n_pts     = seg_size + 1;   // PathBoundary size = N+1
      auto* internal_pb = target_ref_line_info_->mutable_path_boundary();
      internal_pb->reset(start_s, delta, n_pts);
      internal_pb->set_label("regular");
      auto* barrier = internal_pb->mutable_barrier_boundary();
      auto* soft    = internal_pb->mutable_soft_boundary();
      barrier->reserve(n_pts);
      soft->reserve(n_pts);
      for (int i = 0; i < n_pts; ++i) {
        const double s = start_s + i * delta;
        barrier->emplace_back(s, -half_w, half_w);
        soft->emplace_back(s, -half_w, half_w);
      }
    } else {
      RLOG(W, "[RedundantLateralPlanner] [Unconstrained] ref line too short to build default boundary, skip.");
    }
  }

  // ──────────────────────────────────────────────────────────────
  // Step 4: 直接生成无约束路径（复用父类 protected 方法）
  //   跳过：边界生成（generateRegularPathBound 等）和 OCP 约束优化（optimizer_.proc）
  // ──────────────────────────────────────────────────────────────
  generateUnconstrainedPath(path_data);

  auto t_unconstrained = std::chrono::steady_clock::now();
  auto duration_un = std::chrono::duration<float, std::milli>(t_unconstrained - t_start).count();
  RLOG(I, fmt::format("[RedundantLateralPlanner] [Unconstrained] generateUnconstrainedPath: {:.2f} ms", duration_un));

  // ──────────────────────────────────────────────────────────────
  // Step 4: 最终安全兜底
  //   若无约束路径仍为空（极端情况），启用保护路径
  // ──────────────────────────────────────────────────────────────
  if (path_data->discretized_path().empty()) {
    RLOG(W, "[RedundantLateralPlanner] [Unconstrained] Path is empty, activating PROTECTION fallback.");
    debug_status_.emplace(PathData::DebugStatusType::SOLVER_PROTECT);
    generateProtectPath(prev_speed_data, time_stamp, path_data);
  }

  auto t_protect = std::chrono::steady_clock::now();
  auto duration_protect = std::chrono::duration<float, std::milli>(t_protect - t_unconstrained).count();
  RLOG(I, fmt::format("[RedundantLateralPlanner] [Unconstrained] Protection fallback: {:.2f} ms", duration_protect));

  // ──────────────────────────────────────────────────────────────
  // Step 5: 设置剩余距离信息（与约束模式保持一致）
  // ──────────────────────────────────────────────────────────────
  path_data->setRemainDisInfo(snap_shot_data.remain_dis_info);

  // ──────────────────────────────────────────────────────────────
  // Step 6: 补充默认 path_boundary，供纵向规划使用
  //   无约束模式跳过了边界生成，target_ref_line_info_->path_boundary() 为空。
  //   纵向规划（OcpQpSpeedPlanner）从此处读取边界信息，为防止崩溃需补全。
  // ──────────────────────────────────────────────────────────────
  fillDefaultPathBoundary(snap_shot_data);

  auto t_end = std::chrono::steady_clock::now();
  auto duration_total = std::chrono::duration<float, std::milli>(t_end - t_start).count();

  // 调试信息
  r_debug_info_ = fmt::format("[unconstrained_lat: {:.2f} ms]", duration_total);
  path_data->mutableDebugInfo()->append(r_debug_info_);
  path_data->mutablePlannerDebugStatus()->push_back(PathData::DebugStatusType::SOLVER_UNCONSTRAINED);
  debug_info_ += "\n[UNCONSTRAINED_LATERAL_ACTIVE]";

  RLOG(I, fmt::format("[RedundantLateralPlanner] ------------------Unconstrained Lateral Planning Done: {:.2f} ms.------------------",
                      duration_total));
  return PathData::StatusType::RUNNING;
}

void RedundantLateralPlanner::fillDefaultPathBoundary(SnapShotData& snap_shot_data) {
  // 确保 path_boundary_ptr 已分配
  auto& pb_ptr = snap_shot_data.path_boundary_ptr;
  if (pb_ptr == nullptr) {
    pb_ptr = std::make_shared<PathBoundary>();
  }
  pb_ptr->reset();
  // label 必须是 OcpPathOptimizer profiles map 中存在的合法 key（如 "regular"），
  // 否则 preProcess() 会因 label 查不到而返回 false（虽然有兜底，但直接使用合法 key 更安全）。
  pb_ptr->set_label("regular");

  // 同步写入 target_ref_line_info_ 内部的 path_boundary（纵向规划从这里读取）
  auto* ref_pb = snap_shot_data.target_ref_line_info_ptr->mutable_path_boundary();
  ref_pb->reset();
  ref_pb->set_label("regular");

  // 基于无约束路径的 s 坐标，生成等宽宽松边界
  // 半宽由配置项控制，语义为：中心线两侧各 half_width 米均可通行
  // tuple 格式: (s, l_right（负值向右）, l_left（正值向左）)
  const double half_width = static_cast<double>(planning_config_.unconstrained_default_boundary_half_width());
  const auto& path = snap_shot_data.path_data->discretized_path();

  if (path.empty()) {
    RLOG(W, "[RedundantLateralPlanner] fillDefaultPathBoundary: path is empty, boundary will be empty.");
    return;
  }

  // PathBoundary 没有 appendBoundaryPoint 接口，直接通过 mutable 指针向 vector 写入
  // barrier_boundary_ 和 soft_boundary_ 均填充相同的宽松默认边界
  auto* barrier = pb_ptr->mutable_barrier_boundary();
  auto* soft    = pb_ptr->mutable_soft_boundary();
  auto* ref_barrier = ref_pb->mutable_barrier_boundary();
  auto* ref_soft    = ref_pb->mutable_soft_boundary();

  barrier->reserve(path.size());
  soft->reserve(path.size());
  ref_barrier->reserve(path.size());
  ref_soft->reserve(path.size());
   // TODO 需要时空联合决策上线后，输入正确的边界，并输入block @zhangzhineng
  for (const auto& pt : path) {
    const double s = pt.s();
    barrier->emplace_back(s, -half_width, half_width);
    soft->emplace_back(s, -half_width, half_width);
    ref_barrier->emplace_back(s, -half_width, half_width);
    ref_soft->emplace_back(s, -half_width, half_width);
  }

  RLOG(D, fmt::format("[RedundantLateralPlanner] fillDefaultPathBoundary: {} points, half_width={:.2f}m",
                      path.size(), half_width));
}

// =============================================================================
// 完整约束横向规划（原有逻辑，保持不变）
// =============================================================================

bool RedundantLateralPlanner::preprocess(SnapShotData& snap_shot_data) {
  if (!snap_shot_data.target_ref_line_info_ptr->isValid()) {
    if (snap_shot_data.localization_ptr->isLastOdometryLocalization()
        == snap_shot_data.localization_ptr->isOdometryLocalization()) {
      PathPlannerBase::runRefinePath(snap_shot_data.pre_path_data, snap_shot_data.planning_start_point,
                                     snap_shot_data.path_data.get());
      snap_shot_data.path_data->mutablePlannerDebugStatus()->push_back(PathData::DebugStatusType::REF_NULL);
      RLOG(W, "[RedundantLateralPlanner::preprocess]: target_ref_line_info is nullptr, refine prev path");
    } else {
      RLOG(W, "[RedundantLateralPlanner::preprocess]: target_ref_line_info is nullptr, clear pathdata");
    }
    return false;
  }
  // 当定位坐标系发生变更时，frame->getMutablePathDataPtr()->discretized_path()为空，将其作为上帧历史轨迹不会对规划造成错误影响.
  if (snap_shot_data.pre_path_data != nullptr
      && snap_shot_data.localization_ptr->isLastOdometryLocalization()
             == snap_shot_data.localization_ptr->isOdometryLocalization()) {
    *snap_shot_data.path_data->mutableDiscretizedPath() = snap_shot_data.pre_path_data->discretized_path();
  }
  return true;
}

PathData::StatusType RedundantLateralPlanner::runOnce(SnapShotData& snap_shot_data) {
  // ======================================================================
  // 1. 预处理阶段：直接复用父类的功能
  // ======================================================================
  // preProcess 是父类的 protected 方法，子类可以直接调用。
  // 它会设置 behavior_ 等重要的 protected 成员变量。
  RLOG(I, "[RedundantLateralPlanner] ------------------Lateral Path planning Start.------------------");
  auto t1 = std::chrono::steady_clock::now();
  const auto& target_reference_line_info = *snap_shot_data.target_ref_line_info_ptr;
  const auto& current_reference_line_info = *snap_shot_data.drive_ref_line_info_ptr;
  const auto& Localization = *snap_shot_data.localization_ptr;
  const auto& chassis = *snap_shot_data.chassis_ptr;
  const auto& decision_result = *snap_shot_data.decision_result_ptr;
  const auto& prev_speed_data = snap_shot_data.prev_speed_data;
  const auto& console = *snap_shot_data.console_ptr;
  const auto& freespace = *snap_shot_data.freespace_ptr;
  const auto time_stamp = snap_shot_data.time_stamp;
  auto& path_boundary = snap_shot_data.path_boundary_ptr;
  auto path_data = snap_shot_data.path_data.get();
  auto& debug_info = snap_shot_data.debug_info;
  std::pair<bool, double> remain_dis_info = snap_shot_data.remain_dis_info;
  preProcess(target_reference_line_info, current_reference_line_info, Localization, chassis, decision_result,
             prev_speed_data);
  path_boundary->reset();

  // 复用父类的 protected 方法
  generateFibonacciCVTSRange(0.0, path_bound_points_config_.adc_t_range(), 0.1);

  // 对于 Nudge 关闭的场景，直接复用父类的逻辑
  if (!real_time_path_planner_config_.enable_nudge()) {
    PathPlannerBase::runLoadRefLine(target_ref_line_info_->ref_line(), planning_start_point_, path_data);
    path_data->mutablePlannerDebugStatus()->push_back(PathData::DebugStatusType::NUDGE_DISABLED_REF);
    RLOG(W, "[RedundantLateralPlanner] Nudge is disabled, using reference line directly.");
    return PathData::StatusType::RUNNING;
  }
  auto t2 = std::chrono::steady_clock::now();
  auto duration1 = std::chrono::duration<float, std::milli>(t2 - t1).count();
  RLOG(I, fmt::format("[RedundantLateralPlanner] [RedundantLateralPreprocess] {} ms", duration1));
  // ======================================================================
  // 2. 路径边界生成：这是子类定制化的核心逻辑
  // ======================================================================
  // behavior_ 是父类的 protected 成员，在 preProcess 中被赋值，这里可以直接读取
  const bool is_lane_change = (behavior_ == FsmState::LEFT_CHANGE || behavior_ == FsmState::RIGHT_CHANGE);
  const bool is_lane_change_return = (behavior_ == FsmState::LEFT_RETURN || behavior_ == FsmState::RIGHT_RETURN);
  const bool is_lane_change_hold = (behavior_ == FsmState::LEFT_HOLD || behavior_ == FsmState::RIGHT_HOLD);
  const bool is_lane_change_attempt = (behavior_ == FsmState::LEFT_ATTEMPT || behavior_ == FsmState::RIGHT_ATTEMPT);

  Status status;
  // debug_status_ 是父类的 protected 成员，子类可以直接修改
  if (is_lane_change || is_lane_change_return) {
    // 变道或变道返回场景：调用父类提供的变道边界生成函数
    const auto label = is_lane_change ? "lane_change" : "lane_change_return";
    const auto debug_type =
        is_lane_change ? PathData::DebugStatusType::LANE_CHANGE : PathData::DebugStatusType::LANE_CHANGE_RETURN;
    target_ref_line_info_->mutable_path_boundary()->set_label(label);
    debug_status_.emplace(debug_type);

    // generateLaneChangePathBound 是父类的 protected 方法
    status = generateLaneChangePathBound(console, freespace, *target_ref_line_info_,
                                         decision_result.getLateralBoundaryDecision(),
                                         decision_result.getLongitudinalBoundaryDecision(),
                                         target_ref_line_info_->mutable_path_boundary());
  } else {
    // 直行或其它场景
    if (PathOptimizer::isInIgnoreRange(ignore_ranges_, adc_frenet_s_)) {
      // 强制返回场景：调用父类提供的强制返回边界生成函数
      target_ref_line_info_->mutable_path_boundary()->set_label("force_back");
      debug_status_.emplace(PathData::DebugStatusType::IGNORE_RANGE_REF);
      debug_status_.emplace(PathData::DebugStatusType::FORCE_BACK);

      // generateForceBackPathBound 是父类的 protected 方法
      status = generateForceBackPathBound(*target_ref_line_info_, decision_result.getLateralBoundaryDecision(),
                                          target_ref_line_info_->mutable_path_boundary());
    } else {
      // 常规场景：调用父类提供的常规边界生成函数
      target_ref_line_info_->mutable_path_boundary()->set_label("regular");
      debug_status_.emplace(PathData::DebugStatusType::REGULAR);

      // generateRegularPathBound 是父类的 protected 方法
      status = generateRegularPathBound(console, freespace, *target_ref_line_info_,
                                        decision_result.getLateralBoundaryDecision(),
                                        decision_result.getLongitudinalBoundaryDecision(),
                                        target_ref_line_info_->mutable_path_boundary());
    }
  }

  debug_info += r_debug_info_;

  // 边界处理
  if (!status.ok()) {
    RLOG(E, "[RedundantLateralPlanner] Path boundary generation failed: " + status.error_message());
    target_ref_line_info_->mutable_path_boundary()->set_label("reference_line");
  } else {
    target_ref_line_info_->mutable_path_boundary()->trim(real_time_path_planner_config_.force_tail_length());
  }

  auto t3 = std::chrono::steady_clock::now();
  auto duration2 = std::chrono::duration<float, std::milli>(t3 - t2).count();
  RLOG(I, fmt::format("[RedundantLateralPlanner] [RedundantLateralBoundParser] {} ms", duration2));

  // 复用父类的方法
  updateRefOffsetInfo(decision_result, ((is_lane_change_hold || is_lane_change_attempt) && l_offset_behavior_valid_));

  // ======================================================================
  // 3. 路径优化阶段：直接复用父类的功能
  // ======================================================================
  // 复用父类的方法
  calcLaneKeepStartS(decision_result, path_data);

  // optimizer_ 是父类的 protected 成员，可以直接访问和修改
  optimizer_.setIsLaneChange(is_lane_change);

  // processPathOptimizer 是父类的 protected 方法
  status = processPathOptimizerCustom(decision_result, prev_speed_data, time_stamp, path_data);
  last_optimizer_status_ = status;  // last_optimizer_status_ 是父类的 protected 成员
  pre_change_ref_ = change_ref_;    // pre_change_ref_, change_ref_ 都是父类的 protected 成员

  if (!status.ok()) {
    RLOG(E, "[RedundantLateralPlanner] Path optimizer failed: " + status.error_message());
  }

  // ======================================================================
  // 4. 剩余距离信息设置
  // ======================================================================
  path_data->setRemainDisInfo(remain_dis_info);
  RLOG(D, fmt::format("[RedundantLateralPlanner] Remain distance info: valid={}, distance={:.2f}",
                      remain_dis_info.first, remain_dis_info.second));

  // ======================================================================
  // 5. 后处理与可视化：直接复用父类的功能
  // ======================================================================
  // 复用父类的方法
  auto path_barrier_boundary_info =
      target_ref_line_info_->mutable_path_boundary()->mutable_path_barrier_boundary_info();
  generateRealtimeTrajBoundaryInfo(path_barrier_boundary_info, path_data);

  *path_boundary = target_ref_line_info_->path_boundary();

  auto t4 = std::chrono::steady_clock::now();
  auto duration3 = std::chrono::duration<float, std::milli>(t4 - t3).count();
  RLOG(I, fmt::format("[RedundantLateralPlanner] [RedundantLateralPathPlan] {} ms", duration3));

  // 复用父类的方法
  CollisionPostProcess(freespace, snap_shot_data.localization_ptr, path_data);

  auto t5 = std::chrono::steady_clock::now();
  auto duration4 = std::chrono::duration<float, std::milli>(t5 - t4).count();
  RLOG(I, fmt::format("[RedundantLateralPlanner] [RedundantLateralCollisionPostProcess] {} ms", duration4));

  // 将本轮的调试信息更新到 path_data
  path_data->mutableDebugInfo()->append(debug_info_);
  for (const auto& it : debug_status_) {
    path_data->mutablePlannerDebugStatus()->push_back(it);
  }

  // 复用父类的方法
  visualization(path_data);
  auto t6 = std::chrono::steady_clock::now();
  auto duration5 = std::chrono::duration<float, std::milli>(t6 - t5).count();
  RLOG(I, fmt::format("[RedundantLateralPlanner] [RedundantLateralVisualization] {} ms", duration5));
  RLOG(D, "[RedundantLateralPlanner] [RedundantLateralCost] = ",
       (duration1 + duration2 + duration3 + duration4 + duration5), " ms");
  return PathData::StatusType::RUNNING;
}

Status RedundantLateralPlanner::processPathOptimizerCustom(const DecisionResult& decision_result,
                                                           const SpeedData& prev_speed_data, const int64_t& time_stamp,
                                                           PathData* const path_data) {
  auto& ref_line_info = target_ref_line_info_;
  optimizer_.setSpecialSceneRange(special_scene_s_range_sets_);
  optimizer_.setRefOffsetsInfo(ref_offsets_info_);

  // 检查边界生成是否失败，如果失败，直接加载参考线（复用父类逻辑）
  if (ref_line_info->path_boundary().label().find("reference_line") != std::string::npos) {
    RLOG(E, "[RedundantLateralPlanner] Boundary generation failed, loading reference line.");
    PathPlannerBase::runLoadRefLine(ref_line_info->ref_line(), ref_line_info->adc_planning_point(), path_data);
    return Status::OK();
  }

  // --- 同步优化核心逻辑 ---
  // 直接调用同步 proc() 接口
  auto lat_start = std::chrono::steady_clock::now();
  Status status =
      optimizer_.proc(ref_line_info->ref_line(), planning_start_point_, ref_line_info->path_boundary(), path_data);
  auto lat_end = std::chrono::steady_clock::now();
  auto opt_duration = std::chrono::duration<float, std::milli>(lat_end - lat_start).count();
  RLOG(I, fmt::format("[RedundantLateralPlanner] [RedundantLateralOptimization] {} ms", opt_duration));
  auto solve_info = optimizer_.solverInfo();
  r_debug_info_ = fmt::format(" [i:{}, t:{:.2f}]", solve_info.iteration_number, solve_info.computation_time * 1000.0);
  r_debug_info_ += fmt::format("[opt: {:.2f} {}]", opt_duration, status.ok() ? "" : "[Failed]");
  // 检查同步求解的结果
  if (!status.ok()) {
    // 如果主路径求解失败，记录状态，并尝试使用“无约束”模式作为第一层兜底
    RLOG(E, "[RedundantLateralPlanner] Synchronous solver failed. Attempting UNCONSTRAINED fallback.");
    debug_status_.emplace(PathData::DebugStatusType::SOLVER_FAILED);
    generateUnconstrainedPath(path_data);
  }
  auto un_end = std::chrono::steady_clock::now();
  auto un_duration = std::chrono::duration<float, std::milli>(un_end - lat_end).count();
  RLOG(I, fmt::format("[RedundantLateralPlanner] [RedundantLateralUnconstrained] {} ms", un_duration));
  r_debug_info_ += fmt::format("[un: {:.2f}]", un_duration);
  // --- 最终安全检查 ---
  // 无论主路径求解或无约束求解是否成功，只要最终路径为空，就必须启用最终极的“保护”模式兜底
  if (path_data->discretized_path().empty()) {
    RLOG(E, "[RedundantLateralPlanner] Path is still empty. Activating PROTECTION fallback.");
    debug_status_.emplace(PathData::DebugStatusType::SOLVER_PROTECT);

    // generateProtectPath 是父类提供的 protected 方法，用于生成保护路径
    generateProtectPath(prev_speed_data, time_stamp, path_data);
  }
  auto pro_end = std::chrono::steady_clock::now();
  auto pro_duration = std::chrono::duration<float, std::milli>(pro_end - un_end).count();
  r_debug_info_ += fmt::format("[pro: {:.2f}]", pro_duration);
  // 输出保护路径生成的时间
  RLOG(I, fmt::format("[RedundantLateralPlanner] [RedundantLateralProtection] {} ms", pro_duration));
  // 更新调试信息
  debug_info_ += "\n[SYNC_PLANNER_ACTIVE]";

  return Status::OK();
}

}  // namespace gpal::pnc::planning