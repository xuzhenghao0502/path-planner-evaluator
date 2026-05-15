/**
 * @file real_time_path_planner.cpp
 * @brief 实时路径规划核心模块，包含路径边界生成、优化决策、碰撞检测等功能
 * @struct RealTimePathPlannerConfig 路径规划配置参数
 * @var vehicle_config_ 车辆配置参数
 * @var path_bound_points_config_ 路径边界点配置
 * @uml{flow}
 * start
 * :初始化配置参数;
 * :创建路径边界解析器;
 * :初始化各类型滤波器;
 * :加载障碍物距离映射表;
 * end
 * @enduml
 */

#include "path_planner/real_time_path_planner.h"
#include "config_manager/config_manager.h"
#include "base/singleton.h"
#include <fmt/chrono.h>

namespace gpal::pnc::planning {

/**
 * @brief 初始化路径规划器
 * @details 该函数负责初始化路径规划器的所有核心组件，包括加载配置参数、初始化路径边界解析器、滤波器、障碍物距离映射表以及优化器等。
 * 
 * @par 输入参数说明:
 * - 无显式输入参数，依赖全局配置管理器
 * 
 * @par 关键变量说明:
 * - vehicle_config_ (VehicleConfig): 车辆配置参数，包含车辆尺寸、动力学参数等
 * - real_time_path_planner_config_ (RealTimePathPlannerConfig): 实时路径规划配置
 * - path_bound_points_config_ (PathBoundPointsConfig): 路径边界点配置
 * - bound_parser_ (PathBoundParser): 路径边界解析器
 * - prior_physical_barrier_bound_filter_ (PathBoundFilter): 物理障碍物硬边界滤波器
 * - freespace_barrier_bound_filter_ (PathBoundFilter): 自由空间硬边界滤波器
 * - static_obstacle_barrier_bound_filter_ (PathBoundFilter): 静态障碍物硬边界滤波器
 * - obs_type_lateral_distance_map_ (std::map<ObsType, ObsInfo>): 障碍物类型与横向距离的映射表
 * - optimizer_ (PathOptimizer): 路径优化器
 * 
 * @par 初始化流程:
 * 1. 加载车辆配置
 * 2. 加载路径规划配置
 * 3. 初始化边界解析器
 * 4. 初始化各类型滤波器
 * 5. 加载障碍物距离映射表
 * 6. 初始化优化器
 * 
 * @par 流程图:
 * @startuml
 * start
 * :加载车辆配置;
 * :加载路径规划配置;
 * :初始化边界解析器;
 * :初始化各类型滤波器;
 * :加载障碍物距离映射表;
 * :初始化优化器;
 * end
 * @enduml
 * 
 * @return bool 初始化是否成功
 * @retval true 初始化成功
 * @retval false 初始化失败
 * 
 * @note 该函数应在系统启动时调用，确保所有组件正确初始化
 * 
 * @warning 需确保ConfigManager已正确初始化
 */
bool RealTimePathPlanner::init() {
  auto config_manager = Singleton<ConfigManager>::get_instance();
  vehicle_config_ = std::make_shared<VehicleConfig>(config_manager->vehicle_config());
  real_time_path_planner_config_ = config_manager->getConfig<RealTimePathPlannerConfig>("RealTimePathPlannerConfig");
  path_bound_points_config_ = real_time_path_planner_config_.path_bound_points_config();

  // bound_parser_
  auto path_bound_parser_config = config_manager->getConfig<PathBoundParserConfig>("PathBoundParserConfig");
  path_bound_parser_config.set_max_range(path_bound_points_config_.max_road_width_right() +
                                         0.5 * vehicle_config_->vehicle_param().width());
  bound_parser_ = std::make_unique<PathBoundParser>(path_bound_parser_config);

  // filter
  PathBoundFilterConfig prior_physical_filter_config;
  prior_physical_filter_config.set_filter_in(path_bound_points_config_.prior_physical_filter_in());
  prior_physical_filter_config.set_filter_out(path_bound_points_config_.prior_physical_filter_out());
  PathBoundFilterConfig fs_filter_config;
  fs_filter_config.set_filter_in(path_bound_points_config_.fs_filter_in());
  fs_filter_config.set_filter_out(path_bound_points_config_.fs_filter_out());
  PathBoundFilterConfig od_filter_config;
  od_filter_config.set_filter_in(path_bound_points_config_.od_filter_in());
  od_filter_config.set_filter_out(path_bound_points_config_.od_filter_out());

  prior_physical_barrier_bound_filter_ = std::make_unique<PathBoundFilter>(prior_physical_filter_config);
  prior_physical_soft_bound_filter_ = std::make_unique<PathBoundFilter>(prior_physical_filter_config);
  freespace_barrier_bound_filter_ = std::make_unique<PathBoundFilter>(fs_filter_config);
  freespace_soft_bound_filter_ = std::make_unique<PathBoundFilter>(fs_filter_config);
  static_obstacle_barrier_bound_filter_ = std::make_unique<PathBoundFilter>(od_filter_config);
  static_obstacle_soft_bound_filter_ = std::make_unique<PathBoundFilter>(od_filter_config);
  dynamic_obstacle_soft_bound_filter_ = std::make_unique<PathBoundFilter>(od_filter_config);

  // type_lateral_dist_pairs
  obs_type_lateral_distance_map_.clear();
  for (const auto& type_dist_pair : path_bound_points_config_.type_lateral_dist_pairs().type_lateral_dist_pair()) {
    obs_type_lateral_distance_map_.emplace(type_dist_pair.obs_type(), type_dist_pair.obs_info());
  }

  // optimizer_
  optimizer_.init();

  return true;
}

/**
 * @brief 重置路径规划器状态
 * @details 该函数用于重置路径规划器的内部状态，清除调试信息、重置参考线相关状态，为新的规划周期做准备。
 *          通常在路径规划器需要重新初始化或开始新的规划周期时调用。
 * 
 * @par 输入参数说明:
 * - 无显式输入参数
 * 
 * @par 关键变量说明:
 * - debug_status_ (std::vector<PathData::DebugStatusType>): 调试状态集合，记录路径规划过程中的调试状态
 * - debug_info_ (std::string): 调试信息字符串，记录路径规划过程中的调试信息
 * - pre_ref_id_ (std::string): 前次参考线ID，用于判断参考线是否发生变化
 * - pre_lane_id_ (std::vector<std::string>): 前次车道ID集合，用于判断车道是否发生变化
 * - pre_smooth_type_ (ReferenceLine::SmoothType): 前次参考线平滑类型，用于判断参考线平滑类型是否发生变化
 * - pre_line_type_ (ReferenceLine::LineType): 前次参考线类型，用于判断参考线类型是否发生变化
 * - prev_global_start_s_ (double): 前次全局起始点S值，范围[0, reference_line.length]
 * - prev_start_pt_ (math::Vec3d): 前次起始点坐标，包含x、y、z三个维度
 * 
 * @par 重置流程:
 * 1. 清除调试信息
 * 2. 重置参考线相关状态
 * 3. 重置全局起始点
 * 
 * @par 流程图:
 * @startuml
 * start
 * :清除调试信息;
 * :重置参考线相关状态;
 * :重置全局起始点;
 * end
 * @enduml
 * 
 * @return bool 重置是否成功
 * @retval true 重置成功
 * @retval false 重置失败
 * 
 * @note 该函数应在每次规划周期开始时调用，确保路径规划器的状态正确重置
 * 
 * @warning 需确保在调用该函数前，路径规划器的其他组件已正确初始化
 */
bool RealTimePathPlanner::reset() {
  debug_status_.clear();
  debug_info_.clear();
  pre_ref_id_ = "";
  pre_lane_id_.clear();
  pre_smooth_type_ = ReferenceLine::SmoothType::RAW;
  pre_line_type_ = ReferenceLine::LineType::UNKNOWN;
  prev_global_start_s_ = 0.0;
  prev_start_pt_ = math::Vec3d(0.0, 0.0, 0.0);

  return true;
}

/**
 * @brief 路径规划主循环，执行一次完整的路径规划过程
 * @details 该函数是路径规划的核心逻辑，负责处理传感器数据、生成路径边界、执行优化计算、碰撞检测等任务。
 *          根据不同的决策行为（如车道变换、强制回退等）生成相应的路径边界，并通过优化器计算最终路径。
 * 
 * @param[in] target_reference_line_info 目标参考线信息，包含目标车道的几何信息和属性
 * @param[in] current_reference_line_info 当前参考线信息，包含当前车道的几何信息和属性
 * @param[in] local_view 局部环境感知数据，包含车辆位姿、道路结构、障碍物等信息
 * @param[in] decision_result 决策模块输出结果，包含横向边界决策、障碍物决策等
 * @param[in] prev_speed_data 历史速度数据，用于路径规划的连续性
 * @param[in] stage_state 规划阶段状态，0-初始化，5-正常规划
 * @param[in] time_stamp 当前时间戳，用于时间同步
 * @param[out] path_boundary 生成的路径边界，包含硬边界和软边界
 * @param[out] path_data 最终路径数据，包含路径点、速度、加速度等信息
 * 
 * @return PathData::StatusType 路径状态码
 * @retval RUNNING 正常执行
 * 
 * @par 关键变量说明:
 * - stage_state_ (int): 规划阶段状态，范围[0, 5]
 * - adc_frenet_s_ (double): 自车Frenet坐标S值，范围[0, reference_line.length]
 * - behavior_ (FsmState): 当前决策行为，如车道变换、强制回退等
 * - debug_status_ (std::vector<PathData::DebugStatusType>): 调试状态集合
 * - debug_info_ (std::string): 调试信息字符串
 * 
 * @par 判断条件:
 * - 横向误差阈值: replan_lateral_error_thresold (默认0.3米)
 * - 航向角阈值: replan_heading_thresold (默认5度)
 * 
 * @par 流程图:
 * @startuml
 * start
 * :预处理传感器数据;
 * :生成斐波那契采样序列;
 * if (禁用nudge功能?) then
 *   :直接加载参考线;
 * else
 *   if (车道变换?) then
 *     :生成车道变换路径边界;
 *   else if (强制回退?) then
 *     :生成强制回退路径边界;
 *   else
 *     :生成常规路径边界;
 *   endif
 *   :执行优化器计算;
 * endif
 * :碰撞后处理;
 * :可视化调试;
 * end
 * @enduml
 * 
 * @note 包含多种场景处理逻辑:
 * 1. 非里程计场景下：根据感知车道偏差切换
 * 2. 里程计场景下：强制使用局部路径参考线
 * 
 * @warning 需确保reference_lines_已通过processReferenceLine()更新
 */
PathData::StatusType RealTimePathPlanner::runOnce(const ReferenceLineInfo& target_reference_line_info,
                                                  const ReferenceLineInfo& current_reference_line_info,
                                                  const LocalView& local_view,
                                                  const DecisionResult& decision_result, 
                                                  const SpeedData& prev_speed_data,
                                                  const int& stage_state,
                                                  const int64_t& time_stamp, 
                                                  PathBoundary* path_boundary,
                                                  PathData* const path_data) {
  auto t1 = std::chrono::steady_clock::now();
  // 预处理.
  stage_state_ = stage_state;
  preProcess(target_reference_line_info, current_reference_line_info, *local_view.getLocalizationPtr(), *local_view.getChassisPtr(), decision_result, prev_speed_data);
  path_boundary->reset();

// 获取指定时间范围内(最好与决策保持一致)内的斐波那契adc_ts_ranges vector，包括每项包含自车的start_t end_t start_s end_s
  generateFibonacciCVTSRange(0.0, path_bound_points_config_.adc_t_range(), 0.1);

  // nudge功能抑制，截取参考线作为横向规划输出，可用于初版算法的链路调试.
  if (!real_time_path_planner_config_.enable_nudge()) {
    // ERT_PLOG_D << "[RealTimePathPlanner::runOnce]: nudge disabled, load reference line" ;
    PathPlannerBase::runLoadRefLine(target_ref_line_info_->ref_line(), planning_start_point_, path_data);
    path_data->mutablePlannerDebugStatus()->push_back(PathData::DebugStatusType::NUDGE_DISABLED_REF);
    return PathData::StatusType::RUNNING;
  }
  auto t2 = std::chrono::steady_clock::now();
  auto duration1 = std::chrono::duration<float, std::milli>(t2 - t1).count();
  ERT_PLOG_D << "[PathPlanTimeCost] [RealTimePreprocess] " << duration1 << " ms";

  // 不同行为决策及场景下的横向边界生成.
  bool is_lane_change = (behavior_ == FsmState::LEFT_CHANGE || behavior_ == FsmState::RIGHT_CHANGE);
  bool is_lane_change_return = (behavior_ == FsmState::LEFT_RETURN || behavior_ == FsmState::RIGHT_RETURN);
  bool is_lane_change_hold = (behavior_ == FsmState::LEFT_HOLD || behavior_ == FsmState::RIGHT_HOLD);
  bool is_lane_change_attempt = (behavior_ == FsmState::LEFT_ATTEMPT || behavior_ == FsmState::RIGHT_ATTEMPT);
  Status status;
  if (is_lane_change) {
    target_ref_line_info_->mutable_path_boundary()->set_label("lane_change");
    debug_status_.emplace(PathData::DebugStatusType::LANE_CHANGE);
    status = generateLaneChangePathBound(
        *local_view.getConsolePtr(), *local_view.getFreespacePtr(),
                                         *target_ref_line_info_, decision_result.getLateralBoundaryDecision(),
                                         decision_result.getLongitudinalBoundaryDecision(),
                                         target_ref_line_info_->mutable_path_boundary());
  } else if (is_lane_change_return) {
    target_ref_line_info_->mutable_path_boundary()->set_label("lane_change_return");
    debug_status_.emplace(PathData::DebugStatusType::LANE_CHANGE_RETURN);
    status = generateLaneChangePathBound(
        *local_view.getConsolePtr(), *local_view.getFreespacePtr(),
                                         *target_ref_line_info_, decision_result.getLateralBoundaryDecision(),
                                         decision_result.getLongitudinalBoundaryDecision(),
                                         target_ref_line_info_->mutable_path_boundary());
  } else {
    if (PathOptimizer::isInIgnoreRange(ignore_ranges_, adc_frenet_s_)) {
      target_ref_line_info_->mutable_path_boundary()->set_label("force_back");
      debug_status_.emplace(PathData::DebugStatusType::IGNORE_RANGE_REF);
      debug_status_.emplace(PathData::DebugStatusType::FORCE_BACK);
      status = generateForceBackPathBound(
          *target_ref_line_info_, decision_result.getLateralBoundaryDecision(),
                                          target_ref_line_info_->mutable_path_boundary());
    } else {
      target_ref_line_info_->mutable_path_boundary()->set_label("regular");
      debug_status_.emplace(PathData::DebugStatusType::REGULAR);
      status = generateRegularPathBound(
          *local_view.getConsolePtr(), *local_view.getFreespacePtr(),
                                        *target_ref_line_info_, decision_result.getLateralBoundaryDecision(),
                                        decision_result.getLongitudinalBoundaryDecision(),
                                        target_ref_line_info_->mutable_path_boundary());
    }
  }
  if (!status.ok()) {
    ERT_PLOG_W << status.error_message() ;
    target_ref_line_info_->mutable_path_boundary()->set_label("reference_line");
  } else {
    //todo make sure force_tail_length is right ?
    target_ref_line_info_->mutable_path_boundary()->trim(real_time_path_planner_config_.force_tail_length());
  }
  updateRefOffsetInfo(decision_result,((is_lane_change_hold || is_lane_change_attempt) && l_offset_behavior_valid_));
  // print info
  ERT_PLOG_D << "[bypass]: [RealTimePathPlanner::runOnce]: final: start_s = " << target_ref_line_info_->path_boundary().start_s()
            << "  end_s = " << target_ref_line_info_->path_boundary().end_s()
            << "  size = " << target_ref_line_info_->path_boundary().size()
            << "  delta_s = " << target_ref_line_info_->path_boundary().delta_s()
            << "  length = " << target_ref_line_info_->path_boundary().length() ;
  ERT_PLOG_D << "[bypass]: [RealTimePathPlanner::runOnce]: final: barrier_size = " << target_ref_line_info_->path_boundary().barrier_boundary().size()
            << "  soft_size = " << target_ref_line_info_->path_boundary().soft_boundary().size()
            ;
  // for(int i = 0; i < std::min<int>(20, target_ref_line_info_->path_boundary().barrier_boundary().size()); ++i) {
  //   ERT_PLOG_D << "[bypass]: i = " << i ;
  //   ERT_PLOG_D << "   [bypass]: barrier: s = " << std::get<0>(target_ref_line_info_->path_boundary().barrier_boundary().at(i))
  //             << "   l_min = " << std::get<1>(target_ref_line_info_->path_boundary().barrier_boundary().at(i))
  //             << "   l_max = " << std::get<2>(target_ref_line_info_->path_boundary().barrier_boundary().at(i)) ;
  //   ERT_PLOG_D << "   [bypass]: soft: s = " << std::get<0>(target_ref_line_info_->path_boundary().soft_boundary().at(i))
  //             << "   l_min = " << std::get<1>(target_ref_line_info_->path_boundary().soft_boundary().at(i))
  //             << "   l_max = " << std::get<2>(target_ref_line_info_->path_boundary().soft_boundary().at(i)) ;
  // }
  auto t3 = std::chrono::steady_clock::now();
  auto duration2 = std::chrono::duration<float, std::milli>(t3 - t2).count();
  ERT_PLOG_D << "[PathPlanTimeCost] [RealTimeBoundParser] " << duration2 << " ms";

  // lane_keep_start_s生成.
  calcLaneKeepStartS(decision_result, path_data);
  optimizer_.setIsLaneChange(is_lane_change);
  // ocp优化过程.
  status = processPathOptimizer(decision_result, prev_speed_data, time_stamp, path_data);
  last_optimizer_status_ = status;
  pre_change_ref_ = change_ref_;
  if (!status.ok()) {
    ERT_PLOG_W << "[RealTimePathPlanner::runOnce]: path optimizer failed" ;
  }

  //add path boundary info base lateral plan result
  auto path_barrier_boundary_info =  target_ref_line_info_->mutable_path_boundary()->mutable_path_barrier_boundary_info();
  generateRealtimeTrajBoundaryInfo(path_barrier_boundary_info, path_data);

  *path_boundary = target_ref_line_info_->path_boundary();
  auto t4 = std::chrono::steady_clock::now();
  auto duration3 = std::chrono::duration<float, std::milli>(t4 - t3).count();
  ERT_PLOG_D << "[PathPlanTimeCost] [RealTimePathOptimizer] " << duration3 << " ms";
  // 基于fs的碰撞检测
  CollisionPostProcess(*local_view.getFreespacePtr(), local_view.getLocalizationPtr(), path_data);
  auto t5 = std::chrono::steady_clock::now();
  auto duration4 = std::chrono::duration<float, std::milli>(t5 - t4).count();
  ERT_PLOG_D << "[PathPlanTimeCost] [RealTimeCollisionPostProcess] " << duration4 << " ms";

  // debug info.
  path_data->mutableDebugInfo()->append(debug_info_);
  for (const auto& it : debug_status_) {
    path_data->mutablePlannerDebugStatus()->push_back(it);
  }

  // display
  visualization(path_data);
  auto t6 = std::chrono::steady_clock::now();
  auto duration5 = std::chrono::duration<float, std::milli>(t6 - t5).count();
  ERT_PLOG_D << "[PathPlanTimeCost] [RealTimeVisualization] " << duration5 << " ms";
  ERT_PLOG_D << "[PathPlanTimeCost] [PathPlanner] " << std::chrono::duration<float, std::milli>(t6 - t1).count() << " ms";

  return PathData::StatusType::RUNNING;
}


/**
 * @brief 预处理函数，准备路径规划所需的数据
 * @details 该函数负责在路径规划主循环开始前，对输入数据进行预处理，包括初始化参考线信息、更新车辆状态、计算Frenet坐标等。
 * 
 * @param[in] target_reference_line_info 目标参考线信息，包含目标车道的几何信息和属性
 * @param[in] current_reference_line_info 当前参考线信息，包含当前车道的几何信息和属性
 * @param[in] local_view 局部环境感知数据，包含车辆位姿、道路结构、障碍物等信息
 * @param[in] decision_result 决策模块输出结果，包含横向边界决策、障碍物决策等
 * @param[in] prev_speed_data 历史速度数据，用于路径规划的连续性
 * 
 * @par 关键变量说明:
 * - target_ref_line_info_ (ReferenceLineInfo): 目标参考线信息
 * - current_ref_line_info_ (ReferenceLineInfo): 当前参考线信息
 * - planning_start_point_ (PathPoint): 规划起始点
 * - adc_width_ (double): 车辆宽度
 * - adc_wheel_width_ (double): 车辆轮距
 * - adc_front_length_ (double): 车辆前悬长度
 * - adc_rear_length_ (double): 车辆后悬长度
 * - adc_sl_info_ (std::pair<math::Vec3d, math::Vec3d>): 自车Frenet坐标信息
 * - adc_frenet_s_ (double): 自车Frenet坐标S值
 * - adc_frenet_l_ (double): 自车Frenet坐标L值
 * - adc_frenet_sd_ (double): 自车Frenet坐标S方向速度
 * - adc_frenet_end_s_ (double): 自车Frenet坐标S方向终点值
 * - behavior_ (FsmState): 当前决策行为
 * - debug_status_ (std::vector<PathData::DebugStatusType>): 调试状态集合
 * - debug_info_ (std::string): 调试信息字符串
 * 
 * @par 预处理流程:
 * 1. 初始化目标参考线和当前参考线信息
 * 2. 更新规划起始点
 * 3. 计算自车Frenet坐标
 * 4. 更新车辆状态信息
 * 5. 检查参考线是否变更
 * 6. 更新决策行为
 * 7. 处理静态和动态障碍物
 * 
 * @par 流程图:
 * @startuml
 * start
 * :初始化目标参考线和当前参考线信息;
 * :更新规划起始点;
 * :计算自车Frenet坐标;
 * :更新车辆状态信息;
 * :检查参考线是否变更;
 * :更新决策行为;
 * :处理静态和动态障碍物;
 * end
 * @enduml
 * 
 * @note 该函数应在每次路径规划循环开始时调用，确保所有数据正确初始化
 * 
 * @warning 需确保输入数据有效，特别是local_view和decision_result
 */
void RealTimePathPlanner::preProcess(const ReferenceLineInfo& target_reference_line_info, const ReferenceLineInfo& current_reference_line_info,
                                     const Localization& localization, const Chassis& chassis, const DecisionResult& decision_result, const SpeedData& prev_speed_data) {
  debug_status_.clear();
  debug_info_.clear();
  special_scene_s_range_sets_.clear();
  l_offset_behavior_valid_ = false;

  // get target_ref_line_info_ and current_ref_line_info_
  target_ref_line_info_ = std::make_unique<ReferenceLineInfo>(target_reference_line_info);
  current_ref_line_info_ = std::make_unique<ReferenceLineInfo>(current_reference_line_info);
  const auto& target_ref = target_ref_line_info_->ref_line();
  const auto& driven_ref = current_ref_line_info_->ref_line();

  // update planning_start_point_
  planning_start_point_ = target_ref_line_info_->adc_planning_point();
  if (real_time_path_planner_config_.reset_plan_start_point_with_loc_heading_when_stop() && chassis.Speed() < kSpeedEpsilon) {
    planning_start_point_.mutable_path_pt()->set_theta(localization.vehicleAlignPosePoint().yaw());
  }
  debug_info_ += fmt::format("\n front_steer = {:.5f} , theta = {:.5f}", planning_start_point_.path_pt().front_steer() * RAD2ANG, planning_start_point_.path_pt().theta()* RAD2ANG);
  // tf: map->ego
  tf_map_2_ego_ = localization.getTfMap2Ego();

  // adc info
  adc_width_ = vehicle_config_->vehicle_param().width();
  adc_wheel_width_ = vehicle_config_->vehicle_param().width_wheel();
  adc_front_length_ = vehicle_config_->vehicle_param().front_edge_to_ego();
  adc_rear_length_ = vehicle_config_->vehicle_param().rear_edge_to_ego();
  adc_sl_info_ = target_ref.toFrenetFrame(planning_start_point_);
  adc_frenet_s_ = std::fmin(adc_sl_info_.first[0], target_ref.length());
  adc_frenet_l_ = adc_sl_info_.second[0];
  adc_frenet_sd_ = std::max(kSpeedEpsilon, adc_sl_info_.first[1]);
  adc_frenet_end_s_ = std::fmin(adc_frenet_s_ + path_bound_points_config_.resolution() * path_bound_points_config_.horizon(),
                target_ref.length());
  bound_parser_->setAdcFrenetS(adc_frenet_s_);
  bound_parser_->setAdcFrenetSpeed(adc_frenet_sd_);
  bound_parser_->setAdcFrontLength(adc_front_length_);
  bound_parser_->setAdcRearLength(adc_rear_length_); 
  //add speed data
  std::vector<std::pair<double, double>> prev_speed_traj;
  if(!prev_speed_data.empty()){
    for(auto &speedpoint : prev_speed_data){
      prev_speed_traj.emplace_back(speedpoint.t(), speedpoint.s());
    }
  }
  bound_parser_->setTargetReferenceLine(target_reference_line_info.ref_line());
  bound_parser_->setAdcSpeedTraj(prev_speed_traj);
  auto adc_driven_sl_info = driven_ref.toFrenetFrame(planning_start_point_);
  adc_driven_frenet_s_ = std::fmin(adc_driven_sl_info.first[0], driven_ref.length());
  adc_driven_frenet_l_ = adc_driven_sl_info.second[0];

  // 基于自车当前位姿的后轴中心的左右边界：curr_right_bound_adc_, curr_left_bound_adc_
  constexpr double max_ldd = 0.2;
  double adc_lat_decel_buffer =
      (adc_sl_info_.second[1] > 0 ? 0.5 : -0.5) * adc_sl_info_.second[1] * adc_sl_info_.second[1] / max_ldd;  // buffer = v_lateral^v_lateral/2/a_lateral
  // dl > 0, left:l+fabs(buffer), right:l;  dl < 0, left:l, right:l-fabs(buffer)
  curr_right_bound_adc_ = std::fmin(adc_frenet_l_, adc_frenet_l_ + adc_lat_decel_buffer);
  curr_left_bound_adc_ = std::fmax(adc_frenet_l_, adc_frenet_l_ + adc_lat_decel_buffer);
  debug_info_ += fmt::format("\nadc_s: <{:.2f}, {:.2f}, {:.2f}> adc_l: <{:.2f}, {:.2f}, {:.2f}>", adc_sl_info_.first[0], adc_sl_info_.first[1], adc_sl_info_.first[2],
                                                                                                    adc_sl_info_.second[0], adc_sl_info_.second[1], adc_sl_info_.second[2]);

  // 检查参考线是否变更，若是则resetFilter
  std::pair<double, double> s_range = std::make_pair(adc_frenet_s_, adc_frenet_end_s_);
  change_ref_ = hasChangeReference(s_range, *target_ref_line_info_);
  target_ref_line_info_->mutable_ref_line()->set_has_change_reference(change_ref_);
  if(change_ref_) {
    //ERT_PLOG_D << "[RealTimePathPlanner::preProcess]: target reference line changed " ;
    resetFilter();
  }
  debug_info_ += fmt::format("\nchange_ref: {}", change_ref_);

  // 基于决策结果获取决策行为
  behavior_ = decision_result.getCurrFsmState();
  debug_info_ += fmt::format(", behavior: {}", static_cast<int>(behavior_));

  if (behavior_ == FsmState::LEFT_HOLD || behavior_ == FsmState::RIGHT_HOLD || 
      behavior_ == FsmState::LEFT_ATTEMPT || behavior_ == FsmState::RIGHT_ATTEMPT) {
    if (decision_result.getRefTrajInfo() != nullptr && decision_result.getRefTrajInfo()->use_ref_lateral &&
        !decision_result.getRefTrajInfo()->traj_points.empty()) {
      l_offset_behavior_valid_ = true;
    }
  }
  ref_offsets_info_.clear();
  // 基于决策结果筛选出所有打横向tag的od
  decision_static_obstacles_.clear();
  decision_dynamic_obstacles_.clear();
  for (const auto &[id, decision_od] : *decision_result.getOdDecisions()) {
    if(decision_od.lat_od_tag == LateralOdTag::LEFT_BYPASS
       || decision_od.lat_od_tag == LateralOdTag::RIGHT_BYPASS) {
      decision_static_obstacles_.emplace_back(make_shared<Decision::DecisionObject>(decision_od));
    } else if(decision_od.lat_od_tag == LateralOdTag::DYNAMIC_LEFT_BYPASS
             || decision_od.lat_od_tag == LateralOdTag::DYNAMIC_RIGHT_BYPASS) {
      decision_dynamic_obstacles_.emplace_back(make_shared<Decision::DecisionObject>(decision_od));
    }
  }

  //updateIgnoreRanges(decision_result.pathDecision().ignoreRange());

  path_blocked_idx_ = -1;
}

/**
 * @brief 判断参考线是否发生变化
 * @details 该函数用于判断当前参考线是否与上一次的参考线发生变化，主要根据参考线ID、平滑类型和参考线类型进行判断。
 * 
 * @param[in] s_range 参考线的S值范围，包含起始和结束S值
 * @param[in] ref_info 参考线信息，包含参考线的ID、平滑类型和参考线类型
 * 
 * @par 关键变量说明:
 * - pre_ref_id_ (std::string): 前次参考线ID，用于判断参考线是否发生变化
 * - pre_smooth_type_ (ReferenceLine::SmoothType): 前次参考线平滑类型，用于判断参考线平滑类型是否发生变化
 * - pre_line_type_ (ReferenceLine::LineType): 前次参考线类型，用于判断参考线类型是否发生变化
 * 
 * @par 判断逻辑:
 * 1. 如果参考线类型为MEMORIZED_ROUTE，则根据参考线ID、平滑类型和参考线类型进行判断
 * 2. 如果参考线类型为PERCEPTION_ROAD_STRUCTURE、PERCEPTION_LANE或LOCAL_ROUTE，则根据参考线ID、平滑类型和参考线类型进行判断
 * 
 * @par 流程图:
 * @startuml
 * start
 * :判断参考线类型;
 * if (参考线类型为MEMORIZED_ROUTE?) then
 *   :根据参考线ID、平滑类型和参考线类型判断是否变化;
 * else if (参考线类型为PERCEPTION_ROAD_STRUCTURE、PERCEPTION_LANE或LOCAL_ROUTE?) then
 *   :根据参考线ID、平滑类型和参考线类型判断是否变化;
 * endif
 * :更新前次参考线信息;
 * :返回是否变化;
 * end
 * @enduml
 * 
 * @return bool 参考线是否发生变化
 * @retval true 参考线发生变化
 * @retval false 参考线未发生变化
 * 
 * @note 该函数应在每次规划周期开始时调用，确保参考线状态正确更新
 * 
 * @warning 需确保输入参数有效，特别是ref_info
 */
bool RealTimePathPlanner::hasChangeReference(const std::pair<double, double>& s_range, const ReferenceLineInfo& ref_info) {
  auto isReferenceLineIdChanged = [](const std::string& last_ref_id, const std::string& current_ref_id) {
    if(last_ref_id == current_ref_id) {
      return false;
    }
    if(last_ref_id.find("left") != std::string::npos && current_ref_id.find("left") != std::string::npos) {
      return false;
    }
    if(last_ref_id.find("right") != std::string::npos && current_ref_id.find("right") != std::string::npos) {
      return false;
    }
    if(last_ref_id.find("left") == std::string::npos && current_ref_id.find("left") == std::string::npos
       && last_ref_id.find("right") == std::string::npos && current_ref_id.find("right") == std::string::npos) {
      return false;
    }
    return true;
  };
  if(ref_info.ref_line().line_type() == ReferenceLine::LineType::MEMORIZED_ROUTE) { 
    if (isReferenceLineIdChanged(pre_ref_id_, ref_info.ref_line().id()) 
        || ref_info.ref_line().smooth_type() != pre_smooth_type_
        || ref_info.ref_line().line_type() != pre_line_type_) {
      pre_ref_id_ = ref_info.ref_line().id();
      pre_smooth_type_ = ref_info.ref_line().smooth_type();
      pre_line_type_ = ref_info.ref_line().line_type();
      return true;
    }
  } else if(ref_info.ref_line().line_type() == ReferenceLine::LineType::PERCEPTION_ROAD_STRUCTURE
           || ref_info.ref_line().line_type() == ReferenceLine::LineType::PERCEPTION_LANE
           || ref_info.ref_line().line_type() == ReferenceLine::LineType::LOCAL_ROUTE) {
    if (ref_info.ref_line().id() != pre_ref_id_ 
        || ref_info.ref_line().smooth_type() != pre_smooth_type_
        || ref_info.ref_line().line_type() != pre_line_type_) {
      pre_ref_id_ = ref_info.ref_line().id();
      pre_smooth_type_ = ref_info.ref_line().smooth_type();
      pre_line_type_ = ref_info.ref_line().line_type();
      return true;
    }
  }
  pre_ref_id_ = ref_info.ref_line().id();
  pre_smooth_type_ = ref_info.ref_line().smooth_type();
  pre_line_type_ = ref_info.ref_line().line_type();
  return false;
}

/**
 * @brief 重置滤波器状态
 * @details 该函数用于重置路径规划器中所有滤波器的状态，确保在参考线发生变化或规划周期重新开始时，滤波器能够重新初始化。
 * 
 * @par 关键变量说明:
 * - prior_physical_barrier_bound_filter_ (PathBoundFilter): 物理障碍物硬边界滤波器
 * - prior_physical_soft_bound_filter_ (PathBoundFilter): 物理障碍物软边界滤波器
 * - freespace_barrier_bound_filter_ (PathBoundFilter): 自由空间硬边界滤波器
 * - freespace_soft_bound_filter_ (PathBoundFilter): 自由空间软边界滤波器
 * - static_obstacle_barrier_bound_filter_ (PathBoundFilter): 静态障碍物硬边界滤波器
 * - static_obstacle_soft_bound_filter_ (PathBoundFilter): 静态障碍物软边界滤波器
 * - dynamic_obstacle_soft_bound_filter_ (PathBoundFilter): 动态障碍物软边界滤波器
 * 
 * @par 重置流程:
 * 1. 重置物理障碍物硬边界滤波器
 * 2. 重置物理障碍物软边界滤波器
 * 3. 重置自由空间硬边界滤波器
 * 4. 重置自由空间软边界滤波器
 * 5. 重置静态障碍物硬边界滤波器
 * 6. 重置静态障碍物软边界滤波器
 * 7. 重置动态障碍物软边界滤波器
 * 
 * @par 流程图:
 * @startuml
 * start
 * :重置物理障碍物硬边界滤波器;
 * :重置物理障碍物软边界滤波器;
 * :重置自由空间硬边界滤波器;
 * :重置自由空间软边界滤波器;
 * :重置静态障碍物硬边界滤波器;
 * :重置静态障碍物软边界滤波器;
 * :重置动态障碍物软边界滤波器;
 * end
 * @enduml
 * 
 * @note 该函数应在参考线发生变化或规划周期重新开始时调用，确保滤波器状态正确重置
 * 
 * @warning 需确保所有滤波器已正确初始化
 */
void RealTimePathPlanner::resetFilter() {
  if (prior_physical_barrier_bound_filter_) {
    prior_physical_barrier_bound_filter_->reset();
  }
  if (prior_physical_soft_bound_filter_) {
    prior_physical_soft_bound_filter_->reset();
  }
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
  if (dynamic_obstacle_soft_bound_filter_) {
    dynamic_obstacle_soft_bound_filter_->reset();
  }
}

/**
 * @brief 更新忽略范围信息
 * @details 该函数用于更新路径规划中的忽略范围信息，通常用于处理特定场景下的路径规划限制，例如闸机、施工区域等。
 * 
 * @param[in] ignore_ranges 忽略范围信息列表，包含每个忽略范围的起始和结束S值以及类型
 * 
 * @par 关键变量说明:
 * - ignore_ranges_ (std::vector<IgnoreRangeInfo>): 忽略范围信息列表，包含每个忽略范围的起始和结束S值以及类型
 * - gate_ranges_ (std::vector<std::pair<double, double>>): 闸机范围列表，包含每个闸机范围的起始和结束S值
 * - debug_info_ (std::string): 调试信息字符串，记录路径规划过程中的调试信息
 * 
 * @par 更新流程:
 * 1. 更新忽略范围信息列表
 * 2. 清空闸机范围列表
 * 3. 遍历忽略范围信息列表，将闸机范围添加到闸机范围列表中
 * 4. 将忽略范围信息添加到调试信息中
 * 
 * @par 流程图:
 * @startuml
 * start
 * :更新忽略范围信息列表;
 * :清空闸机范围列表;
 * :遍历忽略范围信息列表;
 * if (忽略范围类型为闸机?) then
 *   :将闸机范围添加到闸机范围列表中;
 * endif
 * :将忽略范围信息添加到调试信息中;
 * end
 * @enduml
 * 
 * @note 该函数应在每次路径规划循环开始时调用，确保忽略范围信息正确更新
 * 
 * @warning 需确保输入参数有效，特别是ignore_ranges
 */
void RealTimePathPlanner::updateIgnoreRanges(const std::vector<IgnoreRangeInfo>& ignore_ranges) {
  ignore_ranges_ = ignore_ranges;
  gate_ranges_.clear();
  for (const auto& range : ignore_ranges_) {
    debug_info_ += fmt::format("\nignore range {}: <{:5.2f}, {:5.2f}> {:5.2f}", static_cast<int>(range.type), range.start_s, range.end_s, adc_frenet_s_);
    if (range.type == IgnoreRangeInfo::GATE) {
      gate_ranges_.emplace_back(range.start_s, range.end_s);
    }
  }
}

/**
 * @brief 生成斐波那契采样序列的时间-空间范围
 * @details 该函数用于生成自车在指定时间范围内的斐波那契采样序列，包含每个时间段的起始时间、结束时间、起始S值和结束S值。
 *          这些信息用于路径规划中的时间-空间采样，确保路径规划的连续性和平滑性。
 * 
 * @param[in] t0 起始时间，单位为秒
 * @param[in] t1 结束时间，单位为秒
 * @param[in] dt 时间步长，单位为秒
 * 
 * @par 关键变量说明:
 * - adc_ts_ranges (std::vector<std::tuple<double, double, double, double>>): 时间-空间范围列表，包含每个时间段的起始时间、结束时间、起始S值和结束S值
 * - adc_frenet_s_ (double): 自车Frenet坐标S值，范围[0, reference_line.length]
 * - adc_frenet_sd_ (double): 自车Frenet坐标S方向速度，单位为米/秒
 * 
 * @par 生成流程:
 * 1. 初始化时间-空间范围列表
 * 2. 遍历时间范围，生成每个时间段的起始时间、结束时间、起始S值和结束S值
 * 3. 将生成的时间-空间范围列表设置到bound_parser_中
 * 
 * @par 流程图:
 * @startuml
 * start
 * :初始化时间-空间范围列表;
 * :遍历时间范围;
 * :生成每个时间段的起始时间、结束时间、起始S值和结束S值;
 * :将时间-空间范围列表设置到bound_parser_中;
 * end
 * @enduml
 * 
 * @return Status 执行状态
 * @retval OK 成功
 * 
 * @note 该函数应在每次路径规划循环开始时调用，确保时间-空间范围正确生成
 * 
 * @warning 需确保输入参数有效，特别是t0、t1和dt
 */
Status RealTimePathPlanner::generateFibonacciCVTSRange(const double& t0, const double& t1, const double& dt) {
  std::vector<std::tuple<double, double, double, double>> adc_ts_ranges;  // start_t end_t start_s end_s
  // std::vector<double> dt_vec = getFibonacciVec(dt, t1 - t0);
  // double prev_s = adc_frenet_s_ + adc_frenet_sd_ * t0;
  // for (double i = 1; i < dt_vec.size(); i++) {
  //   const double tl = t0 + dt_vec[i - 1];
  //   const double tu = t0 + dt_vec[i];
  //   double sk = adc_frenet_s_ + adc_frenet_sd_ * tu;
  //   adc_ts_ranges.emplace_back(tl, tu, prev_s, sk);
  //   prev_s = sk;
  // }

  double prev_s = adc_frenet_s_ + adc_frenet_sd_ * t0;
  for (double t = t0; t + dt <= t1; t += dt) {
    double sk = adc_frenet_s_ + adc_frenet_sd_ * (t + dt);
    adc_ts_ranges.emplace_back(t, t + dt, prev_s, sk);
    prev_s = sk;
  }

  bound_parser_->setAdcTSRange(std::move(adc_ts_ranges));
  return Status::OK();
}

/**
 * @brief 生成斐波那契序列
 * @details 该函数用于生成一个斐波那契序列，序列中的每个元素表示时间步长，用于路径规划中的时间采样。序列的生成基于初始时间步长和最大时间范围，确保序列中的元素不超过最大时间范围。
 * 
 * @param[in] dt 初始时间步长，单位为秒
 * @param[in] tmax 最大时间范围，单位为秒
 * @param[in] n_max 序列的最大长度
 * 
 * @par 关键变量说明:
 * - res (std::vector<double>): 生成的斐波那契序列，包含时间步长
 * - idx (int): 当前序列的索引，用于生成下一个斐波那契数
 * 
 * @par 生成流程:
 * 1. 初始化序列，包含初始值0.0
 * 2. 如果初始时间步长小于最大时间范围，则生成斐波那契序列
 * 3. 将初始时间步长和两倍初始时间步长添加到序列中
 * 4. 循环生成斐波那契数，直到序列长度达到最大长度或序列中的最后一个元素超过最大时间范围
 * 5. 如果序列中的最后一个元素超过最大时间范围，则将其设置为最大时间范围
 * 6. 如果初始时间步长大于或等于最大时间范围，则直接将最大时间范围添加到序列中
 * 
 * @par 流程图:
 * @startuml
 * start
 * :初始化序列，包含初始值0.0;
 * if (初始时间步长 < 最大时间范围?) then
 *   :将初始时间步长和两倍初始时间步长添加到序列中;
 *   :循环生成斐波那契数;
 *   if (序列长度 < 最大长度 且 最后一个元素 < 最大时间范围?) then
 *     :生成下一个斐波那契数并添加到序列中;
 *   endif
 *   :如果最后一个元素 > 最大时间范围，则将其设置为最大时间范围;
 * else
 *   :将最大时间范围添加到序列中;
 * endif
 * end
 * @enduml
 * 
 * @return std::vector<double> 生成的斐波那契序列
 * 
 * @note 该函数用于生成时间采样序列，确保路径规划中的时间采样点分布合理
 * 
 * @warning 需确保输入参数有效，特别是dt和tmax
 */
std::vector<double> RealTimePathPlanner::getFibonacciVec(const double& dt, const double& tmax, const size_t n_max) {
  std::vector<double> res(1, 0.0);
  if (dt < tmax) {
    res.emplace_back(dt);
    res.emplace_back(2 * dt);
    int idx = 1;
    while (res.size() < n_max && res.back() < tmax) {
      res.emplace_back(res[idx] + res[idx + 1]);
      idx++;
    }
    if (!res.empty() && res.back() > tmax) {
      res.back() = tmax;
    }
  } else {
    res.emplace_back(tmax);
  }
  return res;
}

/**
 * @brief 生成常规路径边界
 * @details 该函数用于生成常规路径规划的边界信息，包括硬边界、软边界和决策边界。这些边界信息用于路径优化和碰撞检测，确保路径的安全性和可行性。
 * 
 * @param[in] console 控制台输入数据，包含车辆状态、环境感知等信息
 * @param[in] freespace 自由空间数据，包含可行驶区域的信息
 * @param[in] reference_line_info 参考线信息，包含参考线的几何信息和属性
 * @param[in] decision_boundary 决策边界，包含横向边界决策信息
 * @param[out] boundary 生成的路径边界，包含硬边界、软边界和决策边界
 * 
 * @par 关键变量说明:
 * - barrier_boundary (PathBoundary::BoundaryType): 硬边界，表示车辆不可逾越的边界
 * - soft_boundary (PathBoundary::BoundaryType): 软边界，表示车辆应尽量避免的边界
 * - decision_boundary (PathBoundary::BoundaryType): 决策边界，表示基于决策模块生成的边界
 * - key_obstacles (std::unordered_map<double, PathBoundary::ObstacleInfo>): 关键障碍物信息，包含每个S值对应的最近障碍物
 * 
 * @par 生成流程:
 * 1. 初始化路径边界数据结构
 * 2. 基于自由空间数据更新硬边界和软边界
 * 3. 处理静态障碍物边界
 * 4. 处理动态障碍物边界
 * 5. 应用特殊场景修正
 * 6. 最终整理路径边界信息
 * 
 * @par 流程图:
 * @startuml
 * start
 * :初始化路径边界数据结构;
 * :基于自由空间数据更新硬边界和软边界;
 * :处理静态障碍物边界;
 * :处理动态障碍物边界;
 * :应用特殊场景修正;
 * :最终整理路径边界信息;
 * end
 * @enduml
 * 
 * @return Status 执行状态
 * @retval OK 成功
 * 
 * @note 该函数应在每次路径规划循环开始时调用，确保路径边界信息正确生成
 * 
 * @warning 需确保输入参数有效，特别是reference_line_info和decision_boundary
 */
Status RealTimePathPlanner::generateRegularPathBound(const Console& console,
                                                     const Freespace& freespace, 
                                                     const ReferenceLineInfo& reference_line_info,
                                                     const LateralBoundDecision& decision_boundary,
                                                     const LongitudinalBoundDecision& longitudinal_decision_boundary,
                                                     PathBoundary* boundary) {
  // 横向各边界信息的初始化:
  // decision_boundary: 决策边界，采样原决策边界值（车尾起始，已去除半车宽）.
  // barrier_boundary: 硬边界，来源于decision_boundary（车尾起始，已去除半车宽）.
  // soft_boundary: 软边界，考虑max_allowed_soft_bounds（车尾起始，已去除半车宽）.
  // key_obstacles: 各s处对应的左右最近Obs的id.
  if (!initPathBoundary(reference_line_info, decision_boundary, longitudinal_decision_boundary, boundary)) {
    const std::string msg = "[RealTimePathPlanner::generateRegularPathBound]: initPathBoundary failed.";
    ERT_PLOG_W << msg ;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // 基于fs的横向硬边界barrier_boundary、软边界soft_boundary更新.
  if (real_time_path_planner_config_.enable_freespace_nudge() && stage_state_ == 5) {
    if (!calcBoundaryFromFreespace(freespace, boundary)) {
      const std::string msg = "[RealTimePathPlanner::generateRegularPathBound]: calcBoundaryFromFreespace failed";
      ERT_PLOG_W << msg ;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
  }

  // 静态绕行: 基于决策nudge
  // od的横向硬边界barrier_boundary、软边界soft_boundary、key_obstacles_更新(车尾起始，已去除半车宽).
  if (real_time_path_planner_config_.enable_static_nudge()) {
    if (!calcBoundaryFromStaticObstacles(boundary)) {
      const std::string msg = "[RealTimePathPlanner::generateRegularPathBound]: calcBoundaryFromStaticObstacles failed";
      ERT_PLOG_W << msg ;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
  }

  // 动态绕行: 基于决策dynamic nudge od的横向软边界soft_boundary更新(车尾起始，已去除半车宽).
  if (real_time_path_planner_config_.enable_dynamic_nudge()) {
    if (!calcBoundaryFromDynamicObstacles(reference_line_info, boundary)) {
      const std::string msg =
          "[RealTimePathPlanner::generateRegularPathBound]: calcBoundaryFromDynamicObstacles failed";
      ERT_PLOG_W << msg ;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
  }

  // 最终整理决策边界，fs边界，静态边界，动态边界.
  bindBoundary(boundary);

  // 基于场景，更新横向软边界soft_boundary.
  if (!refineBoundaryUnderSpecialScene(boundary->mutable_barrier_boundary(), boundary->mutable_soft_boundary())) {
    const std::string msg = "[RealTimePathPlanner::generateRegularPathBound]: refineBoundaryUnderSpecialScene failed";
    ERT_PLOG_W << msg ;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  //ERT_PLOG_D << "[RealTimePathPlanner::generateRegularPathBound]: generateRegularPathBound completed" ;
  return Status::OK();
}

/**
 * @brief 生成变道路径边界
 * @details 该函数用于生成变道路径规划的边界信息，包括硬边界、软边界和决策边界。这些边界信息用于路径优化和碰撞检测，确保变道路径的安全性和可行性。
 * 
 * @param[in] console 控制台输入数据，包含车辆状态、环境感知等信息
 * @param[in] freespace 自由空间数据，包含可行驶区域的信息
 * @param[in] reference_line_info 参考线信息，包含参考线的几何信息和属性
 * @param[in] decision_boundary 决策边界，包含横向边界决策信息
 * @param[out] boundary 生成的路径边界，包含硬边界、软边界和决策边界
 * 
 * @par 关键变量说明:
 * - barrier_boundary (PathBoundary::BoundaryType): 硬边界，表示车辆不可逾越的边界
 * - soft_boundary (PathBoundary::BoundaryType): 软边界，表示车辆应尽量避免的边界
 * - decision_boundary (PathBoundary::BoundaryType): 决策边界，表示基于决策模块生成的边界
 * - key_obstacles (std::unordered_map<double, PathBoundary::ObstacleInfo>): 关键障碍物信息，包含每个S值对应的最近障碍物
 * 
 * @par 生成流程:
 * 1. 初始化路径边界数据结构
 * 2. 基于自由空间数据更新硬边界和软边界
 * 3. 处理静态障碍物边界
 * 4. 处理动态障碍物边界
 * 5. 应用特殊场景修正
 * 6. 最终整理路径边界信息
 * 
 * @par 流程图:
 * @startuml
 * start
 * :初始化路径边界数据结构;
 * :基于自由空间数据更新硬边界和软边界;
 * :处理静态障碍物边界;
 * :处理动态障碍物边界;
 * :应用特殊场景修正;
 * :最终整理路径边界信息;
 * end
 * @enduml
 * 
 * @return Status 执行状态
 * @retval OK 成功
 * 
 * @note 该函数应在每次变道路径规划循环开始时调用，确保路径边界信息正确生成
 * 
 * @warning 需确保输入参数有效，特别是reference_line_info和decision_boundary
 */
Status RealTimePathPlanner::generateLaneChangePathBound(const Console& console,
                                                        const Freespace& freespace, 
                                                        const ReferenceLineInfo& reference_line_info,
                                                        const LateralBoundDecision& decision_boundary,
                                                        const LongitudinalBoundDecision& longitudinal_boundary,
                                                        PathBoundary* boundary) {
  // 横向各边界信息的初始化:
  // decision_boundary: 决策边界，采样原决策边界值（车尾起始，已去除半车宽）.
  // barrier_boundary: 硬边界，来源于decision_boundary（车尾起始，已去除半车宽）.
  // soft_boundary: 软边界，考虑max_allowed_soft_bounds（车尾起始，已去除半车宽）.
  // key_obstacles: 各s处对应的左右最近Obs的id.
  if (!initPathBoundary(reference_line_info, decision_boundary, longitudinal_boundary, boundary)) {
    const std::string msg = "[RealTimePathPlanner::generateLaneChangePathBound]: initPathBoundary failed.";
    ERT_PLOG_W << msg ;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // 基于fs的横向硬边界barrier_boundary、软边界soft_boundary更新.
  if (real_time_path_planner_config_.enable_freespace_nudge() && stage_state_ == 5) {
    if (!calcBoundaryFromFreespace(freespace, boundary)) {
      const std::string msg = "[RealTimePathPlanner::generateLaneChangePathBound]: calcBoundaryFromFreespace failed";
      ERT_PLOG_W << msg ;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
  }

  // 基于决策nudge od的横向硬边界barrier_boundary、软边界soft_boundary、key_obstacles_更新.
  if (real_time_path_planner_config_.enable_static_nudge()) {
    if (!calcBoundaryFromStaticObstacles(boundary)) {
      const std::string msg = "[RealTimePathPlanner::generateLaneChangePathBound]: calcBoundaryFromStaticObstacles failed";
      ERT_PLOG_W << msg ;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
  }

  bindBoundary(boundary);
  //ERT_PLOG_D << "[RealTimePathPlanner::generateLaneChangePathBound]: generateRegularPathBound completed" ;
  return Status::OK();
}

/**
 * @brief 生成强制倒车路径边界
 * @details 该函数用于生成强制倒车路径规划的边界信息，包括硬边界、软边界和决策边界。这些边界信息用于路径优化和碰撞检测，确保强制倒车路径的安全性和可行性。
 * 
 * @param[in] reference_line_info 参考线信息，包含参考线的几何信息和属性
 * @param[in] decision_boundary 决策边界，包含横向边界决策信息
 * @param[out] boundary 生成的路径边界，包含硬边界、软边界和决策边界
 * 
 * @par 关键变量说明:
 * - barrier_boundary (PathBoundary::BoundaryType): 硬边界，表示车辆不可逾越的边界
 * - soft_boundary (PathBoundary::BoundaryType): 软边界，表示车辆应尽量避免的边界
 * - decision_boundary (PathBoundary::BoundaryType): 决策边界，表示基于决策模块生成的边界
 * - key_obstacles (std::unordered_map<double, PathBoundary::ObstacleInfo>): 关键障碍物信息，包含每个S值对应的最近障碍物
 * 
 * @par 生成流程:
 * 1. 初始化路径边界数据结构
 * 2. 基于参考线数据更新硬边界和软边界
 * 3. 处理静态障碍物边界
 * 4. 处理动态障碍物边界
 * 5. 应用特殊场景修正
 * 6. 最终整理路径边界信息
 * 
 * @par 流程图:
 * @startuml
 * start
 * :初始化路径边界数据结构;
 * :基于参考线数据更新硬边界和软边界;
 * :处理静态障碍物边界;
 * :处理动态障碍物边界;
 * :应用特殊场景修正;
 * :最终整理路径边界信息;
 * end
 * @enduml
 * 
 * @return Status 执行状态
 * @retval OK 成功
 * 
 * @note 该函数应在每次强制倒车路径规划循环开始时调用，确保路径边界信息正确生成
 * 
 * @warning 需确保输入参数有效，特别是reference_line_info和decision_boundary
 */
Status RealTimePathPlanner::generateForceBackPathBound(const ReferenceLineInfo& reference_line_info,
                                                       const LateralBoundDecision& decision_boundary,
                                                       PathBoundary* boundary) {
  CHECK_NOTNULL(boundary);
  const auto& reference_line = reference_line_info.ref_line();

  // 横向边界的start_s, delta_s, size初始化，车尾起始.
  double start_s = std::max(0.0, adc_frenet_s_ - adc_rear_length_);
  double end_s = std::min(reference_line.length(), adc_frenet_end_s_ + adc_front_length_);
  if (!updatePathSegmentInfo(start_s, end_s, boundary)) {
    return Status(PLANNING_ERROR, "too short reference_line");
  }

  const double half_width = adc_width_ * 0.5;

  util::AbstractTable1d<double, double, double> decision_bounds;
  // decision_bounds 决策边界（已去除半车宽）.
  decision_bounds.clear();
  for (const auto& bound : decision_boundary) {
    // 目前仅使用LAT_HARD类型的决策边界.
    if(bound.type == LateralBoundaryType::LAT_HARD) {
      for (const auto& bound_pt : bound.points) {
        decision_bounds.emplace_back(bound_pt.s, bound_pt.l_right + half_width, bound_pt.l_left - half_width);
      }
      break;
    }
  }

  // decision_boundary: 决策边界，采样原决策边界值（车尾起始，已去除半车宽）.
  // barrier_boundary: 硬边界，考虑road_width、自车当前位置的最大允许边界（车尾起始，已去除半车宽）.
  // soft_boundary: 软边界，考虑lane_width、自车当前位置、一定buffer的边界（车尾起始，已去除半车宽）.
  boundary->mutable_decision_boundary()->clear();
  boundary->mutable_barrier_boundary()->clear();
  boundary->mutable_soft_boundary()->clear();
  for (int i = 0; i < boundary->size(); ++i) {
    double curr_s = start_s + i * boundary->delta_s();

    auto [max_decision_bound, has_max_decision_bound] = decision_bounds.interpolate(curr_s);
    // lxy 20240821 maybe block???
    if (!has_max_decision_bound) {
      std::get<1>(max_decision_bound) = std::min(0.0, curr_right_bound_adc_);
      std::get<2>(max_decision_bound) = std::max(0.0, curr_left_bound_adc_);
    }
    std::get<0>(max_decision_bound) = curr_s;
    boundary->mutable_decision_boundary()->emplace_back(std::move(max_decision_bound));

    double left_bound = 0.0, right_bound = 0.0, middle_l = 0.0;
    reference_line.getRoadBound(curr_s, &left_bound, &right_bound);
    middle_l = 0.5 * (left_bound + right_bound);
    left_bound = std::max(left_bound, middle_l + path_bound_points_config_.max_road_width_left());
    right_bound = std::min(right_bound, middle_l - path_bound_points_config_.max_road_width_right());
    boundary->mutable_barrier_boundary()->emplace_back(curr_s,
                                                       std::min(middle_l, std::min(curr_right_bound_adc_, right_bound)),
                                                       std::max(middle_l, std::max(curr_left_bound_adc_, left_bound)));

    reference_line.getLaneBound(curr_s, &left_bound, &right_bound);
    middle_l = 0.5 * (left_bound + right_bound);
    left_bound = std::max(left_bound, middle_l + path_bound_points_config_.max_lane_width_left());
    right_bound = std::min(right_bound, middle_l - path_bound_points_config_.max_lane_width_right());
    boundary->mutable_soft_boundary()->emplace_back(curr_s,
                                                    std::min(middle_l, std::min(curr_right_bound_adc_, right_bound)),
                                                    std::max(middle_l, std::max(curr_left_bound_adc_, left_bound)));
  }
  return Status::OK();
}

/**
 * @brief 初始化路径边界
 * @details 该函数用于初始化路径边界数据结构，包括硬边界、软边界和决策边界。这些边界信息用于路径优化和碰撞检测，确保路径的安全性和可行性。
 * 
 * @param[in] reference_line_info 参考线信息，包含参考线的几何信息和属性
 * @param[in] decision_boundary 决策边界，包含横向边界决策信息
 * @param[out] path_bound 生成的路径边界，包含硬边界、软边界和决策边界
 * 
 * @par 关键变量说明:
 * - barrier_boundary (PathBoundary::BoundaryType): 硬边界，表示车辆不可逾越的边界
 * - soft_boundary (PathBoundary::BoundaryType): 软边界，表示车辆应尽量避免的边界
 * - decision_boundary (PathBoundary::BoundaryType): 决策边界，表示基于决策模块生成的边界
 * - key_obstacles (std::unordered_map<double, PathBoundary::ObstacleInfo>): 关键障碍物信息，包含每个S值对应的最近障碍物
 * 
 * @par 初始化流程:
 * 1. 初始化路径边界数据结构
 * 2. 基于参考线数据更新硬边界和软边界
 * 3. 处理静态障碍物边界
 * 4. 处理动态障碍物边界
 * 5. 应用特殊场景修正
 * 6. 最终整理路径边界信息
 * 
 * @par 流程图:
 * @startuml
 * start
 * :初始化路径边界数据结构;
 * :基于参考线数据更新硬边界和软边界;
 * :处理静态障碍物边界;
 * :处理动态障碍物边界;
 * :应用特殊场景修正;
 * :最终整理路径边界信息;
 * end
 * @enduml
 * 
 * @return bool 初始化状态
 * @retval true 成功
 * @retval false 失败
 * 
 * @note 该函数应在每次路径规划循环开始时调用，确保路径边界信息正确初始化
 * 
 * @warning 需确保输入参数有效，特别是reference_line_info和decision_boundary
 */
bool RealTimePathPlanner::initPathBoundary(const ReferenceLineInfo& reference_line_info,
                                           const LateralBoundDecision& decision_boundary,
                                           const LongitudinalBoundDecision& longitudinal_boundary,
                                           PathBoundary* path_bound) {
  CHECK_NOTNULL(path_bound);

  const auto& reference_line = reference_line_info.ref_line();
  double start_s = std::max(0.0, adc_frenet_s_ - adc_rear_length_);
  double end_s = std::min(reference_line.length(), adc_frenet_end_s_ + adc_front_length_);
  // 横向边界的start_s, delta_s, size初始化，车尾起始.
  if (!updatePathSegmentInfo(start_s, end_s, path_bound)) {
    return false;
  }

  // 横向各边界信息的初始化:
  // decision_boundary: 决策边界，采样原决策边界值（车尾起始，已去除半车宽）.
  // barrier_boundary: 硬边界，来源于decision_boundary（车尾起始，已去除半车宽）.
  // soft_boundary: 软边界，考虑max_allowed_soft_bounds（车尾起始，已去除半车宽）.
  // key_obstacles: 各s处对应的左右最近Obs的id.
  path_bound->mutable_decision_boundary()->clear();
  path_bound->mutable_barrier_boundary()->clear();
  path_bound->mutable_soft_boundary()->clear();

  path_bound->mutable_prior_physical_barrier_boundary()->clear();
  path_bound->mutable_prior_physical_soft_boundary()->clear();

  path_bound->mutable_env_perception_barrier_boundary()->clear();
  path_bound->mutable_env_perception_soft_boundary()->clear();

  path_bound->mutable_static_od_barrier_boundary()->clear();
  path_bound->mutable_static_od_soft_boundary()->clear();

  path_bound->mutable_dynamic_od_soft_boundary()->clear();

  path_bound->mutable_fs_barrier_boundary()->clear();
  path_bound->mutable_fs_soft_boundary()->clear();

  path_bound->mutable_path_barrier_boundary_info()->clear();
  path_bound->mutable_path_soft_boundary_info()->clear();

  auto key_obs = path_bound->mutable_key_obstacles();
  key_obs->clear();

  realtime_traj_boundary_type_info_.clear();
  // boundary s right(-) left(+)
  util::AbstractTable1d<double, double, double> decision_bounds, max_allowed_barrier_bounds, max_allowed_soft_bounds,
      max_range;
  // decision_bounds 决策边界（已去除半车宽）.
  // max_allowed_barrier_bounds 来源于decision_bounds（已去除半车宽）.
  // max_allowed_soft_bounds 考虑lane_width的最大允许边界（已去除半车宽）.
  // max_range用于bound_parser_的init.
  const double half_width = adc_width_ * 0.5;

  ignore_start_s = 0.0, ignore_end_s = 0.0;
  ignore_boundary = ignoreBoundary(longitudinal_boundary, ignore_start_s, ignore_end_s);

  for (const auto& bound : decision_boundary) {
    // 目前仅使用LAT_HARD类型的决策边界.
    if (bound.type == LateralBoundaryType::LAT_HARD) {
      const double search_radius = path_bound_points_config_.search_radius();
      const double search_step = path_bound_points_config_.search_step();
      for (size_t i = 0; i < bound.points.size(); ++i) {
        const auto& bound_pt = bound.points[i];
        double curr_s = bound_pt.s;
        if (std::isinf(curr_s) || std::isnan(curr_s)) {
          ERT_PLOG_E << "[RealTimePathPlanner::initPathBoundary] invalid curr_s: " << curr_s;
          return false;
        }
        double tightest_raw_left = std::numeric_limits<double>::max();
        double tightest_raw_right = std::numeric_limits<double>::lowest();
        int search_count = 0;
        int max_search_count = static_cast<int>(2 * search_radius / search_step) + 1;
        for (double s = curr_s - search_radius; s <= curr_s + search_radius + 1e-6; s += search_step) {
          ++search_count;
          auto [raw_left, raw_right, info, type] = Decision::LateralBoundConsInterpolate(s, bound.points);
          if (raw_left < tightest_raw_left) {
            tightest_raw_left = raw_left;
          }
          if (raw_right > tightest_raw_right) {
            tightest_raw_right = raw_right;
          }
          if (search_count > max_search_count) {
            ERT_PLOG_W << "[RealTimePathPlanner::initPathBoundary] too many search count at curr_s: " << curr_s;
            break;
          }
        }
        double decision_left_l = tightest_raw_left - half_width;
        double decision_right_l = tightest_raw_right + half_width;

        decision_bounds.emplace_back(bound_pt.s, decision_right_l, decision_left_l);
        // 自车前2.0s时距~**~米考虑硬边界不侵占自车box 在bindboundary 里实现
        // todo: add decision boundary extend buffer consider with bound source
        std::vector<double> speed_vec;
        std::vector<double> decision_bound_buffer_vec;
        for (const auto& ele : path_bound_points_config_.speed_decision_bound_buffer_map().elements()) {
          speed_vec.emplace_back(ele.speed());
          decision_bound_buffer_vec.emplace_back(ele.buffer());
        }
        double decision_hard_buffer = math::TableLookUp1D(speed_vec, decision_bound_buffer_vec, adc_sl_info_.first[1]);
        double decision_soft_buffer = path_bound_points_config_.barrier_bound_extend_buffer();
        double left_decision_buffer = decision_soft_buffer, right_decision_buffer = -decision_soft_buffer;  // 外扩
        if(bound_pt.left_type == BoundaryPointTypeInfo::PHYSICALLY_IMPASSABLE){
          left_decision_buffer = -decision_hard_buffer;  // 内收
        }
        if(bound_pt.right_type == BoundaryPointTypeInfo::PHYSICALLY_IMPASSABLE){
          right_decision_buffer = decision_hard_buffer;  // 内收
        }
        double max_range_left = decision_left_l + left_decision_buffer;
        double max_range_right = decision_right_l + right_decision_buffer;
        // ConvertBoundaryType
        realtime_traj_boundary_type_info_.emplace_back(bound_pt.s, ConvertBoundaryType(bound_pt.right_type), ConvertBoundaryType(bound_pt.left_type));
        if (behavior_ == FsmState::LEFT_CHANGE || behavior_ == FsmState::RIGHT_RETURN) {
          max_range_right =
              std::min(max_range_right, curr_right_bound_adc_ - path_bound_points_config_.max_road_width_right());
        } else if (behavior_ == FsmState::RIGHT_CHANGE || behavior_ == FsmState::LEFT_RETURN) {
          max_range_left =
              std::max(max_range_left, curr_left_bound_adc_ + path_bound_points_config_.max_road_width_left());
        } else if ((behavior_ == FsmState::LEFT_HOLD || behavior_ == FsmState::RIGHT_ATTEMPT) &&
                   l_offset_behavior_valid_) {
          max_range_right =
              std::min(max_range_right, curr_right_bound_adc_ - path_bound_points_config_.max_road_width_right());
        } else if ((behavior_ == FsmState::RIGHT_HOLD || behavior_ == FsmState::LEFT_ATTEMPT) &&
                   l_offset_behavior_valid_) {
          max_range_left =
              std::max(max_range_left, curr_left_bound_adc_ + path_bound_points_config_.max_road_width_left());
        }

        max_allowed_barrier_bounds.emplace_back(bound_pt.s, max_range_right, max_range_left);
        if (!real_time_path_planner_config_.enable_polyline_boundary()) {
          max_allowed_soft_bounds.emplace_back(bound_pt.s, max_range_right + decision_soft_buffer,
                                               max_range_left - decision_soft_buffer);
        }
        max_range.emplace_back(bound_pt.s, max_range_right, max_range_left);
      }
    }
    //等决策接口上线打开
    if (bound.type == LateralBoundaryType::LAT_SOFT && real_time_path_planner_config_.enable_polyline_boundary()) {
      for (size_t i = 0; i < bound.points.size(); ++i) {
        const auto& bound_pt = bound.points[i];
        double soft_decision_left_l = bound_pt.l_left - half_width,
               soft_decision_right_l = bound_pt.l_right + half_width;
        // 软边界收缩至硬边界内
        // todo: add decision boundary extend buffer consider with bound source
        double max_range_right = std::min(soft_decision_right_l, curr_right_bound_adc_);
        double max_range_left = std::max(soft_decision_left_l, curr_left_bound_adc_);

        if (behavior_ == FsmState::LEFT_CHANGE || behavior_ == FsmState::RIGHT_RETURN) {
          max_range_right =
              std::min(max_range_right, curr_right_bound_adc_ - path_bound_points_config_.max_road_width_right());
        } else if (behavior_ == FsmState::RIGHT_CHANGE || behavior_ == FsmState::LEFT_RETURN) {
          max_range_left =
              std::max(max_range_left, curr_left_bound_adc_ + path_bound_points_config_.max_road_width_left());
        } else if ((behavior_ == FsmState::LEFT_HOLD || behavior_ == FsmState::RIGHT_ATTEMPT) &&
                   l_offset_behavior_valid_) {
          max_range_right =
              std::min(max_range_right, curr_right_bound_adc_ - path_bound_points_config_.max_road_width_right());
        } else if ((behavior_ == FsmState::RIGHT_HOLD || behavior_ == FsmState::LEFT_ATTEMPT) &&
                   l_offset_behavior_valid_) {
          max_range_left =
              std::max(max_range_left, curr_left_bound_adc_ + path_bound_points_config_.max_road_width_left());
        }

        max_allowed_soft_bounds.emplace_back(bound_pt.s, max_range_right, max_range_left);
        // max_range[i] = make_tuple(bound_pt.s, max_range_right, max_range_left);
      }
    }
  }

  if (max_allowed_barrier_bounds.empty() || max_allowed_soft_bounds.empty()) {
    max_allowed_barrier_bounds.clear();
    max_allowed_soft_bounds.clear();
    for (int i = 0; i < path_bound->size(); ++i) {
      double curr_s = start_s + i * path_bound->delta_s();

      double barrier_left_bound = 0.0, barrier_right_bound = 0.0;
      reference_line.getRoadBound(curr_s, &barrier_left_bound, &barrier_right_bound);

      double max_range_left = barrier_left_bound - half_width;
      double max_range_right = barrier_right_bound + half_width;

      // double max_range_left = std::max(std::get<2>(max_allowed_barrier_bounds.back()), barrier_middle_l +
      // path_bound_points_config_.max_road_width_left()); double max_range_right =
      // std::min(std::get<1>(max_allowed_barrier_bounds.back()), barrier_middle_l -
      // path_bound_points_config_.max_road_width_right());

      double soft_left_bound = 0.0, soft_right_bound = 0.0;
      reference_line.getLaneBound(curr_s, &soft_left_bound, &soft_right_bound);

      double max_soft_range_left = soft_left_bound - half_width;
      double max_soft_range_right = soft_right_bound + half_width;

      if (behavior_ == FsmState::LEFT_CHANGE || behavior_ == FsmState::RIGHT_RETURN) {
        max_range_right =
            std::min(max_range_right, curr_right_bound_adc_ - path_bound_points_config_.max_road_width_right());
        max_soft_range_right =
            std::min(max_soft_range_right, curr_right_bound_adc_ - path_bound_points_config_.max_road_width_right());
      } else if (behavior_ == FsmState::RIGHT_CHANGE || behavior_ == FsmState::LEFT_RETURN) {
        max_range_left =
            std::max(max_range_left, curr_left_bound_adc_ + path_bound_points_config_.max_road_width_left());
        max_soft_range_left =
            std::max(max_soft_range_left, curr_left_bound_adc_ + path_bound_points_config_.max_road_width_left());
      } else if ((behavior_ == FsmState::LEFT_HOLD || behavior_ == FsmState::RIGHT_ATTEMPT) &&
                 l_offset_behavior_valid_) {
        max_range_right =
            std::min(max_range_right, curr_right_bound_adc_ - path_bound_points_config_.max_road_width_right());
        max_soft_range_right =
            std::min(max_soft_range_right, curr_right_bound_adc_ - path_bound_points_config_.max_road_width_right());
      } else if ((behavior_ == FsmState::RIGHT_HOLD || behavior_ == FsmState::LEFT_ATTEMPT) &&
                 l_offset_behavior_valid_) {
        max_range_left =
            std::max(max_range_left, curr_left_bound_adc_ + path_bound_points_config_.max_road_width_left());
        max_soft_range_left =
            std::max(max_soft_range_left, curr_left_bound_adc_ + path_bound_points_config_.max_road_width_left());
      }
      max_range.emplace_back(curr_s, max_range_right, max_range_left);
      max_allowed_barrier_bounds.emplace_back(curr_s, max_range_right, max_range_left);
      max_allowed_soft_bounds.emplace_back(curr_s, max_soft_range_right, max_soft_range_left);
      // ERT_PLOG_D<<"aaaaaaaa my: "<<std::get<0>(max_allowed_barrier_bounds.back())<<"
      // "<<std::get<1>(max_allowed_barrier_bounds.back())<<"  "<<std::get<2>(max_allowed_barrier_bounds.back());
    }
  }

  // bound_parser_初始化：车尾起始，基于参考线等间距采样，生成sample_points_，sample_segments_，ego_sample_points_，check_ranges_
  if (!bound_parser_->init(reference_line, tf_map_2_ego_, start_s, end_s)) {
    return false;
  }

  prior_physical_barrier_bound_filter_->update(max_allowed_barrier_bounds);
  prior_physical_soft_bound_filter_->update(max_allowed_soft_bounds);

  for (auto& sample_pt : bound_parser_->sample_points()) {
    auto [max_decision_bound, has_max_decision_bound] = decision_bounds.interpolate(sample_pt.local_s());
    // lxy 20240821 maybe block???
    if (!has_max_decision_bound) {
      std::get<1>(max_decision_bound) = std::min(0.0, curr_right_bound_adc_);
      std::get<2>(max_decision_bound) = std::max(0.0, curr_left_bound_adc_);
    }
    std::get<0>(max_decision_bound) = sample_pt.local_s();
    path_bound->mutable_decision_boundary()->emplace_back(std::move(max_decision_bound));

    auto [max_barrier_bound, has_max_barrier_bound] =
        prior_physical_barrier_bound_filter_->bound().interpolate(sample_pt.local_s());
    // lxy 20240821 maybe block???
    if (!has_max_barrier_bound) {
      std::get<1>(max_barrier_bound) = std::min(0.0, curr_right_bound_adc_);
      std::get<2>(max_barrier_bound) = std::max(0.0, curr_left_bound_adc_);
    }
    std::get<0>(max_barrier_bound) = sample_pt.local_s();
    path_bound->mutable_prior_physical_barrier_boundary()->emplace_back(max_barrier_bound);
    path_bound->mutable_env_perception_barrier_boundary()->emplace_back(
        std::make_tuple(sample_pt.local_s(), -path_bound_points_config_.max_road_width_right(),
                        path_bound_points_config_.max_road_width_left()));
    path_bound->mutable_static_od_barrier_boundary()->emplace_back(
        std::make_tuple(sample_pt.local_s(), -path_bound_points_config_.max_road_width_right(),
                        path_bound_points_config_.max_road_width_left()));
    path_bound->mutable_fs_barrier_boundary()->emplace_back(
        std::make_tuple(sample_pt.local_s(), -path_bound_points_config_.max_road_width_right(),
                        path_bound_points_config_.max_road_width_left()));
    auto [max_soft_bound, has_max_soft_bound] =
        prior_physical_soft_bound_filter_->bound().interpolate(sample_pt.local_s());
    // lxy 20240821 maybe block???
    if (!has_max_soft_bound) {
      std::get<1>(max_soft_bound) = std::min(0.0, curr_right_bound_adc_);
      std::get<2>(max_soft_bound) = std::max(0.0, curr_left_bound_adc_);
    }
    std::get<0>(max_soft_bound) = sample_pt.local_s();
    path_bound->mutable_prior_physical_soft_boundary()->emplace_back(max_soft_bound);
    path_bound->mutable_env_perception_soft_boundary()->emplace_back(
        std::make_tuple(sample_pt.local_s(), -path_bound_points_config_.max_road_width_right(),
                        path_bound_points_config_.max_road_width_left()));
    path_bound->mutable_static_od_soft_boundary()->emplace_back(
        std::make_tuple(sample_pt.local_s(), -path_bound_points_config_.max_road_width_right(),
                        path_bound_points_config_.max_road_width_left()));
    path_bound->mutable_fs_soft_boundary()->emplace_back(
        std::make_tuple(sample_pt.local_s(), -path_bound_points_config_.max_road_width_right(),
                        path_bound_points_config_.max_road_width_left()));
    path_bound->mutable_dynamic_od_soft_boundary()->emplace_back(
        std::make_tuple(sample_pt.local_s(), -path_bound_points_config_.max_road_width_right(),
                        path_bound_points_config_.max_road_width_left()));
    key_obs->emplace(sample_pt.local_s(), PathBoundary::ObstacleInfo());
  }

  // // print info
  // ERT_PLOG_D << "[bypass]: [RealTimePathPlanner::initPathBoundary]: start_s = " << path_bound->start_s()
  //           << "  end_s = " << path_bound->end_s()
  //           << "  size = " << path_bound->size()
  //           << "  delta_s = " << path_bound->delta_s()
  //           << "  length = " << path_bound->length() ;
  // ERT_PLOG_D << "[bypass]: barrier_size = " << path_bound->barrier_boundary().size()
  //           << "  soft_size = " << path_bound->soft_boundary().size()
  //           << "  decision_size = " << path_bound->decision_boundary().size() ;
  // for(int i = 0; i < std::min<int>(30, path_bound->barrier_boundary().size()); ++i) {
  //   ERT_PLOG_D << "[bypass]: i = " << i ;
  //   ERT_PLOG_D << "   [bypass]: barrier: s = " << std::get<0>(path_bound->barrier_boundary().at(i))
  //             << "   l_min = " << std::get<1>(path_bound->barrier_boundary().at(i))
  //             << "   l_max = " << std::get<2>(path_bound->barrier_boundary().at(i)) ;
  //   ERT_PLOG_D << "   [bypass]: soft: s = " << std::get<0>(path_bound->soft_boundary().at(i))
  //             << "   l_min = " << std::get<1>(path_bound->soft_boundary().at(i))
  //             << "   l_max = " << std::get<2>(path_bound->soft_boundary().at(i)) ;
  //   ERT_PLOG_D << "   decision: s = " << std::get<0>(path_bound->decision_boundary().at(i))
  //             << "   l_min = " << std::get<1>(path_bound->decision_boundary().at(i))
  //             << "   l_max = " << std::get<2>(path_bound->decision_boundary().at(i)) ;
  // }
  return true;
}


bool RealTimePathPlanner::ignoreBoundary(const LongitudinalBoundDecision& longitudinal_bound_decision,
                                        double& ignore_start_s, double& ignore_end_s) {
  const auto& vehicle_param = vehicle_config_->vehicle_param();
  for (const auto& bound : longitudinal_bound_decision) {
    if (bound.type == WallType::LONG_RSA_WALL) {
      ignore_start_s = bound.s + vehicle_param.front_edge_to_ego() - path_bound_points_config_.bound_ignore_rear_buffer();
      ignore_end_s = bound.s + vehicle_param.front_edge_to_ego() + path_bound_points_config_.bound_ignore_front_buffer();
      return true;
    }
  }
  return false;
}

/**
 * @brief 更新路径段信息
 * @details 该函数用于更新路径段的起始位置、结束位置、分段大小和分辨率等信息。这些信息用于路径规划中的边界生成和优化，确保路径的连续性和平滑性。
 * 
 * @param[in] start_s 路径段的起始位置，单位为米
 * @param[in] end_s 路径段的结束位置，单位为米
 * @param[out] path_bound 生成的路径边界，包含路径段的起始位置、结束位置、分段大小和分辨率
 * 
 * @par 关键变量说明:
 * - path_segment_size_ (int): 路径段的分段数量
 * - path_resolution_ (double): 路径段的分辨率，单位为米
 * - start_s (double): 路径段的起始位置，单位为米
 * - end_s (double): 路径段的结束位置，单位为米
 * 
 * @par 更新流程:
 * 1. 计算路径段的长度
 * 2. 检查路径段长度是否有效
 * 3. 计算路径段的分段数量和分辨率
 * 4. 更新路径段的起始位置、结束位置、分段大小和分辨率
 * 
 * @par 流程图:
 * @startuml
 * start
 * :计算路径段长度;
 * if (路径段长度是否有效?) then (是)
 *   :计算路径段的分段数量和分辨率;
 *   :更新路径段的起始位置、结束位置、分段大小和分辨率;
 * else (否)
 *   :返回false;
 * endif
 * end
 * @enduml
 * 
 * @return bool 更新状态
 * @retval true 成功
 * @retval false 失败
 * 
 * @note 该函数应在每次路径规划循环开始时调用，确保路径段信息正确更新
 * 
 * @warning 需确保输入参数有效，特别是start_s和end_s
 */
bool RealTimePathPlanner::updatePathSegmentInfo(const double& start_s, const double& end_s, PathBoundary* path_bound) {
  double length = end_s - start_s;
  // if (length - adc_rear_length_ - adc_front_length_ < 1e-6) {
  if (length < 1e-6) {
    ERT_PLOG_D << "start_s = " << start_s
              << " end_s = " << end_s
              << " adc_rear_length_ = " << adc_rear_length_
              << " adc_front_length_ = " << adc_front_length_
              << " res = " << length - adc_rear_length_ - adc_front_length_
              ;
    return false;
  }

  // ERT_PLOG_D << "before: length = " << length << " path_segment_size_ = " << path_segment_size_ << " path_resolution_ = " << path_resolution_ ;
  // if (path_segment_size_ == 0 ||
  //     std::abs(length - path_segment_size_ * path_resolution_) > path_segment_size_ * kPathResolutionTol) {
  //   path_segment_size_ = std::max<int>(1, std::round(length / kPathResolution) + 1e-10);
  // }
  path_segment_size_ = std::max<int>(1, std::round(length / kPathResolution) + 1e-10);
  path_resolution_ = length / path_segment_size_;
  // ERT_PLOG_D << "after: path_segment_size_ = " << path_segment_size_ << " path_resolution_ = " << path_resolution_ ;
  if (path_resolution_ < 1e-6) {
    return false;
  }

  // 横向边界的start_s, delta_s, size初始化，车尾起始.
  path_bound->reset(start_s, path_resolution_, path_segment_size_ + 1);
  return true;
}

/**
 * @brief 基于自由空间计算路径边界
 * @details 该函数用于根据自由空间数据计算路径的硬边界和软边界。这些边界信息用于路径优化和碰撞检测，确保路径的安全性和可行性。
 * 
 * @param[in] freespace 自由空间数据，包含可行驶区域的信息
 * @param[out] boundary 生成的路径边界，包含硬边界和软边界
 * 
 * @par 关键变量说明:
 * - barrier_boundary (PathBoundary::BoundaryType): 硬边界，表示车辆不可逾越的边界
 * - soft_boundary (PathBoundary::BoundaryType): 软边界，表示车辆应尽量避免的边界
 * - block_freespace_info (PathBoundary::BlockingFreespaceInfo): 阻塞自由空间信息，包含阻塞区域的信息
 * - path_blocked_idx (int): 路径阻塞的索引，表示路径在哪个位置被阻塞
 * 
 * @par 计算流程:
 * 1. 初始化路径边界数据结构
 * 2. 基于自由空间数据更新硬边界
 * 3. 基于自由空间数据更新软边界
 * 4. 处理阻塞区域信息
 * 5. 最终整理路径边界信息
 * 
 * @par 流程图:
 * @startuml
 * start
 * :初始化路径边界数据结构;
 * :基于自由空间数据更新硬边界;
 * :基于自由空间数据更新软边界;
 * :处理阻塞区域信息;
 * :最终整理路径边界信息;
 * end
 * @enduml
 * 
 * @return bool 计算状态
 * @retval true 成功
 * @retval false 失败
 * 
 * @note 该函数应在每次路径规划循环中调用，确保路径边界信息正确计算
 * 
 * @warning 需确保输入参数有效，特别是freespace
 */
bool RealTimePathPlanner::calcBoundaryFromFreespace(const Freespace& freespace, PathBoundary* boundary) {
  auto barrier_boundary = boundary->mutable_fs_barrier_boundary();
  auto soft_boundary = boundary->mutable_fs_soft_boundary();
  auto block_freespace_info = boundary->mutable_blocking_freespace_info();
  int path_blocked_idx = -1;

  // std::vector<double> sample_offsets;
  // for (auto& sample_pt : bound_parser_->sample_points()) {
  //   double offset = 0.0;
  //   auto [decision_bound, has_decision_bound] = decision_bounds_.interpolate(sample_pt.local_s());
  //   if (has_decision_bound) {
  //     auto& [s, lower, upper] = decision_bound;
  //     offset = 0.5 * (lower + upper);
  //   }
  //   sample_offsets.emplace_back(offset);
  // }

  // 基于fs的硬边界生成.
  // auto raw_bound_info = bound_parser_->getBoundInfoFromFreespace(freespace,
  //                                                                sample_offsets,
  //                                                                path_bound_points_config_.default_barrier_lateral_buffer_for_fs());
  auto raw_bound_info = bound_parser_->getBoundInfoFromFreespace(freespace, 
                                                                 path_bound_points_config_.default_barrier_lateral_buffer_for_fs());
  auto barrier_bound_info = bound_parser_->getFreespaceBoundInfoWithLongiBuffer(raw_bound_info, 
                                                                                path_bound_points_config_.barrier_start_long_buffer_for_fs(), 
                                                                                path_bound_points_config_.barrier_end_long_buffer_for_fs());
  freespace_barrier_bound_filter_->update(barrier_bound_info.bound);
  for (int i = 0; i < barrier_boundary->size(); i++) {
    auto& [s, lower, upper] = barrier_boundary->at(i);
    auto [fs_bound, has_bound] = freespace_barrier_bound_filter_->bound().interpolate(s);
    CHECK(has_bound);
    auto& [fs_s, fs_lower, fs_upper] = fs_bound;
    // extend freespace bound when out of map to avoid undetected hard boundary
    if (fs_s < s) {
      break;
    } else {
      if (!updateBoundary(0.5 * adc_width_ + path_bound_points_config_.default_barrier_lateral_buffer_for_fs(),
                          fs_lower, fs_upper, barrier_boundary->at(i), true)) {
        path_blocked_idx = i;
        auto block_s = fs_s - adc_frenet_s_;
        ERT_PLOG_D << "[bypass]: barrier block fs detected! block info: index = " << i << "  block_s = " << block_s ;
        debug_status_.emplace(PathData::DebugStatusType::BLOCK_FS_BOUND);
        // break;
      }
    }
  }
  // trimPathBounds(path_blocked_idx, barrier_boundary);
  // trimPathBounds(path_blocked_idx, soft_boundary);

  // 基于fs的软边界生成.
  auto soft_bound_info = bound_parser_->getFreespaceBoundInfoWithLongiBuffer(raw_bound_info, 
                                                                             path_bound_points_config_.soft_start_long_buffer_for_fs(), 
                                                                             path_bound_points_config_.soft_end_long_buffer_for_fs());
  // // update the bound from the contract_range_info_, for soft boundary
  // for (auto& boundary : soft_bound_info.bound) {
  //   if (contract_range_info_.empty()) {
  //     break;
  //   }
  //   auto& [s, lower, upper] = boundary;
  //   auto iter = std::find_if(contract_range_info_.begin(), contract_range_info_.end(),
  //                            [s](const RangeInfo& range) { return range.start_s <= s && s < range.end_s; });
  //   double contract_distance = config_.contract_lat_distance();
  //   double left_bound_contract_distace = 0.0;
  //   double right_bound_contract_distace = 0.0;
  //   if (iter != contract_range_info_.end()) {
  //     contract_distance *= iter->contract_coff;
  //     iter->type == RangeInfo::LEFT ? (left_bound_contract_distace = -contract_distance)
  //                                   : (right_bound_contract_distace = contract_distance);
  //   }
  //   lower += right_bound_contract_distace;
  //   upper += left_bound_contract_distace;
  // }

  freespace_soft_bound_filter_->update(soft_bound_info.bound);
  for (int i = 0; i < soft_boundary->size(); i++) {
    auto& [s, lower, upper] = soft_boundary->at(i);
    auto [fs_bound, has_bound] = freespace_soft_bound_filter_->bound().interpolate(s);
    CHECK(has_bound);
    auto& [fs_s, fs_lower, fs_upper] = fs_bound;
    // extend freespace bound when out of map to avoid undetected hard boundary
    if (fs_s < s) {
      break;
    } else {
      // if(!updateBoundary(0.5 * adc_width_ + path_bound_points_config_.default_soft_lateral_buffer_for_fs(),
      //                    fs_lower, fs_upper, soft_boundary->at(i), true)) {
      //   ERT_PLOG_D << "[bypass]: soft block fs detected! block info: index = " << i ;
      //   break;
      // }

      // deal with soft bound cross in OcpPathOptimizer::getSoftCenterBound() and getSoftEdgeBound()
      updateBoundary(0.5 * adc_width_ + path_bound_points_config_.default_soft_lateral_buffer_for_fs(), fs_lower,
                     fs_upper, soft_boundary->at(i), false);
    }
  }

  // print info
  // ERT_PLOG_D << "[bypass]: [RealTimePathPlanner::calcBoundaryFromFreespace]: start_s = " << boundary->start_s()
  //           << "  end_s = " << boundary->end_s()
  //           << "  size = " << boundary->size()
  //           << "  delta_s = " << boundary->delta_s()
  //           << "  length = " << boundary->length() ;
  // ERT_PLOG_D << "[bypass][Freespace]: fs barrier_size = " << boundary->barrier_boundary().size()
  //           << "  soft_size = " << boundary->soft_boundary().size()
  //           << "  decision_size = " << boundary->decision_boundary().size() ;
  // for(int i = 0; i < std::min<int>(30, boundary->barrier_boundary().size()); ++i) {
  //   ERT_PLOG_D << "[bypass][Freespace]: i = " << i ;
  //   ERT_PLOG_D << "   [bypass][Freespace]: barrier: s = " << std::get<0>(boundary->barrier_boundary().at(i))
  //             << "   l_min = " << std::get<1>(boundary->barrier_boundary().at(i))
  //             << "   l_max = " << std::get<2>(boundary->barrier_boundary().at(i)) ;
  //   ERT_PLOG_D << "   [bypass][Freespace]: soft: s = " << std::get<0>(boundary->soft_boundary().at(i))
  //             << "   l_min = " << std::get<1>(boundary->soft_boundary().at(i))
  //             << "   l_max = " << std::get<2>(boundary->soft_boundary().at(i)) ;
  //   ERT_PLOG_D << "   decision: s = " << std::get<0>(boundary->decision_boundary().at(i))
  //             << "   l_min = " << std::get<1>(boundary->decision_boundary().at(i))
  //             << "   l_max = " << std::get<2>(boundary->decision_boundary().at(i)) ;
  // }

  return true;
}

/**
 * @brief 基于静态障碍物计算路径边界
 * @details 该函数用于根据静态障碍物数据计算路径的硬边界和软边界。这些边界信息用于路径优化和碰撞检测，确保路径的安全性和可行性。
 * 
 * @param[out] boundary 生成的路径边界，包含硬边界和软边界
 * 
 * @par 关键变量说明:
 * - barrier_boundary (PathBoundary::BoundaryType): 硬边界，表示车辆不可逾越的边界
 * - soft_boundary (PathBoundary::BoundaryType): 软边界，表示车辆应尽量避免的边界
 * - path_blocked_idx (int): 路径阻塞的索引，表示路径在哪个位置被阻塞
 * 
 * @par 计算流程:
 * 1. 初始化路径边界数据结构
 * 2. 基于静态障碍物数据更新硬边界
 * 3. 基于静态障碍物数据更新软边界
 * 4. 处理阻塞区域信息
 * 5. 最终整理路径边界信息
 * 
 * @par 流程图:
 * @startuml
 * start
 * :初始化路径边界数据结构;
 * :基于静态障碍物数据更新硬边界;
 * :基于静态障碍物数据更新软边界;
 * :处理阻塞区域信息;
 * :最终整理路径边界信息;
 * end
 * @enduml
 * 
 * @return bool 计算状态
 * @retval true 成功
 * @retval false 失败
 * 
 * @note 该函数应在每次路径规划循环中调用，确保路径边界信息正确计算
 * 
 * @warning 需确保输入参数有效，特别是boundary
 */
bool RealTimePathPlanner::calcBoundaryFromStaticObstacles(PathBoundary* boundary) {
  auto barrier_boundary = boundary->mutable_static_od_barrier_boundary();
  auto soft_boundary = boundary->mutable_static_od_soft_boundary();
  int path_blocked_idx = -1;

  // 1.基于od速度及type信息，为决策静态nudge的od增加横向软、硬边界的buffer.
  std::vector<std::tuple<double, double, double>> barrier_buffer, soft_buffer;
  ERT_PLOG_D << "[bypass]: decision_static_obstacles_size() = " << decision_static_obstacles_.size() ;
  std::vector<double> speed_vec;
  std::vector<double> speed_start_longit_bound_buffer_vec;
  std::vector<double> speed_end_longit_bound_buffer_vec;
  for (const auto& ele : path_bound_points_config_.speed_longit_bound_buffer_map().elements()) {
    speed_vec.emplace_back(ele.speed());
    speed_start_longit_bound_buffer_vec.emplace_back(ele.start_buffer());
    speed_end_longit_bound_buffer_vec.emplace_back(ele.end_buffer());
  }
  double start_longit_hard_buffer = path_bound_points_config_.barrier_start_long_buffer_for_obs();
  double end_longit_hard_buffer = path_bound_points_config_.barrier_end_long_buffer_for_obs();
  if (!speed_start_longit_bound_buffer_vec.empty() && !speed_vec.empty()) {
    start_longit_hard_buffer =
        math::TableLookUp1D(speed_vec, speed_start_longit_bound_buffer_vec, adc_sl_info_.first[1]);
  }
  if (!speed_end_longit_bound_buffer_vec.empty() && !speed_vec.empty()) {
    end_longit_hard_buffer = math::TableLookUp1D(speed_vec, speed_end_longit_bound_buffer_vec, adc_sl_info_.first[1]);
  }
  for (auto obstacle : decision_static_obstacles_) {
    const auto lateral_safe_buffer_pair = getObsLaterSafeBuffer(obstacle, path_bound_points_config_.enable_lateral_buffer_use_obs_type(),
    path_bound_points_config_.enable_lateral_buffer_use_obs_speed());
    barrier_buffer.emplace_back(start_longit_hard_buffer, end_longit_hard_buffer, lateral_safe_buffer_pair.first);
    soft_buffer.emplace_back(path_bound_points_config_.soft_start_long_buffer_for_obs(), path_bound_points_config_.soft_end_long_buffer_for_obs(),
                             lateral_safe_buffer_pair.second);
    ERT_PLOG_D << "[bypass]:decision_static_obstacles_id = "<< obstacle->id 
              << "  lateral_tag = " << static_cast<int>(obstacle->lat_od_tag)
              << "  start_l = " << obstacle->cur_sl_bound.start_l()
              << "  end_l = " << obstacle->cur_sl_bound.end_l()
              << "  start_s = " << obstacle->cur_sl_bound.start_s()
              << "  end_s = " << obstacle->cur_sl_bound.end_s()
              << "  l_buffer = " << lateral_safe_buffer_pair.first
              << "  soft_l_buffer = " << lateral_safe_buffer_pair.second
              ;
  }

  // 2.基于决策nudge od的横向硬边界barrier_boundary更新(车尾起始，已去除半车宽).
  //   1）基于od及buffer，在bound_parser_中生成沿参考线的ObstacleBoundaryInfo，包括: bound，right_bound_obstacles，left_bound_obstacles.
  auto barrier_bound_info = bound_parser_->getBoundaryFromStaticObstacles(decision_static_obstacles_, barrier_buffer);
  //   2）ObstacleBoundaryInfo.bound的filter处理，考虑real_time_path_planner_config_.filter_in(), real_time_path_planner_config_.filter_out().
  static_obstacle_barrier_bound_filter_->update(barrier_bound_info.bound);
  //   3）更新横向硬边界barrier_boundary.
  for (int i = 0; i < barrier_boundary->size(); ++i) {
    const auto& [s, lower, upper] = barrier_boundary->at(i);
    const auto& [static_od_bound, has_bound] = static_obstacle_barrier_bound_filter_->bound().interpolate(s);
    CHECK(has_bound);
    const auto& [static_od_s, static_od_lower, static_od_upper] = static_od_bound;  // s l_right l_left
    if (!updateBoundary(0.5 * adc_width_, static_od_lower, static_od_upper, barrier_boundary->at(i), true)) {
      path_blocked_idx = i;
      ERT_PLOG_D << "[bypass]:static barrier block info: index = " << i ;
      debug_status_.emplace(PathData::DebugStatusType::BLOCK_STATIC_OBSTACLE);
      // break;
    }
  }
  //   4）若边界交叉，则block，裁剪barrier_boundary与soft_boundary.
  // trimPathBounds(path_blocked_idx, barrier_boundary);
  // trimPathBounds(path_blocked_idx, soft_boundary);

  // 3.基于决策nudge od的横向软边界soft_boundary、key_obstacles_更新(车尾起始，已去除半车宽).
  //   1）基于od及buffer，在bound_parser_中生成沿参考线的ObstacleBoundaryInfo，包括: bound，right_bound_obstacles，left_bound_obstacles.
  auto soft_bound_info = bound_parser_->getBoundaryFromStaticObstacles(decision_static_obstacles_, soft_buffer);
  //   2）ObstacleBoundaryInfo.bound的filter处理，考虑filter_out_speed_，快进慢出.
  static_obstacle_soft_bound_filter_->update(soft_bound_info.bound);
  //   3）更新横向软边界soft_boundary及key_obstacles_.
  for (int i = 0; i < soft_boundary->size(); ++i) {
    const auto& [s, lower, upper] = soft_boundary->at(i);
    const auto& [static_od_bound, has_bound] = static_obstacle_soft_bound_filter_->bound().interpolate(s);
    CHECK(has_bound);
    const auto& [ods, static_od_lower, static_od_upper] = static_od_bound;
    const Decision::DecisionObject* ob_left = nullptr;
    const Decision::DecisionObject* ob_right = nullptr;
    if (i < soft_bound_info.left_bound_obstacles.size()) {
      ob_left = soft_bound_info.left_bound_obstacles[i];
      ob_right = soft_bound_info.right_bound_obstacles[i];
    }
    if(!updateBoundaryFromObstacle(static_od_lower, static_od_upper, ob_left, ob_right, &soft_boundary->at(i),
                                    boundary->findKeyObstacle(s))) {
      // ERT_PLOG_D << "[bypass]:static soft block info: index = " << i ;
      // break;
    }

    // deal with soft bound cross in OcpPathOptimizer::getSoftCenterBound() and getSoftEdgeBound()
    updateBoundary(0.5 * adc_width_, static_od_lower, static_od_upper, soft_boundary->at(i), false);
  }

  // print info
  ERT_PLOG_D << "[bypass]: [RealTimePathPlanner::calcBoundaryFromStaticObstacles]: start_s = " << boundary->start_s()
            << "  end_s = " << boundary->end_s()
            << "  size = " << boundary->size()
            << "  delta_s = " << boundary->delta_s()
            << "  length = " << boundary->length() ;
  ERT_PLOG_D << "[bypass]: static barrier_size = " << boundary->barrier_boundary().size()
            << "  soft_size = " << boundary->soft_boundary().size()
            << "  decision_size = " << boundary->decision_boundary().size() ;
  // for(int i = 0; i < std::min<int>(30, boundary->barrier_boundary().size()); ++i) {
  //   ERT_PLOG_D << "[bypass]: i = " << i ;
  //   ERT_PLOG_D << "   [bypass]: barrier: s = " << std::get<0>(boundary->barrier_boundary().at(i))
  //             << "   l_min = " << std::get<1>(boundary->barrier_boundary().at(i))
  //             << "   l_max = " << std::get<2>(boundary->barrier_boundary().at(i)) ;
  //   ERT_PLOG_D << "   [bypass]: soft: s = " << std::get<0>(boundary->soft_boundary().at(i))
  //             << "   l_min = " << std::get<1>(boundary->soft_boundary().at(i))
  //             << "   l_max = " << std::get<2>(boundary->soft_boundary().at(i)) ;
  //   ERT_PLOG_D << "   decision: s = " << std::get<0>(boundary->decision_boundary().at(i))
  //             << "   l_min = " << std::get<1>(boundary->decision_boundary().at(i))
  //             << "   l_max = " << std::get<2>(boundary->decision_boundary().at(i)) ;
  // }
  return true;
}

/**
 * @brief 获取障碍物的横向安全缓冲区
 * @details 该函数用于根据障碍物的类型和速度计算横向安全缓冲区。这些缓冲区用于路径规划中的边界生成和优化，确保路径的安全性和可行性。
 * 
 * @param[in] obs 障碍物对象，包含障碍物的类型和速度信息
 * @param[in] use_obs_type 是否使用障碍物类型来计算缓冲区
 * @param[in] use_obs_speed 是否使用障碍物速度来计算缓冲区
 * 
 * @par 关键变量说明:
 * - safe_buffer_pair (std::pair<double, double>): 安全缓冲区，包含硬边界和软边界的缓冲区
 * - default_barrier_lateral_buffer (double): 默认的硬边界横向缓冲区
 * - default_soft_lateral_buffer (double): 默认的软边界横向缓冲区
 * 
 * @par 计算流程:
 * 1. 初始化安全缓冲区
 * 2. 根据障碍物速度计算缓冲区
 * 3. 根据障碍物类型调整缓冲区
 * 4. 返回最终的安全缓冲区
 * 
 * @par 流程图:
 * @startuml
 * start
 * :初始化安全缓冲区;
 * if (是否使用障碍物速度?) then (是)
 *   :根据障碍物速度计算缓冲区;
 * else (否)
 *   :使用默认缓冲区;
 * endif
 * if (是否使用障碍物类型?) then (是)
 *   :根据障碍物类型调整缓冲区;
 * else (否)
 *   :使用默认缓冲区;
 * endif
 * :返回最终的安全缓冲区;
 * end
 * @enduml
 * 
 * @return std::pair<double, double> 安全缓冲区，包含硬边界和软边界的缓冲区
 * 
 * @note 该函数应在每次路径规划循环中调用，确保路径边界信息正确计算
 * 
 * @warning 需确保输入参数有效，特别是obs
 */
std::pair<double, double> RealTimePathPlanner::getObsLaterSafeBuffer(const std::shared_ptr<Decision::DecisionObject> obs,
                                                                     const bool& use_obs_type,
                                                                     const bool& use_obs_speed) {
  std::pair<double, double> safe_buffer_pair(0.0, 0.0);
  safe_buffer_pair.first = math::lerp(path_bound_points_config_.default_min_barrier_lateral_buffer_for_obs(), 0.0,
  path_bound_points_config_.default_max_barrier_lateral_buffer_for_obs(),
  path_bound_points_config_.max_speed_limit(), adc_frenet_sd_ * MS_KMH);
  safe_buffer_pair.first =
      std::min<double>(std::max<double>(safe_buffer_pair.first,
        path_bound_points_config_.default_min_barrier_lateral_buffer_for_obs()),
        path_bound_points_config_.default_max_barrier_lateral_buffer_for_obs());

  safe_buffer_pair.second = math::lerp(path_bound_points_config_.default_min_soft_lateral_buffer_for_obs(), 0.0,
  path_bound_points_config_.default_max_soft_lateral_buffer_for_obs(),
  path_bound_points_config_.max_speed_limit(), adc_frenet_sd_ * MS_KMH);
  safe_buffer_pair.second =
      std::min<double>(std::max<double>(safe_buffer_pair.second,
        path_bound_points_config_.default_min_soft_lateral_buffer_for_obs()),
        path_bound_points_config_.default_max_soft_lateral_buffer_for_obs());

  if (use_obs_speed) {
    safe_buffer_pair = getObsLaterSafeBufferFromSpeed(obs, safe_buffer_pair.first, safe_buffer_pair.second);
  } else if (use_obs_type) {
    safe_buffer_pair = getObsLaterSafeBufferFromType(obs->type, safe_buffer_pair.first, safe_buffer_pair.second);
  } else {
    ;
  }
  return safe_buffer_pair;
}
/**
 * @brief 根据障碍物类型获取横向安全缓冲区
 * @details 该函数用于根据障碍物的类型获取横向安全缓冲区。这些缓冲区用于路径规划中的边界生成和优化，确保路径的安全性和可行性。
 * 
 * @param[in] obs_type 障碍物类型，用于确定安全缓冲区的大小
 * @param[in] default_barrier_lateral_buffer 默认的硬边界横向缓冲区
 * @param[in] default_soft_lateral_buffer 默认的软边界横向缓冲区
 * 
 * @par 关键变量说明:
 * - safe_buffer_pair (std::pair<double, double>): 安全缓冲区，包含硬边界和软边界的缓冲区
 * - obs_type_lateral_distance_map_ (std::map<int, LateralDistanceConfig>): 障碍物类型与横向距离配置的映射
 * 
 * @par 计算流程:
 * 1. 初始化安全缓冲区
 * 2. 根据障碍物类型查找对应的横向距离配置
 * 3. 如果找到对应的配置，则更新安全缓冲区
 * 4. 返回最终的安全缓冲区
 * 
 * @par 流程图:
 * @startuml
 * start
 * :初始化安全缓冲区;
 * if (是否找到障碍物类型对应的配置?) then (是)
 *   :更新安全缓冲区;
 * else (否)
 *   :使用默认缓冲区;
 * endif
 * :返回最终的安全缓冲区;
 * end
 * @enduml
 * 
 * @return std::pair<double, double> 安全缓冲区，包含硬边界和软边界的缓冲区
 * 
 * @note 该函数应在每次路径规划循环中调用，确保路径边界信息正确计算
 * 
 * @warning 需确保输入参数有效，特别是obs_type
 */
std::pair<double, double> RealTimePathPlanner::getObsLaterSafeBufferFromType(
    const Decision::ObjectType& obs_type, const double& default_barrier_lateral_buffer,
    const double& default_soft_lateral_buffer) {
  std::pair<double, double> safe_buffer_pair(default_barrier_lateral_buffer, default_soft_lateral_buffer);
  if (obs_type_lateral_distance_map_.empty()) {
    return safe_buffer_pair;
  }

  // ERT_PLOG_D << "obs_type = " << static_cast<int>(obs_type) ;
  const auto& iter = obs_type_lateral_distance_map_.find(static_cast<int>(obs_type));
  if (iter != obs_type_lateral_distance_map_.end()) {
    safe_buffer_pair.first = iter->second.barrier_lateral_safe_distance();  // barrier
    safe_buffer_pair.second = iter->second.soft_lateral_safe_distance();    // soft
    // ERT_PLOG_D << "find obs type: barrier_safe_dis = " << safe_buffer_pair.first
    //           << "  soft_safe_dis = " << safe_buffer_pair.second ;
    return safe_buffer_pair;
  } else {
    // ERT_PLOG_D << "failed to find obs type: barrier_safe_dis = " << safe_buffer_pair.first
    //           << "  soft_safe_dis = " << safe_buffer_pair.second ;
    return safe_buffer_pair;
  }
}

/**
 * @brief 根据障碍物速度获取横向安全缓冲区
 * @details 该函数用于根据障碍物的速度计算横向安全缓冲区。这些缓冲区用于路径规划中的边界生成和优化，确保路径的安全性和可行性。
 * 
 * @param[in] obs 障碍物对象，包含障碍物的速度信息
 * @param[in] default_barrier_lateral_buffer 默认的硬边界横向缓冲区
 * @param[in] default_soft_lateral_buffer 默认的软边界横向缓冲区
 * 
 * @par 关键变量说明:
 * - safe_buffer_pair (std::pair<double, double>): 安全缓冲区，包含硬边界和软边界的缓冲区
 * - coff_a (double): 速度影响系数a
 * - coff_b (double): 速度影响系数b
 * - coff_c_min (double): 速度影响系数c_min
 * - lateral_buffer (double): 计算得到的横向缓冲区
 * 
 * @par 计算流程:
 * 1. 初始化安全缓冲区
 * 2. 根据障碍物类型查找对应的速度影响系数
 * 3. 根据自车速度和障碍物速度计算横向缓冲区
 * 4. 更新安全缓冲区
 * 5. 返回最终的安全缓冲区
 * 
 * @par 流程图:
 * @startuml
 * start
 * :初始化安全缓冲区;
 * :查找速度影响系数;
 * :根据自车速度和障碍物速度计算横向缓冲区;
 * :更新安全缓冲区;
 * :返回最终的安全缓冲区;
 * end
 * @enduml
 * 
 * @return std::pair<double, double> 安全缓冲区，包含硬边界和软边界的缓冲区
 * 
 * @note 该函数应在每次路径规划循环中调用，确保路径边界信息正确计算
 * 
 * @warning 需确保输入参数有效，特别是obs
 */
std::pair<double, double> RealTimePathPlanner::getObsLaterSafeBufferFromSpeed(
    const std::shared_ptr<Decision::DecisionObject> obs, const double& default_barrier_lateral_buffer,
    const double& default_soft_lateral_buffer) {
  std::pair<double, double> safe_buffer_pair(default_barrier_lateral_buffer, default_soft_lateral_buffer);
  // get the coff of speed
  double coff_a = 0.0;
  double coff_b = 0.0;
  double coff_c_min = 0.0;  // TODO develop it in future
  double lateral_buffer = 0.0;
  auto elvLaterBuffer = [=](const double& v_ego, const double& v_obs, const double& coff_a, const double& coff_b,
                            const double& coff_c_min) -> double {
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
  constexpr double default_soft_buffer_coff_obs = 1.5;
  lateral_buffer = elvLaterBuffer(adc_frenet_sd_, obs->spd, coff_a, coff_b, coff_c_min);
  safe_buffer_pair.first = lateral_buffer;   // TODO develop it in future
  safe_buffer_pair.second = (iter != obs_type_lateral_distance_map_.end())
                                ? iter->second.soft_lateral_safe_distance()
                                : default_soft_buffer_coff_obs * lateral_buffer;
  return safe_buffer_pair;
}

/**
 * @brief 更新路径边界
 * @details 该函数用于根据给定的横向缓冲区、左右边界信息更新路径边界。更新后的边界用于路径优化和碰撞检测，确保路径的安全性和可行性。
 * 
 * @param[in] lat_buffer 横向缓冲区，用于调整路径边界的宽度
 * @param[in] right_bound 右侧边界值，表示路径的右侧限制
 * @param[in] left_bound 左侧边界值，表示路径的左侧限制
 * @param[in,out] boundary 待更新的路径边界，包含路径点的位置、左侧边界和右侧边界
 * @param[in] return_if_valid 是否在边界有效时立即返回
 * 
 * @par 关键变量说明:
 * - s (double): 路径点的位置
 * - lower (double): 路径的左侧边界
 * - upper (double): 路径的右侧边界
 * - new_l_min (double): 更新后的左侧边界
 * - new_l_max (double): 更新后的右侧边界
 * 
 * @par 更新流程:
 * 1. 计算新的左侧边界和右侧边界
 * 2. 如果路径点位于车辆范围内，则调整边界以确保车辆安全
 * 3. 更新路径边界
 * 4. 如果边界无效且return_if_valid为true，则返回false
 * 
 * @par 流程图:
 * @startuml
 * start
 * :计算新的左侧边界和右侧边界;
 * if (路径点是否位于车辆范围内?) then (是)
 *   :调整边界以确保车辆安全;
 * endif
 * :更新路径边界;
 * if (边界是否无效且return_if_valid为true?) then (是)
 *   :返回false;
 * endif
 * :返回true;
 * end
 * @enduml
 * 
 * @return bool 更新状态
 * @retval true 成功
 * @retval false 失败
 * 
 * @note 该函数应在每次路径规划循环中调用，确保路径边界信息正确更新
 * 
 * @warning 需确保输入参数有效，特别是lat_buffer、right_bound和left_bound
 */
bool RealTimePathPlanner::updateBoundary(const double& lat_buffer, const double& right_bound, const double& left_bound,
                                         std::tuple<double, double, double>& boundary, const bool return_if_valid) {
  auto& [s, lower, upper] = boundary;
  double new_l_min = std::fmax(lower, right_bound + lat_buffer);
  double new_l_max = std::fmin(upper, left_bound - lat_buffer);
  if(s > adc_frenet_s_ - adc_rear_length_ - (1e-2) && s < adc_frenet_s_ + adc_front_length_ + (1e-2)) {
    new_l_min = std::fmin(curr_right_bound_adc_, new_l_min);
    new_l_max = std::fmax(curr_left_bound_adc_, new_l_max);
  }

  lower = new_l_min;
  upper = new_l_max;
  if (return_if_valid && new_l_min > new_l_max) {
    ERT_PLOG_D << "[RealTimePathPlanner::updateBoundary]: block info: origin bound " << lower << " " << upper << "to " << new_l_min << " " << new_l_max ;
    ERT_PLOG_D << "[RealTimePathPlanner::updateBoundary]: block info: perception bound " << right_bound << " " << left_bound << " lat_buffer " << lat_buffer ;
    return false;
  }

  return true;
}

/**
 * @brief 根据障碍物信息更新路径边界
 * @details 该函数用于根据障碍物的左右边界信息更新路径边界。更新后的边界用于路径优化和碰撞检测，确保路径的安全性和可行性。
 * 
 * @param[in] right_bound 右侧边界值，表示路径的右侧限制
 * @param[in] left_bound 左侧边界值，表示路径的左侧限制
 * @param[in] ob_left 左侧障碍物对象，包含障碍物的ID等信息
 * @param[in] ob_right 右侧障碍物对象，包含障碍物的ID等信息
 * @param[in,out] boundary 待更新的路径边界，包含路径点的位置、左侧边界和右侧边界
 * @param[in] key_ob 关键障碍物信息，用于记录障碍物ID
 * 
 * @par 关键变量说明:
 * - s (double): 路径点的位置
 * - lmin (double): 路径的左侧边界
 * - lmax (double): 路径的右侧边界
 * - new_l_min (double): 更新后的左侧边界
 * - new_l_max (double): 更新后的右侧边界
 * 
 * @par 更新流程:
 * 1. 根据右侧边界更新左侧边界
 * 2. 根据左侧边界更新右侧边界
 * 3. 如果路径点位于车辆范围内，则调整边界以确保车辆安全
 * 4. 更新路径边界
 * 
 * @par 流程图:
 * @startuml
 * start
 * :根据右侧边界更新左侧边界;
 * :根据左侧边界更新右侧边界;
 * if (路径点是否位于车辆范围内?) then (是)
 *   :调整边界以确保车辆安全;
 * endif
 * :更新路径边界;
 * end
 * @enduml
 * 
 * @return bool 更新状态
 * @retval true 成功
 * @retval false 失败
 * 
 * @note 该函数应在每次路径规划循环中调用，确保路径边界信息正确更新
 * 
 * @warning 需确保输入参数有效，特别是right_bound和left_bound
 */
bool RealTimePathPlanner::updateBoundaryFromObstacle(const double& right_bound, const double& left_bound,
                                                     const Decision::DecisionObject* const ob_left, const Decision::DecisionObject* const ob_right,
                                                     std::tuple<double, double, double>* boundary,
                                                     std::optional<PathBoundary::ObstacleMap::iterator> key_ob) {
  auto& [s, lmin, lmax] = *boundary;
  double new_l_min = lmin;
  if (right_bound > new_l_min) {
    new_l_min = right_bound;
    if (key_ob && ob_right) {
      key_ob.value()->second.ob_id_right = ob_right->id;
    }
  }
  double new_l_max = lmax;
  if (left_bound < new_l_max) {
    new_l_max = left_bound;
    if (key_ob && ob_left) {
      key_ob.value()->second.ob_id_left = ob_left->id;
    }
  }

  if(s > adc_frenet_s_ - adc_rear_length_ - (1e-2) && s < adc_frenet_s_ + adc_front_length_ + (1e-2)) {
    new_l_min = std::fmin(curr_right_bound_adc_, new_l_min);
    new_l_max = std::fmax(curr_left_bound_adc_, new_l_max);
  }
  if (new_l_min >= new_l_max) {
    ERT_PLOG_D << "[RealTimePathPlanner::updateBoundaryFromObstacle]: block info: origin bound " << lmin << " " << lmax << "to " << new_l_min << " " << new_l_max ;
    // return false;  // block by obstacle
  }
  lmin = new_l_min;
  lmax = new_l_max;
  return true;
}
/**
 * @brief 裁剪路径边界
 * @details 该函数用于根据路径阻塞索引裁剪路径边界。当路径被障碍物阻塞时，裁剪阻塞点之后的路径边界，确保路径规划的安全性。
 * 
 * @param[in] path_blocked_idx 路径阻塞的索引，表示路径在哪个位置被阻塞
 * @param[in,out] path_boundaries 待裁剪的路径边界，包含路径点的位置、左侧边界和右侧边界
 * 
 * @par 关键变量说明:
 * - path_blocked_idx_ (int): 路径阻塞的索引，用于记录阻塞位置
 * 
 * @par 裁剪流程:
 * 1. 检查路径阻塞索引是否有效
 * 2. 如果路径阻塞索引为0，表示路径完全被阻塞
 * 3. 裁剪阻塞点之后的路径边界
 * 
 * @par 流程图:
 * @startuml
 * start
 * :检查路径阻塞索引是否有效;
 * if (路径阻塞索引是否为0?) then (是)
 *   :路径完全被阻塞;
 * endif
 * :裁剪阻塞点之后的路径边界;
 * end
 * @enduml
 * 
 * @note 该函数应在每次路径规划循环中调用，确保路径边界信息正确裁剪
 * 
 * @warning 需确保输入参数有效，特别是path_blocked_idx
 */
void RealTimePathPlanner::trimPathBounds(const int& path_blocked_idx,
                                         std::vector<std::tuple<double, double, double>>* const path_boundaries) {
  if (path_blocked_idx >= 0 && path_blocked_idx < path_boundaries->size() - 1) {
    if (path_blocked_idx == 0) {
      ERT_PLOG_W << "[RealTimePathPlanner::trimPathBounds]: Completely blocked. Cannot move at all" ;
    }
    path_blocked_idx_ = path_blocked_idx;
    path_boundaries->erase(path_boundaries->begin() + path_blocked_idx, path_boundaries->end());
  }
}

/**
 * @brief 基于动态障碍物计算路径边界
 * @details 该函数用于根据动态障碍物数据计算路径的软边界。这些边界信息用于路径优化和碰撞检测，确保路径的安全性和可行性。
 * 
 * @param[in] reference_line_info 参考线信息，包含参考线的几何信息和障碍物信息
 * @param[out] boundary 生成的路径边界，包含软边界
 * 
 * @par 关键变量说明:
 * - soft_boundary (PathBoundary::BoundaryType): 软边界，表示车辆应尽量避免的边界
 * - soft_buffer (std::vector<std::tuple<double, double, double, double, double, double, double>>): 动态障碍物的软边界缓冲区
 * 
 * @par 计算流程:
 * 1. 初始化软边界数据结构
 * 2. 基于动态障碍物数据计算软边界缓冲区
 * 3. 基于软边界缓冲区更新软边界
 * 4. 处理阻塞区域信息
 * 5. 最终整理路径边界信息
 * 
 * @par 流程图:
 * @startuml
 * start
 * :初始化软边界数据结构;
 * :基于动态障碍物数据计算软边界缓冲区;
 * :基于软边界缓冲区更新软边界;
 * :处理阻塞区域信息;
 * :最终整理路径边界信息;
 * end
 * @enduml
 * 
 * @return bool 计算状态
 * @retval true 成功
 * @retval false 失败
 * 
 * @note 该函数应在每次路径规划循环中调用，确保路径边界信息正确计算
 * 
 * @warning 需确保输入参数有效，特别是reference_line_info和boundary
 */
bool RealTimePathPlanner::calcBoundaryFromDynamicObstacles(const ReferenceLineInfo& reference_line_info, PathBoundary* boundary) { 
  auto soft_boundary = boundary->mutable_dynamic_od_soft_boundary();

  // 1.基于od速度及type信息，为决策动态nudge的od增加横向软边界的buffer.
  std::vector<std::tuple<double, double, double, double, double, double, double>> soft_buffer;
  ERT_PLOG_D << "[bypass]: decision_dynamic_obstacles_size() = " << decision_dynamic_obstacles_.size() ;
  for (auto obstacle : decision_dynamic_obstacles_) {
    const auto lateral_safe_buffer_pair = getDynamicObsLaterSafeBuffer(obstacle, path_bound_points_config_.enable_lateral_buffer_use_obs_type(), 
    path_bound_points_config_.enable_lateral_buffer_use_obs_speed());
    soft_buffer.emplace_back(path_bound_points_config_.soft_start_long_buffer_for_dynamic_obs(),
                             path_bound_points_config_.soft_start_long_pre_time_for_dynamic_obs(),
                             path_bound_points_config_.soft_end_long_buffer_for_dynamic_obs(),
                             path_bound_points_config_.soft_end_long_pre_time_for_dynamic_obs(),
                             lateral_safe_buffer_pair.second,
                             path_bound_points_config_.hysteresis_delta_veloc_threshold(),
                             path_bound_points_config_.hysteresis_delta_veloc_buffer());
    ERT_PLOG_D << "[bypass]:decision_dynamic_obstacles_id = "<< obstacle->id 
              << "  lateral_tag = " << static_cast<int>(obstacle->lat_od_tag)
              << "  speed = " << obstacle->spd
              << "  start_l = " << obstacle->cur_sl_bound.start_l()
              << "  end_l = " << obstacle->cur_sl_bound.end_l()
              << "  start_s = " << obstacle->cur_sl_bound.start_s()
              << "  end_s = " << obstacle->cur_sl_bound.end_s()
              << "  l_buffer = " << std::get<4>(soft_buffer.back())
              << "  start_long_buffer = " << std::get<0>(soft_buffer.back())
              << "  t = " << std::get<1>(soft_buffer.back())
              << "  end_long_buffer = " << std::get<2>(soft_buffer.back())
              << "  t = " << std::get<3>(soft_buffer.back())
              ;
  }

  // 2.基于决策nudge od的横向软边界soft_boundary、key_obstacles_更新(车尾起始，已去除半车宽).
  //   1）基于od及buffer，在bound_parser_中生成沿参考线的ObstacleBoundaryInfo，包括: bound，right_bound_obstacles，left_bound_obstacles.
  auto soft_bound_info = bound_parser_->getBoundaryFromDynamicObstacles(reference_line_info.ref_line(), decision_dynamic_obstacles_, soft_buffer);
  //   2）ObstacleBoundaryInfo.bound的filter处理，考虑filter_out_speed_，快进慢出.
  dynamic_obstacle_soft_bound_filter_->update(soft_bound_info.bound);
  //   3）更新横向软边界soft_boundary及key_obstacles_.
  for (int i = 0; i < soft_boundary->size(); ++i) {
    const auto& [s, lower, upper] = soft_boundary->at(i);
    const auto& [od_bound, has_bound] = dynamic_obstacle_soft_bound_filter_->bound().interpolate(s);
    CHECK(has_bound);
    const auto& [ods, od_lower, od_upper] = od_bound;

    // deal with soft bound cross in OcpPathOptimizer::getSoftCenterBound() and getSoftEdgeBound()
    updateBoundary(0.5 * adc_width_, od_lower, od_upper, soft_boundary->at(i), false);
  }

  // print info
  // ERT_PLOG_D << "[bypass]: [RealTimePathPlanner::calcBoundaryFromDynamicObstacles]: start_s = " << boundary->start_s()
  //           << "  end_s = " << boundary->end_s()
  //           << "  size = " << boundary->size()
  //           << "  delta_s = " << boundary->delta_s()
  //           << "  length = " << boundary->length() ;
  // ERT_PLOG_D << "[bypass]: dynamic barrier_size = " << boundary->barrier_boundary().size()
  //           << "  soft_size = " << boundary->soft_boundary().size()
  //           << "  decision_size = " << boundary->decision_boundary().size() ;
  // for(int i = 0; i < std::min<int>(30, boundary->barrier_boundary().size()); ++i) {
  //   ERT_PLOG_D << "[bypass]: i = " << i ;
  //   ERT_PLOG_D << "   [bypass]: barrier: s = " << std::get<0>(boundary->barrier_boundary().at(i))
  //             << "   l_min = " << std::get<1>(boundary->barrier_boundary().at(i))
  //             << "   l_max = " << std::get<2>(boundary->barrier_boundary().at(i)) ;
  //   ERT_PLOG_D << "   [bypass]: soft: s = " << std::get<0>(boundary->soft_boundary().at(i))
  //             << "   l_min = " << std::get<1>(boundary->soft_boundary().at(i))
  //             << "   l_max = " << std::get<2>(boundary->soft_boundary().at(i)) ;
  //   ERT_PLOG_D << "   decision: s = " << std::get<0>(boundary->decision_boundary().at(i))
  //             << "   l_min = " << std::get<1>(boundary->decision_boundary().at(i))
  //             << "   l_max = " << std::get<2>(boundary->decision_boundary().at(i)) ;
  // }

  return true;
}

/**
 * @brief 绑定路径边界
 * @details 该函数用于将不同类型的路径边界（如物理边界、感知边界、静态障碍物边界等）绑定到最终的路径边界中。根据不同的模式（如常规模式或强制回退模式），选择不同的边界组合方式。
 * 
 * @param[in,out] boundary 待绑定的路径边界，包含硬边界和软边界
 * @param[in] mode 绑定模式，决定如何组合不同类型的边界
 * 
 * @par 关键变量说明:
 * - barrier_boundary (PathBoundary::BoundaryType): 硬边界，表示车辆不可逾越的边界
 * - soft_boundary (PathBoundary::BoundaryType): 软边界，表示车辆应尽量避免的边界
 * - prior_physical_barrier_boundary (PathBoundary::BoundaryType): 先验物理硬边界
 * - prior_physical_soft_boundary (PathBoundary::BoundaryType): 先验物理软边界
 * - env_perception_barrier_boundary (PathBoundary::BoundaryType): 环境感知硬边界
 * - env_perception_soft_boundary (PathBoundary::BoundaryType): 环境感知软边界
 * - fs_barrier_boundary (PathBoundary::BoundaryType): 自由空间硬边界
 * - fs_soft_boundary (PathBoundary::BoundaryType): 自由空间软边界
 * - static_od_barrier_boundary (PathBoundary::BoundaryType): 静态障碍物硬边界
 * - static_od_soft_boundary (PathBoundary::BoundaryType): 静态障碍物软边界
 * - dynamic_obstacle_soft_boundary (PathBoundary::BoundaryType): 动态障碍物软边界
 * - path_barrier_boundary_info (PathBoundary::PathBoundaryUnitInfo): 路径硬边界信息
 * 
 * @par 绑定流程:
 * 1. 清空当前的硬边界和软边界
 * 2. 根据模式选择不同的边界组合方式
 * 3. 在常规模式下，将先验物理边界与环境感知边界结合
 * 4. 在强制回退模式下，直接使用先验物理边界
 * 5. 更新路径边界信息
 * 
 * @par 流程图:
 * @startuml
 * start
 * :清空当前的硬边界和软边界;
 * if (模式是否为常规模式?) then (是)
 *   :将先验物理边界与环境感知边界结合;
 * else (否)
 *   :直接使用先验物理边界;
 * endif
 * :更新路径边界信息;
 * end
 * @enduml
 * 
 * @note 该函数应在每次路径规划循环中调用，确保路径边界信息正确绑定
 * 
 * @warning 需确保输入参数有效，特别是boundary和mode
 */
void RealTimePathPlanner::bindBoundary(PathBoundary* boundary, string mode) {
  auto barrier_boundary = boundary->mutable_barrier_boundary();
  auto soft_boundary = boundary->mutable_soft_boundary();
  barrier_boundary->clear();
  soft_boundary->clear();

  auto prior_physical_barrier_boundary = boundary->mutable_prior_physical_barrier_boundary();
  auto prior_physical_soft_boundary = boundary->mutable_prior_physical_soft_boundary();

  auto env_perception_barrier_boundary = boundary->mutable_env_perception_barrier_boundary();
  auto env_perception_soft_boundary = boundary->mutable_env_perception_soft_boundary();

  auto fs_barrier_boundary = boundary->mutable_fs_barrier_boundary();
  auto fs_soft_boundary = boundary->mutable_fs_soft_boundary();

  auto static_od_barrier_boundary = boundary->mutable_static_od_barrier_boundary();
  auto static_od_soft_boundary = boundary->mutable_static_od_soft_boundary();

  auto dynamic_obstacle_soft_boundary = boundary->mutable_dynamic_od_soft_boundary();

  auto path_barrier_boundary_info = boundary->mutable_path_barrier_boundary_info();

  size_t min_index_barrier = std::min({env_perception_barrier_boundary->size(),fs_barrier_boundary->size(), static_od_barrier_boundary->size()});
  for (int i = 0; i < min_index_barrier; i++) {
    auto& [fs_barrier_s, fs_barrier_lower, fs_barrier_upper] = fs_barrier_boundary->at(i);
    auto& [static_od_barrier_s, static_od_barrier_lower, static_od_barrier_upper] = static_od_barrier_boundary->at(i);
    auto& [env_perception_barrier_s, env_perception_barrier_lower, env_perception_barrier_upper] = env_perception_barrier_boundary->at(i);
    env_perception_barrier_lower = std::max({env_perception_barrier_lower, static_od_barrier_lower, fs_barrier_lower});
    env_perception_barrier_upper = std::min({env_perception_barrier_upper, static_od_barrier_upper, fs_barrier_upper});
  }

  size_t min_index_soft = std::min({env_perception_soft_boundary->size(), fs_soft_boundary->size(), static_od_soft_boundary->size(),dynamic_obstacle_soft_boundary->size()});
  for (int i = 0; i < min_index_soft; i++) {
    auto& [fs_soft_s, fs_soft_lower, fs_soft_upper] = fs_soft_boundary->at(i);
    auto& [static_od_soft_s, static_od_soft_lower, static_od_soft_upper] = static_od_soft_boundary->at(i);
    auto& [dynamic_obs_soft_s, dynamic_obs_soft_lower, dynamic_obs_soft_upper] = dynamic_obstacle_soft_boundary->at(i);
    auto& [env_perception_soft_s, env_perception_soft_lower, env_perception_soft_upper] =
        env_perception_soft_boundary->at(i);
    env_perception_soft_lower = std::max({env_perception_soft_lower,static_od_soft_lower, fs_soft_lower, dynamic_obs_soft_lower});
    env_perception_soft_upper = std::min({env_perception_soft_upper,static_od_soft_upper, fs_soft_upper, dynamic_obs_soft_upper});
  }

  // SFIELD_INFO(lateral_planning, " barrier size {} {} {} {}    ", barrier_boundary->size(), soft_boundary->size(),
  //             env_perception_barrier_boundary->size(), env_perception_soft_boundary->size())
  int path_blocked_idx = -1;

  if (mode == "regular") {
    constexpr double trim_default_width = 1.75;
    size_t min_index_barrier =
        std::min({prior_physical_barrier_boundary->size(), env_perception_barrier_boundary->size()});
    for (int i = 0; i < min_index_barrier; i++) {
      auto& [prior_physical_barrier_s, prior_physical_barrier_lower, prior_physical_barrier_upper] =
          prior_physical_barrier_boundary->at(i);
      auto& [env_perception_barrier_s, env_perception_barrier_lower, env_perception_barrier_upper] =
          env_perception_barrier_boundary->at(i);

      barrier_boundary->emplace_back(make_tuple(prior_physical_barrier_s,
                                                std::max(prior_physical_barrier_lower, env_perception_barrier_lower),
                                                std::min(prior_physical_barrier_upper, env_perception_barrier_upper)));
      PathBoundary::PathBoundaryUnitInfo bound_unit_info(
          prior_physical_barrier_s, prior_physical_barrier_upper,prior_physical_barrier_lower,
          PathBoundary::BoundaryUnitTypeInfo::DECISION, PathBoundary::BoundaryUnitTypeInfo::DECISION);
      auto& [barrier_s, barrier_lower, barrier_upper] = barrier_boundary->at(i);
      if (path_blocked_idx > -1) {
        barrier_lower = -trim_default_width;
        barrier_upper = trim_default_width;
        continue;
      }
      if (barrier_lower > barrier_upper) {
        path_blocked_idx = i;
        // add block info
        ERT_PLOG_D << "[RealTimePathPlanner::bindBoundary]: block info: index = " << i << "  s = " << barrier_s
                  << "  l_min = " << barrier_lower << "  l_max = " << barrier_upper ;
        barrier_lower = -trim_default_width;
        barrier_upper = trim_default_width;
      }
      // 增加车前2.0s的边界保护
      if (prior_physical_barrier_s > adc_frenet_s_ - adc_rear_length_ - (1e-2) &&
          prior_physical_barrier_s < adc_frenet_s_ + adc_front_length_ + (planning_start_point_.v() * 2.0)) {
        barrier_lower = std::fmin(curr_right_bound_adc_, barrier_lower);
        barrier_upper = std::fmax(curr_left_bound_adc_, barrier_upper);
      }
      path_barrier_boundary_info->emplace_back(bound_unit_info);
      // ERT_PLOG_D << "[RealTimePathPlanner::bindBoundary]: path_barrier_boundary_info  s ="<< bound_unit_info.s<<"
      // bound_unit_info.l_left_type = "<<int(bound_unit_info.l_left_type)<< "  bound_unit_info.l_right_type =
      // "<<int(bound_unit_info.l_right_type);
    }

    size_t min_index_soft = std::min({prior_physical_soft_boundary->size(), env_perception_soft_boundary->size()});
    for (int i = 0; i < min_index_soft; i++) {
      auto& [prior_physical_soft_s, prior_physical_soft_lower, prior_physical_soft_upper] =
          prior_physical_soft_boundary->at(i);
      auto& [env_perception_soft_s, env_perception_soft_lower, env_perception_soft_upper] =
          env_perception_soft_boundary->at(i);

      soft_boundary->emplace_back(make_tuple(prior_physical_soft_s,
                                             std::max(prior_physical_soft_lower, env_perception_soft_lower),
                                             std::min(prior_physical_soft_upper, env_perception_soft_upper)));
    }
  } else if (mode == "force_back") {
    barrier_boundary->assign(prior_physical_barrier_boundary->begin(), prior_physical_barrier_boundary->end());
    soft_boundary->assign(prior_physical_soft_boundary->begin(), prior_physical_soft_boundary->end());
  }
}

/**
 * @brief 获取动态障碍物的横向安全缓冲区
 * @details 该函数用于根据动态障碍物的类型和速度信息计算横向安全缓冲区。这些缓冲区用于路径规划中的边界生成和优化，确保路径的安全性和可行性。
 * 
 * @param[in] obs 动态障碍物对象，包含障碍物的类型和速度信息
 * @param[in] use_obs_type 是否使用障碍物类型来计算缓冲区
 * @param[in] use_obs_speed 是否使用障碍物速度来计算缓冲区
 * 
 * @par 关键变量说明:
 * - safe_buffer_pair (std::pair<double, double>): 安全缓冲区，包含硬边界和软边界的缓冲区
 * 
 * @par 计算流程:
 * 1. 初始化安全缓冲区
 * 2. 如果使用障碍物速度，则根据速度计算缓冲区
 * 3. 如果使用障碍物类型，则根据类型计算缓冲区
 * 4. 返回最终的安全缓冲区
 * 
 * @par 流程图:
 * @startuml
 * start
 * :初始化安全缓冲区;
 * if (是否使用障碍物速度?) then (是)
 *   :根据速度计算缓冲区;
 * else if (是否使用障碍物类型?) then (是)
 *   :根据类型计算缓冲区;
 * endif
 * :返回最终的安全缓冲区;
 * end
 * @enduml
 * 
 * @return std::pair<double, double> 安全缓冲区，包含硬边界和软边界的缓冲区
 * 
 * @note 该函数应在每次路径规划循环中调用，确保路径边界信息正确计算
 * 
 * @warning 需确保输入参数有效，特别是obs
 */
std::pair<double, double> RealTimePathPlanner::getDynamicObsLaterSafeBuffer(const std::shared_ptr<Decision::DecisionObject> obs,
                                                                            const bool use_obs_type,
                                                                            const bool use_obs_speed) {
  std::pair<double, double> safe_buffer_pair(
    path_bound_points_config_.default_barrier_lateral_buffer_for_dynamic_obs(),
    path_bound_points_config_.default_soft_lateral_buffer_for_dynamic_obs());
  if (use_obs_speed) {
    safe_buffer_pair = getObsLaterSafeBufferFromSpeed(
        obs, path_bound_points_config_.default_barrier_lateral_buffer_for_dynamic_obs(),
        path_bound_points_config_.default_soft_lateral_buffer_for_dynamic_obs());
  } else if (use_obs_type) {
    safe_buffer_pair = getObsLaterSafeBufferFromType(
        obs->type, path_bound_points_config_.default_barrier_lateral_buffer_for_dynamic_obs(),
        path_bound_points_config_.default_soft_lateral_buffer_for_dynamic_obs());
  } else {
    ;
  }
  return safe_buffer_pair;
}

/**
 * @brief 在特殊场景下优化路径边界
 * @details 该函数用于在特殊场景（如阻塞、狭窄、弯道等）下优化路径的硬边界和软边界。根据场景类型调整边界，确保路径的安全性和可行性。
 * 
 * @param[in,out] barrier_boundary 待优化的硬边界，包含路径点的位置、左侧边界和右侧边界
 * @param[in,out] soft_boundary 待优化的软边界，包含路径点的位置、左侧边界和右侧边界
 * 
 * @par 关键变量说明:
 * - special_scene_s_range_sets_ (std::vector<std::tuple<string, double, double>>): 特殊场景的范围集合，包含场景标签、起始位置和结束位置
 * - scene_tag (string): 场景标签，标识当前场景类型（如阻塞、狭窄、弯道等）
 * - start_s (double): 场景的起始位置
 * - end_s (double): 场景的结束位置
 * 
 * @par 优化流程:
 * 1. 获取特殊场景的范围信息
 * 2. 获取弯道场景信息
 * 3. 更新调试信息
 * 4. 如果未启用特殊场景优化，则直接返回
 * 5. 遍历路径边界，根据场景类型调整边界
 * 
 * @par 流程图:
 * @startuml
 * start
 * :获取特殊场景的范围信息;
 * :获取弯道场景信息;
 * :更新调试信息;
 * if (是否启用特殊场景优化?) then (否)
 *   :直接返回;
 * endif
 * :遍历路径边界，根据场景类型调整边界;
 * end
 * @enduml
 * 
 * @return bool 优化状态
 * @retval true 成功
 * @retval false 失败
 * 
 * @note 该函数应在每次路径规划循环中调用，确保路径边界信息正确优化
 * 
 * @warning 需确保输入参数有效，特别是barrier_boundary和soft_boundary
 */
bool RealTimePathPlanner::refineBoundaryUnderSpecialScene(
    std::vector<std::tuple<double, double, double>>* const barrier_boundary,
    std::vector<std::tuple<double, double, double>>* const soft_boundary) {
  getSpecialSceneSrange(barrier_boundary, soft_boundary);
  getCurveScene();
  updateTagRangesDebugInfo();

  int bound_size = std::min(barrier_boundary->size(), soft_boundary->size());
  for(int i = 0; i < bound_size; ++i) {
    auto& [s, lower, upper] = barrier_boundary->at(i);
    double middle = 0.5 * (lower + upper);
    auto& [s_s, s_lower, s_upper] = soft_boundary->at(i);
    double s_middle = 0.5 * (s_lower + s_upper);
    
    if (ignore_boundary && s >= ignore_start_s && s <= ignore_end_s) {
      upper = path_bound_points_config_.max_lane_width_left();
      lower = -path_bound_points_config_.max_lane_width_right();
    }
    if (ignore_boundary && s_s >= ignore_start_s - path_bound_points_config_.soft_start_long_buffer_for_obs()
        && s_s <= ignore_end_s + path_bound_points_config_.soft_end_long_buffer_for_obs()) {
      s_upper = path_bound_points_config_.max_lane_width_left();
      s_lower = -path_bound_points_config_.max_lane_width_right();
    }
    if (!real_time_path_planner_config_.enable_refine_boundary_under_special_scene()){
      continue;
    }
    for (int j = 0; j < special_scene_s_range_sets_.size(); ++j) {
      const auto& [scene_tag, start_s, end_s] = special_scene_s_range_sets_.at(j);
      if (s_s >= start_s && s_s <= end_s) {
        if (scene_tag == "Block") {
          lower = middle - path_bound_points_config_.max_road_width_right();
          upper = middle + path_bound_points_config_.max_road_width_left();
          s_lower = s_middle - path_bound_points_config_.max_lane_width_right();
          s_upper = s_middle + path_bound_points_config_.max_lane_width_left();
          continue;
        }
        if (scene_tag == "Narrow") {
          s_lower += real_time_path_planner_config_.narrow_scene_soft_bound_shift();
          s_upper -= real_time_path_planner_config_.narrow_scene_soft_bound_shift();
          // lxy 240822 modify
          s_upper = std::fmax(s_lower + 1e-6, s_upper);
          continue;
        }
        if (scene_tag == "ExtremeCurve") {
          auto max_bound_kappa = target_ref_line_info_->ref_line().getReferencePoint(s_s).kappa();
          double max_kappa_bound = 2 + 0.6 * (std::clamp(double(1 / abs(max_bound_kappa)), 10.0, 40.0) - 10.0);
          s_lower = std::fmax(s_lower, s_middle - max_kappa_bound);
          s_upper = std::fmin(s_upper, s_middle + max_kappa_bound);
          // lxy 240822 modify
          s_upper = std::fmax(s_lower + 1e-6, s_upper);
          continue;
        }
        if (scene_tag == "Curve") {
          s_lower += real_time_path_planner_config_.curve_scene_soft_bound_shift();
          s_upper -= real_time_path_planner_config_.curve_scene_soft_bound_shift();
          // lxy 240822 modify
          s_upper = std::fmax(s_lower + 1e-6, s_upper);
          continue;
        }
        if (scene_tag == "Dest") {
          s_lower = std::fmax(s_lower, s_middle - real_time_path_planner_config_.dest_scene_permited_soft_bound());
          s_upper = std::fmin(s_upper, s_middle + real_time_path_planner_config_.dest_scene_permited_soft_bound());
          // lxy 240822 modify
          s_upper = std::fmax(s_lower + 1e-6, s_upper);
          continue;
        }
      }
    }
  }

  // // print info
  // ERT_PLOG_D << "[bypass]: [RealTimePathPlanner::refineBoundaryUnderSpecialScene]: refine barrier_size = " << barrier_boundary->size()
  //           << "  soft_size = " << soft_boundary->size()
  //           ;
  // for(int i = 0; i < std::min<int>(20, barrier_boundary->size()); ++i) {
  //   ERT_PLOG_D << "[bypass]: i = " << i ;
  //   ERT_PLOG_D << "   [bypass]: barrier: s = " << std::get<0>(barrier_boundary->at(i))
  //             << "   l_min = " << std::get<1>(barrier_boundary->at(i))
  //             << "   l_max = " << std::get<2>(barrier_boundary->at(i)) ;
  //   ERT_PLOG_D << "   [bypass]: soft: s = " << std::get<0>(soft_boundary->at(i))
  //             << "   l_min = " << std::get<1>(soft_boundary->at(i))
  //             << "   l_max = " << std::get<2>(soft_boundary->at(i)) ;
  // }

  return true;
}

/**
 * @brief 获取特殊场景的范围信息
 * @details 该函数用于根据路径的软边界和硬边界信息，识别并记录特殊场景（如狭窄场景、阻塞场景）的范围。这些范围信息用于后续的路径优化和边界调整。
 * 
 * @param[in] barrier_boundary 路径的硬边界，包含路径点的位置、左侧边界和右侧边界
 * @param[in] soft_boundary 路径的软边界，包含路径点的位置、左侧边界和右侧边界
 * 
 * @par 关键变量说明:
 * - narrow_scene_s_range (std::pair<double, double>): 狭窄场景的范围，包含起始位置和结束位置
 * - in_narrow_scence (bool): 是否处于狭窄场景中
 * - special_scene_s_range_sets_ (std::vector<std::tuple<string, double, double>>): 特殊场景的范围集合，包含场景标签、起始位置和结束位置
 * 
 * @par 获取流程:
 * 1. 遍历软边界，识别狭窄场景的范围
 * 2. 如果路径被阻塞，记录阻塞场景的范围
 * 3. 将识别到的场景范围信息存储到special_scene_s_range_sets_中
 * 
 * @par 流程图:
 * @startuml
 * start
 * :遍历软边界;
 * if (是否进入狭窄场景?) then (是)
 *   :记录狭窄场景的起始位置;
 * else if (是否离开狭窄场景?) then (是)
 *   :记录狭窄场景的结束位置;
 *   :将狭窄场景范围存储到special_scene_s_range_sets_中;
 * endif
 * if (路径是否被阻塞?) then (是)
 *   :记录阻塞场景的范围;
 * endif
 * end
 * @enduml
 * 
 * @note 该函数应在每次路径规划循环中调用，确保特殊场景信息正确更新
 * 
 * @warning 需确保输入参数有效，特别是barrier_boundary和soft_boundary
 */
void RealTimePathPlanner::getSpecialSceneSrange(std::vector<std::tuple<double, double, double>>* const barrier_boundary,
                                                std::vector<std::tuple<double, double, double>>* const soft_boundary) {
  // narrow scene
  std::pair<double, double> narrow_scene_s_range(std::make_pair(0.0, 0.0));
  bool in_narrow_scence = false;
  for (int i = 0; i < soft_boundary->size(); ++i) {
    auto& [s, lower, upper] = soft_boundary->at(i);
    // narrow_scene
    if (upper - lower <= real_time_path_planner_config_.narrow_scene_barrier_bound_interval() && !in_narrow_scence) {
      in_narrow_scence = true;
      narrow_scene_s_range.first = s;
    } else if (upper - lower >= real_time_path_planner_config_.narrow_scene_barrier_bound_interval() + 0.5 && in_narrow_scence) {     
      in_narrow_scence = false;
      narrow_scene_s_range.second = s;
      special_scene_s_range_sets_.emplace_back(
          std::tuple<string, double, double>{"Narrow", narrow_scene_s_range.first, narrow_scene_s_range.second});
    }
  }
  if (in_narrow_scence) {
    in_narrow_scence = false;
    narrow_scene_s_range.second = get<0>(soft_boundary->back());
    special_scene_s_range_sets_.emplace_back(
        std::tuple<string, double, double>{"Narrow", narrow_scene_s_range.first, narrow_scene_s_range.second});
  }

  // block_scene
  if (path_blocked_idx_ != -1 && !barrier_boundary->empty()) {
    special_scene_s_range_sets_.emplace_back(std::tuple<string, double, double>{
        "Block", get<0>(barrier_boundary->back()) - 1.0, get<0>(barrier_boundary->back()) + 1.0});
  }
  return;
}

/**
 * @brief 获取弯道场景信息
 * @details 该函数用于根据参考线的曲率信息，识别并记录弯道场景的范围。这些范围信息用于后续的路径优化和边界调整，确保路径的安全性和可行性。
 * 
 * @par 关键变量说明:
 * - reference_line (ReferenceLine): 参考线对象，包含参考线的几何信息和曲率信息
 * - start_s (double): 场景的起始位置
 * - end_s (double): 场景的结束位置
 * - curve_scene_s_range (std::pair<double, double>): 弯道场景的范围，包含起始位置和结束位置
 * - in_curve_scene (bool): 是否处于弯道场景中
 * - special_scene_s_range_sets_ (std::vector<std::tuple<string, double, double>>): 特殊场景的范围集合，包含场景标签、起始位置和结束位置
 * 
 * @par 获取流程:
 * 1. 获取参考线的起始位置和结束位置
 * 2. 遍历参考线的参考点，根据曲率信息识别弯道场景
 * 3. 将识别到的弯道场景范围信息存储到special_scene_s_range_sets_中
 * 
 * @par 流程图:
 * @startuml
 * start
 * :获取参考线的起始位置和结束位置;
 * :遍历参考线的参考点;
 * if (曲率是否大于进入弯道阈值?) then (是)
 *   :记录弯道场景的起始位置;
 * else if (曲率是否小于离开弯道阈值?) then (是)
 *   :记录弯道场景的结束位置;
 *   :将弯道场景范围存储到special_scene_s_range_sets_中;
 * endif
 * end
 * @enduml
 * 
 * @note 该函数应在每次路径规划循环中调用，确保弯道场景信息正确更新
 * 
 * @warning 需确保参考线信息有效，特别是reference_line
 */
void RealTimePathPlanner::getCurveScene() {
  const auto& reference_line = target_ref_line_info_->ref_line();
  double start_s = std::max(0.0, adc_frenet_s_ - adc_rear_length_);
  double end_s = std::min(reference_line.length(), adc_frenet_end_s_ + adc_front_length_);

  // uturn scene
  auto uturn_ranges = reference_line.getSpecifiedDirectionRanges(DrivingDirection::kDirectionUTurnOnly);
  for(const auto& range : uturn_ranges) {
    if (range.second < start_s) continue;
    if (range.first > end_s) break;
    special_scene_s_range_sets_.emplace_back(
        std::tuple<string, double, double>{"UTurn", std::max<double>(range.first, start_s), std::min<double>(range.second, end_s)});
  }
  // left_turn scene
  auto left_turn_ranges = reference_line.getSpecifiedDirectionRanges(DrivingDirection::kDirectionLeftOnly);
  for(const auto& range : left_turn_ranges) {
    if (range.second < start_s) continue;
    if (range.first > end_s) break;
    special_scene_s_range_sets_.emplace_back(
        std::tuple<string, double, double>{"LeftTurn", std::max<double>(range.first, start_s), std::min<double>(range.second, end_s)});
  }
  // right_turn scene
  auto right_turn_ranges = reference_line.getSpecifiedDirectionRanges(DrivingDirection::kDirectionRightOnly);
  for(const auto& range : right_turn_ranges) {
    if (range.second < start_s) continue;
    if (range.first > end_s) break;
    special_scene_s_range_sets_.emplace_back(
        std::tuple<string, double, double>{"RightTurn", std::max<double>(range.first, start_s), std::min<double>(range.second, end_s)});
  }

  // curve scene
  std::pair<double, double> curve_scene_s_range(std::make_pair(0.0, 0.0));
  bool in_curve_scene = false;
  for (int i = 0; i < reference_line.reference_points().size(); ++i) {
    double s = reference_line.reference_points().at(i).local_s();
    if (s < start_s) continue;
    if (s > end_s) {
      if (in_curve_scene) {
        in_curve_scene = false;
        curve_scene_s_range.second = s;
        special_scene_s_range_sets_.emplace_back(
            std::tuple<string, double, double>{"Curve", curve_scene_s_range.first, curve_scene_s_range.second});
      }
      break;
    }

    if (fabs(reference_line.reference_points().at(i).kappa()) > real_time_path_planner_config_.enter_curve_scene_kappa_threshold() &&
        !in_curve_scene) {
      in_curve_scene = true;
      curve_scene_s_range.first = s;
    } else if (fabs(reference_line.reference_points().at(i).kappa()) < real_time_path_planner_config_.exit_curve_scene_kappa_threshold() &&
               in_curve_scene) {
      in_curve_scene = false;
      curve_scene_s_range.second = s;
      special_scene_s_range_sets_.emplace_back(
          std::tuple<string, double, double>{"Curve", curve_scene_s_range.first, curve_scene_s_range.second});
    }
  }

  if (in_curve_scene) {
    in_curve_scene = false;
    curve_scene_s_range.second = reference_line.reference_points().back().local_s();
    special_scene_s_range_sets_.emplace_back(
        std::tuple<string, double, double>{"Curve", curve_scene_s_range.first, curve_scene_s_range.second});
  }

  return;
}

/**
 * @brief 更新特殊场景范围的调试信息
 * @details 该函数用于将特殊场景的范围信息（如狭窄场景、阻塞场景、弯道场景等）格式化为调试信息，并追加到`debug_info_`中。这些调试信息用于后续的日志记录和问题排查。
 * 
 * @par 关键变量说明:
 * - special_scene_s_range_sets_ (std::vector<std::tuple<string, double, double>>): 特殊场景的范围集合，包含场景标签、起始位置和结束位置
 * - debug_info_ (std::string): 调试信息字符串，用于存储格式化后的调试信息
 * 
 * @par 更新流程:
 * 1. 遍历特殊场景的范围集合
 * 2. 将每个场景的范围信息格式化为字符串
 * 3. 将格式化后的字符串追加到`debug_info_`中
 * 
 * @par 流程图:
 * @startuml
 * start
 * :遍历特殊场景的范围集合;
 * :将场景范围信息格式化为字符串;
 * :将格式化后的字符串追加到debug_info_中;
 * end
 * @enduml
 * 
 * @note 该函数应在每次路径规划循环中调用，确保调试信息正确更新
 * 
 * @warning 需确保`special_scene_s_range_sets_`已正确初始化，否则可能导致调试信息不完整
 */
void RealTimePathPlanner::updateTagRangesDebugInfo() {
  for (const auto& [tag, start_s, end_s] : special_scene_s_range_sets_) {
    debug_info_ += fmt::format("\n{}: <{:5.2f}, {:5.2f}> {:5.2f}", tag, start_s, end_s, adc_frenet_s_);
  }
}

/**
 * @brief 判断是否处于特定场景中
 * @details 该函数用于判断当前路径点是否处于指定的特殊场景（如狭窄场景、阻塞场景、弯道场景等）范围内。通过遍历特殊场景的范围集合，检查当前路径点是否在指定场景的范围内。
 * 
 * @param[in] scene_tag 场景标签，标识需要判断的场景类型（如"Narrow"、"Block"、"Curve"等）
 * @param[in] s 当前路径点的位置（沿参考线的纵向坐标）
 * 
 * @par 关键变量说明:
 * - special_scene_s_range_sets_ (std::vector<std::tuple<string, double, double>>): 特殊场景的范围集合，包含场景标签、起始位置和结束位置
 * - tag (string): 场景标签，标识当前场景类型
 * - start_s (double): 场景的起始位置
 * - end_s (double): 场景的结束位置
 * 
 * @par 判断流程:
 * 1. 遍历特殊场景的范围集合
 * 2. 检查当前场景标签是否与指定场景标签匹配
 * 3. 检查当前路径点是否在该场景的范围内
 * 4. 如果匹配且路径点在范围内，则返回true，否则继续遍历
 * 
 * @par 流程图:
 * @startuml
 * start
 * :遍历特殊场景的范围集合;
 * if (场景标签是否匹配?) then (是)
 *   if (路径点是否在场景范围内?) then (是)
 *     :返回true;
 *   endif
 * endif
 * :返回false;
 * end
 * @enduml
 * 
 * @return bool 是否处于指定场景中
 * @retval true 当前路径点处于指定场景中
 * @retval false 当前路径点不处于指定场景中
 * 
 * @note 该函数应在每次路径规划循环中调用，确保场景判断的准确性
 * 
 * @warning 需确保`special_scene_s_range_sets_`已正确初始化，否则可能导致判断错误
 */
bool RealTimePathPlanner::isInSpecificScene(const std::string& scene_tag, const float& s) {
  if (!special_scene_s_range_sets_.empty()) {
    for (int i = 0; i < special_scene_s_range_sets_.size(); ++i) {
      const auto& [tag, start_s, end_s] = special_scene_s_range_sets_.at(i);
      if (tag == scene_tag && s >= start_s && s <= end_s) {
        return true;
      }
    }
  }
  return false;
}



/**
 * @brief 计算车道保持的起始位置
 * @details 该函数用于计算车道保持的起始位置，根据当前车辆状态、决策结果和路径边界信息，确定车道保持的起始位置。该位置用于后续的路径规划和优化，确保车辆能够平稳地保持在车道内。
 * 
 * @param[in] decision_result 决策结果，包含当前车辆的决策信息
 * @param[in,out] path_data 路径数据，包含路径的边界信息和规划结果
 * 
 * @par 关键变量说明:
 * - current_ref (ReferenceLine): 当前参考线，包含当前车道的几何信息
 * - target_ref (ReferenceLine): 目标参考线，包含目标车道的几何信息
 * - boundary (PathBoundary): 路径边界，包含硬边界和软边界信息
 * - max_s (double): 最大路径长度，用于限制车道保持的起始位置
 * - res_s (double): 计算结果，表示车道保持的起始位置
 * - l0 (double): 当前车辆的横向位置
 * - dl0 (double): 当前车辆的横向速度
 * - whole_time (double): 车道保持的总时间
 * - upper_time (double): 换道时间的上限
 * - lower_time (double): 换道时间的下限
 * - upper_v (double): 换道速度的上限
 * - lower_v (double): 换道速度的下限
 * 
 * @par 计算流程:
 * 1. 获取当前参考线和目标参考线
 * 2. 获取路径边界信息
 * 3. 根据车辆状态和决策结果计算车道保持的起始位置
 * 4. 如果车辆处于换道状态，则根据换道时间和速度计算起始位置
 * 5. 如果车辆处于车道保持状态，则根据横向位置和速度计算起始位置
 * 6. 更新路径边界信息中的车道保持起始位置
 * 
 * @par 流程图:
 * @startuml
 * start
 * :获取当前参考线和目标参考线;
 * :获取路径边界信息;
 * if (车辆是否处于换道状态?) then (是)
 *   :根据换道时间和速度计算起始位置;
 * else (否)
 *   :根据横向位置和速度计算起始位置;
 * endif
 * :更新路径边界信息中的车道保持起始位置;
 * end
 * @enduml
 * 
 * @note 该函数应在每次路径规划循环中调用，确保车道保持起始位置正确计算
 * 
 * @warning 需确保输入参数有效，特别是decision_result和path_data
 */
void RealTimePathPlanner::calcLaneKeepStartS(const DecisionResult& decision_result, PathData* const path_data) {
  const auto &current_ref = current_ref_line_info_->ref_line();
  const auto &target_ref = target_ref_line_info_->ref_line();
  const auto &boundary = target_ref_line_info_->path_boundary();
  int boundary_size = boundary.size() - 1;
  double max_s = std::max<float>(0.0, boundary.delta_s() * (boundary_size <= 0 ? 150 : boundary_size) - adc_front_length_ - adc_rear_length_);

  double res_s = 0.0;
  double l0 = adc_sl_info_.second[0];
  double dl0 = adc_sl_info_.second[1];
  double whole_time = real_time_path_planner_config_.lane_keep_start_time();
  double upper_time = real_time_path_planner_config_.lane_change_time_upper(), lower_time = real_time_path_planner_config_.lane_change_time_lower();
  double upper_v = real_time_path_planner_config_.lane_change_velocity_upper(), lower_v = real_time_path_planner_config_.lane_change_velocity_lower();
  // 换道中.
  if (behavior_ == FsmState::LEFT_CHANGE || behavior_ == FsmState::RIGHT_CHANGE ) {
    gpal::pnc::SLPoint ego_sl_on_current, ego_sl_on_target;
    double current_left_bound = 0.0, current_right_bound = 0.0, target_left_bound = 0.0, target_right_bound = 0.0;
    current_ref.xy2sl(math::Vec3d(planning_start_point_.path_pt().x(), planning_start_point_.path_pt().y(), planning_start_point_.path_pt().z()), &ego_sl_on_current);
    target_ref.xy2sl(math::Vec3d(planning_start_point_.path_pt().x(), planning_start_point_.path_pt().y(), planning_start_point_.path_pt().z()), &ego_sl_on_target);
    current_ref.getLaneBound(ego_sl_on_current.s(), &current_left_bound, &current_right_bound);
    target_ref.getLaneBound(ego_sl_on_target.s(), &target_left_bound, &target_right_bound);
    double init_l = 0.0;
    if (behavior_ == FsmState::LEFT_CHANGE) {
      init_l = current_left_bound - target_right_bound;
    } else if (behavior_ == FsmState::RIGHT_CHANGE) {
      init_l = -current_right_bound + target_left_bound;
    } 
    
    // 换道时间. 30km/h ~ 130km/h 对应 3.0s ~ 5.5s
    if(real_time_path_planner_config_.enable_adaptive_lane_change_time()){
      whole_time = (std::clamp(planning_start_point_.v(), lower_v, upper_v) - lower_v) / (upper_v - lower_v) * (upper_time - lower_time) + lower_time;
    }
    double l_dot = fabs(init_l) / whole_time;
    double time_left = fabs(l0) / l_dot;
    res_s = std::min(time_left * planning_start_point_.v(), max_s);
  } else {
    if (behavior_ == FsmState::LEFT_HOLD || behavior_ == FsmState::RIGHT_HOLD || behavior_ == FsmState::LEFT_ATTEMPT || behavior_ == FsmState::RIGHT_ATTEMPT) {
      if (l_offset_behavior_valid_) {
        double target_l = decision_result.getRefTrajInfo()->traj_points.front().l;  // 与决策协定目前lane change hold
                                                                                    // 状态下不同s处l均相同
        l0 = l0 - target_l;
      }
    }
    // 其他场景.
    double v0 = fabs(planning_start_point_.v() * dl0); // lateral speed
    double a = real_time_path_planner_config_.lane_keep_lat_acc();
    double t0 = 0.0, t1 = 0.0, t = 0.0;
    if ((l0 > 0 && dl0 < 0) || (l0 < 0 && dl0 > 0)) {
      t0 = 0.0;
      double v1 = v0 * v0 + 2 * a * fabs(l0);
      t1 = sqrt(v1) / a;
    } else {
      t0 = v0 / a;
      double l1 = fabs(l0) + 0.5 * a * t0 * t0;
      t1 = sqrt(2 * l1 / a);
    }
    t = t0 + t1;
    res_s = std::min(max_s, t * std::fabs(planning_start_point_.v()));
    // ERT_PLOG_D << "v0 = " << v0
    //           << "  l0 = " << l0
    //           << "  v = " << planning_start_point_.v()
    //           << "  dl0 = " << dl0
    //           << "  t0 = " << t0
    //           << "  t1 = " << t1
    //           << "  t = " << t
    //           << "  max_s = " << max_s
    //           ;
  }

  target_ref_line_info_->mutable_path_boundary()->setLaneKeepStartS(adc_frenet_s_ + res_s);
  // ERT_PLOG_D << "[RealTimePathPlanner::calcLaneKeepStartS]: lane_keep_start_s = " << target_ref_line_info_->path_boundary().lane_keep_start_s()
  //           << " length = " << res_s ;
  
  debug_info_ += fmt::format("\nlane_keep_start_s: {:.2f}, time: {:.2f}, valid:{}, lat_acc: {:.2f}", res_s, whole_time,l_offset_behavior_valid_,real_time_path_planner_config_.lane_keep_lat_acc());
}



/**
 * @brief 处理路径优化器
 * @details 该函数用于处理路径优化器的执行，根据决策结果、速度数据和路径数据，调用优化器进行路径规划。该函数负责处理路径优化的异步执行、兜底场景的处理以及调试信息的更新。
 * 
 * @param[in] decision_result 决策结果，包含当前车辆的决策信息
 * @param[in] prev_speed_data 上一帧的速度数据，用于路径规划的初始条件
 * @param[in] time_stamp 当前时间戳，用于路径规划的时间同步
 * @param[in,out] path_data 路径数据，包含路径的边界信息和规划结果
 * 
 * @par 关键变量说明:
 * - ref_line_info (ReferenceLineInfo): 目标参考线信息，包含参考线的几何信息和路径边界
 * - optimizer_ (OcpPathOptimizer): 路径优化器，负责路径规划的核心逻辑
 * - async_status (OcpPathOptimizer::AsyncStatus): 异步执行状态，标识路径优化的执行状态
 * - fallback_status (FallbackStatus): 兜底场景状态，标识路径规划的兜底策略
 * - debug_status_ (std::set<PathData::DebugStatusType>): 调试状态集合，用于记录路径规划的调试信息
 * 
 * @par 处理流程:
 * 1. 设置优化器的特殊场景范围和参考线偏移信息
 * 2. 检查路径边界是否生成失败，若失败则直接使用参考线作为路径输出
 * 3. 调用优化器进行异步路径规划
 * 4. 根据异步执行状态进行兜底场景分类
 * 5. 根据兜底场景状态选择相应的路径生成策略
 * 6. 更新调试信息
 * 
 * @par 流程图:
 * @startuml
 * start
 * :设置优化器的特殊场景范围和参考线偏移信息;
 * if (路径边界是否生成失败?) then (是)
 *   :直接使用参考线作为路径输出;
 * else (否)
 *   :调用优化器进行异步路径规划;
 *   :根据异步执行状态进行兜底场景分类;
 *   switch (兜底场景状态)
 *     case (PROTECTION)
 *       :生成保护路径;
 *     case (UNCONSTRAINED)
 *       :生成无约束路径;
 *     case (HISTORY)
 *       :优化历史路径;
 *   endswitch
 * endif
 * :更新调试信息;
 * end
 * @enduml
 * 
 * @return Status 执行状态
 * @retval Status::OK 成功
 * @retval Status::ERROR 失败
 * 
 * @note 该函数应在每次路径规划循环中调用，确保路径规划的正确执行
 * 
 * @warning 需确保输入参数有效，特别是decision_result和path_data
 */
Status RealTimePathPlanner::processPathOptimizer(const DecisionResult& decision_result,
                                                 const SpeedData& prev_speed_data, const int64_t& time_stamp,
                                                 PathData* const path_data) {
  auto& ref_line_info = target_ref_line_info_;
  optimizer_.setSpecialSceneRange(special_scene_s_range_sets_);
  optimizer_.setRefOffsetsInfo(ref_offsets_info_);
  // debug info for reference line
  if (ref_line_info->ref_line().smooth_type() == ReferenceLine::SmoothType::RAW) {
    debug_status_.emplace(PathData::DebugStatusType::REF_RAW);
  } else if (ref_line_info->ref_line().smooth_type() == ReferenceLine::SmoothType::OCP) {
    debug_status_.emplace(PathData::DebugStatusType::REF_OCP);
  }

  if (ref_line_info->path_boundary().label().find("reference_line") != std::string::npos) {
    // 边界生成失败：则截取参考线作为最终的轨迹输出.
    ERT_PLOG_W << "[RealTimePathPlanner::processPathOptimizer]:fail to get boundary, load reference line" ;
    PathPlannerBase::runLoadRefLine(ref_line_info->ref_line(), ref_line_info->adc_planning_point(), path_data);
  } else {
    if (change_ref_ && optimizer_.hasAsyncProcess()) {
      optimizer_.dropAsyncProcess();
    }
    auto async_status = optimizer_.asyncProc(std::chrono::milliseconds(real_time_path_planner_config_.timeout_ms()), ref_line_info->ref_line(),
                                             planning_start_point_, ref_line_info->path_boundary(), path_data);
    // 横向兜底场景分类.
    FallbackStatus fallback_status = FallbackStatus::INVALID;
    if (async_status == OcpPathOptimizer::ASYNC_SOLVED && real_time_path_planner_config_.enable_solved_path_validity_check()) {
      // 1）若求解成功(SYNC_SOLVED)：进行轨迹有效性检验，判断轨迹是否超出硬边界，超出则UNCONSTRAINED(不考虑硬约束仅考虑软约束，规划起点与有约束轨迹一致).
      bool solved_path_validity_check_status = solvedPathValidityCheck(path_data->discretized_path(), 
                                                                       ref_line_info->path_boundary(),
                                                                       ref_line_info->ref_line());
      if (!solved_path_validity_check_status) {
        debug_status_.emplace(PathData::DebugStatusType::SOLVED_PATH_VALIDITY_CHECK);
        fallback_status = FallbackStatus::UNCONSTRAINED;
      }
    } else if (async_status != OcpPathOptimizer::ASYNC_SOLVED) {
      if (async_status == OcpPathOptimizer::ASYNC_ERROR_SOLVED) {
        debug_status_.emplace(PathData::DebugStatusType::SOLVER_WRONG_STEER_ANGEL);
      }

      // 2）若求解不成功：PREPROCESS_FAILED前处理失败，则PROTECTION(若PREPROCESS_FAILED则采用UNCONSTRAINED也会失败，因此用PROTECTION，但规划起点在自车当前位置，不利于横向控制跟踪).
      if (async_status == OcpPathOptimizer::PREPROCESS_FAILED) {
        fallback_status = FallbackStatus::PROTECTION;
      } else if (async_status == OcpPathOptimizer::INFEASIBLED) {
        fallback_status = FallbackStatus::UNCONSTRAINED;
      } else if (path_data->discretized_path().empty()) {
        // 3）若求解不成功：历史轨迹为空，则UNCONSTRAINED.
        debug_status_.emplace(PathData::DebugStatusType::PREV_PATH_INVALID);
        fallback_status = FallbackStatus::UNCONSTRAINED;
      } else {
        if (change_ref_ || (pre_async_status_ == OcpPathOptimizer::ASYNC_UNDERLOCKING && pre_change_ref_)) {
          // 4）若求解不成功：本帧参考线变化，或上帧参考线变化且异步求解中(ASYNC_UNDERLOCKING)，则HISTORY.
          debug_status_.emplace(PathData::DebugStatusType::REF_CHANGED);
          fallback_status = FallbackStatus::HISTORY; // FallbackStatus::UNCONSTRAINED;
        } else if (async_status == OcpPathOptimizer::ASYNC_UNDERLOCKING) {
          // 5）若求解不成功：异步求解中(ASYNC_UNDERLOCKING)，只是需要时间，则HISTORY.
          debug_status_.emplace(PathData::DebugStatusType::SOLVER_UNDERLOCK_REFINE);
          fallback_status = FallbackStatus::HISTORY;
        } else {
          SLPoint sl_pt;
          ref_line_info->ref_line().xy2sl(path_data->discretized_path().back(), &sl_pt);
          // 6）若求解不成功：历史轨迹过短，则UNCONSTRAINED.
          if (sl_pt.s() - adc_frenet_s_ < real_time_path_planner_config_.min_refine_length() && sl_pt.s() < adc_frenet_end_s_) {
            debug_status_.emplace(PathData::DebugStatusType::PREV_PATH_TOO_SHORT);
            fallback_status = FallbackStatus::UNCONSTRAINED; // FallbackStatus::PROTECTION;
          } else {
            // 7）若求解不成功：自车与上帧轨迹横向偏差或航向偏差过大，则UNCONSTRAINED.
            if (real_time_path_planner_config_.enable_local_path_when_path_too_far() &&
                !path_data->discretized_path().empty()) {
              const double road_error_l = CalculateRoadError(PathPt(ref_line_info->vehicle_state().x(), 
                                                                    ref_line_info->vehicle_state().y(),
                                            ref_line_info->vehicle_state().z()),
                                                             path_data->discretized_path()).f_rear_road_error;
              const double road_error_heading = math::NormalizeAngle(
                  path_data->discretized_path().front().theta() - ref_line_info->vehicle_state().yaw());
              if (std::abs(road_error_l) > real_time_path_planner_config_.l_thrd_not_refine_path() ||
                  std::abs(road_error_heading) > real_time_path_planner_config_.heading_thrd_not_refine_path() * ANG2RAD) { 
                debug_status_.emplace(PathData::DebugStatusType::PREV_PATH_TOO_FAR_REFINE_FAILED);
                fallback_status = FallbackStatus::UNCONSTRAINED;
              }
              else {
                debug_status_.emplace(PathData::DebugStatusType::SOLVER_FAILED_REFINE);
                fallback_status = FallbackStatus::HISTORY;
              }
            } else {
              // 8）若求解不成功：其他场景，则HISTORY.
              debug_status_.emplace(PathData::DebugStatusType::SOLVER_FAILED_REFINE);
              fallback_status = FallbackStatus::HISTORY;
            }
          }
        }
      }
    }

    // 基于横向兜底场景的处理.
    // PROTECTION 兜底轨迹：
    //     1) 采用BicycleTrajectoryTracker ocp模型，模拟横向控制的行驶轨迹，但规划起点在自车当前位置，不利于横向控制跟踪.
    //     2) 若1）失败导致横向轨迹为空时，返回fall back结果，即横向规划的初解(第一帧为参考线，其他为上帧横向轨迹)，此方法优先级最低
    // UNCONSTRAINED 无约束求解轨迹：模型及规划起点与有约束轨迹一致，但无横向硬边界约束，仅考虑软边界约束，利于横向控制跟踪；优先级最高.
    // HISTORY 历史轨迹：refine path
    switch (fallback_status) {
      case FallbackStatus::PROTECTION: {
        // ocp模型变更为BicycleTrajectoryTracker
        generateProtectPath(prev_speed_data, time_stamp, path_data);
        debug_status_.emplace(PathData::DebugStatusType::SOLVER_PROTECT);
      } break;

      case FallbackStatus::UNCONSTRAINED: {
        // ocp模型仍为“LateralGeneral”，但相比主线规划，仅考虑软边界（体现在initUnconstrainedLateralGeneral()中）.
        generateUnconstrainedPath(path_data);
        debug_status_.emplace(PathData::DebugStatusType::SOLVER_UNCONSTRAINED);
      } break;

      case FallbackStatus::HISTORY: {
        optimizer_.refinePath(planning_start_point_, path_data);
      } break;

      default:
        break;
    }
    // 最终检查，采用PROTECTION.
    if (path_data->discretized_path().empty()) {
      generateProtectPath(prev_speed_data, time_stamp, path_data);
      fallback_status = FallbackStatus::PROTECTION;
      debug_status_.emplace(PathData::DebugStatusType::SOLVER_PROTECT);
    }
    debug_info_ += fmt::format("\nasync: {}", static_cast<int>(async_status)) + optimizer_.getAsyncPlannerDebugInfo();

    pre_async_status_ = async_status;
  }

  return Status::OK();
}

/**
 * @brief 检查求解路径的有效性
 * @details 该函数用于检查求解的路径是否在硬边界范围内。通过遍历路径点，检查每个路径点的横向位置是否超出硬边界，确保路径的安全性和可行性。
 * 
 * @param[in] path 求解的路径，包含路径点的位置信息
 * @param[in] boundary 路径边界，包含硬边界和软边界信息
 * @param[in] ref_line 参考线，用于将路径点从笛卡尔坐标系转换到Frenet坐标系
 * 
 * @par 关键变量说明:
 * - barrier_path_boundary (util::AbstractTable1d<double, double, double>): 硬边界信息，包含路径点的位置、左侧边界和右侧边界
 * - sl_point (SLPoint): Frenet坐标系下的路径点，包含纵向位置和横向位置
 * 
 * @par 检查流程:
 * 1. 初始化硬边界信息
 * 2. 遍历路径点，将路径点从笛卡尔坐标系转换到Frenet坐标系
 * 3. 检查路径点的横向位置是否超出硬边界
 * 4. 如果路径点超出硬边界，则返回false，否则继续检查
 * 
 * @par 流程图:
 * @startuml
 * start
 * :初始化硬边界信息;
 * :遍历路径点;
 * :将路径点从笛卡尔坐标系转换到Frenet坐标系;
 * if (路径点是否超出硬边界?) then (是)
 *   :返回false;
 * endif
 * :返回true;
 * end
 * @enduml
 * 
 * @return bool 路径是否有效
 * @retval true 路径有效，未超出硬边界
 * @retval false 路径无效，超出硬边界
 * 
 * @note 该函数应在每次路径规划循环中调用，确保路径的有效性
 * 
 * @warning 需确保输入参数有效，特别是path和boundary
 */
bool RealTimePathPlanner::solvedPathValidityCheck(const DiscretizedPath& path, const PathBoundary& boundary,
                                                  const ReferenceLine& ref_line) {
  util::AbstractTable1d<double, double, double> barrier_path_boundary;
  barrier_path_boundary.assign(boundary.barrier_boundary().begin(), boundary.barrier_boundary().end());
  for (int i = 0; i < path.size(); ++i) {
    auto [barrier_boundary, valid] = barrier_path_boundary.interpolate(path.at(i).s() + adc_frenet_s_);
    if (valid) {
      auto [s, lower, upper] = barrier_boundary;
      SLPoint sl_point;
      if (ref_line.xy2sl(math::Vec3d(path.at(i).x(), path.at(i).y(), path.at(i).z()), &sl_point)) {
        if (sl_point.l() > upper + 0.2 || sl_point.l() < lower - 0.2) {
		      ERT_PLOG_I << "[bypass][solvedPathValidityCheck]: invalid! s = "<< s 
               << "  barrier_lower = " << lower
               << "  barrier_upper = " << upper
               << "  path_l = " << sl_point.l();
          return false;
        }
      }
    }
  }

  return true;
}

/**
 * @brief 生成无约束路径
 * @details 该函数用于生成无约束的路径，即在路径规划过程中不考虑硬边界约束，仅考虑软边界约束。该函数调用路径优化器的无约束处理函数，生成一条无约束的路径。
 * 
 * @param[in,out] path_data 路径数据，包含路径的边界信息和规划结果
 * 
 * @par 关键变量说明:
 * - optimizer_ (OcpPathOptimizer): 路径优化器，负责路径规划的核心逻辑
 * - target_ref_line_info_ (ReferenceLineInfo): 目标参考线信息，包含参考线的几何信息和路径边界
 * - planning_start_point_ (PathPoint): 规划起点，包含路径规划的初始条件
 * 
 * @par 生成流程:
 * 1. 调用路径优化器的无约束处理函数，生成无约束路径
 * 2. 如果路径生成失败，则清空路径数据
 * 
 * @par 流程图:
 * @startuml
 * start
 * :调用路径优化器的无约束处理函数;
 * if (路径生成是否成功?) then (否)
 *   :清空路径数据;
 * endif
 * end
 * @enduml
 * 
 * @note 该函数应在路径规划过程中调用，确保无约束路径的正确生成
 * 
 * @warning 需确保输入参数有效，特别是path_data
 */
void RealTimePathPlanner::generateUnconstrainedPath(PathData* const path_data) {
  
  auto status = optimizer_.unconstrainedProc(target_ref_line_info_->ref_line(), 
                                             planning_start_point_, 
                                             target_ref_line_info_->path_boundary(), 
                                             path_data);

  if(!status.ok()) {
    path_data->mutableDiscretizedPath()->clear();
  }
}

/**
 * @brief 生成保护路径
 * @details 该函数用于在路径规划失败或需要兜底时生成保护路径。保护路径通常用于确保车辆在紧急情况下能够安全行驶。该函数会优先尝试生成局部保护路径，如果失败则使用优化器的兜底结果。
 * 
 * @param[in] prev_speed_data 上一帧的速度数据，用于路径规划的初始条件
 * @param[in] time_stamp 当前时间戳，用于路径规划的时间同步
 * @param[in,out] path_data 路径数据，包含路径的边界信息和规划结果
 * 
 * @par 关键变量说明:
 * - ref_line_info (ReferenceLineInfo): 目标参考线信息，包含参考线的几何信息和路径边界
 * - local_path (DiscretizedPath): 局部保护路径，用于在紧急情况下提供安全路径
 * - optimizer_ (OcpPathOptimizer): 路径优化器，负责路径规划的核心逻辑
 * 
 * @par 生成流程:
 * 1. 获取目标参考线信息
 * 2. 调用路径优化器生成局部保护路径
 * 3. 如果局部保护路径生成成功，则更新路径数据
 * 4. 如果局部保护路径生成失败，则使用优化器的兜底结果
 * 
 * @par 流程图:
 * @startuml
 * start
 * :获取目标参考线信息;
 * :调用路径优化器生成局部保护路径;
 * if (局部保护路径是否生成成功?) then (是)
 *   :更新路径数据;
 * else (否)
 *   :使用优化器的兜底结果;
 * endif
 * end
 * @enduml
 * 
 * @note 该函数应在路径规划失败或需要兜底时调用，确保车辆能够安全行驶
 * 
 * @warning 需确保输入参数有效，特别是path_data
 */
void RealTimePathPlanner::generateProtectPath(const SpeedData& prev_speed_data, const int64_t& time_stamp, PathData* const path_data) {
  auto& ref_line_info = target_ref_line_info_;
  PathPlannerBase::runLoadRefLine(ref_line_info->ref_line(), ref_line_info->adc_planning_point(), path_data);
  auto local_path =  optimizer_.generateProtectPath(ref_line_info->vehicle_state(), ref_line_info->ref_line(), prev_speed_data,
                                                    path_data->discretized_path(), time_stamp);
  if (!local_path.empty()) {
    *path_data->mutableDiscretizedPath() = std::move(local_path);
  } else if (optimizer_.getFallbackResult()->discretized_path().size() > 1) {
    *path_data->mutableDiscretizedPath() = std::move(optimizer_.getFallbackResult()->discretized_path());
  }
}

/**
 * @brief 判断决策对象是否为行人
 * @details 该函数用于判断给定的决策对象是否为行人。通过检查决策对象的类型，确定其是否为行人类型。
 * 
 * @param[in] obs 决策对象，包含决策对象的类型信息
 * 
 * @par 关键变量说明:
 * - obs.type (Decision::ObjectType): 决策对象的类型，用于判断是否为行人
 * 
 * @par 判断流程:
 * 1. 检查决策对象的类型是否为行人类型
 * 2. 如果是行人类型，则返回true，否则返回false
 * 
 * @par 流程图:
 * @startuml
 * start
 * :检查决策对象的类型是否为行人类型;
 * if (是否为行人类型?) then (是)
 *   :返回true;
 * else (否)
 *   :返回false;
 * endif
 * end
 * @enduml
 * 
 * @return bool 是否为行人
 * @retval true 决策对象是行人
 * @retval false 决策对象不是行人
 * 
 * @note 该函数应在每次决策对象处理时调用，确保行人判断的准确性
 * 
 * @warning 需确保输入参数有效，特别是obs
 */
bool RealTimePathPlanner::isPedestrian(const Decision::DecisionObject& obs) {
  bool isPed = obs.type == Decision::ObjectType::PEDESTRIAN;
  return isPed;
}

/**
 * @brief 碰撞后处理
 * @details 该函数用于在路径规划后处理碰撞检测结果。通过将路径点从地图坐标系转换到自车坐标系，进行碰撞检测，并将检测结果转换回地图坐标系，更新路径数据中的碰撞信息。
 * 
 * @param[in] freespace 自由空间信息，包含障碍物和可行驶区域的信息
 * @param[in] loc 定位信息，包含自车的位置和姿态信息
 * @param[in,out] path_data 路径数据，包含路径的边界信息和规划结果
 * 
 * @par 关键变量说明:
 * - ego_path (std::vector<PathPt>): 自车坐标系下的路径点集合
 * - tf_map_2_ego (math::Transform): 地图坐标系到自车坐标系的变换矩阵
 * - block_fs_info (BlockFSInfo): 碰撞检测结果，包含碰撞点的位置和方向信息
 * - tf_ego_2_map (math::Transform): 自车坐标系到地图坐标系的变换矩阵
 * 
 * @par 处理流程:
 * 1. 检查是否启用碰撞检测，若未启用则直接返回
 * 2. 将路径点从地图坐标系转换到自车坐标系
 * 3. 进行碰撞检测，获取碰撞检测结果
 * 4. 如果检测到碰撞，则将碰撞点从自车坐标系转换回地图坐标系
 * 5. 更新路径数据中的碰撞信息
 * 
 * @par 流程图:
 * @startuml
 * start
 * if (是否启用碰撞检测?) then (否)
 *   :直接返回;
 * endif
 * :将路径点从地图坐标系转换到自车坐标系;
 * :进行碰撞检测，获取碰撞检测结果;
 * if (是否检测到碰撞?) then (是)
 *   :将碰撞点从自车坐标系转换回地图坐标系;
 *   :更新路径数据中的碰撞信息;
 * endif
 * end
 * @enduml
 * 
 * @note 该函数应在每次路径规划循环中调用，确保碰撞检测结果正确更新
 * 
 * @warning 需确保输入参数有效，特别是freespace和loc
 */
void RealTimePathPlanner::CollisionPostProcess(const Freespace& freespace, const std::shared_ptr<Localization> loc, PathData* const path_data) {
  if (!real_time_path_planner_config_.enable_fs_collision_check()) {
    return;
  }

  // path: map->ego
  std::vector<PathPt> ego_path;
  auto tf_map_2_ego = loc->getTfMap2Ego();
  for (const auto& path_pt : path_data->discretized_path()) {
    math::Vec3d flu_position_vec3d(path_pt.x(), path_pt.y(), path_pt.z());
    transfer::transformPoint(tf_map_2_ego, &flu_position_vec3d);
    math::Vec3d flu_rpy_vec3d(0.0, 0.0, path_pt.theta());
    transfer::transformRPY(tf_map_2_ego, &flu_rpy_vec3d);

    ego_path.emplace_back(path_pt);
    ego_path.back().set_x(flu_position_vec3d.x());
    ego_path.back().set_y(flu_position_vec3d.y());
    ego_path.back().set_z(flu_position_vec3d.z());
    ego_path.back().set_theta(flu_rpy_vec3d.z());
  }

  auto block_fs_info = collisionCheck(freespace, ego_path,
                                      real_time_path_planner_config_.collision_check_buffer(),
                                      real_time_path_planner_config_.corner_width(),
                                      real_time_path_planner_config_.enable_curve_decide_process(),
                                      real_time_path_planner_config_.curve_look_ahead_distance(),
                                      real_time_path_planner_config_.curve_kappa_thresold(),                           
                                      real_time_path_planner_config_.side_box_length(),
                                      real_time_path_planner_config_.side_box_width());
  if(block_fs_info.is_valid) {
    block_fs_info.path_type = PathData::PathType::LATERAL;
    block_fs_info.block_point_direction = getBlockFsPointDirection(block_fs_info.check_point, block_fs_info.block_point);

    // ego->map
    auto tf_ego_2_map = loc->getTfEgo2Map();
    math::Vec3d map_check_point(block_fs_info.check_point.x(), block_fs_info.check_point.y(), block_fs_info.check_point.z());
    transfer::transformPoint(tf_ego_2_map, &map_check_point);
    block_fs_info.check_point = PathPt(map_check_point.x(), map_check_point.y(), map_check_point.z());
    math::Vec3d map_block_point(block_fs_info.block_point.x(), block_fs_info.block_point.y(), 0.0);
    transfer::transformPoint(tf_ego_2_map, &map_block_point);
    block_fs_info.block_point = math::Vec2d(map_block_point.x(), map_block_point.y());
    std::vector<math::Vec2d> map_vis_pts;
    for(const auto& vis_pt : block_fs_info.vis_pts) {
      math::Vec3d map_vis_pt(vis_pt.x(), vis_pt.y(), 0.0);
      transfer::transformPoint(tf_ego_2_map, &map_vis_pt);
      map_vis_pts.emplace_back(map_vis_pt.x(), map_vis_pt.y());
    }
    block_fs_info.vis_pts = std::move(map_vis_pts);

    path_data->mutableBlockFSInfo()->emplace_back(block_fs_info);
    path_data->mutablePlannerDebugStatus()->push_back(PathData::DebugStatusType::BLOCK_FS_POST);
  }

  ERT_PLOG_D << "[RealTimePathPlanner::CollisionPostProcess]: blockFSInfo().size = " << path_data->blockFSInfo().size()
            ;
  for(const auto& block_fs : path_data->blockFSInfo()) {
    ERT_PLOG_D << "is_valid = " << block_fs.is_valid
              << "  s = " << block_fs.s
              << "  type = " << static_cast<int>(block_fs.path_type)
              << "  direction = " << static_cast<int>(block_fs.block_point_direction)
              << " block x = " << block_fs.block_point.x()
              << "  y = " << block_fs.block_point.y()
              << "  check x = " << block_fs.check_point.x()
              << "  y = " << block_fs.check_point.y()
              << "  z = " << block_fs.check_point.z()
              ;
  }
}

/**
 * @brief 转换边界点类型为路径边界单元类型
 * @details 该函数用于将决策边界点类型（BoundaryPointTypeInfo）转换为路径边界单元类型（BoundaryUnitTypeInfo）。通过判断输入的决策边界点类型，返回对应的路径边界单元类型。
 * 
 * @param[in] decision_type 决策边界点类型，包含边界点的类型信息
 * 
 * @par 关键变量说明:
 * - decision_type (BoundaryPointTypeInfo): 决策边界点类型，用于判断边界点的类型
 * 
 * @par 转换流程:
 * 1. 判断决策边界点类型是否为虚线车道线
 * 2. 判断决策边界点类型是否为实线车道线
 * 3. 判断决策边界点类型是否为物理不可通过边界
 * 4. 如果以上条件均不满足，则返回无效类型
 * 
 * @par 流程图:
 * @startuml
 * start
 * if (决策边界点类型是否为虚线车道线?) then (是)
 *   :返回虚线车道线类型;
 * else if (决策边界点类型是否为实线车道线?) then (是)
 *   :返回实线车道线类型;
 * else if (决策边界点类型是否为物理不可通过边界?) then (是)
 *   :返回物理不可通过边界类型;
 * else (否)
 *   :返回无效类型;
 * endif
 * end
 * @enduml
 * 
 * @return PathBoundary::BoundaryUnitTypeInfo 路径边界单元类型
 * @retval PathBoundary::BoundaryUnitTypeInfo::DASHED_LANE_LINE 虚线车道线类型
 * @retval PathBoundary::BoundaryUnitTypeInfo::SOLID_LANE_LINE 实线车道线类型
 * @retval PathBoundary::BoundaryUnitTypeInfo::PHYSICALLY_IMPASSABLE 物理不可通过边界类型
 * @retval PathBoundary::BoundaryUnitTypeInfo::INVALID 无效类型
 * 
 * @note 该函数应在每次处理决策边界点时调用，确保边界点类型正确转换
 * 
 * @warning 需确保输入参数有效，特别是decision_type
 */
PathBoundary::BoundaryUnitTypeInfo RealTimePathPlanner::ConvertBoundaryType(BoundaryPointTypeInfo decision_type) {
  switch (decision_type) {
    case BoundaryPointTypeInfo::DASHED_LANE_LINE:
      return PathBoundary::BoundaryUnitTypeInfo::DASHED_LANE_LINE;
    case BoundaryPointTypeInfo::SOLID_LANE_LINE:
      return PathBoundary::BoundaryUnitTypeInfo::SOLID_LANE_LINE;
    case BoundaryPointTypeInfo::PHYSICALLY_IMPASSABLE:
      return PathBoundary::BoundaryUnitTypeInfo::PHYSICALLY_IMPASSABLE;
    case BoundaryPointTypeInfo::INVALID:
    default:
      return PathBoundary::BoundaryUnitTypeInfo::INVALID;
  }
}

/**
 * @brief 更新参考线偏移信息
 * @details 该函数用于根据决策结果更新参考线的偏移信息。通过分析决策结果中的横向边界决策和参考轨迹点，计算并更新参考线的偏移信息，确保路径规划时能够正确考虑横向偏移。
 * 
 * @param[in] decision_result 决策结果，包含当前车辆的决策信息
 * @param[in] condition 更新条件，用于控制是否执行更新操作
 * 
 * @par 关键变量说明:
 * - offsets_info (std::vector<std::pair<double, double>>): 偏移信息集合，包含路径点的位置和偏移量
 * - ref_traj_points (std::vector<PathPoint>): 参考轨迹点集合，包含参考轨迹点的位置信息
 * - traj_converted (std::vector<std::pair<double, double>>): 转换后的轨迹点集合，包含路径点的位置和偏移量
 * - ref_offsets_info_ (std::vector<std::pair<double, double>>): 参考线偏移信息集合，包含路径点的位置和偏移量
 * 
 * @par 更新流程:
 * 1. 检查决策结果是否有效，若无效则直接返回
 * 2. 初始化偏移信息集合
 * 3. 获取参考轨迹点集合
 * 4. 计算参考轨迹点的起始位置和结束位置
 * 5. 根据起始位置和结束位置裁剪偏移信息集合
 * 6. 将参考轨迹点转换为偏移信息集合
 * 7. 合并偏移信息集合和转换后的轨迹点集合
 * 8. 更新参考线偏移信息集合
 * 
 * @par 流程图:
 * @startuml
 * start
 * if (决策结果是否有效?) then (否)
 *   :直接返回;
 * endif
 * :初始化偏移信息集合;
 * :获取参考轨迹点集合;
 * :计算参考轨迹点的起始位置和结束位置;
 * :根据起始位置和结束位置裁剪偏移信息集合;
 * :将参考轨迹点转换为偏移信息集合;
 * :合并偏移信息集合和转换后的轨迹点集合;
 * :更新参考线偏移信息集合;
 * end
 * @enduml
 * 
 * @note 该函数应在每次路径规划循环中调用，确保参考线偏移信息正确更新
 * 
 * @warning 需确保输入参数有效，特别是decision_result
 */
void RealTimePathPlanner::updateRefOffsetInfo(const DecisionResult& decision_result, bool condition) {
  if (decision_result.getRefTrajInfo() == nullptr || decision_result.getLateralBoundaryDecision().empty() || !condition) {
    return;
  }

  std::vector<std::pair<double, double>> offsets_info;
  //以决策边界size为参照
  for (auto pt : decision_result.getLateralBoundaryDecision().front().points) {
    offsets_info.emplace_back(pt.s, 0.0);
  }

  auto ref_traj_points = decision_result.getRefTrajInfo()->traj_points;

  double s_start = ref_traj_points.front().s;
  double s_end = ref_traj_points.back().s;

  auto lower_compare = [](const std::pair<double, double>& elem, double s) { return elem.first < s; };
  auto upper_compare = [](double s, const std::pair<double, double>& elem) { return s < elem.first; };

  auto start_it = std::lower_bound(offsets_info.begin(), offsets_info.end(), s_start, lower_compare);
  auto end_it = std::upper_bound(offsets_info.begin(), offsets_info.end(), s_end, upper_compare);

  offsets_info.erase(start_it, end_it);

  std::vector<std::pair<double, double>> traj_converted;
  traj_converted.reserve(ref_traj_points.size());
  for (const auto& point : ref_traj_points) {
    traj_converted.emplace_back(point.s, point.l);
  }

  auto split_pos = std::upper_bound(offsets_info.begin(), offsets_info.end(), s_start, upper_compare);

  std::vector<std::pair<double, double>> res;
  res.reserve(offsets_info.size() + traj_converted.size());

  res.insert(res.end(), offsets_info.begin(), split_pos);
  res.insert(res.end(), traj_converted.begin(), traj_converted.end());
  res.insert(res.end(), split_pos, offsets_info.end());
  ref_offsets_info_ = std::move(res);
  return;
}

/**
 * @brief 可视化路径边界信息
 * @details 该函数用于将路径的边界信息（如硬边界、软边界等）转换为可视化数据，并存储到路径数据中。通过将边界点从Frenet坐标系转换到笛卡尔坐标系，生成用于可视化的路径边界点集合。
 * 
 * @param[in,out] path_data 路径数据，包含路径的边界信息和规划结果
 * 
 * @par 关键变量说明:
 * - bounds_vec3d (PathData::BoundsVec3d): 路径边界点集合，包含硬边界和软边界的左右边界点
 * - bounds_vec3d_with_id (std::vector<PathData::BoundsVec3dWithId>): 带标识的路径边界点集合，用于区分不同类型的边界
 * - barrier_boundary (std::vector<std::tuple<double, double, double>>): 硬边界信息，包含路径点的位置、左侧边界和右侧边界
 * - soft_boundary (std::vector<std::tuple<double, double, double>>): 软边界信息，包含路径点的位置、左侧边界和右侧边界
 * 
 * @par 处理流程:
 * 1. 清空路径边界点集合
 * 2. 遍历硬边界，将边界点从Frenet坐标系转换到笛卡尔坐标系，并存储到路径边界点集合中
 * 3. 遍历软边界，将边界点从Frenet坐标系转换到笛卡尔坐标系，并存储到路径边界点集合中
 * 4. 初始化带标识的路径边界点集合，并存储不同类型的边界信息
 * 
 * @par 流程图:
 * @startuml
 * start
 * :清空路径边界点集合;
 * :遍历硬边界;
 * :将硬边界点从Frenet坐标系转换到笛卡尔坐标系;
 * :存储硬边界点到路径边界点集合中;
 * :遍历软边界;
 * :将软边界点从Frenet坐标系转换到笛卡尔坐标系;
 * :存储软边界点到路径边界点集合中;
 * :初始化带标识的路径边界点集合;
 * :存储不同类型的边界信息;
 * end
 * @enduml
 * 
 * @note 该函数应在每次路径规划循环中调用，确保路径边界信息正确可视化
 * 
 * @warning 需确保输入参数有效，特别是path_data
 */
void RealTimePathPlanner::visualization(PathData* const path_data) {
  // display data
  auto bounds_vec3d = path_data->mutableBoundsVec3d();
  bounds_vec3d->clear();
  // ERT_PLOG_D << "[bypass][RealTimePathPlanner::visualization]: start_s = " <<
  // target_ref_line_info_->path_boundary().start_s()
  //           << "  end_s = " << target_ref_line_info_->path_boundary().end_s()
  //           << "  size = " << target_ref_line_info_->path_boundary().size()
  //           << "  delta_s = " << target_ref_line_info_->path_boundary().delta_s()
  //           << "  length = " << target_ref_line_info_->path_boundary().length() ;
  // ERT_PLOG_D << "[bypass]barrier_size = " << target_ref_line_info_->path_boundary().barrier_boundary().size()
  //           << "  soft_size = " << target_ref_line_info_->path_boundary().soft_boundary().size()
  //           << "  decision_size = " << target_ref_line_info_->path_boundary().decision_boundary().size();
  // for(int i = 0; i < std::min<int>(20, target_ref_line_info_->path_boundary().barrier_boundary().size()); ++i) {
  //   ERT_PLOG_D << "[bypass]i = " << i ;
  //   ERT_PLOG_D << "   [bypass]barrier: s = " <<
  //   std::get<0>(target_ref_line_info_->path_boundary().barrier_boundary().at(i))
  //             << "   l_min = " << std::get<1>(target_ref_line_info_->path_boundary().barrier_boundary().at(i))
  //             << "   l_max = " << std::get<2>(target_ref_line_info_->path_boundary().barrier_boundary().at(i)) ;
  //   ERT_PLOG_D << "   [bypass]soft: s = " << std::get<0>(target_ref_line_info_->path_boundary().soft_boundary().at(i))
  //             << "   l_min = " << std::get<1>(target_ref_line_info_->path_boundary().soft_boundary().at(i))
  //             << "   l_max = " << std::get<2>(target_ref_line_info_->path_boundary().soft_boundary().at(i)) ;
  // }

  for (const auto& barrier_bound : target_ref_line_info_->path_boundary().barrier_boundary()) {
    gpal::pnc::SLPoint sl;
    sl.set_s(std::get<0>(barrier_bound));
    sl.set_l(std::get<1>(barrier_bound));
    bounds_vec3d->barrier_bound_right.emplace_back();
    target_ref_line_info_->ref_line().sl2xy(sl, &bounds_vec3d->barrier_bound_right.back());
    sl.set_l(std::get<2>(barrier_bound));
    bounds_vec3d->barrier_bound_left.emplace_back();
    target_ref_line_info_->ref_line().sl2xy(sl, &bounds_vec3d->barrier_bound_left.back());
  }
  for (const auto& soft_bound : target_ref_line_info_->path_boundary().soft_boundary()) {
    gpal::pnc::SLPoint sl;
    sl.set_s(std::get<0>(soft_bound));
    sl.set_l(std::get<1>(soft_bound));
    bounds_vec3d->soft_bound_right.emplace_back();
    target_ref_line_info_->ref_line().sl2xy(sl, &bounds_vec3d->soft_bound_right.back());
    sl.set_l(std::get<2>(soft_bound));
    bounds_vec3d->soft_bound_left.emplace_back();
    target_ref_line_info_->ref_line().sl2xy(sl, &bounds_vec3d->soft_bound_left.back());
  }

  auto bounds_vec3d_with_id = path_data->mutableBoundsVec3dWithId();
  PathData::BoundsVec3dWithId bound_with_id;
  // for prior_physical
  bound_with_id.id = "prior_physical_boundary";
  for (const auto& barrier_bound : target_ref_line_info_->path_boundary().prior_physical_barrier_boundary()) {
    gpal::pnc::SLPoint sl;
    sl.set_s(std::get<0>(barrier_bound));
    sl.set_l(std::get<1>(barrier_bound));
    bound_with_id.barrier_bound_right.emplace_back();
    target_ref_line_info_->ref_line().sl2xy(sl, &bound_with_id.barrier_bound_right.back());
    sl.set_l(std::get<2>(barrier_bound));
    bound_with_id.barrier_bound_left.emplace_back();
    target_ref_line_info_->ref_line().sl2xy(sl, &bound_with_id.barrier_bound_left.back());
  }
  for (const auto& soft_bound : target_ref_line_info_->path_boundary().prior_physical_soft_boundary()) {
    gpal::pnc::SLPoint sl;
    sl.set_s(std::get<0>(soft_bound));
    sl.set_l(std::get<1>(soft_bound));
    bound_with_id.soft_bound_right.emplace_back();
    target_ref_line_info_->ref_line().sl2xy(sl, &bound_with_id.soft_bound_right.back());
    sl.set_l(std::get<2>(soft_bound));
    bound_with_id.soft_bound_left.emplace_back();
    target_ref_line_info_->ref_line().sl2xy(sl, &bound_with_id.soft_bound_left.back());
  }
  bounds_vec3d_with_id->emplace_back(bound_with_id);
  // for static_od
  bound_with_id.clear();
  bound_with_id.id = "static_od_boundary";
  for (const auto& barrier_bound : target_ref_line_info_->path_boundary().static_od_barrier_boundary()) {
    gpal::pnc::SLPoint sl;
    sl.set_s(std::get<0>(barrier_bound));
    sl.set_l(std::get<1>(barrier_bound));
    bound_with_id.barrier_bound_right.emplace_back();
    target_ref_line_info_->ref_line().sl2xy(sl, &bound_with_id.barrier_bound_right.back());
    sl.set_l(std::get<2>(barrier_bound));
    bound_with_id.barrier_bound_left.emplace_back();
    target_ref_line_info_->ref_line().sl2xy(sl, &bound_with_id.barrier_bound_left.back());
  }
  for (const auto& soft_bound : target_ref_line_info_->path_boundary().static_od_soft_boundary()) {
    gpal::pnc::SLPoint sl;
    sl.set_s(std::get<0>(soft_bound));
    sl.set_l(std::get<1>(soft_bound));
    bound_with_id.soft_bound_right.emplace_back();
    target_ref_line_info_->ref_line().sl2xy(sl, &bound_with_id.soft_bound_right.back());
    sl.set_l(std::get<2>(soft_bound));
    bound_with_id.soft_bound_left.emplace_back();
    target_ref_line_info_->ref_line().sl2xy(sl, &bound_with_id.soft_bound_left.back());
  }
  bounds_vec3d_with_id->emplace_back(bound_with_id);
  // for dynamic_od
  bound_with_id.clear();
  bound_with_id.id = "dynamic_od_boundary";
  for (const auto& soft_bound : target_ref_line_info_->path_boundary().dynamic_od_soft_boundary()) {
    gpal::pnc::SLPoint sl;
    sl.set_s(std::get<0>(soft_bound));
    sl.set_l(std::get<1>(soft_bound));
    bound_with_id.soft_bound_right.emplace_back();
    target_ref_line_info_->ref_line().sl2xy(sl, &bound_with_id.soft_bound_right.back());
    sl.set_l(std::get<2>(soft_bound));
    bound_with_id.soft_bound_left.emplace_back();
    target_ref_line_info_->ref_line().sl2xy(sl, &bound_with_id.soft_bound_left.back());
  }
  bounds_vec3d_with_id->emplace_back(bound_with_id);
}

/**
 * @brief 生成实时轨迹边界信息
 * @details 该函数用于根据路径数据和原始边界信息，生成实时轨迹的边界信息。通过插值计算，将原始边界信息与实时轨迹对齐，并更新边界类型信息，确保路径规划时能够正确考虑边界约束。
 * 
 * @param[in,out] path_boundary_info 路径边界信息，包含路径点的位置、左侧边界和右侧边界
 * @param[in] path_data 路径数据，包含路径的边界信息和规划结果
 * 
 * @par 关键变量说明:
 * - trajectory (DiscretizedPath): 实时轨迹，包含路径点的位置信息
 * - original_bound_info (std::vector<PathBoundary::PathBoundaryUnitInfo>): 原始边界信息，包含路径点的位置、左侧边界和右侧边界
 * - new_boundary (std::vector<PathBoundary::PathBoundaryUnitInfo>): 新的边界信息，包含插值后的路径点位置、左侧边界和右侧边界
 * - realtime_traj_boundary_type_info_ (std::vector<std::tuple<double, PathBoundary::BoundaryUnitTypeInfo, PathBoundary::BoundaryUnitTypeInfo>>): 实时轨迹边界类型信息，包含路径点的位置、左侧边界类型和右侧边界类型
 * 
 * @par 生成流程:
 * 1. 初始化新的边界信息集合
 * 2. 检查原始边界信息和实时轨迹是否为空，若为空则直接返回
 * 3. 遍历原始边界信息，更新边界类型信息
 * 4. 遍历实时轨迹，通过插值计算生成新的边界信息
 * 5. 更新路径边界信息
 * 
 * @par 流程图:
 * @startuml
 * start
 * :初始化新的边界信息集合;
 * if (原始边界信息或实时轨迹是否为空?) then (是)
 *   :直接返回;
 * endif
 * :遍历原始边界信息;
 * :更新边界类型信息;
 * :遍历实时轨迹;
 * :通过插值计算生成新的边界信息;
 * :更新路径边界信息;
 * end
 * @enduml
 * 
 * @note 该函数应在每次路径规划循环中调用，确保实时轨迹边界信息正确生成
 * 
 * @warning 需确保输入参数有效，特别是path_boundary_info和path_data
 */
void RealTimePathPlanner::generateRealtimeTrajBoundaryInfo(
    std::vector<PathBoundary::PathBoundaryUnitInfo>* path_boundary_info, PathData* path_data) {
  std::vector<PathBoundary::PathBoundaryUnitInfo> new_boundary;
  auto trajectory = path_data->discretized_path();
  new_boundary.reserve(trajectory.size());
  auto& original_bound_info = *path_boundary_info;
  if (original_bound_info.empty() || trajectory.empty()) return;

  double s_offset = adc_sl_info_.first[0];
  for (auto& unit : original_bound_info) {
    // path_boundary_info s 为参考线s ，修改其 s 从0 开始
    unit.s -= s_offset;
  }
  PathData::BlockFSInfo block_fs_info;
  size_t boundary_idx = 0;
  for (const auto& pt : trajectory) {
    const double s_traj = pt.s();
    const double l_traj = pt.l();

    // 定位边界插值区间
    while (boundary_idx + 1 < original_bound_info.size() && original_bound_info[boundary_idx + 1].s <= s_traj) {
      ++boundary_idx;
    }

    // 插值计算
    if (boundary_idx + 1 < original_bound_info.size()) {
      const auto& curr = original_bound_info[boundary_idx];
      const auto& next = original_bound_info[boundary_idx + 1];
      const double ratio = (s_traj - curr.s) / (next.s - curr.s);

      const double raw_left = curr.l_left + ratio * (next.l_left - curr.l_left);
      const double raw_right = curr.l_right + ratio * (next.l_right - curr.l_right);
      new_boundary.emplace_back(s_traj, raw_left - l_traj, raw_right - l_traj, curr.l_left_type, curr.l_right_type);
    } else {
      const auto& last = original_bound_info.back();
      new_boundary.emplace_back(s_traj, last.l_left - l_traj, last.l_right - l_traj, last.l_left_type,
                                last.l_right_type);
    }
    if((new_boundary.back().l_left - new_boundary.back().l_right < 1e-6) &&  !block_fs_info.is_valid){
      block_fs_info.is_valid = true;
      block_fs_info.s = pt.s();
      block_fs_info.path_type = PathData::PathType::LATERAL;
      block_fs_info.block_point = math::Vec2d(pt.x(), pt.y());
      block_fs_info.check_point = pt;
      block_fs_info.vis_pts.emplace_back(pt);
      path_data->mutableBlockFSInfo()->emplace_back(block_fs_info);
    }

  }

  *path_boundary_info = std::move(new_boundary);
}
}  // namespace gpal::pnc::planning
