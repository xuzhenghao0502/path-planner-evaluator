/**
 * @file speed_model_param.cpp
 * @brief 速度规划模型参数生成器
 * @details 本类负责速度规划问题中各类约束条件和优化目标的参数计算与维护
 */
#include "speed_optimizer/speed_model_param.h"

#include <filesystem>
#include <iostream>

#include "empirical_speed_planner/empirical_speed_planning.h"
#include "speed_preprocessor/speed_limit.h"

namespace gpal::pnc::planning {
/**
 * @brief 初始化速度模型参数
 * @details 初始化阶段数、默认参数向量，加载配置管理器实例及车辆/规划配置参数。
 *
 * 主要完成以下初始化工作：
 * 1. 设置阶段数(stage_num_)为时间网格长度
 * 2. 初始化参数向量为默认参数副本
 * 3. 设置初始跟车时距系数K_0_和速度差斜率delta_v_slope_for_K_
 * 4. 从配置管理器加载车辆参数和优化器配置
 *
 * @par 输入参数:
 * - time_grid_ 时间网格数组（需≥2个元素）
 *
 * @par 输出成员变量:
 * - stage_num_ 阶段总数（等于时间网格长度）
 * - parameters_ 参数向量（每个阶段一个参数映射）
 * - K_0_ 初始跟车时距系数（默认1.5秒）
 * - delta_v_slope_for_K_ 速度差斜率（默认0.3）
 * - vehicle_param_ 车辆参数（来自配置）
 * - speed_ocp_qp_optimizer_config_ 优化器配置（来自配置）
 *
 * @par 关键流程:
 * @startuml
 * start
 * :设置stage_num_ = time_grid_.size();
 * :初始化parameters_为stage_num_个默认参数副本;
 * :设置K_0_=1.5, delta_v_slope_for_K_=0.3;
 * :获取ConfigManager单例实例;
 * :加载vehicle_param_车辆参数;
 * :加载speed_ocp_qp_optimizer_config_优化器配置;
 * :加载speed_planner_config_速度规划配置;
 * stop
 * @enduml
 */
void SpeedModelParam::init() {
  stage_num_ = time_grid_.size();  // must be >=2
  parameters_ = std::vector<std::unordered_map<std::string, double>>(stage_num_, default_params_);
  K_0_ = 1.5;                  // TODO:param
  delta_v_slope_for_K_ = 0.3;  // TODO:param

  config_manager_ = Singleton<ConfigManager>::get_instance();
  CHECK_NOTNULL(config_manager_);
  vehicle_param_ = config_manager_->vehicle_config().vehicle_param();
  speed_ocp_qp_optimizer_config_ = config_manager_->getConfig<SpeedOcpQpOptimizerConfig>("SpeedOcpQpOptimizerConfig");
  speed_planner_config_ = config_manager_->getConfig<SpeedPlannerConfig>("SpeedPlannerConfig");
}
/**
 * @brief 设置参数到参数表
 * @param[in] name 参数名称
 * @param[in] t_index 时间索引
 * @param[in] value 参数值
 * @note 自动校验时间索引有效性
 */
void SpeedModelParam::setParameter(const std::string& param_name, size_t i, double param_val) {
  if (parameters_[0].find(param_name) == parameters_[0].end()) {
    // PERROR << "no [" << param_name << "] in parameters! ";
    return;
  }
  parameters_[i][param_name] = param_val;
}
/**
 * @brief 从参数表获取参数
 * @param[in] name 参数名称
 * @param[in] t_index 时间索引
 * @return double 参数值
 * @throw 索引越界时抛出std::out_of_range
 */
double SpeedModelParam::getParameter(const std::string& param_name, size_t i) { return parameters_[i][param_name]; }
/**
 * @brief 计算速度模型优化参数（主入口函数）
 * @param[in] local_view 局部环境视图（包含车辆状态、传感器数据）
 * @param[in] decision_result 决策结果（包含行为状态、参考轨迹）
 * @param[in] obstacle_map 障碍物集合（键：障碍物ID，值：障碍物对象指针）
 * @param[in] path_group 路径组（原始路径+扩展路径）
 * @param[in] speed_init_state 初始速度状态（s,v,a,j）
 * @param[in] speed_result 速度规划结果（输出容器）
 * @return vector<unordered_map<string, double>> 各阶段优化参数集合
 *
 * @details 实现速度规划模型参数计算的完整流程：
 * 1. 初始化阶段：
 *    - 初始化时间网格和默认参数
 *    - 加载车辆参数和规划配置
 * 2. 障碍物分析：
 *    - 识别最近入侵障碍物及其时空信息
 *    - 计算相对速度差(deltaV)和距离差(deltaS)
 * 3. 约束计算：
 *    - 生成s/v/a的硬/软约束边界
 *    - 计算防撞加速度和经验加速度范围
 * 4. 权重设置：
 *    - 调整S/V/A/J各维度的优化权重
 * 5. 特殊场景处理：
 *    - 处理障碍物停车、变道、交错驾驶等场景
 *
 * @par 输入参数说明:
 * - local_view: 必须包含有效的VehiclePose、Chassis、Localization数据
 * - obstacle_map: 障碍物需包含ST边界和风险场信息
 * - path_group: 至少包含原始路径origin_path_
 *
 * @par 输出参数说明:
 * - 返回参数向量结构：vector<unordered_map<参数名, 参数值>>
 *   每个元素对应一个时间阶段的参数集合，包含：
 *   - SHardUpperBound/SHardLowerBound: s硬约束
 *   - VRef/VWeight: 速度参考值及权重
 *   - K/k: 跟车时距参数
 *
 * @startuml
 * start
 * :调用init()初始化参数;
 * :加载初始状态x_0_;
 * :分析障碍物investigateNearestObj();
 * :更新成员变量updateMemberVariables();
 * if (启用风险模型) then (是)
 *   :计算风险场限速calcRiskFieldSpeedLimit();
 * endif
 * :设置约束参数setContraintParam();
 * :设置权重参数setWeightParam();
 * :处理特殊场景（变道/停车/交错驾驶）;
 * :返回优化参数parameters_;
 * stop
 * @enduml
 */
std::vector<std::unordered_map<std::string, double>> SpeedModelParam::calculateSpeedModelParam(
    const LocalView& local_view, const DecisionResult& decision_result, const ObstacleSet& obstacle_map,
    const PathGroup& path_group, const SpeedState& speed_init_state, std::shared_ptr<SpeedResult> speed_result) {
  init();
  x_0_ = speed_init_state;
  init_s_ = x_0_.s;
  init_v_ = x_0_.v;
  init_a_ = x_0_.a;
  obstacle_map_ = obstacle_map;
  behavior_state_ = speed_result->getBehaviorState();
  // preprocess
  investigateNearestObj(obstacle_map, speed_result->StGraphData(), speed_result->speed_walls(),
                        speed_result->mutableNearestInvasionObstacle());

  // output:
  // nearest_front_obj_id_;nearest_front_obj_invasion_s_;nearest_front_obj_invasion_v_;nearest_front_obj_obstacle_
  // output: deltaV_,deltaS_,initHWT_,init_K_,dv_risk_,front_obj_v_

  updateMemberVariables(speed_result->StGraphData(), local_view, path_group.origin_path_,
                        speed_result->speedLimitResult(), speed_result->speed_walls());

  if (speed_ocp_qp_optimizer_config_.enable_speed_risk_model()) {
    calcRiskFieldSpeedLimit(local_view, obstacle_map, path_group, speed_result);
  }

  // set param
  setContraintParam(local_view, speed_result->StGraphData(), speed_result->speedLimitResult().max_speed_limit_);

  setWeightParam();

  // special case deal
  updateDecisionCoarseTraj(decision_result);

  specialCaseForOvertakeObstacle(decision_result, obstacle_map, speed_result->StGraphData());

  specialCaseForObstacleStop();

  specialCaseForLaneChange(decision_result, obstacle_map, speed_result->StGraphData());


  if(speed_ocp_qp_optimizer_config_.enable_merge_special_case()){
    specialCaseForMergeObstacle(path_group, obstacle_map);
  }

  // if (speed_ocp_qp_optimizer_config_.enable_stagger_driving()) {
  //   setBoundaryForStaggerDriving(path_group, obstacle_map);
  // }

  if (speed_planner_config_.enable_local_path_od_check()) {
    localPathRiskyConsideration(local_view, speed_result);
  }

  weightsProtection();

  // size_t speed_data_size_ = std::round(time_horizon_ / drivable_boundary_dt_) + 1UL;
  // EmpiricalSpeedPlan empirical_speed_plan(init_s_, init_v_, init_a_,
  // speed_result->speedLimitResult().max_speed_limit_, drivable_boundary_dt_,
  //                                             speed_data_size_, delta_v_slope_for_K_, K_0_, 5.0,
  //                                             speed_planner_config_);
  // empirical_speed_plan.calcEmpiricalSpeedPlanningResult(local_view, path_group.origin_path_, speed_result,
  // complete_s_hard_upper_bounds_[0UL].second,
  //                                                           nearest_front_obj_invasion_v_);

  // for (size_t t_index = 0; t_index < time_grid_.size(); ++t_index) {
  //   ERT_PLOG_I << " t = " << time_grid_[t_index] << "   s_hard(" << getParameter("SHardLowerBound", t_index) << ","
  //             << getParameter("SHardUpperBound", t_index) << ");  " << "s_soft("
  //             << getParameter("SSoftLowerBound", t_index) << "," << getParameter("SSoftUpperBound", t_index) << "); "
  //             << "v_hard(" << getParameter("VHardLowerBound", t_index) << ","
  //             << getParameter("VHardUpperBound", t_index) << ");  " << "v_soft(" << " " << ","
  //             << getParameter("VSoftUpperBound", t_index) << ");  " << "a_hard("
  //             << getParameter("AHardLowerBound", t_index) << "," << getParameter("AHardUpperBound", t_index) << "); "
  //             << "a_soft(" << getParameter("ASoftLowerBound", t_index) << ","
  //             << getParameter("ASoftUpperBound", t_index) << ");  " << "j_hard("
  //             << getParameter("JHardLowerBound", t_index) << "," << getParameter("JHardUpperBound", t_index) << "); "
  //             << "s_v_ref(" << getParameter("SRef", t_index) << "," << getParameter("VRef", t_index) << ");  " <<
  //             "K_k("
  //             << getParameter("K", t_index) << "," << getParameter("k", t_index) << ");" ;
  // }
  // for (size_t t_index = 0; t_index < time_grid_.size(); ++t_index) {
  //   ERT_PLOG_I << " t = " << time_grid_[t_index] << "   SWeight(" << getParameter("SWeight", t_index) << ");  "
  //             << "VWeight(" << getParameter("VWeight", t_index) << ");  " << "AWeight("
  //             << getParameter("AWeight", t_index) << ");  " << "JWeight(" << getParameter("JWeight", t_index) << "); "
  //             << "SlackSUpperWeight(" << getParameter("SlackSUpperWeight", t_index) << ");  " << "SlackSLowerWeight("
  //             << getParameter("SlackSLowerWeight", t_index) << ");  " << "SlackVUpperWeight("
  //             << getParameter("SlackVUpperWeight", t_index) << ");  " << "SlackAUpperWeight("
  //             << getParameter("SlackAUpperWeight", t_index) << ");  " << "SlackALowerWeight("
  //             << getParameter("SlackALowerWeight", t_index) << ");  " << "SlackDVWeight("
  //             << getParameter("SlackDVWeight", t_index) << ");  " << "SafeDistForDVConstraint("
  //             << getParameter("SafeDistForDVConstraint", t_index) << ");  " << "SUpperBoundForDVConstraint("
  //             << getParameter("SUpperBoundForDVConstraint", t_index) << ");  " ;
  // }

  return parameters_;
}
/**
 * @brief 更新速度模型的核心成员变量（约束、加速度、K参数）
 * @param[in] st_graph ST图（包含可行驶边界和障碍物边界）
 * @param[in] local_view 局部环境视图（包含车辆状态、传感器数据）
 * @param[in] path 离散化路径（用于曲率限速计算）
 * @param[in] speed_limit_result 限速结果（包含路径速度限制）
 * @param[in] speed_walls 速度墙集合（如红绿灯停止墙）
 *
 * @details 完成以下核心计算：
 * 1. 空间约束计算：
 *    - 计算s维度的硬上界(complete_s_hard_upper_bounds_)
 *    - 计算s维度的软下界(complete_s_soft_lower_bounds_)
 *    - 计算s维度的软上界(complete_s_soft_upper_bounds_)
 * 2. 速度约束计算：
 *    - 计算v维度的软上界(complete_v_soft_upper_bounds_)
 * 3. 加速度计算：
 *    - 计算防撞加速度(antiCollisionAccel_)
 *    - 计算经验加速度范围(empirical_acc_min_/empirical_acc_max_)
 * 4. 跟车参数计算：
 *    - 动态调整跟车时距系数K
 *
 * @par 关键成员变量更新:
 * - complete_s_hard_upper_bounds_ 时空硬上界约束集合
 * - complete_s_soft_lower_bounds_ 时空软下界约束集合
 * - complete_v_soft_upper_bounds_ 速度软上界约束集合
 * - antiCollisionAccel_ 防撞最大减速度
 * - empirical_acc_min_/empirical_acc_max_ 经验加速度范围
 * - K_ 动态调整后的跟车时距系数
 *
 * @startuml
 * start
 * :调用calcCompleteSHardUpperBounds(st_graph);
 * :调用calcCompleteSSoftLowerBounds(st_graph);
 * :调用calcCompleteSSoftUpperBounds(st_graph, speed_walls);
 * :调用calcCompleteVSoftBounds(local_view, st_graph, path, speed_limit_result.path_v_t_);
 * :计算antiCollisionAccel_ = calcAntiCollisionAccel(...);
 * :调用calcEmpiricalAccelRange(...);
 * :调用calcK(local_view);
 * stop
 * @enduml
 */
void SpeedModelParam::updateMemberVariables(const StGraph& st_graph, const LocalView& local_view,
                                            const DiscretizedPath& path, const SpeedLimitResult& speed_limit_result,
                                            const std::vector<SpeedWall>& speed_walls) {
  // Calculate the init state of this planning frame
  //  TODO :move to task calcInitState(frame);
  // s
  calcCompleteSHardUpperBounds(st_graph);
  calcCompleteSSoftLowerBounds(st_graph);
  calcCompleteSSoftUpperBounds(st_graph, speed_walls);

  // v
  calcCompleteVSoftBounds(local_view, st_graph, path, speed_limit_result.path_v_t_);

  // a
  antiCollisionAccel_ = calcAntiCollisionAccel(complete_s_hard_upper_bounds_);
  calcEmpiricalAccelRange(empirical_acc_min_, empirical_acc_max_);

  // K
  calcK(local_view);
}
/**
 * @brief 查找最近的入侵障碍物（主分析函数）
 * @param[in] obstacle_map 障碍物集合（键：障碍物ID，值：障碍物对象指针）
 * @param[in] st_graph ST图数据（包含时空边界信息）
 * @param[in] speed_walls 速度墙集合（如红绿灯停止墙）
 * @param[out] nearest_invasion_obstacle 输出最近入侵障碍物信息
 *
 * @details 核心处理流程：
 * 1. 初始化最近障碍物相关参数
 * 2. 遍历ST图边界：
 *    - 筛选FOLLOW/YIELD类型的障碍物边界
 *    - 计算入侵时间点和位置
 *    - 更新最近入侵障碍物信息
 * 3. 处理速度墙：
 *    - 选择s值最小的速度墙作为约束
 * 4. 输出处理结果：
 *    - 填充入侵障碍物信息结构体
 *    - 计算跟车相关参数（deltaV/deltaS等）
 *
 * @startuml
 * start
 * :初始化最近障碍物参数;
 * :遍历ST图边界;
 *   if (边界类型是FOLLOW/YIELD) then (是)
 *     :获取障碍物ID和对象;
 *     :计算入侵点时空信息;
 *     if (是更近的障碍物) then (是)
 *       :更新最近障碍物信息;
 *     endif
 *   endif
 * :遍历速度墙;
 *   if (速度墙s更小) then (是)
 *     :更新为速度墙约束;
 *   endif
 * :填充输出结构体;
 * :计算deltaV/deltaS等跟车参数;
 * stop
 * @enduml
 */
void SpeedModelParam::investigateNearestObj(const ObstacleSet& obstacle_map, const StGraph& st_graph,
                                            const std::vector<SpeedWall> speed_walls,
                                            InvasionObstacle* nearest_invasion_obstacle) {
  STPoint invasion_pt;
  std::string invasion_obj_id;
  double t_window_length = 3.0;
  double invasion_t;
  double invasion_s;
  double invasion_v;
  has_front_yield_obj_ = false;
  nearest_front_obj_id_ = "None";
  nearest_front_obj_invasion_s_ = std::numeric_limits<double>::infinity();
  nearest_front_obj_obstacle_ = false;

  for (auto st_boundary : st_graph.st_boundaries()) {
    if ((st_boundary->boundary_type() == STBoundary::BoundaryType::FOLLOW) ||
        (st_boundary->boundary_type() == STBoundary::BoundaryType::YIELD)) {
      ERT_PLOG_D << "st_boundary->boundary_type() = " << (int)st_boundary->boundary_type();
      invasion_obj_id = st_boundary->id();

      if (obstacle_map.find(invasion_obj_id) == obstacle_map.end()) {
        continue;
      }
      auto obs = obstacle_map.at(invasion_obj_id);

      bool speed_reverse_direction = false;
      double s_0 = st_boundary->bottom_left_point().s();
      double s_1 = st_boundary->bottom_right_point().s();
      speed_reverse_direction = ((s_1 - s_0 <= -0.1));

      invasion_pt = st_boundary->bottom_left_point();
      // Just for debug:
      // if (!speed_reverse_direction) {
      //   const auto& pt_0 = st_boundary->bottom_left_point();
      //   const auto& pt_1 = st_boundary->lowest_point();
      // SFIELD_DEBUG(qp_optimizer, "[LOWESTPOINT]t0 = {}, s0 = ", pt_0.t(), pt_0.s());
      // SFIELD_DEBUG(qp_optimizer, "[LOWESTPOINT]t1 = {}, s1 = ", pt_1.t(), pt_1.s());
      // }
      // Debug ends

      invasion_t = invasion_pt.t();
      if ((invasion_t <= -kMathEpsilon) && (st_boundary->max_t() >= kMathEpsilon)) {
        invasion_t = 0.0F;
        double upper_s;
        st_boundary->getBoundarySRange(invasion_t, &upper_s, &invasion_s);
      } else if (invasion_t >= 0.0F) {
        // BugFix:
        invasion_s = invasion_pt.s();
      } else {
        continue;
      }

      if (invasion_s < nearest_front_obj_invasion_s_) {
        nearest_front_obj_id_ = invasion_obj_id;
        nearest_front_obj_invasion_s_ = invasion_s;
        nearest_front_obj_invasion_t_ = invasion_t;
        nearest_front_obj_invasion_v_ = st_boundary->calcSTLowerBoundProjectedSpeed(invasion_t, t_window_length);
        if (!obs->getRiskFieldInfos().empty() &&
            abs(obs->getRiskFieldInfos().front().longitudinal_speed - nearest_front_obj_invasion_v_) > 1.0) {
          nearest_front_obj_invasion_v_ = obs->getRiskFieldInfos().front().longitudinal_speed;
        }
        if (speed_reverse_direction && nearest_front_obj_invasion_v_ < 0.0) {
          nearest_front_obj_invasion_v_ = 0.0;
        }
        nearest_front_obj_obstacle_ = true;
      }
    }
  }

  // TODO:add speed_wall
  for (auto speed_wall : speed_walls) {
    if (speed_wall.st_wall[0].y() > nearest_front_obj_invasion_s_) {
      continue;
    }
    nearest_front_obj_id_ = getSpeedWallIdThroughType(speed_wall.type);
    nearest_front_obj_invasion_s_ = speed_wall.st_wall[0].y();
    nearest_front_obj_invasion_t_ = speed_wall.st_wall[0].x();
    nearest_front_obj_invasion_v_ = 0.0;
    nearest_front_obj_obstacle_ = false;
    // nearest_speed_wall_stop_distance_ = speed_wall.stop_distance;
  }

  {
    nearest_invasion_obstacle->obj_id_ = nearest_front_obj_id_;
    nearest_invasion_obstacle->invasion_s_ = nearest_front_obj_invasion_s_;
    nearest_invasion_obstacle->invasion_t_ = nearest_front_obj_invasion_t_;
    nearest_invasion_obstacle->invasion_v_ = nearest_front_obj_invasion_v_;
    nearest_invasion_obstacle->is_obstacle_ = nearest_front_obj_obstacle_;
  }

  has_front_yield_obj_ = (nearest_front_obj_invasion_s_ < std::numeric_limits<double>::infinity());

  // TODO
  double safe_distance = 5.0;

  deltaV_ = init_v_ - nearest_front_obj_invasion_v_;

  deltaS_ = nearest_front_obj_invasion_s_ - nearest_front_obj_invasion_v_ * nearest_front_obj_invasion_t_ -
            (K_0_ + delta_v_slope_for_K_ * fmax(0.0, deltaV_)) * init_v_ - safe_distance;

  initHWT_ =
      (nearest_front_obj_invasion_s_ - nearest_front_obj_invasion_t_ * fmax(nearest_front_obj_invasion_v_, 0.0)) /
      fmax(0.1, init_v_);

  init_K_ = (nearest_front_obj_invasion_s_ - safe_distance) / fmax(0.1, init_v_);

  dv_risk_ = deltaV_ * init_v_ / fmax(nearest_front_obj_invasion_s_, 0.1);
  ERT_PLOG_D << "[INVASIONOBJ] : nearest_front_obj_id_ = " << nearest_front_obj_id_ << " nearest_front_obj_invasion_s_ "
       << nearest_front_obj_invasion_s_ << "    nearest_front_obj_invasion_v_ = " << nearest_front_obj_invasion_v_
       << "  deltaV_ =  " << deltaV_ << "  deltaS_ = " << deltaS_ << "  initHWT_ = " << initHWT_
       << "  init_K_ = " << init_K_ << "  dv_risk_ = " << dv_risk_ ;

  // SFIELD_DEBUG(qp_optimizer, "[INVASIONOBJ]Nearest front obj id = {}", nearest_front_obj_id_);
  // SFIELD_DEBUG(qp_optimizer, "[INVASIONOBJ]nearest_front_obj_invasion_v_ = ", nearest_front_obj_invasion_v_);
}
// Calculating the max acceleration range for speed limit
/**
 * @brief 计算基于限速值的经验最大加速度
 * @param[in] speedLimit 当前限速值（单位：m/s）
 * @return double 经验最大加速度（单位：m/s²）
 *
 * @details 通过速度差计算加速度约束：
 * - 公式：accel = -0.15 * (当前速度 - 限速值)
 * - 当自车速度超过限速时产生负加速度（减速）
 * - 当自车速度低于限速时产生正加速度（加速）
 */
double SpeedModelParam::calcEmpiricalMaxAccelForSpeedLimit(double speedLimit) const {
  double k_0 = 0.15;
  double deltaV = init_v_ - speedLimit;
  return -k_0 * deltaV;
}
/**
 * @brief 计算经验加速度范围
 * @param[out] min_a 最小加速度（单位：m/s²）
 * @param[out] max_a 最大加速度（单位：m/s²）
 * @details 分两种场景计算：
 *          1. 无前方障碍物：基于地图限速计算
 *          2. 有前方障碍物：调用calcEmpiricalAccelRangeForObs计算
 * @note 最终结果取两种场景的交集
 *
 * @startuml
 start
 if (存在前方让行对象?) then (是)
   :调用calcEmpiricalAccelRangeForObs;
 else (否)
   :基于地图限速计算;
 endif
 :取两种场景的交集;
 @enduml
 */
void SpeedModelParam::calcEmpiricalAccelRange(double& min_a, double& max_a) {
  double speed_limit = speed_planner_config_.max_speed_limit() * KMH_MS;
  double empirical_acc_open_road = calcEmpiricalMaxAccelForSpeedLimit(speed_limit);
  if (!has_front_yield_obj_) {
    min_a = empirical_acc_open_road;
    max_a = empirical_acc_open_road;
  }
  if (has_front_yield_obj_) {
    SpeedPoint invasion_pt;
    invasion_pt.set_s(nearest_front_obj_invasion_s_);
    invasion_pt.set_v(fmax(nearest_front_obj_invasion_v_, 0.0));
    invasion_pt.set_a(0.0);
    invasion_pt.set_t(nearest_front_obj_invasion_t_);
    calcEmpiricalAccelRangeForObs(invasion_pt, min_a, max_a);
    max_a = fmin(max_a, empirical_acc_open_road);
    min_a = fmin(min_a, empirical_acc_open_road);
  }
}

/**
 * @brief 计算障碍物相关经验加速度范围
 * @param[in] obj_invasion_point 障碍物入侵点信息
 * @param[out] min_a 最小加速度（单位：m/s²）
 * @param[out] max_a 最大加速度（单位：m/s²）
 * @details 基于相对运动学公式和查表机制计算：
 *          - deltaS: 相对距离 = 障碍物s - 自车s - 安全余量
 *          - deltaV: 相对速度 = 自车速度 - 障碍物速度
 *          - 应用经验公式：A = (k*deltaS - deltaV) / (K_0 + delta_v_slope*deltaV)
 *
 * @startuml
 start
 :计算deltaS和deltaV;
 :应用经验公式计算A1/A2;
 :min_a = max(2.0, 取较小值);
 :max_a = min(-6.0, 取较大值);
 @enduml
 */
void SpeedModelParam::calcEmpiricalAccelRangeForObs(const SpeedPoint& obj_invasion_point, double& min_a,
                                                    double& max_a) {
  double deltaS = obj_invasion_point.s() - obj_invasion_point.v() * obj_invasion_point.t() -
                  (K_0_ + delta_v_slope_for_K_ * fmax(0.0, init_v_ - obj_invasion_point.v())) * init_v_ - 4.0;
  double deltaV = init_v_ - obj_invasion_point.v();
  double k1 = 0.3;
  double A1 = (deltaV >= 0.0) ? (k1 * deltaS - deltaV + delta_v_slope_for_K_ * obj_invasion_point.a() * init_v_) /
                                    (K_0_ + delta_v_slope_for_K_ * (deltaV + init_v_))
                              : (k1 * deltaS - deltaV) / K_0_;
  double k2 = 0.3;
  double A2 = (deltaV >= 0.0) ? (k2 * deltaS - deltaV + delta_v_slope_for_K_ * obj_invasion_point.a() * init_v_) /
                                    (K_0_ + delta_v_slope_for_K_ * (deltaV + init_v_))
                              : (k2 * deltaS - deltaV) / K_0_;
  min_a = fmax(2.0, fmin(A1, A2));   // TODO: param
  max_a = fmin(-6.0, fmax(A1, A2));  // TODO: param
  // Only for debug
  antiCollisionAccel_ =
      fmin(-0.5 * deltaV * deltaV / (obj_invasion_point.s() - obj_invasion_point.v() * obj_invasion_point.t()),
           antiCollisionAccel_);
}
/**
 * @brief 计算完整s维度硬上界约束（时空联合优化关键步骤）
 * @param[in] st_graph ST图对象（包含可行驶边界数据）
 * 
 * @par 处理流程:
 * @startuml
start
partition 数据初始化 {
  :获取可行驶边界数量complete_size;
  if (数组大小不匹配) then (是)
    :调整complete_s_hard_upper_bounds_大小;
  endif
  :从ST图读取原始s硬上界值;
}
partition 单调递增处理 {
  :反向遍历约束点序列;
  :维护right_margin_min确保右边界不小于当前值;
  :更新当前时间点的s硬上界约束值;
}
@enduml
 *
 * @par 关键参数说明:
 * | 参数                        | 类型           | 说明                     |
 * |-----------------------------|----------------|--------------------------|
 * | st_drivable_boundaries      | vector<StDrivableBoundary> | ST图可行驶边界数据源 |
 * | complete_s_hard_upper_bounds_ | vector<pair<double,double>> | 输出结果容器     |
 *
 * @note 功能特性:
 * - 反向遍历维护最小值机制：确保后续时间点的约束不低于当前值
 * - 硬约束来源：直接使用ST图提供的s_upper_bound字段
 * - 结果应用：用于QP优化问题的s_hard_upper_bound参数生成
 *
 * @warning 使用约束:
 * - 需确保st_graph包含有效可行驶边界数据
 * - 时间网格需与st_drivable_boundaries时间序列严格对齐
 * - 反向遍历时需处理数组越界防护(fmin函数)
 */
void SpeedModelParam::calcCompleteSHardUpperBounds(const StGraph& st_graph) {
  size_t complete_size = st_graph.st_drivable_boundaries().size();
  if (complete_s_hard_upper_bounds_.size() != complete_size) {
    complete_s_hard_upper_bounds_.resize(complete_size);
  }
  for (size_t i = 0UL; i < complete_size; i++) {
    complete_s_hard_upper_bounds_[i].first = st_graph.st_drivable_boundaries()[i].t;
    complete_s_hard_upper_bounds_[i].second = st_graph.st_drivable_boundaries()[i].s_upper_bound;
    // TODO: drivable boundaries里同时包含硬、软约束，避免硬约束直接到300的问题，暂时依然用反向障碍物后续s作为硬约束
  }
  // TODO: [DELET] speed_wall
  // Converting it to a monotonely-increasing upper-bounds:
  double right_margin_min = std::numeric_limits<double>::infinity();
  for (int i = static_cast<int>(complete_size - 1UL); i >= 0; i--) {
    right_margin_min = fmin(right_margin_min, complete_s_hard_upper_bounds_[i].second);
    complete_s_hard_upper_bounds_[i].second = right_margin_min;
  }
}
/**
 * @brief 计算s维度的软下界约束（核心实现）
 *
 * @param[in] st_graph ST图对象（包含可行驶边界数据）
 *
 * @details 实现步骤：
 * 1. 获取可行驶边界数量并调整存储数组大小
 * 2. 从ST图读取每个时间点的s软下界值
 * 3. 正向遍历确保软下界单调递增（取右侧最大值）
 *
 * @par 关键处理逻辑:
 * - 输入：ST图的drivable_boundaries数组（时间-空间对集合）
 * - 输出：成员变量complete_s_soft_lower_bounds_（单调递增的时间-空间约束）
 * - 正向遍历时维护right_margin_max，确保当前s下界不低于后续时间点的最大值
 *
 * @startuml
 * start
 * :获取可行驶边界数量complete_size;
 * if (数组大小不匹配) then (是)
 *   :调整complete_s_soft_lower_bounds_大小;
 * endif
 * :遍历每个可行驶边界i;
 *   :存储t和s_lower_bound到数组;
 * :初始化right_margin_max为0.0;
 * :正向遍历i从0到complete_size-1;
 *   :right_margin_max = max(当前s下界, right_margin_max);
 *   :更新当前s下界为right_margin_max;
 * stop
 * @enduml
 */
void SpeedModelParam::calcCompleteSSoftLowerBounds(const StGraph& st_graph) {
  size_t complete_size = st_graph.st_drivable_boundaries().size();
  if (complete_s_soft_lower_bounds_.size() != complete_size) {
    complete_s_soft_lower_bounds_.resize(complete_size);
  }

  for (size_t i = 0UL; i < complete_size; i++) {
    complete_s_soft_lower_bounds_[i].first = st_graph.st_drivable_boundaries()[i].t;
    complete_s_soft_lower_bounds_[i].second = st_graph.st_drivable_boundaries()[i].s_lower_bound;
  }
  // Converting it to a monotonely-increasing lower-bounds:
  double right_margin_max = 0.0;
  for (int i = 0; i < complete_size; i++) {
    right_margin_max = fmax(right_margin_max, complete_s_soft_lower_bounds_[i].second);
    complete_s_soft_lower_bounds_[i].second = right_margin_max;
  }
}
/**
 * @brief 计算s维度的软上界约束（核心实现）
 *
 * @param[in] st_graph ST图对象（包含可行驶边界和障碍物信息）
 * @param[in] speed_walls 速度墙集合（如红绿灯停止墙）
 *
 * @details 实现步骤：
 * 1. 初始化软上界约束数组大小
 * 2. 处理障碍物软上界约束（YIELD/FOLLOW类型）
 * 3. 处理速度墙约束（生成安全停止距离）
 * 4. 综合障碍物和速度墙约束生成最终软上界
 *
 * @par 关键处理逻辑:
 * - 输入：ST图的drivable_boundaries数组 + 速度墙数据
 * - 输出：成员变量complete_s_soft_upper_bounds_
 * - 约束优先级：速度墙安全距离 > 障碍物软上界 > 硬上界约束
 *
 * @startuml
 * start
 * :获取可行驶边界数量complete_size;
 * if (数组大小不匹配) then (是)
 *   :调整complete_s_soft_upper_bounds_大小;
 * endif
 * :遍历障碍物ST边界;
 *   if (是让行/跟车障碍物) then (是)
 *     :生成障碍物软上界曲线;
 *   endif
 * :遍历速度墙;
 *   :计算安全停止距离;
 *   :更新速度墙软上界;
 * :遍历每个时间点;
 *   :综合障碍物和速度墙约束;
 *   :确保软上界不超过硬上界;
 * stop
 * @enduml
 */
void SpeedModelParam::calcCompleteSSoftUpperBounds(const StGraph& st_graph, const std::vector<SpeedWall>& speed_walls) {
  size_t complete_size = st_graph.st_drivable_boundaries().size();
  if (complete_s_soft_upper_bounds_.size() != complete_size) {
    complete_s_soft_upper_bounds_.resize(complete_size);
  }
  // obstacle
  unordered_map<std::string, std::vector<STPoint>> st_soft_upper_boundaries;
  const auto& st_boundaries = st_graph.st_boundaries();
  for (size_t i = 0UL; i < st_boundaries.size(); i++) {
    const auto* st_boundary_ptr = st_boundaries[i];
    auto boundary_type = st_boundary_ptr->boundary_type();
    bool is_yield_obj =
        (boundary_type == STBoundary::BoundaryType::FOLLOW) || (boundary_type == STBoundary::BoundaryType::YIELD);
    if (is_yield_obj) {
      std::vector<STPoint> st_soft_upper_boundary;
      CalcYieldObstacleSoftUpperBounds(st_boundary_ptr, st_soft_upper_boundary);
      st_soft_upper_boundaries[st_boundary_ptr->id()] = std::move(st_soft_upper_boundary);
    }
  }
  // TODO: speed wall
  double speed_wall_s_soft_upper = 300.0;
  for (const auto& speed_wall : speed_walls) {
    double safe_distance = speed_wall.stop_distance;
    double soft_upper_s = fmax(kMathEpsilon, speed_wall.st_wall[0].y() - safe_distance);
    speed_wall_s_soft_upper = fmin(speed_wall_s_soft_upper, soft_upper_s);
  }
  for (size_t i = 0UL; i < complete_size; i++) {
    double s_soft_upper_bound = std::numeric_limits<double>::infinity();
    for (const auto& boundary : st_graph.st_boundaries()) {
      auto& drivabe_boundary = st_graph.st_drivable_boundaries()[i];
      complete_s_soft_upper_bounds_[i].first = drivabe_boundary.t;
      if (st_soft_upper_boundaries.find(boundary->id()) != st_soft_upper_boundaries.end()) {
        auto& obs_soft_upper_bounds = st_soft_upper_boundaries[boundary->id()];
        if (!obs_soft_upper_bounds.empty()) {
          if (drivabe_boundary.t > obs_soft_upper_bounds.back().t() + kEpsilon) {
            continue;
          }
          size_t idx = (drivabe_boundary.t < obs_soft_upper_bounds[0UL].t())
                           ? 0UL
                           : static_cast<size_t>(
                                 round((drivabe_boundary.t - obs_soft_upper_bounds[0UL].t()) / drivable_boundary_dt_));
          idx = fmin(idx, obs_soft_upper_bounds.size() - 1UL);
          s_soft_upper_bound = std::fmin(obs_soft_upper_bounds[idx].s(), s_soft_upper_bound);
        }
      }
      s_soft_upper_bound = fmin(s_soft_upper_bound, complete_s_hard_upper_bounds_[i].second - 0.1);
    }
    complete_s_soft_upper_bounds_[i].second = fmin(s_soft_upper_bound, speed_wall_s_soft_upper);
  }
}
/**
 * @brief 计算让行障碍物的s维度软上界约束（核心实现）
 *
 * @param[in] st_boundary_ptr 障碍物的ST边界对象（包含时间-空间边界点、横向距离等信息）
 * @param[out] st_soft_upper_boundary 输出的软上界点集合（时间t，空间s）
 *
 * @details 实现步骤：
 * 1. 获取障碍物ST边界的最小s值及对应时间点
 * 2. 计算横向距离影响因子，动态调整安全距离
 * 3. 遍历时间点生成初始软上界约束
 * 4. 插值生成连续约束点集
 *
 * @par 核心处理逻辑:
 * - 安全距离 = max(横向影响因子*最大安全距离, 最小安全距离)
 * - 软上界 = 障碍物s边界 - 动态安全距离
 * - 通过插值保证软上界点的时间连续性
 *
 * @startuml
 * start
 * :获取障碍物ST边界的最小s值min_s;
 * :查找最小s对应的时间点t_for_min_s;
 * :计算横向距离影响因子lateral_dist_factor;
 * :计算动态安全距离safe_dist_for_min_s;
 * :初始化软上界s_soft_upper_bound;
 * :遍历s_upper_bounds时间点;
 *   if (时间点 <= t_for_min_s) then (是)
 *     :使用最小s对应安全距离;
 *   else (否)
 *     :动态调整安全距离;
 *   endif
 *   :生成软上界点;
 * :插值生成连续约束点集;
 * stop
 * @enduml
 */
void SpeedModelParam::CalcYieldObstacleSoftUpperBounds(const STBoundary* st_boundary_ptr,
                                                       std::vector<STPoint>& st_soft_upper_boundary) {
  std::vector<STPoint> s_upper_bounds =
      st_boundary_ptr->lower_points();  // here s_upper_bounds means the upper bounds caused by the object
  double min_s = st_boundary_ptr->min_s();

  double obs_safe_distance = 5.0;  // TODO 基于不同障碍物调整
  double max_safe_dist = obs_safe_distance;
  double min_safe_dist = 3.0;  // TODO 基于车速确认std::min(3.0, K_0_ * init_v_);
  double safe_dist_increasing_slope = 1.5;

  // First, calc the cruise point (t1, s1), with lowest s
  double t_for_min_s = -std::numeric_limits<double>::infinity();
  size_t min_s_idx = 0;

  // Calculate t for min_s, namely the time point at which SUpperBound is min_s
  for (size_t i = 0UL; i < s_upper_bounds.size(); i++) {
    if (s_upper_bounds[i].s() == min_s) {
      t_for_min_s = s_upper_bounds[i].t();
      min_s_idx = i;
      break;
    }
  }
  const auto& lateral_distances = st_boundary_ptr->lateral_signed_distances();
  // Calculate the s_upper_bounds vector caused by the obstacle
  if (t_for_min_s >= st_boundary_ptr->min_t()) {
    // Calculate the safe_dist_for_min_s
    double a_desire = 1.0;
    double safe_dist_for_min_s = min_s - init_v_ * init_v_ / (2 * a_desire);
    double lateral_dist_for_min_s = abs(lateral_distances[min_s_idx]);
    double lateral_dist_ratio_for_min_s = lateral_dist_for_min_s / (0.5 * vehicle_param_.width() + 0.3);
    double lateral_dist_factor_for_min_s = math::TableLookUp1D(
        lateral_distance_ratio_table_, lateral_distance_factor_for_safe_dist_table_, lateral_dist_ratio_for_min_s);
    double desired_safe_dist_for_min_s = fmax(lateral_dist_factor_for_min_s * max_safe_dist, min_safe_dist);
    safe_dist_for_min_s = fmax(fmin(safe_dist_for_min_s, desired_safe_dist_for_min_s), min_safe_dist);
    double s_soft_upper_bound = std::max(min_s - safe_dist_for_min_s, 0.0);
    double min_s_soft_upper_bound = s_soft_upper_bound;
    // if (parameter_.saturation_safe_dist_between_s_hard_upper_and_soft_upper()) {  // false
    //   s_soft_upper_bound = std::fmin(min_s - obs_safe_disance, s_soft_upper_bound);
    // }

    double safe_dist = 0.0;
    for (size_t i = 0UL; i < s_upper_bounds.size(); i++) {
      if (s_upper_bounds[i].t() <= t_for_min_s) {
        safe_dist = s_upper_bounds[i].s() - s_soft_upper_bound;
      } else {
        double lateral_dist = abs(lateral_distances[i]);
        double lateral_dist_ratio = lateral_dist / (0.5 * vehicle_param_.width() + 0.3);
        double lateral_dist_factor = math::TableLookUp1D(
            lateral_distance_ratio_table_, lateral_distance_factor_for_safe_dist_table_, lateral_dist_ratio);
        double desired_safe_dist = fmax(lateral_dist_factor * max_safe_dist, min_safe_dist);
        if (safe_dist <= desired_safe_dist) {
          double dt = (i >= 1UL) ? s_upper_bounds[i].t() - s_upper_bounds[i - 1UL].t() : 0.0;
          safe_dist += safe_dist_increasing_slope * dt;
        }
        safe_dist = fmin(safe_dist, desired_safe_dist);
        s_soft_upper_bound = std::max(s_upper_bounds[i].s() - safe_dist, min_s_soft_upper_bound);
        // if (parameter_.saturation_safe_dist_between_s_hard_upper_and_soft_upper()) {
        //   s_soft_upper_bound = std::fmin(s_upper_bounds[i].s() - obs_safe_disance, s_soft_upper_bound);
        // }
      }

      if (i > 0UL) {
        double last_t = st_soft_upper_boundary.back().t();
        double last_s = st_soft_upper_boundary.back().s();
        size_t N = static_cast<size_t>(round((s_upper_bounds[i].t() - last_t) / drivable_boundary_dt_));
        for (size_t j = 1UL; j <= N; j++) {
          double t = last_t + drivable_boundary_dt_ * static_cast<double>(j);
          double alpha = (t - last_t) / (s_upper_bounds[i].t() - last_t);
          double s = (1 - alpha) * last_s + alpha * s_soft_upper_bound;
          st_soft_upper_boundary.emplace_back(STPoint(s, t));
        }
      } else {
        st_soft_upper_boundary.emplace_back(STPoint(s_soft_upper_bound, s_upper_bounds[i].t()));
      }
    }
  }
}
/**
 * @brief 计算防止碰撞的最大减速度（防撞加速度）
 *
 * @param[in] complete_s_hard_upper_bounds 硬上界约束的时间-空间对（时间t，空间s）
 * @return double 防撞加速度（最大减速度，单位：m/s²）
 *
 * @details
 * 核心逻辑：
 * 1. 遍历所有时间点的硬上界约束
 * 2. 对每个有效时间点（t≥0.1s且s>0），基于匀减速运动公式计算最大允许减速度
 * 3. 取所有计算结果中的最小值作为最终防撞加速度
 *
 * @note 当多个时间点存在约束时，选择最严格的减速度约束（即最小值）
 *
 * @startuml
 * start
 * :初始化result=2.0（默认最大值）;
 * :遍历每个时间-空间对t_s_pair;
 *   :t = t_s_pair.first;
 *   :s = t_s_pair.second;
 *   if (s>0.0 且 t≥0.1) then (是)
 *     :计算当前减速度a=2*(s - init_s_ - init_v_*t)/(t*t);
 *     :result = min(result, a);
 *   endif
 * :返回result;
 * stop
 * @enduml
 */
double SpeedModelParam::calcAntiCollisionAccel(
    const std::vector<std::pair<double, double>>& complete_s_hard_upper_bounds) {
  double t, s;
  double result = 2.0;  // parameter_.a_hard_upper_bound();
  for (auto& t_s_pair : complete_s_hard_upper_bounds) {
    t = t_s_pair.first;
    s = t_s_pair.second;
    if ((s > 0.0) && (t >= 0.1)) {
      result = fmin(result, 2.0 * (s - init_s_ - init_v_ * t) / (t * t));
    }
  }
  return result;
}
/**
 * @brief 动态调整跟车时距系数K（核心实现）
 *
 * @param[in] local_view 局部环境视图（用于获取ACC状态和跟车距离等级）
 *
 * @details 实现步骤：
 * 1. 根据驾驶模式选择初始K0：
 *    - ACC模式：通过跟车距离等级查表获取
 *    - 常规模式：根据车速查表获取
 * 2. 通过风险值和初始K查表确定速度差调整斜率
 * 3. 存在前车时结合速度差动态调整最终K值
 *
 * @par 核心处理逻辑:
 * - 初始K值基于车速或跟车等级确定
 * - 速度差调整斜率通过双重查表确定（初始K查表 + 风险值查表）
 * - 最终K = K0 + delta_v_slope * 速度差（下限1.3）
 *
 * @startuml
 * start
 * if (ACC模式) then (是)
 *   :通过跟车等级获取K0;
 * else (否)
 *   :根据车速查表获取K0;
 * endif
 * :查表获取初始K对应的delta_v_slope;
 * :查表获取风险值对应的delta_v_slope;
 * :取两者较大值作为最终delta_v_slope;
 * if (存在前车让行对象) then (是)
 *   :K = K0 + delta_v_slope * 速度差;
 *   :确保K ≥ 1.3;
 * endif
 * stop
 * @enduml
 */
void SpeedModelParam::calcK(const LocalView& local_view) {
  // To Do
  if (behavior_state_.is_acc_state_ || behavior_state_.is_lcc_state_) {
    K_0_ = getAccThw(local_view.getConsolePtr()->followingDistanceLevel());
  } else {
    std::vector<double> ego_speed_km_h_table_for_K_0_ = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90};
    std::vector<double> K_0_table_wrt_ego_speed = {1.30, 1.30, 1.50, 1.50, 1.60, 1.70, 1.80, 1.90, 2.0, 2.1};
    K_0_ = math::TableLookUp1D(ego_speed_km_h_table_for_K_0_, K_0_table_wrt_ego_speed, MS_KMH * init_v_);
  }
  K_ = K_0_;
  // Calculate the delta_v_slope based on distance and risk
  std::vector<double> init_K_table_for_dv_slope = {0.0, 1.0, 2.0, 3.0, 4.0};
  std::vector<double> dv_slope_table_wrt_init_K = {0.4, 0.3, 0.2, 0.1, 0.0};

  std::vector<double> risk_table_for_dv_slope = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0};
  std::vector<double> dv_slope_table_wrt_risk = {0.0, 0.1, 0.15, 0.20, 0.25, 0.25};

  std::vector<double> object_dis_table = {40.0, 60.0, 80.0, 100.0, 120.0};
  std::vector<double> dv_attenuation_coeff_table = {1.0, 0.8, 0.7, 0.6, 0.5};
  double dv_attenuation_coeff =
      math::TableLookUp1D(object_dis_table, dv_attenuation_coeff_table, nearest_front_obj_invasion_s_);

  delta_v_slope_for_K_ = math::TableLookUp1D(init_K_table_for_dv_slope, dv_slope_table_wrt_init_K, init_K_);
  delta_v_slope_for_K_ =
      fmax(delta_v_slope_for_K_,
           dv_attenuation_coeff * math::TableLookUp1D(risk_table_for_dv_slope, dv_slope_table_wrt_risk, dv_risk_));
  ERT_PLOG_D << "  K_ = " << K_ << "  delta_v_slope_for_K_ = " << delta_v_slope_for_K_ ;
  if (has_front_yield_obj_) {
    K_ += delta_v_slope_for_K_ * deltaV_;
    K_ = fmax(K_, 1.3);
  }
  ERT_PLOG_D << " >>>>>>>>>>>>>>>>>>> K_ = " << K_ ;
}
/**
 * @brief 计算速度差对K值的调整斜率
 * @param[in] init_K 初始跟车时距系数，范围[0.0, 10.0]
 * @param[in] dv_risk 速度差风险值，范围[0.0, 9.0]
 * @return double 调整后的斜率值，范围[0.0, 0.6]
 * @details 通过双查表机制确定最终斜率值：
 *          1. 基于初始K值的线性插值
 *          2. 基于风险值的非线性插值
 * @par 查表参数:
 * - init_K_table_for_dv_slope = {0.0,1.0,2.0,3.0,4.0}
 * - dv_slope_table_wrt_init_K = {0.4,0.4,0.2,0.1,0.0}
 * - risk_table_for_dv_slope = {0.0,1.0,2.0,3.0,4.0,5.0}
 * - dv_slope_table_wrt_risk = {0.0,0.1,0.2,0.3,0.4,0.6}
 *
 * @startuml
 start
 :输入init_K和dv_risk;
 :查表获取init_K对应斜率;
 :查表获取dv_risk对应斜率;
 :取两者较大值作为最终结果;
 end
 @enduml
 */
double SpeedModelParam::calcKSlope(double init_K, double dv_risk) {
  std::vector<double> init_K_table_for_dv_slope = {0.0, 1.0, 2.0, 3.0, 4.0};
  std::vector<double> dv_slope_table_wrt_init_K = {0.4, 0.4, 0.2, 0.1, 0.0};

  std::vector<double> risk_table_for_dv_slope = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0};
  std::vector<double> dv_slope_table_wrt_risk = {0.0, 0.1, 0.2, 0.3, 0.4, 0.6};

  double delta_v_slope_for_K = math::TableLookUp1D(init_K_table_for_dv_slope, dv_slope_table_wrt_init_K, init_K);
  delta_v_slope_for_K =
      fmax(delta_v_slope_for_K, math::TableLookUp1D(risk_table_for_dv_slope, dv_slope_table_wrt_risk, dv_risk));
  return delta_v_slope_for_K;
}
/**
 * @brief 计算速度维度的软上界约束（核心实现）
 *
 * @param[in] local_view 局部环境视图（用于获取车辆状态）
 * @param[in] st_graph ST图（用于获取可行驶边界数量）
 * @param[in] path 离散化路径（用于曲率限速计算）
 * @param[in] path_v_t 路径速度时间序列（静态限速数据）
 *
 * @details 实现步骤：
 * 1. 获取可行驶边界数量并调整软上界数组大小
 * 2. 遍历每个时间点，通过静态限速查表生成软上界速度
 * 3. 将时间-速度对存入成员变量complete_v_soft_upper_bounds_
 *
 * @par 核心处理逻辑:
 * - 输入：静态限速路径点（path_v_t）
 * - 输出：时间-速度对的软上界约束（complete_v_soft_upper_bounds_）
 * - 当前实现主要处理静态限速约束，横向风险约束暂未完全实现
 *
 * @startuml
 * start
 * :获取可行驶边界数量complete_size;
 * if (数组大小不匹配) then (是)
 *   :调整complete_v_soft_upper_bounds_大小;
 * endif
 * :遍历每个时间点i;
 *   :计算当前时间t = i * 时间间隔;
 *   :通过静态限速查表获取v_soft_upper_bound;
 *   :存储(t, v_soft_upper_bound)到数组;
 * stop
 * @enduml
 */
void SpeedModelParam::calcCompleteVSoftBounds(const LocalView& local_view, const StGraph& st_graph,
                                              const DiscretizedPath& path,
                                              const vector<proto::TrajectoryPoint>& path_v_t) {
  // get curvature_speed_path
  std::vector<proto::TrajectoryPoint> curvature_speed_path;
  // get map_related_speed_path
  std::vector<proto::TrajectoryPoint> map_related_speed_path;
  // get obstacle lateral_risk_speed_path
  // std::vector<proto::TrajectoryPoint> lateral_risk_speed_path;
  // transform v-s  --->  v-t

  // Generating The completeVSoftUpper
  size_t complete_size = st_graph.st_drivable_boundaries().size();
  if (complete_v_soft_upper_bounds_.size() != complete_size) {
    complete_v_soft_upper_bounds_.resize(complete_size);
  }
  for (size_t i = 0UL; i < complete_size; i++) {
    double t = drivable_boundary_dt_ * static_cast<double>(i);
    double v_soft_upper_bound = getStaticVSoftBound(path_v_t, t);
    complete_v_soft_upper_bounds_[i] = std::make_pair(t, v_soft_upper_bound);
  }

  // if (parameter_.enable_lateral_risk_speed_limit()) {
  //   // SFIELD_DEBUG(lateral_risk, "main box lateral risk check");
  //   generateLateralRiskVSoftBound(frame);
  // }
}
/**
 * @brief 设置优化问题的全量约束参数（核心实现）
 *
 * @param[in] local_view 局部环境视图（提供车辆状态信息）
 * @param[in] st_graph ST图（提供可行驶边界和障碍物信息）
 * @param[in] max_speed_limit 最大车速限制（用于静态限速）
 * @return Status 状态码（成功或错误信息）
 *
 * @details 实现步骤：
 * 1. 时间网格校验：确保时间序列严格递增
 * 2. 初始化各维度约束的默认值（s/v/a/j的硬/软边界）
 * 3. 遍历每个时间网格点：
 *    - 计算s维度约束（结合障碍物和可行驶边界）
 *    - 计算v维度约束（结合静态限速和软上界）
 *    - 计算a维度约束（考虑横摆率和转向角速率）
 *    - 设置jerk边界
 *    - 生成参考轨迹点（s_ref, v_ref）
 * 4. 调用底层API设置QP问题参数
 *
 * @par 核心处理逻辑:
 * - 输入：时间网格、障碍物信息、车辆状态
 * - 输出：QP优化问题所需的约束参数矩阵
 * - 特殊处理：对变道场景和近距离障碍物场景的约束强化
 *
 * @startuml
 * start
 * :校验时间网格严格递增;
 * :初始化s/v/a/j默认约束;
 * :遍历每个时间点t_index;
 *   :计算当前时间t;
 *   :获取可行驶边界索引idx;
 *   :计算s硬/软上下界（结合障碍物）;
 *   :计算v硬/软上下界（静态+动态限速）;
 *   :计算a上下界（基于横摆率和转向角速率查表）;
 *   :设置jerk硬边界;
 *   :生成s_ref/v_ref参考值;
 *   :调用setParameter设置所有参数;
 * :返回Status::OK();
 * stop
 * @enduml
 */
Status SpeedModelParam::setContraintParam(const LocalView& local_view, const StGraph& st_graph,
                                          const double& max_speed_limit) {
  // time grid sanity check
  for (size_t i = 0; i < time_grid_.size() - 1; ++i) {
    if (time_grid_[i] >= time_grid_[i + 1]) {
      // ERT_PLOG_I << "time grid vector is invalid! " << static_cast<int>(i) << ": " << time_grid_[i] << " is not smaller
      // than "
      //      << static_cast<int>(i + 1) << ": " << time_grid_[i + 1] ;
      return Status(ErrorCode::PLANNING_ERROR, "time grid is invalid!");
    }
  }
  // ERT_PLOG_I << " ------------------------------------------" ;

  // init
  double t;
  double s_hard_upper_bound = 300.0;
  double s_soft_upper_bound = 300.0;
  double s_hard_lower_bound = -300.0;
  double s_soft_lower_bound = 0.0;
  double v_hard_upper_bound = kMaxSpeedMS + 10 * KMH_MS;
  double v_hard_lower_bound = 0.0;
  double v_soft_upper_bound = kMaxSpeedMS;
  double v_ref;
  double s_ref;
  double a_hard_upper_bound = 2.0;
  double a_hard_lower_bound = -6.0;
  double a_soft_upper_bound = 1.99;
  double a_soft_lower_bound = -5.0;
  double j_hard_upper_bound = 3.0;
  double j_hard_lower_bound = -8.0;

  // param
  double min_s_upper_for_free_road = 300;

  // WARNING!!! Every soft-upper-bound must be strictly smaller than hard-upper-bound,
  // Every soft-lower-bound must be strictly larger than hard-lower-bound otherwise causes
  // the problem of infeasibility in our frame.
  for (size_t t_index = 0; t_index < time_grid_.size(); ++t_index) {
    // idx in both completeSHardUpperBounds and st_graph.st_drivable_boundaries()
    t = time_grid_[t_index];
    size_t idx = static_cast<size_t>(round(t / drivable_boundary_dt_));
    idx = fmin(idx, complete_s_hard_upper_bounds_.size() - 1UL);
    // v
    double overspeed_percent = 0.2;  // TODO:计算障碍物无关的限速
    v_hard_upper_bound = fmax(max_speed_limit * (1 + overspeed_percent), x_0_.v + 5.0 * KMH_MS);  //
    v_soft_upper_bound = complete_v_soft_upper_bounds_[idx].second;  // getStaticVSoftBound(t);

    // s
    bool has_front_yield_obj = st_graph.st_drivable_boundaries()[idx].has_front_yield_obj;
    s_hard_upper_bound = has_front_yield_obj ? complete_s_hard_upper_bounds_[idx].second
                                             : fmax(time_horizon_ * v_hard_upper_bound, min_s_upper_for_free_road);
    s_soft_lower_bound = complete_s_soft_lower_bounds_[idx].second;
    s_soft_upper_bound = fmin(complete_s_soft_upper_bounds_[idx].second, s_hard_upper_bound - 0.1);
    if (t_index > 0) {
      double delta_t = t - time_grid_[t_index - 1];
      s_soft_upper_bound = fmin(s_soft_upper_bound, getParameter("SSoftUpperBound", t_index - 1) +
                                                        delta_t * getParameter("VHardUpperBound", t_index - 1));
    }
    boundValidProtect(s_hard_upper_bound, s_hard_lower_bound, s_soft_upper_bound, s_soft_lower_bound);

    // s_ref,v_ref
    v_ref = fmin(st_graph.st_drivable_boundaries()[idx].v_upper, max_speed_limit);
    s_ref = s_soft_upper_bound;

    // a

    std::vector<double> yaw_rate_table = {0.0, 0.08, 0.1, 0.115, 0.13, 0.15, 0.18};
    std::vector<double> a_upper_bound_table = {a_soft_upper_bound,
                                               a_soft_upper_bound,
                                               fmin(a_soft_upper_bound, 1.8),
                                               fmin(a_soft_upper_bound, 1.4),
                                               fmin(a_soft_upper_bound, 0.4),
                                               fmin(a_soft_upper_bound, 0.2),
                                               fmin(a_soft_upper_bound, 0.1)};
    double yaw_rate_acc = math::TableLookUp1D(yaw_rate_table, a_upper_bound_table,
                                              std::fabs(static_cast<double>(local_view.getChassisPtr()->YawRate())));
    // acceleration bound based on steering wheel rate
    double steering_wheel_rate_acc = a_soft_upper_bound;
    if (local_view.getChassisPtr()->Speed() <= 20.0 * KMH_MS && local_view.getChassisPtr()->Speed() >= 1.0) {
      double steering_wheel_rate = 0;
      // abs(local_view.getChassisPtr()->SteeringAngle() - last_steering_wheel_angle_) / kDelayTime;
      std::vector<double> speed_table_kmh = {0.0, 10.0, 15.0, 20.0};
      std::vector<double> steering_wheel_rate_bound_table = {270.0, 218.0, 180.0, 136.0};
      double steering_wheel_rate_bound =
          math::TableLookUp1D(speed_table_kmh, steering_wheel_rate_bound_table,
                              fmax(local_view.getChassisPtr()->Speed() * MS_KMH, kMathEpsilon));
      double steering_angle_ratio = steering_wheel_rate / (steering_wheel_rate_bound + kMathEpsilon);
      std::vector<double> ratio_table = {0.0, 0.6, 0.63, 0.7, 0.9};
      std::vector<double> steering_wheel_rate_a_bound_table = {
          a_soft_upper_bound, a_soft_upper_bound, fmin(a_soft_upper_bound, 1.5), fmin(a_soft_upper_bound, 0.3),
          fmin(a_soft_upper_bound, 0.1)};
      steering_wheel_rate_acc =
          math::TableLookUp1D(ratio_table, steering_wheel_rate_a_bound_table, steering_angle_ratio);
      std::vector<double> slack_a_weight_ratio_wrt_steering_rate_table = {1.0, 1.0, 1000.0, 10000.0, 30000.0};
    }

    double special_acc_bound = a_soft_upper_bound;
    special_acc_bound = fmin(fmin(special_acc_bound, yaw_rate_acc), steering_wheel_rate_acc);
    std::vector<double> time_table = {0.0, 1.0, 3.0, 8.0};
    std::vector<double> acc_table = {special_acc_bound, special_acc_bound, a_soft_upper_bound, a_soft_upper_bound};
    a_soft_upper_bound = math::TableLookUp1D(time_table, acc_table, t);
    // j

    // K
    double K = K_;
    double k = 0.3;
    const auto& drivable_boundary = st_graph.st_drivable_boundaries()[idx];
    std::string upper_obj_id = drivable_boundary.upper_obj_id;
    // ERT_PLOG_I<<"  upper_obj_id = "<<upper_obj_id;
    if (upper_obj_id != "None") {
      if (upper_obj_id == "tsr" || upper_obj_id == "junction_stop") {
        double speed = 0.0;
        double accel = 0.0;
        double dv_risk = init_v_ * init_v_ / fmax(drivable_boundary.s_upper_bound, 0.1);
        double init_K = (drivable_boundary.s_upper_bound - 1) / fmax(0.1, init_v_);
        double delta_v_slope_for_K = calcKSlope(init_K, dv_risk);
        K = K_0_ + 0.5 * fmax(0.0, -accel);
        K += 0.5 * delta_v_slope_for_K * fmax(init_v_ - speed, 0.0);  // consider speed diff
        // ERT_PLOG_I<<" dv_risk = "    <<dv_risk<<"  init_K = "<<init_K<<"  delta_v_slope_for_K = "<<delta_v_slope_for_K<<"
        // K = "<<K;

      } else {
        auto obs = findObstacle(upper_obj_id);
        if (obs) {
          const auto& follow_params = obs->follow_params();
          if (follow_params.follow_params_valid_) {
            double speed = follow_params.filtered_speed_;
            double accel = follow_params.filtered_accel_;
            K = K_0_ + 0.5 * fmax(0.0, -accel);
            // K += delta_v_slope_for_K_ * fmax(init_v_ - speed, 0.0);  // consider speed diff
            std::vector<double> time_table = {0.0, 0.3, 0.7, 1.0, 1.5, 2.0, 3.0, 4.0, 5.0};
            std::vector<double> k_attenuation_coeff_table = {1.0, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.3};
            double k_attenuation_coeff =
                math::TableLookUp1D(time_table, k_attenuation_coeff_table, time_grid_.at(t_index));
            K += delta_v_slope_for_K_ * (init_v_ - speed) * k_attenuation_coeff;  // consider speed diff
            K = fmax(1.3, K);
            // ERT_PLOG_I<<" speed = "<<speed<<"  accel = "<<accel<<"  K = "<<K;
          } else if (drivable_boundary.s_upper_bound == complete_s_hard_upper_bounds_[idx].second) {
            double K = K_0_;
            double lateral_dist = abs(drivable_boundary.upper_signed_lateral_distance);
            double lateral_dist_ratio = lateral_dist / (vehicle_param_.width() + 0.3);
            double lateral_distance_factor = math::TableLookUp1D(
                lateral_distance_ratio_table_, lateral_distance_factor_for_K_0_table_, lateral_dist_ratio);
            K = K_0_ * lateral_distance_factor;
            // SpeedPlannerObstacle* mutableObstacle(const string& id) { return obstacles_.Find(id);};
            const auto& obs_st_boundary = obs->pathStBoundary();
            if ((t >= obs_st_boundary.min_t()) && (t <= obs_st_boundary.max_t())) {
              double projected_speed = drivable_boundary.v_upper;
              projected_speed = fmax(projected_speed, 0.0);
              K += delta_v_slope_for_K_ * fmax(init_v_ - projected_speed, 0.0);
            }
            k = fmin(K, 0.3);
          } else {
            ;
          }
        }
      }
    }
    // boundValidProtect(s_hard_upper_bound, s_hard_lower_bound, s_soft_upper_bound, s_soft_lower_bound);

    setParameter("SHardUpperBound", t_index, s_hard_upper_bound);
    setParameter("SHardLowerBound", t_index, s_hard_lower_bound);
    setParameter("SSoftUpperBound", t_index, s_soft_upper_bound);
    setParameter("SSoftLowerBound", t_index, s_soft_lower_bound);
    setParameter("VHardUpperBound", t_index, v_hard_upper_bound);
    setParameter("VHardLowerBound", t_index, v_hard_lower_bound);
    setParameter("VSoftUpperBound", t_index, v_soft_upper_bound);
    setParameter("SRef", t_index, s_ref);
    setParameter("VRef", t_index, v_ref);
    setParameter("AHardUpperBound", t_index, a_hard_upper_bound);
    setParameter("ASoftUpperBound", t_index, a_soft_upper_bound);
    setParameter("AHardLowerBound", t_index, a_hard_lower_bound);
    setParameter("ASoftLowerBound", t_index, a_soft_lower_bound);
    setParameter("JHardUpperBound", t_index, j_hard_upper_bound);
    setParameter("JHardLowerBound", t_index, j_hard_lower_bound);
    setParameter("K", t_index, K);
    setParameter("k", t_index, k);
    setParameter("SUpperBoundForDVConstraint", t_index, s_hard_upper_bound);
  }
  return Status::OK();
}
/**
 * @brief 设置优化问题的权重参数（核心实现）
 *
 * @details 实现步骤：
 * 1. 初始化默认权重：s/v/a/j权重及松弛变量权重
 * 2. 动态调整v权重：
 *    - 基于速度差风险(dv_risk_)查表调整
 *    - 结合初始K值(init_K_)查表获取调整因子
 * 3. 调整s权重：根据自车与障碍物的距离差(deltaS_)查表确定
 * 4. 处理特殊场景权重：
 *    - 近距离障碍物增大松弛权重
 *    - 起步场景特殊处理（TODO：当前未完全实现）
 * 5. 遍历时间网格，设置所有时间点的权重参数
 *
 * @par 核心处理逻辑:
 * - 输入：速度差风险、初始K值、距离差等状态参数
 * - 输出：QP优化问题所需的权重参数矩阵
 * - 权重调整策略：通过多重查表机制实现动态权重调整
 *
 * @startuml
 * start
 * :初始化默认权重;
 * :查表调整v_weight（基于速度差风险）;
 * :查表调整s_weight（基于距离差）;
 * :处理特殊场景（近距离障碍物、起步场景）;
 * :遍历时间点，调用setParameter设置所有权重;
 * stop
 * @enduml
 */
void SpeedModelParam::setWeightParam() {
  // init
  //  Default Weights:
  double s_weight = 4.0;
  double v_weight = 1.0;
  double a_weight = 100.0;
  double j_weight = 10000.0;
  double slack_s_upper_weight = 100000.0;
  double slack_s_lower_weight = 0.1;
  double slack_v_upper_weight = 100000000.0;
  double slack_a_upper_weight = 100.0;
  double slack_a_lower_weight = 100.0;
  double slack_dv_weight = 1000.0;

  // vWeight_ = vWeight_0_;
  std::vector<double> dv_risk_table = {0.0, 0.1, 0.2, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0,
                                       1.5, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  std::vector<double> v_weight_table_wrt_dv_risk = {1.0,    10.0,   10.0,   100.0,  150.0,  200.0,
                                                    200.0,  200.0,  200.0,  300.0,  500.0,  500.0,
                                                    1000.0, 4000.0, 8000.0, 8000.0, 8000.0, 8000.0};
  std::vector<double> min_v_weight_table_wrt_dv_risk = {10.0, 10.0, 10.0,  10.0,  10.0,  20.0,  30.0,  40.0,  50.0,
                                                        65.0, 80.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0};
  // std::vector<double> min_v_weight_table_wrt_dv_risk = {10.0, 10.0, 10.0,  10.0,  10.0,  20.0,  30.0,  40.0,  50.0,
  //                                                       65.0, 100.0, 500.0, 1000.0, 4000.0, 20000.0, 20000.0,
  //                                                       20000.0, 20000.0};

  double k_factor_for_k_6 = 0.075;  // parameter_.k_factor_for_k_6();
  double k_factor_for_k_7 = 0.05;   // parameter_.k_factor_for_k_7();
  double k_factor_for_k_8 = 0.03;   // parameter_.k_factor_for_k_8();
  double k_factor_for_k_9 = 0.03;   // parameter_.k_factor_for_k_9();
  double k_factor_for_k_10 = 0.03;  // parameter_.k_factor_for_k_10();
  std::vector<double> init_K_table = {0.0, 1.0, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.5, 6.0, 7.0, 8.0, 9.0, 10.0};
  std::vector<double> init_K_factor_table = {1.0,
                                             1.0,
                                             1.0,
                                             0.5,
                                             0.1,
                                             0.1,
                                             0.1,
                                             0.1,
                                             0.1,
                                             k_factor_for_k_6,
                                             k_factor_for_k_7,
                                             k_factor_for_k_8,
                                             k_factor_for_k_9,
                                             k_factor_for_k_10};
  double init_K_factor = math::TableLookUp1D(init_K_table, init_K_factor_table, init_K_);
  v_weight = math::TableLookUp1D(dv_risk_table, v_weight_table_wrt_dv_risk, dv_risk_);
  double min_v_weight = math::TableLookUp1D(dv_risk_table, min_v_weight_table_wrt_dv_risk, dv_risk_);
  v_weight = fmax(min_v_weight, v_weight * init_K_factor);
  v_weight = 1.0;

  // SWeight w.r.t. deltaS:
  std::vector<double> deltaSTableForSWeight = {-20,  -10,  -5.0, -1.0, 0.0,   5.0,   10.0, 20.0,
                                               30.0, 40.0, 50.0, 70.0, 100.0, 200.0, 300.0};
  std::vector<double> sWeightTableWRTdeltaS = {
      50.0, 30.0, 20.0, 10.0, 5.0, 5.0, 5.0, 5.0,
      // std::vector<double> sWeightTableWRTdeltaS = {300.0, 100.0, 50.0, 20.0, 5.0, 5.0, 5.0, 5.0,
      5.0, 5, 5.0, 5.0, 5.0, 5.0, 5.0};
  s_weight = math::TableLookUp1D(deltaSTableForSWeight, sWeightTableWRTdeltaS, deltaS_);

  if (nearest_front_obj_obstacle_ && complete_s_soft_upper_bounds_.size() > 0 &&
      complete_s_soft_upper_bounds_[0].second < 3.0) {
    slack_s_upper_weight = 10000000.0;
  }

  std::vector<double> dv_risk_table_for_slack_dv_weight = {0.0, 1.0, 1.5, 1.6, 1.7, 1.8, 1.9, 2.0, 2.5, 3.0};
  std::vector<double> slack_dv_weight_table_wrt_dv_risk = {1.0,   1.0,    1.0,    1.0,    1.0,
                                                           500.0, 1000.0, 5000.0, 5000.0, 5000.0};
  slack_dv_weight = math::TableLookUp1D(dv_risk_table_for_slack_dv_weight, slack_dv_weight_table_wrt_dv_risk, dv_risk_);
  // TODO: 确认此函数对于起步的影响
  // specialCaseOfVehicleStart(s_weight);

  slack_dv_weight = 10000.0;

  // Determine the slackvupperweight:

  double epsilon = 1E-3;
  // SFIELD_DEBUG(qp_optimizer, "[VWeight001]: VWeight = {}", vWeight_);
  for (size_t i = 0UL; i < time_grid_.size(); i++) {
    setParameter("JWeight", i, j_weight * epsilon);
    setParameter("SWeight", i, s_weight * epsilon);
    setParameter("VWeight", i, v_weight * epsilon);
    setParameter("AWeight", i, a_weight * epsilon);
    setParameter("SlackSUpperWeight", i, slack_s_upper_weight * epsilon);
    setParameter("SlackSLowerWeight", i, slack_s_lower_weight * epsilon);
    setParameter("SlackVUpperWeight", i, slack_v_upper_weight * epsilon);
    setParameter("SlackAUpperWeight", i, slack_a_upper_weight * epsilon);
    setParameter("SlackALowerWeight", i, slack_a_lower_weight * epsilon);
    setParameter("SlackDVWeight", i, slack_dv_weight * epsilon);
    setParameter("SafeDistForDVConstraint", i, 4.0);
  }
}
/**
 * @brief 权重参数保护机制（核心实现）
 *
 * @details 实现步骤：
 * 1. 定义保护函数：遍历所有时间点，确保权重参数不小于最小值
 * 2. 对所有优化相关权重参数应用保护机制，防止数值过小
 *
 * @par 核心处理逻辑:
 * - 输入：各时间点的原始权重参数
 * - 输出：经过下限保护的权重参数
 * - 安全机制：确保所有权重 ≥ 0.01（避免优化问题数值不稳定）
 *
 * @startuml
 * start
 * :定义保护lambda函数;
 * :遍历所有时间点;
 *   :对每个权重参数取max(当前值, 0.01);
 * :对S/V/A/J权重及松弛权重调用保护函数;
 * stop
 * @enduml
 */
void SpeedModelParam::weightsProtection() {
  const double min_weight = 0.01;
  auto weight_protect = [&](std::string name) {
    for (size_t i = 0; i < time_grid_.size(); ++i) {
      setParameter(name, i, fmax(getParameter(name, i), min_weight));
    }
  };
  weight_protect("SWeight");
  weight_protect("VWeight");
  weight_protect("AWeight");
  weight_protect("JWeight");
  weight_protect("SlackSUpperWeight");
  weight_protect("SlackSLowerWeight");
  weight_protect("SlackVUpperWeight");
  weight_protect("SlackAUpperWeight");
  weight_protect("SlackALowerWeight");
  weight_protect("SlackDVWeight");
}
/**
 * @brief 设置变道场景约束
 * @param[in] decision_result 决策结果数据
 * @details 变道时调整参考轨迹的s/v值，并增加跟踪权重
 * @par 调整策略:
 * - v_ref_weight从1000调整为100000
 * - s_ref根据决策轨迹插值计算
 * - 松弛权重调整系数epsilon=1E-3
 * @par 变量说明:
 * - v_weight_for_lc: 变道时速度权重，默认1000.0
 * - slack_s_lower_weight_for_lc: 下界松弛权重，默认1000000.0
 * @note 仅在决策状态为LEFT_CHANGE/RIGHT_CHANGE时生效
 *
 * @startuml
 start
 :检查变道状态;
 if (非变道状态) then (yes)
   :返回;
 endif
 :遍历时间网格;
   :插值获取参考s/v值;
   :设置SRef/VRef参数;
   :增大跟踪权重;
 end
 @enduml
 */
void SpeedModelParam::updateDecisionCoarseTraj(const DecisionResult& decision_result) {
  // update v_ref and s_ref
  auto decision_coarse_traj = decision_result.getRefTrajInfo();
  if (decision_coarse_traj->traj_points.empty() || decision_result.getCurrFsmState() == FsmState::KEEP) {
    return;
  }
  // for(auto& pt: decision_coarse_traj->traj_points) {
  //   ERT_PLOG_I<<" s = "<<pt.s<<"  v ="<<pt.v<<" a = "<<pt.a << "t =" <<pt.t;
  // }
  double epsilon = 1E-3;
  double v_weight = 1000.0;
  for (size_t t_index = 0; t_index < time_grid_.size(); ++t_index) {
    double v_ref = 0.0;
    double s_ref = 0.0;
    if (getRefFromDecisionResult(*decision_coarse_traj, time_grid_[t_index], v_ref, s_ref)) {
      setParameter("SRef", t_index, s_ref);
      setParameter("VRef", t_index, v_ref);
      setParameter("VWeight", t_index, v_weight * epsilon);
    }
  }
}
/**
 * @brief 从决策轨迹插值获取参考速度/位置（核心实现）
 * @param[in] trajectory 决策参考轨迹（含时间-速度-位置序列）
 * @param[in] t 当前查询时间（单位：秒）
 * @param[out] v_ref 输出参考速度（单位：m/s）
 * @param[out] s_ref 输出参考位置（相对轨迹起点的累积距离，单位：m）
 * @return bool 是否成功获取参考值
 * 
 * @par 处理流程:
 * @startuml
start
if (t ≥ 轨迹末端时间?) then (yes)
    :取末端速度v_ref=traj_points.back().v;
    :计算s_ref=末端位置 + 时间差*末端速度 - 起点位置;
else (no)
    :遍历轨迹点查找时间区间[i, i+1];
    :在相邻轨迹点间线性插值计算v_ref/s_ref;
endif
stop
@enduml
 *
 * @note 特性说明:
 * - 当t超过轨迹时间范围时，使用末端速度外推
 * - 相对位置计算：s_ref = 插值s值 - 轨迹起点s值
 * - 线性插值方法：math::lerp
 * 
 * @par 参数说明:
 * | 参数        | 类型                | 取值范围          | 单位 | 说明                  |
 * |-------------|---------------------|------------------|------|-----------------------|
 * | trajectory  | Decision::RefTrajInfo | t ∈ [0, time_horizon] | 秒 | 决策模块输出的参考轨迹|
 * | t           | double              | ≥0               | 秒   | 当前查询时刻          |
 * | v_ref       | double              | [0, max_speed]   | m/s  | 时间t对应的参考速度   |
 * | s_ref       | double              | ≥0               | m    | 时间t对应的参考位置    |
 */
bool SpeedModelParam::getRefFromDecisionResult(const Decision::RefTrajInfo& trajectory, const double& t, double& v_ref,
                                               double& s_ref) {
  auto& traj_points = trajectory.traj_points;
  if (trajectory.traj_points.back().t <= t) {
    v_ref = traj_points.back().v;
    s_ref = traj_points.back().s + (t - traj_points.back().t) * v_ref - traj_points.front().s;
    return true;
  }
  for (size_t i = 0; i < traj_points.size() - 1; ++i) {
    if (traj_points.at(0).t > t) {
      v_ref = traj_points.at(0).v;
      s_ref = 0;
      return true;
    }
    if (traj_points.at(i).t <= t && traj_points.at(i + 1).t > t) {
      v_ref = math::lerp(traj_points.at(i).v, traj_points.at(i).t, traj_points.at(i + 1).v, traj_points.at(i + 1).t, t);
      s_ref =
          math::lerp(traj_points.at(i).s, traj_points.at(i).t, traj_points.at(i + 1).s, traj_points.at(i + 1).t, t) -
          traj_points.front().s;
      return true;
    }
  }
  return false;
}
/**
 * @brief 变道场景动态参数调整（HWT参数更新与滤波）
 * @param[in] decision_result 决策结果数据
 * @param[in] obstacle_set 障碍物集合
 * @param[in] st_graph ST图数据
 * 
 * @par 处理流程:
 * @startuml
start
partition 场景判断 {
  if (非变道状态?) then (是)
    :遍历时间网格;
      :检查历史HWT参数;
      :应用指数衰减滤波(0.95历史系数);
      if (参数收敛?) then (是)
        :清除历史记录;
      endif
    :直接返回;
  else (否)
    :遍历时间网格;
  endif
}
partition 前车处理 {
  :识别前车障碍物;
  :基于决策结果更新HWT参数;
  :应用HWT参数滤波;
}
@enduml
 *
 * @par 关键参数调整:
 * | 参数 | 调整规则                          | 生效条件                |
 * |------|----------------------------------|-------------------------|
 * | K    | 前车HWT决策值滤波更新            | 前车存在且HWT>0.5       |
 *
 * @note 功能特性:
 * - HWT滤波机制：采用0.95历史系数进行平滑滤波
 * - 状态判断：涵盖LEFT_ATTEMPT/RIGHT_ATTEMPT等6种变道相关FSM状态
 * - 收敛条件：滤波后参数变化量<0.1时清除历史值
 *
 * @warning 使用约束:
 * - 需与ST图处理器同步更新drivable_boundary_dt参数
 * - 必须在processObstacles之后调用
 * - 滤波系数需与决策模块参数对齐
 */
void SpeedModelParam::specialCaseForLaneChange(const DecisionResult& decision_result, const ObstacleSet& obstacle_set,
                                               const StGraph& st_graph) {
  size_t complete_size = st_graph.st_drivable_boundaries().size();
  if (history_obj_hwt_.size() != complete_size) {
    history_obj_hwt_.resize(complete_size);
  }
  auto& current_fsm_state = decision_result.getCurrFsmState();
  bool is_lane_change = (current_fsm_state == FsmState::LEFT_ATTEMPT || current_fsm_state == FsmState::RIGHT_ATTEMPT ||
                         current_fsm_state == FsmState::LEFT_CHANGE || current_fsm_state == FsmState::RIGHT_CHANGE ||
                         current_fsm_state == FsmState::LEFT_HOLD || current_fsm_state == FsmState::RIGHT_HOLD);
  if (!is_lane_change) {
    for (size_t t_index = 0; t_index < time_grid_.size(); ++t_index) {
      auto t = time_grid_[t_index];
      size_t idx = static_cast<size_t>(round(t / drivable_boundary_dt_));
      idx = fmin(idx, st_graph.st_drivable_boundaries().size() - 1UL);
      // update front_obstacle K
      const auto& drivable_boundary = st_graph.st_drivable_boundaries()[idx];
      std::string upper_obj_id = drivable_boundary.upper_obj_id;
      if (history_obj_hwt_[idx].first == upper_obj_id) {
        auto k = getParameter("K", t_index);
        auto fliter_hwt = history_obj_hwt_[idx].second * 0.95 + 0.05 * getParameter("K", t_index);
        setParameter("K", t_index, fliter_hwt);
        if (abs(fliter_hwt - k) < 0.1) {
          history_obj_hwt_[idx].first = "";
          history_obj_hwt_[idx].second = 0.0;
        } else {
          history_obj_hwt_[idx].second = fliter_hwt;
        }
      }
    }
    return;
  }

  double t = 0.0;
  constexpr double slack_slower_weight_for_overtake = 10000.0;
  constexpr double slack_v_upper_weight_for_overtake = 100.0;
  constexpr double kEpsilon = 1E-3;
  for (size_t t_index = 0; t_index < time_grid_.size(); ++t_index) {
    t = time_grid_[t_index];
    size_t idx = static_cast<size_t>(round(t / drivable_boundary_dt_));
    idx = fmin(idx, st_graph.st_drivable_boundaries().size() - 1UL);
    // update front_obstacle K
    const auto& drivable_boundary = st_graph.st_drivable_boundaries()[idx];
    std::string upper_obj_id = drivable_boundary.upper_obj_id;
    if (drivable_boundary.has_front_yield_obj && obstacle_set.find(upper_obj_id) != obstacle_set.end()) {
      double decision_hwt = obstacle_set.find(upper_obj_id)->second->getObstacleDecisionHwt();
      if (decision_hwt > speed_ocp_qp_optimizer_config_.decision_thw_threshold()) {
        setParameter("K", t_index, decision_hwt);
        history_obj_hwt_[idx].first = upper_obj_id;
        history_obj_hwt_[idx].second = decision_hwt;
      }
    }
  }
}
/**
 * @brief 处理车辆起步场景的特殊权重调整
 *
 * @param[in,out] s_weight s维度优化权重（输入原始值，输出调整后的值）
 *
 * @details 实现步骤：
 * 1. 计算与前方障碍物的安全距离
 * 2. 通过四维查表获取调整因子：
 *    - 初始速度因子（egoSpeedFactor）
 *    - 距离因子（distanceFactor）
 *    - HWT/K比值因子（hwt_K_ratio_factor）
 *    - 速度差因子（delta_v_factor）
 * 3. 综合因子计算权重调整系数（needLargeSWeightForLowSpeed）
 * 4. 线性插值生成最终s_weight（原始权重与起步专用权重的混合）
 *
 * @par 核心处理逻辑:
 * - 输入：自车初始速度、障碍物距离、速度差等状态参数
 * - 输出：调整后的s维度优化权重
 * - 安全机制：当障碍物速度≥60km/h时禁用权重调整
 *
 * @startuml
 * start
 * :计算自车与障碍物的安全距离;
 * :查表获取初始速度因子;
 * :查表获取距离因子;
 * :计算HWT/K比值并查表获取比值因子;
 * :查表获取速度差因子;
 * :计算综合调整系数;
 * if (障碍物速度≥60km/h) then (是)
 *   :禁用权重调整;
 * endif
 * :s_weight = 原始权重 * (1 - 系数) + 起步权重 * 系数;
 * stop
 * @enduml
 */
void SpeedModelParam::specialCaseOfVehicleStart(double& s_weight) {
  double distance =
      nearest_front_obj_invasion_s_ - nearest_front_obj_invasion_t_ * fmax(0.0, nearest_front_obj_invasion_v_);
  std::vector<double> init_v_table_for_start_ = {0.0, 10.0, 15.0, 20.0, 30.0, 40.0, 50.0, 60.0};  // v in KM/H
  std::vector<double> init_v_factor_table_for_start_ = {1.0, 1.0, 0.5, 0.2, 0.1, 0.0, 0.0};
  std::vector<double> distance_table_for_start_ = {0.0, 50.0, 70.0, 100};
  std::vector<double> distance_factor_table_for_start_ = {1.0, 1.0, 0.0, 0.0};
  double egoSpeedFactor = math::TableLookUp1D(init_v_table_for_start_, init_v_factor_table_for_start_, init_v_ * 3.6F);
  double distanceFactor = math::TableLookUp1D(distance_table_for_start_, distance_factor_table_for_start_, distance);
  double hwt_K_ratio = initHWT_ / K_;
  std::vector<double> hwt_K_ratio_table_for_start_ = {0.0, 1.0, 1.2, 1.5, 2.0};
  std::vector<double> hwt_K_ratio_factor_table_for_start_ = {0.0, 0.0, 0.0, 1.0, 1.0};
  double hwt_K_ratio_factor =
      math::TableLookUp1D(hwt_K_ratio_table_for_start_, hwt_K_ratio_factor_table_for_start_, hwt_K_ratio);
  std::vector<double> delta_v_table = {0.2, 1.5};
  std::vector<double> delta_v_factor_table = {0.0, 1.0};
  double s_weight_for_start_ = 10000;
  // if (config_manager_->vehicle_type() == proto::SensorTable::BT ||
  //     config_manager_->vehicle_type() == proto::SensorTable::DT) {
  //   if (nearest_front_obj_id_ != "None") {
  //     double safe_distance =
  //         nearest_front_obj_obstacle_ ? parameter_.od_safe_distance() : nearest_speed_wall_stop_distance_;
  //     if (abs(nearest_front_obj_invasion_v_) < 0.1 && init_v_ < 0.5 &&
  //         nearest_front_obj_invasion_s_ - safe_distance > 3.0) {
  //       delta_v_factor_table = {0.2, 1.0};
  //       s_weight_for_start_ = 10000;
  //     }
  //   }
  // }
  double nearest_front_obj_delta_v = nearest_front_obj_invasion_v_ - init_v_;
  double delta_v_factor = math::TableLookUp1D(delta_v_table, delta_v_factor_table, nearest_front_obj_delta_v);
  double needLargeSWeightForLowSpeed = egoSpeedFactor * distanceFactor * hwt_K_ratio_factor * delta_v_factor;
  if (60 * KMH_MS <= nearest_front_obj_invasion_v_) {
    needLargeSWeightForLowSpeed = 0.0F;
  }
  s_weight = s_weight * (1.0 - needLargeSWeightForLowSpeed) + s_weight_for_start_ * needLargeSWeightForLowSpeed;
}
/**
 * @brief 处理静态障碍物停车场景的特殊约束
 *
 * @details 实现步骤：
 * 1. 识别静态障碍物类型（终点、红绿灯、路口停止线等）
 * 2. 计算停车减速度（stop_accel = -0.5*v²/(front_distance - safe_distance)）
 * 3. 根据障碍物类型动态调整参数阈值：
 *    - 红绿灯场景：增大减速度阈值和距离阈值
 * 4. 对低速停车场景（速度<5m/s，距离<15m）进行特殊处理：
 *    - 调整参考速度v_ref为停车轨迹速度
 *    - 调整参考位置s_ref为停车轨迹位置
 *    - 增大s和v的优化权重（s_ref_weight=100000, v_ref_weight=10000）
 *    - 设置s下界松弛权重（SlackSLowerWeight=100000）
 *
 * @par 核心处理逻辑:
 * - 输入：最近障碍物信息、自车状态
 * - 输出：优化问题的s/v参考轨迹及权重参数
 * - 安全机制：对终点障碍物保持0.5米位置裕度
 *
 * @startuml
 * start
 * :判断障碍物类型（终点/红绿灯/其他）;
 * :计算安全距离front_distance;
 * :计算停车减速度stop_accel;
 * :动态调整参数阈值（红绿灯场景特殊处理）;
 * if (低速停车场景) then (是)
 *   :遍历时间网格调整参考轨迹;
 *   :设置高s/v权重强制跟踪停车轨迹;
 * endif
 * stop
 * @enduml
 */
void SpeedModelParam::specialCaseForObstacleStop() {
  double nearest_obstacle_static = 0.0;
  double front_distance = 300.0F;
  double safe_distance = 5.0;
  if (nearest_front_obj_obstacle_) {
    auto obs = findObstacle(nearest_front_obj_id_);
    if (obs == nullptr || (obs->pathStBoundary().boundary_type() != STBoundary::BoundaryType::FOLLOW &&
                           obs->pathStBoundary().boundary_type() != STBoundary::BoundaryType::YIELD)) {
      ERT_PLOG_D << "!!!!!!!!!!!!!!!!!!!!!!!!!!! obstacle is not follow !!!!!!!!!!" ;
      return;
    }
    // const auto& follow_params = obs->follow_params();
    // if (!follow_params.follow_params_valid_) {
    //   return;
    // }
    // Judge if it's a purely stop scenario

    std::vector<double> speed_ratio_table = {0.0, 0.1, 0.2, 0.3};
    std::vector<double> low_speed_logic_table_wrt_speed_ratio = {1.0, 1.0, 0.6, 0.0};
    double speed_ratio = nearest_front_obj_invasion_v_ / fmax(init_v_, 1.0);
    double low_speed_wrt_speed_ratio =
        math::TableLookUp1D(speed_ratio_table, low_speed_logic_table_wrt_speed_ratio, speed_ratio);

    std::vector<double> speed_table = {0.0, 0.5, 1.0, 2.0};
    std::vector<double> low_speed_logic_table_wrt_speed = {1.0, 1.0, 0.8, 0.0};
    double low_speed_wrt_speed =
        math::TableLookUp1D(speed_table, low_speed_logic_table_wrt_speed, nearest_front_obj_invasion_v_);

    nearest_obstacle_static = 1.0F - (1.0F - low_speed_wrt_speed_ratio) * (1.0F - low_speed_wrt_speed);

    front_distance = nearest_front_obj_invasion_s_;
  } else if (nearest_front_obj_id_ == speed_wall_id_map.at(DESTINATION_WALL)) {
    nearest_obstacle_static = 1.0;
    front_distance = nearest_front_obj_invasion_s_;
    safe_distance = 0;
  } else if (nearest_front_obj_id_ == speed_wall_id_map.at(TFL_WALL) ||
             nearest_front_obj_id_ == speed_wall_id_map.at(JUNCTION_STOP) ||
             nearest_front_obj_id_ == speed_wall_id_map.at(FREESPACE_WALL)) {
    nearest_obstacle_static = 1.0;
    front_distance = nearest_front_obj_invasion_s_;
    safe_distance = 1.0;
  } else {
    return;
  }

  double init_v_for_obstacle_stop = init_v_;
  double stop_accel = (front_distance - safe_distance >= 0.1)
                          ? -(0.5 * init_v_ * init_v_) / (front_distance - safe_distance)
                          : 1.2 * antiCollisionAccel_;

  // Special Treatment for low speed
  double param_stop_accel_threshold = -1.0;
  double param_stop_ego_speed_threshold = 5.0;
  double param_stop_distance_threshold = 15.0;
  if (nearest_front_obj_id_ == speed_wall_id_map.at(TFL_WALL)) {
    param_stop_accel_threshold = -1.5;
    param_stop_ego_speed_threshold = 7.0;
    param_stop_distance_threshold = 25.0;
  }

  bool need_special_treatment_for_low_speed_stop =
      (init_v_for_obstacle_stop <= param_stop_ego_speed_threshold) && (nearest_obstacle_static >= 0.9) &&
      (front_distance - safe_distance >= 0.1) && (front_distance - safe_distance <= param_stop_distance_threshold) &&
      (stop_accel >= param_stop_accel_threshold);

  ERT_PLOG_D << "init_v_for_obstacle_stop =  " << init_v_for_obstacle_stop
       << " nearest_obstacle_static =  " << nearest_obstacle_static << "  front_distance = " << front_distance
       << "  safe_distance = " << safe_distance << "  stop_accel =  " << stop_accel ;

  if (need_special_treatment_for_low_speed_stop) {
    if (init_v_ <= 1.5 && (front_distance - safe_distance) >= 0.5) {
      init_v_for_obstacle_stop = 1.5;
      stop_accel = -(0.5 * init_v_for_obstacle_stop * init_v_for_obstacle_stop) / (front_distance - safe_distance);
    }
    ERT_PLOG_D << " !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! specail case obstacle stop !!!!!!!!!!!!!!!" ;
    double s_ref_weight = 10000.0, v_ref_weight = 10000.0;
    if (nearest_front_obj_id_ == speed_wall_id_map.at(DESTINATION_WALL) ||
        nearest_front_obj_id_ == speed_wall_id_map.at(TFL_WALL) || 
        nearest_front_obj_id_ == speed_wall_id_map.at(JUNCTION_STOP) || 
        nearest_front_obj_id_ == speed_wall_id_map.at(FREESPACE_WALL)) {
      s_ref_weight = 100000.0, v_ref_weight = 10000.0;
    }
    for (size_t i = 0UL; i < time_grid_.size(); i++) {
      double t = time_grid_[i];
      double epsilon = 1E-3;
      double s_ref = (stop_accel * t + init_v_for_obstacle_stop <= 0.0)
                         ? front_distance - safe_distance
                         : init_v_for_obstacle_stop * t + 0.5 * stop_accel * t * t;
      double v_ref = fmax(stop_accel * t + init_v_for_obstacle_stop, 0.0);
      setParameter("k", i, 0.1);
      setParameter("K", i, 0.0);
      setParameter("VRef", i, v_ref);
      // use_empirical_vref_.at(i) = true;
      setParameter("VWeight", i, 1000.0 * epsilon);
      setParameter("SRef", i, s_ref);
      setParameter("VWeight", i, v_ref_weight * epsilon);
      setParameter("SWeight", i, s_ref_weight * epsilon);
      if (nearest_front_obj_id_ == speed_wall_id_map.at(DESTINATION_WALL) && (front_distance - safe_distance) > 0.2) {
        double s_soft_lower_bound = fmax(s_ref - 0.5, 0.0);
        s_soft_lower_bound = fmax(getParameter("SSoftLowerBound", i),
                                  fmin(s_soft_lower_bound, getParameter("SSoftUpperBound", i) - 0.1));
        setParameter("SSoftLowerBound", i, s_ref - 0.5);
        setParameter("SlackSLowerWeight", i, 100000.0F * epsilon);
      }
    }
  }
}

/**
 * @brief 处理汇入障碍物场景的动态参数调整（速度限制与加速度规划）
 * @param[in] path_group 路径信息组（含扩展路径）
 * @param[in] obstacle_map 障碍物集合（含风险ST边界）
 * 
 * @par 处理流程:
 * @startuml
start
partition 关键障碍物识别 {
  :遍历障碍物集合;
  :筛选具有合并风险且ST边界非空的障碍物;
  :记录最小入侵s值对应的关键障碍物;
}
if (无关键障碍物?) then (是)
  :直接返回;
else (否)
  :计算障碍物入侵s/t坐标;
  :投影计算障碍物速度;
endif
partition 速度限制调整 {
  :查表获取速度差delta_v;
  :计算安全汇入加速度;
  :遍历时间网格设置S/V约束;
}
@enduml
 *
 * @par 关键参数调整:
 * | 参数        | 调整规则                          | 生效条件                |
 * |-------------|----------------------------------|-------------------------|
 * | VRef        | 基于障碍物速度插值计算          | 全程生效                |
 * | SSoftUpperBound | 取障碍物s_lower - 安全距离    | 障碍物存在风险ST边界    |
 * | K           | 设置为0                          | 全程生效                |
 *
 * @note 功能特性:
 * - 速度投影：根据障碍物ST边界计算纵向投影速度
 * - 安全距离：3.0米固定值补偿
 * - 加速度规划：采用舒适加加速度(comfort_jerk)进行三次曲线规划
 * 
 * @warning 使用约束:
 * - 需在ST图处理完成后调用
 * - 依赖speed_ocp_qp_optimizer_config配置参数
 * - 时间网格需严格递增
 */
void SpeedModelParam::specialCaseForMergeObstacle(const PathGroup& path_group, const ObstacleSet& obstacle_map){

  bool merge_trigger = false;
  double merge_s = 300.0;
  double key_obs_v = kMaxSpeedMS;
  SpeedPlannerObstacle key_obs;
  
  for (const auto& obs : obstacle_map) {
    if( !obs.second->isMergeObstacle() || 
        obs.second->riskStBoundary().IsEmpty()){
      continue;
    }
    double current_invasion_s = obs.second->riskStBoundary().bottom_right_point().s();
    if (current_invasion_s < merge_s) {
      merge_trigger = true;
      merge_s = current_invasion_s;
      key_obs = *obs.second;
    }
  }
  if (!merge_trigger) {
    return;
  }

  double obs_invasion_t = key_obs.riskStBoundary().bottom_left_point().t();
  key_obs_v = key_obs.riskStBoundary().calcSTLowerBoundProjectedSpeed(obs_invasion_t,3.0);
  // ERT_PLOG_I << ">>>>>>>>>>>>>>>>>>>>>> merge_s = " << merge_s << " key_obs_v = " << key_obs_v ;
  std::vector<double> delta_v_table = {10.0, 8.0, 6.0, 5.0};
  std::vector<double> stagger_s_table = {-5.0, 0.0, 1.0, 3.0};
  double delta_v = math::TableLookUp1D(stagger_s_table, delta_v_table, merge_s) * KMH_MS;

  const double stagger_delta_s = 3.0;
  auto getSUpperBoundAtTimeT = [=](const double& t, const SpeedPlannerObstacle& key_obs) {
    double s_upper_bound = kPostiveInfinity;
    double s_lower = kPostiveInfinity, s_upper = kPostiveInfinity;
    if(key_obs.riskStBoundary().getBoundarySRange(t, &s_upper, &s_lower)){
      s_upper_bound = max(s_lower - 3.0, 0.0);
    }
    return s_upper_bound;
  };

  auto getEgoSAtTimeT = [=](const double& t, const double& time_to_max_dec) {
    if (t <= 0.0) {
      return 0.0;
    }
    double ego_s = 0.0;
    const double comfort_jerk =
        speed_ocp_qp_optimizer_config_
            .comfort_jerk_for_stagger_driving();  // parameter_.comfort_jerk_for_stagger_driving();
    const double comfort_max_decc =
        speed_ocp_qp_optimizer_config_
            .comfort_decc_for_stagger_driving();  // parameter_.comfort_decc_for_stagger_driving();
    if (t < time_to_max_dec) {
      ego_s = init_v_ * t + 0.5 * init_a_ * pow(t, 2) + (1 / 6.0) * comfort_jerk * pow(t, 3);
    } else {
      ego_s =
          init_v_ * time_to_max_dec + 0.5 * init_a_ * pow(time_to_max_dec, 2) +
          (1 / 6.0) * comfort_jerk * pow(time_to_max_dec, 3) +
          (init_v_ + init_a_ * time_to_max_dec + 0.5 * comfort_jerk * pow(time_to_max_dec, 2)) * (t - time_to_max_dec) +
          0.5 * comfort_max_decc * pow(t - time_to_max_dec, 2);
    }
    return ego_s;
  };

  const double comfort_max_decc =
      speed_ocp_qp_optimizer_config_
          .comfort_decc_for_stagger_driving(); 
  const double comfort_jerk =
      speed_ocp_qp_optimizer_config_
          .comfort_jerk_for_stagger_driving();  
  double stagger_init_a = init_a_;
  if (init_a_ < comfort_max_decc) {
    stagger_init_a = comfort_max_decc;
  }
  double time_to_max_dec = (comfort_max_decc - stagger_init_a) / comfort_jerk;
  double v_ref_at_max_dec =
      init_v_ + stagger_init_a * time_to_max_dec + 0.5 *  time_to_max_dec * time_to_max_dec;
  const double min_s_upper_bound = 10.0;

  const double epsilon = 1e-03;
  for (size_t i = 0UL; i < time_grid_.size(); i++) {
    double obs_s_upper_bound = getSUpperBoundAtTimeT(time_grid_[i], key_obs);
    double ego_s = getEgoSAtTimeT(time_grid_[i], time_to_max_dec);
    double s_upper_bound = max(ego_s , obs_s_upper_bound);
    // ERT_PLOG_I<<"  i = "<< time_grid_[i]<<"  s = "<< s_upper_bound<<"  obs_s_upper_bound = "<<obs_s_upper_bound<<
    // " ego_s = "<<ego_s <<" s_org = "<< getParameter("SSoftUpperBound", i) ;
    setParameter("SSoftUpperBound", i, min(s_upper_bound, getParameter("SSoftUpperBound", i)));
  }
}


// void SpeedModelParam::setBoundaryForStaggerDriving(
//     const PathGroup& path_group, const ObstacleSet& obstacle_map) {
  // // if (frame->decisionOutput().scenarioIdentificationRes().staggered_driving_result.staggered_ods.empty()) {
  // //   return;
  // // }
  // bool stagger_trigger = false;
  // double stagger_s = 300.0;
  // double key_obs_v = kMaxSpeedMS;
  // // const auto stagger_decision = frame->decisionOutput().scenarioIdentificationRes().staggered_driving_result;
  // SpeedPlannerObstacle key_obs;
  // std::string key_obs_id = "";
  // if (obstacle_map.find(key_obs_id) != obstacle_map.end()) {
  //   auto obs = obstacle_map.at(key_obs_id);
  //   double current_stagger_s = obs->lateralRiskInfos().empty() ? obs->perceptionSLBoundary().start_s()
  //                                                              : obs->lateralRiskInfos().front().s_lower;
  //   if (current_stagger_s < stagger_s) {
  //     stagger_trigger = true;
  //     stagger_s = current_stagger_s;
  //     key_obs = *obs;
  //   }
  // }

  // // for (size_t i = 0; i < stagger_decision.staggered_ods.size(); ++i) {
  // //   auto obs = findObstacle(std::get<0>(stagger_decision.staggered_ods.at(i)));
  // //   if (obs == nullptr) {
  // //     continue;
  // //   }
  // //   if (std::get<1>(stagger_decision.staggered_ods.at(i))) {
  // //     PERROR << "Wrong stagger direction";
  // //     continue;
  // //   }
  // //   double current_stagger_s = obs->lateralRiskInfos().empty() ? obs->perceptionSLBoundary().start_s()
  // //                                                              : obs->lateralRiskInfos().front().s_lower;
  // //   if (current_stagger_s < stagger_s) {
  // //     stagger_trigger = true;
  // //     stagger_s = current_stagger_s;
  // //     key_obs = *obs;
  // //   }
  // // }
  // if (!stagger_trigger) {
  //   return;
  // }
  // if (stagger_trigger) {
  //   if (key_obs.lateralRiskInfos().empty()) {
  //     int init_collision_segment_index = 0;
  //     double center_s = CalcSLCoordinatesToInfinitPath(path_group, 0.0, key_obs.perceptionBoundingBox().center(),
  //                                                      init_collision_segment_index)
  //                           .first;
  //     auto center_pt = path_group.extand_interval_path_.evaluate(center_s);
  //     double center_theta = center_pt.theta();
  //     double delta_theta = math::NormalizeAngle(key_obs.perceptionBoundingBox().heading() - center_theta);
  //     double proj_speed = (key_obs.perceptionBoundingBox().cos_heading() * std::cos(center_theta) +
  //                          key_obs.perceptionBoundingBox().sin_heading() * std::sin(center_theta)) *
  //                         key_obs.speed();
  //     key_obs_v = std::fmax(0.0, proj_speed);
  //   } else {
  //     key_obs_v = key_obs.lateralRiskInfos().front().longitudinal_speed;
  //   }
  // }
  // ERT_PLOG_I << ">>>>>>>>>>>>>>>>>>>>>> stagger_s = " << stagger_s << " key_obs_v = " << key_obs_v ;
  // std::vector<double> delta_v_table = {10.0, 8.0, 6.0, 5.0};
  // std::vector<double> stagger_s_table = {-5.0, 0.0, 1.0, 3.0};
  // double delta_v = math::TableLookUp1D(stagger_s_table, delta_v_table, stagger_s) * KMH_MS;

  // const double stagger_delta_s = speed_ocp_qp_optimizer_config_.stagger_delta_s();
  // auto getSUpperBoundAtTimeT = [=](const double& t, const SpeedPlannerObstacle& key_obs) {
  //  double s_upper_bound;
  //   if (key_obs.lateralRiskInfos().empty()) {
  //     auto obs_box = key_obs.perceptionBoundingBox();
  //     std::vector<math::Vec2d> obs_corners;
  //     obs_box.GetAllCorners(&obs_corners);
  //     double s_lower = -1000.0;
  //     int init_collision_segment_index = 0;
  //     for (size_t corner_index = 0; corner_index < obs_corners.size(); ++corner_index) {
  //       std::pair<double, double> result;
  //       if (corner_index == 0) {
  //         result =
  //             CalcSLCoordinatesToInfinitPath(path_group, 0.0, obs_corners[corner_index],
  //             init_collision_segment_index);
  //       } else {
  //         int current_collision_segment_index = 0;
  //         result = CalcSLCoordinatesToInfinitPath(path_group, 0.0, obs_corners[corner_index],
  //                                                 current_collision_segment_index);
  //       }
  //       double s = result.first;
  //       double l = result.second;
  //       double l_0_sign;
  //       if (corner_index == 0UL) {
  //         l_0_sign = (l >= kMathEpsilon) ? 1.0 : -1.0;
  //       }
  //       s_lower = fmin(s_lower, s);
  //     }
  //     s_lower = s_lower - vehicle_param_.front_edge_to_ego();
  //     s_upper_bound = s_lower - stagger_delta_s;
  //   } else {
  //     auto comp = [](const SpeedPlannerObstacle::LateralRiskInfo& risk, const double t) { return risk.t < t; };
  //     auto it = std::lower_bound(key_obs.lateralRiskInfos().begin(), key_obs.lateralRiskInfos().end(), t, comp);
  //     size_t index = std::distance(key_obs.lateralRiskInfos().begin(), it);
  //     if (index == 0) {
  //       s_upper_bound = key_obs.lateralRiskInfos().front().s_lower - stagger_delta_s;
  //     } else if (it == key_obs.lateralRiskInfos().end()) {
  //       s_upper_bound = key_obs.lateralRiskInfos().back().s_lower - stagger_delta_s;
  //     } else {
  //       s_upper_bound = math::lerp(key_obs.lateralRiskInfos().at(index - 1).s_lower - stagger_delta_s,
  //                                  key_obs.lateralRiskInfos().at(index - 1).t,
  //                                  key_obs.lateralRiskInfos().at(index).s_lower - stagger_delta_s,
  //                                  key_obs.lateralRiskInfos().at(index).t, t);
  //     }
  //   }
  //   return s_upper_bound;
  // };

  // const double min_target_v = speed_ocp_qp_optimizer_config_.min_stagger_driving_speed() * KMH_MS;
  // auto getEgoSAtTimeT = [=](const double& t, const double& time_to_max_dec) {
  //   if (t <= 0.0) {
  //     return 0.0;
  //   }
  //   double ego_s = 0.0;
  //   const double comfort_jerk =
  //       speed_ocp_qp_optimizer_config_
  //           .comfort_jerk_for_stagger_driving();  // parameter_.comfort_jerk_for_stagger_driving();
  //   const double comfort_max_decc =
  //       speed_ocp_qp_optimizer_config_
  //           .comfort_decc_for_stagger_driving();  // parameter_.comfort_decc_for_stagger_driving();
  //   if (t < time_to_max_dec) {
  //     ego_s = init_v_ * t + 0.5 * init_a_ * pow(t, 2) + (1 / 6.0) * comfort_jerk * pow(t, 3);
  //   } else {
  //     ego_s =
  //         init_v_ * time_to_max_dec + 0.5 * init_a_ * pow(time_to_max_dec, 2) +
  //         (1 / 6.0) * comfort_jerk * pow(time_to_max_dec, 3) +
  //         (init_v_ + init_a_ * time_to_max_dec + 0.5 * comfort_jerk * pow(time_to_max_dec, 2)) * (t -
  //         time_to_max_dec) + 0.5 * comfort_max_decc * pow(t - time_to_max_dec, 2);
  //   }
  //   return ego_s;
  // };

  // // 交错边界
  // const double comfort_max_decc =
  //     speed_ocp_qp_optimizer_config_
  //         .comfort_decc_for_stagger_driving();  // parameter_.comfort_decc_for_stagger_driving();
  // const double comfort_jerk =
  //     speed_ocp_qp_optimizer_config_
  //         .comfort_jerk_for_stagger_driving();  // parameter_.comfort_jerk_for_stagger_driving();
  // double stagger_init_a = init_a_;
  // if (init_a_ < comfort_max_decc) {
  //   stagger_init_a = comfort_max_decc;
  // }
  // double time_to_max_dec = (comfort_max_decc - stagger_init_a) / comfort_jerk;
  // double v_ref_at_max_dec =
  //     init_v_ + stagger_init_a * time_to_max_dec + 0.5 * comfort_jerk * time_to_max_dec * time_to_max_dec;
  // const double min_s_upper_bound = 10.0;

  // const double epsilon = 1e-03;
  // for (size_t i = 0UL; i < time_grid_.size(); i++) {
  //   if (i == 0) {
  //     setParameter("VSoftUpperBound", i, init_v_);
  //   } else {
  //     double v_ref;
  //     if (time_grid_[i] <= time_to_max_dec) {
  //       v_ref = init_v_ + stagger_init_a * time_grid_[i] + 0.5 * comfort_jerk * time_grid_[i] * time_grid_[i];
  //     } else {
  //       v_ref = v_ref_at_max_dec + comfort_max_decc * (time_grid_[i] - time_to_max_dec);
  //     }
  //     v_ref = fmax(v_ref, min_target_v);
  //     auto origin_v_soft_upper_bound = getParameter("VSoftUpperBound", i);
  //     setParameter("VSoftUpperBound", i, fmin(origin_v_soft_upper_bound, v_ref));

  //     ERT_PLOG_I << " >>>>>>>>>>>>  v_ref = " << v_ref << "  time_grid_[i] = " << time_grid_[i]
  //          << "   ego_s = " << getEgoSAtTimeT(time_grid_[i], time_to_max_dec)
  //          << "  obs_s = " << getSUpperBoundAtTimeT(time_grid_[i], key_obs) ;
  //     if (getEgoSAtTimeT(time_grid_[i], time_to_max_dec) < getSUpperBoundAtTimeT(time_grid_[i], key_obs) &&
  //         v_ref < 0.9 * key_obs_v && i + 1 < time_grid_.size()) {
  //       double last_v_soft_upper_bound = getParameter("VSoftUpperBound", i);
  //       for (size_t j = i + 1; j < time_grid_.size(); ++j) {
  //         setParameter("VSoftUpperBound", j, std::min(v_ref, getParameter("VSoftUpperBound", j)));
  //         double s_upper_bound = fmax(min_s_upper_bound, getSUpperBoundAtTimeT(time_grid_[j], key_obs));
  //         setParameter("SSoftUpperBound", j, std::min(s_upper_bound, getParameter("SSoftUpperBound", j)));
  //         setParameter("JWeight", j, 100000 * epsilon);
  //       }
  //       break;
  //     }
  //   }
  //   setParameter("JWeight", i, 100000 * epsilon);
  // }
// }

/**
 * @brief 超车场景动态参数调整（安全距离与路径权重优化）
 * @param[in] decision_result 决策结果数据
 * @param[in] obstacle_set 障碍物集合
 * @param[in] st_graph ST图数据
 *
 * @par 处理流程:
 * @startuml
start
partition 场景判断 {
  if (非变道场景?) then (是)
    :历史HWT参数衰减;
    :直接返回;
  else (否)
    :遍历时间网格;
  endif
}
partition 前车处理 {
  :识别前车障碍物;
  :基于决策结果更新HWT参数;
  :应用HWT参数滤波;
}
partition 后车处理 {
  :识别后车超车障碍;
  :计算安全超车距离;
  :更新路径软下界约束;
}
@enduml
 *
 * @par 关键参数调整:
 * | 参数                | 调整规则                          | 生效条件                |
 * |---------------------|----------------------------------|-------------------------|
 * | K                   | 前车HWT决策值滤波更新            | 前车存在且HWT>0.5       |
 * | SSoftLowerBound     | 后车超车安全距离补偿             | 后车存在且HWT>0         |
 * | SlackSLowerWeight   | 提升至10000 * ε                  | 后车超车场景            |
 * | SlackVUpperWeight  | 降低至100 * ε                    | 后车超车场景            |
 *
 * @note 功能特性:
 * - 前向HWT滤波：采用0.95历史系数进行平滑
 * - 后向超车补偿：根据障碍物速度动态计算s_soft_lower
 * - 路径权重协调：降低速度松弛权重，提升位置跟踪权重
 *
 * @warning 使用约束:
 * - 需与路径规划模块的变道决策同步
 * - 必须在processObstacles之后调用
 * - 滤波系数需与决策模块参数对齐
 */
void SpeedModelParam::specialCaseForOvertakeObstacle(const DecisionResult& decision_result,
                                                     const ObstacleSet& obstacle_set, const StGraph& st_graph) {
  double t = 0.0;
  constexpr double slack_slower_weight_for_overtake = 10000.0;
  constexpr double slack_v_upper_weight_for_overtake = 10000.0;
  constexpr double kEpsilon = 1E-3;
  bool has_overtake_obstacle = false;
  for (size_t t_index = 0; t_index < time_grid_.size(); ++t_index) {
    t = time_grid_[t_index];
    size_t idx = static_cast<size_t>(round(t / drivable_boundary_dt_));
    idx = fmin(idx, st_graph.st_drivable_boundaries().size() - 1UL);
    const auto& drivable_boundary = st_graph.st_drivable_boundaries()[idx];
    // update back obstacle soft lower bound
    std::string lower_obj_id = drivable_boundary.lower_obj_id;
    if (drivable_boundary.has_back_take_over_obj && obstacle_set.find(lower_obj_id) != obstacle_set.end()) {
      double decision_hwt = max(speed_ocp_qp_optimizer_config_.decision_thw_threshold(),
                                obstacle_set.find(lower_obj_id)->second->getObstacleDecisionHwt());
      double overtake_s_lower = st_graph.st_drivable_boundaries()[idx].s_lower_bound;
      overtake_s_lower = overtake_s_lower + max(speed_ocp_qp_optimizer_config_.min_overtake_safe_distance(),
                                                decision_hwt * obstacle_set.find(lower_obj_id)->second->speed());
      overtake_s_lower = fmax(getParameter("SSoftLowerBound", t_index), overtake_s_lower);
      setParameter("SSoftLowerBound", t_index, fmin(overtake_s_lower, getParameter("SSoftUpperBound", t_index) - 0.1));
      setParameter("SlackSLowerWeight", t_index, slack_slower_weight_for_overtake * kEpsilon);
      //防止离散采样导致的入侵时间误差，延长一个t_index
      if(t_index> 0 && getParameter("SSoftLowerBound", t_index -1) < 0.1){
        setParameter("SSoftLowerBound", t_index-1, fmin(overtake_s_lower, getParameter("SSoftUpperBound", t_index-1) - 0.1));
        setParameter("SlackSLowerWeight", t_index-1, slack_slower_weight_for_overtake * kEpsilon);
      }
      has_overtake_obstacle = true;
    }
  }
  if (has_overtake_obstacle) {
    for (size_t t_index = 0; t_index < time_grid_.size(); ++t_index) {
      setParameter("SlackVUpperWeight", t_index, slack_v_upper_weight_for_overtake * kEpsilon);
    }
  }
}

/**
 * @brief 处理局部路径风险约束（核心实现）
 *
 * @param[in] local_view 局部环境感知信息
 * @param[in,out] speed_result 速度规划结果（包含局部路径边界信息）
 *
 * @details 实现步骤：
 * 1. 获取局部路径的s软上界数据（local_path_s_soft_upper）
 * 2. 遍历时间网格，在指定时间范围内（local_path_check_time_horizon）：
 *    - 通过线性插值计算当前时间的s软上界
 *    - 应用安全距离补偿（local_path_boundary_safe_dis）
 *    - 更新SSoftUpperBound参数
 * 3. 处理最近障碍物：
 *    - 识别最近的有效障碍物信息
 *    - 在满足条件时全局收紧s软上界
 *
 * @par 核心处理逻辑:
 * - 输入：局部路径的s软上界信息（含障碍物ID和时间标记）
 * - 输出：调整后的s软上界约束（SSoftUpperBound）
 * - 安全机制：保持0.1m最小s裕度，应用安全距离补偿
 *
 * @startuml
 * start
 * :获取local_path_s_soft_upper;
 * if (边界数据为空) then (是)
 *   :直接返回;
 * else (否)
 *   :遍历时间网格（限制在check_time_horizon内）;
 *   :线性插值计算s_lower_bound;
 *   :应用安全距离补偿;
 *   :更新SSoftUpperBound;
 *   :识别最近有效障碍物;
 *   if (满足全局收紧条件) then (是)
 *     :全局调整SSoftUpperBound;
 *   endif
 * endif
 * stop
 * @enduml
 */
void SpeedModelParam::localPathRiskyConsideration(const LocalView& local_view,
                                                  std::shared_ptr<SpeedResult> speed_result) {
  auto local_path_s_soft_upper = speed_result->localPathLowerBoundary();
  if (local_path_s_soft_upper.empty()) {
    return;
  }
  for (int i = 0; i < time_grid_.size(); ++i) {
    if (time_grid_[i] > speed_planner_config_.local_path_check_time_horizon()) {
      break;
    }
    if (time_grid_[i] < local_path_s_soft_upper.front().first.t()) {
      continue;
    }
    if (time_grid_[i] > local_path_s_soft_upper.back().first.t()) {
      break;
    }

    double s_lower_bound = kPostiveInfinity;
    size_t left = 0;
    size_t right = 0;
    auto comp = [](const std::pair<STPoint, std::string>& p, const double t) { return p.first.t() < t; };
    auto first_ge =
        std::lower_bound(local_path_s_soft_upper.begin(), local_path_s_soft_upper.end(), time_grid_[i], comp);
    size_t index = std::distance(local_path_s_soft_upper.begin(), first_ge);
    if (index == 0) {
      left = 0;
      right = 0;
    } else if (first_ge == local_path_s_soft_upper.end()) {
      left = local_path_s_soft_upper.size() - 1;
      right = local_path_s_soft_upper.size() - 1;
    } else {
      left = index - 1;
      right = index;
    }
    if (left == right) {
      s_lower_bound = local_path_s_soft_upper[left].first.s();
    } else {
      s_lower_bound =
          math::lerp(local_path_s_soft_upper[left].first.s(), local_path_s_soft_upper[left].first.t(),
                     local_path_s_soft_upper[right].first.s(), local_path_s_soft_upper[right].first.t(), time_grid_[i]);
    }
    s_lower_bound = fmax(0.1, s_lower_bound - speed_planner_config_.local_path_boundary_safe_dis());
    if (s_lower_bound < getParameter("SSoftUpperBound", i)) {
      setParameter("SSoftUpperBound", i, s_lower_bound);
    }
  }

  STPoint last_valid_st_point = local_path_s_soft_upper.front().first;
  std::tuple<bool, std::string, double> closest_local_path_obs =
      std::make_tuple(false, "", kPostiveInfinity);  // flag, id, distance
  for (const auto& bound : local_path_s_soft_upper) {
    if (bound.second != "") {
      last_valid_st_point = bound.first;
      if (last_valid_st_point.s() < std::get<2>(closest_local_path_obs)) {
        std::get<1>(closest_local_path_obs) = bound.second;
        std::get<2>(closest_local_path_obs) = last_valid_st_point.s();
      }
    }
  }
  double last_valid_st_point_s =
      fmax(0.1, last_valid_st_point.s() - speed_planner_config_.local_path_boundary_safe_dis());
  if (last_valid_st_point.t() >= speed_planner_config_.local_path_check_time_horizon() - 0.6 &&
      last_valid_st_point_s <= speed_planner_config_.local_path_boundary_expand_s_threshold()) {
    std::get<0>(closest_local_path_obs) = true;
    for (int i = 0; i < time_grid_.size(); ++i) {
      if (last_valid_st_point_s < getParameter("SSoftUpperBound", i)) {
        setParameter("SSoftUpperBound", i, last_valid_st_point_s);
      }
    }
  }
}
/**
 * @brief 生成风险场速度限制（核心实现）
 *
 * @param[in] local_view 局部环境感知信息
 * @param[in] obstacle_map 障碍物集合
 * @param[in] path_group 路径组信息
 * @param[in,out] speed_result 速度规划结果
 *
 * @details 实现步骤：
 * 1. 初始化ST速度限制容器
 * 2. 遍历所有时间网格：
 *    - 生成障碍物风险场速度限制（generateObsSpeedLimitVecByTimeIndex）
 *    - 合并静态速度限制和风险场限制（updateSingleFrameSpeedLimit）
 *    - 构建连续的速度限制插值曲线
 * 3. 输出调试信息（可选）
 *
 * @par 核心处理逻辑:
 * - 输入：障碍物风险场、静态速度限制
 * - 输出：ST速度限制曲线（st_speed_limit_）
 * - 插值机制：使用线性插值生成连续速度限制曲线
 * - 安全机制：对静态障碍物应用最小减速度限制
 *
 * @startuml
 * start
 * :清空st_speed_limit_;
 * :遍历每个时间点i;
 *   :生成障碍物风险速度限制obs_speed_limit_set;
 *   :合并静态速度限制time_independent_speed_limit_set;
 *   :构建插值曲线speed_limit;
 *   :存储到st_speed_limit_;
 * end
 * stop
 * @enduml
 */
void SpeedModelParam::calcRiskFieldSpeedLimit(const LocalView& local_view, const ObstacleSet& obstacle_map,
                                              const PathGroup& path_group, std::shared_ptr<SpeedResult> speed_result) {
  st_speed_limit_.clear();
  const double s_interval = 1.0;

  std::vector<SpatialSpeedLimit> time_independent_speed_limit_set = speed_result->speedLimitResult().speed_limit_;
  for (int i = 0; i < time_grid_.size(); ++i) {
    // ERT_PLOG_I<<">>>>>>>>>>>>>>>>>>>>>>>> time_grid_[i] = "<<time_grid_[i];
    std::vector<SpatialSpeedLimit> res_speed_limit_set;
    auto vehicle_config_ = config_manager_->vehicle_config();
    std::vector<SpatialSpeedLimit> obs_speed_limit_set =
        generateObsSpeedLimitVecByTimeIndex(local_view, obstacle_map, path_group, speed_result, i, s_interval);
    if (!updateSingleFrameSpeedLimit(path_group, obs_speed_limit_set, res_speed_limit_set)) {
      ERT_PLOG_D << "update obs speedlimit failed!" ;
    }
    if (!updateSingleFrameSpeedLimit(path_group, time_independent_speed_limit_set, res_speed_limit_set)) {
      ERT_PLOG_D << "update time independent speedlimit failed!" ;
    }

    // updateSingleFrameSpeedLimit(curve_speed_limit_set, res_speed_limit_set);

    math::IntervalData<SpatialSpeedLimit> speed_limit(

        res_speed_limit_set.front().s, s_interval, res_speed_limit_set,
        [](const SpatialSpeedLimit& p0, const SpatialSpeedLimit& p1, const double x) {
          return SpatialSpeedLimit(x, p0.speed_limit + (p1.speed_limit - p0.speed_limit) * (x - p0.s) / (p1.s - p0.s),
                                   p0.id);
        });

    speed_limit.setLeftExtrapolationFunction(
        [](const SpatialSpeedLimit& p0, const double x) { return SpatialSpeedLimit(x, p0.speed_limit, p0.id); });
    speed_limit.setRightExtrapolationFunction(
        [](const SpatialSpeedLimit& p0, const double x) { return SpatialSpeedLimit(x, p0.speed_limit, p0.id); });
    speed_limit.setDifferentiationFunction(
        [](const SpatialSpeedLimit& p0, const SpatialSpeedLimit& p1, const double s) {
          return SpatialSpeedLimit(s, (p1.speed_limit - p0.speed_limit) / (p1.s - p0.s), p0.id);
        });

    // for(auto& sp: res_speed_limit_set){
    //   ERT_PLOG_I<<"   s = "<<sp.s<<"  speed_limit = "<<sp.speed_limit<<"  id = "<<sp.id;
    // }
    st_speed_limit_.emplace_back(speed_limit);
  }

  if (0) {
    // plot
    if (st_speed_limit_.size() > 0) {
      std::filesystem::path buffer = std::filesystem::path(__FILE__).parent_path();
      std::string direction = buffer;
      direction += "risk_speed_limit.csv";
      ERT_PLOG_I << "direction = " << direction ;
      std::ofstream test_file;
      test_file.open(direction, std::ios::out | std::ios::app);
      if (!test_file) {
        std::cerr << "Unable to open file: " << direction ;
        return;
      }

      int time_num = time_grid_.size();
      int point_num = 150;
      for (int i = 0; i < time_num; i++) {
        auto& speed_limit_info = st_speed_limit_[i];
        for (int j = 0; j < point_num; j++) {
          if (j < speed_limit_info.getOriginData().size()) {
            test_file << speed_limit_info.getOriginData().at(j).speed_limit << " ";
          } else {
            test_file << speed_limit_info.getOriginData().back().speed_limit << " ";
          }
        }
      }
      test_file << "\n";
      test_file.close();
    }
  }
}
/**
 * @brief 生成障碍物风险场速度限制（核心实现）
 *
 * @param[in] local_view 局部环境感知信息
 * @param[in] obstacle_map 障碍物集合
 * @param[in] path_group 路径组信息
 * @param[in] speed_result 速度规划结果
 * @param[in] time_index 时间网格索引
 * @param[in] interval 路径采样间隔（单位：m）
 * @return std::vector<SpatialSpeedLimit> 空间速度限制集合
 *
 * @details 实现步骤：
 * 1. 遍历障碍物集合，筛选有效风险场：
 *    - 仅处理车辆/行人/自行车等动态障碍物
 *    - 排除OVERTAKE类型的障碍物
 * 2. 获取指定时间索引的风险场信息
 * 3. 沿路径按间隔采样，计算每个位置的最小速度限制：
 *    - 评估所有障碍物的速度限制
 *    - 对静态障碍物应用最小减速度约束（max_risk_dacc = -0.5）
 *
 * @par 核心处理逻辑:
 * - 输入：障碍物风险场、路径信息
 * - 输出：空间离散化的速度限制曲线
 * - 安全机制：静态障碍物强制应用0.5m/s²减速度限制
 *
 * @startuml
 * start
 * :遍历障碍物集合;
 * if (障碍物类型有效 && 非超车类型) then (是)
 *   :获取time_index对应风险场信息;
 * else (否)
 *   :跳过该障碍物;
 * endif
 * :沿路径按interval采样;
 *   :计算每个s点的最小速度限制;
 *   :应用静态障碍物减速度约束;
 * :返回res_speed_limit_set;
 * stop
 * @enduml
 */
std::vector<SpatialSpeedLimit> SpeedModelParam::generateObsSpeedLimitVecByTimeIndex(
    const LocalView& local_view, const ObstacleSet& obstacle_map, const PathGroup& path_group,
    std::shared_ptr<SpeedResult> speed_result, int time_index, double interval) {
  std::vector<SpatialSpeedLimit> res_speed_limit_set;
  std::vector<math::IntervalData<SpatialSpeedLimit>> obs_speed_limit_set;
  // ERT_PLOG_I<<" >>>>>>>>>>>>>>>>>>>>>>>>>>>>>  time_index = "<<time_index<<"    t = "<<time_grid_[time_index] ;
  for (const auto& obstacle_item : obstacle_map) {
    const auto& obstacle = obstacle_item.second;
    // ERT_PLOG_I<<"%%%%%%%%%%%%%%%  obstacle id = "<<obstacle->id();
    if (!obstacle->hasRiskField()) {
      continue;
    }
    if (obstacle->type() != Decision::ObjectType::VEHICLE && obstacle->type() != Decision::ObjectType::HEAVY_VEHICLE
        && obstacle->type() != Decision::ObjectType::VRU && obstacle->type() != Decision::ObjectType::PEDESTRIAN) {
      continue;
    }
    if (obstacle->pathStBoundary().boundary_type() == STBoundary::BoundaryType::OVERTAKE) {
      continue;
    }

    auto risk_field_infos = obstacle->getRiskFieldInfos();
    if (time_index < risk_field_infos.size() &&
        !risk_field_infos.at(time_index).speed_limit_info.getOriginData().empty()) {
      obs_speed_limit_set.emplace_back(risk_field_infos.at(time_index).speed_limit_info);
    }
  }
  for (double s = 0.0; s <= path_group.origin_path_.back().s(); s += interval) {
    double v = kMaxSpeedMS;
    std::string id = "default";
    for (const auto& obs_speed_limit : obs_speed_limit_set) {
      auto spatial_speed_limit = obs_speed_limit.evaluate(s);
      // TODO
      double max_risk_dacc = -1.0;
      if (obstacle_map.find(spatial_speed_limit.id) != obstacle_map.end() &&
          obstacle_map.at(spatial_speed_limit.id)->isStatic()) {
        max_risk_dacc = -0.5;
      }
      double min_risk_speed_limit =
          sqrt(2 * max_risk_dacc * s + local_view.getChassisPtr()->Speed() * local_view.getChassisPtr()->Speed());
      spatial_speed_limit.speed_limit = fmax(spatial_speed_limit.speed_limit, min_risk_speed_limit);

      if (spatial_speed_limit.speed_limit <= v) {
        v = spatial_speed_limit.speed_limit;
        id = spatial_speed_limit.id;
      }
    }
    // ERT_PLOG_I<<"   s = "<<s<<"  v = "<<v<<"  id = "<<id;
    res_speed_limit_set.emplace_back(SpatialSpeedLimit(s, v, id));
  }
  return res_speed_limit_set;
}
/**
 * @brief 单帧速度限制合并（核心实现）
 *
 * @param[in] path_group 路径组信息
 * @param[in] input_speed_limit_vec 输入速度限制集合
 * @param[in,out] res_speed_limit_vec 结果速度限制集合（输出参数）
 * @return bool 合并成功(true)或失败(false)
 *
 * @details 实现步骤：
 * 1. 初始合并：当结果集为空时直接复制输入数据
 * 2. 安全性校验：输入与结果集尺寸必须一致
 * 3. 路径点级合并：保留更严格的速度限制值（取输入和结果中的较小值）
 *
 * @par 核心处理逻辑:
 * - 输入：待合并的速度限制集合
 * - 输出：更新后的结果集合
 * - 合并规则：取输入和现有结果中的较小速度限制值
 * - 安全机制：强制1米间隔采样保证空间一致性
 *
 * @startuml
 * start
 * if (结果集为空) then (是)
 *   :直接复制输入数据;
 * else (否)
 *   :校验输入与结果尺寸是否一致;
 *   if (尺寸不一致) then (是)
 *     :返回false;
 *   else (否)
 *     :遍历每个路径点;
 *     :比较输入和现有速度限制，保留较小值;
 *   endif
 * endif
 * :返回true;
 * stop
 * @enduml
 */
bool SpeedModelParam::updateSingleFrameSpeedLimit(const PathGroup& path_group,
                                                  const std::vector<SpatialSpeedLimit>& input_speed_limit_vec,
                                                  std::vector<SpatialSpeedLimit>& res_speed_limit_vec) {
  if (res_speed_limit_vec.empty() && !input_speed_limit_vec.empty()) {
    res_speed_limit_vec.assign(input_speed_limit_vec.begin(), input_speed_limit_vec.end());
    return true;
  }
  if (input_speed_limit_vec.size() != res_speed_limit_vec.size()) {
    return false;
  }
  double max_s = path_group.origin_path_.back().s();
  const double s_interval = 1.0;
  int index = 0;
  for (double s = 0.0; s <= max_s; s += s_interval) {
    if (input_speed_limit_vec[index].speed_limit <= res_speed_limit_vec[index].speed_limit) {
      res_speed_limit_vec[index] =
          SpatialSpeedLimit(s, input_speed_limit_vec[index].speed_limit, input_speed_limit_vec[index].id);
    }
    ++index;
  }
  return true;
}
/**
 * @brief 边界有效性保护（核心实现）
 *
 * @param[in] hard_upper 硬上界（单位：m）
 * @param[in] hard_lower 硬下界（单位：m）
 * @param[in,out] soft_upper 软上界（输入输出参数，单位：m）
 * @param[in,out] soft_lower 软下界（输入输出参数，单位：m）
 *
 * @details 实现步骤：
 * 1. 软边界约束调整：
 *    - 确保soft_upper位于[hard_lower + 2ε, hard_upper - 2ε]区间
 *    - 确保soft_lower位于[hard_lower + 2ε, hard_upper - 2ε]区间
 * 2. 交叉保护：
 *    - 若调整后soft_upper <= soft_lower，重置soft_lower = soft_upper - ε
 *
 * @par 核心保护机制:
 * - 安全裕度：使用kMathEpsilon(1e-6)防止数值误差
 * - 强制约束：保证 soft_lower < soft_upper < hard_upper
 * - 强制约束：保证 hard_lower < soft_lower < soft_upper
 *
 * @startuml
 * start
 * :调整soft_upper到硬边界范围内;
 * :调整soft_lower到硬边界范围内;
 * if (soft_upper <= soft_lower) then (是)
 *   :重置soft_lower = soft_upper - ε;
 * endif
 * stop
 * @enduml
 */
void SpeedModelParam::boundValidProtect(const double& hard_upper, const double& hard_lower, double& soft_upper,
                                        double& soft_lower) {
  soft_upper = fmin(fmax(hard_lower + 2.0 * kMathEpsilon, soft_upper), hard_upper - 2.0 * kMathEpsilon);
  soft_lower = fmin(fmax(hard_lower + 2.0 * kMathEpsilon, soft_lower), hard_upper - 2.0 * kMathEpsilon);
  if (soft_upper <= soft_lower) {
    soft_lower = soft_upper - kMathEpsilon;
  }
}

std::string SpeedModelParam::getSpeedWallIdThroughType(const SpeedWallType& type) const {
  auto it = speed_wall_id_map.find(type);
  if (it == speed_wall_id_map.end()) {
    return speed_wall_id_map.at(UNKNOWN_WALL);
  }
  return speed_wall_id_map.at(type);
}
/**
 * @brief 计算静态速度软上界
 * @param[in] path_v_t 路径速度时间序列
 * @param[in] t 目标时间（单位：s）
 * @return double 允许的最大速度（单位：m/s）
 * @details 通过查表获取t时刻的静态限速：
 *          1. 时间超出范围时取末值
 *          2. 时间在中间时线性插值
 * @note 包含曲率限速和地图限速的融合
 *
 * @startuml
 start
 if (t ≥ 最大时间) then (是)
   :取末速度;
 else if (t ≤ 0) then (是)
   :取初速度;
 else (否)
   :线性插值计算;
 endif
 stop
 @enduml
 */
double SpeedModelParam::getStaticVSoftBound(vector<proto::TrajectoryPoint> path, double t) {
  for (int i = 0; i < path.size() - 1; i++) {
    if (path.at(i).relative_time() <= t && path.at(i + 1).relative_time() > t) {
      return math::lerp(path.at(i).v(), path.at(i).relative_time(), path.at(i + 1).v(), path.at(i + 1).relative_time(),
                        t);
    }
  }
  return path.back().v();
}

std::shared_ptr<SpeedPlannerObstacle> SpeedModelParam::findObstacle(const std::string& id) {
  auto iter = obstacle_map_.find(id);
  if (iter != obstacle_map_.end()) {
    return iter->second;
  }
  return nullptr;
}

/**
 * @brief 获取ACC跟车时距
 * @param[in] distance_level 跟车距离等级（1-3级）
 * @return double 时距系数（单位：s）
 * @details 跟车等级映射：
 *          1 → 1.8s, 2 → 2.0s, 3 → 2.2s
 * @warning 非法等级默认返回2.0s
 *
 * @startuml
 |start|
 :创建等级-时距映射表;
 :查表返回对应时距;
 @enduml
 */
double SpeedModelParam::getAccThw(FollowingDistanceLevel following_distance_level) {
  switch (following_distance_level) {
    case FollowingDistanceLevel::Invalid:
      return speed_ocp_qp_optimizer_config_.thw_mid();
    case FollowingDistanceLevel::Min:
      return speed_ocp_qp_optimizer_config_.thw_min();
    case FollowingDistanceLevel::Low:
      return speed_ocp_qp_optimizer_config_.thw_low();
    case FollowingDistanceLevel::Mid:
      return speed_ocp_qp_optimizer_config_.thw_mid();
    case FollowingDistanceLevel::High:
      return speed_ocp_qp_optimizer_config_.thw_high();
    case FollowingDistanceLevel::Max:
      return speed_ocp_qp_optimizer_config_.thw_max();
    default:
      return speed_ocp_qp_optimizer_config_.thw_mid();
  }
}

}  // namespace gpal::pnc::planning