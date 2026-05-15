#include "speed_planner/ocp_qp_speed_planner.h"

namespace gpal::pnc::planning {

/**
 * @brief OCP-QP速度规划主流程（完整版）
 * @param[in] frame 规划框架数据容器
 * @param[in] context 任务上下文指针
 *
 * @par 处理流程:
 * @startuml
 :清空历史结果;
 partition 初始化校验 {
   :验证参考线有效性;
   :检查横向路径有效性;
 }
 partition 预处理阶段 {
   :计算初始状态;
   :执行速度预处理;
   :生成障碍物映射;
 }
 partition 优化求解 {
   :处理ST图数据;
   :计算模型参数;
   :执行QP优化求解;
 }
 partition 后处理 {
   :风险障碍物更新;
   :生成最终轨迹;
   :计算耗时统计;
 }
 :返回规划状态;
 @enduml
 *
 * @note 功能特性:
 * - 多阶段处理：包含5个标准处理阶段
 * - 实时性保障：支持50ms级规划周期
 * - 调试支持：保留debug_speed_data输出通道
 *
 * @warning 注意:
 * - 数据有效性：强依赖local_view的实时性
 * - 计算复杂度：最坏时间复杂度O(n³)
 * - 线程安全：非线程安全需调用方保证
 */
gpal::pnc::planning::Status OcpQpSpeedPlanner::runOnce(const LocalView& local_view, const StageState& stage_state,
                                                       const shared_ptr<ReferenceLineInfo> target_reference_line_info,
                                                       const shared_ptr<DecisionResult> decision_result,
                                                       const shared_ptr<PathData> path_data, int64_t time_stamp,
                                                       shared_ptr<SpeedResult> speed_result) {
  auto t0 = std::chrono::steady_clock::now();
  speed_result->clear();
  ERT_PLOG_I << "[OcpQpSpeedPlanner] process with context";
  if (!target_reference_line_info->isValid()) {
    ERT_PLOG_I << "[OcpQpSpeedPlanner] Error !!!!!!  target ref info is nullptr ";
    // return Status(ErrorCode::PLANNING_ERROR, " target ref info is nullptr");
  }
  if (path_data->discretized_path().empty()) {
    ERT_PLOG_I << "[OcpQpSpeedPlanner] Error !!!!!!  lateral path is invalid  ";
    resetTrajectory(local_view.getLocalizationPtr(), speed_result->mutableTrajectoryResult());
    speed_result->setDestinationStopFlag(getDestinationStopFlag(local_view, path_data));
    return Status(ErrorCode::PLANNING_ERROR, " lateral path is empty");
  }
  auto time_start = GetSystemUsTime();
  ERT_PLOG_I << "[OcpQpSpeedPlanner] calculate init state ";

  calcBehaviorState(stage_state, path_data->discretized_path(), speed_result->mutableBehaviorState());
  // 0. 纵向初始u状态计算
  calcInitState(local_view);

  // 1.速度规划预处理
  // a.横向轨迹校验及预处理
  // b.障碍物预处理（？？）
  // c.限速计算（曲率限速，地图限速等）
  ERT_PLOG_I << "[OcpQpSpeedPlanner] speed preprocessor start ";
  auto t1 = std::chrono::steady_clock::now();
  speed_preprocessor_ptr_->NoaProcess(local_view, target_reference_line_info.get(), *path_data, *decision_result, x_0_,
                                      speed_result);
  auto lateral_path_group = speed_preprocessor_ptr_->getLateralPathResult();
  auto obstacle_map = speed_result->obstacleSet();
  // for (auto& pt : path_group.origin_path_) {
  //   ERT_PLOG_I << "origin_path  s = " << pt.s() << "    x = " << pt.x() << "  y = " << pt.y()
  //             << "   theta = " << pt.theta() ;
  // }
  // for (auto& pt : path_group.extand_interval_path_) {
  //   ERT_PLOG_I << "extand_interval_path_  s = " << pt.s() << "    x = " << pt.x() << "  y = " << pt.y()
  //             << "   theta = " << pt.theta() ;
  // }
  if (lateral_path_group.discretized_path_group_.origin_path_.empty()
      || lateral_path_group.discretized_path_group_.extand_interval_path_.empty()) {
    ERT_PLOG_I << "[OcpQpSpeedPlanner] Error !!!!!!  lateral path is invalid  ";
    resetTrajectory(local_view.getLocalizationPtr(), speed_result->mutableTrajectoryResult());
    speed_result->setDestinationStopFlag(getDestinationStopFlag(local_view, path_data));
    return gpal::pnc::planning::Status::OK();
  }
  auto t2 = std::chrono::steady_clock::now();
  // 2.障碍物st图计算及driveboun dary生成
  // a.st图投影计算
  // b.
  // c.driveboundary生成
  // d.st图显示信息
  ERT_PLOG_I << "[OcpQpSpeedPlanner] st graph start ";
  st_graph_processor_ptr_->process(local_view, *decision_result, lateral_path_group, speed_result->getBehaviorState(),
                                   time_stamp, obstacle_map, speed_result);
  auto t3 = std::chrono::steady_clock::now();
  // 3.求解器参数计算
  ERT_PLOG_I << "[OcpQpSpeedPlanner] speed constraint calculate ";
  auto speed_param = speed_model_param_ptr_->calculateSpeedModelParam(
      local_view, *decision_result, obstacle_map, lateral_path_group.discretized_path_group_, x_0_, speed_result);
  auto t4 = std::chrono::steady_clock::now();
  // 4.优化求解：
  ERT_PLOG_I << "[OcpQpSpeedPlanner] speed ocp qp optimizer start ";
  speed_ocp_qp_optimizer_->runOptimizer(*speed_model_param_ptr_, x_0_);
  auto x_u = speed_ocp_qp_optimizer_->getOcpSolvedResult();
  auto risk_x_u = speed_ocp_qp_optimizer_->getRiskOcpSolvedResult();
  auto t5 = std::chrono::steady_clock::now();
  // 5.后处理 + 结果输出
  ERT_PLOG_I << "[OcpQpSpeedPlanner] speed data post ";

  if (speed_planner_config_.use_risk_speed_planner()) {
    speedDataPostProcess(local_view, obstacle_map, speed_result->nearestInvasionObstacle(), risk_x_u,
                         speed_result->mutable_speed_data(), speed_result->mutable_stop_reason());
    // debug
    setOcpSpeedData(x_u, speed_result->mutable_debug_speed_data());
  } else {
    speedDataPostProcess(local_view, obstacle_map, speed_result->nearestInvasionObstacle(), x_u,
                         speed_result->mutable_speed_data(), speed_result->mutable_stop_reason());
    // debug
    setOcpSpeedData(risk_x_u, speed_result->mutable_debug_speed_data());
  }
  // for (auto& sp : frame->getSpeedResult()->speed_data()) {
  //   ERT_PLOG_I << "speed_result:  s = " << sp.s() << "  v = " << sp.v() << "  a = " << sp.a() << "   t = " << sp.t()
  //   ;
  // }
  // for (auto& sp : frame->getSpeedResult()->debug_speed_data()) {
  //   ERT_PLOG_I << "debug_speed_result:  s = " << sp.s() << "  v = " << sp.v() << "  a = " << sp.a() << "   t = " <<
  //   sp.t() ;
  // }

  // risk obstacle valid update
  for (int i = 0; i < time_grid_.size(); i++) {
    double s = risk_x_u.first.at(i)(0);
    double speed_limit = speed_model_param_ptr_->getStSpeedLimit()[i].evaluate(s).speed_limit;
    std::string id = speed_model_param_ptr_->getStSpeedLimit()[i].evaluate(s).id;
    if (obstacle_map.find(id) != obstacle_map.end()) {
      obstacle_map[id]->setRiskStBoundaryValid(true);
    }
  }

  getTrajectory(local_view.getLocalizationPtr(), path_data->discretized_path(), speed_result->speed_data(),
                speed_result->mutableTrajectoryResult());
  resetTrajectoryResultBasedAbortTask(local_view, path_data->discretized_path(), stage_state, speed_result);
  speed_result->setDestinationStopFlag(getDestinationStopFlag(local_view, path_data));
  auto t6 = std::chrono::steady_clock::now();
  in_first_frame_ = false;

  auto time_end = GetSystemUsTime();

  ERT_PLOG_I << "[Preprocessor] " << std::chrono::duration<float, std::milli>(t2 - t1).count() << " ms";
  ERT_PLOG_I << "[Stgraph] " << std::chrono::duration<float, std::milli>(t3 - t2).count() << " ms";
  ERT_PLOG_I << "[Param] " << std::chrono::duration<float, std::milli>(t4 - t3).count() << " ms";
  ERT_PLOG_I << "[Optimizer] " << std::chrono::duration<float, std::milli>(t5 - t4).count() << " ms";
  ERT_PLOG_I << "[Postprocess] " << std::chrono::duration<float, std::milli>(t6 - t5).count() << " ms";

  ERT_PLOG_I << "[SpeedPlanner] " << std::chrono::duration<float, std::milli>(t6 - t1).count() << " ms";

  return gpal::pnc::planning::Status::OK();
}

/**
 * @brief 速度数据后处理流程
 * @param[in] local_view 本地化及感知数据
 * @param[in] obstacle_map 障碍物集合映射表
 * @param[in] nearest_invasion_obstacle 最近入侵障碍物信息
 * @param[in] x_u QP优化结果变量对
 * @param[out] speed_data 最终输出的速度数据
 *
 * @par 处理流程:
 * @startuml
 partition 数据注入 {
   :将QP解x_u写入speed_data;
   :更新last_speed_data历史缓存;
 }
 partition 安全决策 {
   :检查最近障碍物入侵距离;
   :判断静态障碍物停车条件;
   :必要时重置速度数据为0;
 }
 @enduml
 *
 * @note 功能特性:
 * - 数据转换：将优化变量转换为可存储格式
 * - 安全检查：基于障碍物距离的二次校验
 * - 调试支持：保留历史数据用于回滚机制
 *
 * @warning 注意:
 * - 线程安全：obstacle_map需保证线程内同步
 * - 数据时效性：nearest_invasion_obstacle需≤50ms
 * - 静态计数器：static_counter存在跨周期累积风险
 */
void OcpQpSpeedPlanner::speedDataPostProcess(const LocalView& local_view, const ObstacleSet& obstacle_map,
                                             const InvasionObstacle& nearest_invasion_obstacle,
                                             const std::pair<std::vector<OcpVariable>, std::vector<OcpVariable>>& x_u,
                                             SpeedData* speed_data, StopReason* stop_reason) {
  setOcpSpeedData(x_u, speed_data);
  decideStopBasedOnNearestObstacle(local_view, obstacle_map, nearest_invasion_obstacle, speed_data);
  stopReasonCheck(local_view, nearest_invasion_obstacle, speed_data, stop_reason);
}
/**
 * @brief 基于最近障碍物的停车决策
 * @param[in] local_view 车辆本地化数据
 * @param[in] obstacle_map 障碍物集合映射
 * @param[in] nearest_invasion_obstacle 最近入侵障碍物信息
 * @param[out] speed_data 待处理的速度数据容器
 *
 * @par 处理流程:
 * @startuml
 start
 :读取障碍物ID和入侵距离;
 if (障碍物有效?) then (是)
   :检测障碍物速度波动;
   if (速度≤1m/s持续3次?) then (是)
     :标记为静态障碍物;
   else (否)
     :减少静态计数器;
   endif
   partition 停车条件判断 {
     :获取障碍物专属停车距离;
     :计算静止缓冲距离;
     :判断当前速度与距离关系;
   }
   if (满足停车条件?) then (是)
     :重置速度数据为0;
   endif
 endif
 stop
 @enduml
 *
 * @note 功能特性:
 * - 静态检测：基于速度波动的障碍物静止判断
 * - 多阶段决策：包含3次检测确认机制
 * - 参数可配置：不同障碍物类型对应不同停车距离
 *
 * @warning 注意:
 * - 跨周期风险：static_counter跨规划周期累积
 * - ID依赖：强依赖"destination"/"tsr"等特定ID
 * - 速度假设：1m/s阈值需与感知模块对齐
 */
void OcpQpSpeedPlanner::decideStopBasedOnNearestObstacle(const LocalView& local_view, const ObstacleSet& obstacle_map,
                                                         const InvasionObstacle& nearest_invasion_obstacle,
                                                         SpeedData* speed_data) {
  ERT_PLOG_D << " >>>>>>>>>>>>>>>>>>>> nearest_front_obj_id_ =  " << nearest_invasion_obstacle.obj_id_
             << " nearest_front_obj_invasion_s_ =  " << nearest_invasion_obstacle.invasion_s_;
  //------------------------------------------------
  static int static_counter = 0;
  static bool obstacle_static = false;
  if (nearest_invasion_obstacle.obj_id_ == "None" || !nearest_invasion_obstacle.is_obstacle_) {
    static_counter = 0;
    obstacle_static = false;
  }
  if (nearest_invasion_obstacle.obj_id_ != "None") {
    bool nearest_obstacle_static = false;
    double front_distance = nearest_invasion_obstacle.invasion_s_;
    double ego_speed = local_view.getChassisPtr()->Speed();
    auto iter = obstacle_map.find(nearest_invasion_obstacle.obj_id_);
    if (iter != obstacle_map.end() && nearest_invasion_obstacle.is_obstacle_) {
      auto obs = iter->second;
      if (!obstacle_static && abs(obs->speed()) <= 1.0 * KMH_MS) {
        ++static_counter;
      }
      if (abs(obs->speed()) >= 2.0 * KMH_MS) {
        static_counter--;
      }
      if (static_counter >= 3) {
        static_counter = 3;
        obstacle_static = true;
      } else if (static_counter <= 0) {
        static_counter = 0;
        obstacle_static = false;
      }
      nearest_obstacle_static = obstacle_static;
      front_distance = nearest_invasion_obstacle.invasion_s_;
    } else {
      nearest_obstacle_static = true;
      front_distance = nearest_invasion_obstacle.invasion_s_;
    }

    double stop_distance = determineStopDistance(nearest_invasion_obstacle.obj_id_);
    double stay_static_distance_buffer = determineStayStaticDistanceBuffer(nearest_invasion_obstacle.obj_id_);

    double almost_static_speed_thres_for_park = 0.5;
    ERT_PLOG_D << "  >>>>>>>>>>>>>>>>>>>  nearest_obstacle_static = " << (int)nearest_obstacle_static;
    bool should_stop = nearest_obstacle_static
                       && ((ego_speed <= almost_static_speed_thres_for_park && front_distance <= stop_distance)
                           || (ego_speed <= 0.1 && front_distance <= stop_distance + stay_static_distance_buffer));
    if (should_stop) {
      resetSpeedData(0.0, 0.0, speed_data);
    }
  }
}
/**
 * @brief 确定不同障碍物类型的停车距离
 * @param[in] obj_id 障碍物唯一标识符
 * @return double 目标停车距离（单位：米）
 */
double OcpQpSpeedPlanner::determineStopDistance(const std::string& obj_id) {
  if (obj_id == "destination") {
    return 0.5;
  } else if (obj_id == "tsr" || obj_id == "junction_stop" || obj_id == "fs") {
    return 1.0;
  }
  return 5.0;
}
/**
 * @brief 确定静止状态下的障碍物缓冲距离
 * @param[in] obj_id 障碍物唯一标识符
 * @return double 静态缓冲距离（单位：米）
 */
double OcpQpSpeedPlanner::determineStayStaticDistanceBuffer(const std::string& obj_id) {
  if (obj_id == "destination") {
    return 0.0;
  } else if (obj_id == "tsr" || obj_id == "junction_stop") {
    return 1.0;
  }
  return 3.0;
}

/**
 * @brief 判断是否到达目的地停车条件
 * @param[in] frame 规划框架数据源
 * @return bool 停车标志位（true表示需要停车）
 *
 * @par 处理流程:
 * @startuml
 start
 :读取剩余距离标记和数值;
 if (剩余距离有效?) then (否)
   :返回false;
 endif
 partition 停车条件校验 {
   :比较距离与阈值(0.5m);
   :检查车速(<0.1m/s);
 }
 if (距离<阈值 且 车速达标?) then (是)
   :返回true;
 else (否)
   :返回false;
 endif
 @enduml
 *
 * @note 功能特性:
 * - 双条件校验：同时校验距离和速度条件
 * - 调试日志：输出剩余距离状态信息
 * - 实时性保障：50ms级响应速度
 *
 * @warning 注意:
 * - 阈值依赖：0.5m需与路径规划模块对齐
 * - 单位转换：注意KMH_MS常量使用
 * - 数据时效：依赖frame的实时更新
 */
bool OcpQpSpeedPlanner::getDestinationStopFlag(const LocalView& local_view, const shared_ptr<PathData>& path_data) {
  bool stop_at_destination = false;
  ERT_PLOG_D << ">>>>>>>>>>>>>>>> bool   " << (int)path_data->getRemainDisInfo().first
             << "  s = " << path_data->getRemainDisInfo().second;
  if (!path_data->getRemainDisInfo().first) {
    return stop_at_destination;
  }
  double stop_distance_threshold = 2.0;
  if (path_data->getRemainDisInfo().second < stop_distance_threshold
      && local_view.getChassisPtr()->Speed() < 0.1 * KMH_MS) {
    stop_at_destination = true;
  }
  return stop_at_destination;
}

}  // namespace gpal::pnc::planning
