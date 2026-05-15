#include "speed_planner/speed_planner_base.h"

namespace gpal::pnc::planning {

/*
* @brief OCP-QP速度规划器初始化
*
* @par 初始化流程:
* @startuml
:从配置管理器获取参数;
partition 处理器创建 {
 :创建速度预处理器;
 :创建ST图处理器;
 :创建模型参数处理器;
 :创建QP优化器;
}
:设置初始化标志位;
@enduml
*
* @note 功能特性:
* - 单例配置管理：通过 Singleton 获取全局配置
* - 模块化设计：各处理模块独立初始化
* - 延迟初始化：实际资源在首次process调用时创建
*
* @warning 实现细节:
* - 线程安全：配置管理器非线程安全需同步访问0000000000
* - 配置依赖：强依赖SpeedPlannerConfig配置项
* - 初始化状态：必须检查init_标志位后再执行规划
*/
bool SpeedPlannerBase::init() {
  config_manager_ = Singleton<ConfigManager>::get_instance();
  CHECK_NOTNULL(config_manager_);
  vehicle_param_ = config_manager_->vehicle_config().vehicle_param();
  speed_planner_config_ = config_manager_->getConfig<SpeedPlannerConfig>("SpeedPlannerConfig");
  speed_preprocessor_ptr_ = std::make_shared<SpeedPreprocessor>(time_grid_, time_resolution_, time_horizon_);
  st_graph_processor_ptr_ = std::make_shared<STGraphProcessor>(time_grid_, time_resolution_, time_horizon_);
  speed_model_param_ptr_ = std::make_shared<SpeedModelParam>(time_grid_, time_resolution_, time_horizon_);
  speed_ocp_qp_optimizer_ = std::make_shared<SpeedOCPQPOptimizer>(time_grid_, time_resolution_, time_horizon_);

  init_ = true;
  return true;
};

/**
 * @brief 计算行为状态标志位（与决策系统同步）
 * @param[in] frame 规划框架数据源
 * @param[out] behavior_state 行为状态输出容器
 *
 * @par 处理流程:
 * @startuml
 :读取阶段状态;
 if (HPA驾驶阶段?) then (是)
   :设置寻库标志=true;
 else (否)
   :设置寻库标志=false;
 endif

 if (泊入阶段?) then (是)
   :设置泊入标志=true;
 else (否)
   :设置泊入标志=false;
 endif

 if (路径方向==倒车?) then (是)
   :设置R挡标志=true;
 else (否)
   :设置R挡标志=false;
 endif
 @enduml
 *
 * @note 功能特性:
 * - 实时更新：每个规划周期（50ms）调用
 * - 状态同步：与决策系统StageState强关联
 * - 扩展预留：支持新增驾驶场景类型
 *
 * @warning 注意:
 * - 线程安全：frame参数需保证线程内同步
 * - 状态延迟：反映的是上一周期决策结果
 * - 枚举依赖：强依赖StageState枚举定义
 */
void SpeedPlannerBase::calcBehaviorState(const StageState& stage_state,const DiscretizedPath& discretized_path, BehaviorState* behavior_state) {
  behavior_state->clear();
  // 寻库
  if (stage_state == StageState::HpaDrivingStage) {
    behavior_state->search_parklot_state_ = true;
  }
  // 泊车
  if (stage_state == StageState::ParkInStage || stage_state == StageState::ParkOutStage) {
    behavior_state->park_in_state_ = true;
  }
  // 倒车
  if (discretized_path.front().direction() == PathPt::Direction::BACKWARD) {
    behavior_state->is_r_gear_ = true;
  }
  // lcc
  if (stage_state == StageState::LccDrivingStage) {
    behavior_state->is_lcc_state_ = true;
  }

  // acc
  if (stage_state == StageState::AccDrivingStage) {
    behavior_state->is_acc_state_ = true;
  }
}
/**
 * @brief 计算纵向运动初始状态（位置/速度/加速度）
 * @param[in] local_view 局部环境感知数据
 *
 * @par 处理流程:
 * @startuml
 if (首次调用?) then (是)
   :使用底盘速度初始化;
 else (否)
   partition 时间处理 {
     :计算时间间隔dt;
     :限制dt在[0.08s,0.15s];
     :计算最近历史索引;
   }
   partition 状态插值 {
     :基于jerk进行二次多项式插值;
     :计算s/v/a初始值;
   }
   partition 误差校正 {
     :查表计算允许速度误差;
     :根据加速度方向调整误差带;
   }
 endif
 :强制s=0.0（调试模式）;
 :限制加速度在配置范围内;
 @enduml
 *
 * @note 功能特性:
 * - 实时递推：基于历史数据外推当前状态
 * - 容错机制：自动处理过大位置偏差
 * - 状态连续：保证规划周期间状态平滑
 *
 * @warning 注意:
 * - 时间同步风险：依赖localization时间戳精确性
 * - 误差表依赖：speed_m_s_allow_error_table需现场标定
 * - 调试代码：强制s=0.0仅用于调试需后续移除
 */
void SpeedPlannerBase::calcInitState(const LocalView& local_view) {
  // TODO move to tas
  if (in_first_frame_
      || (local_view.getChassisPtr()->drivingMode()
              != proto::PowertrainInfo::DrivingMode::PowertrainInfo_DrivingMode_kCompleteAutoDrive
          && local_view.getChassisPtr()->drivingMode()
                 != proto::PowertrainInfo::DrivingMode::PowertrainInfo_DrivingMode_kAutoSpeedOnly)) {
    x_0_.s = 0.0;
    x_0_.v = local_view.getChassisPtr()->Speed();
    x_0_.a = local_view.getChassisPtr()->LogituAcc();
  } else {
    double dt = (local_view.getLocalizationPtr()->TimeStamp() - last_time_stamp_) * 1e-09;
    // Must be added
    dt = fmax(fmin(dt, 0.15), 0.08);
    size_t l_idx = std::round(dt / time_resolution_);
    size_t size = last_speed_data_.size();
    // To Do: Sanity check of vectors
    l_idx = fmin(l_idx, size - 1UL);
    double s_0 = last_speed_data_[l_idx].s();
    double v_0 = last_speed_data_[l_idx].v();
    double a_0 = last_speed_data_[l_idx].a();
    double t_0 = last_speed_data_[l_idx].t();
    double jerk_0 = (last_speed_data_[l_idx + 1UL].a() - last_speed_data_[l_idx].a()) / time_resolution_;
    double t = (dt - t_0);
    double t_sq = t * t;
    double t_cu = t_sq * t;

    x_0_.s = s_0
             - last_pos_.DistanceTo(math::Vec2d(local_view.getLocalizationPtr()->vehicleAlignPosePoint().x(),
                                                local_view.getLocalizationPtr()->vehicleAlignPosePoint().y()));
    // init_s_ = 0.0;
    if (abs(x_0_.s) > fmin(s_0, 0.15 * kMaxSpeedMS)) {
      x_0_.s = 0.0;
    }
    x_0_.v = v_0 + a_0 * t + 0.5 * jerk_0 * t_sq;
    x_0_.a = a_0 + jerk_0 * t;
    double ego_speed = local_view.getChassisPtr()->Speed();
    std::vector<double> speed_m_s_allow_error_table = {0.0, 0.139, 0.139, 0.555};
    std::vector<double> speed_m_s_allow_lower_error_table = {0.139, 0.139, 0.139, 0.555};
    std::vector<double> ego_speed_km_h_table_for_allow_error = {3.0, 5.0, 10.0, 40.0};
    double allow_error =
        math::TableLookUp1D(ego_speed_km_h_table_for_allow_error, speed_m_s_allow_error_table, ego_speed * MS_KMH);
    x_0_.v = fmin(fmax(x_0_.v, ego_speed - allow_error), ego_speed + allow_error);
    if (x_0_.a < kMathEpsilon) {  // 减速时缩小允许的误差
      double old_init_v = x_0_.v;
      double tolerance_for_decelerate = 0.0;
      std::vector<double> decelerate_table = {-0.3, -0.2, -0.1, 0.0};
      std::vector<double> tolerance_table = {0.0, 0.2 * allow_error, 0.8 * allow_error, allow_error};
      tolerance_for_decelerate = math::TableLookUp1D(decelerate_table, tolerance_table, x_0_.a);
      x_0_.v = fmin(x_0_.v, ego_speed + tolerance_for_decelerate);
      //   SFIELD_DEBUG(qp_optimizer, "init_a = {}, allow error = {}, old init_v = {}, real speed = {}, new init_v =
      // {}",
      //                init_a_, allow_error, old_init_v, ego_speed, init_v_);
    }
    double allow_lower_error = math::TableLookUp1D(ego_speed_km_h_table_for_allow_error,
                                                   speed_m_s_allow_lower_error_table, ego_speed * MS_KMH);
    x_0_.v = fmin(fmax(x_0_.v, ego_speed - allow_lower_error), ego_speed + allow_error);
  }
  // To Do: calc init_s
  x_0_.s = 0.0;  // Just for debug
  x_0_.v = fmax(0.0, x_0_.v);
  x_0_.a = fmin(fmax(x_0_.a, speed_planner_config_.a_hard_lower_bound()), speed_planner_config_.a_hard_upper_bound());
  last_pos_.set_x(local_view.getLocalizationPtr()->vehicleAlignPosePoint().x());
  last_pos_.set_y(local_view.getLocalizationPtr()->vehicleAlignPosePoint().y());
  last_time_stamp_ = local_view.getLocalizationPtr()->TimeStamp();
}

/**
 * @brief 将QP优化结果写入速度数据结构
 * @param[in] x_u 优化变量对（状态序列+控制序列）
 * @param[out] mutable_speed_data 可修改的速度数据容器
 *
 * @par 处理流程:
 * @startuml
 start
 partition 数据校验 {
   :检查x_u有效性;
   :调整容器大小;
 }
 partition 数据注入 {
   :遍历时间网格;
   :计算插值点状态;
   :更新当前速度数据;
   :更新历史缓存数据;
 }
 partition 终值处理 {
   :处理最后一个时间节点;
 }
 @enduml
 *
 * @note 功能特性:
 * - 数据结构转换：将优化变量转换为轨迹点
 * - 历史数据缓存：维护last_speed_data_用于状态递推
 * - 调试支持：保留完整的时间序列数据
 *
 * @warning 注意:
 * - 线程安全：非线程安全需调用方同步
 * - 时间同步：依赖time_grid_与time_resolution_匹配
 * - 输入验证：需前置检查x_u.empty()情况
 */
void SpeedPlannerBase::setOcpSpeedData(const std::pair<std::vector<OcpVariable>, std::vector<OcpVariable>>& x_u,
                                        SpeedData* mutable_speed_data) {
  if (x_u.first.empty() || x_u.second.empty()) {
    ERT_PLOG_I << "[OcpQpSpeedPlanner]: x_u is empty!";
    return;
  }
  size_t speed_data_size_ = std::round(time_horizon_ / time_resolution_) + 1UL;
  if (mutable_speed_data->size() != speed_data_size_) {
    mutable_speed_data->resize(speed_data_size_);
  }
  if (last_speed_data_.size() != speed_data_size_) {
    last_speed_data_.resize(speed_data_size_);
  }
  double t = 0.0;
  double s = 0.0;
  double v = 0.0;
  double a = 0.0;
  size_t idx = 0;
  size_t x_size = time_grid_.size();
  for (size_t i = 0; i < x_size - 1UL; ++i) {
    double s0 = x_u.first[i](0);
    double v0 = x_u.first[i](1);
    double a0 = x_u.first[i](2);
    double j0 = x_u.second[i](0);

    double dt = 0.0;
    double dt_sq;
    double dt_cu;
    while (dt + time_grid_[i] < time_grid_[i + 1UL]) {
      t = dt + time_grid_[i];
      idx = std::round(t / time_resolution_);
      if (idx >= mutable_speed_data->size()) {
        break;
      }
      dt_sq = dt * dt;
      dt_cu = dt_sq * dt;
      s = s0 + v0 * dt + 0.5 * a0 * dt_sq + 0.16666666666 * j0 * dt_cu;
      v = v0 + a0 * dt + 0.5 * j0 * dt_sq;
      a = a0 + j0 * dt;
      mutable_speed_data->at(idx).set_t(t);
      mutable_speed_data->at(idx).set_s(s);
      mutable_speed_data->at(idx).set_v(v);
      mutable_speed_data->at(idx).set_a(a);
      mutable_speed_data->at(idx).set_da(j0);
      last_speed_data_.at(idx).set_t(t);
      last_speed_data_.at(idx).set_s(s);
      last_speed_data_.at(idx).set_v(v);
      last_speed_data_.at(idx).set_a(a);
      last_speed_data_.at(idx).set_da(j0);
      dt += time_resolution_;
    }
  }
  // For the last one
  if (time_grid_[x_size - 1UL] <= time_horizon_) {
    t = time_grid_[x_size - 1UL];
    idx = static_cast<size_t>(t / time_resolution_);
    s = x_u.first.back()(0);
    v = x_u.first.back()(1);
    a = x_u.first.back()(2);
    mutable_speed_data->at(idx).set_t(t);
    mutable_speed_data->at(idx).set_s(s);
    mutable_speed_data->at(idx).set_v(v);
    mutable_speed_data->at(idx).set_a(a);
    last_speed_data_.at(idx).set_t(t);
    last_speed_data_.at(idx).set_s(s);
    last_speed_data_.at(idx).set_v(v);
    last_speed_data_.at(idx).set_a(a);
  }
}
/**
 * @brief 重置速度数据到指定状态
 * @param[in] desire_acc 目标加速度（单位：m/s²）
 * @param[in] desire_speed 目标速度（单位：m/s）
 * @param[out] mutable_speed_data 待重置的速度数据容器
 *
 * @par 处理流程:
 * @startuml
 start
 partition 容器调整 {
   :计算所需数据长度;
   :调整当前速度数据容器大小;
   :调整历史缓存数据容器大小;
 }
 partition 数据注入 {
   :遍历每个时间节点;
   :设置t/s/v/a到当前数据;
   :清空历史缓存数据;
 }
 @enduml
 *
 * @note 功能特性:
 * - 强制重置：忽略优化结果强制指定速度
 * - 数据初始化：用于紧急停车或系统复位
 * - 历史维护：清空last_speed_data_缓存
 *
 * @warning 注意:
 * - 线程安全：非线程安全需调用方同步
 * - 参数有效性：需保证desire_acc在物理可行范围
 * - 历史覆盖：会清除历史缓存影响后续规划
 */
void SpeedPlannerBase::resetSpeedData(const double& desire_acc, const double& desire_speed,
                                       SpeedData* mutable_speed_data) {
  size_t speed_data_size_ = std::round(time_horizon_ / time_resolution_) + 1UL;
  if (mutable_speed_data->size() != speed_data_size_) {
    mutable_speed_data->resize(speed_data_size_);
  }
  if (last_speed_data_.size() != speed_data_size_) {
    last_speed_data_.resize(speed_data_size_);
  }
  for (size_t i = 0UL; i < speed_data_size_; i++) {
    double t = static_cast<double>(i) * time_resolution_;
    mutable_speed_data->at(i).set_t(t);
    mutable_speed_data->at(i).set_s(0.0);
    mutable_speed_data->at(i).set_v(desire_speed);
    mutable_speed_data->at(i).set_a(desire_acc);
    last_speed_data_[i].set_t(t);
    last_speed_data_[i].set_s(0.0);
    last_speed_data_[i].set_v(0.0);
    last_speed_data_[i].set_a(0.0);
  }
}
/**
 * @brief 将速度数据转换为轨迹数据
 * @param[in] localization 本地化定位数据
 * @param[in] discretized_path 离散化路径数据
 * @param[in] speed_data 速度规划结果
 * @param[out] trajectory_result 输出的轨迹数据容器
 *
 * @par 处理流程:
 * @startuml
 start
 :清空轨迹容器;
 partition 轨迹生成 {
   :遍历速度数据节点;
   if (离散路径为空?) then (是)
     :使用定位数据填充轨迹点;
   else (否)
     :基于s值查询路径点;
   endif
   :设置速度/加速度/时间戳;
 }
 @enduml
 *
 * @note 功能特性:
 * - 数据转换：将速度域数据映射到空间域
 * - 路径适配：自动处理无路径时的定位回退
 * - 实时性保障：50ms级数据转换效率
 *
 * @warning 注意:
 * - 线程安全：非线程安全需调用方同步
 * - 路径匹配：依赖discretized_path与speed_data.s的同步
 * - 定位依赖：无路径时强依赖localization数据有效性
 */
void SpeedPlannerBase::getTrajectory(const std::shared_ptr<Localization>& localization,
                                      const DiscretizedPath& discretized_path, const SpeedData& speed_data,
                                      proto::Trajectory* trajectory_result) {
  double acculumate_s = 0.0;
  const double min_delta_s = 0.1;
  trajectory_result->Clear();

  for (size_t i = 0; i < speed_data.size(); ++i) {
    proto::TrajectoryPoint* tmp_trajectory_point = trajectory_result->add_trajectory_point();
    tmp_trajectory_point->set_v(speed_data[i].v());
    tmp_trajectory_point->set_a(speed_data[i].a());
    tmp_trajectory_point->set_relative_time(static_cast<double>(i) * time_resolution_);
    if (discretized_path.empty()) {
      tmp_trajectory_point->mutable_path_point()->set_x(localization->vehicleAlignPosePoint().x());
      tmp_trajectory_point->mutable_path_point()->set_y(localization->vehicleAlignPosePoint().y());
      tmp_trajectory_point->mutable_path_point()->set_z(localization->vehicleAlignPosePoint().z());
      tmp_trajectory_point->mutable_path_point()->set_theta(localization->vehicleAlignPosePoint().yaw());
      tmp_trajectory_point->mutable_path_point()->set_s(0.0);
    } else {
      PathPt::toPathPoint(discretized_path.evaluate(speed_data[i].s()), tmp_trajectory_point->mutable_path_point());
    }
  }
}
/**
 * @brief 重置轨迹数据到初始状态
 * @param[in] frame 规划框架数据源
 * @param[out] trajectory_result 待重置的轨迹数据容器
 *
 * @par 处理流程:
 * @startuml
 start
 :清空轨迹容器;
 partition 轨迹生成 {
   :遍历时间网格节点;
   :设置速度为0;
   :设置加速度为0;
   :填充定位位置信息;
 }
 @enduml
 *
 * @note 功能特性:
 * - 数据清空：确保容器初始状态
 * - 初始状态：速度/加速度归零
 * - 实时性保障：基于最新定位数据生成
 *
 * @warning 注意:
 * - 线程安全：非线程安全需调用方同步
 * - 定位依赖：强依赖frame中的localization数据
 * - 计算效率：时间复杂度O(n)需控制网格密度
 */
void SpeedPlannerBase::resetTrajectory(const std::shared_ptr<Localization>& localization,
                                       proto::Trajectory* trajectory_result) {
  trajectory_result->Clear();
  size_t speed_data_size_ = std::round(time_horizon_ / time_resolution_) + 1UL;
  for (size_t i = 0; i < speed_data_size_; ++i) {
    proto::TrajectoryPoint* tmp_trajectory_point = trajectory_result->add_trajectory_point();
    tmp_trajectory_point->set_v(0.0);
    tmp_trajectory_point->set_a(0.0);
    tmp_trajectory_point->set_relative_time(static_cast<double>(i) * time_resolution_);
    tmp_trajectory_point->mutable_path_point()->set_x(
        localization->vehicleAlignPosePoint().x());
    tmp_trajectory_point->mutable_path_point()->set_y(
        localization->vehicleAlignPosePoint().y());
    tmp_trajectory_point->mutable_path_point()->set_z(
        localization->vehicleAlignPosePoint().z());
    tmp_trajectory_point->mutable_path_point()->set_theta(
        localization->vehicleAlignPosePoint().yaw());
    tmp_trajectory_point->mutable_path_point()->set_s(0.0);
  }
}


/**
 * @brief 判断停车原因并更新停止理由
 * @param[in] local_view 本地化及感知数据
 * @param[in] nearest_invasion_obstacle 最近入侵障碍物信息
 * @param[out] stop_reason 停止理由输出容器
 *
 * @par 处理流程:
 * @startuml
 start
 if (车速>0.1m/s?) then (是)
   :标记为驾驶状态;
 else (否)
   if (非自动驾驶模式?) then (是)
     :标记为手动模式;
   else (否)
     :障碍物判断 
   endif
 endif
 :更新停止理由;
 @enduml
 *
 * @note 功能特性:
 * - 多条件判断：包含5种主要停车原因类型
 * - 模块化设计：不同障碍物类型独立处理
 * - 实时性保障：50ms级响应速度
 *
 * @warning 注意:
 * - 线程安全：非线程安全需调用方同步
 * - 数据时效：依赖local_view的实时更新
 * - ID依赖：强依赖speed_wall_id_map预定义映射
 */
void SpeedPlannerBase::stopReasonCheck(const LocalView& local_view, const InvasionObstacle& nearest_invasion_obstacle,
                                       SpeedData* speed_data, StopReason* stop_reason) {
  if (abs(local_view.getChassisPtr()->Speed()) > 0.1 * KMH_MS || (!speed_data->empty() && speed_data->back().s() > 0.5)) {
    stop_reason->updateStopReason("driving", StopReason::StopReasonType::DRIVING);
    return;
  }
  // if (local_view.getChassisPtr()->drivingMode()
  //         != proto::PowertrainInfo::DrivingMode::PowertrainInfo_DrivingMode_kCompleteAutoDrive
  //     && local_view.getChassisPtr()->drivingMode()
  //            != proto::PowertrainInfo::DrivingMode::PowertrainInfo_DrivingMode_kAutoSpeedOnly) {
  //   stop_reason->updateStopReason("manual", StopReason::StopReasonType::MANUAL);
  //   return;
  // }
  if (stop_reason->stop_reason_type == StopReason::StopReasonType::INVALID && nearest_invasion_obstacle.obj_id_ != "") {
    if (nearest_invasion_obstacle.is_obstacle_) {
      stop_reason->updateStopReason(nearest_invasion_obstacle.obj_id_, StopReason::StopReasonType::BLOCK_OD,
                                    nearest_invasion_obstacle.invasion_s_);
    } else if (nearest_invasion_obstacle.obj_id_ == speed_wall_id_map.at(FREESPACE_WALL)) {
      stop_reason->updateStopReason(nearest_invasion_obstacle.obj_id_, StopReason::StopReasonType::BLOCK_FS,
                                    nearest_invasion_obstacle.invasion_s_);
    } else if (nearest_invasion_obstacle.obj_id_ == speed_wall_id_map.at(RSA_WALL)) {
      stop_reason->updateStopReason(nearest_invasion_obstacle.obj_id_, StopReason::StopReasonType::GATE,
                                    nearest_invasion_obstacle.invasion_s_);
    } else if (nearest_invasion_obstacle.obj_id_ == speed_wall_id_map.at(TFL_WALL)) {
      stop_reason->updateStopReason(nearest_invasion_obstacle.obj_id_, StopReason::StopReasonType::STOP_LINE,
                                    nearest_invasion_obstacle.invasion_s_);
    } else if (nearest_invasion_obstacle.obj_id_ == speed_wall_id_map.at(DESTINATION_WALL)) {
      stop_reason->updateStopReason(nearest_invasion_obstacle.obj_id_, StopReason::StopReasonType::DESTINATION,
                                    nearest_invasion_obstacle.invasion_s_);
    } else if (nearest_invasion_obstacle.obj_id_ == speed_wall_id_map.at(JUNCTION_STOP)) {
      stop_reason->updateStopReason(nearest_invasion_obstacle.obj_id_, StopReason::StopReasonType::STOP_LINE,
                                    nearest_invasion_obstacle.invasion_s_);
    } else if (nearest_invasion_obstacle.obj_id_ == speed_wall_id_map.at(VIRTUAL_LINE)) {
      stop_reason->updateStopReason(nearest_invasion_obstacle.obj_id_, StopReason::StopReasonType::STOP_LINE,
                                    nearest_invasion_obstacle.invasion_s_);
    } else if (nearest_invasion_obstacle.obj_id_ == speed_wall_id_map.at(CLEAR_ZONE)) {
      stop_reason->updateStopReason(nearest_invasion_obstacle.obj_id_, StopReason::StopReasonType::CLEAR_ZONE,
                                    nearest_invasion_obstacle.invasion_s_);
    } else if (nearest_invasion_obstacle.obj_id_ == speed_wall_id_map.at(END_POINT)) {
      stop_reason->updateStopReason(nearest_invasion_obstacle.obj_id_, StopReason::StopReasonType::DESTINATION,
                                    nearest_invasion_obstacle.invasion_s_);
    } else if (nearest_invasion_obstacle.obj_id_ == speed_wall_id_map.at(REF_LINE_END)) {
      stop_reason->updateStopReason(nearest_invasion_obstacle.obj_id_, StopReason::StopReasonType::REF_LINE_END,
                                    nearest_invasion_obstacle.invasion_s_);
    }
    return;
  }
  stop_reason->updateStopReason("unrecognized_stop_reason", StopReason::StopReasonType::INVALID);
}

void SpeedPlannerBase::resetTrajectoryResultBasedAbortTask(const LocalView& local_view,
                                                           const DiscretizedPath& discretized_path,
                                                           const StageState& stage_state,
                                                           shared_ptr<SpeedResult> speed_result) {
  auto trajectory_result = speed_result->mutableTrajectoryResult();
  auto planning_traj = speed_result->trajectoryResult().trajectory_point();
  if ((stage_state == StageState::HpaDrivingStage || stage_state == StageState::ParkInStage
       || stage_state == StageState::ParkOutStage)
      && local_view.getConsolePtr()->taskStage()
             == proto::TaskCommand::TaskStage::TaskCommand_TaskStage_kParkingAbortTask) {
    trajectory_result->Clear();

    if (planning_traj.empty()) {
      return;
    }
    auto tmp_trajectory_point = trajectory_result->add_trajectory_point();
    tmp_trajectory_point->mutable_path_point()->set_s(0.0);
    tmp_trajectory_point->set_v(planning_traj.at(0).v());
    tmp_trajectory_point->set_a(planning_traj.at(0).a());
    tmp_trajectory_point->set_relative_time(0.0);
    tmp_trajectory_point->mutable_path_point()->set_x(planning_traj.at(0).path_point().x());
    tmp_trajectory_point->mutable_path_point()->set_y(planning_traj.at(0).path_point().y());
    tmp_trajectory_point->mutable_path_point()->set_z(planning_traj.at(0).path_point().z());
    tmp_trajectory_point->mutable_path_point()->set_theta(planning_traj.at(0).path_point().theta());
    tmp_trajectory_point->mutable_path_point()->set_kappa(planning_traj.at(0).path_point().kappa());
    tmp_trajectory_point->mutable_path_point()->set_front_steer(planning_traj.at(0).path_point().front_steer());

    for(int i = 1; i < planning_traj.size(); ++i){
      float a = fmin(planning_traj[i].a(), -2.0f);
      float v = fmax(tmp_trajectory_point->v() + a * 0.1, 0.0f);
      float s = tmp_trajectory_point->path_point().s() + fmax(v * 0.1 + 0.5 * a * 0.1 * 0.1, 0.0);
      auto point = discretized_path.evaluate(s);
      tmp_trajectory_point = trajectory_result->add_trajectory_point();
      tmp_trajectory_point->mutable_path_point()->set_s(fmax(s, 0.0));
      tmp_trajectory_point->set_v(v);
      tmp_trajectory_point->set_a(a);
      tmp_trajectory_point->set_relative_time(static_cast<double>(i) * 0.1);
      tmp_trajectory_point->mutable_path_point()->set_x(point.x());
      tmp_trajectory_point->mutable_path_point()->set_y(point.y());
      tmp_trajectory_point->mutable_path_point()->set_z(point.z());
      tmp_trajectory_point->mutable_path_point()->set_theta(point.theta());
      tmp_trajectory_point->mutable_path_point()->set_kappa(point.kappa());
      tmp_trajectory_point->mutable_path_point()->set_front_steer(point.front_steer());
    }
  }
}
}


