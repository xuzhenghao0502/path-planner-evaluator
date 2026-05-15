/**
 * @file st_boundary_mapper.cpp
 * @brief 时空边界映射器
 * @details 本类负责将障碍物预测轨迹映射到ST图空间，生成时空约束边界。
 */
#include "st_graph_processor/st_boundary_mapper.h"

namespace gpal::pnc::planning {
/**
 * @brief ST边界构造函数
 * 
 * @param[in] time_grid 时间网格序列（单位：秒）
 * @param[in] time_resolution 时间分辨率（ST图离散精度）
 * @param[in] time_horizon 规划时域（最大前瞻时间）
 * 
 * @details 初始化流程：
 * 1. 加载车辆几何参数（长/宽/轴距）
 * 2. 获取速度规划器全局配置
 * 3. 建立时间坐标系（0.1秒~8.0秒）
 * 
 * @par 关键配置项:
 * - 车辆参数：vehicle_config_
 * - ST图分辨率：speed_planner_config_.st_resolution()
 */
STBoundaryMapper::STBoundaryMapper(std::vector<double> time_grid, double time_resolution, double time_horizon)
    : time_grid_(time_grid), time_resolution_(time_resolution), time_horizon_(time_horizon) {
  auto config_manager = Singleton<ConfigManager>::get_instance();
  CHECK_NOTNULL(config_manager);
  vehicle_config_ = config_manager->vehicle_config();
  speed_planner_config_ = config_manager->getConfig<SpeedPlannerConfig>("SpeedPlannerConfig");
};
/**
 * @brief ST边界主处理流程（时空约束生成）
 * @param[in] local_view 局部环境感知数据
 * @param[in] obstacle_set 障碍物集合（含预测轨迹）
 * @param[in] path_group 路径信息组（原始+扩展路径）
 * 
 * @par 处理流程:
 * @startuml
 repeat :遍历障碍物集合;
   :打印障碍物初始状态;
   if (轨迹为空?) then (yes)
     :记录空轨迹警告;
   endif
   
   partition 初级过滤 {
     if (包围盒重叠检测失败?) then (yes)
       :记录过滤日志;
     endif
   }
   
   partition 次级过滤 {
     if (路径投影分析失败?) then (yes)
       :记录过滤日志;
     endif
   }
   
   partition ST边界计算 {
     :初始化ST处理器;
     :执行时空耦合计算;
     :输出ST边界结果;
   }
 repeat while (存在未处理障碍物?)
 @enduml
 *
 * @note 数据流说明:
 * - 输入: 障碍物预测轨迹(5秒/50帧)
 * - 输出: STObstacleProcessor生成的时空约束边界
 * 
 * @par 日志说明:
 * - 打印被过滤障碍物ID及原因
 * - 记录有效障碍物的坐标/航向/速度
 */
void STBoundaryMapper::process(const LocalView& local_view,
                               const ObstacleSet& obstacle_set,
                               const PathGroup& path_group,
                               const BehaviorState& behavior_state) {
  for (auto& obstacle_item : obstacle_set) {
    auto obstacle = obstacle_item.second;
    ERT_PLOG_D << "  obstace: id = " << obstacle->id() << "  x = " << obstacle->getBoundingBoxAtTime(0.0).center_x()
         << "  y = " << obstacle->getBoundingBoxAtTime(0.0).center_y()
         << " heading = " << obstacle->getBoundingBoxAtTime(0.0).heading() <<
          " length = "<<obstacle->getBoundingBoxAtTime(0.0).length() << 
          " width = " << obstacle->getBoundingBoxAtTime(0.0).width() <<
          "  speed = " << obstacle->speed() ;
    
    if (obstacle->predictedTrajectory().empty()) {
      ERT_PLOG_D << "     >>>>>>>> trajectory  empty " ;
      continue;
    }
    auto t0 = std::chrono::steady_clock::now();
    if (!primaryObstacleFilter(obstacle, path_group.extand_interval_path_)) {
      ERT_PLOG_D << "     >>>>>>>> primaryObstacleFilter ignore " ;
      continue;
    }
    auto t1 = std::chrono::steady_clock::now();
    vector<BoxProjectInfo> box_project_info;
    size_t t_index_min = obstacle->predictedTrajectory().size() - 1;
    size_t t_index_max = 0;
    if (!secondObstacleFilter(obstacle, path_group.extand_interval_path_,
                              path_group.path_points_for_segmentation_after_extand_, local_view.getChassisPtr()->Speed(), t_index_min, t_index_max,
                              &box_project_info)) {
      ERT_PLOG_D << "     >>>>>>>> secondObstacleFilter ignore " ;
      continue;
    }
    
    if(!isValidObstacleInParking(behavior_state, obstacle, box_project_info)){
      ERT_PLOG_D << "     >>>>>>>> isValidObstacleInParking ignore " ;
      continue;
    }

    auto t2 = std::chrono::steady_clock::now();
    STObstacleProcessor st_obstacle_processor(time_grid_, time_resolution_, time_horizon_);
    st_obstacle_processor.ComputeSTBoundary(obstacle, path_group.extand_interval_path_, behavior_state,  box_project_info, t_index_min,
                                            t_index_max);
    auto t3 = std::chrono::steady_clock::now();
    ERT_PLOG_I << "[primaryObstacleFilter] " << std::chrono::duration<float, std::milli>(t1 - t0).count() << " ms";
    ERT_PLOG_I << "[secondObstacleFilter] " << std::chrono::duration<float, std::milli>(t2 - t1).count() << " ms";
    ERT_PLOG_I << "[ComputeSTBoundary] " << std::chrono::duration<float, std::milli>(t3 - t2).count() << " ms";
  }
};
/**
 * @brief 计算局部路径速度下限边界（安全走廊生成）
 * @param[in] local_view 局部环境感知数据
 * @param[in] obstacle_set 障碍物集合（含预测轨迹）
 * @param[in] local_path_group 局部路径组数据
 * @param[out] local_path_lower_boundary 输出参数，存储(s,t)点及关联障碍物ID
 * 
 * @par 输入/输出参数说明:
 * | 参数                       | 类型                                  | 取值范围        | 说明                     |
 * |----------------------------|---------------------------------------|----------------|--------------------------|
 * | local_view                 | const LocalView&                     | -              | 车辆环境感知数据          |
 * | obstacle_set               | const ObstacleSet&                   | -              | 障碍物集合容器            |
 * | local_path_group           | const PathGroup&                     | -              | 局部路径组数据            |
 * | local_path_lower_boundary  | vector<pair<STPoint,string>>&        | -              | 输出参数，安全边界结果容器 |
 *
 * @par 处理流程:
 * @startuml
 :初始化时间序列边界(s=+INF);
 
 repeat :遍历障碍物集合;
   :打印障碍物初始状态;
   if (轨迹为空?) then (yes)
     :记录空轨迹警告;
   endif
   
   partition 初级过滤 {
     if (包围盒重叠检测失败?) then (yes)
       :记录过滤日志;
     endif
   }
   
   partition 次级过滤 {
     if (路径投影分析失败?) then (yes)
       :记录过滤日志;
     endif
   }
   
   partition ST边界计算 {
     :初始化ST处理器;
     :执行时空耦合计算;
     :提取障碍物ST边界;
   }
   
   partition 安全边界更新 {
     repeat :遍历时间序列;
       if (当前时刻存在有效边界?) then (yes)
         :更新最小s值边界;
         :记录关联障碍物ID;
       endif
     repeat while (存在未处理时刻?)
   }
 repeat while (存在未处理障碍物?)
 @enduml
 *
 * @note 核心特性:
 * - 时间分辨率: 0.1秒（通过time_resolution_配置）
 * - 安全策略: 取所有障碍物ST边界的最小s值
 * - 输出结构: vector<pair<STPoint,障碍物ID>>
 *
 * @warning 边界有效性条件:
 * - 障碍物边界时间需在检查时域内（默认5秒）
 * - 障碍物上边界s值需大于等于0
 */
void STBoundaryMapper::CaculateLocalPathLowerBoundary(
    const LocalView& local_view, const ObstacleSet& obstacle_set,
    const PathGroup& local_path_group,const BehaviorState& behavior_state, std::vector<std::pair<STPoint, std::string>>& local_path_lower_boundary) {
  local_path_lower_boundary.clear();
  for (size_t i = 0; i < static_cast<size_t>(speed_planner_config_.local_path_check_time_horizon() / time_resolution_);
       ++i) {
    local_path_lower_boundary.emplace_back(
        std::make_pair(STPoint(kPostiveInfinity, static_cast<double>(i) * time_resolution_), ""));
  }
  // bool ego_away_from_discretized_path = isEgoAwayFromDiscretizedPath(local_view, local_path_group.origin_path_);
  for (auto& obstacle_item : obstacle_set) {
    auto obstacle = obstacle_item.second;
    ERT_PLOG_D << "local_path  obstace: id = " << obstacle->id() << "  x = " << obstacle->getBoundingBoxAtTime(0.0).center_x()
         << "  y = " << obstacle->getBoundingBoxAtTime(0.0).center_y()
         << " heading = " << obstacle->getBoundingBoxAtTime(0.0).heading() << "  speed = " << obstacle->speed() ;
    if (obstacle->predictedTrajectory().empty()) {
      ERT_PLOG_D << "     >>>>>>>> trajectory  empty " ;
      continue;
    }
        auto time1 = GetSystemUsTime();

    if (!primaryObstacleFilter(obstacle, local_path_group.extand_interval_path_)) {
      ERT_PLOG_D << "     >>>>>>>> primaryObstacleFilter ignore " ;
      continue;
    }
    vector<BoxProjectInfo> box_project_info;
    size_t t_index_min = obstacle->predictedTrajectory().size();
    size_t t_index_max = 0;
    if (!secondObstacleFilter(obstacle, local_path_group.extand_interval_path_,
                              local_path_group.path_points_for_segmentation_after_extand_, local_view.getChassisPtr()->Speed(), t_index_min, t_index_max,
                              &box_project_info)) {
      ERT_PLOG_D << "     >>>>>>>> secondObstacleFilter ignore " ;
      continue;
    }
    if(!isValidObstacleInParking(behavior_state, obstacle, box_project_info)){
      ERT_PLOG_D << "     >>>>>>>> isValidObstacleInParking ignore " ;
      continue;
    }
    //TODO
    size_t max_time = 30;
    t_index_max = min(t_index_max, max_time);
    if (t_index_min > t_index_max) {
      continue;
    }
    STBoundary boundary;
    STObstacleProcessor st_obstacle_processor(time_grid_, time_resolution_, time_horizon_);
    st_obstacle_processor.ComputeSTBoundaryLocalPath(obstacle, local_path_group.extand_interval_path_, behavior_state,  box_project_info,
                                                     t_index_min, t_index_max, boundary);
    if (boundary.IsEmpty() ||
        boundary.bottom_left_point().t() >= speed_planner_config_.local_path_check_time_horizon() + kMathEpsilon ||
        boundary.upper_left_point().s() < kMathEpsilon) {
      continue;
    }
    for (size_t i = 0; i < local_path_lower_boundary.size(); ++i) {
      double s_upper, s_lower;
      if (!boundary.getBoundarySRange(local_path_lower_boundary[i].first.t(), &s_upper, &s_lower)) {
        continue;
      }
      if (s_lower < local_path_lower_boundary[i].first.s()) {
        local_path_lower_boundary[i].first.set_s(s_lower);
        local_path_lower_boundary[i].second = boundary.id();
      }
    }
  }
}
/**
 * @brief 判断自车是否偏离参考路径（路径跟随状态监测）
 * @param[in] local_view 局部环境感知数据（包含定位信息）
 * @param[in] discretized_path 离散化参考路径
 * @return bool 偏离状态（true表示偏离需接管，false表示正常跟随）
 * 
 * @par 处理流程:
 * @startuml
 partition 数据准备 {
   :获取车辆定位信息;
   :查找最近路径点;
   :计算横向/航向误差;
 }
 
 partition 动态阈值计算 {
   :根据车速查表获取阈值;
   :横向误差阈值: 0.1m~0.3m;
   :航向误差阈值: 3°~10°;
 }
 
 partition 偏离判定 {
   if (横向误差 < 阈值 && 航向误差 < 阈值?) then (yes)
     :返回正常跟随(false);
   else (no)
     :返回需要接管(true);
   endif
 }
 @enduml
 *
 * @note 坐标系说明:
 * - 横向误差计算：Frenet坐标系l方向分量
 * - 航向误差计算：路径切线方向与车辆航向角差值
 * - 路径点theta为路径切线方向（考虑曲率影响）
 *
 * @warning 需确保输入参数:
 * - 定位数据已完成坐标系对齐
 * - 参考路径点包含有效theta值
 */
bool STBoundaryMapper::isEgoAwayFromDiscretizedPath(const LocalView& local_view,
                                                    const DiscretizedPath& discretized_path) {
  bool ego_away_from_discretized_path = true;
  std::vector<double> heading_err_thres_table = {10.0, 10.0, 5.0, 5.0, 3.0, 3.0, 3.0, 3.0};
  std::vector<double> lat_err_thres_table = {0.3, 0.3, 0.3, 0.2, 0.1, 0.1, 0.1, 0.1};
  std::vector<double> ego_speed_for_err = {0.0, 5.0, 10.0, 20.0, 40.0, 60.0, 80.0, 100.0};
  const double ego_speed = local_view.getChassisPtr()->Speed();
  double heading_err_thres = math::TableLookUp1D(ego_speed_for_err, heading_err_thres_table, ego_speed);
  double lat_err_thres = math::TableLookUp1D(ego_speed_for_err, lat_err_thres_table, ego_speed);
  double min_dist = std::numeric_limits<double>::max();
  auto nearest_point =
      discretized_path.getNearestPoint(math::Vec3d(local_view.getLocalizationPtr()->vehicleAlignPosePoint().x(),
                                                   local_view.getLocalizationPtr()->vehicleAlignPosePoint().y(),
                                                   local_view.getLocalizationPtr()->vehicleAlignPosePoint().z()),
                                       min_dist);
  const double dx = local_view.getLocalizationPtr()->vehicleAlignPosePoint().x() - nearest_point.x();
  const double dy = local_view.getLocalizationPtr()->vehicleAlignPosePoint().y() - nearest_point.y();
  const double cos_theta = std::cos(local_view.getLocalizationPtr()->vehicleAlignPosePoint().yaw());
  const double sin_theta = std::sin(local_view.getLocalizationPtr()->vehicleAlignPosePoint().yaw());
  const double longi_error = cos_theta * dx + sin_theta * dy;
  const double lat_error = -sin_theta * dx + cos_theta * dy;
  double heading_err = local_view.getLocalizationPtr()->vehicleAlignPosePoint().yaw() - nearest_point.theta();
  heading_err = math::NormalizeAngle(heading_err);
  if (abs(lat_error) < lat_err_thres && abs(heading_err) < heading_err_thres * ANG2RAD) {
    ego_away_from_discretized_path = false;
  }
  return ego_away_from_discretized_path;
}
/**
 * @brief 障碍物初级过滤器（快速碰撞风险检测）
 * @param[in] obstacle 待检测障碍物对象
 * @param[in] path 参考路径离散点序列
 * @return bool 是否保留该障碍物（true表示存在潜在风险）
 * 
 * @par 处理流程:
 * @startuml
 if (障碍物轨迹为空?) then (yes)
   :返回false;
 else (no)
   partition 轨迹降采样 {
     repeat :遍历原始轨迹;
       :每隔10帧采样;
     repeat while (存在未处理轨迹点?)
   }
   
   partition 初次快速检测 {
     :全路径范围AABB检测;
     if (缓冲区系数1.0) then (无重叠)
       :返回false;
     endif
   }
   
   partition 二次分段检测 {
     repeat :遍历路径分段;
       :8点间隔检测;
       if (缓冲区系数0.67存在重叠?) then (yes)
         :标记存在风险;
         break;
       endif
     repeat while (存在未处理路径段?)
   }
 endif
 :返回风险标记;
 @enduml
 *
 * @note 关键参数:
 * - 轨迹采样间隔: 10帧
 * - 路径分段间隔: 8点
 * - 缓冲区系数: 1.0→0.67
 *
 * @warning 特性说明:
 * - 牺牲检测精度换取计算速度（相比OBB快10倍）
 * - 适用于障碍物快速筛选阶段
 */
bool STBoundaryMapper::primaryObstacleFilter(const shared_ptr<SpeedPlannerObstacle>& obstacle, const DiscretizedPath& path) {
  auto trajectory = obstacle->predictedTrajectory();

  if (trajectory.empty()) {
    return false;
  }
  size_t last_t_index = 0;
  std::vector<proto::TrajectoryPoint> sampled_obs_trajectory;
  const size_t obs_sampling_interval = 10UL;
  const size_t path_sampling_interval = 8UL;
  for (size_t i = 0UL; i < trajectory.size(); i += obs_sampling_interval) {
    last_t_index = i;
    sampled_obs_trajectory.emplace_back(trajectory[i]);
  }
  if ((last_t_index + 1UL < trajectory.size()) && (last_t_index > 0UL)) {
    sampled_obs_trajectory.emplace_back(trajectory.back());
  }

  bool has_risk = false;
  // first time to skip the obs which is far from us.
  if (!checkAABoxOverlap(obstacle, sampled_obs_trajectory, 0, sampled_obs_trajectory.size(), path, 0, path.size(),
                         1.0)) {
    // TODO: @lvjidong 使用extand path做检查
    has_risk = false;
    return has_risk;
  }

  // second time to skip the obs which is far from us.
  for (size_t i = 0UL; i < path.size(); ++i) {  // 第二次筛选
    if (i % path_sampling_interval == 0) {
      has_risk |= checkAABoxOverlap(obstacle, sampled_obs_trajectory, 0, sampled_obs_trajectory.size(), path, i,
                                    i + path_sampling_interval < path.size() ? i + path_sampling_interval : path.size(),
                                    2.0 / 3.0);
    }
    if (has_risk) {
      break;
    }
  }
  return has_risk;
}
/**
 * @brief 执行轴对齐包围盒快速碰撞检测
 * @details 通过计算障碍物轨迹段和自车路径段的AABB包围盒，快速判断两者是否存在潜在碰撞风险
 * 
 * @param[in] obstacle 障碍物对象，包含几何属性和预测轨迹
 *   - 类型: shared_ptr<SpeedPlannerObstacle>
 *   - 关键属性: perceptionBoundingBox() 获取长宽尺寸
 * @param[in] obs_trajectory 障碍物预测轨迹点序列
 *   - 类型: vector<proto::TrajectoryPoint>
 *   - 取值范围: 轨迹点时间范围应覆盖检测时域
 * @param[in] obs_start_idx 障碍物轨迹起始索引
 *   - 类型: size_t
 *   - 取值范围: [0, obs_trajectory.size())
 * @param[in] obs_end_idx 障碍物轨迹结束索引
 *   - 类型: size_t
 *   - 取值范围: (obs_start_idx, obs_trajectory.size()]
 * @param[in] path 自车参考路径离散点序列
 *   - 类型: DiscretizedPath
 *   - 数据结构: 包含x,y坐标的路径点数组
 * @param[in] buffer 缓冲区扩展系数
 *   - 类型: double
 *   - 取值范围: (0.0, 1.0] 典型值0.67/1.0
 * 
 * @par 处理流程图:
 * @startuml
 * start
 * :初始化障碍物轨迹边界极值;
 * repeat :遍历障碍物轨迹段
 *   :更新x_min/x_max/y_min/y_max;
 * repeat while (存在未处理轨迹点?)
 * 
 * :计算障碍物缓冲区尺寸\n(长宽最大值*buffer+0.1);
 * :创建障碍物AABB包围盒;
 * 
 * :初始化自车路径边界极值;
 * repeat :遍历自车路径段
 *   :更新x_min/x_max/y_min/y_max;
 * repeat while (存在未处理路径点?)
 * 
 * :计算自车缓冲区尺寸\n(车长车宽*buffer+0.01);
 * :创建自车AABB包围盒;
 * 
 * if (包围盒存在重叠?) then (是)
 *   :返回true;
 * else (否)
 *   :返回false;
 * endif
 * stop
 * @enduml
 * 
 * @par 关键变量说明:
 * - obs_buffer: 障碍物包围盒扩展量，计算公式 = max(长,宽)*buffer + 0.1
 * - adc_buffer: 自车包围盒扩展量，计算公式 = max(车长,车宽)*buffer + 0.01
 * - kPostiveInfinity/kNegativeInfinity: 极值初始值(1e10/-1e10)
 * 
 * @note 检测特性:
 * - 时间复杂度: O(M+N) M-轨迹点数 N-路径点数
 * - 空间复杂度: O(1)
 * - 检测精度: 保守估计，可能包含10cm级别的误差
 * 
 * @warning 注意索引越界:
 * - 原代码存在未赋值的三目运算符逻辑问题，建议修正为:
 *   size_t valid_obs_end = std::min(obs_end_idx, obs_trajectory.size());
 *   size_t valid_adc_end = std::min(adc_end_idx, path.size());
 * 
 * @return bool 碰撞检测结果
 *   - true: 存在潜在碰撞风险
 *   - false: 无碰撞风险
 */
bool STBoundaryMapper::checkAABoxOverlap(const shared_ptr<SpeedPlannerObstacle>& obstacle,
                                         const std::vector<proto::TrajectoryPoint>& obs_trajectory,
                                         const size_t& obs_start_idx, const size_t& obs_end_idx,
                                         const DiscretizedPath& path, const size_t& adc_start_idx,
                                         const size_t& adc_end_idx, const double& buffer) {
  double obs_min_x = kPostiveInfinity;
  double obs_max_x = kNegativeInfinity;
  double obs_min_y = kPostiveInfinity;
  double obs_max_y = kNegativeInfinity;
  obs_end_idx > obs_trajectory.size() ? obs_trajectory.size() : obs_end_idx;
  adc_end_idx > path.size() ? path.size() : adc_end_idx;
  for (size_t i = obs_start_idx; i < obs_end_idx; i++) {
    obs_min_x = std::min(obs_min_x, obs_trajectory[i].path_point().x());
    obs_max_x = std::max(obs_max_x, obs_trajectory[i].path_point().x());
    obs_min_y = std::min(obs_min_y, obs_trajectory[i].path_point().y());
    obs_max_y = std::max(obs_max_y, obs_trajectory[i].path_point().y());
  }
  double obs_buffer =
      std::max(obstacle->perceptionBoundingBox().length(), obstacle->perceptionBoundingBox().width()) * buffer + 0.1;
  math::Box2d obs_box = math::Box2d::CreateAABox(math::Vec2d(obs_min_x - obs_buffer, obs_min_y - obs_buffer),
                                                 math::Vec2d(obs_max_x + obs_buffer, obs_max_y + obs_buffer));

  double adc_min_x = kPostiveInfinity;
  double adc_max_x = kNegativeInfinity;
  double adc_min_y = kPostiveInfinity;
  double adc_max_y = kNegativeInfinity;
  for (size_t i = adc_start_idx; i < adc_end_idx; i++) {
    adc_min_x = std::min(adc_min_x, path[i].x());
    adc_max_x = std::max(adc_max_x, path[i].x());
    adc_min_y = std::min(adc_min_y, path[i].y());
    adc_max_y = std::max(adc_max_y, path[i].y());
  }
  double adc_buffer =
      std::max(vehicle_config_.vehicle_param().length(), vehicle_config_.vehicle_param().width()) * buffer + 0.01;
  math::Box2d adc_box = math::Box2d::CreateAABox(math::Vec2d(adc_min_x - adc_buffer, adc_min_y - adc_buffer),
                                                 math::Vec2d(adc_max_x + adc_buffer, adc_max_y + adc_buffer));
  return adc_box.HasOverlap(obs_box);
}
/**
 * @brief 障碍物次级过滤器（精确路径投影分析）
 * @param[in] obstacle 待检测障碍物对象
 * @param[in] path 参考路径离散点序列
 * @param[in] path_segmentation 扩展路径分割点序列
 * @param[out] t_index_min 输出参数，障碍物影响时间范围起始索引
 * @param[out] t_index_max 输出参数，障碍物影响时间范围结束索引
 * @param[out] box_project_info 输出参数，存储障碍物投影信息集合
 * 
 * @par 输入/输出参数说明:
 * | 参数                | 类型                            | 取值范围          | 说明                     |
 * |---------------------|---------------------------------|------------------|--------------------------|
 * | obstacle            | const shared_ptr<SpeedPlannerObstacle>& | - | 包含预测轨迹的障碍物对象 |
 * | path                | const DiscretizedPath&          | - | 扩展间隔路径数据         |
 * | path_segmentation   | const vector<PathPt>&           | - | 路径分割后的参考点序列   |
 * | t_index_min         | size_t&                         | [0, trajectory.size()) | 影响起始时间索引         |
 * | t_index_max         | size_t&                         | [t_index_min, trajectory.size()) | 影响结束时间索引       |
 * | box_project_info    | std::vector<BoxProjectInfo>*     | - | 投影信息集合容器指针     |
 *
 * @par 处理流程:
 * @startuml
 partition 轨迹降采样 {
   :按10帧间隔采样预测轨迹;
   :保留首末帧保证完整性;
 }
 
 partition 路径分段投影 {
   repeat :遍历各路径段;
     :计算障碍物中心点投影;
     :记录最小横向距离及对应s值;
   repeat while (存在未处理路径段?)
 }
 
 partition 横向缓冲区检查 {
   :设置2.0米横向缓冲阈值;
   :筛选满足缓冲条件的投影点;
 }
 
 partition 风险判定 {
   :基于障碍物尺寸和速度计算检测距离;
   :更新时间索引范围[min, max];
 }
 @enduml
 *
 * @note 核心算法：
 * - 横向缓冲区阈值：2.0米（考虑路径跟踪误差）
 * - 动态障碍物检测距离：max(障碍物尺寸+自车半宽+速度项,4.0米)
 * - 轨迹采样间隔：10帧（平衡精度与效率）
 *
 * @warning 实现限制:
 * - 依赖路径分割点的准确性（path_segmentation）
 * - 静态障碍物仅处理首帧投影信息
 * - 索引范围需满足 t_index_min <= t_index_max
 */
bool STBoundaryMapper::secondObstacleFilter(const shared_ptr<SpeedPlannerObstacle>& obstacle, const DiscretizedPath& path,
                                            const vector<PathPt>& path_segmentation, const double& vehicle_speed, size_t& t_index_min,
                                            size_t& t_index_max, std::vector<BoxProjectInfo>* box_project_info) {
  box_project_info->clear();
  std::vector<proto::TrajectoryPoint> trajectory = obstacle->predictedTrajectory();
  if (trajectory.empty()) {
    return false;
  }
  // Subsampling the trajectory of prediction
  size_t sampling_delta_i = 10UL;

  bool has_lateral_risk = false;
  std::vector<size_t> sampled_t_indices;
  for (size_t i = 0UL; i < trajectory.size(); i += sampling_delta_i) {
    sampled_t_indices.emplace_back(i);
  }
  size_t last_t_index = sampled_t_indices.back();
  if ((last_t_index + 1UL < trajectory.size()) && (last_t_index > 0UL)) {
    sampled_t_indices.emplace_back(trajectory.size() - 1UL);
  }
  std::pair<double, double> final_project_result = {kPostiveInfinity,
                                                    kPostiveInfinity};  // <project_pt_s,  dis_to_path >
  std::vector<std::pair<double, double>> project_results_candidates = {};
  const double lateral_check_buffer = 2.0;
  for (size_t j = 0UL; j < sampled_t_indices.size(); ++j) {
    size_t i = sampled_t_indices[j];
    const auto& traj_pt = trajectory[i];
    const math::Box2d obs_box = obstacle->getBoundingBox(traj_pt);
    final_project_result = {kPostiveInfinity, kPostiveInfinity};
    project_results_candidates.clear();

    for (size_t seg_index = 0; seg_index + 1 < path_segmentation.size(); ++seg_index) {
      auto result = calcSLProjection(obs_box.center(), path, path_segmentation.at(seg_index).s(),
                                     path_segmentation.at(seg_index + 1).s());
      if (fabs(result.second) < fabs(final_project_result.second)) {
        final_project_result = result;
      }
      project_results_candidates.emplace_back(result);
    }

    std::sort(project_results_candidates.begin(), project_results_candidates.end(),
              [](const std::pair<double, double>& a, const std::pair<double, double>& b) { return a.first < b.first; });
    for (const auto& project_result : project_results_candidates) {
      if (fabs(project_result.second) <= fabs(final_project_result.second) + lateral_check_buffer) {
        final_project_result = project_result;
        break;
      }
    }

    box_project_info->emplace_back(BoxProjectInfo(i, traj_pt.relative_time(), final_project_result));
    if (isObstacleStatic(obstacle->type(), obstacle->predictedTrajectory(), obstacle->isStatic())) {
      break;
    }
  }
  double half_adc_width = vehicle_config_.vehicle_param().width() / 2.0;
  const math::Box2d obs_box = obstacle->getBoundingBoxAtTime(0.0);
  double max_check_l =
      std::fmax(obs_box.length() + obs_box.width() + half_adc_width + 1.0 * fabs(obstacle->speed()), 4.0);
  double max_check_s = std::fmax(100, vehicle_speed * 8.0);
  bool has_longitual_risk = (box_project_info->front().project_s <= max_check_s);
  bool has_risk = false;
  t_index_min = trajectory.size() - 1;
  t_index_max = 0;
  for (size_t i = 0; i < box_project_info->size(); ++i) {
    // ERT_PLOG_I << "     index : " << box_project_info->at(i).index << "  time : " << box_project_info->at(i).time
    //      << "  project_s : " << box_project_info->at(i).project_s
    //      << "  project_l : " << box_project_info->at(i).project_l ;
    if (box_project_info->at(i).project_l <= max_check_l && has_longitual_risk) {
      t_index_min = min(t_index_min, box_project_info->at(i).index);
      t_index_max = max(t_index_max, box_project_info->at(i).index);
      has_risk = true;
    }
  }
  return has_risk;
}
/**
 * @brief 计算障碍物在Frenet坐标系下的SL投影（路径相对坐标系）
 * @param[in] obs_pt 障碍物坐标（世界坐标系）
 * @param[in] path 参考路径离散点序列
 * @param[in] start_s 路径段起始s值
 * @param[in] end_s 路径段结束s值
 * @return std::pair<double, double> (投影点s值, 横向距离l)
 * 
 * @par 输入/输出参数说明:
 * | 参数        | 类型                 | 取值范围          | 说明                     |
 * |-------------|----------------------|------------------|--------------------------|
 * | obs_pt      | const math::Vec2d&   | -                | 障碍物世界坐标系坐标     |
 * | path        | const DiscretizedPath& | -              | 参考路径离散点序列       |
 * | start_s     | const double&        | [0, path总长)    | 路径段起始s坐标          |
 * | end_s       | const double&        | [start_s, path总长) | 路径段结束s坐标        |
 * | 返回值      | std::pair<double, double> | -          | <投影s坐标,横向距离>     |
 *
 * @par 处理流程:
 * @startuml
 partition 路径段处理 {
   if (路径段为单点?) then (yes)
     :直接计算欧氏距离;
   else
     :初始化s搜索范围;
   endif
 }
 
 partition 优化求解 {
   :配置二次规划求解器参数;
   :构建目标函数(最小化横向距离);
   :添加s坐标边界约束;
   :执行IPM算法求解;
 }
 
 partition 异常处理 {
   if (求解失败?) then (yes)
     :回退到路径段起点计算;
     :记录优化失败日志;
   endif
 }
 @enduml
 *
 * @note 核心算法：
 * - 使用二次规划优化横向距离：min( (x_pt - x_path)^2 + (y_pt - y_path)^2 )
 * - 约束条件：s ∈ [start_s, end_s]
 * - 优化器配置：IPM算法，收敛精度1e-2
 *
 * @warning 关键限制条件:
 * - 强依赖路径曲率参数(kappa)的准确性
 * - 路径段长度需大于0.1米（避免数值不稳定）
 * - 优化失败时返回路径段起点计算结果
 */
std::pair<double, double> STBoundaryMapper::calcSLProjection(const math::Vec2d& obs_pt, const DiscretizedPath& path,
                                                             const double& start_s, const double& end_s) {
  const double init_s = (start_s + end_s) / 2.0;
  double dx = 0.0;
  double dy = 0.0;
  double s_lower = start_s;
  double s_upper = end_s;
  if (start_s > end_s) {
    s_lower = end_s;
    s_upper = start_s;
  } else if (start_s == end_s) {
    auto curr_pt = path.evaluate(init_s);
    dx = curr_pt.x() - obs_pt.x();
    dy = curr_pt.y() - obs_pt.y();
    double lateral_dist = std::sqrt(dx * dx + dy * dy);
    return std::make_pair(start_s, lateral_dist);
  }
  if (sl_optimizer_ == nullptr) {
    sl_optimizer_ = std::make_shared<QuadraticProgrammingSolver>(1, 0, 2);
    sl_optimizer_->config()->set_first_order_tol(1e-2);
    sl_optimizer_->config()->set_equality_constraint_tol(1e-3);
    sl_optimizer_->config()->set_complementary_tol(1e-2);
    sl_optimizer_->config()->set_inequality_constraint_tol(1e-2);
    sl_optimizer_->config()->set_mu_min(1e-3);
    sl_optimizer_->config()->set_barrier_strategy(IPMConfig::FIXED);
    sl_optimizer_->config()->set_auto_update_param(true);
  }
  sl_optimizer_->setParam([&](Eigen::MatrixXd* Q, Eigen::VectorXd* c, Eigen::MatrixXd* A, Eigen::VectorXd* b,
                              Eigen::MatrixXd* G, Eigen::VectorXd* h, const Eigen::VectorXd& x) {
    auto curr_pt = path.evaluate(x(0));
    dx = curr_pt.x() - obs_pt.x();
    dy = curr_pt.y() - obs_pt.y();
    double jacobian = std::cos(curr_pt.theta()) * dx + std::sin(curr_pt.theta()) * dy;
    double hessian = curr_pt.kappa() * (-std::sin(curr_pt.theta()) * dx + std::cos(curr_pt.theta()) * dy) + 1.0;
    if (abs(hessian) < 1e-5) {
      hessian = 1e-5;
    }
    Q_sl_ << hessian;
    c_sl_ << jacobian;
    G_sl_ << -1.0, 1.0;
    h_sl_ << s_lower, -s_upper;
    *Q = Q_sl_;
    *c = c_sl_;
    *G = G_sl_;
    *h = h_sl_;
    *A = Eigen::MatrixXd::Zero(0, 1);
    *b = Eigen::VectorXd::Zero(0);
  });
  x_init_ << init_s;
  sl_optimizer_->setX0(x_init_);
  sl_optimizer_->solve();
  auto solution = sl_optimizer_->getX();
  auto info = sl_optimizer_->getSolveInfo();

  if (info.status != SolveStatus::SOLVED) {
    // PERROR << "[SL_FAILED]status: " << info.status;
    // SFIELD_DEBUG(
    //     sl_optimizer,
    //     "optimizer failed: flag: {}, s bound: [{},{}], solution: {}, distance: {}, iter: {}, computation time: {}ms",
    //     info.status, s_lower, s_upper, solution(0), std::sqrt(dx * dx + dy * dy), info.iteration_number,
    //     info.computation_time * 1000.0);
    sl_optimizer_->solve();
    auto re_info = sl_optimizer_->getSolveInfo();
    if (re_info.status != SolveStatus::SOLVED) {
      //   PERROR << "[SL_FAILED2]status: " << re_info.status;
      solution(0) = start_s;
      auto curr_pt = path.evaluate(start_s);
      dx = curr_pt.x() - obs_pt.x();
      dy = curr_pt.y() - obs_pt.y();
    }
  }
  return std::make_pair(solution(0), std::sqrt(dx * dx + dy * dy));
}

bool STBoundaryMapper::isValidObstacleInParking(const BehaviorState& behavior_state,
                                                std::shared_ptr<SpeedPlannerObstacle> obstacle,
                                                const std::vector<BoxProjectInfo>& box_project_info) {
  if (behavior_state.park_in_state_) {
    if (box_project_info.empty()) {
      return false;
    }
    if (box_project_info[0].project_s < 0.0 && obstacle->type() == Decision::ObjectType::PEDESTRIAN) {
      return false;
    }
  }
  return true;
}

}  // namespace gpal::pnc::planning