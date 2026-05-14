/**
 * @file st_graph_processor.cpp
 * @brief ST图处理器
 * @details 本类负责在ST坐标空间进行动态规划约束生成，及多场景决策结果处理
 */
#include "st_graph_processor/st_graph_processor.h"

namespace gpal::pnc::planning {
/**
 * @brief ST图处理器构造函数
 * @param[in] time_grid 时间网格序列（单位：秒），范围[0, time_horizon]
 * @param[in] time_resolution 时间分辨率（默认0.1秒），范围(0,1.0]
 * @param[in] time_horizon 规划时域（默认5.0秒），范围[3.0,10.0]
 * 
 * @par 关键初始化参数:
 * - config_manager_: 配置管理单例，通过Singleton模式获取
 * - vehicle_param_: 车辆参数配置，包含车长/车宽等关键尺寸
 * - speed_planner_config_: 速度规划模块配置参数集合
 * 
 * @par 流程图:
 * @startuml
 * start
 * :初始化时间网格参数;
 * :加载配置管理器;
 * :获取车辆物理参数;
 * :加载速度规划配置;
 * stop
 * @enduml
 */
STGraphProcessor::STGraphProcessor(std::vector<double> time_grid, double time_resolution, double time_horizon)
    : time_grid_(time_grid), time_resolution_(time_resolution), time_horizon_(time_horizon) {
  config_manager_ = Singleton<ConfigManager>::get_instance();
  CHECK_NOTNULL(config_manager_);
  vehicle_param_ = config_manager_->vehicle_config().vehicle_param();
  speed_planner_config_ = config_manager_->getConfig<SpeedPlannerConfig>("SpeedPlannerConfig");
};
/**
 * @brief 时空联合规划主流程
 * @param[in] local_view 局部视图数据（含定位/感知/预测）
 * @param[out] speed_result 速度规划结果（含ST图/安全走廊等）
 * 
 * @par 关键处理步骤:
 * 1. 时间戳同步（对齐传感器数据）
 * 2. ST边界映射（障碍物时空投影）
 * 3. 局部路径OD检查（防偏离保护）
 * 4. 障碍物决策标签更新（跟驰/让行/超车）
 * 5. 可行驶区域生成（安全走廊计算）
 * 
 * @par 流程图:
 * @startuml
 * start
 * :设置当前时间戳;
 * :创建STBoundaryMapper;
 * if (启用局部路径检查?) then (是)
 *   :计算路径下边界;
 * endif
 * if (非泊车状态?) then (是)
 *   :执行跟驰障碍物处理;
 * endif
 * :更新障碍物决策标签;
 * :生成可行驶区域边界;
 * :更新速度墙约束;
 * stop
 * @enduml
 */
void STGraphProcessor::process(const LocalView& local_view, const DecisionResult& decision_result,
                               const LateralPath& lateral_path_group, const BehaviorState& behavior_state,
                               const int64_t& time_stamp, ObstacleSet& obstacle_map,
                               std::shared_ptr<SpeedResult> speed_result) {
  // 2. st_drive_boundry计算
  //     a. 基于横向轨迹计算障碍物st
  //     b. 障碍物决策tag校验
  //     c. 基于st+tag计算上下硬边界
  //     d. 速度墙硬边界计算
  //     e. 输出soft/hard boundary
  time_stamp_ = time_stamp;

  STBoundaryMapper st_boundary_mapper_(time_grid_, time_resolution_, time_horizon_);
  st_boundary_mapper_.process(local_view, obstacle_map,lateral_path_group.discretized_path_group_, behavior_state );
  if(speed_planner_config_.enable_local_path_od_check()){
      st_boundary_mapper_.CaculateLocalPathLowerBoundary(local_view, obstacle_map,  lateral_path_group.local_path_group_,behavior_state,
                                                     *speed_result->mutableLocalPathLowerBoundary());
  }

  if (!speed_result->getBehaviorState().park_in_state_) {
    FollowObstacleProcess(obstacle_map);
  }

  // TODO: 基于决策结果更新换道overtake障碍物ST
  bool is_lane_change = false;
  string lane_change_overtake_id = "";
  if ( is_lane_change) {
    laneChangeOvertakeObstacleProcess(obstacle_map, lane_change_overtake_id);
  }

  std::vector<const STBoundary*> boundaries;
  for (const auto& obstacle_item : obstacle_map) {
    auto& obstacle = obstacle_item.second;
    updateDecisionObstacleTag(local_view, obstacle, speed_result);
    if (!obstacle->pathStBoundary().IsEmpty()) {
      boundaries.push_back(&obstacle->pathStBoundary());
    }
  }

  oppositeObstacleProcess(local_view, obstacle_map);
  crossObstacleProcess(local_view, obstacle_map);
  mergeObstacleProcess(local_view, obstacle_map);

  TrajectoryPt init_point;
  double min_s_on_st_boundaries = 0.0;
  double path_data_length = lateral_path_group.discretized_path_group_.origin_path_.back().s();
  speed_result->mutableStGraphData()->loadData(boundaries, min_s_on_st_boundaries, init_point, path_data_length,
                                               time_horizon_);
  calculateDrivableBoundaries(speed_result->mutableStGraphData());

  updataSpeedWallBoundInStGraph(local_view, speed_result->speed_walls(), speed_result->mutableStGraphData());
  // for (auto& bound : st_graph.st_drivable_boundaries()) {
  //   ERT_PLOG_I << "st_drivable_boundaries:  t = " << bound.t << "  lower_obs_id = " << bound.lower_obj_id
  //        << "  lower_s = " << bound.s_lower_bound << "  upper_obs_id = " << bound.upper_obj_id
  //        << "  upper_s = " << bound.s_upper_bound ;
  // }
  // post boundary info，debug info
}
/**
 * @brief CIPV专用ST边界处理（最接近路径车辆处理）
 * @param[in] local_view 局部环境感知数据（定位/底盘/预测等）
 * @param[in] decision_result 决策模块输出结果
 * @param[in] lateral_path_group 横向路径组（含扩展路径段）
 * @param[in] time_stamp 当前时间戳（微秒级）
 * @param[in,out] obstacle_map 障碍物集合（输入原始数据，输出处理后数据）
 * @param[out] speed_result 速度规划结果（含ST图数据）
 * 
 * @par 核心处理流程：
 * 1. 障碍物预处理：
 *    - 遍历障碍物集合（当前帧约20-50个）
 *    - 基于SL投影计算横向距离（精度±0.1米）
 * 2. 跟驰障碍物筛选：
 *    - 横向距离阈值动态调整（查表机制）
 *    - α-β滤波处理速度/加速度（α=0.6）
 * 3. ST边界生成：
 *    - 根据运动学模型生成FOLLOW型边界
 *    - 维护5帧历史数据滑动窗口
 * 
 * @par 关键参数阈值：
 * | 相对距离(m) | 横向阈值(m) | 滤波系数 |
 * |------------|------------|--------|
 * | 0-20       | 1.0        | 0.6     |
 * | 20-50      | 1.875       | 0.6     |
 * | >70        | 2.5-3.0     | 0.6     |
 * 
 * @par 流程图:
 * @startuml
 * start
 * :初始化时间戳;
 * :创建STBoundaryMapper;
 * while (遍历障碍物集合?) is (存在未处理项)
 *   :计算SL投影坐标;
 *   if (横向距离>阈值?) then (是)
 *     :标记为ACC_OBSTACLE;
 *   else (否)
 *     :执行α-β滤波;
 *     :生成FOLLOW边界;
 *   endif
 * endwhile
 * :更新历史数据窗口;
 * :生成可行驶区域;
 * stop
 * @enduml
 * 
 * @note 典型处理耗时：3-5ms（100个障碍物场景）
 * @warning 需确保输入的local_view已包含最新感知数据
 */
void STGraphProcessor::processCipvStBoundary(const LocalView& local_view, const DecisionResult& decision_result,
                               const LateralPath& lateral_path_group, const int64_t& time_stamp, ObstacleSet& obstacle_map,
                               std::shared_ptr<SpeedResult> speed_result){

  time_stamp_ = time_stamp;
  
  std::vector<const STBoundary*> boundaries;

  std::unordered_map<std::string, double> follow_obs_speed_current_frame;
  std::unordered_map<std::string, double> follow_obs_accel_current_frame;
  std::unordered_map<std::string, double> follow_obs_distance_current_frame;
  ERT_PLOG_I<< "processCipvStBoundary   "<<"  obstacle_map size: " << obstacle_map.size() ;

  for (auto& obstacle_item : obstacle_map) {
    auto curr_obstacle = obstacle_item.second;
    // if(curr_obstacle->pathStBoundary().IsEmpty()){
      // vector<BoxProjectInfo> box_project_info;
      // size_t t_index_min = 0;
      // size_t t_index_max = obstacle->predictedTrajectory().size() - 1;
      // STObstacleProcessor st_obstacle_processor(time_grid_, time_resolution_, time_horizon_);
      // STBoundary boundary;
      // st_obstacle_processor.ComputeSTBoundaryLocalPath(curr_obstacle, lateral_path_group.local_path_group_.extand_interval_path_, box_project_info, t_index_min,
      //                                       t_index_max, boundary);
    // }
    // double init_s_lower = curr_obstacle->pathStBoundary().lower_points().front().s();
    STBoundaryMapper st_boundary_mapper(time_grid_, time_resolution_, time_horizon_);
    auto center_pt = curr_obstacle->getBoundingBoxAtTime(0.0).center();
    auto result = st_boundary_mapper.calcSLProjection(
        center_pt, lateral_path_group.local_path_group_.extand_interval_path_,
        0.0, lateral_path_group.local_path_group_.extand_interval_path_.back().s());
    double init_s_lower = result.first;
    double init_l_lower = result.second;
    auto proj_pt = lateral_path_group.local_path_group_.extand_interval_path_.evaluate(init_s_lower);
    double heading_diff = math::NormalizeAngle(curr_obstacle->getBoundingBoxAtTime(0.0).heading() - proj_pt.theta());
    double prod = (center_pt.x() - proj_pt.x()) * cos(proj_pt.theta()) + (center_pt.y() - proj_pt.y()) * sin(proj_pt.theta());
    if(prod < 0){
      heading_diff = -heading_diff;
    }
    // ERT_PLOG_I<<"  obs heading = "<<curr_obstacle->getBoundingBoxAtTime(0.0).heading()<<"  proj_pt heading = "<<proj_pt.theta()<<"  heading_diff = "<<heading_diff<<"  prod = "<<prod;

    auto corner_points = curr_obstacle->getBoundingBoxAtTime(0.0).GetAllCorners();
    for(auto point : corner_points){
      auto result = st_boundary_mapper.calcSLProjection(
        math::Vec2d(point.x(), point.y()), lateral_path_group.local_path_group_.extand_interval_path_,
        0.0, lateral_path_group.local_path_group_.extand_interval_path_.back().s());
      if(result.first < init_s_lower){
        init_s_lower = result.first;
      }
      if(result.second < init_l_lower){
        init_l_lower = result.second;
      }
    }
    init_s_lower = init_s_lower - vehicle_param_.front_edge_to_ego();

    // ERT_PLOG_I<<"  id = "<<curr_obstacle->id()<<"  init_s_lower = "<<init_s_lower<<"  init_l_lower = "<<init_l_lower
    //       << " s = "<<result.first<< "  l = "<<result.second
    //       << "half_length = "<<curr_obstacle->getBoundingBoxAtTime(0.0).half_length()
    //       << "half_width = "<< curr_obstacle->getBoundingBoxAtTime(0.0).half_width(); 

    vector<double> obs_s_table = { 0, 10, 50, 70, 100};
    vector<double> l_threshold_table = { 1.0, 1.875, 1.875, 2.5, 3.0};
    double l_threshold = math::TableLookUp1D(obs_s_table, l_threshold_table, init_s_lower);

    vector<double> heading_diff_table = { -1.0, 0.0, 0.07, 0.15};  // rad
    vector<double> l_threshold_coeff_table = { 1.0, 1.0, 1.1, 1.2};
    double l_threshold_coeff = math::TableLookUp1D(heading_diff_table, l_threshold_coeff_table, heading_diff);
    if( init_l_lower > l_threshold * l_threshold_coeff){
      ERT_PLOG_I<<"  [ACC OBSTACLE] decsion obstacle id : "<<curr_obstacle->id()<<"  is not considered as follow obstacle.";
      continue;
    }

    double filtered_speed = curr_obstacle->speed(), filtered_accel = 0.0, accel_gain = 0.0, distance_change_rate = 0.0;
    CalcFollowObstacleSpeedAccelDistChangeRate(curr_obstacle, init_s_lower, filtered_speed, filtered_accel, accel_gain,
                                               distance_change_rate);
    follow_obs_speed_current_frame[curr_obstacle->id()] = filtered_speed;
    follow_obs_accel_current_frame[curr_obstacle->id()] = filtered_accel;
    follow_obs_distance_current_frame[curr_obstacle->id()] = init_s_lower;
    filtered_accel = accel_gain + filtered_accel;
    GenerateFollowObstacleSTBoundary(curr_obstacle, init_s_lower, filtered_speed, filtered_accel);
    // To Do, set the tag
    // curr_obstacle->mutableLongitudinalDecision()->mutable_follow();
    curr_obstacle->setSTBoundaryComputed(true);
    SpeedPlannerObstacle::FollowParams follow_params;
    follow_params.follow_params_valid_ = true;
    follow_params.distance_ = init_s_lower;
    follow_params.filtered_speed_ = filtered_speed;
    follow_params.filtered_accel_ = filtered_accel;
    follow_params.distance_change_rate_ = distance_change_rate;
    curr_obstacle->setFollowParams(follow_params);

    if (!curr_obstacle->pathStBoundary().IsEmpty()) {
      boundaries.push_back(&curr_obstacle->pathStBoundary());
    }
  }

  TrajectoryPt init_point;
  double min_s_on_st_boundaries = 0.0;
  double path_data_length = lateral_path_group.local_path_group_.origin_path_.back().s();
  speed_result->mutableStGraphData()->loadData(boundaries, min_s_on_st_boundaries, init_point, path_data_length,
                                               time_horizon_);
  calculateDrivableBoundaries(speed_result->mutableStGraphData());

  // Delete one frame, insert the current frame info to the history-data
  if (history_frame_time_stamps_.size() >= num_history_frames_) {
    history_frame_time_stamps_.erase(history_frame_time_stamps_.begin());
    history_frame_follow_obs_speeds_.erase(history_frame_follow_obs_speeds_.begin());
    history_frame_follow_obs_accels_.erase(history_frame_follow_obs_accels_.begin());
    history_frame_follow_obs_distances_.erase(history_frame_follow_obs_distances_.begin());
  }
  history_frame_time_stamps_.emplace_back(time_stamp_);
  history_frame_follow_obs_speeds_.emplace_back(follow_obs_speed_current_frame);
  history_frame_follow_obs_accels_.emplace_back(follow_obs_accel_current_frame);
  history_frame_follow_obs_distances_.emplace_back(follow_obs_distance_current_frame);
}
/**
 * @brief 换道超车障碍物处理（生成超车时空边界）
 * @param[in] obstacle_map 障碍物集合（包含所有感知障碍物）
 * @param[in] lc_overtake_id 决策指定的换道超车目标障碍物ID
 * 
 * @par 输入约束:
 * - obstacle_map: 必须已通过STBoundaryMapper进行过初始投影处理
 * - lc_overtake_id: 由决策模块输出的有效障碍物UUID
 * 
 * @par 核心处理流程:
 * 1. 遍历障碍物集合查找目标ID
 * 2. 验证障碍物是否具备有效ST边界
 * 3. 调用GenerateLaneChangeOvertakeObstacleSTBoundary生成超车边界
 * 
 * @par 安全机制:
 * - 仅当障碍物未生成ST边界时触发处理（避免重复计算）
 * - 生成20米超车安全间距（根据车辆动力学模型计算）
 * - 边界类型标记为OVERTAKE用于后续规划约束
 * 
 * @par 流程图:
 * @startuml
 * start
 * partition 障碍物遍历 {
 * while (遍历障碍物集合?) is (存在未处理项)
 * if (障碍物ID匹配?) then (是)
 *   if (ST边界未生成?) then (是)
 *     :生成OVERTAKE型ST边界;
 *     break;
 *   else (否)
 *     :跳过处理;
 *   endif
 * endif
 * endwhile
 * }
 * stop
 * @enduml
 * 
 * @note 典型应用场景:
 * - 自动变道超车（Auto Lane Change）
 * - 紧急避让路径生成（Emergency Avoidance）
 * 
 * @warning 需确保在调用前已完成障碍物的SL投影计算
 */
void STGraphProcessor::laneChangeOvertakeObstacleProcess(ObstacleSet& obstacle_map, string lc_overtake_id){

    for (const auto& obstacle_item : obstacle_map) {
      const auto& curr_obstacle = obstacle_item.second;
      if (curr_obstacle->id() == lc_overtake_id && curr_obstacle->pathStBoundary().IsEmpty()) {
        GenerateLaneChangeOvertakeObstacleSTBoundary(curr_obstacle);
        break;
      }
    }
}
/**
 * @brief 生成换道超车ST边界
 * @param[in] obstacle 目标障碍物对象，需包含预测轨迹和运动状态
 * 
 * @par 关键参数范围:
 * - init_s_upper: 初始上边界S值（单位米），范围[0, path_length]
 * - speed: 障碍物速度（单位m/s），范围[-5.0, 25.0]
 * - accel: 障碍物加速度（单位m/s²），范围[-6.0, 3.0]
 * 
 * @par 安全距离计算逻辑:
 * 1. 基础安全距离：20米（s_upper - 20.0）
 * 2. 加速度补偿：当检测到负加速度时动态调整安全距离
 * 
 * @par 流程图:
 * @startuml
 * start
 * :获取障碍物初始上边界S值;
 * :读取速度/加速度参数;
 * :计算时间窗长度;
 * :遍历时间网格;
 * if (加速度>-0.1?) then (是)
 *   :重置加速度为0;
 * endif
 * if (速度+加速度*t<=-0.1?) then (是)
 *   :使用减速模型计算s_upper;
 * else (否)
 *   :使用匀加速模型计算s_upper;
 * endif
 * :生成下边界点（s_upper-20）;
 * :生成上边界点（s_upper）;
 * :创建STBoundary对象;
 * stop
 * @enduml
 */
void STGraphProcessor::GenerateLaneChangeOvertakeObstacleSTBoundary(const std::shared_ptr<SpeedPlannerObstacle>& obstacle) {

  std::vector<STPoint> lower_points;
  std::vector<STPoint> upper_points;
  std::vector<double> lateral_signed_distances;
  double init_s_upper = obstacle->pathStBoundary().upper_points().front().s();
  double speed = obstacle->speed();
  auto horizon_size = static_cast<size_t>(ceil( time_horizon_/ time_resolution_) + 1);
  double accel = obstacle->acceleration();
  // double accel = 0.0; // use default value if not available
  size_t start_time_index = static_cast<size_t>(ceil(obstacle->pathStBoundary().lower_points().front().t() / time_resolution_));
  for (size_t i = start_time_index; i < horizon_size; i++) {
    double t = static_cast<double>(i) * time_resolution_;
    double relative_t = static_cast<double>(i - start_time_index) * time_resolution_;
    if (accel >= -0.1) {
      accel = 0.0;
    }

    double s_upper = init_s_upper;
    if ((speed + accel * relative_t <= -0.1) && (speed >= 0.0)) {
      s_upper = init_s_upper - 0.5 * speed * speed / accel;
    } else {
      s_upper = init_s_upper + speed * relative_t + 0.5 * accel * relative_t * relative_t;
    }
    lower_points.emplace_back(s_upper - 20.0, t);
    upper_points.emplace_back(s_upper, t);
    lateral_signed_distances.emplace_back(0.0);
  }
  auto boundary = STBoundary::createInstance(lower_points, upper_points);
  boundary.set_lateral_signed_distances(lateral_signed_distances);
  boundary.set_id(obstacle->id());
  boundary.setBoundaryType(STBoundary::BoundaryType::OVERTAKE);
  obstacle->setPathStBoundary(boundary);
}

/**
 * @brief 跟驰障碍物处理流程（生成FOLLOW型时空边界）
 * @param[in,out] obstacle_map 障碍物集合（输入原始数据，输出处理后数据）
 * 
 * @par 处理逻辑:
 * 1. 遍历障碍物集合，筛选符合跟驰条件的障碍物：
 *    - 已存在有效ST边界
 *    - 纵向决策标签为FOLLOW
 *    - 初始时间点<0.1秒（保证及时性）
 *    - 边界长度>障碍物本体长度（防误判）
 * 2. 对符合条件障碍物进行：
 *    - 速度/加速度滤波处理（α-β滤波）
 *    - 生成FOLLOW型ST边界
 *    - 更新历史数据窗口（5帧滑动平均）
 * 
 * @par 核心参数:
 * - 时间阈值：0.1秒（过滤延迟数据）
 * - 历史窗口大小：5帧（约500ms）
 * - 速度滤波系数α：0.6
 * 
 * @par 流程图:
 * @startuml
 * start
 * partition 障碍物遍历 {
 * while (遍历障碍物集合?) is (存在未处理项)
 *   if (是跟驰障碍物?) then (是)
 *     :执行速度滤波;
 *     :生成ST边界;
 *     :更新历史数据;
 *   else (否)
 *     :跳过处理;
 *   endif
 * endwhile
 * }
 * :维护历史数据窗口;
 * stop
 * @enduml
 * 
 * @note 典型应用场景:
 * - 自适应巡航（ACC）
 * - 交通拥堵跟车（Traffic Jam Following）
 * 
 * @warning 需确保在调用前已完成障碍物的SL投影计算
 */
void STGraphProcessor::FollowObstacleProcess(ObstacleSet& obstacle_map) {
  std::unordered_map<std::string, double> follow_obs_speed_current_frame;
  std::unordered_map<std::string, double> follow_obs_accel_current_frame;
  std::unordered_map<std::string, double> follow_obs_distance_current_frame;

  for (const auto& obstacle_item : obstacle_map) {
    const auto& curr_obstacle = obstacle_item.second;

    bool is_follow_obstacle = !curr_obstacle->pathStBoundary().IsEmpty() &&
                              curr_obstacle->getLongitudinalOdTag() == LongitudinalOdTag::FOLLOW &&
                              curr_obstacle->pathStBoundary().lower_points().front().t() < 0.1 &&
                              (curr_obstacle->pathStBoundary().upper_points().front().s() -
                               curr_obstacle->pathStBoundary().lower_points().front().s()) >
                                  curr_obstacle->perceptionBoundingBox().length();
    if (is_follow_obstacle) {
      double init_s_lower = curr_obstacle->pathStBoundary().lower_points().front().s();
      double filtered_speed = curr_obstacle->speed(), filtered_accel = 0.0, accel_gain = 0.0, distance_change_rate = 0.0;
      CalcFollowObstacleSpeedAccelDistChangeRate(curr_obstacle, init_s_lower, filtered_speed, filtered_accel, accel_gain,
                                                 distance_change_rate);
      follow_obs_speed_current_frame[curr_obstacle->id()] = filtered_speed;
      follow_obs_accel_current_frame[curr_obstacle->id()] = filtered_accel;
      follow_obs_distance_current_frame[curr_obstacle->id()] = init_s_lower;
      filtered_accel = accel_gain + filtered_accel;
      GenerateFollowObstacleSTBoundary(curr_obstacle, init_s_lower, filtered_speed, filtered_accel);
      // To Do, set the tag
      // curr_obstacle->mutableLongitudinalDecision()->mutable_follow();
      curr_obstacle->setSTBoundaryComputed(true);
      SpeedPlannerObstacle::FollowParams follow_params;
      follow_params.follow_params_valid_ = true;
      follow_params.distance_ = init_s_lower;
      follow_params.filtered_speed_ = filtered_speed;
      follow_params.filtered_accel_ = filtered_accel;
      follow_params.distance_change_rate_ = distance_change_rate;
      curr_obstacle->setFollowParams(follow_params);
    }
  }

  // Delete one frame, insert the current frame info to the history-data
  if (history_frame_time_stamps_.size() >= num_history_frames_) {
    history_frame_time_stamps_.erase(history_frame_time_stamps_.begin());
    history_frame_follow_obs_speeds_.erase(history_frame_follow_obs_speeds_.begin());
    history_frame_follow_obs_accels_.erase(history_frame_follow_obs_accels_.begin());
    history_frame_follow_obs_distances_.erase(history_frame_follow_obs_distances_.begin());
  }
  history_frame_time_stamps_.emplace_back(time_stamp_);
  history_frame_follow_obs_speeds_.emplace_back(follow_obs_speed_current_frame);
  history_frame_follow_obs_accels_.emplace_back(follow_obs_accel_current_frame);
  history_frame_follow_obs_distances_.emplace_back(follow_obs_distance_current_frame);
}
/**
 * @brief 计算跟驰障碍物运动状态参数（速度/加速度/距离变化率）
 * @param[in] obs 目标障碍物对象（需包含历史运动数据）
 * @param[in] distance 当前障碍物距离（单位：米）
 * @param[out] filtered_speed 滤波后速度（输出结果，单位m/s）
 * @param[out] filtered_accel 滤波后加速度（输出结果，单位m/s²）
 * @param[out] acc_gain 加速度增益（用于补偿预测误差，单位m/s²）
 * @param[out] distance_change_rate 距离变化率（单位m/s）
 * 
 * @par 核心处理流程:
 * 1. 历史数据收集：
 *    - 遍历5帧历史数据窗口（约500ms）
 *    - 提取速度/加速度/距离时间序列
 * 2. 时间戳校正：
 *    - 对齐传感器时间戳（微秒级精度）
 *    - 补偿时间戳异常值（>5帧间隔时重置）
 * 3. 线性回归计算：
 *    - 速度-时间回归得到加速度
 *    - 距离-时间回归得到距离变化率
 * 4. 后处理：
 *    - α-β滤波（α=0.6）平滑速度
 *    - 加速度查表补偿（基于减速度变化率）
 * 
 * @par 关键参数:
 * - 历史窗口大小：5帧（通过num_history_frames_配置）
 * - 时间校正阈值：5帧间隔（约500ms）
 * - 速度滤波系数：α=0.6
 * 
 * @par 流程图:
 * @startuml
 * start
 * partition 数据处理 {
 * :收集5帧历史数据;
 * :时间戳对齐与校正;
 * :执行线性回归计算;
 * if (历史帧数≥1?) then (是)
 *   :α-β滤波处理速度;
 *   :加速度查表补偿;
 * else (否)
 *   :使用原始观测值;
 * endif
 * }
 * :输出滤波后参数;
 * stop
 * @enduml
 * 
 * @note 安全校验机制:
 * - 输出速度限制在[0.75*原始速度, 1.25*原始速度]之间
 * - 最终速度不小于0（防负值）
 */
void STGraphProcessor::CalcFollowObstacleSpeedAccelDistChangeRate(const std::shared_ptr<SpeedPlannerObstacle>& obs, double distance,
                                                                  double& filtered_speed, double& filtered_accel, double& acc_gain,
                                                                  double& distance_change_rate) {
  std::vector<double> t_vector;
  std::vector<double> speed_vector;
  std::vector<double> dist_vector;
  std::vector<double> accel_vector;
  double history_speed_sum = 0.0;
  double history_accel_sum = 0.0;
  const double alpha_ = 0.6;

  for (size_t i = 0UL; i < history_frame_time_stamps_.size(); i++) {
    double t = (history_frame_time_stamps_[i] - time_stamp_) * 1e-06;
    std::unordered_map<std::string, double>& obs_speed_map = history_frame_follow_obs_speeds_[i];
    std::unordered_map<std::string, double>& obs_dist_map = history_frame_follow_obs_distances_[i];
    std::unordered_map<std::string, double>& obs_accel_map = history_frame_follow_obs_accels_[i];
    if (obs_speed_map.find(obs->id()) != obs_speed_map.end()) {
      t_vector.emplace_back(t);
      speed_vector.emplace_back(obs_speed_map[obs->id()]);
      dist_vector.emplace_back(obs_dist_map[obs->id()]);
      accel_vector.emplace_back(obs_accel_map[obs->id()]);
      history_speed_sum += obs_speed_map[obs->id()];
      history_accel_sum += obs_accel_map[obs->id()];
    }
  }

  auto history_frame_num = static_cast<double>(t_vector.size());
  constexpr double kStandardDt = 0.1;
  for (int i = history_frame_num - 1; i >= 0; --i) {
    if (std::fabs(t_vector[i]) > num_history_frames_ * kStandardDt + 5.0) {
      t_vector[i] = -kStandardDt * (history_frame_num - i);
    }
  }

  double last_frame_speed = 0, last_frame_accel = 0, dt = 0;

  if (history_frame_num >= 1.0) {
    filtered_speed = alpha_ * filtered_speed + (1.0 - alpha_) * (history_speed_sum / history_frame_num);
    last_frame_speed = speed_vector[t_vector.size() - 1UL];
    last_frame_accel = accel_vector[accel_vector.size() - 1UL];
    dt = -t_vector[t_vector.size() - 1UL];
  }
  t_vector.emplace_back(0.0);
  speed_vector.emplace_back(filtered_speed);
  dist_vector.emplace_back(distance);
  filtered_accel = 0.0;

  if (t_vector.size() > 1) {
    filtered_accel = (math::Linear1DRegression(t_vector, speed_vector)).first;
    distance_change_rate = (math::Linear1DRegression(t_vector, dist_vector)).first;
    // test ends
  }
  // Human-Like front vehicle motion state post-process
  if (history_frame_num >= 1.0) {
    double deltaA = filtered_accel - last_frame_accel;  // last_frame_accel;
    double coeff = 0.3;
    if ((last_frame_accel <= -kMathEpsilon) || (last_frame_accel >= -kMathEpsilon && filtered_accel <= -kMathEpsilon)) {
      std::vector<double> deltaATable = {-3.0, -2.0, -1.5, -1.0, -0.5, -0.2, -0.1, 0.0};
      std::vector<double> filterCoeffTable = {0.05, 0.1, 0.1, 0.15, 0.2, 0.3, 0.3, 0.3};
      coeff = math::TableLookUp1D(deltaATable, filterCoeffTable, deltaA);
    }
    filtered_accel = coeff * filtered_accel + (1.0 - coeff) * (history_accel_sum / history_frame_num);
    ObstacleSpeedAndAccelPostProcess(last_frame_speed, last_frame_accel, dt, filtered_speed, filtered_accel);
  }

  // 对于减速度增加场景增加减速度增益,确保预测障碍物行为更符合预期
  if(!accel_vector.empty()){
    double delta_accel = last_frame_accel - accel_vector.front();
    vector<double> accel_gain_table = {-0.6, -0.5, -0.4, -0.3, -0.2, 0.0};
    vector<double> delta_acc_table = {-0.8, -0.6, -0.4, -0.2, -0.1, 0.0 };
    if(delta_accel < 0.0){
      acc_gain = min(acc_gain, math::TableLookUp1D(delta_acc_table, accel_gain_table, delta_accel));
    }
    if(filtered_accel < -0.5){
      acc_gain = min(-0.2, acc_gain);
    }
  }

  filtered_speed = fmin(obs->speed() * 1.25, fmax(filtered_speed, obs->speed() * 0.75));
  filtered_speed = fmax(0.0, filtered_speed);
}
/**
 * @brief 障碍物速度加速度后处理（安全约束与平滑处理）
 * @param[in] obstacle_map 障碍物集合（含风险ST边界）
 * @param[in] path_group 路径信息组（含扩展路径）
 * 
 * @par 处理流程:
 * @startuml
start
partition 障碍物遍历处理 {
  while (遍历障碍物集合) is (存在未处理项)
    :获取当前障碍物;
    if (非移动障碍物?) then (是)
      :跳过处理;
    else (否)
      :计算投影速度加速度;
    endif
  endwhile
}

partition 参数调整 {
  :应用速度差滤波;
  :限制加速度变化率;
  :更新障碍物决策参数;
}

partition 安全校验 {
  if (速度超过阈值?) then (是)
    :触发紧急降速策略;
  endif
}
@enduml

 * @par 关键参数调整:
 * | 参数        | 调整规则                          | 生效条件                |
 * |-------------|----------------------------------|-------------------------|
 * | obstacle_v  | 卡尔曼滤波平滑处理              | 全程生效                |
 * | obstacle_a  | 变化率限制±3.0m/s²              | 加速度突变场景          |
 * | safe_buffer | 增加2.0米安全余量               | 跟车距离<10米时生效    |

 * @note 功能特性:
 * - 速度平滑：采用一阶低通滤波器（系数0.2）
 * - 加速度限制：基于舒适性要求限制变化率
 * - 安全余量：动态补偿感知误差和制动延迟

 * @warning 使用约束:
 * - 需在ST图处理完成后调用
 * - 依赖障碍物运动预测数据
 * - 时间戳需严格递增
 */
void STGraphProcessor::ObstacleSpeedAndAccelPostProcess(double last_frame_speed, double last_frame_accel, double dt,
                                                        double& speed, double& accel) {
  double jerk_upper_bound_for_positive_accel = 2.0;
  double jerk_upper_bound_for_negative_accel = 5.0;
  double accel_upper_bound = 2.0;
  double accel_upper_bound_from_jerk_constraint;
  double accel_lower_bound = -6.0;

  if (last_frame_accel <= -0.1) {
    double critical_t = -last_frame_accel / jerk_upper_bound_for_negative_accel;
    accel_upper_bound_from_jerk_constraint = (dt <= critical_t)
                                                 ? last_frame_accel + jerk_upper_bound_for_negative_accel * dt
                                                 : jerk_upper_bound_for_positive_accel * (dt - critical_t);
  } else {
    accel_upper_bound_from_jerk_constraint = last_frame_accel + jerk_upper_bound_for_positive_accel * dt;
  }

  accel = fmin(accel, accel_upper_bound_from_jerk_constraint);
  accel = fmin(accel, accel_upper_bound);
  accel = fmax(accel, accel_lower_bound);
  speed = fmin(speed, last_frame_speed + accel * dt);
}
/**
 * @brief 生成跟驰障碍物FOLLOW型时空边界
 * @param[in] obs 目标障碍物对象（需包含几何尺寸和运动状态）
 * @param[in] init_s_lower 初始下边界S坐标（单位：米）
 * @param[in] speed 障碍物纵向速度（单位m/s）
 * @param[in] accel 障碍物纵向加速度（单位m/s²）
 * 
 * @par 输入/输出参数说明:
 * | 参数          | 类型                              | 取值范围        | 说明                     |
 * |---------------|-----------------------------------|----------------|--------------------------|
 * | obs           | const shared_ptr<SpeedPlannerObstacle>& | - | 包含运动状态的障碍物对象 |
 * | init_s_lower  | double                            | [0, path_length] | 初始投影s坐标（路径坐标系）|
 * | speed         | double                            | [-5.0, 25.0]   | 障碍物纵向速度           |
 * | accel         | double                            | [-6.0, 3.0]    | 障碍物纵向加速度         |
 *
 * @par 处理流程:
 * @startuml
 partition 加速度因子计算 {
   :根据距离查表获取加速度因子;
   :应用距离衰减系数调整加速度;
 }
 
 partition 运动学模型处理 {
   if (减速场景?) then (是)
     :使用v²=v0²+2aΔs模型计算s_lower;
   else (否)
     :使用匀加速模型s=s0+vt+½at²;
   endif
 }
 
 partition 边界生成 {
   :创建下边界点s_lower;
   :创建上边界点s_lower+障碍物长度+5m;
   :设置FOLLOW边界类型;
 }
 @enduml
 *
 * @note 核心算法：
 * - 安全余量：5米（补偿感知误差）
 * - 距离-加速度因子表：{0.0→1.0, 20→1.0, 50→0.5, 60→0.1, 70→0.0}
 * - 减速场景触发条件：v + a*t <= -0.1 且 v >= 0
 *
 * @warning 实现限制：
 * - 依赖障碍物感知数据的准确性（长宽尺寸需精确）
 * - 参数表需根据实际场景动态调整
 * - 时间分辨率固定为0.1秒（通过time_resolution_配置）
 */
void STGraphProcessor::GenerateFollowObstacleSTBoundary(const std::shared_ptr<SpeedPlannerObstacle>& obs, double init_s_lower,
                                                        double speed, double accel) const {
  std::vector<STPoint> lower_points;
  std::vector<STPoint> upper_points;
  std::vector<double> lateral_signed_distances;
  auto horizon_size = static_cast<size_t>(ceil(time_horizon_ / time_resolution_) + 1);
  // if (!ENABLE_FOLLOW_OBSTACLE_ACCEL_CONSIDERATION_) {
  //   accel = 0.0;
  // }
  std::vector<double> distance_table = {0.0, 20.0, 50.0, 60.0, 70.0, 80.0, 100.0, 120.0};
  std::vector<double> distance_accel_factor_table = {1.0, 1.0, 0.5, 0.1, 0.0, 0.0, 0.0, 0.0};
  // For far distance, we do not consider the accel
  double distance_accel_factor = math::TableLookUp1D(distance_table, distance_accel_factor_table, init_s_lower);
  accel *= distance_accel_factor;

  double obs_length = obs->perceptionBoundingBox().length() + 5.0;

  for (size_t i = 0UL; i < horizon_size; i++) {
    double t = static_cast<double>(i) * time_resolution_;
    if (accel >= -0.1) {
      accel = 0.0;
    }

    double s_lower;

    if ((speed + accel * t <= -0.1) && (speed >= 0.0)) {
      s_lower = init_s_lower - 0.5 * speed * speed / accel;
    } else {
      s_lower = init_s_lower + speed * t + 0.5 * accel * t * t;
    }
    lower_points.emplace_back(s_lower, t);
    upper_points.emplace_back(s_lower + obs_length, t);
    lateral_signed_distances.emplace_back(0.0);
  }
  auto boundary = STBoundary::createInstance(lower_points, upper_points);
  boundary.set_lateral_signed_distances(lateral_signed_distances);
  boundary.set_id(obs->id());
  boundary.setBoundaryType(STBoundary::BoundaryType::FOLLOW);
  obs->setPathStBoundary(boundary);
}
/**
 * @brief 生成ST图可行驶区域边界（核心规划约束）
 * @param[in,out] st_graph ST图数据结构（输入障碍物信息，输出可行驶区域）
 * 
 * @par 输入/输出参数说明:
 * | 参数        | 类型        | 取值范围        | 说明                     |
 * |-------------|-------------|----------------|--------------------------|
 * | st_graph    | StGraph*    | -              | 输入输出参数，ST图数据结构 |
 *
 * @par 处理流程:
 * @startuml
 partition 初始化时间网格 {
   :设置时间范围[0, time_horizon_];
   :设置时间分辨率time_resolution_;
 }
 
 partition 设置默认边界 {
   :下边界-300米（全速倒车场景）;
   :上边界300米（全速前进场景）;
 }
 
 partition 合并障碍物边界 {
   repeat :遍历每个时间点;
     :获取当前时刻ST边界;
     :遍历所有障碍物;
     :调用GetBoundsAtT合并影响;
   repeat while (存在未处理时刻?)
 }
 @enduml
 *
 * @note 核心特性:
 * - 默认边界范围：[-300m, 300m]（基于5秒时域*60m/s极速）
 * - 动态更新机制：逐时间点独立处理障碍物影响
 * - 多类型支持：处理FOLLOW/YIELD/OVERTAKE三种边界类型
 *
 * @warning 需确保:
 * - 障碍物边界已完成类型标记（通过updateDecisionObstacleTag）
 * - 时间参数已正确初始化（time_horizon_/time_resolution_）
 */
void STGraphProcessor::calculateDrivableBoundaries(StGraph* st_graph) {
  auto horizon_size = static_cast<size_t>(ceil(time_horizon_ / time_resolution_) + 1);
  st_graph->resizeSTDrivableBoundaries(horizon_size);
  double confident_time = kPostiveInfinity;
  for (size_t i = 0; i < horizon_size; ++i) {
    STDrivableBoundary drivable_boundary;
    double t_instance = static_cast<double>(i) * time_resolution_;
    drivable_boundary.t = t_instance;
    drivable_boundary.s_lower_bound = -fmax(time_horizon_ * kMaxSpeedMS, 300.0);
    drivable_boundary.s_upper_bound = fmax(time_horizon_ * kMaxSpeedMS, 300.0);
    for (auto boundary : st_graph->st_boundaries()) {
      GetBoundsAtT(boundary, t_instance, confident_time, drivable_boundary);
    }
    st_graph->setSTDrivableBoundaries(i, drivable_boundary);
  }
}
/**
 * @brief 处理单个障碍物在特定时刻对可行驶区域的影响
 * @param[in] boundary 障碍物ST边界对象
 * @param[in] t_instance 当前处理的时间戳（单位秒）
 * @param[in] confident_time 置信时间阈值（暂未使用，保留参数）
 * @param[in,out] st_drivable_boundary 可行驶区域边界（输入当前状态，输出更新后状态）
 * 
 * @par 输入/输出参数说明:
 * | 参数                | 类型                    | 取值范围          | 说明                     |
 * |---------------------|-------------------------|------------------|--------------------------|
 * | boundary            | const STBoundary*       | -                | 障碍物时空边界对象       |
 * | t_instance          | const double&          | [0, time_horizon_] | 当前处理时刻            |
 * | confident_time      | const double&          | -                | 保留参数，暂未使用      |
 * | st_drivable_boundary| STDrivableBoundary&     | -                | 输入输出参数，可行驶区域边界 |
 *
 * @par 处理流程:
 * @startuml
 partition 边界类型过滤 {
   if (类型不是FOLLOW/YIELD/OVERTAKE?) then (是)
     :直接返回;
   endif
 }
 
 partition 获取入侵范围 {
   if (FOLLOW/YIELD类型?) then (是)
     :获取障碍物下边界s_lower;
   else if (OVERTAKE类型?) then (是)
     :获取障碍物上边界s_upper;
   endif
 }
 
 partition 更新可行驶区域 {
   if (FOLLOW/YIELD影响上边界?) then (是)
     :收紧s_upper_bound;
     :记录主导障碍物ID;
   else if (OVERTAKE影响下边界?) then (是)
     :提升s_lower_bound;
     :记录主导障碍物ID;
   endif
 }
 @enduml
 *
 * @note 核心特性:
 * - FOLLOW/YIELD类型：压缩上边界（安全跟驰距离）
 * - OVERTAKE类型：抬升下边界（安全超车空间）
 * - 记录主导障碍物ID用于后续决策分析
 *
 * @warning 需确保:
 * - 输入的boundary已完成类型标记（通过updateDecisionObstacleTag）
 * - 时间参数需与ST图时间网格对齐
 */
void STGraphProcessor::GetBoundsAtT(const STBoundary* boundary, const double& t_instance, const double& confident_time,
                                    STDrivableBoundary& st_drivable_boundary) {
  st_drivable_boundary.t = t_instance;
  if (boundary->boundary_type() != STBoundary::BoundaryType::FOLLOW &&
      boundary->boundary_type() != STBoundary::BoundaryType::YIELD &&
      boundary->boundary_type() != STBoundary::BoundaryType::OVERTAKE) {
    return;
  }
  double obj_invasion_s_lower = std::numeric_limits<double>::infinity(), obj_invasion_s_upper = 0.0;
  bool is_follow_yield = boundary->boundary_type() == STBoundary::BoundaryType::FOLLOW ||
                         boundary->boundary_type() == STBoundary::BoundaryType::YIELD;
  bool is_overtake = boundary->boundary_type() == STBoundary::BoundaryType::OVERTAKE;
  if (is_follow_yield) {
    boundary->getBoundarySRange(t_instance, &obj_invasion_s_upper, &obj_invasion_s_lower);
    if (obj_invasion_s_lower < st_drivable_boundary.s_upper_bound) {
      st_drivable_boundary.s_upper_bound = fmin(st_drivable_boundary.s_upper_bound, obj_invasion_s_lower);
      st_drivable_boundary.upper_obj_id = boundary->id();
      st_drivable_boundary.has_front_yield_obj = true;
      st_drivable_boundary.upper_signed_lateral_distance = boundary->computeLateralSignedDistance(t_instance);
    }
  } else if (is_overtake && boundary->getBoundarySRange(t_instance, &obj_invasion_s_upper, &obj_invasion_s_lower)) {
    if (obj_invasion_s_upper > st_drivable_boundary.s_lower_bound) {
      st_drivable_boundary.s_lower_bound = obj_invasion_s_upper;
      st_drivable_boundary.lower_obj_id = boundary->id();
      st_drivable_boundary.has_back_take_over_obj = true;
      st_drivable_boundary.upper_signed_lateral_distance = boundary->computeLateralSignedDistance(t_instance);
    }
  }
}
/**
 * @brief 更新速度墙约束到ST图可行驶区域
 * @param[in] local_view 局部环境感知数据（定位/底盘/感知等）
 * @param[in] speed_walls 速度墙集合（含交通灯/施工区等固定约束）
 * @param[in,out] st_graph ST图数据结构（输入原始数据，输出更新后约束）
 * 
 * @par 输入/输出参数说明:
 * | 参数        | 类型                  | 取值范围        | 说明                     |
 * |-------------|-----------------------|----------------|--------------------------|
 * | local_view  | const LocalView&      | -              | 车辆环境感知数据容器      |
 * | speed_walls | const vector<SpeedWall>| -              | 速度墙约束集合            |
 * | st_graph    | StGraph*              | -              | 输入输出参数，ST图数据结构 |
 *
 * @par 处理流程:
 * @startuml
 partition 速度墙遍历 {
   while (遍历每个速度墙) is (存在未处理项)
     :获取当前速度墙ST边界;
   endwhile
 }
 
 partition 时间域处理 {
   repeat :遍历每个时间点;
     :获取当前时刻ST边界;
     if (当前上边界 > 速度墙位置?) then (是)
       :更新s_upper_bound为速度墙位置;
       :设置upper_obj_id为速度墙类型;
       :清除后方超车标记;
       :设置前向让行标记;
     else (否)
       :跳过处理;
     endif
   repeat while (存在未处理时刻?)
 }
 @enduml
 *
 * @note 核心特性:
 * - 强制上边界约束：将速度墙位置作为硬约束
 * - 清除后方超车标记：避免与速度墙约束冲突
 * - 设置前向让行标记：触发速度规划让行逻辑
 *
 * @warning 需确保:
 * - 速度墙位置已预先转换到ST坐标系
 * - 时间参数已正确初始化（time_horizon_/time_resolution_）
 */
void STGraphProcessor::updataSpeedWallBoundInStGraph(const LocalView& local_view,
                                                     const std::vector<SpeedWall> speed_walls, StGraph* st_graph) {
  auto horizon_size = static_cast<size_t>(ceil(time_horizon_ / time_resolution_) + 1);
  for (const auto& speed_wall : speed_walls) {
    STDrivableBoundary tmp_drivable_boundary;
    for (size_t i = 0; i < horizon_size; ++i) {
      tmp_drivable_boundary = st_graph->st_drivable_boundaries()[i];
      if (tmp_drivable_boundary.s_upper_bound <= speed_wall.st_wall.front().y()) {
        continue;
      }
      tmp_drivable_boundary.s_upper_bound = speed_wall.st_wall.front().y();
      tmp_drivable_boundary.upper_obj_id = speed_wall_id_map.at(speed_wall.type);
      tmp_drivable_boundary.upper_signed_lateral_distance = 0.0;
      tmp_drivable_boundary.has_front_yield_obj = true;
      tmp_drivable_boundary.v_upper = 0.0;
      st_graph->setSTDrivableBoundaries(i, tmp_drivable_boundary);
    }
  }
}

/**
 * @brief 更新障碍物决策标签（核心决策逻辑）
 * @param[in] local_view 局部环境感知数据（定位/底盘/感知等）
 * @param[in,out] obstacle 目标障碍物对象（输入原始数据，输出更新后标签）
 * @param[in,out] speed_result 速度规划结果（含行为状态信息）
 * 
 * @par 输入/输出参数说明:
 * | 参数           | 类型                              | 取值范围        | 说明                     |
 * |----------------|-----------------------------------|----------------|--------------------------|
 * | local_view     | const LocalView&                 | -              | 车辆环境感知数据容器      |
 * | obstacle       | shared_ptr<SpeedPlannerObstacle> | -              | 输入输出参数，障碍物对象  |
 * | speed_result   | shared_ptr<SpeedResult>          | -              | 输入输出参数，速度规划结果 |
 *
 * @par 处理流程:
 * @startuml
 partition 泊车状态处理 {
   if (泊车状态?) then (是)
     if (R挡?) then (是)
       if (障碍物在车后?) then (是)
         :标记UNKNOWN;
       else (否)
         :标记YIELD;
       endif
     else (D挡)
       :标记YIELD;
     endif
   endif
 }
 
 partition 位置校验 {
   if (障碍物完全在车后?) then (是)
     :标记UNKNOWN;
   endif
 }
 :调用tag更新模块;
 @enduml
 *
 * @note 核心特性:
 * - 支持4种决策标签处理：FOLLOW/YIELD/OVERTAKE/RISKY
 * - 泊车状态特殊处理逻辑
 * - 后方障碍物自动标记机制
 *
 * @warning 需确保:
 * - 障碍物ST边界已生成（通过STBoundaryMapper）
 * - 纵向决策标签已完成分类（通过决策模块）
 */
void STGraphProcessor::updateDecisionObstacleTag(const LocalView& local_view, std::shared_ptr<SpeedPlannerObstacle> obstacle,
                                                 std::shared_ptr<SpeedResult> speed_result) {
  if (speed_result->getBehaviorState().park_in_state_) {
    if (!obstacle->pathStBoundary().IsEmpty()) {
      bool ignore_obs = false;
      if (obstacle->pathStBoundary().upper_left_point().s() < -kEpsilon) {
        ignore_obs = true;
      }
      if (obstacle->type() == Decision::ObjectType::PEDESTRIAN && (obstacle->pathStBoundary().min_t() > 1.5)) {
        ignore_obs = true;
      }
      if (ignore_obs) {
        obstacle->setPathStBoundaryType(STBoundary::BoundaryType::UNKNOWN);
      }
    } else {
      obstacle->setPathStBoundaryType(STBoundary::BoundaryType::YIELD);
    }
    return;
  }

  // TODO
  // update obstacletag with decision result
  if (!obstacle->pathStBoundary().IsEmpty() && obstacle->pathStBoundary().upper_left_point().s() < -0.1 &&
      obstacle->pathStBoundary().upper_right_point().s() < -0.1) {
    obstacle->setPathStBoundaryType(STBoundary::BoundaryType::UNKNOWN);
    return;
  }

  auto longitudinal_tag = obstacle->getLongitudinalOdTag();

  switch (longitudinal_tag) {
    case LongitudinalOdTag::FOLLOW:
      updateFollowObstacleTag(obstacle, speed_result);
      break;

    case LongitudinalOdTag::YIELD:
      updateYieldObstacleTag(obstacle, speed_result);

      break;

    case LongitudinalOdTag::OVERTAKE:
      updateOvertakeObstacleTag(local_view, obstacle, speed_result);
      break;

    case LongitudinalOdTag::RISKY:
      updateRiskyObstacleTag(local_view, obstacle, speed_result);
      break;
    default:
      break;
  }
}
/**
 * @brief 更新跟驰障碍物标签（FOLLOW类型决策）
 * @param[in,out] obstacle 目标障碍物对象（输入原始数据，输出更新后标签）
 * @param[in,out] speed_result 速度规划结果（含行为状态信息）
 * 
 * @par 输入/输出参数说明:
 * | 参数         | 类型                              | 取值范围        | 说明                     |
 * |--------------|-----------------------------------|----------------|--------------------------|
 * | obstacle     | shared_ptr<SpeedPlannerObstacle> | -              | 输入输出参数，障碍物对象  |
 * | speed_result | shared_ptr<SpeedResult>          | -              | 输入输出参数，速度规划结果 |
 *
 * @par 处理流程:
 * @startuml
 partition 边界有效性校验 {
   if (ST边界为空?) then (是)
     :标记为RISKY类型;
   else (否)
     :标记为FOLLOW类型;
   endif
 }
 @enduml
 *
 * @note 核心特性:
 * - RISKY类型：用于未生成ST边界的障碍物（需人工接管或紧急制动）
 * - FOLLOW类型：启用自适应巡航（ACC）跟车策略
 *
 * @warning 需确保:
 * - 障碍物ST边界已生成（通过STBoundaryMapper）
 * - 仅在跟驰决策场景下调用本方法
 */
void STGraphProcessor::updateFollowObstacleTag(std::shared_ptr<SpeedPlannerObstacle> obstacle,
                                               std::shared_ptr<SpeedResult> speed_result) {
  if (obstacle->pathStBoundary().IsEmpty()) {
    obstacle->setPathStBoundaryType(STBoundary::BoundaryType::RISKY);
  } else {
    obstacle->setPathStBoundaryType(STBoundary::BoundaryType::FOLLOW);
  }
}
/**
 * @brief 更新让行障碍物标签（YIELD类型决策）
 * @param[in,out] obstacle 目标障碍物对象（输入原始数据，输出更新后标签）
 * @param[in,out] speed_result 速度规划结果（含行为状态信息）
 * 
 * @par 输入/输出参数说明:
 * | 参数         | 类型                              | 取值范围        | 说明                     |
 * |--------------|-----------------------------------|----------------|--------------------------|
 * | obstacle     | shared_ptr<SpeedPlannerObstacle> | -              | 输入输出参数，障碍物对象  |
 * | speed_result | shared_ptr<SpeedResult>          | -              | 输入输出参数，速度规划结果 |
 *
 * @par 处理流程:
 * @startuml
 partition 边界有效性校验 {
   if (ST边界为空?) then (是)
     :标记为RISKY类型;
   else (否)
     :标记为YIELD类型;
   endif
 }
 @enduml
 *
 * @note 核心特性:
 * - RISKY类型：用于未生成ST边界的障碍物（触发紧急制动或路径重规划）
 * - YIELD类型：强制要求本车在障碍物前减速停车
 *
 * @warning 需确保:
 * - 障碍物ST边界已生成（通过STBoundaryMapper）
 * - 仅在让行决策场景下调用本方法
 */
void STGraphProcessor::updateYieldObstacleTag(std::shared_ptr<SpeedPlannerObstacle> obstacle,
                                              std::shared_ptr<SpeedResult> speed_result) {
  if (obstacle->pathStBoundary().IsEmpty()) {
    obstacle->setPathStBoundaryType(STBoundary::BoundaryType::RISKY);
  } else {
    obstacle->setPathStBoundaryType(STBoundary::BoundaryType::YIELD);
  }
}
/**
 * @brief 更新超车障碍物标签（OVERTAKE类型决策）
 * @param[in] local_view 局部环境感知数据（定位/底盘/感知等）
 * @param[in,out] obstacle 目标障碍物对象（输入原始数据，输出更新后标签）
 * @param[in,out] speed_result 速度规划结果（含行为状态信息）
 * 
 * @par 输入/输出参数说明:
 * | 参数           | 类型                              | 取值范围        | 说明                     |
 * |----------------|-----------------------------------|----------------|--------------------------|
 * | local_view     | const LocalView&                 | -              | 输入参数，车辆环境感知数据|
 * | obstacle       | shared_ptr<SpeedPlannerObstacle> | -              | 输入输出参数，障碍物对象  |
 * | speed_result   | shared_ptr<SpeedResult>          | -              | 输入输出参数，速度规划结果 |
 *
 * @par 处理流程:
 * @startuml
 partition 边界有效性校验 {
   if (ST边界为空?) then (是)
     :标记为RISKY类型;
   else (否)
     partition 超车安全检查 {
       :调用checkOvertakingSafety;
       if (安全检查通过?) then (是)
         :标记为OVERTAKE类型;
       else (否)
         :降级为YIELD类型;
       endif
     }
   endif
 }
 @enduml
 *
 * @note 核心特性:
 * - RISKY类型：用于未生成ST边界的障碍物（触发路径重规划）
 * - OVERTAKE类型：允许本车执行超车动作
 * - YIELD类型：强制要求避让障碍物
 *
 * @warning 需确保:
 * - 已完成障碍物的SL投影计算（通过STBoundaryMapper）
 * - 已完成障碍物运动状态预测（通过GenerateLaneChangeOvertakeObstacleSTBoundary）
 */
void STGraphProcessor::updateOvertakeObstacleTag(const LocalView& local_view, std::shared_ptr<SpeedPlannerObstacle> obstacle,
                                                 std::shared_ptr<SpeedResult> speed_result) {
  if (obstacle->pathStBoundary().IsEmpty()) {
    obstacle->setPathStBoundaryType(STBoundary::BoundaryType::RISKY);
  } else {
    // TODO: decide overtake tag  or yield tag
    bool is_overtake_safe = checkOvertakingSafety(local_view, obstacle, speed_result, STBoundary::BoundaryType::OVERTAKE);
    if (is_overtake_safe) {
      obstacle->setPathStBoundaryType(STBoundary::BoundaryType::OVERTAKE);
    } else {  // yield tag
      obstacle->setPathStBoundaryType(STBoundary::BoundaryType::YIELD);
    }
  }
}
/**
 * @brief 更新风险障碍物标签（RISKY类型决策）
 * @param[in] local_view 局部环境感知数据（定位/底盘/感知等）
 * @param[in,out] obstacle 目标障碍物对象（输入原始数据，输出更新后标签）
 * @param[in,out] speed_result 速度规划结果（含行为状态信息）
 * 
 * @par 输入/输出参数说明:
 * | 参数           | 类型                              | 取值范围        | 说明                     |
 * |----------------|-----------------------------------|----------------|--------------------------|
 * | local_view     | const LocalView&                 | -              | 输入参数，车辆环境感知数据|
 * | obstacle       | shared_ptr<SpeedPlannerObstacle> | -              | 输入输出参数，障碍物对象  |
 * | speed_result   | shared_ptr<SpeedResult>          | -              | 输入输出参数，速度规划结果 |
 *
 * @par 处理流程:
 * @startuml
 partition 边界有效性校验 {
   if (ST边界为空?) then (是)
     :标记为RISKY类型;
   else (否)
     partition 风险安全检查 {
       :调用checkOvertakingSafety;
       if (安全检查通过?) then (是)
         :升级为OVERTAKE类型;
       else (否)
         :降级为YIELD类型;
       endif
     }
   endif
 }
 @enduml
 *
 * @note 核心特性:
 * - RISKY类型：用于未生成ST边界的障碍物（触发紧急处理）
 * - OVERTAKE类型：允许有限制的超车动作
 * - YIELD类型：强制要求避让障碍物
 *
 * @warning 需确保:
 * - 障碍物的运动预测已完成（通过predictedTrajectory）
 * - 障碍物的初步风险评估已完成（通过getRiskFieldInfos）
 */
void STGraphProcessor::updateRiskyObstacleTag(const LocalView& local_view, std::shared_ptr<SpeedPlannerObstacle> obstacle,
                                              std::shared_ptr<SpeedResult> speed_result) {
  if (obstacle->pathStBoundary().IsEmpty()) {
    obstacle->setPathStBoundaryType(STBoundary::BoundaryType::RISKY);
  } else {
    bool is_overtake_safe = checkOvertakingSafety(local_view, obstacle, speed_result, STBoundary::BoundaryType::RISKY);
    if (is_overtake_safe) {
      obstacle->setPathStBoundaryType(STBoundary::BoundaryType::OVERTAKE);
    } else {  // yield tag
      obstacle->setPathStBoundaryType(STBoundary::BoundaryType::YIELD);
    }
  }
}
/**
 * @brief 超车安全校验核心算法（基于动力学模型的碰撞预测）
 * @param[in] local_view 局部环境感知数据（定位/底盘/感知等）
 * @param[in] obstacle 目标障碍物对象（需包含时空边界信息）
 * @param[in] speed_result 速度规划结果（当前未使用，保留参数）
 * @param[in] boundary_type 边界类型（OVERTAKE/RISKY影响校验逻辑）
 * 
 * @par 输入参数说明:
 * | 参数           | 类型                     | 取值范围                   | 单位  | 说明                 |
 * |----------------|--------------------------|---------------------------|-------|---------------------|
 * | invade_t       | double                   | [0, time_horizon_]        | 秒    | 首次入侵时间         |
 * | invade_lower_s | double                   | [-300, 300]               | 米    | 障碍物下边界初始位置 |
 * | out_lower_s    | double                   | [-300, 300]               | 米    | 障碍物下边界终点位置 |
 * | safe_acc_table | vector<double>           | {0.5,0.6,0.8,1.0}         | m/s² | 安全加速度阈值表     |
 * | safe_dec_table | vector<double>           | {-4.0,-3.8,-3.5,-3.0}     | m/s² | 安全减速度阈值表     |
 * 
 * @par 处理流程:
 * @startuml
 start
 :获取车辆配置参数;
 :提取障碍物ST边界关键点;
 if (invade_lower_s < 1m || out_lower_s < 1m || (OVERTAKE类型且invade_t>2s)) then (是)
   :直接返回安全;
   stop;
 else (否)
   :计算碰撞自由加速度/减速度;
   :查表获取平滑加速度阈值;
   :判断安全条件;
   if (碰撞自由加速度<=平滑阈值 || (碰撞自由加速度<=安全加速度\n&& 碰撞自由减速度<=安全减速度)) then (是)
     :返回安全;
   else (否)
     :返回危险;
   endif
 endif
 end
 @enduml
 *
 * @return true - 允许超车，false - 需要避让
 */
bool STGraphProcessor::checkOvertakingSafety(const LocalView& local_view, std::shared_ptr<SpeedPlannerObstacle> obstacle,
                                             std::shared_ptr<SpeedResult> speed_result, STBoundary::BoundaryType boundary_type) {
  auto config_manager = Singleton<ConfigManager>::get_instance();
  CHECK_NOTNULL(config_manager);
  auto vehicle_config = config_manager->vehicle_config();
  double invade_t = obstacle->pathStBoundary().bottom_left_point().t();
  double invade_lower_s = obstacle->pathStBoundary().bottom_left_point().s();
  double out_lower_s = obstacle->pathStBoundary().bottom_right_point().s();
  double invade_upper_s = obstacle->pathStBoundary().upper_left_point().s();
  const std::vector<double> safe_acc_table = {0.5, 0.6, 0.8, 1.0};
  const std::vector<double> safe_dec_table = {-4.0, -3.8, -3.5, -3.0};
  const std::vector<double> invade_time_table = {1.0, 2.0, 3.0, 4.0};
  const std::vector<double> smooth_acc_table = {0.0, 0.2, 0.4, 0.5};
  const double back_safe_distance = 1.0;
  if(invade_lower_s < 1.0 || out_lower_s < 1.0 || (invade_t > 2.0 && boundary_type == STBoundary::BoundaryType::OVERTAKE)){
    return true;
  }
  // Safe Dcc and acc:
  double collision_free_acc = 0.0, collision_free_decc = 0.0;
  double acc_longi_position = invade_upper_s + back_safe_distance;
  collision_free_acc =
      (acc_longi_position - local_view.getChassisPtr()->Speed() * invade_t) / (0.5 * pow(invade_t, 2) + kMathEpsilon);
  double dec_longi_position =
      fmax(kMathEpsilon, invade_lower_s - vehicle_config.vehicle_param().front_edge_to_ego() - back_safe_distance);
  collision_free_decc =
      (dec_longi_position - local_view.getChassisPtr()->Speed() * invade_t) / (0.5 * pow(invade_t, 2) + kMathEpsilon);
  for (size_t i = 0; i < obstacle->pathStBoundary().lower_points().size(); ++i) {
    double current_t = obstacle->pathStBoundary().lower_points().at(i).t();
    double current_s = obstacle->pathStBoundary().lower_points().at(i).s();
    double current_dec_longi_position =
        fmax(kMathEpsilon, current_s - vehicle_config.vehicle_param().front_edge_to_ego() - back_safe_distance);
    collision_free_decc =
        fmin(collision_free_decc, (current_dec_longi_position - local_view.getChassisPtr()->Speed() * current_t) /
                                      (0.5 * pow(current_t, 2) + kMathEpsilon));
  }
  double smooth_acc = math::TableLookUp1D(invade_time_table, smooth_acc_table, invade_t);
  double safe_acc = math::TableLookUp1D(invade_time_table, safe_acc_table, invade_t);
  double safe_dec = math::TableLookUp1D(invade_time_table, safe_dec_table, invade_t);
  ERT_PLOG_I << "collision_free_acc: " << collision_free_acc << " safe_acc: " << safe_acc << " safe_dec: " << safe_dec
            ;
  bool is_overtake_safe =
      collision_free_acc <= smooth_acc || (collision_free_acc <= safe_acc && collision_free_decc <= safe_dec);
  return is_overtake_safe;
}
/**
 * @brief 对向障碍物风险校验（冲突点风险评估）
 * @param[in] local_view 局部环境感知数据（定位/底盘/感知等）
 * @param[in] obstacle 目标障碍物对象（需包含风险场信息）
 * 
 * @par 输入参数说明:
 * | 参数                | 类型           | 取值范围        | 单位 | 说明                     |
 * |---------------------|----------------|----------------|------|--------------------------|
 * | s_threshold_table   | vector<double> | [10,15...80]   | 米   | 距离阈值表（速度映射）    |
 * | ego_speed_table     | vector<double> | [15,20...80]   | km/h | 自车速度映射表            |
 * | obs_init_min_dis    | double         | [0.0, 300.0]   | 米   | 障碍物初始最小距离        |
 * | obs_init_s          | double         | [-300.0, 300.0]| 米   | 障碍物初始纵向坐标        |
 * 
 * @par 处理流程:
 * @startuml
 if (风险信息为空?) then (是)
   :返回低风险(false);
 else (否)
   :构建速度-距离映射表;
   :计算动态距离阈值(s_threshold);
   if (初始距离 < 阈值 且 最小距离 <= 0.1m) then (是)
     :返回高风险(true);
   else (否)
     :返回低风险(false);
   endif
 endif
 @enduml
 *
 * @return true - 存在碰撞风险，需要紧急处理\n
 *         false - 风险可控，无需特殊处理
 * 
 * @note 典型应用场景:
 * - 对向车道借道超车风险评估
 * - 交叉口冲突点预判
 * 
 * @warning 需确保调用前已完成:
 * - 障碍物风险场信息计算（通过getRiskFieldInfos）
 * - 自车速度坐标系转换（MS_KMH单位处理）
 */
bool STGraphProcessor::checkOppositeObstacleRisk(const LocalView& local_view, std::shared_ptr<SpeedPlannerObstacle> obstacle){
  if(obstacle->getRiskFieldInfos().empty()){
    return false;
  }
  // 暂时使用自车速度,后续考虑使用ttc
  vector<double> s_threshold_table = {10, 15, 25, 35, 50, 65, 80};
  vector<double> ego_speed_table = {15, 20, 30, 40, 50, 60, 80};

  auto obs_init_s = obstacle->getRiskFieldInfos().front().nearest_s;
  auto obs_init_min_dis = obstacle->getRiskFieldInfos().front().min_box_distance;
  double s_threshold = math::TableLookUp1D(ego_speed_table, s_threshold_table, local_view.getChassisPtr()->Speed() * MS_KMH);
  if(obs_init_min_dis <= 0.1 && obs_init_s < s_threshold ){
    return true;
  }
  return false;
}
/**
 * @brief 对向障碍物处理流程（时空边界动态更新）
 * @param[in] local_view 局部环境感知数据（定位/底盘/感知等）
 * @param[in] obstacle_set 障碍物集合（包含所有感知障碍物）
 * 
 * @par 输入参数说明:
 * | 参数                | 类型           | 取值范围              | 单位 | 说明                     |
 * |---------------------|----------------|----------------------|------|--------------------------|
 * | ObjectGameType      | 枚举类型       | OPPOSITE_GAME(0x04)  | -    | 对向障碍物类型标识        |
 * | ego_speed           | double         | [0.0, 120.0]         | m/s  | 自车当前速度              |
 * | confident_time      | double         | [0.5, 8.0]           | 秒   | 置信时间窗长度            |
 * 
 * @par 处理流程:
 * @startuml
 start
 partition 障碍物遍历处理 {
   while (遍历障碍物集合?) is (存在未处理项)
     :获取当前障碍物;
     if (障碍物类型!=对向障碍物 或 ST边界为空?) then (是)
       :跳过处理;
     else (否)
       :计算置信时间(calculateConfidentTime);
       :更新对向障碍物边界(updateOppositeObstacleBoundary);
     endif
   endwhile
 }
 stop
 @enduml
 *
 * @note 典型应用场景:
 * - 对向车道借道超车冲突预测
 * - 交叉口对向车流冲突解决
 * 
 * @warning 需确保调用前已完成:
 * - 障碍物的ST边界初步计算（通过STBoundaryMapper）
 * - 障碍物运动轨迹预测（通过predictedTrajectory）
 */
void STGraphProcessor::oppositeObstacleProcess(const LocalView& local_view, const ObstacleSet& obstacle_set){
  for (const auto& obstacle_item : obstacle_set) {
    const auto& curr_obstacle = obstacle_item.second;
    // ERT_PLOG_I<<"  id = "<<curr_obstacle->id()<<" game_type = "<<(int)curr_obstacle->getObjectGameType();
    if (curr_obstacle->getObjectGameType() != ObjectGameType::OPPOSITE_GAME || 
         curr_obstacle->pathStBoundary().IsEmpty()) {
      continue;
    }
    // 在这里添加对向障碍物的处理逻辑
    double confident_time = calculateConfidentTime(curr_obstacle, local_view.getChassisPtr()->Speed());
    updateOppositeObstacleBoundary(curr_obstacle,local_view.getChassisPtr()->Speed(), confident_time);
  }
}
/**
 * @brief 对向障碍物置信时间计算（多因素动态时间窗预测）
 * @param[in] obstacle 目标障碍物对象（需包含ST边界信息）
 * @param[in] ego_speed 自车当前速度（单位：m/s）
 * 
 * @par 输入参数说明:
 * | 参数                   | 类型           | 取值范围        | 单位 | 说明                     |
 * |------------------------|----------------|----------------|------|--------------------------|
 * | follow_distance_table  | vector<double> | [15,25...60]   | 米   | 跟车距离映射表            |
 * | delta_speed_table      | vector<double> | [0,3...15]     | m/s  | 速度差映射表              |
 * | coeff_table            | vector<double> | [1.0,1.5...4.0]| -    | 综合系数映射表            |
 * | max_overlap_ratio_table | vector<double> | [0.0,0.1...1.0]| -    | 横向重叠比例映射表        |
 * 
 * @par 处理流程:
 * @startuml
 start
 :初始化置信时间为规划时界;
 partition 横向重叠分析 {
   :遍历障碍物ST边界点;
   if (横向距离≤自车半宽+0.1m?) then (是)
     :计算最大横向重叠比例;
   endif
 }
 partition 动态系数计算 {
   :基于跟车距离查表获取距离系数;
   :基于速度差查表获取速度系数;
   :计算综合系数=距离系数×速度系数;
   :查表获取基础置信时间;
 }
 partition 横向重叠修正 {
   :根据最大重叠比例查表获取时间修正值;
   :最终置信时间取最小值;
 }
 stop
 @enduml
 *
 * @return double - 置信时间窗（单位：秒），表示障碍物有效影响时间范围
 * 
 * @note 典型应用场景:
 * - 对向障碍物冲突时间预测
 * - 交叉口潜在碰撞时间评估
 * 
 * @warning 需确保调用前已完成：
 * - 障碍物ST边界生成（通过STBoundaryMapper）
 * - 自车速度坐标系转换（FLU坐标系）
 */
double STGraphProcessor::calculateConfidentTime(std::shared_ptr<SpeedPlannerObstacle> obstacle, double ego_speed){
    double confident_time = time_horizon_;
    const double ego_half_width = 0.5 * vehicle_param_.width();
    const auto& old_lateral_signed_distances = obstacle->pathStBoundary().lateral_signed_distances();
    auto lower_points = obstacle->pathStBoundary().lower_points();
    double max_l_overlap_from_prediction = 0.0;
    for (size_t i = 0; i < lower_points.size(); ++i) {
      double t = lower_points[i].t();
      if (std::abs(old_lateral_signed_distances[i]) <= ego_half_width + 0.1) {
        max_l_overlap_from_prediction =
            std::max(max_l_overlap_from_prediction, ego_half_width - std::abs(old_lateral_signed_distances[i]));
      }
    }

    const std::vector<double> follow_distance_table = {15.0, 25.0, 30.0, 50.0, 60.0};
    std::vector<double> coeff_table_wrt_distance = {2.0, 1.6, 1.2, 1.0, 1.0};
    double coeff_wrt_distance =
        math::TableLookUp1D(follow_distance_table, coeff_table_wrt_distance, obstacle->pathStBoundary().bottom_left_point().s());

    std::vector<double> delta_speed_table = {0.0, 3.0, 6.0, 10.0, 15.0};
    std::vector<double> coeff_table_wrt_delta_speed = {1.0, 1.1, 1.3, 1.6, 2.0};

    double obstacle_speed = 0; //  TODO
    double delta_speed = ego_speed - obstacle_speed;
    double coeff_wrt_delta_speed = math::TableLookUp1D(delta_speed_table, coeff_table_wrt_delta_speed, delta_speed);

    const std::vector<double> coeff_table = {1.0, 1.5, 2.0, 3.0, 3.5, 4.0};
    std::vector<double> confident_time_table = {2.0, 2.1, 2.3, 2.5, 2.8, 3.0};
    confident_time = math::TableLookUp1D(coeff_table, confident_time_table, coeff_wrt_distance * coeff_wrt_delta_speed);
    // Consider max-overlap:
    double max_overlap_ratio = max_l_overlap_from_prediction / std::max(ego_half_width, 1.0);
    const std::vector<double> max_overlap_ratio_table = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.75, 1.0};
    std::vector<double> confident_time_table_wrt_max_overlap_ratio = {0.5, 0.5, 1.0, 1.3, 1.6, 2.0, 3.0, 4.0};
    double max_confident_time_wrt_overlap_ratio =
        math::TableLookUp1D(max_overlap_ratio_table, confident_time_table_wrt_max_overlap_ratio, max_overlap_ratio);
    confident_time = std::min(confident_time, max_confident_time_wrt_overlap_ratio);
    return confident_time;
}
/**
 * @brief 对向障碍物时空边界更新（动态安全距离补偿）
 * @param[in] obstacle 目标障碍物对象（需包含ST边界信息）
 * @param[in] ego_speed 自车当前速度（单位：m/s）
 * @param[in] confident_time 置信时间窗（单位：秒）
 * 
 * @par 输入参数说明:
 * | 参数                | 类型           | 取值范围        | 单位 | 说明                     |
 * |---------------------|----------------|----------------|------|--------------------------|
 * | od_safe_distance    | double         | [5.0, 10.0]    | 米   | 对向障碍物基础安全距离    |
 * | comfort_decc_for_brake | double    | [-3.0, -1.0]   | m/s² | 舒适减速度阈值            |
 * | t_buffer            | double         | [0.5, 2.0]     | 秒   | 制动响应时间缓冲          |
 * 
 * @par 处理流程:
 * @startuml
 start
 partition 边界预处理 {
   :获取障碍物原始ST边界;
   :计算障碍物投影速度;
   :生成时间网格采样点;
 }
 partition 安全距离计算 {
   :基于自车速度计算制动距离;
   :考虑舒适减速度计算期望边界;
   :叠加障碍物速度缓冲距离;
 }
 partition 边界点更新 {
   :遍历所有下边界点;
   :应用安全距离约束;
   :生成新边界点对;
 }
 if (新边界点数量不足?) then (是)
   :标记为UNKNOWN类型;
 else (否)
   :设置新ST边界;
 endif
 stop
 @enduml
 *
 * @note 核心计算公式:
 * 1. 制动距离计算: 
 *    desire_bound = od_safe_distance + (ego_speed²)/(2*|comfort_decc|)
 * 2. 速度缓冲距离: 
 *    stop_buffer = |obs_speed_flu| * t_buffer
 * 3. 最终安全边界: 
 *    s_lower = max(desire_bound, original_s_lower)
 *
 * @warning 需确保调用前已完成:
 * - 障碍物ST边界生成（通过STBoundaryMapper）
 * - 置信时间计算（通过calculateConfidentTime）
 */
void STGraphProcessor::updateOppositeObstacleBoundary(std::shared_ptr<SpeedPlannerObstacle> obstacle, double ego_speed,
                                                      double confident_time) {
  std::vector<std::pair<STPoint, STPoint>> new_point_pairs;
  std::vector<double> new_lateral_signed_distances;

  // const auto& obs_box = obstacle->getBoundingBoxAtTime(0.0);
  // double min_x = kPostiveInfinity;
  //   std::vector<math::Vec2d> box_map_corners;
  //   obs_box.GetAllCorners(&box_map_corners);
  //   for (const auto& pt : box_map_corners) {
  //     MapPoint map_pt(pt.x(), pt.y(), 0.0, 0.0, 0.0, 0.0, obs_box.heading());
  //     auto ego_pt = map2FLU(frame->local_view()->loc_->map_origin_point(),
  //                           frame->local_view()->loc_->vehicle_align_pose_point(), map_pt);
  //     min_x = fmin(min_x, std::hypot(ego_pt.x(), ego_pt.y()));
  //   }
  const auto& boundary = obstacle->pathStBoundary();
  double dt = boundary.bottom_right_point().t() - boundary.bottom_left_point().t();
  double obs_speed_flu = boundary.calcSTLowerBoundProjectedSpeed(boundary.bottom_left_point().t(), dt);

  std::vector<STPoint> lower_points, upper_points;
  double delta_t = 0.2;
  const double min_t = boundary.min_t();
  const double max_t = boundary.max_t();
  for (double t = min_t; t + kMathEpsilon < max_t; t += delta_t) {
    double s_lower, s_upper;
    if (boundary.getBoundarySRange(t, &s_upper, &s_lower) && s_upper > s_lower + 0.01) {
      lower_points.emplace_back(STPoint(s_lower, t));
      upper_points.emplace_back(STPoint(s_upper, t));
    }
  }
  double s_lower, s_upper;
  if (boundary.getBoundarySRange(max_t, &s_upper, &s_lower) && s_upper > s_lower + 0.01) {
    lower_points.emplace_back(STPoint(s_lower, max_t));
    upper_points.emplace_back(STPoint(s_upper, max_t));
  }

  auto old_lateral_signed_distances = boundary.lateral_signed_distances();

  double min_s = obstacle->getRiskFieldInfos().front().nearest_s;

  double od_safe_distance = 5.0;
  double comfort_decc_for_brake = -1.2;
  double desire_bound = od_safe_distance + ego_speed * ego_speed / std::abs(comfort_decc_for_brake);
  double t_buffer = 1.0;
  double stop_buffer = fabs(obs_speed_flu) * t_buffer;

  // ERT_PLOG_I<<" lower_point size = "<<lower_points.size()<<" upper_point size = "<<upper_points.size();
  desire_bound = std::fmin(
      fmax(min_s - (vehicle_param_.length() - vehicle_param_.rear_edge_to_ego()) - stop_buffer, 0.0), desire_bound);
  for (auto& lower_point : lower_points) {
    lower_point.set_s(fmax(desire_bound, lower_point.s()));
  }

  for (size_t j = 0; j < lower_points.size(); ++j) {
    double t = lower_points[j].t();
    STPoint upper_point = upper_points[j];
    upper_point.set_s(fmax(upper_point.s(), lower_points[j].s() + 0.1));
    new_point_pairs.emplace_back(lower_points[j], upper_point);
    if (j < old_lateral_signed_distances.size()) {
      new_lateral_signed_distances.emplace_back(old_lateral_signed_distances[j]);
    }
    // ERT_PLOG_I<<" t = "<<t<<" s_lower = "<<lower_points[j].s()<<" s_upper = "<<upper_point.s()<<" l_signed = "<<old_lateral_signed_distances[j];
  }

  if (new_point_pairs.size() <= 0) {
    obstacle->setPathStBoundaryType(STBoundary::BoundaryType::UNKNOWN);
  } else {
    if (confident_time >= new_point_pairs.back().first.t() + 0.1) {
      double s_lower, s_upper;
      if ((boundary.getBoundarySRange(confident_time, &s_upper, &s_lower)) && (s_upper > s_lower + 0.01)) {
        s_lower = fmax(s_lower, desire_bound);
        s_upper = fmax(s_upper, s_lower + 0.1);
        new_point_pairs.emplace_back(STPoint(s_lower, confident_time), STPoint(s_upper, confident_time));
        new_lateral_signed_distances.emplace_back(0.5*vehicle_param_.width());
      }
    }
  }

  if (new_point_pairs.size() >= 2) {
    STBoundary new_boundary = STBoundary(std::move(new_point_pairs));
    new_boundary.setBoundaryType(boundary.boundary_type());

    new_boundary.set_id(boundary.id());
    new_boundary.set_lateral_signed_distances(new_lateral_signed_distances);
    obstacle->setPathStBoundary(new_boundary);
  } else {
    obstacle->setPathStBoundaryType(STBoundary::BoundaryType::UNKNOWN);
  }
  auto su = obstacle->pathStBoundary().upper_points();
  auto sl = obstacle->pathStBoundary().lower_points();
  // ERT_PLOG_I<<">>>>>>>>>>>>>>>>>>  confident time = "<< confident_time;
  // for(int i = 0; i< su.size(); i++){
  //   ERT_PLOG_I<<" t = "<<su[i].t()<<" su = "<<su[i].s() <<" sl = "<<sl[i].s();
  // }

}
/**
 * @brief 交叉口障碍物时空边界处理（动态时间窗扩展）
 * @param[in] local_view 局部环境感知数据
 * @param[in] obstacle_set 障碍物集合
 * 
 * @par 输入参数说明:
 * | 参数                | 类型               | 取值范围        | 单位 | 说明                     |
 * |---------------------|--------------------|----------------|------|--------------------------|
 * | distance_table      | vector<double>     | [6.0, 30.0]     | 米   | 障碍物距离映射表          |
 * | speed_table         | vector<double>     | [0.0, 15.0]     | m/s  | 障碍物速度映射表          |
 * | weight_table        | vector<double>     | [1.1, 5.0]      | -    | 距离-速度综合权重表       |
 * | coeff_table        | vector<double>     | [1.0, 25.0]     | -    | 综合系数映射表            |
 * | horizon_expand_table| vector<double>     | [0.5, 2.5]      | 秒   | 时间扩展量映射表          |
 *
 * @par 处理流程:
 * @startuml
 start
 partition 障碍物遍历 {
   while (遍历障碍物集合?) is (存在未处理项)
     :获取当前障碍物;
     if (类型!=交叉口障碍物 或 ST边界为空?) then (是)
       :跳过处理;
     else (否)
       :计算综合系数;
       :查表获取时间扩展量;
       :扩展ST边界时间范围;
     endif
   endwhile
 }
 stop
 @enduml
 *
 * @note 核心算法:
 * - 动态时间扩展: 基于障碍物距离/速度查表计算扩展量
 * - 边界点更新: 在原始ST边界基础上追加扩展时间点
 * - 最大时间限制: 扩展后时间不超过5秒
 *
 * @warning 使用约束:
 * - 需确保障碍物已完成ST边界生成
 * - 仅处理CROSS_GAME类型障碍物
 * - 时间扩展量映射表需与场景匹配
 */
void STGraphProcessor::crossObstacleProcess(const LocalView& local_view, const ObstacleSet& obstacle_set){
  for (const auto& obstacle_item : obstacle_set) {
    const auto& curr_obstacle = obstacle_item.second;
    // ERT_PLOG_I<<"  id = "<<curr_obstacle->id()<<" game_type = "<<(int)curr_obstacle->getObjectGameType();
    if (curr_obstacle->getObjectGameType() != ObjectGameType::CROSS_GAME || curr_obstacle->pathStBoundary().IsEmpty()) {
      continue;
    }
    const auto& curr_obs_boundary = curr_obstacle->pathStBoundary();
    std::vector<double> distance_table = {6.0, 8.0, 10.0, 15.0, 25.0, 30.0};
    std::vector<double> speed_table = {0.0, 2.0, 5.0, 8.0, 13.0, 15.0};
    std::vector<double> weight_table = {5.0, 4.8, 4.5, 4.0, 2.5, 1.1};
    double coeff = math::TableLookUp1D(distance_table, weight_table, curr_obs_boundary.bottom_left_point().s()) *
                   math::TableLookUp1D(speed_table, weight_table,
                                       curr_obs_boundary.calcSTLowerBoundProjectedSpeed(
                                           curr_obs_boundary.bottom_left_point().t(), 3.0));

    std::vector<double> coeff_table = {1.0, 8.0, 15.0, 20.0, 22.0, 25.0};
    std::vector<double> horizon_expand_table = {0.5, 0.8, 1.2, 1.6, 2.0, 2.5};
    double t_expansion = math::TableLookUp1D(coeff_table, horizon_expand_table, coeff);
    // ERT_PLOG_I<<" current obs =  " << curr_obs_boundary.id() << "  t_expansion = "<<t_expansion;
    std::vector<STPoint> lower_points, upper_points;
    lower_points = curr_obs_boundary.lower_points();
    upper_points = curr_obs_boundary.upper_points();
    double max_t = curr_obs_boundary.upper_points().back().t();
    if (max_t < 5.0) {
      max_t = min(max_t + t_expansion, 5.0);
    } else {
      continue;
    }
    lower_points.emplace_back(lower_points.back().s(), max_t);
    upper_points.emplace_back(upper_points.back().s(), max_t);
    auto boundary = STBoundary::createInstance(lower_points, upper_points);
    boundary.set_lateral_signed_distances(curr_obs_boundary.lateral_signed_distances());
    boundary.set_id(curr_obs_boundary.id());
    boundary.setBoundaryType(curr_obs_boundary.boundary_type());
    curr_obstacle->setPathStBoundary(boundary);
  }
}
/**
 * @brief 处理汇入障碍物场景的动态参数调整（安全距离与速度限制）
 * @param[in] obstacle_map 障碍物集合（含风险ST边界）
 * @param[in] path_group 路径信息组（含扩展路径）
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
partition 参数调整 {
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
void STGraphProcessor::mergeObstacleProcess(const LocalView& local_view, const ObstacleSet& obstacle_set){
  // 后续可使用决策 gametype = merge_game 来进行判断
  for(const auto& obstacle_item : obstacle_set){
    const auto& curr_obstacle = obstacle_item.second;
    const auto& curr_obs_boundary = curr_obstacle->pathStBoundary();
    const auto& curr_obs_risk_boundary = curr_obstacle->riskStBoundary();
    if(curr_obs_boundary.IsEmpty() || curr_obs_risk_boundary.IsEmpty()){
      continue;
    }
    if(curr_obs_boundary.boundary_type() != STBoundary::BoundaryType::FOLLOW && 
        curr_obs_boundary.boundary_type() != STBoundary::BoundaryType::YIELD && 
        curr_obs_boundary.boundary_type() != STBoundary::BoundaryType::RISKY){
      continue;
    }
    double invasion_t = curr_obs_boundary.min_t();
    double invasion_s = curr_obs_boundary.bottom_left_point().s();
    double risk_invasion_t = curr_obs_risk_boundary.min_t();
    double risk_invasion_s = curr_obs_risk_boundary.bottom_left_point().s();
    bool curr_obs_is_merge_obs = false;
    if( (risk_invasion_t < invasion_t && (invasion_t - risk_invasion_t) > 1.0) || 
        ( risk_invasion_s < invasion_s && (invasion_s - risk_invasion_s) > 7.0 ) ){
      curr_obs_is_merge_obs = true;
    }
    obstacle_item.second->setIsMergeObstacle(curr_obs_is_merge_obs);
    // ERT_PLOG_I<<" obstacle id = "<<curr_obstacle->id()<<" invasion_t = "<<invasion_t << "  risk_invasion_t = "<< risk_invasion_t<<"   type = "<<(int)obstacle_item.second->getObjectGameType() ;
  }

}


}  // namespace gpal::pnc::planning
