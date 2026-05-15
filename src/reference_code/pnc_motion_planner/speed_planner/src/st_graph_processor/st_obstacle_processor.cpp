/**
 * @file st_obstacle_processor.cpp
 * @brief 障碍物ST投影处理器
 * @details 本类负责障碍物时空边界的精确计算与建模
 */
#include "st_graph_processor/st_obstacle_processor.h"

#include <utility>

#include "math/linear_interpolation.h"

namespace gpal::pnc::planning {
/**
 * @brief 初始化ST障碍物处理器（构造函数）
 * @param[in] time_grid 时间采样网格数组
 * @param[in] time_resolution 时间分辨率（网格间隔）
 * @param[in] time_horizon 时间规划范围
 * 
 * @par 输入参数说明:
 * | 参数             | 类型            | 取值范围          | 单位 | 说明                  |
 * |------------------|-----------------|------------------|------|-----------------------|
 * | time_grid        | vector<double> | [0.0, time_horizon] | 秒 | 时间轴离散采样点序列  |
 * | time_resolution  | double          | (0.0, 5.0]       | 秒   | 时间轴最小分辨率      |
 * | time_horizon     | double          | [5.0, 15.0]      | 秒   | 最大规划时间范围      |
 *
 * @note 关键初始化操作:
 * 1. 时间网格采用移动语义传递（避免大规模数组拷贝）
 * 2. 通过配置管理器获取：
 *    - 车辆动力学参数（vehicle_config）
 *    - ST图处理器配置（StGraphProcessorConfig）
 * 
 * @warning 需确保:
 * - 配置管理器已正确初始化（通过Singleton模板）
 * - 时间网格数组需严格递增排序
 */
STObstacleProcessor::STObstacleProcessor(std::vector<double> time_grid, double time_resolution, double time_horizon) {
  time_grid_ = std::move(time_grid);
  time_resolution_ = time_resolution;
  time_horizon_ = time_horizon;
  auto config_manager = Singleton<ConfigManager>::get_instance();
  CHECK_NOTNULL(config_manager);
  vehicle_config_ = config_manager->vehicle_config();
  param_ = config_manager->getConfig<StGraphProcessorConfig>("StGraphProcessorConfig");
}
/**
 * @brief ST边界计算主入口（时空风险场生成）
 * @param[in] obstacle 障碍物对象（需包含预测轨迹）
 * @param[in] discretized_path 参考路径离散点
 * 
 * @par 输入参数说明:
 * | 参数                | 类型           | 取值范围        | 单位 | 说明                     |
 * |---------------------|----------------|----------------|------|--------------------------|
 * | time_grid           | vector<double> | [0.0, time_horizon] | 秒 | 时间采样网格              |
 * | obstacle_buffer     | double         | [0.0, 0.5]     | 米   | 障碍物膨胀缓冲距离         |
 * | check_range         | double         | [5.0, 15.0]    | 米   | 路径搜索范围               |
 * 
 * @par 处理流程:
 * @startuml
 start
 :初始化时间索引范围;
 partition 风险场计算 {
   :调用calcRiskFieldInfos;
   :生成ST边界点集合;
   :获取横向签名距离;
 }
 :创建ST边界对象;
 :设置边界ID和类型;
 :生成网格化风险场数据;
 stop
 @enduml
 *
 * @note 典型应用场景:
 * - 障碍物时空走廊生成
 * - 动态障碍物风险场建模
 * 
 * @warning 需确保调用前已完成:
 * - 障碍物预测轨迹生成（通过predictedTrajectory）
 * - 参考路径离散化处理（通过DiscretizedPath）
 */
void STObstacleProcessor::ComputeSTBoundary(const std::shared_ptr<SpeedPlannerObstacle>& obstacle,
                                            const DiscretizedPath& discretized_path, const BehaviorState& behavior_state) {
  size_t t_index_min = 0;
  size_t t_index_max = obstacle->predictedTrajectory().size()-1;
  std::vector<BoxProjectInfo> box_project_info;
  ComputeSTBoundary(obstacle, discretized_path, behavior_state, box_project_info, t_index_min, t_index_max);
}
/**
 * @brief ST边界计算扩展接口（带投影信息约束）
 * @param[in] box_project_info 障碍物框投影信息集合
 * @param[in] t_index_min 最小时间索引
 * @param[in] t_index_max 最大时间索引
 * 
 * @par 核心处理步骤:
 * 1. 调用风险场计算核心算法
 * 2. 生成ST边界上下界点集合
 * 3. 构建网格化时间风险场信息
 * 4. 更新障碍物风险场数据
 */
void STObstacleProcessor::ComputeSTBoundary(const std::shared_ptr<SpeedPlannerObstacle>& obstacle,
                                            const DiscretizedPath& discretized_path,const BehaviorState& behavior_state,
                                            const std::vector<BoxProjectInfo>& box_project_info,
                                            const size_t& t_index_min, const size_t& t_index_max) {
  std::vector<STPoint> lower_points;
  std::vector<STPoint> upper_points;
  std::vector<double> lateral_signed_distances;

  math::IntervalData<SpeedPlannerObstacle::RiskFieldInfo> risk_field_infos =
      calcRiskFieldInfos(obstacle, discretized_path, behavior_state, box_project_info, t_index_min, t_index_max, &lower_points,
                         &upper_points, &lateral_signed_distances);

  auto boundary = STBoundary::createInstance(lower_points, upper_points);
  boundary.set_id(obstacle->id());
  boundary.setBoundaryType(obstacle->pathStBoundary().boundary_type());
  boundary.set_lateral_signed_distances(lateral_signed_distances);  // to do
  obstacle->setPathStBoundary(boundary);
  // for (int i = 0; i < lower_points.size(); i++) {
  //   ERT_PLOG_I << "       t=  " << lower_points.at(i).t() << "  s_lower = " << lower_points.at(i).s()
  //        << "   s_upper = " << upper_points.at(i).s() ;
  // }
  vector<SpeedPlannerObstacle::RiskFieldInfo> grid_time_risk_field_infos;
  for (size_t i = 0; i < time_grid_.size(); i++) {
    grid_time_risk_field_infos.emplace_back(risk_field_infos.evaluate(time_grid_[i]));
  }
  obstacle->setRiskFieldInfos(grid_time_risk_field_infos);
}
/**
 * @brief 局部路径ST边界计算（动态障碍物投影处理）
 * @param[in] obstacle 障碍物对象（需包含预测轨迹）
 * @param[in] discretized_path 参考路径离散点
 * @param[in] box_project_info 障碍物框投影信息集合
 * @param[in] t_index_min 最小时间索引
 * @param[in] t_index_max 最大时间索引
 * @param[out] boundary 输出生成的ST边界
 * 
 * @par 输入参数说明:
 * | 参数                | 类型           | 取值范围        | 单位 | 说明                     |
 * |---------------------|----------------|----------------|------|--------------------------|
 * | t_index_min         | size_t         | [0, traj_size] | -    | 轨迹起始时间索引          |
 * | t_index_max         | size_t         | [0, traj_size] | -    | 轨迹结束时间索引          |
 * | obstacle_buffer     | double         | [0.0, 0.5]     | 米   | 障碍物膨胀缓冲距离         |
 * 
 * @par 处理流程:
 * @startuml
 start
 partition 风险场计算 {
   :调用calcRiskFieldInfos;
   :获取ST边界点集合;
   :获取横向签名距离;
 }
 :创建ST边界实例;
 :设置障碍物ID;
 :设置横向签名距离;
 :输出边界点数量;
 stop
 @enduml
 *
 * @note 核心特征:
 * - 支持动态障碍物轨迹投影
 * - 输出边界点包含横向距离信息
 * 
 * @warning 需确保调用前已完成:
 * - 障碍物预测轨迹生成（通过predictedTrajectory）
 * - 路径离散点预处理（通过DiscretizedPath）
 */
void STObstacleProcessor::ComputeSTBoundaryLocalPath(const std::shared_ptr<SpeedPlannerObstacle>& obstacle,
                                                     const DiscretizedPath& discretized_path,const BehaviorState& behavior_state,
                                                     const std::vector<BoxProjectInfo>& box_project_info,
                                                     const size_t& t_index_min, const size_t& t_index_max, STBoundary& boundary) {
  std::vector<STPoint> lower_points;
  std::vector<STPoint> upper_points;
  std::vector<double> lateral_signed_distances;
  math::IntervalData<SpeedPlannerObstacle::RiskFieldInfo> risk_field_infos =
      calcRiskFieldInfos(obstacle, discretized_path,behavior_state, box_project_info, t_index_min, t_index_max, &lower_points,
                         &upper_points, &lateral_signed_distances);
  boundary = STBoundary::createInstance(lower_points, upper_points);
  boundary.set_id(obstacle->id());
  boundary.set_lateral_signed_distances(lateral_signed_distances);  // to do
  ERT_PLOG_D<<"lower_points size: "<<lower_points.size()<<" upper_points size: "<<upper_points.size();
}
/**
 * @brief 时空风险场核心计算（多维度障碍物建模）
 * @param[in] obstacle 障碍物对象（需包含预测轨迹）
 * @param[in] path 参考路径离散点
 * @param[in] box_project_info 障碍物框投影信息集合
 * 
 * @par 输入参数说明:
 * | 参数                | 类型           | 取值范围        | 单位 | 说明                     |
 * |---------------------|----------------|----------------|------|--------------------------|
 * | traj_sample         | int            | [1,5]          | -    | 轨迹采样间隔              |
 * | search_resolution   | double         | [0.5,2.0]      | 米   | 路径搜索分辨率            |
 * | obstacle_buffer     | double         | [0.0,0.5]      | 米   | 障碍物膨胀缓冲距离        |
 * 
 * @par 处理流程:
 * @startuml
 start
 partition 轨迹处理 {
   :遍历预测轨迹时间点;
   :计算初始检查点s值;
   :生成障碍物包围盒;
 }
 partition 路径搜索 {
   :前向路径搜索(检查点s递减);
   :后向路径搜索(检查点s递增);
   :动态调整搜索步长;
 }
 partition 风险场建模 {
   :构建距离插值曲线;
   :计算纵向投影速度;
   :生成时空速度限制;
 }
 :返回风险场区间数据;
 stop
 @enduml
 *
 * @note 核心算法特征:
 * 1. 双方向路径搜索机制（前向+后向）
 * 2. 动态搜索步长调整策略（0.5m~4.0m）
 * 3. 线性插值生成连续风险场数据
 *
 * @warning 需确保调用前已完成:
 * - 障碍物预测轨迹时间对齐
 * - 参考路径坐标系转换（FLU坐标系）
 */
math::IntervalData<SpeedPlannerObstacle::RiskFieldInfo> STObstacleProcessor::calcRiskFieldInfos(
    const std::shared_ptr<SpeedPlannerObstacle>& obstacle, const DiscretizedPath& path, const BehaviorState& behavior_state,
    const std::vector<BoxProjectInfo>& box_project_info, const size_t& t_index_min, const size_t& t_index_max,
    std::vector<STPoint>* lower_points, std::vector<STPoint>* upper_points,
    std::vector<double>* lateral_signed_distances) {
      
  int traj_sample = 2;  // 统一采样间隔
  bool is_follow_obstacle = checkIfFollowObstacle(obstacle,box_project_info);
  if(is_follow_obstacle){
    traj_sample = 10;
  }

  const double check_range = vehicle_config_.vehicle_param().length();
  double obstacle_buffer = 0.0;
  if (obstacle->type() == Decision::ObjectType::HEAVY_VEHICLE || obstacle->type() == Decision::ObjectType::VEHICLE
      || obstacle->type() == Decision::ObjectType::VRU || obstacle->type() == Decision::ObjectType::PEDESTRIAN) {
    obstacle_buffer = 0.2;
  }
  if(behavior_state.park_in_state_){
    obstacle_buffer = 0.1;
    vehicle_width_ = vehicle_config_.vehicle_param().width();
  }else{
    vehicle_width_ = vehicle_config_.vehicle_param().width();
  }
  double back_check_dis =
      vehicle_config_.vehicle_param().rear_edge_to_ego() + vehicle_config_.vehicle_param().front_edge_to_ego();

  const auto trajectory = obstacle->predictedTrajectory();

  auto getCheckS = [box_project_info](const size_t& index) -> double {
    if (box_project_info.empty()) {
      return 0.0;
    } else if (index <= box_project_info.front().index) {
      return box_project_info.front().project_s;
    } else if (index >= box_project_info.back().index) {
      return box_project_info.back().project_s;
    }
    for (size_t i = 0; i + 1 < box_project_info.size(); ++i) {
      if (box_project_info[i].index <= index && box_project_info[i + 1].index > index) {
        return math::lerp(box_project_info[i].project_s, box_project_info[i].index, box_project_info[i + 1].project_s,
                          box_project_info[i + 1].index, index);
      }
    }
    return 0.0;
  };
  std::vector<SpeedPlannerObstacle::RiskFieldInfo> risk_field_vector;
  std::vector<std::pair<size_t, double>> box_lateral_distance;  // index, l
  size_t max_index = std::min(t_index_max, trajectory.size() - 1);
  for (size_t i = t_index_min; i <= max_index; i += ( i+ traj_sample > max_index ? max_index - i : traj_sample)) {
    SpeedPlannerObstacle::RiskFieldInfo risk_field_info;
    risk_field_info.alpha.clear();
    risk_field_info.t = trajectory.at(i).relative_time();
    auto init_check_s = getCheckS(i);
    const double search_resolution = 1.0;
    double check_step = search_resolution;
    // TODO:坐标系问题确认
    math::Vec2d nearest_pt(0, 0);
    double min_box_distance = kPostiveInfinity;
    double check_s = init_check_s - check_step, nearest_s = init_check_s, box_dis = kPostiveInfinity;
    double relative_t = trajectory.at(i).relative_time();
    const double box_distance_ignore_threshold = max(6.0, obstacle->speed() * traj_sample * 0.1);
    std::vector<std::pair<double, double>> box_distance_temp;
    std::vector<std::pair<double, double>> alpha_temp;
    auto obs_boxes = getObsBoxes(*obstacle, relative_t, 0.0, obstacle_buffer);
    double init_box_dis = kPostiveInfinity;
    // ERT_PLOG_I<<" i =  "<<i<<"  check_s = "<<check_s<<"  init_check_s = "<<init_check_s;
    auto alpha_init = getBoxesRelation(path.evaluate(init_check_s), obs_boxes, &min_box_distance, &nearest_s,
                                       &nearest_pt, &init_box_dis);
    box_distance_temp.emplace_back(std::make_pair(init_check_s, init_box_dis));
    alpha_temp.emplace_back(std::make_pair(init_check_s, alpha_init));

    double box_dis_last = init_box_dis;
    double delta_box_dis = kMathEpsilon;
    double delta_box_dis_last = delta_box_dis;

    double alpha_last = alpha_init;
    double delta_alpha = kMathEpsilon;
    double delta_alpha_last = delta_alpha;

    while (check_s >= path.front().s()) {
      auto alpha =
          getBoxesRelation(path.evaluate(check_s), obs_boxes, &min_box_distance, &nearest_s, &nearest_pt, &box_dis);
      delta_box_dis = (box_dis - box_dis_last) / check_step;
      delta_alpha = (alpha - alpha_last) / check_step;
      for (double ds = search_resolution; ds <= check_step; ds += search_resolution) {
        double current_s = check_s + check_step - ds;
        box_distance_temp.emplace_back(std::make_pair(current_s, box_dis_last + delta_box_dis * ds));
        alpha_temp.emplace_back(std::make_pair(current_s, alpha_last + delta_alpha * ds));
      }

      if (box_dis <= -0.5 || box_dis >= 3.0 || abs(box_dis - box_dis_last) < 0.1 ||
          abs(delta_box_dis - delta_box_dis_last) < 0.1) {
        check_step = std::min(search_resolution * 4.0, check_step + search_resolution);
      } else {
        check_step = search_resolution;
      }

      if (box_dis > box_distance_ignore_threshold) {
        break;
      }

      box_dis_last = box_dis;
      delta_box_dis_last = delta_box_dis;

      alpha_last = alpha;
      delta_alpha_last = delta_alpha;

      check_s -= check_step;
    }

    check_step = search_resolution;
    check_s = init_check_s + check_step;

    box_dis_last = init_box_dis;
    delta_box_dis = kMathEpsilon;
    delta_box_dis_last = delta_box_dis;

    alpha_last = alpha_init;
    delta_alpha = kMathEpsilon;
    delta_alpha_last = delta_alpha;

    while (check_s <= path.back().s()) {
      auto alpha =
          getBoxesRelation(path.evaluate(check_s), obs_boxes, &min_box_distance, &nearest_s, &nearest_pt, &box_dis);
      delta_box_dis = (box_dis - box_dis_last) / check_step;
      delta_alpha = (alpha - alpha_last) / check_step;
      for (double ds = search_resolution; ds <= check_step; ds += search_resolution) {
        double current_s = check_s - check_step + ds;
        box_distance_temp.emplace_back(std::make_pair(current_s, box_dis_last + delta_box_dis * ds));
        alpha_temp.emplace_back(std::make_pair(current_s, alpha_last + delta_alpha * ds));
      }
      if (box_dis <= -0.5 || box_dis >= 3.0 || abs(box_dis - box_dis_last) < 0.1 ||
          abs(delta_box_dis - delta_box_dis_last) < 0.1) {
        check_step = std::min(search_resolution * 4.0, check_step + search_resolution);
      } else {
        check_step = search_resolution;
      }
      if (box_dis > box_distance_ignore_threshold) {
        break;
      }

      box_dis_last = box_dis;
      delta_box_dis_last = delta_box_dis;

      alpha_last = alpha;
      delta_alpha_last = delta_alpha;

      check_s += check_step;
    }
    std::sort(box_distance_temp.begin(), box_distance_temp.end(),
              [](const std::pair<double, double>& a, const std::pair<double, double>& b) { return a.first < b.first; });

    math::IntervalData<std::pair<double, double>> box_distance(
        box_distance_temp.front().first, search_resolution, box_distance_temp,
        [](const std::pair<double, double>& p0, const std::pair<double, double>& p1, const double x) {
          return std::make_pair(x, p0.second + (p1.second - p0.second) * (x - p0.first) / (p1.first - p0.first));
        });
    box_distance.setLeftExtrapolationFunction(
        [](const std::pair<double, double>& p0, const double x) { return std::make_pair(x, p0.second); });
    box_distance.setRightExtrapolationFunction(
        [](const std::pair<double, double>& p0, const double x) { return std::make_pair(x, p0.second); });
    double signed_box_distance = getSignedBoxDistance(obs_boxes, path, min_box_distance, nearest_s);
    risk_field_info.min_box_distance = signed_box_distance;
    risk_field_info.nearest_s = nearest_s;


    //
    std::sort(alpha_temp.begin(), alpha_temp.end(),
              [](const std::pair<double, double>& a, const std::pair<double, double>& b) { return a.first < b.first; });

    math::IntervalData<std::pair<double, double>> alpha(
        alpha_temp.front().first, search_resolution, alpha_temp,
        [](const std::pair<double, double>& p0, const std::pair<double, double>& p1, const double x) {
          return std::make_pair(x, p0.second + (p1.second - p0.second) * (x - p0.first) / (p1.first - p0.first));
        });
    alpha.setLeftExtrapolationFunction(
        [](const std::pair<double, double>& p0, const double x) { return std::make_pair(x, p0.second); });
    alpha.setRightExtrapolationFunction(
        [](const std::pair<double, double>& p0, const double x) { return std::make_pair(x, p0.second); });

    // TODO: obs双box？
    auto obs_box = obs_boxes.front();
    auto project_pt = path.evaluate(init_check_s);
    double max_kappa = 0.0;
    for (double s = init_check_s - vehicle_config_.vehicle_param().length() / 2.0;
         s < init_check_s + vehicle_config_.vehicle_param().length() / 2.0; s += 2.0) {
      auto path_pt = path.evaluate(s);
      max_kappa = std::max(max_kappa, abs(path_pt.kappa()));
    }
    risk_field_info.max_kappa = max_kappa;

    double proj_speed = (std::cos(obs_box.theta) * std::cos(project_pt.theta()) +
                         std::sin(obs_box.theta) * std::sin(project_pt.theta())) *
                        obstacle->speed();
    risk_field_info.longitudinal_speed = std::max(proj_speed, 0.0);
    risk_field_info.lateral_speed = obstacle->speed() * std::sin(obs_box.theta - project_pt.theta());
    risk_field_info.box_distance = box_distance;
    risk_field_info.alpha = alpha;
    calcTemperalSpatialSpeedLimit(&risk_field_info, obstacle->id(), obstacle->type());
    // auto speed_limit = risk_field_info.speed_limit_info.getOriginData();
    // auto box_distance_data = risk_field_info.box_distance.getOriginData();
    // for(auto box_dis_pair : box_distance_data){
    //   ERT_PLOG_I<<" s = "<<box_dis_pair.first<<"  box_dis = "<<box_dis_pair.second;
    // }
    // for(auto speed_limit_pair : speed_limit){
    //   ERT_PLOG_I<<" s = "<<speed_limit_pair.s<<"  speed_limit = "<<speed_limit_pair.speed_limit;
    // }
    // auto alpha_data = risk_field_info.alpha.getOriginData();
    // for (auto alpha_pair : alpha_data) {
    //   ERT_PLOG_I << " s = " << alpha_pair.first << "  alpha = " << alpha_pair.second ;
    // }

    if (isObstacleStatic(obstacle->type(), obstacle->predictedTrajectory(), obstacle->isStatic())) {
      auto static_risk_info = risk_field_info;
      for (size_t i = t_index_min; i < std::max(t_index_max, trajectory.size()); i += traj_sample) {
        box_lateral_distance.emplace_back(std::pair(i, signed_box_distance));
        static_risk_info.t = trajectory.at(i).relative_time();
        risk_field_vector.emplace_back(static_risk_info);
      }
      break;
    }

    box_lateral_distance.emplace_back(std::pair(i, signed_box_distance));
    risk_field_vector.emplace_back(risk_field_info);
    if( i == max_index){
      break;
    }
  }

  if (risk_field_vector.empty() || box_lateral_distance.empty()) {
    return math::IntervalData<SpeedPlannerObstacle::RiskFieldInfo>();
  }

  math::IntervalData<SpeedPlannerObstacle::RiskFieldInfo> risk_field_infos(
      risk_field_vector.front().t, risk_field_vector[1].t - risk_field_vector[0].t, risk_field_vector,
      [=](const SpeedPlannerObstacle::RiskFieldInfo& data0, const SpeedPlannerObstacle::RiskFieldInfo& data1, const double& t) {
        SpeedPlannerObstacle::RiskFieldInfo data;
        data.t = t;
        data.longitudinal_speed = math::lerp(data0.longitudinal_speed, data0.t, data1.longitudinal_speed, data1.t, t);
        data.lateral_speed = math::lerp(data0.lateral_speed, data0.t, data1.lateral_speed, data1.t, t);
        data.max_kappa = math::lerp(data0.max_kappa, data0.t, data1.max_kappa, data1.t, t);

        std::vector<std::pair<double, double>> box_dis_vector;
        std::vector<std::pair<double, double>> alpha_vector;
        double min_s = std::min(data0.box_distance.getOriginData().front().first,
                                data1.box_distance.getOriginData().front().first);
        double max_s =
            std::max(data0.box_distance.getOriginData().back().first, data1.box_distance.getOriginData().back().first);
        const double resolution = 1.0;
        double min_box_distance = kPostiveInfinity;
        double nearest_s = kPostiveInfinity;
        for (double s = min_s; s <= max_s; s += resolution) {
          double box_dis_value = math::lerp(data0.box_distance.evaluate(s).second, data0.t,
                                            data1.box_distance.evaluate(s).second, data1.t, t);
          double alpha_value = math::lerp(data0.alpha.evaluate(s).second, data0.t, data1.alpha.evaluate(s).second,
                                        data1.t, t);
          if (box_dis_value < min_box_distance) {
            min_box_distance = box_dis_value;
            nearest_s = s;
          }
          box_dis_vector.emplace_back(std::make_pair(s, box_dis_value));
          alpha_vector.emplace_back(std::make_pair(s, alpha_value));
        }
        auto obs_boxes = getObsBoxes(*obstacle, t, 0.0, obstacle_buffer);
        double signed_min_box_distance = getSignedBoxDistance(obs_boxes, path, min_box_distance, nearest_s);
        data.min_box_distance = signed_min_box_distance;
        data.nearest_s = nearest_s;

        math::IntervalData<std::pair<double, double>> box_distance(
            box_dis_vector.front().first, resolution, box_dis_vector,
            [](const std::pair<double, double>& p0, const std::pair<double, double>& p1, const double x) {
              return std::make_pair(x, p0.second + (p1.second - p0.second) * (x - p0.first) / (p1.first - p0.first));
            });
        box_distance.setLeftExtrapolationFunction(
            [](const std::pair<double, double>& p0, const double x) { return std::make_pair(x, p0.second); });
        box_distance.setRightExtrapolationFunction(
            [](const std::pair<double, double>& p0, const double x) { return std::make_pair(x, p0.second); });
        data.box_distance = box_distance;

        math::IntervalData<std::pair<double, double>> alpha(
            alpha_vector.front().first, resolution, alpha_vector,
            [](const std::pair<double, double>& p0, const std::pair<double, double>& p1, const double x) {
              return std::make_pair(x, p0.second + (p1.second - p0.second) * (x - p0.first) / (p1.first - p0.first));
            });
        alpha.setLeftExtrapolationFunction(
            [](const std::pair<double, double>& p0, const double x) { return std::make_pair(x, p0.second); });
        alpha.setRightExtrapolationFunction(
            [](const std::pair<double, double>& p0, const double x) { return std::make_pair(x, p0.second); });
        data.alpha = alpha;

        calcTemperalSpatialSpeedLimit(&data, obstacle->id(), obstacle->type());
        return data;
      });

  auto getLateralDistance = [box_lateral_distance](const size_t& index) -> double {
    auto func = [](const std::pair<size_t, double>& tp, const size_t& i) { return tp.first < i; };
    auto it_lower = std::lower_bound(box_lateral_distance.begin(), box_lateral_distance.end(), index, func);
    if (it_lower == box_lateral_distance.begin()) {
      return box_lateral_distance.front().second;
    }
    if (it_lower == box_lateral_distance.end()) {
      return box_lateral_distance.back().second;
    }
    return math::lerp((*(it_lower - 1)).second, (*(it_lower - 1)).first, (*it_lower).second, (*it_lower).first, index);
  };
  getBoundarySTPoints(risk_field_infos, box_project_info, 0.0, lower_points, upper_points, lateral_signed_distances);

  std::vector<STPoint> risk_lower_points;
  std::vector<STPoint> risk_upper_points;
  std::vector<double> risk_lateral_signed_distances;
  getBoundarySTPoints(risk_field_infos, box_project_info, 1.0, &risk_lower_points, &risk_upper_points,
                      &risk_lateral_signed_distances);
  auto boundary = STBoundary::createInstance(risk_lower_points, risk_upper_points);
  boundary.set_id(obstacle->id() + "_risk");
  boundary.setBoundaryType(obstacle->pathStBoundary().boundary_type());
  obstacle->setRiskStBoundary(boundary);

  return risk_field_infos;
}

/**
 * @brief 包围盒关系计算（几何碰撞检测核心算法）
 * @param[in] check_pt 路径检查点（需包含FLU坐标系信息）
 * @param[in] obs_boxes 障碍物包围盒集合（已膨胀处理）
 * @param[out] min_box_distance 最小障碍物距离（带符号）
 * @param[out] nearest_s 最近点对应路径s坐标
 * @param[out] nearest_pt 最近点坐标（FLU坐标系）
 * @param[out] box_dis 当前检查点的实际障碍物距离
 * @return alpha 几何关系系数（描述包围盒相对位置）
 * 
 * @par 输入参数说明:
 * | 参数                | 类型           | 取值范围        | 单位 | 说明                     |
 * |---------------------|----------------|----------------|------|--------------------------|
 * | check_pt.s          | double         | [path_min_s, path_max_s] | 米 | 当前检查点路径s坐标      |
 * | obs_boxes.size      | size_t         | >=1            | -    | 障碍物包围盒数量          |
 * 
 * @par 处理流程:
 * @startuml
start
 :生成自车包围盒;
 partition 几何关系计算 {
   :遍历自车包围盒;
   :遍历障碍物包围盒;
   :调用二次规划求解器;
   :记录最小距离和最近点;
 }
 :更新全局最小距离;
stop
 @enduml
 *
 * @note 核心算法特征:
 * - 使用IPM算法求解凸优化问题
 * - 目标函数: min 0.5x'Qx + c'x
 * - 约束条件: Gx <= h
 * 
 * @warning 需确保调用前已完成:
 * - 路径点坐标系转换（FLU坐标系）
 * - 障碍物包围盒膨胀处理
 */
double STObstacleProcessor::getBoxesRelation(const PathPt& check_pt, const std::vector<BoxInfo>& obs_boxes,
                                             double* min_box_distance, double* nearest_s, math::Vec2d* nearest_pt,
                                             double* box_dis) {
  auto ego_boxes = getEgoBoxes(check_pt);
  math::Vec2d tmp_pt;
  double alpha = getAlphaBetweenBoxes(ego_boxes, obs_boxes, &tmp_pt, box_dis);
  if (*box_dis < *min_box_distance) {
    *min_box_distance = *box_dis;
    *nearest_pt = tmp_pt;
    *nearest_s = check_pt.s();
  }
  return alpha;
}
/**
 * @brief 包围盒间几何关系系数计算（多包围盒碰撞检测）
 * @param[in] ego_boxes 自车包围盒集合（支持多包围盒结构）
 * @param[in] obs_boxes 障碍物包围盒集合（支持多包围盒结构）
 * @param[out] nearest_pt 最近点坐标（FLU坐标系）
 * @param[out] box_dis 带符号的障碍物距离（正：障碍物在右侧）
 * @return alpha 几何关系系数（描述包围盒相对位置）
 * 
 * @par 输入参数说明:
 * | 参数                | 类型           | 取值范围        | 单位 | 说明                     |
 * |---------------------|----------------|----------------|------|--------------------------|
 * | ego_boxes.size       | size_t         | >=1            | -    | 自车包围盒数量           |
 * | obs_boxes.size       | size_t         | >=1            | -    | 障碍物包围盒数量         |
 * | BoxInfo.length       | double         | [3.0,5.0]      | 米   | 包围盒长度（车类障碍物）  |
 * | BoxInfo.width        | double         | [1.8,2.5]      | 米   | 包围盒宽度（车类障碍物）  |
 * 
 * @par 处理流程:
 * @startuml
start
 partition 双循环检测 {
   :遍历自车包围盒;
   :遍历障碍物包围盒;
   :调用calcAlphaBetweenBox计算单对包围盒;
 }
 :记录最小alpha值;
 :更新最近点坐标;
stop
 @enduml
 *
 * @note 核心算法特征:
 * 1. 支持多包围盒结构（应对复杂形状障碍物）
 * 2. 带符号距离计算（正负号表示障碍物方位）
 * 3. 二次规划求解几何约束
 *
 * @warning 需确保调用前已完成:
 * - 包围盒坐标系统一（FLU坐标系）
 * - 障碍物包围盒膨胀处理（根据碰撞检测需求）
 */
double STObstacleProcessor::getAlphaBetweenBoxes(const std::vector<BoxInfo>& ego_boxes,
                                                 const std::vector<BoxInfo>& obs_boxes, math::Vec2d* nearest_pt,
                                                 double* box_dis) {
  *box_dis = kPostiveInfinity;
  double res = kPostiveInfinity, res_dis = kPostiveInfinity;
  for (const auto& ego_box : ego_boxes) {
    for (const auto& obs_box : obs_boxes) {
      math::Vec2d pt;
      double alpha = calcAlphaBetweenBox(obs_box, ego_box, false, &pt);
      double dis = signedDistanceToBoxInfo(pt, ego_box);
      if (dis < res_dis) {
        res_dis = dis;
        res = alpha;
        *nearest_pt = pt;
      }
    }
  }
  *box_dis = res_dis;
  return res;
}
/**
 * @brief 单对包围盒几何关系系数计算（二次规划核心求解器）
 * @param[in] obs_box 障碍物包围盒（已考虑膨胀处理）
 * @param[in] ego_box 自车包围盒（基于当前路径点生成）
 * @param[in] is_obs_expanded 障碍物是否已膨胀标志位
 * @param[out] nearest_pt 最近点坐标（FLU坐标系）
 * @return alpha 几何关系系数（优化问题解的分量）
 * 
 * @par 输入参数说明:
 * | 参数                | 类型           | 取值范围        | 单位 | 说明                     |
 * |---------------------|----------------|----------------|------|--------------------------|
 * | ego_box.length      | double         | [3.0,5.0]      | 米   | 自车包围盒长度           |
 * | ego_box.width       | double         | [1.8,2.5]      | 米   | 自车包围盒宽度           |
 * | obs_box.length      | double         | [3.0,5.0]      | 米   | 障碍物包围盒长度         |
 * | is_obs_expanded     | bool           | {true,false}   | -    | 障碍物包围盒膨胀标志     |
 * 
 * @par 处理流程:
 * @startuml
start
 :初始化二次规划求解器;
 partition 约束矩阵配置 {
   if (is_obs_expanded?) then (yes)
     :配置膨胀障碍物约束矩阵;
   else (no)
     :配置未膨胀障碍物约束矩阵;
   endif
 }
 :求解二次规划问题;
 if (求解失败?) then (yes)
   :重试求解;
   if (仍失败?) then (yes)
     :返回默认值0.0;
   endif
 endif
 :设置最近点坐标;
stop
 @enduml
 *
 * @note 核心数学模型:
 * 目标函数: min 0.5x'Qx + c'x
 * 约束条件: Gx <= h
 * 其中x=[pt_x, pt_y, alpha]为优化变量
 *
 * @warning 需确保调用前已完成:
 * - 自车与障碍物包围盒坐标系对齐（FLU坐标系）
 * - 二次规划求解器参数预配置（Q_, c_等）
 */
double STObstacleProcessor::calcAlphaBetweenBox(const BoxInfo& obs_box, const BoxInfo& ego_box,
                                                const bool& is_obs_expanded, math::Vec2d* nearest_pt) {
  const double& x_ego = ego_box.x;
  const double& y_ego = ego_box.y;
  const double& theta_ego = ego_box.theta;
  const double& length_ego = ego_box.length;
  const double& width_ego = ego_box.width;
  const double& x_obs = obs_box.x;
  const double& y_obs = obs_box.y;
  const double& theta_obs = obs_box.theta;
  const double& length_obs = obs_box.length;
  const double& width_obs = obs_box.width;
  if (alpha_optimizer_ == nullptr) {
    alpha_optimizer_ = std::make_shared<QuadraticProgrammingSolver>(3, 0, 9);
    alpha_optimizer_->config()->set_first_order_tol(1e-3);
    alpha_optimizer_->config()->set_equality_constraint_tol(1e-3);
    alpha_optimizer_->config()->set_complementary_tol(1e-2);
    alpha_optimizer_->config()->set_inequality_constraint_tol(1e-2);
    alpha_optimizer_->config()->set_mu_min(1e-4);
    alpha_optimizer_->config()->set_barrier_strategy(IPMConfig::ADAPTIVE);
  }
  const double sin_theta_ego = sin(theta_ego);
  const double cos_theta_ego = cos(theta_ego);
  const double sin_theta_obs = sin(theta_obs);
  const double cos_theta_obs = cos(theta_obs);
  const double length_expan_centre_ego = length_ego / 2.0;  // 膨胀中心离车头
  const double width_expan_centre_ego = width_ego / 2.0;    // 膨胀中心离车左侧
  const double length_expan_centre_obs = length_obs / 2.0;
  const double width_expan_centre_obs = width_obs / 2.0;

  if (is_obs_expanded) {
    G_ << sin_theta_ego, -cos_theta_ego, -width_ego + width_expan_centre_ego, -sin_theta_ego, cos_theta_ego,
        -width_expan_centre_ego, -cos_theta_ego, -sin_theta_ego, -length_ego + length_expan_centre_ego, cos_theta_ego,
        sin_theta_ego, -length_expan_centre_ego, sin_theta_obs, -cos_theta_obs, -width_obs + width_expan_centre_obs,
        -sin_theta_obs, cos_theta_obs, -width_expan_centre_obs, -cos_theta_obs, -sin_theta_obs,
        -length_obs + length_expan_centre_obs, cos_theta_obs, sin_theta_obs, -length_expan_centre_obs, 0, 0, -1;
    h_ << -sin_theta_ego * x_ego + cos_theta_ego * y_ego, sin_theta_ego * x_ego - cos_theta_ego * y_ego,
        cos_theta_ego * x_ego + sin_theta_ego * y_ego, -cos_theta_ego * x_ego - sin_theta_ego * y_ego,
        -sin_theta_obs * x_obs + cos_theta_obs * y_obs, sin_theta_obs * x_obs - cos_theta_obs * y_obs,
        cos_theta_obs * x_obs + sin_theta_obs * y_obs, -cos_theta_obs * x_obs - sin_theta_obs * y_obs, 0;
  } else {
    G_ << sin_theta_ego, -cos_theta_ego, -width_ego + width_expan_centre_ego, -sin_theta_ego, cos_theta_ego,
        -width_expan_centre_ego, -cos_theta_ego, -sin_theta_ego, -length_ego + length_expan_centre_ego, cos_theta_ego,
        sin_theta_ego, -length_expan_centre_ego, sin_theta_obs, -cos_theta_obs, 0, -sin_theta_obs, cos_theta_obs, 0,
        -cos_theta_obs, -sin_theta_obs, 0, cos_theta_obs, sin_theta_obs, 0, 0, 0, -1;
    h_ << -sin_theta_ego * x_ego + cos_theta_ego * y_ego, sin_theta_ego * x_ego - cos_theta_ego * y_ego,
        cos_theta_ego * x_ego + sin_theta_ego * y_ego, -cos_theta_ego * x_ego - sin_theta_ego * y_ego,
        -sin_theta_obs * x_obs + cos_theta_obs * y_obs - width_obs + width_expan_centre_obs,
        sin_theta_obs * x_obs - cos_theta_obs * y_obs - width_expan_centre_obs,
        cos_theta_obs * x_obs + sin_theta_obs * y_obs - length_obs + length_expan_centre_obs,
        -cos_theta_obs * x_obs - sin_theta_obs * y_obs - length_expan_centre_obs, 0;
  }

  alpha_optimizer_->setParam(Q_, c_, G_, h_);
  alpha_optimizer_->solve();
  auto solution = alpha_optimizer_->getX();

  auto flag = alpha_optimizer_->getExitFlag();
  if (flag != SolveStatus::SOLVED) {
    ERT_PLOG_I << "flag:  " << flag << "  ego x :" << x_ego << "  y:" << y_ego << "  theta:" << theta_ego
         << "  length:" << length_ego << "  width:" << width_ego << "  obs x :" << x_obs << "  y:" << y_obs
         << "  theta:" << theta_obs << "  length:" << length_obs << "  width:" << width_obs;
    // SFIELD_DEBUG(alpha_optimizer,
    //              "flag: {}, ego x: {}, y: {}, theta: {}, length: {}, width: {}, obs x: {}, y: {}, theta: {}, length:
    //              "
    //              "{}, width: {}",
    //              flag, x_ego, y_ego, theta_ego, length_ego, width_ego, x_obs, y_obs, theta_obs, length_obs,
    //              width_obs);
    // PERROR << "[LP FAILED] flag: " << flag;
    alpha_optimizer_->solve();
    auto solution = alpha_optimizer_->getX();
    auto re_flag = alpha_optimizer_->getExitFlag();
    // SFIELD_DEBUG(alpha_optimizer, "re_flag: {}", re_flag);
    if (re_flag != SolveStatus::SOLVED) {
      PERROR << "[LP FAILED2] flag: " << re_flag ;
      // nearest_pt???
      return 0.0;
    }
  }
  nearest_pt->set_x(solution(0));
  nearest_pt->set_y(solution(1));
  return solution(2);
}
/**
 * @brief 计算点到包围盒的带符号距离（几何碰撞检测基础）
 * @param[in] pt 待检测点坐标（FLU坐标系）
 * @param[in] box_info 包围盒信息（包含位置、朝向、尺寸）
 * @return 带符号的几何距离（正：点在包围盒外侧，负：内侧）
 * 
 * @par 输入参数说明:
 * | 参数                | 类型           | 取值范围        | 单位 | 说明                     |
 * |---------------------|----------------|----------------|------|--------------------------|
 * | box_info.length     | double         | [3.0,5.0]      | 米   | 包围盒长度（车类障碍物）  |
 * | box_info.width      | double         | [1.8,2.5]      | 米   | 包围盒宽度（车类障碍物）  |
 * 
 * @par 处理流程:
 * @startuml
start
 :坐标转换到包围盒局部坐标系;
 partition 距离计算 {
   :计算纵向距离dx;
   :计算横向距离dy;
   if (dx<=0 && dy<=0) then (内部)
     :返回最大负距离;
   elseif (dx>0 && dy<=0) then (纵向外侧)
     :返回dx正值;
   elseif (dx<=0 && dy>0) then (横向外侧)
     :返回dy正值;
   else (完全外侧)
     :返回欧式距离;
   endif
 }
stop
 @enduml
 *
 * @note 核心算法特征:
 * 1. 使用局部坐标系简化计算
 * 2. 符号规则：
 *    - 正：点在包围盒外
 *    - 负：点在包围盒内
 * 3. 优先返回轴向距离（应对轴对齐情况）
 *
 * @warning 需确保调用前已完成:
 * - 包围盒坐标系转换（FLU到局部坐标系）
 * - 包围盒参数有效性检查（长宽>0）
 */
double STObstacleProcessor::signedDistanceToBoxInfo(const math::Vec2d& pt, const BoxInfo& box_info) {
  const double x0 = pt.x() - box_info.x;
  const double y0 = pt.y() - box_info.y;
  const double dx = std::abs(x0 * box_info.cos_heading + y0 * box_info.sin_heading) - 0.5 * box_info.length;
  const double dy = std::abs(x0 * box_info.sin_heading - y0 * box_info.cos_heading) - 0.5 * box_info.width;
  if (dx <= 0 && dy <= 0) {
    return std::max(dx, dy);
  } else if (dx > 0 && dy <= 0) {
    return dx;
  } else if (dx <= 0 && dy > 0) {
    return dy;
  }
  return hypot(dx, dy);
}
/**
 * @brief 计算带符号的障碍物距离（横向位置关系判断）
 * @param[in] obs_boxes 障碍物包围盒集合（需至少包含一个包围盒）
 * @param[in] path 参考路径离散点集合
 * @param[in] unsigned_distance 原始无符号距离（需>=0）
 * @param[in] s 路径s坐标（用于计算自车位置）
 * @return 带符号的障碍物距离（正：障碍物在右侧，负：左侧）
 * 
 * @par 输入参数说明:
 * | 参数                | 类型           | 取值范围        | 单位 | 说明                     |
 * |---------------------|----------------|----------------|------|--------------------------|
 * | s                   | double         | [path_min_s, path_max_s] | 米 | 路径查询点s坐标      |
 * | unsigned_distance   | double         | >=0.0          | 米   | 原始障碍物距离           |
 * 
 * @par 处理流程:
 * @startuml
start
 :获取自车中心点坐标;
 :计算障碍物中心点FLU坐标;
 :转换到自车坐标系;
 if (横向坐标 < 0?) then (yes)
   :符号设为-1;
 else
   :符号设为+1;
 endif
 :返回带符号距离;
stop
 @enduml
 *
 * @note 核心算法特征:
 * 1. 使用自车中心坐标系判断方位
 * 2. 符号规则：
 *    - 正：障碍物在自车右侧
 *    - 负：障碍物在自车左侧
 * 3. 基于路径点坐标系转换（FLU到自车坐标系）
 *
 * @warning 需确保调用前已完成:
 * - 障碍物包围盒中心坐标有效性检查
 * - 路径点坐标系转换（FLU坐标系）
 */
double STObstacleProcessor::getSignedBoxDistance(const std::vector<BoxInfo>& obs_boxes, const DiscretizedPath& path,
                                                 const double unsigned_distance, const double s) {
  int dis_sign = 1;
  double res = unsigned_distance;
  if (res <= 0.0) {
    return 0.0;
  }
  if (res > 0.0) {
    auto ego_center = GetADCCenterPointFromPathPoint(path.evaluate(s));
    double dx = obs_boxes.front().x - ego_center.x();
    double dy = obs_boxes.front().y - ego_center.y();
    auto obs_center_ego_frame_y = -std::sin(ego_center.theta()) * dx + std::cos(ego_center.theta()) * dy;
    dis_sign = obs_center_ego_frame_y < 0.0 ? -1 : 1;
  }
  res = dis_sign * res;
  return res;
}
/**
 * @brief 生成时空速度限制（风险场建模核心）
 * @param[in,out] risk_field_info 风险场信息（需包含纵向速度和alpha数据）
 * @param[in] obstacle_id 障碍物唯一标识符
 * @param[in] type 障碍物类型（影响速度限制策略）
 * 
 * @par 输入参数说明:
 * | 参数                | 类型           | 取值范围        | 单位 | 说明                     |
 * |---------------------|----------------|----------------|------|--------------------------|
 * | risk_field_info.longitudinal_speed | double | >=0.0 | m/s | 纵向基准速度          |
 * | risk_field_info.alpha | IntervalData | [0.5,4.0] | -    | 风险系数插值数据       |
 * | type                | ObstacleType  | [kTypeCar, kTypePedestrian...] | - | 障碍物分类标识      |
 * 
 * @par 处理流程:
 * @startuml
start
 :初始化横向距离比例表;
 :初始化速度增量表;
 partition 速度限制生成 {
   :遍历alpha插值数据;
   :查表获取速度增量;
   :计算最终速度限制值;
 }
 :构建时空速度限制插值;
 :配置外推插值函数;
stop
 @enduml
 *
 * @note 核心参数配置:
 * - 横向距离比例表: [0.0, 0.1,...,3.0]
 * - 速度增量表: [-3.0, -1.0,...,30.0]
 * - 速度限制计算公式: 
 *    speed_limit = max(0.0, longitudinal_speed + delta_speed)
 *
 * @warning 需确保调用前已完成:
 * - 风险场alpha数据初始化
 * - 障碍物类型正确分类
 * - 纵向速度参数有效性校验
 */
void STObstacleProcessor::calcTemperalSpatialSpeedLimit(SpeedPlannerObstacle::RiskFieldInfo* risk_field_info,
                                                        const std::string& obstacle_id,
                                                        const Decision::ObjectType& type) {
  auto box_distance_vector = risk_field_info->box_distance.getOriginData();
  if (box_distance_vector.size() < 2) {
    ERT_PLOG_I<<"box_distance_vector size less than 2 !!!!!!!!!!!!!";
    return;
  }

  auto alpha_vector = risk_field_info->alpha.getOriginData();
  if (alpha_vector.size() < 2) {
    ERT_PLOG_I<<"alpha_vector size less than 2 !!!!!!!!!!!!!";
    return;
  }

  std::vector<double> lateral_distance_ratio_table = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6,
                                                      0.7, 1.0, 1.2, 1.5, 2.0, 2.5, 3.0};
  std::vector<double> delta_speed_limit_table;

  // delta_speed_limit_table = {0.0, 0.05, 0.2, 0.9, 1.6, 2.5, 3.6, 5.0, 8.0, 12.5, 14.5, 16.666, 20.0, 30.0};
  delta_speed_limit_table = {-3.0, -1.0, 0.0, 1.0, 3.0, 5.0, 13.0, 20.0, 25.0, 30.0, 30.0, 30.0, 30.0, 30.0};
  // if (max_kappa >= 0.002F) {
  //   delta_speed_limit_table = {-5.0, -2.0, 0.0, 0.0, 1.0, 3.0, 6.0, 10.0, 13.0, 18.0, 25.0, 30.0, 30.0, 30.0};
  // }
  std::vector<double> alpha_ratio_table = {1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 2.0, 2.2, 2.5, 3.0, 3.5, 4.0};

  std::vector<SpatialSpeedLimit> speed_limit_vec;
  // const double interval = box_distance_vector[1].first - box_distance_vector[0].first;
  // for (auto box_distance : box_distance_vector) {
  //   if (param_.enable_lateral_risk_function()) {
  //     double speed_limit = getSpeedLimit(box_distance.second, risk_field_info->longitudinal_speed, type);
  //     SpatialSpeedLimit spatial_speed_limit(box_distance.first, speed_limit, obstacle_id);
  //     speed_limit_vec.emplace_back(spatial_speed_limit);
  //   } else {
  //     double deltaSpeedLimit =
  //         math::TableLookUp1D(lateral_distance_ratio_table, delta_speed_limit_table, box_distance.second);
  //     SpatialSpeedLimit spatial_speed_limit(
  //         box_distance.first, fmax(0.0, risk_field_info->longitudinal_speed + deltaSpeedLimit), obstacle_id);
  //     speed_limit_vec.emplace_back(spatial_speed_limit);
  //   }
  // }

  const double interval = alpha_vector[1].first - alpha_vector[0].first;
  for (auto alpha : alpha_vector) {
      double deltaSpeedLimit =
          math::TableLookUp1D(alpha_ratio_table, delta_speed_limit_table, alpha.second);
      SpatialSpeedLimit spatial_speed_limit(
          alpha.first, fmax(0.0, risk_field_info->longitudinal_speed + deltaSpeedLimit), obstacle_id);
      speed_limit_vec.emplace_back(spatial_speed_limit);
      // ERT_PLOG_I<<"alpha = "<<alpha.second<<"  deltaSpeedLimit = "<<deltaSpeedLimit;
  }
  math::IntervalData<SpatialSpeedLimit> speed_limit(
      speed_limit_vec.front().s, interval, speed_limit_vec,
      [](const SpatialSpeedLimit& p0, const SpatialSpeedLimit& p1, const double x) {
        return SpatialSpeedLimit(
            x, p0.speed_limit + (p1.speed_limit - p0.speed_limit) * (x - p0.s) / (p1.s - p0.s), p0.id);
      });

  speed_limit.setLeftExtrapolationFunction([](const SpatialSpeedLimit& p0, const double x) {
    return SpatialSpeedLimit(x, p0.speed_limit, p0.id);
  });
  speed_limit.setRightExtrapolationFunction([](const SpatialSpeedLimit& p0, const double x) {
    return SpatialSpeedLimit(x, p0.speed_limit, p0.id);
  });
  speed_limit.setDifferentiationFunction(
      [](const SpatialSpeedLimit& p0, const SpatialSpeedLimit& p1, const double s) {
        return SpatialSpeedLimit(s, (p1.speed_limit - p0.speed_limit) / (p1.s - p0.s), p0.id);
      });
  risk_field_info->speed_limit_info = speed_limit;
}
/**
 * @brief 计算横向安全距离约束下的速度限制（安全模型计算）
 * @param[in] box_dis 障碍物横向距离（带符号）
 * @param[in] longitudinal_speed 自车纵向速度
 * @param[in] type 障碍物类型（影响安全距离参数）
 * @return 建议速度限制值（>=0）
 * 
 * @par 输入参数说明:
 * | 参数                | 类型           | 取值范围        | 单位 | 说明                     |
 * |---------------------|----------------|----------------|------|--------------------------|
 * | box_dis             | double         | [-∞,+∞]        | 米   | 带符号横向距离           |
 * | longitudinal_speed  | double         | >=0.0          | m/s  | 自车纵向速度基准值       |
 * 
 * @par 处理流程:
 * @startuml
start
 partition 参数配置 {
   :根据障碍物类型设置c值;
   :锥桶/不可逾越障碍物: c=0.0;
   :自行车/行人: c=0.4;
 }
 :计算安全距离方程参数;
 if (判别式>=0?) then (yes)
   :求解二次方程正根;
 else (no)
   :返回0.0;
 endif
 :返回安全速度限制;
stop
 @enduml
 *
 * @note 核心数学模型:
 * L_lateral = a*ΔV² + b*V_ego + c_min
 * 其中：
 * - a = 0.002（可标定参数）
 * - b = 0.0（可标定参数）
 * - c_min = 根据障碍物类型变化
 * - ΔV = 相对纵向速度
 * - V_ego = 自车纵向速度
 *
 * @warning 需确保调用前已完成:
 * - 障碍物类型正确分类
 * - 纵向速度参数有效性校验
 */
double STObstacleProcessor::getSpeedLimit(const double box_dis, const double longitudinal_speed,
                                          const Decision::ObjectType& type) {
  // L_lateral = a*ΔV^2+b*V_ego+c_min
  // - L_lateral：横向安全距离（m）
  // - ΔV：ego与障碍物的相对纵向速度（m/s）
  // - V_ego：ego绝对纵向速度（m/s）
  // - c_min：最小横向安全距离（m/s）
  // - a = 0.002（可标定）
  // - b = 0.0（可标定）
  // - c_min = 0.25m（可标定）
  // 行人速度暂取0
  double a = param_.lateral_risk_function_a(), b = param_.lateral_risk_function_b(),
         c = param_.lateral_risk_function_c(), v_o = longitudinal_speed;
  if (type == Decision::ObjectType::UNKNOWN) {
    c = 0.0;
  } else if (type == Decision::ObjectType::PEDESTRIAN || type == Decision::ObjectType::VRU) {
    c = 0.4;
  }
  double temp_parameter = fmax(0.0, pow(b - 2 * a * v_o, 2) - 4 * a * (v_o * v_o * a + c - fabs(box_dis)));
  double res = fmax(0.0, (-(b - 2 * a * v_o) + sqrt(temp_parameter)) / (2 * a + kMathEpsilon));
  return res;
}
/**
 * @brief 生成ST边界点集合（时空走廊生成核心）
 * @param[in] risk_infos 风险场插值数据（需包含时间序列）
 * @param[in] box_project_info 障碍物投影信息集合
 * @param[in] buffer 安全缓冲距离（需>=0）
 * @param[out] lower_points ST下边界点集合
 * @param[out] upper_points ST上边界点集合
 * @param[out] lateral_signed_distances 横向签名距离集合
 * 
 * @par 输入参数说明:
 * | 参数                | 类型           | 取值范围        | 单位 | 说明                     |
 * |---------------------|----------------|----------------|------|--------------------------|
 * | buffer              | double         | >=0.0          | 米   | 安全缓冲距离             |
 * | risk_infos.t        | vector<double> | [0.0, time_horizon] | 秒 | 时间网格序列            |
 * 
 * @par 处理流程:
 * @startuml
start
 partition 时间范围确定 {
   :正向遍历找最小时间;
   :逆向遍历找最大时间;
 }
 partition 边界点生成 {
   :遍历时间网格;
   :评估风险场信息;
   if (存在侵入区域?) then (yes)
     :记录ST边界点;
   else (no)
     :跳过当前时间点;
   endif
 }
 :处理最大时间边界点;
stop
 @enduml
 *
 * @note 核心算法特征:
 * 1. 双阶段时间搜索机制（正向+逆向）
 * 2. 动态时间采样间隔（0.1s->0.2s）
 * 3. 基于风险场的侵入区域判断
 *
 * @warning 需确保调用前已完成:
 * - 风险场数据的时间对齐
 * - 障碍物投影信息预处理
 */
void STObstacleProcessor::getBoundarySTPoints(math::IntervalData<SpeedPlannerObstacle::RiskFieldInfo>& risk_infos,
                                              const std::vector<BoxProjectInfo>& box_project_info, double buffer,
                                              std::vector<STPoint>* lower_points, std::vector<STPoint>* upper_points,
                                              std::vector<double>* lateral_signed_distances) {
  auto getLateralDistance = [box_project_info](const double& t) -> double {
    auto func = [](const BoxProjectInfo& tp, const double& t) { return tp.time < t; };
    auto it_lower = std::lower_bound(box_project_info.begin(), box_project_info.end(), t, func);
    if (it_lower == box_project_info.begin()) {
      return box_project_info.front().project_l;
    }
    if (it_lower == box_project_info.end()) {
      return box_project_info.back().project_l;
    }
    return math::lerp((*(it_lower - 1)).project_l, (*(it_lower - 1)).time, (*it_lower).project_l, (*it_lower).time, t);
  };
  double max_t = risk_infos.getOriginData().back().t;
  double min_t = risk_infos.getOriginData().front().t;
  double t_interval = 0.1;

  for (double t = min_t; t <= max_t; t += t_interval) {
    auto risk_info = risk_infos.evaluate(t);
    if (abs(risk_info.min_box_distance) <= buffer) {
      min_t = t;
      break;
    }
  }

  for (double t = max_t; t > min_t; t -= t_interval) {
    auto risk_info = risk_infos.evaluate(t);
    if (abs(risk_info.min_box_distance) <= buffer) {
      max_t = t;
      break;
    }
  }

  //TODO: 优化Uturn多次入侵问题
  for(const auto& risk_info : risk_infos.getOriginData()){
    if(risk_info.t < min_t){
      continue;
    }
    if(risk_info.t > max_t){
      break;
    }
    if(abs(risk_info.min_box_distance) > buffer){
      max_t = risk_info.t;
      break;
    }
  }

  t_interval = 0.2;
  for (double t = min_t; t < max_t - 1e-5; t += t_interval) {
    double s_lower = kPostiveInfinity, s_upper = -kPostiveInfinity;
    auto risk_info = risk_infos.evaluate(t);
    // ERT_PLOG_I<<">>>>>>>>>>>>>>  t = "<<t;
    // for(auto &y :risk_info.box_distance.getOriginData()){
    //   ERT_PLOG_I<<"  >>>>>>>y = "<<y;
    // }

    if (!risk_info.s_invade_in(buffer, &s_lower) || !risk_info.s_invade_out(buffer, &s_upper)) {
      continue;
    }
    lower_points->emplace_back(STPoint(s_lower, t));
    upper_points->emplace_back(STPoint(s_upper, t));
    lateral_signed_distances->emplace_back(getLateralDistance(t));
  }

  double s_lower = kPostiveInfinity, s_upper = -kPostiveInfinity;
  auto risk_info = risk_infos.evaluate(max_t);
  if (risk_info.s_invade_in(buffer, &s_lower) && risk_info.s_invade_out(buffer, &s_upper)) {
    lower_points->emplace_back(STPoint(s_lower, max_t));
    upper_points->emplace_back(STPoint(s_upper, max_t));
    lateral_signed_distances->emplace_back(getLateralDistance(max_t));
  }
}

// SpeedPlannerObstacle::LateralRiskInfo STObstacleProcessor::getLateralRiskInfo(const BoxProjectInfo& project_info,
//                                                                   const proto::TrajectoryPoint& traj_pt,
//                                                                   const math::Box2d& obs_box, const double&
//                                                                   obs_speed, const double& box_lateral_distance) {
//   auto project_pt = path_.evaluate(project_info.project_s);
//   double cos_delta_theta = std::cos(project_pt.theta() - obs_box.heading());
//   double s_bound1 = project_info.project_s - obs_box.half_length() * cos_delta_theta;
//   double s_bound2 = project_info.project_s + obs_box.half_length() * cos_delta_theta;
//   double s_lower = fmin(s_bound1, s_bound2), s_upper = fmax(s_bound1, s_bound2);
//   SpeedPlannerObstacle::LateralRiskInfo res;
//   res.s_lower = s_lower - vehicle_config_.vehicle_param().front_edge_to_ego();
//   res.s_upper = s_upper - vehicle_config_.vehicle_param().front_edge_to_ego();
//   res.t = traj_pt.relative_time();
//   DiscretizedPath nearby_path_pts;
//   path_.getPathPts(s_lower, s_upper, &nearby_path_pts);
//   res.max_kappa = 0.0;
//   for (const auto& pt : nearby_path_pts) {
//     res.max_kappa = fmax(res.max_kappa, abs(pt.kappa()));
//   }

//   double proj_speed =
//       (obs_box.cos_heading() * std::cos(project_pt.theta()) + obs_box.sin_heading() * std::sin(project_pt.theta())) *
//       obs_speed;
//   res.longitudinal_speed = std::fmax(0, proj_speed);
//   res.delta_theta = math::NormalizeAngle(obs_box.heading() - project_pt.theta());
//   res.lateral_speed = obs_speed * std::sin(obs_box.heading() - project_pt.theta());

//   res.box_lateral_distance = box_lateral_distance;
//   CalcLateralRiskSpeedLimit(&res);

//   return res;
// }

// void STObstacleProcessor::CalcLateralRiskSpeedLimit(SpeedPlannerObstacle::LateralRiskInfo* lateral_risk) {
//   // TOOD: 未来使用v(s, t)代替，初版继续使用v(t)
//   // Calc longitudinal_speed and speed_limit of the lateral_risk_info:
//   const double half_adc_width = 0.5 * vehicle_config_.vehicle_param().width();
//   const double L = vehicle_config_.vehicle_param().front_edge_to_ego();
//   std::vector<double> longitudinal_distance_table = {1.0, 10.0, 20.0, 40.0, 60.0, 80.0, 100.0, 110.0, 150.0};
//   std::vector<double> lateral_risk_distance_thres_table = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

//   lateral_risk_distance_thres_table = {1.0, 1.0, 1.0, 1.0, 0.9, 0.6, 0.2, 0.15, 0.1};

//   std::vector<double> lateral_distance_ratio_table = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6,
//                                                       0.7, 1.0, 1.2, 1.5, 2.0, 2.5, 3.0};
//   double box_lat_distance = std::abs(lateral_risk->box_lateral_distance);
//   std::vector<double> delta_speed_limit_table = {0.0, 0.05, 0.2,  0.9,  1.6,    2.5,  3.6,
//                                                  5.0, 8.0,  12.5, 14.5, 16.666, 20.0, 30.0};

//   // delta_speed_limit_table = {-3.0, -1.0, 0.0, 1.0, 3.0, 5.0, 13.0, 20.0, 25.0, 30.0, 30.0, 30.0, 30.0, 30.0};

//   if (std::abs(lateral_risk->max_kappa) >= 0.002F) {
//     // conservative params for curved path;
//     delta_speed_limit_table = {-5.0, -2.0, 0.0, 0.0, 1.0, 3.0, 6.0, 10.0, 13.0, 18.0, 25.0, 30.0, 30.0, 30.0};
//   }

//   double longitudinal_dist = lateral_risk->s_lower;
//   // To Do
//   longitudinal_dist = fmax(longitudinal_dist, 1.0);
//   double lat_risk_distance_thres =
//       math::TableLookUp1D(longitudinal_distance_table, lateral_risk_distance_thres_table, longitudinal_dist);
//   double lateral_risk_threshold = 3.0F * lat_risk_distance_thres;
//   // if (config_manager->vehicle_type() == proto::SensorTable::BT) {
//   //   lateral_risk_threshold = half_adc_width + lat_risk_distance_thres;
//   // }
//   lateral_risk->has_lateral_risk = (box_lat_distance <= lateral_risk_threshold);
//   double lat_dist_ratio = box_lat_distance / (lat_risk_distance_thres + kMathEpsilon);
//   double deltaSpeedLimit = math::TableLookUp1D(lateral_distance_ratio_table, delta_speed_limit_table,
//   lat_dist_ratio); lateral_risk->time_spatial_speed_limit = fmax(0.0, lateral_risk->longitudinal_speed +
//   deltaSpeedLimit);
// }

//----------------------------------------------------common
/**
 * @brief 生成自车包围盒集合（几何碰撞检测基础）
 * @param[in] check_pt 路径检查点（包含位置和朝向信息）
 * @param[in] half_s_buffer 纵向缓冲半长（默认0.0）
 * @param[in] half_l_buffer 横向缓冲半宽（默认0.0）
 * @return 自车包围盒集合（当前实现为单包围盒）
 * 
 * @par 输入参数说明:
 * | 参数                | 类型           | 取值范围        | 单位 | 说明                     |
 * |---------------------|----------------|----------------|------|--------------------------|
 * | check_pt.x          | double         | [-∞,+∞]        | 米   | FLU坐标系x坐标           |
 * | check_pt.y          | double         | [-∞,+∞]        | 米   | FLU坐标系y坐标           |
 * | check_pt.theta      | double         | [-π,+π]        | 弧度 | 自车朝向角              |
 * | half_s_buffer       | double         | >=0.0          | 米   | 纵向扩展半长度           |
 * | half_l_buffer       | double         | >=0.0          | 米   | 横向扩展半宽度           |
 *
 * @note 核心特征:
 * 1. 基于车辆参数构建基准包围盒:
 *    - 长度 = 车辆长度 + 2*half_s_buffer
 *    - 宽度 = 车辆宽度 + 2*half_l_buffer
 * 2. 当前实现返回单个包围盒（后续可扩展多包围盒结构）
 *
 * @warning 需确保:
 * - 路径点已完成坐标系转换（FLU坐标系）
 * - 缓冲参数需根据障碍物类型合理设置（默认0.0）
 */
std::vector<BoxInfo> STObstacleProcessor::getEgoBoxes(const PathPt& check_pt, double half_s_buffer,
                                                      double half_l_buffer) const {
  std::vector<BoxInfo> res;

  auto center_pt = GetADCCenterPointFromPathPoint(check_pt);
  BoxInfo box(center_pt.x(), center_pt.y(), center_pt.theta(),
              vehicle_config_.vehicle_param().length() + half_s_buffer * 2.0,
              vehicle_width_ + half_l_buffer * 2.0);
  res.emplace_back(box);

  return res;
}
/**
 * @brief 生成自车包围盒集合（几何碰撞检测基础）
 * @param[in] check_pt 路径检查点（包含位置和朝向信息）
 * @param[in] half_s_buffer 纵向缓冲半长（默认0.0）
 * @param[in] half_l_buffer 横向缓冲半宽（默认0.0）
 * @return 自车包围盒集合（当前实现为单包围盒）
 * 
 * @par 输入参数说明:
 * | 参数                | 类型           | 取值范围        | 单位 | 说明                     |
 * |---------------------|----------------|----------------|------|--------------------------|
 * | check_pt.x          | double         | [-∞,+∞]        | 米   | FLU坐标系x坐标           |
 * | check_pt.y          | double         | [-∞,+∞]        | 米   | FLU坐标系y坐标           |
 * | check_pt.theta      | double         | [-π,+π]        | 弧度 | 自车朝向角              |
 * | half_s_buffer       | double         | >=0.0          | 米   | 纵向扩展半长度           |
 * | half_l_buffer       | double         | >=0.0          | 米   | 横向扩展半宽度           |
 * 
 * @par 处理流程:
 * @startuml
start
 :计算自车中心点坐标;
 :基于车辆参数构建包围盒;
 :应用缓冲扩展参数;
 :生成包围盒集合;
stop
 @enduml
 *
 * @note 核心参数配置:
 * - 包围盒长度 = 车辆长度 + 2*half_s_buffer
 * - 包围盒宽度 = 车辆宽度 + 2*half_l_buffer
 * - 当前实现返回单个包围盒（后续可扩展多包围盒结构）
 *
 * @warning 需确保调用前已完成:
 * - 路径点坐标系转换（FLU坐标系）
 * - 车辆参数正确配置（长度/宽度等）
 */
std::vector<BoxInfo> STObstacleProcessor::getObsBoxes(const SpeedPlannerObstacle& obs, const double& t, double half_s_buffer,
                                                      double half_l_buffer) const {
  // TODO: @lvjidong seperate-box to be finished
  std::vector<BoxInfo> res;
  math::Box2d box = obs.getBoundingBoxAtTime(t);
  BoxInfo expand_box(box.center_x(), box.center_y(), box.heading(), box.length() + half_s_buffer * 2.0,
                     box.width() + half_l_buffer * 2.0);
  res.emplace_back(expand_box);
  return res;
}
/**
 * @brief 计算自车中心点坐标（几何变换核心）
 * @param[in] curr_point 当前路径点（包含位置和朝向信息）
 * @return 自车中心点坐标（FLU坐标系）
 * 
 * @par 输入参数说明:
 * | 参数                | 类型           | 取值范围        | 单位 | 说明                     |
 * |---------------------|----------------|----------------|------|--------------------------|
 * | curr_point.x        | double         | [-∞,+∞]        | 米   | FLU坐标系x坐标           |
 * | curr_point.y        | double         | [-∞,+∞]        | 米   | FLU坐标系y坐标           |
 * | curr_point.theta    | double         | [-π,+π]        | 弧度 | 自车朝向角              |
 * 
 * @par 处理流程:
 * @startuml
start
 :获取当前路径点theta角度;
 :计算前后轴距中心偏移量;
 :计算中心点x坐标:
  x = curr_point.x + cos(theta) * (front_edge - rear_edge)/2;
 :计算中心点y坐标:
  y = curr_point.y + sin(theta) * (front_edge - rear_edge)/2;
 :保持中心点theta与原路径点一致;
stop
 @enduml
 *
 * @note 核心参数配置:
 * - 前轴到ego距离: vehicle_config_.vehicle_param().front_edge_to_ego()
 * - 后轴到ego距离: vehicle_config_.vehicle_param().rear_edge_to_ego()
 * - 中心偏移量 = (前轴距离 - 后轴距离)/2
 *
 * @warning 需确保调用前已完成:
 * - 车辆参数正确配置（前后轴距参数）
 * - 路径点坐标系转换（FLU坐标系）
 */
PathPt STObstacleProcessor::GetADCCenterPointFromPathPoint(const PathPt& curr_point) const {
  double theta = curr_point.theta();
  // TODO: 考虑倒车情况
  PathPt center_point;
  center_point.set_x(curr_point.x() + std::cos(theta) *
                                          (vehicle_config_.vehicle_param().length()/2.0 -
                                           vehicle_config_.vehicle_param().rear_edge_to_ego()));
  center_point.set_y(curr_point.y() + std::sin(theta) *
                                          (vehicle_config_.vehicle_param().length()/2.0 -
                                           vehicle_config_.vehicle_param().rear_edge_to_ego()));
  center_point.set_theta(curr_point.theta());
  return center_point;
}

bool STObstacleProcessor::checkIfFollowObstacle(const std::shared_ptr<SpeedPlannerObstacle>& obstacle,
                                                const std::vector<BoxProjectInfo>& box_project_info) {
  if (box_project_info.empty()) {
    return false;
  }
  double follow_check_threshold = 3.0;
  for (const auto& box_info : box_project_info) {
    if ((box_info.time < follow_check_threshold && box_info.project_l > vehicle_config_.vehicle_param().width()) ||
        obstacle->getLongitudinalOdTag() != LongitudinalOdTag::FOLLOW) {
      return false;
    }
  }
  return true;
}

}  // namespace gpal::pnc::planning