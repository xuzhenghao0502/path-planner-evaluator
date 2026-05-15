/**
 * @file speed_wall.cpp
 * @brief 多类型速度墙生成器
 * @details 本类负责生成速度规划所需的各种类型速度墙
 */
#include "speed_preprocessor/speed_wall.h"

namespace gpal::pnc::planning {
/**
 * @brief 速度墙处理器初始化（配置加载）
 * @param[in] time_horizon 时间规划范围（单位：秒）
 * @return bool 初始化状态（true=成功）
 * 
 * @par 输入/输出参数说明:
 * | 参数          | 类型        | 取值范围       | 说明                     |
 * |---------------|-------------|----------------|--------------------------|
 * | time_horizon  | const double& | (0.0, ∞)      | 速度墙时间范围           |
 * | 返回值        | bool        | {true, false}  | 初始化是否成功           |
 *
 * @par 处理流程:
 * @startuml
 :设置时间范围参数;
 :获取配置管理器单例;
 :加载车辆参数配置;
 :加载速度预处理配置;
 @enduml
 *
 * @note 关键配置项:
 * - vehicle_config().vehicle_param: 车辆物理参数
 * - SpeedPreProcessorConfig: 速度预处理相关参数
 *
 * @warning 前置条件:
 * - 配置系统必须已正确初始化
 * - 必须在使用其他SpeedWallProcessor方法前调用
 */
bool SpeedWallProcessor::init(const double& time_horizon) {
  time_horizon_ = time_horizon;
  config_manager_ = Singleton<ConfigManager>::get_instance();
  CHECK_NOTNULL(config_manager_);

  vehicle_param_ = config_manager_->vehicle_config().vehicle_param();
  speed_preprocessor_config_ = config_manager_->getConfig<SpeedPreProcessorConfig>("SpeedPreProcessorConfig");
  return true;
}
/**
 * @brief 生成交通灯相关速度墙（含掉头场景处理）
 * @param[in] local_view 局部环境信息（含定位/底盘数据）
 * @param[in] reference_line_info 参考线信息
 * @param[in] decision_result 决策结果
 * @param[in] is_turn_around 是否处于掉头状态
 * @param[out] speed_walls 速度墙输出容器
 * 
 * @par 输入/输出参数说明:
 * | 参数                | 类型                     | 取值范围        | 说明                     |
 * |---------------------|--------------------------|----------------|--------------------------|
 * | local_view          | const LocalView&        | -              | 车辆实时感知数据         |
 * | reference_line_info | ReferenceLineInfo*      | [nullptr, 有效]| 道路参考线信息           |
 * | decision_result     | const DecisionResult&   | -              | 交通灯决策结果           |
 * | is_turn_around      | const bool&             | {true, false}  | 当前是否处于掉头状态     |
 * | speed_walls         | std::vector<SpeedWall>* | -              | 速度墙输出容器           |
 *
 * @par 处理流程:
 * @startuml
 :检查参考线有效性;
 if (无效?) then (是)
   :清空速度墙输出容器;
 else (否)
   :获取车辆当前位置;
   :查询前方停止线;
 partition 掉头场景处理 {
   if (启用掉头停车?) then (是)
     :检测持续停车状态;
     if (车速<0.01m/s持续10次?) then (是)
       :解除掉头停车需求;
     else (否)
       :保持停车需求;
     endif
   endif
 }
 if (存在停止线且需要停车?) then (是)
   :计算安全停车距离;
   if (允许越线?) then (是)
     :动态调整速度墙位置;
   else (否)
     :使用严格停车位置;
   endif
   :生成TFL速度墙;
 endif
 @enduml
 *
 * @note 关键算法:
 * - 停车距离计算: s = v*t_delay - v²/(2*a_min)
 * - 越线判断: 当制动距离 < 允许越线距离+当前位置时调整速度墙
 *
 * @warning 阈值参数:
 * - 掉头停车计数阈值: 10次
 * - 最大允许越线距离: 15m
 * - 基础安全距离: 1.0m
 * - 最小加速度: -2.5m/s²
 */
void SpeedWallProcessor::addTflSpeedWall(const LocalView& local_view, ReferenceLineInfo* reference_line_info,
                                         const DecisionResult& decision_result, const bool& is_turn_around,
                                         std::vector<SpeedWall>* speed_walls) {
  if (reference_line_info == nullptr || !reference_line_info->isValid()) {
    return;
  }
  const float stop_buffer = 1.0;
  auto ltm_point = local_view.getLocalizationPtr()->vehicleAlignPosePoint();

  auto ego_reference_point = reference_line_info->ref_line().getReferencePoint(ltm_point.x(), ltm_point.y());
  auto stop_lines = reference_line_info->ref_line().getStopLinesFromSRange(ego_reference_point.local_s());

  // vitual stop
  //  掉头场景处理
  if (speed_preprocessor_config_.enable_u_turn_stop()) {
    constexpr int stopped_cnt_th = 10;
    static int stopped_cnt = 0;
    if (is_turn_around) {
      /// 停止线停车判定
      if (stopped_cnt <= stopped_cnt_th && !stop_lines.empty() && local_view.getChassisPtr()->Speed() >= 0.01f) {
        require_stop_for_turn_around_ = true;
      }
      if (local_view.getChassisPtr()->Speed() < 0.01f) {
        stopped_cnt++;
        if (stopped_cnt > stopped_cnt_th) {
          require_stop_for_turn_around_ = false;
        }
      }
    } else {
      require_stop_for_turn_around_ = false;
      stopped_cnt = 0;
    }
  }

  if (!stop_lines.empty() && (require_stop_for_turn_around_ || decision_result.getTrafficLightDecision().is_need_stop)) {
    double stop_s =
        stop_lines.front().s - ego_reference_point.local_s() - vehicle_param_.front_edge_to_ego() - stop_buffer;

    double desired_acc_min = -2.5;
    double t_delay = 0.2;                                    // 延迟
    double allowed_cross = 15.0;                             // 最大越线距离(TODO: 自车长)
    double ego_speed = local_view.getChassisPtr()->Speed();  // 自车速度
    double stop_distance = ego_speed * t_delay - ego_speed * ego_speed / (2 * desired_acc_min);  // 制动距离
    double tfl_block_s = 300;
    double safe_distance = 1.0;
    double tfl_to_front_edge = fmax(stop_s, kMathEpsilon);
    if (speed_preprocessor_config_.allow_exceed_stop_line()) {
      if (stop_distance < allowed_cross + tfl_to_front_edge) {  // 越线范围内可以刹停
        tfl_block_s = fmax(tfl_to_front_edge, stop_distance);
        tfl_block_s = fmax(tfl_block_s, kMathEpsilon);
      } else if (tfl_to_front_edge > allowed_cross) {  // 距离红绿灯较远且车速较快
        tfl_block_s = tfl_to_front_edge;
      }
    } else {
      tfl_block_s = tfl_to_front_edge;
    }
    SpeedWall tfl_speed_wall;
    tfl_speed_wall.type = TFL_WALL;
    tfl_speed_wall.st_wall.clear();
    tfl_speed_wall.st_wall.emplace_back(math::Vec2d(0.0, tfl_block_s));
    tfl_speed_wall.st_wall.emplace_back(math::Vec2d(time_horizon_, tfl_block_s));
    tfl_speed_wall.stop_distance = safe_distance;
    speed_walls->emplace_back(tfl_speed_wall);
  }
}
/**
 * @brief 生成终点目标速度墙（固定位置约束）
 * @param[in] path_data 路径数据（包含剩余距离信息）
 * @param[out] speed_walls 速度墙输出容器
 * 
 * @par 输入/输出参数说明:
 * | 参数          | 类型                     | 取值范围        | 说明                     |
 * |---------------|--------------------------|----------------|--------------------------|
 * | path_data     | const PathData&          | -              | 包含剩余距离的路径数据    |
 * | speed_walls   | std::vector<SpeedWall>*   | -              | 速度墙输出容器           |
 *
 * @par 处理流程:
 * @startuml
 if (存在剩余距离?) then (是)
   :创建DESTINATION类型速度墙;
   :设置沿时间轴的垂直约束线;
   :添加到速度墙容器;
 endif
 @enduml
 *
 * @note 功能特性:
 * - 固定位置约束：在ST图中生成垂直线型速度墙
 * - 安全余量处理：使用kMathEpsilon避免负值
 * - 作用范围：覆盖整个规划时间域(time_horizon_)
 *
 * @warning 关键参数:
 * - 剩余距离来源：path_data.getRemainDisInfo()
 * - 最小有效距离：kMathEpsilon（避免数值问题）
 */
void SpeedWallProcessor::addDestinationSpeedWall(const PathData& path_data, std::vector<SpeedWall>* speed_walls) {
  if (path_data.getRemainDisInfo().first) {
    double stop_s = path_data.getRemainDisInfo().second;
    SpeedWall destination_speed_wall;
    destination_speed_wall.type = DESTINATION_WALL;
    destination_speed_wall.st_wall.clear();
    double destination_block_s = fmax(stop_s, kMathEpsilon);
    destination_speed_wall.st_wall.emplace_back(math::Vec2d(0.0, destination_block_s));
    destination_speed_wall.st_wall.emplace_back(math::Vec2d(time_horizon_, destination_block_s));
    destination_speed_wall.stop_distance = 0.0;
    speed_walls->emplace_back(destination_speed_wall);
  }
}
/**
 * @brief 生成终点目标速度墙（固定位置约束）
 * @param[in] path_data 路径数据（包含剩余距离信息）
 * @param[out] speed_walls 速度墙输出容器
 * 
 * @par 输入/输出参数说明:
 * | 参数          | 类型                     | 取值范围        | 说明                     |
 * |---------------|--------------------------|----------------|--------------------------|
 * | path_data     | const PathData&          | -              | 包含剩余距离的路径数据    |
 * | speed_walls   | std::vector<SpeedWall>*   | -              | 速度墙输出容器           |
 *
 * @par 处理流程:
 * @startuml
 if (存在剩余距离?) then (是)
   :创建DESTINATION类型速度墙;
   :设置沿时间轴的垂直约束线;
   :添加到速度墙容器;
 endif
 @enduml
 *
 * @note 功能特性:
 * - 固定位置约束：在ST图中生成垂直线型速度墙
 * - 安全余量处理：使用kMathEpsilon避免负值
 * - 作用范围：覆盖整个规划时间域(time_horizon_)
 *
 * @warning 关键参数:
 * - 剩余距离来源：path_data.getRemainDisInfo()
 * - 最小有效距离：kMathEpsilon（避免数值问题）
 */
void SpeedWallProcessor::addDecisionStopSpeedWall(const LocalView& local_view, const DecisionResult& decision_result,
                                                  ReferenceLineInfo* reference_line_info,
                                                  std::vector<SpeedWall>* speed_walls) {
  if (reference_line_info == nullptr || !reference_line_info->isValid()) {
    return;
  }
  // TODO
  auto& boundary_decision = decision_result.getLongitudinalBoundaryDecision();
  for (auto& boundary_constraint : boundary_decision) {
    if (boundary_constraint.type == WallType::LONG_JUNCTION_STOP
        || boundary_constraint.type == WallType::LONG_RSA_WALL) {
      auto ltm_point = local_view.getLocalizationPtr()->vehicleAlignPosePoint();
      auto ego_reference_point = reference_line_info->ref_line().getReferencePoint(ltm_point.x(), ltm_point.y());
      double ego_speed = local_view.getChassisPtr()->Speed();
      double decision_speed_limit = boundary_constraint.v;
      double stop_s = boundary_constraint.s - ego_reference_point.local_s();

      SpeedWall decision_speed_wall;
      decision_speed_wall.type = JUNCTION_STOP;
      decision_speed_wall.st_wall.clear();
      double destination_block_s = fmax(stop_s, kMathEpsilon);
      decision_speed_wall.st_wall.emplace_back(math::Vec2d(0.0, destination_block_s));
      decision_speed_wall.st_wall.emplace_back(math::Vec2d(time_horizon_, destination_block_s));
      decision_speed_wall.stop_distance = 0.0;
      speed_walls->emplace_back(decision_speed_wall);
    }
    if (boundary_constraint.type == WallType::LONG_VIRTUAL_STOP_WALL) {
      double stop_s = boundary_constraint.s;
      SpeedWall decision_speed_wall;
      decision_speed_wall.type = VIRTUAL_LINE;
      decision_speed_wall.st_wall.clear();
      double virtual_stop_s = fmax(stop_s, kMathEpsilon);
      decision_speed_wall.st_wall.emplace_back(math::Vec2d(0.0, virtual_stop_s));
      decision_speed_wall.st_wall.emplace_back(math::Vec2d(time_horizon_, virtual_stop_s));
      decision_speed_wall.stop_distance = 0.0;
      speed_walls->emplace_back(decision_speed_wall);
    }
  }
}
/**
 * @brief 生成自由空间速度墙（障碍物遮挡区域）
 * @param[in] path_data 路径数据（包含阻塞区域信息）
 * @param[in] behavior_state 行为状态
 * @param[out] speed_walls 速度墙输出容器
 * 
 * @par 输入/输出参数说明:
 * | 参数            | 类型                     | 取值范围        | 说明                     |
 * |-----------------|--------------------------|----------------|--------------------------|
 * | path_data       | const PathData&          | -              | 包含阻塞区域的路径数据    |
 * | behavior_state  | const BehaviorState&     | -              | 当前行为状态（含泊车状态）|
 * | speed_walls     | std::vector<SpeedWall>*   | -              | 速度墙输出容器           |
 *
 * @par 处理流程:
 * @startuml
 if (存在阻塞区域?) then (是)
   :查找最近阻塞点s坐标;
   if (泊车状态?) then (是)
     :设置安全距离0.5m;
   else (否)
     :设置默认安全距离5.0m;
   endif
   :创建FREESPACE类型速度墙;
   :设置沿时间轴的垂直约束线;
   :添加到速度墙容器;
 endif
 @enduml
 *
 * @note 功能特性:
 * - 障碍物遮挡处理：在ST图中生成最近障碍点的垂直约束
 * - 动态安全距离：泊车场景使用更小的安全余量
 * - 位置优化：确保最小有效距离(kMathEpsilon)
 *
 * @warning 关键参数:
 * - 默认安全距离：5.0m
 * - 泊车安全距离：0.5m
 * - 最近阻塞点选择：取最小s值的阻塞区域
 */
void SpeedWallProcessor::addFreespaceSpeedWall(const PathData& path_data, const BehaviorState& behavior_state,
                                               std::vector<SpeedWall>* speed_walls) {
  if (path_data.blockFSInfo().empty()) {
    return;
  }
  double closest_fs_s = kPostiveInfinity;
  for (const auto& block_fs : path_data.blockFSInfo()) {
    if (block_fs.s < closest_fs_s) {
      closest_fs_s = block_fs.s;
    }
  }

  double safe_distance = 1.0;
  if (behavior_state.park_in_state_) {
    safe_distance = 0.5;
  }

  for (int i = 0; i < speed_walls->size(); i++) {
    if (speed_walls->at(i).type == JUNCTION_STOP && abs(closest_fs_s - speed_walls->at(i).st_wall.front().y()) < 2.0) {
      return;
    }
  }

  SpeedWall freespace_speed_wall;
  freespace_speed_wall.type = FREESPACE_WALL;
  freespace_speed_wall.st_wall.clear();
  double freespace_block_s = fmax(closest_fs_s, kMathEpsilon);
  freespace_speed_wall.st_wall.emplace_back(math::Vec2d(0.0, freespace_block_s));
  freespace_speed_wall.st_wall.emplace_back(math::Vec2d(time_horizon_, freespace_block_s));
  freespace_speed_wall.stop_distance = safe_distance;
  speed_walls->emplace_back(freespace_speed_wall);
}

}  // namespace gpal::pnc::planning
