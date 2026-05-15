/**
 * @file speed_preprocessor.cpp
 * @brief 多场景速度预处理中枢
 * @details 本类负责统筹速度规划前的多源数据处理,包括障碍物处理、速度限制集成、速度墙生成等。
 */
#include "speed_preprocessor/speed_preprocessor.h"

namespace gpal::pnc::planning {
  /**
 * @brief 速度预处理器初始化（配置参数加载）
 * 
 * @par 功能说明:
 * | 功能项                | 说明                          |
 * |-----------------------|------------------------------|
 * | 配置管理器初始化       | 获取单例配置管理器实例        |
 * | 速度规划参数加载       | 从配置系统加载速度规划参数      |
 * | 预处理器参数加载       | 加载速度预处理专用配置参数      |
 * 
 * @par 处理流程:
 * @startuml
 :获取配置管理器单例;
 :加载SpeedPlannerConfig配置;
 :加载SpeedPreProcessorConfig配置;
 :校验配置管理器有效性;
 @enduml
 *
 * @note 关键配置项:
 * - SpeedPlannerConfig：速度规划器全局参数
 * - SpeedPreProcessorConfig：速度预处理专用参数
 * 
 * @warning 调用约束:
 * - 必须在类实例化后首先调用
 * - 需要确保配置系统已正确初始化
 */
void SpeedPreprocessor::init() {
  config_manager_ = Singleton<ConfigManager>::get_instance();
  CHECK_NOTNULL(config_manager_);
  speed_planner_config_ = config_manager_->getConfig<SpeedPlannerConfig>("SpeedPlannerConfig");
  speed_preprocessor_config_ = config_manager_->getConfig<SpeedPreProcessorConfig>("SpeedPreProcessorConfig");
}

/**
 * @brief NOA场景速度预处理主流程
 * @param[in] local_view 局部环境信息（包含定位、底盘、障碍物等）
 * @param[in] reference_line_info 参考线信息（可为空）
 * @param[in] path_data 路径数据（包含离散化路径和局部路径）
 * @param[in] decision_result 决策结果
 * @param[in] init_speed_state 初始速度状态
 * @param[out] speed_result 速度规划结果（包含障碍物集/速度限制/速度墙等）
 * 
 * @par 输入/输出参数说明:
 * | 参数              | 类型                     | 取值范围        | 说明                     |
 * |-------------------|--------------------------|----------------|--------------------------|
 * | local_view        | const LocalView&         | -              | 包含车辆环境感知信息      |
 * | reference_line_info| ReferenceLineInfo*      | [nullptr, 有效]| 参考线信息（道路级数据）  |
 * | path_data         | const PathData&          | -              | 规划路径数据              |
 * | decision_result   | const DecisionResult&    | -              | 决策系统输出结果          |
 * | init_speed_state  | const SpeedState&        | -              | 初始速度状态              |
 * | speed_result      | shared_ptr<SpeedResult>  | -              | 速度规划结果容器          |
 *
 * @par 处理流程:
 * @startuml
 :初始化配置参数;
 partition 横向路径处理 {
   :生成离散化路径组;
   :生成局部路径组;
 }
 partition 障碍物处理 {
   :基于决策标签过滤障碍物;
 }
 partition 转弯状态计算 {
   :计算转弯标志位;
   :更新结果中的转弯状态;
 }
 partition 速度限制处理 {
   :获取路径速度限制;
 }
 partition 速度墙生成 {
   :清空现有速度墙;
   :添加交通灯速度墙;
   :添加终点速度墙;
   :添加决策停车速度墙;
   :添加自由空间速度墙;
 }
 @enduml
 *
 * @note 核心处理模块:
 * - 横向路径生成：为速度规划提供路径采样
 * - 障碍物过滤：基于决策标签筛选相关障碍物
 * - 转弯预判：通过参考线曲率变化预判转弯状态
 * - 速度限制集成：综合道路曲率/地图限速等因素
 * - 速度墙构建：生成各种类型的速度约束边界
 */
void SpeedPreprocessor::NoaProcess(const LocalView& local_view, ReferenceLineInfo* reference_line_info,
                                   const PathData& path_data, const DecisionResult& decision_result, const SpeedState& init_speed_state,
                                   std::shared_ptr<SpeedResult> speed_result) {
  init();
  // 1. lateral path
  lateral_path_.discretized_path_group_ = lateral_path_processor_.getDiscretizedPathGroup(path_data.discretized_path());
  lateral_path_.local_path_group_ = lateral_path_processor_.getLocalPathGroup(path_data.local_path(), 50.0, 1.0);
  // 2. obstacle
  // obstacle_set_processor_.preprocessObstacle(local_view, decision_result, speed_result);
  obstacle_set_processor_.filterObstaclesByDecisionTag(decision_result, speed_result->mutableObstacleSet());

  // tmp process turn flag
  caculateTurnFlag(local_view, reference_line_info);
  speed_result->setTurnFlag(last_turn_state_);

  // 3. speed_limit
  speed_limit_processor_.getSpeedLimit(local_view, decision_result, reference_line_info,
                                       lateral_path_.discretized_path_group_.origin_path_, last_turn_state_,
                                       speed_result->getBehaviorState(),init_speed_state, speed_result->mutableSpeedLimiteResult());
  // 4. speed_wall
  speed_result->mutable_speed_walls()->clear();
  if(!speed_wall_processor_.init(time_horizon_)){
    // ERT_PLOG_I<<"  speed_wall_processor init failed!";
  }
  speed_wall_processor_.addTflSpeedWall(local_view, reference_line_info, decision_result, is_turn_around_, speed_result->mutable_speed_walls());
  speed_wall_processor_.addDestinationSpeedWall(path_data, speed_result->mutable_speed_walls());
  speed_wall_processor_.addDecisionStopSpeedWall(local_view, decision_result, reference_line_info,
                                                 speed_result->mutable_speed_walls());
  speed_wall_processor_.addFreespaceSpeedWall(path_data, speed_result->getBehaviorState(),
                                              speed_result->mutable_speed_walls());
}
/**
 * @brief ACC场景速度预处理主流程（自适应巡航控制）
 * @param[in] local_view 局部环境信息（包含定位、感知等）
 * @param[in] path_data 路径数据（包含局部路径信息）
 * @param[in] decision_result 决策结果
 * @param[in] init_speed_state 初始速度状态
 * @param[out] speed_result 速度规划结果容器
 * 
 * @par 输入/输出参数说明:
 * | 参数              | 类型                     | 取值范围        | 说明                     |
 * |-------------------|--------------------------|----------------|--------------------------|
 * | local_view        | const LocalView&         | -              | 实时车辆环境感知数据      |
 * | path_data         | const PathData&          | -              | 局部路径数据              |
 * | decision_result   | const DecisionResult&    | -              | 决策系统输出结果          |
 * | init_speed_state  | const SpeedState&        | -              | 初始速度状态              |
 * | speed_result      | shared_ptr<SpeedResult>  | -              | 速度规划结果容器          |
 *
 * @par 处理流程:
 * @startuml
 :初始化配置参数;
 partition 横向路径处理 {
   :生成局部路径组(长度50m,间隔1m);
 }
 partition 障碍物处理 {
   :过滤ACC相关障碍物;
 }
 partition 速度限制处理 {
   :获取ACC场景速度限制;
 }
 @enduml
 *
 * @note 功能特性:
 * - 专注ACC场景：针对跟车场景优化处理
 * - 障碍物过滤策略：保留前向关键障碍物
 * - 速度限制来源：曲率限速/跟车限速
 *
 * @warning 实现差异:
 * - 相比NOA流程：不处理转弯标志和复杂速度墙
 * - 路径处理范围：仅处理局部路径（非全局路径）
 */
void SpeedPreprocessor::AccProcess(const LocalView& local_view, const PathData& path_data,
                                   const DecisionResult& decision_result,const SpeedState& init_speed_state, std::shared_ptr<SpeedResult> speed_result) {
  init();
  // 1. lateral path

  lateral_path_.local_path_group_ = lateral_path_processor_.getLocalPathGroup(path_data.local_path(), 50.0, 1.0);

  // 2. obstacle
  obstacle_set_processor_.filterInterestingObstaclesForAcc(decision_result, speed_result->mutableObstacleSet());

  // 3. speed_limit
  speed_limit_processor_.getSpeedLimit(local_view, decision_result, nullptr,
                                       lateral_path_.local_path_group_.origin_path_, last_turn_state_,
                                       speed_result->getBehaviorState(), init_speed_state, speed_result->mutableSpeedLimiteResult());

  // 4. speed_wall
}
/**
 * @brief 泊车场景速度预处理主流程
 * @param[in] local_view 局部环境信息（包含定位、感知等）
 * @param[in] path_data 路径数据（包含离散化路径）
 * @param[in] decision_result 决策结果
 * @param[in] init_speed_state 初始速度状态
 * @param[out] speed_result 速度规划结果容器
 * 
 * @par 输入/输出参数说明:
 * | 参数              | 类型                     | 取值范围        | 说明                     |
 * |-------------------|--------------------------|----------------|--------------------------|
 * | local_view        | const LocalView&         | -              | 车辆环境感知数据          |
 * | path_data         | const PathData&          | -              | 泊车路径数据              |
 * | decision_result   | const DecisionResult&    | -              | 泊车决策结果              |
 * | init_speed_state  | const SpeedState&        | -              | 初始速度状态              |
 * | speed_result      | shared_ptr<SpeedResult>  | -              | 速度规划结果容器          |
 *
 * @par 处理流程:
 * @startuml
 :初始化配置参数;
 partition 横向路径处理 {
   :生成离散化路径组(长度100m,间隔0.2m);
   :生成局部路径组(长度50m,间隔1m);
 }
 partition 障碍物处理 {
   :清空障碍物集合;
 }
 partition 速度限制处理 {
   :获取泊车场景速度限制;
 }
 partition 速度墙生成 {
   :清空现有速度墙;
   :添加终点速度墙;
 }
 @enduml
 *
 * @note 功能特性:
 * - 专注泊车场景：简化障碍物处理逻辑
 * - 路径采样密度：0.2m间隔（高于常规场景）
 * - 关键约束：终点位置速度墙
 *
 * @warning 实现差异:
 * - 障碍物处理：当前版本暂未启用泊车障碍物过滤
 * - 路径精度：离散化路径采样间隔更小（0.2m）
 */
void SpeedPreprocessor::ParkProcess(const LocalView& local_view, const PathData& path_data,
                                    const DecisionResult& decision_result, const SpeedState& init_speed_state, std::shared_ptr<SpeedResult> speed_result) {
  init();
  // 1. lateral path
  lateral_path_.discretized_path_group_ =
      lateral_path_processor_.getDiscretizedPathGroup(path_data.discretized_path(), 100, 0.2, 10.0);
  lateral_path_.local_path_group_ = lateral_path_processor_.getLocalPathGroup(path_data.local_path(), 50.0, 0.2, 10.0);

  // 2. obstacle
  speed_result->mutableObstacleSet()->clear(); // tmp not use obstacle set for parking
  if (speed_planner_config_.enable_parking_od_check()) {
    obstacle_set_processor_.filterInterestingObstaclesForParking(decision_result, speed_result->mutableObstacleSet());
  }
  // 3. speed_limit
  speed_limit_processor_.getSpeedLimit(local_view, decision_result, nullptr,
                                       lateral_path_.discretized_path_group_.origin_path_,last_turn_state_,
                                       speed_result->getBehaviorState(),init_speed_state, speed_result->mutableSpeedLimiteResult());
  // 4. speed_wall
  speed_result->mutable_speed_walls()->clear();
  if(!speed_wall_processor_.init(time_horizon_)){
    // ERT_PLOG_I<<"  speed_wall_processor init failed!";
  }
  speed_wall_processor_.addDestinationSpeedWall(path_data, speed_result->mutable_speed_walls());
}

/**
 * @brief 计算转弯状态标志（基于参考线航向变化）
 * @param[in] local_view 局部环境信息（包含定位数据）
 * @param[in] reference_line_info 参考线信息（可为空）
 * 
 * @par 输入/输出参数说明:
 * | 参数                | 类型                  | 取值范围        | 说明                     |
 * |---------------------|-----------------------|----------------|--------------------------|
 * | local_view          | const LocalView&      | -              | 包含车辆定位信息         |
 * | reference_line_info | ReferenceLineInfo*    | [nullptr, 有效]| 道路参考线信息           |
 * | 输出                | 成员变量              | [0,2]          | 转弯状态(last_turn_state_)|
 *
 * @par 处理流程:
 * @startuml
 start
 :检查参考线有效性;
 if (无效?) then (是)
   stop
 else (否)
   :获取车辆当前位置;
   :确定参考线搜索范围(后15m + 前70m);
   :遍历参考点计算航向变化;
   :记录最小cos(Δheading)及其索引;
   :根据当前状态机判断转弯类型;
   if (cosΔθ < -0.95) then (急转弯)
     :标记调头状态;
   elseif (cosΔθ < 0.34) then (普通转弯)
     :更新转弯状态;
   else (直行)
     :保持当前状态;
   endif
 stop
 @enduml
 *
 * @note 状态机说明:
 * - 0: 直行状态
 * - 1: 普通转弯
 * - 2: 调头转弯
 * 
 * @warning 关键阈值:
 * - 后视距离: 15m
 * - 前视距离: 70m
 * - 调头阈值: cosΔθ < -0.95 → 约168度
 * - 转弯阈值: cosΔθ < 0.34 → 约70度
 * - 出弯阈值: cosΔθ > 0.98 → 约11.5度
 */
void SpeedPreprocessor::caculateTurnFlag(const LocalView& local_view, ReferenceLineInfo* reference_line_info) {
  if (reference_line_info == nullptr || !reference_line_info->isValid()) {
    return;
  }
  const float backward_thr = 15.0;
  const float forward_thr = 70.0;
  auto ltm_point = local_view.getLocalizationPtr()->vehicleAlignPosePoint();
  auto ego_reference_point = reference_line_info->ref_line().getReferencePoint(ltm_point.x(), ltm_point.y());
  auto start_index =
      reference_line_info->ref_line().getNearestReferenceIndex(ego_reference_point.local_s() - backward_thr);
  auto end_index =
      reference_line_info->ref_line().getNearestReferenceIndex(ego_reference_point.local_s() + forward_thr);
  auto reference_points = reference_line_info->ref_line().reference_points();
  if (start_index >= end_index) {
    return;
  }
  int min_end_index = start_index;
  float start_heading = reference_points.at(start_index).heading();
  float min_cosheading = 1.f;
  for (int i = start_index + 1; i < end_index; ++i) {
    float cos_t = cosf(reference_points.at(i).heading() - start_heading);
    if (cos_t < min_cosheading) {
      min_cosheading = cos_t;
      min_end_index = i;
    }
  }

  if (last_turn_state_ == 0) {
    // 入弯
    if (min_cosheading < -0.95f) {
      is_turn_around_ = true;
      last_turn_state_ = 2;
    } else if (min_cosheading < 0.34f) {
      last_turn_state_ = 1;
    } else {
    }
  } else if (last_turn_state_ == 1) {
    if (min_cosheading < -0.95f) {  // normal turn to turn around
      is_turn_around_ = true;
      last_turn_state_ = 2;
    }
    if (min_cosheading > 0.98f) {
      last_turn_state_ = 0;
      is_turn_around_ = false;
    }
  } else {
    // 出弯
    if (min_cosheading > 0.98f) {
      last_turn_state_ = 0;
      is_turn_around_ = false;
    }
  }
}

}  // namespace gpal::pnc::planning
