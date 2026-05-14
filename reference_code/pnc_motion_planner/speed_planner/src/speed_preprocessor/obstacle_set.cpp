/**
 * @file obstacle_set.cpp
 * @brief 障碍物集合处理器
 * @details 本类负责多场景下的障碍物过滤与预处理
 */ 
#include "speed_preprocessor/obstacle_set.h"

namespace gpal::pnc::planning {

namespace {
std::mutex g_obstacle_access_mutex;
}  // namespace
/**
 * @brief 基于决策标签过滤障碍物
 * @param[in] local_view 局部视图数据（包含感知障碍物）
 * @param[in] decision_result 决策结果（包含OD标签）
 * @param[out] obstacle_set 输出障碍物集合
 * 
 * @par 输入参数说明:
 * | 参数                | 类型           | 取值范围        | 说明                     |
 * |---------------------|----------------|----------------|--------------------------|
 * | local_view          | LocalView      | -              | 包含感知障碍物集合       |
 * | decision_result      | DecisionResult | -              | 包含纵向决策标签数据     |
 * 
 * @par 处理流程:
 * @startuml
 :清空输出集合;
 if (决策标签无效?) then (yes)
   :输出错误日志;
   :return;
 endif
 partition 障碍物过滤 {
   :遍历所有感知障碍物;
   if (无对应决策标签?) then (yes)
     :跳过当前障碍物;
   else
     if (标签为INVALID/IGNORE?) then (yes)
       :跳过;
     else
       :创建SpeedPlannerObstacle;
       :加入输出集合;
     endif
   endif
 }
 @enduml
 *
 * @note 核心过滤规则:
 * 1. 仅保留LONGITUDINAL_OD_TAG有效且非忽略的障碍物
 * 2. 输出集合键值为障碍物ID
 *
 * @warning 需确保调用前已完成:
 * - 决策模块输出结果的有效性校验
 * - 感知障碍物数据的时间对齐
 */
void ObstacleSetProcessor::filterObstaclesByDecisionTag(const DecisionResult& decision_result,
                                                        ObstacleSet* obstacle_set) {
  obstacle_set->clear();
  auto od_tag_map = decision_result.getOdDecisions();
  if (od_tag_map == nullptr) {
    ERT_PLOG_I << "[OBSTACLE] decision obstacle invalid";
    return;
  }
  {
    std::lock_guard<std::mutex> lock(g_obstacle_access_mutex);
    try {
      for (const auto& [obs_id, decision_obj] : *od_tag_map) {
        if (decision_obj.long_od_tag == LongitudinalOdTag::INVALID
            || decision_obj.long_od_tag == LongitudinalOdTag::IGNORE) {
          continue;
        }

        auto od_decision = std::make_shared<Decision::DecisionObject>(decision_obj);
        auto speed_obstacle = std::make_shared<SpeedPlannerObstacle>(od_decision);
        obstacle_set->emplace(obs_id, speed_obstacle);
      }
    } catch (const std::exception& e) {
      ERT_PLOG_E << "Exception in filterObstaclesByDecisionTag: " << e.what();
    }
  }
}
/**
 * @brief 泊车场景障碍物过滤器（特殊场景适配）
 * @param[in] local_view 局部视图数据（包含感知障碍物）
 * @param[in] decision_result 决策结果（包含OD标签）
 * @param[out] obstacle_set 输出障碍物集合
 * 
 * @par 输入参数说明:
 * | 参数                | 类型           | 取值范围        | 说明                     |
 * |---------------------|----------------|----------------|--------------------------|
 * | local_view          | LocalView      | -              | 包含感知障碍物集合       |
 * | decision_result      | DecisionResult | -              | 决策模块输出结果         |
 * 
 * @par 处理流程:
 * @startuml
 :清空输出集合;
 if (决策标签无效?) then (yes)
   :输出错误日志;
   :return;
 endif
 partition 障碍物过滤 {
   :遍历所有感知障碍物;
   if (黑名单类型?) then (yes)
     :跳过;
   else
     if (无对应决策标签?) then (yes)
       :创建无决策障碍物对象;
       :加入输出集合;
     else
       :创建带决策障碍物对象;
       :设置LONGITUDINAL_OD_TAG为FOLLOW;
       :加入输出集合;
     endif
   endif
 }
 @enduml
 *
 * @note 特殊处理策略:
 * 1. 黑名单过滤规则见IsBlackListObject方法
 * 2. 无决策标签障碍物默认保留
 * 3. 强制设置所有障碍物的纵向决策标签为FOLLOW
 *
 * @warning 需确保调用前已完成:
 * - 泊车场景类型识别
 * - 感知障碍物坐标系对齐
 */
void ObstacleSetProcessor::filterInterestingObstaclesForParking(const DecisionResult& decision_result,
                                                                ObstacleSet* obstacle_set) {
  obstacle_set->clear();
  auto od_tag_map = decision_result.getOdDecisions();
  if (od_tag_map == nullptr) {
    ERT_PLOG_I << "[OBSTACLE] decision obstacle invalid";
    return;
  }
  try {
    for (const auto& [obs_id, decision_obj] : *od_tag_map) {
      if (IsBlackListObject(decision_obj.type)) {
        continue;
      }
      auto od_decision = std::make_shared<Decision::DecisionObject>(decision_obj);
      auto speed_obstacle = std::make_shared<SpeedPlannerObstacle>(od_decision);
      speed_obstacle->setLongitudinalOdTag(LongitudinalOdTag::FOLLOW);
      obstacle_set->emplace(obs_id, speed_obstacle);
    }
  } catch (const std::exception& e) {
    ERT_PLOG_E << "Exception in filterInterestingObstaclesForParking: " << e.what();
  }
}
/**
 * @brief ACC场景障碍物过滤器（跟车场景适配）
 * @param[in] local_view 局部视图数据（包含感知障碍物）
 * @param[in] decision_result 决策结果（包含CIPV候选集）
 * @param[out] obstacle_set 输出障碍物集合
 * 
 * @par 输入参数说明:
 * | 参数                | 类型           | 取值范围        | 说明                     |
 * |---------------------|----------------|----------------|--------------------------|
 * | local_view          | LocalView      | -              | 感知障碍物数据源         |
 * | decision_result      | DecisionResult | -              | 包含CIPV候选障碍物ID集合 |
 * 
 * @par 处理流程:
 * @startuml
 :清空输出集合;
 if (决策标签无效?) then (yes)
   :输出错误日志;
   :return;
 endif
 partition 障碍物过滤 {
   :遍历CIPV候选障碍物;
   if (障碍物存在且决策有效?) then (yes)
     :创建带决策障碍物对象;
     :设置LONGITUDINAL_OD_TAG为FOLLOW;
     :加入输出集合;
   endif
 }
 @enduml
 *
 * @note 核心处理策略:
 * 1. 仅处理CIPV候选集内的障碍物
 * 2. 强制设置纵向决策标签为FOLLOW
 * 3. 依赖CIPV模块输出候选障碍物ID
 *
 * @warning 需确保调用前已完成:
 * - CIPV模块候选障碍物计算
 * - 感知障碍物坐标系对齐
 */
void ObstacleSetProcessor::filterInterestingObstaclesForAcc(const DecisionResult& decision_result,
                                                            ObstacleSet* obstacle_set) {
  obstacle_set->clear();
  auto cipv_set = decision_result.getCipoCandidates();
  auto od_tag_map = decision_result.getOdDecisions();
  if (od_tag_map == nullptr) {
    ERT_PLOG_I << "[OBSTACLE] decision obstacle invalid";
    return;
  }
  try {
    for (const auto& [obs_id, decision_obj] : *od_tag_map) {
      auto od_decision = std::make_shared<Decision::DecisionObject>(decision_obj);
      auto speed_obstacle = std::make_shared<SpeedPlannerObstacle>(od_decision);
      speed_obstacle->setLongitudinalOdTag(LongitudinalOdTag::FOLLOW);
      obstacle_set->emplace(obs_id, speed_obstacle);
    }
  } catch (const std::exception& e) {
    ERT_PLOG_E << "Exception in filterInterestingObstaclesForAcc: " << e.what();
  }
}
/**
 * @brief 障碍物类型黑名单过滤器（静态规则引擎）
 * @param[in] obj_type 待判断的障碍物类型
 * @return bool 是否属于黑名单类型（true表示需要过滤）
 * 
 * @par 输入参数说明:
 * | 参数          | 类型                                   | 取值范围       | 说明                     |
 * |---------------|----------------------------------------|---------------|--------------------------|
 * | obj_type      | proto::PerceptionObstacle_ObstacleType | 枚举类型       | 障碍物类型枚举值         |
 *
 * @note 黑名单包含类型:
 * 1. 无效类型/未知类型
 * 2. 停车设施类（地锁/车轮挡）
 * 3. 施工设施类（洒水器/倒锥）
 * 4. 特殊行人类型（非站立状态）
 * 5. 特殊路权类型（不可行驶区域相关）
 *
 * @warning 需与感知模块类型定义保持同步更新
 */
bool ObstacleSetProcessor::IsBlackListObject(const Decision::ObjectType& obj_type) {
  switch (obj_type) {
    case Decision::ObjectType::UNKNOWN:
      return true;
    default:
      return false;
  }
  return true;
}

// void ObstacleSetProcessor::setObstacleDefaultPrediction(std::shared_ptr<SpeedPlannerObstacle> obstacle) {
//   auto raw_prediction_multi = obstacle->getMutableRawPredictionMultiModal();
//   SpeedPlannerObstacle::RawSinglePrediction prediction;
//   proto::TrajectoryPoint traj_point;
//   traj_point.mutable_path_point()->set_x(obstacle->perceptionBoundingBox().center_x());
//   traj_point.mutable_path_point()->set_y(obstacle->perceptionBoundingBox().center_y());
//   traj_point.mutable_path_point()->set_theta(obstacle->perceptionBoundingBox().heading());
//   traj_point.mutable_path_point()->set_s(0.0);
//   for (int i = 0; i <= 50; i++) {
//     traj_point.set_relative_time(0.1 * i);
//     prediction.traj.push_back(traj_point);
//   }
//   raw_prediction_multi->push_back(prediction);
// }

}  // namespace gpal::pnc::planning