/**
 * @file obstacles_container.cc
 * @brief 此文件实现了 `ObstaclesContainer` 类，用于管理障碍物信息。
 * @details `ObstaclesContainer` 类继承自 `Container`，借助 LRU
 * 缓存机制管理障碍物，实现了障碍物数据的插入、清除，管理不同类型障碍物的 ID 等功能。
 */

#include "decision_data/container/obstacles/obstacles_container.h"

#include "decision_data/container/obstacles/obstacle.h"

namespace gpal {
namespace pnc {
namespace prediction {

ObstaclesContainer::ObstaclesContainer() {
  SetConfig();

  ptr_obstacles_.ChangeCapacity(config_.container_config().obstacle_container_config().max_obstacle_num());
}

/**
 * @brief 向容器中插入数据消息
 * @details 尝试将传入的 Protobuf 数据消息插入到容器中。
 *
 * @param[in] message 待插入的 Protobuf 数据消息，使用 `std::any`
 * 类型兼容多种消息类型。
 * @par 输入参数说明:
 * - message: 必须是有效的 Protobuf 消息对象，包含障碍物相关信息。
 *
 * @return bool 插入成功返回 `true`，失败返回 `false`。
 *
 * @par 流程图:
 * @startuml
 * start
 * :接收消息;
 * if (消息有效?) then (是)
 * :解析消息并插入障碍物数据;
 * :返回 true;
 * else (否)
 * :返回 false;
 * endif
 * stop
 * @enduml
 */
bool ObstaclesContainer::Insert(const std::any &message, const bool is_hpa_stage) {
  ClearFrameIds();

  if (timestamp_ < 0.0) {
    ERT_PLOG_E << "Called before timestamp is set.";
    return false;
  }

  try {
    const auto &frame_obstacles = std::any_cast<planning::IndexedObstacles const &>(message);

    auto *ego_obstacle = GetObstacle(config_.ego_vehicle_id());
    if (ego_obstacle == nullptr) {
      ERT_PLOG_W << "Ego obstacle is not inserted yet, skip perception insertion.";
      return false;
    }

    const auto &valid_distance_to_ego = config_.container_config().obstacle_container_config().valid_distance_to_ego();
    double ego_x = ego_obstacle->latest_feature().position().x();
    double ego_y = ego_obstacle->latest_feature().position().y();
    for (const auto &planning_obstacle : frame_obstacles.items()) {
      const auto &delay_time = planning_obstacle->timeDelay();
      const auto &obstacle_boxes = planning_obstacle->obstacle_boxes();
      const auto &obstacle_position = obstacle_boxes.at(0).position();
      double distance = std::hypot(obstacle_position.x() - ego_x, obstacle_position.y() - ego_y);
      if (distance > valid_distance_to_ego) {
        ERT_PLOG_W << fp2 << "Skip obstacle [" << planning_obstacle->id()
                   << "] insertion, distance to ego is too large: " << distance << "m.";
        continue;
      }

      InsertPlanningObstacle(*planning_obstacle, timestamp_, is_hpa_stage);
    }
    return true;
  } catch (const std::bad_any_cast &e) {
    ERT_PLOG_E << "Failed to cast message to IndexedObstacles. Error: " << e.what();
  }

  return false;
}

/**
 * @brief 清空容器中的数据
 * @details 清除容器中存储的所有障碍物信息和相关 ID 列表，重置时间戳。
 *
 * @par 流程图:
 * @startuml
 * start
 * :清空障碍物指针缓存;
 * :清空各类障碍物 ID 列表;
 * :重置时间戳;
 * stop
 * @enduml
 */
void ObstaclesContainer::Clear() {
  timestamp_ = -1.0;

  ptr_obstacles_.Clear();

  ClearFrameIds();
}

/**
 * @brief 设置容器的时间戳和延迟时间
 * @details 更新容器的时间戳，并根据延迟时间进行相应处理。
 *
 * @param[in] timestamp 要设置的时间戳。
 * @par 输入参数说明:
 * - timestamp: 双精度浮点数，取值范围 `[0, +∞)`。
 *
 * @param[in] delay_time 延迟时间。
 * @par 输入参数说明:
 * - delay_time: 单精度浮点数，取值范围 `[0, +∞)`。
 *
 * @return bool 设置成功返回 `true`，失败返回 `false`。
 *
 * @par 流程图:
 * @startuml
 * start
 * :接收时间戳和延迟时间;
 * :更新容器时间戳;
 * :根据延迟时间处理相关逻辑;
 * if (处理成功?) then (是)
 * :返回 true;
 * else (否)
 * :返回 false;
 * endif
 * stop
 * @enduml
 */
bool ObstaclesContainer::SetTimestamp(double timestamp, float delay_time) {
  double duration = std::fabs(timestamp - timestamp_);
  if (duration < 1e-6) {
    ERT_PLOG_W << fp2 << "Got duplicated timestamp(last [" << timestamp_ << "] current [" << timestamp << "]).";

    UpdateDelayTime(delay_time);
    return false;
  }
  ERT_LOG_I("ObstaclesContainer SetTimestamp: ", timestamp_, " timestamp: ", timestamp,
            " delay_time: ", delay_time);
  if (duration > config_.container_config().obstacle_container_config().perception_message_outdated_duration()) {
    ERT_PLOG_W << fp2 << "Got outdated timestamp(last [" << timestamp_ << "] current [" << timestamp
               << "]), clear the container.";
    Clear();
  }
  timestamp_ = timestamp;
  ERT_LOG_I("ObstaclesContainer SetTimestamp: ", timestamp_);

  return true;
}

/**
 * @brief 插入规划障碍物信息
 * @details 根据传入的规划障碍物信息和时间戳，将障碍物信息插入到容器中。
 *
 * @param[in] planning_obstacle 规划障碍物信息对象。
 * @par 输入参数说明:
 * - planning_obstacle: 有效的 `planning::Obstacle`
 * 对象，包含规划障碍物的详细信息。
 *
 * @param[in] timestamp 对应的时间戳。
 * @par 输入参数说明:
 * - timestamp: 双精度浮点数，取值范围 `[0, +∞)`。
 *
 * @par 流程图:
 * @startuml
 * start
 * :接收规划障碍物信息和时间戳;
 * :创建新的障碍物对象;
 * :将规划障碍物信息填充到新对象;
 * :将新对象插入到缓存;
 * stop
 * @enduml
 */
void ObstaclesContainer::InsertPlanningObstacle(const planning::Obstacle &planning_obstacle, const double timestamp,
                                                const bool is_hpa_stage) {
  if (timestamp < 0.0) {
    ERT_PLOG_E << "Invalid timestamp for obstacle insertion [" << timestamp << "].";
    return;
  }

  const auto &obstacle_boxes = planning_obstacle.obstacle_boxes();
  if (obstacle_boxes.empty()) {
    ERT_PLOG_E << "Obstacle [" << planning_obstacle.id() << "] has no sub-obstacles!";
    return;
  }

  int id = planning_obstacle.perception_id();
  if (auto *obstacle_ptr = GetObstacleWithLRUUpdate(id); obstacle_ptr != nullptr) {
    obstacle_ptr->Insert(planning_obstacle, timestamp);
  } else if (auto obstacle_ptr = Obstacle::Create(planning_obstacle, timestamp); obstacle_ptr != nullptr) {
    ptr_obstacles_.Put(id, std::move(obstacle_ptr));
  } else {
    ERT_PLOG_E << "Failed to insert obstacle [" << id << "]";
    return;
  }

  if (id == config_.ego_vehicle_id()) {
    return;
  }

  frame_obstacle_ids_.push_back(id);
  if (FilterOd(planning_obstacle, is_hpa_stage)) {
    frame_static_obstacle_ids_.push_back(id);
  }
  if ((Obstacle::IsStaticObstacle(planning_obstacle.type()))
      || (planning_obstacle.motion_status() != proto::PerceptionObstacle_ObstacleMotionStatus_kMotionDynamic)) {
    if (std::find(frame_static_obstacle_ids_.begin(), frame_static_obstacle_ids_.end(), id)
        == frame_static_obstacle_ids_.end()) {
      frame_static_obstacle_ids_.push_back(id);
    }
  } else if ((Obstacle::IsVehicle(planning_obstacle.type()))
             && (planning_obstacle.motion_status() == proto::PerceptionObstacle_ObstacleMotionStatus_kMotionDynamic)) {
    frame_vehicle_ids_.push_back(id);
    frame_movable_obstacle_ids_.push_back(id);
  } else if ((Obstacle::IsVru(planning_obstacle.type()))
             && (planning_obstacle.motion_status() == proto::PerceptionObstacle_ObstacleMotionStatus_kMotionDynamic)) {
    frame_movable_obstacle_ids_.push_back(id);
  }
}

bool ObstaclesContainer::FilterOd(const planning::Obstacle &planning_obstacle, const bool is_hpa_stage) {
  auto *config_manager = planning::Singleton<planning::ConfigManager>::get_instance();
  const auto &prediction_config = config_manager->getConfig<PredictionConfig>("PredictionConfig");
  if ((is_hpa_stage) && (prediction_config.enable_hpa_static_od_filt())) {
    double vel_threashlod = prediction_config.vel_threashlod_od();  // 5km/h
    auto vel_x = planning_obstacle.obstacle_boxes().front().velocity().x();
    auto vel_y = planning_obstacle.obstacle_boxes().front().velocity().y();
    if (!Obstacle::IsVehicle(planning_obstacle.type())) {
      vel_threashlod = prediction_config.vel_threashlod_vru();  // 2km/h
    }
    return ((std::fabs(vel_x) < vel_threashlod) and (std::fabs(vel_y) < vel_threashlod));
  } else {
    return false;
  }
}

/**
 * @brief 获取指定 ID 的障碍物指针
 * @details 根据传入的障碍物 ID，从容器中查找并返回对应的障碍物指针。
 *
 * @param[in] id 障碍物的 ID。
 * @par 输入参数说明:
 * - id: 整数，取值范围 `[0, +∞)`。
 *
 * @return Obstacle* 障碍物指针，若未找到则返回 `nullptr`。
 *
 * @par 流程图:
 * @startuml
 * start
 * :接收障碍物 ID;
 * :在缓存中查找对应 ID 的障碍物;
 * if (找到障碍物?) then (是)
 * :返回障碍物指针;
 * else (否)
 * :返回 nullptr;
 * endif
 * stop
 * @enduml
 */
Obstacle *ObstaclesContainer::GetObstacle(const int id) {
  if (auto *ptr_obstacle = ptr_obstacles_.GetSilently(id); ptr_obstacle != nullptr) {
    return ptr_obstacle->get();
  }

  return nullptr;
}

/**
 * @brief 设置当前帧中经过优先级分配后的有效障碍物 ID
 * @details 根据一定规则筛选并设置当前帧中被认为有效的障碍物 ID 列表。
 *
 * @par 流程图:
 * @startuml
 * start
 * :遍历所有障碍物;
 * :根据优先级规则判断障碍物是否有效;
 * :将有效障碍物的 ID 添加到列表;
 * stop
 * @enduml
 */
void ObstaclesContainer::SetConsideredObstacleIds() {
  frame_considered_obstacle_ids_.clear();
  for (const int id : frame_movable_obstacle_ids_) {
    if (auto *ptr_obstacle = GetObstacle(id); ((ptr_obstacle != nullptr) && (!ptr_obstacle->IsIgnore()))) {
      frame_considered_obstacle_ids_.push_back(id);
    }
  }

  ERT_PLOG_I << "Obstacles total[" << frame_obstacle_ids_.size() << "], movable[" << frame_movable_obstacle_ids_.size()
             << "], static[" << frame_static_obstacle_ids_.size() << "], considered["
             << frame_considered_obstacle_ids_.size() << "], vehicle[" << frame_vehicle_ids_.size() << "].";
}

/**
 * @brief 设置预测配置
 * @details 根据一定规则设置容器的预测配置信息。
 *
 * @par 流程图:
 * @startuml
 * start
 * :获取配置信息;
 * :设置容器的预测配置;
 * stop
 * @enduml
 */
void ObstaclesContainer::SetConfig() {
  auto *config_manager = planning::Singleton<planning::ConfigManager>::get_instance();

  config_ = config_manager->getConfig<PredictionConfig>("PredictionConfig");
}

/**
 * @brief 清空当前帧的障碍物 ID 列表
 * @details 清除当前帧中记录的各类障碍物 ID 列表。
 *
 * @par 流程图:
 * @startuml
 * start
 * :清空所有障碍物 ID 列表;
 * stop
 * @enduml
 */
void ObstaclesContainer::ClearFrameIds() {
  frame_vehicle_ids_.clear();
  frame_obstacle_ids_.clear();
  frame_static_obstacle_ids_.clear();
  frame_movable_obstacle_ids_.clear();
  frame_considered_obstacle_ids_.clear();
}

/**
 * @brief 获取指定 ID 的障碍物指针，并更新 LRU 缓存
 * @details 根据传入的障碍物 ID，从容器中查找对应的障碍物指针，同时更新 LRU
 * 缓存中该障碍物的访问记录。
 *
 * @param[in] obstacle_id 障碍物的 ID。
 * @par 输入参数说明:
 * - obstacle_id: 整数，取值范围 `[0, +∞)`。
 *
 * @return Obstacle* 障碍物指针，若未找到则返回 `nullptr`。
 *
 * @par 流程图:
 * @startuml
 * start
 * :接收障碍物 ID;
 * :在缓存中查找对应 ID 的障碍物;
 * if (找到障碍物?) then (是)
 * :更新 LRU 缓存访问记录;
 * :返回障碍物指针;
 * else (否)
 * :返回 nullptr;
 * endif
 * stop
 * @enduml
 */
Obstacle *ObstaclesContainer::GetObstacleWithLRUUpdate(const int obstacle_id) {
  if (auto *ptr_obstacle = ptr_obstacles_.Get(obstacle_id); ptr_obstacle != nullptr) {
    return ptr_obstacle->get();
  }
  return nullptr;
}

/**
 * @brief 更新延迟时间
 * @details 根据传入的延迟时间，更新容器内与延迟相关的状态。
 *
 * @param[in] delay_time 延迟时间。
 * @par 输入参数说明:
 * - delay_time: 单精度浮点数，取值范围 `[0, +∞)`。
 *
 * @par 流程图:
 * @startuml
 * start
 * :接收延迟时间;
 * :更新容器内延迟相关状态;
 * stop
 * @enduml
 */
void ObstaclesContainer::UpdateDelayTime(float delay_time) {
  for (const int id : frame_obstacle_ids_) {
    if (auto *ptr_obstacle = GetObstacle(id); ptr_obstacle != nullptr) {
      ERT_PLOG_D << fp4 << "Obstacle [" << id << "] delay time [" << ptr_obstacle->delay_time() << "] update to ["
                 << delay_time << "]";
      ptr_obstacle->SetDelayTime(delay_time);
    }
  }
}

}  // namespace prediction
}  // namespace pnc
}  // namespace gpal