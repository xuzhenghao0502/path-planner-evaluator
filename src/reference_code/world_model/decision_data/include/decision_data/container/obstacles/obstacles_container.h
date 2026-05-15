/**
 * @file obstacles_container.h
 * @brief 此文件定义了 `ObstaclesContainer` 类，用于管理障碍物信息，
 * @details `ObstaclesContainer` 类继承自 `Container`，借助 LRU 缓存机制管理障碍物，能够跟踪当前帧不同类型障碍物的 ID。
 */

#pragma once

#include <any>
#include <memory>
#include <string>
#include <vector>

#include "decision_data/common/utils/lru_cache.h"
#include "decision_data/container/container.h"
#include "decision_data/container/obstacles/obstacle.h"

namespace gpal {
namespace pnc {
namespace prediction {

/**
 * @class ObstaclesContainer
 * @brief 管理障碍物信息的容器类，继承自 `Container`。
 * @details 负责处理障碍物数据的插入、清除，管理不同类型障碍物的 ID，提供获取障碍物指针和深拷贝容器的功能。
 */
class ObstaclesContainer : public Container {
 public:
  /**
   * @brief 构造函数
   * @details 初始化 `ObstaclesContainer` 对象，进行必要的成员变量初始化。
   */
  ObstaclesContainer();

  /**
   * @brief 析构函数
   * @details 虚析构函数，确保正确释放派生类对象的资源，默认实现。
   */
  virtual ~ObstaclesContainer() = default;

  bool Insert(const std::any& message, const bool is_hpa_stage = false) override;

  void Clear() override;

  bool SetTimestamp(double timestamp, float delay_time);

  void InsertPlanningObstacle(const planning::Obstacle& planning_obstacle, const double timestamp,
                              const bool is_hpa_stage = false);

  Obstacle* GetObstacle(const int id);

  void SetConsideredObstacleIds();

  /**
   * @brief 获取容器中障碍物的数量
   * @details 返回当前容器中存储的障碍物的数量。
   *
   * @return size_t 障碍物的数量。
   *
   * @par 流程图:
   * @startuml
   * start
   * :获取缓存中障碍物的数量;
   * :返回该数量;
   * stop
   * @enduml
   */
  size_t size() { return ptr_obstacles_.size(); }

  /**
   * @brief 获取当前帧中所有障碍物的 ID
   * @details 返回当前帧中所有障碍物的 ID 列表的常量引用。
   *
   * @return const std::vector<int>& 当前帧中所有障碍物的 ID 列表。
   */
  const std::vector<int>& frame_vehicle_ids() const { return frame_vehicle_ids_; }

  const std::vector<int>& frame_obstacle_ids() const { return frame_obstacle_ids_; }

  /**
   * @brief 获取当前帧中静止物体的 ID
   * @details 返回当前帧中静止物体的 ID 列表的常量引用。
   *
   * @return const std::vector<int>& 当前帧中静止物体的 ID 列表。
   */
  const std::vector<int>& frame_static_obstacle_ids() const { return frame_static_obstacle_ids_; }

  /**
   * @brief 获取当前帧中可移动障碍物的 ID
   * @details 返回当前帧中可移动障碍物的 ID 列表的常量引用。
   *
   * @return const std::vector<int>& 当前帧中可移动障碍物的 ID 列表。
   */
  const std::vector<int>& frame_movable_obstacle_ids() const { return frame_movable_obstacle_ids_; }

  /**
   * @brief 获取当前帧中未被忽略级别的障碍物的 ID
   * @details 返回当前帧中未被标记为忽略级别的障碍物的 ID 列表的常量引用。
   *
   * @return const std::vector<int>& 当前帧中未被忽略级别的障碍物的 ID 列表。
   */
  const std::vector<int>& frame_considered_obstacle_ids() const { return frame_considered_obstacle_ids_; }

  /**
   * @brief 获取容器的时间戳
   * @details 返回当前容器记录的时间戳。
   *
   * @return double 容器的时间戳。
   */
  double timestamp() const { return timestamp_; }

  /**
   * @brief 深拷贝当前容器
   * @details 创建一个新的 `ObstaclesContainer` 对象，将当前容器的数据深拷贝到新对象中。
   *
   * @return std::shared_ptr<ObstaclesContainer> 深拷贝后的新容器的智能指针。
   *
   * @par 流程图:
   * @startuml
   * start
   * :创建新的 ObstaclesContainer 对象;
   * :复制时间戳;
   * :复制各类障碍物 ID 列表;
   * :遍历原容器的障碍物缓存;
   * :为每个障碍物创建新对象并复制数据;
   * :将新障碍物对象插入到新容器的缓存;
   * :返回新容器的智能指针;
   * stop
   * @enduml
   */
  std::shared_ptr<ObstaclesContainer> Clone() const {
    auto new_container = std::make_shared<ObstaclesContainer>();

    new_container->timestamp_ = this->timestamp_;
    new_container->frame_obstacle_ids_ = this->frame_obstacle_ids_;
    new_container->frame_movable_obstacle_ids_ = this->frame_movable_obstacle_ids_;
    new_container->frame_considered_obstacle_ids_ = this->frame_considered_obstacle_ids_;

    // 深拷贝 ptr_obstacles_
    auto& obstacles = this->ptr_obstacles_;
    const auto& map = obstacles.GetMap();
    for (const auto& id_obstacle_pair : map) {
      const auto& [id, obstacle_ptr] = id_obstacle_pair;
      auto new_obstacle = std::make_unique<Obstacle>(*(obstacle_ptr.val));
      new_container->ptr_obstacles_.Put(id, std::move(new_obstacle));
    }
    return new_container;
  }

 private:
  void SetConfig();

  void ClearFrameIds();

  Obstacle* GetObstacleWithLRUUpdate(const int obstacle_id);

  void UpdateDelayTime(float delay_time);

  bool FilterOd(const planning::Obstacle& planning_obstacle, const bool is_hpa_stage = false);

 private:
  PredictionConfig config_;  ///< 预测配置信息，包含障碍物预测相关的配置参数
  double timestamp_ = -1.0;  ///< 容器的时间戳，初始值为 -1.0，取值范围 `[-1.0, +∞)`
  common::LRUCache<int, std::unique_ptr<Obstacle>>
      ptr_obstacles_;  ///< 使用 LRU 缓存管理障碍物指针，键为障碍物 ID，值为障碍物指针

  std::vector<int> frame_vehicle_ids_;              ///< 当前帧中他车的 ID 列表
  std::vector<int> frame_obstacle_ids_;             ///< 当前帧中所有障碍物的 ID 列表
  std::vector<int> frame_static_obstacle_ids_;      ///< 当前帧中静止障碍物的 ID 列表
  std::vector<int> frame_movable_obstacle_ids_;     ///< 当前帧中可移动障碍物的 ID 列表
  std::vector<int> frame_considered_obstacle_ids_;  ///< 当前帧中未被忽略级别的障碍物的 ID 列表
};

}  // namespace prediction
}  // namespace pnc
}  // namespace gpal