/**
 * @file obstacle.h
 * @brief 此文件定义了 `Obstacle` 类，用于表示和管理障碍物信息。
 * @details 该类包含障碍物的基本属性、状态判断方法、特征管理方法等，可用于处理感知到的障碍物数据。
 */

#pragma once

#include <deque>

#include "base/log.h"
#include "decision_data/common/prediction_map/prediction_map.h"
#include "obstacle/obstacle.h"
#include "proto/prediction/feature.pb.h"
#include "proto/prediction/prediction_config.pb.h"

namespace gpal {
namespace pnc {
namespace prediction {

/**
 * @class Obstacle
 * @brief 表示和管理障碍物信息的类。
 * @details 该类封装了障碍物的各种属性，如 ID、类型、时间戳等，同时提供了判断障碍物状态、管理历史特征等功能。
 */
class Obstacle {
 public:
  static std::unique_ptr<Obstacle> Create(const planning::Obstacle& planning_obstacle, double timestamp);

  /**
   * @brief 默认构造函数。
   */
  Obstacle() = default;

  /**
   * @brief 虚析构函数，确保正确释放派生类对象。
   */
  virtual ~Obstacle() = default;

  bool Insert(const planning::Obstacle& planning_obstacle, double timestamp);

  bool IsVehicle() const;

  static bool IsVehicle(const proto::PerceptionObstacle::ObstacleType& type);

  bool IsPedestrian() const;

  static bool IsPedestrian(const proto::PerceptionObstacle::ObstacleType& type);

  bool IsNonMotorizedVehicle() const;

  static bool IsNonMotorizedVehicle(const proto::PerceptionObstacle::ObstacleType& type);

  bool IsStaticObstacle() const;

  static bool IsStaticObstacle(const proto::PerceptionObstacle::ObstacleType& type);

  bool IsPredictable() const;

  bool IsOnLane() const;

  static bool IsOnLane(const Feature& feature);

  bool IsInJunction() const;

  bool IsDowngraded() const;

  bool IsFallback() const;

  /**
   * @brief 获取障碍物的 ID。
   * @return int 障碍物的 ID。
   */
  int id() const { return id_; }

  /**
   * @brief 获取感知障碍物的类型。
   * @return proto::PerceptionObstacle::ObstacleType 感知障碍物的类型。
   */
  proto::PerceptionObstacle::ObstacleType type() const { return type_; }

  /**
   * @brief 获取障碍物的时间戳。
   * @return double 障碍物的时间戳。
   */
  double timestamp() const { return latest_feature().timestamp(); }

  /**
   * @brief 获取障碍物的延迟时间。
   * @return float 障碍物的延迟时间。
   */
  float delay_time() const { return latest_feature().delay_time(); }

  /**
   * @brief 获取障碍物的速度比例。
   * @return float 障碍物的速度比例。
   */
  float speed_ratio() const { return speed_ratio_; }

  /**
   * @brief 从最新到最早获取第 i 个特征。
   * @details 根据索引从历史特征记录中获取对应的特征。
   *
   * @param[in] i 特征的索引。
   * @return const Feature& 第 i 个特征的常量引用。
   *
   * @par 输入参数说明:
   * - i: 无符号整数，取值范围 `[0, history_size())`，表示特征的索引。
   *
   * @par 流程图:
   * @startuml
   * start
   * :检查历史特征记录是否为空;
   * if (不为空?) then (是)
   * :检查索引是否合法;
   * if (合法?) then (是)
   * :返回第 i 个特征;
   * else (否)
   * :触发断言错误;
   * endif
   * else (否)
   * :触发断言错误;
   * endif
   * stop
   * @enduml
   */
  const Feature& feature(size_t i) const {
    GCHECK(!feature_history_.empty());
    return feature_history_[i];
  }

  /**
   * @brief 从最新到最早获取指向第 i 个特征的指针。
   * @details 根据索引从历史特征记录中获取对应的特征指针。
   *
   * @param[in] i 特征的索引。
   * @return Feature* 指向第 i 个特征的指针。
   *
   * @par 输入参数说明:
   * - i: 无符号整数，取值范围 `[0, history_size())`，表示特征的索引。
   *
   * @par 流程图:
   * @startuml
   * start
   * :调用 feature 方法获取特征引用;
   * :返回特征引用的地址;
   * stop
   * @enduml
   */
  Feature* mutable_feature(size_t i) { return &const_cast<Feature&>(feature(i)); }

  /**
   * @brief 获取最新的特征。
   * @return const Feature& 最新特征的常量引用。
   *
   * @par 流程图:
   * @startuml
   * start
   * :检查历史特征记录是否为空;
   * if (不为空?) then (是)
   * :返回历史特征记录的第一个元素;
   * else (否)
   * :触发断言错误;
   * endif
   * stop
   * @enduml
   */
  const Feature& latest_feature() const {
    GCHECK(!feature_history_.empty());
    return feature_history_.front();
  }

  /**
   * @brief 获取指向最新特征的指针。
   * @return Feature* 指向最新特征的指针。
   *
   * @par 流程图:
   * @startuml
   * start
   * :调用 latest_feature 方法获取特征引用;
   * :返回特征引用的地址;
   * stop
   * @enduml
   */
  Feature* mutable_latest_feature() { return &const_cast<Feature&>(latest_feature()); }

  /**
   * @brief 获取最早的特征。
   * @return const Feature& 最早特征的常量引用。
   *
   * @par 流程图:
   * @startuml
   * start
   * :检查历史特征记录是否为空;
   * if (不为空?) then (是)
   * :返回历史特征记录的最后一个元素;
   * else (否)
   * :触发断言错误;
   * endif
   * stop
   * @enduml
   */
  const Feature& earliest_feature() const {
    GCHECK(!feature_history_.empty());
    return feature_history_.back();
  }

  /**
   * @brief 获取历史特征的数量。
   * @return size_t 历史特征的数量。
   */
  size_t history_size() const { return feature_history_.size(); }

  /**
   * @brief 获取障碍物的优先级。
   * @return ObstaclePriority 障碍物的优先级。
   */
  ObstaclePriority priority() const { return latest_feature().priority(); }

  bool IsStill() const;

  float GetStillSpeedThreshold() const;

  bool IsVru() const;

  static bool IsVru(const proto::PerceptionObstacle::ObstacleType& type);

  bool IsHeavyVehicle() const;

  bool IsCaution() const;

  bool IsNormal() const;

  bool IsIgnore() const;

  bool IsRetrogradeDriving() const;

  void SetEvaluatorType(ObstacleConfig::EvaluatorType type);

  void SetPredictorType(ObstacleConfig::PredictorType type);

  void SetDowngrade();

  void SetFallback();

  void SetDelayTime(float delay_time);

 private:
  void SetConfig();

  bool ReceivedOlderMessage(double timestamp) const;

  void SetId(const planning::Obstacle& planning_obstacle, Feature* feature);

  void SetType(const planning::Obstacle& planning_obstacle, Feature* feature);

  void SetTimestamp(double timestamp, double delay_time, Feature* feature);

  void SetBoxInfo(const pnc::ObstacleBox& obstacle_box, Feature* feature);

  void SetState(const pnc::ObstacleBox& obstacle_box, Feature* feature);

  void SetPosition(const pnc::ObstacleBox& obstacle_box, Feature* feature);

  void SetVelocity(const pnc::ObstacleBox& obstacle_box, Feature* feature);

  void SetSpeedRatio(const Feature& feature);

  void SetAcceleration(const pnc::ObstacleBox& obstacle_box, Feature* feature);

  void SetAngle(const pnc::ObstacleBox& obstacle_box, Feature* feature);

  void SetAngleRate(const pnc::ObstacleBox& obstacle_box, Feature* feature);

  void SetMotionStatus(const planning::Obstacle& planning_obstacle, Feature* feature);

  void InsertFeatureToHistory(const Feature& feature);

  void DiscardOutdatedHistory();

 private:
  PredictionConfig config_;  ///< 预测配置信息，包含障碍物预测相关的配置参数
  int id_ = -2;              ///< 障碍物的 ID，初始值为 -2
  float speed_ratio_ = 1.0;  ///< 障碍物的速度比例，初始值为 1.0
  proto::PerceptionObstacle::ObstacleType type_ =
      proto::PerceptionObstacle::kTypeUnknown;  ///< 障碍物的类型，初始为未知类型
  std::deque<Feature> feature_history_;         ///< 障碍物的历史特征记录，按时间顺序存储特征信息
};

}  // namespace prediction
}  // namespace pnc
}  // namespace gpal