/**
 * @file localization_container.h
 * @brief 此文件定义了 `LocalizationContainer` 类，用于管理定位相关数据。
 * @details 该类继承自 `Container`，提供插入、更新定位数据，清除数据，获取时间戳和感知障碍物指针等功能。
 */

#pragma once

#include <any>
#include <memory>

#include "decision_data/container/container.h"
#include "local_view/chassis.h"
#include "local_view/localization.h"
#include "proto/obstacle_box.pb.h"

namespace gpal {
namespace pnc {
namespace prediction {

/**
 * @class LocalizationContainer
 * @brief 管理定位相关数据的容器类。
 * @details 继承自 `Container` 类，负责处理定位数据的插入、更新，清除数据，以及提供时间戳和感知障碍物指针等操作。
 */
class LocalizationContainer : public Container {
 public:
  /**
   * @brief 默认构造函数。
   */
  LocalizationContainer() = default;

  /**
   * @brief 虚析构函数，确保正确释放派生类对象。
   */
  virtual ~LocalizationContainer() = default;

  bool Insert(const std::any& message, const bool is_hpa_stage = false) override;
  bool Insert(const double x, const double y, const double z, const double yaw, const double pitch, const double roll,
              const double timestamp);

  bool Update(const planning::Chassis& chassis);

  void Clear() override;

  /**
   * @brief 获取当前时间戳。
   * @return double 当前时间戳，单位为秒。若未更新过则为 -1.0。
   */
  double timestamp() const { return timestamp_; }

  bool GetEgoObstacle(const double delay_time, planning::Obstacle& ego_obstacle);

 private:
  bool Update(const planning::Localization& localization);
  bool Update(const double x, const double y, const double z, const double yaw, const double pitch, const double roll,
              const double timestamp);

 private:
  double timestamp_ = -1.0;  ///< 当前时间戳，单位为秒，初始值为 -1.0 表示未更新过
  std::unique_ptr<pnc::ObstacleBox> obstacle_ptr_ = nullptr;  ///< 指向感知障碍物对象的智能指针，初始为 `nullptr`
};

}  // namespace prediction
}  // namespace pnc
}  // namespace gpal