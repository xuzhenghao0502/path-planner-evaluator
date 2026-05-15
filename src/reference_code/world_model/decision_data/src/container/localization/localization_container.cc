/**
 * @file localization_container.cc
 * @brief 此文件实现了 `LocalizationContainer` 类，用于管理定位相关数据。
 * @details 该类继承自 `Container`，实现了插入、更新定位数据，清除数据，获取时间戳和感知障碍物指针等功能。
 */

#include "decision_data/container/localization/localization_container.h"

namespace gpal {
namespace pnc {
namespace prediction {

/**
 * @brief 向容器中插入数据消息。
 * @details 尝试将传入的 `std::any` 类型消息转换为 `planning::Localization`
 * 类型，若转换成功则调用 `Update` 方法更新数据。
 *
 * @param[in] message 待插入的 Protobuf 格式数据消息。
 * @return bool 插入成功返回 `true`，失败返回 `false`。
 *
 * @par 输入参数说明:
 * - message: 应包含有效的 `planning::Localization` 类型数据。
 *
 * @par 流程图:
 * @startuml
 * start
 * :尝试将 message 转换为 planning::Localization;
 * if (转换成功?) then (是)
 * :调用 Update 方法更新数据;
 * if (Update 成功?) then (是)
 * :返回 true;
 * else (否)
 * :返回 false;
 * endif
 * else (否)
 * :记录错误日志;
 * :返回 false;
 * endif
 * stop
 * @enduml
 */
bool LocalizationContainer::Insert(const std::any& message, const bool is_hpa_stage) {
  try {
    const auto& localization = std::any_cast<planning::Localization const&>(message);
    return Update(localization);
  } catch (const std::bad_any_cast& e) {
    ERT_PLOG_E << "Failed to cast data to Localization. Error: " << e.what();
  }

  return false;
}

bool LocalizationContainer::Insert(const double x, const double y, const double z, const double yaw, const double pitch,
                                   const double roll, const double timestamp) {
  try {
    return Update(x, y, z, yaw, pitch, roll, timestamp);
  } catch (const std::bad_any_cast& e) {
    ERT_PLOG_E << "Failed to cast data to Localization. Error: " << e.what();
  }

  return false;
}

/**
 * @brief 清除容器内的数据。
 */
void LocalizationContainer::Clear() {
  timestamp_ = -1.0;
  obstacle_ptr_ = nullptr;
}

/**
 * @brief 更新运动状态。
 * @details 根据传入的底盘信息更新容器内的运动状态。
 *
 * @param[in] chassis 底盘信息，包含车辆运动相关数据。
 * @return bool 更新成功返回 `true`，失败返回 `false`。
 *
 * @par 输入参数说明:
 * - chassis: 有效的 `planning::Chassis` 对象。
 *
 * @par 流程图:
 * @startuml
 * start
 * :根据 chassis 更新运动状态;
 * if (更新成功?) then (是)
 * :返回 true;
 * else (否)
 * :返回 false;
 * endif
 * stop
 * @enduml
 */
bool LocalizationContainer::Update(const planning::Localization& localization) {
  // if (!localization.isValid()) {
  //   ERT_PLOG_E << "Invalid localization message";
  //   return false;
  // }

  timestamp_ = localization.TimeStamp() * 1e-9;
  const auto& pose = localization.vehicleAlignPosePoint();

  if (obstacle_ptr_ == nullptr) {
    obstacle_ptr_ = std::make_unique<pnc::ObstacleBox>();
  }

  pnc::ObstacleBox ego_box;
  auto* config_manager = planning::Singleton<planning::ConfigManager>::get_instance();
  auto& vehicle_config = config_manager->vehicle_config();
  ERT_PLOG_D << "Vehicle config: " << vehicle_config.DebugString();
  ego_box.set_width(vehicle_config.vehicle_param().width());
  ego_box.set_length(vehicle_config.vehicle_param().length());
  ego_box.set_height(2.0f);

  ego_box.mutable_position()->set_x(pose.x());
  ego_box.mutable_position()->set_y(pose.y());
  ego_box.mutable_position()->set_z(pose.z());

  ego_box.mutable_angle()->set_x(pose.roll());
  ego_box.mutable_angle()->set_y(pose.pitch());
  ego_box.mutable_angle()->set_z(pose.yaw());

  *obstacle_ptr_ = std::move(ego_box);
  ERT_PLOG_D << fp4 << "[Ego Obstacle] Timestamp: " << localization.TimeStamp() << ", "
             << obstacle_ptr_->ShortDebugString();
  return true;
}

/**
 * @brief 更新运动状态。
 * @details 根据传入的位置、姿态和时间戳更新容器内的运动状态。
 *
 * @param[in] x 车辆的X坐标位置。
 * @param[in] y 车辆的Y坐标位置。
 * @param[in] z 车辆的Z坐标位置。
 * @param[in] yaw 车辆的航向角。
 * @param[in] pitch 车辆的俯仰角。
 * @param[in] roll 车辆的横滚角。
 * @param[in] timestamp 数据的时间戳。
 * @return bool 更新成功返回 `true`，失败返回 `false`。
 *
 * @par 输入参数说明:
 * - x, y, z: 表示车辆在空间中的位置坐标。
 * - yaw, pitch, roll: 表示车辆的姿态角。
 * - timestamp: 表示数据的时间戳，用于同步。
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查输入数据是否有效;
 * if (数据有效?) then (是)
 * :更新时间戳;
 * :获取车辆配置信息;
 * :创建或更新障碍物对象;
 * :设置障碍物属性和位置信息;
 * :返回 true;
 * else (否)
 * :记录错误日志;
 * :返回 false;
 * endif
 * stop
 * @enduml
 */
bool LocalizationContainer::Update(const double x, const double y, const double z, const double yaw, const double pitch,
                                   const double roll, const double timestamp) {
  constexpr double kEpsilon = 1e-6f;
  if ((std::fabs(x) < kEpsilon) && (std::fabs(y) < kEpsilon) && (std::fabs(z) < kEpsilon)
      && (std::fabs(yaw) < kEpsilon)) {
    ERT_PLOG_E << "Invalid localization message";
    return false;
  }

  timestamp_ = timestamp;

  if (obstacle_ptr_ == nullptr) {
    obstacle_ptr_ = std::make_unique<pnc::ObstacleBox>();
  }

  pnc::ObstacleBox ego_box;
  auto* config_manager = planning::Singleton<planning::ConfigManager>::get_instance();
  auto& vehicle_config = config_manager->vehicle_config();
  ERT_PLOG_D << "Vehicle config: " << vehicle_config.DebugString();
  ego_box.set_width(vehicle_config.vehicle_param().width());
  ego_box.set_length(vehicle_config.vehicle_param().length());
  ego_box.set_height(2.0f);

  ego_box.mutable_position()->set_x(x);
  ego_box.mutable_position()->set_y(y);
  ego_box.mutable_position()->set_z(z);

  ego_box.mutable_angle()->set_x(roll);
  ego_box.mutable_angle()->set_y(pitch);
  ego_box.mutable_angle()->set_z(yaw);

  *obstacle_ptr_ = std::move(ego_box);
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(64) << " LocalizationContainer vehicleAlignPosePoint: x: " << x << " y: " << y
      << " z: " << z << " yaw: " << yaw;
  ERT_LOG_D(oss.str());

  ERT_PLOG_D << fp4 << "[Ego Obstacle] Timestamp: " << timestamp << ", " << obstacle_ptr_->ShortDebugString();
  return true;
}

/**
 * @brief 更新位姿信息。
 * @details 根据传入的定位消息更新容器内的位姿信息。
 *
 * @param[in] localization 接收到的定位消息，包含车辆位姿相关数据。
 * @return bool 更新成功返回 `true`，失败返回 `false`。
 *
 * @par 输入参数说明:
 * - localization: 有效的 `planning::Localization` 对象。
 *
 * @par 流程图:
 * @startuml
 * start
 * :根据 localization 更新位姿信息;
 * if (更新成功?) then (是)
 * :返回 true;
 * else (否)
 * :返回 false;
 * endif
 * stop
 * @enduml
 */
bool LocalizationContainer::Update(const planning::Chassis& chassis) {
  if (obstacle_ptr_ == nullptr) {
    ERT_PLOG_E << "Not initialized ego obstacle by LocalizationI";
    return false;
  }

  const auto& speed = chassis.Speed();
  const auto& heading = obstacle_ptr_->angle().z();
  obstacle_ptr_->mutable_velocity()->set_x(speed * std::cos(heading));
  obstacle_ptr_->mutable_velocity()->set_y(speed * std::sin(heading));
  obstacle_ptr_->mutable_velocity()->set_z(0.0f);

  const auto& acc = std::hypot(chassis.LateralAcc(), chassis.LogituAcc());
  obstacle_ptr_->mutable_acceleration()->set_x(acc * std::cos(heading));
  obstacle_ptr_->mutable_acceleration()->set_y(acc * std::sin(heading));
  obstacle_ptr_->mutable_acceleration()->set_z(0.0f);
  obstacle_ptr_->mutable_angular_rate()->set_x(0.0f);
  obstacle_ptr_->mutable_angular_rate()->set_y(0.0f);
  obstacle_ptr_->mutable_angular_rate()->set_z(chassis.YawRate());

  ERT_PLOG_D << "[Ego Obstacle] " << obstacle_ptr_->ShortDebugString();
  return true;
}

/**
 * @brief 将位姿转换为感知障碍物。
 * @details 根据容器内的定位信息，将位姿转换为一个感知障碍物对象。
 *
 * @return const planning::Obstacle* 指向感知障碍物对象的指针，若未生成则返回
 * `nullptr`。
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查定位信息是否可用;
 * if (可用?) then (是)
 * :将位姿转换为感知障碍物;
 * :返回障碍物指针;
 * else (否)
 * :返回 nullptr;
 * endif
 * stop
 * @enduml
 */
bool LocalizationContainer::GetEgoObstacle(const double delay_time, planning::Obstacle& ego_obstacle) {
  if (obstacle_ptr_ == nullptr) {
    ERT_PLOG_W << "Ego obstacle is not initialized";
    return false;
  }

  auto* config_manager = planning::Singleton<planning::ConfigManager>::get_instance();
  auto prediction_config = config_manager->getConfig<PredictionConfig>("PredictionConfig");
  const auto ego_id = prediction_config.ego_vehicle_id();
  const std::string ego_id_str = std::to_string(ego_id);
  ego_obstacle.setPerceptionId(ego_id);
  ego_obstacle.setId(ego_id_str);
  ego_obstacle.setTimeDelay(delay_time);
  ego_obstacle.set_type(proto::PerceptionObstacle_ObstacleType_kTypeCar);
  ego_obstacle.set_motion_status(proto::PerceptionObstacle_ObstacleMotionStatus_kMotionDynamic);
  ego_obstacle.mutable_obstacle_boxes().push_back(std::move(*obstacle_ptr_));
  const auto& ego_boxes = ego_obstacle.mutable_obstacle_boxes();
  if (!ego_boxes.empty()) {
    ego_obstacle.setSpeed(std::hypot(ego_boxes.at(0).velocity().x(), ego_boxes.at(0).velocity().y()));
  }

  return true;
}

}  // namespace prediction
}  // namespace pnc
}  // namespace gpal