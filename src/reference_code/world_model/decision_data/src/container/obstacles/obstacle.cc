/**
 * @file obstacle.cc
 * @brief 此文件实现了 `Obstacle` 类，用于表示和管理障碍物信息。
 * @details 该类包含障碍物的基本属性、状态判断方法、特征管理方法等的具体实现，可用于处理感知到的障碍物数据。
 */

#include "decision_data/container/obstacles/obstacle.h"

#include <math.h>

#include "base/log.h"
#include "base/singleton.h"
#include "config_manager/config_manager.h"
#include "math/math_utils.h"

namespace gpal {
namespace pnc {
namespace prediction {

static constexpr float kMS_KMH = 3.6f;
static constexpr float kLowSpeedLimitInTown = 30.0f / kMS_KMH;
static constexpr float kHighSpeedLimitInTown = 60.0f / kMS_KMH;
static constexpr float kSpeedLimitInHighway = 120.0f / kMS_KMH;

static const std::unordered_set<proto::PerceptionObstacle::ObstacleMotionStatus> kStillMotionStatus = {
    proto::PerceptionObstacle::kMotionStopped, proto::PerceptionObstacle::kMotionStatic};

static const std::unordered_set<proto::PerceptionObstacle::ObstacleType> kVehicleTypes = {
    proto::PerceptionObstacle::kTypeCar,       proto::PerceptionObstacle::kTypeBus,
    proto::PerceptionObstacle::kTypeTruck,     proto::PerceptionObstacle::kTypeHeavyEquipement,
    proto::PerceptionObstacle::kTypeTractor,   proto::PerceptionObstacle::kTypeTrailer,
    proto::PerceptionObstacle::kTypeSmallTruck};

static const std::unordered_set<proto::PerceptionObstacle::ObstacleType> kStaticObstacleTypes = {
    proto::PerceptionObstacle::kTypeBicycle,
    proto::PerceptionObstacle::kTypeTricycle,
    proto::PerceptionObstacle::kTypeInvalid,
    proto::PerceptionObstacle::kTypeCones,
    proto::PerceptionObstacle::kTypeParkingLockUp,
    proto::PerceptionObstacle::kTypeParkingLockDown,
    proto::PerceptionObstacle::kTypeWheelStopper,
    proto::PerceptionObstacle::kTypeWarningTriangle,
    proto::PerceptionObstacle::kTypeSprinkler,
    proto::PerceptionObstacle::kTypeFallenCone,
    proto::PerceptionObstacle::kTypePedestrianNonStanding,
    proto::PerceptionObstacle::kTypeUnderDrivable,
    proto::PerceptionObstacle::kTypeOverDrivable,
    proto::PerceptionObstacle::kTypeImpassible,
    proto::PerceptionObstacle::kTypeUnknown};

static const std::unordered_set<proto::PerceptionObstacle::ObstacleType> kVruTypes = {
    proto::PerceptionObstacle::kTypeTricycle, proto::PerceptionObstacle::kTypePedestrian,
    proto::PerceptionObstacle::kTypeRider};

static const std::unordered_set<proto::PerceptionObstacle::ObstacleType> kHeavyVehicleTypes = {
    proto::PerceptionObstacle::kTypeBus, proto::PerceptionObstacle::kTypeTruck,
    proto::PerceptionObstacle::kTypeHeavyEquipement, proto::PerceptionObstacle::kTypeTractor,
    proto::PerceptionObstacle::kTypeTrailer, proto::PerceptionObstacle::kTypeCar};

bool IsReverseAngle(double angle) {
  return std::fabs(angle) > M_PI_2;
}

/**
 * @brief 创建 `Obstacle` 对象的静态工厂方法。
 * @details 根据感知到的障碍物信息和时间戳创建一个新的 `Obstacle` 对象。
 *
 * @param[in] planning_obstacle 感知到的障碍物信息。
 * @param[in] timestamp 感知到障碍物的时间戳。
 * @return std::unique_ptr<Obstacle> 指向新创建的 `Obstacle`
 * 对象的智能指针，若创建失败返回 `nullptr`。
 *
 * @par 输入参数说明:
 * - planning_obstacle: 有效的 `planning::Obstacle`
 * 对象，包含感知到的障碍物信息。
 * - timestamp: 双精度浮点数，取值范围 `[0, +∞)`，表示感知到障碍物的时间戳。
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查输入参数有效性;
 * if (参数有效?) then (是)
 * :创建 Obstacle 对象;
 * :初始化对象属性;
 * :返回指向对象的智能指针;
 * else (否)
 * :返回 nullptr;
 * endif
 * stop
 * @enduml
 */
std::unique_ptr<Obstacle> Obstacle::Create(const planning::Obstacle& planning_obstacle, double timestamp) {
  std::unique_ptr<Obstacle> ptr_obstacle(new Obstacle());

  ptr_obstacle->SetConfig();
  if (!ptr_obstacle->Insert(planning_obstacle, timestamp)) {
    return nullptr;
  }
  return ptr_obstacle;
}

/**
 * @brief 插入一个感知到的障碍物及其时间戳。
 * @details
 * 将新感知到的障碍物信息和时间戳插入到障碍物对象中，更新其属性和历史特征。
 *
 * @param[in] planning_obstacle 感知到的障碍物信息。
 * @param[in] timestamp 感知到障碍物的时间戳。
 * @return bool 插入成功返回 `true`，失败返回 `false`。
 *
 * @par 输入参数说明:
 * - planning_obstacle: 有效的 `planning::Obstacle`
 * 对象，包含感知到的障碍物信息。
 * - timestamp: 双精度浮点数，取值范围 `[0, +∞)`，表示感知到障碍物的时间戳。
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查输入参数有效性;
 * if (参数有效?) then (是)
 * :更新障碍物属性;
 * :插入新特征到历史记录;
 * :丢弃过时的历史记录;
 * :返回 true;
 * else (否)
 * :返回 false;
 * endif
 * stop
 * @enduml
 */
bool Obstacle::Insert(const planning::Obstacle& planning_obstacle, double timestamp) {
  auto t0 = std::chrono::steady_clock::now();

  if ((planning_obstacle.id() == "") || (planning_obstacle.perception_id() < 0)) {
    ERT_PLOG_E << "Planning obstacle does not have id or perception id!";
    return false;
  }

  if (ReceivedOlderMessage(timestamp)) {
    ERT_PLOG_W << fp2 << "Obstacle [" << id_ << "] received an older frame [" << timestamp
               << "] than the previous one [" << this->timestamp() << "].";
    return false;
  }
  auto t1 = std::chrono::steady_clock::now();

  // note Set ID, Type, and Status etc of the feature and insert to
  // feature history.
  Feature feature;
  SetId(planning_obstacle, &feature);
  SetType(planning_obstacle, &feature);
  SetTimestamp(timestamp, planning_obstacle.timeDelay(), &feature);
  auto t2 = std::chrono::steady_clock::now();

  if (planning_obstacle.obstacle_boxes().empty()) {
    ERT_PLOG_E << "Planning obstacle [" << planning_obstacle.id() << "] does not have obstacle boxes!";
    return false;
  }
  const auto& obstacle_box = planning_obstacle.obstacle_boxes().at(0);
  SetBoxInfo(obstacle_box, &feature);
  auto t3 = std::chrono::steady_clock::now();

  SetState(obstacle_box, &feature);
  auto t4 = std::chrono::steady_clock::now();

  SetMotionStatus(planning_obstacle, &feature);
  auto t5 = std::chrono::steady_clock::now();

  // ERT_PLOG_D << "Obstacle [" << id_ << "](delay time[" << feature.delay_time() << "], position["
  //            << feature.position().x() << ", " << feature.position().y() << "] heading[" << feature.angle().z() << ", "
  //            << feature.velocity_heading() << "], velocity[" << feature.velocity().x() << ", " << feature.velocity().y()
  //            << ", " << feature.speed() << "], yaw_rate[" << feature.angular_rate().z() << "], is_still["
  //            << feature.is_still() << "]) has " << feature_history_.size() << " features.";

  InsertFeatureToHistory(feature);
  auto t6 = std::chrono::steady_clock::now();

  DiscardOutdatedHistory();
  auto t7 = std::chrono::steady_clock::now();

  // ERT_PLOG_D << fp4 << "Obstacle [" << id_ << "] ["
  //            << std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - t0).count()
  //            << "ms], ReceivedOlderMessage[" << std::chrono::duration<float, std::milli>(t1 - t0).count() << "ms], "
  //            << "SetIdTypeTimestamp[" << std::chrono::duration<float, std::milli>(t2 - t1).count() << "ms], SetBoxInfo["
  //            << std::chrono::duration<float, std::milli>(t3 - t2).count() << "ms], "
  //            << "SetState[" << std::chrono::duration<float, std::milli>(t4 - t3).count() << "ms], "
  //            << "SetMotionStatus[" << std::chrono::duration<float, std::milli>(t5 - t4).count() << "ms], "
  //            << "InsertFeatureToHistory[" << std::chrono::duration<float, std::milli>(t6 - t5).count() << "ms], "
  //            << "DiscardOutdatedHistory[" << std::chrono::duration<float, std::milli>(t7 - t6).count() << "ms].";
  return true;
}

/**
 * @brief 判断障碍物是否可移动。
 * @details 根据障碍物的类型判断其是否可移动。
 *
 * @return bool 可移动返回 `true`，否则返回 `false`。
 *
 * @par 流程图:
 * @startuml
 * start
 * :获取障碍物类型;
 * :调用 IsVehicle 方法判断;
 * :返回判断结果;
 * stop
 * @enduml
 */
bool Obstacle::IsVehicle() const {
  return IsVehicle(type_);
}

/**
 * @brief 静态方法，根据障碍物类型判断是否可移动。
 * @details 依据传入的障碍物类型判断该类型的障碍物是否可移动。
 *
 * @param[in] type 障碍物的类型。
 * @return bool 可移动返回 `true`，否则返回 `false`。
 *
 * @par 输入参数说明:
 * - type: 有效的 `proto::PerceptionObstacle::ObstacleType`
 * 枚举值，表示障碍物的类型。
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查障碍物类型;
 * :根据类型判断是否可移动;
 * :返回判断结果;
 * stop
 * @enduml
 */
bool Obstacle::IsVehicle(const proto::PerceptionObstacle::ObstacleType& type) {
  return (kVehicleTypes.find(type) != kVehicleTypes.end());
}

/**
 * @brief 判断障碍物是否为行人。
 * @details 根据障碍物的类型判断其是否为行人。
 *
 * @return bool 是行人返回 `true`，否则返回 `false`。
 *
 * @par 流程图:
 * @startuml
 * start
 * :获取障碍物类型;
 * :调用 IsPedestrian 方法判断;
 * :返回判断结果;
 * stop
 * @enduml
 */
bool Obstacle::IsPedestrian() const {
  return IsPedestrian(type_);
}

/**
 * @brief 静态方法，根据障碍物类型判断是否为行人。
 * @details 依据传入的障碍物类型判断该类型的障碍物是否为行人。
 *
 * @param[in] type 障碍物的类型。
 * @return bool 是行人返回 `true`，否则返回 `false`。
 *
 * @par 输入参数说明:
 * - type: 有效的 `proto::PerceptionObstacle::ObstacleType`
 * 枚举值，表示障碍物的类型。
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查障碍物类型;
 * :根据类型判断是否为行人;
 * :返回判断结果;
 * stop
 * @enduml
 */
bool Obstacle::IsPedestrian(const proto::PerceptionObstacle::ObstacleType& type) {
  return (type == proto::PerceptionObstacle::kTypePedestrian);
}

/**
 * @brief 判断障碍物是否为非机动车。
 * @details 根据障碍物的类型判断其是否为非机动车。
 *
 * @return bool 是非机动车返回 `true`，否则返回 `false`。
 *
 * @par 流程图:
 * @startuml
 * start
 * :获取障碍物类型;
 * :调用 IsNonMotorizedVehicle 方法判断;
 * :返回判断结果;
 * stop
 * @enduml
 */
bool Obstacle::IsNonMotorizedVehicle() const {
  return IsNonMotorizedVehicle(type_);
}

/**
 * @brief 静态方法，根据障碍物类型判断是否为非机动车。
 * @details 依据传入的障碍物类型判断该类型的障碍物是否为非机动车。
 *
 * @param[in] type 障碍物的类型。
 * @return bool 是非机动车返回 `true`，否则返回 `false`。
 *
 * @par 输入参数说明:
 * - type: 有效的 `proto::PerceptionObstacle::ObstacleType`
 * 枚举值，表示障碍物的类型。
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查障碍物类型;
 * :根据类型判断是否为非机动车;
 * :返回判断结果;
 * stop
 * @enduml
 */
bool Obstacle::IsNonMotorizedVehicle(const proto::PerceptionObstacle::ObstacleType& type) {
  return ((type == proto::PerceptionObstacle::kTypeBicycle) || (type == proto::PerceptionObstacle::kTypeTricycle));
}

/**
 * @brief 判断障碍物是否为不可移动类型障碍物。
 * @details 根据障碍物的类型判断其是否为不可移动类型障碍物。
 *
 * @return bool 是非机动车返回 `true`，否则返回 `false`。
 *
 * @par 流程图:
 * @startuml
 * start
 * :获取障碍物类型;
 * :调用 IsStaticObstacle 方法判断;
 * :返回判断结果;
 * stop
 * @enduml
 */
bool Obstacle::IsStaticObstacle() const {
  return IsStaticObstacle(type_);
}

/**
 * @brief 静态方法，根据障碍物的类型判断其是否为不可移动类型障碍物。
 * @details 依据传入的障碍物类型判断该类型的障碍物是否不可移动。
 *
 * @param[in] type 障碍物的类型。
 * @return bool 是不可移动障碍物返回 `true`，否则返回 `false`。
 *
 * @par 输入参数说明:
 * - type: 有效的 `proto::PerceptionObstacle::ObstacleType`
 * 枚举值，表示障碍物的类型。
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查障碍物类型;
 * :根据类型判断是否为不可移动障碍物;
 * :返回判断结果;
 * stop
 * @enduml
 */
bool Obstacle::IsStaticObstacle(const proto::PerceptionObstacle::ObstacleType& type) {
  return (kStaticObstacleTypes.find(type) != kStaticObstacleTypes.end());
}

/**
 * @brief 判断障碍物是否可预测。
 * @details 根据障碍物的属性和状态判断其是否可预测。
 *
 * @return bool 可预测返回 `true`，否则返回 `false`。
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查障碍物属性和状态;
 * :根据条件判断是否可预测;
 * :返回判断结果;
 * stop
 * @enduml
 */
bool Obstacle::IsPredictable() const {
  return ((!IsStaticObstacle()) && (!IsStill()));
}

/**
 * @brief 判断障碍物是否为 vulnerable road user (VRU)。
 * @details 根据障碍物的类型判断其是否为 VRU。
 *
 * @return bool 是 VRU 返回 `true`，否则返回 `false`。
 *
 * @par 流程图:
 * @startuml
 * start
 * :获取障碍物类型;
 * :根据类型判断是否为 VRU;
 * :返回判断结果;
 * stop
 * @enduml
 */
bool Obstacle::IsVru() const {
  return IsVru(type_);
}

bool Obstacle::IsVru(const proto::PerceptionObstacle::ObstacleType& type) {
  return (kVruTypes.count(type) > 0);
}

/**
 * @brief 判断障碍物是否为重型车辆。
 * @details 根据障碍物的类型判断其是否为重型车辆。
 *
 * @return bool 是重型车辆返回 `true`，否则返回 `false`。
 *
 * @par 流程图:
 * @startuml
 * start
 * :获取障碍物类型;
 * :根据类型判断是否为重型车辆;
 * :返回判断结果;
 * stop
 * @enduml
 */
bool Obstacle::IsHeavyVehicle() const {
  return (kHeavyVehicleTypes.count(type_) > 0);
}

/**
 * @brief 判断障碍物是否在车道上。
 * @details 根据障碍物的特征判断其是否在车道上。
 *
 * @return bool 在车道上返回 `true`，否则返回 `false`。
 *
 * @par 流程图:
 * @startuml
 * start
 * :获取障碍物特征;
 * :根据特征判断是否在车道上;
 * :返回判断结果;
 * stop
 * @enduml
 */
bool Obstacle::IsOnLane() const {
  const auto& feature = latest_feature();
  return IsOnLane(feature);
}

/**
 * @brief 静态方法，根据障碍物特征判断是否在车道上。
 * @details 依据传入的障碍物特征判断该障碍物是否在车道上。
 *
 * @param[in] feature 障碍物的特征信息。
 * @return bool 在车道上返回 `true`，否则返回 `false`。
 *
 * @par 输入参数说明:
 * - feature: 有效的 `Feature` 对象，包含障碍物的特征信息。
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查特征信息;
 * :根据特征判断是否在车道上;
 * :返回判断结果;
 * stop
 * @enduml
 */
bool Obstacle::IsOnLane(const Feature& feature) {
  return (feature.has_lane_feature_set()) && (feature.lane_feature_set().has_current_lane_feature());
}

/**
 * @brief 判断障碍物是否在路口。
 * @details 根据障碍物的属性判断其是否在路口。
 *
 * @return bool 在路口返回 `true`，否则返回 `false`。
 *
 * @par 流程图:
 * @startuml
 * start
 * :获取障碍物属性;
 * :根据属性判断是否在路口;
 * :返回判断结果;
 * stop
 * @enduml
 */
bool Obstacle::IsInJunction() const {
  const auto& feature = latest_feature();
  return (feature.has_junction_feature()) && (feature.junction_feature().exits_size() > 0);
}

/**
 * @brief 判断障碍物是否降级。
 * @details 根据障碍物的状态判断其是否降级。
 *
 * @return bool 降级返回 `true`，否则返回 `false`。
 *
 * @par 流程图:
 * @startuml
 * start
 * :获取障碍物状态;
 * :根据状态判断是否降级;
 * :返回判断结果;
 * stop
 * @enduml
 */
bool Obstacle::IsDowngraded() const {
  return latest_feature().is_downgraded();
}

/**
 * @brief 判断障碍物是否为备用状态。
 * @details 根据障碍物的状态判断其是否为备用状态。
 *
 * @return bool 是备用状态返回 `true`，否则返回 `false`。
 *
 * @par 流程图:
 * @startuml
 * start
 * :获取障碍物状态;
 * :根据状态判断是否为备用状态;
 * :返回判断结果;
 * stop
 * @enduml
 */
bool Obstacle::IsFallback() const {
  return latest_feature().is_fallback();
}

/**
 * @brief 设置配置信息。
 * @details 根据配置文件设置障碍物的相关配置信息。
 */
void Obstacle::SetConfig() {
  auto* config_manager = planning::Singleton<planning::ConfigManager>::get_instance();

  config_ = config_manager->getConfig<PredictionConfig>("PredictionConfig");
}

/**
 * @brief 判断是否接收到更旧的消息。
 * @details 根据传入的时间戳判断是否接收到比当前记录更旧的消息。
 *
 * @param[in] timestamp 待比较的时间戳。
 * @return bool 接收到更旧的消息返回 `true`，否则返回 `false`。
 *
 * @par 输入参数说明:
 * - timestamp: 双精度浮点数，取值范围 `[0, +∞)`，表示待比较的时间戳。
 *
 * @par 流程图:
 * @startuml
 * start
 * :获取当前记录的时间戳;
 * :与传入的时间戳比较;
 * :返回比较结果;
 * stop
 * @enduml
 */
bool Obstacle::ReceivedOlderMessage(double timestamp) const {
  if (feature_history_.empty()) {
    return false;
  }

  auto last_timestamp_received = this->timestamp();
  return timestamp - last_timestamp_received < 1e-6;
}

/**
 * @brief 判断障碍物是否静止。
 * @details 根据障碍物的速度判断其是否静止。
 *
 * @return bool 静止返回 `true`，否则返回 `false`。
 *
 * @par 流程图:
 * @startuml
 * start
 * :获取障碍物速度;
 * :与静止速度阈值比较;
 * :返回比较结果;
 * stop
 * @enduml
 */
bool Obstacle::IsStill() const {
  return IsStaticObstacle() ? true : latest_feature().is_still();
}

/**
 * @brief 获取静止速度阈值。
 * @return float 静止速度阈值。
 */
float Obstacle::GetStillSpeedThreshold() const {
  float still_speed_threshold = config_.container_config().obstacle_container_config().still_vehicle_speed_threshold();

  if (type_ == proto::PerceptionObstacle::kTypePedestrian) {
    still_speed_threshold = config_.container_config().obstacle_container_config().still_pedestrian_speed_threshold();
  }
  return still_speed_threshold;
}

/**
 * @brief 判断障碍物是否为需要谨慎处理的状态。
 * @return bool 是需要谨慎处理的状态返回 `true`，否则返回 `false`。
 */
bool Obstacle::IsCaution() const {
  return (latest_feature().priority() == ObstaclePriority::CAUTION);
}

/**
 * @brief 判断障碍物是否为正常状态。
 * @return bool 是正常状态返回 `true`，否则返回 `false`。
 */
bool Obstacle::IsNormal() const {
  return (latest_feature().priority() == ObstaclePriority::NORMAL);
}

/**
 * @brief 判断障碍物是否为忽略状态。
 * @return bool 是忽略状态返回 `true`，否则返回 `false`。
 */
bool Obstacle::IsIgnore() const {
  return (latest_feature().priority() == ObstaclePriority::IGNORE);
}

/**
 * @brief 判断障碍物是否逆行。
 * @details 根据障碍物的运动状态判断其是否逆行。
 *
 * @return bool 逆行返回 `true`，否则返回 `false`。
 *
 * @par 流程图:
 * @startuml
 * start
 * :获取障碍物运动状态;
 * :根据状态判断是否逆行;
 * :返回判断结果;
 * stop
 * @enduml
 */
bool Obstacle::IsRetrogradeDriving() const {
  return false;
}

/**
 * @brief 设置障碍物的评估器类型。
 * @details 根据传入的评估器类型设置障碍物的评估器类型。
 *
 * @param[in] type 障碍物的评估器类型。
 *
 * @par 输入参数说明:
 * - type: 有效的 `ObstacleConfig::EvaluatorType`
 * 枚举值，表示障碍物的评估器类型。
 */
void Obstacle::SetEvaluatorType(ObstacleConfig::EvaluatorType type) {
  auto feature = mutable_latest_feature();
  feature->set_evaluator_type(type);
}

/**
 * @brief 设置障碍物的预测器类型。
 * @details 根据传入的预测器类型设置障碍物的预测器类型。
 *
 * @param[in] type 障碍物的预测器类型。
 *
 * @par 输入参数说明:
 * - type: 有效的 `ObstacleConfig::PredictorType`
 * 枚举值，表示障碍物的预测器类型。
 */
void Obstacle::SetPredictorType(ObstacleConfig::PredictorType type) {
  auto feature = mutable_latest_feature();
  feature->set_predictor_type(type);
}

/**
 * @brief 设置障碍物为降级状态。
 */
void Obstacle::SetDowngrade() {
  mutable_latest_feature()->set_is_downgraded(true);
}

/**
 * @brief 设置障碍物为备用状态。
 */
void Obstacle::SetFallback() {
  mutable_latest_feature()->set_is_fallback(true);
}

/**
 * @brief 设置障碍物的延迟时间。
 * @details 根据传入的延迟时间设置障碍物的延迟时间。
 *
 * @param[in] delay_time 障碍物的延迟时间。
 *
 * @par 输入参数说明:
 * - delay_time: 浮点数，取值范围 `[0, +∞)`，表示障碍物的延迟时间。
 */
void Obstacle::SetDelayTime(float delay_time) {
  mutable_latest_feature()->set_delay_time(delay_time);
}

/**
 * @brief 根据感知障碍物信息设置障碍物的 ID。
 * @details 根据传入的感知障碍物信息和特征指针设置障碍物的 ID。
 *
 * @param[in] planning_obstacle 感知到的障碍物信息。
 * @param[out] feature 指向障碍物特征的指针，用于存储 ID 信息。
 *
 * @par 输入参数说明:
 * - planning_obstacle: 有效的 `proto::PerceptionObstacle`
 * 对象，包含感知到的障碍物信息。
 * - feature: 有效的 `Feature` 对象指针，用于存储 ID 信息。
 */
void Obstacle::SetId(const planning::Obstacle& planning_obstacle, Feature* feature) {
  id_ = planning_obstacle.perception_id();
  feature->set_id(id_);
}

/**
 * @brief 根据感知障碍物信息设置障碍物的类型。
 * @details 根据传入的感知障碍物信息和特征指针设置障碍物的类型。
 *
 * @param[in] planning_obstacle 感知到的障碍物信息。
 * @param[out] feature 指向障碍物特征的指针，用于存储类型信息。
 *
 * @par 输入参数说明:
 * - planning_obstacle: 有效的 `proto::PerceptionObstacle`
 * 对象，包含感知到的障碍物信息。
 * - feature: 有效的 `Feature` 对象指针，用于存储类型信息。
 */
void Obstacle::SetType(const planning::Obstacle& planning_obstacle, Feature* feature) {
  type_ = planning_obstacle.type();
  feature->set_type(type_);
}

/**
 * @brief 根据时间戳和延迟时间设置障碍物的时间戳信息。
 * @details 根据传入的时间戳、延迟时间和特征指针设置障碍物的时间戳信息。
 *
 * @param[in] timestamp 感知到障碍物的时间戳。
 * @param[in] delay_time 障碍物的延迟时间。
 * @param[out] feature 指向障碍物特征的指针，用于存储时间戳信息。
 *
 * @par 输入参数说明:
 * - timestamp: 双精度浮点数，取值范围 `[0, +∞)`，表示感知到障碍物的时间戳。
 * - delay_time: 浮点数，取值范围 `[0, +∞)`，表示障碍物的延迟时间。
 * - feature: 有效的 `Feature` 对象指针，用于存储时间戳信息。
 */
void Obstacle::SetTimestamp(double timestamp, double delay_time, Feature* feature) {
  feature->set_timestamp(timestamp);
  feature->set_delay_time(-delay_time); //plan delay_time为负数
}

/**
 * @brief 根据子障碍物信息设置障碍物的边界框信息。
 * @details 根据传入的子障碍物信息和特征指针设置障碍物的边界框信息。
 *
 * @param[in] sub_obstacle 子障碍物信息。
 * @param[out] feature 指向障碍物特征的指针，用于存储边界框信息。
 *
 * @par 输入参数说明:
 * - sub_obstacle: 有效的 `proto::SubObstacle` 对象，包含子障碍物信息。
 * - feature: 有效的 `Feature` 对象指针，用于存储边界框信息。
 */
void Obstacle::SetBoxInfo(const pnc::ObstacleBox& obstacle_box, Feature* feature) {
  float length = -1.0;
  if (obstacle_box.has_length()) {
    length = obstacle_box.length();
  }

  float width = -1.0;
  if (obstacle_box.has_width()) {
    width = obstacle_box.width();
  }

  float height = -1.0;
  if (obstacle_box.has_height()) {
    height = obstacle_box.height();
  }

  feature->set_length(length);
  feature->set_width(width);
  feature->set_height(height);

  // box points
  float half_diagonal_length = std::hypot(length, width) * 0.5;
  float diagonal_angle = std::atan2(width, length);
  float diagonal_supplementary_angle = M_PI - diagonal_angle;
  std::vector<float> rotate_angles = {diagonal_angle, diagonal_supplementary_angle, -diagonal_supplementary_angle,
                                      -diagonal_angle};

  const float theta = obstacle_box.angle().z();
  const auto& center = obstacle_box.position();
  for (const auto& rotate_angle : rotate_angles) {
    auto* box_point = feature->mutable_box_point()->Add();
    float abs_diagonal_angle = rotate_angle + theta;
    box_point->set_x(center.x() + half_diagonal_length * std::cos(abs_diagonal_angle));
    box_point->set_y(center.y() + half_diagonal_length * std::sin(abs_diagonal_angle));
    box_point->set_z(center.z());
  }

  // ERT_PLOG_D << "Obstacle [" << id_ << "] has dimension [" << length << ", " << width << ", " << height << "].";
}

/**
 * @brief 根据子障碍物信息设置障碍物的状态信息。
 * @details 根据传入的子障碍物信息和特征指针设置障碍物的状态信息。
 *
 * @param[in] sub_obstacle 子障碍物信息。
 * @param[out] feature 指向障碍物特征的指针，用于存储状态信息。
 *
 * @par 输入参数说明:
 * - sub_obstacle: 有效的 `proto::SubObstacle` 对象，包含子障碍物信息。
 * - feature: 有效的 `Feature` 对象指针，用于存储状态信息。
 */
void Obstacle::SetState(const pnc::ObstacleBox& obstacle_box, Feature* feature) {
  SetPosition(obstacle_box, feature);
  SetAngle(obstacle_box, feature);
  SetVelocity(obstacle_box, feature);
  SetSpeedRatio(*feature);
  SetAcceleration(obstacle_box, feature);
  SetAngleRate(obstacle_box, feature);
}

/**
 * @brief 根据子障碍物信息设置障碍物的位置信息。
 * @details 根据传入的子障碍物信息和特征指针设置障碍物的位置信息。
 *
 * @param[in] sub_obstacle 子障碍物信息。
 * @param[out] feature 指向障碍物特征的指针，用于存储位置信息。
 *
 * @par 输入参数说明:
 * - sub_obstacle: 有效的 `proto::SubObstacle` 对象，包含子障碍物信息。
 * - feature: 有效的 `Feature` 对象指针，用于存储位置信息。
 */
void Obstacle::SetPosition(const pnc::ObstacleBox& obstacle_box, Feature* feature) {
  feature->mutable_position()->CopyFrom(obstacle_box.position());

  // ERT_PLOG_D << "Obstacle [" << id_ << "] has position [" << feature->position().x() << ", " << feature->position().y()
  //            << "].";
}

/**
 * @brief 根据子障碍物信息设置障碍物的速度信息。
 * @details 根据传入的子障碍物信息和特征指针设置障碍物的速度信息。
 *
 * @param[in] sub_obstacle 子障碍物信息。
 * @param[out] feature 指向障碍物特征的指针，用于存储速度信息。
 *
 * @par 输入参数说明:
 * - sub_obstacle: 有效的 `proto::SubObstacle` 对象，包含子障碍物信息。
 * - feature: 有效的 `Feature` 对象指针，用于存储速度信息。
 */
void Obstacle::SetVelocity(const pnc::ObstacleBox& obstacle_box, Feature* feature) {
  if (!obstacle_box.has_velocity()) {
    ERT_PLOG_W << "Obstacle[" << id_ << "] has no velocity.";
    return;
  }

  auto velocity = obstacle_box.velocity();
  if (!velocity.has_x() || std::isnan(velocity.x())) {
    ERT_PLOG_E << "Obstacle[" << id_ << "] has invalid velocity x(" << velocity.x() << "), set to 0.0";
    velocity.set_x(0.0);
  }

  if (!velocity.has_y() || std::isnan(velocity.y())) {
    ERT_PLOG_E << "Obstacle[" << id_ << "] has invalid velocity y(" << velocity.y() << "), set to 0.0";
    velocity.set_y(0.0);
  }

  feature->mutable_velocity()->CopyFrom(velocity);

  double speed = std::hypot(velocity.x(), velocity.y());
  feature->set_speed(speed);

  double velocity_heading = std::atan2(velocity.y(), velocity.x());
  feature->set_velocity_heading(velocity_heading);

  // if (id_ > config_.ego_vehicle_id()) {
  //   ERT_PLOG_D << "Obstacle [" << id_ << "] has velocity [" << velocity.x() << ", " << velocity.y() << "], speed ["
  //              << speed << "], velocity_heading [" << velocity_heading << "].";
  // }
}

/**
 * @brief 根据特征信息设置障碍物的速度比例。
 * @details 根据传入的特征信息设置障碍物的速度比例。
 *
 * @param[in] feature 障碍物的特征信息。
 *
 * @par 输入参数说明:
 * - feature: 有效的 `Feature` 对象，包含障碍物的特征信息。
 */
void Obstacle::SetSpeedRatio(const Feature& feature) {
  if (!feature.has_speed()) {
    ERT_PLOG_W << "Obstacle [" << id_ << "] try to set speed_ratio without speed";
    return;
  }

  auto speed = feature.speed();
  auto speed_threshold = speed < kHighSpeedLimitInTown ? kHighSpeedLimitInTown : kSpeedLimitInHighway;
  speed_threshold = speed_threshold < kLowSpeedLimitInTown ? kLowSpeedLimitInTown : speed_threshold;
  speed_ratio_ = speed / speed_threshold;
  // ERT_PLOG_D << "Obstacle [" << id_ << "] has speed_ratio [" << speed_ratio_ << "] for speed [" << speed
  //            << "] and speed_threshold [" << speed_threshold << "]";
}

/**
 * @brief 根据子障碍物信息设置障碍物的加速度信息。
 * @details 根据传入的子障碍物信息和特征指针设置障碍物的加速度信息。
 *
 * @param[in] sub_obstacle 子障碍物信息。
 * @param[out] feature 指向障碍物特征的指针，用于存储加速度信息。
 *
 * @par 输入参数说明:
 * - sub_obstacle: 有效的 `proto::SubObstacle` 对象，包含子障碍物信息。
 * - feature: 有效的 `Feature` 对象指针，用于存储加速度信息。
 */
void Obstacle::SetAcceleration(const pnc::ObstacleBox& obstacle_box, Feature* feature) {
  if (!obstacle_box.has_acceleration()) {
    ERT_PLOG_W << "Obstacle[" << id_ << "] has no acceleration.";
    return;
  }

  auto acceleration = obstacle_box.acceleration();
  if (!acceleration.has_x() || std::isnan(acceleration.x())) {
    ERT_PLOG_E << "Obstacle[" << id_ << "] has invalid acceleration x " << acceleration.x() << ", set to 0.0";
    acceleration.set_x(0.0);
  }

  if (!acceleration.has_y() || std::isnan(acceleration.y())) {
    ERT_PLOG_E << "Obstacle[" << id_ << "] has invalid acceleration y " << acceleration.y() << ", set to 0.0";
    acceleration.set_y(0.0);
  }

  feature->mutable_acceleration()->CopyFrom(acceleration);

  float velocity_heading = feature->velocity_heading();
  float acc = acceleration.x() * std::cos(velocity_heading) + acceleration.y() * std::sin(velocity_heading);
  feature->set_acc(acc);

  // if (id_ > config_.ego_vehicle_id()) {
  //   ERT_PLOG_D << "Obstacle [" << id_ << "] has acceleration [" << acceleration.x() << ", " << acceleration.y()
  //              << "], acc [" << acc << "].";
  // }
}

/**
 * @brief 根据子障碍物信息设置障碍物的角度信息。
 * @details 根据传入的子障碍物信息和特征指针设置障碍物的角度信息。
 *
 * @param[in] sub_obstacle 子障碍物信息。
 * @param[out] feature 指向障碍物特征的指针，用于存储角度信息。
 *
 * @par 输入参数说明:
 * - sub_obstacle: 有效的 `proto::SubObstacle` 对象，包含子障碍物信息。
 * - feature: 有效的 `Feature` 对象指针，用于存储角度信息。
 */
void Obstacle::SetAngle(const pnc::ObstacleBox& obstacle_box, Feature* feature) {
  if (!obstacle_box.has_angle()) {
    ERT_PLOG_W << "Obstacle[" << id_ << "] has no angle.";
    return;
  }

  if (std::isnan(obstacle_box.angle().z())) {
    ERT_PLOG_W << "Obstacle[" << id_ << "] has invalid angle " << obstacle_box.angle().z() << ", skip.";
    return;
  }
  feature->mutable_angle()->CopyFrom(obstacle_box.angle());
  // ERT_PLOG_D << "Obstacle [" << id_ << "] has theta [" << obstacle_box.angle().z() << "].";
}

/**
 * @brief 根据子障碍物信息设置障碍物的偏航率信息。
 * @details 根据传入的子障碍物信息和特征指针设置障碍物的偏航率信息。
 *
 * @param[in] sub_obstacle 子障碍物信息。
 * @param[out] feature 指向障碍物特征的指针，用于存储偏航率信息。
 *
 * @par 输入参数说明:
 * - sub_obstacle: 有效的 `proto::SubObstacle` 对象，包含子障碍物信息。
 * - feature: 有效的 `Feature` 对象指针，用于存储偏航率信息。
 */
void Obstacle::SetAngleRate(const pnc::ObstacleBox& obstacle_box, Feature* feature) {
  if (!obstacle_box.has_angular_rate()) {
    ERT_PLOG_W << "Obstacle[" << id_ << "] has no angular_rate.";
    return;
  }

  float yaw_rate = obstacle_box.angular_rate().z();
  if (std::isnan(yaw_rate)) {
    ERT_PLOG_E << "Obstacle[" << id_ << "] has invalid yaw_rate " << yaw_rate << ", set to 0.0";
    yaw_rate = 0.0;
  }
  feature->mutable_angular_rate()->CopyFrom(obstacle_box.angular_rate());
  // ERT_PLOG_D << fp6 << "Obstacle [" << id_ << "] has angular_rate raw[" << obstacle_box.angular_rate().z()
  //            << "], prediction[" << yaw_rate << "].";
}

/**
 * @brief 根据感知障碍物信息设置障碍物的运动状态。
 * @details 根据传入的感知障碍物信息和特征指针设置障碍物的运动状态。
 *
 * @param[in] planning_obstacle 感知到的障碍物信息。
 * @param[out] feature 指向障碍物特征的指针，用于存储运动状态信息。
 *
 * @par 输入参数说明:
 * - planning_obstacle: 有效的 `proto::PerceptionObstacle`
 * 对象，包含感知到的障碍物信息。
 * - feature: 有效的 `Feature` 对象指针，用于存储运动状态信息。
 */
void Obstacle::SetMotionStatus(const planning::Obstacle& planning_obstacle, Feature* feature) {
  if ((IsStaticObstacle(planning_obstacle.type()))
      || (kStillMotionStatus.find(planning_obstacle.motion_status()) != kStillMotionStatus.end())) {
    feature->set_is_still(true);
  } else {
    float still_speed_threshold = GetStillSpeedThreshold();
    float speed = feature->speed();

    feature->set_is_still(false);
    if (speed < still_speed_threshold) {
      feature->set_is_still(true);
    }
  }

  if (feature->is_still()) {
    feature->set_velocity_heading(feature->angle().z());
  }
  // ERT_PLOG_D << "Obstacle [" << id_ << "] has motion_status [" << planning_obstacle.motion_status() << "], speed ["
  //            << feature->speed() << "], is_still [" << feature->is_still() << "] ";
}

/**
 * @brief 将特征插入到历史记录中。
 * @details 根据传入的特征信息将其插入到障碍物的历史特征记录中。
 *
 * @param[in] feature 待插入的障碍物特征信息。
 *
 * @par 输入参数说明:
 * - feature: 有效的 `Feature` 对象，包含待插入的障碍物特征信息。
 */
void Obstacle::InsertFeatureToHistory(const Feature& feature) {
  feature_history_.emplace_front(std::move(feature));
}

/**
 * @brief 丢弃过时的历史记录。
 * @details 根据时间戳等信息丢弃障碍物历史特征记录中过时的记录。
 */
void Obstacle::DiscardOutdatedHistory() {
  if (feature_history_.empty()) {
    return;
  }

  const auto& latest_ts = feature_history_.front().timestamp();
  const auto& max_history_time = config_.container_config().obstacle_container_config().max_history_time();

  auto it = std::find_if(feature_history_.rbegin(), feature_history_.rend(),
                         [latest_ts, max_history_time](const auto& feature) {
                           return latest_ts - feature.timestamp() > max_history_time;
                         });

  // Erase outdated frames
  if (it != feature_history_.rend()) {
    auto erase_start = it.base() - 1;
    const size_t num_of_discarded_frames = std::distance(erase_start, feature_history_.end());
    feature_history_.erase(erase_start, feature_history_.end());
    // ERT_PLOG_D << fp4 << "Obstacle [" << id_ << "] discarded " << num_of_discarded_frames << " left "
    //            << feature_history_.size() << " frames ";
  }
}

}  // namespace prediction
}  // namespace pnc
}  // namespace gpal