#pragma once

#include "config/path_planner/parking_path_planner_config.pb.h"

namespace gpal::pnc::planning {

/**
 * @brief ProfileTraits 模板结构体，用于从上层结构中获取子配置 profiles。
 *
 * 该结构用于配置系统中的统一访问接口，根据类型特化，提供对应的 `getProfiles()` 方法。
 * 常用于加载 proto 中的 map<string, profile> 类型配置，避免重复模板代码。
 *
 * @tparam T 配置类型，如 ParkingPathPlannerProfile、PlannerProfile 等
 */
template <typename T>
struct ProfileTraits;  // 默认未实现，需要特化

// === 特化版本：ParkingPathPlannerProfile 从 ParkingPathPlannerConfig 中提取 ===

/**
 * @brief ParkingPathPlannerProfile 特化版本。
 * 从 ParkingPathPlannerConfig 中获取顶层 profiles。
 */
template <>
struct ProfileTraits<ParkingPathPlannerProfile> {
  /**
   * @brief 获取 config.profiles()
   * @param config ParkingPathPlannerConfig 对象
   * @return const 引用，指向 profiles() map
   */
  static const auto& getProfiles(const ParkingPathPlannerConfig& config) { return config.profiles(); }
};

// === 特化版本：PlannerProfile 从 ParkingPathPlannerProfile 中提取 ===

/**
 * @brief PlannerProfile 特化版本。
 * 从 ParkingPathPlannerProfile 中获取 planner_profiles。
 */
template <>
struct ProfileTraits<PlannerProfile> {
  static const auto& getProfiles(const ParkingPathPlannerProfile& profile) { return profile.planner_profiles(); }
};

// === 特化版本：ParkingPathGeneratorProfile 从 PlannerProfile 中提取 ===

/**
 * @brief ParkingPathGeneratorProfile 特化版本。
 * 从 PlannerProfile 中获取 generator_profiles。
 */
template <>
struct ProfileTraits<ParkingPathGeneratorProfile> {
  static const auto& getProfiles(const PlannerProfile& profile) { return profile.generator_profiles(); }
};

// === 特化版本：ParkingPathOptimizerProfile 从 PlannerProfile 中提取 ===

/**
 * @brief ParkingPathOptimizerProfile 特化版本。
 * 从 PlannerProfile 中获取 optimizer_profiles。
 */
template <>
struct ProfileTraits<ParkingPathOptimizerProfile> {
  static const auto& getProfiles(const PlannerProfile& profile) { return profile.optimizer_profiles(); }
};

// 🔧 如果未来有新的 profile 类型，只需继续添加特化即可。

}  // namespace gpal::pnc::planning
