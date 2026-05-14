#pragma once

#include <string_view>
#include <array>
#include "config/path_planner/parking_path_planner_config.pb.h"

namespace gpal::pnc::planning {

// === 类型别名定义（便于阅读） ===
using ParkingMode = ParkingPathPlannerProfile::ParkingMode;
using ParkingType = ParkingPathPlannerConfig::ParkingType;
using GeneratorType = ParkingPathGeneratorProfile::GeneratorType;
using OptimizerType = ParkingPathOptimizerProfile::OptimizerType;

namespace {

// === 枚举与字符串映射表 ===

/**
 * @brief ParkingMode ↔ 字符串映射表
 */
constexpr std::array<std::pair<ParkingMode, std::string_view>, 6> kParkingModeArray = {{
    {ParkingPathPlannerProfile::PARALLEL_IN, "parallel_parking_in"},
    {ParkingPathPlannerProfile::VERTICAL_IN, "vertical_parking_in"},
    {ParkingPathPlannerProfile::PARALLEL_OUT, "parallel_parking_out"},
    {ParkingPathPlannerProfile::VERTICAL_OUT, "vertical_parking_out"},
    {ParkingPathPlannerProfile::ESCAPE, "escape"},
    {ParkingPathPlannerProfile::UNKNOWN_PARKING_MODE, "unknown_parking_mode"},
}};

/**
 * @brief GeneratorType ↔ 字符串映射表
 */
constexpr std::array<std::pair<GeneratorType, std::string_view>, 3> kGeneratorTypeArray = {{
    {ParkingPathGeneratorProfile::HYBRID_A_STAR, "hybrid_a_star"},
    {ParkingPathGeneratorProfile::GEOMETRY, "geometry"},
    {ParkingPathGeneratorProfile::UNKNOWN_GENERATOR, "unknown_generator"},
}};

/**
 * @brief OptimizerType ↔ 字符串映射表
 */
constexpr std::array<std::pair<OptimizerType, std::string_view>, 2> kOptimizerTypeArray = {{
    {ParkingPathOptimizerProfile::IPM_OCP, "ipm_ocp"},
    {ParkingPathOptimizerProfile::UNKNOWN_OPTIMIZER, "unknown_optimizer"},
}};

/**
 * @brief ParkingType ↔ 字符串映射表
 */
constexpr std::array<std::pair<ParkingType, std::string_view>, 5> kParkingTypeArray = {{
    {ParkingPathPlannerConfig::AVP, "avp"},  ///< 全自动泊车（Autonomous Valet Parking）
    {ParkingPathPlannerConfig::RPA, "rpa"},  ///< 遥控泊车（Remote Parking Assist）
    {ParkingPathPlannerConfig::HPP, "hpp"},  ///< 记忆泊车（Home Parking Pilot）
    {ParkingPathPlannerConfig::APA, "apa"},  ///< 自动泊车辅助（Automated Parking Assist）
    {ParkingPathPlannerConfig::UNKNOWN_PARKING_TYPE, "unknown_parking_type"},
}};

// === ParkingMode 映射函数 ===

/**
 * @brief ParkingMode → 字符串
 */
inline std::string_view parkingModeToStr(ParkingMode mode) {
  for (const auto& [k, v] : kParkingModeArray) {
    if (k == mode) return v;
  }
  return "unknown_parking_mode";
}

/**
 * @brief 字符串 → ParkingMode
 */
inline ParkingMode strToParkingMode(std::string_view str) {
  for (const auto& [k, v] : kParkingModeArray) {
    if (v == str) return k;
  }
  return ParkingPathPlannerProfile::UNKNOWN_PARKING_MODE;
}

// === GeneratorType 映射函数 ===

/**
 * @brief GeneratorType → 字符串
 */
inline std::string_view generatorTypeToStr(GeneratorType type) {
  for (const auto& [k, v] : kGeneratorTypeArray) {
    if (k == type) return v;
  }
  return "unknown_generator";
}

/**
 * @brief 字符串 → GeneratorType
 */
inline GeneratorType strToGeneratorType(std::string_view str) {
  for (const auto& [k, v] : kGeneratorTypeArray) {
    if (v == str) return k;
  }
  return ParkingPathGeneratorProfile::UNKNOWN_GENERATOR;
}

// === OptimizerType 映射函数 ===

/**
 * @brief OptimizerType → 字符串
 */
inline std::string_view optimizerTypeToStr(OptimizerType type) {
  for (const auto& [k, v] : kOptimizerTypeArray) {
    if (k == type) return v;
  }
  return "unknown_optimizer";
}

/**
 * @brief 字符串 → OptimizerType
 */
inline OptimizerType strToOptimizerType(std::string_view str) {
  for (const auto& [k, v] : kOptimizerTypeArray) {
    if (v == str) return k;
  }
  return ParkingPathOptimizerProfile::UNKNOWN_OPTIMIZER;
}

// === ParkingType 映射函数 ===

/**
 * @brief ParkingType → 字符串
 */
inline std::string_view parkingTypeToStr(ParkingType type) {
  for (const auto& [k, v] : kParkingTypeArray) {
    if (k == type) return v;
  }
  return "unknown_parking_type";
}

/**
 * @brief 字符串 → ParkingType
 */
inline ParkingType strToParkingType(std::string_view str) {
  for (const auto& [k, v] : kParkingTypeArray) {
    if (v == str) return k;
  }
  return ParkingPathPlannerConfig::UNKNOWN_PARKING_TYPE;
}

}  // namespace
}  // namespace gpal::pnc::planning
