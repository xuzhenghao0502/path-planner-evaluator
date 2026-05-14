#pragma once

#include <memory>
#include <unordered_map>
#include <utility>

#include "base_parking_planner.h"
#include "parallel_parking_in_planner.h"
#include "vertical_parking_in_planner.h"
#include "vertical_parking_out_planner.h"
#include "parallel_parking_out_planner.h"
#include "escape_planner.h"

namespace gpal::pnc::planning {

/**
 * @brief 泊车路径规划器工厂类，用于根据泊车模式和类型创建对应的规划器实例。
 *
 * 该工厂类根据传入的泊车模式（垂直泊入 / 平行泊入等）和泊车类型（AVP/RPA等）生成对应的派生类规划器对象。
 */
class ParkingPlannerFactory {
 public:
  /**
   * @brief 泊车规划器的组合类型，包含泊车模式和泊车类型的组合。
   */
  using ParkingPlannerKey = std::pair<ParkingMode, ParkingType>;

  /**
   * @brief 创建对应类型的泊车路径规划器。
   *
   * @param key 包含泊车模式和泊车类型的键。
   * @return std::unique_ptr<BaseParkingPlanner> 返回对应类型的规划器对象；若不支持该类型则返回 nullptr。
   */
  static std::unique_ptr<BaseParkingPlanner> Create(const ParkingPlannerKey& key) {
    switch (key.first) {
      case ParkingPathPlannerProfile::VERTICAL_IN:
        return std::make_unique<VerticalParkingInPlanner>(key);
      case ParkingPathPlannerProfile::PARALLEL_IN:
        return std::make_unique<ParallelParkingInPlanner>(key);
      case ParkingPathPlannerProfile::VERTICAL_OUT:
        return std::make_unique<VerticalParkingOutPlanner>(key);
      case ParkingPathPlannerProfile::PARALLEL_OUT:
        return std::make_unique<ParallelParkingOutPlanner>(key);
      case ParkingPathPlannerProfile::ESCAPE:
        return std::make_unique<EscapePlanner>(key);
      default:
        return nullptr;
    }
  }
};
}  // namespace gpal::pnc::planning
