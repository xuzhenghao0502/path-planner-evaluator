#pragma once

/**
 * @file
 * @brief Defines the function state class.
 */

namespace gpal::pnc::planning {

enum class ScenarioState {
  ManualScenario = 0,
  AccScenario = 1,
  LccScenario = 2,
  HighwayNoaScenario = 3,
  CityNoaScenario = 4,
  ApaScenario = 5,
  HpaScenario = 6,
  RemoteControlScenario = 7
};

enum class StageState {
  IdleStage = 0,
  AccDrivingStage = 1,
  LccDrivingStage = 2,
  HighwayNoaDrivingStage = 3,
  CityNoaDrivingStage = 4,
  HpaDrivingStage = 5,
  ParkInStage = 6,
  ParkOutStage = 7,
  RemoteControlStage = 8
};

}  // namespace gpal::pnc::planning