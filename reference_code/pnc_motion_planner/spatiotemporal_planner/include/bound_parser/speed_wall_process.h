#pragma once

#include "config/spatiotemporal_planner/longitudinal_bound_parser_config.pb.h"
#include "manager/spatiotemporal_planner_data_manager.h"
#include "speed/speed_result.h"

namespace gpal::pnc::planning {

class SpeedWallProcess {
 public:
  SpeedWallProcess() = default;

  ~SpeedWallProcess() = default;

  using DataManager = SpatiotemporalPlannerDataManager;

  bool init(LongitudinalBoundParserProfile profile);
  void getSpeedWall(const ReferenceLineInfo* reference_line_info, const Chassis* chassis,
                    const unique_ptr<DataManager::VehicleInfo>& vehicle_info,
                    const pair<bool, double>& destination_remain_dis_info, const DecisionResult& decision_result,
                    std::vector<SpeedWall>* speed_walls);

 private:
  void addTflSpeedWall(const ReferenceLineInfo* reference_line_info, const Chassis* chassis,
                       const unique_ptr<DataManager::VehicleInfo>& vehicle_info,
                       const TrafficLightDecision& traffic_light_decision, std::vector<SpeedWall>* speed_walls);

  void addDecisionStopSpeedWall(const unique_ptr<DataManager::VehicleInfo>& vehicle_info,
                                const LongitudinalBoundDecision& longitudinal_bound_decision,
                                std::vector<SpeedWall>* speed_walls);
  void addDestinationSpeedWall(const pair<bool, double>& destination_remain_dis_info,
                               std::vector<SpeedWall>* speed_walls);

 private:
  ConfigManager* config_manager_ = nullptr;
  VehicleParam vehicle_param_;
  LongitudinalBoundParserProfile profile_;
  double time_horizon_{8.0};
};

}  // namespace gpal::pnc::planning