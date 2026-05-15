#pragma once

#include "config/spatiotemporal_planner/longitudinal_bound_parser_config.pb.h"
#include "manager/spatiotemporal_planner_data_manager.h"
#include "speed/speed_data.h"

namespace gpal::pnc::planning {

class SpeedLimitProcess {
 public:
  enum class SpeedLimitType {
    kMap = 0,
    kConsole = 1,
  };

 public:
  SpeedLimitProcess() = default;
  ~SpeedLimitProcess() = default;

  using DataManager = SpatiotemporalPlannerDataManager;

  bool init(LongitudinalBoundParserProfile profile);
  void reset();
  std::vector<SpatialSpeedLimit> getSpeedLimit(const ReferenceLineInfo* reference_line_info,
                                               const math::IntervalData<ReferencePoint>& reference_points,
                                               const Console* console, const Chassis* chassis,
                                               const unique_ptr<DataManager::VehicleInfo>& vehicle_info,
                                               const StageState& stag_state, const proto::road_cognition::ScenarioInfo& scenario_info,
                                               const DecisionResult& decision_result);
  std::vector<SpatialSpeedLimit> getMapSpeedLimit() { return map_speed_limit_; }
  std::vector<SpatialSpeedLimit> getCurveSpeedLimit() { return curve_speed_limit_; }
  double getMaxSpeedLimit() { return max_speed_limit_; }

 private:
  void getMapSpeedlimit(const ReferenceLineInfo* reference_line_info,
                        const std::vector<ReferencePoint>& reference_points);
  void getNoaSpeedLimit(const ReferenceLineInfo* reference_line_info,
                        const std::vector<ReferencePoint>& reference_points, const Console* console,
                        const StageState& stage_state);
  void getCurveSpeedLimit(const std::vector<ReferencePoint>& reference_points);
  void getDecisionSpeedLimit(const Chassis* chassis, const DecisionResult& decision_result,
                             const unique_ptr<DataManager::VehicleInfo>& vehicle_info);
  void getTurnSpeedLimit(const ReferenceLineInfo* reference_line_info, const int& turn_state,
                         const unique_ptr<DataManager::VehicleInfo>& vehicle_info);
  void getConstSpeedLimit(const Console* console, const StageState& stage_state, const proto::road_cognition::ScenarioInfo& scenario_info);
  pair<string, double> getDrivingSpeedLimit(const Console* console);
  pair<string, double> getLccDrivingSpeedLimit(const Console* console);
  pair<string, double> getHpaSpeedLimit();
  pair<string, double> getScenarioSpeedLimit( const proto::road_cognition::ScenarioInfo& scenario_info);
  void updateSpeedLimit(double speed_limit, double speed_limit_dis, string speed_limit_type);
  void smoothSpeedLimits();
  void outCurveSpeedLimit(const ReferenceLineInfo* reference_line_info, const Chassis* chassis,
                          const unique_ptr<DataManager::VehicleInfo>& vehicle_info);
  void caculateTurnFlag(const ReferenceLineInfo* reference_line_info,
                        const unique_ptr<DataManager::VehicleInfo>& vehicle_info);

  float getMapSpeedLimitFromS(std::vector<SpeedLimit> map_speed_limits, double s );

 private:
  ConfigManager* config_manager_ = nullptr;
  VehicleParam vehicle_param_;
  LongitudinalBoundParserProfile profile_;

  std::vector<SpatialSpeedLimit> speed_limits_;
  std::vector<SpatialSpeedLimit> map_speed_limit_;
  std::vector<SpatialSpeedLimit> curve_speed_limit_;
  double max_speed_limit_ = kMaxSpeedMS;
  bool last_steer_curve_acc_limit_ = false;
  double init_v_ = 0.0;
  size_t init_index_ = 0;

  bool is_turn_around_ = false;
  int last_turn_state_ = 0;

  double last_map_speed_limit_ = kMaxSpeedMS;
  double last_console_speed_limit_ = kMaxSpeedMS;
  SpeedLimitType speed_limit_type_ = SpeedLimitType::kMap;
};

}  // namespace gpal::pnc::planning