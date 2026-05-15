#pragma once

#include "manager/spatiotemporal_planner_data_manager.h"
#include "spatiotemporal_abstract_module.h"

namespace gpal::pnc::planning {

class SpatiotemporalOptimizerLoader : public SpatiotemporalAbstractModule {
 public:
  SpatiotemporalOptimizerLoader() = default;
  ~SpatiotemporalOptimizerLoader() = default;

  /**
   * @brief 获取模块 ID
   * @return std::string 模块 ID
   * @details 该方法返回当前模块的唯一标识符
   */
  std::string id() const override;

  bool init() override;
  void reset() override;
  bool run(SpatiotemporalPlannerDataManager& data_manager) override;

  const ReferencePoint getInitPoint(const double s) { return target_ref_line_->getReferencePoint(s); }

  const ReferencePoint getInitPoint(const double x, const double y) {
    return target_ref_line_->getReferencePoint(x, y);
  }

  void constructFollowTrajectoryTarget(
      const SpatiotemporalPlannerDataManager::BoundaryInfo& boundary_info, const Decision::RefTargetPoint& target_point_info);  // get feature point and target point
  std::vector<std::vector<ObjectInfo>> extractFollowObjects();
  std::vector<std::vector<ObjectInfo>> extractInteractionObjects();
  void constructInteractionInfo(std::vector<std::vector<ObjectInfo>> key_object, std::vector<InteractionInfo>& interaction_infos);
  unordered_map<string, std::pair<double, double>> processOppositeObstacles();
  std::vector<std::vector<ObjectInfo>> extractOppositeObjects(
      const unordered_map<string, std::pair<double, double>>& opposite_obs_feature_map);
  void extractNearestObstacle(const std::vector<std::vector<ObjectInfo>>& follow_object,
                              std::vector<std::vector<ObjectInfo>>& opposite_object,
                              const SpatiotemporalPlannerDataManager::BoundaryInfo& boundary_info,
                              string& nearest_obs_id, double& nearest_obs_s, double& nearest_obs_speed);
  void initializeObjectiveTarget(std::vector<SpatiotemporalState>& objective_target);

  std::tuple<double, std::string, double> calculateEmpiricalAcceleration(
      int step_index, double prev_s, double prev_v, const math::IntervalData<Boundary>& v_soft_bound,
      const math::IntervalData<Boundary>& s_soft_bound, const std::vector<ObjectInfo>& follow_obstacles,
      const std::vector<ObjectInfo>& opposite_obstacles,
      const unordered_map<string, std::pair<double, double>>& opposite_obs_feature_map);
  double calculateControlInput(double prev_a, double empirical_a, double k_4, string id, double ttc);
  std::tuple<double, double, double> updateState(double prev_s, double prev_v, double prev_a, double empirical_u);
  void updateObjectiveTargetParameters(std::vector<std::pair<bool, double>> feature_s_points,
                                                                    bool need_stop, std::vector<std::unordered_map<std::string, std::pair<bool, double>>> &optimizer_parameters);

  void constructConflictAreaTarget();

  void constructTerminalTarget();  // l_terminal theta_terminal s_terminal  v_terminal a_terminal

  void outputProcess(SpatiotemporalPlannerDataManager& data_manager);

  bool checkInitGuessValid(const DecisionResult* decision_result);

  double calcLaneKeepStartS(const Decision::RefTargetPoint& target_point_info);

  void currentDrivingScenario();

  double calculateTargetAcc(double s_gap, double v_ego, double acc_ego, double v_obs, double acc_obs);
  double calcEmpiricalAcc(const double& ego_v, const double& delta_dist, const double& delta_v, const double& front_a,
                          const double& k_0, const double& k_1, const double& k_2, const double& acc_coefficient);
  void updateRoadScene();
  void getCurveScene();
  void CalculateSteerFromGuess(std::vector<SpatiotemporalState>& init_guess, double wheelbase);
  void CalculateControlInputs(std::vector<SpatiotemporalState>& init_guess, double dt);
  double calculateConfidentTime(const std::vector<ObjectInfo>& obj_info, double ego_speed);
  double cacualteMinDesireBound(const std::vector<ObjectInfo>& obj_info, double ego_s, double ego_speed);
  bool needStopBasedOnNearestObstacle(string nearest_obs_id, double nearest_obs_distance, double nearest_obs_v,
                                      double ego_speed);

  double caculateGamma(double ego_speed, double obj_distance, double obj_speed);
  double getThw(FollowingDistanceLevel following_distance_level);
  double getMaxAccLimitBasedScenario(const proto::road_cognition::ScenarioInfo& scenario_info);
  void updateLateralOffset(const SpatiotemporalPlannerDataManager::BoundaryInfo& boundary_info,
                           const std::vector<std::vector<ObjectInfo>> target_obstacle_data,
                           std::vector<std::unordered_map<std::string, std::pair<bool, double>>> &optimizer_parameters);
  std::vector<std::vector<ObjectInfo>> extractNudgeObjects(const std::vector<std::vector<ObjectInfo>> target_obstacle_data);

 private:
  ConfigManager* config_manager_ = nullptr;
  VehicleParam vehicle_param_;
  SpatiotemporalOptimizerProfile optimizer_config_;
  proto::road_cognition::ScenarioInfo scenario_info_;
  std::unique_ptr<ReferenceLine> target_ref_line_ = nullptr;
  std::unique_ptr<ReferenceLine> current_ref_line_ = nullptr;
  std::pair<std::array<double, 3>, std::array<double, 3>> sl_planning_start_point_;
  std::pair<std::array<double, 3>, std::array<double, 3>> driven_sl_info_;  ///< 车辆状态信息 (s, l) 对应的 (x, y) 坐标
  TrajectoryPt xy_planning_start_point_;
  std::vector<std::vector<ObjectInfo>> target_obstacle_data_;
  std::vector<SpatiotemporalState> init_guess_;
  double boundary_length_ = 150.0;
  double dt_ = 0.1;
  double horizon_ = 5.0;
  size_t N_ = 50;

  FsmState behavior_ = FsmState::KEEP;

  std::vector<ConflictArea> conflict_areas_;

  string nearest_obs_id_ = "";
  double nearest_obs_s_ = std::numeric_limits<double>::max();
  double nearest_obs_speed_ = 0.0;

  Localization localization_;
  double thw_ = 1.5;
  string model = "SpatiotemporalPlannerModel";
  std::vector<std::unordered_map<std::string, std::pair<bool, double>>> optimizer_parameters_;  // {{"parameter_name(s_coarse)", {is_valid, value}}
  std::vector<std::pair<bool, double>> feature_s_points_; // size = N_, {is_feature_point, s_value}
  std::vector<std::pair<bool, double>> feature_v_points_; // size = N_, {is_feature_point, v_value}
  std::vector<InteractionInfo> interaction_infos_; // size = N_
  bool need_stop_ = false;

  std::unordered_map<std::string, std::pair<bool, double>> optimizer_parameter_ = {
      {"l_terminal", {false, 0.0}},  {"theta_terminal", {false, 0.0}}, {"s_terminal", {false, 1000.0}},
      {"v_terminal", {false, 20.0}}, {"a_terminal", {false, 0.0}},     {"l_offset", {false, 0.0}},
      {"a_offset", {false, 0.0}},    {"s_coarse", {false, 1000.0}},    {"v_coarse", {false, 20.0}},
      {"a_coarse", {false, 0.0}}};

  std::string debug_info_ = "";
  std::unordered_map<std::string, bool> current_driving_scenario_;
  std::pair<bool, double> destination_remain_dis_info_ = {false, 10000.0};  ///< FIRST: 是否有终点信息, SECOND: 剩余距离信息
  TrajectoryPt destination_point_ = TrajectoryPt();  ///< 目的地参考点
  EnvRoadCognition env_road_cognition_ = EnvRoadCognition();  ///< 环境道路认知信息
};

}  // namespace gpal::pnc::planning
