#pragma once

#include "manager/spatiotemporal_planner_data_manager.h"
#include "spatiotemporal_abstract_module.h"

namespace gpal::pnc::planning {
class SpatiotemporalOptimizer : public SpatiotemporalAbstractModule {
 public:
  SpatiotemporalOptimizer() = default;
  ~SpatiotemporalOptimizer() = default;

  /**
   * @brief 获取模块 ID
   * @return std::string 模块 ID
   * @details 该方法返回当前模块的唯一标识符
   */
  std::string id() const override;

  bool init() override;
  void reset() override;

  bool run(SpatiotemporalPlannerDataManager& data_manager) override;

  std::string name() const { return "SpatiotemporalOcpOptimizer"; }
  bool hasAsyncProcess() const;
  void dropAsyncProcess();
  bool preProcess(SpatiotemporalPlannerDataManager& data_manager);

  SpatiotemporalPlannerDataManager::OptimizerInfo::AsyncStatus asyncProc(
      const std::chrono::milliseconds& timeout, SpatiotemporalPlannerDataManager& data_manager);

  // only for gtest
  void setSolverInfo(const SolveInfo& solver_info) { gtest_solver_info_ = solver_info; }
  const SolveInfo& getSolverInfo() const { return gtest_solver_info_; }

  bool hasValidPrevPath() { return std::get<0>(prev_traj_info_); }

 protected:
  // [ADD]: Added a snapshot struct to hold all data for a single solver run.
  // This isolates the worker thread from the main thread's state changes.
  struct SolverInputSnapshot {
    // Vehicle and planning parameters
    std::shared_ptr<VehicleParam> vehicle_param;
    std::pair<std::array<double, 3>, std::array<double, 3>> sl_planning_start_point;
    double lane_keep_start_s = 0.0;
    double cost_weight_max_decay_length = 100.0;

    // Reference line (copied)
    ReferenceLine target_ref_line;

    // Obstacle data (copied)
    std::vector<std::vector<ObjectInfo>> target_obstacle_data;
    std::array<bool, 4> obs_valid;

    // Bounds data (copied)
    math::IntervalData<Boundary> v_steer_bound;
    math::IntervalData<Boundary> v_dsteer_bound;
    math::IntervalData<Boundary> soft_speed_bound;
    math::IntervalData<Boundary> hard_speed_bound;
    math::IntervalData<Boundary> soft_lateral_bound;
    math::IntervalData<Boundary> hard_lateral_bound;
    math::IntervalData<Boundary> s_hard_bound;
    math::IntervalData<Boundary> s_soft_bound;
    math::IntervalData<ReferencePoint> ref_line_data;

    // Optimizer parameters and initial guess (copied)
    std::vector<std::unordered_map<std::string, std::pair<bool, double>>> optimizer_parameters;
    std::vector<SpatiotemporalState> init_guess;

    // Context for this specific run
    SpatiotemporalOptimizerContext context;
  };

  // [FIX]: Updated function signatures to accept the snapshot
  std::shared_ptr<OptimalControlProblem> initModel(const SpatiotemporalOptimizerContext& context,
                                                   const Trajectory& trajectory_data,
                                                   std::shared_ptr<const SolverInputSnapshot> snapshot);

  std::shared_ptr<OptimalControlProblem> initModel(const SpatiotemporalOptimizerContext& context,
                                                   const Trajectory& trajectory_data);

  bool initSpatiotemporalPlanner(std::shared_ptr<OptimalControlProblem> model);
  bool initSpatiotemporalPlanner(std::shared_ptr<OptimalControlProblem> model,
                                 std::shared_ptr<const SolverInputSnapshot> snapshot);
  bool updateSpatiotemporalPlanner(std::shared_ptr<OptimalControlProblem> model,
                                   std::shared_ptr<const SolverInputSnapshot> snapshot,
                                   const Trajectory& prev_trajectory);

  bool applySpatiotemporalXGuess(std::shared_ptr<OptimalControlProblem> model, const size_t idx);
  bool tranStatesToPathData(std::shared_ptr<OptimalControlProblem> model, Trajectory& trajectory_data);
  bool applyFrontWheelSteeringPolicy(std::shared_ptr<OptimalControlProblem> model,
                                     std::vector<SpatiotemporalState> coarse_states,
                                     std::shared_ptr<const SolverInputSnapshot> snapshot, const size_t idx);
  bool trajNonCollisionCheck(std::shared_ptr<OptimalControlProblem> model);

 private:
  bool hasValidPrevModel();
  void checkValid(std::shared_ptr<SpatiotemporalPlannerDataManager::OptimizerInfo::SolverData> data);
  bool resultCheck(std::shared_ptr<SpatiotemporalPlannerDataManager::OptimizerInfo::SolverData> data);
  const SpatiotemporalOptimizerContext& context() const;
  const DrivingDirection getInitPointDrivingDirection(const double s);
  double calcAlphaBetweenBox(const ObjectInfo& obs_box, const ObjectInfo& ego_box, const bool& is_obs_expanded,
                             math::Vec2d* nearest_pt);

  std::pair<std::tuple<double, double>, bool> getRefOffset(const ReferencePoint& rpt);

  const std::vector<std::tuple<std::string, float, float>>& getSpecialSceneRange() const;
  double getSpeedLimit(const double curr_s, const double ref_kappa);
  double linearInterpolation(double i, double N, double value_lower, double value_upper) {
    if (std::abs(N) < 1e-9)
      return value_lower;
    if (i >= N)
      return value_upper;

    const double step = (value_upper - value_lower) / N;
    double result = value_lower + i * step;

    // 边界截断功能
    return std::clamp(result, min(value_lower, value_upper), max(value_lower, value_upper));
  };
  double sgn(const double value, const double eps) { return value / (std::abs(value) + eps); }

  std::vector<std::pair<std::string, double>> interpolateMultiParams(
      double speed, const std::vector<double>& speed_vec,
      const std::vector<std::vector<std::pair<std::string, double>>>& params_vec);

 private:
  enum UpdateParamsMethod {
    SPATIOTEMPORAL = 0,
  };
  TaskHandler<1> task_handler_;
  std::shared_ptr<SpatiotemporalPlannerDataManager::OptimizerInfo::SolverData> async_data_ = nullptr;
  SpatiotemporalOptimizerProfile optimizer_config_;
  std::string curr_context_type_ = "";
  UpdateParamsMethod curr_update_params_method_ = UpdateParamsMethod::SPATIOTEMPORAL;
  std::shared_ptr<VehicleParam> vehicle_param_;  ///< 车辆配置参数
  double dt_ = 0.1;
  double horizon_ = 5.0;
  size_t N_ = 50;
  bool has_risk_detection_ = false;

  Eigen::VectorXd start_state_ = Eigen::VectorXd::Zero(7);
  std::tuple<std::string, std::string, UpdateParamsMethod, std::shared_ptr<OptimalControlProblem>, SolveStatus>
      prev_model_info_;
  int prev_iteration_ = 0;
  std::tuple<bool, double, TrajectoryPt> prev_traj_info_;
  std::unique_ptr<ReferenceLine> target_ref_line_ = nullptr;
  std::vector<double> accumulated_s_;
  std::pair<std::array<double, 3>, std::array<double, 3>> sl_planning_start_point_;
  TrajectoryPt xy_planning_start_point_;
  double lane_keep_start_s_ = 0.0;
  double cost_weight_max_decay_length_ = 100.0;  ///< 用于优化cost的最大衰减长度
  std::unordered_map<std::string, bool> current_driving_scenario_;
  std::vector<std::vector<ObjectInfo>> target_obstacle_data_;
  ParamIndexCache param_cache_;  ///< 参数索引缓存

  math::IntervalData<Boundary> v_steer_bound_;
  math::IntervalData<Boundary> v_dsteer_bound_;

  math::IntervalData<Boundary> soft_speed_bound_;
  math::IntervalData<Boundary> hard_speed_bound_;
  math::IntervalData<Boundary> soft_lateral_bound_;
  math::IntervalData<Boundary> hard_lateral_bound_;

  math::IntervalData<Boundary> s_hard_bound_;
  math::IntervalData<Boundary> s_soft_bound_;

  math::IntervalData<ReferencePoint> ref_line_data_;

  std::vector<std::unordered_map<std::string, std::pair<bool, double>>> optimizer_parameters_;

  std::vector<double> steer_speed_vec_;
  std::vector<double> steer_coffient_vec_;
  std::vector<double> dsteer_speed_vec_;
  std::vector<double> dsteer_coffient_vec_;

  std::vector<double> lane_change_speed_vec_;
  std::vector<std::vector<std::pair<string, double>>> lane_change_params_vec_;

  std::string async_optimizer_debug_info_ = "";
  // only for gtest
  SolveInfo gtest_solver_info_;
  std::vector<SpatiotemporalState> init_guess_;

  // ===== box碰撞优化求解相关 ===== //

  /// @brief 二次规划求解器（用于α计算）
  std::shared_ptr<QuadraticProgrammingSolver> alpha_optimizer_{nullptr};

  /// @brief 线性目标函数系数（3维向量）
  Eigen::Vector3d c_{0, 0, 1};

  /// @brief 二次规划权重矩阵（3x3矩阵）
  Eigen::Matrix<double, 3, 3> Q_ = Eigen::Matrix<double, 3, 3>::Zero(3, 3);

  /// @brief 不等式约束矩阵（9x3矩阵）
  Eigen::Matrix<double, 9, 3> G_;

  /// @brief 不等式约束边界（9x1向量）
  Eigen::Matrix<double, 9, 1> h_;
};

}  // namespace gpal::pnc::planning
