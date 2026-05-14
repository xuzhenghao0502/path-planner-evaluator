/**
 * @file speed_model_param.h
 * @brief 速度规划模型参数生成器
 * @details 本类负责速度规划问题中各类约束条件和优化目标的参数计算与维护
 */
#pragma once
#include <string>
#include <unordered_map>
#include <vector>

#include "config/speed_planner/speed_ocp_qp_optimizer.pb.h"
#include "config/speed_planner/speed_planner.pb.h"
#include "config_manager/config_manager.h"
#include "local_view/local_view.h"
#include "math/math_utils.h"
#include "speed/st_graph.h"
#include "speed_common/speed_common.h"

namespace gpal::pnc::planning {

class SpeedModelParam {
 public:
 /**
 * @brief 构造函数
 * @param[in] time_grid 时间采样网格数组
 * @param[in] drivable_boundary_dt 可行驶边界时间间隔
 * @param[in] time_horizon 规划时间范围
 * 
 * @par 输入参数说明:
 * | 参数                    | 类型            | 取值范围          | 单位 | 说明                  |
 * |-------------------------|-----------------|------------------|------|-----------------------|
 * | time_grid               | vector<double> | [0.0, time_horizon] | 秒 | 优化问题时间离散序列  |
 * | drivable_boundary_dt    | double          | (0.0, 1.0]       | 秒   | 可行驶边界更新频率    |
 * | time_horizon            | double          | [5.0, 15.0]      | 秒   | 最大规划时间范围      |
 */
  SpeedModelParam(std::vector<double> time_grid, double drivable_boundary_dt, double time_horizon)
      : time_grid_(time_grid), drivable_boundary_dt_(drivable_boundary_dt), time_horizon_(time_horizon) {};
  /// @brief 析构函数
  ~SpeedModelParam() = default;
  void init();
  std::vector<std::unordered_map<std::string, double>> calculateSpeedModelParam(
      const LocalView& local_view, const DecisionResult& decision_result,
      const ObstacleSet& obstacle_map, const PathGroup& path_group,
      const SpeedState& speed_init_state, std::shared_ptr<SpeedResult> speed_result);
  void setParameter(const std::string& param_name, size_t i, double param_val);
  double getParameter(const std::string& param_name, size_t i);
  /**
   * @brief 获取不同离散时间下的限速信息
   *
   * @return 三维限速表
   */
  const std::vector<math::IntervalData<SpatialSpeedLimit>>& getStSpeedLimit() const { return st_speed_limit_; }  

 private:
  void investigateNearestObj(const ObstacleSet& obstacle_map,
                             const StGraph& st_graph, const std::vector<SpeedWall> speed_walls,
                             InvasionObstacle* nearest_invasion_obstacle);

  void updateMemberVariables(const StGraph& st_graph, const LocalView& local_view, const DiscretizedPath& path,
                             const SpeedLimitResult& speed_limit_result,const std::vector<SpeedWall>& speed_walls);
  void calcCompleteSHardUpperBounds(const StGraph& st_graph);
  void calcCompleteSSoftLowerBounds(const StGraph& st_graph);
  void calcCompleteSSoftUpperBounds(const StGraph& st_graph, const std::vector<SpeedWall>& speed_walls);
  void CalcYieldObstacleSoftUpperBounds(const STBoundary* st_boundary_ptr,
                                        std::vector<STPoint>& st_soft_upper_boundary);
  void calcCompleteVSoftBounds(const LocalView& local_view, const StGraph& st_graph, const DiscretizedPath& path,
                               const vector<proto::TrajectoryPoint>& path_v_t);
  double calcAntiCollisionAccel(const std::vector<std::pair<double, double>>& complete_s_hard_upper_bounds);
  void calcEmpiricalAccelRange(double& min_a, double& max_a);
  void calcEmpiricalAccelRangeForObs(const SpeedPoint& obj_invasion_point, double& min_a, double& max_a);
  double calcEmpiricalMaxAccelForSpeedLimit(double speedLimit) const;

  void calcK(const LocalView& local_view);
  double calcKSlope(double init_K, double dv_risk);
  double getAccThw(FollowingDistanceLevel following_distance_level);

  Status setContraintParam(const LocalView& local_view, const StGraph& st_graph, const double& max_speed_limit);

  void setWeightParam();
  void weightsProtection();

  //special cases
  void specialCaseOfVehicleStart(double& s_weight);

  void updateDecisionCoarseTraj(const DecisionResult& decision_result);
  bool getRefFromDecisionResult(const Decision::RefTrajInfo& trajectory, const double& t, double& v_ref, double& s_ref);

//   void specialCaseForStop();
  void specialCaseForObstacleStop();
//   void setBoundaryForStaggerDriving(const PathGroup& path_group,
//                                     const ObstacleSet& obstacle_map);
  void localPathRiskyConsideration(const LocalView& local_view,std::shared_ptr<SpeedResult> speed_result);

  void specialCaseForLaneChange(const DecisionResult& decision_result, const ObstacleSet& obstacle_set, const StGraph& st_graph);
  void specialCaseForOvertakeObstacle(const DecisionResult& decision_result, const ObstacleSet& obstacle_set, const StGraph& st_graph);

  void specialCaseForMergeObstacle(const PathGroup& path_group, const ObstacleSet& obstacle_map);
  //
  std::string getSpeedWallIdThroughType(const SpeedWallType& type) const;
  void boundValidProtect(const double& hard_upper, const double& hard_lower, double& soft_upper, double& soft_lower);
  double getStaticVSoftBound(vector<proto::TrajectoryPoint> path, double t);
  std::shared_ptr<SpeedPlannerObstacle> findObstacle(const std::string& id);

  //risk speed limit
  void calcRiskFieldSpeedLimit(const LocalView& local_view,
                               const ObstacleSet& obstacle_map,
                               const PathGroup& path_group, std::shared_ptr<SpeedResult> speed_result);
  std::vector<SpatialSpeedLimit> generateObsSpeedLimitVecByTimeIndex(const LocalView& local_view, 
      const ObstacleSet& obstacle_map, const PathGroup& path_group,
      std::shared_ptr<SpeedResult> speed_result, int time_index, double interval);
  bool updateSingleFrameSpeedLimit(const PathGroup& path_group,
                                   const std::vector<SpatialSpeedLimit>& input_speed_limit_vec,
                                   std::vector<SpatialSpeedLimit>& res_speed_limit_vec);

 private:
 // ===== 时间域参数 ===== //
  /// @brief 时间网格划分（单位：秒）
  std::vector<double> time_grid_;
  
  /// @brief 优化问题阶段数（根据时间网格自动计算）
  size_t stage_num_{0};
  
  /// @brief 可行驶边界时间间隔（默认0.1秒）
  double drivable_boundary_dt_{0.1};
  
  /// @brief 规划时间范围（默认8秒）
  double time_horizon_{8.0};

  // ===== 配置管理 ===== //
  /// @brief 配置管理器指针（延迟初始化）
  ConfigManager* config_manager_ = nullptr;
  
  /// @brief 车辆物理参数（轴距/质量等）
  VehicleParam vehicle_param_;
  
  /// @brief 速度规划器全局配置
  SpeedPlannerConfig speed_planner_config_;
  
  /// @brief 二次规划优化器配置参数
  SpeedOcpQpOptimizerConfig speed_ocp_qp_optimizer_config_;


  // ===== 初始状态 ===== //
  /// @brief 初始纵向位置（单位：米）
  double init_s_;
  
  /// @brief 初始速度（单位：m/s）
  double init_v_;
  
  /// @brief 初始加速度（单位：m/s²）
  double init_a_;
  
  /// @brief 初始速度状态（包含v0,a0）
  SpeedState x_0_;
  
  /// @brief 当前行为状态（含跟车/换道等状态）
  BehaviorState behavior_state_;
  
  /// @brief 障碍物集合（含分类和预测信息）
  ObstacleSet obstacle_map_;

  // Nearest obj infomation
  // ===== 关键障碍物信息 ===== //
  /// @brief 前方存在让行障碍物标志
  bool has_front_yield_obj_;
  
  /// @brief 最近前方障碍物ID
  string nearest_front_obj_id_;
  
  /// @brief 最近障碍物是否为静态障碍物
  bool nearest_front_obj_obstacle_ = false;
  
  /// @brief 障碍物入侵路径起点s坐标
  double nearest_front_obj_invasion_s_;
  
  /// @brief 障碍物入侵路径时间点
  double nearest_front_obj_invasion_t_;
  
  /// @brief 障碍物入侵时速度
  double nearest_front_obj_invasion_v_;
  // ===== 安全距离参数 ===== //
  /// @brief 自车与障碍物速度差（ego_v - obs_v）
  double deltaV_;
  
  /// @brief 安全距离补偿量：ΔS = S_obs - S_ego - (K + αΔV)*V_ego
  double deltaS_;
  
  /// @brief 速度差对K值的补偿系数（默认0.3）
  double delta_v_slope_for_K_ = 0.3;
  
  /// @brief 初始车头时距（Headway Time）
  double initHWT_;
  
  /// @brief 初始安全距离系数
  double init_K_;
  
  /// @brief 风险速度差（用于安全距离计算）
  double dv_risk_;
  
  /// @brief 防碰撞需求加速度
  double antiCollisionAccel_;

  // ===== 经验加速度限制 ===== //
  /// @brief 最大允许加速度（经验值）
  double empirical_acc_max_;
  
  /// @brief 最小允许加速度（经验值）
  double empirical_acc_min_;
  
  /// @brief 速度限制对应的预估加速度
  double estimated_acc_for_speed_limit_;

  // Be careful that complete_s_upper_bounds is the upper bounds with much higher
  // t-resolution, which would be crucial not only for our constraints generation
 // ===== 边界约束数据 ===== //
  /// @brief 完整s上边界约束集合（时间, 位置）
  std::vector<std::pair<double, double>> complete_s_hard_upper_bounds_;
  
  /// @brief 速度墙生成的s硬约束
  std::vector<std::pair<double, double>> speed_wall_complete_s_hard_upper_bounds_;
  
  /// @brief s软上边界约束集合
  std::vector<std::pair<double, double>> complete_s_soft_upper_bounds_;
  
  /// @brief s软下边界约束集合
  std::vector<std::pair<double, double>> complete_s_soft_lower_bounds_;
  
  /// @brief v软上边界约束集合
  std::vector<std::pair<double, double>> complete_v_soft_upper_bounds_;
  
  /// @brief ST速度限制区间数据
  std::vector<math::IntervalData<SpatialSpeedLimit>> st_speed_limit_;
  // ===== 安全距离参数表 ===== //
  /// @brief 基础安全距离系数
  double K_0_ = 1.5;
  
  /// @brief 动态调整后的安全距离系数
  double K_ = 1.5;
  /// @brief 历史障碍物的HWT（用于计算安全距离）
  std::vector<std::pair<string,double>> history_obj_hwt_;

  // ST-Parameters for lateral distance:
  /// @brief 横向距离比率表（0.0-1.0）
  std::vector<double> lateral_distance_ratio_table_ = {0.0, 0.5, 1.0};
  
  /// @brief 横向距离对K0的影响系数表
  std::vector<double> lateral_distance_factor_for_K_0_table_ = {1.0, 0.6, 0.2};
  
  /// @brief 横向距离对安全距离的影响系数表
  std::vector<double> lateral_distance_factor_for_safe_dist_table_ = {1.0, 0.6, 0.2};


 private:
  /// @brief 二次规划参数容器（按阶段存储）
  std::vector<std::unordered_map<std::string, double>> parameters_;
  /// @brief 默认优化参数模板
  const std::unordered_map<std::string, double> default_params_ = {{"SHardUpperBound", 1000.0},
                                                                   {"SHardLowerBound", 0.0},
                                                                   {"SSoftUpperBound", 999.0},
                                                                   {"SSoftLowerBound", 0.0},
                                                                   {"VHardUpperBound", 100.0},
                                                                   {"VHardLowerBound", 0.0},
                                                                   {"VSoftUpperBound", 90.0},
                                                                   {"AHardUpperBound", 5.0},
                                                                   {"AHardLowerBound", -5.0},
                                                                   {"JHardUpperBound", 10.0},
                                                                   {"JHardLowerBound", -10.0},
                                                                   {"ASoftUpperBound", 1.99},
                                                                   {"ASoftLowerBound", -5.0},
                                                                   {"SRef", 900.0},
                                                                   {"VRef", 100.0},
                                                                   {"K", 1.0},
                                                                   {"k", 0.0},
                                                                   {"SWeight", 10.0},
                                                                   {"VWeight", 100.0},
                                                                   {"AWeight", 1000.0},
                                                                   {"JWeight", 10000.0},
                                                                   {"SlackSUpperWeight", 1.0},
                                                                   {"SlackSLowerWeight", 2.0},
                                                                   {"SlackVUpperWeight", 3.0},
                                                                   {"SlackAUpperWeight", 4.0},
                                                                   {"SlackALowerWeight", 5.0},
                                                                   {"SlackDVWeight", 6.0},
                                                                   {"SafeDistForDVConstraint", 4.0},
                                                                   {"SUpperBoundForDVConstraint", 1000.0}};
};

}  // namespace gpal::pnc::planning