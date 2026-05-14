/**
 * @file speed_planner_obstacle.h
 * @brief 速度规划障碍物封装类
 * @details 本类整合感知与决策系统障碍物信息，为速度规划提供统一接口
 */
#pragma once
#include "decision_data/decision_result.h"
#include "obstacle/obstacle.h"
#include "speed/speed_data.h"

namespace gpal::pnc::planning {

class SpeedPlannerObstacle {
 public:
  struct FollowParams {
    /// @brief 跟车参数有效性标识（true表示参数已计算且有效）
    bool follow_params_valid_{false};

    /// @brief 纵向安全跟车距离（单位：米）
    double follow_distance_{0.0};

    /// @brief 横向安全跟车距离（单位：米）
    double lateral_follow_distance_{0.0};

    /// @brief 中心线横向跟车距离（单位：米）
    double center_lateral_follow_distance_{0.0};

    /// @brief 跟车速度基准值（单位：m/s）
    double follow_distance_speed_{0.0};

    /// @brief 横向跟车速度分量（单位：m/s）
    double lateral_follow_distance_speed_{0.0};

    /// @brief 实时动态跟车距离（单位：米）
    double distance_{0.0};

    /// @brief 滤波后的跟车速度（单位：m/s）
    double filtered_speed_{0.0};

    /// @brief 滤波后的加速度（单位：m/s²）
    double filtered_accel_{0.0};

    /// @brief 距离变化率（单位：m/s）
    double distance_change_rate_{0.0};

    /// @brief 跟车控制系数（用于距离-速度控制模型）
    double K_{0.0};
  };

  struct RiskFieldInfo {
    /// @brief 风险场时间戳（单位：秒）
    double t = 0.0;

    /// @brief 障碍物纵向速度分量（单位：m/s）
    double longitudinal_speed = 0.0;

    /// @brief 障碍物横向速度分量（单位：m/s）
    double lateral_speed = 0.0;

    /// @brief 路径最大曲率值（单位：1/m）
    double max_kappa = 0.0;

    /// @brief 最小膨胀系数（无单位）
    double min_alpha = 0.0;

    /// @brief 膨胀系数区间集合
    /// @details 存储时间区间及其对应的膨胀系数范围
    math::IntervalData<std::pair<double, double>> alpha;

    /// @brief 障碍物最小边界距离（单位：米）
    double min_box_distance = 0.0;

    /// @brief 最近入侵点的纵向距离（单位：米）
    double nearest_s = 0.0;

    /// @brief 障碍物边界距离区间集合
    /// @details 存储时间区间及其对应的边界距离范围
    math::IntervalData<std::pair<double, double>> box_distance;

    /// @brief 空间速度限制区间集合
    math::IntervalData<SpatialSpeedLimit> speed_limit_info;

    /**
     * @brief 清除所有风险场数据
     *
     * 重置内容：
     * - 所有标量值归零或设为极大值
     * - 清空所有区间数据集合
     */
    void clear() {
      t = 0.0;
      longitudinal_speed = 0.0;
      lateral_speed = 0.0;
      max_kappa = 0.0;
      alpha.clear();
      speed_limit_info.clear();
      box_distance.clear();
      min_box_distance = kPostiveInfinity;
      nearest_s = kPostiveInfinity;
      min_alpha = kPostiveInfinity;
    };

    /**
     * @brief 判断障碍物是否入侵下边界
     * @param buffer 安全缓冲距离（单位：米）
     * @param[out] lower_bound 输出实际入侵的下边界值
     * @return 是否发生边界入侵
     */
    bool s_invade_in(double buffer, double* lower_bound);

    /**
     * @brief 判断障碍物是否入侵上边界
     * @param buffer 安全缓冲距离（单位：米）
     * @param[out] upper_bound 输出实际入侵的上边界值
     * @return 是否发生边界入侵
     */
    bool s_invade_out(double buffer, double* upper_bound);
  };

 public:
  /**
   * @brief 默认构造函数
   */
  SpeedPlannerObstacle() = default;

  /**
   * @brief 基于决策障碍物的构造函数
   * @param od_decision 决策结果对象，包含所有障碍物信息
   * @note 只需要决策对象，简化接口，避免数据源冗余
   * @note 会立即深拷贝所有需要的数据，之后不依赖外部对象
   */
  explicit SpeedPlannerObstacle(const std::shared_ptr<Decision::DecisionObject>& od_decision);

  /// @brief 析构函数
  ~SpeedPlannerObstacle() = default;

  /// @name 基础属性访问接口
  /// @{
  /// @brief 获取障碍物ID
  const string& id() const { return id_; }

  /// @brief 获取障碍物类型
  const Decision::ObjectType type() const { return type_; }

  /// @brief 获取障碍物速度（静态障碍物返回0）
  const double speed() const { return is_static_ ? 0.0 : speed_; }

  /// @brief 获取加速度
  double acceleration() const { return acceleration_; }
  /// @}

  /// @brief 获取边界框几何数据
  const math::Box2d& perceptionBoundingBox() const { return bounding_box_; }

  /// @brief 判断是否为静态障碍物
  bool isStatic() const { return is_static_; }

  /// @name 预测轨迹相关接口
  /// @{
  /// @brief 获取预测轨迹点集合
  const std::vector<proto::TrajectoryPoint>& predictedTrajectory() const { return predicted_trajectory_; }

  /// @brief 判断是否存在有效预测轨迹
  bool hasPredictedTrajectory() const { return !predicted_trajectory_.empty(); }

  /// @brief 获取指定相对时间的轨迹点
  proto::TrajectoryPoint getPointAtTime(const double relative_time) const;

  /// @brief 获取指定轨迹点的边界框
  math::Box2d getBoundingBox(const proto::TrajectoryPoint& point) const;

  /// @brief 获取指定时间的边界框
  math::Box2d getBoundingBoxAtTime(const double relative_time) const;
  /// @}

  /// @name 决策特征接口
  /// @{
  /// @brief 获取纵向决策标签
  const LongitudinalOdTag& getLongitudinalOdTag() const { return longitudinal_od_tag_; }

  /// @brief 设置纵向决策标签
  void setLongitudinalOdTag(const LongitudinalOdTag& tag) { longitudinal_od_tag_ = tag; }

  /// @brief 获取博弈类型
  const ObjectGameType& getObjectGameType() const { return object_game_type_; }

  /// @brief 获取决策时间戳
  const double& getObstacleDecisionHwt() const { return obstacle_decision_hwt_; }
  /// @}

  /// @name 风险场管理接口
  /// @{
  /// @brief 获取风险场信息集合
  const std::vector<RiskFieldInfo>& getRiskFieldInfos() const { return risk_field_infos_; }

  /// @brief 设置风险场信息集合
  void setRiskFieldInfos(const std::vector<RiskFieldInfo>& risk_field_infos) {
    risk_field_infos_ = risk_field_infos;
    has_risk_field_ = true;
  }

  /// @brief 判断是否存在风险场数据
  bool hasRiskField() const { return has_risk_field_; }
  /// @}

  /// @name ST边界管理接口
  /// @{
  /// @brief 获取路径ST边界
  const STBoundary& pathStBoundary() const { return path_st_boundary_; }

  /// @brief 设置路径ST边界
  void setPathStBoundary(const STBoundary& boundary) { path_st_boundary_ = boundary; }

  /// @brief 设置路径ST边界类型
  void setPathStBoundaryType(const STBoundary::BoundaryType type) { path_st_boundary_.setBoundaryType(type); }

  /// @brief 设置ST边界计算状态
  void setSTBoundaryComputed(bool st_boundary_computed) { st_boundary_computed_ = st_boundary_computed; }

  /// @brief 判断ST边界是否已计算
  bool isSTBoundaryComputed() const { return st_boundary_computed_; }

  /// @brief 清除路径ST边界数据
  void erasePathStBoundary() { path_st_boundary_ = STBoundary(); }
  /// @}

  /// @name 风险ST边界接口
  /// @{
  /// @brief 获取风险ST边界
  const STBoundary& riskStBoundary() const { return risk_st_boundary_; }

  /// @brief 设置风险ST边界
  void setRiskStBoundary(const STBoundary& boundary) { risk_st_boundary_ = boundary; }

  /// @brief 启用/禁用风险ST边界
  void setRiskStBoundaryValid(bool enable_risk_st_boundary) { enable_risk_st_boundary_ = enable_risk_st_boundary; }

  /// @brief 获取风险ST边界启用状态
  const bool getRiskStBoundaryValid() const { return enable_risk_st_boundary_; }
  /// @}

  /// @name 跟车参数接口
  /// @{
  /// @brief 获取跟车参数
  const FollowParams& follow_params() const { return follow_params_; }

  /// @brief 设置跟车参数
  void setFollowParams(const FollowParams& params) { follow_params_ = params; }
  /// @}

  /// @brief 设置汇入障碍物标识
  const bool isMergeObstacle() const { return is_merge_obstacle_; }
  void setIsMergeObstacle(bool is_merge_obstacle) { is_merge_obstacle_ = is_merge_obstacle; }
  /// @}

 private:
  /// @brief 从DecisionObject深拷贝所有数据
  void deepCopyFromDecision(const std::shared_ptr<Decision::DecisionObject>& od_decision);

  /// @brief 设置障碍物预测轨迹
  void setObstaclePrediction(const std::vector<gpal::proto::TrajectoryPoint>& predicted_trajectory);

  /// @brief 设置障碍物默认预测轨迹
  void setObstacleDefaultPrediction();

 private:
  // 基础属性
  std::string id_ = "";                                        ///< 障碍物ID
  Decision::ObjectType type_ = Decision::ObjectType::UNKNOWN;  ///< 障碍物类型
  double speed_{0.0};                                          ///< 速度（m/s）
  double acceleration_{0.0};                                   ///< 加速度（m/s²）
  bool is_static_{false};                                      ///< 静态障碍物标识

  // 几何属性
  double x_{0.0};             ///< 障碍物位置X（米）
  double y_{0.0};             ///< 障碍物位置Y（米）
  double z_{0.0};             ///< 障碍物位置Z（米）
  double heading_{0.0};       ///< 障碍物航向（弧度）
  double length_{0.0};        ///< 长度（米）
  double width_{0.0};         ///< 宽度（米）
  math::Box2d bounding_box_;  ///< 边界框

  // 预测决策信息
  std::vector<proto::TrajectoryPoint> predicted_trajectory_;           ///< 预测轨迹点序列
  LongitudinalOdTag longitudinal_od_tag_{LongitudinalOdTag::INVALID};  ///< 纵向决策标签
  ObjectGameType object_game_type_{ObjectGameType::NON_GAME};          ///< 博弈交互类型
  double obstacle_decision_hwt_{-1.0};                                 ///< 决策时间戳（单位：秒）

  // 风险场数据
  bool has_risk_field_{false};              ///< 风险场计算标识
  vector<RiskFieldInfo> risk_field_infos_;  ///< 风险场信息集合

  // ST边界数据
  STBoundary path_st_boundary_;       ///< 路径相关的ST边界
  bool st_boundary_computed_{false};  ///< ST边界计算完成标识

  // 风险ST边界
  STBoundary risk_st_boundary_;          ///< 风险相关的ST边界
  bool enable_risk_st_boundary_{false};  ///< 风险边界启用标识

  // 跟车参数
  FollowParams follow_params_;  ///< 跟车控制参数集合

  bool is_merge_obstacle_{false};  ///< 是否为合并障碍物标识
};

/// @brief 障碍物集合类型定义（Key: 障碍物ID）
typedef std::unordered_map<std::string, std::shared_ptr<SpeedPlannerObstacle>> ObstacleSet;

}  // namespace gpal::pnc::planning
