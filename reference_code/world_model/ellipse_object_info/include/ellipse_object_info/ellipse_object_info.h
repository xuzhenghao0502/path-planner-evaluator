#pragma once

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <memory>
#include <functional>
#include <array>

// 包含必要的决策定义
#include "decision_data/decision_common.h"

namespace gpal::pnc::planning {

/**
 * @class ObjectInfo
 * @brief 障碍物信息类，负责障碍物的几何建模（椭圆化）、距离计算及风险评估
 */
class ObjectInfo {
 public:
  using BufferCalculationFunction =
      std::function<std::pair<double, double>(const ObjectInfo&, double curr_v, double curr_a, int k)>;

  // --- 构造函数 ---
  ObjectInfo() = default;

  /**
   * @brief 基础构造函数：基于尺寸和Buffer构建椭圆
   */
  ObjectInfo(const double& x, const double& y, const double& theta, const double& local_s, const double& length,
             const double& width, const double& v, const double& acc, const double& long_buffer,
             const double& lateral_buffer, const double& ego_buffer);

  /**
   * @brief 时间外推构造函数：考虑预测时间段内的位移
   */
  ObjectInfo(const double& x, const double& y, const double& theta, const double& local_s, const double& length,
             const double& width, const double& v, const double& acc, const double& long_buffer,
             const double& lateral_buffer, const double& ego_buffer, const double& time_horizon);

  /**
   * @brief 协方差构造函数：基于感知不确定性（协方差矩阵）构建椭圆
   */
  ObjectInfo(const double& x, const double& y, const double& theta, const double& local_s, const double& length,
             const double& width, const double& v, const double& acc, const double& ego_buffer,
             const Eigen::Matrix2d& covariance, const double& ellipse_angle,
             const BufferCalculationFunction& buffer_func, const double& n_std = 3.0);

  // --- 核心公共接口 ---

  void updateDynamicBuffers(double curr_v, double curr_a, int k);

  void adjust(double ego_heading, double ang_a_min_factor = 0.6, double ang_a_max_factor = 1.0,
              double ang_b_min_factor = 1.0, double ang_b_max_factor = 1.6);

  double ComputeMahalanobisDistance(double point_x, double point_y) const;

  double ComputePerceptionMahalanobisDistance(double point_x, double point_y, double point_theta, double point_speed,
                                              double perception_t) const;

  double ComputeHeadingRisk(double point_x, double point_y, double point_theta) const;

  bool CalculateRiskySpeed(double point_x, double point_y, double point_theta, double point_speed,
                           double& risky_speed) const;

  std::vector<std::array<double, 2>> computeRectangleCorners() const;

  static double computeMinimalScaleFactor(const std::vector<std::array<double, 2>>& rect_corners, double a, double b,
                                          double ellipse_angle, const std::array<double, 2>& center);

 private:
  void extractEllipseFromCovariance(const Eigen::Matrix2d& cov_matrix, double n_std);
  double computeAngleFactor(double ego_heading, double obs_heading) const;

 public:
  // --- 成员变量 ---
  std::string id = "";
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  double l = 0.0;
  double v = 0.0;
  double acc = 0.0;
  double local_s = 0.0;
  double start_s = 0.0;
  double end_s = 0.0;
  double start_l = 0.0;
  double end_l = 0.0;
  double length = 0.0;
  double width = 0.0;

  // 椭圆参数
  double a = 0.0;
  double b = 0.0;
  double a_square = 0.0;
  double b_square = 0.0;
  double cos_theta = 0.0;
  double sin_theta = 0.0;

  // 缓冲区与基础参数
  double safe_distance = 3.0;
  double ellipse_long_buffer = 0.0;
  double ellipse_lat_buffer = 0.0;
  double ego_buffer = 0.0;
  double base_a = 0.0;
  double base_b = 0.0;
  double ellipse_angle = 0.0;

  // 决策状态
  gpal::pnc::planning::Decision::LongitudinalOdTag longitudinal_od_tag = gpal::pnc::planning::Decision::LongitudinalOdTag::INVALID;
  gpal::pnc::planning::Decision::LateralOdTag lateral_od_tag = gpal::pnc::planning::Decision::LateralOdTag::INVALID;
  gpal::pnc::planning::Decision::ObjectGameType object_game_type = gpal::pnc::planning::Decision::ObjectGameType::NON_GAME;

  Eigen::Matrix2d covariance_matrix;
  bool enable_optimized = true;

 private:
  std::shared_ptr<BufferCalculationFunction> buffer_calculator_{nullptr};
};

} // namespace gpal::pnc::planning
