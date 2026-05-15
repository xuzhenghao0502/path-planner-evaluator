/**
 * @file ocp_reference_line_smoother.h
 * @brief 基于最优控制问题（OCP）的参考线平滑器实现文件
 * @details 该文件实现了OcpReferenceLineSmoother类，用于对输入的参考线进行平滑处理，生成平滑后的参考线。
 *          主要功能包括：初始化OCP模型、求解最优控制问题、获取平滑后的参考线等。
 */

#pragma once

#include <vector>

#include "Eigen/Dense"
#include "math/math_utils.h"
#include "ocp/ocp_model.h"
#include "reference_line/reference_line.h"
#include "reference_line/reference_line_smoother.h"
#include "reference_line/reference_point.h"

namespace gpal::pnc::planning {

/**
 * @class OcpReferenceLineSmoother
 * @brief 基于最优控制问题（OCP）的参考线平滑器
 */
class OcpReferenceLineSmoother : public ReferenceLineSmoother {
 public:
  explicit OcpReferenceLineSmoother(const planning::ReferenceLineSmootherConfig& config);

  /**
   * @brief 析构函数
   */
  virtual ~OcpReferenceLineSmoother() = default;

  bool smooth(const ReferenceLine& raw_reference_line, ReferenceLine* const smoothed_reference_line,
              bool is_async = false) override;

  double getUnifySpaceHeading(const double heading_base, const double heading);

 private:
  void debugSolver(std::shared_ptr<OptimalControlProblem> model);

  std::string ReferenceLineEvaluate(const ReferenceLine& smoothed_reference_line,
                                    const std::vector<Eigen::Vector2d>& raw_point2d,
                                    const std::vector<DrivingDirection>& raw_direction, double duration);

  gpal::proto::PathPoint to_path_point(const double x, const double y, const double z, const double s,
                                       const double theta, const double kappa, const double dkappa) const;

  std::vector<AnchorPoint> anchor_points_;                       ///< 锚点集合
  SolveStatus optimizer_status_ = SolveStatus::SOLVER_UNINIT;    ///< 优化器状态
  int NStates = 4;                                               ///< 状态变量数量
  int NControls = 2;                                             ///< 控制变量数量
  Eigen::VectorXd start_state = Eigen::VectorXd::Zero(NStates);  ///< 初始状态

 private:
  double kappa_upper_limit = 0.2;   ///< 曲率上限
  double offset_upper_limit = 2.0;  ///< 偏移量上限
  double width_lower_limit = 1;     ///< 车道中线距离车道边线距离下限
};

}  // namespace gpal::pnc::planning
