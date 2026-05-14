/**
 * @file ocp_line_smoother.h
 * @brief 基于最优控制问题（OCP）的线平滑器实现文件
 * @details 该文件实现了OcpLineSmoother类，用于对输入的线进行平滑处理，生成平滑后的线。
 *          主要功能包括：初始化OCP模型、求解最优控制问题、获取平滑后的线等。
 */

#pragma once

#include <vector>

#include "Eigen/Dense"
#include "config/reference_line_smoother_config.pb.h"
#include "math/math_utils.h"
#include "ocp/ocp_model.h"
#include "reference_line/reference_line.h"

namespace gpal::pnc::planning {

/**
 * @class OcpLineSmoother
 * @brief 基于最优控制问题（OCP）的线平滑器
 */
class OcpLineSmoother {
 public:
  explicit OcpLineSmoother(const planning::ReferenceLineSmootherConfig& config);

  /**
   * @brief 析构函数
   */
  ~OcpLineSmoother() = default;

  bool smooth(const std::vector<math::Vec3d>& raw_line, std::vector<math::Vec3d>* const res_line);

  double getUnifySpaceHeading(const double heading_base, const double heading);

 private:
  void debugSolver(std::shared_ptr<OptimalControlProblem> model);

  gpal::proto::PathPoint to_path_point(const double x, const double y, const double z, const double s,
                                       const double theta, const double kappa, const double dkappa) const;

 private:
  planning::ReferenceLineSmootherConfig config_;                 ///< 参考线平滑器配置
  SolveStatus optimizer_status_ = SolveStatus::SOLVER_UNINIT;    ///< 优化器状态
  int NStates = 4;                                               ///< 状态变量数量
  int NControls = 2;                                             ///< 控制变量数量
  Eigen::VectorXd start_state = Eigen::VectorXd::Zero(NStates);  ///< 初始状态
};

}  // namespace gpal::pnc::planning
