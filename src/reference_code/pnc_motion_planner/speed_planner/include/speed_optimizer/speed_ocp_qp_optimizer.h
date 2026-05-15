/**
 * @file speed_ocp_qp_optimizer.h
 * @brief 速度最优控制问题二次规划求解器
 * @details 本类负责构建和求解速度规划的最优控制问题（OCP）
 */
#pragma once

#include "config_manager/config_manager.h"
#include "ocp/ocp_model.h"
#include "speed_optimizer/speed_model_param.h"

namespace gpal::pnc::planning {

class SpeedOCPQPOptimizer {
 public:
  /// @brief   优化状态枚举
  /// @details 0: 成功且有效，1: 成功但无效，2: 失败
  enum OCPQPSolveState { SUCCESS_AND_VALID = 0, SUCCESS_BUT_INVALID = 1, FAILED = 2 };
  /// @brief 构造函数
  SpeedOCPQPOptimizer(std::vector<double> time_grid, double time_resolution, double time_horizon)
      : time_grid_(time_grid), time_resolution_(time_resolution), time_horizon_(time_horizon) {};
  /// @brief 析构函数
  ~SpeedOCPQPOptimizer() = default;

  OCPQPSolveState runOptimizer(SpeedModelParam& speed_model_param,SpeedState& speed_init_state);
  std::pair<std::vector<OcpVariable>, std::vector<OcpVariable>> getOcpSolvedResult();
  std::pair<std::vector<OcpVariable>, std::vector<OcpVariable>> getRiskOcpSolvedResult();

 private:
  void init();
  SolveStatus solveSpeedOCPModel(SpeedModelParam& speed_model_param, bool is_relaxed);

 private:
  // ===== 时间域参数 ===== //
  /// @brief 时间网格划分（单位：秒）
  std::vector<double> time_grid_;
  
  /// @brief 时间分辨率（默认0.1秒）
  double time_resolution_{0.1};
  
  /// @brief 规划时间范围（默认8秒）
  double time_horizon_{8.0};

  // ===== 配置管理 ===== //
  /// @brief 配置管理器指针（延迟初始化）
  ConfigManager* config_manager_ = nullptr;
  
  /// @brief 车辆物理参数（轴距/质量等）
  VehicleParam vehicle_param_;
  
  /// @brief 二次规划优化器专用配置参数
  SpeedOcpQpOptimizerConfig speed_ocp_qp_optimizer_config_;

  // ===== 优化问题状态 ===== //
  /// @brief 初始速度状态（v0,a0）
  SpeedState x_0_;

  // ===== 优化问题实例 ===== //
  /// @brief 主速度优化问题实例
  std::shared_ptr<OptimalControlProblem> speed_ocp_{nullptr};
  
  /// @brief 风险场景备用优化问题实例
  std::shared_ptr<OptimalControlProblem> speed_risk_ocp_{nullptr};
};

}  // namespace gpal::pnc::planning