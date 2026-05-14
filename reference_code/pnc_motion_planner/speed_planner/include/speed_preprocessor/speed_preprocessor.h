/**
 * @file speed_preprocessor.h
 * @brief 多场景速度预处理中枢
 * @details 本类负责统筹速度规划前的多源数据处理,包括障碍物处理、速度限制集成、速度墙生成等。
 */
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "base/status.h"
#include "config_manager/config_manager.h"
#include "decision_data/decision_result.h"
#include "local_view/local_view.h"
#include "speed/st_graph.h"
#include "speed_common/speed_common.h"
#include "speed_preprocessor/lateral_path.h"
#include "speed_preprocessor/speed_limit.h"
#include "speed_preprocessor/speed_wall.h"
#include "speed_preprocessor/obstacle_set.h"

namespace gpal::pnc::planning {

class SpeedPreprocessor {
 public:
   /**
   * @brief 构造函数（初始化时间域参数）
   * @param[in] time_grid 时间网格划分（单位：秒）
   * @param[in] time_resolution 时间分辨率（单位：秒）
   * @param[in] time_horizon 规划时间范围（单位：秒）
   */
  SpeedPreprocessor(std::vector<double> time_grid, double time_resolution, double time_horizon)
      : time_grid_(time_grid), time_resolution_(time_resolution), time_horizon_(time_horizon) {};
  /// @brief 虚析构函数
  virtual ~SpeedPreprocessor() = default;
  void init();
  void NoaProcess(const LocalView& local_view, ReferenceLineInfo* reference_line_info, const PathData& path_data,
               const DecisionResult& decision_result,const SpeedState& init_speed_state, std::shared_ptr<SpeedResult> speed_result);
  void ParkProcess(const LocalView& local_view, const PathData& path_data, const DecisionResult& decision_result,const SpeedState& init_speed_state,
                   std::shared_ptr<SpeedResult> speed_result);
  void AccProcess(const LocalView& local_view, const PathData& path_data, const DecisionResult& decision_result,const SpeedState& init_speed_state,
                  std::shared_ptr<SpeedResult> speed_result);
    /**
   * @brief 获取横向路径处理结果
   * @return LateralPath 包含离散路径组和局部路径组的结果容器
   */
  LateralPath getLateralPathResult() { return lateral_path_; }

 private:

  // temp
  void caculateTurnFlag(const LocalView& local_view, ReferenceLineInfo* reference_line_info);
// ===== 转弯状态控制 ===== //
  /// @brief 调头状态标志（true表示正在执行调头）
  bool is_turn_around_ = false;
  
  /// @brief 最近一次转弯状态（0-直行 1-普通转弯 2-调头）
  int last_turn_state_ = 0;
  
  /// @brief 调头场景停车需求标志
  bool require_stop_for_turn_around_ = false;

  // ===== 时间域参数 ===== //
  /// @brief 时间网格划分（用于ST图生成）
  std::vector<double> time_grid_;
  
  /// @brief 时间分辨率（默认0.1秒）
  double time_resolution_{0.1};
  
  /// @brief 规划时间范围（默认8秒）
  double time_horizon_{8.0};

  // ===== 功能处理器 ===== //
  /// @brief 横向路径处理器
  LateralPathProcessor lateral_path_processor_;
  
  /// @brief 速度限制处理器
  SpeedLimitProcessor speed_limit_processor_;
  
  /// @brief 速度墙处理器
  SpeedWallProcessor speed_wall_processor_;
  
  /// @brief 障碍物集合处理器
  ObstacleSetProcessor obstacle_set_processor_;

  // ===== 配置管理 ===== //
  /// @brief 配置管理器
  ConfigManager* config_manager_ = nullptr;
  
  /// @brief 车辆物理参数配置
  VehicleParam vehicle_param_;
  
  /// @brief 速度规划全局配置
  SpeedPlannerConfig speed_planner_config_;
  
  /// @brief 速度预处理专用配置
  SpeedPreProcessorConfig speed_preprocessor_config_;

  // ===== 处理结果缓存 ===== //
  /// @brief 横向路径处理结果
  LateralPath lateral_path_;
};

}  // namespace gpal::pnc::planning
