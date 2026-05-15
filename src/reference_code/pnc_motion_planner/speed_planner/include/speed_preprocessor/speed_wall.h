/**
 * @file speed_wall.h
 * @brief 多类型速度墙生成器
 * @details 本类负责生成速度规划所需的各种类型速度墙
 */
#pragma once

#include "speed_common/speed_common.h"
#include "config/speed_planner/speed_preprocessor.pb.h"
#include "config_manager/config_manager.h"
#include "decision_data/decision_result.h"
#include "local_view/local_view.h"
#include "reference_line_info/reference_line_info.h"

namespace gpal::pnc::planning {

class SpeedWallProcessor {
 public:
  /// @brief 默认构造函数
  SpeedWallProcessor() = default;
  /// @brief 析构函数
  ~SpeedWallProcessor() = default;
  
  bool init(const double& time_horizon);

  void addTflSpeedWall(const LocalView& local_view, ReferenceLineInfo* reference_line_info,
                                        const DecisionResult& decision_result, const bool& is_turn_around, std::vector<SpeedWall>* speed_walls);
  void addDestinationSpeedWall(const PathData& path_data, std::vector<SpeedWall>* speed_walls);
  void addDecisionStopSpeedWall(const LocalView& local_view, const DecisionResult& decision_result,
                                                 ReferenceLineInfo* reference_line_info,
                                                 std::vector<SpeedWall>* speed_walls);
  void addFreespaceSpeedWall(const PathData& path_data, const BehaviorState& behavior_state,
                                              std::vector<SpeedWall>* speed_walls);


  private:
  
  /// @brief 配置管理器指针
  ConfigManager* config_manager_ = nullptr;
  
  /// @brief 车辆物理参数（轴距/前悬等尺寸信息）
  VehicleParam vehicle_param_;
  
  /// @brief 速度预处理专用配置参数
  SpeedPreProcessorConfig speed_preprocessor_config_;

  // ===== 运行时状态 ===== //
  /// @brief 初始化完成标志（true表示配置加载完成）
  bool init_ = false;
  
  /// @brief 速度墙作用时间范围（单位：秒）
  double time_horizon_{5.0};
  
  /// @brief 掉头场景停车需求标志（来自转弯状态计算）
  bool require_stop_for_turn_around_ = false;

};

}  // namespace gpal::pnc::planning 