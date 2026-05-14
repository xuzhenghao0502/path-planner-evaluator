#pragma once

#include "redundant_common.h"
#include "path_planner/real_time_path_planner.h"
#include "config/spatiotemporal_planner/redundant_planner_config.pb.h"

namespace gpal::pnc::planning {
class RedundantLateralPlanner : public RealTimePathPlanner {
 public:
  RedundantLateralPlanner() = default;
  ~RedundantLateralPlanner() = default;
  bool init();

  bool reset();

  /**
   * @brief 执行横向路径规划
   * @param snap_shot_data 输入数据快照
   * @return bool 是否成功
   *
   * @details 根据配置项 enable_unconstrained_lateral_mode 选择规划模式：
   *   - false（默认）: 调用 runOnce()，走完整约束求解链路（边界生成 + OCP 优化）
   *   - true         : 调用 runOnceUnconstrained()，跳过边界生成和 OCP 优化，
   *                    直接使用 generateUnconstrainedPath() 生成横向路径。
   *                    纵向规划所需的 path_boundary 由 fillDefaultPathBoundary() 补充。
   */
  bool processing(SnapShotData& snap_shot_data);

  /**
   * @brief 完整约束横向规划
   * @details 走完整的边界生成 → OCP 约束优化 → 无约束兜底 → 保护路径四阶段流程。
   *          在 enable_unconstrained_lateral_mode == false 时由 processing() 调用。
   */
  PathData::StatusType runOnce(SnapShotData& snap_shot_data);

 private:
  bool preprocess(SnapShotData& snap_shot_data);
  bool postprocess(SnapShotData& snap_shot_data, Trajectory& trajectory);

  /**
   * @brief 处理同步路径优化器（约束模式专用）
   * @details 直接调用同步的 optimizer_.proc()。
   *          主路径求解失败时调用无约束求解兜底；路径仍为空时调用保护模式兜底。
   */
  Status processPathOptimizerCustom(const DecisionResult& decision_result, const SpeedData& prev_speed_data,
                                    const int64_t& time_stamp, PathData* const path_data);

  /**
   * @brief 轻量化无约束横向规划
   * @details 跳过边界生成和 OCP 优化，直接调用 generateUnconstrainedPath()。
   *          流程：preProcess() → generateUnconstrainedPath() → generateProtectPath()（兜底）
   *                → setRemainDisInfo() → fillDefaultPathBoundary()
   *          在 enable_unconstrained_lateral_mode == true 时由 processing() 调用。
   */
  PathData::StatusType runOnceUnconstrained(SnapShotData& snap_shot_data);

  /**
   * @brief 为无约束模式补充默认 path_boundary
   * @details 无约束模式跳过了边界生成阶段，但纵向规划从
   *          target_ref_line_info_->path_boundary() 读取边界信息。
   *          此函数以无约束路径每个点为中心，生成一条等宽的宽松边界，
   *          并同步写入 snap_shot_data.path_boundary_ptr，防止纵向访问空边界崩溃。
   *          边界半宽由配置项 unconstrained_default_boundary_half_width 控制。
   */
  void fillDefaultPathBoundary(SnapShotData& snap_shot_data);

  std::string r_debug_info_ = "";
  RedundantPlannerConfig planning_config_;
};
}  // namespace gpal::pnc::planning