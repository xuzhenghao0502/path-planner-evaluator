#pragma once

#include "redundant_lateral_planner.h"
#include "redundant_local_planner.h"
#include "redundant_speed_planner.h"
#include "util/task_handler.h"
#include "path_planner/path_planner_base.h"
#include "config/spatiotemporal_planner/redundant_planner_config.pb.h"

namespace gpal::pnc::planning {
class RedundantPlanner {
 public:
  RedundantPlanner() = default;
  ~RedundantPlanner() = default;

  /**
   * @brief 初始化时空规划器
   * @return bool 初始化是否成功
   */
  bool init();

  /**
   * @brief 重置时空规划器
   * @return bool 重置是否成功
   */
  bool reset();

  /**
   * @brief 异步启动备用规划器的计算。
   * 此函数会立即返回，不会阻塞。
   * @return 如果成功启动任务，返回 true；如果已有任务在运行，则返回 false。
   */
  bool startRunningAsync(const std::shared_ptr<ReferenceLineInfo>& target_ref_line_info,
                         const std::shared_ptr<ReferenceLineInfo>& drive_ref_line_info,
                         const LocalView* const local_view, const DecisionResult* const decision_result,
                         const StageState& stage_state, const int64_t& time_stamp,
                         const std::pair<bool, double>& remain_dis_info);

  /**
   * @brief 检查计算是否完成，如果完成则返回轨迹结果。
   * @return 如果计算已成功完成，返回包含轨迹的 std::optional；
   * 如果任务正在运行或失败，返回 std::nullopt。
   */
  std::optional<RedundantPlannerResult> getResultIfReady();

  /**
   * @brief (阻塞) 等待指定时间并获取结果
   * @param timeout 等待的超时时间
   * @return 如果在超时时间内计算成功，返回轨迹；否则返回 std::nullopt
   */
  std::optional<RedundantPlannerResult> getResultWithWait(const std::chrono::milliseconds& timeout);

  std::string getDebugInfo() const;

 private:
  // 内部实际执行规划的函数
  bool runInternal();

  bool lateralProcess();
  bool speedProcess();
  bool toResult(const SpeedResult& speed_result, const PathData& path_data, const Localization* localization,
                RedundantPlannerResult& result);
  TrajectoryPt parsePlanningStartPoint(const SnapShotData& snap_shot_data);
  void geneStartPath(const TrajectoryPt& nearest_pt, const DiscretizedPath& prev_path, const double& curr_v,
                     const double& lateral_diff, DiscretizedPath* starting_path, TrajectoryPt* starting_pt);
  std::shared_ptr<DecisionResult> deepCopyDecisionResult(const DecisionResult* decision_result);
  std::shared_ptr<LocalView> deepCopyLocalView(const LocalView* local_view);

  // --- 内部状态管理成员 ---
  ConfigManager* config_manager_ = nullptr;  // 配参管理器
  RedundantPlannerConfig planning_config_;
  TaskHandler<1> task_handler_;
  std::future<bool> future_;

  // 为当前计算结果和上一次成功结果分别设置互斥锁
  mutable std::mutex current_result_mutex_;
  std::mutex last_successful_result_mutex_;

  RedundantPlannerResult current_result_;          // 存储当前异步任务的计算数据
  RedundantPlannerResult last_successful_result_;  // 存储上一次成功的结果
  std::string current_debug_info_ = "";            // debug信息

  SnapShotData internal_data_;               // 存储输入的快照
  RedundantLateralPlanner lateral_planner_;  // 横向规划器实例
  RedundantLocalPlanner local_path_planner_;  // Local Path规划器实例
  RedundantSpeedPlanner speed_planner_;      // 纵向规划器实例

  // 标记 last_successful_result_ 是否已被用作一次 fallback
  std::atomic<bool> last_result_was_used_{false};
  // 标记当前正在运行的任务是否已经超时了一帧
  std::atomic<bool> task_is_overrunning_{false};
};

}  // namespace gpal::pnc::planning