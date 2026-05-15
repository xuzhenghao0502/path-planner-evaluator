#pragma once

#include "redundant_common.h"
#include "path_planner/local_path_optimizer.h"
#include "config/path_planner/local_path_optimizer_config.pb.h"
#include "config_manager/config_manager.h"
#include "path_planner/path_planner_base.h"

namespace gpal::pnc::planning {

/**
 * @class RedundantLocalPlanner
 * @brief 冗余链路中的 LocalPath 规划器
 */
class RedundantLocalPlanner {
 public:
  RedundantLocalPlanner() = default;
  ~RedundantLocalPlanner() = default;

  bool init();
  bool reset(); // 适配接口

  /**
   * @brief 执行 LocalPath 规划
   * @param snap_shot_data 数据快照
   */
  bool processing(SnapShotData& snap_shot_data);

 private:
  // 核心逻辑函数，适配 SnapShotData
  bool processPathOptimizer(SnapShotData& snap_shot_data);
  std::vector<math::LineSegment2d> mapBoundary(SnapShotData& snap_shot_data);
  void appendLineSegments(const std::vector<math::Vec2d>& points, 
                          std::vector<math::LineSegment2d>& segments,
                          double min_length, double max_heading_err);
  void generateBlockFSInfo(PathData::BlockFSInfo& block_fs_info, const SnapShotData& snap_shot_data);

 private:
  ConfigManager* config_manager_ = nullptr;
  LocalPathOptimizerConfig optimizer_config_;
  LocalPathOptimizer optimizer_;
  
  std::string debug_info_ = "";
  std::vector<PathData::DebugStatusType> debug_status_;

  // --- 适配 LocalPathPlanner 中的 PlannerAdapter ---
  // 用于调用基类的 collisionCheck 等通用方法
  class PlannerAdapter : public PathPlannerBase {
   public:
    bool reset() override { return true; }
  };
  PlannerAdapter path_planner_;
};

}  // namespace gpal::pnc::planning