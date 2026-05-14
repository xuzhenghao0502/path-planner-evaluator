#pragma once

#include "config/spatiotemporal_planner/longitudinal_bound_parser_config.pb.h"
#include "manager/spatiotemporal_planner_data_manager.h"
#include "manager/spatiotemporal_planner_scenario_manager.h"
#include "path_planner/local_path_optimizer.h"

namespace gpal::pnc::planning {

class LocalPathProcess {
 public:
  LocalPathProcess() = default;
  ~LocalPathProcess() = default;
  using DataManager = SpatiotemporalPlannerDataManager;

  bool init(LongitudinalBoundParserProfile profile);
  void reset();

  double getLocalPathBlockDis(const Chassis* chassis, const VehicleState* vehicle_state,
                                               const Localization* loc, const Freespace* freespace,
                                               const DecisionResult& decision_result);
  const DiscretizedPath& getLocalPath() { return local_path_;}
  const PathData::BlockFSInfo& getBlockFSInfo() {return block_fs_info_;}
  const PathData::BlockFSInfo& getBlockObsInfo() {return block_obs_info_;}


 private:
  void calculateLocalPath(const Chassis* chassis, const VehicleState* vehicle_state);
  PathData::BlockFSInfo calculateBlockFsInfo(const Localization* loc, const Freespace* freespace);
  PathData::BlockFSInfo calculateBlockObsInfo(const DecisionResult& decision_result);
  math::Box2d getEgoBox(const PathPt& check_pt, double half_s_buffer, double half_l_buffer) const;

 private:
  LongitudinalBoundParserProfile profile_;
  ConfigManager* config_manager_ = nullptr;
  VehicleConfig vehicle_config_;
  LocalPathOptimizerConfig optimizer_config_;
  LocalPathOptimizer optimizer_;

  DiscretizedPath local_path_;
  PathData::BlockFSInfo block_fs_info_;
  PathData::BlockFSInfo  block_obs_info_;

 private:
  class PlannerAdapter : public PathPlannerBase {
   public:
    bool reset() override { return true; }
  };
  PlannerAdapter path_planner_;  ///< 路径规划适配器实例
};

}  // namespace gpal::pnc::planning