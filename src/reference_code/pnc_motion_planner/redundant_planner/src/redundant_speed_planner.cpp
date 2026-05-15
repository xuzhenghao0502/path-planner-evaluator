#include "redundant_speed_planner.h"

namespace gpal::pnc::planning {
bool RedundantSpeedPlanner::init() {
  ocp_qp_speed_planner_.init();
  return true;
}
bool RedundantSpeedPlanner::reset() {
  // 重置纵向规划器状态
  // 可以添加必要的重置逻辑
  return true;
}
bool RedundantSpeedPlanner::processing(SnapShotData& snap_shot_data) {
  auto status = ocp_qp_speed_planner_.runOnce(*snap_shot_data.local_view_ptr, snap_shot_data.stage_state,
                                              snap_shot_data.target_ref_line_info_ptr,
                                              snap_shot_data.decision_result_ptr, snap_shot_data.path_data,
                                              snap_shot_data.time_stamp, snap_shot_data.speed_result_ptr);
  if (status != Status::OK()) {
    // 处理规划失败的情况
    RLOG(E, "[RedundantSpeedPlanner] Speed planning failed with status: ", status);
    return false;  // 返回失败状态
  }
  RLOG(I, "[RedundantSpeedPlanner] Speed planning completed successfully.");
  return true;  // 返回是否成功
}
}  // namespace gpal::pnc::planning