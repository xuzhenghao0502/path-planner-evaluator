#pragma once

#include "redundant_common.h"
#include "speed_planner/ocp_qp_speed_planner.h"

namespace gpal::pnc::planning {
class RedundantSpeedPlanner {
 public:
  RedundantSpeedPlanner() = default;
  ~RedundantSpeedPlanner() = default;

  bool init();
  bool reset();
  /**
   * @brief 执行纵向路径规划
   * @param snap_shot_data 输入数据快照
   * @param trajectory 输出轨迹
   * @return bool 是否成功
   */
  bool processing(SnapShotData& snap_shot_data);

 private:
  OcpQpSpeedPlanner ocp_qp_speed_planner_;
};
}  // namespace gpal::pnc::planning
