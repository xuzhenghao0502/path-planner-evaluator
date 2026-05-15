/**
 * @file speed_result.cpp
 * @brief 速度规划结果容器
 * @details 本类集成速度规划全流程输出数据，作为模块间数据交换的统一接口
 */

#include "speed/speed_result.h"

namespace gpal::pnc::planning {

void SpeedResult::clear() {
  behavior_state_.clear();
  speed_limit_result_.clear();
  obstacle_set_.clear();
  nearest_invasion_obstacle_.clear();
  speed_walls_.clear();
  speed_data_.clear();
  stop_reason_.clear();
  st_graph_.clear();
  turn_flag_ = 0;
  destination_stop_flag_ = false;
  trajectory_result_.Clear();
  local_path_lower_boundary.clear();
  destination_stop_flag_ = false;
  obstacle_remain_distance_ = 10000.0;
}

}  // namespace gpal::pnc::planning
