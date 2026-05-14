#pragma once

#include <fmt/core.h>
#include "base/log.h"
#include "config_manager/config_manager.h"
#include "config/vehicle_model/vehicle_config.pb.h"
#include "local_view/local_view.h"
#include "reference_line_info/reference_line_info.h"
#include "decision_data/decision_result.h"
#include "speed/speed_data.h"
#include "speed/speed_result.h"
#include "path/path_data.h"
#include "trajectory_data/trajectory.h"
#include "local_view/function_state.h"

namespace gpal::pnc::planning {
#define RLOG(level, ...) ERT_LOG_##level("[redundant] ", __VA_ARGS__)
// 内部数据快照结构体，用于安全地存储输入数据的副本
struct SnapShotData {
  std::shared_ptr<ReferenceLineInfo> target_ref_line_info_ptr = nullptr;
  std::shared_ptr<ReferenceLineInfo> drive_ref_line_info_ptr = nullptr;
  std::shared_ptr<Localization> localization_ptr = nullptr;
  std::shared_ptr<PathData> path_data = nullptr;
  std::shared_ptr<SpeedResult> speed_result_ptr = nullptr;
  std::shared_ptr<PathData> pre_path_data = nullptr;
  TrajectoryPt planning_start_point;
  std::shared_ptr<Console> console_ptr = nullptr;
  std::shared_ptr<Freespace> freespace_ptr = nullptr;
  std::shared_ptr<Chassis> chassis_ptr = nullptr;
  std::shared_ptr<DecisionResult> decision_result_ptr;
  std::shared_ptr<PathBoundary> path_boundary_ptr = nullptr;
  StageState stage_state;
  SpeedData prev_speed_data;
  int64_t time_stamp = 0;
  std::shared_ptr<gpal::pnc::VehicleState> vehicle_state_ptr = nullptr;
  std::string debug_info = "";  ///< 调试信息输出
  std::shared_ptr<LocalView> local_view_ptr; // 用于存储本地视图数据, 临时使用，未来必须删除
  std::pair<bool, double> remain_dis_info = std::make_pair<bool, double>(false, 1e4);  // 剩余距离信息，默认无效

  SnapShotData() = default;
};

struct RedundantPlannerResult {
  Trajectory trajectory; ///< 规划轨迹
  PathData path_data;   ///< 路径数据
  SpeedResult speed_result; ///< 速度规划结果
};
}  // namespace gpal::pnc::planning