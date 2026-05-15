#pragma once

#include "path/discretized_path.h"
#include "path/path_data.h"
#include "point/path_pt.h"
#include "local_view/local_view.h"
#include "decision_data/decision_result.h"
#include "openspace_path_planner/utils/openspace_common.h"
#include "config/path_planner/parking_path_planner_config.pb.h"

namespace gpal::pnc::planning {

/**
 * @brief 开放空间轨迹规划核心数据的基类。
 *
 * 该结构体作为所有开放空间（OpenSpace）轨迹规划中间数据结构的基类，方便进行统一的类型管理与扩展。
 * 派生类可包含如原始轨迹、优化轨迹、debug信息、车辆状态等内容。
 */
struct BaseOpenspaceCoreData {
  /**
   * @brief 虚析构函数，确保派生类在销毁时资源释放正确。
   */
  virtual ~BaseOpenspaceCoreData() = default;
};

}  // namespace gpal::pnc::planning
