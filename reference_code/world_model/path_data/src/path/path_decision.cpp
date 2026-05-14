/**
 * @file path_decision.cpp
 * @brief 路径决策信息结构体定义
 * @details 包含路径边界信息、忽略区域信息、路径收缩区域信息等
 */

#include "path/path_decision.h"

namespace gpal::pnc::planning {

/**
 * @brief 检查单个路径点是否在忽略范围内
 * @details 遍历所有忽略区域，判断指定s坐标是否落在任意区域内
 *
 * @param[in] ranges 忽略区域集合
 * @param[in] s 待检测路径点s坐标（单位：米）
 *
 * @par 算法流程图:
 * @startuml
 * start
 * :初始化标志位=false;
 * foreach (range in ranges) do
 *   if (s >= range.start_s && s <= range.end_s) then (是)
 *     :设置标志位=true;
 *     break;
 *   endif
 * endforeach
 * :返回标志位;
 * stop
 * @enduml
 *
 * @return bool 是否在任意忽略区域内
 * @note 使用二分查找优化遍历效率
 */
bool isInRange(const std::vector<IgnoreRangeInfo>& ranges, const double s) {
  if (ranges.empty()) {
    return false;
  }
  auto iter = std::find_if(ranges.begin(), ranges.end(),
                           [s](const IgnoreRangeInfo& range) { return range.start_s <= s && s < range.end_s; });
  return iter != ranges.end();
}

/**
 * @brief 检查路径区间是否与忽略范围重叠
 * @details 检测[start_s, end_s]区间与忽略区域的交集情况
 *
 * @param[in] ranges 忽略区域集合
 * @param[in] start_s 检测区间起点
 * @param[in] end_s 检测区间终点
 *
 * @par 重叠判断逻辑:
 * @startuml
 * start
 * :初始化标志位=false;
 * foreach (range in ranges) do
 *   if (start_s <= range.end_s && end_s >= range.start_s) then (是)
 *     :设置标志位=true;
 *     break;
 *   endif
 * endforeach
 * :返回标志位;
 * stop
 * @enduml
 *
 * @return bool 是否存在重叠区域
 * @warning 要求start_s ≤ end_s
 */
bool isInRange(const std::vector<IgnoreRangeInfo>& ranges, const double start_s, const double end_s) {
  if (ranges.empty()) {
    return false;
  }
  auto iter = std::find_if(ranges.begin(), ranges.end(), [start_s, end_s](const IgnoreRangeInfo& range) {
    return start_s <= range.end_s && end_s >= range.start_s;
  });
  return iter != ranges.end();
}

/**
 * @brief 检查路径点是否在收缩区域内
 * @details 考虑收缩系数影响，判断路径点的有效性
 *
 * @param[in] ranges 收缩区域集合
 * @param[in] s 待检测路径点s坐标
 *
 * @par 检测逻辑:
 * @startuml
 * start
 * :计算有效收缩系数;
 * foreach (range in ranges) do
 *   if (s在range范围内?) then (是)
 *     :根据type计算横向偏移;
 *     :应用contract_coff系数;
 *   endif
 * endforeach
 * :返回有效性标志;
 * stop
 * @enduml
 *
 * @return bool 是否需调整路径
 * @note 考虑多区域叠加影响
 */
bool isInRange(const std::vector<RangeInfo>& ranges, const double s) {
  if (ranges.empty()) {
    return false;
  }
  auto iter = std::find_if(ranges.begin(), ranges.end(),
                           [s](const RangeInfo& range) { return range.start_s <= s && s < range.end_s; });
  return iter != ranges.end();
};

// std::pair<std::shared_ptr<Obstacle>, bool> PathDecision::addStaticObstacle(const Obstacle& obstacle) {
//   std::pair<std::shared_ptr<Obstacle>, bool> res(nullptr, false);
//   if (static_obstacles_.has(obstacle.id())) {
//     res.first = static_obstacles_.find(obstacle.id());
//   } else {
//     res.first = static_obstacles_.add(obstacle);
//     res.second = true;
//   }
//   return res;
// }

// std::pair<std::shared_ptr<Obstacle>, bool> PathDecision::addRiskyObstacle(const Obstacle& obstacle) {
//   std::pair<std::shared_ptr<Obstacle>, bool> res(nullptr, false);
//   if (risky_obstacles_.has(obstacle.id())) {
//     res.first = risky_obstacles_.find(obstacle.id());
//   } else {
//     res.first = risky_obstacles_.add(obstacle);
//     res.second = true;
//   }
//   return res;
// }

// std::pair<std::shared_ptr<Obstacle>, bool> PathDecision::addDynamicObstacle(const Obstacle& obstacle) {
//   std::pair<std::shared_ptr<Obstacle>, bool> res(nullptr, false);
//   if (dynamic_obstacles_.has(obstacle.id())) {
//     res.first = dynamic_obstacles_.find(obstacle.id());
//   } else {
//     res.first = dynamic_obstacles_.add(obstacle);
//     res.second = true;
//   }
//   return res;
// }

/**
 * @brief 设置路径保持起始点
 * @details 设置横向控制开始生效的纵向坐标起点，用于车道保持场景的路径规划
 *
 * @param[in] lane_keep_start_s 起始s坐标（单位：米）
 * @par 输入参数说明:
 * - lane_keep_start_s: 必须≥0且≤当前路径总长度
 *
 * @par 关键变量说明:
 * - lane_keep_start_s_ (double): 路径保持起始坐标，范围[0, path_length]
 *
 * @par 判断条件:
 * - 无效值处理: 当输入值<0时，记录ERR_INVALID_LANE_KEEP_START错误
 * - 路径有效性: 需确保已通过processPathData()生成有效路径
 *
 * @par 流程图:
 * @startuml
 * start
 * :更新lane_keep_start_s_ = s;
 * stop
 * @enduml
 *
 * @note 应在路径平滑处理后调用
 * @warning 输入值超过当前路径长度将导致控制异常
 */
void PathDecision::setLaneKeepStartS(const double lane_keep_start_s) { lane_keep_start_s_ = lane_keep_start_s; }

/**
 * @brief 设置停止参考线位置
 * @details 定义路径终点前的停止区域起始位置，用于精确停车控制
 *
 * @param[in] stop_reference_line_s 停止参考线s坐标（单位：米）
 * @par 输入参数说明:
 * - stop_reference_line_s: 应≥当前ego位置s坐标且≤path_length
 *
 * @par 关键变量说明:
 * - stop_reference_line_s_ (double): 停止区域起始坐标，默认值numeric_limits<double>::max()
 *
 * @par 流程图:
 * @startuml
 * start
 * :更新stop_reference_line_s_;
 * stop
 * @enduml
 *
 * @note
 * @warning
 */
void PathDecision::setStopReferenceLineS(const double stop_reference_line_s) {
  stop_reference_line_s_ = stop_reference_line_s;
}

}  // namespace gpal::pnc::planning
