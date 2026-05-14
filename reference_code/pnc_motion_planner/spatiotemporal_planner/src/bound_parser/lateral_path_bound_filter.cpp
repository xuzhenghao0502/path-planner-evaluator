/**
 * @file lateral_path_bound_filter.cpp
 * @brief 路径边界过滤器实现文件
 * @details
 * 该文件实现了路径边界过滤器的核心功能，包括路径边界的更新、过滤、重置等操作。路径边界过滤器用于在路径规划过程中对路径边界进行平滑处理，确保路径的连续性和可行性。
 */

#include "bound_parser/lateral_path_bound_filter.h"

namespace gpal::pnc::planning {

/**
 * @brief 重置路径边界过滤器
 * @details 该函数用于清除当前存储的路径边界信息，将内部边界数据清空，以便重新开始处理新的路径边界数据。
 *
 * @par 关键变量说明:
 * - bound_: 存储当前路径边界信息的容器
 *
 * @par 处理流程:
 * 1. 调用clear方法清空bound_容器
 *
 * @par 流程图:
 * @startuml
 * start
 * :调用clear方法清空bound_容器;
 * stop
 * @enduml
 *
 * @note 该函数应在需要重新处理路径边界数据时调用，确保之前的边界信息不会影响新的处理过程
 *
 * @warning 调用该函数后，所有之前存储的路径边界信息将被清除，无法恢复
 */
void LateralPathBoundFilter::reset() {
  bound_.clear();
}

/**
 * @brief 更新路径边界信息
 * @details 该函数用于更新路径边界信息，接收原始边界数据，并根据过滤器配置对边界进行处理和更新。
 *
 * @param[in] raw_bound 原始边界数据，包含路径点的s值、左侧边界和右侧边界
 *
 * @par 关键变量说明:
 * - bound_: 存储当前路径边界信息的容器
 * - config_: 路径边界过滤器配置，包含过滤器的各项参数
 *
 * @par 处理流程:
 * 1. 初始化临时边界容器
 * 2. 遍历原始边界数据
 * 3. 将每个路径点的边界信息添加到临时容器中
 * 4. 调用updateFilterBoundary函数对边界进行过滤处理
 * 5. 将处理后的边界信息更新到bound_容器中
 *
 * @par 流程图:
 * @startuml
 * start
 * :初始化临时边界容器;
 * :遍历原始边界数据;
 * :将每个路径点的边界信息添加到临时容器中;
 * :调用updateFilterBoundary函数对边界进行过滤处理;
 * :将处理后的边界信息更新到bound_容器中;
 * stop
 * @enduml
 *
 * @return void
 *
 * @note 该函数应在需要更新路径边界信息时调用，确保边界信息能够根据过滤器配置进行正确处理
 *
 * @warning 需确保传入的原始边界数据有效
 */
void LateralPathBoundFilter::update(const std::vector<std::tuple<double, double, double>>& raw_bound,
                                    const double filter_in, const double filter_out) {
  util::AbstractTable1d<double, double, double> bound;
  for (const auto& curr_bound : raw_bound) {
    bound.emplace_back(curr_bound);
    updateFilterBoundary(filter_in, filter_out, bound.back());
  }
  bound_ = std::move(bound);
}

/**
 * @brief 更新路径边界过滤信息
 * @details
 * 该函数用于根据过滤器的输入和输出参数，更新当前路径点的边界信息。通过插值获取前一帧的边界信息，并根据当前边界与前一帧边界的关系，使用不同的过滤系数进行平滑处理。
 *
 * @param[in] filter_in 过滤输入系数，用于平滑处理当前边界与前一帧边界的关系
 * @param[in] filter_out 过滤输出系数，用于平滑处理当前边界与前一帧边界的关系
 * @param[in,out] current_bound 当前路径点的边界信息，包含s值、左侧边界和右侧边界
 *
 * @par 关键变量说明:
 * - s: 当前路径点的s值
 * - lower: 当前路径点的左侧边界
 * - upper: 当前路径点的右侧边界
 * - prev_bound: 前一帧的边界信息，包含s值、左侧边界和右侧边界
 * - has_prev_bound: 是否存在前一帧的边界信息
 * - prev_s: 前一帧路径点的s值
 * - prev_lower: 前一帧路径点的左侧边界
 * - prev_upper: 前一帧路径点的右侧边界
 *
 * @par 处理流程:
 * 1. 获取当前路径点的s值、左侧边界和右侧边界
 * 2. 通过插值获取前一帧的边界信息
 * 3. 如果存在前一帧的边界信息，则根据当前边界与前一帧边界的关系，使用不同的过滤系数进行平滑处理
 * 4. 更新当前路径点的左侧边界和右侧边界
 *
 * @par 流程图:
 * @startuml
 * start
 * :获取当前路径点的s值、左侧边界和右侧边界;
 * :通过插值获取前一帧的边界信息;
 * if (存在前一帧的边界信息) then (yes)
 *   :根据当前边界与前一帧边界的关系，使用不同的过滤系数进行平滑处理;
 *   :更新当前路径点的左侧边界和右侧边界;
 * endif
 * stop
 * @enduml
 *
 * @return void
 *
 * @note 该函数应在更新路径边界信息时调用，确保边界信息能够根据过滤器配置进行平滑处理
 *
 * @warning 需确保传入的过滤系数和当前路径点的边界信息有效
 */
void LateralPathBoundFilter::updateFilterBoundary(const double& filter_in, const double& filter_out,
                                                  std::tuple<double, double, double>& current_bound) {
  auto& [s, lower, upper] = current_bound;
  const auto& [prev_bound, has_prev_bound] = bound_.interpolate(s);
  if (has_prev_bound) {
    const auto& [prev_s, prev_lower, prev_upper] = prev_bound;
    if (lower > prev_lower) {
      lower = filter_in * lower + (1.0 - filter_in) * prev_lower;
    } else {
      lower = filter_out * lower + (1.0 - filter_out) * prev_lower;
    }
    if (upper < prev_upper) {
      upper = filter_in * upper + (1.0 - filter_in) * prev_upper;
    } else {
      upper = filter_out * upper + (1.0 - filter_out) * prev_upper;
    }
  }
}
}  // namespace gpal::pnc::planning
