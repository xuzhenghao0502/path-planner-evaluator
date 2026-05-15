/**
 * @file local_router.cpp
 * @brief 本地路径路由器实现文件
 * @details
 * 该文件实现了本地路径路由器的核心功能，包括路径边界的重建、速度限制的重建、方向信息的重建等。本地路径路由器用于管理和处理车辆在局部路径上的各种信息，如边界、速度限制、方向等。
 */

#include "navigation_data/local_router.h"

#include <algorithm>
#include <limits>
#include <unordered_set>

namespace gpal::pnc::planning {

/**
 * @brief 重建边界交叉范围
 * @details 该函数用于重新构建边界交叉范围，确保边界交叉范围按照起始位置排序，并合并相邻的相同类型边界
 *
 * @param[in] boundary_cross_ranges 边界交叉范围列表，包含多个边界交叉范围信息
 * @par 输入参数说明:
 * - boundary_cross_ranges: 必须包含有效的边界交叉范围信息，不能为空
 *
 * @par 关键变量说明:
 * - new_ranges (std::vector<BoundaryCrossRange>): 存储重建后的边界交叉范围
 * - range (BoundaryCrossRange): 当前处理的边界交叉范围
 * - boundRange (std::tuple<bool, float, float>): 边界范围信息，包含是否有效、起始位置和结束位置
 *
 * @par 处理流程:
 * 1. 检查输入参数有效性
 * 2. 按照边界起始位置排序
 * 3. 遍历所有边界交叉范围
 * 4. 合并相邻的相同类型边界
 * 5. 检查边界交叉范围的完整性
 * 6. 更新边界交叉范围列表
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查输入参数有效性;
 * :按照边界起始位置排序;
 * :初始化new_ranges;
 * while (遍历boundary_cross_ranges?) is (是)
 *   :获取当前range;
 *   if (range有效?) then (是)
 *     if (new_ranges为空?) then (是)
 *       :添加range到new_ranges;
 *     else (否)
 *       if (range类型与new_ranges.back()相同?) then (是)
 *         :合并range到new_ranges.back();
 *       else (否)
 *         :添加range到new_ranges;
 *       endif
 *     endif
 *   endif
 * endwhile
 * :检查边界交叉范围完整性;
 * :更新boundary_cross_ranges;
 * stop
 * @enduml
 *
 * @note 该函数主要用于优化边界交叉范围，减少重复的边界信息
 *
 * @warning 需确保输入的boundary_cross_ranges包含有效的边界信息
 */
bool LocalRoute::RebuildBoundaryCrossRanges(std::vector<BoundaryCrossRange>* boundary_cross_ranges) {
  if (boundary_cross_ranges == nullptr) {
    std::cerr << "[LocalRoute::RebuildBoundaryCrossRanges]: boundary_cross_ranges is nullptr";
    return false;
  }
  if (boundary_cross_ranges->empty()) {
    std::cerr << "[LocalRoute::RebuildBoundaryCrossRanges]: boundary_cross_ranges is empty";
    return false;
  }

  auto sorter = [](const BoundaryCrossRange& lhs, const BoundaryCrossRange& rhs) {
    return std::get<1>(lhs.boundRange()) < std::get<1>(rhs.boundRange());
  };
  std::sort(boundary_cross_ranges->begin(), boundary_cross_ranges->end(), sorter);

  std::vector<BoundaryCrossRange> new_ranges;
  for (const auto& range : *boundary_cross_ranges) {
    if (std::get<0>(range.boundRange())) {
      if (new_ranges.empty()) {
        new_ranges.emplace_back(range);
      } else {
        if (range.boundType() == new_ranges.back().boundType()) {
          std::get<2>(*new_ranges.back().mutableBoundRange()) = std::get<2>(range.boundRange());
          new_ranges.back().mutableBoundRangePoints()->second = range.boundRangePoints().second;
          new_ranges.back().mutableSegmentsId()->insert(new_ranges.back().mutableSegmentsId()->end(),
                                                        range.segmentsId().begin(), range.segmentsId().end());
          new_ranges.back().mutableSegmentsRange()->insert(new_ranges.back().mutableSegmentsRange()->end(),
                                                           range.segmentsRange().begin(), range.segmentsRange().end());
          new_ranges.back().mutableSegmentsRangePoints()->insert(new_ranges.back().mutableSegmentsRangePoints()->end(),
                                                                 range.segmentsRangePoints().begin(),
                                                                 range.segmentsRangePoints().end());
          new_ranges.back().mutableNeighborSegmentsId()->insert(new_ranges.back().mutableNeighborSegmentsId()->end(),
                                                                range.neighborSegmentsId().begin(),
                                                                range.neighborSegmentsId().end());
        } else {
          new_ranges.emplace_back(range);
        }
      }
    }
  }

  // for (auto& range : new_ranges) {
  //   if (range.segmentsId().size() != range.neighborSegmentsId().size() ||
  //       range.segmentsId().size() != range.segmentsRange().size() ||
  //       range.segmentsId().size() != range.segmentsRangePoints().size()) {
  //     std::cerr << "[LocalRoute::RebuildBoundaryCrossRanges]: Error! range.segmentsId().size() "
  //               << range.segmentsId().size() << " range.neighborSegmentsId().size() "
  //               << range.neighborSegmentsId().size() << " range.segmentsRange().size() " << range.segmentsRange().size()
  //               << " range.segmentsRangePoints().size() " << range.segmentsRangePoints().size() << ", not equal!!!";
  //     return false;
  //   }
  // }

  *boundary_cross_ranges = std::move(new_ranges);
  return true;
}

/**
 * @brief 重建速度限制信息
 * @details 该函数用于重新构建速度限制信息，确保速度限制按照起始位置排序，并合并相邻的相同速度限制
 *
 * @par 输入参数说明:
 * - speed_limits_: 必须包含有效的速度限制信息，不能为空
 *
 * @par 关键变量说明:
 * - new_speed_limits (std::vector<SpeedLimit>): 存储重建后的速度限制信息
 * - speed_limit (SpeedLimit): 当前处理的速度限制信息
 *
 * @par 处理流程:
 * 1. 检查速度限制信息是否为空
 * 2. 按照速度限制的起始位置排序
 * 3. 遍历所有速度限制信息
 * 4. 合并相邻的相同速度限制
 * 5. 更新速度限制信息
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查speed_limits_是否为空;
 * :按照start_s排序speed_limits_;
 * :初始化new_speed_limits;
 * while (遍历speed_limits_?) is (是)
 *   :获取当前speed_limit;
 *   if (new_speed_limits为空?) then (是)
 *     :添加speed_limit到new_speed_limits;
 *   else (否)
 *     if (speed_limit与new_speed_limits.back()相同?) then (是)
 *       :合并speed_limit到new_speed_limits.back();
 *     else (否)
 *       :添加speed_limit到new_speed_limits;
 *     endif
 *   endif
 * endwhile
 * :更新speed_limits_;
 * stop
 * @enduml
 *
 * @note 该函数主要用于优化速度限制信息，减少重复的速度限制
 *
 * @warning 需确保speed_limits_包含有效的速度限制信息
 */
bool LocalRoute::RebuildSpeedLimits() {
  if (speed_limits_.empty()) {
    std::cerr << "[LocalRoute::RebuildSpeedLimits]: speed_limits_ is empty";
    return false;
  }

  auto sorter = [](const SpeedLimit& lhs, const SpeedLimit& rhs) { return lhs.start_s < rhs.start_s; };
  std::sort(speed_limits_.begin(), speed_limits_.end(), sorter);

  std::vector<SpeedLimit> new_speed_limits;
  for (const auto& speed_limit : speed_limits_) {
    if (new_speed_limits.empty()) {
      new_speed_limits.emplace_back(speed_limit);
    } else {
      if (std::fabs(speed_limit.max_speed_limit - new_speed_limits.back().max_speed_limit) > 1e-2 ||
          std::fabs(speed_limit.min_speed_limit - new_speed_limits.back().min_speed_limit) > 1e-2) {
        new_speed_limits.emplace_back(speed_limit);
      } else {
        new_speed_limits.back().end_s = speed_limit.end_s;
        new_speed_limits.back().end_point = speed_limit.end_point;
      }
    }
  }
  speed_limits_ = std::move(new_speed_limits);
  return true;
}

/**
 * @brief 重建方向信息
 * @details 该函数用于重新构建方向信息，确保方向信息按照起始位置排序，并合并相邻的相同方向
 *
 * @par 输入参数说明:
 * - directions_: 必须包含有效的方向信息，不能为空
 *
 * @par 关键变量说明:
 * - new_directions (std::vector<SegmentDirection>): 存储重建后的方向信息
 * - direction (SegmentDirection): 当前处理的方向信息
 *
 * @par 处理流程:
 * 1. 检查方向信息是否为空
 * 2. 按照方向的起始位置排序
 * 3. 遍历所有方向信息
 * 4. 合并相邻的相同方向
 * 5. 更新方向信息
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查directions_是否为空;
 * :按照start_s排序directions_;
 * :初始化new_directions;
 * while (遍历directions_?) is (是)
 *   :获取当前direction;
 *   if (new_directions为空?) then (是)
 *     :添加direction到new_directions;
 *   else (否)
 *     if (direction与new_directions.back()相同?) then (是)
 *       :合并direction到new_directions.back();
 *     else (否)
 *       :添加direction到new_directions;
 *     endif
 *   endif
 * endwhile
 * :更新directions_;
 * stop
 * @enduml
 *
 * @note 该函数主要用于优化方向信息，减少重复的方向信息
 *
 * @warning 需确保directions_包含有效的方向信息
 */
bool LocalRoute::RebuildDirections() {
  if (directions_.empty()) {
    std::cerr << "[LocalRoute::RebuildDirections]: directions_ is empty";
    return false;
  }

  auto sorter = [](const SegmentDirection& lhs, const SegmentDirection& rhs) { return lhs.start_s < rhs.start_s; };
  std::sort(directions_.begin(), directions_.end(), sorter);

  std::vector<SegmentDirection> new_directions;
  for (const auto& direction : directions_) {
    if (new_directions.empty()) {
      new_directions.emplace_back(direction);
    } else {
      if (direction.direction != new_directions.back().direction) {
        new_directions.emplace_back(direction);
      } else {
        new_directions.back().end_s = direction.end_s;
        new_directions.back().end_point = direction.end_point;
      }
    }
  }
  directions_ = std::move(new_directions);
  return true;
}

/**
 * @brief 重建来源信息
 * @details 该函数用于重新构建来源信息，确保来源信息按照起始位置排序，并合并相邻的相同来源信息
 *
 * @par 输入参数说明:
 * - source_infos_: 必须包含有效的来源信息，不能为空
 *
 * @par 关键变量说明:
 * - new_sources (std::vector<std::tuple<LineSourceType, float, float>>): 存储重建后的来源信息
 * - source (std::tuple<LineSourceType, float, float>): 当前处理的来源信息
 *
 * @par 处理流程:
 * 1. 检查来源信息是否为空
 * 2. 按照来源信息的起始位置排序
 * 3. 遍历所有来源信息
 * 4. 合并相邻的相同来源信息
 * 5. 更新来源信息
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查source_infos_是否为空;
 * :按照起始位置排序source_infos_;
 * :初始化new_sources;
 * while (遍历source_infos_?) is (是)
 *   :获取当前source;
 *   if (new_sources为空?) then (是)
 *     :添加source到new_sources;
 *   else (否)
 *     if (source类型与new_sources.back()相同?) then (是)
 *       :合并source到new_sources.back();
 *     else (否)
 *       :添加source到new_sources;
 *     endif
 *   endif
 * endwhile
 * :更新source_infos_;
 * stop
 * @enduml
 *
 * @note 该函数主要用于优化来源信息，减少重复的来源信息
 *
 * @warning 需确保source_infos_包含有效的来源信息
 */
bool LocalRoute::RebuildSourceInfos() {
  if (source_infos_.empty()) {
    std::cerr << "[LocalRoute::RebuildSourceInfos]: source_infos_ is empty";
    return false;
  }

  auto sorter = [](const std::tuple<LineSourceType, float, float>& lhs,
                   const std::tuple<LineSourceType, float, float>& rhs) { return std::get<1>(lhs) < std::get<1>(rhs); };
  std::sort(source_infos_.begin(), source_infos_.end(), sorter);

  std::vector<std::tuple<LineSourceType, float, float>> new_sources;
  for (const auto& source : source_infos_) {
    if (new_sources.empty()) {
      new_sources.emplace_back(source);
    } else {
      if (std::get<0>(source) != std::get<0>(new_sources.back())) {
        new_sources.emplace_back(std::get<0>(source), std::get<2>(new_sources.back()), std::get<2>(source));
      } else {
        std::get<2>(new_sources.back()) = std::get<2>(source);
      }
    }
  }
  source_infos_ = std::move(new_sources);
  return true;
}

/**
 * @brief 根据s范围获取相邻LocalRoute
 * @details 该函数用于在给定的s范围内获取相邻的LocalRoute信息
 *
 * @param[in] boundary_cross_ranges 边界交叉范围列表，包含多个边界交叉范围信息
 * @param[in] start_s 起始s值，表示查询范围的起点
 * @param[in] end_s 结束s值，表示查询范围的终点
 *
 * @par 输入参数说明:
 * - boundary_cross_ranges: 必须包含有效的边界交叉范围信息，不能为空
 * - start_s: 查询范围的起始s值，单位米
 * - end_s: 查询范围的结束s值，单位米
 *
 * @par 关键变量说明:
 * - neighbor_local_routes_unordered_map (std::unordered_map<int, NeighborLocalRoute>): 存储相邻LocalRoute信息的无序映射
 * - adc_start_s (float): 调整后的起始s值，确保在有效范围内
 * - adc_end_s (float): 调整后的结束s值，确保在有效范围内
 * - start_index (int): 起始索引，表示在boundary_cross_ranges中的起始位置
 * - end_index (int): 结束索引，表示在boundary_cross_ranges中的结束位置
 * - element (NeighborLocalRoute::Element): 相邻LocalRoute的元素信息
 *
 * @par 处理流程:
 * 1. 检查输入参数有效性
 * 2. 调整start_s和end_s，确保在有效范围内
 * 3. 确定start_index和end_index
 * 4. 遍历boundary_cross_ranges，获取相邻LocalRoute信息
 * 5. 合并相邻的相同类型边界
 * 6. 返回相邻LocalRoute信息
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查boundary_cross_ranges是否为空;
 * :调整start_s和end_s;
 * :确定start_index和end_index;
 * while (遍历boundary_cross_ranges?) is (是)
 *   :获取当前range;
 *   if (range有效?) then (是)
 *     :获取相邻LocalRoute信息;
 *     :合并相邻的相同类型边界;
 *   endif
 * endwhile
 * :返回相邻LocalRoute信息;
 * stop
 * @enduml
 *
 * @note 该函数主要用于获取在给定s范围内的相邻LocalRoute信息
 *
 * @warning 需确保输入的boundary_cross_ranges包含有效的边界信息
 */
std::unordered_map<int, NeighborLocalRoute> LocalRoute::getNeighborLocalRouteBySRange(
    const std::vector<BoundaryCrossRange>& boundary_cross_ranges, const float& start_s, const float& end_s) const {
  std::unordered_map<int, NeighborLocalRoute> neighbor_local_routes_unordered_map;

  if (boundary_cross_ranges.empty()) {
    std::cerr << "[LocalRoute::getNeighborLocalRouteBySRange]: boundary_cross_ranges is empty!";
    return neighbor_local_routes_unordered_map;
  }

  float adc_start_s = start_s;
  if (adc_start_s < 0) {
    adc_start_s = 0.0f;
  }
  if (adc_start_s > std::get<2>(boundary_cross_ranges.back().boundRange())) {
    adc_start_s = std::get<2>(boundary_cross_ranges.back().boundRange());
  }
  float adc_end_s = end_s;
  if (adc_end_s < 0) {
    adc_end_s = 0.0f;
  }
  if (adc_end_s > std::get<2>(boundary_cross_ranges.back().boundRange())) {
    adc_end_s = std::get<2>(boundary_cross_ranges.back().boundRange());
  }
  if (adc_start_s > adc_end_s) {
    std::swap(adc_start_s, adc_end_s);
  }

  int start_index = 0;
  int end_index = static_cast<int>(boundary_cross_ranges.size()) - 1;
  for (int i = 0; i < boundary_cross_ranges.size(); i++) {
    if (std::get<0>(boundary_cross_ranges.at(i).boundRange())) {
      float range_start_s = std::get<1>(boundary_cross_ranges.at(i).boundRange());
      float range_end_s = std::get<2>(boundary_cross_ranges.at(i).boundRange());
      if (range_start_s - (1e-2) < adc_start_s && adc_start_s < range_end_s + (1e-2)) {
        start_index = i;
      }
      if (range_start_s - (1e-2) < adc_end_s && adc_end_s < range_end_s + (1e-2)) {
        end_index = i;
      }
    }
  }
  if (start_index > end_index) {
    std::swap(start_index, end_index);
  }
  for (int i = start_index; i <= end_index; i++) {
    // if (boundary_cross_ranges.at(i).neighborSegmentsId().size() != boundary_cross_ranges.at(i).segmentsRange().size() ||
    //     boundary_cross_ranges.at(i).neighborSegmentsId().size() !=
    //         boundary_cross_ranges.at(i).segmentsRangePoints().size() ||
    //     boundary_cross_ranges.at(i).neighborSegmentsId().size() != boundary_cross_ranges.at(i).segmentsId().size()) {
    //   std::cerr << "[LocalRoute::RebuildBoundaryCrossRanges]: Error!  local route id " << id_
    //             << ", boundary_cross_ranges: neighborSegmentsId().size() "
    //             << boundary_cross_ranges.at(i).neighborSegmentsId().size() << ", segmentsRange().size() "
    //             << boundary_cross_ranges.at(i).segmentsRange().size() << ", segmentsRangePoints().size() "
    //             << boundary_cross_ranges.at(i).segmentsRangePoints().size() << ", segmentsId().size() "
    //             << boundary_cross_ranges.at(i).segmentsId().size() << ", not equal!!!";
    //   neighbor_local_routes_unordered_map.clear();
    //   return neighbor_local_routes_unordered_map;
    // }
    for (int j = 0; j < boundary_cross_ranges.at(i).neighborSegmentsId().size(); j++) {
      const auto neighbor = boundary_cross_ranges.at(i).neighborSegmentsId().at(j);
      NeighborLocalRoute::Element element;
      element.bound_type = boundary_cross_ranges.at(i).boundType();
      element.range = boundary_cross_ranges.at(i).segmentsRange().at(j);
      element.points = boundary_cross_ranges.at(i).segmentsRangePoints().at(j);
      element.relation_segment_id_pair.emplace_back();
      element.relation_segment_id_pair.back().first = boundary_cross_ranges.at(i).segmentsId().at(j);
      element.relation_segment_id_pair.back().second = neighbor.first;
      for (const auto& local_route_id : neighbor.second) {
        neighbor_local_routes_unordered_map[local_route_id].id = std::to_string(local_route_id);
        neighbor_local_routes_unordered_map[local_route_id].elements.emplace_back(element);
      }
    }
  }

  std::unordered_map<int, NeighborLocalRoute> res;
  for (const auto& local_route : neighbor_local_routes_unordered_map) {
    res[local_route.first].id = local_route.second.id;
    res[local_route.first].elements.clear();
    for (const auto& element : local_route.second.elements) {
      if (std::get<0>(element.range)) {
        if (res[local_route.first].elements.empty()) {
          res[local_route.first].elements.emplace_back(element);
        } else {
          if (element.bound_type == res[local_route.first].elements.back().bound_type) {
            std::get<2>(res[local_route.first].elements.back().range) = std::get<2>(element.range);
            res[local_route.first].elements.back().points.second = element.points.second;
            res[local_route.first].elements.back().relation_segment_id_pair.insert(
                res[local_route.first].elements.back().relation_segment_id_pair.end(),
                element.relation_segment_id_pair.begin(), element.relation_segment_id_pair.end());

          } else {
            res[local_route.first].elements.emplace_back(element);
          }
        }
      }
    }
  }
  neighbor_local_routes_unordered_map = std::move(res);

  // // print info
  // ERT_PLOG_I << "[LocalRoute::getNeighborLocalRouteBySRange]: neighbor_local_routes_unordered_map.size() = " <<
  // neighbor_local_routes_unordered_map.size()
  //           << "  check range <" << adc_start_s << ", " << adc_end_s << ">" ;
  // for(const auto& local_route : neighbor_local_routes_unordered_map) {
  //   ERT_PLOG_I << "  [LocalRoute::getNeighborLocalRouteBySRange]: key = " << local_route.first
  //             << "  neighbor local route id = " << local_route.second.id
  //             << "  element size = " << local_route.second.elements.size() ;
  //   for(const auto& element : local_route.second.elements) {
  //     ERT_PLOG_I << "    [LocalRoute::getNeighborLocalRouteBySRange]: bound_type = " <<
  //     static_cast<int>(element.bound_type)
  //               << "  range = <" << std::get<0>(element.range) << ", " << std::get<1>(element.range)<< ", " <<
  //               std::get<2>(element.range)
  //               << ">  relation_segment_id_pair.size() = " << element.relation_segment_id_pair.size()
  //               ;
  //   }
  // }

  return neighbor_local_routes_unordered_map;
}

/**
 * @brief 根据s范围和前后距离获取相邻LocalRoute
 * @details 该函数用于在给定的s值附近，根据前后距离获取相邻的LocalRoute信息
 *
 * @param[in] boundary_cross_ranges 边界交叉范围列表，包含多个边界交叉范围信息
 * @param[in] s 当前s值，表示查询的中心点
 * @param[in] forward_dis 前向距离，表示从s值向前查询的范围
 * @param[in] backward_dis 后向距离，表示从s值向后查询的范围
 *
 * @par 输入参数说明:
 * - boundary_cross_ranges: 必须包含有效的边界交叉范围信息，不能为空
 * - s: 查询的中心s值，单位米
 * - forward_dis: 前向查询距离，单位米
 * - backward_dis: 后向查询距离，单位米
 *
 * @par 关键变量说明:
 * - start_s (float): 查询范围的起始s值，计算为s - backward_dis
 * - end_s (float): 查询范围的结束s值，计算为s + forward_dis
 *
 * @par 处理流程:
 * 1. 计算查询范围的起始s值和结束s值
 * 2. 调用getNeighborLocalRouteBySRange函数获取相邻LocalRoute信息
 *
 * @par 流程图:
 * @startuml
 * start
 * :计算start_s = s - backward_dis;
 * :计算end_s = s + forward_dis;
 * :调用getNeighborLocalRouteBySRange(boundary_cross_ranges, start_s, end_s);
 * stop
 * @enduml
 *
 * @note 该函数主要用于在给定s值附近，根据前后距离获取相邻LocalRoute信息
 *
 * @warning 需确保输入的boundary_cross_ranges包含有效的边界信息
 */
std::unordered_map<int, NeighborLocalRoute> LocalRoute::getNeighborLocalRouteBySRange(
    const std::vector<BoundaryCrossRange>& boundary_cross_ranges, const float& s, const float& forward_dis,
    const float& backward_dis) const {
  return getNeighborLocalRouteBySRange(boundary_cross_ranges, s - backward_dis, s + forward_dis);
}

/**
 * @brief 根据s范围获取左侧相邻LocalRoute
 * @details 该函数用于在给定的s范围内获取左侧相邻的LocalRoute信息
 *
 * @param[in] start_s 起始s值，表示查询范围的起点
 * @param[in] end_s 结束s值，表示查询范围的终点
 *
 * @par 输入参数说明:
 * - start_s: 查询范围的起始s值，单位米
 * - end_s: 查询范围的结束s值，单位米
 *
 * @par 关键变量说明:
 * - left_boundary_cross_ranges_: 左侧边界交叉范围列表，包含多个左侧边界交叉范围信息
 *
 * @par 处理流程:
 * 1. 调用getNeighborLocalRouteBySRange函数，传入左侧边界交叉范围列表和s范围
 * 2. 返回左侧相邻LocalRoute信息
 *
 * @par 流程图:
 * @startuml
 * start
 * :调用getNeighborLocalRouteBySRange(left_boundary_cross_ranges_, start_s, end_s);
 * :返回左侧相邻LocalRoute信息;
 * stop
 * @enduml
 *
 * @note 该函数主要用于获取在给定s范围内的左侧相邻LocalRoute信息
 *
 * @warning 需确保输入的s范围在有效范围内
 */
std::unordered_map<int, NeighborLocalRoute> LocalRoute::getLeftNeighborLocalRouteBySRange(const float& start_s,
                                                                                          const float& end_s) const {
  return getNeighborLocalRouteBySRange(left_boundary_cross_ranges_, start_s, end_s);
}

/**
 * @brief 根据s范围和前后距离获取左侧相邻LocalRoute
 * @details 该函数用于在给定的s值附近，根据前后距离获取左侧相邻的LocalRoute信息
 *
 * @param[in] s 当前s值，表示查询的中心点
 * @param[in] forward_dis 前向距离，表示从s值向前查询的范围
 * @param[in] backward_dis 后向距离，表示从s值向后查询的范围
 *
 * @par 输入参数说明:
 * - s: 查询的中心s值，单位米
 * - forward_dis: 前向查询距离，单位米
 * - backward_dis: 后向查询距离，单位米
 *
 * @par 关键变量说明:
 * - left_boundary_cross_ranges_: 左侧边界交叉范围列表，包含多个左侧边界交叉范围信息
 * - start_s (float): 查询范围的起始s值，计算为s - backward_dis
 * - end_s (float): 查询范围的结束s值，计算为s + forward_dis
 *
 * @par 处理流程:
 * 1. 计算查询范围的起始s值和结束s值
 * 2. 调用getNeighborLocalRouteBySRange函数，传入左侧边界交叉范围列表和s范围
 * 3. 返回左侧相邻LocalRoute信息
 *
 * @par 流程图:
 * @startuml
 * start
 * :计算start_s = s - backward_dis;
 * :计算end_s = s + forward_dis;
 * :调用getNeighborLocalRouteBySRange(left_boundary_cross_ranges_, start_s, end_s);
 * :返回左侧相邻LocalRoute信息;
 * stop
 * @enduml
 *
 * @note 该函数主要用于在给定s值附近，根据前后距离获取左侧相邻LocalRoute信息
 *
 * @warning 需确保输入的s范围在有效范围内
 */
std::unordered_map<int, NeighborLocalRoute> LocalRoute::getLeftNeighborLocalRouteBySRange(
    const float& s, const float& forward_dis, const float& backward_dis) const {
  return getNeighborLocalRouteBySRange(left_boundary_cross_ranges_, s, forward_dis, backward_dis);
}

/**
 * @brief 根据s范围和前后距离获取右侧相邻LocalRoute
 * @details 该函数用于在给定的s值附近，根据前后距离获取右侧相邻的LocalRoute信息
 *
 * @param[in] s 当前s值，表示查询的中心点
 * @param[in] forward_dis 前向距离，表示从s值向前查询的范围
 * @param[in] backward_dis 后向距离，表示从s值向后查询的范围
 *
 * @par 输入参数说明:
 * - s: 查询的中心s值，单位米
 * - forward_dis: 前向查询距离，单位米
 * - backward_dis: 后向查询距离，单位米
 *
 * @par 关键变量说明:
 * - right_boundary_cross_ranges_: 右侧边界交叉范围列表，包含多个右侧边界交叉范围信息
 * - start_s (float): 查询范围的起始s值，计算为s - backward_dis
 * - end_s (float): 查询范围的结束s值，计算为s + forward_dis
 *
 * @par 处理流程:
 * 1. 计算查询范围的起始s值和结束s值
 * 2. 调用getNeighborLocalRouteBySRange函数，传入右侧边界交叉范围列表和s范围
 * 3. 返回右侧相邻LocalRoute信息
 *
 * @par 流程图:
 * @startuml
 * start
 * :计算start_s = s - backward_dis;
 * :计算end_s = s + forward_dis;
 * :调用getNeighborLocalRouteBySRange(right_boundary_c cross_ranges_, start_s, end_s);
 * :返回右侧相邻LocalRoute信息;
 * stop
 * @enduml
 *
 * @note 该函数主要用于在给定s值附近，根据前后距离获取右侧相邻LocalRoute信息
 *
 * @warning 需确保输入的s范围在有效范围内
 */
std::unordered_map<int, NeighborLocalRoute> LocalRoute::getRightNeighborLocalRouteBySRange(
    const float& s, const float& forward_dis, const float& backward_dis) const {
  return getNeighborLocalRouteBySRange(right_boundary_cross_ranges_, s, forward_dis, backward_dis);
}

/**
 * @brief 根据s范围获取右侧相邻LocalRoute
 * @details 该函数用于在给定的s范围内获取右侧相邻的LocalRoute信息
 *
 * @param[in] start_s 起始s值，表示查询范围的起点
 * @param[in] end_s 结束s值，表示查询范围的终点
 *
 * @par 输入参数说明:
 * - start_s: 查询范围的起始s值，单位米
 * - end_s: 查询范围的结束s值，单位米
 *
 * @par 关键变量说明:
 * - right_boundary_cross_ranges_: 右侧边界交叉范围列表，包含多个右侧边界交叉范围信息
 *
 * @par 处理流程:
 * 1. 调用getNeighborLocalRouteBySRange函数，传入右侧边界交叉范围列表和s范围
 * 2. 返回右侧相邻LocalRoute信息
 *
 * @par 流程图:
 * @startuml
 * start
 * :调用getNeighborLocalRouteBySRange(right_boundary_cross_ranges_, start_s, end_s);
 * :返回右侧相邻LocalRoute信息;
 * stop
 * @enduml
 *
 * @note 该函数主要用于获取在给定s范围内的右侧相邻LocalRoute信息
 *
 * @warning 需确保输入的s范围在有效范围内
 */
std::unordered_map<int, NeighborLocalRoute> LocalRoute::getRightNeighborLocalRouteBySRange(const float& start_s,
                                                                                           const float& end_s) const {
  return getNeighborLocalRouteBySRange(right_boundary_cross_ranges_, start_s, end_s);
}

/**
 * @brief 根据s范围获取合并/分叉范围信息
 * @details 该函数用于在给定的s范围内获取合并/分叉范围信息
 *
 * @param[in] start_s 起始s值，表示查询范围的起点
 * @param[in] end_s 结束s值，表示查询范围的终点
 *
 * @par 输入参数说明:
 * - start_s: 查询范围的起始s值，单位米
 * - end_s: 查询范围的结束s值，单位米
 *
 * @par 关键变量说明:
 * - merge_fork_ranges_: 存储合并/分叉范围信息的列表
 * - res (std::vector<MergeForkRange>): 存储查询结果的列表
 *
 * @par 处理流程:
 * 1. 遍历所有合并/分叉范围信息
 * 2. 判断当前范围是否在查询范围内
 * 3. 如果在查询范围内，则添加到结果列表中
 *
 * @par 流程图:
 * @startuml
 * start
 * :初始化结果列表res;
 * while (遍历merge_fork_ranges_?) is (是)
 *   :获取当前range;
 *   if (range.s() < start_s?) then (是)
 *     :跳过当前range;
 *   else if (range.s() > end_s?) then (是)
 *     :结束遍历;
 *   else (否)
 *     :将range添加到res中;
 *   endif
 * endwhile
 * :返回结果列表res;
 * stop
 * @enduml
 *
 * @note 该函数主要用于获取在给定s范围内的合并/分叉范围信息
 *
 * @warning 需确保输入的s范围在有效范围内
 */
std::vector<MergeForkRange> LocalRoute::getMergeForkRangesFromSRange(const float& start_s, const float& end_s) {
  std::vector<MergeForkRange> res;
  for (const auto& range : merge_fork_ranges_) {
    if (range.s() < start_s) {
      continue;
    } else if (range.s() > end_s) {
      break;
    }
    res.emplace_back(range);
  }
  return res;
}

/**
 * @brief 根据s范围和前后距离获取合并/分叉范围信息
 * @details 该函数用于在给定的s值附近，根据前后距离获取合并/分叉范围信息
 *
 * @param[in] s 当前s值，表示查询的中心点
 * @param[in] forward_dis 前向距离，表示从s值向前查询的范围
 * @param[in] backward_dis 后向距离，表示从s值向后查询的范围
 *
 * @par 输入参数说明:
 * - s: 查询的中心s值，单位米
 * - forward_dis: 前向查询距离，单位米
 * - backward_dis: 后向查询距离，单位米
 *
 * @par 关键变量说明:
 * - merge_fork_ranges_: 存储合并/分叉范围信息的列表
 * - start_s (float): 查询范围的起始s值，计算为s - backward_dis
 * - end_s (float): 查询范围的结束s值，计算为s + forward_dis
 *
 * @par 处理流程:
 * 1. 计算查询范围的起始s值和结束s值
 * 2. 调用getMergeForkRangesFromSRange函数，传入起始s值和结束s值
 * 3. 返回合并/分叉范围信息
 *
 * @par 流程图:
 * @startuml
 * start
 * :计算start_s = s - backward_dis;
 * :计算end_s = s + forward_dis;
 * :调用getMergeForkRangesFromSRange(start_s, end_s);
 * :返回合并/分叉范围信息;
 * stop
 * @enduml
 *
 * @note 该函数主要用于在给定s值附近，根据前后距离获取合并/分叉范围信息
 *
 * @warning 需确保输入的s范围在有效范围内
 */
std::vector<MergeForkRange> LocalRoute::getMergeForkRangesFromSRange(const float& s, const float& forward_dis,
                                                                     const float& backward_dis) {
  return getMergeForkRangesFromSRange(s - backward_dis, s + forward_dis);
}

/**
 * @brief 根据s范围获取合并范围信息
 * @details 该函数用于在给定的s范围内获取合并范围信息
 *
 * @param[in] start_s 起始s值，表示查询范围的起点
 * @param[in] end_s 结束s值，表示查询范围的终点
 *
 * @par 输入参数说明:
 * - start_s: 查询范围的起始s值，单位米
 * - end_s: 查询范围的结束s值，单位米
 *
 * @par 关键变量说明:
 * - merge_fork_ranges_: 存储合并/分叉范围信息的列表
 * - res (std::vector<MergeForkRange>): 存储查询结果的列表
 *
 * @par 处理流程:
 * 1. 遍历所有合并/分叉范围信息
 * 2. 判断当前范围是否为合并类型且在查询范围内
 * 3. 如果满足条件，则添加到结果列表中
 *
 * @par 流程图:
 * @startuml
 * start
 * :初始化结果列表res;
 * while (遍历merge_fork_ranges_?) is (是)
 *   :获取当前range;
 *   if (range类型为合并?) then (是)
 *     if (range.s() < start_s?) then (是)
 *       :跳过当前range;
 *     else if (range.s() > end_s?) then (是)
 *       :结束遍历;
 *     else (否)
 *       :将range添加到res中;
 *     endif
 *   endif
 * endwhile
 * :返回结果列表res;
 * stop
 * @enduml
 *
 * @note 该函数主要用于获取在给定s范围内的合并范围信息
 *
 * @warning 需确保输入的s范围在有效范围内
 */
std::vector<MergeForkRange> LocalRoute::getMergeRangesFromSRange(const float& start_s, const float& end_s) {
  std::vector<MergeForkRange> res;
  for (const auto& range : merge_fork_ranges_) {
    if (range.type() == MergeForkRange::MergeForkType::MERGE) {
      if (range.s() < start_s) {
        continue;
      } else if (range.s() > end_s) {
        break;
      }
      res.emplace_back(range);
    }
  }
  return res;
}

/**
 * @brief 根据s范围和前后距离获取合并范围信息
 * @details 该函数用于在给定的s值附近，根据前后距离获取合并范围信息
 *
 * @param[in] s 当前s值，表示查询的中心点
 * @param[in] forward_dis 前向距离，表示从s值向前查询的范围
 * @param[in] backward_dis 后向距离，表示从s值向后查询的范围
 *
 * @par 输入参数说明:
 * - s: 查询的中心s值，单位米
 * - forward_dis: 前向查询距离，单位米
 * - backward_dis: 后向查询距离，单位米
 *
 * @par 关键变量说明:
 * - merge_fork_ranges_: 存储合并/分叉范围信息的列表
 * - start_s (float): 查询范围的起始s值，计算为s - backward_dis
 * - end_s (float): 查询范围的结束s值，计算为s + forward_dis
 *
 * @par 处理流程:
 * 1. 计算查询范围的起始s值和结束s值
 * 2. 调用getMergeRangesFromSRange函数，传入起始s值和结束s值
 * 3. 返回合并范围信息
 *
 * @par 流程图:
 * @startuml
 * start
 * :计算start_s = s - backward_dis;
 * :计算end_s = s + forward_dis;
 * :调用getMergeRangesFromSRange(start_s, end_s);
 * :返回合并范围信息;
 * stop
 * @enduml
 *
 * @note 该函数主要用于在给定s值附近，根据前后距离获取合并范围信息
 *
 * @warning 需确保输入的s范围在有效范围内
 */
std::vector<MergeForkRange> LocalRoute::getMergeRangesFromSRange(const float& s, const float& forward_dis,
                                                                 const float& backward_dis) {
  return getMergeRangesFromSRange(s - backward_dis, s + forward_dis);
}

/**
 * @brief 根据s范围获取分叉范围信息
 * @details 该函数用于在给定的s范围内获取分叉范围信息
 *
 * @param[in] start_s 起始s值，表示查询范围的起点
 * @param[in] end_s 结束s值，表示查询范围的终点
 *
 * @par 输入参数说明:
 * - start_s: 查询范围的起始s值，单位米
 * - end_s: 查询范围的结束s值，单位米
 *
 * @par 关键变量说明:
 * - merge_fork_ranges_: 存储合并/分叉范围信息的列表
 * - res (std::vector<MergeForkRange>): 存储查询结果的列表
 *
 * @par 处理流程:
 * 1. 遍历所有合并/分叉范围信息
 * 2. 判断当前范围是否为分叉类型且在查询范围内
 * 3. 如果满足条件，则添加到结果列表中
 *
 * @par 流程图:
 * @startuml
 * start
 * :初始化结果列表res;
 * while (遍历merge_fork_ranges_?) is (是)
 *   :获取当前range;
 *   if (range类型为分叉?) then (是)
 *     if (range.s() < start_s?) then (是)
 *       :跳过当前range;
 *     else if (range.s() > end_s?) then (是)
 *       :结束遍历;
 *     else (否)
 *       :将range添加到res中;
 *     endif
 *   endif
 * endwhile
 * :返回结果列表res;
 * stop
 * @enduml
 *
 * @note 该函数主要用于获取在给定s范围内的分叉范围信息
 *
 * @warning 需确保输入的s范围在有效范围内
 */
std::vector<MergeForkRange> LocalRoute::getForkRangesFromSRange(const float& start_s, const float& end_s) {
  std::vector<MergeForkRange> res;
  for (const auto& range : merge_fork_ranges_) {
    if (range.type() == MergeForkRange::MergeForkType::FORK) {
      if (range.s() < start_s) {
        continue;
      } else if (range.s() > end_s) {
        break;
      }
      res.emplace_back(range);
    }
  }
  return res;
}

/**
 * @brief 根据s范围和前后距离获取分叉范围信息
 * @details 该函数用于在给定的s值附近，根据前后距离获取分叉范围信息
 *
 * @param[in] s 当前s值，表示查询的中心点
 * @param[in] forward_dis 前向距离，表示从s值向前查询的范围
 * @param[in] backward_dis 后向距离，表示从s值向后查询的范围
 *
 * @par 输入参数说明:
 * - s: 查询的中心s值，单位米
 * - forward_dis: 前向查询距离，单位米
 * - backward_dis: 后向查询距离，单位米
 *
 * @par 关键变量说明:
 * - merge_fork_ranges_: 存储合并/分叉范围信息的列表
 * - start_s (float): 查询范围的起始s值，计算为s - backward_dis
 * - end_s (float): 查询范围的结束s值，计算为s + forward_dis
 *
 * @par 处理流程:
 * 1. 计算查询范围的起始s值和结束s值
 * 2. 调用getForkRangesFromSRange函数，传入起始s值和结束s值
 * 3. 返回分叉范围信息
 *
 * @par 流程图:
 * @startuml
 * start
 * :计算start_s = s - backward_dis;
 * :计算end_s = s + forward_dis;
 * :调用getForkRangesFromSRange(start_s, end_s);
 * :返回分叉范围信息;
 * stop
 * @enduml
 *
 * @note 该函数主要用于在给定s值附近，根据前后距离获取分叉范围信息
 *
 * @warning 需确保输入的s范围在有效范围内
 */
std::vector<MergeForkRange> LocalRoute::getForkRangesFromSRange(const float& s, const float& forward_dis,
                                                                const float& backward_dis) {
  return getForkRangesFromSRange(s - backward_dis, s + forward_dis);
}

/**
 * @brief 根据s范围获取停止线信息
 * @details 该函数用于在给定的s范围内获取停止线信息
 *
 * @param[in] start_s 起始s值，表示查询范围的起点
 * @param[in] end_s 结束s值，表示查询范围的终点
 *
 * @par 输入参数说明:
 * - start_s: 查询范围的起始s值，单位米
 * - end_s: 查询范围的结束s值，单位米
 *
 * @par 关键变量说明:
 * - stop_lines_: 存储停止线信息的列表
 * - res (std::vector<StopLine>): 存储查询结果的列表
 *
 * @par 处理流程:
 * 1. 遍历所有停止线信息
 * 2. 判断当前停止线的s值是否在查询范围内
 * 3. 如果在查询范围内，则添加到结果列表中
 *
 * @par 流程图:
 * @startuml
 * start
 * :初始化结果列表res;
 * while (遍历stop_lines_?) is (是)
 *   :获取当前stop_line;
 *   if (stop_line.s < start_s?) then (是)
 *     :跳过当前stop_line;
 *   else if (stop_line.s > end_s?) then (是)
 *     :结束遍历;
 *   else (否)
 *     :将stop_line添加到res中;
 *   endif
 * endwhile
 * :返回结果列表res;
 * stop
 * @enduml
 *
 * @note 该函数主要用于获取在给定s范围内的停止线信息
 *
 * @warning 需确保输入的s范围在有效范围内
 */
std::vector<StopLine> LocalRoute::getStopLinesFromSRange(const float& start_s, const float& end_s) {
  std::vector<StopLine> res;
  for (const auto& stop_line : stop_lines_) {
    if (stop_line.s < start_s) {
      continue;
    } else if (stop_line.s > end_s) {
      break;
    }
    res.emplace_back(stop_line);
  }
  return res;
}

/**
 * @brief 根据s范围和前后距离获取停止线信息
 * @details 该函数用于在给定的s值附近，根据前后距离获取停止线信息
 *
 * @param[in] s 当前s值，表示查询的中心点
 * @param[in] forward_dis 前向距离，表示从s值向前查询的范围
 * @param[in] backward_dis 后向距离，表示从s值向后查询的范围
 *
 * @par 输入参数说明:
 * - s: 查询的中心s值，单位米
 * - forward_dis: 前向查询距离，单位米
 * - backward_dis: 后向查询距离，单位米
 *
 * @par 关键变量说明:
 * - stop_lines_: 存储停止线信息的列表
 * - start_s (float): 查询范围的起始s值，计算为s - backward_dis
 * - end_s (float): 查询范围的结束s值，计算为s + forward_dis
 *
 * @par 处理流程:
 * 1. 计算查询范围的起始s值和结束s值
 * 2. 调用getStopLinesFromSRange函数，传入起始s值和结束s值
 * 3. 返回停止线信息
 *
 * @par 流程图:
 * @startuml
 * start
 * :计算start_s = s - backward_dis;
 * :计算end_s = s + forward_dis;
 * :调用getStopLinesFromSRange(start_s, end_s);
 * :返回停止线信息;
 * stop
 * @enduml
 *
 * @note 该函数主要用于在给定s值附近，根据前后距离获取停止线信息
 *
 * @warning 需确保输入的s范围在有效范围内
 */
std::vector<StopLine> LocalRoute::getStopLinesFromSRange(const float& s, const float& forward_dis,
                                                         const float& backward_dis) {
  return getStopLinesFromSRange(s - backward_dis, s + forward_dis);
}

/**
 * @brief 根据s范围获取闸机信息
 * @details 该函数用于在给定的s范围内获取闸机信息
 *
 * @param[in] start_s 起始s值，表示查询范围的起点
 * @param[in] end_s 结束s值，表示查询范围的终点
 *
 * @par 输入参数说明:
 * - start_s: 查询范围的起始s值，单位米
 * - end_s: 查询范围的结束s值，单位米
 *
 * @par 关键变量说明:
 * - gates_: 存储闸机信息的列表
 * - res (std::vector<Gate>): 存储查询结果的列表
 *
 * @par 处理流程:
 * 1. 遍历所有闸机信息
 * 2. 判断当前闸机的s值是否在查询范围内
 * 3. 如果在查询范围内，则添加到结果列表中
 *
 * @par 流程图:
 * @startuml
 * start
 * :初始化结果列表res;
 * while (遍历gates_?) is (是)
 *   :获取当前gate;
 *   if (gate.s < start_s?) then (是)
 *     :跳过当前gate;
 *   else if (gate.s > end_s?) then (是)
 *     :结束遍历;
 *   else (否)
 *     :将gate添加到res中;
 *   endif
 * endwhile
 * :返回结果列表res;
 * stop
 * @enduml
 *
 * @note 该函数主要用于获取在给定s范围内的闸机信息
 *
 * @warning 需确保输入的s范围在有效范围内
 */
std::vector<Gate> LocalRoute::getGatesFromSRange(const float& start_s, const float& end_s) {
  std::vector<Gate> res;
  for (const auto& gate : gates_) {
    if (gate.s < start_s) {
      continue;
    } else if (gate.s > end_s) {
      break;
    }
    res.emplace_back(gate);
  }
  return res;
}

/**
 * @brief 根据s范围和前后距离获取闸机信息
 * @details 该函数用于在给定的s值附近，根据前后距离获取闸机信息
 *
 * @param[in] s 当前s值，表示查询的中心点
 * @param[in] forward_dis 前向距离，表示从s值向前查询的范围
 * @param[in] backward_dis 后向距离，表示从s值向后查询的范围
 *
 * @par 输入参数说明:
 * - s: 查询的中心s值，单位米
 * - forward_dis: 前向查询距离，单位米
 * - backward_dis: 后向查询距离，单位米
 *
 * @par 关键变量说明:
 * - gates_: 存储闸机信息的列表
 * - start_s (float): 查询范围的起始s值，计算为s - backward_dis
 * - end_s (float): 查询范围的结束s值，计算为s + forward_dis
 *
 * @par 处理流程:
 * 1. 计算查询范围的起始s值和结束s值
 * 2. 调用getGatesFromSRange函数，传入起始s值和结束s值
 * 3. 返回闸机信息
 *
 * @par 流程图:
 * @startuml
 * start
 * :计算start_s = s - backward_dis;
 * :计算end_s = s + forward_dis;
 * :调用getGatesFromSRange(start_s, end_s);
 * :返回闸机信息;
 * stop
 * @enduml
 *
 * @note 该函数主要用于在给定s值附近，根据前后距离获取闸机信息
 *
 * @warning 需确保输入的s范围在有效范围内
 */
std::vector<Gate> LocalRoute::getGatesFromSRange(const float& s, const float& forward_dis, const float& backward_dis) {
  return getGatesFromSRange(s - backward_dis, s + forward_dis);
}

/**
 * @brief 根据s值获取速度限制信息
 * @details 该函数用于在给定的s值处获取速度限制信息
 *
 * @param[in] s 当前s值，表示查询点
 *
 * @par 输入参数说明:
 * - s: 查询点的s值，单位米
 *
 * @par 关键变量说明:
 * - speed_limits_: 存储速度限制信息的列表
 *
 * @par 处理流程:
 * 1. 遍历所有速度限制信息
 * 2. 判断当前s值是否在速度限制的范围内
 * 3. 如果在范围内，则返回该速度限制信息
 * 4. 如果未找到匹配的速度限制，则返回默认的SpeedLimit对象
 *
 * @par 流程图:
 * @startuml
 * start
 * :遍历speed_limits_;
 * if (s >= speed_limit.start_s && s <= speed_limit.end_s?) then (是)
 *   :返回speed_limit;
 * else (否)
 *   :继续遍历;
 * endif
 * :返回默认SpeedLimit对象;
 * stop
 * @enduml
 *
 * @note 该函数主要用于获取在给定s值处的速度限制信息
 *
 * @warning 需确保输入的s值在有效范围内
 */
SpeedLimit LocalRoute::getSpeedLimitFromS(const float& s) {
  for (const auto& speed_limit : speed_limits_) {
    if (s >= speed_limit.start_s && s <= speed_limit.end_s) {
      return speed_limit;
    }
  }
  return SpeedLimit();
}

/**
 * @brief 根据s范围获取方向信息
 * @details 该函数用于在给定的s范围内获取方向信息
 *
 * @param[in] start_s 起始s值，表示查询范围的起点
 * @param[in] end_s 结束s值，表示查询范围的终点
 *
 * @par 输入参数说明:
 * - start_s: 查询范围的起始s值，单位米
 * - end_s: 查询范围的结束s值，单位米
 *
 * @par 关键变量说明:
 * - directions_: 存储方向信息的列表
 * - res (std::vector<SegmentDirection>): 存储查询结果的列表
 *
 * @par 处理流程:
 * 1. 遍历所有方向信息
 * 2. 判断当前方向的s范围是否与查询范围有重叠
 * 3. 如果有重叠，则添加到结果列表中
 *
 * @par 流程图:
 * @startuml
 * start
 * :初始化结果列表res;
 * while (遍历directions_?) is (是)
 *   :获取当前direction;
 *   if (direction.end_s < start_s?) then (是)
 *     :跳过当前direction;
 *   else if (direction.start_s > end_s?) then (是)
 *     :结束遍历;
 *   else (否)
 *     :将direction添加到res中;
 *   endif
 * endwhile
 * :返回结果列表res;
 * stop
 * @enduml
 *
 * @note 该函数主要用于获取在给定s范围内的方向信息
 *
 * @warning 需确保输入的s范围在有效范围内
 */
std::vector<SegmentDirection> LocalRoute::getDirectionsFromSRange(const float& start_s, const float& end_s) {
  std::vector<SegmentDirection> res;
  for (const auto& direction : directions_) {
    if (direction.end_s < start_s) {
      continue;
    } else if (direction.start_s > end_s) {
      break;
    }
    res.emplace_back(direction);
  }
  return res;
}

/**
 * @brief 根据s范围和前后距离获取方向信息
 * @details 该函数用于在给定的s值附近，根据前后距离获取方向信息
 *
 * @param[in] s 当前s值，表示查询的中心点
 * @param[in] forward_dis 前向距离，表示从s值向前查询的范围
 * @param[in] backward_dis 后向距离，表示从s值向后查询的范围
 *
 * @par 输入参数说明:
 * - s: 查询的中心s值，单位米
 * - forward_dis: 前向查询距离，单位米
 * - backward_dis: 后向查询距离，单位米
 *
 * @par 关键变量说明:
 * - directions_: 存储方向信息的列表
 * - start_s (float): 查询范围的起始s值，计算为s - backward_dis
 * - end_s (float): 查询范围的结束s值，计算为s + forward_dis
 *
 * @par 处理流程:
 * 1. 计算查询范围的起始s值和结束s值
 * 2. 调用getDirectionsFromSRange函数，传入起始s值和结束s值
 * 3. 返回方向信息
 *
 * @par 流程图:
 * @startuml
 * start
 * :计算start_s = s - backward_dis;
 * :计算end_s = s + forward_dis;
 * :调用getDirectionsFromSRange(start_s, end_s);
 * :返回方向信息;
 * stop
 * @enduml
 *
 * @note 该函数主要用于在给定s值附近，根据前后距离获取方向信息
 *
 * @warning 需确保输入的s范围在有效范围内
 */
std::vector<SegmentDirection> LocalRoute::getDirectionsFromSRange(const float& s, const float& forward_dis,
                                                                  const float& backward_dis) {
  return getDirectionsFromSRange(s - backward_dis, s + forward_dis);
}

/**
 * @brief 根据s值获取方向信息
 * @details 该函数用于在给定的s值处获取方向信息
 *
 * @param[in] s 当前s值，表示查询点
 *
 * @par 输入参数说明:
 * - s: 查询点的s值，单位米
 *
 * @par 关键变量说明:
 * - directions_: 存储方向信息的列表
 *
 * @par 处理流程:
 * 1. 遍历所有方向信息
 * 2. 判断当前s值是否在方向的范围内
 * 3. 如果在范围内，则返回该方向信息
 * 4. 如果未找到匹配的方向信息，则返回默认的SegmentDirection对象
 *
 * @par 流程图:
 * @startuml
 * start
 * :遍历directions_;
 * if (s >= direction.start_s && s <= direction.end_s?) then (是)
 *   :返回direction;
 * else (否)
 *   :继续遍历;
 * endif
 * :返回默认SegmentDirection对象;
 * stop
 * @enduml
 *
 * @note 该函数主要用于获取在给定s值处的方向信息
 *
 * @warning 需确保输入的s值在有效范围内
 */
SegmentDirection LocalRoute::getDirectionFromS(const float& s) {
  for (const auto& direction : directions_) {
    if (s >= direction.start_s && s <= direction.end_s) {
      return direction;
    }
  }
  return SegmentDirection();
}

/**
 * @brief 根据s范围获取源信息
 * @details 该函数用于在给定的s范围内获取源信息
 *
 * @param[in] start_s 起始s值，表示查询范围的起点
 * @param[in] end_s 结束s值，表示查询范围的终点
 *
 * @par 输入参数说明:
 * - start_s: 查询范围的起始s值，单位米
 * - end_s: 查询范围的结束s值，单位米
 *
 * @par 关键变量说明:
 * - source_infos_: 存储源信息的列表
 * - res (std::vector<std::tuple<LineSourceType, float, float>>): 存储查询结果的列表
 *
 * @par 处理流程:
 * 1. 遍历所有源信息
 * 2. 判断当前源信息的s范围是否与查询范围有重叠
 * 3. 如果有重叠，则添加到结果列表中
 *
 * @par 流程图:
 * @startuml
 * start
 * :初始化结果列表res;
 * while (遍历source_infos_?) is (是)
 *   :获取当前source_info;
 *   if (std::get<2>(source_info) < start_s?) then (是)
 *     :跳过当前source_info;
 *   else if (std::get<1>(source_info) > end_s?) then (是)
 *     :结束遍历;
 *   else (否)
 *     :将source_info添加到res中;
 *   endif
 * endwhile
 * :返回结果列表res;
 * stop
 * @enduml
 *
 * @note 该函数主要用于获取在给定s范围内的源信息
 *
 * @warning 需确保输入的s范围在有效范围内
 */
std::vector<std::tuple<LineSourceType, float, float>> LocalRoute::getSourceInfosFromSRange(const float& start_s,
                                                                                           const float& end_s) {
  std::vector<std::tuple<LineSourceType, float, float>> res;
  for (const auto& source_info : source_infos_) {
    if (std::get<2>(source_info) < start_s) {
      continue;
    } else if (std::get<1>(source_info) > end_s) {
      break;
    }
    res.emplace_back(source_info);
  }
  return res;
}

/**
 * @brief 根据s范围和前后距离获取源信息
 * @details 该函数用于在给定的s值附近，根据前后距离获取源信息
 *
 * @param[in] s 当前s值，表示查询的中心点
 * @param[in] forward_dis 前向距离，表示从s值向前查询的范围
 * @param[in] backward_dis 后向距离，表示从s值向后查询的范围
 *
 * @par 输入参数说明:
 * - s: 查询的中心s值，单位米
 * - forward_dis: 前向查询距离，单位米
 * - backward_dis: 后向查询距离，单位米
 *
 * @par 关键变量说明:
 * - source_infos_: 存储源信息的列表
 * - start_s (float): 查询范围的起始s值，计算为s - backward_dis
 * - end_s (float): 查询范围的结束s值，计算为s + forward_dis
 *
 * @par 处理流程:
 * 1. 计算查询范围的起始s值和结束s值
 * 2. 调用getSourceInfosFromSRange函数，传入起始s值和结束s值
 * 3. 返回源信息
 *
 * @par 流程图:
 * @startuml
 * start
 * :计算start_s = s - backward_dis;
 * :计算end_s = s + forward_dis;
 * :调用getSourceInfosFromSRange(start_s, end_s);
 * :返回源信息;
 * stop
 * @enduml
 *
 * @note 该函数主要用于在给定s值附近，根据前后距离获取源信息
 *
 * @warning 需确保输入的s范围在有效范围内
 */
std::vector<std::tuple<LineSourceType, float, float>> LocalRoute::getSourceInfosFromSRange(const float& s,
                                                                                           const float& forward_dis,
                                                                                           const float& backward_dis) {
  return getSourceInfosFromSRange(s - backward_dis, s + forward_dis);
}

/**
 * @brief 根据s值获取源信息
 * @details 该函数用于在给定的s值处获取源信息
 *
 * @param[in] s 当前s值，表示查询点
 *
 * @par 输入参数说明:
 * - s: 查询点的s值，单位米
 *
 * @par 关键变量说明:
 * - source_infos_: 存储源信息的列表
 *
 * @par 处理流程:
 * 1. 遍历所有源信息
 * 2. 判断当前s值是否在源信息的范围内
 * 3. 如果在范围内，则返回该源信息
 * 4. 如果未找到匹配的源信息，则返回默认的源信息
 *
 * @par 流程图:
 * @startuml
 * start
 * :遍历source_infos_;
 * if (s >= std::get<1>(source_info) && s <= std::get<2>(source_info)?) then (是)
 *   :返回source_info;
 * else (否)
 *   :继续遍历;
 * endif
 * :返回默认源信息;
 * stop
 * @enduml
 *
 * @note 该函数主要用于获取在给定s值处的源信息
 *
 * @warning 需确保输入的s值在有效范围内
 */
std::tuple<LineSourceType, float, float> LocalRoute::getSourceInfoFromS(const float& s) {
  for (const auto& source_info : source_infos_) {
    if (s >= std::get<1>(source_info) && s <= std::get<2>(source_info)) {
      return source_info;
    }
  }
  return std::make_tuple<LineSourceType, float, float>(
      LineSourceType::LocalRouteReferenceLine_LineSource_LineSourceType_kTypeInvalid, 0.0, 0.0);
}

/**
 * @brief 根据s范围获取边界类型信息
 * @details 该函数用于在给定的s范围内获取边界类型信息
 *
 * @param[in] start_s 起始s值，表示查询范围的起点
 * @param[in] end_s 结束s值，表示查询范围的终点
 *
 * @par 输入参数说明:
 * - start_s: 查询范围的起始s值，单位米
 * - end_s: 查询范围的结束s值，单位米
 *
 * @par 关键变量说明:
 * - boundary_types_: 存储边界类型信息的列表
 * - res (std::vector<SegmentBoundaryType>): 存储查询结果的列表
 *
 * @par 处理流程:
 * 1. 遍历所有边界类型信息
 * 2. 判断当前边界类型的s范围是否与查询范围有重叠
 * 3. 如果有重叠，则添加到结果列表中
 *
 * @par 流程图:
 * @startuml
 * start
 * :初始化结果列表res;
 * while (遍历boundary_types_?) is (是)
 *   :获取当前boundary_type;
 *   if (boundary_type.end_s < start_s?) then (是)
 *     :跳过当前boundary_type;
 *   else if (boundary_type.start_s > end_s?) then (是)
 *     :结束遍历;
 *   else (否)
 *     :将boundary_type添加到res中;
 *   endif
 * endwhile
 * :返回结果列表res;
 * stop
 * @enduml
 *
 * @note 该函数主要用于获取在给定s范围内的边界类型信息
 *
 * @warning 需确保输入的s范围在有效范围内
 */
std::vector<SegmentBoundaryType> LocalRoute::getBoundaryTypesFromSRange(const float& start_s, const float& end_s) {
  std::vector<SegmentBoundaryType> res;
  for (const auto& boundary_type : boundary_types_) {
    if (boundary_type.end_s < start_s) {
      continue;
    } else if (boundary_type.start_s > end_s) {
      break;
    }
    res.emplace_back(boundary_type);
  }
  return res;
}

/**
 * @brief 根据s范围和前后距离获取边界类型信息
 * @details 该函数用于在给定的s值附近，根据前后距离获取边界类型信息
 *
 * @param[in] s 当前s值，表示查询的中心点
 * @param[in] forward_dis 前向距离，表示从s值向前查询的范围
 * @param[in] backward_dis 后向距离，表示从s值向后查询的范围
 *
 * @par 输入参数说明:
 * - s: 查询的中心s值，单位米
 * - forward_dis: 前向查询距离，单位米
 * - backward_dis: 后向查询距离，单位米
 *
 * @par 关键变量说明:
 * - boundary_types_: 存储边界类型信息的列表
 * - start_s (float): 查询范围的起始s值，计算为s - backward_dis
 * - end_s (float): 查询范围的结束s值，计算为s + forward_dis
 *
 * @par 处理流程:
 * 1. 计算查询范围的起始s值和结束s值
 * 2. 调用getBoundaryTypesFromSRange函数，传入起始s值和结束s值
 * 3. 返回边界类型信息
 *
 * @par 流程图:
 * @startuml
 * start
 * :计算start_s = s - backward_dis;
 * :计算end_s = s + forward_dis;
 * :调用getBoundaryTypesFromSRange(start_s, end_s);
 * :返回边界类型信息;
 * stop
 * @enduml
 *
 * @note 该函数主要用于在给定s值附近，根据前后距离获取边界类型信息
 *
 * @warning 需确保输入的s范围在有效范围内
 */
std::vector<SegmentBoundaryType> LocalRoute::getBoundaryTypesFromSRange(const float& s, const float& forward_dis,
                                                                        const float& backward_dis) {
  return getBoundaryTypesFromSRange(s - backward_dis, s + forward_dis);
}

/**
 * @brief 根据s值获取边界类型信息
 * @details 该函数用于在给定的s值处获取边界类型信息
 *
 * @param[in] s 当前s值，表示查询点
 *
 * @par 输入参数说明:
 * - s: 查询点的s值，单位米
 *
 * @par 关键变量说明:
 * - boundary_types_: 存储边界类型信息的列表
 *
 * @par 处理流程:
 * 1. 遍历所有边界类型信息
 * 2. 判断当前s值是否在边界类型的范围内
 * 3. 如果在范围内，则返回该边界类型信息
 * 4. 如果未找到匹配的边界类型信息，则返回默认的SegmentBoundaryType对象
 *
 * @par 流程图:
 * @startuml
 * start
 * :遍历boundary_types_;
 * if (s >= boundary_type.start_s && s <= boundary_type.end_s?) then (是)
 *   :返回boundary_type;
 * else (否)
 *   :继续遍历;
 * endif
 * :返回默认SegmentBoundaryType对象;
 * stop
 * @enduml
 *
 * @note 该函数主要用于获取在给定s值处的边界类型信息
 *
 * @warning 需确保输入的s值在有效范围内
 */
SegmentBoundaryType LocalRoute::getBoundaryTypeFromS(const float& s) {
  for (const auto& boundary_type : boundary_types_) {
    if (s >= boundary_type.start_s && s <= boundary_type.end_s) {
      return boundary_type;
    }
  }
  return SegmentBoundaryType();
}

/**
 * @brief LocalRouter构造函数
 * @details 该函数用于初始化LocalRouter对象
 *
 * @par 处理流程:
 * 1. 调用reset()函数重置所有成员变量
 * 2. 清空历史LocalRouteID列表
 *
 * @note 该函数主要用于创建LocalRouter对象时进行初始化
 */
LocalRouter::LocalRouter() {
  reset();
  history_local_routes_id_vec_.clear();
}

/**
 * @brief LocalRouter析构函数
 * @details 该函数用于销毁LocalRouter对象
 *
 * @note 该函数主要用于在LocalRouter对象生命周期结束时自动调用
 */
LocalRouter::~LocalRouter() {}

/**
 * @brief 重置LocalRouter对象的状态
 * @details 该函数用于重置LocalRouter对象的所有成员变量，将其恢复到初始状态
 *
 * @par 处理流程:
 * 1. 清空LocalRoute列表
 * 2. 清空车道与路段映射表
 * 3. 清空车道拓扑信息
 * 4. 重置当前LocalRouteID为0
 * 5. 重置当前LocalRoute的车道ID为空字符串
 * 6. 重置当前LocalRoute的路段ID为空字符串
 * 7. 清空道路信息
 * 8. 重置时间消耗为0.0
 *
 * @note 该函数主要用于在重新生成LocalRoute之前，清除之前的状态信息
 *
 * @warning 调用该函数后，所有之前存储的LocalRoute信息将被清除
 */
void LocalRouter::reset() {
  local_routes_.clear();
  lanes_segments_map_.clear();
  lanes_topos_.clear();
  current_local_route_id_ = 0;
  current_local_route_lane_id_ = "";
  current_local_route_segment_id_ = "";
  road_info_.clear();
  time_consumption_ = 0.0;
}

/**
 * @brief LocalRouter主处理函数
 * @details 该函数是LocalRouter的核心处理流程，负责生成、重建、选择和生成导航属性
 *
 * @par 处理流程:
 * 1. 生成LocalRoute
 * 2. 重建LocalRoute
 * 3. 选择需要平滑的LocalRoute
 * 4. 生成导航换道属性
 *
 * @par 关键变量说明:
 * - local_routes_: 存储所有LocalRoute的容器
 * - lanes_segments_map_: 车道与路段映射表
 * - lanes_topos_: 车道拓扑信息
 * - current_local_route_id_: 当前LocalRouteID
 * - current_local_route_lane_id_: 当前LocalRoute的车道ID
 * - current_local_route_segment_id_: 当前LocalRoute的路段ID
 * - road_info_: 道路信息
 * - time_consumption_: 时间消耗
 *
 * @par 流程图:
 * @startuml
 * start
 * :生成LocalRoute;
 * :重建LocalRoute;
 * :选择需要平滑的LocalRoute;
 * :生成导航换道属性;
 * stop
 * @enduml
 *
 * @note 该函数是LocalRouter的核心处理流程，负责协调各个子模块的工作
 *
 * @warning 该函数执行时间较长，需注意性能优化
 */
void LocalRouter::process() {
  // auto t1 = std::chrono::steady_clock::now();
  generateLocalRoutes();
  // auto t2 = std::chrono::steady_clock::now();
  rebuildLocalRoutes();
  // auto t3 = std::chrono::steady_clock::now();
  selectLocalRouteForSmooth();
  // auto t4 = std::chrono::steady_clock::now();
  generateNavigationLaneChangeProperty();
  // auto t5 = std::chrono::steady_clock::now();
  // auto durationa = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
  // auto durationb = std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count();
  // auto durationc = std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3).count();
  // auto durationd = std::chrono::duration_cast<std::chrono::milliseconds>(t5 - t4).count();
  // ERT_PLOG_I << "LocalRouteaaa: process   durationa = " << durationa << "  durationb = " << durationb
  //      << "  durationc= " << durationc << "  durationd= " << durationd ;

  // print useful information
  for (const auto& local_route : local_routes_) {
    ERT_PLOG_I << "  [LocalRouter::process]: key = " << local_route.first << "  id = " << local_route.second.id()
               << "  is_need_smooth = <" << std::get<0>(local_route.second.validRange()) << ", "
               << std::get<1>(local_route.second.validRange()) << ", " << std::get<2>(local_route.second.validRange())
               << "> lanes_id = <";
    for (const auto& lane_id : std::get<3>(local_route.second.validRange())) {
      ERT_PLOG_I << " " << lane_id;
    }
    ERT_PLOG_I << ">  adc_l = " << local_route.second.adcL()
               << "  adc_heading_diff = " << local_route.second.adcHeadingDiff()
               << "  adc_s = " << local_route.second.adcS() << "  adc_seg_id = " << local_route.second.adcSegId()
               << "  adc_lane_id = " << local_route.second.adcLaneId()
               << "  global_start_s = " << local_route.second.globalStartS()
               << "  length = " << local_route.second.length();

    //   // merge/fork info
    //   ERT_PLOG_I << "    [LocalRouter::process]: merge_fork_size = " << local_route.second.mergeForkRanges().size()
    //             ;
    //   for (const auto& range : local_route.second.mergeForkRanges()) {
    //     ERT_PLOG_I << "      [LocalRouter::process]: type = " << static_cast<int>(range.type()) << "  s = " <<
    //     range.s()
    //               << "  idinfo = <" << range.idInfo().first << ", " << range.idInfo().second
    //               << ">  relation_size = " << range.relationIdsInfo().size() ;
    //     for (const auto& info : range.relationIdsInfo()) {
    //       ERT_PLOG_I << "        [LocalRouter::process]: relation: lane_id = " << std::get<0>(info)
    //                 << "  seg_id = " << std::get<1>(info) << "  local_route_id = <";
    //       for (const auto& local_route_id : std::get<2>(info)) {
    //         ERT_PLOG_I << local_route_id << ", ";
    //       }
    //       ERT_PLOG_I << ">" ;
    //     }
    //   }

    //   // stop_line info
    //   ERT_PLOG_I << "    [LocalRouter::process]: stop_lines_size = " << local_route.second.stopLines().size() ;
    //   for (const auto& stop_line : local_route.second.stopLines()) {
    //     ERT_PLOG_I << "      [LocalRouter::process]: id = " << stop_line.id << "  is_exist = " << stop_line.is_exist
    //               << "  is_virtual = " << stop_line.is_virtual << "  s = " << stop_line.s
    //               << "  direction = " << static_cast<int>(stop_line.direction) ;
    //   }

    //   // gate info
    //   ERT_PLOG_I << "    [LocalRouter::process]: gates_size = " << local_route.second.gates().size() ;
    //   for (const auto& gate : local_route.second.gates()) {
    //     ERT_PLOG_I << "      [LocalRouter::process]: id = " << gate.id << "  is_exist = " << gate.is_exist
    //               << "  s = " << gate.s << "  gate_status = " << static_cast<int>(gate.gate_status)
    //               << "  head_stop_distance = " << gate.head_stop_distance ;
    //   }

    //   // directions info
    //   ERT_PLOG_I << "    [LocalRouter::process]: directions_size = " << local_route.second.directions().size()
    //             ;
    //   for (const auto& direction : local_route.second.directions()) {
    //     ERT_PLOG_I << "      [LocalRouter::process]: start_s = " << direction.start_s << "  end_s = " <<
    //     direction.end_s
    //               << "  direction = " << static_cast<int>(direction.direction) ;
    //   }

    //   // speedLimits info
    //   ERT_PLOG_I << "    [LocalRouter::process]: speedLimits_size = " << local_route.second.speedLimits().size()
    //             ;
    //   for (const auto& speed_limit : local_route.second.speedLimits()) {
    //     ERT_PLOG_I << "      [LocalRouter::process]: start_s = " << speed_limit.start_s
    //               << "  end_s = " << speed_limit.end_s << "  max_speed_limit = " << speed_limit.max_speed_limit *
    //               MS_KMH
    //               << "  min_speed_limit = " << speed_limit.min_speed_limit * MS_KMH ;
    //   }

    //   // boundaryTypes info
    //   ERT_PLOG_I << "    [LocalRouter::process]: boundaryTypes_size = " << local_route.second.boundaryTypes().size()
    //             ;
    //   for (const auto& boundary_type : local_route.second.boundaryTypes()) {
    //     ERT_PLOG_I << "      [LocalRouter::process]: start_s = " << boundary_type.start_s
    //               << "  end_s = " << boundary_type.end_s << "  left_type = " <<
    //               static_cast<int>(boundary_type.left_type)
    //               << "  right_type = " << static_cast<int>(boundary_type.right_type) ;
    //   }

    //   // neighbors info
    //   auto left_neighbors = local_route.second.getLeftNeighborLocalRouteBySRange(0.0, local_route.second.length());
    //   ERT_PLOG_I << "    [LocalRouter::process]: left_neighbors.size() = " << left_neighbors.size()
    //             << "  check range <0.0, " << local_route.second.length() << ">" ;
    //   for (const auto& neighbor : left_neighbors) {
    //     ERT_PLOG_I << "      [LocalRouter::process]: key = " << neighbor.first
    //               << "  neighbor local route id = " << neighbor.second.id
    //               << "  element size = " << neighbor.second.elements.size() ;
    //     for (const auto& element : neighbor.second.elements) {
    //       ERT_PLOG_I << "        [LocalRouter::process]: bound_type = " << static_cast<int>(element.bound_type)
    //                 << "  range = <" << std::get<0>(element.range) << ", " << std::get<1>(element.range) << ", "
    //                 << std::get<2>(element.range)
    //                 << ">  relation_segment_id_pair.size() = " << element.relation_segment_id_pair.size() ;
    //     }
    //   }
    //   auto right_neighbors = local_route.second.getRightNeighborLocalRouteBySRange(0.0, local_route.second.length());
    //   ERT_PLOG_I << "    [LocalRouter::process]: right_neighbors.size() = " << right_neighbors.size()
    //             << "  check range <0.0, " << local_route.second.length() << ">" ;
    //   for (const auto& neighbor : right_neighbors) {
    //     ERT_PLOG_I << "      [LocalRouter::process]: key = " << neighbor.first
    //               << "  neighbor local route id = " << neighbor.second.id
    //               << "  element size = " << neighbor.second.elements.size() ;
    //     for (const auto& element : neighbor.second.elements) {
    //       ERT_PLOG_I << "        [LocalRouter::process]: bound_type = " << static_cast<int>(element.bound_type)
    //                 << "  range = <" << std::get<0>(element.range) << ", " << std::get<1>(element.range) << ", "
    //                 << std::get<2>(element.range)
    //                 << ">  relation_segment_id_pair.size() = " << element.relation_segment_id_pair.size() ;
    //     }
    //   }

    //   // lanesSegments info
    //   ERT_PLOG_I << "    [LocalRouter::process]: lanesSegments_size = " << local_route.second.lanesSegments().size()
    //             ;
    //   for (const auto& lane_segs : local_route.second.lanesSegments()) {
    //     ERT_PLOG_I << "      [LocalRouter::process]: lane_id = " << lane_segs.first ;
    //     for (const auto& seg : lane_segs.second) {
    //       ERT_PLOG_I << "        [LocalRouter::process]: seg_id = " << seg.id() << "  score = " <<
    //       seg.navigationScore()
    //                 << "  range <" << std::get<0>(seg.range()) << ", " << std::get<1>(seg.range()) << ", "
    //                 << std::get<2>(seg.range()) << ">" ;
    //     }
    //   }

    //   // sourceInfos info
    //   ERT_PLOG_I << "    [LocalRouter::process]: sourceInfos_size = " << local_route.second.sourceInfos().size()
    //             ;
    //   for (const auto& source : local_route.second.sourceInfos()) {
    //     ERT_PLOG_I << "      [LocalRouter::process]: type = " << static_cast<int>(std::get<0>(source))
    //               << "  starts_s = " << std::get<1>(source) << "  end_s = " << std::get<2>(source) ;
    //   }

    // // navigation_lane_change_info_
    // ERT_PLOG_I << "      [LocalRouter::process]: navi_lane_change: dis = "
    //           << std::get<0>(local_route.second.navigationLaneChangeInfo())
    //           << "  direction = " << std::get<1>(local_route.second.navigationLaneChangeInfo())
    //           << "  num = " << std::get<2>(local_route.second.navigationLaneChangeInfo()) ;
  }
}

/**
 * @brief 生成LocalRoute
 * @details 该函数用于根据车道拓扑信息生成LocalRoute
 *
 * @par 处理流程:
 * 1. 清空LocalRoute列表
 * 2. 遍历所有车道拓扑
 * 3. 为每个车道拓扑创建LocalRoute对象
 * 4. 处理车道合并信息
 * 5. 处理车道分叉信息
 * 6. 处理路段信息（包括引导线、边界交叉范围、停止线、闸门、速度限制等）
 * 7. 设置道路信息和自车定位信息
 * 8. 生成唯一LocalRouteID
 * 9. 将LocalRoute添加到LocalRoute列表中
 *
 * @par 关键变量说明:
 * - local_routes_: 存储所有LocalRoute的容器
 * - lanes_topos_: 车道拓扑信息
 * - road_info_: 道路信息
 * - adc_loc_: 自车定位信息
 *
 * @par 流程图:
 * @startuml
 * start
 * :清空LocalRoute列表;
 * while (遍历车道拓扑) is (是)
 *   :创建LocalRoute对象;
 *   :处理车道合并信息;
 *   :处理车道分叉信息;
 *   :处理路段信息;
 *   :设置道路信息和自车定位信息;
 *   :生成唯一LocalRouteID;
 *   :将LocalRoute添加到列表中;
 * endwhile
 * stop
 * @enduml
 *
 * @note 该函数是LocalRoute生成的核心函数，负责根据车道拓扑信息生成LocalRoute
 *
 * @warning 该函数执行时间较长，需注意性能优化
 */
void LocalRouter::generateLocalRoutes() {
  local_routes_.clear();

  for (const auto& lane_topo : lanes_topos_) {
    LocalRoute local_route;

    float accumulated_seg_s = 0.0;
    float min_adc_l = std::numeric_limits<float>::max();
    for (int k = 0; k < lane_topo.size(); k++) {
      const auto& lane = lane_topo.at(k);
      // Merge info
      if (lane.previous_lanes.size() > 0 && k > 0) {
        std::vector<std::tuple<std::string, std::string, std::vector<std::string>>> relation_ids_info;
        for (const auto& previous_lane : lane.previous_lanes) {
          if (previous_lane.first != lane_topo.at(k - 1).lane_id) {
            std::tuple<std::string, std::string, std::vector<std::string>> relation_id_info;
            std::get<0>(relation_id_info) = previous_lane.first;
            std::get<1>(relation_id_info) = previous_lane.second;
            std::get<2>(relation_id_info).clear();
            relation_ids_info.emplace_back(std::move(relation_id_info));
          }
        }
        if (relation_ids_info.size() > 0) {
          local_route.mutableMergeForkRanges()->emplace_back();
          local_route.mutableMergeForkRanges()->back().setType(MergeForkRange::MergeForkType::MERGE);
          float s = local_route.guideLine().empty() ? 0.0 : local_route.guideLine().back().s();
          local_route.mutableMergeForkRanges()->back().setS(std::move(s));
          LocalRoutePoint point = local_route.guideLine().empty() ? LocalRoutePoint() : local_route.guideLine().back();
          local_route.mutableMergeForkRanges()->back().setPoint(std::move(point));
          std::pair<std::string, std::string> id_info;
          id_info.first = lane.lane_id;
          id_info.second = lane.lane_segments.empty() ? "" : lane.lane_segments.front().first;
          local_route.mutableMergeForkRanges()->back().setIdInfo(std::move(id_info));
          *(local_route.mutableMergeForkRanges()->back().mutableRelationIdsInfo()) = std::move(relation_ids_info);
        }
      }

      // info from segs
      local_route.mutableLanesSegments()->emplace_back();
      local_route.mutableLanesSegments()->back().first = lane.lane_id;
      for (const auto& seg : lane.lane_segments) {
        auto res = getSegmentBySegmentId(seg.first);
        if (res.first) {
          // guideLine
          for (auto pt : res.second.guidePoints()) {
            float s = pt.s() + accumulated_seg_s;
            pt.setS(s);
            local_route.mutableGuideLine()->emplace_back(std::move(pt));
          }

          // BoundaryCrossRange
          std::tuple<bool, float, float> range(std::get<0>(res.second.range()),
                                               std::get<1>(res.second.range()) + accumulated_seg_s,
                                               std::get<2>(res.second.range()) + accumulated_seg_s);
          local_route.mutableLeftBoundaryCrossRanges()->emplace_back();
          local_route.mutableLeftBoundaryCrossRanges()->back().setBoundType(res.second.leftBoundType());
          local_route.mutableLeftBoundaryCrossRanges()->back().setBoundRange(range);
          local_route.mutableLeftBoundaryCrossRanges()->back().setBoundRangePoints(res.second.rangePoints());
          local_route.mutableLeftBoundaryCrossRanges()->back().mutableSegmentsId()->emplace_back(res.second.id());
          local_route.mutableLeftBoundaryCrossRanges()->back().mutableSegmentsRange()->emplace_back(range);
          local_route.mutableLeftBoundaryCrossRanges()->back().mutableSegmentsRangePoints()->emplace_back(
              res.second.rangePoints());
          local_route.mutableLeftBoundaryCrossRanges()->back().mutableNeighborSegmentsId()->emplace_back();
          local_route.mutableLeftBoundaryCrossRanges()->back().mutableNeighborSegmentsId()->back().first =
              res.second.leftNeighborId();
          local_route.mutableLeftBoundaryCrossRanges()->back().mutableNeighborSegmentsId()->back().second.clear();
          local_route.mutableRightBoundaryCrossRanges()->emplace_back();
          local_route.mutableRightBoundaryCrossRanges()->back().setBoundType(res.second.rightBoundType());
          local_route.mutableRightBoundaryCrossRanges()->back().setBoundRange(range);
          local_route.mutableRightBoundaryCrossRanges()->back().setBoundRangePoints(res.second.rangePoints());
          local_route.mutableRightBoundaryCrossRanges()->back().mutableSegmentsId()->emplace_back(res.second.id());
          local_route.mutableRightBoundaryCrossRanges()->back().mutableSegmentsRange()->emplace_back(range);
          local_route.mutableRightBoundaryCrossRanges()->back().mutableSegmentsRangePoints()->emplace_back(
              res.second.rangePoints());
          local_route.mutableRightBoundaryCrossRanges()->back().mutableNeighborSegmentsId()->emplace_back();
          local_route.mutableRightBoundaryCrossRanges()->back().mutableNeighborSegmentsId()->back().first =
              res.second.rightNeighborId();
          local_route.mutableRightBoundaryCrossRanges()->back().mutableNeighborSegmentsId()->back().second.clear();

          // StopLine
          if (res.second.stopLine().is_exist) {
            local_route.mutableStopLines()->emplace_back(res.second.stopLine());
            local_route.mutableStopLines()->back().s = res.second.stopLine().s + accumulated_seg_s;
          }

          // Gate
          if (res.second.gate().is_exist) {
            local_route.mutableGates()->emplace_back(res.second.gate());
            local_route.mutableGates()->back().s = res.second.gate().s + accumulated_seg_s;
          }

          // SpeedLimit
          local_route.mutableSpeedLimits()->emplace_back(res.second.speedLimit());
          local_route.mutableSpeedLimits()->back().start_s = res.second.speedLimit().start_s + accumulated_seg_s;
          local_route.mutableSpeedLimits()->back().end_s = res.second.speedLimit().end_s + accumulated_seg_s;

          // SegmentDirection
          local_route.mutableDirections()->emplace_back();
          local_route.mutableDirections()->back().start_s = std::get<1>(range);
          local_route.mutableDirections()->back().end_s = std::get<2>(range);
          local_route.mutableDirections()->back().direction = res.second.direction();
          local_route.mutableDirections()->back().start_point =
              math::Vec3d(res.second.rangePoints().first.x(), res.second.rangePoints().first.y(),
                          res.second.rangePoints().first.z());
          local_route.mutableDirections()->back().end_point =
              math::Vec3d(res.second.rangePoints().second.x(), res.second.rangePoints().second.y(),
                          res.second.rangePoints().second.z());

          // SegmentBoundaryType
          local_route.mutableBoundaryTypes()->emplace_back();
          local_route.mutableBoundaryTypes()->back().start_s = std::get<1>(range);
          local_route.mutableBoundaryTypes()->back().end_s = std::get<2>(range);
          local_route.mutableBoundaryTypes()->back().left_type = res.second.leftBoundType();
          local_route.mutableBoundaryTypes()->back().right_type = res.second.rightBoundType();
          local_route.mutableBoundaryTypes()->back().start_point =
              math::Vec3d(res.second.rangePoints().first.x(), res.second.rangePoints().first.y(),
                          res.second.rangePoints().first.z());
          local_route.mutableBoundaryTypes()->back().end_point =
              math::Vec3d(res.second.rangePoints().second.x(), res.second.rangePoints().second.y(),
                          res.second.rangePoints().second.z());

          // Area
          local_route.mutableAreas()->insert(local_route.mutableAreas()->end(), res.second.areas().begin(),
                                             res.second.areas().end());

          // adc l s seg id
          if (res.second.adcL() < min_adc_l && fabs(res.second.adcHeadingDiff()) < M_PI_2) {
            min_adc_l = res.second.adcL();
            local_route.setAdcL(min_adc_l);
            local_route.setAdcHeadingDiff(res.second.adcHeadingDiff());
            local_route.setAdcS(res.second.adcS() + accumulated_seg_s);
            local_route.setAdcSegId(res.second.id());
            local_route.setAdcLaneId(lane.lane_id);
          }

          // source infos
          for (const auto& source : res.second.sourceInfos()) {
            std::tuple<LineSourceType, float, float> res;
            std::get<0>(res) = std::get<0>(source);
            std::get<1>(res) = std::get<1>(source) + accumulated_seg_s;
            std::get<2>(res) = std::get<2>(source) + accumulated_seg_s;
            local_route.mutableSourceInfos()->emplace_back(std::move(res));
          }

          // accumulated_seg_s
          accumulated_seg_s += res.second.length();

          // local_route
          local_route.mutableLanesSegments()->back().second.emplace_back(std::move(res.second));
        }
      }

      // Fork info
      if (lane.next_lanes.size() > 0 && (k + 1) < lane_topo.size()) {
        std::vector<std::tuple<std::string, std::string, std::vector<std::string>>> relation_ids_info;
        for (const auto& next_lane : lane.next_lanes) {
          if (next_lane.first != lane_topo.at(k + 1).lane_id) {
            std::tuple<std::string, std::string, std::vector<std::string>> relation_id_info;
            std::get<0>(relation_id_info) = next_lane.first;
            std::get<1>(relation_id_info) = next_lane.second;
            std::get<2>(relation_id_info).clear();
            relation_ids_info.emplace_back(std::move(relation_id_info));
          }
        }
        if (relation_ids_info.size() > 0) {
          local_route.mutableMergeForkRanges()->emplace_back();
          local_route.mutableMergeForkRanges()->back().setType(MergeForkRange::MergeForkType::FORK);
          float s = local_route.guideLine().empty() ? 0.0 : local_route.guideLine().back().s();
          local_route.mutableMergeForkRanges()->back().setS(std::move(s));
          LocalRoutePoint point = local_route.guideLine().empty() ? LocalRoutePoint() : local_route.guideLine().back();
          local_route.mutableMergeForkRanges()->back().setPoint(std::move(point));
          std::pair<std::string, std::string> id_info;
          id_info.first = lane.lane_id;
          id_info.second = lane.lane_segments.empty() ? "" : lane.lane_segments.back().first;
          local_route.mutableMergeForkRanges()->back().setIdInfo(std::move(id_info));
          *(local_route.mutableMergeForkRanges()->back().mutableRelationIdsInfo()) = std::move(relation_ids_info);
        }
      }
    }
    // RoadInfo
    local_route.setRoadInfo(road_info_);
    // loc
    *(local_route.mutableAdcLocalization()) = adc_loc_;

    local_route.setId(generateUniqueLocalRouteId(local_route));
    local_routes_[local_route.id()] = std::move(local_route);
  }
}

/**
 * @brief 生成唯一的LocalRoute ID
 * @details 该函数用于为每个LocalRoute生成唯一的ID，确保ID的唯一性和连续性
 *
 * @param[in] local_route 需要生成ID的LocalRoute对象
 *
 * @par 处理流程:
 * 1. 初始化ID为0
 * 2. 检查历史LocalRoute ID列表
 * 3. 如果当前LocalRoute的车道ID与历史LocalRoute的车道ID有重叠，则复用历史ID
 * 4. 如果没有可复用的历史ID，则生成新的ID
 * 5. 确保新ID在有效范围内（1到1e6）
 *
 * @par 关键变量说明:
 * - history_local_routes_id_vec_: 存储历史LocalRoute ID的列表
 * - new_id_cnt: 用于生成新ID的计数器
 *
 * @par 流程图:
 * @startuml
 * start
 * :初始化ID为0;
 * if (检查历史LocalRoute ID列表) then (有可复用ID)
 *   :复用历史ID;
 * else (无可复用ID)
 *   :生成新ID;
 * endif
 * :确保ID在有效范围内;
 * :返回生成的ID;
 * stop
 * @enduml
 *
 * @note 该函数主要用于确保LocalRoute ID的唯一性和连续性
 *
 * @warning 需确保生成的ID在有效范围内（1到1e6）
 */
int LocalRouter::generateUniqueLocalRouteId(const LocalRoute& local_route) {
  int id = 0;
  static int new_id_cnt = -1;
  std::vector<std::string> lane_id_vec;
  for (const auto& lane_segs : local_route.lanesSegments()) {
    lane_id_vec.emplace_back(lane_segs.first);
  }

  if (lane_id_vec.empty()) {
    new_id_cnt = (new_id_cnt + 1) % static_cast<int>(1e6);
    if (new_id_cnt == 0) {
      new_id_cnt = 1;
    }
    id = new_id_cnt;
    return id;
  }

  int max_common_id_cnt = 0;
  int history_local_route_id_reuesd_index = -1;
  for (int i = 0; i < history_local_routes_id_vec_.size(); i++) {
    const auto& history_local_route_id = history_local_routes_id_vec_.at(i);
    if (!std::get<0>(history_local_route_id)) {
      std::vector<std::string> common_id_vec;
      auto history_lane_id = std::get<2>(history_local_route_id).begin();
      auto lane_id = lane_id_vec.begin();
      while (history_lane_id != std::get<2>(history_local_route_id).end()) {
        while (lane_id != lane_id_vec.end()) {
          if (history_lane_id != std::get<2>(history_local_route_id).end() && *history_lane_id == *lane_id) {
            common_id_vec.emplace_back(*history_lane_id);
            ++history_lane_id;
            ++lane_id;
          } else {
            if (common_id_vec.size() > 0) {
              break;
            }
            ++lane_id;
          }
        }
        if (common_id_vec.size() > 0) {
          break;
        }
        ++history_lane_id;
        lane_id = lane_id_vec.begin();
      }

      if (common_id_vec.size() > 0 && common_id_vec.back() == std::get<2>(history_local_route_id).back() &&
          common_id_vec.front() == lane_id_vec.front()) {
        if (common_id_vec.size() > max_common_id_cnt) {
          max_common_id_cnt = common_id_vec.size();
          history_local_route_id_reuesd_index = i;
          id = std::get<1>(history_local_route_id);
        }
      }
    }
  }

  if (id == 0 || history_local_route_id_reuesd_index < 0) {
    new_id_cnt = (new_id_cnt + 1) % static_cast<int>(1e6);
    if (new_id_cnt == 0) {
      new_id_cnt = 1;
    }
    id = new_id_cnt;
  } else {
    std::get<0>(history_local_routes_id_vec_.at(history_local_route_id_reuesd_index)) = true;
  }

  return id;
}

/**
 * @brief 重建LocalRoute
 * @details 该函数用于重建LocalRoute的边界交叉范围、合并分叉范围、速度限制、方向信息和源信息
 *
 * @par 处理流程:
 * 1. 重建边界交叉范围
 * 2. 重建合并分叉范围
 * 3. 清空历史LocalRoute ID列表
 * 4. 遍历所有LocalRoute，重建其边界交叉范围、速度限制、方向信息和源信息
 * 5. 更新历史LocalRoute ID列表
 *
 * @par 关键变量说明:
 * - local_routes_: 存储所有LocalRoute的容器
 * - history_local_routes_id_vec_: 存储历史LocalRoute ID的列表
 *
 * @par 流程图:
 * @startuml
 * start
 * :重建边界交叉范围;
 * :重建合并分叉范围;
 * :清空历史LocalRoute ID列表;
 * while (遍历local_routes_) is (是)
 *   :重建边界交叉范围;
 *   :重建速度限制;
 *   :重建方向信息;
 *   :重建源信息;
 *   :更新历史LocalRoute ID列表;
 * endwhile
 * stop
 * @enduml
 *
 * @note 该函数主要用于在生成LocalRoute后，重建其各种属性信息
 *
 * @warning 该函数执行时间较长，需注意性能优化
 */
void LocalRouter::rebuildLocalRoutes() {
  rebuildBoundaryCrossRangesByLocalRouteId();
  rebuildMergeForkRangesByLocalRouteId();

  history_local_routes_id_vec_.clear();
  for (auto& local_route : local_routes_) {
    if (!local_route.second.RebuildBoundaryCrossRanges(local_route.second.mutableLeftBoundaryCrossRanges())) {
      std::cerr << "[LocalRouter::process]: local_route " << local_route.first
                << " rebuild left_boundary_cross_ranges failed! ";
    }
    if (!local_route.second.RebuildBoundaryCrossRanges(local_route.second.mutableRightBoundaryCrossRanges())) {
      std::cerr << "[LocalRouter::process]: local_route " << local_route.first
                << " rebuild right_boundary_cross_ranges failed! ";
    }
    if (!local_route.second.RebuildSpeedLimits()) {
      std::cerr << "[LocalRouter::process]: local_route " << local_route.first << " rebuild speed_limits failed! ";
    }
    if (!local_route.second.RebuildDirections()) {
      std::cerr << "[LocalRouter::process]: local_route " << local_route.first << " rebuild directions failed! ";
    }
    if (!local_route.second.RebuildSourceInfos()) {
      std::cerr << "[LocalRouter::process]: local_route " << local_route.first << " rebuild sourcesinfo failed! ";
    }

    history_local_routes_id_vec_.emplace_back();
    std::get<0>(history_local_routes_id_vec_.back()) = false;
    std::get<1>(history_local_routes_id_vec_.back()) = local_route.second.id();
    std::get<2>(history_local_routes_id_vec_.back()).clear();
    for (const auto& lane_segs : local_route.second.lanesSegments()) {
      std::get<2>(history_local_routes_id_vec_.back()).emplace_back(lane_segs.first);
    }
  }

  ERT_PLOG_I << "[LocalRouter::rebuildLocalRoutes]: history_local_routes_id_vec_size() = "
             << history_local_routes_id_vec_.size();
}

/**
 * @brief 根据LocalRoute ID重建边界交叉范围
 * @details 该函数用于重建LocalRoute的左右边界交叉范围，并更新其邻居路段ID信息
 *
 * @par 处理流程:
 * 1. 遍历所有LocalRoute
 * 2. 对于每个LocalRoute，遍历其左边界交叉范围
 * 3. 清除邻居路段ID信息，并根据路段ID获取相关LocalRoute ID
 * 4. 如果找到相关LocalRoute ID，则更新邻居路段ID信息
 * 5. 重复步骤2-4，处理右边界交叉范围
 *
 * @par 关键变量说明:
 * - local_routes_: 存储所有LocalRoute的容器
 * - lanes_segments_map_: 存储车道与路段映射的容器
 *
 * @par 流程图:
 * @startuml
 * start
 * while (遍历local_routes_) is (是)
 *   :遍历左边界交叉范围;
 *   :清除邻居路段ID信息;
 *   :根据路段ID获取相关LocalRoute ID;
 *   if (找到相关LocalRoute ID) then (是)
 *     :更新邻居路段ID信息;
 *   else (否)
 *     :输出错误信息;
 *   endif
 *   :遍历右边界交叉范围;
 *   :清除邻居路段ID信息;
 *   :根据路段ID获取相关LocalRoute ID;
 *   if (找到相关LocalRoute ID) then (是)
 *     :更新邻居路段ID信息;
 *   else (否)
 *     :输出错误信息;
 *   endif
 * endwhile
 * stop
 * @enduml
 *
 * @note 该函数主要用于在生成LocalRoute后，重建其边界交叉范围信息
 *
 * @warning 需确保输入的LocalRoute ID和路段ID有效
 */
void LocalRouter::rebuildBoundaryCrossRangesByLocalRouteId() {
  for (auto& local_route : local_routes_) {
    for (auto& l_range : *local_route.second.mutableLeftBoundaryCrossRanges()) {
      for (auto& neighbor : *l_range.mutableNeighborSegmentsId()) {
        neighbor.second.clear();
        auto res = getLocalRoutesIdBySegmentId(neighbor.first);
        if (!res.first) {
          // std::cerr << "[LocalRouter::rebuildBoundaryCrossRangesByLocalRouteId]: local_route id " <<
          // local_route.first
          //           << " - left_neighbor id " << neighbor.first
          //           << " belongs to no local route";
        } else {
          auto itr = std::find(res.second.begin(), res.second.end(), local_route.second.id());
          if (itr != res.second.end()) {
            res.second.erase(itr);
          }
          if (res.second.empty()) {
            std::cerr << "[LocalRouter::rebuildBoundaryCrossRangesByLocalRouteId]: local_route id " << local_route.first
                      << " - left_neighbor id " << neighbor.first
                      << " belongs to no other local routes except current local route";
          }

          neighbor.second = std::move(res.second);
          // ERT_PLOG_I << "current local_route id = " << local_route.first
          //           << "  neighbor seg id = " << neighbor.first
          //           << "  range = <" << std::get<1>(l_range.boundRange())
          //           << ", " << std::get<2>(l_range.boundRange())
          //           << ">  type = " << static_cast<int>(l_range.boundType())
          //           ;
          // for(const auto& el : neighbor.second) {
          //   ERT_PLOG_I << "      neighbor local_route id = " << el ;
          // }
        }
      }
    }

    for (auto& r_range : *local_route.second.mutableRightBoundaryCrossRanges()) {
      for (auto& neighbor : *r_range.mutableNeighborSegmentsId()) {
        neighbor.second.clear();
        auto res = getLocalRoutesIdBySegmentId(neighbor.first);
        if (!res.first) {
          // std::cerr << "[LocalRouter::rebuildBoundaryCrossRangesByLocalRouteId]: local_route id " <<
          // local_route.first
          //           << " - right_neighbor id " << neighbor.first
          //           << " belongs to no local route";
        } else {
          auto itr = std::find(res.second.begin(), res.second.end(), local_route.second.id());
          if (itr != res.second.end()) {
            res.second.erase(itr);
          }
          if (res.second.empty()) {
            std::cerr << "[LocalRouter::rebuildBoundaryCrossRangesByLocalRouteId]: local_route id " << local_route.first
                      << " - right_neighbor id " << neighbor.first
                      << " belongs to no other local routes except current local route";
          }

          neighbor.second = std::move(res.second);
          // ERT_PLOG_I << "current local_route id = " << local_route.first
          //           << "  neighbor seg id = " << neighbor.first
          //           << "  range = <" << std::get<1>(r_range.boundRange())
          //           << ", " << std::get<2>(r_range.boundRange())
          //           << ">  type = " << static_cast<int>(r_range.boundType())
          //           ;
          // for(const auto& el : neighbor.second) {
          //   ERT_PLOG_I << "      neighbor local_route id = " << el ;
          // }
        }
      }
    }
  }
}

/**
 * @brief 根据LocalRoute ID重建合并分叉范围
 * @details 该函数用于重建LocalRoute的合并分叉范围，并更新其相关LocalRoute ID信息
 *
 * @par 处理流程:
 * 1. 遍历所有LocalRoute
 * 2. 对于每个LocalRoute，遍历其合并分叉范围
 * 3. 清除相关LocalRoute ID信息，并根据车道ID获取相关LocalRoute ID
 * 4. 如果找到相关LocalRoute ID，则更新相关LocalRoute ID信息
 *
 * @par 关键变量说明:
 * - local_routes_: 存储所有LocalRoute的容器
 * - lanes_segments_map_: 存储车道与路段映射的容器
 *
 * @par 流程图:
 * @startuml
 * start
 * while (遍历local_routes_) is (是)
 *   :遍历合并分叉范围;
 *   :清除相关LocalRoute ID信息;
 *   :根据车道ID获取相关LocalRoute ID;
 *   if (找到相关LocalRoute ID) then (是)
 *     :更新相关LocalRoute ID信息;
 *   else (否)
 *     :输出错误信息;
 *   endif
 * endwhile
 * stop
 * @enduml
 *
 * @note 该函数主要用于在生成LocalRoute后，重建其合并分叉范围信息
 *
 * @warning 需确保输入的LocalRoute ID和车道ID有效
 */
void LocalRouter::rebuildMergeForkRangesByLocalRouteId() {
  for (auto& local_route : local_routes_) {
    for (auto& range : *local_route.second.mutableMergeForkRanges()) {
      for (auto& relation : *range.mutableRelationIdsInfo()) {
        std::get<2>(relation).clear();
        auto res = getLocalRoutesStringIdByLaneId(std::get<0>(relation));
        if (!res.first) {
          // std::cerr << "[LocalRouter::rebuildMergeForkRangesByLocalRouteId]: local_route id " << local_route.first
          //           << " - lane id " << std::get<0>(relation)
          //           << " belongs to no local route";
        } else {
          auto itr = std::find(res.second.begin(), res.second.end(), std::to_string(local_route.second.id()));
          if (itr != res.second.end()) {
            res.second.erase(itr);
          }
          if (res.second.empty()) {
            // std::cerr << "[LocalRouter::rebuildMergeForkRangesByLocalRouteId]: local_route id " << local_route.first
            //         << " - lane id " << std::get<0>(relation)
            //         << " belongs to no other local routes except current local route";
          }
          std::get<2>(relation) = std::move(res.second);
          // ERT_PLOG_I << "current local_route id = " << local_route.first
          //           << "  relation lane id = " << std::get<0>(relation)
          //           << "  relation seg id = <" << std::get<1>(relation)
          //           << "  relation local_route id size = <" << std::get<2>(relation).size()
          //           ;
          // for(const auto& el : std::get<2>(relation)) {
          //   ERT_PLOG_I << "  relation local_route id = " << el ;
          // }
        }
      }
    }
  }
}

/**
 * @brief 选择需要平滑的LocalRoute
 * @details 该函数用于根据路段属性、导航分、指定范围及自车位置等信息，筛选出需要平滑的LocalRoute
 *
 * @par 处理流程:
 * 1. 基于路段属性及导航分，剔除不需平滑的local_routes
 * 2. 基于指定范围及虚拟掉头路口运动学约束，进一步剔除不需平滑的local_routes
 * 3. 基于自车位置，剔除距离较远或方向差异较大的local_routes
 * 4. 更新当前LocalRoute信息
 *
 * @par 关键变量说明:
 * - local_routes_: 存储所有LocalRoute的容器
 * - current_local_route_id_: 当前选中的LocalRoute ID
 * - current_local_route_lane_id_: 当前选中的车道ID
 * - current_local_route_segment_id_: 当前选中的路段ID
 *
 * @par 流程图:
 * @startuml
 * start
 * :基于路段属性及导航分剔除;
 * :基于指定范围及虚拟掉头路口运动学约束剔除;
 * :基于自车位置剔除;
 * :更新当前LocalRoute信息;
 * stop
 * @enduml
 *
 * @note 该函数主要用于在生成LocalRoute后，筛选出需要平滑的LocalRoute
 *
 * @warning 需确保输入的LocalRoute信息有效
 */
void LocalRouter::selectLocalRouteForSmooth() {
  // 1）基于路段属性及导航分，剔除不需平滑的local_routes
  for (auto& local_route : local_routes_) {
    auto valid_range = validPolicyByNavigationScore(local_route.second);
    local_route.second.setValidRange(std::move(valid_range));
  }
  localRoutesSubsetRelationProcess();
  int first_cnt = 0;
  for (const auto& local_route : local_routes_) {
    if (std::get<0>(local_route.second.validRange())) {
      first_cnt++;
    }
  }
  ERT_PLOG_I << "[LocalRouter::selectLocalRouteForSmooth]: first: local_route_need_smooth_size = " << first_cnt;
  // for(auto& local_route : local_routes_) {
  //   ERT_PLOG_I << "  [LocalRouter::process]: key = " << local_route.first
  //             << "  id = " << local_route.second.id()
  //             << "  is_need_smooth = <" << std::get<0>(local_route.second.validRange())
  //             << ", " << std::get<1>(local_route.second.validRange())
  //             << ", " << std::get<2>(local_route.second.validRange())
  //             << "> lanes_id = <";
  //   for(auto& lane_id : std::get<3>(local_route.second.validRange())) {
  //     ERT_PLOG_I << " " << lane_id;
  //   }
  //   ERT_PLOG_I << "> ";
  // }

  // 2）基于指定范围及虚拟掉头路口运动学约束，剔除不需平滑的local_routes
  for (auto& local_route : local_routes_) {
    auto valid_range = validPolicyBySpecifiedRange(local_route.second);
    local_route.second.setValidRange(std::move(valid_range));
  }
  localRoutesSubsetRelationProcess();
  int second_cnt = 0;
  for (const auto& local_route : local_routes_) {
    if (std::get<0>(local_route.second.validRange())) {
      second_cnt++;
    }
  }
  ERT_PLOG_I << "[LocalRouter::selectLocalRouteForSmooth]: second: local_route_need_smooth_size = " << second_cnt;
  // for(auto& local_route : local_routes_) {
  //   ERT_PLOG_I << "  [LocalRouter::process]: key = " << local_route.first
  //             << "  id = " << local_route.second.id()
  //             << "  is_need_smooth = <" << std::get<0>(local_route.second.validRange())
  //             << ", " << std::get<1>(local_route.second.validRange())
  //             << ", " << std::get<2>(local_route.second.validRange())
  //             << "> lanes_id = <";
  //   for(auto& lane_id : std::get<3>(local_route.second.validRange())) {
  //     ERT_PLOG_I << " " << lane_id;
  //   }
  //   ERT_PLOG_I << "> ";
  // }

  float max_adc_s = std::numeric_limits<float>::lowest();
  float min_adc_l = std::numeric_limits<float>::max();
  for (auto& local_route : local_routes_) {
    if (local_route.second.adcS() > max_adc_s) {
      max_adc_s = local_route.second.adcS();
    }

    // 在需要平滑的local_routes中，找出最近的local_route_id及lane_id及segment_id
    if (std::get<0>(local_route.second.validRange())) {
      if (local_route.second.adcL() < min_adc_l && std::fabs(local_route.second.adcHeadingDiff()) < M_PI_2) {
        min_adc_l = local_route.second.adcL();
        current_local_route_segment_id_ = local_route.second.adcSegId();
        current_local_route_lane_id_ = local_route.second.adcLaneId();
        current_local_route_id_ = local_route.first;
      }
    }
  }

  float interst_range = 20.0;
  for (auto& local_route : local_routes_) {
    local_route.second.setGlobalStartS(max_adc_s - local_route.second.adcS());

    if (current_local_route_id_ == local_route.second.id()) {
      local_route.second.setIsCurrentLocalRoute(true);
    } else {
      local_route.second.setIsCurrentLocalRoute(false);
    }

    // 3）基于自车位置，剔除不需平滑的local_routes
    if (std::fabs(local_route.second.adcL()) > min_adc_l + interst_range ||
        std::fabs(local_route.second.adcHeadingDiff()) >= M_PI_2) {
      local_route.second.setValidRange(
          std::make_tuple<bool, float, float, std::vector<string>>(false, 0.0, 0.0, std::vector<string>()));
    }
  }
  int third_cnt = 0;
  for (const auto& local_route : local_routes_) {
    if (std::get<0>(local_route.second.validRange())) {
      third_cnt++;
    }
  }
  ERT_PLOG_I << "[LocalRouter::selectLocalRouteForSmooth]: third: local_route_need_smooth_size = " << third_cnt;
  ERT_PLOG_I << "[LocalRouter::selectLocalRouteForSmooth]: local_route_size = " << local_routes_.size()
             << "  min_adc_l = " << min_adc_l << "  current_local_route_id_ = " << current_local_route_id_
             << "  current_local_route_lane_id_ = " << current_local_route_lane_id_
             << "  current_local_route_segment_id_ = " << current_local_route_segment_id_;
}

/**
 * @brief 根据导航分计算LocalRoute的有效范围
 * @details 该函数用于根据导航分判断LocalRoute的有效范围，主要处理虚拟路段和导航分较低的情况
 *
 * @par 处理流程:
 * 1. 初始化有效范围，默认无效
 * 2. 遍历LocalRoute的所有车道和路段
 * 3. 如果当前路段是虚拟路段且导航分低于阈值，则截断有效范围
 * 4. 累加路段长度，更新有效范围的起始和结束位置
 * 5. 记录有效范围内的车道ID
 *
 * @par 关键变量说明:
 * - local_route: 输入的LocalRoute对象
 * - valid_range: 返回的有效范围，包含是否有效、起始位置、结束位置和车道ID列表
 *
 * @par 流程图:
 * @startuml
 * start
 * :初始化有效范围;
 * while (遍历车道) is (是)
 *   while (遍历路段) is (是)
 *     if (虚拟路段且导航分低于阈值) then (是)
 *       :截断有效范围;
 *       break;
 *     endif
 *     :累加路段长度;
 *     :更新有效范围;
 *     :记录车道ID;
 *   endwhile
 * endwhile
 * :返回有效范围;
 * stop
 * @enduml
 *
 * @note 该函数主要用于处理虚拟路段和导航分较低的情况，确保LocalRoute的有效性
 *
 * @warning 需确保输入的LocalRoute对象有效
 */
std::tuple<bool, float, float, std::vector<std::string>> LocalRouter::validPolicyByNavigationScore(
    const LocalRoute& local_route) {
  std::tuple<bool, float, float, std::vector<std::string>> valid_range(false, 0.0, 0.0, std::vector<string>());

  float accumulated_s = 0.0;
  bool stop_flag = false;
  for (const auto& lane_segs : local_route.lanesSegments()) {
    for (int i = 0; i < lane_segs.second.size(); i++) {
      // lane中首个路段为虚拟路段且导航分0时，截断.
      if (lane_segs.second.at(0).type() == PerceptionLaneSegment::LaneSegmentType::kTypeVirtual &&
          lane_segs.second.at(0).navigationScore() < (1e-3)) {
        stop_flag = true;
        break;
      }

      accumulated_s += lane_segs.second.at(i).length();
      if (std::get<3>(valid_range).empty()) {
        std::get<0>(valid_range) = true;
        std::get<1>(valid_range) = accumulated_s - lane_segs.second.at(i).length();
        std::get<2>(valid_range) = accumulated_s;
        std::get<3>(valid_range).emplace_back(lane_segs.first);
      } else {
        std::get<2>(valid_range) = accumulated_s;
        if (std::get<3>(valid_range).back() != lane_segs.first) {
          std::get<3>(valid_range).emplace_back(lane_segs.first);
        }
      }
    }
    if (stop_flag) {
      break;
    }
  }

  return valid_range;
}

/**
 * @brief 根据指定范围计算LocalRoute的有效范围
 * @details 该函数用于根据指定范围判断LocalRoute的有效范围，主要处理虚拟路段和掉头路口运动学约束
 *
 * @par 处理流程:
 * 1. 检查当前LocalRoute是否有效，如果无效则直接返回
 * 2. 如果LocalRoute的结束位置小于自车位置，则返回无效范围
 * 3. 根据前后距离阈值计算LocalRoute的有效起始和结束位置
 * 4. 遍历LocalRoute的所有车道和路段
 * 5. 如果当前路段是虚拟路段且是掉头路口，且不满足运动学约束，则截断有效范围
 * 6. 累加路段长度，更新有效范围的起始和结束位置
 * 7. 记录有效范围内的车道ID
 *
 * @par 关键变量说明:
 * - local_route: 输入的LocalRoute对象
 * - valid_range: 返回的有效范围，包含是否有效、起始位置、结束位置和车道ID列表
 * - front_length_thresold: 前方距离阈值，默认400.0米
 * - back_length_thresold: 后方距离阈值，默认150.0米
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查LocalRoute是否有效;
 * if (无效) then (是)
 *   :返回无效范围;
 * endif
 * if (结束位置小于自车位置) then (是)
 *   :返回无效范围;
 * endif
 * :计算有效起始和结束位置;
 * while (遍历车道) is (是)
 *   while (遍历路段) is (是)
 *     if (虚拟路段且是掉头路口且不满足运动学约束) then (是)
 *       :截断有效范围;
 *       break;
 *     endif
 *     :累加路段长度;
 *     :更新有效范围;
 *     :记录车道ID;
 *   endwhile
 * endwhile
 * :返回有效范围;
 * stop
 * @enduml
 *
 * @note 该函数主要用于处理虚拟路段和掉头路口运动学约束，确保LocalRoute的有效性
 *
 * @warning 需确保输入的LocalRoute对象有效
 */
std::tuple<bool, float, float, std::vector<std::string>> LocalRouter::validPolicyBySpecifiedRange(
    const LocalRoute& local_route) {
  if (!std::get<0>(local_route.validRange())) {
    return local_route.validRange();
  }

  std::tuple<bool, float, float, std::vector<std::string>> valid_range(false, 0.0, 0.0, std::vector<string>());
  if (std::get<2>(local_route.validRange()) < local_route.adcS() + (1e-2)) {
    return valid_range;
  }

  float front_length_thresold = 400.0;
  float back_length_thresold = 150.0;
  float local_route_start_s =
      std::fmax(std::get<1>(local_route.validRange()), std::fmax(0.0, local_route.adcS() - back_length_thresold));
  float local_route_end_s = std::fmin(std::get<2>(local_route.validRange()),
                                      std::fmin(local_route.length(), local_route.adcS() + front_length_thresold));
  float accumulated_s = 0.0;
  bool stop_flag = false;
  for (const auto& lane_segs : local_route.lanesSegments()) {
    for (int i = 0; i < lane_segs.second.size(); i++) {
      if (lane_segs.second.at(0).type() == PerceptionLaneSegment::LaneSegmentType::kTypeVirtual &&
          lane_segs.second.at(0).direction() == DrivingDirection::kDirectionUTurnOnly &&
          accumulated_s + lane_segs.second.at(i).length() + 20.0 > local_route.adcS() &&
          !IsSegmentMatchMotionConstraints(lane_segs.second.at(0))) {
        stop_flag = true;
        break;
      }

      accumulated_s += lane_segs.second.at(i).length();
      // 截取指定范围内的有效lane
      if (accumulated_s < local_route_start_s) {
        continue;
      }
      if (std::get<3>(valid_range).empty()) {
        std::get<0>(valid_range) = true;
        std::get<1>(valid_range) = accumulated_s - lane_segs.second.at(i).length();
        std::get<2>(valid_range) = accumulated_s;
        std::get<3>(valid_range).emplace_back(lane_segs.first);
      } else {
        if (accumulated_s - lane_segs.second.at(i).length() <= local_route_end_s) {
          std::get<2>(valid_range) = accumulated_s;
          if (std::get<3>(valid_range).back() != lane_segs.first) {
            std::get<3>(valid_range).emplace_back(lane_segs.first);
          }
        } else {
          stop_flag = true;
          break;
        }
      }
    }
    if (stop_flag) {
      break;
    }
  }

  return valid_range;
}

/**
 * @brief 处理LocalRoute的子集关系
 * @details 该函数用于检查LocalRoute之间的子集关系，并优先向ID较小的LocalRoute合并，以帮助决策端使用历史ID的维稳
 *
 * @par 处理流程:
 * 1. 定义子集检查函数，判断一个LocalRoute是否是另一个LocalRoute的子集
 * 2. 定义查找超集函数，查找包含当前LocalRoute的超集LocalRoute
 * 3. 使用反向迭代器倒序遍历所有LocalRoute
 * 4. 如果当前LocalRoute有效，则查找其超集LocalRoute
 * 5. 如果找到超集LocalRoute，则将当前LocalRoute标记为无效，并清除其有效范围信息
 *
 * @par 关键变量说明:
 * - local_routes_: 存储所有LocalRoute的容器
 * - history_local_routes_id_vec_: 存储历史LocalRoute ID的列表
 *
 * @par 流程图:
 * @startuml
 * start
 * :定义子集检查函数;
 * :定义查找超集函数;
 * while (倒序遍历local_routes_) is (是)
 *   if (当前LocalRoute有效) then (是)
 *     :查找超集LocalRoute;
 *     if (找到超集LocalRoute) then (是)
 *       :标记当前LocalRoute为无效;
 *       :清除有效范围信息;
 *     endif
 *   endif
 * endwhile
 * stop
 * @enduml
 *
 * @note 该函数主要用于处理LocalRoute之间的子集关系，确保决策端使用历史ID的维稳
 *
 * @warning 需确保输入的LocalRoute对象有效
 */
void LocalRouter::localRoutesSubsetRelationProcess() {
  // subset and superset check
  auto isSubsetLocalRoute = [](const LocalRoute& superset_route, const LocalRoute& subset_route) {
    auto superset = std::get<3>(superset_route.validRange());
    auto subset = std::get<3>(subset_route.validRange());
    auto it_superset = superset.begin();
    auto it_subset = subset.begin();
    while (it_subset != subset.end() && it_superset != superset.end()) {
      if (*it_superset == *it_subset) {
        ++it_subset;
      }
      ++it_superset;
    }
    return it_subset == subset.end();
  };

  auto isFindSupersetLocalRoute = [&](const std::map<int, LocalRoute>& routes, const LocalRoute& check_route) {
    std::pair<bool, int> res(false, 0);
    for (const auto& route : routes) {
      if (route.first != check_route.id() && std::get<0>(route.second.validRange()) &&
          isSubsetLocalRoute(route.second, check_route)) {
        res.first = true;
        res.second = route.first;
        return res;
      }
    }
    return res;
  };

  // 使用反向迭代器倒序遍历, 若两条local_route互为子集关系，则优先向id小的local_route合并，以帮助决策端使用历时id的维稳
  for (std::map<int, LocalRoute>::reverse_iterator itr = local_routes_.rbegin(); itr != local_routes_.rend(); ++itr) {
    if (std::get<0>(itr->second.validRange())) {
      auto res = isFindSupersetLocalRoute(local_routes_, itr->second);
      if (res.first) {
        std::get<0>(*(itr->second.mutableValidRange())) = false;
        std::get<1>(*(itr->second.mutableValidRange())) = 0.0;
        std::get<2>(*(itr->second.mutableValidRange())) = 0.0;
        std::get<3>(*(itr->second.mutableValidRange())).clear();
        // ERT_PLOG_I << "" << itr->second.id() << " is sub_route of " << res.second ;
      }
    }
  }
}

/**
 * @brief 生成导航变道属性信息
 * @details 该函数用于为每个LocalRoute生成导航变道相关的属性信息，包括变道距离、变道方向和变道次数
 *
 * @par 处理流程:
 * 1. 初始化最大导航分和最大导航分对应的路段ID列表
 * 2. 遍历所有LocalRoute，获取其当前路段的导航变道信息
 * 3. 如果当前路段的导航变道距离有效，则计算变道距离并更新最大导航分
 * 4. 如果最大导航分对应的路段ID列表不为空，则遍历所有LocalRoute，判断其与最大导航分路段的邻接关系
 * 5. 根据邻接关系更新LocalRoute的导航变道信息
 *
 * @par 关键变量说明:
 * - max_navigation_score_segments: 存储最大导航分对应的路段ID列表
 * - navigation_route_score_thresold: 导航分阈值，默认1.0
 * - max_navigation_score: 最大导航分
 *
 * @par 流程图:
 * @startuml
 * start
 * :初始化最大导航分和路段ID列表;
 * while (遍历local_routes_) is (是)
 *   :获取当前路段的导航变道信息;
 *   if (导航变道距离有效) then (是)
 *     :计算变道距离;
 *     :更新最大导航分;
 *   endif
 * endwhile
 * if (最大导航分路段ID列表不为空) then (是)
 *   while (遍历local_routes_) is (是)
 *     :判断与最大导航分路段的邻接关系;
 *     :更新导航变道信息;
 *   endwhile
 * endif
 * stop
 * @enduml
 *
 * @note 该函数主要用于为LocalRoute生成导航变道相关的属性信息，帮助决策端进行变道决策
 *
 * @warning 需确保输入的LocalRoute对象有效
 */
void LocalRouter::generateNavigationLaneChangeProperty() {
  std::vector<std::string> max_navigation_score_segments;
  const float navigation_route_score_thresold = 1.0;
  float max_navigation_score = std::numeric_limits<float>::lowest();
  for (auto& local_route : local_routes_) {
    local_route.second.setNavigationLaneChangeInfo(std::make_tuple<float, int, int>(-1.0, 0, 0));
    const auto& [valid, seg] = getSegmentBySegmentId(local_route.second.adcSegId());
    if (valid && seg.navigationLaneChangeDistance() >= 0.0) {
      std::get<0>(*local_route.second.mutableNavigationLaneChangeInfo()) =
          std::clamp(seg.length() - seg.adcS(), 0.0f, seg.length()) + seg.navigationLaneChangeDistance();
      if (seg.navigationScore() >= navigation_route_score_thresold && seg.navigationScore() >= max_navigation_score) {
        max_navigation_score = seg.navigationScore();
        max_navigation_score_segments.emplace_back(seg.id());
      }
    }
  }
  // distance_to_navi_lane_change, navi_lane_change_direction, navi_lane_change_num
  if (!max_navigation_score_segments.empty()) {
    for (auto& local_route : local_routes_) {
      auto res = judgeSegmentsNeighborRelation(local_route.second.adcSegId(), max_navigation_score_segments);
      if (std::get<0>(res) != 0) {
        std::get<1>(*local_route.second.mutableNavigationLaneChangeInfo()) = std::get<0>(res);
        std::get<2>(*local_route.second.mutableNavigationLaneChangeInfo()) = std::get<1>(res);
      }
    }
  }
}

/**
 * @brief 判断路段之间的邻接关系
 * @details 该函数用于判断目标路段与基础路段之间的邻接关系，包括变道方向和变道次数
 *
 * @par 处理流程:
 * 1. 初始化返回结果，默认变道方向为0（无变道），变道次数为0
 * 2. 向左搜索邻接路段，直到找到目标路段或无法继续搜索
 * 3. 如果找到目标路段，则更新变道方向和变道次数
 * 4. 向右搜索邻接路段，直到找到目标路段或无法继续搜索
 * 5. 如果找到目标路段，则更新变道方向和变道次数
 * 6. 返回最终的变道方向和变道次数
 *
 * @par 关键变量说明:
 * - base_seg_id: 基础路段ID
 * - target_seg_ids: 目标路段ID列表
 * - res: 返回结果，包含变道方向和变道次数
 *
 * @par 流程图:
 * @startuml
 * start
 * :初始化返回结果;
 * :向左搜索邻接路段;
 * if (找到目标路段) then (是)
 *   :更新变道方向和变道次数;
 * endif
 * :向右搜索邻接路段;
 * if (找到目标路段) then (是)
 *   :更新变道方向和变道次数;
 * endif
 * :返回最终结果;
 * stop
 * @enduml
 *
 * @note 该函数主要用于判断路段之间的邻接关系，帮助决策端进行变道决策
 *
 * @warning 需确保输入的路段ID有效
 */
std::tuple<int, int> LocalRouter::judgeSegmentsNeighborRelation(const std::string& base_seg_id,
                                                                const std::vector<std::string>& target_seg_ids) {
  std::tuple<int, int> res(0, 0);  // lane_change_direction(default: 0  left:1  right:-1), lane_change_num

  // left direction search
  std::string left_start_seg_id = base_seg_id;
  int left_search_num = 0;
  bool left_search_flag = true;
  bool left_search_success_flag = false;
  while (left_search_flag) {
    auto itr = std::find(target_seg_ids.begin(), target_seg_ids.end(), left_start_seg_id);
    if (itr != target_seg_ids.end()) {
      left_search_flag = false;
    } else {
      const auto& [valid, seg] = getSegmentBySegmentId(left_start_seg_id);
      if (valid) {
        if (seg.leftNeighborId() == "" || seg.leftNeighborId() == "0") {
          left_search_flag = false;
        } else {
          auto itr1 = std::find(target_seg_ids.begin(), target_seg_ids.end(), seg.leftNeighborId());
          if (itr1 != target_seg_ids.end()) {
            left_search_num++;
            left_start_seg_id = seg.leftNeighborId();
            left_search_flag = false;
            left_search_success_flag = true;
          } else {
            left_search_num++;
            left_start_seg_id = seg.leftNeighborId();
          }
        }
      } else {
        left_search_flag = false;
      }
    }
  }
  if (left_search_success_flag && left_search_num > 0) {
    std::get<0>(res) = 1;
    std::get<1>(res) = left_search_num;
    return res;
  }

  // right direction search
  std::string right_start_seg_id = base_seg_id;
  int right_search_num = 0;
  bool right_search_flag = true;
  bool right_search_success_flag = false;
  while (right_search_flag) {
    auto itr = std::find(target_seg_ids.begin(), target_seg_ids.end(), right_start_seg_id);
    if (itr != target_seg_ids.end()) {
      right_search_flag = false;
    } else {
      const auto& [valid, seg] = getSegmentBySegmentId(right_start_seg_id);
      if (valid) {
        if (seg.rightNeighborId() == "" || seg.rightNeighborId() == "0") {
          right_search_flag = false;
        } else {
          auto itr1 = std::find(target_seg_ids.begin(), target_seg_ids.end(), seg.rightNeighborId());
          if (itr1 != target_seg_ids.end()) {
            right_search_num++;
            right_start_seg_id = seg.rightNeighborId();
            right_search_flag = false;
            right_search_success_flag = true;
          } else {
            right_search_num++;
            right_start_seg_id = seg.rightNeighborId();
          }
        }
      } else {
        right_search_flag = false;
      }
    }
  }
  if (right_search_success_flag && right_search_num > 0) {
    std::get<0>(res) = -1;
    std::get<1>(res) = right_search_num;
    return res;
  }

  return res;
}

/**
 * @brief 判断路段是否满足运动学约束
 * @details 该函数用于判断给定路段是否满足车辆运动学约束，主要检查路段的转向半径是否满足要求
 *
 * @par 处理流程:
 * 1. 检查路段是否包含引导点
 * 2. 如果包含引导点，则计算路段起点和终点之间的距离
 * 3. 判断该距离是否大于车辆最小转向半径的两倍
 *
 * @par 关键变量说明:
 * - seg: 输入的LocalRouteSegment对象
 * - vehicle_turn_radius: 车辆最小转向半径，默认7.0米
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查路段是否包含引导点;
 * if (包含引导点) then (是)
 *   :计算路段起点和终点之间的距离;
 *   if (距离 > 2 * 车辆最小转向半径) then (是)
 *     :返回true;
 *   else (否)
 *     :返回false;
 *   endif
 * else (否)
 *   :返回false;
 * endif
 * stop
 * @enduml
 *
 * @note 该函数主要用于判断路段是否满足车辆运动学约束，确保车辆能够安全通过该路段
 *
 * @warning 需确保输入的LocalRouteSegment对象有效
 */
bool LocalRouter::IsSegmentMatchMotionConstraints(const LocalRouteSegment& seg) {
  float vehicle_turn_radius = 7.0;
  if (!seg.guidePoints().empty()) {
    return std::hypot(seg.guidePoints().front().x() - seg.guidePoints().back().x(),
                      seg.guidePoints().front().y() - seg.guidePoints().back().y()) > 2 * vehicle_turn_radius;
  }
  return false;
}

/**
 * @brief 根据路段ID获取路段信息
 * @details 该函数用于根据路段ID从车道与路段映射表中获取对应的路段信息
 *
 * @par 处理流程:
 * 1. 检查输入的路段ID是否有效
 * 2. 如果路段ID无效，则返回空结果并输出错误信息
 * 3. 遍历车道与路段映射表，查找与输入路段ID匹配的路段
 * 4. 如果找到匹配的路段，则返回该路段信息
 * 5. 如果未找到匹配的路段，则返回空结果
 *
 * @par 关键变量说明:
 * - seg_id: 输入的路段ID
 * - res: 返回结果，包含是否找到路段和路段信息
 * - lanes_segments_map_: 存储车道与路段映射的容器
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查路段ID是否有效;
 * if (路段ID无效) then (是)
 *   :返回空结果并输出错误信息;
 * else (否)
 *   :遍历车道与路段映射表;
 *   if (找到匹配的路段) then (是)
 *     :返回该路段信息;
 *   else (否)
 *     :返回空结果;
 *   endif
 * endif
 * stop
 * @enduml
 *
 * @note 该函数主要用于根据路段ID获取路段信息，帮助其他函数进行路段相关操作
 *
 * @warning 需确保输入的路段ID有效
 */
std::pair<bool, LocalRouteSegment> LocalRouter::getSegmentBySegmentId(const std::string seg_id) {
  std::pair<bool, LocalRouteSegment> res(false, LocalRouteSegment());
  if (seg_id == "0" || seg_id == "") {
    std::cerr << "[LocalRouter::getSegmentBySegmentId]: seg_id = " << seg_id
              << " Invalid, can not find valid local route segment! ";
    return res;
  }

  for (const auto& lane_segments_map : lanes_segments_map_) {
    auto itr = lane_segments_map.second.find(seg_id);
    if (itr != lane_segments_map.second.end()) {
      auto seg = itr->second;
      return std::make_pair<bool, LocalRouteSegment>(true, std::move(seg));
    }
  }
  return res;
}

/**
 * @brief 根据路段ID获取车道ID
 * @details 该函数用于根据路段ID从车道与路段映射表中获取对应的车道ID
 *
 * @par 处理流程:
 * 1. 检查输入的路段ID是否有效
 * 2. 如果路段ID无效，则返回空结果并输出错误信息
 * 3. 遍历车道与路段映射表，查找与输入路段ID匹配的车道
 * 4. 如果找到匹配的车道，则返回该车道ID
 * 5. 如果未找到匹配的车道，则返回空结果
 *
 * @par 关键变量说明:
 * - seg_id: 输入的路段ID
 * - res: 返回结果，包含是否找到车道和车道ID
 * - lanes_segments_map_: 存储车道与路段映射的容器
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查路段ID是否有效;
 * if (路段ID无效) then (是)
 *   :返回空结果并输出错误信息;
 * else (否)
 *   :遍历车道与路段映射表;
 *   if (找到匹配的车道) then (是)
 *     :返回该车道ID;
 *   else (否)
 *     :返回空结果;
 *   endif
 * endif
 * stop
 * @enduml
 *
 * @note 该函数主要用于根据路段ID获取车道ID，帮助其他函数进行车道相关操作
 *
 * @warning 需确保输入的路段ID有效
 */
std::pair<bool, std::string> LocalRouter::getLaneIdBySegmentId(const std::string seg_id) {
  std::pair<bool, std::string> res(false, "");
  if (seg_id == "0" || seg_id == "") {
    std::cerr << "[LocalRouter::getLaneIdBySegmentId]: seg_id = " << seg_id << " Invalid, can not find valid lane id! ";
    return res;
  }

  for (const auto& lane_segments_map : lanes_segments_map_) {
    auto itr = lane_segments_map.second.find(seg_id);
    if (itr != lane_segments_map.second.end()) {
      auto lane_id = lane_segments_map.first;
      return std::make_pair<bool, std::string>(true, std::move(lane_id));
    }
  }
  return res;
}

/**
 * @brief 根据路段ID获取关联的LocalRoute ID列表
 * @details 该函数用于根据路段ID从所有LocalRoute中查找包含该路段ID的LocalRoute，并返回其ID列表
 *
 * @par 处理流程:
 * 1. 检查输入的路段ID是否有效
 * 2. 如果路段ID无效，则返回空结果
 * 3. 遍历所有LocalRoute，查找其包含的车道和路段
 * 4. 如果找到匹配的路段ID，则将当前LocalRoute ID加入结果列表
 * 5. 返回包含匹配LocalRoute ID的列表
 *
 * @par 关键变量说明:
 * - seg_id: 输入的路段ID
 * - res: 返回结果，包含是否找到匹配的LocalRoute和LocalRoute ID列表
 * - local_routes_: 存储所有LocalRoute的容器
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查路段ID是否有效;
 * if (路段ID无效) then (是)
 *   :返回空结果;
 * else (否)
 *   :遍历所有LocalRoute;
 *   :查找包含该路段ID的LocalRoute;
 *   if (找到匹配的LocalRoute) then (是)
 *     :将LocalRoute ID加入结果列表;
 *   endif
 *   :返回结果列表;
 * endif
 * stop
 * @enduml
 *
 * @note 该函数主要用于根据路段ID查找关联的LocalRoute，帮助其他函数进行LocalRoute相关操作
 *
 * @warning 需确保输入的路段ID有效
 */
std::pair<bool, std::vector<int>> LocalRouter::getLocalRoutesIdBySegmentId(const std::string& seg_id) {
  std::pair<bool, std::vector<int>> res(false, std::vector<int>());
  if (seg_id == "0" || seg_id == "") {
    // std::cerr << "[LocalRouter::getLocalRoutesIdBySegmentId]: seg_id = "<< seg_id << " Invalid, can not find valid
    // local route id! ";
    return res;
  }

  for (const auto& local_route : local_routes_) {
    bool flag = false;
    for (const auto& lane_segs : local_route.second.lanesSegments()) {
      for (const auto& seg : lane_segs.second) {
        if (seg.id() == seg_id) {
          res.second.emplace_back(local_route.first);
          flag = true;
          break;
        }
      }
      if (flag) {
        break;
      }
    }
  }
  if (res.second.size() > 0) {
    res.first = true;
  }
  return res;
}

/**
 * @brief 根据路段ID获取关联的LocalRoute列表
 * @details 该函数用于根据路段ID从所有LocalRoute中查找包含该路段ID的LocalRoute，并返回其列表
 *
 * @par 处理流程:
 * 1. 检查输入的路段ID是否有效
 * 2. 如果路段ID无效，则返回空结果并输出错误信息
 * 3. 遍历所有LocalRoute，查找其包含的车道和路段
 * 4. 如果找到匹配的路段ID，则将当前LocalRoute加入结果列表
 * 5. 返回包含匹配LocalRoute的列表
 *
 * @par 关键变量说明:
 * - seg_id: 输入的路段ID
 * - res: 返回结果，包含是否找到匹配的LocalRoute和LocalRoute列表
 * - local_routes_: 存储所有LocalRoute的容器
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查路段ID是否有效;
 * if (路段ID无效) then (是)
 *   :返回空结果并输出错误信息;
 * else (否)
 *   :遍历所有LocalRoute;
 *   :查找包含该路段ID的LocalRoute;
 *   if (找到匹配的LocalRoute) then (是)
 *     :将LocalRoute加入结果列表;
 *   endif
 *   :返回结果列表;
 * endif
 * stop
 * @enduml
 *
 * @note 该函数主要用于根据路段ID查找关联的LocalRoute，帮助其他函数进行LocalRoute相关操作
 *
 * @warning 需确保输入的路段ID有效
 */
std::pair<bool, std::vector<LocalRoute>> LocalRouter::getLocalRoutesBySegmentId(const std::string& seg_id) {
  std::pair<bool, std::vector<LocalRoute>> res(false, std::vector<LocalRoute>());
  if (seg_id == "0" || seg_id == "") {
    std::cerr << "[LocalRouter::getLocalRoutesBySegmentId]: seg_id = " << seg_id
              << " Invalid, can not find valid local route! ";
    return res;
  }

  for (const auto& local_route : local_routes_) {
    bool flag = false;
    for (const auto& lane_segs : local_route.second.lanesSegments()) {
      for (const auto& seg : lane_segs.second) {
        if (seg.id() == seg_id) {
          res.second.emplace_back(local_route.second);
          flag = true;
          break;
        }
      }
      if (flag) {
        break;
      }
    }
  }
  if (res.second.size() > 0) {
    res.first = true;
  }
  return res;
}

/**
 * @brief 根据车道ID获取路段ID列表
 * @details 该函数用于根据车道ID从车道与路段映射表中获取对应的路段ID列表
 *
 * @par 处理流程:
 * 1. 检查输入的车道ID是否有效
 * 2. 如果车道ID无效，则返回空结果并输出错误信息
 * 3. 在车道与路段映射表中查找与输入车道ID匹配的条目
 * 4. 如果找到匹配的条目，则提取所有路段ID并返回
 * 5. 如果未找到匹配的条目，则返回空结果
 *
 * @par 关键变量说明:
 * - lane_id: 输入的车道ID
 * - res: 返回结果，包含是否找到路段和路段ID列表
 * - lanes_segments_map_: 存储车道与路段映射的容器
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查车道ID是否有效;
 * if (车道ID无效) then (是)
 *   :返回空结果并输出错误信息;
 * else (否)
 *   :在车道与路段映射表中查找;
 *   if (找到匹配的条目) then (是)
 *     :提取所有路段ID;
 *     :返回路段ID列表;
 *   else (否)
 *     :返回空结果;
 *   endif
 * endif
 * stop
 * @enduml
 *
 * @note 该函数主要用于根据车道ID获取路段ID列表，帮助其他函数进行路段相关操作
 *
 * @warning 需确保输入的车道ID有效
 */
std::pair<bool, std::vector<std::string>> LocalRouter::getSegmentsIdByLaneId(const std::string& lane_id) {
  std::pair<bool, std::vector<std::string>> res(false, std::vector<std::string>());
  if (lane_id == "0" || lane_id == "") {
    std::cerr << "[LocalRouter::getSegmentsIdByLaneId]: lane_id = " << lane_id
              << " Invalid, can not find valid segments id! ";
    return res;
  }

  auto itr = lanes_segments_map_.find(lane_id);
  if (itr != lanes_segments_map_.end()) {
    std::vector<std::string> segs_id;
    for (const auto& seg : itr->second) {
      segs_id.emplace_back(seg.first);
    }
    if (!segs_id.empty()) {
      return std::make_pair<bool, std::vector<std::string>>(true, std::move(segs_id));
    }
  }
  return res;
}

/**
 * @brief 根据车道ID获取路段列表
 * @details 该函数用于根据车道ID从车道与路段映射表中获取对应的路段列表
 *
 * @par 处理流程:
 * 1. 检查输入的车道ID是否有效
 * 2. 如果车道ID无效，则返回空结果并输出错误信息
 * 3. 在车道与路段映射表中查找与输入车道ID匹配的条目
 * 4. 如果找到匹配的条目，则提取所有路段并返回
 * 5. 如果未找到匹配的条目，则返回空结果
 *
 * @par 关键变量说明:
 * - lane_id: 输入的车道ID
 * - res: 返回结果，包含是否找到路段和路段列表
 * - lanes_segments_map_: 存储车道与路段映射的容器
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查车道ID是否有效;
 * if (车道ID无效) then (是)
 *   :返回空结果并输出错误信息;
 * else (否)
 *   :在车道与路段映射表中查找;
 *   if (找到匹配的条目) then (是)
 *     :提取所有路段;
 *     :返回路段列表;
 *   else (否)
 *     :返回空结果;
 *   endif
 * endif
 * stop
 * @enduml
 *
 * @note 该函数主要用于根据车道ID获取路段列表，帮助其他函数进行路段相关操作
 *
 * @warning 需确保输入的车道ID有效
 */
std::pair<bool, std::vector<LocalRouteSegment>> LocalRouter::getSegmentsByLaneId(const std::string& lane_id) {
  std::pair<bool, std::vector<LocalRouteSegment>> res(false, std::vector<LocalRouteSegment>());
  if (lane_id == "0" || lane_id == "") {
    std::cerr << "[LocalRouter::getSegmentsByLaneId]: lane_id = " << lane_id
              << " Invalid, can not find valid segments! ";
    return res;
  }

  auto itr = lanes_segments_map_.find(lane_id);
  if (itr != lanes_segments_map_.end()) {
    std::vector<LocalRouteSegment> segs;
    for (const auto& seg : itr->second) {
      segs.emplace_back(seg.second);
    }
    if (!segs.empty()) {
      return std::make_pair<bool, std::vector<LocalRouteSegment>>(true, std::move(segs));
    }
  }
  return res;
}

/**
 * @brief 根据车道ID获取关联的LocalRoute ID列表
 * @details 该函数用于根据车道ID从所有LocalRoute中查找包含该车道ID的LocalRoute，并返回其ID列表
 *
 * @par 处理流程:
 * 1. 检查输入的车道ID是否有效
 * 2. 如果车道ID无效，则返回空结果并输出错误信息
 * 3. 遍历所有LocalRoute，查找其包含的车道
 * 4. 如果找到匹配的车道ID，则将当前LocalRoute ID加入结果列表
 * 5. 返回包含匹配LocalRoute ID的列表
 *
 * @par 关键变量说明:
 * - lane_id: 输入的车道ID
 * - res: 返回结果，包含是否找到匹配的LocalRoute和LocalRoute ID列表
 * - local_routes_: 存储所有LocalRoute的容器
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查车道ID是否有效;
 * if (车道ID无效) then (是)
 *   :返回空结果并输出错误信息;
 * else (否)
 *   :遍历所有LocalRoute;
 *   :查找包含该车道ID的LocalRoute;
 *   if (找到匹配的LocalRoute) then (是)
 *     :将LocalRoute ID加入结果列表;
 *   endif
 *   :返回结果列表;
 * endif
 * stop
 * @enduml
 *
 * @note 该函数主要用于根据车道ID查找关联的LocalRoute，帮助其他函数进行LocalRoute相关操作
 *
 * @warning 需确保输入的车道ID有效
 */
std::pair<bool, std::vector<int>> LocalRouter::getLocalRoutesIdByLaneId(const std::string& lane_id) {
  std::pair<bool, std::vector<int>> res(false, std::vector<int>());

  if (lane_id == "0" || lane_id == "") {
    std::cerr << "[LocalRouter::getLocalRoutesIdByLaneId]: lane_id = " << lane_id
              << " Invalid, can not find valid local route id! ";
    return res;
  }

  for (const auto& local_route : local_routes_) {
    for (const auto& lane_segs : local_route.second.lanesSegments()) {
      if (lane_segs.first == lane_id) {
        res.first = true;
        res.second.emplace_back(local_route.first);
        break;
      }
    }
  }

  return res;
}

/**
 * @brief 根据车道ID获取关联的LocalRoute ID字符串列表
 * @details 该函数用于根据车道ID从所有LocalRoute中查找包含该车道ID的LocalRoute，并返回其ID的字符串列表
 *
 * @par 处理流程:
 * 1. 检查输入的车道ID是否有效
 * 2. 如果车道ID无效，则返回空结果并输出错误信息
 * 3. 遍历所有LocalRoute，查找其包含的车道
 * 4. 如果找到匹配的车道ID，则将当前LocalRoute ID转换为字符串并加入结果列表
 * 5. 返回包含匹配LocalRoute ID的字符串列表
 *
 * @par 关键变量说明:
 * - lane_id: 输入的车道ID
 * - res: 返回结果，包含是否找到匹配的LocalRoute和LocalRoute ID字符串列表
 * - local_routes_: 存储所有LocalRoute的容器
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查车道ID是否有效;
 * if (车道ID无效) then (是)
 *   :返回空结果并输出错误信息;
 * else (否)
 *   :遍历所有LocalRoute;
 *   :查找包含该车道ID的LocalRoute;
 *   if (找到匹配的LocalRoute) then (是)
 *     :将LocalRoute ID转换为字符串;
 *     :加入结果列表;
 *   endif
 *   :返回结果列表;
 * endif
 * stop
 * @enduml
 *
 * @note 该函数主要用于根据车道ID查找关联的LocalRoute，并返回其ID的字符串列表，帮助其他函数进行LocalRoute相关操作
 *
 * @warning 需确保输入的车道ID有效
 */
std::pair<bool, std::vector<std::string>> LocalRouter::getLocalRoutesStringIdByLaneId(const std::string& lane_id) {
  std::pair<bool, std::vector<std::string>> res(false, std::vector<std::string>());

  if (lane_id == "0" || lane_id == "") {
    std::cerr << "[LocalRouter::getLocalRoutesStringIdByLaneId]: lane_id = " << lane_id
              << " Invalid, can not find valid local route id! ";
    return res;
  }

  for (const auto& local_route : local_routes_) {
    for (const auto& lane_segs : local_route.second.lanesSegments()) {
      if (lane_segs.first == lane_id) {
        res.first = true;
        res.second.emplace_back(std::to_string(local_route.first));
        break;
      }
    }
  }

  return res;
}

/**
 * @brief 根据车道ID获取关联的LocalRoute列表
 * @details 该函数用于根据车道ID从所有LocalRoute中查找包含该车道ID的LocalRoute，并返回其列表
 *
 * @par 处理流程:
 * 1. 检查输入的车道ID是否有效
 * 2. 如果车道ID无效，则返回空结果并输出错误信息
 * 3. 遍历所有LocalRoute，查找其包含的车道
 * 4. 如果找到匹配的车道ID，则将当前LocalRoute加入结果列表
 * 5. 返回包含匹配LocalRoute的列表
 *
 * @par 关键变量说明:
 * - lane_id: 输入的车道ID
 * - res: 返回结果，包含是否找到匹配的LocalRoute和LocalRoute列表
 * - local_routes_: 存储所有LocalRoute的容器
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查车道ID是否有效;
 * if (车道ID无效) then (是)
 *   :返回空结果并输出错误信息;
 * else (否)
 *   :遍历所有LocalRoute;
 *   :查找包含该车道ID的LocalRoute;
 *   if (找到匹配的LocalRoute) then (是)
 *     :将LocalRoute加入结果列表;
 *   endif
 *   :返回结果列表;
 * endif
 * stop
 * @enduml
 *
 * @note 该函数主要用于根据车道ID查找关联的LocalRoute，帮助其他函数进行LocalRoute相关操作
 *
 * @warning 需确保输入的车道ID有效
 */
std::pair<bool, std::vector<LocalRoute>> LocalRouter::getLocalRoutesByLaneId(const std::string& lane_id) {
  std::pair<bool, std::vector<LocalRoute>> res(false, std::vector<LocalRoute>());

  if (lane_id == "0" || lane_id == "") {
    std::cerr << "[LocalRouter::getLocalRoutesByLaneId]: lane_id = " << lane_id
              << " Invalid, can not find valid local route! ";
    return res;
  }

  for (const auto& local_route : local_routes_) {
    for (const auto& lane_segs : local_route.second.lanesSegments()) {
      if (lane_segs.first == lane_id) {
        res.first = true;
        res.second.emplace_back(local_route.second);
        break;
      }
    }
  }

  return res;
}

/**
 * @brief 根据LocalRoute ID获取LocalRoute对象
 * @details 该函数用于根据LocalRoute ID从所有LocalRoute中查找对应的LocalRoute对象
 *
 * @par 处理流程:
 * 1. 在LocalRoute容器中查找与输入LocalRoute ID匹配的条目
 * 2. 如果找到匹配的LocalRoute，则返回该LocalRoute对象
 * 3. 如果未找到匹配的LocalRoute，则返回空结果
 *
 * @par 关键变量说明:
 * - local_route_id: 输入的LocalRoute ID
 * - res: 返回结果，包含是否找到LocalRoute和LocalRoute对象
 * - local_routes_: 存储所有LocalRoute的容器
 *
 * @par 流程图:
 * @startuml
 * start
 * :在LocalRoute容器中查找;
 * if (找到匹配的LocalRoute) then (是)
 *   :返回该LocalRoute对象;
 * else (否)
 *   :返回空结果;
 * endif
 * stop
 * @enduml
 *
 * @note 该函数主要用于根据LocalRoute ID获取LocalRoute对象，帮助其他函数进行LocalRoute相关操作
 *
 * @warning 需确保输入的LocalRoute ID有效
 */
std::pair<bool, LocalRoute> LocalRouter::getLocalRouteByLocalRouteId(const int& local_route_id) {
  auto itr = local_routes_.find(local_route_id);
  if (itr != local_routes_.end()) {
    auto local_route = itr->second;
    return std::make_pair<bool, LocalRoute>(true, std::move(local_route));
  }
  return std::make_pair<bool, LocalRoute>(false, LocalRoute());
}

}  // namespace gpal::pnc::planning
