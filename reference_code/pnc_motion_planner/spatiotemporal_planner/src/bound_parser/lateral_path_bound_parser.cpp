/**
 * @file lateral_path_bound_parser.cpp
 * @brief 路径边界解析器实现文件
 * @details
 * 该文件实现了路径边界解析器的核心功能，包括路径边界的初始化、静态和动态障碍物的边界信息获取等。
 * 路径边界解析器用于在路径规划过程中处理障碍物对路径边界的影响，确保路径的安全性和可行性。
 */

#include "bound_parser/lateral_path_bound_parser.h"

namespace gpal::pnc::planning {

bool LateralPathBoundParser::init() {
  return true;  // TODO: Implement actual initialization logic
}

/**
 * @brief 初始化路径边界解析器
 * @details
 * 该函数用于重置路径边界解析器，根据参考线、地图到自车的变换矩阵以及路径的起始和结束位置，生成采样点并计算自车坐标系下的采样点信息。
 *
 * @param[in] ref_line 参考线，用于获取路径上的参考点
 * @param[in] start_s 路径的起始位置，单位为米
 * @param[in] end_s 路径的结束位置，单位为米
 *
 * @par 关键变量说明:
 * - check_ranges_: 检查范围，包含路径的起始和结束位置以及最大范围
 * - bound_resolution: 边界分辨率，用于确定采样点的间隔
 * - bound_segment_size: 边界段的数量，根据路径长度和分辨率计算
 * - sample_points_: 采样点集合，包含路径上的参考点
 * - ego_sample_points_: 自车坐标系下的采样点集合
 * - sample_segments_: 采样段集合，包含采样点之间的线段
 *
 * @par 处理流程:
 * 1. 如果检查范围为空，则初始化检查范围
 * 2. 计算路径长度和边界分辨率
 * 3. 根据路径长度和分辨率计算边界段的数量
 * 4. 生成采样点，并将地图坐标系下的点转换到自车坐标系
 * 5. 生成采样段，包含采样点之间的线段
 *
 * @par 流程图:
 * @startuml
 * start
 * :重置检查范围;
 * :计算路径长度和边界分辨率;
 * :计算边界段的数量;
 * :生成采样点;
 * :将采样点转换到自车坐标系;
 * :生成采样段;
 * stop
 * @enduml
 *
 * @return bool 重置结果，true表示成功，false表示失败
 *
 * @note 该函数应在路径边界解析器初始化时调用，确保采样点和采样段正确生成
 *
 * @warning 需确保传入的参考线、起始位置和结束位置有效且合理
 */
bool LateralPathBoundParser::reset(const ReferenceLine& ref_line, const double& start_s, const double& end_s,
                                   const double& resolution) {
  double length = end_s - start_s;
  resolution_ = resolution;
  int bound_segment_size = std::max<int>(1, std::round(length / resolution_) + 1e-10);
  auto bound_resolution = length / bound_segment_size;
  sample_points_.clear();
  for (int i = 0; i <= bound_segment_size; i++) {
    double curr_s = start_s + i * bound_resolution;
    sample_points_.emplace_back(ref_line.getReferencePoint(curr_s));
    sample_points_.back().setLocalS(curr_s);
  }
  sample_segments_.clear();
  for (const auto& sample_pt : sample_points_) {
    math::Vec3d unit_direction(cos(sample_pt.heading()), sin(sample_pt.heading()), 0.0);
    sample_segments_.emplace_back(sample_pt, sample_pt + bound_resolution * unit_direction);
  }
  return sample_points_.size() > 0;
}

/**
 * @brief 从静态障碍物获取边界信息
 * @details 该函数用于从静态障碍物中获取路径边界信息，包括左右边界的位置以及对应的障碍物信息，同时考虑缓冲的影响。
 *
 * @param[in] decision_object 静态障碍物集合，包含所有需要处理的静态障碍物
 * @param[in] buffer 缓冲区参数集合，包含每个障碍物的起始缓冲、结束缓冲和横向缓冲
 *
 * @par 关键变量说明:
 * - res: 返回的障碍物边界信息，包含边界位置和对应的障碍物
 * - buffer_map: 障碍物与缓冲的映射关系
 * - small_objects: 小型障碍物集合
 * - left_ranges: 左侧障碍物的范围信息
 * - right_ranges: 右侧障碍物的范围信息
 * - left_od_bounds: 左侧障碍物的边界范围信息
 * - right_od_bounds: 右侧障碍物的边界范围信息
 * - left_objects: 左侧障碍物的信息集合
 * - right_objects: 右侧障碍物的信息集合
 *
 * @par 处理流程:
 * 1. 获取障碍物与缓冲的映射关系
 * 2. 对静态障碍物进行排序，分为左侧和右侧障碍物，并筛选出小型障碍物
 * 3. 对小型障碍物的边界范围进行排序
 * 4. 遍历采样点，更新左右边界信息
 * 5. 根据障碍物位置和缓冲调整边界范围
 * 6. 对左右边界范围进行排序
 * 7. 遍历采样点，生成最终的边界信息
 *
 * @par 流程图:
 * @startuml
 * start
 * :获取障碍物与缓冲的映射关系;
 * :对静态障碍物进行排序;
 * :对小型障碍物的边界范围进行排序;
 * :遍历采样点;
 * :更新左右边界信息;
 * :根据障碍物位置和缓冲调整边界范围;
 * :对左右边界范围进行排序;
 * :生成最终的边界信息;
 * stop
 * @enduml
 *
 * @return  bool 返回结果，true表示成功获取边界信息，false表示失败
 *
 * @note 该函数应在路径规划过程中调用，确保边界信息正确生成
 *
 * @warning 需确保传入的静态障碍物集合和缓冲参数有效
 */
bool LateralPathBoundParser::getBoundaryFromStaticObjects(
    const std::vector<std::shared_ptr<Decision::DecisionObject>>& decision_object,
    const std::vector<std::tuple<double, double, double>>& buffer, std::vector<Boundary>& boundaries) {
  auto buffer_map = getObjectBufferMap(decision_object, buffer);
  std::vector<std::shared_ptr<Decision::DecisionObject>> small_objects;
  auto [left_ranges, right_ranges] = sortStaticObjectRange(decision_object, 2 * resolution_, &small_objects);
  STLOG(D, "[LateralPathBoundParser::getBoundaryFromStaticObjects] left_ranges size: ", left_ranges.size(),
        ", right_ranges size: ", right_ranges.size(), ", small_objects size: ", small_objects.size());
  auto [left_od_bounds, right_od_bounds] =
      sortStaticObjectBoundRange(small_objects, buffer_map);  // buffer: start_buffer, end_buffer, lat_buffer

  int curr_idx_left = 0, curr_idx_right = 0;
  StaticObjectInfoMap left_objects, right_objects;
  for (int i = 0; i + 1 < sample_points_.size(); i++) {
    double curr_s = sample_points_[i].local_s();
    double next_s = sample_points_[i + 1].local_s();

    // 更新左侧障碍物
    while (curr_idx_left < left_ranges.size()) {
      auto& [in_bound, s, ptr_obs, idx] = left_ranges[curr_idx_left];
      if (next_s < s) {
        break;
      }
      if (in_bound) {
        left_objects.emplace(ptr_obs, getStaticObjectInfo(*ptr_obs, std::get<2>(buffer[idx])));  // segments, lat_buffer
      } else {
        left_objects.erase(ptr_obs);
      }
      curr_idx_left++;
    }

    // 更新右侧障碍物
    while (curr_idx_right < right_ranges.size()) {
      auto& [in_bound, s, ptr_obs, idx] = right_ranges[curr_idx_right];
      if (next_s < s) {
        break;
      }
      if (in_bound) {
        right_objects.emplace(ptr_obs, getStaticObjectInfo(*ptr_obs, std::get<2>(buffer[idx])));
      } else {
        right_objects.erase(ptr_obs);
      }
      curr_idx_right++;
    }

    // 计算边界
    const auto& [right_in_bound, right_offset] = getRightOffsetInfoFromStaticObject(sample_segments_[i], right_objects);
    if (right_in_bound) {
      const auto& [start_buffer, end_buffer, lat_buffer] = buffer_map[right_in_bound];
      right_od_bounds.emplace_back(1, curr_s - start_buffer, right_offset, right_in_bound);
      right_od_bounds.emplace_back(0, next_s + end_buffer, right_offset, right_in_bound);
    }
    const auto& [left_in_bound, left_offset] = getLeftOffsetInfoFromStaticObject(sample_segments_[i], left_objects);
    if (left_in_bound) {
      const auto& [start_buffer, end_buffer, lat_buffer] = buffer_map[left_in_bound];
      left_od_bounds.emplace_back(1, curr_s - start_buffer, left_offset, left_in_bound);
      left_od_bounds.emplace_back(0, next_s + end_buffer, left_offset,
                                  left_in_bound);  // 一系列的头和尾，而不是只有一个
    }
  }

  auto sorter = [](const ObjectBoundRange& lhs, const ObjectBoundRange& rhs) {
    if (std::get<1>(lhs) != std::get<1>(rhs)) {
      return std::get<1>(lhs) < std::get<1>(rhs);
    }
    return std::get<0>(lhs) > std::get<0>(rhs);
  };
  std::sort(right_od_bounds.begin(), right_od_bounds.end(), sorter);
  std::sort(left_od_bounds.begin(), left_od_bounds.end(), sorter);

  std::multimap<double, const Decision::DecisionObject*, std::greater<double>> right_bounds;
  right_bounds.emplace(std::numeric_limits<double>::lowest(), nullptr);
  std::multimap<double, const Decision::DecisionObject*> left_bounds;
  left_bounds.emplace(std::numeric_limits<double>::max(), nullptr);
  int right_obs_idx = 0, left_obs_idx = 0;
  for (int i = 0; i + 1 < boundaries.size(); i++) {
    double curr_s = boundaries[i].s();
    while (right_obs_idx < right_od_bounds.size() && std::get<1>(right_od_bounds[right_obs_idx]) < curr_s) {
      auto& [in_bound, obs_s, obs_l, ptr_obs] = right_od_bounds[right_obs_idx];
      if (in_bound) {
        right_bounds.emplace(obs_l, ptr_obs);
      } else {
        auto it = right_bounds.find(obs_l);
        if (it != right_bounds.end()) {
          right_bounds.erase(it);
        }
      }
      ++right_obs_idx;
    }
    while (left_obs_idx < left_od_bounds.size() && std::get<1>(left_od_bounds[left_obs_idx]) < curr_s) {
      auto& [in_bound, obs_s, obs_l, ptr_obs] = left_od_bounds[left_obs_idx];
      if (in_bound) {
        // DecisionObject is to the right of center-line, should pass from left.
        left_bounds.emplace(obs_l, ptr_obs);
      } else {
        auto it = left_bounds.find(obs_l);
        if (it != left_bounds.end()) {
          left_bounds.erase(it);
        }
      }
      ++left_obs_idx;
    }

    double bound_left = profile_.max_range();
    double bound_right = -profile_.max_range();

    if (!right_bounds.empty()) {
      bound_right = right_bounds.begin()->first;
    }
    if (!left_bounds.empty()) {
      bound_left = left_bounds.begin()->first;
    }
    if (bound_right > bound_left) {
      // 阻塞情况：右边界在左边界的左边，这是物理上不可能通过的
      continue;  // 继续处理下一个点
    }
    boundaries[i].clip(std::max(boundaries[i].lower(), bound_right), std::min(boundaries[i].upper(), bound_left));
  }
  return true;
}

/**
 * @brief 对静态障碍物进行排序并分类
 * @details 该函数用于对静态障碍物进行排序，并根据其位置和类型将其分为左侧和右侧障碍物。同时，可以筛选出小型障碍物。
 *
 * @param[in] decision_object 静态障碍物集合，包含所有需要处理的静态障碍物
 * @param[in] resolution_thrd 分辨率阈值，用于判断障碍物是否为小型障碍物
 * @param[out] ptr_small 小型障碍物集合，用于存储筛选出的小型障碍物
 *
 * @par 关键变量说明:
 * - res: 返回的障碍物范围信息，包含左侧和右侧障碍物的范围
 * - left_ranges: 左侧障碍物的范围信息
 * - right_ranges: 右侧障碍物的范围信息
 * - end_point: 路径的结束点，用于计算障碍物的投影
 * - unit_vec: 路径结束点的单位向量，用于计算障碍物的投影
 * - near_dest_thresold: 接近终点的阈值，用于判断障碍物是否在终点附近
 * - projection: 障碍物在路径上的投影距离
 * - out_roi: 判断障碍物是否在感兴趣区域外
 * - nudge_tag: 障碍物的绕行标签，用于判断障碍物是左侧绕行还是右侧绕行
 *
 * @par 处理流程:
 * 1. 初始化返回结果和路径结束点的单位向量
 * 2. 遍历所有障碍物，计算其在路径上的投影
 * 3. 判断障碍物是否在感兴趣区域内
 * 4. 根据障碍物的绕行标签将其分类为左侧或右侧障碍物
 * 5. 如果障碍物长度小于分辨率阈值，则将其归类为小型障碍物
 * 6. 对左侧和右侧障碍物范围进行排序
 *
 * @par 流程图:
 * @startuml
 * start
 * :初始化返回结果和路径结束点的单位向量;
 * :遍历所有障碍物;
 * :计算障碍物在路径上的投影;
 * :判断障碍物是否在感兴趣区域内;
 * :根据绕行标签分类障碍物;
 * if (障碍物长度 < 分辨率阈值) then (yes)
 *   :归类为小型障碍物;
 * endif
 * :对左侧和右侧障碍物范围进行排序;
 * stop
 * @enduml
 *
 * @return std::pair<std::vector<StaticObjectRange>, std::vector<StaticObjectRange>>
 * 返回包含左侧和右侧障碍物范围信息的结构体
 *
 * @note 该函数应在路径规划过程中调用，确保障碍物正确分类和排序
 *
 * @warning 需确保传入的静态障碍物集合有效
 */
std::pair<std::vector<LateralPathBoundParser::StaticObjectRange>,
          std::vector<LateralPathBoundParser::StaticObjectRange>>
LateralPathBoundParser::sortStaticObjectRange(
    const std::vector<std::shared_ptr<Decision::DecisionObject>>& decision_object, const double resolution_thrd,
    std::vector<std::shared_ptr<Decision::DecisionObject>>* ptr_small) {
  std::pair<std::vector<StaticObjectRange>, std::vector<StaticObjectRange>> res;
  auto& [left_ranges, right_ranges] = res;  // each range: is in range, start_s or end_s, object_ptr, index

  ReferencePoint end_point = sample_points_.back();
  const double heading = end_point.heading();
  math::Vec2d unit_vec(cos(heading), sin(heading));
  constexpr double near_dest_thresold = 0.1;
  for (int i = 0; i < decision_object.size(); i++) {
    const auto& object = decision_object[i];
    // ignore the objects which far from the sample_points_
    double projection = std::numeric_limits<double>::max();
    for (const auto& point : object->cur_box.GetAllCorners()) {
      math::Vec2d vec(point.x() - end_point.x(), point.y() - end_point.y());
      if (unit_vec.InnerProd(vec) < projection) {
        projection = unit_vec.InnerProd(vec);
      }
    }

    bool out_roi = false;
    if (std::abs(object->cur_sl_bound.start_s() - end_point.local_s()) < near_dest_thresold
        && projection > profile_.roi_threshold()) {
      out_roi = true;
    }

    if (object->cur_sl_bound.start_s() > end_point.local_s() || out_roi) {
      continue;
    }
    // Only focus on objects that are ahead of Adc.
    if (object->cur_sl_bound.end_s() < sample_points_.front().local_s()) {
      continue;
    }
    const LateralOdTag nudge_tag = object->lat_od_tag;

    if (object->cur_sl_bound.end_s() - object->cur_sl_bound.start_s() > resolution_thrd) {
      switch (nudge_tag) {
        case LateralOdTag::LEFT_BYPASS: {
          right_ranges.emplace_back(1, object->cur_sl_bound.start_s(), object.get(), i);
          right_ranges.emplace_back(0, object->cur_sl_bound.end_s(), object.get(), i);

        } break;
        case LateralOdTag::RIGHT_BYPASS: {
          left_ranges.emplace_back(1, object->cur_sl_bound.start_s(), object.get(), i);
          left_ranges.emplace_back(0, object->cur_sl_bound.end_s(), object.get(), i);
        } break;
      }
    } else if (ptr_small != nullptr) {
      ptr_small->emplace_back(object);
    }
  }

  auto sorter = [](const StaticObjectRange& lhs, const StaticObjectRange& rhs) {  // ture不调换
    if (std::get<1>(lhs) != std::get<1>(rhs)) {
      return std::get<1>(lhs) < std::get<1>(rhs);
    }
    return std::get<0>(lhs) > std::get<0>(rhs);
  };
  std::sort(right_ranges.begin(), right_ranges.end(), sorter);
  std::sort(left_ranges.begin(), left_ranges.end(), sorter);
  return res;
}

/**
 * @brief 对静态障碍物的边界范围进行排序
 * @details
 * 该函数用于对静态障碍物的边界范围进行排序，并根据障碍物的绕行标签（LEFT_BYPASS或RIGHT_BYPASS）将其分为左侧和右侧边界范围。
 *
 * @param[in] decision_object 静态障碍物集合，包含所有需要处理的静态障碍物
 * @param[in] buffer_map 障碍物与缓冲区的映射关系，包含每个障碍物的起始缓冲区、结束缓冲区和横向缓冲区
 *
 * @par 关键变量说明:
 * - res: 返回的障碍物边界范围信息，包含左侧和右侧障碍物的边界范围
 * - left_ranges: 左侧障碍物的边界范围信息
 * - right_ranges: 右侧障碍物的边界范围信息
 * - start_buffer: 障碍物的起始缓冲区
 * - end_buffer: 障碍物的结束缓冲区
 * - lat_buffer: 障碍物的横向缓冲区
 * - nudge_tag: 障碍物的绕行标签，用于判断障碍物是左侧绕行还是右侧绕行
 *
 * @par 处理流程:
 * 1. 遍历所有障碍物，获取其缓冲区信息
 * 2. 判断障碍物是否在路径的起始和结束位置之间
 * 3. 根据障碍物的绕行标签将其分类为左侧或右侧边界范围
 * 4. 对左侧和右侧边界范围进行排序
 *
 * @par 流程图:
 * @startuml
 * start
 * :遍历所有障碍物;
 * :获取障碍物的缓冲区信息;
 * :判断障碍物是否在路径的起始和结束位置之间;
 * :根据绕行标签分类障碍物边界范围;
 * :对左侧和右侧边界范围进行排序;
 * stop
 * @enduml
 *
 * @return std::pair<std::vector<ObjectBoundRange>, std::vector<ObjectBoundRange>>
 * 返回包含左侧和右侧障碍物边界范围信息的结构体
 *
 * @note 该函数应在路径规划过程中调用，确保障碍物边界范围正确分类和排序
 *
 * @warning 需确保传入的静态障碍物集合和缓冲区映射关系有效
 */
std::pair<std::vector<LateralPathBoundParser::ObjectBoundRange>, std::vector<LateralPathBoundParser::ObjectBoundRange>>
LateralPathBoundParser::sortStaticObjectBoundRange(
    const std::vector<std::shared_ptr<Decision::DecisionObject>>& decision_object,
    const std::unordered_map<const Decision::DecisionObject*, std::tuple<double, double, double>>& buffer_map) {
  std::pair<std::vector<ObjectBoundRange>, std::vector<ObjectBoundRange>> res;
  auto& [left_ranges, right_ranges] = res;  // each range: is in range, start_s or end_s, offset, object_ptr

  for (int i = 0; i < decision_object.size(); i++) {
    const auto& object = decision_object[i];
    const auto& [start_buffer, end_buffer, lat_buffer] = buffer_map.at(object.get());
    if (object->cur_sl_bound.start_s() - start_buffer > sample_points_.back().local_s()) {
      continue;
    }
    // Only focus on objects that are ahead of Adc.
    if (object->cur_sl_bound.end_s() + end_buffer < sample_points_.front().local_s()) {
      continue;
    }

    const planning::LateralOdTag nudge_tag = object->lat_od_tag;

    switch (nudge_tag) {
      case LateralOdTag::LEFT_BYPASS: {
        right_ranges.emplace_back(1, object->cur_sl_bound.start_s() - start_buffer,
                                  object->cur_sl_bound.end_l() + lat_buffer, object.get());
        right_ranges.emplace_back(0, object->cur_sl_bound.end_s() + end_buffer,
                                  object->cur_sl_bound.end_l() + lat_buffer, object.get());

      } break;
      case LateralOdTag::RIGHT_BYPASS: {
        left_ranges.emplace_back(1, object->cur_sl_bound.start_s() - start_buffer,
                                 object->cur_sl_bound.start_l() - lat_buffer, object.get());
        left_ranges.emplace_back(0, object->cur_sl_bound.end_s() + end_buffer,
                                 object->cur_sl_bound.start_l() - lat_buffer, object.get());
      } break;
    }
  }

  auto sorter = [](const ObjectBoundRange& lhs, const ObjectBoundRange& rhs) {
    if (std::get<1>(lhs) != std::get<1>(rhs)) {
      return std::get<1>(lhs) < std::get<1>(rhs);
    }
    return std::get<0>(lhs) > std::get<0>(rhs);
  };
  std::sort(right_ranges.begin(), right_ranges.end(), sorter);
  std::sort(left_ranges.begin(), left_ranges.end(), sorter);
  return res;
}

/**
 * @brief 获取静态障碍物的信息
 * @details
 * 该函数用于获取静态障碍物的几何信息，包括障碍物的所有角点以及由这些角点构成的线段集合。同时，可以指定横向缓冲区大小。
 *
 * @param[in] static_object 静态障碍物对象，包含障碍物的几何信息
 * @param[in] lat_buffer 横向缓冲区大小，用于调整障碍物的边界
 *
 * @par 关键变量说明:
 * - points: 障碍物的所有角点集合
 * - segments: 由障碍物角点构成的线段集合
 *
 * @par 处理流程:
 * 1. 获取障碍物的所有角点
 * 2. 根据角点生成障碍物的边界线段
 * 3. 返回包含线段集合和横向缓冲区的障碍物信息
 *
 * @par 流程图:
 * @startuml
 * start
 * :获取障碍物的所有角点;
 * :根据角点生成障碍物的边界线段;
 * :返回包含线段集合和横向缓冲区的障碍物信息;
 * stop
 * @enduml
 *
 * @return StaticObjectInfo 返回包含障碍物线段集合和横向缓冲区的结构体
 *
 * @note 该函数应在处理静态障碍物时调用，确保获取到障碍物的几何信息
 *
 * @warning 需确保传入的静态障碍物对象有效
 */
LateralPathBoundParser::StaticObjectInfo LateralPathBoundParser::getStaticObjectInfo(
    const Decision::DecisionObject& static_object, const double lat_buffer) {
  const std::vector<math::Vec2d>& points = static_object.cur_box.GetAllCorners();
  std::vector<math::LineSegment2d> segments;
  if (points.size() > 1) {
    for (int i = 0; i + 1 < points.size(); i++) {
      segments.emplace_back(points[i], points[i + 1]);
    }
    segments.emplace_back(points.back(), points.front());
  }
  return StaticObjectInfo(segments, lat_buffer);  // std::tuple<std::vector<math::LineSegment2d>, double>;
}

/**
 * @brief 从静态障碍物获取左侧偏移信息
 * @details
 * 该函数用于从静态障碍物中获取左侧偏移信息，包括障碍物对象和对应的偏移量。通过计算障碍物边界与中心线的距离，确定是否需要左侧偏移。
 *
 * @param[in] center 中心线段，表示当前路径的中心线
 * @param[in] object_borders 障碍物边界信息集合，包含障碍物的线段集合和横向缓冲区
 *
 * @par 关键变量说明:
 * - res: 返回的障碍物偏移信息，包含障碍物对象和对应的偏移量
 * - in_bound_obs: 需要左侧偏移的障碍物对象
 * - offset: 左侧偏移量
 * - left_offset_enable_radius: 左侧偏移的启用半径，用于判断障碍物是否在偏移范围内
 * - segments: 障碍物的边界线段集合
 * - lat_buffer: 障碍物的横向缓冲区
 * - dis_point_to_line: 障碍物边界线段到中心线的距离
 * - enable_left_offset: 是否启用左侧偏移的标志
 * - min_offset: 障碍物边界线段到中心线的最小偏移量
 * - max_offset: 障碍物边界线段到中心线的最大偏移量
 * - in_range: 障碍物边界线段是否在中心线的偏移范围内
 * - obs_offset: 障碍物的偏移量
 *
 * @par 处理流程:
 * 1. 遍历所有障碍物边界信息
 * 2. 判断障碍物边界线段是否在左侧偏移的启用半径内
 * 3. 如果启用左侧偏移，则计算障碍物边界线段到中心线的偏移量
 * 4. 如果偏移量小于当前最小偏移量，则更新最小偏移量和对应的障碍物对象
 *
 * @par 流程图:
 * @startuml
 * start
 * :遍历所有障碍物边界信息;
 * :判断障碍物边界线段是否在左侧偏移的启用半径内;
 * if (启用左侧偏移) then (yes)
 *   :计算障碍物边界线段到中心线的偏移量;
 *   if (偏移量 < 当前最小偏移量) then (yes)
 *     :更新最小偏移量和对应的障碍物对象;
 *   endif
 * endif
 * stop
 * @enduml
 *
 * @return std::tuple<const Decision::DecisionObject*, double> 返回包含障碍物对象和对应偏移量的结构体
 *
 * @note 该函数应在路径规划过程中调用，确保左侧偏移信息正确生成
 *
 * @warning 需确保传入的中心线段和障碍物边界信息集合有效
 */
std::tuple<const Decision::DecisionObject*, double> LateralPathBoundParser::getLeftOffsetInfoFromStaticObject(
    const math::LineSegment2d& center, const StaticObjectInfoMap& object_borders) {
  std::tuple<const Decision::DecisionObject*, double> res{nullptr, std::numeric_limits<double>::max()};
  auto& [in_bound_obs, offset] = res;
  double left_offset_enable_radius = 10;
  for (const auto& [ptr_obs, obs_info] : object_borders) {  // object_borders: std::map<const DecisionObject*,
                                                            // std::tuple<std::vector<math::LineSegment2d>, double>>
    const auto& [segments, lat_buffer] = obs_info;
    bool enable_left_offset = false;
    for (const auto& segment : segments) {
      float dis_point_to_line = CalculatePointDistanceToLineSegment(
          segment,
          math::Vec2d((center.start().x() + center.end().x()) / 2.0, (center.start().y() + center.end().y()) / 2.0));
      if (dis_point_to_line < left_offset_enable_radius) {
        enable_left_offset = true;
        break;
      }
    }
    if (!enable_left_offset) {
      continue;
    }
    for (const auto& segment : segments) {
      const auto& [min_offset, max_offset, in_range] = getOffset(center, segment);
      double obs_offset = min_offset - lat_buffer;
      if (in_range && obs_offset < offset) {
        in_bound_obs = ptr_obs;
        offset = obs_offset;
      }
    }
  }
  return res;
}

/**
 * @brief 从静态障碍物获取右侧偏移信息
 * @details
 * 该函数用于从静态障碍物中获取右侧偏移信息，包括障碍物对象和对应的偏移量。通过计算障碍物边界与中心线的距离，确定是否需要右侧偏移。
 *
 * @param[in] center 中心线段，表示当前路径的中心线
 * @param[in] object_borders 障碍物边界信息集合，包含障碍物的线段集合和横向缓冲区
 *
 * @par 关键变量说明:
 * - res: 返回的障碍物偏移信息，包含障碍物对象和对应的偏移量
 * - in_bound_obs: 需要右侧偏移的障碍物对象
 * - offset: 右侧偏移量
 * - right_offset_enable_radius: 右侧偏移的启用半径，用于判断障碍物是否在偏移范围内
 * - segments: 障碍物的边界线段集合
 * - lat_buffer: 障碍物的横向缓冲区
 * - dis_point_to_line: 障碍物边界线段到中心线的距离
 * - enable_right_offset: 是否启用右侧偏移的标志
 * - min_offset: 障碍物边界线段到中心线的最小偏移量
 * - max_offset: 障碍物边界线段到中心线的最大偏移量
 * - in_range: 障碍物边界线段是否在中心线的偏移范围内
 * - obs_offset: 障碍物的偏移量
 *
 * @par 处理流程:
 * 1. 遍历所有障碍物边界信息
 * 2. 判断障碍物边界线段是否在右侧偏移的启用半径内
 * 3. 如果启用右侧偏移，则计算障碍物边界线段到中心线的偏移量
 * 4. 如果偏移量大于当前最大偏移量，则更新最大偏移量和对应的障碍物对象
 *
 * @par 流程图:
 * @startuml
 * start
 * :遍历所有障碍物边界信息;
 * :判断障碍物边界线段是否在右侧偏移的启用半径内;
 * if (启用右侧偏移) then (yes)
 *   :计算障碍物边界线段到中心线的偏移量;
 *   if (偏移量 > 当前最大偏移量) then (yes)
 *     :更新最大偏移量和对应的障碍物对象;
 *   endif
 * endif
 * stop
 * @enduml
 *
 * @return std::tuple<const Decision::DecisionObject*, double> 返回包含障碍物对象和对应偏移量的结构体
 *
 * @note 该函数应在路径规划过程中调用，确保右侧偏移信息正确生成
 *
 * @warning 需确保传入的中心线段和障碍物边界信息集合有效
 */
std::tuple<const Decision::DecisionObject*, double> LateralPathBoundParser::getRightOffsetInfoFromStaticObject(
    const math::LineSegment2d& center, const StaticObjectInfoMap& object_borders) {
  std::tuple<const Decision::DecisionObject*, double> res{nullptr, std::numeric_limits<double>::lowest()};
  auto& [in_bound_obs, offset] = res;
  for (const auto& [ptr_obs, obs_info] : object_borders) {  // object_borders: std::map<const DecisionObject*,
                                                            // std::tuple<std::vector<math::LineSegment2d>, double>>
    const auto& [segments, lat_buffer] = obs_info;
    for (const auto& segment : segments) {
      const auto& [min_offset, max_offset, in_range] = getOffset(center, segment);
      double obs_offset = max_offset + lat_buffer;
      if (in_range && obs_offset > offset) {
        in_bound_obs = ptr_obs;
        offset = obs_offset;
      }
    }
  }
  return res;
}

/**
 * @brief 计算线段与边界线段的偏移量
 * @details 该函数用于计算给定线段与边界线段之间的偏移量，包括最小偏移量、最大偏移量以及是否存在偏移。
 *
 * @param[in] segment 当前线段，表示路径的中心线或参考线
 * @param[in] bound_segment 边界线段，表示障碍物的边界
 *
 * @par 关键变量说明:
 * - res: 返回的偏移量信息，包含最小偏移量、最大偏移量和是否存在偏移
 * - min_offset: 最小偏移量
 * - max_offset: 最大偏移量
 * - has_offset: 是否存在偏移的标志
 * - dx1, dy1: 边界线段起点到当前线段起点的向量
 * - x1, y1: 边界线段起点在当前线段局部坐标系中的坐标
 * - dx2, dy2: 边界线段终点到当前线段起点的向量
 * - x2, y2: 边界线段终点在当前线段局部坐标系中的坐标
 * - gx1, gx2: 边界线段起点和终点在当前线段上的投影位置标志
 * - k: 边界线段在当前线段上的斜率
 *
 * @par 处理流程:
 * 1. 初始化返回结果
 * 2. 计算边界线段起点和终点在当前线段局部坐标系中的坐标
 * 3. 判断边界线段是否与当前线段有重叠
 * 4. 如果有重叠，则计算最小和最大偏移量
 * 5. 根据边界线段的位置调整偏移量
 * 6. 确保最小偏移量不大于最大偏移量
 *
 * @par 流程图:
 * @startuml
 * start
 * :初始化返回结果;
 * :计算边界线段起点和终点在当前线段局部坐标系中的坐标;
 * :判断边界线段是否与当前线段有重叠;
 * if (有重叠) then (yes)
 *   :计算最小和最大偏移量;
 *   :根据边界线段的位置调整偏移量;
 *   :确保最小偏移量不大于最大偏移量;
 * endif
 * stop
 * @enduml
 *
 * @return std::tuple<double, double, bool> 返回包含最小偏移量、最大偏移量和是否存在偏移的结构体
 *
 * @note 该函数应在路径规划过程中调用，确保偏移量计算正确
 *
 * @warning 需确保传入的线段和边界线段有效
 */
std::tuple<double, double, bool> LateralPathBoundParser::getOffset(const math::LineSegment2d& segment,
                                                                   const math::LineSegment2d& bound_segment) {
  std::tuple<double, double, bool> res{std::numeric_limits<double>::lowest(), std::numeric_limits<double>::max(),
                                       false};
  auto& [min_offset, max_offset, has_offset] = res;
  const double dx1 = bound_segment.start().x() - segment.start().x();
  const double dy1 = bound_segment.start().y() - segment.start().y();
  double x1 = dx1 * segment.cos_heading() + dy1 * segment.sin_heading();
  double y1 = -dx1 * segment.sin_heading() + dy1 * segment.cos_heading();
  const double dx2 = bound_segment.end().x() - segment.start().x();
  const double dy2 = bound_segment.end().y() - segment.start().y();
  double x2 = dx2 * segment.cos_heading() + dy2 * segment.sin_heading();
  double y2 = -dx2 * segment.sin_heading() + dy2 * segment.cos_heading();
  int gx1 = (x1 > segment.length() ? 1 : (x1 < 0.0 ? -1 : 0));
  int gx2 = (x2 > segment.length() ? 1 : (x2 < 0.0 ? -1 : 0));
  if (gx1 * gx2 <= 0) {
    has_offset = true;
    if (gx1 == 0 && gx2 == 0) {
      min_offset = y1;
      max_offset = y2;
    } else {
      if (gx1 > gx2) {
        std::swap(x1, x2);
        std::swap(y1, y2);
        std::swap(gx1, gx2);
      }
      min_offset = y1;
      max_offset = y2;
      double k = (y2 - y1) / (x2 - x1);
      if (gx1 < 0) {
        min_offset = y1 - x1 * k;
      }
      if (gx2 > 0) {
        max_offset = y2 + (-x2 + segment.length()) * k;
      }
    }
    if (min_offset > max_offset) {
      std::swap(min_offset, max_offset);
    }
  }
  return res;
}

/**
 * @brief 获取障碍物与缓冲区的映射关系
 * @details
 * 该函数用于生成障碍物对象与其对应缓冲区信息的映射关系。每个障碍物对象会关联一个包含三个double值的元组，分别表示起始缓冲区、结束缓冲区和横向缓冲区。
 *
 * @param[in] objects 障碍物集合，包含所有需要处理的障碍物对象
 * @param[in] buffer 缓冲区信息集合，包含每个障碍物的起始缓冲区、结束缓冲区和横向缓冲区
 *
 * @par 关键变量说明:
 * - res: 返回的映射关系，包含障碍物对象与其对应缓冲区信息的键值对
 *
 * @par 处理流程:
 * 1. 遍历所有障碍物对象
 * 2. 将每个障碍物对象与其对应的缓冲区信息关联
 * 3. 返回生成的映射关系
 *
 * @par 流程图:
 * @startuml
 * start
 * :遍历所有障碍物对象;
 * :将障碍物对象与其对应的缓冲区信息关联;
 * :返回生成的映射关系;
 * stop
 * @enduml
 *
 * @return std::unordered_map<const Decision::DecisionObject*, std::tuple<double, double, double>>
 * 返回包含障碍物对象与其对应缓冲区信息的映射关系
 *
 * @note 该函数应在路径规划过程中调用，确保障碍物与缓冲区的映射关系正确生成
 *
 * @warning 需确保传入的障碍物集合和缓冲区信息集合有效
 */
std::unordered_map<const Decision::DecisionObject*, std::tuple<double, double, double>>
LateralPathBoundParser::getObjectBufferMap(const std::vector<std::shared_ptr<Decision::DecisionObject>>& objects,
                                           const std::vector<std::tuple<double, double, double>>& buffer) {
  std::unordered_map<const Decision::DecisionObject*, std::tuple<double, double, double>> res;
  for (int i = 0; i < objects.size(); i++) {
    res.emplace(objects[i].get(), buffer[i]);
  }
  return std::move(res);
}

}  // namespace gpal::pnc::planning