/**
 * @file path_bound_parser.cpp
 * @brief 路径边界解析器实现文件
 * @details
 * 该文件实现了路径边界解析器的核心功能，包括路径边界的初始化、静态和动态障碍物的边界信息获取等。路径边界解析器用于在路径规划过程中处理障碍物对路径边界的影响，确保路径的安全性和可行性。
 */

#include "path_bound_parser/path_bound_parser.h"

namespace gpal::pnc::planning {

/**
 * @brief PathBoundParser构造函数
 * @details 该函数用于初始化PathBoundParser对象，根据传入的配置参数进行初始化。
 *
 * @param[in] config PathBoundParser的配置参数，包含最大范围、分辨率等参数
 *
 * @par 关键变量说明:
 * - config_: PathBoundParser的配置参数，包含最大范围、分辨率等参数
 *
 * @par 处理流程:
 * 1. 将传入的配置参数赋值给成员变量config_
 *
 * @par 流程图:
 * @startuml
 * start
 * :初始化config_成员变量;
 * stop
 * @enduml
 *
 * @note 该函数应在创建PathBoundParser对象时调用，确保对象正确初始化
 *
 * @warning 需确保传入的配置参数有效
 */
PathBoundParser::PathBoundParser(const PathBoundParserConfig& config) : config_(config) {}

/**
 * @brief 初始化路径边界解析器
 * @details
 * 该函数用于初始化路径边界解析器，根据参考线、地图到自车的变换矩阵以及路径的起始和结束位置，生成采样点并计算自车坐标系下的采样点信息。
 *
 * @param[in] ref_line 参考线，用于获取路径上的参考点
 * @param[in] tf_map_2_ego 地图到自车的变换矩阵，用于将地图坐标系下的点转换到自车坐标系
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
 * :初始化检查范围;
 * :计算路径长度和边界分辨率;
 * :计算边界段的数量;
 * :生成采样点;
 * :将采样点转换到自车坐标系;
 * :生成采样段;
 * stop
 * @enduml
 *
 * @return bool 初始化结果，true表示成功，false表示失败
 *
 * @note 该函数应在路径边界解析器初始化时调用，确保采样点和采样段正确生成
 *
 * @warning 需确保传入的参考线和变换矩阵有效
 */
bool PathBoundParser::init(const ReferenceLine& ref_line, const Eigen::Matrix4d& tf_map_2_ego, const double& start_s,
                           const double& end_s) {
  if (check_ranges_.empty()) {
    check_ranges_.emplace_back(start_s, -config_.max_range(), config_.max_range());
    check_ranges_.emplace_back(end_s, -config_.max_range(), config_.max_range());
  }
  double length = end_s - start_s;
  double bound_resolution = config_.resolution();
  int bound_segment_size = std::max<int>(1, std::round(length / bound_resolution) + 1e-10);
  bound_resolution = length / bound_segment_size;
  sample_points_.clear();
  for (int i = 0; i <= bound_segment_size; i++) {
    double curr_s = start_s + i * bound_resolution;
    sample_points_.emplace_back(ref_line.getReferencePoint(curr_s));
    sample_points_.back().setLocalS(curr_s);
  }
  ego_sample_points_.clear();
  sample_segments_.clear();
  for (const auto& sample_pt : sample_points_) {
    math::Vec3d flu_position_vec3d(sample_pt.x(), sample_pt.y(), sample_pt.z());
    transfer::transformPoint(tf_map_2_ego, &flu_position_vec3d);
    math::Vec3d flu_rpy_vec3d(0.0, 0.0, sample_pt.heading());
    transfer::transformRPY(tf_map_2_ego, &flu_rpy_vec3d);

    ego_sample_points_.emplace_back(sample_pt);
    ego_sample_points_.back().set_x(flu_position_vec3d.x());
    ego_sample_points_.back().set_y(flu_position_vec3d.y());
    ego_sample_points_.back().set_z(flu_position_vec3d.z());
    ego_sample_points_.back().setHeading(flu_rpy_vec3d.z());

    math::Vec3d unit_direction(cos(sample_pt.heading()), sin(sample_pt.heading()), 0.0);
    sample_segments_.emplace_back(sample_pt, sample_pt + bound_resolution * unit_direction);
  }
  return sample_points_.size() > 0;
}

/**
 * @brief 初始化路径边界解析器（带最大范围参数）
 * @details
 * 该函数用于初始化路径边界解析器，根据参考线、地图到自车的变换矩阵、路径的起始和结束位置以及最大范围参数，生成采样点并计算自车坐标系下的采样点信息。
 *
 * @param[in] ref_line 参考线，用于获取路径上的参考点
 * @param[in] tf_map_2_ego 地图到自车的变换矩阵，用于将地图坐标系下的点转换到自车坐标系
 * @param[in] start_s 路径的起始位置，单位为米
 * @param[in] end_s 路径的结束位置，单位为米
 * @param[in] max_range 最大范围参数，包含路径的起始和结束位置以及最大范围
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
 * 1. 设置检查范围为传入的最大范围参数
 * 2. 调用不带最大范围参数的init函数进行初始化
 *
 * @par 流程图:
 * @startuml
 * start
 * :设置检查范围为传入的最大范围参数;
 * :调用不带最大范围参数的init函数进行初始化;
 * stop
 * @enduml
 *
 * @return bool 初始化结果，true表示成功，false表示失败
 *
 * @note 该函数应在路径边界解析器初始化时调用，确保采样点和采样段正确生成
 *
 * @warning 需确保传入的参考线、变换矩阵和最大范围参数有效
 */
bool PathBoundParser::init(const ReferenceLine& ref_line, const Eigen::Matrix4d& tf_map_2_ego, const double& start_s,
                           const double& end_s, const std::vector<std::tuple<double, double, double>>& max_range) {
  check_ranges_ = max_range;
  return init(ref_line, tf_map_2_ego, start_s, end_s);
}

/**
 * @brief 从静态障碍物获取边界信息（带缓冲区）
 * @details 该函数用于从静态障碍物中获取路径边界信息，包括左右边界的位置以及对应的障碍物信息，同时考虑缓冲区的影响。
 *
 * @param[in] decision_obstacles 静态障碍物集合，包含所有需要处理的静态障碍物
 * @param[in] buffer 缓冲区参数集合，包含每个障碍物的起始缓冲区、结束缓冲区和横向缓冲区
 *
 * @par 关键变量说明:
 * - res: 返回的障碍物边界信息，包含边界位置和对应的障碍物
 * - buffer_map: 障碍物与缓冲区的映射关系
 * - small_obstacles: 小型障碍物集合
 * - left_ranges: 左侧障碍物的范围信息
 * - right_ranges: 右侧障碍物的范围信息
 * - left_od_bounds: 左侧障碍物的边界范围信息
 * - right_od_bounds: 右侧障碍物的边界范围信息
 * - left_obstacles: 左侧障碍物的信息集合
 * - right_obstacles: 右侧障碍物的信息集合
 *
 * @par 处理流程:
 * 1. 获取障碍物与缓冲区的映射关系
 * 2. 对静态障碍物进行排序，分为左侧和右侧障碍物，并筛选出小型障碍物
 * 3. 对小型障碍物的边界范围进行排序
 * 4. 遍历采样点，更新左右边界信息
 * 5. 根据障碍物位置和缓冲区调整边界范围
 * 6. 对左右边界范围进行排序
 * 7. 遍历采样点，生成最终的边界信息
 *
 * @par 流程图:
 * @startuml
 * start
 * :获取障碍物与缓冲区的映射关系;
 * :对静态障碍物进行排序;
 * :对小型障碍物的边界范围进行排序;
 * :遍历采样点;
 * :更新左右边界信息;
 * :根据障碍物位置和缓冲区调整边界范围;
 * :对左右边界范围进行排序;
 * :生成最终的边界信息;
 * stop
 * @enduml
 *
 * @return ObstacleBoundaryInfo 返回包含边界信息和对应障碍物的结构体
 *
 * @note 该函数应在路径规划过程中调用，确保边界信息正确生成
 *
 * @warning 需确保传入的静态障碍物集合和缓冲区参数有效
 */
ObstacleBoundaryInfo PathBoundParser::getBoundaryFromStaticObstacles(
    const std::vector<std::shared_ptr<Decision::DecisionObject>>& decision_obstacles,
    const std::vector<std::tuple<double, double, double>>& buffer) {
  ObstacleBoundaryInfo res;
  auto buffer_map = getObstacleBufferMap(decision_obstacles, buffer);
  std::vector<std::shared_ptr<Decision::DecisionObject>> small_obstacles;
  auto [left_ranges, right_ranges] =
      sortStaticObstacleRange(decision_obstacles, 2 * config_.resolution(), &small_obstacles);
  auto [left_od_bounds, right_od_bounds] =
      sortStaticObstacleBoundRange(small_obstacles, buffer_map);  // buffer: start_buffer, end_buffer, lat_buffer

  int curr_idx_left = 0, curr_idx_right = 0;
  StaticObstacleInfoMap left_obstacles, right_obstacles;
  for (int i = 0; i + 1 < sample_points_.size(); i++) {
    double curr_s = sample_points_[i].local_s();
    double next_s = sample_points_[i + 1].local_s();
    while (curr_idx_left < left_ranges.size()) {
      auto& [in_bound, s, ptr_obs, idx] = left_ranges[curr_idx_left];
      if (next_s < s) {
        break;
      }
      if (in_bound) {
        left_obstacles.emplace(ptr_obs,
                               getStaticObstacleInfo(*ptr_obs, std::get<2>(buffer[idx])));  // segments, lat_buffer
      } else {
        left_obstacles.erase(ptr_obs);
      }
      curr_idx_left++;
    }
    while (curr_idx_right < right_ranges.size()) {
      auto& [in_bound, s, ptr_obs, idx] = right_ranges[curr_idx_right];
      if (next_s < s) {
        break;
      }
      if (in_bound) {
        right_obstacles.emplace(ptr_obs, getStaticObstacleInfo(*ptr_obs, std::get<2>(buffer[idx])));
      } else {
        right_obstacles.erase(ptr_obs);
      }
      curr_idx_right++;
    }
    const auto& [right_in_bound, right_offset] =
        getRightOffsetInfoFromStaticObstacle(sample_segments_[i], right_obstacles);
    if (right_in_bound) {
      const auto& [start_buffer, end_buffer, lat_buffer] = buffer_map[right_in_bound];
      right_od_bounds.emplace_back(1, curr_s - start_buffer, right_offset, right_in_bound);
      right_od_bounds.emplace_back(0, next_s + end_buffer, right_offset, right_in_bound);
    }
    const auto& [left_in_bound, left_offset] = getLeftOffsetInfoFromStaticObstacle(sample_segments_[i], left_obstacles);
    if (left_in_bound) {
      const auto& [start_buffer, end_buffer, lat_buffer] = buffer_map[left_in_bound];
      left_od_bounds.emplace_back(1, curr_s - start_buffer, left_offset, left_in_bound);
      left_od_bounds.emplace_back(0, next_s + end_buffer, left_offset,
                                  left_in_bound);  // 一系列的头和尾，而不是只有一个
    }
  }

  auto sorter = [](const ObstacleBoundRange& lhs, const ObstacleBoundRange& rhs) {
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
  for (int i = 0; i + 1 < sample_points_.size(); i++) {
    double curr_s = sample_points_[i].local_s();
    while (right_obs_idx < right_od_bounds.size() && std::get<1>(right_od_bounds[right_obs_idx]) < curr_s) {
      auto& [in_bound, obs_s, obs_l, ptr_obs] = right_od_bounds[right_obs_idx];
      if (in_bound) {
        // DecisionObject is to the right of center-line, should pass from left.
        right_bounds.emplace(obs_l, ptr_obs);
      } else {
        right_bounds.erase(right_bounds.find(obs_l));
      }
      ++right_obs_idx;
    }
    while (left_obs_idx < left_od_bounds.size() && std::get<1>(left_od_bounds[left_obs_idx]) < curr_s) {
      auto& [in_bound, obs_s, obs_l, ptr_obs] = left_od_bounds[left_obs_idx];
      if (in_bound) {
        // DecisionObject is to the right of center-line, should pass from left.
        left_bounds.emplace(obs_l, ptr_obs);
      } else {
        left_bounds.erase(left_bounds.find(obs_l));
      }
      ++left_obs_idx;
    }
    auto [raw_bound, raw_has_max_bound] = check_ranges_.interpolate(curr_s);
    auto bound = raw_bound;
    auto& [bound_s, bound_right, bound_left] = bound;
    const Decision::DecisionObject* ptr_right = nullptr;
    if (right_bounds.begin()->first > bound_right) {
      bound_right = right_bounds.begin()->first;
      ptr_right = right_bounds.begin()->second;
    }
    const Decision::DecisionObject* ptr_left = nullptr;
    if (left_bounds.begin()->first < bound_left) {
      bound_left = left_bounds.begin()->first;
      ptr_left = left_bounds.begin()->second;
    }
    if (bound_right > bound_left) {
      // early break for block
      bound_left = std::get<2>(raw_bound);
      bound_right = std::get<1>(raw_bound);
    }
    res.bound.emplace_back(curr_s, bound_right, bound_left);
    res.right_bound_obstacles.emplace_back(ptr_right);
    res.left_bound_obstacles.emplace_back(ptr_left);
  }

  return res;
}

/**
 * @brief 判断障碍物是否逆向行驶
 * @details 该函数用于判断给定的障碍物是否相对于参考线逆向行驶，通过比较障碍物的航向与参考线投影点的航向差来判断。
 *
 * @param[in] ref_line 参考线，用于获取障碍物投影点的航向
 * @param[in] npc 障碍物对象，包含障碍物的当前位置和航向信息
 *
 * @par 关键变量说明:
 * - nearest_ref_point: 障碍物中心点在参考线上的最近投影点
 * - npc_projection_heading: 障碍物在参考线上的投影点航向
 * - heading_diff: 障碍物航向与投影点航向的差值
 *
 * @par 处理流程:
 * 1. 获取障碍物中心点在参考线上的最近投影点
 * 2. 计算障碍物航向与投影点航向的差值
 * 3. 判断航向差是否大于90度或小于-90度，若是则判定为逆向行驶
 *
 * @par 流程图:
 * @startuml
 * start
 * :获取障碍物中心点在参考线上的最近投影点;
 * :计算障碍物航向与投影点航向的差值;
 * if (航向差 > 90度 || 航向差 < -90度) then (yes)
 *   :返回true;
 * else (no)
 *   :返回false;
 * endif
 * stop
 * @enduml
 *
 * @return bool 返回true表示障碍物逆向行驶，false表示正向行驶
 *
 * @note 该函数应在判断障碍物行驶方向时调用，确保路径规划时正确处理逆向行驶的障碍物
 *
 * @warning 需确保传入的参考线和障碍物对象有效
 */
// bool PathBoundParser::isReverse(const ReferenceLine& ref_line, const Decision::DecisionObject* npc) {
//   if (ref_line.reference_points().empty()) {
//     return false;
//   }
//   ReferencePoint nearest_ref_point = ref_line.getNearestReferencePoint(math::Vec3d(npc->cur_box.center(), 0.0));
//   double npc_projection_heading = nearest_ref_point.heading();
//   double heading_diff = math::NormalizeAngle(npc->cur_box.heading() - npc_projection_heading);

//   return (heading_diff > M_PI_2 || heading_diff < -M_PI_2);
// }

/**
 * @brief 对静态障碍物进行排序并分类
 * @details 该函数用于对静态障碍物进行排序，并根据其位置和类型将其分为左侧和右侧障碍物。同时，可以筛选出小型障碍物。
 *
 * @param[in] decision_obstacles 静态障碍物集合，包含所有需要处理的静态障碍物
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
 * @return std::pair<std::vector<StaticObstacleRange>, std::vector<StaticObstacleRange>>
 * 返回包含左侧和右侧障碍物范围信息的结构体
 *
 * @note 该函数应在路径规划过程中调用，确保障碍物正确分类和排序
 *
 * @warning 需确保传入的静态障碍物集合有效
 */
std::pair<std::vector<PathBoundParser::StaticObstacleRange>, std::vector<PathBoundParser::StaticObstacleRange>>
PathBoundParser::sortStaticObstacleRange(
    const std::vector<std::shared_ptr<Decision::DecisionObject>>& decision_obstacles, const double resolution_thrd,
    std::vector<std::shared_ptr<Decision::DecisionObject>>* ptr_small) {
  std::pair<std::vector<StaticObstacleRange>, std::vector<StaticObstacleRange>> res;
  auto& [left_ranges, right_ranges] = res;  // each range: is in range, start_s or end_s, obstacle_ptr, index

  ReferencePoint end_point = sample_points_.back();
  const double heading = end_point.heading();
  math::Vec2d unit_vec(cos(heading), sin(heading));
  constexpr double near_dest_thresold = 0.1;
  for (int i = 0; i < decision_obstacles.size(); i++) {
    const auto& obstacle = decision_obstacles[i];
    // ignore the obstacles which far from the sample_points_
    double projection = std::numeric_limits<double>::max();
    for (const auto& point : obstacle->cur_box.GetAllCorners()) {
      math::Vec2d vec(point.x() - end_point.x(), point.y() - end_point.y());
      if (unit_vec.InnerProd(vec) < projection) {
        projection = unit_vec.InnerProd(vec);
      }
    }

    bool out_roi = false;
    if (std::abs(obstacle->cur_sl_bound.start_s() - end_point.local_s()) < near_dest_thresold &&
        projection > config_.roi_thresold()) {
      out_roi = true;
    }

    if (obstacle->cur_sl_bound.start_s() > end_point.local_s() || out_roi) {
      continue;
    }
    // Only focus on obstacles that are ahead of Adc.
    if (obstacle->cur_sl_bound.end_s() < sample_points_.front().local_s()) {
      continue;
    }
    const LateralOdTag nudge_tag = obstacle->lat_od_tag;

    if (obstacle->cur_sl_bound.end_s() - obstacle->cur_sl_bound.start_s() > resolution_thrd) {
      switch (nudge_tag) {
        case LateralOdTag::LEFT_BYPASS: {
          right_ranges.emplace_back(1, obstacle->cur_sl_bound.start_s(), obstacle.get(), i);
          right_ranges.emplace_back(0, obstacle->cur_sl_bound.end_s(), obstacle.get(), i);

        } break;
        case LateralOdTag::RIGHT_BYPASS: {
          left_ranges.emplace_back(1, obstacle->cur_sl_bound.start_s(), obstacle.get(), i);
          left_ranges.emplace_back(0, obstacle->cur_sl_bound.end_s(), obstacle.get(), i);
        } break;
      }
    } else if (ptr_small != nullptr) {
      ptr_small->emplace_back(obstacle);
    }
  }

  auto sorter = [](const StaticObstacleRange& lhs, const StaticObstacleRange& rhs) {  // ture不调换
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
 * @param[in] decision_obstacles 静态障碍物集合，包含所有需要处理的静态障碍物
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
 * @return std::pair<std::vector<ObstacleBoundRange>, std::vector<ObstacleBoundRange>>
 * 返回包含左侧和右侧障碍物边界范围信息的结构体
 *
 * @note 该函数应在路径规划过程中调用，确保障碍物边界范围正确分类和排序
 *
 * @warning 需确保传入的静态障碍物集合和缓冲区映射关系有效
 */
std::pair<std::vector<PathBoundParser::ObstacleBoundRange>, std::vector<PathBoundParser::ObstacleBoundRange>>
PathBoundParser::sortStaticObstacleBoundRange(
    const std::vector<std::shared_ptr<Decision::DecisionObject>>& decision_obstacles,
    const std::unordered_map<const Decision::DecisionObject*, std::tuple<double, double, double>>& buffer_map) {
  std::pair<std::vector<ObstacleBoundRange>, std::vector<ObstacleBoundRange>> res;
  auto& [left_ranges, right_ranges] = res;  // each range: is in range, start_s or end_s, offset, obstacle_ptr

  for (int i = 0; i < decision_obstacles.size(); i++) {
    const auto& obstacle = decision_obstacles[i];
    const auto& [start_buffer, end_buffer, lat_buffer] = buffer_map.at(obstacle.get());
    if (obstacle->cur_sl_bound.start_s() - start_buffer > sample_points_.back().local_s()) {
      continue;
    }
    // Only focus on obstacles that are ahead of Adc.
    if (obstacle->cur_sl_bound.end_s() + end_buffer < sample_points_.front().local_s()) {
      continue;
    }

    const planning::LateralOdTag nudge_tag = obstacle->lat_od_tag;

    switch (nudge_tag) {
      case LateralOdTag::LEFT_BYPASS: {
        right_ranges.emplace_back(1, obstacle->cur_sl_bound.start_s() - start_buffer,
                                  obstacle->cur_sl_bound.end_l() + lat_buffer, obstacle.get());
        right_ranges.emplace_back(0, obstacle->cur_sl_bound.end_s() + end_buffer,
                                  obstacle->cur_sl_bound.end_l() + lat_buffer, obstacle.get());

      } break;
      case LateralOdTag::RIGHT_BYPASS: {
        left_ranges.emplace_back(1, obstacle->cur_sl_bound.start_s() - start_buffer,
                                 obstacle->cur_sl_bound.start_l() - lat_buffer, obstacle.get());
        left_ranges.emplace_back(0, obstacle->cur_sl_bound.end_s() + end_buffer,
                                 obstacle->cur_sl_bound.start_l() - lat_buffer, obstacle.get());
      } break;
    }
  }

  auto sorter = [](const ObstacleBoundRange& lhs, const ObstacleBoundRange& rhs) {
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
 * @param[in] static_obstacle 静态障碍物对象，包含障碍物的几何信息
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
 * @return StaticObstacleInfo 返回包含障碍物线段集合和横向缓冲区的结构体
 *
 * @note 该函数应在处理静态障碍物时调用，确保获取到障碍物的几何信息
 *
 * @warning 需确保传入的静态障碍物对象有效
 */
PathBoundParser::StaticObstacleInfo PathBoundParser::getStaticObstacleInfo(
    const Decision::DecisionObject& static_obstacle, const double lat_buffer) {
  const std::vector<math::Vec2d>& points = static_obstacle.cur_box.GetAllCorners();
  std::vector<math::LineSegment2d> segments;
  if (points.size() > 1) {
    for (int i = 0; i + 1 < points.size(); i++) {
      segments.emplace_back(points[i], points[i + 1]);
    }
    segments.emplace_back(points.back(), points.front());
  }
  return StaticObstacleInfo(segments, lat_buffer);  // std::tuple<std::vector<math::LineSegment2d>, double>;
}

/**
 * @brief 从静态障碍物获取左侧偏移信息
 * @details
 * 该函数用于从静态障碍物中获取左侧偏移信息，包括障碍物对象和对应的偏移量。通过计算障碍物边界与中心线的距离，确定是否需要左侧偏移。
 *
 * @param[in] center 中心线段，表示当前路径的中心线
 * @param[in] obstacle_borders 障碍物边界信息集合，包含障碍物的线段集合和横向缓冲区
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
std::tuple<const Decision::DecisionObject*, double> PathBoundParser::getLeftOffsetInfoFromStaticObstacle(
    const math::LineSegment2d& center, const StaticObstacleInfoMap& obstacle_borders) {
  std::tuple<const Decision::DecisionObject*, double> res{nullptr, std::numeric_limits<double>::max()};
  auto& [in_bound_obs, offset] = res;
  double left_offset_enable_radius = 10;
  for (const auto& [ptr_obs, obs_info] : obstacle_borders) {  // obstacle_borders: std::map<const DecisionObject*,
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
 * @param[in] obstacle_borders 障碍物边界信息集合，包含障碍物的线段集合和横向缓冲区
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
std::tuple<const Decision::DecisionObject*, double> PathBoundParser::getRightOffsetInfoFromStaticObstacle(
    const math::LineSegment2d& center, const StaticObstacleInfoMap& obstacle_borders) {
  std::tuple<const Decision::DecisionObject*, double> res{nullptr, std::numeric_limits<double>::lowest()};
  auto& [in_bound_obs, offset] = res;
  double right_offset_enable_radius = 10;
  for (const auto& [ptr_obs, obs_info] : obstacle_borders) {  // obstacle_borders: std::map<const DecisionObject*,
                                                              // std::tuple<std::vector<math::LineSegment2d>, double>>
    const auto& [segments, lat_buffer] = obs_info;
    bool enable_right_offset = false;
    for (const auto& segment : segments) {
      float dis_point_to_line = CalculatePointDistanceToLineSegment(
          segment,
          math::Vec2d((center.start().x() + center.end().x()) / 2.0, (center.start().y() + center.end().y()) / 2.0));
      if (dis_point_to_line < right_offset_enable_radius) {
        enable_right_offset = true;
        break;
      }
    }
    if (!enable_right_offset) {
      continue;
    }
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
std::tuple<double, double, bool> PathBoundParser::getOffset(const math::LineSegment2d& segment,
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
 * @param[in] obstacles 障碍物集合，包含所有需要处理的障碍物对象
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
PathBoundParser::getObstacleBufferMap(const std::vector<std::shared_ptr<Decision::DecisionObject>>& obstacles,
                                      const std::vector<std::tuple<double, double, double>>& buffer) {
  std::unordered_map<const Decision::DecisionObject*, std::tuple<double, double, double>> res;
  for (int i = 0; i < obstacles.size(); i++) {
    res.emplace(obstacles[i].get(), buffer[i]);
  }
  return std::move(res);
}

/**
 * @brief 从动态障碍物获取边界信息
 * @details
 * 该函数用于从动态障碍物中获取边界信息，包括左侧和右侧的边界范围。通过遍历动态障碍物，计算其在路径上的投影，并根据障碍物的位置和类型将其分类为左侧或右侧边界。
 *
 * @param[in] ref_line 参考线，表示当前路径的中心线
 * @param[in] decision_obstacles 动态障碍物集合，包含所有需要处理的动态障碍物
 * @param[in] buffer 缓冲区信息集合，包含每个障碍物的起始缓冲区、结束缓冲区和横向缓冲区
 *
 * @par 关键变量说明:
 * - res: 返回的边界信息，包含左侧和右侧的边界范围
 * - left_od_bound_ranges: 左侧动态障碍物的边界范围信息
 * - right_od_bound_ranges: 右侧动态障碍物的边界范围信息
 * - right_bounds: 右侧边界集合，用于存储右侧障碍物的边界信息
 * - left_bounds: 左侧边界集合，用于存储左侧障碍物的边界信息
 * - right_obs_idx: 右侧障碍物的索引
 * - left_obs_idx: 左侧障碍物的索引
 * - curr_s: 当前路径点的s值
 * - bound: 当前路径点的边界信息
 * - ptr_right: 右侧障碍物对象
 * - ptr_left: 左侧障碍物对象
 *
 * @par 处理流程:
 * 1. 对动态障碍物进行排序并分类
 * 2. 初始化右侧和左侧边界集合
 * 3. 遍历所有路径点，计算当前路径点的边界信息
 * 4. 更新右侧和左侧边界集合
 * 5. 返回包含左侧和右侧边界信息的结构体
 *
 * @par 流程图:
 * @startuml
 * start
 * :对动态障碍物进行排序并分类;
 * :初始化右侧和左侧边界集合;
 * :遍历所有路径点;
 * :计算当前路径点的边界信息;
 * :更新右侧和左侧边界集合;
 * :返回包含左侧和右侧边界信息的结构体;
 * stop
 * @enduml
 *
 * @return ObstacleBoundaryInfo 返回包含左侧和右侧边界信息的结构体
 *
 * @note 该函数应在路径规划过程中调用，确保动态障碍物边界信息正确生成
 *
 * @warning 需确保传入的参考线、动态障碍物集合和缓冲区信息集合有效
 */
ObstacleBoundaryInfo PathBoundParser::getBoundaryFromDynamicObstacles(
    const ReferenceLine& ref_line, const std::vector<std::shared_ptr<Decision::DecisionObject>>& decision_obstacles,
    const std::vector<std::tuple<double, double, double, double, double, double, double>>& buffer) {
  ObstacleBoundaryInfo res;
  auto [left_od_bound_ranges, right_od_bound_ranges] =
      sortDynamicObstacleAndBoundaryRange(ref_line, decision_obstacles, buffer);

  std::multimap<double, const Decision::DecisionObject*, std::greater<double>> right_bounds;
  right_bounds.emplace(std::numeric_limits<double>::lowest(), nullptr);
  std::multimap<double, const Decision::DecisionObject*> left_bounds;
  left_bounds.emplace(std::numeric_limits<double>::max(), nullptr);
  int right_obs_idx = 0, left_obs_idx = 0;
  for (int i = 0; i + 1 < sample_points_.size(); i++) {
    double curr_s = sample_points_[i].local_s();
    while (right_obs_idx < right_od_bound_ranges.size() && std::get<1>(right_od_bound_ranges[right_obs_idx]) < curr_s) {
      auto& [in_bound, obs_s, obs_l, right_offset, ptr_obs, idx] = right_od_bound_ranges[right_obs_idx];
      if (in_bound) {
        // DecisionObject is to the right of center-line, should pass from left.
        right_bounds.emplace(obs_l, ptr_obs);
      } else {
        right_bounds.erase(right_bounds.find(obs_l));
      }
      ++right_obs_idx;
    }
    while (left_obs_idx < left_od_bound_ranges.size() && std::get<1>(left_od_bound_ranges[left_obs_idx]) < curr_s) {
      auto& [in_bound, obs_s, obs_l, left_offset, ptr_obs, idx] = left_od_bound_ranges[left_obs_idx];
      if (in_bound) {
        // DecisionObject is to the right of center-line, should pass from left.
        left_bounds.emplace(obs_l, ptr_obs);
      } else {
        left_bounds.erase(left_bounds.find(obs_l));
      }
      ++left_obs_idx;
    }
    auto [bound, has_max_bound] = check_ranges_.interpolate(curr_s);
    auto& [bound_s, bound_right, bound_left] = bound;
    const Decision::DecisionObject* ptr_right = nullptr;
    if (right_bounds.begin()->first > bound_right) {
      bound_right = right_bounds.begin()->first;
      ptr_right = right_bounds.begin()->second;
    }
    const Decision::DecisionObject* ptr_left = nullptr;
    if (left_bounds.begin()->first < bound_left) {
      bound_left = left_bounds.begin()->first;
      ptr_left = left_bounds.begin()->second;
    }
    res.bound.emplace_back(curr_s, bound_right, bound_left);
    res.right_bound_obstacles.emplace_back(ptr_right);
    res.left_bound_obstacles.emplace_back(ptr_left);
    if (bound_right > bound_left) {
      // early break for block
      break;
    }
  }
  return res;
}

/**
 * @brief 对动态障碍物及其边界范围进行排序和分类
 * @details
 * 该函数用于对动态障碍物进行排序，并根据其位置和类型将其分为左侧和右侧边界范围。通过遍历动态障碍物，计算其在路径上的投影，并根据障碍物的位置和类型将其分类为左侧或右侧边界范围。
 *
 * @param[in] ref_line 参考线，表示当前路径的中心线
 * @param[in] decision_obstacles 动态障碍物集合，包含所有需要处理的动态障碍物
 * @param[in] buffer 缓冲区信息集合，包含每个障碍物的起始缓冲区、结束缓冲区和横向缓冲区
 *
 * @par 关键变量说明:
 * - res: 返回的边界范围信息，包含左侧和右侧动态障碍物的边界范围
 * - left_ranges: 左侧动态障碍物的边界范围信息
 * - right_ranges: 右侧动态障碍物的边界范围信息
 * - dynamic_obs_to_nudge: 需要动态绕行的动态障碍物集合
 * - dynamic_obs_to_nudge_buffer: 需要动态绕行的动态障碍物的缓冲区信息
 * - min_lon_buffer: 最小纵向缓冲区
 * - half_length: 障碍物长度的一半
 * - half_width: 障碍物宽度的一半
 * - dynamic_obstacle_lon_start_dis: 动态障碍物的起始纵向距离
 * - dynamic_obstacle_lon_end_dis: 动态障碍物的结束纵向距离
 * - is_right_bound: 是否为右侧边界的标志
 * - is_in_bound: 是否在边界范围内的标志
 * - start_sl: 动态障碍物起始点在参考线上的投影
 * - end_sl: 动态障碍物结束点在参考线上的投影
 * - lower_s: 动态障碍物的起始纵向位置
 * - upper_s: 动态障碍物的结束纵向位置
 * - start_l: 动态障碍物起始点的横向位置
 * - end_l: 动态障碍物结束点的横向位置
 *
 * @par 处理流程:
 * 1. 基于速度滞回挑选需要动态绕行的动态障碍物
 * 2. 遍历所有需要动态绕行的动态障碍物
 * 3. 计算动态障碍物在参考线上的投影
 * 4. 根据动态障碍物的位置和类型将其分类为左侧或右侧边界范围
 * 5. 对左侧和右侧边界范围进行排序
 *
 * @par 流程图:
 * @startuml
 * start
 * :基于速度滞回挑选需要动态绕行的动态障碍物;
 * :遍历所有需要动态绕行的动态障碍物;
 * :计算动态障碍物在参考线上的投影;
 * :根据动态障碍物的位置和类型将其分类为左侧或右侧边界范围;
 * :对左侧和右侧边界范围进行排序;
 * stop
 * @enduml
 *
 * @return std::pair<std::vector<DynamicObstacleRange>, std::vector<DynamicObstacleRange>>
 * 返回包含左侧和右侧动态障碍物边界范围信息的结构体
 *
 * @note 该函数应在路径规划过程中调用，确保动态障碍物边界范围正确分类和排序
 *
 * @warning 需确保传入的参考线、动态障碍物集合和缓冲区信息集合有效
 */
std::pair<std::vector<PathBoundParser::DynamicObstacleRange>, std::vector<PathBoundParser::DynamicObstacleRange>>
PathBoundParser::sortDynamicObstacleAndBoundaryRange(
    const ReferenceLine& ref_line, const std::vector<std::shared_ptr<Decision::DecisionObject>>& decision_obstacles,
    const std::vector<std::tuple<double, double, double, double, double, double, double>>& buffer) {
  std::pair<std::vector<DynamicObstacleRange>, std::vector<DynamicObstacleRange>> res;
  auto& [left_ranges, right_ranges] = res;

  // 基于速度滞回挑选实际需要动态nudge的目标.
  const auto& [dynamic_obs_to_nudge, dynamic_obs_to_nudge_buffer] =
      pickDynamicObstaclesForDynamicNudge(ref_line, adc_frenet_sd_, decision_obstacles, buffer);

  // left_ranges, right_ranges计算.
  double min_lon_buffer = 2.0;
  for (int k = 0; k < dynamic_obs_to_nudge.size(); k++) {
    const auto& obstacle = dynamic_obs_to_nudge.at(k);
    const double half_length = 0.5 * obstacle->cur_box.length();
    const double half_width = 0.5 * obstacle->cur_box.width();
    double dynamic_obstacle_lon_start_dis =
        std::fmax(std::get<0>(dynamic_obs_to_nudge_buffer.at(k)),
                  std::get<1>(dynamic_obs_to_nudge_buffer.at(k)) * std::fabs(adc_frenet_sd_ - obstacle->spd));
    double dynamic_obstacle_lon_end_dis =
        std::fmax(std::get<2>(dynamic_obs_to_nudge_buffer.at(k)),
                  std::get<3>(dynamic_obs_to_nudge_buffer.at(k)) * std::fabs(adc_frenet_sd_));

    bool is_right_bound = false;
    if (obstacle->lat_od_tag == LateralOdTag::DYNAMIC_LEFT_BYPASS) {
      is_right_bound = true;
    }
    // ERT_PLOG_I << "[bypass]: is_right_bound = " << is_right_bound ;

    bool is_in_bound = false;
    for (int i = 0; i < adc_ts_ranges_.size(); i++) {
      const auto& [start_t, end_t, start_s, end_s] = adc_ts_ranges_[i];
      // ERT_PLOG_I << "[bypass]: adc_ts_ranges_: i = " << i
      //           << " start_t = " << start_t
      //           << " end_t = " << end_t
      //           << " start_s = " << start_s
      //           << " end_s = " << end_s
      //           ;
      if (start_t > obstacle->bypass_end_time) {
        break;
      }
      if (end_t < obstacle->bypass_start_time) {
        continue;
      }
      SLPoint start_sl, end_sl;  // obs中心点不同时刻在参考线上的投影
      if (obstacle->raw_predictions.empty() || obstacle->raw_predictions.front().traj.empty() ||
          !ref_line.xy2sl(obstacle->raw_predictions.front().traj.front().path_point(), &start_sl) ||
          !ref_line.xy2sl(obstacle->raw_predictions.front().traj.back().path_point(), &end_sl)) {
        continue;
      }

      const double lower_s = start_sl.s() - half_length - min_lon_buffer - dynamic_obstacle_lon_start_dis;
      const double upper_s = end_sl.s() + half_length + min_lon_buffer + dynamic_obstacle_lon_end_dis;
      // ERT_PLOG_I << "[bypass]: obs info: lower_s = " << lower_s
      //           << " upper_s = " << upper_s
      //           << " start_sl.s() = " << start_sl.s()
      //           << " end_sl.s() = " << end_sl.s()
      //           << " half_length = " << half_length
      //           << " min_lon_buffer = " << min_lon_buffer
      //           << " obs_lon_start_dis = " << dynamic_obstacle_lon_start_dis
      //           << " obs_lon_end_dis = " << dynamic_obstacle_lon_end_dis
      //           ;
      // if (end_sl.s() - end_s > start_sl.s() - start_s) {
      //   if (is_in_bound) {
      //     if (is_right_bound) {
      //       right_ranges.erase(right_ranges.end() - 1);
      //     } else {
      //       left_ranges.erase(left_ranges.end() - 1);
      //     }
      //     is_in_bound = false;
      //   }
      //   break;
      // }

      const double start_l = start_sl.l() + (is_right_bound ? half_width : -half_width);
      const double end_l = end_sl.l() + (is_right_bound ? half_width : -half_width);
      // ERT_PLOG_I << "[bypass]: obs info: start_l = " << start_l
      //           << " end_l = " << end_l
      //           << " start_sl.l() = " << start_sl.l()
      //           << " end_sl.l() = " << end_sl.l()
      //           << " half_width = " << half_width
      //           ;
      bool is_range_overlap =
          std::fmax(upper_s, end_s) - std::fmin(lower_s, start_s) < (upper_s - lower_s) + (end_s - start_s);
      // if (!is_in_bound && lower_s < start_s && start_s <= upper_s) {
      //  // ERT_PLOG_I << "[bypass]: scenario 11111: !is_in_bound && lower_s < start_s && start_s <= upper_s";
      if (!is_in_bound && is_range_overlap) {
        // ERT_PLOG_I << "  [bypass]: scenario 11111: !is_in_bound && is_range_overlap" ;
        is_in_bound = true;

        // 1）add new range
        if (is_right_bound) {
          // is in bound, s, l, offset, obstacle_ptr, index(adc_ts_ranges_)
          right_ranges.emplace_back(1, lower_s, start_l + std::get<4>(dynamic_obs_to_nudge_buffer.at(k)), 0,
                                    obstacle.get(), i);
        } else {
          // is in bound, s, l, offset, obstacle_ptr, index(adc_ts_ranges_)
          left_ranges.emplace_back(1, lower_s, start_l - std::get<4>(dynamic_obs_to_nudge_buffer.at(k)), 0,
                                   obstacle.get(), i);
        }

        // 2）若障碍车实际（不考虑buffer）在自车身后，则is_in_bound = false，且删除掉刚才add的new range
        if (start_s > end_sl.s() + half_length + min_lon_buffer) {
          // ERT_PLOG_I << "[bypass]: scenario 11111: 筛选后方障碍车，忽略!!! adc_ts_ranges: start_t = " << start_t
          //           << " end_t = " << end_t
          //           << " start_s = " << start_s
          //           << " end_s = " << end_s
          //           << "   obs: end_sl.s() + half_length = " << end_sl.s() + half_length
          //           << " id = " << obstacle->id()
          //           ;
          is_in_bound = false;
          if (is_right_bound) {
            right_ranges.erase(right_ranges.end() - 1);
          } else {
            left_ranges.erase(left_ranges.end() - 1);
          }
        }

        // ERT_PLOG_I << "[bypass]: scenario 11111: right_ranges.size() = " << right_ranges.size() ;
        // for(int m = 0; m < right_ranges.size(); m++) {
        //   ERT_PLOG_I << "  [bypass]: m = " << m
        //             << " in_bound = " << std::get<0>(right_ranges.at(m))
        //             << " s = " << std::get<1>(right_ranges.at(m))
        //             << " l = " << std::get<2>(right_ranges.at(m))
        //             << " offset = " << std::get<3>(right_ranges.at(m))
        //             << " obs_id = " << std::get<4>(right_ranges.at(m))->id()
        //             << " adc_ts_index = " << std::get<5>(right_ranges.at(m))
        //             ;
        // }
        // ERT_PLOG_I << "[bypass]: scenario 11111: left_ranges.size() = " << left_ranges.size() ;
        // for(int m = 0; m < left_ranges.size(); m++) {
        //   ERT_PLOG_I << "  [bypass]: m = " << m
        //             << " in_bound = " << std::get<0>(left_ranges.at(m))
        //             << " s = " << std::get<1>(left_ranges.at(m))
        //             << " l = " << std::get<2>(left_ranges.at(m))
        //             << " offset = " << std::get<3>(left_ranges.at(m))
        //             << " obs_id = " << std::get<4>(left_ranges.at(m))->id()
        //             << " adc_ts_index = " << std::get<5>(left_ranges.at(m))
        //             ;
        // }
      } else if (is_in_bound) {
        // ERT_PLOG_I << "[bypass]: scenario 22222: " ;
        // 1）更新l
        if (is_right_bound) {
          std::get<2>(right_ranges.back()) = std::max(std::get<2>(right_ranges.back()), start_l);
        } else {
          std::get<2>(left_ranges.back()) = std::min(std::get<2>(left_ranges.back()), start_l);
        }

        // 2）若障碍车实际（考虑buffer）在自车身后，则is_in_bound从true变为false，并增加截断的range
        if (start_s > upper_s) {
          is_in_bound = false;
          if (is_right_bound) {
            // is in bound, s, l, offset, obstacle_ptr, index(adc_ts_ranges_)
            auto right_range = right_ranges.back();
            std::get<0>(right_range) = 0;
            std::get<1>(right_range) = start_s;
            right_ranges.emplace_back(std::move(right_range));
          } else {
            // is in bound, s, l, offset, obstacle_ptr, index(adc_ts_ranges_)
            auto left_range = left_ranges.back();
            std::get<0>(left_range) = 0;
            std::get<1>(left_range) = start_s;
            left_ranges.emplace_back(std::move(left_range));
          }
          break;
        }

        // ERT_PLOG_I << "[bypass]: scenario 22222: right_ranges.size() = " << right_ranges.size() ;
        // for(int m = 0; m < right_ranges.size(); m++) {
        //   ERT_PLOG_I << "  [bypass]: m = " << m
        //             << " in_bound = " << std::get<0>(right_ranges.at(m))
        //             << " s = " << std::get<1>(right_ranges.at(m))
        //             << " l = " << std::get<2>(right_ranges.at(m))
        //             << " offset = " << std::get<3>(right_ranges.at(m))
        //             << " obs_id = " << std::get<4>(right_ranges.at(m))->id()
        //             << " adc_ts_index = " << std::get<5>(right_ranges.at(m))
        //             ;
        // }
        // ERT_PLOG_I << "[bypass]: scenario 22222: left_ranges.size() = " << left_ranges.size() ;
        // for(int m = 0; m < left_ranges.size(); m++) {
        //   ERT_PLOG_I << "  [bypass]: m = " << m
        //             << " in_bound = " << std::get<0>(left_ranges.at(m))
        //             << " s = " << std::get<1>(left_ranges.at(m))
        //             << " l = " << std::get<2>(left_ranges.at(m))
        //             << " offset = " << std::get<3>(left_ranges.at(m))
        //             << " obs_id = " << std::get<4>(left_ranges.at(m))->id()
        //             << " adc_ts_index = " << std::get<5>(left_ranges.at(m))
        //             ;
        // }
      } else {
        // ERT_PLOG_I << "[bypass]: scenario 33333: no process" ;
      }
    }

    // right_ranges fall back
    // 对经过速度差滞回锁定必须绕行的目标，但搜索不到目标在自车期望轨迹上的投影边界时，以目标轨迹在自车轨迹最远端的投影、作为虚拟边界，避免边界的跳动.
    // if (is_right_bound && right_ranges.empty()) {
    //   const auto& [final_start_t, final_end_t, final_start_s, final_end_s] = adc_ts_ranges_[adc_ts_ranges_.size() -
    //   1]; const auto& [initial_start_t, initial_end_t, initial_start_s, initial_end_s] = adc_ts_ranges_[0]; SLPoint
    //   final_start_sl; SLPoint initial_start_sl; bool flag_final_start_sl =
    //   ref_line.xy2sl(obstacle->getPointAtTime(final_start_t).path_point(), &final_start_sl); bool
    //   flag_initial_start_sl = ref_line.xy2sl(obstacle->getPointAtTime(initial_start_t).path_point(),
    //   &initial_start_sl); if (flag_final_start_sl && flag_initial_start_sl) {
    //     double final_lower_s = final_start_sl.s() - half_length - min_lon_buffer - dynamic_obstacle_lon_start_dis;
    //     double initial_start_l = initial_start_sl.l() + half_width + std::get<4>(dynamic_obs_to_nudge_buffer.at(k));
    //     // is in bound, s, l, offset, obstacle_ptr, index(adc_ts_ranges_)
    //     right_ranges.emplace_back(1, final_lower_s, initial_start_l, 0, obstacle.get(), adc_ts_ranges_.size() - 1);

    //     double initial_upper_s = initial_start_sl.s() + half_length;
    //     // 若障碍车实际（不考虑buffer）在自车身后
    //     if (initial_start_s  > initial_upper_s) {
    //       right_ranges.erase(right_ranges.end() - 1);
    //     }
    //   }

    //   if (!right_ranges.empty() && std::get<0>(right_ranges.back()) == 1 && flag_final_start_sl) {
    //     auto right = right_ranges.back();
    //     double final_upper_s = final_start_sl.s() + half_length + min_lon_buffer + dynamic_obstacle_lon_end_dis;
    //     std::get<0>(right) = 0;
    //     std::get<1>(right) = final_upper_s;
    //     right_ranges.emplace_back(std::move(right));
    //   }
    // }
    // ERT_PLOG_I << "[bypass]: fallback: right_ranges.size() = " << right_ranges.size() ;
    // for(int i = 0; i < right_ranges.size(); i++) {
    //   ERT_PLOG_I << "  [bypass]: i = " << i
    //             << " in_bound = " << std::get<0>(right_ranges.at(i))
    //             << " s = " << std::get<1>(right_ranges.at(i))
    //             << " l = " << std::get<2>(right_ranges.at(i))
    //             << " offset = " << std::get<3>(right_ranges.at(i))
    //             << " obs_id = " << std::get<4>(right_ranges.at(i))->id()
    //             << " adc_ts_index = " << std::get<5>(right_ranges.at(i))
    //             ;
    // }

    // left_ranges fall back
    // if (!is_right_bound && left_ranges.empty()) {
    //   const auto& [final_start_t, final_end_t, final_start_s, final_end_s] = adc_ts_ranges_[adc_ts_ranges_.size() -
    //   1]; const auto& [initial_start_t, initial_end_t, initial_start_s, initial_end_s] = adc_ts_ranges_[0]; SLPoint
    //   final_start_sl; SLPoint initial_start_sl; bool flag_final_start_sl =
    //   ref_line.xy2sl(obstacle->getPointAtTime(final_start_t).path_point(), &final_start_sl); bool
    //   flag_initial_start_sl = ref_line.xy2sl(obstacle->getPointAtTime(initial_start_t).path_point(),
    //   &initial_start_sl); if (flag_final_start_sl && flag_initial_start_sl) {
    //     double final_lower_s = final_start_sl.s() - half_length - min_lon_buffer - dynamic_obstacle_lon_start_dis;
    //     double initial_start_l = initial_start_sl.l() - half_width - std::get<4>(dynamic_obs_to_nudge_buffer.at(k));
    //     // is in bound, s, l, offset, obstacle_ptr, index(adc_ts_ranges_)
    //     left_ranges.emplace_back(1, final_lower_s, initial_start_l, 0, obstacle.get(), adc_ts_ranges_.size() - 1);

    //     double initial_upper_s = initial_start_sl.s() + half_length;
    //     // 若障碍车实际（不考虑buffer）在自车身后
    //     if (initial_start_s > initial_upper_s) {
    //       left_ranges.erase(left_ranges.end() - 1);
    //     }
    //   }

    //   if (!left_ranges.empty() && std::get<0>(left_ranges.back()) == 1 && flag_final_start_sl) {
    //     auto left = left_ranges.back();
    //     double final_upper_s = final_start_sl.s() + half_length + min_lon_buffer + dynamic_obstacle_lon_end_dis;
    //     std::get<0>(left) = 0;
    //     std::get<1>(left) = final_upper_s;
    //     left_ranges.emplace_back(std::move(left));
    //   }
    // }
    // ERT_PLOG_I << "[bypass]: fallback: left_ranges.size() = " << left_ranges.size() ;
    // for(int i = 0; i < left_ranges.size(); i++) {
    //   ERT_PLOG_I << "  [bypass]: i = " << i
    //             << " in_bound = " << std::get<0>(left_ranges.at(i))
    //             << " s = " << std::get<1>(left_ranges.at(i))
    //             << " l = " << std::get<2>(left_ranges.at(i))
    //             << " offset = " << std::get<3>(left_ranges.at(i))
    //             << " obs_id = " << std::get<4>(left_ranges.at(i))->id()
    //             << " adc_ts_index = " << std::get<5>(left_ranges.at(i))
    //             ;
    // }

    // add last element with is_in_bound is false
    if (is_in_bound) {
      if (!left_ranges.empty()) {
        if (std::get<0>(left_ranges.back()) == 1) {
          auto left = left_ranges.back();
          std::get<0>(left) = 0;
          std::get<1>(left) = std::get<3>(adc_ts_ranges_.back()) + 2.0;
          left_ranges.emplace_back(std::move(left));
        }
      }
      if (!right_ranges.empty()) {
        if (std::get<0>(right_ranges.back()) == 1) {
          auto right = right_ranges.back();
          std::get<0>(right) = 0;
          std::get<1>(right) = std::get<3>(adc_ts_ranges_.back()) + 2.0;
          right_ranges.emplace_back(std::move(right));
        }
      }
    }

    // ERT_PLOG_I << "[bypass]: add last element: right_ranges.size() = " << right_ranges.size() ;
    // for(int i = 0; i < right_ranges.size(); i++) {
    //   ERT_PLOG_I << "  [bypass]: i = " << i
    //             << " in_bound = " << std::get<0>(right_ranges.at(i))
    //             << " s = " << std::get<1>(right_ranges.at(i))
    //             << " l = " << std::get<2>(right_ranges.at(i))
    //             << " offset = " << std::get<3>(right_ranges.at(i))
    //             << " obs_id = " << std::get<4>(right_ranges.at(i))->id()
    //             << " adc_ts_index = " << std::get<5>(right_ranges.at(i))
    //             ;
    // }
    // ERT_PLOG_I << "[bypass]: add last element: left_ranges.size() = " << left_ranges.size() ;
    // for(int i = 0; i < left_ranges.size(); i++) {
    //   ERT_PLOG_I << "  [bypass]: i = " << i
    //             << " in_bound = " << std::get<0>(left_ranges.at(i))
    //             << " s = " << std::get<1>(left_ranges.at(i))
    //             << " l = " << std::get<2>(left_ranges.at(i))
    //             << " offset = " << std::get<3>(left_ranges.at(i))
    //             << " obs_id = " << std::get<4>(left_ranges.at(i))->id()
    //             << " adc_ts_index = " << std::get<5>(left_ranges.at(i))
    //             ;
    // }
  }

  // sorter
  auto sorter = [](const DynamicObstacleRange& lhs, const DynamicObstacleRange& rhs) {
    if (std::get<1>(lhs) != std::get<1>(rhs)) {
      return std::get<1>(lhs) < std::get<1>(rhs);
    }
    return std::get<0>(lhs) > std::get<0>(rhs);
  };
  std::sort(right_ranges.begin(), right_ranges.end(), sorter);
  std::sort(left_ranges.begin(), left_ranges.end(), sorter);

  // ERT_PLOG_I << "[bypass]: final: right_ranges.size() = " << right_ranges.size() ;
  // for(int i = 0; i < right_ranges.size(); i++) {
  //   ERT_PLOG_I << "  [bypass]: i = " << i
  //             << " in_bound = " << std::get<0>(right_ranges.at(i))
  //             << " s = " << std::get<1>(right_ranges.at(i))
  //             << " l = " << std::get<2>(right_ranges.at(i))
  //             << " offset = " << std::get<3>(right_ranges.at(i))
  //             << " obs_id = " << std::get<4>(right_ranges.at(i))->id()
  //             << " adc_ts_index = " << std::get<5>(right_ranges.at(i))
  //             ;
  // }
  // ERT_PLOG_I << "[bypass]: final: left_ranges.size() = " << left_ranges.size() ;
  // for(int i = 0; i < left_ranges.size(); i++) {
  //   ERT_PLOG_I << "  [bypass]: i = " << i
  //             << " in_bound = " << std::get<0>(left_ranges.at(i))
  //             << " s = " << std::get<1>(left_ranges.at(i))
  //             << " l = " << std::get<2>(left_ranges.at(i))
  //             << " offset = " << std::get<3>(left_ranges.at(i))
  //             << " obs_id = " << std::get<4>(left_ranges.at(i))->id()
  //             << " adc_ts_index = " << std::get<5>(left_ranges.at(i))
  //             ;
  // }

  return res;
}

/**
 * @brief 基于速度滞回筛选需要动态绕行的动态障碍物
 * @details
 * 该函数用于根据速度滞回机制筛选出需要动态绕行的动态障碍物。通过比较障碍物与自车的速度差异，并结合历史帧的速度信息，确定哪些障碍物需要进行动态绕行。
 *
 * @param[in] ref_line 参考线，表示当前路径的中心线
 * @param[in] ego_veloc 自车速度，用于与障碍物速度进行比较
 * @param[in] decision_obstacles 动态障碍物集合，包含所有需要处理的动态障碍物
 * @param[in] buffer 缓冲区信息集合，包含每个障碍物的起始缓冲区、结束缓冲区和横向缓冲区
 *
 * @par 关键变量说明:
 * - hysteresis_delta_velocity_map_: 速度滞回映射表，用于存储障碍物的速度滞回信息
 * - last_ids: 上一帧的障碍物ID集合
 * - current_ids: 当前帧的障碍物ID集合
 * - dynamic_obs_to_nudge: 需要动态绕行的动态障碍物集合
 * - dynamic_obs_to_nudge_buffer: 需要动态绕行的动态障碍物的缓冲区信息
 *
 * @par 处理流程:
 * 1. 检查动态障碍物集合是否为空，若为空则直接返回
 * 2. 比较上一帧和当前帧的障碍物ID集合，更新速度滞回映射表
 * 3. 遍历所有动态障碍物，根据速度滞回机制筛选出需要动态绕行的障碍物
 * 4. 返回筛选后的动态障碍物集合及其对应的缓冲区信息
 *
 * @par 流程图:
 * @startuml
 * start
 * :检查动态障碍物集合是否为空;
 * if (为空) then (yes)
 *   :直接返回;
 * else (no)
 *   :比较上一帧和当前帧的障碍物ID集合;
 *   :更新速度滞回映射表;
 *   :遍历所有动态障碍物;
 *   :根据速度滞回机制筛选出需要动态绕行的障碍物;
 *   :返回筛选后的动态障碍物集合及其对应的缓冲区信息;
 * endif
 * stop
 * @enduml
 *
 * @return std::pair<std::vector<std::shared_ptr<Decision::DecisionObject>>, std::vector<std::tuple<double, double,
 * double, double, double, double, double>>> 返回包含需要动态绕行的动态障碍物集合及其对应的缓冲区信息的结构体
 *
 * @note 该函数应在路径规划过程中调用，确保动态绕行障碍物正确筛选
 *
 * @warning 需确保传入的参考线、自车速度、动态障碍物集合和缓冲区信息集合有效
 */
std::pair<std::vector<std::shared_ptr<Decision::DecisionObject>>,
          std::vector<std::tuple<double, double, double, double, double, double, double>>>
PathBoundParser::pickDynamicObstaclesForDynamicNudge(
    const ReferenceLine& ref_line, const double& ego_veloc,
    const std::vector<std::shared_ptr<Decision::DecisionObject>>& decision_obstacles,
    const std::vector<std::tuple<double, double, double, double, double, double, double>>& buffer) {
  if (decision_obstacles.empty()) {
    return std::make_pair<std::vector<std::shared_ptr<Decision::DecisionObject>>,
                          std::vector<std::tuple<double, double, double, double, double, double, double>>>(
        std::vector<std::shared_ptr<Decision::DecisionObject>>(decision_obstacles),
        std::vector<std::tuple<double, double, double, double, double, double, double>>(buffer));
  }

  // 考虑速度滞回，筛选出最终需要动态nudge的dynamic_obs（速度低于自车），输出至dynamic_obs_to_nudge
  std::vector<std::string> last_ids, current_ids;
  // ERT_PLOG_I << "[PathBoundParser::pickNpcsForDynamicNudge]: hysteresis_delta_velocity_map_ size = " <<
  // hysteresis_delta_velocity_map_.size() ;
  for (auto itm : hysteresis_delta_velocity_map_) {
    last_ids.push_back(itm.first);
  }
  for (auto itm : decision_obstacles) {
    current_ids.push_back(itm->id);
  }
  // 1）相比历史帧，本帧消失的dynamic_obs
  for (auto last_id : last_ids) {
    if (std::find(current_ids.begin(), current_ids.end(), last_id) == current_ids.end()) {
      hysteresis_delta_velocity_map_.erase(last_id);
    }
  }
  // 2）相比历史帧，本帧新出现的dynamic_obs
  for (auto current_id : current_ids) {
    if (std::find(last_ids.begin(), last_ids.end(), current_id) == last_ids.end()) {
      hysteresis_delta_velocity_map_[current_id] =
          util::HysteresisFlag<double>(std::get<5>(buffer.front()), std::get<6>(buffer.front()));
    }
  }
  // 3）最终需要动态nudge的dynamic_obs（速度低于自车）
  std::vector<std::shared_ptr<Decision::DecisionObject>> dynamic_obs_to_nudge;
  std::vector<std::tuple<double, double, double, double, double, double, double>> dynamic_obs_to_nudge_buffer;
  for (int i = 0; i < decision_obstacles.size(); i++) {
    const auto& obs = decision_obstacles.at(i);
    // if (isReverse(ref_line, obs.get())) {
    //   // 逆向obs: 逆向方案上了之后再改回来, 目前是筛选出来逆向的动态车辆，不考虑速度滞回
    //   dynamic_obs_to_nudge.emplace_back(obs);
    //   dynamic_obs_to_nudge_buffer.emplace_back(buffer.at(i));
    // } else {
    //   //todo: 基于上一帧的速度轨迹与当前障碍物的预测轨迹，判断是否需要动态nudge
    //   std::vector<std::pair<double, double>> obs_traj;
    //   double t = 0.0;
    //   double dt = 0.1;
    //   for(auto pt : obs->getRawPredictionMultiModal().front().traj){
    //     auto ref_pt  = target_reference_line_.getReferencePoint(pt.path_point().x(), pt.path_point().y());
    //     obs_traj.emplace_back(t,(ref_pt.local_s() - adc_frenet_s_));
    //     t += dt;
    //   }
    //   double obs_front = 0.5 * obs->cur_box.half_length();
    //   double obs_rear = 0.5 * obs->cur_box.half_length();
    //   auto hasreached = [this, obs_front, obs_rear, &obs_traj]() -> bool {
    //     const size_t min_len = std::min(adc_speed_traj_.size(), obs_traj.size());
    //     for (size_t i = 0; i < min_len; ++i) {
    //         const double s1 = adc_speed_traj_[i].second;
    //         const double s2 = obs_traj[i].second;

    //         const double ego_start = s1 - adc_rear_length_;
    //         const double ego_end = s1 + adc_front_length_;
    //         const double obs_start = s2 - obs_rear;
    //         const double obs_end = s2 + obs_front;

    //         if (ego_start <= obs_end && obs_start <= ego_end) {
    //             return true;
    //         }
    //     }
    //     return false;
    // };
    //   bool is_npc_slower_than_ego = hysteresis_delta_velocity_map_[obs->id()](hasreached());
    //   if (is_npc_slower_than_ego) {
    //     dynamic_obs_to_nudge.emplace_back(obs);
    //     dynamic_obs_to_nudge_buffer.emplace_back(buffer.at(i));
    //   }
    // }
    dynamic_obs_to_nudge.emplace_back(obs);
    dynamic_obs_to_nudge_buffer.emplace_back(buffer.at(i));
  }
  ERT_PLOG_I << "[bypass]:[PathBoundParser::pickNpcsForDynamicNudge]: dynamic_obs_to_nudge size = "
             << dynamic_obs_to_nudge.size()
             << " dynamic_obs_to_nudge_buffer size = " << dynamic_obs_to_nudge_buffer.size();
  for (int i = 0; i < dynamic_obs_to_nudge.size(); i++) {
    ERT_PLOG_I << "  [bypass]: id = " << dynamic_obs_to_nudge.at(i)->id
              << "  lateral_tag = " << static_cast<int>(dynamic_obs_to_nudge.at(i)->lat_od_tag) << "  speed_kph = "
              << dynamic_obs_to_nudge.at(i)->spd * MS_KMH
              // << "  start_l = " << obs->cur_sl_bound.start_l()
              // << "  end_l = " << obs->cur_sl_bound.end_l()
              // << "  start_s = " << obs->cur_sl_bound.start_s()
              // << "  end_s = " << obs->cur_sl_bound.end_s()
              << "  start_s_buffer = " << std::get<0>(dynamic_obs_to_nudge_buffer.at(i))
              << "  start_pre_time = " << std::get<1>(dynamic_obs_to_nudge_buffer.at(i))
              << "  start_pre_s = " << std::get<1>(dynamic_obs_to_nudge_buffer.at(i)) * std::fabs(adc_frenet_sd_)
              << "  end_s_buffer = " << std::get<2>(dynamic_obs_to_nudge_buffer.at(i))
              << "  end_pre_time = " << std::get<3>(dynamic_obs_to_nudge_buffer.at(i))
              << "  end_pre_s = " << std::get<3>(dynamic_obs_to_nudge_buffer.at(i)) * std::fabs(adc_frenet_sd_)
              << "  l_buffer = " << std::get<4>(dynamic_obs_to_nudge_buffer.at(i)) ;
  }

  return std::make_pair<std::vector<std::shared_ptr<Decision::DecisionObject>>,
                        std::vector<std::tuple<double, double, double, double, double, double, double>>>(
      std::move(dynamic_obs_to_nudge), std::move(dynamic_obs_to_nudge_buffer));
}

/**
 * @brief 从自由空间获取边界信息
 * @details
 * 该函数用于从自由空间中获取边界信息，包括左侧和右侧的边界范围。通过遍历采样点，计算其在自由空间中的投影，并根据自由空间的边界信息确定是否需要扩展边界。
 *
 * @param[in] freespace 自由空间，表示当前路径的自由空间信息
 * @param[in] buffer 缓冲区，用于扩展自由空间的边界
 *
 * @par 关键变量说明:
 * - res: 返回的边界信息，包含左侧和右侧的边界范围
 * - freespace_bound: 自由空间的边界信息
 * - ego_sample_pt: 自车采样点
 * - bound: 当前路径点的边界信息
 * - bound_s: 当前路径点的s值
 * - bound_right: 当前路径点的右侧边界
 * - bound_left: 当前路径点的左侧边界
 * - search_range: 搜索范围，用于确定自由空间的边界
 * - fs_barrier_lower: 自由空间的左侧边界
 * - fs_barrier_upper: 自由空间的右侧边界
 *
 * @par 处理流程:
 * 1. 遍历所有采样点
 * 2. 判断采样点是否在地图外，若在地图外则直接跳过
 * 3. 计算当前路径点的边界信息
 * 4. 根据自由空间的边界信息确定是否需要扩展边界
 * 5. 返回包含左侧和右侧边界信息的结构体
 *
 * @par 流程图:
 * @startuml
 * start
 * :遍历所有采样点;
 * :判断采样点是否在地图外;
 * if (在地图外) then (yes)
 *   :跳过;
 * else (no)
 *   :计算当前路径点的边界信息;
 *   :根据自由空间的边界信息确定是否需要扩展边界;
 * endif
 * :返回包含左侧和右侧边界信息的结构体;
 * stop
 * @enduml
 *
 * @return FreespaceBoundaryInfo 返回包含左侧和右侧边界信息的结构体
 *
 * @note 该函数应在路径规划过程中调用，确保自由空间边界信息正确生成
 *
 * @warning 需确保传入的自由空间和缓冲区有效
 */
FreespaceBoundaryInfo PathBoundParser::getBoundInfoFromFreespace(const Freespace& freespace, const double buffer) {
  FreespaceBoundaryInfo res;
  auto& freespace_bound = res.bound;
  for (int i = 0; i < sample_points_.size(); i++) {
    auto& ego_sample_pt = ego_sample_points_[i];
    // extend freespace bound when out of map to avoid undetected hard boundary
    if (freespace.isOutOfMap(ego_sample_pt.x(), ego_sample_pt.y())) {
      break;
    } else {
      auto [bound, has_max_bound] = check_ranges_.interpolate(sample_points_[i].local_s());
      auto [bound_s, bound_right, bound_left] = bound;
      std::pair<double, double> search_range(bound_left, bound_right);
      auto [fs_barrier_lower, fs_barrier_upper] =
          freespace.getLateralBoundary(ego_sample_pt, ego_sample_pt.heading(), search_range, buffer);
      freespace_bound.emplace_back(ego_sample_pt.local_s(), fs_barrier_lower - buffer, fs_barrier_upper + buffer);
      // ERT_PLOG_I << "aaaa bound = " << ego_sample_pt.local_s() << "  " << fs_barrier_lower - buffer << "  " <<
      // fs_barrier_upper + buffer;
    }
  }
  return res;
}

/**
 * @brief 从自由空间获取边界信息（带偏移量）
 * @details
 * 该函数用于从自由空间中获取边界信息，包括左侧和右侧的边界范围。通过遍历采样点，并根据偏移量调整采样点的位置，计算其在自由空间中的投影，并根据自由空间的边界信息确定是否需要扩展边界。
 *
 * @param[in] freespace 自由空间，表示当前路径的自由空间信息
 * @param[in] sample_offsets 采样点偏移量集合，用于调整采样点的位置
 * @param[in] buffer 缓冲区，用于扩展自由空间的边界
 *
 * @par 关键变量说明:
 * - res: 返回的边界信息，包含左侧和右侧的边界范围
 * - freespace_bound: 自由空间的边界信息
 * - ego_sample_pt: 自车采样点
 * - offset: 采样点的偏移量
 * - check_pt: 调整后的采样点
 * - bound: 当前路径点的边界信息
 * - bound_s: 当前路径点的s值
 * - bound_right: 当前路径点的右侧边界
 * - bound_left: 当前路径点的左侧边界
 * - search_range: 搜索范围，用于确定自由空间的边界
 * - fs_barrier_lower: 自由空间的左侧边界
 * - fs_barrier_upper: 自由空间的右侧边界
 *
 * @par 处理流程:
 * 1. 遍历所有采样点
 * 2. 根据偏移量调整采样点的位置
 * 3. 判断调整后的采样点是否在地图外，若在地图外则直接跳过
 * 4. 计算当前路径点的边界信息
 * 5. 根据自由空间的边界信息确定是否需要扩展边界
 * 6. 返回包含左侧和右侧边界信息的结构体
 *
 * @par 流程图:
 * @startuml
 * start
 * :遍历所有采样点;
 * :根据偏移量调整采样点的位置;
 * :判断调整后的采样点是否在地图外;
 * if (在地图外) then (yes)
 *   :跳过;
 * else (no)
 *   :计算当前路径点的边界信息;
 *   :根据自由空间的边界信息确定是否需要扩展边界;
 * endif
 * :返回包含左侧和右侧边界信息的结构体;
 * stop
 * @enduml
 *
 * @return FreespaceBoundaryInfo 返回包含左侧和右侧边界信息的结构体
 *
 * @note 该函数应在路径规划过程中调用，确保自由空间边界信息正确生成
 *
 * @warning 需确保传入的自由空间、采样点偏移量和缓冲区有效
 */
FreespaceBoundaryInfo PathBoundParser::getBoundInfoFromFreespace(const Freespace& freespace,
                                                                 const std::vector<double>& sample_offsets,
                                                                 const double buffer) {
  FreespaceBoundaryInfo res;
  auto& freespace_bound = res.bound;
  for (int i = 0; i < sample_points_.size(); i++) {
    auto& ego_sample_pt = ego_sample_points_[i];
    double offset = 0.0;
    if (i < sample_offsets.size()) {
      offset = sample_offsets[i];
    }
    ReferencePoint check_pt = ego_sample_pt.lateralShift(offset);
    // extend freespace bound when out of map to avoid undetected hard boundary
    if (freespace.isOutOfMap(check_pt.x(), check_pt.y())) {
      continue;
    } else {
      auto [bound, has_max_bound] = check_ranges_.interpolate(sample_points_[i].local_s());
      auto [bound_s, bound_right, bound_left] = bound;
      std::pair<double, double> search_range(bound_left - offset, bound_right - offset);
      auto [fs_barrier_lower, fs_barrier_upper] =
          freespace.getLateralBoundary(check_pt, check_pt.heading(), search_range, buffer);
      freespace_bound.emplace_back(ego_sample_pt.local_s(), fs_barrier_lower + offset - buffer,
                                   fs_barrier_upper + offset + buffer);
    }
  }
  return res;
}

/**
 * @brief 从自由空间获取边界信息（带忽略范围和偏移量）
 * @details
 * 该函数用于从自由空间中获取边界信息，包括左侧和右侧的边界范围。通过遍历采样点，并根据偏移量调整采样点的位置，同时考虑忽略范围，计算其在自由空间中的投影，并根据自由空间的边界信息确定是否需要扩展边界。
 *
 * @param[in] freespace 自由空间，表示当前路径的自由空间信息
 * @param[in] ignore_ranges 忽略范围信息集合，包含需要忽略的路径范围
 * @param[in] sample_offsets 采样点偏移量集合，用于调整采样点的位置
 * @param[in] buffer 缓冲区，用于扩展自由空间的边界
 *
 * @par 关键变量说明:
 * - res: 返回的边界信息，包含左侧和右侧的边界范围
 * - freespace_bound: 自由空间的边界信息
 * - ego_sample_pt: 自车采样点
 * - offset: 采样点的偏移量
 * - check_pt: 调整后的采样点
 * - bound: 当前路径点的边界信息
 * - bound_s: 当前路径点的s值
 * - bound_right: 当前路径点的右侧边界
 * - bound_left: 当前路径点的左侧边界
 * - search_range: 搜索范围，用于确定自由空间的边界
 * - fs_barrier_lower: 自由空间的左侧边界
 * - fs_barrier_upper: 自由空间的右侧边界
 *
 * @par 处理流程:
 * 1. 遍历所有采样点
 * 2. 根据偏移量调整采样点的位置
 * 3. 判断采样点是否在忽略范围内，若在忽略范围内则直接跳过
 * 4. 判断调整后的采样点是否在地图外，若在地图外则直接跳过
 * 5. 计算当前路径点的边界信息
 * 6. 根据自由空间的边界信息确定是否需要扩展边界
 * 7. 返回包含左侧和右侧边界信息的结构体
 *
 * @par 流程图:
 * @startuml
 * start
 * :遍历所有采样点;
 * :根据偏移量调整采样点的位置;
 * :判断采样点是否在忽略范围内;
 * if (在忽略范围内) then (yes)
 *   :跳过;
 * else (no)
 *   :判断调整后的采样点是否在地图外;
 *   if (在地图外) then (yes)
 *     :跳过;
 *   else (no)
 *     :计算当前路径点的边界信息;
 *     :根据自由空间的边界信息确定是否需要扩展边界;
 *   endif
 * endif
 * :返回包含左侧和右侧边界信息的结构体;
 * stop
 * @enduml
 *
 * @return FreespaceBoundaryInfo 返回包含左侧和右侧边界信息的结构体
 *
 * @note 该函数应在路径规划过程中调用，确保自由空间边界信息正确生成
 *
 * @warning 需确保传入的自由空间、忽略范围信息、采样点偏移量和缓冲区有效
 */
FreespaceBoundaryInfo PathBoundParser::getBoundInfoFromFreespace(const Freespace& freespace,
                                                                 const std::vector<IgnoreRangeInfo>& ignore_ranges,
                                                                 const std::vector<double>& sample_offsets,
                                                                 const double buffer) {
  FreespaceBoundaryInfo res;
  auto& freespace_bound = res.bound;
  for (int i = 0; i < sample_points_.size(); i++) {
    auto& ego_sample_pt = ego_sample_points_[i];
    // if (pp_tools::inIgnoreRange(ignore_ranges, ego_sample_pt.local_s())) continue;
    double offset = 0.0;
    if (i < sample_offsets.size()) {
      offset = sample_offsets[i];
    }
    ReferencePoint check_pt = ego_sample_pt.lateralShift(offset);
    // extend freespace bound when out of map to avoid undetected hard boundary
    if (freespace.isOutOfMap(check_pt.x(), check_pt.y())) {
      continue;
    } else {
      auto [bound, has_max_bound] = check_ranges_.interpolate(sample_points_[i].local_s());
      auto [bound_s, bound_right, bound_left] = bound;
      std::pair<double, double> search_range(bound_left - offset, bound_right - offset);
      auto [fs_barrier_lower, fs_barrier_upper] =
          freespace.getLateralBoundary(check_pt, check_pt.heading(), search_range, buffer);
      freespace_bound.emplace_back(ego_sample_pt.local_s(), fs_barrier_lower + offset - buffer,
                                   fs_barrier_upper + offset + buffer);
    }
  }
  return res;
}

/**
 * @brief 从自由空间获取边界信息（带偏移量和纵向缓冲区）
 * @details
 * 该函数用于从自由空间中获取边界信息，包括左侧和右侧的边界范围。通过遍历采样点，并根据偏移量调整采样点的位置，同时考虑纵向缓冲区，计算其在自由空间中的投影，并根据自由空间的边界信息确定是否需要扩展边界。
 *
 * @param[in] freespace 自由空间，表示当前路径的自由空间信息
 * @param[in] sample_offsets 采样点偏移量集合，用于调整采样点的位置
 * @param[in] buffer 缓冲区，用于扩展自由空间的边界
 * @param[in] start_buffer 起始纵向缓冲区，用于扩展起始边界
 * @param[in] end_buffer 结束纵向缓冲区，用于扩展结束边界
 *
 * @par 关键变量说明:
 * - res: 返回的边界信息，包含左侧和右侧的边界范围
 * - freespace_bound: 自由空间的边界信息
 * - ego_sample_pt: 自车采样点
 * - offset: 采样点的偏移量
 * - check_pt: 调整后的采样点
 * - bound: 当前路径点的边界信息
 * - bound_s: 当前路径点的s值
 * - bound_right: 当前路径点的右侧边界
 * - bound_left: 当前路径点的左侧边界
 * - search_range: 搜索范围，用于确定自由空间的边界
 * - fs_barrier_lower: 自由空间的左侧边界
 * - fs_barrier_upper: 自由空间的右侧边界
 *
 * @par 处理流程:
 * 1. 调用不带纵向缓冲区的函数获取原始边界信息
 * 2. 根据起始和结束纵向缓冲区调整边界信息
 * 3. 返回包含左侧和右侧边界信息的结构体
 *
 * @par 流程图:
 * @startuml
 * start
 * :调用不带纵向缓冲区的函数获取原始边界信息;
 * :根据起始和结束纵向缓冲区调整边界信息;
 * :返回包含左侧和右侧边界信息的结构体;
 * stop
 * @enduml
 *
 * @return FreespaceBoundaryInfo 返回包含左侧和右侧边界信息的结构体
 *
 * @note 该函数应在路径规划过程中调用，确保自由空间边界信息正确生成
 *
 * @warning 需确保传入的自由空间、采样点偏移量、缓冲区和纵向缓冲区有效
 */
FreespaceBoundaryInfo PathBoundParser::getBoundInfoFromFreespace(const Freespace& freespace,
                                                                 const std::vector<double>& sample_offsets,
                                                                 const double buffer, const double start_buffer,
                                                                 const double end_buffer) {
  auto raw = getBoundInfoFromFreespace(freespace, sample_offsets, buffer);
  return getFreespaceBoundInfoWithLongiBuffer(raw, start_buffer, end_buffer);
}

/**
 * @brief 根据纵向缓冲区调整自由空间边界信息
 * @details
 * 该函数用于根据起始和结束纵向缓冲区调整自由空间的边界信息。通过遍历边界信息，并根据纵向缓冲区扩展边界范围，确保边界信息能够覆盖缓冲区范围。
 *
 * @param[in] boundary 原始边界信息，包含左侧和右侧的边界范围
 * @param[in] start_buffer 起始纵向缓冲区，用于扩展起始边界
 * @param[in] end_buffer 结束纵向缓冲区，用于扩展结束边界
 *
 * @par 关键变量说明:
 * - res: 返回的边界信息，包含调整后的左侧和右侧边界范围
 * - right_bounds: 右侧边界集合，用于存储右侧障碍物的边界信息
 * - left_bounds: 左侧边界集合，用于存储左侧障碍物的边界信息
 * - insert_bound_idx: 插入边界信息的索引
 * - erase_bound_idx: 删除边界信息的索引
 * - curr_s: 当前路径点的s值
 * - curr_right: 当前路径点的右侧边界
 * - curr_left: 当前路径点的左侧边界
 * - raw_s: 原始边界信息的s值
 * - raw_right: 原始边界信息的右侧边界
 * - raw_left: 原始边界信息的左侧边界
 *
 * @par 处理流程:
 * 1. 初始化返回结果和边界集合
 * 2. 遍历所有路径点，计算当前路径点的边界信息
 * 3. 根据起始纵向缓冲区插入边界信息
 * 4. 根据结束纵向缓冲区删除边界信息
 * 5. 更新当前路径点的边界信息
 * 6. 返回调整后的边界信息
 *
 * @par 流程图:
 * @startuml
 * start
 * :初始化返回结果和边界集合;
 * :遍历所有路径点;
 * :根据起始纵向缓冲区插入边界信息;
 * :根据结束纵向缓冲区删除边界信息;
 * :更新当前路径点的边界信息;
 * :返回调整后的边界信息;
 * stop
 * @enduml
 *
 * @return FreespaceBoundaryInfo 返回包含调整后的左侧和右侧边界信息的结构体
 *
 * @note 该函数应在路径规划过程中调用，确保边界信息能够覆盖缓冲区范围
 *
 * @warning 需确保传入的原始边界信息和纵向缓冲区有效
 */
FreespaceBoundaryInfo PathBoundParser::getFreespaceBoundInfoWithLongiBuffer(const FreespaceBoundaryInfo& boundary,
                                                                            const double start_buffer,
                                                                            const double end_buffer) {
  FreespaceBoundaryInfo res(boundary);
  std::multiset<double, std::greater<double>> right_bounds;
  std::multiset<double> left_bounds;
  int insert_bound_idx = 0;
  int erase_bound_idx = 0;
  for (int i = 0; i < res.bound.size(); i++) {
    auto& [curr_s, curr_right, curr_left] = res.bound[i];
    while (insert_bound_idx < boundary.bound.size()) {
      auto& [raw_s, raw_right, raw_left] = boundary.bound[insert_bound_idx];
      double start_s = raw_s - start_buffer;
      if (start_s > curr_s) {
        break;
      }
      right_bounds.insert(raw_right);
      left_bounds.insert(raw_left);
      insert_bound_idx++;
    }
    while (erase_bound_idx < insert_bound_idx) {
      auto& [raw_s, raw_right, raw_left] = boundary.bound[erase_bound_idx];
      double end_s = raw_s + end_buffer;
      if (end_s > curr_s) {
        break;
      }
      right_bounds.erase(raw_right);
      left_bounds.erase(raw_left);
      erase_bound_idx++;
    }
    if (!right_bounds.empty()) {
      curr_right = *right_bounds.begin();
    }
    if (!left_bounds.empty()) {
      curr_left = *left_bounds.begin();
    }
  }
  return res;
}

/**
 * @brief 获取多段线缓冲区映射
 * @details 该函数将多段线信息和对应的缓冲区信息映射到一个unordered_map中，键为多段线ID，值为对应的缓冲区信息。
 * @param polylines 多段线信息集合
 * @param buffer 缓冲区信息集合
 * @return std::unordered_map<std::string, std::tuple<double, double, double>> 返回多段线ID和对应缓冲区信息的映射
 * @par 流程图:
 * @startuml
 * start
 * :遍历多段线信息;
 * :将多段线ID和对应的缓冲区信息存入unordered_map;
 * :返回unordered_map;
 * stop
 * @enduml
 */
std::unordered_map<std::string, std::tuple<double, double, double>> PathBoundParser::getPolylineBufferMap(
    const std::vector<PolylineInput>& polylines, const std::vector<std::tuple<double, double, double>>& buffer) {
  std::unordered_map<std::string, std::tuple<double, double, double>> res;
  for (size_t i = 0; i < polylines.size(); i++) {
    res.emplace(polylines[i].id, buffer[i]);
  }
  return std::move(res);
}

/**
 * @brief 对多段线范围进行排序和分类
 * @details 该函数遍历多段线信息，计算每个多段线的范围，并根据偏移量进行分类和排序。
 * @param polylines 多段线信息集合
 * @param resolution_thrd 分辨率阈值，用于判断多段线是否需要进一步处理
 * @param ptr_small 可选参数，用于存储小范围的多段线信息
 * @return std::pair<std::vector<PolylineRange>, std::vector<PolylineRange>> 返回左侧和右侧的多段线范围
 * @par 流程图:
 * @startuml
 * start
 * :遍历多段线信息;
 * :计算每个多段线的范围;
 * if (多段线在采样点范围内) then (yes)
 *   :根据偏移量进行分类和排序;
 * else (no)
 *   :跳过当前多段线;
 * endif
 * :返回左侧和右侧的多段线范围;
 * stop
 * @enduml
 */
std::pair<std::vector<PolylineRange>, std::vector<PolylineRange>> PathBoundParser::sortPolylineRange(
    const std::vector<PolylineInput>& polylines, const double resolution_thrd, std::vector<PolylineInput>* ptr_small) {
  std::pair<std::vector<PolylineRange>, std::vector<PolylineRange>> res;
  auto& [left_ranges, right_ranges] = res;

  ReferencePoint end_point = sample_points_.back();

  for (size_t i = 0; i < polylines.size(); i++) {
    const auto& polyline = polylines[i];

    if (polyline.start_s > end_point.local_s() || polyline.end_s < sample_points_.front().local_s()) {
      continue;
    }

    const auto nudge_tag = polyline.nudge_type;

    if (polyline.end_s - polyline.start_s > resolution_thrd) {
      switch (nudge_tag) {
        case PolylineInput::NudgeType::LEFT_BYPASS:
          right_ranges.emplace_back(1, polyline.start_s, &polyline, i);
          right_ranges.emplace_back(0, polyline.end_s, &polyline, i);
          break;
        case PolylineInput::NudgeType::RIGHT_BYPASS:
          left_ranges.emplace_back(1, polyline.start_s, &polyline, i);
          left_ranges.emplace_back(0, polyline.end_s, &polyline, i);
          break;
        case PolylineInput::NudgeType::UNKNOWN:
          break;
      }
    } else if (ptr_small != nullptr) {
      ptr_small->emplace_back(polyline);
    }
  }

  auto sorter = [](const PolylineRange& lhs, const PolylineRange& rhs) {
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
 * @brief 对多段线边界范围进行排序和分类
 * @details 该函数遍历多段线信息，计算每个多段线的边界范围，并根据偏移量进行分类和排序。
 * @param polylines 多段线信息集合
 * @param buffer_map 缓冲区映射，包含多段线ID和对应的缓冲区信息
 * @return std::pair<std::vector<PolylineBoundRange>, std::vector<PolylineBoundRange>> 返回左侧和右侧的多段线边界范围
 * @par 流程图:
 * @startuml
 * start
 * :遍历多段线信息;
 * :计算每个多段线的边界范围;
 * if (多段线在采样点范围内) then (yes)
 *   :根据偏移量进行分类和排序;
 * else (no)
 *   :跳过当前多段线;
 * endif
 * :返回左侧和右侧的多段线边界范围;
 * stop
 * @enduml
 */
std::pair<std::vector<PolylineBoundRange>, std::vector<PolylineBoundRange>> PathBoundParser::sortPolylineBoundRange(
    const std::vector<PolylineInput>& polylines,
    const std::unordered_map<std::string, std::tuple<double, double, double>>& buffer_map) {
  std::pair<std::vector<PolylineBoundRange>, std::vector<PolylineBoundRange>> res;
  auto& [left_ranges, right_ranges] = res;

  for (size_t i = 0; i < polylines.size(); i++) {
    const auto& polyline = polylines[i];
    const auto& [start_buffer, end_buffer, lat_buffer] = buffer_map.at(polyline.id);
    if (polyline.start_s - start_buffer > sample_points_.back().local_s() ||
        polyline.end_s + end_buffer < sample_points_.front().local_s()) {
      continue;
    }

    const auto nudge_tag = polyline.nudge_type;

    switch (nudge_tag) {
      case PolylineInput::NudgeType::LEFT_BYPASS:
        right_ranges.emplace_back(1, polyline.start_s - start_buffer, polyline.end_l + lat_buffer, polyline.id);
        right_ranges.emplace_back(0, polyline.end_s + end_buffer, polyline.end_l + lat_buffer, polyline.id);
        break;
      case PolylineInput::NudgeType::RIGHT_BYPASS:
        left_ranges.emplace_back(1, polyline.start_s - start_buffer, polyline.start_l - lat_buffer, polyline.id);
        left_ranges.emplace_back(0, polyline.end_s + end_buffer, polyline.start_l - lat_buffer, polyline.id);
        break;
      case PolylineInput::NudgeType::UNKNOWN:
        break;
    }
  }

  auto sorter = [](const PolylineBoundRange& lhs, const PolylineBoundRange& rhs) {
    if (std::get<1>(lhs) != std::get<1>(rhs)) return std::get<1>(lhs) < std::get<1>(rhs);
    return std::get<0>(lhs) > std::get<0>(rhs);
  };
  std::sort(right_ranges.begin(), right_ranges.end(), sorter);
  std::sort(left_ranges.begin(), left_ranges.end(), sorter);
  return res;
}

/**
 * @brief 从多段线中获取左侧偏移信息
 * @details 该函数遍历多段线信息，计算每个多段线与中心线段的偏移信息，并返回具有最小偏移的多段线ID和偏移值。
 * @param center 中心线段，用于计算偏移
 * @param polyline_borders 多段线信息映射，包含多段线ID和对应的边界信息
 * @par 流程图:
 * @startuml
 * start
 * :遍历多段线信息;
 * :计算每个多段线与中心线段的偏移信息;
 * if (偏移在允许范围内) then (yes)
 *   :更新最小偏移值和对应的多段线ID;
 * else (no)
 *   :继续下一个多段线;
 * endif
 * :返回具有最小偏移的多段线ID和偏移值;
 * stop
 * @enduml
 * @return std::tuple<std::string, double> 返回包含多段线ID和最小偏移值的元组
 */
std::tuple<std::string, double> PathBoundParser::getLeftOffsetInfoFromPolyline(
    const math::LineSegment2d& center, const PolylineInfoMap& polyline_borders) {
  std::tuple<std::string, double> res{"", std::numeric_limits<double>::max()};
  auto& [in_bound_obs, offset] = res;
  double left_offset_enable_radius = 10;
  for (const auto& [ptr_poly, poly_info] : polyline_borders) {
    const auto& [segments, lat_buffer] = poly_info;
    bool enable_left_offset = false;
    for (const auto& segment : *segments) {
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
    for (const auto& segment : *segments) {
      const auto& [min_offset, max_offset, in_range] = getOffset(center, segment);
      double obs_offset = min_offset - lat_buffer;
      if (in_range && obs_offset < offset) {
        in_bound_obs = ptr_poly;
        offset = obs_offset;
      }
    }
  }
  return res;
}

/**
 * @brief 从多段线中获取右侧偏移信息
 * @details 该函数遍历多段线信息，计算每个多段线与中心线段的偏移信息，并返回具有最大偏移的多段线ID和偏移值。
 * @param center 中心线段，用于计算偏移
 * @param polyline_info 多段线信息映射，包含多段线ID和对应的边界信息
 * @par 流程图:
 * @startuml
 * start
 * :遍历多段线信息;
 * :计算每个多段线与中心线段的偏移信息;
 * if (偏移在允许范围内) then (yes)
 *   :更新最大偏移值和对应的多段线ID;
 * else (no)
 *   :继续下一个多段线;
 * endif
 * :返回具有最大偏移的多段线ID和偏移值;
 * stop
 * @enduml
 * @return std::tuple<std::string, double> 返回包含多段线ID和最大偏移值的元组
 */
std::tuple<std::string, double> PathBoundParser::getRightOffsetInfoFromPolyline(const math::LineSegment2d& center,
                                                                                const PolylineInfoMap& polyline_info) {
  std::tuple<std::string, double> res{"", std::numeric_limits<double>::lowest()};
  auto& [in_bound_obs, offset] = res;
  double right_offset_enable_radius = 10;
  for (const auto& [ptr_poly, poly_info] : polyline_info) {
    const auto& [segments, lat_buffer] = poly_info;
    bool enable_right_offset = false;
    for (const auto& segment : *segments) {
      float dis_point_to_line = CalculatePointDistanceToLineSegment(
          segment,
          math::Vec2d((center.start().x() + center.end().x()) / 2.0, (center.start().y() + center.end().y()) / 2.0));
      if (dis_point_to_line < right_offset_enable_radius) {
        enable_right_offset = true;
        break;
      }
    }
    if (!enable_right_offset) {
      continue;
    }
    for (const auto& segment : *segments) {
      const auto& [min_offset, max_offset, in_range] = getOffset(center, segment);
      double obs_offset = max_offset + lat_buffer;
      if (in_range && obs_offset > offset) {
        in_bound_obs = ptr_poly;
        offset = obs_offset;
      }
    }
  }
  return res;
}

/**
 * @brief 从抽象的线段集合中获取边界信息。
 * @details 这是新增的、解耦的函数，它接收一组 PolylineInput。
 * 该函数将遍历所有采样点，并根据每个采样点的 s 值和对应的线段信息，计算出左侧和右侧的边界信息。
 * 它还会根据给定的缓冲区信息来调整边界范围。
 * 该函数的目的是为了从抽象的线段集合中获取边界信息，并返回一个包含左侧和右侧边界的 ObstacleBoundaryInfo 结构体。
 * @par 流程图:
 * @startuml
 * start
 * :遍历所有采样点;
 * :根据每个采样点的 s 值和对应的线段信息计算边界信息;
 * :根据缓冲区信息调整边界范围;
 * :返回包含左侧和右侧边界的 ObstacleBoundaryInfo 结构体;
 * stop
 * @enduml
 * @param polylines 输入的线段集合，每个线段包含起始和结束的 s 值、左侧和右侧的偏移量等信息。
 * @param buffer 每个线段的缓冲区信息，包含起始和结束的缓冲区以及横向偏移量。
 * @return ObstacleBoundaryInfo 返回包含左侧和右侧边界信息的 ObstacleBoundaryInfo 结构体。
 */
ObstacleBoundaryInfo PathBoundParser::getBoundaryFromPolyline(
    const std::vector<PolylineInput>& polylines, const std::vector<std::tuple<double, double, double>>& buffer) {
  ObstacleBoundaryInfo res;
  auto buffer_map = getPolylineBufferMap(polylines, buffer);

  std::vector<PolylineInput> small_polylines;
  auto [left_ranges, right_ranges] = sortPolylineRange(polylines, 2 * config_.resolution(), &small_polylines);

  auto [left_od_bounds, right_od_bounds] = sortPolylineBoundRange(small_polylines, buffer_map);

  int curr_idx_left = 0, curr_idx_right = 0;
  PolylineInfoMap left_obstacles, right_obstacles;

  for (size_t i = 0; i + 1 < sample_points_.size(); i++) {
    double curr_s = sample_points_[i].local_s();
    double next_s = sample_points_[i + 1].local_s();
    while (curr_idx_left < left_ranges.size()) {
      auto& [in_bound, s, ptr_poly, idx] = left_ranges[curr_idx_left];
      if (next_s < s) break;
      if (in_bound) {
        left_obstacles.emplace(ptr_poly->id, std::make_pair(&ptr_poly->segments, std::get<2>(buffer[idx])));
      } else {
        left_obstacles.erase(ptr_poly->id);
      }
      curr_idx_left++;
    }
    while (curr_idx_right < right_ranges.size()) {
      auto& [in_bound, s, ptr_poly, idx] = right_ranges[curr_idx_right];
      if (next_s < s) break;
      if (in_bound) {
        right_obstacles.emplace(ptr_poly->id, std::make_pair(&ptr_poly->segments, std::get<2>(buffer[idx])));
      } else {
        right_obstacles.erase(ptr_poly->id);
      }
      curr_idx_right++;
    }
    const auto& [right_in_bound_id, right_offset] =
        getRightOffsetInfoFromPolyline(sample_segments_[i], right_obstacles);
    if (right_in_bound_id != "") {
      const auto& [start_buffer, end_buffer, lat_buffer] = buffer_map.at(right_in_bound_id);
      right_od_bounds.emplace_back(1, curr_s - start_buffer, right_offset, right_in_bound_id);
      right_od_bounds.emplace_back(0, curr_s + end_buffer, right_offset, right_in_bound_id);
    }
    const auto& [left_in_bound_id, left_offset] = getLeftOffsetInfoFromPolyline(sample_segments_[i], left_obstacles);
    if (left_in_bound_id != "") {
      const auto& [start_buffer, end_buffer, lat_buffer] = buffer_map.at(left_in_bound_id);
      left_od_bounds.emplace_back(1, curr_s - start_buffer, left_offset, left_in_bound_id);
      left_od_bounds.emplace_back(0, curr_s + end_buffer, left_offset, left_in_bound_id);
    }
  }

  auto sorter = [](const PolylineBoundRange& lhs, const PolylineBoundRange& rhs) {
    if (std::get<1>(lhs) != std::get<1>(rhs)) return std::get<1>(lhs) < std::get<1>(rhs);
    return std::get<0>(lhs) > std::get<0>(rhs);
  };
  std::sort(right_od_bounds.begin(), right_od_bounds.end(), sorter);
  std::sort(left_od_bounds.begin(), left_od_bounds.end(), sorter);

  std::multimap<double, std::string, std::greater<double>> right_bounds;
  right_bounds.emplace(std::numeric_limits<double>::lowest(), "");
  std::multimap<double, std::string> left_bounds;
  left_bounds.emplace(std::numeric_limits<double>::max(), "");
  int right_obs_idx = 0, left_obs_idx = 0;
  for (size_t i = 0; i + 1 < sample_points_.size(); i++) {
    double curr_s = sample_points_[i].local_s();
    while (right_obs_idx < right_od_bounds.size() && std::get<1>(right_od_bounds[right_obs_idx]) < curr_s) {
      auto& [in_bound, obs_s, obs_l, ptr_id] = right_od_bounds[right_obs_idx];
      if (in_bound)
        right_bounds.emplace(obs_l, ptr_id);
      else
        right_bounds.erase(right_bounds.find(obs_l));
      ++right_obs_idx;
    }
    while (left_obs_idx < left_od_bounds.size() && std::get<1>(left_od_bounds[left_obs_idx]) < curr_s) {
      auto& [in_bound, obs_s, obs_l, ptr_id] = left_od_bounds[left_obs_idx];
      if (in_bound)
        left_bounds.emplace(obs_l, ptr_id);
      else
        left_bounds.erase(left_bounds.find(obs_l));
      ++left_obs_idx;
    }
    auto [raw_bound, raw_has_max_bound] = check_ranges_.interpolate(curr_s);
    auto [bound, has_max_bound] = check_ranges_.interpolate(curr_s);
    auto& [bound_s, bound_right, bound_left] = bound;
    if (right_bounds.begin()->first > bound_right) {
      bound_right = right_bounds.begin()->first;
    }
    if (left_bounds.begin()->first < bound_left) {
      bound_left = left_bounds.begin()->first;
    }
    if (bound_right > bound_left) {
      bound_left = std::get<2>(raw_bound);
      bound_right = std::get<1>(raw_bound);
    }
    res.bound.emplace_back(curr_s, bound_right, bound_left);
    res.right_bound_obstacles.push_back(nullptr);
    res.left_bound_obstacles.push_back(nullptr);
  }
  return res;
}

}  // namespace gpal::pnc::planning
