#include "openspace_path_planner/utils/openspace_tools.h"

#include "config_manager/config_manager.h"

namespace gpal::pnc::planning {
/**
 * 轨迹碰撞校验
 * @param width_safe_dis 自车box的膨胀宽度距离,m
 * @param length_safe_dis 自车box的膨胀长度距离,m
 * @param fs_round_corner_width 针对fs的圆角宽度,m
 */
bool trajCollisonCheck(const Freespace& freespace, const vector<DiscretizedPath>& traj, const RoiDecideResult& roi,
                       const double width_safe_dis, const double length_safe_dis, const double fs_round_corner_width) {
  for (auto path : traj) {
    for (auto pt : path) {
      if (!pointCollisonCheck(freespace, pt, roi, width_safe_dis, length_safe_dis, fs_round_corner_width)) {
        return false;
      }
    }
  }
  return true;
}

/**
 * 轨迹点碰撞校验
 * @param width_safe_dis 自车box的膨胀宽度距离,m
 * @param length_safe_dis 自车box的膨胀长度距离,m
 * @param fs_round_corner_width 针对fs的圆角宽度,m
 */
bool pointCollisonCheck(const Freespace& freespace, const PathPt& point, const RoiDecideResult& roi,
                        const double width_safe_dis, const double length_safe_dis, const double fs_round_corner_width) {
  double x = point.x();
  double y = point.y();
  double theta = point.theta();

  auto vehicle_config = Singleton<ConfigManager>::get_instance()->vehicle_config();
  // 车辆box
  double ego_length = vehicle_config.vehicle_param().length() + 2 * length_safe_dis;
  double ego_width = vehicle_config.vehicle_param().width_without_rearview_mirror() + 2 * width_safe_dis;
  double shift_distance = ego_length / 2.0 - (vehicle_config.vehicle_param().rear_edge_to_ego() + length_safe_dis);
  math::Box2d bounding_box(
      {point.x() + shift_distance * std::cos(point.theta()), point.y() + shift_distance * std::sin(point.theta())},
      point.theta(), ego_length, ego_width);

  // 对OD进行校验
  for (const auto& obstacle_linesegments : roi.obstacles_linesegments) {
    for (const math::LineSegment2d& linesegment : std::get<2>(obstacle_linesegments)) {
      if (bounding_box.HasOverlap(linesegment)) {
        return false;
      }
    }
  }

  // fs校验
  math::Vec2d key_vec;
  return !freespace.isOccupied(bounding_box, &key_vec, fs_round_corner_width);
}

/**
 * 轨迹点碰撞校验
 * @param width_safe_dis 自车box的膨胀宽度距离,m
 * @param length_safe_dis 自车box的膨胀长度距离,m
 * @param fs_round_corner_width 针对fs的圆角宽度,m
 */
bool pointCollisonCheck(double& min_dis, const Freespace& freespace, const PathPt& point, const RoiDecideResult& roi,
                        const double width_safe_dis, const double length_safe_dis, const double fs_round_corner_width) {
  min_dis = std::numeric_limits<double>::max();
  double x = point.x();
  double y = point.y();
  double theta = point.theta();

  auto vehicle_config = Singleton<ConfigManager>::get_instance()->vehicle_config();
  // 车辆box
  double ego_length = vehicle_config.vehicle_param().length() + 2 * length_safe_dis;
  double ego_width = vehicle_config.vehicle_param().width_without_rearview_mirror() + 2 * width_safe_dis;
  double shift_distance = ego_length / 2.0 - (vehicle_config.vehicle_param().rear_edge_to_ego() + length_safe_dis);
  math::Box2d bounding_box(
      {point.x() + shift_distance * std::cos(point.theta()), point.y() + shift_distance * std::sin(point.theta())},
      point.theta(), ego_length, ego_width);

  // 对OD进行校验
  for (const auto& obstacle_linesegments : roi.obstacles_linesegments) {
    for (const math::LineSegment2d& linesegment : std::get<2>(obstacle_linesegments)) {
      min_dis = std::min(min_dis, bounding_box.DistanceTo(linesegment));
      if (min_dis < kMathEpsilon) {
        return false;
      }
    }
  }

  // fs校验
  math::Vec2d key_vec;
  min_dis = std::min(min_dis, freespace.getMinFreespaceDisFromBox(bounding_box, &key_vec));
  return min_dis > kMathEpsilon;
}

bool pointCollisonCheck(const Freespace& freespace, const double& x, const double& y, const double& theta,
                        const RoiDecideResult& roi, const double width_safe_dis, const double length_safe_dis,
                        const double fs_round_corner_width) {
  PathPt point(x, y, 0.0);
  point.set_theta(theta);
  return pointCollisonCheck(freespace, point, roi, width_safe_dis, length_safe_dis, fs_round_corner_width);
}

bool pointCollisonCheck(double& min_dis, const Freespace& freespace, const double& x, const double& y,
                        const double& theta, const RoiDecideResult& roi, const double width_safe_dis,
                        const double length_safe_dis, const double fs_round_corner_width) {
  PathPt point(x, y, 0.0);
  point.set_theta(theta);
  return pointCollisonCheck(min_dis, freespace, point, roi, width_safe_dis, length_safe_dis, fs_round_corner_width);
}

/**
 * 按照轨迹direction，对轨迹进行分段
 * @param orin_traj 原始未分段的轨迹
 * @param partition_paths 分段后的轨迹
 */
void pathPartition(const DiscretizedPath& orin_traj, vector<DiscretizedPath>& partition_paths) {
  DiscretizedPath partition_path;
  auto direction = orin_traj.begin()->direction();
  for (auto p : orin_traj) {
    if (direction != p.direction()) {
      partition_paths.push_back(partition_path);
      direction = p.direction();
      partition_path.clear();
      partition_path.push_back(p);
    } else {
      partition_path.push_back(p);
    }
  }
  partition_paths.push_back(partition_path);
}

/**
 * 输出当前flu轨迹，并对当前轨迹进行裁减，计算剩余距离
 */
void currentTrajSelectAndCut(const std::vector<DiscretizedPath>& partition_paths, DiscretizedPath& current_path,
                             double& remain_dis) {
  PathPt veh_orin_pose;
  remain_dis = 0.0;

  if (partition_paths.empty() || partition_paths.front().empty()) {
    return;
  }

  current_path = partition_paths.front();
  ReCalculateLineLength(current_path);

  // 1. 查找粗略最近点
  double min_Distance = 1e6;
  int n_nearest_index = -1;
  if (!findNearestPointInTraj(current_path, veh_orin_pose, n_nearest_index, min_Distance)) {
    OPENSPACE_LOG(E, "findNearestPointInTraj error");
    return;
  }
  // 2. 分情况处理插值
  DiscretizedPath dense_segment;
  const double interpolation_step = 0.01;

  if (n_nearest_index > 0 && n_nearest_index + 1 < current_path.size()) {
    // 情况1：能取到三个点（前一点、最近点、后一点）
    const auto& prev_pt = current_path[n_nearest_index - 1];
    const auto& nearest_pt = current_path[n_nearest_index];
    const auto& next_pt = current_path[n_nearest_index + 1];
    // 三段式插值（前段、当前段、后段）
    double seg1_length = prev_pt.DistanceTo(nearest_pt);
    double seg2_length = nearest_pt.DistanceTo(next_pt);
    double total_length = seg1_length + seg2_length;
    int num_points = std::max(1, static_cast<int>(total_length / interpolation_step));

    for (int i = 0; i <= num_points; ++i) {
      double s = static_cast<double>(i) / num_points * total_length;
      PathPt interp_pt;
      if (s <= seg1_length) {
        interp_pt = interpolateUsingLinearApproximation(prev_pt, nearest_pt, prev_pt.s() + s);
      } else {
        interp_pt = interpolateUsingLinearApproximation(nearest_pt, next_pt, nearest_pt.s() + (s - seg1_length));
      }
      dense_segment.push_back(interp_pt);
    }
  } else {
    // 情况2：只能取到两个点（起点或终点附近）
    int start_idx = std::max(0, n_nearest_index - 1);
    int end_idx = std::min(static_cast<int>(current_path.size() - 1), n_nearest_index + 1);

    if (start_idx < end_idx) {  // 确保有两个不同点
      const auto& pt1 = current_path[start_idx];
      const auto& pt2 = current_path[end_idx];
      double seg_length = pt1.DistanceTo(pt2);
      int num_points = std::max(1, static_cast<int>(seg_length / interpolation_step));

      for (int i = 0; i <= num_points; ++i) {
        double s = static_cast<double>(i) / num_points * seg_length;
        dense_segment.push_back(interpolateUsingLinearApproximation(pt1, pt2, pt1.s() + s));
      }
    } else {
      // 极端情况：只有一个点，无法插值
      dense_segment.push_back(current_path[n_nearest_index]);
    }
  }

  // 3. 在密集插值段中查找精确最近点
  int dense_nearest_index = -1;
  double dense_min_distance = 1e6;
  if (!findNearestPointInTraj(dense_segment, veh_orin_pose, dense_nearest_index, dense_min_distance)) {
    OPENSPACE_LOG(E, "Failed to find nearest point in dense segment");
    return;
  }

  // 4. 轨迹裁剪与拼接
  current_path.erase(current_path.begin(),
                     current_path.begin() + std::min(static_cast<int>(current_path.size() - 1), n_nearest_index + 1));
  current_path.insert(current_path.begin(), dense_segment.at(dense_nearest_index));  // 插入密集插值后的精确轨迹点

  // 5. 计算剩余距离
  if (!current_path.empty()) {
    PathPt nearest_point = current_path.front();
    // 倒车时，翻转theta指向轨迹朝向
    double extend_theta = nearest_point.direction() == PathPt::Direction::BACKWARD
                              ? headingReversal(nearest_point.theta())
                              : nearest_point.theta();
    double extend_dis = 1.0;
    PathPt previous_point =
        PathPt(std::cos(extend_theta) * extend_dis + nearest_point.x(),
               std::sin(extend_theta) * extend_dis + nearest_point.y(), nearest_point.z(), 0.0, nearest_point.theta());
    float factor = 0;
    PathPt pedal_point = CalculatePedalPoint(veh_orin_pose, nearest_point, previous_point, factor);
    double compensate_dis = pedal_point.DistanceTo(nearest_point);
    if (factor > 0) {
      compensate_dis = -compensate_dis;
    }
    for (int i = 0; i + 1 < current_path.size(); i++) {
      remain_dis += current_path.at(i).DistanceTo(current_path.at(i + 1));
    }
    remain_dis += compensate_dis;
  }
}

/**
 * 将点的航向反转PI
 * @param orin_heading FLU, (-PI, PI]
 */
double headingReversal(const double& orin_heading) {
  return gpal::pnc::planning::math::NormalizeAngle(orin_heading + M_PI);
}

/**
 * 将轨迹上点的航向反转PI
 * @param traj FLU, (-PI, PI]
 * default: FLU, (-PI, PI]
 * mode = 0: FLU, (-PI, PI]
 * mode = 1: FLU, [0, 2*PI)
 */
void headingReversal(DiscretizedPath& traj) {
  for (auto& point : traj) {
    point.set_theta(headingReversal(point.theta()));
  }
}

/**
 * 计算轨迹上距离某一点最近的轨迹点
 * @param traj 目标轨迹
 * @param ego_pose 轨迹外某一点
 * @param min_dis_index 最近轨迹点序号
 * @param min_dis 最近轨迹点距离
 */
bool findNearestPointInTraj(const DiscretizedPath& traj, const PathPt& ego_pose, int& min_dis_index, double& min_dis) {
  min_dis = 1e6;
  min_dis_index = -1;
  for (int i = 0; i < traj.size(); i++) {
    auto dis = traj.at(i).DistanceTo(ego_pose);
    double delta_theta = std::abs(math::NormalizeAngle(ego_pose.theta() - traj.at(i).theta()));  // 需方向同向
    if (dis < min_dis && delta_theta < M_PI_2) {
      min_dis_index = i;
      min_dis = dis;
    }
  }
  return min_dis_index != -1;
}

/**
 * 计算从一点开始向前的安全延长直线
 * @param freespace  freespace
 * @param roi  roi
 * @param start  起点
 * @param max_length  最大延长长度
 * @param width_safe_buff  宽度安全buf
 * @param length_safe_buff  长度安全buf
 * @param step  直线轨迹步长
 * @return  安全延长直线
 */
std::vector<PathPt> safe_straight_line_generator(const Freespace& freespace, const RoiDecideResult& roi,
                                                 const PathPt& start, const double& max_length,
                                                 const double width_safe_buff, const double length_safe_buff,
                                                 const double step) {
  double cos_theta = cos(start.theta());
  double sin_theta = sin(start.theta());
  // 计算安全直线伸距离
  double safe_extend_dis = 0.0;
  double safe_extend_step = start.direction() == PathPt::Direction::BACKWARD ? -0.01 : 0.01;
  int index = static_cast<int>(std::round(max_length / abs(safe_extend_step)));
  for (int i = 1; i <= index; ++i) {
    PathPt extend_point(start.x() + i * safe_extend_step * cos_theta, start.y() + i * safe_extend_step * sin_theta,
                        start.z());
    extend_point.set_theta(start.theta());
    if (!pointCollisonCheck(freespace, extend_point, roi, width_safe_buff, length_safe_buff)) {
      break;
    }
    safe_extend_dis = i * abs(safe_extend_step);
  }
  OPENSPACE_LOG(D, "safe_extend_dis: ", safe_extend_dis);

  // 计算直线轨迹
  std::vector<PathPt> straight_line_path;
  index = static_cast<int>(std::round(safe_extend_dis / abs(step)));
  if (index == 0)
    return straight_line_path;
  auto step_modify =
      start.direction() == PathPt::Direction::BACKWARD ? (-safe_extend_dis / index) : (safe_extend_dis / index);
  for (int i = 1; i <= index; ++i) {
    PathPt extend_point(start.x() + i * step_modify * cos_theta, start.y() + i * step_modify * sin_theta, start.z());
    extend_point.set_theta(start.theta());
    extend_point.set_direction(start.direction());
    extend_point.set_s(start.s() + i * abs(step_modify));
    straight_line_path.emplace_back(extend_point);
  }
  OPENSPACE_LOG(D, "straight_line_path size: ", straight_line_path.size());
  return straight_line_path;
};

/**
 * 计算从一点开始，沿着heading，向direction方向延长的直线,不考虑碰撞
 * @param start  起点
 * @param max_length  最大延长长度
 * @param step  直线轨迹步长
 * @return  延长直线
 */
std::vector<PathPt> straight_line_generator(const PathPt& start, const double& max_length, const double step) {
  std::vector<PathPt> straight_line_path;
  if (std::abs(step) < 1e-6)
    return straight_line_path;

  double cos_theta = cos(start.theta());
  double sin_theta = sin(start.theta());
  int index = static_cast<int>(std::round(max_length / abs(step)));
  if (index == 0)
    return straight_line_path;
  auto step_modify = start.direction() == PathPt::Direction::BACKWARD ? (-max_length / index) : (max_length / index);
  for (int i = 1; i <= index; ++i) {
    PathPt extend_point(start.x() + i * step_modify * cos_theta, start.y() + i * step_modify * sin_theta, start.z());
    extend_point.set_theta(start.theta());
    extend_point.set_s(start.s() + i * abs(step_modify));
    extend_point.set_direction(start.direction());
    straight_line_path.emplace_back(extend_point);
  }
  return straight_line_path;
};

/**
 * 反转直线轨迹，包括方向
 * @param straight_line_path  直线轨迹
 */
void reverse_straight_line(std::vector<PathPt>& straight_line_path) {
  if (straight_line_path.empty())
    return;

  std::reverse(straight_line_path.begin(), straight_line_path.end());
  auto reverse_direc = straight_line_path.back().direction() == PathPt::Direction::BACKWARD
                           ? PathPt::Direction::FORWARD
                           : PathPt::Direction::BACKWARD;
  std::for_each(straight_line_path.begin(), straight_line_path.end(),
                [reverse_direc](PathPt& pt) { pt.set_direction(reverse_direc); });
};

/**
 * 计算库位前、后方空旷空间
 * @param local_view
 * @param roi
 * @param orin_slot  原始库位
 * @return  修正后空旷库位
 */
Slot calFreeSpaceSlot(const LocalView& local_view, const RoiDecideResult& roi, const Slot& orin_slot) {
  auto has_collision = [&](const math::Box2d& box) {
    for (const auto& obstacle_linesegments : roi.obstacles_linesegments) {
      for (const math::LineSegment2d& linesegment : std::get<2>(obstacle_linesegments)) {
        if (box.HasOverlap(linesegment)) {
          return true;
        }
      }
    }
    math::Vec2d key_vec;
    return local_view.getFreespacePtr()->isOccupied(box, &key_vec);
  };

  std::vector<PathPt> free_slot_corners(4);
  double step = 0.1;  // 膨胀步长
  double current_expansion = 0.0;
  bool collision_detected = false;

  // 判断库位前方空间
  math::Box2d expanded_box(orin_slot.topEdgeCenter(), orin_slot.heading(), 0.01, orin_slot.width());
  while (current_expansion < 5.0 && !collision_detected) {
    OPENSPACE_LOG(D, "orin_slot.topEdgeCenter(): ", orin_slot.topEdgeCenter().x(), " ", orin_slot.topEdgeCenter().y());
    for (auto p : expanded_box.GetAllCorners()) {
      OPENSPACE_LOG(D, "expanded_box: ", p.x(), " ", p.y());
    }
    math::Box2d expanding_box = expanded_box;
    math::Vec2d shift_vec(step * 0.5 * std::cos(expanded_box.heading()), step * 0.5 * std::sin(expanded_box.heading()));
    expanding_box.Shift(shift_vec);
    expanding_box.LongitudinalExtend(step);

    if (has_collision(expanding_box)) {
      collision_detected = true;
    } else {
      expanded_box = expanding_box;
      current_expansion += step;
    }
  }
  OPENSPACE_LOG(D, "current_expansion front: ", current_expansion);
  free_slot_corners.at(0) =
      PathPt(orin_slot.corner(0).x() + current_expansion * std::cos(orin_slot.heading()),
             orin_slot.corner(0).y() + current_expansion * std::sin(orin_slot.heading()), orin_slot.corner(0).z());
  free_slot_corners.at(3) =
      PathPt(orin_slot.corner(3).x() + current_expansion * std::cos(orin_slot.heading()),
             orin_slot.corner(3).y() + current_expansion * std::sin(orin_slot.heading()), orin_slot.corner(3).z());

  // 判断库位后方空间
  expanded_box = math::Box2d(orin_slot.bottomEdgeCenter(), orin_slot.heading(), 0.01, orin_slot.width());
  current_expansion = 0.0;
  collision_detected = false;
  while (current_expansion < 5.0 && !collision_detected) {
    math::Box2d expanding_box = expanded_box;
    math::Vec2d shift_vec(-step * 0.5 * std::cos(expanded_box.heading()),
                          -step * 0.5 * std::sin(expanded_box.heading()));
    expanding_box.Shift(shift_vec);
    expanding_box.LongitudinalExtend(step);

    if (has_collision(expanding_box)) {
      collision_detected = true;
    } else {
      expanded_box = expanding_box;
      current_expansion += step;
    }
  }
  OPENSPACE_LOG(D, "current_expansion back: ", current_expansion);
  free_slot_corners.at(1) =
      PathPt(orin_slot.corner(1).x() - current_expansion * std::cos(orin_slot.heading()),
             orin_slot.corner(1).y() - current_expansion * std::sin(orin_slot.heading()), orin_slot.corner(1).z());
  free_slot_corners.at(2) =
      PathPt(orin_slot.corner(2).x() - current_expansion * std::cos(orin_slot.heading()),
             orin_slot.corner(2).y() - current_expansion * std::sin(orin_slot.heading()), orin_slot.corner(2).z());
  Slot free_space_slot(free_slot_corners, Slot::SlotType::PARALLEL, orin_slot.direction());
  return free_space_slot;
}

/**
 * @brief 计算box在某个方向上的安全距离
 * @param local_view  含fs信息
 * @param obstacle_ods  障碍物box信息
 * @param box  输入box
 * @param direction  方向
 * @param max_dist  最大距离
 * @param step  步长
 * @return double  安全距离
 */
double calcBoxSafeDist(const LocalView& local_view, const std::vector<Decision::DecisionObject>& obstacle_ods,
                       const std::vector<math::LineSegment2d>& line_segs, const math::Box2d& box,
                       const math::Vec2d& direction, const double max_dist, const double step) {
  if (max_dist < 1e-6 || step < 1e-6) {
    return 0.0;
  }

  // 步长倍增快速定位碰撞区间
  double current_step = 10.0 * step;
  double safe_dist = 0.0;
  auto test_box = box;
  // 快速前进直到碰撞
  while (safe_dist + current_step <= max_dist) {
    test_box = box;  // 重置box
    test_box.Shift(math::Vec2d((safe_dist + current_step) * direction.x(), (safe_dist + current_step) * direction.y()));
    if (boxIsCollided(local_view, obstacle_ods, test_box) || boxIsCollided(line_segs, test_box)) {
      break;
    }
    safe_dist += current_step;
  }

  // 如果已经达到最大距离且安全
  if (safe_dist >= max_dist - 1e-6) {
    return max_dist;
  }

  // 在最后的安全区间内进行二分查找
  double left = safe_dist;
  double right = std::min(safe_dist + current_step, max_dist);

  while (right - left > step) {
    double mid = (left + right) / 2.0;
    test_box = box;
    test_box.Shift(math::Vec2d(mid * direction.x(), mid * direction.y()));
    if (boxIsCollided(local_view, obstacle_ods, test_box) || boxIsCollided(line_segs, test_box)) {
      right = mid;
    } else {
      left = mid;
      safe_dist = mid;
    }
  }
  return safe_dist;
}

/**
 * @brief 计算box在某个方向上的安全距离
 * @param local_view  含fs信息
 * @param roi  roi结果，含障碍物LineSegment信息
 * @param box  输入box
 * @param direction  方向
 * @param max_dist  最大距离
 * @param step  步长
 * @return double  安全距离
 */
double calcBoxSafeDist(const LocalView& local_view, const RoiDecideResult& roi, const math::Box2d& box,
                       const math::Vec2d& direction, const double max_dist, const double step) {
  if (max_dist < 1e-6 || step < 1e-6) {
    return 0.0;
  }

  // 步长倍增快速定位碰撞区间
  double current_step = 10.0 * step;
  double safe_dist = 0.0;
  auto test_box = box;

  // 快速前进直到碰撞
  while (safe_dist + current_step <= max_dist) {
    test_box = box;  // 重置box
    test_box.Shift(math::Vec2d((safe_dist + current_step) * direction.x(), (safe_dist + current_step) * direction.y()));

    if (boxIsCollided(local_view, roi, test_box)) {
      break;
    }
    safe_dist += current_step;
  }

  // 如果已经达到最大距离且安全
  if (safe_dist >= max_dist - 1e-6) {
    return max_dist;
  }

  // 在最后的安全区间内进行二分查找
  double left = safe_dist;
  double right = std::min(safe_dist + current_step, max_dist);

  while (right - left > step) {
    double mid = (left + right) / 2.0;
    test_box = box;
    test_box.Shift(math::Vec2d(mid * direction.x(), mid * direction.y()));

    if (boxIsCollided(local_view, roi, test_box)) {
      right = mid;
    } else {
      left = mid;
      safe_dist = mid;
    }
  }
  return safe_dist;
}

/**
 * @brief 计算box是否有碰撞
 * @param local_view  含fs信息
 * @param roi  roi结果，含障碍物LineSegment信息
 * @param box  输入box
 * @return true  有碰撞
 * @return false  无碰撞
 */
bool boxIsCollided(const LocalView& local_view, const RoiDecideResult& roi, const math::Box2d& box) {
  if (boxIsCollided(roi, box) || boxIsCollided(local_view, box)) {
    return true;
  }
  return false;
}

/**
 * @brief 计算box是否有碰撞
 * @param local_view  含fs信息
 * @param line_segs  障碍物LineSegment信息
 * @param box  输入box
 * @return true  有碰撞
 * @return false  无碰撞
 */
bool boxIsCollided(const LocalView& local_view, const std::vector<math::LineSegment2d>& line_segs,
                   const math::Box2d& box) {
  if (boxIsCollided(line_segs, box) || boxIsCollided(local_view, box)) {
    return true;
  }
  return false;
}

/**
 * @brief 计算box是否有碰撞
 * @param local_view  含fs信息
 * @param obstacle_ods  障碍物box信息
 * @param box  输入box
 * @return true  有碰撞
 * @return false  无碰撞
 */
bool boxIsCollided(const LocalView& local_view, const std::vector<Decision::DecisionObject>& obstacle_ods,
                   const math::Box2d& box) {
  if (boxIsCollided(obstacle_ods, box) || boxIsCollided(local_view, box)) {
    return true;
  }
  return false;
}

bool boxIsCollided(const LocalView& local_view, const math::Box2d& box) {
  math::Vec2d key_vec;
  if (local_view.getFreespacePtr()->isOccupied(box, &key_vec)) {
    return true;
  }
  return false;
}

bool boxIsCollided(const std::vector<Decision::DecisionObject>& obstacle_ods, const math::Box2d& box) {
  for (const auto& od : obstacle_ods) {
    if (box.HasOverlap(od.cur_box)) {
      return true;
    }
  }
  return false;
}

bool boxIsCollided(const std::vector<math::LineSegment2d>& line_segs, const math::Box2d& box) {
  for (const auto& linesegment : line_segs) {
    if (box.HasOverlap(linesegment)) {
      return true;
    }
  }
  return false;
}

bool boxIsCollided(const RoiDecideResult& roi, const math::Box2d& box) {
  for (const auto& obstacle_linesegments : roi.obstacles_linesegments) {
    for (const math::LineSegment2d& linesegment : std::get<2>(obstacle_linesegments)) {
      if (box.HasOverlap(linesegment)) {
        return true;
      }
    }
  }
  return false;
}

/**
 * @brief 轨迹点的时间补偿
 * @param tf_ego_2_map_orin 原始帧ego到map的变换矩阵
 * @param tf_map_2_ego_target  map到目标帧ego的变换矩阵
 * @param path_pt  输入轨迹点
 */
void trajectoryPointTimeCompensate(const Eigen::Matrix4d tf_ego_2_map_orin, const Eigen::Matrix4d tf_map_2_ego_target,
                                   PathPt& path_pt) {
  // 将轨迹点从原始帧ego转到map
  transfer::transformPoint(tf_ego_2_map_orin, &path_pt);
  math::Vec3d path_pt_3d_rpy(0.0, 0.0, path_pt.theta());
  transfer::transformRPY(tf_ego_2_map_orin, &path_pt_3d_rpy);
  path_pt.set_theta(path_pt_3d_rpy.z());

  // 将轨迹点从map转到目标帧ego
  transfer::transformPoint(tf_map_2_ego_target, &path_pt);
  math::Vec3d path_pt_3d_rpy_target(0.0, 0.0, path_pt.theta());
  transfer::transformRPY(tf_map_2_ego_target, &path_pt_3d_rpy_target);
  path_pt.set_theta(path_pt_3d_rpy_target.z());
}
/**
 * @brief 轨迹的时间补偿
 * @param tf_ego_2_map_orin 原始帧ego到map的变换矩阵
 * @param tf_map_2_ego_target  map到目标帧ego的变换矩阵
 * @param path  输入轨迹
 */
void trajectoryTimeCompensate(const Eigen::Matrix4d tf_ego_2_map_orin, const Eigen::Matrix4d tf_map_2_ego_target,
                              std::vector<PathPt>& path) {
  for (auto& path_pt : path) {
    trajectoryPointTimeCompensate(tf_ego_2_map_orin, tf_map_2_ego_target, path_pt);
  }
}

void appendLineSegments(const std::vector<math::Vec2d>& points, std::vector<math::LineSegment2d>& segments,
                        double min_length, double max_heading_err) {
  if (points.empty()) {
    return;
  }
  for (auto iter = points.begin(); iter != points.end(); iter++) {
    if (segments.empty()) {
      if (iter != points.begin()) {
        segments.emplace_back(*(iter - 1), *iter);
      }
      continue;
    }
    auto pos_err = (*iter) - segments.back().end();
    if (pos_err.Length() < min_length) {
      continue;
    }
    const double heading_err = math::NormalizeAngle(pos_err.Angle() - segments.back().heading());
    if (std::abs(heading_err) < max_heading_err) {
      segments.back() = math::LineSegment2d(segments.back().start(), *iter);
      continue;
    }
    segments.emplace_back(segments.back().end(), *iter);
  }
}

std::tuple<bool, double, double, int, int> getOverlapRange(const math::Polygon2d& polygon,
                                                           const std::vector<math::LineSegment2d>& segments,
                                                           const std::vector<double>& accumulated_s) {
  double length = 0.0;
  if (!accumulated_s.empty()) {
    length = accumulated_s.back();
  }
  std::tuple<bool, double, double, int, int> res(false, length, 0.0, segments.size(), 0);
  auto& [in_range, start_s, end_s, start_index, end_index] = res;
  bool has_range = false;
  if (!segments.empty()) {
    for (int i = 0; i < segments.size(); i++) {
      if (polygon.HasOverlap(segments[i])) {
        has_range = true;
        math::Vec2d first, second;
        polygon.GetOverlap(segments[i], &first, &second);
        double overlap_start_s = accumulated_s[i] + segments[i].ProjectOntoUnit(first);
        double overlap_end_s = accumulated_s[i] + segments[i].ProjectOntoUnit(second);
        start_s = std::min(start_s, overlap_start_s);
        end_s = std::max(end_s, overlap_end_s);
        start_index = std::min(start_index, i);
        end_index = std::max(end_index, i);
      } else if (has_range) {
        break;
      }
    }
    in_range = start_s <= end_s && start_index <= end_index && end_index < segments.size();
  }
  return res;
}

}  // namespace gpal::pnc::planning
