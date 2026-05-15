/**
 * @file speed_common.cpp
 * @brief 速度规划的公共函数实现
 * @details 包含速度规划基础函数
 */
#include "speed_common/speed_common.h"
#include <algorithm>


namespace gpal::pnc::planning {

DiscretizedPath intervalPath(const DiscretizedPath& path, const double& step) {
  std::vector<PathPt> interval_path_points;
  if (path.size() > 1) {
    const double length = path.back().s() - path.front().s();
    if (std::fabs(length) < kMathEpsilon) {
      interval_path_points = std::vector<PathPt>{path.front()};
      return DiscretizedPath(std::move(interval_path_points));
    }
    double interval_ds = 0.0;
    if (length > -kMathEpsilon) {
      interval_ds = std::fmax(step, 0.1);
    } else {
      interval_ds = std::fmin(length / static_cast<double>(path.size() - 1), -0.1);
    }
    int num = length / interval_ds;
    if(num < 1){
      interval_path_points = std::vector<PathPt>{path.front(), path.back()};
      return DiscretizedPath(std::move(interval_path_points));
    }
    size_t last_index = 0UL;
    for (int index = 0; index <= num; ++index) {
      double s = path.front().s() + index * interval_ds;
      auto index_pt = evaluatePath(path, last_index, s);
      interval_path_points.emplace_back(index_pt.second);
      last_index = index_pt.first;
    }
  } else {
    interval_path_points = path.empty() ? std::vector<PathPt>() : std::vector<PathPt>{path.front()};
  }
  return DiscretizedPath(std::move(interval_path_points));
}

std::pair<size_t, PathPt> evaluatePath(const DiscretizedPath& path, size_t start_index,
                                                         const double s) {
  if (path.empty()) {
    return std::pair<size_t, PathPt>(0, PathPt());
  }

  if (path.size() == 1) {
    return std::pair<size_t, PathPt>(0, path.front());
  }

  if (std::fabs(path.back().s() - path.front().s()) < kMathEpsilon) {
    return std::pair<size_t, PathPt>(0, path.front());
  }

  bool forward = path.back().s() - path.front().s() > kMathEpsilon;

  if (forward) {
    if (s < path.front().s()) {
      return std::pair<size_t, PathPt>(0, path.front());
    }
    if (s > path.back().s()) {
      return std::pair<size_t, PathPt>(path.size() - 1UL, path.back());
    }
  } else {
    if (s > path.front().s()) {
      return std::pair<size_t, PathPt>(0, path.front());
    }
    if (s < path.back().s()) {
      return std::pair<size_t, PathPt>(path.size() - 1UL, path.back());
    }
  }

  for (size_t index = std::max(start_index, 0UL); index < path.size() - 1UL; ++index) {
    if ((path[index].s() <= s && s <= path[index + 1UL].s()) || (path[index + 1UL].s() <= s && s <= path[index].s())) {
      return std::pair<size_t, PathPt>(index,
                                       math::interpolateUsingLinearApproximation(path[index], path[index + 1UL], s));
    }
  }

  // if not find
  if (start_index > 0UL) {
    return std::pair<size_t, PathPt>(
        start_index - 1UL, math::interpolateUsingLinearApproximation(path[start_index - 1UL], path[start_index], s));
  }

  return std::pair<size_t, PathPt>(0, path.front());
}


// std::pair<double, double> CalcSLCoordinatesToInfinitPath(
//     const PathGroup& path_group, double init_s, const math::Vec2d& pt, int& collision_segment_index) {
//   // First calculate the distance of center point of obstacle box to our path segment:
//   double pt_x = pt.x();
//   double pt_y = pt.y();
//   std::vector<std::tuple<double, double, int>> init_s_dis_vec;  // s, l, index
//   std::vector<PathPt> final_use_path_points_for_segmentation = path_group.path_points_for_segmentation_after_extand_;
//   int start_segment_index = 0;

//   for (int i = 0; i + 1 < final_use_path_points_for_segmentation.size(); i++) {
//     auto path_pt_0 = final_use_path_points_for_segmentation[i];
//     auto path_pt_1 = final_use_path_points_for_segmentation[i + 1];
//     math::LineSegment2d path_segment(path_pt_0, path_pt_1);
//     math::Vec2d start_to_pt(pt - path_pt_0);
//     double proj =
//         start_to_pt.x() * path_segment.unit_direction().x() + start_to_pt.y() * path_segment.unit_direction().y();
//     if (proj >= 0.0 && proj <= path_segment.length()) {
//       math::Vec2d nearest_pt;
//       double current_l = path_segment.DistanceTo(pt, &nearest_pt);
//       double current_s = nearest_pt.DistanceTo(path_pt_0) + path_pt_0.s();
//       init_s_dis_vec.emplace_back(std::make_tuple(current_s, current_l, start_segment_index + i));
//     }
//   }

//   if (init_s_dis_vec.size() == 1) {
//     init_s = std::get<0>(init_s_dis_vec.front());
//     collision_segment_index = std::get<2>(init_s_dis_vec.front());
//   } else if (init_s_dis_vec.size() > 1) {
//     std::sort(init_s_dis_vec.begin(), init_s_dis_vec.end(),
//               [](const std::tuple<double, double, int>& a, const std::tuple<double, double, int>& b) {
//                 return fabs(std::get<1>(a)) < fabs(std::get<1>(b));
//               });
//     init_s = std::get<0>(init_s_dis_vec.front());
//     collision_segment_index = std::get<2>(init_s_dis_vec.front());
//   } else {
//     double s_to_min_l = kPostiveInfinity, min_l = kPostiveInfinity;
//     int index = 0;
//     for (int i = 0; i + 1 < final_use_path_points_for_segmentation.size(); i++) {
//       auto path_pt_0 = final_use_path_points_for_segmentation[i];
//       auto path_pt_1 = final_use_path_points_for_segmentation[i + 1];
//       math::LineSegment2d path_segment(path_pt_0, path_pt_1);
//       math::Vec2d nearest_pt;
//       double current_l = path_segment.DistanceTo(pt, &nearest_pt);
//       if (current_l > min_l) {
//         continue;
//       }
//       min_l = current_l;
//       s_to_min_l = nearest_pt.DistanceTo(path_pt_0) + path_pt_0.s();
//       index = start_segment_index + i;
//     }
//     init_s = s_to_min_l;
//     collision_segment_index = index;
//   }

//   final_use_path_points_for_segmentation.clear();
//   final_use_path_points_for_segmentation.assign(
//         path_group.path_points_for_segmentation_after_extand_.begin() + max(0, collision_segment_index - 1),
//         path_group.path_points_for_segmentation_after_extand_.begin() +
//             min(static_cast<int>(path_group.path_points_for_segmentation_after_extand_.size()), collision_segment_index + 2));
  
//   double curr_s = init_s;
//   vector<DiscretizedPath> using_discretized_path;
//   for (size_t i = 0; i + 1 < final_use_path_points_for_segmentation.size(); ++i) {
//     DiscretizedPath path_segment;
//     path_group.extand_interval_path_.getPathPts(final_use_path_points_for_segmentation.at(i).s(),
//                                        final_use_path_points_for_segmentation.at(i + 1).s(), &path_segment);
//     if (path_segment.empty()) {
//       continue;
//     }
//     using_discretized_path.emplace_back(path_segment);
//   }
//   PathPt curr_pt =  path_group.extand_interval_path_.evaluate(curr_s);
//   int max_search_time = 20;
//   double ds_tolerance = 0.1;
//   double lateral_dist = 10000.0;

//   for (const auto& path_segment : using_discretized_path) {
//     double project_s = std::clamp(init_s, path_segment.front().s(), path_segment.back().s());
//     double project_lateral_dist = 10000.0;
//     PathPt project_pt = path_segment.evaluate(project_s);
//     for (size_t i = 0UL; i < max_search_time; i++) {
//       double curr_theta = project_pt.theta();
//       double curr_kappa = project_pt.kappa();
//       double curr_path_obs_diff_x = project_pt.x() - pt_x;
//       double curr_path_obs_diff_y = project_pt.y() - pt_y;
//       project_lateral_dist = fmin(project_lateral_dist, sqrt(curr_path_obs_diff_x * curr_path_obs_diff_x +
//                                                              curr_path_obs_diff_y * curr_path_obs_diff_y));
//       double dF = std::cos(curr_theta) * curr_path_obs_diff_x + std::sin(curr_theta) * curr_path_obs_diff_y;
//       double ddF =
//           curr_kappa * (-std::sin(curr_theta) * curr_path_obs_diff_x + std::cos(curr_theta) * curr_path_obs_diff_y) +
//           1.0;
//       ddF = (std::fabs(ddF) < 1e-5) ? 1.0 : ddF;
//       double ds = -(dF / ddF);
//       if (abs(ds) < ds_tolerance) {
//         break;
//       } else {
//         project_s += ds;
//         bool early_break = false;
//         if (project_s < path_segment.front().s() || project_s > path_segment.back().s()) {
//           project_s = std::clamp(project_s, path_segment.front().s(), path_segment.back().s());
//           early_break = true;
//         }
//         project_pt = path_segment.evaluate(project_s);
//         if (early_break) {
//           break;
//         }
//       }
//     }
//     if (fabs(project_lateral_dist) < fabs(lateral_dist)) {
//       curr_pt = project_pt;
//       lateral_dist = project_lateral_dist;
//     }
//   }
//   double dx = pt_x - curr_pt.x();
//   double dy = pt_y - curr_pt.y();
//   lateral_dist = fmin(lateral_dist, sqrt(dx * dx + dy * dy));
//   // Check its signature:
//   double curr_theta = curr_pt.theta();
//   double sigd = -dx * std::sin(curr_theta) + dy * std::cos(curr_theta);
//   sigd = (sigd > kMathEpsilon) ? 1.0 : -1.0;
//   return std::make_pair(curr_pt.s(), sigd * lateral_dist);
// }

bool isObstacleStatic(const Decision::ObjectType& obstacle_type,  
                      const std::vector<proto::TrajectoryPoint>& obs_pred_trajectory, const bool& is_static) {
  if (obstacle_type == Decision::ObjectType::VRU || obstacle_type == Decision::ObjectType::PEDESTRIAN) {
    return false;  // VRU一律按动态处理
  }
  // if (obstacle.type() == proto::PerceptionObstacle:: || obstacle.type() == proto::PerceptionObstacle::) {
  //   return true;
  // }
  if (obs_pred_trajectory.size() <= 1) {
    return true;
  } else {
    int trajectory_size = obs_pred_trajectory.size();
    auto middle_point =
        obs_pred_trajectory.at(static_cast<int>(trajectory_size / 2)).path_point();
    auto end_point = obs_pred_trajectory.back().path_point();
    return middle_point.s() < 0.01 && end_point.s() < 0.01 && is_static;
  }
}
}