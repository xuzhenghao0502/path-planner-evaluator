#pragma once
#include "basic_algorithm_lib/basic_algorithm_lib.h"
#include "basic_algorithm_lib/geometry_calculation.h"
#include "config/openspace/openspace_search.pb.h"
#include "config_manager/config_manager.h"
#include "local_view/local_view.h"
#include "openspace_path_planner/core/generator/openspace_roi_decider.h"
#include "openspace_path_planner/utils/openspace_common.h"
#include "openspace_path_planner/utils/slot.h"
#include "path/discretized_path.h"

namespace gpal::pnc::planning {

bool trajCollisonCheck(const Freespace& freespace, const vector<DiscretizedPath>& traj, const RoiDecideResult& roi,
                       const double width_safe_dis = 0.0, const double length_safe_dis = 0.0,
                       const double fs_round_corner_width = 0.0);
bool pointCollisonCheck(const Freespace& freespace, const PathPt& point, const RoiDecideResult& roi,
                        const double width_safe_dis = 0.0, const double length_safe_dis = 0.0,
                        const double fs_round_corner_width = 0.0);
bool pointCollisonCheck(double& min_dis, const Freespace& freespace, const PathPt& point, const RoiDecideResult& roi,
                        const double width_safe_dis = 0.0, const double length_safe_dis = 0.0,
                        const double fs_round_corner_width = 0.0);
bool pointCollisonCheck(const Freespace& freespace, const double& x, const double& y, const double& theta,
                        const RoiDecideResult& roi, const double width_safe_dis = 0.0,
                        const double length_safe_dis = 0.0, const double fs_round_corner_width = 0.0);
bool pointCollisonCheck(double& min_dis, const Freespace& freespace, const double& x, const double& y,
                        const double& theta, const RoiDecideResult& roi, const double width_safe_dis = 0.0,
                        const double length_safe_dis = 0.0, const double fs_round_corner_width = 0.0);

void pathPartition(const DiscretizedPath& orin_traj, vector<DiscretizedPath>& partition_paths);

void currentTrajSelectAndCut(const std::vector<DiscretizedPath>& partition_paths, DiscretizedPath& current_path,
                             double& remain_dis);

double headingReversal(const double& orin_heading);
void headingReversal(DiscretizedPath& traj);

bool findNearestPointInTraj(const DiscretizedPath& traj, const PathPt& ego_pose, int& min_dis_index, double& min_dis);

std::vector<PathPt> safe_straight_line_generator(const Freespace& freespace, const RoiDecideResult& roi,
                                                 const PathPt& start, const double& max_length,
                                                 const double width_safe_buff = 0.1,
                                                 const double length_safe_buff = 0.1, const double step = 0.1);
std::vector<PathPt> straight_line_generator(const PathPt& start, const double& max_length, const double step = 0.1);
void reverse_straight_line(std::vector<PathPt>& straight_line_path);

Slot calFreeSpaceSlot(const LocalView& local_view, const RoiDecideResult& roi, const Slot& orin_slot);

double calcBoxSafeDist(const LocalView& local_view, const std::vector<Decision::DecisionObject>& obstacle_ods,
                       const std::vector<math::LineSegment2d>& line_segs, const math::Box2d& box,
                       const math::Vec2d& direction, const double max_dist, const double step = 0.01);
double calcBoxSafeDist(const LocalView& local_view, const RoiDecideResult& roi, const math::Box2d& box,
                       const math::Vec2d& direction, const double max_dist, const double step = 0.01);
bool boxIsCollided(const LocalView& local_view, const RoiDecideResult& roi, const math::Box2d& box);
bool boxIsCollided(const LocalView& local_view, const std::vector<math::LineSegment2d>& line_segs,
                   const math::Box2d& box);
bool boxIsCollided(const LocalView& local_view, const std::vector<Decision::DecisionObject>& obstacle_ods,
                   const math::Box2d& box);
bool boxIsCollided(const LocalView& local_view, const math::Box2d& box);
bool boxIsCollided(const std::vector<Decision::DecisionObject>& obstacle_ods, const math::Box2d& box);
bool boxIsCollided(const std::vector<math::LineSegment2d>& line_segs, const math::Box2d& box);
bool boxIsCollided(const RoiDecideResult& roi, const math::Box2d& box);

void trajectoryPointTimeCompensate(const Eigen::Matrix4d tf_ego_2_map_orin, const Eigen::Matrix4d tf_map_2_ego_target,
                                   PathPt& path_pt);
void trajectoryTimeCompensate(const Eigen::Matrix4d tf_ego_2_map_orin, const Eigen::Matrix4d tf_map_2_ego_target,
                              std::vector<PathPt>& path);

void appendLineSegments(const std::vector<math::Vec2d>& points, std::vector<math::LineSegment2d>& segments,
                        double min_length, double max_heading_err);

std::tuple<bool, double, double, int, int> getOverlapRange(const math::Polygon2d& polygon,
                                                           const std::vector<math::LineSegment2d>& segments,
                                                           const std::vector<double>& accumulated_s);
}  // namespace gpal::pnc::planning