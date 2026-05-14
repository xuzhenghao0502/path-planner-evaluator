#include "openspace_path_planner/core/generator/openspace_roi_decider.h"

namespace gpal::pnc::planning {

void RoiDeciderBaseHAStar::init() {
  clear();
}

void RoiDeciderBaseHAStar::clear() {
  roi_boundary_.clear();
  roi_boundary_segs_.clear();
}

bool RoiDeciderBaseHAStar::generateRoi(const LocalView& local_view,
                                       const std::vector<Decision::DecisionObject>& all_obstacles) {
  if (!formulateBoundaryConstraints(local_view, roi_boundary_, all_obstacles)) {
    OPENSPACE_LOG(E, "[ROIDecider]ROI failed   ! ");
    return false;
  }
  roi_decide_result_.roi_xy_boundary = roi_map_boundary_;
  roi_decide_result_.obstacles_vert = obstacles_vertices_;
  roi_decide_result_.obstacles_edges_num = obstacles_edges_num_;
  roi_decide_result_.obstacles_linesegments = *obstacles_linesegments_;
  roi_decide_result_.obstacles_linesegments_map = *obstacles_linesegments_map_;
  // SFIELD_INFO(openspace, "ROI success   ! ");
  return true;
}

bool RoiDeciderBaseHAStar::formulateBoundaryConstraints(const LocalView& local_view,
                                                        const std::vector<ObstaclesVertice>& roi_boundary,
                                                        const std::vector<Decision::DecisionObject>& obstacles) {
  if (!loadObstacleInVertices(local_view, roi_boundary, obstacles)) {
    OPENSPACE_LOG(E, "[ROIDecider]Fail at LoadObstacleInVertices()");
    return false;
  }
  return true;
}

bool RoiDeciderBaseHAStar::loadObstacleInVertices(const LocalView& local_view,
                                                  const std::vector<ObstaclesVertice>& roi_boundary,
                                                  const std::vector<Decision::DecisionObject>& obstacles) {
  if (obstacles_linesegments_ == nullptr) {
    obstacles_linesegments_ = std::make_unique<std::vector<ObstaclesLinesegment>>();
    obstacles_linesegments_map_ = std::make_unique<std::vector<ObstaclesLinesegment>>();
  }
  obstacles_linesegments_->clear();
  obstacles_linesegments_map_->clear();
  obstacles_vertices_.clear();
  std::vector<ObstaclesVertice> obstacles_vertices_map;
  int obstacles_num = 0;  // OD的数量
  Eigen::Matrix4d tf_map_2_ego = local_view.getLocalizationPtr()->getTfMap2Ego();
  Eigen::Matrix4d tf_ego_2_map = local_view.getLocalizationPtr()->getTfEgo2Map();

  for (auto obs : obstacles) {
    ++obstacles_num;
    std::vector<math::Vec2d> obs_vertices = obs.cur_box.GetAllCorners();
    std::vector<math::Vec2d> map_obs_vertices;
    OpenspaceObjectType obs_type = obs.type;
    std::string obs_id = obs.id;
    for (const auto& p : obs_vertices) {
      PathPt od_point_ego = PathPt(p.x(), p.y(), 0.0, 0.0, 0.0);
      transfer::transformPoint(tf_ego_2_map, &od_point_ego);
      map_obs_vertices.emplace_back(od_point_ego.x(), od_point_ego.y());
    }

    obstacles_vertices_.push_back(std::make_tuple(obs_id, obs_type, obs_vertices));
    obstacles_vertices_map.push_back(std::make_tuple(obs_id, obs_type, map_obs_vertices));
  }
  std::vector<ObstaclesVertice> roi_boundary_map;
  for (const auto& bound : roi_boundary) {
    std::vector<math::Vec2d> bound_points_map;
    for (const auto& b : std::get<2>(bound)) {
      PathPt bound_point_map = PathPt(b.x(), b.y(), 0.0, 0.0, 0.0);
      transfer::transformPoint(tf_ego_2_map, &bound_point_map);

      bound_points_map.emplace_back(bound_point_map.x(), bound_point_map.y());
    }
    roi_boundary_map.emplace_back(std::get<0>(bound), std::get<1>(bound), bound_points_map);
  }
  int od_num = obstacles_vertices_.size();
  obstacles_vertices_.insert(obstacles_vertices_.end(), roi_boundary.begin(), roi_boundary.end());
  obstacles_vertices_map.insert(obstacles_vertices_map.end(), roi_boundary_map.begin(), roi_boundary_map.end());
  obstacles_num += roi_boundary.size();

  obstacles_num_ = obstacles_num;

  for (const auto& obstacle_vertices : obstacles_vertices_) {
    int vertices_num = std::get<2>(obstacle_vertices).size();
    if (vertices_num < 2) {
      continue;
    }
    ObstaclesLinesegment obstacle_linesegments;
    for (int i = 0; i < vertices_num - 1; ++i) {
      math::LineSegment2d line_segment =
          math::LineSegment2d(std::get<2>(obstacle_vertices)[i], std::get<2>(obstacle_vertices)[i + 1]);
      std::get<2>(obstacle_linesegments).emplace_back(line_segment);
    }
    math::LineSegment2d line_segment =
        math::LineSegment2d(std::get<2>(obstacle_vertices).back(), std::get<2>(obstacle_vertices).front());
    std::get<2>(obstacle_linesegments).emplace_back(line_segment);
    std::get<1>(obstacle_linesegments) = std::get<1>(obstacle_vertices);
    std::get<0>(obstacle_linesegments) = std::get<0>(obstacle_vertices);
    obstacles_linesegments_->emplace_back(obstacle_linesegments);
  }

  for (const auto& obstacle_vertice_map : obstacles_vertices_map) {
    int vertices_num = std::get<2>(obstacle_vertice_map).size();
    if (vertices_num < 2) {
      continue;
    }
    ObstaclesLinesegment obstacle_linesegments;
    for (int i = 0; i < vertices_num - 1; ++i) {
      math::LineSegment2d line_segment =
          math::LineSegment2d(std::get<2>(obstacle_vertice_map)[i], std::get<2>(obstacle_vertice_map)[i + 1]);
      std::get<2>(obstacle_linesegments).emplace_back(line_segment);
    }
    constexpr int min_polygon_vertices = 3;
    if (vertices_num >= min_polygon_vertices) {
      math::LineSegment2d line_segment =
          math::LineSegment2d(std::get<2>(obstacle_vertice_map).back(), std::get<2>(obstacle_vertice_map).front());
      std::get<2>(obstacle_linesegments).emplace_back(line_segment);
    }
    std::get<1>(obstacle_linesegments) = std::get<1>(obstacle_vertice_map);
    std::get<0>(obstacle_linesegments) = std::get<0>(obstacle_vertice_map);
    obstacles_linesegments_map_->emplace_back(obstacle_linesegments);
  }
  for (auto bound_segs : roi_boundary_segs_) {
    obstacles_linesegments_->emplace_back(bound_segs);

    for (auto& bound_seg : std::get<2>(bound_segs)) {
      math::Vec3d start(bound_seg.start(), 0.0);
      math::Vec3d end(bound_seg.end(), 0.0);
      transfer::transformPoint(tf_ego_2_map, &start);
      transfer::transformPoint(tf_ego_2_map, &end);
      bound_seg.reset(start, end);
    }
    obstacles_linesegments_map_->emplace_back(bound_segs);
  }

  return true;
}

}  // namespace gpal::pnc::planning