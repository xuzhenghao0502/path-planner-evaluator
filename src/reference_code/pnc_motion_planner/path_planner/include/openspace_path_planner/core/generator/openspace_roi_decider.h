#pragma once
#include "config/openspace/openspace_search.pb.h"
#include "config_manager/config_manager.h"
#include "decision_data/decision_result.h"
#include "local_view/local_view.h"
#include "openspace_path_planner/utils/openspace_common.h"

namespace gpal::pnc::planning {
struct RoiDecideResult {
  std::vector<double> roi_xy_boundary;
  std::vector<ObstaclesVertice> obstacles_vert;
  std::vector<ObstaclesLinesegment> obstacles_linesegments;
  std::vector<ObstaclesLinesegment> obstacles_linesegments_map;
  Eigen::MatrixXi obstacles_edges_num;
};

class RoiDeciderBaseHAStar {
 public:
  RoiDeciderBaseHAStar() : roi_map_boundary_({-1e9, 1e9, -1e9, 1e9}) { init(); };
  ~RoiDeciderBaseHAStar() {};

  void init();
  void clear();
  bool generateRoi(const LocalView& local_view, const std::vector<Decision::DecisionObject>& all_obstacles);
  void getRoiDecideResuilt(RoiDecideResult& res) { res = roi_decide_result_; };

  void setBoundary(std::vector<double> boundary) { roi_map_boundary_.assign(boundary.begin(), boundary.end()); };
  void addBoundary(std::tuple<std::string, OpenspaceObjectType, std::vector<math::Vec2d>> boundary) {
    roi_boundary_.push_back(boundary);
  };
  void addBoundary(std::tuple<std::string, OpenspaceObjectType, std::vector<math::LineSegment2d>> boundary) {
    roi_boundary_segs_.push_back(boundary);
  };

 private:
  bool formulateBoundaryConstraints(const LocalView& local_view, const std::vector<ObstaclesVertice>& roi_boundary,
                                    const std::vector<Decision::DecisionObject>& obstacles);
  bool loadObstacleInVertices(const LocalView& local_view, const std::vector<ObstaclesVertice>& roi_boundary,
                              const std::vector<Decision::DecisionObject>& obstacles);

 private:
  RoiDecideResult roi_decide_result_;  // roi 结果

  int obstacles_num_ = 0;
  std::vector<ObstaclesVertice> obstacles_vertices_;                               // OD的顶点的集合
  Eigen::MatrixXi obstacles_edges_num_;                                            // OD的边的数量的集合 // OD的数量
  std::unique_ptr<std::vector<ObstaclesLinesegment>> obstacles_linesegments_;      // OD的边的集合
  std::unique_ptr<std::vector<ObstaclesLinesegment>> obstacles_linesegments_map_;  // OD的边的集合（地图坐标系）
  std::vector<double> roi_map_boundary_;
  std::vector<ObstaclesVertice> roi_boundary_;
  std::vector<ObstaclesLinesegment> roi_boundary_segs_;
};

}  // namespace gpal::pnc::planning