#include <base/singleton.h>
#include <fmt/chrono.h>
#include <gtest/gtest.h>

#include <any>

#include "config_manager/config_manager.h"

#define private public
#include "speed/st_graph.h"


namespace gpal::pnc::planning {

TEST(StGraphTest, ClearTest) {
  StGraph st_graph;

  st_graph.clear();

  std::vector<std::pair<STPoint, STPoint>> st_bound;
  std::vector<double> lateral_s;
  for (int i = 0; i < 51; i++) {
    STPoint pt1(20.0, i * 0.1);
    STPoint pt2(20.0 + 5.0, i * 0.1);
    st_bound.emplace_back(pt1, pt2);
    lateral_s.push_back(0.1);
  }
  STBoundary* st_boundary = new STBoundary(st_bound);
  st_boundary->id_ = "1001";
  st_boundary->set_lateral_signed_distances(lateral_s);
  std::vector<const STBoundary*> st_boundaries;
  st_boundaries.push_back(st_boundary);

  STDrivableBoundary st_drivable_boundary;

  for (int i = 0; i < 50; i++) {
    st_drivable_boundary.t = i * 0.1;
    st_drivable_boundary.s_upper_bound = 20;
  }
  TrajectoryPt init_point;
  st_graph.loadData(st_boundaries, 10, init_point, 50, 5);

  st_graph.st_boundaries();
  st_graph.mutable_st_boundaries();
  st_graph.min_s_on_st_boundaries();
  st_graph.init_point();
  st_graph.path_length();
  st_graph.total_time_by_conf();
  st_graph.st_drivable_boundaries();

  st_graph.setSTDrivableBoundaries(20, st_drivable_boundary);
  st_graph.setSTDrivableBoundaries(80, st_drivable_boundary);

  st_graph.calcSUpperBoundsProjectedSpeed(3);

  st_graph.resizeSTDrivableBoundaries(50);
  st_graph.resizeSTDrivableBoundaries(30);
}

TEST(StGraphTest, STDrivableBoundaries) {
  StGraph st_graph;

  STDrivableBoundary st_drivable_boundary;

  st_graph.resizeSTDrivableBoundaries(50);
  for (int i = 0; i < 50; i++) {
    st_drivable_boundary.t = i * 0.1;
    st_drivable_boundary.s_upper_bound = 20;

    if (i > 10) {
      st_drivable_boundary.upper_obj_id = "1001";
    }
    st_graph.setSTDrivableBoundaries(i, st_drivable_boundary);
  }

  st_graph.calcSUpperBoundsProjectedSpeed(3);
}

}  // namespace gpal::pnc::planning