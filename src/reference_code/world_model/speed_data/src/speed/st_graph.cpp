/**
 * @file st_graph.cpp
 * @brief 时空图数据容器
 * @details 本类管理速度规划中的时空约束数据
 */
#include "speed/st_graph.h"

namespace gpal::pnc::planning {

void StGraph::clear() {
  init_ = false;
  has_back_obs_risk_ = false;
  min_s_on_st_boundaries_ = 0.0;
  init_point_ = TrajectoryPt();
  cruise_speed_ = 0.0;
  path_data_length_ = 0.0;
  path_length_by_conf_ = 0.0;
  total_time_by_conf_ = 0.0;

  st_boundaries_.clear();
  st_drivable_boundaries_.clear();
}

void StGraph::loadData(const std::vector<const STBoundary*>& st_boundaries, const double min_s_on_st_boundaries,
                       const TrajectoryPt& init_point, const double path_data_length, const double total_time_by_conf) {
  init_ = true;
  st_boundaries_ = st_boundaries;
  min_s_on_st_boundaries_ = min_s_on_st_boundaries;
  init_point_ = init_point;
  path_data_length_ = path_data_length;
  total_time_by_conf_ = total_time_by_conf;
}

const std::vector<const STBoundary*>& StGraph::st_boundaries() const { return st_boundaries_; }

std::vector<const STBoundary*>* StGraph::mutable_st_boundaries() { return &st_boundaries_; }

double StGraph::min_s_on_st_boundaries() const { return min_s_on_st_boundaries_; }

const TrajectoryPt& StGraph::init_point() const { return init_point_; }

double StGraph::path_length() const { return path_data_length_; }

double StGraph::total_time_by_conf() const { return total_time_by_conf_; }

bool StGraph::setSTDrivableBoundaries(size_t t_index, const STDrivableBoundary& st_drivable_boundary) {
  if (t_index >= st_drivable_boundaries_.size()) {
    return false;
  }
  st_drivable_boundaries_[t_index] = st_drivable_boundary;
  return true;
}

void StGraph::calcSUpperBoundsProjectedSpeed(double time_window_length) {
  for (size_t i = 0UL; i < st_drivable_boundaries_.size(); i++) {
    if (st_drivable_boundaries_[i].upper_obj_id != "None") {
      double sigma_t = 0.0;
      double sigma_s = 0.0;
      double sigma_t_s = 0.0;
      double sigma_t_sq = 0.0;
      double N = 0.0;
      for (size_t j = i; j < st_drivable_boundaries_.size(); j++) {
        bool in_time_window = (st_drivable_boundaries_[j].t <= st_drivable_boundaries_[i].t + time_window_length) &&
                              (st_drivable_boundaries_[j].upper_obj_id == st_drivable_boundaries_[i].upper_obj_id);
        if (!in_time_window) {
          break;
        } else {
          double t = st_drivable_boundaries_[j].t;
          double s = st_drivable_boundaries_[j].s_upper_bound;
          sigma_t += t;
          sigma_s += s;
          sigma_t_s += t * s;
          sigma_t_sq += t * t;
          N += 1.0;
        }
      }

      if ((N >= 2.0) && (fabs(N * sigma_t_sq - sigma_t * sigma_t) >= 1e-10)) {
        st_drivable_boundaries_[i].v_upper =
            fmin(st_drivable_boundaries_[i].v_upper,
                 (sigma_t * sigma_s - N * sigma_t_s) / (sigma_t * sigma_t - N * sigma_t_sq));
      }
    }
  }
}

bool StGraph::resizeSTDrivableBoundaries(size_t drivable_boundary_size) {
  if (st_drivable_boundaries_.size() != drivable_boundary_size) {
    st_drivable_boundaries_.resize(drivable_boundary_size);
  }
  return true;
}

const STDrivableBoundaries& StGraph::st_drivable_boundaries() const { return st_drivable_boundaries_; }

}  // namespace gpal::pnc::planning
