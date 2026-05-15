#pragma once

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include "gpal-interface/planning/trajectory.pb.h"
#include "local_view/function_state.h"
#include "obstacle/obstacle.h"
#include "path/path_data.h"
// #include "util/log_config.hpp"
#include "decision_data/decision_result.h"
#include "qp_solver.h"
#include "ellipse_object_info/ellipse_object_info.h"

namespace gpal::pnc::planning {
#define STLOG(level, ...) ERT_LOG_##level("[spatiotemporal] ", __VA_ARGS__)
struct SpatiotemporalState {
  double x{0.0};
  double y{0.0};
  double theta{0.0};
  double l{0.0};
  double steer{0.0};
  double dsteer{0.0};
  double s{0.0};
  double v{0.0};
  double a{0.0};
  double jerk{0.0};
};

enum class ConflictAreaType { UNKNOWN, OVERTAKE, YIELD };

struct ConflictArea {
  string conflict_object_id = "";
  int start_index{0};
  int end_index{0};
  double start_s{0.0};
  double end_s{0.0};
  bool is_conflict{false};
  ConflictAreaType conflict_area_type{ConflictAreaType::UNKNOWN};
};

struct InteractionInfo {
  float upper_bound_s = std::numeric_limits<float>::max();
  float lower_bound_s = std::numeric_limits<float>::lowest();
  bool has_upper_bound_object = false;
  bool has_lower_bound_object = false;
  ObjectInfo upper_bound_object;
  ObjectInfo lower_bound_object;
  float t = 0.0;
};

  struct ParamIndexCache {
    // 参考线参数
    int x_ref = -1;
    int y_ref = -1;
    int theta_ref = -1;
    int kappa_ref = -1;
    int steer_ref = -1;
    int xf_ref = -1;
    int yf_ref = -1;
    int thetaf_ref = -1;
    int xr_ref = -1;
    int yr_ref = -1;
    int thetar_ref = -1;

    // 目标参数
    int s_coarse = -1;
    int v_coarse = -1;
    int a_coarse = -1;
    int l_offset = -1;

    // 边界参数
    int LHardLowerBound = -1;
    int LHardUpperBound = -1;
    int LFHardLowerBound = -1;
    int LFHardUpperBound = -1;
    int LRHardLowerBound = -1;
    int LRHardUpperBound = -1;
    int LSoftLowerBound = -1;
    int LSoftUpperBound = -1;
    int LFSoftLowerBound = -1;
    int LFSoftUpperBound = -1;
    int LRSoftLowerBound = -1;
    int LRSoftUpperBound = -1;
    int SteerLowerBound = -1;
    int SteerUpperBound = -1;
    int DSteerLowerBound = -1;
    int DSteerUpperBound = -1;
    int VSoftLowerBound = -1;
    int VSoftUpperBound = -1;
    int VHardLowerBound = -1;
    int VHardUpperBound = -1;

    // 障碍物参数（4个障碍物）
    std::array<int, 8> obs_a;
    std::array<int, 8> obs_b;
    std::array<int, 8> obs_a_square;
    std::array<int, 8> obs_b_square;
    std::array<int, 8> obs_x;
    std::array<int, 8> obs_y;
    std::array<int, 8> obs_theta;
    std::array<int, 8> obs_v;
    std::array<int, 8> obs_length;
    std::array<int, 8> obs_width;
    std::array<int, 8> obs_cos_theta;
    std::array<int, 8> obs_sin_theta;
    std::array<int, 8> obs_weight;
    std::array<int, 8> obs_avoided_bound;

    // 其他
    int s_coarse_weight = -1;
    int v_coarse_weight = -1;

    bool initialized = false;

    void init(OcpVariable* ptr) {
      if (initialized)
        return;

      // 参考线参数
      x_ref = ptr->getParamIndex("x_ref");
      y_ref = ptr->getParamIndex("y_ref");
      theta_ref = ptr->getParamIndex("theta_ref");
      kappa_ref = ptr->getParamIndex("kappa_ref");
      steer_ref = ptr->getParamIndex("steer_ref");
      xf_ref = ptr->getParamIndex("xf_ref");
      yf_ref = ptr->getParamIndex("yf_ref");
      thetaf_ref = ptr->getParamIndex("thetaf_ref");
      xr_ref = ptr->getParamIndex("xr_ref");
      yr_ref = ptr->getParamIndex("yr_ref");
      thetar_ref = ptr->getParamIndex("thetar_ref");

      // 目标参数
      s_coarse = ptr->getParamIndex("s_coarse");
      v_coarse = ptr->getParamIndex("v_coarse");
      a_coarse = ptr->getParamIndex("a_coarse");
      l_offset = ptr->getParamIndex("l_offset");

      // 边界参数
      LHardLowerBound = ptr->getParamIndex("LHardLowerBound");
      LHardUpperBound = ptr->getParamIndex("LHardUpperBound");
      LFHardLowerBound = ptr->getParamIndex("LFHardLowerBound");
      LFHardUpperBound = ptr->getParamIndex("LFHardUpperBound");
      LRHardLowerBound = ptr->getParamIndex("LRHardLowerBound");
      LRHardUpperBound = ptr->getParamIndex("LRHardUpperBound");
      LSoftLowerBound = ptr->getParamIndex("LSoftLowerBound");
      LSoftUpperBound = ptr->getParamIndex("LSoftUpperBound");
      LFSoftLowerBound = ptr->getParamIndex("LFSoftLowerBound");
      LFSoftUpperBound = ptr->getParamIndex("LFSoftUpperBound");
      LRSoftLowerBound = ptr->getParamIndex("LRSoftLowerBound");
      LRSoftUpperBound = ptr->getParamIndex("LRSoftUpperBound");
      SteerLowerBound = ptr->getParamIndex("SteerLowerBound");
      SteerUpperBound = ptr->getParamIndex("SteerUpperBound");
      DSteerLowerBound = ptr->getParamIndex("DSteerLowerBound");
      DSteerUpperBound = ptr->getParamIndex("DSteerUpperBound");
      VSoftLowerBound = ptr->getParamIndex("VSoftLowerBound");
      VSoftUpperBound = ptr->getParamIndex("VSoftUpperBound");
      VHardLowerBound = ptr->getParamIndex("VHardLowerBound");
      VHardUpperBound = ptr->getParamIndex("VHardUpperBound");

      // 障碍物参数
      for (int i = 0; i < 8; i++) {
        std::string idx_str = std::to_string(i);
        obs_a[i] = ptr->getParamIndex("a_" + idx_str);
        obs_b[i] = ptr->getParamIndex("b_" + idx_str);
        obs_a_square[i] = ptr->getParamIndex("a_" + idx_str + "_square");
        obs_b_square[i] = ptr->getParamIndex("b_" + idx_str + "_square");
        obs_x[i] = ptr->getParamIndex("x_" + idx_str);
        obs_y[i] = ptr->getParamIndex("y_" + idx_str);
        obs_theta[i] = ptr->getParamIndex("theta_" + idx_str);
        obs_v[i] = ptr->getParamIndex("v_" + idx_str);
        obs_length[i] = ptr->getParamIndex("length_" + idx_str);
        obs_width[i] = ptr->getParamIndex("width_" + idx_str);
        obs_cos_theta[i] = ptr->getParamIndex("cos_theta_" + idx_str);
        obs_sin_theta[i] = ptr->getParamIndex("sin_theta_" + idx_str);
        obs_weight[i] = ptr->getParamIndex("obs_" + idx_str + "_weight");
        obs_avoided_bound[i] = ptr->getParamIndex("Obs_"+ idx_str+"_AvoidedBound");
      }

      // 其他
      s_coarse_weight = ptr->getParamIndex("s_coarse_weight");
      v_coarse_weight = ptr->getParamIndex("v_coarse_weight");

      initialized = true;
    }
  };

namespace spatiotemporal_functions {
inline double linearInterpolation(double i, double N, double value_lower, double value_upper) {
  if (std::abs(N) < 1e-9)
    return value_lower;
  if (i >= N)
    return value_upper;

  const double step = (value_upper - value_lower) / N;
  double result = value_lower + i * step;

  // 边界截断功能
  return std::clamp(result, min(value_lower, value_upper), max(value_lower, value_upper));
};

inline double getUnifySpaceHeading(const double heading_base, const double heading) {
  double delta_heading = math::NormalizeAngle(heading - heading_base);
  return heading_base + delta_heading;
};

}  // namespace spatiotemporal_functions
}  // namespace gpal::pnc::planning
