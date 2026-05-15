#pragma once

#include <map>
#include <string>
#include <vector>

#include "decision_data/decision_common.h"
#include "math/box2d.h"
#include "path/discretized_path.h"
namespace gpal::pnc::adas {

struct TrajPoint {
  TrajPoint() = default;
  TrajPoint(const double x_in, const double y_in) : x(x_in), y(y_in) {}
  TrajPoint(const double x_in, const double y_in, const double theta_in) : x(x_in), y(y_in), theta(theta_in) {}
  TrajPoint(const double x_in, const double y_in, const double theta_in, const double s_in, const double l_in,
            const double v_in, const double a_in, const double t_in)
      : x(x_in), y(y_in), theta(theta_in), s(s_in), l(l_in), v(v_in), a(a_in), t(t_in) {}
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  double s = 0.0;
  double l = 0.0;
  double v = 0.0;
  double a = 0.0;
  double t = 0.0;
};

struct PidConfig {
  float kp;
  float ki;
  float kd;
  float saturation_output_high;
  float saturation_output_low;
  float saturation_integrator_high;
  float saturation_integrator_low;
  bool enable_integrator;
};

struct AdasEgo {
  planning::DiscretizedPath pre_path;
  planning::DiscretizedPath local_path;
  std::vector<planning::math::Box2d> ego_boxes;
  bool is_intervention;
};

struct AdasObject {
  float aeb_collision_time;
  float aeb_collision_distance;
  float aeb_collision_speed;
  float aeb_collision_acceleration;
  float aeb_collision_heading;
  float aeb_collision_halflength;
  float aeb_collision_halfwidth;
  planning::Decision::ObjectType aeb_collision_type;
};

struct AdasCommonData {
  AdasEgo ego{};
  std::map<std::string, AdasObject> obj_map{};
  const planning::DiscretizedPath& getPrePath() const { return ego.pre_path; }
  planning::DiscretizedPath* getMutablePrePath() { return &ego.pre_path; }
  const planning::DiscretizedPath& getLocalPath() const { return ego.local_path; }
  planning::DiscretizedPath* getMutableLocalPath() { return &ego.local_path; }
  const std::vector<planning::math::Box2d>& getEgoBoxes() const { return ego.ego_boxes; }
  std::vector<planning::math::Box2d>* getMutableEgoBoxes() { return &ego.ego_boxes; }
};

}  // namespace gpal::pnc::adas