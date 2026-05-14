#pragma once
#include <vector>
#include <string>
#include "math/vec2d.h"

namespace gpal::pnc::planning {

class PathBoundary {
 public:
  const std::vector<std::pair<double, double>>& boundary() const { return boundary_; }
  std::vector<std::pair<double, double>>* mutableBoundary() { return &boundary_; }
  double delta_s() const { return delta_s_; }

 private:
  std::vector<std::pair<double, double>> boundary_;
  double delta_s_ = 0.1;
};

}  // namespace gpal::pnc::planning
