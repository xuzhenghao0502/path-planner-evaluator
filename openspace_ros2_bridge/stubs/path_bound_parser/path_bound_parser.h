#pragma once
#include <vector>
#include "math/vec2d.h"
#include "math/line_segment2d.h"
#include "path/path_data.h"

namespace gpal::pnc::planning {

struct PathBoundaryResult {
  std::vector<double> s_vec;
  std::vector<double> left_bound;
  std::vector<double> right_bound;
};

}  // namespace gpal::pnc::planning
