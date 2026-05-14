#pragma once
#include <vector>
#include <string>
#include "math/vec2d.h"
#include "math/line_segment2d.h"

namespace gpal::pnc::planning {

class Obstacle {
 public:
  const std::string& id() const { return id_; }
  std::vector<math::Vec2d> vertices() const { return {}; }
 private:
  std::string id_;
};

class IndexedObstacles {
 public:
  const std::vector<Obstacle>& obstacles() const { return obstacles_; }
 private:
  std::vector<Obstacle> obstacles_;
};

}  // namespace gpal::pnc::planning
