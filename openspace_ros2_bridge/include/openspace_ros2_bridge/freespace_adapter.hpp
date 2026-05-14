#pragma once
#include <vector>
#include <memory>
#include "local_view/Freespace.h"
#include "math/vec2d.h"

namespace openspace_ros2_bridge {

struct PolygonRegion {
  std::vector<math::Vec2d> vertices;
  bool is_occupied = true;  // true = obstacle, false = free
};

class FreespaceBuilder {
 public:
  FreespaceBuilder() = default;

  void init(double resolution, double map_length, double map_width,
            double origin_x, double origin_y);

  void addPolygon(const PolygonRegion& region);

  std::shared_ptr<gpal::pnc::planning::Freespace> build();

 private:
  gpal::pnc::planning::FreespaceConfig config_;
  std::vector<PolygonRegion> regions_;
};

}  // namespace openspace_ros2_bridge
