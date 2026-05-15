#include "openspace_ros2_bridge/freespace_adapter.hpp"
#include <cmath>
#include <algorithm>

namespace openspace_ros2_bridge {

void FreespaceBuilder::init(double resolution, double map_length, double map_width,
                            double origin_x, double origin_y) {
  config_.mutable_grid_data()->set_resolution(resolution);
  config_.mutable_grid_data()->set_map_length(static_cast<int>(map_length / resolution));
  config_.mutable_grid_data()->set_map_width(static_cast<int>(map_width / resolution));
  config_.mutable_grid_data()->set_origin_x(static_cast<int>(origin_x / resolution));
  config_.mutable_grid_data()->set_origin_y(static_cast<int>(origin_y / resolution));
}

void FreespaceBuilder::addPolygon(const PolygonRegion& region) {
  regions_.push_back(region);
}

std::shared_ptr<gpal::pnc::planning::Freespace> FreespaceBuilder::build() {
  auto freespace = std::make_shared<gpal::pnc::planning::Freespace>();
  freespace->init(config_);

  auto* grid_map = freespace->mutable_grid_map();

  for (const auto& region : regions_) {
    if (!region.is_occupied) continue;
    if (region.vertices.size() < 3) continue;

    // Compute bounding box for the polygon
    double min_x = region.vertices[0].x(), max_x = min_x;
    double min_y = region.vertices[0].y(), max_y = min_y;
    for (const auto& v : region.vertices) {
      min_x = std::min(min_x, v.x());
      max_x = std::max(max_x, v.x());
      min_y = std::min(min_y, v.y());
      max_y = std::max(max_y, v.y());
    }

    double res = grid_map->resolution();
    // Rasterize polygon: mark all grid cells inside as occupied
    for (double x = min_x; x <= max_x; x += res * 0.5) {
      for (double y = min_y; y <= max_y; y += res * 0.5) {
        // Point-in-polygon test
        bool inside = false;
        size_t n = region.vertices.size();
        for (size_t i = 0, j = n - 1; i < n; j = i++) {
          double xi = region.vertices[i].x(), yi = region.vertices[i].y();
          double xj = region.vertices[j].x(), yj = region.vertices[j].y();
          if (((yi > y) != (yj > y)) &&
              (x < (xj - xi) * (y - yi) / (yj - yi) + xi)) {
            inside = !inside;
          }
        }
        if (inside) {
          grid_map->setOccupy(x, y);
        }
      }
    }
  }

  freespace->updateGridMap();
  return freespace;
}

}  // namespace openspace_ros2_bridge
