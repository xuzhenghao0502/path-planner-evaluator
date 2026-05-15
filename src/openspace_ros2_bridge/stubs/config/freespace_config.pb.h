#pragma once
#include <cstdint>

namespace gpal::pnc::planning {

struct GridData {
  int32_t map_length_ = 800;
  int32_t map_width_ = 400;
  int32_t origin_x_ = 200;
  int32_t origin_y_ = 200;
  float resolution_ = 0.1;
  float force_bottom_boundary_ = 10.0;
  int32_t park_map_length_ = 400;
  int32_t park_map_width_ = 400;
  int32_t park_origin_x_ = 100;
  int32_t park_origin_y_ = 200;
  float park_resolution_ = 0.2;

  int32_t map_length() const { return map_length_; }
  int32_t map_width() const { return map_width_; }
  int32_t origin_x() const { return origin_x_; }
  int32_t origin_y() const { return origin_y_; }
  float resolution() const { return resolution_; }
  void set_resolution(float v) { resolution_ = v; }
  void set_map_length(int32_t v) { map_length_ = v; }
  void set_map_width(int32_t v) { map_width_ = v; }
  void set_origin_x(int32_t v) { origin_x_ = v; }
  void set_origin_y(int32_t v) { origin_y_ = v; }
  float force_bottom_boundary() const { return force_bottom_boundary_; }
  bool has_force_bottom_boundary() const { return force_bottom_boundary_ > 0; }

  int32_t park_map_length() const { return park_map_length_; }
  int32_t park_map_width() const { return park_map_width_; }
  int32_t park_origin_x() const { return park_origin_x_; }
  int32_t park_origin_y() const { return park_origin_y_; }
  float park_resolution() const { return park_resolution_; }
};

struct FreespaceConfig {
  GridData grid_data_;
  const GridData& grid_data() const { return grid_data_; }
  GridData* mutable_grid_data() { return &grid_data_; }
};

}  // namespace gpal::pnc::planning
