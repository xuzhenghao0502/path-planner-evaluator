#pragma once

namespace gpal::pnc::planning {

struct VehicleParam {
  double wheel_base_ = 2.8;
  double length_ = 4.8;
  double width_ = 1.9;
  double width_without_rearview_mirror_ = 1.9;
  double rear_overhang_ = 0.9;
  double rear_edge_to_ego_ = 3.5;

  double wheel_base() const { return wheel_base_; }
  double length() const { return length_; }
  double width() const { return width_; }
  double width_without_rearview_mirror() const { return width_without_rearview_mirror_; }
  double rear_overhang() const { return rear_overhang_; }
  double rear_edge_to_ego() const { return rear_edge_to_ego_; }

  void set_wheel_base(double v) { wheel_base_ = v; }
  void set_length(double v) { length_ = v; }
  void set_width(double v) { width_ = v; }
  void set_width_without_rearview_mirror(double v) { width_without_rearview_mirror_ = v; }
  void set_rear_overhang(double v) { rear_overhang_ = v; }
  void set_rear_edge_to_ego(double v) { rear_edge_to_ego_ = v; }
};

struct VehicleConfig {
  VehicleParam vehicle_param_;
  const VehicleParam& vehicle_param() const { return vehicle_param_; }
  VehicleParam* mutable_vehicle_param() { return &vehicle_param_; }
};

}  // namespace gpal::pnc::planning
