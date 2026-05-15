#pragma once
#include <string>
#include <map>
#include <vector>

namespace gpal::pnc::planning {

struct DynamicArcModelConfig {
  std::string param_;
  float value_ = 0;
  const std::string& param() const { return param_; }
  float value() const { return value_; }
  void set_param(const std::string& p) { param_ = p; }
  void set_value(float v) { value_ = v; }
};

struct DynamicArcModelConfigs {
  std::vector<DynamicArcModelConfig> dynamic_arc_model_configs_;
  int dynamic_arc_model_configs_size() const { return dynamic_arc_model_configs_.size(); }
  const DynamicArcModelConfig& dynamic_arc_model_configs(int i) const { return dynamic_arc_model_configs_[i]; }
  const auto& dynamic_arc_model_configs() const { return dynamic_arc_model_configs_; }
};

struct ArcModelConfig {
  int32_t steering_angle_discrete_num_ = 1;
  double move_step_ = 0.3;
  int32_t move_nums_ = 5;
  float steering_penalty_ = 1.0;
  float changing_gear_penalty_ = 10.0;
  float steering_change_penalty_ = 0.0;
  float moving_penalty_ = 1.0;
  float collision_penalty_ = 0.0;
  float collision_penalty_range_ = 6.0;
  float shot_distance_ = 8.0;
  float map_x_lower_ = -1000, map_y_lower_ = -1000, map_x_upper_ = 1000, map_y_upper_ = 1000;
  int32_t angular_upper_ = 300;
  float map_grid_resolution_ = 1.0;
  float steering_radian_ = 0.535;
  float search_stop_distance_x_ = 0.5;
  float search_stop_distance_y_ = 0.5;
  float search_stop_angle_ = 5.0;
  float min_path_length_limit_ = 1.0;
  float search_time_limit_ = 10.0;
  bool debug_print_switch_ = false;
  uint32_t straight_extend_num_ = 1;
  uint32_t max_dynamic_straight_extend_num_ = 1;
  float last_leg_min_length_limit_ = 1.0;
  float min_path_length_limit_before_rs_ = 1.0;
  float width_collision_buff_ = 0.1;
  float length_collision_buff_ = 0.1;
  float dynamic_straight_extend_width_collision_buff_ = 0.2;
  float dynamic_straight_extend_length_collision_buff_ = 0.5;
  bool enable_s_shape_ = true;
  bool enable_start_ignore_ = true;
  int32_t search_start_ignore_num_ = 2;
  std::map<std::string, DynamicArcModelConfigs> dynamic_arc_model_config_map_;

  int32_t steering_angle_discrete_num() const { return steering_angle_discrete_num_; }
  double move_step() const { return move_step_; }
  int32_t move_nums() const { return move_nums_; }
  float steering_penalty() const { return steering_penalty_; }
  float changing_gear_penalty() const { return changing_gear_penalty_; }
  float steering_change_penalty() const { return steering_change_penalty_; }
  float moving_penalty() const { return moving_penalty_; }
  float collision_penalty() const { return collision_penalty_; }
  float collision_penalty_range() const { return collision_penalty_range_; }
  float shot_distance() const { return shot_distance_; }
  float map_x_lower() const { return map_x_lower_; }
  float map_y_lower() const { return map_y_lower_; }
  float map_x_upper() const { return map_x_upper_; }
  float map_y_upper() const { return map_y_upper_; }
  int32_t angular_upper() const { return angular_upper_; }
  float map_grid_resolution() const { return map_grid_resolution_; }
  float steering_radian() const { return steering_radian_; }
  float search_stop_distance_x() const { return search_stop_distance_x_; }
  float search_stop_distance_y() const { return search_stop_distance_y_; }
  float search_stop_angle() const { return search_stop_angle_; }
  float min_path_length_limit() const { return min_path_length_limit_; }
  float search_time_limit() const { return search_time_limit_; }
  bool debug_print_switch() const { return debug_print_switch_; }
  uint32_t straight_extend_num() const { return straight_extend_num_; }
  uint32_t max_dynamic_straight_extend_num() const { return max_dynamic_straight_extend_num_; }
  float last_leg_min_length_limit() const { return last_leg_min_length_limit_; }
  float min_path_length_limit_before_rs() const { return min_path_length_limit_before_rs_; }
  float width_collision_buff() const { return width_collision_buff_; }
  float length_collision_buff() const { return length_collision_buff_; }
  float dynamic_straight_extend_width_collision_buff() const { return dynamic_straight_extend_width_collision_buff_; }
  float dynamic_straight_extend_length_collision_buff() const { return dynamic_straight_extend_length_collision_buff_; }
  bool enable_s_shape() const { return enable_s_shape_; }
  bool enable_start_ignore() const { return enable_start_ignore_; }
  int32_t search_start_ignore_num() const { return search_start_ignore_num_; }
  const auto& dynamic_arc_model_config_map() const { return dynamic_arc_model_config_map_; }
};

struct OpenspaceSearchConfig {
  std::map<std::string, ArcModelConfig> arc_model_configs_;
  const std::map<std::string, ArcModelConfig>& arc_model_configs() const { return arc_model_configs_; }
};

}  // namespace gpal::pnc::planning
