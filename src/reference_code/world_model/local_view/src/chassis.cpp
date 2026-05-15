#include "local_view/chassis.h"

namespace gpal::pnc::planning {

Chassis::Chassis() { Clear(); }

Chassis::~Chassis() {}

void Chassis::Clear() {
  // StampedBase::reset();
  n_gear_ = 0;
  f_speed_ = 0.0;
  f_front_left_wheel_speed_ = 0.0;
  f_front_right_wheel_speed_ = 0.0;
  f_rear_left_wheel_speed_ = 0.0;
  f_rear_right_wheel_speed_ = 0.0;
  f_acc_ = 0.0;
  f_lateral_acc_ = 0.0;
  f_longitu_acc_ = 0.0;
  f_steering_angle_ = 0.0;
  f_yaw_rate_ = 0.0;
  n_driver_belt_ = 0;
  n_driver_override_ = 0;
  n_turn_switch_ = 0;
  n_steer_hands_on_ = 0;
  f_driver_steering_torque_ = 0.0;
  f_driver_input_rod_travel_ = 0.0;
  f_driver_gas_pedal_position_ = 0.0;
  n_mcu_brake_system_control_state_feedback_ = 0;
  n_mcu_lateral_control_state_feedback_ = 0;
  f_driver_brake_pedal_position_ = 0.0;
  b_is_autodrive_active_ = false;
  b_is_acc_active_ = false;
  b_is_break_pedal_pressed_ = false;
  f_driver_steering_angular_rate_ = 0.0;
  front_left_wheel_direction_ =
      gpal::proto::AxleInfo_WheelPulseDirection::AxleInfo_WheelPulseDirection_PULSE_DIRECTION_FORWARD;
  front_right_wheel_direction_ =
      gpal::proto::AxleInfo_WheelPulseDirection::AxleInfo_WheelPulseDirection_PULSE_DIRECTION_FORWARD;
  rear_left_wheel_direction_ =
      gpal::proto::AxleInfo_WheelPulseDirection::AxleInfo_WheelPulseDirection_PULSE_DIRECTION_FORWARD;
  rear_right_wheel_direction_ =
      gpal::proto::AxleInfo_WheelPulseDirection::AxleInfo_WheelPulseDirection_PULSE_DIRECTION_FORWARD;
  left_direction_light_ = false;
  right_direction_light_ = false;
  left_direction_switch_state_ = false;
  right_direction_switch_state_ = false;
  front_left_door_state_ = gpal::proto::ActuatorInfo_DoorStatus::ActuatorInfo_DoorStatus_DOOR_STATUS_CLOSED;
  front_right_door_state_ = gpal::proto::ActuatorInfo_DoorStatus::ActuatorInfo_DoorStatus_DOOR_STATUS_CLOSED;
  rear_left_door_state_ = gpal::proto::ActuatorInfo_DoorStatus::ActuatorInfo_DoorStatus_DOOR_STATUS_CLOSED;
  rear_right_door_state_ = gpal::proto::ActuatorInfo_DoorStatus::ActuatorInfo_DoorStatus_DOOR_STATUS_CLOSED;
  vehicle_standstill_ = false;
  abs_active_ = false;
}

}  // namespace gpal::pnc::planning