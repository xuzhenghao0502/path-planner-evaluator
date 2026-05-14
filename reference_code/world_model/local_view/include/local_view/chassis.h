#pragma once

/**
 * @file
 * @brief Defines the Chassis class.
 */
#include "gpal-interface/hal/chassis_info.pb.h"
#include "util/base_struct.h"

namespace gpal::pnc::planning {
/**
 * @class Chassis
 *
 * @brief Implements a class of Chassis Data.
 */

class Chassis : public StampedBase {
 public:
  friend class ChassisInfoAdapter;
  explicit Chassis();
  ~Chassis();

  void Clear();

  // ERROR = 0; SPORT = 1; WINTER = 2; M = 3; NONE = 4; DRIVE = 5; NEUTRAL = 6; REVERSE = 7; PARK = 8;
  void set_Gear(const uint8_t& n_gear) { n_gear_ = n_gear; }
  uint8_t Gear() const { return n_gear_; }

  // m/s 正值，结合档位判断正负
  void set_Speed(const float& f_speed) { f_speed_ = f_speed; }
  float Speed() const { return f_speed_; }

  void set_FLWheelSpeed(const float& f_front_left_wheel_speed) { f_front_left_wheel_speed_ = f_front_left_wheel_speed; }
  float FLWheelSpeed() const { return f_front_left_wheel_speed_; }

  void set_FRWheelSpeed(const float& f_front_right_wheel_speed) {
    f_front_right_wheel_speed_ = f_front_right_wheel_speed;
  }
  float FRWheelSpeed() const { return f_front_right_wheel_speed_; }

  void set_RLWheelSpeed(const float& f_rear_left_wheel_speed) { f_rear_left_wheel_speed_ = f_rear_left_wheel_speed; }
  float RLWheelSpeed() const { return f_rear_left_wheel_speed_; }

  void set_RRWheelSpeed(const float& f_rear_right_wheel_speed) { f_rear_right_wheel_speed_ = f_rear_right_wheel_speed; }
  float RRWheelSpeed() const { return f_rear_right_wheel_speed_; }

  void set_Acc(const float& f_acc) { f_acc_ = f_acc; }
  float Acc() const { return f_acc_; }

  void set_LateralAcc(const float& f_lateral_acc) { f_lateral_acc_ = f_lateral_acc; }
  float LateralAcc() const { return f_lateral_acc_; }

  void set_LongituAcc(const float& f_longitu_acc) { f_longitu_acc_ = f_longitu_acc; }
  float LogituAcc() const { return f_longitu_acc_; }

  void set_SteeringAngle(const float& f_steering_angle) { f_steering_angle_ = f_steering_angle; }
  float SteeringAngle() const { return f_steering_angle_; }

  void set_YawRate(const float& f_yaw_rate) { f_yaw_rate_ = f_yaw_rate; }
  float YawRate() const { return f_yaw_rate_; }

  void setDrivingMode(const proto::PowertrainInfo::DrivingMode& driving_mode) { driving_mode_ = driving_mode; }
  const proto::PowertrainInfo::DrivingMode& drivingMode() const { return driving_mode_; }

  void set_DriverBelt(const uint8_t& n_driver_belt) { n_driver_belt_ = n_driver_belt; }
  uint8_t DriverBelt() const { return n_driver_belt_; }

  void set_DriverOverride(const uint8_t& n_driver_override) { n_driver_override_ = n_driver_override; }
  uint8_t DriverOverride() const { return n_driver_override_; }

  void set_TurnSwitch(const uint8_t& n_turn_switch) { n_turn_switch_ = n_turn_switch; }
  uint8_t TurnSwitch() const { return n_turn_switch_; }

  void set_SteerHandsOn(const uint8_t& n_steer_hands_on) { n_steer_hands_on_ = n_steer_hands_on; }
  uint8_t SteerHandsOn() const { return n_steer_hands_on_; }

  void set_DriverSteeringTorque(const float& f_driver_steering_torque) {
    f_driver_steering_torque_ = f_driver_steering_torque;
  }
  float DriverSteeringTorque() const { return f_driver_steering_torque_; }

  void set_DriverInputRodTravel(const float& f_driver_input_rod_travel) {
    f_driver_input_rod_travel_ = f_driver_input_rod_travel;
  }
  float DriverInputRodTravel() const { return f_driver_input_rod_travel_; }

  void set_DriverGasPedalPosition(const float& f_driver_gas_pedal_position) {
    f_driver_gas_pedal_position_ = f_driver_gas_pedal_position;
  }
  float DriverGasPedalPosition() const { return f_driver_gas_pedal_position_; }

  void set_McuBrakeSystemControlStateFeedback(const uint8_t& n_mcu_brake_system_control_state_feedback) {
    n_mcu_brake_system_control_state_feedback_ = n_mcu_brake_system_control_state_feedback;
  }
  uint8_t McuBrakeSystemControlStateFeedback() const { return n_mcu_brake_system_control_state_feedback_; }

  void set_McuLateralControlStateFeedback(const uint8_t& n_mcu_lateral_control_state_feedback) {
    n_mcu_lateral_control_state_feedback_ = n_mcu_lateral_control_state_feedback;
  }
  uint8_t McuLateralControlStateFeedback() const { return n_mcu_lateral_control_state_feedback_; }

  void set_DriverBrakePedalPosition(const float& f_driver_brake_pedal_position) {
    f_driver_brake_pedal_position_ = f_driver_brake_pedal_position;
  }
  float DriverBrakePedalPosition() const { return f_driver_brake_pedal_position_; }

  void set_IsAutodriveActive(const bool& b_is_autodrive_active) { b_is_autodrive_active_ = b_is_autodrive_active; }
  bool IsAutodriveActive() const { return b_is_autodrive_active_; }

  void set_IsAccActive(const bool& b_is_acc_active) { b_is_acc_active_ = b_is_acc_active; }
  bool IsAccActive() const { return b_is_acc_active_; }

  void set_IsBreakPedalPressed(const bool& b_is_break_pedal_pressed) { b_is_break_pedal_pressed_ = b_is_break_pedal_pressed; }
  bool IsBreakPedalPressed() const { return b_is_break_pedal_pressed_; }

  void set_DriverSteeringAngleRate(const float& f_driver_steering_angular_rate) {
    f_driver_steering_angular_rate_ = f_driver_steering_angular_rate;
  }
  float DriverSteeringAngleRate() const { return f_driver_steering_angular_rate_; }

  void set_FrontLeftWheelDirection(const gpal::proto::AxleInfo_WheelPulseDirection& front_left_wheel_direction) {
    front_left_wheel_direction_ = front_left_wheel_direction;
  };
  gpal::proto::AxleInfo_WheelPulseDirection FrontLeftWheelDirection() const { return front_left_wheel_direction_; }

  void set_FrontRightWheelDirection(const gpal::proto::AxleInfo_WheelPulseDirection& front_right_wheel_direction) {
    front_right_wheel_direction_ = front_right_wheel_direction;
  };
  gpal::proto::AxleInfo_WheelPulseDirection FrontRightWheelDirection() const { return front_right_wheel_direction_; }

  void set_RearLeftWheelDirection(const gpal::proto::AxleInfo_WheelPulseDirection& rear_left_wheel_direction) {
    rear_left_wheel_direction_ = rear_left_wheel_direction;
  };
  gpal::proto::AxleInfo_WheelPulseDirection RearLeftWheelDirection() const { return rear_left_wheel_direction_; }

  void set_RearRightWheelDirection(const gpal::proto::AxleInfo_WheelPulseDirection& rear_right_wheel_direction) {
    rear_right_wheel_direction_ = rear_right_wheel_direction;
  };
  gpal::proto::AxleInfo_WheelPulseDirection RearRightWheelDirection() const { return rear_right_wheel_direction_; }

  void set_LeftDirectionLight(const bool& left_direction_light) { left_direction_light_ = left_direction_light; };
  bool LeftDirectionLight() const { return left_direction_light_; }

  void set_RightDirectionLight(const bool& right_direction_light) { right_direction_light_ = right_direction_light; };
  bool RightDirectionLight() const { return right_direction_light_; }

  void set_LeftDirectionSwitchState(const bool& left_direction_switch_state) {
    left_direction_switch_state_ = left_direction_switch_state;
  };
  bool LeftDirectionSwitchState() const { return left_direction_switch_state_; }

  void set_RightDirectionSwitchState(const bool& right_direction_switch_state) {
    right_direction_switch_state_ = right_direction_switch_state;
  };
  bool RightDirectionSwitchState() const { return right_direction_switch_state_; }

  void set_FrontLeftDoorState(const gpal::proto::ActuatorInfo_DoorStatus& front_left_door_state) {
    front_left_door_state_ = front_left_door_state;
  };
  gpal::proto::ActuatorInfo_DoorStatus FrontLeftDoorState() const { return front_left_door_state_; }

  void set_FrontRightDoorState(const gpal::proto::ActuatorInfo_DoorStatus& front_right_door_state) {
    front_right_door_state_ = front_right_door_state;
  };
  gpal::proto::ActuatorInfo_DoorStatus FrontRightDoorState() const { return front_right_door_state_; }

  void set_RearLeftDoorState(const gpal::proto::ActuatorInfo_DoorStatus& rear_left_door_state) {
    rear_left_door_state_ = rear_left_door_state;
  };
  gpal::proto::ActuatorInfo_DoorStatus RearLeftDoorState() const { return rear_left_door_state_; }

  void set_RearRightDoorState(const gpal::proto::ActuatorInfo_DoorStatus& rear_right_door_state) {
    rear_right_door_state_ = rear_right_door_state;
  };
  gpal::proto::ActuatorInfo_DoorStatus RearRightDoorState() const { return rear_right_door_state_; }

  void set_VehicleStandstill(const bool& vehicle_standstill) { vehicle_standstill_ = vehicle_standstill; };
  bool VehicleStandstill() const { return vehicle_standstill_; }

  void set_AbsActive(const bool& abs_active) { abs_active_ = abs_active; };
  bool AbsActive() const { return abs_active_; }

 protected:
  uint8_t n_gear_ = 0;
  float f_speed_ = 0.0;
  float f_front_left_wheel_speed_ = 0.0;
  float f_front_right_wheel_speed_ = 0.0;
  float f_rear_left_wheel_speed_ = 0.0;
  float f_rear_right_wheel_speed_ = 0.0;
  float f_acc_ = 0.0;
  float f_lateral_acc_ = 0.0;
  float f_longitu_acc_ = 0.0;
  float f_steering_angle_ = 0.0;
  float f_yaw_rate_ = 0.0;
  proto::PowertrainInfo::DrivingMode driving_mode_ =
      proto::PowertrainInfo::DrivingMode::PowertrainInfo_DrivingMode_kInvalid;
  uint8_t n_driver_belt_ = 0;
  uint8_t n_driver_override_ = 0;
  uint8_t n_turn_switch_ = 0;
  uint8_t n_steer_hands_on_ = 0;
  float f_driver_steering_torque_ = 0.0;
  float f_driver_input_rod_travel_ = 0.0;
  float f_driver_gas_pedal_position_ = 0.0;
  uint8_t n_mcu_brake_system_control_state_feedback_ = 0;
  uint8_t n_mcu_lateral_control_state_feedback_ = 0;
  float f_driver_brake_pedal_position_ = 0.0;
  bool b_is_autodrive_active_ = false;
  bool b_is_acc_active_ = false;
  bool b_is_break_pedal_pressed_ = false;
  float f_driver_steering_angular_rate_ = 0.0;
  gpal::proto::AxleInfo_WheelPulseDirection front_left_wheel_direction_ =
      gpal::proto::AxleInfo_WheelPulseDirection::AxleInfo_WheelPulseDirection_PULSE_DIRECTION_FORWARD;
  gpal::proto::AxleInfo_WheelPulseDirection front_right_wheel_direction_ =
      gpal::proto::AxleInfo_WheelPulseDirection::AxleInfo_WheelPulseDirection_PULSE_DIRECTION_FORWARD;
  gpal::proto::AxleInfo_WheelPulseDirection rear_left_wheel_direction_ =
      gpal::proto::AxleInfo_WheelPulseDirection::AxleInfo_WheelPulseDirection_PULSE_DIRECTION_FORWARD;
  gpal::proto::AxleInfo_WheelPulseDirection rear_right_wheel_direction_ =
      gpal::proto::AxleInfo_WheelPulseDirection::AxleInfo_WheelPulseDirection_PULSE_DIRECTION_FORWARD;
  bool left_direction_light_ = false;
  bool right_direction_light_ = false;
  bool left_direction_switch_state_ = false;
  bool right_direction_switch_state_ = false;
  gpal::proto::ActuatorInfo_DoorStatus front_left_door_state_ =
      gpal::proto::ActuatorInfo_DoorStatus::ActuatorInfo_DoorStatus_DOOR_STATUS_CLOSED;
  gpal::proto::ActuatorInfo_DoorStatus front_right_door_state_ =
      gpal::proto::ActuatorInfo_DoorStatus::ActuatorInfo_DoorStatus_DOOR_STATUS_CLOSED;
  gpal::proto::ActuatorInfo_DoorStatus rear_left_door_state_ =
      gpal::proto::ActuatorInfo_DoorStatus::ActuatorInfo_DoorStatus_DOOR_STATUS_CLOSED;
  gpal::proto::ActuatorInfo_DoorStatus rear_right_door_state_ =
      gpal::proto::ActuatorInfo_DoorStatus::ActuatorInfo_DoorStatus_DOOR_STATUS_CLOSED;
  bool vehicle_standstill_ = false;
  bool abs_active_ = false;
};
}  // namespace gpal::pnc::planning