#pragma once
/**
 * @file
 * @brief Defines the console class.
 */
#include "basic_algorithm_lib/basic_algorithm_lib.h"
#include "gpal-interface/application/task_command.pb.h"
#include "gpal-interface/perception/perception_parking_slot.pb.h"
namespace gpal::pnc::planning {

enum class FollowingDistanceLevel { Invalid = 0, Min = 1, Low = 2, Mid = 3, High = 4, Max = 5 };

class Console : public StampedBase {
 public:
  friend class ConsoleAdapter;
  Console();
  ~Console() = default;

  void SetValidity(const bool is_valid) { is_valid_ = is_valid; }
  bool IsValid() const { return is_valid_; }

  void setParkTaskId(const int park_task_id) { park_task_id_ = park_task_id; }
  const int& parkTaskId() const { return park_task_id_; }

  void setTaskStage(const proto::TaskCommand::TaskStage& task_stage) { task_stage_ = task_stage; }
  const proto::TaskCommand::TaskStage& taskStage() const { return task_stage_; }

  void setDriveMode(const proto::DriveMode& drive_mode){ drive_mode_ = drive_mode; } 
  const proto::DriveMode& driveMode() const { return drive_mode_; }

  void setDrivingTaskInfo(const proto::DrivingTaskInfo& driving_task_info) { driving_task_info_ = driving_task_info; }
  const proto::DrivingTaskInfo& drivingTaskInfo() const { return driving_task_info_; }

  void setSpeedLimit(const float& speed_limit) { speed_limit_ = speed_limit; }
  const float& speedLimit() const { return speed_limit_; }

  void setAccDesiredSpeed(const float& acc_desired_speed) { acc_desired_speed_ = acc_desired_speed; }
  const float& accDesiredSpeed() const { return acc_desired_speed_; }

  void setFollowingDistanceLevel(const FollowingDistanceLevel& following_distance_level) {
    following_distance_level_ = following_distance_level;
  }
  const FollowingDistanceLevel& followingDistanceLevel() const { return following_distance_level_; }

  void setCurTaskStageStartTime(const double& cur_task_stage_start_time) {
    cur_task_stage_start_time_ = cur_task_stage_start_time;
  }
  const double& curTaskStageStartTime() const { return cur_task_stage_start_time_; }

  void setParkOutDirection(const proto::ParkingTaskInfo_ParkingOutDirection& park_out_direction) {
    park_out_direction_ = park_out_direction;
  }
  const proto::ParkingTaskInfo_ParkingOutDirection& parkOutDirection() const { return park_out_direction_; }

  void setEmergencyBrakeCommand(const bool& emergency_brake_command) {
    emergency_brake_command_ = emergency_brake_command;
  }
  const bool& emergencyBrakeCommand() const { return emergency_brake_command_; }

  // 远控兜底开关，仅在远控状态kADCRemoteControl下使用 
  void setRemoteAvoidCollisionCommand(const bool& remote_avoid_collision_command) {
    remote_avoid_collision_command_ = remote_avoid_collision_command;
  }
  const bool& remoteAvoidCollisionCommand() const { return remote_avoid_collision_command_; }

 private:
  bool is_valid_ = false;
  int park_task_id_ = -1;
  proto::TaskCommand::TaskStage task_stage_ = proto::TaskCommand::TaskStage::TaskCommand_TaskStage_kInvalidTask;
  double cur_task_stage_start_time_ = 0.0;
  proto::DrivingTaskInfo driving_task_info_;
  float speed_limit_ = kMaxSpeedMS;
  float acc_desired_speed_ = kMaxSpeedMS;
  FollowingDistanceLevel following_distance_level_ = FollowingDistanceLevel::Invalid;
  proto::ParkingTaskInfo_ParkingOutDirection park_out_direction_ =
      proto::ParkingTaskInfo_ParkingOutDirection_kInvalidDirection;
  proto::DriveMode drive_mode_ = proto::kInvalidDriveMode;
  bool remote_avoid_collision_command_ = true;
  bool emergency_brake_command_ = false;
};

}  // namespace gpal::pnc::planning
