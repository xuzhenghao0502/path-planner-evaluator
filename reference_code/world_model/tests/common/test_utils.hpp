#pragma once

#include <google/protobuf/text_format.h>

#include "base/singleton.h"
#include "config_manager/config_manager.h"
#include "gpal-interface/perception/perception_obstacle.pb.h"
#include "local_view/local_view.h"
#include "obstacle/obstacle.h"
#include "proto/prediction/prediction_config.pb.h"

namespace gpal {
namespace pnc {
namespace planning {

inline void InitConfigManager() {
  auto config_manager = Singleton<ConfigManager>::get_instance();
  char* dummy_argv[] = {const_cast<char*>("policy_planner")};
  config_manager->init(1, dummy_argv);
}

inline Localization MockLocalization() {
  Localization localization;
  localization.setIsValid(true);
  localization.setStamp(1234567890);
  auto* pose = localization.mutableVehicleAlignPosePoint();
  pose->set_x(10.0);
  pose->set_y(20.0);
  pose->set_z(0.5);
  pose->set_yaw(1.57);

  return localization;
}

inline proto::PerceptionObstacle MockPerceptionObstacle(size_t id = 1) {
  proto::PerceptionObstacle perception_obstacle;
  perception_obstacle.set_id(id);
  perception_obstacle.set_obstacle_type(proto::PerceptionObstacle::kTypeBicycle);

  auto* sub_obstacle = perception_obstacle.add_sub_obstacles();
  sub_obstacle->mutable_position()->set_x(10.0);
  sub_obstacle->mutable_position()->set_y(20.0);
  sub_obstacle->mutable_velocity()->set_x(1.0);
  sub_obstacle->mutable_velocity()->set_y(2.0);
  sub_obstacle->mutable_acceleration()->set_x(0.5);
  sub_obstacle->mutable_acceleration()->set_y(1.0);
  sub_obstacle->set_heading_angle(0.5);
  sub_obstacle->set_heading_rate(0.1);
  sub_obstacle->set_length(5.0);
  sub_obstacle->set_width(2.0);
  sub_obstacle->set_height(1.5);
  return perception_obstacle;
}

inline std::vector<planning::Obstacle> MockPlanningObstacle(size_t id = 1, double timestamp = 1000.0) {
  proto::PerceptionObstacle perception_obstacle = std::move(MockPerceptionObstacle(id));
  return planning::Obstacle::createObstacles(perception_obstacle, timestamp);
}

inline prediction::PredictionConfig MockPredictionConfig() {
  static const std::string kPredictionConfigText = R"(
    container_config {
      messages: {
        type: LOCALIZATION
      }
      messages: {
        type: PERCEPTION_OBSTACLES
      }

      obstacle_container_config {
        max_obstacle_num: 200
        perception_message_outdated_duration: 1.0
        valid_distance_to_ego: 500.0

        max_history_time: 2.0
        still_vehicle_speed_threshold: 0.5
        still_pedestrian_speed_threshold: 0.06
        still_velocity_threshold: 0.5
        still_acceleration_threshold: 1.0
      }

      prediction_map_config {
        enable: true
        max_leaf_size: 100
      }
    }

    scenario_config {
      enable: true
      prioritizer_config: {
        normal_distance_to_ego: 120.0
        caution_range {
          front_distance: 100.0
          rear_distance: 60.0
          lateral_distance: 40.0
        }
      }
    }

    predictor_config {
      prediction_duration: 5.0
      prediction_time_resolution: 0.1
      max_trajectory_point_num: 100
      
      start_decay_time: 2.0
      end_decay_time: 4.0
      still_yaw_rate_threshold: 0.1
      motion_model: CONSTANT_VELOCITY_TURN_RATE
    }

    ego_vehicle_id: 0
    enable_evaluation: false
    obstacle_config {
      obstacle_type: VEHICLE
      priority_type: CAUTION
      evaluator_type: EMPTY_EVALUATOR
      predictor_type: FREE_MOVE_PREDICTOR
    }
    obstacle_config {
      obstacle_type: VEHICLE
      priority_type: NORMAL
      evaluator_type: EMPTY_EVALUATOR
      predictor_type: FREE_MOVE_PREDICTOR
    }
    obstacle_config {
      obstacle_type: VEHICLE
      priority_type: IGNORE
      evaluator_type: EMPTY_EVALUATOR
      predictor_type: FREE_MOVE_PREDICTOR
    }
    obstacle_config {
      obstacle_type: VEHICLE
      priority_type: UNKNOWN
      evaluator_type: EMPTY_EVALUATOR
      predictor_type: FREE_MOVE_PREDICTOR
    }
    obstacle_config {
      obstacle_type: PEDESTRIAN
      obstacle_status: MOVING
      evaluator_type: EMPTY_EVALUATOR
      predictor_type: FREE_MOVE_PREDICTOR
    }
    obstacle_config {
      obstacle_type: OBSTACLE_TYPE_UNKNOWN
      obstacle_status: MOVING
      evaluator_type: EMPTY_EVALUATOR
      predictor_type: FREE_MOVE_PREDICTOR
    }
    obstacle_config {
      priority_type: IGNORE
      evaluator_type: EMPTY_EVALUATOR
      predictor_type: FREE_MOVE_PREDICTOR
    }
    obstacle_config {
      obstacle_type: VEHICLE
      evaluator_type: EMPTY_EVALUATOR
      predictor_type: FREE_MOVE_PREDICTOR
    }
    obstacle_config {
      obstacle_type: VEHICLE
      priority_type: IGNORE
      predictor_type: FREE_MOVE_PREDICTOR
    }
    obstacle_config {
      obstacle_type: VEHICLE
      priority_type: IGNORE
      evaluator_type: EMPTY_EVALUATOR
    }
  
  )";
  prediction::PredictionConfig prediction_config;
  google::protobuf::io::ArrayInputStream input(kPredictionConfigText.data(), kPredictionConfigText.size());
  google::protobuf::TextFormat::Parse(&input, &prediction_config);
  ERT_LOG_I(prediction_config.DebugString());
  return prediction_config;
}

}  // namespace planning
}  // namespace pnc
}  // namespace gpal