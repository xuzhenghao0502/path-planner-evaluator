#include <gtest/gtest.h>
#include <any>
#include "config_manager/config_manager.h"
#include "config/speed_planner/speed_preprocessor.pb.h"

#define private public
#include "speed_preprocessor/speed_limit.h"


namespace gpal::pnc::planning {
class SpeedLimitProcessorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    int argc = 0;
    // char path[] = "/work/eka_program/eka/prebuilt/linux-x86_64/bin/policy_planner";
    char path[] = "policy_planner";
    char* argv[]{path};
    constexpr std::chrono::seconds vehicle_parameters_timeout(0);
    auto vehicle_parameters_callback =
        std::make_shared<std::packaged_task<proto::VehicleParameters(std::shared_ptr<proto::VehicleParameters>)>>(
            [](std::shared_ptr<proto::VehicleParameters> ptr_msg) { return *ptr_msg; });
    if (!Singleton<ConfigManager>::get_instance()->init(argc, argv, vehicle_parameters_callback->get_future(),
                                                        vehicle_parameters_timeout)) {
      std::cout << "Failed to init config!";
    }

    // Setup default config
    speed_preprocessor_config.set_enable_turn_speed_limit(true);
    speed_preprocessor_config.set_enable_decision_speed_limit(true);
    processor_.speed_preprocessor_config_ = speed_preprocessor_config;
  }

  void addReferenceLineInfo() {
    // 设置有效的reference line和停止线
    vector<ReferencePoint> ref_points;
    for (int i = 0; i < 50; i++) {
      ref_points.emplace_back(math::Vec3d(i * 1.0, 0.0, 0.0), 0.0, 0.0, 0.0001, 0.0);
      ref_points.back().local_s_ = i * 1.0;
    }
    ReferenceLine ref_line(ref_points);
    ref_line.speed_limits_.emplace_back(0.0, 30.0, 10.0, 0.0, math::Vec3d(), math::Vec3d());

    reference_line_info_.ref_line_ = ref_line;

    reference_line_info_.vehicle_state_.set_x(0.0);
    reference_line_info_.vehicle_state_.set_y(0.0);
  }

  void addPath() {
    // 设置有效的reference line和停止线
    vector<PathPt> path_points;
    for (int i = 0; i < 50; i++) {
      PathPt pt;
      pt.set_x(i * 1.0);
      pt.set_y(0.0);
      pt.set_theta(0.0);
      pt.set_kappa(0.2);
      pt.set_s(i * 1.0);
      path_points.push_back(pt);
    }
    path_ = DiscretizedPath(path_points);
  }

  void initSpeedLimit() {
    processor_.speed_limit_.resize(path_.size());
    for (int i = 0; i < path_.size(); i++) {
      processor_.speed_limit_.at(i).s = path_.at(i).s();
      processor_.speed_limit_.at(i).speed_limit = kMaxSpeedMS;
      processor_.speed_limit_.at(i).id = "default";
    }
  }

  SpeedLimitProcessor processor_;
  LocalView local_view_;
  DecisionResult decision_result_;
  ReferenceLineInfo reference_line_info_;
  DiscretizedPath path_;
  BehaviorState behavior_state_;
  SpeedState speed_state_;
  SpeedLimitResult speed_limit_result_;
  SpeedPreProcessorConfig speed_preprocessor_config;
};

TEST_F(SpeedLimitProcessorTest, MapSpeedLimitEmpty) { 
  processor_.getMapRelatedSpeedlimit(nullptr, path_);

  reference_line_info_.ref_line_.reference_points_.clear();
  processor_.getMapRelatedSpeedlimit(&reference_line_info_, path_);
}

TEST_F(SpeedLimitProcessorTest, MapSpeedLimitNormal) {
  addPath();
  addReferenceLineInfo();
  initSpeedLimit();
  processor_.getMapRelatedSpeedlimit(&reference_line_info_, path_);
  processor_.speed_preprocessor_config_.set_enable_map_speed_limit(false);
  processor_.getMapRelatedSpeedlimit(&reference_line_info_, path_);

}

TEST_F(SpeedLimitProcessorTest, getTurnSpeedLimitEmpty) { 
  processor_.getTurnSpeedLimit(local_view_, nullptr, path_, 0);

  reference_line_info_.ref_line_.reference_points_.clear();
  processor_.getTurnSpeedLimit(local_view_, &reference_line_info_, path_, 0);
}

TEST_F(SpeedLimitProcessorTest, GetTurnSpeedLimitEmptyStopLines) {
  addReferenceLineInfo();
  addPath();
  initSpeedLimit();

  processor_.getTurnSpeedLimit(local_view_, &reference_line_info_, path_, 0);
  processor_.getTurnSpeedLimit(local_view_, &reference_line_info_, path_, 1);
  processor_.getTurnSpeedLimit(local_view_, &reference_line_info_, path_, 2);
  processor_.speed_preprocessor_config_.set_enable_turn_accelerate_limit(true);
  processor_.getTurnSpeedLimit(local_view_, &reference_line_info_, path_, 1);
}

TEST_F(SpeedLimitProcessorTest, GetTurnSpeedLimitLeftTurnStopLine) {
  addReferenceLineInfo();
  addPath();
  initSpeedLimit();
  
  // 添加左转停止线 (s=20m)
  StopLine stop_line;
  stop_line.s = 20.0;
  stop_line.direction = DrivingDirection::kDirectionLeftOnly;
  reference_line_info_.ref_line_.stop_lines_.push_back(stop_line);
  processor_.getTurnSpeedLimit(local_view_, &reference_line_info_, path_, 1);
}

TEST_F(SpeedLimitProcessorTest, GetTurnSpeedLimitRightTurnStopLine) {
  addReferenceLineInfo();
  addPath();
  initSpeedLimit();
  
  // 添加右转停止线 (s=30m)
  StopLine stop_line;
  stop_line.s = 30.0;
  stop_line.direction = DrivingDirection::kDirectionRightOnly;
  reference_line_info_.ref_line_.stop_lines_.push_back(stop_line);
  
  processor_.getTurnSpeedLimit(local_view_, &reference_line_info_, path_, 1);
}

TEST_F(SpeedLimitProcessorTest, GetTurnSpeedLimitUturnStopLine) {
  addReferenceLineInfo();
  addPath();
  initSpeedLimit();
  
  // 添加掉头停止线 (s=15m)
  StopLine stop_line;
  stop_line.s = 15.0;
  stop_line.direction = DrivingDirection::kDirectionUTurnOnly;
  reference_line_info_.ref_line_.stop_lines_.push_back(stop_line);
  processor_.getTurnSpeedLimit(local_view_, &reference_line_info_, path_, 1);
  processor_.getTurnSpeedLimit(local_view_, &reference_line_info_, path_, 2);
  stop_line.direction = DrivingDirection::kDirectionInvalid;
  processor_.getTurnSpeedLimit(local_view_, &reference_line_info_, path_, 1);
}

TEST_F(SpeedLimitProcessorTest, DecisionSpeedLimitInvalidReferenceLine) {
  DecisionResult decision_result;
  
  // 空指针测试
  processor_.getDecisionSpeedLimit(local_view_, nullptr, decision_result);
  
  // 无效参考线测试
  ReferenceLineInfo invalid_ref;
  processor_.getDecisionSpeedLimit(local_view_, &invalid_ref, decision_result);
}

TEST_F(SpeedLimitProcessorTest, DecisionSpeedLimitValidDecelWall) {
  addReferenceLineInfo();
  addPath();
  initSpeedLimit();
  
  // 设置车辆状态
  local_view_.chassis_->set_Speed(10.0);  // 10m/s
  // 添加减速墙决策 (s=30m, 限速5m/s)
  WallConstraint boundary;
  boundary.type = WallType::LONG_DECC_WALL;
  boundary.s = 30.0;
  boundary.v = 5.0;
  decision_result_.getMutableLongitudinalBoundaryDecision()->push_back(boundary);
  
  processor_.getDecisionSpeedLimit(local_view_, &reference_line_info_, decision_result_);
}
TEST_F(SpeedLimitProcessorTest, DecisionSpeedLimitInsufficientDecel) {
  addReferenceLineInfo();
  addPath();
  initSpeedLimit();
  
  // 设置车辆状态
  local_view_.getMutableChassisPtr()->set_Speed(2.0);  // 低速
  
  // 添加减速墙决策 (s=30m, 限速1m/s)
  WallConstraint boundary;
  boundary.type = WallType::LONG_DECC_WALL;
  boundary.s = 30.0;
  boundary.v = 1.0;
  decision_result_.getMutableLongitudinalBoundaryDecision()->push_back(boundary);
  
  processor_.getDecisionSpeedLimit(local_view_, &reference_line_info_, decision_result_);
}

TEST_F(SpeedLimitProcessorTest, DecisionSpeedLimitBehindVehicle) {
  addReferenceLineInfo();
  addPath();
  initSpeedLimit();
  
  // 设置车辆状态 (已过减速墙)
  local_view_.loc_->vehicle_align_pose_point_.set_x(40.0);
  local_view_.getMutableChassisPtr()->set_Speed(5.0);
  
  // 添加减速墙 (s=30m, 在车辆后方)
  WallConstraint boundary;
  boundary.type = WallType::LONG_DECC_WALL;
  boundary.s = 30.0;
  boundary.v = 3.0;
  decision_result_.getMutableLongitudinalBoundaryDecision()->push_back(boundary);
  
  processor_.getDecisionSpeedLimit(local_view_, &reference_line_info_, decision_result_);
}

TEST_F(SpeedLimitProcessorTest, DecisionSpeedLimitZeroSpeedLimit) {
  addReferenceLineInfo();
  addPath();
  initSpeedLimit();
  
  // 设置车辆状态
  local_view_.getMutableChassisPtr()->set_Speed(10.0);
  
  // 添加减速墙 (限速0m/s)
  WallConstraint boundary;
  boundary.type = WallType::LONG_DECC_WALL;
  boundary.s = 30.0;
  boundary.v = 0.0;
  decision_result_.getMutableLongitudinalBoundaryDecision()->push_back(boundary);
  
  processor_.getDecisionSpeedLimit(local_view_, &reference_line_info_, decision_result_);
}
TEST_F(SpeedLimitProcessorTest, DecisionSpeedLimitNonDecelWall) {
  addReferenceLineInfo();
  addPath();
  initSpeedLimit();
  
  // 添加非减速墙类型
  WallConstraint boundary;
  boundary.type = WallType::LONG_CLEAR_ZONE;  
  boundary.s = 30.0;
  boundary.v = 5.0;
  decision_result_.getMutableLongitudinalBoundaryDecision()->push_back(boundary);
  
  processor_.getDecisionSpeedLimit(local_view_, &reference_line_info_, decision_result_);
}

TEST_F(SpeedLimitProcessorTest, CurveSpeedLimitBase) {
  // Setup path with 50 points
  std::vector<PathPt> path_points;
  for (int i = 0; i < 50; i++) {
    PathPt pt;
    pt.set_x(i * 1.0);
    pt.set_y(0.0);
    pt.set_theta(0.0);
    pt.set_kappa(0.001);
    pt.set_s(i * 1.0);
    path_points.push_back(pt);
  }
  path_points.front().set_kappa(0.0);
  path_points.back().set_kappa(NAN);
  DiscretizedPath path = DiscretizedPath(path_points);

  // Test with default parameters
  processor_.getCurveSpeedLimit(local_view_, decision_result_, path);

  // Verify basic results
  // EXPECT_EQ(speed_limit_result_.speed_limit_.size(), 50);
}

TEST_F(SpeedLimitProcessorTest, CurveSpeedLimitBigCurve) {
  // Setup path with 50 points
  std::vector<PathPt> path_points;
  for (int i = 0; i < 20; i++) {
    PathPt pt;
    pt.set_x(i * 1.0);
    pt.set_y(0.0);
    pt.set_theta(0.0);
    pt.set_kappa(0.2);
    pt.set_s(i * 1.0);
    path_points.push_back(pt);
  }
  for (int i = 0; i < 30; i++) {
    PathPt pt;
    pt.set_x(i * 1.0);
    pt.set_y(0.0);
    pt.set_theta(0.0);
    pt.set_kappa(0.001);
    pt.set_s(i * 1.0);
    path_points.push_back(pt);
  }
  DiscretizedPath path = DiscretizedPath(path_points);

  // Test with default parameters
  processor_.getCurveSpeedLimit(local_view_, decision_result_, path);

  // Verify basic results
  // EXPECT_EQ(speed_limit_result_.speed_limit_.size(), 50);
}

TEST_F(SpeedLimitProcessorTest, getConstSpeedLimitNormal) {
  local_view_.chassis_->set_Speed(2);

  processor_.max_speed_limit_ = 100;
  
  processor_.speed_planner_config_.set_max_speed_limit(150);
  local_view_.console_->speed_limit_ = 30;
  processor_.getConstSpeedLimit(local_view_, 0, behavior_state_);


  local_view_.console_->speed_limit_ = 20;
  local_view_.console_->acc_desired_speed_ = 19;
  behavior_state_.is_acc_state_ = true;
  processor_.getConstSpeedLimit(local_view_, 0, behavior_state_);


  local_view_.console_->speed_limit_ = 18;
  local_view_.console_->acc_desired_speed_ = 17;
  behavior_state_.is_acc_state_ = false;
  behavior_state_.is_lcc_state_= true;
  processor_.getConstSpeedLimit(local_view_, 1, behavior_state_);


  behavior_state_.search_parklot_state_ = true;
  processor_.getConstSpeedLimit(local_view_, 2, behavior_state_);
  
  behavior_state_.park_in_state_ = true;
  behavior_state_.search_parklot_state_ = false;
  processor_.getConstSpeedLimit(local_view_, 2, behavior_state_);

}

TEST_F(SpeedLimitProcessorTest, getConstSpeedLimitNormal2) {
  local_view_.chassis_->set_Speed(2);
  processor_.max_speed_limit_ = 100;
  processor_.speed_planner_config_.set_max_speed_limit(150);
  processor_.getConstSpeedLimit(local_view_, 2, behavior_state_);
}

TEST_F(SpeedLimitProcessorTest, updateSpeedLimit) {
  addPath();
  initSpeedLimit();
  processor_.updateSpeedLimit(20.0,10,"map");
}



TEST_F(SpeedLimitProcessorTest, steeringAngleAndYawRateFiliterNormal) {
  processor_.history_steering_angle_.push_back(10);
  processor_.history_yaw_rate_.push_back(0.1);
  double fliter_steering_angle;
  double fliter_yaw_rate;
  double steering_angle_change_rate;
  double yaw_rate_change_rate;
  processor_.steeringAngleAndYawRateFiliter(10.1,0.2,fliter_steering_angle,fliter_yaw_rate,steering_angle_change_rate,yaw_rate_change_rate  );

  processor_.history_yaw_rate_.clear();
  processor_.steeringAngleAndYawRateFiliter(10.1,0.2,fliter_steering_angle,fliter_yaw_rate,steering_angle_change_rate,yaw_rate_change_rate  );

  processor_.history_steering_angle_.clear();
  processor_.history_steering_angle_.resize(5,10.0);
  processor_.history_yaw_rate_.resize(5,0.1);
  processor_.steeringAngleAndYawRateFiliter(10.1,0.2,fliter_steering_angle,fliter_yaw_rate,steering_angle_change_rate,yaw_rate_change_rate  );
}


TEST_F(SpeedLimitProcessorTest, ewmaFilter) {
  processor_.ewmaFilter(1,2,0.5);
}


TEST_F(SpeedLimitProcessorTest, SingleElement) {
  std::vector<double> single = {5.0};
  EXPECT_DOUBLE_EQ(5.0, processor_.movingAverageFilter(single));
}

TEST_F(SpeedLimitProcessorTest, MultipleElements) {
  std::vector<double> multiple = {1.0, 2.0, 3.0, 4.0, 5.0};
  EXPECT_DOUBLE_EQ(3.0, processor_.movingAverageFilter(multiple));
}

TEST_F(SpeedLimitProcessorTest, NegativeNumbers) {
  std::vector<double> negatives = {-1.0, -2.0, -3.0};
  EXPECT_DOUBLE_EQ(-2.0, processor_.movingAverageFilter(negatives));
}

TEST_F(SpeedLimitProcessorTest, MixedNumbers) {
  std::vector<double> mixed = {-2.0, 0.0, 2.0};
  EXPECT_DOUBLE_EQ(0.0, processor_.movingAverageFilter(mixed));
}

TEST_F(SpeedLimitProcessorTest, LargeNumbers) {
  std::vector<double> large = {1e20, 2e20, 3e20};
  EXPECT_DOUBLE_EQ(2e20, processor_.movingAverageFilter(large));
}

TEST_F(SpeedLimitProcessorTest, PrecisionTest) {
  std::vector<double> precise = {0.1, 0.1, 0.1};
  EXPECT_DOUBLE_EQ(0.1, processor_.movingAverageFilter(precise));
}

TEST_F(SpeedLimitProcessorTest, CalculatePathVTParkingState) {
  addReferenceLineInfo();
  addPath();
  initSpeedLimit();
    processor_.getMapRelatedSpeedlimit(&reference_line_info_, path_);
  processor_.getCurveSpeedLimit(local_view_, decision_result_, path_);
  
  // 设置泊车状态
  behavior_state_.park_in_state_ = true;
  
  // 设置初始速度高于限速
  processor_.init_speed_state_.v = 20.0;
  for (auto& limit : processor_.speed_limit_) {
    limit.speed_limit = 10.0;
  }
  
  processor_.calculatePathVT(local_view_, behavior_state_, &reference_line_info_);
}
TEST_F(SpeedLimitProcessorTest, CalculatePathVTHighInitialSpeed) {
  addReferenceLineInfo();
  addPath();
  initSpeedLimit();
    processor_.getMapRelatedSpeedlimit(&reference_line_info_, path_);
  processor_.getCurveSpeedLimit(local_view_, decision_result_, path_);
  
  // 设置初始速度高于限速
  processor_.init_speed_state_.v = 15.0;
  for (auto& limit : processor_.speed_limit_) {
    limit.speed_limit = 10.0;
    limit.id = "driving";
  }
  
  processor_.calculatePathVT(local_view_, behavior_state_, &reference_line_info_);
}
TEST_F(SpeedLimitProcessorTest, CalculatePathVTLowInitialSpeed) {
  addReferenceLineInfo();
  addPath();
  initSpeedLimit();
    processor_.getMapRelatedSpeedlimit(&reference_line_info_, path_);
  processor_.getCurveSpeedLimit(local_view_, decision_result_, path_);

  
  // 设置初始速度低于限速
  processor_.init_speed_state_.v = 5.0;
  for (auto& limit : processor_.speed_limit_) {
    limit.speed_limit = 15.0;
    limit.id = "map";
  }
  processor_.calculatePathVT(local_view_, behavior_state_, &reference_line_info_);
}
TEST_F(SpeedLimitProcessorTest, CalculatePathVTCurvatureSpeedDifference) {
  addReferenceLineInfo();
  addPath();
  initSpeedLimit();
    processor_.getMapRelatedSpeedlimit(&reference_line_info_, path_);
  processor_.getCurveSpeedLimit(local_view_, decision_result_, path_);
  
  
  // 设置曲率速度路径低于当前限速
  processor_.curvature_speed_path_.resize(processor_.speed_limit_.size());
  for (size_t i = 0; i < processor_.curvature_speed_path_.size(); ++i) {
    processor_.curvature_speed_path_[i].set_v(8.0);
  }
  
  processor_.calculatePathVT(local_view_, behavior_state_, &reference_line_info_);
}

TEST_F(SpeedLimitProcessorTest, CalculatePathVTOutCurveAccLimit) {
  addReferenceLineInfo();
  addPath();
  initSpeedLimit();
    processor_.getMapRelatedSpeedlimit(&reference_line_info_, path_);
  processor_.getCurveSpeedLimit(local_view_, decision_result_, path_);
  
  // 设置大曲率区域 (触发出弯加速度限制)
  reference_line_info_.ref_line_.reference_points_[10].kappa_ = 0.5;
  
  // 设置方向盘转角超过阈值
  local_view_.chassis_->set_SteeringAngle(30.0);
  
  processor_.calculatePathVT(local_view_, behavior_state_, &reference_line_info_);
}

TEST_F(SpeedLimitProcessorTest, CalculatePathVTLargeDistance) {
  addReferenceLineInfo();
  addPath();
  initSpeedLimit();
  processor_.getMapRelatedSpeedlimit(&reference_line_info_, path_);
  processor_.getCurveSpeedLimit(local_view_, decision_result_, path_);
  
  // 设置路径点距离超过150米
  for (auto& limit : processor_.speed_limit_) {
    limit.s = limit.s * 10.0;  // 放大距离
  }
  
  processor_.calculatePathVT(local_view_, behavior_state_, &reference_line_info_);

  processor_.speed_preprocessor_config_.set_out_curve_consider_curve_threshold(0.0);
  local_view_.chassis_->set_SteeringAngle(50.0);
  processor_.calculatePathVT(local_view_, behavior_state_, &reference_line_info_);
    
}

}  // namespace gpal::pnc::planning