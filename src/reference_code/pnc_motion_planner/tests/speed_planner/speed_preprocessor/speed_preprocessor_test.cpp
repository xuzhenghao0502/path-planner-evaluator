#include <gtest/gtest.h>
#include <any>

#include "config_manager/config_manager.h"
#include "config/speed_planner/speed_preprocessor.pb.h"

#define private public
#include "speed_preprocessor/speed_preprocessor.h"


namespace gpal::pnc::planning {

class SpeedPreprocessorTest : public ::testing::Test {
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
    processor_.speed_planner_config_.set_enable_parking_od_check(true);
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
    path_data_.discretized_path_ = DiscretizedPath(path_points);
    path_data_.local_path_ = DiscretizedPath(path_points);
  }

  void addObstacle() {
    Obstacle obs("1001", math::Polygon2d(math::Box2d(math::Vec2d(20.0, 0), 0.0, 5.0, 2.0)));
    local_view_.getMutableIndexedObstaclesPtr()->add(obs);
  }

  void addDecision() {
    std::vector<Decision::RawSinglePrediction> raw_prediction_in;
    Decision::DecisionObject decision_obj("1001", 5.0, 2.0, 20.0, 0.0, 0.0, 5.0, false, raw_prediction_in);
    decision_obj.long_od_tag = LongitudinalOdTag::IGNORE;
    decision_result_.getMutableOdDecisions()->emplace("1001", decision_obj);
  }

  const std::vector<double> time_grid_ = {0.0, 0.1, 0.2, 0.4, 0.6, 1.0, 1.4, 1.8,
                                          2.2, 2.6, 3.0, 3.4, 3.8, 4.2, 4.6, 5.0};
  const double time_resolution_{0.1};
  const double time_horizon_{5.0};
  SpeedPreprocessor processor_ = SpeedPreprocessor(time_grid_, time_resolution_, time_horizon_);
  std::shared_ptr<SpeedResult> speed_result_;
  LocalView local_view_;
  ReferenceLineInfo reference_line_info_;
  PathData path_data_;
  DecisionResult decision_result_;
  SpeedState init_speed_state_;
  SpeedPreProcessorConfig speed_preprocessor_config;
};

TEST_F(SpeedPreprocessorTest, NoaNormalCase) {
  // Setup normal case inputs
  // Assume these are properly initialized with typical values
  addReferenceLineInfo();
  addObstacle();
  addDecision();
  addPath();
  speed_result_ = make_shared<SpeedResult>();
  processor_.NoaProcess(local_view_, &reference_line_info_, path_data_, decision_result_, init_speed_state_,
                        speed_result_);
  // Add assertions to verify speed_result_ contains expected values
}

TEST_F(SpeedPreprocessorTest, AccNormalCase) {
  // Setup normal case inputs
  // Assume these are properly initialized with typical values
  addReferenceLineInfo();
  addObstacle();
  addDecision();
  addPath();
  speed_result_ = make_shared<SpeedResult>();
  processor_.AccProcess(local_view_, path_data_, decision_result_, init_speed_state_, speed_result_);
  // Add assertions to verify speed_result_ contains expected values
}

TEST_F(SpeedPreprocessorTest, ParkNormalCase) {
  // Setup normal case inputs
  // Assume these are properly initialized with typical values
  addReferenceLineInfo();
  addObstacle();
  addDecision();
  addPath();
  speed_result_ = make_shared<SpeedResult>();
  processor_.ParkProcess(local_view_, path_data_, decision_result_, init_speed_state_, speed_result_);
  // Add assertions to verify speed_result_ contains expected values
}

TEST_F(SpeedPreprocessorTest, caculateTurnFlag) {
  processor_.caculateTurnFlag(local_view_, nullptr);
  vector<ReferencePoint> ref_points;
  ReferenceLine ref_line(ref_points);
  reference_line_info_.ref_line_ = ref_line;
  processor_.caculateTurnFlag(local_view_, &reference_line_info_);
}

TEST_F(SpeedPreprocessorTest, CaculateTurnFlagInvalidReferenceLine) {
  // 测试无效参考线
  processor_.caculateTurnFlag(local_view_, nullptr);
  // 验证状态未变化
  EXPECT_EQ(processor_.last_turn_state_, 0);
  EXPECT_FALSE(processor_.is_turn_around_);
}

TEST_F(SpeedPreprocessorTest, CaculateTurnFlagNoValidRange) {
  // 创建空参考线
  vector<ReferencePoint> ref_points;
  ReferenceLine ref_line(ref_points);
  reference_line_info_.ref_line_ = ref_line;
  
  // 设置定位点使索引无效
  local_view_.loc_->vehicle_align_pose_point_.set_x(100.0);
  local_view_.loc_->vehicle_align_pose_point_.set_y(100.0);
  
  processor_.caculateTurnFlag(local_view_, &reference_line_info_);
  // 验证状态未变化
  EXPECT_EQ(processor_.last_turn_state_, 0);
  EXPECT_FALSE(processor_.is_turn_around_);
}

TEST_F(SpeedPreprocessorTest, CaculateTurnFlagState0ToTurnAround) {
  addReferenceLineInfo();
  // 构造U型路径 (角度变化180°)
  for (size_t i = 0; i < reference_line_info_.ref_line_.reference_points_.size(); ++i) {
    if (i > 10 && i < 40) {
      reference_line_info_.ref_line_.reference_points_[i].heading_ = M_PI; // 180度转向
    }
  }
  
  processor_.last_turn_state_ = 0;
  processor_.caculateTurnFlag(local_view_, &reference_line_info_);
  
  // 验证进入掉头状态
  EXPECT_TRUE(processor_.is_turn_around_);
  EXPECT_EQ(processor_.last_turn_state_, 2);
}

TEST_F(SpeedPreprocessorTest, CaculateTurnFlagState0ToTurn) {
  addReferenceLineInfo();
  // 构造90°转弯路径
  for (size_t i = 0; i < reference_line_info_.ref_line_.reference_points_.size(); ++i) {
    if (i > 10 && i < 40) {
      reference_line_info_.ref_line_.reference_points_[i].heading_ = M_PI_2; // 90度转向
    }
  }
  
  processor_.last_turn_state_ = 0;
  processor_.caculateTurnFlag(local_view_, &reference_line_info_);
  
  // 验证进入转弯状态
  EXPECT_EQ(processor_.last_turn_state_, 1);
  EXPECT_FALSE(processor_.is_turn_around_);
}

TEST_F(SpeedPreprocessorTest, CaculateTurnFlagState0NoTurn) {
  addReferenceLineInfo(); // 直线路径
  processor_.last_turn_state_ = 0;
  processor_.caculateTurnFlag(local_view_, &reference_line_info_);
  
  // 验证保持初始状态
  EXPECT_EQ(processor_.last_turn_state_, 0);
  EXPECT_FALSE(processor_.is_turn_around_);
}

TEST_F(SpeedPreprocessorTest, CaculateTurnFlagState1ToTurnAround) {
  addReferenceLineInfo();
  // 构造U型路径
  for (size_t i = 0; i < reference_line_info_.ref_line_.reference_points_.size(); ++i) {
    if (i > 10 && i < 40) {
      reference_line_info_.ref_line_.reference_points_[i].heading_ = M_PI; // 180度转向
    }
  }
  
  processor_.last_turn_state_ = 1; // 初始为转弯状态
  processor_.caculateTurnFlag(local_view_, &reference_line_info_);
  
  // 验证进入掉头状态
  EXPECT_TRUE(processor_.is_turn_around_);
  EXPECT_EQ(processor_.last_turn_state_, 2);
}

TEST_F(SpeedPreprocessorTest, CaculateTurnFlagState1ToStraight) {
  addReferenceLineInfo(); // 直线路径
  processor_.last_turn_state_ = 1; // 初始为转弯状态
  processor_.caculateTurnFlag(local_view_, &reference_line_info_);
  
  // 验证回到直行状态
  EXPECT_EQ(processor_.last_turn_state_, 0);
  EXPECT_FALSE(processor_.is_turn_around_);
}

TEST_F(SpeedPreprocessorTest, CaculateTurnFlagState1Keep) {
  addReferenceLineInfo();
  // 构造45°转弯路径 (cos(45°)=0.7 > 0.34)
  for (size_t i = 0; i < reference_line_info_.ref_line_.reference_points_.size(); ++i) {
    if (i > 10 && i < 40) {
      reference_line_info_.ref_line_.reference_points_[i].heading_ = M_PI/4; // 45度转向
    }
  }
  
  processor_.last_turn_state_ = 1;
  processor_.caculateTurnFlag(local_view_, &reference_line_info_);
  
  // 验证保持转弯状态
  EXPECT_EQ(processor_.last_turn_state_, 1);
  EXPECT_FALSE(processor_.is_turn_around_);
}

TEST_F(SpeedPreprocessorTest, CaculateTurnFlagState2ToStraight) {
  addReferenceLineInfo(); // 直线路径
  processor_.last_turn_state_ = 2; // 初始为掉头状态
  processor_.is_turn_around_ = true;
  processor_.caculateTurnFlag(local_view_, &reference_line_info_);
  
  // 验证回到直行状态
  EXPECT_EQ(processor_.last_turn_state_, 0);
  EXPECT_FALSE(processor_.is_turn_around_);
}

TEST_F(SpeedPreprocessorTest, CaculateTurnFlagState2Keep) {
  addReferenceLineInfo();
  // 构造135°转弯路径 (未达到直行条件)

  for (size_t i = 0; i < reference_line_info_.ref_line_.reference_points_.size(); ++i) {
    if (i > 10 && i < 40) {
      reference_line_info_.ref_line_.reference_points_[i].heading_ = 3*M_PI/4; // 135度转向
    }
  }
  
  processor_.last_turn_state_ = 2;
  processor_.is_turn_around_ = true;
  processor_.caculateTurnFlag(local_view_, &reference_line_info_);
  
  // 验证保持掉头状态
  EXPECT_EQ(processor_.last_turn_state_, 2);
  EXPECT_TRUE(processor_.is_turn_around_);
}

}  // namespace gpal::pnc::planning