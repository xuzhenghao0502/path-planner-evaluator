#include <gtest/gtest.h>
#include <any>
#include "config_manager/config_manager.h"
#include "config/speed_planner/st_graph_processor.pb.h"

#define private public
#include "speed_optimizer/speed_ocp_qp_optimizer.h"


namespace gpal::pnc::planning {

class SpeedOcpQpOptimizerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    int argc = 0;
    // char path[] = "/work/eka_program/eka/prebuilt/linux-x86_64/bin/policy_planner";
    char path[] = "policy_planner";
    char* argv[]{path};
    // constexpr std::chrono::seconds vehicle_parameters_timeout(0);
    // auto vehicle_parameters_callback =
    //     std::make_shared<std::packaged_task<proto::VehicleParameters(std::shared_ptr<proto::VehicleParameters>)>>(
    //         [](std::shared_ptr<proto::VehicleParameters> ptr_msg) { return *ptr_msg; });
    if (!Singleton<ConfigManager>::get_instance()->init(argc, argv)) {
      std::cout << "Failed to init config!";
    }

    processor_.speed_ocp_qp_optimizer_config_.set_enable_speed_risk_model(false);

    // Setup default config
  }

  void addStSpeedLimit() {
    std::vector<SpatialSpeedLimit> speed_limits;
    for (int i = 0; i < 100; i++) {
      speed_limits.emplace_back(i, 20.0, "default");
    }
    double s_interval = 1.0;
    for (int i = 0; i < time_grid_.size(); i++) {
      math::IntervalData<SpatialSpeedLimit> speed_limit(
          speed_limits.front().s, s_interval, speed_limits,
          [](const SpatialSpeedLimit& p0, const SpatialSpeedLimit& p1, const double x) {
            return SpatialSpeedLimit(x, p0.speed_limit + (p1.speed_limit - p0.speed_limit) * (x - p0.s) / (p1.s - p0.s),
                                     p0.id);
          });
      speed_limit.setLeftExtrapolationFunction(
          [](const SpatialSpeedLimit& p0, const double x) { return SpatialSpeedLimit(x, p0.speed_limit, p0.id); });
      speed_limit.setRightExtrapolationFunction(
          [](const SpatialSpeedLimit& p0, const double x) { return SpatialSpeedLimit(x, p0.speed_limit, p0.id); });
      speed_limit.setDifferentiationFunction(
          [](const SpatialSpeedLimit& p0, const SpatialSpeedLimit& p1, const double s) {
            return SpatialSpeedLimit(s, (p1.speed_limit - p0.speed_limit) / (p1.s - p0.s), p0.id);
          });

      speed_model_param_.st_speed_limit_.push_back(speed_limit);
    }
  }

  const std::vector<double> time_grid_ = {0.0, 0.1, 0.2, 0.4, 0.6, 1.0, 1.4, 1.8,
                                          2.2, 2.6, 3.0, 3.4, 3.8, 4.2, 4.6, 5.0};
  const double time_resolution_{0.1};
  const double time_horizon_{5.0};
  SpeedOCPQPOptimizer processor_ = SpeedOCPQPOptimizer(time_grid_, time_resolution_, time_horizon_);

  SpeedModelParam speed_model_param_ = SpeedModelParam(time_grid_, time_resolution_, time_horizon_);
  SpeedState speed_init_state_;
};

TEST_F(SpeedOcpQpOptimizerTest, runOptimizerNormal) {
  speed_model_param_.init();
  addStSpeedLimit();
  processor_.runOptimizer(speed_model_param_, speed_init_state_);

  processor_.getOcpSolvedResult();
  processor_.getRiskOcpSolvedResult();
}

TEST_F(SpeedOcpQpOptimizerTest, runOptimizerNoRisk) {
  speed_model_param_.init();
  addStSpeedLimit();
  processor_.speed_ocp_qp_optimizer_config_.set_enable_speed_risk_model(false);
  processor_.runOptimizer(speed_model_param_, speed_init_state_);

  processor_.getOcpSolvedResult();
  processor_.getRiskOcpSolvedResult();
}

TEST_F(SpeedOcpQpOptimizerTest, runOptimizerResolve) {
  speed_model_param_.init();
  addStSpeedLimit();
  
  for(int i = 0; i < time_grid_.size(); i++ ){
    speed_model_param_.setParameter("SHardUpperBound", i, 1.0);
  }
  speed_init_state_.v = 20.0;

  processor_.runOptimizer(speed_model_param_, speed_init_state_);
  processor_.getOcpSolvedResult();
  processor_.getRiskOcpSolvedResult();
}

TEST_F(SpeedOcpQpOptimizerTest, runOptimizerFail) {
  speed_model_param_.init();
  addStSpeedLimit();
  
  for(int i = 0; i < time_grid_.size(); i++ ){
    speed_model_param_.setParameter("SHardUpperBound", i, 1.0);
    speed_model_param_.setParameter("SHardLowerBound", i, 30.0);
    speed_model_param_.setParameter("VHardLowerBound", i, 30.0);
  }
  speed_init_state_.v = 20.0;

  processor_.runOptimizer(speed_model_param_, speed_init_state_);
  processor_.getOcpSolvedResult();
  processor_.getRiskOcpSolvedResult();
}

TEST_F(SpeedOcpQpOptimizerTest, getOcpSolvedResultNull){
  processor_.getOcpSolvedResult();
  processor_.getRiskOcpSolvedResult();
}


}  // namespace gpal::pnc::planning