#include "bicycle_trajectory_tracker.h"

namespace gpal::pnc::planning {

BicycleTrajectoryTracker::BicycleTrajectoryTracker()
    : OptimalControlProblem("BicycleTrajectoryTracker", 5, 2, 0, 19, 0) {
  default_state_.emplace("x");
  default_state_.emplace("y");
  default_state_.emplace("theta");
  default_state_.emplace("v");
  default_state_.emplace("kappa");

  default_ctrl_.emplace("a");
  default_ctrl_.emplace("dkappa");

  default_param_.emplace("xr", 0.0);
  default_param_.emplace("yr", 0.0);
  default_param_.emplace("thetar", 0.0);
  default_param_.emplace("vr", 0.0);
  default_param_.emplace("kr", 0.0);
  default_param_.emplace("weight_position", 0.01);
  default_param_.emplace("weight_heading", 0.1);
  default_param_.emplace("weight_v", 0.001);
  default_param_.emplace("weight_kappa", 1.0);
  default_param_.emplace("weight_a", 0.01);
  default_param_.emplace("weight_dkappa", 10.0);
  default_param_.emplace("v_lower", 0.0);
  default_param_.emplace("v_upper", 40);
  default_param_.emplace("a_lower", -10.0);
  default_param_.emplace("a_upper", 5.0);
  default_param_.emplace("kappa_lower", -0.1);
  default_param_.emplace("kappa_upper", 0.1);
  default_param_.emplace("dkappa_lower", -0.1);
  default_param_.emplace("dkappa_upper", 0.1);
}

std::shared_ptr<Dynamics> BicycleTrajectoryTracker::createDynamics(OcpConfig::IntegratorType type) const {
  return std::make_shared<BicycleTrajectoryTrackerDynamics>(type);
}

std::shared_ptr<CostFunction> BicycleTrajectoryTracker::createCostFunction() const {
  return std::make_shared<BicycleTrajectoryTrackerCost<StageType::NORMINAL>>();
}

std::shared_ptr<Constraints> BicycleTrajectoryTracker::createConstraint() const {
  return std::make_shared<BicycleTrajectoryTrackerConstraints<StageType::NORMINAL>>();
}

std::shared_ptr<Constraints> BicycleTrajectoryTracker::createInitialConstraint() const {
  return std::make_shared<BicycleTrajectoryTrackerConstraints<StageType::INITIAL>>();
}

std::shared_ptr<Constraints> BicycleTrajectoryTracker::createTerminalConstraint() const {
  return std::make_shared<BicycleTrajectoryTrackerConstraints<StageType::TERMINAL>>();
}

std::shared_ptr<Constraints> BicycleTrajectoryTracker::createStateOnlyEqualities() const {
  return std::make_shared<BicycleTrajectoryTrackerStateOnlyEqualities<StageType::NORMINAL>>();
}

std::shared_ptr<Constraints> BicycleTrajectoryTracker::createInitialStateOnlyEqualities() const {
  return std::make_shared<BicycleTrajectoryTrackerStateOnlyEqualities<StageType::INITIAL>>();
}

std::shared_ptr<Constraints> BicycleTrajectoryTracker::createTerminalEqualities() const {
  return std::make_shared<BicycleTrajectoryTrackerStateOnlyEqualities<StageType::TERMINAL>>();
}

std::shared_ptr<IPMHelper> BicycleTrajectoryTracker::createBaseIpmHelper() const {
  IPMHelper::Creater creater{
      {{StageType::NORMINAL, OcpConfig::FORWARD_EULER},
       []() {
         return std::make_shared<BicycleTrajectoryTrackerIpmEvaluator<StageType::NORMINAL, OcpConfig::FORWARD_EULER>>();
       }},
      {{StageType::NORMINAL, OcpConfig::ERK4},
       []() { return std::make_shared<BicycleTrajectoryTrackerIpmEvaluator<StageType::NORMINAL, OcpConfig::ERK4>>(); }},
      {{StageType::INITIAL, OcpConfig::FORWARD_EULER},
       []() {
         return std::make_shared<BicycleTrajectoryTrackerIpmEvaluator<StageType::INITIAL, OcpConfig::FORWARD_EULER>>();
       }},
      {{StageType::INITIAL, OcpConfig::ERK4},
       []() { return std::make_shared<BicycleTrajectoryTrackerIpmEvaluator<StageType::INITIAL, OcpConfig::ERK4>>(); }},
      {{StageType::TERMINAL, OcpConfig::FORWARD_EULER},
       []() { return std::make_shared<BicycleTrajectoryTrackerIpmEvaluatorTerminal>(); }},
      {{StageType::TERMINAL, OcpConfig::ERK4},
       []() { return std::make_shared<BicycleTrajectoryTrackerIpmEvaluatorTerminal>(); }}};
  return std::make_shared<IPMHelper>(std::move(creater));
}

REGIST_MODEL(BicycleTrajectoryTracker)
}  // namespace gpal::pnc::planning