#include "tracker_model.h"

namespace gpal::pnc::planning {

TrackerModel::TrackerModel() : OptimalControlProblem("TrackerModel", 5, 2, 1, 28, 3) {
  default_state_.emplace("s");
  default_state_.emplace("x");
  default_state_.emplace("y");
  default_state_.emplace("theta");
  default_state_.emplace("steer");

  default_ctrl_.emplace("dsteer");
  default_ctrl_.emplace("l_slack");

  default_param_.emplace("x_ref", 0.0);
  default_param_.emplace("y_ref", 0.0);
  default_param_.emplace("theta_ref", 0.0);
  default_param_.emplace("kappa_ref", 0.0);
  default_param_.emplace("v_ref", 0.0);
  default_param_.emplace("l_terminal", 0.0);
  default_param_.emplace("wheelbase", 3.0);
  default_param_.emplace("front_overhang", 1.0);
  default_param_.emplace("rear_overhang", 1.0);
  default_param_.emplace("length", 5.0);
  default_param_.emplace("width", 2.0);
  default_param_.emplace("l_ref_weight", 1.0);
  default_param_.emplace("theta_ref_weight", 1.0);
  default_param_.emplace("steer_weight", 1.0);
  default_param_.emplace("dsteer_weight", 1.0);
  default_param_.emplace("l_slack_weight", 1.0);
  default_param_.emplace("terminal_l_weight", 1.0);
  default_param_.emplace("terminal_theta_weight", 1.0);
  default_param_.emplace("LHardLowerBound", -20.0);
  default_param_.emplace("LHardUpperBound", 20.0);
  default_param_.emplace("SteerLowerBound", -0.436);
  default_param_.emplace("SteerUpperBound", 0.436);
  default_param_.emplace("DSteerLowerBound", -0.436);
  default_param_.emplace("DSteerUpperBound", 0.436);
  default_param_.emplace("l_scale", 1.0);
  default_param_.emplace("theta_scale", 1.0);
  default_param_.emplace("steer_scale", 1.0);
  default_param_.emplace("dsteer_scale", 1.0);

  default_global_.emplace("kappa");
  default_global_.emplace("l");
  default_global_.emplace("ds");
  default_global_.setUpdater(std::make_shared<OcpVariable::Updater>([](const OcpStage& stage, OcpVariable* ptr_global) {
    // Determine stage variables
    const auto& x = stage.x(1);
    const auto& y = stage.x(2);
    const auto& theta = stage.x(3);
    const auto& steer = stage.x(4);
    const auto& x_ref = stage.params(0);
    const auto& y_ref = stage.params(1);
    const auto& theta_ref = stage.params(2);
    const auto& kappa_ref = stage.params(3);
    const auto& wheelbase = stage.params(6);

    (*ptr_global)(0) = tan(steer)/wheelbase;
    (*ptr_global)(1) = -(x - x_ref)*sin(theta_ref) + (y - y_ref)*cos(theta_ref);
    (*ptr_global)(2) = cos(theta - theta_ref)/(-kappa_ref*(-(x - x_ref)*sin(theta_ref) + (y - y_ref)*cos(theta_ref)) + 1);
  }));
}

std::shared_ptr<Dynamics> TrackerModel::createDynamics(OcpConfig::IntegratorType type) const {
  return std::make_shared<TrackerModelDynamics>(type);
}

std::shared_ptr<CostFunction> TrackerModel::createCostFunction() const {
  return std::make_shared<TrackerModelCost<StageType::NORMINAL>>();
}

std::shared_ptr<CostFunction> TrackerModel::createTerminalCostFunction() const {
  return std::make_shared<TrackerModelCost<StageType::TERMINAL>>();
}

std::shared_ptr<Constraints> TrackerModel::createConstraint() const {
  return std::make_shared<TrackerModelConstraints<StageType::NORMINAL>>();
}

std::shared_ptr<Constraints> TrackerModel::createInitialConstraint() const {
  return std::make_shared<TrackerModelConstraints<StageType::INITIAL>>();
}

std::shared_ptr<Constraints> TrackerModel::createTerminalConstraint() const {
  return std::make_shared<TrackerModelConstraints<StageType::TERMINAL>>();
}

std::shared_ptr<Constraints> TrackerModel::createStateOnlyEqualities() const {
  return std::make_shared<TrackerModelStateOnlyEqualities<StageType::NORMINAL>>();
}

std::shared_ptr<Constraints> TrackerModel::createInitialStateOnlyEqualities() const {
  return std::make_shared<TrackerModelStateOnlyEqualities<StageType::INITIAL>>();
}

std::shared_ptr<Constraints> TrackerModel::createTerminalEqualities() const {
  return std::make_shared<TrackerModelStateOnlyEqualities<StageType::TERMINAL>>();
}

std::shared_ptr<IPMHelper> TrackerModel::createBaseIpmHelper() const {
  IPMHelper::Creater creater{
    {{StageType::NORMINAL, OcpConfig::FORWARD_EULER},[]() { return std::make_shared<TrackerModelIpmEvaluator<StageType::NORMINAL, OcpConfig::FORWARD_EULER>>(); }},
    {{StageType::NORMINAL, OcpConfig::ERK4},[]() { return std::make_shared<TrackerModelIpmEvaluator<StageType::NORMINAL, OcpConfig::ERK4>>(); }},
    {{StageType::INITIAL, OcpConfig::FORWARD_EULER},[]() { return std::make_shared<TrackerModelIpmEvaluator<StageType::INITIAL, OcpConfig::FORWARD_EULER>>(); }},
    {{StageType::INITIAL, OcpConfig::ERK4},[]() { return std::make_shared<TrackerModelIpmEvaluator<StageType::INITIAL, OcpConfig::ERK4>>(); }},
    {{StageType::TERMINAL, OcpConfig::FORWARD_EULER},[]() { return std::make_shared<TrackerModelIpmEvaluatorTerminal>(); }},
    {{StageType::TERMINAL, OcpConfig::ERK4},[]() { return std::make_shared<TrackerModelIpmEvaluatorTerminal>(); }}};
  return std::make_shared<IPMHelper>(std::move(creater));
}

REGIST_MODEL(TrackerModel)
} // namespace gpal::pnc::planning