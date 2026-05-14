#include "speed_risk_model.h"

namespace gpal::pnc::planning {

SpeedRiskModel::SpeedRiskModel() : OptimalControlProblem("SpeedRiskModel", 3, 8, 7, 33, 0) {
  default_state_.emplace("s");
  default_state_.emplace("v");
  default_state_.emplace("a");

  default_ctrl_.emplace("j");
  default_ctrl_.emplace("slack_s_upper");
  default_ctrl_.emplace("slack_s_lower");
  default_ctrl_.emplace("slack_v_upper");
  default_ctrl_.emplace("slack_v_lower");
  default_ctrl_.emplace("slack_a_upper");
  default_ctrl_.emplace("slack_a_lower");
  default_ctrl_.emplace("slack_d_v");

  default_param_.emplace("SRef", 285.0);
  default_param_.emplace("VRef", 30.0);
  default_param_.emplace("K", 1.5);
  default_param_.emplace("SWeight", 0.004);
  default_param_.emplace("VWeight", 0.001);
  default_param_.emplace("AWeight", 0.1);
  default_param_.emplace("JWeight", 10.0);
  default_param_.emplace("SlackSUpperWeight", 100.0);
  default_param_.emplace("SlackSLowerWeight", 0.0001);
  default_param_.emplace("SlackVUpperWeight", 100000.0);
  default_param_.emplace("SlackVLowerWeight", 0.0001);
  default_param_.emplace("SlackAUpperWeight", 0.1);
  default_param_.emplace("SlackALowerWeight", 0.1);
  default_param_.emplace("SlackDVWeight", 1.0);
  default_param_.emplace("SHardUpperBound", 300.0);
  default_param_.emplace("SHardLowerBound", 0.0);
  default_param_.emplace("SSoftUpperBound", 200.0);
  default_param_.emplace("SSoftLowerBound", 0.0);
  default_param_.emplace("SUpperBoundForDVConstraint", 200.0);
  default_param_.emplace("VHardUpperBound", 40.0);
  default_param_.emplace("VHardLowerBound", 0.0);
  default_param_.emplace("VSoftUpperBound", 30.0);
  default_param_.emplace("VSoftLowerBound", 0.0);
  default_param_.emplace("AHardUpperBound", 2.0);
  default_param_.emplace("AHardLowerBound", -6.0);
  default_param_.emplace("ASoftUpperBound", 1.99);
  default_param_.emplace("ASoftLowerBound", -5.0);
  default_param_.emplace("JHardUpperBound", 2.0);
  default_param_.emplace("JHardLowerBound", -10.0);
  default_param_.emplace("SafeDistForDVConstraint", 4.0);
  default_param_.emplace("k", 0.3);
  default_param_.emplace("residual", 0.0);
  default_param_.emplace("df", 0.0);
}

std::shared_ptr<Dynamics> SpeedRiskModel::createDynamics(OcpConfig::IntegratorType type) const {
  return std::make_shared<SpeedRiskModelDynamics>(type);
}

std::shared_ptr<CostFunction> SpeedRiskModel::createCostFunction() const {
  return std::make_shared<SpeedRiskModelCost<StageType::NORMINAL>>();
}

std::shared_ptr<CostFunction> SpeedRiskModel::createTerminalCostFunction() const {
  return std::make_shared<SpeedRiskModelCost<StageType::TERMINAL>>();
}

std::shared_ptr<Constraints> SpeedRiskModel::createConstraint() const {
  return std::make_shared<SpeedRiskModelConstraints<StageType::NORMINAL>>();
}

std::shared_ptr<Constraints> SpeedRiskModel::createInitialConstraint() const {
  return std::make_shared<SpeedRiskModelConstraints<StageType::INITIAL>>();
}

std::shared_ptr<Constraints> SpeedRiskModel::createTerminalConstraint() const {
  return std::make_shared<SpeedRiskModelConstraints<StageType::TERMINAL>>();
}

std::shared_ptr<Constraints> SpeedRiskModel::createStateOnlyEqualities() const {
  return std::make_shared<SpeedRiskModelStateOnlyEqualities<StageType::NORMINAL>>();
}

std::shared_ptr<Constraints> SpeedRiskModel::createInitialStateOnlyEqualities() const {
  return std::make_shared<SpeedRiskModelStateOnlyEqualities<StageType::INITIAL>>();
}

std::shared_ptr<Constraints> SpeedRiskModel::createTerminalEqualities() const {
  return std::make_shared<SpeedRiskModelStateOnlyEqualities<StageType::TERMINAL>>();
}

std::shared_ptr<IPMHelper> SpeedRiskModel::createBaseIpmHelper() const {
  IPMHelper::Creater creater{
      {{StageType::NORMINAL, OcpConfig::FORWARD_EULER},
       []() { return std::make_shared<SpeedRiskModelIpmEvaluator<StageType::NORMINAL, OcpConfig::FORWARD_EULER>>(); }},
      {{StageType::NORMINAL, OcpConfig::ERK4},
       []() { return std::make_shared<SpeedRiskModelIpmEvaluator<StageType::NORMINAL, OcpConfig::ERK4>>(); }},
      {{StageType::INITIAL, OcpConfig::FORWARD_EULER},
       []() { return std::make_shared<SpeedRiskModelIpmEvaluator<StageType::INITIAL, OcpConfig::FORWARD_EULER>>(); }},
      {{StageType::INITIAL, OcpConfig::ERK4},
       []() { return std::make_shared<SpeedRiskModelIpmEvaluator<StageType::INITIAL, OcpConfig::ERK4>>(); }},
      {{StageType::TERMINAL, OcpConfig::FORWARD_EULER},
       []() { return std::make_shared<SpeedRiskModelIpmEvaluatorTerminal>(); }},
      {{StageType::TERMINAL, OcpConfig::ERK4},
       []() { return std::make_shared<SpeedRiskModelIpmEvaluatorTerminal>(); }}};
  return std::make_shared<IPMHelper>(std::move(creater));
}

REGIST_MODEL(SpeedRiskModel)
}  // namespace gpal::pnc::planning