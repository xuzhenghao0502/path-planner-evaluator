#include "lateral_multi_gear.h"

namespace gpal::pnc::planning {

LateralMultiGear::LateralMultiGear() : OptimalControlProblem("LateralMultiGear", 4, 1, 0, 15, 3) {
  default_state_.emplace("x");
  default_state_.emplace("y");
  default_state_.emplace("theta");
  default_state_.emplace("kappa");

  default_ctrl_.emplace("dkappa");

  default_param_.emplace("xr", 0.0);
  default_param_.emplace("yr", 0.0);
  default_param_.emplace("thetar", 0.0);
  default_param_.emplace("kr", 0.0);
  default_param_.emplace("vr", 0.0);
  default_param_.emplace("weight_position", 0.01);
  default_param_.emplace("weight_heading", 0.1);
  default_param_.emplace("weight_kappa", 1.0);
  default_param_.emplace("weight_dkappa", 10.0);
  default_param_.emplace("terminal_position_weight", 1.0);
  default_param_.emplace("terminal_heading_weight", 1.0);
  default_param_.emplace("kappa_lower", -0.1);
  default_param_.emplace("kappa_upper", 0.1);
  default_param_.emplace("dkappa_lower", -0.1);
  default_param_.emplace("dkappa_upper", 0.1);

  default_global_.emplace("cos_thetar");
  default_global_.emplace("sin_thetar");
  default_global_.emplace("l");
  default_global_.setUpdater(std::make_shared<OcpVariable::Updater>([](const OcpStage& stage, OcpVariable* ptr_global) {
    // Determine stage variables
    const auto& x = stage.x(0);
    const auto& y = stage.x(1);
    const auto& xr = stage.params(0);
    const auto& yr = stage.params(1);
    const auto& thetar = stage.params(2);

    (*ptr_global)(0) = cos(thetar);
    (*ptr_global)(1) = sin(thetar);
    (*ptr_global)(2) = (*ptr_global)(0) * (y - yr) - (*ptr_global)(1) * (x - xr);
  }));
}

std::shared_ptr<Dynamics> LateralMultiGear::createDynamics(OcpConfig::IntegratorType type) const {
  return std::make_shared<LateralMultiGearDynamics>(type);
}

std::shared_ptr<CostFunction> LateralMultiGear::createCostFunction() const {
  return std::make_shared<LateralMultiGearCost<StageType::NORMINAL>>();
}

std::shared_ptr<CostFunction> LateralMultiGear::createTerminalCostFunction() const {
  return std::make_shared<LateralMultiGearCost<StageType::TERMINAL>>();
}

std::shared_ptr<Constraints> LateralMultiGear::createConstraint() const {
  return std::make_shared<LateralMultiGearConstraints<StageType::NORMINAL>>();
}

std::shared_ptr<Constraints> LateralMultiGear::createInitialConstraint() const {
  return std::make_shared<LateralMultiGearConstraints<StageType::INITIAL>>();
}

std::shared_ptr<Constraints> LateralMultiGear::createTerminalConstraint() const {
  return std::make_shared<LateralMultiGearConstraints<StageType::TERMINAL>>();
}

std::shared_ptr<Constraints> LateralMultiGear::createStateOnlyEqualities() const {
  return std::make_shared<LateralMultiGearStateOnlyEqualities<StageType::NORMINAL>>();
}

std::shared_ptr<Constraints> LateralMultiGear::createInitialStateOnlyEqualities() const {
  return std::make_shared<LateralMultiGearStateOnlyEqualities<StageType::INITIAL>>();
}

std::shared_ptr<Constraints> LateralMultiGear::createTerminalEqualities() const {
  return std::make_shared<LateralMultiGearStateOnlyEqualities<StageType::TERMINAL>>();
}

std::shared_ptr<IPMHelper> LateralMultiGear::createBaseIpmHelper() const {
  IPMHelper::Creater creater{
      {{StageType::NORMINAL, OcpConfig::FORWARD_EULER},
       []() {
         return std::make_shared<LateralMultiGearIpmEvaluator<StageType::NORMINAL, OcpConfig::FORWARD_EULER>>();
       }},
      {{StageType::NORMINAL, OcpConfig::ERK4},
       []() { return std::make_shared<LateralMultiGearIpmEvaluator<StageType::NORMINAL, OcpConfig::ERK4>>(); }},
      {{StageType::INITIAL, OcpConfig::FORWARD_EULER},
       []() { return std::make_shared<LateralMultiGearIpmEvaluator<StageType::INITIAL, OcpConfig::FORWARD_EULER>>(); }},
      {{StageType::INITIAL, OcpConfig::ERK4},
       []() { return std::make_shared<LateralMultiGearIpmEvaluator<StageType::INITIAL, OcpConfig::ERK4>>(); }},
      {{StageType::TERMINAL, OcpConfig::FORWARD_EULER},
       []() { return std::make_shared<LateralMultiGearIpmEvaluatorTerminal>(); }},
      {{StageType::TERMINAL, OcpConfig::ERK4},
       []() { return std::make_shared<LateralMultiGearIpmEvaluatorTerminal>(); }}};
  return std::make_shared<IPMHelper>(std::move(creater));
}

REGIST_MODEL(LateralMultiGear)
}  // namespace gpal::pnc::planning