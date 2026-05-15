#include "reference_line_model.h"

namespace gpal::pnc::planning {

ReferenceLineModel::ReferenceLineModel() : OptimalControlProblem("ReferenceLineModel", 4, 2, 0, 16, 4) {
  default_state_.emplace("x");
  default_state_.emplace("y");
  default_state_.emplace("theta");
  default_state_.emplace("kappa");

  default_ctrl_.emplace("dkappa");
  default_ctrl_.emplace("slack_offset");

  default_param_.emplace("xr", 0.0);
  default_param_.emplace("yr", 0.0);
  default_param_.emplace("thetar", 0.0);
  default_param_.emplace("l_weight", 0.0);
  default_param_.emplace("kappa_weight", 0.0);
  default_param_.emplace("dkappa_weight", 0.0);
  default_param_.emplace("v_weight", 0.0);
  default_param_.emplace("slack_offset_weight", 0.0);
  default_param_.emplace("l_weight_terminal", 0.0);
  default_param_.emplace("kappa_weight_terminal", 0.0);
  default_param_.emplace("sll", 0.0);
  default_param_.emplace("slu", 0.0);
  default_param_.emplace("KappaLowerBound", 0.0);
  default_param_.emplace("KappaUpperBound", 0.0);
  default_param_.emplace("DKappaLowerBound", 0.0);
  default_param_.emplace("DKappaUpperBound", 0.0);

  default_global_.emplace("cos_thetar");
  default_global_.emplace("sin_thetar");
  default_global_.emplace("theta_offset");
  default_global_.emplace("l");
  default_global_.setUpdater(std::make_shared<OcpVariable::Updater>([](const OcpStage& stage, OcpVariable* ptr_global) {
    // Determine stage variables
    const auto& x = stage.x(0);
    const auto& y = stage.x(1);
    const auto& theta = stage.x(2);
    const auto& xr = stage.params(0);
    const auto& yr = stage.params(1);
    const auto& thetar = stage.params(2);

    (*ptr_global)(0) = cos(thetar);
    (*ptr_global)(1) = sin(thetar);
    (*ptr_global)(2) = cos(theta - thetar);
    (*ptr_global)(3) = (*ptr_global)(0) * (y - yr) - (*ptr_global)(1) * (x - xr);
  }));
}

std::shared_ptr<Dynamics> ReferenceLineModel::createDynamics(OcpConfig::IntegratorType type) const {
  return std::make_shared<ReferenceLineModelDynamics>(type);
}

std::shared_ptr<CostFunction> ReferenceLineModel::createCostFunction() const {
  return std::make_shared<ReferenceLineModelCost<StageType::NORMINAL>>();
}

std::shared_ptr<CostFunction> ReferenceLineModel::createTerminalCostFunction() const {
  return std::make_shared<ReferenceLineModelCost<StageType::TERMINAL>>();
}

std::shared_ptr<Constraints> ReferenceLineModel::createConstraint() const {
  return std::make_shared<ReferenceLineModelConstraints<StageType::NORMINAL>>();
}

std::shared_ptr<Constraints> ReferenceLineModel::createInitialConstraint() const {
  return std::make_shared<ReferenceLineModelConstraints<StageType::INITIAL>>();
}

std::shared_ptr<Constraints> ReferenceLineModel::createTerminalConstraint() const {
  return std::make_shared<ReferenceLineModelConstraints<StageType::TERMINAL>>();
}

std::shared_ptr<Constraints> ReferenceLineModel::createStateOnlyEqualities() const {
  return std::make_shared<ReferenceLineModelStateOnlyEqualities<StageType::NORMINAL>>();
}

std::shared_ptr<Constraints> ReferenceLineModel::createInitialStateOnlyEqualities() const {
  return std::make_shared<ReferenceLineModelStateOnlyEqualities<StageType::INITIAL>>();
}

std::shared_ptr<Constraints> ReferenceLineModel::createTerminalEqualities() const {
  return std::make_shared<ReferenceLineModelStateOnlyEqualities<StageType::TERMINAL>>();
}

std::shared_ptr<IPMHelper> ReferenceLineModel::createBaseIpmHelper() const {
  IPMHelper::Creater creater{
      {{StageType::NORMINAL, OcpConfig::FORWARD_EULER},
       []() {
         return std::make_shared<ReferenceLineModelIpmEvaluator<StageType::NORMINAL, OcpConfig::FORWARD_EULER>>();
       }},
      {{StageType::NORMINAL, OcpConfig::ERK4},
       []() { return std::make_shared<ReferenceLineModelIpmEvaluator<StageType::NORMINAL, OcpConfig::ERK4>>(); }},
      {{StageType::INITIAL, OcpConfig::FORWARD_EULER},
       []() {
         return std::make_shared<ReferenceLineModelIpmEvaluator<StageType::INITIAL, OcpConfig::FORWARD_EULER>>();
       }},
      {{StageType::INITIAL, OcpConfig::ERK4},
       []() { return std::make_shared<ReferenceLineModelIpmEvaluator<StageType::INITIAL, OcpConfig::ERK4>>(); }},
      {{StageType::TERMINAL, OcpConfig::FORWARD_EULER},
       []() { return std::make_shared<ReferenceLineModelIpmEvaluatorTerminal>(); }},
      {{StageType::TERMINAL, OcpConfig::ERK4},
       []() { return std::make_shared<ReferenceLineModelIpmEvaluatorTerminal>(); }}};
  return std::make_shared<IPMHelper>(std::move(creater));
}

REGIST_MODEL(ReferenceLineModel)
}  // namespace gpal::pnc::planning