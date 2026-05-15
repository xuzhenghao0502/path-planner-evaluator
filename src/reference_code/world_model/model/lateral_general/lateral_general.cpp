#include "lateral_general.h"

namespace gpal::pnc::planning {

LateralGeneral::LateralGeneral() : OptimalControlProblem("LateralGeneral", 5, 2, 1, 42, 5) {
  default_state_.emplace("s");
  default_state_.emplace("x");
  default_state_.emplace("y");
  default_state_.emplace("theta");
  default_state_.emplace("steer");

  default_ctrl_.emplace("dsteer");
  default_ctrl_.emplace("slack_offset");

  default_param_.emplace("wheelbase", 0.0);
  default_param_.emplace("xr", 0.0);
  default_param_.emplace("yr", 0.0);
  default_param_.emplace("thetar", 0.0);
  default_param_.emplace("kr", 0.0);
  default_param_.emplace("l_weight", 0.01);
  default_param_.emplace("theta_ref_weight", 0.1);
  default_param_.emplace("steer_weight", 0.1);
  default_param_.emplace("dsteer_weight", 1.0);
  default_param_.emplace("weight_slack_offset", 1.0);
  default_param_.emplace("terminal_l_weight", 1.0);
  default_param_.emplace("terminal_heading_weight", 1.0);
  default_param_.emplace("xrf", 0.0);
  default_param_.emplace("yrf", 0.0);
  default_param_.emplace("thetarf", 0.0);
  default_param_.emplace("xrr", 0.0);
  default_param_.emplace("yrr", 0.0);
  default_param_.emplace("thetarr", 0.0);
  default_param_.emplace("lf", 3.701);
  default_param_.emplace("lr", 1.0);
  default_param_.emplace("ll", -20);
  default_param_.emplace("lu", 20);
  default_param_.emplace("sll", -20);
  default_param_.emplace("slu", 20);
  default_param_.emplace("lfl", -20);
  default_param_.emplace("lfu", 20);
  default_param_.emplace("slfl", -20);
  default_param_.emplace("slfu", 20);
  default_param_.emplace("lrl", -20);
  default_param_.emplace("lru", 20);
  default_param_.emplace("slrl", -20);
  default_param_.emplace("slru", 20);
  default_param_.emplace("SteerLowerBound", -0.436);
  default_param_.emplace("SteerUpperBound", 0.436);
  default_param_.emplace("DSteerLowerBound", -0.436);
  default_param_.emplace("DSteerUpperBound", 0.436);
  default_param_.emplace("weight_prev_steer", 0.0);
  default_param_.emplace("weight_prev_dsteer", 0.0);
  default_param_.emplace("prev_steer", 0.0);
  default_param_.emplace("prev_dsteer", 0.0);
  default_param_.emplace("l_offset", 0.0);
  default_param_.emplace("vr", 1);

  default_global_.emplace("kappa");
  default_global_.emplace("l");
  default_global_.emplace("blf");
  default_global_.emplace("blr");
  default_global_.emplace("ds");
  default_global_.setUpdater(std::make_shared<OcpVariable::Updater>([](const OcpStage& stage, OcpVariable* ptr_global) {
    // Determine stage variables
    const auto& x = stage.x(1);
    const auto& y = stage.x(2);
    const auto& theta = stage.x(3);
    const auto& steer = stage.x(4);
    const auto& wheelbase = stage.params(0);
    const auto& xr = stage.params(1);
    const auto& yr = stage.params(2);
    const auto& thetar = stage.params(3);
    const auto& kr = stage.params(4);
    const auto& xrf = stage.params(12);
    const auto& yrf = stage.params(13);
    const auto& thetarf = stage.params(14);
    const auto& xrr = stage.params(15);
    const auto& yrr = stage.params(16);
    const auto& thetarr = stage.params(17);
    const auto& lf = stage.params(18);
    const auto& lr = stage.params(19);

    (*ptr_global)(0) = tan(steer)/wheelbase;
    (*ptr_global)(1) = -(x - xr)*sin(thetar) + (y - yr)*cos(thetar);
    (*ptr_global)(2) = (lf*sin(theta) + y - yrf)*cos(thetarf) - (lf*cos(theta) + x - xrf)*sin(thetarf);
    (*ptr_global)(3) = (-lr*sin(theta) + y - yrr)*cos(thetarr) - (-lr*cos(theta) + x - xrr)*sin(thetarr);
    (*ptr_global)(4) = cos(theta - thetar)/(-kr*(-(x - xr)*sin(thetar) + (y - yr)*cos(thetar)) + 1);
  }));
}

std::shared_ptr<Dynamics> LateralGeneral::createDynamics(OcpConfig::IntegratorType type) const {
  return std::make_shared<LateralGeneralDynamics>(type);
}

std::shared_ptr<CostFunction> LateralGeneral::createCostFunction() const {
  return std::make_shared<LateralGeneralCost<StageType::NORMINAL>>();
}

std::shared_ptr<CostFunction> LateralGeneral::createTerminalCostFunction() const {
  return std::make_shared<LateralGeneralCost<StageType::TERMINAL>>();
}

std::shared_ptr<Constraints> LateralGeneral::createConstraint() const {
  return std::make_shared<LateralGeneralConstraints<StageType::NORMINAL>>();
}

std::shared_ptr<Constraints> LateralGeneral::createInitialConstraint() const {
  return std::make_shared<LateralGeneralConstraints<StageType::INITIAL>>();
}

std::shared_ptr<Constraints> LateralGeneral::createTerminalConstraint() const {
  return std::make_shared<LateralGeneralConstraints<StageType::TERMINAL>>();
}

std::shared_ptr<Constraints> LateralGeneral::createStateOnlyEqualities() const {
  return std::make_shared<LateralGeneralStateOnlyEqualities<StageType::NORMINAL>>();
}

std::shared_ptr<Constraints> LateralGeneral::createInitialStateOnlyEqualities() const {
  return std::make_shared<LateralGeneralStateOnlyEqualities<StageType::INITIAL>>();
}

std::shared_ptr<Constraints> LateralGeneral::createTerminalEqualities() const {
  return std::make_shared<LateralGeneralStateOnlyEqualities<StageType::TERMINAL>>();
}

std::shared_ptr<IPMHelper> LateralGeneral::createBaseIpmHelper() const {
  IPMHelper::Creater creater{
    {{StageType::NORMINAL, OcpConfig::FORWARD_EULER},[]() { return std::make_shared<LateralGeneralIpmEvaluator<StageType::NORMINAL, OcpConfig::FORWARD_EULER>>(); }},
    {{StageType::NORMINAL, OcpConfig::ERK4},[]() { return std::make_shared<LateralGeneralIpmEvaluator<StageType::NORMINAL, OcpConfig::ERK4>>(); }},
    {{StageType::INITIAL, OcpConfig::FORWARD_EULER},[]() { return std::make_shared<LateralGeneralIpmEvaluator<StageType::INITIAL, OcpConfig::FORWARD_EULER>>(); }},
    {{StageType::INITIAL, OcpConfig::ERK4},[]() { return std::make_shared<LateralGeneralIpmEvaluator<StageType::INITIAL, OcpConfig::ERK4>>(); }},
    {{StageType::TERMINAL, OcpConfig::FORWARD_EULER},[]() { return std::make_shared<LateralGeneralIpmEvaluatorTerminal>(); }},
    {{StageType::TERMINAL, OcpConfig::ERK4},[]() { return std::make_shared<LateralGeneralIpmEvaluatorTerminal>(); }}};
  return std::make_shared<IPMHelper>(std::move(creater));
}

REGIST_MODEL(LateralGeneral)
} // namespace gpal::pnc::planning