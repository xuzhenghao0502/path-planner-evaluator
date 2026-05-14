#include "park_lateral_general.h"

namespace gpal::pnc::planning {

ParkLateralGeneral::ParkLateralGeneral() : OptimalControlProblem("ParkLateralGeneral", 5, 2, 1, 39, 14) {
  default_state_.emplace("s");
  default_state_.emplace("x");
  default_state_.emplace("y");
  default_state_.emplace("theta");
  default_state_.emplace("kappa");

  default_ctrl_.emplace("dkappa");
  default_ctrl_.emplace("slack_offset");

  default_param_.emplace("xr", 0.0);
  default_param_.emplace("yr", 0.0);
  default_param_.emplace("thetar", 0.0);
  default_param_.emplace("kr", 0.0);
  default_param_.emplace("vr", 0.0);
  default_param_.emplace("ref_weight", 0.01);
  default_param_.emplace("kappa_weight", 10.0);
  default_param_.emplace("dkappa_weight", 100.0);
  default_param_.emplace("weigth_slack_offset", 1.0);
  default_param_.emplace("terminal_position_weight", 1.0);
  default_param_.emplace("terminal_heading_weight", 1.0);
  default_param_.emplace("xrf", 0.0);
  default_param_.emplace("yrf", 0.0);
  default_param_.emplace("thetarf", 0.0);
  default_param_.emplace("xrr", 0.0);
  default_param_.emplace("yrr", 0.0);
  default_param_.emplace("thetarr", 0.0);
  default_param_.emplace("lf", 4.0);
  default_param_.emplace("lr", 1.0);
  default_param_.emplace("ll", -20);
  default_param_.emplace("lu", 20);
  default_param_.emplace("lfl", -20);
  default_param_.emplace("lfu", 20);
  default_param_.emplace("lrl", -20);
  default_param_.emplace("lru", 20);
  default_param_.emplace("sll", -20);
  default_param_.emplace("slu", 20);
  default_param_.emplace("KappaLowerBound", -0.1);
  default_param_.emplace("KappaUpperBound", 0.1);
  default_param_.emplace("DKappaLowerBound", -0.1);
  default_param_.emplace("DKappaUpperBound", 0.1);
  default_param_.emplace("weigth_prev_kappa", 0.0);
  default_param_.emplace("weight_prev_dkappa", 0.0);
  default_param_.emplace("prev_kappa", 0.0);
  default_param_.emplace("prev_dkappa", 0.0);
  default_param_.emplace("slfl", -20);
  default_param_.emplace("slfu", 20);
  default_param_.emplace("slrl", -20);
  default_param_.emplace("slru", 20);

  default_global_.emplace("cos_theta");
  default_global_.emplace("sin_theta");
  default_global_.emplace("anonymous_0");
  default_global_.emplace("cos_thetar");
  default_global_.emplace("sin_thetar");
  default_global_.emplace("cos_thetarf");
  default_global_.emplace("sin_thetarf");
  default_global_.emplace("cos_thetarr");
  default_global_.emplace("sin_thetarr");
  default_global_.emplace("cos_anonymous_0");
  default_global_.emplace("l");
  default_global_.emplace("blf");
  default_global_.emplace("blr");
  default_global_.emplace("ds");
  default_global_.setUpdater(std::make_shared<OcpVariable::Updater>([](const OcpStage& stage, OcpVariable* ptr_global) {
    // Determine stage variables
    const auto& x = stage.x(1);
    const auto& y = stage.x(2);
    const auto& theta = stage.x(3);
    const auto& xr = stage.params(0);
    const auto& yr = stage.params(1);
    const auto& thetar = stage.params(2);
    const auto& kr = stage.params(3);
    const auto& xrf = stage.params(11);
    const auto& yrf = stage.params(12);
    const auto& thetarf = stage.params(13);
    const auto& xrr = stage.params(14);
    const auto& yrr = stage.params(15);
    const auto& thetarr = stage.params(16);
    const auto& lf = stage.params(17);
    const auto& lr = stage.params(18);

    (*ptr_global)(0) = cos(theta);
    (*ptr_global)(1) = sin(theta);
    (*ptr_global)(2) = theta - thetar;
    (*ptr_global)(3) = cos(thetar);
    (*ptr_global)(4) = sin(thetar);
    (*ptr_global)(5) = cos(thetarf);
    (*ptr_global)(6) = sin(thetarf);
    (*ptr_global)(7) = cos(thetarr);
    (*ptr_global)(8) = sin(thetarr);
    (*ptr_global)(9) = cos((*ptr_global)(2));
    (*ptr_global)(10) = (*ptr_global)(3) * (y - yr) - (*ptr_global)(4) * (x - xr);
    (*ptr_global)(11) =
        (*ptr_global)(5) * ((*ptr_global)(1) * lf + y - yrf) - (*ptr_global)(6) * ((*ptr_global)(0) * lf + x - xrf);
    (*ptr_global)(12) =
        (*ptr_global)(7) * (-(*ptr_global)(1) * lr + y - yrr) - (*ptr_global)(8) * (-(*ptr_global)(0) * lr + x - xrr);
    (*ptr_global)(13) = (*ptr_global)(9) / (-(*ptr_global)(10) * kr + 1);
  }));
}

std::shared_ptr<Dynamics> ParkLateralGeneral::createDynamics(OcpConfig::IntegratorType type) const {
  return std::make_shared<ParkLateralGeneralDynamics>(type);
}

std::shared_ptr<CostFunction> ParkLateralGeneral::createCostFunction() const {
  return std::make_shared<ParkLateralGeneralCost<StageType::NORMINAL>>();
}

std::shared_ptr<CostFunction> ParkLateralGeneral::createTerminalCostFunction() const {
  return std::make_shared<ParkLateralGeneralCost<StageType::TERMINAL>>();
}

std::shared_ptr<Constraints> ParkLateralGeneral::createConstraint() const {
  return std::make_shared<ParkLateralGeneralConstraints<StageType::NORMINAL>>();
}

std::shared_ptr<Constraints> ParkLateralGeneral::createInitialConstraint() const {
  return std::make_shared<ParkLateralGeneralConstraints<StageType::INITIAL>>();
}

std::shared_ptr<Constraints> ParkLateralGeneral::createTerminalConstraint() const {
  return std::make_shared<ParkLateralGeneralConstraints<StageType::TERMINAL>>();
}

std::shared_ptr<Constraints> ParkLateralGeneral::createStateOnlyEqualities() const {
  return std::make_shared<ParkLateralGeneralStateOnlyEqualities<StageType::NORMINAL>>();
}

std::shared_ptr<Constraints> ParkLateralGeneral::createInitialStateOnlyEqualities() const {
  return std::make_shared<ParkLateralGeneralStateOnlyEqualities<StageType::INITIAL>>();
}

std::shared_ptr<Constraints> ParkLateralGeneral::createTerminalEqualities() const {
  return std::make_shared<ParkLateralGeneralStateOnlyEqualities<StageType::TERMINAL>>();
}

std::shared_ptr<IPMHelper> ParkLateralGeneral::createBaseIpmHelper() const {
  IPMHelper::Creater creater{
      {{StageType::NORMINAL, OcpConfig::FORWARD_EULER},
       []() {
         return std::make_shared<ParkLateralGeneralIpmEvaluator<StageType::NORMINAL, OcpConfig::FORWARD_EULER>>();
       }},
      {{StageType::NORMINAL, OcpConfig::ERK4},
       []() { return std::make_shared<ParkLateralGeneralIpmEvaluator<StageType::NORMINAL, OcpConfig::ERK4>>(); }},
      {{StageType::INITIAL, OcpConfig::FORWARD_EULER},
       []() {
         return std::make_shared<ParkLateralGeneralIpmEvaluator<StageType::INITIAL, OcpConfig::FORWARD_EULER>>();
       }},
      {{StageType::INITIAL, OcpConfig::ERK4},
       []() { return std::make_shared<ParkLateralGeneralIpmEvaluator<StageType::INITIAL, OcpConfig::ERK4>>(); }},
      {{StageType::TERMINAL, OcpConfig::FORWARD_EULER},
       []() { return std::make_shared<ParkLateralGeneralIpmEvaluatorTerminal>(); }},
      {{StageType::TERMINAL, OcpConfig::ERK4},
       []() { return std::make_shared<ParkLateralGeneralIpmEvaluatorTerminal>(); }}};
  return std::make_shared<IPMHelper>(std::move(creater));
}

REGIST_MODEL(ParkLateralGeneral)
}  // namespace gpal::pnc::planning