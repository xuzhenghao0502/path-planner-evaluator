#include "parking_general.h"

namespace gpal::pnc::planning {

ParkingGeneral::ParkingGeneral() : OptimalControlProblem("ParkingGeneral", 6, 3, 1, 43, 5) {
  default_state_.emplace("s");
  default_state_.emplace("x");
  default_state_.emplace("y");
  default_state_.emplace("theta");
  default_state_.emplace("steer");
  default_state_.emplace("v");

  default_ctrl_.emplace("dsteer");
  default_ctrl_.emplace("a");
  default_ctrl_.emplace("slack_offset");

  default_param_.emplace("wheelbase", 0.0);
  default_param_.emplace("xr", 0.0);
  default_param_.emplace("yr", 0.0);
  default_param_.emplace("thetar", 0.0);
  default_param_.emplace("kr", 0.0);
  default_param_.emplace("vr", 0.0);
  default_param_.emplace("ref_weight", 0.01);
  default_param_.emplace("steer_weight", 10.0);
  default_param_.emplace("dsteer_weight", 0.1);
  default_param_.emplace("terminal_ref_weight", 1.0);
  default_param_.emplace("terminal_heading_weight", 1.0);
  default_param_.emplace("terminal_v_weight", 1.0);
  default_param_.emplace("v_weight", 0.0);
  default_param_.emplace("a_weight", 1.0);
  default_param_.emplace("weigth_slack_offset", 1.0);
  default_param_.emplace("xrf", 0.0);
  default_param_.emplace("yrf", 0.0);
  default_param_.emplace("thetarf", 0.0);
  default_param_.emplace("xrr", 0.0);
  default_param_.emplace("yrr", 0.0);
  default_param_.emplace("thetarr", 0.0);
  default_param_.emplace("steer_lower", -0.436);
  default_param_.emplace("steer_upper", 0.436);
  default_param_.emplace("dsteer_lower", -0.436);
  default_param_.emplace("dsteer_upper", 0.436);
  default_param_.emplace("v_lower", -3);
  default_param_.emplace("v_upper", 3);
  default_param_.emplace("a_lower", -6);
  default_param_.emplace("a_upper", 2);
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
  default_param_.emplace("slfl", -20);
  default_param_.emplace("slfu", 20);
  default_param_.emplace("slrl", -20);
  default_param_.emplace("slru", 20);

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
    const auto& xrf = stage.params(15);
    const auto& yrf = stage.params(16);
    const auto& thetarf = stage.params(17);
    const auto& xrr = stage.params(18);
    const auto& yrr = stage.params(19);
    const auto& thetarr = stage.params(20);
    const auto& lf = stage.params(29);
    const auto& lr = stage.params(30);

    (*ptr_global)(0) = tan(steer)/wheelbase;
    (*ptr_global)(1) = -(x - xr)*sin(thetar) + (y - yr)*cos(thetar);
    (*ptr_global)(2) = (lf*sin(theta) + y - yrf)*cos(thetarf) - (lf*cos(theta) + x - xrf)*sin(thetarf);
    (*ptr_global)(3) = (-lr*sin(theta) + y - yrr)*cos(thetarr) - (-lr*cos(theta) + x - xrr)*sin(thetarr);
    (*ptr_global)(4) = cos(theta - thetar)/(-kr*(-(x - xr)*sin(thetar) + (y - yr)*cos(thetar)) + 1);
  }));
}

std::shared_ptr<Dynamics> ParkingGeneral::createDynamics(OcpConfig::IntegratorType type) const {
  return std::make_shared<ParkingGeneralDynamics>(type);
}

std::shared_ptr<CostFunction> ParkingGeneral::createCostFunction() const {
  return std::make_shared<ParkingGeneralCost<StageType::NORMINAL>>();
}

std::shared_ptr<Constraints> ParkingGeneral::createConstraint() const {
  return std::make_shared<ParkingGeneralConstraints<StageType::NORMINAL>>();
}

std::shared_ptr<Constraints> ParkingGeneral::createInitialConstraint() const {
  return std::make_shared<ParkingGeneralConstraints<StageType::INITIAL>>();
}

std::shared_ptr<Constraints> ParkingGeneral::createTerminalConstraint() const {
  return std::make_shared<ParkingGeneralConstraints<StageType::TERMINAL>>();
}

std::shared_ptr<Constraints> ParkingGeneral::createStateOnlyEqualities() const {
  return std::make_shared<ParkingGeneralStateOnlyEqualities<StageType::NORMINAL>>();
}

std::shared_ptr<Constraints> ParkingGeneral::createInitialStateOnlyEqualities() const {
  return std::make_shared<ParkingGeneralStateOnlyEqualities<StageType::INITIAL>>();
}

std::shared_ptr<Constraints> ParkingGeneral::createTerminalEqualities() const {
  return std::make_shared<ParkingGeneralStateOnlyEqualities<StageType::TERMINAL>>();
}

std::shared_ptr<IPMHelper> ParkingGeneral::createBaseIpmHelper() const {
  IPMHelper::Creater creater{
    {{StageType::NORMINAL, OcpConfig::FORWARD_EULER},[]() { return std::make_shared<ParkingGeneralIpmEvaluator<StageType::NORMINAL, OcpConfig::FORWARD_EULER>>(); }},
    {{StageType::NORMINAL, OcpConfig::ERK4},[]() { return std::make_shared<ParkingGeneralIpmEvaluator<StageType::NORMINAL, OcpConfig::ERK4>>(); }},
    {{StageType::INITIAL, OcpConfig::FORWARD_EULER},[]() { return std::make_shared<ParkingGeneralIpmEvaluator<StageType::INITIAL, OcpConfig::FORWARD_EULER>>(); }},
    {{StageType::INITIAL, OcpConfig::ERK4},[]() { return std::make_shared<ParkingGeneralIpmEvaluator<StageType::INITIAL, OcpConfig::ERK4>>(); }},
    {{StageType::TERMINAL, OcpConfig::FORWARD_EULER},[]() { return std::make_shared<ParkingGeneralIpmEvaluatorTerminal>(); }},
    {{StageType::TERMINAL, OcpConfig::ERK4},[]() { return std::make_shared<ParkingGeneralIpmEvaluatorTerminal>(); }}};
  return std::make_shared<IPMHelper>(std::move(creater));
}

REGIST_MODEL(ParkingGeneral)
} // namespace gpal::pnc::planning