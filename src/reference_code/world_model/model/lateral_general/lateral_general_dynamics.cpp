#include "lateral_general_dynamics.h"

namespace gpal::pnc::planning {

Eigen::VectorXd LateralGeneralDynamics::evaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t) const {
  // Determine stage variables
  const auto& x = states(1);
  const auto& y = states(2);
  const auto& theta = states(3);
  const auto& steer = states(4);
  const auto& dsteer = ctrls(0);
  const auto& wheelbase = params(0);
  const auto& xr = params(1);
  const auto& yr = params(2);
  const auto& thetar = params(3);
  const auto& kr = params(4);
  const auto& vr = params(41);

  // Evaluation of Vector f
  Eigen::VectorXd f = Eigen::VectorXd::Zero(5);

  f(0) = vr*cos(theta - thetar)/(-kr*(-(x - xr)*sin(thetar) + (y - yr)*cos(thetar)) + 1);
  f(1) = vr*cos(theta);
  f(2) = vr*sin(theta);
  f(3) = vr*tan(steer)/wheelbase;
  f(4) = dsteer;

  return f;
}

Eigen::MatrixXd LateralGeneralDynamics::evaluateJx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t) const {
  // Determine stage variables
  const auto& x = states(1);
  const auto& y = states(2);
  const auto& theta = states(3);
  const auto& steer = states(4);
  const auto& wheelbase = params(0);
  const auto& xr = params(1);
  const auto& yr = params(2);
  const auto& thetar = params(3);
  const auto& kr = params(4);
  const auto& vr = params(41);

  // Determine internal variables
  const double internal_0 = sin(thetar);
  const double internal_1 = cos(thetar);
  const double internal_3 = theta - thetar;
  const double internal_2 = -kr*(-internal_0*(x - xr) + internal_1*(y - yr)) + 1;
  const double internal_4 = kr*vr*cos(internal_3)/pow(internal_2, 2);

  // Evaluation of Matrix jx
  Eigen::MatrixXd jx = Eigen::MatrixXd::Zero(5, 5);

  jx(0, 1) = -internal_0*internal_4;
  jx(0, 2) = internal_1*internal_4;
  jx(0, 3) = -vr*sin(internal_3)/internal_2;
  jx(1, 3) = -vr*sin(theta);
  jx(2, 3) = vr*cos(theta);
  jx(3, 4) = vr*(pow(tan(steer), 2) + 1)/wheelbase;

  return jx;
}

Eigen::MatrixXd LateralGeneralDynamics::evaluateJu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t) const {
  // Evaluation of Matrix ju
  Eigen::MatrixXd ju = Eigen::MatrixXd::Zero(5, 2);

  ju(4, 0) = 1;

  return ju;
}

void LateralGeneralDynamics::updateEvaluation(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, Eigen::VectorXd& f) const {
  // Determine stage variables
  const auto& x = states(1);
  const auto& y = states(2);
  const auto& theta = states(3);
  const auto& steer = states(4);
  const auto& dsteer = ctrls(0);
  const auto& wheelbase = params(0);
  const auto& xr = params(1);
  const auto& yr = params(2);
  const auto& thetar = params(3);
  const auto& kr = params(4);
  const auto& vr = params(41);

  // Evaluation of Vector f
  f(0) = vr*cos(theta - thetar)/(-kr*(-(x - xr)*sin(thetar) + (y - yr)*cos(thetar)) + 1);
  f(1) = vr*cos(theta);
  f(2) = vr*sin(theta);
  f(3) = vr*tan(steer)/wheelbase;
  f(4) = dsteer;

}

void LateralGeneralDynamics::updateJx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, Eigen::MatrixXd& jx) const {
  // Determine stage variables
  const auto& x = states(1);
  const auto& y = states(2);
  const auto& theta = states(3);
  const auto& steer = states(4);
  const auto& wheelbase = params(0);
  const auto& xr = params(1);
  const auto& yr = params(2);
  const auto& thetar = params(3);
  const auto& kr = params(4);
  const auto& vr = params(41);

  // Determine internal variables
  const double internal_0 = sin(thetar);
  const double internal_1 = cos(thetar);
  const double internal_3 = theta - thetar;
  const double internal_2 = -kr*(-internal_0*(x - xr) + internal_1*(y - yr)) + 1;
  const double internal_4 = kr*vr*cos(internal_3)/pow(internal_2, 2);

  // Evaluation of Matrix jx
  jx(0, 1) = -internal_0*internal_4;
  jx(0, 2) = internal_1*internal_4;
  jx(0, 3) = -vr*sin(internal_3)/internal_2;
  jx(1, 3) = -vr*sin(theta);
  jx(2, 3) = vr*cos(theta);
  jx(3, 4) = vr*(pow(tan(steer), 2) + 1)/wheelbase;

}

void LateralGeneralDynamics::updateJu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, Eigen::MatrixXd& ju) const {
  // Evaluation of Matrix ju
  ju(4, 0) = 1;

}

std::unique_ptr<Integrator> LateralGeneralDynamics::createIntegrator(OcpConfig::IntegratorType type) const {
  switch(type) {
    case OcpConfig::ERK4:
      return std::make_unique<LateralGeneralIntegrator<OcpConfig::ERK4>>(this);
  }
  return std::make_unique<LateralGeneralIntegrator<OcpConfig::FORWARD_EULER>>(this);
}
}  // namespace gpal::pnc::planning