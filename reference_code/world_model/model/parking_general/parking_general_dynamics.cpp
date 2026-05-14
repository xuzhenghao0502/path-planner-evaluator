#include "parking_general_dynamics.h"

namespace gpal::pnc::planning {

Eigen::VectorXd ParkingGeneralDynamics::evaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t) const {
  // Determine stage variables
  const auto& x = states(1);
  const auto& y = states(2);
  const auto& theta = states(3);
  const auto& steer = states(4);
  const auto& v = states(5);
  const auto& dsteer = ctrls(0);
  const auto& a = ctrls(1);
  const auto& wheelbase = params(0);
  const auto& xr = params(1);
  const auto& yr = params(2);
  const auto& thetar = params(3);
  const auto& kr = params(4);

  // Evaluation of Vector f
  Eigen::VectorXd f = Eigen::VectorXd::Zero(6);

  f(0) = v*cos(theta - thetar)/(-kr*(-(x - xr)*sin(thetar) + (y - yr)*cos(thetar)) + 1);
  f(1) = v*cos(theta);
  f(2) = v*sin(theta);
  f(3) = v*tan(steer)/wheelbase;
  f(4) = dsteer;
  f(5) = a;

  return f;
}

Eigen::MatrixXd ParkingGeneralDynamics::evaluateJx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t) const {
  // Determine stage variables
  const auto& x = states(1);
  const auto& y = states(2);
  const auto& theta = states(3);
  const auto& steer = states(4);
  const auto& v = states(5);
  const auto& wheelbase = params(0);
  const auto& xr = params(1);
  const auto& yr = params(2);
  const auto& thetar = params(3);
  const auto& kr = params(4);

  // Determine internal variables
  const double internal_7 = sin(theta);
  const double internal_8 = cos(theta);
  const double internal_10 = tan(steer);
  const double internal_9 = 1.0/wheelbase;
  const double internal_0 = sin(thetar);
  const double internal_1 = theta - thetar;
  const double internal_3 = cos(thetar);
  const double internal_2 = cos(internal_1);
  const double internal_4 = -kr*(-internal_0*(x - xr) + internal_3*(y - yr)) + 1;
  const double internal_6 = 1.0/internal_4;
  const double internal_5 = internal_2*kr*v/pow(internal_4, 2);

  // Evaluation of Matrix jx
  Eigen::MatrixXd jx = Eigen::MatrixXd::Zero(6, 6);

  jx(0, 1) = -internal_0*internal_5;
  jx(0, 2) = internal_3*internal_5;
  jx(0, 3) = -internal_6*v*sin(internal_1);
  jx(0, 5) = internal_2*internal_6;
  jx(1, 3) = -internal_7*v;
  jx(1, 5) = internal_8;
  jx(2, 3) = internal_8*v;
  jx(2, 5) = internal_7;
  jx(3, 4) = internal_9*v*(pow(internal_10, 2) + 1);
  jx(3, 5) = internal_10*internal_9;

  return jx;
}

Eigen::MatrixXd ParkingGeneralDynamics::evaluateJu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t) const {
  // Evaluation of Matrix ju
  Eigen::MatrixXd ju = Eigen::MatrixXd::Zero(6, 3);

  ju(4, 0) = 1;
  ju(5, 1) = 1;

  return ju;
}

void ParkingGeneralDynamics::updateEvaluation(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, Eigen::VectorXd& f) const {
  // Determine stage variables
  const auto& x = states(1);
  const auto& y = states(2);
  const auto& theta = states(3);
  const auto& steer = states(4);
  const auto& v = states(5);
  const auto& dsteer = ctrls(0);
  const auto& a = ctrls(1);
  const auto& wheelbase = params(0);
  const auto& xr = params(1);
  const auto& yr = params(2);
  const auto& thetar = params(3);
  const auto& kr = params(4);

  // Evaluation of Vector f
  f(0) = v*cos(theta - thetar)/(-kr*(-(x - xr)*sin(thetar) + (y - yr)*cos(thetar)) + 1);
  f(1) = v*cos(theta);
  f(2) = v*sin(theta);
  f(3) = v*tan(steer)/wheelbase;
  f(4) = dsteer;
  f(5) = a;

}

void ParkingGeneralDynamics::updateJx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, Eigen::MatrixXd& jx) const {
  // Determine stage variables
  const auto& x = states(1);
  const auto& y = states(2);
  const auto& theta = states(3);
  const auto& steer = states(4);
  const auto& v = states(5);
  const auto& wheelbase = params(0);
  const auto& xr = params(1);
  const auto& yr = params(2);
  const auto& thetar = params(3);
  const auto& kr = params(4);

  // Determine internal variables
  const double internal_7 = sin(theta);
  const double internal_8 = cos(theta);
  const double internal_10 = tan(steer);
  const double internal_9 = 1.0/wheelbase;
  const double internal_0 = sin(thetar);
  const double internal_1 = theta - thetar;
  const double internal_3 = cos(thetar);
  const double internal_2 = cos(internal_1);
  const double internal_4 = -kr*(-internal_0*(x - xr) + internal_3*(y - yr)) + 1;
  const double internal_6 = 1.0/internal_4;
  const double internal_5 = internal_2*kr*v/pow(internal_4, 2);

  // Evaluation of Matrix jx
  jx(0, 1) = -internal_0*internal_5;
  jx(0, 2) = internal_3*internal_5;
  jx(0, 3) = -internal_6*v*sin(internal_1);
  jx(0, 5) = internal_2*internal_6;
  jx(1, 3) = -internal_7*v;
  jx(1, 5) = internal_8;
  jx(2, 3) = internal_8*v;
  jx(2, 5) = internal_7;
  jx(3, 4) = internal_9*v*(pow(internal_10, 2) + 1);
  jx(3, 5) = internal_10*internal_9;

}

void ParkingGeneralDynamics::updateJu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, Eigen::MatrixXd& ju) const {
  // Evaluation of Matrix ju
  ju(4, 0) = 1;
  ju(5, 1) = 1;

}

std::unique_ptr<Integrator> ParkingGeneralDynamics::createIntegrator(OcpConfig::IntegratorType type) const {
  switch(type) {
    case OcpConfig::ERK4:
      return std::make_unique<ParkingGeneralIntegrator<OcpConfig::ERK4>>(this);
  }
  return std::make_unique<ParkingGeneralIntegrator<OcpConfig::FORWARD_EULER>>(this);
}
}  // namespace gpal::pnc::planning