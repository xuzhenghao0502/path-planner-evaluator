#include "spatiotemporal_planner_model_dynamics.h"

namespace gpal::pnc::planning {

Eigen::VectorXd SpatiotemporalPlannerModelDynamics::evaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t) const {
  // Determine stage variables
  const auto& x = states(1);
  const auto& y = states(2);
  const auto& theta = states(3);
  const auto& steer = states(4);
  const auto& v = states(5);
  const auto& a = states(6);
  const auto& dsteer = ctrls(0);
  const auto& jerk = ctrls(1);
  const auto& x_ref = params(0);
  const auto& y_ref = params(1);
  const auto& theta_ref = params(2);
  const auto& kappa_ref = params(3);
  const auto& wheelbase = params(14);

  // Evaluation of Vector f
  Eigen::VectorXd f = Eigen::VectorXd::Zero(7);

  f(0) = v*cos(theta - theta_ref)/(-kappa_ref*(-(x - x_ref)*sin(theta_ref) + (y - y_ref)*cos(theta_ref)) + 1);
  f(1) = v*cos(theta);
  f(2) = v*sin(theta);
  f(3) = v*tan(steer)/wheelbase;
  f(4) = dsteer;
  f(5) = a;
  f(6) = jerk;

  return f;
}

Eigen::MatrixXd SpatiotemporalPlannerModelDynamics::evaluateJx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t) const {
  // Determine stage variables
  const auto& x = states(1);
  const auto& y = states(2);
  const auto& theta = states(3);
  const auto& steer = states(4);
  const auto& v = states(5);
  const auto& x_ref = params(0);
  const auto& y_ref = params(1);
  const auto& theta_ref = params(2);
  const auto& kappa_ref = params(3);
  const auto& wheelbase = params(14);

  // Determine internal variables
  const double internal_7 = sin(theta);
  const double internal_8 = cos(theta);
  const double internal_10 = tan(steer);
  const double internal_0 = sin(theta_ref);
  const double internal_1 = theta - theta_ref;
  const double internal_3 = cos(theta_ref);
  const double internal_9 = 1.0/wheelbase;
  const double internal_2 = cos(internal_1);
  const double internal_4 = -kappa_ref*(-internal_0*(x - x_ref) + internal_3*(y - y_ref)) + 1;
  const double internal_6 = 1.0/internal_4;
  const double internal_5 = internal_2*kappa_ref*v/pow(internal_4, 2);

  // Evaluation of Matrix jx
  Eigen::MatrixXd jx = Eigen::MatrixXd::Zero(7, 7);

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
  jx(5, 6) = 1;

  return jx;
}

Eigen::MatrixXd SpatiotemporalPlannerModelDynamics::evaluateJu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t) const {
  // Evaluation of Matrix ju
  Eigen::MatrixXd ju = Eigen::MatrixXd::Zero(7, 13);

  ju(4, 0) = 1;
  ju(6, 1) = 1;

  return ju;
}

void SpatiotemporalPlannerModelDynamics::updateEvaluation(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, Eigen::VectorXd& f) const {
  // Determine stage variables
  const auto& x = states(1);
  const auto& y = states(2);
  const auto& theta = states(3);
  const auto& steer = states(4);
  const auto& v = states(5);
  const auto& a = states(6);
  const auto& dsteer = ctrls(0);
  const auto& jerk = ctrls(1);
  const auto& x_ref = params(0);
  const auto& y_ref = params(1);
  const auto& theta_ref = params(2);
  const auto& kappa_ref = params(3);
  const auto& wheelbase = params(14);

  // Evaluation of Vector f
  f(0) = v*cos(theta - theta_ref)/(-kappa_ref*(-(x - x_ref)*sin(theta_ref) + (y - y_ref)*cos(theta_ref)) + 1);
  f(1) = v*cos(theta);
  f(2) = v*sin(theta);
  f(3) = v*tan(steer)/wheelbase;
  f(4) = dsteer;
  f(5) = a;
  f(6) = jerk;

}

void SpatiotemporalPlannerModelDynamics::updateJx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, Eigen::MatrixXd& jx) const {
  // Determine stage variables
  const auto& x = states(1);
  const auto& y = states(2);
  const auto& theta = states(3);
  const auto& steer = states(4);
  const auto& v = states(5);
  const auto& x_ref = params(0);
  const auto& y_ref = params(1);
  const auto& theta_ref = params(2);
  const auto& kappa_ref = params(3);
  const auto& wheelbase = params(14);

  // Determine internal variables
  const double internal_7 = sin(theta);
  const double internal_8 = cos(theta);
  const double internal_10 = tan(steer);
  const double internal_0 = sin(theta_ref);
  const double internal_1 = theta - theta_ref;
  const double internal_3 = cos(theta_ref);
  const double internal_9 = 1.0/wheelbase;
  const double internal_2 = cos(internal_1);
  const double internal_4 = -kappa_ref*(-internal_0*(x - x_ref) + internal_3*(y - y_ref)) + 1;
  const double internal_6 = 1.0/internal_4;
  const double internal_5 = internal_2*kappa_ref*v/pow(internal_4, 2);

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
  jx(5, 6) = 1;

}

void SpatiotemporalPlannerModelDynamics::updateJu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, Eigen::MatrixXd& ju) const {
  // Evaluation of Matrix ju
  ju(4, 0) = 1;
  ju(6, 1) = 1;

}

std::unique_ptr<Integrator> SpatiotemporalPlannerModelDynamics::createIntegrator(OcpConfig::IntegratorType type) const {
  switch(type) {
    case OcpConfig::ERK4:
      return std::make_unique<SpatiotemporalPlannerModelIntegrator<OcpConfig::ERK4>>(this);
  }
  return std::make_unique<SpatiotemporalPlannerModelIntegrator<OcpConfig::FORWARD_EULER>>(this);
}
}  // namespace gpal::pnc::planning