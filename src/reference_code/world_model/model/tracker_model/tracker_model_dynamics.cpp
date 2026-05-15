#include "tracker_model_dynamics.h"

namespace gpal::pnc::planning {

Eigen::VectorXd TrackerModelDynamics::evaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t) const {
  // Determine stage variables
  const auto& x = states(1);
  const auto& y = states(2);
  const auto& theta = states(3);
  const auto& steer = states(4);
  const auto& dsteer = ctrls(0);
  const auto& x_ref = params(0);
  const auto& y_ref = params(1);
  const auto& theta_ref = params(2);
  const auto& kappa_ref = params(3);
  const auto& v_ref = params(4);
  const auto& wheelbase = params(6);

  // Evaluation of Vector f
  Eigen::VectorXd f = Eigen::VectorXd::Zero(5);

  f(0) = v_ref*cos(theta - theta_ref)/(-kappa_ref*(-(x - x_ref)*sin(theta_ref) + (y - y_ref)*cos(theta_ref)) + 1);
  f(1) = v_ref*cos(theta);
  f(2) = v_ref*sin(theta);
  f(3) = v_ref*tan(steer)/wheelbase;
  f(4) = dsteer;

  return f;
}

Eigen::MatrixXd TrackerModelDynamics::evaluateJx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t) const {
  // Determine stage variables
  const auto& x = states(1);
  const auto& y = states(2);
  const auto& theta = states(3);
  const auto& steer = states(4);
  const auto& x_ref = params(0);
  const auto& y_ref = params(1);
  const auto& theta_ref = params(2);
  const auto& kappa_ref = params(3);
  const auto& v_ref = params(4);
  const auto& wheelbase = params(6);

  // Determine internal variables
  const double internal_0 = sin(theta_ref);
  const double internal_1 = cos(theta_ref);
  const double internal_3 = theta - theta_ref;
  const double internal_2 = -kappa_ref*(-internal_0*(x - x_ref) + internal_1*(y - y_ref)) + 1;
  const double internal_4 = kappa_ref*v_ref*cos(internal_3)/pow(internal_2, 2);

  // Evaluation of Matrix jx
  Eigen::MatrixXd jx = Eigen::MatrixXd::Zero(5, 5);

  jx(0, 1) = -internal_0*internal_4;
  jx(0, 2) = internal_1*internal_4;
  jx(0, 3) = -v_ref*sin(internal_3)/internal_2;
  jx(1, 3) = -v_ref*sin(theta);
  jx(2, 3) = v_ref*cos(theta);
  jx(3, 4) = v_ref*(pow(tan(steer), 2) + 1)/wheelbase;

  return jx;
}

Eigen::MatrixXd TrackerModelDynamics::evaluateJu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t) const {
  // Evaluation of Matrix ju
  Eigen::MatrixXd ju = Eigen::MatrixXd::Zero(5, 2);

  ju(4, 0) = 1;

  return ju;
}

void TrackerModelDynamics::updateEvaluation(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, Eigen::VectorXd& f) const {
  // Determine stage variables
  const auto& x = states(1);
  const auto& y = states(2);
  const auto& theta = states(3);
  const auto& steer = states(4);
  const auto& dsteer = ctrls(0);
  const auto& x_ref = params(0);
  const auto& y_ref = params(1);
  const auto& theta_ref = params(2);
  const auto& kappa_ref = params(3);
  const auto& v_ref = params(4);
  const auto& wheelbase = params(6);

  // Evaluation of Vector f
  f(0) = v_ref*cos(theta - theta_ref)/(-kappa_ref*(-(x - x_ref)*sin(theta_ref) + (y - y_ref)*cos(theta_ref)) + 1);
  f(1) = v_ref*cos(theta);
  f(2) = v_ref*sin(theta);
  f(3) = v_ref*tan(steer)/wheelbase;
  f(4) = dsteer;

}

void TrackerModelDynamics::updateJx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, Eigen::MatrixXd& jx) const {
  // Determine stage variables
  const auto& x = states(1);
  const auto& y = states(2);
  const auto& theta = states(3);
  const auto& steer = states(4);
  const auto& x_ref = params(0);
  const auto& y_ref = params(1);
  const auto& theta_ref = params(2);
  const auto& kappa_ref = params(3);
  const auto& v_ref = params(4);
  const auto& wheelbase = params(6);

  // Determine internal variables
  const double internal_0 = sin(theta_ref);
  const double internal_1 = cos(theta_ref);
  const double internal_3 = theta - theta_ref;
  const double internal_2 = -kappa_ref*(-internal_0*(x - x_ref) + internal_1*(y - y_ref)) + 1;
  const double internal_4 = kappa_ref*v_ref*cos(internal_3)/pow(internal_2, 2);

  // Evaluation of Matrix jx
  jx(0, 1) = -internal_0*internal_4;
  jx(0, 2) = internal_1*internal_4;
  jx(0, 3) = -v_ref*sin(internal_3)/internal_2;
  jx(1, 3) = -v_ref*sin(theta);
  jx(2, 3) = v_ref*cos(theta);
  jx(3, 4) = v_ref*(pow(tan(steer), 2) + 1)/wheelbase;

}

void TrackerModelDynamics::updateJu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, Eigen::MatrixXd& ju) const {
  // Evaluation of Matrix ju
  ju(4, 0) = 1;

}

std::unique_ptr<Integrator> TrackerModelDynamics::createIntegrator(OcpConfig::IntegratorType type) const {
  switch(type) {
    case OcpConfig::ERK4:
      return std::make_unique<TrackerModelIntegrator<OcpConfig::ERK4>>(this);
  }
  return std::make_unique<TrackerModelIntegrator<OcpConfig::FORWARD_EULER>>(this);
}
}  // namespace gpal::pnc::planning