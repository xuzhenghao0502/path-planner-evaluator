#include "multi_agents_trajectory_model_dynamics.h"

namespace gpal::pnc::planning {

Eigen::VectorXd MultiAgentsTrajectoryModelDynamics::evaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t) const {
  // Determine stage variables
  const auto& ego_theta = states(3);
  const auto& ego_v = states(4);
  const auto& agent_0_v = states(9);
  const auto& ego_a = ctrls(0);
  const auto& agent_0_a = ctrls(2);
  const auto& agent_0_theta_ref = params(7);

  // Determine global variables
  const auto& ego_kappa = globals(0);
  const auto& ego_ds = globals(6);

  // Evaluation of Vector f
  Eigen::VectorXd f = Eigen::VectorXd::Zero(10);

  f(0) = ego_ds*ego_v;
  f(1) = ego_v*cos(ego_theta);
  f(2) = ego_v*sin(ego_theta);
  f(3) = ego_kappa*ego_v;
  f(4) = ego_a;
  f(5) = agent_0_v;
  f(6) = agent_0_v*cos(agent_0_theta_ref);
  f(7) = agent_0_v*sin(agent_0_theta_ref);
  f(9) = agent_0_a;

  return f;
}

Eigen::MatrixXd MultiAgentsTrajectoryModelDynamics::evaluateJx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t) const {
  // Determine stage variables
  const auto& ego_theta = states(3);
  const auto& ego_v = states(4);
  const auto& ego_theta_ref = params(2);
  const auto& ego_kappa_ref = params(3);
  const auto& agent_0_theta_ref = params(7);

  // Determine global variables
  const auto& ego_kappa = globals(0);
  const auto& ego_l = globals(3);
  const auto& ego_ds = globals(6);

  // Determine internal variables
  const double internal_2 = sin(ego_theta);
  const double internal_3 = cos(ego_theta);
  const double internal_0 = ego_v/(-ego_kappa_ref*ego_l + 1);
  const double internal_1 = ego_ds*ego_kappa_ref*internal_0;

  // Evaluation of Matrix jx
  Eigen::MatrixXd jx = Eigen::MatrixXd::Zero(10, 10);

  jx(0, 1) = -internal_1*sin(ego_theta_ref);
  jx(0, 2) = internal_1*cos(ego_theta_ref);
  jx(0, 3) = -internal_0*sin(ego_theta - ego_theta_ref);
  jx(0, 4) = ego_ds;
  jx(1, 3) = -ego_v*internal_2;
  jx(1, 4) = internal_3;
  jx(2, 3) = ego_v*internal_3;
  jx(2, 4) = internal_2;
  jx(3, 4) = ego_kappa;
  jx(5, 9) = 1;
  jx(6, 9) = cos(agent_0_theta_ref);
  jx(7, 9) = sin(agent_0_theta_ref);

  return jx;
}

Eigen::MatrixXd MultiAgentsTrajectoryModelDynamics::evaluateJu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t) const {
  // Determine stage variables
  const auto& ego_v = states(4);
  const auto& ego_steer = ctrls(1);
  const auto& wheelbase = params(17);

  // Evaluation of Matrix ju
  Eigen::MatrixXd ju = Eigen::MatrixXd::Zero(10, 9);

  ju(3, 1) = ego_v*(pow(tan(ego_steer), 2) + 1)/wheelbase;
  ju(4, 0) = 1;
  ju(9, 2) = 1;

  return ju;
}

void MultiAgentsTrajectoryModelDynamics::updateEvaluation(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, Eigen::VectorXd& f) const {
  // Determine stage variables
  const auto& ego_theta = states(3);
  const auto& ego_v = states(4);
  const auto& agent_0_v = states(9);
  const auto& ego_a = ctrls(0);
  const auto& agent_0_a = ctrls(2);
  const auto& agent_0_theta_ref = params(7);

  // Determine global variables
  const auto& ego_kappa = globals(0);
  const auto& ego_ds = globals(6);

  // Evaluation of Vector f
  f(0) = ego_ds*ego_v;
  f(1) = ego_v*cos(ego_theta);
  f(2) = ego_v*sin(ego_theta);
  f(3) = ego_kappa*ego_v;
  f(4) = ego_a;
  f(5) = agent_0_v;
  f(6) = agent_0_v*cos(agent_0_theta_ref);
  f(7) = agent_0_v*sin(agent_0_theta_ref);
  f(9) = agent_0_a;

}

void MultiAgentsTrajectoryModelDynamics::updateJx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, Eigen::MatrixXd& jx) const {
  // Determine stage variables
  const auto& ego_theta = states(3);
  const auto& ego_v = states(4);
  const auto& ego_theta_ref = params(2);
  const auto& ego_kappa_ref = params(3);
  const auto& agent_0_theta_ref = params(7);

  // Determine global variables
  const auto& ego_kappa = globals(0);
  const auto& ego_l = globals(3);
  const auto& ego_ds = globals(6);

  // Determine internal variables
  const double internal_2 = sin(ego_theta);
  const double internal_3 = cos(ego_theta);
  const double internal_0 = ego_v/(-ego_kappa_ref*ego_l + 1);
  const double internal_1 = ego_ds*ego_kappa_ref*internal_0;

  // Evaluation of Matrix jx
  jx(0, 1) = -internal_1*sin(ego_theta_ref);
  jx(0, 2) = internal_1*cos(ego_theta_ref);
  jx(0, 3) = -internal_0*sin(ego_theta - ego_theta_ref);
  jx(0, 4) = ego_ds;
  jx(1, 3) = -ego_v*internal_2;
  jx(1, 4) = internal_3;
  jx(2, 3) = ego_v*internal_3;
  jx(2, 4) = internal_2;
  jx(3, 4) = ego_kappa;
  jx(5, 9) = 1;
  jx(6, 9) = cos(agent_0_theta_ref);
  jx(7, 9) = sin(agent_0_theta_ref);

}

void MultiAgentsTrajectoryModelDynamics::updateJu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, Eigen::MatrixXd& ju) const {
  // Determine stage variables
  const auto& ego_v = states(4);
  const auto& ego_steer = ctrls(1);
  const auto& wheelbase = params(17);

  // Evaluation of Matrix ju
  ju(3, 1) = ego_v*(pow(tan(ego_steer), 2) + 1)/wheelbase;
  ju(4, 0) = 1;
  ju(9, 2) = 1;

}

std::unique_ptr<Integrator> MultiAgentsTrajectoryModelDynamics::createIntegrator(OcpConfig::IntegratorType type) const {
  switch(type) {
    case OcpConfig::ERK4:
      return std::make_unique<MultiAgentsTrajectoryModelIntegrator<OcpConfig::ERK4>>(this);
  }
  return std::make_unique<MultiAgentsTrajectoryModelIntegrator<OcpConfig::FORWARD_EULER>>(this);
}
}  // namespace gpal::pnc::planning