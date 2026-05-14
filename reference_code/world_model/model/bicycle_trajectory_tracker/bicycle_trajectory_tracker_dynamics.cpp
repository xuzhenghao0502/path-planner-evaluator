#include "bicycle_trajectory_tracker_dynamics.h"

namespace gpal::pnc::planning {

Eigen::VectorXd BicycleTrajectoryTrackerDynamics::evaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                                           const double t) const {
  // Determine stage variables
  const auto& theta = states(2);
  const auto& v = states(3);
  const auto& kappa = states(4);
  const auto& a = ctrls(0);
  const auto& dkappa = ctrls(1);

  // Determine internal variables
  const double cos_theta = cos(theta);
  const double sin_theta = sin(theta);

  // Evaluation of Vector f
  Eigen::VectorXd f = Eigen::VectorXd::Zero(5);

  f(0) = cos_theta * v;
  f(1) = sin_theta * v;
  f(2) = kappa * v;
  f(3) = a;
  f(4) = dkappa;

  return f;
}

Eigen::MatrixXd BicycleTrajectoryTrackerDynamics::evaluateJx(const Eigen::VectorXd& states,
                                                             const Eigen::VectorXd& ctrls, const double t) const {
  // Determine stage variables
  const auto& theta = states(2);
  const auto& v = states(3);
  const auto& kappa = states(4);

  // Determine internal variables
  const double cos_theta = cos(theta);
  const double sin_theta = sin(theta);

  // Evaluation of Matrix jx
  Eigen::MatrixXd jx = Eigen::MatrixXd::Zero(5, 5);

  jx(0, 2) = -sin_theta * v;
  jx(0, 3) = cos_theta;
  jx(1, 2) = cos_theta * v;
  jx(1, 3) = sin_theta;
  jx(2, 3) = kappa;
  jx(2, 4) = v;

  return jx;
}

Eigen::MatrixXd BicycleTrajectoryTrackerDynamics::evaluateJu(const Eigen::VectorXd& states,
                                                             const Eigen::VectorXd& ctrls, const double t) const {
  // Evaluation of Matrix ju
  Eigen::MatrixXd ju = Eigen::MatrixXd::Zero(5, 2);

  ju(3, 0) = 1;
  ju(4, 1) = 1;

  return ju;
}

void BicycleTrajectoryTrackerDynamics::updateEvaluation(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                                        const double t, Eigen::VectorXd& f) const {
  // Determine stage variables
  const auto& theta = states(2);
  const auto& v = states(3);
  const auto& kappa = states(4);
  const auto& a = ctrls(0);
  const auto& dkappa = ctrls(1);

  // Determine internal variables
  const double cos_theta = cos(theta);
  const double sin_theta = sin(theta);

  // Evaluation of Vector f
  f(0) = cos_theta * v;
  f(1) = sin_theta * v;
  f(2) = kappa * v;
  f(3) = a;
  f(4) = dkappa;
}

void BicycleTrajectoryTrackerDynamics::updateJx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                                const double t, Eigen::MatrixXd& jx) const {
  // Determine stage variables
  const auto& theta = states(2);
  const auto& v = states(3);
  const auto& kappa = states(4);

  // Determine internal variables
  const double cos_theta = cos(theta);
  const double sin_theta = sin(theta);

  // Evaluation of Matrix jx
  jx(0, 2) = -sin_theta * v;
  jx(0, 3) = cos_theta;
  jx(1, 2) = cos_theta * v;
  jx(1, 3) = sin_theta;
  jx(2, 3) = kappa;
  jx(2, 4) = v;
}

void BicycleTrajectoryTrackerDynamics::updateJu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                                const double t, Eigen::MatrixXd& ju) const {
  // Evaluation of Matrix ju
  ju(3, 0) = 1;
  ju(4, 1) = 1;
}

std::unique_ptr<Integrator> BicycleTrajectoryTrackerDynamics::createIntegrator(OcpConfig::IntegratorType type) const {
  switch (type) {
    case OcpConfig::ERK4:
      return std::make_unique<BicycleTrajectoryTrackerIntegrator<OcpConfig::ERK4>>(this);
  }
  return std::make_unique<BicycleTrajectoryTrackerIntegrator<OcpConfig::FORWARD_EULER>>(this);
}
}  // namespace gpal::pnc::planning