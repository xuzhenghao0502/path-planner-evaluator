#include "reference_line_model_dynamics.h"

namespace gpal::pnc::planning {

Eigen::VectorXd ReferenceLineModelDynamics::evaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                                     const double t) const {
  // Determine stage variables
  const auto& theta = states(2);
  const auto& kappa = states(3);
  const auto& dkappa = ctrls(0);

  // Determine internal variables
  const double cos_theta = cos(theta);
  const double sin_theta = sin(theta);

  // Evaluation of Vector f
  Eigen::VectorXd f = Eigen::VectorXd::Zero(4);

  f(0) = cos_theta;
  f(1) = sin_theta;
  f(2) = kappa;
  f(3) = dkappa;

  return f;
}

Eigen::MatrixXd ReferenceLineModelDynamics::evaluateJx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                                       const double t) const {
  // Determine stage variables
  const auto& theta = states(2);

  // Determine internal variables
  const double cos_theta = cos(theta);
  const double sin_theta = sin(theta);

  // Evaluation of Matrix jx
  Eigen::MatrixXd jx = Eigen::MatrixXd::Zero(4, 4);

  jx(0, 2) = -sin_theta;
  jx(1, 2) = cos_theta;
  jx(2, 3) = 1;

  return jx;
}

Eigen::MatrixXd ReferenceLineModelDynamics::evaluateJu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                                       const double t) const {
  // Evaluation of Matrix ju
  Eigen::MatrixXd ju = Eigen::MatrixXd::Zero(4, 2);

  ju(3, 0) = 1;

  return ju;
}

void ReferenceLineModelDynamics::updateEvaluation(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                                  const double t, Eigen::VectorXd& f) const {
  // Determine stage variables
  const auto& theta = states(2);
  const auto& kappa = states(3);
  const auto& dkappa = ctrls(0);

  // Determine internal variables
  const double cos_theta = cos(theta);
  const double sin_theta = sin(theta);

  // Evaluation of Vector f
  f(0) = cos_theta;
  f(1) = sin_theta;
  f(2) = kappa;
  f(3) = dkappa;
}

void ReferenceLineModelDynamics::updateJx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t,
                                          Eigen::MatrixXd& jx) const {
  // Determine stage variables
  const auto& theta = states(2);

  // Determine internal variables
  const double cos_theta = cos(theta);
  const double sin_theta = sin(theta);

  // Evaluation of Matrix jx
  jx(0, 2) = -sin_theta;
  jx(1, 2) = cos_theta;
  jx(2, 3) = 1;
}

void ReferenceLineModelDynamics::updateJu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t,
                                          Eigen::MatrixXd& ju) const {
  // Evaluation of Matrix ju
  ju(3, 0) = 1;
}

std::unique_ptr<Integrator> ReferenceLineModelDynamics::createIntegrator(OcpConfig::IntegratorType type) const {
  switch (type) {
    case OcpConfig::ERK4:
      return std::make_unique<ReferenceLineModelIntegrator<OcpConfig::ERK4>>(this);
  }
  return std::make_unique<ReferenceLineModelIntegrator<OcpConfig::FORWARD_EULER>>(this);
}
}  // namespace gpal::pnc::planning