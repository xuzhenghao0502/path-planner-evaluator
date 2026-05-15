#include "speed_risk_model_dynamics.h"

namespace gpal::pnc::planning {

Eigen::VectorXd SpeedRiskModelDynamics::evaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                                 const double t) const {
  // Determine stage variables
  const auto& v = states(1);
  const auto& a = states(2);
  const auto& j = ctrls(0);

  // Evaluation of Vector f
  Eigen::VectorXd f = Eigen::VectorXd::Zero(3);

  f(0) = v;
  f(1) = a;
  f(2) = j;

  return f;
}

Eigen::MatrixXd SpeedRiskModelDynamics::evaluateJx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                                   const double t) const {
  // Evaluation of Matrix jx
  Eigen::MatrixXd jx = Eigen::MatrixXd::Zero(3, 3);

  jx(0, 1) = 1;
  jx(1, 2) = 1;

  return jx;
}

Eigen::MatrixXd SpeedRiskModelDynamics::evaluateJu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                                   const double t) const {
  // Evaluation of Matrix ju
  Eigen::MatrixXd ju = Eigen::MatrixXd::Zero(3, 8);

  ju(2, 0) = 1;

  return ju;
}

void SpeedRiskModelDynamics::updateEvaluation(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                              const double t, Eigen::VectorXd& f) const {
  // Determine stage variables
  const auto& v = states(1);
  const auto& a = states(2);
  const auto& j = ctrls(0);

  // Evaluation of Vector f
  f(0) = v;
  f(1) = a;
  f(2) = j;
}

void SpeedRiskModelDynamics::updateJx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t,
                                      Eigen::MatrixXd& jx) const {
  // Evaluation of Matrix jx
  jx(0, 1) = 1;
  jx(1, 2) = 1;
}

void SpeedRiskModelDynamics::updateJu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t,
                                      Eigen::MatrixXd& ju) const {
  // Evaluation of Matrix ju
  ju(2, 0) = 1;
}

std::unique_ptr<Integrator> SpeedRiskModelDynamics::createIntegrator(OcpConfig::IntegratorType type) const {
  switch (type) {
    case OcpConfig::ERK4:
      return std::make_unique<SpeedRiskModelIntegrator<OcpConfig::ERK4>>(this);
  }
  return std::make_unique<SpeedRiskModelIntegrator<OcpConfig::FORWARD_EULER>>(this);
}
}  // namespace gpal::pnc::planning