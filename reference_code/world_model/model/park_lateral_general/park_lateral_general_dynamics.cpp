#include "park_lateral_general_dynamics.h"

namespace gpal::pnc::planning {

Eigen::VectorXd ParkLateralGeneralDynamics::evaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                                     const double t) const {
  // Determine stage variables
  const auto& x = states(1);
  const auto& y = states(2);
  const auto& theta = states(3);
  const auto& kappa = states(4);
  const auto& dkappa = ctrls(0);
  const auto& xr = params(0);
  const auto& yr = params(1);
  const auto& thetar = params(2);
  const auto& kr = params(3);
  const auto& vr = params(4);

  // Determine internal variables
  const double cos_theta = cos(theta);
  const double sin_theta = sin(theta);
  const double cos_thetar = cos(thetar);
  const double sin_thetar = sin(thetar);
  const double internal_0 = theta - thetar;
  const double cos_internal_0 = cos(internal_0);

  // Evaluation of Vector f
  Eigen::VectorXd f = Eigen::VectorXd::Zero(5);

  f(0) = cos_internal_0 * vr / (-kr * (cos_thetar * (y - yr) - sin_thetar * (x - xr)) + 1);
  f(1) = cos_theta * vr;
  f(2) = sin_theta * vr;
  f(3) = kappa * vr;
  f(4) = dkappa;

  return f;
}

Eigen::MatrixXd ParkLateralGeneralDynamics::evaluateJx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                                       const double t) const {
  // Determine stage variables
  const auto& x = states(1);
  const auto& y = states(2);
  const auto& theta = states(3);
  const auto& xr = params(0);
  const auto& yr = params(1);
  const auto& thetar = params(2);
  const auto& kr = params(3);
  const auto& vr = params(4);

  // Determine internal variables
  const double cos_theta = cos(theta);
  const double sin_theta = sin(theta);
  const double cos_thetar = cos(thetar);
  const double sin_thetar = sin(thetar);
  const double internal_0 = theta - thetar;
  const double cos_internal_0 = cos(internal_0);
  const double sin_internal_0 = sin(internal_0);
  const double internal_1 = -kr * (cos_thetar * (y - yr) - sin_thetar * (x - xr)) + 1;
  const double internal_1_p2 = internal_1 * internal_1;

  // Evaluation of Matrix jx
  Eigen::MatrixXd jx = Eigen::MatrixXd::Zero(5, 5);

  jx(0, 1) = -cos_internal_0 * kr * sin_thetar * vr / internal_1_p2;
  jx(0, 2) = cos_internal_0 * cos_thetar * kr * vr / internal_1_p2;
  jx(0, 3) = -sin_internal_0 * vr / internal_1;
  jx(1, 3) = -sin_theta * vr;
  jx(2, 3) = cos_theta * vr;
  jx(3, 4) = vr;

  return jx;
}

Eigen::MatrixXd ParkLateralGeneralDynamics::evaluateJu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                                       const double t) const {
  // Evaluation of Matrix ju
  Eigen::MatrixXd ju = Eigen::MatrixXd::Zero(5, 2);

  ju(4, 0) = 1;

  return ju;
}

void ParkLateralGeneralDynamics::updateEvaluation(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                                  const double t, Eigen::VectorXd& f) const {
  // Determine stage variables
  const auto& x = states(1);
  const auto& y = states(2);
  const auto& theta = states(3);
  const auto& kappa = states(4);
  const auto& dkappa = ctrls(0);
  const auto& xr = params(0);
  const auto& yr = params(1);
  const auto& thetar = params(2);
  const auto& kr = params(3);
  const auto& vr = params(4);

  // Determine internal variables
  const double cos_theta = cos(theta);
  const double sin_theta = sin(theta);
  const double cos_thetar = cos(thetar);
  const double sin_thetar = sin(thetar);
  const double internal_0 = theta - thetar;
  const double cos_internal_0 = cos(internal_0);

  // Evaluation of Vector f
  f(0) = cos_internal_0 * vr / (-kr * (cos_thetar * (y - yr) - sin_thetar * (x - xr)) + 1);
  f(1) = cos_theta * vr;
  f(2) = sin_theta * vr;
  f(3) = kappa * vr;
  f(4) = dkappa;
}

void ParkLateralGeneralDynamics::updateJx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t,
                                          Eigen::MatrixXd& jx) const {
  // Determine stage variables
  const auto& x = states(1);
  const auto& y = states(2);
  const auto& theta = states(3);
  const auto& xr = params(0);
  const auto& yr = params(1);
  const auto& thetar = params(2);
  const auto& kr = params(3);
  const auto& vr = params(4);

  // Determine internal variables
  const double cos_theta = cos(theta);
  const double sin_theta = sin(theta);
  const double cos_thetar = cos(thetar);
  const double sin_thetar = sin(thetar);
  const double internal_0 = theta - thetar;
  const double cos_internal_0 = cos(internal_0);
  const double sin_internal_0 = sin(internal_0);
  const double internal_1 = -kr * (cos_thetar * (y - yr) - sin_thetar * (x - xr)) + 1;
  const double internal_1_p2 = internal_1 * internal_1;

  // Evaluation of Matrix jx
  jx(0, 1) = -cos_internal_0 * kr * sin_thetar * vr / internal_1_p2;
  jx(0, 2) = cos_internal_0 * cos_thetar * kr * vr / internal_1_p2;
  jx(0, 3) = -sin_internal_0 * vr / internal_1;
  jx(1, 3) = -sin_theta * vr;
  jx(2, 3) = cos_theta * vr;
  jx(3, 4) = vr;
}

void ParkLateralGeneralDynamics::updateJu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t,
                                          Eigen::MatrixXd& ju) const {
  // Evaluation of Matrix ju
  ju(4, 0) = 1;
}

std::unique_ptr<Integrator> ParkLateralGeneralDynamics::createIntegrator(OcpConfig::IntegratorType type) const {
  switch (type) {
    case OcpConfig::ERK4:
      return std::make_unique<ParkLateralGeneralIntegrator<OcpConfig::ERK4>>(this);
  }
  return std::make_unique<ParkLateralGeneralIntegrator<OcpConfig::FORWARD_EULER>>(this);
}
}  // namespace gpal::pnc::planning