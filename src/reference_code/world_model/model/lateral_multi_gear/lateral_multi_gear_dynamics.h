#pragma once

#include "ocp/dynamics.h"

namespace gpal::pnc::planning {

class LateralMultiGearDynamics : public Dynamics {
 public:
  explicit LateralMultiGearDynamics(OcpConfig::IntegratorType type = OcpConfig::ERK4) : Dynamics(4, 1, type) {}
  virtual ~LateralMultiGearDynamics() = default;

  virtual Eigen::VectorXd evaluate(const Eigen::VectorXd&, const Eigen::VectorXd&, const double) const override;
  virtual Eigen::MatrixXd evaluateJx(const Eigen::VectorXd&, const Eigen::VectorXd&, const double) const override;
  virtual Eigen::MatrixXd evaluateJu(const Eigen::VectorXd&, const Eigen::VectorXd&, const double) const override;
  virtual void updateEvaluation(const Eigen::VectorXd& x, const Eigen::VectorXd& u, const double,
                                Eigen::VectorXd&) const override;
  virtual void updateJx(const Eigen::VectorXd& x, const Eigen::VectorXd& u, const double,
                        Eigen::MatrixXd&) const override;
  virtual void updateJu(const Eigen::VectorXd& x, const Eigen::VectorXd& u, const double,
                        Eigen::MatrixXd&) const override;
  virtual std::unique_ptr<Integrator> createIntegrator(OcpConfig::IntegratorType type) const override;
};

template <OcpConfig::IntegratorType Ttype>
class LateralMultiGearIntegrator : public Integrator {
 public:
  LateralMultiGearIntegrator(const Dynamics* dynamics) : Integrator(dynamics) {}
  virtual ~LateralMultiGearIntegrator() = default;

  virtual OcpConfig::IntegratorType type() const { return OcpConfig::FORWARD_EULER; }
  virtual void update(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, const double t_next,
                      Eigen::VectorXd& F) const override {
    // Determine stage variables
    const auto& x = states(0);
    const auto& y = states(1);
    const auto& theta = states(2);
    const auto& kappa = states(3);
    const auto& dkappa = ctrls(0);
    const auto& vr = params(4);

    // Determine internal variables
    const double dt = -t + t_next;
    const double cos_theta = cos(theta);
    const double sin_theta = sin(theta);

    // Evaluation of Vector F
    F(0) = cos_theta * dt * vr + x;
    F(1) = dt * sin_theta * vr + y;
    F(2) = dt * kappa * vr + theta;
    F(3) = dkappa * dt + kappa;
  }

  virtual void update(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, const double t_next,
                      Eigen::VectorXd& F, Eigen::MatrixXd& Fx, Eigen::MatrixXd& Fu) const override {
    // Determine stage variables
    const auto& x = states(0);
    const auto& y = states(1);
    const auto& theta = states(2);
    const auto& kappa = states(3);
    const auto& dkappa = ctrls(0);
    const auto& vr = params(4);

    // Determine internal variables
    const double dt = -t + t_next;
    const double cos_theta = cos(theta);
    const double sin_theta = sin(theta);

    // Evaluation of Vector F
    F(0) = cos_theta * dt * vr + x;
    F(1) = dt * sin_theta * vr + y;
    F(2) = dt * kappa * vr + theta;
    F(3) = dkappa * dt + kappa;

    // Evaluation of Matrix Fx
    Fx(0, 0) = 1;
    Fx(0, 2) = -dt * sin_theta * vr;
    Fx(1, 1) = 1;
    Fx(1, 2) = cos_theta * dt * vr;
    Fx(2, 2) = 1;
    Fx(2, 3) = dt * vr;
    Fx(3, 3) = 1;

    // Evaluation of Matrix Fu
    Fu(3, 0) = dt;
  }

  virtual void updateFx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t,
                        const double t_next, Eigen::MatrixXd& Fx) const override {
    // Determine stage variables
    const auto& theta = states(2);
    const auto& vr = params(4);

    // Determine internal variables
    const double dt = -t + t_next;
    const double cos_theta = cos(theta);
    const double sin_theta = sin(theta);

    // Evaluation of Matrix Fx
    Fx(0, 0) = 1;
    Fx(0, 2) = -dt * sin_theta * vr;
    Fx(1, 1) = 1;
    Fx(1, 2) = cos_theta * dt * vr;
    Fx(2, 2) = 1;
    Fx(2, 3) = dt * vr;
    Fx(3, 3) = 1;
  }

  virtual void updateFu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t,
                        const double t_next, Eigen::MatrixXd& Fu) const override {
    // Determine internal variables
    const double dt = -t + t_next;

    // Evaluation of Matrix Fu
    Fu(3, 0) = dt;
  }
};

template <>
class LateralMultiGearIntegrator<OcpConfig::ERK4> : public Integrator {
 public:
  LateralMultiGearIntegrator(const Dynamics* dynamics) : Integrator(dynamics) {}
  virtual ~LateralMultiGearIntegrator() = default;

  virtual OcpConfig::IntegratorType type() const { return OcpConfig::ERK4; }
  virtual void update(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, const double t_next,
                      Eigen::VectorXd& F) const override {
    // Determine stage variables
    const auto& x = states(0);
    const auto& y = states(1);
    const auto& theta = states(2);
    const auto& kappa = states(3);
    const auto& dkappa = ctrls(0);
    const auto& vr = params(4);

    // Determine internal variables
    const double dt = -t + t_next;
    const double cos_theta = cos(theta);
    const double sin_theta = sin(theta);
    const double internal_2 = 0.5 * dt * kappa * vr + theta;
    const double cos_internal_2 = cos(internal_2);
    const double internal_0 = 0.5 * dt * vr * (0.5 * dkappa * dt + kappa) + theta;
    const double internal_1 = dt * vr * (0.5 * dkappa * dt + kappa) + theta;
    const double sin_internal_2 = sin(internal_2);
    const double cos_internal_0 = cos(internal_0);
    const double cos_internal_1 = cos(internal_1);
    const double sin_internal_0 = sin(internal_0);
    const double sin_internal_1 = sin(internal_1);

    // Evaluation of Vector F
    F(0) = 0.33333333333333331 * dt * (cos_internal_0 * vr + cos_internal_2 * vr) +
           0.16666666666666666 * dt * (cos_internal_1 * vr + cos_theta * vr) + x;
    F(1) = 0.33333333333333331 * dt * (sin_internal_0 * vr + sin_internal_2 * vr) +
           0.16666666666666666 * dt * (sin_internal_1 * vr + sin_theta * vr) + y;
    F(2) = 0.66666666666666663 * dt * vr * (0.5 * dkappa * dt + kappa) +
           0.16666666666666666 * dt * (kappa * vr + vr * (dkappa * dt + kappa)) + theta;
    F(3) = dkappa * dt + kappa;
  }

  virtual void update(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, const double t_next,
                      Eigen::VectorXd& F, Eigen::MatrixXd& Fx, Eigen::MatrixXd& Fu) const override {
    // Determine stage variables
    const auto& x = states(0);
    const auto& y = states(1);
    const auto& theta = states(2);
    const auto& kappa = states(3);
    const auto& dkappa = ctrls(0);
    const auto& vr = params(4);

    // Determine internal variables
    const double dt = -t + t_next;
    const double cos_theta = cos(theta);
    const double sin_theta = sin(theta);
    const double vr_p2 = vr * vr;
    const double dt_p2 = dt * dt;
    const double internal_2 = 0.5 * dt * kappa * vr + theta;
    const double dt_p3 = dt * dt_p2;
    const double cos_internal_2 = cos(internal_2);
    const double internal_0 = 0.5 * dt * vr * (0.5 * dkappa * dt + kappa) + theta;
    const double internal_1 = dt * vr * (0.5 * dkappa * dt + kappa) + theta;
    const double sin_internal_2 = sin(internal_2);
    const double cos_internal_0 = cos(internal_0);
    const double cos_internal_1 = cos(internal_1);
    const double sin_internal_0 = sin(internal_0);
    const double sin_internal_1 = sin(internal_1);
    const double internal_3 = 0.16666666666666666 * dt * (cos_internal_1 * vr + cos_theta * vr);
    const double internal_4 = 0.33333333333333331 * dt * (cos_internal_0 * vr + cos_internal_2 * vr);
    const double internal_5 = 0.16666666666666666 * dt * (sin_internal_1 * vr + sin_theta * vr);
    const double internal_6 = 0.33333333333333331 * dt * (sin_internal_0 * vr + sin_internal_2 * vr);

    // Evaluation of Vector F
    F(0) = internal_3 + internal_4 + x;
    F(1) = internal_5 + internal_6 + y;
    F(2) = 0.66666666666666663 * dt * vr * (0.5 * dkappa * dt + kappa) +
           0.16666666666666666 * dt * (kappa * vr + vr * (dkappa * dt + kappa)) + theta;
    F(3) = dkappa * dt + kappa;

    // Evaluation of Matrix Fx
    Fx(0, 0) = 1;
    Fx(0, 2) = 0.33333333333333331 * dt * (-sin_internal_0 * vr - sin_internal_2 * vr) +
               0.16666666666666666 * dt * (-sin_internal_1 * vr - sin_theta * vr);
    Fx(0, 3) = 0.33333333333333331 * dt * (-0.5 * dt * sin_internal_0 * vr_p2 - 0.5 * dt * sin_internal_2 * vr_p2) -
               0.16666666666666666 * dt_p2 * sin_internal_1 * vr_p2;
    Fx(1, 1) = 1;
    Fx(1, 2) = internal_3 + internal_4;
    Fx(1, 3) = 0.16666666666666666 * cos_internal_1 * dt_p2 * vr_p2 +
               0.33333333333333331 * dt * (0.5 * cos_internal_0 * dt * vr_p2 + 0.5 * cos_internal_2 * dt * vr_p2);
    Fx(2, 2) = 1;
    Fx(2, 3) = dt * vr;
    Fx(3, 3) = 1;

    // Evaluation of Matrix Fu
    Fu(0, 0) =
        -0.083333333333333329 * dt_p3 * sin_internal_0 * vr_p2 - 0.083333333333333329 * dt_p3 * sin_internal_1 * vr_p2;
    Fu(1, 0) =
        0.083333333333333329 * cos_internal_0 * dt_p3 * vr_p2 + 0.083333333333333329 * cos_internal_1 * dt_p3 * vr_p2;
    Fu(2, 0) = 0.5 * dt_p2 * vr;
    Fu(3, 0) = dt;
  }

  virtual void updateFx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t,
                        const double t_next, Eigen::MatrixXd& Fx) const override {
    // Determine stage variables
    const auto& theta = states(2);
    const auto& kappa = states(3);
    const auto& dkappa = ctrls(0);
    const auto& vr = params(4);

    // Determine internal variables
    const double dt = -t + t_next;
    const double cos_theta = cos(theta);
    const double sin_theta = sin(theta);
    const double vr_p2 = vr * vr;
    const double dt_p2 = dt * dt;
    const double internal_0 = 0.5 * dt * kappa * vr + theta;
    const double cos_internal_0 = cos(internal_0);
    const double internal_1 = 0.5 * dt * vr * (0.5 * dkappa * dt + kappa) + theta;
    const double internal_2 = dt * vr * (0.5 * dkappa * dt + kappa) + theta;
    const double sin_internal_0 = sin(internal_0);
    const double cos_internal_1 = cos(internal_1);
    const double cos_internal_2 = cos(internal_2);
    const double sin_internal_1 = sin(internal_1);
    const double sin_internal_2 = sin(internal_2);

    // Evaluation of Matrix Fx
    Fx(0, 0) = 1;
    Fx(0, 2) = 0.33333333333333331 * dt * (-sin_internal_0 * vr - sin_internal_1 * vr) +
               0.16666666666666666 * dt * (-sin_internal_2 * vr - sin_theta * vr);
    Fx(0, 3) = 0.33333333333333331 * dt * (-0.5 * dt * sin_internal_0 * vr_p2 - 0.5 * dt * sin_internal_1 * vr_p2) -
               0.16666666666666666 * dt_p2 * sin_internal_2 * vr_p2;
    Fx(1, 1) = 1;
    Fx(1, 2) = 0.33333333333333331 * dt * (cos_internal_0 * vr + cos_internal_1 * vr) +
               0.16666666666666666 * dt * (cos_internal_2 * vr + cos_theta * vr);
    Fx(1, 3) = 0.16666666666666666 * cos_internal_2 * dt_p2 * vr_p2 +
               0.33333333333333331 * dt * (0.5 * cos_internal_0 * dt * vr_p2 + 0.5 * cos_internal_1 * dt * vr_p2);
    Fx(2, 2) = 1;
    Fx(2, 3) = dt * vr;
    Fx(3, 3) = 1;
  }

  virtual void updateFu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t,
                        const double t_next, Eigen::MatrixXd& Fu) const override {
    // Determine stage variables
    const auto& theta = states(2);
    const auto& kappa = states(3);
    const auto& dkappa = ctrls(0);
    const auto& vr = params(4);

    // Determine internal variables
    const double dt = -t + t_next;
    const double vr_p2 = vr * vr;
    const double dt_p2 = dt * dt;
    const double dt_p3 = dt * dt_p2;
    const double internal_0 = 0.5 * dt * vr * (0.5 * dkappa * dt + kappa) + theta;
    const double internal_1 = dt * vr * (0.5 * dkappa * dt + kappa) + theta;
    const double cos_internal_0 = cos(internal_0);
    const double cos_internal_1 = cos(internal_1);
    const double sin_internal_0 = sin(internal_0);
    const double sin_internal_1 = sin(internal_1);

    // Evaluation of Matrix Fu
    Fu(0, 0) =
        -0.083333333333333329 * dt_p3 * sin_internal_0 * vr_p2 - 0.083333333333333329 * dt_p3 * sin_internal_1 * vr_p2;
    Fu(1, 0) =
        0.083333333333333329 * cos_internal_0 * dt_p3 * vr_p2 + 0.083333333333333329 * cos_internal_1 * dt_p3 * vr_p2;
    Fu(2, 0) = 0.5 * dt_p2 * vr;
    Fu(3, 0) = dt;
  }
};

}  // namespace gpal::pnc::planning