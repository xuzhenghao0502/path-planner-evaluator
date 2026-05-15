#pragma once

#include "ocp/dynamics.h"

namespace gpal::pnc::planning {

class BicycleTrajectoryTrackerDynamics : public Dynamics {
 public:
  explicit BicycleTrajectoryTrackerDynamics(OcpConfig::IntegratorType type = OcpConfig::ERK4) : Dynamics(5, 2, type) {}
  virtual ~BicycleTrajectoryTrackerDynamics() = default;

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
class BicycleTrajectoryTrackerIntegrator : public Integrator {
 public:
  BicycleTrajectoryTrackerIntegrator(const Dynamics* dynamics) : Integrator(dynamics) {}
  virtual ~BicycleTrajectoryTrackerIntegrator() = default;

  virtual OcpConfig::IntegratorType type() const { return OcpConfig::FORWARD_EULER; }
  virtual void update(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, const double t_next,
                      Eigen::VectorXd& F) const override {
    // Determine stage variables
    const auto& x = states(0);
    const auto& y = states(1);
    const auto& theta = states(2);
    const auto& v = states(3);
    const auto& kappa = states(4);
    const auto& a = ctrls(0);
    const auto& dkappa = ctrls(1);

    // Determine internal variables
    const double dt = -t + t_next;
    const double cos_theta = cos(theta);
    const double sin_theta = sin(theta);

    // Evaluation of Vector F
    F(0) = cos_theta * dt * v + x;
    F(1) = dt * sin_theta * v + y;
    F(2) = dt * kappa * v + theta;
    F(3) = a * dt + v;
    F(4) = dkappa * dt + kappa;
  }

  virtual void update(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, const double t_next,
                      Eigen::VectorXd& F, Eigen::MatrixXd& Fx, Eigen::MatrixXd& Fu) const override {
    // Determine stage variables
    const auto& x = states(0);
    const auto& y = states(1);
    const auto& theta = states(2);
    const auto& v = states(3);
    const auto& kappa = states(4);
    const auto& a = ctrls(0);
    const auto& dkappa = ctrls(1);

    // Determine internal variables
    const double dt = -t + t_next;
    const double cos_theta = cos(theta);
    const double sin_theta = sin(theta);

    // Evaluation of Vector F
    F(0) = cos_theta * dt * v + x;
    F(1) = dt * sin_theta * v + y;
    F(2) = dt * kappa * v + theta;
    F(3) = a * dt + v;
    F(4) = dkappa * dt + kappa;

    // Evaluation of Matrix Fx
    Fx(0, 0) = 1;
    Fx(0, 2) = -dt * sin_theta * v;
    Fx(0, 3) = cos_theta * dt;
    Fx(1, 1) = 1;
    Fx(1, 2) = Fx(0, 3) * v;
    Fx(1, 3) = dt * sin_theta;
    Fx(2, 2) = 1;
    Fx(2, 3) = dt * kappa;
    Fx(2, 4) = dt * v;
    Fx(3, 3) = 1;
    Fx(4, 4) = 1;

    // Evaluation of Matrix Fu
    Fu(3, 0) = dt;
    Fu(4, 1) = dt;
  }

  virtual void updateFx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t,
                        const double t_next, Eigen::MatrixXd& Fx) const override {
    // Determine stage variables
    const auto& theta = states(2);
    const auto& v = states(3);
    const auto& kappa = states(4);

    // Determine internal variables
    const double dt = -t + t_next;
    const double cos_theta = cos(theta);
    const double sin_theta = sin(theta);

    // Evaluation of Matrix Fx
    Fx(0, 0) = 1;
    Fx(0, 2) = -dt * sin_theta * v;
    Fx(0, 3) = cos_theta * dt;
    Fx(1, 1) = 1;
    Fx(1, 2) = Fx(0, 3) * v;
    Fx(1, 3) = dt * sin_theta;
    Fx(2, 2) = 1;
    Fx(2, 3) = dt * kappa;
    Fx(2, 4) = dt * v;
    Fx(3, 3) = 1;
    Fx(4, 4) = 1;
  }

  virtual void updateFu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t,
                        const double t_next, Eigen::MatrixXd& Fu) const override {
    // Determine internal variables
    const double dt = -t + t_next;

    // Evaluation of Matrix Fu
    Fu(3, 0) = dt;
    Fu(4, 1) = dt;
  }
};

template <>
class BicycleTrajectoryTrackerIntegrator<OcpConfig::ERK4> : public Integrator {
 public:
  BicycleTrajectoryTrackerIntegrator(const Dynamics* dynamics) : Integrator(dynamics) {}
  virtual ~BicycleTrajectoryTrackerIntegrator() = default;

  virtual OcpConfig::IntegratorType type() const { return OcpConfig::ERK4; }
  virtual void update(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, const double t_next,
                      Eigen::VectorXd& F) const override {
    // Determine stage variables
    const auto& x = states(0);
    const auto& y = states(1);
    const auto& theta = states(2);
    const auto& v = states(3);
    const auto& kappa = states(4);
    const auto& a = ctrls(0);
    const auto& dkappa = ctrls(1);

    // Determine internal variables
    const double dt = -t + t_next;
    const double cos_theta = cos(theta);
    const double sin_theta = sin(theta);
    const double internal_0 = 0.5 * dt * (a * dt + v) * (0.5 * dkappa * dt + kappa) + theta;
    const double internal_1 = dt * (1.5 * a * dt + v) * (0.5 * dkappa * dt + kappa) + theta;
    const double internal_2 = 0.5 * dt * kappa * (0.5 * a * dt + v) + theta;
    const double cos_internal_0 = cos(internal_0);
    const double cos_internal_1 = cos(internal_1);
    const double cos_internal_2 = cos(internal_2);
    const double sin_internal_0 = sin(internal_0);
    const double sin_internal_1 = sin(internal_1);
    const double sin_internal_2 = sin(internal_2);

    // Evaluation of Vector F
    F(0) = 0.33333333333333331 * dt * (cos_internal_0 * (0.5 * a * dt + v) + cos_internal_2 * (0.5 * a * dt + v)) +
           0.16666666666666666 * dt * (cos_internal_1 * (a * dt + v) + cos_theta * v) + x;
    F(1) = 0.33333333333333331 * dt * (sin_internal_0 * (0.5 * a * dt + v) + sin_internal_2 * (0.5 * a * dt + v)) +
           0.16666666666666666 * dt * (sin_internal_1 * (a * dt + v) + sin_theta * v) + y;
    F(2) = 0.66666666666666663 * dt * (0.5 * a * dt + v) * (0.5 * dkappa * dt + kappa) +
           0.16666666666666666 * dt * (kappa * v + (a * dt + v) * (dkappa * dt + kappa)) + theta;
    F(3) = a * dt + v;
    F(4) = dkappa * dt + kappa;
  }

  virtual void update(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, const double t_next,
                      Eigen::VectorXd& F, Eigen::MatrixXd& Fx, Eigen::MatrixXd& Fu) const override {
    // Determine stage variables
    const auto& x = states(0);
    const auto& y = states(1);
    const auto& theta = states(2);
    const auto& v = states(3);
    const auto& kappa = states(4);
    const auto& a = ctrls(0);
    const auto& dkappa = ctrls(1);

    // Determine internal variables
    const double dt = -t + t_next;
    const double cos_theta = cos(theta);
    const double sin_theta = sin(theta);
    const double dt_p2 = dt * dt;
    const double internal_3 = 0.5 * a * dt + v;
    const double dt_p3 = dt * dt_p2;
    const double internal_0 = 0.5 * dt * (a * dt + v) * (0.5 * dkappa * dt + kappa) + theta;
    const double internal_1 = dt * (1.5 * a * dt + v) * (0.5 * dkappa * dt + kappa) + theta;
    const double internal_2 = 0.5 * dt * kappa * (0.5 * a * dt + v) + theta;
    const double internal_3_p2 = internal_3 * internal_3;
    const double cos_internal_0 = cos(internal_0);
    const double cos_internal_1 = cos(internal_1);
    const double cos_internal_2 = cos(internal_2);
    const double sin_internal_0 = sin(internal_0);
    const double sin_internal_1 = sin(internal_1);
    const double sin_internal_2 = sin(internal_2);
    const double internal_4 = 0.16666666666666666 * dt * (cos_internal_1 * (a * dt + v) + cos_theta * v);
    const double internal_5 = 0.16666666666666666 * dt * (sin_internal_1 * (a * dt + v) + sin_theta * v);
    const double internal_6 = 0.33333333333333331 * dt * (cos_internal_0 * internal_3 + cos_internal_2 * internal_3);
    const double internal_7 = 0.33333333333333331 * dt * (internal_3 * sin_internal_0 + internal_3 * sin_internal_2);

    // Evaluation of Vector F
    F(0) = internal_4 + internal_6 + x;
    F(1) = internal_5 + internal_7 + y;
    F(2) = 0.66666666666666663 * dt * internal_3 * (0.5 * dkappa * dt + kappa) +
           0.16666666666666666 * dt * (kappa * v + (a * dt + v) * (dkappa * dt + kappa)) + theta;
    F(3) = a * dt + v;
    F(4) = dkappa * dt + kappa;

    // Evaluation of Matrix Fx
    Fx(0, 0) = 1;
    Fx(0, 2) = 0.16666666666666666 * dt * (-F(3, 0) * sin_internal_1 - sin_theta * v) +
               0.33333333333333331 * dt * (-internal_3 * sin_internal_0 - internal_3 * sin_internal_2);
    Fx(0, 3) = 0.16666666666666666 * dt *
                   (-F(3, 0) * dt * sin_internal_1 * (0.5 * dkappa * dt + kappa) + cos_internal_1 + cos_theta) +
               0.33333333333333331 * dt *
                   (cos_internal_0 + cos_internal_2 - 0.5 * dt * internal_3 * kappa * sin_internal_2 -
                    0.5 * dt * internal_3 * sin_internal_0 * (0.5 * dkappa * dt + kappa));
    Fx(0, 4) = -0.16666666666666666 * F(3, 0) * dt_p2 * sin_internal_1 * (1.5 * a * dt + v) +
               0.33333333333333331 * dt *
                   (-0.5 * F(3, 0) * dt * internal_3 * sin_internal_0 - 0.5 * dt * internal_3_p2 * sin_internal_2);
    Fx(1, 1) = 1;
    Fx(1, 2) = internal_4 + internal_6;
    Fx(1, 3) = 0.16666666666666666 * dt *
                   (F(3, 0) * cos_internal_1 * dt * (0.5 * dkappa * dt + kappa) + sin_internal_1 + sin_theta) +
               0.33333333333333331 * dt *
                   (0.5 * cos_internal_0 * dt * internal_3 * (0.5 * dkappa * dt + kappa) +
                    0.5 * cos_internal_2 * dt * internal_3 * kappa + sin_internal_0 + sin_internal_2);
    Fx(1, 4) = 0.16666666666666666 * F(3, 0) * cos_internal_1 * dt_p2 * (1.5 * a * dt + v) +
               0.33333333333333331 * dt *
                   (0.5 * F(3, 0) * cos_internal_0 * dt * internal_3 + 0.5 * cos_internal_2 * dt * internal_3_p2);
    Fx(2, 2) = 1;
    Fx(2, 3) =
        0.66666666666666663 * dt * (0.5 * dkappa * dt + kappa) + 0.16666666666666666 * dt * (dkappa * dt + 2 * kappa);
    Fx(2, 4) = 0.66666666666666663 * dt * internal_3 + 0.16666666666666666 * dt * (a * dt + 2 * v);
    Fx(3, 3) = 1;
    Fx(4, 4) = 1;

    // Evaluation of Matrix Fu
    Fu(0, 0) = 0.16666666666666666 * dt *
                   (-1.5 * F(3, 0) * dt_p2 * sin_internal_1 * (0.5 * dkappa * dt + kappa) + cos_internal_1 * dt) +
               0.33333333333333331 * dt *
                   (0.5 * cos_internal_0 * dt + 0.5 * cos_internal_2 * dt -
                    0.25 * dt_p2 * internal_3 * kappa * sin_internal_2 -
                    0.5 * dt_p2 * internal_3 * sin_internal_0 * (0.5 * dkappa * dt + kappa));
    Fu(0, 1) = -0.083333333333333329 * F(3, 0) * dt_p3 * internal_3 * sin_internal_0 -
               0.083333333333333329 * F(3, 0) * dt_p3 * sin_internal_1 * (1.5 * a * dt + v);
    Fu(1, 0) = 0.16666666666666666 * dt *
                   (1.5 * F(3, 0) * cos_internal_1 * dt_p2 * (0.5 * dkappa * dt + kappa) + dt * sin_internal_1) +
               0.33333333333333331 * dt *
                   (0.5 * cos_internal_0 * dt_p2 * internal_3 * (0.5 * dkappa * dt + kappa) +
                    0.25 * cos_internal_2 * dt_p2 * internal_3 * kappa + 0.5 * dt * sin_internal_0 +
                    0.5 * dt * sin_internal_2);
    Fu(1, 1) = 0.083333333333333329 * F(3, 0) * cos_internal_0 * dt_p3 * internal_3 +
               0.083333333333333329 * F(3, 0) * cos_internal_1 * dt_p3 * (1.5 * a * dt + v);
    Fu(2, 0) = 0.16666666666666666 * F(4, 0) * dt_p2 + 0.33333333333333331 * dt_p2 * (0.5 * dkappa * dt + kappa);
    Fu(2, 1) = 0.16666666666666666 * F(3, 0) * dt_p2 + 0.33333333333333331 * dt_p2 * internal_3;
    Fu(3, 0) = dt;
    Fu(4, 1) = dt;
  }

  virtual void updateFx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t,
                        const double t_next, Eigen::MatrixXd& Fx) const override {
    // Determine stage variables
    const auto& theta = states(2);
    const auto& v = states(3);
    const auto& kappa = states(4);
    const auto& a = ctrls(0);
    const auto& dkappa = ctrls(1);

    // Determine internal variables
    const double dt = -t + t_next;
    const double cos_theta = cos(theta);
    const double sin_theta = sin(theta);
    const double dt_p2 = dt * dt;
    const double internal_3 = 0.5 * a * dt + v;
    const double internal_0 = dt * (1.5 * a * dt + v) * (0.5 * dkappa * dt + kappa) + theta;
    const double internal_1 = 0.5 * dt * kappa * (0.5 * a * dt + v) + theta;
    const double internal_2 = 0.5 * dt * (a * dt + v) * (0.5 * dkappa * dt + kappa) + theta;
    const double internal_3_p2 = internal_3 * internal_3;
    const double cos_internal_0 = cos(internal_0);
    const double cos_internal_1 = cos(internal_1);
    const double cos_internal_2 = cos(internal_2);
    const double sin_internal_0 = sin(internal_0);
    const double sin_internal_1 = sin(internal_1);
    const double sin_internal_2 = sin(internal_2);

    // Evaluation of Matrix Fx
    Fx(0, 0) = 1;
    Fx(0, 2) = 0.33333333333333331 * dt * (-internal_3 * sin_internal_1 - internal_3 * sin_internal_2) +
               0.16666666666666666 * dt * (-sin_internal_0 * (a * dt + v) - sin_theta * v);
    Fx(0, 3) = 0.16666666666666666 * dt *
                   (cos_internal_0 + cos_theta - dt * sin_internal_0 * (a * dt + v) * (0.5 * dkappa * dt + kappa)) +
               0.33333333333333331 * dt *
                   (cos_internal_1 + cos_internal_2 - 0.5 * dt * internal_3 * kappa * sin_internal_1 -
                    0.5 * dt * internal_3 * sin_internal_2 * (0.5 * dkappa * dt + kappa));
    Fx(0, 4) =
        0.33333333333333331 * dt *
            (-0.5 * dt * internal_3 * sin_internal_2 * (a * dt + v) - 0.5 * dt * internal_3_p2 * sin_internal_1) -
        0.16666666666666666 * dt_p2 * sin_internal_0 * (a * dt + v) * (1.5 * a * dt + v);
    Fx(1, 1) = 1;
    Fx(1, 2) = 0.16666666666666666 * dt * (cos_internal_0 * (a * dt + v) + cos_theta * v) +
               0.33333333333333331 * dt * (cos_internal_1 * internal_3 + cos_internal_2 * internal_3);
    Fx(1, 3) =
        0.16666666666666666 * dt *
            (cos_internal_0 * dt * (a * dt + v) * (0.5 * dkappa * dt + kappa) + sin_internal_0 + sin_theta) +
        0.33333333333333331 * dt *
            (0.5 * cos_internal_1 * dt * internal_3 * kappa +
             0.5 * cos_internal_2 * dt * internal_3 * (0.5 * dkappa * dt + kappa) + sin_internal_1 + sin_internal_2);
    Fx(1, 4) = 0.16666666666666666 * cos_internal_0 * dt_p2 * (a * dt + v) * (1.5 * a * dt + v) +
               0.33333333333333331 * dt *
                   (0.5 * cos_internal_1 * dt * internal_3_p2 + 0.5 * cos_internal_2 * dt * internal_3 * (a * dt + v));
    Fx(2, 2) = 1;
    Fx(2, 3) =
        0.66666666666666663 * dt * (0.5 * dkappa * dt + kappa) + 0.16666666666666666 * dt * (dkappa * dt + 2 * kappa);
    Fx(2, 4) = 0.66666666666666663 * dt * internal_3 + 0.16666666666666666 * dt * (a * dt + 2 * v);
    Fx(3, 3) = 1;
    Fx(4, 4) = 1;
  }

  virtual void updateFu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t,
                        const double t_next, Eigen::MatrixXd& Fu) const override {
    // Determine stage variables
    const auto& theta = states(2);
    const auto& v = states(3);
    const auto& kappa = states(4);
    const auto& a = ctrls(0);
    const auto& dkappa = ctrls(1);

    // Determine internal variables
    const double dt = -t + t_next;
    const double dt_p2 = dt * dt;
    const double dt_p3 = dt * dt_p2;
    const double internal_0 = dt * (1.5 * a * dt + v) * (0.5 * dkappa * dt + kappa) + theta;
    const double internal_1 = 0.5 * dt * kappa * (0.5 * a * dt + v) + theta;
    const double internal_2 = 0.5 * dt * (a * dt + v) * (0.5 * dkappa * dt + kappa) + theta;
    const double cos_internal_0 = cos(internal_0);
    const double cos_internal_1 = cos(internal_1);
    const double cos_internal_2 = cos(internal_2);
    const double sin_internal_0 = sin(internal_0);
    const double sin_internal_1 = sin(internal_1);
    const double sin_internal_2 = sin(internal_2);

    // Evaluation of Matrix Fu
    Fu(0, 0) = 0.16666666666666666 * dt *
                   (cos_internal_0 * dt - 1.5 * dt_p2 * sin_internal_0 * (a * dt + v) * (0.5 * dkappa * dt + kappa)) +
               0.33333333333333331 * dt *
                   (0.5 * cos_internal_1 * dt + 0.5 * cos_internal_2 * dt -
                    0.25 * dt_p2 * kappa * sin_internal_1 * (0.5 * a * dt + v) -
                    0.5 * dt_p2 * sin_internal_2 * (0.5 * a * dt + v) * (0.5 * dkappa * dt + kappa));
    Fu(0, 1) = -0.083333333333333329 * dt_p3 * sin_internal_0 * (a * dt + v) * (1.5 * a * dt + v) -
               0.083333333333333329 * dt_p3 * sin_internal_2 * (0.5 * a * dt + v) * (a * dt + v);
    Fu(1, 0) = 0.16666666666666666 * dt *
                   (1.5 * cos_internal_0 * dt_p2 * (a * dt + v) * (0.5 * dkappa * dt + kappa) + dt * sin_internal_0) +
               0.33333333333333331 * dt *
                   (0.25 * cos_internal_1 * dt_p2 * kappa * (0.5 * a * dt + v) +
                    0.5 * cos_internal_2 * dt_p2 * (0.5 * a * dt + v) * (0.5 * dkappa * dt + kappa) +
                    0.5 * dt * sin_internal_1 + 0.5 * dt * sin_internal_2);
    Fu(1, 1) = 0.083333333333333329 * cos_internal_0 * dt_p3 * (a * dt + v) * (1.5 * a * dt + v) +
               0.083333333333333329 * cos_internal_2 * dt_p3 * (0.5 * a * dt + v) * (a * dt + v);
    Fu(2, 0) =
        0.33333333333333331 * dt_p2 * (0.5 * dkappa * dt + kappa) + 0.16666666666666666 * dt_p2 * (dkappa * dt + kappa);
    Fu(2, 1) = 0.33333333333333331 * dt_p2 * (0.5 * a * dt + v) + 0.16666666666666666 * dt_p2 * (a * dt + v);
    Fu(3, 0) = dt;
    Fu(4, 1) = dt;
  }
};

}  // namespace gpal::pnc::planning