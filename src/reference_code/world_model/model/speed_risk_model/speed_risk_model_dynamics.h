#pragma once

#include "ocp/dynamics.h"

namespace gpal::pnc::planning {

class SpeedRiskModelDynamics : public Dynamics {
 public:
  explicit SpeedRiskModelDynamics(OcpConfig::IntegratorType type = OcpConfig::ERK4) : Dynamics(3, 8, type) {}
  virtual ~SpeedRiskModelDynamics() = default;

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
class SpeedRiskModelIntegrator : public Integrator {
 public:
  SpeedRiskModelIntegrator(const Dynamics* dynamics) : Integrator(dynamics) {}
  virtual ~SpeedRiskModelIntegrator() = default;

  virtual OcpConfig::IntegratorType type() const { return OcpConfig::FORWARD_EULER; }
  virtual void update(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, const double t_next,
                      Eigen::VectorXd& F) const override {
    // Determine stage variables
    const auto& s = states(0);
    const auto& v = states(1);
    const auto& a = states(2);
    const auto& j = ctrls(0);

    // Determine internal variables
    const double dt = -t + t_next;

    // Evaluation of Vector F
    F(0) = dt * v + s;
    F(1) = a * dt + v;
    F(2) = a + dt * j;
  }

  virtual void update(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, const double t_next,
                      Eigen::VectorXd& F, Eigen::MatrixXd& Fx, Eigen::MatrixXd& Fu) const override {
    // Determine stage variables
    const auto& s = states(0);
    const auto& v = states(1);
    const auto& a = states(2);
    const auto& j = ctrls(0);

    // Determine internal variables
    const double dt = -t + t_next;

    // Evaluation of Vector F
    F(0) = dt * v + s;
    F(1) = a * dt + v;
    F(2) = a + dt * j;

    // Evaluation of Matrix Fx
    Fx(0, 0) = 1;
    Fx(0, 1) = dt;
    Fx(1, 1) = 1;
    Fx(1, 2) = dt;
    Fx(2, 2) = 1;

    // Evaluation of Matrix Fu
    Fu(2, 0) = dt;
  }

  virtual void updateFx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t,
                        const double t_next, Eigen::MatrixXd& Fx) const override {
    // Determine internal variables
    const double dt = -t + t_next;

    // Evaluation of Matrix Fx
    Fx(0, 0) = 1;
    Fx(0, 1) = dt;
    Fx(1, 1) = 1;
    Fx(1, 2) = dt;
    Fx(2, 2) = 1;
  }

  virtual void updateFu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t,
                        const double t_next, Eigen::MatrixXd& Fu) const override {
    // Determine internal variables
    const double dt = -t + t_next;

    // Evaluation of Matrix Fu
    Fu(2, 0) = dt;
  }
};

template <>
class SpeedRiskModelIntegrator<OcpConfig::ERK4> : public Integrator {
 public:
  SpeedRiskModelIntegrator(const Dynamics* dynamics) : Integrator(dynamics) {}
  virtual ~SpeedRiskModelIntegrator() = default;

  virtual OcpConfig::IntegratorType type() const { return OcpConfig::ERK4; }
  virtual void update(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, const double t_next,
                      Eigen::VectorXd& F) const override {
    // Determine stage variables
    const auto& s = states(0);
    const auto& v = states(1);
    const auto& a = states(2);
    const auto& j = ctrls(0);

    // Determine internal variables
    const double dt = -t + t_next;

    // Evaluation of Vector F
    F(0) = 0.16666666666666666 * dt * (dt * (a + 0.5 * dt * j) + 2 * v) +
           0.33333333333333331 * dt * (0.5 * a * dt + 0.5 * dt * (a + 0.5 * dt * j) + 2 * v) + s;
    F(1) = 0.5 * dt * (2 * a + dt * j) + v;
    F(2) = a + dt * j;
  }

  virtual void update(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, const double t_next,
                      Eigen::VectorXd& F, Eigen::MatrixXd& Fx, Eigen::MatrixXd& Fu) const override {
    // Determine stage variables
    const auto& s = states(0);
    const auto& v = states(1);
    const auto& a = states(2);
    const auto& j = ctrls(0);

    // Determine internal variables
    const double dt = -t + t_next;
    const double dt_p2 = dt * dt;
    const double dt_p3 = dt * dt_p2;

    // Evaluation of Vector F
    F(0) = 0.16666666666666666 * dt * (dt * (a + 0.5 * dt * j) + 2 * v) +
           0.33333333333333331 * dt * (0.5 * a * dt + 0.5 * dt * (a + 0.5 * dt * j) + 2 * v) + s;
    F(1) = 0.5 * dt * (2 * a + dt * j) + v;
    F(2) = a + dt * j;

    // Evaluation of Matrix Fx
    Fx(0, 0) = 1;
    Fx(0, 1) = dt;
    Fx(0, 2) = 0.5 * dt_p2;
    Fx(1, 1) = 1;
    Fx(1, 2) = dt;
    Fx(2, 2) = 1;

    // Evaluation of Matrix Fu
    Fu(0, 0) = 0.16666666666666666 * dt_p3;
    Fu(1, 0) = Fx(0, 2);
    Fu(2, 0) = dt;
  }

  virtual void updateFx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t,
                        const double t_next, Eigen::MatrixXd& Fx) const override {
    // Determine internal variables
    const double dt = -t + t_next;
    const double dt_p2 = dt * dt;

    // Evaluation of Matrix Fx
    Fx(0, 0) = 1;
    Fx(0, 1) = dt;
    Fx(0, 2) = 0.5 * dt_p2;
    Fx(1, 1) = 1;
    Fx(1, 2) = dt;
    Fx(2, 2) = 1;
  }

  virtual void updateFu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t,
                        const double t_next, Eigen::MatrixXd& Fu) const override {
    // Determine internal variables
    const double dt = -t + t_next;
    const double dt_p2 = dt * dt;
    const double dt_p3 = dt * dt_p2;

    // Evaluation of Matrix Fu
    Fu(0, 0) = 0.16666666666666666 * dt_p3;
    Fu(1, 0) = 0.5 * dt_p2;
    Fu(2, 0) = dt;
  }
};

}  // namespace gpal::pnc::planning