#pragma once

#include "ocp/dynamics.h"

namespace gpal::pnc::planning {

class ParkLateralGeneralDynamics : public Dynamics {
 public:
  explicit ParkLateralGeneralDynamics(OcpConfig::IntegratorType type = OcpConfig::ERK4) : Dynamics(5, 2, type) {}
  virtual ~ParkLateralGeneralDynamics() = default;

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
class ParkLateralGeneralIntegrator : public Integrator {
 public:
  ParkLateralGeneralIntegrator(const Dynamics* dynamics) : Integrator(dynamics) {}
  virtual ~ParkLateralGeneralIntegrator() = default;

  virtual OcpConfig::IntegratorType type() const { return OcpConfig::FORWARD_EULER; }
  virtual void update(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, const double t_next,
                      Eigen::VectorXd& F) const override {
    // Determine stage variables
    const auto& s = states(0);
    const auto& x = states(1);
    const auto& y = states(2);
    const auto& theta = states(3);
    const auto& kappa = states(4);
    const auto& dkappa = ctrls(0);
    const auto& vr = params(4);

    // Determine global variables
    const auto& cos_theta = globals(0);
    const auto& sin_theta = globals(1);
    const auto& ds = globals(13);

    // Determine internal variables
    const double dt = -t + t_next;

    // Evaluation of Vector F
    F(0) = ds * dt * vr + s;
    F(1) = cos_theta * dt * vr + x;
    F(2) = dt * sin_theta * vr + y;
    F(3) = dt * kappa * vr + theta;
    F(4) = dkappa * dt + kappa;
  }

  virtual void update(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, const double t_next,
                      Eigen::VectorXd& F, Eigen::MatrixXd& Fx, Eigen::MatrixXd& Fu) const override {
    // Determine stage variables
    const auto& s = states(0);
    const auto& x = states(1);
    const auto& y = states(2);
    const auto& theta = states(3);
    const auto& kappa = states(4);
    const auto& dkappa = ctrls(0);
    const auto& kr = params(3);
    const auto& vr = params(4);

    // Determine global variables
    const auto& cos_theta = globals(0);
    const auto& sin_theta = globals(1);
    const auto& anonymous_0 = globals(2);
    const auto& cos_thetar = globals(3);
    const auto& sin_thetar = globals(4);
    const auto& l = globals(10);
    const auto& ds = globals(13);

    // Determine internal variables
    const double dt = -t + t_next;
    const double sin_anonymous_0 = sin(anonymous_0);
    const double internal_0 = -kr * l + 1;

    // Evaluation of Vector F
    F(0) = ds * dt * vr + s;
    F(1) = cos_theta * dt * vr + x;
    F(2) = dt * sin_theta * vr + y;
    F(3) = dt * kappa * vr + theta;
    F(4) = dkappa * dt + kappa;

    // Evaluation of Matrix Fx
    Fx(0, 0) = 1;
    Fx(0, 1) = -ds * dt * kr * sin_thetar * vr / internal_0;
    Fx(0, 2) = cos_thetar * ds * dt * kr * vr / internal_0;
    Fx(0, 3) = -dt * sin_anonymous_0 * vr / internal_0;
    Fx(1, 1) = 1;
    Fx(1, 3) = -dt * sin_theta * vr;
    Fx(2, 2) = 1;
    Fx(2, 3) = cos_theta * dt * vr;
    Fx(3, 3) = 1;
    Fx(3, 4) = dt * vr;
    Fx(4, 4) = 1;

    // Evaluation of Matrix Fu
    Fu(4, 0) = dt;
  }

  virtual void updateFx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t,
                        const double t_next, Eigen::MatrixXd& Fx) const override {
    // Determine stage variables
    const auto& kr = params(3);
    const auto& vr = params(4);

    // Determine global variables
    const auto& cos_theta = globals(0);
    const auto& sin_theta = globals(1);
    const auto& anonymous_0 = globals(2);
    const auto& cos_thetar = globals(3);
    const auto& sin_thetar = globals(4);
    const auto& l = globals(10);
    const auto& ds = globals(13);

    // Determine internal variables
    const double dt = -t + t_next;
    const double sin_anonymous_0 = sin(anonymous_0);
    const double internal_0 = -kr * l + 1;

    // Evaluation of Matrix Fx
    Fx(0, 0) = 1;
    Fx(0, 1) = -ds * dt * kr * sin_thetar * vr / internal_0;
    Fx(0, 2) = cos_thetar * ds * dt * kr * vr / internal_0;
    Fx(0, 3) = -dt * sin_anonymous_0 * vr / internal_0;
    Fx(1, 1) = 1;
    Fx(1, 3) = -dt * sin_theta * vr;
    Fx(2, 2) = 1;
    Fx(2, 3) = cos_theta * dt * vr;
    Fx(3, 3) = 1;
    Fx(3, 4) = dt * vr;
    Fx(4, 4) = 1;
  }

  virtual void updateFu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t,
                        const double t_next, Eigen::MatrixXd& Fu) const override {
    // Determine internal variables
    const double dt = -t + t_next;

    // Evaluation of Matrix Fu
    Fu(4, 0) = dt;
  }
};

template <>
class ParkLateralGeneralIntegrator<OcpConfig::ERK4> : public Integrator {
 public:
  ParkLateralGeneralIntegrator(const Dynamics* dynamics) : Integrator(dynamics) {}
  virtual ~ParkLateralGeneralIntegrator() = default;

  virtual OcpConfig::IntegratorType type() const { return OcpConfig::ERK4; }
  virtual void update(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, const double t_next,
                      Eigen::VectorXd& F) const override {
    // Determine stage variables
    const auto& s = states(0);
    const auto& x = states(1);
    const auto& y = states(2);
    const auto& theta = states(3);
    const auto& kappa = states(4);
    const auto& dkappa = ctrls(0);
    const auto& xr = params(0);
    const auto& yr = params(1);
    const auto& kr = params(3);
    const auto& vr = params(4);

    // Determine global variables
    const auto& cos_theta = globals(0);
    const auto& sin_theta = globals(1);
    const auto& anonymous_0 = globals(2);
    const auto& cos_thetar = globals(3);
    const auto& sin_thetar = globals(4);
    const auto& ds = globals(13);

    // Determine internal variables
    const double dt = -t + t_next;
    const double internal_1 = 0.5 * dt * kappa * vr + theta;
    const double internal_4 = anonymous_0 + 0.5 * dt * kappa * vr;
    const double cos_internal_1 = cos(internal_1);
    const double cos_internal_4 = cos(internal_4);
    const double internal_0 = 0.5 * dt * vr * (0.5 * dkappa * dt + kappa) + theta;
    const double internal_2 = anonymous_0 + dt * vr * (0.5 * dkappa * dt + kappa);
    const double internal_3 = anonymous_0 + 0.5 * dt * vr * (0.5 * dkappa * dt + kappa);
    const double internal_5 = dt * vr * (0.5 * dkappa * dt + kappa) + theta;
    const double sin_internal_1 = sin(internal_1);
    const double cos_internal_0 = cos(internal_0);
    const double cos_internal_2 = cos(internal_2);
    const double cos_internal_3 = cos(internal_3);
    const double cos_internal_5 = cos(internal_5);
    const double sin_internal_0 = sin(internal_0);
    const double sin_internal_5 = sin(internal_5);

    // Evaluation of Vector F
    F(0) = 0.16666666666666666 * dt *
               (cos_internal_2 * vr /
                    (-kr * (cos_thetar * (dt * sin_internal_0 * vr + y - yr) -
                            sin_thetar * (cos_internal_0 * dt * vr + x - xr)) +
                     1) +
                ds * vr) +
           0.33333333333333331 * dt *
               (cos_internal_3 * vr /
                    (-kr * (cos_thetar * (0.5 * dt * sin_internal_1 * vr + y - yr) -
                            sin_thetar * (0.5 * cos_internal_1 * dt * vr + x - xr)) +
                     1) +
                cos_internal_4 * vr /
                    (-kr * (cos_thetar * (0.5 * dt * sin_theta * vr + y - yr) -
                            sin_thetar * (0.5 * cos_theta * dt * vr + x - xr)) +
                     1)) +
           s;
    F(1) = 0.33333333333333331 * dt * (cos_internal_0 * vr + cos_internal_1 * vr) +
           0.16666666666666666 * dt * (cos_internal_5 * vr + cos_theta * vr) + x;
    F(2) = 0.33333333333333331 * dt * (sin_internal_0 * vr + sin_internal_1 * vr) +
           0.16666666666666666 * dt * (sin_internal_5 * vr + sin_theta * vr) + y;
    F(3) = 0.66666666666666663 * dt * vr * (0.5 * dkappa * dt + kappa) +
           0.16666666666666666 * dt * (kappa * vr + vr * (dkappa * dt + kappa)) + theta;
    F(4) = dkappa * dt + kappa;
  }

  virtual void update(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, const double t_next,
                      Eigen::VectorXd& F, Eigen::MatrixXd& Fx, Eigen::MatrixXd& Fu) const override {
    // Determine stage variables
    const auto& s = states(0);
    const auto& x = states(1);
    const auto& y = states(2);
    const auto& theta = states(3);
    const auto& kappa = states(4);
    const auto& dkappa = ctrls(0);
    const auto& xr = params(0);
    const auto& yr = params(1);
    const auto& kr = params(3);
    const auto& vr = params(4);

    // Determine global variables
    const auto& cos_theta = globals(0);
    const auto& sin_theta = globals(1);
    const auto& anonymous_0 = globals(2);
    const auto& cos_thetar = globals(3);
    const auto& sin_thetar = globals(4);
    const auto& l = globals(10);
    const auto& ds = globals(13);

    // Determine internal variables
    const double dt = -t + t_next;
    const double vr_p2 = vr * vr;
    const double sin_anonymous_0 = sin(anonymous_0);
    const double dt_p2 = dt * dt;
    const double internal_1 = 0.5 * dt * kappa * vr + theta;
    const double internal_4 = anonymous_0 + 0.5 * dt * kappa * vr;
    const double dt_p3 = dt * dt_p2;
    const double cos_internal_1 = cos(internal_1);
    const double cos_internal_4 = cos(internal_4);
    const double internal_0 = 0.5 * dt * vr * (0.5 * dkappa * dt + kappa) + theta;
    const double internal_2 = anonymous_0 + dt * vr * (0.5 * dkappa * dt + kappa);
    const double internal_3 = anonymous_0 + 0.5 * dt * vr * (0.5 * dkappa * dt + kappa);
    const double internal_5 = dt * vr * (0.5 * dkappa * dt + kappa) + theta;
    const double sin_internal_1 = sin(internal_1);
    const double sin_internal_4 = sin(internal_4);
    const double cos_internal_0 = cos(internal_0);
    const double cos_internal_2 = cos(internal_2);
    const double cos_internal_3 = cos(internal_3);
    const double cos_internal_5 = cos(internal_5);
    const double internal_6 =
        -kr * (cos_thetar * (0.5 * dt * sin_theta * vr + y - yr) - sin_thetar * (0.5 * cos_theta * dt * vr + x - xr)) +
        1;
    const double sin_internal_0 = sin(internal_0);
    const double sin_internal_2 = sin(internal_2);
    const double sin_internal_3 = sin(internal_3);
    const double sin_internal_5 = sin(internal_5);
    const double internal_6_p2 = internal_6 * internal_6;
    const double internal_10 = 0.33333333333333331 * dt * (cos_internal_0 * vr + cos_internal_1 * vr);
    const double internal_11 = 0.16666666666666666 * dt * (sin_internal_5 * vr + sin_theta * vr);
    const double internal_12 = 0.33333333333333331 * dt * (sin_internal_0 * vr + sin_internal_1 * vr);
    const double internal_9 = 0.16666666666666666 * dt * (cos_internal_5 * vr + cos_theta * vr);
    const double internal_7 = -kr * (cos_thetar * (0.5 * dt * sin_internal_1 * vr + y - yr) -
                                     sin_thetar * (0.5 * cos_internal_1 * dt * vr + x - xr)) +
                              1;
    const double internal_7_p2 = internal_7 * internal_7;
    const double internal_8 =
        -kr * (cos_thetar * (dt * sin_internal_0 * vr + y - yr) - sin_thetar * (cos_internal_0 * dt * vr + x - xr)) + 1;
    const double internal_8_p2 = internal_8 * internal_8;

    // Evaluation of Vector F
    F(0) = 0.16666666666666666 * dt * (cos_internal_2 * vr / internal_8 + ds * vr) +
           0.33333333333333331 * dt * (cos_internal_3 * vr / internal_7 + cos_internal_4 * vr / internal_6) + s;
    F(1) = internal_10 + internal_9 + x;
    F(2) = internal_11 + internal_12 + y;
    F(3) = 0.66666666666666663 * dt * vr * (0.5 * dkappa * dt + kappa) +
           0.16666666666666666 * dt * (kappa * vr + vr * (dkappa * dt + kappa)) + theta;
    F(4) = dkappa * dt + kappa;

    // Evaluation of Matrix Fx
    Fx(0, 0) = 1;
    Fx(0, 1) =
        0.16666666666666666 * dt *
            (-cos_internal_2 * kr * sin_thetar * vr / internal_8_p2 - ds * kr * sin_thetar * vr / (-kr * l + 1)) +
        0.33333333333333331 * dt *
            (-cos_internal_3 * kr * sin_thetar * vr / internal_7_p2 -
             cos_internal_4 * kr * sin_thetar * vr / internal_6_p2);
    Fx(0, 2) = 0.16666666666666666 * dt *
                   (cos_internal_2 * cos_thetar * kr * vr / internal_8_p2 + cos_thetar * ds * kr * vr / (-kr * l + 1)) +
               0.33333333333333331 * dt *
                   (cos_internal_3 * cos_thetar * kr * vr / internal_7_p2 +
                    cos_internal_4 * cos_thetar * kr * vr / internal_6_p2);
    Fx(0, 3) =
        0.16666666666666666 * dt *
            (cos_internal_2 * kr * vr *
                 (cos_internal_0 * cos_thetar * dt * vr + dt * sin_internal_0 * sin_thetar * vr) / internal_8_p2 -
             sin_anonymous_0 * vr / (-kr * l + 1) - sin_internal_2 * vr / internal_8) +
        0.33333333333333331 * dt *
            (cos_internal_3 * kr * vr *
                 (0.5 * cos_internal_1 * cos_thetar * dt * vr + 0.5 * dt * sin_internal_1 * sin_thetar * vr) /
                 internal_7_p2 +
             cos_internal_4 * kr * vr *
                 (0.5 * cos_theta * cos_thetar * dt * vr + 0.5 * dt * sin_theta * sin_thetar * vr) / internal_6_p2 -
             sin_internal_3 * vr / internal_7 - sin_internal_4 * vr / internal_6);
    Fx(0, 4) = 0.16666666666666666 * dt *
                   (cos_internal_2 * kr * vr *
                        (0.5 * cos_internal_0 * cos_thetar * dt_p2 * vr_p2 +
                         0.5 * dt_p2 * sin_internal_0 * sin_thetar * vr_p2) /
                        internal_8_p2 -
                    dt * sin_internal_2 * vr_p2 / internal_8) +
               0.33333333333333331 * dt *
                   (cos_internal_3 * kr * vr *
                        (0.25 * cos_internal_1 * cos_thetar * dt_p2 * vr_p2 +
                         0.25 * dt_p2 * sin_internal_1 * sin_thetar * vr_p2) /
                        internal_7_p2 -
                    0.5 * dt * sin_internal_3 * vr_p2 / internal_7 - 0.5 * dt * sin_internal_4 * vr_p2 / internal_6);
    Fx(1, 1) = 1;
    Fx(1, 3) = 0.33333333333333331 * dt * (-sin_internal_0 * vr - sin_internal_1 * vr) +
               0.16666666666666666 * dt * (-sin_internal_5 * vr - sin_theta * vr);
    Fx(1, 4) = 0.33333333333333331 * dt * (-0.5 * dt * sin_internal_0 * vr_p2 - 0.5 * dt * sin_internal_1 * vr_p2) -
               0.16666666666666666 * dt_p2 * sin_internal_5 * vr_p2;
    Fx(2, 2) = 1;
    Fx(2, 3) = internal_10 + internal_9;
    Fx(2, 4) = 0.16666666666666666 * cos_internal_5 * dt_p2 * vr_p2 +
               0.33333333333333331 * dt * (0.5 * cos_internal_0 * dt * vr_p2 + 0.5 * cos_internal_1 * dt * vr_p2);
    Fx(3, 3) = 1;
    Fx(3, 4) = dt * vr;
    Fx(4, 4) = 1;

    // Evaluation of Matrix Fu
    Fu(0, 0) = 0.16666666666666666 * dt *
                   (cos_internal_2 * kr * vr *
                        (0.25 * cos_internal_0 * cos_thetar * dt_p3 * vr_p2 +
                         0.25 * dt_p3 * sin_internal_0 * sin_thetar * vr_p2) /
                        internal_8_p2 -
                    0.5 * dt_p2 * sin_internal_2 * vr_p2 / internal_8) -
               0.083333333333333329 * dt_p3 * sin_internal_3 * vr_p2 / internal_7;
    Fu(1, 0) =
        -0.083333333333333329 * dt_p3 * sin_internal_0 * vr_p2 - 0.083333333333333329 * dt_p3 * sin_internal_5 * vr_p2;
    Fu(2, 0) =
        0.083333333333333329 * cos_internal_0 * dt_p3 * vr_p2 + 0.083333333333333329 * cos_internal_5 * dt_p3 * vr_p2;
    Fu(3, 0) = 0.5 * dt_p2 * vr;
    Fu(4, 0) = dt;
  }

  virtual void updateFx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t,
                        const double t_next, Eigen::MatrixXd& Fx) const override {
    // Determine stage variables
    const auto& x = states(1);
    const auto& y = states(2);
    const auto& theta = states(3);
    const auto& kappa = states(4);
    const auto& dkappa = ctrls(0);
    const auto& xr = params(0);
    const auto& yr = params(1);
    const auto& kr = params(3);
    const auto& vr = params(4);

    // Determine global variables
    const auto& cos_theta = globals(0);
    const auto& sin_theta = globals(1);
    const auto& anonymous_0 = globals(2);
    const auto& cos_thetar = globals(3);
    const auto& sin_thetar = globals(4);
    const auto& l = globals(10);
    const auto& ds = globals(13);

    // Determine internal variables
    const double dt = -t + t_next;
    const double vr_p2 = vr * vr;
    const double sin_anonymous_0 = sin(anonymous_0);
    const double dt_p2 = dt * dt;
    const double internal_1 = 0.5 * dt * kappa * vr + theta;
    const double internal_4 = anonymous_0 + 0.5 * dt * kappa * vr;
    const double cos_internal_1 = cos(internal_1);
    const double cos_internal_4 = cos(internal_4);
    const double internal_0 = 0.5 * dt * vr * (0.5 * dkappa * dt + kappa) + theta;
    const double internal_2 = anonymous_0 + dt * vr * (0.5 * dkappa * dt + kappa);
    const double internal_3 = anonymous_0 + 0.5 * dt * vr * (0.5 * dkappa * dt + kappa);
    const double internal_8 = dt * vr * (0.5 * dkappa * dt + kappa) + theta;
    const double sin_internal_1 = sin(internal_1);
    const double sin_internal_4 = sin(internal_4);
    const double cos_internal_0 = cos(internal_0);
    const double cos_internal_2 = cos(internal_2);
    const double cos_internal_3 = cos(internal_3);
    const double cos_internal_8 = cos(internal_8);
    const double internal_5 =
        -kr * (cos_thetar * (0.5 * dt * sin_theta * vr + y - yr) - sin_thetar * (0.5 * cos_theta * dt * vr + x - xr)) +
        1;
    const double sin_internal_0 = sin(internal_0);
    const double sin_internal_2 = sin(internal_2);
    const double sin_internal_3 = sin(internal_3);
    const double sin_internal_8 = sin(internal_8);
    const double internal_5_p2 = internal_5 * internal_5;
    const double internal_6 = -kr * (cos_thetar * (0.5 * dt * sin_internal_1 * vr + y - yr) -
                                     sin_thetar * (0.5 * cos_internal_1 * dt * vr + x - xr)) +
                              1;
    const double internal_6_p2 = internal_6 * internal_6;
    const double internal_7 =
        -kr * (cos_thetar * (dt * sin_internal_0 * vr + y - yr) - sin_thetar * (cos_internal_0 * dt * vr + x - xr)) + 1;
    const double internal_7_p2 = internal_7 * internal_7;

    // Evaluation of Matrix Fx
    Fx(0, 0) = 1;
    Fx(0, 1) =
        0.16666666666666666 * dt *
            (-cos_internal_2 * kr * sin_thetar * vr / internal_7_p2 - ds * kr * sin_thetar * vr / (-kr * l + 1)) +
        0.33333333333333331 * dt *
            (-cos_internal_3 * kr * sin_thetar * vr / internal_6_p2 -
             cos_internal_4 * kr * sin_thetar * vr / internal_5_p2);
    Fx(0, 2) = 0.16666666666666666 * dt *
                   (cos_internal_2 * cos_thetar * kr * vr / internal_7_p2 + cos_thetar * ds * kr * vr / (-kr * l + 1)) +
               0.33333333333333331 * dt *
                   (cos_internal_3 * cos_thetar * kr * vr / internal_6_p2 +
                    cos_internal_4 * cos_thetar * kr * vr / internal_5_p2);
    Fx(0, 3) =
        0.16666666666666666 * dt *
            (cos_internal_2 * kr * vr *
                 (cos_internal_0 * cos_thetar * dt * vr + dt * sin_internal_0 * sin_thetar * vr) / internal_7_p2 -
             sin_anonymous_0 * vr / (-kr * l + 1) - sin_internal_2 * vr / internal_7) +
        0.33333333333333331 * dt *
            (cos_internal_3 * kr * vr *
                 (0.5 * cos_internal_1 * cos_thetar * dt * vr + 0.5 * dt * sin_internal_1 * sin_thetar * vr) /
                 internal_6_p2 +
             cos_internal_4 * kr * vr *
                 (0.5 * cos_theta * cos_thetar * dt * vr + 0.5 * dt * sin_theta * sin_thetar * vr) / internal_5_p2 -
             sin_internal_3 * vr / internal_6 - sin_internal_4 * vr / internal_5);
    Fx(0, 4) = 0.16666666666666666 * dt *
                   (cos_internal_2 * kr * vr *
                        (0.5 * cos_internal_0 * cos_thetar * dt_p2 * vr_p2 +
                         0.5 * dt_p2 * sin_internal_0 * sin_thetar * vr_p2) /
                        internal_7_p2 -
                    dt * sin_internal_2 * vr_p2 / internal_7) +
               0.33333333333333331 * dt *
                   (cos_internal_3 * kr * vr *
                        (0.25 * cos_internal_1 * cos_thetar * dt_p2 * vr_p2 +
                         0.25 * dt_p2 * sin_internal_1 * sin_thetar * vr_p2) /
                        internal_6_p2 -
                    0.5 * dt * sin_internal_3 * vr_p2 / internal_6 - 0.5 * dt * sin_internal_4 * vr_p2 / internal_5);
    Fx(1, 1) = 1;
    Fx(1, 3) = 0.33333333333333331 * dt * (-sin_internal_0 * vr - sin_internal_1 * vr) +
               0.16666666666666666 * dt * (-sin_internal_8 * vr - sin_theta * vr);
    Fx(1, 4) = 0.33333333333333331 * dt * (-0.5 * dt * sin_internal_0 * vr_p2 - 0.5 * dt * sin_internal_1 * vr_p2) -
               0.16666666666666666 * dt_p2 * sin_internal_8 * vr_p2;
    Fx(2, 2) = 1;
    Fx(2, 3) = 0.33333333333333331 * dt * (cos_internal_0 * vr + cos_internal_1 * vr) +
               0.16666666666666666 * dt * (cos_internal_8 * vr + cos_theta * vr);
    Fx(2, 4) = 0.16666666666666666 * cos_internal_8 * dt_p2 * vr_p2 +
               0.33333333333333331 * dt * (0.5 * cos_internal_0 * dt * vr_p2 + 0.5 * cos_internal_1 * dt * vr_p2);
    Fx(3, 3) = 1;
    Fx(3, 4) = dt * vr;
    Fx(4, 4) = 1;
  }

  virtual void updateFu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t,
                        const double t_next, Eigen::MatrixXd& Fu) const override {
    // Determine stage variables
    const auto& x = states(1);
    const auto& y = states(2);
    const auto& theta = states(3);
    const auto& kappa = states(4);
    const auto& dkappa = ctrls(0);
    const auto& xr = params(0);
    const auto& yr = params(1);
    const auto& kr = params(3);
    const auto& vr = params(4);

    // Determine global variables
    const auto& anonymous_0 = globals(2);
    const auto& cos_thetar = globals(3);
    const auto& sin_thetar = globals(4);

    // Determine internal variables
    const double dt = -t + t_next;
    const double vr_p2 = vr * vr;
    const double dt_p2 = dt * dt;
    const double internal_1 = 0.5 * dt * kappa * vr + theta;
    const double dt_p3 = dt * dt_p2;
    const double cos_internal_1 = cos(internal_1);
    const double internal_0 = anonymous_0 + dt * vr * (0.5 * dkappa * dt + kappa);
    const double internal_2 = 0.5 * dt * vr * (0.5 * dkappa * dt + kappa) + theta;
    const double internal_3 = anonymous_0 + 0.5 * dt * vr * (0.5 * dkappa * dt + kappa);
    const double internal_5 = dt * vr * (0.5 * dkappa * dt + kappa) + theta;
    const double sin_internal_1 = sin(internal_1);
    const double cos_internal_0 = cos(internal_0);
    const double cos_internal_2 = cos(internal_2);
    const double cos_internal_5 = cos(internal_5);
    const double sin_internal_0 = sin(internal_0);
    const double sin_internal_2 = sin(internal_2);
    const double sin_internal_3 = sin(internal_3);
    const double sin_internal_5 = sin(internal_5);
    const double internal_4 =
        -kr * (cos_thetar * (dt * sin_internal_2 * vr + y - yr) - sin_thetar * (cos_internal_2 * dt * vr + x - xr)) + 1;
    const double internal_4_p2 = internal_4 * internal_4;

    // Evaluation of Matrix Fu
    Fu(0, 0) = 0.16666666666666666 * dt *
                   (cos_internal_0 * kr * vr *
                        (0.25 * cos_internal_2 * cos_thetar * dt_p3 * vr_p2 +
                         0.25 * dt_p3 * sin_internal_2 * sin_thetar * vr_p2) /
                        internal_4_p2 -
                    0.5 * dt_p2 * sin_internal_0 * vr_p2 / internal_4) -
               0.083333333333333329 * dt_p3 * sin_internal_3 * vr_p2 /
                   (-kr * (cos_thetar * (0.5 * dt * sin_internal_1 * vr + y - yr) -
                           sin_thetar * (0.5 * cos_internal_1 * dt * vr + x - xr)) +
                    1);
    Fu(1, 0) =
        -0.083333333333333329 * dt_p3 * sin_internal_2 * vr_p2 - 0.083333333333333329 * dt_p3 * sin_internal_5 * vr_p2;
    Fu(2, 0) =
        0.083333333333333329 * cos_internal_2 * dt_p3 * vr_p2 + 0.083333333333333329 * cos_internal_5 * dt_p3 * vr_p2;
    Fu(3, 0) = 0.5 * dt_p2 * vr;
    Fu(4, 0) = dt;
  }
};

}  // namespace gpal::pnc::planning