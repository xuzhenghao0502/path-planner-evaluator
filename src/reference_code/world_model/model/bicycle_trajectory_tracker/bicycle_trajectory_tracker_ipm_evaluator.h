#pragma once

#include "ocp/ipm_evaluator.h"

namespace gpal::pnc::planning {

template <StageType stage_type, OcpConfig::IntegratorType integrator_type>
class BicycleTrajectoryTrackerIpmEvaluator : public IpmEvaluator {
 public:
  BicycleTrajectoryTrackerIpmEvaluator() : IpmEvaluator(5, 2, 8, 0, 0) {}
  virtual ~BicycleTrajectoryTrackerIpmEvaluator() = default;

  virtual void updateLX(const Eigen::VectorXd& l_x, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& g_x,
                        const Eigen::MatrixXd& h_x, const Eigen::MatrixXd& m_x, const Eigen::VectorXd& p,
                        const Eigen::VectorXd& lambda, const Eigen::VectorXd& eta, const Eigen::VectorXd& nu,
                        Eigen::VectorXd& sl_x) const override {
    // Evaluation of Vector sl_x
    sl_x(0) = Fx(0, 0) * p(0) + l_x(0);
    sl_x(1) = Fx(1, 1) * p(1) + l_x(1);
    sl_x(2) = Fx(0, 2) * p(0) + Fx(1, 2) * p(1) + Fx(2, 2) * p(2) + l_x(2);
    sl_x(3) = Fx(0, 3) * p(0) + Fx(1, 3) * p(1) + Fx(2, 3) * p(2) + Fx(3, 3) * p(3) + g_x(0, 3) * lambda(0) +
              g_x(1, 3) * lambda(1) + l_x(3);
    sl_x(4) = Fx(2, 4) * p(2) + Fx(4, 4) * p(4) + g_x(2, 4) * lambda(2) + g_x(3, 4) * lambda(3) + l_x(4);
  }

  virtual void updateLU(const Eigen::VectorXd& l_u, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& g_u,
                        const Eigen::MatrixXd& h_u, const Eigen::VectorXd& p, const Eigen::VectorXd& lambda,
                        const Eigen::VectorXd& eta, Eigen::VectorXd& sl_u) const override {
    // Evaluation of Vector sl_u
    sl_u(0) = Fu(3, 0) * p(3) + g_u(4, 0) * lambda(4) + g_u(5, 0) * lambda(5) + l_u(0);
    sl_u(1) = Fu(4, 1) * p(4) + g_u(6, 1) * lambda(6) + g_u(7, 1) * lambda(7) + l_u(1);
  }

  virtual void updateQH(const Eigen::MatrixXd& l_xx, const Eigen::MatrixXd& l_xu, const Eigen::MatrixXd& l_uu,
                        const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::MatrixXd& Fx,
                        const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& V_xx_prev) override {
    // Determine internal variables
    const double Fu_3_0_p2 = Fu(3, 0) * Fu(3, 0);
    const double Fu_4_1_p2 = Fu(4, 1) * Fu(4, 1);
    const double Fx_0_0_p2 = Fx(0, 0) * Fx(0, 0);
    const double Fx_1_1_p2 = Fx(1, 1) * Fx(1, 1);
    const double g_u_4_0_p2 = g_u(4, 0) * g_u(4, 0);
    const double g_u_5_0_p2 = g_u(5, 0) * g_u(5, 0);
    const double g_u_6_1_p2 = g_u(6, 1) * g_u(6, 1);
    const double g_u_7_1_p2 = g_u(7, 1) * g_u(7, 1);
    const double g_x_0_3_p2 = g_x(0, 3) * g_x(0, 3);
    const double g_x_1_3_p2 = g_x(1, 3) * g_x(1, 3);
    const double g_x_2_4_p2 = g_x(2, 4) * g_x(2, 4);
    const double g_x_3_4_p2 = g_x(3, 4) * g_x(3, 4);

    // Evaluation of Matrix Q_xx_
    Q_xx_(0, 0) = Fx_0_0_p2 * V_xx_prev(0, 0) + l_xx(0, 0);
    Q_xx_(0, 1) = Fx(0, 0) * Fx(1, 1) * V_xx_prev(0, 1);
    Q_xx_(0, 2) = Fx(0, 0) * Fx(0, 2) * V_xx_prev(0, 0) + Fx(0, 0) * Fx(1, 2) * V_xx_prev(0, 1) +
                  Fx(0, 0) * Fx(2, 2) * V_xx_prev(0, 2);
    Q_xx_(0, 3) = Fx(0, 0) * Fx(0, 3) * V_xx_prev(0, 0) + Fx(0, 0) * Fx(1, 3) * V_xx_prev(0, 1) +
                  Fx(0, 0) * Fx(2, 3) * V_xx_prev(0, 2) + Fx(0, 0) * Fx(3, 3) * V_xx_prev(0, 3);
    Q_xx_(0, 4) = Fx(0, 0) * Fx(2, 4) * V_xx_prev(0, 2) + Fx(0, 0) * Fx(4, 4) * V_xx_prev(0, 4);
    Q_xx_(1, 0) = Fx(0, 0) * Fx(1, 1) * V_xx_prev(1, 0);
    Q_xx_(1, 1) = Fx_1_1_p2 * V_xx_prev(1, 1) + l_xx(1, 1);
    Q_xx_(1, 2) = Fx(0, 2) * Fx(1, 1) * V_xx_prev(1, 0) + Fx(1, 1) * Fx(1, 2) * V_xx_prev(1, 1) +
                  Fx(1, 1) * Fx(2, 2) * V_xx_prev(1, 2);
    Q_xx_(1, 3) = Fx(0, 3) * Fx(1, 1) * V_xx_prev(1, 0) + Fx(1, 1) * Fx(1, 3) * V_xx_prev(1, 1) +
                  Fx(1, 1) * Fx(2, 3) * V_xx_prev(1, 2) + Fx(1, 1) * Fx(3, 3) * V_xx_prev(1, 3);
    Q_xx_(1, 4) = Fx(1, 1) * Fx(2, 4) * V_xx_prev(1, 2) + Fx(1, 1) * Fx(4, 4) * V_xx_prev(1, 4);
    Q_xx_(2, 0) = Fx(0, 0) * (Fx(0, 2) * V_xx_prev(0, 0) + Fx(1, 2) * V_xx_prev(1, 0) + Fx(2, 2) * V_xx_prev(2, 0));
    Q_xx_(2, 1) = Fx(1, 1) * (Fx(0, 2) * V_xx_prev(0, 1) + Fx(1, 2) * V_xx_prev(1, 1) + Fx(2, 2) * V_xx_prev(2, 1));
    Q_xx_(2, 2) = Fx(0, 2) * (Fx(0, 2) * V_xx_prev(0, 0) + Fx(1, 2) * V_xx_prev(1, 0) + Fx(2, 2) * V_xx_prev(2, 0)) +
                  Fx(1, 2) * (Fx(0, 2) * V_xx_prev(0, 1) + Fx(1, 2) * V_xx_prev(1, 1) + Fx(2, 2) * V_xx_prev(2, 1)) +
                  Fx(2, 2) * (Fx(0, 2) * V_xx_prev(0, 2) + Fx(1, 2) * V_xx_prev(1, 2) + Fx(2, 2) * V_xx_prev(2, 2)) +
                  l_xx(2, 2);
    Q_xx_(2, 3) = Fx(0, 3) * (Fx(0, 2) * V_xx_prev(0, 0) + Fx(1, 2) * V_xx_prev(1, 0) + Fx(2, 2) * V_xx_prev(2, 0)) +
                  Fx(1, 3) * (Fx(0, 2) * V_xx_prev(0, 1) + Fx(1, 2) * V_xx_prev(1, 1) + Fx(2, 2) * V_xx_prev(2, 1)) +
                  Fx(2, 3) * (Fx(0, 2) * V_xx_prev(0, 2) + Fx(1, 2) * V_xx_prev(1, 2) + Fx(2, 2) * V_xx_prev(2, 2)) +
                  Fx(3, 3) * (Fx(0, 2) * V_xx_prev(0, 3) + Fx(1, 2) * V_xx_prev(1, 3) + Fx(2, 2) * V_xx_prev(2, 3));
    Q_xx_(2, 4) = Fx(2, 4) * (Fx(0, 2) * V_xx_prev(0, 2) + Fx(1, 2) * V_xx_prev(1, 2) + Fx(2, 2) * V_xx_prev(2, 2)) +
                  Fx(4, 4) * (Fx(0, 2) * V_xx_prev(0, 4) + Fx(1, 2) * V_xx_prev(1, 4) + Fx(2, 2) * V_xx_prev(2, 4));
    Q_xx_(3, 0) = Fx(0, 0) * (Fx(0, 3) * V_xx_prev(0, 0) + Fx(1, 3) * V_xx_prev(1, 0) + Fx(2, 3) * V_xx_prev(2, 0) +
                              Fx(3, 3) * V_xx_prev(3, 0));
    Q_xx_(3, 1) = Fx(1, 1) * (Fx(0, 3) * V_xx_prev(0, 1) + Fx(1, 3) * V_xx_prev(1, 1) + Fx(2, 3) * V_xx_prev(2, 1) +
                              Fx(3, 3) * V_xx_prev(3, 1));
    Q_xx_(3, 2) = Fx(0, 2) * (Fx(0, 3) * V_xx_prev(0, 0) + Fx(1, 3) * V_xx_prev(1, 0) + Fx(2, 3) * V_xx_prev(2, 0) +
                              Fx(3, 3) * V_xx_prev(3, 0)) +
                  Fx(1, 2) * (Fx(0, 3) * V_xx_prev(0, 1) + Fx(1, 3) * V_xx_prev(1, 1) + Fx(2, 3) * V_xx_prev(2, 1) +
                              Fx(3, 3) * V_xx_prev(3, 1)) +
                  Fx(2, 2) * (Fx(0, 3) * V_xx_prev(0, 2) + Fx(1, 3) * V_xx_prev(1, 2) + Fx(2, 3) * V_xx_prev(2, 2) +
                              Fx(3, 3) * V_xx_prev(3, 2));
    Q_xx_(3, 3) = Fx(0, 3) * (Fx(0, 3) * V_xx_prev(0, 0) + Fx(1, 3) * V_xx_prev(1, 0) + Fx(2, 3) * V_xx_prev(2, 0) +
                              Fx(3, 3) * V_xx_prev(3, 0)) +
                  Fx(1, 3) * (Fx(0, 3) * V_xx_prev(0, 1) + Fx(1, 3) * V_xx_prev(1, 1) + Fx(2, 3) * V_xx_prev(2, 1) +
                              Fx(3, 3) * V_xx_prev(3, 1)) +
                  Fx(2, 3) * (Fx(0, 3) * V_xx_prev(0, 2) + Fx(1, 3) * V_xx_prev(1, 2) + Fx(2, 3) * V_xx_prev(2, 2) +
                              Fx(3, 3) * V_xx_prev(3, 2)) +
                  Fx(3, 3) * (Fx(0, 3) * V_xx_prev(0, 3) + Fx(1, 3) * V_xx_prev(1, 3) + Fx(2, 3) * V_xx_prev(2, 3) +
                              Fx(3, 3) * V_xx_prev(3, 3)) +
                  Sigma_(0) * g_x_0_3_p2 + Sigma_(1) * g_x_1_3_p2 + l_xx(3, 3);
    Q_xx_(3, 4) = Fx(2, 4) * (Fx(0, 3) * V_xx_prev(0, 2) + Fx(1, 3) * V_xx_prev(1, 2) + Fx(2, 3) * V_xx_prev(2, 2) +
                              Fx(3, 3) * V_xx_prev(3, 2)) +
                  Fx(4, 4) * (Fx(0, 3) * V_xx_prev(0, 4) + Fx(1, 3) * V_xx_prev(1, 4) + Fx(2, 3) * V_xx_prev(2, 4) +
                              Fx(3, 3) * V_xx_prev(3, 4));
    Q_xx_(4, 0) = Fx(0, 0) * (Fx(2, 4) * V_xx_prev(2, 0) + Fx(4, 4) * V_xx_prev(4, 0));
    Q_xx_(4, 1) = Fx(1, 1) * (Fx(2, 4) * V_xx_prev(2, 1) + Fx(4, 4) * V_xx_prev(4, 1));
    Q_xx_(4, 2) = Fx(0, 2) * (Fx(2, 4) * V_xx_prev(2, 0) + Fx(4, 4) * V_xx_prev(4, 0)) +
                  Fx(1, 2) * (Fx(2, 4) * V_xx_prev(2, 1) + Fx(4, 4) * V_xx_prev(4, 1)) +
                  Fx(2, 2) * (Fx(2, 4) * V_xx_prev(2, 2) + Fx(4, 4) * V_xx_prev(4, 2));
    Q_xx_(4, 3) = Fx(0, 3) * (Fx(2, 4) * V_xx_prev(2, 0) + Fx(4, 4) * V_xx_prev(4, 0)) +
                  Fx(1, 3) * (Fx(2, 4) * V_xx_prev(2, 1) + Fx(4, 4) * V_xx_prev(4, 1)) +
                  Fx(2, 3) * (Fx(2, 4) * V_xx_prev(2, 2) + Fx(4, 4) * V_xx_prev(4, 2)) +
                  Fx(3, 3) * (Fx(2, 4) * V_xx_prev(2, 3) + Fx(4, 4) * V_xx_prev(4, 3));
    Q_xx_(4, 4) = Fx(2, 4) * (Fx(2, 4) * V_xx_prev(2, 2) + Fx(4, 4) * V_xx_prev(4, 2)) +
                  Fx(4, 4) * (Fx(2, 4) * V_xx_prev(2, 4) + Fx(4, 4) * V_xx_prev(4, 4)) + Sigma_(2) * g_x_2_4_p2 +
                  Sigma_(3) * g_x_3_4_p2 + l_xx(4, 4);

    // Evaluation of Matrix Q_xu_
    Q_xu_(0, 0) = Fu(3, 0) * Fx(0, 0) * V_xx_prev(0, 3);
    Q_xu_(0, 1) = Fu(4, 1) * Fx(0, 0) * V_xx_prev(0, 4);
    Q_xu_(1, 0) = Fu(3, 0) * Fx(1, 1) * V_xx_prev(1, 3);
    Q_xu_(1, 1) = Fu(4, 1) * Fx(1, 1) * V_xx_prev(1, 4);
    Q_xu_(2, 0) = Fu(3, 0) * (Fx(0, 2) * V_xx_prev(0, 3) + Fx(1, 2) * V_xx_prev(1, 3) + Fx(2, 2) * V_xx_prev(2, 3));
    Q_xu_(2, 1) = Fu(4, 1) * (Fx(0, 2) * V_xx_prev(0, 4) + Fx(1, 2) * V_xx_prev(1, 4) + Fx(2, 2) * V_xx_prev(2, 4));
    Q_xu_(3, 0) = Fu(3, 0) * (Fx(0, 3) * V_xx_prev(0, 3) + Fx(1, 3) * V_xx_prev(1, 3) + Fx(2, 3) * V_xx_prev(2, 3) +
                              Fx(3, 3) * V_xx_prev(3, 3));
    Q_xu_(3, 1) = Fu(4, 1) * (Fx(0, 3) * V_xx_prev(0, 4) + Fx(1, 3) * V_xx_prev(1, 4) + Fx(2, 3) * V_xx_prev(2, 4) +
                              Fx(3, 3) * V_xx_prev(3, 4));
    Q_xu_(4, 0) = Fu(3, 0) * (Fx(2, 4) * V_xx_prev(2, 3) + Fx(4, 4) * V_xx_prev(4, 3));
    Q_xu_(4, 1) = Fu(4, 1) * (Fx(2, 4) * V_xx_prev(2, 4) + Fx(4, 4) * V_xx_prev(4, 4));

    // Evaluation of Matrix Q_uu_
    Q_uu_(0, 0) = Fu_3_0_p2 * V_xx_prev(3, 3) + Sigma_(4) * g_u_4_0_p2 + Sigma_(5) * g_u_5_0_p2 + l_uu(0, 0);
    Q_uu_(0, 1) = Fu(3, 0) * Fu(4, 1) * V_xx_prev(3, 4);
    Q_uu_(1, 0) = Fu(3, 0) * Fu(4, 1) * V_xx_prev(4, 3);
    Q_uu_(1, 1) = Fu_4_1_p2 * V_xx_prev(4, 4) + Sigma_(6) * g_u_6_1_p2 + Sigma_(7) * g_u_7_1_p2 + l_uu(1, 1);
  }

  virtual void updateQG(const Eigen::VectorXd& sl_x, const Eigen::VectorXd& sl_u, const Eigen::MatrixXd& g_x,
                        const Eigen::MatrixXd& g_u, const Eigen::VectorXd& r_f, const Eigen::MatrixXd& Fx,
                        const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& V_xx_prev,
                        const Eigen::VectorXd& V_x_prev) override {
    // Evaluation of Vector Q_x_
    Q_x_(0) = Fx(0, 0) * (V_x_prev(0) + V_xx_prev(0, 0) * r_f(0) + V_xx_prev(0, 1) * r_f(1) + V_xx_prev(0, 2) * r_f(2) +
                          V_xx_prev(0, 3) * r_f(3) + V_xx_prev(0, 4) * r_f(4)) +
              sl_x(0);
    Q_x_(1) = Fx(1, 1) * (V_x_prev(1) + V_xx_prev(1, 0) * r_f(0) + V_xx_prev(1, 1) * r_f(1) + V_xx_prev(1, 2) * r_f(2) +
                          V_xx_prev(1, 3) * r_f(3) + V_xx_prev(1, 4) * r_f(4)) +
              sl_x(1);
    Q_x_(2) = Fx(0, 2) * (V_x_prev(0) + V_xx_prev(0, 0) * r_f(0) + V_xx_prev(0, 1) * r_f(1) + V_xx_prev(0, 2) * r_f(2) +
                          V_xx_prev(0, 3) * r_f(3) + V_xx_prev(0, 4) * r_f(4)) +
              Fx(1, 2) * (V_x_prev(1) + V_xx_prev(1, 0) * r_f(0) + V_xx_prev(1, 1) * r_f(1) + V_xx_prev(1, 2) * r_f(2) +
                          V_xx_prev(1, 3) * r_f(3) + V_xx_prev(1, 4) * r_f(4)) +
              Fx(2, 2) * (V_x_prev(2) + V_xx_prev(2, 0) * r_f(0) + V_xx_prev(2, 1) * r_f(1) + V_xx_prev(2, 2) * r_f(2) +
                          V_xx_prev(2, 3) * r_f(3) + V_xx_prev(2, 4) * r_f(4)) +
              sl_x(2);
    Q_x_(3) = Fx(0, 3) * (V_x_prev(0) + V_xx_prev(0, 0) * r_f(0) + V_xx_prev(0, 1) * r_f(1) + V_xx_prev(0, 2) * r_f(2) +
                          V_xx_prev(0, 3) * r_f(3) + V_xx_prev(0, 4) * r_f(4)) +
              Fx(1, 3) * (V_x_prev(1) + V_xx_prev(1, 0) * r_f(0) + V_xx_prev(1, 1) * r_f(1) + V_xx_prev(1, 2) * r_f(2) +
                          V_xx_prev(1, 3) * r_f(3) + V_xx_prev(1, 4) * r_f(4)) +
              Fx(2, 3) * (V_x_prev(2) + V_xx_prev(2, 0) * r_f(0) + V_xx_prev(2, 1) * r_f(1) + V_xx_prev(2, 2) * r_f(2) +
                          V_xx_prev(2, 3) * r_f(3) + V_xx_prev(2, 4) * r_f(4)) +
              Fx(3, 3) * (V_x_prev(3) + V_xx_prev(3, 0) * r_f(0) + V_xx_prev(3, 1) * r_f(1) + V_xx_prev(3, 2) * r_f(2) +
                          V_xx_prev(3, 3) * r_f(3) + V_xx_prev(3, 4) * r_f(4)) +
              V_(0) * g_x(0, 3) + V_(1) * g_x(1, 3) + sl_x(3);
    Q_x_(4) = Fx(2, 4) * (V_x_prev(2) + V_xx_prev(2, 0) * r_f(0) + V_xx_prev(2, 1) * r_f(1) + V_xx_prev(2, 2) * r_f(2) +
                          V_xx_prev(2, 3) * r_f(3) + V_xx_prev(2, 4) * r_f(4)) +
              Fx(4, 4) * (V_x_prev(4) + V_xx_prev(4, 0) * r_f(0) + V_xx_prev(4, 1) * r_f(1) + V_xx_prev(4, 2) * r_f(2) +
                          V_xx_prev(4, 3) * r_f(3) + V_xx_prev(4, 4) * r_f(4)) +
              V_(2) * g_x(2, 4) + V_(3) * g_x(3, 4) + sl_x(4);

    // Evaluation of Vector Q_u_
    Q_u_(0) = Fu(3, 0) * (V_x_prev(3) + V_xx_prev(3, 0) * r_f(0) + V_xx_prev(3, 1) * r_f(1) + V_xx_prev(3, 2) * r_f(2) +
                          V_xx_prev(3, 3) * r_f(3) + V_xx_prev(3, 4) * r_f(4)) +
              V_(4) * g_u(4, 0) + V_(5) * g_u(5, 0) + sl_u(0);
    Q_u_(1) = Fu(4, 1) * (V_x_prev(4) + V_xx_prev(4, 0) * r_f(0) + V_xx_prev(4, 1) * r_f(1) + V_xx_prev(4, 2) * r_f(2) +
                          V_xx_prev(4, 3) * r_f(3) + V_xx_prev(4, 4) * r_f(4)) +
              V_(6) * g_u(6, 1) + V_(7) * g_u(7, 1) + sl_u(1);
  }

  virtual void updateQuuInv() override {
    // Determine internal variables
    const double Q_uu__0_0_p2 = Q_uu_(0, 0) * Q_uu_(0, 0);
    const double Q_uu__0_1_p2 = Q_uu_(0, 1) * Q_uu_(0, 1);
    const double internal_0 = Q_uu_(1, 1) - Q_uu__0_1_p2 / Q_uu_(0, 0);

    // Evaluation of Matrix Q_uu_inv_
    Q_uu_inv_(0, 0) = Q_uu__0_1_p2 / (Q_uu__0_0_p2 * internal_0) + 1.0 / Q_uu_(0, 0);
    Q_uu_inv_(0, 1) = -Q_uu_(0, 1) / (Q_uu_(0, 0) * internal_0);
    Q_uu_inv_(1, 0) = Q_uu_inv_(0, 1);
    Q_uu_inv_(1, 1) = 1.0 / internal_0;
  }

  virtual void updateKVU(Eigen::MatrixXd& k_ux, Eigen::VectorXd& v_u) override {
    // Evaluation of Matrix k_ux
    k_ux(0, 0) = -Q_uu_inv_(0, 0) * Q_xu_(0, 0) - Q_uu_inv_(0, 1) * Q_xu_(0, 1);
    k_ux(0, 1) = -Q_uu_inv_(0, 0) * Q_xu_(1, 0) - Q_uu_inv_(0, 1) * Q_xu_(1, 1);
    k_ux(0, 2) = -Q_uu_inv_(0, 0) * Q_xu_(2, 0) - Q_uu_inv_(0, 1) * Q_xu_(2, 1);
    k_ux(0, 3) = -Q_uu_inv_(0, 0) * Q_xu_(3, 0) - Q_uu_inv_(0, 1) * Q_xu_(3, 1);
    k_ux(0, 4) = -Q_uu_inv_(0, 0) * Q_xu_(4, 0) - Q_uu_inv_(0, 1) * Q_xu_(4, 1);
    k_ux(1, 0) = -Q_uu_inv_(0, 1) * Q_xu_(0, 0) - Q_uu_inv_(1, 1) * Q_xu_(0, 1);
    k_ux(1, 1) = -Q_uu_inv_(0, 1) * Q_xu_(1, 0) - Q_uu_inv_(1, 1) * Q_xu_(1, 1);
    k_ux(1, 2) = -Q_uu_inv_(0, 1) * Q_xu_(2, 0) - Q_uu_inv_(1, 1) * Q_xu_(2, 1);
    k_ux(1, 3) = -Q_uu_inv_(0, 1) * Q_xu_(3, 0) - Q_uu_inv_(1, 1) * Q_xu_(3, 1);
    k_ux(1, 4) = -Q_uu_inv_(0, 1) * Q_xu_(4, 0) - Q_uu_inv_(1, 1) * Q_xu_(4, 1);

    // Evaluation of Vector v_u
    v_u(0) = -Q_u_(0) * Q_uu_inv_(0, 0) - Q_u_(1) * Q_uu_inv_(0, 1);
    v_u(1) = -Q_u_(0) * Q_uu_inv_(0, 1) - Q_u_(1) * Q_uu_inv_(1, 1);
  }

  virtual void updateQee(const Eigen::MatrixXd& h_u) override {}

  virtual void updateQeeInv() override {}

  virtual void updateQxe(const Eigen::VectorXd& h, const Eigen::MatrixXd& h_x, const Eigen::MatrixXd& h_u,
                         const Eigen::MatrixXd& k_ux, const Eigen::VectorXd& v_u) override {}

  virtual void updateKVE(const Eigen::MatrixXd& h_u, Eigen::MatrixXd& k_ex, Eigen::MatrixXd& k_ue,
                         Eigen::VectorXd& v_e) override {}

  virtual void updateVxx(const Eigen::MatrixXd& k_ux, const Eigen::MatrixXd& k_ex, const Eigen::VectorXd& v_u,
                         const Eigen::VectorXd& v_e, Eigen::MatrixXd& V_xx, Eigen::VectorXd& V_x) override {
    // Evaluation of Matrix V_xx
    V_xx(0, 0) = Q_xu_(0, 0) * k_ux(0, 0) + Q_xu_(0, 1) * k_ux(1, 0) + Q_xx_(0, 0);
    V_xx(0, 1) = Q_xu_(0, 0) * k_ux(0, 1) + Q_xu_(0, 1) * k_ux(1, 1) + Q_xx_(0, 1);
    V_xx(0, 2) = Q_xu_(0, 0) * k_ux(0, 2) + Q_xu_(0, 1) * k_ux(1, 2) + Q_xx_(0, 2);
    V_xx(0, 3) = Q_xu_(0, 0) * k_ux(0, 3) + Q_xu_(0, 1) * k_ux(1, 3) + Q_xx_(0, 3);
    V_xx(0, 4) = Q_xu_(0, 0) * k_ux(0, 4) + Q_xu_(0, 1) * k_ux(1, 4) + Q_xx_(0, 4);
    V_xx(1, 0) = Q_xu_(1, 0) * k_ux(0, 0) + Q_xu_(1, 1) * k_ux(1, 0) + Q_xx_(0, 1);
    V_xx(1, 1) = Q_xu_(1, 0) * k_ux(0, 1) + Q_xu_(1, 1) * k_ux(1, 1) + Q_xx_(1, 1);
    V_xx(1, 2) = Q_xu_(1, 0) * k_ux(0, 2) + Q_xu_(1, 1) * k_ux(1, 2) + Q_xx_(1, 2);
    V_xx(1, 3) = Q_xu_(1, 0) * k_ux(0, 3) + Q_xu_(1, 1) * k_ux(1, 3) + Q_xx_(1, 3);
    V_xx(1, 4) = Q_xu_(1, 0) * k_ux(0, 4) + Q_xu_(1, 1) * k_ux(1, 4) + Q_xx_(1, 4);
    V_xx(2, 0) = Q_xu_(2, 0) * k_ux(0, 0) + Q_xu_(2, 1) * k_ux(1, 0) + Q_xx_(0, 2);
    V_xx(2, 1) = Q_xu_(2, 0) * k_ux(0, 1) + Q_xu_(2, 1) * k_ux(1, 1) + Q_xx_(1, 2);
    V_xx(2, 2) = Q_xu_(2, 0) * k_ux(0, 2) + Q_xu_(2, 1) * k_ux(1, 2) + Q_xx_(2, 2);
    V_xx(2, 3) = Q_xu_(2, 0) * k_ux(0, 3) + Q_xu_(2, 1) * k_ux(1, 3) + Q_xx_(2, 3);
    V_xx(2, 4) = Q_xu_(2, 0) * k_ux(0, 4) + Q_xu_(2, 1) * k_ux(1, 4) + Q_xx_(2, 4);
    V_xx(3, 0) = Q_xu_(3, 0) * k_ux(0, 0) + Q_xu_(3, 1) * k_ux(1, 0) + Q_xx_(0, 3);
    V_xx(3, 1) = Q_xu_(3, 0) * k_ux(0, 1) + Q_xu_(3, 1) * k_ux(1, 1) + Q_xx_(1, 3);
    V_xx(3, 2) = Q_xu_(3, 0) * k_ux(0, 2) + Q_xu_(3, 1) * k_ux(1, 2) + Q_xx_(2, 3);
    V_xx(3, 3) = Q_xu_(3, 0) * k_ux(0, 3) + Q_xu_(3, 1) * k_ux(1, 3) + Q_xx_(3, 3);
    V_xx(3, 4) = Q_xu_(3, 0) * k_ux(0, 4) + Q_xu_(3, 1) * k_ux(1, 4) + Q_xx_(3, 4);
    V_xx(4, 0) = Q_xu_(4, 0) * k_ux(0, 0) + Q_xu_(4, 1) * k_ux(1, 0) + Q_xx_(0, 4);
    V_xx(4, 1) = Q_xu_(4, 0) * k_ux(0, 1) + Q_xu_(4, 1) * k_ux(1, 1) + Q_xx_(1, 4);
    V_xx(4, 2) = Q_xu_(4, 0) * k_ux(0, 2) + Q_xu_(4, 1) * k_ux(1, 2) + Q_xx_(2, 4);
    V_xx(4, 3) = Q_xu_(4, 0) * k_ux(0, 3) + Q_xu_(4, 1) * k_ux(1, 3) + Q_xx_(3, 4);
    V_xx(4, 4) = Q_xu_(4, 0) * k_ux(0, 4) + Q_xu_(4, 1) * k_ux(1, 4) + Q_xx_(4, 4);

    // Evaluation of Vector V_x
    V_x(0) = Q_x_(0) + Q_xu_(0, 0) * v_u(0) + Q_xu_(0, 1) * v_u(1);
    V_x(1) = Q_x_(1) + Q_xu_(1, 0) * v_u(0) + Q_xu_(1, 1) * v_u(1);
    V_x(2) = Q_x_(2) + Q_xu_(2, 0) * v_u(0) + Q_xu_(2, 1) * v_u(1);
    V_x(3) = Q_x_(3) + Q_xu_(3, 0) * v_u(0) + Q_xu_(3, 1) * v_u(1);
    V_x(4) = Q_x_(4) + Q_xu_(4, 0) * v_u(0) + Q_xu_(4, 1) * v_u(1);
  }

  virtual void updateQnx(const Eigen::VectorXd& r_f, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& Fu,
                         const Eigen::MatrixXd& h_u, const Eigen::MatrixXd& k_ux, const Eigen::MatrixXd& k_ex,
                         const Eigen::MatrixXd& k_ue, const Eigen::VectorXd& v_u, const Eigen::VectorXd& v_e) override {
    // Evaluation of Matrix Q_un_
    Q_un_(0, 3) = -Fu(3, 0) * Q_uu_inv_(0, 0);
    Q_un_(0, 4) = -Fu(4, 1) * Q_uu_inv_(0, 1);
    Q_un_(1, 3) = -Fu(3, 0) * Q_uu_inv_(0, 1);
    Q_un_(1, 4) = -Fu(4, 1) * Q_uu_inv_(1, 1);

    // Evaluation of Matrix Q_nuu_
    Q_nuu_(0, 0) = -Q_uu_(0, 0);
    Q_nuu_(0, 1) = -Q_uu_(0, 1);
    Q_nuu_(1, 0) = -Q_uu_(0, 1);
    Q_nuu_(1, 1) = -Q_uu_(1, 1);

    // Evaluation of Matrix Q_vnx_
    Q_vnx_(0, 0) = Fx(0, 0);
    Q_vnx_(0, 2) = Fx(0, 2);
    Q_vnx_(0, 3) = Fx(0, 3);
    Q_vnx_(1, 1) = Fx(1, 1);
    Q_vnx_(1, 2) = Fx(1, 2);
    Q_vnx_(1, 3) = Fx(1, 3);
    Q_vnx_(2, 2) = Fx(2, 2);
    Q_vnx_(2, 3) = Fx(2, 3);
    Q_vnx_(2, 4) = Fx(2, 4);
    Q_vnx_(3, 0) = Fu(3, 0) * k_ux(0, 0);
    Q_vnx_(3, 1) = Fu(3, 0) * k_ux(0, 1);
    Q_vnx_(3, 2) = Fu(3, 0) * k_ux(0, 2);
    Q_vnx_(3, 3) = Fu(3, 0) * k_ux(0, 3) + Fx(3, 3);
    Q_vnx_(3, 4) = Fu(3, 0) * k_ux(0, 4);
    Q_vnx_(4, 0) = Fu(4, 1) * k_ux(1, 0);
    Q_vnx_(4, 1) = Fu(4, 1) * k_ux(1, 1);
    Q_vnx_(4, 2) = Fu(4, 1) * k_ux(1, 2);
    Q_vnx_(4, 3) = Fu(4, 1) * k_ux(1, 3);
    Q_vnx_(4, 4) = Fu(4, 1) * k_ux(1, 4) + Fx(4, 4);

    // Evaluation of Vector Q_vn_
    Q_vn_(0) = r_f(0);
    Q_vn_(1) = r_f(1);
    Q_vn_(2) = r_f(2);
    Q_vn_(3) = Fu(3, 0) * v_u(0) + r_f(3);
    Q_vn_(4) = Fu(4, 1) * v_u(1) + r_f(4);
  }
};

template <>
class BicycleTrajectoryTrackerIpmEvaluator<StageType::INITIAL, OcpConfig::FORWARD_EULER> : public IpmEvaluator {
 public:
  BicycleTrajectoryTrackerIpmEvaluator() : IpmEvaluator(5, 2, 4, 0, 5) {}
  virtual ~BicycleTrajectoryTrackerIpmEvaluator() = default;

  virtual void updateLX(const Eigen::VectorXd& l_x, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& g_x,
                        const Eigen::MatrixXd& h_x, const Eigen::MatrixXd& m_x, const Eigen::VectorXd& p,
                        const Eigen::VectorXd& lambda, const Eigen::VectorXd& eta, const Eigen::VectorXd& nu,
                        Eigen::VectorXd& sl_x) const override {
    // Evaluation of Vector sl_x
    sl_x(0) = Fx(0, 0) * p(0) + l_x(0) + m_x(0, 0) * nu(0);
    sl_x(1) = Fx(1, 1) * p(1) + l_x(1) + m_x(1, 1) * nu(1);
    sl_x(2) = Fx(0, 2) * p(0) + Fx(1, 2) * p(1) + Fx(2, 2) * p(2) + l_x(2) + m_x(2, 2) * nu(2);
    sl_x(3) = Fx(0, 3) * p(0) + Fx(1, 3) * p(1) + Fx(2, 3) * p(2) + Fx(3, 3) * p(3) + l_x(3) + m_x(3, 3) * nu(3);
    sl_x(4) = Fx(2, 4) * p(2) + Fx(4, 4) * p(4) + l_x(4) + m_x(4, 4) * nu(4);
  }

  virtual void updateLU(const Eigen::VectorXd& l_u, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& g_u,
                        const Eigen::MatrixXd& h_u, const Eigen::VectorXd& p, const Eigen::VectorXd& lambda,
                        const Eigen::VectorXd& eta, Eigen::VectorXd& sl_u) const override {
    // Evaluation of Vector sl_u
    sl_u(0) = Fu(3, 0) * p(3) + g_u(0, 0) * lambda(0) + g_u(1, 0) * lambda(1) + l_u(0);
    sl_u(1) = Fu(4, 1) * p(4) + g_u(2, 1) * lambda(2) + g_u(3, 1) * lambda(3) + l_u(1);
  }

  virtual void updateQH(const Eigen::MatrixXd& l_xx, const Eigen::MatrixXd& l_xu, const Eigen::MatrixXd& l_uu,
                        const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::MatrixXd& Fx,
                        const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& V_xx_prev) override {
    // Determine internal variables
    const double Fu_3_0_p2 = Fu(3, 0) * Fu(3, 0);
    const double Fu_4_1_p2 = Fu(4, 1) * Fu(4, 1);
    const double Fx_0_0_p2 = Fx(0, 0) * Fx(0, 0);
    const double Fx_1_1_p2 = Fx(1, 1) * Fx(1, 1);
    const double g_u_0_0_p2 = g_u(0, 0) * g_u(0, 0);
    const double g_u_1_0_p2 = g_u(1, 0) * g_u(1, 0);
    const double g_u_2_1_p2 = g_u(2, 1) * g_u(2, 1);
    const double g_u_3_1_p2 = g_u(3, 1) * g_u(3, 1);

    // Evaluation of Matrix Q_xx_
    Q_xx_(0, 0) = Fx_0_0_p2 * V_xx_prev(0, 0) + l_xx(0, 0);
    Q_xx_(0, 1) = Fx(0, 0) * Fx(1, 1) * V_xx_prev(0, 1);
    Q_xx_(0, 2) = Fx(0, 0) * Fx(0, 2) * V_xx_prev(0, 0) + Fx(0, 0) * Fx(1, 2) * V_xx_prev(0, 1) +
                  Fx(0, 0) * Fx(2, 2) * V_xx_prev(0, 2);
    Q_xx_(0, 3) = Fx(0, 0) * Fx(0, 3) * V_xx_prev(0, 0) + Fx(0, 0) * Fx(1, 3) * V_xx_prev(0, 1) +
                  Fx(0, 0) * Fx(2, 3) * V_xx_prev(0, 2) + Fx(0, 0) * Fx(3, 3) * V_xx_prev(0, 3);
    Q_xx_(0, 4) = Fx(0, 0) * Fx(2, 4) * V_xx_prev(0, 2) + Fx(0, 0) * Fx(4, 4) * V_xx_prev(0, 4);
    Q_xx_(1, 0) = Fx(0, 0) * Fx(1, 1) * V_xx_prev(1, 0);
    Q_xx_(1, 1) = Fx_1_1_p2 * V_xx_prev(1, 1) + l_xx(1, 1);
    Q_xx_(1, 2) = Fx(0, 2) * Fx(1, 1) * V_xx_prev(1, 0) + Fx(1, 1) * Fx(1, 2) * V_xx_prev(1, 1) +
                  Fx(1, 1) * Fx(2, 2) * V_xx_prev(1, 2);
    Q_xx_(1, 3) = Fx(0, 3) * Fx(1, 1) * V_xx_prev(1, 0) + Fx(1, 1) * Fx(1, 3) * V_xx_prev(1, 1) +
                  Fx(1, 1) * Fx(2, 3) * V_xx_prev(1, 2) + Fx(1, 1) * Fx(3, 3) * V_xx_prev(1, 3);
    Q_xx_(1, 4) = Fx(1, 1) * Fx(2, 4) * V_xx_prev(1, 2) + Fx(1, 1) * Fx(4, 4) * V_xx_prev(1, 4);
    Q_xx_(2, 0) = Fx(0, 0) * (Fx(0, 2) * V_xx_prev(0, 0) + Fx(1, 2) * V_xx_prev(1, 0) + Fx(2, 2) * V_xx_prev(2, 0));
    Q_xx_(2, 1) = Fx(1, 1) * (Fx(0, 2) * V_xx_prev(0, 1) + Fx(1, 2) * V_xx_prev(1, 1) + Fx(2, 2) * V_xx_prev(2, 1));
    Q_xx_(2, 2) = Fx(0, 2) * (Fx(0, 2) * V_xx_prev(0, 0) + Fx(1, 2) * V_xx_prev(1, 0) + Fx(2, 2) * V_xx_prev(2, 0)) +
                  Fx(1, 2) * (Fx(0, 2) * V_xx_prev(0, 1) + Fx(1, 2) * V_xx_prev(1, 1) + Fx(2, 2) * V_xx_prev(2, 1)) +
                  Fx(2, 2) * (Fx(0, 2) * V_xx_prev(0, 2) + Fx(1, 2) * V_xx_prev(1, 2) + Fx(2, 2) * V_xx_prev(2, 2)) +
                  l_xx(2, 2);
    Q_xx_(2, 3) = Fx(0, 3) * (Fx(0, 2) * V_xx_prev(0, 0) + Fx(1, 2) * V_xx_prev(1, 0) + Fx(2, 2) * V_xx_prev(2, 0)) +
                  Fx(1, 3) * (Fx(0, 2) * V_xx_prev(0, 1) + Fx(1, 2) * V_xx_prev(1, 1) + Fx(2, 2) * V_xx_prev(2, 1)) +
                  Fx(2, 3) * (Fx(0, 2) * V_xx_prev(0, 2) + Fx(1, 2) * V_xx_prev(1, 2) + Fx(2, 2) * V_xx_prev(2, 2)) +
                  Fx(3, 3) * (Fx(0, 2) * V_xx_prev(0, 3) + Fx(1, 2) * V_xx_prev(1, 3) + Fx(2, 2) * V_xx_prev(2, 3));
    Q_xx_(2, 4) = Fx(2, 4) * (Fx(0, 2) * V_xx_prev(0, 2) + Fx(1, 2) * V_xx_prev(1, 2) + Fx(2, 2) * V_xx_prev(2, 2)) +
                  Fx(4, 4) * (Fx(0, 2) * V_xx_prev(0, 4) + Fx(1, 2) * V_xx_prev(1, 4) + Fx(2, 2) * V_xx_prev(2, 4));
    Q_xx_(3, 0) = Fx(0, 0) * (Fx(0, 3) * V_xx_prev(0, 0) + Fx(1, 3) * V_xx_prev(1, 0) + Fx(2, 3) * V_xx_prev(2, 0) +
                              Fx(3, 3) * V_xx_prev(3, 0));
    Q_xx_(3, 1) = Fx(1, 1) * (Fx(0, 3) * V_xx_prev(0, 1) + Fx(1, 3) * V_xx_prev(1, 1) + Fx(2, 3) * V_xx_prev(2, 1) +
                              Fx(3, 3) * V_xx_prev(3, 1));
    Q_xx_(3, 2) = Fx(0, 2) * (Fx(0, 3) * V_xx_prev(0, 0) + Fx(1, 3) * V_xx_prev(1, 0) + Fx(2, 3) * V_xx_prev(2, 0) +
                              Fx(3, 3) * V_xx_prev(3, 0)) +
                  Fx(1, 2) * (Fx(0, 3) * V_xx_prev(0, 1) + Fx(1, 3) * V_xx_prev(1, 1) + Fx(2, 3) * V_xx_prev(2, 1) +
                              Fx(3, 3) * V_xx_prev(3, 1)) +
                  Fx(2, 2) * (Fx(0, 3) * V_xx_prev(0, 2) + Fx(1, 3) * V_xx_prev(1, 2) + Fx(2, 3) * V_xx_prev(2, 2) +
                              Fx(3, 3) * V_xx_prev(3, 2));
    Q_xx_(3, 3) = Fx(0, 3) * (Fx(0, 3) * V_xx_prev(0, 0) + Fx(1, 3) * V_xx_prev(1, 0) + Fx(2, 3) * V_xx_prev(2, 0) +
                              Fx(3, 3) * V_xx_prev(3, 0)) +
                  Fx(1, 3) * (Fx(0, 3) * V_xx_prev(0, 1) + Fx(1, 3) * V_xx_prev(1, 1) + Fx(2, 3) * V_xx_prev(2, 1) +
                              Fx(3, 3) * V_xx_prev(3, 1)) +
                  Fx(2, 3) * (Fx(0, 3) * V_xx_prev(0, 2) + Fx(1, 3) * V_xx_prev(1, 2) + Fx(2, 3) * V_xx_prev(2, 2) +
                              Fx(3, 3) * V_xx_prev(3, 2)) +
                  Fx(3, 3) * (Fx(0, 3) * V_xx_prev(0, 3) + Fx(1, 3) * V_xx_prev(1, 3) + Fx(2, 3) * V_xx_prev(2, 3) +
                              Fx(3, 3) * V_xx_prev(3, 3)) +
                  l_xx(3, 3);
    Q_xx_(3, 4) = Fx(2, 4) * (Fx(0, 3) * V_xx_prev(0, 2) + Fx(1, 3) * V_xx_prev(1, 2) + Fx(2, 3) * V_xx_prev(2, 2) +
                              Fx(3, 3) * V_xx_prev(3, 2)) +
                  Fx(4, 4) * (Fx(0, 3) * V_xx_prev(0, 4) + Fx(1, 3) * V_xx_prev(1, 4) + Fx(2, 3) * V_xx_prev(2, 4) +
                              Fx(3, 3) * V_xx_prev(3, 4));
    Q_xx_(4, 0) = Fx(0, 0) * (Fx(2, 4) * V_xx_prev(2, 0) + Fx(4, 4) * V_xx_prev(4, 0));
    Q_xx_(4, 1) = Fx(1, 1) * (Fx(2, 4) * V_xx_prev(2, 1) + Fx(4, 4) * V_xx_prev(4, 1));
    Q_xx_(4, 2) = Fx(0, 2) * (Fx(2, 4) * V_xx_prev(2, 0) + Fx(4, 4) * V_xx_prev(4, 0)) +
                  Fx(1, 2) * (Fx(2, 4) * V_xx_prev(2, 1) + Fx(4, 4) * V_xx_prev(4, 1)) +
                  Fx(2, 2) * (Fx(2, 4) * V_xx_prev(2, 2) + Fx(4, 4) * V_xx_prev(4, 2));
    Q_xx_(4, 3) = Fx(0, 3) * (Fx(2, 4) * V_xx_prev(2, 0) + Fx(4, 4) * V_xx_prev(4, 0)) +
                  Fx(1, 3) * (Fx(2, 4) * V_xx_prev(2, 1) + Fx(4, 4) * V_xx_prev(4, 1)) +
                  Fx(2, 3) * (Fx(2, 4) * V_xx_prev(2, 2) + Fx(4, 4) * V_xx_prev(4, 2)) +
                  Fx(3, 3) * (Fx(2, 4) * V_xx_prev(2, 3) + Fx(4, 4) * V_xx_prev(4, 3));
    Q_xx_(4, 4) = Fx(2, 4) * (Fx(2, 4) * V_xx_prev(2, 2) + Fx(4, 4) * V_xx_prev(4, 2)) +
                  Fx(4, 4) * (Fx(2, 4) * V_xx_prev(2, 4) + Fx(4, 4) * V_xx_prev(4, 4)) + l_xx(4, 4);

    // Evaluation of Matrix Q_xu_
    Q_xu_(0, 0) = Fu(3, 0) * Fx(0, 0) * V_xx_prev(0, 3);
    Q_xu_(0, 1) = Fu(4, 1) * Fx(0, 0) * V_xx_prev(0, 4);
    Q_xu_(1, 0) = Fu(3, 0) * Fx(1, 1) * V_xx_prev(1, 3);
    Q_xu_(1, 1) = Fu(4, 1) * Fx(1, 1) * V_xx_prev(1, 4);
    Q_xu_(2, 0) = Fu(3, 0) * (Fx(0, 2) * V_xx_prev(0, 3) + Fx(1, 2) * V_xx_prev(1, 3) + Fx(2, 2) * V_xx_prev(2, 3));
    Q_xu_(2, 1) = Fu(4, 1) * (Fx(0, 2) * V_xx_prev(0, 4) + Fx(1, 2) * V_xx_prev(1, 4) + Fx(2, 2) * V_xx_prev(2, 4));
    Q_xu_(3, 0) = Fu(3, 0) * (Fx(0, 3) * V_xx_prev(0, 3) + Fx(1, 3) * V_xx_prev(1, 3) + Fx(2, 3) * V_xx_prev(2, 3) +
                              Fx(3, 3) * V_xx_prev(3, 3));
    Q_xu_(3, 1) = Fu(4, 1) * (Fx(0, 3) * V_xx_prev(0, 4) + Fx(1, 3) * V_xx_prev(1, 4) + Fx(2, 3) * V_xx_prev(2, 4) +
                              Fx(3, 3) * V_xx_prev(3, 4));
    Q_xu_(4, 0) = Fu(3, 0) * (Fx(2, 4) * V_xx_prev(2, 3) + Fx(4, 4) * V_xx_prev(4, 3));
    Q_xu_(4, 1) = Fu(4, 1) * (Fx(2, 4) * V_xx_prev(2, 4) + Fx(4, 4) * V_xx_prev(4, 4));

    // Evaluation of Matrix Q_uu_
    Q_uu_(0, 0) = Fu_3_0_p2 * V_xx_prev(3, 3) + Sigma_(0) * g_u_0_0_p2 + Sigma_(1) * g_u_1_0_p2 + l_uu(0, 0);
    Q_uu_(0, 1) = Fu(3, 0) * Fu(4, 1) * V_xx_prev(3, 4);
    Q_uu_(1, 0) = Fu(3, 0) * Fu(4, 1) * V_xx_prev(4, 3);
    Q_uu_(1, 1) = Fu_4_1_p2 * V_xx_prev(4, 4) + Sigma_(2) * g_u_2_1_p2 + Sigma_(3) * g_u_3_1_p2 + l_uu(1, 1);
  }

  virtual void updateQG(const Eigen::VectorXd& sl_x, const Eigen::VectorXd& sl_u, const Eigen::MatrixXd& g_x,
                        const Eigen::MatrixXd& g_u, const Eigen::VectorXd& r_f, const Eigen::MatrixXd& Fx,
                        const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& V_xx_prev,
                        const Eigen::VectorXd& V_x_prev) override {
    // Evaluation of Vector Q_x_
    Q_x_(0) = Fx(0, 0) * (V_x_prev(0) + V_xx_prev(0, 0) * r_f(0) + V_xx_prev(0, 1) * r_f(1) + V_xx_prev(0, 2) * r_f(2) +
                          V_xx_prev(0, 3) * r_f(3) + V_xx_prev(0, 4) * r_f(4)) +
              sl_x(0);
    Q_x_(1) = Fx(1, 1) * (V_x_prev(1) + V_xx_prev(1, 0) * r_f(0) + V_xx_prev(1, 1) * r_f(1) + V_xx_prev(1, 2) * r_f(2) +
                          V_xx_prev(1, 3) * r_f(3) + V_xx_prev(1, 4) * r_f(4)) +
              sl_x(1);
    Q_x_(2) = Fx(0, 2) * (V_x_prev(0) + V_xx_prev(0, 0) * r_f(0) + V_xx_prev(0, 1) * r_f(1) + V_xx_prev(0, 2) * r_f(2) +
                          V_xx_prev(0, 3) * r_f(3) + V_xx_prev(0, 4) * r_f(4)) +
              Fx(1, 2) * (V_x_prev(1) + V_xx_prev(1, 0) * r_f(0) + V_xx_prev(1, 1) * r_f(1) + V_xx_prev(1, 2) * r_f(2) +
                          V_xx_prev(1, 3) * r_f(3) + V_xx_prev(1, 4) * r_f(4)) +
              Fx(2, 2) * (V_x_prev(2) + V_xx_prev(2, 0) * r_f(0) + V_xx_prev(2, 1) * r_f(1) + V_xx_prev(2, 2) * r_f(2) +
                          V_xx_prev(2, 3) * r_f(3) + V_xx_prev(2, 4) * r_f(4)) +
              sl_x(2);
    Q_x_(3) = Fx(0, 3) * (V_x_prev(0) + V_xx_prev(0, 0) * r_f(0) + V_xx_prev(0, 1) * r_f(1) + V_xx_prev(0, 2) * r_f(2) +
                          V_xx_prev(0, 3) * r_f(3) + V_xx_prev(0, 4) * r_f(4)) +
              Fx(1, 3) * (V_x_prev(1) + V_xx_prev(1, 0) * r_f(0) + V_xx_prev(1, 1) * r_f(1) + V_xx_prev(1, 2) * r_f(2) +
                          V_xx_prev(1, 3) * r_f(3) + V_xx_prev(1, 4) * r_f(4)) +
              Fx(2, 3) * (V_x_prev(2) + V_xx_prev(2, 0) * r_f(0) + V_xx_prev(2, 1) * r_f(1) + V_xx_prev(2, 2) * r_f(2) +
                          V_xx_prev(2, 3) * r_f(3) + V_xx_prev(2, 4) * r_f(4)) +
              Fx(3, 3) * (V_x_prev(3) + V_xx_prev(3, 0) * r_f(0) + V_xx_prev(3, 1) * r_f(1) + V_xx_prev(3, 2) * r_f(2) +
                          V_xx_prev(3, 3) * r_f(3) + V_xx_prev(3, 4) * r_f(4)) +
              sl_x(3);
    Q_x_(4) = Fx(2, 4) * (V_x_prev(2) + V_xx_prev(2, 0) * r_f(0) + V_xx_prev(2, 1) * r_f(1) + V_xx_prev(2, 2) * r_f(2) +
                          V_xx_prev(2, 3) * r_f(3) + V_xx_prev(2, 4) * r_f(4)) +
              Fx(4, 4) * (V_x_prev(4) + V_xx_prev(4, 0) * r_f(0) + V_xx_prev(4, 1) * r_f(1) + V_xx_prev(4, 2) * r_f(2) +
                          V_xx_prev(4, 3) * r_f(3) + V_xx_prev(4, 4) * r_f(4)) +
              sl_x(4);

    // Evaluation of Vector Q_u_
    Q_u_(0) = Fu(3, 0) * (V_x_prev(3) + V_xx_prev(3, 0) * r_f(0) + V_xx_prev(3, 1) * r_f(1) + V_xx_prev(3, 2) * r_f(2) +
                          V_xx_prev(3, 3) * r_f(3) + V_xx_prev(3, 4) * r_f(4)) +
              V_(0) * g_u(0, 0) + V_(1) * g_u(1, 0) + sl_u(0);
    Q_u_(1) = Fu(4, 1) * (V_x_prev(4) + V_xx_prev(4, 0) * r_f(0) + V_xx_prev(4, 1) * r_f(1) + V_xx_prev(4, 2) * r_f(2) +
                          V_xx_prev(4, 3) * r_f(3) + V_xx_prev(4, 4) * r_f(4)) +
              V_(2) * g_u(2, 1) + V_(3) * g_u(3, 1) + sl_u(1);
  }

  virtual void updateQuuInv() override {
    // Determine internal variables
    const double Q_uu__0_0_p2 = Q_uu_(0, 0) * Q_uu_(0, 0);
    const double Q_uu__0_1_p2 = Q_uu_(0, 1) * Q_uu_(0, 1);
    const double internal_0 = Q_uu_(1, 1) - Q_uu__0_1_p2 / Q_uu_(0, 0);

    // Evaluation of Matrix Q_uu_inv_
    Q_uu_inv_(0, 0) = Q_uu__0_1_p2 / (Q_uu__0_0_p2 * internal_0) + 1.0 / Q_uu_(0, 0);
    Q_uu_inv_(0, 1) = -Q_uu_(0, 1) / (Q_uu_(0, 0) * internal_0);
    Q_uu_inv_(1, 0) = Q_uu_inv_(0, 1);
    Q_uu_inv_(1, 1) = 1.0 / internal_0;
  }

  virtual void updateKVU(Eigen::MatrixXd& k_ux, Eigen::VectorXd& v_u) override {
    // Evaluation of Matrix k_ux
    k_ux(0, 0) = -Q_uu_inv_(0, 0) * Q_xu_(0, 0) - Q_uu_inv_(0, 1) * Q_xu_(0, 1);
    k_ux(0, 1) = -Q_uu_inv_(0, 0) * Q_xu_(1, 0) - Q_uu_inv_(0, 1) * Q_xu_(1, 1);
    k_ux(0, 2) = -Q_uu_inv_(0, 0) * Q_xu_(2, 0) - Q_uu_inv_(0, 1) * Q_xu_(2, 1);
    k_ux(0, 3) = -Q_uu_inv_(0, 0) * Q_xu_(3, 0) - Q_uu_inv_(0, 1) * Q_xu_(3, 1);
    k_ux(0, 4) = -Q_uu_inv_(0, 0) * Q_xu_(4, 0) - Q_uu_inv_(0, 1) * Q_xu_(4, 1);
    k_ux(1, 0) = -Q_uu_inv_(0, 1) * Q_xu_(0, 0) - Q_uu_inv_(1, 1) * Q_xu_(0, 1);
    k_ux(1, 1) = -Q_uu_inv_(0, 1) * Q_xu_(1, 0) - Q_uu_inv_(1, 1) * Q_xu_(1, 1);
    k_ux(1, 2) = -Q_uu_inv_(0, 1) * Q_xu_(2, 0) - Q_uu_inv_(1, 1) * Q_xu_(2, 1);
    k_ux(1, 3) = -Q_uu_inv_(0, 1) * Q_xu_(3, 0) - Q_uu_inv_(1, 1) * Q_xu_(3, 1);
    k_ux(1, 4) = -Q_uu_inv_(0, 1) * Q_xu_(4, 0) - Q_uu_inv_(1, 1) * Q_xu_(4, 1);

    // Evaluation of Vector v_u
    v_u(0) = -Q_u_(0) * Q_uu_inv_(0, 0) - Q_u_(1) * Q_uu_inv_(0, 1);
    v_u(1) = -Q_u_(0) * Q_uu_inv_(0, 1) - Q_u_(1) * Q_uu_inv_(1, 1);
  }

  virtual void updateQee(const Eigen::MatrixXd& h_u) override {}

  virtual void updateQeeInv() override {}

  virtual void updateQxe(const Eigen::VectorXd& h, const Eigen::MatrixXd& h_x, const Eigen::MatrixXd& h_u,
                         const Eigen::MatrixXd& k_ux, const Eigen::VectorXd& v_u) override {}

  virtual void updateKVE(const Eigen::MatrixXd& h_u, Eigen::MatrixXd& k_ex, Eigen::MatrixXd& k_ue,
                         Eigen::VectorXd& v_e) override {}

  virtual void updateVxx(const Eigen::MatrixXd& k_ux, const Eigen::MatrixXd& k_ex, const Eigen::VectorXd& v_u,
                         const Eigen::VectorXd& v_e, Eigen::MatrixXd& V_xx, Eigen::VectorXd& V_x) override {
    // Evaluation of Matrix V_xx
    V_xx(0, 0) = Q_xu_(0, 0) * k_ux(0, 0) + Q_xu_(0, 1) * k_ux(1, 0) + Q_xx_(0, 0);
    V_xx(0, 1) = Q_xu_(0, 0) * k_ux(0, 1) + Q_xu_(0, 1) * k_ux(1, 1) + Q_xx_(0, 1);
    V_xx(0, 2) = Q_xu_(0, 0) * k_ux(0, 2) + Q_xu_(0, 1) * k_ux(1, 2) + Q_xx_(0, 2);
    V_xx(0, 3) = Q_xu_(0, 0) * k_ux(0, 3) + Q_xu_(0, 1) * k_ux(1, 3) + Q_xx_(0, 3);
    V_xx(0, 4) = Q_xu_(0, 0) * k_ux(0, 4) + Q_xu_(0, 1) * k_ux(1, 4) + Q_xx_(0, 4);
    V_xx(1, 0) = Q_xu_(1, 0) * k_ux(0, 0) + Q_xu_(1, 1) * k_ux(1, 0) + Q_xx_(0, 1);
    V_xx(1, 1) = Q_xu_(1, 0) * k_ux(0, 1) + Q_xu_(1, 1) * k_ux(1, 1) + Q_xx_(1, 1);
    V_xx(1, 2) = Q_xu_(1, 0) * k_ux(0, 2) + Q_xu_(1, 1) * k_ux(1, 2) + Q_xx_(1, 2);
    V_xx(1, 3) = Q_xu_(1, 0) * k_ux(0, 3) + Q_xu_(1, 1) * k_ux(1, 3) + Q_xx_(1, 3);
    V_xx(1, 4) = Q_xu_(1, 0) * k_ux(0, 4) + Q_xu_(1, 1) * k_ux(1, 4) + Q_xx_(1, 4);
    V_xx(2, 0) = Q_xu_(2, 0) * k_ux(0, 0) + Q_xu_(2, 1) * k_ux(1, 0) + Q_xx_(0, 2);
    V_xx(2, 1) = Q_xu_(2, 0) * k_ux(0, 1) + Q_xu_(2, 1) * k_ux(1, 1) + Q_xx_(1, 2);
    V_xx(2, 2) = Q_xu_(2, 0) * k_ux(0, 2) + Q_xu_(2, 1) * k_ux(1, 2) + Q_xx_(2, 2);
    V_xx(2, 3) = Q_xu_(2, 0) * k_ux(0, 3) + Q_xu_(2, 1) * k_ux(1, 3) + Q_xx_(2, 3);
    V_xx(2, 4) = Q_xu_(2, 0) * k_ux(0, 4) + Q_xu_(2, 1) * k_ux(1, 4) + Q_xx_(2, 4);
    V_xx(3, 0) = Q_xu_(3, 0) * k_ux(0, 0) + Q_xu_(3, 1) * k_ux(1, 0) + Q_xx_(0, 3);
    V_xx(3, 1) = Q_xu_(3, 0) * k_ux(0, 1) + Q_xu_(3, 1) * k_ux(1, 1) + Q_xx_(1, 3);
    V_xx(3, 2) = Q_xu_(3, 0) * k_ux(0, 2) + Q_xu_(3, 1) * k_ux(1, 2) + Q_xx_(2, 3);
    V_xx(3, 3) = Q_xu_(3, 0) * k_ux(0, 3) + Q_xu_(3, 1) * k_ux(1, 3) + Q_xx_(3, 3);
    V_xx(3, 4) = Q_xu_(3, 0) * k_ux(0, 4) + Q_xu_(3, 1) * k_ux(1, 4) + Q_xx_(3, 4);
    V_xx(4, 0) = Q_xu_(4, 0) * k_ux(0, 0) + Q_xu_(4, 1) * k_ux(1, 0) + Q_xx_(0, 4);
    V_xx(4, 1) = Q_xu_(4, 0) * k_ux(0, 1) + Q_xu_(4, 1) * k_ux(1, 1) + Q_xx_(1, 4);
    V_xx(4, 2) = Q_xu_(4, 0) * k_ux(0, 2) + Q_xu_(4, 1) * k_ux(1, 2) + Q_xx_(2, 4);
    V_xx(4, 3) = Q_xu_(4, 0) * k_ux(0, 3) + Q_xu_(4, 1) * k_ux(1, 3) + Q_xx_(3, 4);
    V_xx(4, 4) = Q_xu_(4, 0) * k_ux(0, 4) + Q_xu_(4, 1) * k_ux(1, 4) + Q_xx_(4, 4);

    // Evaluation of Vector V_x
    V_x(0) = Q_x_(0) + Q_xu_(0, 0) * v_u(0) + Q_xu_(0, 1) * v_u(1);
    V_x(1) = Q_x_(1) + Q_xu_(1, 0) * v_u(0) + Q_xu_(1, 1) * v_u(1);
    V_x(2) = Q_x_(2) + Q_xu_(2, 0) * v_u(0) + Q_xu_(2, 1) * v_u(1);
    V_x(3) = Q_x_(3) + Q_xu_(3, 0) * v_u(0) + Q_xu_(3, 1) * v_u(1);
    V_x(4) = Q_x_(4) + Q_xu_(4, 0) * v_u(0) + Q_xu_(4, 1) * v_u(1);
  }

  virtual void updateQnx(const Eigen::VectorXd& r_f, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& Fu,
                         const Eigen::MatrixXd& h_u, const Eigen::MatrixXd& k_ux, const Eigen::MatrixXd& k_ex,
                         const Eigen::MatrixXd& k_ue, const Eigen::VectorXd& v_u, const Eigen::VectorXd& v_e) override {
    // Evaluation of Matrix Q_un_
    Q_un_(0, 3) = -Fu(3, 0) * Q_uu_inv_(0, 0);
    Q_un_(0, 4) = -Fu(4, 1) * Q_uu_inv_(0, 1);
    Q_un_(1, 3) = -Fu(3, 0) * Q_uu_inv_(0, 1);
    Q_un_(1, 4) = -Fu(4, 1) * Q_uu_inv_(1, 1);

    // Evaluation of Matrix Q_nuu_
    Q_nuu_(0, 0) = -Q_uu_(0, 0);
    Q_nuu_(0, 1) = -Q_uu_(0, 1);
    Q_nuu_(1, 0) = -Q_uu_(0, 1);
    Q_nuu_(1, 1) = -Q_uu_(1, 1);

    // Evaluation of Matrix Q_vnx_
    Q_vnx_(0, 0) = Fx(0, 0);
    Q_vnx_(0, 2) = Fx(0, 2);
    Q_vnx_(0, 3) = Fx(0, 3);
    Q_vnx_(1, 1) = Fx(1, 1);
    Q_vnx_(1, 2) = Fx(1, 2);
    Q_vnx_(1, 3) = Fx(1, 3);
    Q_vnx_(2, 2) = Fx(2, 2);
    Q_vnx_(2, 3) = Fx(2, 3);
    Q_vnx_(2, 4) = Fx(2, 4);
    Q_vnx_(3, 0) = Fu(3, 0) * k_ux(0, 0);
    Q_vnx_(3, 1) = Fu(3, 0) * k_ux(0, 1);
    Q_vnx_(3, 2) = Fu(3, 0) * k_ux(0, 2);
    Q_vnx_(3, 3) = Fu(3, 0) * k_ux(0, 3) + Fx(3, 3);
    Q_vnx_(3, 4) = Fu(3, 0) * k_ux(0, 4);
    Q_vnx_(4, 0) = Fu(4, 1) * k_ux(1, 0);
    Q_vnx_(4, 1) = Fu(4, 1) * k_ux(1, 1);
    Q_vnx_(4, 2) = Fu(4, 1) * k_ux(1, 2);
    Q_vnx_(4, 3) = Fu(4, 1) * k_ux(1, 3);
    Q_vnx_(4, 4) = Fu(4, 1) * k_ux(1, 4) + Fx(4, 4);

    // Evaluation of Vector Q_vn_
    Q_vn_(0) = r_f(0);
    Q_vn_(1) = r_f(1);
    Q_vn_(2) = r_f(2);
    Q_vn_(3) = Fu(3, 0) * v_u(0) + r_f(3);
    Q_vn_(4) = Fu(4, 1) * v_u(1) + r_f(4);
  }
};

template <>
class BicycleTrajectoryTrackerIpmEvaluator<StageType::NORMINAL, OcpConfig::ERK4> : public IpmEvaluator {
 public:
  BicycleTrajectoryTrackerIpmEvaluator() : IpmEvaluator(5, 2, 8, 0, 0) {}
  virtual ~BicycleTrajectoryTrackerIpmEvaluator() = default;

  virtual void updateLX(const Eigen::VectorXd& l_x, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& g_x,
                        const Eigen::MatrixXd& h_x, const Eigen::MatrixXd& m_x, const Eigen::VectorXd& p,
                        const Eigen::VectorXd& lambda, const Eigen::VectorXd& eta, const Eigen::VectorXd& nu,
                        Eigen::VectorXd& sl_x) const override {
    // Evaluation of Vector sl_x
    sl_x(0) = Fx(0, 0) * p(0) + l_x(0);
    sl_x(1) = Fx(1, 1) * p(1) + l_x(1);
    sl_x(2) = Fx(0, 2) * p(0) + Fx(1, 2) * p(1) + Fx(2, 2) * p(2) + l_x(2);
    sl_x(3) = Fx(0, 3) * p(0) + Fx(1, 3) * p(1) + Fx(2, 3) * p(2) + Fx(3, 3) * p(3) + g_x(0, 3) * lambda(0) +
              g_x(1, 3) * lambda(1) + l_x(3);
    sl_x(4) = Fx(0, 4) * p(0) + Fx(1, 4) * p(1) + Fx(2, 4) * p(2) + Fx(4, 4) * p(4) + g_x(2, 4) * lambda(2) +
              g_x(3, 4) * lambda(3) + l_x(4);
  }

  virtual void updateLU(const Eigen::VectorXd& l_u, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& g_u,
                        const Eigen::MatrixXd& h_u, const Eigen::VectorXd& p, const Eigen::VectorXd& lambda,
                        const Eigen::VectorXd& eta, Eigen::VectorXd& sl_u) const override {
    // Evaluation of Vector sl_u
    sl_u(0) = Fu(0, 0) * p(0) + Fu(1, 0) * p(1) + Fu(2, 0) * p(2) + Fu(3, 0) * p(3) + g_u(4, 0) * lambda(4) +
              g_u(5, 0) * lambda(5) + l_u(0);
    sl_u(1) = Fu(0, 1) * p(0) + Fu(1, 1) * p(1) + Fu(2, 1) * p(2) + Fu(4, 1) * p(4) + g_u(6, 1) * lambda(6) +
              g_u(7, 1) * lambda(7) + l_u(1);
  }

  virtual void updateQH(const Eigen::MatrixXd& l_xx, const Eigen::MatrixXd& l_xu, const Eigen::MatrixXd& l_uu,
                        const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::MatrixXd& Fx,
                        const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& V_xx_prev) override {
    // Determine internal variables
    const double Fx_0_0_p2 = Fx(0, 0) * Fx(0, 0);
    const double Fx_1_1_p2 = Fx(1, 1) * Fx(1, 1);
    const double g_u_4_0_p2 = g_u(4, 0) * g_u(4, 0);
    const double g_u_5_0_p2 = g_u(5, 0) * g_u(5, 0);
    const double g_u_6_1_p2 = g_u(6, 1) * g_u(6, 1);
    const double g_u_7_1_p2 = g_u(7, 1) * g_u(7, 1);
    const double g_x_0_3_p2 = g_x(0, 3) * g_x(0, 3);
    const double g_x_1_3_p2 = g_x(1, 3) * g_x(1, 3);
    const double g_x_2_4_p2 = g_x(2, 4) * g_x(2, 4);
    const double g_x_3_4_p2 = g_x(3, 4) * g_x(3, 4);

    // Evaluation of Matrix Q_xx_
    Q_xx_(0, 0) = Fx_0_0_p2 * V_xx_prev(0, 0) + l_xx(0, 0);
    Q_xx_(0, 1) = Fx(0, 0) * Fx(1, 1) * V_xx_prev(0, 1);
    Q_xx_(0, 2) = Fx(0, 0) * Fx(0, 2) * V_xx_prev(0, 0) + Fx(0, 0) * Fx(1, 2) * V_xx_prev(0, 1) +
                  Fx(0, 0) * Fx(2, 2) * V_xx_prev(0, 2);
    Q_xx_(0, 3) = Fx(0, 0) * Fx(0, 3) * V_xx_prev(0, 0) + Fx(0, 0) * Fx(1, 3) * V_xx_prev(0, 1) +
                  Fx(0, 0) * Fx(2, 3) * V_xx_prev(0, 2) + Fx(0, 0) * Fx(3, 3) * V_xx_prev(0, 3);
    Q_xx_(0, 4) = Fx(0, 0) * Fx(0, 4) * V_xx_prev(0, 0) + Fx(0, 0) * Fx(1, 4) * V_xx_prev(0, 1) +
                  Fx(0, 0) * Fx(2, 4) * V_xx_prev(0, 2) + Fx(0, 0) * Fx(4, 4) * V_xx_prev(0, 4);
    Q_xx_(1, 0) = Fx(0, 0) * Fx(1, 1) * V_xx_prev(1, 0);
    Q_xx_(1, 1) = Fx_1_1_p2 * V_xx_prev(1, 1) + l_xx(1, 1);
    Q_xx_(1, 2) = Fx(0, 2) * Fx(1, 1) * V_xx_prev(1, 0) + Fx(1, 1) * Fx(1, 2) * V_xx_prev(1, 1) +
                  Fx(1, 1) * Fx(2, 2) * V_xx_prev(1, 2);
    Q_xx_(1, 3) = Fx(0, 3) * Fx(1, 1) * V_xx_prev(1, 0) + Fx(1, 1) * Fx(1, 3) * V_xx_prev(1, 1) +
                  Fx(1, 1) * Fx(2, 3) * V_xx_prev(1, 2) + Fx(1, 1) * Fx(3, 3) * V_xx_prev(1, 3);
    Q_xx_(1, 4) = Fx(0, 4) * Fx(1, 1) * V_xx_prev(1, 0) + Fx(1, 1) * Fx(1, 4) * V_xx_prev(1, 1) +
                  Fx(1, 1) * Fx(2, 4) * V_xx_prev(1, 2) + Fx(1, 1) * Fx(4, 4) * V_xx_prev(1, 4);
    Q_xx_(2, 0) = Fx(0, 0) * (Fx(0, 2) * V_xx_prev(0, 0) + Fx(1, 2) * V_xx_prev(1, 0) + Fx(2, 2) * V_xx_prev(2, 0));
    Q_xx_(2, 1) = Fx(1, 1) * (Fx(0, 2) * V_xx_prev(0, 1) + Fx(1, 2) * V_xx_prev(1, 1) + Fx(2, 2) * V_xx_prev(2, 1));
    Q_xx_(2, 2) = Fx(0, 2) * (Fx(0, 2) * V_xx_prev(0, 0) + Fx(1, 2) * V_xx_prev(1, 0) + Fx(2, 2) * V_xx_prev(2, 0)) +
                  Fx(1, 2) * (Fx(0, 2) * V_xx_prev(0, 1) + Fx(1, 2) * V_xx_prev(1, 1) + Fx(2, 2) * V_xx_prev(2, 1)) +
                  Fx(2, 2) * (Fx(0, 2) * V_xx_prev(0, 2) + Fx(1, 2) * V_xx_prev(1, 2) + Fx(2, 2) * V_xx_prev(2, 2)) +
                  l_xx(2, 2);
    Q_xx_(2, 3) = Fx(0, 3) * (Fx(0, 2) * V_xx_prev(0, 0) + Fx(1, 2) * V_xx_prev(1, 0) + Fx(2, 2) * V_xx_prev(2, 0)) +
                  Fx(1, 3) * (Fx(0, 2) * V_xx_prev(0, 1) + Fx(1, 2) * V_xx_prev(1, 1) + Fx(2, 2) * V_xx_prev(2, 1)) +
                  Fx(2, 3) * (Fx(0, 2) * V_xx_prev(0, 2) + Fx(1, 2) * V_xx_prev(1, 2) + Fx(2, 2) * V_xx_prev(2, 2)) +
                  Fx(3, 3) * (Fx(0, 2) * V_xx_prev(0, 3) + Fx(1, 2) * V_xx_prev(1, 3) + Fx(2, 2) * V_xx_prev(2, 3));
    Q_xx_(2, 4) = Fx(0, 4) * (Fx(0, 2) * V_xx_prev(0, 0) + Fx(1, 2) * V_xx_prev(1, 0) + Fx(2, 2) * V_xx_prev(2, 0)) +
                  Fx(1, 4) * (Fx(0, 2) * V_xx_prev(0, 1) + Fx(1, 2) * V_xx_prev(1, 1) + Fx(2, 2) * V_xx_prev(2, 1)) +
                  Fx(2, 4) * (Fx(0, 2) * V_xx_prev(0, 2) + Fx(1, 2) * V_xx_prev(1, 2) + Fx(2, 2) * V_xx_prev(2, 2)) +
                  Fx(4, 4) * (Fx(0, 2) * V_xx_prev(0, 4) + Fx(1, 2) * V_xx_prev(1, 4) + Fx(2, 2) * V_xx_prev(2, 4));
    Q_xx_(3, 0) = Fx(0, 0) * (Fx(0, 3) * V_xx_prev(0, 0) + Fx(1, 3) * V_xx_prev(1, 0) + Fx(2, 3) * V_xx_prev(2, 0) +
                              Fx(3, 3) * V_xx_prev(3, 0));
    Q_xx_(3, 1) = Fx(1, 1) * (Fx(0, 3) * V_xx_prev(0, 1) + Fx(1, 3) * V_xx_prev(1, 1) + Fx(2, 3) * V_xx_prev(2, 1) +
                              Fx(3, 3) * V_xx_prev(3, 1));
    Q_xx_(3, 2) = Fx(0, 2) * (Fx(0, 3) * V_xx_prev(0, 0) + Fx(1, 3) * V_xx_prev(1, 0) + Fx(2, 3) * V_xx_prev(2, 0) +
                              Fx(3, 3) * V_xx_prev(3, 0)) +
                  Fx(1, 2) * (Fx(0, 3) * V_xx_prev(0, 1) + Fx(1, 3) * V_xx_prev(1, 1) + Fx(2, 3) * V_xx_prev(2, 1) +
                              Fx(3, 3) * V_xx_prev(3, 1)) +
                  Fx(2, 2) * (Fx(0, 3) * V_xx_prev(0, 2) + Fx(1, 3) * V_xx_prev(1, 2) + Fx(2, 3) * V_xx_prev(2, 2) +
                              Fx(3, 3) * V_xx_prev(3, 2));
    Q_xx_(3, 3) = Fx(0, 3) * (Fx(0, 3) * V_xx_prev(0, 0) + Fx(1, 3) * V_xx_prev(1, 0) + Fx(2, 3) * V_xx_prev(2, 0) +
                              Fx(3, 3) * V_xx_prev(3, 0)) +
                  Fx(1, 3) * (Fx(0, 3) * V_xx_prev(0, 1) + Fx(1, 3) * V_xx_prev(1, 1) + Fx(2, 3) * V_xx_prev(2, 1) +
                              Fx(3, 3) * V_xx_prev(3, 1)) +
                  Fx(2, 3) * (Fx(0, 3) * V_xx_prev(0, 2) + Fx(1, 3) * V_xx_prev(1, 2) + Fx(2, 3) * V_xx_prev(2, 2) +
                              Fx(3, 3) * V_xx_prev(3, 2)) +
                  Fx(3, 3) * (Fx(0, 3) * V_xx_prev(0, 3) + Fx(1, 3) * V_xx_prev(1, 3) + Fx(2, 3) * V_xx_prev(2, 3) +
                              Fx(3, 3) * V_xx_prev(3, 3)) +
                  Sigma_(0) * g_x_0_3_p2 + Sigma_(1) * g_x_1_3_p2 + l_xx(3, 3);
    Q_xx_(3, 4) = Fx(0, 4) * (Fx(0, 3) * V_xx_prev(0, 0) + Fx(1, 3) * V_xx_prev(1, 0) + Fx(2, 3) * V_xx_prev(2, 0) +
                              Fx(3, 3) * V_xx_prev(3, 0)) +
                  Fx(1, 4) * (Fx(0, 3) * V_xx_prev(0, 1) + Fx(1, 3) * V_xx_prev(1, 1) + Fx(2, 3) * V_xx_prev(2, 1) +
                              Fx(3, 3) * V_xx_prev(3, 1)) +
                  Fx(2, 4) * (Fx(0, 3) * V_xx_prev(0, 2) + Fx(1, 3) * V_xx_prev(1, 2) + Fx(2, 3) * V_xx_prev(2, 2) +
                              Fx(3, 3) * V_xx_prev(3, 2)) +
                  Fx(4, 4) * (Fx(0, 3) * V_xx_prev(0, 4) + Fx(1, 3) * V_xx_prev(1, 4) + Fx(2, 3) * V_xx_prev(2, 4) +
                              Fx(3, 3) * V_xx_prev(3, 4));
    Q_xx_(4, 0) = Fx(0, 0) * (Fx(0, 4) * V_xx_prev(0, 0) + Fx(1, 4) * V_xx_prev(1, 0) + Fx(2, 4) * V_xx_prev(2, 0) +
                              Fx(4, 4) * V_xx_prev(4, 0));
    Q_xx_(4, 1) = Fx(1, 1) * (Fx(0, 4) * V_xx_prev(0, 1) + Fx(1, 4) * V_xx_prev(1, 1) + Fx(2, 4) * V_xx_prev(2, 1) +
                              Fx(4, 4) * V_xx_prev(4, 1));
    Q_xx_(4, 2) = Fx(0, 2) * (Fx(0, 4) * V_xx_prev(0, 0) + Fx(1, 4) * V_xx_prev(1, 0) + Fx(2, 4) * V_xx_prev(2, 0) +
                              Fx(4, 4) * V_xx_prev(4, 0)) +
                  Fx(1, 2) * (Fx(0, 4) * V_xx_prev(0, 1) + Fx(1, 4) * V_xx_prev(1, 1) + Fx(2, 4) * V_xx_prev(2, 1) +
                              Fx(4, 4) * V_xx_prev(4, 1)) +
                  Fx(2, 2) * (Fx(0, 4) * V_xx_prev(0, 2) + Fx(1, 4) * V_xx_prev(1, 2) + Fx(2, 4) * V_xx_prev(2, 2) +
                              Fx(4, 4) * V_xx_prev(4, 2));
    Q_xx_(4, 3) = Fx(0, 3) * (Fx(0, 4) * V_xx_prev(0, 0) + Fx(1, 4) * V_xx_prev(1, 0) + Fx(2, 4) * V_xx_prev(2, 0) +
                              Fx(4, 4) * V_xx_prev(4, 0)) +
                  Fx(1, 3) * (Fx(0, 4) * V_xx_prev(0, 1) + Fx(1, 4) * V_xx_prev(1, 1) + Fx(2, 4) * V_xx_prev(2, 1) +
                              Fx(4, 4) * V_xx_prev(4, 1)) +
                  Fx(2, 3) * (Fx(0, 4) * V_xx_prev(0, 2) + Fx(1, 4) * V_xx_prev(1, 2) + Fx(2, 4) * V_xx_prev(2, 2) +
                              Fx(4, 4) * V_xx_prev(4, 2)) +
                  Fx(3, 3) * (Fx(0, 4) * V_xx_prev(0, 3) + Fx(1, 4) * V_xx_prev(1, 3) + Fx(2, 4) * V_xx_prev(2, 3) +
                              Fx(4, 4) * V_xx_prev(4, 3));
    Q_xx_(4, 4) = Fx(0, 4) * (Fx(0, 4) * V_xx_prev(0, 0) + Fx(1, 4) * V_xx_prev(1, 0) + Fx(2, 4) * V_xx_prev(2, 0) +
                              Fx(4, 4) * V_xx_prev(4, 0)) +
                  Fx(1, 4) * (Fx(0, 4) * V_xx_prev(0, 1) + Fx(1, 4) * V_xx_prev(1, 1) + Fx(2, 4) * V_xx_prev(2, 1) +
                              Fx(4, 4) * V_xx_prev(4, 1)) +
                  Fx(2, 4) * (Fx(0, 4) * V_xx_prev(0, 2) + Fx(1, 4) * V_xx_prev(1, 2) + Fx(2, 4) * V_xx_prev(2, 2) +
                              Fx(4, 4) * V_xx_prev(4, 2)) +
                  Fx(4, 4) * (Fx(0, 4) * V_xx_prev(0, 4) + Fx(1, 4) * V_xx_prev(1, 4) + Fx(2, 4) * V_xx_prev(2, 4) +
                              Fx(4, 4) * V_xx_prev(4, 4)) +
                  Sigma_(2) * g_x_2_4_p2 + Sigma_(3) * g_x_3_4_p2 + l_xx(4, 4);

    // Evaluation of Matrix Q_xu_
    Q_xu_(0, 0) = Fu(0, 0) * Fx(0, 0) * V_xx_prev(0, 0) + Fu(1, 0) * Fx(0, 0) * V_xx_prev(0, 1) +
                  Fu(2, 0) * Fx(0, 0) * V_xx_prev(0, 2) + Fu(3, 0) * Fx(0, 0) * V_xx_prev(0, 3);
    Q_xu_(0, 1) = Fu(0, 1) * Fx(0, 0) * V_xx_prev(0, 0) + Fu(1, 1) * Fx(0, 0) * V_xx_prev(0, 1) +
                  Fu(2, 1) * Fx(0, 0) * V_xx_prev(0, 2) + Fu(4, 1) * Fx(0, 0) * V_xx_prev(0, 4);
    Q_xu_(1, 0) = Fu(0, 0) * Fx(1, 1) * V_xx_prev(1, 0) + Fu(1, 0) * Fx(1, 1) * V_xx_prev(1, 1) +
                  Fu(2, 0) * Fx(1, 1) * V_xx_prev(1, 2) + Fu(3, 0) * Fx(1, 1) * V_xx_prev(1, 3);
    Q_xu_(1, 1) = Fu(0, 1) * Fx(1, 1) * V_xx_prev(1, 0) + Fu(1, 1) * Fx(1, 1) * V_xx_prev(1, 1) +
                  Fu(2, 1) * Fx(1, 1) * V_xx_prev(1, 2) + Fu(4, 1) * Fx(1, 1) * V_xx_prev(1, 4);
    Q_xu_(2, 0) = Fu(0, 0) * (Fx(0, 2) * V_xx_prev(0, 0) + Fx(1, 2) * V_xx_prev(1, 0) + Fx(2, 2) * V_xx_prev(2, 0)) +
                  Fu(1, 0) * (Fx(0, 2) * V_xx_prev(0, 1) + Fx(1, 2) * V_xx_prev(1, 1) + Fx(2, 2) * V_xx_prev(2, 1)) +
                  Fu(2, 0) * (Fx(0, 2) * V_xx_prev(0, 2) + Fx(1, 2) * V_xx_prev(1, 2) + Fx(2, 2) * V_xx_prev(2, 2)) +
                  Fu(3, 0) * (Fx(0, 2) * V_xx_prev(0, 3) + Fx(1, 2) * V_xx_prev(1, 3) + Fx(2, 2) * V_xx_prev(2, 3));
    Q_xu_(2, 1) = Fu(0, 1) * (Fx(0, 2) * V_xx_prev(0, 0) + Fx(1, 2) * V_xx_prev(1, 0) + Fx(2, 2) * V_xx_prev(2, 0)) +
                  Fu(1, 1) * (Fx(0, 2) * V_xx_prev(0, 1) + Fx(1, 2) * V_xx_prev(1, 1) + Fx(2, 2) * V_xx_prev(2, 1)) +
                  Fu(2, 1) * (Fx(0, 2) * V_xx_prev(0, 2) + Fx(1, 2) * V_xx_prev(1, 2) + Fx(2, 2) * V_xx_prev(2, 2)) +
                  Fu(4, 1) * (Fx(0, 2) * V_xx_prev(0, 4) + Fx(1, 2) * V_xx_prev(1, 4) + Fx(2, 2) * V_xx_prev(2, 4));
    Q_xu_(3, 0) = Fu(0, 0) * (Fx(0, 3) * V_xx_prev(0, 0) + Fx(1, 3) * V_xx_prev(1, 0) + Fx(2, 3) * V_xx_prev(2, 0) +
                              Fx(3, 3) * V_xx_prev(3, 0)) +
                  Fu(1, 0) * (Fx(0, 3) * V_xx_prev(0, 1) + Fx(1, 3) * V_xx_prev(1, 1) + Fx(2, 3) * V_xx_prev(2, 1) +
                              Fx(3, 3) * V_xx_prev(3, 1)) +
                  Fu(2, 0) * (Fx(0, 3) * V_xx_prev(0, 2) + Fx(1, 3) * V_xx_prev(1, 2) + Fx(2, 3) * V_xx_prev(2, 2) +
                              Fx(3, 3) * V_xx_prev(3, 2)) +
                  Fu(3, 0) * (Fx(0, 3) * V_xx_prev(0, 3) + Fx(1, 3) * V_xx_prev(1, 3) + Fx(2, 3) * V_xx_prev(2, 3) +
                              Fx(3, 3) * V_xx_prev(3, 3));
    Q_xu_(3, 1) = Fu(0, 1) * (Fx(0, 3) * V_xx_prev(0, 0) + Fx(1, 3) * V_xx_prev(1, 0) + Fx(2, 3) * V_xx_prev(2, 0) +
                              Fx(3, 3) * V_xx_prev(3, 0)) +
                  Fu(1, 1) * (Fx(0, 3) * V_xx_prev(0, 1) + Fx(1, 3) * V_xx_prev(1, 1) + Fx(2, 3) * V_xx_prev(2, 1) +
                              Fx(3, 3) * V_xx_prev(3, 1)) +
                  Fu(2, 1) * (Fx(0, 3) * V_xx_prev(0, 2) + Fx(1, 3) * V_xx_prev(1, 2) + Fx(2, 3) * V_xx_prev(2, 2) +
                              Fx(3, 3) * V_xx_prev(3, 2)) +
                  Fu(4, 1) * (Fx(0, 3) * V_xx_prev(0, 4) + Fx(1, 3) * V_xx_prev(1, 4) + Fx(2, 3) * V_xx_prev(2, 4) +
                              Fx(3, 3) * V_xx_prev(3, 4));
    Q_xu_(4, 0) = Fu(0, 0) * (Fx(0, 4) * V_xx_prev(0, 0) + Fx(1, 4) * V_xx_prev(1, 0) + Fx(2, 4) * V_xx_prev(2, 0) +
                              Fx(4, 4) * V_xx_prev(4, 0)) +
                  Fu(1, 0) * (Fx(0, 4) * V_xx_prev(0, 1) + Fx(1, 4) * V_xx_prev(1, 1) + Fx(2, 4) * V_xx_prev(2, 1) +
                              Fx(4, 4) * V_xx_prev(4, 1)) +
                  Fu(2, 0) * (Fx(0, 4) * V_xx_prev(0, 2) + Fx(1, 4) * V_xx_prev(1, 2) + Fx(2, 4) * V_xx_prev(2, 2) +
                              Fx(4, 4) * V_xx_prev(4, 2)) +
                  Fu(3, 0) * (Fx(0, 4) * V_xx_prev(0, 3) + Fx(1, 4) * V_xx_prev(1, 3) + Fx(2, 4) * V_xx_prev(2, 3) +
                              Fx(4, 4) * V_xx_prev(4, 3));
    Q_xu_(4, 1) = Fu(0, 1) * (Fx(0, 4) * V_xx_prev(0, 0) + Fx(1, 4) * V_xx_prev(1, 0) + Fx(2, 4) * V_xx_prev(2, 0) +
                              Fx(4, 4) * V_xx_prev(4, 0)) +
                  Fu(1, 1) * (Fx(0, 4) * V_xx_prev(0, 1) + Fx(1, 4) * V_xx_prev(1, 1) + Fx(2, 4) * V_xx_prev(2, 1) +
                              Fx(4, 4) * V_xx_prev(4, 1)) +
                  Fu(2, 1) * (Fx(0, 4) * V_xx_prev(0, 2) + Fx(1, 4) * V_xx_prev(1, 2) + Fx(2, 4) * V_xx_prev(2, 2) +
                              Fx(4, 4) * V_xx_prev(4, 2)) +
                  Fu(4, 1) * (Fx(0, 4) * V_xx_prev(0, 4) + Fx(1, 4) * V_xx_prev(1, 4) + Fx(2, 4) * V_xx_prev(2, 4) +
                              Fx(4, 4) * V_xx_prev(4, 4));

    // Evaluation of Matrix Q_uu_
    Q_uu_(0, 0) = Fu(0, 0) * (Fu(0, 0) * V_xx_prev(0, 0) + Fu(1, 0) * V_xx_prev(1, 0) + Fu(2, 0) * V_xx_prev(2, 0) +
                              Fu(3, 0) * V_xx_prev(3, 0)) +
                  Fu(1, 0) * (Fu(0, 0) * V_xx_prev(0, 1) + Fu(1, 0) * V_xx_prev(1, 1) + Fu(2, 0) * V_xx_prev(2, 1) +
                              Fu(3, 0) * V_xx_prev(3, 1)) +
                  Fu(2, 0) * (Fu(0, 0) * V_xx_prev(0, 2) + Fu(1, 0) * V_xx_prev(1, 2) + Fu(2, 0) * V_xx_prev(2, 2) +
                              Fu(3, 0) * V_xx_prev(3, 2)) +
                  Fu(3, 0) * (Fu(0, 0) * V_xx_prev(0, 3) + Fu(1, 0) * V_xx_prev(1, 3) + Fu(2, 0) * V_xx_prev(2, 3) +
                              Fu(3, 0) * V_xx_prev(3, 3)) +
                  Sigma_(4) * g_u_4_0_p2 + Sigma_(5) * g_u_5_0_p2 + l_uu(0, 0);
    Q_uu_(0, 1) = Fu(0, 1) * (Fu(0, 0) * V_xx_prev(0, 0) + Fu(1, 0) * V_xx_prev(1, 0) + Fu(2, 0) * V_xx_prev(2, 0) +
                              Fu(3, 0) * V_xx_prev(3, 0)) +
                  Fu(1, 1) * (Fu(0, 0) * V_xx_prev(0, 1) + Fu(1, 0) * V_xx_prev(1, 1) + Fu(2, 0) * V_xx_prev(2, 1) +
                              Fu(3, 0) * V_xx_prev(3, 1)) +
                  Fu(2, 1) * (Fu(0, 0) * V_xx_prev(0, 2) + Fu(1, 0) * V_xx_prev(1, 2) + Fu(2, 0) * V_xx_prev(2, 2) +
                              Fu(3, 0) * V_xx_prev(3, 2)) +
                  Fu(4, 1) * (Fu(0, 0) * V_xx_prev(0, 4) + Fu(1, 0) * V_xx_prev(1, 4) + Fu(2, 0) * V_xx_prev(2, 4) +
                              Fu(3, 0) * V_xx_prev(3, 4));
    Q_uu_(1, 0) = Fu(0, 0) * (Fu(0, 1) * V_xx_prev(0, 0) + Fu(1, 1) * V_xx_prev(1, 0) + Fu(2, 1) * V_xx_prev(2, 0) +
                              Fu(4, 1) * V_xx_prev(4, 0)) +
                  Fu(1, 0) * (Fu(0, 1) * V_xx_prev(0, 1) + Fu(1, 1) * V_xx_prev(1, 1) + Fu(2, 1) * V_xx_prev(2, 1) +
                              Fu(4, 1) * V_xx_prev(4, 1)) +
                  Fu(2, 0) * (Fu(0, 1) * V_xx_prev(0, 2) + Fu(1, 1) * V_xx_prev(1, 2) + Fu(2, 1) * V_xx_prev(2, 2) +
                              Fu(4, 1) * V_xx_prev(4, 2)) +
                  Fu(3, 0) * (Fu(0, 1) * V_xx_prev(0, 3) + Fu(1, 1) * V_xx_prev(1, 3) + Fu(2, 1) * V_xx_prev(2, 3) +
                              Fu(4, 1) * V_xx_prev(4, 3));
    Q_uu_(1, 1) = Fu(0, 1) * (Fu(0, 1) * V_xx_prev(0, 0) + Fu(1, 1) * V_xx_prev(1, 0) + Fu(2, 1) * V_xx_prev(2, 0) +
                              Fu(4, 1) * V_xx_prev(4, 0)) +
                  Fu(1, 1) * (Fu(0, 1) * V_xx_prev(0, 1) + Fu(1, 1) * V_xx_prev(1, 1) + Fu(2, 1) * V_xx_prev(2, 1) +
                              Fu(4, 1) * V_xx_prev(4, 1)) +
                  Fu(2, 1) * (Fu(0, 1) * V_xx_prev(0, 2) + Fu(1, 1) * V_xx_prev(1, 2) + Fu(2, 1) * V_xx_prev(2, 2) +
                              Fu(4, 1) * V_xx_prev(4, 2)) +
                  Fu(4, 1) * (Fu(0, 1) * V_xx_prev(0, 4) + Fu(1, 1) * V_xx_prev(1, 4) + Fu(2, 1) * V_xx_prev(2, 4) +
                              Fu(4, 1) * V_xx_prev(4, 4)) +
                  Sigma_(6) * g_u_6_1_p2 + Sigma_(7) * g_u_7_1_p2 + l_uu(1, 1);
  }

  virtual void updateQG(const Eigen::VectorXd& sl_x, const Eigen::VectorXd& sl_u, const Eigen::MatrixXd& g_x,
                        const Eigen::MatrixXd& g_u, const Eigen::VectorXd& r_f, const Eigen::MatrixXd& Fx,
                        const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& V_xx_prev,
                        const Eigen::VectorXd& V_x_prev) override {
    // Evaluation of Vector Q_x_
    Q_x_(0) = Fx(0, 0) * (V_x_prev(0) + V_xx_prev(0, 0) * r_f(0) + V_xx_prev(0, 1) * r_f(1) + V_xx_prev(0, 2) * r_f(2) +
                          V_xx_prev(0, 3) * r_f(3) + V_xx_prev(0, 4) * r_f(4)) +
              sl_x(0);
    Q_x_(1) = Fx(1, 1) * (V_x_prev(1) + V_xx_prev(1, 0) * r_f(0) + V_xx_prev(1, 1) * r_f(1) + V_xx_prev(1, 2) * r_f(2) +
                          V_xx_prev(1, 3) * r_f(3) + V_xx_prev(1, 4) * r_f(4)) +
              sl_x(1);
    Q_x_(2) = Fx(0, 2) * (V_x_prev(0) + V_xx_prev(0, 0) * r_f(0) + V_xx_prev(0, 1) * r_f(1) + V_xx_prev(0, 2) * r_f(2) +
                          V_xx_prev(0, 3) * r_f(3) + V_xx_prev(0, 4) * r_f(4)) +
              Fx(1, 2) * (V_x_prev(1) + V_xx_prev(1, 0) * r_f(0) + V_xx_prev(1, 1) * r_f(1) + V_xx_prev(1, 2) * r_f(2) +
                          V_xx_prev(1, 3) * r_f(3) + V_xx_prev(1, 4) * r_f(4)) +
              Fx(2, 2) * (V_x_prev(2) + V_xx_prev(2, 0) * r_f(0) + V_xx_prev(2, 1) * r_f(1) + V_xx_prev(2, 2) * r_f(2) +
                          V_xx_prev(2, 3) * r_f(3) + V_xx_prev(2, 4) * r_f(4)) +
              sl_x(2);
    Q_x_(3) = Fx(0, 3) * (V_x_prev(0) + V_xx_prev(0, 0) * r_f(0) + V_xx_prev(0, 1) * r_f(1) + V_xx_prev(0, 2) * r_f(2) +
                          V_xx_prev(0, 3) * r_f(3) + V_xx_prev(0, 4) * r_f(4)) +
              Fx(1, 3) * (V_x_prev(1) + V_xx_prev(1, 0) * r_f(0) + V_xx_prev(1, 1) * r_f(1) + V_xx_prev(1, 2) * r_f(2) +
                          V_xx_prev(1, 3) * r_f(3) + V_xx_prev(1, 4) * r_f(4)) +
              Fx(2, 3) * (V_x_prev(2) + V_xx_prev(2, 0) * r_f(0) + V_xx_prev(2, 1) * r_f(1) + V_xx_prev(2, 2) * r_f(2) +
                          V_xx_prev(2, 3) * r_f(3) + V_xx_prev(2, 4) * r_f(4)) +
              Fx(3, 3) * (V_x_prev(3) + V_xx_prev(3, 0) * r_f(0) + V_xx_prev(3, 1) * r_f(1) + V_xx_prev(3, 2) * r_f(2) +
                          V_xx_prev(3, 3) * r_f(3) + V_xx_prev(3, 4) * r_f(4)) +
              V_(0) * g_x(0, 3) + V_(1) * g_x(1, 3) + sl_x(3);
    Q_x_(4) = Fx(0, 4) * (V_x_prev(0) + V_xx_prev(0, 0) * r_f(0) + V_xx_prev(0, 1) * r_f(1) + V_xx_prev(0, 2) * r_f(2) +
                          V_xx_prev(0, 3) * r_f(3) + V_xx_prev(0, 4) * r_f(4)) +
              Fx(1, 4) * (V_x_prev(1) + V_xx_prev(1, 0) * r_f(0) + V_xx_prev(1, 1) * r_f(1) + V_xx_prev(1, 2) * r_f(2) +
                          V_xx_prev(1, 3) * r_f(3) + V_xx_prev(1, 4) * r_f(4)) +
              Fx(2, 4) * (V_x_prev(2) + V_xx_prev(2, 0) * r_f(0) + V_xx_prev(2, 1) * r_f(1) + V_xx_prev(2, 2) * r_f(2) +
                          V_xx_prev(2, 3) * r_f(3) + V_xx_prev(2, 4) * r_f(4)) +
              Fx(4, 4) * (V_x_prev(4) + V_xx_prev(4, 0) * r_f(0) + V_xx_prev(4, 1) * r_f(1) + V_xx_prev(4, 2) * r_f(2) +
                          V_xx_prev(4, 3) * r_f(3) + V_xx_prev(4, 4) * r_f(4)) +
              V_(2) * g_x(2, 4) + V_(3) * g_x(3, 4) + sl_x(4);

    // Evaluation of Vector Q_u_
    Q_u_(0) = Fu(0, 0) * (V_x_prev(0) + V_xx_prev(0, 0) * r_f(0) + V_xx_prev(0, 1) * r_f(1) + V_xx_prev(0, 2) * r_f(2) +
                          V_xx_prev(0, 3) * r_f(3) + V_xx_prev(0, 4) * r_f(4)) +
              Fu(1, 0) * (V_x_prev(1) + V_xx_prev(1, 0) * r_f(0) + V_xx_prev(1, 1) * r_f(1) + V_xx_prev(1, 2) * r_f(2) +
                          V_xx_prev(1, 3) * r_f(3) + V_xx_prev(1, 4) * r_f(4)) +
              Fu(2, 0) * (V_x_prev(2) + V_xx_prev(2, 0) * r_f(0) + V_xx_prev(2, 1) * r_f(1) + V_xx_prev(2, 2) * r_f(2) +
                          V_xx_prev(2, 3) * r_f(3) + V_xx_prev(2, 4) * r_f(4)) +
              Fu(3, 0) * (V_x_prev(3) + V_xx_prev(3, 0) * r_f(0) + V_xx_prev(3, 1) * r_f(1) + V_xx_prev(3, 2) * r_f(2) +
                          V_xx_prev(3, 3) * r_f(3) + V_xx_prev(3, 4) * r_f(4)) +
              V_(4) * g_u(4, 0) + V_(5) * g_u(5, 0) + sl_u(0);
    Q_u_(1) = Fu(0, 1) * (V_x_prev(0) + V_xx_prev(0, 0) * r_f(0) + V_xx_prev(0, 1) * r_f(1) + V_xx_prev(0, 2) * r_f(2) +
                          V_xx_prev(0, 3) * r_f(3) + V_xx_prev(0, 4) * r_f(4)) +
              Fu(1, 1) * (V_x_prev(1) + V_xx_prev(1, 0) * r_f(0) + V_xx_prev(1, 1) * r_f(1) + V_xx_prev(1, 2) * r_f(2) +
                          V_xx_prev(1, 3) * r_f(3) + V_xx_prev(1, 4) * r_f(4)) +
              Fu(2, 1) * (V_x_prev(2) + V_xx_prev(2, 0) * r_f(0) + V_xx_prev(2, 1) * r_f(1) + V_xx_prev(2, 2) * r_f(2) +
                          V_xx_prev(2, 3) * r_f(3) + V_xx_prev(2, 4) * r_f(4)) +
              Fu(4, 1) * (V_x_prev(4) + V_xx_prev(4, 0) * r_f(0) + V_xx_prev(4, 1) * r_f(1) + V_xx_prev(4, 2) * r_f(2) +
                          V_xx_prev(4, 3) * r_f(3) + V_xx_prev(4, 4) * r_f(4)) +
              V_(6) * g_u(6, 1) + V_(7) * g_u(7, 1) + sl_u(1);
  }

  virtual void updateQuuInv() override {
    // Determine internal variables
    const double Q_uu__0_0_p2 = Q_uu_(0, 0) * Q_uu_(0, 0);
    const double Q_uu__0_1_p2 = Q_uu_(0, 1) * Q_uu_(0, 1);
    const double internal_0 = Q_uu_(1, 1) - Q_uu__0_1_p2 / Q_uu_(0, 0);

    // Evaluation of Matrix Q_uu_inv_
    Q_uu_inv_(0, 0) = Q_uu__0_1_p2 / (Q_uu__0_0_p2 * internal_0) + 1.0 / Q_uu_(0, 0);
    Q_uu_inv_(0, 1) = -Q_uu_(0, 1) / (Q_uu_(0, 0) * internal_0);
    Q_uu_inv_(1, 0) = Q_uu_inv_(0, 1);
    Q_uu_inv_(1, 1) = 1.0 / internal_0;
  }

  virtual void updateKVU(Eigen::MatrixXd& k_ux, Eigen::VectorXd& v_u) override {
    // Evaluation of Matrix k_ux
    k_ux(0, 0) = -Q_uu_inv_(0, 0) * Q_xu_(0, 0) - Q_uu_inv_(0, 1) * Q_xu_(0, 1);
    k_ux(0, 1) = -Q_uu_inv_(0, 0) * Q_xu_(1, 0) - Q_uu_inv_(0, 1) * Q_xu_(1, 1);
    k_ux(0, 2) = -Q_uu_inv_(0, 0) * Q_xu_(2, 0) - Q_uu_inv_(0, 1) * Q_xu_(2, 1);
    k_ux(0, 3) = -Q_uu_inv_(0, 0) * Q_xu_(3, 0) - Q_uu_inv_(0, 1) * Q_xu_(3, 1);
    k_ux(0, 4) = -Q_uu_inv_(0, 0) * Q_xu_(4, 0) - Q_uu_inv_(0, 1) * Q_xu_(4, 1);
    k_ux(1, 0) = -Q_uu_inv_(0, 1) * Q_xu_(0, 0) - Q_uu_inv_(1, 1) * Q_xu_(0, 1);
    k_ux(1, 1) = -Q_uu_inv_(0, 1) * Q_xu_(1, 0) - Q_uu_inv_(1, 1) * Q_xu_(1, 1);
    k_ux(1, 2) = -Q_uu_inv_(0, 1) * Q_xu_(2, 0) - Q_uu_inv_(1, 1) * Q_xu_(2, 1);
    k_ux(1, 3) = -Q_uu_inv_(0, 1) * Q_xu_(3, 0) - Q_uu_inv_(1, 1) * Q_xu_(3, 1);
    k_ux(1, 4) = -Q_uu_inv_(0, 1) * Q_xu_(4, 0) - Q_uu_inv_(1, 1) * Q_xu_(4, 1);

    // Evaluation of Vector v_u
    v_u(0) = -Q_u_(0) * Q_uu_inv_(0, 0) - Q_u_(1) * Q_uu_inv_(0, 1);
    v_u(1) = -Q_u_(0) * Q_uu_inv_(0, 1) - Q_u_(1) * Q_uu_inv_(1, 1);
  }

  virtual void updateQee(const Eigen::MatrixXd& h_u) override {}

  virtual void updateQeeInv() override {}

  virtual void updateQxe(const Eigen::VectorXd& h, const Eigen::MatrixXd& h_x, const Eigen::MatrixXd& h_u,
                         const Eigen::MatrixXd& k_ux, const Eigen::VectorXd& v_u) override {}

  virtual void updateKVE(const Eigen::MatrixXd& h_u, Eigen::MatrixXd& k_ex, Eigen::MatrixXd& k_ue,
                         Eigen::VectorXd& v_e) override {}

  virtual void updateVxx(const Eigen::MatrixXd& k_ux, const Eigen::MatrixXd& k_ex, const Eigen::VectorXd& v_u,
                         const Eigen::VectorXd& v_e, Eigen::MatrixXd& V_xx, Eigen::VectorXd& V_x) override {
    // Evaluation of Matrix V_xx
    V_xx(0, 0) = Q_xu_(0, 0) * k_ux(0, 0) + Q_xu_(0, 1) * k_ux(1, 0) + Q_xx_(0, 0);
    V_xx(0, 1) = Q_xu_(0, 0) * k_ux(0, 1) + Q_xu_(0, 1) * k_ux(1, 1) + Q_xx_(0, 1);
    V_xx(0, 2) = Q_xu_(0, 0) * k_ux(0, 2) + Q_xu_(0, 1) * k_ux(1, 2) + Q_xx_(0, 2);
    V_xx(0, 3) = Q_xu_(0, 0) * k_ux(0, 3) + Q_xu_(0, 1) * k_ux(1, 3) + Q_xx_(0, 3);
    V_xx(0, 4) = Q_xu_(0, 0) * k_ux(0, 4) + Q_xu_(0, 1) * k_ux(1, 4) + Q_xx_(0, 4);
    V_xx(1, 0) = Q_xu_(1, 0) * k_ux(0, 0) + Q_xu_(1, 1) * k_ux(1, 0) + Q_xx_(0, 1);
    V_xx(1, 1) = Q_xu_(1, 0) * k_ux(0, 1) + Q_xu_(1, 1) * k_ux(1, 1) + Q_xx_(1, 1);
    V_xx(1, 2) = Q_xu_(1, 0) * k_ux(0, 2) + Q_xu_(1, 1) * k_ux(1, 2) + Q_xx_(1, 2);
    V_xx(1, 3) = Q_xu_(1, 0) * k_ux(0, 3) + Q_xu_(1, 1) * k_ux(1, 3) + Q_xx_(1, 3);
    V_xx(1, 4) = Q_xu_(1, 0) * k_ux(0, 4) + Q_xu_(1, 1) * k_ux(1, 4) + Q_xx_(1, 4);
    V_xx(2, 0) = Q_xu_(2, 0) * k_ux(0, 0) + Q_xu_(2, 1) * k_ux(1, 0) + Q_xx_(0, 2);
    V_xx(2, 1) = Q_xu_(2, 0) * k_ux(0, 1) + Q_xu_(2, 1) * k_ux(1, 1) + Q_xx_(1, 2);
    V_xx(2, 2) = Q_xu_(2, 0) * k_ux(0, 2) + Q_xu_(2, 1) * k_ux(1, 2) + Q_xx_(2, 2);
    V_xx(2, 3) = Q_xu_(2, 0) * k_ux(0, 3) + Q_xu_(2, 1) * k_ux(1, 3) + Q_xx_(2, 3);
    V_xx(2, 4) = Q_xu_(2, 0) * k_ux(0, 4) + Q_xu_(2, 1) * k_ux(1, 4) + Q_xx_(2, 4);
    V_xx(3, 0) = Q_xu_(3, 0) * k_ux(0, 0) + Q_xu_(3, 1) * k_ux(1, 0) + Q_xx_(0, 3);
    V_xx(3, 1) = Q_xu_(3, 0) * k_ux(0, 1) + Q_xu_(3, 1) * k_ux(1, 1) + Q_xx_(1, 3);
    V_xx(3, 2) = Q_xu_(3, 0) * k_ux(0, 2) + Q_xu_(3, 1) * k_ux(1, 2) + Q_xx_(2, 3);
    V_xx(3, 3) = Q_xu_(3, 0) * k_ux(0, 3) + Q_xu_(3, 1) * k_ux(1, 3) + Q_xx_(3, 3);
    V_xx(3, 4) = Q_xu_(3, 0) * k_ux(0, 4) + Q_xu_(3, 1) * k_ux(1, 4) + Q_xx_(3, 4);
    V_xx(4, 0) = Q_xu_(4, 0) * k_ux(0, 0) + Q_xu_(4, 1) * k_ux(1, 0) + Q_xx_(0, 4);
    V_xx(4, 1) = Q_xu_(4, 0) * k_ux(0, 1) + Q_xu_(4, 1) * k_ux(1, 1) + Q_xx_(1, 4);
    V_xx(4, 2) = Q_xu_(4, 0) * k_ux(0, 2) + Q_xu_(4, 1) * k_ux(1, 2) + Q_xx_(2, 4);
    V_xx(4, 3) = Q_xu_(4, 0) * k_ux(0, 3) + Q_xu_(4, 1) * k_ux(1, 3) + Q_xx_(3, 4);
    V_xx(4, 4) = Q_xu_(4, 0) * k_ux(0, 4) + Q_xu_(4, 1) * k_ux(1, 4) + Q_xx_(4, 4);

    // Evaluation of Vector V_x
    V_x(0) = Q_x_(0) + Q_xu_(0, 0) * v_u(0) + Q_xu_(0, 1) * v_u(1);
    V_x(1) = Q_x_(1) + Q_xu_(1, 0) * v_u(0) + Q_xu_(1, 1) * v_u(1);
    V_x(2) = Q_x_(2) + Q_xu_(2, 0) * v_u(0) + Q_xu_(2, 1) * v_u(1);
    V_x(3) = Q_x_(3) + Q_xu_(3, 0) * v_u(0) + Q_xu_(3, 1) * v_u(1);
    V_x(4) = Q_x_(4) + Q_xu_(4, 0) * v_u(0) + Q_xu_(4, 1) * v_u(1);
  }

  virtual void updateQnx(const Eigen::VectorXd& r_f, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& Fu,
                         const Eigen::MatrixXd& h_u, const Eigen::MatrixXd& k_ux, const Eigen::MatrixXd& k_ex,
                         const Eigen::MatrixXd& k_ue, const Eigen::VectorXd& v_u, const Eigen::VectorXd& v_e) override {
    // Evaluation of Matrix Q_un_
    Q_un_(0, 0) = -Fu(0, 0) * Q_uu_inv_(0, 0) - Fu(0, 1) * Q_uu_inv_(0, 1);
    Q_un_(0, 1) = -Fu(1, 0) * Q_uu_inv_(0, 0) - Fu(1, 1) * Q_uu_inv_(0, 1);
    Q_un_(0, 2) = -Fu(2, 0) * Q_uu_inv_(0, 0) - Fu(2, 1) * Q_uu_inv_(0, 1);
    Q_un_(0, 3) = -Fu(3, 0) * Q_uu_inv_(0, 0);
    Q_un_(0, 4) = -Fu(4, 1) * Q_uu_inv_(0, 1);
    Q_un_(1, 0) = -Fu(0, 0) * Q_uu_inv_(0, 1) - Fu(0, 1) * Q_uu_inv_(1, 1);
    Q_un_(1, 1) = -Fu(1, 0) * Q_uu_inv_(0, 1) - Fu(1, 1) * Q_uu_inv_(1, 1);
    Q_un_(1, 2) = -Fu(2, 0) * Q_uu_inv_(0, 1) - Fu(2, 1) * Q_uu_inv_(1, 1);
    Q_un_(1, 3) = -Fu(3, 0) * Q_uu_inv_(0, 1);
    Q_un_(1, 4) = -Fu(4, 1) * Q_uu_inv_(1, 1);

    // Evaluation of Matrix Q_nuu_
    Q_nuu_(0, 0) = -Q_uu_(0, 0);
    Q_nuu_(0, 1) = -Q_uu_(0, 1);
    Q_nuu_(1, 0) = -Q_uu_(0, 1);
    Q_nuu_(1, 1) = -Q_uu_(1, 1);

    // Evaluation of Matrix Q_vnx_
    Q_vnx_(0, 0) = Fu(0, 0) * k_ux(0, 0) + Fu(0, 1) * k_ux(1, 0) + Fx(0, 0);
    Q_vnx_(0, 1) = Fu(0, 0) * k_ux(0, 1) + Fu(0, 1) * k_ux(1, 1);
    Q_vnx_(0, 2) = Fu(0, 0) * k_ux(0, 2) + Fu(0, 1) * k_ux(1, 2) + Fx(0, 2);
    Q_vnx_(0, 3) = Fu(0, 0) * k_ux(0, 3) + Fu(0, 1) * k_ux(1, 3) + Fx(0, 3);
    Q_vnx_(0, 4) = Fu(0, 0) * k_ux(0, 4) + Fu(0, 1) * k_ux(1, 4) + Fx(0, 4);
    Q_vnx_(1, 0) = Fu(1, 0) * k_ux(0, 0) + Fu(1, 1) * k_ux(1, 0);
    Q_vnx_(1, 1) = Fu(1, 0) * k_ux(0, 1) + Fu(1, 1) * k_ux(1, 1) + Fx(1, 1);
    Q_vnx_(1, 2) = Fu(1, 0) * k_ux(0, 2) + Fu(1, 1) * k_ux(1, 2) + Fx(1, 2);
    Q_vnx_(1, 3) = Fu(1, 0) * k_ux(0, 3) + Fu(1, 1) * k_ux(1, 3) + Fx(1, 3);
    Q_vnx_(1, 4) = Fu(1, 0) * k_ux(0, 4) + Fu(1, 1) * k_ux(1, 4) + Fx(1, 4);
    Q_vnx_(2, 0) = Fu(2, 0) * k_ux(0, 0) + Fu(2, 1) * k_ux(1, 0);
    Q_vnx_(2, 1) = Fu(2, 0) * k_ux(0, 1) + Fu(2, 1) * k_ux(1, 1);
    Q_vnx_(2, 2) = Fu(2, 0) * k_ux(0, 2) + Fu(2, 1) * k_ux(1, 2) + Fx(2, 2);
    Q_vnx_(2, 3) = Fu(2, 0) * k_ux(0, 3) + Fu(2, 1) * k_ux(1, 3) + Fx(2, 3);
    Q_vnx_(2, 4) = Fu(2, 0) * k_ux(0, 4) + Fu(2, 1) * k_ux(1, 4) + Fx(2, 4);
    Q_vnx_(3, 0) = Fu(3, 0) * k_ux(0, 0);
    Q_vnx_(3, 1) = Fu(3, 0) * k_ux(0, 1);
    Q_vnx_(3, 2) = Fu(3, 0) * k_ux(0, 2);
    Q_vnx_(3, 3) = Fu(3, 0) * k_ux(0, 3) + Fx(3, 3);
    Q_vnx_(3, 4) = Fu(3, 0) * k_ux(0, 4);
    Q_vnx_(4, 0) = Fu(4, 1) * k_ux(1, 0);
    Q_vnx_(4, 1) = Fu(4, 1) * k_ux(1, 1);
    Q_vnx_(4, 2) = Fu(4, 1) * k_ux(1, 2);
    Q_vnx_(4, 3) = Fu(4, 1) * k_ux(1, 3);
    Q_vnx_(4, 4) = Fu(4, 1) * k_ux(1, 4) + Fx(4, 4);

    // Evaluation of Vector Q_vn_
    Q_vn_(0) = Fu(0, 0) * v_u(0) + Fu(0, 1) * v_u(1) + r_f(0);
    Q_vn_(1) = Fu(1, 0) * v_u(0) + Fu(1, 1) * v_u(1) + r_f(1);
    Q_vn_(2) = Fu(2, 0) * v_u(0) + Fu(2, 1) * v_u(1) + r_f(2);
    Q_vn_(3) = Fu(3, 0) * v_u(0) + r_f(3);
    Q_vn_(4) = Fu(4, 1) * v_u(1) + r_f(4);
  }
};

template <>
class BicycleTrajectoryTrackerIpmEvaluator<StageType::INITIAL, OcpConfig::ERK4> : public IpmEvaluator {
 public:
  BicycleTrajectoryTrackerIpmEvaluator() : IpmEvaluator(5, 2, 4, 0, 5) {}
  virtual ~BicycleTrajectoryTrackerIpmEvaluator() = default;

  virtual void updateLX(const Eigen::VectorXd& l_x, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& g_x,
                        const Eigen::MatrixXd& h_x, const Eigen::MatrixXd& m_x, const Eigen::VectorXd& p,
                        const Eigen::VectorXd& lambda, const Eigen::VectorXd& eta, const Eigen::VectorXd& nu,
                        Eigen::VectorXd& sl_x) const override {
    // Evaluation of Vector sl_x
    sl_x(0) = Fx(0, 0) * p(0) + l_x(0) + m_x(0, 0) * nu(0);
    sl_x(1) = Fx(1, 1) * p(1) + l_x(1) + m_x(1, 1) * nu(1);
    sl_x(2) = Fx(0, 2) * p(0) + Fx(1, 2) * p(1) + Fx(2, 2) * p(2) + l_x(2) + m_x(2, 2) * nu(2);
    sl_x(3) = Fx(0, 3) * p(0) + Fx(1, 3) * p(1) + Fx(2, 3) * p(2) + Fx(3, 3) * p(3) + l_x(3) + m_x(3, 3) * nu(3);
    sl_x(4) = Fx(0, 4) * p(0) + Fx(1, 4) * p(1) + Fx(2, 4) * p(2) + Fx(4, 4) * p(4) + l_x(4) + m_x(4, 4) * nu(4);
  }

  virtual void updateLU(const Eigen::VectorXd& l_u, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& g_u,
                        const Eigen::MatrixXd& h_u, const Eigen::VectorXd& p, const Eigen::VectorXd& lambda,
                        const Eigen::VectorXd& eta, Eigen::VectorXd& sl_u) const override {
    // Evaluation of Vector sl_u
    sl_u(0) = Fu(0, 0) * p(0) + Fu(1, 0) * p(1) + Fu(2, 0) * p(2) + Fu(3, 0) * p(3) + g_u(0, 0) * lambda(0) +
              g_u(1, 0) * lambda(1) + l_u(0);
    sl_u(1) = Fu(0, 1) * p(0) + Fu(1, 1) * p(1) + Fu(2, 1) * p(2) + Fu(4, 1) * p(4) + g_u(2, 1) * lambda(2) +
              g_u(3, 1) * lambda(3) + l_u(1);
  }

  virtual void updateQH(const Eigen::MatrixXd& l_xx, const Eigen::MatrixXd& l_xu, const Eigen::MatrixXd& l_uu,
                        const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::MatrixXd& Fx,
                        const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& V_xx_prev) override {
    // Determine internal variables
    const double Fx_0_0_p2 = Fx(0, 0) * Fx(0, 0);
    const double Fx_1_1_p2 = Fx(1, 1) * Fx(1, 1);
    const double g_u_0_0_p2 = g_u(0, 0) * g_u(0, 0);
    const double g_u_1_0_p2 = g_u(1, 0) * g_u(1, 0);
    const double g_u_2_1_p2 = g_u(2, 1) * g_u(2, 1);
    const double g_u_3_1_p2 = g_u(3, 1) * g_u(3, 1);

    // Evaluation of Matrix Q_xx_
    Q_xx_(0, 0) = Fx_0_0_p2 * V_xx_prev(0, 0) + l_xx(0, 0);
    Q_xx_(0, 1) = Fx(0, 0) * Fx(1, 1) * V_xx_prev(0, 1);
    Q_xx_(0, 2) = Fx(0, 0) * Fx(0, 2) * V_xx_prev(0, 0) + Fx(0, 0) * Fx(1, 2) * V_xx_prev(0, 1) +
                  Fx(0, 0) * Fx(2, 2) * V_xx_prev(0, 2);
    Q_xx_(0, 3) = Fx(0, 0) * Fx(0, 3) * V_xx_prev(0, 0) + Fx(0, 0) * Fx(1, 3) * V_xx_prev(0, 1) +
                  Fx(0, 0) * Fx(2, 3) * V_xx_prev(0, 2) + Fx(0, 0) * Fx(3, 3) * V_xx_prev(0, 3);
    Q_xx_(0, 4) = Fx(0, 0) * Fx(0, 4) * V_xx_prev(0, 0) + Fx(0, 0) * Fx(1, 4) * V_xx_prev(0, 1) +
                  Fx(0, 0) * Fx(2, 4) * V_xx_prev(0, 2) + Fx(0, 0) * Fx(4, 4) * V_xx_prev(0, 4);
    Q_xx_(1, 0) = Fx(0, 0) * Fx(1, 1) * V_xx_prev(1, 0);
    Q_xx_(1, 1) = Fx_1_1_p2 * V_xx_prev(1, 1) + l_xx(1, 1);
    Q_xx_(1, 2) = Fx(0, 2) * Fx(1, 1) * V_xx_prev(1, 0) + Fx(1, 1) * Fx(1, 2) * V_xx_prev(1, 1) +
                  Fx(1, 1) * Fx(2, 2) * V_xx_prev(1, 2);
    Q_xx_(1, 3) = Fx(0, 3) * Fx(1, 1) * V_xx_prev(1, 0) + Fx(1, 1) * Fx(1, 3) * V_xx_prev(1, 1) +
                  Fx(1, 1) * Fx(2, 3) * V_xx_prev(1, 2) + Fx(1, 1) * Fx(3, 3) * V_xx_prev(1, 3);
    Q_xx_(1, 4) = Fx(0, 4) * Fx(1, 1) * V_xx_prev(1, 0) + Fx(1, 1) * Fx(1, 4) * V_xx_prev(1, 1) +
                  Fx(1, 1) * Fx(2, 4) * V_xx_prev(1, 2) + Fx(1, 1) * Fx(4, 4) * V_xx_prev(1, 4);
    Q_xx_(2, 0) = Fx(0, 0) * (Fx(0, 2) * V_xx_prev(0, 0) + Fx(1, 2) * V_xx_prev(1, 0) + Fx(2, 2) * V_xx_prev(2, 0));
    Q_xx_(2, 1) = Fx(1, 1) * (Fx(0, 2) * V_xx_prev(0, 1) + Fx(1, 2) * V_xx_prev(1, 1) + Fx(2, 2) * V_xx_prev(2, 1));
    Q_xx_(2, 2) = Fx(0, 2) * (Fx(0, 2) * V_xx_prev(0, 0) + Fx(1, 2) * V_xx_prev(1, 0) + Fx(2, 2) * V_xx_prev(2, 0)) +
                  Fx(1, 2) * (Fx(0, 2) * V_xx_prev(0, 1) + Fx(1, 2) * V_xx_prev(1, 1) + Fx(2, 2) * V_xx_prev(2, 1)) +
                  Fx(2, 2) * (Fx(0, 2) * V_xx_prev(0, 2) + Fx(1, 2) * V_xx_prev(1, 2) + Fx(2, 2) * V_xx_prev(2, 2)) +
                  l_xx(2, 2);
    Q_xx_(2, 3) = Fx(0, 3) * (Fx(0, 2) * V_xx_prev(0, 0) + Fx(1, 2) * V_xx_prev(1, 0) + Fx(2, 2) * V_xx_prev(2, 0)) +
                  Fx(1, 3) * (Fx(0, 2) * V_xx_prev(0, 1) + Fx(1, 2) * V_xx_prev(1, 1) + Fx(2, 2) * V_xx_prev(2, 1)) +
                  Fx(2, 3) * (Fx(0, 2) * V_xx_prev(0, 2) + Fx(1, 2) * V_xx_prev(1, 2) + Fx(2, 2) * V_xx_prev(2, 2)) +
                  Fx(3, 3) * (Fx(0, 2) * V_xx_prev(0, 3) + Fx(1, 2) * V_xx_prev(1, 3) + Fx(2, 2) * V_xx_prev(2, 3));
    Q_xx_(2, 4) = Fx(0, 4) * (Fx(0, 2) * V_xx_prev(0, 0) + Fx(1, 2) * V_xx_prev(1, 0) + Fx(2, 2) * V_xx_prev(2, 0)) +
                  Fx(1, 4) * (Fx(0, 2) * V_xx_prev(0, 1) + Fx(1, 2) * V_xx_prev(1, 1) + Fx(2, 2) * V_xx_prev(2, 1)) +
                  Fx(2, 4) * (Fx(0, 2) * V_xx_prev(0, 2) + Fx(1, 2) * V_xx_prev(1, 2) + Fx(2, 2) * V_xx_prev(2, 2)) +
                  Fx(4, 4) * (Fx(0, 2) * V_xx_prev(0, 4) + Fx(1, 2) * V_xx_prev(1, 4) + Fx(2, 2) * V_xx_prev(2, 4));
    Q_xx_(3, 0) = Fx(0, 0) * (Fx(0, 3) * V_xx_prev(0, 0) + Fx(1, 3) * V_xx_prev(1, 0) + Fx(2, 3) * V_xx_prev(2, 0) +
                              Fx(3, 3) * V_xx_prev(3, 0));
    Q_xx_(3, 1) = Fx(1, 1) * (Fx(0, 3) * V_xx_prev(0, 1) + Fx(1, 3) * V_xx_prev(1, 1) + Fx(2, 3) * V_xx_prev(2, 1) +
                              Fx(3, 3) * V_xx_prev(3, 1));
    Q_xx_(3, 2) = Fx(0, 2) * (Fx(0, 3) * V_xx_prev(0, 0) + Fx(1, 3) * V_xx_prev(1, 0) + Fx(2, 3) * V_xx_prev(2, 0) +
                              Fx(3, 3) * V_xx_prev(3, 0)) +
                  Fx(1, 2) * (Fx(0, 3) * V_xx_prev(0, 1) + Fx(1, 3) * V_xx_prev(1, 1) + Fx(2, 3) * V_xx_prev(2, 1) +
                              Fx(3, 3) * V_xx_prev(3, 1)) +
                  Fx(2, 2) * (Fx(0, 3) * V_xx_prev(0, 2) + Fx(1, 3) * V_xx_prev(1, 2) + Fx(2, 3) * V_xx_prev(2, 2) +
                              Fx(3, 3) * V_xx_prev(3, 2));
    Q_xx_(3, 3) = Fx(0, 3) * (Fx(0, 3) * V_xx_prev(0, 0) + Fx(1, 3) * V_xx_prev(1, 0) + Fx(2, 3) * V_xx_prev(2, 0) +
                              Fx(3, 3) * V_xx_prev(3, 0)) +
                  Fx(1, 3) * (Fx(0, 3) * V_xx_prev(0, 1) + Fx(1, 3) * V_xx_prev(1, 1) + Fx(2, 3) * V_xx_prev(2, 1) +
                              Fx(3, 3) * V_xx_prev(3, 1)) +
                  Fx(2, 3) * (Fx(0, 3) * V_xx_prev(0, 2) + Fx(1, 3) * V_xx_prev(1, 2) + Fx(2, 3) * V_xx_prev(2, 2) +
                              Fx(3, 3) * V_xx_prev(3, 2)) +
                  Fx(3, 3) * (Fx(0, 3) * V_xx_prev(0, 3) + Fx(1, 3) * V_xx_prev(1, 3) + Fx(2, 3) * V_xx_prev(2, 3) +
                              Fx(3, 3) * V_xx_prev(3, 3)) +
                  l_xx(3, 3);
    Q_xx_(3, 4) = Fx(0, 4) * (Fx(0, 3) * V_xx_prev(0, 0) + Fx(1, 3) * V_xx_prev(1, 0) + Fx(2, 3) * V_xx_prev(2, 0) +
                              Fx(3, 3) * V_xx_prev(3, 0)) +
                  Fx(1, 4) * (Fx(0, 3) * V_xx_prev(0, 1) + Fx(1, 3) * V_xx_prev(1, 1) + Fx(2, 3) * V_xx_prev(2, 1) +
                              Fx(3, 3) * V_xx_prev(3, 1)) +
                  Fx(2, 4) * (Fx(0, 3) * V_xx_prev(0, 2) + Fx(1, 3) * V_xx_prev(1, 2) + Fx(2, 3) * V_xx_prev(2, 2) +
                              Fx(3, 3) * V_xx_prev(3, 2)) +
                  Fx(4, 4) * (Fx(0, 3) * V_xx_prev(0, 4) + Fx(1, 3) * V_xx_prev(1, 4) + Fx(2, 3) * V_xx_prev(2, 4) +
                              Fx(3, 3) * V_xx_prev(3, 4));
    Q_xx_(4, 0) = Fx(0, 0) * (Fx(0, 4) * V_xx_prev(0, 0) + Fx(1, 4) * V_xx_prev(1, 0) + Fx(2, 4) * V_xx_prev(2, 0) +
                              Fx(4, 4) * V_xx_prev(4, 0));
    Q_xx_(4, 1) = Fx(1, 1) * (Fx(0, 4) * V_xx_prev(0, 1) + Fx(1, 4) * V_xx_prev(1, 1) + Fx(2, 4) * V_xx_prev(2, 1) +
                              Fx(4, 4) * V_xx_prev(4, 1));
    Q_xx_(4, 2) = Fx(0, 2) * (Fx(0, 4) * V_xx_prev(0, 0) + Fx(1, 4) * V_xx_prev(1, 0) + Fx(2, 4) * V_xx_prev(2, 0) +
                              Fx(4, 4) * V_xx_prev(4, 0)) +
                  Fx(1, 2) * (Fx(0, 4) * V_xx_prev(0, 1) + Fx(1, 4) * V_xx_prev(1, 1) + Fx(2, 4) * V_xx_prev(2, 1) +
                              Fx(4, 4) * V_xx_prev(4, 1)) +
                  Fx(2, 2) * (Fx(0, 4) * V_xx_prev(0, 2) + Fx(1, 4) * V_xx_prev(1, 2) + Fx(2, 4) * V_xx_prev(2, 2) +
                              Fx(4, 4) * V_xx_prev(4, 2));
    Q_xx_(4, 3) = Fx(0, 3) * (Fx(0, 4) * V_xx_prev(0, 0) + Fx(1, 4) * V_xx_prev(1, 0) + Fx(2, 4) * V_xx_prev(2, 0) +
                              Fx(4, 4) * V_xx_prev(4, 0)) +
                  Fx(1, 3) * (Fx(0, 4) * V_xx_prev(0, 1) + Fx(1, 4) * V_xx_prev(1, 1) + Fx(2, 4) * V_xx_prev(2, 1) +
                              Fx(4, 4) * V_xx_prev(4, 1)) +
                  Fx(2, 3) * (Fx(0, 4) * V_xx_prev(0, 2) + Fx(1, 4) * V_xx_prev(1, 2) + Fx(2, 4) * V_xx_prev(2, 2) +
                              Fx(4, 4) * V_xx_prev(4, 2)) +
                  Fx(3, 3) * (Fx(0, 4) * V_xx_prev(0, 3) + Fx(1, 4) * V_xx_prev(1, 3) + Fx(2, 4) * V_xx_prev(2, 3) +
                              Fx(4, 4) * V_xx_prev(4, 3));
    Q_xx_(4, 4) = Fx(0, 4) * (Fx(0, 4) * V_xx_prev(0, 0) + Fx(1, 4) * V_xx_prev(1, 0) + Fx(2, 4) * V_xx_prev(2, 0) +
                              Fx(4, 4) * V_xx_prev(4, 0)) +
                  Fx(1, 4) * (Fx(0, 4) * V_xx_prev(0, 1) + Fx(1, 4) * V_xx_prev(1, 1) + Fx(2, 4) * V_xx_prev(2, 1) +
                              Fx(4, 4) * V_xx_prev(4, 1)) +
                  Fx(2, 4) * (Fx(0, 4) * V_xx_prev(0, 2) + Fx(1, 4) * V_xx_prev(1, 2) + Fx(2, 4) * V_xx_prev(2, 2) +
                              Fx(4, 4) * V_xx_prev(4, 2)) +
                  Fx(4, 4) * (Fx(0, 4) * V_xx_prev(0, 4) + Fx(1, 4) * V_xx_prev(1, 4) + Fx(2, 4) * V_xx_prev(2, 4) +
                              Fx(4, 4) * V_xx_prev(4, 4)) +
                  l_xx(4, 4);

    // Evaluation of Matrix Q_xu_
    Q_xu_(0, 0) = Fu(0, 0) * Fx(0, 0) * V_xx_prev(0, 0) + Fu(1, 0) * Fx(0, 0) * V_xx_prev(0, 1) +
                  Fu(2, 0) * Fx(0, 0) * V_xx_prev(0, 2) + Fu(3, 0) * Fx(0, 0) * V_xx_prev(0, 3);
    Q_xu_(0, 1) = Fu(0, 1) * Fx(0, 0) * V_xx_prev(0, 0) + Fu(1, 1) * Fx(0, 0) * V_xx_prev(0, 1) +
                  Fu(2, 1) * Fx(0, 0) * V_xx_prev(0, 2) + Fu(4, 1) * Fx(0, 0) * V_xx_prev(0, 4);
    Q_xu_(1, 0) = Fu(0, 0) * Fx(1, 1) * V_xx_prev(1, 0) + Fu(1, 0) * Fx(1, 1) * V_xx_prev(1, 1) +
                  Fu(2, 0) * Fx(1, 1) * V_xx_prev(1, 2) + Fu(3, 0) * Fx(1, 1) * V_xx_prev(1, 3);
    Q_xu_(1, 1) = Fu(0, 1) * Fx(1, 1) * V_xx_prev(1, 0) + Fu(1, 1) * Fx(1, 1) * V_xx_prev(1, 1) +
                  Fu(2, 1) * Fx(1, 1) * V_xx_prev(1, 2) + Fu(4, 1) * Fx(1, 1) * V_xx_prev(1, 4);
    Q_xu_(2, 0) = Fu(0, 0) * (Fx(0, 2) * V_xx_prev(0, 0) + Fx(1, 2) * V_xx_prev(1, 0) + Fx(2, 2) * V_xx_prev(2, 0)) +
                  Fu(1, 0) * (Fx(0, 2) * V_xx_prev(0, 1) + Fx(1, 2) * V_xx_prev(1, 1) + Fx(2, 2) * V_xx_prev(2, 1)) +
                  Fu(2, 0) * (Fx(0, 2) * V_xx_prev(0, 2) + Fx(1, 2) * V_xx_prev(1, 2) + Fx(2, 2) * V_xx_prev(2, 2)) +
                  Fu(3, 0) * (Fx(0, 2) * V_xx_prev(0, 3) + Fx(1, 2) * V_xx_prev(1, 3) + Fx(2, 2) * V_xx_prev(2, 3));
    Q_xu_(2, 1) = Fu(0, 1) * (Fx(0, 2) * V_xx_prev(0, 0) + Fx(1, 2) * V_xx_prev(1, 0) + Fx(2, 2) * V_xx_prev(2, 0)) +
                  Fu(1, 1) * (Fx(0, 2) * V_xx_prev(0, 1) + Fx(1, 2) * V_xx_prev(1, 1) + Fx(2, 2) * V_xx_prev(2, 1)) +
                  Fu(2, 1) * (Fx(0, 2) * V_xx_prev(0, 2) + Fx(1, 2) * V_xx_prev(1, 2) + Fx(2, 2) * V_xx_prev(2, 2)) +
                  Fu(4, 1) * (Fx(0, 2) * V_xx_prev(0, 4) + Fx(1, 2) * V_xx_prev(1, 4) + Fx(2, 2) * V_xx_prev(2, 4));
    Q_xu_(3, 0) = Fu(0, 0) * (Fx(0, 3) * V_xx_prev(0, 0) + Fx(1, 3) * V_xx_prev(1, 0) + Fx(2, 3) * V_xx_prev(2, 0) +
                              Fx(3, 3) * V_xx_prev(3, 0)) +
                  Fu(1, 0) * (Fx(0, 3) * V_xx_prev(0, 1) + Fx(1, 3) * V_xx_prev(1, 1) + Fx(2, 3) * V_xx_prev(2, 1) +
                              Fx(3, 3) * V_xx_prev(3, 1)) +
                  Fu(2, 0) * (Fx(0, 3) * V_xx_prev(0, 2) + Fx(1, 3) * V_xx_prev(1, 2) + Fx(2, 3) * V_xx_prev(2, 2) +
                              Fx(3, 3) * V_xx_prev(3, 2)) +
                  Fu(3, 0) * (Fx(0, 3) * V_xx_prev(0, 3) + Fx(1, 3) * V_xx_prev(1, 3) + Fx(2, 3) * V_xx_prev(2, 3) +
                              Fx(3, 3) * V_xx_prev(3, 3));
    Q_xu_(3, 1) = Fu(0, 1) * (Fx(0, 3) * V_xx_prev(0, 0) + Fx(1, 3) * V_xx_prev(1, 0) + Fx(2, 3) * V_xx_prev(2, 0) +
                              Fx(3, 3) * V_xx_prev(3, 0)) +
                  Fu(1, 1) * (Fx(0, 3) * V_xx_prev(0, 1) + Fx(1, 3) * V_xx_prev(1, 1) + Fx(2, 3) * V_xx_prev(2, 1) +
                              Fx(3, 3) * V_xx_prev(3, 1)) +
                  Fu(2, 1) * (Fx(0, 3) * V_xx_prev(0, 2) + Fx(1, 3) * V_xx_prev(1, 2) + Fx(2, 3) * V_xx_prev(2, 2) +
                              Fx(3, 3) * V_xx_prev(3, 2)) +
                  Fu(4, 1) * (Fx(0, 3) * V_xx_prev(0, 4) + Fx(1, 3) * V_xx_prev(1, 4) + Fx(2, 3) * V_xx_prev(2, 4) +
                              Fx(3, 3) * V_xx_prev(3, 4));
    Q_xu_(4, 0) = Fu(0, 0) * (Fx(0, 4) * V_xx_prev(0, 0) + Fx(1, 4) * V_xx_prev(1, 0) + Fx(2, 4) * V_xx_prev(2, 0) +
                              Fx(4, 4) * V_xx_prev(4, 0)) +
                  Fu(1, 0) * (Fx(0, 4) * V_xx_prev(0, 1) + Fx(1, 4) * V_xx_prev(1, 1) + Fx(2, 4) * V_xx_prev(2, 1) +
                              Fx(4, 4) * V_xx_prev(4, 1)) +
                  Fu(2, 0) * (Fx(0, 4) * V_xx_prev(0, 2) + Fx(1, 4) * V_xx_prev(1, 2) + Fx(2, 4) * V_xx_prev(2, 2) +
                              Fx(4, 4) * V_xx_prev(4, 2)) +
                  Fu(3, 0) * (Fx(0, 4) * V_xx_prev(0, 3) + Fx(1, 4) * V_xx_prev(1, 3) + Fx(2, 4) * V_xx_prev(2, 3) +
                              Fx(4, 4) * V_xx_prev(4, 3));
    Q_xu_(4, 1) = Fu(0, 1) * (Fx(0, 4) * V_xx_prev(0, 0) + Fx(1, 4) * V_xx_prev(1, 0) + Fx(2, 4) * V_xx_prev(2, 0) +
                              Fx(4, 4) * V_xx_prev(4, 0)) +
                  Fu(1, 1) * (Fx(0, 4) * V_xx_prev(0, 1) + Fx(1, 4) * V_xx_prev(1, 1) + Fx(2, 4) * V_xx_prev(2, 1) +
                              Fx(4, 4) * V_xx_prev(4, 1)) +
                  Fu(2, 1) * (Fx(0, 4) * V_xx_prev(0, 2) + Fx(1, 4) * V_xx_prev(1, 2) + Fx(2, 4) * V_xx_prev(2, 2) +
                              Fx(4, 4) * V_xx_prev(4, 2)) +
                  Fu(4, 1) * (Fx(0, 4) * V_xx_prev(0, 4) + Fx(1, 4) * V_xx_prev(1, 4) + Fx(2, 4) * V_xx_prev(2, 4) +
                              Fx(4, 4) * V_xx_prev(4, 4));

    // Evaluation of Matrix Q_uu_
    Q_uu_(0, 0) = Fu(0, 0) * (Fu(0, 0) * V_xx_prev(0, 0) + Fu(1, 0) * V_xx_prev(1, 0) + Fu(2, 0) * V_xx_prev(2, 0) +
                              Fu(3, 0) * V_xx_prev(3, 0)) +
                  Fu(1, 0) * (Fu(0, 0) * V_xx_prev(0, 1) + Fu(1, 0) * V_xx_prev(1, 1) + Fu(2, 0) * V_xx_prev(2, 1) +
                              Fu(3, 0) * V_xx_prev(3, 1)) +
                  Fu(2, 0) * (Fu(0, 0) * V_xx_prev(0, 2) + Fu(1, 0) * V_xx_prev(1, 2) + Fu(2, 0) * V_xx_prev(2, 2) +
                              Fu(3, 0) * V_xx_prev(3, 2)) +
                  Fu(3, 0) * (Fu(0, 0) * V_xx_prev(0, 3) + Fu(1, 0) * V_xx_prev(1, 3) + Fu(2, 0) * V_xx_prev(2, 3) +
                              Fu(3, 0) * V_xx_prev(3, 3)) +
                  Sigma_(0) * g_u_0_0_p2 + Sigma_(1) * g_u_1_0_p2 + l_uu(0, 0);
    Q_uu_(0, 1) = Fu(0, 1) * (Fu(0, 0) * V_xx_prev(0, 0) + Fu(1, 0) * V_xx_prev(1, 0) + Fu(2, 0) * V_xx_prev(2, 0) +
                              Fu(3, 0) * V_xx_prev(3, 0)) +
                  Fu(1, 1) * (Fu(0, 0) * V_xx_prev(0, 1) + Fu(1, 0) * V_xx_prev(1, 1) + Fu(2, 0) * V_xx_prev(2, 1) +
                              Fu(3, 0) * V_xx_prev(3, 1)) +
                  Fu(2, 1) * (Fu(0, 0) * V_xx_prev(0, 2) + Fu(1, 0) * V_xx_prev(1, 2) + Fu(2, 0) * V_xx_prev(2, 2) +
                              Fu(3, 0) * V_xx_prev(3, 2)) +
                  Fu(4, 1) * (Fu(0, 0) * V_xx_prev(0, 4) + Fu(1, 0) * V_xx_prev(1, 4) + Fu(2, 0) * V_xx_prev(2, 4) +
                              Fu(3, 0) * V_xx_prev(3, 4));
    Q_uu_(1, 0) = Fu(0, 0) * (Fu(0, 1) * V_xx_prev(0, 0) + Fu(1, 1) * V_xx_prev(1, 0) + Fu(2, 1) * V_xx_prev(2, 0) +
                              Fu(4, 1) * V_xx_prev(4, 0)) +
                  Fu(1, 0) * (Fu(0, 1) * V_xx_prev(0, 1) + Fu(1, 1) * V_xx_prev(1, 1) + Fu(2, 1) * V_xx_prev(2, 1) +
                              Fu(4, 1) * V_xx_prev(4, 1)) +
                  Fu(2, 0) * (Fu(0, 1) * V_xx_prev(0, 2) + Fu(1, 1) * V_xx_prev(1, 2) + Fu(2, 1) * V_xx_prev(2, 2) +
                              Fu(4, 1) * V_xx_prev(4, 2)) +
                  Fu(3, 0) * (Fu(0, 1) * V_xx_prev(0, 3) + Fu(1, 1) * V_xx_prev(1, 3) + Fu(2, 1) * V_xx_prev(2, 3) +
                              Fu(4, 1) * V_xx_prev(4, 3));
    Q_uu_(1, 1) = Fu(0, 1) * (Fu(0, 1) * V_xx_prev(0, 0) + Fu(1, 1) * V_xx_prev(1, 0) + Fu(2, 1) * V_xx_prev(2, 0) +
                              Fu(4, 1) * V_xx_prev(4, 0)) +
                  Fu(1, 1) * (Fu(0, 1) * V_xx_prev(0, 1) + Fu(1, 1) * V_xx_prev(1, 1) + Fu(2, 1) * V_xx_prev(2, 1) +
                              Fu(4, 1) * V_xx_prev(4, 1)) +
                  Fu(2, 1) * (Fu(0, 1) * V_xx_prev(0, 2) + Fu(1, 1) * V_xx_prev(1, 2) + Fu(2, 1) * V_xx_prev(2, 2) +
                              Fu(4, 1) * V_xx_prev(4, 2)) +
                  Fu(4, 1) * (Fu(0, 1) * V_xx_prev(0, 4) + Fu(1, 1) * V_xx_prev(1, 4) + Fu(2, 1) * V_xx_prev(2, 4) +
                              Fu(4, 1) * V_xx_prev(4, 4)) +
                  Sigma_(2) * g_u_2_1_p2 + Sigma_(3) * g_u_3_1_p2 + l_uu(1, 1);
  }

  virtual void updateQG(const Eigen::VectorXd& sl_x, const Eigen::VectorXd& sl_u, const Eigen::MatrixXd& g_x,
                        const Eigen::MatrixXd& g_u, const Eigen::VectorXd& r_f, const Eigen::MatrixXd& Fx,
                        const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& V_xx_prev,
                        const Eigen::VectorXd& V_x_prev) override {
    // Evaluation of Vector Q_x_
    Q_x_(0) = Fx(0, 0) * (V_x_prev(0) + V_xx_prev(0, 0) * r_f(0) + V_xx_prev(0, 1) * r_f(1) + V_xx_prev(0, 2) * r_f(2) +
                          V_xx_prev(0, 3) * r_f(3) + V_xx_prev(0, 4) * r_f(4)) +
              sl_x(0);
    Q_x_(1) = Fx(1, 1) * (V_x_prev(1) + V_xx_prev(1, 0) * r_f(0) + V_xx_prev(1, 1) * r_f(1) + V_xx_prev(1, 2) * r_f(2) +
                          V_xx_prev(1, 3) * r_f(3) + V_xx_prev(1, 4) * r_f(4)) +
              sl_x(1);
    Q_x_(2) = Fx(0, 2) * (V_x_prev(0) + V_xx_prev(0, 0) * r_f(0) + V_xx_prev(0, 1) * r_f(1) + V_xx_prev(0, 2) * r_f(2) +
                          V_xx_prev(0, 3) * r_f(3) + V_xx_prev(0, 4) * r_f(4)) +
              Fx(1, 2) * (V_x_prev(1) + V_xx_prev(1, 0) * r_f(0) + V_xx_prev(1, 1) * r_f(1) + V_xx_prev(1, 2) * r_f(2) +
                          V_xx_prev(1, 3) * r_f(3) + V_xx_prev(1, 4) * r_f(4)) +
              Fx(2, 2) * (V_x_prev(2) + V_xx_prev(2, 0) * r_f(0) + V_xx_prev(2, 1) * r_f(1) + V_xx_prev(2, 2) * r_f(2) +
                          V_xx_prev(2, 3) * r_f(3) + V_xx_prev(2, 4) * r_f(4)) +
              sl_x(2);
    Q_x_(3) = Fx(0, 3) * (V_x_prev(0) + V_xx_prev(0, 0) * r_f(0) + V_xx_prev(0, 1) * r_f(1) + V_xx_prev(0, 2) * r_f(2) +
                          V_xx_prev(0, 3) * r_f(3) + V_xx_prev(0, 4) * r_f(4)) +
              Fx(1, 3) * (V_x_prev(1) + V_xx_prev(1, 0) * r_f(0) + V_xx_prev(1, 1) * r_f(1) + V_xx_prev(1, 2) * r_f(2) +
                          V_xx_prev(1, 3) * r_f(3) + V_xx_prev(1, 4) * r_f(4)) +
              Fx(2, 3) * (V_x_prev(2) + V_xx_prev(2, 0) * r_f(0) + V_xx_prev(2, 1) * r_f(1) + V_xx_prev(2, 2) * r_f(2) +
                          V_xx_prev(2, 3) * r_f(3) + V_xx_prev(2, 4) * r_f(4)) +
              Fx(3, 3) * (V_x_prev(3) + V_xx_prev(3, 0) * r_f(0) + V_xx_prev(3, 1) * r_f(1) + V_xx_prev(3, 2) * r_f(2) +
                          V_xx_prev(3, 3) * r_f(3) + V_xx_prev(3, 4) * r_f(4)) +
              sl_x(3);
    Q_x_(4) = Fx(0, 4) * (V_x_prev(0) + V_xx_prev(0, 0) * r_f(0) + V_xx_prev(0, 1) * r_f(1) + V_xx_prev(0, 2) * r_f(2) +
                          V_xx_prev(0, 3) * r_f(3) + V_xx_prev(0, 4) * r_f(4)) +
              Fx(1, 4) * (V_x_prev(1) + V_xx_prev(1, 0) * r_f(0) + V_xx_prev(1, 1) * r_f(1) + V_xx_prev(1, 2) * r_f(2) +
                          V_xx_prev(1, 3) * r_f(3) + V_xx_prev(1, 4) * r_f(4)) +
              Fx(2, 4) * (V_x_prev(2) + V_xx_prev(2, 0) * r_f(0) + V_xx_prev(2, 1) * r_f(1) + V_xx_prev(2, 2) * r_f(2) +
                          V_xx_prev(2, 3) * r_f(3) + V_xx_prev(2, 4) * r_f(4)) +
              Fx(4, 4) * (V_x_prev(4) + V_xx_prev(4, 0) * r_f(0) + V_xx_prev(4, 1) * r_f(1) + V_xx_prev(4, 2) * r_f(2) +
                          V_xx_prev(4, 3) * r_f(3) + V_xx_prev(4, 4) * r_f(4)) +
              sl_x(4);

    // Evaluation of Vector Q_u_
    Q_u_(0) = Fu(0, 0) * (V_x_prev(0) + V_xx_prev(0, 0) * r_f(0) + V_xx_prev(0, 1) * r_f(1) + V_xx_prev(0, 2) * r_f(2) +
                          V_xx_prev(0, 3) * r_f(3) + V_xx_prev(0, 4) * r_f(4)) +
              Fu(1, 0) * (V_x_prev(1) + V_xx_prev(1, 0) * r_f(0) + V_xx_prev(1, 1) * r_f(1) + V_xx_prev(1, 2) * r_f(2) +
                          V_xx_prev(1, 3) * r_f(3) + V_xx_prev(1, 4) * r_f(4)) +
              Fu(2, 0) * (V_x_prev(2) + V_xx_prev(2, 0) * r_f(0) + V_xx_prev(2, 1) * r_f(1) + V_xx_prev(2, 2) * r_f(2) +
                          V_xx_prev(2, 3) * r_f(3) + V_xx_prev(2, 4) * r_f(4)) +
              Fu(3, 0) * (V_x_prev(3) + V_xx_prev(3, 0) * r_f(0) + V_xx_prev(3, 1) * r_f(1) + V_xx_prev(3, 2) * r_f(2) +
                          V_xx_prev(3, 3) * r_f(3) + V_xx_prev(3, 4) * r_f(4)) +
              V_(0) * g_u(0, 0) + V_(1) * g_u(1, 0) + sl_u(0);
    Q_u_(1) = Fu(0, 1) * (V_x_prev(0) + V_xx_prev(0, 0) * r_f(0) + V_xx_prev(0, 1) * r_f(1) + V_xx_prev(0, 2) * r_f(2) +
                          V_xx_prev(0, 3) * r_f(3) + V_xx_prev(0, 4) * r_f(4)) +
              Fu(1, 1) * (V_x_prev(1) + V_xx_prev(1, 0) * r_f(0) + V_xx_prev(1, 1) * r_f(1) + V_xx_prev(1, 2) * r_f(2) +
                          V_xx_prev(1, 3) * r_f(3) + V_xx_prev(1, 4) * r_f(4)) +
              Fu(2, 1) * (V_x_prev(2) + V_xx_prev(2, 0) * r_f(0) + V_xx_prev(2, 1) * r_f(1) + V_xx_prev(2, 2) * r_f(2) +
                          V_xx_prev(2, 3) * r_f(3) + V_xx_prev(2, 4) * r_f(4)) +
              Fu(4, 1) * (V_x_prev(4) + V_xx_prev(4, 0) * r_f(0) + V_xx_prev(4, 1) * r_f(1) + V_xx_prev(4, 2) * r_f(2) +
                          V_xx_prev(4, 3) * r_f(3) + V_xx_prev(4, 4) * r_f(4)) +
              V_(2) * g_u(2, 1) + V_(3) * g_u(3, 1) + sl_u(1);
  }

  virtual void updateQuuInv() override {
    // Determine internal variables
    const double Q_uu__0_0_p2 = Q_uu_(0, 0) * Q_uu_(0, 0);
    const double Q_uu__0_1_p2 = Q_uu_(0, 1) * Q_uu_(0, 1);
    const double internal_0 = Q_uu_(1, 1) - Q_uu__0_1_p2 / Q_uu_(0, 0);

    // Evaluation of Matrix Q_uu_inv_
    Q_uu_inv_(0, 0) = Q_uu__0_1_p2 / (Q_uu__0_0_p2 * internal_0) + 1.0 / Q_uu_(0, 0);
    Q_uu_inv_(0, 1) = -Q_uu_(0, 1) / (Q_uu_(0, 0) * internal_0);
    Q_uu_inv_(1, 0) = Q_uu_inv_(0, 1);
    Q_uu_inv_(1, 1) = 1.0 / internal_0;
  }

  virtual void updateKVU(Eigen::MatrixXd& k_ux, Eigen::VectorXd& v_u) override {
    // Evaluation of Matrix k_ux
    k_ux(0, 0) = -Q_uu_inv_(0, 0) * Q_xu_(0, 0) - Q_uu_inv_(0, 1) * Q_xu_(0, 1);
    k_ux(0, 1) = -Q_uu_inv_(0, 0) * Q_xu_(1, 0) - Q_uu_inv_(0, 1) * Q_xu_(1, 1);
    k_ux(0, 2) = -Q_uu_inv_(0, 0) * Q_xu_(2, 0) - Q_uu_inv_(0, 1) * Q_xu_(2, 1);
    k_ux(0, 3) = -Q_uu_inv_(0, 0) * Q_xu_(3, 0) - Q_uu_inv_(0, 1) * Q_xu_(3, 1);
    k_ux(0, 4) = -Q_uu_inv_(0, 0) * Q_xu_(4, 0) - Q_uu_inv_(0, 1) * Q_xu_(4, 1);
    k_ux(1, 0) = -Q_uu_inv_(0, 1) * Q_xu_(0, 0) - Q_uu_inv_(1, 1) * Q_xu_(0, 1);
    k_ux(1, 1) = -Q_uu_inv_(0, 1) * Q_xu_(1, 0) - Q_uu_inv_(1, 1) * Q_xu_(1, 1);
    k_ux(1, 2) = -Q_uu_inv_(0, 1) * Q_xu_(2, 0) - Q_uu_inv_(1, 1) * Q_xu_(2, 1);
    k_ux(1, 3) = -Q_uu_inv_(0, 1) * Q_xu_(3, 0) - Q_uu_inv_(1, 1) * Q_xu_(3, 1);
    k_ux(1, 4) = -Q_uu_inv_(0, 1) * Q_xu_(4, 0) - Q_uu_inv_(1, 1) * Q_xu_(4, 1);

    // Evaluation of Vector v_u
    v_u(0) = -Q_u_(0) * Q_uu_inv_(0, 0) - Q_u_(1) * Q_uu_inv_(0, 1);
    v_u(1) = -Q_u_(0) * Q_uu_inv_(0, 1) - Q_u_(1) * Q_uu_inv_(1, 1);
  }

  virtual void updateQee(const Eigen::MatrixXd& h_u) override {}

  virtual void updateQeeInv() override {}

  virtual void updateQxe(const Eigen::VectorXd& h, const Eigen::MatrixXd& h_x, const Eigen::MatrixXd& h_u,
                         const Eigen::MatrixXd& k_ux, const Eigen::VectorXd& v_u) override {}

  virtual void updateKVE(const Eigen::MatrixXd& h_u, Eigen::MatrixXd& k_ex, Eigen::MatrixXd& k_ue,
                         Eigen::VectorXd& v_e) override {}

  virtual void updateVxx(const Eigen::MatrixXd& k_ux, const Eigen::MatrixXd& k_ex, const Eigen::VectorXd& v_u,
                         const Eigen::VectorXd& v_e, Eigen::MatrixXd& V_xx, Eigen::VectorXd& V_x) override {
    // Evaluation of Matrix V_xx
    V_xx(0, 0) = Q_xu_(0, 0) * k_ux(0, 0) + Q_xu_(0, 1) * k_ux(1, 0) + Q_xx_(0, 0);
    V_xx(0, 1) = Q_xu_(0, 0) * k_ux(0, 1) + Q_xu_(0, 1) * k_ux(1, 1) + Q_xx_(0, 1);
    V_xx(0, 2) = Q_xu_(0, 0) * k_ux(0, 2) + Q_xu_(0, 1) * k_ux(1, 2) + Q_xx_(0, 2);
    V_xx(0, 3) = Q_xu_(0, 0) * k_ux(0, 3) + Q_xu_(0, 1) * k_ux(1, 3) + Q_xx_(0, 3);
    V_xx(0, 4) = Q_xu_(0, 0) * k_ux(0, 4) + Q_xu_(0, 1) * k_ux(1, 4) + Q_xx_(0, 4);
    V_xx(1, 0) = Q_xu_(1, 0) * k_ux(0, 0) + Q_xu_(1, 1) * k_ux(1, 0) + Q_xx_(0, 1);
    V_xx(1, 1) = Q_xu_(1, 0) * k_ux(0, 1) + Q_xu_(1, 1) * k_ux(1, 1) + Q_xx_(1, 1);
    V_xx(1, 2) = Q_xu_(1, 0) * k_ux(0, 2) + Q_xu_(1, 1) * k_ux(1, 2) + Q_xx_(1, 2);
    V_xx(1, 3) = Q_xu_(1, 0) * k_ux(0, 3) + Q_xu_(1, 1) * k_ux(1, 3) + Q_xx_(1, 3);
    V_xx(1, 4) = Q_xu_(1, 0) * k_ux(0, 4) + Q_xu_(1, 1) * k_ux(1, 4) + Q_xx_(1, 4);
    V_xx(2, 0) = Q_xu_(2, 0) * k_ux(0, 0) + Q_xu_(2, 1) * k_ux(1, 0) + Q_xx_(0, 2);
    V_xx(2, 1) = Q_xu_(2, 0) * k_ux(0, 1) + Q_xu_(2, 1) * k_ux(1, 1) + Q_xx_(1, 2);
    V_xx(2, 2) = Q_xu_(2, 0) * k_ux(0, 2) + Q_xu_(2, 1) * k_ux(1, 2) + Q_xx_(2, 2);
    V_xx(2, 3) = Q_xu_(2, 0) * k_ux(0, 3) + Q_xu_(2, 1) * k_ux(1, 3) + Q_xx_(2, 3);
    V_xx(2, 4) = Q_xu_(2, 0) * k_ux(0, 4) + Q_xu_(2, 1) * k_ux(1, 4) + Q_xx_(2, 4);
    V_xx(3, 0) = Q_xu_(3, 0) * k_ux(0, 0) + Q_xu_(3, 1) * k_ux(1, 0) + Q_xx_(0, 3);
    V_xx(3, 1) = Q_xu_(3, 0) * k_ux(0, 1) + Q_xu_(3, 1) * k_ux(1, 1) + Q_xx_(1, 3);
    V_xx(3, 2) = Q_xu_(3, 0) * k_ux(0, 2) + Q_xu_(3, 1) * k_ux(1, 2) + Q_xx_(2, 3);
    V_xx(3, 3) = Q_xu_(3, 0) * k_ux(0, 3) + Q_xu_(3, 1) * k_ux(1, 3) + Q_xx_(3, 3);
    V_xx(3, 4) = Q_xu_(3, 0) * k_ux(0, 4) + Q_xu_(3, 1) * k_ux(1, 4) + Q_xx_(3, 4);
    V_xx(4, 0) = Q_xu_(4, 0) * k_ux(0, 0) + Q_xu_(4, 1) * k_ux(1, 0) + Q_xx_(0, 4);
    V_xx(4, 1) = Q_xu_(4, 0) * k_ux(0, 1) + Q_xu_(4, 1) * k_ux(1, 1) + Q_xx_(1, 4);
    V_xx(4, 2) = Q_xu_(4, 0) * k_ux(0, 2) + Q_xu_(4, 1) * k_ux(1, 2) + Q_xx_(2, 4);
    V_xx(4, 3) = Q_xu_(4, 0) * k_ux(0, 3) + Q_xu_(4, 1) * k_ux(1, 3) + Q_xx_(3, 4);
    V_xx(4, 4) = Q_xu_(4, 0) * k_ux(0, 4) + Q_xu_(4, 1) * k_ux(1, 4) + Q_xx_(4, 4);

    // Evaluation of Vector V_x
    V_x(0) = Q_x_(0) + Q_xu_(0, 0) * v_u(0) + Q_xu_(0, 1) * v_u(1);
    V_x(1) = Q_x_(1) + Q_xu_(1, 0) * v_u(0) + Q_xu_(1, 1) * v_u(1);
    V_x(2) = Q_x_(2) + Q_xu_(2, 0) * v_u(0) + Q_xu_(2, 1) * v_u(1);
    V_x(3) = Q_x_(3) + Q_xu_(3, 0) * v_u(0) + Q_xu_(3, 1) * v_u(1);
    V_x(4) = Q_x_(4) + Q_xu_(4, 0) * v_u(0) + Q_xu_(4, 1) * v_u(1);
  }

  virtual void updateQnx(const Eigen::VectorXd& r_f, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& Fu,
                         const Eigen::MatrixXd& h_u, const Eigen::MatrixXd& k_ux, const Eigen::MatrixXd& k_ex,
                         const Eigen::MatrixXd& k_ue, const Eigen::VectorXd& v_u, const Eigen::VectorXd& v_e) override {
    // Evaluation of Matrix Q_un_
    Q_un_(0, 0) = -Fu(0, 0) * Q_uu_inv_(0, 0) - Fu(0, 1) * Q_uu_inv_(0, 1);
    Q_un_(0, 1) = -Fu(1, 0) * Q_uu_inv_(0, 0) - Fu(1, 1) * Q_uu_inv_(0, 1);
    Q_un_(0, 2) = -Fu(2, 0) * Q_uu_inv_(0, 0) - Fu(2, 1) * Q_uu_inv_(0, 1);
    Q_un_(0, 3) = -Fu(3, 0) * Q_uu_inv_(0, 0);
    Q_un_(0, 4) = -Fu(4, 1) * Q_uu_inv_(0, 1);
    Q_un_(1, 0) = -Fu(0, 0) * Q_uu_inv_(0, 1) - Fu(0, 1) * Q_uu_inv_(1, 1);
    Q_un_(1, 1) = -Fu(1, 0) * Q_uu_inv_(0, 1) - Fu(1, 1) * Q_uu_inv_(1, 1);
    Q_un_(1, 2) = -Fu(2, 0) * Q_uu_inv_(0, 1) - Fu(2, 1) * Q_uu_inv_(1, 1);
    Q_un_(1, 3) = -Fu(3, 0) * Q_uu_inv_(0, 1);
    Q_un_(1, 4) = -Fu(4, 1) * Q_uu_inv_(1, 1);

    // Evaluation of Matrix Q_nuu_
    Q_nuu_(0, 0) = -Q_uu_(0, 0);
    Q_nuu_(0, 1) = -Q_uu_(0, 1);
    Q_nuu_(1, 0) = -Q_uu_(0, 1);
    Q_nuu_(1, 1) = -Q_uu_(1, 1);

    // Evaluation of Matrix Q_vnx_
    Q_vnx_(0, 0) = Fu(0, 0) * k_ux(0, 0) + Fu(0, 1) * k_ux(1, 0) + Fx(0, 0);
    Q_vnx_(0, 1) = Fu(0, 0) * k_ux(0, 1) + Fu(0, 1) * k_ux(1, 1);
    Q_vnx_(0, 2) = Fu(0, 0) * k_ux(0, 2) + Fu(0, 1) * k_ux(1, 2) + Fx(0, 2);
    Q_vnx_(0, 3) = Fu(0, 0) * k_ux(0, 3) + Fu(0, 1) * k_ux(1, 3) + Fx(0, 3);
    Q_vnx_(0, 4) = Fu(0, 0) * k_ux(0, 4) + Fu(0, 1) * k_ux(1, 4) + Fx(0, 4);
    Q_vnx_(1, 0) = Fu(1, 0) * k_ux(0, 0) + Fu(1, 1) * k_ux(1, 0);
    Q_vnx_(1, 1) = Fu(1, 0) * k_ux(0, 1) + Fu(1, 1) * k_ux(1, 1) + Fx(1, 1);
    Q_vnx_(1, 2) = Fu(1, 0) * k_ux(0, 2) + Fu(1, 1) * k_ux(1, 2) + Fx(1, 2);
    Q_vnx_(1, 3) = Fu(1, 0) * k_ux(0, 3) + Fu(1, 1) * k_ux(1, 3) + Fx(1, 3);
    Q_vnx_(1, 4) = Fu(1, 0) * k_ux(0, 4) + Fu(1, 1) * k_ux(1, 4) + Fx(1, 4);
    Q_vnx_(2, 0) = Fu(2, 0) * k_ux(0, 0) + Fu(2, 1) * k_ux(1, 0);
    Q_vnx_(2, 1) = Fu(2, 0) * k_ux(0, 1) + Fu(2, 1) * k_ux(1, 1);
    Q_vnx_(2, 2) = Fu(2, 0) * k_ux(0, 2) + Fu(2, 1) * k_ux(1, 2) + Fx(2, 2);
    Q_vnx_(2, 3) = Fu(2, 0) * k_ux(0, 3) + Fu(2, 1) * k_ux(1, 3) + Fx(2, 3);
    Q_vnx_(2, 4) = Fu(2, 0) * k_ux(0, 4) + Fu(2, 1) * k_ux(1, 4) + Fx(2, 4);
    Q_vnx_(3, 0) = Fu(3, 0) * k_ux(0, 0);
    Q_vnx_(3, 1) = Fu(3, 0) * k_ux(0, 1);
    Q_vnx_(3, 2) = Fu(3, 0) * k_ux(0, 2);
    Q_vnx_(3, 3) = Fu(3, 0) * k_ux(0, 3) + Fx(3, 3);
    Q_vnx_(3, 4) = Fu(3, 0) * k_ux(0, 4);
    Q_vnx_(4, 0) = Fu(4, 1) * k_ux(1, 0);
    Q_vnx_(4, 1) = Fu(4, 1) * k_ux(1, 1);
    Q_vnx_(4, 2) = Fu(4, 1) * k_ux(1, 2);
    Q_vnx_(4, 3) = Fu(4, 1) * k_ux(1, 3);
    Q_vnx_(4, 4) = Fu(4, 1) * k_ux(1, 4) + Fx(4, 4);

    // Evaluation of Vector Q_vn_
    Q_vn_(0) = Fu(0, 0) * v_u(0) + Fu(0, 1) * v_u(1) + r_f(0);
    Q_vn_(1) = Fu(1, 0) * v_u(0) + Fu(1, 1) * v_u(1) + r_f(1);
    Q_vn_(2) = Fu(2, 0) * v_u(0) + Fu(2, 1) * v_u(1) + r_f(2);
    Q_vn_(3) = Fu(3, 0) * v_u(0) + r_f(3);
    Q_vn_(4) = Fu(4, 1) * v_u(1) + r_f(4);
  }
};

class BicycleTrajectoryTrackerIpmEvaluatorTerminal : public IpmEvaluatorTerminal {
 public:
  BicycleTrajectoryTrackerIpmEvaluatorTerminal() : IpmEvaluatorTerminal(5, 2, 4, 0) {}
  virtual ~BicycleTrajectoryTrackerIpmEvaluatorTerminal() = default;

  virtual void updateLX(const Eigen::VectorXd& l_x, const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& m_x,
                        const Eigen::VectorXd& p, const Eigen::VectorXd& lambda, const Eigen::VectorXd& nu,
                        Eigen::VectorXd& sl_x) const override {
    // Evaluation of Vector sl_x
    sl_x(0) = l_x(0) - p(0);
    sl_x(1) = l_x(1) - p(1);
    sl_x(2) = l_x(2) - p(2);
    sl_x(3) = g_x(0, 3) * lambda(0) + g_x(1, 3) * lambda(1) + l_x(3) - p(3);
    sl_x(4) = g_x(2, 4) * lambda(2) + g_x(3, 4) * lambda(3) + l_x(4) - p(4);
  }

  virtual void updateVxx(const Eigen::MatrixXd& l_xx, const Eigen::VectorXd& sl_x, const Eigen::MatrixXd& g_x,
                         Eigen::MatrixXd& V_xx, Eigen::VectorXd& V_x) override {
    // Determine internal variables
    const double g_x_0_3_p2 = g_x(0, 3) * g_x(0, 3);
    const double g_x_1_3_p2 = g_x(1, 3) * g_x(1, 3);
    const double g_x_2_4_p2 = g_x(2, 4) * g_x(2, 4);
    const double g_x_3_4_p2 = g_x(3, 4) * g_x(3, 4);

    // Evaluation of SymmetricMatrix V_xx
    V_xx(0, 0) = l_xx(0, 0);
    V_xx(1, 1) = l_xx(1, 1);
    V_xx(2, 2) = l_xx(2, 2);
    V_xx(3, 3) = Sigma_(0) * g_x_0_3_p2 + Sigma_(1) * g_x_1_3_p2 + l_xx(3, 3);
    V_xx(4, 4) = Sigma_(2) * g_x_2_4_p2 + Sigma_(3) * g_x_3_4_p2 + l_xx(4, 4);

    // Evaluation of Vector V_x
    V_x(0) = sl_x(0);
    V_x(1) = sl_x(1);
    V_x(2) = sl_x(2);
    V_x(3) = V_(0) * g_x(0, 3) + V_(1) * g_x(1, 3) + sl_x(3);
    V_x(4) = V_(2) * g_x(2, 4) + V_(3) * g_x(3, 4) + sl_x(4);
  }
};

}  // namespace gpal::pnc::planning