#pragma once

#include "ocp/ipm_evaluator.h"

namespace gpal::pnc::planning {

template <StageType stage_type, OcpConfig::IntegratorType integrator_type>
class ParkLateralGeneralIpmEvaluator : public IpmEvaluator {
 public:
  ParkLateralGeneralIpmEvaluator() : IpmEvaluator(5, 2, 17, 0, 0) {}
  virtual ~ParkLateralGeneralIpmEvaluator() = default;

  virtual void updateLX(const Eigen::VectorXd& l_x, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& g_x,
                        const Eigen::MatrixXd& h_x, const Eigen::MatrixXd& m_x, const Eigen::VectorXd& p,
                        const Eigen::VectorXd& lambda, const Eigen::VectorXd& eta, const Eigen::VectorXd& nu,
                        Eigen::VectorXd& sl_x) const override {
    // Evaluation of Vector sl_x
    sl_x(0) = Fx(0, 0) * p(0);
    sl_x(1) = Fx(0, 1) * p(0) + Fx(1, 1) * p(1) + g_x(0, 1) * lambda(0) + g_x(1, 1) * lambda(1) +
              g_x(11, 1) * lambda(11) + g_x(12, 1) * lambda(12) + g_x(13, 1) * lambda(13) + g_x(14, 1) * lambda(14) +
              g_x(15, 1) * lambda(15) + g_x(16, 1) * lambda(16) + g_x(2, 1) * lambda(2) + g_x(3, 1) * lambda(3) +
              g_x(4, 1) * lambda(4) + g_x(5, 1) * lambda(5) + l_x(1);
    sl_x(2) = Fx(0, 2) * p(0) + Fx(2, 2) * p(2) + g_x(0, 2) * lambda(0) + g_x(1, 2) * lambda(1) +
              g_x(11, 2) * lambda(11) + g_x(12, 2) * lambda(12) + g_x(13, 2) * lambda(13) + g_x(14, 2) * lambda(14) +
              g_x(15, 2) * lambda(15) + g_x(16, 2) * lambda(16) + g_x(2, 2) * lambda(2) + g_x(3, 2) * lambda(3) +
              g_x(4, 2) * lambda(4) + g_x(5, 2) * lambda(5) + l_x(2);
    sl_x(3) = Fx(0, 3) * p(0) + Fx(1, 3) * p(1) + Fx(2, 3) * p(2) + Fx(3, 3) * p(3) + g_x(13, 3) * lambda(13) +
              g_x(14, 3) * lambda(14) + g_x(15, 3) * lambda(15) + g_x(16, 3) * lambda(16) + g_x(2, 3) * lambda(2) +
              g_x(3, 3) * lambda(3) + g_x(4, 3) * lambda(4) + g_x(5, 3) * lambda(5) + g_x(8, 3) * lambda(8);
    sl_x(4) = Fx(3, 4) * p(3) + Fx(4, 4) * p(4) + g_x(6, 4) * lambda(6) + g_x(7, 4) * lambda(7) + l_x(4);
  }

  virtual void updateLU(const Eigen::VectorXd& l_u, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& g_u,
                        const Eigen::MatrixXd& h_u, const Eigen::VectorXd& p, const Eigen::VectorXd& lambda,
                        const Eigen::VectorXd& eta, Eigen::VectorXd& sl_u) const override {
    // Evaluation of Vector sl_u
    sl_u(0) = Fu(4, 0) * p(4) + g_u(10, 0) * lambda(10) + g_u(9, 0) * lambda(9) + l_u(0);
    sl_u(1) = g_u(11, 1) * lambda(11) + g_u(12, 1) * lambda(12) + g_u(13, 1) * lambda(13) + g_u(14, 1) * lambda(14) +
              g_u(15, 1) * lambda(15) + g_u(16, 1) * lambda(16) + l_u(1);
  }

  virtual void updateQH(const Eigen::MatrixXd& l_xx, const Eigen::MatrixXd& l_xu, const Eigen::MatrixXd& l_uu,
                        const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::MatrixXd& Fx,
                        const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& V_xx_prev) override {
    // Determine internal variables
    const double Fu_4_0_p2 = Fu(4, 0) * Fu(4, 0);
    const double Fx_0_0_p2 = Fx(0, 0) * Fx(0, 0);
    const double g_u_10_0_p2 = g_u(10, 0) * g_u(10, 0);
    const double g_u_11_1_p2 = g_u(11, 1) * g_u(11, 1);
    const double g_u_12_1_p2 = g_u(12, 1) * g_u(12, 1);
    const double g_u_13_1_p2 = g_u(13, 1) * g_u(13, 1);
    const double g_u_14_1_p2 = g_u(14, 1) * g_u(14, 1);
    const double g_u_15_1_p2 = g_u(15, 1) * g_u(15, 1);
    const double g_u_16_1_p2 = g_u(16, 1) * g_u(16, 1);
    const double g_u_9_0_p2 = g_u(9, 0) * g_u(9, 0);
    const double g_x_0_1_p2 = g_x(0, 1) * g_x(0, 1);
    const double g_x_0_2_p2 = g_x(0, 2) * g_x(0, 2);
    const double g_x_11_1_p2 = g_x(11, 1) * g_x(11, 1);
    const double g_x_11_2_p2 = g_x(11, 2) * g_x(11, 2);
    const double g_x_12_1_p2 = g_x(12, 1) * g_x(12, 1);
    const double g_x_12_2_p2 = g_x(12, 2) * g_x(12, 2);
    const double g_x_13_1_p2 = g_x(13, 1) * g_x(13, 1);
    const double g_x_13_2_p2 = g_x(13, 2) * g_x(13, 2);
    const double g_x_13_3_p2 = g_x(13, 3) * g_x(13, 3);
    const double g_x_14_1_p2 = g_x(14, 1) * g_x(14, 1);
    const double g_x_14_2_p2 = g_x(14, 2) * g_x(14, 2);
    const double g_x_14_3_p2 = g_x(14, 3) * g_x(14, 3);
    const double g_x_15_1_p2 = g_x(15, 1) * g_x(15, 1);
    const double g_x_15_2_p2 = g_x(15, 2) * g_x(15, 2);
    const double g_x_15_3_p2 = g_x(15, 3) * g_x(15, 3);
    const double g_x_16_1_p2 = g_x(16, 1) * g_x(16, 1);
    const double g_x_16_2_p2 = g_x(16, 2) * g_x(16, 2);
    const double g_x_16_3_p2 = g_x(16, 3) * g_x(16, 3);
    const double g_x_1_1_p2 = g_x(1, 1) * g_x(1, 1);
    const double g_x_1_2_p2 = g_x(1, 2) * g_x(1, 2);
    const double g_x_2_1_p2 = g_x(2, 1) * g_x(2, 1);
    const double g_x_2_2_p2 = g_x(2, 2) * g_x(2, 2);
    const double g_x_2_3_p2 = g_x(2, 3) * g_x(2, 3);
    const double g_x_3_1_p2 = g_x(3, 1) * g_x(3, 1);
    const double g_x_3_2_p2 = g_x(3, 2) * g_x(3, 2);
    const double g_x_3_3_p2 = g_x(3, 3) * g_x(3, 3);
    const double g_x_4_1_p2 = g_x(4, 1) * g_x(4, 1);
    const double g_x_4_2_p2 = g_x(4, 2) * g_x(4, 2);
    const double g_x_4_3_p2 = g_x(4, 3) * g_x(4, 3);
    const double g_x_5_1_p2 = g_x(5, 1) * g_x(5, 1);
    const double g_x_5_2_p2 = g_x(5, 2) * g_x(5, 2);
    const double g_x_5_3_p2 = g_x(5, 3) * g_x(5, 3);
    const double g_x_6_4_p2 = g_x(6, 4) * g_x(6, 4);
    const double g_x_7_4_p2 = g_x(7, 4) * g_x(7, 4);
    const double g_x_8_3_p2 = g_x(8, 3) * g_x(8, 3);
    const double internal_0 = Sigma_(0) * g_x(0, 1) * g_x(0, 2);
    const double internal_1 = Sigma_(1) * g_x(1, 1) * g_x(1, 2);
    const double internal_10 = Sigma_(4) * g_x(4, 1) * g_x(4, 2);
    const double internal_11 = Sigma_(5) * g_x(5, 1) * g_x(5, 2);
    const double internal_12 = Sigma_(13) * g_x(13, 1) * g_x(13, 3);
    const double internal_13 = Sigma_(14) * g_x(14, 1) * g_x(14, 3);
    const double internal_14 = Sigma_(15) * g_x(15, 1) * g_x(15, 3);
    const double internal_15 = Sigma_(16) * g_x(16, 1) * g_x(16, 3);
    const double internal_16 = Sigma_(2) * g_x(2, 1) * g_x(2, 3);
    const double internal_17 = Sigma_(3) * g_x(3, 1) * g_x(3, 3);
    const double internal_18 = Sigma_(4) * g_x(4, 1) * g_x(4, 3);
    const double internal_19 = Sigma_(5) * g_x(5, 1) * g_x(5, 3);
    const double internal_2 = Sigma_(11) * g_x(11, 1) * g_x(11, 2);
    const double internal_20 = Sigma_(13) * g_x(13, 2) * g_x(13, 3);
    const double internal_21 = Sigma_(14) * g_x(14, 2) * g_x(14, 3);
    const double internal_22 = Sigma_(15) * g_x(15, 2) * g_x(15, 3);
    const double internal_23 = Sigma_(16) * g_x(16, 2) * g_x(16, 3);
    const double internal_24 = Sigma_(2) * g_x(2, 2) * g_x(2, 3);
    const double internal_25 = Sigma_(3) * g_x(3, 2) * g_x(3, 3);
    const double internal_26 = Sigma_(4) * g_x(4, 2) * g_x(4, 3);
    const double internal_27 = Sigma_(5) * g_x(5, 2) * g_x(5, 3);
    const double internal_3 = Sigma_(12) * g_x(12, 1) * g_x(12, 2);
    const double internal_4 = Sigma_(13) * g_x(13, 1) * g_x(13, 2);
    const double internal_5 = Sigma_(14) * g_x(14, 1) * g_x(14, 2);
    const double internal_6 = Sigma_(15) * g_x(15, 1) * g_x(15, 2);
    const double internal_7 = Sigma_(16) * g_x(16, 1) * g_x(16, 2);
    const double internal_8 = Sigma_(2) * g_x(2, 1) * g_x(2, 2);
    const double internal_9 = Sigma_(3) * g_x(3, 1) * g_x(3, 2);

    // Evaluation of Matrix Q_xx_
    Q_xx_(0, 0) = Fx_0_0_p2 * V_xx_prev(0, 0);
    Q_xx_(0, 1) = Fx(0, 0) * Fx(0, 1) * V_xx_prev(0, 0) + Fx(0, 0) * Fx(1, 1) * V_xx_prev(0, 1);
    Q_xx_(0, 2) = Fx(0, 0) * Fx(0, 2) * V_xx_prev(0, 0) + Fx(0, 0) * Fx(2, 2) * V_xx_prev(0, 2);
    Q_xx_(0, 3) = Fx(0, 0) * Fx(0, 3) * V_xx_prev(0, 0) + Fx(0, 0) * Fx(1, 3) * V_xx_prev(0, 1) +
                  Fx(0, 0) * Fx(2, 3) * V_xx_prev(0, 2) + Fx(0, 0) * Fx(3, 3) * V_xx_prev(0, 3);
    Q_xx_(0, 4) = Fx(0, 0) * Fx(3, 4) * V_xx_prev(0, 3) + Fx(0, 0) * Fx(4, 4) * V_xx_prev(0, 4);
    Q_xx_(1, 0) = Fx(0, 0) * (Fx(0, 1) * V_xx_prev(0, 0) + Fx(1, 1) * V_xx_prev(1, 0));
    Q_xx_(1, 1) = Fx(0, 1) * (Fx(0, 1) * V_xx_prev(0, 0) + Fx(1, 1) * V_xx_prev(1, 0)) +
                  Fx(1, 1) * (Fx(0, 1) * V_xx_prev(0, 1) + Fx(1, 1) * V_xx_prev(1, 1)) + Sigma_(0) * g_x_0_1_p2 +
                  Sigma_(1) * g_x_1_1_p2 + Sigma_(11) * g_x_11_1_p2 + Sigma_(12) * g_x_12_1_p2 +
                  Sigma_(13) * g_x_13_1_p2 + Sigma_(14) * g_x_14_1_p2 + Sigma_(15) * g_x_15_1_p2 +
                  Sigma_(16) * g_x_16_1_p2 + Sigma_(2) * g_x_2_1_p2 + Sigma_(3) * g_x_3_1_p2 + Sigma_(4) * g_x_4_1_p2 +
                  Sigma_(5) * g_x_5_1_p2 + l_xx(1, 1);
    Q_xx_(1, 2) = Fx(0, 2) * (Fx(0, 1) * V_xx_prev(0, 0) + Fx(1, 1) * V_xx_prev(1, 0)) +
                  Fx(2, 2) * (Fx(0, 1) * V_xx_prev(0, 2) + Fx(1, 1) * V_xx_prev(1, 2)) + internal_0 + internal_1 +
                  internal_10 + internal_11 + internal_2 + internal_3 + internal_4 + internal_5 + internal_6 +
                  internal_7 + internal_8 + internal_9 + l_xx(1, 2);
    Q_xx_(1, 3) = Fx(0, 3) * (Fx(0, 1) * V_xx_prev(0, 0) + Fx(1, 1) * V_xx_prev(1, 0)) +
                  Fx(1, 3) * (Fx(0, 1) * V_xx_prev(0, 1) + Fx(1, 1) * V_xx_prev(1, 1)) +
                  Fx(2, 3) * (Fx(0, 1) * V_xx_prev(0, 2) + Fx(1, 1) * V_xx_prev(1, 2)) +
                  Fx(3, 3) * (Fx(0, 1) * V_xx_prev(0, 3) + Fx(1, 1) * V_xx_prev(1, 3)) + internal_12 + internal_13 +
                  internal_14 + internal_15 + internal_16 + internal_17 + internal_18 + internal_19;
    Q_xx_(1, 4) = Fx(3, 4) * (Fx(0, 1) * V_xx_prev(0, 3) + Fx(1, 1) * V_xx_prev(1, 3)) +
                  Fx(4, 4) * (Fx(0, 1) * V_xx_prev(0, 4) + Fx(1, 1) * V_xx_prev(1, 4));
    Q_xx_(2, 0) = Fx(0, 0) * (Fx(0, 2) * V_xx_prev(0, 0) + Fx(2, 2) * V_xx_prev(2, 0));
    Q_xx_(2, 1) = Fx(0, 1) * (Fx(0, 2) * V_xx_prev(0, 0) + Fx(2, 2) * V_xx_prev(2, 0)) +
                  Fx(1, 1) * (Fx(0, 2) * V_xx_prev(0, 1) + Fx(2, 2) * V_xx_prev(2, 1)) + internal_0 + internal_1 +
                  internal_10 + internal_11 + internal_2 + internal_3 + internal_4 + internal_5 + internal_6 +
                  internal_7 + internal_8 + internal_9 + l_xx(2, 1);
    Q_xx_(2, 2) = Fx(0, 2) * (Fx(0, 2) * V_xx_prev(0, 0) + Fx(2, 2) * V_xx_prev(2, 0)) +
                  Fx(2, 2) * (Fx(0, 2) * V_xx_prev(0, 2) + Fx(2, 2) * V_xx_prev(2, 2)) + Sigma_(0) * g_x_0_2_p2 +
                  Sigma_(1) * g_x_1_2_p2 + Sigma_(11) * g_x_11_2_p2 + Sigma_(12) * g_x_12_2_p2 +
                  Sigma_(13) * g_x_13_2_p2 + Sigma_(14) * g_x_14_2_p2 + Sigma_(15) * g_x_15_2_p2 +
                  Sigma_(16) * g_x_16_2_p2 + Sigma_(2) * g_x_2_2_p2 + Sigma_(3) * g_x_3_2_p2 + Sigma_(4) * g_x_4_2_p2 +
                  Sigma_(5) * g_x_5_2_p2 + l_xx(2, 2);
    Q_xx_(2, 3) = Fx(0, 3) * (Fx(0, 2) * V_xx_prev(0, 0) + Fx(2, 2) * V_xx_prev(2, 0)) +
                  Fx(1, 3) * (Fx(0, 2) * V_xx_prev(0, 1) + Fx(2, 2) * V_xx_prev(2, 1)) +
                  Fx(2, 3) * (Fx(0, 2) * V_xx_prev(0, 2) + Fx(2, 2) * V_xx_prev(2, 2)) +
                  Fx(3, 3) * (Fx(0, 2) * V_xx_prev(0, 3) + Fx(2, 2) * V_xx_prev(2, 3)) + internal_20 + internal_21 +
                  internal_22 + internal_23 + internal_24 + internal_25 + internal_26 + internal_27;
    Q_xx_(2, 4) = Fx(3, 4) * (Fx(0, 2) * V_xx_prev(0, 3) + Fx(2, 2) * V_xx_prev(2, 3)) +
                  Fx(4, 4) * (Fx(0, 2) * V_xx_prev(0, 4) + Fx(2, 2) * V_xx_prev(2, 4));
    Q_xx_(3, 0) = Fx(0, 0) * (Fx(0, 3) * V_xx_prev(0, 0) + Fx(1, 3) * V_xx_prev(1, 0) + Fx(2, 3) * V_xx_prev(2, 0) +
                              Fx(3, 3) * V_xx_prev(3, 0));
    Q_xx_(3, 1) = Fx(0, 1) * (Fx(0, 3) * V_xx_prev(0, 0) + Fx(1, 3) * V_xx_prev(1, 0) + Fx(2, 3) * V_xx_prev(2, 0) +
                              Fx(3, 3) * V_xx_prev(3, 0)) +
                  Fx(1, 1) * (Fx(0, 3) * V_xx_prev(0, 1) + Fx(1, 3) * V_xx_prev(1, 1) + Fx(2, 3) * V_xx_prev(2, 1) +
                              Fx(3, 3) * V_xx_prev(3, 1)) +
                  internal_12 + internal_13 + internal_14 + internal_15 + internal_16 + internal_17 + internal_18 +
                  internal_19;
    Q_xx_(3, 2) = Fx(0, 2) * (Fx(0, 3) * V_xx_prev(0, 0) + Fx(1, 3) * V_xx_prev(1, 0) + Fx(2, 3) * V_xx_prev(2, 0) +
                              Fx(3, 3) * V_xx_prev(3, 0)) +
                  Fx(2, 2) * (Fx(0, 3) * V_xx_prev(0, 2) + Fx(1, 3) * V_xx_prev(1, 2) + Fx(2, 3) * V_xx_prev(2, 2) +
                              Fx(3, 3) * V_xx_prev(3, 2)) +
                  internal_20 + internal_21 + internal_22 + internal_23 + internal_24 + internal_25 + internal_26 +
                  internal_27;
    Q_xx_(3, 3) = Fx(0, 3) * (Fx(0, 3) * V_xx_prev(0, 0) + Fx(1, 3) * V_xx_prev(1, 0) + Fx(2, 3) * V_xx_prev(2, 0) +
                              Fx(3, 3) * V_xx_prev(3, 0)) +
                  Fx(1, 3) * (Fx(0, 3) * V_xx_prev(0, 1) + Fx(1, 3) * V_xx_prev(1, 1) + Fx(2, 3) * V_xx_prev(2, 1) +
                              Fx(3, 3) * V_xx_prev(3, 1)) +
                  Fx(2, 3) * (Fx(0, 3) * V_xx_prev(0, 2) + Fx(1, 3) * V_xx_prev(1, 2) + Fx(2, 3) * V_xx_prev(2, 2) +
                              Fx(3, 3) * V_xx_prev(3, 2)) +
                  Fx(3, 3) * (Fx(0, 3) * V_xx_prev(0, 3) + Fx(1, 3) * V_xx_prev(1, 3) + Fx(2, 3) * V_xx_prev(2, 3) +
                              Fx(3, 3) * V_xx_prev(3, 3)) +
                  Sigma_(13) * g_x_13_3_p2 + Sigma_(14) * g_x_14_3_p2 + Sigma_(15) * g_x_15_3_p2 +
                  Sigma_(16) * g_x_16_3_p2 + Sigma_(2) * g_x_2_3_p2 + Sigma_(3) * g_x_3_3_p2 + Sigma_(4) * g_x_4_3_p2 +
                  Sigma_(5) * g_x_5_3_p2 + Sigma_(8) * g_x_8_3_p2;
    Q_xx_(3, 4) = Fx(3, 4) * (Fx(0, 3) * V_xx_prev(0, 3) + Fx(1, 3) * V_xx_prev(1, 3) + Fx(2, 3) * V_xx_prev(2, 3) +
                              Fx(3, 3) * V_xx_prev(3, 3)) +
                  Fx(4, 4) * (Fx(0, 3) * V_xx_prev(0, 4) + Fx(1, 3) * V_xx_prev(1, 4) + Fx(2, 3) * V_xx_prev(2, 4) +
                              Fx(3, 3) * V_xx_prev(3, 4));
    Q_xx_(4, 0) = Fx(0, 0) * (Fx(3, 4) * V_xx_prev(3, 0) + Fx(4, 4) * V_xx_prev(4, 0));
    Q_xx_(4, 1) = Fx(0, 1) * (Fx(3, 4) * V_xx_prev(3, 0) + Fx(4, 4) * V_xx_prev(4, 0)) +
                  Fx(1, 1) * (Fx(3, 4) * V_xx_prev(3, 1) + Fx(4, 4) * V_xx_prev(4, 1));
    Q_xx_(4, 2) = Fx(0, 2) * (Fx(3, 4) * V_xx_prev(3, 0) + Fx(4, 4) * V_xx_prev(4, 0)) +
                  Fx(2, 2) * (Fx(3, 4) * V_xx_prev(3, 2) + Fx(4, 4) * V_xx_prev(4, 2));
    Q_xx_(4, 3) = Fx(0, 3) * (Fx(3, 4) * V_xx_prev(3, 0) + Fx(4, 4) * V_xx_prev(4, 0)) +
                  Fx(1, 3) * (Fx(3, 4) * V_xx_prev(3, 1) + Fx(4, 4) * V_xx_prev(4, 1)) +
                  Fx(2, 3) * (Fx(3, 4) * V_xx_prev(3, 2) + Fx(4, 4) * V_xx_prev(4, 2)) +
                  Fx(3, 3) * (Fx(3, 4) * V_xx_prev(3, 3) + Fx(4, 4) * V_xx_prev(4, 3));
    Q_xx_(4, 4) = Fx(3, 4) * (Fx(3, 4) * V_xx_prev(3, 3) + Fx(4, 4) * V_xx_prev(4, 3)) +
                  Fx(4, 4) * (Fx(3, 4) * V_xx_prev(3, 4) + Fx(4, 4) * V_xx_prev(4, 4)) + Sigma_(6) * g_x_6_4_p2 +
                  Sigma_(7) * g_x_7_4_p2 + l_xx(4, 4);

    // Evaluation of Matrix Q_xu_
    Q_xu_(0, 0) = Fu(4, 0) * Fx(0, 0) * V_xx_prev(0, 4);
    Q_xu_(1, 0) = Fu(4, 0) * (Fx(0, 1) * V_xx_prev(0, 4) + Fx(1, 1) * V_xx_prev(1, 4));
    Q_xu_(1, 1) = Sigma_(11) * g_u(11, 1) * g_x(11, 1) + Sigma_(12) * g_u(12, 1) * g_x(12, 1) +
                  Sigma_(13) * g_u(13, 1) * g_x(13, 1) + Sigma_(14) * g_u(14, 1) * g_x(14, 1) +
                  Sigma_(15) * g_u(15, 1) * g_x(15, 1) + Sigma_(16) * g_u(16, 1) * g_x(16, 1);
    Q_xu_(2, 0) = Fu(4, 0) * (Fx(0, 2) * V_xx_prev(0, 4) + Fx(2, 2) * V_xx_prev(2, 4));
    Q_xu_(2, 1) = Sigma_(11) * g_u(11, 1) * g_x(11, 2) + Sigma_(12) * g_u(12, 1) * g_x(12, 2) +
                  Sigma_(13) * g_u(13, 1) * g_x(13, 2) + Sigma_(14) * g_u(14, 1) * g_x(14, 2) +
                  Sigma_(15) * g_u(15, 1) * g_x(15, 2) + Sigma_(16) * g_u(16, 1) * g_x(16, 2);
    Q_xu_(3, 0) = Fu(4, 0) * (Fx(0, 3) * V_xx_prev(0, 4) + Fx(1, 3) * V_xx_prev(1, 4) + Fx(2, 3) * V_xx_prev(2, 4) +
                              Fx(3, 3) * V_xx_prev(3, 4));
    Q_xu_(3, 1) = Sigma_(13) * g_u(13, 1) * g_x(13, 3) + Sigma_(14) * g_u(14, 1) * g_x(14, 3) +
                  Sigma_(15) * g_u(15, 1) * g_x(15, 3) + Sigma_(16) * g_u(16, 1) * g_x(16, 3);
    Q_xu_(4, 0) = Fu(4, 0) * (Fx(3, 4) * V_xx_prev(3, 4) + Fx(4, 4) * V_xx_prev(4, 4));

    // Evaluation of Matrix Q_uu_
    Q_uu_(0, 0) = Fu_4_0_p2 * V_xx_prev(4, 4) + Sigma_(10) * g_u_10_0_p2 + Sigma_(9) * g_u_9_0_p2 + l_uu(0, 0);
    Q_uu_(1, 1) = Sigma_(11) * g_u_11_1_p2 + Sigma_(12) * g_u_12_1_p2 + Sigma_(13) * g_u_13_1_p2 +
                  Sigma_(14) * g_u_14_1_p2 + Sigma_(15) * g_u_15_1_p2 + Sigma_(16) * g_u_16_1_p2 + l_uu(1, 1);
  }

  virtual void updateQG(const Eigen::VectorXd& sl_x, const Eigen::VectorXd& sl_u, const Eigen::MatrixXd& g_x,
                        const Eigen::MatrixXd& g_u, const Eigen::VectorXd& r_f, const Eigen::MatrixXd& Fx,
                        const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& V_xx_prev,
                        const Eigen::VectorXd& V_x_prev) override {
    // Evaluation of Vector Q_x_
    Q_x_(0) = Fx(0, 0) * (V_x_prev(0) + V_xx_prev(0, 0) * r_f(0) + V_xx_prev(0, 1) * r_f(1) + V_xx_prev(0, 2) * r_f(2) +
                          V_xx_prev(0, 3) * r_f(3) + V_xx_prev(0, 4) * r_f(4)) +
              sl_x(0);
    Q_x_(1) = Fx(0, 1) * (V_x_prev(0) + V_xx_prev(0, 0) * r_f(0) + V_xx_prev(0, 1) * r_f(1) + V_xx_prev(0, 2) * r_f(2) +
                          V_xx_prev(0, 3) * r_f(3) + V_xx_prev(0, 4) * r_f(4)) +
              Fx(1, 1) * (V_x_prev(1) + V_xx_prev(1, 0) * r_f(0) + V_xx_prev(1, 1) * r_f(1) + V_xx_prev(1, 2) * r_f(2) +
                          V_xx_prev(1, 3) * r_f(3) + V_xx_prev(1, 4) * r_f(4)) +
              V_(0) * g_x(0, 1) + V_(1) * g_x(1, 1) + V_(11) * g_x(11, 1) + V_(12) * g_x(12, 1) + V_(13) * g_x(13, 1) +
              V_(14) * g_x(14, 1) + V_(15) * g_x(15, 1) + V_(16) * g_x(16, 1) + V_(2) * g_x(2, 1) + V_(3) * g_x(3, 1) +
              V_(4) * g_x(4, 1) + V_(5) * g_x(5, 1) + sl_x(1);
    Q_x_(2) = Fx(0, 2) * (V_x_prev(0) + V_xx_prev(0, 0) * r_f(0) + V_xx_prev(0, 1) * r_f(1) + V_xx_prev(0, 2) * r_f(2) +
                          V_xx_prev(0, 3) * r_f(3) + V_xx_prev(0, 4) * r_f(4)) +
              Fx(2, 2) * (V_x_prev(2) + V_xx_prev(2, 0) * r_f(0) + V_xx_prev(2, 1) * r_f(1) + V_xx_prev(2, 2) * r_f(2) +
                          V_xx_prev(2, 3) * r_f(3) + V_xx_prev(2, 4) * r_f(4)) +
              V_(0) * g_x(0, 2) + V_(1) * g_x(1, 2) + V_(11) * g_x(11, 2) + V_(12) * g_x(12, 2) + V_(13) * g_x(13, 2) +
              V_(14) * g_x(14, 2) + V_(15) * g_x(15, 2) + V_(16) * g_x(16, 2) + V_(2) * g_x(2, 2) + V_(3) * g_x(3, 2) +
              V_(4) * g_x(4, 2) + V_(5) * g_x(5, 2) + sl_x(2);
    Q_x_(3) = Fx(0, 3) * (V_x_prev(0) + V_xx_prev(0, 0) * r_f(0) + V_xx_prev(0, 1) * r_f(1) + V_xx_prev(0, 2) * r_f(2) +
                          V_xx_prev(0, 3) * r_f(3) + V_xx_prev(0, 4) * r_f(4)) +
              Fx(1, 3) * (V_x_prev(1) + V_xx_prev(1, 0) * r_f(0) + V_xx_prev(1, 1) * r_f(1) + V_xx_prev(1, 2) * r_f(2) +
                          V_xx_prev(1, 3) * r_f(3) + V_xx_prev(1, 4) * r_f(4)) +
              Fx(2, 3) * (V_x_prev(2) + V_xx_prev(2, 0) * r_f(0) + V_xx_prev(2, 1) * r_f(1) + V_xx_prev(2, 2) * r_f(2) +
                          V_xx_prev(2, 3) * r_f(3) + V_xx_prev(2, 4) * r_f(4)) +
              Fx(3, 3) * (V_x_prev(3) + V_xx_prev(3, 0) * r_f(0) + V_xx_prev(3, 1) * r_f(1) + V_xx_prev(3, 2) * r_f(2) +
                          V_xx_prev(3, 3) * r_f(3) + V_xx_prev(3, 4) * r_f(4)) +
              V_(13) * g_x(13, 3) + V_(14) * g_x(14, 3) + V_(15) * g_x(15, 3) + V_(16) * g_x(16, 3) +
              V_(2) * g_x(2, 3) + V_(3) * g_x(3, 3) + V_(4) * g_x(4, 3) + V_(5) * g_x(5, 3) + V_(8) * g_x(8, 3) +
              sl_x(3);
    Q_x_(4) = Fx(3, 4) * (V_x_prev(3) + V_xx_prev(3, 0) * r_f(0) + V_xx_prev(3, 1) * r_f(1) + V_xx_prev(3, 2) * r_f(2) +
                          V_xx_prev(3, 3) * r_f(3) + V_xx_prev(3, 4) * r_f(4)) +
              Fx(4, 4) * (V_x_prev(4) + V_xx_prev(4, 0) * r_f(0) + V_xx_prev(4, 1) * r_f(1) + V_xx_prev(4, 2) * r_f(2) +
                          V_xx_prev(4, 3) * r_f(3) + V_xx_prev(4, 4) * r_f(4)) +
              V_(6) * g_x(6, 4) + V_(7) * g_x(7, 4) + sl_x(4);

    // Evaluation of Vector Q_u_
    Q_u_(0) = Fu(4, 0) * (V_x_prev(4) + V_xx_prev(4, 0) * r_f(0) + V_xx_prev(4, 1) * r_f(1) + V_xx_prev(4, 2) * r_f(2) +
                          V_xx_prev(4, 3) * r_f(3) + V_xx_prev(4, 4) * r_f(4)) +
              V_(10) * g_u(10, 0) + V_(9) * g_u(9, 0) + sl_u(0);
    Q_u_(1) = V_(11) * g_u(11, 1) + V_(12) * g_u(12, 1) + V_(13) * g_u(13, 1) + V_(14) * g_u(14, 1) +
              V_(15) * g_u(15, 1) + V_(16) * g_u(16, 1) + sl_u(1);
  }

  virtual void updateQuuInv() override {
    // Evaluation of Matrix Q_uu_inv_
    Q_uu_inv_(0, 0) = 1.0 / Q_uu_(0, 0);
    Q_uu_inv_(1, 1) = 1.0 / Q_uu_(1, 1);
  }

  virtual void updateKVU(Eigen::MatrixXd& k_ux, Eigen::VectorXd& v_u) override {
    // Evaluation of Matrix k_ux
    k_ux(0, 0) = -Q_uu_inv_(0, 0) * Q_xu_(0, 0);
    k_ux(0, 1) = -Q_uu_inv_(0, 0) * Q_xu_(1, 0);
    k_ux(0, 2) = -Q_uu_inv_(0, 0) * Q_xu_(2, 0);
    k_ux(0, 3) = -Q_uu_inv_(0, 0) * Q_xu_(3, 0);
    k_ux(0, 4) = -Q_uu_inv_(0, 0) * Q_xu_(4, 0);
    k_ux(1, 1) = -Q_uu_inv_(1, 1) * Q_xu_(1, 1);
    k_ux(1, 2) = -Q_uu_inv_(1, 1) * Q_xu_(2, 1);
    k_ux(1, 3) = -Q_uu_inv_(1, 1) * Q_xu_(3, 1);

    // Evaluation of Vector v_u
    v_u(0) = -Q_u_(0) * Q_uu_inv_(0, 0);
    v_u(1) = -Q_u_(1) * Q_uu_inv_(1, 1);
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
    V_xx(0, 0) = Q_xu_(0, 0) * k_ux(0, 0) + Q_xx_(0, 0);
    V_xx(0, 1) = Q_xu_(0, 0) * k_ux(0, 1) + Q_xx_(0, 1);
    V_xx(0, 2) = Q_xu_(0, 0) * k_ux(0, 2) + Q_xx_(0, 2);
    V_xx(0, 3) = Q_xu_(0, 0) * k_ux(0, 3) + Q_xx_(0, 3);
    V_xx(0, 4) = Q_xu_(0, 0) * k_ux(0, 4) + Q_xx_(0, 4);
    V_xx(1, 0) = Q_xu_(1, 0) * k_ux(0, 0) + Q_xx_(0, 1);
    V_xx(1, 1) = Q_xu_(1, 0) * k_ux(0, 1) + Q_xu_(1, 1) * k_ux(1, 1) + Q_xx_(1, 1);
    V_xx(1, 2) = Q_xu_(1, 0) * k_ux(0, 2) + Q_xu_(1, 1) * k_ux(1, 2) + Q_xx_(1, 2);
    V_xx(1, 3) = Q_xu_(1, 0) * k_ux(0, 3) + Q_xu_(1, 1) * k_ux(1, 3) + Q_xx_(1, 3);
    V_xx(1, 4) = Q_xu_(1, 0) * k_ux(0, 4) + Q_xx_(1, 4);
    V_xx(2, 0) = Q_xu_(2, 0) * k_ux(0, 0) + Q_xx_(0, 2);
    V_xx(2, 1) = Q_xu_(2, 0) * k_ux(0, 1) + Q_xu_(2, 1) * k_ux(1, 1) + Q_xx_(1, 2);
    V_xx(2, 2) = Q_xu_(2, 0) * k_ux(0, 2) + Q_xu_(2, 1) * k_ux(1, 2) + Q_xx_(2, 2);
    V_xx(2, 3) = Q_xu_(2, 0) * k_ux(0, 3) + Q_xu_(2, 1) * k_ux(1, 3) + Q_xx_(2, 3);
    V_xx(2, 4) = Q_xu_(2, 0) * k_ux(0, 4) + Q_xx_(2, 4);
    V_xx(3, 0) = Q_xu_(3, 0) * k_ux(0, 0) + Q_xx_(0, 3);
    V_xx(3, 1) = Q_xu_(3, 0) * k_ux(0, 1) + Q_xu_(3, 1) * k_ux(1, 1) + Q_xx_(1, 3);
    V_xx(3, 2) = Q_xu_(3, 0) * k_ux(0, 2) + Q_xu_(3, 1) * k_ux(1, 2) + Q_xx_(2, 3);
    V_xx(3, 3) = Q_xu_(3, 0) * k_ux(0, 3) + Q_xu_(3, 1) * k_ux(1, 3) + Q_xx_(3, 3);
    V_xx(3, 4) = Q_xu_(3, 0) * k_ux(0, 4) + Q_xx_(3, 4);
    V_xx(4, 0) = Q_xu_(4, 0) * k_ux(0, 0) + Q_xx_(0, 4);
    V_xx(4, 1) = Q_xu_(4, 0) * k_ux(0, 1) + Q_xx_(1, 4);
    V_xx(4, 2) = Q_xu_(4, 0) * k_ux(0, 2) + Q_xx_(2, 4);
    V_xx(4, 3) = Q_xu_(4, 0) * k_ux(0, 3) + Q_xx_(3, 4);
    V_xx(4, 4) = Q_xu_(4, 0) * k_ux(0, 4) + Q_xx_(4, 4);

    // Evaluation of Vector V_x
    V_x(0) = Q_x_(0) + Q_xu_(0, 0) * v_u(0);
    V_x(1) = Q_x_(1) + Q_xu_(1, 0) * v_u(0) + Q_xu_(1, 1) * v_u(1);
    V_x(2) = Q_x_(2) + Q_xu_(2, 0) * v_u(0) + Q_xu_(2, 1) * v_u(1);
    V_x(3) = Q_x_(3) + Q_xu_(3, 0) * v_u(0) + Q_xu_(3, 1) * v_u(1);
    V_x(4) = Q_x_(4) + Q_xu_(4, 0) * v_u(0);
  }

  virtual void updateQnx(const Eigen::VectorXd& r_f, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& Fu,
                         const Eigen::MatrixXd& h_u, const Eigen::MatrixXd& k_ux, const Eigen::MatrixXd& k_ex,
                         const Eigen::MatrixXd& k_ue, const Eigen::VectorXd& v_u, const Eigen::VectorXd& v_e) override {
    // Evaluation of Matrix Q_un_
    Q_un_(0, 4) = -Fu(4, 0) * Q_uu_inv_(0, 0);

    // Evaluation of Matrix Q_nuu_
    Q_nuu_(0, 0) = -Q_uu_(0, 0);
    Q_nuu_(1, 1) = -Q_uu_(1, 1);

    // Evaluation of Matrix Q_vnx_
    Q_vnx_(0, 0) = Fx(0, 0);
    Q_vnx_(0, 1) = Fx(0, 1);
    Q_vnx_(0, 2) = Fx(0, 2);
    Q_vnx_(0, 3) = Fx(0, 3);
    Q_vnx_(1, 1) = Fx(1, 1);
    Q_vnx_(1, 3) = Fx(1, 3);
    Q_vnx_(2, 2) = Fx(2, 2);
    Q_vnx_(2, 3) = Fx(2, 3);
    Q_vnx_(3, 3) = Fx(3, 3);
    Q_vnx_(3, 4) = Fx(3, 4);
    Q_vnx_(4, 0) = Fu(4, 0) * k_ux(0, 0);
    Q_vnx_(4, 1) = Fu(4, 0) * k_ux(0, 1);
    Q_vnx_(4, 2) = Fu(4, 0) * k_ux(0, 2);
    Q_vnx_(4, 3) = Fu(4, 0) * k_ux(0, 3);
    Q_vnx_(4, 4) = Fu(4, 0) * k_ux(0, 4) + Fx(4, 4);

    // Evaluation of Vector Q_vn_
    Q_vn_(0) = r_f(0);
    Q_vn_(1) = r_f(1);
    Q_vn_(2) = r_f(2);
    Q_vn_(3) = r_f(3);
    Q_vn_(4) = Fu(4, 0) * v_u(0) + r_f(4);
  }
};

template <>
class ParkLateralGeneralIpmEvaluator<StageType::INITIAL, OcpConfig::FORWARD_EULER> : public IpmEvaluator {
 public:
  ParkLateralGeneralIpmEvaluator() : IpmEvaluator(5, 2, 2, 0, 5) {}
  virtual ~ParkLateralGeneralIpmEvaluator() = default;

  virtual void updateLX(const Eigen::VectorXd& l_x, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& g_x,
                        const Eigen::MatrixXd& h_x, const Eigen::MatrixXd& m_x, const Eigen::VectorXd& p,
                        const Eigen::VectorXd& lambda, const Eigen::VectorXd& eta, const Eigen::VectorXd& nu,
                        Eigen::VectorXd& sl_x) const override {
    // Evaluation of Vector sl_x
    sl_x(0) = Fx(0, 0) * p(0) + m_x(0, 0) * nu(0);
    sl_x(1) = Fx(0, 1) * p(0) + Fx(1, 1) * p(1) + l_x(1) + m_x(1, 1) * nu(1);
    sl_x(2) = Fx(0, 2) * p(0) + Fx(2, 2) * p(2) + l_x(2) + m_x(2, 2) * nu(2);
    sl_x(3) = Fx(0, 3) * p(0) + Fx(1, 3) * p(1) + Fx(2, 3) * p(2) + Fx(3, 3) * p(3) + m_x(3, 3) * nu(3);
    sl_x(4) = Fx(3, 4) * p(3) + Fx(4, 4) * p(4) + l_x(4) + m_x(4, 4) * nu(4);
  }

  virtual void updateLU(const Eigen::VectorXd& l_u, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& g_u,
                        const Eigen::MatrixXd& h_u, const Eigen::VectorXd& p, const Eigen::VectorXd& lambda,
                        const Eigen::VectorXd& eta, Eigen::VectorXd& sl_u) const override {
    // Evaluation of Vector sl_u
    sl_u(0) = Fu(4, 0) * p(4) + g_u(0, 0) * lambda(0) + g_u(1, 0) * lambda(1) + l_u(0);
    sl_u(1) = l_u(1);
  }

  virtual void updateQH(const Eigen::MatrixXd& l_xx, const Eigen::MatrixXd& l_xu, const Eigen::MatrixXd& l_uu,
                        const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::MatrixXd& Fx,
                        const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& V_xx_prev) override {
    // Determine internal variables
    const double Fu_4_0_p2 = Fu(4, 0) * Fu(4, 0);
    const double Fx_0_0_p2 = Fx(0, 0) * Fx(0, 0);
    const double g_u_0_0_p2 = g_u(0, 0) * g_u(0, 0);
    const double g_u_1_0_p2 = g_u(1, 0) * g_u(1, 0);

    // Evaluation of Matrix Q_xx_
    Q_xx_(0, 0) = Fx_0_0_p2 * V_xx_prev(0, 0);
    Q_xx_(0, 1) = Fx(0, 0) * Fx(0, 1) * V_xx_prev(0, 0) + Fx(0, 0) * Fx(1, 1) * V_xx_prev(0, 1);
    Q_xx_(0, 2) = Fx(0, 0) * Fx(0, 2) * V_xx_prev(0, 0) + Fx(0, 0) * Fx(2, 2) * V_xx_prev(0, 2);
    Q_xx_(0, 3) = Fx(0, 0) * Fx(0, 3) * V_xx_prev(0, 0) + Fx(0, 0) * Fx(1, 3) * V_xx_prev(0, 1) +
                  Fx(0, 0) * Fx(2, 3) * V_xx_prev(0, 2) + Fx(0, 0) * Fx(3, 3) * V_xx_prev(0, 3);
    Q_xx_(0, 4) = Fx(0, 0) * Fx(3, 4) * V_xx_prev(0, 3) + Fx(0, 0) * Fx(4, 4) * V_xx_prev(0, 4);
    Q_xx_(1, 0) = Fx(0, 0) * (Fx(0, 1) * V_xx_prev(0, 0) + Fx(1, 1) * V_xx_prev(1, 0));
    Q_xx_(1, 1) = Fx(0, 1) * (Fx(0, 1) * V_xx_prev(0, 0) + Fx(1, 1) * V_xx_prev(1, 0)) +
                  Fx(1, 1) * (Fx(0, 1) * V_xx_prev(0, 1) + Fx(1, 1) * V_xx_prev(1, 1)) + l_xx(1, 1);
    Q_xx_(1, 2) = Fx(0, 2) * (Fx(0, 1) * V_xx_prev(0, 0) + Fx(1, 1) * V_xx_prev(1, 0)) +
                  Fx(2, 2) * (Fx(0, 1) * V_xx_prev(0, 2) + Fx(1, 1) * V_xx_prev(1, 2)) + l_xx(1, 2);
    Q_xx_(1, 3) = Fx(0, 3) * (Fx(0, 1) * V_xx_prev(0, 0) + Fx(1, 1) * V_xx_prev(1, 0)) +
                  Fx(1, 3) * (Fx(0, 1) * V_xx_prev(0, 1) + Fx(1, 1) * V_xx_prev(1, 1)) +
                  Fx(2, 3) * (Fx(0, 1) * V_xx_prev(0, 2) + Fx(1, 1) * V_xx_prev(1, 2)) +
                  Fx(3, 3) * (Fx(0, 1) * V_xx_prev(0, 3) + Fx(1, 1) * V_xx_prev(1, 3));
    Q_xx_(1, 4) = Fx(3, 4) * (Fx(0, 1) * V_xx_prev(0, 3) + Fx(1, 1) * V_xx_prev(1, 3)) +
                  Fx(4, 4) * (Fx(0, 1) * V_xx_prev(0, 4) + Fx(1, 1) * V_xx_prev(1, 4));
    Q_xx_(2, 0) = Fx(0, 0) * (Fx(0, 2) * V_xx_prev(0, 0) + Fx(2, 2) * V_xx_prev(2, 0));
    Q_xx_(2, 1) = Fx(0, 1) * (Fx(0, 2) * V_xx_prev(0, 0) + Fx(2, 2) * V_xx_prev(2, 0)) +
                  Fx(1, 1) * (Fx(0, 2) * V_xx_prev(0, 1) + Fx(2, 2) * V_xx_prev(2, 1)) + l_xx(2, 1);
    Q_xx_(2, 2) = Fx(0, 2) * (Fx(0, 2) * V_xx_prev(0, 0) + Fx(2, 2) * V_xx_prev(2, 0)) +
                  Fx(2, 2) * (Fx(0, 2) * V_xx_prev(0, 2) + Fx(2, 2) * V_xx_prev(2, 2)) + l_xx(2, 2);
    Q_xx_(2, 3) = Fx(0, 3) * (Fx(0, 2) * V_xx_prev(0, 0) + Fx(2, 2) * V_xx_prev(2, 0)) +
                  Fx(1, 3) * (Fx(0, 2) * V_xx_prev(0, 1) + Fx(2, 2) * V_xx_prev(2, 1)) +
                  Fx(2, 3) * (Fx(0, 2) * V_xx_prev(0, 2) + Fx(2, 2) * V_xx_prev(2, 2)) +
                  Fx(3, 3) * (Fx(0, 2) * V_xx_prev(0, 3) + Fx(2, 2) * V_xx_prev(2, 3));
    Q_xx_(2, 4) = Fx(3, 4) * (Fx(0, 2) * V_xx_prev(0, 3) + Fx(2, 2) * V_xx_prev(2, 3)) +
                  Fx(4, 4) * (Fx(0, 2) * V_xx_prev(0, 4) + Fx(2, 2) * V_xx_prev(2, 4));
    Q_xx_(3, 0) = Fx(0, 0) * (Fx(0, 3) * V_xx_prev(0, 0) + Fx(1, 3) * V_xx_prev(1, 0) + Fx(2, 3) * V_xx_prev(2, 0) +
                              Fx(3, 3) * V_xx_prev(3, 0));
    Q_xx_(3, 1) = Fx(0, 1) * (Fx(0, 3) * V_xx_prev(0, 0) + Fx(1, 3) * V_xx_prev(1, 0) + Fx(2, 3) * V_xx_prev(2, 0) +
                              Fx(3, 3) * V_xx_prev(3, 0)) +
                  Fx(1, 1) * (Fx(0, 3) * V_xx_prev(0, 1) + Fx(1, 3) * V_xx_prev(1, 1) + Fx(2, 3) * V_xx_prev(2, 1) +
                              Fx(3, 3) * V_xx_prev(3, 1));
    Q_xx_(3, 2) = Fx(0, 2) * (Fx(0, 3) * V_xx_prev(0, 0) + Fx(1, 3) * V_xx_prev(1, 0) + Fx(2, 3) * V_xx_prev(2, 0) +
                              Fx(3, 3) * V_xx_prev(3, 0)) +
                  Fx(2, 2) * (Fx(0, 3) * V_xx_prev(0, 2) + Fx(1, 3) * V_xx_prev(1, 2) + Fx(2, 3) * V_xx_prev(2, 2) +
                              Fx(3, 3) * V_xx_prev(3, 2));
    Q_xx_(3, 3) = Fx(0, 3) * (Fx(0, 3) * V_xx_prev(0, 0) + Fx(1, 3) * V_xx_prev(1, 0) + Fx(2, 3) * V_xx_prev(2, 0) +
                              Fx(3, 3) * V_xx_prev(3, 0)) +
                  Fx(1, 3) * (Fx(0, 3) * V_xx_prev(0, 1) + Fx(1, 3) * V_xx_prev(1, 1) + Fx(2, 3) * V_xx_prev(2, 1) +
                              Fx(3, 3) * V_xx_prev(3, 1)) +
                  Fx(2, 3) * (Fx(0, 3) * V_xx_prev(0, 2) + Fx(1, 3) * V_xx_prev(1, 2) + Fx(2, 3) * V_xx_prev(2, 2) +
                              Fx(3, 3) * V_xx_prev(3, 2)) +
                  Fx(3, 3) * (Fx(0, 3) * V_xx_prev(0, 3) + Fx(1, 3) * V_xx_prev(1, 3) + Fx(2, 3) * V_xx_prev(2, 3) +
                              Fx(3, 3) * V_xx_prev(3, 3));
    Q_xx_(3, 4) = Fx(3, 4) * (Fx(0, 3) * V_xx_prev(0, 3) + Fx(1, 3) * V_xx_prev(1, 3) + Fx(2, 3) * V_xx_prev(2, 3) +
                              Fx(3, 3) * V_xx_prev(3, 3)) +
                  Fx(4, 4) * (Fx(0, 3) * V_xx_prev(0, 4) + Fx(1, 3) * V_xx_prev(1, 4) + Fx(2, 3) * V_xx_prev(2, 4) +
                              Fx(3, 3) * V_xx_prev(3, 4));
    Q_xx_(4, 0) = Fx(0, 0) * (Fx(3, 4) * V_xx_prev(3, 0) + Fx(4, 4) * V_xx_prev(4, 0));
    Q_xx_(4, 1) = Fx(0, 1) * (Fx(3, 4) * V_xx_prev(3, 0) + Fx(4, 4) * V_xx_prev(4, 0)) +
                  Fx(1, 1) * (Fx(3, 4) * V_xx_prev(3, 1) + Fx(4, 4) * V_xx_prev(4, 1));
    Q_xx_(4, 2) = Fx(0, 2) * (Fx(3, 4) * V_xx_prev(3, 0) + Fx(4, 4) * V_xx_prev(4, 0)) +
                  Fx(2, 2) * (Fx(3, 4) * V_xx_prev(3, 2) + Fx(4, 4) * V_xx_prev(4, 2));
    Q_xx_(4, 3) = Fx(0, 3) * (Fx(3, 4) * V_xx_prev(3, 0) + Fx(4, 4) * V_xx_prev(4, 0)) +
                  Fx(1, 3) * (Fx(3, 4) * V_xx_prev(3, 1) + Fx(4, 4) * V_xx_prev(4, 1)) +
                  Fx(2, 3) * (Fx(3, 4) * V_xx_prev(3, 2) + Fx(4, 4) * V_xx_prev(4, 2)) +
                  Fx(3, 3) * (Fx(3, 4) * V_xx_prev(3, 3) + Fx(4, 4) * V_xx_prev(4, 3));
    Q_xx_(4, 4) = Fx(3, 4) * (Fx(3, 4) * V_xx_prev(3, 3) + Fx(4, 4) * V_xx_prev(4, 3)) +
                  Fx(4, 4) * (Fx(3, 4) * V_xx_prev(3, 4) + Fx(4, 4) * V_xx_prev(4, 4)) + l_xx(4, 4);

    // Evaluation of Matrix Q_xu_
    Q_xu_(0, 0) = Fu(4, 0) * Fx(0, 0) * V_xx_prev(0, 4);
    Q_xu_(1, 0) = Fu(4, 0) * (Fx(0, 1) * V_xx_prev(0, 4) + Fx(1, 1) * V_xx_prev(1, 4));
    Q_xu_(2, 0) = Fu(4, 0) * (Fx(0, 2) * V_xx_prev(0, 4) + Fx(2, 2) * V_xx_prev(2, 4));
    Q_xu_(3, 0) = Fu(4, 0) * (Fx(0, 3) * V_xx_prev(0, 4) + Fx(1, 3) * V_xx_prev(1, 4) + Fx(2, 3) * V_xx_prev(2, 4) +
                              Fx(3, 3) * V_xx_prev(3, 4));
    Q_xu_(4, 0) = Fu(4, 0) * (Fx(3, 4) * V_xx_prev(3, 4) + Fx(4, 4) * V_xx_prev(4, 4));

    // Evaluation of Matrix Q_uu_
    Q_uu_(0, 0) = Fu_4_0_p2 * V_xx_prev(4, 4) + Sigma_(0) * g_u_0_0_p2 + Sigma_(1) * g_u_1_0_p2 + l_uu(0, 0);
    Q_uu_(1, 1) = l_uu(1, 1);
  }

  virtual void updateQG(const Eigen::VectorXd& sl_x, const Eigen::VectorXd& sl_u, const Eigen::MatrixXd& g_x,
                        const Eigen::MatrixXd& g_u, const Eigen::VectorXd& r_f, const Eigen::MatrixXd& Fx,
                        const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& V_xx_prev,
                        const Eigen::VectorXd& V_x_prev) override {
    // Evaluation of Vector Q_x_
    Q_x_(0) = Fx(0, 0) * (V_x_prev(0) + V_xx_prev(0, 0) * r_f(0) + V_xx_prev(0, 1) * r_f(1) + V_xx_prev(0, 2) * r_f(2) +
                          V_xx_prev(0, 3) * r_f(3) + V_xx_prev(0, 4) * r_f(4)) +
              sl_x(0);
    Q_x_(1) = Fx(0, 1) * (V_x_prev(0) + V_xx_prev(0, 0) * r_f(0) + V_xx_prev(0, 1) * r_f(1) + V_xx_prev(0, 2) * r_f(2) +
                          V_xx_prev(0, 3) * r_f(3) + V_xx_prev(0, 4) * r_f(4)) +
              Fx(1, 1) * (V_x_prev(1) + V_xx_prev(1, 0) * r_f(0) + V_xx_prev(1, 1) * r_f(1) + V_xx_prev(1, 2) * r_f(2) +
                          V_xx_prev(1, 3) * r_f(3) + V_xx_prev(1, 4) * r_f(4)) +
              sl_x(1);
    Q_x_(2) = Fx(0, 2) * (V_x_prev(0) + V_xx_prev(0, 0) * r_f(0) + V_xx_prev(0, 1) * r_f(1) + V_xx_prev(0, 2) * r_f(2) +
                          V_xx_prev(0, 3) * r_f(3) + V_xx_prev(0, 4) * r_f(4)) +
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
    Q_x_(4) = Fx(3, 4) * (V_x_prev(3) + V_xx_prev(3, 0) * r_f(0) + V_xx_prev(3, 1) * r_f(1) + V_xx_prev(3, 2) * r_f(2) +
                          V_xx_prev(3, 3) * r_f(3) + V_xx_prev(3, 4) * r_f(4)) +
              Fx(4, 4) * (V_x_prev(4) + V_xx_prev(4, 0) * r_f(0) + V_xx_prev(4, 1) * r_f(1) + V_xx_prev(4, 2) * r_f(2) +
                          V_xx_prev(4, 3) * r_f(3) + V_xx_prev(4, 4) * r_f(4)) +
              sl_x(4);

    // Evaluation of Vector Q_u_
    Q_u_(0) = Fu(4, 0) * (V_x_prev(4) + V_xx_prev(4, 0) * r_f(0) + V_xx_prev(4, 1) * r_f(1) + V_xx_prev(4, 2) * r_f(2) +
                          V_xx_prev(4, 3) * r_f(3) + V_xx_prev(4, 4) * r_f(4)) +
              V_(0) * g_u(0, 0) + V_(1) * g_u(1, 0) + sl_u(0);
    Q_u_(1) = sl_u(1);
  }

  virtual void updateQuuInv() override {
    // Evaluation of Matrix Q_uu_inv_
    Q_uu_inv_(0, 0) = 1.0 / Q_uu_(0, 0);
    Q_uu_inv_(1, 1) = 1.0 / Q_uu_(1, 1);
  }

  virtual void updateKVU(Eigen::MatrixXd& k_ux, Eigen::VectorXd& v_u) override {
    // Evaluation of Matrix k_ux
    k_ux(0, 0) = -Q_uu_inv_(0, 0) * Q_xu_(0, 0);
    k_ux(0, 1) = -Q_uu_inv_(0, 0) * Q_xu_(1, 0);
    k_ux(0, 2) = -Q_uu_inv_(0, 0) * Q_xu_(2, 0);
    k_ux(0, 3) = -Q_uu_inv_(0, 0) * Q_xu_(3, 0);
    k_ux(0, 4) = -Q_uu_inv_(0, 0) * Q_xu_(4, 0);

    // Evaluation of Vector v_u
    v_u(0) = -Q_u_(0) * Q_uu_inv_(0, 0);
    v_u(1) = -Q_u_(1) * Q_uu_inv_(1, 1);
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
    V_xx(0, 0) = Q_xu_(0, 0) * k_ux(0, 0) + Q_xx_(0, 0);
    V_xx(0, 1) = Q_xu_(0, 0) * k_ux(0, 1) + Q_xx_(0, 1);
    V_xx(0, 2) = Q_xu_(0, 0) * k_ux(0, 2) + Q_xx_(0, 2);
    V_xx(0, 3) = Q_xu_(0, 0) * k_ux(0, 3) + Q_xx_(0, 3);
    V_xx(0, 4) = Q_xu_(0, 0) * k_ux(0, 4) + Q_xx_(0, 4);
    V_xx(1, 0) = Q_xu_(1, 0) * k_ux(0, 0) + Q_xx_(0, 1);
    V_xx(1, 1) = Q_xu_(1, 0) * k_ux(0, 1) + Q_xx_(1, 1);
    V_xx(1, 2) = Q_xu_(1, 0) * k_ux(0, 2) + Q_xx_(1, 2);
    V_xx(1, 3) = Q_xu_(1, 0) * k_ux(0, 3) + Q_xx_(1, 3);
    V_xx(1, 4) = Q_xu_(1, 0) * k_ux(0, 4) + Q_xx_(1, 4);
    V_xx(2, 0) = Q_xu_(2, 0) * k_ux(0, 0) + Q_xx_(0, 2);
    V_xx(2, 1) = Q_xu_(2, 0) * k_ux(0, 1) + Q_xx_(1, 2);
    V_xx(2, 2) = Q_xu_(2, 0) * k_ux(0, 2) + Q_xx_(2, 2);
    V_xx(2, 3) = Q_xu_(2, 0) * k_ux(0, 3) + Q_xx_(2, 3);
    V_xx(2, 4) = Q_xu_(2, 0) * k_ux(0, 4) + Q_xx_(2, 4);
    V_xx(3, 0) = Q_xu_(3, 0) * k_ux(0, 0) + Q_xx_(0, 3);
    V_xx(3, 1) = Q_xu_(3, 0) * k_ux(0, 1) + Q_xx_(1, 3);
    V_xx(3, 2) = Q_xu_(3, 0) * k_ux(0, 2) + Q_xx_(2, 3);
    V_xx(3, 3) = Q_xu_(3, 0) * k_ux(0, 3) + Q_xx_(3, 3);
    V_xx(3, 4) = Q_xu_(3, 0) * k_ux(0, 4) + Q_xx_(3, 4);
    V_xx(4, 0) = Q_xu_(4, 0) * k_ux(0, 0) + Q_xx_(0, 4);
    V_xx(4, 1) = Q_xu_(4, 0) * k_ux(0, 1) + Q_xx_(1, 4);
    V_xx(4, 2) = Q_xu_(4, 0) * k_ux(0, 2) + Q_xx_(2, 4);
    V_xx(4, 3) = Q_xu_(4, 0) * k_ux(0, 3) + Q_xx_(3, 4);
    V_xx(4, 4) = Q_xu_(4, 0) * k_ux(0, 4) + Q_xx_(4, 4);

    // Evaluation of Vector V_x
    V_x(0) = Q_x_(0) + Q_xu_(0, 0) * v_u(0);
    V_x(1) = Q_x_(1) + Q_xu_(1, 0) * v_u(0);
    V_x(2) = Q_x_(2) + Q_xu_(2, 0) * v_u(0);
    V_x(3) = Q_x_(3) + Q_xu_(3, 0) * v_u(0);
    V_x(4) = Q_x_(4) + Q_xu_(4, 0) * v_u(0);
  }

  virtual void updateQnx(const Eigen::VectorXd& r_f, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& Fu,
                         const Eigen::MatrixXd& h_u, const Eigen::MatrixXd& k_ux, const Eigen::MatrixXd& k_ex,
                         const Eigen::MatrixXd& k_ue, const Eigen::VectorXd& v_u, const Eigen::VectorXd& v_e) override {
    // Evaluation of Matrix Q_un_
    Q_un_(0, 4) = -Fu(4, 0) * Q_uu_inv_(0, 0);

    // Evaluation of Matrix Q_nuu_
    Q_nuu_(0, 0) = -Q_uu_(0, 0);
    Q_nuu_(1, 1) = -Q_uu_(1, 1);

    // Evaluation of Matrix Q_vnx_
    Q_vnx_(0, 0) = Fx(0, 0);
    Q_vnx_(0, 1) = Fx(0, 1);
    Q_vnx_(0, 2) = Fx(0, 2);
    Q_vnx_(0, 3) = Fx(0, 3);
    Q_vnx_(1, 1) = Fx(1, 1);
    Q_vnx_(1, 3) = Fx(1, 3);
    Q_vnx_(2, 2) = Fx(2, 2);
    Q_vnx_(2, 3) = Fx(2, 3);
    Q_vnx_(3, 3) = Fx(3, 3);
    Q_vnx_(3, 4) = Fx(3, 4);
    Q_vnx_(4, 0) = Fu(4, 0) * k_ux(0, 0);
    Q_vnx_(4, 1) = Fu(4, 0) * k_ux(0, 1);
    Q_vnx_(4, 2) = Fu(4, 0) * k_ux(0, 2);
    Q_vnx_(4, 3) = Fu(4, 0) * k_ux(0, 3);
    Q_vnx_(4, 4) = Fu(4, 0) * k_ux(0, 4) + Fx(4, 4);

    // Evaluation of Vector Q_vn_
    Q_vn_(0) = r_f(0);
    Q_vn_(1) = r_f(1);
    Q_vn_(2) = r_f(2);
    Q_vn_(3) = r_f(3);
    Q_vn_(4) = Fu(4, 0) * v_u(0) + r_f(4);
  }
};

template <>
class ParkLateralGeneralIpmEvaluator<StageType::NORMINAL, OcpConfig::ERK4> : public IpmEvaluator {
 public:
  ParkLateralGeneralIpmEvaluator() : IpmEvaluator(5, 2, 17, 0, 0) {}
  virtual ~ParkLateralGeneralIpmEvaluator() = default;

  virtual void updateLX(const Eigen::VectorXd& l_x, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& g_x,
                        const Eigen::MatrixXd& h_x, const Eigen::MatrixXd& m_x, const Eigen::VectorXd& p,
                        const Eigen::VectorXd& lambda, const Eigen::VectorXd& eta, const Eigen::VectorXd& nu,
                        Eigen::VectorXd& sl_x) const override {
    // Evaluation of Vector sl_x
    sl_x(0) = Fx(0, 0) * p(0);
    sl_x(1) = Fx(0, 1) * p(0) + Fx(1, 1) * p(1) + g_x(0, 1) * lambda(0) + g_x(1, 1) * lambda(1) +
              g_x(11, 1) * lambda(11) + g_x(12, 1) * lambda(12) + g_x(13, 1) * lambda(13) + g_x(14, 1) * lambda(14) +
              g_x(15, 1) * lambda(15) + g_x(16, 1) * lambda(16) + g_x(2, 1) * lambda(2) + g_x(3, 1) * lambda(3) +
              g_x(4, 1) * lambda(4) + g_x(5, 1) * lambda(5) + l_x(1);
    sl_x(2) = Fx(0, 2) * p(0) + Fx(2, 2) * p(2) + g_x(0, 2) * lambda(0) + g_x(1, 2) * lambda(1) +
              g_x(11, 2) * lambda(11) + g_x(12, 2) * lambda(12) + g_x(13, 2) * lambda(13) + g_x(14, 2) * lambda(14) +
              g_x(15, 2) * lambda(15) + g_x(16, 2) * lambda(16) + g_x(2, 2) * lambda(2) + g_x(3, 2) * lambda(3) +
              g_x(4, 2) * lambda(4) + g_x(5, 2) * lambda(5) + l_x(2);
    sl_x(3) = Fx(0, 3) * p(0) + Fx(1, 3) * p(1) + Fx(2, 3) * p(2) + Fx(3, 3) * p(3) + g_x(13, 3) * lambda(13) +
              g_x(14, 3) * lambda(14) + g_x(15, 3) * lambda(15) + g_x(16, 3) * lambda(16) + g_x(2, 3) * lambda(2) +
              g_x(3, 3) * lambda(3) + g_x(4, 3) * lambda(4) + g_x(5, 3) * lambda(5) + g_x(8, 3) * lambda(8);
    sl_x(4) = Fx(0, 4) * p(0) + Fx(1, 4) * p(1) + Fx(2, 4) * p(2) + Fx(3, 4) * p(3) + Fx(4, 4) * p(4) +
              g_x(6, 4) * lambda(6) + g_x(7, 4) * lambda(7) + l_x(4);
  }

  virtual void updateLU(const Eigen::VectorXd& l_u, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& g_u,
                        const Eigen::MatrixXd& h_u, const Eigen::VectorXd& p, const Eigen::VectorXd& lambda,
                        const Eigen::VectorXd& eta, Eigen::VectorXd& sl_u) const override {
    // Evaluation of Vector sl_u
    sl_u(0) = Fu(0, 0) * p(0) + Fu(1, 0) * p(1) + Fu(2, 0) * p(2) + Fu(3, 0) * p(3) + Fu(4, 0) * p(4) +
              g_u(10, 0) * lambda(10) + g_u(9, 0) * lambda(9) + l_u(0);
    sl_u(1) = g_u(11, 1) * lambda(11) + g_u(12, 1) * lambda(12) + g_u(13, 1) * lambda(13) + g_u(14, 1) * lambda(14) +
              g_u(15, 1) * lambda(15) + g_u(16, 1) * lambda(16) + l_u(1);
  }

  virtual void updateQH(const Eigen::MatrixXd& l_xx, const Eigen::MatrixXd& l_xu, const Eigen::MatrixXd& l_uu,
                        const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::MatrixXd& Fx,
                        const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& V_xx_prev) override {
    // Determine internal variables
    const double Fx_0_0_p2 = Fx(0, 0) * Fx(0, 0);
    const double g_u_10_0_p2 = g_u(10, 0) * g_u(10, 0);
    const double g_u_11_1_p2 = g_u(11, 1) * g_u(11, 1);
    const double g_u_12_1_p2 = g_u(12, 1) * g_u(12, 1);
    const double g_u_13_1_p2 = g_u(13, 1) * g_u(13, 1);
    const double g_u_14_1_p2 = g_u(14, 1) * g_u(14, 1);
    const double g_u_15_1_p2 = g_u(15, 1) * g_u(15, 1);
    const double g_u_16_1_p2 = g_u(16, 1) * g_u(16, 1);
    const double g_u_9_0_p2 = g_u(9, 0) * g_u(9, 0);
    const double g_x_0_1_p2 = g_x(0, 1) * g_x(0, 1);
    const double g_x_0_2_p2 = g_x(0, 2) * g_x(0, 2);
    const double g_x_11_1_p2 = g_x(11, 1) * g_x(11, 1);
    const double g_x_11_2_p2 = g_x(11, 2) * g_x(11, 2);
    const double g_x_12_1_p2 = g_x(12, 1) * g_x(12, 1);
    const double g_x_12_2_p2 = g_x(12, 2) * g_x(12, 2);
    const double g_x_13_1_p2 = g_x(13, 1) * g_x(13, 1);
    const double g_x_13_2_p2 = g_x(13, 2) * g_x(13, 2);
    const double g_x_13_3_p2 = g_x(13, 3) * g_x(13, 3);
    const double g_x_14_1_p2 = g_x(14, 1) * g_x(14, 1);
    const double g_x_14_2_p2 = g_x(14, 2) * g_x(14, 2);
    const double g_x_14_3_p2 = g_x(14, 3) * g_x(14, 3);
    const double g_x_15_1_p2 = g_x(15, 1) * g_x(15, 1);
    const double g_x_15_2_p2 = g_x(15, 2) * g_x(15, 2);
    const double g_x_15_3_p2 = g_x(15, 3) * g_x(15, 3);
    const double g_x_16_1_p2 = g_x(16, 1) * g_x(16, 1);
    const double g_x_16_2_p2 = g_x(16, 2) * g_x(16, 2);
    const double g_x_16_3_p2 = g_x(16, 3) * g_x(16, 3);
    const double g_x_1_1_p2 = g_x(1, 1) * g_x(1, 1);
    const double g_x_1_2_p2 = g_x(1, 2) * g_x(1, 2);
    const double g_x_2_1_p2 = g_x(2, 1) * g_x(2, 1);
    const double g_x_2_2_p2 = g_x(2, 2) * g_x(2, 2);
    const double g_x_2_3_p2 = g_x(2, 3) * g_x(2, 3);
    const double g_x_3_1_p2 = g_x(3, 1) * g_x(3, 1);
    const double g_x_3_2_p2 = g_x(3, 2) * g_x(3, 2);
    const double g_x_3_3_p2 = g_x(3, 3) * g_x(3, 3);
    const double g_x_4_1_p2 = g_x(4, 1) * g_x(4, 1);
    const double g_x_4_2_p2 = g_x(4, 2) * g_x(4, 2);
    const double g_x_4_3_p2 = g_x(4, 3) * g_x(4, 3);
    const double g_x_5_1_p2 = g_x(5, 1) * g_x(5, 1);
    const double g_x_5_2_p2 = g_x(5, 2) * g_x(5, 2);
    const double g_x_5_3_p2 = g_x(5, 3) * g_x(5, 3);
    const double g_x_6_4_p2 = g_x(6, 4) * g_x(6, 4);
    const double g_x_7_4_p2 = g_x(7, 4) * g_x(7, 4);
    const double g_x_8_3_p2 = g_x(8, 3) * g_x(8, 3);
    const double internal_0 = Sigma_(0) * g_x(0, 1) * g_x(0, 2);
    const double internal_1 = Sigma_(1) * g_x(1, 1) * g_x(1, 2);
    const double internal_10 = Sigma_(4) * g_x(4, 1) * g_x(4, 2);
    const double internal_11 = Sigma_(5) * g_x(5, 1) * g_x(5, 2);
    const double internal_12 = Sigma_(13) * g_x(13, 1) * g_x(13, 3);
    const double internal_13 = Sigma_(14) * g_x(14, 1) * g_x(14, 3);
    const double internal_14 = Sigma_(15) * g_x(15, 1) * g_x(15, 3);
    const double internal_15 = Sigma_(16) * g_x(16, 1) * g_x(16, 3);
    const double internal_16 = Sigma_(2) * g_x(2, 1) * g_x(2, 3);
    const double internal_17 = Sigma_(3) * g_x(3, 1) * g_x(3, 3);
    const double internal_18 = Sigma_(4) * g_x(4, 1) * g_x(4, 3);
    const double internal_19 = Sigma_(5) * g_x(5, 1) * g_x(5, 3);
    const double internal_2 = Sigma_(11) * g_x(11, 1) * g_x(11, 2);
    const double internal_20 = Sigma_(13) * g_x(13, 2) * g_x(13, 3);
    const double internal_21 = Sigma_(14) * g_x(14, 2) * g_x(14, 3);
    const double internal_22 = Sigma_(15) * g_x(15, 2) * g_x(15, 3);
    const double internal_23 = Sigma_(16) * g_x(16, 2) * g_x(16, 3);
    const double internal_24 = Sigma_(2) * g_x(2, 2) * g_x(2, 3);
    const double internal_25 = Sigma_(3) * g_x(3, 2) * g_x(3, 3);
    const double internal_26 = Sigma_(4) * g_x(4, 2) * g_x(4, 3);
    const double internal_27 = Sigma_(5) * g_x(5, 2) * g_x(5, 3);
    const double internal_3 = Sigma_(12) * g_x(12, 1) * g_x(12, 2);
    const double internal_4 = Sigma_(13) * g_x(13, 1) * g_x(13, 2);
    const double internal_5 = Sigma_(14) * g_x(14, 1) * g_x(14, 2);
    const double internal_6 = Sigma_(15) * g_x(15, 1) * g_x(15, 2);
    const double internal_7 = Sigma_(16) * g_x(16, 1) * g_x(16, 2);
    const double internal_8 = Sigma_(2) * g_x(2, 1) * g_x(2, 2);
    const double internal_9 = Sigma_(3) * g_x(3, 1) * g_x(3, 2);

    // Evaluation of Matrix Q_xx_
    Q_xx_(0, 0) = Fx_0_0_p2 * V_xx_prev(0, 0);
    Q_xx_(0, 1) = Fx(0, 0) * Fx(0, 1) * V_xx_prev(0, 0) + Fx(0, 0) * Fx(1, 1) * V_xx_prev(0, 1);
    Q_xx_(0, 2) = Fx(0, 0) * Fx(0, 2) * V_xx_prev(0, 0) + Fx(0, 0) * Fx(2, 2) * V_xx_prev(0, 2);
    Q_xx_(0, 3) = Fx(0, 0) * Fx(0, 3) * V_xx_prev(0, 0) + Fx(0, 0) * Fx(1, 3) * V_xx_prev(0, 1) +
                  Fx(0, 0) * Fx(2, 3) * V_xx_prev(0, 2) + Fx(0, 0) * Fx(3, 3) * V_xx_prev(0, 3);
    Q_xx_(0, 4) = Fx(0, 0) * Fx(0, 4) * V_xx_prev(0, 0) + Fx(0, 0) * Fx(1, 4) * V_xx_prev(0, 1) +
                  Fx(0, 0) * Fx(2, 4) * V_xx_prev(0, 2) + Fx(0, 0) * Fx(3, 4) * V_xx_prev(0, 3) +
                  Fx(0, 0) * Fx(4, 4) * V_xx_prev(0, 4);
    Q_xx_(1, 0) = Fx(0, 0) * (Fx(0, 1) * V_xx_prev(0, 0) + Fx(1, 1) * V_xx_prev(1, 0));
    Q_xx_(1, 1) = Fx(0, 1) * (Fx(0, 1) * V_xx_prev(0, 0) + Fx(1, 1) * V_xx_prev(1, 0)) +
                  Fx(1, 1) * (Fx(0, 1) * V_xx_prev(0, 1) + Fx(1, 1) * V_xx_prev(1, 1)) + Sigma_(0) * g_x_0_1_p2 +
                  Sigma_(1) * g_x_1_1_p2 + Sigma_(11) * g_x_11_1_p2 + Sigma_(12) * g_x_12_1_p2 +
                  Sigma_(13) * g_x_13_1_p2 + Sigma_(14) * g_x_14_1_p2 + Sigma_(15) * g_x_15_1_p2 +
                  Sigma_(16) * g_x_16_1_p2 + Sigma_(2) * g_x_2_1_p2 + Sigma_(3) * g_x_3_1_p2 + Sigma_(4) * g_x_4_1_p2 +
                  Sigma_(5) * g_x_5_1_p2 + l_xx(1, 1);
    Q_xx_(1, 2) = Fx(0, 2) * (Fx(0, 1) * V_xx_prev(0, 0) + Fx(1, 1) * V_xx_prev(1, 0)) +
                  Fx(2, 2) * (Fx(0, 1) * V_xx_prev(0, 2) + Fx(1, 1) * V_xx_prev(1, 2)) + internal_0 + internal_1 +
                  internal_10 + internal_11 + internal_2 + internal_3 + internal_4 + internal_5 + internal_6 +
                  internal_7 + internal_8 + internal_9 + l_xx(1, 2);
    Q_xx_(1, 3) = Fx(0, 3) * (Fx(0, 1) * V_xx_prev(0, 0) + Fx(1, 1) * V_xx_prev(1, 0)) +
                  Fx(1, 3) * (Fx(0, 1) * V_xx_prev(0, 1) + Fx(1, 1) * V_xx_prev(1, 1)) +
                  Fx(2, 3) * (Fx(0, 1) * V_xx_prev(0, 2) + Fx(1, 1) * V_xx_prev(1, 2)) +
                  Fx(3, 3) * (Fx(0, 1) * V_xx_prev(0, 3) + Fx(1, 1) * V_xx_prev(1, 3)) + internal_12 + internal_13 +
                  internal_14 + internal_15 + internal_16 + internal_17 + internal_18 + internal_19;
    Q_xx_(1, 4) = Fx(0, 4) * (Fx(0, 1) * V_xx_prev(0, 0) + Fx(1, 1) * V_xx_prev(1, 0)) +
                  Fx(1, 4) * (Fx(0, 1) * V_xx_prev(0, 1) + Fx(1, 1) * V_xx_prev(1, 1)) +
                  Fx(2, 4) * (Fx(0, 1) * V_xx_prev(0, 2) + Fx(1, 1) * V_xx_prev(1, 2)) +
                  Fx(3, 4) * (Fx(0, 1) * V_xx_prev(0, 3) + Fx(1, 1) * V_xx_prev(1, 3)) +
                  Fx(4, 4) * (Fx(0, 1) * V_xx_prev(0, 4) + Fx(1, 1) * V_xx_prev(1, 4));
    Q_xx_(2, 0) = Fx(0, 0) * (Fx(0, 2) * V_xx_prev(0, 0) + Fx(2, 2) * V_xx_prev(2, 0));
    Q_xx_(2, 1) = Fx(0, 1) * (Fx(0, 2) * V_xx_prev(0, 0) + Fx(2, 2) * V_xx_prev(2, 0)) +
                  Fx(1, 1) * (Fx(0, 2) * V_xx_prev(0, 1) + Fx(2, 2) * V_xx_prev(2, 1)) + internal_0 + internal_1 +
                  internal_10 + internal_11 + internal_2 + internal_3 + internal_4 + internal_5 + internal_6 +
                  internal_7 + internal_8 + internal_9 + l_xx(2, 1);
    Q_xx_(2, 2) = Fx(0, 2) * (Fx(0, 2) * V_xx_prev(0, 0) + Fx(2, 2) * V_xx_prev(2, 0)) +
                  Fx(2, 2) * (Fx(0, 2) * V_xx_prev(0, 2) + Fx(2, 2) * V_xx_prev(2, 2)) + Sigma_(0) * g_x_0_2_p2 +
                  Sigma_(1) * g_x_1_2_p2 + Sigma_(11) * g_x_11_2_p2 + Sigma_(12) * g_x_12_2_p2 +
                  Sigma_(13) * g_x_13_2_p2 + Sigma_(14) * g_x_14_2_p2 + Sigma_(15) * g_x_15_2_p2 +
                  Sigma_(16) * g_x_16_2_p2 + Sigma_(2) * g_x_2_2_p2 + Sigma_(3) * g_x_3_2_p2 + Sigma_(4) * g_x_4_2_p2 +
                  Sigma_(5) * g_x_5_2_p2 + l_xx(2, 2);
    Q_xx_(2, 3) = Fx(0, 3) * (Fx(0, 2) * V_xx_prev(0, 0) + Fx(2, 2) * V_xx_prev(2, 0)) +
                  Fx(1, 3) * (Fx(0, 2) * V_xx_prev(0, 1) + Fx(2, 2) * V_xx_prev(2, 1)) +
                  Fx(2, 3) * (Fx(0, 2) * V_xx_prev(0, 2) + Fx(2, 2) * V_xx_prev(2, 2)) +
                  Fx(3, 3) * (Fx(0, 2) * V_xx_prev(0, 3) + Fx(2, 2) * V_xx_prev(2, 3)) + internal_20 + internal_21 +
                  internal_22 + internal_23 + internal_24 + internal_25 + internal_26 + internal_27;
    Q_xx_(2, 4) = Fx(0, 4) * (Fx(0, 2) * V_xx_prev(0, 0) + Fx(2, 2) * V_xx_prev(2, 0)) +
                  Fx(1, 4) * (Fx(0, 2) * V_xx_prev(0, 1) + Fx(2, 2) * V_xx_prev(2, 1)) +
                  Fx(2, 4) * (Fx(0, 2) * V_xx_prev(0, 2) + Fx(2, 2) * V_xx_prev(2, 2)) +
                  Fx(3, 4) * (Fx(0, 2) * V_xx_prev(0, 3) + Fx(2, 2) * V_xx_prev(2, 3)) +
                  Fx(4, 4) * (Fx(0, 2) * V_xx_prev(0, 4) + Fx(2, 2) * V_xx_prev(2, 4));
    Q_xx_(3, 0) = Fx(0, 0) * (Fx(0, 3) * V_xx_prev(0, 0) + Fx(1, 3) * V_xx_prev(1, 0) + Fx(2, 3) * V_xx_prev(2, 0) +
                              Fx(3, 3) * V_xx_prev(3, 0));
    Q_xx_(3, 1) = Fx(0, 1) * (Fx(0, 3) * V_xx_prev(0, 0) + Fx(1, 3) * V_xx_prev(1, 0) + Fx(2, 3) * V_xx_prev(2, 0) +
                              Fx(3, 3) * V_xx_prev(3, 0)) +
                  Fx(1, 1) * (Fx(0, 3) * V_xx_prev(0, 1) + Fx(1, 3) * V_xx_prev(1, 1) + Fx(2, 3) * V_xx_prev(2, 1) +
                              Fx(3, 3) * V_xx_prev(3, 1)) +
                  internal_12 + internal_13 + internal_14 + internal_15 + internal_16 + internal_17 + internal_18 +
                  internal_19;
    Q_xx_(3, 2) = Fx(0, 2) * (Fx(0, 3) * V_xx_prev(0, 0) + Fx(1, 3) * V_xx_prev(1, 0) + Fx(2, 3) * V_xx_prev(2, 0) +
                              Fx(3, 3) * V_xx_prev(3, 0)) +
                  Fx(2, 2) * (Fx(0, 3) * V_xx_prev(0, 2) + Fx(1, 3) * V_xx_prev(1, 2) + Fx(2, 3) * V_xx_prev(2, 2) +
                              Fx(3, 3) * V_xx_prev(3, 2)) +
                  internal_20 + internal_21 + internal_22 + internal_23 + internal_24 + internal_25 + internal_26 +
                  internal_27;
    Q_xx_(3, 3) = Fx(0, 3) * (Fx(0, 3) * V_xx_prev(0, 0) + Fx(1, 3) * V_xx_prev(1, 0) + Fx(2, 3) * V_xx_prev(2, 0) +
                              Fx(3, 3) * V_xx_prev(3, 0)) +
                  Fx(1, 3) * (Fx(0, 3) * V_xx_prev(0, 1) + Fx(1, 3) * V_xx_prev(1, 1) + Fx(2, 3) * V_xx_prev(2, 1) +
                              Fx(3, 3) * V_xx_prev(3, 1)) +
                  Fx(2, 3) * (Fx(0, 3) * V_xx_prev(0, 2) + Fx(1, 3) * V_xx_prev(1, 2) + Fx(2, 3) * V_xx_prev(2, 2) +
                              Fx(3, 3) * V_xx_prev(3, 2)) +
                  Fx(3, 3) * (Fx(0, 3) * V_xx_prev(0, 3) + Fx(1, 3) * V_xx_prev(1, 3) + Fx(2, 3) * V_xx_prev(2, 3) +
                              Fx(3, 3) * V_xx_prev(3, 3)) +
                  Sigma_(13) * g_x_13_3_p2 + Sigma_(14) * g_x_14_3_p2 + Sigma_(15) * g_x_15_3_p2 +
                  Sigma_(16) * g_x_16_3_p2 + Sigma_(2) * g_x_2_3_p2 + Sigma_(3) * g_x_3_3_p2 + Sigma_(4) * g_x_4_3_p2 +
                  Sigma_(5) * g_x_5_3_p2 + Sigma_(8) * g_x_8_3_p2;
    Q_xx_(3, 4) = Fx(0, 4) * (Fx(0, 3) * V_xx_prev(0, 0) + Fx(1, 3) * V_xx_prev(1, 0) + Fx(2, 3) * V_xx_prev(2, 0) +
                              Fx(3, 3) * V_xx_prev(3, 0)) +
                  Fx(1, 4) * (Fx(0, 3) * V_xx_prev(0, 1) + Fx(1, 3) * V_xx_prev(1, 1) + Fx(2, 3) * V_xx_prev(2, 1) +
                              Fx(3, 3) * V_xx_prev(3, 1)) +
                  Fx(2, 4) * (Fx(0, 3) * V_xx_prev(0, 2) + Fx(1, 3) * V_xx_prev(1, 2) + Fx(2, 3) * V_xx_prev(2, 2) +
                              Fx(3, 3) * V_xx_prev(3, 2)) +
                  Fx(3, 4) * (Fx(0, 3) * V_xx_prev(0, 3) + Fx(1, 3) * V_xx_prev(1, 3) + Fx(2, 3) * V_xx_prev(2, 3) +
                              Fx(3, 3) * V_xx_prev(3, 3)) +
                  Fx(4, 4) * (Fx(0, 3) * V_xx_prev(0, 4) + Fx(1, 3) * V_xx_prev(1, 4) + Fx(2, 3) * V_xx_prev(2, 4) +
                              Fx(3, 3) * V_xx_prev(3, 4));
    Q_xx_(4, 0) = Fx(0, 0) * (Fx(0, 4) * V_xx_prev(0, 0) + Fx(1, 4) * V_xx_prev(1, 0) + Fx(2, 4) * V_xx_prev(2, 0) +
                              Fx(3, 4) * V_xx_prev(3, 0) + Fx(4, 4) * V_xx_prev(4, 0));
    Q_xx_(4, 1) = Fx(0, 1) * (Fx(0, 4) * V_xx_prev(0, 0) + Fx(1, 4) * V_xx_prev(1, 0) + Fx(2, 4) * V_xx_prev(2, 0) +
                              Fx(3, 4) * V_xx_prev(3, 0) + Fx(4, 4) * V_xx_prev(4, 0)) +
                  Fx(1, 1) * (Fx(0, 4) * V_xx_prev(0, 1) + Fx(1, 4) * V_xx_prev(1, 1) + Fx(2, 4) * V_xx_prev(2, 1) +
                              Fx(3, 4) * V_xx_prev(3, 1) + Fx(4, 4) * V_xx_prev(4, 1));
    Q_xx_(4, 2) = Fx(0, 2) * (Fx(0, 4) * V_xx_prev(0, 0) + Fx(1, 4) * V_xx_prev(1, 0) + Fx(2, 4) * V_xx_prev(2, 0) +
                              Fx(3, 4) * V_xx_prev(3, 0) + Fx(4, 4) * V_xx_prev(4, 0)) +
                  Fx(2, 2) * (Fx(0, 4) * V_xx_prev(0, 2) + Fx(1, 4) * V_xx_prev(1, 2) + Fx(2, 4) * V_xx_prev(2, 2) +
                              Fx(3, 4) * V_xx_prev(3, 2) + Fx(4, 4) * V_xx_prev(4, 2));
    Q_xx_(4, 3) = Fx(0, 3) * (Fx(0, 4) * V_xx_prev(0, 0) + Fx(1, 4) * V_xx_prev(1, 0) + Fx(2, 4) * V_xx_prev(2, 0) +
                              Fx(3, 4) * V_xx_prev(3, 0) + Fx(4, 4) * V_xx_prev(4, 0)) +
                  Fx(1, 3) * (Fx(0, 4) * V_xx_prev(0, 1) + Fx(1, 4) * V_xx_prev(1, 1) + Fx(2, 4) * V_xx_prev(2, 1) +
                              Fx(3, 4) * V_xx_prev(3, 1) + Fx(4, 4) * V_xx_prev(4, 1)) +
                  Fx(2, 3) * (Fx(0, 4) * V_xx_prev(0, 2) + Fx(1, 4) * V_xx_prev(1, 2) + Fx(2, 4) * V_xx_prev(2, 2) +
                              Fx(3, 4) * V_xx_prev(3, 2) + Fx(4, 4) * V_xx_prev(4, 2)) +
                  Fx(3, 3) * (Fx(0, 4) * V_xx_prev(0, 3) + Fx(1, 4) * V_xx_prev(1, 3) + Fx(2, 4) * V_xx_prev(2, 3) +
                              Fx(3, 4) * V_xx_prev(3, 3) + Fx(4, 4) * V_xx_prev(4, 3));
    Q_xx_(4, 4) = Fx(0, 4) * (Fx(0, 4) * V_xx_prev(0, 0) + Fx(1, 4) * V_xx_prev(1, 0) + Fx(2, 4) * V_xx_prev(2, 0) +
                              Fx(3, 4) * V_xx_prev(3, 0) + Fx(4, 4) * V_xx_prev(4, 0)) +
                  Fx(1, 4) * (Fx(0, 4) * V_xx_prev(0, 1) + Fx(1, 4) * V_xx_prev(1, 1) + Fx(2, 4) * V_xx_prev(2, 1) +
                              Fx(3, 4) * V_xx_prev(3, 1) + Fx(4, 4) * V_xx_prev(4, 1)) +
                  Fx(2, 4) * (Fx(0, 4) * V_xx_prev(0, 2) + Fx(1, 4) * V_xx_prev(1, 2) + Fx(2, 4) * V_xx_prev(2, 2) +
                              Fx(3, 4) * V_xx_prev(3, 2) + Fx(4, 4) * V_xx_prev(4, 2)) +
                  Fx(3, 4) * (Fx(0, 4) * V_xx_prev(0, 3) + Fx(1, 4) * V_xx_prev(1, 3) + Fx(2, 4) * V_xx_prev(2, 3) +
                              Fx(3, 4) * V_xx_prev(3, 3) + Fx(4, 4) * V_xx_prev(4, 3)) +
                  Fx(4, 4) * (Fx(0, 4) * V_xx_prev(0, 4) + Fx(1, 4) * V_xx_prev(1, 4) + Fx(2, 4) * V_xx_prev(2, 4) +
                              Fx(3, 4) * V_xx_prev(3, 4) + Fx(4, 4) * V_xx_prev(4, 4)) +
                  Sigma_(6) * g_x_6_4_p2 + Sigma_(7) * g_x_7_4_p2 + l_xx(4, 4);

    // Evaluation of Matrix Q_xu_
    Q_xu_(0, 0) = Fu(0, 0) * Fx(0, 0) * V_xx_prev(0, 0) + Fu(1, 0) * Fx(0, 0) * V_xx_prev(0, 1) +
                  Fu(2, 0) * Fx(0, 0) * V_xx_prev(0, 2) + Fu(3, 0) * Fx(0, 0) * V_xx_prev(0, 3) +
                  Fu(4, 0) * Fx(0, 0) * V_xx_prev(0, 4);
    Q_xu_(1, 0) = Fu(0, 0) * (Fx(0, 1) * V_xx_prev(0, 0) + Fx(1, 1) * V_xx_prev(1, 0)) +
                  Fu(1, 0) * (Fx(0, 1) * V_xx_prev(0, 1) + Fx(1, 1) * V_xx_prev(1, 1)) +
                  Fu(2, 0) * (Fx(0, 1) * V_xx_prev(0, 2) + Fx(1, 1) * V_xx_prev(1, 2)) +
                  Fu(3, 0) * (Fx(0, 1) * V_xx_prev(0, 3) + Fx(1, 1) * V_xx_prev(1, 3)) +
                  Fu(4, 0) * (Fx(0, 1) * V_xx_prev(0, 4) + Fx(1, 1) * V_xx_prev(1, 4));
    Q_xu_(1, 1) = Sigma_(11) * g_u(11, 1) * g_x(11, 1) + Sigma_(12) * g_u(12, 1) * g_x(12, 1) +
                  Sigma_(13) * g_u(13, 1) * g_x(13, 1) + Sigma_(14) * g_u(14, 1) * g_x(14, 1) +
                  Sigma_(15) * g_u(15, 1) * g_x(15, 1) + Sigma_(16) * g_u(16, 1) * g_x(16, 1);
    Q_xu_(2, 0) = Fu(0, 0) * (Fx(0, 2) * V_xx_prev(0, 0) + Fx(2, 2) * V_xx_prev(2, 0)) +
                  Fu(1, 0) * (Fx(0, 2) * V_xx_prev(0, 1) + Fx(2, 2) * V_xx_prev(2, 1)) +
                  Fu(2, 0) * (Fx(0, 2) * V_xx_prev(0, 2) + Fx(2, 2) * V_xx_prev(2, 2)) +
                  Fu(3, 0) * (Fx(0, 2) * V_xx_prev(0, 3) + Fx(2, 2) * V_xx_prev(2, 3)) +
                  Fu(4, 0) * (Fx(0, 2) * V_xx_prev(0, 4) + Fx(2, 2) * V_xx_prev(2, 4));
    Q_xu_(2, 1) = Sigma_(11) * g_u(11, 1) * g_x(11, 2) + Sigma_(12) * g_u(12, 1) * g_x(12, 2) +
                  Sigma_(13) * g_u(13, 1) * g_x(13, 2) + Sigma_(14) * g_u(14, 1) * g_x(14, 2) +
                  Sigma_(15) * g_u(15, 1) * g_x(15, 2) + Sigma_(16) * g_u(16, 1) * g_x(16, 2);
    Q_xu_(3, 0) = Fu(0, 0) * (Fx(0, 3) * V_xx_prev(0, 0) + Fx(1, 3) * V_xx_prev(1, 0) + Fx(2, 3) * V_xx_prev(2, 0) +
                              Fx(3, 3) * V_xx_prev(3, 0)) +
                  Fu(1, 0) * (Fx(0, 3) * V_xx_prev(0, 1) + Fx(1, 3) * V_xx_prev(1, 1) + Fx(2, 3) * V_xx_prev(2, 1) +
                              Fx(3, 3) * V_xx_prev(3, 1)) +
                  Fu(2, 0) * (Fx(0, 3) * V_xx_prev(0, 2) + Fx(1, 3) * V_xx_prev(1, 2) + Fx(2, 3) * V_xx_prev(2, 2) +
                              Fx(3, 3) * V_xx_prev(3, 2)) +
                  Fu(3, 0) * (Fx(0, 3) * V_xx_prev(0, 3) + Fx(1, 3) * V_xx_prev(1, 3) + Fx(2, 3) * V_xx_prev(2, 3) +
                              Fx(3, 3) * V_xx_prev(3, 3)) +
                  Fu(4, 0) * (Fx(0, 3) * V_xx_prev(0, 4) + Fx(1, 3) * V_xx_prev(1, 4) + Fx(2, 3) * V_xx_prev(2, 4) +
                              Fx(3, 3) * V_xx_prev(3, 4));
    Q_xu_(3, 1) = Sigma_(13) * g_u(13, 1) * g_x(13, 3) + Sigma_(14) * g_u(14, 1) * g_x(14, 3) +
                  Sigma_(15) * g_u(15, 1) * g_x(15, 3) + Sigma_(16) * g_u(16, 1) * g_x(16, 3);
    Q_xu_(4, 0) = Fu(0, 0) * (Fx(0, 4) * V_xx_prev(0, 0) + Fx(1, 4) * V_xx_prev(1, 0) + Fx(2, 4) * V_xx_prev(2, 0) +
                              Fx(3, 4) * V_xx_prev(3, 0) + Fx(4, 4) * V_xx_prev(4, 0)) +
                  Fu(1, 0) * (Fx(0, 4) * V_xx_prev(0, 1) + Fx(1, 4) * V_xx_prev(1, 1) + Fx(2, 4) * V_xx_prev(2, 1) +
                              Fx(3, 4) * V_xx_prev(3, 1) + Fx(4, 4) * V_xx_prev(4, 1)) +
                  Fu(2, 0) * (Fx(0, 4) * V_xx_prev(0, 2) + Fx(1, 4) * V_xx_prev(1, 2) + Fx(2, 4) * V_xx_prev(2, 2) +
                              Fx(3, 4) * V_xx_prev(3, 2) + Fx(4, 4) * V_xx_prev(4, 2)) +
                  Fu(3, 0) * (Fx(0, 4) * V_xx_prev(0, 3) + Fx(1, 4) * V_xx_prev(1, 3) + Fx(2, 4) * V_xx_prev(2, 3) +
                              Fx(3, 4) * V_xx_prev(3, 3) + Fx(4, 4) * V_xx_prev(4, 3)) +
                  Fu(4, 0) * (Fx(0, 4) * V_xx_prev(0, 4) + Fx(1, 4) * V_xx_prev(1, 4) + Fx(2, 4) * V_xx_prev(2, 4) +
                              Fx(3, 4) * V_xx_prev(3, 4) + Fx(4, 4) * V_xx_prev(4, 4));

    // Evaluation of Matrix Q_uu_
    Q_uu_(0, 0) = Fu(0, 0) * (Fu(0, 0) * V_xx_prev(0, 0) + Fu(1, 0) * V_xx_prev(1, 0) + Fu(2, 0) * V_xx_prev(2, 0) +
                              Fu(3, 0) * V_xx_prev(3, 0) + Fu(4, 0) * V_xx_prev(4, 0)) +
                  Fu(1, 0) * (Fu(0, 0) * V_xx_prev(0, 1) + Fu(1, 0) * V_xx_prev(1, 1) + Fu(2, 0) * V_xx_prev(2, 1) +
                              Fu(3, 0) * V_xx_prev(3, 1) + Fu(4, 0) * V_xx_prev(4, 1)) +
                  Fu(2, 0) * (Fu(0, 0) * V_xx_prev(0, 2) + Fu(1, 0) * V_xx_prev(1, 2) + Fu(2, 0) * V_xx_prev(2, 2) +
                              Fu(3, 0) * V_xx_prev(3, 2) + Fu(4, 0) * V_xx_prev(4, 2)) +
                  Fu(3, 0) * (Fu(0, 0) * V_xx_prev(0, 3) + Fu(1, 0) * V_xx_prev(1, 3) + Fu(2, 0) * V_xx_prev(2, 3) +
                              Fu(3, 0) * V_xx_prev(3, 3) + Fu(4, 0) * V_xx_prev(4, 3)) +
                  Fu(4, 0) * (Fu(0, 0) * V_xx_prev(0, 4) + Fu(1, 0) * V_xx_prev(1, 4) + Fu(2, 0) * V_xx_prev(2, 4) +
                              Fu(3, 0) * V_xx_prev(3, 4) + Fu(4, 0) * V_xx_prev(4, 4)) +
                  Sigma_(10) * g_u_10_0_p2 + Sigma_(9) * g_u_9_0_p2 + l_uu(0, 0);
    Q_uu_(1, 1) = Sigma_(11) * g_u_11_1_p2 + Sigma_(12) * g_u_12_1_p2 + Sigma_(13) * g_u_13_1_p2 +
                  Sigma_(14) * g_u_14_1_p2 + Sigma_(15) * g_u_15_1_p2 + Sigma_(16) * g_u_16_1_p2 + l_uu(1, 1);
  }

  virtual void updateQG(const Eigen::VectorXd& sl_x, const Eigen::VectorXd& sl_u, const Eigen::MatrixXd& g_x,
                        const Eigen::MatrixXd& g_u, const Eigen::VectorXd& r_f, const Eigen::MatrixXd& Fx,
                        const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& V_xx_prev,
                        const Eigen::VectorXd& V_x_prev) override {
    // Evaluation of Vector Q_x_
    Q_x_(0) = Fx(0, 0) * (V_x_prev(0) + V_xx_prev(0, 0) * r_f(0) + V_xx_prev(0, 1) * r_f(1) + V_xx_prev(0, 2) * r_f(2) +
                          V_xx_prev(0, 3) * r_f(3) + V_xx_prev(0, 4) * r_f(4)) +
              sl_x(0);
    Q_x_(1) = Fx(0, 1) * (V_x_prev(0) + V_xx_prev(0, 0) * r_f(0) + V_xx_prev(0, 1) * r_f(1) + V_xx_prev(0, 2) * r_f(2) +
                          V_xx_prev(0, 3) * r_f(3) + V_xx_prev(0, 4) * r_f(4)) +
              Fx(1, 1) * (V_x_prev(1) + V_xx_prev(1, 0) * r_f(0) + V_xx_prev(1, 1) * r_f(1) + V_xx_prev(1, 2) * r_f(2) +
                          V_xx_prev(1, 3) * r_f(3) + V_xx_prev(1, 4) * r_f(4)) +
              V_(0) * g_x(0, 1) + V_(1) * g_x(1, 1) + V_(11) * g_x(11, 1) + V_(12) * g_x(12, 1) + V_(13) * g_x(13, 1) +
              V_(14) * g_x(14, 1) + V_(15) * g_x(15, 1) + V_(16) * g_x(16, 1) + V_(2) * g_x(2, 1) + V_(3) * g_x(3, 1) +
              V_(4) * g_x(4, 1) + V_(5) * g_x(5, 1) + sl_x(1);
    Q_x_(2) = Fx(0, 2) * (V_x_prev(0) + V_xx_prev(0, 0) * r_f(0) + V_xx_prev(0, 1) * r_f(1) + V_xx_prev(0, 2) * r_f(2) +
                          V_xx_prev(0, 3) * r_f(3) + V_xx_prev(0, 4) * r_f(4)) +
              Fx(2, 2) * (V_x_prev(2) + V_xx_prev(2, 0) * r_f(0) + V_xx_prev(2, 1) * r_f(1) + V_xx_prev(2, 2) * r_f(2) +
                          V_xx_prev(2, 3) * r_f(3) + V_xx_prev(2, 4) * r_f(4)) +
              V_(0) * g_x(0, 2) + V_(1) * g_x(1, 2) + V_(11) * g_x(11, 2) + V_(12) * g_x(12, 2) + V_(13) * g_x(13, 2) +
              V_(14) * g_x(14, 2) + V_(15) * g_x(15, 2) + V_(16) * g_x(16, 2) + V_(2) * g_x(2, 2) + V_(3) * g_x(3, 2) +
              V_(4) * g_x(4, 2) + V_(5) * g_x(5, 2) + sl_x(2);
    Q_x_(3) = Fx(0, 3) * (V_x_prev(0) + V_xx_prev(0, 0) * r_f(0) + V_xx_prev(0, 1) * r_f(1) + V_xx_prev(0, 2) * r_f(2) +
                          V_xx_prev(0, 3) * r_f(3) + V_xx_prev(0, 4) * r_f(4)) +
              Fx(1, 3) * (V_x_prev(1) + V_xx_prev(1, 0) * r_f(0) + V_xx_prev(1, 1) * r_f(1) + V_xx_prev(1, 2) * r_f(2) +
                          V_xx_prev(1, 3) * r_f(3) + V_xx_prev(1, 4) * r_f(4)) +
              Fx(2, 3) * (V_x_prev(2) + V_xx_prev(2, 0) * r_f(0) + V_xx_prev(2, 1) * r_f(1) + V_xx_prev(2, 2) * r_f(2) +
                          V_xx_prev(2, 3) * r_f(3) + V_xx_prev(2, 4) * r_f(4)) +
              Fx(3, 3) * (V_x_prev(3) + V_xx_prev(3, 0) * r_f(0) + V_xx_prev(3, 1) * r_f(1) + V_xx_prev(3, 2) * r_f(2) +
                          V_xx_prev(3, 3) * r_f(3) + V_xx_prev(3, 4) * r_f(4)) +
              V_(13) * g_x(13, 3) + V_(14) * g_x(14, 3) + V_(15) * g_x(15, 3) + V_(16) * g_x(16, 3) +
              V_(2) * g_x(2, 3) + V_(3) * g_x(3, 3) + V_(4) * g_x(4, 3) + V_(5) * g_x(5, 3) + V_(8) * g_x(8, 3) +
              sl_x(3);
    Q_x_(4) = Fx(0, 4) * (V_x_prev(0) + V_xx_prev(0, 0) * r_f(0) + V_xx_prev(0, 1) * r_f(1) + V_xx_prev(0, 2) * r_f(2) +
                          V_xx_prev(0, 3) * r_f(3) + V_xx_prev(0, 4) * r_f(4)) +
              Fx(1, 4) * (V_x_prev(1) + V_xx_prev(1, 0) * r_f(0) + V_xx_prev(1, 1) * r_f(1) + V_xx_prev(1, 2) * r_f(2) +
                          V_xx_prev(1, 3) * r_f(3) + V_xx_prev(1, 4) * r_f(4)) +
              Fx(2, 4) * (V_x_prev(2) + V_xx_prev(2, 0) * r_f(0) + V_xx_prev(2, 1) * r_f(1) + V_xx_prev(2, 2) * r_f(2) +
                          V_xx_prev(2, 3) * r_f(3) + V_xx_prev(2, 4) * r_f(4)) +
              Fx(3, 4) * (V_x_prev(3) + V_xx_prev(3, 0) * r_f(0) + V_xx_prev(3, 1) * r_f(1) + V_xx_prev(3, 2) * r_f(2) +
                          V_xx_prev(3, 3) * r_f(3) + V_xx_prev(3, 4) * r_f(4)) +
              Fx(4, 4) * (V_x_prev(4) + V_xx_prev(4, 0) * r_f(0) + V_xx_prev(4, 1) * r_f(1) + V_xx_prev(4, 2) * r_f(2) +
                          V_xx_prev(4, 3) * r_f(3) + V_xx_prev(4, 4) * r_f(4)) +
              V_(6) * g_x(6, 4) + V_(7) * g_x(7, 4) + sl_x(4);

    // Evaluation of Vector Q_u_
    Q_u_(0) = Fu(0, 0) * (V_x_prev(0) + V_xx_prev(0, 0) * r_f(0) + V_xx_prev(0, 1) * r_f(1) + V_xx_prev(0, 2) * r_f(2) +
                          V_xx_prev(0, 3) * r_f(3) + V_xx_prev(0, 4) * r_f(4)) +
              Fu(1, 0) * (V_x_prev(1) + V_xx_prev(1, 0) * r_f(0) + V_xx_prev(1, 1) * r_f(1) + V_xx_prev(1, 2) * r_f(2) +
                          V_xx_prev(1, 3) * r_f(3) + V_xx_prev(1, 4) * r_f(4)) +
              Fu(2, 0) * (V_x_prev(2) + V_xx_prev(2, 0) * r_f(0) + V_xx_prev(2, 1) * r_f(1) + V_xx_prev(2, 2) * r_f(2) +
                          V_xx_prev(2, 3) * r_f(3) + V_xx_prev(2, 4) * r_f(4)) +
              Fu(3, 0) * (V_x_prev(3) + V_xx_prev(3, 0) * r_f(0) + V_xx_prev(3, 1) * r_f(1) + V_xx_prev(3, 2) * r_f(2) +
                          V_xx_prev(3, 3) * r_f(3) + V_xx_prev(3, 4) * r_f(4)) +
              Fu(4, 0) * (V_x_prev(4) + V_xx_prev(4, 0) * r_f(0) + V_xx_prev(4, 1) * r_f(1) + V_xx_prev(4, 2) * r_f(2) +
                          V_xx_prev(4, 3) * r_f(3) + V_xx_prev(4, 4) * r_f(4)) +
              V_(10) * g_u(10, 0) + V_(9) * g_u(9, 0) + sl_u(0);
    Q_u_(1) = V_(11) * g_u(11, 1) + V_(12) * g_u(12, 1) + V_(13) * g_u(13, 1) + V_(14) * g_u(14, 1) +
              V_(15) * g_u(15, 1) + V_(16) * g_u(16, 1) + sl_u(1);
  }

  virtual void updateQuuInv() override {
    // Evaluation of Matrix Q_uu_inv_
    Q_uu_inv_(0, 0) = 1.0 / Q_uu_(0, 0);
    Q_uu_inv_(1, 1) = 1.0 / Q_uu_(1, 1);
  }

  virtual void updateKVU(Eigen::MatrixXd& k_ux, Eigen::VectorXd& v_u) override {
    // Evaluation of Matrix k_ux
    k_ux(0, 0) = -Q_uu_inv_(0, 0) * Q_xu_(0, 0);
    k_ux(0, 1) = -Q_uu_inv_(0, 0) * Q_xu_(1, 0);
    k_ux(0, 2) = -Q_uu_inv_(0, 0) * Q_xu_(2, 0);
    k_ux(0, 3) = -Q_uu_inv_(0, 0) * Q_xu_(3, 0);
    k_ux(0, 4) = -Q_uu_inv_(0, 0) * Q_xu_(4, 0);
    k_ux(1, 1) = -Q_uu_inv_(1, 1) * Q_xu_(1, 1);
    k_ux(1, 2) = -Q_uu_inv_(1, 1) * Q_xu_(2, 1);
    k_ux(1, 3) = -Q_uu_inv_(1, 1) * Q_xu_(3, 1);

    // Evaluation of Vector v_u
    v_u(0) = -Q_u_(0) * Q_uu_inv_(0, 0);
    v_u(1) = -Q_u_(1) * Q_uu_inv_(1, 1);
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
    V_xx(0, 0) = Q_xu_(0, 0) * k_ux(0, 0) + Q_xx_(0, 0);
    V_xx(0, 1) = Q_xu_(0, 0) * k_ux(0, 1) + Q_xx_(0, 1);
    V_xx(0, 2) = Q_xu_(0, 0) * k_ux(0, 2) + Q_xx_(0, 2);
    V_xx(0, 3) = Q_xu_(0, 0) * k_ux(0, 3) + Q_xx_(0, 3);
    V_xx(0, 4) = Q_xu_(0, 0) * k_ux(0, 4) + Q_xx_(0, 4);
    V_xx(1, 0) = Q_xu_(1, 0) * k_ux(0, 0) + Q_xx_(0, 1);
    V_xx(1, 1) = Q_xu_(1, 0) * k_ux(0, 1) + Q_xu_(1, 1) * k_ux(1, 1) + Q_xx_(1, 1);
    V_xx(1, 2) = Q_xu_(1, 0) * k_ux(0, 2) + Q_xu_(1, 1) * k_ux(1, 2) + Q_xx_(1, 2);
    V_xx(1, 3) = Q_xu_(1, 0) * k_ux(0, 3) + Q_xu_(1, 1) * k_ux(1, 3) + Q_xx_(1, 3);
    V_xx(1, 4) = Q_xu_(1, 0) * k_ux(0, 4) + Q_xx_(1, 4);
    V_xx(2, 0) = Q_xu_(2, 0) * k_ux(0, 0) + Q_xx_(0, 2);
    V_xx(2, 1) = Q_xu_(2, 0) * k_ux(0, 1) + Q_xu_(2, 1) * k_ux(1, 1) + Q_xx_(1, 2);
    V_xx(2, 2) = Q_xu_(2, 0) * k_ux(0, 2) + Q_xu_(2, 1) * k_ux(1, 2) + Q_xx_(2, 2);
    V_xx(2, 3) = Q_xu_(2, 0) * k_ux(0, 3) + Q_xu_(2, 1) * k_ux(1, 3) + Q_xx_(2, 3);
    V_xx(2, 4) = Q_xu_(2, 0) * k_ux(0, 4) + Q_xx_(2, 4);
    V_xx(3, 0) = Q_xu_(3, 0) * k_ux(0, 0) + Q_xx_(0, 3);
    V_xx(3, 1) = Q_xu_(3, 0) * k_ux(0, 1) + Q_xu_(3, 1) * k_ux(1, 1) + Q_xx_(1, 3);
    V_xx(3, 2) = Q_xu_(3, 0) * k_ux(0, 2) + Q_xu_(3, 1) * k_ux(1, 2) + Q_xx_(2, 3);
    V_xx(3, 3) = Q_xu_(3, 0) * k_ux(0, 3) + Q_xu_(3, 1) * k_ux(1, 3) + Q_xx_(3, 3);
    V_xx(3, 4) = Q_xu_(3, 0) * k_ux(0, 4) + Q_xx_(3, 4);
    V_xx(4, 0) = Q_xu_(4, 0) * k_ux(0, 0) + Q_xx_(0, 4);
    V_xx(4, 1) = Q_xu_(4, 0) * k_ux(0, 1) + Q_xx_(1, 4);
    V_xx(4, 2) = Q_xu_(4, 0) * k_ux(0, 2) + Q_xx_(2, 4);
    V_xx(4, 3) = Q_xu_(4, 0) * k_ux(0, 3) + Q_xx_(3, 4);
    V_xx(4, 4) = Q_xu_(4, 0) * k_ux(0, 4) + Q_xx_(4, 4);

    // Evaluation of Vector V_x
    V_x(0) = Q_x_(0) + Q_xu_(0, 0) * v_u(0);
    V_x(1) = Q_x_(1) + Q_xu_(1, 0) * v_u(0) + Q_xu_(1, 1) * v_u(1);
    V_x(2) = Q_x_(2) + Q_xu_(2, 0) * v_u(0) + Q_xu_(2, 1) * v_u(1);
    V_x(3) = Q_x_(3) + Q_xu_(3, 0) * v_u(0) + Q_xu_(3, 1) * v_u(1);
    V_x(4) = Q_x_(4) + Q_xu_(4, 0) * v_u(0);
  }

  virtual void updateQnx(const Eigen::VectorXd& r_f, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& Fu,
                         const Eigen::MatrixXd& h_u, const Eigen::MatrixXd& k_ux, const Eigen::MatrixXd& k_ex,
                         const Eigen::MatrixXd& k_ue, const Eigen::VectorXd& v_u, const Eigen::VectorXd& v_e) override {
    // Evaluation of Matrix Q_un_
    Q_un_(0, 0) = -Fu(0, 0) * Q_uu_inv_(0, 0);
    Q_un_(0, 1) = -Fu(1, 0) * Q_uu_inv_(0, 0);
    Q_un_(0, 2) = -Fu(2, 0) * Q_uu_inv_(0, 0);
    Q_un_(0, 3) = -Fu(3, 0) * Q_uu_inv_(0, 0);
    Q_un_(0, 4) = -Fu(4, 0) * Q_uu_inv_(0, 0);

    // Evaluation of Matrix Q_nuu_
    Q_nuu_(0, 0) = -Q_uu_(0, 0);
    Q_nuu_(1, 1) = -Q_uu_(1, 1);

    // Evaluation of Matrix Q_vnx_
    Q_vnx_(0, 0) = Fu(0, 0) * k_ux(0, 0) + Fx(0, 0);
    Q_vnx_(0, 1) = Fu(0, 0) * k_ux(0, 1) + Fx(0, 1);
    Q_vnx_(0, 2) = Fu(0, 0) * k_ux(0, 2) + Fx(0, 2);
    Q_vnx_(0, 3) = Fu(0, 0) * k_ux(0, 3) + Fx(0, 3);
    Q_vnx_(0, 4) = Fu(0, 0) * k_ux(0, 4) + Fx(0, 4);
    Q_vnx_(1, 0) = Fu(1, 0) * k_ux(0, 0);
    Q_vnx_(1, 1) = Fu(1, 0) * k_ux(0, 1) + Fx(1, 1);
    Q_vnx_(1, 2) = Fu(1, 0) * k_ux(0, 2);
    Q_vnx_(1, 3) = Fu(1, 0) * k_ux(0, 3) + Fx(1, 3);
    Q_vnx_(1, 4) = Fu(1, 0) * k_ux(0, 4) + Fx(1, 4);
    Q_vnx_(2, 0) = Fu(2, 0) * k_ux(0, 0);
    Q_vnx_(2, 1) = Fu(2, 0) * k_ux(0, 1);
    Q_vnx_(2, 2) = Fu(2, 0) * k_ux(0, 2) + Fx(2, 2);
    Q_vnx_(2, 3) = Fu(2, 0) * k_ux(0, 3) + Fx(2, 3);
    Q_vnx_(2, 4) = Fu(2, 0) * k_ux(0, 4) + Fx(2, 4);
    Q_vnx_(3, 0) = Fu(3, 0) * k_ux(0, 0);
    Q_vnx_(3, 1) = Fu(3, 0) * k_ux(0, 1);
    Q_vnx_(3, 2) = Fu(3, 0) * k_ux(0, 2);
    Q_vnx_(3, 3) = Fu(3, 0) * k_ux(0, 3) + Fx(3, 3);
    Q_vnx_(3, 4) = Fu(3, 0) * k_ux(0, 4) + Fx(3, 4);
    Q_vnx_(4, 0) = Fu(4, 0) * k_ux(0, 0);
    Q_vnx_(4, 1) = Fu(4, 0) * k_ux(0, 1);
    Q_vnx_(4, 2) = Fu(4, 0) * k_ux(0, 2);
    Q_vnx_(4, 3) = Fu(4, 0) * k_ux(0, 3);
    Q_vnx_(4, 4) = Fu(4, 0) * k_ux(0, 4) + Fx(4, 4);

    // Evaluation of Vector Q_vn_
    Q_vn_(0) = Fu(0, 0) * v_u(0) + r_f(0);
    Q_vn_(1) = Fu(1, 0) * v_u(0) + r_f(1);
    Q_vn_(2) = Fu(2, 0) * v_u(0) + r_f(2);
    Q_vn_(3) = Fu(3, 0) * v_u(0) + r_f(3);
    Q_vn_(4) = Fu(4, 0) * v_u(0) + r_f(4);
  }
};

template <>
class ParkLateralGeneralIpmEvaluator<StageType::INITIAL, OcpConfig::ERK4> : public IpmEvaluator {
 public:
  ParkLateralGeneralIpmEvaluator() : IpmEvaluator(5, 2, 2, 0, 5) {}
  virtual ~ParkLateralGeneralIpmEvaluator() = default;

  virtual void updateLX(const Eigen::VectorXd& l_x, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& g_x,
                        const Eigen::MatrixXd& h_x, const Eigen::MatrixXd& m_x, const Eigen::VectorXd& p,
                        const Eigen::VectorXd& lambda, const Eigen::VectorXd& eta, const Eigen::VectorXd& nu,
                        Eigen::VectorXd& sl_x) const override {
    // Evaluation of Vector sl_x
    sl_x(0) = Fx(0, 0) * p(0) + m_x(0, 0) * nu(0);
    sl_x(1) = Fx(0, 1) * p(0) + Fx(1, 1) * p(1) + l_x(1) + m_x(1, 1) * nu(1);
    sl_x(2) = Fx(0, 2) * p(0) + Fx(2, 2) * p(2) + l_x(2) + m_x(2, 2) * nu(2);
    sl_x(3) = Fx(0, 3) * p(0) + Fx(1, 3) * p(1) + Fx(2, 3) * p(2) + Fx(3, 3) * p(3) + m_x(3, 3) * nu(3);
    sl_x(4) = Fx(0, 4) * p(0) + Fx(1, 4) * p(1) + Fx(2, 4) * p(2) + Fx(3, 4) * p(3) + Fx(4, 4) * p(4) + l_x(4) +
              m_x(4, 4) * nu(4);
  }

  virtual void updateLU(const Eigen::VectorXd& l_u, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& g_u,
                        const Eigen::MatrixXd& h_u, const Eigen::VectorXd& p, const Eigen::VectorXd& lambda,
                        const Eigen::VectorXd& eta, Eigen::VectorXd& sl_u) const override {
    // Evaluation of Vector sl_u
    sl_u(0) = Fu(0, 0) * p(0) + Fu(1, 0) * p(1) + Fu(2, 0) * p(2) + Fu(3, 0) * p(3) + Fu(4, 0) * p(4) +
              g_u(0, 0) * lambda(0) + g_u(1, 0) * lambda(1) + l_u(0);
    sl_u(1) = l_u(1);
  }

  virtual void updateQH(const Eigen::MatrixXd& l_xx, const Eigen::MatrixXd& l_xu, const Eigen::MatrixXd& l_uu,
                        const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::MatrixXd& Fx,
                        const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& V_xx_prev) override {
    // Determine internal variables
    const double Fx_0_0_p2 = Fx(0, 0) * Fx(0, 0);
    const double g_u_0_0_p2 = g_u(0, 0) * g_u(0, 0);
    const double g_u_1_0_p2 = g_u(1, 0) * g_u(1, 0);

    // Evaluation of Matrix Q_xx_
    Q_xx_(0, 0) = Fx_0_0_p2 * V_xx_prev(0, 0);
    Q_xx_(0, 1) = Fx(0, 0) * Fx(0, 1) * V_xx_prev(0, 0) + Fx(0, 0) * Fx(1, 1) * V_xx_prev(0, 1);
    Q_xx_(0, 2) = Fx(0, 0) * Fx(0, 2) * V_xx_prev(0, 0) + Fx(0, 0) * Fx(2, 2) * V_xx_prev(0, 2);
    Q_xx_(0, 3) = Fx(0, 0) * Fx(0, 3) * V_xx_prev(0, 0) + Fx(0, 0) * Fx(1, 3) * V_xx_prev(0, 1) +
                  Fx(0, 0) * Fx(2, 3) * V_xx_prev(0, 2) + Fx(0, 0) * Fx(3, 3) * V_xx_prev(0, 3);
    Q_xx_(0, 4) = Fx(0, 0) * Fx(0, 4) * V_xx_prev(0, 0) + Fx(0, 0) * Fx(1, 4) * V_xx_prev(0, 1) +
                  Fx(0, 0) * Fx(2, 4) * V_xx_prev(0, 2) + Fx(0, 0) * Fx(3, 4) * V_xx_prev(0, 3) +
                  Fx(0, 0) * Fx(4, 4) * V_xx_prev(0, 4);
    Q_xx_(1, 0) = Fx(0, 0) * (Fx(0, 1) * V_xx_prev(0, 0) + Fx(1, 1) * V_xx_prev(1, 0));
    Q_xx_(1, 1) = Fx(0, 1) * (Fx(0, 1) * V_xx_prev(0, 0) + Fx(1, 1) * V_xx_prev(1, 0)) +
                  Fx(1, 1) * (Fx(0, 1) * V_xx_prev(0, 1) + Fx(1, 1) * V_xx_prev(1, 1)) + l_xx(1, 1);
    Q_xx_(1, 2) = Fx(0, 2) * (Fx(0, 1) * V_xx_prev(0, 0) + Fx(1, 1) * V_xx_prev(1, 0)) +
                  Fx(2, 2) * (Fx(0, 1) * V_xx_prev(0, 2) + Fx(1, 1) * V_xx_prev(1, 2)) + l_xx(1, 2);
    Q_xx_(1, 3) = Fx(0, 3) * (Fx(0, 1) * V_xx_prev(0, 0) + Fx(1, 1) * V_xx_prev(1, 0)) +
                  Fx(1, 3) * (Fx(0, 1) * V_xx_prev(0, 1) + Fx(1, 1) * V_xx_prev(1, 1)) +
                  Fx(2, 3) * (Fx(0, 1) * V_xx_prev(0, 2) + Fx(1, 1) * V_xx_prev(1, 2)) +
                  Fx(3, 3) * (Fx(0, 1) * V_xx_prev(0, 3) + Fx(1, 1) * V_xx_prev(1, 3));
    Q_xx_(1, 4) = Fx(0, 4) * (Fx(0, 1) * V_xx_prev(0, 0) + Fx(1, 1) * V_xx_prev(1, 0)) +
                  Fx(1, 4) * (Fx(0, 1) * V_xx_prev(0, 1) + Fx(1, 1) * V_xx_prev(1, 1)) +
                  Fx(2, 4) * (Fx(0, 1) * V_xx_prev(0, 2) + Fx(1, 1) * V_xx_prev(1, 2)) +
                  Fx(3, 4) * (Fx(0, 1) * V_xx_prev(0, 3) + Fx(1, 1) * V_xx_prev(1, 3)) +
                  Fx(4, 4) * (Fx(0, 1) * V_xx_prev(0, 4) + Fx(1, 1) * V_xx_prev(1, 4));
    Q_xx_(2, 0) = Fx(0, 0) * (Fx(0, 2) * V_xx_prev(0, 0) + Fx(2, 2) * V_xx_prev(2, 0));
    Q_xx_(2, 1) = Fx(0, 1) * (Fx(0, 2) * V_xx_prev(0, 0) + Fx(2, 2) * V_xx_prev(2, 0)) +
                  Fx(1, 1) * (Fx(0, 2) * V_xx_prev(0, 1) + Fx(2, 2) * V_xx_prev(2, 1)) + l_xx(2, 1);
    Q_xx_(2, 2) = Fx(0, 2) * (Fx(0, 2) * V_xx_prev(0, 0) + Fx(2, 2) * V_xx_prev(2, 0)) +
                  Fx(2, 2) * (Fx(0, 2) * V_xx_prev(0, 2) + Fx(2, 2) * V_xx_prev(2, 2)) + l_xx(2, 2);
    Q_xx_(2, 3) = Fx(0, 3) * (Fx(0, 2) * V_xx_prev(0, 0) + Fx(2, 2) * V_xx_prev(2, 0)) +
                  Fx(1, 3) * (Fx(0, 2) * V_xx_prev(0, 1) + Fx(2, 2) * V_xx_prev(2, 1)) +
                  Fx(2, 3) * (Fx(0, 2) * V_xx_prev(0, 2) + Fx(2, 2) * V_xx_prev(2, 2)) +
                  Fx(3, 3) * (Fx(0, 2) * V_xx_prev(0, 3) + Fx(2, 2) * V_xx_prev(2, 3));
    Q_xx_(2, 4) = Fx(0, 4) * (Fx(0, 2) * V_xx_prev(0, 0) + Fx(2, 2) * V_xx_prev(2, 0)) +
                  Fx(1, 4) * (Fx(0, 2) * V_xx_prev(0, 1) + Fx(2, 2) * V_xx_prev(2, 1)) +
                  Fx(2, 4) * (Fx(0, 2) * V_xx_prev(0, 2) + Fx(2, 2) * V_xx_prev(2, 2)) +
                  Fx(3, 4) * (Fx(0, 2) * V_xx_prev(0, 3) + Fx(2, 2) * V_xx_prev(2, 3)) +
                  Fx(4, 4) * (Fx(0, 2) * V_xx_prev(0, 4) + Fx(2, 2) * V_xx_prev(2, 4));
    Q_xx_(3, 0) = Fx(0, 0) * (Fx(0, 3) * V_xx_prev(0, 0) + Fx(1, 3) * V_xx_prev(1, 0) + Fx(2, 3) * V_xx_prev(2, 0) +
                              Fx(3, 3) * V_xx_prev(3, 0));
    Q_xx_(3, 1) = Fx(0, 1) * (Fx(0, 3) * V_xx_prev(0, 0) + Fx(1, 3) * V_xx_prev(1, 0) + Fx(2, 3) * V_xx_prev(2, 0) +
                              Fx(3, 3) * V_xx_prev(3, 0)) +
                  Fx(1, 1) * (Fx(0, 3) * V_xx_prev(0, 1) + Fx(1, 3) * V_xx_prev(1, 1) + Fx(2, 3) * V_xx_prev(2, 1) +
                              Fx(3, 3) * V_xx_prev(3, 1));
    Q_xx_(3, 2) = Fx(0, 2) * (Fx(0, 3) * V_xx_prev(0, 0) + Fx(1, 3) * V_xx_prev(1, 0) + Fx(2, 3) * V_xx_prev(2, 0) +
                              Fx(3, 3) * V_xx_prev(3, 0)) +
                  Fx(2, 2) * (Fx(0, 3) * V_xx_prev(0, 2) + Fx(1, 3) * V_xx_prev(1, 2) + Fx(2, 3) * V_xx_prev(2, 2) +
                              Fx(3, 3) * V_xx_prev(3, 2));
    Q_xx_(3, 3) = Fx(0, 3) * (Fx(0, 3) * V_xx_prev(0, 0) + Fx(1, 3) * V_xx_prev(1, 0) + Fx(2, 3) * V_xx_prev(2, 0) +
                              Fx(3, 3) * V_xx_prev(3, 0)) +
                  Fx(1, 3) * (Fx(0, 3) * V_xx_prev(0, 1) + Fx(1, 3) * V_xx_prev(1, 1) + Fx(2, 3) * V_xx_prev(2, 1) +
                              Fx(3, 3) * V_xx_prev(3, 1)) +
                  Fx(2, 3) * (Fx(0, 3) * V_xx_prev(0, 2) + Fx(1, 3) * V_xx_prev(1, 2) + Fx(2, 3) * V_xx_prev(2, 2) +
                              Fx(3, 3) * V_xx_prev(3, 2)) +
                  Fx(3, 3) * (Fx(0, 3) * V_xx_prev(0, 3) + Fx(1, 3) * V_xx_prev(1, 3) + Fx(2, 3) * V_xx_prev(2, 3) +
                              Fx(3, 3) * V_xx_prev(3, 3));
    Q_xx_(3, 4) = Fx(0, 4) * (Fx(0, 3) * V_xx_prev(0, 0) + Fx(1, 3) * V_xx_prev(1, 0) + Fx(2, 3) * V_xx_prev(2, 0) +
                              Fx(3, 3) * V_xx_prev(3, 0)) +
                  Fx(1, 4) * (Fx(0, 3) * V_xx_prev(0, 1) + Fx(1, 3) * V_xx_prev(1, 1) + Fx(2, 3) * V_xx_prev(2, 1) +
                              Fx(3, 3) * V_xx_prev(3, 1)) +
                  Fx(2, 4) * (Fx(0, 3) * V_xx_prev(0, 2) + Fx(1, 3) * V_xx_prev(1, 2) + Fx(2, 3) * V_xx_prev(2, 2) +
                              Fx(3, 3) * V_xx_prev(3, 2)) +
                  Fx(3, 4) * (Fx(0, 3) * V_xx_prev(0, 3) + Fx(1, 3) * V_xx_prev(1, 3) + Fx(2, 3) * V_xx_prev(2, 3) +
                              Fx(3, 3) * V_xx_prev(3, 3)) +
                  Fx(4, 4) * (Fx(0, 3) * V_xx_prev(0, 4) + Fx(1, 3) * V_xx_prev(1, 4) + Fx(2, 3) * V_xx_prev(2, 4) +
                              Fx(3, 3) * V_xx_prev(3, 4));
    Q_xx_(4, 0) = Fx(0, 0) * (Fx(0, 4) * V_xx_prev(0, 0) + Fx(1, 4) * V_xx_prev(1, 0) + Fx(2, 4) * V_xx_prev(2, 0) +
                              Fx(3, 4) * V_xx_prev(3, 0) + Fx(4, 4) * V_xx_prev(4, 0));
    Q_xx_(4, 1) = Fx(0, 1) * (Fx(0, 4) * V_xx_prev(0, 0) + Fx(1, 4) * V_xx_prev(1, 0) + Fx(2, 4) * V_xx_prev(2, 0) +
                              Fx(3, 4) * V_xx_prev(3, 0) + Fx(4, 4) * V_xx_prev(4, 0)) +
                  Fx(1, 1) * (Fx(0, 4) * V_xx_prev(0, 1) + Fx(1, 4) * V_xx_prev(1, 1) + Fx(2, 4) * V_xx_prev(2, 1) +
                              Fx(3, 4) * V_xx_prev(3, 1) + Fx(4, 4) * V_xx_prev(4, 1));
    Q_xx_(4, 2) = Fx(0, 2) * (Fx(0, 4) * V_xx_prev(0, 0) + Fx(1, 4) * V_xx_prev(1, 0) + Fx(2, 4) * V_xx_prev(2, 0) +
                              Fx(3, 4) * V_xx_prev(3, 0) + Fx(4, 4) * V_xx_prev(4, 0)) +
                  Fx(2, 2) * (Fx(0, 4) * V_xx_prev(0, 2) + Fx(1, 4) * V_xx_prev(1, 2) + Fx(2, 4) * V_xx_prev(2, 2) +
                              Fx(3, 4) * V_xx_prev(3, 2) + Fx(4, 4) * V_xx_prev(4, 2));
    Q_xx_(4, 3) = Fx(0, 3) * (Fx(0, 4) * V_xx_prev(0, 0) + Fx(1, 4) * V_xx_prev(1, 0) + Fx(2, 4) * V_xx_prev(2, 0) +
                              Fx(3, 4) * V_xx_prev(3, 0) + Fx(4, 4) * V_xx_prev(4, 0)) +
                  Fx(1, 3) * (Fx(0, 4) * V_xx_prev(0, 1) + Fx(1, 4) * V_xx_prev(1, 1) + Fx(2, 4) * V_xx_prev(2, 1) +
                              Fx(3, 4) * V_xx_prev(3, 1) + Fx(4, 4) * V_xx_prev(4, 1)) +
                  Fx(2, 3) * (Fx(0, 4) * V_xx_prev(0, 2) + Fx(1, 4) * V_xx_prev(1, 2) + Fx(2, 4) * V_xx_prev(2, 2) +
                              Fx(3, 4) * V_xx_prev(3, 2) + Fx(4, 4) * V_xx_prev(4, 2)) +
                  Fx(3, 3) * (Fx(0, 4) * V_xx_prev(0, 3) + Fx(1, 4) * V_xx_prev(1, 3) + Fx(2, 4) * V_xx_prev(2, 3) +
                              Fx(3, 4) * V_xx_prev(3, 3) + Fx(4, 4) * V_xx_prev(4, 3));
    Q_xx_(4, 4) = Fx(0, 4) * (Fx(0, 4) * V_xx_prev(0, 0) + Fx(1, 4) * V_xx_prev(1, 0) + Fx(2, 4) * V_xx_prev(2, 0) +
                              Fx(3, 4) * V_xx_prev(3, 0) + Fx(4, 4) * V_xx_prev(4, 0)) +
                  Fx(1, 4) * (Fx(0, 4) * V_xx_prev(0, 1) + Fx(1, 4) * V_xx_prev(1, 1) + Fx(2, 4) * V_xx_prev(2, 1) +
                              Fx(3, 4) * V_xx_prev(3, 1) + Fx(4, 4) * V_xx_prev(4, 1)) +
                  Fx(2, 4) * (Fx(0, 4) * V_xx_prev(0, 2) + Fx(1, 4) * V_xx_prev(1, 2) + Fx(2, 4) * V_xx_prev(2, 2) +
                              Fx(3, 4) * V_xx_prev(3, 2) + Fx(4, 4) * V_xx_prev(4, 2)) +
                  Fx(3, 4) * (Fx(0, 4) * V_xx_prev(0, 3) + Fx(1, 4) * V_xx_prev(1, 3) + Fx(2, 4) * V_xx_prev(2, 3) +
                              Fx(3, 4) * V_xx_prev(3, 3) + Fx(4, 4) * V_xx_prev(4, 3)) +
                  Fx(4, 4) * (Fx(0, 4) * V_xx_prev(0, 4) + Fx(1, 4) * V_xx_prev(1, 4) + Fx(2, 4) * V_xx_prev(2, 4) +
                              Fx(3, 4) * V_xx_prev(3, 4) + Fx(4, 4) * V_xx_prev(4, 4)) +
                  l_xx(4, 4);

    // Evaluation of Matrix Q_xu_
    Q_xu_(0, 0) = Fu(0, 0) * Fx(0, 0) * V_xx_prev(0, 0) + Fu(1, 0) * Fx(0, 0) * V_xx_prev(0, 1) +
                  Fu(2, 0) * Fx(0, 0) * V_xx_prev(0, 2) + Fu(3, 0) * Fx(0, 0) * V_xx_prev(0, 3) +
                  Fu(4, 0) * Fx(0, 0) * V_xx_prev(0, 4);
    Q_xu_(1, 0) = Fu(0, 0) * (Fx(0, 1) * V_xx_prev(0, 0) + Fx(1, 1) * V_xx_prev(1, 0)) +
                  Fu(1, 0) * (Fx(0, 1) * V_xx_prev(0, 1) + Fx(1, 1) * V_xx_prev(1, 1)) +
                  Fu(2, 0) * (Fx(0, 1) * V_xx_prev(0, 2) + Fx(1, 1) * V_xx_prev(1, 2)) +
                  Fu(3, 0) * (Fx(0, 1) * V_xx_prev(0, 3) + Fx(1, 1) * V_xx_prev(1, 3)) +
                  Fu(4, 0) * (Fx(0, 1) * V_xx_prev(0, 4) + Fx(1, 1) * V_xx_prev(1, 4));
    Q_xu_(2, 0) = Fu(0, 0) * (Fx(0, 2) * V_xx_prev(0, 0) + Fx(2, 2) * V_xx_prev(2, 0)) +
                  Fu(1, 0) * (Fx(0, 2) * V_xx_prev(0, 1) + Fx(2, 2) * V_xx_prev(2, 1)) +
                  Fu(2, 0) * (Fx(0, 2) * V_xx_prev(0, 2) + Fx(2, 2) * V_xx_prev(2, 2)) +
                  Fu(3, 0) * (Fx(0, 2) * V_xx_prev(0, 3) + Fx(2, 2) * V_xx_prev(2, 3)) +
                  Fu(4, 0) * (Fx(0, 2) * V_xx_prev(0, 4) + Fx(2, 2) * V_xx_prev(2, 4));
    Q_xu_(3, 0) = Fu(0, 0) * (Fx(0, 3) * V_xx_prev(0, 0) + Fx(1, 3) * V_xx_prev(1, 0) + Fx(2, 3) * V_xx_prev(2, 0) +
                              Fx(3, 3) * V_xx_prev(3, 0)) +
                  Fu(1, 0) * (Fx(0, 3) * V_xx_prev(0, 1) + Fx(1, 3) * V_xx_prev(1, 1) + Fx(2, 3) * V_xx_prev(2, 1) +
                              Fx(3, 3) * V_xx_prev(3, 1)) +
                  Fu(2, 0) * (Fx(0, 3) * V_xx_prev(0, 2) + Fx(1, 3) * V_xx_prev(1, 2) + Fx(2, 3) * V_xx_prev(2, 2) +
                              Fx(3, 3) * V_xx_prev(3, 2)) +
                  Fu(3, 0) * (Fx(0, 3) * V_xx_prev(0, 3) + Fx(1, 3) * V_xx_prev(1, 3) + Fx(2, 3) * V_xx_prev(2, 3) +
                              Fx(3, 3) * V_xx_prev(3, 3)) +
                  Fu(4, 0) * (Fx(0, 3) * V_xx_prev(0, 4) + Fx(1, 3) * V_xx_prev(1, 4) + Fx(2, 3) * V_xx_prev(2, 4) +
                              Fx(3, 3) * V_xx_prev(3, 4));
    Q_xu_(4, 0) = Fu(0, 0) * (Fx(0, 4) * V_xx_prev(0, 0) + Fx(1, 4) * V_xx_prev(1, 0) + Fx(2, 4) * V_xx_prev(2, 0) +
                              Fx(3, 4) * V_xx_prev(3, 0) + Fx(4, 4) * V_xx_prev(4, 0)) +
                  Fu(1, 0) * (Fx(0, 4) * V_xx_prev(0, 1) + Fx(1, 4) * V_xx_prev(1, 1) + Fx(2, 4) * V_xx_prev(2, 1) +
                              Fx(3, 4) * V_xx_prev(3, 1) + Fx(4, 4) * V_xx_prev(4, 1)) +
                  Fu(2, 0) * (Fx(0, 4) * V_xx_prev(0, 2) + Fx(1, 4) * V_xx_prev(1, 2) + Fx(2, 4) * V_xx_prev(2, 2) +
                              Fx(3, 4) * V_xx_prev(3, 2) + Fx(4, 4) * V_xx_prev(4, 2)) +
                  Fu(3, 0) * (Fx(0, 4) * V_xx_prev(0, 3) + Fx(1, 4) * V_xx_prev(1, 3) + Fx(2, 4) * V_xx_prev(2, 3) +
                              Fx(3, 4) * V_xx_prev(3, 3) + Fx(4, 4) * V_xx_prev(4, 3)) +
                  Fu(4, 0) * (Fx(0, 4) * V_xx_prev(0, 4) + Fx(1, 4) * V_xx_prev(1, 4) + Fx(2, 4) * V_xx_prev(2, 4) +
                              Fx(3, 4) * V_xx_prev(3, 4) + Fx(4, 4) * V_xx_prev(4, 4));

    // Evaluation of Matrix Q_uu_
    Q_uu_(0, 0) = Fu(0, 0) * (Fu(0, 0) * V_xx_prev(0, 0) + Fu(1, 0) * V_xx_prev(1, 0) + Fu(2, 0) * V_xx_prev(2, 0) +
                              Fu(3, 0) * V_xx_prev(3, 0) + Fu(4, 0) * V_xx_prev(4, 0)) +
                  Fu(1, 0) * (Fu(0, 0) * V_xx_prev(0, 1) + Fu(1, 0) * V_xx_prev(1, 1) + Fu(2, 0) * V_xx_prev(2, 1) +
                              Fu(3, 0) * V_xx_prev(3, 1) + Fu(4, 0) * V_xx_prev(4, 1)) +
                  Fu(2, 0) * (Fu(0, 0) * V_xx_prev(0, 2) + Fu(1, 0) * V_xx_prev(1, 2) + Fu(2, 0) * V_xx_prev(2, 2) +
                              Fu(3, 0) * V_xx_prev(3, 2) + Fu(4, 0) * V_xx_prev(4, 2)) +
                  Fu(3, 0) * (Fu(0, 0) * V_xx_prev(0, 3) + Fu(1, 0) * V_xx_prev(1, 3) + Fu(2, 0) * V_xx_prev(2, 3) +
                              Fu(3, 0) * V_xx_prev(3, 3) + Fu(4, 0) * V_xx_prev(4, 3)) +
                  Fu(4, 0) * (Fu(0, 0) * V_xx_prev(0, 4) + Fu(1, 0) * V_xx_prev(1, 4) + Fu(2, 0) * V_xx_prev(2, 4) +
                              Fu(3, 0) * V_xx_prev(3, 4) + Fu(4, 0) * V_xx_prev(4, 4)) +
                  Sigma_(0) * g_u_0_0_p2 + Sigma_(1) * g_u_1_0_p2 + l_uu(0, 0);
    Q_uu_(1, 1) = l_uu(1, 1);
  }

  virtual void updateQG(const Eigen::VectorXd& sl_x, const Eigen::VectorXd& sl_u, const Eigen::MatrixXd& g_x,
                        const Eigen::MatrixXd& g_u, const Eigen::VectorXd& r_f, const Eigen::MatrixXd& Fx,
                        const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& V_xx_prev,
                        const Eigen::VectorXd& V_x_prev) override {
    // Evaluation of Vector Q_x_
    Q_x_(0) = Fx(0, 0) * (V_x_prev(0) + V_xx_prev(0, 0) * r_f(0) + V_xx_prev(0, 1) * r_f(1) + V_xx_prev(0, 2) * r_f(2) +
                          V_xx_prev(0, 3) * r_f(3) + V_xx_prev(0, 4) * r_f(4)) +
              sl_x(0);
    Q_x_(1) = Fx(0, 1) * (V_x_prev(0) + V_xx_prev(0, 0) * r_f(0) + V_xx_prev(0, 1) * r_f(1) + V_xx_prev(0, 2) * r_f(2) +
                          V_xx_prev(0, 3) * r_f(3) + V_xx_prev(0, 4) * r_f(4)) +
              Fx(1, 1) * (V_x_prev(1) + V_xx_prev(1, 0) * r_f(0) + V_xx_prev(1, 1) * r_f(1) + V_xx_prev(1, 2) * r_f(2) +
                          V_xx_prev(1, 3) * r_f(3) + V_xx_prev(1, 4) * r_f(4)) +
              sl_x(1);
    Q_x_(2) = Fx(0, 2) * (V_x_prev(0) + V_xx_prev(0, 0) * r_f(0) + V_xx_prev(0, 1) * r_f(1) + V_xx_prev(0, 2) * r_f(2) +
                          V_xx_prev(0, 3) * r_f(3) + V_xx_prev(0, 4) * r_f(4)) +
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
              Fx(3, 4) * (V_x_prev(3) + V_xx_prev(3, 0) * r_f(0) + V_xx_prev(3, 1) * r_f(1) + V_xx_prev(3, 2) * r_f(2) +
                          V_xx_prev(3, 3) * r_f(3) + V_xx_prev(3, 4) * r_f(4)) +
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
              Fu(4, 0) * (V_x_prev(4) + V_xx_prev(4, 0) * r_f(0) + V_xx_prev(4, 1) * r_f(1) + V_xx_prev(4, 2) * r_f(2) +
                          V_xx_prev(4, 3) * r_f(3) + V_xx_prev(4, 4) * r_f(4)) +
              V_(0) * g_u(0, 0) + V_(1) * g_u(1, 0) + sl_u(0);
    Q_u_(1) = sl_u(1);
  }

  virtual void updateQuuInv() override {
    // Evaluation of Matrix Q_uu_inv_
    Q_uu_inv_(0, 0) = 1.0 / Q_uu_(0, 0);
    Q_uu_inv_(1, 1) = 1.0 / Q_uu_(1, 1);
  }

  virtual void updateKVU(Eigen::MatrixXd& k_ux, Eigen::VectorXd& v_u) override {
    // Evaluation of Matrix k_ux
    k_ux(0, 0) = -Q_uu_inv_(0, 0) * Q_xu_(0, 0);
    k_ux(0, 1) = -Q_uu_inv_(0, 0) * Q_xu_(1, 0);
    k_ux(0, 2) = -Q_uu_inv_(0, 0) * Q_xu_(2, 0);
    k_ux(0, 3) = -Q_uu_inv_(0, 0) * Q_xu_(3, 0);
    k_ux(0, 4) = -Q_uu_inv_(0, 0) * Q_xu_(4, 0);

    // Evaluation of Vector v_u
    v_u(0) = -Q_u_(0) * Q_uu_inv_(0, 0);
    v_u(1) = -Q_u_(1) * Q_uu_inv_(1, 1);
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
    V_xx(0, 0) = Q_xu_(0, 0) * k_ux(0, 0) + Q_xx_(0, 0);
    V_xx(0, 1) = Q_xu_(0, 0) * k_ux(0, 1) + Q_xx_(0, 1);
    V_xx(0, 2) = Q_xu_(0, 0) * k_ux(0, 2) + Q_xx_(0, 2);
    V_xx(0, 3) = Q_xu_(0, 0) * k_ux(0, 3) + Q_xx_(0, 3);
    V_xx(0, 4) = Q_xu_(0, 0) * k_ux(0, 4) + Q_xx_(0, 4);
    V_xx(1, 0) = Q_xu_(1, 0) * k_ux(0, 0) + Q_xx_(0, 1);
    V_xx(1, 1) = Q_xu_(1, 0) * k_ux(0, 1) + Q_xx_(1, 1);
    V_xx(1, 2) = Q_xu_(1, 0) * k_ux(0, 2) + Q_xx_(1, 2);
    V_xx(1, 3) = Q_xu_(1, 0) * k_ux(0, 3) + Q_xx_(1, 3);
    V_xx(1, 4) = Q_xu_(1, 0) * k_ux(0, 4) + Q_xx_(1, 4);
    V_xx(2, 0) = Q_xu_(2, 0) * k_ux(0, 0) + Q_xx_(0, 2);
    V_xx(2, 1) = Q_xu_(2, 0) * k_ux(0, 1) + Q_xx_(1, 2);
    V_xx(2, 2) = Q_xu_(2, 0) * k_ux(0, 2) + Q_xx_(2, 2);
    V_xx(2, 3) = Q_xu_(2, 0) * k_ux(0, 3) + Q_xx_(2, 3);
    V_xx(2, 4) = Q_xu_(2, 0) * k_ux(0, 4) + Q_xx_(2, 4);
    V_xx(3, 0) = Q_xu_(3, 0) * k_ux(0, 0) + Q_xx_(0, 3);
    V_xx(3, 1) = Q_xu_(3, 0) * k_ux(0, 1) + Q_xx_(1, 3);
    V_xx(3, 2) = Q_xu_(3, 0) * k_ux(0, 2) + Q_xx_(2, 3);
    V_xx(3, 3) = Q_xu_(3, 0) * k_ux(0, 3) + Q_xx_(3, 3);
    V_xx(3, 4) = Q_xu_(3, 0) * k_ux(0, 4) + Q_xx_(3, 4);
    V_xx(4, 0) = Q_xu_(4, 0) * k_ux(0, 0) + Q_xx_(0, 4);
    V_xx(4, 1) = Q_xu_(4, 0) * k_ux(0, 1) + Q_xx_(1, 4);
    V_xx(4, 2) = Q_xu_(4, 0) * k_ux(0, 2) + Q_xx_(2, 4);
    V_xx(4, 3) = Q_xu_(4, 0) * k_ux(0, 3) + Q_xx_(3, 4);
    V_xx(4, 4) = Q_xu_(4, 0) * k_ux(0, 4) + Q_xx_(4, 4);

    // Evaluation of Vector V_x
    V_x(0) = Q_x_(0) + Q_xu_(0, 0) * v_u(0);
    V_x(1) = Q_x_(1) + Q_xu_(1, 0) * v_u(0);
    V_x(2) = Q_x_(2) + Q_xu_(2, 0) * v_u(0);
    V_x(3) = Q_x_(3) + Q_xu_(3, 0) * v_u(0);
    V_x(4) = Q_x_(4) + Q_xu_(4, 0) * v_u(0);
  }

  virtual void updateQnx(const Eigen::VectorXd& r_f, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& Fu,
                         const Eigen::MatrixXd& h_u, const Eigen::MatrixXd& k_ux, const Eigen::MatrixXd& k_ex,
                         const Eigen::MatrixXd& k_ue, const Eigen::VectorXd& v_u, const Eigen::VectorXd& v_e) override {
    // Evaluation of Matrix Q_un_
    Q_un_(0, 0) = -Fu(0, 0) * Q_uu_inv_(0, 0);
    Q_un_(0, 1) = -Fu(1, 0) * Q_uu_inv_(0, 0);
    Q_un_(0, 2) = -Fu(2, 0) * Q_uu_inv_(0, 0);
    Q_un_(0, 3) = -Fu(3, 0) * Q_uu_inv_(0, 0);
    Q_un_(0, 4) = -Fu(4, 0) * Q_uu_inv_(0, 0);

    // Evaluation of Matrix Q_nuu_
    Q_nuu_(0, 0) = -Q_uu_(0, 0);
    Q_nuu_(1, 1) = -Q_uu_(1, 1);

    // Evaluation of Matrix Q_vnx_
    Q_vnx_(0, 0) = Fu(0, 0) * k_ux(0, 0) + Fx(0, 0);
    Q_vnx_(0, 1) = Fu(0, 0) * k_ux(0, 1) + Fx(0, 1);
    Q_vnx_(0, 2) = Fu(0, 0) * k_ux(0, 2) + Fx(0, 2);
    Q_vnx_(0, 3) = Fu(0, 0) * k_ux(0, 3) + Fx(0, 3);
    Q_vnx_(0, 4) = Fu(0, 0) * k_ux(0, 4) + Fx(0, 4);
    Q_vnx_(1, 0) = Fu(1, 0) * k_ux(0, 0);
    Q_vnx_(1, 1) = Fu(1, 0) * k_ux(0, 1) + Fx(1, 1);
    Q_vnx_(1, 2) = Fu(1, 0) * k_ux(0, 2);
    Q_vnx_(1, 3) = Fu(1, 0) * k_ux(0, 3) + Fx(1, 3);
    Q_vnx_(1, 4) = Fu(1, 0) * k_ux(0, 4) + Fx(1, 4);
    Q_vnx_(2, 0) = Fu(2, 0) * k_ux(0, 0);
    Q_vnx_(2, 1) = Fu(2, 0) * k_ux(0, 1);
    Q_vnx_(2, 2) = Fu(2, 0) * k_ux(0, 2) + Fx(2, 2);
    Q_vnx_(2, 3) = Fu(2, 0) * k_ux(0, 3) + Fx(2, 3);
    Q_vnx_(2, 4) = Fu(2, 0) * k_ux(0, 4) + Fx(2, 4);
    Q_vnx_(3, 0) = Fu(3, 0) * k_ux(0, 0);
    Q_vnx_(3, 1) = Fu(3, 0) * k_ux(0, 1);
    Q_vnx_(3, 2) = Fu(3, 0) * k_ux(0, 2);
    Q_vnx_(3, 3) = Fu(3, 0) * k_ux(0, 3) + Fx(3, 3);
    Q_vnx_(3, 4) = Fu(3, 0) * k_ux(0, 4) + Fx(3, 4);
    Q_vnx_(4, 0) = Fu(4, 0) * k_ux(0, 0);
    Q_vnx_(4, 1) = Fu(4, 0) * k_ux(0, 1);
    Q_vnx_(4, 2) = Fu(4, 0) * k_ux(0, 2);
    Q_vnx_(4, 3) = Fu(4, 0) * k_ux(0, 3);
    Q_vnx_(4, 4) = Fu(4, 0) * k_ux(0, 4) + Fx(4, 4);

    // Evaluation of Vector Q_vn_
    Q_vn_(0) = Fu(0, 0) * v_u(0) + r_f(0);
    Q_vn_(1) = Fu(1, 0) * v_u(0) + r_f(1);
    Q_vn_(2) = Fu(2, 0) * v_u(0) + r_f(2);
    Q_vn_(3) = Fu(3, 0) * v_u(0) + r_f(3);
    Q_vn_(4) = Fu(4, 0) * v_u(0) + r_f(4);
  }
};

class ParkLateralGeneralIpmEvaluatorTerminal : public IpmEvaluatorTerminal {
 public:
  ParkLateralGeneralIpmEvaluatorTerminal() : IpmEvaluatorTerminal(5, 2, 9, 0) {}
  virtual ~ParkLateralGeneralIpmEvaluatorTerminal() = default;

  virtual void updateLX(const Eigen::VectorXd& l_x, const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& m_x,
                        const Eigen::VectorXd& p, const Eigen::VectorXd& lambda, const Eigen::VectorXd& nu,
                        Eigen::VectorXd& sl_x) const override {
    // Evaluation of Vector sl_x
    sl_x(0) = -p(0);
    sl_x(1) = g_x(0, 1) * lambda(0) + g_x(1, 1) * lambda(1) + g_x(2, 1) * lambda(2) + g_x(3, 1) * lambda(3) +
              g_x(4, 1) * lambda(4) + g_x(5, 1) * lambda(5) + l_x(1) - p(1);
    sl_x(2) = g_x(0, 2) * lambda(0) + g_x(1, 2) * lambda(1) + g_x(2, 2) * lambda(2) + g_x(3, 2) * lambda(3) +
              g_x(4, 2) * lambda(4) + g_x(5, 2) * lambda(5) + l_x(2) - p(2);
    sl_x(3) = g_x(2, 3) * lambda(2) + g_x(3, 3) * lambda(3) + g_x(4, 3) * lambda(4) + g_x(5, 3) * lambda(5) +
              g_x(8, 3) * lambda(8) + l_x(3) - p(3);
    sl_x(4) = g_x(6, 4) * lambda(6) + g_x(7, 4) * lambda(7) - p(4);
  }

  virtual void updateVxx(const Eigen::MatrixXd& l_xx, const Eigen::VectorXd& sl_x, const Eigen::MatrixXd& g_x,
                         Eigen::MatrixXd& V_xx, Eigen::VectorXd& V_x) override {
    // Determine internal variables
    const double g_x_0_1_p2 = g_x(0, 1) * g_x(0, 1);
    const double g_x_0_2_p2 = g_x(0, 2) * g_x(0, 2);
    const double g_x_1_1_p2 = g_x(1, 1) * g_x(1, 1);
    const double g_x_1_2_p2 = g_x(1, 2) * g_x(1, 2);
    const double g_x_2_1_p2 = g_x(2, 1) * g_x(2, 1);
    const double g_x_2_2_p2 = g_x(2, 2) * g_x(2, 2);
    const double g_x_2_3_p2 = g_x(2, 3) * g_x(2, 3);
    const double g_x_3_1_p2 = g_x(3, 1) * g_x(3, 1);
    const double g_x_3_2_p2 = g_x(3, 2) * g_x(3, 2);
    const double g_x_3_3_p2 = g_x(3, 3) * g_x(3, 3);
    const double g_x_4_1_p2 = g_x(4, 1) * g_x(4, 1);
    const double g_x_4_2_p2 = g_x(4, 2) * g_x(4, 2);
    const double g_x_4_3_p2 = g_x(4, 3) * g_x(4, 3);
    const double g_x_5_1_p2 = g_x(5, 1) * g_x(5, 1);
    const double g_x_5_2_p2 = g_x(5, 2) * g_x(5, 2);
    const double g_x_5_3_p2 = g_x(5, 3) * g_x(5, 3);
    const double g_x_6_4_p2 = g_x(6, 4) * g_x(6, 4);
    const double g_x_7_4_p2 = g_x(7, 4) * g_x(7, 4);
    const double g_x_8_3_p2 = g_x(8, 3) * g_x(8, 3);
    const double internal_1 = Sigma_(2) * g_x(2, 1) * g_x(2, 3);
    const double internal_2 = Sigma_(3) * g_x(3, 1) * g_x(3, 3);
    const double internal_3 = Sigma_(4) * g_x(4, 1) * g_x(4, 3);
    const double internal_4 = Sigma_(5) * g_x(5, 1) * g_x(5, 3);
    const double internal_5 = Sigma_(2) * g_x(2, 2) * g_x(2, 3);
    const double internal_6 = Sigma_(3) * g_x(3, 2) * g_x(3, 3);
    const double internal_7 = Sigma_(4) * g_x(4, 2) * g_x(4, 3);
    const double internal_8 = Sigma_(5) * g_x(5, 2) * g_x(5, 3);
    const double internal_0 = Sigma_(0) * g_x(0, 1) * g_x(0, 2) + Sigma_(1) * g_x(1, 1) * g_x(1, 2) +
                              Sigma_(2) * g_x(2, 1) * g_x(2, 2) + Sigma_(3) * g_x(3, 1) * g_x(3, 2) +
                              Sigma_(4) * g_x(4, 1) * g_x(4, 2) + Sigma_(5) * g_x(5, 1) * g_x(5, 2);

    // Evaluation of Matrix V_xx
    V_xx(1, 1) = Sigma_(0) * g_x_0_1_p2 + Sigma_(1) * g_x_1_1_p2 + Sigma_(2) * g_x_2_1_p2 + Sigma_(3) * g_x_3_1_p2 +
                 Sigma_(4) * g_x_4_1_p2 + Sigma_(5) * g_x_5_1_p2 + l_xx(1, 1);
    V_xx(1, 2) = internal_0 + l_xx(1, 2);
    V_xx(1, 3) = internal_1 + internal_2 + internal_3 + internal_4;
    V_xx(2, 1) = internal_0 + l_xx(2, 1);
    V_xx(2, 2) = Sigma_(0) * g_x_0_2_p2 + Sigma_(1) * g_x_1_2_p2 + Sigma_(2) * g_x_2_2_p2 + Sigma_(3) * g_x_3_2_p2 +
                 Sigma_(4) * g_x_4_2_p2 + Sigma_(5) * g_x_5_2_p2 + l_xx(2, 2);
    V_xx(2, 3) = internal_5 + internal_6 + internal_7 + internal_8;
    V_xx(3, 1) = V_xx(1, 3);
    V_xx(3, 2) = V_xx(2, 3);
    V_xx(3, 3) = Sigma_(2) * g_x_2_3_p2 + Sigma_(3) * g_x_3_3_p2 + Sigma_(4) * g_x_4_3_p2 + Sigma_(5) * g_x_5_3_p2 +
                 Sigma_(8) * g_x_8_3_p2 + l_xx(3, 3);
    V_xx(4, 4) = Sigma_(6) * g_x_6_4_p2 + Sigma_(7) * g_x_7_4_p2;

    // Evaluation of Vector V_x
    V_x(0) = sl_x(0);
    V_x(1) = V_(0) * g_x(0, 1) + V_(1) * g_x(1, 1) + V_(2) * g_x(2, 1) + V_(3) * g_x(3, 1) + V_(4) * g_x(4, 1) +
             V_(5) * g_x(5, 1) + sl_x(1);
    V_x(2) = V_(0) * g_x(0, 2) + V_(1) * g_x(1, 2) + V_(2) * g_x(2, 2) + V_(3) * g_x(3, 2) + V_(4) * g_x(4, 2) +
             V_(5) * g_x(5, 2) + sl_x(2);
    V_x(3) =
        V_(2) * g_x(2, 3) + V_(3) * g_x(3, 3) + V_(4) * g_x(4, 3) + V_(5) * g_x(5, 3) + V_(8) * g_x(8, 3) + sl_x(3);
    V_x(4) = V_(6) * g_x(6, 4) + V_(7) * g_x(7, 4) + sl_x(4);
  }
};

}  // namespace gpal::pnc::planning