#pragma once

#include "ocp/ipm_evaluator.h"

namespace gpal::pnc::planning {

template <StageType stage_type, OcpConfig::IntegratorType integrator_type>
class SpeedRiskModelIpmEvaluator : public IpmEvaluator {
 public:
  SpeedRiskModelIpmEvaluator() : IpmEvaluator(3, 8, 15, 0, 0) {}
  virtual ~SpeedRiskModelIpmEvaluator() = default;

  virtual void updateLX(const Eigen::VectorXd& l_x, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& g_x,
                        const Eigen::MatrixXd& h_x, const Eigen::MatrixXd& m_x, const Eigen::VectorXd& p,
                        const Eigen::VectorXd& lambda, const Eigen::VectorXd& eta, const Eigen::VectorXd& nu,
                        Eigen::VectorXd& sl_x) const override {
    // Evaluation of Vector sl_x
    sl_x(0) = Fx(0, 0) * p(0) + g_x(0, 0) * lambda(0) + g_x(1, 0) * lambda(1) + g_x(10, 0) * lambda(10) +
              g_x(14, 0) * lambda(14) + g_x(8, 0) * lambda(8) + g_x(9, 0) * lambda(9) + l_x(0);
    sl_x(1) = Fx(0, 1) * p(0) + Fx(1, 1) * p(1) + g_x(10, 1) * lambda(10) + g_x(11, 1) * lambda(11) +
              g_x(14, 1) * lambda(14) + g_x(2, 1) * lambda(2) + g_x(3, 1) * lambda(3) + l_x(1);
    sl_x(2) = Fx(1, 2) * p(1) + Fx(2, 2) * p(2) + g_x(12, 2) * lambda(12) + g_x(13, 2) * lambda(13) +
              g_x(4, 2) * lambda(4) + g_x(5, 2) * lambda(5) + l_x(2);
  }

  virtual void updateLU(const Eigen::VectorXd& l_u, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& g_u,
                        const Eigen::MatrixXd& h_u, const Eigen::VectorXd& p, const Eigen::VectorXd& lambda,
                        const Eigen::VectorXd& eta, Eigen::VectorXd& sl_u) const override {
    // Evaluation of Vector sl_u
    sl_u(0) = Fu(2, 0) * p(2) + g_u(6, 0) * lambda(6) + g_u(7, 0) * lambda(7) + l_u(0);
    sl_u(1) = g_u(8, 1) * lambda(8) + l_u(1);
    sl_u(2) = g_u(9, 2) * lambda(9) + l_u(2);
    sl_u(3) = g_u(10, 3) * lambda(10) + l_u(3);
    sl_u(4) = g_u(11, 4) * lambda(11) + l_u(4);
    sl_u(5) = g_u(12, 5) * lambda(12) + l_u(5);
    sl_u(6) = g_u(13, 6) * lambda(13) + l_u(6);
    sl_u(7) = g_u(14, 7) * lambda(14) + l_u(7);
  }

  virtual void updateQH(const Eigen::MatrixXd& l_xx, const Eigen::MatrixXd& l_xu, const Eigen::MatrixXd& l_uu,
                        const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::MatrixXd& Fx,
                        const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& V_xx_prev) override {
    // Determine internal variables
    const double Fu_2_0_p2 = Fu(2, 0) * Fu(2, 0);
    const double Fx_0_0_p2 = Fx(0, 0) * Fx(0, 0);
    const double g_u_10_3_p2 = g_u(10, 3) * g_u(10, 3);
    const double g_u_11_4_p2 = g_u(11, 4) * g_u(11, 4);
    const double g_u_12_5_p2 = g_u(12, 5) * g_u(12, 5);
    const double g_u_13_6_p2 = g_u(13, 6) * g_u(13, 6);
    const double g_u_14_7_p2 = g_u(14, 7) * g_u(14, 7);
    const double g_u_6_0_p2 = g_u(6, 0) * g_u(6, 0);
    const double g_u_7_0_p2 = g_u(7, 0) * g_u(7, 0);
    const double g_u_8_1_p2 = g_u(8, 1) * g_u(8, 1);
    const double g_u_9_2_p2 = g_u(9, 2) * g_u(9, 2);
    const double g_x_0_0_p2 = g_x(0, 0) * g_x(0, 0);
    const double g_x_10_0_p2 = g_x(10, 0) * g_x(10, 0);
    const double g_x_10_1_p2 = g_x(10, 1) * g_x(10, 1);
    const double g_x_11_1_p2 = g_x(11, 1) * g_x(11, 1);
    const double g_x_12_2_p2 = g_x(12, 2) * g_x(12, 2);
    const double g_x_13_2_p2 = g_x(13, 2) * g_x(13, 2);
    const double g_x_14_0_p2 = g_x(14, 0) * g_x(14, 0);
    const double g_x_14_1_p2 = g_x(14, 1) * g_x(14, 1);
    const double g_x_1_0_p2 = g_x(1, 0) * g_x(1, 0);
    const double g_x_2_1_p2 = g_x(2, 1) * g_x(2, 1);
    const double g_x_3_1_p2 = g_x(3, 1) * g_x(3, 1);
    const double g_x_4_2_p2 = g_x(4, 2) * g_x(4, 2);
    const double g_x_5_2_p2 = g_x(5, 2) * g_x(5, 2);
    const double g_x_8_0_p2 = g_x(8, 0) * g_x(8, 0);
    const double g_x_9_0_p2 = g_x(9, 0) * g_x(9, 0);
    const double internal_0 = Sigma_(10) * g_x(10, 0) * g_x(10, 1);
    const double internal_1 = Sigma_(14) * g_x(14, 0) * g_x(14, 1);

    // Evaluation of Matrix Q_xx_
    Q_xx_(0, 0) = Fx_0_0_p2 * V_xx_prev(0, 0) + Sigma_(0) * g_x_0_0_p2 + Sigma_(1) * g_x_1_0_p2 +
                  Sigma_(10) * g_x_10_0_p2 + Sigma_(14) * g_x_14_0_p2 + Sigma_(8) * g_x_8_0_p2 +
                  Sigma_(9) * g_x_9_0_p2 + l_xx(0, 0);
    Q_xx_(0, 1) = Fx(0, 0) * Fx(0, 1) * V_xx_prev(0, 0) + Fx(0, 0) * Fx(1, 1) * V_xx_prev(0, 1) + internal_0 +
                  internal_1 + l_xx(0, 1);
    Q_xx_(0, 2) = Fx(0, 0) * Fx(1, 2) * V_xx_prev(0, 1) + Fx(0, 0) * Fx(2, 2) * V_xx_prev(0, 2);
    Q_xx_(1, 0) =
        Fx(0, 0) * (Fx(0, 1) * V_xx_prev(0, 0) + Fx(1, 1) * V_xx_prev(1, 0)) + internal_0 + internal_1 + l_xx(1, 0);
    Q_xx_(1, 1) = Fx(0, 1) * (Fx(0, 1) * V_xx_prev(0, 0) + Fx(1, 1) * V_xx_prev(1, 0)) +
                  Fx(1, 1) * (Fx(0, 1) * V_xx_prev(0, 1) + Fx(1, 1) * V_xx_prev(1, 1)) + Sigma_(10) * g_x_10_1_p2 +
                  Sigma_(11) * g_x_11_1_p2 + Sigma_(14) * g_x_14_1_p2 + Sigma_(2) * g_x_2_1_p2 +
                  Sigma_(3) * g_x_3_1_p2 + l_xx(1, 1);
    Q_xx_(1, 2) = Fx(1, 2) * (Fx(0, 1) * V_xx_prev(0, 1) + Fx(1, 1) * V_xx_prev(1, 1)) +
                  Fx(2, 2) * (Fx(0, 1) * V_xx_prev(0, 2) + Fx(1, 1) * V_xx_prev(1, 2));
    Q_xx_(2, 0) = Fx(0, 0) * (Fx(1, 2) * V_xx_prev(1, 0) + Fx(2, 2) * V_xx_prev(2, 0));
    Q_xx_(2, 1) = Fx(0, 1) * (Fx(1, 2) * V_xx_prev(1, 0) + Fx(2, 2) * V_xx_prev(2, 0)) +
                  Fx(1, 1) * (Fx(1, 2) * V_xx_prev(1, 1) + Fx(2, 2) * V_xx_prev(2, 1));
    Q_xx_(2, 2) = Fx(1, 2) * (Fx(1, 2) * V_xx_prev(1, 1) + Fx(2, 2) * V_xx_prev(2, 1)) +
                  Fx(2, 2) * (Fx(1, 2) * V_xx_prev(1, 2) + Fx(2, 2) * V_xx_prev(2, 2)) + Sigma_(12) * g_x_12_2_p2 +
                  Sigma_(13) * g_x_13_2_p2 + Sigma_(4) * g_x_4_2_p2 + Sigma_(5) * g_x_5_2_p2 + l_xx(2, 2);

    // Evaluation of Matrix Q_xu_
    Q_xu_(0, 0) = Fu(2, 0) * Fx(0, 0) * V_xx_prev(0, 2);
    Q_xu_(0, 1) = Sigma_(8) * g_u(8, 1) * g_x(8, 0);
    Q_xu_(0, 2) = Sigma_(9) * g_u(9, 2) * g_x(9, 0);
    Q_xu_(0, 3) = Sigma_(10) * g_u(10, 3) * g_x(10, 0);
    Q_xu_(0, 7) = Sigma_(14) * g_u(14, 7) * g_x(14, 0);
    Q_xu_(1, 0) = Fu(2, 0) * (Fx(0, 1) * V_xx_prev(0, 2) + Fx(1, 1) * V_xx_prev(1, 2));
    Q_xu_(1, 3) = Sigma_(10) * g_u(10, 3) * g_x(10, 1);
    Q_xu_(1, 4) = Sigma_(11) * g_u(11, 4) * g_x(11, 1);
    Q_xu_(1, 7) = Sigma_(14) * g_u(14, 7) * g_x(14, 1);
    Q_xu_(2, 0) = Fu(2, 0) * (Fx(1, 2) * V_xx_prev(1, 2) + Fx(2, 2) * V_xx_prev(2, 2));
    Q_xu_(2, 5) = Sigma_(12) * g_u(12, 5) * g_x(12, 2);
    Q_xu_(2, 6) = Sigma_(13) * g_u(13, 6) * g_x(13, 2);

    // Evaluation of Matrix Q_uu_
    Q_uu_(0, 0) = Fu_2_0_p2 * V_xx_prev(2, 2) + Sigma_(6) * g_u_6_0_p2 + Sigma_(7) * g_u_7_0_p2 + l_uu(0, 0);
    Q_uu_(1, 1) = Sigma_(8) * g_u_8_1_p2 + l_uu(1, 1);
    Q_uu_(2, 2) = Sigma_(9) * g_u_9_2_p2 + l_uu(2, 2);
    Q_uu_(3, 3) = Sigma_(10) * g_u_10_3_p2 + l_uu(3, 3);
    Q_uu_(4, 4) = Sigma_(11) * g_u_11_4_p2 + l_uu(4, 4);
    Q_uu_(5, 5) = Sigma_(12) * g_u_12_5_p2 + l_uu(5, 5);
    Q_uu_(6, 6) = Sigma_(13) * g_u_13_6_p2 + l_uu(6, 6);
    Q_uu_(7, 7) = Sigma_(14) * g_u_14_7_p2 + l_uu(7, 7);
  }

  virtual void updateQG(const Eigen::VectorXd& sl_x, const Eigen::VectorXd& sl_u, const Eigen::MatrixXd& g_x,
                        const Eigen::MatrixXd& g_u, const Eigen::VectorXd& r_f, const Eigen::MatrixXd& Fx,
                        const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& V_xx_prev,
                        const Eigen::VectorXd& V_x_prev) override {
    // Evaluation of Vector Q_x_
    Q_x_(0) =
        Fx(0, 0) * (V_x_prev(0) + V_xx_prev(0, 0) * r_f(0) + V_xx_prev(0, 1) * r_f(1) + V_xx_prev(0, 2) * r_f(2)) +
        V_(0) * g_x(0, 0) + V_(1) * g_x(1, 0) + V_(10) * g_x(10, 0) + V_(14) * g_x(14, 0) + V_(8) * g_x(8, 0) +
        V_(9) * g_x(9, 0) + sl_x(0);
    Q_x_(1) =
        Fx(0, 1) * (V_x_prev(0) + V_xx_prev(0, 0) * r_f(0) + V_xx_prev(0, 1) * r_f(1) + V_xx_prev(0, 2) * r_f(2)) +
        Fx(1, 1) * (V_x_prev(1) + V_xx_prev(1, 0) * r_f(0) + V_xx_prev(1, 1) * r_f(1) + V_xx_prev(1, 2) * r_f(2)) +
        V_(10) * g_x(10, 1) + V_(11) * g_x(11, 1) + V_(14) * g_x(14, 1) + V_(2) * g_x(2, 1) + V_(3) * g_x(3, 1) +
        sl_x(1);
    Q_x_(2) =
        Fx(1, 2) * (V_x_prev(1) + V_xx_prev(1, 0) * r_f(0) + V_xx_prev(1, 1) * r_f(1) + V_xx_prev(1, 2) * r_f(2)) +
        Fx(2, 2) * (V_x_prev(2) + V_xx_prev(2, 0) * r_f(0) + V_xx_prev(2, 1) * r_f(1) + V_xx_prev(2, 2) * r_f(2)) +
        V_(12) * g_x(12, 2) + V_(13) * g_x(13, 2) + V_(4) * g_x(4, 2) + V_(5) * g_x(5, 2) + sl_x(2);

    // Evaluation of Vector Q_u_
    Q_u_(0) =
        Fu(2, 0) * (V_x_prev(2) + V_xx_prev(2, 0) * r_f(0) + V_xx_prev(2, 1) * r_f(1) + V_xx_prev(2, 2) * r_f(2)) +
        V_(6) * g_u(6, 0) + V_(7) * g_u(7, 0) + sl_u(0);
    Q_u_(1) = V_(8) * g_u(8, 1) + sl_u(1);
    Q_u_(2) = V_(9) * g_u(9, 2) + sl_u(2);
    Q_u_(3) = V_(10) * g_u(10, 3) + sl_u(3);
    Q_u_(4) = V_(11) * g_u(11, 4) + sl_u(4);
    Q_u_(5) = V_(12) * g_u(12, 5) + sl_u(5);
    Q_u_(6) = V_(13) * g_u(13, 6) + sl_u(6);
    Q_u_(7) = V_(14) * g_u(14, 7) + sl_u(7);
  }

  virtual void updateQuuInv() override {
    // Evaluation of Matrix Q_uu_inv_
    Q_uu_inv_(0, 0) = 1.0 / Q_uu_(0, 0);
    Q_uu_inv_(1, 1) = 1.0 / Q_uu_(1, 1);
    Q_uu_inv_(2, 2) = 1.0 / Q_uu_(2, 2);
    Q_uu_inv_(3, 3) = 1.0 / Q_uu_(3, 3);
    Q_uu_inv_(4, 4) = 1.0 / Q_uu_(4, 4);
    Q_uu_inv_(5, 5) = 1.0 / Q_uu_(5, 5);
    Q_uu_inv_(6, 6) = 1.0 / Q_uu_(6, 6);
    Q_uu_inv_(7, 7) = 1.0 / Q_uu_(7, 7);
  }

  virtual void updateKVU(Eigen::MatrixXd& k_ux, Eigen::VectorXd& v_u) override {
    // Evaluation of Matrix k_ux
    k_ux(0, 0) = -Q_uu_inv_(0, 0) * Q_xu_(0, 0);
    k_ux(0, 1) = -Q_uu_inv_(0, 0) * Q_xu_(1, 0);
    k_ux(0, 2) = -Q_uu_inv_(0, 0) * Q_xu_(2, 0);
    k_ux(1, 0) = -Q_uu_inv_(1, 1) * Q_xu_(0, 1);
    k_ux(2, 0) = -Q_uu_inv_(2, 2) * Q_xu_(0, 2);
    k_ux(3, 0) = -Q_uu_inv_(3, 3) * Q_xu_(0, 3);
    k_ux(3, 1) = -Q_uu_inv_(3, 3) * Q_xu_(1, 3);
    k_ux(4, 1) = -Q_uu_inv_(4, 4) * Q_xu_(1, 4);
    k_ux(5, 2) = -Q_uu_inv_(5, 5) * Q_xu_(2, 5);
    k_ux(6, 2) = -Q_uu_inv_(6, 6) * Q_xu_(2, 6);
    k_ux(7, 0) = -Q_uu_inv_(7, 7) * Q_xu_(0, 7);
    k_ux(7, 1) = -Q_uu_inv_(7, 7) * Q_xu_(1, 7);

    // Evaluation of Vector v_u
    v_u(0) = -Q_u_(0) * Q_uu_inv_(0, 0);
    v_u(1) = -Q_u_(1) * Q_uu_inv_(1, 1);
    v_u(2) = -Q_u_(2) * Q_uu_inv_(2, 2);
    v_u(3) = -Q_u_(3) * Q_uu_inv_(3, 3);
    v_u(4) = -Q_u_(4) * Q_uu_inv_(4, 4);
    v_u(5) = -Q_u_(5) * Q_uu_inv_(5, 5);
    v_u(6) = -Q_u_(6) * Q_uu_inv_(6, 6);
    v_u(7) = -Q_u_(7) * Q_uu_inv_(7, 7);
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
    V_xx(0, 0) = Q_xu_(0, 0) * k_ux(0, 0) + Q_xu_(0, 1) * k_ux(1, 0) + Q_xu_(0, 2) * k_ux(2, 0) +
                 Q_xu_(0, 3) * k_ux(3, 0) + Q_xu_(0, 7) * k_ux(7, 0) + Q_xx_(0, 0);
    V_xx(0, 1) = Q_xu_(0, 0) * k_ux(0, 1) + Q_xu_(0, 3) * k_ux(3, 1) + Q_xu_(0, 7) * k_ux(7, 1) + Q_xx_(0, 1);
    V_xx(0, 2) = Q_xu_(0, 0) * k_ux(0, 2) + Q_xx_(0, 2);
    V_xx(1, 0) = Q_xu_(1, 0) * k_ux(0, 0) + Q_xu_(1, 3) * k_ux(3, 0) + Q_xu_(1, 7) * k_ux(7, 0) + Q_xx_(0, 1);
    V_xx(1, 1) = Q_xu_(1, 0) * k_ux(0, 1) + Q_xu_(1, 3) * k_ux(3, 1) + Q_xu_(1, 4) * k_ux(4, 1) +
                 Q_xu_(1, 7) * k_ux(7, 1) + Q_xx_(1, 1);
    V_xx(1, 2) = Q_xu_(1, 0) * k_ux(0, 2) + Q_xx_(1, 2);
    V_xx(2, 0) = Q_xu_(2, 0) * k_ux(0, 0) + Q_xx_(0, 2);
    V_xx(2, 1) = Q_xu_(2, 0) * k_ux(0, 1) + Q_xx_(1, 2);
    V_xx(2, 2) = Q_xu_(2, 0) * k_ux(0, 2) + Q_xu_(2, 5) * k_ux(5, 2) + Q_xu_(2, 6) * k_ux(6, 2) + Q_xx_(2, 2);

    // Evaluation of Vector V_x
    V_x(0) = Q_x_(0) + Q_xu_(0, 0) * v_u(0) + Q_xu_(0, 1) * v_u(1) + Q_xu_(0, 2) * v_u(2) + Q_xu_(0, 3) * v_u(3) +
             Q_xu_(0, 7) * v_u(7);
    V_x(1) = Q_x_(1) + Q_xu_(1, 0) * v_u(0) + Q_xu_(1, 3) * v_u(3) + Q_xu_(1, 4) * v_u(4) + Q_xu_(1, 7) * v_u(7);
    V_x(2) = Q_x_(2) + Q_xu_(2, 0) * v_u(0) + Q_xu_(2, 5) * v_u(5) + Q_xu_(2, 6) * v_u(6);
  }

  virtual void updateQnx(const Eigen::VectorXd& r_f, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& Fu,
                         const Eigen::MatrixXd& h_u, const Eigen::MatrixXd& k_ux, const Eigen::MatrixXd& k_ex,
                         const Eigen::MatrixXd& k_ue, const Eigen::VectorXd& v_u, const Eigen::VectorXd& v_e) override {
    // Evaluation of Matrix Q_un_
    Q_un_(0, 2) = -Fu(2, 0) * Q_uu_inv_(0, 0);

    // Evaluation of Matrix Q_nuu_
    Q_nuu_(0, 0) = -Q_uu_(0, 0);
    Q_nuu_(1, 1) = -Q_uu_(1, 1);
    Q_nuu_(2, 2) = -Q_uu_(2, 2);
    Q_nuu_(3, 3) = -Q_uu_(3, 3);
    Q_nuu_(4, 4) = -Q_uu_(4, 4);
    Q_nuu_(5, 5) = -Q_uu_(5, 5);
    Q_nuu_(6, 6) = -Q_uu_(6, 6);
    Q_nuu_(7, 7) = -Q_uu_(7, 7);

    // Evaluation of Matrix Q_vnx_
    Q_vnx_(0, 0) = Fx(0, 0);
    Q_vnx_(0, 1) = Fx(0, 1);
    Q_vnx_(1, 1) = Fx(1, 1);
    Q_vnx_(1, 2) = Fx(1, 2);
    Q_vnx_(2, 0) = Fu(2, 0) * k_ux(0, 0);
    Q_vnx_(2, 1) = Fu(2, 0) * k_ux(0, 1);
    Q_vnx_(2, 2) = Fu(2, 0) * k_ux(0, 2) + Fx(2, 2);

    // Evaluation of Vector Q_vn_
    Q_vn_(0) = r_f(0);
    Q_vn_(1) = r_f(1);
    Q_vn_(2) = Fu(2, 0) * v_u(0) + r_f(2);
  }
};

template <>
class SpeedRiskModelIpmEvaluator<StageType::INITIAL, OcpConfig::FORWARD_EULER> : public IpmEvaluator {
 public:
  SpeedRiskModelIpmEvaluator() : IpmEvaluator(3, 8, 2, 0, 3) {}
  virtual ~SpeedRiskModelIpmEvaluator() = default;

  virtual void updateLX(const Eigen::VectorXd& l_x, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& g_x,
                        const Eigen::MatrixXd& h_x, const Eigen::MatrixXd& m_x, const Eigen::VectorXd& p,
                        const Eigen::VectorXd& lambda, const Eigen::VectorXd& eta, const Eigen::VectorXd& nu,
                        Eigen::VectorXd& sl_x) const override {
    // Evaluation of Vector sl_x
    sl_x(0) = Fx(0, 0) * p(0) + l_x(0) + m_x(0, 0) * nu(0);
    sl_x(1) = Fx(0, 1) * p(0) + Fx(1, 1) * p(1) + l_x(1) + m_x(1, 1) * nu(1);
    sl_x(2) = Fx(1, 2) * p(1) + Fx(2, 2) * p(2) + l_x(2) + m_x(2, 2) * nu(2);
  }

  virtual void updateLU(const Eigen::VectorXd& l_u, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& g_u,
                        const Eigen::MatrixXd& h_u, const Eigen::VectorXd& p, const Eigen::VectorXd& lambda,
                        const Eigen::VectorXd& eta, Eigen::VectorXd& sl_u) const override {
    // Evaluation of Vector sl_u
    sl_u(0) = Fu(2, 0) * p(2) + g_u(0, 0) * lambda(0) + g_u(1, 0) * lambda(1) + l_u(0);
    sl_u(1) = l_u(1);
    sl_u(2) = l_u(2);
    sl_u(3) = l_u(3);
    sl_u(4) = l_u(4);
    sl_u(5) = l_u(5);
    sl_u(6) = l_u(6);
    sl_u(7) = l_u(7);
  }

  virtual void updateQH(const Eigen::MatrixXd& l_xx, const Eigen::MatrixXd& l_xu, const Eigen::MatrixXd& l_uu,
                        const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::MatrixXd& Fx,
                        const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& V_xx_prev) override {
    // Determine internal variables
    const double Fu_2_0_p2 = Fu(2, 0) * Fu(2, 0);
    const double Fx_0_0_p2 = Fx(0, 0) * Fx(0, 0);
    const double g_u_0_0_p2 = g_u(0, 0) * g_u(0, 0);
    const double g_u_1_0_p2 = g_u(1, 0) * g_u(1, 0);

    // Evaluation of Matrix Q_xx_
    Q_xx_(0, 0) = Fx_0_0_p2 * V_xx_prev(0, 0) + l_xx(0, 0);
    Q_xx_(0, 1) = Fx(0, 0) * Fx(0, 1) * V_xx_prev(0, 0) + Fx(0, 0) * Fx(1, 1) * V_xx_prev(0, 1) + l_xx(0, 1);
    Q_xx_(0, 2) = Fx(0, 0) * Fx(1, 2) * V_xx_prev(0, 1) + Fx(0, 0) * Fx(2, 2) * V_xx_prev(0, 2);
    Q_xx_(1, 0) = Fx(0, 0) * (Fx(0, 1) * V_xx_prev(0, 0) + Fx(1, 1) * V_xx_prev(1, 0)) + l_xx(1, 0);
    Q_xx_(1, 1) = Fx(0, 1) * (Fx(0, 1) * V_xx_prev(0, 0) + Fx(1, 1) * V_xx_prev(1, 0)) +
                  Fx(1, 1) * (Fx(0, 1) * V_xx_prev(0, 1) + Fx(1, 1) * V_xx_prev(1, 1)) + l_xx(1, 1);
    Q_xx_(1, 2) = Fx(1, 2) * (Fx(0, 1) * V_xx_prev(0, 1) + Fx(1, 1) * V_xx_prev(1, 1)) +
                  Fx(2, 2) * (Fx(0, 1) * V_xx_prev(0, 2) + Fx(1, 1) * V_xx_prev(1, 2));
    Q_xx_(2, 0) = Fx(0, 0) * (Fx(1, 2) * V_xx_prev(1, 0) + Fx(2, 2) * V_xx_prev(2, 0));
    Q_xx_(2, 1) = Fx(0, 1) * (Fx(1, 2) * V_xx_prev(1, 0) + Fx(2, 2) * V_xx_prev(2, 0)) +
                  Fx(1, 1) * (Fx(1, 2) * V_xx_prev(1, 1) + Fx(2, 2) * V_xx_prev(2, 1));
    Q_xx_(2, 2) = Fx(1, 2) * (Fx(1, 2) * V_xx_prev(1, 1) + Fx(2, 2) * V_xx_prev(2, 1)) +
                  Fx(2, 2) * (Fx(1, 2) * V_xx_prev(1, 2) + Fx(2, 2) * V_xx_prev(2, 2)) + l_xx(2, 2);

    // Evaluation of Matrix Q_xu_
    Q_xu_(0, 0) = Fu(2, 0) * Fx(0, 0) * V_xx_prev(0, 2);
    Q_xu_(1, 0) = Fu(2, 0) * (Fx(0, 1) * V_xx_prev(0, 2) + Fx(1, 1) * V_xx_prev(1, 2));
    Q_xu_(2, 0) = Fu(2, 0) * (Fx(1, 2) * V_xx_prev(1, 2) + Fx(2, 2) * V_xx_prev(2, 2));

    // Evaluation of Matrix Q_uu_
    Q_uu_(0, 0) = Fu_2_0_p2 * V_xx_prev(2, 2) + Sigma_(0) * g_u_0_0_p2 + Sigma_(1) * g_u_1_0_p2 + l_uu(0, 0);
    Q_uu_(1, 1) = l_uu(1, 1);
    Q_uu_(2, 2) = l_uu(2, 2);
    Q_uu_(3, 3) = l_uu(3, 3);
    Q_uu_(4, 4) = l_uu(4, 4);
    Q_uu_(5, 5) = l_uu(5, 5);
    Q_uu_(6, 6) = l_uu(6, 6);
    Q_uu_(7, 7) = l_uu(7, 7);
  }

  virtual void updateQG(const Eigen::VectorXd& sl_x, const Eigen::VectorXd& sl_u, const Eigen::MatrixXd& g_x,
                        const Eigen::MatrixXd& g_u, const Eigen::VectorXd& r_f, const Eigen::MatrixXd& Fx,
                        const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& V_xx_prev,
                        const Eigen::VectorXd& V_x_prev) override {
    // Evaluation of Vector Q_x_
    Q_x_(0) =
        Fx(0, 0) * (V_x_prev(0) + V_xx_prev(0, 0) * r_f(0) + V_xx_prev(0, 1) * r_f(1) + V_xx_prev(0, 2) * r_f(2)) +
        sl_x(0);
    Q_x_(1) =
        Fx(0, 1) * (V_x_prev(0) + V_xx_prev(0, 0) * r_f(0) + V_xx_prev(0, 1) * r_f(1) + V_xx_prev(0, 2) * r_f(2)) +
        Fx(1, 1) * (V_x_prev(1) + V_xx_prev(1, 0) * r_f(0) + V_xx_prev(1, 1) * r_f(1) + V_xx_prev(1, 2) * r_f(2)) +
        sl_x(1);
    Q_x_(2) =
        Fx(1, 2) * (V_x_prev(1) + V_xx_prev(1, 0) * r_f(0) + V_xx_prev(1, 1) * r_f(1) + V_xx_prev(1, 2) * r_f(2)) +
        Fx(2, 2) * (V_x_prev(2) + V_xx_prev(2, 0) * r_f(0) + V_xx_prev(2, 1) * r_f(1) + V_xx_prev(2, 2) * r_f(2)) +
        sl_x(2);

    // Evaluation of Vector Q_u_
    Q_u_(0) =
        Fu(2, 0) * (V_x_prev(2) + V_xx_prev(2, 0) * r_f(0) + V_xx_prev(2, 1) * r_f(1) + V_xx_prev(2, 2) * r_f(2)) +
        V_(0) * g_u(0, 0) + V_(1) * g_u(1, 0) + sl_u(0);
    Q_u_(1) = sl_u(1);
    Q_u_(2) = sl_u(2);
    Q_u_(3) = sl_u(3);
    Q_u_(4) = sl_u(4);
    Q_u_(5) = sl_u(5);
    Q_u_(6) = sl_u(6);
    Q_u_(7) = sl_u(7);
  }

  virtual void updateQuuInv() override {
    // Evaluation of Matrix Q_uu_inv_
    Q_uu_inv_(0, 0) = 1.0 / Q_uu_(0, 0);
    Q_uu_inv_(1, 1) = 1.0 / Q_uu_(1, 1);
    Q_uu_inv_(2, 2) = 1.0 / Q_uu_(2, 2);
    Q_uu_inv_(3, 3) = 1.0 / Q_uu_(3, 3);
    Q_uu_inv_(4, 4) = 1.0 / Q_uu_(4, 4);
    Q_uu_inv_(5, 5) = 1.0 / Q_uu_(5, 5);
    Q_uu_inv_(6, 6) = 1.0 / Q_uu_(6, 6);
    Q_uu_inv_(7, 7) = 1.0 / Q_uu_(7, 7);
  }

  virtual void updateKVU(Eigen::MatrixXd& k_ux, Eigen::VectorXd& v_u) override {
    // Evaluation of Matrix k_ux
    k_ux(0, 0) = -Q_uu_inv_(0, 0) * Q_xu_(0, 0);
    k_ux(0, 1) = -Q_uu_inv_(0, 0) * Q_xu_(1, 0);
    k_ux(0, 2) = -Q_uu_inv_(0, 0) * Q_xu_(2, 0);

    // Evaluation of Vector v_u
    v_u(0) = -Q_u_(0) * Q_uu_inv_(0, 0);
    v_u(1) = -Q_u_(1) * Q_uu_inv_(1, 1);
    v_u(2) = -Q_u_(2) * Q_uu_inv_(2, 2);
    v_u(3) = -Q_u_(3) * Q_uu_inv_(3, 3);
    v_u(4) = -Q_u_(4) * Q_uu_inv_(4, 4);
    v_u(5) = -Q_u_(5) * Q_uu_inv_(5, 5);
    v_u(6) = -Q_u_(6) * Q_uu_inv_(6, 6);
    v_u(7) = -Q_u_(7) * Q_uu_inv_(7, 7);
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
    V_xx(1, 0) = Q_xu_(1, 0) * k_ux(0, 0) + Q_xx_(0, 1);
    V_xx(1, 1) = Q_xu_(1, 0) * k_ux(0, 1) + Q_xx_(1, 1);
    V_xx(1, 2) = Q_xu_(1, 0) * k_ux(0, 2) + Q_xx_(1, 2);
    V_xx(2, 0) = Q_xu_(2, 0) * k_ux(0, 0) + Q_xx_(0, 2);
    V_xx(2, 1) = Q_xu_(2, 0) * k_ux(0, 1) + Q_xx_(1, 2);
    V_xx(2, 2) = Q_xu_(2, 0) * k_ux(0, 2) + Q_xx_(2, 2);

    // Evaluation of Vector V_x
    V_x(0) = Q_x_(0) + Q_xu_(0, 0) * v_u(0);
    V_x(1) = Q_x_(1) + Q_xu_(1, 0) * v_u(0);
    V_x(2) = Q_x_(2) + Q_xu_(2, 0) * v_u(0);
  }

  virtual void updateQnx(const Eigen::VectorXd& r_f, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& Fu,
                         const Eigen::MatrixXd& h_u, const Eigen::MatrixXd& k_ux, const Eigen::MatrixXd& k_ex,
                         const Eigen::MatrixXd& k_ue, const Eigen::VectorXd& v_u, const Eigen::VectorXd& v_e) override {
    // Evaluation of Matrix Q_un_
    Q_un_(0, 2) = -Fu(2, 0) * Q_uu_inv_(0, 0);

    // Evaluation of Matrix Q_nuu_
    Q_nuu_(0, 0) = -Q_uu_(0, 0);
    Q_nuu_(1, 1) = -Q_uu_(1, 1);
    Q_nuu_(2, 2) = -Q_uu_(2, 2);
    Q_nuu_(3, 3) = -Q_uu_(3, 3);
    Q_nuu_(4, 4) = -Q_uu_(4, 4);
    Q_nuu_(5, 5) = -Q_uu_(5, 5);
    Q_nuu_(6, 6) = -Q_uu_(6, 6);
    Q_nuu_(7, 7) = -Q_uu_(7, 7);

    // Evaluation of Matrix Q_vnx_
    Q_vnx_(0, 0) = Fx(0, 0);
    Q_vnx_(0, 1) = Fx(0, 1);
    Q_vnx_(1, 1) = Fx(1, 1);
    Q_vnx_(1, 2) = Fx(1, 2);
    Q_vnx_(2, 0) = Fu(2, 0) * k_ux(0, 0);
    Q_vnx_(2, 1) = Fu(2, 0) * k_ux(0, 1);
    Q_vnx_(2, 2) = Fu(2, 0) * k_ux(0, 2) + Fx(2, 2);

    // Evaluation of Vector Q_vn_
    Q_vn_(0) = r_f(0);
    Q_vn_(1) = r_f(1);
    Q_vn_(2) = Fu(2, 0) * v_u(0) + r_f(2);
  }
};

template <>
class SpeedRiskModelIpmEvaluator<StageType::NORMINAL, OcpConfig::ERK4> : public IpmEvaluator {
 public:
  SpeedRiskModelIpmEvaluator() : IpmEvaluator(3, 8, 15, 0, 0) {}
  virtual ~SpeedRiskModelIpmEvaluator() = default;

  virtual void updateLX(const Eigen::VectorXd& l_x, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& g_x,
                        const Eigen::MatrixXd& h_x, const Eigen::MatrixXd& m_x, const Eigen::VectorXd& p,
                        const Eigen::VectorXd& lambda, const Eigen::VectorXd& eta, const Eigen::VectorXd& nu,
                        Eigen::VectorXd& sl_x) const override {
    // Evaluation of Vector sl_x
    sl_x(0) = Fx(0, 0) * p(0) + g_x(0, 0) * lambda(0) + g_x(1, 0) * lambda(1) + g_x(10, 0) * lambda(10) +
              g_x(14, 0) * lambda(14) + g_x(8, 0) * lambda(8) + g_x(9, 0) * lambda(9) + l_x(0);
    sl_x(1) = Fx(0, 1) * p(0) + Fx(1, 1) * p(1) + g_x(10, 1) * lambda(10) + g_x(11, 1) * lambda(11) +
              g_x(14, 1) * lambda(14) + g_x(2, 1) * lambda(2) + g_x(3, 1) * lambda(3) + l_x(1);
    sl_x(2) = Fx(0, 2) * p(0) + Fx(1, 2) * p(1) + Fx(2, 2) * p(2) + g_x(12, 2) * lambda(12) + g_x(13, 2) * lambda(13) +
              g_x(4, 2) * lambda(4) + g_x(5, 2) * lambda(5) + l_x(2);
  }

  virtual void updateLU(const Eigen::VectorXd& l_u, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& g_u,
                        const Eigen::MatrixXd& h_u, const Eigen::VectorXd& p, const Eigen::VectorXd& lambda,
                        const Eigen::VectorXd& eta, Eigen::VectorXd& sl_u) const override {
    // Evaluation of Vector sl_u
    sl_u(0) =
        Fu(0, 0) * p(0) + Fu(1, 0) * p(1) + Fu(2, 0) * p(2) + g_u(6, 0) * lambda(6) + g_u(7, 0) * lambda(7) + l_u(0);
    sl_u(1) = g_u(8, 1) * lambda(8) + l_u(1);
    sl_u(2) = g_u(9, 2) * lambda(9) + l_u(2);
    sl_u(3) = g_u(10, 3) * lambda(10) + l_u(3);
    sl_u(4) = g_u(11, 4) * lambda(11) + l_u(4);
    sl_u(5) = g_u(12, 5) * lambda(12) + l_u(5);
    sl_u(6) = g_u(13, 6) * lambda(13) + l_u(6);
    sl_u(7) = g_u(14, 7) * lambda(14) + l_u(7);
  }

  virtual void updateQH(const Eigen::MatrixXd& l_xx, const Eigen::MatrixXd& l_xu, const Eigen::MatrixXd& l_uu,
                        const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::MatrixXd& Fx,
                        const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& V_xx_prev) override {
    // Determine internal variables
    const double Fx_0_0_p2 = Fx(0, 0) * Fx(0, 0);
    const double g_u_10_3_p2 = g_u(10, 3) * g_u(10, 3);
    const double g_u_11_4_p2 = g_u(11, 4) * g_u(11, 4);
    const double g_u_12_5_p2 = g_u(12, 5) * g_u(12, 5);
    const double g_u_13_6_p2 = g_u(13, 6) * g_u(13, 6);
    const double g_u_14_7_p2 = g_u(14, 7) * g_u(14, 7);
    const double g_u_6_0_p2 = g_u(6, 0) * g_u(6, 0);
    const double g_u_7_0_p2 = g_u(7, 0) * g_u(7, 0);
    const double g_u_8_1_p2 = g_u(8, 1) * g_u(8, 1);
    const double g_u_9_2_p2 = g_u(9, 2) * g_u(9, 2);
    const double g_x_0_0_p2 = g_x(0, 0) * g_x(0, 0);
    const double g_x_10_0_p2 = g_x(10, 0) * g_x(10, 0);
    const double g_x_10_1_p2 = g_x(10, 1) * g_x(10, 1);
    const double g_x_11_1_p2 = g_x(11, 1) * g_x(11, 1);
    const double g_x_12_2_p2 = g_x(12, 2) * g_x(12, 2);
    const double g_x_13_2_p2 = g_x(13, 2) * g_x(13, 2);
    const double g_x_14_0_p2 = g_x(14, 0) * g_x(14, 0);
    const double g_x_14_1_p2 = g_x(14, 1) * g_x(14, 1);
    const double g_x_1_0_p2 = g_x(1, 0) * g_x(1, 0);
    const double g_x_2_1_p2 = g_x(2, 1) * g_x(2, 1);
    const double g_x_3_1_p2 = g_x(3, 1) * g_x(3, 1);
    const double g_x_4_2_p2 = g_x(4, 2) * g_x(4, 2);
    const double g_x_5_2_p2 = g_x(5, 2) * g_x(5, 2);
    const double g_x_8_0_p2 = g_x(8, 0) * g_x(8, 0);
    const double g_x_9_0_p2 = g_x(9, 0) * g_x(9, 0);
    const double internal_0 = Sigma_(10) * g_x(10, 0) * g_x(10, 1);
    const double internal_1 = Sigma_(14) * g_x(14, 0) * g_x(14, 1);

    // Evaluation of Matrix Q_xx_
    Q_xx_(0, 0) = Fx_0_0_p2 * V_xx_prev(0, 0) + Sigma_(0) * g_x_0_0_p2 + Sigma_(1) * g_x_1_0_p2 +
                  Sigma_(10) * g_x_10_0_p2 + Sigma_(14) * g_x_14_0_p2 + Sigma_(8) * g_x_8_0_p2 +
                  Sigma_(9) * g_x_9_0_p2 + l_xx(0, 0);
    Q_xx_(0, 1) = Fx(0, 0) * Fx(0, 1) * V_xx_prev(0, 0) + Fx(0, 0) * Fx(1, 1) * V_xx_prev(0, 1) + internal_0 +
                  internal_1 + l_xx(0, 1);
    Q_xx_(0, 2) = Fx(0, 0) * Fx(0, 2) * V_xx_prev(0, 0) + Fx(0, 0) * Fx(1, 2) * V_xx_prev(0, 1) +
                  Fx(0, 0) * Fx(2, 2) * V_xx_prev(0, 2);
    Q_xx_(1, 0) =
        Fx(0, 0) * (Fx(0, 1) * V_xx_prev(0, 0) + Fx(1, 1) * V_xx_prev(1, 0)) + internal_0 + internal_1 + l_xx(1, 0);
    Q_xx_(1, 1) = Fx(0, 1) * (Fx(0, 1) * V_xx_prev(0, 0) + Fx(1, 1) * V_xx_prev(1, 0)) +
                  Fx(1, 1) * (Fx(0, 1) * V_xx_prev(0, 1) + Fx(1, 1) * V_xx_prev(1, 1)) + Sigma_(10) * g_x_10_1_p2 +
                  Sigma_(11) * g_x_11_1_p2 + Sigma_(14) * g_x_14_1_p2 + Sigma_(2) * g_x_2_1_p2 +
                  Sigma_(3) * g_x_3_1_p2 + l_xx(1, 1);
    Q_xx_(1, 2) = Fx(0, 2) * (Fx(0, 1) * V_xx_prev(0, 0) + Fx(1, 1) * V_xx_prev(1, 0)) +
                  Fx(1, 2) * (Fx(0, 1) * V_xx_prev(0, 1) + Fx(1, 1) * V_xx_prev(1, 1)) +
                  Fx(2, 2) * (Fx(0, 1) * V_xx_prev(0, 2) + Fx(1, 1) * V_xx_prev(1, 2));
    Q_xx_(2, 0) = Fx(0, 0) * (Fx(0, 2) * V_xx_prev(0, 0) + Fx(1, 2) * V_xx_prev(1, 0) + Fx(2, 2) * V_xx_prev(2, 0));
    Q_xx_(2, 1) = Fx(0, 1) * (Fx(0, 2) * V_xx_prev(0, 0) + Fx(1, 2) * V_xx_prev(1, 0) + Fx(2, 2) * V_xx_prev(2, 0)) +
                  Fx(1, 1) * (Fx(0, 2) * V_xx_prev(0, 1) + Fx(1, 2) * V_xx_prev(1, 1) + Fx(2, 2) * V_xx_prev(2, 1));
    Q_xx_(2, 2) = Fx(0, 2) * (Fx(0, 2) * V_xx_prev(0, 0) + Fx(1, 2) * V_xx_prev(1, 0) + Fx(2, 2) * V_xx_prev(2, 0)) +
                  Fx(1, 2) * (Fx(0, 2) * V_xx_prev(0, 1) + Fx(1, 2) * V_xx_prev(1, 1) + Fx(2, 2) * V_xx_prev(2, 1)) +
                  Fx(2, 2) * (Fx(0, 2) * V_xx_prev(0, 2) + Fx(1, 2) * V_xx_prev(1, 2) + Fx(2, 2) * V_xx_prev(2, 2)) +
                  Sigma_(12) * g_x_12_2_p2 + Sigma_(13) * g_x_13_2_p2 + Sigma_(4) * g_x_4_2_p2 +
                  Sigma_(5) * g_x_5_2_p2 + l_xx(2, 2);

    // Evaluation of Matrix Q_xu_
    Q_xu_(0, 0) = Fu(0, 0) * Fx(0, 0) * V_xx_prev(0, 0) + Fu(1, 0) * Fx(0, 0) * V_xx_prev(0, 1) +
                  Fu(2, 0) * Fx(0, 0) * V_xx_prev(0, 2);
    Q_xu_(0, 1) = Sigma_(8) * g_u(8, 1) * g_x(8, 0);
    Q_xu_(0, 2) = Sigma_(9) * g_u(9, 2) * g_x(9, 0);
    Q_xu_(0, 3) = Sigma_(10) * g_u(10, 3) * g_x(10, 0);
    Q_xu_(0, 7) = Sigma_(14) * g_u(14, 7) * g_x(14, 0);
    Q_xu_(1, 0) = Fu(0, 0) * (Fx(0, 1) * V_xx_prev(0, 0) + Fx(1, 1) * V_xx_prev(1, 0)) +
                  Fu(1, 0) * (Fx(0, 1) * V_xx_prev(0, 1) + Fx(1, 1) * V_xx_prev(1, 1)) +
                  Fu(2, 0) * (Fx(0, 1) * V_xx_prev(0, 2) + Fx(1, 1) * V_xx_prev(1, 2));
    Q_xu_(1, 3) = Sigma_(10) * g_u(10, 3) * g_x(10, 1);
    Q_xu_(1, 4) = Sigma_(11) * g_u(11, 4) * g_x(11, 1);
    Q_xu_(1, 7) = Sigma_(14) * g_u(14, 7) * g_x(14, 1);
    Q_xu_(2, 0) = Fu(0, 0) * (Fx(0, 2) * V_xx_prev(0, 0) + Fx(1, 2) * V_xx_prev(1, 0) + Fx(2, 2) * V_xx_prev(2, 0)) +
                  Fu(1, 0) * (Fx(0, 2) * V_xx_prev(0, 1) + Fx(1, 2) * V_xx_prev(1, 1) + Fx(2, 2) * V_xx_prev(2, 1)) +
                  Fu(2, 0) * (Fx(0, 2) * V_xx_prev(0, 2) + Fx(1, 2) * V_xx_prev(1, 2) + Fx(2, 2) * V_xx_prev(2, 2));
    Q_xu_(2, 5) = Sigma_(12) * g_u(12, 5) * g_x(12, 2);
    Q_xu_(2, 6) = Sigma_(13) * g_u(13, 6) * g_x(13, 2);

    // Evaluation of Matrix Q_uu_
    Q_uu_(0, 0) = Fu(0, 0) * (Fu(0, 0) * V_xx_prev(0, 0) + Fu(1, 0) * V_xx_prev(1, 0) + Fu(2, 0) * V_xx_prev(2, 0)) +
                  Fu(1, 0) * (Fu(0, 0) * V_xx_prev(0, 1) + Fu(1, 0) * V_xx_prev(1, 1) + Fu(2, 0) * V_xx_prev(2, 1)) +
                  Fu(2, 0) * (Fu(0, 0) * V_xx_prev(0, 2) + Fu(1, 0) * V_xx_prev(1, 2) + Fu(2, 0) * V_xx_prev(2, 2)) +
                  Sigma_(6) * g_u_6_0_p2 + Sigma_(7) * g_u_7_0_p2 + l_uu(0, 0);
    Q_uu_(1, 1) = Sigma_(8) * g_u_8_1_p2 + l_uu(1, 1);
    Q_uu_(2, 2) = Sigma_(9) * g_u_9_2_p2 + l_uu(2, 2);
    Q_uu_(3, 3) = Sigma_(10) * g_u_10_3_p2 + l_uu(3, 3);
    Q_uu_(4, 4) = Sigma_(11) * g_u_11_4_p2 + l_uu(4, 4);
    Q_uu_(5, 5) = Sigma_(12) * g_u_12_5_p2 + l_uu(5, 5);
    Q_uu_(6, 6) = Sigma_(13) * g_u_13_6_p2 + l_uu(6, 6);
    Q_uu_(7, 7) = Sigma_(14) * g_u_14_7_p2 + l_uu(7, 7);
  }

  virtual void updateQG(const Eigen::VectorXd& sl_x, const Eigen::VectorXd& sl_u, const Eigen::MatrixXd& g_x,
                        const Eigen::MatrixXd& g_u, const Eigen::VectorXd& r_f, const Eigen::MatrixXd& Fx,
                        const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& V_xx_prev,
                        const Eigen::VectorXd& V_x_prev) override {
    // Evaluation of Vector Q_x_
    Q_x_(0) =
        Fx(0, 0) * (V_x_prev(0) + V_xx_prev(0, 0) * r_f(0) + V_xx_prev(0, 1) * r_f(1) + V_xx_prev(0, 2) * r_f(2)) +
        V_(0) * g_x(0, 0) + V_(1) * g_x(1, 0) + V_(10) * g_x(10, 0) + V_(14) * g_x(14, 0) + V_(8) * g_x(8, 0) +
        V_(9) * g_x(9, 0) + sl_x(0);
    Q_x_(1) =
        Fx(0, 1) * (V_x_prev(0) + V_xx_prev(0, 0) * r_f(0) + V_xx_prev(0, 1) * r_f(1) + V_xx_prev(0, 2) * r_f(2)) +
        Fx(1, 1) * (V_x_prev(1) + V_xx_prev(1, 0) * r_f(0) + V_xx_prev(1, 1) * r_f(1) + V_xx_prev(1, 2) * r_f(2)) +
        V_(10) * g_x(10, 1) + V_(11) * g_x(11, 1) + V_(14) * g_x(14, 1) + V_(2) * g_x(2, 1) + V_(3) * g_x(3, 1) +
        sl_x(1);
    Q_x_(2) =
        Fx(0, 2) * (V_x_prev(0) + V_xx_prev(0, 0) * r_f(0) + V_xx_prev(0, 1) * r_f(1) + V_xx_prev(0, 2) * r_f(2)) +
        Fx(1, 2) * (V_x_prev(1) + V_xx_prev(1, 0) * r_f(0) + V_xx_prev(1, 1) * r_f(1) + V_xx_prev(1, 2) * r_f(2)) +
        Fx(2, 2) * (V_x_prev(2) + V_xx_prev(2, 0) * r_f(0) + V_xx_prev(2, 1) * r_f(1) + V_xx_prev(2, 2) * r_f(2)) +
        V_(12) * g_x(12, 2) + V_(13) * g_x(13, 2) + V_(4) * g_x(4, 2) + V_(5) * g_x(5, 2) + sl_x(2);

    // Evaluation of Vector Q_u_
    Q_u_(0) =
        Fu(0, 0) * (V_x_prev(0) + V_xx_prev(0, 0) * r_f(0) + V_xx_prev(0, 1) * r_f(1) + V_xx_prev(0, 2) * r_f(2)) +
        Fu(1, 0) * (V_x_prev(1) + V_xx_prev(1, 0) * r_f(0) + V_xx_prev(1, 1) * r_f(1) + V_xx_prev(1, 2) * r_f(2)) +
        Fu(2, 0) * (V_x_prev(2) + V_xx_prev(2, 0) * r_f(0) + V_xx_prev(2, 1) * r_f(1) + V_xx_prev(2, 2) * r_f(2)) +
        V_(6) * g_u(6, 0) + V_(7) * g_u(7, 0) + sl_u(0);
    Q_u_(1) = V_(8) * g_u(8, 1) + sl_u(1);
    Q_u_(2) = V_(9) * g_u(9, 2) + sl_u(2);
    Q_u_(3) = V_(10) * g_u(10, 3) + sl_u(3);
    Q_u_(4) = V_(11) * g_u(11, 4) + sl_u(4);
    Q_u_(5) = V_(12) * g_u(12, 5) + sl_u(5);
    Q_u_(6) = V_(13) * g_u(13, 6) + sl_u(6);
    Q_u_(7) = V_(14) * g_u(14, 7) + sl_u(7);
  }

  virtual void updateQuuInv() override {
    // Evaluation of Matrix Q_uu_inv_
    Q_uu_inv_(0, 0) = 1.0 / Q_uu_(0, 0);
    Q_uu_inv_(1, 1) = 1.0 / Q_uu_(1, 1);
    Q_uu_inv_(2, 2) = 1.0 / Q_uu_(2, 2);
    Q_uu_inv_(3, 3) = 1.0 / Q_uu_(3, 3);
    Q_uu_inv_(4, 4) = 1.0 / Q_uu_(4, 4);
    Q_uu_inv_(5, 5) = 1.0 / Q_uu_(5, 5);
    Q_uu_inv_(6, 6) = 1.0 / Q_uu_(6, 6);
    Q_uu_inv_(7, 7) = 1.0 / Q_uu_(7, 7);
  }

  virtual void updateKVU(Eigen::MatrixXd& k_ux, Eigen::VectorXd& v_u) override {
    // Evaluation of Matrix k_ux
    k_ux(0, 0) = -Q_uu_inv_(0, 0) * Q_xu_(0, 0);
    k_ux(0, 1) = -Q_uu_inv_(0, 0) * Q_xu_(1, 0);
    k_ux(0, 2) = -Q_uu_inv_(0, 0) * Q_xu_(2, 0);
    k_ux(1, 0) = -Q_uu_inv_(1, 1) * Q_xu_(0, 1);
    k_ux(2, 0) = -Q_uu_inv_(2, 2) * Q_xu_(0, 2);
    k_ux(3, 0) = -Q_uu_inv_(3, 3) * Q_xu_(0, 3);
    k_ux(3, 1) = -Q_uu_inv_(3, 3) * Q_xu_(1, 3);
    k_ux(4, 1) = -Q_uu_inv_(4, 4) * Q_xu_(1, 4);
    k_ux(5, 2) = -Q_uu_inv_(5, 5) * Q_xu_(2, 5);
    k_ux(6, 2) = -Q_uu_inv_(6, 6) * Q_xu_(2, 6);
    k_ux(7, 0) = -Q_uu_inv_(7, 7) * Q_xu_(0, 7);
    k_ux(7, 1) = -Q_uu_inv_(7, 7) * Q_xu_(1, 7);

    // Evaluation of Vector v_u
    v_u(0) = -Q_u_(0) * Q_uu_inv_(0, 0);
    v_u(1) = -Q_u_(1) * Q_uu_inv_(1, 1);
    v_u(2) = -Q_u_(2) * Q_uu_inv_(2, 2);
    v_u(3) = -Q_u_(3) * Q_uu_inv_(3, 3);
    v_u(4) = -Q_u_(4) * Q_uu_inv_(4, 4);
    v_u(5) = -Q_u_(5) * Q_uu_inv_(5, 5);
    v_u(6) = -Q_u_(6) * Q_uu_inv_(6, 6);
    v_u(7) = -Q_u_(7) * Q_uu_inv_(7, 7);
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
    V_xx(0, 0) = Q_xu_(0, 0) * k_ux(0, 0) + Q_xu_(0, 1) * k_ux(1, 0) + Q_xu_(0, 2) * k_ux(2, 0) +
                 Q_xu_(0, 3) * k_ux(3, 0) + Q_xu_(0, 7) * k_ux(7, 0) + Q_xx_(0, 0);
    V_xx(0, 1) = Q_xu_(0, 0) * k_ux(0, 1) + Q_xu_(0, 3) * k_ux(3, 1) + Q_xu_(0, 7) * k_ux(7, 1) + Q_xx_(0, 1);
    V_xx(0, 2) = Q_xu_(0, 0) * k_ux(0, 2) + Q_xx_(0, 2);
    V_xx(1, 0) = Q_xu_(1, 0) * k_ux(0, 0) + Q_xu_(1, 3) * k_ux(3, 0) + Q_xu_(1, 7) * k_ux(7, 0) + Q_xx_(0, 1);
    V_xx(1, 1) = Q_xu_(1, 0) * k_ux(0, 1) + Q_xu_(1, 3) * k_ux(3, 1) + Q_xu_(1, 4) * k_ux(4, 1) +
                 Q_xu_(1, 7) * k_ux(7, 1) + Q_xx_(1, 1);
    V_xx(1, 2) = Q_xu_(1, 0) * k_ux(0, 2) + Q_xx_(1, 2);
    V_xx(2, 0) = Q_xu_(2, 0) * k_ux(0, 0) + Q_xx_(0, 2);
    V_xx(2, 1) = Q_xu_(2, 0) * k_ux(0, 1) + Q_xx_(1, 2);
    V_xx(2, 2) = Q_xu_(2, 0) * k_ux(0, 2) + Q_xu_(2, 5) * k_ux(5, 2) + Q_xu_(2, 6) * k_ux(6, 2) + Q_xx_(2, 2);

    // Evaluation of Vector V_x
    V_x(0) = Q_x_(0) + Q_xu_(0, 0) * v_u(0) + Q_xu_(0, 1) * v_u(1) + Q_xu_(0, 2) * v_u(2) + Q_xu_(0, 3) * v_u(3) +
             Q_xu_(0, 7) * v_u(7);
    V_x(1) = Q_x_(1) + Q_xu_(1, 0) * v_u(0) + Q_xu_(1, 3) * v_u(3) + Q_xu_(1, 4) * v_u(4) + Q_xu_(1, 7) * v_u(7);
    V_x(2) = Q_x_(2) + Q_xu_(2, 0) * v_u(0) + Q_xu_(2, 5) * v_u(5) + Q_xu_(2, 6) * v_u(6);
  }

  virtual void updateQnx(const Eigen::VectorXd& r_f, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& Fu,
                         const Eigen::MatrixXd& h_u, const Eigen::MatrixXd& k_ux, const Eigen::MatrixXd& k_ex,
                         const Eigen::MatrixXd& k_ue, const Eigen::VectorXd& v_u, const Eigen::VectorXd& v_e) override {
    // Evaluation of Matrix Q_un_
    Q_un_(0, 0) = -Fu(0, 0) * Q_uu_inv_(0, 0);
    Q_un_(0, 1) = -Fu(1, 0) * Q_uu_inv_(0, 0);
    Q_un_(0, 2) = -Fu(2, 0) * Q_uu_inv_(0, 0);

    // Evaluation of Matrix Q_nuu_
    Q_nuu_(0, 0) = -Q_uu_(0, 0);
    Q_nuu_(1, 1) = -Q_uu_(1, 1);
    Q_nuu_(2, 2) = -Q_uu_(2, 2);
    Q_nuu_(3, 3) = -Q_uu_(3, 3);
    Q_nuu_(4, 4) = -Q_uu_(4, 4);
    Q_nuu_(5, 5) = -Q_uu_(5, 5);
    Q_nuu_(6, 6) = -Q_uu_(6, 6);
    Q_nuu_(7, 7) = -Q_uu_(7, 7);

    // Evaluation of Matrix Q_vnx_
    Q_vnx_(0, 0) = Fu(0, 0) * k_ux(0, 0) + Fx(0, 0);
    Q_vnx_(0, 1) = Fu(0, 0) * k_ux(0, 1) + Fx(0, 1);
    Q_vnx_(0, 2) = Fu(0, 0) * k_ux(0, 2) + Fx(0, 2);
    Q_vnx_(1, 0) = Fu(1, 0) * k_ux(0, 0);
    Q_vnx_(1, 1) = Fu(1, 0) * k_ux(0, 1) + Fx(1, 1);
    Q_vnx_(1, 2) = Fu(1, 0) * k_ux(0, 2) + Fx(1, 2);
    Q_vnx_(2, 0) = Fu(2, 0) * k_ux(0, 0);
    Q_vnx_(2, 1) = Fu(2, 0) * k_ux(0, 1);
    Q_vnx_(2, 2) = Fu(2, 0) * k_ux(0, 2) + Fx(2, 2);

    // Evaluation of Vector Q_vn_
    Q_vn_(0) = Fu(0, 0) * v_u(0) + r_f(0);
    Q_vn_(1) = Fu(1, 0) * v_u(0) + r_f(1);
    Q_vn_(2) = Fu(2, 0) * v_u(0) + r_f(2);
  }
};

template <>
class SpeedRiskModelIpmEvaluator<StageType::INITIAL, OcpConfig::ERK4> : public IpmEvaluator {
 public:
  SpeedRiskModelIpmEvaluator() : IpmEvaluator(3, 8, 2, 0, 3) {}
  virtual ~SpeedRiskModelIpmEvaluator() = default;

  virtual void updateLX(const Eigen::VectorXd& l_x, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& g_x,
                        const Eigen::MatrixXd& h_x, const Eigen::MatrixXd& m_x, const Eigen::VectorXd& p,
                        const Eigen::VectorXd& lambda, const Eigen::VectorXd& eta, const Eigen::VectorXd& nu,
                        Eigen::VectorXd& sl_x) const override {
    // Evaluation of Vector sl_x
    sl_x(0) = Fx(0, 0) * p(0) + l_x(0) + m_x(0, 0) * nu(0);
    sl_x(1) = Fx(0, 1) * p(0) + Fx(1, 1) * p(1) + l_x(1) + m_x(1, 1) * nu(1);
    sl_x(2) = Fx(0, 2) * p(0) + Fx(1, 2) * p(1) + Fx(2, 2) * p(2) + l_x(2) + m_x(2, 2) * nu(2);
  }

  virtual void updateLU(const Eigen::VectorXd& l_u, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& g_u,
                        const Eigen::MatrixXd& h_u, const Eigen::VectorXd& p, const Eigen::VectorXd& lambda,
                        const Eigen::VectorXd& eta, Eigen::VectorXd& sl_u) const override {
    // Evaluation of Vector sl_u
    sl_u(0) =
        Fu(0, 0) * p(0) + Fu(1, 0) * p(1) + Fu(2, 0) * p(2) + g_u(0, 0) * lambda(0) + g_u(1, 0) * lambda(1) + l_u(0);
    sl_u(1) = l_u(1);
    sl_u(2) = l_u(2);
    sl_u(3) = l_u(3);
    sl_u(4) = l_u(4);
    sl_u(5) = l_u(5);
    sl_u(6) = l_u(6);
    sl_u(7) = l_u(7);
  }

  virtual void updateQH(const Eigen::MatrixXd& l_xx, const Eigen::MatrixXd& l_xu, const Eigen::MatrixXd& l_uu,
                        const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::MatrixXd& Fx,
                        const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& V_xx_prev) override {
    // Determine internal variables
    const double Fx_0_0_p2 = Fx(0, 0) * Fx(0, 0);
    const double g_u_0_0_p2 = g_u(0, 0) * g_u(0, 0);
    const double g_u_1_0_p2 = g_u(1, 0) * g_u(1, 0);

    // Evaluation of Matrix Q_xx_
    Q_xx_(0, 0) = Fx_0_0_p2 * V_xx_prev(0, 0) + l_xx(0, 0);
    Q_xx_(0, 1) = Fx(0, 0) * Fx(0, 1) * V_xx_prev(0, 0) + Fx(0, 0) * Fx(1, 1) * V_xx_prev(0, 1) + l_xx(0, 1);
    Q_xx_(0, 2) = Fx(0, 0) * Fx(0, 2) * V_xx_prev(0, 0) + Fx(0, 0) * Fx(1, 2) * V_xx_prev(0, 1) +
                  Fx(0, 0) * Fx(2, 2) * V_xx_prev(0, 2);
    Q_xx_(1, 0) = Fx(0, 0) * (Fx(0, 1) * V_xx_prev(0, 0) + Fx(1, 1) * V_xx_prev(1, 0)) + l_xx(1, 0);
    Q_xx_(1, 1) = Fx(0, 1) * (Fx(0, 1) * V_xx_prev(0, 0) + Fx(1, 1) * V_xx_prev(1, 0)) +
                  Fx(1, 1) * (Fx(0, 1) * V_xx_prev(0, 1) + Fx(1, 1) * V_xx_prev(1, 1)) + l_xx(1, 1);
    Q_xx_(1, 2) = Fx(0, 2) * (Fx(0, 1) * V_xx_prev(0, 0) + Fx(1, 1) * V_xx_prev(1, 0)) +
                  Fx(1, 2) * (Fx(0, 1) * V_xx_prev(0, 1) + Fx(1, 1) * V_xx_prev(1, 1)) +
                  Fx(2, 2) * (Fx(0, 1) * V_xx_prev(0, 2) + Fx(1, 1) * V_xx_prev(1, 2));
    Q_xx_(2, 0) = Fx(0, 0) * (Fx(0, 2) * V_xx_prev(0, 0) + Fx(1, 2) * V_xx_prev(1, 0) + Fx(2, 2) * V_xx_prev(2, 0));
    Q_xx_(2, 1) = Fx(0, 1) * (Fx(0, 2) * V_xx_prev(0, 0) + Fx(1, 2) * V_xx_prev(1, 0) + Fx(2, 2) * V_xx_prev(2, 0)) +
                  Fx(1, 1) * (Fx(0, 2) * V_xx_prev(0, 1) + Fx(1, 2) * V_xx_prev(1, 1) + Fx(2, 2) * V_xx_prev(2, 1));
    Q_xx_(2, 2) = Fx(0, 2) * (Fx(0, 2) * V_xx_prev(0, 0) + Fx(1, 2) * V_xx_prev(1, 0) + Fx(2, 2) * V_xx_prev(2, 0)) +
                  Fx(1, 2) * (Fx(0, 2) * V_xx_prev(0, 1) + Fx(1, 2) * V_xx_prev(1, 1) + Fx(2, 2) * V_xx_prev(2, 1)) +
                  Fx(2, 2) * (Fx(0, 2) * V_xx_prev(0, 2) + Fx(1, 2) * V_xx_prev(1, 2) + Fx(2, 2) * V_xx_prev(2, 2)) +
                  l_xx(2, 2);

    // Evaluation of Matrix Q_xu_
    Q_xu_(0, 0) = Fu(0, 0) * Fx(0, 0) * V_xx_prev(0, 0) + Fu(1, 0) * Fx(0, 0) * V_xx_prev(0, 1) +
                  Fu(2, 0) * Fx(0, 0) * V_xx_prev(0, 2);
    Q_xu_(1, 0) = Fu(0, 0) * (Fx(0, 1) * V_xx_prev(0, 0) + Fx(1, 1) * V_xx_prev(1, 0)) +
                  Fu(1, 0) * (Fx(0, 1) * V_xx_prev(0, 1) + Fx(1, 1) * V_xx_prev(1, 1)) +
                  Fu(2, 0) * (Fx(0, 1) * V_xx_prev(0, 2) + Fx(1, 1) * V_xx_prev(1, 2));
    Q_xu_(2, 0) = Fu(0, 0) * (Fx(0, 2) * V_xx_prev(0, 0) + Fx(1, 2) * V_xx_prev(1, 0) + Fx(2, 2) * V_xx_prev(2, 0)) +
                  Fu(1, 0) * (Fx(0, 2) * V_xx_prev(0, 1) + Fx(1, 2) * V_xx_prev(1, 1) + Fx(2, 2) * V_xx_prev(2, 1)) +
                  Fu(2, 0) * (Fx(0, 2) * V_xx_prev(0, 2) + Fx(1, 2) * V_xx_prev(1, 2) + Fx(2, 2) * V_xx_prev(2, 2));

    // Evaluation of Matrix Q_uu_
    Q_uu_(0, 0) = Fu(0, 0) * (Fu(0, 0) * V_xx_prev(0, 0) + Fu(1, 0) * V_xx_prev(1, 0) + Fu(2, 0) * V_xx_prev(2, 0)) +
                  Fu(1, 0) * (Fu(0, 0) * V_xx_prev(0, 1) + Fu(1, 0) * V_xx_prev(1, 1) + Fu(2, 0) * V_xx_prev(2, 1)) +
                  Fu(2, 0) * (Fu(0, 0) * V_xx_prev(0, 2) + Fu(1, 0) * V_xx_prev(1, 2) + Fu(2, 0) * V_xx_prev(2, 2)) +
                  Sigma_(0) * g_u_0_0_p2 + Sigma_(1) * g_u_1_0_p2 + l_uu(0, 0);
    Q_uu_(1, 1) = l_uu(1, 1);
    Q_uu_(2, 2) = l_uu(2, 2);
    Q_uu_(3, 3) = l_uu(3, 3);
    Q_uu_(4, 4) = l_uu(4, 4);
    Q_uu_(5, 5) = l_uu(5, 5);
    Q_uu_(6, 6) = l_uu(6, 6);
    Q_uu_(7, 7) = l_uu(7, 7);
  }

  virtual void updateQG(const Eigen::VectorXd& sl_x, const Eigen::VectorXd& sl_u, const Eigen::MatrixXd& g_x,
                        const Eigen::MatrixXd& g_u, const Eigen::VectorXd& r_f, const Eigen::MatrixXd& Fx,
                        const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& V_xx_prev,
                        const Eigen::VectorXd& V_x_prev) override {
    // Evaluation of Vector Q_x_
    Q_x_(0) =
        Fx(0, 0) * (V_x_prev(0) + V_xx_prev(0, 0) * r_f(0) + V_xx_prev(0, 1) * r_f(1) + V_xx_prev(0, 2) * r_f(2)) +
        sl_x(0);
    Q_x_(1) =
        Fx(0, 1) * (V_x_prev(0) + V_xx_prev(0, 0) * r_f(0) + V_xx_prev(0, 1) * r_f(1) + V_xx_prev(0, 2) * r_f(2)) +
        Fx(1, 1) * (V_x_prev(1) + V_xx_prev(1, 0) * r_f(0) + V_xx_prev(1, 1) * r_f(1) + V_xx_prev(1, 2) * r_f(2)) +
        sl_x(1);
    Q_x_(2) =
        Fx(0, 2) * (V_x_prev(0) + V_xx_prev(0, 0) * r_f(0) + V_xx_prev(0, 1) * r_f(1) + V_xx_prev(0, 2) * r_f(2)) +
        Fx(1, 2) * (V_x_prev(1) + V_xx_prev(1, 0) * r_f(0) + V_xx_prev(1, 1) * r_f(1) + V_xx_prev(1, 2) * r_f(2)) +
        Fx(2, 2) * (V_x_prev(2) + V_xx_prev(2, 0) * r_f(0) + V_xx_prev(2, 1) * r_f(1) + V_xx_prev(2, 2) * r_f(2)) +
        sl_x(2);

    // Evaluation of Vector Q_u_
    Q_u_(0) =
        Fu(0, 0) * (V_x_prev(0) + V_xx_prev(0, 0) * r_f(0) + V_xx_prev(0, 1) * r_f(1) + V_xx_prev(0, 2) * r_f(2)) +
        Fu(1, 0) * (V_x_prev(1) + V_xx_prev(1, 0) * r_f(0) + V_xx_prev(1, 1) * r_f(1) + V_xx_prev(1, 2) * r_f(2)) +
        Fu(2, 0) * (V_x_prev(2) + V_xx_prev(2, 0) * r_f(0) + V_xx_prev(2, 1) * r_f(1) + V_xx_prev(2, 2) * r_f(2)) +
        V_(0) * g_u(0, 0) + V_(1) * g_u(1, 0) + sl_u(0);
    Q_u_(1) = sl_u(1);
    Q_u_(2) = sl_u(2);
    Q_u_(3) = sl_u(3);
    Q_u_(4) = sl_u(4);
    Q_u_(5) = sl_u(5);
    Q_u_(6) = sl_u(6);
    Q_u_(7) = sl_u(7);
  }

  virtual void updateQuuInv() override {
    // Evaluation of Matrix Q_uu_inv_
    Q_uu_inv_(0, 0) = 1.0 / Q_uu_(0, 0);
    Q_uu_inv_(1, 1) = 1.0 / Q_uu_(1, 1);
    Q_uu_inv_(2, 2) = 1.0 / Q_uu_(2, 2);
    Q_uu_inv_(3, 3) = 1.0 / Q_uu_(3, 3);
    Q_uu_inv_(4, 4) = 1.0 / Q_uu_(4, 4);
    Q_uu_inv_(5, 5) = 1.0 / Q_uu_(5, 5);
    Q_uu_inv_(6, 6) = 1.0 / Q_uu_(6, 6);
    Q_uu_inv_(7, 7) = 1.0 / Q_uu_(7, 7);
  }

  virtual void updateKVU(Eigen::MatrixXd& k_ux, Eigen::VectorXd& v_u) override {
    // Evaluation of Matrix k_ux
    k_ux(0, 0) = -Q_uu_inv_(0, 0) * Q_xu_(0, 0);
    k_ux(0, 1) = -Q_uu_inv_(0, 0) * Q_xu_(1, 0);
    k_ux(0, 2) = -Q_uu_inv_(0, 0) * Q_xu_(2, 0);

    // Evaluation of Vector v_u
    v_u(0) = -Q_u_(0) * Q_uu_inv_(0, 0);
    v_u(1) = -Q_u_(1) * Q_uu_inv_(1, 1);
    v_u(2) = -Q_u_(2) * Q_uu_inv_(2, 2);
    v_u(3) = -Q_u_(3) * Q_uu_inv_(3, 3);
    v_u(4) = -Q_u_(4) * Q_uu_inv_(4, 4);
    v_u(5) = -Q_u_(5) * Q_uu_inv_(5, 5);
    v_u(6) = -Q_u_(6) * Q_uu_inv_(6, 6);
    v_u(7) = -Q_u_(7) * Q_uu_inv_(7, 7);
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
    V_xx(1, 0) = Q_xu_(1, 0) * k_ux(0, 0) + Q_xx_(0, 1);
    V_xx(1, 1) = Q_xu_(1, 0) * k_ux(0, 1) + Q_xx_(1, 1);
    V_xx(1, 2) = Q_xu_(1, 0) * k_ux(0, 2) + Q_xx_(1, 2);
    V_xx(2, 0) = Q_xu_(2, 0) * k_ux(0, 0) + Q_xx_(0, 2);
    V_xx(2, 1) = Q_xu_(2, 0) * k_ux(0, 1) + Q_xx_(1, 2);
    V_xx(2, 2) = Q_xu_(2, 0) * k_ux(0, 2) + Q_xx_(2, 2);

    // Evaluation of Vector V_x
    V_x(0) = Q_x_(0) + Q_xu_(0, 0) * v_u(0);
    V_x(1) = Q_x_(1) + Q_xu_(1, 0) * v_u(0);
    V_x(2) = Q_x_(2) + Q_xu_(2, 0) * v_u(0);
  }

  virtual void updateQnx(const Eigen::VectorXd& r_f, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& Fu,
                         const Eigen::MatrixXd& h_u, const Eigen::MatrixXd& k_ux, const Eigen::MatrixXd& k_ex,
                         const Eigen::MatrixXd& k_ue, const Eigen::VectorXd& v_u, const Eigen::VectorXd& v_e) override {
    // Evaluation of Matrix Q_un_
    Q_un_(0, 0) = -Fu(0, 0) * Q_uu_inv_(0, 0);
    Q_un_(0, 1) = -Fu(1, 0) * Q_uu_inv_(0, 0);
    Q_un_(0, 2) = -Fu(2, 0) * Q_uu_inv_(0, 0);

    // Evaluation of Matrix Q_nuu_
    Q_nuu_(0, 0) = -Q_uu_(0, 0);
    Q_nuu_(1, 1) = -Q_uu_(1, 1);
    Q_nuu_(2, 2) = -Q_uu_(2, 2);
    Q_nuu_(3, 3) = -Q_uu_(3, 3);
    Q_nuu_(4, 4) = -Q_uu_(4, 4);
    Q_nuu_(5, 5) = -Q_uu_(5, 5);
    Q_nuu_(6, 6) = -Q_uu_(6, 6);
    Q_nuu_(7, 7) = -Q_uu_(7, 7);

    // Evaluation of Matrix Q_vnx_
    Q_vnx_(0, 0) = Fu(0, 0) * k_ux(0, 0) + Fx(0, 0);
    Q_vnx_(0, 1) = Fu(0, 0) * k_ux(0, 1) + Fx(0, 1);
    Q_vnx_(0, 2) = Fu(0, 0) * k_ux(0, 2) + Fx(0, 2);
    Q_vnx_(1, 0) = Fu(1, 0) * k_ux(0, 0);
    Q_vnx_(1, 1) = Fu(1, 0) * k_ux(0, 1) + Fx(1, 1);
    Q_vnx_(1, 2) = Fu(1, 0) * k_ux(0, 2) + Fx(1, 2);
    Q_vnx_(2, 0) = Fu(2, 0) * k_ux(0, 0);
    Q_vnx_(2, 1) = Fu(2, 0) * k_ux(0, 1);
    Q_vnx_(2, 2) = Fu(2, 0) * k_ux(0, 2) + Fx(2, 2);

    // Evaluation of Vector Q_vn_
    Q_vn_(0) = Fu(0, 0) * v_u(0) + r_f(0);
    Q_vn_(1) = Fu(1, 0) * v_u(0) + r_f(1);
    Q_vn_(2) = Fu(2, 0) * v_u(0) + r_f(2);
  }
};

class SpeedRiskModelIpmEvaluatorTerminal : public IpmEvaluatorTerminal {
 public:
  SpeedRiskModelIpmEvaluatorTerminal() : IpmEvaluatorTerminal(3, 8, 6, 0) {}
  virtual ~SpeedRiskModelIpmEvaluatorTerminal() = default;

  virtual void updateLX(const Eigen::VectorXd& l_x, const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& m_x,
                        const Eigen::VectorXd& p, const Eigen::VectorXd& lambda, const Eigen::VectorXd& nu,
                        Eigen::VectorXd& sl_x) const override {
    // Evaluation of Vector sl_x
    sl_x(0) = g_x(0, 0) * lambda(0) + g_x(1, 0) * lambda(1) + l_x(0) - p(0);
    sl_x(1) = g_x(2, 1) * lambda(2) + g_x(3, 1) * lambda(3) + l_x(1) - p(1);
    sl_x(2) = g_x(4, 2) * lambda(4) + g_x(5, 2) * lambda(5) + l_x(2) - p(2);
  }

  virtual void updateVxx(const Eigen::MatrixXd& l_xx, const Eigen::VectorXd& sl_x, const Eigen::MatrixXd& g_x,
                         Eigen::MatrixXd& V_xx, Eigen::VectorXd& V_x) override {
    // Determine internal variables
    const double g_x_0_0_p2 = g_x(0, 0) * g_x(0, 0);
    const double g_x_1_0_p2 = g_x(1, 0) * g_x(1, 0);
    const double g_x_2_1_p2 = g_x(2, 1) * g_x(2, 1);
    const double g_x_3_1_p2 = g_x(3, 1) * g_x(3, 1);
    const double g_x_4_2_p2 = g_x(4, 2) * g_x(4, 2);
    const double g_x_5_2_p2 = g_x(5, 2) * g_x(5, 2);

    // Evaluation of Matrix V_xx
    V_xx(0, 0) = Sigma_(0) * g_x_0_0_p2 + Sigma_(1) * g_x_1_0_p2 + l_xx(0, 0);
    V_xx(0, 1) = l_xx(0, 1);
    V_xx(1, 0) = l_xx(1, 0);
    V_xx(1, 1) = Sigma_(2) * g_x_2_1_p2 + Sigma_(3) * g_x_3_1_p2 + l_xx(1, 1);
    V_xx(2, 2) = Sigma_(4) * g_x_4_2_p2 + Sigma_(5) * g_x_5_2_p2 + l_xx(2, 2);

    // Evaluation of Vector V_x
    V_x(0) = V_(0) * g_x(0, 0) + V_(1) * g_x(1, 0) + sl_x(0);
    V_x(1) = V_(2) * g_x(2, 1) + V_(3) * g_x(3, 1) + sl_x(1);
    V_x(2) = V_(4) * g_x(4, 2) + V_(5) * g_x(5, 2) + sl_x(2);
  }
};

}  // namespace gpal::pnc::planning