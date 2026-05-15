#pragma once

#include "ocp/ipm_evaluator.h"

namespace gpal::pnc::planning {

template <StageType stage_type, OcpConfig::IntegratorType integrator_type>
class TrackerModelIpmEvaluator : public IpmEvaluator {
 public:
  TrackerModelIpmEvaluator() : IpmEvaluator(5, 2, 7, 0, 0) {}
  virtual ~TrackerModelIpmEvaluator() = default;

  virtual void updateLX(const Eigen::VectorXd& l_x, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& h_x, const Eigen::MatrixXd& m_x, const Eigen::VectorXd& p, const Eigen::VectorXd& lambda, const Eigen::VectorXd& eta, const Eigen::VectorXd& nu, Eigen::VectorXd& sl_x) const override {
    // Evaluation of Vector sl_x
    sl_x(0) = Fx(0, 0)*p(0);
    sl_x(1) = Fx(0, 1)*p(0) + Fx(1, 1)*p(1) + g_x(5, 1)*lambda(5) + g_x(6, 1)*lambda(6) + l_x(1);
    sl_x(2) = Fx(0, 2)*p(0) + Fx(2, 2)*p(2) + g_x(5, 2)*lambda(5) + g_x(6, 2)*lambda(6) + l_x(2);
    sl_x(3) = Fx(0, 3)*p(0) + Fx(1, 3)*p(1) + Fx(2, 3)*p(2) + Fx(3, 3)*p(3) + g_x(2, 3)*lambda(2) + l_x(3);
    sl_x(4) = Fx(3, 4)*p(3) + Fx(4, 4)*p(4) + g_x(0, 4)*lambda(0) + g_x(1, 4)*lambda(1) + l_x(4);

  }

  virtual void updateLU(const Eigen::VectorXd& l_u, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& g_u, const Eigen::MatrixXd& h_u, const Eigen::VectorXd& p, const Eigen::VectorXd& lambda, const Eigen::VectorXd& eta, Eigen::VectorXd& sl_u) const override {
    // Evaluation of Vector sl_u
    sl_u(0) = Fu(4, 0)*p(4) + g_u(3, 0)*lambda(3) + g_u(4, 0)*lambda(4) + l_u(0);
    sl_u(1) = g_u(5, 1)*lambda(5) + g_u(6, 1)*lambda(6) + l_u(1);

  }

  virtual void updateQH(const Eigen::MatrixXd& l_xx, const Eigen::MatrixXd& l_xu, const Eigen::MatrixXd& l_uu, const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& V_xx_prev) override {
    // Determine internal variables
    const double internal_0 = Fx(0, 1)*V_xx_prev(0, 0);
    const double internal_1 = Fx(0, 0)*V_xx_prev(0, 1);
    const double internal_10 = Sigma_(5)*g_x(5, 1);
    const double internal_11 = Sigma_(6)*g_x(6, 1);
    const double internal_2 = Fx(0, 2)*V_xx_prev(0, 0);
    const double internal_3 = Fx(0, 0)*V_xx_prev(0, 2);
    const double internal_4 = Fx(0, 3)*V_xx_prev(0, 0);
    const double internal_5 = Fx(0, 0)*V_xx_prev(0, 3);
    const double internal_6 = Fx(0, 0)*V_xx_prev(0, 4);
    const double internal_13 = Fx(0, 1)*V_xx_prev(0, 3) + Fx(1, 1)*V_xx_prev(1, 3);
    const double internal_14 = Fx(0, 1)*V_xx_prev(0, 4) + Fx(1, 1)*V_xx_prev(1, 4);
    const double internal_16 = Fx(0, 2)*V_xx_prev(0, 1) + Fx(2, 2)*V_xx_prev(2, 1);
    const double internal_17 = Fx(0, 2)*V_xx_prev(0, 2) + Fx(2, 2)*V_xx_prev(2, 2);
    const double internal_18 = Fx(0, 2)*V_xx_prev(0, 3) + Fx(2, 2)*V_xx_prev(2, 3);
    const double internal_19 = Fx(0, 2)*V_xx_prev(0, 4) + Fx(2, 2)*V_xx_prev(2, 4);
    const double internal_21 = Fx(0, 3)*V_xx_prev(0, 1) + Fx(1, 3)*V_xx_prev(1, 1) + Fx(2, 3)*V_xx_prev(2, 1) + Fx(3, 3)*V_xx_prev(3, 1);
    const double internal_22 = Fx(0, 3)*V_xx_prev(0, 2) + Fx(1, 3)*V_xx_prev(1, 2) + Fx(2, 3)*V_xx_prev(2, 2) + Fx(3, 3)*V_xx_prev(3, 2);
    const double internal_23 = Fx(0, 3)*V_xx_prev(0, 3) + Fx(1, 3)*V_xx_prev(1, 3) + Fx(2, 3)*V_xx_prev(2, 3) + Fx(3, 3)*V_xx_prev(3, 3);
    const double internal_24 = Fx(0, 3)*V_xx_prev(0, 4) + Fx(1, 3)*V_xx_prev(1, 4) + Fx(2, 3)*V_xx_prev(2, 4) + Fx(3, 3)*V_xx_prev(3, 4);
    const double internal_25 = Fx(3, 4)*V_xx_prev(3, 0) + Fx(4, 4)*V_xx_prev(4, 0);
    const double internal_26 = Fx(3, 4)*V_xx_prev(3, 1) + Fx(4, 4)*V_xx_prev(4, 1);
    const double internal_27 = Fx(3, 4)*V_xx_prev(3, 2) + Fx(4, 4)*V_xx_prev(4, 2);
    const double internal_28 = Fx(3, 4)*V_xx_prev(3, 3) + Fx(4, 4)*V_xx_prev(4, 3);
    const double internal_29 = Fx(3, 4)*V_xx_prev(3, 4) + Fx(4, 4)*V_xx_prev(4, 4);
    const double internal_8 = Fx(0, 1)*V_xx_prev(0, 1) + Fx(1, 1)*V_xx_prev(1, 1);
    const double internal_9 = Fx(0, 1)*V_xx_prev(0, 2) + Fx(1, 1)*V_xx_prev(1, 2);
    const double internal_15 = Fx(2, 2)*V_xx_prev(2, 0) + internal_2;
    const double internal_20 = Fx(1, 3)*V_xx_prev(1, 0) + Fx(2, 3)*V_xx_prev(2, 0) + Fx(3, 3)*V_xx_prev(3, 0) + internal_4;
    const double internal_7 = Fx(1, 1)*V_xx_prev(1, 0) + internal_0;
    const double internal_12 = g_x(5, 2)*internal_10 + g_x(6, 2)*internal_11;

    // Evaluation of Matrix Q_xx_
    Q_xx_(0, 0) = pow(Fx(0, 0), 2)*V_xx_prev(0, 0);
    Q_xx_(0, 1) = Fx(0, 0)*internal_0 + Fx(1, 1)*internal_1;
    Q_xx_(0, 2) = Fx(0, 0)*internal_2 + Fx(2, 2)*internal_3;
    Q_xx_(0, 3) = Fx(0, 0)*internal_4 + Fx(1, 3)*internal_1 + Fx(2, 3)*internal_3 + Fx(3, 3)*internal_5;
    Q_xx_(0, 4) = Fx(3, 4)*internal_5 + Fx(4, 4)*internal_6;
    Q_xx_(1, 0) = Fx(0, 0)*internal_7;
    Q_xx_(1, 1) = Fx(0, 1)*internal_7 + Fx(1, 1)*internal_8 + Sigma_(5)*pow(g_x(5, 1), 2) + Sigma_(6)*pow(g_x(6, 1), 2) + l_xx(1, 1);
    Q_xx_(1, 2) = Fx(0, 2)*internal_7 + Fx(2, 2)*internal_9 + internal_12 + l_xx(1, 2);
    Q_xx_(1, 3) = Fx(0, 3)*internal_7 + Fx(1, 3)*internal_8 + Fx(2, 3)*internal_9 + Fx(3, 3)*internal_13;
    Q_xx_(1, 4) = Fx(3, 4)*internal_13 + Fx(4, 4)*internal_14;
    Q_xx_(2, 0) = Fx(0, 0)*internal_15;
    Q_xx_(2, 1) = Fx(0, 1)*internal_15 + Fx(1, 1)*internal_16 + internal_12 + l_xx(2, 1);
    Q_xx_(2, 2) = Fx(0, 2)*internal_15 + Fx(2, 2)*internal_17 + Sigma_(5)*pow(g_x(5, 2), 2) + Sigma_(6)*pow(g_x(6, 2), 2) + l_xx(2, 2);
    Q_xx_(2, 3) = Fx(0, 3)*internal_15 + Fx(1, 3)*internal_16 + Fx(2, 3)*internal_17 + Fx(3, 3)*internal_18;
    Q_xx_(2, 4) = Fx(3, 4)*internal_18 + Fx(4, 4)*internal_19;
    Q_xx_(3, 0) = Fx(0, 0)*internal_20;
    Q_xx_(3, 1) = Fx(0, 1)*internal_20 + Fx(1, 1)*internal_21;
    Q_xx_(3, 2) = Fx(0, 2)*internal_20 + Fx(2, 2)*internal_22;
    Q_xx_(3, 3) = Fx(0, 3)*internal_20 + Fx(1, 3)*internal_21 + Fx(2, 3)*internal_22 + Fx(3, 3)*internal_23 + Sigma_(2)*pow(g_x(2, 3), 2) + l_xx(3, 3);
    Q_xx_(3, 4) = Fx(3, 4)*internal_23 + Fx(4, 4)*internal_24;
    Q_xx_(4, 0) = Fx(0, 0)*internal_25;
    Q_xx_(4, 1) = Fx(0, 1)*internal_25 + Fx(1, 1)*internal_26;
    Q_xx_(4, 2) = Fx(0, 2)*internal_25 + Fx(2, 2)*internal_27;
    Q_xx_(4, 3) = Fx(0, 3)*internal_25 + Fx(1, 3)*internal_26 + Fx(2, 3)*internal_27 + Fx(3, 3)*internal_28;
    Q_xx_(4, 4) = Fx(3, 4)*internal_28 + Fx(4, 4)*internal_29 + Sigma_(0)*pow(g_x(0, 4), 2) + Sigma_(1)*pow(g_x(1, 4), 2) + l_xx(4, 4);

    // Evaluation of Matrix Q_xu_
    Q_xu_(0, 0) = Fu(4, 0)*internal_6;
    Q_xu_(1, 0) = Fu(4, 0)*internal_14;
    Q_xu_(1, 1) = g_u(5, 1)*internal_10 + g_u(6, 1)*internal_11;
    Q_xu_(2, 0) = Fu(4, 0)*internal_19;
    Q_xu_(2, 1) = Sigma_(5)*g_u(5, 1)*g_x(5, 2) + Sigma_(6)*g_u(6, 1)*g_x(6, 2);
    Q_xu_(3, 0) = Fu(4, 0)*internal_24;
    Q_xu_(4, 0) = Fu(4, 0)*internal_29;

    // Evaluation of Matrix Q_uu_
    Q_uu_(0, 0) = pow(Fu(4, 0), 2)*V_xx_prev(4, 4) + Sigma_(3)*pow(g_u(3, 0), 2) + Sigma_(4)*pow(g_u(4, 0), 2) + l_uu(0, 0);
    Q_uu_(1, 1) = Sigma_(5)*pow(g_u(5, 1), 2) + Sigma_(6)*pow(g_u(6, 1), 2) + l_uu(1, 1);

  }

  virtual void updateQG(const Eigen::VectorXd& sl_x, const Eigen::VectorXd& sl_u, const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::VectorXd& r_f, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& V_xx_prev, const Eigen::VectorXd& V_x_prev) override {
    // Determine internal variables
    const double internal_0 = V_x_prev(0) + V_xx_prev(0, 0)*r_f(0) + V_xx_prev(0, 1)*r_f(1) + V_xx_prev(0, 2)*r_f(2) + V_xx_prev(0, 3)*r_f(3) + V_xx_prev(0, 4)*r_f(4);
    const double internal_1 = V_x_prev(1) + V_xx_prev(1, 0)*r_f(0) + V_xx_prev(1, 1)*r_f(1) + V_xx_prev(1, 2)*r_f(2) + V_xx_prev(1, 3)*r_f(3) + V_xx_prev(1, 4)*r_f(4);
    const double internal_2 = V_x_prev(2) + V_xx_prev(2, 0)*r_f(0) + V_xx_prev(2, 1)*r_f(1) + V_xx_prev(2, 2)*r_f(2) + V_xx_prev(2, 3)*r_f(3) + V_xx_prev(2, 4)*r_f(4);
    const double internal_3 = V_x_prev(3) + V_xx_prev(3, 0)*r_f(0) + V_xx_prev(3, 1)*r_f(1) + V_xx_prev(3, 2)*r_f(2) + V_xx_prev(3, 3)*r_f(3) + V_xx_prev(3, 4)*r_f(4);
    const double internal_4 = V_x_prev(4) + V_xx_prev(4, 0)*r_f(0) + V_xx_prev(4, 1)*r_f(1) + V_xx_prev(4, 2)*r_f(2) + V_xx_prev(4, 3)*r_f(3) + V_xx_prev(4, 4)*r_f(4);

    // Evaluation of Vector Q_x_
    Q_x_(0) = Fx(0, 0)*internal_0 + sl_x(0);
    Q_x_(1) = Fx(0, 1)*internal_0 + Fx(1, 1)*internal_1 + V_(5)*g_x(5, 1) + V_(6)*g_x(6, 1) + sl_x(1);
    Q_x_(2) = Fx(0, 2)*internal_0 + Fx(2, 2)*internal_2 + V_(5)*g_x(5, 2) + V_(6)*g_x(6, 2) + sl_x(2);
    Q_x_(3) = Fx(0, 3)*internal_0 + Fx(1, 3)*internal_1 + Fx(2, 3)*internal_2 + Fx(3, 3)*internal_3 + V_(2)*g_x(2, 3) + sl_x(3);
    Q_x_(4) = Fx(3, 4)*internal_3 + Fx(4, 4)*internal_4 + V_(0)*g_x(0, 4) + V_(1)*g_x(1, 4) + sl_x(4);

    // Evaluation of Vector Q_u_
    Q_u_(0) = Fu(4, 0)*internal_4 + V_(3)*g_u(3, 0) + V_(4)*g_u(4, 0) + sl_u(0);
    Q_u_(1) = V_(5)*g_u(5, 1) + V_(6)*g_u(6, 1) + sl_u(1);

  }

  virtual void updateQuuInv() override {
    // Evaluation of Matrix Q_uu_inv_
    Q_uu_inv_(0, 0) = 1.0/Q_uu_(0, 0);
    Q_uu_inv_(1, 1) = 1.0/Q_uu_(1, 1);

  }

  virtual void updateKVU(Eigen::MatrixXd& k_ux, Eigen::VectorXd& v_u) override {
    // Evaluation of Matrix k_ux
    k_ux(0, 0) = -Q_uu_inv_(0, 0)*Q_xu_(0, 0);
    k_ux(0, 1) = -Q_uu_inv_(0, 0)*Q_xu_(1, 0);
    k_ux(0, 2) = -Q_uu_inv_(0, 0)*Q_xu_(2, 0);
    k_ux(0, 3) = -Q_uu_inv_(0, 0)*Q_xu_(3, 0);
    k_ux(0, 4) = -Q_uu_inv_(0, 0)*Q_xu_(4, 0);
    k_ux(1, 1) = -Q_uu_inv_(1, 1)*Q_xu_(1, 1);
    k_ux(1, 2) = -Q_uu_inv_(1, 1)*Q_xu_(2, 1);

    // Evaluation of Vector v_u
    v_u(0) = -Q_u_(0)*Q_uu_inv_(0, 0);
    v_u(1) = -Q_u_(1)*Q_uu_inv_(1, 1);

  }

  virtual void updateQee(const Eigen::MatrixXd& h_u) override {

  }

  virtual void updateQeeInv() override {

  }

  virtual void updateQxe(const Eigen::VectorXd& h, const Eigen::MatrixXd& h_x, const Eigen::MatrixXd& h_u, const Eigen::MatrixXd& k_ux, const Eigen::VectorXd& v_u) override {

  }

  virtual void updateKVE(const Eigen::MatrixXd& h_u, Eigen::MatrixXd& k_ex, Eigen::MatrixXd& k_ue, Eigen::VectorXd& v_e) override {

  }

  virtual void updateVxx(const Eigen::MatrixXd& k_ux, const Eigen::MatrixXd& k_ex, const Eigen::VectorXd& v_u, const Eigen::VectorXd& v_e, Eigen::MatrixXd& V_xx, Eigen::VectorXd& V_x) override {
    // Evaluation of Matrix V_xx
    V_xx(0, 0) = Q_xu_(0, 0)*k_ux(0, 0) + Q_xx_(0, 0);
    V_xx(0, 1) = Q_xu_(0, 0)*k_ux(0, 1) + Q_xx_(0, 1);
    V_xx(0, 2) = Q_xu_(0, 0)*k_ux(0, 2) + Q_xx_(0, 2);
    V_xx(0, 3) = Q_xu_(0, 0)*k_ux(0, 3) + Q_xx_(0, 3);
    V_xx(0, 4) = Q_xu_(0, 0)*k_ux(0, 4) + Q_xx_(0, 4);
    V_xx(1, 0) = Q_xu_(1, 0)*k_ux(0, 0) + Q_xx_(0, 1);
    V_xx(1, 1) = Q_xu_(1, 0)*k_ux(0, 1) + Q_xu_(1, 1)*k_ux(1, 1) + Q_xx_(1, 1);
    V_xx(1, 2) = Q_xu_(1, 0)*k_ux(0, 2) + Q_xu_(1, 1)*k_ux(1, 2) + Q_xx_(1, 2);
    V_xx(1, 3) = Q_xu_(1, 0)*k_ux(0, 3) + Q_xx_(1, 3);
    V_xx(1, 4) = Q_xu_(1, 0)*k_ux(0, 4) + Q_xx_(1, 4);
    V_xx(2, 0) = Q_xu_(2, 0)*k_ux(0, 0) + Q_xx_(0, 2);
    V_xx(2, 1) = Q_xu_(2, 0)*k_ux(0, 1) + Q_xu_(2, 1)*k_ux(1, 1) + Q_xx_(1, 2);
    V_xx(2, 2) = Q_xu_(2, 0)*k_ux(0, 2) + Q_xu_(2, 1)*k_ux(1, 2) + Q_xx_(2, 2);
    V_xx(2, 3) = Q_xu_(2, 0)*k_ux(0, 3) + Q_xx_(2, 3);
    V_xx(2, 4) = Q_xu_(2, 0)*k_ux(0, 4) + Q_xx_(2, 4);
    V_xx(3, 0) = Q_xu_(3, 0)*k_ux(0, 0) + Q_xx_(0, 3);
    V_xx(3, 1) = Q_xu_(3, 0)*k_ux(0, 1) + Q_xx_(1, 3);
    V_xx(3, 2) = Q_xu_(3, 0)*k_ux(0, 2) + Q_xx_(2, 3);
    V_xx(3, 3) = Q_xu_(3, 0)*k_ux(0, 3) + Q_xx_(3, 3);
    V_xx(3, 4) = Q_xu_(3, 0)*k_ux(0, 4) + Q_xx_(3, 4);
    V_xx(4, 0) = Q_xu_(4, 0)*k_ux(0, 0) + Q_xx_(0, 4);
    V_xx(4, 1) = Q_xu_(4, 0)*k_ux(0, 1) + Q_xx_(1, 4);
    V_xx(4, 2) = Q_xu_(4, 0)*k_ux(0, 2) + Q_xx_(2, 4);
    V_xx(4, 3) = Q_xu_(4, 0)*k_ux(0, 3) + Q_xx_(3, 4);
    V_xx(4, 4) = Q_xu_(4, 0)*k_ux(0, 4) + Q_xx_(4, 4);

    // Evaluation of Vector V_x
    V_x(0) = Q_x_(0) + Q_xu_(0, 0)*v_u(0);
    V_x(1) = Q_x_(1) + Q_xu_(1, 0)*v_u(0) + Q_xu_(1, 1)*v_u(1);
    V_x(2) = Q_x_(2) + Q_xu_(2, 0)*v_u(0) + Q_xu_(2, 1)*v_u(1);
    V_x(3) = Q_x_(3) + Q_xu_(3, 0)*v_u(0);
    V_x(4) = Q_x_(4) + Q_xu_(4, 0)*v_u(0);

  }

  virtual void updateQnx(const Eigen::VectorXd& r_f, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& h_u, const Eigen::MatrixXd& k_ux, const Eigen::MatrixXd& k_ex, const Eigen::MatrixXd& k_ue, const Eigen::VectorXd& v_u, const Eigen::VectorXd& v_e) override {
    // Evaluation of Matrix Q_un_
    Q_un_(0, 4) = -Fu(4, 0)*Q_uu_inv_(0, 0);

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
    Q_vnx_(4, 0) = Fu(4, 0)*k_ux(0, 0);
    Q_vnx_(4, 1) = Fu(4, 0)*k_ux(0, 1);
    Q_vnx_(4, 2) = Fu(4, 0)*k_ux(0, 2);
    Q_vnx_(4, 3) = Fu(4, 0)*k_ux(0, 3);
    Q_vnx_(4, 4) = Fu(4, 0)*k_ux(0, 4) + Fx(4, 4);

    // Evaluation of Vector Q_vn_
    Q_vn_(0) = r_f(0);
    Q_vn_(1) = r_f(1);
    Q_vn_(2) = r_f(2);
    Q_vn_(3) = r_f(3);
    Q_vn_(4) = Fu(4, 0)*v_u(0) + r_f(4);

  }

};

template <>
class TrackerModelIpmEvaluator<StageType::INITIAL, OcpConfig::FORWARD_EULER> : public IpmEvaluator {
 public:
  TrackerModelIpmEvaluator() : IpmEvaluator(5, 2, 2, 0, 5) {}
  virtual ~TrackerModelIpmEvaluator() = default;

  virtual void updateLX(const Eigen::VectorXd& l_x, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& h_x, const Eigen::MatrixXd& m_x, const Eigen::VectorXd& p, const Eigen::VectorXd& lambda, const Eigen::VectorXd& eta, const Eigen::VectorXd& nu, Eigen::VectorXd& sl_x) const override {
    // Evaluation of Vector sl_x
    sl_x(0) = Fx(0, 0)*p(0) + m_x(0, 0)*nu(0);
    sl_x(1) = Fx(0, 1)*p(0) + Fx(1, 1)*p(1) + l_x(1) + m_x(1, 1)*nu(1);
    sl_x(2) = Fx(0, 2)*p(0) + Fx(2, 2)*p(2) + l_x(2) + m_x(2, 2)*nu(2);
    sl_x(3) = Fx(0, 3)*p(0) + Fx(1, 3)*p(1) + Fx(2, 3)*p(2) + Fx(3, 3)*p(3) + l_x(3) + m_x(3, 3)*nu(3);
    sl_x(4) = Fx(3, 4)*p(3) + Fx(4, 4)*p(4) + l_x(4) + m_x(4, 4)*nu(4);

  }

  virtual void updateLU(const Eigen::VectorXd& l_u, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& g_u, const Eigen::MatrixXd& h_u, const Eigen::VectorXd& p, const Eigen::VectorXd& lambda, const Eigen::VectorXd& eta, Eigen::VectorXd& sl_u) const override {
    // Evaluation of Vector sl_u
    sl_u(0) = Fu(4, 0)*p(4) + g_u(0, 0)*lambda(0) + g_u(1, 0)*lambda(1) + l_u(0);
    sl_u(1) = l_u(1);

  }

  virtual void updateQH(const Eigen::MatrixXd& l_xx, const Eigen::MatrixXd& l_xu, const Eigen::MatrixXd& l_uu, const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& V_xx_prev) override {
    // Determine internal variables
    const double internal_0 = Fx(0, 1)*V_xx_prev(0, 0);
    const double internal_1 = Fx(0, 0)*V_xx_prev(0, 1);
    const double internal_2 = Fx(0, 2)*V_xx_prev(0, 0);
    const double internal_3 = Fx(0, 0)*V_xx_prev(0, 2);
    const double internal_4 = Fx(0, 3)*V_xx_prev(0, 0);
    const double internal_5 = Fx(0, 0)*V_xx_prev(0, 3);
    const double internal_6 = Fx(0, 0)*V_xx_prev(0, 4);
    const double internal_10 = Fx(0, 1)*V_xx_prev(0, 3) + Fx(1, 1)*V_xx_prev(1, 3);
    const double internal_11 = Fx(0, 1)*V_xx_prev(0, 4) + Fx(1, 1)*V_xx_prev(1, 4);
    const double internal_13 = Fx(0, 2)*V_xx_prev(0, 1) + Fx(2, 2)*V_xx_prev(2, 1);
    const double internal_14 = Fx(0, 2)*V_xx_prev(0, 2) + Fx(2, 2)*V_xx_prev(2, 2);
    const double internal_15 = Fx(0, 2)*V_xx_prev(0, 3) + Fx(2, 2)*V_xx_prev(2, 3);
    const double internal_16 = Fx(0, 2)*V_xx_prev(0, 4) + Fx(2, 2)*V_xx_prev(2, 4);
    const double internal_18 = Fx(0, 3)*V_xx_prev(0, 1) + Fx(1, 3)*V_xx_prev(1, 1) + Fx(2, 3)*V_xx_prev(2, 1) + Fx(3, 3)*V_xx_prev(3, 1);
    const double internal_19 = Fx(0, 3)*V_xx_prev(0, 2) + Fx(1, 3)*V_xx_prev(1, 2) + Fx(2, 3)*V_xx_prev(2, 2) + Fx(3, 3)*V_xx_prev(3, 2);
    const double internal_20 = Fx(0, 3)*V_xx_prev(0, 3) + Fx(1, 3)*V_xx_prev(1, 3) + Fx(2, 3)*V_xx_prev(2, 3) + Fx(3, 3)*V_xx_prev(3, 3);
    const double internal_21 = Fx(0, 3)*V_xx_prev(0, 4) + Fx(1, 3)*V_xx_prev(1, 4) + Fx(2, 3)*V_xx_prev(2, 4) + Fx(3, 3)*V_xx_prev(3, 4);
    const double internal_22 = Fx(3, 4)*V_xx_prev(3, 0) + Fx(4, 4)*V_xx_prev(4, 0);
    const double internal_23 = Fx(3, 4)*V_xx_prev(3, 1) + Fx(4, 4)*V_xx_prev(4, 1);
    const double internal_24 = Fx(3, 4)*V_xx_prev(3, 2) + Fx(4, 4)*V_xx_prev(4, 2);
    const double internal_25 = Fx(3, 4)*V_xx_prev(3, 3) + Fx(4, 4)*V_xx_prev(4, 3);
    const double internal_26 = Fx(3, 4)*V_xx_prev(3, 4) + Fx(4, 4)*V_xx_prev(4, 4);
    const double internal_8 = Fx(0, 1)*V_xx_prev(0, 1) + Fx(1, 1)*V_xx_prev(1, 1);
    const double internal_9 = Fx(0, 1)*V_xx_prev(0, 2) + Fx(1, 1)*V_xx_prev(1, 2);
    const double internal_12 = Fx(2, 2)*V_xx_prev(2, 0) + internal_2;
    const double internal_17 = Fx(1, 3)*V_xx_prev(1, 0) + Fx(2, 3)*V_xx_prev(2, 0) + Fx(3, 3)*V_xx_prev(3, 0) + internal_4;
    const double internal_7 = Fx(1, 1)*V_xx_prev(1, 0) + internal_0;

    // Evaluation of Matrix Q_xx_
    Q_xx_(0, 0) = pow(Fx(0, 0), 2)*V_xx_prev(0, 0);
    Q_xx_(0, 1) = Fx(0, 0)*internal_0 + Fx(1, 1)*internal_1;
    Q_xx_(0, 2) = Fx(0, 0)*internal_2 + Fx(2, 2)*internal_3;
    Q_xx_(0, 3) = Fx(0, 0)*internal_4 + Fx(1, 3)*internal_1 + Fx(2, 3)*internal_3 + Fx(3, 3)*internal_5;
    Q_xx_(0, 4) = Fx(3, 4)*internal_5 + Fx(4, 4)*internal_6;
    Q_xx_(1, 0) = Fx(0, 0)*internal_7;
    Q_xx_(1, 1) = Fx(0, 1)*internal_7 + Fx(1, 1)*internal_8 + l_xx(1, 1);
    Q_xx_(1, 2) = Fx(0, 2)*internal_7 + Fx(2, 2)*internal_9 + l_xx(1, 2);
    Q_xx_(1, 3) = Fx(0, 3)*internal_7 + Fx(1, 3)*internal_8 + Fx(2, 3)*internal_9 + Fx(3, 3)*internal_10;
    Q_xx_(1, 4) = Fx(3, 4)*internal_10 + Fx(4, 4)*internal_11;
    Q_xx_(2, 0) = Fx(0, 0)*internal_12;
    Q_xx_(2, 1) = Fx(0, 1)*internal_12 + Fx(1, 1)*internal_13 + l_xx(2, 1);
    Q_xx_(2, 2) = Fx(0, 2)*internal_12 + Fx(2, 2)*internal_14 + l_xx(2, 2);
    Q_xx_(2, 3) = Fx(0, 3)*internal_12 + Fx(1, 3)*internal_13 + Fx(2, 3)*internal_14 + Fx(3, 3)*internal_15;
    Q_xx_(2, 4) = Fx(3, 4)*internal_15 + Fx(4, 4)*internal_16;
    Q_xx_(3, 0) = Fx(0, 0)*internal_17;
    Q_xx_(3, 1) = Fx(0, 1)*internal_17 + Fx(1, 1)*internal_18;
    Q_xx_(3, 2) = Fx(0, 2)*internal_17 + Fx(2, 2)*internal_19;
    Q_xx_(3, 3) = Fx(0, 3)*internal_17 + Fx(1, 3)*internal_18 + Fx(2, 3)*internal_19 + Fx(3, 3)*internal_20 + l_xx(3, 3);
    Q_xx_(3, 4) = Fx(3, 4)*internal_20 + Fx(4, 4)*internal_21;
    Q_xx_(4, 0) = Fx(0, 0)*internal_22;
    Q_xx_(4, 1) = Fx(0, 1)*internal_22 + Fx(1, 1)*internal_23;
    Q_xx_(4, 2) = Fx(0, 2)*internal_22 + Fx(2, 2)*internal_24;
    Q_xx_(4, 3) = Fx(0, 3)*internal_22 + Fx(1, 3)*internal_23 + Fx(2, 3)*internal_24 + Fx(3, 3)*internal_25;
    Q_xx_(4, 4) = Fx(3, 4)*internal_25 + Fx(4, 4)*internal_26 + l_xx(4, 4);

    // Evaluation of Matrix Q_xu_
    Q_xu_(0, 0) = Fu(4, 0)*internal_6;
    Q_xu_(1, 0) = Fu(4, 0)*internal_11;
    Q_xu_(2, 0) = Fu(4, 0)*internal_16;
    Q_xu_(3, 0) = Fu(4, 0)*internal_21;
    Q_xu_(4, 0) = Fu(4, 0)*internal_26;

    // Evaluation of Matrix Q_uu_
    Q_uu_(0, 0) = pow(Fu(4, 0), 2)*V_xx_prev(4, 4) + Sigma_(0)*pow(g_u(0, 0), 2) + Sigma_(1)*pow(g_u(1, 0), 2) + l_uu(0, 0);
    Q_uu_(1, 1) = l_uu(1, 1);

  }

  virtual void updateQG(const Eigen::VectorXd& sl_x, const Eigen::VectorXd& sl_u, const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::VectorXd& r_f, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& V_xx_prev, const Eigen::VectorXd& V_x_prev) override {
    // Determine internal variables
    const double internal_0 = V_x_prev(0) + V_xx_prev(0, 0)*r_f(0) + V_xx_prev(0, 1)*r_f(1) + V_xx_prev(0, 2)*r_f(2) + V_xx_prev(0, 3)*r_f(3) + V_xx_prev(0, 4)*r_f(4);
    const double internal_1 = V_x_prev(1) + V_xx_prev(1, 0)*r_f(0) + V_xx_prev(1, 1)*r_f(1) + V_xx_prev(1, 2)*r_f(2) + V_xx_prev(1, 3)*r_f(3) + V_xx_prev(1, 4)*r_f(4);
    const double internal_2 = V_x_prev(2) + V_xx_prev(2, 0)*r_f(0) + V_xx_prev(2, 1)*r_f(1) + V_xx_prev(2, 2)*r_f(2) + V_xx_prev(2, 3)*r_f(3) + V_xx_prev(2, 4)*r_f(4);
    const double internal_3 = V_x_prev(3) + V_xx_prev(3, 0)*r_f(0) + V_xx_prev(3, 1)*r_f(1) + V_xx_prev(3, 2)*r_f(2) + V_xx_prev(3, 3)*r_f(3) + V_xx_prev(3, 4)*r_f(4);
    const double internal_4 = V_x_prev(4) + V_xx_prev(4, 0)*r_f(0) + V_xx_prev(4, 1)*r_f(1) + V_xx_prev(4, 2)*r_f(2) + V_xx_prev(4, 3)*r_f(3) + V_xx_prev(4, 4)*r_f(4);

    // Evaluation of Vector Q_x_
    Q_x_(0) = Fx(0, 0)*internal_0 + sl_x(0);
    Q_x_(1) = Fx(0, 1)*internal_0 + Fx(1, 1)*internal_1 + sl_x(1);
    Q_x_(2) = Fx(0, 2)*internal_0 + Fx(2, 2)*internal_2 + sl_x(2);
    Q_x_(3) = Fx(0, 3)*internal_0 + Fx(1, 3)*internal_1 + Fx(2, 3)*internal_2 + Fx(3, 3)*internal_3 + sl_x(3);
    Q_x_(4) = Fx(3, 4)*internal_3 + Fx(4, 4)*internal_4 + sl_x(4);

    // Evaluation of Vector Q_u_
    Q_u_(0) = Fu(4, 0)*internal_4 + V_(0)*g_u(0, 0) + V_(1)*g_u(1, 0) + sl_u(0);
    Q_u_(1) = sl_u(1);

  }

  virtual void updateQuuInv() override {
    // Evaluation of Matrix Q_uu_inv_
    Q_uu_inv_(0, 0) = 1.0/Q_uu_(0, 0);
    Q_uu_inv_(1, 1) = 1.0/Q_uu_(1, 1);

  }

  virtual void updateKVU(Eigen::MatrixXd& k_ux, Eigen::VectorXd& v_u) override {
    // Evaluation of Matrix k_ux
    k_ux(0, 0) = -Q_uu_inv_(0, 0)*Q_xu_(0, 0);
    k_ux(0, 1) = -Q_uu_inv_(0, 0)*Q_xu_(1, 0);
    k_ux(0, 2) = -Q_uu_inv_(0, 0)*Q_xu_(2, 0);
    k_ux(0, 3) = -Q_uu_inv_(0, 0)*Q_xu_(3, 0);
    k_ux(0, 4) = -Q_uu_inv_(0, 0)*Q_xu_(4, 0);

    // Evaluation of Vector v_u
    v_u(0) = -Q_u_(0)*Q_uu_inv_(0, 0);
    v_u(1) = -Q_u_(1)*Q_uu_inv_(1, 1);

  }

  virtual void updateQee(const Eigen::MatrixXd& h_u) override {

  }

  virtual void updateQeeInv() override {

  }

  virtual void updateQxe(const Eigen::VectorXd& h, const Eigen::MatrixXd& h_x, const Eigen::MatrixXd& h_u, const Eigen::MatrixXd& k_ux, const Eigen::VectorXd& v_u) override {

  }

  virtual void updateKVE(const Eigen::MatrixXd& h_u, Eigen::MatrixXd& k_ex, Eigen::MatrixXd& k_ue, Eigen::VectorXd& v_e) override {

  }

  virtual void updateVxx(const Eigen::MatrixXd& k_ux, const Eigen::MatrixXd& k_ex, const Eigen::VectorXd& v_u, const Eigen::VectorXd& v_e, Eigen::MatrixXd& V_xx, Eigen::VectorXd& V_x) override {
    // Evaluation of Matrix V_xx
    V_xx(0, 0) = Q_xu_(0, 0)*k_ux(0, 0) + Q_xx_(0, 0);
    V_xx(0, 1) = Q_xu_(0, 0)*k_ux(0, 1) + Q_xx_(0, 1);
    V_xx(0, 2) = Q_xu_(0, 0)*k_ux(0, 2) + Q_xx_(0, 2);
    V_xx(0, 3) = Q_xu_(0, 0)*k_ux(0, 3) + Q_xx_(0, 3);
    V_xx(0, 4) = Q_xu_(0, 0)*k_ux(0, 4) + Q_xx_(0, 4);
    V_xx(1, 0) = Q_xu_(1, 0)*k_ux(0, 0) + Q_xx_(0, 1);
    V_xx(1, 1) = Q_xu_(1, 0)*k_ux(0, 1) + Q_xx_(1, 1);
    V_xx(1, 2) = Q_xu_(1, 0)*k_ux(0, 2) + Q_xx_(1, 2);
    V_xx(1, 3) = Q_xu_(1, 0)*k_ux(0, 3) + Q_xx_(1, 3);
    V_xx(1, 4) = Q_xu_(1, 0)*k_ux(0, 4) + Q_xx_(1, 4);
    V_xx(2, 0) = Q_xu_(2, 0)*k_ux(0, 0) + Q_xx_(0, 2);
    V_xx(2, 1) = Q_xu_(2, 0)*k_ux(0, 1) + Q_xx_(1, 2);
    V_xx(2, 2) = Q_xu_(2, 0)*k_ux(0, 2) + Q_xx_(2, 2);
    V_xx(2, 3) = Q_xu_(2, 0)*k_ux(0, 3) + Q_xx_(2, 3);
    V_xx(2, 4) = Q_xu_(2, 0)*k_ux(0, 4) + Q_xx_(2, 4);
    V_xx(3, 0) = Q_xu_(3, 0)*k_ux(0, 0) + Q_xx_(0, 3);
    V_xx(3, 1) = Q_xu_(3, 0)*k_ux(0, 1) + Q_xx_(1, 3);
    V_xx(3, 2) = Q_xu_(3, 0)*k_ux(0, 2) + Q_xx_(2, 3);
    V_xx(3, 3) = Q_xu_(3, 0)*k_ux(0, 3) + Q_xx_(3, 3);
    V_xx(3, 4) = Q_xu_(3, 0)*k_ux(0, 4) + Q_xx_(3, 4);
    V_xx(4, 0) = Q_xu_(4, 0)*k_ux(0, 0) + Q_xx_(0, 4);
    V_xx(4, 1) = Q_xu_(4, 0)*k_ux(0, 1) + Q_xx_(1, 4);
    V_xx(4, 2) = Q_xu_(4, 0)*k_ux(0, 2) + Q_xx_(2, 4);
    V_xx(4, 3) = Q_xu_(4, 0)*k_ux(0, 3) + Q_xx_(3, 4);
    V_xx(4, 4) = Q_xu_(4, 0)*k_ux(0, 4) + Q_xx_(4, 4);

    // Evaluation of Vector V_x
    V_x(0) = Q_x_(0) + Q_xu_(0, 0)*v_u(0);
    V_x(1) = Q_x_(1) + Q_xu_(1, 0)*v_u(0);
    V_x(2) = Q_x_(2) + Q_xu_(2, 0)*v_u(0);
    V_x(3) = Q_x_(3) + Q_xu_(3, 0)*v_u(0);
    V_x(4) = Q_x_(4) + Q_xu_(4, 0)*v_u(0);

  }

  virtual void updateQnx(const Eigen::VectorXd& r_f, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& h_u, const Eigen::MatrixXd& k_ux, const Eigen::MatrixXd& k_ex, const Eigen::MatrixXd& k_ue, const Eigen::VectorXd& v_u, const Eigen::VectorXd& v_e) override {
    // Evaluation of Matrix Q_un_
    Q_un_(0, 4) = -Fu(4, 0)*Q_uu_inv_(0, 0);

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
    Q_vnx_(4, 0) = Fu(4, 0)*k_ux(0, 0);
    Q_vnx_(4, 1) = Fu(4, 0)*k_ux(0, 1);
    Q_vnx_(4, 2) = Fu(4, 0)*k_ux(0, 2);
    Q_vnx_(4, 3) = Fu(4, 0)*k_ux(0, 3);
    Q_vnx_(4, 4) = Fu(4, 0)*k_ux(0, 4) + Fx(4, 4);

    // Evaluation of Vector Q_vn_
    Q_vn_(0) = r_f(0);
    Q_vn_(1) = r_f(1);
    Q_vn_(2) = r_f(2);
    Q_vn_(3) = r_f(3);
    Q_vn_(4) = Fu(4, 0)*v_u(0) + r_f(4);

  }

};

template <>
class TrackerModelIpmEvaluator<StageType::NORMINAL, OcpConfig::ERK4> : public IpmEvaluator {
 public:
  TrackerModelIpmEvaluator() : IpmEvaluator(5, 2, 7, 0, 0) {}
  virtual ~TrackerModelIpmEvaluator() = default;

  virtual void updateLX(const Eigen::VectorXd& l_x, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& h_x, const Eigen::MatrixXd& m_x, const Eigen::VectorXd& p, const Eigen::VectorXd& lambda, const Eigen::VectorXd& eta, const Eigen::VectorXd& nu, Eigen::VectorXd& sl_x) const override {
    // Evaluation of Vector sl_x
    sl_x(0) = Fx(0, 0)*p(0);
    sl_x(1) = Fx(0, 1)*p(0) + Fx(1, 1)*p(1) + g_x(5, 1)*lambda(5) + g_x(6, 1)*lambda(6) + l_x(1);
    sl_x(2) = Fx(0, 2)*p(0) + Fx(2, 2)*p(2) + g_x(5, 2)*lambda(5) + g_x(6, 2)*lambda(6) + l_x(2);
    sl_x(3) = Fx(0, 3)*p(0) + Fx(1, 3)*p(1) + Fx(2, 3)*p(2) + Fx(3, 3)*p(3) + g_x(2, 3)*lambda(2) + l_x(3);
    sl_x(4) = Fx(0, 4)*p(0) + Fx(1, 4)*p(1) + Fx(2, 4)*p(2) + Fx(3, 4)*p(3) + Fx(4, 4)*p(4) + g_x(0, 4)*lambda(0) + g_x(1, 4)*lambda(1) + l_x(4);

  }

  virtual void updateLU(const Eigen::VectorXd& l_u, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& g_u, const Eigen::MatrixXd& h_u, const Eigen::VectorXd& p, const Eigen::VectorXd& lambda, const Eigen::VectorXd& eta, Eigen::VectorXd& sl_u) const override {
    // Evaluation of Vector sl_u
    sl_u(0) = Fu(0, 0)*p(0) + Fu(1, 0)*p(1) + Fu(2, 0)*p(2) + Fu(3, 0)*p(3) + Fu(4, 0)*p(4) + g_u(3, 0)*lambda(3) + g_u(4, 0)*lambda(4) + l_u(0);
    sl_u(1) = g_u(5, 1)*lambda(5) + g_u(6, 1)*lambda(6) + l_u(1);

  }

  virtual void updateQH(const Eigen::MatrixXd& l_xx, const Eigen::MatrixXd& l_xu, const Eigen::MatrixXd& l_uu, const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& V_xx_prev) override {
    // Determine internal variables
    const double internal_0 = Fx(0, 1)*V_xx_prev(0, 0);
    const double internal_1 = Fx(0, 0)*V_xx_prev(0, 1);
    const double internal_11 = Sigma_(5)*g_x(5, 1);
    const double internal_12 = Sigma_(6)*g_x(6, 1);
    const double internal_2 = Fx(0, 2)*V_xx_prev(0, 0);
    const double internal_3 = Fx(0, 0)*V_xx_prev(0, 2);
    const double internal_31 = Fu(0, 0)*V_xx_prev(0, 0);
    const double internal_4 = Fx(0, 3)*V_xx_prev(0, 0);
    const double internal_5 = Fx(0, 0)*V_xx_prev(0, 3);
    const double internal_6 = Fx(0, 4)*V_xx_prev(0, 0);
    const double internal_7 = Fx(0, 0)*V_xx_prev(0, 4);
    const double internal_10 = Fx(0, 1)*V_xx_prev(0, 2) + Fx(1, 1)*V_xx_prev(1, 2);
    const double internal_14 = Fx(0, 1)*V_xx_prev(0, 3) + Fx(1, 1)*V_xx_prev(1, 3);
    const double internal_15 = Fx(0, 1)*V_xx_prev(0, 4) + Fx(1, 1)*V_xx_prev(1, 4);
    const double internal_17 = Fx(0, 2)*V_xx_prev(0, 1) + Fx(2, 2)*V_xx_prev(2, 1);
    const double internal_18 = Fx(0, 2)*V_xx_prev(0, 2) + Fx(2, 2)*V_xx_prev(2, 2);
    const double internal_19 = Fx(0, 2)*V_xx_prev(0, 3) + Fx(2, 2)*V_xx_prev(2, 3);
    const double internal_20 = Fx(0, 2)*V_xx_prev(0, 4) + Fx(2, 2)*V_xx_prev(2, 4);
    const double internal_22 = Fx(0, 3)*V_xx_prev(0, 1) + Fx(1, 3)*V_xx_prev(1, 1) + Fx(2, 3)*V_xx_prev(2, 1) + Fx(3, 3)*V_xx_prev(3, 1);
    const double internal_23 = Fx(0, 3)*V_xx_prev(0, 2) + Fx(1, 3)*V_xx_prev(1, 2) + Fx(2, 3)*V_xx_prev(2, 2) + Fx(3, 3)*V_xx_prev(3, 2);
    const double internal_24 = Fx(0, 3)*V_xx_prev(0, 3) + Fx(1, 3)*V_xx_prev(1, 3) + Fx(2, 3)*V_xx_prev(2, 3) + Fx(3, 3)*V_xx_prev(3, 3);
    const double internal_25 = Fx(0, 3)*V_xx_prev(0, 4) + Fx(1, 3)*V_xx_prev(1, 4) + Fx(2, 3)*V_xx_prev(2, 4) + Fx(3, 3)*V_xx_prev(3, 4);
    const double internal_27 = Fx(0, 4)*V_xx_prev(0, 1) + Fx(1, 4)*V_xx_prev(1, 1) + Fx(2, 4)*V_xx_prev(2, 1) + Fx(3, 4)*V_xx_prev(3, 1) + Fx(4, 4)*V_xx_prev(4, 1);
    const double internal_28 = Fx(0, 4)*V_xx_prev(0, 2) + Fx(1, 4)*V_xx_prev(1, 2) + Fx(2, 4)*V_xx_prev(2, 2) + Fx(3, 4)*V_xx_prev(3, 2) + Fx(4, 4)*V_xx_prev(4, 2);
    const double internal_29 = Fx(0, 4)*V_xx_prev(0, 3) + Fx(1, 4)*V_xx_prev(1, 3) + Fx(2, 4)*V_xx_prev(2, 3) + Fx(3, 4)*V_xx_prev(3, 3) + Fx(4, 4)*V_xx_prev(4, 3);
    const double internal_30 = Fx(0, 4)*V_xx_prev(0, 4) + Fx(1, 4)*V_xx_prev(1, 4) + Fx(2, 4)*V_xx_prev(2, 4) + Fx(3, 4)*V_xx_prev(3, 4) + Fx(4, 4)*V_xx_prev(4, 4);
    const double internal_9 = Fx(0, 1)*V_xx_prev(0, 1) + Fx(1, 1)*V_xx_prev(1, 1);
    const double internal_16 = Fx(2, 2)*V_xx_prev(2, 0) + internal_2;
    const double internal_21 = Fx(1, 3)*V_xx_prev(1, 0) + Fx(2, 3)*V_xx_prev(2, 0) + Fx(3, 3)*V_xx_prev(3, 0) + internal_4;
    const double internal_26 = Fx(1, 4)*V_xx_prev(1, 0) + Fx(2, 4)*V_xx_prev(2, 0) + Fx(3, 4)*V_xx_prev(3, 0) + Fx(4, 4)*V_xx_prev(4, 0) + internal_6;
    const double internal_8 = Fx(1, 1)*V_xx_prev(1, 0) + internal_0;
    const double internal_13 = g_x(5, 2)*internal_11 + g_x(6, 2)*internal_12;

    // Evaluation of Matrix Q_xx_
    Q_xx_(0, 0) = pow(Fx(0, 0), 2)*V_xx_prev(0, 0);
    Q_xx_(0, 1) = Fx(0, 0)*internal_0 + Fx(1, 1)*internal_1;
    Q_xx_(0, 2) = Fx(0, 0)*internal_2 + Fx(2, 2)*internal_3;
    Q_xx_(0, 3) = Fx(0, 0)*internal_4 + Fx(1, 3)*internal_1 + Fx(2, 3)*internal_3 + Fx(3, 3)*internal_5;
    Q_xx_(0, 4) = Fx(0, 0)*internal_6 + Fx(1, 4)*internal_1 + Fx(2, 4)*internal_3 + Fx(3, 4)*internal_5 + Fx(4, 4)*internal_7;
    Q_xx_(1, 0) = Fx(0, 0)*internal_8;
    Q_xx_(1, 1) = Fx(0, 1)*internal_8 + Fx(1, 1)*internal_9 + Sigma_(5)*pow(g_x(5, 1), 2) + Sigma_(6)*pow(g_x(6, 1), 2) + l_xx(1, 1);
    Q_xx_(1, 2) = Fx(0, 2)*internal_8 + Fx(2, 2)*internal_10 + internal_13 + l_xx(1, 2);
    Q_xx_(1, 3) = Fx(0, 3)*internal_8 + Fx(1, 3)*internal_9 + Fx(2, 3)*internal_10 + Fx(3, 3)*internal_14;
    Q_xx_(1, 4) = Fx(0, 4)*internal_8 + Fx(1, 4)*internal_9 + Fx(2, 4)*internal_10 + Fx(3, 4)*internal_14 + Fx(4, 4)*internal_15;
    Q_xx_(2, 0) = Fx(0, 0)*internal_16;
    Q_xx_(2, 1) = Fx(0, 1)*internal_16 + Fx(1, 1)*internal_17 + internal_13 + l_xx(2, 1);
    Q_xx_(2, 2) = Fx(0, 2)*internal_16 + Fx(2, 2)*internal_18 + Sigma_(5)*pow(g_x(5, 2), 2) + Sigma_(6)*pow(g_x(6, 2), 2) + l_xx(2, 2);
    Q_xx_(2, 3) = Fx(0, 3)*internal_16 + Fx(1, 3)*internal_17 + Fx(2, 3)*internal_18 + Fx(3, 3)*internal_19;
    Q_xx_(2, 4) = Fx(0, 4)*internal_16 + Fx(1, 4)*internal_17 + Fx(2, 4)*internal_18 + Fx(3, 4)*internal_19 + Fx(4, 4)*internal_20;
    Q_xx_(3, 0) = Fx(0, 0)*internal_21;
    Q_xx_(3, 1) = Fx(0, 1)*internal_21 + Fx(1, 1)*internal_22;
    Q_xx_(3, 2) = Fx(0, 2)*internal_21 + Fx(2, 2)*internal_23;
    Q_xx_(3, 3) = Fx(0, 3)*internal_21 + Fx(1, 3)*internal_22 + Fx(2, 3)*internal_23 + Fx(3, 3)*internal_24 + Sigma_(2)*pow(g_x(2, 3), 2) + l_xx(3, 3);
    Q_xx_(3, 4) = Fx(0, 4)*internal_21 + Fx(1, 4)*internal_22 + Fx(2, 4)*internal_23 + Fx(3, 4)*internal_24 + Fx(4, 4)*internal_25;
    Q_xx_(4, 0) = Fx(0, 0)*internal_26;
    Q_xx_(4, 1) = Fx(0, 1)*internal_26 + Fx(1, 1)*internal_27;
    Q_xx_(4, 2) = Fx(0, 2)*internal_26 + Fx(2, 2)*internal_28;
    Q_xx_(4, 3) = Fx(0, 3)*internal_26 + Fx(1, 3)*internal_27 + Fx(2, 3)*internal_28 + Fx(3, 3)*internal_29;
    Q_xx_(4, 4) = Fx(0, 4)*internal_26 + Fx(1, 4)*internal_27 + Fx(2, 4)*internal_28 + Fx(3, 4)*internal_29 + Fx(4, 4)*internal_30 + Sigma_(0)*pow(g_x(0, 4), 2) + Sigma_(1)*pow(g_x(1, 4), 2) + l_xx(4, 4);

    // Evaluation of Matrix Q_xu_
    Q_xu_(0, 0) = Fu(1, 0)*internal_1 + Fu(2, 0)*internal_3 + Fu(3, 0)*internal_5 + Fu(4, 0)*internal_7 + Fx(0, 0)*internal_31;
    Q_xu_(1, 0) = Fu(0, 0)*internal_8 + Fu(1, 0)*internal_9 + Fu(2, 0)*internal_10 + Fu(3, 0)*internal_14 + Fu(4, 0)*internal_15;
    Q_xu_(1, 1) = g_u(5, 1)*internal_11 + g_u(6, 1)*internal_12;
    Q_xu_(2, 0) = Fu(0, 0)*internal_16 + Fu(1, 0)*internal_17 + Fu(2, 0)*internal_18 + Fu(3, 0)*internal_19 + Fu(4, 0)*internal_20;
    Q_xu_(2, 1) = Sigma_(5)*g_u(5, 1)*g_x(5, 2) + Sigma_(6)*g_u(6, 1)*g_x(6, 2);
    Q_xu_(3, 0) = Fu(0, 0)*internal_21 + Fu(1, 0)*internal_22 + Fu(2, 0)*internal_23 + Fu(3, 0)*internal_24 + Fu(4, 0)*internal_25;
    Q_xu_(4, 0) = Fu(0, 0)*internal_26 + Fu(1, 0)*internal_27 + Fu(2, 0)*internal_28 + Fu(3, 0)*internal_29 + Fu(4, 0)*internal_30;

    // Evaluation of Matrix Q_uu_
    Q_uu_(0, 0) = Fu(0, 0)*(Fu(1, 0)*V_xx_prev(1, 0) + Fu(2, 0)*V_xx_prev(2, 0) + Fu(3, 0)*V_xx_prev(3, 0) + Fu(4, 0)*V_xx_prev(4, 0) + internal_31) + Fu(1, 0)*(Fu(0, 0)*V_xx_prev(0, 1) + Fu(1, 0)*V_xx_prev(1, 1) + Fu(2, 0)*V_xx_prev(2, 1) + Fu(3, 0)*V_xx_prev(3, 1) + Fu(4, 0)*V_xx_prev(4, 1)) + Fu(2, 0)*(Fu(0, 0)*V_xx_prev(0, 2) + Fu(1, 0)*V_xx_prev(1, 2) + Fu(2, 0)*V_xx_prev(2, 2) + Fu(3, 0)*V_xx_prev(3, 2) + Fu(4, 0)*V_xx_prev(4, 2)) + Fu(3, 0)*(Fu(0, 0)*V_xx_prev(0, 3) + Fu(1, 0)*V_xx_prev(1, 3) + Fu(2, 0)*V_xx_prev(2, 3) + Fu(3, 0)*V_xx_prev(3, 3) + Fu(4, 0)*V_xx_prev(4, 3)) + Fu(4, 0)*(Fu(0, 0)*V_xx_prev(0, 4) + Fu(1, 0)*V_xx_prev(1, 4) + Fu(2, 0)*V_xx_prev(2, 4) + Fu(3, 0)*V_xx_prev(3, 4) + Fu(4, 0)*V_xx_prev(4, 4)) + Sigma_(3)*pow(g_u(3, 0), 2) + Sigma_(4)*pow(g_u(4, 0), 2) + l_uu(0, 0);
    Q_uu_(1, 1) = Sigma_(5)*pow(g_u(5, 1), 2) + Sigma_(6)*pow(g_u(6, 1), 2) + l_uu(1, 1);

  }

  virtual void updateQG(const Eigen::VectorXd& sl_x, const Eigen::VectorXd& sl_u, const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::VectorXd& r_f, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& V_xx_prev, const Eigen::VectorXd& V_x_prev) override {
    // Determine internal variables
    const double internal_0 = V_x_prev(0) + V_xx_prev(0, 0)*r_f(0) + V_xx_prev(0, 1)*r_f(1) + V_xx_prev(0, 2)*r_f(2) + V_xx_prev(0, 3)*r_f(3) + V_xx_prev(0, 4)*r_f(4);
    const double internal_1 = V_x_prev(1) + V_xx_prev(1, 0)*r_f(0) + V_xx_prev(1, 1)*r_f(1) + V_xx_prev(1, 2)*r_f(2) + V_xx_prev(1, 3)*r_f(3) + V_xx_prev(1, 4)*r_f(4);
    const double internal_2 = V_x_prev(2) + V_xx_prev(2, 0)*r_f(0) + V_xx_prev(2, 1)*r_f(1) + V_xx_prev(2, 2)*r_f(2) + V_xx_prev(2, 3)*r_f(3) + V_xx_prev(2, 4)*r_f(4);
    const double internal_3 = V_x_prev(3) + V_xx_prev(3, 0)*r_f(0) + V_xx_prev(3, 1)*r_f(1) + V_xx_prev(3, 2)*r_f(2) + V_xx_prev(3, 3)*r_f(3) + V_xx_prev(3, 4)*r_f(4);
    const double internal_4 = V_x_prev(4) + V_xx_prev(4, 0)*r_f(0) + V_xx_prev(4, 1)*r_f(1) + V_xx_prev(4, 2)*r_f(2) + V_xx_prev(4, 3)*r_f(3) + V_xx_prev(4, 4)*r_f(4);

    // Evaluation of Vector Q_x_
    Q_x_(0) = Fx(0, 0)*internal_0 + sl_x(0);
    Q_x_(1) = Fx(0, 1)*internal_0 + Fx(1, 1)*internal_1 + V_(5)*g_x(5, 1) + V_(6)*g_x(6, 1) + sl_x(1);
    Q_x_(2) = Fx(0, 2)*internal_0 + Fx(2, 2)*internal_2 + V_(5)*g_x(5, 2) + V_(6)*g_x(6, 2) + sl_x(2);
    Q_x_(3) = Fx(0, 3)*internal_0 + Fx(1, 3)*internal_1 + Fx(2, 3)*internal_2 + Fx(3, 3)*internal_3 + V_(2)*g_x(2, 3) + sl_x(3);
    Q_x_(4) = Fx(0, 4)*internal_0 + Fx(1, 4)*internal_1 + Fx(2, 4)*internal_2 + Fx(3, 4)*internal_3 + Fx(4, 4)*internal_4 + V_(0)*g_x(0, 4) + V_(1)*g_x(1, 4) + sl_x(4);

    // Evaluation of Vector Q_u_
    Q_u_(0) = Fu(0, 0)*internal_0 + Fu(1, 0)*internal_1 + Fu(2, 0)*internal_2 + Fu(3, 0)*internal_3 + Fu(4, 0)*internal_4 + V_(3)*g_u(3, 0) + V_(4)*g_u(4, 0) + sl_u(0);
    Q_u_(1) = V_(5)*g_u(5, 1) + V_(6)*g_u(6, 1) + sl_u(1);

  }

  virtual void updateQuuInv() override {
    // Evaluation of Matrix Q_uu_inv_
    Q_uu_inv_(0, 0) = 1.0/Q_uu_(0, 0);
    Q_uu_inv_(1, 1) = 1.0/Q_uu_(1, 1);

  }

  virtual void updateKVU(Eigen::MatrixXd& k_ux, Eigen::VectorXd& v_u) override {
    // Evaluation of Matrix k_ux
    k_ux(0, 0) = -Q_uu_inv_(0, 0)*Q_xu_(0, 0);
    k_ux(0, 1) = -Q_uu_inv_(0, 0)*Q_xu_(1, 0);
    k_ux(0, 2) = -Q_uu_inv_(0, 0)*Q_xu_(2, 0);
    k_ux(0, 3) = -Q_uu_inv_(0, 0)*Q_xu_(3, 0);
    k_ux(0, 4) = -Q_uu_inv_(0, 0)*Q_xu_(4, 0);
    k_ux(1, 1) = -Q_uu_inv_(1, 1)*Q_xu_(1, 1);
    k_ux(1, 2) = -Q_uu_inv_(1, 1)*Q_xu_(2, 1);

    // Evaluation of Vector v_u
    v_u(0) = -Q_u_(0)*Q_uu_inv_(0, 0);
    v_u(1) = -Q_u_(1)*Q_uu_inv_(1, 1);

  }

  virtual void updateQee(const Eigen::MatrixXd& h_u) override {

  }

  virtual void updateQeeInv() override {

  }

  virtual void updateQxe(const Eigen::VectorXd& h, const Eigen::MatrixXd& h_x, const Eigen::MatrixXd& h_u, const Eigen::MatrixXd& k_ux, const Eigen::VectorXd& v_u) override {

  }

  virtual void updateKVE(const Eigen::MatrixXd& h_u, Eigen::MatrixXd& k_ex, Eigen::MatrixXd& k_ue, Eigen::VectorXd& v_e) override {

  }

  virtual void updateVxx(const Eigen::MatrixXd& k_ux, const Eigen::MatrixXd& k_ex, const Eigen::VectorXd& v_u, const Eigen::VectorXd& v_e, Eigen::MatrixXd& V_xx, Eigen::VectorXd& V_x) override {
    // Evaluation of Matrix V_xx
    V_xx(0, 0) = Q_xu_(0, 0)*k_ux(0, 0) + Q_xx_(0, 0);
    V_xx(0, 1) = Q_xu_(0, 0)*k_ux(0, 1) + Q_xx_(0, 1);
    V_xx(0, 2) = Q_xu_(0, 0)*k_ux(0, 2) + Q_xx_(0, 2);
    V_xx(0, 3) = Q_xu_(0, 0)*k_ux(0, 3) + Q_xx_(0, 3);
    V_xx(0, 4) = Q_xu_(0, 0)*k_ux(0, 4) + Q_xx_(0, 4);
    V_xx(1, 0) = Q_xu_(1, 0)*k_ux(0, 0) + Q_xx_(0, 1);
    V_xx(1, 1) = Q_xu_(1, 0)*k_ux(0, 1) + Q_xu_(1, 1)*k_ux(1, 1) + Q_xx_(1, 1);
    V_xx(1, 2) = Q_xu_(1, 0)*k_ux(0, 2) + Q_xu_(1, 1)*k_ux(1, 2) + Q_xx_(1, 2);
    V_xx(1, 3) = Q_xu_(1, 0)*k_ux(0, 3) + Q_xx_(1, 3);
    V_xx(1, 4) = Q_xu_(1, 0)*k_ux(0, 4) + Q_xx_(1, 4);
    V_xx(2, 0) = Q_xu_(2, 0)*k_ux(0, 0) + Q_xx_(0, 2);
    V_xx(2, 1) = Q_xu_(2, 0)*k_ux(0, 1) + Q_xu_(2, 1)*k_ux(1, 1) + Q_xx_(1, 2);
    V_xx(2, 2) = Q_xu_(2, 0)*k_ux(0, 2) + Q_xu_(2, 1)*k_ux(1, 2) + Q_xx_(2, 2);
    V_xx(2, 3) = Q_xu_(2, 0)*k_ux(0, 3) + Q_xx_(2, 3);
    V_xx(2, 4) = Q_xu_(2, 0)*k_ux(0, 4) + Q_xx_(2, 4);
    V_xx(3, 0) = Q_xu_(3, 0)*k_ux(0, 0) + Q_xx_(0, 3);
    V_xx(3, 1) = Q_xu_(3, 0)*k_ux(0, 1) + Q_xx_(1, 3);
    V_xx(3, 2) = Q_xu_(3, 0)*k_ux(0, 2) + Q_xx_(2, 3);
    V_xx(3, 3) = Q_xu_(3, 0)*k_ux(0, 3) + Q_xx_(3, 3);
    V_xx(3, 4) = Q_xu_(3, 0)*k_ux(0, 4) + Q_xx_(3, 4);
    V_xx(4, 0) = Q_xu_(4, 0)*k_ux(0, 0) + Q_xx_(0, 4);
    V_xx(4, 1) = Q_xu_(4, 0)*k_ux(0, 1) + Q_xx_(1, 4);
    V_xx(4, 2) = Q_xu_(4, 0)*k_ux(0, 2) + Q_xx_(2, 4);
    V_xx(4, 3) = Q_xu_(4, 0)*k_ux(0, 3) + Q_xx_(3, 4);
    V_xx(4, 4) = Q_xu_(4, 0)*k_ux(0, 4) + Q_xx_(4, 4);

    // Evaluation of Vector V_x
    V_x(0) = Q_x_(0) + Q_xu_(0, 0)*v_u(0);
    V_x(1) = Q_x_(1) + Q_xu_(1, 0)*v_u(0) + Q_xu_(1, 1)*v_u(1);
    V_x(2) = Q_x_(2) + Q_xu_(2, 0)*v_u(0) + Q_xu_(2, 1)*v_u(1);
    V_x(3) = Q_x_(3) + Q_xu_(3, 0)*v_u(0);
    V_x(4) = Q_x_(4) + Q_xu_(4, 0)*v_u(0);

  }

  virtual void updateQnx(const Eigen::VectorXd& r_f, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& h_u, const Eigen::MatrixXd& k_ux, const Eigen::MatrixXd& k_ex, const Eigen::MatrixXd& k_ue, const Eigen::VectorXd& v_u, const Eigen::VectorXd& v_e) override {
    // Evaluation of Matrix Q_un_
    Q_un_(0, 0) = -Fu(0, 0)*Q_uu_inv_(0, 0);
    Q_un_(0, 1) = -Fu(1, 0)*Q_uu_inv_(0, 0);
    Q_un_(0, 2) = -Fu(2, 0)*Q_uu_inv_(0, 0);
    Q_un_(0, 3) = -Fu(3, 0)*Q_uu_inv_(0, 0);
    Q_un_(0, 4) = -Fu(4, 0)*Q_uu_inv_(0, 0);

    // Evaluation of Matrix Q_nuu_
    Q_nuu_(0, 0) = -Q_uu_(0, 0);
    Q_nuu_(1, 1) = -Q_uu_(1, 1);

    // Evaluation of Matrix Q_vnx_
    Q_vnx_(0, 0) = Fu(0, 0)*k_ux(0, 0) + Fx(0, 0);
    Q_vnx_(0, 1) = Fu(0, 0)*k_ux(0, 1) + Fx(0, 1);
    Q_vnx_(0, 2) = Fu(0, 0)*k_ux(0, 2) + Fx(0, 2);
    Q_vnx_(0, 3) = Fu(0, 0)*k_ux(0, 3) + Fx(0, 3);
    Q_vnx_(0, 4) = Fu(0, 0)*k_ux(0, 4) + Fx(0, 4);
    Q_vnx_(1, 0) = Fu(1, 0)*k_ux(0, 0);
    Q_vnx_(1, 1) = Fu(1, 0)*k_ux(0, 1) + Fx(1, 1);
    Q_vnx_(1, 2) = Fu(1, 0)*k_ux(0, 2);
    Q_vnx_(1, 3) = Fu(1, 0)*k_ux(0, 3) + Fx(1, 3);
    Q_vnx_(1, 4) = Fu(1, 0)*k_ux(0, 4) + Fx(1, 4);
    Q_vnx_(2, 0) = Fu(2, 0)*k_ux(0, 0);
    Q_vnx_(2, 1) = Fu(2, 0)*k_ux(0, 1);
    Q_vnx_(2, 2) = Fu(2, 0)*k_ux(0, 2) + Fx(2, 2);
    Q_vnx_(2, 3) = Fu(2, 0)*k_ux(0, 3) + Fx(2, 3);
    Q_vnx_(2, 4) = Fu(2, 0)*k_ux(0, 4) + Fx(2, 4);
    Q_vnx_(3, 0) = Fu(3, 0)*k_ux(0, 0);
    Q_vnx_(3, 1) = Fu(3, 0)*k_ux(0, 1);
    Q_vnx_(3, 2) = Fu(3, 0)*k_ux(0, 2);
    Q_vnx_(3, 3) = Fu(3, 0)*k_ux(0, 3) + Fx(3, 3);
    Q_vnx_(3, 4) = Fu(3, 0)*k_ux(0, 4) + Fx(3, 4);
    Q_vnx_(4, 0) = Fu(4, 0)*k_ux(0, 0);
    Q_vnx_(4, 1) = Fu(4, 0)*k_ux(0, 1);
    Q_vnx_(4, 2) = Fu(4, 0)*k_ux(0, 2);
    Q_vnx_(4, 3) = Fu(4, 0)*k_ux(0, 3);
    Q_vnx_(4, 4) = Fu(4, 0)*k_ux(0, 4) + Fx(4, 4);

    // Evaluation of Vector Q_vn_
    Q_vn_(0) = Fu(0, 0)*v_u(0) + r_f(0);
    Q_vn_(1) = Fu(1, 0)*v_u(0) + r_f(1);
    Q_vn_(2) = Fu(2, 0)*v_u(0) + r_f(2);
    Q_vn_(3) = Fu(3, 0)*v_u(0) + r_f(3);
    Q_vn_(4) = Fu(4, 0)*v_u(0) + r_f(4);

  }

};

template <>
class TrackerModelIpmEvaluator<StageType::INITIAL, OcpConfig::ERK4> : public IpmEvaluator {
 public:
  TrackerModelIpmEvaluator() : IpmEvaluator(5, 2, 2, 0, 5) {}
  virtual ~TrackerModelIpmEvaluator() = default;

  virtual void updateLX(const Eigen::VectorXd& l_x, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& h_x, const Eigen::MatrixXd& m_x, const Eigen::VectorXd& p, const Eigen::VectorXd& lambda, const Eigen::VectorXd& eta, const Eigen::VectorXd& nu, Eigen::VectorXd& sl_x) const override {
    // Evaluation of Vector sl_x
    sl_x(0) = Fx(0, 0)*p(0) + m_x(0, 0)*nu(0);
    sl_x(1) = Fx(0, 1)*p(0) + Fx(1, 1)*p(1) + l_x(1) + m_x(1, 1)*nu(1);
    sl_x(2) = Fx(0, 2)*p(0) + Fx(2, 2)*p(2) + l_x(2) + m_x(2, 2)*nu(2);
    sl_x(3) = Fx(0, 3)*p(0) + Fx(1, 3)*p(1) + Fx(2, 3)*p(2) + Fx(3, 3)*p(3) + l_x(3) + m_x(3, 3)*nu(3);
    sl_x(4) = Fx(0, 4)*p(0) + Fx(1, 4)*p(1) + Fx(2, 4)*p(2) + Fx(3, 4)*p(3) + Fx(4, 4)*p(4) + l_x(4) + m_x(4, 4)*nu(4);

  }

  virtual void updateLU(const Eigen::VectorXd& l_u, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& g_u, const Eigen::MatrixXd& h_u, const Eigen::VectorXd& p, const Eigen::VectorXd& lambda, const Eigen::VectorXd& eta, Eigen::VectorXd& sl_u) const override {
    // Evaluation of Vector sl_u
    sl_u(0) = Fu(0, 0)*p(0) + Fu(1, 0)*p(1) + Fu(2, 0)*p(2) + Fu(3, 0)*p(3) + Fu(4, 0)*p(4) + g_u(0, 0)*lambda(0) + g_u(1, 0)*lambda(1) + l_u(0);
    sl_u(1) = l_u(1);

  }

  virtual void updateQH(const Eigen::MatrixXd& l_xx, const Eigen::MatrixXd& l_xu, const Eigen::MatrixXd& l_uu, const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& V_xx_prev) override {
    // Determine internal variables
    const double internal_0 = Fx(0, 1)*V_xx_prev(0, 0);
    const double internal_1 = Fx(0, 0)*V_xx_prev(0, 1);
    const double internal_2 = Fx(0, 2)*V_xx_prev(0, 0);
    const double internal_28 = Fu(0, 0)*V_xx_prev(0, 0);
    const double internal_3 = Fx(0, 0)*V_xx_prev(0, 2);
    const double internal_4 = Fx(0, 3)*V_xx_prev(0, 0);
    const double internal_5 = Fx(0, 0)*V_xx_prev(0, 3);
    const double internal_6 = Fx(0, 4)*V_xx_prev(0, 0);
    const double internal_7 = Fx(0, 0)*V_xx_prev(0, 4);
    const double internal_10 = Fx(0, 1)*V_xx_prev(0, 2) + Fx(1, 1)*V_xx_prev(1, 2);
    const double internal_11 = Fx(0, 1)*V_xx_prev(0, 3) + Fx(1, 1)*V_xx_prev(1, 3);
    const double internal_12 = Fx(0, 1)*V_xx_prev(0, 4) + Fx(1, 1)*V_xx_prev(1, 4);
    const double internal_14 = Fx(0, 2)*V_xx_prev(0, 1) + Fx(2, 2)*V_xx_prev(2, 1);
    const double internal_15 = Fx(0, 2)*V_xx_prev(0, 2) + Fx(2, 2)*V_xx_prev(2, 2);
    const double internal_16 = Fx(0, 2)*V_xx_prev(0, 3) + Fx(2, 2)*V_xx_prev(2, 3);
    const double internal_17 = Fx(0, 2)*V_xx_prev(0, 4) + Fx(2, 2)*V_xx_prev(2, 4);
    const double internal_19 = Fx(0, 3)*V_xx_prev(0, 1) + Fx(1, 3)*V_xx_prev(1, 1) + Fx(2, 3)*V_xx_prev(2, 1) + Fx(3, 3)*V_xx_prev(3, 1);
    const double internal_20 = Fx(0, 3)*V_xx_prev(0, 2) + Fx(1, 3)*V_xx_prev(1, 2) + Fx(2, 3)*V_xx_prev(2, 2) + Fx(3, 3)*V_xx_prev(3, 2);
    const double internal_21 = Fx(0, 3)*V_xx_prev(0, 3) + Fx(1, 3)*V_xx_prev(1, 3) + Fx(2, 3)*V_xx_prev(2, 3) + Fx(3, 3)*V_xx_prev(3, 3);
    const double internal_22 = Fx(0, 3)*V_xx_prev(0, 4) + Fx(1, 3)*V_xx_prev(1, 4) + Fx(2, 3)*V_xx_prev(2, 4) + Fx(3, 3)*V_xx_prev(3, 4);
    const double internal_24 = Fx(0, 4)*V_xx_prev(0, 1) + Fx(1, 4)*V_xx_prev(1, 1) + Fx(2, 4)*V_xx_prev(2, 1) + Fx(3, 4)*V_xx_prev(3, 1) + Fx(4, 4)*V_xx_prev(4, 1);
    const double internal_25 = Fx(0, 4)*V_xx_prev(0, 2) + Fx(1, 4)*V_xx_prev(1, 2) + Fx(2, 4)*V_xx_prev(2, 2) + Fx(3, 4)*V_xx_prev(3, 2) + Fx(4, 4)*V_xx_prev(4, 2);
    const double internal_26 = Fx(0, 4)*V_xx_prev(0, 3) + Fx(1, 4)*V_xx_prev(1, 3) + Fx(2, 4)*V_xx_prev(2, 3) + Fx(3, 4)*V_xx_prev(3, 3) + Fx(4, 4)*V_xx_prev(4, 3);
    const double internal_27 = Fx(0, 4)*V_xx_prev(0, 4) + Fx(1, 4)*V_xx_prev(1, 4) + Fx(2, 4)*V_xx_prev(2, 4) + Fx(3, 4)*V_xx_prev(3, 4) + Fx(4, 4)*V_xx_prev(4, 4);
    const double internal_9 = Fx(0, 1)*V_xx_prev(0, 1) + Fx(1, 1)*V_xx_prev(1, 1);
    const double internal_13 = Fx(2, 2)*V_xx_prev(2, 0) + internal_2;
    const double internal_18 = Fx(1, 3)*V_xx_prev(1, 0) + Fx(2, 3)*V_xx_prev(2, 0) + Fx(3, 3)*V_xx_prev(3, 0) + internal_4;
    const double internal_23 = Fx(1, 4)*V_xx_prev(1, 0) + Fx(2, 4)*V_xx_prev(2, 0) + Fx(3, 4)*V_xx_prev(3, 0) + Fx(4, 4)*V_xx_prev(4, 0) + internal_6;
    const double internal_8 = Fx(1, 1)*V_xx_prev(1, 0) + internal_0;

    // Evaluation of Matrix Q_xx_
    Q_xx_(0, 0) = pow(Fx(0, 0), 2)*V_xx_prev(0, 0);
    Q_xx_(0, 1) = Fx(0, 0)*internal_0 + Fx(1, 1)*internal_1;
    Q_xx_(0, 2) = Fx(0, 0)*internal_2 + Fx(2, 2)*internal_3;
    Q_xx_(0, 3) = Fx(0, 0)*internal_4 + Fx(1, 3)*internal_1 + Fx(2, 3)*internal_3 + Fx(3, 3)*internal_5;
    Q_xx_(0, 4) = Fx(0, 0)*internal_6 + Fx(1, 4)*internal_1 + Fx(2, 4)*internal_3 + Fx(3, 4)*internal_5 + Fx(4, 4)*internal_7;
    Q_xx_(1, 0) = Fx(0, 0)*internal_8;
    Q_xx_(1, 1) = Fx(0, 1)*internal_8 + Fx(1, 1)*internal_9 + l_xx(1, 1);
    Q_xx_(1, 2) = Fx(0, 2)*internal_8 + Fx(2, 2)*internal_10 + l_xx(1, 2);
    Q_xx_(1, 3) = Fx(0, 3)*internal_8 + Fx(1, 3)*internal_9 + Fx(2, 3)*internal_10 + Fx(3, 3)*internal_11;
    Q_xx_(1, 4) = Fx(0, 4)*internal_8 + Fx(1, 4)*internal_9 + Fx(2, 4)*internal_10 + Fx(3, 4)*internal_11 + Fx(4, 4)*internal_12;
    Q_xx_(2, 0) = Fx(0, 0)*internal_13;
    Q_xx_(2, 1) = Fx(0, 1)*internal_13 + Fx(1, 1)*internal_14 + l_xx(2, 1);
    Q_xx_(2, 2) = Fx(0, 2)*internal_13 + Fx(2, 2)*internal_15 + l_xx(2, 2);
    Q_xx_(2, 3) = Fx(0, 3)*internal_13 + Fx(1, 3)*internal_14 + Fx(2, 3)*internal_15 + Fx(3, 3)*internal_16;
    Q_xx_(2, 4) = Fx(0, 4)*internal_13 + Fx(1, 4)*internal_14 + Fx(2, 4)*internal_15 + Fx(3, 4)*internal_16 + Fx(4, 4)*internal_17;
    Q_xx_(3, 0) = Fx(0, 0)*internal_18;
    Q_xx_(3, 1) = Fx(0, 1)*internal_18 + Fx(1, 1)*internal_19;
    Q_xx_(3, 2) = Fx(0, 2)*internal_18 + Fx(2, 2)*internal_20;
    Q_xx_(3, 3) = Fx(0, 3)*internal_18 + Fx(1, 3)*internal_19 + Fx(2, 3)*internal_20 + Fx(3, 3)*internal_21 + l_xx(3, 3);
    Q_xx_(3, 4) = Fx(0, 4)*internal_18 + Fx(1, 4)*internal_19 + Fx(2, 4)*internal_20 + Fx(3, 4)*internal_21 + Fx(4, 4)*internal_22;
    Q_xx_(4, 0) = Fx(0, 0)*internal_23;
    Q_xx_(4, 1) = Fx(0, 1)*internal_23 + Fx(1, 1)*internal_24;
    Q_xx_(4, 2) = Fx(0, 2)*internal_23 + Fx(2, 2)*internal_25;
    Q_xx_(4, 3) = Fx(0, 3)*internal_23 + Fx(1, 3)*internal_24 + Fx(2, 3)*internal_25 + Fx(3, 3)*internal_26;
    Q_xx_(4, 4) = Fx(0, 4)*internal_23 + Fx(1, 4)*internal_24 + Fx(2, 4)*internal_25 + Fx(3, 4)*internal_26 + Fx(4, 4)*internal_27 + l_xx(4, 4);

    // Evaluation of Matrix Q_xu_
    Q_xu_(0, 0) = Fu(1, 0)*internal_1 + Fu(2, 0)*internal_3 + Fu(3, 0)*internal_5 + Fu(4, 0)*internal_7 + Fx(0, 0)*internal_28;
    Q_xu_(1, 0) = Fu(0, 0)*internal_8 + Fu(1, 0)*internal_9 + Fu(2, 0)*internal_10 + Fu(3, 0)*internal_11 + Fu(4, 0)*internal_12;
    Q_xu_(2, 0) = Fu(0, 0)*internal_13 + Fu(1, 0)*internal_14 + Fu(2, 0)*internal_15 + Fu(3, 0)*internal_16 + Fu(4, 0)*internal_17;
    Q_xu_(3, 0) = Fu(0, 0)*internal_18 + Fu(1, 0)*internal_19 + Fu(2, 0)*internal_20 + Fu(3, 0)*internal_21 + Fu(4, 0)*internal_22;
    Q_xu_(4, 0) = Fu(0, 0)*internal_23 + Fu(1, 0)*internal_24 + Fu(2, 0)*internal_25 + Fu(3, 0)*internal_26 + Fu(4, 0)*internal_27;

    // Evaluation of Matrix Q_uu_
    Q_uu_(0, 0) = Fu(0, 0)*(Fu(1, 0)*V_xx_prev(1, 0) + Fu(2, 0)*V_xx_prev(2, 0) + Fu(3, 0)*V_xx_prev(3, 0) + Fu(4, 0)*V_xx_prev(4, 0) + internal_28) + Fu(1, 0)*(Fu(0, 0)*V_xx_prev(0, 1) + Fu(1, 0)*V_xx_prev(1, 1) + Fu(2, 0)*V_xx_prev(2, 1) + Fu(3, 0)*V_xx_prev(3, 1) + Fu(4, 0)*V_xx_prev(4, 1)) + Fu(2, 0)*(Fu(0, 0)*V_xx_prev(0, 2) + Fu(1, 0)*V_xx_prev(1, 2) + Fu(2, 0)*V_xx_prev(2, 2) + Fu(3, 0)*V_xx_prev(3, 2) + Fu(4, 0)*V_xx_prev(4, 2)) + Fu(3, 0)*(Fu(0, 0)*V_xx_prev(0, 3) + Fu(1, 0)*V_xx_prev(1, 3) + Fu(2, 0)*V_xx_prev(2, 3) + Fu(3, 0)*V_xx_prev(3, 3) + Fu(4, 0)*V_xx_prev(4, 3)) + Fu(4, 0)*(Fu(0, 0)*V_xx_prev(0, 4) + Fu(1, 0)*V_xx_prev(1, 4) + Fu(2, 0)*V_xx_prev(2, 4) + Fu(3, 0)*V_xx_prev(3, 4) + Fu(4, 0)*V_xx_prev(4, 4)) + Sigma_(0)*pow(g_u(0, 0), 2) + Sigma_(1)*pow(g_u(1, 0), 2) + l_uu(0, 0);
    Q_uu_(1, 1) = l_uu(1, 1);

  }

  virtual void updateQG(const Eigen::VectorXd& sl_x, const Eigen::VectorXd& sl_u, const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::VectorXd& r_f, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& V_xx_prev, const Eigen::VectorXd& V_x_prev) override {
    // Determine internal variables
    const double internal_0 = V_x_prev(0) + V_xx_prev(0, 0)*r_f(0) + V_xx_prev(0, 1)*r_f(1) + V_xx_prev(0, 2)*r_f(2) + V_xx_prev(0, 3)*r_f(3) + V_xx_prev(0, 4)*r_f(4);
    const double internal_1 = V_x_prev(1) + V_xx_prev(1, 0)*r_f(0) + V_xx_prev(1, 1)*r_f(1) + V_xx_prev(1, 2)*r_f(2) + V_xx_prev(1, 3)*r_f(3) + V_xx_prev(1, 4)*r_f(4);
    const double internal_2 = V_x_prev(2) + V_xx_prev(2, 0)*r_f(0) + V_xx_prev(2, 1)*r_f(1) + V_xx_prev(2, 2)*r_f(2) + V_xx_prev(2, 3)*r_f(3) + V_xx_prev(2, 4)*r_f(4);
    const double internal_3 = V_x_prev(3) + V_xx_prev(3, 0)*r_f(0) + V_xx_prev(3, 1)*r_f(1) + V_xx_prev(3, 2)*r_f(2) + V_xx_prev(3, 3)*r_f(3) + V_xx_prev(3, 4)*r_f(4);
    const double internal_4 = V_x_prev(4) + V_xx_prev(4, 0)*r_f(0) + V_xx_prev(4, 1)*r_f(1) + V_xx_prev(4, 2)*r_f(2) + V_xx_prev(4, 3)*r_f(3) + V_xx_prev(4, 4)*r_f(4);

    // Evaluation of Vector Q_x_
    Q_x_(0) = Fx(0, 0)*internal_0 + sl_x(0);
    Q_x_(1) = Fx(0, 1)*internal_0 + Fx(1, 1)*internal_1 + sl_x(1);
    Q_x_(2) = Fx(0, 2)*internal_0 + Fx(2, 2)*internal_2 + sl_x(2);
    Q_x_(3) = Fx(0, 3)*internal_0 + Fx(1, 3)*internal_1 + Fx(2, 3)*internal_2 + Fx(3, 3)*internal_3 + sl_x(3);
    Q_x_(4) = Fx(0, 4)*internal_0 + Fx(1, 4)*internal_1 + Fx(2, 4)*internal_2 + Fx(3, 4)*internal_3 + Fx(4, 4)*internal_4 + sl_x(4);

    // Evaluation of Vector Q_u_
    Q_u_(0) = Fu(0, 0)*internal_0 + Fu(1, 0)*internal_1 + Fu(2, 0)*internal_2 + Fu(3, 0)*internal_3 + Fu(4, 0)*internal_4 + V_(0)*g_u(0, 0) + V_(1)*g_u(1, 0) + sl_u(0);
    Q_u_(1) = sl_u(1);

  }

  virtual void updateQuuInv() override {
    // Evaluation of Matrix Q_uu_inv_
    Q_uu_inv_(0, 0) = 1.0/Q_uu_(0, 0);
    Q_uu_inv_(1, 1) = 1.0/Q_uu_(1, 1);

  }

  virtual void updateKVU(Eigen::MatrixXd& k_ux, Eigen::VectorXd& v_u) override {
    // Evaluation of Matrix k_ux
    k_ux(0, 0) = -Q_uu_inv_(0, 0)*Q_xu_(0, 0);
    k_ux(0, 1) = -Q_uu_inv_(0, 0)*Q_xu_(1, 0);
    k_ux(0, 2) = -Q_uu_inv_(0, 0)*Q_xu_(2, 0);
    k_ux(0, 3) = -Q_uu_inv_(0, 0)*Q_xu_(3, 0);
    k_ux(0, 4) = -Q_uu_inv_(0, 0)*Q_xu_(4, 0);

    // Evaluation of Vector v_u
    v_u(0) = -Q_u_(0)*Q_uu_inv_(0, 0);
    v_u(1) = -Q_u_(1)*Q_uu_inv_(1, 1);

  }

  virtual void updateQee(const Eigen::MatrixXd& h_u) override {

  }

  virtual void updateQeeInv() override {

  }

  virtual void updateQxe(const Eigen::VectorXd& h, const Eigen::MatrixXd& h_x, const Eigen::MatrixXd& h_u, const Eigen::MatrixXd& k_ux, const Eigen::VectorXd& v_u) override {

  }

  virtual void updateKVE(const Eigen::MatrixXd& h_u, Eigen::MatrixXd& k_ex, Eigen::MatrixXd& k_ue, Eigen::VectorXd& v_e) override {

  }

  virtual void updateVxx(const Eigen::MatrixXd& k_ux, const Eigen::MatrixXd& k_ex, const Eigen::VectorXd& v_u, const Eigen::VectorXd& v_e, Eigen::MatrixXd& V_xx, Eigen::VectorXd& V_x) override {
    // Evaluation of Matrix V_xx
    V_xx(0, 0) = Q_xu_(0, 0)*k_ux(0, 0) + Q_xx_(0, 0);
    V_xx(0, 1) = Q_xu_(0, 0)*k_ux(0, 1) + Q_xx_(0, 1);
    V_xx(0, 2) = Q_xu_(0, 0)*k_ux(0, 2) + Q_xx_(0, 2);
    V_xx(0, 3) = Q_xu_(0, 0)*k_ux(0, 3) + Q_xx_(0, 3);
    V_xx(0, 4) = Q_xu_(0, 0)*k_ux(0, 4) + Q_xx_(0, 4);
    V_xx(1, 0) = Q_xu_(1, 0)*k_ux(0, 0) + Q_xx_(0, 1);
    V_xx(1, 1) = Q_xu_(1, 0)*k_ux(0, 1) + Q_xx_(1, 1);
    V_xx(1, 2) = Q_xu_(1, 0)*k_ux(0, 2) + Q_xx_(1, 2);
    V_xx(1, 3) = Q_xu_(1, 0)*k_ux(0, 3) + Q_xx_(1, 3);
    V_xx(1, 4) = Q_xu_(1, 0)*k_ux(0, 4) + Q_xx_(1, 4);
    V_xx(2, 0) = Q_xu_(2, 0)*k_ux(0, 0) + Q_xx_(0, 2);
    V_xx(2, 1) = Q_xu_(2, 0)*k_ux(0, 1) + Q_xx_(1, 2);
    V_xx(2, 2) = Q_xu_(2, 0)*k_ux(0, 2) + Q_xx_(2, 2);
    V_xx(2, 3) = Q_xu_(2, 0)*k_ux(0, 3) + Q_xx_(2, 3);
    V_xx(2, 4) = Q_xu_(2, 0)*k_ux(0, 4) + Q_xx_(2, 4);
    V_xx(3, 0) = Q_xu_(3, 0)*k_ux(0, 0) + Q_xx_(0, 3);
    V_xx(3, 1) = Q_xu_(3, 0)*k_ux(0, 1) + Q_xx_(1, 3);
    V_xx(3, 2) = Q_xu_(3, 0)*k_ux(0, 2) + Q_xx_(2, 3);
    V_xx(3, 3) = Q_xu_(3, 0)*k_ux(0, 3) + Q_xx_(3, 3);
    V_xx(3, 4) = Q_xu_(3, 0)*k_ux(0, 4) + Q_xx_(3, 4);
    V_xx(4, 0) = Q_xu_(4, 0)*k_ux(0, 0) + Q_xx_(0, 4);
    V_xx(4, 1) = Q_xu_(4, 0)*k_ux(0, 1) + Q_xx_(1, 4);
    V_xx(4, 2) = Q_xu_(4, 0)*k_ux(0, 2) + Q_xx_(2, 4);
    V_xx(4, 3) = Q_xu_(4, 0)*k_ux(0, 3) + Q_xx_(3, 4);
    V_xx(4, 4) = Q_xu_(4, 0)*k_ux(0, 4) + Q_xx_(4, 4);

    // Evaluation of Vector V_x
    V_x(0) = Q_x_(0) + Q_xu_(0, 0)*v_u(0);
    V_x(1) = Q_x_(1) + Q_xu_(1, 0)*v_u(0);
    V_x(2) = Q_x_(2) + Q_xu_(2, 0)*v_u(0);
    V_x(3) = Q_x_(3) + Q_xu_(3, 0)*v_u(0);
    V_x(4) = Q_x_(4) + Q_xu_(4, 0)*v_u(0);

  }

  virtual void updateQnx(const Eigen::VectorXd& r_f, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& h_u, const Eigen::MatrixXd& k_ux, const Eigen::MatrixXd& k_ex, const Eigen::MatrixXd& k_ue, const Eigen::VectorXd& v_u, const Eigen::VectorXd& v_e) override {
    // Evaluation of Matrix Q_un_
    Q_un_(0, 0) = -Fu(0, 0)*Q_uu_inv_(0, 0);
    Q_un_(0, 1) = -Fu(1, 0)*Q_uu_inv_(0, 0);
    Q_un_(0, 2) = -Fu(2, 0)*Q_uu_inv_(0, 0);
    Q_un_(0, 3) = -Fu(3, 0)*Q_uu_inv_(0, 0);
    Q_un_(0, 4) = -Fu(4, 0)*Q_uu_inv_(0, 0);

    // Evaluation of Matrix Q_nuu_
    Q_nuu_(0, 0) = -Q_uu_(0, 0);
    Q_nuu_(1, 1) = -Q_uu_(1, 1);

    // Evaluation of Matrix Q_vnx_
    Q_vnx_(0, 0) = Fu(0, 0)*k_ux(0, 0) + Fx(0, 0);
    Q_vnx_(0, 1) = Fu(0, 0)*k_ux(0, 1) + Fx(0, 1);
    Q_vnx_(0, 2) = Fu(0, 0)*k_ux(0, 2) + Fx(0, 2);
    Q_vnx_(0, 3) = Fu(0, 0)*k_ux(0, 3) + Fx(0, 3);
    Q_vnx_(0, 4) = Fu(0, 0)*k_ux(0, 4) + Fx(0, 4);
    Q_vnx_(1, 0) = Fu(1, 0)*k_ux(0, 0);
    Q_vnx_(1, 1) = Fu(1, 0)*k_ux(0, 1) + Fx(1, 1);
    Q_vnx_(1, 2) = Fu(1, 0)*k_ux(0, 2);
    Q_vnx_(1, 3) = Fu(1, 0)*k_ux(0, 3) + Fx(1, 3);
    Q_vnx_(1, 4) = Fu(1, 0)*k_ux(0, 4) + Fx(1, 4);
    Q_vnx_(2, 0) = Fu(2, 0)*k_ux(0, 0);
    Q_vnx_(2, 1) = Fu(2, 0)*k_ux(0, 1);
    Q_vnx_(2, 2) = Fu(2, 0)*k_ux(0, 2) + Fx(2, 2);
    Q_vnx_(2, 3) = Fu(2, 0)*k_ux(0, 3) + Fx(2, 3);
    Q_vnx_(2, 4) = Fu(2, 0)*k_ux(0, 4) + Fx(2, 4);
    Q_vnx_(3, 0) = Fu(3, 0)*k_ux(0, 0);
    Q_vnx_(3, 1) = Fu(3, 0)*k_ux(0, 1);
    Q_vnx_(3, 2) = Fu(3, 0)*k_ux(0, 2);
    Q_vnx_(3, 3) = Fu(3, 0)*k_ux(0, 3) + Fx(3, 3);
    Q_vnx_(3, 4) = Fu(3, 0)*k_ux(0, 4) + Fx(3, 4);
    Q_vnx_(4, 0) = Fu(4, 0)*k_ux(0, 0);
    Q_vnx_(4, 1) = Fu(4, 0)*k_ux(0, 1);
    Q_vnx_(4, 2) = Fu(4, 0)*k_ux(0, 2);
    Q_vnx_(4, 3) = Fu(4, 0)*k_ux(0, 3);
    Q_vnx_(4, 4) = Fu(4, 0)*k_ux(0, 4) + Fx(4, 4);

    // Evaluation of Vector Q_vn_
    Q_vn_(0) = Fu(0, 0)*v_u(0) + r_f(0);
    Q_vn_(1) = Fu(1, 0)*v_u(0) + r_f(1);
    Q_vn_(2) = Fu(2, 0)*v_u(0) + r_f(2);
    Q_vn_(3) = Fu(3, 0)*v_u(0) + r_f(3);
    Q_vn_(4) = Fu(4, 0)*v_u(0) + r_f(4);

  }

};

class TrackerModelIpmEvaluatorTerminal : public IpmEvaluatorTerminal {
 public:
  TrackerModelIpmEvaluatorTerminal() : IpmEvaluatorTerminal(5, 2, 3, 0) {}
  virtual ~TrackerModelIpmEvaluatorTerminal() = default;

  virtual void updateLX(const Eigen::VectorXd& l_x, const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& m_x, const Eigen::VectorXd& p, const Eigen::VectorXd& lambda, const Eigen::VectorXd& nu, Eigen::VectorXd& sl_x) const override {
    // Evaluation of Vector sl_x
    sl_x(0) = -p(0);
    sl_x(1) = l_x(1) - p(1);
    sl_x(2) = l_x(2) - p(2);
    sl_x(3) = g_x(2, 3)*lambda(2) + l_x(3) - p(3);
    sl_x(4) = g_x(0, 4)*lambda(0) + g_x(1, 4)*lambda(1) - p(4);

  }

  virtual void updateVxx(const Eigen::MatrixXd& l_xx, const Eigen::VectorXd& sl_x, const Eigen::MatrixXd& g_x, Eigen::MatrixXd& V_xx, Eigen::VectorXd& V_x) override {
    // Evaluation of Matrix V_xx
    V_xx(1, 1) = l_xx(1, 1);
    V_xx(1, 2) = l_xx(1, 2);
    V_xx(2, 1) = l_xx(2, 1);
    V_xx(2, 2) = l_xx(2, 2);
    V_xx(3, 3) = Sigma_(2)*pow(g_x(2, 3), 2) + l_xx(3, 3);
    V_xx(4, 4) = Sigma_(0)*pow(g_x(0, 4), 2) + Sigma_(1)*pow(g_x(1, 4), 2);

    // Evaluation of Vector V_x
    V_x(0) = sl_x(0);
    V_x(1) = sl_x(1);
    V_x(2) = sl_x(2);
    V_x(3) = V_(2)*g_x(2, 3) + sl_x(3);
    V_x(4) = V_(0)*g_x(0, 4) + V_(1)*g_x(1, 4) + sl_x(4);

  }

};

}  // namespace gpal::pnc::planning