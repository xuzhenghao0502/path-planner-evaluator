#pragma once

#include "ocp/ipm_evaluator.h"

namespace gpal::pnc::planning {

template <StageType stage_type, OcpConfig::IntegratorType integrator_type>
class ParkingGeneralIpmEvaluator : public IpmEvaluator {
 public:
  ParkingGeneralIpmEvaluator() : IpmEvaluator(6, 3, 21, 0, 0) {}
  virtual ~ParkingGeneralIpmEvaluator() = default;

  virtual void updateLX(const Eigen::VectorXd& l_x, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& h_x, const Eigen::MatrixXd& m_x, const Eigen::VectorXd& p, const Eigen::VectorXd& lambda, const Eigen::VectorXd& eta, const Eigen::VectorXd& nu, Eigen::VectorXd& sl_x) const override {
    // Evaluation of Vector sl_x
    sl_x(0) = Fx(0, 0)*p(0);
    sl_x(1) = Fx(0, 1)*p(0) + Fx(1, 1)*p(1) + g_x(0, 1)*lambda(0) + g_x(1, 1)*lambda(1) + g_x(15, 1)*lambda(15) + g_x(16, 1)*lambda(16) + g_x(17, 1)*lambda(17) + g_x(18, 1)*lambda(18) + g_x(19, 1)*lambda(19) + g_x(2, 1)*lambda(2) + g_x(20, 1)*lambda(20) + g_x(3, 1)*lambda(3) + g_x(4, 1)*lambda(4) + g_x(5, 1)*lambda(5) + l_x(1);
    sl_x(2) = Fx(0, 2)*p(0) + Fx(2, 2)*p(2) + g_x(0, 2)*lambda(0) + g_x(1, 2)*lambda(1) + g_x(15, 2)*lambda(15) + g_x(16, 2)*lambda(16) + g_x(17, 2)*lambda(17) + g_x(18, 2)*lambda(18) + g_x(19, 2)*lambda(19) + g_x(2, 2)*lambda(2) + g_x(20, 2)*lambda(20) + g_x(3, 2)*lambda(3) + g_x(4, 2)*lambda(4) + g_x(5, 2)*lambda(5) + l_x(2);
    sl_x(3) = Fx(0, 3)*p(0) + Fx(1, 3)*p(1) + Fx(2, 3)*p(2) + Fx(3, 3)*p(3) + g_x(10, 3)*lambda(10) + g_x(17, 3)*lambda(17) + g_x(18, 3)*lambda(18) + g_x(19, 3)*lambda(19) + g_x(2, 3)*lambda(2) + g_x(20, 3)*lambda(20) + g_x(3, 3)*lambda(3) + g_x(4, 3)*lambda(4) + g_x(5, 3)*lambda(5);
    sl_x(4) = Fx(3, 4)*p(3) + Fx(4, 4)*p(4) + g_x(6, 4)*lambda(6) + g_x(7, 4)*lambda(7) + l_x(4);
    sl_x(5) = Fx(0, 5)*p(0) + Fx(1, 5)*p(1) + Fx(2, 5)*p(2) + Fx(3, 5)*p(3) + Fx(5, 5)*p(5) + g_x(8, 5)*lambda(8) + g_x(9, 5)*lambda(9) + l_x(5);

  }

  virtual void updateLU(const Eigen::VectorXd& l_u, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& g_u, const Eigen::MatrixXd& h_u, const Eigen::VectorXd& p, const Eigen::VectorXd& lambda, const Eigen::VectorXd& eta, Eigen::VectorXd& sl_u) const override {
    // Evaluation of Vector sl_u
    sl_u(0) = Fu(4, 0)*p(4) + g_u(11, 0)*lambda(11) + g_u(12, 0)*lambda(12) + l_u(0);
    sl_u(1) = Fu(5, 1)*p(5) + g_u(13, 1)*lambda(13) + g_u(14, 1)*lambda(14) + l_u(1);
    sl_u(2) = g_u(15, 2)*lambda(15) + g_u(16, 2)*lambda(16) + g_u(17, 2)*lambda(17) + g_u(18, 2)*lambda(18) + g_u(19, 2)*lambda(19) + g_u(20, 2)*lambda(20) + l_u(2);

  }

  virtual void updateQH(const Eigen::MatrixXd& l_xx, const Eigen::MatrixXd& l_xu, const Eigen::MatrixXd& l_uu, const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& V_xx_prev) override {
    // Determine internal variables
    const double internal_0 = Fx(0, 1)*V_xx_prev(0, 0);
    const double internal_1 = Fx(0, 0)*V_xx_prev(0, 1);
    const double internal_12 = Sigma_(15)*g_x(15, 1);
    const double internal_13 = Sigma_(16)*g_x(16, 1);
    const double internal_14 = Sigma_(17)*g_x(17, 1);
    const double internal_15 = Sigma_(18)*g_x(18, 1);
    const double internal_16 = Sigma_(19)*g_x(19, 1);
    const double internal_17 = Sigma_(2)*g_x(2, 1);
    const double internal_18 = Sigma_(20)*g_x(20, 1);
    const double internal_19 = Sigma_(3)*g_x(3, 1);
    const double internal_2 = Fx(0, 2)*V_xx_prev(0, 0);
    const double internal_20 = Sigma_(4)*g_x(4, 1);
    const double internal_21 = Sigma_(5)*g_x(5, 1);
    const double internal_3 = Fx(0, 0)*V_xx_prev(0, 2);
    const double internal_31 = Sigma_(17)*g_x(17, 2);
    const double internal_32 = Sigma_(18)*g_x(18, 2);
    const double internal_33 = Sigma_(19)*g_x(19, 2);
    const double internal_34 = Sigma_(20)*g_x(20, 2);
    const double internal_4 = Fx(0, 3)*V_xx_prev(0, 0);
    const double internal_5 = Fx(0, 0)*V_xx_prev(0, 3);
    const double internal_56 = Fu(4, 0)*Fu(5, 1);
    const double internal_6 = Fx(0, 0)*V_xx_prev(0, 4);
    const double internal_7 = Fx(0, 5)*V_xx_prev(0, 0);
    const double internal_8 = Fx(0, 0)*V_xx_prev(0, 5);
    const double internal_10 = Fx(0, 1)*V_xx_prev(0, 1) + Fx(1, 1)*V_xx_prev(1, 1);
    const double internal_11 = Fx(0, 1)*V_xx_prev(0, 2) + Fx(1, 1)*V_xx_prev(1, 2);
    const double internal_23 = Fx(0, 1)*V_xx_prev(0, 3) + Fx(1, 1)*V_xx_prev(1, 3);
    const double internal_25 = Fx(0, 1)*V_xx_prev(0, 4) + Fx(1, 1)*V_xx_prev(1, 4);
    const double internal_26 = Fx(0, 1)*V_xx_prev(0, 5) + Fx(1, 1)*V_xx_prev(1, 5);
    const double internal_28 = Fx(0, 2)*V_xx_prev(0, 1) + Fx(2, 2)*V_xx_prev(2, 1);
    const double internal_29 = Fx(0, 2)*V_xx_prev(0, 2) + Fx(2, 2)*V_xx_prev(2, 2);
    const double internal_30 = Fx(0, 2)*V_xx_prev(0, 3) + Fx(2, 2)*V_xx_prev(2, 3);
    const double internal_36 = Fx(0, 2)*V_xx_prev(0, 4) + Fx(2, 2)*V_xx_prev(2, 4);
    const double internal_37 = Fx(0, 2)*V_xx_prev(0, 5) + Fx(2, 2)*V_xx_prev(2, 5);
    const double internal_39 = Fx(0, 3)*V_xx_prev(0, 1) + Fx(1, 3)*V_xx_prev(1, 1) + Fx(2, 3)*V_xx_prev(2, 1) + Fx(3, 3)*V_xx_prev(3, 1);
    const double internal_40 = Fx(0, 3)*V_xx_prev(0, 2) + Fx(1, 3)*V_xx_prev(1, 2) + Fx(2, 3)*V_xx_prev(2, 2) + Fx(3, 3)*V_xx_prev(3, 2);
    const double internal_41 = Fx(0, 3)*V_xx_prev(0, 3) + Fx(1, 3)*V_xx_prev(1, 3) + Fx(2, 3)*V_xx_prev(2, 3) + Fx(3, 3)*V_xx_prev(3, 3);
    const double internal_42 = Fx(0, 3)*V_xx_prev(0, 4) + Fx(1, 3)*V_xx_prev(1, 4) + Fx(2, 3)*V_xx_prev(2, 4) + Fx(3, 3)*V_xx_prev(3, 4);
    const double internal_43 = Fx(0, 3)*V_xx_prev(0, 5) + Fx(1, 3)*V_xx_prev(1, 5) + Fx(2, 3)*V_xx_prev(2, 5) + Fx(3, 3)*V_xx_prev(3, 5);
    const double internal_44 = Fx(3, 4)*V_xx_prev(3, 0) + Fx(4, 4)*V_xx_prev(4, 0);
    const double internal_45 = Fx(3, 4)*V_xx_prev(3, 1) + Fx(4, 4)*V_xx_prev(4, 1);
    const double internal_46 = Fx(3, 4)*V_xx_prev(3, 2) + Fx(4, 4)*V_xx_prev(4, 2);
    const double internal_47 = Fx(3, 4)*V_xx_prev(3, 3) + Fx(4, 4)*V_xx_prev(4, 3);
    const double internal_48 = Fx(3, 4)*V_xx_prev(3, 4) + Fx(4, 4)*V_xx_prev(4, 4);
    const double internal_49 = Fx(3, 4)*V_xx_prev(3, 5) + Fx(4, 4)*V_xx_prev(4, 5);
    const double internal_51 = Fx(0, 5)*V_xx_prev(0, 1) + Fx(1, 5)*V_xx_prev(1, 1) + Fx(2, 5)*V_xx_prev(2, 1) + Fx(3, 5)*V_xx_prev(3, 1) + Fx(5, 5)*V_xx_prev(5, 1);
    const double internal_52 = Fx(0, 5)*V_xx_prev(0, 2) + Fx(1, 5)*V_xx_prev(1, 2) + Fx(2, 5)*V_xx_prev(2, 2) + Fx(3, 5)*V_xx_prev(3, 2) + Fx(5, 5)*V_xx_prev(5, 2);
    const double internal_53 = Fx(0, 5)*V_xx_prev(0, 3) + Fx(1, 5)*V_xx_prev(1, 3) + Fx(2, 5)*V_xx_prev(2, 3) + Fx(3, 5)*V_xx_prev(3, 3) + Fx(5, 5)*V_xx_prev(5, 3);
    const double internal_54 = Fx(0, 5)*V_xx_prev(0, 4) + Fx(1, 5)*V_xx_prev(1, 4) + Fx(2, 5)*V_xx_prev(2, 4) + Fx(3, 5)*V_xx_prev(3, 4) + Fx(5, 5)*V_xx_prev(5, 4);
    const double internal_55 = Fx(0, 5)*V_xx_prev(0, 5) + Fx(1, 5)*V_xx_prev(1, 5) + Fx(2, 5)*V_xx_prev(2, 5) + Fx(3, 5)*V_xx_prev(3, 5) + Fx(5, 5)*V_xx_prev(5, 5);
    const double internal_27 = Fx(2, 2)*V_xx_prev(2, 0) + internal_2;
    const double internal_38 = Fx(1, 3)*V_xx_prev(1, 0) + Fx(2, 3)*V_xx_prev(2, 0) + Fx(3, 3)*V_xx_prev(3, 0) + internal_4;
    const double internal_50 = Fx(1, 5)*V_xx_prev(1, 0) + Fx(2, 5)*V_xx_prev(2, 0) + Fx(3, 5)*V_xx_prev(3, 0) + Fx(5, 5)*V_xx_prev(5, 0) + internal_7;
    const double internal_9 = Fx(1, 1)*V_xx_prev(1, 0) + internal_0;
    const double internal_22 = Sigma_(0)*g_x(0, 1)*g_x(0, 2) + Sigma_(1)*g_x(1, 1)*g_x(1, 2) + g_x(15, 2)*internal_12 + g_x(16, 2)*internal_13 + g_x(17, 2)*internal_14 + g_x(18, 2)*internal_15 + g_x(19, 2)*internal_16 + g_x(2, 2)*internal_17 + g_x(20, 2)*internal_18 + g_x(3, 2)*internal_19 + g_x(4, 2)*internal_20 + g_x(5, 2)*internal_21;
    const double internal_24 = g_x(17, 3)*internal_14 + g_x(18, 3)*internal_15 + g_x(19, 3)*internal_16 + g_x(2, 3)*internal_17 + g_x(20, 3)*internal_18 + g_x(3, 3)*internal_19 + g_x(4, 3)*internal_20 + g_x(5, 3)*internal_21;
    const double internal_35 = Sigma_(2)*g_x(2, 2)*g_x(2, 3) + Sigma_(3)*g_x(3, 2)*g_x(3, 3) + Sigma_(4)*g_x(4, 2)*g_x(4, 3) + Sigma_(5)*g_x(5, 2)*g_x(5, 3) + g_x(17, 3)*internal_31 + g_x(18, 3)*internal_32 + g_x(19, 3)*internal_33 + g_x(20, 3)*internal_34;

    // Evaluation of Matrix Q_xx_
    Q_xx_(0, 0) = pow(Fx(0, 0), 2)*V_xx_prev(0, 0);
    Q_xx_(0, 1) = Fx(0, 0)*internal_0 + Fx(1, 1)*internal_1;
    Q_xx_(0, 2) = Fx(0, 0)*internal_2 + Fx(2, 2)*internal_3;
    Q_xx_(0, 3) = Fx(0, 0)*internal_4 + Fx(1, 3)*internal_1 + Fx(2, 3)*internal_3 + Fx(3, 3)*internal_5;
    Q_xx_(0, 4) = Fx(3, 4)*internal_5 + Fx(4, 4)*internal_6;
    Q_xx_(0, 5) = Fx(0, 0)*internal_7 + Fx(1, 5)*internal_1 + Fx(2, 5)*internal_3 + Fx(3, 5)*internal_5 + Fx(5, 5)*internal_8;
    Q_xx_(1, 0) = Fx(0, 0)*internal_9;
    Q_xx_(1, 1) = Fx(0, 1)*internal_9 + Fx(1, 1)*internal_10 + Sigma_(0)*pow(g_x(0, 1), 2) + Sigma_(1)*pow(g_x(1, 1), 2) + Sigma_(15)*pow(g_x(15, 1), 2) + Sigma_(16)*pow(g_x(16, 1), 2) + Sigma_(17)*pow(g_x(17, 1), 2) + Sigma_(18)*pow(g_x(18, 1), 2) + Sigma_(19)*pow(g_x(19, 1), 2) + Sigma_(2)*pow(g_x(2, 1), 2) + Sigma_(20)*pow(g_x(20, 1), 2) + Sigma_(3)*pow(g_x(3, 1), 2) + Sigma_(4)*pow(g_x(4, 1), 2) + Sigma_(5)*pow(g_x(5, 1), 2) + l_xx(1, 1);
    Q_xx_(1, 2) = Fx(0, 2)*internal_9 + Fx(2, 2)*internal_11 + internal_22 + l_xx(1, 2);
    Q_xx_(1, 3) = Fx(0, 3)*internal_9 + Fx(1, 3)*internal_10 + Fx(2, 3)*internal_11 + Fx(3, 3)*internal_23 + internal_24;
    Q_xx_(1, 4) = Fx(3, 4)*internal_23 + Fx(4, 4)*internal_25;
    Q_xx_(1, 5) = Fx(0, 5)*internal_9 + Fx(1, 5)*internal_10 + Fx(2, 5)*internal_11 + Fx(3, 5)*internal_23 + Fx(5, 5)*internal_26;
    Q_xx_(2, 0) = Fx(0, 0)*internal_27;
    Q_xx_(2, 1) = Fx(0, 1)*internal_27 + Fx(1, 1)*internal_28 + internal_22 + l_xx(2, 1);
    Q_xx_(2, 2) = Fx(0, 2)*internal_27 + Fx(2, 2)*internal_29 + Sigma_(0)*pow(g_x(0, 2), 2) + Sigma_(1)*pow(g_x(1, 2), 2) + Sigma_(15)*pow(g_x(15, 2), 2) + Sigma_(16)*pow(g_x(16, 2), 2) + Sigma_(17)*pow(g_x(17, 2), 2) + Sigma_(18)*pow(g_x(18, 2), 2) + Sigma_(19)*pow(g_x(19, 2), 2) + Sigma_(2)*pow(g_x(2, 2), 2) + Sigma_(20)*pow(g_x(20, 2), 2) + Sigma_(3)*pow(g_x(3, 2), 2) + Sigma_(4)*pow(g_x(4, 2), 2) + Sigma_(5)*pow(g_x(5, 2), 2) + l_xx(2, 2);
    Q_xx_(2, 3) = Fx(0, 3)*internal_27 + Fx(1, 3)*internal_28 + Fx(2, 3)*internal_29 + Fx(3, 3)*internal_30 + internal_35;
    Q_xx_(2, 4) = Fx(3, 4)*internal_30 + Fx(4, 4)*internal_36;
    Q_xx_(2, 5) = Fx(0, 5)*internal_27 + Fx(1, 5)*internal_28 + Fx(2, 5)*internal_29 + Fx(3, 5)*internal_30 + Fx(5, 5)*internal_37;
    Q_xx_(3, 0) = Fx(0, 0)*internal_38;
    Q_xx_(3, 1) = Fx(0, 1)*internal_38 + Fx(1, 1)*internal_39 + internal_24;
    Q_xx_(3, 2) = Fx(0, 2)*internal_38 + Fx(2, 2)*internal_40 + internal_35;
    Q_xx_(3, 3) = Fx(0, 3)*internal_38 + Fx(1, 3)*internal_39 + Fx(2, 3)*internal_40 + Fx(3, 3)*internal_41 + Sigma_(10)*pow(g_x(10, 3), 2) + Sigma_(17)*pow(g_x(17, 3), 2) + Sigma_(18)*pow(g_x(18, 3), 2) + Sigma_(19)*pow(g_x(19, 3), 2) + Sigma_(2)*pow(g_x(2, 3), 2) + Sigma_(20)*pow(g_x(20, 3), 2) + Sigma_(3)*pow(g_x(3, 3), 2) + Sigma_(4)*pow(g_x(4, 3), 2) + Sigma_(5)*pow(g_x(5, 3), 2);
    Q_xx_(3, 4) = Fx(3, 4)*internal_41 + Fx(4, 4)*internal_42;
    Q_xx_(3, 5) = Fx(0, 5)*internal_38 + Fx(1, 5)*internal_39 + Fx(2, 5)*internal_40 + Fx(3, 5)*internal_41 + Fx(5, 5)*internal_43;
    Q_xx_(4, 0) = Fx(0, 0)*internal_44;
    Q_xx_(4, 1) = Fx(0, 1)*internal_44 + Fx(1, 1)*internal_45;
    Q_xx_(4, 2) = Fx(0, 2)*internal_44 + Fx(2, 2)*internal_46;
    Q_xx_(4, 3) = Fx(0, 3)*internal_44 + Fx(1, 3)*internal_45 + Fx(2, 3)*internal_46 + Fx(3, 3)*internal_47;
    Q_xx_(4, 4) = Fx(3, 4)*internal_47 + Fx(4, 4)*internal_48 + Sigma_(6)*pow(g_x(6, 4), 2) + Sigma_(7)*pow(g_x(7, 4), 2) + l_xx(4, 4);
    Q_xx_(4, 5) = Fx(0, 5)*internal_44 + Fx(1, 5)*internal_45 + Fx(2, 5)*internal_46 + Fx(3, 5)*internal_47 + Fx(5, 5)*internal_49;
    Q_xx_(5, 0) = Fx(0, 0)*internal_50;
    Q_xx_(5, 1) = Fx(0, 1)*internal_50 + Fx(1, 1)*internal_51;
    Q_xx_(5, 2) = Fx(0, 2)*internal_50 + Fx(2, 2)*internal_52;
    Q_xx_(5, 3) = Fx(0, 3)*internal_50 + Fx(1, 3)*internal_51 + Fx(2, 3)*internal_52 + Fx(3, 3)*internal_53;
    Q_xx_(5, 4) = Fx(3, 4)*internal_53 + Fx(4, 4)*internal_54;
    Q_xx_(5, 5) = Fx(0, 5)*internal_50 + Fx(1, 5)*internal_51 + Fx(2, 5)*internal_52 + Fx(3, 5)*internal_53 + Fx(5, 5)*internal_55 + Sigma_(8)*pow(g_x(8, 5), 2) + Sigma_(9)*pow(g_x(9, 5), 2) + l_xx(5, 5);

    // Evaluation of Matrix Q_xu_
    Q_xu_(0, 0) = Fu(4, 0)*internal_6;
    Q_xu_(0, 1) = Fu(5, 1)*internal_8;
    Q_xu_(1, 0) = Fu(4, 0)*internal_25;
    Q_xu_(1, 1) = Fu(5, 1)*internal_26;
    Q_xu_(1, 2) = g_u(15, 2)*internal_12 + g_u(16, 2)*internal_13 + g_u(17, 2)*internal_14 + g_u(18, 2)*internal_15 + g_u(19, 2)*internal_16 + g_u(20, 2)*internal_18;
    Q_xu_(2, 0) = Fu(4, 0)*internal_36;
    Q_xu_(2, 1) = Fu(5, 1)*internal_37;
    Q_xu_(2, 2) = Sigma_(15)*g_u(15, 2)*g_x(15, 2) + Sigma_(16)*g_u(16, 2)*g_x(16, 2) + g_u(17, 2)*internal_31 + g_u(18, 2)*internal_32 + g_u(19, 2)*internal_33 + g_u(20, 2)*internal_34;
    Q_xu_(3, 0) = Fu(4, 0)*internal_42;
    Q_xu_(3, 1) = Fu(5, 1)*internal_43;
    Q_xu_(3, 2) = Sigma_(17)*g_u(17, 2)*g_x(17, 3) + Sigma_(18)*g_u(18, 2)*g_x(18, 3) + Sigma_(19)*g_u(19, 2)*g_x(19, 3) + Sigma_(20)*g_u(20, 2)*g_x(20, 3);
    Q_xu_(4, 0) = Fu(4, 0)*internal_48;
    Q_xu_(4, 1) = Fu(5, 1)*internal_49;
    Q_xu_(5, 0) = Fu(4, 0)*internal_54;
    Q_xu_(5, 1) = Fu(5, 1)*internal_55;

    // Evaluation of Matrix Q_uu_
    Q_uu_(0, 0) = pow(Fu(4, 0), 2)*V_xx_prev(4, 4) + Sigma_(11)*pow(g_u(11, 0), 2) + Sigma_(12)*pow(g_u(12, 0), 2) + l_uu(0, 0);
    Q_uu_(0, 1) = V_xx_prev(4, 5)*internal_56;
    Q_uu_(1, 0) = V_xx_prev(5, 4)*internal_56;
    Q_uu_(1, 1) = pow(Fu(5, 1), 2)*V_xx_prev(5, 5) + Sigma_(13)*pow(g_u(13, 1), 2) + Sigma_(14)*pow(g_u(14, 1), 2) + l_uu(1, 1);
    Q_uu_(2, 2) = Sigma_(15)*pow(g_u(15, 2), 2) + Sigma_(16)*pow(g_u(16, 2), 2) + Sigma_(17)*pow(g_u(17, 2), 2) + Sigma_(18)*pow(g_u(18, 2), 2) + Sigma_(19)*pow(g_u(19, 2), 2) + Sigma_(20)*pow(g_u(20, 2), 2) + l_uu(2, 2);

  }

  virtual void updateQG(const Eigen::VectorXd& sl_x, const Eigen::VectorXd& sl_u, const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::VectorXd& r_f, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& V_xx_prev, const Eigen::VectorXd& V_x_prev) override {
    // Determine internal variables
    const double internal_0 = V_x_prev(0) + V_xx_prev(0, 0)*r_f(0) + V_xx_prev(0, 1)*r_f(1) + V_xx_prev(0, 2)*r_f(2) + V_xx_prev(0, 3)*r_f(3) + V_xx_prev(0, 4)*r_f(4) + V_xx_prev(0, 5)*r_f(5);
    const double internal_1 = V_x_prev(1) + V_xx_prev(1, 0)*r_f(0) + V_xx_prev(1, 1)*r_f(1) + V_xx_prev(1, 2)*r_f(2) + V_xx_prev(1, 3)*r_f(3) + V_xx_prev(1, 4)*r_f(4) + V_xx_prev(1, 5)*r_f(5);
    const double internal_2 = V_x_prev(2) + V_xx_prev(2, 0)*r_f(0) + V_xx_prev(2, 1)*r_f(1) + V_xx_prev(2, 2)*r_f(2) + V_xx_prev(2, 3)*r_f(3) + V_xx_prev(2, 4)*r_f(4) + V_xx_prev(2, 5)*r_f(5);
    const double internal_3 = V_x_prev(3) + V_xx_prev(3, 0)*r_f(0) + V_xx_prev(3, 1)*r_f(1) + V_xx_prev(3, 2)*r_f(2) + V_xx_prev(3, 3)*r_f(3) + V_xx_prev(3, 4)*r_f(4) + V_xx_prev(3, 5)*r_f(5);
    const double internal_4 = V_x_prev(4) + V_xx_prev(4, 0)*r_f(0) + V_xx_prev(4, 1)*r_f(1) + V_xx_prev(4, 2)*r_f(2) + V_xx_prev(4, 3)*r_f(3) + V_xx_prev(4, 4)*r_f(4) + V_xx_prev(4, 5)*r_f(5);
    const double internal_5 = V_x_prev(5) + V_xx_prev(5, 0)*r_f(0) + V_xx_prev(5, 1)*r_f(1) + V_xx_prev(5, 2)*r_f(2) + V_xx_prev(5, 3)*r_f(3) + V_xx_prev(5, 4)*r_f(4) + V_xx_prev(5, 5)*r_f(5);

    // Evaluation of Vector Q_x_
    Q_x_(0) = Fx(0, 0)*internal_0 + sl_x(0);
    Q_x_(1) = Fx(0, 1)*internal_0 + Fx(1, 1)*internal_1 + V_(0)*g_x(0, 1) + V_(1)*g_x(1, 1) + V_(15)*g_x(15, 1) + V_(16)*g_x(16, 1) + V_(17)*g_x(17, 1) + V_(18)*g_x(18, 1) + V_(19)*g_x(19, 1) + V_(2)*g_x(2, 1) + V_(20)*g_x(20, 1) + V_(3)*g_x(3, 1) + V_(4)*g_x(4, 1) + V_(5)*g_x(5, 1) + sl_x(1);
    Q_x_(2) = Fx(0, 2)*internal_0 + Fx(2, 2)*internal_2 + V_(0)*g_x(0, 2) + V_(1)*g_x(1, 2) + V_(15)*g_x(15, 2) + V_(16)*g_x(16, 2) + V_(17)*g_x(17, 2) + V_(18)*g_x(18, 2) + V_(19)*g_x(19, 2) + V_(2)*g_x(2, 2) + V_(20)*g_x(20, 2) + V_(3)*g_x(3, 2) + V_(4)*g_x(4, 2) + V_(5)*g_x(5, 2) + sl_x(2);
    Q_x_(3) = Fx(0, 3)*internal_0 + Fx(1, 3)*internal_1 + Fx(2, 3)*internal_2 + Fx(3, 3)*internal_3 + V_(10)*g_x(10, 3) + V_(17)*g_x(17, 3) + V_(18)*g_x(18, 3) + V_(19)*g_x(19, 3) + V_(2)*g_x(2, 3) + V_(20)*g_x(20, 3) + V_(3)*g_x(3, 3) + V_(4)*g_x(4, 3) + V_(5)*g_x(5, 3) + sl_x(3);
    Q_x_(4) = Fx(3, 4)*internal_3 + Fx(4, 4)*internal_4 + V_(6)*g_x(6, 4) + V_(7)*g_x(7, 4) + sl_x(4);
    Q_x_(5) = Fx(0, 5)*internal_0 + Fx(1, 5)*internal_1 + Fx(2, 5)*internal_2 + Fx(3, 5)*internal_3 + Fx(5, 5)*internal_5 + V_(8)*g_x(8, 5) + V_(9)*g_x(9, 5) + sl_x(5);

    // Evaluation of Vector Q_u_
    Q_u_(0) = Fu(4, 0)*internal_4 + V_(11)*g_u(11, 0) + V_(12)*g_u(12, 0) + sl_u(0);
    Q_u_(1) = Fu(5, 1)*internal_5 + V_(13)*g_u(13, 1) + V_(14)*g_u(14, 1) + sl_u(1);
    Q_u_(2) = V_(15)*g_u(15, 2) + V_(16)*g_u(16, 2) + V_(17)*g_u(17, 2) + V_(18)*g_u(18, 2) + V_(19)*g_u(19, 2) + V_(20)*g_u(20, 2) + sl_u(2);

  }

  virtual void updateQuuInv() override {
    // Determine internal variables
    const double internal_0 = 1.0/Q_uu_(0, 0);
    const double internal_1 = pow(Q_uu_(0, 1), 2);
    const double internal_2 = 1.0/(Q_uu_(1, 1) - internal_0*internal_1);
    const double internal_3 = -Q_uu_(0, 1)*internal_0*internal_2;

    // Evaluation of Matrix Q_uu_inv_
    Q_uu_inv_(0, 0) = internal_0 + internal_1*internal_2/pow(Q_uu_(0, 0), 2);
    Q_uu_inv_(0, 1) = internal_3;
    Q_uu_inv_(1, 0) = internal_3;
    Q_uu_inv_(1, 1) = internal_2;
    Q_uu_inv_(2, 2) = 1.0/Q_uu_(2, 2);

  }

  virtual void updateKVU(Eigen::MatrixXd& k_ux, Eigen::VectorXd& v_u) override {
    // Evaluation of Matrix k_ux
    k_ux(0, 0) = -Q_uu_inv_(0, 0)*Q_xu_(0, 0) - Q_uu_inv_(0, 1)*Q_xu_(0, 1);
    k_ux(0, 1) = -Q_uu_inv_(0, 0)*Q_xu_(1, 0) - Q_uu_inv_(0, 1)*Q_xu_(1, 1);
    k_ux(0, 2) = -Q_uu_inv_(0, 0)*Q_xu_(2, 0) - Q_uu_inv_(0, 1)*Q_xu_(2, 1);
    k_ux(0, 3) = -Q_uu_inv_(0, 0)*Q_xu_(3, 0) - Q_uu_inv_(0, 1)*Q_xu_(3, 1);
    k_ux(0, 4) = -Q_uu_inv_(0, 0)*Q_xu_(4, 0) - Q_uu_inv_(0, 1)*Q_xu_(4, 1);
    k_ux(0, 5) = -Q_uu_inv_(0, 0)*Q_xu_(5, 0) - Q_uu_inv_(0, 1)*Q_xu_(5, 1);
    k_ux(1, 0) = -Q_uu_inv_(0, 1)*Q_xu_(0, 0) - Q_uu_inv_(1, 1)*Q_xu_(0, 1);
    k_ux(1, 1) = -Q_uu_inv_(0, 1)*Q_xu_(1, 0) - Q_uu_inv_(1, 1)*Q_xu_(1, 1);
    k_ux(1, 2) = -Q_uu_inv_(0, 1)*Q_xu_(2, 0) - Q_uu_inv_(1, 1)*Q_xu_(2, 1);
    k_ux(1, 3) = -Q_uu_inv_(0, 1)*Q_xu_(3, 0) - Q_uu_inv_(1, 1)*Q_xu_(3, 1);
    k_ux(1, 4) = -Q_uu_inv_(0, 1)*Q_xu_(4, 0) - Q_uu_inv_(1, 1)*Q_xu_(4, 1);
    k_ux(1, 5) = -Q_uu_inv_(0, 1)*Q_xu_(5, 0) - Q_uu_inv_(1, 1)*Q_xu_(5, 1);
    k_ux(2, 1) = -Q_uu_inv_(2, 2)*Q_xu_(1, 2);
    k_ux(2, 2) = -Q_uu_inv_(2, 2)*Q_xu_(2, 2);
    k_ux(2, 3) = -Q_uu_inv_(2, 2)*Q_xu_(3, 2);

    // Evaluation of Vector v_u
    v_u(0) = -Q_u_(0)*Q_uu_inv_(0, 0) - Q_u_(1)*Q_uu_inv_(0, 1);
    v_u(1) = -Q_u_(0)*Q_uu_inv_(0, 1) - Q_u_(1)*Q_uu_inv_(1, 1);
    v_u(2) = -Q_u_(2)*Q_uu_inv_(2, 2);

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
    V_xx(0, 0) = Q_xu_(0, 0)*k_ux(0, 0) + Q_xu_(0, 1)*k_ux(1, 0) + Q_xx_(0, 0);
    V_xx(0, 1) = Q_xu_(0, 0)*k_ux(0, 1) + Q_xu_(0, 1)*k_ux(1, 1) + Q_xx_(0, 1);
    V_xx(0, 2) = Q_xu_(0, 0)*k_ux(0, 2) + Q_xu_(0, 1)*k_ux(1, 2) + Q_xx_(0, 2);
    V_xx(0, 3) = Q_xu_(0, 0)*k_ux(0, 3) + Q_xu_(0, 1)*k_ux(1, 3) + Q_xx_(0, 3);
    V_xx(0, 4) = Q_xu_(0, 0)*k_ux(0, 4) + Q_xu_(0, 1)*k_ux(1, 4) + Q_xx_(0, 4);
    V_xx(0, 5) = Q_xu_(0, 0)*k_ux(0, 5) + Q_xu_(0, 1)*k_ux(1, 5) + Q_xx_(0, 5);
    V_xx(1, 0) = Q_xu_(1, 0)*k_ux(0, 0) + Q_xu_(1, 1)*k_ux(1, 0) + Q_xx_(0, 1);
    V_xx(1, 1) = Q_xu_(1, 0)*k_ux(0, 1) + Q_xu_(1, 1)*k_ux(1, 1) + Q_xu_(1, 2)*k_ux(2, 1) + Q_xx_(1, 1);
    V_xx(1, 2) = Q_xu_(1, 0)*k_ux(0, 2) + Q_xu_(1, 1)*k_ux(1, 2) + Q_xu_(1, 2)*k_ux(2, 2) + Q_xx_(1, 2);
    V_xx(1, 3) = Q_xu_(1, 0)*k_ux(0, 3) + Q_xu_(1, 1)*k_ux(1, 3) + Q_xu_(1, 2)*k_ux(2, 3) + Q_xx_(1, 3);
    V_xx(1, 4) = Q_xu_(1, 0)*k_ux(0, 4) + Q_xu_(1, 1)*k_ux(1, 4) + Q_xx_(1, 4);
    V_xx(1, 5) = Q_xu_(1, 0)*k_ux(0, 5) + Q_xu_(1, 1)*k_ux(1, 5) + Q_xx_(1, 5);
    V_xx(2, 0) = Q_xu_(2, 0)*k_ux(0, 0) + Q_xu_(2, 1)*k_ux(1, 0) + Q_xx_(0, 2);
    V_xx(2, 1) = Q_xu_(2, 0)*k_ux(0, 1) + Q_xu_(2, 1)*k_ux(1, 1) + Q_xu_(2, 2)*k_ux(2, 1) + Q_xx_(1, 2);
    V_xx(2, 2) = Q_xu_(2, 0)*k_ux(0, 2) + Q_xu_(2, 1)*k_ux(1, 2) + Q_xu_(2, 2)*k_ux(2, 2) + Q_xx_(2, 2);
    V_xx(2, 3) = Q_xu_(2, 0)*k_ux(0, 3) + Q_xu_(2, 1)*k_ux(1, 3) + Q_xu_(2, 2)*k_ux(2, 3) + Q_xx_(2, 3);
    V_xx(2, 4) = Q_xu_(2, 0)*k_ux(0, 4) + Q_xu_(2, 1)*k_ux(1, 4) + Q_xx_(2, 4);
    V_xx(2, 5) = Q_xu_(2, 0)*k_ux(0, 5) + Q_xu_(2, 1)*k_ux(1, 5) + Q_xx_(2, 5);
    V_xx(3, 0) = Q_xu_(3, 0)*k_ux(0, 0) + Q_xu_(3, 1)*k_ux(1, 0) + Q_xx_(0, 3);
    V_xx(3, 1) = Q_xu_(3, 0)*k_ux(0, 1) + Q_xu_(3, 1)*k_ux(1, 1) + Q_xu_(3, 2)*k_ux(2, 1) + Q_xx_(1, 3);
    V_xx(3, 2) = Q_xu_(3, 0)*k_ux(0, 2) + Q_xu_(3, 1)*k_ux(1, 2) + Q_xu_(3, 2)*k_ux(2, 2) + Q_xx_(2, 3);
    V_xx(3, 3) = Q_xu_(3, 0)*k_ux(0, 3) + Q_xu_(3, 1)*k_ux(1, 3) + Q_xu_(3, 2)*k_ux(2, 3) + Q_xx_(3, 3);
    V_xx(3, 4) = Q_xu_(3, 0)*k_ux(0, 4) + Q_xu_(3, 1)*k_ux(1, 4) + Q_xx_(3, 4);
    V_xx(3, 5) = Q_xu_(3, 0)*k_ux(0, 5) + Q_xu_(3, 1)*k_ux(1, 5) + Q_xx_(3, 5);
    V_xx(4, 0) = Q_xu_(4, 0)*k_ux(0, 0) + Q_xu_(4, 1)*k_ux(1, 0) + Q_xx_(0, 4);
    V_xx(4, 1) = Q_xu_(4, 0)*k_ux(0, 1) + Q_xu_(4, 1)*k_ux(1, 1) + Q_xx_(1, 4);
    V_xx(4, 2) = Q_xu_(4, 0)*k_ux(0, 2) + Q_xu_(4, 1)*k_ux(1, 2) + Q_xx_(2, 4);
    V_xx(4, 3) = Q_xu_(4, 0)*k_ux(0, 3) + Q_xu_(4, 1)*k_ux(1, 3) + Q_xx_(3, 4);
    V_xx(4, 4) = Q_xu_(4, 0)*k_ux(0, 4) + Q_xu_(4, 1)*k_ux(1, 4) + Q_xx_(4, 4);
    V_xx(4, 5) = Q_xu_(4, 0)*k_ux(0, 5) + Q_xu_(4, 1)*k_ux(1, 5) + Q_xx_(4, 5);
    V_xx(5, 0) = Q_xu_(5, 0)*k_ux(0, 0) + Q_xu_(5, 1)*k_ux(1, 0) + Q_xx_(0, 5);
    V_xx(5, 1) = Q_xu_(5, 0)*k_ux(0, 1) + Q_xu_(5, 1)*k_ux(1, 1) + Q_xx_(1, 5);
    V_xx(5, 2) = Q_xu_(5, 0)*k_ux(0, 2) + Q_xu_(5, 1)*k_ux(1, 2) + Q_xx_(2, 5);
    V_xx(5, 3) = Q_xu_(5, 0)*k_ux(0, 3) + Q_xu_(5, 1)*k_ux(1, 3) + Q_xx_(3, 5);
    V_xx(5, 4) = Q_xu_(5, 0)*k_ux(0, 4) + Q_xu_(5, 1)*k_ux(1, 4) + Q_xx_(4, 5);
    V_xx(5, 5) = Q_xu_(5, 0)*k_ux(0, 5) + Q_xu_(5, 1)*k_ux(1, 5) + Q_xx_(5, 5);

    // Evaluation of Vector V_x
    V_x(0) = Q_x_(0) + Q_xu_(0, 0)*v_u(0) + Q_xu_(0, 1)*v_u(1);
    V_x(1) = Q_x_(1) + Q_xu_(1, 0)*v_u(0) + Q_xu_(1, 1)*v_u(1) + Q_xu_(1, 2)*v_u(2);
    V_x(2) = Q_x_(2) + Q_xu_(2, 0)*v_u(0) + Q_xu_(2, 1)*v_u(1) + Q_xu_(2, 2)*v_u(2);
    V_x(3) = Q_x_(3) + Q_xu_(3, 0)*v_u(0) + Q_xu_(3, 1)*v_u(1) + Q_xu_(3, 2)*v_u(2);
    V_x(4) = Q_x_(4) + Q_xu_(4, 0)*v_u(0) + Q_xu_(4, 1)*v_u(1);
    V_x(5) = Q_x_(5) + Q_xu_(5, 0)*v_u(0) + Q_xu_(5, 1)*v_u(1);

  }

  virtual void updateQnx(const Eigen::VectorXd& r_f, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& h_u, const Eigen::MatrixXd& k_ux, const Eigen::MatrixXd& k_ex, const Eigen::MatrixXd& k_ue, const Eigen::VectorXd& v_u, const Eigen::VectorXd& v_e) override {
    // Determine internal variables
    const double internal_0 = -Q_uu_(0, 1);

    // Evaluation of Matrix Q_un_
    Q_un_(0, 4) = -Fu(4, 0)*Q_uu_inv_(0, 0);
    Q_un_(0, 5) = -Fu(5, 1)*Q_uu_inv_(0, 1);
    Q_un_(1, 4) = -Fu(4, 0)*Q_uu_inv_(0, 1);
    Q_un_(1, 5) = -Fu(5, 1)*Q_uu_inv_(1, 1);

    // Evaluation of Matrix Q_nuu_
    Q_nuu_(0, 0) = -Q_uu_(0, 0);
    Q_nuu_(0, 1) = internal_0;
    Q_nuu_(1, 0) = internal_0;
    Q_nuu_(1, 1) = -Q_uu_(1, 1);
    Q_nuu_(2, 2) = -Q_uu_(2, 2);

    // Evaluation of Matrix Q_vnx_
    Q_vnx_(0, 0) = Fx(0, 0);
    Q_vnx_(0, 1) = Fx(0, 1);
    Q_vnx_(0, 2) = Fx(0, 2);
    Q_vnx_(0, 3) = Fx(0, 3);
    Q_vnx_(0, 5) = Fx(0, 5);
    Q_vnx_(1, 1) = Fx(1, 1);
    Q_vnx_(1, 3) = Fx(1, 3);
    Q_vnx_(1, 5) = Fx(1, 5);
    Q_vnx_(2, 2) = Fx(2, 2);
    Q_vnx_(2, 3) = Fx(2, 3);
    Q_vnx_(2, 5) = Fx(2, 5);
    Q_vnx_(3, 3) = Fx(3, 3);
    Q_vnx_(3, 4) = Fx(3, 4);
    Q_vnx_(3, 5) = Fx(3, 5);
    Q_vnx_(4, 0) = Fu(4, 0)*k_ux(0, 0);
    Q_vnx_(4, 1) = Fu(4, 0)*k_ux(0, 1);
    Q_vnx_(4, 2) = Fu(4, 0)*k_ux(0, 2);
    Q_vnx_(4, 3) = Fu(4, 0)*k_ux(0, 3);
    Q_vnx_(4, 4) = Fu(4, 0)*k_ux(0, 4) + Fx(4, 4);
    Q_vnx_(4, 5) = Fu(4, 0)*k_ux(0, 5);
    Q_vnx_(5, 0) = Fu(5, 1)*k_ux(1, 0);
    Q_vnx_(5, 1) = Fu(5, 1)*k_ux(1, 1);
    Q_vnx_(5, 2) = Fu(5, 1)*k_ux(1, 2);
    Q_vnx_(5, 3) = Fu(5, 1)*k_ux(1, 3);
    Q_vnx_(5, 4) = Fu(5, 1)*k_ux(1, 4);
    Q_vnx_(5, 5) = Fu(5, 1)*k_ux(1, 5) + Fx(5, 5);

    // Evaluation of Vector Q_vn_
    Q_vn_(0) = r_f(0);
    Q_vn_(1) = r_f(1);
    Q_vn_(2) = r_f(2);
    Q_vn_(3) = r_f(3);
    Q_vn_(4) = Fu(4, 0)*v_u(0) + r_f(4);
    Q_vn_(5) = Fu(5, 1)*v_u(1) + r_f(5);

  }

};

template <>
class ParkingGeneralIpmEvaluator<StageType::INITIAL, OcpConfig::FORWARD_EULER> : public IpmEvaluator {
 public:
  ParkingGeneralIpmEvaluator() : IpmEvaluator(6, 3, 4, 0, 6) {}
  virtual ~ParkingGeneralIpmEvaluator() = default;

  virtual void updateLX(const Eigen::VectorXd& l_x, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& h_x, const Eigen::MatrixXd& m_x, const Eigen::VectorXd& p, const Eigen::VectorXd& lambda, const Eigen::VectorXd& eta, const Eigen::VectorXd& nu, Eigen::VectorXd& sl_x) const override {
    // Evaluation of Vector sl_x
    sl_x(0) = Fx(0, 0)*p(0) + m_x(0, 0)*nu(0);
    sl_x(1) = Fx(0, 1)*p(0) + Fx(1, 1)*p(1) + l_x(1) + m_x(1, 1)*nu(1);
    sl_x(2) = Fx(0, 2)*p(0) + Fx(2, 2)*p(2) + l_x(2) + m_x(2, 2)*nu(2);
    sl_x(3) = Fx(0, 3)*p(0) + Fx(1, 3)*p(1) + Fx(2, 3)*p(2) + Fx(3, 3)*p(3) + m_x(3, 3)*nu(3);
    sl_x(4) = Fx(3, 4)*p(3) + Fx(4, 4)*p(4) + l_x(4) + m_x(4, 4)*nu(4);
    sl_x(5) = Fx(0, 5)*p(0) + Fx(1, 5)*p(1) + Fx(2, 5)*p(2) + Fx(3, 5)*p(3) + Fx(5, 5)*p(5) + l_x(5) + m_x(5, 5)*nu(5);

  }

  virtual void updateLU(const Eigen::VectorXd& l_u, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& g_u, const Eigen::MatrixXd& h_u, const Eigen::VectorXd& p, const Eigen::VectorXd& lambda, const Eigen::VectorXd& eta, Eigen::VectorXd& sl_u) const override {
    // Evaluation of Vector sl_u
    sl_u(0) = Fu(4, 0)*p(4) + g_u(0, 0)*lambda(0) + g_u(1, 0)*lambda(1) + l_u(0);
    sl_u(1) = Fu(5, 1)*p(5) + g_u(2, 1)*lambda(2) + g_u(3, 1)*lambda(3) + l_u(1);
    sl_u(2) = l_u(2);

  }

  virtual void updateQH(const Eigen::MatrixXd& l_xx, const Eigen::MatrixXd& l_xu, const Eigen::MatrixXd& l_uu, const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& V_xx_prev) override {
    // Determine internal variables
    const double internal_0 = Fx(0, 1)*V_xx_prev(0, 0);
    const double internal_1 = Fx(0, 0)*V_xx_prev(0, 1);
    const double internal_2 = Fx(0, 2)*V_xx_prev(0, 0);
    const double internal_3 = Fx(0, 0)*V_xx_prev(0, 2);
    const double internal_39 = Fu(4, 0)*Fu(5, 1);
    const double internal_4 = Fx(0, 3)*V_xx_prev(0, 0);
    const double internal_5 = Fx(0, 0)*V_xx_prev(0, 3);
    const double internal_6 = Fx(0, 0)*V_xx_prev(0, 4);
    const double internal_7 = Fx(0, 5)*V_xx_prev(0, 0);
    const double internal_8 = Fx(0, 0)*V_xx_prev(0, 5);
    const double internal_10 = Fx(0, 1)*V_xx_prev(0, 1) + Fx(1, 1)*V_xx_prev(1, 1);
    const double internal_11 = Fx(0, 1)*V_xx_prev(0, 2) + Fx(1, 1)*V_xx_prev(1, 2);
    const double internal_12 = Fx(0, 1)*V_xx_prev(0, 3) + Fx(1, 1)*V_xx_prev(1, 3);
    const double internal_13 = Fx(0, 1)*V_xx_prev(0, 4) + Fx(1, 1)*V_xx_prev(1, 4);
    const double internal_14 = Fx(0, 1)*V_xx_prev(0, 5) + Fx(1, 1)*V_xx_prev(1, 5);
    const double internal_16 = Fx(0, 2)*V_xx_prev(0, 1) + Fx(2, 2)*V_xx_prev(2, 1);
    const double internal_17 = Fx(0, 2)*V_xx_prev(0, 2) + Fx(2, 2)*V_xx_prev(2, 2);
    const double internal_18 = Fx(0, 2)*V_xx_prev(0, 3) + Fx(2, 2)*V_xx_prev(2, 3);
    const double internal_19 = Fx(0, 2)*V_xx_prev(0, 4) + Fx(2, 2)*V_xx_prev(2, 4);
    const double internal_20 = Fx(0, 2)*V_xx_prev(0, 5) + Fx(2, 2)*V_xx_prev(2, 5);
    const double internal_22 = Fx(0, 3)*V_xx_prev(0, 1) + Fx(1, 3)*V_xx_prev(1, 1) + Fx(2, 3)*V_xx_prev(2, 1) + Fx(3, 3)*V_xx_prev(3, 1);
    const double internal_23 = Fx(0, 3)*V_xx_prev(0, 2) + Fx(1, 3)*V_xx_prev(1, 2) + Fx(2, 3)*V_xx_prev(2, 2) + Fx(3, 3)*V_xx_prev(3, 2);
    const double internal_24 = Fx(0, 3)*V_xx_prev(0, 3) + Fx(1, 3)*V_xx_prev(1, 3) + Fx(2, 3)*V_xx_prev(2, 3) + Fx(3, 3)*V_xx_prev(3, 3);
    const double internal_25 = Fx(0, 3)*V_xx_prev(0, 4) + Fx(1, 3)*V_xx_prev(1, 4) + Fx(2, 3)*V_xx_prev(2, 4) + Fx(3, 3)*V_xx_prev(3, 4);
    const double internal_26 = Fx(0, 3)*V_xx_prev(0, 5) + Fx(1, 3)*V_xx_prev(1, 5) + Fx(2, 3)*V_xx_prev(2, 5) + Fx(3, 3)*V_xx_prev(3, 5);
    const double internal_27 = Fx(3, 4)*V_xx_prev(3, 0) + Fx(4, 4)*V_xx_prev(4, 0);
    const double internal_28 = Fx(3, 4)*V_xx_prev(3, 1) + Fx(4, 4)*V_xx_prev(4, 1);
    const double internal_29 = Fx(3, 4)*V_xx_prev(3, 2) + Fx(4, 4)*V_xx_prev(4, 2);
    const double internal_30 = Fx(3, 4)*V_xx_prev(3, 3) + Fx(4, 4)*V_xx_prev(4, 3);
    const double internal_31 = Fx(3, 4)*V_xx_prev(3, 4) + Fx(4, 4)*V_xx_prev(4, 4);
    const double internal_32 = Fx(3, 4)*V_xx_prev(3, 5) + Fx(4, 4)*V_xx_prev(4, 5);
    const double internal_34 = Fx(0, 5)*V_xx_prev(0, 1) + Fx(1, 5)*V_xx_prev(1, 1) + Fx(2, 5)*V_xx_prev(2, 1) + Fx(3, 5)*V_xx_prev(3, 1) + Fx(5, 5)*V_xx_prev(5, 1);
    const double internal_35 = Fx(0, 5)*V_xx_prev(0, 2) + Fx(1, 5)*V_xx_prev(1, 2) + Fx(2, 5)*V_xx_prev(2, 2) + Fx(3, 5)*V_xx_prev(3, 2) + Fx(5, 5)*V_xx_prev(5, 2);
    const double internal_36 = Fx(0, 5)*V_xx_prev(0, 3) + Fx(1, 5)*V_xx_prev(1, 3) + Fx(2, 5)*V_xx_prev(2, 3) + Fx(3, 5)*V_xx_prev(3, 3) + Fx(5, 5)*V_xx_prev(5, 3);
    const double internal_37 = Fx(0, 5)*V_xx_prev(0, 4) + Fx(1, 5)*V_xx_prev(1, 4) + Fx(2, 5)*V_xx_prev(2, 4) + Fx(3, 5)*V_xx_prev(3, 4) + Fx(5, 5)*V_xx_prev(5, 4);
    const double internal_38 = Fx(0, 5)*V_xx_prev(0, 5) + Fx(1, 5)*V_xx_prev(1, 5) + Fx(2, 5)*V_xx_prev(2, 5) + Fx(3, 5)*V_xx_prev(3, 5) + Fx(5, 5)*V_xx_prev(5, 5);
    const double internal_15 = Fx(2, 2)*V_xx_prev(2, 0) + internal_2;
    const double internal_21 = Fx(1, 3)*V_xx_prev(1, 0) + Fx(2, 3)*V_xx_prev(2, 0) + Fx(3, 3)*V_xx_prev(3, 0) + internal_4;
    const double internal_33 = Fx(1, 5)*V_xx_prev(1, 0) + Fx(2, 5)*V_xx_prev(2, 0) + Fx(3, 5)*V_xx_prev(3, 0) + Fx(5, 5)*V_xx_prev(5, 0) + internal_7;
    const double internal_9 = Fx(1, 1)*V_xx_prev(1, 0) + internal_0;

    // Evaluation of Matrix Q_xx_
    Q_xx_(0, 0) = pow(Fx(0, 0), 2)*V_xx_prev(0, 0);
    Q_xx_(0, 1) = Fx(0, 0)*internal_0 + Fx(1, 1)*internal_1;
    Q_xx_(0, 2) = Fx(0, 0)*internal_2 + Fx(2, 2)*internal_3;
    Q_xx_(0, 3) = Fx(0, 0)*internal_4 + Fx(1, 3)*internal_1 + Fx(2, 3)*internal_3 + Fx(3, 3)*internal_5;
    Q_xx_(0, 4) = Fx(3, 4)*internal_5 + Fx(4, 4)*internal_6;
    Q_xx_(0, 5) = Fx(0, 0)*internal_7 + Fx(1, 5)*internal_1 + Fx(2, 5)*internal_3 + Fx(3, 5)*internal_5 + Fx(5, 5)*internal_8;
    Q_xx_(1, 0) = Fx(0, 0)*internal_9;
    Q_xx_(1, 1) = Fx(0, 1)*internal_9 + Fx(1, 1)*internal_10 + l_xx(1, 1);
    Q_xx_(1, 2) = Fx(0, 2)*internal_9 + Fx(2, 2)*internal_11 + l_xx(1, 2);
    Q_xx_(1, 3) = Fx(0, 3)*internal_9 + Fx(1, 3)*internal_10 + Fx(2, 3)*internal_11 + Fx(3, 3)*internal_12;
    Q_xx_(1, 4) = Fx(3, 4)*internal_12 + Fx(4, 4)*internal_13;
    Q_xx_(1, 5) = Fx(0, 5)*internal_9 + Fx(1, 5)*internal_10 + Fx(2, 5)*internal_11 + Fx(3, 5)*internal_12 + Fx(5, 5)*internal_14;
    Q_xx_(2, 0) = Fx(0, 0)*internal_15;
    Q_xx_(2, 1) = Fx(0, 1)*internal_15 + Fx(1, 1)*internal_16 + l_xx(2, 1);
    Q_xx_(2, 2) = Fx(0, 2)*internal_15 + Fx(2, 2)*internal_17 + l_xx(2, 2);
    Q_xx_(2, 3) = Fx(0, 3)*internal_15 + Fx(1, 3)*internal_16 + Fx(2, 3)*internal_17 + Fx(3, 3)*internal_18;
    Q_xx_(2, 4) = Fx(3, 4)*internal_18 + Fx(4, 4)*internal_19;
    Q_xx_(2, 5) = Fx(0, 5)*internal_15 + Fx(1, 5)*internal_16 + Fx(2, 5)*internal_17 + Fx(3, 5)*internal_18 + Fx(5, 5)*internal_20;
    Q_xx_(3, 0) = Fx(0, 0)*internal_21;
    Q_xx_(3, 1) = Fx(0, 1)*internal_21 + Fx(1, 1)*internal_22;
    Q_xx_(3, 2) = Fx(0, 2)*internal_21 + Fx(2, 2)*internal_23;
    Q_xx_(3, 3) = Fx(0, 3)*internal_21 + Fx(1, 3)*internal_22 + Fx(2, 3)*internal_23 + Fx(3, 3)*internal_24;
    Q_xx_(3, 4) = Fx(3, 4)*internal_24 + Fx(4, 4)*internal_25;
    Q_xx_(3, 5) = Fx(0, 5)*internal_21 + Fx(1, 5)*internal_22 + Fx(2, 5)*internal_23 + Fx(3, 5)*internal_24 + Fx(5, 5)*internal_26;
    Q_xx_(4, 0) = Fx(0, 0)*internal_27;
    Q_xx_(4, 1) = Fx(0, 1)*internal_27 + Fx(1, 1)*internal_28;
    Q_xx_(4, 2) = Fx(0, 2)*internal_27 + Fx(2, 2)*internal_29;
    Q_xx_(4, 3) = Fx(0, 3)*internal_27 + Fx(1, 3)*internal_28 + Fx(2, 3)*internal_29 + Fx(3, 3)*internal_30;
    Q_xx_(4, 4) = Fx(3, 4)*internal_30 + Fx(4, 4)*internal_31 + l_xx(4, 4);
    Q_xx_(4, 5) = Fx(0, 5)*internal_27 + Fx(1, 5)*internal_28 + Fx(2, 5)*internal_29 + Fx(3, 5)*internal_30 + Fx(5, 5)*internal_32;
    Q_xx_(5, 0) = Fx(0, 0)*internal_33;
    Q_xx_(5, 1) = Fx(0, 1)*internal_33 + Fx(1, 1)*internal_34;
    Q_xx_(5, 2) = Fx(0, 2)*internal_33 + Fx(2, 2)*internal_35;
    Q_xx_(5, 3) = Fx(0, 3)*internal_33 + Fx(1, 3)*internal_34 + Fx(2, 3)*internal_35 + Fx(3, 3)*internal_36;
    Q_xx_(5, 4) = Fx(3, 4)*internal_36 + Fx(4, 4)*internal_37;
    Q_xx_(5, 5) = Fx(0, 5)*internal_33 + Fx(1, 5)*internal_34 + Fx(2, 5)*internal_35 + Fx(3, 5)*internal_36 + Fx(5, 5)*internal_38 + l_xx(5, 5);

    // Evaluation of Matrix Q_xu_
    Q_xu_(0, 0) = Fu(4, 0)*internal_6;
    Q_xu_(0, 1) = Fu(5, 1)*internal_8;
    Q_xu_(1, 0) = Fu(4, 0)*internal_13;
    Q_xu_(1, 1) = Fu(5, 1)*internal_14;
    Q_xu_(2, 0) = Fu(4, 0)*internal_19;
    Q_xu_(2, 1) = Fu(5, 1)*internal_20;
    Q_xu_(3, 0) = Fu(4, 0)*internal_25;
    Q_xu_(3, 1) = Fu(5, 1)*internal_26;
    Q_xu_(4, 0) = Fu(4, 0)*internal_31;
    Q_xu_(4, 1) = Fu(5, 1)*internal_32;
    Q_xu_(5, 0) = Fu(4, 0)*internal_37;
    Q_xu_(5, 1) = Fu(5, 1)*internal_38;

    // Evaluation of Matrix Q_uu_
    Q_uu_(0, 0) = pow(Fu(4, 0), 2)*V_xx_prev(4, 4) + Sigma_(0)*pow(g_u(0, 0), 2) + Sigma_(1)*pow(g_u(1, 0), 2) + l_uu(0, 0);
    Q_uu_(0, 1) = V_xx_prev(4, 5)*internal_39;
    Q_uu_(1, 0) = V_xx_prev(5, 4)*internal_39;
    Q_uu_(1, 1) = pow(Fu(5, 1), 2)*V_xx_prev(5, 5) + Sigma_(2)*pow(g_u(2, 1), 2) + Sigma_(3)*pow(g_u(3, 1), 2) + l_uu(1, 1);
    Q_uu_(2, 2) = l_uu(2, 2);

  }

  virtual void updateQG(const Eigen::VectorXd& sl_x, const Eigen::VectorXd& sl_u, const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::VectorXd& r_f, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& V_xx_prev, const Eigen::VectorXd& V_x_prev) override {
    // Determine internal variables
    const double internal_0 = V_x_prev(0) + V_xx_prev(0, 0)*r_f(0) + V_xx_prev(0, 1)*r_f(1) + V_xx_prev(0, 2)*r_f(2) + V_xx_prev(0, 3)*r_f(3) + V_xx_prev(0, 4)*r_f(4) + V_xx_prev(0, 5)*r_f(5);
    const double internal_1 = V_x_prev(1) + V_xx_prev(1, 0)*r_f(0) + V_xx_prev(1, 1)*r_f(1) + V_xx_prev(1, 2)*r_f(2) + V_xx_prev(1, 3)*r_f(3) + V_xx_prev(1, 4)*r_f(4) + V_xx_prev(1, 5)*r_f(5);
    const double internal_2 = V_x_prev(2) + V_xx_prev(2, 0)*r_f(0) + V_xx_prev(2, 1)*r_f(1) + V_xx_prev(2, 2)*r_f(2) + V_xx_prev(2, 3)*r_f(3) + V_xx_prev(2, 4)*r_f(4) + V_xx_prev(2, 5)*r_f(5);
    const double internal_3 = V_x_prev(3) + V_xx_prev(3, 0)*r_f(0) + V_xx_prev(3, 1)*r_f(1) + V_xx_prev(3, 2)*r_f(2) + V_xx_prev(3, 3)*r_f(3) + V_xx_prev(3, 4)*r_f(4) + V_xx_prev(3, 5)*r_f(5);
    const double internal_4 = V_x_prev(4) + V_xx_prev(4, 0)*r_f(0) + V_xx_prev(4, 1)*r_f(1) + V_xx_prev(4, 2)*r_f(2) + V_xx_prev(4, 3)*r_f(3) + V_xx_prev(4, 4)*r_f(4) + V_xx_prev(4, 5)*r_f(5);
    const double internal_5 = V_x_prev(5) + V_xx_prev(5, 0)*r_f(0) + V_xx_prev(5, 1)*r_f(1) + V_xx_prev(5, 2)*r_f(2) + V_xx_prev(5, 3)*r_f(3) + V_xx_prev(5, 4)*r_f(4) + V_xx_prev(5, 5)*r_f(5);

    // Evaluation of Vector Q_x_
    Q_x_(0) = Fx(0, 0)*internal_0 + sl_x(0);
    Q_x_(1) = Fx(0, 1)*internal_0 + Fx(1, 1)*internal_1 + sl_x(1);
    Q_x_(2) = Fx(0, 2)*internal_0 + Fx(2, 2)*internal_2 + sl_x(2);
    Q_x_(3) = Fx(0, 3)*internal_0 + Fx(1, 3)*internal_1 + Fx(2, 3)*internal_2 + Fx(3, 3)*internal_3 + sl_x(3);
    Q_x_(4) = Fx(3, 4)*internal_3 + Fx(4, 4)*internal_4 + sl_x(4);
    Q_x_(5) = Fx(0, 5)*internal_0 + Fx(1, 5)*internal_1 + Fx(2, 5)*internal_2 + Fx(3, 5)*internal_3 + Fx(5, 5)*internal_5 + sl_x(5);

    // Evaluation of Vector Q_u_
    Q_u_(0) = Fu(4, 0)*internal_4 + V_(0)*g_u(0, 0) + V_(1)*g_u(1, 0) + sl_u(0);
    Q_u_(1) = Fu(5, 1)*internal_5 + V_(2)*g_u(2, 1) + V_(3)*g_u(3, 1) + sl_u(1);
    Q_u_(2) = sl_u(2);

  }

  virtual void updateQuuInv() override {
    // Determine internal variables
    const double internal_0 = 1.0/Q_uu_(0, 0);
    const double internal_1 = pow(Q_uu_(0, 1), 2);
    const double internal_2 = 1.0/(Q_uu_(1, 1) - internal_0*internal_1);
    const double internal_3 = -Q_uu_(0, 1)*internal_0*internal_2;

    // Evaluation of Matrix Q_uu_inv_
    Q_uu_inv_(0, 0) = internal_0 + internal_1*internal_2/pow(Q_uu_(0, 0), 2);
    Q_uu_inv_(0, 1) = internal_3;
    Q_uu_inv_(1, 0) = internal_3;
    Q_uu_inv_(1, 1) = internal_2;
    Q_uu_inv_(2, 2) = 1.0/Q_uu_(2, 2);

  }

  virtual void updateKVU(Eigen::MatrixXd& k_ux, Eigen::VectorXd& v_u) override {
    // Evaluation of Matrix k_ux
    k_ux(0, 0) = -Q_uu_inv_(0, 0)*Q_xu_(0, 0) - Q_uu_inv_(0, 1)*Q_xu_(0, 1);
    k_ux(0, 1) = -Q_uu_inv_(0, 0)*Q_xu_(1, 0) - Q_uu_inv_(0, 1)*Q_xu_(1, 1);
    k_ux(0, 2) = -Q_uu_inv_(0, 0)*Q_xu_(2, 0) - Q_uu_inv_(0, 1)*Q_xu_(2, 1);
    k_ux(0, 3) = -Q_uu_inv_(0, 0)*Q_xu_(3, 0) - Q_uu_inv_(0, 1)*Q_xu_(3, 1);
    k_ux(0, 4) = -Q_uu_inv_(0, 0)*Q_xu_(4, 0) - Q_uu_inv_(0, 1)*Q_xu_(4, 1);
    k_ux(0, 5) = -Q_uu_inv_(0, 0)*Q_xu_(5, 0) - Q_uu_inv_(0, 1)*Q_xu_(5, 1);
    k_ux(1, 0) = -Q_uu_inv_(0, 1)*Q_xu_(0, 0) - Q_uu_inv_(1, 1)*Q_xu_(0, 1);
    k_ux(1, 1) = -Q_uu_inv_(0, 1)*Q_xu_(1, 0) - Q_uu_inv_(1, 1)*Q_xu_(1, 1);
    k_ux(1, 2) = -Q_uu_inv_(0, 1)*Q_xu_(2, 0) - Q_uu_inv_(1, 1)*Q_xu_(2, 1);
    k_ux(1, 3) = -Q_uu_inv_(0, 1)*Q_xu_(3, 0) - Q_uu_inv_(1, 1)*Q_xu_(3, 1);
    k_ux(1, 4) = -Q_uu_inv_(0, 1)*Q_xu_(4, 0) - Q_uu_inv_(1, 1)*Q_xu_(4, 1);
    k_ux(1, 5) = -Q_uu_inv_(0, 1)*Q_xu_(5, 0) - Q_uu_inv_(1, 1)*Q_xu_(5, 1);

    // Evaluation of Vector v_u
    v_u(0) = -Q_u_(0)*Q_uu_inv_(0, 0) - Q_u_(1)*Q_uu_inv_(0, 1);
    v_u(1) = -Q_u_(0)*Q_uu_inv_(0, 1) - Q_u_(1)*Q_uu_inv_(1, 1);
    v_u(2) = -Q_u_(2)*Q_uu_inv_(2, 2);

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
    V_xx(0, 0) = Q_xu_(0, 0)*k_ux(0, 0) + Q_xu_(0, 1)*k_ux(1, 0) + Q_xx_(0, 0);
    V_xx(0, 1) = Q_xu_(0, 0)*k_ux(0, 1) + Q_xu_(0, 1)*k_ux(1, 1) + Q_xx_(0, 1);
    V_xx(0, 2) = Q_xu_(0, 0)*k_ux(0, 2) + Q_xu_(0, 1)*k_ux(1, 2) + Q_xx_(0, 2);
    V_xx(0, 3) = Q_xu_(0, 0)*k_ux(0, 3) + Q_xu_(0, 1)*k_ux(1, 3) + Q_xx_(0, 3);
    V_xx(0, 4) = Q_xu_(0, 0)*k_ux(0, 4) + Q_xu_(0, 1)*k_ux(1, 4) + Q_xx_(0, 4);
    V_xx(0, 5) = Q_xu_(0, 0)*k_ux(0, 5) + Q_xu_(0, 1)*k_ux(1, 5) + Q_xx_(0, 5);
    V_xx(1, 0) = Q_xu_(1, 0)*k_ux(0, 0) + Q_xu_(1, 1)*k_ux(1, 0) + Q_xx_(0, 1);
    V_xx(1, 1) = Q_xu_(1, 0)*k_ux(0, 1) + Q_xu_(1, 1)*k_ux(1, 1) + Q_xx_(1, 1);
    V_xx(1, 2) = Q_xu_(1, 0)*k_ux(0, 2) + Q_xu_(1, 1)*k_ux(1, 2) + Q_xx_(1, 2);
    V_xx(1, 3) = Q_xu_(1, 0)*k_ux(0, 3) + Q_xu_(1, 1)*k_ux(1, 3) + Q_xx_(1, 3);
    V_xx(1, 4) = Q_xu_(1, 0)*k_ux(0, 4) + Q_xu_(1, 1)*k_ux(1, 4) + Q_xx_(1, 4);
    V_xx(1, 5) = Q_xu_(1, 0)*k_ux(0, 5) + Q_xu_(1, 1)*k_ux(1, 5) + Q_xx_(1, 5);
    V_xx(2, 0) = Q_xu_(2, 0)*k_ux(0, 0) + Q_xu_(2, 1)*k_ux(1, 0) + Q_xx_(0, 2);
    V_xx(2, 1) = Q_xu_(2, 0)*k_ux(0, 1) + Q_xu_(2, 1)*k_ux(1, 1) + Q_xx_(1, 2);
    V_xx(2, 2) = Q_xu_(2, 0)*k_ux(0, 2) + Q_xu_(2, 1)*k_ux(1, 2) + Q_xx_(2, 2);
    V_xx(2, 3) = Q_xu_(2, 0)*k_ux(0, 3) + Q_xu_(2, 1)*k_ux(1, 3) + Q_xx_(2, 3);
    V_xx(2, 4) = Q_xu_(2, 0)*k_ux(0, 4) + Q_xu_(2, 1)*k_ux(1, 4) + Q_xx_(2, 4);
    V_xx(2, 5) = Q_xu_(2, 0)*k_ux(0, 5) + Q_xu_(2, 1)*k_ux(1, 5) + Q_xx_(2, 5);
    V_xx(3, 0) = Q_xu_(3, 0)*k_ux(0, 0) + Q_xu_(3, 1)*k_ux(1, 0) + Q_xx_(0, 3);
    V_xx(3, 1) = Q_xu_(3, 0)*k_ux(0, 1) + Q_xu_(3, 1)*k_ux(1, 1) + Q_xx_(1, 3);
    V_xx(3, 2) = Q_xu_(3, 0)*k_ux(0, 2) + Q_xu_(3, 1)*k_ux(1, 2) + Q_xx_(2, 3);
    V_xx(3, 3) = Q_xu_(3, 0)*k_ux(0, 3) + Q_xu_(3, 1)*k_ux(1, 3) + Q_xx_(3, 3);
    V_xx(3, 4) = Q_xu_(3, 0)*k_ux(0, 4) + Q_xu_(3, 1)*k_ux(1, 4) + Q_xx_(3, 4);
    V_xx(3, 5) = Q_xu_(3, 0)*k_ux(0, 5) + Q_xu_(3, 1)*k_ux(1, 5) + Q_xx_(3, 5);
    V_xx(4, 0) = Q_xu_(4, 0)*k_ux(0, 0) + Q_xu_(4, 1)*k_ux(1, 0) + Q_xx_(0, 4);
    V_xx(4, 1) = Q_xu_(4, 0)*k_ux(0, 1) + Q_xu_(4, 1)*k_ux(1, 1) + Q_xx_(1, 4);
    V_xx(4, 2) = Q_xu_(4, 0)*k_ux(0, 2) + Q_xu_(4, 1)*k_ux(1, 2) + Q_xx_(2, 4);
    V_xx(4, 3) = Q_xu_(4, 0)*k_ux(0, 3) + Q_xu_(4, 1)*k_ux(1, 3) + Q_xx_(3, 4);
    V_xx(4, 4) = Q_xu_(4, 0)*k_ux(0, 4) + Q_xu_(4, 1)*k_ux(1, 4) + Q_xx_(4, 4);
    V_xx(4, 5) = Q_xu_(4, 0)*k_ux(0, 5) + Q_xu_(4, 1)*k_ux(1, 5) + Q_xx_(4, 5);
    V_xx(5, 0) = Q_xu_(5, 0)*k_ux(0, 0) + Q_xu_(5, 1)*k_ux(1, 0) + Q_xx_(0, 5);
    V_xx(5, 1) = Q_xu_(5, 0)*k_ux(0, 1) + Q_xu_(5, 1)*k_ux(1, 1) + Q_xx_(1, 5);
    V_xx(5, 2) = Q_xu_(5, 0)*k_ux(0, 2) + Q_xu_(5, 1)*k_ux(1, 2) + Q_xx_(2, 5);
    V_xx(5, 3) = Q_xu_(5, 0)*k_ux(0, 3) + Q_xu_(5, 1)*k_ux(1, 3) + Q_xx_(3, 5);
    V_xx(5, 4) = Q_xu_(5, 0)*k_ux(0, 4) + Q_xu_(5, 1)*k_ux(1, 4) + Q_xx_(4, 5);
    V_xx(5, 5) = Q_xu_(5, 0)*k_ux(0, 5) + Q_xu_(5, 1)*k_ux(1, 5) + Q_xx_(5, 5);

    // Evaluation of Vector V_x
    V_x(0) = Q_x_(0) + Q_xu_(0, 0)*v_u(0) + Q_xu_(0, 1)*v_u(1);
    V_x(1) = Q_x_(1) + Q_xu_(1, 0)*v_u(0) + Q_xu_(1, 1)*v_u(1);
    V_x(2) = Q_x_(2) + Q_xu_(2, 0)*v_u(0) + Q_xu_(2, 1)*v_u(1);
    V_x(3) = Q_x_(3) + Q_xu_(3, 0)*v_u(0) + Q_xu_(3, 1)*v_u(1);
    V_x(4) = Q_x_(4) + Q_xu_(4, 0)*v_u(0) + Q_xu_(4, 1)*v_u(1);
    V_x(5) = Q_x_(5) + Q_xu_(5, 0)*v_u(0) + Q_xu_(5, 1)*v_u(1);

  }

  virtual void updateQnx(const Eigen::VectorXd& r_f, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& h_u, const Eigen::MatrixXd& k_ux, const Eigen::MatrixXd& k_ex, const Eigen::MatrixXd& k_ue, const Eigen::VectorXd& v_u, const Eigen::VectorXd& v_e) override {
    // Determine internal variables
    const double internal_0 = -Q_uu_(0, 1);

    // Evaluation of Matrix Q_un_
    Q_un_(0, 4) = -Fu(4, 0)*Q_uu_inv_(0, 0);
    Q_un_(0, 5) = -Fu(5, 1)*Q_uu_inv_(0, 1);
    Q_un_(1, 4) = -Fu(4, 0)*Q_uu_inv_(0, 1);
    Q_un_(1, 5) = -Fu(5, 1)*Q_uu_inv_(1, 1);

    // Evaluation of Matrix Q_nuu_
    Q_nuu_(0, 0) = -Q_uu_(0, 0);
    Q_nuu_(0, 1) = internal_0;
    Q_nuu_(1, 0) = internal_0;
    Q_nuu_(1, 1) = -Q_uu_(1, 1);
    Q_nuu_(2, 2) = -Q_uu_(2, 2);

    // Evaluation of Matrix Q_vnx_
    Q_vnx_(0, 0) = Fx(0, 0);
    Q_vnx_(0, 1) = Fx(0, 1);
    Q_vnx_(0, 2) = Fx(0, 2);
    Q_vnx_(0, 3) = Fx(0, 3);
    Q_vnx_(0, 5) = Fx(0, 5);
    Q_vnx_(1, 1) = Fx(1, 1);
    Q_vnx_(1, 3) = Fx(1, 3);
    Q_vnx_(1, 5) = Fx(1, 5);
    Q_vnx_(2, 2) = Fx(2, 2);
    Q_vnx_(2, 3) = Fx(2, 3);
    Q_vnx_(2, 5) = Fx(2, 5);
    Q_vnx_(3, 3) = Fx(3, 3);
    Q_vnx_(3, 4) = Fx(3, 4);
    Q_vnx_(3, 5) = Fx(3, 5);
    Q_vnx_(4, 0) = Fu(4, 0)*k_ux(0, 0);
    Q_vnx_(4, 1) = Fu(4, 0)*k_ux(0, 1);
    Q_vnx_(4, 2) = Fu(4, 0)*k_ux(0, 2);
    Q_vnx_(4, 3) = Fu(4, 0)*k_ux(0, 3);
    Q_vnx_(4, 4) = Fu(4, 0)*k_ux(0, 4) + Fx(4, 4);
    Q_vnx_(4, 5) = Fu(4, 0)*k_ux(0, 5);
    Q_vnx_(5, 0) = Fu(5, 1)*k_ux(1, 0);
    Q_vnx_(5, 1) = Fu(5, 1)*k_ux(1, 1);
    Q_vnx_(5, 2) = Fu(5, 1)*k_ux(1, 2);
    Q_vnx_(5, 3) = Fu(5, 1)*k_ux(1, 3);
    Q_vnx_(5, 4) = Fu(5, 1)*k_ux(1, 4);
    Q_vnx_(5, 5) = Fu(5, 1)*k_ux(1, 5) + Fx(5, 5);

    // Evaluation of Vector Q_vn_
    Q_vn_(0) = r_f(0);
    Q_vn_(1) = r_f(1);
    Q_vn_(2) = r_f(2);
    Q_vn_(3) = r_f(3);
    Q_vn_(4) = Fu(4, 0)*v_u(0) + r_f(4);
    Q_vn_(5) = Fu(5, 1)*v_u(1) + r_f(5);

  }

};

template <>
class ParkingGeneralIpmEvaluator<StageType::NORMINAL, OcpConfig::ERK4> : public IpmEvaluator {
 public:
  ParkingGeneralIpmEvaluator() : IpmEvaluator(6, 3, 21, 0, 0) {}
  virtual ~ParkingGeneralIpmEvaluator() = default;

  virtual void updateLX(const Eigen::VectorXd& l_x, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& h_x, const Eigen::MatrixXd& m_x, const Eigen::VectorXd& p, const Eigen::VectorXd& lambda, const Eigen::VectorXd& eta, const Eigen::VectorXd& nu, Eigen::VectorXd& sl_x) const override {
    // Evaluation of Vector sl_x
    sl_x(0) = Fx(0, 0)*p(0);
    sl_x(1) = Fx(0, 1)*p(0) + Fx(1, 1)*p(1) + g_x(0, 1)*lambda(0) + g_x(1, 1)*lambda(1) + g_x(15, 1)*lambda(15) + g_x(16, 1)*lambda(16) + g_x(17, 1)*lambda(17) + g_x(18, 1)*lambda(18) + g_x(19, 1)*lambda(19) + g_x(2, 1)*lambda(2) + g_x(20, 1)*lambda(20) + g_x(3, 1)*lambda(3) + g_x(4, 1)*lambda(4) + g_x(5, 1)*lambda(5) + l_x(1);
    sl_x(2) = Fx(0, 2)*p(0) + Fx(2, 2)*p(2) + g_x(0, 2)*lambda(0) + g_x(1, 2)*lambda(1) + g_x(15, 2)*lambda(15) + g_x(16, 2)*lambda(16) + g_x(17, 2)*lambda(17) + g_x(18, 2)*lambda(18) + g_x(19, 2)*lambda(19) + g_x(2, 2)*lambda(2) + g_x(20, 2)*lambda(20) + g_x(3, 2)*lambda(3) + g_x(4, 2)*lambda(4) + g_x(5, 2)*lambda(5) + l_x(2);
    sl_x(3) = Fx(0, 3)*p(0) + Fx(1, 3)*p(1) + Fx(2, 3)*p(2) + Fx(3, 3)*p(3) + g_x(10, 3)*lambda(10) + g_x(17, 3)*lambda(17) + g_x(18, 3)*lambda(18) + g_x(19, 3)*lambda(19) + g_x(2, 3)*lambda(2) + g_x(20, 3)*lambda(20) + g_x(3, 3)*lambda(3) + g_x(4, 3)*lambda(4) + g_x(5, 3)*lambda(5);
    sl_x(4) = Fx(0, 4)*p(0) + Fx(1, 4)*p(1) + Fx(2, 4)*p(2) + Fx(3, 4)*p(3) + Fx(4, 4)*p(4) + g_x(6, 4)*lambda(6) + g_x(7, 4)*lambda(7) + l_x(4);
    sl_x(5) = Fx(0, 5)*p(0) + Fx(1, 5)*p(1) + Fx(2, 5)*p(2) + Fx(3, 5)*p(3) + Fx(5, 5)*p(5) + g_x(8, 5)*lambda(8) + g_x(9, 5)*lambda(9) + l_x(5);

  }

  virtual void updateLU(const Eigen::VectorXd& l_u, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& g_u, const Eigen::MatrixXd& h_u, const Eigen::VectorXd& p, const Eigen::VectorXd& lambda, const Eigen::VectorXd& eta, Eigen::VectorXd& sl_u) const override {
    // Evaluation of Vector sl_u
    sl_u(0) = Fu(0, 0)*p(0) + Fu(1, 0)*p(1) + Fu(2, 0)*p(2) + Fu(3, 0)*p(3) + Fu(4, 0)*p(4) + g_u(11, 0)*lambda(11) + g_u(12, 0)*lambda(12) + l_u(0);
    sl_u(1) = Fu(0, 1)*p(0) + Fu(1, 1)*p(1) + Fu(2, 1)*p(2) + Fu(3, 1)*p(3) + Fu(5, 1)*p(5) + g_u(13, 1)*lambda(13) + g_u(14, 1)*lambda(14) + l_u(1);
    sl_u(2) = g_u(15, 2)*lambda(15) + g_u(16, 2)*lambda(16) + g_u(17, 2)*lambda(17) + g_u(18, 2)*lambda(18) + g_u(19, 2)*lambda(19) + g_u(20, 2)*lambda(20) + l_u(2);

  }

  virtual void updateQH(const Eigen::MatrixXd& l_xx, const Eigen::MatrixXd& l_xu, const Eigen::MatrixXd& l_uu, const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& V_xx_prev) override {
    // Determine internal variables
    const double internal_0 = Fx(0, 1)*V_xx_prev(0, 0);
    const double internal_1 = Fx(0, 0)*V_xx_prev(0, 1);
    const double internal_13 = Sigma_(15)*g_x(15, 1);
    const double internal_14 = Sigma_(16)*g_x(16, 1);
    const double internal_15 = Sigma_(17)*g_x(17, 1);
    const double internal_16 = Sigma_(18)*g_x(18, 1);
    const double internal_17 = Sigma_(19)*g_x(19, 1);
    const double internal_18 = Sigma_(2)*g_x(2, 1);
    const double internal_19 = Sigma_(20)*g_x(20, 1);
    const double internal_2 = Fx(0, 2)*V_xx_prev(0, 0);
    const double internal_20 = Sigma_(3)*g_x(3, 1);
    const double internal_21 = Sigma_(4)*g_x(4, 1);
    const double internal_22 = Sigma_(5)*g_x(5, 1);
    const double internal_3 = Fx(0, 0)*V_xx_prev(0, 2);
    const double internal_32 = Sigma_(17)*g_x(17, 2);
    const double internal_33 = Sigma_(18)*g_x(18, 2);
    const double internal_34 = Sigma_(19)*g_x(19, 2);
    const double internal_35 = Sigma_(20)*g_x(20, 2);
    const double internal_4 = Fx(0, 3)*V_xx_prev(0, 0);
    const double internal_5 = Fx(0, 0)*V_xx_prev(0, 3);
    const double internal_57 = Fu(0, 0)*V_xx_prev(0, 0);
    const double internal_58 = Fu(0, 1)*V_xx_prev(0, 0);
    const double internal_6 = Fx(0, 4)*V_xx_prev(0, 0);
    const double internal_7 = Fx(0, 0)*V_xx_prev(0, 4);
    const double internal_8 = Fx(0, 5)*V_xx_prev(0, 0);
    const double internal_9 = Fx(0, 0)*V_xx_prev(0, 5);
    const double internal_11 = Fx(0, 1)*V_xx_prev(0, 1) + Fx(1, 1)*V_xx_prev(1, 1);
    const double internal_12 = Fx(0, 1)*V_xx_prev(0, 2) + Fx(1, 1)*V_xx_prev(1, 2);
    const double internal_24 = Fx(0, 1)*V_xx_prev(0, 3) + Fx(1, 1)*V_xx_prev(1, 3);
    const double internal_26 = Fx(0, 1)*V_xx_prev(0, 4) + Fx(1, 1)*V_xx_prev(1, 4);
    const double internal_27 = Fx(0, 1)*V_xx_prev(0, 5) + Fx(1, 1)*V_xx_prev(1, 5);
    const double internal_29 = Fx(0, 2)*V_xx_prev(0, 1) + Fx(2, 2)*V_xx_prev(2, 1);
    const double internal_30 = Fx(0, 2)*V_xx_prev(0, 2) + Fx(2, 2)*V_xx_prev(2, 2);
    const double internal_31 = Fx(0, 2)*V_xx_prev(0, 3) + Fx(2, 2)*V_xx_prev(2, 3);
    const double internal_37 = Fx(0, 2)*V_xx_prev(0, 4) + Fx(2, 2)*V_xx_prev(2, 4);
    const double internal_38 = Fx(0, 2)*V_xx_prev(0, 5) + Fx(2, 2)*V_xx_prev(2, 5);
    const double internal_40 = Fx(0, 3)*V_xx_prev(0, 1) + Fx(1, 3)*V_xx_prev(1, 1) + Fx(2, 3)*V_xx_prev(2, 1) + Fx(3, 3)*V_xx_prev(3, 1);
    const double internal_41 = Fx(0, 3)*V_xx_prev(0, 2) + Fx(1, 3)*V_xx_prev(1, 2) + Fx(2, 3)*V_xx_prev(2, 2) + Fx(3, 3)*V_xx_prev(3, 2);
    const double internal_42 = Fx(0, 3)*V_xx_prev(0, 3) + Fx(1, 3)*V_xx_prev(1, 3) + Fx(2, 3)*V_xx_prev(2, 3) + Fx(3, 3)*V_xx_prev(3, 3);
    const double internal_43 = Fx(0, 3)*V_xx_prev(0, 4) + Fx(1, 3)*V_xx_prev(1, 4) + Fx(2, 3)*V_xx_prev(2, 4) + Fx(3, 3)*V_xx_prev(3, 4);
    const double internal_44 = Fx(0, 3)*V_xx_prev(0, 5) + Fx(1, 3)*V_xx_prev(1, 5) + Fx(2, 3)*V_xx_prev(2, 5) + Fx(3, 3)*V_xx_prev(3, 5);
    const double internal_46 = Fx(0, 4)*V_xx_prev(0, 1) + Fx(1, 4)*V_xx_prev(1, 1) + Fx(2, 4)*V_xx_prev(2, 1) + Fx(3, 4)*V_xx_prev(3, 1) + Fx(4, 4)*V_xx_prev(4, 1);
    const double internal_47 = Fx(0, 4)*V_xx_prev(0, 2) + Fx(1, 4)*V_xx_prev(1, 2) + Fx(2, 4)*V_xx_prev(2, 2) + Fx(3, 4)*V_xx_prev(3, 2) + Fx(4, 4)*V_xx_prev(4, 2);
    const double internal_48 = Fx(0, 4)*V_xx_prev(0, 3) + Fx(1, 4)*V_xx_prev(1, 3) + Fx(2, 4)*V_xx_prev(2, 3) + Fx(3, 4)*V_xx_prev(3, 3) + Fx(4, 4)*V_xx_prev(4, 3);
    const double internal_49 = Fx(0, 4)*V_xx_prev(0, 4) + Fx(1, 4)*V_xx_prev(1, 4) + Fx(2, 4)*V_xx_prev(2, 4) + Fx(3, 4)*V_xx_prev(3, 4) + Fx(4, 4)*V_xx_prev(4, 4);
    const double internal_50 = Fx(0, 4)*V_xx_prev(0, 5) + Fx(1, 4)*V_xx_prev(1, 5) + Fx(2, 4)*V_xx_prev(2, 5) + Fx(3, 4)*V_xx_prev(3, 5) + Fx(4, 4)*V_xx_prev(4, 5);
    const double internal_52 = Fx(0, 5)*V_xx_prev(0, 1) + Fx(1, 5)*V_xx_prev(1, 1) + Fx(2, 5)*V_xx_prev(2, 1) + Fx(3, 5)*V_xx_prev(3, 1) + Fx(5, 5)*V_xx_prev(5, 1);
    const double internal_53 = Fx(0, 5)*V_xx_prev(0, 2) + Fx(1, 5)*V_xx_prev(1, 2) + Fx(2, 5)*V_xx_prev(2, 2) + Fx(3, 5)*V_xx_prev(3, 2) + Fx(5, 5)*V_xx_prev(5, 2);
    const double internal_54 = Fx(0, 5)*V_xx_prev(0, 3) + Fx(1, 5)*V_xx_prev(1, 3) + Fx(2, 5)*V_xx_prev(2, 3) + Fx(3, 5)*V_xx_prev(3, 3) + Fx(5, 5)*V_xx_prev(5, 3);
    const double internal_55 = Fx(0, 5)*V_xx_prev(0, 4) + Fx(1, 5)*V_xx_prev(1, 4) + Fx(2, 5)*V_xx_prev(2, 4) + Fx(3, 5)*V_xx_prev(3, 4) + Fx(5, 5)*V_xx_prev(5, 4);
    const double internal_56 = Fx(0, 5)*V_xx_prev(0, 5) + Fx(1, 5)*V_xx_prev(1, 5) + Fx(2, 5)*V_xx_prev(2, 5) + Fx(3, 5)*V_xx_prev(3, 5) + Fx(5, 5)*V_xx_prev(5, 5);
    const double internal_60 = Fu(0, 0)*V_xx_prev(0, 1) + Fu(1, 0)*V_xx_prev(1, 1) + Fu(2, 0)*V_xx_prev(2, 1) + Fu(3, 0)*V_xx_prev(3, 1) + Fu(4, 0)*V_xx_prev(4, 1);
    const double internal_61 = Fu(0, 0)*V_xx_prev(0, 2) + Fu(1, 0)*V_xx_prev(1, 2) + Fu(2, 0)*V_xx_prev(2, 2) + Fu(3, 0)*V_xx_prev(3, 2) + Fu(4, 0)*V_xx_prev(4, 2);
    const double internal_62 = Fu(0, 0)*V_xx_prev(0, 3) + Fu(1, 0)*V_xx_prev(1, 3) + Fu(2, 0)*V_xx_prev(2, 3) + Fu(3, 0)*V_xx_prev(3, 3) + Fu(4, 0)*V_xx_prev(4, 3);
    const double internal_64 = Fu(0, 1)*V_xx_prev(0, 1) + Fu(1, 1)*V_xx_prev(1, 1) + Fu(2, 1)*V_xx_prev(2, 1) + Fu(3, 1)*V_xx_prev(3, 1) + Fu(5, 1)*V_xx_prev(5, 1);
    const double internal_65 = Fu(0, 1)*V_xx_prev(0, 2) + Fu(1, 1)*V_xx_prev(1, 2) + Fu(2, 1)*V_xx_prev(2, 2) + Fu(3, 1)*V_xx_prev(3, 2) + Fu(5, 1)*V_xx_prev(5, 2);
    const double internal_66 = Fu(0, 1)*V_xx_prev(0, 3) + Fu(1, 1)*V_xx_prev(1, 3) + Fu(2, 1)*V_xx_prev(2, 3) + Fu(3, 1)*V_xx_prev(3, 3) + Fu(5, 1)*V_xx_prev(5, 3);
    const double internal_10 = Fx(1, 1)*V_xx_prev(1, 0) + internal_0;
    const double internal_28 = Fx(2, 2)*V_xx_prev(2, 0) + internal_2;
    const double internal_39 = Fx(1, 3)*V_xx_prev(1, 0) + Fx(2, 3)*V_xx_prev(2, 0) + Fx(3, 3)*V_xx_prev(3, 0) + internal_4;
    const double internal_45 = Fx(1, 4)*V_xx_prev(1, 0) + Fx(2, 4)*V_xx_prev(2, 0) + Fx(3, 4)*V_xx_prev(3, 0) + Fx(4, 4)*V_xx_prev(4, 0) + internal_6;
    const double internal_51 = Fx(1, 5)*V_xx_prev(1, 0) + Fx(2, 5)*V_xx_prev(2, 0) + Fx(3, 5)*V_xx_prev(3, 0) + Fx(5, 5)*V_xx_prev(5, 0) + internal_8;
    const double internal_59 = Fu(1, 0)*V_xx_prev(1, 0) + Fu(2, 0)*V_xx_prev(2, 0) + Fu(3, 0)*V_xx_prev(3, 0) + Fu(4, 0)*V_xx_prev(4, 0) + internal_57;
    const double internal_63 = Fu(1, 1)*V_xx_prev(1, 0) + Fu(2, 1)*V_xx_prev(2, 0) + Fu(3, 1)*V_xx_prev(3, 0) + Fu(5, 1)*V_xx_prev(5, 0) + internal_58;
    const double internal_23 = Sigma_(0)*g_x(0, 1)*g_x(0, 2) + Sigma_(1)*g_x(1, 1)*g_x(1, 2) + g_x(15, 2)*internal_13 + g_x(16, 2)*internal_14 + g_x(17, 2)*internal_15 + g_x(18, 2)*internal_16 + g_x(19, 2)*internal_17 + g_x(2, 2)*internal_18 + g_x(20, 2)*internal_19 + g_x(3, 2)*internal_20 + g_x(4, 2)*internal_21 + g_x(5, 2)*internal_22;
    const double internal_25 = g_x(17, 3)*internal_15 + g_x(18, 3)*internal_16 + g_x(19, 3)*internal_17 + g_x(2, 3)*internal_18 + g_x(20, 3)*internal_19 + g_x(3, 3)*internal_20 + g_x(4, 3)*internal_21 + g_x(5, 3)*internal_22;
    const double internal_36 = Sigma_(2)*g_x(2, 2)*g_x(2, 3) + Sigma_(3)*g_x(3, 2)*g_x(3, 3) + Sigma_(4)*g_x(4, 2)*g_x(4, 3) + Sigma_(5)*g_x(5, 2)*g_x(5, 3) + g_x(17, 3)*internal_32 + g_x(18, 3)*internal_33 + g_x(19, 3)*internal_34 + g_x(20, 3)*internal_35;

    // Evaluation of Matrix Q_xx_
    Q_xx_(0, 0) = pow(Fx(0, 0), 2)*V_xx_prev(0, 0);
    Q_xx_(0, 1) = Fx(0, 0)*internal_0 + Fx(1, 1)*internal_1;
    Q_xx_(0, 2) = Fx(0, 0)*internal_2 + Fx(2, 2)*internal_3;
    Q_xx_(0, 3) = Fx(0, 0)*internal_4 + Fx(1, 3)*internal_1 + Fx(2, 3)*internal_3 + Fx(3, 3)*internal_5;
    Q_xx_(0, 4) = Fx(0, 0)*internal_6 + Fx(1, 4)*internal_1 + Fx(2, 4)*internal_3 + Fx(3, 4)*internal_5 + Fx(4, 4)*internal_7;
    Q_xx_(0, 5) = Fx(0, 0)*internal_8 + Fx(1, 5)*internal_1 + Fx(2, 5)*internal_3 + Fx(3, 5)*internal_5 + Fx(5, 5)*internal_9;
    Q_xx_(1, 0) = Fx(0, 0)*internal_10;
    Q_xx_(1, 1) = Fx(0, 1)*internal_10 + Fx(1, 1)*internal_11 + Sigma_(0)*pow(g_x(0, 1), 2) + Sigma_(1)*pow(g_x(1, 1), 2) + Sigma_(15)*pow(g_x(15, 1), 2) + Sigma_(16)*pow(g_x(16, 1), 2) + Sigma_(17)*pow(g_x(17, 1), 2) + Sigma_(18)*pow(g_x(18, 1), 2) + Sigma_(19)*pow(g_x(19, 1), 2) + Sigma_(2)*pow(g_x(2, 1), 2) + Sigma_(20)*pow(g_x(20, 1), 2) + Sigma_(3)*pow(g_x(3, 1), 2) + Sigma_(4)*pow(g_x(4, 1), 2) + Sigma_(5)*pow(g_x(5, 1), 2) + l_xx(1, 1);
    Q_xx_(1, 2) = Fx(0, 2)*internal_10 + Fx(2, 2)*internal_12 + internal_23 + l_xx(1, 2);
    Q_xx_(1, 3) = Fx(0, 3)*internal_10 + Fx(1, 3)*internal_11 + Fx(2, 3)*internal_12 + Fx(3, 3)*internal_24 + internal_25;
    Q_xx_(1, 4) = Fx(0, 4)*internal_10 + Fx(1, 4)*internal_11 + Fx(2, 4)*internal_12 + Fx(3, 4)*internal_24 + Fx(4, 4)*internal_26;
    Q_xx_(1, 5) = Fx(0, 5)*internal_10 + Fx(1, 5)*internal_11 + Fx(2, 5)*internal_12 + Fx(3, 5)*internal_24 + Fx(5, 5)*internal_27;
    Q_xx_(2, 0) = Fx(0, 0)*internal_28;
    Q_xx_(2, 1) = Fx(0, 1)*internal_28 + Fx(1, 1)*internal_29 + internal_23 + l_xx(2, 1);
    Q_xx_(2, 2) = Fx(0, 2)*internal_28 + Fx(2, 2)*internal_30 + Sigma_(0)*pow(g_x(0, 2), 2) + Sigma_(1)*pow(g_x(1, 2), 2) + Sigma_(15)*pow(g_x(15, 2), 2) + Sigma_(16)*pow(g_x(16, 2), 2) + Sigma_(17)*pow(g_x(17, 2), 2) + Sigma_(18)*pow(g_x(18, 2), 2) + Sigma_(19)*pow(g_x(19, 2), 2) + Sigma_(2)*pow(g_x(2, 2), 2) + Sigma_(20)*pow(g_x(20, 2), 2) + Sigma_(3)*pow(g_x(3, 2), 2) + Sigma_(4)*pow(g_x(4, 2), 2) + Sigma_(5)*pow(g_x(5, 2), 2) + l_xx(2, 2);
    Q_xx_(2, 3) = Fx(0, 3)*internal_28 + Fx(1, 3)*internal_29 + Fx(2, 3)*internal_30 + Fx(3, 3)*internal_31 + internal_36;
    Q_xx_(2, 4) = Fx(0, 4)*internal_28 + Fx(1, 4)*internal_29 + Fx(2, 4)*internal_30 + Fx(3, 4)*internal_31 + Fx(4, 4)*internal_37;
    Q_xx_(2, 5) = Fx(0, 5)*internal_28 + Fx(1, 5)*internal_29 + Fx(2, 5)*internal_30 + Fx(3, 5)*internal_31 + Fx(5, 5)*internal_38;
    Q_xx_(3, 0) = Fx(0, 0)*internal_39;
    Q_xx_(3, 1) = Fx(0, 1)*internal_39 + Fx(1, 1)*internal_40 + internal_25;
    Q_xx_(3, 2) = Fx(0, 2)*internal_39 + Fx(2, 2)*internal_41 + internal_36;
    Q_xx_(3, 3) = Fx(0, 3)*internal_39 + Fx(1, 3)*internal_40 + Fx(2, 3)*internal_41 + Fx(3, 3)*internal_42 + Sigma_(10)*pow(g_x(10, 3), 2) + Sigma_(17)*pow(g_x(17, 3), 2) + Sigma_(18)*pow(g_x(18, 3), 2) + Sigma_(19)*pow(g_x(19, 3), 2) + Sigma_(2)*pow(g_x(2, 3), 2) + Sigma_(20)*pow(g_x(20, 3), 2) + Sigma_(3)*pow(g_x(3, 3), 2) + Sigma_(4)*pow(g_x(4, 3), 2) + Sigma_(5)*pow(g_x(5, 3), 2);
    Q_xx_(3, 4) = Fx(0, 4)*internal_39 + Fx(1, 4)*internal_40 + Fx(2, 4)*internal_41 + Fx(3, 4)*internal_42 + Fx(4, 4)*internal_43;
    Q_xx_(3, 5) = Fx(0, 5)*internal_39 + Fx(1, 5)*internal_40 + Fx(2, 5)*internal_41 + Fx(3, 5)*internal_42 + Fx(5, 5)*internal_44;
    Q_xx_(4, 0) = Fx(0, 0)*internal_45;
    Q_xx_(4, 1) = Fx(0, 1)*internal_45 + Fx(1, 1)*internal_46;
    Q_xx_(4, 2) = Fx(0, 2)*internal_45 + Fx(2, 2)*internal_47;
    Q_xx_(4, 3) = Fx(0, 3)*internal_45 + Fx(1, 3)*internal_46 + Fx(2, 3)*internal_47 + Fx(3, 3)*internal_48;
    Q_xx_(4, 4) = Fx(0, 4)*internal_45 + Fx(1, 4)*internal_46 + Fx(2, 4)*internal_47 + Fx(3, 4)*internal_48 + Fx(4, 4)*internal_49 + Sigma_(6)*pow(g_x(6, 4), 2) + Sigma_(7)*pow(g_x(7, 4), 2) + l_xx(4, 4);
    Q_xx_(4, 5) = Fx(0, 5)*internal_45 + Fx(1, 5)*internal_46 + Fx(2, 5)*internal_47 + Fx(3, 5)*internal_48 + Fx(5, 5)*internal_50;
    Q_xx_(5, 0) = Fx(0, 0)*internal_51;
    Q_xx_(5, 1) = Fx(0, 1)*internal_51 + Fx(1, 1)*internal_52;
    Q_xx_(5, 2) = Fx(0, 2)*internal_51 + Fx(2, 2)*internal_53;
    Q_xx_(5, 3) = Fx(0, 3)*internal_51 + Fx(1, 3)*internal_52 + Fx(2, 3)*internal_53 + Fx(3, 3)*internal_54;
    Q_xx_(5, 4) = Fx(0, 4)*internal_51 + Fx(1, 4)*internal_52 + Fx(2, 4)*internal_53 + Fx(3, 4)*internal_54 + Fx(4, 4)*internal_55;
    Q_xx_(5, 5) = Fx(0, 5)*internal_51 + Fx(1, 5)*internal_52 + Fx(2, 5)*internal_53 + Fx(3, 5)*internal_54 + Fx(5, 5)*internal_56 + Sigma_(8)*pow(g_x(8, 5), 2) + Sigma_(9)*pow(g_x(9, 5), 2) + l_xx(5, 5);

    // Evaluation of Matrix Q_xu_
    Q_xu_(0, 0) = Fu(1, 0)*internal_1 + Fu(2, 0)*internal_3 + Fu(3, 0)*internal_5 + Fu(4, 0)*internal_7 + Fx(0, 0)*internal_57;
    Q_xu_(0, 1) = Fu(1, 1)*internal_1 + Fu(2, 1)*internal_3 + Fu(3, 1)*internal_5 + Fu(5, 1)*internal_9 + Fx(0, 0)*internal_58;
    Q_xu_(1, 0) = Fu(0, 0)*internal_10 + Fu(1, 0)*internal_11 + Fu(2, 0)*internal_12 + Fu(3, 0)*internal_24 + Fu(4, 0)*internal_26;
    Q_xu_(1, 1) = Fu(0, 1)*internal_10 + Fu(1, 1)*internal_11 + Fu(2, 1)*internal_12 + Fu(3, 1)*internal_24 + Fu(5, 1)*internal_27;
    Q_xu_(1, 2) = g_u(15, 2)*internal_13 + g_u(16, 2)*internal_14 + g_u(17, 2)*internal_15 + g_u(18, 2)*internal_16 + g_u(19, 2)*internal_17 + g_u(20, 2)*internal_19;
    Q_xu_(2, 0) = Fu(0, 0)*internal_28 + Fu(1, 0)*internal_29 + Fu(2, 0)*internal_30 + Fu(3, 0)*internal_31 + Fu(4, 0)*internal_37;
    Q_xu_(2, 1) = Fu(0, 1)*internal_28 + Fu(1, 1)*internal_29 + Fu(2, 1)*internal_30 + Fu(3, 1)*internal_31 + Fu(5, 1)*internal_38;
    Q_xu_(2, 2) = Sigma_(15)*g_u(15, 2)*g_x(15, 2) + Sigma_(16)*g_u(16, 2)*g_x(16, 2) + g_u(17, 2)*internal_32 + g_u(18, 2)*internal_33 + g_u(19, 2)*internal_34 + g_u(20, 2)*internal_35;
    Q_xu_(3, 0) = Fu(0, 0)*internal_39 + Fu(1, 0)*internal_40 + Fu(2, 0)*internal_41 + Fu(3, 0)*internal_42 + Fu(4, 0)*internal_43;
    Q_xu_(3, 1) = Fu(0, 1)*internal_39 + Fu(1, 1)*internal_40 + Fu(2, 1)*internal_41 + Fu(3, 1)*internal_42 + Fu(5, 1)*internal_44;
    Q_xu_(3, 2) = Sigma_(17)*g_u(17, 2)*g_x(17, 3) + Sigma_(18)*g_u(18, 2)*g_x(18, 3) + Sigma_(19)*g_u(19, 2)*g_x(19, 3) + Sigma_(20)*g_u(20, 2)*g_x(20, 3);
    Q_xu_(4, 0) = Fu(0, 0)*internal_45 + Fu(1, 0)*internal_46 + Fu(2, 0)*internal_47 + Fu(3, 0)*internal_48 + Fu(4, 0)*internal_49;
    Q_xu_(4, 1) = Fu(0, 1)*internal_45 + Fu(1, 1)*internal_46 + Fu(2, 1)*internal_47 + Fu(3, 1)*internal_48 + Fu(5, 1)*internal_50;
    Q_xu_(5, 0) = Fu(0, 0)*internal_51 + Fu(1, 0)*internal_52 + Fu(2, 0)*internal_53 + Fu(3, 0)*internal_54 + Fu(4, 0)*internal_55;
    Q_xu_(5, 1) = Fu(0, 1)*internal_51 + Fu(1, 1)*internal_52 + Fu(2, 1)*internal_53 + Fu(3, 1)*internal_54 + Fu(5, 1)*internal_56;

    // Evaluation of Matrix Q_uu_
    Q_uu_(0, 0) = Fu(0, 0)*internal_59 + Fu(1, 0)*internal_60 + Fu(2, 0)*internal_61 + Fu(3, 0)*internal_62 + Fu(4, 0)*(Fu(0, 0)*V_xx_prev(0, 4) + Fu(1, 0)*V_xx_prev(1, 4) + Fu(2, 0)*V_xx_prev(2, 4) + Fu(3, 0)*V_xx_prev(3, 4) + Fu(4, 0)*V_xx_prev(4, 4)) + Sigma_(11)*pow(g_u(11, 0), 2) + Sigma_(12)*pow(g_u(12, 0), 2) + l_uu(0, 0);
    Q_uu_(0, 1) = Fu(0, 1)*internal_59 + Fu(1, 1)*internal_60 + Fu(2, 1)*internal_61 + Fu(3, 1)*internal_62 + Fu(5, 1)*(Fu(0, 0)*V_xx_prev(0, 5) + Fu(1, 0)*V_xx_prev(1, 5) + Fu(2, 0)*V_xx_prev(2, 5) + Fu(3, 0)*V_xx_prev(3, 5) + Fu(4, 0)*V_xx_prev(4, 5));
    Q_uu_(1, 0) = Fu(0, 0)*internal_63 + Fu(1, 0)*internal_64 + Fu(2, 0)*internal_65 + Fu(3, 0)*internal_66 + Fu(4, 0)*(Fu(0, 1)*V_xx_prev(0, 4) + Fu(1, 1)*V_xx_prev(1, 4) + Fu(2, 1)*V_xx_prev(2, 4) + Fu(3, 1)*V_xx_prev(3, 4) + Fu(5, 1)*V_xx_prev(5, 4));
    Q_uu_(1, 1) = Fu(0, 1)*internal_63 + Fu(1, 1)*internal_64 + Fu(2, 1)*internal_65 + Fu(3, 1)*internal_66 + Fu(5, 1)*(Fu(0, 1)*V_xx_prev(0, 5) + Fu(1, 1)*V_xx_prev(1, 5) + Fu(2, 1)*V_xx_prev(2, 5) + Fu(3, 1)*V_xx_prev(3, 5) + Fu(5, 1)*V_xx_prev(5, 5)) + Sigma_(13)*pow(g_u(13, 1), 2) + Sigma_(14)*pow(g_u(14, 1), 2) + l_uu(1, 1);
    Q_uu_(2, 2) = Sigma_(15)*pow(g_u(15, 2), 2) + Sigma_(16)*pow(g_u(16, 2), 2) + Sigma_(17)*pow(g_u(17, 2), 2) + Sigma_(18)*pow(g_u(18, 2), 2) + Sigma_(19)*pow(g_u(19, 2), 2) + Sigma_(20)*pow(g_u(20, 2), 2) + l_uu(2, 2);

  }

  virtual void updateQG(const Eigen::VectorXd& sl_x, const Eigen::VectorXd& sl_u, const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::VectorXd& r_f, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& V_xx_prev, const Eigen::VectorXd& V_x_prev) override {
    // Determine internal variables
    const double internal_0 = V_x_prev(0) + V_xx_prev(0, 0)*r_f(0) + V_xx_prev(0, 1)*r_f(1) + V_xx_prev(0, 2)*r_f(2) + V_xx_prev(0, 3)*r_f(3) + V_xx_prev(0, 4)*r_f(4) + V_xx_prev(0, 5)*r_f(5);
    const double internal_1 = V_x_prev(1) + V_xx_prev(1, 0)*r_f(0) + V_xx_prev(1, 1)*r_f(1) + V_xx_prev(1, 2)*r_f(2) + V_xx_prev(1, 3)*r_f(3) + V_xx_prev(1, 4)*r_f(4) + V_xx_prev(1, 5)*r_f(5);
    const double internal_2 = V_x_prev(2) + V_xx_prev(2, 0)*r_f(0) + V_xx_prev(2, 1)*r_f(1) + V_xx_prev(2, 2)*r_f(2) + V_xx_prev(2, 3)*r_f(3) + V_xx_prev(2, 4)*r_f(4) + V_xx_prev(2, 5)*r_f(5);
    const double internal_3 = V_x_prev(3) + V_xx_prev(3, 0)*r_f(0) + V_xx_prev(3, 1)*r_f(1) + V_xx_prev(3, 2)*r_f(2) + V_xx_prev(3, 3)*r_f(3) + V_xx_prev(3, 4)*r_f(4) + V_xx_prev(3, 5)*r_f(5);
    const double internal_4 = V_x_prev(4) + V_xx_prev(4, 0)*r_f(0) + V_xx_prev(4, 1)*r_f(1) + V_xx_prev(4, 2)*r_f(2) + V_xx_prev(4, 3)*r_f(3) + V_xx_prev(4, 4)*r_f(4) + V_xx_prev(4, 5)*r_f(5);
    const double internal_5 = V_x_prev(5) + V_xx_prev(5, 0)*r_f(0) + V_xx_prev(5, 1)*r_f(1) + V_xx_prev(5, 2)*r_f(2) + V_xx_prev(5, 3)*r_f(3) + V_xx_prev(5, 4)*r_f(4) + V_xx_prev(5, 5)*r_f(5);

    // Evaluation of Vector Q_x_
    Q_x_(0) = Fx(0, 0)*internal_0 + sl_x(0);
    Q_x_(1) = Fx(0, 1)*internal_0 + Fx(1, 1)*internal_1 + V_(0)*g_x(0, 1) + V_(1)*g_x(1, 1) + V_(15)*g_x(15, 1) + V_(16)*g_x(16, 1) + V_(17)*g_x(17, 1) + V_(18)*g_x(18, 1) + V_(19)*g_x(19, 1) + V_(2)*g_x(2, 1) + V_(20)*g_x(20, 1) + V_(3)*g_x(3, 1) + V_(4)*g_x(4, 1) + V_(5)*g_x(5, 1) + sl_x(1);
    Q_x_(2) = Fx(0, 2)*internal_0 + Fx(2, 2)*internal_2 + V_(0)*g_x(0, 2) + V_(1)*g_x(1, 2) + V_(15)*g_x(15, 2) + V_(16)*g_x(16, 2) + V_(17)*g_x(17, 2) + V_(18)*g_x(18, 2) + V_(19)*g_x(19, 2) + V_(2)*g_x(2, 2) + V_(20)*g_x(20, 2) + V_(3)*g_x(3, 2) + V_(4)*g_x(4, 2) + V_(5)*g_x(5, 2) + sl_x(2);
    Q_x_(3) = Fx(0, 3)*internal_0 + Fx(1, 3)*internal_1 + Fx(2, 3)*internal_2 + Fx(3, 3)*internal_3 + V_(10)*g_x(10, 3) + V_(17)*g_x(17, 3) + V_(18)*g_x(18, 3) + V_(19)*g_x(19, 3) + V_(2)*g_x(2, 3) + V_(20)*g_x(20, 3) + V_(3)*g_x(3, 3) + V_(4)*g_x(4, 3) + V_(5)*g_x(5, 3) + sl_x(3);
    Q_x_(4) = Fx(0, 4)*internal_0 + Fx(1, 4)*internal_1 + Fx(2, 4)*internal_2 + Fx(3, 4)*internal_3 + Fx(4, 4)*internal_4 + V_(6)*g_x(6, 4) + V_(7)*g_x(7, 4) + sl_x(4);
    Q_x_(5) = Fx(0, 5)*internal_0 + Fx(1, 5)*internal_1 + Fx(2, 5)*internal_2 + Fx(3, 5)*internal_3 + Fx(5, 5)*internal_5 + V_(8)*g_x(8, 5) + V_(9)*g_x(9, 5) + sl_x(5);

    // Evaluation of Vector Q_u_
    Q_u_(0) = Fu(0, 0)*internal_0 + Fu(1, 0)*internal_1 + Fu(2, 0)*internal_2 + Fu(3, 0)*internal_3 + Fu(4, 0)*internal_4 + V_(11)*g_u(11, 0) + V_(12)*g_u(12, 0) + sl_u(0);
    Q_u_(1) = Fu(0, 1)*internal_0 + Fu(1, 1)*internal_1 + Fu(2, 1)*internal_2 + Fu(3, 1)*internal_3 + Fu(5, 1)*internal_5 + V_(13)*g_u(13, 1) + V_(14)*g_u(14, 1) + sl_u(1);
    Q_u_(2) = V_(15)*g_u(15, 2) + V_(16)*g_u(16, 2) + V_(17)*g_u(17, 2) + V_(18)*g_u(18, 2) + V_(19)*g_u(19, 2) + V_(20)*g_u(20, 2) + sl_u(2);

  }

  virtual void updateQuuInv() override {
    // Determine internal variables
    const double internal_0 = 1.0/Q_uu_(0, 0);
    const double internal_1 = pow(Q_uu_(0, 1), 2);
    const double internal_2 = 1.0/(Q_uu_(1, 1) - internal_0*internal_1);
    const double internal_3 = -Q_uu_(0, 1)*internal_0*internal_2;

    // Evaluation of Matrix Q_uu_inv_
    Q_uu_inv_(0, 0) = internal_0 + internal_1*internal_2/pow(Q_uu_(0, 0), 2);
    Q_uu_inv_(0, 1) = internal_3;
    Q_uu_inv_(1, 0) = internal_3;
    Q_uu_inv_(1, 1) = internal_2;
    Q_uu_inv_(2, 2) = 1.0/Q_uu_(2, 2);

  }

  virtual void updateKVU(Eigen::MatrixXd& k_ux, Eigen::VectorXd& v_u) override {
    // Evaluation of Matrix k_ux
    k_ux(0, 0) = -Q_uu_inv_(0, 0)*Q_xu_(0, 0) - Q_uu_inv_(0, 1)*Q_xu_(0, 1);
    k_ux(0, 1) = -Q_uu_inv_(0, 0)*Q_xu_(1, 0) - Q_uu_inv_(0, 1)*Q_xu_(1, 1);
    k_ux(0, 2) = -Q_uu_inv_(0, 0)*Q_xu_(2, 0) - Q_uu_inv_(0, 1)*Q_xu_(2, 1);
    k_ux(0, 3) = -Q_uu_inv_(0, 0)*Q_xu_(3, 0) - Q_uu_inv_(0, 1)*Q_xu_(3, 1);
    k_ux(0, 4) = -Q_uu_inv_(0, 0)*Q_xu_(4, 0) - Q_uu_inv_(0, 1)*Q_xu_(4, 1);
    k_ux(0, 5) = -Q_uu_inv_(0, 0)*Q_xu_(5, 0) - Q_uu_inv_(0, 1)*Q_xu_(5, 1);
    k_ux(1, 0) = -Q_uu_inv_(0, 1)*Q_xu_(0, 0) - Q_uu_inv_(1, 1)*Q_xu_(0, 1);
    k_ux(1, 1) = -Q_uu_inv_(0, 1)*Q_xu_(1, 0) - Q_uu_inv_(1, 1)*Q_xu_(1, 1);
    k_ux(1, 2) = -Q_uu_inv_(0, 1)*Q_xu_(2, 0) - Q_uu_inv_(1, 1)*Q_xu_(2, 1);
    k_ux(1, 3) = -Q_uu_inv_(0, 1)*Q_xu_(3, 0) - Q_uu_inv_(1, 1)*Q_xu_(3, 1);
    k_ux(1, 4) = -Q_uu_inv_(0, 1)*Q_xu_(4, 0) - Q_uu_inv_(1, 1)*Q_xu_(4, 1);
    k_ux(1, 5) = -Q_uu_inv_(0, 1)*Q_xu_(5, 0) - Q_uu_inv_(1, 1)*Q_xu_(5, 1);
    k_ux(2, 1) = -Q_uu_inv_(2, 2)*Q_xu_(1, 2);
    k_ux(2, 2) = -Q_uu_inv_(2, 2)*Q_xu_(2, 2);
    k_ux(2, 3) = -Q_uu_inv_(2, 2)*Q_xu_(3, 2);

    // Evaluation of Vector v_u
    v_u(0) = -Q_u_(0)*Q_uu_inv_(0, 0) - Q_u_(1)*Q_uu_inv_(0, 1);
    v_u(1) = -Q_u_(0)*Q_uu_inv_(0, 1) - Q_u_(1)*Q_uu_inv_(1, 1);
    v_u(2) = -Q_u_(2)*Q_uu_inv_(2, 2);

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
    V_xx(0, 0) = Q_xu_(0, 0)*k_ux(0, 0) + Q_xu_(0, 1)*k_ux(1, 0) + Q_xx_(0, 0);
    V_xx(0, 1) = Q_xu_(0, 0)*k_ux(0, 1) + Q_xu_(0, 1)*k_ux(1, 1) + Q_xx_(0, 1);
    V_xx(0, 2) = Q_xu_(0, 0)*k_ux(0, 2) + Q_xu_(0, 1)*k_ux(1, 2) + Q_xx_(0, 2);
    V_xx(0, 3) = Q_xu_(0, 0)*k_ux(0, 3) + Q_xu_(0, 1)*k_ux(1, 3) + Q_xx_(0, 3);
    V_xx(0, 4) = Q_xu_(0, 0)*k_ux(0, 4) + Q_xu_(0, 1)*k_ux(1, 4) + Q_xx_(0, 4);
    V_xx(0, 5) = Q_xu_(0, 0)*k_ux(0, 5) + Q_xu_(0, 1)*k_ux(1, 5) + Q_xx_(0, 5);
    V_xx(1, 0) = Q_xu_(1, 0)*k_ux(0, 0) + Q_xu_(1, 1)*k_ux(1, 0) + Q_xx_(0, 1);
    V_xx(1, 1) = Q_xu_(1, 0)*k_ux(0, 1) + Q_xu_(1, 1)*k_ux(1, 1) + Q_xu_(1, 2)*k_ux(2, 1) + Q_xx_(1, 1);
    V_xx(1, 2) = Q_xu_(1, 0)*k_ux(0, 2) + Q_xu_(1, 1)*k_ux(1, 2) + Q_xu_(1, 2)*k_ux(2, 2) + Q_xx_(1, 2);
    V_xx(1, 3) = Q_xu_(1, 0)*k_ux(0, 3) + Q_xu_(1, 1)*k_ux(1, 3) + Q_xu_(1, 2)*k_ux(2, 3) + Q_xx_(1, 3);
    V_xx(1, 4) = Q_xu_(1, 0)*k_ux(0, 4) + Q_xu_(1, 1)*k_ux(1, 4) + Q_xx_(1, 4);
    V_xx(1, 5) = Q_xu_(1, 0)*k_ux(0, 5) + Q_xu_(1, 1)*k_ux(1, 5) + Q_xx_(1, 5);
    V_xx(2, 0) = Q_xu_(2, 0)*k_ux(0, 0) + Q_xu_(2, 1)*k_ux(1, 0) + Q_xx_(0, 2);
    V_xx(2, 1) = Q_xu_(2, 0)*k_ux(0, 1) + Q_xu_(2, 1)*k_ux(1, 1) + Q_xu_(2, 2)*k_ux(2, 1) + Q_xx_(1, 2);
    V_xx(2, 2) = Q_xu_(2, 0)*k_ux(0, 2) + Q_xu_(2, 1)*k_ux(1, 2) + Q_xu_(2, 2)*k_ux(2, 2) + Q_xx_(2, 2);
    V_xx(2, 3) = Q_xu_(2, 0)*k_ux(0, 3) + Q_xu_(2, 1)*k_ux(1, 3) + Q_xu_(2, 2)*k_ux(2, 3) + Q_xx_(2, 3);
    V_xx(2, 4) = Q_xu_(2, 0)*k_ux(0, 4) + Q_xu_(2, 1)*k_ux(1, 4) + Q_xx_(2, 4);
    V_xx(2, 5) = Q_xu_(2, 0)*k_ux(0, 5) + Q_xu_(2, 1)*k_ux(1, 5) + Q_xx_(2, 5);
    V_xx(3, 0) = Q_xu_(3, 0)*k_ux(0, 0) + Q_xu_(3, 1)*k_ux(1, 0) + Q_xx_(0, 3);
    V_xx(3, 1) = Q_xu_(3, 0)*k_ux(0, 1) + Q_xu_(3, 1)*k_ux(1, 1) + Q_xu_(3, 2)*k_ux(2, 1) + Q_xx_(1, 3);
    V_xx(3, 2) = Q_xu_(3, 0)*k_ux(0, 2) + Q_xu_(3, 1)*k_ux(1, 2) + Q_xu_(3, 2)*k_ux(2, 2) + Q_xx_(2, 3);
    V_xx(3, 3) = Q_xu_(3, 0)*k_ux(0, 3) + Q_xu_(3, 1)*k_ux(1, 3) + Q_xu_(3, 2)*k_ux(2, 3) + Q_xx_(3, 3);
    V_xx(3, 4) = Q_xu_(3, 0)*k_ux(0, 4) + Q_xu_(3, 1)*k_ux(1, 4) + Q_xx_(3, 4);
    V_xx(3, 5) = Q_xu_(3, 0)*k_ux(0, 5) + Q_xu_(3, 1)*k_ux(1, 5) + Q_xx_(3, 5);
    V_xx(4, 0) = Q_xu_(4, 0)*k_ux(0, 0) + Q_xu_(4, 1)*k_ux(1, 0) + Q_xx_(0, 4);
    V_xx(4, 1) = Q_xu_(4, 0)*k_ux(0, 1) + Q_xu_(4, 1)*k_ux(1, 1) + Q_xx_(1, 4);
    V_xx(4, 2) = Q_xu_(4, 0)*k_ux(0, 2) + Q_xu_(4, 1)*k_ux(1, 2) + Q_xx_(2, 4);
    V_xx(4, 3) = Q_xu_(4, 0)*k_ux(0, 3) + Q_xu_(4, 1)*k_ux(1, 3) + Q_xx_(3, 4);
    V_xx(4, 4) = Q_xu_(4, 0)*k_ux(0, 4) + Q_xu_(4, 1)*k_ux(1, 4) + Q_xx_(4, 4);
    V_xx(4, 5) = Q_xu_(4, 0)*k_ux(0, 5) + Q_xu_(4, 1)*k_ux(1, 5) + Q_xx_(4, 5);
    V_xx(5, 0) = Q_xu_(5, 0)*k_ux(0, 0) + Q_xu_(5, 1)*k_ux(1, 0) + Q_xx_(0, 5);
    V_xx(5, 1) = Q_xu_(5, 0)*k_ux(0, 1) + Q_xu_(5, 1)*k_ux(1, 1) + Q_xx_(1, 5);
    V_xx(5, 2) = Q_xu_(5, 0)*k_ux(0, 2) + Q_xu_(5, 1)*k_ux(1, 2) + Q_xx_(2, 5);
    V_xx(5, 3) = Q_xu_(5, 0)*k_ux(0, 3) + Q_xu_(5, 1)*k_ux(1, 3) + Q_xx_(3, 5);
    V_xx(5, 4) = Q_xu_(5, 0)*k_ux(0, 4) + Q_xu_(5, 1)*k_ux(1, 4) + Q_xx_(4, 5);
    V_xx(5, 5) = Q_xu_(5, 0)*k_ux(0, 5) + Q_xu_(5, 1)*k_ux(1, 5) + Q_xx_(5, 5);

    // Evaluation of Vector V_x
    V_x(0) = Q_x_(0) + Q_xu_(0, 0)*v_u(0) + Q_xu_(0, 1)*v_u(1);
    V_x(1) = Q_x_(1) + Q_xu_(1, 0)*v_u(0) + Q_xu_(1, 1)*v_u(1) + Q_xu_(1, 2)*v_u(2);
    V_x(2) = Q_x_(2) + Q_xu_(2, 0)*v_u(0) + Q_xu_(2, 1)*v_u(1) + Q_xu_(2, 2)*v_u(2);
    V_x(3) = Q_x_(3) + Q_xu_(3, 0)*v_u(0) + Q_xu_(3, 1)*v_u(1) + Q_xu_(3, 2)*v_u(2);
    V_x(4) = Q_x_(4) + Q_xu_(4, 0)*v_u(0) + Q_xu_(4, 1)*v_u(1);
    V_x(5) = Q_x_(5) + Q_xu_(5, 0)*v_u(0) + Q_xu_(5, 1)*v_u(1);

  }

  virtual void updateQnx(const Eigen::VectorXd& r_f, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& h_u, const Eigen::MatrixXd& k_ux, const Eigen::MatrixXd& k_ex, const Eigen::MatrixXd& k_ue, const Eigen::VectorXd& v_u, const Eigen::VectorXd& v_e) override {
    // Determine internal variables
    const double internal_0 = -Q_uu_(0, 1);

    // Evaluation of Matrix Q_un_
    Q_un_(0, 0) = -Fu(0, 0)*Q_uu_inv_(0, 0) - Fu(0, 1)*Q_uu_inv_(0, 1);
    Q_un_(0, 1) = -Fu(1, 0)*Q_uu_inv_(0, 0) - Fu(1, 1)*Q_uu_inv_(0, 1);
    Q_un_(0, 2) = -Fu(2, 0)*Q_uu_inv_(0, 0) - Fu(2, 1)*Q_uu_inv_(0, 1);
    Q_un_(0, 3) = -Fu(3, 0)*Q_uu_inv_(0, 0) - Fu(3, 1)*Q_uu_inv_(0, 1);
    Q_un_(0, 4) = -Fu(4, 0)*Q_uu_inv_(0, 0);
    Q_un_(0, 5) = -Fu(5, 1)*Q_uu_inv_(0, 1);
    Q_un_(1, 0) = -Fu(0, 0)*Q_uu_inv_(0, 1) - Fu(0, 1)*Q_uu_inv_(1, 1);
    Q_un_(1, 1) = -Fu(1, 0)*Q_uu_inv_(0, 1) - Fu(1, 1)*Q_uu_inv_(1, 1);
    Q_un_(1, 2) = -Fu(2, 0)*Q_uu_inv_(0, 1) - Fu(2, 1)*Q_uu_inv_(1, 1);
    Q_un_(1, 3) = -Fu(3, 0)*Q_uu_inv_(0, 1) - Fu(3, 1)*Q_uu_inv_(1, 1);
    Q_un_(1, 4) = -Fu(4, 0)*Q_uu_inv_(0, 1);
    Q_un_(1, 5) = -Fu(5, 1)*Q_uu_inv_(1, 1);

    // Evaluation of Matrix Q_nuu_
    Q_nuu_(0, 0) = -Q_uu_(0, 0);
    Q_nuu_(0, 1) = internal_0;
    Q_nuu_(1, 0) = internal_0;
    Q_nuu_(1, 1) = -Q_uu_(1, 1);
    Q_nuu_(2, 2) = -Q_uu_(2, 2);

    // Evaluation of Matrix Q_vnx_
    Q_vnx_(0, 0) = Fu(0, 0)*k_ux(0, 0) + Fu(0, 1)*k_ux(1, 0) + Fx(0, 0);
    Q_vnx_(0, 1) = Fu(0, 0)*k_ux(0, 1) + Fu(0, 1)*k_ux(1, 1) + Fx(0, 1);
    Q_vnx_(0, 2) = Fu(0, 0)*k_ux(0, 2) + Fu(0, 1)*k_ux(1, 2) + Fx(0, 2);
    Q_vnx_(0, 3) = Fu(0, 0)*k_ux(0, 3) + Fu(0, 1)*k_ux(1, 3) + Fx(0, 3);
    Q_vnx_(0, 4) = Fu(0, 0)*k_ux(0, 4) + Fu(0, 1)*k_ux(1, 4) + Fx(0, 4);
    Q_vnx_(0, 5) = Fu(0, 0)*k_ux(0, 5) + Fu(0, 1)*k_ux(1, 5) + Fx(0, 5);
    Q_vnx_(1, 0) = Fu(1, 0)*k_ux(0, 0) + Fu(1, 1)*k_ux(1, 0);
    Q_vnx_(1, 1) = Fu(1, 0)*k_ux(0, 1) + Fu(1, 1)*k_ux(1, 1) + Fx(1, 1);
    Q_vnx_(1, 2) = Fu(1, 0)*k_ux(0, 2) + Fu(1, 1)*k_ux(1, 2);
    Q_vnx_(1, 3) = Fu(1, 0)*k_ux(0, 3) + Fu(1, 1)*k_ux(1, 3) + Fx(1, 3);
    Q_vnx_(1, 4) = Fu(1, 0)*k_ux(0, 4) + Fu(1, 1)*k_ux(1, 4) + Fx(1, 4);
    Q_vnx_(1, 5) = Fu(1, 0)*k_ux(0, 5) + Fu(1, 1)*k_ux(1, 5) + Fx(1, 5);
    Q_vnx_(2, 0) = Fu(2, 0)*k_ux(0, 0) + Fu(2, 1)*k_ux(1, 0);
    Q_vnx_(2, 1) = Fu(2, 0)*k_ux(0, 1) + Fu(2, 1)*k_ux(1, 1);
    Q_vnx_(2, 2) = Fu(2, 0)*k_ux(0, 2) + Fu(2, 1)*k_ux(1, 2) + Fx(2, 2);
    Q_vnx_(2, 3) = Fu(2, 0)*k_ux(0, 3) + Fu(2, 1)*k_ux(1, 3) + Fx(2, 3);
    Q_vnx_(2, 4) = Fu(2, 0)*k_ux(0, 4) + Fu(2, 1)*k_ux(1, 4) + Fx(2, 4);
    Q_vnx_(2, 5) = Fu(2, 0)*k_ux(0, 5) + Fu(2, 1)*k_ux(1, 5) + Fx(2, 5);
    Q_vnx_(3, 0) = Fu(3, 0)*k_ux(0, 0) + Fu(3, 1)*k_ux(1, 0);
    Q_vnx_(3, 1) = Fu(3, 0)*k_ux(0, 1) + Fu(3, 1)*k_ux(1, 1);
    Q_vnx_(3, 2) = Fu(3, 0)*k_ux(0, 2) + Fu(3, 1)*k_ux(1, 2);
    Q_vnx_(3, 3) = Fu(3, 0)*k_ux(0, 3) + Fu(3, 1)*k_ux(1, 3) + Fx(3, 3);
    Q_vnx_(3, 4) = Fu(3, 0)*k_ux(0, 4) + Fu(3, 1)*k_ux(1, 4) + Fx(3, 4);
    Q_vnx_(3, 5) = Fu(3, 0)*k_ux(0, 5) + Fu(3, 1)*k_ux(1, 5) + Fx(3, 5);
    Q_vnx_(4, 0) = Fu(4, 0)*k_ux(0, 0);
    Q_vnx_(4, 1) = Fu(4, 0)*k_ux(0, 1);
    Q_vnx_(4, 2) = Fu(4, 0)*k_ux(0, 2);
    Q_vnx_(4, 3) = Fu(4, 0)*k_ux(0, 3);
    Q_vnx_(4, 4) = Fu(4, 0)*k_ux(0, 4) + Fx(4, 4);
    Q_vnx_(4, 5) = Fu(4, 0)*k_ux(0, 5);
    Q_vnx_(5, 0) = Fu(5, 1)*k_ux(1, 0);
    Q_vnx_(5, 1) = Fu(5, 1)*k_ux(1, 1);
    Q_vnx_(5, 2) = Fu(5, 1)*k_ux(1, 2);
    Q_vnx_(5, 3) = Fu(5, 1)*k_ux(1, 3);
    Q_vnx_(5, 4) = Fu(5, 1)*k_ux(1, 4);
    Q_vnx_(5, 5) = Fu(5, 1)*k_ux(1, 5) + Fx(5, 5);

    // Evaluation of Vector Q_vn_
    Q_vn_(0) = Fu(0, 0)*v_u(0) + Fu(0, 1)*v_u(1) + r_f(0);
    Q_vn_(1) = Fu(1, 0)*v_u(0) + Fu(1, 1)*v_u(1) + r_f(1);
    Q_vn_(2) = Fu(2, 0)*v_u(0) + Fu(2, 1)*v_u(1) + r_f(2);
    Q_vn_(3) = Fu(3, 0)*v_u(0) + Fu(3, 1)*v_u(1) + r_f(3);
    Q_vn_(4) = Fu(4, 0)*v_u(0) + r_f(4);
    Q_vn_(5) = Fu(5, 1)*v_u(1) + r_f(5);

  }

};

template <>
class ParkingGeneralIpmEvaluator<StageType::INITIAL, OcpConfig::ERK4> : public IpmEvaluator {
 public:
  ParkingGeneralIpmEvaluator() : IpmEvaluator(6, 3, 4, 0, 6) {}
  virtual ~ParkingGeneralIpmEvaluator() = default;

  virtual void updateLX(const Eigen::VectorXd& l_x, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& h_x, const Eigen::MatrixXd& m_x, const Eigen::VectorXd& p, const Eigen::VectorXd& lambda, const Eigen::VectorXd& eta, const Eigen::VectorXd& nu, Eigen::VectorXd& sl_x) const override {
    // Evaluation of Vector sl_x
    sl_x(0) = Fx(0, 0)*p(0) + m_x(0, 0)*nu(0);
    sl_x(1) = Fx(0, 1)*p(0) + Fx(1, 1)*p(1) + l_x(1) + m_x(1, 1)*nu(1);
    sl_x(2) = Fx(0, 2)*p(0) + Fx(2, 2)*p(2) + l_x(2) + m_x(2, 2)*nu(2);
    sl_x(3) = Fx(0, 3)*p(0) + Fx(1, 3)*p(1) + Fx(2, 3)*p(2) + Fx(3, 3)*p(3) + m_x(3, 3)*nu(3);
    sl_x(4) = Fx(0, 4)*p(0) + Fx(1, 4)*p(1) + Fx(2, 4)*p(2) + Fx(3, 4)*p(3) + Fx(4, 4)*p(4) + l_x(4) + m_x(4, 4)*nu(4);
    sl_x(5) = Fx(0, 5)*p(0) + Fx(1, 5)*p(1) + Fx(2, 5)*p(2) + Fx(3, 5)*p(3) + Fx(5, 5)*p(5) + l_x(5) + m_x(5, 5)*nu(5);

  }

  virtual void updateLU(const Eigen::VectorXd& l_u, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& g_u, const Eigen::MatrixXd& h_u, const Eigen::VectorXd& p, const Eigen::VectorXd& lambda, const Eigen::VectorXd& eta, Eigen::VectorXd& sl_u) const override {
    // Evaluation of Vector sl_u
    sl_u(0) = Fu(0, 0)*p(0) + Fu(1, 0)*p(1) + Fu(2, 0)*p(2) + Fu(3, 0)*p(3) + Fu(4, 0)*p(4) + g_u(0, 0)*lambda(0) + g_u(1, 0)*lambda(1) + l_u(0);
    sl_u(1) = Fu(0, 1)*p(0) + Fu(1, 1)*p(1) + Fu(2, 1)*p(2) + Fu(3, 1)*p(3) + Fu(5, 1)*p(5) + g_u(2, 1)*lambda(2) + g_u(3, 1)*lambda(3) + l_u(1);
    sl_u(2) = l_u(2);

  }

  virtual void updateQH(const Eigen::MatrixXd& l_xx, const Eigen::MatrixXd& l_xu, const Eigen::MatrixXd& l_uu, const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& V_xx_prev) override {
    // Determine internal variables
    const double internal_0 = Fx(0, 1)*V_xx_prev(0, 0);
    const double internal_1 = Fx(0, 0)*V_xx_prev(0, 1);
    const double internal_2 = Fx(0, 2)*V_xx_prev(0, 0);
    const double internal_3 = Fx(0, 0)*V_xx_prev(0, 2);
    const double internal_4 = Fx(0, 3)*V_xx_prev(0, 0);
    const double internal_40 = Fu(0, 0)*V_xx_prev(0, 0);
    const double internal_41 = Fu(0, 1)*V_xx_prev(0, 0);
    const double internal_5 = Fx(0, 0)*V_xx_prev(0, 3);
    const double internal_6 = Fx(0, 4)*V_xx_prev(0, 0);
    const double internal_7 = Fx(0, 0)*V_xx_prev(0, 4);
    const double internal_8 = Fx(0, 5)*V_xx_prev(0, 0);
    const double internal_9 = Fx(0, 0)*V_xx_prev(0, 5);
    const double internal_11 = Fx(0, 1)*V_xx_prev(0, 1) + Fx(1, 1)*V_xx_prev(1, 1);
    const double internal_12 = Fx(0, 1)*V_xx_prev(0, 2) + Fx(1, 1)*V_xx_prev(1, 2);
    const double internal_13 = Fx(0, 1)*V_xx_prev(0, 3) + Fx(1, 1)*V_xx_prev(1, 3);
    const double internal_14 = Fx(0, 1)*V_xx_prev(0, 4) + Fx(1, 1)*V_xx_prev(1, 4);
    const double internal_15 = Fx(0, 1)*V_xx_prev(0, 5) + Fx(1, 1)*V_xx_prev(1, 5);
    const double internal_17 = Fx(0, 2)*V_xx_prev(0, 1) + Fx(2, 2)*V_xx_prev(2, 1);
    const double internal_18 = Fx(0, 2)*V_xx_prev(0, 2) + Fx(2, 2)*V_xx_prev(2, 2);
    const double internal_19 = Fx(0, 2)*V_xx_prev(0, 3) + Fx(2, 2)*V_xx_prev(2, 3);
    const double internal_20 = Fx(0, 2)*V_xx_prev(0, 4) + Fx(2, 2)*V_xx_prev(2, 4);
    const double internal_21 = Fx(0, 2)*V_xx_prev(0, 5) + Fx(2, 2)*V_xx_prev(2, 5);
    const double internal_23 = Fx(0, 3)*V_xx_prev(0, 1) + Fx(1, 3)*V_xx_prev(1, 1) + Fx(2, 3)*V_xx_prev(2, 1) + Fx(3, 3)*V_xx_prev(3, 1);
    const double internal_24 = Fx(0, 3)*V_xx_prev(0, 2) + Fx(1, 3)*V_xx_prev(1, 2) + Fx(2, 3)*V_xx_prev(2, 2) + Fx(3, 3)*V_xx_prev(3, 2);
    const double internal_25 = Fx(0, 3)*V_xx_prev(0, 3) + Fx(1, 3)*V_xx_prev(1, 3) + Fx(2, 3)*V_xx_prev(2, 3) + Fx(3, 3)*V_xx_prev(3, 3);
    const double internal_26 = Fx(0, 3)*V_xx_prev(0, 4) + Fx(1, 3)*V_xx_prev(1, 4) + Fx(2, 3)*V_xx_prev(2, 4) + Fx(3, 3)*V_xx_prev(3, 4);
    const double internal_27 = Fx(0, 3)*V_xx_prev(0, 5) + Fx(1, 3)*V_xx_prev(1, 5) + Fx(2, 3)*V_xx_prev(2, 5) + Fx(3, 3)*V_xx_prev(3, 5);
    const double internal_29 = Fx(0, 4)*V_xx_prev(0, 1) + Fx(1, 4)*V_xx_prev(1, 1) + Fx(2, 4)*V_xx_prev(2, 1) + Fx(3, 4)*V_xx_prev(3, 1) + Fx(4, 4)*V_xx_prev(4, 1);
    const double internal_30 = Fx(0, 4)*V_xx_prev(0, 2) + Fx(1, 4)*V_xx_prev(1, 2) + Fx(2, 4)*V_xx_prev(2, 2) + Fx(3, 4)*V_xx_prev(3, 2) + Fx(4, 4)*V_xx_prev(4, 2);
    const double internal_31 = Fx(0, 4)*V_xx_prev(0, 3) + Fx(1, 4)*V_xx_prev(1, 3) + Fx(2, 4)*V_xx_prev(2, 3) + Fx(3, 4)*V_xx_prev(3, 3) + Fx(4, 4)*V_xx_prev(4, 3);
    const double internal_32 = Fx(0, 4)*V_xx_prev(0, 4) + Fx(1, 4)*V_xx_prev(1, 4) + Fx(2, 4)*V_xx_prev(2, 4) + Fx(3, 4)*V_xx_prev(3, 4) + Fx(4, 4)*V_xx_prev(4, 4);
    const double internal_33 = Fx(0, 4)*V_xx_prev(0, 5) + Fx(1, 4)*V_xx_prev(1, 5) + Fx(2, 4)*V_xx_prev(2, 5) + Fx(3, 4)*V_xx_prev(3, 5) + Fx(4, 4)*V_xx_prev(4, 5);
    const double internal_35 = Fx(0, 5)*V_xx_prev(0, 1) + Fx(1, 5)*V_xx_prev(1, 1) + Fx(2, 5)*V_xx_prev(2, 1) + Fx(3, 5)*V_xx_prev(3, 1) + Fx(5, 5)*V_xx_prev(5, 1);
    const double internal_36 = Fx(0, 5)*V_xx_prev(0, 2) + Fx(1, 5)*V_xx_prev(1, 2) + Fx(2, 5)*V_xx_prev(2, 2) + Fx(3, 5)*V_xx_prev(3, 2) + Fx(5, 5)*V_xx_prev(5, 2);
    const double internal_37 = Fx(0, 5)*V_xx_prev(0, 3) + Fx(1, 5)*V_xx_prev(1, 3) + Fx(2, 5)*V_xx_prev(2, 3) + Fx(3, 5)*V_xx_prev(3, 3) + Fx(5, 5)*V_xx_prev(5, 3);
    const double internal_38 = Fx(0, 5)*V_xx_prev(0, 4) + Fx(1, 5)*V_xx_prev(1, 4) + Fx(2, 5)*V_xx_prev(2, 4) + Fx(3, 5)*V_xx_prev(3, 4) + Fx(5, 5)*V_xx_prev(5, 4);
    const double internal_39 = Fx(0, 5)*V_xx_prev(0, 5) + Fx(1, 5)*V_xx_prev(1, 5) + Fx(2, 5)*V_xx_prev(2, 5) + Fx(3, 5)*V_xx_prev(3, 5) + Fx(5, 5)*V_xx_prev(5, 5);
    const double internal_43 = Fu(0, 0)*V_xx_prev(0, 1) + Fu(1, 0)*V_xx_prev(1, 1) + Fu(2, 0)*V_xx_prev(2, 1) + Fu(3, 0)*V_xx_prev(3, 1) + Fu(4, 0)*V_xx_prev(4, 1);
    const double internal_44 = Fu(0, 0)*V_xx_prev(0, 2) + Fu(1, 0)*V_xx_prev(1, 2) + Fu(2, 0)*V_xx_prev(2, 2) + Fu(3, 0)*V_xx_prev(3, 2) + Fu(4, 0)*V_xx_prev(4, 2);
    const double internal_45 = Fu(0, 0)*V_xx_prev(0, 3) + Fu(1, 0)*V_xx_prev(1, 3) + Fu(2, 0)*V_xx_prev(2, 3) + Fu(3, 0)*V_xx_prev(3, 3) + Fu(4, 0)*V_xx_prev(4, 3);
    const double internal_47 = Fu(0, 1)*V_xx_prev(0, 1) + Fu(1, 1)*V_xx_prev(1, 1) + Fu(2, 1)*V_xx_prev(2, 1) + Fu(3, 1)*V_xx_prev(3, 1) + Fu(5, 1)*V_xx_prev(5, 1);
    const double internal_48 = Fu(0, 1)*V_xx_prev(0, 2) + Fu(1, 1)*V_xx_prev(1, 2) + Fu(2, 1)*V_xx_prev(2, 2) + Fu(3, 1)*V_xx_prev(3, 2) + Fu(5, 1)*V_xx_prev(5, 2);
    const double internal_49 = Fu(0, 1)*V_xx_prev(0, 3) + Fu(1, 1)*V_xx_prev(1, 3) + Fu(2, 1)*V_xx_prev(2, 3) + Fu(3, 1)*V_xx_prev(3, 3) + Fu(5, 1)*V_xx_prev(5, 3);
    const double internal_10 = Fx(1, 1)*V_xx_prev(1, 0) + internal_0;
    const double internal_16 = Fx(2, 2)*V_xx_prev(2, 0) + internal_2;
    const double internal_22 = Fx(1, 3)*V_xx_prev(1, 0) + Fx(2, 3)*V_xx_prev(2, 0) + Fx(3, 3)*V_xx_prev(3, 0) + internal_4;
    const double internal_28 = Fx(1, 4)*V_xx_prev(1, 0) + Fx(2, 4)*V_xx_prev(2, 0) + Fx(3, 4)*V_xx_prev(3, 0) + Fx(4, 4)*V_xx_prev(4, 0) + internal_6;
    const double internal_34 = Fx(1, 5)*V_xx_prev(1, 0) + Fx(2, 5)*V_xx_prev(2, 0) + Fx(3, 5)*V_xx_prev(3, 0) + Fx(5, 5)*V_xx_prev(5, 0) + internal_8;
    const double internal_42 = Fu(1, 0)*V_xx_prev(1, 0) + Fu(2, 0)*V_xx_prev(2, 0) + Fu(3, 0)*V_xx_prev(3, 0) + Fu(4, 0)*V_xx_prev(4, 0) + internal_40;
    const double internal_46 = Fu(1, 1)*V_xx_prev(1, 0) + Fu(2, 1)*V_xx_prev(2, 0) + Fu(3, 1)*V_xx_prev(3, 0) + Fu(5, 1)*V_xx_prev(5, 0) + internal_41;

    // Evaluation of Matrix Q_xx_
    Q_xx_(0, 0) = pow(Fx(0, 0), 2)*V_xx_prev(0, 0);
    Q_xx_(0, 1) = Fx(0, 0)*internal_0 + Fx(1, 1)*internal_1;
    Q_xx_(0, 2) = Fx(0, 0)*internal_2 + Fx(2, 2)*internal_3;
    Q_xx_(0, 3) = Fx(0, 0)*internal_4 + Fx(1, 3)*internal_1 + Fx(2, 3)*internal_3 + Fx(3, 3)*internal_5;
    Q_xx_(0, 4) = Fx(0, 0)*internal_6 + Fx(1, 4)*internal_1 + Fx(2, 4)*internal_3 + Fx(3, 4)*internal_5 + Fx(4, 4)*internal_7;
    Q_xx_(0, 5) = Fx(0, 0)*internal_8 + Fx(1, 5)*internal_1 + Fx(2, 5)*internal_3 + Fx(3, 5)*internal_5 + Fx(5, 5)*internal_9;
    Q_xx_(1, 0) = Fx(0, 0)*internal_10;
    Q_xx_(1, 1) = Fx(0, 1)*internal_10 + Fx(1, 1)*internal_11 + l_xx(1, 1);
    Q_xx_(1, 2) = Fx(0, 2)*internal_10 + Fx(2, 2)*internal_12 + l_xx(1, 2);
    Q_xx_(1, 3) = Fx(0, 3)*internal_10 + Fx(1, 3)*internal_11 + Fx(2, 3)*internal_12 + Fx(3, 3)*internal_13;
    Q_xx_(1, 4) = Fx(0, 4)*internal_10 + Fx(1, 4)*internal_11 + Fx(2, 4)*internal_12 + Fx(3, 4)*internal_13 + Fx(4, 4)*internal_14;
    Q_xx_(1, 5) = Fx(0, 5)*internal_10 + Fx(1, 5)*internal_11 + Fx(2, 5)*internal_12 + Fx(3, 5)*internal_13 + Fx(5, 5)*internal_15;
    Q_xx_(2, 0) = Fx(0, 0)*internal_16;
    Q_xx_(2, 1) = Fx(0, 1)*internal_16 + Fx(1, 1)*internal_17 + l_xx(2, 1);
    Q_xx_(2, 2) = Fx(0, 2)*internal_16 + Fx(2, 2)*internal_18 + l_xx(2, 2);
    Q_xx_(2, 3) = Fx(0, 3)*internal_16 + Fx(1, 3)*internal_17 + Fx(2, 3)*internal_18 + Fx(3, 3)*internal_19;
    Q_xx_(2, 4) = Fx(0, 4)*internal_16 + Fx(1, 4)*internal_17 + Fx(2, 4)*internal_18 + Fx(3, 4)*internal_19 + Fx(4, 4)*internal_20;
    Q_xx_(2, 5) = Fx(0, 5)*internal_16 + Fx(1, 5)*internal_17 + Fx(2, 5)*internal_18 + Fx(3, 5)*internal_19 + Fx(5, 5)*internal_21;
    Q_xx_(3, 0) = Fx(0, 0)*internal_22;
    Q_xx_(3, 1) = Fx(0, 1)*internal_22 + Fx(1, 1)*internal_23;
    Q_xx_(3, 2) = Fx(0, 2)*internal_22 + Fx(2, 2)*internal_24;
    Q_xx_(3, 3) = Fx(0, 3)*internal_22 + Fx(1, 3)*internal_23 + Fx(2, 3)*internal_24 + Fx(3, 3)*internal_25;
    Q_xx_(3, 4) = Fx(0, 4)*internal_22 + Fx(1, 4)*internal_23 + Fx(2, 4)*internal_24 + Fx(3, 4)*internal_25 + Fx(4, 4)*internal_26;
    Q_xx_(3, 5) = Fx(0, 5)*internal_22 + Fx(1, 5)*internal_23 + Fx(2, 5)*internal_24 + Fx(3, 5)*internal_25 + Fx(5, 5)*internal_27;
    Q_xx_(4, 0) = Fx(0, 0)*internal_28;
    Q_xx_(4, 1) = Fx(0, 1)*internal_28 + Fx(1, 1)*internal_29;
    Q_xx_(4, 2) = Fx(0, 2)*internal_28 + Fx(2, 2)*internal_30;
    Q_xx_(4, 3) = Fx(0, 3)*internal_28 + Fx(1, 3)*internal_29 + Fx(2, 3)*internal_30 + Fx(3, 3)*internal_31;
    Q_xx_(4, 4) = Fx(0, 4)*internal_28 + Fx(1, 4)*internal_29 + Fx(2, 4)*internal_30 + Fx(3, 4)*internal_31 + Fx(4, 4)*internal_32 + l_xx(4, 4);
    Q_xx_(4, 5) = Fx(0, 5)*internal_28 + Fx(1, 5)*internal_29 + Fx(2, 5)*internal_30 + Fx(3, 5)*internal_31 + Fx(5, 5)*internal_33;
    Q_xx_(5, 0) = Fx(0, 0)*internal_34;
    Q_xx_(5, 1) = Fx(0, 1)*internal_34 + Fx(1, 1)*internal_35;
    Q_xx_(5, 2) = Fx(0, 2)*internal_34 + Fx(2, 2)*internal_36;
    Q_xx_(5, 3) = Fx(0, 3)*internal_34 + Fx(1, 3)*internal_35 + Fx(2, 3)*internal_36 + Fx(3, 3)*internal_37;
    Q_xx_(5, 4) = Fx(0, 4)*internal_34 + Fx(1, 4)*internal_35 + Fx(2, 4)*internal_36 + Fx(3, 4)*internal_37 + Fx(4, 4)*internal_38;
    Q_xx_(5, 5) = Fx(0, 5)*internal_34 + Fx(1, 5)*internal_35 + Fx(2, 5)*internal_36 + Fx(3, 5)*internal_37 + Fx(5, 5)*internal_39 + l_xx(5, 5);

    // Evaluation of Matrix Q_xu_
    Q_xu_(0, 0) = Fu(1, 0)*internal_1 + Fu(2, 0)*internal_3 + Fu(3, 0)*internal_5 + Fu(4, 0)*internal_7 + Fx(0, 0)*internal_40;
    Q_xu_(0, 1) = Fu(1, 1)*internal_1 + Fu(2, 1)*internal_3 + Fu(3, 1)*internal_5 + Fu(5, 1)*internal_9 + Fx(0, 0)*internal_41;
    Q_xu_(1, 0) = Fu(0, 0)*internal_10 + Fu(1, 0)*internal_11 + Fu(2, 0)*internal_12 + Fu(3, 0)*internal_13 + Fu(4, 0)*internal_14;
    Q_xu_(1, 1) = Fu(0, 1)*internal_10 + Fu(1, 1)*internal_11 + Fu(2, 1)*internal_12 + Fu(3, 1)*internal_13 + Fu(5, 1)*internal_15;
    Q_xu_(2, 0) = Fu(0, 0)*internal_16 + Fu(1, 0)*internal_17 + Fu(2, 0)*internal_18 + Fu(3, 0)*internal_19 + Fu(4, 0)*internal_20;
    Q_xu_(2, 1) = Fu(0, 1)*internal_16 + Fu(1, 1)*internal_17 + Fu(2, 1)*internal_18 + Fu(3, 1)*internal_19 + Fu(5, 1)*internal_21;
    Q_xu_(3, 0) = Fu(0, 0)*internal_22 + Fu(1, 0)*internal_23 + Fu(2, 0)*internal_24 + Fu(3, 0)*internal_25 + Fu(4, 0)*internal_26;
    Q_xu_(3, 1) = Fu(0, 1)*internal_22 + Fu(1, 1)*internal_23 + Fu(2, 1)*internal_24 + Fu(3, 1)*internal_25 + Fu(5, 1)*internal_27;
    Q_xu_(4, 0) = Fu(0, 0)*internal_28 + Fu(1, 0)*internal_29 + Fu(2, 0)*internal_30 + Fu(3, 0)*internal_31 + Fu(4, 0)*internal_32;
    Q_xu_(4, 1) = Fu(0, 1)*internal_28 + Fu(1, 1)*internal_29 + Fu(2, 1)*internal_30 + Fu(3, 1)*internal_31 + Fu(5, 1)*internal_33;
    Q_xu_(5, 0) = Fu(0, 0)*internal_34 + Fu(1, 0)*internal_35 + Fu(2, 0)*internal_36 + Fu(3, 0)*internal_37 + Fu(4, 0)*internal_38;
    Q_xu_(5, 1) = Fu(0, 1)*internal_34 + Fu(1, 1)*internal_35 + Fu(2, 1)*internal_36 + Fu(3, 1)*internal_37 + Fu(5, 1)*internal_39;

    // Evaluation of Matrix Q_uu_
    Q_uu_(0, 0) = Fu(0, 0)*internal_42 + Fu(1, 0)*internal_43 + Fu(2, 0)*internal_44 + Fu(3, 0)*internal_45 + Fu(4, 0)*(Fu(0, 0)*V_xx_prev(0, 4) + Fu(1, 0)*V_xx_prev(1, 4) + Fu(2, 0)*V_xx_prev(2, 4) + Fu(3, 0)*V_xx_prev(3, 4) + Fu(4, 0)*V_xx_prev(4, 4)) + Sigma_(0)*pow(g_u(0, 0), 2) + Sigma_(1)*pow(g_u(1, 0), 2) + l_uu(0, 0);
    Q_uu_(0, 1) = Fu(0, 1)*internal_42 + Fu(1, 1)*internal_43 + Fu(2, 1)*internal_44 + Fu(3, 1)*internal_45 + Fu(5, 1)*(Fu(0, 0)*V_xx_prev(0, 5) + Fu(1, 0)*V_xx_prev(1, 5) + Fu(2, 0)*V_xx_prev(2, 5) + Fu(3, 0)*V_xx_prev(3, 5) + Fu(4, 0)*V_xx_prev(4, 5));
    Q_uu_(1, 0) = Fu(0, 0)*internal_46 + Fu(1, 0)*internal_47 + Fu(2, 0)*internal_48 + Fu(3, 0)*internal_49 + Fu(4, 0)*(Fu(0, 1)*V_xx_prev(0, 4) + Fu(1, 1)*V_xx_prev(1, 4) + Fu(2, 1)*V_xx_prev(2, 4) + Fu(3, 1)*V_xx_prev(3, 4) + Fu(5, 1)*V_xx_prev(5, 4));
    Q_uu_(1, 1) = Fu(0, 1)*internal_46 + Fu(1, 1)*internal_47 + Fu(2, 1)*internal_48 + Fu(3, 1)*internal_49 + Fu(5, 1)*(Fu(0, 1)*V_xx_prev(0, 5) + Fu(1, 1)*V_xx_prev(1, 5) + Fu(2, 1)*V_xx_prev(2, 5) + Fu(3, 1)*V_xx_prev(3, 5) + Fu(5, 1)*V_xx_prev(5, 5)) + Sigma_(2)*pow(g_u(2, 1), 2) + Sigma_(3)*pow(g_u(3, 1), 2) + l_uu(1, 1);
    Q_uu_(2, 2) = l_uu(2, 2);

  }

  virtual void updateQG(const Eigen::VectorXd& sl_x, const Eigen::VectorXd& sl_u, const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::VectorXd& r_f, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& V_xx_prev, const Eigen::VectorXd& V_x_prev) override {
    // Determine internal variables
    const double internal_0 = V_x_prev(0) + V_xx_prev(0, 0)*r_f(0) + V_xx_prev(0, 1)*r_f(1) + V_xx_prev(0, 2)*r_f(2) + V_xx_prev(0, 3)*r_f(3) + V_xx_prev(0, 4)*r_f(4) + V_xx_prev(0, 5)*r_f(5);
    const double internal_1 = V_x_prev(1) + V_xx_prev(1, 0)*r_f(0) + V_xx_prev(1, 1)*r_f(1) + V_xx_prev(1, 2)*r_f(2) + V_xx_prev(1, 3)*r_f(3) + V_xx_prev(1, 4)*r_f(4) + V_xx_prev(1, 5)*r_f(5);
    const double internal_2 = V_x_prev(2) + V_xx_prev(2, 0)*r_f(0) + V_xx_prev(2, 1)*r_f(1) + V_xx_prev(2, 2)*r_f(2) + V_xx_prev(2, 3)*r_f(3) + V_xx_prev(2, 4)*r_f(4) + V_xx_prev(2, 5)*r_f(5);
    const double internal_3 = V_x_prev(3) + V_xx_prev(3, 0)*r_f(0) + V_xx_prev(3, 1)*r_f(1) + V_xx_prev(3, 2)*r_f(2) + V_xx_prev(3, 3)*r_f(3) + V_xx_prev(3, 4)*r_f(4) + V_xx_prev(3, 5)*r_f(5);
    const double internal_4 = V_x_prev(4) + V_xx_prev(4, 0)*r_f(0) + V_xx_prev(4, 1)*r_f(1) + V_xx_prev(4, 2)*r_f(2) + V_xx_prev(4, 3)*r_f(3) + V_xx_prev(4, 4)*r_f(4) + V_xx_prev(4, 5)*r_f(5);
    const double internal_5 = V_x_prev(5) + V_xx_prev(5, 0)*r_f(0) + V_xx_prev(5, 1)*r_f(1) + V_xx_prev(5, 2)*r_f(2) + V_xx_prev(5, 3)*r_f(3) + V_xx_prev(5, 4)*r_f(4) + V_xx_prev(5, 5)*r_f(5);

    // Evaluation of Vector Q_x_
    Q_x_(0) = Fx(0, 0)*internal_0 + sl_x(0);
    Q_x_(1) = Fx(0, 1)*internal_0 + Fx(1, 1)*internal_1 + sl_x(1);
    Q_x_(2) = Fx(0, 2)*internal_0 + Fx(2, 2)*internal_2 + sl_x(2);
    Q_x_(3) = Fx(0, 3)*internal_0 + Fx(1, 3)*internal_1 + Fx(2, 3)*internal_2 + Fx(3, 3)*internal_3 + sl_x(3);
    Q_x_(4) = Fx(0, 4)*internal_0 + Fx(1, 4)*internal_1 + Fx(2, 4)*internal_2 + Fx(3, 4)*internal_3 + Fx(4, 4)*internal_4 + sl_x(4);
    Q_x_(5) = Fx(0, 5)*internal_0 + Fx(1, 5)*internal_1 + Fx(2, 5)*internal_2 + Fx(3, 5)*internal_3 + Fx(5, 5)*internal_5 + sl_x(5);

    // Evaluation of Vector Q_u_
    Q_u_(0) = Fu(0, 0)*internal_0 + Fu(1, 0)*internal_1 + Fu(2, 0)*internal_2 + Fu(3, 0)*internal_3 + Fu(4, 0)*internal_4 + V_(0)*g_u(0, 0) + V_(1)*g_u(1, 0) + sl_u(0);
    Q_u_(1) = Fu(0, 1)*internal_0 + Fu(1, 1)*internal_1 + Fu(2, 1)*internal_2 + Fu(3, 1)*internal_3 + Fu(5, 1)*internal_5 + V_(2)*g_u(2, 1) + V_(3)*g_u(3, 1) + sl_u(1);
    Q_u_(2) = sl_u(2);

  }

  virtual void updateQuuInv() override {
    // Determine internal variables
    const double internal_0 = 1.0/Q_uu_(0, 0);
    const double internal_1 = pow(Q_uu_(0, 1), 2);
    const double internal_2 = 1.0/(Q_uu_(1, 1) - internal_0*internal_1);
    const double internal_3 = -Q_uu_(0, 1)*internal_0*internal_2;

    // Evaluation of Matrix Q_uu_inv_
    Q_uu_inv_(0, 0) = internal_0 + internal_1*internal_2/pow(Q_uu_(0, 0), 2);
    Q_uu_inv_(0, 1) = internal_3;
    Q_uu_inv_(1, 0) = internal_3;
    Q_uu_inv_(1, 1) = internal_2;
    Q_uu_inv_(2, 2) = 1.0/Q_uu_(2, 2);

  }

  virtual void updateKVU(Eigen::MatrixXd& k_ux, Eigen::VectorXd& v_u) override {
    // Evaluation of Matrix k_ux
    k_ux(0, 0) = -Q_uu_inv_(0, 0)*Q_xu_(0, 0) - Q_uu_inv_(0, 1)*Q_xu_(0, 1);
    k_ux(0, 1) = -Q_uu_inv_(0, 0)*Q_xu_(1, 0) - Q_uu_inv_(0, 1)*Q_xu_(1, 1);
    k_ux(0, 2) = -Q_uu_inv_(0, 0)*Q_xu_(2, 0) - Q_uu_inv_(0, 1)*Q_xu_(2, 1);
    k_ux(0, 3) = -Q_uu_inv_(0, 0)*Q_xu_(3, 0) - Q_uu_inv_(0, 1)*Q_xu_(3, 1);
    k_ux(0, 4) = -Q_uu_inv_(0, 0)*Q_xu_(4, 0) - Q_uu_inv_(0, 1)*Q_xu_(4, 1);
    k_ux(0, 5) = -Q_uu_inv_(0, 0)*Q_xu_(5, 0) - Q_uu_inv_(0, 1)*Q_xu_(5, 1);
    k_ux(1, 0) = -Q_uu_inv_(0, 1)*Q_xu_(0, 0) - Q_uu_inv_(1, 1)*Q_xu_(0, 1);
    k_ux(1, 1) = -Q_uu_inv_(0, 1)*Q_xu_(1, 0) - Q_uu_inv_(1, 1)*Q_xu_(1, 1);
    k_ux(1, 2) = -Q_uu_inv_(0, 1)*Q_xu_(2, 0) - Q_uu_inv_(1, 1)*Q_xu_(2, 1);
    k_ux(1, 3) = -Q_uu_inv_(0, 1)*Q_xu_(3, 0) - Q_uu_inv_(1, 1)*Q_xu_(3, 1);
    k_ux(1, 4) = -Q_uu_inv_(0, 1)*Q_xu_(4, 0) - Q_uu_inv_(1, 1)*Q_xu_(4, 1);
    k_ux(1, 5) = -Q_uu_inv_(0, 1)*Q_xu_(5, 0) - Q_uu_inv_(1, 1)*Q_xu_(5, 1);

    // Evaluation of Vector v_u
    v_u(0) = -Q_u_(0)*Q_uu_inv_(0, 0) - Q_u_(1)*Q_uu_inv_(0, 1);
    v_u(1) = -Q_u_(0)*Q_uu_inv_(0, 1) - Q_u_(1)*Q_uu_inv_(1, 1);
    v_u(2) = -Q_u_(2)*Q_uu_inv_(2, 2);

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
    V_xx(0, 0) = Q_xu_(0, 0)*k_ux(0, 0) + Q_xu_(0, 1)*k_ux(1, 0) + Q_xx_(0, 0);
    V_xx(0, 1) = Q_xu_(0, 0)*k_ux(0, 1) + Q_xu_(0, 1)*k_ux(1, 1) + Q_xx_(0, 1);
    V_xx(0, 2) = Q_xu_(0, 0)*k_ux(0, 2) + Q_xu_(0, 1)*k_ux(1, 2) + Q_xx_(0, 2);
    V_xx(0, 3) = Q_xu_(0, 0)*k_ux(0, 3) + Q_xu_(0, 1)*k_ux(1, 3) + Q_xx_(0, 3);
    V_xx(0, 4) = Q_xu_(0, 0)*k_ux(0, 4) + Q_xu_(0, 1)*k_ux(1, 4) + Q_xx_(0, 4);
    V_xx(0, 5) = Q_xu_(0, 0)*k_ux(0, 5) + Q_xu_(0, 1)*k_ux(1, 5) + Q_xx_(0, 5);
    V_xx(1, 0) = Q_xu_(1, 0)*k_ux(0, 0) + Q_xu_(1, 1)*k_ux(1, 0) + Q_xx_(0, 1);
    V_xx(1, 1) = Q_xu_(1, 0)*k_ux(0, 1) + Q_xu_(1, 1)*k_ux(1, 1) + Q_xx_(1, 1);
    V_xx(1, 2) = Q_xu_(1, 0)*k_ux(0, 2) + Q_xu_(1, 1)*k_ux(1, 2) + Q_xx_(1, 2);
    V_xx(1, 3) = Q_xu_(1, 0)*k_ux(0, 3) + Q_xu_(1, 1)*k_ux(1, 3) + Q_xx_(1, 3);
    V_xx(1, 4) = Q_xu_(1, 0)*k_ux(0, 4) + Q_xu_(1, 1)*k_ux(1, 4) + Q_xx_(1, 4);
    V_xx(1, 5) = Q_xu_(1, 0)*k_ux(0, 5) + Q_xu_(1, 1)*k_ux(1, 5) + Q_xx_(1, 5);
    V_xx(2, 0) = Q_xu_(2, 0)*k_ux(0, 0) + Q_xu_(2, 1)*k_ux(1, 0) + Q_xx_(0, 2);
    V_xx(2, 1) = Q_xu_(2, 0)*k_ux(0, 1) + Q_xu_(2, 1)*k_ux(1, 1) + Q_xx_(1, 2);
    V_xx(2, 2) = Q_xu_(2, 0)*k_ux(0, 2) + Q_xu_(2, 1)*k_ux(1, 2) + Q_xx_(2, 2);
    V_xx(2, 3) = Q_xu_(2, 0)*k_ux(0, 3) + Q_xu_(2, 1)*k_ux(1, 3) + Q_xx_(2, 3);
    V_xx(2, 4) = Q_xu_(2, 0)*k_ux(0, 4) + Q_xu_(2, 1)*k_ux(1, 4) + Q_xx_(2, 4);
    V_xx(2, 5) = Q_xu_(2, 0)*k_ux(0, 5) + Q_xu_(2, 1)*k_ux(1, 5) + Q_xx_(2, 5);
    V_xx(3, 0) = Q_xu_(3, 0)*k_ux(0, 0) + Q_xu_(3, 1)*k_ux(1, 0) + Q_xx_(0, 3);
    V_xx(3, 1) = Q_xu_(3, 0)*k_ux(0, 1) + Q_xu_(3, 1)*k_ux(1, 1) + Q_xx_(1, 3);
    V_xx(3, 2) = Q_xu_(3, 0)*k_ux(0, 2) + Q_xu_(3, 1)*k_ux(1, 2) + Q_xx_(2, 3);
    V_xx(3, 3) = Q_xu_(3, 0)*k_ux(0, 3) + Q_xu_(3, 1)*k_ux(1, 3) + Q_xx_(3, 3);
    V_xx(3, 4) = Q_xu_(3, 0)*k_ux(0, 4) + Q_xu_(3, 1)*k_ux(1, 4) + Q_xx_(3, 4);
    V_xx(3, 5) = Q_xu_(3, 0)*k_ux(0, 5) + Q_xu_(3, 1)*k_ux(1, 5) + Q_xx_(3, 5);
    V_xx(4, 0) = Q_xu_(4, 0)*k_ux(0, 0) + Q_xu_(4, 1)*k_ux(1, 0) + Q_xx_(0, 4);
    V_xx(4, 1) = Q_xu_(4, 0)*k_ux(0, 1) + Q_xu_(4, 1)*k_ux(1, 1) + Q_xx_(1, 4);
    V_xx(4, 2) = Q_xu_(4, 0)*k_ux(0, 2) + Q_xu_(4, 1)*k_ux(1, 2) + Q_xx_(2, 4);
    V_xx(4, 3) = Q_xu_(4, 0)*k_ux(0, 3) + Q_xu_(4, 1)*k_ux(1, 3) + Q_xx_(3, 4);
    V_xx(4, 4) = Q_xu_(4, 0)*k_ux(0, 4) + Q_xu_(4, 1)*k_ux(1, 4) + Q_xx_(4, 4);
    V_xx(4, 5) = Q_xu_(4, 0)*k_ux(0, 5) + Q_xu_(4, 1)*k_ux(1, 5) + Q_xx_(4, 5);
    V_xx(5, 0) = Q_xu_(5, 0)*k_ux(0, 0) + Q_xu_(5, 1)*k_ux(1, 0) + Q_xx_(0, 5);
    V_xx(5, 1) = Q_xu_(5, 0)*k_ux(0, 1) + Q_xu_(5, 1)*k_ux(1, 1) + Q_xx_(1, 5);
    V_xx(5, 2) = Q_xu_(5, 0)*k_ux(0, 2) + Q_xu_(5, 1)*k_ux(1, 2) + Q_xx_(2, 5);
    V_xx(5, 3) = Q_xu_(5, 0)*k_ux(0, 3) + Q_xu_(5, 1)*k_ux(1, 3) + Q_xx_(3, 5);
    V_xx(5, 4) = Q_xu_(5, 0)*k_ux(0, 4) + Q_xu_(5, 1)*k_ux(1, 4) + Q_xx_(4, 5);
    V_xx(5, 5) = Q_xu_(5, 0)*k_ux(0, 5) + Q_xu_(5, 1)*k_ux(1, 5) + Q_xx_(5, 5);

    // Evaluation of Vector V_x
    V_x(0) = Q_x_(0) + Q_xu_(0, 0)*v_u(0) + Q_xu_(0, 1)*v_u(1);
    V_x(1) = Q_x_(1) + Q_xu_(1, 0)*v_u(0) + Q_xu_(1, 1)*v_u(1);
    V_x(2) = Q_x_(2) + Q_xu_(2, 0)*v_u(0) + Q_xu_(2, 1)*v_u(1);
    V_x(3) = Q_x_(3) + Q_xu_(3, 0)*v_u(0) + Q_xu_(3, 1)*v_u(1);
    V_x(4) = Q_x_(4) + Q_xu_(4, 0)*v_u(0) + Q_xu_(4, 1)*v_u(1);
    V_x(5) = Q_x_(5) + Q_xu_(5, 0)*v_u(0) + Q_xu_(5, 1)*v_u(1);

  }

  virtual void updateQnx(const Eigen::VectorXd& r_f, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& h_u, const Eigen::MatrixXd& k_ux, const Eigen::MatrixXd& k_ex, const Eigen::MatrixXd& k_ue, const Eigen::VectorXd& v_u, const Eigen::VectorXd& v_e) override {
    // Determine internal variables
    const double internal_0 = -Q_uu_(0, 1);

    // Evaluation of Matrix Q_un_
    Q_un_(0, 0) = -Fu(0, 0)*Q_uu_inv_(0, 0) - Fu(0, 1)*Q_uu_inv_(0, 1);
    Q_un_(0, 1) = -Fu(1, 0)*Q_uu_inv_(0, 0) - Fu(1, 1)*Q_uu_inv_(0, 1);
    Q_un_(0, 2) = -Fu(2, 0)*Q_uu_inv_(0, 0) - Fu(2, 1)*Q_uu_inv_(0, 1);
    Q_un_(0, 3) = -Fu(3, 0)*Q_uu_inv_(0, 0) - Fu(3, 1)*Q_uu_inv_(0, 1);
    Q_un_(0, 4) = -Fu(4, 0)*Q_uu_inv_(0, 0);
    Q_un_(0, 5) = -Fu(5, 1)*Q_uu_inv_(0, 1);
    Q_un_(1, 0) = -Fu(0, 0)*Q_uu_inv_(0, 1) - Fu(0, 1)*Q_uu_inv_(1, 1);
    Q_un_(1, 1) = -Fu(1, 0)*Q_uu_inv_(0, 1) - Fu(1, 1)*Q_uu_inv_(1, 1);
    Q_un_(1, 2) = -Fu(2, 0)*Q_uu_inv_(0, 1) - Fu(2, 1)*Q_uu_inv_(1, 1);
    Q_un_(1, 3) = -Fu(3, 0)*Q_uu_inv_(0, 1) - Fu(3, 1)*Q_uu_inv_(1, 1);
    Q_un_(1, 4) = -Fu(4, 0)*Q_uu_inv_(0, 1);
    Q_un_(1, 5) = -Fu(5, 1)*Q_uu_inv_(1, 1);

    // Evaluation of Matrix Q_nuu_
    Q_nuu_(0, 0) = -Q_uu_(0, 0);
    Q_nuu_(0, 1) = internal_0;
    Q_nuu_(1, 0) = internal_0;
    Q_nuu_(1, 1) = -Q_uu_(1, 1);
    Q_nuu_(2, 2) = -Q_uu_(2, 2);

    // Evaluation of Matrix Q_vnx_
    Q_vnx_(0, 0) = Fu(0, 0)*k_ux(0, 0) + Fu(0, 1)*k_ux(1, 0) + Fx(0, 0);
    Q_vnx_(0, 1) = Fu(0, 0)*k_ux(0, 1) + Fu(0, 1)*k_ux(1, 1) + Fx(0, 1);
    Q_vnx_(0, 2) = Fu(0, 0)*k_ux(0, 2) + Fu(0, 1)*k_ux(1, 2) + Fx(0, 2);
    Q_vnx_(0, 3) = Fu(0, 0)*k_ux(0, 3) + Fu(0, 1)*k_ux(1, 3) + Fx(0, 3);
    Q_vnx_(0, 4) = Fu(0, 0)*k_ux(0, 4) + Fu(0, 1)*k_ux(1, 4) + Fx(0, 4);
    Q_vnx_(0, 5) = Fu(0, 0)*k_ux(0, 5) + Fu(0, 1)*k_ux(1, 5) + Fx(0, 5);
    Q_vnx_(1, 0) = Fu(1, 0)*k_ux(0, 0) + Fu(1, 1)*k_ux(1, 0);
    Q_vnx_(1, 1) = Fu(1, 0)*k_ux(0, 1) + Fu(1, 1)*k_ux(1, 1) + Fx(1, 1);
    Q_vnx_(1, 2) = Fu(1, 0)*k_ux(0, 2) + Fu(1, 1)*k_ux(1, 2);
    Q_vnx_(1, 3) = Fu(1, 0)*k_ux(0, 3) + Fu(1, 1)*k_ux(1, 3) + Fx(1, 3);
    Q_vnx_(1, 4) = Fu(1, 0)*k_ux(0, 4) + Fu(1, 1)*k_ux(1, 4) + Fx(1, 4);
    Q_vnx_(1, 5) = Fu(1, 0)*k_ux(0, 5) + Fu(1, 1)*k_ux(1, 5) + Fx(1, 5);
    Q_vnx_(2, 0) = Fu(2, 0)*k_ux(0, 0) + Fu(2, 1)*k_ux(1, 0);
    Q_vnx_(2, 1) = Fu(2, 0)*k_ux(0, 1) + Fu(2, 1)*k_ux(1, 1);
    Q_vnx_(2, 2) = Fu(2, 0)*k_ux(0, 2) + Fu(2, 1)*k_ux(1, 2) + Fx(2, 2);
    Q_vnx_(2, 3) = Fu(2, 0)*k_ux(0, 3) + Fu(2, 1)*k_ux(1, 3) + Fx(2, 3);
    Q_vnx_(2, 4) = Fu(2, 0)*k_ux(0, 4) + Fu(2, 1)*k_ux(1, 4) + Fx(2, 4);
    Q_vnx_(2, 5) = Fu(2, 0)*k_ux(0, 5) + Fu(2, 1)*k_ux(1, 5) + Fx(2, 5);
    Q_vnx_(3, 0) = Fu(3, 0)*k_ux(0, 0) + Fu(3, 1)*k_ux(1, 0);
    Q_vnx_(3, 1) = Fu(3, 0)*k_ux(0, 1) + Fu(3, 1)*k_ux(1, 1);
    Q_vnx_(3, 2) = Fu(3, 0)*k_ux(0, 2) + Fu(3, 1)*k_ux(1, 2);
    Q_vnx_(3, 3) = Fu(3, 0)*k_ux(0, 3) + Fu(3, 1)*k_ux(1, 3) + Fx(3, 3);
    Q_vnx_(3, 4) = Fu(3, 0)*k_ux(0, 4) + Fu(3, 1)*k_ux(1, 4) + Fx(3, 4);
    Q_vnx_(3, 5) = Fu(3, 0)*k_ux(0, 5) + Fu(3, 1)*k_ux(1, 5) + Fx(3, 5);
    Q_vnx_(4, 0) = Fu(4, 0)*k_ux(0, 0);
    Q_vnx_(4, 1) = Fu(4, 0)*k_ux(0, 1);
    Q_vnx_(4, 2) = Fu(4, 0)*k_ux(0, 2);
    Q_vnx_(4, 3) = Fu(4, 0)*k_ux(0, 3);
    Q_vnx_(4, 4) = Fu(4, 0)*k_ux(0, 4) + Fx(4, 4);
    Q_vnx_(4, 5) = Fu(4, 0)*k_ux(0, 5);
    Q_vnx_(5, 0) = Fu(5, 1)*k_ux(1, 0);
    Q_vnx_(5, 1) = Fu(5, 1)*k_ux(1, 1);
    Q_vnx_(5, 2) = Fu(5, 1)*k_ux(1, 2);
    Q_vnx_(5, 3) = Fu(5, 1)*k_ux(1, 3);
    Q_vnx_(5, 4) = Fu(5, 1)*k_ux(1, 4);
    Q_vnx_(5, 5) = Fu(5, 1)*k_ux(1, 5) + Fx(5, 5);

    // Evaluation of Vector Q_vn_
    Q_vn_(0) = Fu(0, 0)*v_u(0) + Fu(0, 1)*v_u(1) + r_f(0);
    Q_vn_(1) = Fu(1, 0)*v_u(0) + Fu(1, 1)*v_u(1) + r_f(1);
    Q_vn_(2) = Fu(2, 0)*v_u(0) + Fu(2, 1)*v_u(1) + r_f(2);
    Q_vn_(3) = Fu(3, 0)*v_u(0) + Fu(3, 1)*v_u(1) + r_f(3);
    Q_vn_(4) = Fu(4, 0)*v_u(0) + r_f(4);
    Q_vn_(5) = Fu(5, 1)*v_u(1) + r_f(5);

  }

};

class ParkingGeneralIpmEvaluatorTerminal : public IpmEvaluatorTerminal {
 public:
  ParkingGeneralIpmEvaluatorTerminal() : IpmEvaluatorTerminal(6, 3, 11, 3) {}
  virtual ~ParkingGeneralIpmEvaluatorTerminal() = default;

  virtual void updateLX(const Eigen::VectorXd& l_x, const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& m_x, const Eigen::VectorXd& p, const Eigen::VectorXd& lambda, const Eigen::VectorXd& nu, Eigen::VectorXd& sl_x) const override {
    // Evaluation of Vector sl_x
    sl_x(0) = -p(0);
    sl_x(1) = g_x(0, 1)*lambda(0) + g_x(1, 1)*lambda(1) + g_x(2, 1)*lambda(2) + g_x(3, 1)*lambda(3) + g_x(4, 1)*lambda(4) + g_x(5, 1)*lambda(5) + l_x(1) + m_x(0, 1)*nu(0) - p(1);
    sl_x(2) = g_x(0, 2)*lambda(0) + g_x(1, 2)*lambda(1) + g_x(2, 2)*lambda(2) + g_x(3, 2)*lambda(3) + g_x(4, 2)*lambda(4) + g_x(5, 2)*lambda(5) + l_x(2) + m_x(1, 2)*nu(1) - p(2);
    sl_x(3) = g_x(10, 3)*lambda(10) + g_x(2, 3)*lambda(2) + g_x(3, 3)*lambda(3) + g_x(4, 3)*lambda(4) + g_x(5, 3)*lambda(5) + m_x(2, 3)*nu(2) - p(3);
    sl_x(4) = g_x(6, 4)*lambda(6) + g_x(7, 4)*lambda(7) + l_x(4) - p(4);
    sl_x(5) = g_x(8, 5)*lambda(8) + g_x(9, 5)*lambda(9) + l_x(5) - p(5);

  }

  virtual void updateVxx(const Eigen::MatrixXd& l_xx, const Eigen::VectorXd& sl_x, const Eigen::MatrixXd& g_x, Eigen::MatrixXd& V_xx, Eigen::VectorXd& V_x) override {
    // Determine internal variables
    const double internal_0 = Sigma_(2)*g_x(2, 1);
    const double internal_1 = Sigma_(3)*g_x(3, 1);
    const double internal_2 = Sigma_(4)*g_x(4, 1);
    const double internal_3 = Sigma_(5)*g_x(5, 1);
    const double internal_6 = Sigma_(2)*g_x(2, 2)*g_x(2, 3) + Sigma_(3)*g_x(3, 2)*g_x(3, 3) + Sigma_(4)*g_x(4, 2)*g_x(4, 3) + Sigma_(5)*g_x(5, 2)*g_x(5, 3);
    const double internal_4 = Sigma_(0)*g_x(0, 1)*g_x(0, 2) + Sigma_(1)*g_x(1, 1)*g_x(1, 2) + g_x(2, 2)*internal_0 + g_x(3, 2)*internal_1 + g_x(4, 2)*internal_2 + g_x(5, 2)*internal_3;
    const double internal_5 = g_x(2, 3)*internal_0 + g_x(3, 3)*internal_1 + g_x(4, 3)*internal_2 + g_x(5, 3)*internal_3;

    // Evaluation of Matrix V_xx
    V_xx(1, 1) = Sigma_(0)*pow(g_x(0, 1), 2) + Sigma_(1)*pow(g_x(1, 1), 2) + Sigma_(2)*pow(g_x(2, 1), 2) + Sigma_(3)*pow(g_x(3, 1), 2) + Sigma_(4)*pow(g_x(4, 1), 2) + Sigma_(5)*pow(g_x(5, 1), 2) + l_xx(1, 1);
    V_xx(1, 2) = internal_4 + l_xx(1, 2);
    V_xx(1, 3) = internal_5;
    V_xx(2, 1) = internal_4 + l_xx(2, 1);
    V_xx(2, 2) = Sigma_(0)*pow(g_x(0, 2), 2) + Sigma_(1)*pow(g_x(1, 2), 2) + Sigma_(2)*pow(g_x(2, 2), 2) + Sigma_(3)*pow(g_x(3, 2), 2) + Sigma_(4)*pow(g_x(4, 2), 2) + Sigma_(5)*pow(g_x(5, 2), 2) + l_xx(2, 2);
    V_xx(2, 3) = internal_6;
    V_xx(3, 1) = internal_5;
    V_xx(3, 2) = internal_6;
    V_xx(3, 3) = Sigma_(10)*pow(g_x(10, 3), 2) + Sigma_(2)*pow(g_x(2, 3), 2) + Sigma_(3)*pow(g_x(3, 3), 2) + Sigma_(4)*pow(g_x(4, 3), 2) + Sigma_(5)*pow(g_x(5, 3), 2);
    V_xx(4, 4) = Sigma_(6)*pow(g_x(6, 4), 2) + Sigma_(7)*pow(g_x(7, 4), 2) + l_xx(4, 4);
    V_xx(5, 5) = Sigma_(8)*pow(g_x(8, 5), 2) + Sigma_(9)*pow(g_x(9, 5), 2) + l_xx(5, 5);

    // Evaluation of Vector V_x
    V_x(0) = sl_x(0);
    V_x(1) = V_(0)*g_x(0, 1) + V_(1)*g_x(1, 1) + V_(2)*g_x(2, 1) + V_(3)*g_x(3, 1) + V_(4)*g_x(4, 1) + V_(5)*g_x(5, 1) + sl_x(1);
    V_x(2) = V_(0)*g_x(0, 2) + V_(1)*g_x(1, 2) + V_(2)*g_x(2, 2) + V_(3)*g_x(3, 2) + V_(4)*g_x(4, 2) + V_(5)*g_x(5, 2) + sl_x(2);
    V_x(3) = V_(10)*g_x(10, 3) + V_(2)*g_x(2, 3) + V_(3)*g_x(3, 3) + V_(4)*g_x(4, 3) + V_(5)*g_x(5, 3) + sl_x(3);
    V_x(4) = V_(6)*g_x(6, 4) + V_(7)*g_x(7, 4) + sl_x(4);
    V_x(5) = V_(8)*g_x(8, 5) + V_(9)*g_x(9, 5) + sl_x(5);

  }

};

}  // namespace gpal::pnc::planning