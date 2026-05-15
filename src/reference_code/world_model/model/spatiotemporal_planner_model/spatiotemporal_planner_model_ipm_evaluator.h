#pragma once

#include "ocp/ipm_evaluator.h"

namespace gpal::pnc::planning {

template <StageType stage_type, OcpConfig::IntegratorType integrator_type>
class SpatiotemporalPlannerModelIpmEvaluator : public IpmEvaluator {
 public:
  SpatiotemporalPlannerModelIpmEvaluator() : IpmEvaluator(7, 13, 43, 0, 0) {}
  virtual ~SpatiotemporalPlannerModelIpmEvaluator() = default;

  virtual void updateLX(const Eigen::VectorXd& l_x, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& h_x, const Eigen::MatrixXd& m_x, const Eigen::VectorXd& p, const Eigen::VectorXd& lambda, const Eigen::VectorXd& eta, const Eigen::VectorXd& nu, Eigen::VectorXd& sl_x) const override {
    // Evaluation of Vector sl_x
    sl_x(0) = Fx(0, 0)*p(0) + g_x(41, 0)*lambda(41) + g_x(8, 0)*lambda(8) + g_x(9, 0)*lambda(9) + l_x(0);
    sl_x(1) = Fx(0, 1)*p(0) + Fx(1, 1)*p(1) + g_x(0, 1)*lambda(0) + g_x(1, 1)*lambda(1) + g_x(19, 1)*lambda(19) + g_x(2, 1)*lambda(2) + g_x(20, 1)*lambda(20) + g_x(21, 1)*lambda(21) + g_x(22, 1)*lambda(22) + g_x(23, 1)*lambda(23) + g_x(24, 1)*lambda(24) + g_x(25, 1)*lambda(25) + g_x(26, 1)*lambda(26) + g_x(27, 1)*lambda(27) + g_x(28, 1)*lambda(28) + g_x(29, 1)*lambda(29) + g_x(3, 1)*lambda(3) + g_x(30, 1)*lambda(30) + g_x(31, 1)*lambda(31) + g_x(32, 1)*lambda(32) + g_x(33, 1)*lambda(33) + g_x(34, 1)*lambda(34) + g_x(35, 1)*lambda(35) + g_x(36, 1)*lambda(36) + g_x(37, 1)*lambda(37) + g_x(38, 1)*lambda(38) + g_x(39, 1)*lambda(39) + g_x(4, 1)*lambda(4) + g_x(40, 1)*lambda(40) + g_x(5, 1)*lambda(5) + l_x(1);
    sl_x(2) = Fx(0, 2)*p(0) + Fx(2, 2)*p(2) + g_x(0, 2)*lambda(0) + g_x(1, 2)*lambda(1) + g_x(19, 2)*lambda(19) + g_x(2, 2)*lambda(2) + g_x(20, 2)*lambda(20) + g_x(21, 2)*lambda(21) + g_x(22, 2)*lambda(22) + g_x(23, 2)*lambda(23) + g_x(24, 2)*lambda(24) + g_x(25, 2)*lambda(25) + g_x(26, 2)*lambda(26) + g_x(27, 2)*lambda(27) + g_x(28, 2)*lambda(28) + g_x(29, 2)*lambda(29) + g_x(3, 2)*lambda(3) + g_x(30, 2)*lambda(30) + g_x(31, 2)*lambda(31) + g_x(32, 2)*lambda(32) + g_x(33, 2)*lambda(33) + g_x(34, 2)*lambda(34) + g_x(35, 2)*lambda(35) + g_x(36, 2)*lambda(36) + g_x(37, 2)*lambda(37) + g_x(38, 2)*lambda(38) + g_x(39, 2)*lambda(39) + g_x(4, 2)*lambda(4) + g_x(40, 2)*lambda(40) + g_x(5, 2)*lambda(5) + l_x(2);
    sl_x(3) = Fx(0, 3)*p(0) + Fx(1, 3)*p(1) + Fx(2, 3)*p(2) + Fx(3, 3)*p(3) + g_x(14, 3)*lambda(14) + g_x(19, 3)*lambda(19) + g_x(2, 3)*lambda(2) + g_x(20, 3)*lambda(20) + g_x(21, 3)*lambda(21) + g_x(22, 3)*lambda(22) + g_x(23, 3)*lambda(23) + g_x(24, 3)*lambda(24) + g_x(25, 3)*lambda(25) + g_x(26, 3)*lambda(26) + g_x(27, 3)*lambda(27) + g_x(28, 3)*lambda(28) + g_x(29, 3)*lambda(29) + g_x(3, 3)*lambda(3) + g_x(30, 3)*lambda(30) + g_x(31, 3)*lambda(31) + g_x(32, 3)*lambda(32) + g_x(33, 3)*lambda(33) + g_x(34, 3)*lambda(34) + g_x(37, 3)*lambda(37) + g_x(38, 3)*lambda(38) + g_x(39, 3)*lambda(39) + g_x(4, 3)*lambda(4) + g_x(40, 3)*lambda(40) + g_x(5, 3)*lambda(5) + l_x(3);
    sl_x(4) = Fx(3, 4)*p(3) + Fx(4, 4)*p(4) + g_x(6, 4)*lambda(6) + g_x(7, 4)*lambda(7) + l_x(4);
    sl_x(5) = Fx(0, 5)*p(0) + Fx(1, 5)*p(1) + Fx(2, 5)*p(2) + Fx(3, 5)*p(3) + Fx(5, 5)*p(5) + g_x(10, 5)*lambda(10) + g_x(11, 5)*lambda(11) + g_x(42, 5)*lambda(42) + l_x(5);
    sl_x(6) = Fx(5, 6)*p(5) + Fx(6, 6)*p(6) + g_x(12, 6)*lambda(12) + g_x(13, 6)*lambda(13) + l_x(6);

  }

  virtual void updateLU(const Eigen::VectorXd& l_u, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& g_u, const Eigen::MatrixXd& h_u, const Eigen::VectorXd& p, const Eigen::VectorXd& lambda, const Eigen::VectorXd& eta, Eigen::VectorXd& sl_u) const override {
    // Evaluation of Vector sl_u
    sl_u(0) = Fu(4, 0)*p(4) + g_u(15, 0)*lambda(15) + g_u(16, 0)*lambda(16) + l_u(0);
    sl_u(1) = Fu(6, 1)*p(6) + g_u(17, 1)*lambda(17) + g_u(18, 1)*lambda(18) + l_u(1);
    sl_u(2) = g_u(35, 2)*lambda(35) + g_u(36, 2)*lambda(36) + g_u(37, 2)*lambda(37) + g_u(38, 2)*lambda(38) + g_u(39, 2)*lambda(39) + g_u(40, 2)*lambda(40) + l_u(2);
    sl_u(3) = g_u(41, 3)*lambda(41) + l_u(3);
    sl_u(4) = g_u(42, 4)*lambda(42) + l_u(4);
    sl_u(5) = g_u(19, 5)*lambda(19) + g_u(27, 5)*lambda(27) + l_u(5);
    sl_u(6) = g_u(20, 6)*lambda(20) + g_u(28, 6)*lambda(28) + l_u(6);
    sl_u(7) = g_u(21, 7)*lambda(21) + g_u(29, 7)*lambda(29) + l_u(7);
    sl_u(8) = g_u(22, 8)*lambda(22) + g_u(30, 8)*lambda(30) + l_u(8);
    sl_u(9) = g_u(23, 9)*lambda(23) + g_u(31, 9)*lambda(31) + l_u(9);
    sl_u(10) = g_u(24, 10)*lambda(24) + g_u(32, 10)*lambda(32) + l_u(10);
    sl_u(11) = g_u(25, 11)*lambda(25) + g_u(33, 11)*lambda(33) + l_u(11);
    sl_u(12) = g_u(26, 12)*lambda(26) + g_u(34, 12)*lambda(34) + l_u(12);

  }

  virtual void updateQH(const Eigen::MatrixXd& l_xx, const Eigen::MatrixXd& l_xu, const Eigen::MatrixXd& l_uu, const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& V_xx_prev) override {
    // Determine internal variables
    const double internal_0 = Fx(0, 1)*V_xx_prev(0, 0);
    const double internal_1 = Fx(0, 0)*V_xx_prev(0, 1);
    const double internal_101 = Fu(4, 0)*Fu(6, 1);
    const double internal_13 = Sigma_(19)*g_x(19, 1);
    const double internal_14 = Sigma_(2)*g_x(2, 1);
    const double internal_15 = Sigma_(20)*g_x(20, 1);
    const double internal_16 = Sigma_(21)*g_x(21, 1);
    const double internal_17 = Sigma_(22)*g_x(22, 1);
    const double internal_18 = Sigma_(23)*g_x(23, 1);
    const double internal_19 = Sigma_(24)*g_x(24, 1);
    const double internal_2 = Fx(0, 2)*V_xx_prev(0, 0);
    const double internal_20 = Sigma_(25)*g_x(25, 1);
    const double internal_21 = Sigma_(26)*g_x(26, 1);
    const double internal_22 = Sigma_(27)*g_x(27, 1);
    const double internal_23 = Sigma_(28)*g_x(28, 1);
    const double internal_24 = Sigma_(29)*g_x(29, 1);
    const double internal_25 = Sigma_(3)*g_x(3, 1);
    const double internal_26 = Sigma_(30)*g_x(30, 1);
    const double internal_27 = Sigma_(31)*g_x(31, 1);
    const double internal_28 = Sigma_(32)*g_x(32, 1);
    const double internal_29 = Sigma_(33)*g_x(33, 1);
    const double internal_3 = Fx(0, 0)*V_xx_prev(0, 2);
    const double internal_30 = Sigma_(34)*g_x(34, 1);
    const double internal_31 = Sigma_(35)*g_x(35, 1);
    const double internal_32 = Sigma_(36)*g_x(36, 1);
    const double internal_33 = Sigma_(37)*g_x(37, 1);
    const double internal_34 = Sigma_(38)*g_x(38, 1);
    const double internal_35 = Sigma_(39)*g_x(39, 1);
    const double internal_36 = Sigma_(4)*g_x(4, 1);
    const double internal_37 = Sigma_(40)*g_x(40, 1);
    const double internal_38 = Sigma_(5)*g_x(5, 1);
    const double internal_4 = Fx(0, 3)*V_xx_prev(0, 0);
    const double internal_49 = Sigma_(19)*g_x(19, 2);
    const double internal_5 = Fx(0, 0)*V_xx_prev(0, 3);
    const double internal_50 = Sigma_(20)*g_x(20, 2);
    const double internal_51 = Sigma_(21)*g_x(21, 2);
    const double internal_52 = Sigma_(22)*g_x(22, 2);
    const double internal_53 = Sigma_(23)*g_x(23, 2);
    const double internal_54 = Sigma_(24)*g_x(24, 2);
    const double internal_55 = Sigma_(25)*g_x(25, 2);
    const double internal_56 = Sigma_(26)*g_x(26, 2);
    const double internal_57 = Sigma_(27)*g_x(27, 2);
    const double internal_58 = Sigma_(28)*g_x(28, 2);
    const double internal_59 = Sigma_(29)*g_x(29, 2);
    const double internal_6 = Fx(0, 0)*V_xx_prev(0, 4);
    const double internal_60 = Sigma_(30)*g_x(30, 2);
    const double internal_61 = Sigma_(31)*g_x(31, 2);
    const double internal_62 = Sigma_(32)*g_x(32, 2);
    const double internal_63 = Sigma_(33)*g_x(33, 2);
    const double internal_64 = Sigma_(34)*g_x(34, 2);
    const double internal_65 = Sigma_(37)*g_x(37, 2);
    const double internal_66 = Sigma_(38)*g_x(38, 2);
    const double internal_67 = Sigma_(39)*g_x(39, 2);
    const double internal_68 = Sigma_(40)*g_x(40, 2);
    const double internal_7 = Fx(0, 5)*V_xx_prev(0, 0);
    const double internal_8 = Fx(0, 0)*V_xx_prev(0, 5);
    const double internal_9 = Fx(0, 0)*V_xx_prev(0, 6);
    const double internal_100 = Fx(5, 6)*V_xx_prev(5, 6) + Fx(6, 6)*V_xx_prev(6, 6);
    const double internal_11 = Fx(0, 1)*V_xx_prev(0, 1) + Fx(1, 1)*V_xx_prev(1, 1);
    const double internal_12 = Fx(0, 1)*V_xx_prev(0, 2) + Fx(1, 1)*V_xx_prev(1, 2);
    const double internal_40 = Fx(0, 1)*V_xx_prev(0, 3) + Fx(1, 1)*V_xx_prev(1, 3);
    const double internal_42 = Fx(0, 1)*V_xx_prev(0, 4) + Fx(1, 1)*V_xx_prev(1, 4);
    const double internal_43 = Fx(0, 1)*V_xx_prev(0, 5) + Fx(1, 1)*V_xx_prev(1, 5);
    const double internal_44 = Fx(0, 1)*V_xx_prev(0, 6) + Fx(1, 1)*V_xx_prev(1, 6);
    const double internal_46 = Fx(0, 2)*V_xx_prev(0, 1) + Fx(2, 2)*V_xx_prev(2, 1);
    const double internal_47 = Fx(0, 2)*V_xx_prev(0, 2) + Fx(2, 2)*V_xx_prev(2, 2);
    const double internal_48 = Fx(0, 2)*V_xx_prev(0, 3) + Fx(2, 2)*V_xx_prev(2, 3);
    const double internal_70 = Fx(0, 2)*V_xx_prev(0, 4) + Fx(2, 2)*V_xx_prev(2, 4);
    const double internal_71 = Fx(0, 2)*V_xx_prev(0, 5) + Fx(2, 2)*V_xx_prev(2, 5);
    const double internal_72 = Fx(0, 2)*V_xx_prev(0, 6) + Fx(2, 2)*V_xx_prev(2, 6);
    const double internal_74 = Fx(0, 3)*V_xx_prev(0, 1) + Fx(1, 3)*V_xx_prev(1, 1) + Fx(2, 3)*V_xx_prev(2, 1) + Fx(3, 3)*V_xx_prev(3, 1);
    const double internal_75 = Fx(0, 3)*V_xx_prev(0, 2) + Fx(1, 3)*V_xx_prev(1, 2) + Fx(2, 3)*V_xx_prev(2, 2) + Fx(3, 3)*V_xx_prev(3, 2);
    const double internal_76 = Fx(0, 3)*V_xx_prev(0, 3) + Fx(1, 3)*V_xx_prev(1, 3) + Fx(2, 3)*V_xx_prev(2, 3) + Fx(3, 3)*V_xx_prev(3, 3);
    const double internal_77 = Fx(0, 3)*V_xx_prev(0, 4) + Fx(1, 3)*V_xx_prev(1, 4) + Fx(2, 3)*V_xx_prev(2, 4) + Fx(3, 3)*V_xx_prev(3, 4);
    const double internal_78 = Fx(0, 3)*V_xx_prev(0, 5) + Fx(1, 3)*V_xx_prev(1, 5) + Fx(2, 3)*V_xx_prev(2, 5) + Fx(3, 3)*V_xx_prev(3, 5);
    const double internal_79 = Fx(0, 3)*V_xx_prev(0, 6) + Fx(1, 3)*V_xx_prev(1, 6) + Fx(2, 3)*V_xx_prev(2, 6) + Fx(3, 3)*V_xx_prev(3, 6);
    const double internal_80 = Fx(3, 4)*V_xx_prev(3, 0) + Fx(4, 4)*V_xx_prev(4, 0);
    const double internal_81 = Fx(3, 4)*V_xx_prev(3, 1) + Fx(4, 4)*V_xx_prev(4, 1);
    const double internal_82 = Fx(3, 4)*V_xx_prev(3, 2) + Fx(4, 4)*V_xx_prev(4, 2);
    const double internal_83 = Fx(3, 4)*V_xx_prev(3, 3) + Fx(4, 4)*V_xx_prev(4, 3);
    const double internal_84 = Fx(3, 4)*V_xx_prev(3, 4) + Fx(4, 4)*V_xx_prev(4, 4);
    const double internal_85 = Fx(3, 4)*V_xx_prev(3, 5) + Fx(4, 4)*V_xx_prev(4, 5);
    const double internal_86 = Fx(3, 4)*V_xx_prev(3, 6) + Fx(4, 4)*V_xx_prev(4, 6);
    const double internal_88 = Fx(0, 5)*V_xx_prev(0, 1) + Fx(1, 5)*V_xx_prev(1, 1) + Fx(2, 5)*V_xx_prev(2, 1) + Fx(3, 5)*V_xx_prev(3, 1) + Fx(5, 5)*V_xx_prev(5, 1);
    const double internal_89 = Fx(0, 5)*V_xx_prev(0, 2) + Fx(1, 5)*V_xx_prev(1, 2) + Fx(2, 5)*V_xx_prev(2, 2) + Fx(3, 5)*V_xx_prev(3, 2) + Fx(5, 5)*V_xx_prev(5, 2);
    const double internal_90 = Fx(0, 5)*V_xx_prev(0, 3) + Fx(1, 5)*V_xx_prev(1, 3) + Fx(2, 5)*V_xx_prev(2, 3) + Fx(3, 5)*V_xx_prev(3, 3) + Fx(5, 5)*V_xx_prev(5, 3);
    const double internal_91 = Fx(0, 5)*V_xx_prev(0, 4) + Fx(1, 5)*V_xx_prev(1, 4) + Fx(2, 5)*V_xx_prev(2, 4) + Fx(3, 5)*V_xx_prev(3, 4) + Fx(5, 5)*V_xx_prev(5, 4);
    const double internal_92 = Fx(0, 5)*V_xx_prev(0, 5) + Fx(1, 5)*V_xx_prev(1, 5) + Fx(2, 5)*V_xx_prev(2, 5) + Fx(3, 5)*V_xx_prev(3, 5) + Fx(5, 5)*V_xx_prev(5, 5);
    const double internal_93 = Fx(0, 5)*V_xx_prev(0, 6) + Fx(1, 5)*V_xx_prev(1, 6) + Fx(2, 5)*V_xx_prev(2, 6) + Fx(3, 5)*V_xx_prev(3, 6) + Fx(5, 5)*V_xx_prev(5, 6);
    const double internal_94 = Fx(5, 6)*V_xx_prev(5, 0) + Fx(6, 6)*V_xx_prev(6, 0);
    const double internal_95 = Fx(5, 6)*V_xx_prev(5, 1) + Fx(6, 6)*V_xx_prev(6, 1);
    const double internal_96 = Fx(5, 6)*V_xx_prev(5, 2) + Fx(6, 6)*V_xx_prev(6, 2);
    const double internal_97 = Fx(5, 6)*V_xx_prev(5, 3) + Fx(6, 6)*V_xx_prev(6, 3);
    const double internal_98 = Fx(5, 6)*V_xx_prev(5, 4) + Fx(6, 6)*V_xx_prev(6, 4);
    const double internal_99 = Fx(5, 6)*V_xx_prev(5, 5) + Fx(6, 6)*V_xx_prev(6, 5);
    const double internal_10 = Fx(1, 1)*V_xx_prev(1, 0) + internal_0;
    const double internal_45 = Fx(2, 2)*V_xx_prev(2, 0) + internal_2;
    const double internal_73 = Fx(1, 3)*V_xx_prev(1, 0) + Fx(2, 3)*V_xx_prev(2, 0) + Fx(3, 3)*V_xx_prev(3, 0) + internal_4;
    const double internal_87 = Fx(1, 5)*V_xx_prev(1, 0) + Fx(2, 5)*V_xx_prev(2, 0) + Fx(3, 5)*V_xx_prev(3, 0) + Fx(5, 5)*V_xx_prev(5, 0) + internal_7;
    const double internal_39 = Sigma_(0)*g_x(0, 1)*g_x(0, 2) + Sigma_(1)*g_x(1, 1)*g_x(1, 2) + g_x(19, 2)*internal_13 + g_x(2, 2)*internal_14 + g_x(20, 2)*internal_15 + g_x(21, 2)*internal_16 + g_x(22, 2)*internal_17 + g_x(23, 2)*internal_18 + g_x(24, 2)*internal_19 + g_x(25, 2)*internal_20 + g_x(26, 2)*internal_21 + g_x(27, 2)*internal_22 + g_x(28, 2)*internal_23 + g_x(29, 2)*internal_24 + g_x(3, 2)*internal_25 + g_x(30, 2)*internal_26 + g_x(31, 2)*internal_27 + g_x(32, 2)*internal_28 + g_x(33, 2)*internal_29 + g_x(34, 2)*internal_30 + g_x(35, 2)*internal_31 + g_x(36, 2)*internal_32 + g_x(37, 2)*internal_33 + g_x(38, 2)*internal_34 + g_x(39, 2)*internal_35 + g_x(4, 2)*internal_36 + g_x(40, 2)*internal_37 + g_x(5, 2)*internal_38;
    const double internal_41 = g_x(19, 3)*internal_13 + g_x(2, 3)*internal_14 + g_x(20, 3)*internal_15 + g_x(21, 3)*internal_16 + g_x(22, 3)*internal_17 + g_x(23, 3)*internal_18 + g_x(24, 3)*internal_19 + g_x(25, 3)*internal_20 + g_x(26, 3)*internal_21 + g_x(27, 3)*internal_22 + g_x(28, 3)*internal_23 + g_x(29, 3)*internal_24 + g_x(3, 3)*internal_25 + g_x(30, 3)*internal_26 + g_x(31, 3)*internal_27 + g_x(32, 3)*internal_28 + g_x(33, 3)*internal_29 + g_x(34, 3)*internal_30 + g_x(37, 3)*internal_33 + g_x(38, 3)*internal_34 + g_x(39, 3)*internal_35 + g_x(4, 3)*internal_36 + g_x(40, 3)*internal_37 + g_x(5, 3)*internal_38;
    const double internal_69 = Sigma_(2)*g_x(2, 2)*g_x(2, 3) + Sigma_(3)*g_x(3, 2)*g_x(3, 3) + Sigma_(4)*g_x(4, 2)*g_x(4, 3) + Sigma_(5)*g_x(5, 2)*g_x(5, 3) + g_x(19, 3)*internal_49 + g_x(20, 3)*internal_50 + g_x(21, 3)*internal_51 + g_x(22, 3)*internal_52 + g_x(23, 3)*internal_53 + g_x(24, 3)*internal_54 + g_x(25, 3)*internal_55 + g_x(26, 3)*internal_56 + g_x(27, 3)*internal_57 + g_x(28, 3)*internal_58 + g_x(29, 3)*internal_59 + g_x(30, 3)*internal_60 + g_x(31, 3)*internal_61 + g_x(32, 3)*internal_62 + g_x(33, 3)*internal_63 + g_x(34, 3)*internal_64 + g_x(37, 3)*internal_65 + g_x(38, 3)*internal_66 + g_x(39, 3)*internal_67 + g_x(40, 3)*internal_68;

    // Evaluation of Matrix Q_xx_
    Q_xx_(0, 0) = pow(Fx(0, 0), 2)*V_xx_prev(0, 0) + Sigma_(41)*pow(g_x(41, 0), 2) + Sigma_(8)*pow(g_x(8, 0), 2) + Sigma_(9)*pow(g_x(9, 0), 2) + l_xx(0, 0);
    Q_xx_(0, 1) = Fx(0, 0)*internal_0 + Fx(1, 1)*internal_1;
    Q_xx_(0, 2) = Fx(0, 0)*internal_2 + Fx(2, 2)*internal_3;
    Q_xx_(0, 3) = Fx(0, 0)*internal_4 + Fx(1, 3)*internal_1 + Fx(2, 3)*internal_3 + Fx(3, 3)*internal_5;
    Q_xx_(0, 4) = Fx(3, 4)*internal_5 + Fx(4, 4)*internal_6;
    Q_xx_(0, 5) = Fx(0, 0)*internal_7 + Fx(1, 5)*internal_1 + Fx(2, 5)*internal_3 + Fx(3, 5)*internal_5 + Fx(5, 5)*internal_8;
    Q_xx_(0, 6) = Fx(5, 6)*internal_8 + Fx(6, 6)*internal_9;
    Q_xx_(1, 0) = Fx(0, 0)*internal_10;
    Q_xx_(1, 1) = Fx(0, 1)*internal_10 + Fx(1, 1)*internal_11 + Sigma_(0)*pow(g_x(0, 1), 2) + Sigma_(1)*pow(g_x(1, 1), 2) + Sigma_(19)*pow(g_x(19, 1), 2) + Sigma_(2)*pow(g_x(2, 1), 2) + Sigma_(20)*pow(g_x(20, 1), 2) + Sigma_(21)*pow(g_x(21, 1), 2) + Sigma_(22)*pow(g_x(22, 1), 2) + Sigma_(23)*pow(g_x(23, 1), 2) + Sigma_(24)*pow(g_x(24, 1), 2) + Sigma_(25)*pow(g_x(25, 1), 2) + Sigma_(26)*pow(g_x(26, 1), 2) + Sigma_(27)*pow(g_x(27, 1), 2) + Sigma_(28)*pow(g_x(28, 1), 2) + Sigma_(29)*pow(g_x(29, 1), 2) + Sigma_(3)*pow(g_x(3, 1), 2) + Sigma_(30)*pow(g_x(30, 1), 2) + Sigma_(31)*pow(g_x(31, 1), 2) + Sigma_(32)*pow(g_x(32, 1), 2) + Sigma_(33)*pow(g_x(33, 1), 2) + Sigma_(34)*pow(g_x(34, 1), 2) + Sigma_(35)*pow(g_x(35, 1), 2) + Sigma_(36)*pow(g_x(36, 1), 2) + Sigma_(37)*pow(g_x(37, 1), 2) + Sigma_(38)*pow(g_x(38, 1), 2) + Sigma_(39)*pow(g_x(39, 1), 2) + Sigma_(4)*pow(g_x(4, 1), 2) + Sigma_(40)*pow(g_x(40, 1), 2) + Sigma_(5)*pow(g_x(5, 1), 2) + l_xx(1, 1);
    Q_xx_(1, 2) = Fx(0, 2)*internal_10 + Fx(2, 2)*internal_12 + internal_39 + l_xx(1, 2);
    Q_xx_(1, 3) = Fx(0, 3)*internal_10 + Fx(1, 3)*internal_11 + Fx(2, 3)*internal_12 + Fx(3, 3)*internal_40 + internal_41;
    Q_xx_(1, 4) = Fx(3, 4)*internal_40 + Fx(4, 4)*internal_42;
    Q_xx_(1, 5) = Fx(0, 5)*internal_10 + Fx(1, 5)*internal_11 + Fx(2, 5)*internal_12 + Fx(3, 5)*internal_40 + Fx(5, 5)*internal_43;
    Q_xx_(1, 6) = Fx(5, 6)*internal_43 + Fx(6, 6)*internal_44;
    Q_xx_(2, 0) = Fx(0, 0)*internal_45;
    Q_xx_(2, 1) = Fx(0, 1)*internal_45 + Fx(1, 1)*internal_46 + internal_39 + l_xx(2, 1);
    Q_xx_(2, 2) = Fx(0, 2)*internal_45 + Fx(2, 2)*internal_47 + Sigma_(0)*pow(g_x(0, 2), 2) + Sigma_(1)*pow(g_x(1, 2), 2) + Sigma_(19)*pow(g_x(19, 2), 2) + Sigma_(2)*pow(g_x(2, 2), 2) + Sigma_(20)*pow(g_x(20, 2), 2) + Sigma_(21)*pow(g_x(21, 2), 2) + Sigma_(22)*pow(g_x(22, 2), 2) + Sigma_(23)*pow(g_x(23, 2), 2) + Sigma_(24)*pow(g_x(24, 2), 2) + Sigma_(25)*pow(g_x(25, 2), 2) + Sigma_(26)*pow(g_x(26, 2), 2) + Sigma_(27)*pow(g_x(27, 2), 2) + Sigma_(28)*pow(g_x(28, 2), 2) + Sigma_(29)*pow(g_x(29, 2), 2) + Sigma_(3)*pow(g_x(3, 2), 2) + Sigma_(30)*pow(g_x(30, 2), 2) + Sigma_(31)*pow(g_x(31, 2), 2) + Sigma_(32)*pow(g_x(32, 2), 2) + Sigma_(33)*pow(g_x(33, 2), 2) + Sigma_(34)*pow(g_x(34, 2), 2) + Sigma_(35)*pow(g_x(35, 2), 2) + Sigma_(36)*pow(g_x(36, 2), 2) + Sigma_(37)*pow(g_x(37, 2), 2) + Sigma_(38)*pow(g_x(38, 2), 2) + Sigma_(39)*pow(g_x(39, 2), 2) + Sigma_(4)*pow(g_x(4, 2), 2) + Sigma_(40)*pow(g_x(40, 2), 2) + Sigma_(5)*pow(g_x(5, 2), 2) + l_xx(2, 2);
    Q_xx_(2, 3) = Fx(0, 3)*internal_45 + Fx(1, 3)*internal_46 + Fx(2, 3)*internal_47 + Fx(3, 3)*internal_48 + internal_69;
    Q_xx_(2, 4) = Fx(3, 4)*internal_48 + Fx(4, 4)*internal_70;
    Q_xx_(2, 5) = Fx(0, 5)*internal_45 + Fx(1, 5)*internal_46 + Fx(2, 5)*internal_47 + Fx(3, 5)*internal_48 + Fx(5, 5)*internal_71;
    Q_xx_(2, 6) = Fx(5, 6)*internal_71 + Fx(6, 6)*internal_72;
    Q_xx_(3, 0) = Fx(0, 0)*internal_73;
    Q_xx_(3, 1) = Fx(0, 1)*internal_73 + Fx(1, 1)*internal_74 + internal_41;
    Q_xx_(3, 2) = Fx(0, 2)*internal_73 + Fx(2, 2)*internal_75 + internal_69;
    Q_xx_(3, 3) = Fx(0, 3)*internal_73 + Fx(1, 3)*internal_74 + Fx(2, 3)*internal_75 + Fx(3, 3)*internal_76 + Sigma_(14)*pow(g_x(14, 3), 2) + Sigma_(19)*pow(g_x(19, 3), 2) + Sigma_(2)*pow(g_x(2, 3), 2) + Sigma_(20)*pow(g_x(20, 3), 2) + Sigma_(21)*pow(g_x(21, 3), 2) + Sigma_(22)*pow(g_x(22, 3), 2) + Sigma_(23)*pow(g_x(23, 3), 2) + Sigma_(24)*pow(g_x(24, 3), 2) + Sigma_(25)*pow(g_x(25, 3), 2) + Sigma_(26)*pow(g_x(26, 3), 2) + Sigma_(27)*pow(g_x(27, 3), 2) + Sigma_(28)*pow(g_x(28, 3), 2) + Sigma_(29)*pow(g_x(29, 3), 2) + Sigma_(3)*pow(g_x(3, 3), 2) + Sigma_(30)*pow(g_x(30, 3), 2) + Sigma_(31)*pow(g_x(31, 3), 2) + Sigma_(32)*pow(g_x(32, 3), 2) + Sigma_(33)*pow(g_x(33, 3), 2) + Sigma_(34)*pow(g_x(34, 3), 2) + Sigma_(37)*pow(g_x(37, 3), 2) + Sigma_(38)*pow(g_x(38, 3), 2) + Sigma_(39)*pow(g_x(39, 3), 2) + Sigma_(4)*pow(g_x(4, 3), 2) + Sigma_(40)*pow(g_x(40, 3), 2) + Sigma_(5)*pow(g_x(5, 3), 2) + l_xx(3, 3);
    Q_xx_(3, 4) = Fx(3, 4)*internal_76 + Fx(4, 4)*internal_77;
    Q_xx_(3, 5) = Fx(0, 5)*internal_73 + Fx(1, 5)*internal_74 + Fx(2, 5)*internal_75 + Fx(3, 5)*internal_76 + Fx(5, 5)*internal_78;
    Q_xx_(3, 6) = Fx(5, 6)*internal_78 + Fx(6, 6)*internal_79;
    Q_xx_(4, 0) = Fx(0, 0)*internal_80;
    Q_xx_(4, 1) = Fx(0, 1)*internal_80 + Fx(1, 1)*internal_81;
    Q_xx_(4, 2) = Fx(0, 2)*internal_80 + Fx(2, 2)*internal_82;
    Q_xx_(4, 3) = Fx(0, 3)*internal_80 + Fx(1, 3)*internal_81 + Fx(2, 3)*internal_82 + Fx(3, 3)*internal_83;
    Q_xx_(4, 4) = Fx(3, 4)*internal_83 + Fx(4, 4)*internal_84 + Sigma_(6)*pow(g_x(6, 4), 2) + Sigma_(7)*pow(g_x(7, 4), 2) + l_xx(4, 4);
    Q_xx_(4, 5) = Fx(0, 5)*internal_80 + Fx(1, 5)*internal_81 + Fx(2, 5)*internal_82 + Fx(3, 5)*internal_83 + Fx(5, 5)*internal_85;
    Q_xx_(4, 6) = Fx(5, 6)*internal_85 + Fx(6, 6)*internal_86;
    Q_xx_(5, 0) = Fx(0, 0)*internal_87;
    Q_xx_(5, 1) = Fx(0, 1)*internal_87 + Fx(1, 1)*internal_88;
    Q_xx_(5, 2) = Fx(0, 2)*internal_87 + Fx(2, 2)*internal_89;
    Q_xx_(5, 3) = Fx(0, 3)*internal_87 + Fx(1, 3)*internal_88 + Fx(2, 3)*internal_89 + Fx(3, 3)*internal_90;
    Q_xx_(5, 4) = Fx(3, 4)*internal_90 + Fx(4, 4)*internal_91;
    Q_xx_(5, 5) = Fx(0, 5)*internal_87 + Fx(1, 5)*internal_88 + Fx(2, 5)*internal_89 + Fx(3, 5)*internal_90 + Fx(5, 5)*internal_92 + Sigma_(10)*pow(g_x(10, 5), 2) + Sigma_(11)*pow(g_x(11, 5), 2) + Sigma_(42)*pow(g_x(42, 5), 2) + l_xx(5, 5);
    Q_xx_(5, 6) = Fx(5, 6)*internal_92 + Fx(6, 6)*internal_93;
    Q_xx_(6, 0) = Fx(0, 0)*internal_94;
    Q_xx_(6, 1) = Fx(0, 1)*internal_94 + Fx(1, 1)*internal_95;
    Q_xx_(6, 2) = Fx(0, 2)*internal_94 + Fx(2, 2)*internal_96;
    Q_xx_(6, 3) = Fx(0, 3)*internal_94 + Fx(1, 3)*internal_95 + Fx(2, 3)*internal_96 + Fx(3, 3)*internal_97;
    Q_xx_(6, 4) = Fx(3, 4)*internal_97 + Fx(4, 4)*internal_98;
    Q_xx_(6, 5) = Fx(0, 5)*internal_94 + Fx(1, 5)*internal_95 + Fx(2, 5)*internal_96 + Fx(3, 5)*internal_97 + Fx(5, 5)*internal_99;
    Q_xx_(6, 6) = Fx(5, 6)*internal_99 + Fx(6, 6)*internal_100 + Sigma_(12)*pow(g_x(12, 6), 2) + Sigma_(13)*pow(g_x(13, 6), 2) + l_xx(6, 6);

    // Evaluation of Matrix Q_xu_
    Q_xu_(0, 0) = Fu(4, 0)*internal_6;
    Q_xu_(0, 1) = Fu(6, 1)*internal_9;
    Q_xu_(0, 3) = Sigma_(41)*g_u(41, 3)*g_x(41, 0);
    Q_xu_(1, 0) = Fu(4, 0)*internal_42;
    Q_xu_(1, 1) = Fu(6, 1)*internal_44;
    Q_xu_(1, 2) = g_u(35, 2)*internal_31 + g_u(36, 2)*internal_32 + g_u(37, 2)*internal_33 + g_u(38, 2)*internal_34 + g_u(39, 2)*internal_35 + g_u(40, 2)*internal_37;
    Q_xu_(1, 5) = g_u(19, 5)*internal_13 + g_u(27, 5)*internal_22;
    Q_xu_(1, 6) = g_u(20, 6)*internal_15 + g_u(28, 6)*internal_23;
    Q_xu_(1, 7) = g_u(21, 7)*internal_16 + g_u(29, 7)*internal_24;
    Q_xu_(1, 8) = g_u(22, 8)*internal_17 + g_u(30, 8)*internal_26;
    Q_xu_(1, 9) = g_u(23, 9)*internal_18 + g_u(31, 9)*internal_27;
    Q_xu_(1, 10) = g_u(24, 10)*internal_19 + g_u(32, 10)*internal_28;
    Q_xu_(1, 11) = g_u(25, 11)*internal_20 + g_u(33, 11)*internal_29;
    Q_xu_(1, 12) = g_u(26, 12)*internal_21 + g_u(34, 12)*internal_30;
    Q_xu_(2, 0) = Fu(4, 0)*internal_70;
    Q_xu_(2, 1) = Fu(6, 1)*internal_72;
    Q_xu_(2, 2) = Sigma_(35)*g_u(35, 2)*g_x(35, 2) + Sigma_(36)*g_u(36, 2)*g_x(36, 2) + g_u(37, 2)*internal_65 + g_u(38, 2)*internal_66 + g_u(39, 2)*internal_67 + g_u(40, 2)*internal_68;
    Q_xu_(2, 5) = g_u(19, 5)*internal_49 + g_u(27, 5)*internal_57;
    Q_xu_(2, 6) = g_u(20, 6)*internal_50 + g_u(28, 6)*internal_58;
    Q_xu_(2, 7) = g_u(21, 7)*internal_51 + g_u(29, 7)*internal_59;
    Q_xu_(2, 8) = g_u(22, 8)*internal_52 + g_u(30, 8)*internal_60;
    Q_xu_(2, 9) = g_u(23, 9)*internal_53 + g_u(31, 9)*internal_61;
    Q_xu_(2, 10) = g_u(24, 10)*internal_54 + g_u(32, 10)*internal_62;
    Q_xu_(2, 11) = g_u(25, 11)*internal_55 + g_u(33, 11)*internal_63;
    Q_xu_(2, 12) = g_u(26, 12)*internal_56 + g_u(34, 12)*internal_64;
    Q_xu_(3, 0) = Fu(4, 0)*internal_77;
    Q_xu_(3, 1) = Fu(6, 1)*internal_79;
    Q_xu_(3, 2) = Sigma_(37)*g_u(37, 2)*g_x(37, 3) + Sigma_(38)*g_u(38, 2)*g_x(38, 3) + Sigma_(39)*g_u(39, 2)*g_x(39, 3) + Sigma_(40)*g_u(40, 2)*g_x(40, 3);
    Q_xu_(3, 5) = Sigma_(19)*g_u(19, 5)*g_x(19, 3) + Sigma_(27)*g_u(27, 5)*g_x(27, 3);
    Q_xu_(3, 6) = Sigma_(20)*g_u(20, 6)*g_x(20, 3) + Sigma_(28)*g_u(28, 6)*g_x(28, 3);
    Q_xu_(3, 7) = Sigma_(21)*g_u(21, 7)*g_x(21, 3) + Sigma_(29)*g_u(29, 7)*g_x(29, 3);
    Q_xu_(3, 8) = Sigma_(22)*g_u(22, 8)*g_x(22, 3) + Sigma_(30)*g_u(30, 8)*g_x(30, 3);
    Q_xu_(3, 9) = Sigma_(23)*g_u(23, 9)*g_x(23, 3) + Sigma_(31)*g_u(31, 9)*g_x(31, 3);
    Q_xu_(3, 10) = Sigma_(24)*g_u(24, 10)*g_x(24, 3) + Sigma_(32)*g_u(32, 10)*g_x(32, 3);
    Q_xu_(3, 11) = Sigma_(25)*g_u(25, 11)*g_x(25, 3) + Sigma_(33)*g_u(33, 11)*g_x(33, 3);
    Q_xu_(3, 12) = Sigma_(26)*g_u(26, 12)*g_x(26, 3) + Sigma_(34)*g_u(34, 12)*g_x(34, 3);
    Q_xu_(4, 0) = Fu(4, 0)*internal_84;
    Q_xu_(4, 1) = Fu(6, 1)*internal_86;
    Q_xu_(5, 0) = Fu(4, 0)*internal_91;
    Q_xu_(5, 1) = Fu(6, 1)*internal_93;
    Q_xu_(5, 4) = Sigma_(42)*g_u(42, 4)*g_x(42, 5);
    Q_xu_(6, 0) = Fu(4, 0)*internal_98;
    Q_xu_(6, 1) = Fu(6, 1)*internal_100;

    // Evaluation of Matrix Q_uu_
    Q_uu_(0, 0) = pow(Fu(4, 0), 2)*V_xx_prev(4, 4) + Sigma_(15)*pow(g_u(15, 0), 2) + Sigma_(16)*pow(g_u(16, 0), 2) + l_uu(0, 0);
    Q_uu_(0, 1) = V_xx_prev(4, 6)*internal_101;
    Q_uu_(1, 0) = V_xx_prev(6, 4)*internal_101;
    Q_uu_(1, 1) = pow(Fu(6, 1), 2)*V_xx_prev(6, 6) + Sigma_(17)*pow(g_u(17, 1), 2) + Sigma_(18)*pow(g_u(18, 1), 2) + l_uu(1, 1);
    Q_uu_(2, 2) = Sigma_(35)*pow(g_u(35, 2), 2) + Sigma_(36)*pow(g_u(36, 2), 2) + Sigma_(37)*pow(g_u(37, 2), 2) + Sigma_(38)*pow(g_u(38, 2), 2) + Sigma_(39)*pow(g_u(39, 2), 2) + Sigma_(40)*pow(g_u(40, 2), 2) + l_uu(2, 2);
    Q_uu_(3, 3) = Sigma_(41)*pow(g_u(41, 3), 2) + l_uu(3, 3);
    Q_uu_(4, 4) = Sigma_(42)*pow(g_u(42, 4), 2) + l_uu(4, 4);
    Q_uu_(5, 5) = Sigma_(19)*pow(g_u(19, 5), 2) + Sigma_(27)*pow(g_u(27, 5), 2) + l_uu(5, 5);
    Q_uu_(6, 6) = Sigma_(20)*pow(g_u(20, 6), 2) + Sigma_(28)*pow(g_u(28, 6), 2) + l_uu(6, 6);
    Q_uu_(7, 7) = Sigma_(21)*pow(g_u(21, 7), 2) + Sigma_(29)*pow(g_u(29, 7), 2) + l_uu(7, 7);
    Q_uu_(8, 8) = Sigma_(22)*pow(g_u(22, 8), 2) + Sigma_(30)*pow(g_u(30, 8), 2) + l_uu(8, 8);
    Q_uu_(9, 9) = Sigma_(23)*pow(g_u(23, 9), 2) + Sigma_(31)*pow(g_u(31, 9), 2) + l_uu(9, 9);
    Q_uu_(10, 10) = Sigma_(24)*pow(g_u(24, 10), 2) + Sigma_(32)*pow(g_u(32, 10), 2) + l_uu(10, 10);
    Q_uu_(11, 11) = Sigma_(25)*pow(g_u(25, 11), 2) + Sigma_(33)*pow(g_u(33, 11), 2) + l_uu(11, 11);
    Q_uu_(12, 12) = Sigma_(26)*pow(g_u(26, 12), 2) + Sigma_(34)*pow(g_u(34, 12), 2) + l_uu(12, 12);

  }

  virtual void updateQG(const Eigen::VectorXd& sl_x, const Eigen::VectorXd& sl_u, const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::VectorXd& r_f, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& V_xx_prev, const Eigen::VectorXd& V_x_prev) override {
    // Determine internal variables
    const double internal_0 = V_x_prev(0) + V_xx_prev(0, 0)*r_f(0) + V_xx_prev(0, 1)*r_f(1) + V_xx_prev(0, 2)*r_f(2) + V_xx_prev(0, 3)*r_f(3) + V_xx_prev(0, 4)*r_f(4) + V_xx_prev(0, 5)*r_f(5) + V_xx_prev(0, 6)*r_f(6);
    const double internal_1 = V_x_prev(1) + V_xx_prev(1, 0)*r_f(0) + V_xx_prev(1, 1)*r_f(1) + V_xx_prev(1, 2)*r_f(2) + V_xx_prev(1, 3)*r_f(3) + V_xx_prev(1, 4)*r_f(4) + V_xx_prev(1, 5)*r_f(5) + V_xx_prev(1, 6)*r_f(6);
    const double internal_2 = V_x_prev(2) + V_xx_prev(2, 0)*r_f(0) + V_xx_prev(2, 1)*r_f(1) + V_xx_prev(2, 2)*r_f(2) + V_xx_prev(2, 3)*r_f(3) + V_xx_prev(2, 4)*r_f(4) + V_xx_prev(2, 5)*r_f(5) + V_xx_prev(2, 6)*r_f(6);
    const double internal_3 = V_x_prev(3) + V_xx_prev(3, 0)*r_f(0) + V_xx_prev(3, 1)*r_f(1) + V_xx_prev(3, 2)*r_f(2) + V_xx_prev(3, 3)*r_f(3) + V_xx_prev(3, 4)*r_f(4) + V_xx_prev(3, 5)*r_f(5) + V_xx_prev(3, 6)*r_f(6);
    const double internal_4 = V_x_prev(4) + V_xx_prev(4, 0)*r_f(0) + V_xx_prev(4, 1)*r_f(1) + V_xx_prev(4, 2)*r_f(2) + V_xx_prev(4, 3)*r_f(3) + V_xx_prev(4, 4)*r_f(4) + V_xx_prev(4, 5)*r_f(5) + V_xx_prev(4, 6)*r_f(6);
    const double internal_5 = V_x_prev(5) + V_xx_prev(5, 0)*r_f(0) + V_xx_prev(5, 1)*r_f(1) + V_xx_prev(5, 2)*r_f(2) + V_xx_prev(5, 3)*r_f(3) + V_xx_prev(5, 4)*r_f(4) + V_xx_prev(5, 5)*r_f(5) + V_xx_prev(5, 6)*r_f(6);
    const double internal_6 = V_x_prev(6) + V_xx_prev(6, 0)*r_f(0) + V_xx_prev(6, 1)*r_f(1) + V_xx_prev(6, 2)*r_f(2) + V_xx_prev(6, 3)*r_f(3) + V_xx_prev(6, 4)*r_f(4) + V_xx_prev(6, 5)*r_f(5) + V_xx_prev(6, 6)*r_f(6);

    // Evaluation of Vector Q_x_
    Q_x_(0) = Fx(0, 0)*internal_0 + V_(41)*g_x(41, 0) + V_(8)*g_x(8, 0) + V_(9)*g_x(9, 0) + sl_x(0);
    Q_x_(1) = Fx(0, 1)*internal_0 + Fx(1, 1)*internal_1 + V_(0)*g_x(0, 1) + V_(1)*g_x(1, 1) + V_(19)*g_x(19, 1) + V_(2)*g_x(2, 1) + V_(20)*g_x(20, 1) + V_(21)*g_x(21, 1) + V_(22)*g_x(22, 1) + V_(23)*g_x(23, 1) + V_(24)*g_x(24, 1) + V_(25)*g_x(25, 1) + V_(26)*g_x(26, 1) + V_(27)*g_x(27, 1) + V_(28)*g_x(28, 1) + V_(29)*g_x(29, 1) + V_(3)*g_x(3, 1) + V_(30)*g_x(30, 1) + V_(31)*g_x(31, 1) + V_(32)*g_x(32, 1) + V_(33)*g_x(33, 1) + V_(34)*g_x(34, 1) + V_(35)*g_x(35, 1) + V_(36)*g_x(36, 1) + V_(37)*g_x(37, 1) + V_(38)*g_x(38, 1) + V_(39)*g_x(39, 1) + V_(4)*g_x(4, 1) + V_(40)*g_x(40, 1) + V_(5)*g_x(5, 1) + sl_x(1);
    Q_x_(2) = Fx(0, 2)*internal_0 + Fx(2, 2)*internal_2 + V_(0)*g_x(0, 2) + V_(1)*g_x(1, 2) + V_(19)*g_x(19, 2) + V_(2)*g_x(2, 2) + V_(20)*g_x(20, 2) + V_(21)*g_x(21, 2) + V_(22)*g_x(22, 2) + V_(23)*g_x(23, 2) + V_(24)*g_x(24, 2) + V_(25)*g_x(25, 2) + V_(26)*g_x(26, 2) + V_(27)*g_x(27, 2) + V_(28)*g_x(28, 2) + V_(29)*g_x(29, 2) + V_(3)*g_x(3, 2) + V_(30)*g_x(30, 2) + V_(31)*g_x(31, 2) + V_(32)*g_x(32, 2) + V_(33)*g_x(33, 2) + V_(34)*g_x(34, 2) + V_(35)*g_x(35, 2) + V_(36)*g_x(36, 2) + V_(37)*g_x(37, 2) + V_(38)*g_x(38, 2) + V_(39)*g_x(39, 2) + V_(4)*g_x(4, 2) + V_(40)*g_x(40, 2) + V_(5)*g_x(5, 2) + sl_x(2);
    Q_x_(3) = Fx(0, 3)*internal_0 + Fx(1, 3)*internal_1 + Fx(2, 3)*internal_2 + Fx(3, 3)*internal_3 + V_(14)*g_x(14, 3) + V_(19)*g_x(19, 3) + V_(2)*g_x(2, 3) + V_(20)*g_x(20, 3) + V_(21)*g_x(21, 3) + V_(22)*g_x(22, 3) + V_(23)*g_x(23, 3) + V_(24)*g_x(24, 3) + V_(25)*g_x(25, 3) + V_(26)*g_x(26, 3) + V_(27)*g_x(27, 3) + V_(28)*g_x(28, 3) + V_(29)*g_x(29, 3) + V_(3)*g_x(3, 3) + V_(30)*g_x(30, 3) + V_(31)*g_x(31, 3) + V_(32)*g_x(32, 3) + V_(33)*g_x(33, 3) + V_(34)*g_x(34, 3) + V_(37)*g_x(37, 3) + V_(38)*g_x(38, 3) + V_(39)*g_x(39, 3) + V_(4)*g_x(4, 3) + V_(40)*g_x(40, 3) + V_(5)*g_x(5, 3) + sl_x(3);
    Q_x_(4) = Fx(3, 4)*internal_3 + Fx(4, 4)*internal_4 + V_(6)*g_x(6, 4) + V_(7)*g_x(7, 4) + sl_x(4);
    Q_x_(5) = Fx(0, 5)*internal_0 + Fx(1, 5)*internal_1 + Fx(2, 5)*internal_2 + Fx(3, 5)*internal_3 + Fx(5, 5)*internal_5 + V_(10)*g_x(10, 5) + V_(11)*g_x(11, 5) + V_(42)*g_x(42, 5) + sl_x(5);
    Q_x_(6) = Fx(5, 6)*internal_5 + Fx(6, 6)*internal_6 + V_(12)*g_x(12, 6) + V_(13)*g_x(13, 6) + sl_x(6);

    // Evaluation of Vector Q_u_
    Q_u_(0) = Fu(4, 0)*internal_4 + V_(15)*g_u(15, 0) + V_(16)*g_u(16, 0) + sl_u(0);
    Q_u_(1) = Fu(6, 1)*internal_6 + V_(17)*g_u(17, 1) + V_(18)*g_u(18, 1) + sl_u(1);
    Q_u_(2) = V_(35)*g_u(35, 2) + V_(36)*g_u(36, 2) + V_(37)*g_u(37, 2) + V_(38)*g_u(38, 2) + V_(39)*g_u(39, 2) + V_(40)*g_u(40, 2) + sl_u(2);
    Q_u_(3) = V_(41)*g_u(41, 3) + sl_u(3);
    Q_u_(4) = V_(42)*g_u(42, 4) + sl_u(4);
    Q_u_(5) = V_(19)*g_u(19, 5) + V_(27)*g_u(27, 5) + sl_u(5);
    Q_u_(6) = V_(20)*g_u(20, 6) + V_(28)*g_u(28, 6) + sl_u(6);
    Q_u_(7) = V_(21)*g_u(21, 7) + V_(29)*g_u(29, 7) + sl_u(7);
    Q_u_(8) = V_(22)*g_u(22, 8) + V_(30)*g_u(30, 8) + sl_u(8);
    Q_u_(9) = V_(23)*g_u(23, 9) + V_(31)*g_u(31, 9) + sl_u(9);
    Q_u_(10) = V_(24)*g_u(24, 10) + V_(32)*g_u(32, 10) + sl_u(10);
    Q_u_(11) = V_(25)*g_u(25, 11) + V_(33)*g_u(33, 11) + sl_u(11);
    Q_u_(12) = V_(26)*g_u(26, 12) + V_(34)*g_u(34, 12) + sl_u(12);

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
    Q_uu_inv_(3, 3) = 1.0/Q_uu_(3, 3);
    Q_uu_inv_(4, 4) = 1.0/Q_uu_(4, 4);
    Q_uu_inv_(5, 5) = 1.0/Q_uu_(5, 5);
    Q_uu_inv_(6, 6) = 1.0/Q_uu_(6, 6);
    Q_uu_inv_(7, 7) = 1.0/Q_uu_(7, 7);
    Q_uu_inv_(8, 8) = 1.0/Q_uu_(8, 8);
    Q_uu_inv_(9, 9) = 1.0/Q_uu_(9, 9);
    Q_uu_inv_(10, 10) = 1.0/Q_uu_(10, 10);
    Q_uu_inv_(11, 11) = 1.0/Q_uu_(11, 11);
    Q_uu_inv_(12, 12) = 1.0/Q_uu_(12, 12);

  }

  virtual void updateKVU(Eigen::MatrixXd& k_ux, Eigen::VectorXd& v_u) override {
    // Evaluation of Matrix k_ux
    k_ux(0, 0) = -Q_uu_inv_(0, 0)*Q_xu_(0, 0) - Q_uu_inv_(0, 1)*Q_xu_(0, 1);
    k_ux(0, 1) = -Q_uu_inv_(0, 0)*Q_xu_(1, 0) - Q_uu_inv_(0, 1)*Q_xu_(1, 1);
    k_ux(0, 2) = -Q_uu_inv_(0, 0)*Q_xu_(2, 0) - Q_uu_inv_(0, 1)*Q_xu_(2, 1);
    k_ux(0, 3) = -Q_uu_inv_(0, 0)*Q_xu_(3, 0) - Q_uu_inv_(0, 1)*Q_xu_(3, 1);
    k_ux(0, 4) = -Q_uu_inv_(0, 0)*Q_xu_(4, 0) - Q_uu_inv_(0, 1)*Q_xu_(4, 1);
    k_ux(0, 5) = -Q_uu_inv_(0, 0)*Q_xu_(5, 0) - Q_uu_inv_(0, 1)*Q_xu_(5, 1);
    k_ux(0, 6) = -Q_uu_inv_(0, 0)*Q_xu_(6, 0) - Q_uu_inv_(0, 1)*Q_xu_(6, 1);
    k_ux(1, 0) = -Q_uu_inv_(0, 1)*Q_xu_(0, 0) - Q_uu_inv_(1, 1)*Q_xu_(0, 1);
    k_ux(1, 1) = -Q_uu_inv_(0, 1)*Q_xu_(1, 0) - Q_uu_inv_(1, 1)*Q_xu_(1, 1);
    k_ux(1, 2) = -Q_uu_inv_(0, 1)*Q_xu_(2, 0) - Q_uu_inv_(1, 1)*Q_xu_(2, 1);
    k_ux(1, 3) = -Q_uu_inv_(0, 1)*Q_xu_(3, 0) - Q_uu_inv_(1, 1)*Q_xu_(3, 1);
    k_ux(1, 4) = -Q_uu_inv_(0, 1)*Q_xu_(4, 0) - Q_uu_inv_(1, 1)*Q_xu_(4, 1);
    k_ux(1, 5) = -Q_uu_inv_(0, 1)*Q_xu_(5, 0) - Q_uu_inv_(1, 1)*Q_xu_(5, 1);
    k_ux(1, 6) = -Q_uu_inv_(0, 1)*Q_xu_(6, 0) - Q_uu_inv_(1, 1)*Q_xu_(6, 1);
    k_ux(2, 1) = -Q_uu_inv_(2, 2)*Q_xu_(1, 2);
    k_ux(2, 2) = -Q_uu_inv_(2, 2)*Q_xu_(2, 2);
    k_ux(2, 3) = -Q_uu_inv_(2, 2)*Q_xu_(3, 2);
    k_ux(3, 0) = -Q_uu_inv_(3, 3)*Q_xu_(0, 3);
    k_ux(4, 5) = -Q_uu_inv_(4, 4)*Q_xu_(5, 4);
    k_ux(5, 1) = -Q_uu_inv_(5, 5)*Q_xu_(1, 5);
    k_ux(5, 2) = -Q_uu_inv_(5, 5)*Q_xu_(2, 5);
    k_ux(5, 3) = -Q_uu_inv_(5, 5)*Q_xu_(3, 5);
    k_ux(6, 1) = -Q_uu_inv_(6, 6)*Q_xu_(1, 6);
    k_ux(6, 2) = -Q_uu_inv_(6, 6)*Q_xu_(2, 6);
    k_ux(6, 3) = -Q_uu_inv_(6, 6)*Q_xu_(3, 6);
    k_ux(7, 1) = -Q_uu_inv_(7, 7)*Q_xu_(1, 7);
    k_ux(7, 2) = -Q_uu_inv_(7, 7)*Q_xu_(2, 7);
    k_ux(7, 3) = -Q_uu_inv_(7, 7)*Q_xu_(3, 7);
    k_ux(8, 1) = -Q_uu_inv_(8, 8)*Q_xu_(1, 8);
    k_ux(8, 2) = -Q_uu_inv_(8, 8)*Q_xu_(2, 8);
    k_ux(8, 3) = -Q_uu_inv_(8, 8)*Q_xu_(3, 8);
    k_ux(9, 1) = -Q_uu_inv_(9, 9)*Q_xu_(1, 9);
    k_ux(9, 2) = -Q_uu_inv_(9, 9)*Q_xu_(2, 9);
    k_ux(9, 3) = -Q_uu_inv_(9, 9)*Q_xu_(3, 9);
    k_ux(10, 1) = -Q_uu_inv_(10, 10)*Q_xu_(1, 10);
    k_ux(10, 2) = -Q_uu_inv_(10, 10)*Q_xu_(2, 10);
    k_ux(10, 3) = -Q_uu_inv_(10, 10)*Q_xu_(3, 10);
    k_ux(11, 1) = -Q_uu_inv_(11, 11)*Q_xu_(1, 11);
    k_ux(11, 2) = -Q_uu_inv_(11, 11)*Q_xu_(2, 11);
    k_ux(11, 3) = -Q_uu_inv_(11, 11)*Q_xu_(3, 11);
    k_ux(12, 1) = -Q_uu_inv_(12, 12)*Q_xu_(1, 12);
    k_ux(12, 2) = -Q_uu_inv_(12, 12)*Q_xu_(2, 12);
    k_ux(12, 3) = -Q_uu_inv_(12, 12)*Q_xu_(3, 12);

    // Evaluation of Vector v_u
    v_u(0) = -Q_u_(0)*Q_uu_inv_(0, 0) - Q_u_(1)*Q_uu_inv_(0, 1);
    v_u(1) = -Q_u_(0)*Q_uu_inv_(0, 1) - Q_u_(1)*Q_uu_inv_(1, 1);
    v_u(2) = -Q_u_(2)*Q_uu_inv_(2, 2);
    v_u(3) = -Q_u_(3)*Q_uu_inv_(3, 3);
    v_u(4) = -Q_u_(4)*Q_uu_inv_(4, 4);
    v_u(5) = -Q_u_(5)*Q_uu_inv_(5, 5);
    v_u(6) = -Q_u_(6)*Q_uu_inv_(6, 6);
    v_u(7) = -Q_u_(7)*Q_uu_inv_(7, 7);
    v_u(8) = -Q_u_(8)*Q_uu_inv_(8, 8);
    v_u(9) = -Q_u_(9)*Q_uu_inv_(9, 9);
    v_u(10) = -Q_u_(10)*Q_uu_inv_(10, 10);
    v_u(11) = -Q_u_(11)*Q_uu_inv_(11, 11);
    v_u(12) = -Q_u_(12)*Q_uu_inv_(12, 12);

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
    V_xx(0, 0) = Q_xu_(0, 0)*k_ux(0, 0) + Q_xu_(0, 1)*k_ux(1, 0) + Q_xu_(0, 3)*k_ux(3, 0) + Q_xx_(0, 0);
    V_xx(0, 1) = Q_xu_(0, 0)*k_ux(0, 1) + Q_xu_(0, 1)*k_ux(1, 1) + Q_xx_(0, 1);
    V_xx(0, 2) = Q_xu_(0, 0)*k_ux(0, 2) + Q_xu_(0, 1)*k_ux(1, 2) + Q_xx_(0, 2);
    V_xx(0, 3) = Q_xu_(0, 0)*k_ux(0, 3) + Q_xu_(0, 1)*k_ux(1, 3) + Q_xx_(0, 3);
    V_xx(0, 4) = Q_xu_(0, 0)*k_ux(0, 4) + Q_xu_(0, 1)*k_ux(1, 4) + Q_xx_(0, 4);
    V_xx(0, 5) = Q_xu_(0, 0)*k_ux(0, 5) + Q_xu_(0, 1)*k_ux(1, 5) + Q_xx_(0, 5);
    V_xx(0, 6) = Q_xu_(0, 0)*k_ux(0, 6) + Q_xu_(0, 1)*k_ux(1, 6) + Q_xx_(0, 6);
    V_xx(1, 0) = Q_xu_(1, 0)*k_ux(0, 0) + Q_xu_(1, 1)*k_ux(1, 0) + Q_xx_(0, 1);
    V_xx(1, 1) = Q_xu_(1, 0)*k_ux(0, 1) + Q_xu_(1, 1)*k_ux(1, 1) + Q_xu_(1, 10)*k_ux(10, 1) + Q_xu_(1, 11)*k_ux(11, 1) + Q_xu_(1, 12)*k_ux(12, 1) + Q_xu_(1, 2)*k_ux(2, 1) + Q_xu_(1, 5)*k_ux(5, 1) + Q_xu_(1, 6)*k_ux(6, 1) + Q_xu_(1, 7)*k_ux(7, 1) + Q_xu_(1, 8)*k_ux(8, 1) + Q_xu_(1, 9)*k_ux(9, 1) + Q_xx_(1, 1);
    V_xx(1, 2) = Q_xu_(1, 0)*k_ux(0, 2) + Q_xu_(1, 1)*k_ux(1, 2) + Q_xu_(1, 10)*k_ux(10, 2) + Q_xu_(1, 11)*k_ux(11, 2) + Q_xu_(1, 12)*k_ux(12, 2) + Q_xu_(1, 2)*k_ux(2, 2) + Q_xu_(1, 5)*k_ux(5, 2) + Q_xu_(1, 6)*k_ux(6, 2) + Q_xu_(1, 7)*k_ux(7, 2) + Q_xu_(1, 8)*k_ux(8, 2) + Q_xu_(1, 9)*k_ux(9, 2) + Q_xx_(1, 2);
    V_xx(1, 3) = Q_xu_(1, 0)*k_ux(0, 3) + Q_xu_(1, 1)*k_ux(1, 3) + Q_xu_(1, 10)*k_ux(10, 3) + Q_xu_(1, 11)*k_ux(11, 3) + Q_xu_(1, 12)*k_ux(12, 3) + Q_xu_(1, 2)*k_ux(2, 3) + Q_xu_(1, 5)*k_ux(5, 3) + Q_xu_(1, 6)*k_ux(6, 3) + Q_xu_(1, 7)*k_ux(7, 3) + Q_xu_(1, 8)*k_ux(8, 3) + Q_xu_(1, 9)*k_ux(9, 3) + Q_xx_(1, 3);
    V_xx(1, 4) = Q_xu_(1, 0)*k_ux(0, 4) + Q_xu_(1, 1)*k_ux(1, 4) + Q_xx_(1, 4);
    V_xx(1, 5) = Q_xu_(1, 0)*k_ux(0, 5) + Q_xu_(1, 1)*k_ux(1, 5) + Q_xx_(1, 5);
    V_xx(1, 6) = Q_xu_(1, 0)*k_ux(0, 6) + Q_xu_(1, 1)*k_ux(1, 6) + Q_xx_(1, 6);
    V_xx(2, 0) = Q_xu_(2, 0)*k_ux(0, 0) + Q_xu_(2, 1)*k_ux(1, 0) + Q_xx_(0, 2);
    V_xx(2, 1) = Q_xu_(2, 0)*k_ux(0, 1) + Q_xu_(2, 1)*k_ux(1, 1) + Q_xu_(2, 10)*k_ux(10, 1) + Q_xu_(2, 11)*k_ux(11, 1) + Q_xu_(2, 12)*k_ux(12, 1) + Q_xu_(2, 2)*k_ux(2, 1) + Q_xu_(2, 5)*k_ux(5, 1) + Q_xu_(2, 6)*k_ux(6, 1) + Q_xu_(2, 7)*k_ux(7, 1) + Q_xu_(2, 8)*k_ux(8, 1) + Q_xu_(2, 9)*k_ux(9, 1) + Q_xx_(1, 2);
    V_xx(2, 2) = Q_xu_(2, 0)*k_ux(0, 2) + Q_xu_(2, 1)*k_ux(1, 2) + Q_xu_(2, 10)*k_ux(10, 2) + Q_xu_(2, 11)*k_ux(11, 2) + Q_xu_(2, 12)*k_ux(12, 2) + Q_xu_(2, 2)*k_ux(2, 2) + Q_xu_(2, 5)*k_ux(5, 2) + Q_xu_(2, 6)*k_ux(6, 2) + Q_xu_(2, 7)*k_ux(7, 2) + Q_xu_(2, 8)*k_ux(8, 2) + Q_xu_(2, 9)*k_ux(9, 2) + Q_xx_(2, 2);
    V_xx(2, 3) = Q_xu_(2, 0)*k_ux(0, 3) + Q_xu_(2, 1)*k_ux(1, 3) + Q_xu_(2, 10)*k_ux(10, 3) + Q_xu_(2, 11)*k_ux(11, 3) + Q_xu_(2, 12)*k_ux(12, 3) + Q_xu_(2, 2)*k_ux(2, 3) + Q_xu_(2, 5)*k_ux(5, 3) + Q_xu_(2, 6)*k_ux(6, 3) + Q_xu_(2, 7)*k_ux(7, 3) + Q_xu_(2, 8)*k_ux(8, 3) + Q_xu_(2, 9)*k_ux(9, 3) + Q_xx_(2, 3);
    V_xx(2, 4) = Q_xu_(2, 0)*k_ux(0, 4) + Q_xu_(2, 1)*k_ux(1, 4) + Q_xx_(2, 4);
    V_xx(2, 5) = Q_xu_(2, 0)*k_ux(0, 5) + Q_xu_(2, 1)*k_ux(1, 5) + Q_xx_(2, 5);
    V_xx(2, 6) = Q_xu_(2, 0)*k_ux(0, 6) + Q_xu_(2, 1)*k_ux(1, 6) + Q_xx_(2, 6);
    V_xx(3, 0) = Q_xu_(3, 0)*k_ux(0, 0) + Q_xu_(3, 1)*k_ux(1, 0) + Q_xx_(0, 3);
    V_xx(3, 1) = Q_xu_(3, 0)*k_ux(0, 1) + Q_xu_(3, 1)*k_ux(1, 1) + Q_xu_(3, 10)*k_ux(10, 1) + Q_xu_(3, 11)*k_ux(11, 1) + Q_xu_(3, 12)*k_ux(12, 1) + Q_xu_(3, 2)*k_ux(2, 1) + Q_xu_(3, 5)*k_ux(5, 1) + Q_xu_(3, 6)*k_ux(6, 1) + Q_xu_(3, 7)*k_ux(7, 1) + Q_xu_(3, 8)*k_ux(8, 1) + Q_xu_(3, 9)*k_ux(9, 1) + Q_xx_(1, 3);
    V_xx(3, 2) = Q_xu_(3, 0)*k_ux(0, 2) + Q_xu_(3, 1)*k_ux(1, 2) + Q_xu_(3, 10)*k_ux(10, 2) + Q_xu_(3, 11)*k_ux(11, 2) + Q_xu_(3, 12)*k_ux(12, 2) + Q_xu_(3, 2)*k_ux(2, 2) + Q_xu_(3, 5)*k_ux(5, 2) + Q_xu_(3, 6)*k_ux(6, 2) + Q_xu_(3, 7)*k_ux(7, 2) + Q_xu_(3, 8)*k_ux(8, 2) + Q_xu_(3, 9)*k_ux(9, 2) + Q_xx_(2, 3);
    V_xx(3, 3) = Q_xu_(3, 0)*k_ux(0, 3) + Q_xu_(3, 1)*k_ux(1, 3) + Q_xu_(3, 10)*k_ux(10, 3) + Q_xu_(3, 11)*k_ux(11, 3) + Q_xu_(3, 12)*k_ux(12, 3) + Q_xu_(3, 2)*k_ux(2, 3) + Q_xu_(3, 5)*k_ux(5, 3) + Q_xu_(3, 6)*k_ux(6, 3) + Q_xu_(3, 7)*k_ux(7, 3) + Q_xu_(3, 8)*k_ux(8, 3) + Q_xu_(3, 9)*k_ux(9, 3) + Q_xx_(3, 3);
    V_xx(3, 4) = Q_xu_(3, 0)*k_ux(0, 4) + Q_xu_(3, 1)*k_ux(1, 4) + Q_xx_(3, 4);
    V_xx(3, 5) = Q_xu_(3, 0)*k_ux(0, 5) + Q_xu_(3, 1)*k_ux(1, 5) + Q_xx_(3, 5);
    V_xx(3, 6) = Q_xu_(3, 0)*k_ux(0, 6) + Q_xu_(3, 1)*k_ux(1, 6) + Q_xx_(3, 6);
    V_xx(4, 0) = Q_xu_(4, 0)*k_ux(0, 0) + Q_xu_(4, 1)*k_ux(1, 0) + Q_xx_(0, 4);
    V_xx(4, 1) = Q_xu_(4, 0)*k_ux(0, 1) + Q_xu_(4, 1)*k_ux(1, 1) + Q_xx_(1, 4);
    V_xx(4, 2) = Q_xu_(4, 0)*k_ux(0, 2) + Q_xu_(4, 1)*k_ux(1, 2) + Q_xx_(2, 4);
    V_xx(4, 3) = Q_xu_(4, 0)*k_ux(0, 3) + Q_xu_(4, 1)*k_ux(1, 3) + Q_xx_(3, 4);
    V_xx(4, 4) = Q_xu_(4, 0)*k_ux(0, 4) + Q_xu_(4, 1)*k_ux(1, 4) + Q_xx_(4, 4);
    V_xx(4, 5) = Q_xu_(4, 0)*k_ux(0, 5) + Q_xu_(4, 1)*k_ux(1, 5) + Q_xx_(4, 5);
    V_xx(4, 6) = Q_xu_(4, 0)*k_ux(0, 6) + Q_xu_(4, 1)*k_ux(1, 6) + Q_xx_(4, 6);
    V_xx(5, 0) = Q_xu_(5, 0)*k_ux(0, 0) + Q_xu_(5, 1)*k_ux(1, 0) + Q_xx_(0, 5);
    V_xx(5, 1) = Q_xu_(5, 0)*k_ux(0, 1) + Q_xu_(5, 1)*k_ux(1, 1) + Q_xx_(1, 5);
    V_xx(5, 2) = Q_xu_(5, 0)*k_ux(0, 2) + Q_xu_(5, 1)*k_ux(1, 2) + Q_xx_(2, 5);
    V_xx(5, 3) = Q_xu_(5, 0)*k_ux(0, 3) + Q_xu_(5, 1)*k_ux(1, 3) + Q_xx_(3, 5);
    V_xx(5, 4) = Q_xu_(5, 0)*k_ux(0, 4) + Q_xu_(5, 1)*k_ux(1, 4) + Q_xx_(4, 5);
    V_xx(5, 5) = Q_xu_(5, 0)*k_ux(0, 5) + Q_xu_(5, 1)*k_ux(1, 5) + Q_xu_(5, 4)*k_ux(4, 5) + Q_xx_(5, 5);
    V_xx(5, 6) = Q_xu_(5, 0)*k_ux(0, 6) + Q_xu_(5, 1)*k_ux(1, 6) + Q_xx_(5, 6);
    V_xx(6, 0) = Q_xu_(6, 0)*k_ux(0, 0) + Q_xu_(6, 1)*k_ux(1, 0) + Q_xx_(0, 6);
    V_xx(6, 1) = Q_xu_(6, 0)*k_ux(0, 1) + Q_xu_(6, 1)*k_ux(1, 1) + Q_xx_(1, 6);
    V_xx(6, 2) = Q_xu_(6, 0)*k_ux(0, 2) + Q_xu_(6, 1)*k_ux(1, 2) + Q_xx_(2, 6);
    V_xx(6, 3) = Q_xu_(6, 0)*k_ux(0, 3) + Q_xu_(6, 1)*k_ux(1, 3) + Q_xx_(3, 6);
    V_xx(6, 4) = Q_xu_(6, 0)*k_ux(0, 4) + Q_xu_(6, 1)*k_ux(1, 4) + Q_xx_(4, 6);
    V_xx(6, 5) = Q_xu_(6, 0)*k_ux(0, 5) + Q_xu_(6, 1)*k_ux(1, 5) + Q_xx_(5, 6);
    V_xx(6, 6) = Q_xu_(6, 0)*k_ux(0, 6) + Q_xu_(6, 1)*k_ux(1, 6) + Q_xx_(6, 6);

    // Evaluation of Vector V_x
    V_x(0) = Q_x_(0) + Q_xu_(0, 0)*v_u(0) + Q_xu_(0, 1)*v_u(1) + Q_xu_(0, 3)*v_u(3);
    V_x(1) = Q_x_(1) + Q_xu_(1, 0)*v_u(0) + Q_xu_(1, 1)*v_u(1) + Q_xu_(1, 10)*v_u(10) + Q_xu_(1, 11)*v_u(11) + Q_xu_(1, 12)*v_u(12) + Q_xu_(1, 2)*v_u(2) + Q_xu_(1, 5)*v_u(5) + Q_xu_(1, 6)*v_u(6) + Q_xu_(1, 7)*v_u(7) + Q_xu_(1, 8)*v_u(8) + Q_xu_(1, 9)*v_u(9);
    V_x(2) = Q_x_(2) + Q_xu_(2, 0)*v_u(0) + Q_xu_(2, 1)*v_u(1) + Q_xu_(2, 10)*v_u(10) + Q_xu_(2, 11)*v_u(11) + Q_xu_(2, 12)*v_u(12) + Q_xu_(2, 2)*v_u(2) + Q_xu_(2, 5)*v_u(5) + Q_xu_(2, 6)*v_u(6) + Q_xu_(2, 7)*v_u(7) + Q_xu_(2, 8)*v_u(8) + Q_xu_(2, 9)*v_u(9);
    V_x(3) = Q_x_(3) + Q_xu_(3, 0)*v_u(0) + Q_xu_(3, 1)*v_u(1) + Q_xu_(3, 10)*v_u(10) + Q_xu_(3, 11)*v_u(11) + Q_xu_(3, 12)*v_u(12) + Q_xu_(3, 2)*v_u(2) + Q_xu_(3, 5)*v_u(5) + Q_xu_(3, 6)*v_u(6) + Q_xu_(3, 7)*v_u(7) + Q_xu_(3, 8)*v_u(8) + Q_xu_(3, 9)*v_u(9);
    V_x(4) = Q_x_(4) + Q_xu_(4, 0)*v_u(0) + Q_xu_(4, 1)*v_u(1);
    V_x(5) = Q_x_(5) + Q_xu_(5, 0)*v_u(0) + Q_xu_(5, 1)*v_u(1) + Q_xu_(5, 4)*v_u(4);
    V_x(6) = Q_x_(6) + Q_xu_(6, 0)*v_u(0) + Q_xu_(6, 1)*v_u(1);

  }

  virtual void updateQnx(const Eigen::VectorXd& r_f, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& h_u, const Eigen::MatrixXd& k_ux, const Eigen::MatrixXd& k_ex, const Eigen::MatrixXd& k_ue, const Eigen::VectorXd& v_u, const Eigen::VectorXd& v_e) override {
    // Determine internal variables
    const double internal_0 = -Q_uu_(0, 1);

    // Evaluation of Matrix Q_un_
    Q_un_(0, 4) = -Fu(4, 0)*Q_uu_inv_(0, 0);
    Q_un_(0, 6) = -Fu(6, 1)*Q_uu_inv_(0, 1);
    Q_un_(1, 4) = -Fu(4, 0)*Q_uu_inv_(0, 1);
    Q_un_(1, 6) = -Fu(6, 1)*Q_uu_inv_(1, 1);

    // Evaluation of Matrix Q_nuu_
    Q_nuu_(0, 0) = -Q_uu_(0, 0);
    Q_nuu_(0, 1) = internal_0;
    Q_nuu_(1, 0) = internal_0;
    Q_nuu_(1, 1) = -Q_uu_(1, 1);
    Q_nuu_(2, 2) = -Q_uu_(2, 2);
    Q_nuu_(3, 3) = -Q_uu_(3, 3);
    Q_nuu_(4, 4) = -Q_uu_(4, 4);
    Q_nuu_(5, 5) = -Q_uu_(5, 5);
    Q_nuu_(6, 6) = -Q_uu_(6, 6);
    Q_nuu_(7, 7) = -Q_uu_(7, 7);
    Q_nuu_(8, 8) = -Q_uu_(8, 8);
    Q_nuu_(9, 9) = -Q_uu_(9, 9);
    Q_nuu_(10, 10) = -Q_uu_(10, 10);
    Q_nuu_(11, 11) = -Q_uu_(11, 11);
    Q_nuu_(12, 12) = -Q_uu_(12, 12);

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
    Q_vnx_(4, 6) = Fu(4, 0)*k_ux(0, 6);
    Q_vnx_(5, 5) = Fx(5, 5);
    Q_vnx_(5, 6) = Fx(5, 6);
    Q_vnx_(6, 0) = Fu(6, 1)*k_ux(1, 0);
    Q_vnx_(6, 1) = Fu(6, 1)*k_ux(1, 1);
    Q_vnx_(6, 2) = Fu(6, 1)*k_ux(1, 2);
    Q_vnx_(6, 3) = Fu(6, 1)*k_ux(1, 3);
    Q_vnx_(6, 4) = Fu(6, 1)*k_ux(1, 4);
    Q_vnx_(6, 5) = Fu(6, 1)*k_ux(1, 5);
    Q_vnx_(6, 6) = Fu(6, 1)*k_ux(1, 6) + Fx(6, 6);

    // Evaluation of Vector Q_vn_
    Q_vn_(0) = r_f(0);
    Q_vn_(1) = r_f(1);
    Q_vn_(2) = r_f(2);
    Q_vn_(3) = r_f(3);
    Q_vn_(4) = Fu(4, 0)*v_u(0) + r_f(4);
    Q_vn_(5) = r_f(5);
    Q_vn_(6) = Fu(6, 1)*v_u(1) + r_f(6);

  }

};

template <>
class SpatiotemporalPlannerModelIpmEvaluator<StageType::INITIAL, OcpConfig::FORWARD_EULER> : public IpmEvaluator {
 public:
  SpatiotemporalPlannerModelIpmEvaluator() : IpmEvaluator(7, 13, 4, 0, 7) {}
  virtual ~SpatiotemporalPlannerModelIpmEvaluator() = default;

  virtual void updateLX(const Eigen::VectorXd& l_x, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& h_x, const Eigen::MatrixXd& m_x, const Eigen::VectorXd& p, const Eigen::VectorXd& lambda, const Eigen::VectorXd& eta, const Eigen::VectorXd& nu, Eigen::VectorXd& sl_x) const override {
    // Evaluation of Vector sl_x
    sl_x(0) = Fx(0, 0)*p(0) + l_x(0) + m_x(0, 0)*nu(0);
    sl_x(1) = Fx(0, 1)*p(0) + Fx(1, 1)*p(1) + l_x(1) + m_x(1, 1)*nu(1);
    sl_x(2) = Fx(0, 2)*p(0) + Fx(2, 2)*p(2) + l_x(2) + m_x(2, 2)*nu(2);
    sl_x(3) = Fx(0, 3)*p(0) + Fx(1, 3)*p(1) + Fx(2, 3)*p(2) + Fx(3, 3)*p(3) + l_x(3) + m_x(3, 3)*nu(3);
    sl_x(4) = Fx(3, 4)*p(3) + Fx(4, 4)*p(4) + l_x(4) + m_x(4, 4)*nu(4);
    sl_x(5) = Fx(0, 5)*p(0) + Fx(1, 5)*p(1) + Fx(2, 5)*p(2) + Fx(3, 5)*p(3) + Fx(5, 5)*p(5) + l_x(5) + m_x(5, 5)*nu(5);
    sl_x(6) = Fx(5, 6)*p(5) + Fx(6, 6)*p(6) + l_x(6) + m_x(6, 6)*nu(6);

  }

  virtual void updateLU(const Eigen::VectorXd& l_u, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& g_u, const Eigen::MatrixXd& h_u, const Eigen::VectorXd& p, const Eigen::VectorXd& lambda, const Eigen::VectorXd& eta, Eigen::VectorXd& sl_u) const override {
    // Evaluation of Vector sl_u
    sl_u(0) = Fu(4, 0)*p(4) + g_u(0, 0)*lambda(0) + g_u(1, 0)*lambda(1) + l_u(0);
    sl_u(1) = Fu(6, 1)*p(6) + g_u(2, 1)*lambda(2) + g_u(3, 1)*lambda(3) + l_u(1);
    sl_u(2) = l_u(2);
    sl_u(3) = l_u(3);
    sl_u(4) = l_u(4);
    sl_u(5) = l_u(5);
    sl_u(6) = l_u(6);
    sl_u(7) = l_u(7);
    sl_u(8) = l_u(8);
    sl_u(9) = l_u(9);
    sl_u(10) = l_u(10);
    sl_u(11) = l_u(11);
    sl_u(12) = l_u(12);

  }

  virtual void updateQH(const Eigen::MatrixXd& l_xx, const Eigen::MatrixXd& l_xu, const Eigen::MatrixXd& l_uu, const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& V_xx_prev) override {
    // Determine internal variables
    const double internal_0 = Fx(0, 1)*V_xx_prev(0, 0);
    const double internal_1 = Fx(0, 0)*V_xx_prev(0, 1);
    const double internal_2 = Fx(0, 2)*V_xx_prev(0, 0);
    const double internal_3 = Fx(0, 0)*V_xx_prev(0, 2);
    const double internal_4 = Fx(0, 3)*V_xx_prev(0, 0);
    const double internal_5 = Fx(0, 0)*V_xx_prev(0, 3);
    const double internal_52 = Fu(4, 0)*Fu(6, 1);
    const double internal_6 = Fx(0, 0)*V_xx_prev(0, 4);
    const double internal_7 = Fx(0, 5)*V_xx_prev(0, 0);
    const double internal_8 = Fx(0, 0)*V_xx_prev(0, 5);
    const double internal_9 = Fx(0, 0)*V_xx_prev(0, 6);
    const double internal_11 = Fx(0, 1)*V_xx_prev(0, 1) + Fx(1, 1)*V_xx_prev(1, 1);
    const double internal_12 = Fx(0, 1)*V_xx_prev(0, 2) + Fx(1, 1)*V_xx_prev(1, 2);
    const double internal_13 = Fx(0, 1)*V_xx_prev(0, 3) + Fx(1, 1)*V_xx_prev(1, 3);
    const double internal_14 = Fx(0, 1)*V_xx_prev(0, 4) + Fx(1, 1)*V_xx_prev(1, 4);
    const double internal_15 = Fx(0, 1)*V_xx_prev(0, 5) + Fx(1, 1)*V_xx_prev(1, 5);
    const double internal_16 = Fx(0, 1)*V_xx_prev(0, 6) + Fx(1, 1)*V_xx_prev(1, 6);
    const double internal_18 = Fx(0, 2)*V_xx_prev(0, 1) + Fx(2, 2)*V_xx_prev(2, 1);
    const double internal_19 = Fx(0, 2)*V_xx_prev(0, 2) + Fx(2, 2)*V_xx_prev(2, 2);
    const double internal_20 = Fx(0, 2)*V_xx_prev(0, 3) + Fx(2, 2)*V_xx_prev(2, 3);
    const double internal_21 = Fx(0, 2)*V_xx_prev(0, 4) + Fx(2, 2)*V_xx_prev(2, 4);
    const double internal_22 = Fx(0, 2)*V_xx_prev(0, 5) + Fx(2, 2)*V_xx_prev(2, 5);
    const double internal_23 = Fx(0, 2)*V_xx_prev(0, 6) + Fx(2, 2)*V_xx_prev(2, 6);
    const double internal_25 = Fx(0, 3)*V_xx_prev(0, 1) + Fx(1, 3)*V_xx_prev(1, 1) + Fx(2, 3)*V_xx_prev(2, 1) + Fx(3, 3)*V_xx_prev(3, 1);
    const double internal_26 = Fx(0, 3)*V_xx_prev(0, 2) + Fx(1, 3)*V_xx_prev(1, 2) + Fx(2, 3)*V_xx_prev(2, 2) + Fx(3, 3)*V_xx_prev(3, 2);
    const double internal_27 = Fx(0, 3)*V_xx_prev(0, 3) + Fx(1, 3)*V_xx_prev(1, 3) + Fx(2, 3)*V_xx_prev(2, 3) + Fx(3, 3)*V_xx_prev(3, 3);
    const double internal_28 = Fx(0, 3)*V_xx_prev(0, 4) + Fx(1, 3)*V_xx_prev(1, 4) + Fx(2, 3)*V_xx_prev(2, 4) + Fx(3, 3)*V_xx_prev(3, 4);
    const double internal_29 = Fx(0, 3)*V_xx_prev(0, 5) + Fx(1, 3)*V_xx_prev(1, 5) + Fx(2, 3)*V_xx_prev(2, 5) + Fx(3, 3)*V_xx_prev(3, 5);
    const double internal_30 = Fx(0, 3)*V_xx_prev(0, 6) + Fx(1, 3)*V_xx_prev(1, 6) + Fx(2, 3)*V_xx_prev(2, 6) + Fx(3, 3)*V_xx_prev(3, 6);
    const double internal_31 = Fx(3, 4)*V_xx_prev(3, 0) + Fx(4, 4)*V_xx_prev(4, 0);
    const double internal_32 = Fx(3, 4)*V_xx_prev(3, 1) + Fx(4, 4)*V_xx_prev(4, 1);
    const double internal_33 = Fx(3, 4)*V_xx_prev(3, 2) + Fx(4, 4)*V_xx_prev(4, 2);
    const double internal_34 = Fx(3, 4)*V_xx_prev(3, 3) + Fx(4, 4)*V_xx_prev(4, 3);
    const double internal_35 = Fx(3, 4)*V_xx_prev(3, 4) + Fx(4, 4)*V_xx_prev(4, 4);
    const double internal_36 = Fx(3, 4)*V_xx_prev(3, 5) + Fx(4, 4)*V_xx_prev(4, 5);
    const double internal_37 = Fx(3, 4)*V_xx_prev(3, 6) + Fx(4, 4)*V_xx_prev(4, 6);
    const double internal_39 = Fx(0, 5)*V_xx_prev(0, 1) + Fx(1, 5)*V_xx_prev(1, 1) + Fx(2, 5)*V_xx_prev(2, 1) + Fx(3, 5)*V_xx_prev(3, 1) + Fx(5, 5)*V_xx_prev(5, 1);
    const double internal_40 = Fx(0, 5)*V_xx_prev(0, 2) + Fx(1, 5)*V_xx_prev(1, 2) + Fx(2, 5)*V_xx_prev(2, 2) + Fx(3, 5)*V_xx_prev(3, 2) + Fx(5, 5)*V_xx_prev(5, 2);
    const double internal_41 = Fx(0, 5)*V_xx_prev(0, 3) + Fx(1, 5)*V_xx_prev(1, 3) + Fx(2, 5)*V_xx_prev(2, 3) + Fx(3, 5)*V_xx_prev(3, 3) + Fx(5, 5)*V_xx_prev(5, 3);
    const double internal_42 = Fx(0, 5)*V_xx_prev(0, 4) + Fx(1, 5)*V_xx_prev(1, 4) + Fx(2, 5)*V_xx_prev(2, 4) + Fx(3, 5)*V_xx_prev(3, 4) + Fx(5, 5)*V_xx_prev(5, 4);
    const double internal_43 = Fx(0, 5)*V_xx_prev(0, 5) + Fx(1, 5)*V_xx_prev(1, 5) + Fx(2, 5)*V_xx_prev(2, 5) + Fx(3, 5)*V_xx_prev(3, 5) + Fx(5, 5)*V_xx_prev(5, 5);
    const double internal_44 = Fx(0, 5)*V_xx_prev(0, 6) + Fx(1, 5)*V_xx_prev(1, 6) + Fx(2, 5)*V_xx_prev(2, 6) + Fx(3, 5)*V_xx_prev(3, 6) + Fx(5, 5)*V_xx_prev(5, 6);
    const double internal_45 = Fx(5, 6)*V_xx_prev(5, 0) + Fx(6, 6)*V_xx_prev(6, 0);
    const double internal_46 = Fx(5, 6)*V_xx_prev(5, 1) + Fx(6, 6)*V_xx_prev(6, 1);
    const double internal_47 = Fx(5, 6)*V_xx_prev(5, 2) + Fx(6, 6)*V_xx_prev(6, 2);
    const double internal_48 = Fx(5, 6)*V_xx_prev(5, 3) + Fx(6, 6)*V_xx_prev(6, 3);
    const double internal_49 = Fx(5, 6)*V_xx_prev(5, 4) + Fx(6, 6)*V_xx_prev(6, 4);
    const double internal_50 = Fx(5, 6)*V_xx_prev(5, 5) + Fx(6, 6)*V_xx_prev(6, 5);
    const double internal_51 = Fx(5, 6)*V_xx_prev(5, 6) + Fx(6, 6)*V_xx_prev(6, 6);
    const double internal_10 = Fx(1, 1)*V_xx_prev(1, 0) + internal_0;
    const double internal_17 = Fx(2, 2)*V_xx_prev(2, 0) + internal_2;
    const double internal_24 = Fx(1, 3)*V_xx_prev(1, 0) + Fx(2, 3)*V_xx_prev(2, 0) + Fx(3, 3)*V_xx_prev(3, 0) + internal_4;
    const double internal_38 = Fx(1, 5)*V_xx_prev(1, 0) + Fx(2, 5)*V_xx_prev(2, 0) + Fx(3, 5)*V_xx_prev(3, 0) + Fx(5, 5)*V_xx_prev(5, 0) + internal_7;

    // Evaluation of Matrix Q_xx_
    Q_xx_(0, 0) = pow(Fx(0, 0), 2)*V_xx_prev(0, 0) + l_xx(0, 0);
    Q_xx_(0, 1) = Fx(0, 0)*internal_0 + Fx(1, 1)*internal_1;
    Q_xx_(0, 2) = Fx(0, 0)*internal_2 + Fx(2, 2)*internal_3;
    Q_xx_(0, 3) = Fx(0, 0)*internal_4 + Fx(1, 3)*internal_1 + Fx(2, 3)*internal_3 + Fx(3, 3)*internal_5;
    Q_xx_(0, 4) = Fx(3, 4)*internal_5 + Fx(4, 4)*internal_6;
    Q_xx_(0, 5) = Fx(0, 0)*internal_7 + Fx(1, 5)*internal_1 + Fx(2, 5)*internal_3 + Fx(3, 5)*internal_5 + Fx(5, 5)*internal_8;
    Q_xx_(0, 6) = Fx(5, 6)*internal_8 + Fx(6, 6)*internal_9;
    Q_xx_(1, 0) = Fx(0, 0)*internal_10;
    Q_xx_(1, 1) = Fx(0, 1)*internal_10 + Fx(1, 1)*internal_11 + l_xx(1, 1);
    Q_xx_(1, 2) = Fx(0, 2)*internal_10 + Fx(2, 2)*internal_12 + l_xx(1, 2);
    Q_xx_(1, 3) = Fx(0, 3)*internal_10 + Fx(1, 3)*internal_11 + Fx(2, 3)*internal_12 + Fx(3, 3)*internal_13;
    Q_xx_(1, 4) = Fx(3, 4)*internal_13 + Fx(4, 4)*internal_14;
    Q_xx_(1, 5) = Fx(0, 5)*internal_10 + Fx(1, 5)*internal_11 + Fx(2, 5)*internal_12 + Fx(3, 5)*internal_13 + Fx(5, 5)*internal_15;
    Q_xx_(1, 6) = Fx(5, 6)*internal_15 + Fx(6, 6)*internal_16;
    Q_xx_(2, 0) = Fx(0, 0)*internal_17;
    Q_xx_(2, 1) = Fx(0, 1)*internal_17 + Fx(1, 1)*internal_18 + l_xx(2, 1);
    Q_xx_(2, 2) = Fx(0, 2)*internal_17 + Fx(2, 2)*internal_19 + l_xx(2, 2);
    Q_xx_(2, 3) = Fx(0, 3)*internal_17 + Fx(1, 3)*internal_18 + Fx(2, 3)*internal_19 + Fx(3, 3)*internal_20;
    Q_xx_(2, 4) = Fx(3, 4)*internal_20 + Fx(4, 4)*internal_21;
    Q_xx_(2, 5) = Fx(0, 5)*internal_17 + Fx(1, 5)*internal_18 + Fx(2, 5)*internal_19 + Fx(3, 5)*internal_20 + Fx(5, 5)*internal_22;
    Q_xx_(2, 6) = Fx(5, 6)*internal_22 + Fx(6, 6)*internal_23;
    Q_xx_(3, 0) = Fx(0, 0)*internal_24;
    Q_xx_(3, 1) = Fx(0, 1)*internal_24 + Fx(1, 1)*internal_25;
    Q_xx_(3, 2) = Fx(0, 2)*internal_24 + Fx(2, 2)*internal_26;
    Q_xx_(3, 3) = Fx(0, 3)*internal_24 + Fx(1, 3)*internal_25 + Fx(2, 3)*internal_26 + Fx(3, 3)*internal_27 + l_xx(3, 3);
    Q_xx_(3, 4) = Fx(3, 4)*internal_27 + Fx(4, 4)*internal_28;
    Q_xx_(3, 5) = Fx(0, 5)*internal_24 + Fx(1, 5)*internal_25 + Fx(2, 5)*internal_26 + Fx(3, 5)*internal_27 + Fx(5, 5)*internal_29;
    Q_xx_(3, 6) = Fx(5, 6)*internal_29 + Fx(6, 6)*internal_30;
    Q_xx_(4, 0) = Fx(0, 0)*internal_31;
    Q_xx_(4, 1) = Fx(0, 1)*internal_31 + Fx(1, 1)*internal_32;
    Q_xx_(4, 2) = Fx(0, 2)*internal_31 + Fx(2, 2)*internal_33;
    Q_xx_(4, 3) = Fx(0, 3)*internal_31 + Fx(1, 3)*internal_32 + Fx(2, 3)*internal_33 + Fx(3, 3)*internal_34;
    Q_xx_(4, 4) = Fx(3, 4)*internal_34 + Fx(4, 4)*internal_35 + l_xx(4, 4);
    Q_xx_(4, 5) = Fx(0, 5)*internal_31 + Fx(1, 5)*internal_32 + Fx(2, 5)*internal_33 + Fx(3, 5)*internal_34 + Fx(5, 5)*internal_36;
    Q_xx_(4, 6) = Fx(5, 6)*internal_36 + Fx(6, 6)*internal_37;
    Q_xx_(5, 0) = Fx(0, 0)*internal_38;
    Q_xx_(5, 1) = Fx(0, 1)*internal_38 + Fx(1, 1)*internal_39;
    Q_xx_(5, 2) = Fx(0, 2)*internal_38 + Fx(2, 2)*internal_40;
    Q_xx_(5, 3) = Fx(0, 3)*internal_38 + Fx(1, 3)*internal_39 + Fx(2, 3)*internal_40 + Fx(3, 3)*internal_41;
    Q_xx_(5, 4) = Fx(3, 4)*internal_41 + Fx(4, 4)*internal_42;
    Q_xx_(5, 5) = Fx(0, 5)*internal_38 + Fx(1, 5)*internal_39 + Fx(2, 5)*internal_40 + Fx(3, 5)*internal_41 + Fx(5, 5)*internal_43 + l_xx(5, 5);
    Q_xx_(5, 6) = Fx(5, 6)*internal_43 + Fx(6, 6)*internal_44;
    Q_xx_(6, 0) = Fx(0, 0)*internal_45;
    Q_xx_(6, 1) = Fx(0, 1)*internal_45 + Fx(1, 1)*internal_46;
    Q_xx_(6, 2) = Fx(0, 2)*internal_45 + Fx(2, 2)*internal_47;
    Q_xx_(6, 3) = Fx(0, 3)*internal_45 + Fx(1, 3)*internal_46 + Fx(2, 3)*internal_47 + Fx(3, 3)*internal_48;
    Q_xx_(6, 4) = Fx(3, 4)*internal_48 + Fx(4, 4)*internal_49;
    Q_xx_(6, 5) = Fx(0, 5)*internal_45 + Fx(1, 5)*internal_46 + Fx(2, 5)*internal_47 + Fx(3, 5)*internal_48 + Fx(5, 5)*internal_50;
    Q_xx_(6, 6) = Fx(5, 6)*internal_50 + Fx(6, 6)*internal_51 + l_xx(6, 6);

    // Evaluation of Matrix Q_xu_
    Q_xu_(0, 0) = Fu(4, 0)*internal_6;
    Q_xu_(0, 1) = Fu(6, 1)*internal_9;
    Q_xu_(1, 0) = Fu(4, 0)*internal_14;
    Q_xu_(1, 1) = Fu(6, 1)*internal_16;
    Q_xu_(2, 0) = Fu(4, 0)*internal_21;
    Q_xu_(2, 1) = Fu(6, 1)*internal_23;
    Q_xu_(3, 0) = Fu(4, 0)*internal_28;
    Q_xu_(3, 1) = Fu(6, 1)*internal_30;
    Q_xu_(4, 0) = Fu(4, 0)*internal_35;
    Q_xu_(4, 1) = Fu(6, 1)*internal_37;
    Q_xu_(5, 0) = Fu(4, 0)*internal_42;
    Q_xu_(5, 1) = Fu(6, 1)*internal_44;
    Q_xu_(6, 0) = Fu(4, 0)*internal_49;
    Q_xu_(6, 1) = Fu(6, 1)*internal_51;

    // Evaluation of Matrix Q_uu_
    Q_uu_(0, 0) = pow(Fu(4, 0), 2)*V_xx_prev(4, 4) + Sigma_(0)*pow(g_u(0, 0), 2) + Sigma_(1)*pow(g_u(1, 0), 2) + l_uu(0, 0);
    Q_uu_(0, 1) = V_xx_prev(4, 6)*internal_52;
    Q_uu_(1, 0) = V_xx_prev(6, 4)*internal_52;
    Q_uu_(1, 1) = pow(Fu(6, 1), 2)*V_xx_prev(6, 6) + Sigma_(2)*pow(g_u(2, 1), 2) + Sigma_(3)*pow(g_u(3, 1), 2) + l_uu(1, 1);
    Q_uu_(2, 2) = l_uu(2, 2);
    Q_uu_(3, 3) = l_uu(3, 3);
    Q_uu_(4, 4) = l_uu(4, 4);
    Q_uu_(5, 5) = l_uu(5, 5);
    Q_uu_(6, 6) = l_uu(6, 6);
    Q_uu_(7, 7) = l_uu(7, 7);
    Q_uu_(8, 8) = l_uu(8, 8);
    Q_uu_(9, 9) = l_uu(9, 9);
    Q_uu_(10, 10) = l_uu(10, 10);
    Q_uu_(11, 11) = l_uu(11, 11);
    Q_uu_(12, 12) = l_uu(12, 12);

  }

  virtual void updateQG(const Eigen::VectorXd& sl_x, const Eigen::VectorXd& sl_u, const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::VectorXd& r_f, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& V_xx_prev, const Eigen::VectorXd& V_x_prev) override {
    // Determine internal variables
    const double internal_0 = V_x_prev(0) + V_xx_prev(0, 0)*r_f(0) + V_xx_prev(0, 1)*r_f(1) + V_xx_prev(0, 2)*r_f(2) + V_xx_prev(0, 3)*r_f(3) + V_xx_prev(0, 4)*r_f(4) + V_xx_prev(0, 5)*r_f(5) + V_xx_prev(0, 6)*r_f(6);
    const double internal_1 = V_x_prev(1) + V_xx_prev(1, 0)*r_f(0) + V_xx_prev(1, 1)*r_f(1) + V_xx_prev(1, 2)*r_f(2) + V_xx_prev(1, 3)*r_f(3) + V_xx_prev(1, 4)*r_f(4) + V_xx_prev(1, 5)*r_f(5) + V_xx_prev(1, 6)*r_f(6);
    const double internal_2 = V_x_prev(2) + V_xx_prev(2, 0)*r_f(0) + V_xx_prev(2, 1)*r_f(1) + V_xx_prev(2, 2)*r_f(2) + V_xx_prev(2, 3)*r_f(3) + V_xx_prev(2, 4)*r_f(4) + V_xx_prev(2, 5)*r_f(5) + V_xx_prev(2, 6)*r_f(6);
    const double internal_3 = V_x_prev(3) + V_xx_prev(3, 0)*r_f(0) + V_xx_prev(3, 1)*r_f(1) + V_xx_prev(3, 2)*r_f(2) + V_xx_prev(3, 3)*r_f(3) + V_xx_prev(3, 4)*r_f(4) + V_xx_prev(3, 5)*r_f(5) + V_xx_prev(3, 6)*r_f(6);
    const double internal_4 = V_x_prev(4) + V_xx_prev(4, 0)*r_f(0) + V_xx_prev(4, 1)*r_f(1) + V_xx_prev(4, 2)*r_f(2) + V_xx_prev(4, 3)*r_f(3) + V_xx_prev(4, 4)*r_f(4) + V_xx_prev(4, 5)*r_f(5) + V_xx_prev(4, 6)*r_f(6);
    const double internal_5 = V_x_prev(5) + V_xx_prev(5, 0)*r_f(0) + V_xx_prev(5, 1)*r_f(1) + V_xx_prev(5, 2)*r_f(2) + V_xx_prev(5, 3)*r_f(3) + V_xx_prev(5, 4)*r_f(4) + V_xx_prev(5, 5)*r_f(5) + V_xx_prev(5, 6)*r_f(6);
    const double internal_6 = V_x_prev(6) + V_xx_prev(6, 0)*r_f(0) + V_xx_prev(6, 1)*r_f(1) + V_xx_prev(6, 2)*r_f(2) + V_xx_prev(6, 3)*r_f(3) + V_xx_prev(6, 4)*r_f(4) + V_xx_prev(6, 5)*r_f(5) + V_xx_prev(6, 6)*r_f(6);

    // Evaluation of Vector Q_x_
    Q_x_(0) = Fx(0, 0)*internal_0 + sl_x(0);
    Q_x_(1) = Fx(0, 1)*internal_0 + Fx(1, 1)*internal_1 + sl_x(1);
    Q_x_(2) = Fx(0, 2)*internal_0 + Fx(2, 2)*internal_2 + sl_x(2);
    Q_x_(3) = Fx(0, 3)*internal_0 + Fx(1, 3)*internal_1 + Fx(2, 3)*internal_2 + Fx(3, 3)*internal_3 + sl_x(3);
    Q_x_(4) = Fx(3, 4)*internal_3 + Fx(4, 4)*internal_4 + sl_x(4);
    Q_x_(5) = Fx(0, 5)*internal_0 + Fx(1, 5)*internal_1 + Fx(2, 5)*internal_2 + Fx(3, 5)*internal_3 + Fx(5, 5)*internal_5 + sl_x(5);
    Q_x_(6) = Fx(5, 6)*internal_5 + Fx(6, 6)*internal_6 + sl_x(6);

    // Evaluation of Vector Q_u_
    Q_u_(0) = Fu(4, 0)*internal_4 + V_(0)*g_u(0, 0) + V_(1)*g_u(1, 0) + sl_u(0);
    Q_u_(1) = Fu(6, 1)*internal_6 + V_(2)*g_u(2, 1) + V_(3)*g_u(3, 1) + sl_u(1);
    Q_u_(2) = sl_u(2);
    Q_u_(3) = sl_u(3);
    Q_u_(4) = sl_u(4);
    Q_u_(5) = sl_u(5);
    Q_u_(6) = sl_u(6);
    Q_u_(7) = sl_u(7);
    Q_u_(8) = sl_u(8);
    Q_u_(9) = sl_u(9);
    Q_u_(10) = sl_u(10);
    Q_u_(11) = sl_u(11);
    Q_u_(12) = sl_u(12);

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
    Q_uu_inv_(3, 3) = 1.0/Q_uu_(3, 3);
    Q_uu_inv_(4, 4) = 1.0/Q_uu_(4, 4);
    Q_uu_inv_(5, 5) = 1.0/Q_uu_(5, 5);
    Q_uu_inv_(6, 6) = 1.0/Q_uu_(6, 6);
    Q_uu_inv_(7, 7) = 1.0/Q_uu_(7, 7);
    Q_uu_inv_(8, 8) = 1.0/Q_uu_(8, 8);
    Q_uu_inv_(9, 9) = 1.0/Q_uu_(9, 9);
    Q_uu_inv_(10, 10) = 1.0/Q_uu_(10, 10);
    Q_uu_inv_(11, 11) = 1.0/Q_uu_(11, 11);
    Q_uu_inv_(12, 12) = 1.0/Q_uu_(12, 12);

  }

  virtual void updateKVU(Eigen::MatrixXd& k_ux, Eigen::VectorXd& v_u) override {
    // Evaluation of Matrix k_ux
    k_ux(0, 0) = -Q_uu_inv_(0, 0)*Q_xu_(0, 0) - Q_uu_inv_(0, 1)*Q_xu_(0, 1);
    k_ux(0, 1) = -Q_uu_inv_(0, 0)*Q_xu_(1, 0) - Q_uu_inv_(0, 1)*Q_xu_(1, 1);
    k_ux(0, 2) = -Q_uu_inv_(0, 0)*Q_xu_(2, 0) - Q_uu_inv_(0, 1)*Q_xu_(2, 1);
    k_ux(0, 3) = -Q_uu_inv_(0, 0)*Q_xu_(3, 0) - Q_uu_inv_(0, 1)*Q_xu_(3, 1);
    k_ux(0, 4) = -Q_uu_inv_(0, 0)*Q_xu_(4, 0) - Q_uu_inv_(0, 1)*Q_xu_(4, 1);
    k_ux(0, 5) = -Q_uu_inv_(0, 0)*Q_xu_(5, 0) - Q_uu_inv_(0, 1)*Q_xu_(5, 1);
    k_ux(0, 6) = -Q_uu_inv_(0, 0)*Q_xu_(6, 0) - Q_uu_inv_(0, 1)*Q_xu_(6, 1);
    k_ux(1, 0) = -Q_uu_inv_(0, 1)*Q_xu_(0, 0) - Q_uu_inv_(1, 1)*Q_xu_(0, 1);
    k_ux(1, 1) = -Q_uu_inv_(0, 1)*Q_xu_(1, 0) - Q_uu_inv_(1, 1)*Q_xu_(1, 1);
    k_ux(1, 2) = -Q_uu_inv_(0, 1)*Q_xu_(2, 0) - Q_uu_inv_(1, 1)*Q_xu_(2, 1);
    k_ux(1, 3) = -Q_uu_inv_(0, 1)*Q_xu_(3, 0) - Q_uu_inv_(1, 1)*Q_xu_(3, 1);
    k_ux(1, 4) = -Q_uu_inv_(0, 1)*Q_xu_(4, 0) - Q_uu_inv_(1, 1)*Q_xu_(4, 1);
    k_ux(1, 5) = -Q_uu_inv_(0, 1)*Q_xu_(5, 0) - Q_uu_inv_(1, 1)*Q_xu_(5, 1);
    k_ux(1, 6) = -Q_uu_inv_(0, 1)*Q_xu_(6, 0) - Q_uu_inv_(1, 1)*Q_xu_(6, 1);

    // Evaluation of Vector v_u
    v_u(0) = -Q_u_(0)*Q_uu_inv_(0, 0) - Q_u_(1)*Q_uu_inv_(0, 1);
    v_u(1) = -Q_u_(0)*Q_uu_inv_(0, 1) - Q_u_(1)*Q_uu_inv_(1, 1);
    v_u(2) = -Q_u_(2)*Q_uu_inv_(2, 2);
    v_u(3) = -Q_u_(3)*Q_uu_inv_(3, 3);
    v_u(4) = -Q_u_(4)*Q_uu_inv_(4, 4);
    v_u(5) = -Q_u_(5)*Q_uu_inv_(5, 5);
    v_u(6) = -Q_u_(6)*Q_uu_inv_(6, 6);
    v_u(7) = -Q_u_(7)*Q_uu_inv_(7, 7);
    v_u(8) = -Q_u_(8)*Q_uu_inv_(8, 8);
    v_u(9) = -Q_u_(9)*Q_uu_inv_(9, 9);
    v_u(10) = -Q_u_(10)*Q_uu_inv_(10, 10);
    v_u(11) = -Q_u_(11)*Q_uu_inv_(11, 11);
    v_u(12) = -Q_u_(12)*Q_uu_inv_(12, 12);

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
    V_xx(0, 6) = Q_xu_(0, 0)*k_ux(0, 6) + Q_xu_(0, 1)*k_ux(1, 6) + Q_xx_(0, 6);
    V_xx(1, 0) = Q_xu_(1, 0)*k_ux(0, 0) + Q_xu_(1, 1)*k_ux(1, 0) + Q_xx_(0, 1);
    V_xx(1, 1) = Q_xu_(1, 0)*k_ux(0, 1) + Q_xu_(1, 1)*k_ux(1, 1) + Q_xx_(1, 1);
    V_xx(1, 2) = Q_xu_(1, 0)*k_ux(0, 2) + Q_xu_(1, 1)*k_ux(1, 2) + Q_xx_(1, 2);
    V_xx(1, 3) = Q_xu_(1, 0)*k_ux(0, 3) + Q_xu_(1, 1)*k_ux(1, 3) + Q_xx_(1, 3);
    V_xx(1, 4) = Q_xu_(1, 0)*k_ux(0, 4) + Q_xu_(1, 1)*k_ux(1, 4) + Q_xx_(1, 4);
    V_xx(1, 5) = Q_xu_(1, 0)*k_ux(0, 5) + Q_xu_(1, 1)*k_ux(1, 5) + Q_xx_(1, 5);
    V_xx(1, 6) = Q_xu_(1, 0)*k_ux(0, 6) + Q_xu_(1, 1)*k_ux(1, 6) + Q_xx_(1, 6);
    V_xx(2, 0) = Q_xu_(2, 0)*k_ux(0, 0) + Q_xu_(2, 1)*k_ux(1, 0) + Q_xx_(0, 2);
    V_xx(2, 1) = Q_xu_(2, 0)*k_ux(0, 1) + Q_xu_(2, 1)*k_ux(1, 1) + Q_xx_(1, 2);
    V_xx(2, 2) = Q_xu_(2, 0)*k_ux(0, 2) + Q_xu_(2, 1)*k_ux(1, 2) + Q_xx_(2, 2);
    V_xx(2, 3) = Q_xu_(2, 0)*k_ux(0, 3) + Q_xu_(2, 1)*k_ux(1, 3) + Q_xx_(2, 3);
    V_xx(2, 4) = Q_xu_(2, 0)*k_ux(0, 4) + Q_xu_(2, 1)*k_ux(1, 4) + Q_xx_(2, 4);
    V_xx(2, 5) = Q_xu_(2, 0)*k_ux(0, 5) + Q_xu_(2, 1)*k_ux(1, 5) + Q_xx_(2, 5);
    V_xx(2, 6) = Q_xu_(2, 0)*k_ux(0, 6) + Q_xu_(2, 1)*k_ux(1, 6) + Q_xx_(2, 6);
    V_xx(3, 0) = Q_xu_(3, 0)*k_ux(0, 0) + Q_xu_(3, 1)*k_ux(1, 0) + Q_xx_(0, 3);
    V_xx(3, 1) = Q_xu_(3, 0)*k_ux(0, 1) + Q_xu_(3, 1)*k_ux(1, 1) + Q_xx_(1, 3);
    V_xx(3, 2) = Q_xu_(3, 0)*k_ux(0, 2) + Q_xu_(3, 1)*k_ux(1, 2) + Q_xx_(2, 3);
    V_xx(3, 3) = Q_xu_(3, 0)*k_ux(0, 3) + Q_xu_(3, 1)*k_ux(1, 3) + Q_xx_(3, 3);
    V_xx(3, 4) = Q_xu_(3, 0)*k_ux(0, 4) + Q_xu_(3, 1)*k_ux(1, 4) + Q_xx_(3, 4);
    V_xx(3, 5) = Q_xu_(3, 0)*k_ux(0, 5) + Q_xu_(3, 1)*k_ux(1, 5) + Q_xx_(3, 5);
    V_xx(3, 6) = Q_xu_(3, 0)*k_ux(0, 6) + Q_xu_(3, 1)*k_ux(1, 6) + Q_xx_(3, 6);
    V_xx(4, 0) = Q_xu_(4, 0)*k_ux(0, 0) + Q_xu_(4, 1)*k_ux(1, 0) + Q_xx_(0, 4);
    V_xx(4, 1) = Q_xu_(4, 0)*k_ux(0, 1) + Q_xu_(4, 1)*k_ux(1, 1) + Q_xx_(1, 4);
    V_xx(4, 2) = Q_xu_(4, 0)*k_ux(0, 2) + Q_xu_(4, 1)*k_ux(1, 2) + Q_xx_(2, 4);
    V_xx(4, 3) = Q_xu_(4, 0)*k_ux(0, 3) + Q_xu_(4, 1)*k_ux(1, 3) + Q_xx_(3, 4);
    V_xx(4, 4) = Q_xu_(4, 0)*k_ux(0, 4) + Q_xu_(4, 1)*k_ux(1, 4) + Q_xx_(4, 4);
    V_xx(4, 5) = Q_xu_(4, 0)*k_ux(0, 5) + Q_xu_(4, 1)*k_ux(1, 5) + Q_xx_(4, 5);
    V_xx(4, 6) = Q_xu_(4, 0)*k_ux(0, 6) + Q_xu_(4, 1)*k_ux(1, 6) + Q_xx_(4, 6);
    V_xx(5, 0) = Q_xu_(5, 0)*k_ux(0, 0) + Q_xu_(5, 1)*k_ux(1, 0) + Q_xx_(0, 5);
    V_xx(5, 1) = Q_xu_(5, 0)*k_ux(0, 1) + Q_xu_(5, 1)*k_ux(1, 1) + Q_xx_(1, 5);
    V_xx(5, 2) = Q_xu_(5, 0)*k_ux(0, 2) + Q_xu_(5, 1)*k_ux(1, 2) + Q_xx_(2, 5);
    V_xx(5, 3) = Q_xu_(5, 0)*k_ux(0, 3) + Q_xu_(5, 1)*k_ux(1, 3) + Q_xx_(3, 5);
    V_xx(5, 4) = Q_xu_(5, 0)*k_ux(0, 4) + Q_xu_(5, 1)*k_ux(1, 4) + Q_xx_(4, 5);
    V_xx(5, 5) = Q_xu_(5, 0)*k_ux(0, 5) + Q_xu_(5, 1)*k_ux(1, 5) + Q_xx_(5, 5);
    V_xx(5, 6) = Q_xu_(5, 0)*k_ux(0, 6) + Q_xu_(5, 1)*k_ux(1, 6) + Q_xx_(5, 6);
    V_xx(6, 0) = Q_xu_(6, 0)*k_ux(0, 0) + Q_xu_(6, 1)*k_ux(1, 0) + Q_xx_(0, 6);
    V_xx(6, 1) = Q_xu_(6, 0)*k_ux(0, 1) + Q_xu_(6, 1)*k_ux(1, 1) + Q_xx_(1, 6);
    V_xx(6, 2) = Q_xu_(6, 0)*k_ux(0, 2) + Q_xu_(6, 1)*k_ux(1, 2) + Q_xx_(2, 6);
    V_xx(6, 3) = Q_xu_(6, 0)*k_ux(0, 3) + Q_xu_(6, 1)*k_ux(1, 3) + Q_xx_(3, 6);
    V_xx(6, 4) = Q_xu_(6, 0)*k_ux(0, 4) + Q_xu_(6, 1)*k_ux(1, 4) + Q_xx_(4, 6);
    V_xx(6, 5) = Q_xu_(6, 0)*k_ux(0, 5) + Q_xu_(6, 1)*k_ux(1, 5) + Q_xx_(5, 6);
    V_xx(6, 6) = Q_xu_(6, 0)*k_ux(0, 6) + Q_xu_(6, 1)*k_ux(1, 6) + Q_xx_(6, 6);

    // Evaluation of Vector V_x
    V_x(0) = Q_x_(0) + Q_xu_(0, 0)*v_u(0) + Q_xu_(0, 1)*v_u(1);
    V_x(1) = Q_x_(1) + Q_xu_(1, 0)*v_u(0) + Q_xu_(1, 1)*v_u(1);
    V_x(2) = Q_x_(2) + Q_xu_(2, 0)*v_u(0) + Q_xu_(2, 1)*v_u(1);
    V_x(3) = Q_x_(3) + Q_xu_(3, 0)*v_u(0) + Q_xu_(3, 1)*v_u(1);
    V_x(4) = Q_x_(4) + Q_xu_(4, 0)*v_u(0) + Q_xu_(4, 1)*v_u(1);
    V_x(5) = Q_x_(5) + Q_xu_(5, 0)*v_u(0) + Q_xu_(5, 1)*v_u(1);
    V_x(6) = Q_x_(6) + Q_xu_(6, 0)*v_u(0) + Q_xu_(6, 1)*v_u(1);

  }

  virtual void updateQnx(const Eigen::VectorXd& r_f, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& h_u, const Eigen::MatrixXd& k_ux, const Eigen::MatrixXd& k_ex, const Eigen::MatrixXd& k_ue, const Eigen::VectorXd& v_u, const Eigen::VectorXd& v_e) override {
    // Determine internal variables
    const double internal_0 = -Q_uu_(0, 1);

    // Evaluation of Matrix Q_un_
    Q_un_(0, 4) = -Fu(4, 0)*Q_uu_inv_(0, 0);
    Q_un_(0, 6) = -Fu(6, 1)*Q_uu_inv_(0, 1);
    Q_un_(1, 4) = -Fu(4, 0)*Q_uu_inv_(0, 1);
    Q_un_(1, 6) = -Fu(6, 1)*Q_uu_inv_(1, 1);

    // Evaluation of Matrix Q_nuu_
    Q_nuu_(0, 0) = -Q_uu_(0, 0);
    Q_nuu_(0, 1) = internal_0;
    Q_nuu_(1, 0) = internal_0;
    Q_nuu_(1, 1) = -Q_uu_(1, 1);
    Q_nuu_(2, 2) = -Q_uu_(2, 2);
    Q_nuu_(3, 3) = -Q_uu_(3, 3);
    Q_nuu_(4, 4) = -Q_uu_(4, 4);
    Q_nuu_(5, 5) = -Q_uu_(5, 5);
    Q_nuu_(6, 6) = -Q_uu_(6, 6);
    Q_nuu_(7, 7) = -Q_uu_(7, 7);
    Q_nuu_(8, 8) = -Q_uu_(8, 8);
    Q_nuu_(9, 9) = -Q_uu_(9, 9);
    Q_nuu_(10, 10) = -Q_uu_(10, 10);
    Q_nuu_(11, 11) = -Q_uu_(11, 11);
    Q_nuu_(12, 12) = -Q_uu_(12, 12);

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
    Q_vnx_(4, 6) = Fu(4, 0)*k_ux(0, 6);
    Q_vnx_(5, 5) = Fx(5, 5);
    Q_vnx_(5, 6) = Fx(5, 6);
    Q_vnx_(6, 0) = Fu(6, 1)*k_ux(1, 0);
    Q_vnx_(6, 1) = Fu(6, 1)*k_ux(1, 1);
    Q_vnx_(6, 2) = Fu(6, 1)*k_ux(1, 2);
    Q_vnx_(6, 3) = Fu(6, 1)*k_ux(1, 3);
    Q_vnx_(6, 4) = Fu(6, 1)*k_ux(1, 4);
    Q_vnx_(6, 5) = Fu(6, 1)*k_ux(1, 5);
    Q_vnx_(6, 6) = Fu(6, 1)*k_ux(1, 6) + Fx(6, 6);

    // Evaluation of Vector Q_vn_
    Q_vn_(0) = r_f(0);
    Q_vn_(1) = r_f(1);
    Q_vn_(2) = r_f(2);
    Q_vn_(3) = r_f(3);
    Q_vn_(4) = Fu(4, 0)*v_u(0) + r_f(4);
    Q_vn_(5) = r_f(5);
    Q_vn_(6) = Fu(6, 1)*v_u(1) + r_f(6);

  }

};

template <>
class SpatiotemporalPlannerModelIpmEvaluator<StageType::NORMINAL, OcpConfig::ERK4> : public IpmEvaluator {
 public:
  SpatiotemporalPlannerModelIpmEvaluator() : IpmEvaluator(7, 13, 43, 0, 0) {}
  virtual ~SpatiotemporalPlannerModelIpmEvaluator() = default;

  virtual void updateLX(const Eigen::VectorXd& l_x, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& h_x, const Eigen::MatrixXd& m_x, const Eigen::VectorXd& p, const Eigen::VectorXd& lambda, const Eigen::VectorXd& eta, const Eigen::VectorXd& nu, Eigen::VectorXd& sl_x) const override {
    // Evaluation of Vector sl_x
    sl_x(0) = Fx(0, 0)*p(0) + g_x(41, 0)*lambda(41) + g_x(8, 0)*lambda(8) + g_x(9, 0)*lambda(9) + l_x(0);
    sl_x(1) = Fx(0, 1)*p(0) + Fx(1, 1)*p(1) + g_x(0, 1)*lambda(0) + g_x(1, 1)*lambda(1) + g_x(19, 1)*lambda(19) + g_x(2, 1)*lambda(2) + g_x(20, 1)*lambda(20) + g_x(21, 1)*lambda(21) + g_x(22, 1)*lambda(22) + g_x(23, 1)*lambda(23) + g_x(24, 1)*lambda(24) + g_x(25, 1)*lambda(25) + g_x(26, 1)*lambda(26) + g_x(27, 1)*lambda(27) + g_x(28, 1)*lambda(28) + g_x(29, 1)*lambda(29) + g_x(3, 1)*lambda(3) + g_x(30, 1)*lambda(30) + g_x(31, 1)*lambda(31) + g_x(32, 1)*lambda(32) + g_x(33, 1)*lambda(33) + g_x(34, 1)*lambda(34) + g_x(35, 1)*lambda(35) + g_x(36, 1)*lambda(36) + g_x(37, 1)*lambda(37) + g_x(38, 1)*lambda(38) + g_x(39, 1)*lambda(39) + g_x(4, 1)*lambda(4) + g_x(40, 1)*lambda(40) + g_x(5, 1)*lambda(5) + l_x(1);
    sl_x(2) = Fx(0, 2)*p(0) + Fx(2, 2)*p(2) + g_x(0, 2)*lambda(0) + g_x(1, 2)*lambda(1) + g_x(19, 2)*lambda(19) + g_x(2, 2)*lambda(2) + g_x(20, 2)*lambda(20) + g_x(21, 2)*lambda(21) + g_x(22, 2)*lambda(22) + g_x(23, 2)*lambda(23) + g_x(24, 2)*lambda(24) + g_x(25, 2)*lambda(25) + g_x(26, 2)*lambda(26) + g_x(27, 2)*lambda(27) + g_x(28, 2)*lambda(28) + g_x(29, 2)*lambda(29) + g_x(3, 2)*lambda(3) + g_x(30, 2)*lambda(30) + g_x(31, 2)*lambda(31) + g_x(32, 2)*lambda(32) + g_x(33, 2)*lambda(33) + g_x(34, 2)*lambda(34) + g_x(35, 2)*lambda(35) + g_x(36, 2)*lambda(36) + g_x(37, 2)*lambda(37) + g_x(38, 2)*lambda(38) + g_x(39, 2)*lambda(39) + g_x(4, 2)*lambda(4) + g_x(40, 2)*lambda(40) + g_x(5, 2)*lambda(5) + l_x(2);
    sl_x(3) = Fx(0, 3)*p(0) + Fx(1, 3)*p(1) + Fx(2, 3)*p(2) + Fx(3, 3)*p(3) + g_x(14, 3)*lambda(14) + g_x(19, 3)*lambda(19) + g_x(2, 3)*lambda(2) + g_x(20, 3)*lambda(20) + g_x(21, 3)*lambda(21) + g_x(22, 3)*lambda(22) + g_x(23, 3)*lambda(23) + g_x(24, 3)*lambda(24) + g_x(25, 3)*lambda(25) + g_x(26, 3)*lambda(26) + g_x(27, 3)*lambda(27) + g_x(28, 3)*lambda(28) + g_x(29, 3)*lambda(29) + g_x(3, 3)*lambda(3) + g_x(30, 3)*lambda(30) + g_x(31, 3)*lambda(31) + g_x(32, 3)*lambda(32) + g_x(33, 3)*lambda(33) + g_x(34, 3)*lambda(34) + g_x(37, 3)*lambda(37) + g_x(38, 3)*lambda(38) + g_x(39, 3)*lambda(39) + g_x(4, 3)*lambda(4) + g_x(40, 3)*lambda(40) + g_x(5, 3)*lambda(5) + l_x(3);
    sl_x(4) = Fx(0, 4)*p(0) + Fx(1, 4)*p(1) + Fx(2, 4)*p(2) + Fx(3, 4)*p(3) + Fx(4, 4)*p(4) + g_x(6, 4)*lambda(6) + g_x(7, 4)*lambda(7) + l_x(4);
    sl_x(5) = Fx(0, 5)*p(0) + Fx(1, 5)*p(1) + Fx(2, 5)*p(2) + Fx(3, 5)*p(3) + Fx(5, 5)*p(5) + g_x(10, 5)*lambda(10) + g_x(11, 5)*lambda(11) + g_x(42, 5)*lambda(42) + l_x(5);
    sl_x(6) = Fx(0, 6)*p(0) + Fx(1, 6)*p(1) + Fx(2, 6)*p(2) + Fx(3, 6)*p(3) + Fx(5, 6)*p(5) + Fx(6, 6)*p(6) + g_x(12, 6)*lambda(12) + g_x(13, 6)*lambda(13) + l_x(6);

  }

  virtual void updateLU(const Eigen::VectorXd& l_u, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& g_u, const Eigen::MatrixXd& h_u, const Eigen::VectorXd& p, const Eigen::VectorXd& lambda, const Eigen::VectorXd& eta, Eigen::VectorXd& sl_u) const override {
    // Evaluation of Vector sl_u
    sl_u(0) = Fu(0, 0)*p(0) + Fu(1, 0)*p(1) + Fu(2, 0)*p(2) + Fu(3, 0)*p(3) + Fu(4, 0)*p(4) + g_u(15, 0)*lambda(15) + g_u(16, 0)*lambda(16) + l_u(0);
    sl_u(1) = Fu(0, 1)*p(0) + Fu(1, 1)*p(1) + Fu(2, 1)*p(2) + Fu(3, 1)*p(3) + Fu(5, 1)*p(5) + Fu(6, 1)*p(6) + g_u(17, 1)*lambda(17) + g_u(18, 1)*lambda(18) + l_u(1);
    sl_u(2) = g_u(35, 2)*lambda(35) + g_u(36, 2)*lambda(36) + g_u(37, 2)*lambda(37) + g_u(38, 2)*lambda(38) + g_u(39, 2)*lambda(39) + g_u(40, 2)*lambda(40) + l_u(2);
    sl_u(3) = g_u(41, 3)*lambda(41) + l_u(3);
    sl_u(4) = g_u(42, 4)*lambda(42) + l_u(4);
    sl_u(5) = g_u(19, 5)*lambda(19) + g_u(27, 5)*lambda(27) + l_u(5);
    sl_u(6) = g_u(20, 6)*lambda(20) + g_u(28, 6)*lambda(28) + l_u(6);
    sl_u(7) = g_u(21, 7)*lambda(21) + g_u(29, 7)*lambda(29) + l_u(7);
    sl_u(8) = g_u(22, 8)*lambda(22) + g_u(30, 8)*lambda(30) + l_u(8);
    sl_u(9) = g_u(23, 9)*lambda(23) + g_u(31, 9)*lambda(31) + l_u(9);
    sl_u(10) = g_u(24, 10)*lambda(24) + g_u(32, 10)*lambda(32) + l_u(10);
    sl_u(11) = g_u(25, 11)*lambda(25) + g_u(33, 11)*lambda(33) + l_u(11);
    sl_u(12) = g_u(26, 12)*lambda(26) + g_u(34, 12)*lambda(34) + l_u(12);

  }

  virtual void updateQH(const Eigen::MatrixXd& l_xx, const Eigen::MatrixXd& l_xu, const Eigen::MatrixXd& l_uu, const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& V_xx_prev) override {
    // Determine internal variables
    const double internal_0 = Fx(0, 1)*V_xx_prev(0, 0);
    const double internal_1 = Fx(0, 0)*V_xx_prev(0, 1);
    const double internal_10 = Fx(0, 6)*V_xx_prev(0, 0);
    const double internal_103 = Fu(0, 0)*V_xx_prev(0, 0);
    const double internal_104 = Fu(0, 1)*V_xx_prev(0, 0);
    const double internal_11 = Fx(0, 0)*V_xx_prev(0, 6);
    const double internal_15 = Sigma_(19)*g_x(19, 1);
    const double internal_16 = Sigma_(2)*g_x(2, 1);
    const double internal_17 = Sigma_(20)*g_x(20, 1);
    const double internal_18 = Sigma_(21)*g_x(21, 1);
    const double internal_19 = Sigma_(22)*g_x(22, 1);
    const double internal_2 = Fx(0, 2)*V_xx_prev(0, 0);
    const double internal_20 = Sigma_(23)*g_x(23, 1);
    const double internal_21 = Sigma_(24)*g_x(24, 1);
    const double internal_22 = Sigma_(25)*g_x(25, 1);
    const double internal_23 = Sigma_(26)*g_x(26, 1);
    const double internal_24 = Sigma_(27)*g_x(27, 1);
    const double internal_25 = Sigma_(28)*g_x(28, 1);
    const double internal_26 = Sigma_(29)*g_x(29, 1);
    const double internal_27 = Sigma_(3)*g_x(3, 1);
    const double internal_28 = Sigma_(30)*g_x(30, 1);
    const double internal_29 = Sigma_(31)*g_x(31, 1);
    const double internal_3 = Fx(0, 0)*V_xx_prev(0, 2);
    const double internal_30 = Sigma_(32)*g_x(32, 1);
    const double internal_31 = Sigma_(33)*g_x(33, 1);
    const double internal_32 = Sigma_(34)*g_x(34, 1);
    const double internal_33 = Sigma_(35)*g_x(35, 1);
    const double internal_34 = Sigma_(36)*g_x(36, 1);
    const double internal_35 = Sigma_(37)*g_x(37, 1);
    const double internal_36 = Sigma_(38)*g_x(38, 1);
    const double internal_37 = Sigma_(39)*g_x(39, 1);
    const double internal_38 = Sigma_(4)*g_x(4, 1);
    const double internal_39 = Sigma_(40)*g_x(40, 1);
    const double internal_4 = Fx(0, 3)*V_xx_prev(0, 0);
    const double internal_40 = Sigma_(5)*g_x(5, 1);
    const double internal_5 = Fx(0, 0)*V_xx_prev(0, 3);
    const double internal_51 = Sigma_(19)*g_x(19, 2);
    const double internal_52 = Sigma_(20)*g_x(20, 2);
    const double internal_53 = Sigma_(21)*g_x(21, 2);
    const double internal_54 = Sigma_(22)*g_x(22, 2);
    const double internal_55 = Sigma_(23)*g_x(23, 2);
    const double internal_56 = Sigma_(24)*g_x(24, 2);
    const double internal_57 = Sigma_(25)*g_x(25, 2);
    const double internal_58 = Sigma_(26)*g_x(26, 2);
    const double internal_59 = Sigma_(27)*g_x(27, 2);
    const double internal_6 = Fx(0, 4)*V_xx_prev(0, 0);
    const double internal_60 = Sigma_(28)*g_x(28, 2);
    const double internal_61 = Sigma_(29)*g_x(29, 2);
    const double internal_62 = Sigma_(30)*g_x(30, 2);
    const double internal_63 = Sigma_(31)*g_x(31, 2);
    const double internal_64 = Sigma_(32)*g_x(32, 2);
    const double internal_65 = Sigma_(33)*g_x(33, 2);
    const double internal_66 = Sigma_(34)*g_x(34, 2);
    const double internal_67 = Sigma_(37)*g_x(37, 2);
    const double internal_68 = Sigma_(38)*g_x(38, 2);
    const double internal_69 = Sigma_(39)*g_x(39, 2);
    const double internal_7 = Fx(0, 0)*V_xx_prev(0, 4);
    const double internal_70 = Sigma_(40)*g_x(40, 2);
    const double internal_8 = Fx(0, 5)*V_xx_prev(0, 0);
    const double internal_9 = Fx(0, 0)*V_xx_prev(0, 5);
    const double internal_100 = Fx(0, 6)*V_xx_prev(0, 4) + Fx(1, 6)*V_xx_prev(1, 4) + Fx(2, 6)*V_xx_prev(2, 4) + Fx(3, 6)*V_xx_prev(3, 4) + Fx(5, 6)*V_xx_prev(5, 4) + Fx(6, 6)*V_xx_prev(6, 4);
    const double internal_101 = Fx(0, 6)*V_xx_prev(0, 5) + Fx(1, 6)*V_xx_prev(1, 5) + Fx(2, 6)*V_xx_prev(2, 5) + Fx(3, 6)*V_xx_prev(3, 5) + Fx(5, 6)*V_xx_prev(5, 5) + Fx(6, 6)*V_xx_prev(6, 5);
    const double internal_102 = Fx(0, 6)*V_xx_prev(0, 6) + Fx(1, 6)*V_xx_prev(1, 6) + Fx(2, 6)*V_xx_prev(2, 6) + Fx(3, 6)*V_xx_prev(3, 6) + Fx(5, 6)*V_xx_prev(5, 6) + Fx(6, 6)*V_xx_prev(6, 6);
    const double internal_106 = Fu(0, 0)*V_xx_prev(0, 1) + Fu(1, 0)*V_xx_prev(1, 1) + Fu(2, 0)*V_xx_prev(2, 1) + Fu(3, 0)*V_xx_prev(3, 1) + Fu(4, 0)*V_xx_prev(4, 1);
    const double internal_107 = Fu(0, 0)*V_xx_prev(0, 2) + Fu(1, 0)*V_xx_prev(1, 2) + Fu(2, 0)*V_xx_prev(2, 2) + Fu(3, 0)*V_xx_prev(3, 2) + Fu(4, 0)*V_xx_prev(4, 2);
    const double internal_108 = Fu(0, 0)*V_xx_prev(0, 3) + Fu(1, 0)*V_xx_prev(1, 3) + Fu(2, 0)*V_xx_prev(2, 3) + Fu(3, 0)*V_xx_prev(3, 3) + Fu(4, 0)*V_xx_prev(4, 3);
    const double internal_110 = Fu(0, 1)*V_xx_prev(0, 1) + Fu(1, 1)*V_xx_prev(1, 1) + Fu(2, 1)*V_xx_prev(2, 1) + Fu(3, 1)*V_xx_prev(3, 1) + Fu(5, 1)*V_xx_prev(5, 1) + Fu(6, 1)*V_xx_prev(6, 1);
    const double internal_111 = Fu(0, 1)*V_xx_prev(0, 2) + Fu(1, 1)*V_xx_prev(1, 2) + Fu(2, 1)*V_xx_prev(2, 2) + Fu(3, 1)*V_xx_prev(3, 2) + Fu(5, 1)*V_xx_prev(5, 2) + Fu(6, 1)*V_xx_prev(6, 2);
    const double internal_112 = Fu(0, 1)*V_xx_prev(0, 3) + Fu(1, 1)*V_xx_prev(1, 3) + Fu(2, 1)*V_xx_prev(2, 3) + Fu(3, 1)*V_xx_prev(3, 3) + Fu(5, 1)*V_xx_prev(5, 3) + Fu(6, 1)*V_xx_prev(6, 3);
    const double internal_13 = Fx(0, 1)*V_xx_prev(0, 1) + Fx(1, 1)*V_xx_prev(1, 1);
    const double internal_14 = Fx(0, 1)*V_xx_prev(0, 2) + Fx(1, 1)*V_xx_prev(1, 2);
    const double internal_42 = Fx(0, 1)*V_xx_prev(0, 3) + Fx(1, 1)*V_xx_prev(1, 3);
    const double internal_44 = Fx(0, 1)*V_xx_prev(0, 4) + Fx(1, 1)*V_xx_prev(1, 4);
    const double internal_45 = Fx(0, 1)*V_xx_prev(0, 5) + Fx(1, 1)*V_xx_prev(1, 5);
    const double internal_46 = Fx(0, 1)*V_xx_prev(0, 6) + Fx(1, 1)*V_xx_prev(1, 6);
    const double internal_48 = Fx(0, 2)*V_xx_prev(0, 1) + Fx(2, 2)*V_xx_prev(2, 1);
    const double internal_49 = Fx(0, 2)*V_xx_prev(0, 2) + Fx(2, 2)*V_xx_prev(2, 2);
    const double internal_50 = Fx(0, 2)*V_xx_prev(0, 3) + Fx(2, 2)*V_xx_prev(2, 3);
    const double internal_72 = Fx(0, 2)*V_xx_prev(0, 4) + Fx(2, 2)*V_xx_prev(2, 4);
    const double internal_73 = Fx(0, 2)*V_xx_prev(0, 5) + Fx(2, 2)*V_xx_prev(2, 5);
    const double internal_74 = Fx(0, 2)*V_xx_prev(0, 6) + Fx(2, 2)*V_xx_prev(2, 6);
    const double internal_76 = Fx(0, 3)*V_xx_prev(0, 1) + Fx(1, 3)*V_xx_prev(1, 1) + Fx(2, 3)*V_xx_prev(2, 1) + Fx(3, 3)*V_xx_prev(3, 1);
    const double internal_77 = Fx(0, 3)*V_xx_prev(0, 2) + Fx(1, 3)*V_xx_prev(1, 2) + Fx(2, 3)*V_xx_prev(2, 2) + Fx(3, 3)*V_xx_prev(3, 2);
    const double internal_78 = Fx(0, 3)*V_xx_prev(0, 3) + Fx(1, 3)*V_xx_prev(1, 3) + Fx(2, 3)*V_xx_prev(2, 3) + Fx(3, 3)*V_xx_prev(3, 3);
    const double internal_79 = Fx(0, 3)*V_xx_prev(0, 4) + Fx(1, 3)*V_xx_prev(1, 4) + Fx(2, 3)*V_xx_prev(2, 4) + Fx(3, 3)*V_xx_prev(3, 4);
    const double internal_80 = Fx(0, 3)*V_xx_prev(0, 5) + Fx(1, 3)*V_xx_prev(1, 5) + Fx(2, 3)*V_xx_prev(2, 5) + Fx(3, 3)*V_xx_prev(3, 5);
    const double internal_81 = Fx(0, 3)*V_xx_prev(0, 6) + Fx(1, 3)*V_xx_prev(1, 6) + Fx(2, 3)*V_xx_prev(2, 6) + Fx(3, 3)*V_xx_prev(3, 6);
    const double internal_83 = Fx(0, 4)*V_xx_prev(0, 1) + Fx(1, 4)*V_xx_prev(1, 1) + Fx(2, 4)*V_xx_prev(2, 1) + Fx(3, 4)*V_xx_prev(3, 1) + Fx(4, 4)*V_xx_prev(4, 1);
    const double internal_84 = Fx(0, 4)*V_xx_prev(0, 2) + Fx(1, 4)*V_xx_prev(1, 2) + Fx(2, 4)*V_xx_prev(2, 2) + Fx(3, 4)*V_xx_prev(3, 2) + Fx(4, 4)*V_xx_prev(4, 2);
    const double internal_85 = Fx(0, 4)*V_xx_prev(0, 3) + Fx(1, 4)*V_xx_prev(1, 3) + Fx(2, 4)*V_xx_prev(2, 3) + Fx(3, 4)*V_xx_prev(3, 3) + Fx(4, 4)*V_xx_prev(4, 3);
    const double internal_86 = Fx(0, 4)*V_xx_prev(0, 4) + Fx(1, 4)*V_xx_prev(1, 4) + Fx(2, 4)*V_xx_prev(2, 4) + Fx(3, 4)*V_xx_prev(3, 4) + Fx(4, 4)*V_xx_prev(4, 4);
    const double internal_87 = Fx(0, 4)*V_xx_prev(0, 5) + Fx(1, 4)*V_xx_prev(1, 5) + Fx(2, 4)*V_xx_prev(2, 5) + Fx(3, 4)*V_xx_prev(3, 5) + Fx(4, 4)*V_xx_prev(4, 5);
    const double internal_88 = Fx(0, 4)*V_xx_prev(0, 6) + Fx(1, 4)*V_xx_prev(1, 6) + Fx(2, 4)*V_xx_prev(2, 6) + Fx(3, 4)*V_xx_prev(3, 6) + Fx(4, 4)*V_xx_prev(4, 6);
    const double internal_90 = Fx(0, 5)*V_xx_prev(0, 1) + Fx(1, 5)*V_xx_prev(1, 1) + Fx(2, 5)*V_xx_prev(2, 1) + Fx(3, 5)*V_xx_prev(3, 1) + Fx(5, 5)*V_xx_prev(5, 1);
    const double internal_91 = Fx(0, 5)*V_xx_prev(0, 2) + Fx(1, 5)*V_xx_prev(1, 2) + Fx(2, 5)*V_xx_prev(2, 2) + Fx(3, 5)*V_xx_prev(3, 2) + Fx(5, 5)*V_xx_prev(5, 2);
    const double internal_92 = Fx(0, 5)*V_xx_prev(0, 3) + Fx(1, 5)*V_xx_prev(1, 3) + Fx(2, 5)*V_xx_prev(2, 3) + Fx(3, 5)*V_xx_prev(3, 3) + Fx(5, 5)*V_xx_prev(5, 3);
    const double internal_93 = Fx(0, 5)*V_xx_prev(0, 4) + Fx(1, 5)*V_xx_prev(1, 4) + Fx(2, 5)*V_xx_prev(2, 4) + Fx(3, 5)*V_xx_prev(3, 4) + Fx(5, 5)*V_xx_prev(5, 4);
    const double internal_94 = Fx(0, 5)*V_xx_prev(0, 5) + Fx(1, 5)*V_xx_prev(1, 5) + Fx(2, 5)*V_xx_prev(2, 5) + Fx(3, 5)*V_xx_prev(3, 5) + Fx(5, 5)*V_xx_prev(5, 5);
    const double internal_95 = Fx(0, 5)*V_xx_prev(0, 6) + Fx(1, 5)*V_xx_prev(1, 6) + Fx(2, 5)*V_xx_prev(2, 6) + Fx(3, 5)*V_xx_prev(3, 6) + Fx(5, 5)*V_xx_prev(5, 6);
    const double internal_97 = Fx(0, 6)*V_xx_prev(0, 1) + Fx(1, 6)*V_xx_prev(1, 1) + Fx(2, 6)*V_xx_prev(2, 1) + Fx(3, 6)*V_xx_prev(3, 1) + Fx(5, 6)*V_xx_prev(5, 1) + Fx(6, 6)*V_xx_prev(6, 1);
    const double internal_98 = Fx(0, 6)*V_xx_prev(0, 2) + Fx(1, 6)*V_xx_prev(1, 2) + Fx(2, 6)*V_xx_prev(2, 2) + Fx(3, 6)*V_xx_prev(3, 2) + Fx(5, 6)*V_xx_prev(5, 2) + Fx(6, 6)*V_xx_prev(6, 2);
    const double internal_99 = Fx(0, 6)*V_xx_prev(0, 3) + Fx(1, 6)*V_xx_prev(1, 3) + Fx(2, 6)*V_xx_prev(2, 3) + Fx(3, 6)*V_xx_prev(3, 3) + Fx(5, 6)*V_xx_prev(5, 3) + Fx(6, 6)*V_xx_prev(6, 3);
    const double internal_105 = Fu(1, 0)*V_xx_prev(1, 0) + Fu(2, 0)*V_xx_prev(2, 0) + Fu(3, 0)*V_xx_prev(3, 0) + Fu(4, 0)*V_xx_prev(4, 0) + internal_103;
    const double internal_109 = Fu(1, 1)*V_xx_prev(1, 0) + Fu(2, 1)*V_xx_prev(2, 0) + Fu(3, 1)*V_xx_prev(3, 0) + Fu(5, 1)*V_xx_prev(5, 0) + Fu(6, 1)*V_xx_prev(6, 0) + internal_104;
    const double internal_12 = Fx(1, 1)*V_xx_prev(1, 0) + internal_0;
    const double internal_47 = Fx(2, 2)*V_xx_prev(2, 0) + internal_2;
    const double internal_75 = Fx(1, 3)*V_xx_prev(1, 0) + Fx(2, 3)*V_xx_prev(2, 0) + Fx(3, 3)*V_xx_prev(3, 0) + internal_4;
    const double internal_82 = Fx(1, 4)*V_xx_prev(1, 0) + Fx(2, 4)*V_xx_prev(2, 0) + Fx(3, 4)*V_xx_prev(3, 0) + Fx(4, 4)*V_xx_prev(4, 0) + internal_6;
    const double internal_89 = Fx(1, 5)*V_xx_prev(1, 0) + Fx(2, 5)*V_xx_prev(2, 0) + Fx(3, 5)*V_xx_prev(3, 0) + Fx(5, 5)*V_xx_prev(5, 0) + internal_8;
    const double internal_96 = Fx(1, 6)*V_xx_prev(1, 0) + Fx(2, 6)*V_xx_prev(2, 0) + Fx(3, 6)*V_xx_prev(3, 0) + Fx(5, 6)*V_xx_prev(5, 0) + Fx(6, 6)*V_xx_prev(6, 0) + internal_10;
    const double internal_41 = Sigma_(0)*g_x(0, 1)*g_x(0, 2) + Sigma_(1)*g_x(1, 1)*g_x(1, 2) + g_x(19, 2)*internal_15 + g_x(2, 2)*internal_16 + g_x(20, 2)*internal_17 + g_x(21, 2)*internal_18 + g_x(22, 2)*internal_19 + g_x(23, 2)*internal_20 + g_x(24, 2)*internal_21 + g_x(25, 2)*internal_22 + g_x(26, 2)*internal_23 + g_x(27, 2)*internal_24 + g_x(28, 2)*internal_25 + g_x(29, 2)*internal_26 + g_x(3, 2)*internal_27 + g_x(30, 2)*internal_28 + g_x(31, 2)*internal_29 + g_x(32, 2)*internal_30 + g_x(33, 2)*internal_31 + g_x(34, 2)*internal_32 + g_x(35, 2)*internal_33 + g_x(36, 2)*internal_34 + g_x(37, 2)*internal_35 + g_x(38, 2)*internal_36 + g_x(39, 2)*internal_37 + g_x(4, 2)*internal_38 + g_x(40, 2)*internal_39 + g_x(5, 2)*internal_40;
    const double internal_43 = g_x(19, 3)*internal_15 + g_x(2, 3)*internal_16 + g_x(20, 3)*internal_17 + g_x(21, 3)*internal_18 + g_x(22, 3)*internal_19 + g_x(23, 3)*internal_20 + g_x(24, 3)*internal_21 + g_x(25, 3)*internal_22 + g_x(26, 3)*internal_23 + g_x(27, 3)*internal_24 + g_x(28, 3)*internal_25 + g_x(29, 3)*internal_26 + g_x(3, 3)*internal_27 + g_x(30, 3)*internal_28 + g_x(31, 3)*internal_29 + g_x(32, 3)*internal_30 + g_x(33, 3)*internal_31 + g_x(34, 3)*internal_32 + g_x(37, 3)*internal_35 + g_x(38, 3)*internal_36 + g_x(39, 3)*internal_37 + g_x(4, 3)*internal_38 + g_x(40, 3)*internal_39 + g_x(5, 3)*internal_40;
    const double internal_71 = Sigma_(2)*g_x(2, 2)*g_x(2, 3) + Sigma_(3)*g_x(3, 2)*g_x(3, 3) + Sigma_(4)*g_x(4, 2)*g_x(4, 3) + Sigma_(5)*g_x(5, 2)*g_x(5, 3) + g_x(19, 3)*internal_51 + g_x(20, 3)*internal_52 + g_x(21, 3)*internal_53 + g_x(22, 3)*internal_54 + g_x(23, 3)*internal_55 + g_x(24, 3)*internal_56 + g_x(25, 3)*internal_57 + g_x(26, 3)*internal_58 + g_x(27, 3)*internal_59 + g_x(28, 3)*internal_60 + g_x(29, 3)*internal_61 + g_x(30, 3)*internal_62 + g_x(31, 3)*internal_63 + g_x(32, 3)*internal_64 + g_x(33, 3)*internal_65 + g_x(34, 3)*internal_66 + g_x(37, 3)*internal_67 + g_x(38, 3)*internal_68 + g_x(39, 3)*internal_69 + g_x(40, 3)*internal_70;

    // Evaluation of Matrix Q_xx_
    Q_xx_(0, 0) = pow(Fx(0, 0), 2)*V_xx_prev(0, 0) + Sigma_(41)*pow(g_x(41, 0), 2) + Sigma_(8)*pow(g_x(8, 0), 2) + Sigma_(9)*pow(g_x(9, 0), 2) + l_xx(0, 0);
    Q_xx_(0, 1) = Fx(0, 0)*internal_0 + Fx(1, 1)*internal_1;
    Q_xx_(0, 2) = Fx(0, 0)*internal_2 + Fx(2, 2)*internal_3;
    Q_xx_(0, 3) = Fx(0, 0)*internal_4 + Fx(1, 3)*internal_1 + Fx(2, 3)*internal_3 + Fx(3, 3)*internal_5;
    Q_xx_(0, 4) = Fx(0, 0)*internal_6 + Fx(1, 4)*internal_1 + Fx(2, 4)*internal_3 + Fx(3, 4)*internal_5 + Fx(4, 4)*internal_7;
    Q_xx_(0, 5) = Fx(0, 0)*internal_8 + Fx(1, 5)*internal_1 + Fx(2, 5)*internal_3 + Fx(3, 5)*internal_5 + Fx(5, 5)*internal_9;
    Q_xx_(0, 6) = Fx(0, 0)*internal_10 + Fx(1, 6)*internal_1 + Fx(2, 6)*internal_3 + Fx(3, 6)*internal_5 + Fx(5, 6)*internal_9 + Fx(6, 6)*internal_11;
    Q_xx_(1, 0) = Fx(0, 0)*internal_12;
    Q_xx_(1, 1) = Fx(0, 1)*internal_12 + Fx(1, 1)*internal_13 + Sigma_(0)*pow(g_x(0, 1), 2) + Sigma_(1)*pow(g_x(1, 1), 2) + Sigma_(19)*pow(g_x(19, 1), 2) + Sigma_(2)*pow(g_x(2, 1), 2) + Sigma_(20)*pow(g_x(20, 1), 2) + Sigma_(21)*pow(g_x(21, 1), 2) + Sigma_(22)*pow(g_x(22, 1), 2) + Sigma_(23)*pow(g_x(23, 1), 2) + Sigma_(24)*pow(g_x(24, 1), 2) + Sigma_(25)*pow(g_x(25, 1), 2) + Sigma_(26)*pow(g_x(26, 1), 2) + Sigma_(27)*pow(g_x(27, 1), 2) + Sigma_(28)*pow(g_x(28, 1), 2) + Sigma_(29)*pow(g_x(29, 1), 2) + Sigma_(3)*pow(g_x(3, 1), 2) + Sigma_(30)*pow(g_x(30, 1), 2) + Sigma_(31)*pow(g_x(31, 1), 2) + Sigma_(32)*pow(g_x(32, 1), 2) + Sigma_(33)*pow(g_x(33, 1), 2) + Sigma_(34)*pow(g_x(34, 1), 2) + Sigma_(35)*pow(g_x(35, 1), 2) + Sigma_(36)*pow(g_x(36, 1), 2) + Sigma_(37)*pow(g_x(37, 1), 2) + Sigma_(38)*pow(g_x(38, 1), 2) + Sigma_(39)*pow(g_x(39, 1), 2) + Sigma_(4)*pow(g_x(4, 1), 2) + Sigma_(40)*pow(g_x(40, 1), 2) + Sigma_(5)*pow(g_x(5, 1), 2) + l_xx(1, 1);
    Q_xx_(1, 2) = Fx(0, 2)*internal_12 + Fx(2, 2)*internal_14 + internal_41 + l_xx(1, 2);
    Q_xx_(1, 3) = Fx(0, 3)*internal_12 + Fx(1, 3)*internal_13 + Fx(2, 3)*internal_14 + Fx(3, 3)*internal_42 + internal_43;
    Q_xx_(1, 4) = Fx(0, 4)*internal_12 + Fx(1, 4)*internal_13 + Fx(2, 4)*internal_14 + Fx(3, 4)*internal_42 + Fx(4, 4)*internal_44;
    Q_xx_(1, 5) = Fx(0, 5)*internal_12 + Fx(1, 5)*internal_13 + Fx(2, 5)*internal_14 + Fx(3, 5)*internal_42 + Fx(5, 5)*internal_45;
    Q_xx_(1, 6) = Fx(0, 6)*internal_12 + Fx(1, 6)*internal_13 + Fx(2, 6)*internal_14 + Fx(3, 6)*internal_42 + Fx(5, 6)*internal_45 + Fx(6, 6)*internal_46;
    Q_xx_(2, 0) = Fx(0, 0)*internal_47;
    Q_xx_(2, 1) = Fx(0, 1)*internal_47 + Fx(1, 1)*internal_48 + internal_41 + l_xx(2, 1);
    Q_xx_(2, 2) = Fx(0, 2)*internal_47 + Fx(2, 2)*internal_49 + Sigma_(0)*pow(g_x(0, 2), 2) + Sigma_(1)*pow(g_x(1, 2), 2) + Sigma_(19)*pow(g_x(19, 2), 2) + Sigma_(2)*pow(g_x(2, 2), 2) + Sigma_(20)*pow(g_x(20, 2), 2) + Sigma_(21)*pow(g_x(21, 2), 2) + Sigma_(22)*pow(g_x(22, 2), 2) + Sigma_(23)*pow(g_x(23, 2), 2) + Sigma_(24)*pow(g_x(24, 2), 2) + Sigma_(25)*pow(g_x(25, 2), 2) + Sigma_(26)*pow(g_x(26, 2), 2) + Sigma_(27)*pow(g_x(27, 2), 2) + Sigma_(28)*pow(g_x(28, 2), 2) + Sigma_(29)*pow(g_x(29, 2), 2) + Sigma_(3)*pow(g_x(3, 2), 2) + Sigma_(30)*pow(g_x(30, 2), 2) + Sigma_(31)*pow(g_x(31, 2), 2) + Sigma_(32)*pow(g_x(32, 2), 2) + Sigma_(33)*pow(g_x(33, 2), 2) + Sigma_(34)*pow(g_x(34, 2), 2) + Sigma_(35)*pow(g_x(35, 2), 2) + Sigma_(36)*pow(g_x(36, 2), 2) + Sigma_(37)*pow(g_x(37, 2), 2) + Sigma_(38)*pow(g_x(38, 2), 2) + Sigma_(39)*pow(g_x(39, 2), 2) + Sigma_(4)*pow(g_x(4, 2), 2) + Sigma_(40)*pow(g_x(40, 2), 2) + Sigma_(5)*pow(g_x(5, 2), 2) + l_xx(2, 2);
    Q_xx_(2, 3) = Fx(0, 3)*internal_47 + Fx(1, 3)*internal_48 + Fx(2, 3)*internal_49 + Fx(3, 3)*internal_50 + internal_71;
    Q_xx_(2, 4) = Fx(0, 4)*internal_47 + Fx(1, 4)*internal_48 + Fx(2, 4)*internal_49 + Fx(3, 4)*internal_50 + Fx(4, 4)*internal_72;
    Q_xx_(2, 5) = Fx(0, 5)*internal_47 + Fx(1, 5)*internal_48 + Fx(2, 5)*internal_49 + Fx(3, 5)*internal_50 + Fx(5, 5)*internal_73;
    Q_xx_(2, 6) = Fx(0, 6)*internal_47 + Fx(1, 6)*internal_48 + Fx(2, 6)*internal_49 + Fx(3, 6)*internal_50 + Fx(5, 6)*internal_73 + Fx(6, 6)*internal_74;
    Q_xx_(3, 0) = Fx(0, 0)*internal_75;
    Q_xx_(3, 1) = Fx(0, 1)*internal_75 + Fx(1, 1)*internal_76 + internal_43;
    Q_xx_(3, 2) = Fx(0, 2)*internal_75 + Fx(2, 2)*internal_77 + internal_71;
    Q_xx_(3, 3) = Fx(0, 3)*internal_75 + Fx(1, 3)*internal_76 + Fx(2, 3)*internal_77 + Fx(3, 3)*internal_78 + Sigma_(14)*pow(g_x(14, 3), 2) + Sigma_(19)*pow(g_x(19, 3), 2) + Sigma_(2)*pow(g_x(2, 3), 2) + Sigma_(20)*pow(g_x(20, 3), 2) + Sigma_(21)*pow(g_x(21, 3), 2) + Sigma_(22)*pow(g_x(22, 3), 2) + Sigma_(23)*pow(g_x(23, 3), 2) + Sigma_(24)*pow(g_x(24, 3), 2) + Sigma_(25)*pow(g_x(25, 3), 2) + Sigma_(26)*pow(g_x(26, 3), 2) + Sigma_(27)*pow(g_x(27, 3), 2) + Sigma_(28)*pow(g_x(28, 3), 2) + Sigma_(29)*pow(g_x(29, 3), 2) + Sigma_(3)*pow(g_x(3, 3), 2) + Sigma_(30)*pow(g_x(30, 3), 2) + Sigma_(31)*pow(g_x(31, 3), 2) + Sigma_(32)*pow(g_x(32, 3), 2) + Sigma_(33)*pow(g_x(33, 3), 2) + Sigma_(34)*pow(g_x(34, 3), 2) + Sigma_(37)*pow(g_x(37, 3), 2) + Sigma_(38)*pow(g_x(38, 3), 2) + Sigma_(39)*pow(g_x(39, 3), 2) + Sigma_(4)*pow(g_x(4, 3), 2) + Sigma_(40)*pow(g_x(40, 3), 2) + Sigma_(5)*pow(g_x(5, 3), 2) + l_xx(3, 3);
    Q_xx_(3, 4) = Fx(0, 4)*internal_75 + Fx(1, 4)*internal_76 + Fx(2, 4)*internal_77 + Fx(3, 4)*internal_78 + Fx(4, 4)*internal_79;
    Q_xx_(3, 5) = Fx(0, 5)*internal_75 + Fx(1, 5)*internal_76 + Fx(2, 5)*internal_77 + Fx(3, 5)*internal_78 + Fx(5, 5)*internal_80;
    Q_xx_(3, 6) = Fx(0, 6)*internal_75 + Fx(1, 6)*internal_76 + Fx(2, 6)*internal_77 + Fx(3, 6)*internal_78 + Fx(5, 6)*internal_80 + Fx(6, 6)*internal_81;
    Q_xx_(4, 0) = Fx(0, 0)*internal_82;
    Q_xx_(4, 1) = Fx(0, 1)*internal_82 + Fx(1, 1)*internal_83;
    Q_xx_(4, 2) = Fx(0, 2)*internal_82 + Fx(2, 2)*internal_84;
    Q_xx_(4, 3) = Fx(0, 3)*internal_82 + Fx(1, 3)*internal_83 + Fx(2, 3)*internal_84 + Fx(3, 3)*internal_85;
    Q_xx_(4, 4) = Fx(0, 4)*internal_82 + Fx(1, 4)*internal_83 + Fx(2, 4)*internal_84 + Fx(3, 4)*internal_85 + Fx(4, 4)*internal_86 + Sigma_(6)*pow(g_x(6, 4), 2) + Sigma_(7)*pow(g_x(7, 4), 2) + l_xx(4, 4);
    Q_xx_(4, 5) = Fx(0, 5)*internal_82 + Fx(1, 5)*internal_83 + Fx(2, 5)*internal_84 + Fx(3, 5)*internal_85 + Fx(5, 5)*internal_87;
    Q_xx_(4, 6) = Fx(0, 6)*internal_82 + Fx(1, 6)*internal_83 + Fx(2, 6)*internal_84 + Fx(3, 6)*internal_85 + Fx(5, 6)*internal_87 + Fx(6, 6)*internal_88;
    Q_xx_(5, 0) = Fx(0, 0)*internal_89;
    Q_xx_(5, 1) = Fx(0, 1)*internal_89 + Fx(1, 1)*internal_90;
    Q_xx_(5, 2) = Fx(0, 2)*internal_89 + Fx(2, 2)*internal_91;
    Q_xx_(5, 3) = Fx(0, 3)*internal_89 + Fx(1, 3)*internal_90 + Fx(2, 3)*internal_91 + Fx(3, 3)*internal_92;
    Q_xx_(5, 4) = Fx(0, 4)*internal_89 + Fx(1, 4)*internal_90 + Fx(2, 4)*internal_91 + Fx(3, 4)*internal_92 + Fx(4, 4)*internal_93;
    Q_xx_(5, 5) = Fx(0, 5)*internal_89 + Fx(1, 5)*internal_90 + Fx(2, 5)*internal_91 + Fx(3, 5)*internal_92 + Fx(5, 5)*internal_94 + Sigma_(10)*pow(g_x(10, 5), 2) + Sigma_(11)*pow(g_x(11, 5), 2) + Sigma_(42)*pow(g_x(42, 5), 2) + l_xx(5, 5);
    Q_xx_(5, 6) = Fx(0, 6)*internal_89 + Fx(1, 6)*internal_90 + Fx(2, 6)*internal_91 + Fx(3, 6)*internal_92 + Fx(5, 6)*internal_94 + Fx(6, 6)*internal_95;
    Q_xx_(6, 0) = Fx(0, 0)*internal_96;
    Q_xx_(6, 1) = Fx(0, 1)*internal_96 + Fx(1, 1)*internal_97;
    Q_xx_(6, 2) = Fx(0, 2)*internal_96 + Fx(2, 2)*internal_98;
    Q_xx_(6, 3) = Fx(0, 3)*internal_96 + Fx(1, 3)*internal_97 + Fx(2, 3)*internal_98 + Fx(3, 3)*internal_99;
    Q_xx_(6, 4) = Fx(0, 4)*internal_96 + Fx(1, 4)*internal_97 + Fx(2, 4)*internal_98 + Fx(3, 4)*internal_99 + Fx(4, 4)*internal_100;
    Q_xx_(6, 5) = Fx(0, 5)*internal_96 + Fx(1, 5)*internal_97 + Fx(2, 5)*internal_98 + Fx(3, 5)*internal_99 + Fx(5, 5)*internal_101;
    Q_xx_(6, 6) = Fx(0, 6)*internal_96 + Fx(1, 6)*internal_97 + Fx(2, 6)*internal_98 + Fx(3, 6)*internal_99 + Fx(5, 6)*internal_101 + Fx(6, 6)*internal_102 + Sigma_(12)*pow(g_x(12, 6), 2) + Sigma_(13)*pow(g_x(13, 6), 2) + l_xx(6, 6);

    // Evaluation of Matrix Q_xu_
    Q_xu_(0, 0) = Fu(1, 0)*internal_1 + Fu(2, 0)*internal_3 + Fu(3, 0)*internal_5 + Fu(4, 0)*internal_7 + Fx(0, 0)*internal_103;
    Q_xu_(0, 1) = Fu(1, 1)*internal_1 + Fu(2, 1)*internal_3 + Fu(3, 1)*internal_5 + Fu(5, 1)*internal_9 + Fu(6, 1)*internal_11 + Fx(0, 0)*internal_104;
    Q_xu_(0, 3) = Sigma_(41)*g_u(41, 3)*g_x(41, 0);
    Q_xu_(1, 0) = Fu(0, 0)*internal_12 + Fu(1, 0)*internal_13 + Fu(2, 0)*internal_14 + Fu(3, 0)*internal_42 + Fu(4, 0)*internal_44;
    Q_xu_(1, 1) = Fu(0, 1)*internal_12 + Fu(1, 1)*internal_13 + Fu(2, 1)*internal_14 + Fu(3, 1)*internal_42 + Fu(5, 1)*internal_45 + Fu(6, 1)*internal_46;
    Q_xu_(1, 2) = g_u(35, 2)*internal_33 + g_u(36, 2)*internal_34 + g_u(37, 2)*internal_35 + g_u(38, 2)*internal_36 + g_u(39, 2)*internal_37 + g_u(40, 2)*internal_39;
    Q_xu_(1, 5) = g_u(19, 5)*internal_15 + g_u(27, 5)*internal_24;
    Q_xu_(1, 6) = g_u(20, 6)*internal_17 + g_u(28, 6)*internal_25;
    Q_xu_(1, 7) = g_u(21, 7)*internal_18 + g_u(29, 7)*internal_26;
    Q_xu_(1, 8) = g_u(22, 8)*internal_19 + g_u(30, 8)*internal_28;
    Q_xu_(1, 9) = g_u(23, 9)*internal_20 + g_u(31, 9)*internal_29;
    Q_xu_(1, 10) = g_u(24, 10)*internal_21 + g_u(32, 10)*internal_30;
    Q_xu_(1, 11) = g_u(25, 11)*internal_22 + g_u(33, 11)*internal_31;
    Q_xu_(1, 12) = g_u(26, 12)*internal_23 + g_u(34, 12)*internal_32;
    Q_xu_(2, 0) = Fu(0, 0)*internal_47 + Fu(1, 0)*internal_48 + Fu(2, 0)*internal_49 + Fu(3, 0)*internal_50 + Fu(4, 0)*internal_72;
    Q_xu_(2, 1) = Fu(0, 1)*internal_47 + Fu(1, 1)*internal_48 + Fu(2, 1)*internal_49 + Fu(3, 1)*internal_50 + Fu(5, 1)*internal_73 + Fu(6, 1)*internal_74;
    Q_xu_(2, 2) = Sigma_(35)*g_u(35, 2)*g_x(35, 2) + Sigma_(36)*g_u(36, 2)*g_x(36, 2) + g_u(37, 2)*internal_67 + g_u(38, 2)*internal_68 + g_u(39, 2)*internal_69 + g_u(40, 2)*internal_70;
    Q_xu_(2, 5) = g_u(19, 5)*internal_51 + g_u(27, 5)*internal_59;
    Q_xu_(2, 6) = g_u(20, 6)*internal_52 + g_u(28, 6)*internal_60;
    Q_xu_(2, 7) = g_u(21, 7)*internal_53 + g_u(29, 7)*internal_61;
    Q_xu_(2, 8) = g_u(22, 8)*internal_54 + g_u(30, 8)*internal_62;
    Q_xu_(2, 9) = g_u(23, 9)*internal_55 + g_u(31, 9)*internal_63;
    Q_xu_(2, 10) = g_u(24, 10)*internal_56 + g_u(32, 10)*internal_64;
    Q_xu_(2, 11) = g_u(25, 11)*internal_57 + g_u(33, 11)*internal_65;
    Q_xu_(2, 12) = g_u(26, 12)*internal_58 + g_u(34, 12)*internal_66;
    Q_xu_(3, 0) = Fu(0, 0)*internal_75 + Fu(1, 0)*internal_76 + Fu(2, 0)*internal_77 + Fu(3, 0)*internal_78 + Fu(4, 0)*internal_79;
    Q_xu_(3, 1) = Fu(0, 1)*internal_75 + Fu(1, 1)*internal_76 + Fu(2, 1)*internal_77 + Fu(3, 1)*internal_78 + Fu(5, 1)*internal_80 + Fu(6, 1)*internal_81;
    Q_xu_(3, 2) = Sigma_(37)*g_u(37, 2)*g_x(37, 3) + Sigma_(38)*g_u(38, 2)*g_x(38, 3) + Sigma_(39)*g_u(39, 2)*g_x(39, 3) + Sigma_(40)*g_u(40, 2)*g_x(40, 3);
    Q_xu_(3, 5) = Sigma_(19)*g_u(19, 5)*g_x(19, 3) + Sigma_(27)*g_u(27, 5)*g_x(27, 3);
    Q_xu_(3, 6) = Sigma_(20)*g_u(20, 6)*g_x(20, 3) + Sigma_(28)*g_u(28, 6)*g_x(28, 3);
    Q_xu_(3, 7) = Sigma_(21)*g_u(21, 7)*g_x(21, 3) + Sigma_(29)*g_u(29, 7)*g_x(29, 3);
    Q_xu_(3, 8) = Sigma_(22)*g_u(22, 8)*g_x(22, 3) + Sigma_(30)*g_u(30, 8)*g_x(30, 3);
    Q_xu_(3, 9) = Sigma_(23)*g_u(23, 9)*g_x(23, 3) + Sigma_(31)*g_u(31, 9)*g_x(31, 3);
    Q_xu_(3, 10) = Sigma_(24)*g_u(24, 10)*g_x(24, 3) + Sigma_(32)*g_u(32, 10)*g_x(32, 3);
    Q_xu_(3, 11) = Sigma_(25)*g_u(25, 11)*g_x(25, 3) + Sigma_(33)*g_u(33, 11)*g_x(33, 3);
    Q_xu_(3, 12) = Sigma_(26)*g_u(26, 12)*g_x(26, 3) + Sigma_(34)*g_u(34, 12)*g_x(34, 3);
    Q_xu_(4, 0) = Fu(0, 0)*internal_82 + Fu(1, 0)*internal_83 + Fu(2, 0)*internal_84 + Fu(3, 0)*internal_85 + Fu(4, 0)*internal_86;
    Q_xu_(4, 1) = Fu(0, 1)*internal_82 + Fu(1, 1)*internal_83 + Fu(2, 1)*internal_84 + Fu(3, 1)*internal_85 + Fu(5, 1)*internal_87 + Fu(6, 1)*internal_88;
    Q_xu_(5, 0) = Fu(0, 0)*internal_89 + Fu(1, 0)*internal_90 + Fu(2, 0)*internal_91 + Fu(3, 0)*internal_92 + Fu(4, 0)*internal_93;
    Q_xu_(5, 1) = Fu(0, 1)*internal_89 + Fu(1, 1)*internal_90 + Fu(2, 1)*internal_91 + Fu(3, 1)*internal_92 + Fu(5, 1)*internal_94 + Fu(6, 1)*internal_95;
    Q_xu_(5, 4) = Sigma_(42)*g_u(42, 4)*g_x(42, 5);
    Q_xu_(6, 0) = Fu(0, 0)*internal_96 + Fu(1, 0)*internal_97 + Fu(2, 0)*internal_98 + Fu(3, 0)*internal_99 + Fu(4, 0)*internal_100;
    Q_xu_(6, 1) = Fu(0, 1)*internal_96 + Fu(1, 1)*internal_97 + Fu(2, 1)*internal_98 + Fu(3, 1)*internal_99 + Fu(5, 1)*internal_101 + Fu(6, 1)*internal_102;

    // Evaluation of Matrix Q_uu_
    Q_uu_(0, 0) = Fu(0, 0)*internal_105 + Fu(1, 0)*internal_106 + Fu(2, 0)*internal_107 + Fu(3, 0)*internal_108 + Fu(4, 0)*(Fu(0, 0)*V_xx_prev(0, 4) + Fu(1, 0)*V_xx_prev(1, 4) + Fu(2, 0)*V_xx_prev(2, 4) + Fu(3, 0)*V_xx_prev(3, 4) + Fu(4, 0)*V_xx_prev(4, 4)) + Sigma_(15)*pow(g_u(15, 0), 2) + Sigma_(16)*pow(g_u(16, 0), 2) + l_uu(0, 0);
    Q_uu_(0, 1) = Fu(0, 1)*internal_105 + Fu(1, 1)*internal_106 + Fu(2, 1)*internal_107 + Fu(3, 1)*internal_108 + Fu(5, 1)*(Fu(0, 0)*V_xx_prev(0, 5) + Fu(1, 0)*V_xx_prev(1, 5) + Fu(2, 0)*V_xx_prev(2, 5) + Fu(3, 0)*V_xx_prev(3, 5) + Fu(4, 0)*V_xx_prev(4, 5)) + Fu(6, 1)*(Fu(0, 0)*V_xx_prev(0, 6) + Fu(1, 0)*V_xx_prev(1, 6) + Fu(2, 0)*V_xx_prev(2, 6) + Fu(3, 0)*V_xx_prev(3, 6) + Fu(4, 0)*V_xx_prev(4, 6));
    Q_uu_(1, 0) = Fu(0, 0)*internal_109 + Fu(1, 0)*internal_110 + Fu(2, 0)*internal_111 + Fu(3, 0)*internal_112 + Fu(4, 0)*(Fu(0, 1)*V_xx_prev(0, 4) + Fu(1, 1)*V_xx_prev(1, 4) + Fu(2, 1)*V_xx_prev(2, 4) + Fu(3, 1)*V_xx_prev(3, 4) + Fu(5, 1)*V_xx_prev(5, 4) + Fu(6, 1)*V_xx_prev(6, 4));
    Q_uu_(1, 1) = Fu(0, 1)*internal_109 + Fu(1, 1)*internal_110 + Fu(2, 1)*internal_111 + Fu(3, 1)*internal_112 + Fu(5, 1)*(Fu(0, 1)*V_xx_prev(0, 5) + Fu(1, 1)*V_xx_prev(1, 5) + Fu(2, 1)*V_xx_prev(2, 5) + Fu(3, 1)*V_xx_prev(3, 5) + Fu(5, 1)*V_xx_prev(5, 5) + Fu(6, 1)*V_xx_prev(6, 5)) + Fu(6, 1)*(Fu(0, 1)*V_xx_prev(0, 6) + Fu(1, 1)*V_xx_prev(1, 6) + Fu(2, 1)*V_xx_prev(2, 6) + Fu(3, 1)*V_xx_prev(3, 6) + Fu(5, 1)*V_xx_prev(5, 6) + Fu(6, 1)*V_xx_prev(6, 6)) + Sigma_(17)*pow(g_u(17, 1), 2) + Sigma_(18)*pow(g_u(18, 1), 2) + l_uu(1, 1);
    Q_uu_(2, 2) = Sigma_(35)*pow(g_u(35, 2), 2) + Sigma_(36)*pow(g_u(36, 2), 2) + Sigma_(37)*pow(g_u(37, 2), 2) + Sigma_(38)*pow(g_u(38, 2), 2) + Sigma_(39)*pow(g_u(39, 2), 2) + Sigma_(40)*pow(g_u(40, 2), 2) + l_uu(2, 2);
    Q_uu_(3, 3) = Sigma_(41)*pow(g_u(41, 3), 2) + l_uu(3, 3);
    Q_uu_(4, 4) = Sigma_(42)*pow(g_u(42, 4), 2) + l_uu(4, 4);
    Q_uu_(5, 5) = Sigma_(19)*pow(g_u(19, 5), 2) + Sigma_(27)*pow(g_u(27, 5), 2) + l_uu(5, 5);
    Q_uu_(6, 6) = Sigma_(20)*pow(g_u(20, 6), 2) + Sigma_(28)*pow(g_u(28, 6), 2) + l_uu(6, 6);
    Q_uu_(7, 7) = Sigma_(21)*pow(g_u(21, 7), 2) + Sigma_(29)*pow(g_u(29, 7), 2) + l_uu(7, 7);
    Q_uu_(8, 8) = Sigma_(22)*pow(g_u(22, 8), 2) + Sigma_(30)*pow(g_u(30, 8), 2) + l_uu(8, 8);
    Q_uu_(9, 9) = Sigma_(23)*pow(g_u(23, 9), 2) + Sigma_(31)*pow(g_u(31, 9), 2) + l_uu(9, 9);
    Q_uu_(10, 10) = Sigma_(24)*pow(g_u(24, 10), 2) + Sigma_(32)*pow(g_u(32, 10), 2) + l_uu(10, 10);
    Q_uu_(11, 11) = Sigma_(25)*pow(g_u(25, 11), 2) + Sigma_(33)*pow(g_u(33, 11), 2) + l_uu(11, 11);
    Q_uu_(12, 12) = Sigma_(26)*pow(g_u(26, 12), 2) + Sigma_(34)*pow(g_u(34, 12), 2) + l_uu(12, 12);

  }

  virtual void updateQG(const Eigen::VectorXd& sl_x, const Eigen::VectorXd& sl_u, const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::VectorXd& r_f, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& V_xx_prev, const Eigen::VectorXd& V_x_prev) override {
    // Determine internal variables
    const double internal_0 = V_x_prev(0) + V_xx_prev(0, 0)*r_f(0) + V_xx_prev(0, 1)*r_f(1) + V_xx_prev(0, 2)*r_f(2) + V_xx_prev(0, 3)*r_f(3) + V_xx_prev(0, 4)*r_f(4) + V_xx_prev(0, 5)*r_f(5) + V_xx_prev(0, 6)*r_f(6);
    const double internal_1 = V_x_prev(1) + V_xx_prev(1, 0)*r_f(0) + V_xx_prev(1, 1)*r_f(1) + V_xx_prev(1, 2)*r_f(2) + V_xx_prev(1, 3)*r_f(3) + V_xx_prev(1, 4)*r_f(4) + V_xx_prev(1, 5)*r_f(5) + V_xx_prev(1, 6)*r_f(6);
    const double internal_2 = V_x_prev(2) + V_xx_prev(2, 0)*r_f(0) + V_xx_prev(2, 1)*r_f(1) + V_xx_prev(2, 2)*r_f(2) + V_xx_prev(2, 3)*r_f(3) + V_xx_prev(2, 4)*r_f(4) + V_xx_prev(2, 5)*r_f(5) + V_xx_prev(2, 6)*r_f(6);
    const double internal_3 = V_x_prev(3) + V_xx_prev(3, 0)*r_f(0) + V_xx_prev(3, 1)*r_f(1) + V_xx_prev(3, 2)*r_f(2) + V_xx_prev(3, 3)*r_f(3) + V_xx_prev(3, 4)*r_f(4) + V_xx_prev(3, 5)*r_f(5) + V_xx_prev(3, 6)*r_f(6);
    const double internal_4 = V_x_prev(4) + V_xx_prev(4, 0)*r_f(0) + V_xx_prev(4, 1)*r_f(1) + V_xx_prev(4, 2)*r_f(2) + V_xx_prev(4, 3)*r_f(3) + V_xx_prev(4, 4)*r_f(4) + V_xx_prev(4, 5)*r_f(5) + V_xx_prev(4, 6)*r_f(6);
    const double internal_5 = V_x_prev(5) + V_xx_prev(5, 0)*r_f(0) + V_xx_prev(5, 1)*r_f(1) + V_xx_prev(5, 2)*r_f(2) + V_xx_prev(5, 3)*r_f(3) + V_xx_prev(5, 4)*r_f(4) + V_xx_prev(5, 5)*r_f(5) + V_xx_prev(5, 6)*r_f(6);
    const double internal_6 = V_x_prev(6) + V_xx_prev(6, 0)*r_f(0) + V_xx_prev(6, 1)*r_f(1) + V_xx_prev(6, 2)*r_f(2) + V_xx_prev(6, 3)*r_f(3) + V_xx_prev(6, 4)*r_f(4) + V_xx_prev(6, 5)*r_f(5) + V_xx_prev(6, 6)*r_f(6);

    // Evaluation of Vector Q_x_
    Q_x_(0) = Fx(0, 0)*internal_0 + V_(41)*g_x(41, 0) + V_(8)*g_x(8, 0) + V_(9)*g_x(9, 0) + sl_x(0);
    Q_x_(1) = Fx(0, 1)*internal_0 + Fx(1, 1)*internal_1 + V_(0)*g_x(0, 1) + V_(1)*g_x(1, 1) + V_(19)*g_x(19, 1) + V_(2)*g_x(2, 1) + V_(20)*g_x(20, 1) + V_(21)*g_x(21, 1) + V_(22)*g_x(22, 1) + V_(23)*g_x(23, 1) + V_(24)*g_x(24, 1) + V_(25)*g_x(25, 1) + V_(26)*g_x(26, 1) + V_(27)*g_x(27, 1) + V_(28)*g_x(28, 1) + V_(29)*g_x(29, 1) + V_(3)*g_x(3, 1) + V_(30)*g_x(30, 1) + V_(31)*g_x(31, 1) + V_(32)*g_x(32, 1) + V_(33)*g_x(33, 1) + V_(34)*g_x(34, 1) + V_(35)*g_x(35, 1) + V_(36)*g_x(36, 1) + V_(37)*g_x(37, 1) + V_(38)*g_x(38, 1) + V_(39)*g_x(39, 1) + V_(4)*g_x(4, 1) + V_(40)*g_x(40, 1) + V_(5)*g_x(5, 1) + sl_x(1);
    Q_x_(2) = Fx(0, 2)*internal_0 + Fx(2, 2)*internal_2 + V_(0)*g_x(0, 2) + V_(1)*g_x(1, 2) + V_(19)*g_x(19, 2) + V_(2)*g_x(2, 2) + V_(20)*g_x(20, 2) + V_(21)*g_x(21, 2) + V_(22)*g_x(22, 2) + V_(23)*g_x(23, 2) + V_(24)*g_x(24, 2) + V_(25)*g_x(25, 2) + V_(26)*g_x(26, 2) + V_(27)*g_x(27, 2) + V_(28)*g_x(28, 2) + V_(29)*g_x(29, 2) + V_(3)*g_x(3, 2) + V_(30)*g_x(30, 2) + V_(31)*g_x(31, 2) + V_(32)*g_x(32, 2) + V_(33)*g_x(33, 2) + V_(34)*g_x(34, 2) + V_(35)*g_x(35, 2) + V_(36)*g_x(36, 2) + V_(37)*g_x(37, 2) + V_(38)*g_x(38, 2) + V_(39)*g_x(39, 2) + V_(4)*g_x(4, 2) + V_(40)*g_x(40, 2) + V_(5)*g_x(5, 2) + sl_x(2);
    Q_x_(3) = Fx(0, 3)*internal_0 + Fx(1, 3)*internal_1 + Fx(2, 3)*internal_2 + Fx(3, 3)*internal_3 + V_(14)*g_x(14, 3) + V_(19)*g_x(19, 3) + V_(2)*g_x(2, 3) + V_(20)*g_x(20, 3) + V_(21)*g_x(21, 3) + V_(22)*g_x(22, 3) + V_(23)*g_x(23, 3) + V_(24)*g_x(24, 3) + V_(25)*g_x(25, 3) + V_(26)*g_x(26, 3) + V_(27)*g_x(27, 3) + V_(28)*g_x(28, 3) + V_(29)*g_x(29, 3) + V_(3)*g_x(3, 3) + V_(30)*g_x(30, 3) + V_(31)*g_x(31, 3) + V_(32)*g_x(32, 3) + V_(33)*g_x(33, 3) + V_(34)*g_x(34, 3) + V_(37)*g_x(37, 3) + V_(38)*g_x(38, 3) + V_(39)*g_x(39, 3) + V_(4)*g_x(4, 3) + V_(40)*g_x(40, 3) + V_(5)*g_x(5, 3) + sl_x(3);
    Q_x_(4) = Fx(0, 4)*internal_0 + Fx(1, 4)*internal_1 + Fx(2, 4)*internal_2 + Fx(3, 4)*internal_3 + Fx(4, 4)*internal_4 + V_(6)*g_x(6, 4) + V_(7)*g_x(7, 4) + sl_x(4);
    Q_x_(5) = Fx(0, 5)*internal_0 + Fx(1, 5)*internal_1 + Fx(2, 5)*internal_2 + Fx(3, 5)*internal_3 + Fx(5, 5)*internal_5 + V_(10)*g_x(10, 5) + V_(11)*g_x(11, 5) + V_(42)*g_x(42, 5) + sl_x(5);
    Q_x_(6) = Fx(0, 6)*internal_0 + Fx(1, 6)*internal_1 + Fx(2, 6)*internal_2 + Fx(3, 6)*internal_3 + Fx(5, 6)*internal_5 + Fx(6, 6)*internal_6 + V_(12)*g_x(12, 6) + V_(13)*g_x(13, 6) + sl_x(6);

    // Evaluation of Vector Q_u_
    Q_u_(0) = Fu(0, 0)*internal_0 + Fu(1, 0)*internal_1 + Fu(2, 0)*internal_2 + Fu(3, 0)*internal_3 + Fu(4, 0)*internal_4 + V_(15)*g_u(15, 0) + V_(16)*g_u(16, 0) + sl_u(0);
    Q_u_(1) = Fu(0, 1)*internal_0 + Fu(1, 1)*internal_1 + Fu(2, 1)*internal_2 + Fu(3, 1)*internal_3 + Fu(5, 1)*internal_5 + Fu(6, 1)*internal_6 + V_(17)*g_u(17, 1) + V_(18)*g_u(18, 1) + sl_u(1);
    Q_u_(2) = V_(35)*g_u(35, 2) + V_(36)*g_u(36, 2) + V_(37)*g_u(37, 2) + V_(38)*g_u(38, 2) + V_(39)*g_u(39, 2) + V_(40)*g_u(40, 2) + sl_u(2);
    Q_u_(3) = V_(41)*g_u(41, 3) + sl_u(3);
    Q_u_(4) = V_(42)*g_u(42, 4) + sl_u(4);
    Q_u_(5) = V_(19)*g_u(19, 5) + V_(27)*g_u(27, 5) + sl_u(5);
    Q_u_(6) = V_(20)*g_u(20, 6) + V_(28)*g_u(28, 6) + sl_u(6);
    Q_u_(7) = V_(21)*g_u(21, 7) + V_(29)*g_u(29, 7) + sl_u(7);
    Q_u_(8) = V_(22)*g_u(22, 8) + V_(30)*g_u(30, 8) + sl_u(8);
    Q_u_(9) = V_(23)*g_u(23, 9) + V_(31)*g_u(31, 9) + sl_u(9);
    Q_u_(10) = V_(24)*g_u(24, 10) + V_(32)*g_u(32, 10) + sl_u(10);
    Q_u_(11) = V_(25)*g_u(25, 11) + V_(33)*g_u(33, 11) + sl_u(11);
    Q_u_(12) = V_(26)*g_u(26, 12) + V_(34)*g_u(34, 12) + sl_u(12);

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
    Q_uu_inv_(3, 3) = 1.0/Q_uu_(3, 3);
    Q_uu_inv_(4, 4) = 1.0/Q_uu_(4, 4);
    Q_uu_inv_(5, 5) = 1.0/Q_uu_(5, 5);
    Q_uu_inv_(6, 6) = 1.0/Q_uu_(6, 6);
    Q_uu_inv_(7, 7) = 1.0/Q_uu_(7, 7);
    Q_uu_inv_(8, 8) = 1.0/Q_uu_(8, 8);
    Q_uu_inv_(9, 9) = 1.0/Q_uu_(9, 9);
    Q_uu_inv_(10, 10) = 1.0/Q_uu_(10, 10);
    Q_uu_inv_(11, 11) = 1.0/Q_uu_(11, 11);
    Q_uu_inv_(12, 12) = 1.0/Q_uu_(12, 12);

  }

  virtual void updateKVU(Eigen::MatrixXd& k_ux, Eigen::VectorXd& v_u) override {
    // Evaluation of Matrix k_ux
    k_ux(0, 0) = -Q_uu_inv_(0, 0)*Q_xu_(0, 0) - Q_uu_inv_(0, 1)*Q_xu_(0, 1);
    k_ux(0, 1) = -Q_uu_inv_(0, 0)*Q_xu_(1, 0) - Q_uu_inv_(0, 1)*Q_xu_(1, 1);
    k_ux(0, 2) = -Q_uu_inv_(0, 0)*Q_xu_(2, 0) - Q_uu_inv_(0, 1)*Q_xu_(2, 1);
    k_ux(0, 3) = -Q_uu_inv_(0, 0)*Q_xu_(3, 0) - Q_uu_inv_(0, 1)*Q_xu_(3, 1);
    k_ux(0, 4) = -Q_uu_inv_(0, 0)*Q_xu_(4, 0) - Q_uu_inv_(0, 1)*Q_xu_(4, 1);
    k_ux(0, 5) = -Q_uu_inv_(0, 0)*Q_xu_(5, 0) - Q_uu_inv_(0, 1)*Q_xu_(5, 1);
    k_ux(0, 6) = -Q_uu_inv_(0, 0)*Q_xu_(6, 0) - Q_uu_inv_(0, 1)*Q_xu_(6, 1);
    k_ux(1, 0) = -Q_uu_inv_(0, 1)*Q_xu_(0, 0) - Q_uu_inv_(1, 1)*Q_xu_(0, 1);
    k_ux(1, 1) = -Q_uu_inv_(0, 1)*Q_xu_(1, 0) - Q_uu_inv_(1, 1)*Q_xu_(1, 1);
    k_ux(1, 2) = -Q_uu_inv_(0, 1)*Q_xu_(2, 0) - Q_uu_inv_(1, 1)*Q_xu_(2, 1);
    k_ux(1, 3) = -Q_uu_inv_(0, 1)*Q_xu_(3, 0) - Q_uu_inv_(1, 1)*Q_xu_(3, 1);
    k_ux(1, 4) = -Q_uu_inv_(0, 1)*Q_xu_(4, 0) - Q_uu_inv_(1, 1)*Q_xu_(4, 1);
    k_ux(1, 5) = -Q_uu_inv_(0, 1)*Q_xu_(5, 0) - Q_uu_inv_(1, 1)*Q_xu_(5, 1);
    k_ux(1, 6) = -Q_uu_inv_(0, 1)*Q_xu_(6, 0) - Q_uu_inv_(1, 1)*Q_xu_(6, 1);
    k_ux(2, 1) = -Q_uu_inv_(2, 2)*Q_xu_(1, 2);
    k_ux(2, 2) = -Q_uu_inv_(2, 2)*Q_xu_(2, 2);
    k_ux(2, 3) = -Q_uu_inv_(2, 2)*Q_xu_(3, 2);
    k_ux(3, 0) = -Q_uu_inv_(3, 3)*Q_xu_(0, 3);
    k_ux(4, 5) = -Q_uu_inv_(4, 4)*Q_xu_(5, 4);
    k_ux(5, 1) = -Q_uu_inv_(5, 5)*Q_xu_(1, 5);
    k_ux(5, 2) = -Q_uu_inv_(5, 5)*Q_xu_(2, 5);
    k_ux(5, 3) = -Q_uu_inv_(5, 5)*Q_xu_(3, 5);
    k_ux(6, 1) = -Q_uu_inv_(6, 6)*Q_xu_(1, 6);
    k_ux(6, 2) = -Q_uu_inv_(6, 6)*Q_xu_(2, 6);
    k_ux(6, 3) = -Q_uu_inv_(6, 6)*Q_xu_(3, 6);
    k_ux(7, 1) = -Q_uu_inv_(7, 7)*Q_xu_(1, 7);
    k_ux(7, 2) = -Q_uu_inv_(7, 7)*Q_xu_(2, 7);
    k_ux(7, 3) = -Q_uu_inv_(7, 7)*Q_xu_(3, 7);
    k_ux(8, 1) = -Q_uu_inv_(8, 8)*Q_xu_(1, 8);
    k_ux(8, 2) = -Q_uu_inv_(8, 8)*Q_xu_(2, 8);
    k_ux(8, 3) = -Q_uu_inv_(8, 8)*Q_xu_(3, 8);
    k_ux(9, 1) = -Q_uu_inv_(9, 9)*Q_xu_(1, 9);
    k_ux(9, 2) = -Q_uu_inv_(9, 9)*Q_xu_(2, 9);
    k_ux(9, 3) = -Q_uu_inv_(9, 9)*Q_xu_(3, 9);
    k_ux(10, 1) = -Q_uu_inv_(10, 10)*Q_xu_(1, 10);
    k_ux(10, 2) = -Q_uu_inv_(10, 10)*Q_xu_(2, 10);
    k_ux(10, 3) = -Q_uu_inv_(10, 10)*Q_xu_(3, 10);
    k_ux(11, 1) = -Q_uu_inv_(11, 11)*Q_xu_(1, 11);
    k_ux(11, 2) = -Q_uu_inv_(11, 11)*Q_xu_(2, 11);
    k_ux(11, 3) = -Q_uu_inv_(11, 11)*Q_xu_(3, 11);
    k_ux(12, 1) = -Q_uu_inv_(12, 12)*Q_xu_(1, 12);
    k_ux(12, 2) = -Q_uu_inv_(12, 12)*Q_xu_(2, 12);
    k_ux(12, 3) = -Q_uu_inv_(12, 12)*Q_xu_(3, 12);

    // Evaluation of Vector v_u
    v_u(0) = -Q_u_(0)*Q_uu_inv_(0, 0) - Q_u_(1)*Q_uu_inv_(0, 1);
    v_u(1) = -Q_u_(0)*Q_uu_inv_(0, 1) - Q_u_(1)*Q_uu_inv_(1, 1);
    v_u(2) = -Q_u_(2)*Q_uu_inv_(2, 2);
    v_u(3) = -Q_u_(3)*Q_uu_inv_(3, 3);
    v_u(4) = -Q_u_(4)*Q_uu_inv_(4, 4);
    v_u(5) = -Q_u_(5)*Q_uu_inv_(5, 5);
    v_u(6) = -Q_u_(6)*Q_uu_inv_(6, 6);
    v_u(7) = -Q_u_(7)*Q_uu_inv_(7, 7);
    v_u(8) = -Q_u_(8)*Q_uu_inv_(8, 8);
    v_u(9) = -Q_u_(9)*Q_uu_inv_(9, 9);
    v_u(10) = -Q_u_(10)*Q_uu_inv_(10, 10);
    v_u(11) = -Q_u_(11)*Q_uu_inv_(11, 11);
    v_u(12) = -Q_u_(12)*Q_uu_inv_(12, 12);

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
    V_xx(0, 0) = Q_xu_(0, 0)*k_ux(0, 0) + Q_xu_(0, 1)*k_ux(1, 0) + Q_xu_(0, 3)*k_ux(3, 0) + Q_xx_(0, 0);
    V_xx(0, 1) = Q_xu_(0, 0)*k_ux(0, 1) + Q_xu_(0, 1)*k_ux(1, 1) + Q_xx_(0, 1);
    V_xx(0, 2) = Q_xu_(0, 0)*k_ux(0, 2) + Q_xu_(0, 1)*k_ux(1, 2) + Q_xx_(0, 2);
    V_xx(0, 3) = Q_xu_(0, 0)*k_ux(0, 3) + Q_xu_(0, 1)*k_ux(1, 3) + Q_xx_(0, 3);
    V_xx(0, 4) = Q_xu_(0, 0)*k_ux(0, 4) + Q_xu_(0, 1)*k_ux(1, 4) + Q_xx_(0, 4);
    V_xx(0, 5) = Q_xu_(0, 0)*k_ux(0, 5) + Q_xu_(0, 1)*k_ux(1, 5) + Q_xx_(0, 5);
    V_xx(0, 6) = Q_xu_(0, 0)*k_ux(0, 6) + Q_xu_(0, 1)*k_ux(1, 6) + Q_xx_(0, 6);
    V_xx(1, 0) = Q_xu_(1, 0)*k_ux(0, 0) + Q_xu_(1, 1)*k_ux(1, 0) + Q_xx_(0, 1);
    V_xx(1, 1) = Q_xu_(1, 0)*k_ux(0, 1) + Q_xu_(1, 1)*k_ux(1, 1) + Q_xu_(1, 10)*k_ux(10, 1) + Q_xu_(1, 11)*k_ux(11, 1) + Q_xu_(1, 12)*k_ux(12, 1) + Q_xu_(1, 2)*k_ux(2, 1) + Q_xu_(1, 5)*k_ux(5, 1) + Q_xu_(1, 6)*k_ux(6, 1) + Q_xu_(1, 7)*k_ux(7, 1) + Q_xu_(1, 8)*k_ux(8, 1) + Q_xu_(1, 9)*k_ux(9, 1) + Q_xx_(1, 1);
    V_xx(1, 2) = Q_xu_(1, 0)*k_ux(0, 2) + Q_xu_(1, 1)*k_ux(1, 2) + Q_xu_(1, 10)*k_ux(10, 2) + Q_xu_(1, 11)*k_ux(11, 2) + Q_xu_(1, 12)*k_ux(12, 2) + Q_xu_(1, 2)*k_ux(2, 2) + Q_xu_(1, 5)*k_ux(5, 2) + Q_xu_(1, 6)*k_ux(6, 2) + Q_xu_(1, 7)*k_ux(7, 2) + Q_xu_(1, 8)*k_ux(8, 2) + Q_xu_(1, 9)*k_ux(9, 2) + Q_xx_(1, 2);
    V_xx(1, 3) = Q_xu_(1, 0)*k_ux(0, 3) + Q_xu_(1, 1)*k_ux(1, 3) + Q_xu_(1, 10)*k_ux(10, 3) + Q_xu_(1, 11)*k_ux(11, 3) + Q_xu_(1, 12)*k_ux(12, 3) + Q_xu_(1, 2)*k_ux(2, 3) + Q_xu_(1, 5)*k_ux(5, 3) + Q_xu_(1, 6)*k_ux(6, 3) + Q_xu_(1, 7)*k_ux(7, 3) + Q_xu_(1, 8)*k_ux(8, 3) + Q_xu_(1, 9)*k_ux(9, 3) + Q_xx_(1, 3);
    V_xx(1, 4) = Q_xu_(1, 0)*k_ux(0, 4) + Q_xu_(1, 1)*k_ux(1, 4) + Q_xx_(1, 4);
    V_xx(1, 5) = Q_xu_(1, 0)*k_ux(0, 5) + Q_xu_(1, 1)*k_ux(1, 5) + Q_xx_(1, 5);
    V_xx(1, 6) = Q_xu_(1, 0)*k_ux(0, 6) + Q_xu_(1, 1)*k_ux(1, 6) + Q_xx_(1, 6);
    V_xx(2, 0) = Q_xu_(2, 0)*k_ux(0, 0) + Q_xu_(2, 1)*k_ux(1, 0) + Q_xx_(0, 2);
    V_xx(2, 1) = Q_xu_(2, 0)*k_ux(0, 1) + Q_xu_(2, 1)*k_ux(1, 1) + Q_xu_(2, 10)*k_ux(10, 1) + Q_xu_(2, 11)*k_ux(11, 1) + Q_xu_(2, 12)*k_ux(12, 1) + Q_xu_(2, 2)*k_ux(2, 1) + Q_xu_(2, 5)*k_ux(5, 1) + Q_xu_(2, 6)*k_ux(6, 1) + Q_xu_(2, 7)*k_ux(7, 1) + Q_xu_(2, 8)*k_ux(8, 1) + Q_xu_(2, 9)*k_ux(9, 1) + Q_xx_(1, 2);
    V_xx(2, 2) = Q_xu_(2, 0)*k_ux(0, 2) + Q_xu_(2, 1)*k_ux(1, 2) + Q_xu_(2, 10)*k_ux(10, 2) + Q_xu_(2, 11)*k_ux(11, 2) + Q_xu_(2, 12)*k_ux(12, 2) + Q_xu_(2, 2)*k_ux(2, 2) + Q_xu_(2, 5)*k_ux(5, 2) + Q_xu_(2, 6)*k_ux(6, 2) + Q_xu_(2, 7)*k_ux(7, 2) + Q_xu_(2, 8)*k_ux(8, 2) + Q_xu_(2, 9)*k_ux(9, 2) + Q_xx_(2, 2);
    V_xx(2, 3) = Q_xu_(2, 0)*k_ux(0, 3) + Q_xu_(2, 1)*k_ux(1, 3) + Q_xu_(2, 10)*k_ux(10, 3) + Q_xu_(2, 11)*k_ux(11, 3) + Q_xu_(2, 12)*k_ux(12, 3) + Q_xu_(2, 2)*k_ux(2, 3) + Q_xu_(2, 5)*k_ux(5, 3) + Q_xu_(2, 6)*k_ux(6, 3) + Q_xu_(2, 7)*k_ux(7, 3) + Q_xu_(2, 8)*k_ux(8, 3) + Q_xu_(2, 9)*k_ux(9, 3) + Q_xx_(2, 3);
    V_xx(2, 4) = Q_xu_(2, 0)*k_ux(0, 4) + Q_xu_(2, 1)*k_ux(1, 4) + Q_xx_(2, 4);
    V_xx(2, 5) = Q_xu_(2, 0)*k_ux(0, 5) + Q_xu_(2, 1)*k_ux(1, 5) + Q_xx_(2, 5);
    V_xx(2, 6) = Q_xu_(2, 0)*k_ux(0, 6) + Q_xu_(2, 1)*k_ux(1, 6) + Q_xx_(2, 6);
    V_xx(3, 0) = Q_xu_(3, 0)*k_ux(0, 0) + Q_xu_(3, 1)*k_ux(1, 0) + Q_xx_(0, 3);
    V_xx(3, 1) = Q_xu_(3, 0)*k_ux(0, 1) + Q_xu_(3, 1)*k_ux(1, 1) + Q_xu_(3, 10)*k_ux(10, 1) + Q_xu_(3, 11)*k_ux(11, 1) + Q_xu_(3, 12)*k_ux(12, 1) + Q_xu_(3, 2)*k_ux(2, 1) + Q_xu_(3, 5)*k_ux(5, 1) + Q_xu_(3, 6)*k_ux(6, 1) + Q_xu_(3, 7)*k_ux(7, 1) + Q_xu_(3, 8)*k_ux(8, 1) + Q_xu_(3, 9)*k_ux(9, 1) + Q_xx_(1, 3);
    V_xx(3, 2) = Q_xu_(3, 0)*k_ux(0, 2) + Q_xu_(3, 1)*k_ux(1, 2) + Q_xu_(3, 10)*k_ux(10, 2) + Q_xu_(3, 11)*k_ux(11, 2) + Q_xu_(3, 12)*k_ux(12, 2) + Q_xu_(3, 2)*k_ux(2, 2) + Q_xu_(3, 5)*k_ux(5, 2) + Q_xu_(3, 6)*k_ux(6, 2) + Q_xu_(3, 7)*k_ux(7, 2) + Q_xu_(3, 8)*k_ux(8, 2) + Q_xu_(3, 9)*k_ux(9, 2) + Q_xx_(2, 3);
    V_xx(3, 3) = Q_xu_(3, 0)*k_ux(0, 3) + Q_xu_(3, 1)*k_ux(1, 3) + Q_xu_(3, 10)*k_ux(10, 3) + Q_xu_(3, 11)*k_ux(11, 3) + Q_xu_(3, 12)*k_ux(12, 3) + Q_xu_(3, 2)*k_ux(2, 3) + Q_xu_(3, 5)*k_ux(5, 3) + Q_xu_(3, 6)*k_ux(6, 3) + Q_xu_(3, 7)*k_ux(7, 3) + Q_xu_(3, 8)*k_ux(8, 3) + Q_xu_(3, 9)*k_ux(9, 3) + Q_xx_(3, 3);
    V_xx(3, 4) = Q_xu_(3, 0)*k_ux(0, 4) + Q_xu_(3, 1)*k_ux(1, 4) + Q_xx_(3, 4);
    V_xx(3, 5) = Q_xu_(3, 0)*k_ux(0, 5) + Q_xu_(3, 1)*k_ux(1, 5) + Q_xx_(3, 5);
    V_xx(3, 6) = Q_xu_(3, 0)*k_ux(0, 6) + Q_xu_(3, 1)*k_ux(1, 6) + Q_xx_(3, 6);
    V_xx(4, 0) = Q_xu_(4, 0)*k_ux(0, 0) + Q_xu_(4, 1)*k_ux(1, 0) + Q_xx_(0, 4);
    V_xx(4, 1) = Q_xu_(4, 0)*k_ux(0, 1) + Q_xu_(4, 1)*k_ux(1, 1) + Q_xx_(1, 4);
    V_xx(4, 2) = Q_xu_(4, 0)*k_ux(0, 2) + Q_xu_(4, 1)*k_ux(1, 2) + Q_xx_(2, 4);
    V_xx(4, 3) = Q_xu_(4, 0)*k_ux(0, 3) + Q_xu_(4, 1)*k_ux(1, 3) + Q_xx_(3, 4);
    V_xx(4, 4) = Q_xu_(4, 0)*k_ux(0, 4) + Q_xu_(4, 1)*k_ux(1, 4) + Q_xx_(4, 4);
    V_xx(4, 5) = Q_xu_(4, 0)*k_ux(0, 5) + Q_xu_(4, 1)*k_ux(1, 5) + Q_xx_(4, 5);
    V_xx(4, 6) = Q_xu_(4, 0)*k_ux(0, 6) + Q_xu_(4, 1)*k_ux(1, 6) + Q_xx_(4, 6);
    V_xx(5, 0) = Q_xu_(5, 0)*k_ux(0, 0) + Q_xu_(5, 1)*k_ux(1, 0) + Q_xx_(0, 5);
    V_xx(5, 1) = Q_xu_(5, 0)*k_ux(0, 1) + Q_xu_(5, 1)*k_ux(1, 1) + Q_xx_(1, 5);
    V_xx(5, 2) = Q_xu_(5, 0)*k_ux(0, 2) + Q_xu_(5, 1)*k_ux(1, 2) + Q_xx_(2, 5);
    V_xx(5, 3) = Q_xu_(5, 0)*k_ux(0, 3) + Q_xu_(5, 1)*k_ux(1, 3) + Q_xx_(3, 5);
    V_xx(5, 4) = Q_xu_(5, 0)*k_ux(0, 4) + Q_xu_(5, 1)*k_ux(1, 4) + Q_xx_(4, 5);
    V_xx(5, 5) = Q_xu_(5, 0)*k_ux(0, 5) + Q_xu_(5, 1)*k_ux(1, 5) + Q_xu_(5, 4)*k_ux(4, 5) + Q_xx_(5, 5);
    V_xx(5, 6) = Q_xu_(5, 0)*k_ux(0, 6) + Q_xu_(5, 1)*k_ux(1, 6) + Q_xx_(5, 6);
    V_xx(6, 0) = Q_xu_(6, 0)*k_ux(0, 0) + Q_xu_(6, 1)*k_ux(1, 0) + Q_xx_(0, 6);
    V_xx(6, 1) = Q_xu_(6, 0)*k_ux(0, 1) + Q_xu_(6, 1)*k_ux(1, 1) + Q_xx_(1, 6);
    V_xx(6, 2) = Q_xu_(6, 0)*k_ux(0, 2) + Q_xu_(6, 1)*k_ux(1, 2) + Q_xx_(2, 6);
    V_xx(6, 3) = Q_xu_(6, 0)*k_ux(0, 3) + Q_xu_(6, 1)*k_ux(1, 3) + Q_xx_(3, 6);
    V_xx(6, 4) = Q_xu_(6, 0)*k_ux(0, 4) + Q_xu_(6, 1)*k_ux(1, 4) + Q_xx_(4, 6);
    V_xx(6, 5) = Q_xu_(6, 0)*k_ux(0, 5) + Q_xu_(6, 1)*k_ux(1, 5) + Q_xx_(5, 6);
    V_xx(6, 6) = Q_xu_(6, 0)*k_ux(0, 6) + Q_xu_(6, 1)*k_ux(1, 6) + Q_xx_(6, 6);

    // Evaluation of Vector V_x
    V_x(0) = Q_x_(0) + Q_xu_(0, 0)*v_u(0) + Q_xu_(0, 1)*v_u(1) + Q_xu_(0, 3)*v_u(3);
    V_x(1) = Q_x_(1) + Q_xu_(1, 0)*v_u(0) + Q_xu_(1, 1)*v_u(1) + Q_xu_(1, 10)*v_u(10) + Q_xu_(1, 11)*v_u(11) + Q_xu_(1, 12)*v_u(12) + Q_xu_(1, 2)*v_u(2) + Q_xu_(1, 5)*v_u(5) + Q_xu_(1, 6)*v_u(6) + Q_xu_(1, 7)*v_u(7) + Q_xu_(1, 8)*v_u(8) + Q_xu_(1, 9)*v_u(9);
    V_x(2) = Q_x_(2) + Q_xu_(2, 0)*v_u(0) + Q_xu_(2, 1)*v_u(1) + Q_xu_(2, 10)*v_u(10) + Q_xu_(2, 11)*v_u(11) + Q_xu_(2, 12)*v_u(12) + Q_xu_(2, 2)*v_u(2) + Q_xu_(2, 5)*v_u(5) + Q_xu_(2, 6)*v_u(6) + Q_xu_(2, 7)*v_u(7) + Q_xu_(2, 8)*v_u(8) + Q_xu_(2, 9)*v_u(9);
    V_x(3) = Q_x_(3) + Q_xu_(3, 0)*v_u(0) + Q_xu_(3, 1)*v_u(1) + Q_xu_(3, 10)*v_u(10) + Q_xu_(3, 11)*v_u(11) + Q_xu_(3, 12)*v_u(12) + Q_xu_(3, 2)*v_u(2) + Q_xu_(3, 5)*v_u(5) + Q_xu_(3, 6)*v_u(6) + Q_xu_(3, 7)*v_u(7) + Q_xu_(3, 8)*v_u(8) + Q_xu_(3, 9)*v_u(9);
    V_x(4) = Q_x_(4) + Q_xu_(4, 0)*v_u(0) + Q_xu_(4, 1)*v_u(1);
    V_x(5) = Q_x_(5) + Q_xu_(5, 0)*v_u(0) + Q_xu_(5, 1)*v_u(1) + Q_xu_(5, 4)*v_u(4);
    V_x(6) = Q_x_(6) + Q_xu_(6, 0)*v_u(0) + Q_xu_(6, 1)*v_u(1);

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
    Q_un_(0, 6) = -Fu(6, 1)*Q_uu_inv_(0, 1);
    Q_un_(1, 0) = -Fu(0, 0)*Q_uu_inv_(0, 1) - Fu(0, 1)*Q_uu_inv_(1, 1);
    Q_un_(1, 1) = -Fu(1, 0)*Q_uu_inv_(0, 1) - Fu(1, 1)*Q_uu_inv_(1, 1);
    Q_un_(1, 2) = -Fu(2, 0)*Q_uu_inv_(0, 1) - Fu(2, 1)*Q_uu_inv_(1, 1);
    Q_un_(1, 3) = -Fu(3, 0)*Q_uu_inv_(0, 1) - Fu(3, 1)*Q_uu_inv_(1, 1);
    Q_un_(1, 4) = -Fu(4, 0)*Q_uu_inv_(0, 1);
    Q_un_(1, 5) = -Fu(5, 1)*Q_uu_inv_(1, 1);
    Q_un_(1, 6) = -Fu(6, 1)*Q_uu_inv_(1, 1);

    // Evaluation of Matrix Q_nuu_
    Q_nuu_(0, 0) = -Q_uu_(0, 0);
    Q_nuu_(0, 1) = internal_0;
    Q_nuu_(1, 0) = internal_0;
    Q_nuu_(1, 1) = -Q_uu_(1, 1);
    Q_nuu_(2, 2) = -Q_uu_(2, 2);
    Q_nuu_(3, 3) = -Q_uu_(3, 3);
    Q_nuu_(4, 4) = -Q_uu_(4, 4);
    Q_nuu_(5, 5) = -Q_uu_(5, 5);
    Q_nuu_(6, 6) = -Q_uu_(6, 6);
    Q_nuu_(7, 7) = -Q_uu_(7, 7);
    Q_nuu_(8, 8) = -Q_uu_(8, 8);
    Q_nuu_(9, 9) = -Q_uu_(9, 9);
    Q_nuu_(10, 10) = -Q_uu_(10, 10);
    Q_nuu_(11, 11) = -Q_uu_(11, 11);
    Q_nuu_(12, 12) = -Q_uu_(12, 12);

    // Evaluation of Matrix Q_vnx_
    Q_vnx_(0, 0) = Fu(0, 0)*k_ux(0, 0) + Fu(0, 1)*k_ux(1, 0) + Fx(0, 0);
    Q_vnx_(0, 1) = Fu(0, 0)*k_ux(0, 1) + Fu(0, 1)*k_ux(1, 1) + Fx(0, 1);
    Q_vnx_(0, 2) = Fu(0, 0)*k_ux(0, 2) + Fu(0, 1)*k_ux(1, 2) + Fx(0, 2);
    Q_vnx_(0, 3) = Fu(0, 0)*k_ux(0, 3) + Fu(0, 1)*k_ux(1, 3) + Fx(0, 3);
    Q_vnx_(0, 4) = Fu(0, 0)*k_ux(0, 4) + Fu(0, 1)*k_ux(1, 4) + Fx(0, 4);
    Q_vnx_(0, 5) = Fu(0, 0)*k_ux(0, 5) + Fu(0, 1)*k_ux(1, 5) + Fx(0, 5);
    Q_vnx_(0, 6) = Fu(0, 0)*k_ux(0, 6) + Fu(0, 1)*k_ux(1, 6) + Fx(0, 6);
    Q_vnx_(1, 0) = Fu(1, 0)*k_ux(0, 0) + Fu(1, 1)*k_ux(1, 0);
    Q_vnx_(1, 1) = Fu(1, 0)*k_ux(0, 1) + Fu(1, 1)*k_ux(1, 1) + Fx(1, 1);
    Q_vnx_(1, 2) = Fu(1, 0)*k_ux(0, 2) + Fu(1, 1)*k_ux(1, 2);
    Q_vnx_(1, 3) = Fu(1, 0)*k_ux(0, 3) + Fu(1, 1)*k_ux(1, 3) + Fx(1, 3);
    Q_vnx_(1, 4) = Fu(1, 0)*k_ux(0, 4) + Fu(1, 1)*k_ux(1, 4) + Fx(1, 4);
    Q_vnx_(1, 5) = Fu(1, 0)*k_ux(0, 5) + Fu(1, 1)*k_ux(1, 5) + Fx(1, 5);
    Q_vnx_(1, 6) = Fu(1, 0)*k_ux(0, 6) + Fu(1, 1)*k_ux(1, 6) + Fx(1, 6);
    Q_vnx_(2, 0) = Fu(2, 0)*k_ux(0, 0) + Fu(2, 1)*k_ux(1, 0);
    Q_vnx_(2, 1) = Fu(2, 0)*k_ux(0, 1) + Fu(2, 1)*k_ux(1, 1);
    Q_vnx_(2, 2) = Fu(2, 0)*k_ux(0, 2) + Fu(2, 1)*k_ux(1, 2) + Fx(2, 2);
    Q_vnx_(2, 3) = Fu(2, 0)*k_ux(0, 3) + Fu(2, 1)*k_ux(1, 3) + Fx(2, 3);
    Q_vnx_(2, 4) = Fu(2, 0)*k_ux(0, 4) + Fu(2, 1)*k_ux(1, 4) + Fx(2, 4);
    Q_vnx_(2, 5) = Fu(2, 0)*k_ux(0, 5) + Fu(2, 1)*k_ux(1, 5) + Fx(2, 5);
    Q_vnx_(2, 6) = Fu(2, 0)*k_ux(0, 6) + Fu(2, 1)*k_ux(1, 6) + Fx(2, 6);
    Q_vnx_(3, 0) = Fu(3, 0)*k_ux(0, 0) + Fu(3, 1)*k_ux(1, 0);
    Q_vnx_(3, 1) = Fu(3, 0)*k_ux(0, 1) + Fu(3, 1)*k_ux(1, 1);
    Q_vnx_(3, 2) = Fu(3, 0)*k_ux(0, 2) + Fu(3, 1)*k_ux(1, 2);
    Q_vnx_(3, 3) = Fu(3, 0)*k_ux(0, 3) + Fu(3, 1)*k_ux(1, 3) + Fx(3, 3);
    Q_vnx_(3, 4) = Fu(3, 0)*k_ux(0, 4) + Fu(3, 1)*k_ux(1, 4) + Fx(3, 4);
    Q_vnx_(3, 5) = Fu(3, 0)*k_ux(0, 5) + Fu(3, 1)*k_ux(1, 5) + Fx(3, 5);
    Q_vnx_(3, 6) = Fu(3, 0)*k_ux(0, 6) + Fu(3, 1)*k_ux(1, 6) + Fx(3, 6);
    Q_vnx_(4, 0) = Fu(4, 0)*k_ux(0, 0);
    Q_vnx_(4, 1) = Fu(4, 0)*k_ux(0, 1);
    Q_vnx_(4, 2) = Fu(4, 0)*k_ux(0, 2);
    Q_vnx_(4, 3) = Fu(4, 0)*k_ux(0, 3);
    Q_vnx_(4, 4) = Fu(4, 0)*k_ux(0, 4) + Fx(4, 4);
    Q_vnx_(4, 5) = Fu(4, 0)*k_ux(0, 5);
    Q_vnx_(4, 6) = Fu(4, 0)*k_ux(0, 6);
    Q_vnx_(5, 0) = Fu(5, 1)*k_ux(1, 0);
    Q_vnx_(5, 1) = Fu(5, 1)*k_ux(1, 1);
    Q_vnx_(5, 2) = Fu(5, 1)*k_ux(1, 2);
    Q_vnx_(5, 3) = Fu(5, 1)*k_ux(1, 3);
    Q_vnx_(5, 4) = Fu(5, 1)*k_ux(1, 4);
    Q_vnx_(5, 5) = Fu(5, 1)*k_ux(1, 5) + Fx(5, 5);
    Q_vnx_(5, 6) = Fu(5, 1)*k_ux(1, 6) + Fx(5, 6);
    Q_vnx_(6, 0) = Fu(6, 1)*k_ux(1, 0);
    Q_vnx_(6, 1) = Fu(6, 1)*k_ux(1, 1);
    Q_vnx_(6, 2) = Fu(6, 1)*k_ux(1, 2);
    Q_vnx_(6, 3) = Fu(6, 1)*k_ux(1, 3);
    Q_vnx_(6, 4) = Fu(6, 1)*k_ux(1, 4);
    Q_vnx_(6, 5) = Fu(6, 1)*k_ux(1, 5);
    Q_vnx_(6, 6) = Fu(6, 1)*k_ux(1, 6) + Fx(6, 6);

    // Evaluation of Vector Q_vn_
    Q_vn_(0) = Fu(0, 0)*v_u(0) + Fu(0, 1)*v_u(1) + r_f(0);
    Q_vn_(1) = Fu(1, 0)*v_u(0) + Fu(1, 1)*v_u(1) + r_f(1);
    Q_vn_(2) = Fu(2, 0)*v_u(0) + Fu(2, 1)*v_u(1) + r_f(2);
    Q_vn_(3) = Fu(3, 0)*v_u(0) + Fu(3, 1)*v_u(1) + r_f(3);
    Q_vn_(4) = Fu(4, 0)*v_u(0) + r_f(4);
    Q_vn_(5) = Fu(5, 1)*v_u(1) + r_f(5);
    Q_vn_(6) = Fu(6, 1)*v_u(1) + r_f(6);

  }

};

template <>
class SpatiotemporalPlannerModelIpmEvaluator<StageType::INITIAL, OcpConfig::ERK4> : public IpmEvaluator {
 public:
  SpatiotemporalPlannerModelIpmEvaluator() : IpmEvaluator(7, 13, 4, 0, 7) {}
  virtual ~SpatiotemporalPlannerModelIpmEvaluator() = default;

  virtual void updateLX(const Eigen::VectorXd& l_x, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& h_x, const Eigen::MatrixXd& m_x, const Eigen::VectorXd& p, const Eigen::VectorXd& lambda, const Eigen::VectorXd& eta, const Eigen::VectorXd& nu, Eigen::VectorXd& sl_x) const override {
    // Evaluation of Vector sl_x
    sl_x(0) = Fx(0, 0)*p(0) + l_x(0) + m_x(0, 0)*nu(0);
    sl_x(1) = Fx(0, 1)*p(0) + Fx(1, 1)*p(1) + l_x(1) + m_x(1, 1)*nu(1);
    sl_x(2) = Fx(0, 2)*p(0) + Fx(2, 2)*p(2) + l_x(2) + m_x(2, 2)*nu(2);
    sl_x(3) = Fx(0, 3)*p(0) + Fx(1, 3)*p(1) + Fx(2, 3)*p(2) + Fx(3, 3)*p(3) + l_x(3) + m_x(3, 3)*nu(3);
    sl_x(4) = Fx(0, 4)*p(0) + Fx(1, 4)*p(1) + Fx(2, 4)*p(2) + Fx(3, 4)*p(3) + Fx(4, 4)*p(4) + l_x(4) + m_x(4, 4)*nu(4);
    sl_x(5) = Fx(0, 5)*p(0) + Fx(1, 5)*p(1) + Fx(2, 5)*p(2) + Fx(3, 5)*p(3) + Fx(5, 5)*p(5) + l_x(5) + m_x(5, 5)*nu(5);
    sl_x(6) = Fx(0, 6)*p(0) + Fx(1, 6)*p(1) + Fx(2, 6)*p(2) + Fx(3, 6)*p(3) + Fx(5, 6)*p(5) + Fx(6, 6)*p(6) + l_x(6) + m_x(6, 6)*nu(6);

  }

  virtual void updateLU(const Eigen::VectorXd& l_u, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& g_u, const Eigen::MatrixXd& h_u, const Eigen::VectorXd& p, const Eigen::VectorXd& lambda, const Eigen::VectorXd& eta, Eigen::VectorXd& sl_u) const override {
    // Evaluation of Vector sl_u
    sl_u(0) = Fu(0, 0)*p(0) + Fu(1, 0)*p(1) + Fu(2, 0)*p(2) + Fu(3, 0)*p(3) + Fu(4, 0)*p(4) + g_u(0, 0)*lambda(0) + g_u(1, 0)*lambda(1) + l_u(0);
    sl_u(1) = Fu(0, 1)*p(0) + Fu(1, 1)*p(1) + Fu(2, 1)*p(2) + Fu(3, 1)*p(3) + Fu(5, 1)*p(5) + Fu(6, 1)*p(6) + g_u(2, 1)*lambda(2) + g_u(3, 1)*lambda(3) + l_u(1);
    sl_u(2) = l_u(2);
    sl_u(3) = l_u(3);
    sl_u(4) = l_u(4);
    sl_u(5) = l_u(5);
    sl_u(6) = l_u(6);
    sl_u(7) = l_u(7);
    sl_u(8) = l_u(8);
    sl_u(9) = l_u(9);
    sl_u(10) = l_u(10);
    sl_u(11) = l_u(11);
    sl_u(12) = l_u(12);

  }

  virtual void updateQH(const Eigen::MatrixXd& l_xx, const Eigen::MatrixXd& l_xu, const Eigen::MatrixXd& l_uu, const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& V_xx_prev) override {
    // Determine internal variables
    const double internal_0 = Fx(0, 1)*V_xx_prev(0, 0);
    const double internal_1 = Fx(0, 0)*V_xx_prev(0, 1);
    const double internal_10 = Fx(0, 6)*V_xx_prev(0, 0);
    const double internal_11 = Fx(0, 0)*V_xx_prev(0, 6);
    const double internal_2 = Fx(0, 2)*V_xx_prev(0, 0);
    const double internal_3 = Fx(0, 0)*V_xx_prev(0, 2);
    const double internal_4 = Fx(0, 3)*V_xx_prev(0, 0);
    const double internal_5 = Fx(0, 0)*V_xx_prev(0, 3);
    const double internal_54 = Fu(0, 0)*V_xx_prev(0, 0);
    const double internal_55 = Fu(0, 1)*V_xx_prev(0, 0);
    const double internal_6 = Fx(0, 4)*V_xx_prev(0, 0);
    const double internal_7 = Fx(0, 0)*V_xx_prev(0, 4);
    const double internal_8 = Fx(0, 5)*V_xx_prev(0, 0);
    const double internal_9 = Fx(0, 0)*V_xx_prev(0, 5);
    const double internal_13 = Fx(0, 1)*V_xx_prev(0, 1) + Fx(1, 1)*V_xx_prev(1, 1);
    const double internal_14 = Fx(0, 1)*V_xx_prev(0, 2) + Fx(1, 1)*V_xx_prev(1, 2);
    const double internal_15 = Fx(0, 1)*V_xx_prev(0, 3) + Fx(1, 1)*V_xx_prev(1, 3);
    const double internal_16 = Fx(0, 1)*V_xx_prev(0, 4) + Fx(1, 1)*V_xx_prev(1, 4);
    const double internal_17 = Fx(0, 1)*V_xx_prev(0, 5) + Fx(1, 1)*V_xx_prev(1, 5);
    const double internal_18 = Fx(0, 1)*V_xx_prev(0, 6) + Fx(1, 1)*V_xx_prev(1, 6);
    const double internal_20 = Fx(0, 2)*V_xx_prev(0, 1) + Fx(2, 2)*V_xx_prev(2, 1);
    const double internal_21 = Fx(0, 2)*V_xx_prev(0, 2) + Fx(2, 2)*V_xx_prev(2, 2);
    const double internal_22 = Fx(0, 2)*V_xx_prev(0, 3) + Fx(2, 2)*V_xx_prev(2, 3);
    const double internal_23 = Fx(0, 2)*V_xx_prev(0, 4) + Fx(2, 2)*V_xx_prev(2, 4);
    const double internal_24 = Fx(0, 2)*V_xx_prev(0, 5) + Fx(2, 2)*V_xx_prev(2, 5);
    const double internal_25 = Fx(0, 2)*V_xx_prev(0, 6) + Fx(2, 2)*V_xx_prev(2, 6);
    const double internal_27 = Fx(0, 3)*V_xx_prev(0, 1) + Fx(1, 3)*V_xx_prev(1, 1) + Fx(2, 3)*V_xx_prev(2, 1) + Fx(3, 3)*V_xx_prev(3, 1);
    const double internal_28 = Fx(0, 3)*V_xx_prev(0, 2) + Fx(1, 3)*V_xx_prev(1, 2) + Fx(2, 3)*V_xx_prev(2, 2) + Fx(3, 3)*V_xx_prev(3, 2);
    const double internal_29 = Fx(0, 3)*V_xx_prev(0, 3) + Fx(1, 3)*V_xx_prev(1, 3) + Fx(2, 3)*V_xx_prev(2, 3) + Fx(3, 3)*V_xx_prev(3, 3);
    const double internal_30 = Fx(0, 3)*V_xx_prev(0, 4) + Fx(1, 3)*V_xx_prev(1, 4) + Fx(2, 3)*V_xx_prev(2, 4) + Fx(3, 3)*V_xx_prev(3, 4);
    const double internal_31 = Fx(0, 3)*V_xx_prev(0, 5) + Fx(1, 3)*V_xx_prev(1, 5) + Fx(2, 3)*V_xx_prev(2, 5) + Fx(3, 3)*V_xx_prev(3, 5);
    const double internal_32 = Fx(0, 3)*V_xx_prev(0, 6) + Fx(1, 3)*V_xx_prev(1, 6) + Fx(2, 3)*V_xx_prev(2, 6) + Fx(3, 3)*V_xx_prev(3, 6);
    const double internal_34 = Fx(0, 4)*V_xx_prev(0, 1) + Fx(1, 4)*V_xx_prev(1, 1) + Fx(2, 4)*V_xx_prev(2, 1) + Fx(3, 4)*V_xx_prev(3, 1) + Fx(4, 4)*V_xx_prev(4, 1);
    const double internal_35 = Fx(0, 4)*V_xx_prev(0, 2) + Fx(1, 4)*V_xx_prev(1, 2) + Fx(2, 4)*V_xx_prev(2, 2) + Fx(3, 4)*V_xx_prev(3, 2) + Fx(4, 4)*V_xx_prev(4, 2);
    const double internal_36 = Fx(0, 4)*V_xx_prev(0, 3) + Fx(1, 4)*V_xx_prev(1, 3) + Fx(2, 4)*V_xx_prev(2, 3) + Fx(3, 4)*V_xx_prev(3, 3) + Fx(4, 4)*V_xx_prev(4, 3);
    const double internal_37 = Fx(0, 4)*V_xx_prev(0, 4) + Fx(1, 4)*V_xx_prev(1, 4) + Fx(2, 4)*V_xx_prev(2, 4) + Fx(3, 4)*V_xx_prev(3, 4) + Fx(4, 4)*V_xx_prev(4, 4);
    const double internal_38 = Fx(0, 4)*V_xx_prev(0, 5) + Fx(1, 4)*V_xx_prev(1, 5) + Fx(2, 4)*V_xx_prev(2, 5) + Fx(3, 4)*V_xx_prev(3, 5) + Fx(4, 4)*V_xx_prev(4, 5);
    const double internal_39 = Fx(0, 4)*V_xx_prev(0, 6) + Fx(1, 4)*V_xx_prev(1, 6) + Fx(2, 4)*V_xx_prev(2, 6) + Fx(3, 4)*V_xx_prev(3, 6) + Fx(4, 4)*V_xx_prev(4, 6);
    const double internal_41 = Fx(0, 5)*V_xx_prev(0, 1) + Fx(1, 5)*V_xx_prev(1, 1) + Fx(2, 5)*V_xx_prev(2, 1) + Fx(3, 5)*V_xx_prev(3, 1) + Fx(5, 5)*V_xx_prev(5, 1);
    const double internal_42 = Fx(0, 5)*V_xx_prev(0, 2) + Fx(1, 5)*V_xx_prev(1, 2) + Fx(2, 5)*V_xx_prev(2, 2) + Fx(3, 5)*V_xx_prev(3, 2) + Fx(5, 5)*V_xx_prev(5, 2);
    const double internal_43 = Fx(0, 5)*V_xx_prev(0, 3) + Fx(1, 5)*V_xx_prev(1, 3) + Fx(2, 5)*V_xx_prev(2, 3) + Fx(3, 5)*V_xx_prev(3, 3) + Fx(5, 5)*V_xx_prev(5, 3);
    const double internal_44 = Fx(0, 5)*V_xx_prev(0, 4) + Fx(1, 5)*V_xx_prev(1, 4) + Fx(2, 5)*V_xx_prev(2, 4) + Fx(3, 5)*V_xx_prev(3, 4) + Fx(5, 5)*V_xx_prev(5, 4);
    const double internal_45 = Fx(0, 5)*V_xx_prev(0, 5) + Fx(1, 5)*V_xx_prev(1, 5) + Fx(2, 5)*V_xx_prev(2, 5) + Fx(3, 5)*V_xx_prev(3, 5) + Fx(5, 5)*V_xx_prev(5, 5);
    const double internal_46 = Fx(0, 5)*V_xx_prev(0, 6) + Fx(1, 5)*V_xx_prev(1, 6) + Fx(2, 5)*V_xx_prev(2, 6) + Fx(3, 5)*V_xx_prev(3, 6) + Fx(5, 5)*V_xx_prev(5, 6);
    const double internal_48 = Fx(0, 6)*V_xx_prev(0, 1) + Fx(1, 6)*V_xx_prev(1, 1) + Fx(2, 6)*V_xx_prev(2, 1) + Fx(3, 6)*V_xx_prev(3, 1) + Fx(5, 6)*V_xx_prev(5, 1) + Fx(6, 6)*V_xx_prev(6, 1);
    const double internal_49 = Fx(0, 6)*V_xx_prev(0, 2) + Fx(1, 6)*V_xx_prev(1, 2) + Fx(2, 6)*V_xx_prev(2, 2) + Fx(3, 6)*V_xx_prev(3, 2) + Fx(5, 6)*V_xx_prev(5, 2) + Fx(6, 6)*V_xx_prev(6, 2);
    const double internal_50 = Fx(0, 6)*V_xx_prev(0, 3) + Fx(1, 6)*V_xx_prev(1, 3) + Fx(2, 6)*V_xx_prev(2, 3) + Fx(3, 6)*V_xx_prev(3, 3) + Fx(5, 6)*V_xx_prev(5, 3) + Fx(6, 6)*V_xx_prev(6, 3);
    const double internal_51 = Fx(0, 6)*V_xx_prev(0, 4) + Fx(1, 6)*V_xx_prev(1, 4) + Fx(2, 6)*V_xx_prev(2, 4) + Fx(3, 6)*V_xx_prev(3, 4) + Fx(5, 6)*V_xx_prev(5, 4) + Fx(6, 6)*V_xx_prev(6, 4);
    const double internal_52 = Fx(0, 6)*V_xx_prev(0, 5) + Fx(1, 6)*V_xx_prev(1, 5) + Fx(2, 6)*V_xx_prev(2, 5) + Fx(3, 6)*V_xx_prev(3, 5) + Fx(5, 6)*V_xx_prev(5, 5) + Fx(6, 6)*V_xx_prev(6, 5);
    const double internal_53 = Fx(0, 6)*V_xx_prev(0, 6) + Fx(1, 6)*V_xx_prev(1, 6) + Fx(2, 6)*V_xx_prev(2, 6) + Fx(3, 6)*V_xx_prev(3, 6) + Fx(5, 6)*V_xx_prev(5, 6) + Fx(6, 6)*V_xx_prev(6, 6);
    const double internal_57 = Fu(0, 0)*V_xx_prev(0, 1) + Fu(1, 0)*V_xx_prev(1, 1) + Fu(2, 0)*V_xx_prev(2, 1) + Fu(3, 0)*V_xx_prev(3, 1) + Fu(4, 0)*V_xx_prev(4, 1);
    const double internal_58 = Fu(0, 0)*V_xx_prev(0, 2) + Fu(1, 0)*V_xx_prev(1, 2) + Fu(2, 0)*V_xx_prev(2, 2) + Fu(3, 0)*V_xx_prev(3, 2) + Fu(4, 0)*V_xx_prev(4, 2);
    const double internal_59 = Fu(0, 0)*V_xx_prev(0, 3) + Fu(1, 0)*V_xx_prev(1, 3) + Fu(2, 0)*V_xx_prev(2, 3) + Fu(3, 0)*V_xx_prev(3, 3) + Fu(4, 0)*V_xx_prev(4, 3);
    const double internal_61 = Fu(0, 1)*V_xx_prev(0, 1) + Fu(1, 1)*V_xx_prev(1, 1) + Fu(2, 1)*V_xx_prev(2, 1) + Fu(3, 1)*V_xx_prev(3, 1) + Fu(5, 1)*V_xx_prev(5, 1) + Fu(6, 1)*V_xx_prev(6, 1);
    const double internal_62 = Fu(0, 1)*V_xx_prev(0, 2) + Fu(1, 1)*V_xx_prev(1, 2) + Fu(2, 1)*V_xx_prev(2, 2) + Fu(3, 1)*V_xx_prev(3, 2) + Fu(5, 1)*V_xx_prev(5, 2) + Fu(6, 1)*V_xx_prev(6, 2);
    const double internal_63 = Fu(0, 1)*V_xx_prev(0, 3) + Fu(1, 1)*V_xx_prev(1, 3) + Fu(2, 1)*V_xx_prev(2, 3) + Fu(3, 1)*V_xx_prev(3, 3) + Fu(5, 1)*V_xx_prev(5, 3) + Fu(6, 1)*V_xx_prev(6, 3);
    const double internal_12 = Fx(1, 1)*V_xx_prev(1, 0) + internal_0;
    const double internal_19 = Fx(2, 2)*V_xx_prev(2, 0) + internal_2;
    const double internal_26 = Fx(1, 3)*V_xx_prev(1, 0) + Fx(2, 3)*V_xx_prev(2, 0) + Fx(3, 3)*V_xx_prev(3, 0) + internal_4;
    const double internal_33 = Fx(1, 4)*V_xx_prev(1, 0) + Fx(2, 4)*V_xx_prev(2, 0) + Fx(3, 4)*V_xx_prev(3, 0) + Fx(4, 4)*V_xx_prev(4, 0) + internal_6;
    const double internal_40 = Fx(1, 5)*V_xx_prev(1, 0) + Fx(2, 5)*V_xx_prev(2, 0) + Fx(3, 5)*V_xx_prev(3, 0) + Fx(5, 5)*V_xx_prev(5, 0) + internal_8;
    const double internal_47 = Fx(1, 6)*V_xx_prev(1, 0) + Fx(2, 6)*V_xx_prev(2, 0) + Fx(3, 6)*V_xx_prev(3, 0) + Fx(5, 6)*V_xx_prev(5, 0) + Fx(6, 6)*V_xx_prev(6, 0) + internal_10;
    const double internal_56 = Fu(1, 0)*V_xx_prev(1, 0) + Fu(2, 0)*V_xx_prev(2, 0) + Fu(3, 0)*V_xx_prev(3, 0) + Fu(4, 0)*V_xx_prev(4, 0) + internal_54;
    const double internal_60 = Fu(1, 1)*V_xx_prev(1, 0) + Fu(2, 1)*V_xx_prev(2, 0) + Fu(3, 1)*V_xx_prev(3, 0) + Fu(5, 1)*V_xx_prev(5, 0) + Fu(6, 1)*V_xx_prev(6, 0) + internal_55;

    // Evaluation of Matrix Q_xx_
    Q_xx_(0, 0) = pow(Fx(0, 0), 2)*V_xx_prev(0, 0) + l_xx(0, 0);
    Q_xx_(0, 1) = Fx(0, 0)*internal_0 + Fx(1, 1)*internal_1;
    Q_xx_(0, 2) = Fx(0, 0)*internal_2 + Fx(2, 2)*internal_3;
    Q_xx_(0, 3) = Fx(0, 0)*internal_4 + Fx(1, 3)*internal_1 + Fx(2, 3)*internal_3 + Fx(3, 3)*internal_5;
    Q_xx_(0, 4) = Fx(0, 0)*internal_6 + Fx(1, 4)*internal_1 + Fx(2, 4)*internal_3 + Fx(3, 4)*internal_5 + Fx(4, 4)*internal_7;
    Q_xx_(0, 5) = Fx(0, 0)*internal_8 + Fx(1, 5)*internal_1 + Fx(2, 5)*internal_3 + Fx(3, 5)*internal_5 + Fx(5, 5)*internal_9;
    Q_xx_(0, 6) = Fx(0, 0)*internal_10 + Fx(1, 6)*internal_1 + Fx(2, 6)*internal_3 + Fx(3, 6)*internal_5 + Fx(5, 6)*internal_9 + Fx(6, 6)*internal_11;
    Q_xx_(1, 0) = Fx(0, 0)*internal_12;
    Q_xx_(1, 1) = Fx(0, 1)*internal_12 + Fx(1, 1)*internal_13 + l_xx(1, 1);
    Q_xx_(1, 2) = Fx(0, 2)*internal_12 + Fx(2, 2)*internal_14 + l_xx(1, 2);
    Q_xx_(1, 3) = Fx(0, 3)*internal_12 + Fx(1, 3)*internal_13 + Fx(2, 3)*internal_14 + Fx(3, 3)*internal_15;
    Q_xx_(1, 4) = Fx(0, 4)*internal_12 + Fx(1, 4)*internal_13 + Fx(2, 4)*internal_14 + Fx(3, 4)*internal_15 + Fx(4, 4)*internal_16;
    Q_xx_(1, 5) = Fx(0, 5)*internal_12 + Fx(1, 5)*internal_13 + Fx(2, 5)*internal_14 + Fx(3, 5)*internal_15 + Fx(5, 5)*internal_17;
    Q_xx_(1, 6) = Fx(0, 6)*internal_12 + Fx(1, 6)*internal_13 + Fx(2, 6)*internal_14 + Fx(3, 6)*internal_15 + Fx(5, 6)*internal_17 + Fx(6, 6)*internal_18;
    Q_xx_(2, 0) = Fx(0, 0)*internal_19;
    Q_xx_(2, 1) = Fx(0, 1)*internal_19 + Fx(1, 1)*internal_20 + l_xx(2, 1);
    Q_xx_(2, 2) = Fx(0, 2)*internal_19 + Fx(2, 2)*internal_21 + l_xx(2, 2);
    Q_xx_(2, 3) = Fx(0, 3)*internal_19 + Fx(1, 3)*internal_20 + Fx(2, 3)*internal_21 + Fx(3, 3)*internal_22;
    Q_xx_(2, 4) = Fx(0, 4)*internal_19 + Fx(1, 4)*internal_20 + Fx(2, 4)*internal_21 + Fx(3, 4)*internal_22 + Fx(4, 4)*internal_23;
    Q_xx_(2, 5) = Fx(0, 5)*internal_19 + Fx(1, 5)*internal_20 + Fx(2, 5)*internal_21 + Fx(3, 5)*internal_22 + Fx(5, 5)*internal_24;
    Q_xx_(2, 6) = Fx(0, 6)*internal_19 + Fx(1, 6)*internal_20 + Fx(2, 6)*internal_21 + Fx(3, 6)*internal_22 + Fx(5, 6)*internal_24 + Fx(6, 6)*internal_25;
    Q_xx_(3, 0) = Fx(0, 0)*internal_26;
    Q_xx_(3, 1) = Fx(0, 1)*internal_26 + Fx(1, 1)*internal_27;
    Q_xx_(3, 2) = Fx(0, 2)*internal_26 + Fx(2, 2)*internal_28;
    Q_xx_(3, 3) = Fx(0, 3)*internal_26 + Fx(1, 3)*internal_27 + Fx(2, 3)*internal_28 + Fx(3, 3)*internal_29 + l_xx(3, 3);
    Q_xx_(3, 4) = Fx(0, 4)*internal_26 + Fx(1, 4)*internal_27 + Fx(2, 4)*internal_28 + Fx(3, 4)*internal_29 + Fx(4, 4)*internal_30;
    Q_xx_(3, 5) = Fx(0, 5)*internal_26 + Fx(1, 5)*internal_27 + Fx(2, 5)*internal_28 + Fx(3, 5)*internal_29 + Fx(5, 5)*internal_31;
    Q_xx_(3, 6) = Fx(0, 6)*internal_26 + Fx(1, 6)*internal_27 + Fx(2, 6)*internal_28 + Fx(3, 6)*internal_29 + Fx(5, 6)*internal_31 + Fx(6, 6)*internal_32;
    Q_xx_(4, 0) = Fx(0, 0)*internal_33;
    Q_xx_(4, 1) = Fx(0, 1)*internal_33 + Fx(1, 1)*internal_34;
    Q_xx_(4, 2) = Fx(0, 2)*internal_33 + Fx(2, 2)*internal_35;
    Q_xx_(4, 3) = Fx(0, 3)*internal_33 + Fx(1, 3)*internal_34 + Fx(2, 3)*internal_35 + Fx(3, 3)*internal_36;
    Q_xx_(4, 4) = Fx(0, 4)*internal_33 + Fx(1, 4)*internal_34 + Fx(2, 4)*internal_35 + Fx(3, 4)*internal_36 + Fx(4, 4)*internal_37 + l_xx(4, 4);
    Q_xx_(4, 5) = Fx(0, 5)*internal_33 + Fx(1, 5)*internal_34 + Fx(2, 5)*internal_35 + Fx(3, 5)*internal_36 + Fx(5, 5)*internal_38;
    Q_xx_(4, 6) = Fx(0, 6)*internal_33 + Fx(1, 6)*internal_34 + Fx(2, 6)*internal_35 + Fx(3, 6)*internal_36 + Fx(5, 6)*internal_38 + Fx(6, 6)*internal_39;
    Q_xx_(5, 0) = Fx(0, 0)*internal_40;
    Q_xx_(5, 1) = Fx(0, 1)*internal_40 + Fx(1, 1)*internal_41;
    Q_xx_(5, 2) = Fx(0, 2)*internal_40 + Fx(2, 2)*internal_42;
    Q_xx_(5, 3) = Fx(0, 3)*internal_40 + Fx(1, 3)*internal_41 + Fx(2, 3)*internal_42 + Fx(3, 3)*internal_43;
    Q_xx_(5, 4) = Fx(0, 4)*internal_40 + Fx(1, 4)*internal_41 + Fx(2, 4)*internal_42 + Fx(3, 4)*internal_43 + Fx(4, 4)*internal_44;
    Q_xx_(5, 5) = Fx(0, 5)*internal_40 + Fx(1, 5)*internal_41 + Fx(2, 5)*internal_42 + Fx(3, 5)*internal_43 + Fx(5, 5)*internal_45 + l_xx(5, 5);
    Q_xx_(5, 6) = Fx(0, 6)*internal_40 + Fx(1, 6)*internal_41 + Fx(2, 6)*internal_42 + Fx(3, 6)*internal_43 + Fx(5, 6)*internal_45 + Fx(6, 6)*internal_46;
    Q_xx_(6, 0) = Fx(0, 0)*internal_47;
    Q_xx_(6, 1) = Fx(0, 1)*internal_47 + Fx(1, 1)*internal_48;
    Q_xx_(6, 2) = Fx(0, 2)*internal_47 + Fx(2, 2)*internal_49;
    Q_xx_(6, 3) = Fx(0, 3)*internal_47 + Fx(1, 3)*internal_48 + Fx(2, 3)*internal_49 + Fx(3, 3)*internal_50;
    Q_xx_(6, 4) = Fx(0, 4)*internal_47 + Fx(1, 4)*internal_48 + Fx(2, 4)*internal_49 + Fx(3, 4)*internal_50 + Fx(4, 4)*internal_51;
    Q_xx_(6, 5) = Fx(0, 5)*internal_47 + Fx(1, 5)*internal_48 + Fx(2, 5)*internal_49 + Fx(3, 5)*internal_50 + Fx(5, 5)*internal_52;
    Q_xx_(6, 6) = Fx(0, 6)*internal_47 + Fx(1, 6)*internal_48 + Fx(2, 6)*internal_49 + Fx(3, 6)*internal_50 + Fx(5, 6)*internal_52 + Fx(6, 6)*internal_53 + l_xx(6, 6);

    // Evaluation of Matrix Q_xu_
    Q_xu_(0, 0) = Fu(1, 0)*internal_1 + Fu(2, 0)*internal_3 + Fu(3, 0)*internal_5 + Fu(4, 0)*internal_7 + Fx(0, 0)*internal_54;
    Q_xu_(0, 1) = Fu(1, 1)*internal_1 + Fu(2, 1)*internal_3 + Fu(3, 1)*internal_5 + Fu(5, 1)*internal_9 + Fu(6, 1)*internal_11 + Fx(0, 0)*internal_55;
    Q_xu_(1, 0) = Fu(0, 0)*internal_12 + Fu(1, 0)*internal_13 + Fu(2, 0)*internal_14 + Fu(3, 0)*internal_15 + Fu(4, 0)*internal_16;
    Q_xu_(1, 1) = Fu(0, 1)*internal_12 + Fu(1, 1)*internal_13 + Fu(2, 1)*internal_14 + Fu(3, 1)*internal_15 + Fu(5, 1)*internal_17 + Fu(6, 1)*internal_18;
    Q_xu_(2, 0) = Fu(0, 0)*internal_19 + Fu(1, 0)*internal_20 + Fu(2, 0)*internal_21 + Fu(3, 0)*internal_22 + Fu(4, 0)*internal_23;
    Q_xu_(2, 1) = Fu(0, 1)*internal_19 + Fu(1, 1)*internal_20 + Fu(2, 1)*internal_21 + Fu(3, 1)*internal_22 + Fu(5, 1)*internal_24 + Fu(6, 1)*internal_25;
    Q_xu_(3, 0) = Fu(0, 0)*internal_26 + Fu(1, 0)*internal_27 + Fu(2, 0)*internal_28 + Fu(3, 0)*internal_29 + Fu(4, 0)*internal_30;
    Q_xu_(3, 1) = Fu(0, 1)*internal_26 + Fu(1, 1)*internal_27 + Fu(2, 1)*internal_28 + Fu(3, 1)*internal_29 + Fu(5, 1)*internal_31 + Fu(6, 1)*internal_32;
    Q_xu_(4, 0) = Fu(0, 0)*internal_33 + Fu(1, 0)*internal_34 + Fu(2, 0)*internal_35 + Fu(3, 0)*internal_36 + Fu(4, 0)*internal_37;
    Q_xu_(4, 1) = Fu(0, 1)*internal_33 + Fu(1, 1)*internal_34 + Fu(2, 1)*internal_35 + Fu(3, 1)*internal_36 + Fu(5, 1)*internal_38 + Fu(6, 1)*internal_39;
    Q_xu_(5, 0) = Fu(0, 0)*internal_40 + Fu(1, 0)*internal_41 + Fu(2, 0)*internal_42 + Fu(3, 0)*internal_43 + Fu(4, 0)*internal_44;
    Q_xu_(5, 1) = Fu(0, 1)*internal_40 + Fu(1, 1)*internal_41 + Fu(2, 1)*internal_42 + Fu(3, 1)*internal_43 + Fu(5, 1)*internal_45 + Fu(6, 1)*internal_46;
    Q_xu_(6, 0) = Fu(0, 0)*internal_47 + Fu(1, 0)*internal_48 + Fu(2, 0)*internal_49 + Fu(3, 0)*internal_50 + Fu(4, 0)*internal_51;
    Q_xu_(6, 1) = Fu(0, 1)*internal_47 + Fu(1, 1)*internal_48 + Fu(2, 1)*internal_49 + Fu(3, 1)*internal_50 + Fu(5, 1)*internal_52 + Fu(6, 1)*internal_53;

    // Evaluation of Matrix Q_uu_
    Q_uu_(0, 0) = Fu(0, 0)*internal_56 + Fu(1, 0)*internal_57 + Fu(2, 0)*internal_58 + Fu(3, 0)*internal_59 + Fu(4, 0)*(Fu(0, 0)*V_xx_prev(0, 4) + Fu(1, 0)*V_xx_prev(1, 4) + Fu(2, 0)*V_xx_prev(2, 4) + Fu(3, 0)*V_xx_prev(3, 4) + Fu(4, 0)*V_xx_prev(4, 4)) + Sigma_(0)*pow(g_u(0, 0), 2) + Sigma_(1)*pow(g_u(1, 0), 2) + l_uu(0, 0);
    Q_uu_(0, 1) = Fu(0, 1)*internal_56 + Fu(1, 1)*internal_57 + Fu(2, 1)*internal_58 + Fu(3, 1)*internal_59 + Fu(5, 1)*(Fu(0, 0)*V_xx_prev(0, 5) + Fu(1, 0)*V_xx_prev(1, 5) + Fu(2, 0)*V_xx_prev(2, 5) + Fu(3, 0)*V_xx_prev(3, 5) + Fu(4, 0)*V_xx_prev(4, 5)) + Fu(6, 1)*(Fu(0, 0)*V_xx_prev(0, 6) + Fu(1, 0)*V_xx_prev(1, 6) + Fu(2, 0)*V_xx_prev(2, 6) + Fu(3, 0)*V_xx_prev(3, 6) + Fu(4, 0)*V_xx_prev(4, 6));
    Q_uu_(1, 0) = Fu(0, 0)*internal_60 + Fu(1, 0)*internal_61 + Fu(2, 0)*internal_62 + Fu(3, 0)*internal_63 + Fu(4, 0)*(Fu(0, 1)*V_xx_prev(0, 4) + Fu(1, 1)*V_xx_prev(1, 4) + Fu(2, 1)*V_xx_prev(2, 4) + Fu(3, 1)*V_xx_prev(3, 4) + Fu(5, 1)*V_xx_prev(5, 4) + Fu(6, 1)*V_xx_prev(6, 4));
    Q_uu_(1, 1) = Fu(0, 1)*internal_60 + Fu(1, 1)*internal_61 + Fu(2, 1)*internal_62 + Fu(3, 1)*internal_63 + Fu(5, 1)*(Fu(0, 1)*V_xx_prev(0, 5) + Fu(1, 1)*V_xx_prev(1, 5) + Fu(2, 1)*V_xx_prev(2, 5) + Fu(3, 1)*V_xx_prev(3, 5) + Fu(5, 1)*V_xx_prev(5, 5) + Fu(6, 1)*V_xx_prev(6, 5)) + Fu(6, 1)*(Fu(0, 1)*V_xx_prev(0, 6) + Fu(1, 1)*V_xx_prev(1, 6) + Fu(2, 1)*V_xx_prev(2, 6) + Fu(3, 1)*V_xx_prev(3, 6) + Fu(5, 1)*V_xx_prev(5, 6) + Fu(6, 1)*V_xx_prev(6, 6)) + Sigma_(2)*pow(g_u(2, 1), 2) + Sigma_(3)*pow(g_u(3, 1), 2) + l_uu(1, 1);
    Q_uu_(2, 2) = l_uu(2, 2);
    Q_uu_(3, 3) = l_uu(3, 3);
    Q_uu_(4, 4) = l_uu(4, 4);
    Q_uu_(5, 5) = l_uu(5, 5);
    Q_uu_(6, 6) = l_uu(6, 6);
    Q_uu_(7, 7) = l_uu(7, 7);
    Q_uu_(8, 8) = l_uu(8, 8);
    Q_uu_(9, 9) = l_uu(9, 9);
    Q_uu_(10, 10) = l_uu(10, 10);
    Q_uu_(11, 11) = l_uu(11, 11);
    Q_uu_(12, 12) = l_uu(12, 12);

  }

  virtual void updateQG(const Eigen::VectorXd& sl_x, const Eigen::VectorXd& sl_u, const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::VectorXd& r_f, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& V_xx_prev, const Eigen::VectorXd& V_x_prev) override {
    // Determine internal variables
    const double internal_0 = V_x_prev(0) + V_xx_prev(0, 0)*r_f(0) + V_xx_prev(0, 1)*r_f(1) + V_xx_prev(0, 2)*r_f(2) + V_xx_prev(0, 3)*r_f(3) + V_xx_prev(0, 4)*r_f(4) + V_xx_prev(0, 5)*r_f(5) + V_xx_prev(0, 6)*r_f(6);
    const double internal_1 = V_x_prev(1) + V_xx_prev(1, 0)*r_f(0) + V_xx_prev(1, 1)*r_f(1) + V_xx_prev(1, 2)*r_f(2) + V_xx_prev(1, 3)*r_f(3) + V_xx_prev(1, 4)*r_f(4) + V_xx_prev(1, 5)*r_f(5) + V_xx_prev(1, 6)*r_f(6);
    const double internal_2 = V_x_prev(2) + V_xx_prev(2, 0)*r_f(0) + V_xx_prev(2, 1)*r_f(1) + V_xx_prev(2, 2)*r_f(2) + V_xx_prev(2, 3)*r_f(3) + V_xx_prev(2, 4)*r_f(4) + V_xx_prev(2, 5)*r_f(5) + V_xx_prev(2, 6)*r_f(6);
    const double internal_3 = V_x_prev(3) + V_xx_prev(3, 0)*r_f(0) + V_xx_prev(3, 1)*r_f(1) + V_xx_prev(3, 2)*r_f(2) + V_xx_prev(3, 3)*r_f(3) + V_xx_prev(3, 4)*r_f(4) + V_xx_prev(3, 5)*r_f(5) + V_xx_prev(3, 6)*r_f(6);
    const double internal_4 = V_x_prev(4) + V_xx_prev(4, 0)*r_f(0) + V_xx_prev(4, 1)*r_f(1) + V_xx_prev(4, 2)*r_f(2) + V_xx_prev(4, 3)*r_f(3) + V_xx_prev(4, 4)*r_f(4) + V_xx_prev(4, 5)*r_f(5) + V_xx_prev(4, 6)*r_f(6);
    const double internal_5 = V_x_prev(5) + V_xx_prev(5, 0)*r_f(0) + V_xx_prev(5, 1)*r_f(1) + V_xx_prev(5, 2)*r_f(2) + V_xx_prev(5, 3)*r_f(3) + V_xx_prev(5, 4)*r_f(4) + V_xx_prev(5, 5)*r_f(5) + V_xx_prev(5, 6)*r_f(6);
    const double internal_6 = V_x_prev(6) + V_xx_prev(6, 0)*r_f(0) + V_xx_prev(6, 1)*r_f(1) + V_xx_prev(6, 2)*r_f(2) + V_xx_prev(6, 3)*r_f(3) + V_xx_prev(6, 4)*r_f(4) + V_xx_prev(6, 5)*r_f(5) + V_xx_prev(6, 6)*r_f(6);

    // Evaluation of Vector Q_x_
    Q_x_(0) = Fx(0, 0)*internal_0 + sl_x(0);
    Q_x_(1) = Fx(0, 1)*internal_0 + Fx(1, 1)*internal_1 + sl_x(1);
    Q_x_(2) = Fx(0, 2)*internal_0 + Fx(2, 2)*internal_2 + sl_x(2);
    Q_x_(3) = Fx(0, 3)*internal_0 + Fx(1, 3)*internal_1 + Fx(2, 3)*internal_2 + Fx(3, 3)*internal_3 + sl_x(3);
    Q_x_(4) = Fx(0, 4)*internal_0 + Fx(1, 4)*internal_1 + Fx(2, 4)*internal_2 + Fx(3, 4)*internal_3 + Fx(4, 4)*internal_4 + sl_x(4);
    Q_x_(5) = Fx(0, 5)*internal_0 + Fx(1, 5)*internal_1 + Fx(2, 5)*internal_2 + Fx(3, 5)*internal_3 + Fx(5, 5)*internal_5 + sl_x(5);
    Q_x_(6) = Fx(0, 6)*internal_0 + Fx(1, 6)*internal_1 + Fx(2, 6)*internal_2 + Fx(3, 6)*internal_3 + Fx(5, 6)*internal_5 + Fx(6, 6)*internal_6 + sl_x(6);

    // Evaluation of Vector Q_u_
    Q_u_(0) = Fu(0, 0)*internal_0 + Fu(1, 0)*internal_1 + Fu(2, 0)*internal_2 + Fu(3, 0)*internal_3 + Fu(4, 0)*internal_4 + V_(0)*g_u(0, 0) + V_(1)*g_u(1, 0) + sl_u(0);
    Q_u_(1) = Fu(0, 1)*internal_0 + Fu(1, 1)*internal_1 + Fu(2, 1)*internal_2 + Fu(3, 1)*internal_3 + Fu(5, 1)*internal_5 + Fu(6, 1)*internal_6 + V_(2)*g_u(2, 1) + V_(3)*g_u(3, 1) + sl_u(1);
    Q_u_(2) = sl_u(2);
    Q_u_(3) = sl_u(3);
    Q_u_(4) = sl_u(4);
    Q_u_(5) = sl_u(5);
    Q_u_(6) = sl_u(6);
    Q_u_(7) = sl_u(7);
    Q_u_(8) = sl_u(8);
    Q_u_(9) = sl_u(9);
    Q_u_(10) = sl_u(10);
    Q_u_(11) = sl_u(11);
    Q_u_(12) = sl_u(12);

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
    Q_uu_inv_(3, 3) = 1.0/Q_uu_(3, 3);
    Q_uu_inv_(4, 4) = 1.0/Q_uu_(4, 4);
    Q_uu_inv_(5, 5) = 1.0/Q_uu_(5, 5);
    Q_uu_inv_(6, 6) = 1.0/Q_uu_(6, 6);
    Q_uu_inv_(7, 7) = 1.0/Q_uu_(7, 7);
    Q_uu_inv_(8, 8) = 1.0/Q_uu_(8, 8);
    Q_uu_inv_(9, 9) = 1.0/Q_uu_(9, 9);
    Q_uu_inv_(10, 10) = 1.0/Q_uu_(10, 10);
    Q_uu_inv_(11, 11) = 1.0/Q_uu_(11, 11);
    Q_uu_inv_(12, 12) = 1.0/Q_uu_(12, 12);

  }

  virtual void updateKVU(Eigen::MatrixXd& k_ux, Eigen::VectorXd& v_u) override {
    // Evaluation of Matrix k_ux
    k_ux(0, 0) = -Q_uu_inv_(0, 0)*Q_xu_(0, 0) - Q_uu_inv_(0, 1)*Q_xu_(0, 1);
    k_ux(0, 1) = -Q_uu_inv_(0, 0)*Q_xu_(1, 0) - Q_uu_inv_(0, 1)*Q_xu_(1, 1);
    k_ux(0, 2) = -Q_uu_inv_(0, 0)*Q_xu_(2, 0) - Q_uu_inv_(0, 1)*Q_xu_(2, 1);
    k_ux(0, 3) = -Q_uu_inv_(0, 0)*Q_xu_(3, 0) - Q_uu_inv_(0, 1)*Q_xu_(3, 1);
    k_ux(0, 4) = -Q_uu_inv_(0, 0)*Q_xu_(4, 0) - Q_uu_inv_(0, 1)*Q_xu_(4, 1);
    k_ux(0, 5) = -Q_uu_inv_(0, 0)*Q_xu_(5, 0) - Q_uu_inv_(0, 1)*Q_xu_(5, 1);
    k_ux(0, 6) = -Q_uu_inv_(0, 0)*Q_xu_(6, 0) - Q_uu_inv_(0, 1)*Q_xu_(6, 1);
    k_ux(1, 0) = -Q_uu_inv_(0, 1)*Q_xu_(0, 0) - Q_uu_inv_(1, 1)*Q_xu_(0, 1);
    k_ux(1, 1) = -Q_uu_inv_(0, 1)*Q_xu_(1, 0) - Q_uu_inv_(1, 1)*Q_xu_(1, 1);
    k_ux(1, 2) = -Q_uu_inv_(0, 1)*Q_xu_(2, 0) - Q_uu_inv_(1, 1)*Q_xu_(2, 1);
    k_ux(1, 3) = -Q_uu_inv_(0, 1)*Q_xu_(3, 0) - Q_uu_inv_(1, 1)*Q_xu_(3, 1);
    k_ux(1, 4) = -Q_uu_inv_(0, 1)*Q_xu_(4, 0) - Q_uu_inv_(1, 1)*Q_xu_(4, 1);
    k_ux(1, 5) = -Q_uu_inv_(0, 1)*Q_xu_(5, 0) - Q_uu_inv_(1, 1)*Q_xu_(5, 1);
    k_ux(1, 6) = -Q_uu_inv_(0, 1)*Q_xu_(6, 0) - Q_uu_inv_(1, 1)*Q_xu_(6, 1);

    // Evaluation of Vector v_u
    v_u(0) = -Q_u_(0)*Q_uu_inv_(0, 0) - Q_u_(1)*Q_uu_inv_(0, 1);
    v_u(1) = -Q_u_(0)*Q_uu_inv_(0, 1) - Q_u_(1)*Q_uu_inv_(1, 1);
    v_u(2) = -Q_u_(2)*Q_uu_inv_(2, 2);
    v_u(3) = -Q_u_(3)*Q_uu_inv_(3, 3);
    v_u(4) = -Q_u_(4)*Q_uu_inv_(4, 4);
    v_u(5) = -Q_u_(5)*Q_uu_inv_(5, 5);
    v_u(6) = -Q_u_(6)*Q_uu_inv_(6, 6);
    v_u(7) = -Q_u_(7)*Q_uu_inv_(7, 7);
    v_u(8) = -Q_u_(8)*Q_uu_inv_(8, 8);
    v_u(9) = -Q_u_(9)*Q_uu_inv_(9, 9);
    v_u(10) = -Q_u_(10)*Q_uu_inv_(10, 10);
    v_u(11) = -Q_u_(11)*Q_uu_inv_(11, 11);
    v_u(12) = -Q_u_(12)*Q_uu_inv_(12, 12);

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
    V_xx(0, 6) = Q_xu_(0, 0)*k_ux(0, 6) + Q_xu_(0, 1)*k_ux(1, 6) + Q_xx_(0, 6);
    V_xx(1, 0) = Q_xu_(1, 0)*k_ux(0, 0) + Q_xu_(1, 1)*k_ux(1, 0) + Q_xx_(0, 1);
    V_xx(1, 1) = Q_xu_(1, 0)*k_ux(0, 1) + Q_xu_(1, 1)*k_ux(1, 1) + Q_xx_(1, 1);
    V_xx(1, 2) = Q_xu_(1, 0)*k_ux(0, 2) + Q_xu_(1, 1)*k_ux(1, 2) + Q_xx_(1, 2);
    V_xx(1, 3) = Q_xu_(1, 0)*k_ux(0, 3) + Q_xu_(1, 1)*k_ux(1, 3) + Q_xx_(1, 3);
    V_xx(1, 4) = Q_xu_(1, 0)*k_ux(0, 4) + Q_xu_(1, 1)*k_ux(1, 4) + Q_xx_(1, 4);
    V_xx(1, 5) = Q_xu_(1, 0)*k_ux(0, 5) + Q_xu_(1, 1)*k_ux(1, 5) + Q_xx_(1, 5);
    V_xx(1, 6) = Q_xu_(1, 0)*k_ux(0, 6) + Q_xu_(1, 1)*k_ux(1, 6) + Q_xx_(1, 6);
    V_xx(2, 0) = Q_xu_(2, 0)*k_ux(0, 0) + Q_xu_(2, 1)*k_ux(1, 0) + Q_xx_(0, 2);
    V_xx(2, 1) = Q_xu_(2, 0)*k_ux(0, 1) + Q_xu_(2, 1)*k_ux(1, 1) + Q_xx_(1, 2);
    V_xx(2, 2) = Q_xu_(2, 0)*k_ux(0, 2) + Q_xu_(2, 1)*k_ux(1, 2) + Q_xx_(2, 2);
    V_xx(2, 3) = Q_xu_(2, 0)*k_ux(0, 3) + Q_xu_(2, 1)*k_ux(1, 3) + Q_xx_(2, 3);
    V_xx(2, 4) = Q_xu_(2, 0)*k_ux(0, 4) + Q_xu_(2, 1)*k_ux(1, 4) + Q_xx_(2, 4);
    V_xx(2, 5) = Q_xu_(2, 0)*k_ux(0, 5) + Q_xu_(2, 1)*k_ux(1, 5) + Q_xx_(2, 5);
    V_xx(2, 6) = Q_xu_(2, 0)*k_ux(0, 6) + Q_xu_(2, 1)*k_ux(1, 6) + Q_xx_(2, 6);
    V_xx(3, 0) = Q_xu_(3, 0)*k_ux(0, 0) + Q_xu_(3, 1)*k_ux(1, 0) + Q_xx_(0, 3);
    V_xx(3, 1) = Q_xu_(3, 0)*k_ux(0, 1) + Q_xu_(3, 1)*k_ux(1, 1) + Q_xx_(1, 3);
    V_xx(3, 2) = Q_xu_(3, 0)*k_ux(0, 2) + Q_xu_(3, 1)*k_ux(1, 2) + Q_xx_(2, 3);
    V_xx(3, 3) = Q_xu_(3, 0)*k_ux(0, 3) + Q_xu_(3, 1)*k_ux(1, 3) + Q_xx_(3, 3);
    V_xx(3, 4) = Q_xu_(3, 0)*k_ux(0, 4) + Q_xu_(3, 1)*k_ux(1, 4) + Q_xx_(3, 4);
    V_xx(3, 5) = Q_xu_(3, 0)*k_ux(0, 5) + Q_xu_(3, 1)*k_ux(1, 5) + Q_xx_(3, 5);
    V_xx(3, 6) = Q_xu_(3, 0)*k_ux(0, 6) + Q_xu_(3, 1)*k_ux(1, 6) + Q_xx_(3, 6);
    V_xx(4, 0) = Q_xu_(4, 0)*k_ux(0, 0) + Q_xu_(4, 1)*k_ux(1, 0) + Q_xx_(0, 4);
    V_xx(4, 1) = Q_xu_(4, 0)*k_ux(0, 1) + Q_xu_(4, 1)*k_ux(1, 1) + Q_xx_(1, 4);
    V_xx(4, 2) = Q_xu_(4, 0)*k_ux(0, 2) + Q_xu_(4, 1)*k_ux(1, 2) + Q_xx_(2, 4);
    V_xx(4, 3) = Q_xu_(4, 0)*k_ux(0, 3) + Q_xu_(4, 1)*k_ux(1, 3) + Q_xx_(3, 4);
    V_xx(4, 4) = Q_xu_(4, 0)*k_ux(0, 4) + Q_xu_(4, 1)*k_ux(1, 4) + Q_xx_(4, 4);
    V_xx(4, 5) = Q_xu_(4, 0)*k_ux(0, 5) + Q_xu_(4, 1)*k_ux(1, 5) + Q_xx_(4, 5);
    V_xx(4, 6) = Q_xu_(4, 0)*k_ux(0, 6) + Q_xu_(4, 1)*k_ux(1, 6) + Q_xx_(4, 6);
    V_xx(5, 0) = Q_xu_(5, 0)*k_ux(0, 0) + Q_xu_(5, 1)*k_ux(1, 0) + Q_xx_(0, 5);
    V_xx(5, 1) = Q_xu_(5, 0)*k_ux(0, 1) + Q_xu_(5, 1)*k_ux(1, 1) + Q_xx_(1, 5);
    V_xx(5, 2) = Q_xu_(5, 0)*k_ux(0, 2) + Q_xu_(5, 1)*k_ux(1, 2) + Q_xx_(2, 5);
    V_xx(5, 3) = Q_xu_(5, 0)*k_ux(0, 3) + Q_xu_(5, 1)*k_ux(1, 3) + Q_xx_(3, 5);
    V_xx(5, 4) = Q_xu_(5, 0)*k_ux(0, 4) + Q_xu_(5, 1)*k_ux(1, 4) + Q_xx_(4, 5);
    V_xx(5, 5) = Q_xu_(5, 0)*k_ux(0, 5) + Q_xu_(5, 1)*k_ux(1, 5) + Q_xx_(5, 5);
    V_xx(5, 6) = Q_xu_(5, 0)*k_ux(0, 6) + Q_xu_(5, 1)*k_ux(1, 6) + Q_xx_(5, 6);
    V_xx(6, 0) = Q_xu_(6, 0)*k_ux(0, 0) + Q_xu_(6, 1)*k_ux(1, 0) + Q_xx_(0, 6);
    V_xx(6, 1) = Q_xu_(6, 0)*k_ux(0, 1) + Q_xu_(6, 1)*k_ux(1, 1) + Q_xx_(1, 6);
    V_xx(6, 2) = Q_xu_(6, 0)*k_ux(0, 2) + Q_xu_(6, 1)*k_ux(1, 2) + Q_xx_(2, 6);
    V_xx(6, 3) = Q_xu_(6, 0)*k_ux(0, 3) + Q_xu_(6, 1)*k_ux(1, 3) + Q_xx_(3, 6);
    V_xx(6, 4) = Q_xu_(6, 0)*k_ux(0, 4) + Q_xu_(6, 1)*k_ux(1, 4) + Q_xx_(4, 6);
    V_xx(6, 5) = Q_xu_(6, 0)*k_ux(0, 5) + Q_xu_(6, 1)*k_ux(1, 5) + Q_xx_(5, 6);
    V_xx(6, 6) = Q_xu_(6, 0)*k_ux(0, 6) + Q_xu_(6, 1)*k_ux(1, 6) + Q_xx_(6, 6);

    // Evaluation of Vector V_x
    V_x(0) = Q_x_(0) + Q_xu_(0, 0)*v_u(0) + Q_xu_(0, 1)*v_u(1);
    V_x(1) = Q_x_(1) + Q_xu_(1, 0)*v_u(0) + Q_xu_(1, 1)*v_u(1);
    V_x(2) = Q_x_(2) + Q_xu_(2, 0)*v_u(0) + Q_xu_(2, 1)*v_u(1);
    V_x(3) = Q_x_(3) + Q_xu_(3, 0)*v_u(0) + Q_xu_(3, 1)*v_u(1);
    V_x(4) = Q_x_(4) + Q_xu_(4, 0)*v_u(0) + Q_xu_(4, 1)*v_u(1);
    V_x(5) = Q_x_(5) + Q_xu_(5, 0)*v_u(0) + Q_xu_(5, 1)*v_u(1);
    V_x(6) = Q_x_(6) + Q_xu_(6, 0)*v_u(0) + Q_xu_(6, 1)*v_u(1);

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
    Q_un_(0, 6) = -Fu(6, 1)*Q_uu_inv_(0, 1);
    Q_un_(1, 0) = -Fu(0, 0)*Q_uu_inv_(0, 1) - Fu(0, 1)*Q_uu_inv_(1, 1);
    Q_un_(1, 1) = -Fu(1, 0)*Q_uu_inv_(0, 1) - Fu(1, 1)*Q_uu_inv_(1, 1);
    Q_un_(1, 2) = -Fu(2, 0)*Q_uu_inv_(0, 1) - Fu(2, 1)*Q_uu_inv_(1, 1);
    Q_un_(1, 3) = -Fu(3, 0)*Q_uu_inv_(0, 1) - Fu(3, 1)*Q_uu_inv_(1, 1);
    Q_un_(1, 4) = -Fu(4, 0)*Q_uu_inv_(0, 1);
    Q_un_(1, 5) = -Fu(5, 1)*Q_uu_inv_(1, 1);
    Q_un_(1, 6) = -Fu(6, 1)*Q_uu_inv_(1, 1);

    // Evaluation of Matrix Q_nuu_
    Q_nuu_(0, 0) = -Q_uu_(0, 0);
    Q_nuu_(0, 1) = internal_0;
    Q_nuu_(1, 0) = internal_0;
    Q_nuu_(1, 1) = -Q_uu_(1, 1);
    Q_nuu_(2, 2) = -Q_uu_(2, 2);
    Q_nuu_(3, 3) = -Q_uu_(3, 3);
    Q_nuu_(4, 4) = -Q_uu_(4, 4);
    Q_nuu_(5, 5) = -Q_uu_(5, 5);
    Q_nuu_(6, 6) = -Q_uu_(6, 6);
    Q_nuu_(7, 7) = -Q_uu_(7, 7);
    Q_nuu_(8, 8) = -Q_uu_(8, 8);
    Q_nuu_(9, 9) = -Q_uu_(9, 9);
    Q_nuu_(10, 10) = -Q_uu_(10, 10);
    Q_nuu_(11, 11) = -Q_uu_(11, 11);
    Q_nuu_(12, 12) = -Q_uu_(12, 12);

    // Evaluation of Matrix Q_vnx_
    Q_vnx_(0, 0) = Fu(0, 0)*k_ux(0, 0) + Fu(0, 1)*k_ux(1, 0) + Fx(0, 0);
    Q_vnx_(0, 1) = Fu(0, 0)*k_ux(0, 1) + Fu(0, 1)*k_ux(1, 1) + Fx(0, 1);
    Q_vnx_(0, 2) = Fu(0, 0)*k_ux(0, 2) + Fu(0, 1)*k_ux(1, 2) + Fx(0, 2);
    Q_vnx_(0, 3) = Fu(0, 0)*k_ux(0, 3) + Fu(0, 1)*k_ux(1, 3) + Fx(0, 3);
    Q_vnx_(0, 4) = Fu(0, 0)*k_ux(0, 4) + Fu(0, 1)*k_ux(1, 4) + Fx(0, 4);
    Q_vnx_(0, 5) = Fu(0, 0)*k_ux(0, 5) + Fu(0, 1)*k_ux(1, 5) + Fx(0, 5);
    Q_vnx_(0, 6) = Fu(0, 0)*k_ux(0, 6) + Fu(0, 1)*k_ux(1, 6) + Fx(0, 6);
    Q_vnx_(1, 0) = Fu(1, 0)*k_ux(0, 0) + Fu(1, 1)*k_ux(1, 0);
    Q_vnx_(1, 1) = Fu(1, 0)*k_ux(0, 1) + Fu(1, 1)*k_ux(1, 1) + Fx(1, 1);
    Q_vnx_(1, 2) = Fu(1, 0)*k_ux(0, 2) + Fu(1, 1)*k_ux(1, 2);
    Q_vnx_(1, 3) = Fu(1, 0)*k_ux(0, 3) + Fu(1, 1)*k_ux(1, 3) + Fx(1, 3);
    Q_vnx_(1, 4) = Fu(1, 0)*k_ux(0, 4) + Fu(1, 1)*k_ux(1, 4) + Fx(1, 4);
    Q_vnx_(1, 5) = Fu(1, 0)*k_ux(0, 5) + Fu(1, 1)*k_ux(1, 5) + Fx(1, 5);
    Q_vnx_(1, 6) = Fu(1, 0)*k_ux(0, 6) + Fu(1, 1)*k_ux(1, 6) + Fx(1, 6);
    Q_vnx_(2, 0) = Fu(2, 0)*k_ux(0, 0) + Fu(2, 1)*k_ux(1, 0);
    Q_vnx_(2, 1) = Fu(2, 0)*k_ux(0, 1) + Fu(2, 1)*k_ux(1, 1);
    Q_vnx_(2, 2) = Fu(2, 0)*k_ux(0, 2) + Fu(2, 1)*k_ux(1, 2) + Fx(2, 2);
    Q_vnx_(2, 3) = Fu(2, 0)*k_ux(0, 3) + Fu(2, 1)*k_ux(1, 3) + Fx(2, 3);
    Q_vnx_(2, 4) = Fu(2, 0)*k_ux(0, 4) + Fu(2, 1)*k_ux(1, 4) + Fx(2, 4);
    Q_vnx_(2, 5) = Fu(2, 0)*k_ux(0, 5) + Fu(2, 1)*k_ux(1, 5) + Fx(2, 5);
    Q_vnx_(2, 6) = Fu(2, 0)*k_ux(0, 6) + Fu(2, 1)*k_ux(1, 6) + Fx(2, 6);
    Q_vnx_(3, 0) = Fu(3, 0)*k_ux(0, 0) + Fu(3, 1)*k_ux(1, 0);
    Q_vnx_(3, 1) = Fu(3, 0)*k_ux(0, 1) + Fu(3, 1)*k_ux(1, 1);
    Q_vnx_(3, 2) = Fu(3, 0)*k_ux(0, 2) + Fu(3, 1)*k_ux(1, 2);
    Q_vnx_(3, 3) = Fu(3, 0)*k_ux(0, 3) + Fu(3, 1)*k_ux(1, 3) + Fx(3, 3);
    Q_vnx_(3, 4) = Fu(3, 0)*k_ux(0, 4) + Fu(3, 1)*k_ux(1, 4) + Fx(3, 4);
    Q_vnx_(3, 5) = Fu(3, 0)*k_ux(0, 5) + Fu(3, 1)*k_ux(1, 5) + Fx(3, 5);
    Q_vnx_(3, 6) = Fu(3, 0)*k_ux(0, 6) + Fu(3, 1)*k_ux(1, 6) + Fx(3, 6);
    Q_vnx_(4, 0) = Fu(4, 0)*k_ux(0, 0);
    Q_vnx_(4, 1) = Fu(4, 0)*k_ux(0, 1);
    Q_vnx_(4, 2) = Fu(4, 0)*k_ux(0, 2);
    Q_vnx_(4, 3) = Fu(4, 0)*k_ux(0, 3);
    Q_vnx_(4, 4) = Fu(4, 0)*k_ux(0, 4) + Fx(4, 4);
    Q_vnx_(4, 5) = Fu(4, 0)*k_ux(0, 5);
    Q_vnx_(4, 6) = Fu(4, 0)*k_ux(0, 6);
    Q_vnx_(5, 0) = Fu(5, 1)*k_ux(1, 0);
    Q_vnx_(5, 1) = Fu(5, 1)*k_ux(1, 1);
    Q_vnx_(5, 2) = Fu(5, 1)*k_ux(1, 2);
    Q_vnx_(5, 3) = Fu(5, 1)*k_ux(1, 3);
    Q_vnx_(5, 4) = Fu(5, 1)*k_ux(1, 4);
    Q_vnx_(5, 5) = Fu(5, 1)*k_ux(1, 5) + Fx(5, 5);
    Q_vnx_(5, 6) = Fu(5, 1)*k_ux(1, 6) + Fx(5, 6);
    Q_vnx_(6, 0) = Fu(6, 1)*k_ux(1, 0);
    Q_vnx_(6, 1) = Fu(6, 1)*k_ux(1, 1);
    Q_vnx_(6, 2) = Fu(6, 1)*k_ux(1, 2);
    Q_vnx_(6, 3) = Fu(6, 1)*k_ux(1, 3);
    Q_vnx_(6, 4) = Fu(6, 1)*k_ux(1, 4);
    Q_vnx_(6, 5) = Fu(6, 1)*k_ux(1, 5);
    Q_vnx_(6, 6) = Fu(6, 1)*k_ux(1, 6) + Fx(6, 6);

    // Evaluation of Vector Q_vn_
    Q_vn_(0) = Fu(0, 0)*v_u(0) + Fu(0, 1)*v_u(1) + r_f(0);
    Q_vn_(1) = Fu(1, 0)*v_u(0) + Fu(1, 1)*v_u(1) + r_f(1);
    Q_vn_(2) = Fu(2, 0)*v_u(0) + Fu(2, 1)*v_u(1) + r_f(2);
    Q_vn_(3) = Fu(3, 0)*v_u(0) + Fu(3, 1)*v_u(1) + r_f(3);
    Q_vn_(4) = Fu(4, 0)*v_u(0) + r_f(4);
    Q_vn_(5) = Fu(5, 1)*v_u(1) + r_f(5);
    Q_vn_(6) = Fu(6, 1)*v_u(1) + r_f(6);

  }

};

class SpatiotemporalPlannerModelIpmEvaluatorTerminal : public IpmEvaluatorTerminal {
 public:
  SpatiotemporalPlannerModelIpmEvaluatorTerminal() : IpmEvaluatorTerminal(7, 13, 15, 0) {}
  virtual ~SpatiotemporalPlannerModelIpmEvaluatorTerminal() = default;

  virtual void updateLX(const Eigen::VectorXd& l_x, const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& m_x, const Eigen::VectorXd& p, const Eigen::VectorXd& lambda, const Eigen::VectorXd& nu, Eigen::VectorXd& sl_x) const override {
    // Evaluation of Vector sl_x
    sl_x(0) = g_x(8, 0)*lambda(8) + g_x(9, 0)*lambda(9) + l_x(0) - p(0);
    sl_x(1) = g_x(0, 1)*lambda(0) + g_x(1, 1)*lambda(1) + g_x(2, 1)*lambda(2) + g_x(3, 1)*lambda(3) + g_x(4, 1)*lambda(4) + g_x(5, 1)*lambda(5) + l_x(1) - p(1);
    sl_x(2) = g_x(0, 2)*lambda(0) + g_x(1, 2)*lambda(1) + g_x(2, 2)*lambda(2) + g_x(3, 2)*lambda(3) + g_x(4, 2)*lambda(4) + g_x(5, 2)*lambda(5) + l_x(2) - p(2);
    sl_x(3) = g_x(14, 3)*lambda(14) + g_x(2, 3)*lambda(2) + g_x(3, 3)*lambda(3) + g_x(4, 3)*lambda(4) + g_x(5, 3)*lambda(5) + l_x(3) - p(3);
    sl_x(4) = g_x(6, 4)*lambda(6) + g_x(7, 4)*lambda(7) - p(4);
    sl_x(5) = g_x(10, 5)*lambda(10) + g_x(11, 5)*lambda(11) + l_x(5) - p(5);
    sl_x(6) = g_x(12, 6)*lambda(12) + g_x(13, 6)*lambda(13) - p(6);

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
    V_xx(0, 0) = Sigma_(8)*pow(g_x(8, 0), 2) + Sigma_(9)*pow(g_x(9, 0), 2) + l_xx(0, 0);
    V_xx(1, 1) = Sigma_(0)*pow(g_x(0, 1), 2) + Sigma_(1)*pow(g_x(1, 1), 2) + Sigma_(2)*pow(g_x(2, 1), 2) + Sigma_(3)*pow(g_x(3, 1), 2) + Sigma_(4)*pow(g_x(4, 1), 2) + Sigma_(5)*pow(g_x(5, 1), 2) + l_xx(1, 1);
    V_xx(1, 2) = internal_4 + l_xx(1, 2);
    V_xx(1, 3) = internal_5;
    V_xx(2, 1) = internal_4 + l_xx(2, 1);
    V_xx(2, 2) = Sigma_(0)*pow(g_x(0, 2), 2) + Sigma_(1)*pow(g_x(1, 2), 2) + Sigma_(2)*pow(g_x(2, 2), 2) + Sigma_(3)*pow(g_x(3, 2), 2) + Sigma_(4)*pow(g_x(4, 2), 2) + Sigma_(5)*pow(g_x(5, 2), 2) + l_xx(2, 2);
    V_xx(2, 3) = internal_6;
    V_xx(3, 1) = internal_5;
    V_xx(3, 2) = internal_6;
    V_xx(3, 3) = Sigma_(14)*pow(g_x(14, 3), 2) + Sigma_(2)*pow(g_x(2, 3), 2) + Sigma_(3)*pow(g_x(3, 3), 2) + Sigma_(4)*pow(g_x(4, 3), 2) + Sigma_(5)*pow(g_x(5, 3), 2) + l_xx(3, 3);
    V_xx(4, 4) = Sigma_(6)*pow(g_x(6, 4), 2) + Sigma_(7)*pow(g_x(7, 4), 2);
    V_xx(5, 5) = Sigma_(10)*pow(g_x(10, 5), 2) + Sigma_(11)*pow(g_x(11, 5), 2) + l_xx(5, 5);
    V_xx(6, 6) = Sigma_(12)*pow(g_x(12, 6), 2) + Sigma_(13)*pow(g_x(13, 6), 2);

    // Evaluation of Vector V_x
    V_x(0) = V_(8)*g_x(8, 0) + V_(9)*g_x(9, 0) + sl_x(0);
    V_x(1) = V_(0)*g_x(0, 1) + V_(1)*g_x(1, 1) + V_(2)*g_x(2, 1) + V_(3)*g_x(3, 1) + V_(4)*g_x(4, 1) + V_(5)*g_x(5, 1) + sl_x(1);
    V_x(2) = V_(0)*g_x(0, 2) + V_(1)*g_x(1, 2) + V_(2)*g_x(2, 2) + V_(3)*g_x(3, 2) + V_(4)*g_x(4, 2) + V_(5)*g_x(5, 2) + sl_x(2);
    V_x(3) = V_(14)*g_x(14, 3) + V_(2)*g_x(2, 3) + V_(3)*g_x(3, 3) + V_(4)*g_x(4, 3) + V_(5)*g_x(5, 3) + sl_x(3);
    V_x(4) = V_(6)*g_x(6, 4) + V_(7)*g_x(7, 4) + sl_x(4);
    V_x(5) = V_(10)*g_x(10, 5) + V_(11)*g_x(11, 5) + sl_x(5);
    V_x(6) = V_(12)*g_x(12, 6) + V_(13)*g_x(13, 6) + sl_x(6);

  }

};

}  // namespace gpal::pnc::planning