#pragma once

#include "ocp/ipm_evaluator.h"

namespace gpal::pnc::planning {

template <StageType stage_type, OcpConfig::IntegratorType integrator_type>
class MultiAgentsTrajectoryModelIpmEvaluator : public IpmEvaluator {
 public:
  MultiAgentsTrajectoryModelIpmEvaluator() : IpmEvaluator(10, 9, 28, 0, 0) {}
  virtual ~MultiAgentsTrajectoryModelIpmEvaluator() = default;

  virtual void updateLX(const Eigen::VectorXd& l_x, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& h_x, const Eigen::MatrixXd& m_x, const Eigen::VectorXd& p, const Eigen::VectorXd& lambda, const Eigen::VectorXd& eta, const Eigen::VectorXd& nu, Eigen::VectorXd& sl_x) const override {
    // Evaluation of Vector sl_x
    sl_x(0) = Fx(0, 0)*p(0) + g_x(2, 0)*lambda(2) + g_x(3, 0)*lambda(3) + l_x(0);
    sl_x(1) = Fx(0, 1)*p(0) + Fx(1, 1)*p(1) + g_x(0, 1)*lambda(0) + g_x(1, 1)*lambda(1) + g_x(22, 1)*lambda(22) + g_x(23, 1)*lambda(23) + g_x(24, 1)*lambda(24) + g_x(25, 1)*lambda(25) + g_x(26, 1)*lambda(26) + g_x(27, 1)*lambda(27) + g_x(8, 1)*lambda(8) + l_x(1);
    sl_x(2) = Fx(0, 2)*p(0) + Fx(2, 2)*p(2) + g_x(0, 2)*lambda(0) + g_x(1, 2)*lambda(1) + g_x(22, 2)*lambda(22) + g_x(23, 2)*lambda(23) + g_x(24, 2)*lambda(24) + g_x(25, 2)*lambda(25) + g_x(26, 2)*lambda(26) + g_x(27, 2)*lambda(27) + g_x(8, 2)*lambda(8) + l_x(2);
    sl_x(3) = Fx(0, 3)*p(0) + Fx(1, 3)*p(1) + Fx(2, 3)*p(2) + Fx(3, 3)*p(3) + g_x(22, 3)*lambda(22) + g_x(23, 3)*lambda(23) + g_x(24, 3)*lambda(24) + g_x(25, 3)*lambda(25) + g_x(26, 3)*lambda(26) + g_x(27, 3)*lambda(27) + g_x(9, 3)*lambda(9) + l_x(3);
    sl_x(4) = Fx(0, 4)*p(0) + Fx(1, 4)*p(1) + Fx(2, 4)*p(2) + Fx(3, 4)*p(3) + Fx(4, 4)*p(4) + g_x(5, 4)*lambda(5) + g_x(6, 4)*lambda(6) + l_x(4);
    sl_x(5) = Fx(5, 5)*p(5) + g_x(4, 5)*lambda(4) + l_x(5);
    sl_x(6) = Fx(6, 6)*p(6) + g_x(22, 6)*lambda(22) + g_x(23, 6)*lambda(23);
    sl_x(7) = Fx(7, 7)*p(7) + g_x(22, 7)*lambda(22) + g_x(23, 7)*lambda(23);
    sl_x(8) = Fx(8, 8)*p(8);
    sl_x(9) = Fx(5, 9)*p(5) + Fx(6, 9)*p(6) + Fx(7, 9)*p(7) + Fx(9, 9)*p(9) + g_x(7, 9)*lambda(7) + l_x(9);

  }

  virtual void updateLU(const Eigen::VectorXd& l_u, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& g_u, const Eigen::MatrixXd& h_u, const Eigen::VectorXd& p, const Eigen::VectorXd& lambda, const Eigen::VectorXd& eta, Eigen::VectorXd& sl_u) const override {
    // Evaluation of Vector sl_u
    sl_u(0) = Fu(4, 0)*p(4) + g_u(18, 0)*lambda(18) + g_u(19, 0)*lambda(19) + l_u(0);
    sl_u(1) = Fu(3, 1)*p(3) + g_u(16, 1)*lambda(16) + g_u(17, 1)*lambda(17) + l_u(1);
    sl_u(2) = Fu(9, 2)*p(9) + g_u(20, 2)*lambda(20) + g_u(21, 2)*lambda(21) + l_u(2);
    sl_u(3) = g_u(10, 3)*lambda(10) + g_u(22, 3)*lambda(22) + l_u(3);
    sl_u(4) = g_u(11, 4)*lambda(11) + g_u(23, 4)*lambda(23) + l_u(4);
    sl_u(5) = g_u(12, 5)*lambda(12) + g_u(24, 5)*lambda(24) + l_u(5);
    sl_u(6) = g_u(13, 6)*lambda(13) + g_u(26, 6)*lambda(26) + l_u(6);
    sl_u(7) = g_u(14, 7)*lambda(14) + g_u(25, 7)*lambda(25) + l_u(7);
    sl_u(8) = g_u(15, 8)*lambda(15) + g_u(27, 8)*lambda(27) + l_u(8);

  }

  virtual void updateQH(const Eigen::MatrixXd& l_xx, const Eigen::MatrixXd& l_xu, const Eigen::MatrixXd& l_uu, const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& V_xx_prev) override {
    // Determine internal variables
    const double internal_0 = Fx(0, 1)*V_xx_prev(0, 0);
    const double internal_1 = Fx(0, 0)*V_xx_prev(0, 1);
    const double internal_10 = Fx(0, 0)*V_xx_prev(0, 7);
    const double internal_100 = Fx(7, 7)*V_xx_prev(7, 5);
    const double internal_101 = Fx(7, 7)*V_xx_prev(7, 6);
    const double internal_102 = Fx(7, 7)*Fx(8, 8);
    const double internal_103 = Fx(7, 9)*V_xx_prev(7, 7);
    const double internal_104 = Fx(7, 7)*V_xx_prev(7, 9);
    const double internal_105 = Fx(8, 8)*V_xx_prev(8, 0);
    const double internal_106 = Fx(8, 8)*V_xx_prev(8, 1);
    const double internal_107 = Fx(8, 8)*V_xx_prev(8, 2);
    const double internal_108 = Fx(8, 8)*V_xx_prev(8, 3);
    const double internal_109 = Fx(8, 8)*V_xx_prev(8, 4);
    const double internal_11 = Fx(0, 0)*Fx(8, 8);
    const double internal_110 = Fx(8, 8)*V_xx_prev(8, 9);
    const double internal_12 = Fx(0, 0)*V_xx_prev(0, 9);
    const double internal_120 = Fu(3, 1)*Fu(4, 0);
    const double internal_121 = Fu(4, 0)*Fu(9, 2);
    const double internal_122 = Fu(3, 1)*Fu(9, 2);
    const double internal_16 = Sigma_(22)*g_x(22, 1);
    const double internal_17 = Sigma_(23)*g_x(23, 1);
    const double internal_18 = Sigma_(24)*g_x(24, 1);
    const double internal_19 = Sigma_(25)*g_x(25, 1);
    const double internal_2 = Fx(0, 2)*V_xx_prev(0, 0);
    const double internal_20 = Sigma_(26)*g_x(26, 1);
    const double internal_21 = Sigma_(27)*g_x(27, 1);
    const double internal_3 = Fx(0, 0)*V_xx_prev(0, 2);
    const double internal_36 = Sigma_(22)*g_x(22, 2);
    const double internal_37 = Sigma_(23)*g_x(23, 2);
    const double internal_38 = Sigma_(24)*g_x(24, 2);
    const double internal_39 = Sigma_(25)*g_x(25, 2);
    const double internal_4 = Fx(0, 3)*V_xx_prev(0, 0);
    const double internal_40 = Sigma_(26)*g_x(26, 2);
    const double internal_41 = Sigma_(27)*g_x(27, 2);
    const double internal_5 = Fx(0, 0)*V_xx_prev(0, 3);
    const double internal_57 = Sigma_(22)*g_x(22, 3);
    const double internal_58 = Sigma_(23)*g_x(23, 3);
    const double internal_6 = Fx(0, 4)*V_xx_prev(0, 0);
    const double internal_7 = Fx(0, 0)*V_xx_prev(0, 4);
    const double internal_72 = Fx(5, 5)*V_xx_prev(5, 0);
    const double internal_73 = Fx(5, 5)*V_xx_prev(5, 1);
    const double internal_74 = Fx(5, 5)*V_xx_prev(5, 2);
    const double internal_75 = Fx(5, 5)*V_xx_prev(5, 3);
    const double internal_76 = Fx(5, 5)*V_xx_prev(5, 4);
    const double internal_77 = Fx(5, 5)*V_xx_prev(5, 6);
    const double internal_78 = Fx(5, 5)*V_xx_prev(5, 7);
    const double internal_79 = Fx(5, 5)*Fx(8, 8);
    const double internal_8 = Fx(0, 0)*V_xx_prev(0, 5);
    const double internal_80 = Fx(5, 9)*V_xx_prev(5, 5);
    const double internal_81 = Fx(5, 5)*V_xx_prev(5, 9);
    const double internal_82 = Fx(6, 6)*V_xx_prev(6, 0);
    const double internal_83 = Fx(6, 6)*V_xx_prev(6, 1);
    const double internal_84 = Fx(6, 6)*V_xx_prev(6, 2);
    const double internal_85 = Fx(6, 6)*V_xx_prev(6, 3);
    const double internal_86 = Fx(6, 6)*V_xx_prev(6, 4);
    const double internal_87 = Fx(6, 6)*V_xx_prev(6, 5);
    const double internal_88 = Fx(6, 6)*V_xx_prev(6, 7);
    const double internal_89 = Sigma_(22)*g_x(22, 6);
    const double internal_9 = Fx(0, 0)*V_xx_prev(0, 6);
    const double internal_90 = Sigma_(23)*g_x(23, 6);
    const double internal_92 = Fx(6, 6)*Fx(8, 8);
    const double internal_93 = Fx(6, 9)*V_xx_prev(6, 6);
    const double internal_94 = Fx(6, 6)*V_xx_prev(6, 9);
    const double internal_95 = Fx(7, 7)*V_xx_prev(7, 0);
    const double internal_96 = Fx(7, 7)*V_xx_prev(7, 1);
    const double internal_97 = Fx(7, 7)*V_xx_prev(7, 2);
    const double internal_98 = Fx(7, 7)*V_xx_prev(7, 3);
    const double internal_99 = Fx(7, 7)*V_xx_prev(7, 4);
    const double internal_111 = Fx(5, 9)*V_xx_prev(5, 0) + Fx(6, 9)*V_xx_prev(6, 0) + Fx(7, 9)*V_xx_prev(7, 0) + Fx(9, 9)*V_xx_prev(9, 0);
    const double internal_112 = Fx(5, 9)*V_xx_prev(5, 1) + Fx(6, 9)*V_xx_prev(6, 1) + Fx(7, 9)*V_xx_prev(7, 1) + Fx(9, 9)*V_xx_prev(9, 1);
    const double internal_113 = Fx(5, 9)*V_xx_prev(5, 2) + Fx(6, 9)*V_xx_prev(6, 2) + Fx(7, 9)*V_xx_prev(7, 2) + Fx(9, 9)*V_xx_prev(9, 2);
    const double internal_114 = Fx(5, 9)*V_xx_prev(5, 3) + Fx(6, 9)*V_xx_prev(6, 3) + Fx(7, 9)*V_xx_prev(7, 3) + Fx(9, 9)*V_xx_prev(9, 3);
    const double internal_115 = Fx(5, 9)*V_xx_prev(5, 4) + Fx(6, 9)*V_xx_prev(6, 4) + Fx(7, 9)*V_xx_prev(7, 4) + Fx(9, 9)*V_xx_prev(9, 4);
    const double internal_119 = Fx(5, 9)*V_xx_prev(5, 9) + Fx(6, 9)*V_xx_prev(6, 9) + Fx(7, 9)*V_xx_prev(7, 9) + Fx(9, 9)*V_xx_prev(9, 9);
    const double internal_14 = Fx(0, 1)*V_xx_prev(0, 1) + Fx(1, 1)*V_xx_prev(1, 1);
    const double internal_15 = Fx(0, 1)*V_xx_prev(0, 2) + Fx(1, 1)*V_xx_prev(1, 2);
    const double internal_23 = Fx(0, 1)*V_xx_prev(0, 3) + Fx(1, 1)*V_xx_prev(1, 3);
    const double internal_25 = Fx(0, 1)*V_xx_prev(0, 4) + Fx(1, 1)*V_xx_prev(1, 4);
    const double internal_26 = Fx(0, 1)*V_xx_prev(0, 5) + Fx(1, 1)*V_xx_prev(1, 5);
    const double internal_27 = Fx(0, 1)*V_xx_prev(0, 6) + Fx(1, 1)*V_xx_prev(1, 6);
    const double internal_29 = Fx(0, 1)*V_xx_prev(0, 7) + Fx(1, 1)*V_xx_prev(1, 7);
    const double internal_31 = Fx(0, 1)*V_xx_prev(0, 9) + Fx(1, 1)*V_xx_prev(1, 9);
    const double internal_33 = Fx(0, 2)*V_xx_prev(0, 1) + Fx(2, 2)*V_xx_prev(2, 1);
    const double internal_34 = Fx(0, 2)*V_xx_prev(0, 2) + Fx(2, 2)*V_xx_prev(2, 2);
    const double internal_35 = Fx(0, 2)*V_xx_prev(0, 3) + Fx(2, 2)*V_xx_prev(2, 3);
    const double internal_43 = Fx(0, 2)*V_xx_prev(0, 4) + Fx(2, 2)*V_xx_prev(2, 4);
    const double internal_44 = Fx(0, 2)*V_xx_prev(0, 5) + Fx(2, 2)*V_xx_prev(2, 5);
    const double internal_45 = Fx(0, 2)*V_xx_prev(0, 6) + Fx(2, 2)*V_xx_prev(2, 6);
    const double internal_47 = Fx(0, 2)*V_xx_prev(0, 7) + Fx(2, 2)*V_xx_prev(2, 7);
    const double internal_49 = Fx(0, 2)*V_xx_prev(0, 9) + Fx(2, 2)*V_xx_prev(2, 9);
    const double internal_51 = Fx(0, 3)*V_xx_prev(0, 1) + Fx(1, 3)*V_xx_prev(1, 1) + Fx(2, 3)*V_xx_prev(2, 1) + Fx(3, 3)*V_xx_prev(3, 1);
    const double internal_52 = Fx(0, 3)*V_xx_prev(0, 2) + Fx(1, 3)*V_xx_prev(1, 2) + Fx(2, 3)*V_xx_prev(2, 2) + Fx(3, 3)*V_xx_prev(3, 2);
    const double internal_53 = Fx(0, 3)*V_xx_prev(0, 3) + Fx(1, 3)*V_xx_prev(1, 3) + Fx(2, 3)*V_xx_prev(2, 3) + Fx(3, 3)*V_xx_prev(3, 3);
    const double internal_54 = Fx(0, 3)*V_xx_prev(0, 4) + Fx(1, 3)*V_xx_prev(1, 4) + Fx(2, 3)*V_xx_prev(2, 4) + Fx(3, 3)*V_xx_prev(3, 4);
    const double internal_55 = Fx(0, 3)*V_xx_prev(0, 5) + Fx(1, 3)*V_xx_prev(1, 5) + Fx(2, 3)*V_xx_prev(2, 5) + Fx(3, 3)*V_xx_prev(3, 5);
    const double internal_56 = Fx(0, 3)*V_xx_prev(0, 6) + Fx(1, 3)*V_xx_prev(1, 6) + Fx(2, 3)*V_xx_prev(2, 6) + Fx(3, 3)*V_xx_prev(3, 6);
    const double internal_60 = Fx(0, 3)*V_xx_prev(0, 7) + Fx(1, 3)*V_xx_prev(1, 7) + Fx(2, 3)*V_xx_prev(2, 7) + Fx(3, 3)*V_xx_prev(3, 7);
    const double internal_62 = Fx(0, 3)*V_xx_prev(0, 9) + Fx(1, 3)*V_xx_prev(1, 9) + Fx(2, 3)*V_xx_prev(2, 9) + Fx(3, 3)*V_xx_prev(3, 9);
    const double internal_64 = Fx(0, 4)*V_xx_prev(0, 1) + Fx(1, 4)*V_xx_prev(1, 1) + Fx(2, 4)*V_xx_prev(2, 1) + Fx(3, 4)*V_xx_prev(3, 1) + Fx(4, 4)*V_xx_prev(4, 1);
    const double internal_65 = Fx(0, 4)*V_xx_prev(0, 2) + Fx(1, 4)*V_xx_prev(1, 2) + Fx(2, 4)*V_xx_prev(2, 2) + Fx(3, 4)*V_xx_prev(3, 2) + Fx(4, 4)*V_xx_prev(4, 2);
    const double internal_66 = Fx(0, 4)*V_xx_prev(0, 3) + Fx(1, 4)*V_xx_prev(1, 3) + Fx(2, 4)*V_xx_prev(2, 3) + Fx(3, 4)*V_xx_prev(3, 3) + Fx(4, 4)*V_xx_prev(4, 3);
    const double internal_67 = Fx(0, 4)*V_xx_prev(0, 4) + Fx(1, 4)*V_xx_prev(1, 4) + Fx(2, 4)*V_xx_prev(2, 4) + Fx(3, 4)*V_xx_prev(3, 4) + Fx(4, 4)*V_xx_prev(4, 4);
    const double internal_68 = Fx(0, 4)*V_xx_prev(0, 5) + Fx(1, 4)*V_xx_prev(1, 5) + Fx(2, 4)*V_xx_prev(2, 5) + Fx(3, 4)*V_xx_prev(3, 5) + Fx(4, 4)*V_xx_prev(4, 5);
    const double internal_69 = Fx(0, 4)*V_xx_prev(0, 6) + Fx(1, 4)*V_xx_prev(1, 6) + Fx(2, 4)*V_xx_prev(2, 6) + Fx(3, 4)*V_xx_prev(3, 6) + Fx(4, 4)*V_xx_prev(4, 6);
    const double internal_70 = Fx(0, 4)*V_xx_prev(0, 7) + Fx(1, 4)*V_xx_prev(1, 7) + Fx(2, 4)*V_xx_prev(2, 7) + Fx(3, 4)*V_xx_prev(3, 7) + Fx(4, 4)*V_xx_prev(4, 7);
    const double internal_71 = Fx(0, 4)*V_xx_prev(0, 9) + Fx(1, 4)*V_xx_prev(1, 9) + Fx(2, 4)*V_xx_prev(2, 9) + Fx(3, 4)*V_xx_prev(3, 9) + Fx(4, 4)*V_xx_prev(4, 9);
    const double internal_116 = Fx(6, 9)*V_xx_prev(6, 5) + Fx(7, 9)*V_xx_prev(7, 5) + Fx(9, 9)*V_xx_prev(9, 5) + internal_80;
    const double internal_117 = Fx(5, 9)*V_xx_prev(5, 6) + Fx(7, 9)*V_xx_prev(7, 6) + Fx(9, 9)*V_xx_prev(9, 6) + internal_93;
    const double internal_118 = Fx(5, 9)*V_xx_prev(5, 7) + Fx(6, 9)*V_xx_prev(6, 7) + Fx(9, 9)*V_xx_prev(9, 7) + internal_103;
    const double internal_13 = Fx(1, 1)*V_xx_prev(1, 0) + internal_0;
    const double internal_32 = Fx(2, 2)*V_xx_prev(2, 0) + internal_2;
    const double internal_50 = Fx(1, 3)*V_xx_prev(1, 0) + Fx(2, 3)*V_xx_prev(2, 0) + Fx(3, 3)*V_xx_prev(3, 0) + internal_4;
    const double internal_63 = Fx(1, 4)*V_xx_prev(1, 0) + Fx(2, 4)*V_xx_prev(2, 0) + Fx(3, 4)*V_xx_prev(3, 0) + Fx(4, 4)*V_xx_prev(4, 0) + internal_6;
    const double internal_22 = Sigma_(0)*g_x(0, 1)*g_x(0, 2) + Sigma_(1)*g_x(1, 1)*g_x(1, 2) + Sigma_(8)*g_x(8, 1)*g_x(8, 2) + g_x(22, 2)*internal_16 + g_x(23, 2)*internal_17 + g_x(24, 2)*internal_18 + g_x(25, 2)*internal_19 + g_x(26, 2)*internal_20 + g_x(27, 2)*internal_21;
    const double internal_24 = g_x(22, 3)*internal_16 + g_x(23, 3)*internal_17 + g_x(24, 3)*internal_18 + g_x(25, 3)*internal_19 + g_x(26, 3)*internal_20 + g_x(27, 3)*internal_21;
    const double internal_28 = g_x(22, 6)*internal_16 + g_x(23, 6)*internal_17;
    const double internal_30 = g_x(22, 7)*internal_16 + g_x(23, 7)*internal_17;
    const double internal_42 = g_x(22, 3)*internal_36 + g_x(23, 3)*internal_37 + g_x(24, 3)*internal_38 + g_x(25, 3)*internal_39 + g_x(26, 3)*internal_40 + g_x(27, 3)*internal_41;
    const double internal_46 = g_x(22, 6)*internal_36 + g_x(23, 6)*internal_37;
    const double internal_48 = g_x(22, 7)*internal_36 + g_x(23, 7)*internal_37;
    const double internal_59 = g_x(22, 6)*internal_57 + g_x(23, 6)*internal_58;
    const double internal_61 = g_x(22, 7)*internal_57 + g_x(23, 7)*internal_58;
    const double internal_91 = g_x(22, 7)*internal_89 + g_x(23, 7)*internal_90;

    // Evaluation of Matrix Q_xx_
    Q_xx_(0, 0) = pow(Fx(0, 0), 2)*V_xx_prev(0, 0) + Sigma_(2)*pow(g_x(2, 0), 2) + Sigma_(3)*pow(g_x(3, 0), 2) + l_xx(0, 0);
    Q_xx_(0, 1) = Fx(0, 0)*internal_0 + Fx(1, 1)*internal_1;
    Q_xx_(0, 2) = Fx(0, 0)*internal_2 + Fx(2, 2)*internal_3;
    Q_xx_(0, 3) = Fx(0, 0)*internal_4 + Fx(1, 3)*internal_1 + Fx(2, 3)*internal_3 + Fx(3, 3)*internal_5;
    Q_xx_(0, 4) = Fx(0, 0)*internal_6 + Fx(1, 4)*internal_1 + Fx(2, 4)*internal_3 + Fx(3, 4)*internal_5 + Fx(4, 4)*internal_7;
    Q_xx_(0, 5) = Fx(5, 5)*internal_8;
    Q_xx_(0, 6) = Fx(6, 6)*internal_9;
    Q_xx_(0, 7) = Fx(7, 7)*internal_10;
    Q_xx_(0, 8) = V_xx_prev(0, 8)*internal_11;
    Q_xx_(0, 9) = Fx(5, 9)*internal_8 + Fx(6, 9)*internal_9 + Fx(7, 9)*internal_10 + Fx(9, 9)*internal_12;
    Q_xx_(1, 0) = Fx(0, 0)*internal_13;
    Q_xx_(1, 1) = Fx(0, 1)*internal_13 + Fx(1, 1)*internal_14 + Sigma_(0)*pow(g_x(0, 1), 2) + Sigma_(1)*pow(g_x(1, 1), 2) + Sigma_(22)*pow(g_x(22, 1), 2) + Sigma_(23)*pow(g_x(23, 1), 2) + Sigma_(24)*pow(g_x(24, 1), 2) + Sigma_(25)*pow(g_x(25, 1), 2) + Sigma_(26)*pow(g_x(26, 1), 2) + Sigma_(27)*pow(g_x(27, 1), 2) + Sigma_(8)*pow(g_x(8, 1), 2) + l_xx(1, 1);
    Q_xx_(1, 2) = Fx(0, 2)*internal_13 + Fx(2, 2)*internal_15 + internal_22 + l_xx(1, 2);
    Q_xx_(1, 3) = Fx(0, 3)*internal_13 + Fx(1, 3)*internal_14 + Fx(2, 3)*internal_15 + Fx(3, 3)*internal_23 + internal_24;
    Q_xx_(1, 4) = Fx(0, 4)*internal_13 + Fx(1, 4)*internal_14 + Fx(2, 4)*internal_15 + Fx(3, 4)*internal_23 + Fx(4, 4)*internal_25;
    Q_xx_(1, 5) = Fx(5, 5)*internal_26;
    Q_xx_(1, 6) = Fx(6, 6)*internal_27 + internal_28;
    Q_xx_(1, 7) = Fx(7, 7)*internal_29 + internal_30;
    Q_xx_(1, 8) = Fx(8, 8)*(Fx(0, 1)*V_xx_prev(0, 8) + Fx(1, 1)*V_xx_prev(1, 8));
    Q_xx_(1, 9) = Fx(5, 9)*internal_26 + Fx(6, 9)*internal_27 + Fx(7, 9)*internal_29 + Fx(9, 9)*internal_31;
    Q_xx_(2, 0) = Fx(0, 0)*internal_32;
    Q_xx_(2, 1) = Fx(0, 1)*internal_32 + Fx(1, 1)*internal_33 + internal_22 + l_xx(2, 1);
    Q_xx_(2, 2) = Fx(0, 2)*internal_32 + Fx(2, 2)*internal_34 + Sigma_(0)*pow(g_x(0, 2), 2) + Sigma_(1)*pow(g_x(1, 2), 2) + Sigma_(22)*pow(g_x(22, 2), 2) + Sigma_(23)*pow(g_x(23, 2), 2) + Sigma_(24)*pow(g_x(24, 2), 2) + Sigma_(25)*pow(g_x(25, 2), 2) + Sigma_(26)*pow(g_x(26, 2), 2) + Sigma_(27)*pow(g_x(27, 2), 2) + Sigma_(8)*pow(g_x(8, 2), 2) + l_xx(2, 2);
    Q_xx_(2, 3) = Fx(0, 3)*internal_32 + Fx(1, 3)*internal_33 + Fx(2, 3)*internal_34 + Fx(3, 3)*internal_35 + internal_42;
    Q_xx_(2, 4) = Fx(0, 4)*internal_32 + Fx(1, 4)*internal_33 + Fx(2, 4)*internal_34 + Fx(3, 4)*internal_35 + Fx(4, 4)*internal_43;
    Q_xx_(2, 5) = Fx(5, 5)*internal_44;
    Q_xx_(2, 6) = Fx(6, 6)*internal_45 + internal_46;
    Q_xx_(2, 7) = Fx(7, 7)*internal_47 + internal_48;
    Q_xx_(2, 8) = Fx(8, 8)*(Fx(0, 2)*V_xx_prev(0, 8) + Fx(2, 2)*V_xx_prev(2, 8));
    Q_xx_(2, 9) = Fx(5, 9)*internal_44 + Fx(6, 9)*internal_45 + Fx(7, 9)*internal_47 + Fx(9, 9)*internal_49;
    Q_xx_(3, 0) = Fx(0, 0)*internal_50;
    Q_xx_(3, 1) = Fx(0, 1)*internal_50 + Fx(1, 1)*internal_51 + internal_24;
    Q_xx_(3, 2) = Fx(0, 2)*internal_50 + Fx(2, 2)*internal_52 + internal_42;
    Q_xx_(3, 3) = Fx(0, 3)*internal_50 + Fx(1, 3)*internal_51 + Fx(2, 3)*internal_52 + Fx(3, 3)*internal_53 + Sigma_(22)*pow(g_x(22, 3), 2) + Sigma_(23)*pow(g_x(23, 3), 2) + Sigma_(24)*pow(g_x(24, 3), 2) + Sigma_(25)*pow(g_x(25, 3), 2) + Sigma_(26)*pow(g_x(26, 3), 2) + Sigma_(27)*pow(g_x(27, 3), 2) + Sigma_(9)*pow(g_x(9, 3), 2) + l_xx(3, 3);
    Q_xx_(3, 4) = Fx(0, 4)*internal_50 + Fx(1, 4)*internal_51 + Fx(2, 4)*internal_52 + Fx(3, 4)*internal_53 + Fx(4, 4)*internal_54;
    Q_xx_(3, 5) = Fx(5, 5)*internal_55;
    Q_xx_(3, 6) = Fx(6, 6)*internal_56 + internal_59;
    Q_xx_(3, 7) = Fx(7, 7)*internal_60 + internal_61;
    Q_xx_(3, 8) = Fx(8, 8)*(Fx(0, 3)*V_xx_prev(0, 8) + Fx(1, 3)*V_xx_prev(1, 8) + Fx(2, 3)*V_xx_prev(2, 8) + Fx(3, 3)*V_xx_prev(3, 8));
    Q_xx_(3, 9) = Fx(5, 9)*internal_55 + Fx(6, 9)*internal_56 + Fx(7, 9)*internal_60 + Fx(9, 9)*internal_62;
    Q_xx_(4, 0) = Fx(0, 0)*internal_63;
    Q_xx_(4, 1) = Fx(0, 1)*internal_63 + Fx(1, 1)*internal_64;
    Q_xx_(4, 2) = Fx(0, 2)*internal_63 + Fx(2, 2)*internal_65;
    Q_xx_(4, 3) = Fx(0, 3)*internal_63 + Fx(1, 3)*internal_64 + Fx(2, 3)*internal_65 + Fx(3, 3)*internal_66;
    Q_xx_(4, 4) = Fx(0, 4)*internal_63 + Fx(1, 4)*internal_64 + Fx(2, 4)*internal_65 + Fx(3, 4)*internal_66 + Fx(4, 4)*internal_67 + Sigma_(5)*pow(g_x(5, 4), 2) + Sigma_(6)*pow(g_x(6, 4), 2) + l_xx(4, 4);
    Q_xx_(4, 5) = Fx(5, 5)*internal_68;
    Q_xx_(4, 6) = Fx(6, 6)*internal_69;
    Q_xx_(4, 7) = Fx(7, 7)*internal_70;
    Q_xx_(4, 8) = Fx(8, 8)*(Fx(0, 4)*V_xx_prev(0, 8) + Fx(1, 4)*V_xx_prev(1, 8) + Fx(2, 4)*V_xx_prev(2, 8) + Fx(3, 4)*V_xx_prev(3, 8) + Fx(4, 4)*V_xx_prev(4, 8));
    Q_xx_(4, 9) = Fx(5, 9)*internal_68 + Fx(6, 9)*internal_69 + Fx(7, 9)*internal_70 + Fx(9, 9)*internal_71;
    Q_xx_(5, 0) = Fx(0, 0)*internal_72;
    Q_xx_(5, 1) = Fx(0, 1)*internal_72 + Fx(1, 1)*internal_73;
    Q_xx_(5, 2) = Fx(0, 2)*internal_72 + Fx(2, 2)*internal_74;
    Q_xx_(5, 3) = Fx(0, 3)*internal_72 + Fx(1, 3)*internal_73 + Fx(2, 3)*internal_74 + Fx(3, 3)*internal_75;
    Q_xx_(5, 4) = Fx(0, 4)*internal_72 + Fx(1, 4)*internal_73 + Fx(2, 4)*internal_74 + Fx(3, 4)*internal_75 + Fx(4, 4)*internal_76;
    Q_xx_(5, 5) = pow(Fx(5, 5), 2)*V_xx_prev(5, 5) + Sigma_(4)*pow(g_x(4, 5), 2) + l_xx(5, 5);
    Q_xx_(5, 6) = Fx(6, 6)*internal_77;
    Q_xx_(5, 7) = Fx(7, 7)*internal_78;
    Q_xx_(5, 8) = V_xx_prev(5, 8)*internal_79;
    Q_xx_(5, 9) = Fx(5, 5)*internal_80 + Fx(6, 9)*internal_77 + Fx(7, 9)*internal_78 + Fx(9, 9)*internal_81;
    Q_xx_(6, 0) = Fx(0, 0)*internal_82;
    Q_xx_(6, 1) = Fx(0, 1)*internal_82 + Fx(1, 1)*internal_83 + internal_28;
    Q_xx_(6, 2) = Fx(0, 2)*internal_82 + Fx(2, 2)*internal_84 + internal_46;
    Q_xx_(6, 3) = Fx(0, 3)*internal_82 + Fx(1, 3)*internal_83 + Fx(2, 3)*internal_84 + Fx(3, 3)*internal_85 + internal_59;
    Q_xx_(6, 4) = Fx(0, 4)*internal_82 + Fx(1, 4)*internal_83 + Fx(2, 4)*internal_84 + Fx(3, 4)*internal_85 + Fx(4, 4)*internal_86;
    Q_xx_(6, 5) = Fx(5, 5)*internal_87;
    Q_xx_(6, 6) = pow(Fx(6, 6), 2)*V_xx_prev(6, 6) + Sigma_(22)*pow(g_x(22, 6), 2) + Sigma_(23)*pow(g_x(23, 6), 2);
    Q_xx_(6, 7) = Fx(7, 7)*internal_88 + internal_91;
    Q_xx_(6, 8) = V_xx_prev(6, 8)*internal_92;
    Q_xx_(6, 9) = Fx(5, 9)*internal_87 + Fx(6, 6)*internal_93 + Fx(7, 9)*internal_88 + Fx(9, 9)*internal_94;
    Q_xx_(7, 0) = Fx(0, 0)*internal_95;
    Q_xx_(7, 1) = Fx(0, 1)*internal_95 + Fx(1, 1)*internal_96 + internal_30;
    Q_xx_(7, 2) = Fx(0, 2)*internal_95 + Fx(2, 2)*internal_97 + internal_48;
    Q_xx_(7, 3) = Fx(0, 3)*internal_95 + Fx(1, 3)*internal_96 + Fx(2, 3)*internal_97 + Fx(3, 3)*internal_98 + internal_61;
    Q_xx_(7, 4) = Fx(0, 4)*internal_95 + Fx(1, 4)*internal_96 + Fx(2, 4)*internal_97 + Fx(3, 4)*internal_98 + Fx(4, 4)*internal_99;
    Q_xx_(7, 5) = Fx(5, 5)*internal_100;
    Q_xx_(7, 6) = Fx(6, 6)*internal_101 + internal_91;
    Q_xx_(7, 7) = pow(Fx(7, 7), 2)*V_xx_prev(7, 7) + Sigma_(22)*pow(g_x(22, 7), 2) + Sigma_(23)*pow(g_x(23, 7), 2);
    Q_xx_(7, 8) = V_xx_prev(7, 8)*internal_102;
    Q_xx_(7, 9) = Fx(5, 9)*internal_100 + Fx(6, 9)*internal_101 + Fx(7, 7)*internal_103 + Fx(9, 9)*internal_104;
    Q_xx_(8, 0) = V_xx_prev(8, 0)*internal_11;
    Q_xx_(8, 1) = Fx(0, 1)*internal_105 + Fx(1, 1)*internal_106;
    Q_xx_(8, 2) = Fx(0, 2)*internal_105 + Fx(2, 2)*internal_107;
    Q_xx_(8, 3) = Fx(0, 3)*internal_105 + Fx(1, 3)*internal_106 + Fx(2, 3)*internal_107 + Fx(3, 3)*internal_108;
    Q_xx_(8, 4) = Fx(0, 4)*internal_105 + Fx(1, 4)*internal_106 + Fx(2, 4)*internal_107 + Fx(3, 4)*internal_108 + Fx(4, 4)*internal_109;
    Q_xx_(8, 5) = V_xx_prev(8, 5)*internal_79;
    Q_xx_(8, 6) = V_xx_prev(8, 6)*internal_92;
    Q_xx_(8, 7) = V_xx_prev(8, 7)*internal_102;
    Q_xx_(8, 8) = pow(Fx(8, 8), 2)*V_xx_prev(8, 8);
    Q_xx_(8, 9) = Fx(5, 9)*Fx(8, 8)*V_xx_prev(8, 5) + Fx(6, 9)*Fx(8, 8)*V_xx_prev(8, 6) + Fx(7, 9)*Fx(8, 8)*V_xx_prev(8, 7) + Fx(9, 9)*internal_110;
    Q_xx_(9, 0) = Fx(0, 0)*internal_111;
    Q_xx_(9, 1) = Fx(0, 1)*internal_111 + Fx(1, 1)*internal_112;
    Q_xx_(9, 2) = Fx(0, 2)*internal_111 + Fx(2, 2)*internal_113;
    Q_xx_(9, 3) = Fx(0, 3)*internal_111 + Fx(1, 3)*internal_112 + Fx(2, 3)*internal_113 + Fx(3, 3)*internal_114;
    Q_xx_(9, 4) = Fx(0, 4)*internal_111 + Fx(1, 4)*internal_112 + Fx(2, 4)*internal_113 + Fx(3, 4)*internal_114 + Fx(4, 4)*internal_115;
    Q_xx_(9, 5) = Fx(5, 5)*internal_116;
    Q_xx_(9, 6) = Fx(6, 6)*internal_117;
    Q_xx_(9, 7) = Fx(7, 7)*internal_118;
    Q_xx_(9, 8) = Fx(8, 8)*(Fx(5, 9)*V_xx_prev(5, 8) + Fx(6, 9)*V_xx_prev(6, 8) + Fx(7, 9)*V_xx_prev(7, 8) + Fx(9, 9)*V_xx_prev(9, 8));
    Q_xx_(9, 9) = Fx(5, 9)*internal_116 + Fx(6, 9)*internal_117 + Fx(7, 9)*internal_118 + Fx(9, 9)*internal_119 + Sigma_(7)*pow(g_x(7, 9), 2) + l_xx(9, 9);

    // Evaluation of Matrix Q_xu_
    Q_xu_(0, 0) = Fu(4, 0)*internal_7;
    Q_xu_(0, 1) = Fu(3, 1)*internal_5;
    Q_xu_(0, 2) = Fu(9, 2)*internal_12;
    Q_xu_(1, 0) = Fu(4, 0)*internal_25;
    Q_xu_(1, 1) = Fu(3, 1)*internal_23;
    Q_xu_(1, 2) = Fu(9, 2)*internal_31;
    Q_xu_(1, 3) = g_u(22, 3)*internal_16;
    Q_xu_(1, 4) = g_u(23, 4)*internal_17;
    Q_xu_(1, 5) = g_u(24, 5)*internal_18;
    Q_xu_(1, 6) = g_u(26, 6)*internal_20;
    Q_xu_(1, 7) = g_u(25, 7)*internal_19;
    Q_xu_(1, 8) = g_u(27, 8)*internal_21;
    Q_xu_(2, 0) = Fu(4, 0)*internal_43;
    Q_xu_(2, 1) = Fu(3, 1)*internal_35;
    Q_xu_(2, 2) = Fu(9, 2)*internal_49;
    Q_xu_(2, 3) = g_u(22, 3)*internal_36;
    Q_xu_(2, 4) = g_u(23, 4)*internal_37;
    Q_xu_(2, 5) = g_u(24, 5)*internal_38;
    Q_xu_(2, 6) = g_u(26, 6)*internal_40;
    Q_xu_(2, 7) = g_u(25, 7)*internal_39;
    Q_xu_(2, 8) = g_u(27, 8)*internal_41;
    Q_xu_(3, 0) = Fu(4, 0)*internal_54;
    Q_xu_(3, 1) = Fu(3, 1)*internal_53;
    Q_xu_(3, 2) = Fu(9, 2)*internal_62;
    Q_xu_(3, 3) = g_u(22, 3)*internal_57;
    Q_xu_(3, 4) = g_u(23, 4)*internal_58;
    Q_xu_(3, 5) = Sigma_(24)*g_u(24, 5)*g_x(24, 3);
    Q_xu_(3, 6) = Sigma_(26)*g_u(26, 6)*g_x(26, 3);
    Q_xu_(3, 7) = Sigma_(25)*g_u(25, 7)*g_x(25, 3);
    Q_xu_(3, 8) = Sigma_(27)*g_u(27, 8)*g_x(27, 3);
    Q_xu_(4, 0) = Fu(4, 0)*internal_67;
    Q_xu_(4, 1) = Fu(3, 1)*internal_66;
    Q_xu_(4, 2) = Fu(9, 2)*internal_71;
    Q_xu_(5, 0) = Fu(4, 0)*internal_76;
    Q_xu_(5, 1) = Fu(3, 1)*internal_75;
    Q_xu_(5, 2) = Fu(9, 2)*internal_81;
    Q_xu_(6, 0) = Fu(4, 0)*internal_86;
    Q_xu_(6, 1) = Fu(3, 1)*internal_85;
    Q_xu_(6, 2) = Fu(9, 2)*internal_94;
    Q_xu_(6, 3) = g_u(22, 3)*internal_89;
    Q_xu_(6, 4) = g_u(23, 4)*internal_90;
    Q_xu_(7, 0) = Fu(4, 0)*internal_99;
    Q_xu_(7, 1) = Fu(3, 1)*internal_98;
    Q_xu_(7, 2) = Fu(9, 2)*internal_104;
    Q_xu_(7, 3) = Sigma_(22)*g_u(22, 3)*g_x(22, 7);
    Q_xu_(7, 4) = Sigma_(23)*g_u(23, 4)*g_x(23, 7);
    Q_xu_(8, 0) = Fu(4, 0)*internal_109;
    Q_xu_(8, 1) = Fu(3, 1)*internal_108;
    Q_xu_(8, 2) = Fu(9, 2)*internal_110;
    Q_xu_(9, 0) = Fu(4, 0)*internal_115;
    Q_xu_(9, 1) = Fu(3, 1)*internal_114;
    Q_xu_(9, 2) = Fu(9, 2)*internal_119;

    // Evaluation of Matrix Q_uu_
    Q_uu_(0, 0) = pow(Fu(4, 0), 2)*V_xx_prev(4, 4) + Sigma_(18)*pow(g_u(18, 0), 2) + Sigma_(19)*pow(g_u(19, 0), 2) + l_uu(0, 0);
    Q_uu_(0, 1) = V_xx_prev(4, 3)*internal_120;
    Q_uu_(0, 2) = V_xx_prev(4, 9)*internal_121;
    Q_uu_(1, 0) = V_xx_prev(3, 4)*internal_120;
    Q_uu_(1, 1) = pow(Fu(3, 1), 2)*V_xx_prev(3, 3) + Sigma_(16)*pow(g_u(16, 1), 2) + Sigma_(17)*pow(g_u(17, 1), 2) + l_uu(1, 1);
    Q_uu_(1, 2) = V_xx_prev(3, 9)*internal_122;
    Q_uu_(2, 0) = V_xx_prev(9, 4)*internal_121;
    Q_uu_(2, 1) = V_xx_prev(9, 3)*internal_122;
    Q_uu_(2, 2) = pow(Fu(9, 2), 2)*V_xx_prev(9, 9) + Sigma_(20)*pow(g_u(20, 2), 2) + Sigma_(21)*pow(g_u(21, 2), 2) + l_uu(2, 2);
    Q_uu_(3, 3) = Sigma_(10)*pow(g_u(10, 3), 2) + Sigma_(22)*pow(g_u(22, 3), 2);
    Q_uu_(4, 4) = Sigma_(11)*pow(g_u(11, 4), 2) + Sigma_(23)*pow(g_u(23, 4), 2);
    Q_uu_(5, 5) = Sigma_(12)*pow(g_u(12, 5), 2) + Sigma_(24)*pow(g_u(24, 5), 2);
    Q_uu_(6, 6) = Sigma_(13)*pow(g_u(13, 6), 2) + Sigma_(26)*pow(g_u(26, 6), 2);
    Q_uu_(7, 7) = Sigma_(14)*pow(g_u(14, 7), 2) + Sigma_(25)*pow(g_u(25, 7), 2);
    Q_uu_(8, 8) = Sigma_(15)*pow(g_u(15, 8), 2) + Sigma_(27)*pow(g_u(27, 8), 2);

  }

  virtual void updateQG(const Eigen::VectorXd& sl_x, const Eigen::VectorXd& sl_u, const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::VectorXd& r_f, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& V_xx_prev, const Eigen::VectorXd& V_x_prev) override {
    // Determine internal variables
    const double internal_0 = V_x_prev(0) + V_xx_prev(0, 0)*r_f(0) + V_xx_prev(0, 1)*r_f(1) + V_xx_prev(0, 2)*r_f(2) + V_xx_prev(0, 3)*r_f(3) + V_xx_prev(0, 4)*r_f(4) + V_xx_prev(0, 5)*r_f(5) + V_xx_prev(0, 6)*r_f(6) + V_xx_prev(0, 7)*r_f(7) + V_xx_prev(0, 8)*r_f(8) + V_xx_prev(0, 9)*r_f(9);
    const double internal_1 = V_x_prev(1) + V_xx_prev(1, 0)*r_f(0) + V_xx_prev(1, 1)*r_f(1) + V_xx_prev(1, 2)*r_f(2) + V_xx_prev(1, 3)*r_f(3) + V_xx_prev(1, 4)*r_f(4) + V_xx_prev(1, 5)*r_f(5) + V_xx_prev(1, 6)*r_f(6) + V_xx_prev(1, 7)*r_f(7) + V_xx_prev(1, 8)*r_f(8) + V_xx_prev(1, 9)*r_f(9);
    const double internal_2 = V_x_prev(2) + V_xx_prev(2, 0)*r_f(0) + V_xx_prev(2, 1)*r_f(1) + V_xx_prev(2, 2)*r_f(2) + V_xx_prev(2, 3)*r_f(3) + V_xx_prev(2, 4)*r_f(4) + V_xx_prev(2, 5)*r_f(5) + V_xx_prev(2, 6)*r_f(6) + V_xx_prev(2, 7)*r_f(7) + V_xx_prev(2, 8)*r_f(8) + V_xx_prev(2, 9)*r_f(9);
    const double internal_3 = V_x_prev(3) + V_xx_prev(3, 0)*r_f(0) + V_xx_prev(3, 1)*r_f(1) + V_xx_prev(3, 2)*r_f(2) + V_xx_prev(3, 3)*r_f(3) + V_xx_prev(3, 4)*r_f(4) + V_xx_prev(3, 5)*r_f(5) + V_xx_prev(3, 6)*r_f(6) + V_xx_prev(3, 7)*r_f(7) + V_xx_prev(3, 8)*r_f(8) + V_xx_prev(3, 9)*r_f(9);
    const double internal_4 = V_x_prev(4) + V_xx_prev(4, 0)*r_f(0) + V_xx_prev(4, 1)*r_f(1) + V_xx_prev(4, 2)*r_f(2) + V_xx_prev(4, 3)*r_f(3) + V_xx_prev(4, 4)*r_f(4) + V_xx_prev(4, 5)*r_f(5) + V_xx_prev(4, 6)*r_f(6) + V_xx_prev(4, 7)*r_f(7) + V_xx_prev(4, 8)*r_f(8) + V_xx_prev(4, 9)*r_f(9);
    const double internal_5 = V_x_prev(5) + V_xx_prev(5, 0)*r_f(0) + V_xx_prev(5, 1)*r_f(1) + V_xx_prev(5, 2)*r_f(2) + V_xx_prev(5, 3)*r_f(3) + V_xx_prev(5, 4)*r_f(4) + V_xx_prev(5, 5)*r_f(5) + V_xx_prev(5, 6)*r_f(6) + V_xx_prev(5, 7)*r_f(7) + V_xx_prev(5, 8)*r_f(8) + V_xx_prev(5, 9)*r_f(9);
    const double internal_6 = V_x_prev(6) + V_xx_prev(6, 0)*r_f(0) + V_xx_prev(6, 1)*r_f(1) + V_xx_prev(6, 2)*r_f(2) + V_xx_prev(6, 3)*r_f(3) + V_xx_prev(6, 4)*r_f(4) + V_xx_prev(6, 5)*r_f(5) + V_xx_prev(6, 6)*r_f(6) + V_xx_prev(6, 7)*r_f(7) + V_xx_prev(6, 8)*r_f(8) + V_xx_prev(6, 9)*r_f(9);
    const double internal_7 = V_x_prev(7) + V_xx_prev(7, 0)*r_f(0) + V_xx_prev(7, 1)*r_f(1) + V_xx_prev(7, 2)*r_f(2) + V_xx_prev(7, 3)*r_f(3) + V_xx_prev(7, 4)*r_f(4) + V_xx_prev(7, 5)*r_f(5) + V_xx_prev(7, 6)*r_f(6) + V_xx_prev(7, 7)*r_f(7) + V_xx_prev(7, 8)*r_f(8) + V_xx_prev(7, 9)*r_f(9);
    const double internal_8 = V_x_prev(9) + V_xx_prev(9, 0)*r_f(0) + V_xx_prev(9, 1)*r_f(1) + V_xx_prev(9, 2)*r_f(2) + V_xx_prev(9, 3)*r_f(3) + V_xx_prev(9, 4)*r_f(4) + V_xx_prev(9, 5)*r_f(5) + V_xx_prev(9, 6)*r_f(6) + V_xx_prev(9, 7)*r_f(7) + V_xx_prev(9, 8)*r_f(8) + V_xx_prev(9, 9)*r_f(9);

    // Evaluation of Vector Q_x_
    Q_x_(0) = Fx(0, 0)*internal_0 + V_(2)*g_x(2, 0) + V_(3)*g_x(3, 0) + sl_x(0);
    Q_x_(1) = Fx(0, 1)*internal_0 + Fx(1, 1)*internal_1 + V_(0)*g_x(0, 1) + V_(1)*g_x(1, 1) + V_(22)*g_x(22, 1) + V_(23)*g_x(23, 1) + V_(24)*g_x(24, 1) + V_(25)*g_x(25, 1) + V_(26)*g_x(26, 1) + V_(27)*g_x(27, 1) + V_(8)*g_x(8, 1) + sl_x(1);
    Q_x_(2) = Fx(0, 2)*internal_0 + Fx(2, 2)*internal_2 + V_(0)*g_x(0, 2) + V_(1)*g_x(1, 2) + V_(22)*g_x(22, 2) + V_(23)*g_x(23, 2) + V_(24)*g_x(24, 2) + V_(25)*g_x(25, 2) + V_(26)*g_x(26, 2) + V_(27)*g_x(27, 2) + V_(8)*g_x(8, 2) + sl_x(2);
    Q_x_(3) = Fx(0, 3)*internal_0 + Fx(1, 3)*internal_1 + Fx(2, 3)*internal_2 + Fx(3, 3)*internal_3 + V_(22)*g_x(22, 3) + V_(23)*g_x(23, 3) + V_(24)*g_x(24, 3) + V_(25)*g_x(25, 3) + V_(26)*g_x(26, 3) + V_(27)*g_x(27, 3) + V_(9)*g_x(9, 3) + sl_x(3);
    Q_x_(4) = Fx(0, 4)*internal_0 + Fx(1, 4)*internal_1 + Fx(2, 4)*internal_2 + Fx(3, 4)*internal_3 + Fx(4, 4)*internal_4 + V_(5)*g_x(5, 4) + V_(6)*g_x(6, 4) + sl_x(4);
    Q_x_(5) = Fx(5, 5)*internal_5 + V_(4)*g_x(4, 5) + sl_x(5);
    Q_x_(6) = Fx(6, 6)*internal_6 + V_(22)*g_x(22, 6) + V_(23)*g_x(23, 6) + sl_x(6);
    Q_x_(7) = Fx(7, 7)*internal_7 + V_(22)*g_x(22, 7) + V_(23)*g_x(23, 7) + sl_x(7);
    Q_x_(8) = Fx(8, 8)*(V_x_prev(8) + V_xx_prev(8, 0)*r_f(0) + V_xx_prev(8, 1)*r_f(1) + V_xx_prev(8, 2)*r_f(2) + V_xx_prev(8, 3)*r_f(3) + V_xx_prev(8, 4)*r_f(4) + V_xx_prev(8, 5)*r_f(5) + V_xx_prev(8, 6)*r_f(6) + V_xx_prev(8, 7)*r_f(7) + V_xx_prev(8, 8)*r_f(8) + V_xx_prev(8, 9)*r_f(9)) + sl_x(8);
    Q_x_(9) = Fx(5, 9)*internal_5 + Fx(6, 9)*internal_6 + Fx(7, 9)*internal_7 + Fx(9, 9)*internal_8 + V_(7)*g_x(7, 9) + sl_x(9);

    // Evaluation of Vector Q_u_
    Q_u_(0) = Fu(4, 0)*internal_4 + V_(18)*g_u(18, 0) + V_(19)*g_u(19, 0) + sl_u(0);
    Q_u_(1) = Fu(3, 1)*internal_3 + V_(16)*g_u(16, 1) + V_(17)*g_u(17, 1) + sl_u(1);
    Q_u_(2) = Fu(9, 2)*internal_8 + V_(20)*g_u(20, 2) + V_(21)*g_u(21, 2) + sl_u(2);
    Q_u_(3) = V_(10)*g_u(10, 3) + V_(22)*g_u(22, 3) + sl_u(3);
    Q_u_(4) = V_(11)*g_u(11, 4) + V_(23)*g_u(23, 4) + sl_u(4);
    Q_u_(5) = V_(12)*g_u(12, 5) + V_(24)*g_u(24, 5) + sl_u(5);
    Q_u_(6) = V_(13)*g_u(13, 6) + V_(26)*g_u(26, 6) + sl_u(6);
    Q_u_(7) = V_(14)*g_u(14, 7) + V_(25)*g_u(25, 7) + sl_u(7);
    Q_u_(8) = V_(15)*g_u(15, 8) + V_(27)*g_u(27, 8) + sl_u(8);

  }

  virtual void updateQuuInv() override {
    // Determine internal variables
    const double internal_0 = 1.0/Q_uu_(0, 0);
    const double internal_3 = Q_uu_(0, 1)*internal_0;
    const double internal_5 = Q_uu_(0, 2)*internal_0;
    const double internal_1 = -pow(Q_uu_(0, 1), 2)*internal_0 + Q_uu_(1, 1);
    const double internal_2 = 1.0/internal_1;
    const double internal_6 = -Q_uu_(0, 1)*internal_5 + Q_uu_(1, 2);
    const double internal_4 = internal_2*internal_3;
    const double internal_7 = pow(internal_6, 2);
    const double internal_9 = internal_4*internal_6;
    const double internal_8 = 1.0/(-pow(Q_uu_(0, 2), 2)*internal_0 + Q_uu_(2, 2) - internal_2*internal_7);
    const double internal_10 = internal_8*(-internal_5 + internal_9);
    const double internal_13 = -internal_2*internal_6*internal_8;
    const double internal_12 = internal_2 + internal_7*internal_8/pow(internal_1, 2);
    const double internal_11 = -internal_10*internal_2*internal_6 - internal_4;

    // Evaluation of Matrix Q_uu_inv_
    Q_uu_inv_(0, 0) = internal_0 - internal_10*internal_5 - internal_11*internal_3;
    Q_uu_inv_(0, 1) = Q_uu_(0, 2)*internal_0*internal_2*internal_6*internal_8 - internal_12*internal_3;
    Q_uu_inv_(0, 2) = -internal_5*internal_8 + internal_8*internal_9;
    Q_uu_inv_(1, 0) = internal_11;
    Q_uu_inv_(1, 1) = internal_12;
    Q_uu_inv_(1, 2) = internal_13;
    Q_uu_inv_(2, 0) = internal_10;
    Q_uu_inv_(2, 1) = internal_13;
    Q_uu_inv_(2, 2) = internal_8;
    Q_uu_inv_(3, 3) = 1.0/Q_uu_(3, 3);
    Q_uu_inv_(4, 4) = 1.0/Q_uu_(4, 4);
    Q_uu_inv_(5, 5) = 1.0/Q_uu_(5, 5);
    Q_uu_inv_(6, 6) = 1.0/Q_uu_(6, 6);
    Q_uu_inv_(7, 7) = 1.0/Q_uu_(7, 7);
    Q_uu_inv_(8, 8) = 1.0/Q_uu_(8, 8);

  }

  virtual void updateKVU(Eigen::MatrixXd& k_ux, Eigen::VectorXd& v_u) override {
    // Evaluation of Matrix k_ux
    k_ux(0, 0) = -Q_uu_inv_(0, 0)*Q_xu_(0, 0) - Q_uu_inv_(0, 1)*Q_xu_(0, 1) - Q_uu_inv_(0, 2)*Q_xu_(0, 2);
    k_ux(0, 1) = -Q_uu_inv_(0, 0)*Q_xu_(1, 0) - Q_uu_inv_(0, 1)*Q_xu_(1, 1) - Q_uu_inv_(0, 2)*Q_xu_(1, 2);
    k_ux(0, 2) = -Q_uu_inv_(0, 0)*Q_xu_(2, 0) - Q_uu_inv_(0, 1)*Q_xu_(2, 1) - Q_uu_inv_(0, 2)*Q_xu_(2, 2);
    k_ux(0, 3) = -Q_uu_inv_(0, 0)*Q_xu_(3, 0) - Q_uu_inv_(0, 1)*Q_xu_(3, 1) - Q_uu_inv_(0, 2)*Q_xu_(3, 2);
    k_ux(0, 4) = -Q_uu_inv_(0, 0)*Q_xu_(4, 0) - Q_uu_inv_(0, 1)*Q_xu_(4, 1) - Q_uu_inv_(0, 2)*Q_xu_(4, 2);
    k_ux(0, 5) = -Q_uu_inv_(0, 0)*Q_xu_(5, 0) - Q_uu_inv_(0, 1)*Q_xu_(5, 1) - Q_uu_inv_(0, 2)*Q_xu_(5, 2);
    k_ux(0, 6) = -Q_uu_inv_(0, 0)*Q_xu_(6, 0) - Q_uu_inv_(0, 1)*Q_xu_(6, 1) - Q_uu_inv_(0, 2)*Q_xu_(6, 2);
    k_ux(0, 7) = -Q_uu_inv_(0, 0)*Q_xu_(7, 0) - Q_uu_inv_(0, 1)*Q_xu_(7, 1) - Q_uu_inv_(0, 2)*Q_xu_(7, 2);
    k_ux(0, 8) = -Q_uu_inv_(0, 0)*Q_xu_(8, 0) - Q_uu_inv_(0, 1)*Q_xu_(8, 1) - Q_uu_inv_(0, 2)*Q_xu_(8, 2);
    k_ux(0, 9) = -Q_uu_inv_(0, 0)*Q_xu_(9, 0) - Q_uu_inv_(0, 1)*Q_xu_(9, 1) - Q_uu_inv_(0, 2)*Q_xu_(9, 2);
    k_ux(1, 0) = -Q_uu_inv_(0, 1)*Q_xu_(0, 0) - Q_uu_inv_(1, 1)*Q_xu_(0, 1) - Q_uu_inv_(1, 2)*Q_xu_(0, 2);
    k_ux(1, 1) = -Q_uu_inv_(0, 1)*Q_xu_(1, 0) - Q_uu_inv_(1, 1)*Q_xu_(1, 1) - Q_uu_inv_(1, 2)*Q_xu_(1, 2);
    k_ux(1, 2) = -Q_uu_inv_(0, 1)*Q_xu_(2, 0) - Q_uu_inv_(1, 1)*Q_xu_(2, 1) - Q_uu_inv_(1, 2)*Q_xu_(2, 2);
    k_ux(1, 3) = -Q_uu_inv_(0, 1)*Q_xu_(3, 0) - Q_uu_inv_(1, 1)*Q_xu_(3, 1) - Q_uu_inv_(1, 2)*Q_xu_(3, 2);
    k_ux(1, 4) = -Q_uu_inv_(0, 1)*Q_xu_(4, 0) - Q_uu_inv_(1, 1)*Q_xu_(4, 1) - Q_uu_inv_(1, 2)*Q_xu_(4, 2);
    k_ux(1, 5) = -Q_uu_inv_(0, 1)*Q_xu_(5, 0) - Q_uu_inv_(1, 1)*Q_xu_(5, 1) - Q_uu_inv_(1, 2)*Q_xu_(5, 2);
    k_ux(1, 6) = -Q_uu_inv_(0, 1)*Q_xu_(6, 0) - Q_uu_inv_(1, 1)*Q_xu_(6, 1) - Q_uu_inv_(1, 2)*Q_xu_(6, 2);
    k_ux(1, 7) = -Q_uu_inv_(0, 1)*Q_xu_(7, 0) - Q_uu_inv_(1, 1)*Q_xu_(7, 1) - Q_uu_inv_(1, 2)*Q_xu_(7, 2);
    k_ux(1, 8) = -Q_uu_inv_(0, 1)*Q_xu_(8, 0) - Q_uu_inv_(1, 1)*Q_xu_(8, 1) - Q_uu_inv_(1, 2)*Q_xu_(8, 2);
    k_ux(1, 9) = -Q_uu_inv_(0, 1)*Q_xu_(9, 0) - Q_uu_inv_(1, 1)*Q_xu_(9, 1) - Q_uu_inv_(1, 2)*Q_xu_(9, 2);
    k_ux(2, 0) = -Q_uu_inv_(0, 2)*Q_xu_(0, 0) - Q_uu_inv_(1, 2)*Q_xu_(0, 1) - Q_uu_inv_(2, 2)*Q_xu_(0, 2);
    k_ux(2, 1) = -Q_uu_inv_(0, 2)*Q_xu_(1, 0) - Q_uu_inv_(1, 2)*Q_xu_(1, 1) - Q_uu_inv_(2, 2)*Q_xu_(1, 2);
    k_ux(2, 2) = -Q_uu_inv_(0, 2)*Q_xu_(2, 0) - Q_uu_inv_(1, 2)*Q_xu_(2, 1) - Q_uu_inv_(2, 2)*Q_xu_(2, 2);
    k_ux(2, 3) = -Q_uu_inv_(0, 2)*Q_xu_(3, 0) - Q_uu_inv_(1, 2)*Q_xu_(3, 1) - Q_uu_inv_(2, 2)*Q_xu_(3, 2);
    k_ux(2, 4) = -Q_uu_inv_(0, 2)*Q_xu_(4, 0) - Q_uu_inv_(1, 2)*Q_xu_(4, 1) - Q_uu_inv_(2, 2)*Q_xu_(4, 2);
    k_ux(2, 5) = -Q_uu_inv_(0, 2)*Q_xu_(5, 0) - Q_uu_inv_(1, 2)*Q_xu_(5, 1) - Q_uu_inv_(2, 2)*Q_xu_(5, 2);
    k_ux(2, 6) = -Q_uu_inv_(0, 2)*Q_xu_(6, 0) - Q_uu_inv_(1, 2)*Q_xu_(6, 1) - Q_uu_inv_(2, 2)*Q_xu_(6, 2);
    k_ux(2, 7) = -Q_uu_inv_(0, 2)*Q_xu_(7, 0) - Q_uu_inv_(1, 2)*Q_xu_(7, 1) - Q_uu_inv_(2, 2)*Q_xu_(7, 2);
    k_ux(2, 8) = -Q_uu_inv_(0, 2)*Q_xu_(8, 0) - Q_uu_inv_(1, 2)*Q_xu_(8, 1) - Q_uu_inv_(2, 2)*Q_xu_(8, 2);
    k_ux(2, 9) = -Q_uu_inv_(0, 2)*Q_xu_(9, 0) - Q_uu_inv_(1, 2)*Q_xu_(9, 1) - Q_uu_inv_(2, 2)*Q_xu_(9, 2);
    k_ux(3, 1) = -Q_uu_inv_(3, 3)*Q_xu_(1, 3);
    k_ux(3, 2) = -Q_uu_inv_(3, 3)*Q_xu_(2, 3);
    k_ux(3, 3) = -Q_uu_inv_(3, 3)*Q_xu_(3, 3);
    k_ux(3, 6) = -Q_uu_inv_(3, 3)*Q_xu_(6, 3);
    k_ux(3, 7) = -Q_uu_inv_(3, 3)*Q_xu_(7, 3);
    k_ux(4, 1) = -Q_uu_inv_(4, 4)*Q_xu_(1, 4);
    k_ux(4, 2) = -Q_uu_inv_(4, 4)*Q_xu_(2, 4);
    k_ux(4, 3) = -Q_uu_inv_(4, 4)*Q_xu_(3, 4);
    k_ux(4, 6) = -Q_uu_inv_(4, 4)*Q_xu_(6, 4);
    k_ux(4, 7) = -Q_uu_inv_(4, 4)*Q_xu_(7, 4);
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

    // Evaluation of Vector v_u
    v_u(0) = -Q_u_(0)*Q_uu_inv_(0, 0) - Q_u_(1)*Q_uu_inv_(0, 1) - Q_u_(2)*Q_uu_inv_(0, 2);
    v_u(1) = -Q_u_(0)*Q_uu_inv_(0, 1) - Q_u_(1)*Q_uu_inv_(1, 1) - Q_u_(2)*Q_uu_inv_(1, 2);
    v_u(2) = -Q_u_(0)*Q_uu_inv_(0, 2) - Q_u_(1)*Q_uu_inv_(1, 2) - Q_u_(2)*Q_uu_inv_(2, 2);
    v_u(3) = -Q_u_(3)*Q_uu_inv_(3, 3);
    v_u(4) = -Q_u_(4)*Q_uu_inv_(4, 4);
    v_u(5) = -Q_u_(5)*Q_uu_inv_(5, 5);
    v_u(6) = -Q_u_(6)*Q_uu_inv_(6, 6);
    v_u(7) = -Q_u_(7)*Q_uu_inv_(7, 7);
    v_u(8) = -Q_u_(8)*Q_uu_inv_(8, 8);

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
    V_xx(0, 0) = Q_xu_(0, 0)*k_ux(0, 0) + Q_xu_(0, 1)*k_ux(1, 0) + Q_xu_(0, 2)*k_ux(2, 0) + Q_xx_(0, 0);
    V_xx(0, 1) = Q_xu_(0, 0)*k_ux(0, 1) + Q_xu_(0, 1)*k_ux(1, 1) + Q_xu_(0, 2)*k_ux(2, 1) + Q_xx_(0, 1);
    V_xx(0, 2) = Q_xu_(0, 0)*k_ux(0, 2) + Q_xu_(0, 1)*k_ux(1, 2) + Q_xu_(0, 2)*k_ux(2, 2) + Q_xx_(0, 2);
    V_xx(0, 3) = Q_xu_(0, 0)*k_ux(0, 3) + Q_xu_(0, 1)*k_ux(1, 3) + Q_xu_(0, 2)*k_ux(2, 3) + Q_xx_(0, 3);
    V_xx(0, 4) = Q_xu_(0, 0)*k_ux(0, 4) + Q_xu_(0, 1)*k_ux(1, 4) + Q_xu_(0, 2)*k_ux(2, 4) + Q_xx_(0, 4);
    V_xx(0, 5) = Q_xu_(0, 0)*k_ux(0, 5) + Q_xu_(0, 1)*k_ux(1, 5) + Q_xu_(0, 2)*k_ux(2, 5) + Q_xx_(0, 5);
    V_xx(0, 6) = Q_xu_(0, 0)*k_ux(0, 6) + Q_xu_(0, 1)*k_ux(1, 6) + Q_xu_(0, 2)*k_ux(2, 6) + Q_xx_(0, 6);
    V_xx(0, 7) = Q_xu_(0, 0)*k_ux(0, 7) + Q_xu_(0, 1)*k_ux(1, 7) + Q_xu_(0, 2)*k_ux(2, 7) + Q_xx_(0, 7);
    V_xx(0, 8) = Q_xu_(0, 0)*k_ux(0, 8) + Q_xu_(0, 1)*k_ux(1, 8) + Q_xu_(0, 2)*k_ux(2, 8) + Q_xx_(0, 8);
    V_xx(0, 9) = Q_xu_(0, 0)*k_ux(0, 9) + Q_xu_(0, 1)*k_ux(1, 9) + Q_xu_(0, 2)*k_ux(2, 9) + Q_xx_(0, 9);
    V_xx(1, 0) = Q_xu_(1, 0)*k_ux(0, 0) + Q_xu_(1, 1)*k_ux(1, 0) + Q_xu_(1, 2)*k_ux(2, 0) + Q_xx_(0, 1);
    V_xx(1, 1) = Q_xu_(1, 0)*k_ux(0, 1) + Q_xu_(1, 1)*k_ux(1, 1) + Q_xu_(1, 2)*k_ux(2, 1) + Q_xu_(1, 3)*k_ux(3, 1) + Q_xu_(1, 4)*k_ux(4, 1) + Q_xu_(1, 5)*k_ux(5, 1) + Q_xu_(1, 6)*k_ux(6, 1) + Q_xu_(1, 7)*k_ux(7, 1) + Q_xu_(1, 8)*k_ux(8, 1) + Q_xx_(1, 1);
    V_xx(1, 2) = Q_xu_(1, 0)*k_ux(0, 2) + Q_xu_(1, 1)*k_ux(1, 2) + Q_xu_(1, 2)*k_ux(2, 2) + Q_xu_(1, 3)*k_ux(3, 2) + Q_xu_(1, 4)*k_ux(4, 2) + Q_xu_(1, 5)*k_ux(5, 2) + Q_xu_(1, 6)*k_ux(6, 2) + Q_xu_(1, 7)*k_ux(7, 2) + Q_xu_(1, 8)*k_ux(8, 2) + Q_xx_(1, 2);
    V_xx(1, 3) = Q_xu_(1, 0)*k_ux(0, 3) + Q_xu_(1, 1)*k_ux(1, 3) + Q_xu_(1, 2)*k_ux(2, 3) + Q_xu_(1, 3)*k_ux(3, 3) + Q_xu_(1, 4)*k_ux(4, 3) + Q_xu_(1, 5)*k_ux(5, 3) + Q_xu_(1, 6)*k_ux(6, 3) + Q_xu_(1, 7)*k_ux(7, 3) + Q_xu_(1, 8)*k_ux(8, 3) + Q_xx_(1, 3);
    V_xx(1, 4) = Q_xu_(1, 0)*k_ux(0, 4) + Q_xu_(1, 1)*k_ux(1, 4) + Q_xu_(1, 2)*k_ux(2, 4) + Q_xx_(1, 4);
    V_xx(1, 5) = Q_xu_(1, 0)*k_ux(0, 5) + Q_xu_(1, 1)*k_ux(1, 5) + Q_xu_(1, 2)*k_ux(2, 5) + Q_xx_(1, 5);
    V_xx(1, 6) = Q_xu_(1, 0)*k_ux(0, 6) + Q_xu_(1, 1)*k_ux(1, 6) + Q_xu_(1, 2)*k_ux(2, 6) + Q_xu_(1, 3)*k_ux(3, 6) + Q_xu_(1, 4)*k_ux(4, 6) + Q_xx_(1, 6);
    V_xx(1, 7) = Q_xu_(1, 0)*k_ux(0, 7) + Q_xu_(1, 1)*k_ux(1, 7) + Q_xu_(1, 2)*k_ux(2, 7) + Q_xu_(1, 3)*k_ux(3, 7) + Q_xu_(1, 4)*k_ux(4, 7) + Q_xx_(1, 7);
    V_xx(1, 8) = Q_xu_(1, 0)*k_ux(0, 8) + Q_xu_(1, 1)*k_ux(1, 8) + Q_xu_(1, 2)*k_ux(2, 8) + Q_xx_(1, 8);
    V_xx(1, 9) = Q_xu_(1, 0)*k_ux(0, 9) + Q_xu_(1, 1)*k_ux(1, 9) + Q_xu_(1, 2)*k_ux(2, 9) + Q_xx_(1, 9);
    V_xx(2, 0) = Q_xu_(2, 0)*k_ux(0, 0) + Q_xu_(2, 1)*k_ux(1, 0) + Q_xu_(2, 2)*k_ux(2, 0) + Q_xx_(0, 2);
    V_xx(2, 1) = Q_xu_(2, 0)*k_ux(0, 1) + Q_xu_(2, 1)*k_ux(1, 1) + Q_xu_(2, 2)*k_ux(2, 1) + Q_xu_(2, 3)*k_ux(3, 1) + Q_xu_(2, 4)*k_ux(4, 1) + Q_xu_(2, 5)*k_ux(5, 1) + Q_xu_(2, 6)*k_ux(6, 1) + Q_xu_(2, 7)*k_ux(7, 1) + Q_xu_(2, 8)*k_ux(8, 1) + Q_xx_(1, 2);
    V_xx(2, 2) = Q_xu_(2, 0)*k_ux(0, 2) + Q_xu_(2, 1)*k_ux(1, 2) + Q_xu_(2, 2)*k_ux(2, 2) + Q_xu_(2, 3)*k_ux(3, 2) + Q_xu_(2, 4)*k_ux(4, 2) + Q_xu_(2, 5)*k_ux(5, 2) + Q_xu_(2, 6)*k_ux(6, 2) + Q_xu_(2, 7)*k_ux(7, 2) + Q_xu_(2, 8)*k_ux(8, 2) + Q_xx_(2, 2);
    V_xx(2, 3) = Q_xu_(2, 0)*k_ux(0, 3) + Q_xu_(2, 1)*k_ux(1, 3) + Q_xu_(2, 2)*k_ux(2, 3) + Q_xu_(2, 3)*k_ux(3, 3) + Q_xu_(2, 4)*k_ux(4, 3) + Q_xu_(2, 5)*k_ux(5, 3) + Q_xu_(2, 6)*k_ux(6, 3) + Q_xu_(2, 7)*k_ux(7, 3) + Q_xu_(2, 8)*k_ux(8, 3) + Q_xx_(2, 3);
    V_xx(2, 4) = Q_xu_(2, 0)*k_ux(0, 4) + Q_xu_(2, 1)*k_ux(1, 4) + Q_xu_(2, 2)*k_ux(2, 4) + Q_xx_(2, 4);
    V_xx(2, 5) = Q_xu_(2, 0)*k_ux(0, 5) + Q_xu_(2, 1)*k_ux(1, 5) + Q_xu_(2, 2)*k_ux(2, 5) + Q_xx_(2, 5);
    V_xx(2, 6) = Q_xu_(2, 0)*k_ux(0, 6) + Q_xu_(2, 1)*k_ux(1, 6) + Q_xu_(2, 2)*k_ux(2, 6) + Q_xu_(2, 3)*k_ux(3, 6) + Q_xu_(2, 4)*k_ux(4, 6) + Q_xx_(2, 6);
    V_xx(2, 7) = Q_xu_(2, 0)*k_ux(0, 7) + Q_xu_(2, 1)*k_ux(1, 7) + Q_xu_(2, 2)*k_ux(2, 7) + Q_xu_(2, 3)*k_ux(3, 7) + Q_xu_(2, 4)*k_ux(4, 7) + Q_xx_(2, 7);
    V_xx(2, 8) = Q_xu_(2, 0)*k_ux(0, 8) + Q_xu_(2, 1)*k_ux(1, 8) + Q_xu_(2, 2)*k_ux(2, 8) + Q_xx_(2, 8);
    V_xx(2, 9) = Q_xu_(2, 0)*k_ux(0, 9) + Q_xu_(2, 1)*k_ux(1, 9) + Q_xu_(2, 2)*k_ux(2, 9) + Q_xx_(2, 9);
    V_xx(3, 0) = Q_xu_(3, 0)*k_ux(0, 0) + Q_xu_(3, 1)*k_ux(1, 0) + Q_xu_(3, 2)*k_ux(2, 0) + Q_xx_(0, 3);
    V_xx(3, 1) = Q_xu_(3, 0)*k_ux(0, 1) + Q_xu_(3, 1)*k_ux(1, 1) + Q_xu_(3, 2)*k_ux(2, 1) + Q_xu_(3, 3)*k_ux(3, 1) + Q_xu_(3, 4)*k_ux(4, 1) + Q_xu_(3, 5)*k_ux(5, 1) + Q_xu_(3, 6)*k_ux(6, 1) + Q_xu_(3, 7)*k_ux(7, 1) + Q_xu_(3, 8)*k_ux(8, 1) + Q_xx_(1, 3);
    V_xx(3, 2) = Q_xu_(3, 0)*k_ux(0, 2) + Q_xu_(3, 1)*k_ux(1, 2) + Q_xu_(3, 2)*k_ux(2, 2) + Q_xu_(3, 3)*k_ux(3, 2) + Q_xu_(3, 4)*k_ux(4, 2) + Q_xu_(3, 5)*k_ux(5, 2) + Q_xu_(3, 6)*k_ux(6, 2) + Q_xu_(3, 7)*k_ux(7, 2) + Q_xu_(3, 8)*k_ux(8, 2) + Q_xx_(2, 3);
    V_xx(3, 3) = Q_xu_(3, 0)*k_ux(0, 3) + Q_xu_(3, 1)*k_ux(1, 3) + Q_xu_(3, 2)*k_ux(2, 3) + Q_xu_(3, 3)*k_ux(3, 3) + Q_xu_(3, 4)*k_ux(4, 3) + Q_xu_(3, 5)*k_ux(5, 3) + Q_xu_(3, 6)*k_ux(6, 3) + Q_xu_(3, 7)*k_ux(7, 3) + Q_xu_(3, 8)*k_ux(8, 3) + Q_xx_(3, 3);
    V_xx(3, 4) = Q_xu_(3, 0)*k_ux(0, 4) + Q_xu_(3, 1)*k_ux(1, 4) + Q_xu_(3, 2)*k_ux(2, 4) + Q_xx_(3, 4);
    V_xx(3, 5) = Q_xu_(3, 0)*k_ux(0, 5) + Q_xu_(3, 1)*k_ux(1, 5) + Q_xu_(3, 2)*k_ux(2, 5) + Q_xx_(3, 5);
    V_xx(3, 6) = Q_xu_(3, 0)*k_ux(0, 6) + Q_xu_(3, 1)*k_ux(1, 6) + Q_xu_(3, 2)*k_ux(2, 6) + Q_xu_(3, 3)*k_ux(3, 6) + Q_xu_(3, 4)*k_ux(4, 6) + Q_xx_(3, 6);
    V_xx(3, 7) = Q_xu_(3, 0)*k_ux(0, 7) + Q_xu_(3, 1)*k_ux(1, 7) + Q_xu_(3, 2)*k_ux(2, 7) + Q_xu_(3, 3)*k_ux(3, 7) + Q_xu_(3, 4)*k_ux(4, 7) + Q_xx_(3, 7);
    V_xx(3, 8) = Q_xu_(3, 0)*k_ux(0, 8) + Q_xu_(3, 1)*k_ux(1, 8) + Q_xu_(3, 2)*k_ux(2, 8) + Q_xx_(3, 8);
    V_xx(3, 9) = Q_xu_(3, 0)*k_ux(0, 9) + Q_xu_(3, 1)*k_ux(1, 9) + Q_xu_(3, 2)*k_ux(2, 9) + Q_xx_(3, 9);
    V_xx(4, 0) = Q_xu_(4, 0)*k_ux(0, 0) + Q_xu_(4, 1)*k_ux(1, 0) + Q_xu_(4, 2)*k_ux(2, 0) + Q_xx_(0, 4);
    V_xx(4, 1) = Q_xu_(4, 0)*k_ux(0, 1) + Q_xu_(4, 1)*k_ux(1, 1) + Q_xu_(4, 2)*k_ux(2, 1) + Q_xx_(1, 4);
    V_xx(4, 2) = Q_xu_(4, 0)*k_ux(0, 2) + Q_xu_(4, 1)*k_ux(1, 2) + Q_xu_(4, 2)*k_ux(2, 2) + Q_xx_(2, 4);
    V_xx(4, 3) = Q_xu_(4, 0)*k_ux(0, 3) + Q_xu_(4, 1)*k_ux(1, 3) + Q_xu_(4, 2)*k_ux(2, 3) + Q_xx_(3, 4);
    V_xx(4, 4) = Q_xu_(4, 0)*k_ux(0, 4) + Q_xu_(4, 1)*k_ux(1, 4) + Q_xu_(4, 2)*k_ux(2, 4) + Q_xx_(4, 4);
    V_xx(4, 5) = Q_xu_(4, 0)*k_ux(0, 5) + Q_xu_(4, 1)*k_ux(1, 5) + Q_xu_(4, 2)*k_ux(2, 5) + Q_xx_(4, 5);
    V_xx(4, 6) = Q_xu_(4, 0)*k_ux(0, 6) + Q_xu_(4, 1)*k_ux(1, 6) + Q_xu_(4, 2)*k_ux(2, 6) + Q_xx_(4, 6);
    V_xx(4, 7) = Q_xu_(4, 0)*k_ux(0, 7) + Q_xu_(4, 1)*k_ux(1, 7) + Q_xu_(4, 2)*k_ux(2, 7) + Q_xx_(4, 7);
    V_xx(4, 8) = Q_xu_(4, 0)*k_ux(0, 8) + Q_xu_(4, 1)*k_ux(1, 8) + Q_xu_(4, 2)*k_ux(2, 8) + Q_xx_(4, 8);
    V_xx(4, 9) = Q_xu_(4, 0)*k_ux(0, 9) + Q_xu_(4, 1)*k_ux(1, 9) + Q_xu_(4, 2)*k_ux(2, 9) + Q_xx_(4, 9);
    V_xx(5, 0) = Q_xu_(5, 0)*k_ux(0, 0) + Q_xu_(5, 1)*k_ux(1, 0) + Q_xu_(5, 2)*k_ux(2, 0) + Q_xx_(0, 5);
    V_xx(5, 1) = Q_xu_(5, 0)*k_ux(0, 1) + Q_xu_(5, 1)*k_ux(1, 1) + Q_xu_(5, 2)*k_ux(2, 1) + Q_xx_(1, 5);
    V_xx(5, 2) = Q_xu_(5, 0)*k_ux(0, 2) + Q_xu_(5, 1)*k_ux(1, 2) + Q_xu_(5, 2)*k_ux(2, 2) + Q_xx_(2, 5);
    V_xx(5, 3) = Q_xu_(5, 0)*k_ux(0, 3) + Q_xu_(5, 1)*k_ux(1, 3) + Q_xu_(5, 2)*k_ux(2, 3) + Q_xx_(3, 5);
    V_xx(5, 4) = Q_xu_(5, 0)*k_ux(0, 4) + Q_xu_(5, 1)*k_ux(1, 4) + Q_xu_(5, 2)*k_ux(2, 4) + Q_xx_(4, 5);
    V_xx(5, 5) = Q_xu_(5, 0)*k_ux(0, 5) + Q_xu_(5, 1)*k_ux(1, 5) + Q_xu_(5, 2)*k_ux(2, 5) + Q_xx_(5, 5);
    V_xx(5, 6) = Q_xu_(5, 0)*k_ux(0, 6) + Q_xu_(5, 1)*k_ux(1, 6) + Q_xu_(5, 2)*k_ux(2, 6) + Q_xx_(5, 6);
    V_xx(5, 7) = Q_xu_(5, 0)*k_ux(0, 7) + Q_xu_(5, 1)*k_ux(1, 7) + Q_xu_(5, 2)*k_ux(2, 7) + Q_xx_(5, 7);
    V_xx(5, 8) = Q_xu_(5, 0)*k_ux(0, 8) + Q_xu_(5, 1)*k_ux(1, 8) + Q_xu_(5, 2)*k_ux(2, 8) + Q_xx_(5, 8);
    V_xx(5, 9) = Q_xu_(5, 0)*k_ux(0, 9) + Q_xu_(5, 1)*k_ux(1, 9) + Q_xu_(5, 2)*k_ux(2, 9) + Q_xx_(5, 9);
    V_xx(6, 0) = Q_xu_(6, 0)*k_ux(0, 0) + Q_xu_(6, 1)*k_ux(1, 0) + Q_xu_(6, 2)*k_ux(2, 0) + Q_xx_(0, 6);
    V_xx(6, 1) = Q_xu_(6, 0)*k_ux(0, 1) + Q_xu_(6, 1)*k_ux(1, 1) + Q_xu_(6, 2)*k_ux(2, 1) + Q_xu_(6, 3)*k_ux(3, 1) + Q_xu_(6, 4)*k_ux(4, 1) + Q_xx_(1, 6);
    V_xx(6, 2) = Q_xu_(6, 0)*k_ux(0, 2) + Q_xu_(6, 1)*k_ux(1, 2) + Q_xu_(6, 2)*k_ux(2, 2) + Q_xu_(6, 3)*k_ux(3, 2) + Q_xu_(6, 4)*k_ux(4, 2) + Q_xx_(2, 6);
    V_xx(6, 3) = Q_xu_(6, 0)*k_ux(0, 3) + Q_xu_(6, 1)*k_ux(1, 3) + Q_xu_(6, 2)*k_ux(2, 3) + Q_xu_(6, 3)*k_ux(3, 3) + Q_xu_(6, 4)*k_ux(4, 3) + Q_xx_(3, 6);
    V_xx(6, 4) = Q_xu_(6, 0)*k_ux(0, 4) + Q_xu_(6, 1)*k_ux(1, 4) + Q_xu_(6, 2)*k_ux(2, 4) + Q_xx_(4, 6);
    V_xx(6, 5) = Q_xu_(6, 0)*k_ux(0, 5) + Q_xu_(6, 1)*k_ux(1, 5) + Q_xu_(6, 2)*k_ux(2, 5) + Q_xx_(5, 6);
    V_xx(6, 6) = Q_xu_(6, 0)*k_ux(0, 6) + Q_xu_(6, 1)*k_ux(1, 6) + Q_xu_(6, 2)*k_ux(2, 6) + Q_xu_(6, 3)*k_ux(3, 6) + Q_xu_(6, 4)*k_ux(4, 6) + Q_xx_(6, 6);
    V_xx(6, 7) = Q_xu_(6, 0)*k_ux(0, 7) + Q_xu_(6, 1)*k_ux(1, 7) + Q_xu_(6, 2)*k_ux(2, 7) + Q_xu_(6, 3)*k_ux(3, 7) + Q_xu_(6, 4)*k_ux(4, 7) + Q_xx_(6, 7);
    V_xx(6, 8) = Q_xu_(6, 0)*k_ux(0, 8) + Q_xu_(6, 1)*k_ux(1, 8) + Q_xu_(6, 2)*k_ux(2, 8) + Q_xx_(6, 8);
    V_xx(6, 9) = Q_xu_(6, 0)*k_ux(0, 9) + Q_xu_(6, 1)*k_ux(1, 9) + Q_xu_(6, 2)*k_ux(2, 9) + Q_xx_(6, 9);
    V_xx(7, 0) = Q_xu_(7, 0)*k_ux(0, 0) + Q_xu_(7, 1)*k_ux(1, 0) + Q_xu_(7, 2)*k_ux(2, 0) + Q_xx_(0, 7);
    V_xx(7, 1) = Q_xu_(7, 0)*k_ux(0, 1) + Q_xu_(7, 1)*k_ux(1, 1) + Q_xu_(7, 2)*k_ux(2, 1) + Q_xu_(7, 3)*k_ux(3, 1) + Q_xu_(7, 4)*k_ux(4, 1) + Q_xx_(1, 7);
    V_xx(7, 2) = Q_xu_(7, 0)*k_ux(0, 2) + Q_xu_(7, 1)*k_ux(1, 2) + Q_xu_(7, 2)*k_ux(2, 2) + Q_xu_(7, 3)*k_ux(3, 2) + Q_xu_(7, 4)*k_ux(4, 2) + Q_xx_(2, 7);
    V_xx(7, 3) = Q_xu_(7, 0)*k_ux(0, 3) + Q_xu_(7, 1)*k_ux(1, 3) + Q_xu_(7, 2)*k_ux(2, 3) + Q_xu_(7, 3)*k_ux(3, 3) + Q_xu_(7, 4)*k_ux(4, 3) + Q_xx_(3, 7);
    V_xx(7, 4) = Q_xu_(7, 0)*k_ux(0, 4) + Q_xu_(7, 1)*k_ux(1, 4) + Q_xu_(7, 2)*k_ux(2, 4) + Q_xx_(4, 7);
    V_xx(7, 5) = Q_xu_(7, 0)*k_ux(0, 5) + Q_xu_(7, 1)*k_ux(1, 5) + Q_xu_(7, 2)*k_ux(2, 5) + Q_xx_(5, 7);
    V_xx(7, 6) = Q_xu_(7, 0)*k_ux(0, 6) + Q_xu_(7, 1)*k_ux(1, 6) + Q_xu_(7, 2)*k_ux(2, 6) + Q_xu_(7, 3)*k_ux(3, 6) + Q_xu_(7, 4)*k_ux(4, 6) + Q_xx_(6, 7);
    V_xx(7, 7) = Q_xu_(7, 0)*k_ux(0, 7) + Q_xu_(7, 1)*k_ux(1, 7) + Q_xu_(7, 2)*k_ux(2, 7) + Q_xu_(7, 3)*k_ux(3, 7) + Q_xu_(7, 4)*k_ux(4, 7) + Q_xx_(7, 7);
    V_xx(7, 8) = Q_xu_(7, 0)*k_ux(0, 8) + Q_xu_(7, 1)*k_ux(1, 8) + Q_xu_(7, 2)*k_ux(2, 8) + Q_xx_(7, 8);
    V_xx(7, 9) = Q_xu_(7, 0)*k_ux(0, 9) + Q_xu_(7, 1)*k_ux(1, 9) + Q_xu_(7, 2)*k_ux(2, 9) + Q_xx_(7, 9);
    V_xx(8, 0) = Q_xu_(8, 0)*k_ux(0, 0) + Q_xu_(8, 1)*k_ux(1, 0) + Q_xu_(8, 2)*k_ux(2, 0) + Q_xx_(0, 8);
    V_xx(8, 1) = Q_xu_(8, 0)*k_ux(0, 1) + Q_xu_(8, 1)*k_ux(1, 1) + Q_xu_(8, 2)*k_ux(2, 1) + Q_xx_(1, 8);
    V_xx(8, 2) = Q_xu_(8, 0)*k_ux(0, 2) + Q_xu_(8, 1)*k_ux(1, 2) + Q_xu_(8, 2)*k_ux(2, 2) + Q_xx_(2, 8);
    V_xx(8, 3) = Q_xu_(8, 0)*k_ux(0, 3) + Q_xu_(8, 1)*k_ux(1, 3) + Q_xu_(8, 2)*k_ux(2, 3) + Q_xx_(3, 8);
    V_xx(8, 4) = Q_xu_(8, 0)*k_ux(0, 4) + Q_xu_(8, 1)*k_ux(1, 4) + Q_xu_(8, 2)*k_ux(2, 4) + Q_xx_(4, 8);
    V_xx(8, 5) = Q_xu_(8, 0)*k_ux(0, 5) + Q_xu_(8, 1)*k_ux(1, 5) + Q_xu_(8, 2)*k_ux(2, 5) + Q_xx_(5, 8);
    V_xx(8, 6) = Q_xu_(8, 0)*k_ux(0, 6) + Q_xu_(8, 1)*k_ux(1, 6) + Q_xu_(8, 2)*k_ux(2, 6) + Q_xx_(6, 8);
    V_xx(8, 7) = Q_xu_(8, 0)*k_ux(0, 7) + Q_xu_(8, 1)*k_ux(1, 7) + Q_xu_(8, 2)*k_ux(2, 7) + Q_xx_(7, 8);
    V_xx(8, 8) = Q_xu_(8, 0)*k_ux(0, 8) + Q_xu_(8, 1)*k_ux(1, 8) + Q_xu_(8, 2)*k_ux(2, 8) + Q_xx_(8, 8);
    V_xx(8, 9) = Q_xu_(8, 0)*k_ux(0, 9) + Q_xu_(8, 1)*k_ux(1, 9) + Q_xu_(8, 2)*k_ux(2, 9) + Q_xx_(8, 9);
    V_xx(9, 0) = Q_xu_(9, 0)*k_ux(0, 0) + Q_xu_(9, 1)*k_ux(1, 0) + Q_xu_(9, 2)*k_ux(2, 0) + Q_xx_(0, 9);
    V_xx(9, 1) = Q_xu_(9, 0)*k_ux(0, 1) + Q_xu_(9, 1)*k_ux(1, 1) + Q_xu_(9, 2)*k_ux(2, 1) + Q_xx_(1, 9);
    V_xx(9, 2) = Q_xu_(9, 0)*k_ux(0, 2) + Q_xu_(9, 1)*k_ux(1, 2) + Q_xu_(9, 2)*k_ux(2, 2) + Q_xx_(2, 9);
    V_xx(9, 3) = Q_xu_(9, 0)*k_ux(0, 3) + Q_xu_(9, 1)*k_ux(1, 3) + Q_xu_(9, 2)*k_ux(2, 3) + Q_xx_(3, 9);
    V_xx(9, 4) = Q_xu_(9, 0)*k_ux(0, 4) + Q_xu_(9, 1)*k_ux(1, 4) + Q_xu_(9, 2)*k_ux(2, 4) + Q_xx_(4, 9);
    V_xx(9, 5) = Q_xu_(9, 0)*k_ux(0, 5) + Q_xu_(9, 1)*k_ux(1, 5) + Q_xu_(9, 2)*k_ux(2, 5) + Q_xx_(5, 9);
    V_xx(9, 6) = Q_xu_(9, 0)*k_ux(0, 6) + Q_xu_(9, 1)*k_ux(1, 6) + Q_xu_(9, 2)*k_ux(2, 6) + Q_xx_(6, 9);
    V_xx(9, 7) = Q_xu_(9, 0)*k_ux(0, 7) + Q_xu_(9, 1)*k_ux(1, 7) + Q_xu_(9, 2)*k_ux(2, 7) + Q_xx_(7, 9);
    V_xx(9, 8) = Q_xu_(9, 0)*k_ux(0, 8) + Q_xu_(9, 1)*k_ux(1, 8) + Q_xu_(9, 2)*k_ux(2, 8) + Q_xx_(8, 9);
    V_xx(9, 9) = Q_xu_(9, 0)*k_ux(0, 9) + Q_xu_(9, 1)*k_ux(1, 9) + Q_xu_(9, 2)*k_ux(2, 9) + Q_xx_(9, 9);

    // Evaluation of Vector V_x
    V_x(0) = Q_x_(0) + Q_xu_(0, 0)*v_u(0) + Q_xu_(0, 1)*v_u(1) + Q_xu_(0, 2)*v_u(2);
    V_x(1) = Q_x_(1) + Q_xu_(1, 0)*v_u(0) + Q_xu_(1, 1)*v_u(1) + Q_xu_(1, 2)*v_u(2) + Q_xu_(1, 3)*v_u(3) + Q_xu_(1, 4)*v_u(4) + Q_xu_(1, 5)*v_u(5) + Q_xu_(1, 6)*v_u(6) + Q_xu_(1, 7)*v_u(7) + Q_xu_(1, 8)*v_u(8);
    V_x(2) = Q_x_(2) + Q_xu_(2, 0)*v_u(0) + Q_xu_(2, 1)*v_u(1) + Q_xu_(2, 2)*v_u(2) + Q_xu_(2, 3)*v_u(3) + Q_xu_(2, 4)*v_u(4) + Q_xu_(2, 5)*v_u(5) + Q_xu_(2, 6)*v_u(6) + Q_xu_(2, 7)*v_u(7) + Q_xu_(2, 8)*v_u(8);
    V_x(3) = Q_x_(3) + Q_xu_(3, 0)*v_u(0) + Q_xu_(3, 1)*v_u(1) + Q_xu_(3, 2)*v_u(2) + Q_xu_(3, 3)*v_u(3) + Q_xu_(3, 4)*v_u(4) + Q_xu_(3, 5)*v_u(5) + Q_xu_(3, 6)*v_u(6) + Q_xu_(3, 7)*v_u(7) + Q_xu_(3, 8)*v_u(8);
    V_x(4) = Q_x_(4) + Q_xu_(4, 0)*v_u(0) + Q_xu_(4, 1)*v_u(1) + Q_xu_(4, 2)*v_u(2);
    V_x(5) = Q_x_(5) + Q_xu_(5, 0)*v_u(0) + Q_xu_(5, 1)*v_u(1) + Q_xu_(5, 2)*v_u(2);
    V_x(6) = Q_x_(6) + Q_xu_(6, 0)*v_u(0) + Q_xu_(6, 1)*v_u(1) + Q_xu_(6, 2)*v_u(2) + Q_xu_(6, 3)*v_u(3) + Q_xu_(6, 4)*v_u(4);
    V_x(7) = Q_x_(7) + Q_xu_(7, 0)*v_u(0) + Q_xu_(7, 1)*v_u(1) + Q_xu_(7, 2)*v_u(2) + Q_xu_(7, 3)*v_u(3) + Q_xu_(7, 4)*v_u(4);
    V_x(8) = Q_x_(8) + Q_xu_(8, 0)*v_u(0) + Q_xu_(8, 1)*v_u(1) + Q_xu_(8, 2)*v_u(2);
    V_x(9) = Q_x_(9) + Q_xu_(9, 0)*v_u(0) + Q_xu_(9, 1)*v_u(1) + Q_xu_(9, 2)*v_u(2);

  }

  virtual void updateQnx(const Eigen::VectorXd& r_f, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& h_u, const Eigen::MatrixXd& k_ux, const Eigen::MatrixXd& k_ex, const Eigen::MatrixXd& k_ue, const Eigen::VectorXd& v_u, const Eigen::VectorXd& v_e) override {
    // Determine internal variables
    const double internal_0 = -Q_uu_(0, 1);
    const double internal_1 = -Q_uu_(0, 2);
    const double internal_2 = -Q_uu_(1, 2);

    // Evaluation of Matrix Q_un_
    Q_un_(0, 3) = -Fu(3, 1)*Q_uu_inv_(0, 1);
    Q_un_(0, 4) = -Fu(4, 0)*Q_uu_inv_(0, 0);
    Q_un_(0, 9) = -Fu(9, 2)*Q_uu_inv_(0, 2);
    Q_un_(1, 3) = -Fu(3, 1)*Q_uu_inv_(1, 1);
    Q_un_(1, 4) = -Fu(4, 0)*Q_uu_inv_(0, 1);
    Q_un_(1, 9) = -Fu(9, 2)*Q_uu_inv_(1, 2);
    Q_un_(2, 3) = -Fu(3, 1)*Q_uu_inv_(1, 2);
    Q_un_(2, 4) = -Fu(4, 0)*Q_uu_inv_(0, 2);
    Q_un_(2, 9) = -Fu(9, 2)*Q_uu_inv_(2, 2);

    // Evaluation of Matrix Q_nuu_
    Q_nuu_(0, 0) = -Q_uu_(0, 0);
    Q_nuu_(0, 1) = internal_0;
    Q_nuu_(0, 2) = internal_1;
    Q_nuu_(1, 0) = internal_0;
    Q_nuu_(1, 1) = -Q_uu_(1, 1);
    Q_nuu_(1, 2) = internal_2;
    Q_nuu_(2, 0) = internal_1;
    Q_nuu_(2, 1) = internal_2;
    Q_nuu_(2, 2) = -Q_uu_(2, 2);
    Q_nuu_(3, 3) = -Q_uu_(3, 3);
    Q_nuu_(4, 4) = -Q_uu_(4, 4);
    Q_nuu_(5, 5) = -Q_uu_(5, 5);
    Q_nuu_(6, 6) = -Q_uu_(6, 6);
    Q_nuu_(7, 7) = -Q_uu_(7, 7);
    Q_nuu_(8, 8) = -Q_uu_(8, 8);

    // Evaluation of Matrix Q_vnx_
    Q_vnx_(0, 0) = Fx(0, 0);
    Q_vnx_(0, 1) = Fx(0, 1);
    Q_vnx_(0, 2) = Fx(0, 2);
    Q_vnx_(0, 3) = Fx(0, 3);
    Q_vnx_(0, 4) = Fx(0, 4);
    Q_vnx_(1, 1) = Fx(1, 1);
    Q_vnx_(1, 3) = Fx(1, 3);
    Q_vnx_(1, 4) = Fx(1, 4);
    Q_vnx_(2, 2) = Fx(2, 2);
    Q_vnx_(2, 3) = Fx(2, 3);
    Q_vnx_(2, 4) = Fx(2, 4);
    Q_vnx_(3, 0) = Fu(3, 1)*k_ux(1, 0);
    Q_vnx_(3, 1) = Fu(3, 1)*k_ux(1, 1);
    Q_vnx_(3, 2) = Fu(3, 1)*k_ux(1, 2);
    Q_vnx_(3, 3) = Fu(3, 1)*k_ux(1, 3) + Fx(3, 3);
    Q_vnx_(3, 4) = Fu(3, 1)*k_ux(1, 4) + Fx(3, 4);
    Q_vnx_(3, 5) = Fu(3, 1)*k_ux(1, 5);
    Q_vnx_(3, 6) = Fu(3, 1)*k_ux(1, 6);
    Q_vnx_(3, 7) = Fu(3, 1)*k_ux(1, 7);
    Q_vnx_(3, 8) = Fu(3, 1)*k_ux(1, 8);
    Q_vnx_(3, 9) = Fu(3, 1)*k_ux(1, 9);
    Q_vnx_(4, 0) = Fu(4, 0)*k_ux(0, 0);
    Q_vnx_(4, 1) = Fu(4, 0)*k_ux(0, 1);
    Q_vnx_(4, 2) = Fu(4, 0)*k_ux(0, 2);
    Q_vnx_(4, 3) = Fu(4, 0)*k_ux(0, 3);
    Q_vnx_(4, 4) = Fu(4, 0)*k_ux(0, 4) + Fx(4, 4);
    Q_vnx_(4, 5) = Fu(4, 0)*k_ux(0, 5);
    Q_vnx_(4, 6) = Fu(4, 0)*k_ux(0, 6);
    Q_vnx_(4, 7) = Fu(4, 0)*k_ux(0, 7);
    Q_vnx_(4, 8) = Fu(4, 0)*k_ux(0, 8);
    Q_vnx_(4, 9) = Fu(4, 0)*k_ux(0, 9);
    Q_vnx_(5, 5) = Fx(5, 5);
    Q_vnx_(5, 9) = Fx(5, 9);
    Q_vnx_(6, 6) = Fx(6, 6);
    Q_vnx_(6, 9) = Fx(6, 9);
    Q_vnx_(7, 7) = Fx(7, 7);
    Q_vnx_(7, 9) = Fx(7, 9);
    Q_vnx_(8, 8) = Fx(8, 8);
    Q_vnx_(9, 0) = Fu(9, 2)*k_ux(2, 0);
    Q_vnx_(9, 1) = Fu(9, 2)*k_ux(2, 1);
    Q_vnx_(9, 2) = Fu(9, 2)*k_ux(2, 2);
    Q_vnx_(9, 3) = Fu(9, 2)*k_ux(2, 3);
    Q_vnx_(9, 4) = Fu(9, 2)*k_ux(2, 4);
    Q_vnx_(9, 5) = Fu(9, 2)*k_ux(2, 5);
    Q_vnx_(9, 6) = Fu(9, 2)*k_ux(2, 6);
    Q_vnx_(9, 7) = Fu(9, 2)*k_ux(2, 7);
    Q_vnx_(9, 8) = Fu(9, 2)*k_ux(2, 8);
    Q_vnx_(9, 9) = Fu(9, 2)*k_ux(2, 9) + Fx(9, 9);

    // Evaluation of Vector Q_vn_
    Q_vn_(0) = r_f(0);
    Q_vn_(1) = r_f(1);
    Q_vn_(2) = r_f(2);
    Q_vn_(3) = Fu(3, 1)*v_u(1) + r_f(3);
    Q_vn_(4) = Fu(4, 0)*v_u(0) + r_f(4);
    Q_vn_(5) = r_f(5);
    Q_vn_(6) = r_f(6);
    Q_vn_(7) = r_f(7);
    Q_vn_(8) = r_f(8);
    Q_vn_(9) = Fu(9, 2)*v_u(2) + r_f(9);

  }

};

template <>
class MultiAgentsTrajectoryModelIpmEvaluator<StageType::INITIAL, OcpConfig::FORWARD_EULER> : public IpmEvaluator {
 public:
  MultiAgentsTrajectoryModelIpmEvaluator() : IpmEvaluator(10, 9, 12, 0, 10) {}
  virtual ~MultiAgentsTrajectoryModelIpmEvaluator() = default;

  virtual void updateLX(const Eigen::VectorXd& l_x, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& h_x, const Eigen::MatrixXd& m_x, const Eigen::VectorXd& p, const Eigen::VectorXd& lambda, const Eigen::VectorXd& eta, const Eigen::VectorXd& nu, Eigen::VectorXd& sl_x) const override {
    // Evaluation of Vector sl_x
    sl_x(0) = Fx(0, 0)*p(0) + l_x(0) + m_x(0, 0)*nu(0);
    sl_x(1) = Fx(0, 1)*p(0) + Fx(1, 1)*p(1) + l_x(1) + m_x(1, 1)*nu(1);
    sl_x(2) = Fx(0, 2)*p(0) + Fx(2, 2)*p(2) + l_x(2) + m_x(2, 2)*nu(2);
    sl_x(3) = Fx(0, 3)*p(0) + Fx(1, 3)*p(1) + Fx(2, 3)*p(2) + Fx(3, 3)*p(3) + l_x(3) + m_x(3, 3)*nu(3);
    sl_x(4) = Fx(0, 4)*p(0) + Fx(1, 4)*p(1) + Fx(2, 4)*p(2) + Fx(3, 4)*p(3) + Fx(4, 4)*p(4) + l_x(4) + m_x(4, 4)*nu(4);
    sl_x(5) = Fx(5, 5)*p(5) + l_x(5) + m_x(5, 5)*nu(5);
    sl_x(6) = Fx(6, 6)*p(6) + m_x(6, 6)*nu(6);
    sl_x(7) = Fx(7, 7)*p(7) + m_x(7, 7)*nu(7);
    sl_x(8) = Fx(8, 8)*p(8) + m_x(8, 8)*nu(8);
    sl_x(9) = Fx(5, 9)*p(5) + Fx(6, 9)*p(6) + Fx(7, 9)*p(7) + Fx(9, 9)*p(9) + l_x(9) + m_x(9, 9)*nu(9);

  }

  virtual void updateLU(const Eigen::VectorXd& l_u, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& g_u, const Eigen::MatrixXd& h_u, const Eigen::VectorXd& p, const Eigen::VectorXd& lambda, const Eigen::VectorXd& eta, Eigen::VectorXd& sl_u) const override {
    // Evaluation of Vector sl_u
    sl_u(0) = Fu(4, 0)*p(4) + g_u(8, 0)*lambda(8) + g_u(9, 0)*lambda(9) + l_u(0);
    sl_u(1) = Fu(3, 1)*p(3) + g_u(6, 1)*lambda(6) + g_u(7, 1)*lambda(7) + l_u(1);
    sl_u(2) = Fu(9, 2)*p(9) + g_u(10, 2)*lambda(10) + g_u(11, 2)*lambda(11) + l_u(2);
    sl_u(3) = g_u(0, 3)*lambda(0) + l_u(3);
    sl_u(4) = g_u(1, 4)*lambda(1) + l_u(4);
    sl_u(5) = g_u(2, 5)*lambda(2) + l_u(5);
    sl_u(6) = g_u(3, 6)*lambda(3) + l_u(6);
    sl_u(7) = g_u(4, 7)*lambda(4) + l_u(7);
    sl_u(8) = g_u(5, 8)*lambda(5) + l_u(8);

  }

  virtual void updateQH(const Eigen::MatrixXd& l_xx, const Eigen::MatrixXd& l_xu, const Eigen::MatrixXd& l_uu, const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& V_xx_prev) override {
    // Determine internal variables
    const double internal_0 = Fx(0, 1)*V_xx_prev(0, 0);
    const double internal_1 = Fx(0, 0)*V_xx_prev(0, 1);
    const double internal_10 = Fx(0, 0)*V_xx_prev(0, 7);
    const double internal_11 = Fx(0, 0)*Fx(8, 8);
    const double internal_12 = Fx(0, 0)*V_xx_prev(0, 9);
    const double internal_2 = Fx(0, 2)*V_xx_prev(0, 0);
    const double internal_3 = Fx(0, 0)*V_xx_prev(0, 2);
    const double internal_4 = Fx(0, 3)*V_xx_prev(0, 0);
    const double internal_49 = Fx(5, 5)*V_xx_prev(5, 0);
    const double internal_5 = Fx(0, 0)*V_xx_prev(0, 3);
    const double internal_50 = Fx(5, 5)*V_xx_prev(5, 1);
    const double internal_51 = Fx(5, 5)*V_xx_prev(5, 2);
    const double internal_52 = Fx(5, 5)*V_xx_prev(5, 3);
    const double internal_53 = Fx(5, 5)*V_xx_prev(5, 4);
    const double internal_54 = Fx(5, 5)*V_xx_prev(5, 6);
    const double internal_55 = Fx(5, 5)*V_xx_prev(5, 7);
    const double internal_56 = Fx(5, 5)*Fx(8, 8);
    const double internal_57 = Fx(5, 9)*V_xx_prev(5, 5);
    const double internal_58 = Fx(5, 5)*V_xx_prev(5, 9);
    const double internal_59 = Fx(6, 6)*V_xx_prev(6, 0);
    const double internal_6 = Fx(0, 4)*V_xx_prev(0, 0);
    const double internal_60 = Fx(6, 6)*V_xx_prev(6, 1);
    const double internal_61 = Fx(6, 6)*V_xx_prev(6, 2);
    const double internal_62 = Fx(6, 6)*V_xx_prev(6, 3);
    const double internal_63 = Fx(6, 6)*V_xx_prev(6, 4);
    const double internal_64 = Fx(6, 6)*V_xx_prev(6, 5);
    const double internal_65 = Fx(6, 6)*V_xx_prev(6, 7);
    const double internal_66 = Fx(6, 6)*Fx(8, 8);
    const double internal_67 = Fx(6, 9)*V_xx_prev(6, 6);
    const double internal_68 = Fx(6, 6)*V_xx_prev(6, 9);
    const double internal_69 = Fx(7, 7)*V_xx_prev(7, 0);
    const double internal_7 = Fx(0, 0)*V_xx_prev(0, 4);
    const double internal_70 = Fx(7, 7)*V_xx_prev(7, 1);
    const double internal_71 = Fx(7, 7)*V_xx_prev(7, 2);
    const double internal_72 = Fx(7, 7)*V_xx_prev(7, 3);
    const double internal_73 = Fx(7, 7)*V_xx_prev(7, 4);
    const double internal_74 = Fx(7, 7)*V_xx_prev(7, 5);
    const double internal_75 = Fx(7, 7)*V_xx_prev(7, 6);
    const double internal_76 = Fx(7, 7)*Fx(8, 8);
    const double internal_77 = Fx(7, 9)*V_xx_prev(7, 7);
    const double internal_78 = Fx(7, 7)*V_xx_prev(7, 9);
    const double internal_79 = Fx(8, 8)*V_xx_prev(8, 0);
    const double internal_8 = Fx(0, 0)*V_xx_prev(0, 5);
    const double internal_80 = Fx(8, 8)*V_xx_prev(8, 1);
    const double internal_81 = Fx(8, 8)*V_xx_prev(8, 2);
    const double internal_82 = Fx(8, 8)*V_xx_prev(8, 3);
    const double internal_83 = Fx(8, 8)*V_xx_prev(8, 4);
    const double internal_84 = Fx(8, 8)*V_xx_prev(8, 9);
    const double internal_9 = Fx(0, 0)*V_xx_prev(0, 6);
    const double internal_94 = Fu(3, 1)*Fu(4, 0);
    const double internal_95 = Fu(4, 0)*Fu(9, 2);
    const double internal_96 = Fu(3, 1)*Fu(9, 2);
    const double internal_14 = Fx(0, 1)*V_xx_prev(0, 1) + Fx(1, 1)*V_xx_prev(1, 1);
    const double internal_15 = Fx(0, 1)*V_xx_prev(0, 2) + Fx(1, 1)*V_xx_prev(1, 2);
    const double internal_16 = Fx(0, 1)*V_xx_prev(0, 3) + Fx(1, 1)*V_xx_prev(1, 3);
    const double internal_17 = Fx(0, 1)*V_xx_prev(0, 4) + Fx(1, 1)*V_xx_prev(1, 4);
    const double internal_18 = Fx(0, 1)*V_xx_prev(0, 5) + Fx(1, 1)*V_xx_prev(1, 5);
    const double internal_19 = Fx(0, 1)*V_xx_prev(0, 6) + Fx(1, 1)*V_xx_prev(1, 6);
    const double internal_20 = Fx(0, 1)*V_xx_prev(0, 7) + Fx(1, 1)*V_xx_prev(1, 7);
    const double internal_21 = Fx(0, 1)*V_xx_prev(0, 9) + Fx(1, 1)*V_xx_prev(1, 9);
    const double internal_23 = Fx(0, 2)*V_xx_prev(0, 1) + Fx(2, 2)*V_xx_prev(2, 1);
    const double internal_24 = Fx(0, 2)*V_xx_prev(0, 2) + Fx(2, 2)*V_xx_prev(2, 2);
    const double internal_25 = Fx(0, 2)*V_xx_prev(0, 3) + Fx(2, 2)*V_xx_prev(2, 3);
    const double internal_26 = Fx(0, 2)*V_xx_prev(0, 4) + Fx(2, 2)*V_xx_prev(2, 4);
    const double internal_27 = Fx(0, 2)*V_xx_prev(0, 5) + Fx(2, 2)*V_xx_prev(2, 5);
    const double internal_28 = Fx(0, 2)*V_xx_prev(0, 6) + Fx(2, 2)*V_xx_prev(2, 6);
    const double internal_29 = Fx(0, 2)*V_xx_prev(0, 7) + Fx(2, 2)*V_xx_prev(2, 7);
    const double internal_30 = Fx(0, 2)*V_xx_prev(0, 9) + Fx(2, 2)*V_xx_prev(2, 9);
    const double internal_32 = Fx(0, 3)*V_xx_prev(0, 1) + Fx(1, 3)*V_xx_prev(1, 1) + Fx(2, 3)*V_xx_prev(2, 1) + Fx(3, 3)*V_xx_prev(3, 1);
    const double internal_33 = Fx(0, 3)*V_xx_prev(0, 2) + Fx(1, 3)*V_xx_prev(1, 2) + Fx(2, 3)*V_xx_prev(2, 2) + Fx(3, 3)*V_xx_prev(3, 2);
    const double internal_34 = Fx(0, 3)*V_xx_prev(0, 3) + Fx(1, 3)*V_xx_prev(1, 3) + Fx(2, 3)*V_xx_prev(2, 3) + Fx(3, 3)*V_xx_prev(3, 3);
    const double internal_35 = Fx(0, 3)*V_xx_prev(0, 4) + Fx(1, 3)*V_xx_prev(1, 4) + Fx(2, 3)*V_xx_prev(2, 4) + Fx(3, 3)*V_xx_prev(3, 4);
    const double internal_36 = Fx(0, 3)*V_xx_prev(0, 5) + Fx(1, 3)*V_xx_prev(1, 5) + Fx(2, 3)*V_xx_prev(2, 5) + Fx(3, 3)*V_xx_prev(3, 5);
    const double internal_37 = Fx(0, 3)*V_xx_prev(0, 6) + Fx(1, 3)*V_xx_prev(1, 6) + Fx(2, 3)*V_xx_prev(2, 6) + Fx(3, 3)*V_xx_prev(3, 6);
    const double internal_38 = Fx(0, 3)*V_xx_prev(0, 7) + Fx(1, 3)*V_xx_prev(1, 7) + Fx(2, 3)*V_xx_prev(2, 7) + Fx(3, 3)*V_xx_prev(3, 7);
    const double internal_39 = Fx(0, 3)*V_xx_prev(0, 9) + Fx(1, 3)*V_xx_prev(1, 9) + Fx(2, 3)*V_xx_prev(2, 9) + Fx(3, 3)*V_xx_prev(3, 9);
    const double internal_41 = Fx(0, 4)*V_xx_prev(0, 1) + Fx(1, 4)*V_xx_prev(1, 1) + Fx(2, 4)*V_xx_prev(2, 1) + Fx(3, 4)*V_xx_prev(3, 1) + Fx(4, 4)*V_xx_prev(4, 1);
    const double internal_42 = Fx(0, 4)*V_xx_prev(0, 2) + Fx(1, 4)*V_xx_prev(1, 2) + Fx(2, 4)*V_xx_prev(2, 2) + Fx(3, 4)*V_xx_prev(3, 2) + Fx(4, 4)*V_xx_prev(4, 2);
    const double internal_43 = Fx(0, 4)*V_xx_prev(0, 3) + Fx(1, 4)*V_xx_prev(1, 3) + Fx(2, 4)*V_xx_prev(2, 3) + Fx(3, 4)*V_xx_prev(3, 3) + Fx(4, 4)*V_xx_prev(4, 3);
    const double internal_44 = Fx(0, 4)*V_xx_prev(0, 4) + Fx(1, 4)*V_xx_prev(1, 4) + Fx(2, 4)*V_xx_prev(2, 4) + Fx(3, 4)*V_xx_prev(3, 4) + Fx(4, 4)*V_xx_prev(4, 4);
    const double internal_45 = Fx(0, 4)*V_xx_prev(0, 5) + Fx(1, 4)*V_xx_prev(1, 5) + Fx(2, 4)*V_xx_prev(2, 5) + Fx(3, 4)*V_xx_prev(3, 5) + Fx(4, 4)*V_xx_prev(4, 5);
    const double internal_46 = Fx(0, 4)*V_xx_prev(0, 6) + Fx(1, 4)*V_xx_prev(1, 6) + Fx(2, 4)*V_xx_prev(2, 6) + Fx(3, 4)*V_xx_prev(3, 6) + Fx(4, 4)*V_xx_prev(4, 6);
    const double internal_47 = Fx(0, 4)*V_xx_prev(0, 7) + Fx(1, 4)*V_xx_prev(1, 7) + Fx(2, 4)*V_xx_prev(2, 7) + Fx(3, 4)*V_xx_prev(3, 7) + Fx(4, 4)*V_xx_prev(4, 7);
    const double internal_48 = Fx(0, 4)*V_xx_prev(0, 9) + Fx(1, 4)*V_xx_prev(1, 9) + Fx(2, 4)*V_xx_prev(2, 9) + Fx(3, 4)*V_xx_prev(3, 9) + Fx(4, 4)*V_xx_prev(4, 9);
    const double internal_85 = Fx(5, 9)*V_xx_prev(5, 0) + Fx(6, 9)*V_xx_prev(6, 0) + Fx(7, 9)*V_xx_prev(7, 0) + Fx(9, 9)*V_xx_prev(9, 0);
    const double internal_86 = Fx(5, 9)*V_xx_prev(5, 1) + Fx(6, 9)*V_xx_prev(6, 1) + Fx(7, 9)*V_xx_prev(7, 1) + Fx(9, 9)*V_xx_prev(9, 1);
    const double internal_87 = Fx(5, 9)*V_xx_prev(5, 2) + Fx(6, 9)*V_xx_prev(6, 2) + Fx(7, 9)*V_xx_prev(7, 2) + Fx(9, 9)*V_xx_prev(9, 2);
    const double internal_88 = Fx(5, 9)*V_xx_prev(5, 3) + Fx(6, 9)*V_xx_prev(6, 3) + Fx(7, 9)*V_xx_prev(7, 3) + Fx(9, 9)*V_xx_prev(9, 3);
    const double internal_89 = Fx(5, 9)*V_xx_prev(5, 4) + Fx(6, 9)*V_xx_prev(6, 4) + Fx(7, 9)*V_xx_prev(7, 4) + Fx(9, 9)*V_xx_prev(9, 4);
    const double internal_93 = Fx(5, 9)*V_xx_prev(5, 9) + Fx(6, 9)*V_xx_prev(6, 9) + Fx(7, 9)*V_xx_prev(7, 9) + Fx(9, 9)*V_xx_prev(9, 9);
    const double internal_13 = Fx(1, 1)*V_xx_prev(1, 0) + internal_0;
    const double internal_22 = Fx(2, 2)*V_xx_prev(2, 0) + internal_2;
    const double internal_31 = Fx(1, 3)*V_xx_prev(1, 0) + Fx(2, 3)*V_xx_prev(2, 0) + Fx(3, 3)*V_xx_prev(3, 0) + internal_4;
    const double internal_40 = Fx(1, 4)*V_xx_prev(1, 0) + Fx(2, 4)*V_xx_prev(2, 0) + Fx(3, 4)*V_xx_prev(3, 0) + Fx(4, 4)*V_xx_prev(4, 0) + internal_6;
    const double internal_90 = Fx(6, 9)*V_xx_prev(6, 5) + Fx(7, 9)*V_xx_prev(7, 5) + Fx(9, 9)*V_xx_prev(9, 5) + internal_57;
    const double internal_91 = Fx(5, 9)*V_xx_prev(5, 6) + Fx(7, 9)*V_xx_prev(7, 6) + Fx(9, 9)*V_xx_prev(9, 6) + internal_67;
    const double internal_92 = Fx(5, 9)*V_xx_prev(5, 7) + Fx(6, 9)*V_xx_prev(6, 7) + Fx(9, 9)*V_xx_prev(9, 7) + internal_77;

    // Evaluation of Matrix Q_xx_
    Q_xx_(0, 0) = pow(Fx(0, 0), 2)*V_xx_prev(0, 0) + l_xx(0, 0);
    Q_xx_(0, 1) = Fx(0, 0)*internal_0 + Fx(1, 1)*internal_1;
    Q_xx_(0, 2) = Fx(0, 0)*internal_2 + Fx(2, 2)*internal_3;
    Q_xx_(0, 3) = Fx(0, 0)*internal_4 + Fx(1, 3)*internal_1 + Fx(2, 3)*internal_3 + Fx(3, 3)*internal_5;
    Q_xx_(0, 4) = Fx(0, 0)*internal_6 + Fx(1, 4)*internal_1 + Fx(2, 4)*internal_3 + Fx(3, 4)*internal_5 + Fx(4, 4)*internal_7;
    Q_xx_(0, 5) = Fx(5, 5)*internal_8;
    Q_xx_(0, 6) = Fx(6, 6)*internal_9;
    Q_xx_(0, 7) = Fx(7, 7)*internal_10;
    Q_xx_(0, 8) = V_xx_prev(0, 8)*internal_11;
    Q_xx_(0, 9) = Fx(5, 9)*internal_8 + Fx(6, 9)*internal_9 + Fx(7, 9)*internal_10 + Fx(9, 9)*internal_12;
    Q_xx_(1, 0) = Fx(0, 0)*internal_13;
    Q_xx_(1, 1) = Fx(0, 1)*internal_13 + Fx(1, 1)*internal_14 + l_xx(1, 1);
    Q_xx_(1, 2) = Fx(0, 2)*internal_13 + Fx(2, 2)*internal_15 + l_xx(1, 2);
    Q_xx_(1, 3) = Fx(0, 3)*internal_13 + Fx(1, 3)*internal_14 + Fx(2, 3)*internal_15 + Fx(3, 3)*internal_16;
    Q_xx_(1, 4) = Fx(0, 4)*internal_13 + Fx(1, 4)*internal_14 + Fx(2, 4)*internal_15 + Fx(3, 4)*internal_16 + Fx(4, 4)*internal_17;
    Q_xx_(1, 5) = Fx(5, 5)*internal_18;
    Q_xx_(1, 6) = Fx(6, 6)*internal_19;
    Q_xx_(1, 7) = Fx(7, 7)*internal_20;
    Q_xx_(1, 8) = Fx(8, 8)*(Fx(0, 1)*V_xx_prev(0, 8) + Fx(1, 1)*V_xx_prev(1, 8));
    Q_xx_(1, 9) = Fx(5, 9)*internal_18 + Fx(6, 9)*internal_19 + Fx(7, 9)*internal_20 + Fx(9, 9)*internal_21;
    Q_xx_(2, 0) = Fx(0, 0)*internal_22;
    Q_xx_(2, 1) = Fx(0, 1)*internal_22 + Fx(1, 1)*internal_23 + l_xx(2, 1);
    Q_xx_(2, 2) = Fx(0, 2)*internal_22 + Fx(2, 2)*internal_24 + l_xx(2, 2);
    Q_xx_(2, 3) = Fx(0, 3)*internal_22 + Fx(1, 3)*internal_23 + Fx(2, 3)*internal_24 + Fx(3, 3)*internal_25;
    Q_xx_(2, 4) = Fx(0, 4)*internal_22 + Fx(1, 4)*internal_23 + Fx(2, 4)*internal_24 + Fx(3, 4)*internal_25 + Fx(4, 4)*internal_26;
    Q_xx_(2, 5) = Fx(5, 5)*internal_27;
    Q_xx_(2, 6) = Fx(6, 6)*internal_28;
    Q_xx_(2, 7) = Fx(7, 7)*internal_29;
    Q_xx_(2, 8) = Fx(8, 8)*(Fx(0, 2)*V_xx_prev(0, 8) + Fx(2, 2)*V_xx_prev(2, 8));
    Q_xx_(2, 9) = Fx(5, 9)*internal_27 + Fx(6, 9)*internal_28 + Fx(7, 9)*internal_29 + Fx(9, 9)*internal_30;
    Q_xx_(3, 0) = Fx(0, 0)*internal_31;
    Q_xx_(3, 1) = Fx(0, 1)*internal_31 + Fx(1, 1)*internal_32;
    Q_xx_(3, 2) = Fx(0, 2)*internal_31 + Fx(2, 2)*internal_33;
    Q_xx_(3, 3) = Fx(0, 3)*internal_31 + Fx(1, 3)*internal_32 + Fx(2, 3)*internal_33 + Fx(3, 3)*internal_34 + l_xx(3, 3);
    Q_xx_(3, 4) = Fx(0, 4)*internal_31 + Fx(1, 4)*internal_32 + Fx(2, 4)*internal_33 + Fx(3, 4)*internal_34 + Fx(4, 4)*internal_35;
    Q_xx_(3, 5) = Fx(5, 5)*internal_36;
    Q_xx_(3, 6) = Fx(6, 6)*internal_37;
    Q_xx_(3, 7) = Fx(7, 7)*internal_38;
    Q_xx_(3, 8) = Fx(8, 8)*(Fx(0, 3)*V_xx_prev(0, 8) + Fx(1, 3)*V_xx_prev(1, 8) + Fx(2, 3)*V_xx_prev(2, 8) + Fx(3, 3)*V_xx_prev(3, 8));
    Q_xx_(3, 9) = Fx(5, 9)*internal_36 + Fx(6, 9)*internal_37 + Fx(7, 9)*internal_38 + Fx(9, 9)*internal_39;
    Q_xx_(4, 0) = Fx(0, 0)*internal_40;
    Q_xx_(4, 1) = Fx(0, 1)*internal_40 + Fx(1, 1)*internal_41;
    Q_xx_(4, 2) = Fx(0, 2)*internal_40 + Fx(2, 2)*internal_42;
    Q_xx_(4, 3) = Fx(0, 3)*internal_40 + Fx(1, 3)*internal_41 + Fx(2, 3)*internal_42 + Fx(3, 3)*internal_43;
    Q_xx_(4, 4) = Fx(0, 4)*internal_40 + Fx(1, 4)*internal_41 + Fx(2, 4)*internal_42 + Fx(3, 4)*internal_43 + Fx(4, 4)*internal_44 + l_xx(4, 4);
    Q_xx_(4, 5) = Fx(5, 5)*internal_45;
    Q_xx_(4, 6) = Fx(6, 6)*internal_46;
    Q_xx_(4, 7) = Fx(7, 7)*internal_47;
    Q_xx_(4, 8) = Fx(8, 8)*(Fx(0, 4)*V_xx_prev(0, 8) + Fx(1, 4)*V_xx_prev(1, 8) + Fx(2, 4)*V_xx_prev(2, 8) + Fx(3, 4)*V_xx_prev(3, 8) + Fx(4, 4)*V_xx_prev(4, 8));
    Q_xx_(4, 9) = Fx(5, 9)*internal_45 + Fx(6, 9)*internal_46 + Fx(7, 9)*internal_47 + Fx(9, 9)*internal_48;
    Q_xx_(5, 0) = Fx(0, 0)*internal_49;
    Q_xx_(5, 1) = Fx(0, 1)*internal_49 + Fx(1, 1)*internal_50;
    Q_xx_(5, 2) = Fx(0, 2)*internal_49 + Fx(2, 2)*internal_51;
    Q_xx_(5, 3) = Fx(0, 3)*internal_49 + Fx(1, 3)*internal_50 + Fx(2, 3)*internal_51 + Fx(3, 3)*internal_52;
    Q_xx_(5, 4) = Fx(0, 4)*internal_49 + Fx(1, 4)*internal_50 + Fx(2, 4)*internal_51 + Fx(3, 4)*internal_52 + Fx(4, 4)*internal_53;
    Q_xx_(5, 5) = pow(Fx(5, 5), 2)*V_xx_prev(5, 5) + l_xx(5, 5);
    Q_xx_(5, 6) = Fx(6, 6)*internal_54;
    Q_xx_(5, 7) = Fx(7, 7)*internal_55;
    Q_xx_(5, 8) = V_xx_prev(5, 8)*internal_56;
    Q_xx_(5, 9) = Fx(5, 5)*internal_57 + Fx(6, 9)*internal_54 + Fx(7, 9)*internal_55 + Fx(9, 9)*internal_58;
    Q_xx_(6, 0) = Fx(0, 0)*internal_59;
    Q_xx_(6, 1) = Fx(0, 1)*internal_59 + Fx(1, 1)*internal_60;
    Q_xx_(6, 2) = Fx(0, 2)*internal_59 + Fx(2, 2)*internal_61;
    Q_xx_(6, 3) = Fx(0, 3)*internal_59 + Fx(1, 3)*internal_60 + Fx(2, 3)*internal_61 + Fx(3, 3)*internal_62;
    Q_xx_(6, 4) = Fx(0, 4)*internal_59 + Fx(1, 4)*internal_60 + Fx(2, 4)*internal_61 + Fx(3, 4)*internal_62 + Fx(4, 4)*internal_63;
    Q_xx_(6, 5) = Fx(5, 5)*internal_64;
    Q_xx_(6, 6) = pow(Fx(6, 6), 2)*V_xx_prev(6, 6);
    Q_xx_(6, 7) = Fx(7, 7)*internal_65;
    Q_xx_(6, 8) = V_xx_prev(6, 8)*internal_66;
    Q_xx_(6, 9) = Fx(5, 9)*internal_64 + Fx(6, 6)*internal_67 + Fx(7, 9)*internal_65 + Fx(9, 9)*internal_68;
    Q_xx_(7, 0) = Fx(0, 0)*internal_69;
    Q_xx_(7, 1) = Fx(0, 1)*internal_69 + Fx(1, 1)*internal_70;
    Q_xx_(7, 2) = Fx(0, 2)*internal_69 + Fx(2, 2)*internal_71;
    Q_xx_(7, 3) = Fx(0, 3)*internal_69 + Fx(1, 3)*internal_70 + Fx(2, 3)*internal_71 + Fx(3, 3)*internal_72;
    Q_xx_(7, 4) = Fx(0, 4)*internal_69 + Fx(1, 4)*internal_70 + Fx(2, 4)*internal_71 + Fx(3, 4)*internal_72 + Fx(4, 4)*internal_73;
    Q_xx_(7, 5) = Fx(5, 5)*internal_74;
    Q_xx_(7, 6) = Fx(6, 6)*internal_75;
    Q_xx_(7, 7) = pow(Fx(7, 7), 2)*V_xx_prev(7, 7);
    Q_xx_(7, 8) = V_xx_prev(7, 8)*internal_76;
    Q_xx_(7, 9) = Fx(5, 9)*internal_74 + Fx(6, 9)*internal_75 + Fx(7, 7)*internal_77 + Fx(9, 9)*internal_78;
    Q_xx_(8, 0) = V_xx_prev(8, 0)*internal_11;
    Q_xx_(8, 1) = Fx(0, 1)*internal_79 + Fx(1, 1)*internal_80;
    Q_xx_(8, 2) = Fx(0, 2)*internal_79 + Fx(2, 2)*internal_81;
    Q_xx_(8, 3) = Fx(0, 3)*internal_79 + Fx(1, 3)*internal_80 + Fx(2, 3)*internal_81 + Fx(3, 3)*internal_82;
    Q_xx_(8, 4) = Fx(0, 4)*internal_79 + Fx(1, 4)*internal_80 + Fx(2, 4)*internal_81 + Fx(3, 4)*internal_82 + Fx(4, 4)*internal_83;
    Q_xx_(8, 5) = V_xx_prev(8, 5)*internal_56;
    Q_xx_(8, 6) = V_xx_prev(8, 6)*internal_66;
    Q_xx_(8, 7) = V_xx_prev(8, 7)*internal_76;
    Q_xx_(8, 8) = pow(Fx(8, 8), 2)*V_xx_prev(8, 8);
    Q_xx_(8, 9) = Fx(5, 9)*Fx(8, 8)*V_xx_prev(8, 5) + Fx(6, 9)*Fx(8, 8)*V_xx_prev(8, 6) + Fx(7, 9)*Fx(8, 8)*V_xx_prev(8, 7) + Fx(9, 9)*internal_84;
    Q_xx_(9, 0) = Fx(0, 0)*internal_85;
    Q_xx_(9, 1) = Fx(0, 1)*internal_85 + Fx(1, 1)*internal_86;
    Q_xx_(9, 2) = Fx(0, 2)*internal_85 + Fx(2, 2)*internal_87;
    Q_xx_(9, 3) = Fx(0, 3)*internal_85 + Fx(1, 3)*internal_86 + Fx(2, 3)*internal_87 + Fx(3, 3)*internal_88;
    Q_xx_(9, 4) = Fx(0, 4)*internal_85 + Fx(1, 4)*internal_86 + Fx(2, 4)*internal_87 + Fx(3, 4)*internal_88 + Fx(4, 4)*internal_89;
    Q_xx_(9, 5) = Fx(5, 5)*internal_90;
    Q_xx_(9, 6) = Fx(6, 6)*internal_91;
    Q_xx_(9, 7) = Fx(7, 7)*internal_92;
    Q_xx_(9, 8) = Fx(8, 8)*(Fx(5, 9)*V_xx_prev(5, 8) + Fx(6, 9)*V_xx_prev(6, 8) + Fx(7, 9)*V_xx_prev(7, 8) + Fx(9, 9)*V_xx_prev(9, 8));
    Q_xx_(9, 9) = Fx(5, 9)*internal_90 + Fx(6, 9)*internal_91 + Fx(7, 9)*internal_92 + Fx(9, 9)*internal_93 + l_xx(9, 9);

    // Evaluation of Matrix Q_xu_
    Q_xu_(0, 0) = Fu(4, 0)*internal_7;
    Q_xu_(0, 1) = Fu(3, 1)*internal_5;
    Q_xu_(0, 2) = Fu(9, 2)*internal_12;
    Q_xu_(1, 0) = Fu(4, 0)*internal_17;
    Q_xu_(1, 1) = Fu(3, 1)*internal_16;
    Q_xu_(1, 2) = Fu(9, 2)*internal_21;
    Q_xu_(2, 0) = Fu(4, 0)*internal_26;
    Q_xu_(2, 1) = Fu(3, 1)*internal_25;
    Q_xu_(2, 2) = Fu(9, 2)*internal_30;
    Q_xu_(3, 0) = Fu(4, 0)*internal_35;
    Q_xu_(3, 1) = Fu(3, 1)*internal_34;
    Q_xu_(3, 2) = Fu(9, 2)*internal_39;
    Q_xu_(4, 0) = Fu(4, 0)*internal_44;
    Q_xu_(4, 1) = Fu(3, 1)*internal_43;
    Q_xu_(4, 2) = Fu(9, 2)*internal_48;
    Q_xu_(5, 0) = Fu(4, 0)*internal_53;
    Q_xu_(5, 1) = Fu(3, 1)*internal_52;
    Q_xu_(5, 2) = Fu(9, 2)*internal_58;
    Q_xu_(6, 0) = Fu(4, 0)*internal_63;
    Q_xu_(6, 1) = Fu(3, 1)*internal_62;
    Q_xu_(6, 2) = Fu(9, 2)*internal_68;
    Q_xu_(7, 0) = Fu(4, 0)*internal_73;
    Q_xu_(7, 1) = Fu(3, 1)*internal_72;
    Q_xu_(7, 2) = Fu(9, 2)*internal_78;
    Q_xu_(8, 0) = Fu(4, 0)*internal_83;
    Q_xu_(8, 1) = Fu(3, 1)*internal_82;
    Q_xu_(8, 2) = Fu(9, 2)*internal_84;
    Q_xu_(9, 0) = Fu(4, 0)*internal_89;
    Q_xu_(9, 1) = Fu(3, 1)*internal_88;
    Q_xu_(9, 2) = Fu(9, 2)*internal_93;

    // Evaluation of Matrix Q_uu_
    Q_uu_(0, 0) = pow(Fu(4, 0), 2)*V_xx_prev(4, 4) + Sigma_(8)*pow(g_u(8, 0), 2) + Sigma_(9)*pow(g_u(9, 0), 2) + l_uu(0, 0);
    Q_uu_(0, 1) = V_xx_prev(4, 3)*internal_94;
    Q_uu_(0, 2) = V_xx_prev(4, 9)*internal_95;
    Q_uu_(1, 0) = V_xx_prev(3, 4)*internal_94;
    Q_uu_(1, 1) = pow(Fu(3, 1), 2)*V_xx_prev(3, 3) + Sigma_(6)*pow(g_u(6, 1), 2) + Sigma_(7)*pow(g_u(7, 1), 2) + l_uu(1, 1);
    Q_uu_(1, 2) = V_xx_prev(3, 9)*internal_96;
    Q_uu_(2, 0) = V_xx_prev(9, 4)*internal_95;
    Q_uu_(2, 1) = V_xx_prev(9, 3)*internal_96;
    Q_uu_(2, 2) = pow(Fu(9, 2), 2)*V_xx_prev(9, 9) + Sigma_(10)*pow(g_u(10, 2), 2) + Sigma_(11)*pow(g_u(11, 2), 2) + l_uu(2, 2);
    Q_uu_(3, 3) = Sigma_(0)*pow(g_u(0, 3), 2);
    Q_uu_(4, 4) = Sigma_(1)*pow(g_u(1, 4), 2);
    Q_uu_(5, 5) = Sigma_(2)*pow(g_u(2, 5), 2);
    Q_uu_(6, 6) = Sigma_(3)*pow(g_u(3, 6), 2);
    Q_uu_(7, 7) = Sigma_(4)*pow(g_u(4, 7), 2);
    Q_uu_(8, 8) = Sigma_(5)*pow(g_u(5, 8), 2);

  }

  virtual void updateQG(const Eigen::VectorXd& sl_x, const Eigen::VectorXd& sl_u, const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::VectorXd& r_f, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& V_xx_prev, const Eigen::VectorXd& V_x_prev) override {
    // Determine internal variables
    const double internal_0 = V_x_prev(0) + V_xx_prev(0, 0)*r_f(0) + V_xx_prev(0, 1)*r_f(1) + V_xx_prev(0, 2)*r_f(2) + V_xx_prev(0, 3)*r_f(3) + V_xx_prev(0, 4)*r_f(4) + V_xx_prev(0, 5)*r_f(5) + V_xx_prev(0, 6)*r_f(6) + V_xx_prev(0, 7)*r_f(7) + V_xx_prev(0, 8)*r_f(8) + V_xx_prev(0, 9)*r_f(9);
    const double internal_1 = V_x_prev(1) + V_xx_prev(1, 0)*r_f(0) + V_xx_prev(1, 1)*r_f(1) + V_xx_prev(1, 2)*r_f(2) + V_xx_prev(1, 3)*r_f(3) + V_xx_prev(1, 4)*r_f(4) + V_xx_prev(1, 5)*r_f(5) + V_xx_prev(1, 6)*r_f(6) + V_xx_prev(1, 7)*r_f(7) + V_xx_prev(1, 8)*r_f(8) + V_xx_prev(1, 9)*r_f(9);
    const double internal_2 = V_x_prev(2) + V_xx_prev(2, 0)*r_f(0) + V_xx_prev(2, 1)*r_f(1) + V_xx_prev(2, 2)*r_f(2) + V_xx_prev(2, 3)*r_f(3) + V_xx_prev(2, 4)*r_f(4) + V_xx_prev(2, 5)*r_f(5) + V_xx_prev(2, 6)*r_f(6) + V_xx_prev(2, 7)*r_f(7) + V_xx_prev(2, 8)*r_f(8) + V_xx_prev(2, 9)*r_f(9);
    const double internal_3 = V_x_prev(3) + V_xx_prev(3, 0)*r_f(0) + V_xx_prev(3, 1)*r_f(1) + V_xx_prev(3, 2)*r_f(2) + V_xx_prev(3, 3)*r_f(3) + V_xx_prev(3, 4)*r_f(4) + V_xx_prev(3, 5)*r_f(5) + V_xx_prev(3, 6)*r_f(6) + V_xx_prev(3, 7)*r_f(7) + V_xx_prev(3, 8)*r_f(8) + V_xx_prev(3, 9)*r_f(9);
    const double internal_4 = V_x_prev(4) + V_xx_prev(4, 0)*r_f(0) + V_xx_prev(4, 1)*r_f(1) + V_xx_prev(4, 2)*r_f(2) + V_xx_prev(4, 3)*r_f(3) + V_xx_prev(4, 4)*r_f(4) + V_xx_prev(4, 5)*r_f(5) + V_xx_prev(4, 6)*r_f(6) + V_xx_prev(4, 7)*r_f(7) + V_xx_prev(4, 8)*r_f(8) + V_xx_prev(4, 9)*r_f(9);
    const double internal_5 = V_x_prev(5) + V_xx_prev(5, 0)*r_f(0) + V_xx_prev(5, 1)*r_f(1) + V_xx_prev(5, 2)*r_f(2) + V_xx_prev(5, 3)*r_f(3) + V_xx_prev(5, 4)*r_f(4) + V_xx_prev(5, 5)*r_f(5) + V_xx_prev(5, 6)*r_f(6) + V_xx_prev(5, 7)*r_f(7) + V_xx_prev(5, 8)*r_f(8) + V_xx_prev(5, 9)*r_f(9);
    const double internal_6 = V_x_prev(6) + V_xx_prev(6, 0)*r_f(0) + V_xx_prev(6, 1)*r_f(1) + V_xx_prev(6, 2)*r_f(2) + V_xx_prev(6, 3)*r_f(3) + V_xx_prev(6, 4)*r_f(4) + V_xx_prev(6, 5)*r_f(5) + V_xx_prev(6, 6)*r_f(6) + V_xx_prev(6, 7)*r_f(7) + V_xx_prev(6, 8)*r_f(8) + V_xx_prev(6, 9)*r_f(9);
    const double internal_7 = V_x_prev(7) + V_xx_prev(7, 0)*r_f(0) + V_xx_prev(7, 1)*r_f(1) + V_xx_prev(7, 2)*r_f(2) + V_xx_prev(7, 3)*r_f(3) + V_xx_prev(7, 4)*r_f(4) + V_xx_prev(7, 5)*r_f(5) + V_xx_prev(7, 6)*r_f(6) + V_xx_prev(7, 7)*r_f(7) + V_xx_prev(7, 8)*r_f(8) + V_xx_prev(7, 9)*r_f(9);
    const double internal_8 = V_x_prev(9) + V_xx_prev(9, 0)*r_f(0) + V_xx_prev(9, 1)*r_f(1) + V_xx_prev(9, 2)*r_f(2) + V_xx_prev(9, 3)*r_f(3) + V_xx_prev(9, 4)*r_f(4) + V_xx_prev(9, 5)*r_f(5) + V_xx_prev(9, 6)*r_f(6) + V_xx_prev(9, 7)*r_f(7) + V_xx_prev(9, 8)*r_f(8) + V_xx_prev(9, 9)*r_f(9);

    // Evaluation of Vector Q_x_
    Q_x_(0) = Fx(0, 0)*internal_0 + sl_x(0);
    Q_x_(1) = Fx(0, 1)*internal_0 + Fx(1, 1)*internal_1 + sl_x(1);
    Q_x_(2) = Fx(0, 2)*internal_0 + Fx(2, 2)*internal_2 + sl_x(2);
    Q_x_(3) = Fx(0, 3)*internal_0 + Fx(1, 3)*internal_1 + Fx(2, 3)*internal_2 + Fx(3, 3)*internal_3 + sl_x(3);
    Q_x_(4) = Fx(0, 4)*internal_0 + Fx(1, 4)*internal_1 + Fx(2, 4)*internal_2 + Fx(3, 4)*internal_3 + Fx(4, 4)*internal_4 + sl_x(4);
    Q_x_(5) = Fx(5, 5)*internal_5 + sl_x(5);
    Q_x_(6) = Fx(6, 6)*internal_6 + sl_x(6);
    Q_x_(7) = Fx(7, 7)*internal_7 + sl_x(7);
    Q_x_(8) = Fx(8, 8)*(V_x_prev(8) + V_xx_prev(8, 0)*r_f(0) + V_xx_prev(8, 1)*r_f(1) + V_xx_prev(8, 2)*r_f(2) + V_xx_prev(8, 3)*r_f(3) + V_xx_prev(8, 4)*r_f(4) + V_xx_prev(8, 5)*r_f(5) + V_xx_prev(8, 6)*r_f(6) + V_xx_prev(8, 7)*r_f(7) + V_xx_prev(8, 8)*r_f(8) + V_xx_prev(8, 9)*r_f(9)) + sl_x(8);
    Q_x_(9) = Fx(5, 9)*internal_5 + Fx(6, 9)*internal_6 + Fx(7, 9)*internal_7 + Fx(9, 9)*internal_8 + sl_x(9);

    // Evaluation of Vector Q_u_
    Q_u_(0) = Fu(4, 0)*internal_4 + V_(8)*g_u(8, 0) + V_(9)*g_u(9, 0) + sl_u(0);
    Q_u_(1) = Fu(3, 1)*internal_3 + V_(6)*g_u(6, 1) + V_(7)*g_u(7, 1) + sl_u(1);
    Q_u_(2) = Fu(9, 2)*internal_8 + V_(10)*g_u(10, 2) + V_(11)*g_u(11, 2) + sl_u(2);
    Q_u_(3) = V_(0)*g_u(0, 3) + sl_u(3);
    Q_u_(4) = V_(1)*g_u(1, 4) + sl_u(4);
    Q_u_(5) = V_(2)*g_u(2, 5) + sl_u(5);
    Q_u_(6) = V_(3)*g_u(3, 6) + sl_u(6);
    Q_u_(7) = V_(4)*g_u(4, 7) + sl_u(7);
    Q_u_(8) = V_(5)*g_u(5, 8) + sl_u(8);

  }

  virtual void updateQuuInv() override {
    // Determine internal variables
    const double internal_0 = 1.0/Q_uu_(0, 0);
    const double internal_3 = Q_uu_(0, 1)*internal_0;
    const double internal_5 = Q_uu_(0, 2)*internal_0;
    const double internal_1 = -pow(Q_uu_(0, 1), 2)*internal_0 + Q_uu_(1, 1);
    const double internal_2 = 1.0/internal_1;
    const double internal_6 = -Q_uu_(0, 1)*internal_5 + Q_uu_(1, 2);
    const double internal_4 = internal_2*internal_3;
    const double internal_7 = pow(internal_6, 2);
    const double internal_9 = internal_4*internal_6;
    const double internal_8 = 1.0/(-pow(Q_uu_(0, 2), 2)*internal_0 + Q_uu_(2, 2) - internal_2*internal_7);
    const double internal_10 = internal_8*(-internal_5 + internal_9);
    const double internal_13 = -internal_2*internal_6*internal_8;
    const double internal_12 = internal_2 + internal_7*internal_8/pow(internal_1, 2);
    const double internal_11 = -internal_10*internal_2*internal_6 - internal_4;

    // Evaluation of Matrix Q_uu_inv_
    Q_uu_inv_(0, 0) = internal_0 - internal_10*internal_5 - internal_11*internal_3;
    Q_uu_inv_(0, 1) = Q_uu_(0, 2)*internal_0*internal_2*internal_6*internal_8 - internal_12*internal_3;
    Q_uu_inv_(0, 2) = -internal_5*internal_8 + internal_8*internal_9;
    Q_uu_inv_(1, 0) = internal_11;
    Q_uu_inv_(1, 1) = internal_12;
    Q_uu_inv_(1, 2) = internal_13;
    Q_uu_inv_(2, 0) = internal_10;
    Q_uu_inv_(2, 1) = internal_13;
    Q_uu_inv_(2, 2) = internal_8;
    Q_uu_inv_(3, 3) = 1.0/Q_uu_(3, 3);
    Q_uu_inv_(4, 4) = 1.0/Q_uu_(4, 4);
    Q_uu_inv_(5, 5) = 1.0/Q_uu_(5, 5);
    Q_uu_inv_(6, 6) = 1.0/Q_uu_(6, 6);
    Q_uu_inv_(7, 7) = 1.0/Q_uu_(7, 7);
    Q_uu_inv_(8, 8) = 1.0/Q_uu_(8, 8);

  }

  virtual void updateKVU(Eigen::MatrixXd& k_ux, Eigen::VectorXd& v_u) override {
    // Evaluation of Matrix k_ux
    k_ux(0, 0) = -Q_uu_inv_(0, 0)*Q_xu_(0, 0) - Q_uu_inv_(0, 1)*Q_xu_(0, 1) - Q_uu_inv_(0, 2)*Q_xu_(0, 2);
    k_ux(0, 1) = -Q_uu_inv_(0, 0)*Q_xu_(1, 0) - Q_uu_inv_(0, 1)*Q_xu_(1, 1) - Q_uu_inv_(0, 2)*Q_xu_(1, 2);
    k_ux(0, 2) = -Q_uu_inv_(0, 0)*Q_xu_(2, 0) - Q_uu_inv_(0, 1)*Q_xu_(2, 1) - Q_uu_inv_(0, 2)*Q_xu_(2, 2);
    k_ux(0, 3) = -Q_uu_inv_(0, 0)*Q_xu_(3, 0) - Q_uu_inv_(0, 1)*Q_xu_(3, 1) - Q_uu_inv_(0, 2)*Q_xu_(3, 2);
    k_ux(0, 4) = -Q_uu_inv_(0, 0)*Q_xu_(4, 0) - Q_uu_inv_(0, 1)*Q_xu_(4, 1) - Q_uu_inv_(0, 2)*Q_xu_(4, 2);
    k_ux(0, 5) = -Q_uu_inv_(0, 0)*Q_xu_(5, 0) - Q_uu_inv_(0, 1)*Q_xu_(5, 1) - Q_uu_inv_(0, 2)*Q_xu_(5, 2);
    k_ux(0, 6) = -Q_uu_inv_(0, 0)*Q_xu_(6, 0) - Q_uu_inv_(0, 1)*Q_xu_(6, 1) - Q_uu_inv_(0, 2)*Q_xu_(6, 2);
    k_ux(0, 7) = -Q_uu_inv_(0, 0)*Q_xu_(7, 0) - Q_uu_inv_(0, 1)*Q_xu_(7, 1) - Q_uu_inv_(0, 2)*Q_xu_(7, 2);
    k_ux(0, 8) = -Q_uu_inv_(0, 0)*Q_xu_(8, 0) - Q_uu_inv_(0, 1)*Q_xu_(8, 1) - Q_uu_inv_(0, 2)*Q_xu_(8, 2);
    k_ux(0, 9) = -Q_uu_inv_(0, 0)*Q_xu_(9, 0) - Q_uu_inv_(0, 1)*Q_xu_(9, 1) - Q_uu_inv_(0, 2)*Q_xu_(9, 2);
    k_ux(1, 0) = -Q_uu_inv_(0, 1)*Q_xu_(0, 0) - Q_uu_inv_(1, 1)*Q_xu_(0, 1) - Q_uu_inv_(1, 2)*Q_xu_(0, 2);
    k_ux(1, 1) = -Q_uu_inv_(0, 1)*Q_xu_(1, 0) - Q_uu_inv_(1, 1)*Q_xu_(1, 1) - Q_uu_inv_(1, 2)*Q_xu_(1, 2);
    k_ux(1, 2) = -Q_uu_inv_(0, 1)*Q_xu_(2, 0) - Q_uu_inv_(1, 1)*Q_xu_(2, 1) - Q_uu_inv_(1, 2)*Q_xu_(2, 2);
    k_ux(1, 3) = -Q_uu_inv_(0, 1)*Q_xu_(3, 0) - Q_uu_inv_(1, 1)*Q_xu_(3, 1) - Q_uu_inv_(1, 2)*Q_xu_(3, 2);
    k_ux(1, 4) = -Q_uu_inv_(0, 1)*Q_xu_(4, 0) - Q_uu_inv_(1, 1)*Q_xu_(4, 1) - Q_uu_inv_(1, 2)*Q_xu_(4, 2);
    k_ux(1, 5) = -Q_uu_inv_(0, 1)*Q_xu_(5, 0) - Q_uu_inv_(1, 1)*Q_xu_(5, 1) - Q_uu_inv_(1, 2)*Q_xu_(5, 2);
    k_ux(1, 6) = -Q_uu_inv_(0, 1)*Q_xu_(6, 0) - Q_uu_inv_(1, 1)*Q_xu_(6, 1) - Q_uu_inv_(1, 2)*Q_xu_(6, 2);
    k_ux(1, 7) = -Q_uu_inv_(0, 1)*Q_xu_(7, 0) - Q_uu_inv_(1, 1)*Q_xu_(7, 1) - Q_uu_inv_(1, 2)*Q_xu_(7, 2);
    k_ux(1, 8) = -Q_uu_inv_(0, 1)*Q_xu_(8, 0) - Q_uu_inv_(1, 1)*Q_xu_(8, 1) - Q_uu_inv_(1, 2)*Q_xu_(8, 2);
    k_ux(1, 9) = -Q_uu_inv_(0, 1)*Q_xu_(9, 0) - Q_uu_inv_(1, 1)*Q_xu_(9, 1) - Q_uu_inv_(1, 2)*Q_xu_(9, 2);
    k_ux(2, 0) = -Q_uu_inv_(0, 2)*Q_xu_(0, 0) - Q_uu_inv_(1, 2)*Q_xu_(0, 1) - Q_uu_inv_(2, 2)*Q_xu_(0, 2);
    k_ux(2, 1) = -Q_uu_inv_(0, 2)*Q_xu_(1, 0) - Q_uu_inv_(1, 2)*Q_xu_(1, 1) - Q_uu_inv_(2, 2)*Q_xu_(1, 2);
    k_ux(2, 2) = -Q_uu_inv_(0, 2)*Q_xu_(2, 0) - Q_uu_inv_(1, 2)*Q_xu_(2, 1) - Q_uu_inv_(2, 2)*Q_xu_(2, 2);
    k_ux(2, 3) = -Q_uu_inv_(0, 2)*Q_xu_(3, 0) - Q_uu_inv_(1, 2)*Q_xu_(3, 1) - Q_uu_inv_(2, 2)*Q_xu_(3, 2);
    k_ux(2, 4) = -Q_uu_inv_(0, 2)*Q_xu_(4, 0) - Q_uu_inv_(1, 2)*Q_xu_(4, 1) - Q_uu_inv_(2, 2)*Q_xu_(4, 2);
    k_ux(2, 5) = -Q_uu_inv_(0, 2)*Q_xu_(5, 0) - Q_uu_inv_(1, 2)*Q_xu_(5, 1) - Q_uu_inv_(2, 2)*Q_xu_(5, 2);
    k_ux(2, 6) = -Q_uu_inv_(0, 2)*Q_xu_(6, 0) - Q_uu_inv_(1, 2)*Q_xu_(6, 1) - Q_uu_inv_(2, 2)*Q_xu_(6, 2);
    k_ux(2, 7) = -Q_uu_inv_(0, 2)*Q_xu_(7, 0) - Q_uu_inv_(1, 2)*Q_xu_(7, 1) - Q_uu_inv_(2, 2)*Q_xu_(7, 2);
    k_ux(2, 8) = -Q_uu_inv_(0, 2)*Q_xu_(8, 0) - Q_uu_inv_(1, 2)*Q_xu_(8, 1) - Q_uu_inv_(2, 2)*Q_xu_(8, 2);
    k_ux(2, 9) = -Q_uu_inv_(0, 2)*Q_xu_(9, 0) - Q_uu_inv_(1, 2)*Q_xu_(9, 1) - Q_uu_inv_(2, 2)*Q_xu_(9, 2);

    // Evaluation of Vector v_u
    v_u(0) = -Q_u_(0)*Q_uu_inv_(0, 0) - Q_u_(1)*Q_uu_inv_(0, 1) - Q_u_(2)*Q_uu_inv_(0, 2);
    v_u(1) = -Q_u_(0)*Q_uu_inv_(0, 1) - Q_u_(1)*Q_uu_inv_(1, 1) - Q_u_(2)*Q_uu_inv_(1, 2);
    v_u(2) = -Q_u_(0)*Q_uu_inv_(0, 2) - Q_u_(1)*Q_uu_inv_(1, 2) - Q_u_(2)*Q_uu_inv_(2, 2);
    v_u(3) = -Q_u_(3)*Q_uu_inv_(3, 3);
    v_u(4) = -Q_u_(4)*Q_uu_inv_(4, 4);
    v_u(5) = -Q_u_(5)*Q_uu_inv_(5, 5);
    v_u(6) = -Q_u_(6)*Q_uu_inv_(6, 6);
    v_u(7) = -Q_u_(7)*Q_uu_inv_(7, 7);
    v_u(8) = -Q_u_(8)*Q_uu_inv_(8, 8);

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
    V_xx(0, 0) = Q_xu_(0, 0)*k_ux(0, 0) + Q_xu_(0, 1)*k_ux(1, 0) + Q_xu_(0, 2)*k_ux(2, 0) + Q_xx_(0, 0);
    V_xx(0, 1) = Q_xu_(0, 0)*k_ux(0, 1) + Q_xu_(0, 1)*k_ux(1, 1) + Q_xu_(0, 2)*k_ux(2, 1) + Q_xx_(0, 1);
    V_xx(0, 2) = Q_xu_(0, 0)*k_ux(0, 2) + Q_xu_(0, 1)*k_ux(1, 2) + Q_xu_(0, 2)*k_ux(2, 2) + Q_xx_(0, 2);
    V_xx(0, 3) = Q_xu_(0, 0)*k_ux(0, 3) + Q_xu_(0, 1)*k_ux(1, 3) + Q_xu_(0, 2)*k_ux(2, 3) + Q_xx_(0, 3);
    V_xx(0, 4) = Q_xu_(0, 0)*k_ux(0, 4) + Q_xu_(0, 1)*k_ux(1, 4) + Q_xu_(0, 2)*k_ux(2, 4) + Q_xx_(0, 4);
    V_xx(0, 5) = Q_xu_(0, 0)*k_ux(0, 5) + Q_xu_(0, 1)*k_ux(1, 5) + Q_xu_(0, 2)*k_ux(2, 5) + Q_xx_(0, 5);
    V_xx(0, 6) = Q_xu_(0, 0)*k_ux(0, 6) + Q_xu_(0, 1)*k_ux(1, 6) + Q_xu_(0, 2)*k_ux(2, 6) + Q_xx_(0, 6);
    V_xx(0, 7) = Q_xu_(0, 0)*k_ux(0, 7) + Q_xu_(0, 1)*k_ux(1, 7) + Q_xu_(0, 2)*k_ux(2, 7) + Q_xx_(0, 7);
    V_xx(0, 8) = Q_xu_(0, 0)*k_ux(0, 8) + Q_xu_(0, 1)*k_ux(1, 8) + Q_xu_(0, 2)*k_ux(2, 8) + Q_xx_(0, 8);
    V_xx(0, 9) = Q_xu_(0, 0)*k_ux(0, 9) + Q_xu_(0, 1)*k_ux(1, 9) + Q_xu_(0, 2)*k_ux(2, 9) + Q_xx_(0, 9);
    V_xx(1, 0) = Q_xu_(1, 0)*k_ux(0, 0) + Q_xu_(1, 1)*k_ux(1, 0) + Q_xu_(1, 2)*k_ux(2, 0) + Q_xx_(0, 1);
    V_xx(1, 1) = Q_xu_(1, 0)*k_ux(0, 1) + Q_xu_(1, 1)*k_ux(1, 1) + Q_xu_(1, 2)*k_ux(2, 1) + Q_xx_(1, 1);
    V_xx(1, 2) = Q_xu_(1, 0)*k_ux(0, 2) + Q_xu_(1, 1)*k_ux(1, 2) + Q_xu_(1, 2)*k_ux(2, 2) + Q_xx_(1, 2);
    V_xx(1, 3) = Q_xu_(1, 0)*k_ux(0, 3) + Q_xu_(1, 1)*k_ux(1, 3) + Q_xu_(1, 2)*k_ux(2, 3) + Q_xx_(1, 3);
    V_xx(1, 4) = Q_xu_(1, 0)*k_ux(0, 4) + Q_xu_(1, 1)*k_ux(1, 4) + Q_xu_(1, 2)*k_ux(2, 4) + Q_xx_(1, 4);
    V_xx(1, 5) = Q_xu_(1, 0)*k_ux(0, 5) + Q_xu_(1, 1)*k_ux(1, 5) + Q_xu_(1, 2)*k_ux(2, 5) + Q_xx_(1, 5);
    V_xx(1, 6) = Q_xu_(1, 0)*k_ux(0, 6) + Q_xu_(1, 1)*k_ux(1, 6) + Q_xu_(1, 2)*k_ux(2, 6) + Q_xx_(1, 6);
    V_xx(1, 7) = Q_xu_(1, 0)*k_ux(0, 7) + Q_xu_(1, 1)*k_ux(1, 7) + Q_xu_(1, 2)*k_ux(2, 7) + Q_xx_(1, 7);
    V_xx(1, 8) = Q_xu_(1, 0)*k_ux(0, 8) + Q_xu_(1, 1)*k_ux(1, 8) + Q_xu_(1, 2)*k_ux(2, 8) + Q_xx_(1, 8);
    V_xx(1, 9) = Q_xu_(1, 0)*k_ux(0, 9) + Q_xu_(1, 1)*k_ux(1, 9) + Q_xu_(1, 2)*k_ux(2, 9) + Q_xx_(1, 9);
    V_xx(2, 0) = Q_xu_(2, 0)*k_ux(0, 0) + Q_xu_(2, 1)*k_ux(1, 0) + Q_xu_(2, 2)*k_ux(2, 0) + Q_xx_(0, 2);
    V_xx(2, 1) = Q_xu_(2, 0)*k_ux(0, 1) + Q_xu_(2, 1)*k_ux(1, 1) + Q_xu_(2, 2)*k_ux(2, 1) + Q_xx_(1, 2);
    V_xx(2, 2) = Q_xu_(2, 0)*k_ux(0, 2) + Q_xu_(2, 1)*k_ux(1, 2) + Q_xu_(2, 2)*k_ux(2, 2) + Q_xx_(2, 2);
    V_xx(2, 3) = Q_xu_(2, 0)*k_ux(0, 3) + Q_xu_(2, 1)*k_ux(1, 3) + Q_xu_(2, 2)*k_ux(2, 3) + Q_xx_(2, 3);
    V_xx(2, 4) = Q_xu_(2, 0)*k_ux(0, 4) + Q_xu_(2, 1)*k_ux(1, 4) + Q_xu_(2, 2)*k_ux(2, 4) + Q_xx_(2, 4);
    V_xx(2, 5) = Q_xu_(2, 0)*k_ux(0, 5) + Q_xu_(2, 1)*k_ux(1, 5) + Q_xu_(2, 2)*k_ux(2, 5) + Q_xx_(2, 5);
    V_xx(2, 6) = Q_xu_(2, 0)*k_ux(0, 6) + Q_xu_(2, 1)*k_ux(1, 6) + Q_xu_(2, 2)*k_ux(2, 6) + Q_xx_(2, 6);
    V_xx(2, 7) = Q_xu_(2, 0)*k_ux(0, 7) + Q_xu_(2, 1)*k_ux(1, 7) + Q_xu_(2, 2)*k_ux(2, 7) + Q_xx_(2, 7);
    V_xx(2, 8) = Q_xu_(2, 0)*k_ux(0, 8) + Q_xu_(2, 1)*k_ux(1, 8) + Q_xu_(2, 2)*k_ux(2, 8) + Q_xx_(2, 8);
    V_xx(2, 9) = Q_xu_(2, 0)*k_ux(0, 9) + Q_xu_(2, 1)*k_ux(1, 9) + Q_xu_(2, 2)*k_ux(2, 9) + Q_xx_(2, 9);
    V_xx(3, 0) = Q_xu_(3, 0)*k_ux(0, 0) + Q_xu_(3, 1)*k_ux(1, 0) + Q_xu_(3, 2)*k_ux(2, 0) + Q_xx_(0, 3);
    V_xx(3, 1) = Q_xu_(3, 0)*k_ux(0, 1) + Q_xu_(3, 1)*k_ux(1, 1) + Q_xu_(3, 2)*k_ux(2, 1) + Q_xx_(1, 3);
    V_xx(3, 2) = Q_xu_(3, 0)*k_ux(0, 2) + Q_xu_(3, 1)*k_ux(1, 2) + Q_xu_(3, 2)*k_ux(2, 2) + Q_xx_(2, 3);
    V_xx(3, 3) = Q_xu_(3, 0)*k_ux(0, 3) + Q_xu_(3, 1)*k_ux(1, 3) + Q_xu_(3, 2)*k_ux(2, 3) + Q_xx_(3, 3);
    V_xx(3, 4) = Q_xu_(3, 0)*k_ux(0, 4) + Q_xu_(3, 1)*k_ux(1, 4) + Q_xu_(3, 2)*k_ux(2, 4) + Q_xx_(3, 4);
    V_xx(3, 5) = Q_xu_(3, 0)*k_ux(0, 5) + Q_xu_(3, 1)*k_ux(1, 5) + Q_xu_(3, 2)*k_ux(2, 5) + Q_xx_(3, 5);
    V_xx(3, 6) = Q_xu_(3, 0)*k_ux(0, 6) + Q_xu_(3, 1)*k_ux(1, 6) + Q_xu_(3, 2)*k_ux(2, 6) + Q_xx_(3, 6);
    V_xx(3, 7) = Q_xu_(3, 0)*k_ux(0, 7) + Q_xu_(3, 1)*k_ux(1, 7) + Q_xu_(3, 2)*k_ux(2, 7) + Q_xx_(3, 7);
    V_xx(3, 8) = Q_xu_(3, 0)*k_ux(0, 8) + Q_xu_(3, 1)*k_ux(1, 8) + Q_xu_(3, 2)*k_ux(2, 8) + Q_xx_(3, 8);
    V_xx(3, 9) = Q_xu_(3, 0)*k_ux(0, 9) + Q_xu_(3, 1)*k_ux(1, 9) + Q_xu_(3, 2)*k_ux(2, 9) + Q_xx_(3, 9);
    V_xx(4, 0) = Q_xu_(4, 0)*k_ux(0, 0) + Q_xu_(4, 1)*k_ux(1, 0) + Q_xu_(4, 2)*k_ux(2, 0) + Q_xx_(0, 4);
    V_xx(4, 1) = Q_xu_(4, 0)*k_ux(0, 1) + Q_xu_(4, 1)*k_ux(1, 1) + Q_xu_(4, 2)*k_ux(2, 1) + Q_xx_(1, 4);
    V_xx(4, 2) = Q_xu_(4, 0)*k_ux(0, 2) + Q_xu_(4, 1)*k_ux(1, 2) + Q_xu_(4, 2)*k_ux(2, 2) + Q_xx_(2, 4);
    V_xx(4, 3) = Q_xu_(4, 0)*k_ux(0, 3) + Q_xu_(4, 1)*k_ux(1, 3) + Q_xu_(4, 2)*k_ux(2, 3) + Q_xx_(3, 4);
    V_xx(4, 4) = Q_xu_(4, 0)*k_ux(0, 4) + Q_xu_(4, 1)*k_ux(1, 4) + Q_xu_(4, 2)*k_ux(2, 4) + Q_xx_(4, 4);
    V_xx(4, 5) = Q_xu_(4, 0)*k_ux(0, 5) + Q_xu_(4, 1)*k_ux(1, 5) + Q_xu_(4, 2)*k_ux(2, 5) + Q_xx_(4, 5);
    V_xx(4, 6) = Q_xu_(4, 0)*k_ux(0, 6) + Q_xu_(4, 1)*k_ux(1, 6) + Q_xu_(4, 2)*k_ux(2, 6) + Q_xx_(4, 6);
    V_xx(4, 7) = Q_xu_(4, 0)*k_ux(0, 7) + Q_xu_(4, 1)*k_ux(1, 7) + Q_xu_(4, 2)*k_ux(2, 7) + Q_xx_(4, 7);
    V_xx(4, 8) = Q_xu_(4, 0)*k_ux(0, 8) + Q_xu_(4, 1)*k_ux(1, 8) + Q_xu_(4, 2)*k_ux(2, 8) + Q_xx_(4, 8);
    V_xx(4, 9) = Q_xu_(4, 0)*k_ux(0, 9) + Q_xu_(4, 1)*k_ux(1, 9) + Q_xu_(4, 2)*k_ux(2, 9) + Q_xx_(4, 9);
    V_xx(5, 0) = Q_xu_(5, 0)*k_ux(0, 0) + Q_xu_(5, 1)*k_ux(1, 0) + Q_xu_(5, 2)*k_ux(2, 0) + Q_xx_(0, 5);
    V_xx(5, 1) = Q_xu_(5, 0)*k_ux(0, 1) + Q_xu_(5, 1)*k_ux(1, 1) + Q_xu_(5, 2)*k_ux(2, 1) + Q_xx_(1, 5);
    V_xx(5, 2) = Q_xu_(5, 0)*k_ux(0, 2) + Q_xu_(5, 1)*k_ux(1, 2) + Q_xu_(5, 2)*k_ux(2, 2) + Q_xx_(2, 5);
    V_xx(5, 3) = Q_xu_(5, 0)*k_ux(0, 3) + Q_xu_(5, 1)*k_ux(1, 3) + Q_xu_(5, 2)*k_ux(2, 3) + Q_xx_(3, 5);
    V_xx(5, 4) = Q_xu_(5, 0)*k_ux(0, 4) + Q_xu_(5, 1)*k_ux(1, 4) + Q_xu_(5, 2)*k_ux(2, 4) + Q_xx_(4, 5);
    V_xx(5, 5) = Q_xu_(5, 0)*k_ux(0, 5) + Q_xu_(5, 1)*k_ux(1, 5) + Q_xu_(5, 2)*k_ux(2, 5) + Q_xx_(5, 5);
    V_xx(5, 6) = Q_xu_(5, 0)*k_ux(0, 6) + Q_xu_(5, 1)*k_ux(1, 6) + Q_xu_(5, 2)*k_ux(2, 6) + Q_xx_(5, 6);
    V_xx(5, 7) = Q_xu_(5, 0)*k_ux(0, 7) + Q_xu_(5, 1)*k_ux(1, 7) + Q_xu_(5, 2)*k_ux(2, 7) + Q_xx_(5, 7);
    V_xx(5, 8) = Q_xu_(5, 0)*k_ux(0, 8) + Q_xu_(5, 1)*k_ux(1, 8) + Q_xu_(5, 2)*k_ux(2, 8) + Q_xx_(5, 8);
    V_xx(5, 9) = Q_xu_(5, 0)*k_ux(0, 9) + Q_xu_(5, 1)*k_ux(1, 9) + Q_xu_(5, 2)*k_ux(2, 9) + Q_xx_(5, 9);
    V_xx(6, 0) = Q_xu_(6, 0)*k_ux(0, 0) + Q_xu_(6, 1)*k_ux(1, 0) + Q_xu_(6, 2)*k_ux(2, 0) + Q_xx_(0, 6);
    V_xx(6, 1) = Q_xu_(6, 0)*k_ux(0, 1) + Q_xu_(6, 1)*k_ux(1, 1) + Q_xu_(6, 2)*k_ux(2, 1) + Q_xx_(1, 6);
    V_xx(6, 2) = Q_xu_(6, 0)*k_ux(0, 2) + Q_xu_(6, 1)*k_ux(1, 2) + Q_xu_(6, 2)*k_ux(2, 2) + Q_xx_(2, 6);
    V_xx(6, 3) = Q_xu_(6, 0)*k_ux(0, 3) + Q_xu_(6, 1)*k_ux(1, 3) + Q_xu_(6, 2)*k_ux(2, 3) + Q_xx_(3, 6);
    V_xx(6, 4) = Q_xu_(6, 0)*k_ux(0, 4) + Q_xu_(6, 1)*k_ux(1, 4) + Q_xu_(6, 2)*k_ux(2, 4) + Q_xx_(4, 6);
    V_xx(6, 5) = Q_xu_(6, 0)*k_ux(0, 5) + Q_xu_(6, 1)*k_ux(1, 5) + Q_xu_(6, 2)*k_ux(2, 5) + Q_xx_(5, 6);
    V_xx(6, 6) = Q_xu_(6, 0)*k_ux(0, 6) + Q_xu_(6, 1)*k_ux(1, 6) + Q_xu_(6, 2)*k_ux(2, 6) + Q_xx_(6, 6);
    V_xx(6, 7) = Q_xu_(6, 0)*k_ux(0, 7) + Q_xu_(6, 1)*k_ux(1, 7) + Q_xu_(6, 2)*k_ux(2, 7) + Q_xx_(6, 7);
    V_xx(6, 8) = Q_xu_(6, 0)*k_ux(0, 8) + Q_xu_(6, 1)*k_ux(1, 8) + Q_xu_(6, 2)*k_ux(2, 8) + Q_xx_(6, 8);
    V_xx(6, 9) = Q_xu_(6, 0)*k_ux(0, 9) + Q_xu_(6, 1)*k_ux(1, 9) + Q_xu_(6, 2)*k_ux(2, 9) + Q_xx_(6, 9);
    V_xx(7, 0) = Q_xu_(7, 0)*k_ux(0, 0) + Q_xu_(7, 1)*k_ux(1, 0) + Q_xu_(7, 2)*k_ux(2, 0) + Q_xx_(0, 7);
    V_xx(7, 1) = Q_xu_(7, 0)*k_ux(0, 1) + Q_xu_(7, 1)*k_ux(1, 1) + Q_xu_(7, 2)*k_ux(2, 1) + Q_xx_(1, 7);
    V_xx(7, 2) = Q_xu_(7, 0)*k_ux(0, 2) + Q_xu_(7, 1)*k_ux(1, 2) + Q_xu_(7, 2)*k_ux(2, 2) + Q_xx_(2, 7);
    V_xx(7, 3) = Q_xu_(7, 0)*k_ux(0, 3) + Q_xu_(7, 1)*k_ux(1, 3) + Q_xu_(7, 2)*k_ux(2, 3) + Q_xx_(3, 7);
    V_xx(7, 4) = Q_xu_(7, 0)*k_ux(0, 4) + Q_xu_(7, 1)*k_ux(1, 4) + Q_xu_(7, 2)*k_ux(2, 4) + Q_xx_(4, 7);
    V_xx(7, 5) = Q_xu_(7, 0)*k_ux(0, 5) + Q_xu_(7, 1)*k_ux(1, 5) + Q_xu_(7, 2)*k_ux(2, 5) + Q_xx_(5, 7);
    V_xx(7, 6) = Q_xu_(7, 0)*k_ux(0, 6) + Q_xu_(7, 1)*k_ux(1, 6) + Q_xu_(7, 2)*k_ux(2, 6) + Q_xx_(6, 7);
    V_xx(7, 7) = Q_xu_(7, 0)*k_ux(0, 7) + Q_xu_(7, 1)*k_ux(1, 7) + Q_xu_(7, 2)*k_ux(2, 7) + Q_xx_(7, 7);
    V_xx(7, 8) = Q_xu_(7, 0)*k_ux(0, 8) + Q_xu_(7, 1)*k_ux(1, 8) + Q_xu_(7, 2)*k_ux(2, 8) + Q_xx_(7, 8);
    V_xx(7, 9) = Q_xu_(7, 0)*k_ux(0, 9) + Q_xu_(7, 1)*k_ux(1, 9) + Q_xu_(7, 2)*k_ux(2, 9) + Q_xx_(7, 9);
    V_xx(8, 0) = Q_xu_(8, 0)*k_ux(0, 0) + Q_xu_(8, 1)*k_ux(1, 0) + Q_xu_(8, 2)*k_ux(2, 0) + Q_xx_(0, 8);
    V_xx(8, 1) = Q_xu_(8, 0)*k_ux(0, 1) + Q_xu_(8, 1)*k_ux(1, 1) + Q_xu_(8, 2)*k_ux(2, 1) + Q_xx_(1, 8);
    V_xx(8, 2) = Q_xu_(8, 0)*k_ux(0, 2) + Q_xu_(8, 1)*k_ux(1, 2) + Q_xu_(8, 2)*k_ux(2, 2) + Q_xx_(2, 8);
    V_xx(8, 3) = Q_xu_(8, 0)*k_ux(0, 3) + Q_xu_(8, 1)*k_ux(1, 3) + Q_xu_(8, 2)*k_ux(2, 3) + Q_xx_(3, 8);
    V_xx(8, 4) = Q_xu_(8, 0)*k_ux(0, 4) + Q_xu_(8, 1)*k_ux(1, 4) + Q_xu_(8, 2)*k_ux(2, 4) + Q_xx_(4, 8);
    V_xx(8, 5) = Q_xu_(8, 0)*k_ux(0, 5) + Q_xu_(8, 1)*k_ux(1, 5) + Q_xu_(8, 2)*k_ux(2, 5) + Q_xx_(5, 8);
    V_xx(8, 6) = Q_xu_(8, 0)*k_ux(0, 6) + Q_xu_(8, 1)*k_ux(1, 6) + Q_xu_(8, 2)*k_ux(2, 6) + Q_xx_(6, 8);
    V_xx(8, 7) = Q_xu_(8, 0)*k_ux(0, 7) + Q_xu_(8, 1)*k_ux(1, 7) + Q_xu_(8, 2)*k_ux(2, 7) + Q_xx_(7, 8);
    V_xx(8, 8) = Q_xu_(8, 0)*k_ux(0, 8) + Q_xu_(8, 1)*k_ux(1, 8) + Q_xu_(8, 2)*k_ux(2, 8) + Q_xx_(8, 8);
    V_xx(8, 9) = Q_xu_(8, 0)*k_ux(0, 9) + Q_xu_(8, 1)*k_ux(1, 9) + Q_xu_(8, 2)*k_ux(2, 9) + Q_xx_(8, 9);
    V_xx(9, 0) = Q_xu_(9, 0)*k_ux(0, 0) + Q_xu_(9, 1)*k_ux(1, 0) + Q_xu_(9, 2)*k_ux(2, 0) + Q_xx_(0, 9);
    V_xx(9, 1) = Q_xu_(9, 0)*k_ux(0, 1) + Q_xu_(9, 1)*k_ux(1, 1) + Q_xu_(9, 2)*k_ux(2, 1) + Q_xx_(1, 9);
    V_xx(9, 2) = Q_xu_(9, 0)*k_ux(0, 2) + Q_xu_(9, 1)*k_ux(1, 2) + Q_xu_(9, 2)*k_ux(2, 2) + Q_xx_(2, 9);
    V_xx(9, 3) = Q_xu_(9, 0)*k_ux(0, 3) + Q_xu_(9, 1)*k_ux(1, 3) + Q_xu_(9, 2)*k_ux(2, 3) + Q_xx_(3, 9);
    V_xx(9, 4) = Q_xu_(9, 0)*k_ux(0, 4) + Q_xu_(9, 1)*k_ux(1, 4) + Q_xu_(9, 2)*k_ux(2, 4) + Q_xx_(4, 9);
    V_xx(9, 5) = Q_xu_(9, 0)*k_ux(0, 5) + Q_xu_(9, 1)*k_ux(1, 5) + Q_xu_(9, 2)*k_ux(2, 5) + Q_xx_(5, 9);
    V_xx(9, 6) = Q_xu_(9, 0)*k_ux(0, 6) + Q_xu_(9, 1)*k_ux(1, 6) + Q_xu_(9, 2)*k_ux(2, 6) + Q_xx_(6, 9);
    V_xx(9, 7) = Q_xu_(9, 0)*k_ux(0, 7) + Q_xu_(9, 1)*k_ux(1, 7) + Q_xu_(9, 2)*k_ux(2, 7) + Q_xx_(7, 9);
    V_xx(9, 8) = Q_xu_(9, 0)*k_ux(0, 8) + Q_xu_(9, 1)*k_ux(1, 8) + Q_xu_(9, 2)*k_ux(2, 8) + Q_xx_(8, 9);
    V_xx(9, 9) = Q_xu_(9, 0)*k_ux(0, 9) + Q_xu_(9, 1)*k_ux(1, 9) + Q_xu_(9, 2)*k_ux(2, 9) + Q_xx_(9, 9);

    // Evaluation of Vector V_x
    V_x(0) = Q_x_(0) + Q_xu_(0, 0)*v_u(0) + Q_xu_(0, 1)*v_u(1) + Q_xu_(0, 2)*v_u(2);
    V_x(1) = Q_x_(1) + Q_xu_(1, 0)*v_u(0) + Q_xu_(1, 1)*v_u(1) + Q_xu_(1, 2)*v_u(2);
    V_x(2) = Q_x_(2) + Q_xu_(2, 0)*v_u(0) + Q_xu_(2, 1)*v_u(1) + Q_xu_(2, 2)*v_u(2);
    V_x(3) = Q_x_(3) + Q_xu_(3, 0)*v_u(0) + Q_xu_(3, 1)*v_u(1) + Q_xu_(3, 2)*v_u(2);
    V_x(4) = Q_x_(4) + Q_xu_(4, 0)*v_u(0) + Q_xu_(4, 1)*v_u(1) + Q_xu_(4, 2)*v_u(2);
    V_x(5) = Q_x_(5) + Q_xu_(5, 0)*v_u(0) + Q_xu_(5, 1)*v_u(1) + Q_xu_(5, 2)*v_u(2);
    V_x(6) = Q_x_(6) + Q_xu_(6, 0)*v_u(0) + Q_xu_(6, 1)*v_u(1) + Q_xu_(6, 2)*v_u(2);
    V_x(7) = Q_x_(7) + Q_xu_(7, 0)*v_u(0) + Q_xu_(7, 1)*v_u(1) + Q_xu_(7, 2)*v_u(2);
    V_x(8) = Q_x_(8) + Q_xu_(8, 0)*v_u(0) + Q_xu_(8, 1)*v_u(1) + Q_xu_(8, 2)*v_u(2);
    V_x(9) = Q_x_(9) + Q_xu_(9, 0)*v_u(0) + Q_xu_(9, 1)*v_u(1) + Q_xu_(9, 2)*v_u(2);

  }

  virtual void updateQnx(const Eigen::VectorXd& r_f, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& h_u, const Eigen::MatrixXd& k_ux, const Eigen::MatrixXd& k_ex, const Eigen::MatrixXd& k_ue, const Eigen::VectorXd& v_u, const Eigen::VectorXd& v_e) override {
    // Determine internal variables
    const double internal_0 = -Q_uu_(0, 1);
    const double internal_1 = -Q_uu_(0, 2);
    const double internal_2 = -Q_uu_(1, 2);

    // Evaluation of Matrix Q_un_
    Q_un_(0, 3) = -Fu(3, 1)*Q_uu_inv_(0, 1);
    Q_un_(0, 4) = -Fu(4, 0)*Q_uu_inv_(0, 0);
    Q_un_(0, 9) = -Fu(9, 2)*Q_uu_inv_(0, 2);
    Q_un_(1, 3) = -Fu(3, 1)*Q_uu_inv_(1, 1);
    Q_un_(1, 4) = -Fu(4, 0)*Q_uu_inv_(0, 1);
    Q_un_(1, 9) = -Fu(9, 2)*Q_uu_inv_(1, 2);
    Q_un_(2, 3) = -Fu(3, 1)*Q_uu_inv_(1, 2);
    Q_un_(2, 4) = -Fu(4, 0)*Q_uu_inv_(0, 2);
    Q_un_(2, 9) = -Fu(9, 2)*Q_uu_inv_(2, 2);

    // Evaluation of Matrix Q_nuu_
    Q_nuu_(0, 0) = -Q_uu_(0, 0);
    Q_nuu_(0, 1) = internal_0;
    Q_nuu_(0, 2) = internal_1;
    Q_nuu_(1, 0) = internal_0;
    Q_nuu_(1, 1) = -Q_uu_(1, 1);
    Q_nuu_(1, 2) = internal_2;
    Q_nuu_(2, 0) = internal_1;
    Q_nuu_(2, 1) = internal_2;
    Q_nuu_(2, 2) = -Q_uu_(2, 2);
    Q_nuu_(3, 3) = -Q_uu_(3, 3);
    Q_nuu_(4, 4) = -Q_uu_(4, 4);
    Q_nuu_(5, 5) = -Q_uu_(5, 5);
    Q_nuu_(6, 6) = -Q_uu_(6, 6);
    Q_nuu_(7, 7) = -Q_uu_(7, 7);
    Q_nuu_(8, 8) = -Q_uu_(8, 8);

    // Evaluation of Matrix Q_vnx_
    Q_vnx_(0, 0) = Fx(0, 0);
    Q_vnx_(0, 1) = Fx(0, 1);
    Q_vnx_(0, 2) = Fx(0, 2);
    Q_vnx_(0, 3) = Fx(0, 3);
    Q_vnx_(0, 4) = Fx(0, 4);
    Q_vnx_(1, 1) = Fx(1, 1);
    Q_vnx_(1, 3) = Fx(1, 3);
    Q_vnx_(1, 4) = Fx(1, 4);
    Q_vnx_(2, 2) = Fx(2, 2);
    Q_vnx_(2, 3) = Fx(2, 3);
    Q_vnx_(2, 4) = Fx(2, 4);
    Q_vnx_(3, 0) = Fu(3, 1)*k_ux(1, 0);
    Q_vnx_(3, 1) = Fu(3, 1)*k_ux(1, 1);
    Q_vnx_(3, 2) = Fu(3, 1)*k_ux(1, 2);
    Q_vnx_(3, 3) = Fu(3, 1)*k_ux(1, 3) + Fx(3, 3);
    Q_vnx_(3, 4) = Fu(3, 1)*k_ux(1, 4) + Fx(3, 4);
    Q_vnx_(3, 5) = Fu(3, 1)*k_ux(1, 5);
    Q_vnx_(3, 6) = Fu(3, 1)*k_ux(1, 6);
    Q_vnx_(3, 7) = Fu(3, 1)*k_ux(1, 7);
    Q_vnx_(3, 8) = Fu(3, 1)*k_ux(1, 8);
    Q_vnx_(3, 9) = Fu(3, 1)*k_ux(1, 9);
    Q_vnx_(4, 0) = Fu(4, 0)*k_ux(0, 0);
    Q_vnx_(4, 1) = Fu(4, 0)*k_ux(0, 1);
    Q_vnx_(4, 2) = Fu(4, 0)*k_ux(0, 2);
    Q_vnx_(4, 3) = Fu(4, 0)*k_ux(0, 3);
    Q_vnx_(4, 4) = Fu(4, 0)*k_ux(0, 4) + Fx(4, 4);
    Q_vnx_(4, 5) = Fu(4, 0)*k_ux(0, 5);
    Q_vnx_(4, 6) = Fu(4, 0)*k_ux(0, 6);
    Q_vnx_(4, 7) = Fu(4, 0)*k_ux(0, 7);
    Q_vnx_(4, 8) = Fu(4, 0)*k_ux(0, 8);
    Q_vnx_(4, 9) = Fu(4, 0)*k_ux(0, 9);
    Q_vnx_(5, 5) = Fx(5, 5);
    Q_vnx_(5, 9) = Fx(5, 9);
    Q_vnx_(6, 6) = Fx(6, 6);
    Q_vnx_(6, 9) = Fx(6, 9);
    Q_vnx_(7, 7) = Fx(7, 7);
    Q_vnx_(7, 9) = Fx(7, 9);
    Q_vnx_(8, 8) = Fx(8, 8);
    Q_vnx_(9, 0) = Fu(9, 2)*k_ux(2, 0);
    Q_vnx_(9, 1) = Fu(9, 2)*k_ux(2, 1);
    Q_vnx_(9, 2) = Fu(9, 2)*k_ux(2, 2);
    Q_vnx_(9, 3) = Fu(9, 2)*k_ux(2, 3);
    Q_vnx_(9, 4) = Fu(9, 2)*k_ux(2, 4);
    Q_vnx_(9, 5) = Fu(9, 2)*k_ux(2, 5);
    Q_vnx_(9, 6) = Fu(9, 2)*k_ux(2, 6);
    Q_vnx_(9, 7) = Fu(9, 2)*k_ux(2, 7);
    Q_vnx_(9, 8) = Fu(9, 2)*k_ux(2, 8);
    Q_vnx_(9, 9) = Fu(9, 2)*k_ux(2, 9) + Fx(9, 9);

    // Evaluation of Vector Q_vn_
    Q_vn_(0) = r_f(0);
    Q_vn_(1) = r_f(1);
    Q_vn_(2) = r_f(2);
    Q_vn_(3) = Fu(3, 1)*v_u(1) + r_f(3);
    Q_vn_(4) = Fu(4, 0)*v_u(0) + r_f(4);
    Q_vn_(5) = r_f(5);
    Q_vn_(6) = r_f(6);
    Q_vn_(7) = r_f(7);
    Q_vn_(8) = r_f(8);
    Q_vn_(9) = Fu(9, 2)*v_u(2) + r_f(9);

  }

};

template <>
class MultiAgentsTrajectoryModelIpmEvaluator<StageType::NORMINAL, OcpConfig::ERK4> : public IpmEvaluator {
 public:
  MultiAgentsTrajectoryModelIpmEvaluator() : IpmEvaluator(10, 9, 28, 0, 0) {}
  virtual ~MultiAgentsTrajectoryModelIpmEvaluator() = default;

  virtual void updateLX(const Eigen::VectorXd& l_x, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& h_x, const Eigen::MatrixXd& m_x, const Eigen::VectorXd& p, const Eigen::VectorXd& lambda, const Eigen::VectorXd& eta, const Eigen::VectorXd& nu, Eigen::VectorXd& sl_x) const override {
    // Evaluation of Vector sl_x
    sl_x(0) = Fx(0, 0)*p(0) + g_x(2, 0)*lambda(2) + g_x(3, 0)*lambda(3) + l_x(0);
    sl_x(1) = Fx(0, 1)*p(0) + Fx(1, 1)*p(1) + g_x(0, 1)*lambda(0) + g_x(1, 1)*lambda(1) + g_x(22, 1)*lambda(22) + g_x(23, 1)*lambda(23) + g_x(24, 1)*lambda(24) + g_x(25, 1)*lambda(25) + g_x(26, 1)*lambda(26) + g_x(27, 1)*lambda(27) + g_x(8, 1)*lambda(8) + l_x(1);
    sl_x(2) = Fx(0, 2)*p(0) + Fx(2, 2)*p(2) + g_x(0, 2)*lambda(0) + g_x(1, 2)*lambda(1) + g_x(22, 2)*lambda(22) + g_x(23, 2)*lambda(23) + g_x(24, 2)*lambda(24) + g_x(25, 2)*lambda(25) + g_x(26, 2)*lambda(26) + g_x(27, 2)*lambda(27) + g_x(8, 2)*lambda(8) + l_x(2);
    sl_x(3) = Fx(0, 3)*p(0) + Fx(1, 3)*p(1) + Fx(2, 3)*p(2) + Fx(3, 3)*p(3) + g_x(22, 3)*lambda(22) + g_x(23, 3)*lambda(23) + g_x(24, 3)*lambda(24) + g_x(25, 3)*lambda(25) + g_x(26, 3)*lambda(26) + g_x(27, 3)*lambda(27) + g_x(9, 3)*lambda(9) + l_x(3);
    sl_x(4) = Fx(0, 4)*p(0) + Fx(1, 4)*p(1) + Fx(2, 4)*p(2) + Fx(3, 4)*p(3) + Fx(4, 4)*p(4) + g_x(5, 4)*lambda(5) + g_x(6, 4)*lambda(6) + l_x(4);
    sl_x(5) = Fx(5, 5)*p(5) + g_x(4, 5)*lambda(4) + l_x(5);
    sl_x(6) = Fx(6, 6)*p(6) + g_x(22, 6)*lambda(22) + g_x(23, 6)*lambda(23);
    sl_x(7) = Fx(7, 7)*p(7) + g_x(22, 7)*lambda(22) + g_x(23, 7)*lambda(23);
    sl_x(8) = Fx(8, 8)*p(8);
    sl_x(9) = Fx(5, 9)*p(5) + Fx(6, 9)*p(6) + Fx(7, 9)*p(7) + Fx(9, 9)*p(9) + g_x(7, 9)*lambda(7) + l_x(9);

  }

  virtual void updateLU(const Eigen::VectorXd& l_u, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& g_u, const Eigen::MatrixXd& h_u, const Eigen::VectorXd& p, const Eigen::VectorXd& lambda, const Eigen::VectorXd& eta, Eigen::VectorXd& sl_u) const override {
    // Evaluation of Vector sl_u
    sl_u(0) = Fu(0, 0)*p(0) + Fu(1, 0)*p(1) + Fu(2, 0)*p(2) + Fu(3, 0)*p(3) + Fu(4, 0)*p(4) + g_u(18, 0)*lambda(18) + g_u(19, 0)*lambda(19) + l_u(0);
    sl_u(1) = Fu(0, 1)*p(0) + Fu(1, 1)*p(1) + Fu(2, 1)*p(2) + Fu(3, 1)*p(3) + g_u(16, 1)*lambda(16) + g_u(17, 1)*lambda(17) + l_u(1);
    sl_u(2) = Fu(5, 2)*p(5) + Fu(6, 2)*p(6) + Fu(7, 2)*p(7) + Fu(9, 2)*p(9) + g_u(20, 2)*lambda(20) + g_u(21, 2)*lambda(21) + l_u(2);
    sl_u(3) = g_u(10, 3)*lambda(10) + g_u(22, 3)*lambda(22) + l_u(3);
    sl_u(4) = g_u(11, 4)*lambda(11) + g_u(23, 4)*lambda(23) + l_u(4);
    sl_u(5) = g_u(12, 5)*lambda(12) + g_u(24, 5)*lambda(24) + l_u(5);
    sl_u(6) = g_u(13, 6)*lambda(13) + g_u(26, 6)*lambda(26) + l_u(6);
    sl_u(7) = g_u(14, 7)*lambda(14) + g_u(25, 7)*lambda(25) + l_u(7);
    sl_u(8) = g_u(15, 8)*lambda(15) + g_u(27, 8)*lambda(27) + l_u(8);

  }

  virtual void updateQH(const Eigen::MatrixXd& l_xx, const Eigen::MatrixXd& l_xu, const Eigen::MatrixXd& l_uu, const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& V_xx_prev) override {
    // Determine internal variables
    const double internal_0 = Fx(0, 1)*V_xx_prev(0, 0);
    const double internal_1 = Fx(0, 0)*V_xx_prev(0, 1);
    const double internal_10 = Fx(0, 0)*V_xx_prev(0, 7);
    const double internal_100 = Fx(7, 7)*V_xx_prev(7, 5);
    const double internal_101 = Fx(7, 7)*V_xx_prev(7, 6);
    const double internal_102 = Fx(7, 7)*Fx(8, 8);
    const double internal_103 = Fx(7, 9)*V_xx_prev(7, 7);
    const double internal_104 = Fx(7, 7)*V_xx_prev(7, 9);
    const double internal_105 = Fx(8, 8)*V_xx_prev(8, 0);
    const double internal_106 = Fx(8, 8)*V_xx_prev(8, 1);
    const double internal_107 = Fx(8, 8)*V_xx_prev(8, 2);
    const double internal_108 = Fx(8, 8)*V_xx_prev(8, 3);
    const double internal_109 = Fx(8, 8)*V_xx_prev(8, 4);
    const double internal_11 = Fx(0, 0)*Fx(8, 8);
    const double internal_110 = Fx(8, 8)*V_xx_prev(8, 5);
    const double internal_111 = Fx(8, 8)*V_xx_prev(8, 6);
    const double internal_112 = Fx(8, 8)*V_xx_prev(8, 7);
    const double internal_113 = Fx(8, 8)*V_xx_prev(8, 9);
    const double internal_12 = Fx(0, 0)*V_xx_prev(0, 9);
    const double internal_123 = Fu(0, 0)*V_xx_prev(0, 0);
    const double internal_124 = Fu(0, 1)*V_xx_prev(0, 0);
    const double internal_125 = Fu(5, 2)*V_xx_prev(5, 5);
    const double internal_126 = Fu(6, 2)*V_xx_prev(6, 6);
    const double internal_127 = Fu(7, 2)*V_xx_prev(7, 7);
    const double internal_16 = Sigma_(22)*g_x(22, 1);
    const double internal_17 = Sigma_(23)*g_x(23, 1);
    const double internal_18 = Sigma_(24)*g_x(24, 1);
    const double internal_19 = Sigma_(25)*g_x(25, 1);
    const double internal_2 = Fx(0, 2)*V_xx_prev(0, 0);
    const double internal_20 = Sigma_(26)*g_x(26, 1);
    const double internal_21 = Sigma_(27)*g_x(27, 1);
    const double internal_3 = Fx(0, 0)*V_xx_prev(0, 2);
    const double internal_36 = Sigma_(22)*g_x(22, 2);
    const double internal_37 = Sigma_(23)*g_x(23, 2);
    const double internal_38 = Sigma_(24)*g_x(24, 2);
    const double internal_39 = Sigma_(25)*g_x(25, 2);
    const double internal_4 = Fx(0, 3)*V_xx_prev(0, 0);
    const double internal_40 = Sigma_(26)*g_x(26, 2);
    const double internal_41 = Sigma_(27)*g_x(27, 2);
    const double internal_5 = Fx(0, 0)*V_xx_prev(0, 3);
    const double internal_57 = Sigma_(22)*g_x(22, 3);
    const double internal_58 = Sigma_(23)*g_x(23, 3);
    const double internal_6 = Fx(0, 4)*V_xx_prev(0, 0);
    const double internal_7 = Fx(0, 0)*V_xx_prev(0, 4);
    const double internal_72 = Fx(5, 5)*V_xx_prev(5, 0);
    const double internal_73 = Fx(5, 5)*V_xx_prev(5, 1);
    const double internal_74 = Fx(5, 5)*V_xx_prev(5, 2);
    const double internal_75 = Fx(5, 5)*V_xx_prev(5, 3);
    const double internal_76 = Fx(5, 5)*V_xx_prev(5, 4);
    const double internal_77 = Fx(5, 5)*V_xx_prev(5, 6);
    const double internal_78 = Fx(5, 5)*V_xx_prev(5, 7);
    const double internal_79 = Fx(5, 5)*Fx(8, 8);
    const double internal_8 = Fx(0, 0)*V_xx_prev(0, 5);
    const double internal_80 = Fx(5, 9)*V_xx_prev(5, 5);
    const double internal_81 = Fx(5, 5)*V_xx_prev(5, 9);
    const double internal_82 = Fx(6, 6)*V_xx_prev(6, 0);
    const double internal_83 = Fx(6, 6)*V_xx_prev(6, 1);
    const double internal_84 = Fx(6, 6)*V_xx_prev(6, 2);
    const double internal_85 = Fx(6, 6)*V_xx_prev(6, 3);
    const double internal_86 = Fx(6, 6)*V_xx_prev(6, 4);
    const double internal_87 = Fx(6, 6)*V_xx_prev(6, 5);
    const double internal_88 = Fx(6, 6)*V_xx_prev(6, 7);
    const double internal_89 = Sigma_(22)*g_x(22, 6);
    const double internal_9 = Fx(0, 0)*V_xx_prev(0, 6);
    const double internal_90 = Sigma_(23)*g_x(23, 6);
    const double internal_92 = Fx(6, 6)*Fx(8, 8);
    const double internal_93 = Fx(6, 9)*V_xx_prev(6, 6);
    const double internal_94 = Fx(6, 6)*V_xx_prev(6, 9);
    const double internal_95 = Fx(7, 7)*V_xx_prev(7, 0);
    const double internal_96 = Fx(7, 7)*V_xx_prev(7, 1);
    const double internal_97 = Fx(7, 7)*V_xx_prev(7, 2);
    const double internal_98 = Fx(7, 7)*V_xx_prev(7, 3);
    const double internal_99 = Fx(7, 7)*V_xx_prev(7, 4);
    const double internal_114 = Fx(5, 9)*V_xx_prev(5, 0) + Fx(6, 9)*V_xx_prev(6, 0) + Fx(7, 9)*V_xx_prev(7, 0) + Fx(9, 9)*V_xx_prev(9, 0);
    const double internal_115 = Fx(5, 9)*V_xx_prev(5, 1) + Fx(6, 9)*V_xx_prev(6, 1) + Fx(7, 9)*V_xx_prev(7, 1) + Fx(9, 9)*V_xx_prev(9, 1);
    const double internal_116 = Fx(5, 9)*V_xx_prev(5, 2) + Fx(6, 9)*V_xx_prev(6, 2) + Fx(7, 9)*V_xx_prev(7, 2) + Fx(9, 9)*V_xx_prev(9, 2);
    const double internal_117 = Fx(5, 9)*V_xx_prev(5, 3) + Fx(6, 9)*V_xx_prev(6, 3) + Fx(7, 9)*V_xx_prev(7, 3) + Fx(9, 9)*V_xx_prev(9, 3);
    const double internal_118 = Fx(5, 9)*V_xx_prev(5, 4) + Fx(6, 9)*V_xx_prev(6, 4) + Fx(7, 9)*V_xx_prev(7, 4) + Fx(9, 9)*V_xx_prev(9, 4);
    const double internal_122 = Fx(5, 9)*V_xx_prev(5, 9) + Fx(6, 9)*V_xx_prev(6, 9) + Fx(7, 9)*V_xx_prev(7, 9) + Fx(9, 9)*V_xx_prev(9, 9);
    const double internal_129 = Fu(0, 0)*V_xx_prev(0, 1) + Fu(1, 0)*V_xx_prev(1, 1) + Fu(2, 0)*V_xx_prev(2, 1) + Fu(3, 0)*V_xx_prev(3, 1) + Fu(4, 0)*V_xx_prev(4, 1);
    const double internal_130 = Fu(0, 0)*V_xx_prev(0, 2) + Fu(1, 0)*V_xx_prev(1, 2) + Fu(2, 0)*V_xx_prev(2, 2) + Fu(3, 0)*V_xx_prev(3, 2) + Fu(4, 0)*V_xx_prev(4, 2);
    const double internal_131 = Fu(0, 0)*V_xx_prev(0, 3) + Fu(1, 0)*V_xx_prev(1, 3) + Fu(2, 0)*V_xx_prev(2, 3) + Fu(3, 0)*V_xx_prev(3, 3) + Fu(4, 0)*V_xx_prev(4, 3);
    const double internal_133 = Fu(0, 1)*V_xx_prev(0, 1) + Fu(1, 1)*V_xx_prev(1, 1) + Fu(2, 1)*V_xx_prev(2, 1) + Fu(3, 1)*V_xx_prev(3, 1);
    const double internal_134 = Fu(0, 1)*V_xx_prev(0, 2) + Fu(1, 1)*V_xx_prev(1, 2) + Fu(2, 1)*V_xx_prev(2, 2) + Fu(3, 1)*V_xx_prev(3, 2);
    const double internal_135 = Fu(0, 1)*V_xx_prev(0, 3) + Fu(1, 1)*V_xx_prev(1, 3) + Fu(2, 1)*V_xx_prev(2, 3) + Fu(3, 1)*V_xx_prev(3, 3);
    const double internal_136 = Fu(5, 2)*V_xx_prev(5, 0) + Fu(6, 2)*V_xx_prev(6, 0) + Fu(7, 2)*V_xx_prev(7, 0) + Fu(9, 2)*V_xx_prev(9, 0);
    const double internal_137 = Fu(5, 2)*V_xx_prev(5, 1) + Fu(6, 2)*V_xx_prev(6, 1) + Fu(7, 2)*V_xx_prev(7, 1) + Fu(9, 2)*V_xx_prev(9, 1);
    const double internal_138 = Fu(5, 2)*V_xx_prev(5, 2) + Fu(6, 2)*V_xx_prev(6, 2) + Fu(7, 2)*V_xx_prev(7, 2) + Fu(9, 2)*V_xx_prev(9, 2);
    const double internal_139 = Fu(5, 2)*V_xx_prev(5, 3) + Fu(6, 2)*V_xx_prev(6, 3) + Fu(7, 2)*V_xx_prev(7, 3) + Fu(9, 2)*V_xx_prev(9, 3);
    const double internal_14 = Fx(0, 1)*V_xx_prev(0, 1) + Fx(1, 1)*V_xx_prev(1, 1);
    const double internal_15 = Fx(0, 1)*V_xx_prev(0, 2) + Fx(1, 1)*V_xx_prev(1, 2);
    const double internal_23 = Fx(0, 1)*V_xx_prev(0, 3) + Fx(1, 1)*V_xx_prev(1, 3);
    const double internal_25 = Fx(0, 1)*V_xx_prev(0, 4) + Fx(1, 1)*V_xx_prev(1, 4);
    const double internal_26 = Fx(0, 1)*V_xx_prev(0, 5) + Fx(1, 1)*V_xx_prev(1, 5);
    const double internal_27 = Fx(0, 1)*V_xx_prev(0, 6) + Fx(1, 1)*V_xx_prev(1, 6);
    const double internal_29 = Fx(0, 1)*V_xx_prev(0, 7) + Fx(1, 1)*V_xx_prev(1, 7);
    const double internal_31 = Fx(0, 1)*V_xx_prev(0, 9) + Fx(1, 1)*V_xx_prev(1, 9);
    const double internal_33 = Fx(0, 2)*V_xx_prev(0, 1) + Fx(2, 2)*V_xx_prev(2, 1);
    const double internal_34 = Fx(0, 2)*V_xx_prev(0, 2) + Fx(2, 2)*V_xx_prev(2, 2);
    const double internal_35 = Fx(0, 2)*V_xx_prev(0, 3) + Fx(2, 2)*V_xx_prev(2, 3);
    const double internal_43 = Fx(0, 2)*V_xx_prev(0, 4) + Fx(2, 2)*V_xx_prev(2, 4);
    const double internal_44 = Fx(0, 2)*V_xx_prev(0, 5) + Fx(2, 2)*V_xx_prev(2, 5);
    const double internal_45 = Fx(0, 2)*V_xx_prev(0, 6) + Fx(2, 2)*V_xx_prev(2, 6);
    const double internal_47 = Fx(0, 2)*V_xx_prev(0, 7) + Fx(2, 2)*V_xx_prev(2, 7);
    const double internal_49 = Fx(0, 2)*V_xx_prev(0, 9) + Fx(2, 2)*V_xx_prev(2, 9);
    const double internal_51 = Fx(0, 3)*V_xx_prev(0, 1) + Fx(1, 3)*V_xx_prev(1, 1) + Fx(2, 3)*V_xx_prev(2, 1) + Fx(3, 3)*V_xx_prev(3, 1);
    const double internal_52 = Fx(0, 3)*V_xx_prev(0, 2) + Fx(1, 3)*V_xx_prev(1, 2) + Fx(2, 3)*V_xx_prev(2, 2) + Fx(3, 3)*V_xx_prev(3, 2);
    const double internal_53 = Fx(0, 3)*V_xx_prev(0, 3) + Fx(1, 3)*V_xx_prev(1, 3) + Fx(2, 3)*V_xx_prev(2, 3) + Fx(3, 3)*V_xx_prev(3, 3);
    const double internal_54 = Fx(0, 3)*V_xx_prev(0, 4) + Fx(1, 3)*V_xx_prev(1, 4) + Fx(2, 3)*V_xx_prev(2, 4) + Fx(3, 3)*V_xx_prev(3, 4);
    const double internal_55 = Fx(0, 3)*V_xx_prev(0, 5) + Fx(1, 3)*V_xx_prev(1, 5) + Fx(2, 3)*V_xx_prev(2, 5) + Fx(3, 3)*V_xx_prev(3, 5);
    const double internal_56 = Fx(0, 3)*V_xx_prev(0, 6) + Fx(1, 3)*V_xx_prev(1, 6) + Fx(2, 3)*V_xx_prev(2, 6) + Fx(3, 3)*V_xx_prev(3, 6);
    const double internal_60 = Fx(0, 3)*V_xx_prev(0, 7) + Fx(1, 3)*V_xx_prev(1, 7) + Fx(2, 3)*V_xx_prev(2, 7) + Fx(3, 3)*V_xx_prev(3, 7);
    const double internal_62 = Fx(0, 3)*V_xx_prev(0, 9) + Fx(1, 3)*V_xx_prev(1, 9) + Fx(2, 3)*V_xx_prev(2, 9) + Fx(3, 3)*V_xx_prev(3, 9);
    const double internal_64 = Fx(0, 4)*V_xx_prev(0, 1) + Fx(1, 4)*V_xx_prev(1, 1) + Fx(2, 4)*V_xx_prev(2, 1) + Fx(3, 4)*V_xx_prev(3, 1) + Fx(4, 4)*V_xx_prev(4, 1);
    const double internal_65 = Fx(0, 4)*V_xx_prev(0, 2) + Fx(1, 4)*V_xx_prev(1, 2) + Fx(2, 4)*V_xx_prev(2, 2) + Fx(3, 4)*V_xx_prev(3, 2) + Fx(4, 4)*V_xx_prev(4, 2);
    const double internal_66 = Fx(0, 4)*V_xx_prev(0, 3) + Fx(1, 4)*V_xx_prev(1, 3) + Fx(2, 4)*V_xx_prev(2, 3) + Fx(3, 4)*V_xx_prev(3, 3) + Fx(4, 4)*V_xx_prev(4, 3);
    const double internal_67 = Fx(0, 4)*V_xx_prev(0, 4) + Fx(1, 4)*V_xx_prev(1, 4) + Fx(2, 4)*V_xx_prev(2, 4) + Fx(3, 4)*V_xx_prev(3, 4) + Fx(4, 4)*V_xx_prev(4, 4);
    const double internal_68 = Fx(0, 4)*V_xx_prev(0, 5) + Fx(1, 4)*V_xx_prev(1, 5) + Fx(2, 4)*V_xx_prev(2, 5) + Fx(3, 4)*V_xx_prev(3, 5) + Fx(4, 4)*V_xx_prev(4, 5);
    const double internal_69 = Fx(0, 4)*V_xx_prev(0, 6) + Fx(1, 4)*V_xx_prev(1, 6) + Fx(2, 4)*V_xx_prev(2, 6) + Fx(3, 4)*V_xx_prev(3, 6) + Fx(4, 4)*V_xx_prev(4, 6);
    const double internal_70 = Fx(0, 4)*V_xx_prev(0, 7) + Fx(1, 4)*V_xx_prev(1, 7) + Fx(2, 4)*V_xx_prev(2, 7) + Fx(3, 4)*V_xx_prev(3, 7) + Fx(4, 4)*V_xx_prev(4, 7);
    const double internal_71 = Fx(0, 4)*V_xx_prev(0, 9) + Fx(1, 4)*V_xx_prev(1, 9) + Fx(2, 4)*V_xx_prev(2, 9) + Fx(3, 4)*V_xx_prev(3, 9) + Fx(4, 4)*V_xx_prev(4, 9);
    const double internal_119 = Fx(6, 9)*V_xx_prev(6, 5) + Fx(7, 9)*V_xx_prev(7, 5) + Fx(9, 9)*V_xx_prev(9, 5) + internal_80;
    const double internal_120 = Fx(5, 9)*V_xx_prev(5, 6) + Fx(7, 9)*V_xx_prev(7, 6) + Fx(9, 9)*V_xx_prev(9, 6) + internal_93;
    const double internal_121 = Fx(5, 9)*V_xx_prev(5, 7) + Fx(6, 9)*V_xx_prev(6, 7) + Fx(9, 9)*V_xx_prev(9, 7) + internal_103;
    const double internal_128 = Fu(1, 0)*V_xx_prev(1, 0) + Fu(2, 0)*V_xx_prev(2, 0) + Fu(3, 0)*V_xx_prev(3, 0) + Fu(4, 0)*V_xx_prev(4, 0) + internal_123;
    const double internal_13 = Fx(1, 1)*V_xx_prev(1, 0) + internal_0;
    const double internal_132 = Fu(1, 1)*V_xx_prev(1, 0) + Fu(2, 1)*V_xx_prev(2, 0) + Fu(3, 1)*V_xx_prev(3, 0) + internal_124;
    const double internal_32 = Fx(2, 2)*V_xx_prev(2, 0) + internal_2;
    const double internal_50 = Fx(1, 3)*V_xx_prev(1, 0) + Fx(2, 3)*V_xx_prev(2, 0) + Fx(3, 3)*V_xx_prev(3, 0) + internal_4;
    const double internal_63 = Fx(1, 4)*V_xx_prev(1, 0) + Fx(2, 4)*V_xx_prev(2, 0) + Fx(3, 4)*V_xx_prev(3, 0) + Fx(4, 4)*V_xx_prev(4, 0) + internal_6;
    const double internal_22 = Sigma_(0)*g_x(0, 1)*g_x(0, 2) + Sigma_(1)*g_x(1, 1)*g_x(1, 2) + Sigma_(8)*g_x(8, 1)*g_x(8, 2) + g_x(22, 2)*internal_16 + g_x(23, 2)*internal_17 + g_x(24, 2)*internal_18 + g_x(25, 2)*internal_19 + g_x(26, 2)*internal_20 + g_x(27, 2)*internal_21;
    const double internal_24 = g_x(22, 3)*internal_16 + g_x(23, 3)*internal_17 + g_x(24, 3)*internal_18 + g_x(25, 3)*internal_19 + g_x(26, 3)*internal_20 + g_x(27, 3)*internal_21;
    const double internal_28 = g_x(22, 6)*internal_16 + g_x(23, 6)*internal_17;
    const double internal_30 = g_x(22, 7)*internal_16 + g_x(23, 7)*internal_17;
    const double internal_42 = g_x(22, 3)*internal_36 + g_x(23, 3)*internal_37 + g_x(24, 3)*internal_38 + g_x(25, 3)*internal_39 + g_x(26, 3)*internal_40 + g_x(27, 3)*internal_41;
    const double internal_46 = g_x(22, 6)*internal_36 + g_x(23, 6)*internal_37;
    const double internal_48 = g_x(22, 7)*internal_36 + g_x(23, 7)*internal_37;
    const double internal_59 = g_x(22, 6)*internal_57 + g_x(23, 6)*internal_58;
    const double internal_61 = g_x(22, 7)*internal_57 + g_x(23, 7)*internal_58;
    const double internal_91 = g_x(22, 7)*internal_89 + g_x(23, 7)*internal_90;

    // Evaluation of Matrix Q_xx_
    Q_xx_(0, 0) = pow(Fx(0, 0), 2)*V_xx_prev(0, 0) + Sigma_(2)*pow(g_x(2, 0), 2) + Sigma_(3)*pow(g_x(3, 0), 2) + l_xx(0, 0);
    Q_xx_(0, 1) = Fx(0, 0)*internal_0 + Fx(1, 1)*internal_1;
    Q_xx_(0, 2) = Fx(0, 0)*internal_2 + Fx(2, 2)*internal_3;
    Q_xx_(0, 3) = Fx(0, 0)*internal_4 + Fx(1, 3)*internal_1 + Fx(2, 3)*internal_3 + Fx(3, 3)*internal_5;
    Q_xx_(0, 4) = Fx(0, 0)*internal_6 + Fx(1, 4)*internal_1 + Fx(2, 4)*internal_3 + Fx(3, 4)*internal_5 + Fx(4, 4)*internal_7;
    Q_xx_(0, 5) = Fx(5, 5)*internal_8;
    Q_xx_(0, 6) = Fx(6, 6)*internal_9;
    Q_xx_(0, 7) = Fx(7, 7)*internal_10;
    Q_xx_(0, 8) = V_xx_prev(0, 8)*internal_11;
    Q_xx_(0, 9) = Fx(5, 9)*internal_8 + Fx(6, 9)*internal_9 + Fx(7, 9)*internal_10 + Fx(9, 9)*internal_12;
    Q_xx_(1, 0) = Fx(0, 0)*internal_13;
    Q_xx_(1, 1) = Fx(0, 1)*internal_13 + Fx(1, 1)*internal_14 + Sigma_(0)*pow(g_x(0, 1), 2) + Sigma_(1)*pow(g_x(1, 1), 2) + Sigma_(22)*pow(g_x(22, 1), 2) + Sigma_(23)*pow(g_x(23, 1), 2) + Sigma_(24)*pow(g_x(24, 1), 2) + Sigma_(25)*pow(g_x(25, 1), 2) + Sigma_(26)*pow(g_x(26, 1), 2) + Sigma_(27)*pow(g_x(27, 1), 2) + Sigma_(8)*pow(g_x(8, 1), 2) + l_xx(1, 1);
    Q_xx_(1, 2) = Fx(0, 2)*internal_13 + Fx(2, 2)*internal_15 + internal_22 + l_xx(1, 2);
    Q_xx_(1, 3) = Fx(0, 3)*internal_13 + Fx(1, 3)*internal_14 + Fx(2, 3)*internal_15 + Fx(3, 3)*internal_23 + internal_24;
    Q_xx_(1, 4) = Fx(0, 4)*internal_13 + Fx(1, 4)*internal_14 + Fx(2, 4)*internal_15 + Fx(3, 4)*internal_23 + Fx(4, 4)*internal_25;
    Q_xx_(1, 5) = Fx(5, 5)*internal_26;
    Q_xx_(1, 6) = Fx(6, 6)*internal_27 + internal_28;
    Q_xx_(1, 7) = Fx(7, 7)*internal_29 + internal_30;
    Q_xx_(1, 8) = Fx(8, 8)*(Fx(0, 1)*V_xx_prev(0, 8) + Fx(1, 1)*V_xx_prev(1, 8));
    Q_xx_(1, 9) = Fx(5, 9)*internal_26 + Fx(6, 9)*internal_27 + Fx(7, 9)*internal_29 + Fx(9, 9)*internal_31;
    Q_xx_(2, 0) = Fx(0, 0)*internal_32;
    Q_xx_(2, 1) = Fx(0, 1)*internal_32 + Fx(1, 1)*internal_33 + internal_22 + l_xx(2, 1);
    Q_xx_(2, 2) = Fx(0, 2)*internal_32 + Fx(2, 2)*internal_34 + Sigma_(0)*pow(g_x(0, 2), 2) + Sigma_(1)*pow(g_x(1, 2), 2) + Sigma_(22)*pow(g_x(22, 2), 2) + Sigma_(23)*pow(g_x(23, 2), 2) + Sigma_(24)*pow(g_x(24, 2), 2) + Sigma_(25)*pow(g_x(25, 2), 2) + Sigma_(26)*pow(g_x(26, 2), 2) + Sigma_(27)*pow(g_x(27, 2), 2) + Sigma_(8)*pow(g_x(8, 2), 2) + l_xx(2, 2);
    Q_xx_(2, 3) = Fx(0, 3)*internal_32 + Fx(1, 3)*internal_33 + Fx(2, 3)*internal_34 + Fx(3, 3)*internal_35 + internal_42;
    Q_xx_(2, 4) = Fx(0, 4)*internal_32 + Fx(1, 4)*internal_33 + Fx(2, 4)*internal_34 + Fx(3, 4)*internal_35 + Fx(4, 4)*internal_43;
    Q_xx_(2, 5) = Fx(5, 5)*internal_44;
    Q_xx_(2, 6) = Fx(6, 6)*internal_45 + internal_46;
    Q_xx_(2, 7) = Fx(7, 7)*internal_47 + internal_48;
    Q_xx_(2, 8) = Fx(8, 8)*(Fx(0, 2)*V_xx_prev(0, 8) + Fx(2, 2)*V_xx_prev(2, 8));
    Q_xx_(2, 9) = Fx(5, 9)*internal_44 + Fx(6, 9)*internal_45 + Fx(7, 9)*internal_47 + Fx(9, 9)*internal_49;
    Q_xx_(3, 0) = Fx(0, 0)*internal_50;
    Q_xx_(3, 1) = Fx(0, 1)*internal_50 + Fx(1, 1)*internal_51 + internal_24;
    Q_xx_(3, 2) = Fx(0, 2)*internal_50 + Fx(2, 2)*internal_52 + internal_42;
    Q_xx_(3, 3) = Fx(0, 3)*internal_50 + Fx(1, 3)*internal_51 + Fx(2, 3)*internal_52 + Fx(3, 3)*internal_53 + Sigma_(22)*pow(g_x(22, 3), 2) + Sigma_(23)*pow(g_x(23, 3), 2) + Sigma_(24)*pow(g_x(24, 3), 2) + Sigma_(25)*pow(g_x(25, 3), 2) + Sigma_(26)*pow(g_x(26, 3), 2) + Sigma_(27)*pow(g_x(27, 3), 2) + Sigma_(9)*pow(g_x(9, 3), 2) + l_xx(3, 3);
    Q_xx_(3, 4) = Fx(0, 4)*internal_50 + Fx(1, 4)*internal_51 + Fx(2, 4)*internal_52 + Fx(3, 4)*internal_53 + Fx(4, 4)*internal_54;
    Q_xx_(3, 5) = Fx(5, 5)*internal_55;
    Q_xx_(3, 6) = Fx(6, 6)*internal_56 + internal_59;
    Q_xx_(3, 7) = Fx(7, 7)*internal_60 + internal_61;
    Q_xx_(3, 8) = Fx(8, 8)*(Fx(0, 3)*V_xx_prev(0, 8) + Fx(1, 3)*V_xx_prev(1, 8) + Fx(2, 3)*V_xx_prev(2, 8) + Fx(3, 3)*V_xx_prev(3, 8));
    Q_xx_(3, 9) = Fx(5, 9)*internal_55 + Fx(6, 9)*internal_56 + Fx(7, 9)*internal_60 + Fx(9, 9)*internal_62;
    Q_xx_(4, 0) = Fx(0, 0)*internal_63;
    Q_xx_(4, 1) = Fx(0, 1)*internal_63 + Fx(1, 1)*internal_64;
    Q_xx_(4, 2) = Fx(0, 2)*internal_63 + Fx(2, 2)*internal_65;
    Q_xx_(4, 3) = Fx(0, 3)*internal_63 + Fx(1, 3)*internal_64 + Fx(2, 3)*internal_65 + Fx(3, 3)*internal_66;
    Q_xx_(4, 4) = Fx(0, 4)*internal_63 + Fx(1, 4)*internal_64 + Fx(2, 4)*internal_65 + Fx(3, 4)*internal_66 + Fx(4, 4)*internal_67 + Sigma_(5)*pow(g_x(5, 4), 2) + Sigma_(6)*pow(g_x(6, 4), 2) + l_xx(4, 4);
    Q_xx_(4, 5) = Fx(5, 5)*internal_68;
    Q_xx_(4, 6) = Fx(6, 6)*internal_69;
    Q_xx_(4, 7) = Fx(7, 7)*internal_70;
    Q_xx_(4, 8) = Fx(8, 8)*(Fx(0, 4)*V_xx_prev(0, 8) + Fx(1, 4)*V_xx_prev(1, 8) + Fx(2, 4)*V_xx_prev(2, 8) + Fx(3, 4)*V_xx_prev(3, 8) + Fx(4, 4)*V_xx_prev(4, 8));
    Q_xx_(4, 9) = Fx(5, 9)*internal_68 + Fx(6, 9)*internal_69 + Fx(7, 9)*internal_70 + Fx(9, 9)*internal_71;
    Q_xx_(5, 0) = Fx(0, 0)*internal_72;
    Q_xx_(5, 1) = Fx(0, 1)*internal_72 + Fx(1, 1)*internal_73;
    Q_xx_(5, 2) = Fx(0, 2)*internal_72 + Fx(2, 2)*internal_74;
    Q_xx_(5, 3) = Fx(0, 3)*internal_72 + Fx(1, 3)*internal_73 + Fx(2, 3)*internal_74 + Fx(3, 3)*internal_75;
    Q_xx_(5, 4) = Fx(0, 4)*internal_72 + Fx(1, 4)*internal_73 + Fx(2, 4)*internal_74 + Fx(3, 4)*internal_75 + Fx(4, 4)*internal_76;
    Q_xx_(5, 5) = pow(Fx(5, 5), 2)*V_xx_prev(5, 5) + Sigma_(4)*pow(g_x(4, 5), 2) + l_xx(5, 5);
    Q_xx_(5, 6) = Fx(6, 6)*internal_77;
    Q_xx_(5, 7) = Fx(7, 7)*internal_78;
    Q_xx_(5, 8) = V_xx_prev(5, 8)*internal_79;
    Q_xx_(5, 9) = Fx(5, 5)*internal_80 + Fx(6, 9)*internal_77 + Fx(7, 9)*internal_78 + Fx(9, 9)*internal_81;
    Q_xx_(6, 0) = Fx(0, 0)*internal_82;
    Q_xx_(6, 1) = Fx(0, 1)*internal_82 + Fx(1, 1)*internal_83 + internal_28;
    Q_xx_(6, 2) = Fx(0, 2)*internal_82 + Fx(2, 2)*internal_84 + internal_46;
    Q_xx_(6, 3) = Fx(0, 3)*internal_82 + Fx(1, 3)*internal_83 + Fx(2, 3)*internal_84 + Fx(3, 3)*internal_85 + internal_59;
    Q_xx_(6, 4) = Fx(0, 4)*internal_82 + Fx(1, 4)*internal_83 + Fx(2, 4)*internal_84 + Fx(3, 4)*internal_85 + Fx(4, 4)*internal_86;
    Q_xx_(6, 5) = Fx(5, 5)*internal_87;
    Q_xx_(6, 6) = pow(Fx(6, 6), 2)*V_xx_prev(6, 6) + Sigma_(22)*pow(g_x(22, 6), 2) + Sigma_(23)*pow(g_x(23, 6), 2);
    Q_xx_(6, 7) = Fx(7, 7)*internal_88 + internal_91;
    Q_xx_(6, 8) = V_xx_prev(6, 8)*internal_92;
    Q_xx_(6, 9) = Fx(5, 9)*internal_87 + Fx(6, 6)*internal_93 + Fx(7, 9)*internal_88 + Fx(9, 9)*internal_94;
    Q_xx_(7, 0) = Fx(0, 0)*internal_95;
    Q_xx_(7, 1) = Fx(0, 1)*internal_95 + Fx(1, 1)*internal_96 + internal_30;
    Q_xx_(7, 2) = Fx(0, 2)*internal_95 + Fx(2, 2)*internal_97 + internal_48;
    Q_xx_(7, 3) = Fx(0, 3)*internal_95 + Fx(1, 3)*internal_96 + Fx(2, 3)*internal_97 + Fx(3, 3)*internal_98 + internal_61;
    Q_xx_(7, 4) = Fx(0, 4)*internal_95 + Fx(1, 4)*internal_96 + Fx(2, 4)*internal_97 + Fx(3, 4)*internal_98 + Fx(4, 4)*internal_99;
    Q_xx_(7, 5) = Fx(5, 5)*internal_100;
    Q_xx_(7, 6) = Fx(6, 6)*internal_101 + internal_91;
    Q_xx_(7, 7) = pow(Fx(7, 7), 2)*V_xx_prev(7, 7) + Sigma_(22)*pow(g_x(22, 7), 2) + Sigma_(23)*pow(g_x(23, 7), 2);
    Q_xx_(7, 8) = V_xx_prev(7, 8)*internal_102;
    Q_xx_(7, 9) = Fx(5, 9)*internal_100 + Fx(6, 9)*internal_101 + Fx(7, 7)*internal_103 + Fx(9, 9)*internal_104;
    Q_xx_(8, 0) = V_xx_prev(8, 0)*internal_11;
    Q_xx_(8, 1) = Fx(0, 1)*internal_105 + Fx(1, 1)*internal_106;
    Q_xx_(8, 2) = Fx(0, 2)*internal_105 + Fx(2, 2)*internal_107;
    Q_xx_(8, 3) = Fx(0, 3)*internal_105 + Fx(1, 3)*internal_106 + Fx(2, 3)*internal_107 + Fx(3, 3)*internal_108;
    Q_xx_(8, 4) = Fx(0, 4)*internal_105 + Fx(1, 4)*internal_106 + Fx(2, 4)*internal_107 + Fx(3, 4)*internal_108 + Fx(4, 4)*internal_109;
    Q_xx_(8, 5) = V_xx_prev(8, 5)*internal_79;
    Q_xx_(8, 6) = V_xx_prev(8, 6)*internal_92;
    Q_xx_(8, 7) = V_xx_prev(8, 7)*internal_102;
    Q_xx_(8, 8) = pow(Fx(8, 8), 2)*V_xx_prev(8, 8);
    Q_xx_(8, 9) = Fx(5, 9)*internal_110 + Fx(6, 9)*internal_111 + Fx(7, 9)*internal_112 + Fx(9, 9)*internal_113;
    Q_xx_(9, 0) = Fx(0, 0)*internal_114;
    Q_xx_(9, 1) = Fx(0, 1)*internal_114 + Fx(1, 1)*internal_115;
    Q_xx_(9, 2) = Fx(0, 2)*internal_114 + Fx(2, 2)*internal_116;
    Q_xx_(9, 3) = Fx(0, 3)*internal_114 + Fx(1, 3)*internal_115 + Fx(2, 3)*internal_116 + Fx(3, 3)*internal_117;
    Q_xx_(9, 4) = Fx(0, 4)*internal_114 + Fx(1, 4)*internal_115 + Fx(2, 4)*internal_116 + Fx(3, 4)*internal_117 + Fx(4, 4)*internal_118;
    Q_xx_(9, 5) = Fx(5, 5)*internal_119;
    Q_xx_(9, 6) = Fx(6, 6)*internal_120;
    Q_xx_(9, 7) = Fx(7, 7)*internal_121;
    Q_xx_(9, 8) = Fx(8, 8)*(Fx(5, 9)*V_xx_prev(5, 8) + Fx(6, 9)*V_xx_prev(6, 8) + Fx(7, 9)*V_xx_prev(7, 8) + Fx(9, 9)*V_xx_prev(9, 8));
    Q_xx_(9, 9) = Fx(5, 9)*internal_119 + Fx(6, 9)*internal_120 + Fx(7, 9)*internal_121 + Fx(9, 9)*internal_122 + Sigma_(7)*pow(g_x(7, 9), 2) + l_xx(9, 9);

    // Evaluation of Matrix Q_xu_
    Q_xu_(0, 0) = Fu(1, 0)*internal_1 + Fu(2, 0)*internal_3 + Fu(3, 0)*internal_5 + Fu(4, 0)*internal_7 + Fx(0, 0)*internal_123;
    Q_xu_(0, 1) = Fu(1, 1)*internal_1 + Fu(2, 1)*internal_3 + Fu(3, 1)*internal_5 + Fx(0, 0)*internal_124;
    Q_xu_(0, 2) = Fu(5, 2)*internal_8 + Fu(6, 2)*internal_9 + Fu(7, 2)*internal_10 + Fu(9, 2)*internal_12;
    Q_xu_(1, 0) = Fu(0, 0)*internal_13 + Fu(1, 0)*internal_14 + Fu(2, 0)*internal_15 + Fu(3, 0)*internal_23 + Fu(4, 0)*internal_25;
    Q_xu_(1, 1) = Fu(0, 1)*internal_13 + Fu(1, 1)*internal_14 + Fu(2, 1)*internal_15 + Fu(3, 1)*internal_23;
    Q_xu_(1, 2) = Fu(5, 2)*internal_26 + Fu(6, 2)*internal_27 + Fu(7, 2)*internal_29 + Fu(9, 2)*internal_31;
    Q_xu_(1, 3) = g_u(22, 3)*internal_16;
    Q_xu_(1, 4) = g_u(23, 4)*internal_17;
    Q_xu_(1, 5) = g_u(24, 5)*internal_18;
    Q_xu_(1, 6) = g_u(26, 6)*internal_20;
    Q_xu_(1, 7) = g_u(25, 7)*internal_19;
    Q_xu_(1, 8) = g_u(27, 8)*internal_21;
    Q_xu_(2, 0) = Fu(0, 0)*internal_32 + Fu(1, 0)*internal_33 + Fu(2, 0)*internal_34 + Fu(3, 0)*internal_35 + Fu(4, 0)*internal_43;
    Q_xu_(2, 1) = Fu(0, 1)*internal_32 + Fu(1, 1)*internal_33 + Fu(2, 1)*internal_34 + Fu(3, 1)*internal_35;
    Q_xu_(2, 2) = Fu(5, 2)*internal_44 + Fu(6, 2)*internal_45 + Fu(7, 2)*internal_47 + Fu(9, 2)*internal_49;
    Q_xu_(2, 3) = g_u(22, 3)*internal_36;
    Q_xu_(2, 4) = g_u(23, 4)*internal_37;
    Q_xu_(2, 5) = g_u(24, 5)*internal_38;
    Q_xu_(2, 6) = g_u(26, 6)*internal_40;
    Q_xu_(2, 7) = g_u(25, 7)*internal_39;
    Q_xu_(2, 8) = g_u(27, 8)*internal_41;
    Q_xu_(3, 0) = Fu(0, 0)*internal_50 + Fu(1, 0)*internal_51 + Fu(2, 0)*internal_52 + Fu(3, 0)*internal_53 + Fu(4, 0)*internal_54;
    Q_xu_(3, 1) = Fu(0, 1)*internal_50 + Fu(1, 1)*internal_51 + Fu(2, 1)*internal_52 + Fu(3, 1)*internal_53;
    Q_xu_(3, 2) = Fu(5, 2)*internal_55 + Fu(6, 2)*internal_56 + Fu(7, 2)*internal_60 + Fu(9, 2)*internal_62;
    Q_xu_(3, 3) = g_u(22, 3)*internal_57;
    Q_xu_(3, 4) = g_u(23, 4)*internal_58;
    Q_xu_(3, 5) = Sigma_(24)*g_u(24, 5)*g_x(24, 3);
    Q_xu_(3, 6) = Sigma_(26)*g_u(26, 6)*g_x(26, 3);
    Q_xu_(3, 7) = Sigma_(25)*g_u(25, 7)*g_x(25, 3);
    Q_xu_(3, 8) = Sigma_(27)*g_u(27, 8)*g_x(27, 3);
    Q_xu_(4, 0) = Fu(0, 0)*internal_63 + Fu(1, 0)*internal_64 + Fu(2, 0)*internal_65 + Fu(3, 0)*internal_66 + Fu(4, 0)*internal_67;
    Q_xu_(4, 1) = Fu(0, 1)*internal_63 + Fu(1, 1)*internal_64 + Fu(2, 1)*internal_65 + Fu(3, 1)*internal_66;
    Q_xu_(4, 2) = Fu(5, 2)*internal_68 + Fu(6, 2)*internal_69 + Fu(7, 2)*internal_70 + Fu(9, 2)*internal_71;
    Q_xu_(5, 0) = Fu(0, 0)*internal_72 + Fu(1, 0)*internal_73 + Fu(2, 0)*internal_74 + Fu(3, 0)*internal_75 + Fu(4, 0)*internal_76;
    Q_xu_(5, 1) = Fu(0, 1)*internal_72 + Fu(1, 1)*internal_73 + Fu(2, 1)*internal_74 + Fu(3, 1)*internal_75;
    Q_xu_(5, 2) = Fu(6, 2)*internal_77 + Fu(7, 2)*internal_78 + Fu(9, 2)*internal_81 + Fx(5, 5)*internal_125;
    Q_xu_(6, 0) = Fu(0, 0)*internal_82 + Fu(1, 0)*internal_83 + Fu(2, 0)*internal_84 + Fu(3, 0)*internal_85 + Fu(4, 0)*internal_86;
    Q_xu_(6, 1) = Fu(0, 1)*internal_82 + Fu(1, 1)*internal_83 + Fu(2, 1)*internal_84 + Fu(3, 1)*internal_85;
    Q_xu_(6, 2) = Fu(5, 2)*internal_87 + Fu(7, 2)*internal_88 + Fu(9, 2)*internal_94 + Fx(6, 6)*internal_126;
    Q_xu_(6, 3) = g_u(22, 3)*internal_89;
    Q_xu_(6, 4) = g_u(23, 4)*internal_90;
    Q_xu_(7, 0) = Fu(0, 0)*internal_95 + Fu(1, 0)*internal_96 + Fu(2, 0)*internal_97 + Fu(3, 0)*internal_98 + Fu(4, 0)*internal_99;
    Q_xu_(7, 1) = Fu(0, 1)*internal_95 + Fu(1, 1)*internal_96 + Fu(2, 1)*internal_97 + Fu(3, 1)*internal_98;
    Q_xu_(7, 2) = Fu(5, 2)*internal_100 + Fu(6, 2)*internal_101 + Fu(9, 2)*internal_104 + Fx(7, 7)*internal_127;
    Q_xu_(7, 3) = Sigma_(22)*g_u(22, 3)*g_x(22, 7);
    Q_xu_(7, 4) = Sigma_(23)*g_u(23, 4)*g_x(23, 7);
    Q_xu_(8, 0) = Fu(0, 0)*internal_105 + Fu(1, 0)*internal_106 + Fu(2, 0)*internal_107 + Fu(3, 0)*internal_108 + Fu(4, 0)*internal_109;
    Q_xu_(8, 1) = Fu(0, 1)*internal_105 + Fu(1, 1)*internal_106 + Fu(2, 1)*internal_107 + Fu(3, 1)*internal_108;
    Q_xu_(8, 2) = Fu(5, 2)*internal_110 + Fu(6, 2)*internal_111 + Fu(7, 2)*internal_112 + Fu(9, 2)*internal_113;
    Q_xu_(9, 0) = Fu(0, 0)*internal_114 + Fu(1, 0)*internal_115 + Fu(2, 0)*internal_116 + Fu(3, 0)*internal_117 + Fu(4, 0)*internal_118;
    Q_xu_(9, 1) = Fu(0, 1)*internal_114 + Fu(1, 1)*internal_115 + Fu(2, 1)*internal_116 + Fu(3, 1)*internal_117;
    Q_xu_(9, 2) = Fu(5, 2)*internal_119 + Fu(6, 2)*internal_120 + Fu(7, 2)*internal_121 + Fu(9, 2)*internal_122;

    // Evaluation of Matrix Q_uu_
    Q_uu_(0, 0) = Fu(0, 0)*internal_128 + Fu(1, 0)*internal_129 + Fu(2, 0)*internal_130 + Fu(3, 0)*internal_131 + Fu(4, 0)*(Fu(0, 0)*V_xx_prev(0, 4) + Fu(1, 0)*V_xx_prev(1, 4) + Fu(2, 0)*V_xx_prev(2, 4) + Fu(3, 0)*V_xx_prev(3, 4) + Fu(4, 0)*V_xx_prev(4, 4)) + Sigma_(18)*pow(g_u(18, 0), 2) + Sigma_(19)*pow(g_u(19, 0), 2) + l_uu(0, 0);
    Q_uu_(0, 1) = Fu(0, 1)*internal_128 + Fu(1, 1)*internal_129 + Fu(2, 1)*internal_130 + Fu(3, 1)*internal_131;
    Q_uu_(0, 2) = Fu(5, 2)*(Fu(0, 0)*V_xx_prev(0, 5) + Fu(1, 0)*V_xx_prev(1, 5) + Fu(2, 0)*V_xx_prev(2, 5) + Fu(3, 0)*V_xx_prev(3, 5) + Fu(4, 0)*V_xx_prev(4, 5)) + Fu(6, 2)*(Fu(0, 0)*V_xx_prev(0, 6) + Fu(1, 0)*V_xx_prev(1, 6) + Fu(2, 0)*V_xx_prev(2, 6) + Fu(3, 0)*V_xx_prev(3, 6) + Fu(4, 0)*V_xx_prev(4, 6)) + Fu(7, 2)*(Fu(0, 0)*V_xx_prev(0, 7) + Fu(1, 0)*V_xx_prev(1, 7) + Fu(2, 0)*V_xx_prev(2, 7) + Fu(3, 0)*V_xx_prev(3, 7) + Fu(4, 0)*V_xx_prev(4, 7)) + Fu(9, 2)*(Fu(0, 0)*V_xx_prev(0, 9) + Fu(1, 0)*V_xx_prev(1, 9) + Fu(2, 0)*V_xx_prev(2, 9) + Fu(3, 0)*V_xx_prev(3, 9) + Fu(4, 0)*V_xx_prev(4, 9));
    Q_uu_(1, 0) = Fu(0, 0)*internal_132 + Fu(1, 0)*internal_133 + Fu(2, 0)*internal_134 + Fu(3, 0)*internal_135 + Fu(4, 0)*(Fu(0, 1)*V_xx_prev(0, 4) + Fu(1, 1)*V_xx_prev(1, 4) + Fu(2, 1)*V_xx_prev(2, 4) + Fu(3, 1)*V_xx_prev(3, 4));
    Q_uu_(1, 1) = Fu(0, 1)*internal_132 + Fu(1, 1)*internal_133 + Fu(2, 1)*internal_134 + Fu(3, 1)*internal_135 + Sigma_(16)*pow(g_u(16, 1), 2) + Sigma_(17)*pow(g_u(17, 1), 2) + l_uu(1, 1);
    Q_uu_(1, 2) = Fu(5, 2)*(Fu(0, 1)*V_xx_prev(0, 5) + Fu(1, 1)*V_xx_prev(1, 5) + Fu(2, 1)*V_xx_prev(2, 5) + Fu(3, 1)*V_xx_prev(3, 5)) + Fu(6, 2)*(Fu(0, 1)*V_xx_prev(0, 6) + Fu(1, 1)*V_xx_prev(1, 6) + Fu(2, 1)*V_xx_prev(2, 6) + Fu(3, 1)*V_xx_prev(3, 6)) + Fu(7, 2)*(Fu(0, 1)*V_xx_prev(0, 7) + Fu(1, 1)*V_xx_prev(1, 7) + Fu(2, 1)*V_xx_prev(2, 7) + Fu(3, 1)*V_xx_prev(3, 7)) + Fu(9, 2)*(Fu(0, 1)*V_xx_prev(0, 9) + Fu(1, 1)*V_xx_prev(1, 9) + Fu(2, 1)*V_xx_prev(2, 9) + Fu(3, 1)*V_xx_prev(3, 9));
    Q_uu_(2, 0) = Fu(0, 0)*internal_136 + Fu(1, 0)*internal_137 + Fu(2, 0)*internal_138 + Fu(3, 0)*internal_139 + Fu(4, 0)*(Fu(5, 2)*V_xx_prev(5, 4) + Fu(6, 2)*V_xx_prev(6, 4) + Fu(7, 2)*V_xx_prev(7, 4) + Fu(9, 2)*V_xx_prev(9, 4));
    Q_uu_(2, 1) = Fu(0, 1)*internal_136 + Fu(1, 1)*internal_137 + Fu(2, 1)*internal_138 + Fu(3, 1)*internal_139;
    Q_uu_(2, 2) = Fu(5, 2)*(Fu(6, 2)*V_xx_prev(6, 5) + Fu(7, 2)*V_xx_prev(7, 5) + Fu(9, 2)*V_xx_prev(9, 5) + internal_125) + Fu(6, 2)*(Fu(5, 2)*V_xx_prev(5, 6) + Fu(7, 2)*V_xx_prev(7, 6) + Fu(9, 2)*V_xx_prev(9, 6) + internal_126) + Fu(7, 2)*(Fu(5, 2)*V_xx_prev(5, 7) + Fu(6, 2)*V_xx_prev(6, 7) + Fu(9, 2)*V_xx_prev(9, 7) + internal_127) + Fu(9, 2)*(Fu(5, 2)*V_xx_prev(5, 9) + Fu(6, 2)*V_xx_prev(6, 9) + Fu(7, 2)*V_xx_prev(7, 9) + Fu(9, 2)*V_xx_prev(9, 9)) + Sigma_(20)*pow(g_u(20, 2), 2) + Sigma_(21)*pow(g_u(21, 2), 2) + l_uu(2, 2);
    Q_uu_(3, 3) = Sigma_(10)*pow(g_u(10, 3), 2) + Sigma_(22)*pow(g_u(22, 3), 2);
    Q_uu_(4, 4) = Sigma_(11)*pow(g_u(11, 4), 2) + Sigma_(23)*pow(g_u(23, 4), 2);
    Q_uu_(5, 5) = Sigma_(12)*pow(g_u(12, 5), 2) + Sigma_(24)*pow(g_u(24, 5), 2);
    Q_uu_(6, 6) = Sigma_(13)*pow(g_u(13, 6), 2) + Sigma_(26)*pow(g_u(26, 6), 2);
    Q_uu_(7, 7) = Sigma_(14)*pow(g_u(14, 7), 2) + Sigma_(25)*pow(g_u(25, 7), 2);
    Q_uu_(8, 8) = Sigma_(15)*pow(g_u(15, 8), 2) + Sigma_(27)*pow(g_u(27, 8), 2);

  }

  virtual void updateQG(const Eigen::VectorXd& sl_x, const Eigen::VectorXd& sl_u, const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::VectorXd& r_f, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& V_xx_prev, const Eigen::VectorXd& V_x_prev) override {
    // Determine internal variables
    const double internal_0 = V_x_prev(0) + V_xx_prev(0, 0)*r_f(0) + V_xx_prev(0, 1)*r_f(1) + V_xx_prev(0, 2)*r_f(2) + V_xx_prev(0, 3)*r_f(3) + V_xx_prev(0, 4)*r_f(4) + V_xx_prev(0, 5)*r_f(5) + V_xx_prev(0, 6)*r_f(6) + V_xx_prev(0, 7)*r_f(7) + V_xx_prev(0, 8)*r_f(8) + V_xx_prev(0, 9)*r_f(9);
    const double internal_1 = V_x_prev(1) + V_xx_prev(1, 0)*r_f(0) + V_xx_prev(1, 1)*r_f(1) + V_xx_prev(1, 2)*r_f(2) + V_xx_prev(1, 3)*r_f(3) + V_xx_prev(1, 4)*r_f(4) + V_xx_prev(1, 5)*r_f(5) + V_xx_prev(1, 6)*r_f(6) + V_xx_prev(1, 7)*r_f(7) + V_xx_prev(1, 8)*r_f(8) + V_xx_prev(1, 9)*r_f(9);
    const double internal_2 = V_x_prev(2) + V_xx_prev(2, 0)*r_f(0) + V_xx_prev(2, 1)*r_f(1) + V_xx_prev(2, 2)*r_f(2) + V_xx_prev(2, 3)*r_f(3) + V_xx_prev(2, 4)*r_f(4) + V_xx_prev(2, 5)*r_f(5) + V_xx_prev(2, 6)*r_f(6) + V_xx_prev(2, 7)*r_f(7) + V_xx_prev(2, 8)*r_f(8) + V_xx_prev(2, 9)*r_f(9);
    const double internal_3 = V_x_prev(3) + V_xx_prev(3, 0)*r_f(0) + V_xx_prev(3, 1)*r_f(1) + V_xx_prev(3, 2)*r_f(2) + V_xx_prev(3, 3)*r_f(3) + V_xx_prev(3, 4)*r_f(4) + V_xx_prev(3, 5)*r_f(5) + V_xx_prev(3, 6)*r_f(6) + V_xx_prev(3, 7)*r_f(7) + V_xx_prev(3, 8)*r_f(8) + V_xx_prev(3, 9)*r_f(9);
    const double internal_4 = V_x_prev(4) + V_xx_prev(4, 0)*r_f(0) + V_xx_prev(4, 1)*r_f(1) + V_xx_prev(4, 2)*r_f(2) + V_xx_prev(4, 3)*r_f(3) + V_xx_prev(4, 4)*r_f(4) + V_xx_prev(4, 5)*r_f(5) + V_xx_prev(4, 6)*r_f(6) + V_xx_prev(4, 7)*r_f(7) + V_xx_prev(4, 8)*r_f(8) + V_xx_prev(4, 9)*r_f(9);
    const double internal_5 = V_x_prev(5) + V_xx_prev(5, 0)*r_f(0) + V_xx_prev(5, 1)*r_f(1) + V_xx_prev(5, 2)*r_f(2) + V_xx_prev(5, 3)*r_f(3) + V_xx_prev(5, 4)*r_f(4) + V_xx_prev(5, 5)*r_f(5) + V_xx_prev(5, 6)*r_f(6) + V_xx_prev(5, 7)*r_f(7) + V_xx_prev(5, 8)*r_f(8) + V_xx_prev(5, 9)*r_f(9);
    const double internal_6 = V_x_prev(6) + V_xx_prev(6, 0)*r_f(0) + V_xx_prev(6, 1)*r_f(1) + V_xx_prev(6, 2)*r_f(2) + V_xx_prev(6, 3)*r_f(3) + V_xx_prev(6, 4)*r_f(4) + V_xx_prev(6, 5)*r_f(5) + V_xx_prev(6, 6)*r_f(6) + V_xx_prev(6, 7)*r_f(7) + V_xx_prev(6, 8)*r_f(8) + V_xx_prev(6, 9)*r_f(9);
    const double internal_7 = V_x_prev(7) + V_xx_prev(7, 0)*r_f(0) + V_xx_prev(7, 1)*r_f(1) + V_xx_prev(7, 2)*r_f(2) + V_xx_prev(7, 3)*r_f(3) + V_xx_prev(7, 4)*r_f(4) + V_xx_prev(7, 5)*r_f(5) + V_xx_prev(7, 6)*r_f(6) + V_xx_prev(7, 7)*r_f(7) + V_xx_prev(7, 8)*r_f(8) + V_xx_prev(7, 9)*r_f(9);
    const double internal_8 = V_x_prev(9) + V_xx_prev(9, 0)*r_f(0) + V_xx_prev(9, 1)*r_f(1) + V_xx_prev(9, 2)*r_f(2) + V_xx_prev(9, 3)*r_f(3) + V_xx_prev(9, 4)*r_f(4) + V_xx_prev(9, 5)*r_f(5) + V_xx_prev(9, 6)*r_f(6) + V_xx_prev(9, 7)*r_f(7) + V_xx_prev(9, 8)*r_f(8) + V_xx_prev(9, 9)*r_f(9);

    // Evaluation of Vector Q_x_
    Q_x_(0) = Fx(0, 0)*internal_0 + V_(2)*g_x(2, 0) + V_(3)*g_x(3, 0) + sl_x(0);
    Q_x_(1) = Fx(0, 1)*internal_0 + Fx(1, 1)*internal_1 + V_(0)*g_x(0, 1) + V_(1)*g_x(1, 1) + V_(22)*g_x(22, 1) + V_(23)*g_x(23, 1) + V_(24)*g_x(24, 1) + V_(25)*g_x(25, 1) + V_(26)*g_x(26, 1) + V_(27)*g_x(27, 1) + V_(8)*g_x(8, 1) + sl_x(1);
    Q_x_(2) = Fx(0, 2)*internal_0 + Fx(2, 2)*internal_2 + V_(0)*g_x(0, 2) + V_(1)*g_x(1, 2) + V_(22)*g_x(22, 2) + V_(23)*g_x(23, 2) + V_(24)*g_x(24, 2) + V_(25)*g_x(25, 2) + V_(26)*g_x(26, 2) + V_(27)*g_x(27, 2) + V_(8)*g_x(8, 2) + sl_x(2);
    Q_x_(3) = Fx(0, 3)*internal_0 + Fx(1, 3)*internal_1 + Fx(2, 3)*internal_2 + Fx(3, 3)*internal_3 + V_(22)*g_x(22, 3) + V_(23)*g_x(23, 3) + V_(24)*g_x(24, 3) + V_(25)*g_x(25, 3) + V_(26)*g_x(26, 3) + V_(27)*g_x(27, 3) + V_(9)*g_x(9, 3) + sl_x(3);
    Q_x_(4) = Fx(0, 4)*internal_0 + Fx(1, 4)*internal_1 + Fx(2, 4)*internal_2 + Fx(3, 4)*internal_3 + Fx(4, 4)*internal_4 + V_(5)*g_x(5, 4) + V_(6)*g_x(6, 4) + sl_x(4);
    Q_x_(5) = Fx(5, 5)*internal_5 + V_(4)*g_x(4, 5) + sl_x(5);
    Q_x_(6) = Fx(6, 6)*internal_6 + V_(22)*g_x(22, 6) + V_(23)*g_x(23, 6) + sl_x(6);
    Q_x_(7) = Fx(7, 7)*internal_7 + V_(22)*g_x(22, 7) + V_(23)*g_x(23, 7) + sl_x(7);
    Q_x_(8) = Fx(8, 8)*(V_x_prev(8) + V_xx_prev(8, 0)*r_f(0) + V_xx_prev(8, 1)*r_f(1) + V_xx_prev(8, 2)*r_f(2) + V_xx_prev(8, 3)*r_f(3) + V_xx_prev(8, 4)*r_f(4) + V_xx_prev(8, 5)*r_f(5) + V_xx_prev(8, 6)*r_f(6) + V_xx_prev(8, 7)*r_f(7) + V_xx_prev(8, 8)*r_f(8) + V_xx_prev(8, 9)*r_f(9)) + sl_x(8);
    Q_x_(9) = Fx(5, 9)*internal_5 + Fx(6, 9)*internal_6 + Fx(7, 9)*internal_7 + Fx(9, 9)*internal_8 + V_(7)*g_x(7, 9) + sl_x(9);

    // Evaluation of Vector Q_u_
    Q_u_(0) = Fu(0, 0)*internal_0 + Fu(1, 0)*internal_1 + Fu(2, 0)*internal_2 + Fu(3, 0)*internal_3 + Fu(4, 0)*internal_4 + V_(18)*g_u(18, 0) + V_(19)*g_u(19, 0) + sl_u(0);
    Q_u_(1) = Fu(0, 1)*internal_0 + Fu(1, 1)*internal_1 + Fu(2, 1)*internal_2 + Fu(3, 1)*internal_3 + V_(16)*g_u(16, 1) + V_(17)*g_u(17, 1) + sl_u(1);
    Q_u_(2) = Fu(5, 2)*internal_5 + Fu(6, 2)*internal_6 + Fu(7, 2)*internal_7 + Fu(9, 2)*internal_8 + V_(20)*g_u(20, 2) + V_(21)*g_u(21, 2) + sl_u(2);
    Q_u_(3) = V_(10)*g_u(10, 3) + V_(22)*g_u(22, 3) + sl_u(3);
    Q_u_(4) = V_(11)*g_u(11, 4) + V_(23)*g_u(23, 4) + sl_u(4);
    Q_u_(5) = V_(12)*g_u(12, 5) + V_(24)*g_u(24, 5) + sl_u(5);
    Q_u_(6) = V_(13)*g_u(13, 6) + V_(26)*g_u(26, 6) + sl_u(6);
    Q_u_(7) = V_(14)*g_u(14, 7) + V_(25)*g_u(25, 7) + sl_u(7);
    Q_u_(8) = V_(15)*g_u(15, 8) + V_(27)*g_u(27, 8) + sl_u(8);

  }

  virtual void updateQuuInv() override {
    // Determine internal variables
    const double internal_0 = 1.0/Q_uu_(0, 0);
    const double internal_3 = Q_uu_(0, 1)*internal_0;
    const double internal_5 = Q_uu_(0, 2)*internal_0;
    const double internal_1 = -pow(Q_uu_(0, 1), 2)*internal_0 + Q_uu_(1, 1);
    const double internal_2 = 1.0/internal_1;
    const double internal_6 = -Q_uu_(0, 1)*internal_5 + Q_uu_(1, 2);
    const double internal_4 = internal_2*internal_3;
    const double internal_7 = pow(internal_6, 2);
    const double internal_9 = internal_4*internal_6;
    const double internal_8 = 1.0/(-pow(Q_uu_(0, 2), 2)*internal_0 + Q_uu_(2, 2) - internal_2*internal_7);
    const double internal_10 = internal_8*(-internal_5 + internal_9);
    const double internal_13 = -internal_2*internal_6*internal_8;
    const double internal_12 = internal_2 + internal_7*internal_8/pow(internal_1, 2);
    const double internal_11 = -internal_10*internal_2*internal_6 - internal_4;

    // Evaluation of Matrix Q_uu_inv_
    Q_uu_inv_(0, 0) = internal_0 - internal_10*internal_5 - internal_11*internal_3;
    Q_uu_inv_(0, 1) = Q_uu_(0, 2)*internal_0*internal_2*internal_6*internal_8 - internal_12*internal_3;
    Q_uu_inv_(0, 2) = -internal_5*internal_8 + internal_8*internal_9;
    Q_uu_inv_(1, 0) = internal_11;
    Q_uu_inv_(1, 1) = internal_12;
    Q_uu_inv_(1, 2) = internal_13;
    Q_uu_inv_(2, 0) = internal_10;
    Q_uu_inv_(2, 1) = internal_13;
    Q_uu_inv_(2, 2) = internal_8;
    Q_uu_inv_(3, 3) = 1.0/Q_uu_(3, 3);
    Q_uu_inv_(4, 4) = 1.0/Q_uu_(4, 4);
    Q_uu_inv_(5, 5) = 1.0/Q_uu_(5, 5);
    Q_uu_inv_(6, 6) = 1.0/Q_uu_(6, 6);
    Q_uu_inv_(7, 7) = 1.0/Q_uu_(7, 7);
    Q_uu_inv_(8, 8) = 1.0/Q_uu_(8, 8);

  }

  virtual void updateKVU(Eigen::MatrixXd& k_ux, Eigen::VectorXd& v_u) override {
    // Evaluation of Matrix k_ux
    k_ux(0, 0) = -Q_uu_inv_(0, 0)*Q_xu_(0, 0) - Q_uu_inv_(0, 1)*Q_xu_(0, 1) - Q_uu_inv_(0, 2)*Q_xu_(0, 2);
    k_ux(0, 1) = -Q_uu_inv_(0, 0)*Q_xu_(1, 0) - Q_uu_inv_(0, 1)*Q_xu_(1, 1) - Q_uu_inv_(0, 2)*Q_xu_(1, 2);
    k_ux(0, 2) = -Q_uu_inv_(0, 0)*Q_xu_(2, 0) - Q_uu_inv_(0, 1)*Q_xu_(2, 1) - Q_uu_inv_(0, 2)*Q_xu_(2, 2);
    k_ux(0, 3) = -Q_uu_inv_(0, 0)*Q_xu_(3, 0) - Q_uu_inv_(0, 1)*Q_xu_(3, 1) - Q_uu_inv_(0, 2)*Q_xu_(3, 2);
    k_ux(0, 4) = -Q_uu_inv_(0, 0)*Q_xu_(4, 0) - Q_uu_inv_(0, 1)*Q_xu_(4, 1) - Q_uu_inv_(0, 2)*Q_xu_(4, 2);
    k_ux(0, 5) = -Q_uu_inv_(0, 0)*Q_xu_(5, 0) - Q_uu_inv_(0, 1)*Q_xu_(5, 1) - Q_uu_inv_(0, 2)*Q_xu_(5, 2);
    k_ux(0, 6) = -Q_uu_inv_(0, 0)*Q_xu_(6, 0) - Q_uu_inv_(0, 1)*Q_xu_(6, 1) - Q_uu_inv_(0, 2)*Q_xu_(6, 2);
    k_ux(0, 7) = -Q_uu_inv_(0, 0)*Q_xu_(7, 0) - Q_uu_inv_(0, 1)*Q_xu_(7, 1) - Q_uu_inv_(0, 2)*Q_xu_(7, 2);
    k_ux(0, 8) = -Q_uu_inv_(0, 0)*Q_xu_(8, 0) - Q_uu_inv_(0, 1)*Q_xu_(8, 1) - Q_uu_inv_(0, 2)*Q_xu_(8, 2);
    k_ux(0, 9) = -Q_uu_inv_(0, 0)*Q_xu_(9, 0) - Q_uu_inv_(0, 1)*Q_xu_(9, 1) - Q_uu_inv_(0, 2)*Q_xu_(9, 2);
    k_ux(1, 0) = -Q_uu_inv_(0, 1)*Q_xu_(0, 0) - Q_uu_inv_(1, 1)*Q_xu_(0, 1) - Q_uu_inv_(1, 2)*Q_xu_(0, 2);
    k_ux(1, 1) = -Q_uu_inv_(0, 1)*Q_xu_(1, 0) - Q_uu_inv_(1, 1)*Q_xu_(1, 1) - Q_uu_inv_(1, 2)*Q_xu_(1, 2);
    k_ux(1, 2) = -Q_uu_inv_(0, 1)*Q_xu_(2, 0) - Q_uu_inv_(1, 1)*Q_xu_(2, 1) - Q_uu_inv_(1, 2)*Q_xu_(2, 2);
    k_ux(1, 3) = -Q_uu_inv_(0, 1)*Q_xu_(3, 0) - Q_uu_inv_(1, 1)*Q_xu_(3, 1) - Q_uu_inv_(1, 2)*Q_xu_(3, 2);
    k_ux(1, 4) = -Q_uu_inv_(0, 1)*Q_xu_(4, 0) - Q_uu_inv_(1, 1)*Q_xu_(4, 1) - Q_uu_inv_(1, 2)*Q_xu_(4, 2);
    k_ux(1, 5) = -Q_uu_inv_(0, 1)*Q_xu_(5, 0) - Q_uu_inv_(1, 1)*Q_xu_(5, 1) - Q_uu_inv_(1, 2)*Q_xu_(5, 2);
    k_ux(1, 6) = -Q_uu_inv_(0, 1)*Q_xu_(6, 0) - Q_uu_inv_(1, 1)*Q_xu_(6, 1) - Q_uu_inv_(1, 2)*Q_xu_(6, 2);
    k_ux(1, 7) = -Q_uu_inv_(0, 1)*Q_xu_(7, 0) - Q_uu_inv_(1, 1)*Q_xu_(7, 1) - Q_uu_inv_(1, 2)*Q_xu_(7, 2);
    k_ux(1, 8) = -Q_uu_inv_(0, 1)*Q_xu_(8, 0) - Q_uu_inv_(1, 1)*Q_xu_(8, 1) - Q_uu_inv_(1, 2)*Q_xu_(8, 2);
    k_ux(1, 9) = -Q_uu_inv_(0, 1)*Q_xu_(9, 0) - Q_uu_inv_(1, 1)*Q_xu_(9, 1) - Q_uu_inv_(1, 2)*Q_xu_(9, 2);
    k_ux(2, 0) = -Q_uu_inv_(0, 2)*Q_xu_(0, 0) - Q_uu_inv_(1, 2)*Q_xu_(0, 1) - Q_uu_inv_(2, 2)*Q_xu_(0, 2);
    k_ux(2, 1) = -Q_uu_inv_(0, 2)*Q_xu_(1, 0) - Q_uu_inv_(1, 2)*Q_xu_(1, 1) - Q_uu_inv_(2, 2)*Q_xu_(1, 2);
    k_ux(2, 2) = -Q_uu_inv_(0, 2)*Q_xu_(2, 0) - Q_uu_inv_(1, 2)*Q_xu_(2, 1) - Q_uu_inv_(2, 2)*Q_xu_(2, 2);
    k_ux(2, 3) = -Q_uu_inv_(0, 2)*Q_xu_(3, 0) - Q_uu_inv_(1, 2)*Q_xu_(3, 1) - Q_uu_inv_(2, 2)*Q_xu_(3, 2);
    k_ux(2, 4) = -Q_uu_inv_(0, 2)*Q_xu_(4, 0) - Q_uu_inv_(1, 2)*Q_xu_(4, 1) - Q_uu_inv_(2, 2)*Q_xu_(4, 2);
    k_ux(2, 5) = -Q_uu_inv_(0, 2)*Q_xu_(5, 0) - Q_uu_inv_(1, 2)*Q_xu_(5, 1) - Q_uu_inv_(2, 2)*Q_xu_(5, 2);
    k_ux(2, 6) = -Q_uu_inv_(0, 2)*Q_xu_(6, 0) - Q_uu_inv_(1, 2)*Q_xu_(6, 1) - Q_uu_inv_(2, 2)*Q_xu_(6, 2);
    k_ux(2, 7) = -Q_uu_inv_(0, 2)*Q_xu_(7, 0) - Q_uu_inv_(1, 2)*Q_xu_(7, 1) - Q_uu_inv_(2, 2)*Q_xu_(7, 2);
    k_ux(2, 8) = -Q_uu_inv_(0, 2)*Q_xu_(8, 0) - Q_uu_inv_(1, 2)*Q_xu_(8, 1) - Q_uu_inv_(2, 2)*Q_xu_(8, 2);
    k_ux(2, 9) = -Q_uu_inv_(0, 2)*Q_xu_(9, 0) - Q_uu_inv_(1, 2)*Q_xu_(9, 1) - Q_uu_inv_(2, 2)*Q_xu_(9, 2);
    k_ux(3, 1) = -Q_uu_inv_(3, 3)*Q_xu_(1, 3);
    k_ux(3, 2) = -Q_uu_inv_(3, 3)*Q_xu_(2, 3);
    k_ux(3, 3) = -Q_uu_inv_(3, 3)*Q_xu_(3, 3);
    k_ux(3, 6) = -Q_uu_inv_(3, 3)*Q_xu_(6, 3);
    k_ux(3, 7) = -Q_uu_inv_(3, 3)*Q_xu_(7, 3);
    k_ux(4, 1) = -Q_uu_inv_(4, 4)*Q_xu_(1, 4);
    k_ux(4, 2) = -Q_uu_inv_(4, 4)*Q_xu_(2, 4);
    k_ux(4, 3) = -Q_uu_inv_(4, 4)*Q_xu_(3, 4);
    k_ux(4, 6) = -Q_uu_inv_(4, 4)*Q_xu_(6, 4);
    k_ux(4, 7) = -Q_uu_inv_(4, 4)*Q_xu_(7, 4);
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

    // Evaluation of Vector v_u
    v_u(0) = -Q_u_(0)*Q_uu_inv_(0, 0) - Q_u_(1)*Q_uu_inv_(0, 1) - Q_u_(2)*Q_uu_inv_(0, 2);
    v_u(1) = -Q_u_(0)*Q_uu_inv_(0, 1) - Q_u_(1)*Q_uu_inv_(1, 1) - Q_u_(2)*Q_uu_inv_(1, 2);
    v_u(2) = -Q_u_(0)*Q_uu_inv_(0, 2) - Q_u_(1)*Q_uu_inv_(1, 2) - Q_u_(2)*Q_uu_inv_(2, 2);
    v_u(3) = -Q_u_(3)*Q_uu_inv_(3, 3);
    v_u(4) = -Q_u_(4)*Q_uu_inv_(4, 4);
    v_u(5) = -Q_u_(5)*Q_uu_inv_(5, 5);
    v_u(6) = -Q_u_(6)*Q_uu_inv_(6, 6);
    v_u(7) = -Q_u_(7)*Q_uu_inv_(7, 7);
    v_u(8) = -Q_u_(8)*Q_uu_inv_(8, 8);

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
    V_xx(0, 0) = Q_xu_(0, 0)*k_ux(0, 0) + Q_xu_(0, 1)*k_ux(1, 0) + Q_xu_(0, 2)*k_ux(2, 0) + Q_xx_(0, 0);
    V_xx(0, 1) = Q_xu_(0, 0)*k_ux(0, 1) + Q_xu_(0, 1)*k_ux(1, 1) + Q_xu_(0, 2)*k_ux(2, 1) + Q_xx_(0, 1);
    V_xx(0, 2) = Q_xu_(0, 0)*k_ux(0, 2) + Q_xu_(0, 1)*k_ux(1, 2) + Q_xu_(0, 2)*k_ux(2, 2) + Q_xx_(0, 2);
    V_xx(0, 3) = Q_xu_(0, 0)*k_ux(0, 3) + Q_xu_(0, 1)*k_ux(1, 3) + Q_xu_(0, 2)*k_ux(2, 3) + Q_xx_(0, 3);
    V_xx(0, 4) = Q_xu_(0, 0)*k_ux(0, 4) + Q_xu_(0, 1)*k_ux(1, 4) + Q_xu_(0, 2)*k_ux(2, 4) + Q_xx_(0, 4);
    V_xx(0, 5) = Q_xu_(0, 0)*k_ux(0, 5) + Q_xu_(0, 1)*k_ux(1, 5) + Q_xu_(0, 2)*k_ux(2, 5) + Q_xx_(0, 5);
    V_xx(0, 6) = Q_xu_(0, 0)*k_ux(0, 6) + Q_xu_(0, 1)*k_ux(1, 6) + Q_xu_(0, 2)*k_ux(2, 6) + Q_xx_(0, 6);
    V_xx(0, 7) = Q_xu_(0, 0)*k_ux(0, 7) + Q_xu_(0, 1)*k_ux(1, 7) + Q_xu_(0, 2)*k_ux(2, 7) + Q_xx_(0, 7);
    V_xx(0, 8) = Q_xu_(0, 0)*k_ux(0, 8) + Q_xu_(0, 1)*k_ux(1, 8) + Q_xu_(0, 2)*k_ux(2, 8) + Q_xx_(0, 8);
    V_xx(0, 9) = Q_xu_(0, 0)*k_ux(0, 9) + Q_xu_(0, 1)*k_ux(1, 9) + Q_xu_(0, 2)*k_ux(2, 9) + Q_xx_(0, 9);
    V_xx(1, 0) = Q_xu_(1, 0)*k_ux(0, 0) + Q_xu_(1, 1)*k_ux(1, 0) + Q_xu_(1, 2)*k_ux(2, 0) + Q_xx_(0, 1);
    V_xx(1, 1) = Q_xu_(1, 0)*k_ux(0, 1) + Q_xu_(1, 1)*k_ux(1, 1) + Q_xu_(1, 2)*k_ux(2, 1) + Q_xu_(1, 3)*k_ux(3, 1) + Q_xu_(1, 4)*k_ux(4, 1) + Q_xu_(1, 5)*k_ux(5, 1) + Q_xu_(1, 6)*k_ux(6, 1) + Q_xu_(1, 7)*k_ux(7, 1) + Q_xu_(1, 8)*k_ux(8, 1) + Q_xx_(1, 1);
    V_xx(1, 2) = Q_xu_(1, 0)*k_ux(0, 2) + Q_xu_(1, 1)*k_ux(1, 2) + Q_xu_(1, 2)*k_ux(2, 2) + Q_xu_(1, 3)*k_ux(3, 2) + Q_xu_(1, 4)*k_ux(4, 2) + Q_xu_(1, 5)*k_ux(5, 2) + Q_xu_(1, 6)*k_ux(6, 2) + Q_xu_(1, 7)*k_ux(7, 2) + Q_xu_(1, 8)*k_ux(8, 2) + Q_xx_(1, 2);
    V_xx(1, 3) = Q_xu_(1, 0)*k_ux(0, 3) + Q_xu_(1, 1)*k_ux(1, 3) + Q_xu_(1, 2)*k_ux(2, 3) + Q_xu_(1, 3)*k_ux(3, 3) + Q_xu_(1, 4)*k_ux(4, 3) + Q_xu_(1, 5)*k_ux(5, 3) + Q_xu_(1, 6)*k_ux(6, 3) + Q_xu_(1, 7)*k_ux(7, 3) + Q_xu_(1, 8)*k_ux(8, 3) + Q_xx_(1, 3);
    V_xx(1, 4) = Q_xu_(1, 0)*k_ux(0, 4) + Q_xu_(1, 1)*k_ux(1, 4) + Q_xu_(1, 2)*k_ux(2, 4) + Q_xx_(1, 4);
    V_xx(1, 5) = Q_xu_(1, 0)*k_ux(0, 5) + Q_xu_(1, 1)*k_ux(1, 5) + Q_xu_(1, 2)*k_ux(2, 5) + Q_xx_(1, 5);
    V_xx(1, 6) = Q_xu_(1, 0)*k_ux(0, 6) + Q_xu_(1, 1)*k_ux(1, 6) + Q_xu_(1, 2)*k_ux(2, 6) + Q_xu_(1, 3)*k_ux(3, 6) + Q_xu_(1, 4)*k_ux(4, 6) + Q_xx_(1, 6);
    V_xx(1, 7) = Q_xu_(1, 0)*k_ux(0, 7) + Q_xu_(1, 1)*k_ux(1, 7) + Q_xu_(1, 2)*k_ux(2, 7) + Q_xu_(1, 3)*k_ux(3, 7) + Q_xu_(1, 4)*k_ux(4, 7) + Q_xx_(1, 7);
    V_xx(1, 8) = Q_xu_(1, 0)*k_ux(0, 8) + Q_xu_(1, 1)*k_ux(1, 8) + Q_xu_(1, 2)*k_ux(2, 8) + Q_xx_(1, 8);
    V_xx(1, 9) = Q_xu_(1, 0)*k_ux(0, 9) + Q_xu_(1, 1)*k_ux(1, 9) + Q_xu_(1, 2)*k_ux(2, 9) + Q_xx_(1, 9);
    V_xx(2, 0) = Q_xu_(2, 0)*k_ux(0, 0) + Q_xu_(2, 1)*k_ux(1, 0) + Q_xu_(2, 2)*k_ux(2, 0) + Q_xx_(0, 2);
    V_xx(2, 1) = Q_xu_(2, 0)*k_ux(0, 1) + Q_xu_(2, 1)*k_ux(1, 1) + Q_xu_(2, 2)*k_ux(2, 1) + Q_xu_(2, 3)*k_ux(3, 1) + Q_xu_(2, 4)*k_ux(4, 1) + Q_xu_(2, 5)*k_ux(5, 1) + Q_xu_(2, 6)*k_ux(6, 1) + Q_xu_(2, 7)*k_ux(7, 1) + Q_xu_(2, 8)*k_ux(8, 1) + Q_xx_(1, 2);
    V_xx(2, 2) = Q_xu_(2, 0)*k_ux(0, 2) + Q_xu_(2, 1)*k_ux(1, 2) + Q_xu_(2, 2)*k_ux(2, 2) + Q_xu_(2, 3)*k_ux(3, 2) + Q_xu_(2, 4)*k_ux(4, 2) + Q_xu_(2, 5)*k_ux(5, 2) + Q_xu_(2, 6)*k_ux(6, 2) + Q_xu_(2, 7)*k_ux(7, 2) + Q_xu_(2, 8)*k_ux(8, 2) + Q_xx_(2, 2);
    V_xx(2, 3) = Q_xu_(2, 0)*k_ux(0, 3) + Q_xu_(2, 1)*k_ux(1, 3) + Q_xu_(2, 2)*k_ux(2, 3) + Q_xu_(2, 3)*k_ux(3, 3) + Q_xu_(2, 4)*k_ux(4, 3) + Q_xu_(2, 5)*k_ux(5, 3) + Q_xu_(2, 6)*k_ux(6, 3) + Q_xu_(2, 7)*k_ux(7, 3) + Q_xu_(2, 8)*k_ux(8, 3) + Q_xx_(2, 3);
    V_xx(2, 4) = Q_xu_(2, 0)*k_ux(0, 4) + Q_xu_(2, 1)*k_ux(1, 4) + Q_xu_(2, 2)*k_ux(2, 4) + Q_xx_(2, 4);
    V_xx(2, 5) = Q_xu_(2, 0)*k_ux(0, 5) + Q_xu_(2, 1)*k_ux(1, 5) + Q_xu_(2, 2)*k_ux(2, 5) + Q_xx_(2, 5);
    V_xx(2, 6) = Q_xu_(2, 0)*k_ux(0, 6) + Q_xu_(2, 1)*k_ux(1, 6) + Q_xu_(2, 2)*k_ux(2, 6) + Q_xu_(2, 3)*k_ux(3, 6) + Q_xu_(2, 4)*k_ux(4, 6) + Q_xx_(2, 6);
    V_xx(2, 7) = Q_xu_(2, 0)*k_ux(0, 7) + Q_xu_(2, 1)*k_ux(1, 7) + Q_xu_(2, 2)*k_ux(2, 7) + Q_xu_(2, 3)*k_ux(3, 7) + Q_xu_(2, 4)*k_ux(4, 7) + Q_xx_(2, 7);
    V_xx(2, 8) = Q_xu_(2, 0)*k_ux(0, 8) + Q_xu_(2, 1)*k_ux(1, 8) + Q_xu_(2, 2)*k_ux(2, 8) + Q_xx_(2, 8);
    V_xx(2, 9) = Q_xu_(2, 0)*k_ux(0, 9) + Q_xu_(2, 1)*k_ux(1, 9) + Q_xu_(2, 2)*k_ux(2, 9) + Q_xx_(2, 9);
    V_xx(3, 0) = Q_xu_(3, 0)*k_ux(0, 0) + Q_xu_(3, 1)*k_ux(1, 0) + Q_xu_(3, 2)*k_ux(2, 0) + Q_xx_(0, 3);
    V_xx(3, 1) = Q_xu_(3, 0)*k_ux(0, 1) + Q_xu_(3, 1)*k_ux(1, 1) + Q_xu_(3, 2)*k_ux(2, 1) + Q_xu_(3, 3)*k_ux(3, 1) + Q_xu_(3, 4)*k_ux(4, 1) + Q_xu_(3, 5)*k_ux(5, 1) + Q_xu_(3, 6)*k_ux(6, 1) + Q_xu_(3, 7)*k_ux(7, 1) + Q_xu_(3, 8)*k_ux(8, 1) + Q_xx_(1, 3);
    V_xx(3, 2) = Q_xu_(3, 0)*k_ux(0, 2) + Q_xu_(3, 1)*k_ux(1, 2) + Q_xu_(3, 2)*k_ux(2, 2) + Q_xu_(3, 3)*k_ux(3, 2) + Q_xu_(3, 4)*k_ux(4, 2) + Q_xu_(3, 5)*k_ux(5, 2) + Q_xu_(3, 6)*k_ux(6, 2) + Q_xu_(3, 7)*k_ux(7, 2) + Q_xu_(3, 8)*k_ux(8, 2) + Q_xx_(2, 3);
    V_xx(3, 3) = Q_xu_(3, 0)*k_ux(0, 3) + Q_xu_(3, 1)*k_ux(1, 3) + Q_xu_(3, 2)*k_ux(2, 3) + Q_xu_(3, 3)*k_ux(3, 3) + Q_xu_(3, 4)*k_ux(4, 3) + Q_xu_(3, 5)*k_ux(5, 3) + Q_xu_(3, 6)*k_ux(6, 3) + Q_xu_(3, 7)*k_ux(7, 3) + Q_xu_(3, 8)*k_ux(8, 3) + Q_xx_(3, 3);
    V_xx(3, 4) = Q_xu_(3, 0)*k_ux(0, 4) + Q_xu_(3, 1)*k_ux(1, 4) + Q_xu_(3, 2)*k_ux(2, 4) + Q_xx_(3, 4);
    V_xx(3, 5) = Q_xu_(3, 0)*k_ux(0, 5) + Q_xu_(3, 1)*k_ux(1, 5) + Q_xu_(3, 2)*k_ux(2, 5) + Q_xx_(3, 5);
    V_xx(3, 6) = Q_xu_(3, 0)*k_ux(0, 6) + Q_xu_(3, 1)*k_ux(1, 6) + Q_xu_(3, 2)*k_ux(2, 6) + Q_xu_(3, 3)*k_ux(3, 6) + Q_xu_(3, 4)*k_ux(4, 6) + Q_xx_(3, 6);
    V_xx(3, 7) = Q_xu_(3, 0)*k_ux(0, 7) + Q_xu_(3, 1)*k_ux(1, 7) + Q_xu_(3, 2)*k_ux(2, 7) + Q_xu_(3, 3)*k_ux(3, 7) + Q_xu_(3, 4)*k_ux(4, 7) + Q_xx_(3, 7);
    V_xx(3, 8) = Q_xu_(3, 0)*k_ux(0, 8) + Q_xu_(3, 1)*k_ux(1, 8) + Q_xu_(3, 2)*k_ux(2, 8) + Q_xx_(3, 8);
    V_xx(3, 9) = Q_xu_(3, 0)*k_ux(0, 9) + Q_xu_(3, 1)*k_ux(1, 9) + Q_xu_(3, 2)*k_ux(2, 9) + Q_xx_(3, 9);
    V_xx(4, 0) = Q_xu_(4, 0)*k_ux(0, 0) + Q_xu_(4, 1)*k_ux(1, 0) + Q_xu_(4, 2)*k_ux(2, 0) + Q_xx_(0, 4);
    V_xx(4, 1) = Q_xu_(4, 0)*k_ux(0, 1) + Q_xu_(4, 1)*k_ux(1, 1) + Q_xu_(4, 2)*k_ux(2, 1) + Q_xx_(1, 4);
    V_xx(4, 2) = Q_xu_(4, 0)*k_ux(0, 2) + Q_xu_(4, 1)*k_ux(1, 2) + Q_xu_(4, 2)*k_ux(2, 2) + Q_xx_(2, 4);
    V_xx(4, 3) = Q_xu_(4, 0)*k_ux(0, 3) + Q_xu_(4, 1)*k_ux(1, 3) + Q_xu_(4, 2)*k_ux(2, 3) + Q_xx_(3, 4);
    V_xx(4, 4) = Q_xu_(4, 0)*k_ux(0, 4) + Q_xu_(4, 1)*k_ux(1, 4) + Q_xu_(4, 2)*k_ux(2, 4) + Q_xx_(4, 4);
    V_xx(4, 5) = Q_xu_(4, 0)*k_ux(0, 5) + Q_xu_(4, 1)*k_ux(1, 5) + Q_xu_(4, 2)*k_ux(2, 5) + Q_xx_(4, 5);
    V_xx(4, 6) = Q_xu_(4, 0)*k_ux(0, 6) + Q_xu_(4, 1)*k_ux(1, 6) + Q_xu_(4, 2)*k_ux(2, 6) + Q_xx_(4, 6);
    V_xx(4, 7) = Q_xu_(4, 0)*k_ux(0, 7) + Q_xu_(4, 1)*k_ux(1, 7) + Q_xu_(4, 2)*k_ux(2, 7) + Q_xx_(4, 7);
    V_xx(4, 8) = Q_xu_(4, 0)*k_ux(0, 8) + Q_xu_(4, 1)*k_ux(1, 8) + Q_xu_(4, 2)*k_ux(2, 8) + Q_xx_(4, 8);
    V_xx(4, 9) = Q_xu_(4, 0)*k_ux(0, 9) + Q_xu_(4, 1)*k_ux(1, 9) + Q_xu_(4, 2)*k_ux(2, 9) + Q_xx_(4, 9);
    V_xx(5, 0) = Q_xu_(5, 0)*k_ux(0, 0) + Q_xu_(5, 1)*k_ux(1, 0) + Q_xu_(5, 2)*k_ux(2, 0) + Q_xx_(0, 5);
    V_xx(5, 1) = Q_xu_(5, 0)*k_ux(0, 1) + Q_xu_(5, 1)*k_ux(1, 1) + Q_xu_(5, 2)*k_ux(2, 1) + Q_xx_(1, 5);
    V_xx(5, 2) = Q_xu_(5, 0)*k_ux(0, 2) + Q_xu_(5, 1)*k_ux(1, 2) + Q_xu_(5, 2)*k_ux(2, 2) + Q_xx_(2, 5);
    V_xx(5, 3) = Q_xu_(5, 0)*k_ux(0, 3) + Q_xu_(5, 1)*k_ux(1, 3) + Q_xu_(5, 2)*k_ux(2, 3) + Q_xx_(3, 5);
    V_xx(5, 4) = Q_xu_(5, 0)*k_ux(0, 4) + Q_xu_(5, 1)*k_ux(1, 4) + Q_xu_(5, 2)*k_ux(2, 4) + Q_xx_(4, 5);
    V_xx(5, 5) = Q_xu_(5, 0)*k_ux(0, 5) + Q_xu_(5, 1)*k_ux(1, 5) + Q_xu_(5, 2)*k_ux(2, 5) + Q_xx_(5, 5);
    V_xx(5, 6) = Q_xu_(5, 0)*k_ux(0, 6) + Q_xu_(5, 1)*k_ux(1, 6) + Q_xu_(5, 2)*k_ux(2, 6) + Q_xx_(5, 6);
    V_xx(5, 7) = Q_xu_(5, 0)*k_ux(0, 7) + Q_xu_(5, 1)*k_ux(1, 7) + Q_xu_(5, 2)*k_ux(2, 7) + Q_xx_(5, 7);
    V_xx(5, 8) = Q_xu_(5, 0)*k_ux(0, 8) + Q_xu_(5, 1)*k_ux(1, 8) + Q_xu_(5, 2)*k_ux(2, 8) + Q_xx_(5, 8);
    V_xx(5, 9) = Q_xu_(5, 0)*k_ux(0, 9) + Q_xu_(5, 1)*k_ux(1, 9) + Q_xu_(5, 2)*k_ux(2, 9) + Q_xx_(5, 9);
    V_xx(6, 0) = Q_xu_(6, 0)*k_ux(0, 0) + Q_xu_(6, 1)*k_ux(1, 0) + Q_xu_(6, 2)*k_ux(2, 0) + Q_xx_(0, 6);
    V_xx(6, 1) = Q_xu_(6, 0)*k_ux(0, 1) + Q_xu_(6, 1)*k_ux(1, 1) + Q_xu_(6, 2)*k_ux(2, 1) + Q_xu_(6, 3)*k_ux(3, 1) + Q_xu_(6, 4)*k_ux(4, 1) + Q_xx_(1, 6);
    V_xx(6, 2) = Q_xu_(6, 0)*k_ux(0, 2) + Q_xu_(6, 1)*k_ux(1, 2) + Q_xu_(6, 2)*k_ux(2, 2) + Q_xu_(6, 3)*k_ux(3, 2) + Q_xu_(6, 4)*k_ux(4, 2) + Q_xx_(2, 6);
    V_xx(6, 3) = Q_xu_(6, 0)*k_ux(0, 3) + Q_xu_(6, 1)*k_ux(1, 3) + Q_xu_(6, 2)*k_ux(2, 3) + Q_xu_(6, 3)*k_ux(3, 3) + Q_xu_(6, 4)*k_ux(4, 3) + Q_xx_(3, 6);
    V_xx(6, 4) = Q_xu_(6, 0)*k_ux(0, 4) + Q_xu_(6, 1)*k_ux(1, 4) + Q_xu_(6, 2)*k_ux(2, 4) + Q_xx_(4, 6);
    V_xx(6, 5) = Q_xu_(6, 0)*k_ux(0, 5) + Q_xu_(6, 1)*k_ux(1, 5) + Q_xu_(6, 2)*k_ux(2, 5) + Q_xx_(5, 6);
    V_xx(6, 6) = Q_xu_(6, 0)*k_ux(0, 6) + Q_xu_(6, 1)*k_ux(1, 6) + Q_xu_(6, 2)*k_ux(2, 6) + Q_xu_(6, 3)*k_ux(3, 6) + Q_xu_(6, 4)*k_ux(4, 6) + Q_xx_(6, 6);
    V_xx(6, 7) = Q_xu_(6, 0)*k_ux(0, 7) + Q_xu_(6, 1)*k_ux(1, 7) + Q_xu_(6, 2)*k_ux(2, 7) + Q_xu_(6, 3)*k_ux(3, 7) + Q_xu_(6, 4)*k_ux(4, 7) + Q_xx_(6, 7);
    V_xx(6, 8) = Q_xu_(6, 0)*k_ux(0, 8) + Q_xu_(6, 1)*k_ux(1, 8) + Q_xu_(6, 2)*k_ux(2, 8) + Q_xx_(6, 8);
    V_xx(6, 9) = Q_xu_(6, 0)*k_ux(0, 9) + Q_xu_(6, 1)*k_ux(1, 9) + Q_xu_(6, 2)*k_ux(2, 9) + Q_xx_(6, 9);
    V_xx(7, 0) = Q_xu_(7, 0)*k_ux(0, 0) + Q_xu_(7, 1)*k_ux(1, 0) + Q_xu_(7, 2)*k_ux(2, 0) + Q_xx_(0, 7);
    V_xx(7, 1) = Q_xu_(7, 0)*k_ux(0, 1) + Q_xu_(7, 1)*k_ux(1, 1) + Q_xu_(7, 2)*k_ux(2, 1) + Q_xu_(7, 3)*k_ux(3, 1) + Q_xu_(7, 4)*k_ux(4, 1) + Q_xx_(1, 7);
    V_xx(7, 2) = Q_xu_(7, 0)*k_ux(0, 2) + Q_xu_(7, 1)*k_ux(1, 2) + Q_xu_(7, 2)*k_ux(2, 2) + Q_xu_(7, 3)*k_ux(3, 2) + Q_xu_(7, 4)*k_ux(4, 2) + Q_xx_(2, 7);
    V_xx(7, 3) = Q_xu_(7, 0)*k_ux(0, 3) + Q_xu_(7, 1)*k_ux(1, 3) + Q_xu_(7, 2)*k_ux(2, 3) + Q_xu_(7, 3)*k_ux(3, 3) + Q_xu_(7, 4)*k_ux(4, 3) + Q_xx_(3, 7);
    V_xx(7, 4) = Q_xu_(7, 0)*k_ux(0, 4) + Q_xu_(7, 1)*k_ux(1, 4) + Q_xu_(7, 2)*k_ux(2, 4) + Q_xx_(4, 7);
    V_xx(7, 5) = Q_xu_(7, 0)*k_ux(0, 5) + Q_xu_(7, 1)*k_ux(1, 5) + Q_xu_(7, 2)*k_ux(2, 5) + Q_xx_(5, 7);
    V_xx(7, 6) = Q_xu_(7, 0)*k_ux(0, 6) + Q_xu_(7, 1)*k_ux(1, 6) + Q_xu_(7, 2)*k_ux(2, 6) + Q_xu_(7, 3)*k_ux(3, 6) + Q_xu_(7, 4)*k_ux(4, 6) + Q_xx_(6, 7);
    V_xx(7, 7) = Q_xu_(7, 0)*k_ux(0, 7) + Q_xu_(7, 1)*k_ux(1, 7) + Q_xu_(7, 2)*k_ux(2, 7) + Q_xu_(7, 3)*k_ux(3, 7) + Q_xu_(7, 4)*k_ux(4, 7) + Q_xx_(7, 7);
    V_xx(7, 8) = Q_xu_(7, 0)*k_ux(0, 8) + Q_xu_(7, 1)*k_ux(1, 8) + Q_xu_(7, 2)*k_ux(2, 8) + Q_xx_(7, 8);
    V_xx(7, 9) = Q_xu_(7, 0)*k_ux(0, 9) + Q_xu_(7, 1)*k_ux(1, 9) + Q_xu_(7, 2)*k_ux(2, 9) + Q_xx_(7, 9);
    V_xx(8, 0) = Q_xu_(8, 0)*k_ux(0, 0) + Q_xu_(8, 1)*k_ux(1, 0) + Q_xu_(8, 2)*k_ux(2, 0) + Q_xx_(0, 8);
    V_xx(8, 1) = Q_xu_(8, 0)*k_ux(0, 1) + Q_xu_(8, 1)*k_ux(1, 1) + Q_xu_(8, 2)*k_ux(2, 1) + Q_xx_(1, 8);
    V_xx(8, 2) = Q_xu_(8, 0)*k_ux(0, 2) + Q_xu_(8, 1)*k_ux(1, 2) + Q_xu_(8, 2)*k_ux(2, 2) + Q_xx_(2, 8);
    V_xx(8, 3) = Q_xu_(8, 0)*k_ux(0, 3) + Q_xu_(8, 1)*k_ux(1, 3) + Q_xu_(8, 2)*k_ux(2, 3) + Q_xx_(3, 8);
    V_xx(8, 4) = Q_xu_(8, 0)*k_ux(0, 4) + Q_xu_(8, 1)*k_ux(1, 4) + Q_xu_(8, 2)*k_ux(2, 4) + Q_xx_(4, 8);
    V_xx(8, 5) = Q_xu_(8, 0)*k_ux(0, 5) + Q_xu_(8, 1)*k_ux(1, 5) + Q_xu_(8, 2)*k_ux(2, 5) + Q_xx_(5, 8);
    V_xx(8, 6) = Q_xu_(8, 0)*k_ux(0, 6) + Q_xu_(8, 1)*k_ux(1, 6) + Q_xu_(8, 2)*k_ux(2, 6) + Q_xx_(6, 8);
    V_xx(8, 7) = Q_xu_(8, 0)*k_ux(0, 7) + Q_xu_(8, 1)*k_ux(1, 7) + Q_xu_(8, 2)*k_ux(2, 7) + Q_xx_(7, 8);
    V_xx(8, 8) = Q_xu_(8, 0)*k_ux(0, 8) + Q_xu_(8, 1)*k_ux(1, 8) + Q_xu_(8, 2)*k_ux(2, 8) + Q_xx_(8, 8);
    V_xx(8, 9) = Q_xu_(8, 0)*k_ux(0, 9) + Q_xu_(8, 1)*k_ux(1, 9) + Q_xu_(8, 2)*k_ux(2, 9) + Q_xx_(8, 9);
    V_xx(9, 0) = Q_xu_(9, 0)*k_ux(0, 0) + Q_xu_(9, 1)*k_ux(1, 0) + Q_xu_(9, 2)*k_ux(2, 0) + Q_xx_(0, 9);
    V_xx(9, 1) = Q_xu_(9, 0)*k_ux(0, 1) + Q_xu_(9, 1)*k_ux(1, 1) + Q_xu_(9, 2)*k_ux(2, 1) + Q_xx_(1, 9);
    V_xx(9, 2) = Q_xu_(9, 0)*k_ux(0, 2) + Q_xu_(9, 1)*k_ux(1, 2) + Q_xu_(9, 2)*k_ux(2, 2) + Q_xx_(2, 9);
    V_xx(9, 3) = Q_xu_(9, 0)*k_ux(0, 3) + Q_xu_(9, 1)*k_ux(1, 3) + Q_xu_(9, 2)*k_ux(2, 3) + Q_xx_(3, 9);
    V_xx(9, 4) = Q_xu_(9, 0)*k_ux(0, 4) + Q_xu_(9, 1)*k_ux(1, 4) + Q_xu_(9, 2)*k_ux(2, 4) + Q_xx_(4, 9);
    V_xx(9, 5) = Q_xu_(9, 0)*k_ux(0, 5) + Q_xu_(9, 1)*k_ux(1, 5) + Q_xu_(9, 2)*k_ux(2, 5) + Q_xx_(5, 9);
    V_xx(9, 6) = Q_xu_(9, 0)*k_ux(0, 6) + Q_xu_(9, 1)*k_ux(1, 6) + Q_xu_(9, 2)*k_ux(2, 6) + Q_xx_(6, 9);
    V_xx(9, 7) = Q_xu_(9, 0)*k_ux(0, 7) + Q_xu_(9, 1)*k_ux(1, 7) + Q_xu_(9, 2)*k_ux(2, 7) + Q_xx_(7, 9);
    V_xx(9, 8) = Q_xu_(9, 0)*k_ux(0, 8) + Q_xu_(9, 1)*k_ux(1, 8) + Q_xu_(9, 2)*k_ux(2, 8) + Q_xx_(8, 9);
    V_xx(9, 9) = Q_xu_(9, 0)*k_ux(0, 9) + Q_xu_(9, 1)*k_ux(1, 9) + Q_xu_(9, 2)*k_ux(2, 9) + Q_xx_(9, 9);

    // Evaluation of Vector V_x
    V_x(0) = Q_x_(0) + Q_xu_(0, 0)*v_u(0) + Q_xu_(0, 1)*v_u(1) + Q_xu_(0, 2)*v_u(2);
    V_x(1) = Q_x_(1) + Q_xu_(1, 0)*v_u(0) + Q_xu_(1, 1)*v_u(1) + Q_xu_(1, 2)*v_u(2) + Q_xu_(1, 3)*v_u(3) + Q_xu_(1, 4)*v_u(4) + Q_xu_(1, 5)*v_u(5) + Q_xu_(1, 6)*v_u(6) + Q_xu_(1, 7)*v_u(7) + Q_xu_(1, 8)*v_u(8);
    V_x(2) = Q_x_(2) + Q_xu_(2, 0)*v_u(0) + Q_xu_(2, 1)*v_u(1) + Q_xu_(2, 2)*v_u(2) + Q_xu_(2, 3)*v_u(3) + Q_xu_(2, 4)*v_u(4) + Q_xu_(2, 5)*v_u(5) + Q_xu_(2, 6)*v_u(6) + Q_xu_(2, 7)*v_u(7) + Q_xu_(2, 8)*v_u(8);
    V_x(3) = Q_x_(3) + Q_xu_(3, 0)*v_u(0) + Q_xu_(3, 1)*v_u(1) + Q_xu_(3, 2)*v_u(2) + Q_xu_(3, 3)*v_u(3) + Q_xu_(3, 4)*v_u(4) + Q_xu_(3, 5)*v_u(5) + Q_xu_(3, 6)*v_u(6) + Q_xu_(3, 7)*v_u(7) + Q_xu_(3, 8)*v_u(8);
    V_x(4) = Q_x_(4) + Q_xu_(4, 0)*v_u(0) + Q_xu_(4, 1)*v_u(1) + Q_xu_(4, 2)*v_u(2);
    V_x(5) = Q_x_(5) + Q_xu_(5, 0)*v_u(0) + Q_xu_(5, 1)*v_u(1) + Q_xu_(5, 2)*v_u(2);
    V_x(6) = Q_x_(6) + Q_xu_(6, 0)*v_u(0) + Q_xu_(6, 1)*v_u(1) + Q_xu_(6, 2)*v_u(2) + Q_xu_(6, 3)*v_u(3) + Q_xu_(6, 4)*v_u(4);
    V_x(7) = Q_x_(7) + Q_xu_(7, 0)*v_u(0) + Q_xu_(7, 1)*v_u(1) + Q_xu_(7, 2)*v_u(2) + Q_xu_(7, 3)*v_u(3) + Q_xu_(7, 4)*v_u(4);
    V_x(8) = Q_x_(8) + Q_xu_(8, 0)*v_u(0) + Q_xu_(8, 1)*v_u(1) + Q_xu_(8, 2)*v_u(2);
    V_x(9) = Q_x_(9) + Q_xu_(9, 0)*v_u(0) + Q_xu_(9, 1)*v_u(1) + Q_xu_(9, 2)*v_u(2);

  }

  virtual void updateQnx(const Eigen::VectorXd& r_f, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& h_u, const Eigen::MatrixXd& k_ux, const Eigen::MatrixXd& k_ex, const Eigen::MatrixXd& k_ue, const Eigen::VectorXd& v_u, const Eigen::VectorXd& v_e) override {
    // Determine internal variables
    const double internal_0 = -Q_uu_(0, 1);
    const double internal_1 = -Q_uu_(0, 2);
    const double internal_2 = -Q_uu_(1, 2);

    // Evaluation of Matrix Q_un_
    Q_un_(0, 0) = -Fu(0, 0)*Q_uu_inv_(0, 0) - Fu(0, 1)*Q_uu_inv_(0, 1);
    Q_un_(0, 1) = -Fu(1, 0)*Q_uu_inv_(0, 0) - Fu(1, 1)*Q_uu_inv_(0, 1);
    Q_un_(0, 2) = -Fu(2, 0)*Q_uu_inv_(0, 0) - Fu(2, 1)*Q_uu_inv_(0, 1);
    Q_un_(0, 3) = -Fu(3, 0)*Q_uu_inv_(0, 0) - Fu(3, 1)*Q_uu_inv_(0, 1);
    Q_un_(0, 4) = -Fu(4, 0)*Q_uu_inv_(0, 0);
    Q_un_(0, 5) = -Fu(5, 2)*Q_uu_inv_(0, 2);
    Q_un_(0, 6) = -Fu(6, 2)*Q_uu_inv_(0, 2);
    Q_un_(0, 7) = -Fu(7, 2)*Q_uu_inv_(0, 2);
    Q_un_(0, 9) = -Fu(9, 2)*Q_uu_inv_(0, 2);
    Q_un_(1, 0) = -Fu(0, 0)*Q_uu_inv_(0, 1) - Fu(0, 1)*Q_uu_inv_(1, 1);
    Q_un_(1, 1) = -Fu(1, 0)*Q_uu_inv_(0, 1) - Fu(1, 1)*Q_uu_inv_(1, 1);
    Q_un_(1, 2) = -Fu(2, 0)*Q_uu_inv_(0, 1) - Fu(2, 1)*Q_uu_inv_(1, 1);
    Q_un_(1, 3) = -Fu(3, 0)*Q_uu_inv_(0, 1) - Fu(3, 1)*Q_uu_inv_(1, 1);
    Q_un_(1, 4) = -Fu(4, 0)*Q_uu_inv_(0, 1);
    Q_un_(1, 5) = -Fu(5, 2)*Q_uu_inv_(1, 2);
    Q_un_(1, 6) = -Fu(6, 2)*Q_uu_inv_(1, 2);
    Q_un_(1, 7) = -Fu(7, 2)*Q_uu_inv_(1, 2);
    Q_un_(1, 9) = -Fu(9, 2)*Q_uu_inv_(1, 2);
    Q_un_(2, 0) = -Fu(0, 0)*Q_uu_inv_(0, 2) - Fu(0, 1)*Q_uu_inv_(1, 2);
    Q_un_(2, 1) = -Fu(1, 0)*Q_uu_inv_(0, 2) - Fu(1, 1)*Q_uu_inv_(1, 2);
    Q_un_(2, 2) = -Fu(2, 0)*Q_uu_inv_(0, 2) - Fu(2, 1)*Q_uu_inv_(1, 2);
    Q_un_(2, 3) = -Fu(3, 0)*Q_uu_inv_(0, 2) - Fu(3, 1)*Q_uu_inv_(1, 2);
    Q_un_(2, 4) = -Fu(4, 0)*Q_uu_inv_(0, 2);
    Q_un_(2, 5) = -Fu(5, 2)*Q_uu_inv_(2, 2);
    Q_un_(2, 6) = -Fu(6, 2)*Q_uu_inv_(2, 2);
    Q_un_(2, 7) = -Fu(7, 2)*Q_uu_inv_(2, 2);
    Q_un_(2, 9) = -Fu(9, 2)*Q_uu_inv_(2, 2);

    // Evaluation of Matrix Q_nuu_
    Q_nuu_(0, 0) = -Q_uu_(0, 0);
    Q_nuu_(0, 1) = internal_0;
    Q_nuu_(0, 2) = internal_1;
    Q_nuu_(1, 0) = internal_0;
    Q_nuu_(1, 1) = -Q_uu_(1, 1);
    Q_nuu_(1, 2) = internal_2;
    Q_nuu_(2, 0) = internal_1;
    Q_nuu_(2, 1) = internal_2;
    Q_nuu_(2, 2) = -Q_uu_(2, 2);
    Q_nuu_(3, 3) = -Q_uu_(3, 3);
    Q_nuu_(4, 4) = -Q_uu_(4, 4);
    Q_nuu_(5, 5) = -Q_uu_(5, 5);
    Q_nuu_(6, 6) = -Q_uu_(6, 6);
    Q_nuu_(7, 7) = -Q_uu_(7, 7);
    Q_nuu_(8, 8) = -Q_uu_(8, 8);

    // Evaluation of Matrix Q_vnx_
    Q_vnx_(0, 0) = Fu(0, 0)*k_ux(0, 0) + Fu(0, 1)*k_ux(1, 0) + Fx(0, 0);
    Q_vnx_(0, 1) = Fu(0, 0)*k_ux(0, 1) + Fu(0, 1)*k_ux(1, 1) + Fx(0, 1);
    Q_vnx_(0, 2) = Fu(0, 0)*k_ux(0, 2) + Fu(0, 1)*k_ux(1, 2) + Fx(0, 2);
    Q_vnx_(0, 3) = Fu(0, 0)*k_ux(0, 3) + Fu(0, 1)*k_ux(1, 3) + Fx(0, 3);
    Q_vnx_(0, 4) = Fu(0, 0)*k_ux(0, 4) + Fu(0, 1)*k_ux(1, 4) + Fx(0, 4);
    Q_vnx_(0, 5) = Fu(0, 0)*k_ux(0, 5) + Fu(0, 1)*k_ux(1, 5);
    Q_vnx_(0, 6) = Fu(0, 0)*k_ux(0, 6) + Fu(0, 1)*k_ux(1, 6);
    Q_vnx_(0, 7) = Fu(0, 0)*k_ux(0, 7) + Fu(0, 1)*k_ux(1, 7);
    Q_vnx_(0, 8) = Fu(0, 0)*k_ux(0, 8) + Fu(0, 1)*k_ux(1, 8);
    Q_vnx_(0, 9) = Fu(0, 0)*k_ux(0, 9) + Fu(0, 1)*k_ux(1, 9);
    Q_vnx_(1, 0) = Fu(1, 0)*k_ux(0, 0) + Fu(1, 1)*k_ux(1, 0);
    Q_vnx_(1, 1) = Fu(1, 0)*k_ux(0, 1) + Fu(1, 1)*k_ux(1, 1) + Fx(1, 1);
    Q_vnx_(1, 2) = Fu(1, 0)*k_ux(0, 2) + Fu(1, 1)*k_ux(1, 2);
    Q_vnx_(1, 3) = Fu(1, 0)*k_ux(0, 3) + Fu(1, 1)*k_ux(1, 3) + Fx(1, 3);
    Q_vnx_(1, 4) = Fu(1, 0)*k_ux(0, 4) + Fu(1, 1)*k_ux(1, 4) + Fx(1, 4);
    Q_vnx_(1, 5) = Fu(1, 0)*k_ux(0, 5) + Fu(1, 1)*k_ux(1, 5);
    Q_vnx_(1, 6) = Fu(1, 0)*k_ux(0, 6) + Fu(1, 1)*k_ux(1, 6);
    Q_vnx_(1, 7) = Fu(1, 0)*k_ux(0, 7) + Fu(1, 1)*k_ux(1, 7);
    Q_vnx_(1, 8) = Fu(1, 0)*k_ux(0, 8) + Fu(1, 1)*k_ux(1, 8);
    Q_vnx_(1, 9) = Fu(1, 0)*k_ux(0, 9) + Fu(1, 1)*k_ux(1, 9);
    Q_vnx_(2, 0) = Fu(2, 0)*k_ux(0, 0) + Fu(2, 1)*k_ux(1, 0);
    Q_vnx_(2, 1) = Fu(2, 0)*k_ux(0, 1) + Fu(2, 1)*k_ux(1, 1);
    Q_vnx_(2, 2) = Fu(2, 0)*k_ux(0, 2) + Fu(2, 1)*k_ux(1, 2) + Fx(2, 2);
    Q_vnx_(2, 3) = Fu(2, 0)*k_ux(0, 3) + Fu(2, 1)*k_ux(1, 3) + Fx(2, 3);
    Q_vnx_(2, 4) = Fu(2, 0)*k_ux(0, 4) + Fu(2, 1)*k_ux(1, 4) + Fx(2, 4);
    Q_vnx_(2, 5) = Fu(2, 0)*k_ux(0, 5) + Fu(2, 1)*k_ux(1, 5);
    Q_vnx_(2, 6) = Fu(2, 0)*k_ux(0, 6) + Fu(2, 1)*k_ux(1, 6);
    Q_vnx_(2, 7) = Fu(2, 0)*k_ux(0, 7) + Fu(2, 1)*k_ux(1, 7);
    Q_vnx_(2, 8) = Fu(2, 0)*k_ux(0, 8) + Fu(2, 1)*k_ux(1, 8);
    Q_vnx_(2, 9) = Fu(2, 0)*k_ux(0, 9) + Fu(2, 1)*k_ux(1, 9);
    Q_vnx_(3, 0) = Fu(3, 0)*k_ux(0, 0) + Fu(3, 1)*k_ux(1, 0);
    Q_vnx_(3, 1) = Fu(3, 0)*k_ux(0, 1) + Fu(3, 1)*k_ux(1, 1);
    Q_vnx_(3, 2) = Fu(3, 0)*k_ux(0, 2) + Fu(3, 1)*k_ux(1, 2);
    Q_vnx_(3, 3) = Fu(3, 0)*k_ux(0, 3) + Fu(3, 1)*k_ux(1, 3) + Fx(3, 3);
    Q_vnx_(3, 4) = Fu(3, 0)*k_ux(0, 4) + Fu(3, 1)*k_ux(1, 4) + Fx(3, 4);
    Q_vnx_(3, 5) = Fu(3, 0)*k_ux(0, 5) + Fu(3, 1)*k_ux(1, 5);
    Q_vnx_(3, 6) = Fu(3, 0)*k_ux(0, 6) + Fu(3, 1)*k_ux(1, 6);
    Q_vnx_(3, 7) = Fu(3, 0)*k_ux(0, 7) + Fu(3, 1)*k_ux(1, 7);
    Q_vnx_(3, 8) = Fu(3, 0)*k_ux(0, 8) + Fu(3, 1)*k_ux(1, 8);
    Q_vnx_(3, 9) = Fu(3, 0)*k_ux(0, 9) + Fu(3, 1)*k_ux(1, 9);
    Q_vnx_(4, 0) = Fu(4, 0)*k_ux(0, 0);
    Q_vnx_(4, 1) = Fu(4, 0)*k_ux(0, 1);
    Q_vnx_(4, 2) = Fu(4, 0)*k_ux(0, 2);
    Q_vnx_(4, 3) = Fu(4, 0)*k_ux(0, 3);
    Q_vnx_(4, 4) = Fu(4, 0)*k_ux(0, 4) + Fx(4, 4);
    Q_vnx_(4, 5) = Fu(4, 0)*k_ux(0, 5);
    Q_vnx_(4, 6) = Fu(4, 0)*k_ux(0, 6);
    Q_vnx_(4, 7) = Fu(4, 0)*k_ux(0, 7);
    Q_vnx_(4, 8) = Fu(4, 0)*k_ux(0, 8);
    Q_vnx_(4, 9) = Fu(4, 0)*k_ux(0, 9);
    Q_vnx_(5, 0) = Fu(5, 2)*k_ux(2, 0);
    Q_vnx_(5, 1) = Fu(5, 2)*k_ux(2, 1);
    Q_vnx_(5, 2) = Fu(5, 2)*k_ux(2, 2);
    Q_vnx_(5, 3) = Fu(5, 2)*k_ux(2, 3);
    Q_vnx_(5, 4) = Fu(5, 2)*k_ux(2, 4);
    Q_vnx_(5, 5) = Fu(5, 2)*k_ux(2, 5) + Fx(5, 5);
    Q_vnx_(5, 6) = Fu(5, 2)*k_ux(2, 6);
    Q_vnx_(5, 7) = Fu(5, 2)*k_ux(2, 7);
    Q_vnx_(5, 8) = Fu(5, 2)*k_ux(2, 8);
    Q_vnx_(5, 9) = Fu(5, 2)*k_ux(2, 9) + Fx(5, 9);
    Q_vnx_(6, 0) = Fu(6, 2)*k_ux(2, 0);
    Q_vnx_(6, 1) = Fu(6, 2)*k_ux(2, 1);
    Q_vnx_(6, 2) = Fu(6, 2)*k_ux(2, 2);
    Q_vnx_(6, 3) = Fu(6, 2)*k_ux(2, 3);
    Q_vnx_(6, 4) = Fu(6, 2)*k_ux(2, 4);
    Q_vnx_(6, 5) = Fu(6, 2)*k_ux(2, 5);
    Q_vnx_(6, 6) = Fu(6, 2)*k_ux(2, 6) + Fx(6, 6);
    Q_vnx_(6, 7) = Fu(6, 2)*k_ux(2, 7);
    Q_vnx_(6, 8) = Fu(6, 2)*k_ux(2, 8);
    Q_vnx_(6, 9) = Fu(6, 2)*k_ux(2, 9) + Fx(6, 9);
    Q_vnx_(7, 0) = Fu(7, 2)*k_ux(2, 0);
    Q_vnx_(7, 1) = Fu(7, 2)*k_ux(2, 1);
    Q_vnx_(7, 2) = Fu(7, 2)*k_ux(2, 2);
    Q_vnx_(7, 3) = Fu(7, 2)*k_ux(2, 3);
    Q_vnx_(7, 4) = Fu(7, 2)*k_ux(2, 4);
    Q_vnx_(7, 5) = Fu(7, 2)*k_ux(2, 5);
    Q_vnx_(7, 6) = Fu(7, 2)*k_ux(2, 6);
    Q_vnx_(7, 7) = Fu(7, 2)*k_ux(2, 7) + Fx(7, 7);
    Q_vnx_(7, 8) = Fu(7, 2)*k_ux(2, 8);
    Q_vnx_(7, 9) = Fu(7, 2)*k_ux(2, 9) + Fx(7, 9);
    Q_vnx_(8, 8) = Fx(8, 8);
    Q_vnx_(9, 0) = Fu(9, 2)*k_ux(2, 0);
    Q_vnx_(9, 1) = Fu(9, 2)*k_ux(2, 1);
    Q_vnx_(9, 2) = Fu(9, 2)*k_ux(2, 2);
    Q_vnx_(9, 3) = Fu(9, 2)*k_ux(2, 3);
    Q_vnx_(9, 4) = Fu(9, 2)*k_ux(2, 4);
    Q_vnx_(9, 5) = Fu(9, 2)*k_ux(2, 5);
    Q_vnx_(9, 6) = Fu(9, 2)*k_ux(2, 6);
    Q_vnx_(9, 7) = Fu(9, 2)*k_ux(2, 7);
    Q_vnx_(9, 8) = Fu(9, 2)*k_ux(2, 8);
    Q_vnx_(9, 9) = Fu(9, 2)*k_ux(2, 9) + Fx(9, 9);

    // Evaluation of Vector Q_vn_
    Q_vn_(0) = Fu(0, 0)*v_u(0) + Fu(0, 1)*v_u(1) + r_f(0);
    Q_vn_(1) = Fu(1, 0)*v_u(0) + Fu(1, 1)*v_u(1) + r_f(1);
    Q_vn_(2) = Fu(2, 0)*v_u(0) + Fu(2, 1)*v_u(1) + r_f(2);
    Q_vn_(3) = Fu(3, 0)*v_u(0) + Fu(3, 1)*v_u(1) + r_f(3);
    Q_vn_(4) = Fu(4, 0)*v_u(0) + r_f(4);
    Q_vn_(5) = Fu(5, 2)*v_u(2) + r_f(5);
    Q_vn_(6) = Fu(6, 2)*v_u(2) + r_f(6);
    Q_vn_(7) = Fu(7, 2)*v_u(2) + r_f(7);
    Q_vn_(8) = r_f(8);
    Q_vn_(9) = Fu(9, 2)*v_u(2) + r_f(9);

  }

};

template <>
class MultiAgentsTrajectoryModelIpmEvaluator<StageType::INITIAL, OcpConfig::ERK4> : public IpmEvaluator {
 public:
  MultiAgentsTrajectoryModelIpmEvaluator() : IpmEvaluator(10, 9, 12, 0, 10) {}
  virtual ~MultiAgentsTrajectoryModelIpmEvaluator() = default;

  virtual void updateLX(const Eigen::VectorXd& l_x, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& h_x, const Eigen::MatrixXd& m_x, const Eigen::VectorXd& p, const Eigen::VectorXd& lambda, const Eigen::VectorXd& eta, const Eigen::VectorXd& nu, Eigen::VectorXd& sl_x) const override {
    // Evaluation of Vector sl_x
    sl_x(0) = Fx(0, 0)*p(0) + l_x(0) + m_x(0, 0)*nu(0);
    sl_x(1) = Fx(0, 1)*p(0) + Fx(1, 1)*p(1) + l_x(1) + m_x(1, 1)*nu(1);
    sl_x(2) = Fx(0, 2)*p(0) + Fx(2, 2)*p(2) + l_x(2) + m_x(2, 2)*nu(2);
    sl_x(3) = Fx(0, 3)*p(0) + Fx(1, 3)*p(1) + Fx(2, 3)*p(2) + Fx(3, 3)*p(3) + l_x(3) + m_x(3, 3)*nu(3);
    sl_x(4) = Fx(0, 4)*p(0) + Fx(1, 4)*p(1) + Fx(2, 4)*p(2) + Fx(3, 4)*p(3) + Fx(4, 4)*p(4) + l_x(4) + m_x(4, 4)*nu(4);
    sl_x(5) = Fx(5, 5)*p(5) + l_x(5) + m_x(5, 5)*nu(5);
    sl_x(6) = Fx(6, 6)*p(6) + m_x(6, 6)*nu(6);
    sl_x(7) = Fx(7, 7)*p(7) + m_x(7, 7)*nu(7);
    sl_x(8) = Fx(8, 8)*p(8) + m_x(8, 8)*nu(8);
    sl_x(9) = Fx(5, 9)*p(5) + Fx(6, 9)*p(6) + Fx(7, 9)*p(7) + Fx(9, 9)*p(9) + l_x(9) + m_x(9, 9)*nu(9);

  }

  virtual void updateLU(const Eigen::VectorXd& l_u, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& g_u, const Eigen::MatrixXd& h_u, const Eigen::VectorXd& p, const Eigen::VectorXd& lambda, const Eigen::VectorXd& eta, Eigen::VectorXd& sl_u) const override {
    // Evaluation of Vector sl_u
    sl_u(0) = Fu(0, 0)*p(0) + Fu(1, 0)*p(1) + Fu(2, 0)*p(2) + Fu(3, 0)*p(3) + Fu(4, 0)*p(4) + g_u(8, 0)*lambda(8) + g_u(9, 0)*lambda(9) + l_u(0);
    sl_u(1) = Fu(0, 1)*p(0) + Fu(1, 1)*p(1) + Fu(2, 1)*p(2) + Fu(3, 1)*p(3) + g_u(6, 1)*lambda(6) + g_u(7, 1)*lambda(7) + l_u(1);
    sl_u(2) = Fu(5, 2)*p(5) + Fu(6, 2)*p(6) + Fu(7, 2)*p(7) + Fu(9, 2)*p(9) + g_u(10, 2)*lambda(10) + g_u(11, 2)*lambda(11) + l_u(2);
    sl_u(3) = g_u(0, 3)*lambda(0) + l_u(3);
    sl_u(4) = g_u(1, 4)*lambda(1) + l_u(4);
    sl_u(5) = g_u(2, 5)*lambda(2) + l_u(5);
    sl_u(6) = g_u(3, 6)*lambda(3) + l_u(6);
    sl_u(7) = g_u(4, 7)*lambda(4) + l_u(7);
    sl_u(8) = g_u(5, 8)*lambda(5) + l_u(8);

  }

  virtual void updateQH(const Eigen::MatrixXd& l_xx, const Eigen::MatrixXd& l_xu, const Eigen::MatrixXd& l_uu, const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& V_xx_prev) override {
    // Determine internal variables
    const double internal_0 = Fx(0, 1)*V_xx_prev(0, 0);
    const double internal_1 = Fx(0, 0)*V_xx_prev(0, 1);
    const double internal_10 = Fx(0, 0)*V_xx_prev(0, 7);
    const double internal_100 = Fu(6, 2)*V_xx_prev(6, 6);
    const double internal_101 = Fu(7, 2)*V_xx_prev(7, 7);
    const double internal_11 = Fx(0, 0)*Fx(8, 8);
    const double internal_12 = Fx(0, 0)*V_xx_prev(0, 9);
    const double internal_2 = Fx(0, 2)*V_xx_prev(0, 0);
    const double internal_3 = Fx(0, 0)*V_xx_prev(0, 2);
    const double internal_4 = Fx(0, 3)*V_xx_prev(0, 0);
    const double internal_49 = Fx(5, 5)*V_xx_prev(5, 0);
    const double internal_5 = Fx(0, 0)*V_xx_prev(0, 3);
    const double internal_50 = Fx(5, 5)*V_xx_prev(5, 1);
    const double internal_51 = Fx(5, 5)*V_xx_prev(5, 2);
    const double internal_52 = Fx(5, 5)*V_xx_prev(5, 3);
    const double internal_53 = Fx(5, 5)*V_xx_prev(5, 4);
    const double internal_54 = Fx(5, 5)*V_xx_prev(5, 6);
    const double internal_55 = Fx(5, 5)*V_xx_prev(5, 7);
    const double internal_56 = Fx(5, 5)*Fx(8, 8);
    const double internal_57 = Fx(5, 9)*V_xx_prev(5, 5);
    const double internal_58 = Fx(5, 5)*V_xx_prev(5, 9);
    const double internal_59 = Fx(6, 6)*V_xx_prev(6, 0);
    const double internal_6 = Fx(0, 4)*V_xx_prev(0, 0);
    const double internal_60 = Fx(6, 6)*V_xx_prev(6, 1);
    const double internal_61 = Fx(6, 6)*V_xx_prev(6, 2);
    const double internal_62 = Fx(6, 6)*V_xx_prev(6, 3);
    const double internal_63 = Fx(6, 6)*V_xx_prev(6, 4);
    const double internal_64 = Fx(6, 6)*V_xx_prev(6, 5);
    const double internal_65 = Fx(6, 6)*V_xx_prev(6, 7);
    const double internal_66 = Fx(6, 6)*Fx(8, 8);
    const double internal_67 = Fx(6, 9)*V_xx_prev(6, 6);
    const double internal_68 = Fx(6, 6)*V_xx_prev(6, 9);
    const double internal_69 = Fx(7, 7)*V_xx_prev(7, 0);
    const double internal_7 = Fx(0, 0)*V_xx_prev(0, 4);
    const double internal_70 = Fx(7, 7)*V_xx_prev(7, 1);
    const double internal_71 = Fx(7, 7)*V_xx_prev(7, 2);
    const double internal_72 = Fx(7, 7)*V_xx_prev(7, 3);
    const double internal_73 = Fx(7, 7)*V_xx_prev(7, 4);
    const double internal_74 = Fx(7, 7)*V_xx_prev(7, 5);
    const double internal_75 = Fx(7, 7)*V_xx_prev(7, 6);
    const double internal_76 = Fx(7, 7)*Fx(8, 8);
    const double internal_77 = Fx(7, 9)*V_xx_prev(7, 7);
    const double internal_78 = Fx(7, 7)*V_xx_prev(7, 9);
    const double internal_79 = Fx(8, 8)*V_xx_prev(8, 0);
    const double internal_8 = Fx(0, 0)*V_xx_prev(0, 5);
    const double internal_80 = Fx(8, 8)*V_xx_prev(8, 1);
    const double internal_81 = Fx(8, 8)*V_xx_prev(8, 2);
    const double internal_82 = Fx(8, 8)*V_xx_prev(8, 3);
    const double internal_83 = Fx(8, 8)*V_xx_prev(8, 4);
    const double internal_84 = Fx(8, 8)*V_xx_prev(8, 5);
    const double internal_85 = Fx(8, 8)*V_xx_prev(8, 6);
    const double internal_86 = Fx(8, 8)*V_xx_prev(8, 7);
    const double internal_87 = Fx(8, 8)*V_xx_prev(8, 9);
    const double internal_9 = Fx(0, 0)*V_xx_prev(0, 6);
    const double internal_97 = Fu(0, 0)*V_xx_prev(0, 0);
    const double internal_98 = Fu(0, 1)*V_xx_prev(0, 0);
    const double internal_99 = Fu(5, 2)*V_xx_prev(5, 5);
    const double internal_103 = Fu(0, 0)*V_xx_prev(0, 1) + Fu(1, 0)*V_xx_prev(1, 1) + Fu(2, 0)*V_xx_prev(2, 1) + Fu(3, 0)*V_xx_prev(3, 1) + Fu(4, 0)*V_xx_prev(4, 1);
    const double internal_104 = Fu(0, 0)*V_xx_prev(0, 2) + Fu(1, 0)*V_xx_prev(1, 2) + Fu(2, 0)*V_xx_prev(2, 2) + Fu(3, 0)*V_xx_prev(3, 2) + Fu(4, 0)*V_xx_prev(4, 2);
    const double internal_105 = Fu(0, 0)*V_xx_prev(0, 3) + Fu(1, 0)*V_xx_prev(1, 3) + Fu(2, 0)*V_xx_prev(2, 3) + Fu(3, 0)*V_xx_prev(3, 3) + Fu(4, 0)*V_xx_prev(4, 3);
    const double internal_107 = Fu(0, 1)*V_xx_prev(0, 1) + Fu(1, 1)*V_xx_prev(1, 1) + Fu(2, 1)*V_xx_prev(2, 1) + Fu(3, 1)*V_xx_prev(3, 1);
    const double internal_108 = Fu(0, 1)*V_xx_prev(0, 2) + Fu(1, 1)*V_xx_prev(1, 2) + Fu(2, 1)*V_xx_prev(2, 2) + Fu(3, 1)*V_xx_prev(3, 2);
    const double internal_109 = Fu(0, 1)*V_xx_prev(0, 3) + Fu(1, 1)*V_xx_prev(1, 3) + Fu(2, 1)*V_xx_prev(2, 3) + Fu(3, 1)*V_xx_prev(3, 3);
    const double internal_110 = Fu(5, 2)*V_xx_prev(5, 0) + Fu(6, 2)*V_xx_prev(6, 0) + Fu(7, 2)*V_xx_prev(7, 0) + Fu(9, 2)*V_xx_prev(9, 0);
    const double internal_111 = Fu(5, 2)*V_xx_prev(5, 1) + Fu(6, 2)*V_xx_prev(6, 1) + Fu(7, 2)*V_xx_prev(7, 1) + Fu(9, 2)*V_xx_prev(9, 1);
    const double internal_112 = Fu(5, 2)*V_xx_prev(5, 2) + Fu(6, 2)*V_xx_prev(6, 2) + Fu(7, 2)*V_xx_prev(7, 2) + Fu(9, 2)*V_xx_prev(9, 2);
    const double internal_113 = Fu(5, 2)*V_xx_prev(5, 3) + Fu(6, 2)*V_xx_prev(6, 3) + Fu(7, 2)*V_xx_prev(7, 3) + Fu(9, 2)*V_xx_prev(9, 3);
    const double internal_14 = Fx(0, 1)*V_xx_prev(0, 1) + Fx(1, 1)*V_xx_prev(1, 1);
    const double internal_15 = Fx(0, 1)*V_xx_prev(0, 2) + Fx(1, 1)*V_xx_prev(1, 2);
    const double internal_16 = Fx(0, 1)*V_xx_prev(0, 3) + Fx(1, 1)*V_xx_prev(1, 3);
    const double internal_17 = Fx(0, 1)*V_xx_prev(0, 4) + Fx(1, 1)*V_xx_prev(1, 4);
    const double internal_18 = Fx(0, 1)*V_xx_prev(0, 5) + Fx(1, 1)*V_xx_prev(1, 5);
    const double internal_19 = Fx(0, 1)*V_xx_prev(0, 6) + Fx(1, 1)*V_xx_prev(1, 6);
    const double internal_20 = Fx(0, 1)*V_xx_prev(0, 7) + Fx(1, 1)*V_xx_prev(1, 7);
    const double internal_21 = Fx(0, 1)*V_xx_prev(0, 9) + Fx(1, 1)*V_xx_prev(1, 9);
    const double internal_23 = Fx(0, 2)*V_xx_prev(0, 1) + Fx(2, 2)*V_xx_prev(2, 1);
    const double internal_24 = Fx(0, 2)*V_xx_prev(0, 2) + Fx(2, 2)*V_xx_prev(2, 2);
    const double internal_25 = Fx(0, 2)*V_xx_prev(0, 3) + Fx(2, 2)*V_xx_prev(2, 3);
    const double internal_26 = Fx(0, 2)*V_xx_prev(0, 4) + Fx(2, 2)*V_xx_prev(2, 4);
    const double internal_27 = Fx(0, 2)*V_xx_prev(0, 5) + Fx(2, 2)*V_xx_prev(2, 5);
    const double internal_28 = Fx(0, 2)*V_xx_prev(0, 6) + Fx(2, 2)*V_xx_prev(2, 6);
    const double internal_29 = Fx(0, 2)*V_xx_prev(0, 7) + Fx(2, 2)*V_xx_prev(2, 7);
    const double internal_30 = Fx(0, 2)*V_xx_prev(0, 9) + Fx(2, 2)*V_xx_prev(2, 9);
    const double internal_32 = Fx(0, 3)*V_xx_prev(0, 1) + Fx(1, 3)*V_xx_prev(1, 1) + Fx(2, 3)*V_xx_prev(2, 1) + Fx(3, 3)*V_xx_prev(3, 1);
    const double internal_33 = Fx(0, 3)*V_xx_prev(0, 2) + Fx(1, 3)*V_xx_prev(1, 2) + Fx(2, 3)*V_xx_prev(2, 2) + Fx(3, 3)*V_xx_prev(3, 2);
    const double internal_34 = Fx(0, 3)*V_xx_prev(0, 3) + Fx(1, 3)*V_xx_prev(1, 3) + Fx(2, 3)*V_xx_prev(2, 3) + Fx(3, 3)*V_xx_prev(3, 3);
    const double internal_35 = Fx(0, 3)*V_xx_prev(0, 4) + Fx(1, 3)*V_xx_prev(1, 4) + Fx(2, 3)*V_xx_prev(2, 4) + Fx(3, 3)*V_xx_prev(3, 4);
    const double internal_36 = Fx(0, 3)*V_xx_prev(0, 5) + Fx(1, 3)*V_xx_prev(1, 5) + Fx(2, 3)*V_xx_prev(2, 5) + Fx(3, 3)*V_xx_prev(3, 5);
    const double internal_37 = Fx(0, 3)*V_xx_prev(0, 6) + Fx(1, 3)*V_xx_prev(1, 6) + Fx(2, 3)*V_xx_prev(2, 6) + Fx(3, 3)*V_xx_prev(3, 6);
    const double internal_38 = Fx(0, 3)*V_xx_prev(0, 7) + Fx(1, 3)*V_xx_prev(1, 7) + Fx(2, 3)*V_xx_prev(2, 7) + Fx(3, 3)*V_xx_prev(3, 7);
    const double internal_39 = Fx(0, 3)*V_xx_prev(0, 9) + Fx(1, 3)*V_xx_prev(1, 9) + Fx(2, 3)*V_xx_prev(2, 9) + Fx(3, 3)*V_xx_prev(3, 9);
    const double internal_41 = Fx(0, 4)*V_xx_prev(0, 1) + Fx(1, 4)*V_xx_prev(1, 1) + Fx(2, 4)*V_xx_prev(2, 1) + Fx(3, 4)*V_xx_prev(3, 1) + Fx(4, 4)*V_xx_prev(4, 1);
    const double internal_42 = Fx(0, 4)*V_xx_prev(0, 2) + Fx(1, 4)*V_xx_prev(1, 2) + Fx(2, 4)*V_xx_prev(2, 2) + Fx(3, 4)*V_xx_prev(3, 2) + Fx(4, 4)*V_xx_prev(4, 2);
    const double internal_43 = Fx(0, 4)*V_xx_prev(0, 3) + Fx(1, 4)*V_xx_prev(1, 3) + Fx(2, 4)*V_xx_prev(2, 3) + Fx(3, 4)*V_xx_prev(3, 3) + Fx(4, 4)*V_xx_prev(4, 3);
    const double internal_44 = Fx(0, 4)*V_xx_prev(0, 4) + Fx(1, 4)*V_xx_prev(1, 4) + Fx(2, 4)*V_xx_prev(2, 4) + Fx(3, 4)*V_xx_prev(3, 4) + Fx(4, 4)*V_xx_prev(4, 4);
    const double internal_45 = Fx(0, 4)*V_xx_prev(0, 5) + Fx(1, 4)*V_xx_prev(1, 5) + Fx(2, 4)*V_xx_prev(2, 5) + Fx(3, 4)*V_xx_prev(3, 5) + Fx(4, 4)*V_xx_prev(4, 5);
    const double internal_46 = Fx(0, 4)*V_xx_prev(0, 6) + Fx(1, 4)*V_xx_prev(1, 6) + Fx(2, 4)*V_xx_prev(2, 6) + Fx(3, 4)*V_xx_prev(3, 6) + Fx(4, 4)*V_xx_prev(4, 6);
    const double internal_47 = Fx(0, 4)*V_xx_prev(0, 7) + Fx(1, 4)*V_xx_prev(1, 7) + Fx(2, 4)*V_xx_prev(2, 7) + Fx(3, 4)*V_xx_prev(3, 7) + Fx(4, 4)*V_xx_prev(4, 7);
    const double internal_48 = Fx(0, 4)*V_xx_prev(0, 9) + Fx(1, 4)*V_xx_prev(1, 9) + Fx(2, 4)*V_xx_prev(2, 9) + Fx(3, 4)*V_xx_prev(3, 9) + Fx(4, 4)*V_xx_prev(4, 9);
    const double internal_88 = Fx(5, 9)*V_xx_prev(5, 0) + Fx(6, 9)*V_xx_prev(6, 0) + Fx(7, 9)*V_xx_prev(7, 0) + Fx(9, 9)*V_xx_prev(9, 0);
    const double internal_89 = Fx(5, 9)*V_xx_prev(5, 1) + Fx(6, 9)*V_xx_prev(6, 1) + Fx(7, 9)*V_xx_prev(7, 1) + Fx(9, 9)*V_xx_prev(9, 1);
    const double internal_90 = Fx(5, 9)*V_xx_prev(5, 2) + Fx(6, 9)*V_xx_prev(6, 2) + Fx(7, 9)*V_xx_prev(7, 2) + Fx(9, 9)*V_xx_prev(9, 2);
    const double internal_91 = Fx(5, 9)*V_xx_prev(5, 3) + Fx(6, 9)*V_xx_prev(6, 3) + Fx(7, 9)*V_xx_prev(7, 3) + Fx(9, 9)*V_xx_prev(9, 3);
    const double internal_92 = Fx(5, 9)*V_xx_prev(5, 4) + Fx(6, 9)*V_xx_prev(6, 4) + Fx(7, 9)*V_xx_prev(7, 4) + Fx(9, 9)*V_xx_prev(9, 4);
    const double internal_96 = Fx(5, 9)*V_xx_prev(5, 9) + Fx(6, 9)*V_xx_prev(6, 9) + Fx(7, 9)*V_xx_prev(7, 9) + Fx(9, 9)*V_xx_prev(9, 9);
    const double internal_102 = Fu(1, 0)*V_xx_prev(1, 0) + Fu(2, 0)*V_xx_prev(2, 0) + Fu(3, 0)*V_xx_prev(3, 0) + Fu(4, 0)*V_xx_prev(4, 0) + internal_97;
    const double internal_106 = Fu(1, 1)*V_xx_prev(1, 0) + Fu(2, 1)*V_xx_prev(2, 0) + Fu(3, 1)*V_xx_prev(3, 0) + internal_98;
    const double internal_13 = Fx(1, 1)*V_xx_prev(1, 0) + internal_0;
    const double internal_22 = Fx(2, 2)*V_xx_prev(2, 0) + internal_2;
    const double internal_31 = Fx(1, 3)*V_xx_prev(1, 0) + Fx(2, 3)*V_xx_prev(2, 0) + Fx(3, 3)*V_xx_prev(3, 0) + internal_4;
    const double internal_40 = Fx(1, 4)*V_xx_prev(1, 0) + Fx(2, 4)*V_xx_prev(2, 0) + Fx(3, 4)*V_xx_prev(3, 0) + Fx(4, 4)*V_xx_prev(4, 0) + internal_6;
    const double internal_93 = Fx(6, 9)*V_xx_prev(6, 5) + Fx(7, 9)*V_xx_prev(7, 5) + Fx(9, 9)*V_xx_prev(9, 5) + internal_57;
    const double internal_94 = Fx(5, 9)*V_xx_prev(5, 6) + Fx(7, 9)*V_xx_prev(7, 6) + Fx(9, 9)*V_xx_prev(9, 6) + internal_67;
    const double internal_95 = Fx(5, 9)*V_xx_prev(5, 7) + Fx(6, 9)*V_xx_prev(6, 7) + Fx(9, 9)*V_xx_prev(9, 7) + internal_77;

    // Evaluation of Matrix Q_xx_
    Q_xx_(0, 0) = pow(Fx(0, 0), 2)*V_xx_prev(0, 0) + l_xx(0, 0);
    Q_xx_(0, 1) = Fx(0, 0)*internal_0 + Fx(1, 1)*internal_1;
    Q_xx_(0, 2) = Fx(0, 0)*internal_2 + Fx(2, 2)*internal_3;
    Q_xx_(0, 3) = Fx(0, 0)*internal_4 + Fx(1, 3)*internal_1 + Fx(2, 3)*internal_3 + Fx(3, 3)*internal_5;
    Q_xx_(0, 4) = Fx(0, 0)*internal_6 + Fx(1, 4)*internal_1 + Fx(2, 4)*internal_3 + Fx(3, 4)*internal_5 + Fx(4, 4)*internal_7;
    Q_xx_(0, 5) = Fx(5, 5)*internal_8;
    Q_xx_(0, 6) = Fx(6, 6)*internal_9;
    Q_xx_(0, 7) = Fx(7, 7)*internal_10;
    Q_xx_(0, 8) = V_xx_prev(0, 8)*internal_11;
    Q_xx_(0, 9) = Fx(5, 9)*internal_8 + Fx(6, 9)*internal_9 + Fx(7, 9)*internal_10 + Fx(9, 9)*internal_12;
    Q_xx_(1, 0) = Fx(0, 0)*internal_13;
    Q_xx_(1, 1) = Fx(0, 1)*internal_13 + Fx(1, 1)*internal_14 + l_xx(1, 1);
    Q_xx_(1, 2) = Fx(0, 2)*internal_13 + Fx(2, 2)*internal_15 + l_xx(1, 2);
    Q_xx_(1, 3) = Fx(0, 3)*internal_13 + Fx(1, 3)*internal_14 + Fx(2, 3)*internal_15 + Fx(3, 3)*internal_16;
    Q_xx_(1, 4) = Fx(0, 4)*internal_13 + Fx(1, 4)*internal_14 + Fx(2, 4)*internal_15 + Fx(3, 4)*internal_16 + Fx(4, 4)*internal_17;
    Q_xx_(1, 5) = Fx(5, 5)*internal_18;
    Q_xx_(1, 6) = Fx(6, 6)*internal_19;
    Q_xx_(1, 7) = Fx(7, 7)*internal_20;
    Q_xx_(1, 8) = Fx(8, 8)*(Fx(0, 1)*V_xx_prev(0, 8) + Fx(1, 1)*V_xx_prev(1, 8));
    Q_xx_(1, 9) = Fx(5, 9)*internal_18 + Fx(6, 9)*internal_19 + Fx(7, 9)*internal_20 + Fx(9, 9)*internal_21;
    Q_xx_(2, 0) = Fx(0, 0)*internal_22;
    Q_xx_(2, 1) = Fx(0, 1)*internal_22 + Fx(1, 1)*internal_23 + l_xx(2, 1);
    Q_xx_(2, 2) = Fx(0, 2)*internal_22 + Fx(2, 2)*internal_24 + l_xx(2, 2);
    Q_xx_(2, 3) = Fx(0, 3)*internal_22 + Fx(1, 3)*internal_23 + Fx(2, 3)*internal_24 + Fx(3, 3)*internal_25;
    Q_xx_(2, 4) = Fx(0, 4)*internal_22 + Fx(1, 4)*internal_23 + Fx(2, 4)*internal_24 + Fx(3, 4)*internal_25 + Fx(4, 4)*internal_26;
    Q_xx_(2, 5) = Fx(5, 5)*internal_27;
    Q_xx_(2, 6) = Fx(6, 6)*internal_28;
    Q_xx_(2, 7) = Fx(7, 7)*internal_29;
    Q_xx_(2, 8) = Fx(8, 8)*(Fx(0, 2)*V_xx_prev(0, 8) + Fx(2, 2)*V_xx_prev(2, 8));
    Q_xx_(2, 9) = Fx(5, 9)*internal_27 + Fx(6, 9)*internal_28 + Fx(7, 9)*internal_29 + Fx(9, 9)*internal_30;
    Q_xx_(3, 0) = Fx(0, 0)*internal_31;
    Q_xx_(3, 1) = Fx(0, 1)*internal_31 + Fx(1, 1)*internal_32;
    Q_xx_(3, 2) = Fx(0, 2)*internal_31 + Fx(2, 2)*internal_33;
    Q_xx_(3, 3) = Fx(0, 3)*internal_31 + Fx(1, 3)*internal_32 + Fx(2, 3)*internal_33 + Fx(3, 3)*internal_34 + l_xx(3, 3);
    Q_xx_(3, 4) = Fx(0, 4)*internal_31 + Fx(1, 4)*internal_32 + Fx(2, 4)*internal_33 + Fx(3, 4)*internal_34 + Fx(4, 4)*internal_35;
    Q_xx_(3, 5) = Fx(5, 5)*internal_36;
    Q_xx_(3, 6) = Fx(6, 6)*internal_37;
    Q_xx_(3, 7) = Fx(7, 7)*internal_38;
    Q_xx_(3, 8) = Fx(8, 8)*(Fx(0, 3)*V_xx_prev(0, 8) + Fx(1, 3)*V_xx_prev(1, 8) + Fx(2, 3)*V_xx_prev(2, 8) + Fx(3, 3)*V_xx_prev(3, 8));
    Q_xx_(3, 9) = Fx(5, 9)*internal_36 + Fx(6, 9)*internal_37 + Fx(7, 9)*internal_38 + Fx(9, 9)*internal_39;
    Q_xx_(4, 0) = Fx(0, 0)*internal_40;
    Q_xx_(4, 1) = Fx(0, 1)*internal_40 + Fx(1, 1)*internal_41;
    Q_xx_(4, 2) = Fx(0, 2)*internal_40 + Fx(2, 2)*internal_42;
    Q_xx_(4, 3) = Fx(0, 3)*internal_40 + Fx(1, 3)*internal_41 + Fx(2, 3)*internal_42 + Fx(3, 3)*internal_43;
    Q_xx_(4, 4) = Fx(0, 4)*internal_40 + Fx(1, 4)*internal_41 + Fx(2, 4)*internal_42 + Fx(3, 4)*internal_43 + Fx(4, 4)*internal_44 + l_xx(4, 4);
    Q_xx_(4, 5) = Fx(5, 5)*internal_45;
    Q_xx_(4, 6) = Fx(6, 6)*internal_46;
    Q_xx_(4, 7) = Fx(7, 7)*internal_47;
    Q_xx_(4, 8) = Fx(8, 8)*(Fx(0, 4)*V_xx_prev(0, 8) + Fx(1, 4)*V_xx_prev(1, 8) + Fx(2, 4)*V_xx_prev(2, 8) + Fx(3, 4)*V_xx_prev(3, 8) + Fx(4, 4)*V_xx_prev(4, 8));
    Q_xx_(4, 9) = Fx(5, 9)*internal_45 + Fx(6, 9)*internal_46 + Fx(7, 9)*internal_47 + Fx(9, 9)*internal_48;
    Q_xx_(5, 0) = Fx(0, 0)*internal_49;
    Q_xx_(5, 1) = Fx(0, 1)*internal_49 + Fx(1, 1)*internal_50;
    Q_xx_(5, 2) = Fx(0, 2)*internal_49 + Fx(2, 2)*internal_51;
    Q_xx_(5, 3) = Fx(0, 3)*internal_49 + Fx(1, 3)*internal_50 + Fx(2, 3)*internal_51 + Fx(3, 3)*internal_52;
    Q_xx_(5, 4) = Fx(0, 4)*internal_49 + Fx(1, 4)*internal_50 + Fx(2, 4)*internal_51 + Fx(3, 4)*internal_52 + Fx(4, 4)*internal_53;
    Q_xx_(5, 5) = pow(Fx(5, 5), 2)*V_xx_prev(5, 5) + l_xx(5, 5);
    Q_xx_(5, 6) = Fx(6, 6)*internal_54;
    Q_xx_(5, 7) = Fx(7, 7)*internal_55;
    Q_xx_(5, 8) = V_xx_prev(5, 8)*internal_56;
    Q_xx_(5, 9) = Fx(5, 5)*internal_57 + Fx(6, 9)*internal_54 + Fx(7, 9)*internal_55 + Fx(9, 9)*internal_58;
    Q_xx_(6, 0) = Fx(0, 0)*internal_59;
    Q_xx_(6, 1) = Fx(0, 1)*internal_59 + Fx(1, 1)*internal_60;
    Q_xx_(6, 2) = Fx(0, 2)*internal_59 + Fx(2, 2)*internal_61;
    Q_xx_(6, 3) = Fx(0, 3)*internal_59 + Fx(1, 3)*internal_60 + Fx(2, 3)*internal_61 + Fx(3, 3)*internal_62;
    Q_xx_(6, 4) = Fx(0, 4)*internal_59 + Fx(1, 4)*internal_60 + Fx(2, 4)*internal_61 + Fx(3, 4)*internal_62 + Fx(4, 4)*internal_63;
    Q_xx_(6, 5) = Fx(5, 5)*internal_64;
    Q_xx_(6, 6) = pow(Fx(6, 6), 2)*V_xx_prev(6, 6);
    Q_xx_(6, 7) = Fx(7, 7)*internal_65;
    Q_xx_(6, 8) = V_xx_prev(6, 8)*internal_66;
    Q_xx_(6, 9) = Fx(5, 9)*internal_64 + Fx(6, 6)*internal_67 + Fx(7, 9)*internal_65 + Fx(9, 9)*internal_68;
    Q_xx_(7, 0) = Fx(0, 0)*internal_69;
    Q_xx_(7, 1) = Fx(0, 1)*internal_69 + Fx(1, 1)*internal_70;
    Q_xx_(7, 2) = Fx(0, 2)*internal_69 + Fx(2, 2)*internal_71;
    Q_xx_(7, 3) = Fx(0, 3)*internal_69 + Fx(1, 3)*internal_70 + Fx(2, 3)*internal_71 + Fx(3, 3)*internal_72;
    Q_xx_(7, 4) = Fx(0, 4)*internal_69 + Fx(1, 4)*internal_70 + Fx(2, 4)*internal_71 + Fx(3, 4)*internal_72 + Fx(4, 4)*internal_73;
    Q_xx_(7, 5) = Fx(5, 5)*internal_74;
    Q_xx_(7, 6) = Fx(6, 6)*internal_75;
    Q_xx_(7, 7) = pow(Fx(7, 7), 2)*V_xx_prev(7, 7);
    Q_xx_(7, 8) = V_xx_prev(7, 8)*internal_76;
    Q_xx_(7, 9) = Fx(5, 9)*internal_74 + Fx(6, 9)*internal_75 + Fx(7, 7)*internal_77 + Fx(9, 9)*internal_78;
    Q_xx_(8, 0) = V_xx_prev(8, 0)*internal_11;
    Q_xx_(8, 1) = Fx(0, 1)*internal_79 + Fx(1, 1)*internal_80;
    Q_xx_(8, 2) = Fx(0, 2)*internal_79 + Fx(2, 2)*internal_81;
    Q_xx_(8, 3) = Fx(0, 3)*internal_79 + Fx(1, 3)*internal_80 + Fx(2, 3)*internal_81 + Fx(3, 3)*internal_82;
    Q_xx_(8, 4) = Fx(0, 4)*internal_79 + Fx(1, 4)*internal_80 + Fx(2, 4)*internal_81 + Fx(3, 4)*internal_82 + Fx(4, 4)*internal_83;
    Q_xx_(8, 5) = V_xx_prev(8, 5)*internal_56;
    Q_xx_(8, 6) = V_xx_prev(8, 6)*internal_66;
    Q_xx_(8, 7) = V_xx_prev(8, 7)*internal_76;
    Q_xx_(8, 8) = pow(Fx(8, 8), 2)*V_xx_prev(8, 8);
    Q_xx_(8, 9) = Fx(5, 9)*internal_84 + Fx(6, 9)*internal_85 + Fx(7, 9)*internal_86 + Fx(9, 9)*internal_87;
    Q_xx_(9, 0) = Fx(0, 0)*internal_88;
    Q_xx_(9, 1) = Fx(0, 1)*internal_88 + Fx(1, 1)*internal_89;
    Q_xx_(9, 2) = Fx(0, 2)*internal_88 + Fx(2, 2)*internal_90;
    Q_xx_(9, 3) = Fx(0, 3)*internal_88 + Fx(1, 3)*internal_89 + Fx(2, 3)*internal_90 + Fx(3, 3)*internal_91;
    Q_xx_(9, 4) = Fx(0, 4)*internal_88 + Fx(1, 4)*internal_89 + Fx(2, 4)*internal_90 + Fx(3, 4)*internal_91 + Fx(4, 4)*internal_92;
    Q_xx_(9, 5) = Fx(5, 5)*internal_93;
    Q_xx_(9, 6) = Fx(6, 6)*internal_94;
    Q_xx_(9, 7) = Fx(7, 7)*internal_95;
    Q_xx_(9, 8) = Fx(8, 8)*(Fx(5, 9)*V_xx_prev(5, 8) + Fx(6, 9)*V_xx_prev(6, 8) + Fx(7, 9)*V_xx_prev(7, 8) + Fx(9, 9)*V_xx_prev(9, 8));
    Q_xx_(9, 9) = Fx(5, 9)*internal_93 + Fx(6, 9)*internal_94 + Fx(7, 9)*internal_95 + Fx(9, 9)*internal_96 + l_xx(9, 9);

    // Evaluation of Matrix Q_xu_
    Q_xu_(0, 0) = Fu(1, 0)*internal_1 + Fu(2, 0)*internal_3 + Fu(3, 0)*internal_5 + Fu(4, 0)*internal_7 + Fx(0, 0)*internal_97;
    Q_xu_(0, 1) = Fu(1, 1)*internal_1 + Fu(2, 1)*internal_3 + Fu(3, 1)*internal_5 + Fx(0, 0)*internal_98;
    Q_xu_(0, 2) = Fu(5, 2)*internal_8 + Fu(6, 2)*internal_9 + Fu(7, 2)*internal_10 + Fu(9, 2)*internal_12;
    Q_xu_(1, 0) = Fu(0, 0)*internal_13 + Fu(1, 0)*internal_14 + Fu(2, 0)*internal_15 + Fu(3, 0)*internal_16 + Fu(4, 0)*internal_17;
    Q_xu_(1, 1) = Fu(0, 1)*internal_13 + Fu(1, 1)*internal_14 + Fu(2, 1)*internal_15 + Fu(3, 1)*internal_16;
    Q_xu_(1, 2) = Fu(5, 2)*internal_18 + Fu(6, 2)*internal_19 + Fu(7, 2)*internal_20 + Fu(9, 2)*internal_21;
    Q_xu_(2, 0) = Fu(0, 0)*internal_22 + Fu(1, 0)*internal_23 + Fu(2, 0)*internal_24 + Fu(3, 0)*internal_25 + Fu(4, 0)*internal_26;
    Q_xu_(2, 1) = Fu(0, 1)*internal_22 + Fu(1, 1)*internal_23 + Fu(2, 1)*internal_24 + Fu(3, 1)*internal_25;
    Q_xu_(2, 2) = Fu(5, 2)*internal_27 + Fu(6, 2)*internal_28 + Fu(7, 2)*internal_29 + Fu(9, 2)*internal_30;
    Q_xu_(3, 0) = Fu(0, 0)*internal_31 + Fu(1, 0)*internal_32 + Fu(2, 0)*internal_33 + Fu(3, 0)*internal_34 + Fu(4, 0)*internal_35;
    Q_xu_(3, 1) = Fu(0, 1)*internal_31 + Fu(1, 1)*internal_32 + Fu(2, 1)*internal_33 + Fu(3, 1)*internal_34;
    Q_xu_(3, 2) = Fu(5, 2)*internal_36 + Fu(6, 2)*internal_37 + Fu(7, 2)*internal_38 + Fu(9, 2)*internal_39;
    Q_xu_(4, 0) = Fu(0, 0)*internal_40 + Fu(1, 0)*internal_41 + Fu(2, 0)*internal_42 + Fu(3, 0)*internal_43 + Fu(4, 0)*internal_44;
    Q_xu_(4, 1) = Fu(0, 1)*internal_40 + Fu(1, 1)*internal_41 + Fu(2, 1)*internal_42 + Fu(3, 1)*internal_43;
    Q_xu_(4, 2) = Fu(5, 2)*internal_45 + Fu(6, 2)*internal_46 + Fu(7, 2)*internal_47 + Fu(9, 2)*internal_48;
    Q_xu_(5, 0) = Fu(0, 0)*internal_49 + Fu(1, 0)*internal_50 + Fu(2, 0)*internal_51 + Fu(3, 0)*internal_52 + Fu(4, 0)*internal_53;
    Q_xu_(5, 1) = Fu(0, 1)*internal_49 + Fu(1, 1)*internal_50 + Fu(2, 1)*internal_51 + Fu(3, 1)*internal_52;
    Q_xu_(5, 2) = Fu(6, 2)*internal_54 + Fu(7, 2)*internal_55 + Fu(9, 2)*internal_58 + Fx(5, 5)*internal_99;
    Q_xu_(6, 0) = Fu(0, 0)*internal_59 + Fu(1, 0)*internal_60 + Fu(2, 0)*internal_61 + Fu(3, 0)*internal_62 + Fu(4, 0)*internal_63;
    Q_xu_(6, 1) = Fu(0, 1)*internal_59 + Fu(1, 1)*internal_60 + Fu(2, 1)*internal_61 + Fu(3, 1)*internal_62;
    Q_xu_(6, 2) = Fu(5, 2)*internal_64 + Fu(7, 2)*internal_65 + Fu(9, 2)*internal_68 + Fx(6, 6)*internal_100;
    Q_xu_(7, 0) = Fu(0, 0)*internal_69 + Fu(1, 0)*internal_70 + Fu(2, 0)*internal_71 + Fu(3, 0)*internal_72 + Fu(4, 0)*internal_73;
    Q_xu_(7, 1) = Fu(0, 1)*internal_69 + Fu(1, 1)*internal_70 + Fu(2, 1)*internal_71 + Fu(3, 1)*internal_72;
    Q_xu_(7, 2) = Fu(5, 2)*internal_74 + Fu(6, 2)*internal_75 + Fu(9, 2)*internal_78 + Fx(7, 7)*internal_101;
    Q_xu_(8, 0) = Fu(0, 0)*internal_79 + Fu(1, 0)*internal_80 + Fu(2, 0)*internal_81 + Fu(3, 0)*internal_82 + Fu(4, 0)*internal_83;
    Q_xu_(8, 1) = Fu(0, 1)*internal_79 + Fu(1, 1)*internal_80 + Fu(2, 1)*internal_81 + Fu(3, 1)*internal_82;
    Q_xu_(8, 2) = Fu(5, 2)*internal_84 + Fu(6, 2)*internal_85 + Fu(7, 2)*internal_86 + Fu(9, 2)*internal_87;
    Q_xu_(9, 0) = Fu(0, 0)*internal_88 + Fu(1, 0)*internal_89 + Fu(2, 0)*internal_90 + Fu(3, 0)*internal_91 + Fu(4, 0)*internal_92;
    Q_xu_(9, 1) = Fu(0, 1)*internal_88 + Fu(1, 1)*internal_89 + Fu(2, 1)*internal_90 + Fu(3, 1)*internal_91;
    Q_xu_(9, 2) = Fu(5, 2)*internal_93 + Fu(6, 2)*internal_94 + Fu(7, 2)*internal_95 + Fu(9, 2)*internal_96;

    // Evaluation of Matrix Q_uu_
    Q_uu_(0, 0) = Fu(0, 0)*internal_102 + Fu(1, 0)*internal_103 + Fu(2, 0)*internal_104 + Fu(3, 0)*internal_105 + Fu(4, 0)*(Fu(0, 0)*V_xx_prev(0, 4) + Fu(1, 0)*V_xx_prev(1, 4) + Fu(2, 0)*V_xx_prev(2, 4) + Fu(3, 0)*V_xx_prev(3, 4) + Fu(4, 0)*V_xx_prev(4, 4)) + Sigma_(8)*pow(g_u(8, 0), 2) + Sigma_(9)*pow(g_u(9, 0), 2) + l_uu(0, 0);
    Q_uu_(0, 1) = Fu(0, 1)*internal_102 + Fu(1, 1)*internal_103 + Fu(2, 1)*internal_104 + Fu(3, 1)*internal_105;
    Q_uu_(0, 2) = Fu(5, 2)*(Fu(0, 0)*V_xx_prev(0, 5) + Fu(1, 0)*V_xx_prev(1, 5) + Fu(2, 0)*V_xx_prev(2, 5) + Fu(3, 0)*V_xx_prev(3, 5) + Fu(4, 0)*V_xx_prev(4, 5)) + Fu(6, 2)*(Fu(0, 0)*V_xx_prev(0, 6) + Fu(1, 0)*V_xx_prev(1, 6) + Fu(2, 0)*V_xx_prev(2, 6) + Fu(3, 0)*V_xx_prev(3, 6) + Fu(4, 0)*V_xx_prev(4, 6)) + Fu(7, 2)*(Fu(0, 0)*V_xx_prev(0, 7) + Fu(1, 0)*V_xx_prev(1, 7) + Fu(2, 0)*V_xx_prev(2, 7) + Fu(3, 0)*V_xx_prev(3, 7) + Fu(4, 0)*V_xx_prev(4, 7)) + Fu(9, 2)*(Fu(0, 0)*V_xx_prev(0, 9) + Fu(1, 0)*V_xx_prev(1, 9) + Fu(2, 0)*V_xx_prev(2, 9) + Fu(3, 0)*V_xx_prev(3, 9) + Fu(4, 0)*V_xx_prev(4, 9));
    Q_uu_(1, 0) = Fu(0, 0)*internal_106 + Fu(1, 0)*internal_107 + Fu(2, 0)*internal_108 + Fu(3, 0)*internal_109 + Fu(4, 0)*(Fu(0, 1)*V_xx_prev(0, 4) + Fu(1, 1)*V_xx_prev(1, 4) + Fu(2, 1)*V_xx_prev(2, 4) + Fu(3, 1)*V_xx_prev(3, 4));
    Q_uu_(1, 1) = Fu(0, 1)*internal_106 + Fu(1, 1)*internal_107 + Fu(2, 1)*internal_108 + Fu(3, 1)*internal_109 + Sigma_(6)*pow(g_u(6, 1), 2) + Sigma_(7)*pow(g_u(7, 1), 2) + l_uu(1, 1);
    Q_uu_(1, 2) = Fu(5, 2)*(Fu(0, 1)*V_xx_prev(0, 5) + Fu(1, 1)*V_xx_prev(1, 5) + Fu(2, 1)*V_xx_prev(2, 5) + Fu(3, 1)*V_xx_prev(3, 5)) + Fu(6, 2)*(Fu(0, 1)*V_xx_prev(0, 6) + Fu(1, 1)*V_xx_prev(1, 6) + Fu(2, 1)*V_xx_prev(2, 6) + Fu(3, 1)*V_xx_prev(3, 6)) + Fu(7, 2)*(Fu(0, 1)*V_xx_prev(0, 7) + Fu(1, 1)*V_xx_prev(1, 7) + Fu(2, 1)*V_xx_prev(2, 7) + Fu(3, 1)*V_xx_prev(3, 7)) + Fu(9, 2)*(Fu(0, 1)*V_xx_prev(0, 9) + Fu(1, 1)*V_xx_prev(1, 9) + Fu(2, 1)*V_xx_prev(2, 9) + Fu(3, 1)*V_xx_prev(3, 9));
    Q_uu_(2, 0) = Fu(0, 0)*internal_110 + Fu(1, 0)*internal_111 + Fu(2, 0)*internal_112 + Fu(3, 0)*internal_113 + Fu(4, 0)*(Fu(5, 2)*V_xx_prev(5, 4) + Fu(6, 2)*V_xx_prev(6, 4) + Fu(7, 2)*V_xx_prev(7, 4) + Fu(9, 2)*V_xx_prev(9, 4));
    Q_uu_(2, 1) = Fu(0, 1)*internal_110 + Fu(1, 1)*internal_111 + Fu(2, 1)*internal_112 + Fu(3, 1)*internal_113;
    Q_uu_(2, 2) = Fu(5, 2)*(Fu(6, 2)*V_xx_prev(6, 5) + Fu(7, 2)*V_xx_prev(7, 5) + Fu(9, 2)*V_xx_prev(9, 5) + internal_99) + Fu(6, 2)*(Fu(5, 2)*V_xx_prev(5, 6) + Fu(7, 2)*V_xx_prev(7, 6) + Fu(9, 2)*V_xx_prev(9, 6) + internal_100) + Fu(7, 2)*(Fu(5, 2)*V_xx_prev(5, 7) + Fu(6, 2)*V_xx_prev(6, 7) + Fu(9, 2)*V_xx_prev(9, 7) + internal_101) + Fu(9, 2)*(Fu(5, 2)*V_xx_prev(5, 9) + Fu(6, 2)*V_xx_prev(6, 9) + Fu(7, 2)*V_xx_prev(7, 9) + Fu(9, 2)*V_xx_prev(9, 9)) + Sigma_(10)*pow(g_u(10, 2), 2) + Sigma_(11)*pow(g_u(11, 2), 2) + l_uu(2, 2);
    Q_uu_(3, 3) = Sigma_(0)*pow(g_u(0, 3), 2);
    Q_uu_(4, 4) = Sigma_(1)*pow(g_u(1, 4), 2);
    Q_uu_(5, 5) = Sigma_(2)*pow(g_u(2, 5), 2);
    Q_uu_(6, 6) = Sigma_(3)*pow(g_u(3, 6), 2);
    Q_uu_(7, 7) = Sigma_(4)*pow(g_u(4, 7), 2);
    Q_uu_(8, 8) = Sigma_(5)*pow(g_u(5, 8), 2);

  }

  virtual void updateQG(const Eigen::VectorXd& sl_x, const Eigen::VectorXd& sl_u, const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::VectorXd& r_f, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& V_xx_prev, const Eigen::VectorXd& V_x_prev) override {
    // Determine internal variables
    const double internal_0 = V_x_prev(0) + V_xx_prev(0, 0)*r_f(0) + V_xx_prev(0, 1)*r_f(1) + V_xx_prev(0, 2)*r_f(2) + V_xx_prev(0, 3)*r_f(3) + V_xx_prev(0, 4)*r_f(4) + V_xx_prev(0, 5)*r_f(5) + V_xx_prev(0, 6)*r_f(6) + V_xx_prev(0, 7)*r_f(7) + V_xx_prev(0, 8)*r_f(8) + V_xx_prev(0, 9)*r_f(9);
    const double internal_1 = V_x_prev(1) + V_xx_prev(1, 0)*r_f(0) + V_xx_prev(1, 1)*r_f(1) + V_xx_prev(1, 2)*r_f(2) + V_xx_prev(1, 3)*r_f(3) + V_xx_prev(1, 4)*r_f(4) + V_xx_prev(1, 5)*r_f(5) + V_xx_prev(1, 6)*r_f(6) + V_xx_prev(1, 7)*r_f(7) + V_xx_prev(1, 8)*r_f(8) + V_xx_prev(1, 9)*r_f(9);
    const double internal_2 = V_x_prev(2) + V_xx_prev(2, 0)*r_f(0) + V_xx_prev(2, 1)*r_f(1) + V_xx_prev(2, 2)*r_f(2) + V_xx_prev(2, 3)*r_f(3) + V_xx_prev(2, 4)*r_f(4) + V_xx_prev(2, 5)*r_f(5) + V_xx_prev(2, 6)*r_f(6) + V_xx_prev(2, 7)*r_f(7) + V_xx_prev(2, 8)*r_f(8) + V_xx_prev(2, 9)*r_f(9);
    const double internal_3 = V_x_prev(3) + V_xx_prev(3, 0)*r_f(0) + V_xx_prev(3, 1)*r_f(1) + V_xx_prev(3, 2)*r_f(2) + V_xx_prev(3, 3)*r_f(3) + V_xx_prev(3, 4)*r_f(4) + V_xx_prev(3, 5)*r_f(5) + V_xx_prev(3, 6)*r_f(6) + V_xx_prev(3, 7)*r_f(7) + V_xx_prev(3, 8)*r_f(8) + V_xx_prev(3, 9)*r_f(9);
    const double internal_4 = V_x_prev(4) + V_xx_prev(4, 0)*r_f(0) + V_xx_prev(4, 1)*r_f(1) + V_xx_prev(4, 2)*r_f(2) + V_xx_prev(4, 3)*r_f(3) + V_xx_prev(4, 4)*r_f(4) + V_xx_prev(4, 5)*r_f(5) + V_xx_prev(4, 6)*r_f(6) + V_xx_prev(4, 7)*r_f(7) + V_xx_prev(4, 8)*r_f(8) + V_xx_prev(4, 9)*r_f(9);
    const double internal_5 = V_x_prev(5) + V_xx_prev(5, 0)*r_f(0) + V_xx_prev(5, 1)*r_f(1) + V_xx_prev(5, 2)*r_f(2) + V_xx_prev(5, 3)*r_f(3) + V_xx_prev(5, 4)*r_f(4) + V_xx_prev(5, 5)*r_f(5) + V_xx_prev(5, 6)*r_f(6) + V_xx_prev(5, 7)*r_f(7) + V_xx_prev(5, 8)*r_f(8) + V_xx_prev(5, 9)*r_f(9);
    const double internal_6 = V_x_prev(6) + V_xx_prev(6, 0)*r_f(0) + V_xx_prev(6, 1)*r_f(1) + V_xx_prev(6, 2)*r_f(2) + V_xx_prev(6, 3)*r_f(3) + V_xx_prev(6, 4)*r_f(4) + V_xx_prev(6, 5)*r_f(5) + V_xx_prev(6, 6)*r_f(6) + V_xx_prev(6, 7)*r_f(7) + V_xx_prev(6, 8)*r_f(8) + V_xx_prev(6, 9)*r_f(9);
    const double internal_7 = V_x_prev(7) + V_xx_prev(7, 0)*r_f(0) + V_xx_prev(7, 1)*r_f(1) + V_xx_prev(7, 2)*r_f(2) + V_xx_prev(7, 3)*r_f(3) + V_xx_prev(7, 4)*r_f(4) + V_xx_prev(7, 5)*r_f(5) + V_xx_prev(7, 6)*r_f(6) + V_xx_prev(7, 7)*r_f(7) + V_xx_prev(7, 8)*r_f(8) + V_xx_prev(7, 9)*r_f(9);
    const double internal_8 = V_x_prev(9) + V_xx_prev(9, 0)*r_f(0) + V_xx_prev(9, 1)*r_f(1) + V_xx_prev(9, 2)*r_f(2) + V_xx_prev(9, 3)*r_f(3) + V_xx_prev(9, 4)*r_f(4) + V_xx_prev(9, 5)*r_f(5) + V_xx_prev(9, 6)*r_f(6) + V_xx_prev(9, 7)*r_f(7) + V_xx_prev(9, 8)*r_f(8) + V_xx_prev(9, 9)*r_f(9);

    // Evaluation of Vector Q_x_
    Q_x_(0) = Fx(0, 0)*internal_0 + sl_x(0);
    Q_x_(1) = Fx(0, 1)*internal_0 + Fx(1, 1)*internal_1 + sl_x(1);
    Q_x_(2) = Fx(0, 2)*internal_0 + Fx(2, 2)*internal_2 + sl_x(2);
    Q_x_(3) = Fx(0, 3)*internal_0 + Fx(1, 3)*internal_1 + Fx(2, 3)*internal_2 + Fx(3, 3)*internal_3 + sl_x(3);
    Q_x_(4) = Fx(0, 4)*internal_0 + Fx(1, 4)*internal_1 + Fx(2, 4)*internal_2 + Fx(3, 4)*internal_3 + Fx(4, 4)*internal_4 + sl_x(4);
    Q_x_(5) = Fx(5, 5)*internal_5 + sl_x(5);
    Q_x_(6) = Fx(6, 6)*internal_6 + sl_x(6);
    Q_x_(7) = Fx(7, 7)*internal_7 + sl_x(7);
    Q_x_(8) = Fx(8, 8)*(V_x_prev(8) + V_xx_prev(8, 0)*r_f(0) + V_xx_prev(8, 1)*r_f(1) + V_xx_prev(8, 2)*r_f(2) + V_xx_prev(8, 3)*r_f(3) + V_xx_prev(8, 4)*r_f(4) + V_xx_prev(8, 5)*r_f(5) + V_xx_prev(8, 6)*r_f(6) + V_xx_prev(8, 7)*r_f(7) + V_xx_prev(8, 8)*r_f(8) + V_xx_prev(8, 9)*r_f(9)) + sl_x(8);
    Q_x_(9) = Fx(5, 9)*internal_5 + Fx(6, 9)*internal_6 + Fx(7, 9)*internal_7 + Fx(9, 9)*internal_8 + sl_x(9);

    // Evaluation of Vector Q_u_
    Q_u_(0) = Fu(0, 0)*internal_0 + Fu(1, 0)*internal_1 + Fu(2, 0)*internal_2 + Fu(3, 0)*internal_3 + Fu(4, 0)*internal_4 + V_(8)*g_u(8, 0) + V_(9)*g_u(9, 0) + sl_u(0);
    Q_u_(1) = Fu(0, 1)*internal_0 + Fu(1, 1)*internal_1 + Fu(2, 1)*internal_2 + Fu(3, 1)*internal_3 + V_(6)*g_u(6, 1) + V_(7)*g_u(7, 1) + sl_u(1);
    Q_u_(2) = Fu(5, 2)*internal_5 + Fu(6, 2)*internal_6 + Fu(7, 2)*internal_7 + Fu(9, 2)*internal_8 + V_(10)*g_u(10, 2) + V_(11)*g_u(11, 2) + sl_u(2);
    Q_u_(3) = V_(0)*g_u(0, 3) + sl_u(3);
    Q_u_(4) = V_(1)*g_u(1, 4) + sl_u(4);
    Q_u_(5) = V_(2)*g_u(2, 5) + sl_u(5);
    Q_u_(6) = V_(3)*g_u(3, 6) + sl_u(6);
    Q_u_(7) = V_(4)*g_u(4, 7) + sl_u(7);
    Q_u_(8) = V_(5)*g_u(5, 8) + sl_u(8);

  }

  virtual void updateQuuInv() override {
    // Determine internal variables
    const double internal_0 = 1.0/Q_uu_(0, 0);
    const double internal_3 = Q_uu_(0, 1)*internal_0;
    const double internal_5 = Q_uu_(0, 2)*internal_0;
    const double internal_1 = -pow(Q_uu_(0, 1), 2)*internal_0 + Q_uu_(1, 1);
    const double internal_2 = 1.0/internal_1;
    const double internal_6 = -Q_uu_(0, 1)*internal_5 + Q_uu_(1, 2);
    const double internal_4 = internal_2*internal_3;
    const double internal_7 = pow(internal_6, 2);
    const double internal_9 = internal_4*internal_6;
    const double internal_8 = 1.0/(-pow(Q_uu_(0, 2), 2)*internal_0 + Q_uu_(2, 2) - internal_2*internal_7);
    const double internal_10 = internal_8*(-internal_5 + internal_9);
    const double internal_13 = -internal_2*internal_6*internal_8;
    const double internal_12 = internal_2 + internal_7*internal_8/pow(internal_1, 2);
    const double internal_11 = -internal_10*internal_2*internal_6 - internal_4;

    // Evaluation of Matrix Q_uu_inv_
    Q_uu_inv_(0, 0) = internal_0 - internal_10*internal_5 - internal_11*internal_3;
    Q_uu_inv_(0, 1) = Q_uu_(0, 2)*internal_0*internal_2*internal_6*internal_8 - internal_12*internal_3;
    Q_uu_inv_(0, 2) = -internal_5*internal_8 + internal_8*internal_9;
    Q_uu_inv_(1, 0) = internal_11;
    Q_uu_inv_(1, 1) = internal_12;
    Q_uu_inv_(1, 2) = internal_13;
    Q_uu_inv_(2, 0) = internal_10;
    Q_uu_inv_(2, 1) = internal_13;
    Q_uu_inv_(2, 2) = internal_8;
    Q_uu_inv_(3, 3) = 1.0/Q_uu_(3, 3);
    Q_uu_inv_(4, 4) = 1.0/Q_uu_(4, 4);
    Q_uu_inv_(5, 5) = 1.0/Q_uu_(5, 5);
    Q_uu_inv_(6, 6) = 1.0/Q_uu_(6, 6);
    Q_uu_inv_(7, 7) = 1.0/Q_uu_(7, 7);
    Q_uu_inv_(8, 8) = 1.0/Q_uu_(8, 8);

  }

  virtual void updateKVU(Eigen::MatrixXd& k_ux, Eigen::VectorXd& v_u) override {
    // Evaluation of Matrix k_ux
    k_ux(0, 0) = -Q_uu_inv_(0, 0)*Q_xu_(0, 0) - Q_uu_inv_(0, 1)*Q_xu_(0, 1) - Q_uu_inv_(0, 2)*Q_xu_(0, 2);
    k_ux(0, 1) = -Q_uu_inv_(0, 0)*Q_xu_(1, 0) - Q_uu_inv_(0, 1)*Q_xu_(1, 1) - Q_uu_inv_(0, 2)*Q_xu_(1, 2);
    k_ux(0, 2) = -Q_uu_inv_(0, 0)*Q_xu_(2, 0) - Q_uu_inv_(0, 1)*Q_xu_(2, 1) - Q_uu_inv_(0, 2)*Q_xu_(2, 2);
    k_ux(0, 3) = -Q_uu_inv_(0, 0)*Q_xu_(3, 0) - Q_uu_inv_(0, 1)*Q_xu_(3, 1) - Q_uu_inv_(0, 2)*Q_xu_(3, 2);
    k_ux(0, 4) = -Q_uu_inv_(0, 0)*Q_xu_(4, 0) - Q_uu_inv_(0, 1)*Q_xu_(4, 1) - Q_uu_inv_(0, 2)*Q_xu_(4, 2);
    k_ux(0, 5) = -Q_uu_inv_(0, 0)*Q_xu_(5, 0) - Q_uu_inv_(0, 1)*Q_xu_(5, 1) - Q_uu_inv_(0, 2)*Q_xu_(5, 2);
    k_ux(0, 6) = -Q_uu_inv_(0, 0)*Q_xu_(6, 0) - Q_uu_inv_(0, 1)*Q_xu_(6, 1) - Q_uu_inv_(0, 2)*Q_xu_(6, 2);
    k_ux(0, 7) = -Q_uu_inv_(0, 0)*Q_xu_(7, 0) - Q_uu_inv_(0, 1)*Q_xu_(7, 1) - Q_uu_inv_(0, 2)*Q_xu_(7, 2);
    k_ux(0, 8) = -Q_uu_inv_(0, 0)*Q_xu_(8, 0) - Q_uu_inv_(0, 1)*Q_xu_(8, 1) - Q_uu_inv_(0, 2)*Q_xu_(8, 2);
    k_ux(0, 9) = -Q_uu_inv_(0, 0)*Q_xu_(9, 0) - Q_uu_inv_(0, 1)*Q_xu_(9, 1) - Q_uu_inv_(0, 2)*Q_xu_(9, 2);
    k_ux(1, 0) = -Q_uu_inv_(0, 1)*Q_xu_(0, 0) - Q_uu_inv_(1, 1)*Q_xu_(0, 1) - Q_uu_inv_(1, 2)*Q_xu_(0, 2);
    k_ux(1, 1) = -Q_uu_inv_(0, 1)*Q_xu_(1, 0) - Q_uu_inv_(1, 1)*Q_xu_(1, 1) - Q_uu_inv_(1, 2)*Q_xu_(1, 2);
    k_ux(1, 2) = -Q_uu_inv_(0, 1)*Q_xu_(2, 0) - Q_uu_inv_(1, 1)*Q_xu_(2, 1) - Q_uu_inv_(1, 2)*Q_xu_(2, 2);
    k_ux(1, 3) = -Q_uu_inv_(0, 1)*Q_xu_(3, 0) - Q_uu_inv_(1, 1)*Q_xu_(3, 1) - Q_uu_inv_(1, 2)*Q_xu_(3, 2);
    k_ux(1, 4) = -Q_uu_inv_(0, 1)*Q_xu_(4, 0) - Q_uu_inv_(1, 1)*Q_xu_(4, 1) - Q_uu_inv_(1, 2)*Q_xu_(4, 2);
    k_ux(1, 5) = -Q_uu_inv_(0, 1)*Q_xu_(5, 0) - Q_uu_inv_(1, 1)*Q_xu_(5, 1) - Q_uu_inv_(1, 2)*Q_xu_(5, 2);
    k_ux(1, 6) = -Q_uu_inv_(0, 1)*Q_xu_(6, 0) - Q_uu_inv_(1, 1)*Q_xu_(6, 1) - Q_uu_inv_(1, 2)*Q_xu_(6, 2);
    k_ux(1, 7) = -Q_uu_inv_(0, 1)*Q_xu_(7, 0) - Q_uu_inv_(1, 1)*Q_xu_(7, 1) - Q_uu_inv_(1, 2)*Q_xu_(7, 2);
    k_ux(1, 8) = -Q_uu_inv_(0, 1)*Q_xu_(8, 0) - Q_uu_inv_(1, 1)*Q_xu_(8, 1) - Q_uu_inv_(1, 2)*Q_xu_(8, 2);
    k_ux(1, 9) = -Q_uu_inv_(0, 1)*Q_xu_(9, 0) - Q_uu_inv_(1, 1)*Q_xu_(9, 1) - Q_uu_inv_(1, 2)*Q_xu_(9, 2);
    k_ux(2, 0) = -Q_uu_inv_(0, 2)*Q_xu_(0, 0) - Q_uu_inv_(1, 2)*Q_xu_(0, 1) - Q_uu_inv_(2, 2)*Q_xu_(0, 2);
    k_ux(2, 1) = -Q_uu_inv_(0, 2)*Q_xu_(1, 0) - Q_uu_inv_(1, 2)*Q_xu_(1, 1) - Q_uu_inv_(2, 2)*Q_xu_(1, 2);
    k_ux(2, 2) = -Q_uu_inv_(0, 2)*Q_xu_(2, 0) - Q_uu_inv_(1, 2)*Q_xu_(2, 1) - Q_uu_inv_(2, 2)*Q_xu_(2, 2);
    k_ux(2, 3) = -Q_uu_inv_(0, 2)*Q_xu_(3, 0) - Q_uu_inv_(1, 2)*Q_xu_(3, 1) - Q_uu_inv_(2, 2)*Q_xu_(3, 2);
    k_ux(2, 4) = -Q_uu_inv_(0, 2)*Q_xu_(4, 0) - Q_uu_inv_(1, 2)*Q_xu_(4, 1) - Q_uu_inv_(2, 2)*Q_xu_(4, 2);
    k_ux(2, 5) = -Q_uu_inv_(0, 2)*Q_xu_(5, 0) - Q_uu_inv_(1, 2)*Q_xu_(5, 1) - Q_uu_inv_(2, 2)*Q_xu_(5, 2);
    k_ux(2, 6) = -Q_uu_inv_(0, 2)*Q_xu_(6, 0) - Q_uu_inv_(1, 2)*Q_xu_(6, 1) - Q_uu_inv_(2, 2)*Q_xu_(6, 2);
    k_ux(2, 7) = -Q_uu_inv_(0, 2)*Q_xu_(7, 0) - Q_uu_inv_(1, 2)*Q_xu_(7, 1) - Q_uu_inv_(2, 2)*Q_xu_(7, 2);
    k_ux(2, 8) = -Q_uu_inv_(0, 2)*Q_xu_(8, 0) - Q_uu_inv_(1, 2)*Q_xu_(8, 1) - Q_uu_inv_(2, 2)*Q_xu_(8, 2);
    k_ux(2, 9) = -Q_uu_inv_(0, 2)*Q_xu_(9, 0) - Q_uu_inv_(1, 2)*Q_xu_(9, 1) - Q_uu_inv_(2, 2)*Q_xu_(9, 2);

    // Evaluation of Vector v_u
    v_u(0) = -Q_u_(0)*Q_uu_inv_(0, 0) - Q_u_(1)*Q_uu_inv_(0, 1) - Q_u_(2)*Q_uu_inv_(0, 2);
    v_u(1) = -Q_u_(0)*Q_uu_inv_(0, 1) - Q_u_(1)*Q_uu_inv_(1, 1) - Q_u_(2)*Q_uu_inv_(1, 2);
    v_u(2) = -Q_u_(0)*Q_uu_inv_(0, 2) - Q_u_(1)*Q_uu_inv_(1, 2) - Q_u_(2)*Q_uu_inv_(2, 2);
    v_u(3) = -Q_u_(3)*Q_uu_inv_(3, 3);
    v_u(4) = -Q_u_(4)*Q_uu_inv_(4, 4);
    v_u(5) = -Q_u_(5)*Q_uu_inv_(5, 5);
    v_u(6) = -Q_u_(6)*Q_uu_inv_(6, 6);
    v_u(7) = -Q_u_(7)*Q_uu_inv_(7, 7);
    v_u(8) = -Q_u_(8)*Q_uu_inv_(8, 8);

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
    V_xx(0, 0) = Q_xu_(0, 0)*k_ux(0, 0) + Q_xu_(0, 1)*k_ux(1, 0) + Q_xu_(0, 2)*k_ux(2, 0) + Q_xx_(0, 0);
    V_xx(0, 1) = Q_xu_(0, 0)*k_ux(0, 1) + Q_xu_(0, 1)*k_ux(1, 1) + Q_xu_(0, 2)*k_ux(2, 1) + Q_xx_(0, 1);
    V_xx(0, 2) = Q_xu_(0, 0)*k_ux(0, 2) + Q_xu_(0, 1)*k_ux(1, 2) + Q_xu_(0, 2)*k_ux(2, 2) + Q_xx_(0, 2);
    V_xx(0, 3) = Q_xu_(0, 0)*k_ux(0, 3) + Q_xu_(0, 1)*k_ux(1, 3) + Q_xu_(0, 2)*k_ux(2, 3) + Q_xx_(0, 3);
    V_xx(0, 4) = Q_xu_(0, 0)*k_ux(0, 4) + Q_xu_(0, 1)*k_ux(1, 4) + Q_xu_(0, 2)*k_ux(2, 4) + Q_xx_(0, 4);
    V_xx(0, 5) = Q_xu_(0, 0)*k_ux(0, 5) + Q_xu_(0, 1)*k_ux(1, 5) + Q_xu_(0, 2)*k_ux(2, 5) + Q_xx_(0, 5);
    V_xx(0, 6) = Q_xu_(0, 0)*k_ux(0, 6) + Q_xu_(0, 1)*k_ux(1, 6) + Q_xu_(0, 2)*k_ux(2, 6) + Q_xx_(0, 6);
    V_xx(0, 7) = Q_xu_(0, 0)*k_ux(0, 7) + Q_xu_(0, 1)*k_ux(1, 7) + Q_xu_(0, 2)*k_ux(2, 7) + Q_xx_(0, 7);
    V_xx(0, 8) = Q_xu_(0, 0)*k_ux(0, 8) + Q_xu_(0, 1)*k_ux(1, 8) + Q_xu_(0, 2)*k_ux(2, 8) + Q_xx_(0, 8);
    V_xx(0, 9) = Q_xu_(0, 0)*k_ux(0, 9) + Q_xu_(0, 1)*k_ux(1, 9) + Q_xu_(0, 2)*k_ux(2, 9) + Q_xx_(0, 9);
    V_xx(1, 0) = Q_xu_(1, 0)*k_ux(0, 0) + Q_xu_(1, 1)*k_ux(1, 0) + Q_xu_(1, 2)*k_ux(2, 0) + Q_xx_(0, 1);
    V_xx(1, 1) = Q_xu_(1, 0)*k_ux(0, 1) + Q_xu_(1, 1)*k_ux(1, 1) + Q_xu_(1, 2)*k_ux(2, 1) + Q_xx_(1, 1);
    V_xx(1, 2) = Q_xu_(1, 0)*k_ux(0, 2) + Q_xu_(1, 1)*k_ux(1, 2) + Q_xu_(1, 2)*k_ux(2, 2) + Q_xx_(1, 2);
    V_xx(1, 3) = Q_xu_(1, 0)*k_ux(0, 3) + Q_xu_(1, 1)*k_ux(1, 3) + Q_xu_(1, 2)*k_ux(2, 3) + Q_xx_(1, 3);
    V_xx(1, 4) = Q_xu_(1, 0)*k_ux(0, 4) + Q_xu_(1, 1)*k_ux(1, 4) + Q_xu_(1, 2)*k_ux(2, 4) + Q_xx_(1, 4);
    V_xx(1, 5) = Q_xu_(1, 0)*k_ux(0, 5) + Q_xu_(1, 1)*k_ux(1, 5) + Q_xu_(1, 2)*k_ux(2, 5) + Q_xx_(1, 5);
    V_xx(1, 6) = Q_xu_(1, 0)*k_ux(0, 6) + Q_xu_(1, 1)*k_ux(1, 6) + Q_xu_(1, 2)*k_ux(2, 6) + Q_xx_(1, 6);
    V_xx(1, 7) = Q_xu_(1, 0)*k_ux(0, 7) + Q_xu_(1, 1)*k_ux(1, 7) + Q_xu_(1, 2)*k_ux(2, 7) + Q_xx_(1, 7);
    V_xx(1, 8) = Q_xu_(1, 0)*k_ux(0, 8) + Q_xu_(1, 1)*k_ux(1, 8) + Q_xu_(1, 2)*k_ux(2, 8) + Q_xx_(1, 8);
    V_xx(1, 9) = Q_xu_(1, 0)*k_ux(0, 9) + Q_xu_(1, 1)*k_ux(1, 9) + Q_xu_(1, 2)*k_ux(2, 9) + Q_xx_(1, 9);
    V_xx(2, 0) = Q_xu_(2, 0)*k_ux(0, 0) + Q_xu_(2, 1)*k_ux(1, 0) + Q_xu_(2, 2)*k_ux(2, 0) + Q_xx_(0, 2);
    V_xx(2, 1) = Q_xu_(2, 0)*k_ux(0, 1) + Q_xu_(2, 1)*k_ux(1, 1) + Q_xu_(2, 2)*k_ux(2, 1) + Q_xx_(1, 2);
    V_xx(2, 2) = Q_xu_(2, 0)*k_ux(0, 2) + Q_xu_(2, 1)*k_ux(1, 2) + Q_xu_(2, 2)*k_ux(2, 2) + Q_xx_(2, 2);
    V_xx(2, 3) = Q_xu_(2, 0)*k_ux(0, 3) + Q_xu_(2, 1)*k_ux(1, 3) + Q_xu_(2, 2)*k_ux(2, 3) + Q_xx_(2, 3);
    V_xx(2, 4) = Q_xu_(2, 0)*k_ux(0, 4) + Q_xu_(2, 1)*k_ux(1, 4) + Q_xu_(2, 2)*k_ux(2, 4) + Q_xx_(2, 4);
    V_xx(2, 5) = Q_xu_(2, 0)*k_ux(0, 5) + Q_xu_(2, 1)*k_ux(1, 5) + Q_xu_(2, 2)*k_ux(2, 5) + Q_xx_(2, 5);
    V_xx(2, 6) = Q_xu_(2, 0)*k_ux(0, 6) + Q_xu_(2, 1)*k_ux(1, 6) + Q_xu_(2, 2)*k_ux(2, 6) + Q_xx_(2, 6);
    V_xx(2, 7) = Q_xu_(2, 0)*k_ux(0, 7) + Q_xu_(2, 1)*k_ux(1, 7) + Q_xu_(2, 2)*k_ux(2, 7) + Q_xx_(2, 7);
    V_xx(2, 8) = Q_xu_(2, 0)*k_ux(0, 8) + Q_xu_(2, 1)*k_ux(1, 8) + Q_xu_(2, 2)*k_ux(2, 8) + Q_xx_(2, 8);
    V_xx(2, 9) = Q_xu_(2, 0)*k_ux(0, 9) + Q_xu_(2, 1)*k_ux(1, 9) + Q_xu_(2, 2)*k_ux(2, 9) + Q_xx_(2, 9);
    V_xx(3, 0) = Q_xu_(3, 0)*k_ux(0, 0) + Q_xu_(3, 1)*k_ux(1, 0) + Q_xu_(3, 2)*k_ux(2, 0) + Q_xx_(0, 3);
    V_xx(3, 1) = Q_xu_(3, 0)*k_ux(0, 1) + Q_xu_(3, 1)*k_ux(1, 1) + Q_xu_(3, 2)*k_ux(2, 1) + Q_xx_(1, 3);
    V_xx(3, 2) = Q_xu_(3, 0)*k_ux(0, 2) + Q_xu_(3, 1)*k_ux(1, 2) + Q_xu_(3, 2)*k_ux(2, 2) + Q_xx_(2, 3);
    V_xx(3, 3) = Q_xu_(3, 0)*k_ux(0, 3) + Q_xu_(3, 1)*k_ux(1, 3) + Q_xu_(3, 2)*k_ux(2, 3) + Q_xx_(3, 3);
    V_xx(3, 4) = Q_xu_(3, 0)*k_ux(0, 4) + Q_xu_(3, 1)*k_ux(1, 4) + Q_xu_(3, 2)*k_ux(2, 4) + Q_xx_(3, 4);
    V_xx(3, 5) = Q_xu_(3, 0)*k_ux(0, 5) + Q_xu_(3, 1)*k_ux(1, 5) + Q_xu_(3, 2)*k_ux(2, 5) + Q_xx_(3, 5);
    V_xx(3, 6) = Q_xu_(3, 0)*k_ux(0, 6) + Q_xu_(3, 1)*k_ux(1, 6) + Q_xu_(3, 2)*k_ux(2, 6) + Q_xx_(3, 6);
    V_xx(3, 7) = Q_xu_(3, 0)*k_ux(0, 7) + Q_xu_(3, 1)*k_ux(1, 7) + Q_xu_(3, 2)*k_ux(2, 7) + Q_xx_(3, 7);
    V_xx(3, 8) = Q_xu_(3, 0)*k_ux(0, 8) + Q_xu_(3, 1)*k_ux(1, 8) + Q_xu_(3, 2)*k_ux(2, 8) + Q_xx_(3, 8);
    V_xx(3, 9) = Q_xu_(3, 0)*k_ux(0, 9) + Q_xu_(3, 1)*k_ux(1, 9) + Q_xu_(3, 2)*k_ux(2, 9) + Q_xx_(3, 9);
    V_xx(4, 0) = Q_xu_(4, 0)*k_ux(0, 0) + Q_xu_(4, 1)*k_ux(1, 0) + Q_xu_(4, 2)*k_ux(2, 0) + Q_xx_(0, 4);
    V_xx(4, 1) = Q_xu_(4, 0)*k_ux(0, 1) + Q_xu_(4, 1)*k_ux(1, 1) + Q_xu_(4, 2)*k_ux(2, 1) + Q_xx_(1, 4);
    V_xx(4, 2) = Q_xu_(4, 0)*k_ux(0, 2) + Q_xu_(4, 1)*k_ux(1, 2) + Q_xu_(4, 2)*k_ux(2, 2) + Q_xx_(2, 4);
    V_xx(4, 3) = Q_xu_(4, 0)*k_ux(0, 3) + Q_xu_(4, 1)*k_ux(1, 3) + Q_xu_(4, 2)*k_ux(2, 3) + Q_xx_(3, 4);
    V_xx(4, 4) = Q_xu_(4, 0)*k_ux(0, 4) + Q_xu_(4, 1)*k_ux(1, 4) + Q_xu_(4, 2)*k_ux(2, 4) + Q_xx_(4, 4);
    V_xx(4, 5) = Q_xu_(4, 0)*k_ux(0, 5) + Q_xu_(4, 1)*k_ux(1, 5) + Q_xu_(4, 2)*k_ux(2, 5) + Q_xx_(4, 5);
    V_xx(4, 6) = Q_xu_(4, 0)*k_ux(0, 6) + Q_xu_(4, 1)*k_ux(1, 6) + Q_xu_(4, 2)*k_ux(2, 6) + Q_xx_(4, 6);
    V_xx(4, 7) = Q_xu_(4, 0)*k_ux(0, 7) + Q_xu_(4, 1)*k_ux(1, 7) + Q_xu_(4, 2)*k_ux(2, 7) + Q_xx_(4, 7);
    V_xx(4, 8) = Q_xu_(4, 0)*k_ux(0, 8) + Q_xu_(4, 1)*k_ux(1, 8) + Q_xu_(4, 2)*k_ux(2, 8) + Q_xx_(4, 8);
    V_xx(4, 9) = Q_xu_(4, 0)*k_ux(0, 9) + Q_xu_(4, 1)*k_ux(1, 9) + Q_xu_(4, 2)*k_ux(2, 9) + Q_xx_(4, 9);
    V_xx(5, 0) = Q_xu_(5, 0)*k_ux(0, 0) + Q_xu_(5, 1)*k_ux(1, 0) + Q_xu_(5, 2)*k_ux(2, 0) + Q_xx_(0, 5);
    V_xx(5, 1) = Q_xu_(5, 0)*k_ux(0, 1) + Q_xu_(5, 1)*k_ux(1, 1) + Q_xu_(5, 2)*k_ux(2, 1) + Q_xx_(1, 5);
    V_xx(5, 2) = Q_xu_(5, 0)*k_ux(0, 2) + Q_xu_(5, 1)*k_ux(1, 2) + Q_xu_(5, 2)*k_ux(2, 2) + Q_xx_(2, 5);
    V_xx(5, 3) = Q_xu_(5, 0)*k_ux(0, 3) + Q_xu_(5, 1)*k_ux(1, 3) + Q_xu_(5, 2)*k_ux(2, 3) + Q_xx_(3, 5);
    V_xx(5, 4) = Q_xu_(5, 0)*k_ux(0, 4) + Q_xu_(5, 1)*k_ux(1, 4) + Q_xu_(5, 2)*k_ux(2, 4) + Q_xx_(4, 5);
    V_xx(5, 5) = Q_xu_(5, 0)*k_ux(0, 5) + Q_xu_(5, 1)*k_ux(1, 5) + Q_xu_(5, 2)*k_ux(2, 5) + Q_xx_(5, 5);
    V_xx(5, 6) = Q_xu_(5, 0)*k_ux(0, 6) + Q_xu_(5, 1)*k_ux(1, 6) + Q_xu_(5, 2)*k_ux(2, 6) + Q_xx_(5, 6);
    V_xx(5, 7) = Q_xu_(5, 0)*k_ux(0, 7) + Q_xu_(5, 1)*k_ux(1, 7) + Q_xu_(5, 2)*k_ux(2, 7) + Q_xx_(5, 7);
    V_xx(5, 8) = Q_xu_(5, 0)*k_ux(0, 8) + Q_xu_(5, 1)*k_ux(1, 8) + Q_xu_(5, 2)*k_ux(2, 8) + Q_xx_(5, 8);
    V_xx(5, 9) = Q_xu_(5, 0)*k_ux(0, 9) + Q_xu_(5, 1)*k_ux(1, 9) + Q_xu_(5, 2)*k_ux(2, 9) + Q_xx_(5, 9);
    V_xx(6, 0) = Q_xu_(6, 0)*k_ux(0, 0) + Q_xu_(6, 1)*k_ux(1, 0) + Q_xu_(6, 2)*k_ux(2, 0) + Q_xx_(0, 6);
    V_xx(6, 1) = Q_xu_(6, 0)*k_ux(0, 1) + Q_xu_(6, 1)*k_ux(1, 1) + Q_xu_(6, 2)*k_ux(2, 1) + Q_xx_(1, 6);
    V_xx(6, 2) = Q_xu_(6, 0)*k_ux(0, 2) + Q_xu_(6, 1)*k_ux(1, 2) + Q_xu_(6, 2)*k_ux(2, 2) + Q_xx_(2, 6);
    V_xx(6, 3) = Q_xu_(6, 0)*k_ux(0, 3) + Q_xu_(6, 1)*k_ux(1, 3) + Q_xu_(6, 2)*k_ux(2, 3) + Q_xx_(3, 6);
    V_xx(6, 4) = Q_xu_(6, 0)*k_ux(0, 4) + Q_xu_(6, 1)*k_ux(1, 4) + Q_xu_(6, 2)*k_ux(2, 4) + Q_xx_(4, 6);
    V_xx(6, 5) = Q_xu_(6, 0)*k_ux(0, 5) + Q_xu_(6, 1)*k_ux(1, 5) + Q_xu_(6, 2)*k_ux(2, 5) + Q_xx_(5, 6);
    V_xx(6, 6) = Q_xu_(6, 0)*k_ux(0, 6) + Q_xu_(6, 1)*k_ux(1, 6) + Q_xu_(6, 2)*k_ux(2, 6) + Q_xx_(6, 6);
    V_xx(6, 7) = Q_xu_(6, 0)*k_ux(0, 7) + Q_xu_(6, 1)*k_ux(1, 7) + Q_xu_(6, 2)*k_ux(2, 7) + Q_xx_(6, 7);
    V_xx(6, 8) = Q_xu_(6, 0)*k_ux(0, 8) + Q_xu_(6, 1)*k_ux(1, 8) + Q_xu_(6, 2)*k_ux(2, 8) + Q_xx_(6, 8);
    V_xx(6, 9) = Q_xu_(6, 0)*k_ux(0, 9) + Q_xu_(6, 1)*k_ux(1, 9) + Q_xu_(6, 2)*k_ux(2, 9) + Q_xx_(6, 9);
    V_xx(7, 0) = Q_xu_(7, 0)*k_ux(0, 0) + Q_xu_(7, 1)*k_ux(1, 0) + Q_xu_(7, 2)*k_ux(2, 0) + Q_xx_(0, 7);
    V_xx(7, 1) = Q_xu_(7, 0)*k_ux(0, 1) + Q_xu_(7, 1)*k_ux(1, 1) + Q_xu_(7, 2)*k_ux(2, 1) + Q_xx_(1, 7);
    V_xx(7, 2) = Q_xu_(7, 0)*k_ux(0, 2) + Q_xu_(7, 1)*k_ux(1, 2) + Q_xu_(7, 2)*k_ux(2, 2) + Q_xx_(2, 7);
    V_xx(7, 3) = Q_xu_(7, 0)*k_ux(0, 3) + Q_xu_(7, 1)*k_ux(1, 3) + Q_xu_(7, 2)*k_ux(2, 3) + Q_xx_(3, 7);
    V_xx(7, 4) = Q_xu_(7, 0)*k_ux(0, 4) + Q_xu_(7, 1)*k_ux(1, 4) + Q_xu_(7, 2)*k_ux(2, 4) + Q_xx_(4, 7);
    V_xx(7, 5) = Q_xu_(7, 0)*k_ux(0, 5) + Q_xu_(7, 1)*k_ux(1, 5) + Q_xu_(7, 2)*k_ux(2, 5) + Q_xx_(5, 7);
    V_xx(7, 6) = Q_xu_(7, 0)*k_ux(0, 6) + Q_xu_(7, 1)*k_ux(1, 6) + Q_xu_(7, 2)*k_ux(2, 6) + Q_xx_(6, 7);
    V_xx(7, 7) = Q_xu_(7, 0)*k_ux(0, 7) + Q_xu_(7, 1)*k_ux(1, 7) + Q_xu_(7, 2)*k_ux(2, 7) + Q_xx_(7, 7);
    V_xx(7, 8) = Q_xu_(7, 0)*k_ux(0, 8) + Q_xu_(7, 1)*k_ux(1, 8) + Q_xu_(7, 2)*k_ux(2, 8) + Q_xx_(7, 8);
    V_xx(7, 9) = Q_xu_(7, 0)*k_ux(0, 9) + Q_xu_(7, 1)*k_ux(1, 9) + Q_xu_(7, 2)*k_ux(2, 9) + Q_xx_(7, 9);
    V_xx(8, 0) = Q_xu_(8, 0)*k_ux(0, 0) + Q_xu_(8, 1)*k_ux(1, 0) + Q_xu_(8, 2)*k_ux(2, 0) + Q_xx_(0, 8);
    V_xx(8, 1) = Q_xu_(8, 0)*k_ux(0, 1) + Q_xu_(8, 1)*k_ux(1, 1) + Q_xu_(8, 2)*k_ux(2, 1) + Q_xx_(1, 8);
    V_xx(8, 2) = Q_xu_(8, 0)*k_ux(0, 2) + Q_xu_(8, 1)*k_ux(1, 2) + Q_xu_(8, 2)*k_ux(2, 2) + Q_xx_(2, 8);
    V_xx(8, 3) = Q_xu_(8, 0)*k_ux(0, 3) + Q_xu_(8, 1)*k_ux(1, 3) + Q_xu_(8, 2)*k_ux(2, 3) + Q_xx_(3, 8);
    V_xx(8, 4) = Q_xu_(8, 0)*k_ux(0, 4) + Q_xu_(8, 1)*k_ux(1, 4) + Q_xu_(8, 2)*k_ux(2, 4) + Q_xx_(4, 8);
    V_xx(8, 5) = Q_xu_(8, 0)*k_ux(0, 5) + Q_xu_(8, 1)*k_ux(1, 5) + Q_xu_(8, 2)*k_ux(2, 5) + Q_xx_(5, 8);
    V_xx(8, 6) = Q_xu_(8, 0)*k_ux(0, 6) + Q_xu_(8, 1)*k_ux(1, 6) + Q_xu_(8, 2)*k_ux(2, 6) + Q_xx_(6, 8);
    V_xx(8, 7) = Q_xu_(8, 0)*k_ux(0, 7) + Q_xu_(8, 1)*k_ux(1, 7) + Q_xu_(8, 2)*k_ux(2, 7) + Q_xx_(7, 8);
    V_xx(8, 8) = Q_xu_(8, 0)*k_ux(0, 8) + Q_xu_(8, 1)*k_ux(1, 8) + Q_xu_(8, 2)*k_ux(2, 8) + Q_xx_(8, 8);
    V_xx(8, 9) = Q_xu_(8, 0)*k_ux(0, 9) + Q_xu_(8, 1)*k_ux(1, 9) + Q_xu_(8, 2)*k_ux(2, 9) + Q_xx_(8, 9);
    V_xx(9, 0) = Q_xu_(9, 0)*k_ux(0, 0) + Q_xu_(9, 1)*k_ux(1, 0) + Q_xu_(9, 2)*k_ux(2, 0) + Q_xx_(0, 9);
    V_xx(9, 1) = Q_xu_(9, 0)*k_ux(0, 1) + Q_xu_(9, 1)*k_ux(1, 1) + Q_xu_(9, 2)*k_ux(2, 1) + Q_xx_(1, 9);
    V_xx(9, 2) = Q_xu_(9, 0)*k_ux(0, 2) + Q_xu_(9, 1)*k_ux(1, 2) + Q_xu_(9, 2)*k_ux(2, 2) + Q_xx_(2, 9);
    V_xx(9, 3) = Q_xu_(9, 0)*k_ux(0, 3) + Q_xu_(9, 1)*k_ux(1, 3) + Q_xu_(9, 2)*k_ux(2, 3) + Q_xx_(3, 9);
    V_xx(9, 4) = Q_xu_(9, 0)*k_ux(0, 4) + Q_xu_(9, 1)*k_ux(1, 4) + Q_xu_(9, 2)*k_ux(2, 4) + Q_xx_(4, 9);
    V_xx(9, 5) = Q_xu_(9, 0)*k_ux(0, 5) + Q_xu_(9, 1)*k_ux(1, 5) + Q_xu_(9, 2)*k_ux(2, 5) + Q_xx_(5, 9);
    V_xx(9, 6) = Q_xu_(9, 0)*k_ux(0, 6) + Q_xu_(9, 1)*k_ux(1, 6) + Q_xu_(9, 2)*k_ux(2, 6) + Q_xx_(6, 9);
    V_xx(9, 7) = Q_xu_(9, 0)*k_ux(0, 7) + Q_xu_(9, 1)*k_ux(1, 7) + Q_xu_(9, 2)*k_ux(2, 7) + Q_xx_(7, 9);
    V_xx(9, 8) = Q_xu_(9, 0)*k_ux(0, 8) + Q_xu_(9, 1)*k_ux(1, 8) + Q_xu_(9, 2)*k_ux(2, 8) + Q_xx_(8, 9);
    V_xx(9, 9) = Q_xu_(9, 0)*k_ux(0, 9) + Q_xu_(9, 1)*k_ux(1, 9) + Q_xu_(9, 2)*k_ux(2, 9) + Q_xx_(9, 9);

    // Evaluation of Vector V_x
    V_x(0) = Q_x_(0) + Q_xu_(0, 0)*v_u(0) + Q_xu_(0, 1)*v_u(1) + Q_xu_(0, 2)*v_u(2);
    V_x(1) = Q_x_(1) + Q_xu_(1, 0)*v_u(0) + Q_xu_(1, 1)*v_u(1) + Q_xu_(1, 2)*v_u(2);
    V_x(2) = Q_x_(2) + Q_xu_(2, 0)*v_u(0) + Q_xu_(2, 1)*v_u(1) + Q_xu_(2, 2)*v_u(2);
    V_x(3) = Q_x_(3) + Q_xu_(3, 0)*v_u(0) + Q_xu_(3, 1)*v_u(1) + Q_xu_(3, 2)*v_u(2);
    V_x(4) = Q_x_(4) + Q_xu_(4, 0)*v_u(0) + Q_xu_(4, 1)*v_u(1) + Q_xu_(4, 2)*v_u(2);
    V_x(5) = Q_x_(5) + Q_xu_(5, 0)*v_u(0) + Q_xu_(5, 1)*v_u(1) + Q_xu_(5, 2)*v_u(2);
    V_x(6) = Q_x_(6) + Q_xu_(6, 0)*v_u(0) + Q_xu_(6, 1)*v_u(1) + Q_xu_(6, 2)*v_u(2);
    V_x(7) = Q_x_(7) + Q_xu_(7, 0)*v_u(0) + Q_xu_(7, 1)*v_u(1) + Q_xu_(7, 2)*v_u(2);
    V_x(8) = Q_x_(8) + Q_xu_(8, 0)*v_u(0) + Q_xu_(8, 1)*v_u(1) + Q_xu_(8, 2)*v_u(2);
    V_x(9) = Q_x_(9) + Q_xu_(9, 0)*v_u(0) + Q_xu_(9, 1)*v_u(1) + Q_xu_(9, 2)*v_u(2);

  }

  virtual void updateQnx(const Eigen::VectorXd& r_f, const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& Fu, const Eigen::MatrixXd& h_u, const Eigen::MatrixXd& k_ux, const Eigen::MatrixXd& k_ex, const Eigen::MatrixXd& k_ue, const Eigen::VectorXd& v_u, const Eigen::VectorXd& v_e) override {
    // Determine internal variables
    const double internal_0 = -Q_uu_(0, 1);
    const double internal_1 = -Q_uu_(0, 2);
    const double internal_2 = -Q_uu_(1, 2);

    // Evaluation of Matrix Q_un_
    Q_un_(0, 0) = -Fu(0, 0)*Q_uu_inv_(0, 0) - Fu(0, 1)*Q_uu_inv_(0, 1);
    Q_un_(0, 1) = -Fu(1, 0)*Q_uu_inv_(0, 0) - Fu(1, 1)*Q_uu_inv_(0, 1);
    Q_un_(0, 2) = -Fu(2, 0)*Q_uu_inv_(0, 0) - Fu(2, 1)*Q_uu_inv_(0, 1);
    Q_un_(0, 3) = -Fu(3, 0)*Q_uu_inv_(0, 0) - Fu(3, 1)*Q_uu_inv_(0, 1);
    Q_un_(0, 4) = -Fu(4, 0)*Q_uu_inv_(0, 0);
    Q_un_(0, 5) = -Fu(5, 2)*Q_uu_inv_(0, 2);
    Q_un_(0, 6) = -Fu(6, 2)*Q_uu_inv_(0, 2);
    Q_un_(0, 7) = -Fu(7, 2)*Q_uu_inv_(0, 2);
    Q_un_(0, 9) = -Fu(9, 2)*Q_uu_inv_(0, 2);
    Q_un_(1, 0) = -Fu(0, 0)*Q_uu_inv_(0, 1) - Fu(0, 1)*Q_uu_inv_(1, 1);
    Q_un_(1, 1) = -Fu(1, 0)*Q_uu_inv_(0, 1) - Fu(1, 1)*Q_uu_inv_(1, 1);
    Q_un_(1, 2) = -Fu(2, 0)*Q_uu_inv_(0, 1) - Fu(2, 1)*Q_uu_inv_(1, 1);
    Q_un_(1, 3) = -Fu(3, 0)*Q_uu_inv_(0, 1) - Fu(3, 1)*Q_uu_inv_(1, 1);
    Q_un_(1, 4) = -Fu(4, 0)*Q_uu_inv_(0, 1);
    Q_un_(1, 5) = -Fu(5, 2)*Q_uu_inv_(1, 2);
    Q_un_(1, 6) = -Fu(6, 2)*Q_uu_inv_(1, 2);
    Q_un_(1, 7) = -Fu(7, 2)*Q_uu_inv_(1, 2);
    Q_un_(1, 9) = -Fu(9, 2)*Q_uu_inv_(1, 2);
    Q_un_(2, 0) = -Fu(0, 0)*Q_uu_inv_(0, 2) - Fu(0, 1)*Q_uu_inv_(1, 2);
    Q_un_(2, 1) = -Fu(1, 0)*Q_uu_inv_(0, 2) - Fu(1, 1)*Q_uu_inv_(1, 2);
    Q_un_(2, 2) = -Fu(2, 0)*Q_uu_inv_(0, 2) - Fu(2, 1)*Q_uu_inv_(1, 2);
    Q_un_(2, 3) = -Fu(3, 0)*Q_uu_inv_(0, 2) - Fu(3, 1)*Q_uu_inv_(1, 2);
    Q_un_(2, 4) = -Fu(4, 0)*Q_uu_inv_(0, 2);
    Q_un_(2, 5) = -Fu(5, 2)*Q_uu_inv_(2, 2);
    Q_un_(2, 6) = -Fu(6, 2)*Q_uu_inv_(2, 2);
    Q_un_(2, 7) = -Fu(7, 2)*Q_uu_inv_(2, 2);
    Q_un_(2, 9) = -Fu(9, 2)*Q_uu_inv_(2, 2);

    // Evaluation of Matrix Q_nuu_
    Q_nuu_(0, 0) = -Q_uu_(0, 0);
    Q_nuu_(0, 1) = internal_0;
    Q_nuu_(0, 2) = internal_1;
    Q_nuu_(1, 0) = internal_0;
    Q_nuu_(1, 1) = -Q_uu_(1, 1);
    Q_nuu_(1, 2) = internal_2;
    Q_nuu_(2, 0) = internal_1;
    Q_nuu_(2, 1) = internal_2;
    Q_nuu_(2, 2) = -Q_uu_(2, 2);
    Q_nuu_(3, 3) = -Q_uu_(3, 3);
    Q_nuu_(4, 4) = -Q_uu_(4, 4);
    Q_nuu_(5, 5) = -Q_uu_(5, 5);
    Q_nuu_(6, 6) = -Q_uu_(6, 6);
    Q_nuu_(7, 7) = -Q_uu_(7, 7);
    Q_nuu_(8, 8) = -Q_uu_(8, 8);

    // Evaluation of Matrix Q_vnx_
    Q_vnx_(0, 0) = Fu(0, 0)*k_ux(0, 0) + Fu(0, 1)*k_ux(1, 0) + Fx(0, 0);
    Q_vnx_(0, 1) = Fu(0, 0)*k_ux(0, 1) + Fu(0, 1)*k_ux(1, 1) + Fx(0, 1);
    Q_vnx_(0, 2) = Fu(0, 0)*k_ux(0, 2) + Fu(0, 1)*k_ux(1, 2) + Fx(0, 2);
    Q_vnx_(0, 3) = Fu(0, 0)*k_ux(0, 3) + Fu(0, 1)*k_ux(1, 3) + Fx(0, 3);
    Q_vnx_(0, 4) = Fu(0, 0)*k_ux(0, 4) + Fu(0, 1)*k_ux(1, 4) + Fx(0, 4);
    Q_vnx_(0, 5) = Fu(0, 0)*k_ux(0, 5) + Fu(0, 1)*k_ux(1, 5);
    Q_vnx_(0, 6) = Fu(0, 0)*k_ux(0, 6) + Fu(0, 1)*k_ux(1, 6);
    Q_vnx_(0, 7) = Fu(0, 0)*k_ux(0, 7) + Fu(0, 1)*k_ux(1, 7);
    Q_vnx_(0, 8) = Fu(0, 0)*k_ux(0, 8) + Fu(0, 1)*k_ux(1, 8);
    Q_vnx_(0, 9) = Fu(0, 0)*k_ux(0, 9) + Fu(0, 1)*k_ux(1, 9);
    Q_vnx_(1, 0) = Fu(1, 0)*k_ux(0, 0) + Fu(1, 1)*k_ux(1, 0);
    Q_vnx_(1, 1) = Fu(1, 0)*k_ux(0, 1) + Fu(1, 1)*k_ux(1, 1) + Fx(1, 1);
    Q_vnx_(1, 2) = Fu(1, 0)*k_ux(0, 2) + Fu(1, 1)*k_ux(1, 2);
    Q_vnx_(1, 3) = Fu(1, 0)*k_ux(0, 3) + Fu(1, 1)*k_ux(1, 3) + Fx(1, 3);
    Q_vnx_(1, 4) = Fu(1, 0)*k_ux(0, 4) + Fu(1, 1)*k_ux(1, 4) + Fx(1, 4);
    Q_vnx_(1, 5) = Fu(1, 0)*k_ux(0, 5) + Fu(1, 1)*k_ux(1, 5);
    Q_vnx_(1, 6) = Fu(1, 0)*k_ux(0, 6) + Fu(1, 1)*k_ux(1, 6);
    Q_vnx_(1, 7) = Fu(1, 0)*k_ux(0, 7) + Fu(1, 1)*k_ux(1, 7);
    Q_vnx_(1, 8) = Fu(1, 0)*k_ux(0, 8) + Fu(1, 1)*k_ux(1, 8);
    Q_vnx_(1, 9) = Fu(1, 0)*k_ux(0, 9) + Fu(1, 1)*k_ux(1, 9);
    Q_vnx_(2, 0) = Fu(2, 0)*k_ux(0, 0) + Fu(2, 1)*k_ux(1, 0);
    Q_vnx_(2, 1) = Fu(2, 0)*k_ux(0, 1) + Fu(2, 1)*k_ux(1, 1);
    Q_vnx_(2, 2) = Fu(2, 0)*k_ux(0, 2) + Fu(2, 1)*k_ux(1, 2) + Fx(2, 2);
    Q_vnx_(2, 3) = Fu(2, 0)*k_ux(0, 3) + Fu(2, 1)*k_ux(1, 3) + Fx(2, 3);
    Q_vnx_(2, 4) = Fu(2, 0)*k_ux(0, 4) + Fu(2, 1)*k_ux(1, 4) + Fx(2, 4);
    Q_vnx_(2, 5) = Fu(2, 0)*k_ux(0, 5) + Fu(2, 1)*k_ux(1, 5);
    Q_vnx_(2, 6) = Fu(2, 0)*k_ux(0, 6) + Fu(2, 1)*k_ux(1, 6);
    Q_vnx_(2, 7) = Fu(2, 0)*k_ux(0, 7) + Fu(2, 1)*k_ux(1, 7);
    Q_vnx_(2, 8) = Fu(2, 0)*k_ux(0, 8) + Fu(2, 1)*k_ux(1, 8);
    Q_vnx_(2, 9) = Fu(2, 0)*k_ux(0, 9) + Fu(2, 1)*k_ux(1, 9);
    Q_vnx_(3, 0) = Fu(3, 0)*k_ux(0, 0) + Fu(3, 1)*k_ux(1, 0);
    Q_vnx_(3, 1) = Fu(3, 0)*k_ux(0, 1) + Fu(3, 1)*k_ux(1, 1);
    Q_vnx_(3, 2) = Fu(3, 0)*k_ux(0, 2) + Fu(3, 1)*k_ux(1, 2);
    Q_vnx_(3, 3) = Fu(3, 0)*k_ux(0, 3) + Fu(3, 1)*k_ux(1, 3) + Fx(3, 3);
    Q_vnx_(3, 4) = Fu(3, 0)*k_ux(0, 4) + Fu(3, 1)*k_ux(1, 4) + Fx(3, 4);
    Q_vnx_(3, 5) = Fu(3, 0)*k_ux(0, 5) + Fu(3, 1)*k_ux(1, 5);
    Q_vnx_(3, 6) = Fu(3, 0)*k_ux(0, 6) + Fu(3, 1)*k_ux(1, 6);
    Q_vnx_(3, 7) = Fu(3, 0)*k_ux(0, 7) + Fu(3, 1)*k_ux(1, 7);
    Q_vnx_(3, 8) = Fu(3, 0)*k_ux(0, 8) + Fu(3, 1)*k_ux(1, 8);
    Q_vnx_(3, 9) = Fu(3, 0)*k_ux(0, 9) + Fu(3, 1)*k_ux(1, 9);
    Q_vnx_(4, 0) = Fu(4, 0)*k_ux(0, 0);
    Q_vnx_(4, 1) = Fu(4, 0)*k_ux(0, 1);
    Q_vnx_(4, 2) = Fu(4, 0)*k_ux(0, 2);
    Q_vnx_(4, 3) = Fu(4, 0)*k_ux(0, 3);
    Q_vnx_(4, 4) = Fu(4, 0)*k_ux(0, 4) + Fx(4, 4);
    Q_vnx_(4, 5) = Fu(4, 0)*k_ux(0, 5);
    Q_vnx_(4, 6) = Fu(4, 0)*k_ux(0, 6);
    Q_vnx_(4, 7) = Fu(4, 0)*k_ux(0, 7);
    Q_vnx_(4, 8) = Fu(4, 0)*k_ux(0, 8);
    Q_vnx_(4, 9) = Fu(4, 0)*k_ux(0, 9);
    Q_vnx_(5, 0) = Fu(5, 2)*k_ux(2, 0);
    Q_vnx_(5, 1) = Fu(5, 2)*k_ux(2, 1);
    Q_vnx_(5, 2) = Fu(5, 2)*k_ux(2, 2);
    Q_vnx_(5, 3) = Fu(5, 2)*k_ux(2, 3);
    Q_vnx_(5, 4) = Fu(5, 2)*k_ux(2, 4);
    Q_vnx_(5, 5) = Fu(5, 2)*k_ux(2, 5) + Fx(5, 5);
    Q_vnx_(5, 6) = Fu(5, 2)*k_ux(2, 6);
    Q_vnx_(5, 7) = Fu(5, 2)*k_ux(2, 7);
    Q_vnx_(5, 8) = Fu(5, 2)*k_ux(2, 8);
    Q_vnx_(5, 9) = Fu(5, 2)*k_ux(2, 9) + Fx(5, 9);
    Q_vnx_(6, 0) = Fu(6, 2)*k_ux(2, 0);
    Q_vnx_(6, 1) = Fu(6, 2)*k_ux(2, 1);
    Q_vnx_(6, 2) = Fu(6, 2)*k_ux(2, 2);
    Q_vnx_(6, 3) = Fu(6, 2)*k_ux(2, 3);
    Q_vnx_(6, 4) = Fu(6, 2)*k_ux(2, 4);
    Q_vnx_(6, 5) = Fu(6, 2)*k_ux(2, 5);
    Q_vnx_(6, 6) = Fu(6, 2)*k_ux(2, 6) + Fx(6, 6);
    Q_vnx_(6, 7) = Fu(6, 2)*k_ux(2, 7);
    Q_vnx_(6, 8) = Fu(6, 2)*k_ux(2, 8);
    Q_vnx_(6, 9) = Fu(6, 2)*k_ux(2, 9) + Fx(6, 9);
    Q_vnx_(7, 0) = Fu(7, 2)*k_ux(2, 0);
    Q_vnx_(7, 1) = Fu(7, 2)*k_ux(2, 1);
    Q_vnx_(7, 2) = Fu(7, 2)*k_ux(2, 2);
    Q_vnx_(7, 3) = Fu(7, 2)*k_ux(2, 3);
    Q_vnx_(7, 4) = Fu(7, 2)*k_ux(2, 4);
    Q_vnx_(7, 5) = Fu(7, 2)*k_ux(2, 5);
    Q_vnx_(7, 6) = Fu(7, 2)*k_ux(2, 6);
    Q_vnx_(7, 7) = Fu(7, 2)*k_ux(2, 7) + Fx(7, 7);
    Q_vnx_(7, 8) = Fu(7, 2)*k_ux(2, 8);
    Q_vnx_(7, 9) = Fu(7, 2)*k_ux(2, 9) + Fx(7, 9);
    Q_vnx_(8, 8) = Fx(8, 8);
    Q_vnx_(9, 0) = Fu(9, 2)*k_ux(2, 0);
    Q_vnx_(9, 1) = Fu(9, 2)*k_ux(2, 1);
    Q_vnx_(9, 2) = Fu(9, 2)*k_ux(2, 2);
    Q_vnx_(9, 3) = Fu(9, 2)*k_ux(2, 3);
    Q_vnx_(9, 4) = Fu(9, 2)*k_ux(2, 4);
    Q_vnx_(9, 5) = Fu(9, 2)*k_ux(2, 5);
    Q_vnx_(9, 6) = Fu(9, 2)*k_ux(2, 6);
    Q_vnx_(9, 7) = Fu(9, 2)*k_ux(2, 7);
    Q_vnx_(9, 8) = Fu(9, 2)*k_ux(2, 8);
    Q_vnx_(9, 9) = Fu(9, 2)*k_ux(2, 9) + Fx(9, 9);

    // Evaluation of Vector Q_vn_
    Q_vn_(0) = Fu(0, 0)*v_u(0) + Fu(0, 1)*v_u(1) + r_f(0);
    Q_vn_(1) = Fu(1, 0)*v_u(0) + Fu(1, 1)*v_u(1) + r_f(1);
    Q_vn_(2) = Fu(2, 0)*v_u(0) + Fu(2, 1)*v_u(1) + r_f(2);
    Q_vn_(3) = Fu(3, 0)*v_u(0) + Fu(3, 1)*v_u(1) + r_f(3);
    Q_vn_(4) = Fu(4, 0)*v_u(0) + r_f(4);
    Q_vn_(5) = Fu(5, 2)*v_u(2) + r_f(5);
    Q_vn_(6) = Fu(6, 2)*v_u(2) + r_f(6);
    Q_vn_(7) = Fu(7, 2)*v_u(2) + r_f(7);
    Q_vn_(8) = r_f(8);
    Q_vn_(9) = Fu(9, 2)*v_u(2) + r_f(9);

  }

};

class MultiAgentsTrajectoryModelIpmEvaluatorTerminal : public IpmEvaluatorTerminal {
 public:
  MultiAgentsTrajectoryModelIpmEvaluatorTerminal() : IpmEvaluatorTerminal(10, 9, 10, 0) {}
  virtual ~MultiAgentsTrajectoryModelIpmEvaluatorTerminal() = default;

  virtual void updateLX(const Eigen::VectorXd& l_x, const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& m_x, const Eigen::VectorXd& p, const Eigen::VectorXd& lambda, const Eigen::VectorXd& nu, Eigen::VectorXd& sl_x) const override {
    // Evaluation of Vector sl_x
    sl_x(0) = g_x(2, 0)*lambda(2) + g_x(3, 0)*lambda(3) + l_x(0) - p(0);
    sl_x(1) = g_x(0, 1)*lambda(0) + g_x(1, 1)*lambda(1) + g_x(8, 1)*lambda(8) + l_x(1) - p(1);
    sl_x(2) = g_x(0, 2)*lambda(0) + g_x(1, 2)*lambda(1) + g_x(8, 2)*lambda(8) + l_x(2) - p(2);
    sl_x(3) = g_x(9, 3)*lambda(9) + l_x(3) - p(3);
    sl_x(4) = g_x(5, 4)*lambda(5) + g_x(6, 4)*lambda(6) + l_x(4) - p(4);
    sl_x(5) = g_x(4, 5)*lambda(4) + l_x(5) - p(5);
    sl_x(6) = -p(6);
    sl_x(7) = -p(7);
    sl_x(8) = -p(8);
    sl_x(9) = g_x(7, 9)*lambda(7) + l_x(9) - p(9);

  }

  virtual void updateVxx(const Eigen::MatrixXd& l_xx, const Eigen::VectorXd& sl_x, const Eigen::MatrixXd& g_x, Eigen::MatrixXd& V_xx, Eigen::VectorXd& V_x) override {
    // Determine internal variables
    const double internal_0 = Sigma_(0)*g_x(0, 1)*g_x(0, 2) + Sigma_(1)*g_x(1, 1)*g_x(1, 2) + Sigma_(8)*g_x(8, 1)*g_x(8, 2);

    // Evaluation of Matrix V_xx
    V_xx(0, 0) = Sigma_(2)*pow(g_x(2, 0), 2) + Sigma_(3)*pow(g_x(3, 0), 2) + l_xx(0, 0);
    V_xx(1, 1) = Sigma_(0)*pow(g_x(0, 1), 2) + Sigma_(1)*pow(g_x(1, 1), 2) + Sigma_(8)*pow(g_x(8, 1), 2) + l_xx(1, 1);
    V_xx(1, 2) = internal_0 + l_xx(1, 2);
    V_xx(2, 1) = internal_0 + l_xx(2, 1);
    V_xx(2, 2) = Sigma_(0)*pow(g_x(0, 2), 2) + Sigma_(1)*pow(g_x(1, 2), 2) + Sigma_(8)*pow(g_x(8, 2), 2) + l_xx(2, 2);
    V_xx(3, 3) = Sigma_(9)*pow(g_x(9, 3), 2) + l_xx(3, 3);
    V_xx(4, 4) = Sigma_(5)*pow(g_x(5, 4), 2) + Sigma_(6)*pow(g_x(6, 4), 2) + l_xx(4, 4);
    V_xx(5, 5) = Sigma_(4)*pow(g_x(4, 5), 2) + l_xx(5, 5);
    V_xx(9, 9) = Sigma_(7)*pow(g_x(7, 9), 2) + l_xx(9, 9);

    // Evaluation of Vector V_x
    V_x(0) = V_(2)*g_x(2, 0) + V_(3)*g_x(3, 0) + sl_x(0);
    V_x(1) = V_(0)*g_x(0, 1) + V_(1)*g_x(1, 1) + V_(8)*g_x(8, 1) + sl_x(1);
    V_x(2) = V_(0)*g_x(0, 2) + V_(1)*g_x(1, 2) + V_(8)*g_x(8, 2) + sl_x(2);
    V_x(3) = V_(9)*g_x(9, 3) + sl_x(3);
    V_x(4) = V_(5)*g_x(5, 4) + V_(6)*g_x(6, 4) + sl_x(4);
    V_x(5) = V_(4)*g_x(4, 5) + sl_x(5);
    V_x(6) = sl_x(6);
    V_x(7) = sl_x(7);
    V_x(8) = sl_x(8);
    V_x(9) = V_(7)*g_x(7, 9) + sl_x(9);

  }

};

}  // namespace gpal::pnc::planning