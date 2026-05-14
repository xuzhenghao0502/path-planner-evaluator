#pragma once

#include "ocp/dynamics.h"

namespace gpal::pnc::planning {

class SpatiotemporalPlannerModelDynamics : public Dynamics {
 public:
  explicit SpatiotemporalPlannerModelDynamics(OcpConfig::IntegratorType type = OcpConfig::ERK4) : Dynamics(7, 13, type) {}
  virtual ~SpatiotemporalPlannerModelDynamics() = default;

  virtual Eigen::VectorXd evaluate(const Eigen::VectorXd&, const Eigen::VectorXd&, const double) const override;
  virtual Eigen::MatrixXd evaluateJx(const Eigen::VectorXd&, const Eigen::VectorXd&, const double) const override;
  virtual Eigen::MatrixXd evaluateJu(const Eigen::VectorXd&, const Eigen::VectorXd&, const double) const override;
  virtual void updateEvaluation(const Eigen::VectorXd& x, const Eigen::VectorXd& u, const double, Eigen::VectorXd&) const override;
  virtual void updateJx(const Eigen::VectorXd& x, const Eigen::VectorXd& u, const double, Eigen::MatrixXd&) const override;
  virtual void updateJu(const Eigen::VectorXd& x, const Eigen::VectorXd& u, const double, Eigen::MatrixXd&) const override;
  virtual std::unique_ptr<Integrator> createIntegrator(OcpConfig::IntegratorType type) const override;
};

template <OcpConfig::IntegratorType Ttype>
class SpatiotemporalPlannerModelIntegrator : public Integrator {
 public:
  SpatiotemporalPlannerModelIntegrator(const Dynamics* dynamics) : Integrator(dynamics) {}
  virtual ~SpatiotemporalPlannerModelIntegrator() = default;

  virtual OcpConfig::IntegratorType type() const { return OcpConfig::FORWARD_EULER; }
  virtual void update(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, const double t_next, Eigen::VectorXd& F) const override {
    // Determine stage variables
    const auto& s = states(0);
    const auto& x = states(1);
    const auto& y = states(2);
    const auto& theta = states(3);
    const auto& steer = states(4);
    const auto& v = states(5);
    const auto& a = states(6);
    const auto& dsteer = ctrls(0);
    const auto& jerk = ctrls(1);

    // Determine global variables
    const auto& kappa = globals(0);
    const auto& ds = globals(8);

    // Determine internal variables
    const double dt = -t + t_next;
    const double internal_0 = dt*v;

    // Evaluation of Vector F
    F(0) = ds*internal_0 + s;
    F(1) = internal_0*cos(theta) + x;
    F(2) = internal_0*sin(theta) + y;
    F(3) = internal_0*kappa + theta;
    F(4) = dsteer*dt + steer;
    F(5) = a*dt + v;
    F(6) = a + dt*jerk;

  }

  virtual void update(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, const double t_next, Eigen::VectorXd& F, Eigen::MatrixXd& Fx, Eigen::MatrixXd& Fu) const override {
    // Determine stage variables
    const auto& s = states(0);
    const auto& x = states(1);
    const auto& y = states(2);
    const auto& theta = states(3);
    const auto& steer = states(4);
    const auto& v = states(5);
    const auto& a = states(6);
    const auto& dsteer = ctrls(0);
    const auto& jerk = ctrls(1);
    const auto& theta_ref = params(2);
    const auto& kappa_ref = params(3);
    const auto& wheelbase = params(14);

    // Determine global variables
    const auto& kappa = globals(0);
    const auto& l = globals(3);
    const auto& ds = globals(8);

    // Determine internal variables
    const double dt = -t + t_next;
    const double internal_0 = ds*dt;
    const double internal_2 = dt*cos(theta);
    const double internal_4 = dt*sin(theta);
    const double internal_6 = dt*kappa;
    const double internal_9 = dt*v;
    const double internal_7 = 1.0/(-kappa_ref*l + 1);
    const double internal_1 = internal_0*v;
    const double internal_3 = internal_2*v;
    const double internal_5 = internal_4*v;
    const double internal_8 = internal_1*internal_7*kappa_ref;

    // Evaluation of Vector F
    F(0) = internal_1 + s;
    F(1) = internal_3 + x;
    F(2) = internal_5 + y;
    F(3) = internal_6*v + theta;
    F(4) = dsteer*dt + steer;
    F(5) = a*dt + v;
    F(6) = a + dt*jerk;

    // Evaluation of Matrix Fx
    Fx(0, 0) = 1;
    Fx(0, 1) = -internal_8*sin(theta_ref);
    Fx(0, 2) = internal_8*cos(theta_ref);
    Fx(0, 3) = -internal_7*internal_9*sin(theta - theta_ref);
    Fx(0, 5) = internal_0;
    Fx(1, 1) = 1;
    Fx(1, 3) = -internal_5;
    Fx(1, 5) = internal_2;
    Fx(2, 2) = 1;
    Fx(2, 3) = internal_3;
    Fx(2, 5) = internal_4;
    Fx(3, 3) = 1;
    Fx(3, 4) = internal_9*(pow(tan(steer), 2) + 1)/wheelbase;
    Fx(3, 5) = internal_6;
    Fx(4, 4) = 1;
    Fx(5, 5) = 1;
    Fx(5, 6) = dt;
    Fx(6, 6) = 1;

    // Evaluation of Matrix Fu
    Fu(4, 0) = dt;
    Fu(6, 1) = dt;

  }

  virtual void updateFx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, const double t_next, Eigen::MatrixXd& Fx) const override {
    // Determine stage variables
    const auto& theta = states(3);
    const auto& steer = states(4);
    const auto& v = states(5);
    const auto& theta_ref = params(2);
    const auto& kappa_ref = params(3);
    const auto& wheelbase = params(14);

    // Determine global variables
    const auto& kappa = globals(0);
    const auto& l = globals(3);
    const auto& ds = globals(8);

    // Determine internal variables
    const double dt = -t + t_next;
    const double internal_1 = ds*dt;
    const double internal_3 = dt*v;
    const double internal_4 = dt*sin(theta);
    const double internal_5 = dt*cos(theta);
    const double internal_0 = 1.0/(-kappa_ref*l + 1);
    const double internal_2 = internal_0*internal_1*kappa_ref*v;

    // Evaluation of Matrix Fx
    Fx(0, 0) = 1;
    Fx(0, 1) = -internal_2*sin(theta_ref);
    Fx(0, 2) = internal_2*cos(theta_ref);
    Fx(0, 3) = -internal_0*internal_3*sin(theta - theta_ref);
    Fx(0, 5) = internal_1;
    Fx(1, 1) = 1;
    Fx(1, 3) = -internal_4*v;
    Fx(1, 5) = internal_5;
    Fx(2, 2) = 1;
    Fx(2, 3) = internal_5*v;
    Fx(2, 5) = internal_4;
    Fx(3, 3) = 1;
    Fx(3, 4) = internal_3*(pow(tan(steer), 2) + 1)/wheelbase;
    Fx(3, 5) = dt*kappa;
    Fx(4, 4) = 1;
    Fx(5, 5) = 1;
    Fx(5, 6) = dt;
    Fx(6, 6) = 1;

  }

  virtual void updateFu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, const double t_next, Eigen::MatrixXd& Fu) const override {
    // Determine internal variables
    const double dt = -t + t_next;

    // Evaluation of Matrix Fu
    Fu(4, 0) = dt;
    Fu(6, 1) = dt;

  }

};

template <>
class SpatiotemporalPlannerModelIntegrator<OcpConfig::ERK4> : public Integrator {
 public:
  SpatiotemporalPlannerModelIntegrator(const Dynamics* dynamics) : Integrator(dynamics) {}
  virtual ~SpatiotemporalPlannerModelIntegrator() = default;

  virtual OcpConfig::IntegratorType type() const { return OcpConfig::ERK4; }
  virtual void update(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, const double t_next, Eigen::VectorXd& F) const override {
    // Determine stage variables
    const auto& s = states(0);
    const auto& x = states(1);
    const auto& y = states(2);
    const auto& theta = states(3);
    const auto& steer = states(4);
    const auto& v = states(5);
    const auto& a = states(6);
    const auto& dsteer = ctrls(0);
    const auto& jerk = ctrls(1);
    const auto& x_ref = params(0);
    const auto& y_ref = params(1);
    const auto& theta_ref = params(2);
    const auto& kappa_ref = params(3);
    const auto& wheelbase = params(14);

    // Determine global variables
    const auto& kappa = globals(0);
    const auto& ds = globals(8);

    // Determine internal variables
    const double internal_27 = 2*a;
    const double internal_16 = -theta_ref;
    const double dt = -t + t_next;
    const double internal_13 = x - x_ref;
    const double internal_10 = y - y_ref;
    const double internal_11 = cos(theta_ref);
    const double internal_14 = sin(theta_ref);
    const double internal_5 = 1.0/wheelbase;
    const double internal_18 = 0.16666666666666666*dt;
    const double internal_25 = 0.33333333333333331*dt;
    const double internal_3 = 0.5*dt;
    const double internal_19 = v*sin(theta);
    const double internal_20 = v*cos(theta);
    const double internal_0 = dt*jerk;
    const double internal_6 = dsteer*dt;
    const double internal_4 = a*internal_3;
    const double internal_26 = internal_6 + steer;
    const double internal_1 = dt*(a + 0.5*internal_0);
    const double internal_21 = internal_4 + v;
    const double internal_7 = internal_5*tan(0.5*internal_6 + steer);
    const double internal_15 = internal_1 + v;
    const double internal_2 = 0.5*internal_1 + v;
    const double internal_22 = internal_21*internal_3*kappa + theta;
    const double internal_17 = dt*internal_7*(1.5*internal_1 + v) + theta;
    const double internal_23 = internal_21*sin(internal_22);
    const double internal_24 = internal_21*cos(internal_22);
    const double internal_8 = internal_3*internal_7*(internal_2 + internal_4) + theta;
    const double internal_12 = internal_2*cos(internal_8);
    const double internal_9 = internal_2*sin(internal_8);

    // Evaluation of Vector F
    F(0) = internal_18*(ds*v + internal_15*cos(internal_16 + internal_17)/(-kappa_ref*(internal_11*(dt*internal_9 + internal_10) - internal_14*(dt*internal_12 + internal_13)) + 1)) + internal_25*(internal_2*cos(internal_16 + internal_8)/(-kappa_ref*(internal_11*(internal_10 + internal_23*internal_3) - internal_14*(internal_13 + internal_24*internal_3)) + 1) + internal_21*cos(internal_16 + internal_22)/(-kappa_ref*(internal_11*(internal_10 + internal_19*internal_3) - internal_14*(internal_13 + internal_20*internal_3)) + 1)) + s;
    F(1) = internal_18*(internal_15*cos(internal_17) + internal_20) + internal_25*(internal_12 + internal_24) + x;
    F(2) = internal_18*(internal_15*sin(internal_17) + internal_19) + internal_25*(internal_23 + internal_9) + y;
    F(3) = internal_18*(internal_15*internal_5*tan(internal_26) + kappa*v) + internal_25*(internal_2*internal_7 + internal_21*internal_7) + theta;
    F(4) = internal_26;
    F(5) = internal_18*(internal_0 + internal_27) + internal_25*(internal_0 + internal_27) + v;
    F(6) = a + internal_0;

  }

  virtual void update(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, const double t_next, Eigen::VectorXd& F, Eigen::MatrixXd& Fx, Eigen::MatrixXd& Fu) const override {
    // Determine stage variables
    const auto& s = states(0);
    const auto& x = states(1);
    const auto& y = states(2);
    const auto& theta = states(3);
    const auto& steer = states(4);
    const auto& v = states(5);
    const auto& a = states(6);
    const auto& dsteer = ctrls(0);
    const auto& jerk = ctrls(1);
    const auto& x_ref = params(0);
    const auto& y_ref = params(1);
    const auto& theta_ref = params(2);
    const auto& kappa_ref = params(3);
    const auto& wheelbase = params(14);

    // Determine global variables
    const auto& kappa = globals(0);
    const auto& l = globals(3);
    const auto& ds = globals(8);

    // Determine internal variables
    const double internal_71 = 2*a;
    const double internal_26 = -theta_ref;
    const double dt = -t + t_next;
    const double internal_35 = sin(theta);
    const double internal_38 = cos(theta);
    const double internal_22 = x - x_ref;
    const double internal_17 = y - y_ref;
    const double internal_18 = cos(theta_ref);
    const double internal_23 = sin(theta_ref);
    const double internal_8 = 1.0/wheelbase;
    const double internal_0 = ds*v;
    const double internal_33 = 0.16666666666666666*dt;
    const double internal_5 = 0.5*dt;
    const double internal_59 = 0.33333333333333331*dt;
    const double internal_1 = dt*jerk;
    const double internal_112 = pow(dt, 3);
    const double internal_88 = pow(dt, 2);
    const double internal_9 = dsteer*dt;
    const double internal_36 = internal_35*v;
    const double internal_39 = internal_38*v;
    const double internal_72 = internal_23*kappa_ref;
    const double internal_80 = internal_18*kappa_ref;
    const double internal_73 = 1.0/(-kappa_ref*l + 1);
    const double internal_113 = 0.5*internal_112;
    const double internal_118 = 0.125*internal_112;
    const double internal_120 = 0.16666666666666666*internal_88;
    const double internal_125 = 0.25*internal_112;
    const double internal_127 = 0.083333333333333329*internal_112;
    const double internal_129 = 0.125*pow(dt, 4);
    const double internal_43 = internal_5*kappa;
    const double internal_6 = a*internal_5;
    const double internal_89 = 0.5*internal_88;
    const double internal_97 = 0.25*internal_88;
    const double internal_98 = internal_8*(pow(tan(steer), 2) + 1);
    const double internal_67 = internal_9 + steer;
    const double internal_37 = internal_36*internal_5;
    const double internal_40 = internal_39*internal_5;
    const double internal_74 = internal_0*internal_73;
    const double internal_10 = tan(0.5*internal_9 + steer);
    const double internal_107 = internal_97*kappa;
    const double internal_119 = internal_118*kappa;
    const double internal_2 = dt*(a + 0.5*internal_1);
    const double internal_34 = internal_6 + v;
    const double internal_68 = tan(internal_67);
    const double internal_11 = internal_10*internal_8;
    const double internal_27 = 1.5*internal_2 + v;
    const double internal_3 = internal_2 + v;
    const double internal_4 = 0.5*internal_2 + v;
    const double internal_44 = internal_34*internal_43 + theta;
    const double internal_69 = internal_68*internal_8;
    const double internal_99 = pow(internal_34, 2)*internal_98;
    const double internal_102 = internal_11*internal_89;
    const double internal_116 = internal_11*internal_88;
    const double internal_12 = internal_11*internal_5;
    const double internal_130 = 0.75*internal_11*internal_112;
    const double internal_28 = dt*internal_11;
    const double internal_7 = internal_4 + internal_6;
    const double internal_70 = internal_11*internal_4;
    const double internal_90 = internal_8*(pow(internal_10, 2) + 1);
    const double internal_117 = 1.5*internal_116;
    const double internal_122 = internal_3*internal_8*(pow(internal_68, 2) + 1);
    const double internal_45 = internal_26 + internal_44;
    const double internal_48 = sin(internal_44);
    const double internal_51 = cos(internal_44);
    const double internal_41 = -kappa_ref*(internal_18*(internal_17 + internal_37) - internal_23*(internal_22 + internal_40)) + 1;
    const double internal_123 = internal_34*internal_90;
    const double internal_124 = internal_4*internal_90;
    const double internal_91 = internal_7*internal_90;
    const double internal_95 = internal_27*internal_90;
    const double internal_100 = internal_48*internal_97;
    const double internal_109 = internal_48*internal_5;
    const double internal_13 = internal_12*internal_7 + theta;
    const double internal_29 = internal_27*internal_28 + theta;
    const double internal_46 = cos(internal_45);
    const double internal_49 = internal_34*internal_48;
    const double internal_52 = internal_34*internal_51;
    const double internal_42 = 1.0/internal_41;
    const double internal_101 = internal_5*internal_91;
    const double internal_121 = internal_120*internal_95;
    const double internal_126 = internal_125*internal_91;
    const double internal_128 = internal_127*internal_91;
    const double internal_131 = internal_127*internal_95;
    const double internal_92 = internal_89*internal_91;
    const double internal_14 = sin(internal_13);
    const double internal_19 = cos(internal_13);
    const double internal_30 = internal_26 + internal_29;
    const double internal_50 = internal_49*internal_5;
    const double internal_53 = internal_5*internal_52;
    const double internal_56 = internal_13 + internal_26;
    const double internal_60 = cos(internal_29);
    const double internal_63 = sin(internal_29);
    const double internal_77 = internal_34*internal_46/pow(internal_41, 2);
    const double internal_47 = internal_42*internal_46;
    const double internal_82 = internal_42*sin(internal_45);
    const double internal_108 = -0.5*dt*internal_51 + internal_107*internal_49;
    const double internal_110 = internal_107*internal_52 + internal_109;
    const double internal_104 = dt*internal_14;
    const double internal_15 = internal_14*internal_4;
    const double internal_20 = internal_19*internal_4;
    const double internal_31 = cos(internal_30);
    const double internal_57 = cos(internal_56);
    const double internal_61 = internal_3*internal_60;
    const double internal_64 = internal_3*internal_63;
    const double internal_86 = internal_77*kappa_ref;
    const double internal_83 = internal_34*internal_82;
    const double internal_103 = internal_102*internal_15;
    const double internal_105 = internal_102*internal_20;
    const double internal_114 = internal_11*internal_15;
    const double internal_115 = internal_11*internal_20;
    const double internal_16 = dt*internal_15;
    const double internal_21 = dt*internal_20;
    const double internal_65 = internal_36 + internal_64;
    const double internal_66 = internal_15 + internal_49;
    const double internal_93 = internal_18*internal_20;
    const double internal_94 = internal_15*internal_23;
    const double internal_54 = -kappa_ref*(internal_18*(internal_17 + internal_50) - internal_23*(internal_22 + internal_53)) + 1;
    const double internal_62 = internal_33*(internal_39 + internal_61) + internal_59*(internal_20 + internal_52);
    const double internal_55 = 1.0/internal_54;
    const double internal_78 = pow(internal_54, -2);
    const double internal_24 = -kappa_ref*(internal_18*(internal_16 + internal_17) - internal_23*(internal_21 + internal_22)) + 1;
    const double internal_58 = internal_55*internal_57;
    const double internal_79 = internal_4*internal_57*internal_78;
    const double internal_84 = internal_55*sin(internal_56);
    const double internal_111 = internal_70*internal_84;
    const double internal_25 = 1.0/internal_24;
    const double internal_75 = pow(internal_24, -2);
    const double internal_85 = internal_4*internal_84;
    const double internal_87 = internal_79*kappa_ref;
    const double internal_32 = internal_25*internal_31;
    const double internal_76 = internal_3*internal_31*internal_75;
    const double internal_81 = internal_25*internal_3*sin(internal_30);
    const double internal_106 = internal_76*kappa_ref;
    const double internal_96 = internal_81*internal_95;

    // Evaluation of Vector F
    F(0) = internal_33*(internal_0 + internal_3*internal_32) + internal_59*(internal_34*internal_47 + internal_4*internal_58) + s;
    F(1) = internal_62 + x;
    F(2) = internal_33*internal_65 + internal_59*internal_66 + y;
    F(3) = internal_33*(internal_3*internal_69 + kappa*v) + internal_59*(internal_11*internal_34 + internal_70) + theta;
    F(4) = internal_67;
    F(5) = internal_33*(internal_1 + internal_71) + internal_59*(internal_1 + internal_71) + v;
    F(6) = a + internal_1;

    // Evaluation of Matrix Fx
    Fx(0, 0) = 1;
    Fx(0, 1) = internal_33*(-internal_72*internal_74 - internal_72*internal_76) + internal_59*(-internal_72*internal_77 - internal_72*internal_79);
    Fx(0, 2) = internal_33*(internal_74*internal_80 + internal_76*internal_80) + internal_59*(internal_77*internal_80 + internal_79*internal_80);
    Fx(0, 3) = internal_33*(internal_3*internal_31*internal_75*kappa_ref*(internal_16*internal_23 + internal_18*internal_21) - internal_73*v*sin(internal_26 + theta) - internal_81) + internal_59*(-internal_83 - internal_85 + internal_86*(internal_18*internal_40 + internal_23*internal_37) + internal_87*(internal_18*internal_53 + internal_23*internal_50));
    Fx(0, 4) = internal_33*(-dt*internal_96 + internal_3*internal_31*internal_75*kappa_ref*(internal_92*internal_93 + internal_92*internal_94)) + internal_59*(-internal_101*internal_85 + internal_4*internal_57*internal_78*kappa_ref*(internal_100*internal_23*internal_99 + internal_18*internal_51*internal_97*internal_99) - internal_5*internal_82*internal_99);
    Fx(0, 5) = internal_33*(ds + internal_106*(internal_18*(internal_104 + internal_105) - internal_23*(dt*internal_19 - internal_103)) - internal_28*internal_81 + internal_32) + internal_59*(-internal_111*internal_5 - internal_43*internal_83 + internal_47 + internal_58 + internal_86*(internal_18*internal_35*internal_5 - internal_23*internal_38*internal_5) + internal_87*(internal_108*internal_23 + internal_110*internal_18));
    Fx(0, 6) = internal_33*(dt*internal_32 + internal_106*(internal_18*(internal_113*internal_115 + internal_14*internal_89) - internal_23*(-internal_113*internal_114 + 0.5*internal_19*internal_88)) - internal_117*internal_81) + internal_59*(-internal_107*internal_83 - internal_111*internal_89 + internal_47*internal_5 + internal_5*internal_58 + internal_87*(internal_18*(internal_100 + internal_119*internal_52) - internal_23*(-internal_119*internal_49 + 0.25*internal_51*internal_88)));
    Fx(1, 1) = 1;
    Fx(1, 3) = -internal_33*internal_65 - internal_59*internal_66;
    Fx(1, 4) = 0.33333333333333331*dt*(-internal_101*internal_15 - internal_109*internal_99) - internal_121*internal_64;
    Fx(1, 5) = internal_33*(-internal_28*internal_64 + internal_38 + internal_60) + internal_59*(-internal_12*internal_15 + internal_19 - internal_43*internal_49 + internal_51);
    Fx(1, 6) = internal_33*(dt*internal_60 - internal_117*internal_64) + internal_59*(0.5*dt*internal_19 - internal_103 - internal_108);
    Fx(2, 2) = 1;
    Fx(2, 3) = internal_62;
    Fx(2, 4) = internal_121*internal_61 + internal_59*(internal_101*internal_20 + internal_5*internal_51*internal_99);
    Fx(2, 5) = internal_33*(internal_28*internal_61 + internal_35 + internal_63) + internal_59*(internal_12*internal_20 + internal_14 + internal_43*internal_52 + internal_48);
    Fx(2, 6) = internal_33*(dt*internal_63 + internal_117*internal_61) + internal_59*(0.5*internal_104 + internal_105 + internal_110);
    Fx(3, 3) = 1;
    Fx(3, 4) = internal_33*(internal_122 + internal_98*v) + internal_59*(internal_123 + internal_124);
    Fx(3, 5) = 0.66666666666666663*internal_28 + internal_33*(internal_69 + kappa);
    Fx(3, 6) = 0.33333333333333331*internal_116 + internal_120*internal_69;
    Fx(4, 4) = 1;
    Fx(5, 5) = 1;
    Fx(5, 6) = dt;
    Fx(6, 6) = 1;

    // Evaluation of Matrix Fu
    Fu(0, 0) = 0.16666666666666666*dt*(internal_3*internal_31*internal_75*kappa_ref*(internal_126*internal_93 + internal_126*internal_94) - internal_89*internal_96) - internal_128*internal_85;
    Fu(0, 1) = internal_33*(internal_106*(internal_18*(internal_115*internal_129 + internal_125*internal_14) - internal_23*(0.25*internal_112*internal_19 - internal_114*internal_129)) - internal_130*internal_81 + internal_32*internal_89) + internal_59*(-internal_111*internal_118 + 0.25*internal_55*internal_57*internal_88);
    Fu(1, 0) = -internal_128*internal_15 - internal_131*internal_64;
    Fu(1, 1) = internal_33*(-internal_130*internal_64 + 0.5*internal_60*internal_88) + internal_59*(-internal_114*internal_118 + 0.25*internal_19*internal_88);
    Fu(2, 0) = internal_128*internal_20 + internal_131*internal_61;
    Fu(2, 1) = internal_33*(internal_130*internal_61 + internal_63*internal_89) + internal_59*(internal_115*internal_118 + internal_14*internal_97);
    Fu(3, 0) = internal_120*internal_122 + internal_59*(internal_123*internal_5 + internal_124*internal_5);
    Fu(3, 1) = internal_11*internal_127 + internal_127*internal_69;
    Fu(4, 0) = dt;
    Fu(5, 1) = internal_89;
    Fu(6, 1) = dt;

  }

  virtual void updateFx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, const double t_next, Eigen::MatrixXd& Fx) const override {
    // Determine stage variables
    const auto& x = states(1);
    const auto& y = states(2);
    const auto& theta = states(3);
    const auto& steer = states(4);
    const auto& v = states(5);
    const auto& a = states(6);
    const auto& dsteer = ctrls(0);
    const auto& jerk = ctrls(1);
    const auto& x_ref = params(0);
    const auto& y_ref = params(1);
    const auto& theta_ref = params(2);
    const auto& kappa_ref = params(3);
    const auto& wheelbase = params(14);

    // Determine global variables
    const auto& kappa = globals(0);
    const auto& l = globals(3);
    const auto& ds = globals(8);

    // Determine internal variables
    const double internal_13 = -theta_ref;
    const double dt = -t + t_next;
    const double internal_40 = sin(theta);
    const double internal_43 = cos(theta);
    const double internal_30 = x - x_ref;
    const double internal_26 = y - y_ref;
    const double internal_0 = sin(theta_ref);
    const double internal_27 = cos(theta_ref);
    const double internal_4 = 1.0/wheelbase;
    const double internal_34 = 0.16666666666666666*dt;
    const double internal_59 = 0.33333333333333331*dt;
    const double internal_8 = 0.5*dt;
    const double internal_14 = internal_13 + theta;
    const double internal_5 = dsteer*dt;
    const double internal_71 = pow(dt, 2);
    const double internal_93 = pow(dt, 3);
    const double internal_41 = internal_40*v;
    const double internal_44 = internal_43*v;
    const double internal_1 = internal_0*kappa_ref;
    const double internal_60 = internal_27*kappa_ref;
    const double internal_103 = 0.16666666666666666*internal_71;
    const double internal_19 = a*internal_8;
    const double internal_36 = internal_8*kappa;
    const double internal_72 = 0.5*internal_71;
    const double internal_77 = 0.25*internal_71;
    const double internal_78 = internal_4*(pow(tan(steer), 2) + 1);
    const double internal_2 = v/(-kappa_ref*l + 1);
    const double internal_98 = 0.125*internal_93*kappa;
    const double internal_42 = internal_41*internal_8;
    const double internal_45 = internal_44*internal_8;
    const double internal_108 = tan(internal_5 + steer);
    const double internal_35 = internal_19 + v;
    const double internal_6 = tan(0.5*internal_5 + steer);
    const double internal_89 = internal_77*kappa;
    const double internal_9 = dt*(a + internal_8*jerk);
    const double internal_3 = ds*internal_2;
    const double internal_10 = 1.5*internal_9 + v;
    const double internal_109 = internal_108*internal_4;
    const double internal_17 = internal_9 + v;
    const double internal_18 = 0.5*internal_9 + v;
    const double internal_7 = internal_4*internal_6;
    const double internal_37 = internal_35*internal_36 + theta;
    const double internal_79 = pow(internal_35, 2)*internal_78;
    const double internal_94 = 0.5*internal_7;
    const double internal_107 = internal_17*internal_4;
    const double internal_11 = dt*internal_10;
    const double internal_20 = internal_18 + internal_19;
    const double internal_21 = internal_7*internal_8;
    const double internal_73 = internal_4*(pow(internal_6, 2) + 1);
    const double internal_82 = internal_7*internal_72;
    const double internal_86 = dt*internal_7;
    const double internal_96 = internal_7*internal_71;
    const double internal_38 = internal_13 + internal_37;
    const double internal_50 = sin(internal_37);
    const double internal_53 = cos(internal_37);
    const double internal_95 = internal_93*internal_94;
    const double internal_97 = 1.5*internal_96;
    const double internal_46 = -kappa_ref*(-internal_0*(internal_30 + internal_45) + internal_27*(internal_26 + internal_42)) + 1;
    const double internal_104 = internal_10*internal_103*internal_73;
    const double internal_12 = internal_11*internal_7;
    const double internal_74 = internal_20*internal_73;
    const double internal_102 = 0.5*internal_74;
    const double internal_22 = internal_20*internal_21 + theta;
    const double internal_39 = cos(internal_38);
    const double internal_51 = internal_35*internal_50;
    const double internal_54 = internal_35*internal_53;
    const double internal_80 = internal_50*internal_77;
    const double internal_91 = internal_50*internal_8;
    const double internal_63 = 1.0/internal_46;
    const double internal_15 = internal_12 + internal_14;
    const double internal_75 = internal_72*internal_74;
    const double internal_99 = internal_12 + theta;
    const double internal_23 = sin(internal_22);
    const double internal_28 = cos(internal_22);
    const double internal_47 = internal_35*internal_39/pow(internal_46, 2);
    const double internal_48 = internal_13 + internal_22;
    const double internal_52 = internal_51*internal_8;
    const double internal_55 = internal_54*internal_8;
    const double internal_64 = internal_63*sin(internal_38);
    const double internal_87 = internal_39*internal_63;
    const double internal_100 = sin(internal_99);
    const double internal_105 = cos(internal_99);
    const double internal_16 = cos(internal_15);
    const double internal_90 = -0.5*dt*internal_53 + internal_51*internal_89;
    const double internal_92 = internal_54*internal_89 + internal_91;
    const double internal_24 = dt*internal_23;
    const double internal_29 = dt*internal_18*internal_28;
    const double internal_49 = cos(internal_48);
    const double internal_68 = internal_47*kappa_ref;
    const double internal_70 = internal_18*internal_28;
    const double internal_76 = internal_18*internal_23;
    const double internal_65 = internal_35*internal_64;
    const double internal_101 = internal_100*internal_17;
    const double internal_106 = internal_105*internal_17;
    const double internal_25 = internal_18*internal_24;
    const double internal_83 = internal_76*internal_82;
    const double internal_84 = internal_70*internal_82;
    const double internal_56 = -kappa_ref*(-internal_0*(internal_30 + internal_55) + internal_27*(internal_26 + internal_52)) + 1;
    const double internal_57 = pow(internal_56, -2);
    const double internal_66 = 1.0/internal_56;
    const double internal_31 = -kappa_ref*(-internal_0*(internal_29 + internal_30) + internal_27*(internal_25 + internal_26)) + 1;
    const double internal_58 = internal_18*internal_49*internal_57;
    const double internal_67 = internal_18*internal_66*sin(internal_48);
    const double internal_88 = internal_49*internal_66;
    const double internal_32 = pow(internal_31, -2);
    const double internal_61 = 1.0/internal_31;
    const double internal_69 = internal_58*kappa_ref;
    const double internal_33 = internal_16*internal_17*internal_32;
    const double internal_62 = internal_17*internal_61*sin(internal_15);
    const double internal_81 = internal_16*internal_61;
    const double internal_85 = internal_33*kappa_ref;

    // Evaluation of Matrix Fx
    Fx(0, 0) = 1;
    Fx(0, 1) = internal_34*(-internal_1*internal_3 - internal_1*internal_33) + internal_59*(-internal_1*internal_47 - internal_1*internal_58);
    Fx(0, 2) = internal_34*(internal_3*internal_60 + internal_33*internal_60) + internal_59*(internal_47*internal_60 + internal_58*internal_60);
    Fx(0, 3) = internal_34*(internal_16*internal_17*internal_32*kappa_ref*(internal_0*internal_25 + internal_27*internal_29) - internal_2*sin(internal_14) - internal_62) + internal_59*(-internal_65 - internal_67 + internal_68*(internal_0*internal_42 + internal_27*internal_45) + internal_69*(internal_0*internal_52 + internal_27*internal_55));
    Fx(0, 4) = internal_34*(-internal_11*internal_62*internal_73 + internal_16*internal_17*internal_32*kappa_ref*(internal_0*internal_75*internal_76 + internal_27*internal_70*internal_75)) + internal_59*(internal_18*internal_49*internal_57*kappa_ref*(internal_0*internal_79*internal_80 + internal_27*internal_53*internal_77*internal_79) - internal_64*internal_79*internal_8 - internal_67*internal_74*internal_8);
    Fx(0, 5) = internal_34*(ds - internal_62*internal_86 + internal_81 + internal_85*(-internal_0*(dt*internal_28 - internal_83) + internal_27*(internal_24 + internal_84))) + internal_59*(-internal_21*internal_67 - internal_36*internal_65 + internal_68*(-internal_0*internal_43*internal_8 + internal_27*internal_40*internal_8) + internal_69*(internal_0*internal_90 + internal_27*internal_92) + internal_87 + internal_88);
    Fx(0, 6) = internal_34*(dt*internal_81 - internal_62*internal_97 + internal_85*(-internal_0*(0.5*internal_28*internal_71 - internal_76*internal_95) + internal_27*(internal_23*internal_72 + internal_70*internal_95))) + internal_59*(-internal_65*internal_89 - internal_67*internal_82 + internal_69*(-internal_0*(-internal_51*internal_98 + 0.25*internal_53*internal_71) + internal_27*(internal_54*internal_98 + internal_80)) + internal_8*internal_87 + internal_8*internal_88);
    Fx(1, 1) = 1;
    Fx(1, 3) = internal_34*(-internal_101 - internal_41) + internal_59*(-internal_51 - internal_76);
    Fx(1, 4) = 0.33333333333333331*dt*(-internal_102*internal_25 - internal_79*internal_91) - internal_101*internal_104;
    Fx(1, 5) = internal_34*(-internal_101*internal_86 + internal_105 + internal_43) + internal_59*(-internal_25*internal_94 + internal_28 - internal_36*internal_51 + internal_53);
    Fx(1, 6) = internal_34*(dt*internal_105 - internal_101*internal_97) + internal_59*(0.5*dt*internal_28 - internal_83 - internal_90);
    Fx(2, 2) = 1;
    Fx(2, 3) = internal_34*(internal_106 + internal_44) + internal_59*(internal_54 + internal_70);
    Fx(2, 4) = internal_104*internal_106 + internal_59*(internal_102*internal_29 + internal_53*internal_79*internal_8);
    Fx(2, 5) = internal_34*(dt*internal_105*internal_107*internal_6 + internal_100 + internal_40) + internal_59*(internal_23 + internal_29*internal_94 + internal_36*internal_54 + internal_50);
    Fx(2, 6) = internal_34*(dt*internal_100 + internal_106*internal_97) + internal_59*(0.5*internal_24 + internal_84 + internal_92);
    Fx(3, 3) = 1;
    Fx(3, 4) = internal_34*(internal_107*(pow(internal_108, 2) + 1) + internal_78*v) + internal_59*(internal_18*internal_73 + internal_35*internal_73);
    Fx(3, 5) = internal_34*(internal_109 + kappa) + 0.66666666666666663*internal_86;
    Fx(3, 6) = internal_103*internal_109 + 0.33333333333333331*internal_96;
    Fx(4, 4) = 1;
    Fx(5, 5) = 1;
    Fx(5, 6) = dt;
    Fx(6, 6) = 1;

  }

  virtual void updateFu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, const double t_next, Eigen::MatrixXd& Fu) const override {
    // Determine stage variables
    const auto& x = states(1);
    const auto& y = states(2);
    const auto& theta = states(3);
    const auto& steer = states(4);
    const auto& v = states(5);
    const auto& a = states(6);
    const auto& dsteer = ctrls(0);
    const auto& jerk = ctrls(1);
    const auto& x_ref = params(0);
    const auto& y_ref = params(1);
    const auto& theta_ref = params(2);
    const auto& kappa_ref = params(3);
    const auto& wheelbase = params(14);

    // Determine global variables
    const auto& kappa = globals(0);

    // Determine internal variables
    const double internal_27 = -theta_ref;
    const double dt = -t + t_next;
    const double internal_15 = x - x_ref;
    const double internal_12 = y - y_ref;
    const double internal_13 = cos(theta_ref);
    const double internal_16 = sin(theta_ref);
    const double internal_5 = 1.0/wheelbase;
    const double internal_0 = 0.5*dt;
    const double internal_52 = 0.16666666666666666*dt;
    const double internal_54 = 0.33333333333333331*dt;
    const double internal_20 = pow(dt, 3);
    const double internal_34 = pow(dt, 2);
    const double internal_6 = dsteer*dt;
    const double internal_1 = a*internal_0;
    const double internal_21 = 0.25*internal_20;
    const double internal_35 = 0.5*internal_34;
    const double internal_42 = 0.083333333333333329*internal_20;
    const double internal_2 = dt*(a + internal_0*jerk);
    const double internal_43 = internal_42*internal_5;
    const double internal_59 = tan(internal_6 + steer);
    const double internal_7 = tan(0.5*internal_6 + steer);
    const double internal_37 = internal_0*(internal_1 + v);
    const double internal_19 = internal_2 + v;
    const double internal_28 = 1.5*internal_2 + v;
    const double internal_3 = 0.5*internal_2 + v;
    const double internal_8 = internal_5*internal_7;
    const double internal_22 = pow(internal_7, 2) + 1;
    const double internal_46 = 0.125*internal_8;
    const double internal_11 = dt*internal_3;
    const double internal_38 = internal_37*kappa + theta;
    const double internal_4 = internal_1 + internal_3;
    const double internal_50 = 0.75*internal_20*internal_8;
    const double internal_23 = internal_22*internal_5;
    const double internal_29 = dt*internal_28*internal_8 + theta;
    const double internal_44 = internal_22*internal_43;
    const double internal_47 = pow(dt, 4)*internal_46;
    const double internal_53 = internal_20*internal_46;
    const double internal_51 = internal_19*internal_50;
    const double internal_24 = internal_23*internal_3;
    const double internal_30 = internal_27 + internal_29;
    const double internal_45 = internal_4*internal_44;
    const double internal_55 = sin(internal_29);
    const double internal_57 = internal_28*internal_44;
    const double internal_58 = cos(internal_29);
    const double internal_9 = internal_0*internal_4*internal_8 + theta;
    const double internal_10 = sin(internal_9);
    const double internal_14 = cos(internal_9);
    const double internal_25 = internal_24*internal_4;
    const double internal_31 = cos(internal_30);
    const double internal_32 = sin(internal_30);
    const double internal_40 = internal_27 + internal_9;
    const double internal_56 = internal_19*internal_55;
    const double internal_26 = internal_10*internal_21;
    const double internal_48 = internal_10*internal_3;
    const double internal_49 = internal_14*internal_3;
    const double internal_39 = 1.0/(-kappa_ref*(internal_13*(internal_12 + internal_37*sin(internal_38)) - internal_16*(internal_15 + internal_37*cos(internal_38))) + 1);
    const double internal_41 = internal_3*internal_39*sin(internal_40);
    const double internal_17 = -kappa_ref*(internal_13*(internal_10*internal_11 + internal_12) - internal_16*(internal_11*internal_14 + internal_15)) + 1;
    const double internal_18 = pow(internal_17, -2);
    const double internal_33 = 1.0/internal_17;
    const double internal_36 = internal_33*internal_35;

    // Evaluation of Matrix Fu
    Fu(0, 0) = 0.16666666666666666*dt*(internal_18*internal_19*internal_31*kappa_ref*(internal_13*internal_14*internal_21*internal_25 + internal_16*internal_25*internal_26) - internal_19*internal_23*internal_28*internal_32*internal_36) - internal_41*internal_45;
    Fu(0, 1) = internal_52*(internal_18*internal_19*internal_31*kappa_ref*(internal_13*(internal_26 + internal_47*internal_49) - internal_16*(0.25*internal_14*internal_20 - internal_47*internal_48)) + internal_31*internal_36 - internal_32*internal_33*internal_51) + internal_54*(0.25*internal_34*internal_39*cos(internal_40) - internal_41*internal_53);
    Fu(1, 0) = -internal_45*internal_48 - internal_56*internal_57;
    Fu(1, 1) = internal_52*(0.5*internal_34*internal_58 - internal_50*internal_56) + internal_54*(0.25*internal_14*internal_34 - internal_48*internal_53);
    Fu(2, 0) = internal_19*internal_57*internal_58 + internal_45*internal_49;
    Fu(2, 1) = internal_52*(internal_35*internal_55 + internal_51*internal_58) + internal_54*(0.25*internal_10*internal_34 + internal_49*internal_53);
    Fu(3, 0) = 0.16666666666666666*internal_19*internal_34*internal_5*(pow(internal_59, 2) + 1) + internal_54*(internal_0*internal_24 + internal_23*internal_37);
    Fu(3, 1) = internal_42*internal_8 + internal_43*internal_59;
    Fu(4, 0) = dt;
    Fu(5, 1) = internal_35;
    Fu(6, 1) = dt;

  }

};

}  // namespace gpal::pnc::planning