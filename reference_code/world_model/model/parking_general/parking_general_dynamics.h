#pragma once

#include "ocp/dynamics.h"

namespace gpal::pnc::planning {

class ParkingGeneralDynamics : public Dynamics {
 public:
  explicit ParkingGeneralDynamics(OcpConfig::IntegratorType type = OcpConfig::ERK4) : Dynamics(6, 3, type) {}
  virtual ~ParkingGeneralDynamics() = default;

  virtual Eigen::VectorXd evaluate(const Eigen::VectorXd&, const Eigen::VectorXd&, const double) const override;
  virtual Eigen::MatrixXd evaluateJx(const Eigen::VectorXd&, const Eigen::VectorXd&, const double) const override;
  virtual Eigen::MatrixXd evaluateJu(const Eigen::VectorXd&, const Eigen::VectorXd&, const double) const override;
  virtual void updateEvaluation(const Eigen::VectorXd& x, const Eigen::VectorXd& u, const double, Eigen::VectorXd&) const override;
  virtual void updateJx(const Eigen::VectorXd& x, const Eigen::VectorXd& u, const double, Eigen::MatrixXd&) const override;
  virtual void updateJu(const Eigen::VectorXd& x, const Eigen::VectorXd& u, const double, Eigen::MatrixXd&) const override;
  virtual std::unique_ptr<Integrator> createIntegrator(OcpConfig::IntegratorType type) const override;
};

template <OcpConfig::IntegratorType Ttype>
class ParkingGeneralIntegrator : public Integrator {
 public:
  ParkingGeneralIntegrator(const Dynamics* dynamics) : Integrator(dynamics) {}
  virtual ~ParkingGeneralIntegrator() = default;

  virtual OcpConfig::IntegratorType type() const { return OcpConfig::FORWARD_EULER; }
  virtual void update(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, const double t_next, Eigen::VectorXd& F) const override {
    // Determine stage variables
    const auto& s = states(0);
    const auto& x = states(1);
    const auto& y = states(2);
    const auto& theta = states(3);
    const auto& steer = states(4);
    const auto& v = states(5);
    const auto& dsteer = ctrls(0);
    const auto& a = ctrls(1);

    // Determine global variables
    const auto& kappa = globals(0);
    const auto& ds = globals(4);

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

  }

  virtual void update(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, const double t_next, Eigen::VectorXd& F, Eigen::MatrixXd& Fx, Eigen::MatrixXd& Fu) const override {
    // Determine stage variables
    const auto& s = states(0);
    const auto& x = states(1);
    const auto& y = states(2);
    const auto& theta = states(3);
    const auto& steer = states(4);
    const auto& v = states(5);
    const auto& dsteer = ctrls(0);
    const auto& a = ctrls(1);
    const auto& wheelbase = params(0);
    const auto& thetar = params(3);
    const auto& kr = params(4);

    // Determine global variables
    const auto& kappa = globals(0);
    const auto& l = globals(1);
    const auto& ds = globals(4);

    // Determine internal variables
    const double dt = -t + t_next;
    const double internal_0 = ds*dt;
    const double internal_2 = dt*cos(theta);
    const double internal_4 = dt*sin(theta);
    const double internal_6 = dt*kappa;
    const double internal_9 = dt*v;
    const double internal_7 = 1.0/(-kr*l + 1);
    const double internal_1 = internal_0*v;
    const double internal_3 = internal_2*v;
    const double internal_5 = internal_4*v;
    const double internal_8 = internal_1*internal_7*kr;

    // Evaluation of Vector F
    F(0) = internal_1 + s;
    F(1) = internal_3 + x;
    F(2) = internal_5 + y;
    F(3) = internal_6*v + theta;
    F(4) = dsteer*dt + steer;
    F(5) = a*dt + v;

    // Evaluation of Matrix Fx
    Fx(0, 0) = 1;
    Fx(0, 1) = -internal_8*sin(thetar);
    Fx(0, 2) = internal_8*cos(thetar);
    Fx(0, 3) = -internal_7*internal_9*sin(theta - thetar);
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

    // Evaluation of Matrix Fu
    Fu(4, 0) = dt;
    Fu(5, 1) = dt;

  }

  virtual void updateFx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, const double t_next, Eigen::MatrixXd& Fx) const override {
    // Determine stage variables
    const auto& theta = states(3);
    const auto& steer = states(4);
    const auto& v = states(5);
    const auto& wheelbase = params(0);
    const auto& thetar = params(3);
    const auto& kr = params(4);

    // Determine global variables
    const auto& kappa = globals(0);
    const auto& l = globals(1);
    const auto& ds = globals(4);

    // Determine internal variables
    const double dt = -t + t_next;
    const double internal_1 = ds*dt;
    const double internal_3 = dt*v;
    const double internal_4 = dt*sin(theta);
    const double internal_5 = dt*cos(theta);
    const double internal_0 = 1.0/(-kr*l + 1);
    const double internal_2 = internal_0*internal_1*kr*v;

    // Evaluation of Matrix Fx
    Fx(0, 0) = 1;
    Fx(0, 1) = -internal_2*sin(thetar);
    Fx(0, 2) = internal_2*cos(thetar);
    Fx(0, 3) = -internal_0*internal_3*sin(theta - thetar);
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

  }

  virtual void updateFu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, const double t_next, Eigen::MatrixXd& Fu) const override {
    // Determine internal variables
    const double dt = -t + t_next;

    // Evaluation of Matrix Fu
    Fu(4, 0) = dt;
    Fu(5, 1) = dt;

  }

};

template <>
class ParkingGeneralIntegrator<OcpConfig::ERK4> : public Integrator {
 public:
  ParkingGeneralIntegrator(const Dynamics* dynamics) : Integrator(dynamics) {}
  virtual ~ParkingGeneralIntegrator() = default;

  virtual OcpConfig::IntegratorType type() const { return OcpConfig::ERK4; }
  virtual void update(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, const double t_next, Eigen::VectorXd& F) const override {
    // Determine stage variables
    const auto& s = states(0);
    const auto& x = states(1);
    const auto& y = states(2);
    const auto& theta = states(3);
    const auto& steer = states(4);
    const auto& v = states(5);
    const auto& dsteer = ctrls(0);
    const auto& a = ctrls(1);
    const auto& wheelbase = params(0);
    const auto& xr = params(1);
    const auto& yr = params(2);
    const auto& thetar = params(3);
    const auto& kr = params(4);

    // Determine global variables
    const auto& kappa = globals(0);
    const auto& ds = globals(4);

    // Determine internal variables
    const double internal_13 = -thetar;
    const double dt = -t + t_next;
    const double internal_2 = 1.0/wheelbase;
    const double internal_10 = x - xr;
    const double internal_7 = y - yr;
    const double internal_11 = sin(thetar);
    const double internal_8 = cos(thetar);
    const double internal_15 = 0.16666666666666666*dt;
    const double internal_17 = 0.5*dt;
    const double internal_22 = 0.33333333333333331*dt;
    const double internal_16 = v*sin(theta);
    const double internal_18 = v*cos(theta);
    const double internal_0 = a*dt;
    const double internal_3 = dsteer*dt;
    const double internal_1 = 0.5*internal_0 + v;
    const double internal_12 = internal_0 + v;
    const double internal_23 = internal_3 + steer;
    const double internal_4 = dt*internal_2*tan(0.5*internal_3 + steer);
    const double internal_19 = internal_1*internal_17*kappa + theta;
    const double internal_14 = internal_4*(1.5*internal_0 + v) + theta;
    const double internal_5 = 0.5*internal_4*(internal_0 + v) + theta;
    const double internal_20 = internal_1*sin(internal_19);
    const double internal_21 = internal_1*cos(internal_19);
    const double internal_6 = internal_1*sin(internal_5);
    const double internal_9 = internal_1*cos(internal_5);

    // Evaluation of Vector F
    F(0) = internal_15*(ds*v + internal_12*cos(internal_13 + internal_14)/(-kr*(-internal_11*(dt*internal_9 + internal_10) + internal_8*(dt*internal_6 + internal_7)) + 1)) + internal_22*(internal_1*cos(internal_13 + internal_5)/(-kr*(-internal_11*(internal_10 + internal_17*internal_21) + internal_8*(internal_17*internal_20 + internal_7)) + 1) + internal_1*cos(internal_13 + internal_19)/(-kr*(-internal_11*(internal_10 + internal_17*internal_18) + internal_8*(internal_16*internal_17 + internal_7)) + 1)) + s;
    F(1) = internal_15*(internal_12*cos(internal_14) + internal_18) + internal_22*(internal_21 + internal_9) + x;
    F(2) = internal_15*(internal_12*sin(internal_14) + internal_16) + internal_22*(internal_20 + internal_6) + y;
    F(3) = 0.66666666666666663*internal_1*internal_4 + internal_15*(internal_12*internal_2*tan(internal_23) + kappa*v) + theta;
    F(4) = internal_23;
    F(5) = internal_12;

  }

  virtual void update(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, const double t_next, Eigen::VectorXd& F, Eigen::MatrixXd& Fx, Eigen::MatrixXd& Fu) const override {
    // Determine stage variables
    const auto& s = states(0);
    const auto& x = states(1);
    const auto& y = states(2);
    const auto& theta = states(3);
    const auto& steer = states(4);
    const auto& v = states(5);
    const auto& dsteer = ctrls(0);
    const auto& a = ctrls(1);
    const auto& wheelbase = params(0);
    const auto& xr = params(1);
    const auto& yr = params(2);
    const auto& thetar = params(3);
    const auto& kr = params(4);

    // Determine global variables
    const auto& kappa = globals(0);
    const auto& l = globals(1);
    const auto& ds = globals(4);

    // Determine internal variables
    const double internal_24 = -thetar;
    const double dt = -t + t_next;
    const double internal_32 = sin(theta);
    const double internal_35 = cos(theta);
    const double internal_6 = 1.0/wheelbase;
    const double internal_20 = x - xr;
    const double internal_15 = y - yr;
    const double internal_16 = cos(thetar);
    const double internal_21 = sin(thetar);
    const double internal_0 = ds*v;
    const double internal_31 = 0.16666666666666666*dt;
    const double internal_5 = 0.5*dt;
    const double internal_56 = 0.33333333333333331*dt;
    const double internal_1 = a*dt;
    const double internal_114 = pow(dt, 3);
    const double internal_7 = dsteer*dt;
    const double internal_86 = pow(dt, 2);
    const double internal_33 = internal_32*v;
    const double internal_36 = internal_35*v;
    const double internal_68 = internal_21*kr;
    const double internal_77 = internal_16*kr;
    const double internal_69 = 1.0/(-kr*l + 1);
    const double internal_109 = 0.16666666666666666*internal_86;
    const double internal_116 = 0.083333333333333329*internal_114;
    const double internal_122 = 0.33333333333333331*internal_86;
    const double internal_40 = internal_5*kappa;
    const double internal_87 = 0.5*internal_86;
    const double internal_95 = 0.25*internal_86;
    const double internal_96 = internal_6*(pow(tan(steer), 2) + 1);
    const double internal_120 = 0.125*internal_114*kappa;
    const double internal_2 = internal_1 + v;
    const double internal_25 = 1.5*internal_1 + v;
    const double internal_3 = 0.5*internal_1 + v;
    const double internal_4 = internal_1 + v;
    const double internal_64 = internal_7 + steer;
    const double internal_34 = internal_33*internal_5;
    const double internal_37 = internal_36*internal_5;
    const double internal_70 = internal_0*internal_69;
    const double internal_104 = internal_95*kappa;
    const double internal_8 = tan(0.5*internal_7 + steer);
    const double internal_65 = tan(internal_64);
    const double internal_73 = internal_3*internal_68;
    const double internal_78 = internal_3*internal_77;
    const double internal_83 = internal_3*kr;
    const double internal_41 = internal_3*internal_40 + theta;
    const double internal_9 = internal_6*internal_8;
    const double internal_97 = pow(internal_3, 2)*internal_96;
    const double internal_111 = 0.5*internal_9;
    const double internal_66 = internal_6*internal_65;
    const double internal_10 = internal_5*internal_9;
    const double internal_119 = 1.5*internal_86*internal_9;
    const double internal_26 = dt*internal_9;
    const double internal_42 = internal_24 + internal_41;
    const double internal_45 = sin(internal_41);
    const double internal_48 = cos(internal_41);
    const double internal_88 = internal_6*(pow(internal_8, 2) + 1);
    const double internal_99 = internal_87*internal_9;
    const double internal_112 = internal_2*internal_6*(pow(internal_65, 2) + 1);
    const double internal_118 = internal_111*internal_114;
    const double internal_67 = 0.66666666666666663*internal_26;
    const double internal_38 = -kr*(internal_16*(internal_15 + internal_34) - internal_21*(internal_20 + internal_37)) + 1;
    const double internal_106 = internal_45*internal_5;
    const double internal_113 = internal_3*internal_88;
    const double internal_43 = cos(internal_42);
    const double internal_46 = internal_3*internal_45;
    const double internal_49 = internal_3*internal_48;
    const double internal_89 = internal_4*internal_88;
    const double internal_93 = internal_25*internal_88;
    const double internal_98 = internal_45*internal_95;
    const double internal_108 = 0.5*internal_89;
    const double internal_11 = internal_10*internal_4 + theta;
    const double internal_27 = internal_25*internal_26 + theta;
    const double internal_39 = 1.0/internal_38;
    const double internal_110 = internal_109*internal_93;
    const double internal_115 = 0.25*internal_114*internal_89;
    const double internal_117 = internal_116*internal_89;
    const double internal_121 = internal_116*internal_93;
    const double internal_47 = internal_46*internal_5;
    const double internal_50 = internal_49*internal_5;
    const double internal_90 = internal_87*internal_89;
    const double internal_74 = internal_43/pow(internal_38, 2);
    const double internal_105 = -0.5*dt*internal_48 + internal_104*internal_46;
    const double internal_107 = internal_104*internal_49 + internal_106;
    const double internal_12 = sin(internal_11);
    const double internal_17 = cos(internal_11);
    const double internal_28 = internal_24 + internal_27;
    const double internal_53 = internal_11 + internal_24;
    const double internal_57 = cos(internal_27);
    const double internal_60 = sin(internal_27);
    const double internal_44 = internal_39*internal_43;
    const double internal_80 = internal_39*sin(internal_42);
    const double internal_84 = internal_74*internal_83;
    const double internal_101 = dt*internal_12;
    const double internal_13 = internal_12*internal_3;
    const double internal_18 = internal_17*internal_3;
    const double internal_29 = cos(internal_28);
    const double internal_54 = cos(internal_53);
    const double internal_58 = internal_2*internal_57;
    const double internal_61 = internal_2*internal_60;
    const double internal_81 = internal_3*internal_80;
    const double internal_100 = internal_13*internal_99;
    const double internal_102 = internal_18*internal_99;
    const double internal_14 = dt*internal_13;
    const double internal_19 = dt*internal_18;
    const double internal_62 = internal_33 + internal_61;
    const double internal_63 = internal_13 + internal_46;
    const double internal_91 = internal_16*internal_18;
    const double internal_92 = internal_13*internal_21;
    const double internal_51 = -kr*(internal_16*(internal_15 + internal_47) - internal_21*(internal_20 + internal_50)) + 1;
    const double internal_59 = internal_31*(internal_36 + internal_58) + internal_56*(internal_18 + internal_49);
    const double internal_52 = 1.0/internal_51;
    const double internal_75 = pow(internal_51, -2);
    const double internal_55 = internal_52*internal_54;
    const double internal_76 = internal_54*internal_75;
    const double internal_82 = internal_3*internal_52*sin(internal_53);
    const double internal_22 = -kr*(internal_16*(internal_14 + internal_15) - internal_21*(internal_19 + internal_20)) + 1;
    const double internal_85 = internal_76*internal_83;
    const double internal_23 = 1.0/internal_22;
    const double internal_71 = pow(internal_22, -2);
    const double internal_30 = internal_23*internal_29;
    const double internal_72 = internal_2*internal_29*internal_71;
    const double internal_79 = internal_2*internal_23*sin(internal_28);
    const double internal_103 = internal_72*kr;
    const double internal_94 = internal_79*internal_93;

    // Evaluation of Vector F
    F(0) = internal_31*(internal_0 + internal_2*internal_30) + internal_56*(internal_3*internal_44 + internal_3*internal_55) + s;
    F(1) = internal_59 + x;
    F(2) = internal_31*internal_62 + internal_56*internal_63 + y;
    F(3) = internal_3*internal_67 + internal_31*(internal_2*internal_66 + kappa*v) + theta;
    F(4) = internal_64;
    F(5) = internal_2;

    // Evaluation of Matrix Fx
    Fx(0, 0) = 1;
    Fx(0, 1) = internal_31*(-internal_68*internal_70 - internal_68*internal_72) + internal_56*(-internal_73*internal_74 - internal_73*internal_76);
    Fx(0, 2) = internal_31*(internal_70*internal_77 + internal_72*internal_77) + internal_56*(internal_74*internal_78 + internal_76*internal_78);
    Fx(0, 3) = internal_31*(internal_2*internal_29*internal_71*kr*(internal_14*internal_21 + internal_16*internal_19) - internal_69*v*sin(internal_24 + theta) - internal_79) + internal_56*(-internal_81 - internal_82 + internal_84*(internal_16*internal_37 + internal_21*internal_34) + internal_85*(internal_16*internal_50 + internal_21*internal_47));
    Fx(0, 4) = internal_31*(-dt*internal_94 + internal_2*internal_29*internal_71*kr*(internal_90*internal_91 + internal_90*internal_92)) + internal_56*(internal_3*internal_54*internal_75*kr*(internal_16*internal_48*internal_95*internal_97 + internal_21*internal_97*internal_98) - internal_5*internal_80*internal_97 - internal_5*internal_82*internal_89);
    Fx(0, 5) = internal_31*(ds + internal_103*(internal_16*(internal_101 + internal_102) - internal_21*(dt*internal_17 - internal_100)) - internal_26*internal_79 + internal_30) + internal_56*(-internal_10*internal_82 - internal_40*internal_81 + internal_44 + internal_55 + internal_84*(internal_16*internal_32*internal_5 - internal_21*internal_35*internal_5) + internal_85*(internal_105*internal_21 + internal_107*internal_16));
    Fx(1, 1) = 1;
    Fx(1, 3) = -internal_31*internal_62 - internal_56*internal_63;
    Fx(1, 4) = 0.33333333333333331*dt*(-internal_106*internal_97 - internal_108*internal_14) - internal_110*internal_61;
    Fx(1, 5) = internal_31*(-internal_26*internal_61 + internal_35 + internal_57) + internal_56*(-internal_111*internal_14 + internal_17 - internal_40*internal_46 + internal_48);
    Fx(2, 2) = 1;
    Fx(2, 3) = internal_59;
    Fx(2, 4) = internal_110*internal_58 + internal_56*(internal_108*internal_19 + internal_48*internal_5*internal_97);
    Fx(2, 5) = internal_31*(internal_26*internal_58 + internal_32 + internal_60) + internal_56*(internal_111*internal_19 + internal_12 + internal_40*internal_49 + internal_45);
    Fx(3, 3) = 1;
    Fx(3, 4) = 0.66666666666666663*dt*internal_113 + internal_31*(internal_112 + internal_96*v);
    Fx(3, 5) = internal_31*(internal_66 + kappa) + internal_67;
    Fx(4, 4) = 1;
    Fx(5, 5) = 1;

    // Evaluation of Matrix Fu
    Fu(0, 0) = 0.16666666666666666*dt*(internal_2*internal_29*internal_71*kr*(internal_115*internal_91 + internal_115*internal_92) - internal_87*internal_94) - internal_117*internal_82;
    Fu(0, 1) = internal_31*(dt*internal_30 + internal_103*(internal_16*(internal_118*internal_18 + internal_12*internal_87) - internal_21*(-internal_118*internal_13 + 0.5*internal_17*internal_86)) - internal_119*internal_79) + internal_56*(-internal_104*internal_81 + internal_44*internal_5 + internal_5*internal_55 - internal_82*internal_99 + internal_85*(internal_16*(internal_120*internal_49 + internal_98) - internal_21*(-internal_120*internal_46 + 0.25*internal_48*internal_86)));
    Fu(1, 0) = -internal_117*internal_13 - internal_121*internal_61;
    Fu(1, 1) = internal_31*(dt*internal_57 - internal_119*internal_61) + internal_56*(0.5*dt*internal_17 - internal_100 - internal_105);
    Fu(2, 0) = internal_117*internal_18 + internal_121*internal_58;
    Fu(2, 1) = internal_31*(dt*internal_60 + internal_119*internal_58) + internal_56*(0.5*internal_101 + internal_102 + internal_107);
    Fu(3, 0) = internal_109*internal_112 + internal_113*internal_122;
    Fu(3, 1) = internal_109*internal_66 + internal_122*internal_9;
    Fu(4, 0) = dt;
    Fu(5, 1) = dt;

  }

  virtual void updateFx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, const double t_next, Eigen::MatrixXd& Fx) const override {
    // Determine stage variables
    const auto& x = states(1);
    const auto& y = states(2);
    const auto& theta = states(3);
    const auto& steer = states(4);
    const auto& v = states(5);
    const auto& dsteer = ctrls(0);
    const auto& a = ctrls(1);
    const auto& wheelbase = params(0);
    const auto& xr = params(1);
    const auto& yr = params(2);
    const auto& thetar = params(3);
    const auto& kr = params(4);

    // Determine global variables
    const auto& kappa = globals(0);
    const auto& l = globals(1);
    const auto& ds = globals(4);

    // Determine internal variables
    const double internal_12 = -thetar;
    const double dt = -t + t_next;
    const double internal_39 = sin(theta);
    const double internal_42 = cos(theta);
    const double internal_6 = 1.0/wheelbase;
    const double internal_29 = x - xr;
    const double internal_25 = y - yr;
    const double internal_0 = sin(thetar);
    const double internal_26 = cos(thetar);
    const double internal_19 = 0.5*dt;
    const double internal_33 = 0.16666666666666666*dt;
    const double internal_58 = 0.33333333333333331*dt;
    const double internal_13 = internal_12 + theta;
    const double internal_4 = a*dt;
    const double internal_7 = dsteer*dt;
    const double internal_72 = pow(dt, 2);
    const double internal_40 = internal_39*v;
    const double internal_43 = internal_42*v;
    const double internal_1 = internal_0*kr;
    const double internal_59 = internal_26*kr;
    const double internal_35 = internal_19*kappa;
    const double internal_79 = 0.25*internal_72;
    const double internal_80 = internal_6*(pow(tan(steer), 2) + 1);
    const double internal_2 = v/(-kr*l + 1);
    const double internal_16 = internal_4 + v;
    const double internal_17 = 0.5*internal_4 + v;
    const double internal_18 = internal_4 + v;
    const double internal_5 = 1.5*internal_4 + v;
    const double internal_41 = internal_19*internal_40;
    const double internal_44 = internal_19*internal_43;
    const double internal_8 = tan(0.5*internal_7 + steer);
    const double internal_85 = internal_79*kappa;
    const double internal_93 = tan(internal_7 + steer);
    const double internal_3 = ds*internal_2;
    const double internal_34 = internal_1*internal_17;
    const double internal_60 = internal_17*internal_59;
    const double internal_68 = internal_17*kr;
    const double internal_36 = internal_17*internal_35 + theta;
    const double internal_81 = pow(internal_17, 2)*internal_80;
    const double internal_9 = internal_6*internal_8;
    const double internal_83 = 0.5*internal_9;
    const double internal_10 = dt*internal_9;
    const double internal_20 = internal_19*internal_9;
    const double internal_37 = internal_12 + internal_36;
    const double internal_49 = sin(internal_36);
    const double internal_52 = cos(internal_36);
    const double internal_73 = internal_6*(pow(internal_8, 2) + 1);
    const double internal_82 = internal_79*internal_81;
    const double internal_84 = internal_72*internal_83;
    const double internal_45 = -kr*(-internal_0*(internal_29 + internal_44) + internal_26*(internal_25 + internal_41)) + 1;
    const double internal_11 = internal_10*internal_5;
    const double internal_38 = cos(internal_37);
    const double internal_50 = internal_17*internal_49;
    const double internal_53 = internal_17*internal_52;
    const double internal_74 = internal_18*internal_73;
    const double internal_78 = dt*internal_73;
    const double internal_86 = internal_19*internal_49;
    const double internal_90 = 0.16666666666666666*internal_5*internal_72*internal_73;
    const double internal_21 = internal_18*internal_20 + theta;
    const double internal_75 = 0.5*internal_74;
    const double internal_63 = 1.0/internal_45;
    const double internal_14 = internal_11 + internal_13;
    const double internal_51 = internal_19*internal_50;
    const double internal_54 = internal_19*internal_53;
    const double internal_87 = internal_11 + theta;
    const double internal_46 = internal_38/pow(internal_45, 2);
    const double internal_22 = sin(internal_21);
    const double internal_27 = cos(internal_21);
    const double internal_47 = internal_12 + internal_21;
    const double internal_76 = internal_72*internal_75;
    const double internal_64 = internal_63*sin(internal_37);
    const double internal_15 = cos(internal_14);
    const double internal_88 = sin(internal_87);
    const double internal_91 = cos(internal_87);
    const double internal_69 = internal_46*internal_68;
    const double internal_23 = dt*internal_22;
    const double internal_28 = dt*internal_17*internal_27;
    const double internal_48 = cos(internal_47);
    const double internal_71 = internal_17*internal_27;
    const double internal_77 = internal_17*internal_22;
    const double internal_65 = internal_17*internal_64;
    const double internal_89 = internal_16*internal_88;
    const double internal_92 = internal_16*internal_91;
    const double internal_24 = internal_17*internal_23;
    const double internal_55 = -kr*(-internal_0*(internal_29 + internal_54) + internal_26*(internal_25 + internal_51)) + 1;
    const double internal_56 = pow(internal_55, -2);
    const double internal_66 = 1.0/internal_55;
    const double internal_57 = internal_48*internal_56;
    const double internal_67 = internal_17*internal_66*sin(internal_47);
    const double internal_30 = -kr*(-internal_0*(internal_28 + internal_29) + internal_26*(internal_24 + internal_25)) + 1;
    const double internal_70 = internal_57*internal_68;
    const double internal_31 = pow(internal_30, -2);
    const double internal_61 = 1.0/internal_30;
    const double internal_32 = internal_15*internal_16*internal_31;
    const double internal_62 = internal_16*internal_61*sin(internal_14);

    // Evaluation of Matrix Fx
    Fx(0, 0) = 1;
    Fx(0, 1) = internal_33*(-internal_1*internal_3 - internal_1*internal_32) + internal_58*(-internal_34*internal_46 - internal_34*internal_57);
    Fx(0, 2) = internal_33*(internal_3*internal_59 + internal_32*internal_59) + internal_58*(internal_46*internal_60 + internal_57*internal_60);
    Fx(0, 3) = internal_33*(internal_15*internal_16*internal_31*kr*(internal_0*internal_24 + internal_26*internal_28) - internal_2*sin(internal_13) - internal_62) + internal_58*(-internal_65 - internal_67 + internal_69*(internal_0*internal_41 + internal_26*internal_44) + internal_70*(internal_0*internal_51 + internal_26*internal_54));
    Fx(0, 4) = internal_33*(internal_15*internal_16*internal_31*kr*(internal_0*internal_76*internal_77 + internal_26*internal_71*internal_76) - internal_5*internal_62*internal_78) + internal_58*(internal_17*internal_48*internal_56*kr*(internal_0*internal_49*internal_82 + internal_26*internal_52*internal_82) - internal_19*internal_64*internal_81 - internal_19*internal_67*internal_74);
    Fx(0, 5) = internal_33*(ds - internal_10*internal_62 + internal_15*internal_61 + internal_32*kr*(-internal_0*(dt*internal_27 - internal_77*internal_84) + internal_26*(internal_23 + internal_71*internal_84))) + internal_58*(-internal_20*internal_67 - internal_35*internal_65 + internal_38*internal_63 + internal_48*internal_66 + internal_69*(-internal_0*internal_19*internal_42 + internal_19*internal_26*internal_39) + internal_70*(-internal_0*(0.5*dt*internal_52 - internal_50*internal_85) + internal_26*(internal_53*internal_85 + internal_86)));
    Fx(1, 1) = 1;
    Fx(1, 3) = internal_33*(-internal_40 - internal_89) + internal_58*(-internal_50 - internal_77);
    Fx(1, 4) = 0.33333333333333331*dt*(-internal_24*internal_75 - internal_81*internal_86) - internal_89*internal_90;
    Fx(1, 5) = internal_33*(-internal_10*internal_89 + internal_42 + internal_91) + internal_58*(-internal_24*internal_83 + internal_27 - internal_35*internal_50 + internal_52);
    Fx(2, 2) = 1;
    Fx(2, 3) = internal_33*(internal_43 + internal_92) + internal_58*(internal_53 + internal_71);
    Fx(2, 4) = internal_58*(internal_19*internal_52*internal_81 + internal_28*internal_75) + internal_90*internal_92;
    Fx(2, 5) = internal_33*(internal_10*internal_92 + internal_39 + internal_88) + internal_58*(internal_22 + internal_28*internal_83 + internal_35*internal_53 + internal_49);
    Fx(3, 3) = 1;
    Fx(3, 4) = 0.66666666666666663*internal_17*internal_78 + internal_33*(internal_16*internal_6*(pow(internal_93, 2) + 1) + internal_80*v);
    Fx(3, 5) = 0.66666666666666663*internal_10 + internal_33*(internal_6*internal_93 + kappa);
    Fx(4, 4) = 1;
    Fx(5, 5) = 1;

  }

  virtual void updateFu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, const double t_next, Eigen::MatrixXd& Fu) const override {
    // Determine stage variables
    const auto& x = states(1);
    const auto& y = states(2);
    const auto& theta = states(3);
    const auto& steer = states(4);
    const auto& v = states(5);
    const auto& dsteer = ctrls(0);
    const auto& a = ctrls(1);
    const auto& wheelbase = params(0);
    const auto& xr = params(1);
    const auto& yr = params(2);
    const auto& thetar = params(3);
    const auto& kr = params(4);

    // Determine global variables
    const auto& kappa = globals(0);

    // Determine internal variables
    const double internal_26 = -thetar;
    const double dt = -t + t_next;
    const double internal_3 = 1.0/wheelbase;
    const double internal_14 = x - xr;
    const double internal_11 = y - yr;
    const double internal_12 = cos(thetar);
    const double internal_15 = sin(thetar);
    const double internal_2 = 0.5*dt;
    const double internal_52 = 0.16666666666666666*dt;
    const double internal_61 = 0.33333333333333331*dt;
    const double internal_44 = 0.083333333333333329*internal_3;
    const double internal_0 = a*dt;
    const double internal_19 = pow(dt, 3);
    const double internal_31 = pow(dt, 2);
    const double internal_4 = dsteer*dt;
    const double internal_32 = 0.5*internal_31;
    const double internal_53 = internal_2*v;
    const double internal_59 = 0.25*internal_31;
    const double internal_1 = internal_0 + v;
    const double internal_18 = internal_0 + v;
    const double internal_27 = 1.5*internal_0 + v;
    const double internal_45 = internal_19*internal_44;
    const double internal_68 = internal_3*internal_31;
    const double internal_9 = 0.5*internal_0 + v;
    const double internal_5 = tan(0.5*internal_4 + steer);
    const double internal_67 = tan(internal_4 + steer);
    const double internal_69 = 0.16666666666666666*internal_68;
    const double internal_10 = dt*internal_9;
    const double internal_57 = internal_9*kappa;
    const double internal_36 = 0.5*internal_10;
    const double internal_6 = internal_3*internal_5;
    const double internal_21 = pow(internal_5, 2) + 1;
    const double internal_58 = 0.125*internal_19*internal_57;
    const double internal_46 = internal_6*internal_9;
    const double internal_49 = internal_31*internal_6;
    const double internal_22 = internal_21*internal_9;
    const double internal_28 = dt*internal_27*internal_6 + theta;
    const double internal_35 = internal_21*internal_27;
    const double internal_37 = internal_36*kappa + theta;
    const double internal_47 = 0.5*internal_46;
    const double internal_50 = 1.5*internal_49;
    const double internal_7 = internal_1*internal_2*internal_6 + theta;
    const double internal_54 = 1.0/(-kr*(internal_12*(internal_11 + internal_53*sin(theta)) - internal_15*(internal_14 + internal_53*cos(theta))) + 1);
    const double internal_13 = cos(internal_7);
    const double internal_23 = internal_1*internal_22;
    const double internal_29 = internal_26 + internal_28;
    const double internal_38 = sin(internal_37);
    const double internal_39 = cos(internal_37);
    const double internal_42 = internal_26 + internal_7;
    const double internal_51 = internal_18*internal_50;
    const double internal_55 = internal_26 + internal_37;
    const double internal_62 = sin(internal_28);
    const double internal_64 = internal_35*internal_45;
    const double internal_66 = cos(internal_28);
    const double internal_8 = sin(internal_7);
    const double internal_20 = internal_13*internal_19;
    const double internal_24 = 0.25*internal_23*internal_3;
    const double internal_25 = internal_19*internal_8;
    const double internal_30 = cos(internal_29);
    const double internal_48 = internal_32*internal_8;
    const double internal_56 = cos(internal_42);
    const double internal_60 = internal_38*internal_59;
    const double internal_63 = internal_18*internal_62;
    const double internal_65 = internal_23*internal_44;
    const double internal_16 = -kr*(internal_12*(internal_10*internal_8 + internal_11) - internal_15*(internal_10*internal_13 + internal_14)) + 1;
    const double internal_40 = -kr*(internal_12*(internal_11 + internal_36*internal_38) - internal_15*(internal_14 + internal_36*internal_39)) + 1;
    const double internal_17 = pow(internal_16, -2);
    const double internal_33 = 1.0/internal_16;
    const double internal_41 = 1.0/internal_40;
    const double internal_34 = internal_33*sin(internal_29);
    const double internal_43 = internal_41*sin(internal_42);

    // Evaluation of Matrix Fu
    Fu(0, 0) = 0.16666666666666666*dt*(internal_17*internal_18*internal_30*kr*(internal_12*internal_20*internal_24 + internal_15*internal_24*internal_25) - internal_18*internal_3*internal_32*internal_34*internal_35) - internal_23*internal_43*internal_45;
    Fu(0, 1) = internal_52*(dt*internal_30*internal_33 + internal_17*internal_18*internal_30*kr*(internal_12*(internal_20*internal_47 + internal_48) - internal_15*(0.5*internal_13*internal_31 - internal_25*internal_47)) - internal_34*internal_51) + internal_61*(internal_2*internal_41*internal_56 + internal_2*internal_54*cos(internal_55) - internal_32*internal_43*internal_46 - internal_54*internal_57*internal_59*sin(internal_55) + internal_56*internal_9*kr*(internal_12*(internal_39*internal_58 + internal_60) - internal_15*(0.25*internal_31*internal_39 - internal_38*internal_58))/pow(internal_40, 2));
    Fu(1, 0) = -internal_25*internal_65 - internal_63*internal_64;
    Fu(1, 1) = internal_52*(dt*internal_66 - internal_50*internal_63) + internal_61*(0.5*dt*internal_13 + 0.5*dt*internal_39 - internal_46*internal_48 - internal_57*internal_60);
    Fu(2, 0) = internal_18*internal_64*internal_66 + internal_20*internal_65;
    Fu(2, 1) = internal_52*(dt*internal_62 + internal_51*internal_66) + internal_61*(internal_13*internal_32*internal_46 + internal_2*internal_38 + internal_2*internal_8 + internal_39*internal_57*internal_59);
    Fu(3, 0) = internal_18*internal_69*(pow(internal_67, 2) + 1) + 0.33333333333333331*internal_22*internal_68;
    Fu(3, 1) = 0.33333333333333331*internal_49 + internal_67*internal_69;
    Fu(4, 0) = dt;
    Fu(5, 1) = dt;

  }

};

}  // namespace gpal::pnc::planning