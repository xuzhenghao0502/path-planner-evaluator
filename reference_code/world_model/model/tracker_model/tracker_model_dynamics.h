#pragma once

#include "ocp/dynamics.h"

namespace gpal::pnc::planning {

class TrackerModelDynamics : public Dynamics {
 public:
  explicit TrackerModelDynamics(OcpConfig::IntegratorType type = OcpConfig::ERK4) : Dynamics(5, 2, type) {}
  virtual ~TrackerModelDynamics() = default;

  virtual Eigen::VectorXd evaluate(const Eigen::VectorXd&, const Eigen::VectorXd&, const double) const override;
  virtual Eigen::MatrixXd evaluateJx(const Eigen::VectorXd&, const Eigen::VectorXd&, const double) const override;
  virtual Eigen::MatrixXd evaluateJu(const Eigen::VectorXd&, const Eigen::VectorXd&, const double) const override;
  virtual void updateEvaluation(const Eigen::VectorXd& x, const Eigen::VectorXd& u, const double, Eigen::VectorXd&) const override;
  virtual void updateJx(const Eigen::VectorXd& x, const Eigen::VectorXd& u, const double, Eigen::MatrixXd&) const override;
  virtual void updateJu(const Eigen::VectorXd& x, const Eigen::VectorXd& u, const double, Eigen::MatrixXd&) const override;
  virtual std::unique_ptr<Integrator> createIntegrator(OcpConfig::IntegratorType type) const override;
};

template <OcpConfig::IntegratorType Ttype>
class TrackerModelIntegrator : public Integrator {
 public:
  TrackerModelIntegrator(const Dynamics* dynamics) : Integrator(dynamics) {}
  virtual ~TrackerModelIntegrator() = default;

  virtual OcpConfig::IntegratorType type() const { return OcpConfig::FORWARD_EULER; }
  virtual void update(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, const double t_next, Eigen::VectorXd& F) const override {
    // Determine stage variables
    const auto& s = states(0);
    const auto& x = states(1);
    const auto& y = states(2);
    const auto& theta = states(3);
    const auto& steer = states(4);
    const auto& dsteer = ctrls(0);
    const auto& v_ref = params(4);

    // Determine global variables
    const auto& kappa = globals(0);
    const auto& ds = globals(2);

    // Determine internal variables
    const double dt = -t + t_next;
    const double internal_0 = dt*v_ref;

    // Evaluation of Vector F
    F(0) = ds*internal_0 + s;
    F(1) = internal_0*cos(theta) + x;
    F(2) = internal_0*sin(theta) + y;
    F(3) = internal_0*kappa + theta;
    F(4) = dsteer*dt + steer;

  }

  virtual void update(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, const double t_next, Eigen::VectorXd& F, Eigen::MatrixXd& Fx, Eigen::MatrixXd& Fu) const override {
    // Determine stage variables
    const auto& s = states(0);
    const auto& x = states(1);
    const auto& y = states(2);
    const auto& theta = states(3);
    const auto& steer = states(4);
    const auto& dsteer = ctrls(0);
    const auto& theta_ref = params(2);
    const auto& kappa_ref = params(3);
    const auto& v_ref = params(4);
    const auto& wheelbase = params(6);

    // Determine global variables
    const auto& kappa = globals(0);
    const auto& l = globals(1);
    const auto& ds = globals(2);

    // Determine internal variables
    const double dt = -t + t_next;
    const double internal_0 = dt*v_ref;
    const double internal_4 = 1.0/(-kappa_ref*l + 1);
    const double internal_1 = ds*internal_0;
    const double internal_2 = internal_0*cos(theta);
    const double internal_3 = internal_0*sin(theta);
    const double internal_5 = internal_1*internal_4*kappa_ref;

    // Evaluation of Vector F
    F(0) = internal_1 + s;
    F(1) = internal_2 + x;
    F(2) = internal_3 + y;
    F(3) = internal_0*kappa + theta;
    F(4) = dsteer*dt + steer;

    // Evaluation of Matrix Fx
    Fx(0, 0) = 1;
    Fx(0, 1) = -internal_5*sin(theta_ref);
    Fx(0, 2) = internal_5*cos(theta_ref);
    Fx(0, 3) = -internal_0*internal_4*sin(theta - theta_ref);
    Fx(1, 1) = 1;
    Fx(1, 3) = -internal_3;
    Fx(2, 2) = 1;
    Fx(2, 3) = internal_2;
    Fx(3, 3) = 1;
    Fx(3, 4) = internal_0*(pow(tan(steer), 2) + 1)/wheelbase;
    Fx(4, 4) = 1;

    // Evaluation of Matrix Fu
    Fu(4, 0) = dt;

  }

  virtual void updateFx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, const double t_next, Eigen::MatrixXd& Fx) const override {
    // Determine stage variables
    const auto& theta = states(3);
    const auto& steer = states(4);
    const auto& theta_ref = params(2);
    const auto& kappa_ref = params(3);
    const auto& v_ref = params(4);
    const auto& wheelbase = params(6);

    // Determine global variables
    const auto& l = globals(1);
    const auto& ds = globals(2);

    // Determine internal variables
    const double dt = -t + t_next;
    const double internal_0 = dt*v_ref;
    const double internal_1 = internal_0/(-kappa_ref*l + 1);
    const double internal_2 = ds*internal_1*kappa_ref;

    // Evaluation of Matrix Fx
    Fx(0, 0) = 1;
    Fx(0, 1) = -internal_2*sin(theta_ref);
    Fx(0, 2) = internal_2*cos(theta_ref);
    Fx(0, 3) = -internal_1*sin(theta - theta_ref);
    Fx(1, 1) = 1;
    Fx(1, 3) = -internal_0*sin(theta);
    Fx(2, 2) = 1;
    Fx(2, 3) = internal_0*cos(theta);
    Fx(3, 3) = 1;
    Fx(3, 4) = internal_0*(pow(tan(steer), 2) + 1)/wheelbase;
    Fx(4, 4) = 1;

  }

  virtual void updateFu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, const double t_next, Eigen::MatrixXd& Fu) const override {
    // Determine internal variables
    const double dt = -t + t_next;

    // Evaluation of Matrix Fu
    Fu(4, 0) = dt;

  }

};

template <>
class TrackerModelIntegrator<OcpConfig::ERK4> : public Integrator {
 public:
  TrackerModelIntegrator(const Dynamics* dynamics) : Integrator(dynamics) {}
  virtual ~TrackerModelIntegrator() = default;

  virtual OcpConfig::IntegratorType type() const { return OcpConfig::ERK4; }
  virtual void update(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, const double t_next, Eigen::VectorXd& F) const override {
    // Determine stage variables
    const auto& s = states(0);
    const auto& x = states(1);
    const auto& y = states(2);
    const auto& theta = states(3);
    const auto& steer = states(4);
    const auto& dsteer = ctrls(0);
    const auto& x_ref = params(0);
    const auto& y_ref = params(1);
    const auto& theta_ref = params(2);
    const auto& kappa_ref = params(3);
    const auto& v_ref = params(4);
    const auto& wheelbase = params(6);

    // Determine global variables
    const auto& kappa = globals(0);
    const auto& ds = globals(2);

    // Determine internal variables
    const double internal_10 = -theta_ref;
    const double dt = -t + t_next;
    const double internal_8 = x - x_ref;
    const double internal_5 = y - y_ref;
    const double internal_6 = cos(theta_ref);
    const double internal_9 = sin(theta_ref);
    const double internal_16 = kappa*v_ref;
    const double internal_12 = 0.16666666666666666*dt;
    const double internal_14 = 0.5*dt;
    const double internal_20 = 0.33333333333333331*dt;
    const double internal_13 = v_ref*sin(theta);
    const double internal_15 = v_ref*cos(theta);
    const double internal_1 = v_ref/wheelbase;
    const double internal_0 = dsteer*dt;
    const double internal_17 = internal_14*internal_16 + theta;
    const double internal_21 = internal_0 + steer;
    const double internal_2 = dt*internal_1*tan(0.5*internal_0 + steer);
    const double internal_18 = v_ref*sin(internal_17);
    const double internal_19 = v_ref*cos(internal_17);
    const double internal_11 = internal_2 + theta;
    const double internal_3 = 0.5*internal_2 + theta;
    const double internal_4 = v_ref*sin(internal_3);
    const double internal_7 = v_ref*cos(internal_3);

    // Evaluation of Vector F
    F(0) = internal_12*(ds*v_ref + v_ref*cos(internal_10 + internal_11)/(-kappa_ref*(internal_6*(dt*internal_4 + internal_5) - internal_9*(dt*internal_7 + internal_8)) + 1)) + internal_20*(v_ref*cos(internal_10 + internal_3)/(-kappa_ref*(internal_6*(internal_14*internal_18 + internal_5) - internal_9*(internal_14*internal_19 + internal_8)) + 1) + v_ref*cos(internal_10 + internal_17)/(-kappa_ref*(internal_6*(internal_13*internal_14 + internal_5) - internal_9*(internal_14*internal_15 + internal_8)) + 1)) + s;
    F(1) = internal_12*(internal_15 + v_ref*cos(internal_11)) + internal_20*(internal_19 + internal_7) + x;
    F(2) = internal_12*(internal_13 + v_ref*sin(internal_11)) + internal_20*(internal_18 + internal_4) + y;
    F(3) = internal_12*(internal_1*tan(internal_21) + internal_16) + 0.66666666666666663*internal_2 + theta;
    F(4) = internal_21;

  }

  virtual void update(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, const double t_next, Eigen::VectorXd& F, Eigen::MatrixXd& Fx, Eigen::MatrixXd& Fu) const override {
    // Determine stage variables
    const auto& s = states(0);
    const auto& x = states(1);
    const auto& y = states(2);
    const auto& theta = states(3);
    const auto& steer = states(4);
    const auto& dsteer = ctrls(0);
    const auto& x_ref = params(0);
    const auto& y_ref = params(1);
    const auto& theta_ref = params(2);
    const auto& kappa_ref = params(3);
    const auto& v_ref = params(4);
    const auto& wheelbase = params(6);

    // Determine global variables
    const auto& kappa = globals(0);
    const auto& l = globals(1);
    const auto& ds = globals(2);

    // Determine internal variables
    const double internal_1 = -theta_ref;
    const double dt = -t + t_next;
    const double internal_20 = x - x_ref;
    const double internal_15 = y - y_ref;
    const double internal_16 = cos(theta_ref);
    const double internal_21 = sin(theta_ref);
    const double internal_71 = kappa_ref*v_ref;
    const double internal_4 = 1.0/wheelbase;
    const double internal_26 = kappa*v_ref;
    const double internal_0 = ds*v_ref;
    const double internal_25 = 0.16666666666666666*dt;
    const double internal_27 = 0.5*dt;
    const double internal_49 = 0.33333333333333331*dt;
    const double internal_31 = v_ref*sin(theta);
    const double internal_33 = v_ref*cos(theta);
    const double internal_2 = dsteer*dt;
    const double internal_74 = pow(dt, 2);
    const double internal_81 = pow(tan(steer), 2) + 1;
    const double internal_57 = internal_21*kappa_ref;
    const double internal_66 = internal_16*kappa_ref;
    const double internal_5 = internal_4*v_ref;
    const double internal_76 = internal_4*pow(v_ref, 2);
    const double internal_58 = 1.0/(-kappa_ref*l + 1);
    const double internal_87 = 0.16666666666666666*internal_74;
    const double internal_32 = internal_27*internal_31;
    const double internal_34 = internal_27*internal_33;
    const double internal_28 = internal_26*internal_27 + theta;
    const double internal_55 = internal_2 + steer;
    const double internal_60 = internal_57*v_ref;
    const double internal_67 = internal_66*v_ref;
    const double internal_6 = dt*internal_5;
    const double internal_82 = internal_76*internal_81;
    const double internal_59 = internal_0*internal_58;
    const double internal_3 = tan(0.5*internal_2 + steer);
    const double internal_29 = internal_1 + internal_28;
    const double internal_40 = sin(internal_28);
    const double internal_43 = cos(internal_28);
    const double internal_56 = tan(internal_55);
    const double internal_83 = 0.25*internal_74*internal_82;
    const double internal_84 = internal_27*internal_82;
    const double internal_7 = internal_3*internal_6;
    const double internal_30 = cos(internal_29);
    const double internal_41 = internal_40*v_ref;
    const double internal_44 = internal_43*v_ref;
    const double internal_69 = sin(internal_29);
    const double internal_73 = pow(internal_3, 2) + 1;
    const double internal_11 = 0.5*internal_7 + theta;
    const double internal_8 = internal_7 + theta;
    const double internal_35 = -kappa_ref*(internal_16*(internal_15 + internal_32) - internal_21*(internal_20 + internal_34)) + 1;
    const double internal_42 = internal_27*internal_41;
    const double internal_45 = internal_27*internal_44;
    const double internal_75 = internal_73*internal_74;
    const double internal_79 = internal_73*internal_76;
    const double internal_89 = internal_5*(pow(internal_56, 2) + 1);
    const double internal_12 = sin(internal_11);
    const double internal_17 = cos(internal_11);
    const double internal_38 = internal_1 + internal_11;
    const double internal_50 = cos(internal_8);
    const double internal_52 = sin(internal_8);
    const double internal_9 = internal_1 + internal_8;
    const double internal_36 = 1.0/internal_35;
    const double internal_77 = 0.5*internal_75*internal_76;
    const double internal_85 = internal_27*internal_79;
    const double internal_88 = internal_79*internal_87;
    const double internal_90 = pow(dt, 3)*internal_79;
    const double internal_63 = internal_30/pow(internal_35, 2);
    const double internal_10 = cos(internal_9);
    const double internal_13 = internal_12*v_ref;
    const double internal_18 = internal_17*v_ref;
    const double internal_39 = cos(internal_38);
    const double internal_68 = sin(internal_9);
    const double internal_70 = sin(internal_38);
    const double internal_72 = internal_16*internal_17;
    const double internal_78 = internal_12*internal_21;
    const double internal_91 = 0.25*internal_90;
    const double internal_92 = 0.083333333333333329*internal_90;
    const double internal_37 = internal_36*v_ref;
    const double internal_53 = internal_31 + internal_52*v_ref;
    const double internal_14 = dt*internal_13;
    const double internal_19 = dt*internal_18;
    const double internal_54 = internal_13 + internal_41;
    const double internal_46 = -kappa_ref*(internal_16*(internal_15 + internal_42) - internal_21*(internal_20 + internal_45)) + 1;
    const double internal_51 = internal_25*(internal_33 + internal_50*v_ref) + internal_49*(internal_18 + internal_44);
    const double internal_47 = 1.0/internal_46;
    const double internal_64 = pow(internal_46, -2);
    const double internal_48 = internal_47*v_ref;
    const double internal_65 = internal_39*internal_64;
    const double internal_86 = internal_47*internal_70;
    const double internal_22 = -kappa_ref*(internal_16*(internal_14 + internal_15) - internal_21*(internal_19 + internal_20)) + 1;
    const double internal_23 = 1.0/internal_22;
    const double internal_61 = pow(internal_22, -2);
    const double internal_24 = internal_23*v_ref;
    const double internal_62 = internal_10*internal_61;
    const double internal_80 = internal_23*internal_68;

    // Evaluation of Vector F
    F(0) = internal_25*(internal_0 + internal_10*internal_24) + internal_49*(internal_30*internal_37 + internal_39*internal_48) + s;
    F(1) = internal_51 + x;
    F(2) = internal_25*internal_53 + internal_49*internal_54 + y;
    F(3) = internal_25*(internal_26 + internal_5*internal_56) + 0.66666666666666663*internal_7 + theta;
    F(4) = internal_55;

    // Evaluation of Matrix Fx
    Fx(0, 0) = 1;
    Fx(0, 1) = internal_25*(-internal_57*internal_59 - internal_60*internal_62) + internal_49*(-internal_60*internal_63 - internal_60*internal_65);
    Fx(0, 2) = internal_25*(internal_59*internal_66 + internal_62*internal_67) + internal_49*(internal_63*internal_67 + internal_65*internal_67);
    Fx(0, 3) = internal_25*(internal_10*internal_61*kappa_ref*v_ref*(internal_14*internal_21 + internal_16*internal_19) - internal_24*internal_68 - internal_58*v_ref*sin(internal_1 + theta)) + internal_49*(-internal_37*internal_69 - internal_48*internal_70 + internal_63*internal_71*(internal_16*internal_34 + internal_21*internal_32) + internal_65*internal_71*(internal_16*internal_45 + internal_21*internal_42));
    Fx(0, 4) = internal_25*(-dt*internal_79*internal_80 + internal_10*internal_61*kappa_ref*v_ref*(internal_72*internal_77 + internal_77*internal_78)) + internal_49*(-internal_36*internal_69*internal_84 + internal_39*internal_64*kappa_ref*v_ref*(internal_16*internal_43*internal_83 + internal_21*internal_40*internal_83) - internal_85*internal_86);
    Fx(1, 1) = 1;
    Fx(1, 3) = -internal_25*internal_53 - internal_49*internal_54;
    Fx(1, 4) = 0.33333333333333331*dt*(-internal_12*internal_85 - internal_40*internal_84) - internal_52*internal_88;
    Fx(2, 2) = 1;
    Fx(2, 3) = internal_51;
    Fx(2, 4) = internal_49*(internal_17*internal_85 + internal_43*internal_84) + internal_50*internal_88;
    Fx(3, 3) = 1;
    Fx(3, 4) = internal_25*(internal_5*internal_81 + internal_89) + 0.66666666666666663*internal_6*internal_73;
    Fx(4, 4) = 1;

    // Evaluation of Matrix Fu
    Fu(0, 0) = 0.16666666666666666*dt*(internal_10*internal_61*kappa_ref*v_ref*(internal_72*internal_91 + internal_78*internal_91) - internal_77*internal_80) - internal_86*internal_92;
    Fu(1, 0) = -internal_12*internal_92 - internal_52*internal_92;
    Fu(2, 0) = internal_17*internal_92 + internal_50*internal_92;
    Fu(3, 0) = 0.33333333333333331*internal_5*internal_75 + internal_87*internal_89;
    Fu(4, 0) = dt;

  }

  virtual void updateFx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, const double t_next, Eigen::MatrixXd& Fx) const override {
    // Determine stage variables
    const auto& x = states(1);
    const auto& y = states(2);
    const auto& theta = states(3);
    const auto& steer = states(4);
    const auto& dsteer = ctrls(0);
    const auto& x_ref = params(0);
    const auto& y_ref = params(1);
    const auto& theta_ref = params(2);
    const auto& kappa_ref = params(3);
    const auto& v_ref = params(4);
    const auto& wheelbase = params(6);

    // Determine global variables
    const auto& kappa = globals(0);
    const auto& l = globals(1);
    const auto& ds = globals(2);

    // Determine internal variables
    const double internal_23 = -theta_ref;
    const double dt = -t + t_next;
    const double internal_20 = x - x_ref;
    const double internal_15 = y - y_ref;
    const double internal_0 = sin(theta_ref);
    const double internal_16 = cos(theta_ref);
    const double internal_55 = kappa_ref*v_ref;
    const double internal_7 = 1.0/wheelbase;
    const double internal_28 = 0.16666666666666666*dt;
    const double internal_30 = 0.5*dt;
    const double internal_49 = 0.33333333333333331*dt;
    const double internal_29 = v_ref*sin(theta);
    const double internal_32 = v_ref*cos(theta);
    const double internal_24 = internal_23 + theta;
    const double internal_5 = dsteer*dt;
    const double internal_56 = pow(dt, 2);
    const double internal_62 = pow(tan(steer), 2) + 1;
    const double internal_1 = internal_0*kappa_ref;
    const double internal_50 = internal_16*kappa_ref;
    const double internal_58 = internal_7*pow(v_ref, 2);
    const double internal_8 = internal_7*v_ref;
    const double internal_31 = internal_29*internal_30;
    const double internal_33 = internal_30*internal_32;
    const double internal_2 = v_ref/(-kappa_ref*l + 1);
    const double internal_35 = internal_30*kappa*v_ref + theta;
    const double internal_4 = internal_1*v_ref;
    const double internal_51 = internal_50*v_ref;
    const double internal_63 = internal_58*internal_62;
    const double internal_9 = dt*internal_8;
    const double internal_6 = tan(0.5*internal_5 + steer);
    const double internal_3 = ds*internal_2;
    const double internal_36 = internal_23 + internal_35;
    const double internal_38 = sin(internal_35);
    const double internal_41 = cos(internal_35);
    const double internal_64 = 0.25*internal_56*internal_63;
    const double internal_65 = internal_30*internal_63;
    const double internal_10 = internal_6*internal_9;
    const double internal_39 = internal_38*v_ref;
    const double internal_42 = internal_41*v_ref;
    const double internal_57 = pow(internal_6, 2) + 1;
    const double internal_11 = 0.5*internal_10 + theta;
    const double internal_25 = internal_10 + internal_24;
    const double internal_67 = internal_10 + theta;
    const double internal_34 = -kappa_ref*(-internal_0*(internal_20 + internal_33) + internal_16*(internal_15 + internal_31)) + 1;
    const double internal_40 = internal_30*internal_39;
    const double internal_43 = internal_30*internal_42;
    const double internal_59 = internal_57*internal_58;
    const double internal_12 = sin(internal_11);
    const double internal_17 = cos(internal_11);
    const double internal_26 = cos(internal_25);
    const double internal_46 = internal_11 + internal_23;
    const double internal_68 = sin(internal_67);
    const double internal_70 = cos(internal_67);
    const double internal_60 = internal_56*internal_59;
    const double internal_66 = internal_30*internal_59;
    const double internal_37 = cos(internal_36)/pow(internal_34, 2);
    const double internal_53 = sin(internal_36)/internal_34;
    const double internal_13 = internal_12*v_ref;
    const double internal_18 = internal_17*v_ref;
    const double internal_47 = cos(internal_46);
    const double internal_61 = 0.5*internal_60;
    const double internal_69 = 0.16666666666666666*internal_60;
    const double internal_14 = dt*internal_13;
    const double internal_19 = dt*internal_18;
    const double internal_44 = -kappa_ref*(-internal_0*(internal_20 + internal_43) + internal_16*(internal_15 + internal_40)) + 1;
    const double internal_45 = pow(internal_44, -2);
    const double internal_54 = sin(internal_46)/internal_44;
    const double internal_48 = internal_45*internal_47;
    const double internal_21 = -kappa_ref*(-internal_0*(internal_19 + internal_20) + internal_16*(internal_14 + internal_15)) + 1;
    const double internal_22 = pow(internal_21, -2);
    const double internal_52 = sin(internal_25)/internal_21;
    const double internal_27 = internal_22*internal_26;

    // Evaluation of Matrix Fx
    Fx(0, 0) = 1;
    Fx(0, 1) = internal_28*(-internal_1*internal_3 - internal_27*internal_4) + internal_49*(-internal_37*internal_4 - internal_4*internal_48);
    Fx(0, 2) = internal_28*(internal_27*internal_51 + internal_3*internal_50) + internal_49*(internal_37*internal_51 + internal_48*internal_51);
    Fx(0, 3) = internal_28*(-internal_2*sin(internal_24) + internal_22*internal_26*kappa_ref*v_ref*(internal_0*internal_14 + internal_16*internal_19) - internal_52*v_ref) + internal_49*(internal_37*internal_55*(internal_0*internal_31 + internal_16*internal_33) + internal_48*internal_55*(internal_0*internal_40 + internal_16*internal_43) - internal_53*v_ref - internal_54*v_ref);
    Fx(0, 4) = internal_28*(-dt*internal_52*internal_59 + internal_22*internal_26*kappa_ref*v_ref*(internal_0*internal_12*internal_61 + internal_16*internal_17*internal_61)) + internal_49*(internal_45*internal_47*kappa_ref*v_ref*(internal_0*internal_38*internal_64 + internal_16*internal_41*internal_64) - internal_53*internal_65 - internal_54*internal_66);
    Fx(1, 1) = 1;
    Fx(1, 3) = internal_28*(-internal_29 - internal_68*v_ref) + internal_49*(-internal_13 - internal_39);
    Fx(1, 4) = 0.33333333333333331*dt*(-internal_12*internal_66 - internal_38*internal_65) - internal_68*internal_69;
    Fx(2, 2) = 1;
    Fx(2, 3) = internal_28*(internal_32 + internal_70*v_ref) + internal_49*(internal_18 + internal_42);
    Fx(2, 4) = internal_49*(internal_17*internal_66 + internal_41*internal_65) + internal_69*internal_70;
    Fx(3, 3) = 1;
    Fx(3, 4) = internal_28*(internal_62*internal_8 + internal_8*(pow(tan(internal_5 + steer), 2) + 1)) + 0.66666666666666663*internal_57*internal_9;
    Fx(4, 4) = 1;

  }

  virtual void updateFu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, const double t_next, Eigen::MatrixXd& Fu) const override {
    // Determine stage variables
    const auto& x = states(1);
    const auto& y = states(2);
    const auto& theta = states(3);
    const auto& steer = states(4);
    const auto& dsteer = ctrls(0);
    const auto& x_ref = params(0);
    const auto& y_ref = params(1);
    const auto& theta_ref = params(2);
    const auto& kappa_ref = params(3);
    const auto& v_ref = params(4);
    const auto& wheelbase = params(6);

    // Determine global variables
    const auto& kappa = globals(0);

    // Determine internal variables
    const double internal_17 = -theta_ref;
    const double dt = -t + t_next;
    const double internal_10 = x - x_ref;
    const double internal_7 = y - y_ref;
    const double internal_11 = sin(theta_ref);
    const double internal_8 = cos(theta_ref);
    const double internal_0 = 1.0/wheelbase;
    const double internal_1 = dsteer*dt;
    const double internal_3 = dt*v_ref;
    const double internal_20 = pow(dt, 2)*internal_0;
    const double internal_21 = 0.5*internal_3;
    const double internal_2 = tan(0.5*internal_1 + steer);
    const double internal_24 = internal_20*v_ref;
    const double internal_22 = internal_21*kappa + theta;
    const double internal_4 = internal_0*internal_2*internal_3;
    const double internal_13 = pow(internal_2, 2) + 1;
    const double internal_18 = internal_4 + theta;
    const double internal_5 = 0.5*internal_4 + theta;
    const double internal_14 = internal_13*pow(v_ref, 2);
    const double internal_19 = internal_17 + internal_18;
    const double internal_6 = sin(internal_5);
    const double internal_9 = cos(internal_5);
    const double internal_15 = pow(dt, 3)*internal_0*internal_14;
    const double internal_16 = 0.25*internal_15;
    const double internal_23 = 0.083333333333333329*internal_15;
    const double internal_12 = -kappa_ref*(-internal_11*(internal_10 + internal_3*internal_9) + internal_8*(internal_3*internal_6 + internal_7)) + 1;

    // Evaluation of Matrix Fu
    Fu(0, 0) = 0.16666666666666666*dt*(-0.5*internal_14*internal_20*sin(internal_19)/internal_12 + kappa_ref*v_ref*(internal_11*internal_16*internal_6 + internal_16*internal_8*internal_9)*cos(internal_19)/pow(internal_12, 2)) - internal_23*sin(internal_17 + internal_5)/(-kappa_ref*(-internal_11*(internal_10 + internal_21*cos(internal_22)) + internal_8*(internal_21*sin(internal_22) + internal_7)) + 1);
    Fu(1, 0) = -internal_23*internal_6 - internal_23*sin(internal_18);
    Fu(2, 0) = internal_23*internal_9 + internal_23*cos(internal_18);
    Fu(3, 0) = 0.33333333333333331*internal_13*internal_24 + 0.16666666666666666*internal_24*(pow(tan(internal_1 + steer), 2) + 1);
    Fu(4, 0) = dt;

  }

};

}  // namespace gpal::pnc::planning