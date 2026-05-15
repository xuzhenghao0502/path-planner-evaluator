#pragma once

#include "ocp/dynamics.h"

namespace gpal::pnc::planning {

class MultiAgentsTrajectoryModelDynamics : public Dynamics {
 public:
  explicit MultiAgentsTrajectoryModelDynamics(OcpConfig::IntegratorType type = OcpConfig::ERK4) : Dynamics(10, 9, type) {}
  virtual ~MultiAgentsTrajectoryModelDynamics() = default;

  virtual Eigen::VectorXd evaluate(const Eigen::VectorXd&, const Eigen::VectorXd&, const double) const override;
  virtual Eigen::MatrixXd evaluateJx(const Eigen::VectorXd&, const Eigen::VectorXd&, const double) const override;
  virtual Eigen::MatrixXd evaluateJu(const Eigen::VectorXd&, const Eigen::VectorXd&, const double) const override;
  virtual void updateEvaluation(const Eigen::VectorXd& x, const Eigen::VectorXd& u, const double, Eigen::VectorXd&) const override;
  virtual void updateJx(const Eigen::VectorXd& x, const Eigen::VectorXd& u, const double, Eigen::MatrixXd&) const override;
  virtual void updateJu(const Eigen::VectorXd& x, const Eigen::VectorXd& u, const double, Eigen::MatrixXd&) const override;
  virtual std::unique_ptr<Integrator> createIntegrator(OcpConfig::IntegratorType type) const override;
};

template <OcpConfig::IntegratorType Ttype>
class MultiAgentsTrajectoryModelIntegrator : public Integrator {
 public:
  MultiAgentsTrajectoryModelIntegrator(const Dynamics* dynamics) : Integrator(dynamics) {}
  virtual ~MultiAgentsTrajectoryModelIntegrator() = default;

  virtual OcpConfig::IntegratorType type() const { return OcpConfig::FORWARD_EULER; }
  virtual void update(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, const double t_next, Eigen::VectorXd& F) const override {
    // Determine stage variables
    const auto& ego_s = states(0);
    const auto& ego_x = states(1);
    const auto& ego_y = states(2);
    const auto& ego_theta = states(3);
    const auto& ego_v = states(4);
    const auto& agent_0_s = states(5);
    const auto& agent_0_x = states(6);
    const auto& agent_0_y = states(7);
    const auto& agent_0_theta = states(8);
    const auto& agent_0_v = states(9);
    const auto& ego_a = ctrls(0);
    const auto& agent_0_a = ctrls(2);
    const auto& agent_0_theta_ref = params(7);

    // Determine global variables
    const auto& ego_kappa = globals(0);
    const auto& ego_ds = globals(6);

    // Determine internal variables
    const double dt = -t + t_next;
    const double internal_0 = dt*ego_v;
    const double internal_1 = agent_0_v*dt;

    // Evaluation of Vector F
    F(0) = ego_ds*internal_0 + ego_s;
    F(1) = ego_x + internal_0*cos(ego_theta);
    F(2) = ego_y + internal_0*sin(ego_theta);
    F(3) = ego_kappa*internal_0 + ego_theta;
    F(4) = dt*ego_a + ego_v;
    F(5) = agent_0_s + internal_1;
    F(6) = agent_0_x + internal_1*cos(agent_0_theta_ref);
    F(7) = agent_0_y + internal_1*sin(agent_0_theta_ref);
    F(8) = agent_0_theta;
    F(9) = agent_0_a*dt + agent_0_v;

  }

  virtual void update(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, const double t_next, Eigen::VectorXd& F, Eigen::MatrixXd& Fx, Eigen::MatrixXd& Fu) const override {
    // Determine stage variables
    const auto& ego_s = states(0);
    const auto& ego_x = states(1);
    const auto& ego_y = states(2);
    const auto& ego_theta = states(3);
    const auto& ego_v = states(4);
    const auto& agent_0_s = states(5);
    const auto& agent_0_x = states(6);
    const auto& agent_0_y = states(7);
    const auto& agent_0_theta = states(8);
    const auto& agent_0_v = states(9);
    const auto& ego_a = ctrls(0);
    const auto& ego_steer = ctrls(1);
    const auto& agent_0_a = ctrls(2);
    const auto& ego_theta_ref = params(2);
    const auto& ego_kappa_ref = params(3);
    const auto& agent_0_theta_ref = params(7);
    const auto& wheelbase = params(17);

    // Determine global variables
    const auto& ego_kappa = globals(0);
    const auto& ego_l = globals(3);
    const auto& ego_ds = globals(6);

    // Determine internal variables
    const double dt = -t + t_next;
    const double internal_8 = cos(agent_0_theta_ref);
    const double internal_9 = sin(agent_0_theta_ref);
    const double internal_0 = dt*ego_ds;
    const double internal_12 = dt*ego_v;
    const double internal_2 = dt*cos(ego_theta);
    const double internal_4 = dt*sin(ego_theta);
    const double internal_6 = dt*ego_kappa;
    const double internal_7 = agent_0_v*dt;
    const double internal_10 = 1.0/(-ego_kappa_ref*ego_l + 1);
    const double internal_1 = ego_v*internal_0;
    const double internal_3 = ego_v*internal_2;
    const double internal_5 = ego_v*internal_4;
    const double internal_11 = ego_kappa_ref*internal_1*internal_10;

    // Evaluation of Vector F
    F(0) = ego_s + internal_1;
    F(1) = ego_x + internal_3;
    F(2) = ego_y + internal_5;
    F(3) = ego_theta + ego_v*internal_6;
    F(4) = dt*ego_a + ego_v;
    F(5) = agent_0_s + internal_7;
    F(6) = agent_0_x + internal_7*internal_8;
    F(7) = agent_0_y + internal_7*internal_9;
    F(8) = agent_0_theta;
    F(9) = agent_0_a*dt + agent_0_v;

    // Evaluation of Matrix Fx
    Fx(0, 0) = 1;
    Fx(0, 1) = -internal_11*sin(ego_theta_ref);
    Fx(0, 2) = internal_11*cos(ego_theta_ref);
    Fx(0, 3) = -internal_10*internal_12*sin(ego_theta - ego_theta_ref);
    Fx(0, 4) = internal_0;
    Fx(1, 1) = 1;
    Fx(1, 3) = -internal_5;
    Fx(1, 4) = internal_2;
    Fx(2, 2) = 1;
    Fx(2, 3) = internal_3;
    Fx(2, 4) = internal_4;
    Fx(3, 3) = 1;
    Fx(3, 4) = internal_6;
    Fx(4, 4) = 1;
    Fx(5, 5) = 1;
    Fx(5, 9) = dt;
    Fx(6, 6) = 1;
    Fx(6, 9) = dt*internal_8;
    Fx(7, 7) = 1;
    Fx(7, 9) = dt*internal_9;
    Fx(8, 8) = 1;
    Fx(9, 9) = 1;

    // Evaluation of Matrix Fu
    Fu(3, 1) = internal_12*(pow(tan(ego_steer), 2) + 1)/wheelbase;
    Fu(4, 0) = dt;
    Fu(9, 2) = dt;

  }

  virtual void updateFx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, const double t_next, Eigen::MatrixXd& Fx) const override {
    // Determine stage variables
    const auto& ego_theta = states(3);
    const auto& ego_v = states(4);
    const auto& ego_theta_ref = params(2);
    const auto& ego_kappa_ref = params(3);
    const auto& agent_0_theta_ref = params(7);

    // Determine global variables
    const auto& ego_kappa = globals(0);
    const auto& ego_l = globals(3);
    const auto& ego_ds = globals(6);

    // Determine internal variables
    const double dt = -t + t_next;
    const double internal_0 = dt*ego_ds;
    const double internal_3 = dt*sin(ego_theta);
    const double internal_4 = dt*cos(ego_theta);
    const double internal_1 = ego_v/(-ego_kappa_ref*ego_l + 1);
    const double internal_2 = ego_kappa_ref*internal_0*internal_1;

    // Evaluation of Matrix Fx
    Fx(0, 0) = 1;
    Fx(0, 1) = -internal_2*sin(ego_theta_ref);
    Fx(0, 2) = internal_2*cos(ego_theta_ref);
    Fx(0, 3) = -dt*internal_1*sin(ego_theta - ego_theta_ref);
    Fx(0, 4) = internal_0;
    Fx(1, 1) = 1;
    Fx(1, 3) = -ego_v*internal_3;
    Fx(1, 4) = internal_4;
    Fx(2, 2) = 1;
    Fx(2, 3) = ego_v*internal_4;
    Fx(2, 4) = internal_3;
    Fx(3, 3) = 1;
    Fx(3, 4) = dt*ego_kappa;
    Fx(4, 4) = 1;
    Fx(5, 5) = 1;
    Fx(5, 9) = dt;
    Fx(6, 6) = 1;
    Fx(6, 9) = dt*cos(agent_0_theta_ref);
    Fx(7, 7) = 1;
    Fx(7, 9) = dt*sin(agent_0_theta_ref);
    Fx(8, 8) = 1;
    Fx(9, 9) = 1;

  }

  virtual void updateFu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, const double t_next, Eigen::MatrixXd& Fu) const override {
    // Determine stage variables
    const auto& ego_v = states(4);
    const auto& ego_steer = ctrls(1);
    const auto& wheelbase = params(17);

    // Determine internal variables
    const double dt = -t + t_next;

    // Evaluation of Matrix Fu
    Fu(3, 1) = dt*ego_v*(pow(tan(ego_steer), 2) + 1)/wheelbase;
    Fu(4, 0) = dt;
    Fu(9, 2) = dt;

  }

};

template <>
class MultiAgentsTrajectoryModelIntegrator<OcpConfig::ERK4> : public Integrator {
 public:
  MultiAgentsTrajectoryModelIntegrator(const Dynamics* dynamics) : Integrator(dynamics) {}
  virtual ~MultiAgentsTrajectoryModelIntegrator() = default;

  virtual OcpConfig::IntegratorType type() const { return OcpConfig::ERK4; }
  virtual void update(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, const double t_next, Eigen::VectorXd& F) const override {
    // Determine stage variables
    const auto& ego_s = states(0);
    const auto& ego_x = states(1);
    const auto& ego_y = states(2);
    const auto& ego_theta = states(3);
    const auto& ego_v = states(4);
    const auto& agent_0_s = states(5);
    const auto& agent_0_x = states(6);
    const auto& agent_0_y = states(7);
    const auto& agent_0_theta = states(8);
    const auto& agent_0_v = states(9);
    const auto& ego_a = ctrls(0);
    const auto& agent_0_a = ctrls(2);
    const auto& ego_x_ref = params(0);
    const auto& ego_y_ref = params(1);
    const auto& ego_theta_ref = params(2);
    const auto& ego_kappa_ref = params(3);
    const auto& agent_0_theta_ref = params(7);

    // Determine global variables
    const auto& ego_kappa = globals(0);
    const auto& ego_ds = globals(6);

    // Determine internal variables
    const double internal_22 = 2*agent_0_v;
    const double internal_12 = -ego_theta_ref;
    const double dt = -t + t_next;
    const double internal_9 = ego_x - ego_x_ref;
    const double internal_6 = ego_y - ego_y_ref;
    const double internal_10 = sin(ego_theta_ref);
    const double internal_7 = cos(ego_theta_ref);
    const double internal_24 = cos(agent_0_theta_ref);
    const double internal_27 = sin(agent_0_theta_ref);
    const double internal_14 = 0.16666666666666666*dt;
    const double internal_16 = 0.5*dt;
    const double internal_21 = 0.33333333333333331*dt;
    const double internal_15 = ego_v*sin(ego_theta);
    const double internal_17 = ego_v*cos(ego_theta);
    const double internal_0 = dt*ego_a;
    const double internal_2 = dt*ego_kappa;
    const double internal_23 = agent_0_a*dt;
    const double internal_3 = 0.5*internal_2;
    const double internal_1 = ego_v + 0.5*internal_0;
    const double internal_11 = ego_v + internal_0;
    const double internal_25 = agent_0_v + internal_23;
    const double internal_26 = 0.66666666666666663*dt*(agent_0_v + 0.5*internal_23);
    const double internal_13 = ego_theta + internal_2*(ego_v + 1.5*internal_0);
    const double internal_4 = ego_theta + internal_3*(ego_v + internal_0);
    const double internal_18 = ego_theta + internal_1*internal_3;
    const double internal_5 = internal_1*sin(internal_4);
    const double internal_8 = internal_1*cos(internal_4);
    const double internal_19 = internal_1*sin(internal_18);
    const double internal_20 = internal_1*cos(internal_18);

    // Evaluation of Vector F
    F(0) = ego_s + internal_14*(ego_ds*ego_v + internal_11*cos(internal_12 + internal_13)/(-ego_kappa_ref*(-internal_10*(dt*internal_8 + internal_9) + internal_7*(dt*internal_5 + internal_6)) + 1)) + internal_21*(internal_1*cos(internal_12 + internal_4)/(-ego_kappa_ref*(-internal_10*(internal_16*internal_20 + internal_9) + internal_7*(internal_16*internal_19 + internal_6)) + 1) + internal_1*cos(internal_12 + internal_18)/(-ego_kappa_ref*(-internal_10*(internal_16*internal_17 + internal_9) + internal_7*(internal_15*internal_16 + internal_6)) + 1));
    F(1) = ego_x + internal_14*(internal_11*cos(internal_13) + internal_17) + internal_21*(internal_20 + internal_8);
    F(2) = ego_y + internal_14*(internal_11*sin(internal_13) + internal_15) + internal_21*(internal_19 + internal_5);
    F(3) = ego_theta + 0.66666666666666663*internal_1*internal_2 + internal_14*(ego_kappa*ego_v + ego_kappa*internal_11);
    F(4) = internal_11;
    F(5) = agent_0_s + internal_14*(internal_22 + internal_23) + internal_21*(internal_22 + internal_23);
    F(6) = agent_0_x + internal_14*(agent_0_v*internal_24 + internal_24*internal_25) + internal_24*internal_26;
    F(7) = agent_0_y + internal_14*(agent_0_v*internal_27 + internal_25*internal_27) + internal_26*internal_27;
    F(8) = agent_0_theta;
    F(9) = internal_25;

  }

  virtual void update(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, const double t_next, Eigen::VectorXd& F, Eigen::MatrixXd& Fx, Eigen::MatrixXd& Fu) const override {
    // Determine stage variables
    const auto& ego_s = states(0);
    const auto& ego_x = states(1);
    const auto& ego_y = states(2);
    const auto& ego_theta = states(3);
    const auto& ego_v = states(4);
    const auto& agent_0_s = states(5);
    const auto& agent_0_x = states(6);
    const auto& agent_0_y = states(7);
    const auto& agent_0_theta = states(8);
    const auto& agent_0_v = states(9);
    const auto& ego_a = ctrls(0);
    const auto& ego_steer = ctrls(1);
    const auto& agent_0_a = ctrls(2);
    const auto& ego_x_ref = params(0);
    const auto& ego_y_ref = params(1);
    const auto& ego_theta_ref = params(2);
    const auto& ego_kappa_ref = params(3);
    const auto& agent_0_theta_ref = params(7);
    const auto& wheelbase = params(17);

    // Determine global variables
    const auto& ego_kappa = globals(0);
    const auto& ego_l = globals(3);
    const auto& ego_ds = globals(6);

    // Determine internal variables
    const double internal_61 = 2*agent_0_v;
    const double internal_20 = -ego_theta_ref;
    const double dt = -t + t_next;
    const double internal_27 = sin(ego_theta);
    const double internal_31 = cos(ego_theta);
    const double internal_16 = ego_x - ego_x_ref;
    const double internal_11 = ego_y - ego_y_ref;
    const double internal_12 = cos(ego_theta_ref);
    const double internal_17 = sin(ego_theta_ref);
    const double internal_63 = cos(agent_0_theta_ref);
    const double internal_67 = sin(agent_0_theta_ref);
    const double internal_0 = ego_ds*ego_v;
    const double internal_26 = 0.16666666666666666*dt;
    const double internal_29 = 0.5*dt;
    const double internal_51 = 0.33333333333333331*dt;
    const double internal_1 = dt*ego_a;
    const double internal_5 = dt*ego_kappa;
    const double internal_62 = agent_0_a*dt;
    const double internal_88 = pow(dt, 2);
    const double internal_28 = ego_v*internal_27;
    const double internal_32 = ego_v*internal_31;
    const double internal_69 = ego_kappa_ref*internal_17;
    const double internal_78 = ego_kappa_ref*internal_12;
    const double internal_65 = dt*internal_63;
    const double internal_68 = dt*internal_67;
    const double internal_70 = 1.0/(-ego_kappa_ref*ego_l + 1);
    const double internal_101 = pow(dt, 3)*ego_kappa;
    const double internal_103 = 1.5*internal_88;
    const double internal_6 = 0.5*internal_5;
    const double internal_89 = 0.5*internal_88;
    const double internal_96 = 0.25*internal_88;
    const double internal_106 = (pow(tan(ego_steer), 2) + 1)/wheelbase;
    const double internal_102 = 0.5*internal_101;
    const double internal_104 = 0.125*internal_101;
    const double internal_2 = ego_v + internal_1;
    const double internal_21 = ego_v + 1.5*internal_1;
    const double internal_3 = ego_v + 0.5*internal_1;
    const double internal_4 = ego_v + internal_1;
    const double internal_64 = agent_0_v + internal_62;
    const double internal_66 = 0.66666666666666663*agent_0_v + 0.33333333333333331*internal_62;
    const double internal_30 = internal_28*internal_29;
    const double internal_33 = internal_29*internal_32;
    const double internal_71 = internal_0*internal_70;
    const double internal_111 = ego_kappa*internal_103;
    const double internal_60 = 0.66666666666666663*internal_3;
    const double internal_90 = ego_kappa*internal_89;
    const double internal_97 = ego_kappa*internal_96;
    const double internal_109 = dt*internal_106;
    const double internal_107 = internal_106*internal_4;
    const double internal_113 = 0.16666666666666666*internal_106*internal_21*internal_88;
    const double internal_59 = ego_kappa*internal_2;
    const double internal_74 = internal_3*internal_69;
    const double internal_79 = internal_3*internal_78;
    const double internal_85 = ego_kappa_ref*internal_3;
    const double internal_110 = internal_106*pow(internal_3, 2);
    const double internal_112 = 0.5*internal_107;
    const double internal_22 = ego_theta + internal_21*internal_5;
    const double internal_36 = ego_theta + internal_3*internal_6;
    const double internal_7 = ego_theta + internal_4*internal_6;
    const double internal_108 = internal_107*internal_89;
    const double internal_13 = cos(internal_7);
    const double internal_23 = internal_20 + internal_22;
    const double internal_37 = internal_20 + internal_36;
    const double internal_40 = sin(internal_36);
    const double internal_43 = cos(internal_36);
    const double internal_48 = internal_20 + internal_7;
    const double internal_52 = cos(internal_22);
    const double internal_55 = sin(internal_22);
    const double internal_8 = sin(internal_7);
    const double internal_34 = -ego_kappa_ref*(internal_12*(internal_11 + internal_30) - internal_17*(internal_16 + internal_33)) + 1;
    const double internal_105 = internal_40*internal_96;
    const double internal_14 = internal_13*internal_3;
    const double internal_24 = cos(internal_23);
    const double internal_38 = cos(internal_37);
    const double internal_41 = internal_3*internal_40;
    const double internal_44 = internal_3*internal_43;
    const double internal_49 = cos(internal_48);
    const double internal_53 = internal_2*internal_52;
    const double internal_56 = internal_2*internal_55;
    const double internal_9 = internal_3*internal_8;
    const double internal_92 = dt*internal_8;
    const double internal_99 = internal_29*internal_40;
    const double internal_35 = 1.0/internal_34;
    const double internal_10 = dt*internal_9;
    const double internal_15 = dt*internal_14;
    const double internal_42 = internal_29*internal_41;
    const double internal_45 = internal_29*internal_44;
    const double internal_57 = internal_28 + internal_56;
    const double internal_58 = internal_41 + internal_9;
    const double internal_91 = internal_9*internal_90;
    const double internal_93 = internal_14*internal_90;
    const double internal_75 = internal_38/pow(internal_34, 2);
    const double internal_100 = internal_44*internal_97 + internal_99;
    const double internal_98 = -0.5*dt*internal_43 + internal_41*internal_97;
    const double internal_39 = internal_35*internal_38;
    const double internal_82 = internal_35*sin(internal_37);
    const double internal_54 = internal_26*(internal_32 + internal_53) + internal_51*(internal_14 + internal_44);
    const double internal_86 = internal_75*internal_85;
    const double internal_83 = internal_3*internal_82;
    const double internal_18 = -ego_kappa_ref*(internal_12*(internal_10 + internal_11) - internal_17*(internal_15 + internal_16)) + 1;
    const double internal_46 = -ego_kappa_ref*(internal_12*(internal_11 + internal_42) - internal_17*(internal_16 + internal_45)) + 1;
    const double internal_19 = 1.0/internal_18;
    const double internal_47 = 1.0/internal_46;
    const double internal_72 = pow(internal_18, -2);
    const double internal_76 = pow(internal_46, -2);
    const double internal_25 = internal_19*internal_24;
    const double internal_50 = internal_47*internal_49;
    const double internal_73 = internal_2*internal_24*internal_72;
    const double internal_77 = internal_49*internal_76;
    const double internal_80 = internal_19*sin(internal_23);
    const double internal_84 = internal_3*internal_47*sin(internal_48);
    const double internal_81 = internal_2*internal_80;
    const double internal_87 = internal_77*internal_85;
    const double internal_94 = ego_kappa_ref*internal_73;
    const double internal_95 = internal_59*internal_80;

    // Evaluation of Vector F
    F(0) = ego_s + internal_26*(internal_0 + internal_2*internal_25) + internal_51*(internal_3*internal_39 + internal_3*internal_50);
    F(1) = ego_x + internal_54;
    F(2) = ego_y + internal_26*internal_57 + internal_51*internal_58;
    F(3) = ego_theta + internal_26*(ego_kappa*ego_v + internal_59) + internal_5*internal_60;
    F(4) = internal_2;
    F(5) = agent_0_s + internal_26*(internal_61 + internal_62) + internal_51*(internal_61 + internal_62);
    F(6) = agent_0_x + internal_26*(agent_0_v*internal_63 + internal_63*internal_64) + internal_65*internal_66;
    F(7) = agent_0_y + internal_26*(agent_0_v*internal_67 + internal_64*internal_67) + internal_66*internal_68;
    F(8) = agent_0_theta;
    F(9) = internal_64;

    // Evaluation of Matrix Fx
    Fx(0, 0) = 1;
    Fx(0, 1) = internal_26*(-internal_69*internal_71 - internal_69*internal_73) + internal_51*(-internal_74*internal_75 - internal_74*internal_77);
    Fx(0, 2) = internal_26*(internal_71*internal_78 + internal_73*internal_78) + internal_51*(internal_75*internal_79 + internal_77*internal_79);
    Fx(0, 3) = internal_26*(ego_kappa_ref*internal_2*internal_24*internal_72*(internal_10*internal_17 + internal_12*internal_15) - ego_v*internal_70*sin(ego_theta + internal_20) - internal_81) + internal_51*(-internal_83 - internal_84 + internal_86*(internal_12*internal_33 + internal_17*internal_30) + internal_87*(internal_12*internal_45 + internal_17*internal_42));
    Fx(0, 4) = internal_26*(-dt*internal_95 + ego_ds + internal_25 + internal_94*(internal_12*(internal_92 + internal_93) - internal_17*(dt*internal_13 - internal_91))) + internal_51*(internal_39 + internal_50 - internal_6*internal_83 - internal_6*internal_84 + internal_86*(internal_12*internal_27*internal_29 - internal_17*internal_29*internal_31) + internal_87*(internal_100*internal_12 + internal_17*internal_98));
    Fx(1, 1) = 1;
    Fx(1, 3) = -internal_26*internal_57 - internal_51*internal_58;
    Fx(1, 4) = internal_26*(internal_31 - internal_5*internal_56 + internal_52) + internal_51*(internal_13 - internal_41*internal_6 + internal_43 - internal_6*internal_9);
    Fx(2, 2) = 1;
    Fx(2, 3) = internal_54;
    Fx(2, 4) = internal_26*(internal_27 + internal_5*internal_53 + internal_55) + internal_51*(internal_14*internal_6 + internal_40 + internal_44*internal_6 + internal_8);
    Fx(3, 3) = 1;
    Fx(3, 4) = internal_5;
    Fx(4, 4) = 1;
    Fx(5, 5) = 1;
    Fx(5, 9) = dt;
    Fx(6, 6) = 1;
    Fx(6, 9) = internal_65;
    Fx(7, 7) = 1;
    Fx(7, 9) = internal_68;
    Fx(8, 8) = 1;
    Fx(9, 9) = 1;

    // Evaluation of Matrix Fu
    Fu(0, 0) = internal_26*(dt*internal_25 - internal_103*internal_95 + internal_94*(internal_12*(internal_102*internal_14 + internal_8*internal_89) - internal_17*(-internal_102*internal_9 + 0.5*internal_13*internal_88))) + internal_51*(internal_29*internal_39 + internal_29*internal_50 - internal_83*internal_97 - internal_84*internal_90 + internal_87*(internal_12*(internal_104*internal_44 + internal_105) - internal_17*(-internal_104*internal_41 + 0.25*internal_43*internal_88)));
    Fu(0, 1) = internal_26*(ego_kappa_ref*internal_2*internal_24*internal_72*(internal_108*internal_12*internal_14 + internal_108*internal_17*internal_9) - internal_109*internal_21*internal_81) + internal_51*(ego_kappa_ref*internal_3*internal_49*internal_76*(internal_105*internal_110*internal_17 + internal_110*internal_12*internal_43*internal_96) - internal_107*internal_29*internal_84 - internal_110*internal_29*internal_82);
    Fu(1, 0) = internal_26*(dt*internal_52 - internal_111*internal_56) + internal_51*(0.5*dt*internal_13 - internal_91 - internal_98);
    Fu(1, 1) = 0.33333333333333331*dt*(-internal_10*internal_112 - internal_110*internal_99) - internal_113*internal_56;
    Fu(2, 0) = internal_26*(dt*internal_55 + internal_111*internal_53) + internal_51*(internal_100 + 0.5*internal_92 + internal_93);
    Fu(2, 1) = internal_113*internal_53 + internal_51*(internal_110*internal_29*internal_43 + internal_112*internal_15);
    Fu(3, 0) = internal_90;
    Fu(3, 1) = internal_109*internal_60 + internal_26*(ego_v*internal_106 + internal_106*internal_2);
    Fu(4, 0) = dt;
    Fu(5, 2) = internal_89;
    Fu(6, 2) = internal_63*internal_89;
    Fu(7, 2) = internal_67*internal_89;
    Fu(9, 2) = dt;

  }

  virtual void updateFx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, const double t_next, Eigen::MatrixXd& Fx) const override {
    // Determine stage variables
    const auto& ego_x = states(1);
    const auto& ego_y = states(2);
    const auto& ego_theta = states(3);
    const auto& ego_v = states(4);
    const auto& ego_a = ctrls(0);
    const auto& ego_x_ref = params(0);
    const auto& ego_y_ref = params(1);
    const auto& ego_theta_ref = params(2);
    const auto& ego_kappa_ref = params(3);
    const auto& agent_0_theta_ref = params(7);

    // Determine global variables
    const auto& ego_kappa = globals(0);
    const auto& ego_l = globals(3);
    const auto& ego_ds = globals(6);

    // Determine internal variables
    const double internal_7 = -ego_theta_ref;
    const double internal_71 = 0.5*ego_kappa;
    const double dt = -t + t_next;
    const double internal_31 = sin(ego_theta);
    const double internal_35 = cos(ego_theta);
    const double internal_22 = ego_x - ego_x_ref;
    const double internal_18 = ego_y - ego_y_ref;
    const double internal_0 = sin(ego_theta_ref);
    const double internal_19 = cos(ego_theta_ref);
    const double internal_26 = 0.16666666666666666*dt;
    const double internal_33 = 0.5*dt;
    const double internal_50 = 0.33333333333333331*dt;
    const double internal_8 = ego_theta + internal_7;
    const double internal_4 = dt*ego_a;
    const double internal_5 = dt*ego_kappa;
    const double internal_32 = ego_v*internal_31;
    const double internal_36 = ego_v*internal_35;
    const double internal_1 = ego_kappa_ref*internal_0;
    const double internal_51 = ego_kappa_ref*internal_19;
    const double internal_13 = 0.5*internal_5;
    const double internal_63 = pow(dt, 2)*ego_kappa;
    const double internal_2 = ego_v/(-ego_kappa_ref*ego_l + 1);
    const double internal_11 = ego_v + internal_4;
    const double internal_12 = ego_v + 0.5*internal_4;
    const double internal_64 = 0.5*internal_63;
    const double internal_66 = 0.25*internal_63;
    const double internal_34 = internal_32*internal_33;
    const double internal_37 = internal_33*internal_36;
    const double internal_6 = internal_5*(ego_v + 1.5*internal_4);
    const double internal_3 = ego_ds*internal_2;
    const double internal_14 = ego_theta + internal_13*(ego_v + internal_4);
    const double internal_27 = internal_1*internal_12;
    const double internal_52 = internal_12*internal_51;
    const double internal_59 = ego_kappa_ref*internal_12;
    const double internal_28 = ego_theta + internal_12*internal_13;
    const double internal_67 = ego_theta + internal_6;
    const double internal_9 = internal_6 + internal_8;
    const double internal_15 = sin(internal_14);
    const double internal_20 = cos(internal_14);
    const double internal_40 = internal_14 + internal_7;
    const double internal_10 = cos(internal_9);
    const double internal_29 = internal_28 + internal_7;
    const double internal_42 = sin(internal_28);
    const double internal_45 = cos(internal_28);
    const double internal_68 = sin(internal_67);
    const double internal_70 = cos(internal_67);
    const double internal_16 = dt*internal_15;
    const double internal_21 = dt*internal_12*internal_20;
    const double internal_41 = cos(internal_40);
    const double internal_62 = internal_12*internal_15;
    const double internal_65 = internal_12*internal_20;
    const double internal_38 = -ego_kappa_ref*(-internal_0*(internal_22 + internal_37) + internal_19*(internal_18 + internal_34)) + 1;
    const double internal_30 = cos(internal_29);
    const double internal_43 = internal_12*internal_42;
    const double internal_46 = internal_12*internal_45;
    const double internal_69 = internal_11*internal_68;
    const double internal_72 = internal_11*internal_70;
    const double internal_17 = internal_12*internal_16;
    const double internal_55 = 1.0/internal_38;
    const double internal_44 = internal_33*internal_43;
    const double internal_47 = internal_33*internal_46;
    const double internal_39 = internal_30/pow(internal_38, 2);
    const double internal_56 = internal_12*internal_55*sin(internal_29);
    const double internal_60 = internal_39*internal_59;
    const double internal_23 = -ego_kappa_ref*(-internal_0*(internal_21 + internal_22) + internal_19*(internal_17 + internal_18)) + 1;
    const double internal_48 = -ego_kappa_ref*(-internal_0*(internal_22 + internal_47) + internal_19*(internal_18 + internal_44)) + 1;
    const double internal_24 = pow(internal_23, -2);
    const double internal_53 = 1.0/internal_23;
    const double internal_57 = 1.0/internal_48;
    const double internal_25 = internal_10*internal_11*internal_24;
    const double internal_49 = internal_41/pow(internal_48, 2);
    const double internal_54 = internal_11*internal_53*sin(internal_9);
    const double internal_58 = internal_12*internal_57*sin(internal_40);
    const double internal_61 = internal_49*internal_59;

    // Evaluation of Matrix Fx
    Fx(0, 0) = 1;
    Fx(0, 1) = internal_26*(-internal_1*internal_25 - internal_1*internal_3) + internal_50*(-internal_27*internal_39 - internal_27*internal_49);
    Fx(0, 2) = internal_26*(internal_25*internal_51 + internal_3*internal_51) + internal_50*(internal_39*internal_52 + internal_49*internal_52);
    Fx(0, 3) = internal_26*(ego_kappa_ref*internal_10*internal_11*internal_24*(internal_0*internal_17 + internal_19*internal_21) - internal_2*sin(internal_8) - internal_54) + internal_50*(-internal_56 - internal_58 + internal_60*(internal_0*internal_34 + internal_19*internal_37) + internal_61*(internal_0*internal_44 + internal_19*internal_47));
    Fx(0, 4) = internal_26*(ego_ds + ego_kappa_ref*internal_25*(-internal_0*(dt*internal_20 - internal_62*internal_64) + internal_19*(internal_16 + internal_64*internal_65)) + internal_10*internal_53 - internal_5*internal_54) + internal_50*(-internal_13*internal_56 - internal_13*internal_58 + internal_30*internal_55 + internal_41*internal_57 + internal_60*(-internal_0*internal_33*internal_35 + internal_19*internal_31*internal_33) + internal_61*(-internal_0*(0.5*dt*internal_45 - internal_43*internal_66) + internal_19*(internal_33*internal_42 + internal_46*internal_66)));
    Fx(1, 1) = 1;
    Fx(1, 3) = internal_26*(-internal_32 - internal_69) + internal_50*(-internal_43 - internal_62);
    Fx(1, 4) = internal_26*(internal_35 - internal_5*internal_69 + internal_70) + internal_50*(-internal_13*internal_43 - internal_17*internal_71 + internal_20 + internal_45);
    Fx(2, 2) = 1;
    Fx(2, 3) = internal_26*(internal_36 + internal_72) + internal_50*(internal_46 + internal_65);
    Fx(2, 4) = internal_26*(internal_31 + internal_5*internal_72 + internal_68) + internal_50*(internal_13*internal_46 + internal_15 + internal_21*internal_71 + internal_42);
    Fx(3, 3) = 1;
    Fx(3, 4) = internal_5;
    Fx(4, 4) = 1;
    Fx(5, 5) = 1;
    Fx(5, 9) = dt;
    Fx(6, 6) = 1;
    Fx(6, 9) = dt*cos(agent_0_theta_ref);
    Fx(7, 7) = 1;
    Fx(7, 9) = dt*sin(agent_0_theta_ref);
    Fx(8, 8) = 1;
    Fx(9, 9) = 1;

  }

  virtual void updateFu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double t, const double t_next, Eigen::MatrixXd& Fu) const override {
    // Determine stage variables
    const auto& ego_x = states(1);
    const auto& ego_y = states(2);
    const auto& ego_theta = states(3);
    const auto& ego_v = states(4);
    const auto& ego_a = ctrls(0);
    const auto& ego_steer = ctrls(1);
    const auto& ego_x_ref = params(0);
    const auto& ego_y_ref = params(1);
    const auto& ego_theta_ref = params(2);
    const auto& ego_kappa_ref = params(3);
    const auto& agent_0_theta_ref = params(7);
    const auto& wheelbase = params(17);

    // Determine global variables
    const auto& ego_kappa = globals(0);

    // Determine internal variables
    const double internal_0 = -ego_theta_ref;
    const double dt = -t + t_next;
    const double internal_18 = ego_x - ego_x_ref;
    const double internal_14 = ego_y - ego_y_ref;
    const double internal_15 = cos(ego_theta_ref);
    const double internal_19 = sin(ego_theta_ref);
    const double internal_34 = 0.16666666666666666*dt;
    const double internal_38 = 0.5*dt;
    const double internal_55 = 0.33333333333333331*dt;
    const double internal_1 = dt*ego_a;
    const double internal_23 = pow(dt, 2);
    const double internal_3 = dt*ego_kappa;
    const double internal_27 = 0.5*internal_23;
    const double internal_39 = ego_v*internal_38;
    const double internal_49 = 0.25*internal_23;
    const double internal_57 = (pow(tan(ego_steer), 2) + 1)/wheelbase;
    const double internal_11 = ego_v + 0.5*internal_1;
    const double internal_2 = ego_v + 1.5*internal_1;
    const double internal_29 = ego_v + internal_1;
    const double internal_7 = ego_v + internal_1;
    const double internal_53 = ego_kappa*internal_27;
    const double internal_8 = 0.5*internal_7;
    const double internal_12 = dt*internal_11;
    const double internal_24 = ego_kappa*internal_11;
    const double internal_32 = 1.5*ego_kappa*internal_29;
    const double internal_58 = internal_11*internal_57*internal_7;
    const double internal_59 = internal_29*internal_57;
    const double internal_35 = 0.5*internal_12;
    const double internal_4 = ego_theta + internal_2*internal_3;
    const double internal_62 = pow(internal_11, 2)*internal_57;
    const double internal_68 = internal_57*internal_8;
    const double internal_25 = pow(dt, 3)*internal_24;
    const double internal_33 = internal_23*internal_32;
    const double internal_60 = internal_2*internal_59;
    const double internal_63 = internal_12*internal_57;
    const double internal_9 = ego_theta + internal_3*internal_8;
    const double internal_26 = 0.5*internal_25;
    const double internal_48 = 0.125*internal_25;
    const double internal_5 = internal_0 + internal_4;
    const double internal_64 = cos(internal_4);
    const double internal_65 = sin(internal_4);
    const double internal_69 = 0.16666666666666666*internal_60;
    const double internal_10 = sin(internal_9);
    const double internal_16 = cos(internal_9);
    const double internal_36 = ego_kappa*internal_35 + ego_theta;
    const double internal_46 = internal_0 + internal_9;
    const double internal_31 = sin(internal_5);
    const double internal_40 = 1.0/(-ego_kappa_ref*(internal_15*(internal_14 + internal_39*sin(ego_theta)) - internal_19*(internal_18 + internal_39*cos(ego_theta))) + 1);
    const double internal_6 = cos(internal_5);
    const double internal_66 = internal_23*internal_65;
    const double internal_13 = internal_10*internal_12;
    const double internal_17 = internal_12*internal_16;
    const double internal_28 = internal_10*internal_27;
    const double internal_37 = internal_0 + internal_36;
    const double internal_42 = sin(internal_36);
    const double internal_43 = cos(internal_36);
    const double internal_47 = cos(internal_46);
    const double internal_56 = internal_16*internal_27;
    const double internal_41 = internal_38*internal_40;
    const double internal_50 = internal_42*internal_49;
    const double internal_52 = sin(internal_37);
    const double internal_61 = internal_43*internal_49;
    const double internal_67 = internal_38*internal_42;
    const double internal_20 = -ego_kappa_ref*(internal_15*(internal_13 + internal_14) - internal_19*(internal_17 + internal_18)) + 1;
    const double internal_44 = -ego_kappa_ref*(internal_15*(internal_14 + internal_35*internal_42) - internal_19*(internal_18 + internal_35*internal_43)) + 1;
    const double internal_21 = 1.0/internal_20;
    const double internal_30 = pow(internal_20, -2);
    const double internal_45 = 1.0/internal_44;
    const double internal_51 = pow(internal_44, -2);
    const double internal_22 = dt*internal_21;
    const double internal_54 = internal_45*sin(internal_46);

    // Evaluation of Matrix Fu
    Fu(0, 0) = internal_34*(ego_kappa_ref*internal_29*internal_30*internal_6*(internal_15*(internal_16*internal_26 + internal_28) - internal_19*(-internal_10*internal_26 + 0.5*internal_16*internal_23)) - internal_21*internal_31*internal_33 + internal_22*internal_6) + internal_55*(ego_kappa_ref*internal_11*internal_47*internal_51*(internal_15*(internal_43*internal_48 + internal_50) - internal_19*(0.25*internal_23*internal_43 - internal_42*internal_48)) - internal_11*internal_53*internal_54 - internal_24*internal_40*internal_49*internal_52 + internal_38*internal_45*internal_47 + internal_41*cos(internal_37));
    Fu(0, 1) = internal_34*(ego_kappa_ref*internal_29*internal_30*internal_6*(internal_15*internal_56*internal_58 + internal_19*internal_28*internal_58) - internal_22*internal_31*internal_60) + internal_55*(ego_kappa_ref*internal_11*internal_47*internal_51*(internal_15*internal_61*internal_62 + internal_19*internal_50*internal_62) - internal_41*internal_52*internal_62 - internal_54*internal_63*internal_8);
    Fu(1, 0) = internal_34*(dt*internal_64 - internal_32*internal_66) + internal_55*(0.5*dt*internal_16 + 0.5*dt*internal_43 - internal_24*internal_28 - internal_24*internal_50);
    Fu(1, 1) = 0.33333333333333331*dt*(-internal_13*internal_68 - internal_62*internal_67) - internal_66*internal_69;
    Fu(2, 0) = internal_34*(dt*internal_65 + internal_33*internal_64) + internal_55*(internal_10*internal_38 + internal_24*internal_56 + internal_24*internal_61 + internal_67);
    Fu(2, 1) = internal_23*internal_64*internal_69 + internal_55*(internal_17*internal_68 + internal_38*internal_43*internal_62);
    Fu(3, 0) = internal_53;
    Fu(3, 1) = internal_34*(ego_v*internal_57 + internal_59) + 0.66666666666666663*internal_63;
    Fu(4, 0) = dt;
    Fu(5, 2) = internal_27;
    Fu(6, 2) = internal_27*cos(agent_0_theta_ref);
    Fu(7, 2) = internal_27*sin(agent_0_theta_ref);
    Fu(9, 2) = dt;

  }

};

}  // namespace gpal::pnc::planning