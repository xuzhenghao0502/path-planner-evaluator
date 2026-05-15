#pragma once

#include "ocp/constraints.h"

namespace gpal::pnc::planning {

template <StageType Ttype>
class MultiAgentsTrajectoryModelConstraints : public Constraints {
 public:
  MultiAgentsTrajectoryModelConstraints() : Constraints(10, 9, 10, 12, 6) {
    // Evaluation of Matrix jacobian_.jx
    jacobian_.jx(2, 0) = -1;
    jacobian_.jx(3, 0) = 1;
    jacobian_.jx(4, 5) = -1;
    jacobian_.jx(5, 4) = -1;
    jacobian_.jx(6, 4) = 1;
    jacobian_.jx(7, 9) = -1;

    // Evaluation of Matrix jacobian_.ju
    jacobian_.ju(10, 3) = -1;
    jacobian_.ju(11, 4) = -1;
    jacobian_.ju(12, 5) = -1;
    jacobian_.ju(13, 6) = -1;
    jacobian_.ju(14, 7) = -1;
    jacobian_.ju(15, 8) = -1;
    jacobian_.ju(16, 1) = -1;
    jacobian_.ju(17, 1) = 1;
    jacobian_.ju(18, 0) = -1;
    jacobian_.ju(19, 0) = 1;
    jacobian_.ju(20, 2) = -1;
    jacobian_.ju(21, 2) = 1;
    jacobian_.ju(22, 3) = -1;
    jacobian_.ju(23, 4) = -1;
    jacobian_.ju(24, 5) = -1;
    jacobian_.ju(25, 7) = -1;
    jacobian_.ju(26, 6) = -1;
    jacobian_.ju(27, 8) = -1;
    exprs_[0] = "LHardLowerBound <= -(ego_x - ego_x_ref)*sin(ego_theta_ref) + (ego_y - ego_y_ref)*cos(ego_theta_ref)";
    exprs_[1] = "-(ego_x - ego_x_ref)*sin(ego_theta_ref) + (ego_y - ego_y_ref)*cos(ego_theta_ref) <= LHardUpperBound";
    exprs_[2] = "SHardLowerBound <= ego_s";
    exprs_[3] = "ego_s <= SHardUpperBound";
    exprs_[4] = "agent_0_s >= 0.0";
    exprs_[5] = "VHardLowerBound <= ego_v";
    exprs_[6] = "ego_v <= VHardUpperBound";
    exprs_[7] = "agent_0_v >= 0.0";
    exprs_[8] = "-ego_kappa_ref*(-(ego_x - ego_x_ref)*sin(ego_theta_ref) + (ego_y - ego_y_ref)*cos(ego_theta_ref)) + 1 >= 0.0001";
    exprs_[9] = "cos(ego_theta - ego_theta_ref) >= 0.0";
    exprs_[10] = "ego_f_2_agent_0_slack >= 0.0";
    exprs_[11] = "ego_r_2_agent_0_slack >= 0.0";
    exprs_[12] = "ego_f_2_object_0_slack >= 0.0";
    exprs_[13] = "ego_r_2_object_0_slack >= 0.0";
    exprs_[14] = "ego_f_2_object_1_slack >= 0.0";
    exprs_[15] = "ego_r_2_object_1_slack >= 0.0";
    exprs_[16] = "SteerLowerBound <= ego_steer";
    exprs_[17] = "ego_steer <= SteerUpperBound";
    exprs_[18] = "AHardLowerBound <= ego_a";
    exprs_[19] = "ego_a <= AHardUpperBound";
    exprs_[20] = "Agent_0_AHardLowerBound <= agent_0_a";
    exprs_[21] = "agent_0_a <= Agent_0_AHardUpperBound";
    exprs_[22] = "Agent_0_AvoidedBound <= ego_f_2_agent_0_slack + ((agent_0_x - ego_x - (front_overhang + wheelbase)*cos(ego_theta))*sin(agent_0_theta_ref) + (-agent_0_y + ego_y + (front_overhang + wheelbase)*sin(ego_theta))*cos(agent_0_theta_ref))**2/agent_b_0_square + ((-agent_0_x + ego_x + (front_overhang + wheelbase)*cos(ego_theta))*cos(agent_0_theta_ref) + (-agent_0_y + ego_y + (front_overhang + wheelbase)*sin(ego_theta))*sin(agent_0_theta_ref))**2/agent_a_0_square";
    exprs_[23] = "Agent_0_AvoidedBound <= ego_r_2_agent_0_slack + ((agent_0_x - ego_x + rear_overhang*cos(ego_theta))*sin(agent_0_theta_ref) + (-agent_0_y + ego_y - rear_overhang*sin(ego_theta))*cos(agent_0_theta_ref))**2/agent_b_0_square + ((-agent_0_x + ego_x - rear_overhang*cos(ego_theta))*cos(agent_0_theta_ref) + (-agent_0_y + ego_y - rear_overhang*sin(ego_theta))*sin(agent_0_theta_ref))**2/agent_a_0_square";
    exprs_[24] = "Obs_0_AvoidedBound <= ego_f_2_object_0_slack + (object_cos_theta_0*(ego_y - object_y_0 + (front_overhang + wheelbase)*sin(ego_theta)) + object_sin_theta_0*(-ego_x + object_x_0 - (front_overhang + wheelbase)*cos(ego_theta)))**2/object_b_0_square + (object_cos_theta_0*(ego_x - object_x_0 + (front_overhang + wheelbase)*cos(ego_theta)) + object_sin_theta_0*(ego_y - object_y_0 + (front_overhang + wheelbase)*sin(ego_theta)))**2/object_a_0_square";
    exprs_[25] = "Obs_1_AvoidedBound <= ego_f_2_object_1_slack + (object_cos_theta_1*(ego_y - object_y_1 + (front_overhang + wheelbase)*sin(ego_theta)) + object_sin_theta_1*(-ego_x + object_x_1 - (front_overhang + wheelbase)*cos(ego_theta)))**2/object_b_1_square + (object_cos_theta_1*(ego_x - object_x_1 + (front_overhang + wheelbase)*cos(ego_theta)) + object_sin_theta_1*(ego_y - object_y_1 + (front_overhang + wheelbase)*sin(ego_theta)))**2/object_a_1_square";
    exprs_[26] = "Obs_0_AvoidedBound <= ego_r_2_object_0_slack + (object_cos_theta_0*(ego_y - object_y_0 - rear_overhang*sin(ego_theta)) + object_sin_theta_0*(-ego_x + object_x_0 + rear_overhang*cos(ego_theta)))**2/object_b_0_square + (object_cos_theta_0*(ego_x - object_x_0 - rear_overhang*cos(ego_theta)) + object_sin_theta_0*(ego_y - object_y_0 - rear_overhang*sin(ego_theta)))**2/object_a_0_square";
    exprs_[27] = "Obs_1_AvoidedBound <= ego_r_2_object_1_slack + (object_cos_theta_1*(ego_y - object_y_1 - rear_overhang*sin(ego_theta)) + object_sin_theta_1*(-ego_x + object_x_1 + rear_overhang*cos(ego_theta)))**2/object_b_1_square + (object_cos_theta_1*(ego_x - object_x_1 - rear_overhang*cos(ego_theta)) + object_sin_theta_1*(ego_y - object_y_1 - rear_overhang*sin(ego_theta)))**2/object_a_1_square";

    idx_with_slack_.emplace(10);
    idx_with_slack_.emplace(11);
    idx_with_slack_.emplace(12);
    idx_with_slack_.emplace(13);
    idx_with_slack_.emplace(14);
    idx_with_slack_.emplace(15);
    idx_with_slack_.emplace(22);
    idx_with_slack_.emplace(23);
    idx_with_slack_.emplace(24);
    idx_with_slack_.emplace(25);
    idx_with_slack_.emplace(26);
    idx_with_slack_.emplace(27);
  }

  virtual ~MultiAgentsTrajectoryModelConstraints() = default;

  virtual Type type() const override { return Type::INEQUALITY; }
  virtual void updateEvaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::VectorXd& values) const override {
    // Determine stage variables
    const auto& ego_s = states(0);
    const auto& ego_theta = states(3);
    const auto& ego_v = states(4);
    const auto& agent_0_s = states(5);
    const auto& agent_0_v = states(9);
    const auto& ego_a = ctrls(0);
    const auto& ego_steer = ctrls(1);
    const auto& agent_0_a = ctrls(2);
    const auto& ego_f_2_agent_0_slack = ctrls(3);
    const auto& ego_r_2_agent_0_slack = ctrls(4);
    const auto& ego_f_2_object_0_slack = ctrls(5);
    const auto& ego_r_2_object_0_slack = ctrls(6);
    const auto& ego_f_2_object_1_slack = ctrls(7);
    const auto& ego_r_2_object_1_slack = ctrls(8);
    const auto& ego_theta_ref = params(2);
    const auto& ego_kappa_ref = params(3);
    const auto& Agent_0_AvoidedBound = params(34);
    const auto& Obs_0_AvoidedBound = params(35);
    const auto& Obs_1_AvoidedBound = params(36);
    const auto& LHardLowerBound = params(37);
    const auto& LHardUpperBound = params(38);
    const auto& SteerLowerBound = params(39);
    const auto& SteerUpperBound = params(40);
    const auto& SHardLowerBound = params(41);
    const auto& SHardUpperBound = params(42);
    const auto& VHardLowerBound = params(43);
    const auto& VHardUpperBound = params(44);
    const auto& AHardLowerBound = params(45);
    const auto& AHardUpperBound = params(46);
    const auto& Agent_0_AHardLowerBound = params(47);
    const auto& Agent_0_AHardUpperBound = params(48);

    // Determine global variables
    const auto& ego_l = globals(3);
    const auto& r_dis_2_agent_0 = globals(7);
    const auto& r_dis_2_object_0 = globals(8);
    const auto& r_dis_2_object_1 = globals(9);
    const auto& f_dis_2_agent_0 = globals(10);
    const auto& f_dis_2_object_0 = globals(11);
    const auto& f_dis_2_object_1 = globals(12);

    // Determine internal variables
    const double internal_1 = -ego_s;
    const double internal_2 = -ego_v;
    const double internal_4 = -ego_a;
    const double internal_3 = -ego_steer;
    const double internal_5 = -agent_0_a;
    const double internal_6 = -Agent_0_AvoidedBound;
    const double internal_7 = -Obs_0_AvoidedBound;
    const double internal_8 = -Obs_1_AvoidedBound;
    const double internal_0 = -ego_l;

    // Evaluation of Vector values
    values(0) = LHardLowerBound + internal_0;
    values(1) = -LHardUpperBound - internal_0;
    values(2) = SHardLowerBound + internal_1;
    values(3) = -SHardUpperBound - internal_1;
    values(4) = -agent_0_s;
    values(5) = VHardLowerBound + internal_2;
    values(6) = -VHardUpperBound - internal_2;
    values(7) = -agent_0_v;
    values(8) = ego_kappa_ref*ego_l - 0.99990000000000001;
    values(9) = -cos(ego_theta - ego_theta_ref);
    values(10) = -ego_f_2_agent_0_slack;
    values(11) = -ego_r_2_agent_0_slack;
    values(12) = -ego_f_2_object_0_slack;
    values(13) = -ego_r_2_object_0_slack;
    values(14) = -ego_f_2_object_1_slack;
    values(15) = -ego_r_2_object_1_slack;
    values(16) = SteerLowerBound + internal_3;
    values(17) = -SteerUpperBound - internal_3;
    values(18) = AHardLowerBound + internal_4;
    values(19) = -AHardUpperBound - internal_4;
    values(20) = Agent_0_AHardLowerBound + internal_5;
    values(21) = -Agent_0_AHardUpperBound - internal_5;
    values(22) = -ego_f_2_agent_0_slack - f_dis_2_agent_0 - internal_6;
    values(23) = -ego_r_2_agent_0_slack - internal_6 - r_dis_2_agent_0;
    values(24) = -ego_f_2_object_0_slack - f_dis_2_object_0 - internal_7;
    values(25) = -ego_f_2_object_1_slack - f_dis_2_object_1 - internal_8;
    values(26) = -ego_r_2_object_0_slack - internal_7 - r_dis_2_object_0;
    values(27) = -ego_r_2_object_1_slack - internal_8 - r_dis_2_object_1;

  }

  virtual void updateJacobian(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& jx, Eigen::MatrixXd& ju) const override {
    // Determine stage variables
    const auto& ego_theta = states(3);
    const auto& agent_0_x = states(6);
    const auto& agent_0_y = states(7);
    const auto& ego_theta_ref = params(2);
    const auto& ego_kappa_ref = params(3);
    const auto& agent_0_theta_ref = params(7);
    const auto& wheelbase = params(17);
    const auto& front_overhang = params(18);
    const auto& rear_overhang = params(19);
    const auto& agent_a_0_square = params(50);
    const auto& agent_b_0_square = params(52);
    const auto& object_a_0_square = params(56);
    const auto& object_b_0_square = params(58);
    const auto& object_x_0 = params(59);
    const auto& object_y_0 = params(60);
    const auto& object_cos_theta_0 = params(62);
    const auto& object_sin_theta_0 = params(63);
    const auto& object_a_1_square = params(68);
    const auto& object_b_1_square = params(70);
    const auto& object_x_1 = params(71);
    const auto& object_y_1 = params(72);
    const auto& object_cos_theta_1 = params(74);
    const auto& object_sin_theta_1 = params(75);

    // Determine global variables
    const auto& x_r_circle = globals(1);
    const auto& y_r_circle = globals(2);
    const auto& x_f_circle = globals(4);
    const auto& y_f_circle = globals(5);

    // Determine internal variables
    const double internal_32 = 2*rear_overhang;
    const double internal_39 = 2*object_cos_theta_0;
    const double internal_42 = 2*object_sin_theta_0;
    const double internal_48 = 2*object_cos_theta_1;
    const double internal_51 = 2*object_sin_theta_1;
    const double internal_22 = -x_r_circle;
    const double internal_24 = -y_r_circle;
    const double internal_4 = -x_f_circle;
    const double internal_6 = -y_f_circle;
    const double internal_16 = sin(ego_theta);
    const double internal_20 = cos(ego_theta);
    const double internal_0 = sin(ego_theta_ref);
    const double internal_1 = cos(ego_theta_ref);
    const double internal_2 = cos(agent_0_theta_ref);
    const double internal_8 = sin(agent_0_theta_ref);
    const double internal_17 = front_overhang + wheelbase;
    const double internal_3 = 1.0/agent_a_0_square;
    const double internal_11 = 1.0/agent_b_0_square;
    const double internal_35 = 1.0/object_a_0_square;
    const double internal_40 = 1.0/object_b_0_square;
    const double internal_44 = 1.0/object_a_1_square;
    const double internal_49 = 1.0/object_b_1_square;
    const double internal_18 = 2*internal_17;
    const double internal_23 = agent_0_x + internal_22;
    const double internal_53 = internal_22 + object_x_0;
    const double internal_57 = internal_22 + object_x_1;
    const double internal_25 = -agent_0_y - internal_24;
    const double internal_54 = -internal_24 - object_y_0;
    const double internal_58 = -internal_24 - object_y_1;
    const double internal_36 = internal_4 + object_x_0;
    const double internal_45 = internal_4 + object_x_1;
    const double internal_5 = agent_0_x + internal_4;
    const double internal_37 = -internal_6 - object_y_0;
    const double internal_46 = -internal_6 - object_y_1;
    const double internal_7 = -agent_0_y - internal_6;
    const double internal_33 = internal_20*internal_32;
    const double internal_34 = internal_16*internal_32;
    const double internal_19 = internal_18*internal_2;
    const double internal_21 = internal_18*internal_8;
    const double internal_43 = internal_18*object_cos_theta_0;
    const double internal_52 = internal_18*object_cos_theta_1;
    const double internal_26 = internal_3*(-internal_2*internal_23 + internal_25*internal_8);
    const double internal_28 = internal_11*(internal_2*internal_25 + internal_23*internal_8);
    const double internal_55 = internal_35*(-internal_53*object_cos_theta_0 + internal_54*object_sin_theta_0);
    const double internal_56 = internal_40*(internal_53*object_sin_theta_0 + internal_54*object_cos_theta_0);
    const double internal_59 = internal_44*(-internal_57*object_cos_theta_1 + internal_58*object_sin_theta_1);
    const double internal_60 = internal_49*(internal_57*object_sin_theta_1 + internal_58*object_cos_theta_1);
    const double internal_12 = internal_11*(internal_2*internal_7 + internal_5*internal_8);
    const double internal_38 = internal_35*(-internal_36*object_cos_theta_0 + internal_37*object_sin_theta_0);
    const double internal_41 = internal_40*(internal_36*object_sin_theta_0 + internal_37*object_cos_theta_0);
    const double internal_47 = internal_44*(-internal_45*object_cos_theta_1 + internal_46*object_sin_theta_1);
    const double internal_50 = internal_49*(internal_45*object_sin_theta_1 + internal_46*object_cos_theta_1);
    const double internal_9 = internal_3*(-internal_2*internal_5 + internal_7*internal_8);
    const double internal_27 = 2*internal_26;
    const double internal_29 = 2*internal_28;
    const double internal_10 = 2*internal_9;
    const double internal_13 = 2*internal_12;
    const double internal_30 = -internal_2*internal_27 + internal_29*internal_8;
    const double internal_31 = internal_2*internal_29 + internal_27*internal_8;
    const double internal_14 = -internal_10*internal_2 + internal_13*internal_8;
    const double internal_15 = internal_10*internal_8 + internal_13*internal_2;

    // Evaluation of Matrix jx
    jx(0, 1) = internal_0;
    jx(0, 2) = -internal_1;
    jx(1, 1) = -internal_0;
    jx(1, 2) = internal_1;
    jx(2, 0) = -1;
    jx(3, 0) = 1;
    jx(4, 5) = -1;
    jx(5, 4) = -1;
    jx(6, 4) = 1;
    jx(7, 9) = -1;
    jx(8, 1) = -ego_kappa_ref*internal_0;
    jx(8, 2) = ego_kappa_ref*internal_1;
    jx(9, 3) = sin(ego_theta - ego_theta_ref);
    jx(22, 1) = internal_14;
    jx(22, 2) = -internal_15;
    jx(22, 3) = -internal_12*(internal_16*internal_21 + internal_19*internal_20) - internal_9*(-internal_16*internal_19 + internal_20*internal_21);
    jx(22, 6) = -internal_14;
    jx(22, 7) = internal_15;
    jx(23, 1) = internal_30;
    jx(23, 2) = -internal_31;
    jx(23, 3) = -internal_26*(2*internal_16*internal_2*rear_overhang - internal_33*internal_8) - internal_28*(-internal_2*internal_33 - internal_34*internal_8);
    jx(23, 6) = -internal_30;
    jx(23, 7) = internal_31;
    jx(24, 1) = -internal_38*internal_39 + internal_41*internal_42;
    jx(24, 2) = -internal_38*internal_42 - internal_39*internal_41;
    jx(24, 3) = -internal_38*(-internal_16*internal_43 + 2*internal_17*internal_20*object_sin_theta_0) - internal_41*(internal_16*internal_18*object_sin_theta_0 + internal_20*internal_43);
    jx(25, 1) = -internal_47*internal_48 + internal_50*internal_51;
    jx(25, 2) = -internal_47*internal_51 - internal_48*internal_50;
    jx(25, 3) = -internal_47*(-internal_16*internal_52 + 2*internal_17*internal_20*object_sin_theta_1) - internal_50*(internal_16*internal_18*object_sin_theta_1 + internal_20*internal_52);
    jx(26, 1) = -internal_39*internal_55 + internal_42*internal_56;
    jx(26, 2) = -internal_39*internal_56 - internal_42*internal_55;
    jx(26, 3) = -internal_55*(-internal_33*object_sin_theta_0 + internal_34*object_cos_theta_0) - internal_56*(-internal_33*object_cos_theta_0 - internal_34*object_sin_theta_0);
    jx(27, 1) = -internal_48*internal_59 + internal_51*internal_60;
    jx(27, 2) = -internal_48*internal_60 - internal_51*internal_59;
    jx(27, 3) = -internal_59*(-internal_33*object_sin_theta_1 + internal_34*object_cos_theta_1) - internal_60*(-internal_33*object_cos_theta_1 - internal_34*object_sin_theta_1);

    // Evaluation of Matrix ju
    ju(10, 3) = -1;
    ju(11, 4) = -1;
    ju(12, 5) = -1;
    ju(13, 6) = -1;
    ju(14, 7) = -1;
    ju(15, 8) = -1;
    ju(16, 1) = -1;
    ju(17, 1) = 1;
    ju(18, 0) = -1;
    ju(19, 0) = 1;
    ju(20, 2) = -1;
    ju(21, 2) = 1;
    ju(22, 3) = -1;
    ju(23, 4) = -1;
    ju(24, 5) = -1;
    ju(25, 7) = -1;
    ju(26, 6) = -1;
    ju(27, 8) = -1;

  }

  virtual void updateStateJacobian(const Eigen::VectorXd& states, Eigen::MatrixXd& jx) const override {
    // Determine stage variables
    const auto& ego_theta = states(3);
    const auto& ego_theta_ref = params(2);
    const auto& ego_kappa_ref = params(3);

    // Determine internal variables
    const double internal_0 = sin(ego_theta_ref);
    const double internal_1 = cos(ego_theta_ref);

    // Evaluation of Matrix jx
    jx(0, 1) = internal_0;
    jx(0, 2) = -internal_1;
    jx(1, 1) = -internal_0;
    jx(1, 2) = internal_1;
    jx(2, 0) = -1;
    jx(3, 0) = 1;
    jx(4, 5) = -1;
    jx(5, 4) = -1;
    jx(6, 4) = 1;
    jx(7, 9) = -1;
    jx(8, 1) = -ego_kappa_ref*internal_0;
    jx(8, 2) = ego_kappa_ref*internal_1;
    jx(9, 3) = sin(ego_theta - ego_theta_ref);

  }

  virtual void updateCtrlJacobian(const Eigen::VectorXd& ctrls, Eigen::MatrixXd& ju) const override {
    // Evaluation of Matrix ju
    ju(0, 3) = -1;
    ju(1, 4) = -1;
    ju(2, 5) = -1;
    ju(3, 6) = -1;
    ju(4, 7) = -1;
    ju(5, 8) = -1;
    ju(6, 1) = -1;
    ju(7, 1) = 1;
    ju(8, 0) = -1;
    ju(9, 0) = 1;
    ju(10, 2) = -1;
    ju(11, 2) = 1;

  }

  virtual void updateJacobianDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& jx) const override {
    // Determine stage variables
    const auto& ego_theta = states(3);
    const auto& agent_0_x = states(6);
    const auto& agent_0_y = states(7);
    const auto& ego_theta_ref = params(2);
    const auto& ego_kappa_ref = params(3);
    const auto& agent_0_theta_ref = params(7);
    const auto& wheelbase = params(17);
    const auto& front_overhang = params(18);
    const auto& rear_overhang = params(19);
    const auto& agent_a_0_square = params(50);
    const auto& agent_b_0_square = params(52);
    const auto& object_a_0_square = params(56);
    const auto& object_b_0_square = params(58);
    const auto& object_x_0 = params(59);
    const auto& object_y_0 = params(60);
    const auto& object_cos_theta_0 = params(62);
    const auto& object_sin_theta_0 = params(63);
    const auto& object_a_1_square = params(68);
    const auto& object_b_1_square = params(70);
    const auto& object_x_1 = params(71);
    const auto& object_y_1 = params(72);
    const auto& object_cos_theta_1 = params(74);
    const auto& object_sin_theta_1 = params(75);

    // Determine global variables
    const auto& x_r_circle = globals(1);
    const auto& y_r_circle = globals(2);
    const auto& x_f_circle = globals(4);
    const auto& y_f_circle = globals(5);

    // Determine internal variables
    const double internal_32 = 2*rear_overhang;
    const double internal_39 = 2*object_cos_theta_0;
    const double internal_42 = 2*object_sin_theta_0;
    const double internal_48 = 2*object_cos_theta_1;
    const double internal_51 = 2*object_sin_theta_1;
    const double internal_22 = -x_r_circle;
    const double internal_24 = -y_r_circle;
    const double internal_4 = -x_f_circle;
    const double internal_6 = -y_f_circle;
    const double internal_16 = sin(ego_theta);
    const double internal_20 = cos(ego_theta);
    const double internal_0 = sin(ego_theta_ref);
    const double internal_1 = cos(ego_theta_ref);
    const double internal_2 = cos(agent_0_theta_ref);
    const double internal_8 = sin(agent_0_theta_ref);
    const double internal_17 = front_overhang + wheelbase;
    const double internal_3 = 1.0/agent_a_0_square;
    const double internal_11 = 1.0/agent_b_0_square;
    const double internal_35 = 1.0/object_a_0_square;
    const double internal_40 = 1.0/object_b_0_square;
    const double internal_44 = 1.0/object_a_1_square;
    const double internal_49 = 1.0/object_b_1_square;
    const double internal_18 = 2*internal_17;
    const double internal_23 = agent_0_x + internal_22;
    const double internal_53 = internal_22 + object_x_0;
    const double internal_57 = internal_22 + object_x_1;
    const double internal_25 = -agent_0_y - internal_24;
    const double internal_54 = -internal_24 - object_y_0;
    const double internal_58 = -internal_24 - object_y_1;
    const double internal_36 = internal_4 + object_x_0;
    const double internal_45 = internal_4 + object_x_1;
    const double internal_5 = agent_0_x + internal_4;
    const double internal_37 = -internal_6 - object_y_0;
    const double internal_46 = -internal_6 - object_y_1;
    const double internal_7 = -agent_0_y - internal_6;
    const double internal_33 = internal_20*internal_32;
    const double internal_34 = internal_16*internal_32;
    const double internal_19 = internal_18*internal_2;
    const double internal_21 = internal_18*internal_8;
    const double internal_43 = internal_18*object_cos_theta_0;
    const double internal_52 = internal_18*object_cos_theta_1;
    const double internal_26 = internal_3*(-internal_2*internal_23 + internal_25*internal_8);
    const double internal_28 = internal_11*(internal_2*internal_25 + internal_23*internal_8);
    const double internal_55 = internal_35*(-internal_53*object_cos_theta_0 + internal_54*object_sin_theta_0);
    const double internal_56 = internal_40*(internal_53*object_sin_theta_0 + internal_54*object_cos_theta_0);
    const double internal_59 = internal_44*(-internal_57*object_cos_theta_1 + internal_58*object_sin_theta_1);
    const double internal_60 = internal_49*(internal_57*object_sin_theta_1 + internal_58*object_cos_theta_1);
    const double internal_12 = internal_11*(internal_2*internal_7 + internal_5*internal_8);
    const double internal_38 = internal_35*(-internal_36*object_cos_theta_0 + internal_37*object_sin_theta_0);
    const double internal_41 = internal_40*(internal_36*object_sin_theta_0 + internal_37*object_cos_theta_0);
    const double internal_47 = internal_44*(-internal_45*object_cos_theta_1 + internal_46*object_sin_theta_1);
    const double internal_50 = internal_49*(internal_45*object_sin_theta_1 + internal_46*object_cos_theta_1);
    const double internal_9 = internal_3*(-internal_2*internal_5 + internal_7*internal_8);
    const double internal_27 = 2*internal_26;
    const double internal_29 = 2*internal_28;
    const double internal_10 = 2*internal_9;
    const double internal_13 = 2*internal_12;
    const double internal_30 = -internal_2*internal_27 + internal_29*internal_8;
    const double internal_31 = internal_2*internal_29 + internal_27*internal_8;
    const double internal_14 = -internal_10*internal_2 + internal_13*internal_8;
    const double internal_15 = internal_10*internal_8 + internal_13*internal_2;

    // Evaluation of Matrix jx
    jx(0, 1) = internal_0;
    jx(0, 2) = -internal_1;
    jx(1, 1) = -internal_0;
    jx(1, 2) = internal_1;
    jx(2, 0) = -1;
    jx(3, 0) = 1;
    jx(4, 5) = -1;
    jx(5, 4) = -1;
    jx(6, 4) = 1;
    jx(7, 9) = -1;
    jx(8, 1) = -ego_kappa_ref*internal_0;
    jx(8, 2) = ego_kappa_ref*internal_1;
    jx(9, 3) = sin(ego_theta - ego_theta_ref);
    jx(22, 1) = internal_14;
    jx(22, 2) = -internal_15;
    jx(22, 3) = -internal_12*(internal_16*internal_21 + internal_19*internal_20) - internal_9*(-internal_16*internal_19 + internal_20*internal_21);
    jx(22, 6) = -internal_14;
    jx(22, 7) = internal_15;
    jx(23, 1) = internal_30;
    jx(23, 2) = -internal_31;
    jx(23, 3) = -internal_26*(2*internal_16*internal_2*rear_overhang - internal_33*internal_8) - internal_28*(-internal_2*internal_33 - internal_34*internal_8);
    jx(23, 6) = -internal_30;
    jx(23, 7) = internal_31;
    jx(24, 1) = -internal_38*internal_39 + internal_41*internal_42;
    jx(24, 2) = -internal_38*internal_42 - internal_39*internal_41;
    jx(24, 3) = -internal_38*(-internal_16*internal_43 + 2*internal_17*internal_20*object_sin_theta_0) - internal_41*(internal_16*internal_18*object_sin_theta_0 + internal_20*internal_43);
    jx(25, 1) = -internal_47*internal_48 + internal_50*internal_51;
    jx(25, 2) = -internal_47*internal_51 - internal_48*internal_50;
    jx(25, 3) = -internal_47*(-internal_16*internal_52 + 2*internal_17*internal_20*object_sin_theta_1) - internal_50*(internal_16*internal_18*object_sin_theta_1 + internal_20*internal_52);
    jx(26, 1) = -internal_39*internal_55 + internal_42*internal_56;
    jx(26, 2) = -internal_39*internal_56 - internal_42*internal_55;
    jx(26, 3) = -internal_55*(-internal_33*object_sin_theta_0 + internal_34*object_cos_theta_0) - internal_56*(-internal_33*object_cos_theta_0 - internal_34*object_sin_theta_0);
    jx(27, 1) = -internal_48*internal_59 + internal_51*internal_60;
    jx(27, 2) = -internal_48*internal_60 - internal_51*internal_59;
    jx(27, 3) = -internal_59*(-internal_33*object_sin_theta_1 + internal_34*object_cos_theta_1) - internal_60*(-internal_33*object_cos_theta_1 - internal_34*object_sin_theta_1);

  }

  virtual void updateJacobianDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& ju) const override {
    // Evaluation of Matrix ju
    ju(10, 3) = -1;
    ju(11, 4) = -1;
    ju(12, 5) = -1;
    ju(13, 6) = -1;
    ju(14, 7) = -1;
    ju(15, 8) = -1;
    ju(16, 1) = -1;
    ju(17, 1) = 1;
    ju(18, 0) = -1;
    ju(19, 0) = 1;
    ju(20, 2) = -1;
    ju(21, 2) = 1;
    ju(22, 3) = -1;
    ju(23, 4) = -1;
    ju(24, 5) = -1;
    ju(25, 7) = -1;
    ju(26, 6) = -1;
    ju(27, 8) = -1;

  }

  virtual void updateDeltaEvaluation(const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::VectorXd& dx, const Eigen::VectorXd& du, Eigen::VectorXd& dg) const override {
    // Evaluation of Vector dg
    dg(0) = dx(1)*g_x(0, 1) + dx(2)*g_x(0, 2);
    dg(1) = dx(1)*g_x(1, 1) + dx(2)*g_x(1, 2);
    dg(2) = dx(0)*g_x(2, 0);
    dg(3) = dx(0)*g_x(3, 0);
    dg(4) = dx(5)*g_x(4, 5);
    dg(5) = dx(4)*g_x(5, 4);
    dg(6) = dx(4)*g_x(6, 4);
    dg(7) = dx(9)*g_x(7, 9);
    dg(8) = dx(1)*g_x(8, 1) + dx(2)*g_x(8, 2);
    dg(9) = dx(3)*g_x(9, 3);
    dg(10) = du(3)*g_u(10, 3);
    dg(11) = du(4)*g_u(11, 4);
    dg(12) = du(5)*g_u(12, 5);
    dg(13) = du(6)*g_u(13, 6);
    dg(14) = du(7)*g_u(14, 7);
    dg(15) = du(8)*g_u(15, 8);
    dg(16) = du(1)*g_u(16, 1);
    dg(17) = du(1)*g_u(17, 1);
    dg(18) = du(0)*g_u(18, 0);
    dg(19) = du(0)*g_u(19, 0);
    dg(20) = du(2)*g_u(20, 2);
    dg(21) = du(2)*g_u(21, 2);
    dg(22) = du(3)*g_u(22, 3) + dx(1)*g_x(22, 1) + dx(2)*g_x(22, 2) + dx(3)*g_x(22, 3) + dx(6)*g_x(22, 6) + dx(7)*g_x(22, 7);
    dg(23) = du(4)*g_u(23, 4) + dx(1)*g_x(23, 1) + dx(2)*g_x(23, 2) + dx(3)*g_x(23, 3) + dx(6)*g_x(23, 6) + dx(7)*g_x(23, 7);
    dg(24) = du(5)*g_u(24, 5) + dx(1)*g_x(24, 1) + dx(2)*g_x(24, 2) + dx(3)*g_x(24, 3);
    dg(25) = du(7)*g_u(25, 7) + dx(1)*g_x(25, 1) + dx(2)*g_x(25, 2) + dx(3)*g_x(25, 3);
    dg(26) = du(6)*g_u(26, 6) + dx(1)*g_x(26, 1) + dx(2)*g_x(26, 2) + dx(3)*g_x(26, 3);
    dg(27) = du(8)*g_u(27, 8) + dx(1)*g_x(27, 1) + dx(2)*g_x(27, 2) + dx(3)*g_x(27, 3);

  }

};

template <>
class MultiAgentsTrajectoryModelConstraints<StageType::INITIAL> : public Constraints {
 public:
  MultiAgentsTrajectoryModelConstraints() : Constraints(10, 9, 0, 12, 0) {
    // Evaluation of Matrix jacobian_.ju
    jacobian_.ju(0, 3) = -1;
    jacobian_.ju(1, 4) = -1;
    jacobian_.ju(2, 5) = -1;
    jacobian_.ju(3, 6) = -1;
    jacobian_.ju(4, 7) = -1;
    jacobian_.ju(5, 8) = -1;
    jacobian_.ju(6, 1) = -1;
    jacobian_.ju(7, 1) = 1;
    jacobian_.ju(8, 0) = -1;
    jacobian_.ju(9, 0) = 1;
    jacobian_.ju(10, 2) = -1;
    jacobian_.ju(11, 2) = 1;
    exprs_[0] = "ego_f_2_agent_0_slack >= 0.0";
    exprs_[1] = "ego_r_2_agent_0_slack >= 0.0";
    exprs_[2] = "ego_f_2_object_0_slack >= 0.0";
    exprs_[3] = "ego_r_2_object_0_slack >= 0.0";
    exprs_[4] = "ego_f_2_object_1_slack >= 0.0";
    exprs_[5] = "ego_r_2_object_1_slack >= 0.0";
    exprs_[6] = "SteerLowerBound <= ego_steer";
    exprs_[7] = "ego_steer <= SteerUpperBound";
    exprs_[8] = "AHardLowerBound <= ego_a";
    exprs_[9] = "ego_a <= AHardUpperBound";
    exprs_[10] = "Agent_0_AHardLowerBound <= agent_0_a";
    exprs_[11] = "agent_0_a <= Agent_0_AHardUpperBound";

    idx_with_slack_.emplace(0);
    idx_with_slack_.emplace(1);
    idx_with_slack_.emplace(2);
    idx_with_slack_.emplace(3);
    idx_with_slack_.emplace(4);
    idx_with_slack_.emplace(5);
  }

  virtual ~MultiAgentsTrajectoryModelConstraints() = default;

  virtual Type type() const override { return Type::INEQUALITY; }
  virtual void updateEvaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::VectorXd& values) const override {
    // Determine stage variables
    const auto& ego_a = ctrls(0);
    const auto& ego_steer = ctrls(1);
    const auto& agent_0_a = ctrls(2);
    const auto& ego_f_2_agent_0_slack = ctrls(3);
    const auto& ego_r_2_agent_0_slack = ctrls(4);
    const auto& ego_f_2_object_0_slack = ctrls(5);
    const auto& ego_r_2_object_0_slack = ctrls(6);
    const auto& ego_f_2_object_1_slack = ctrls(7);
    const auto& ego_r_2_object_1_slack = ctrls(8);
    const auto& SteerLowerBound = params(39);
    const auto& SteerUpperBound = params(40);
    const auto& AHardLowerBound = params(45);
    const auto& AHardUpperBound = params(46);
    const auto& Agent_0_AHardLowerBound = params(47);
    const auto& Agent_0_AHardUpperBound = params(48);

    // Determine internal variables
    const double internal_1 = -ego_a;
    const double internal_0 = -ego_steer;
    const double internal_2 = -agent_0_a;

    // Evaluation of Vector values
    values(0) = -ego_f_2_agent_0_slack;
    values(1) = -ego_r_2_agent_0_slack;
    values(2) = -ego_f_2_object_0_slack;
    values(3) = -ego_r_2_object_0_slack;
    values(4) = -ego_f_2_object_1_slack;
    values(5) = -ego_r_2_object_1_slack;
    values(6) = SteerLowerBound + internal_0;
    values(7) = -SteerUpperBound - internal_0;
    values(8) = AHardLowerBound + internal_1;
    values(9) = -AHardUpperBound - internal_1;
    values(10) = Agent_0_AHardLowerBound + internal_2;
    values(11) = -Agent_0_AHardUpperBound - internal_2;

  }

  virtual void updateJacobian(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& jx, Eigen::MatrixXd& ju) const override {
    // Evaluation of Matrix ju
    ju(0, 3) = -1;
    ju(1, 4) = -1;
    ju(2, 5) = -1;
    ju(3, 6) = -1;
    ju(4, 7) = -1;
    ju(5, 8) = -1;
    ju(6, 1) = -1;
    ju(7, 1) = 1;
    ju(8, 0) = -1;
    ju(9, 0) = 1;
    ju(10, 2) = -1;
    ju(11, 2) = 1;

  }

  virtual void updateCtrlJacobian(const Eigen::VectorXd& ctrls, Eigen::MatrixXd& ju) const override {
    // Evaluation of Matrix ju
    ju(0, 3) = -1;
    ju(1, 4) = -1;
    ju(2, 5) = -1;
    ju(3, 6) = -1;
    ju(4, 7) = -1;
    ju(5, 8) = -1;
    ju(6, 1) = -1;
    ju(7, 1) = 1;
    ju(8, 0) = -1;
    ju(9, 0) = 1;
    ju(10, 2) = -1;
    ju(11, 2) = 1;

  }

  virtual void updateJacobianDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& ju) const override {
    // Evaluation of Matrix ju
    ju(0, 3) = -1;
    ju(1, 4) = -1;
    ju(2, 5) = -1;
    ju(3, 6) = -1;
    ju(4, 7) = -1;
    ju(5, 8) = -1;
    ju(6, 1) = -1;
    ju(7, 1) = 1;
    ju(8, 0) = -1;
    ju(9, 0) = 1;
    ju(10, 2) = -1;
    ju(11, 2) = 1;

  }

  virtual void updateDeltaEvaluation(const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::VectorXd& dx, const Eigen::VectorXd& du, Eigen::VectorXd& dg) const override {
    // Evaluation of Vector dg
    dg(0) = du(3)*g_u(0, 3);
    dg(1) = du(4)*g_u(1, 4);
    dg(2) = du(5)*g_u(2, 5);
    dg(3) = du(6)*g_u(3, 6);
    dg(4) = du(7)*g_u(4, 7);
    dg(5) = du(8)*g_u(5, 8);
    dg(6) = du(1)*g_u(6, 1);
    dg(7) = du(1)*g_u(7, 1);
    dg(8) = du(0)*g_u(8, 0);
    dg(9) = du(0)*g_u(9, 0);
    dg(10) = du(2)*g_u(10, 2);
    dg(11) = du(2)*g_u(11, 2);

  }

};

template <>
class MultiAgentsTrajectoryModelConstraints<StageType::TERMINAL> : public Constraints {
 public:
  MultiAgentsTrajectoryModelConstraints() : Constraints(10, 9, 10, 0, 0) {
    // Evaluation of Matrix jacobian_.jx
    jacobian_.jx(2, 0) = -1;
    jacobian_.jx(3, 0) = 1;
    jacobian_.jx(4, 5) = -1;
    jacobian_.jx(5, 4) = -1;
    jacobian_.jx(6, 4) = 1;
    jacobian_.jx(7, 9) = -1;

    exprs_[0] = "LHardLowerBound <= -(ego_x - ego_x_ref)*sin(ego_theta_ref) + (ego_y - ego_y_ref)*cos(ego_theta_ref)";
    exprs_[1] = "-(ego_x - ego_x_ref)*sin(ego_theta_ref) + (ego_y - ego_y_ref)*cos(ego_theta_ref) <= LHardUpperBound";
    exprs_[2] = "SHardLowerBound <= ego_s";
    exprs_[3] = "ego_s <= SHardUpperBound";
    exprs_[4] = "agent_0_s >= 0.0";
    exprs_[5] = "VHardLowerBound <= ego_v";
    exprs_[6] = "ego_v <= VHardUpperBound";
    exprs_[7] = "agent_0_v >= 0.0";
    exprs_[8] = "-ego_kappa_ref*(-(ego_x - ego_x_ref)*sin(ego_theta_ref) + (ego_y - ego_y_ref)*cos(ego_theta_ref)) + 1 >= 0.0001";
    exprs_[9] = "cos(ego_theta - ego_theta_ref) >= 0.0";

  }

  virtual ~MultiAgentsTrajectoryModelConstraints() = default;

  virtual Type type() const override { return Type::INEQUALITY; }
  virtual void updateEvaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::VectorXd& values) const override {
    // Determine stage variables
    const auto& ego_s = states(0);
    const auto& ego_theta = states(3);
    const auto& ego_v = states(4);
    const auto& agent_0_s = states(5);
    const auto& agent_0_v = states(9);
    const auto& ego_theta_ref = params(2);
    const auto& ego_kappa_ref = params(3);
    const auto& LHardLowerBound = params(37);
    const auto& LHardUpperBound = params(38);
    const auto& SHardLowerBound = params(41);
    const auto& SHardUpperBound = params(42);
    const auto& VHardLowerBound = params(43);
    const auto& VHardUpperBound = params(44);

    // Determine global variables
    const auto& ego_l = globals(3);

    // Determine internal variables
    const double internal_1 = -ego_s;
    const double internal_2 = -ego_v;
    const double internal_0 = -ego_l;

    // Evaluation of Vector values
    values(0) = LHardLowerBound + internal_0;
    values(1) = -LHardUpperBound - internal_0;
    values(2) = SHardLowerBound + internal_1;
    values(3) = -SHardUpperBound - internal_1;
    values(4) = -agent_0_s;
    values(5) = VHardLowerBound + internal_2;
    values(6) = -VHardUpperBound - internal_2;
    values(7) = -agent_0_v;
    values(8) = ego_kappa_ref*ego_l - 0.99990000000000001;
    values(9) = -cos(ego_theta - ego_theta_ref);

  }

  virtual void updateJacobian(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& jx, Eigen::MatrixXd& ju) const override {
    // Determine stage variables
    const auto& ego_theta = states(3);
    const auto& ego_theta_ref = params(2);
    const auto& ego_kappa_ref = params(3);

    // Determine internal variables
    const double internal_0 = sin(ego_theta_ref);
    const double internal_1 = cos(ego_theta_ref);

    // Evaluation of Matrix jx
    jx(0, 1) = internal_0;
    jx(0, 2) = -internal_1;
    jx(1, 1) = -internal_0;
    jx(1, 2) = internal_1;
    jx(2, 0) = -1;
    jx(3, 0) = 1;
    jx(4, 5) = -1;
    jx(5, 4) = -1;
    jx(6, 4) = 1;
    jx(7, 9) = -1;
    jx(8, 1) = -ego_kappa_ref*internal_0;
    jx(8, 2) = ego_kappa_ref*internal_1;
    jx(9, 3) = sin(ego_theta - ego_theta_ref);


  }

  virtual void updateStateJacobian(const Eigen::VectorXd& states, Eigen::MatrixXd& jx) const override {
    // Determine stage variables
    const auto& ego_theta = states(3);
    const auto& ego_theta_ref = params(2);
    const auto& ego_kappa_ref = params(3);

    // Determine internal variables
    const double internal_0 = sin(ego_theta_ref);
    const double internal_1 = cos(ego_theta_ref);

    // Evaluation of Matrix jx
    jx(0, 1) = internal_0;
    jx(0, 2) = -internal_1;
    jx(1, 1) = -internal_0;
    jx(1, 2) = internal_1;
    jx(2, 0) = -1;
    jx(3, 0) = 1;
    jx(4, 5) = -1;
    jx(5, 4) = -1;
    jx(6, 4) = 1;
    jx(7, 9) = -1;
    jx(8, 1) = -ego_kappa_ref*internal_0;
    jx(8, 2) = ego_kappa_ref*internal_1;
    jx(9, 3) = sin(ego_theta - ego_theta_ref);

  }

  virtual void updateJacobianDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& jx) const override {
    // Determine stage variables
    const auto& ego_theta = states(3);
    const auto& ego_theta_ref = params(2);
    const auto& ego_kappa_ref = params(3);

    // Determine internal variables
    const double internal_0 = sin(ego_theta_ref);
    const double internal_1 = cos(ego_theta_ref);

    // Evaluation of Matrix jx
    jx(0, 1) = internal_0;
    jx(0, 2) = -internal_1;
    jx(1, 1) = -internal_0;
    jx(1, 2) = internal_1;
    jx(2, 0) = -1;
    jx(3, 0) = 1;
    jx(4, 5) = -1;
    jx(5, 4) = -1;
    jx(6, 4) = 1;
    jx(7, 9) = -1;
    jx(8, 1) = -ego_kappa_ref*internal_0;
    jx(8, 2) = ego_kappa_ref*internal_1;
    jx(9, 3) = sin(ego_theta - ego_theta_ref);

  }

  virtual void updateDeltaEvaluation(const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::VectorXd& dx, const Eigen::VectorXd& du, Eigen::VectorXd& dg) const override {
    // Evaluation of Vector dg
    dg(0) = dx(1)*g_x(0, 1) + dx(2)*g_x(0, 2);
    dg(1) = dx(1)*g_x(1, 1) + dx(2)*g_x(1, 2);
    dg(2) = dx(0)*g_x(2, 0);
    dg(3) = dx(0)*g_x(3, 0);
    dg(4) = dx(5)*g_x(4, 5);
    dg(5) = dx(4)*g_x(5, 4);
    dg(6) = dx(4)*g_x(6, 4);
    dg(7) = dx(9)*g_x(7, 9);
    dg(8) = dx(1)*g_x(8, 1) + dx(2)*g_x(8, 2);
    dg(9) = dx(3)*g_x(9, 3);

  }

};

template <StageType Ttype>
class MultiAgentsTrajectoryModelStateOnlyEqualities : public Constraints {
 public:
  MultiAgentsTrajectoryModelStateOnlyEqualities() : Constraints(10, 9, 0, 0, 0) {
  }

  virtual ~MultiAgentsTrajectoryModelStateOnlyEqualities() = default;

  virtual Type type() const override { return Type::STATE_ONLY_EQUALITY; }
};

template <>
class MultiAgentsTrajectoryModelStateOnlyEqualities<StageType::INITIAL> : public Constraints {
 public:
  MultiAgentsTrajectoryModelStateOnlyEqualities() : Constraints(10, 9, 10, 0, 0) {
    // Evaluation of SymmetricMatrix jacobian_.jx
    jacobian_.jx(0, 0) = 1;
    jacobian_.jx(1, 1) = 1;
    jacobian_.jx(2, 2) = 1;
    jacobian_.jx(3, 3) = 1;
    jacobian_.jx(4, 4) = 1;
    jacobian_.jx(5, 5) = 1;
    jacobian_.jx(6, 6) = 1;
    jacobian_.jx(7, 7) = 1;
    jacobian_.jx(8, 8) = 1;
    jacobian_.jx(9, 9) = 1;

    exprs_[0] = "ego_s = 0";
    exprs_[1] = "ego_x = 0";
    exprs_[2] = "ego_y = 0";
    exprs_[3] = "ego_theta = 0";
    exprs_[4] = "ego_v = 0";
    exprs_[5] = "agent_0_s = 0";
    exprs_[6] = "agent_0_x = 0";
    exprs_[7] = "agent_0_y = 0";
    exprs_[8] = "agent_0_theta = 0";
    exprs_[9] = "agent_0_v = 0";
  }

  virtual ~MultiAgentsTrajectoryModelStateOnlyEqualities() = default;

  virtual Type type() const override { return Type::STATE_ONLY_EQUALITY; }
  virtual void updateEvaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::VectorXd& values) const override {
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

    // Evaluation of Vector values
    values(0) = ego_s;
    values(1) = ego_x;
    values(2) = ego_y;
    values(3) = ego_theta;
    values(4) = ego_v;
    values(5) = agent_0_s;
    values(6) = agent_0_x;
    values(7) = agent_0_y;
    values(8) = agent_0_theta;
    values(9) = agent_0_v;

  }

  virtual void updateJacobian(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& jx, Eigen::MatrixXd& ju) const override {
    // Evaluation of Matrix jx
    jx(0, 0) = 1;
    jx(1, 1) = 1;
    jx(2, 2) = 1;
    jx(3, 3) = 1;
    jx(4, 4) = 1;
    jx(5, 5) = 1;
    jx(6, 6) = 1;
    jx(7, 7) = 1;
    jx(8, 8) = 1;
    jx(9, 9) = 1;


  }

  virtual void updateStateJacobian(const Eigen::VectorXd& states, Eigen::MatrixXd& jx) const override {
    // Evaluation of Matrix jx
    jx(0, 0) = 1;
    jx(1, 1) = 1;
    jx(2, 2) = 1;
    jx(3, 3) = 1;
    jx(4, 4) = 1;
    jx(5, 5) = 1;
    jx(6, 6) = 1;
    jx(7, 7) = 1;
    jx(8, 8) = 1;
    jx(9, 9) = 1;

  }

  virtual void updateJacobianDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& jx) const override {
    // Evaluation of Matrix jx
    jx(0, 0) = 1;
    jx(1, 1) = 1;
    jx(2, 2) = 1;
    jx(3, 3) = 1;
    jx(4, 4) = 1;
    jx(5, 5) = 1;
    jx(6, 6) = 1;
    jx(7, 7) = 1;
    jx(8, 8) = 1;
    jx(9, 9) = 1;

  }

  virtual void updateDeltaEvaluation(const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::VectorXd& dx, const Eigen::VectorXd& du, Eigen::VectorXd& dg) const override {
    // Evaluation of Vector dg
    dg(0) = dx(0)*g_x(0, 0);
    dg(1) = dx(1)*g_x(1, 1);
    dg(2) = dx(2)*g_x(2, 2);
    dg(3) = dx(3)*g_x(3, 3);
    dg(4) = dx(4)*g_x(4, 4);
    dg(5) = dx(5)*g_x(5, 5);
    dg(6) = dx(6)*g_x(6, 6);
    dg(7) = dx(7)*g_x(7, 7);
    dg(8) = dx(8)*g_x(8, 8);
    dg(9) = dx(9)*g_x(9, 9);

  }

};

template <>
class MultiAgentsTrajectoryModelStateOnlyEqualities<StageType::TERMINAL> : public Constraints {
 public:
  MultiAgentsTrajectoryModelStateOnlyEqualities() : Constraints(10, 9, 0, 0, 0) {
  }

  virtual ~MultiAgentsTrajectoryModelStateOnlyEqualities() = default;

  virtual Type type() const override { return Type::STATE_ONLY_EQUALITY; }
};

}  // namespace gpal::pnc::planning