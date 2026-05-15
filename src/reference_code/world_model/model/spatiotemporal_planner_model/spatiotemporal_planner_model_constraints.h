#pragma once

#include "ocp/constraints.h"

namespace gpal::pnc::planning {

template <StageType Ttype>
class SpatiotemporalPlannerModelConstraints : public Constraints {
 public:
  SpatiotemporalPlannerModelConstraints() : Constraints(7, 13, 15, 4, 24) {
    // Evaluation of Matrix jacobian_.jx
    jacobian_.jx(6, 4) = -1;
    jacobian_.jx(7, 4) = 1;
    jacobian_.jx(8, 0) = -1;
    jacobian_.jx(9, 0) = 1;
    jacobian_.jx(10, 5) = -1;
    jacobian_.jx(11, 5) = 1;
    jacobian_.jx(12, 6) = -1;
    jacobian_.jx(13, 6) = 1;
    jacobian_.jx(41, 0) = 1;
    jacobian_.jx(42, 5) = 1;

    // Evaluation of Matrix jacobian_.ju
    jacobian_.ju(15, 0) = -1;
    jacobian_.ju(16, 0) = 1;
    jacobian_.ju(17, 1) = -1;
    jacobian_.ju(18, 1) = 1;
    jacobian_.ju(19, 5) = -1;
    jacobian_.ju(20, 6) = -1;
    jacobian_.ju(21, 7) = -1;
    jacobian_.ju(22, 8) = -1;
    jacobian_.ju(23, 9) = -1;
    jacobian_.ju(24, 10) = -1;
    jacobian_.ju(25, 11) = -1;
    jacobian_.ju(26, 12) = -1;
    jacobian_.ju(27, 5) = -1;
    jacobian_.ju(28, 6) = -1;
    jacobian_.ju(29, 7) = -1;
    jacobian_.ju(30, 8) = -1;
    jacobian_.ju(31, 9) = -1;
    jacobian_.ju(32, 10) = -1;
    jacobian_.ju(33, 11) = -1;
    jacobian_.ju(34, 12) = -1;
    jacobian_.ju(35, 2) = -1;
    jacobian_.ju(36, 2) = -1;
    jacobian_.ju(37, 2) = -1;
    jacobian_.ju(38, 2) = -1;
    jacobian_.ju(39, 2) = -1;
    jacobian_.ju(40, 2) = -1;
    jacobian_.ju(41, 3) = -1;
    jacobian_.ju(42, 4) = -1;
    exprs_[0] = "LHardLowerBound <= -(x - x_ref)*sin(theta_ref) + (y - y_ref)*cos(theta_ref)";
    exprs_[1] = "-(x - x_ref)*sin(theta_ref) + (y - y_ref)*cos(theta_ref) <= LHardUpperBound";
    exprs_[2] = "LFHardLowerBound <= -(x - xf_ref + (front_overhang + wheelbase)*cos(theta))*sin(thetaf_ref) + (y - yf_ref + (front_overhang + wheelbase)*sin(theta))*cos(thetaf_ref)";
    exprs_[3] = "-(x - xf_ref + (front_overhang + wheelbase)*cos(theta))*sin(thetaf_ref) + (y - yf_ref + (front_overhang + wheelbase)*sin(theta))*cos(thetaf_ref) <= LFHardUpperBound";
    exprs_[4] = "LRHardLowerBound <= (-rear_overhang*sin(theta) + y - yr_ref)*cos(thetar_ref) - (-rear_overhang*cos(theta) + x - xr_ref)*sin(thetar_ref)";
    exprs_[5] = "(-rear_overhang*sin(theta) + y - yr_ref)*cos(thetar_ref) - (-rear_overhang*cos(theta) + x - xr_ref)*sin(thetar_ref) <= LRHardUpperBound";
    exprs_[6] = "SteerLowerBound <= steer";
    exprs_[7] = "steer <= SteerUpperBound";
    exprs_[8] = "SHardLowerBound <= s";
    exprs_[9] = "s <= SHardUpperBound";
    exprs_[10] = "VHardLowerBound <= v";
    exprs_[11] = "v <= VHardUpperBound";
    exprs_[12] = "AHardLowerBound <= a";
    exprs_[13] = "a <= AHardUpperBound";
    exprs_[14] = "cos(theta - theta_ref) >= 0.0";
    exprs_[15] = "DSteerLowerBound <= dsteer";
    exprs_[16] = "dsteer <= DSteerUpperBound";
    exprs_[17] = "JHardLowerBound <= jerk";
    exprs_[18] = "jerk <= JHardUpperBound";
    exprs_[19] = "Obs_0_AvoidedBound <= obs_0_slack + (cos_theta_0*(y - y_0 + (front_overhang + wheelbase)*sin(theta)) + sin_theta_0*(-x + x_0 - (front_overhang + wheelbase)*cos(theta)))**2/b_0_square + (cos_theta_0*(x - x_0 + (front_overhang + wheelbase)*cos(theta)) + sin_theta_0*(y - y_0 + (front_overhang + wheelbase)*sin(theta)))**2/a_0_square";
    exprs_[20] = "Obs_1_AvoidedBound <= obs_1_slack + (cos_theta_1*(y - y_1 + (front_overhang + wheelbase)*sin(theta)) + sin_theta_1*(-x + x_1 - (front_overhang + wheelbase)*cos(theta)))**2/b_1_square + (cos_theta_1*(x - x_1 + (front_overhang + wheelbase)*cos(theta)) + sin_theta_1*(y - y_1 + (front_overhang + wheelbase)*sin(theta)))**2/a_1_square";
    exprs_[21] = "Obs_2_AvoidedBound <= obs_2_slack + (cos_theta_2*(y - y_2 + (front_overhang + wheelbase)*sin(theta)) + sin_theta_2*(-x + x_2 - (front_overhang + wheelbase)*cos(theta)))**2/b_2_square + (cos_theta_2*(x - x_2 + (front_overhang + wheelbase)*cos(theta)) + sin_theta_2*(y - y_2 + (front_overhang + wheelbase)*sin(theta)))**2/a_2_square";
    exprs_[22] = "Obs_3_AvoidedBound <= obs_3_slack + (cos_theta_3*(y - y_3 + (front_overhang + wheelbase)*sin(theta)) + sin_theta_3*(-x + x_3 - (front_overhang + wheelbase)*cos(theta)))**2/b_3_square + (cos_theta_3*(x - x_3 + (front_overhang + wheelbase)*cos(theta)) + sin_theta_3*(y - y_3 + (front_overhang + wheelbase)*sin(theta)))**2/a_3_square";
    exprs_[23] = "Obs_4_AvoidedBound <= obs_4_slack + (cos_theta_4*(y - y_4 + (front_overhang + wheelbase)*sin(theta)) + sin_theta_4*(-x + x_4 - (front_overhang + wheelbase)*cos(theta)))**2/b_4_square + (cos_theta_4*(x - x_4 + (front_overhang + wheelbase)*cos(theta)) + sin_theta_4*(y - y_4 + (front_overhang + wheelbase)*sin(theta)))**2/a_4_square";
    exprs_[24] = "Obs_5_AvoidedBound <= obs_5_slack + (cos_theta_5*(y - y_5 + (front_overhang + wheelbase)*sin(theta)) + sin_theta_5*(-x + x_5 - (front_overhang + wheelbase)*cos(theta)))**2/b_5_square + (cos_theta_5*(x - x_5 + (front_overhang + wheelbase)*cos(theta)) + sin_theta_5*(y - y_5 + (front_overhang + wheelbase)*sin(theta)))**2/a_5_square";
    exprs_[25] = "Obs_6_AvoidedBound <= obs_6_slack + (cos_theta_6*(y - y_6 + (front_overhang + wheelbase)*sin(theta)) + sin_theta_6*(-x + x_6 - (front_overhang + wheelbase)*cos(theta)))**2/b_6_square + (cos_theta_6*(x - x_6 + (front_overhang + wheelbase)*cos(theta)) + sin_theta_6*(y - y_6 + (front_overhang + wheelbase)*sin(theta)))**2/a_6_square";
    exprs_[26] = "Obs_7_AvoidedBound <= obs_7_slack + (cos_theta_7*(y - y_7 + (front_overhang + wheelbase)*sin(theta)) + sin_theta_7*(-x + x_7 - (front_overhang + wheelbase)*cos(theta)))**2/b_7_square + (cos_theta_7*(x - x_7 + (front_overhang + wheelbase)*cos(theta)) + sin_theta_7*(y - y_7 + (front_overhang + wheelbase)*sin(theta)))**2/a_7_square";
    exprs_[27] = "Obs_0_AvoidedBound <= obs_0_slack + (cos_theta_0*(-rear_overhang*sin(theta) + y - y_0) + sin_theta_0*(rear_overhang*cos(theta) - x + x_0))**2/b_0_square + (cos_theta_0*(-rear_overhang*cos(theta) + x - x_0) + sin_theta_0*(-rear_overhang*sin(theta) + y - y_0))**2/a_0_square";
    exprs_[28] = "Obs_1_AvoidedBound <= obs_1_slack + (cos_theta_1*(-rear_overhang*sin(theta) + y - y_1) + sin_theta_1*(rear_overhang*cos(theta) - x + x_1))**2/b_1_square + (cos_theta_1*(-rear_overhang*cos(theta) + x - x_1) + sin_theta_1*(-rear_overhang*sin(theta) + y - y_1))**2/a_1_square";
    exprs_[29] = "Obs_2_AvoidedBound <= obs_2_slack + (cos_theta_2*(-rear_overhang*sin(theta) + y - y_2) + sin_theta_2*(rear_overhang*cos(theta) - x + x_2))**2/b_2_square + (cos_theta_2*(-rear_overhang*cos(theta) + x - x_2) + sin_theta_2*(-rear_overhang*sin(theta) + y - y_2))**2/a_2_square";
    exprs_[30] = "Obs_3_AvoidedBound <= obs_3_slack + (cos_theta_3*(-rear_overhang*sin(theta) + y - y_3) + sin_theta_3*(rear_overhang*cos(theta) - x + x_3))**2/b_3_square + (cos_theta_3*(-rear_overhang*cos(theta) + x - x_3) + sin_theta_3*(-rear_overhang*sin(theta) + y - y_3))**2/a_3_square";
    exprs_[31] = "Obs_4_AvoidedBound <= obs_4_slack + (cos_theta_4*(-rear_overhang*sin(theta) + y - y_4) + sin_theta_4*(rear_overhang*cos(theta) - x + x_4))**2/b_4_square + (cos_theta_4*(-rear_overhang*cos(theta) + x - x_4) + sin_theta_4*(-rear_overhang*sin(theta) + y - y_4))**2/a_4_square";
    exprs_[32] = "Obs_5_AvoidedBound <= obs_5_slack + (cos_theta_5*(-rear_overhang*sin(theta) + y - y_5) + sin_theta_5*(rear_overhang*cos(theta) - x + x_5))**2/b_5_square + (cos_theta_5*(-rear_overhang*cos(theta) + x - x_5) + sin_theta_5*(-rear_overhang*sin(theta) + y - y_5))**2/a_5_square";
    exprs_[33] = "Obs_6_AvoidedBound <= obs_6_slack + (cos_theta_6*(-rear_overhang*sin(theta) + y - y_6) + sin_theta_6*(rear_overhang*cos(theta) - x + x_6))**2/b_6_square + (cos_theta_6*(-rear_overhang*cos(theta) + x - x_6) + sin_theta_6*(-rear_overhang*sin(theta) + y - y_6))**2/a_6_square";
    exprs_[34] = "Obs_7_AvoidedBound <= obs_7_slack + (cos_theta_7*(-rear_overhang*sin(theta) + y - y_7) + sin_theta_7*(rear_overhang*cos(theta) - x + x_7))**2/b_7_square + (cos_theta_7*(-rear_overhang*cos(theta) + x - x_7) + sin_theta_7*(-rear_overhang*sin(theta) + y - y_7))**2/a_7_square";
    exprs_[35] = "LSoftLowerBound <= l_slack - (x - x_ref)*sin(theta_ref) + (y - y_ref)*cos(theta_ref)";
    exprs_[36] = "-l_slack - (x - x_ref)*sin(theta_ref) + (y - y_ref)*cos(theta_ref) <= LSoftUpperBound";
    exprs_[37] = "LFSoftLowerBound <= l_slack - (x - xf_ref + (front_overhang + wheelbase)*cos(theta))*sin(thetaf_ref) + (y - yf_ref + (front_overhang + wheelbase)*sin(theta))*cos(thetaf_ref)";
    exprs_[38] = "-l_slack - (x - xf_ref + (front_overhang + wheelbase)*cos(theta))*sin(thetaf_ref) + (y - yf_ref + (front_overhang + wheelbase)*sin(theta))*cos(thetaf_ref) <= LFSoftUpperBound";
    exprs_[39] = "LRSoftLowerBound <= l_slack + (-rear_overhang*sin(theta) + y - yr_ref)*cos(thetar_ref) - (-rear_overhang*cos(theta) + x - xr_ref)*sin(thetar_ref)";
    exprs_[40] = "-l_slack + (-rear_overhang*sin(theta) + y - yr_ref)*cos(thetar_ref) - (-rear_overhang*cos(theta) + x - xr_ref)*sin(thetar_ref) <= LRSoftUpperBound";
    exprs_[41] = "s - s_slack <= SSoftUpperBound";
    exprs_[42] = "v - v_slack <= VSoftUpperBound";

    idx_with_slack_.emplace(19);
    idx_with_slack_.emplace(20);
    idx_with_slack_.emplace(21);
    idx_with_slack_.emplace(22);
    idx_with_slack_.emplace(23);
    idx_with_slack_.emplace(24);
    idx_with_slack_.emplace(25);
    idx_with_slack_.emplace(26);
    idx_with_slack_.emplace(27);
    idx_with_slack_.emplace(28);
    idx_with_slack_.emplace(29);
    idx_with_slack_.emplace(30);
    idx_with_slack_.emplace(31);
    idx_with_slack_.emplace(32);
    idx_with_slack_.emplace(33);
    idx_with_slack_.emplace(34);
    idx_with_slack_.emplace(35);
    idx_with_slack_.emplace(36);
    idx_with_slack_.emplace(37);
    idx_with_slack_.emplace(38);
    idx_with_slack_.emplace(39);
    idx_with_slack_.emplace(40);
    idx_with_slack_.emplace(41);
    idx_with_slack_.emplace(42);
  }

  virtual ~SpatiotemporalPlannerModelConstraints() = default;

  virtual Type type() const override { return Type::INEQUALITY; }
  virtual void updateEvaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::VectorXd& values) const override {
    // Determine stage variables
    const auto& s = states(0);
    const auto& theta = states(3);
    const auto& steer = states(4);
    const auto& v = states(5);
    const auto& a = states(6);
    const auto& dsteer = ctrls(0);
    const auto& jerk = ctrls(1);
    const auto& l_slack = ctrls(2);
    const auto& s_slack = ctrls(3);
    const auto& v_slack = ctrls(4);
    const auto& obs_0_slack = ctrls(5);
    const auto& obs_1_slack = ctrls(6);
    const auto& obs_2_slack = ctrls(7);
    const auto& obs_3_slack = ctrls(8);
    const auto& obs_4_slack = ctrls(9);
    const auto& obs_5_slack = ctrls(10);
    const auto& obs_6_slack = ctrls(11);
    const auto& obs_7_slack = ctrls(12);
    const auto& theta_ref = params(2);
    const auto& Obs_0_AvoidedBound = params(51);
    const auto& Obs_1_AvoidedBound = params(52);
    const auto& Obs_2_AvoidedBound = params(53);
    const auto& Obs_3_AvoidedBound = params(54);
    const auto& Obs_4_AvoidedBound = params(55);
    const auto& Obs_5_AvoidedBound = params(56);
    const auto& Obs_6_AvoidedBound = params(57);
    const auto& Obs_7_AvoidedBound = params(58);
    const auto& LHardLowerBound = params(59);
    const auto& LHardUpperBound = params(60);
    const auto& LFHardLowerBound = params(61);
    const auto& LFHardUpperBound = params(62);
    const auto& LRHardLowerBound = params(63);
    const auto& LRHardUpperBound = params(64);
    const auto& LSoftLowerBound = params(65);
    const auto& LSoftUpperBound = params(66);
    const auto& LFSoftLowerBound = params(67);
    const auto& LFSoftUpperBound = params(68);
    const auto& LRSoftLowerBound = params(69);
    const auto& LRSoftUpperBound = params(70);
    const auto& SteerLowerBound = params(71);
    const auto& SteerUpperBound = params(72);
    const auto& DSteerLowerBound = params(73);
    const auto& DSteerUpperBound = params(74);
    const auto& SHardLowerBound = params(75);
    const auto& SHardUpperBound = params(76);
    const auto& VHardLowerBound = params(77);
    const auto& VHardUpperBound = params(78);
    const auto& AHardLowerBound = params(79);
    const auto& AHardUpperBound = params(80);
    const auto& JHardLowerBound = params(81);
    const auto& JHardUpperBound = params(82);
    const auto& SSoftUpperBound = params(84);
    const auto& VSoftUpperBound = params(86);

    // Determine global variables
    const auto& l = globals(3);
    const auto& blr = globals(6);
    const auto& blf = globals(7);
    const auto& r_dis_2_obs_0 = globals(9);
    const auto& r_dis_2_obs_1 = globals(10);
    const auto& r_dis_2_obs_2 = globals(11);
    const auto& r_dis_2_obs_3 = globals(12);
    const auto& r_dis_2_obs_4 = globals(13);
    const auto& r_dis_2_obs_5 = globals(14);
    const auto& r_dis_2_obs_6 = globals(15);
    const auto& r_dis_2_obs_7 = globals(16);
    const auto& f_dis_2_obs_0 = globals(17);
    const auto& f_dis_2_obs_1 = globals(18);
    const auto& f_dis_2_obs_2 = globals(19);
    const auto& f_dis_2_obs_3 = globals(20);
    const auto& f_dis_2_obs_4 = globals(21);
    const auto& f_dis_2_obs_5 = globals(22);
    const auto& f_dis_2_obs_6 = globals(23);
    const auto& f_dis_2_obs_7 = globals(24);

    // Determine internal variables
    const double internal_4 = -s;
    const double internal_3 = -steer;
    const double internal_5 = -v;
    const double internal_6 = -a;
    const double internal_7 = -dsteer;
    const double internal_8 = -jerk;
    const double internal_0 = -l;
    const double internal_2 = -blr;
    const double internal_1 = -blf;
    const double internal_9 = -Obs_0_AvoidedBound + obs_0_slack;
    const double internal_10 = -Obs_1_AvoidedBound + obs_1_slack;
    const double internal_11 = -Obs_2_AvoidedBound + obs_2_slack;
    const double internal_12 = -Obs_3_AvoidedBound + obs_3_slack;
    const double internal_13 = -Obs_4_AvoidedBound + obs_4_slack;
    const double internal_14 = -Obs_5_AvoidedBound + obs_5_slack;
    const double internal_15 = -Obs_6_AvoidedBound + obs_6_slack;
    const double internal_16 = -Obs_7_AvoidedBound + obs_7_slack;

    // Evaluation of Vector values
    values(0) = LHardLowerBound + internal_0;
    values(1) = -LHardUpperBound - internal_0;
    values(2) = LFHardLowerBound + internal_1;
    values(3) = -LFHardUpperBound - internal_1;
    values(4) = LRHardLowerBound + internal_2;
    values(5) = -LRHardUpperBound - internal_2;
    values(6) = SteerLowerBound + internal_3;
    values(7) = -SteerUpperBound - internal_3;
    values(8) = SHardLowerBound + internal_4;
    values(9) = -SHardUpperBound - internal_4;
    values(10) = VHardLowerBound + internal_5;
    values(11) = -VHardUpperBound - internal_5;
    values(12) = AHardLowerBound + internal_6;
    values(13) = -AHardUpperBound - internal_6;
    values(14) = -cos(theta - theta_ref);
    values(15) = DSteerLowerBound + internal_7;
    values(16) = -DSteerUpperBound - internal_7;
    values(17) = JHardLowerBound + internal_8;
    values(18) = -JHardUpperBound - internal_8;
    values(19) = -f_dis_2_obs_0 - internal_9;
    values(20) = -f_dis_2_obs_1 - internal_10;
    values(21) = -f_dis_2_obs_2 - internal_11;
    values(22) = -f_dis_2_obs_3 - internal_12;
    values(23) = -f_dis_2_obs_4 - internal_13;
    values(24) = -f_dis_2_obs_5 - internal_14;
    values(25) = -f_dis_2_obs_6 - internal_15;
    values(26) = -f_dis_2_obs_7 - internal_16;
    values(27) = -internal_9 - r_dis_2_obs_0;
    values(28) = -internal_10 - r_dis_2_obs_1;
    values(29) = -internal_11 - r_dis_2_obs_2;
    values(30) = -internal_12 - r_dis_2_obs_3;
    values(31) = -internal_13 - r_dis_2_obs_4;
    values(32) = -internal_14 - r_dis_2_obs_5;
    values(33) = -internal_15 - r_dis_2_obs_6;
    values(34) = -internal_16 - r_dis_2_obs_7;
    values(35) = LSoftLowerBound - l - l_slack;
    values(36) = -LSoftUpperBound - internal_0 - l_slack;
    values(37) = LFSoftLowerBound - blf - l_slack;
    values(38) = -LFSoftUpperBound - internal_1 - l_slack;
    values(39) = LRSoftLowerBound - blr - l_slack;
    values(40) = -LRSoftUpperBound - internal_2 - l_slack;
    values(41) = -SSoftUpperBound - internal_4 - s_slack;
    values(42) = -VSoftUpperBound - internal_5 - v_slack;

  }

  virtual void updateJacobian(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& jx, Eigen::MatrixXd& ju) const override {
    // Determine stage variables
    const auto& theta = states(3);
    const auto& theta_ref = params(2);
    const auto& wheelbase = params(14);
    const auto& front_overhang = params(15);
    const auto& rear_overhang = params(16);
    const auto& thetaf_ref = params(47);
    const auto& thetar_ref = params(50);
    const auto& a_0_square = params(90);
    const auto& b_0_square = params(92);
    const auto& x_0 = params(93);
    const auto& y_0 = params(94);
    const auto& cos_theta_0 = params(96);
    const auto& sin_theta_0 = params(97);
    const auto& a_1_square = params(102);
    const auto& b_1_square = params(104);
    const auto& x_1 = params(105);
    const auto& y_1 = params(106);
    const auto& cos_theta_1 = params(108);
    const auto& sin_theta_1 = params(109);
    const auto& a_2_square = params(114);
    const auto& b_2_square = params(116);
    const auto& x_2 = params(117);
    const auto& y_2 = params(118);
    const auto& cos_theta_2 = params(120);
    const auto& sin_theta_2 = params(121);
    const auto& a_3_square = params(126);
    const auto& b_3_square = params(128);
    const auto& x_3 = params(129);
    const auto& y_3 = params(130);
    const auto& cos_theta_3 = params(132);
    const auto& sin_theta_3 = params(133);
    const auto& a_4_square = params(138);
    const auto& b_4_square = params(140);
    const auto& x_4 = params(141);
    const auto& y_4 = params(142);
    const auto& cos_theta_4 = params(144);
    const auto& sin_theta_4 = params(145);
    const auto& a_5_square = params(150);
    const auto& b_5_square = params(152);
    const auto& x_5 = params(153);
    const auto& y_5 = params(154);
    const auto& cos_theta_5 = params(156);
    const auto& sin_theta_5 = params(157);
    const auto& a_6_square = params(162);
    const auto& b_6_square = params(164);
    const auto& x_6 = params(165);
    const auto& y_6 = params(166);
    const auto& cos_theta_6 = params(168);
    const auto& sin_theta_6 = params(169);
    const auto& a_7_square = params(174);
    const auto& b_7_square = params(176);
    const auto& x_7 = params(177);
    const auto& y_7 = params(178);
    const auto& cos_theta_7 = params(180);
    const auto& sin_theta_7 = params(181);

    // Determine global variables
    const auto& x_r_circle = globals(1);
    const auto& y_r_circle = globals(2);
    const auto& x_f_circle = globals(4);
    const auto& y_f_circle = globals(5);

    // Determine internal variables
    const double internal_29 = 2*cos_theta_0;
    const double internal_32 = 2*sin_theta_0;
    const double internal_37 = 2*cos_theta_1;
    const double internal_40 = 2*sin_theta_1;
    const double internal_45 = 2*cos_theta_2;
    const double internal_48 = 2*sin_theta_2;
    const double internal_53 = 2*cos_theta_3;
    const double internal_56 = 2*sin_theta_3;
    const double internal_61 = 2*cos_theta_4;
    const double internal_64 = 2*sin_theta_4;
    const double internal_69 = 2*cos_theta_5;
    const double internal_72 = 2*sin_theta_5;
    const double internal_77 = 2*cos_theta_6;
    const double internal_80 = 2*sin_theta_6;
    const double internal_85 = 2*cos_theta_7;
    const double internal_88 = 2*sin_theta_7;
    const double internal_89 = -x_r_circle;
    const double internal_91 = -y_r_circle;
    const double internal_24 = -x_f_circle;
    const double internal_26 = -y_f_circle;
    const double internal_10 = sin(theta);
    const double internal_8 = cos(theta);
    const double internal_0 = sin(theta_ref);
    const double internal_1 = cos(theta_ref);
    const double internal_7 = front_overhang + wheelbase;
    const double internal_4 = sin(thetaf_ref);
    const double internal_5 = cos(thetaf_ref);
    const double internal_15 = sin(thetar_ref);
    const double internal_16 = cos(thetar_ref);
    const double internal_23 = 1.0/a_0_square;
    const double internal_30 = 1.0/b_0_square;
    const double internal_33 = 1.0/a_1_square;
    const double internal_38 = 1.0/b_1_square;
    const double internal_41 = 1.0/a_2_square;
    const double internal_46 = 1.0/b_2_square;
    const double internal_49 = 1.0/a_3_square;
    const double internal_54 = 1.0/b_3_square;
    const double internal_57 = 1.0/a_4_square;
    const double internal_62 = 1.0/b_4_square;
    const double internal_65 = 1.0/a_5_square;
    const double internal_70 = 1.0/b_5_square;
    const double internal_73 = 1.0/a_6_square;
    const double internal_78 = 1.0/b_6_square;
    const double internal_81 = 1.0/a_7_square;
    const double internal_86 = 1.0/b_7_square;
    const double internal_2 = -internal_1;
    const double internal_3 = -internal_0;
    const double internal_14 = -internal_4;
    const double internal_6 = -internal_5;
    const double internal_17 = -internal_16;
    const double internal_21 = -internal_15;
    const double internal_103 = internal_89 + x_3;
    const double internal_107 = internal_89 + x_4;
    const double internal_111 = internal_89 + x_5;
    const double internal_115 = internal_89 + x_6;
    const double internal_119 = internal_89 + x_7;
    const double internal_90 = internal_89 + x_0;
    const double internal_95 = internal_89 + x_1;
    const double internal_99 = internal_89 + x_2;
    const double internal_100 = -internal_91 - y_2;
    const double internal_104 = -internal_91 - y_3;
    const double internal_108 = -internal_91 - y_4;
    const double internal_112 = -internal_91 - y_5;
    const double internal_116 = -internal_91 - y_6;
    const double internal_120 = -internal_91 - y_7;
    const double internal_92 = -internal_91 - y_0;
    const double internal_96 = -internal_91 - y_1;
    const double internal_25 = internal_24 + x_0;
    const double internal_34 = internal_24 + x_1;
    const double internal_42 = internal_24 + x_2;
    const double internal_50 = internal_24 + x_3;
    const double internal_58 = internal_24 + x_4;
    const double internal_66 = internal_24 + x_5;
    const double internal_74 = internal_24 + x_6;
    const double internal_82 = internal_24 + x_7;
    const double internal_27 = -internal_26 - y_0;
    const double internal_35 = -internal_26 - y_1;
    const double internal_43 = -internal_26 - y_2;
    const double internal_51 = -internal_26 - y_3;
    const double internal_59 = -internal_26 - y_4;
    const double internal_67 = -internal_26 - y_5;
    const double internal_75 = -internal_26 - y_6;
    const double internal_83 = -internal_26 - y_7;
    const double internal_18 = internal_8*rear_overhang;
    const double internal_19 = internal_10*rear_overhang;
    const double internal_11 = internal_10*internal_7;
    const double internal_9 = internal_7*internal_8;
    const double internal_20 = internal_15*internal_19 + internal_16*internal_18;
    const double internal_12 = internal_11*internal_4 + internal_5*internal_9;
    const double internal_101 = internal_41*(-cos_theta_2*internal_99 + internal_100*sin_theta_2);
    const double internal_102 = internal_46*(cos_theta_2*internal_100 + internal_99*sin_theta_2);
    const double internal_105 = internal_49*(-cos_theta_3*internal_103 + internal_104*sin_theta_3);
    const double internal_106 = internal_54*(cos_theta_3*internal_104 + internal_103*sin_theta_3);
    const double internal_109 = internal_57*(-cos_theta_4*internal_107 + internal_108*sin_theta_4);
    const double internal_110 = internal_62*(cos_theta_4*internal_108 + internal_107*sin_theta_4);
    const double internal_113 = internal_65*(-cos_theta_5*internal_111 + internal_112*sin_theta_5);
    const double internal_114 = internal_70*(cos_theta_5*internal_112 + internal_111*sin_theta_5);
    const double internal_117 = internal_73*(-cos_theta_6*internal_115 + internal_116*sin_theta_6);
    const double internal_118 = internal_78*(cos_theta_6*internal_116 + internal_115*sin_theta_6);
    const double internal_121 = internal_81*(-cos_theta_7*internal_119 + internal_120*sin_theta_7);
    const double internal_122 = internal_86*(cos_theta_7*internal_120 + internal_119*sin_theta_7);
    const double internal_93 = internal_23*(-cos_theta_0*internal_90 + internal_92*sin_theta_0);
    const double internal_94 = internal_30*(cos_theta_0*internal_92 + internal_90*sin_theta_0);
    const double internal_97 = internal_33*(-cos_theta_1*internal_95 + internal_96*sin_theta_1);
    const double internal_98 = internal_38*(cos_theta_1*internal_96 + internal_95*sin_theta_1);
    const double internal_28 = internal_23*(-cos_theta_0*internal_25 + internal_27*sin_theta_0);
    const double internal_31 = internal_30*(cos_theta_0*internal_27 + internal_25*sin_theta_0);
    const double internal_36 = internal_33*(-cos_theta_1*internal_34 + internal_35*sin_theta_1);
    const double internal_39 = internal_38*(cos_theta_1*internal_35 + internal_34*sin_theta_1);
    const double internal_44 = internal_41*(-cos_theta_2*internal_42 + internal_43*sin_theta_2);
    const double internal_47 = internal_46*(cos_theta_2*internal_43 + internal_42*sin_theta_2);
    const double internal_52 = internal_49*(-cos_theta_3*internal_50 + internal_51*sin_theta_3);
    const double internal_55 = internal_54*(cos_theta_3*internal_51 + internal_50*sin_theta_3);
    const double internal_60 = internal_57*(-cos_theta_4*internal_58 + internal_59*sin_theta_4);
    const double internal_63 = internal_62*(cos_theta_4*internal_59 + internal_58*sin_theta_4);
    const double internal_68 = internal_65*(-cos_theta_5*internal_66 + internal_67*sin_theta_5);
    const double internal_71 = internal_70*(cos_theta_5*internal_67 + internal_66*sin_theta_5);
    const double internal_76 = internal_73*(-cos_theta_6*internal_74 + internal_75*sin_theta_6);
    const double internal_79 = internal_78*(cos_theta_6*internal_75 + internal_74*sin_theta_6);
    const double internal_84 = internal_81*(-cos_theta_7*internal_82 + internal_83*sin_theta_7);
    const double internal_87 = internal_86*(cos_theta_7*internal_83 + internal_82*sin_theta_7);
    const double internal_22 = -internal_20;
    const double internal_13 = -internal_12;

    // Evaluation of Matrix jx
    jx(0, 1) = internal_0;
    jx(0, 2) = internal_2;
    jx(1, 1) = internal_3;
    jx(1, 2) = internal_1;
    jx(2, 1) = internal_4;
    jx(2, 2) = internal_6;
    jx(2, 3) = internal_13;
    jx(3, 1) = internal_14;
    jx(3, 2) = internal_5;
    jx(3, 3) = internal_12;
    jx(4, 1) = internal_15;
    jx(4, 2) = internal_17;
    jx(4, 3) = internal_20;
    jx(5, 1) = internal_21;
    jx(5, 2) = internal_16;
    jx(5, 3) = internal_22;
    jx(6, 4) = -1;
    jx(7, 4) = 1;
    jx(8, 0) = -1;
    jx(9, 0) = 1;
    jx(10, 5) = -1;
    jx(11, 5) = 1;
    jx(12, 6) = -1;
    jx(13, 6) = 1;
    jx(14, 3) = sin(theta - theta_ref);
    jx(19, 1) = -internal_28*internal_29 + internal_31*internal_32;
    jx(19, 2) = -internal_28*internal_32 - internal_29*internal_31;
    jx(19, 3) = -internal_28*(-internal_11*internal_29 + 2*internal_7*internal_8*sin_theta_0) - internal_31*(internal_11*internal_32 + internal_29*internal_9);
    jx(20, 1) = -internal_36*internal_37 + internal_39*internal_40;
    jx(20, 2) = -internal_36*internal_40 - internal_37*internal_39;
    jx(20, 3) = -internal_36*(-internal_11*internal_37 + 2*internal_7*internal_8*sin_theta_1) - internal_39*(internal_11*internal_40 + internal_37*internal_9);
    jx(21, 1) = -internal_44*internal_45 + internal_47*internal_48;
    jx(21, 2) = -internal_44*internal_48 - internal_45*internal_47;
    jx(21, 3) = -internal_44*(-internal_11*internal_45 + 2*internal_7*internal_8*sin_theta_2) - internal_47*(internal_11*internal_48 + internal_45*internal_9);
    jx(22, 1) = -internal_52*internal_53 + internal_55*internal_56;
    jx(22, 2) = -internal_52*internal_56 - internal_53*internal_55;
    jx(22, 3) = -internal_52*(-internal_11*internal_53 + 2*internal_7*internal_8*sin_theta_3) - internal_55*(internal_11*internal_56 + internal_53*internal_9);
    jx(23, 1) = -internal_60*internal_61 + internal_63*internal_64;
    jx(23, 2) = -internal_60*internal_64 - internal_61*internal_63;
    jx(23, 3) = -internal_60*(-internal_11*internal_61 + 2*internal_7*internal_8*sin_theta_4) - internal_63*(internal_11*internal_64 + internal_61*internal_9);
    jx(24, 1) = -internal_68*internal_69 + internal_71*internal_72;
    jx(24, 2) = -internal_68*internal_72 - internal_69*internal_71;
    jx(24, 3) = -internal_68*(-internal_11*internal_69 + 2*internal_7*internal_8*sin_theta_5) - internal_71*(internal_11*internal_72 + internal_69*internal_9);
    jx(25, 1) = -internal_76*internal_77 + internal_79*internal_80;
    jx(25, 2) = -internal_76*internal_80 - internal_77*internal_79;
    jx(25, 3) = -internal_76*(-internal_11*internal_77 + 2*internal_7*internal_8*sin_theta_6) - internal_79*(internal_11*internal_80 + internal_77*internal_9);
    jx(26, 1) = -internal_84*internal_85 + internal_87*internal_88;
    jx(26, 2) = -internal_84*internal_88 - internal_85*internal_87;
    jx(26, 3) = -internal_84*(-internal_11*internal_85 + 2*internal_7*internal_8*sin_theta_7) - internal_87*(internal_11*internal_88 + internal_85*internal_9);
    jx(27, 1) = -internal_29*internal_93 + internal_32*internal_94;
    jx(27, 2) = -internal_29*internal_94 - internal_32*internal_93;
    jx(27, 3) = -internal_93*(-internal_18*internal_32 + internal_19*internal_29) - internal_94*(-internal_18*internal_29 - internal_19*internal_32);
    jx(28, 1) = -internal_37*internal_97 + internal_40*internal_98;
    jx(28, 2) = -internal_37*internal_98 - internal_40*internal_97;
    jx(28, 3) = -internal_97*(-internal_18*internal_40 + internal_19*internal_37) - internal_98*(-internal_18*internal_37 - internal_19*internal_40);
    jx(29, 1) = -internal_101*internal_45 + internal_102*internal_48;
    jx(29, 2) = -internal_101*internal_48 - internal_102*internal_45;
    jx(29, 3) = -internal_101*(-internal_18*internal_48 + internal_19*internal_45) - internal_102*(-internal_18*internal_45 - internal_19*internal_48);
    jx(30, 1) = -internal_105*internal_53 + internal_106*internal_56;
    jx(30, 2) = -internal_105*internal_56 - internal_106*internal_53;
    jx(30, 3) = -internal_105*(-internal_18*internal_56 + internal_19*internal_53) - internal_106*(-internal_18*internal_53 - internal_19*internal_56);
    jx(31, 1) = -internal_109*internal_61 + internal_110*internal_64;
    jx(31, 2) = -internal_109*internal_64 - internal_110*internal_61;
    jx(31, 3) = -internal_109*(-internal_18*internal_64 + internal_19*internal_61) - internal_110*(-internal_18*internal_61 - internal_19*internal_64);
    jx(32, 1) = -internal_113*internal_69 + internal_114*internal_72;
    jx(32, 2) = -internal_113*internal_72 - internal_114*internal_69;
    jx(32, 3) = -internal_113*(-internal_18*internal_72 + internal_19*internal_69) - internal_114*(-internal_18*internal_69 - internal_19*internal_72);
    jx(33, 1) = -internal_117*internal_77 + internal_118*internal_80;
    jx(33, 2) = -internal_117*internal_80 - internal_118*internal_77;
    jx(33, 3) = -internal_117*(-internal_18*internal_80 + internal_19*internal_77) - internal_118*(-internal_18*internal_77 - internal_19*internal_80);
    jx(34, 1) = -internal_121*internal_85 + internal_122*internal_88;
    jx(34, 2) = -internal_121*internal_88 - internal_122*internal_85;
    jx(34, 3) = -internal_121*(-internal_18*internal_88 + internal_19*internal_85) - internal_122*(-internal_18*internal_85 - internal_19*internal_88);
    jx(35, 1) = internal_0;
    jx(35, 2) = internal_2;
    jx(36, 1) = internal_3;
    jx(36, 2) = internal_1;
    jx(37, 1) = internal_4;
    jx(37, 2) = internal_6;
    jx(37, 3) = internal_13;
    jx(38, 1) = internal_14;
    jx(38, 2) = internal_5;
    jx(38, 3) = internal_12;
    jx(39, 1) = internal_15;
    jx(39, 2) = internal_17;
    jx(39, 3) = internal_20;
    jx(40, 1) = internal_21;
    jx(40, 2) = internal_16;
    jx(40, 3) = internal_22;
    jx(41, 0) = 1;
    jx(42, 5) = 1;

    // Evaluation of Matrix ju
    ju(15, 0) = -1;
    ju(16, 0) = 1;
    ju(17, 1) = -1;
    ju(18, 1) = 1;
    ju(19, 5) = -1;
    ju(20, 6) = -1;
    ju(21, 7) = -1;
    ju(22, 8) = -1;
    ju(23, 9) = -1;
    ju(24, 10) = -1;
    ju(25, 11) = -1;
    ju(26, 12) = -1;
    ju(27, 5) = -1;
    ju(28, 6) = -1;
    ju(29, 7) = -1;
    ju(30, 8) = -1;
    ju(31, 9) = -1;
    ju(32, 10) = -1;
    ju(33, 11) = -1;
    ju(34, 12) = -1;
    ju(35, 2) = -1;
    ju(36, 2) = -1;
    ju(37, 2) = -1;
    ju(38, 2) = -1;
    ju(39, 2) = -1;
    ju(40, 2) = -1;
    ju(41, 3) = -1;
    ju(42, 4) = -1;

  }

  virtual void updateStateJacobian(const Eigen::VectorXd& states, Eigen::MatrixXd& jx) const override {
    // Determine stage variables
    const auto& theta = states(3);
    const auto& theta_ref = params(2);
    const auto& wheelbase = params(14);
    const auto& front_overhang = params(15);
    const auto& rear_overhang = params(16);
    const auto& thetaf_ref = params(47);
    const auto& thetar_ref = params(50);

    // Determine internal variables
    const double internal_5 = cos(theta);
    const double internal_6 = sin(theta);
    const double internal_0 = sin(theta_ref);
    const double internal_1 = cos(theta_ref);
    const double internal_4 = front_overhang + wheelbase;
    const double internal_2 = sin(thetaf_ref);
    const double internal_3 = cos(thetaf_ref);
    const double internal_8 = sin(thetar_ref);
    const double internal_9 = cos(thetar_ref);
    const double internal_7 = internal_2*internal_4*internal_6 + internal_3*internal_4*internal_5;
    const double internal_10 = internal_5*internal_9*rear_overhang + internal_6*internal_8*rear_overhang;

    // Evaluation of Matrix jx
    jx(0, 1) = internal_0;
    jx(0, 2) = -internal_1;
    jx(1, 1) = -internal_0;
    jx(1, 2) = internal_1;
    jx(2, 1) = internal_2;
    jx(2, 2) = -internal_3;
    jx(2, 3) = -internal_7;
    jx(3, 1) = -internal_2;
    jx(3, 2) = internal_3;
    jx(3, 3) = internal_7;
    jx(4, 1) = internal_8;
    jx(4, 2) = -internal_9;
    jx(4, 3) = internal_10;
    jx(5, 1) = -internal_8;
    jx(5, 2) = internal_9;
    jx(5, 3) = -internal_10;
    jx(6, 4) = -1;
    jx(7, 4) = 1;
    jx(8, 0) = -1;
    jx(9, 0) = 1;
    jx(10, 5) = -1;
    jx(11, 5) = 1;
    jx(12, 6) = -1;
    jx(13, 6) = 1;
    jx(14, 3) = sin(theta - theta_ref);

  }

  virtual void updateCtrlJacobian(const Eigen::VectorXd& ctrls, Eigen::MatrixXd& ju) const override {
    // Evaluation of Matrix ju
    ju(0, 0) = -1;
    ju(1, 0) = 1;
    ju(2, 1) = -1;
    ju(3, 1) = 1;

  }

  virtual void updateJacobianDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& jx) const override {
    // Determine stage variables
    const auto& theta = states(3);
    const auto& theta_ref = params(2);
    const auto& wheelbase = params(14);
    const auto& front_overhang = params(15);
    const auto& rear_overhang = params(16);
    const auto& thetaf_ref = params(47);
    const auto& thetar_ref = params(50);
    const auto& a_0_square = params(90);
    const auto& b_0_square = params(92);
    const auto& x_0 = params(93);
    const auto& y_0 = params(94);
    const auto& cos_theta_0 = params(96);
    const auto& sin_theta_0 = params(97);
    const auto& a_1_square = params(102);
    const auto& b_1_square = params(104);
    const auto& x_1 = params(105);
    const auto& y_1 = params(106);
    const auto& cos_theta_1 = params(108);
    const auto& sin_theta_1 = params(109);
    const auto& a_2_square = params(114);
    const auto& b_2_square = params(116);
    const auto& x_2 = params(117);
    const auto& y_2 = params(118);
    const auto& cos_theta_2 = params(120);
    const auto& sin_theta_2 = params(121);
    const auto& a_3_square = params(126);
    const auto& b_3_square = params(128);
    const auto& x_3 = params(129);
    const auto& y_3 = params(130);
    const auto& cos_theta_3 = params(132);
    const auto& sin_theta_3 = params(133);
    const auto& a_4_square = params(138);
    const auto& b_4_square = params(140);
    const auto& x_4 = params(141);
    const auto& y_4 = params(142);
    const auto& cos_theta_4 = params(144);
    const auto& sin_theta_4 = params(145);
    const auto& a_5_square = params(150);
    const auto& b_5_square = params(152);
    const auto& x_5 = params(153);
    const auto& y_5 = params(154);
    const auto& cos_theta_5 = params(156);
    const auto& sin_theta_5 = params(157);
    const auto& a_6_square = params(162);
    const auto& b_6_square = params(164);
    const auto& x_6 = params(165);
    const auto& y_6 = params(166);
    const auto& cos_theta_6 = params(168);
    const auto& sin_theta_6 = params(169);
    const auto& a_7_square = params(174);
    const auto& b_7_square = params(176);
    const auto& x_7 = params(177);
    const auto& y_7 = params(178);
    const auto& cos_theta_7 = params(180);
    const auto& sin_theta_7 = params(181);

    // Determine global variables
    const auto& x_r_circle = globals(1);
    const auto& y_r_circle = globals(2);
    const auto& x_f_circle = globals(4);
    const auto& y_f_circle = globals(5);

    // Determine internal variables
    const double internal_29 = 2*cos_theta_0;
    const double internal_32 = 2*sin_theta_0;
    const double internal_37 = 2*cos_theta_1;
    const double internal_40 = 2*sin_theta_1;
    const double internal_45 = 2*cos_theta_2;
    const double internal_48 = 2*sin_theta_2;
    const double internal_53 = 2*cos_theta_3;
    const double internal_56 = 2*sin_theta_3;
    const double internal_61 = 2*cos_theta_4;
    const double internal_64 = 2*sin_theta_4;
    const double internal_69 = 2*cos_theta_5;
    const double internal_72 = 2*sin_theta_5;
    const double internal_77 = 2*cos_theta_6;
    const double internal_80 = 2*sin_theta_6;
    const double internal_85 = 2*cos_theta_7;
    const double internal_88 = 2*sin_theta_7;
    const double internal_89 = -x_r_circle;
    const double internal_91 = -y_r_circle;
    const double internal_24 = -x_f_circle;
    const double internal_26 = -y_f_circle;
    const double internal_10 = sin(theta);
    const double internal_8 = cos(theta);
    const double internal_0 = sin(theta_ref);
    const double internal_1 = cos(theta_ref);
    const double internal_7 = front_overhang + wheelbase;
    const double internal_4 = sin(thetaf_ref);
    const double internal_5 = cos(thetaf_ref);
    const double internal_15 = sin(thetar_ref);
    const double internal_16 = cos(thetar_ref);
    const double internal_23 = 1.0/a_0_square;
    const double internal_30 = 1.0/b_0_square;
    const double internal_33 = 1.0/a_1_square;
    const double internal_38 = 1.0/b_1_square;
    const double internal_41 = 1.0/a_2_square;
    const double internal_46 = 1.0/b_2_square;
    const double internal_49 = 1.0/a_3_square;
    const double internal_54 = 1.0/b_3_square;
    const double internal_57 = 1.0/a_4_square;
    const double internal_62 = 1.0/b_4_square;
    const double internal_65 = 1.0/a_5_square;
    const double internal_70 = 1.0/b_5_square;
    const double internal_73 = 1.0/a_6_square;
    const double internal_78 = 1.0/b_6_square;
    const double internal_81 = 1.0/a_7_square;
    const double internal_86 = 1.0/b_7_square;
    const double internal_2 = -internal_1;
    const double internal_3 = -internal_0;
    const double internal_14 = -internal_4;
    const double internal_6 = -internal_5;
    const double internal_17 = -internal_16;
    const double internal_21 = -internal_15;
    const double internal_103 = internal_89 + x_3;
    const double internal_107 = internal_89 + x_4;
    const double internal_111 = internal_89 + x_5;
    const double internal_115 = internal_89 + x_6;
    const double internal_119 = internal_89 + x_7;
    const double internal_90 = internal_89 + x_0;
    const double internal_95 = internal_89 + x_1;
    const double internal_99 = internal_89 + x_2;
    const double internal_100 = -internal_91 - y_2;
    const double internal_104 = -internal_91 - y_3;
    const double internal_108 = -internal_91 - y_4;
    const double internal_112 = -internal_91 - y_5;
    const double internal_116 = -internal_91 - y_6;
    const double internal_120 = -internal_91 - y_7;
    const double internal_92 = -internal_91 - y_0;
    const double internal_96 = -internal_91 - y_1;
    const double internal_25 = internal_24 + x_0;
    const double internal_34 = internal_24 + x_1;
    const double internal_42 = internal_24 + x_2;
    const double internal_50 = internal_24 + x_3;
    const double internal_58 = internal_24 + x_4;
    const double internal_66 = internal_24 + x_5;
    const double internal_74 = internal_24 + x_6;
    const double internal_82 = internal_24 + x_7;
    const double internal_27 = -internal_26 - y_0;
    const double internal_35 = -internal_26 - y_1;
    const double internal_43 = -internal_26 - y_2;
    const double internal_51 = -internal_26 - y_3;
    const double internal_59 = -internal_26 - y_4;
    const double internal_67 = -internal_26 - y_5;
    const double internal_75 = -internal_26 - y_6;
    const double internal_83 = -internal_26 - y_7;
    const double internal_18 = internal_8*rear_overhang;
    const double internal_19 = internal_10*rear_overhang;
    const double internal_11 = internal_10*internal_7;
    const double internal_9 = internal_7*internal_8;
    const double internal_20 = internal_15*internal_19 + internal_16*internal_18;
    const double internal_12 = internal_11*internal_4 + internal_5*internal_9;
    const double internal_101 = internal_41*(-cos_theta_2*internal_99 + internal_100*sin_theta_2);
    const double internal_102 = internal_46*(cos_theta_2*internal_100 + internal_99*sin_theta_2);
    const double internal_105 = internal_49*(-cos_theta_3*internal_103 + internal_104*sin_theta_3);
    const double internal_106 = internal_54*(cos_theta_3*internal_104 + internal_103*sin_theta_3);
    const double internal_109 = internal_57*(-cos_theta_4*internal_107 + internal_108*sin_theta_4);
    const double internal_110 = internal_62*(cos_theta_4*internal_108 + internal_107*sin_theta_4);
    const double internal_113 = internal_65*(-cos_theta_5*internal_111 + internal_112*sin_theta_5);
    const double internal_114 = internal_70*(cos_theta_5*internal_112 + internal_111*sin_theta_5);
    const double internal_117 = internal_73*(-cos_theta_6*internal_115 + internal_116*sin_theta_6);
    const double internal_118 = internal_78*(cos_theta_6*internal_116 + internal_115*sin_theta_6);
    const double internal_121 = internal_81*(-cos_theta_7*internal_119 + internal_120*sin_theta_7);
    const double internal_122 = internal_86*(cos_theta_7*internal_120 + internal_119*sin_theta_7);
    const double internal_93 = internal_23*(-cos_theta_0*internal_90 + internal_92*sin_theta_0);
    const double internal_94 = internal_30*(cos_theta_0*internal_92 + internal_90*sin_theta_0);
    const double internal_97 = internal_33*(-cos_theta_1*internal_95 + internal_96*sin_theta_1);
    const double internal_98 = internal_38*(cos_theta_1*internal_96 + internal_95*sin_theta_1);
    const double internal_28 = internal_23*(-cos_theta_0*internal_25 + internal_27*sin_theta_0);
    const double internal_31 = internal_30*(cos_theta_0*internal_27 + internal_25*sin_theta_0);
    const double internal_36 = internal_33*(-cos_theta_1*internal_34 + internal_35*sin_theta_1);
    const double internal_39 = internal_38*(cos_theta_1*internal_35 + internal_34*sin_theta_1);
    const double internal_44 = internal_41*(-cos_theta_2*internal_42 + internal_43*sin_theta_2);
    const double internal_47 = internal_46*(cos_theta_2*internal_43 + internal_42*sin_theta_2);
    const double internal_52 = internal_49*(-cos_theta_3*internal_50 + internal_51*sin_theta_3);
    const double internal_55 = internal_54*(cos_theta_3*internal_51 + internal_50*sin_theta_3);
    const double internal_60 = internal_57*(-cos_theta_4*internal_58 + internal_59*sin_theta_4);
    const double internal_63 = internal_62*(cos_theta_4*internal_59 + internal_58*sin_theta_4);
    const double internal_68 = internal_65*(-cos_theta_5*internal_66 + internal_67*sin_theta_5);
    const double internal_71 = internal_70*(cos_theta_5*internal_67 + internal_66*sin_theta_5);
    const double internal_76 = internal_73*(-cos_theta_6*internal_74 + internal_75*sin_theta_6);
    const double internal_79 = internal_78*(cos_theta_6*internal_75 + internal_74*sin_theta_6);
    const double internal_84 = internal_81*(-cos_theta_7*internal_82 + internal_83*sin_theta_7);
    const double internal_87 = internal_86*(cos_theta_7*internal_83 + internal_82*sin_theta_7);
    const double internal_22 = -internal_20;
    const double internal_13 = -internal_12;

    // Evaluation of Matrix jx
    jx(0, 1) = internal_0;
    jx(0, 2) = internal_2;
    jx(1, 1) = internal_3;
    jx(1, 2) = internal_1;
    jx(2, 1) = internal_4;
    jx(2, 2) = internal_6;
    jx(2, 3) = internal_13;
    jx(3, 1) = internal_14;
    jx(3, 2) = internal_5;
    jx(3, 3) = internal_12;
    jx(4, 1) = internal_15;
    jx(4, 2) = internal_17;
    jx(4, 3) = internal_20;
    jx(5, 1) = internal_21;
    jx(5, 2) = internal_16;
    jx(5, 3) = internal_22;
    jx(6, 4) = -1;
    jx(7, 4) = 1;
    jx(8, 0) = -1;
    jx(9, 0) = 1;
    jx(10, 5) = -1;
    jx(11, 5) = 1;
    jx(12, 6) = -1;
    jx(13, 6) = 1;
    jx(14, 3) = sin(theta - theta_ref);
    jx(19, 1) = -internal_28*internal_29 + internal_31*internal_32;
    jx(19, 2) = -internal_28*internal_32 - internal_29*internal_31;
    jx(19, 3) = -internal_28*(-internal_11*internal_29 + 2*internal_7*internal_8*sin_theta_0) - internal_31*(internal_11*internal_32 + internal_29*internal_9);
    jx(20, 1) = -internal_36*internal_37 + internal_39*internal_40;
    jx(20, 2) = -internal_36*internal_40 - internal_37*internal_39;
    jx(20, 3) = -internal_36*(-internal_11*internal_37 + 2*internal_7*internal_8*sin_theta_1) - internal_39*(internal_11*internal_40 + internal_37*internal_9);
    jx(21, 1) = -internal_44*internal_45 + internal_47*internal_48;
    jx(21, 2) = -internal_44*internal_48 - internal_45*internal_47;
    jx(21, 3) = -internal_44*(-internal_11*internal_45 + 2*internal_7*internal_8*sin_theta_2) - internal_47*(internal_11*internal_48 + internal_45*internal_9);
    jx(22, 1) = -internal_52*internal_53 + internal_55*internal_56;
    jx(22, 2) = -internal_52*internal_56 - internal_53*internal_55;
    jx(22, 3) = -internal_52*(-internal_11*internal_53 + 2*internal_7*internal_8*sin_theta_3) - internal_55*(internal_11*internal_56 + internal_53*internal_9);
    jx(23, 1) = -internal_60*internal_61 + internal_63*internal_64;
    jx(23, 2) = -internal_60*internal_64 - internal_61*internal_63;
    jx(23, 3) = -internal_60*(-internal_11*internal_61 + 2*internal_7*internal_8*sin_theta_4) - internal_63*(internal_11*internal_64 + internal_61*internal_9);
    jx(24, 1) = -internal_68*internal_69 + internal_71*internal_72;
    jx(24, 2) = -internal_68*internal_72 - internal_69*internal_71;
    jx(24, 3) = -internal_68*(-internal_11*internal_69 + 2*internal_7*internal_8*sin_theta_5) - internal_71*(internal_11*internal_72 + internal_69*internal_9);
    jx(25, 1) = -internal_76*internal_77 + internal_79*internal_80;
    jx(25, 2) = -internal_76*internal_80 - internal_77*internal_79;
    jx(25, 3) = -internal_76*(-internal_11*internal_77 + 2*internal_7*internal_8*sin_theta_6) - internal_79*(internal_11*internal_80 + internal_77*internal_9);
    jx(26, 1) = -internal_84*internal_85 + internal_87*internal_88;
    jx(26, 2) = -internal_84*internal_88 - internal_85*internal_87;
    jx(26, 3) = -internal_84*(-internal_11*internal_85 + 2*internal_7*internal_8*sin_theta_7) - internal_87*(internal_11*internal_88 + internal_85*internal_9);
    jx(27, 1) = -internal_29*internal_93 + internal_32*internal_94;
    jx(27, 2) = -internal_29*internal_94 - internal_32*internal_93;
    jx(27, 3) = -internal_93*(-internal_18*internal_32 + internal_19*internal_29) - internal_94*(-internal_18*internal_29 - internal_19*internal_32);
    jx(28, 1) = -internal_37*internal_97 + internal_40*internal_98;
    jx(28, 2) = -internal_37*internal_98 - internal_40*internal_97;
    jx(28, 3) = -internal_97*(-internal_18*internal_40 + internal_19*internal_37) - internal_98*(-internal_18*internal_37 - internal_19*internal_40);
    jx(29, 1) = -internal_101*internal_45 + internal_102*internal_48;
    jx(29, 2) = -internal_101*internal_48 - internal_102*internal_45;
    jx(29, 3) = -internal_101*(-internal_18*internal_48 + internal_19*internal_45) - internal_102*(-internal_18*internal_45 - internal_19*internal_48);
    jx(30, 1) = -internal_105*internal_53 + internal_106*internal_56;
    jx(30, 2) = -internal_105*internal_56 - internal_106*internal_53;
    jx(30, 3) = -internal_105*(-internal_18*internal_56 + internal_19*internal_53) - internal_106*(-internal_18*internal_53 - internal_19*internal_56);
    jx(31, 1) = -internal_109*internal_61 + internal_110*internal_64;
    jx(31, 2) = -internal_109*internal_64 - internal_110*internal_61;
    jx(31, 3) = -internal_109*(-internal_18*internal_64 + internal_19*internal_61) - internal_110*(-internal_18*internal_61 - internal_19*internal_64);
    jx(32, 1) = -internal_113*internal_69 + internal_114*internal_72;
    jx(32, 2) = -internal_113*internal_72 - internal_114*internal_69;
    jx(32, 3) = -internal_113*(-internal_18*internal_72 + internal_19*internal_69) - internal_114*(-internal_18*internal_69 - internal_19*internal_72);
    jx(33, 1) = -internal_117*internal_77 + internal_118*internal_80;
    jx(33, 2) = -internal_117*internal_80 - internal_118*internal_77;
    jx(33, 3) = -internal_117*(-internal_18*internal_80 + internal_19*internal_77) - internal_118*(-internal_18*internal_77 - internal_19*internal_80);
    jx(34, 1) = -internal_121*internal_85 + internal_122*internal_88;
    jx(34, 2) = -internal_121*internal_88 - internal_122*internal_85;
    jx(34, 3) = -internal_121*(-internal_18*internal_88 + internal_19*internal_85) - internal_122*(-internal_18*internal_85 - internal_19*internal_88);
    jx(35, 1) = internal_0;
    jx(35, 2) = internal_2;
    jx(36, 1) = internal_3;
    jx(36, 2) = internal_1;
    jx(37, 1) = internal_4;
    jx(37, 2) = internal_6;
    jx(37, 3) = internal_13;
    jx(38, 1) = internal_14;
    jx(38, 2) = internal_5;
    jx(38, 3) = internal_12;
    jx(39, 1) = internal_15;
    jx(39, 2) = internal_17;
    jx(39, 3) = internal_20;
    jx(40, 1) = internal_21;
    jx(40, 2) = internal_16;
    jx(40, 3) = internal_22;
    jx(41, 0) = 1;
    jx(42, 5) = 1;

  }

  virtual void updateJacobianDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& ju) const override {
    // Evaluation of Matrix ju
    ju(15, 0) = -1;
    ju(16, 0) = 1;
    ju(17, 1) = -1;
    ju(18, 1) = 1;
    ju(19, 5) = -1;
    ju(20, 6) = -1;
    ju(21, 7) = -1;
    ju(22, 8) = -1;
    ju(23, 9) = -1;
    ju(24, 10) = -1;
    ju(25, 11) = -1;
    ju(26, 12) = -1;
    ju(27, 5) = -1;
    ju(28, 6) = -1;
    ju(29, 7) = -1;
    ju(30, 8) = -1;
    ju(31, 9) = -1;
    ju(32, 10) = -1;
    ju(33, 11) = -1;
    ju(34, 12) = -1;
    ju(35, 2) = -1;
    ju(36, 2) = -1;
    ju(37, 2) = -1;
    ju(38, 2) = -1;
    ju(39, 2) = -1;
    ju(40, 2) = -1;
    ju(41, 3) = -1;
    ju(42, 4) = -1;

  }

  virtual void updateDeltaEvaluation(const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::VectorXd& dx, const Eigen::VectorXd& du, Eigen::VectorXd& dg) const override {
    // Evaluation of Vector dg
    dg(0) = dx(1)*g_x(0, 1) + dx(2)*g_x(0, 2);
    dg(1) = dx(1)*g_x(1, 1) + dx(2)*g_x(1, 2);
    dg(2) = dx(1)*g_x(2, 1) + dx(2)*g_x(2, 2) + dx(3)*g_x(2, 3);
    dg(3) = dx(1)*g_x(3, 1) + dx(2)*g_x(3, 2) + dx(3)*g_x(3, 3);
    dg(4) = dx(1)*g_x(4, 1) + dx(2)*g_x(4, 2) + dx(3)*g_x(4, 3);
    dg(5) = dx(1)*g_x(5, 1) + dx(2)*g_x(5, 2) + dx(3)*g_x(5, 3);
    dg(6) = dx(4)*g_x(6, 4);
    dg(7) = dx(4)*g_x(7, 4);
    dg(8) = dx(0)*g_x(8, 0);
    dg(9) = dx(0)*g_x(9, 0);
    dg(10) = dx(5)*g_x(10, 5);
    dg(11) = dx(5)*g_x(11, 5);
    dg(12) = dx(6)*g_x(12, 6);
    dg(13) = dx(6)*g_x(13, 6);
    dg(14) = dx(3)*g_x(14, 3);
    dg(15) = du(0)*g_u(15, 0);
    dg(16) = du(0)*g_u(16, 0);
    dg(17) = du(1)*g_u(17, 1);
    dg(18) = du(1)*g_u(18, 1);
    dg(19) = du(5)*g_u(19, 5) + dx(1)*g_x(19, 1) + dx(2)*g_x(19, 2) + dx(3)*g_x(19, 3);
    dg(20) = du(6)*g_u(20, 6) + dx(1)*g_x(20, 1) + dx(2)*g_x(20, 2) + dx(3)*g_x(20, 3);
    dg(21) = du(7)*g_u(21, 7) + dx(1)*g_x(21, 1) + dx(2)*g_x(21, 2) + dx(3)*g_x(21, 3);
    dg(22) = du(8)*g_u(22, 8) + dx(1)*g_x(22, 1) + dx(2)*g_x(22, 2) + dx(3)*g_x(22, 3);
    dg(23) = du(9)*g_u(23, 9) + dx(1)*g_x(23, 1) + dx(2)*g_x(23, 2) + dx(3)*g_x(23, 3);
    dg(24) = du(10)*g_u(24, 10) + dx(1)*g_x(24, 1) + dx(2)*g_x(24, 2) + dx(3)*g_x(24, 3);
    dg(25) = du(11)*g_u(25, 11) + dx(1)*g_x(25, 1) + dx(2)*g_x(25, 2) + dx(3)*g_x(25, 3);
    dg(26) = du(12)*g_u(26, 12) + dx(1)*g_x(26, 1) + dx(2)*g_x(26, 2) + dx(3)*g_x(26, 3);
    dg(27) = du(5)*g_u(27, 5) + dx(1)*g_x(27, 1) + dx(2)*g_x(27, 2) + dx(3)*g_x(27, 3);
    dg(28) = du(6)*g_u(28, 6) + dx(1)*g_x(28, 1) + dx(2)*g_x(28, 2) + dx(3)*g_x(28, 3);
    dg(29) = du(7)*g_u(29, 7) + dx(1)*g_x(29, 1) + dx(2)*g_x(29, 2) + dx(3)*g_x(29, 3);
    dg(30) = du(8)*g_u(30, 8) + dx(1)*g_x(30, 1) + dx(2)*g_x(30, 2) + dx(3)*g_x(30, 3);
    dg(31) = du(9)*g_u(31, 9) + dx(1)*g_x(31, 1) + dx(2)*g_x(31, 2) + dx(3)*g_x(31, 3);
    dg(32) = du(10)*g_u(32, 10) + dx(1)*g_x(32, 1) + dx(2)*g_x(32, 2) + dx(3)*g_x(32, 3);
    dg(33) = du(11)*g_u(33, 11) + dx(1)*g_x(33, 1) + dx(2)*g_x(33, 2) + dx(3)*g_x(33, 3);
    dg(34) = du(12)*g_u(34, 12) + dx(1)*g_x(34, 1) + dx(2)*g_x(34, 2) + dx(3)*g_x(34, 3);
    dg(35) = du(2)*g_u(35, 2) + dx(1)*g_x(35, 1) + dx(2)*g_x(35, 2);
    dg(36) = du(2)*g_u(36, 2) + dx(1)*g_x(36, 1) + dx(2)*g_x(36, 2);
    dg(37) = du(2)*g_u(37, 2) + dx(1)*g_x(37, 1) + dx(2)*g_x(37, 2) + dx(3)*g_x(37, 3);
    dg(38) = du(2)*g_u(38, 2) + dx(1)*g_x(38, 1) + dx(2)*g_x(38, 2) + dx(3)*g_x(38, 3);
    dg(39) = du(2)*g_u(39, 2) + dx(1)*g_x(39, 1) + dx(2)*g_x(39, 2) + dx(3)*g_x(39, 3);
    dg(40) = du(2)*g_u(40, 2) + dx(1)*g_x(40, 1) + dx(2)*g_x(40, 2) + dx(3)*g_x(40, 3);
    dg(41) = du(3)*g_u(41, 3) + dx(0)*g_x(41, 0);
    dg(42) = du(4)*g_u(42, 4) + dx(5)*g_x(42, 5);

  }

};

template <>
class SpatiotemporalPlannerModelConstraints<StageType::INITIAL> : public Constraints {
 public:
  SpatiotemporalPlannerModelConstraints() : Constraints(7, 13, 0, 4, 0) {
    // Evaluation of Matrix jacobian_.ju
    jacobian_.ju(0, 0) = -1;
    jacobian_.ju(1, 0) = 1;
    jacobian_.ju(2, 1) = -1;
    jacobian_.ju(3, 1) = 1;
    exprs_[0] = "DSteerLowerBound <= dsteer";
    exprs_[1] = "dsteer <= DSteerUpperBound";
    exprs_[2] = "JHardLowerBound <= jerk";
    exprs_[3] = "jerk <= JHardUpperBound";

  }

  virtual ~SpatiotemporalPlannerModelConstraints() = default;

  virtual Type type() const override { return Type::INEQUALITY; }
  virtual void updateEvaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::VectorXd& values) const override {
    // Determine stage variables
    const auto& dsteer = ctrls(0);
    const auto& jerk = ctrls(1);
    const auto& DSteerLowerBound = params(73);
    const auto& DSteerUpperBound = params(74);
    const auto& JHardLowerBound = params(81);
    const auto& JHardUpperBound = params(82);

    // Determine internal variables
    const double internal_0 = -dsteer;
    const double internal_1 = -jerk;

    // Evaluation of Vector values
    values(0) = DSteerLowerBound + internal_0;
    values(1) = -DSteerUpperBound - internal_0;
    values(2) = JHardLowerBound + internal_1;
    values(3) = -JHardUpperBound - internal_1;

  }

  virtual void updateJacobian(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& jx, Eigen::MatrixXd& ju) const override {
    // Evaluation of Matrix ju
    ju(0, 0) = -1;
    ju(1, 0) = 1;
    ju(2, 1) = -1;
    ju(3, 1) = 1;

  }

  virtual void updateCtrlJacobian(const Eigen::VectorXd& ctrls, Eigen::MatrixXd& ju) const override {
    // Evaluation of Matrix ju
    ju(0, 0) = -1;
    ju(1, 0) = 1;
    ju(2, 1) = -1;
    ju(3, 1) = 1;

  }

  virtual void updateJacobianDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& ju) const override {
    // Evaluation of Matrix ju
    ju(0, 0) = -1;
    ju(1, 0) = 1;
    ju(2, 1) = -1;
    ju(3, 1) = 1;

  }

  virtual void updateDeltaEvaluation(const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::VectorXd& dx, const Eigen::VectorXd& du, Eigen::VectorXd& dg) const override {
    // Evaluation of Vector dg
    dg(0) = du(0)*g_u(0, 0);
    dg(1) = du(0)*g_u(1, 0);
    dg(2) = du(1)*g_u(2, 1);
    dg(3) = du(1)*g_u(3, 1);

  }

};

template <>
class SpatiotemporalPlannerModelConstraints<StageType::TERMINAL> : public Constraints {
 public:
  SpatiotemporalPlannerModelConstraints() : Constraints(7, 13, 15, 0, 0) {
    // Evaluation of Matrix jacobian_.jx
    jacobian_.jx(6, 4) = -1;
    jacobian_.jx(7, 4) = 1;
    jacobian_.jx(8, 0) = -1;
    jacobian_.jx(9, 0) = 1;
    jacobian_.jx(10, 5) = -1;
    jacobian_.jx(11, 5) = 1;
    jacobian_.jx(12, 6) = -1;
    jacobian_.jx(13, 6) = 1;

    exprs_[0] = "LHardLowerBound <= -(x - x_ref)*sin(theta_ref) + (y - y_ref)*cos(theta_ref)";
    exprs_[1] = "-(x - x_ref)*sin(theta_ref) + (y - y_ref)*cos(theta_ref) <= LHardUpperBound";
    exprs_[2] = "LFHardLowerBound <= -(x - xf_ref + (front_overhang + wheelbase)*cos(theta))*sin(thetaf_ref) + (y - yf_ref + (front_overhang + wheelbase)*sin(theta))*cos(thetaf_ref)";
    exprs_[3] = "-(x - xf_ref + (front_overhang + wheelbase)*cos(theta))*sin(thetaf_ref) + (y - yf_ref + (front_overhang + wheelbase)*sin(theta))*cos(thetaf_ref) <= LFHardUpperBound";
    exprs_[4] = "LRHardLowerBound <= (-rear_overhang*sin(theta) + y - yr_ref)*cos(thetar_ref) - (-rear_overhang*cos(theta) + x - xr_ref)*sin(thetar_ref)";
    exprs_[5] = "(-rear_overhang*sin(theta) + y - yr_ref)*cos(thetar_ref) - (-rear_overhang*cos(theta) + x - xr_ref)*sin(thetar_ref) <= LRHardUpperBound";
    exprs_[6] = "SteerLowerBound <= steer";
    exprs_[7] = "steer <= SteerUpperBound";
    exprs_[8] = "SHardLowerBound <= s";
    exprs_[9] = "s <= SHardUpperBound";
    exprs_[10] = "VHardLowerBound <= v";
    exprs_[11] = "v <= VHardUpperBound";
    exprs_[12] = "AHardLowerBound <= a";
    exprs_[13] = "a <= AHardUpperBound";
    exprs_[14] = "cos(theta - theta_ref) >= 0.0";

  }

  virtual ~SpatiotemporalPlannerModelConstraints() = default;

  virtual Type type() const override { return Type::INEQUALITY; }
  virtual void updateEvaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::VectorXd& values) const override {
    // Determine stage variables
    const auto& s = states(0);
    const auto& theta = states(3);
    const auto& steer = states(4);
    const auto& v = states(5);
    const auto& a = states(6);
    const auto& theta_ref = params(2);
    const auto& LHardLowerBound = params(59);
    const auto& LHardUpperBound = params(60);
    const auto& LFHardLowerBound = params(61);
    const auto& LFHardUpperBound = params(62);
    const auto& LRHardLowerBound = params(63);
    const auto& LRHardUpperBound = params(64);
    const auto& SteerLowerBound = params(71);
    const auto& SteerUpperBound = params(72);
    const auto& SHardLowerBound = params(75);
    const auto& SHardUpperBound = params(76);
    const auto& VHardLowerBound = params(77);
    const auto& VHardUpperBound = params(78);
    const auto& AHardLowerBound = params(79);
    const auto& AHardUpperBound = params(80);

    // Determine global variables
    const auto& l = globals(3);
    const auto& blr = globals(6);
    const auto& blf = globals(7);

    // Determine internal variables
    const double internal_4 = -s;
    const double internal_3 = -steer;
    const double internal_5 = -v;
    const double internal_6 = -a;
    const double internal_0 = -l;
    const double internal_2 = -blr;
    const double internal_1 = -blf;

    // Evaluation of Vector values
    values(0) = LHardLowerBound + internal_0;
    values(1) = -LHardUpperBound - internal_0;
    values(2) = LFHardLowerBound + internal_1;
    values(3) = -LFHardUpperBound - internal_1;
    values(4) = LRHardLowerBound + internal_2;
    values(5) = -LRHardUpperBound - internal_2;
    values(6) = SteerLowerBound + internal_3;
    values(7) = -SteerUpperBound - internal_3;
    values(8) = SHardLowerBound + internal_4;
    values(9) = -SHardUpperBound - internal_4;
    values(10) = VHardLowerBound + internal_5;
    values(11) = -VHardUpperBound - internal_5;
    values(12) = AHardLowerBound + internal_6;
    values(13) = -AHardUpperBound - internal_6;
    values(14) = -cos(theta - theta_ref);

  }

  virtual void updateJacobian(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& jx, Eigen::MatrixXd& ju) const override {
    // Determine stage variables
    const auto& theta = states(3);
    const auto& theta_ref = params(2);
    const auto& wheelbase = params(14);
    const auto& front_overhang = params(15);
    const auto& rear_overhang = params(16);
    const auto& thetaf_ref = params(47);
    const auto& thetar_ref = params(50);

    // Determine internal variables
    const double internal_5 = cos(theta);
    const double internal_6 = sin(theta);
    const double internal_0 = sin(theta_ref);
    const double internal_1 = cos(theta_ref);
    const double internal_4 = front_overhang + wheelbase;
    const double internal_2 = sin(thetaf_ref);
    const double internal_3 = cos(thetaf_ref);
    const double internal_8 = sin(thetar_ref);
    const double internal_9 = cos(thetar_ref);
    const double internal_7 = internal_2*internal_4*internal_6 + internal_3*internal_4*internal_5;
    const double internal_10 = internal_5*internal_9*rear_overhang + internal_6*internal_8*rear_overhang;

    // Evaluation of Matrix jx
    jx(0, 1) = internal_0;
    jx(0, 2) = -internal_1;
    jx(1, 1) = -internal_0;
    jx(1, 2) = internal_1;
    jx(2, 1) = internal_2;
    jx(2, 2) = -internal_3;
    jx(2, 3) = -internal_7;
    jx(3, 1) = -internal_2;
    jx(3, 2) = internal_3;
    jx(3, 3) = internal_7;
    jx(4, 1) = internal_8;
    jx(4, 2) = -internal_9;
    jx(4, 3) = internal_10;
    jx(5, 1) = -internal_8;
    jx(5, 2) = internal_9;
    jx(5, 3) = -internal_10;
    jx(6, 4) = -1;
    jx(7, 4) = 1;
    jx(8, 0) = -1;
    jx(9, 0) = 1;
    jx(10, 5) = -1;
    jx(11, 5) = 1;
    jx(12, 6) = -1;
    jx(13, 6) = 1;
    jx(14, 3) = sin(theta - theta_ref);


  }

  virtual void updateStateJacobian(const Eigen::VectorXd& states, Eigen::MatrixXd& jx) const override {
    // Determine stage variables
    const auto& theta = states(3);
    const auto& theta_ref = params(2);
    const auto& wheelbase = params(14);
    const auto& front_overhang = params(15);
    const auto& rear_overhang = params(16);
    const auto& thetaf_ref = params(47);
    const auto& thetar_ref = params(50);

    // Determine internal variables
    const double internal_5 = cos(theta);
    const double internal_6 = sin(theta);
    const double internal_0 = sin(theta_ref);
    const double internal_1 = cos(theta_ref);
    const double internal_4 = front_overhang + wheelbase;
    const double internal_2 = sin(thetaf_ref);
    const double internal_3 = cos(thetaf_ref);
    const double internal_8 = sin(thetar_ref);
    const double internal_9 = cos(thetar_ref);
    const double internal_7 = internal_2*internal_4*internal_6 + internal_3*internal_4*internal_5;
    const double internal_10 = internal_5*internal_9*rear_overhang + internal_6*internal_8*rear_overhang;

    // Evaluation of Matrix jx
    jx(0, 1) = internal_0;
    jx(0, 2) = -internal_1;
    jx(1, 1) = -internal_0;
    jx(1, 2) = internal_1;
    jx(2, 1) = internal_2;
    jx(2, 2) = -internal_3;
    jx(2, 3) = -internal_7;
    jx(3, 1) = -internal_2;
    jx(3, 2) = internal_3;
    jx(3, 3) = internal_7;
    jx(4, 1) = internal_8;
    jx(4, 2) = -internal_9;
    jx(4, 3) = internal_10;
    jx(5, 1) = -internal_8;
    jx(5, 2) = internal_9;
    jx(5, 3) = -internal_10;
    jx(6, 4) = -1;
    jx(7, 4) = 1;
    jx(8, 0) = -1;
    jx(9, 0) = 1;
    jx(10, 5) = -1;
    jx(11, 5) = 1;
    jx(12, 6) = -1;
    jx(13, 6) = 1;
    jx(14, 3) = sin(theta - theta_ref);

  }

  virtual void updateJacobianDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& jx) const override {
    // Determine stage variables
    const auto& theta = states(3);
    const auto& theta_ref = params(2);
    const auto& wheelbase = params(14);
    const auto& front_overhang = params(15);
    const auto& rear_overhang = params(16);
    const auto& thetaf_ref = params(47);
    const auto& thetar_ref = params(50);

    // Determine internal variables
    const double internal_5 = cos(theta);
    const double internal_6 = sin(theta);
    const double internal_0 = sin(theta_ref);
    const double internal_1 = cos(theta_ref);
    const double internal_4 = front_overhang + wheelbase;
    const double internal_2 = sin(thetaf_ref);
    const double internal_3 = cos(thetaf_ref);
    const double internal_8 = sin(thetar_ref);
    const double internal_9 = cos(thetar_ref);
    const double internal_7 = internal_2*internal_4*internal_6 + internal_3*internal_4*internal_5;
    const double internal_10 = internal_5*internal_9*rear_overhang + internal_6*internal_8*rear_overhang;

    // Evaluation of Matrix jx
    jx(0, 1) = internal_0;
    jx(0, 2) = -internal_1;
    jx(1, 1) = -internal_0;
    jx(1, 2) = internal_1;
    jx(2, 1) = internal_2;
    jx(2, 2) = -internal_3;
    jx(2, 3) = -internal_7;
    jx(3, 1) = -internal_2;
    jx(3, 2) = internal_3;
    jx(3, 3) = internal_7;
    jx(4, 1) = internal_8;
    jx(4, 2) = -internal_9;
    jx(4, 3) = internal_10;
    jx(5, 1) = -internal_8;
    jx(5, 2) = internal_9;
    jx(5, 3) = -internal_10;
    jx(6, 4) = -1;
    jx(7, 4) = 1;
    jx(8, 0) = -1;
    jx(9, 0) = 1;
    jx(10, 5) = -1;
    jx(11, 5) = 1;
    jx(12, 6) = -1;
    jx(13, 6) = 1;
    jx(14, 3) = sin(theta - theta_ref);

  }

  virtual void updateDeltaEvaluation(const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::VectorXd& dx, const Eigen::VectorXd& du, Eigen::VectorXd& dg) const override {
    // Evaluation of Vector dg
    dg(0) = dx(1)*g_x(0, 1) + dx(2)*g_x(0, 2);
    dg(1) = dx(1)*g_x(1, 1) + dx(2)*g_x(1, 2);
    dg(2) = dx(1)*g_x(2, 1) + dx(2)*g_x(2, 2) + dx(3)*g_x(2, 3);
    dg(3) = dx(1)*g_x(3, 1) + dx(2)*g_x(3, 2) + dx(3)*g_x(3, 3);
    dg(4) = dx(1)*g_x(4, 1) + dx(2)*g_x(4, 2) + dx(3)*g_x(4, 3);
    dg(5) = dx(1)*g_x(5, 1) + dx(2)*g_x(5, 2) + dx(3)*g_x(5, 3);
    dg(6) = dx(4)*g_x(6, 4);
    dg(7) = dx(4)*g_x(7, 4);
    dg(8) = dx(0)*g_x(8, 0);
    dg(9) = dx(0)*g_x(9, 0);
    dg(10) = dx(5)*g_x(10, 5);
    dg(11) = dx(5)*g_x(11, 5);
    dg(12) = dx(6)*g_x(12, 6);
    dg(13) = dx(6)*g_x(13, 6);
    dg(14) = dx(3)*g_x(14, 3);

  }

};

template <StageType Ttype>
class SpatiotemporalPlannerModelStateOnlyEqualities : public Constraints {
 public:
  SpatiotemporalPlannerModelStateOnlyEqualities() : Constraints(7, 13, 0, 0, 0) {
  }

  virtual ~SpatiotemporalPlannerModelStateOnlyEqualities() = default;

  virtual Type type() const override { return Type::STATE_ONLY_EQUALITY; }
};

template <>
class SpatiotemporalPlannerModelStateOnlyEqualities<StageType::INITIAL> : public Constraints {
 public:
  SpatiotemporalPlannerModelStateOnlyEqualities() : Constraints(7, 13, 7, 0, 0) {
    // Evaluation of SymmetricMatrix jacobian_.jx
    jacobian_.jx(0, 0) = 1;
    jacobian_.jx(1, 1) = 1;
    jacobian_.jx(2, 2) = 1;
    jacobian_.jx(3, 3) = 1;
    jacobian_.jx(4, 4) = 1;
    jacobian_.jx(5, 5) = 1;
    jacobian_.jx(6, 6) = 1;

    exprs_[0] = "s = 0";
    exprs_[1] = "x = 0";
    exprs_[2] = "y = 0";
    exprs_[3] = "theta = 0";
    exprs_[4] = "steer = 0";
    exprs_[5] = "v = 0";
    exprs_[6] = "a = 0";
  }

  virtual ~SpatiotemporalPlannerModelStateOnlyEqualities() = default;

  virtual Type type() const override { return Type::STATE_ONLY_EQUALITY; }
  virtual void updateEvaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::VectorXd& values) const override {
    // Determine stage variables
    const auto& s = states(0);
    const auto& x = states(1);
    const auto& y = states(2);
    const auto& theta = states(3);
    const auto& steer = states(4);
    const auto& v = states(5);
    const auto& a = states(6);

    // Evaluation of Vector values
    values(0) = s;
    values(1) = x;
    values(2) = y;
    values(3) = theta;
    values(4) = steer;
    values(5) = v;
    values(6) = a;

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

  }

};

template <>
class SpatiotemporalPlannerModelStateOnlyEqualities<StageType::TERMINAL> : public Constraints {
 public:
  SpatiotemporalPlannerModelStateOnlyEqualities() : Constraints(7, 13, 0, 0, 0) {
  }

  virtual ~SpatiotemporalPlannerModelStateOnlyEqualities() = default;

  virtual Type type() const override { return Type::STATE_ONLY_EQUALITY; }
};

}  // namespace gpal::pnc::planning