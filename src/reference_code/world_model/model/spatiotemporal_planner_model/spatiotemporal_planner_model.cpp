#include "spatiotemporal_planner_model.h"

namespace gpal::pnc::planning {

SpatiotemporalPlannerModel::SpatiotemporalPlannerModel() : OptimalControlProblem("SpatiotemporalPlannerModel", 7, 13, 11, 193, 25) {
  default_state_.emplace("s");
  default_state_.emplace("x");
  default_state_.emplace("y");
  default_state_.emplace("theta");
  default_state_.emplace("steer");
  default_state_.emplace("v");
  default_state_.emplace("a");

  default_ctrl_.emplace("dsteer");
  default_ctrl_.emplace("jerk");
  default_ctrl_.emplace("l_slack");
  default_ctrl_.emplace("s_slack");
  default_ctrl_.emplace("v_slack");
  default_ctrl_.emplace("obs_0_slack");
  default_ctrl_.emplace("obs_1_slack");
  default_ctrl_.emplace("obs_2_slack");
  default_ctrl_.emplace("obs_3_slack");
  default_ctrl_.emplace("obs_4_slack");
  default_ctrl_.emplace("obs_5_slack");
  default_ctrl_.emplace("obs_6_slack");
  default_ctrl_.emplace("obs_7_slack");

  default_param_.emplace("x_ref", 0.0);
  default_param_.emplace("y_ref", 0.0);
  default_param_.emplace("theta_ref", 0.0);
  default_param_.emplace("kappa_ref", 0.0);
  default_param_.emplace("steer_ref", 0.0);
  default_param_.emplace("s_coarse", 0.0);
  default_param_.emplace("v_coarse", 0.0);
  default_param_.emplace("a_coarse", 0.0);
  default_param_.emplace("l_terminal", 0.0);
  default_param_.emplace("theta_terminal", 0.0);
  default_param_.emplace("s_terminal", 0.0);
  default_param_.emplace("v_terminal", 0.0);
  default_param_.emplace("l_offset", 0.0);
  default_param_.emplace("a_offset", 0.0);
  default_param_.emplace("wheelbase", 3.0);
  default_param_.emplace("front_overhang", 1.0);
  default_param_.emplace("rear_overhang", 1.0);
  default_param_.emplace("length", 5.0);
  default_param_.emplace("width", 2.0);
  default_param_.emplace("weight_prev_steer", 1.0);
  default_param_.emplace("prev_steer", 1.0);
  default_param_.emplace("l_ref_weight", 1.0);
  default_param_.emplace("theta_ref_weight", 1.0);
  default_param_.emplace("steer_weight", 1.0);
  default_param_.emplace("dsteer_weight", 1.0);
  default_param_.emplace("s_coarse_weight", 0.0);
  default_param_.emplace("v_coarse_weight", 1.0);
  default_param_.emplace("a_weight", 1.0);
  default_param_.emplace("jerk_weight", 1.0);
  default_param_.emplace("obs_0_weight", 1.0);
  default_param_.emplace("obs_1_weight", 1.0);
  default_param_.emplace("obs_2_weight", 1.0);
  default_param_.emplace("obs_3_weight", 1.0);
  default_param_.emplace("obs_4_weight", 1.0);
  default_param_.emplace("obs_5_weight", 1.0);
  default_param_.emplace("obs_6_weight", 1.0);
  default_param_.emplace("obs_7_weight", 1.0);
  default_param_.emplace("l_slack_weight", 1.0);
  default_param_.emplace("s_slack_weight", 1.0);
  default_param_.emplace("v_slack_weight", 1.0);
  default_param_.emplace("terminal_l_weight", 1.0);
  default_param_.emplace("terminal_theta_weight", 1.0);
  default_param_.emplace("terminal_s_weight", 1.0);
  default_param_.emplace("terminal_v_weight", 1.0);
  default_param_.emplace("terminal_a_weight", 1.0);
  default_param_.emplace("xf_ref", 0.0);
  default_param_.emplace("yf_ref", 0.0);
  default_param_.emplace("thetaf_ref", 0.0);
  default_param_.emplace("xr_ref", 0.0);
  default_param_.emplace("yr_ref", 0.0);
  default_param_.emplace("thetar_ref", 0.0);
  default_param_.emplace("Obs_0_AvoidedBound", 1.0);
  default_param_.emplace("Obs_1_AvoidedBound", 1.0);
  default_param_.emplace("Obs_2_AvoidedBound", 1.0);
  default_param_.emplace("Obs_3_AvoidedBound", 1.0);
  default_param_.emplace("Obs_4_AvoidedBound", 1.0);
  default_param_.emplace("Obs_5_AvoidedBound", 1.0);
  default_param_.emplace("Obs_6_AvoidedBound", 1.0);
  default_param_.emplace("Obs_7_AvoidedBound", 1.0);
  default_param_.emplace("LHardLowerBound", -20.0);
  default_param_.emplace("LHardUpperBound", 20.0);
  default_param_.emplace("LFHardLowerBound", -20.0);
  default_param_.emplace("LFHardUpperBound", 20.0);
  default_param_.emplace("LRHardLowerBound", -20.0);
  default_param_.emplace("LRHardUpperBound", 20.0);
  default_param_.emplace("LSoftLowerBound", -20.0);
  default_param_.emplace("LSoftUpperBound", 20.0);
  default_param_.emplace("LFSoftLowerBound", -20.0);
  default_param_.emplace("LFSoftUpperBound", 20.0);
  default_param_.emplace("LRSoftLowerBound", -20.0);
  default_param_.emplace("LRSoftUpperBound", 20.0);
  default_param_.emplace("SteerLowerBound", -0.436);
  default_param_.emplace("SteerUpperBound", 0.436);
  default_param_.emplace("DSteerLowerBound", -0.436);
  default_param_.emplace("DSteerUpperBound", 0.436);
  default_param_.emplace("SHardLowerBound", 0.0);
  default_param_.emplace("SHardUpperBound", 300.0);
  default_param_.emplace("VHardLowerBound", 0.0);
  default_param_.emplace("VHardUpperBound", 40.0);
  default_param_.emplace("AHardLowerBound", -6.0);
  default_param_.emplace("AHardUpperBound", 3.0);
  default_param_.emplace("JHardLowerBound", -8.0);
  default_param_.emplace("JHardUpperBound", 3.0);
  default_param_.emplace("SSoftLowerBound", 0.0);
  default_param_.emplace("SSoftUpperBound", 300.0);
  default_param_.emplace("VSoftLowerBound", 0.0);
  default_param_.emplace("VSoftUpperBound", 40.0);
  default_param_.emplace("ASoftLowerBound", -6.0);
  default_param_.emplace("ASoftUpperBound", 3.0);
  default_param_.emplace("a_0", 5.0);
  default_param_.emplace("a_0_square", 25.0);
  default_param_.emplace("b_0", 2.0);
  default_param_.emplace("b_0_square", 4.0);
  default_param_.emplace("x_0", 0.0);
  default_param_.emplace("y_0", 0.0);
  default_param_.emplace("theta_0", 0.0);
  default_param_.emplace("cos_theta_0", 1.0);
  default_param_.emplace("sin_theta_0", 0.0);
  default_param_.emplace("v_0", 0.0);
  default_param_.emplace("length_0", 5.0);
  default_param_.emplace("width_0", 2.0);
  default_param_.emplace("a_1", 5.0);
  default_param_.emplace("a_1_square", 25.0);
  default_param_.emplace("b_1", 2.0);
  default_param_.emplace("b_1_square", 4.0);
  default_param_.emplace("x_1", 0.0);
  default_param_.emplace("y_1", 0.0);
  default_param_.emplace("theta_1", 0.0);
  default_param_.emplace("cos_theta_1", 1.0);
  default_param_.emplace("sin_theta_1", 0.0);
  default_param_.emplace("v_1", 0.0);
  default_param_.emplace("length_1", 5.0);
  default_param_.emplace("width_1", 2.0);
  default_param_.emplace("a_2", 5.0);
  default_param_.emplace("a_2_square", 25.0);
  default_param_.emplace("b_2", 2.0);
  default_param_.emplace("b_2_square", 4.0);
  default_param_.emplace("x_2", 0.0);
  default_param_.emplace("y_2", 0.0);
  default_param_.emplace("theta_2", 0.0);
  default_param_.emplace("cos_theta_2", 1.0);
  default_param_.emplace("sin_theta_2", 0.0);
  default_param_.emplace("v_2", 0.0);
  default_param_.emplace("length_2", 5.0);
  default_param_.emplace("width_2", 2.0);
  default_param_.emplace("a_3", 5.0);
  default_param_.emplace("a_3_square", 25.0);
  default_param_.emplace("b_3", 2.0);
  default_param_.emplace("b_3_square", 4.0);
  default_param_.emplace("x_3", 0.0);
  default_param_.emplace("y_3", 0.0);
  default_param_.emplace("theta_3", 0.0);
  default_param_.emplace("cos_theta_3", 1.0);
  default_param_.emplace("sin_theta_3", 0.0);
  default_param_.emplace("v_3", 0.0);
  default_param_.emplace("length_3", 5.0);
  default_param_.emplace("width_3", 2.0);
  default_param_.emplace("a_4", 5.0);
  default_param_.emplace("a_4_square", 25.0);
  default_param_.emplace("b_4", 2.0);
  default_param_.emplace("b_4_square", 4.0);
  default_param_.emplace("x_4", 0.0);
  default_param_.emplace("y_4", 0.0);
  default_param_.emplace("theta_4", 0.0);
  default_param_.emplace("cos_theta_4", 1.0);
  default_param_.emplace("sin_theta_4", 0.0);
  default_param_.emplace("v_4", 0.0);
  default_param_.emplace("length_4", 5.0);
  default_param_.emplace("width_4", 2.0);
  default_param_.emplace("a_5", 5.0);
  default_param_.emplace("a_5_square", 25.0);
  default_param_.emplace("b_5", 2.0);
  default_param_.emplace("b_5_square", 4.0);
  default_param_.emplace("x_5", 0.0);
  default_param_.emplace("y_5", 0.0);
  default_param_.emplace("theta_5", 0.0);
  default_param_.emplace("cos_theta_5", 1.0);
  default_param_.emplace("sin_theta_5", 0.0);
  default_param_.emplace("v_5", 0.0);
  default_param_.emplace("length_5", 5.0);
  default_param_.emplace("width_5", 2.0);
  default_param_.emplace("a_6", 5.0);
  default_param_.emplace("a_6_square", 25.0);
  default_param_.emplace("b_6", 2.0);
  default_param_.emplace("b_6_square", 4.0);
  default_param_.emplace("x_6", 0.0);
  default_param_.emplace("y_6", 0.0);
  default_param_.emplace("theta_6", 0.0);
  default_param_.emplace("cos_theta_6", 1.0);
  default_param_.emplace("sin_theta_6", 0.0);
  default_param_.emplace("v_6", 0.0);
  default_param_.emplace("length_6", 5.0);
  default_param_.emplace("width_6", 2.0);
  default_param_.emplace("a_7", 5.0);
  default_param_.emplace("a_7_square", 25.0);
  default_param_.emplace("b_7", 2.0);
  default_param_.emplace("b_7_square", 4.0);
  default_param_.emplace("x_7", 0.0);
  default_param_.emplace("y_7", 0.0);
  default_param_.emplace("theta_7", 0.0);
  default_param_.emplace("cos_theta_7", 1.0);
  default_param_.emplace("sin_theta_7", 0.0);
  default_param_.emplace("v_7", 0.0);
  default_param_.emplace("length_7", 5.0);
  default_param_.emplace("width_7", 2.0);
  default_param_.emplace("s_scale", 20.0);
  default_param_.emplace("v_scale", 10.0);
  default_param_.emplace("a_scale", 5.0);
  default_param_.emplace("jerk_scale", 5.0);
  default_param_.emplace("l_scale", 1.0);
  default_param_.emplace("theta_scale", 1.0);
  default_param_.emplace("steer_scale", 1.0);
  default_param_.emplace("dsteer_scale", 1.0);

  default_global_.emplace("kappa");
  default_global_.emplace("x_r_circle");
  default_global_.emplace("y_r_circle");
  default_global_.emplace("l");
  default_global_.emplace("x_f_circle");
  default_global_.emplace("y_f_circle");
  default_global_.emplace("blr");
  default_global_.emplace("blf");
  default_global_.emplace("ds");
  default_global_.emplace("r_dis_2_obs_0");
  default_global_.emplace("r_dis_2_obs_1");
  default_global_.emplace("r_dis_2_obs_2");
  default_global_.emplace("r_dis_2_obs_3");
  default_global_.emplace("r_dis_2_obs_4");
  default_global_.emplace("r_dis_2_obs_5");
  default_global_.emplace("r_dis_2_obs_6");
  default_global_.emplace("r_dis_2_obs_7");
  default_global_.emplace("f_dis_2_obs_0");
  default_global_.emplace("f_dis_2_obs_1");
  default_global_.emplace("f_dis_2_obs_2");
  default_global_.emplace("f_dis_2_obs_3");
  default_global_.emplace("f_dis_2_obs_4");
  default_global_.emplace("f_dis_2_obs_5");
  default_global_.emplace("f_dis_2_obs_6");
  default_global_.emplace("f_dis_2_obs_7");
  default_global_.setUpdater(std::make_shared<OcpVariable::Updater>([](const OcpStage& stage, OcpVariable* ptr_global) {
    // Determine stage variables
    const auto& x = stage.x(1);
    const auto& y = stage.x(2);
    const auto& theta = stage.x(3);
    const auto& steer = stage.x(4);
    const auto& x_ref = stage.params(0);
    const auto& y_ref = stage.params(1);
    const auto& theta_ref = stage.params(2);
    const auto& kappa_ref = stage.params(3);
    const auto& wheelbase = stage.params(14);
    const auto& front_overhang = stage.params(15);
    const auto& rear_overhang = stage.params(16);
    const auto& xf_ref = stage.params(45);
    const auto& yf_ref = stage.params(46);
    const auto& thetaf_ref = stage.params(47);
    const auto& xr_ref = stage.params(48);
    const auto& yr_ref = stage.params(49);
    const auto& thetar_ref = stage.params(50);
    const auto& a_0_square = stage.params(90);
    const auto& b_0_square = stage.params(92);
    const auto& x_0 = stage.params(93);
    const auto& y_0 = stage.params(94);
    const auto& cos_theta_0 = stage.params(96);
    const auto& sin_theta_0 = stage.params(97);
    const auto& a_1_square = stage.params(102);
    const auto& b_1_square = stage.params(104);
    const auto& x_1 = stage.params(105);
    const auto& y_1 = stage.params(106);
    const auto& cos_theta_1 = stage.params(108);
    const auto& sin_theta_1 = stage.params(109);
    const auto& a_2_square = stage.params(114);
    const auto& b_2_square = stage.params(116);
    const auto& x_2 = stage.params(117);
    const auto& y_2 = stage.params(118);
    const auto& cos_theta_2 = stage.params(120);
    const auto& sin_theta_2 = stage.params(121);
    const auto& a_3_square = stage.params(126);
    const auto& b_3_square = stage.params(128);
    const auto& x_3 = stage.params(129);
    const auto& y_3 = stage.params(130);
    const auto& cos_theta_3 = stage.params(132);
    const auto& sin_theta_3 = stage.params(133);
    const auto& a_4_square = stage.params(138);
    const auto& b_4_square = stage.params(140);
    const auto& x_4 = stage.params(141);
    const auto& y_4 = stage.params(142);
    const auto& cos_theta_4 = stage.params(144);
    const auto& sin_theta_4 = stage.params(145);
    const auto& a_5_square = stage.params(150);
    const auto& b_5_square = stage.params(152);
    const auto& x_5 = stage.params(153);
    const auto& y_5 = stage.params(154);
    const auto& cos_theta_5 = stage.params(156);
    const auto& sin_theta_5 = stage.params(157);
    const auto& a_6_square = stage.params(162);
    const auto& b_6_square = stage.params(164);
    const auto& x_6 = stage.params(165);
    const auto& y_6 = stage.params(166);
    const auto& cos_theta_6 = stage.params(168);
    const auto& sin_theta_6 = stage.params(169);
    const auto& a_7_square = stage.params(174);
    const auto& b_7_square = stage.params(176);
    const auto& x_7 = stage.params(177);
    const auto& y_7 = stage.params(178);
    const auto& cos_theta_7 = stage.params(180);
    const auto& sin_theta_7 = stage.params(181);

    (*ptr_global)(0) = tan(steer)/wheelbase;
    (*ptr_global)(1) = -rear_overhang*cos(theta) + x;
    (*ptr_global)(2) = -rear_overhang*sin(theta) + y;
    (*ptr_global)(3) = -(x - x_ref)*sin(theta_ref) + (y - y_ref)*cos(theta_ref);
    (*ptr_global)(4) = x + (front_overhang + wheelbase)*cos(theta);
    (*ptr_global)(5) = y + (front_overhang + wheelbase)*sin(theta);
    (*ptr_global)(6) = (-rear_overhang*sin(theta) + y - yr_ref)*cos(thetar_ref) - (-rear_overhang*cos(theta) + x - xr_ref)*sin(thetar_ref);
    (*ptr_global)(7) = -(x - xf_ref + (front_overhang + wheelbase)*cos(theta))*sin(thetaf_ref) + (y - yf_ref + (front_overhang + wheelbase)*sin(theta))*cos(thetaf_ref);
    (*ptr_global)(8) = cos(theta - theta_ref)/(-kappa_ref*(-(x - x_ref)*sin(theta_ref) + (y - y_ref)*cos(theta_ref)) + 1);
    (*ptr_global)(9) = pow(cos_theta_0*(-rear_overhang*sin(theta) + y - y_0) + sin_theta_0*(rear_overhang*cos(theta) - x + x_0), 2)/b_0_square + pow(cos_theta_0*(-rear_overhang*cos(theta) + x - x_0) + sin_theta_0*(-rear_overhang*sin(theta) + y - y_0), 2)/a_0_square;
    (*ptr_global)(10) = pow(cos_theta_1*(-rear_overhang*sin(theta) + y - y_1) + sin_theta_1*(rear_overhang*cos(theta) - x + x_1), 2)/b_1_square + pow(cos_theta_1*(-rear_overhang*cos(theta) + x - x_1) + sin_theta_1*(-rear_overhang*sin(theta) + y - y_1), 2)/a_1_square;
    (*ptr_global)(11) = pow(cos_theta_2*(-rear_overhang*sin(theta) + y - y_2) + sin_theta_2*(rear_overhang*cos(theta) - x + x_2), 2)/b_2_square + pow(cos_theta_2*(-rear_overhang*cos(theta) + x - x_2) + sin_theta_2*(-rear_overhang*sin(theta) + y - y_2), 2)/a_2_square;
    (*ptr_global)(12) = pow(cos_theta_3*(-rear_overhang*sin(theta) + y - y_3) + sin_theta_3*(rear_overhang*cos(theta) - x + x_3), 2)/b_3_square + pow(cos_theta_3*(-rear_overhang*cos(theta) + x - x_3) + sin_theta_3*(-rear_overhang*sin(theta) + y - y_3), 2)/a_3_square;
    (*ptr_global)(13) = pow(cos_theta_4*(-rear_overhang*sin(theta) + y - y_4) + sin_theta_4*(rear_overhang*cos(theta) - x + x_4), 2)/b_4_square + pow(cos_theta_4*(-rear_overhang*cos(theta) + x - x_4) + sin_theta_4*(-rear_overhang*sin(theta) + y - y_4), 2)/a_4_square;
    (*ptr_global)(14) = pow(cos_theta_5*(-rear_overhang*sin(theta) + y - y_5) + sin_theta_5*(rear_overhang*cos(theta) - x + x_5), 2)/b_5_square + pow(cos_theta_5*(-rear_overhang*cos(theta) + x - x_5) + sin_theta_5*(-rear_overhang*sin(theta) + y - y_5), 2)/a_5_square;
    (*ptr_global)(15) = pow(cos_theta_6*(-rear_overhang*sin(theta) + y - y_6) + sin_theta_6*(rear_overhang*cos(theta) - x + x_6), 2)/b_6_square + pow(cos_theta_6*(-rear_overhang*cos(theta) + x - x_6) + sin_theta_6*(-rear_overhang*sin(theta) + y - y_6), 2)/a_6_square;
    (*ptr_global)(16) = pow(cos_theta_7*(-rear_overhang*sin(theta) + y - y_7) + sin_theta_7*(rear_overhang*cos(theta) - x + x_7), 2)/b_7_square + pow(cos_theta_7*(-rear_overhang*cos(theta) + x - x_7) + sin_theta_7*(-rear_overhang*sin(theta) + y - y_7), 2)/a_7_square;
    (*ptr_global)(17) = pow(cos_theta_0*(y - y_0 + (front_overhang + wheelbase)*sin(theta)) + sin_theta_0*(-x + x_0 - (front_overhang + wheelbase)*cos(theta)), 2)/b_0_square + pow(cos_theta_0*(x - x_0 + (front_overhang + wheelbase)*cos(theta)) + sin_theta_0*(y - y_0 + (front_overhang + wheelbase)*sin(theta)), 2)/a_0_square;
    (*ptr_global)(18) = pow(cos_theta_1*(y - y_1 + (front_overhang + wheelbase)*sin(theta)) + sin_theta_1*(-x + x_1 - (front_overhang + wheelbase)*cos(theta)), 2)/b_1_square + pow(cos_theta_1*(x - x_1 + (front_overhang + wheelbase)*cos(theta)) + sin_theta_1*(y - y_1 + (front_overhang + wheelbase)*sin(theta)), 2)/a_1_square;
    (*ptr_global)(19) = pow(cos_theta_2*(y - y_2 + (front_overhang + wheelbase)*sin(theta)) + sin_theta_2*(-x + x_2 - (front_overhang + wheelbase)*cos(theta)), 2)/b_2_square + pow(cos_theta_2*(x - x_2 + (front_overhang + wheelbase)*cos(theta)) + sin_theta_2*(y - y_2 + (front_overhang + wheelbase)*sin(theta)), 2)/a_2_square;
    (*ptr_global)(20) = pow(cos_theta_3*(y - y_3 + (front_overhang + wheelbase)*sin(theta)) + sin_theta_3*(-x + x_3 - (front_overhang + wheelbase)*cos(theta)), 2)/b_3_square + pow(cos_theta_3*(x - x_3 + (front_overhang + wheelbase)*cos(theta)) + sin_theta_3*(y - y_3 + (front_overhang + wheelbase)*sin(theta)), 2)/a_3_square;
    (*ptr_global)(21) = pow(cos_theta_4*(y - y_4 + (front_overhang + wheelbase)*sin(theta)) + sin_theta_4*(-x + x_4 - (front_overhang + wheelbase)*cos(theta)), 2)/b_4_square + pow(cos_theta_4*(x - x_4 + (front_overhang + wheelbase)*cos(theta)) + sin_theta_4*(y - y_4 + (front_overhang + wheelbase)*sin(theta)), 2)/a_4_square;
    (*ptr_global)(22) = pow(cos_theta_5*(y - y_5 + (front_overhang + wheelbase)*sin(theta)) + sin_theta_5*(-x + x_5 - (front_overhang + wheelbase)*cos(theta)), 2)/b_5_square + pow(cos_theta_5*(x - x_5 + (front_overhang + wheelbase)*cos(theta)) + sin_theta_5*(y - y_5 + (front_overhang + wheelbase)*sin(theta)), 2)/a_5_square;
    (*ptr_global)(23) = pow(cos_theta_6*(y - y_6 + (front_overhang + wheelbase)*sin(theta)) + sin_theta_6*(-x + x_6 - (front_overhang + wheelbase)*cos(theta)), 2)/b_6_square + pow(cos_theta_6*(x - x_6 + (front_overhang + wheelbase)*cos(theta)) + sin_theta_6*(y - y_6 + (front_overhang + wheelbase)*sin(theta)), 2)/a_6_square;
    (*ptr_global)(24) = pow(cos_theta_7*(y - y_7 + (front_overhang + wheelbase)*sin(theta)) + sin_theta_7*(-x + x_7 - (front_overhang + wheelbase)*cos(theta)), 2)/b_7_square + pow(cos_theta_7*(x - x_7 + (front_overhang + wheelbase)*cos(theta)) + sin_theta_7*(y - y_7 + (front_overhang + wheelbase)*sin(theta)), 2)/a_7_square;
  }));
}

std::shared_ptr<Dynamics> SpatiotemporalPlannerModel::createDynamics(OcpConfig::IntegratorType type) const {
  return std::make_shared<SpatiotemporalPlannerModelDynamics>(type);
}

std::shared_ptr<CostFunction> SpatiotemporalPlannerModel::createCostFunction() const {
  return std::make_shared<SpatiotemporalPlannerModelCost<StageType::NORMINAL>>();
}

std::shared_ptr<CostFunction> SpatiotemporalPlannerModel::createTerminalCostFunction() const {
  return std::make_shared<SpatiotemporalPlannerModelCost<StageType::TERMINAL>>();
}

std::shared_ptr<Constraints> SpatiotemporalPlannerModel::createConstraint() const {
  return std::make_shared<SpatiotemporalPlannerModelConstraints<StageType::NORMINAL>>();
}

std::shared_ptr<Constraints> SpatiotemporalPlannerModel::createInitialConstraint() const {
  return std::make_shared<SpatiotemporalPlannerModelConstraints<StageType::INITIAL>>();
}

std::shared_ptr<Constraints> SpatiotemporalPlannerModel::createTerminalConstraint() const {
  return std::make_shared<SpatiotemporalPlannerModelConstraints<StageType::TERMINAL>>();
}

std::shared_ptr<Constraints> SpatiotemporalPlannerModel::createStateOnlyEqualities() const {
  return std::make_shared<SpatiotemporalPlannerModelStateOnlyEqualities<StageType::NORMINAL>>();
}

std::shared_ptr<Constraints> SpatiotemporalPlannerModel::createInitialStateOnlyEqualities() const {
  return std::make_shared<SpatiotemporalPlannerModelStateOnlyEqualities<StageType::INITIAL>>();
}

std::shared_ptr<Constraints> SpatiotemporalPlannerModel::createTerminalEqualities() const {
  return std::make_shared<SpatiotemporalPlannerModelStateOnlyEqualities<StageType::TERMINAL>>();
}

std::shared_ptr<IPMHelper> SpatiotemporalPlannerModel::createBaseIpmHelper() const {
  IPMHelper::Creater creater{
    {{StageType::NORMINAL, OcpConfig::FORWARD_EULER},[]() { return std::make_shared<SpatiotemporalPlannerModelIpmEvaluator<StageType::NORMINAL, OcpConfig::FORWARD_EULER>>(); }},
    {{StageType::NORMINAL, OcpConfig::ERK4},[]() { return std::make_shared<SpatiotemporalPlannerModelIpmEvaluator<StageType::NORMINAL, OcpConfig::ERK4>>(); }},
    {{StageType::INITIAL, OcpConfig::FORWARD_EULER},[]() { return std::make_shared<SpatiotemporalPlannerModelIpmEvaluator<StageType::INITIAL, OcpConfig::FORWARD_EULER>>(); }},
    {{StageType::INITIAL, OcpConfig::ERK4},[]() { return std::make_shared<SpatiotemporalPlannerModelIpmEvaluator<StageType::INITIAL, OcpConfig::ERK4>>(); }},
    {{StageType::TERMINAL, OcpConfig::FORWARD_EULER},[]() { return std::make_shared<SpatiotemporalPlannerModelIpmEvaluatorTerminal>(); }},
    {{StageType::TERMINAL, OcpConfig::ERK4},[]() { return std::make_shared<SpatiotemporalPlannerModelIpmEvaluatorTerminal>(); }}};
  return std::make_shared<IPMHelper>(std::move(creater));
}

REGIST_MODEL(SpatiotemporalPlannerModel)
} // namespace gpal::pnc::planning