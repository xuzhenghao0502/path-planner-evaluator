#pragma once

#include "ocp/cost_function.h"

namespace gpal::pnc::planning {

template <StageType Ttype>
class SpatiotemporalPlannerModelCost : public CostFunction {
 public:
  SpatiotemporalPlannerModelCost() : CostFunction(7, 13) {
  }

  virtual ~SpatiotemporalPlannerModelCost() = default;

  virtual double evaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls) const override {
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
    const auto& steer_ref = params(4);
    const auto& s_coarse = params(5);
    const auto& v_coarse = params(6);
    const auto& l_offset = params(12);
    const auto& a_offset = params(13);
    const auto& weight_prev_steer = params(19);
    const auto& prev_steer = params(20);
    const auto& l_ref_weight = params(21);
    const auto& theta_ref_weight = params(22);
    const auto& steer_weight = params(23);
    const auto& dsteer_weight = params(24);
    const auto& s_coarse_weight = params(25);
    const auto& v_coarse_weight = params(26);
    const auto& a_weight = params(27);
    const auto& jerk_weight = params(28);
    const auto& obs_0_weight = params(29);
    const auto& obs_1_weight = params(30);
    const auto& obs_2_weight = params(31);
    const auto& obs_3_weight = params(32);
    const auto& obs_4_weight = params(33);
    const auto& obs_5_weight = params(34);
    const auto& obs_6_weight = params(35);
    const auto& obs_7_weight = params(36);
    const auto& l_slack_weight = params(37);
    const auto& s_slack_weight = params(38);
    const auto& v_slack_weight = params(39);
    const auto& s_scale = params(185);
    const auto& v_scale = params(186);
    const auto& a_scale = params(187);
    const auto& jerk_scale = params(188);
    const auto& l_scale = params(189);
    const auto& theta_scale = params(190);
    const auto& steer_scale = params(191);
    const auto& dsteer_scale = params(192);

    // Determine global variables
    const auto& l = globals(3);

    // Evaluation of Scalar cost
    double cost = 0.5*pow(dsteer, 2)*dsteer_weight/pow(dsteer_scale, 2) + 0.5*pow(jerk, 2)*jerk_weight/pow(jerk_scale, 2) + 0.5*l_ref_weight*pow(l - l_offset, 2)/pow(l_scale, 2) + 0.5*pow(l_slack, 2)*l_slack_weight + 0.5*pow(obs_0_slack, 2)*obs_0_weight + 0.5*pow(obs_1_slack, 2)*obs_1_weight + 0.5*pow(obs_2_slack, 2)*obs_2_weight + 0.5*pow(obs_3_slack, 2)*obs_3_weight + 0.5*pow(obs_4_slack, 2)*obs_4_weight + 0.5*pow(obs_5_slack, 2)*obs_5_weight + 0.5*pow(obs_6_slack, 2)*obs_6_weight + 0.5*pow(obs_7_slack, 2)*obs_7_weight + 0.5*s_coarse_weight*pow(s - s_coarse, 2)/pow(s_scale, 2) + 0.5*pow(s_slack, 2)*s_slack_weight + 0.5*theta_ref_weight*pow(theta - theta_ref, 2)/pow(theta_scale, 2) + 0.5*v_coarse_weight*pow(v - v_coarse, 2)/pow(v_scale, 2) + 0.5*pow(v_slack, 2)*v_slack_weight + 0.5*weight_prev_steer*pow(-prev_steer + steer, 2) + 0.5*steer_weight*pow(steer - steer_ref, 2)/pow(steer_scale, 2) + 0.5*a_weight*pow(a - a_offset, 2)/pow(a_scale, 2);

    return cost;
  }

  virtual void updateGradient(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::VectorXd& gx, Eigen::VectorXd& gu) const override {
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
    const auto& steer_ref = params(4);
    const auto& s_coarse = params(5);
    const auto& v_coarse = params(6);
    const auto& l_offset = params(12);
    const auto& a_offset = params(13);
    const auto& weight_prev_steer = params(19);
    const auto& prev_steer = params(20);
    const auto& l_ref_weight = params(21);
    const auto& theta_ref_weight = params(22);
    const auto& steer_weight = params(23);
    const auto& dsteer_weight = params(24);
    const auto& s_coarse_weight = params(25);
    const auto& v_coarse_weight = params(26);
    const auto& a_weight = params(27);
    const auto& jerk_weight = params(28);
    const auto& obs_0_weight = params(29);
    const auto& obs_1_weight = params(30);
    const auto& obs_2_weight = params(31);
    const auto& obs_3_weight = params(32);
    const auto& obs_4_weight = params(33);
    const auto& obs_5_weight = params(34);
    const auto& obs_6_weight = params(35);
    const auto& obs_7_weight = params(36);
    const auto& l_slack_weight = params(37);
    const auto& s_slack_weight = params(38);
    const auto& v_slack_weight = params(39);
    const auto& s_scale = params(185);
    const auto& v_scale = params(186);
    const auto& a_scale = params(187);
    const auto& jerk_scale = params(188);
    const auto& l_scale = params(189);
    const auto& theta_scale = params(190);
    const auto& steer_scale = params(191);
    const auto& dsteer_scale = params(192);

    // Determine global variables
    const auto& l = globals(3);

    // Determine internal variables
    const double internal_0 = l_ref_weight*(l - l_offset)/pow(l_scale, 2);

    // Evaluation of Vector gx
    gx(0) = 0.5*s_coarse_weight*(2*s - 2*s_coarse)/pow(s_scale, 2);
    gx(1) = -internal_0*sin(theta_ref);
    gx(2) = internal_0*cos(theta_ref);
    gx(3) = 0.5*theta_ref_weight*(2*theta - 2*theta_ref)/pow(theta_scale, 2);
    gx(4) = 0.5*weight_prev_steer*(-2*prev_steer + 2*steer) + 0.5*steer_weight*(2*steer - 2*steer_ref)/pow(steer_scale, 2);
    gx(5) = 0.5*v_coarse_weight*(2*v - 2*v_coarse)/pow(v_scale, 2);
    gx(6) = 0.5*a_weight*(2*a - 2*a_offset)/pow(a_scale, 2);

    // Evaluation of Vector gu
    gu(0) = dsteer*dsteer_weight/pow(dsteer_scale, 2);
    gu(1) = jerk*jerk_weight/pow(jerk_scale, 2);
    gu(2) = l_slack*l_slack_weight;
    gu(3) = s_slack*s_slack_weight;
    gu(4) = v_slack*v_slack_weight;
    gu(5) = obs_0_slack*obs_0_weight;
    gu(6) = obs_1_slack*obs_1_weight;
    gu(7) = obs_2_slack*obs_2_weight;
    gu(8) = obs_3_slack*obs_3_weight;
    gu(9) = obs_4_slack*obs_4_weight;
    gu(10) = obs_5_slack*obs_5_weight;
    gu(11) = obs_6_slack*obs_6_weight;
    gu(12) = obs_7_slack*obs_7_weight;

  }

  virtual void updateGradientDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::VectorXd& gx) const override {
    // Determine stage variables
    const auto& s = states(0);
    const auto& theta = states(3);
    const auto& steer = states(4);
    const auto& v = states(5);
    const auto& a = states(6);
    const auto& theta_ref = params(2);
    const auto& steer_ref = params(4);
    const auto& s_coarse = params(5);
    const auto& v_coarse = params(6);
    const auto& l_offset = params(12);
    const auto& a_offset = params(13);
    const auto& weight_prev_steer = params(19);
    const auto& prev_steer = params(20);
    const auto& l_ref_weight = params(21);
    const auto& theta_ref_weight = params(22);
    const auto& steer_weight = params(23);
    const auto& s_coarse_weight = params(25);
    const auto& v_coarse_weight = params(26);
    const auto& a_weight = params(27);
    const auto& s_scale = params(185);
    const auto& v_scale = params(186);
    const auto& a_scale = params(187);
    const auto& l_scale = params(189);
    const auto& theta_scale = params(190);
    const auto& steer_scale = params(191);

    // Determine global variables
    const auto& l = globals(3);

    // Determine internal variables
    const double internal_0 = l_ref_weight*(l - l_offset)/pow(l_scale, 2);

    // Evaluation of Vector gx
    gx(0) = 0.5*s_coarse_weight*(2*s - 2*s_coarse)/pow(s_scale, 2);
    gx(1) = -internal_0*sin(theta_ref);
    gx(2) = internal_0*cos(theta_ref);
    gx(3) = 0.5*theta_ref_weight*(2*theta - 2*theta_ref)/pow(theta_scale, 2);
    gx(4) = 0.5*weight_prev_steer*(-2*prev_steer + 2*steer) + 0.5*steer_weight*(2*steer - 2*steer_ref)/pow(steer_scale, 2);
    gx(5) = 0.5*v_coarse_weight*(2*v - 2*v_coarse)/pow(v_scale, 2);
    gx(6) = 0.5*a_weight*(2*a - 2*a_offset)/pow(a_scale, 2);

  }

  virtual void updateGradientDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::VectorXd& gu) const override {
    // Determine stage variables
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
    const auto& dsteer_weight = params(24);
    const auto& jerk_weight = params(28);
    const auto& obs_0_weight = params(29);
    const auto& obs_1_weight = params(30);
    const auto& obs_2_weight = params(31);
    const auto& obs_3_weight = params(32);
    const auto& obs_4_weight = params(33);
    const auto& obs_5_weight = params(34);
    const auto& obs_6_weight = params(35);
    const auto& obs_7_weight = params(36);
    const auto& l_slack_weight = params(37);
    const auto& s_slack_weight = params(38);
    const auto& v_slack_weight = params(39);
    const auto& jerk_scale = params(188);
    const auto& dsteer_scale = params(192);

    // Evaluation of Vector gu
    gu(0) = dsteer*dsteer_weight/pow(dsteer_scale, 2);
    gu(1) = jerk*jerk_weight/pow(jerk_scale, 2);
    gu(2) = l_slack*l_slack_weight;
    gu(3) = s_slack*s_slack_weight;
    gu(4) = v_slack*v_slack_weight;
    gu(5) = obs_0_slack*obs_0_weight;
    gu(6) = obs_1_slack*obs_1_weight;
    gu(7) = obs_2_slack*obs_2_weight;
    gu(8) = obs_3_slack*obs_3_weight;
    gu(9) = obs_4_slack*obs_4_weight;
    gu(10) = obs_5_slack*obs_5_weight;
    gu(11) = obs_6_slack*obs_6_weight;
    gu(12) = obs_7_slack*obs_7_weight;

  }

  virtual void updateHessian(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& dxdx, Eigen::MatrixXd& dxdu, Eigen::MatrixXd& dudu) const override {
    // Determine stage variables
    const auto& theta_ref = params(2);
    const auto& weight_prev_steer = params(19);
    const auto& l_ref_weight = params(21);
    const auto& theta_ref_weight = params(22);
    const auto& steer_weight = params(23);
    const auto& dsteer_weight = params(24);
    const auto& s_coarse_weight = params(25);
    const auto& v_coarse_weight = params(26);
    const auto& a_weight = params(27);
    const auto& jerk_weight = params(28);
    const auto& obs_0_weight = params(29);
    const auto& obs_1_weight = params(30);
    const auto& obs_2_weight = params(31);
    const auto& obs_3_weight = params(32);
    const auto& obs_4_weight = params(33);
    const auto& obs_5_weight = params(34);
    const auto& obs_6_weight = params(35);
    const auto& obs_7_weight = params(36);
    const auto& l_slack_weight = params(37);
    const auto& s_slack_weight = params(38);
    const auto& v_slack_weight = params(39);
    const auto& s_scale = params(185);
    const auto& v_scale = params(186);
    const auto& a_scale = params(187);
    const auto& jerk_scale = params(188);
    const auto& l_scale = params(189);
    const auto& theta_scale = params(190);
    const auto& steer_scale = params(191);
    const auto& dsteer_scale = params(192);

    // Determine internal variables
    const double internal_0 = sin(theta_ref);
    const double internal_2 = cos(theta_ref);
    const double internal_1 = l_ref_weight/pow(l_scale, 2);
    const double internal_3 = -internal_0*internal_1*internal_2;

    // Evaluation of SymmetricMatrix dxdx
    dxdx(0, 0) = s_coarse_weight/pow(s_scale, 2);
    dxdx(1, 1) = pow(internal_0, 2)*internal_1;
    dxdx(1, 2) = internal_3;
    dxdx(2, 2) = internal_1*pow(internal_2, 2);
    dxdx(3, 3) = theta_ref_weight/pow(theta_scale, 2);
    dxdx(4, 4) = weight_prev_steer + steer_weight/pow(steer_scale, 2);
    dxdx(5, 5) = v_coarse_weight/pow(v_scale, 2);
    dxdx(6, 6) = a_weight/pow(a_scale, 2);
    dxdx(2, 1) = dxdx(1, 2);

    // Evaluation of SymmetricMatrix dudu
    dudu(0, 0) = dsteer_weight/pow(dsteer_scale, 2);
    dudu(1, 1) = jerk_weight/pow(jerk_scale, 2);
    dudu(2, 2) = l_slack_weight;
    dudu(3, 3) = s_slack_weight;
    dudu(4, 4) = v_slack_weight;
    dudu(5, 5) = obs_0_weight;
    dudu(6, 6) = obs_1_weight;
    dudu(7, 7) = obs_2_weight;
    dudu(8, 8) = obs_3_weight;
    dudu(9, 9) = obs_4_weight;
    dudu(10, 10) = obs_5_weight;
    dudu(11, 11) = obs_6_weight;
    dudu(12, 12) = obs_7_weight;

  }

  virtual void updateHessianDxDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& dxdx) const override {
    // Determine stage variables
    const auto& theta_ref = params(2);
    const auto& weight_prev_steer = params(19);
    const auto& l_ref_weight = params(21);
    const auto& theta_ref_weight = params(22);
    const auto& steer_weight = params(23);
    const auto& s_coarse_weight = params(25);
    const auto& v_coarse_weight = params(26);
    const auto& a_weight = params(27);
    const auto& s_scale = params(185);
    const auto& v_scale = params(186);
    const auto& a_scale = params(187);
    const auto& l_scale = params(189);
    const auto& theta_scale = params(190);
    const auto& steer_scale = params(191);

    // Determine internal variables
    const double internal_0 = sin(theta_ref);
    const double internal_2 = cos(theta_ref);
    const double internal_1 = l_ref_weight/pow(l_scale, 2);
    const double internal_3 = -internal_0*internal_1*internal_2;

    // Evaluation of SymmetricMatrix dxdx
    dxdx(0, 0) = s_coarse_weight/pow(s_scale, 2);
    dxdx(1, 1) = pow(internal_0, 2)*internal_1;
    dxdx(1, 2) = internal_3;
    dxdx(2, 2) = internal_1*pow(internal_2, 2);
    dxdx(3, 3) = theta_ref_weight/pow(theta_scale, 2);
    dxdx(4, 4) = weight_prev_steer + steer_weight/pow(steer_scale, 2);
    dxdx(5, 5) = v_coarse_weight/pow(v_scale, 2);
    dxdx(6, 6) = a_weight/pow(a_scale, 2);
    dxdx(2, 1) = dxdx(1, 2);

  }

  virtual void updateHessianDxDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& dxdu) const override {

  }

  virtual void updateHessianDuDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& dudu) const override {
    // Determine stage variables
    const auto& dsteer_weight = params(24);
    const auto& jerk_weight = params(28);
    const auto& obs_0_weight = params(29);
    const auto& obs_1_weight = params(30);
    const auto& obs_2_weight = params(31);
    const auto& obs_3_weight = params(32);
    const auto& obs_4_weight = params(33);
    const auto& obs_5_weight = params(34);
    const auto& obs_6_weight = params(35);
    const auto& obs_7_weight = params(36);
    const auto& l_slack_weight = params(37);
    const auto& s_slack_weight = params(38);
    const auto& v_slack_weight = params(39);
    const auto& jerk_scale = params(188);
    const auto& dsteer_scale = params(192);

    // Evaluation of SymmetricMatrix dudu
    dudu(0, 0) = dsteer_weight/pow(dsteer_scale, 2);
    dudu(1, 1) = jerk_weight/pow(jerk_scale, 2);
    dudu(2, 2) = l_slack_weight;
    dudu(3, 3) = s_slack_weight;
    dudu(4, 4) = v_slack_weight;
    dudu(5, 5) = obs_0_weight;
    dudu(6, 6) = obs_1_weight;
    dudu(7, 7) = obs_2_weight;
    dudu(8, 8) = obs_3_weight;
    dudu(9, 9) = obs_4_weight;
    dudu(10, 10) = obs_5_weight;
    dudu(11, 11) = obs_6_weight;
    dudu(12, 12) = obs_7_weight;

  }

  virtual void updateGradient(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t, Eigen::VectorXd& gx, Eigen::VectorXd& gu) const override {
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
    const auto& steer_ref = params(4);
    const auto& s_coarse = params(5);
    const auto& v_coarse = params(6);
    const auto& l_offset = params(12);
    const auto& a_offset = params(13);
    const auto& weight_prev_steer = params(19);
    const auto& prev_steer = params(20);
    const auto& l_ref_weight = params(21);
    const auto& theta_ref_weight = params(22);
    const auto& steer_weight = params(23);
    const auto& dsteer_weight = params(24);
    const auto& s_coarse_weight = params(25);
    const auto& v_coarse_weight = params(26);
    const auto& a_weight = params(27);
    const auto& jerk_weight = params(28);
    const auto& obs_0_weight = params(29);
    const auto& obs_1_weight = params(30);
    const auto& obs_2_weight = params(31);
    const auto& obs_3_weight = params(32);
    const auto& obs_4_weight = params(33);
    const auto& obs_5_weight = params(34);
    const auto& obs_6_weight = params(35);
    const auto& obs_7_weight = params(36);
    const auto& l_slack_weight = params(37);
    const auto& s_slack_weight = params(38);
    const auto& v_slack_weight = params(39);
    const auto& s_scale = params(185);
    const auto& v_scale = params(186);
    const auto& a_scale = params(187);
    const auto& jerk_scale = params(188);
    const auto& l_scale = params(189);
    const auto& theta_scale = params(190);
    const auto& steer_scale = params(191);
    const auto& dsteer_scale = params(192);

    // Determine global variables
    const auto& l = globals(3);

    // Determine internal variables
    const double internal_0 = 0.5*delta_t;
    const double internal_1 = delta_t;
    const double internal_2 = internal_1*l_ref_weight*(l - l_offset)/pow(l_scale, 2);

    // Evaluation of Vector gx
    gx(0) = internal_0*s_coarse_weight*(2*s - 2*s_coarse)/pow(s_scale, 2);
    gx(1) = -internal_2*sin(theta_ref);
    gx(2) = internal_2*cos(theta_ref);
    gx(3) = internal_0*theta_ref_weight*(2*theta - 2*theta_ref)/pow(theta_scale, 2);
    gx(4) = delta_t*(0.5*weight_prev_steer*(-2*prev_steer + 2*steer) + 0.5*steer_weight*(2*steer - 2*steer_ref)/pow(steer_scale, 2));
    gx(5) = internal_0*v_coarse_weight*(2*v - 2*v_coarse)/pow(v_scale, 2);
    gx(6) = a_weight*internal_0*(2*a - 2*a_offset)/pow(a_scale, 2);

    // Evaluation of Vector gu
    gu(0) = dsteer*dsteer_weight*internal_1/pow(dsteer_scale, 2);
    gu(1) = internal_1*jerk*jerk_weight/pow(jerk_scale, 2);
    gu(2) = internal_1*l_slack*l_slack_weight;
    gu(3) = internal_1*s_slack*s_slack_weight;
    gu(4) = internal_1*v_slack*v_slack_weight;
    gu(5) = internal_1*obs_0_slack*obs_0_weight;
    gu(6) = internal_1*obs_1_slack*obs_1_weight;
    gu(7) = internal_1*obs_2_slack*obs_2_weight;
    gu(8) = internal_1*obs_3_slack*obs_3_weight;
    gu(9) = internal_1*obs_4_slack*obs_4_weight;
    gu(10) = internal_1*obs_5_slack*obs_5_weight;
    gu(11) = internal_1*obs_6_slack*obs_6_weight;
    gu(12) = internal_1*obs_7_slack*obs_7_weight;

  }

  virtual void updateGradientDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t, Eigen::VectorXd& gx) const override {
    // Determine stage variables
    const auto& s = states(0);
    const auto& theta = states(3);
    const auto& steer = states(4);
    const auto& v = states(5);
    const auto& a = states(6);
    const auto& theta_ref = params(2);
    const auto& steer_ref = params(4);
    const auto& s_coarse = params(5);
    const auto& v_coarse = params(6);
    const auto& l_offset = params(12);
    const auto& a_offset = params(13);
    const auto& weight_prev_steer = params(19);
    const auto& prev_steer = params(20);
    const auto& l_ref_weight = params(21);
    const auto& theta_ref_weight = params(22);
    const auto& steer_weight = params(23);
    const auto& s_coarse_weight = params(25);
    const auto& v_coarse_weight = params(26);
    const auto& a_weight = params(27);
    const auto& s_scale = params(185);
    const auto& v_scale = params(186);
    const auto& a_scale = params(187);
    const auto& l_scale = params(189);
    const auto& theta_scale = params(190);
    const auto& steer_scale = params(191);

    // Determine global variables
    const auto& l = globals(3);

    // Determine internal variables
    const double internal_0 = 0.5*delta_t;
    const double internal_1 = delta_t*l_ref_weight*(l - l_offset)/pow(l_scale, 2);

    // Evaluation of Vector gx
    gx(0) = internal_0*s_coarse_weight*(2*s - 2*s_coarse)/pow(s_scale, 2);
    gx(1) = -internal_1*sin(theta_ref);
    gx(2) = internal_1*cos(theta_ref);
    gx(3) = internal_0*theta_ref_weight*(2*theta - 2*theta_ref)/pow(theta_scale, 2);
    gx(4) = delta_t*(0.5*weight_prev_steer*(-2*prev_steer + 2*steer) + 0.5*steer_weight*(2*steer - 2*steer_ref)/pow(steer_scale, 2));
    gx(5) = internal_0*v_coarse_weight*(2*v - 2*v_coarse)/pow(v_scale, 2);
    gx(6) = a_weight*internal_0*(2*a - 2*a_offset)/pow(a_scale, 2);

  }

  virtual void updateGradientDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t, Eigen::VectorXd& gu) const override {
    // Determine stage variables
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
    const auto& dsteer_weight = params(24);
    const auto& jerk_weight = params(28);
    const auto& obs_0_weight = params(29);
    const auto& obs_1_weight = params(30);
    const auto& obs_2_weight = params(31);
    const auto& obs_3_weight = params(32);
    const auto& obs_4_weight = params(33);
    const auto& obs_5_weight = params(34);
    const auto& obs_6_weight = params(35);
    const auto& obs_7_weight = params(36);
    const auto& l_slack_weight = params(37);
    const auto& s_slack_weight = params(38);
    const auto& v_slack_weight = params(39);
    const auto& jerk_scale = params(188);
    const auto& dsteer_scale = params(192);

    // Determine internal variables
    const double internal_0 = delta_t;

    // Evaluation of Vector gu
    gu(0) = dsteer*dsteer_weight*internal_0/pow(dsteer_scale, 2);
    gu(1) = internal_0*jerk*jerk_weight/pow(jerk_scale, 2);
    gu(2) = internal_0*l_slack*l_slack_weight;
    gu(3) = internal_0*s_slack*s_slack_weight;
    gu(4) = internal_0*v_slack*v_slack_weight;
    gu(5) = internal_0*obs_0_slack*obs_0_weight;
    gu(6) = internal_0*obs_1_slack*obs_1_weight;
    gu(7) = internal_0*obs_2_slack*obs_2_weight;
    gu(8) = internal_0*obs_3_slack*obs_3_weight;
    gu(9) = internal_0*obs_4_slack*obs_4_weight;
    gu(10) = internal_0*obs_5_slack*obs_5_weight;
    gu(11) = internal_0*obs_6_slack*obs_6_weight;
    gu(12) = internal_0*obs_7_slack*obs_7_weight;

  }

  virtual void updateHessian(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t, Eigen::MatrixXd& dxdx, Eigen::MatrixXd& dxdu, Eigen::MatrixXd& dudu) const override {
    // Determine stage variables
    const auto& theta_ref = params(2);
    const auto& weight_prev_steer = params(19);
    const auto& l_ref_weight = params(21);
    const auto& theta_ref_weight = params(22);
    const auto& steer_weight = params(23);
    const auto& dsteer_weight = params(24);
    const auto& s_coarse_weight = params(25);
    const auto& v_coarse_weight = params(26);
    const auto& a_weight = params(27);
    const auto& jerk_weight = params(28);
    const auto& obs_0_weight = params(29);
    const auto& obs_1_weight = params(30);
    const auto& obs_2_weight = params(31);
    const auto& obs_3_weight = params(32);
    const auto& obs_4_weight = params(33);
    const auto& obs_5_weight = params(34);
    const auto& obs_6_weight = params(35);
    const auto& obs_7_weight = params(36);
    const auto& l_slack_weight = params(37);
    const auto& s_slack_weight = params(38);
    const auto& v_slack_weight = params(39);
    const auto& s_scale = params(185);
    const auto& v_scale = params(186);
    const auto& a_scale = params(187);
    const auto& jerk_scale = params(188);
    const auto& l_scale = params(189);
    const auto& theta_scale = params(190);
    const auto& steer_scale = params(191);
    const auto& dsteer_scale = params(192);

    // Determine internal variables
    const double internal_0 = delta_t;
    const double internal_1 = sin(theta_ref);
    const double internal_3 = cos(theta_ref);
    const double internal_2 = internal_0*l_ref_weight/pow(l_scale, 2);
    const double internal_4 = -internal_1*internal_2*internal_3;

    // Evaluation of SymmetricMatrix dxdx
    dxdx(0, 0) = internal_0*s_coarse_weight/pow(s_scale, 2);
    dxdx(1, 1) = pow(internal_1, 2)*internal_2;
    dxdx(1, 2) = internal_4;
    dxdx(2, 2) = internal_2*pow(internal_3, 2);
    dxdx(3, 3) = internal_0*theta_ref_weight/pow(theta_scale, 2);
    dxdx(4, 4) = delta_t*(weight_prev_steer + steer_weight/pow(steer_scale, 2));
    dxdx(5, 5) = internal_0*v_coarse_weight/pow(v_scale, 2);
    dxdx(6, 6) = a_weight*internal_0/pow(a_scale, 2);
    dxdx(2, 1) = dxdx(1, 2);

    // Evaluation of SymmetricMatrix dudu
    dudu(0, 0) = dsteer_weight*internal_0/pow(dsteer_scale, 2);
    dudu(1, 1) = internal_0*jerk_weight/pow(jerk_scale, 2);
    dudu(2, 2) = internal_0*l_slack_weight;
    dudu(3, 3) = internal_0*s_slack_weight;
    dudu(4, 4) = internal_0*v_slack_weight;
    dudu(5, 5) = internal_0*obs_0_weight;
    dudu(6, 6) = internal_0*obs_1_weight;
    dudu(7, 7) = internal_0*obs_2_weight;
    dudu(8, 8) = internal_0*obs_3_weight;
    dudu(9, 9) = internal_0*obs_4_weight;
    dudu(10, 10) = internal_0*obs_5_weight;
    dudu(11, 11) = internal_0*obs_6_weight;
    dudu(12, 12) = internal_0*obs_7_weight;

  }

  virtual void updateHessianDxDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t, Eigen::MatrixXd& dxdx) const override {
    // Determine stage variables
    const auto& theta_ref = params(2);
    const auto& weight_prev_steer = params(19);
    const auto& l_ref_weight = params(21);
    const auto& theta_ref_weight = params(22);
    const auto& steer_weight = params(23);
    const auto& s_coarse_weight = params(25);
    const auto& v_coarse_weight = params(26);
    const auto& a_weight = params(27);
    const auto& s_scale = params(185);
    const auto& v_scale = params(186);
    const auto& a_scale = params(187);
    const auto& l_scale = params(189);
    const auto& theta_scale = params(190);
    const auto& steer_scale = params(191);

    // Determine internal variables
    const double internal_0 = delta_t;
    const double internal_1 = sin(theta_ref);
    const double internal_3 = cos(theta_ref);
    const double internal_2 = internal_0*l_ref_weight/pow(l_scale, 2);
    const double internal_4 = -internal_1*internal_2*internal_3;

    // Evaluation of SymmetricMatrix dxdx
    dxdx(0, 0) = internal_0*s_coarse_weight/pow(s_scale, 2);
    dxdx(1, 1) = pow(internal_1, 2)*internal_2;
    dxdx(1, 2) = internal_4;
    dxdx(2, 2) = internal_2*pow(internal_3, 2);
    dxdx(3, 3) = internal_0*theta_ref_weight/pow(theta_scale, 2);
    dxdx(4, 4) = delta_t*(weight_prev_steer + steer_weight/pow(steer_scale, 2));
    dxdx(5, 5) = internal_0*v_coarse_weight/pow(v_scale, 2);
    dxdx(6, 6) = a_weight*internal_0/pow(a_scale, 2);
    dxdx(2, 1) = dxdx(1, 2);

  }

  virtual void updateHessianDxDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t, Eigen::MatrixXd& dxdu) const override {

  }

  virtual void updateHessianDuDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t, Eigen::MatrixXd& dudu) const override {
    // Determine stage variables
    const auto& dsteer_weight = params(24);
    const auto& jerk_weight = params(28);
    const auto& obs_0_weight = params(29);
    const auto& obs_1_weight = params(30);
    const auto& obs_2_weight = params(31);
    const auto& obs_3_weight = params(32);
    const auto& obs_4_weight = params(33);
    const auto& obs_5_weight = params(34);
    const auto& obs_6_weight = params(35);
    const auto& obs_7_weight = params(36);
    const auto& l_slack_weight = params(37);
    const auto& s_slack_weight = params(38);
    const auto& v_slack_weight = params(39);
    const auto& jerk_scale = params(188);
    const auto& dsteer_scale = params(192);

    // Determine internal variables
    const double internal_0 = delta_t;

    // Evaluation of SymmetricMatrix dudu
    dudu(0, 0) = dsteer_weight*internal_0/pow(dsteer_scale, 2);
    dudu(1, 1) = internal_0*jerk_weight/pow(jerk_scale, 2);
    dudu(2, 2) = internal_0*l_slack_weight;
    dudu(3, 3) = internal_0*s_slack_weight;
    dudu(4, 4) = internal_0*v_slack_weight;
    dudu(5, 5) = internal_0*obs_0_weight;
    dudu(6, 6) = internal_0*obs_1_weight;
    dudu(7, 7) = internal_0*obs_2_weight;
    dudu(8, 8) = internal_0*obs_3_weight;
    dudu(9, 9) = internal_0*obs_4_weight;
    dudu(10, 10) = internal_0*obs_5_weight;
    dudu(11, 11) = internal_0*obs_6_weight;
    dudu(12, 12) = internal_0*obs_7_weight;

  }

};

template <>
class SpatiotemporalPlannerModelCost<StageType::TERMINAL> : public CostFunction {
 public:
  SpatiotemporalPlannerModelCost() : CostFunction(7, 13) {}
  virtual ~SpatiotemporalPlannerModelCost() = default;

  virtual double evaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls) const override {
    // Determine stage variables
    const auto& s = states(0);
    const auto& theta = states(3);
    const auto& v = states(5);
    const auto& l_terminal = params(8);
    const auto& theta_terminal = params(9);
    const auto& s_terminal = params(10);
    const auto& v_terminal = params(11);
    const auto& terminal_l_weight = params(40);
    const auto& terminal_theta_weight = params(41);
    const auto& terminal_s_weight = params(42);
    const auto& terminal_v_weight = params(43);
    const auto& s_scale = params(185);
    const auto& v_scale = params(186);
    const auto& l_scale = params(189);
    const auto& theta_scale = params(190);

    // Determine global variables
    const auto& l = globals(3);

    // Evaluation of Scalar cost
    double cost = 0.5*terminal_theta_weight*pow(theta - theta_terminal, 2)/pow(theta_scale, 2) + 0.5*terminal_v_weight*pow(v - v_terminal, 2)/pow(v_scale, 2) + 0.5*terminal_s_weight*pow(s - s_terminal, 2)/pow(s_scale, 2) + 0.5*terminal_l_weight*pow(l - l_terminal, 2)/pow(l_scale, 2);

    return cost;
  }

  virtual void updateGradientDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::VectorXd& gx) const override {
    // Determine stage variables
    const auto& s = states(0);
    const auto& theta = states(3);
    const auto& v = states(5);
    const auto& theta_ref = params(2);
    const auto& l_terminal = params(8);
    const auto& theta_terminal = params(9);
    const auto& s_terminal = params(10);
    const auto& v_terminal = params(11);
    const auto& terminal_l_weight = params(40);
    const auto& terminal_theta_weight = params(41);
    const auto& terminal_s_weight = params(42);
    const auto& terminal_v_weight = params(43);
    const auto& s_scale = params(185);
    const auto& v_scale = params(186);
    const auto& l_scale = params(189);
    const auto& theta_scale = params(190);

    // Determine global variables
    const auto& l = globals(3);

    // Determine internal variables
    const double internal_0 = terminal_l_weight*(l - l_terminal)/pow(l_scale, 2);

    // Evaluation of Vector gx
    gx(0) = 0.5*terminal_s_weight*(2*s - 2*s_terminal)/pow(s_scale, 2);
    gx(1) = -internal_0*sin(theta_ref);
    gx(2) = internal_0*cos(theta_ref);
    gx(3) = 0.5*terminal_theta_weight*(2*theta - 2*theta_terminal)/pow(theta_scale, 2);
    gx(5) = 0.5*terminal_v_weight*(2*v - 2*v_terminal)/pow(v_scale, 2);

  }

  virtual void updateHessianDxDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& hxx) const override {
    // Determine stage variables
    const auto& theta_ref = params(2);
    const auto& terminal_l_weight = params(40);
    const auto& terminal_theta_weight = params(41);
    const auto& terminal_s_weight = params(42);
    const auto& terminal_v_weight = params(43);
    const auto& s_scale = params(185);
    const auto& v_scale = params(186);
    const auto& l_scale = params(189);
    const auto& theta_scale = params(190);

    // Determine internal variables
    const double internal_0 = sin(theta_ref);
    const double internal_2 = cos(theta_ref);
    const double internal_1 = terminal_l_weight/pow(l_scale, 2);
    const double internal_3 = -internal_0*internal_1*internal_2;

    // Evaluation of SymmetricMatrix hxx
    hxx(0, 0) = terminal_s_weight/pow(s_scale, 2);
    hxx(1, 1) = pow(internal_0, 2)*internal_1;
    hxx(1, 2) = internal_3;
    hxx(2, 2) = internal_1*pow(internal_2, 2);
    hxx(3, 3) = terminal_theta_weight/pow(theta_scale, 2);
    hxx(5, 5) = terminal_v_weight/pow(v_scale, 2);
    hxx(2, 1) = hxx(1, 2);

  }

};

}  //  namespace gpal::pnc::planning