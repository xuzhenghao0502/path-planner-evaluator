#pragma once

#include "ocp/cost_function.h"

namespace gpal::pnc::planning {

template <StageType Ttype>
class MultiAgentsTrajectoryModelCost : public CostFunction {
 public:
  MultiAgentsTrajectoryModelCost() : CostFunction(10, 9) {
  }

  virtual ~MultiAgentsTrajectoryModelCost() = default;

  virtual double evaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls) const override {
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
    const auto& ego_steer_ref = params(4);
    const auto& ego_s_coarse = params(10);
    const auto& ego_v_coarse = params(11);
    const auto& agent_0_s_coarse = params(13);
    const auto& agent_0_v_coarse = params(14);
    const auto& l_offset = params(16);
    const auto& l_ref_weight = params(22);
    const auto& theta_ref_weight = params(23);
    const auto& steer_weight = params(24);
    const auto& ego_s_coarse_weight = params(25);
    const auto& ego_v_coarse_weight = params(26);
    const auto& ego_a_weight = params(27);
    const auto& agent_0_s_coarse_weight = params(28);
    const auto& agent_0_v_coarse_weight = params(29);
    const auto& agent_0_a_weight = params(30);
    const auto& agent_0_avoid_weight = params(31);
    const auto& obs_0_weight = params(32);
    const auto& obs_1_weight = params(33);
    const auto& s_scale = params(79);
    const auto& v_scale = params(80);
    const auto& a_scale = params(81);
    const auto& l_scale = params(83);
    const auto& theta_scale = params(84);
    const auto& steer_scale = params(85);

    // Determine global variables
    const auto& ego_l = globals(3);

    // Determine internal variables
    const double internal_0 = 0.5*agent_0_avoid_weight;
    const double internal_1 = 0.5*obs_0_weight;
    const double internal_2 = 0.5*obs_1_weight;
    const double internal_4 = 0.5/pow(s_scale, 2);
    const double internal_5 = 0.5/pow(v_scale, 2);
    const double internal_3 = 0.5/pow(a_scale, 2);

    // Evaluation of Scalar cost
    double cost = pow(agent_0_a, 2)*agent_0_a_weight*internal_3 + agent_0_s_coarse_weight*internal_4*pow(agent_0_s - agent_0_s_coarse, 2) + agent_0_v_coarse_weight*internal_5*pow(agent_0_v - agent_0_v_coarse, 2) + pow(ego_a, 2)*ego_a_weight*internal_3 + ego_f_2_agent_0_slack*internal_0 + ego_f_2_object_0_slack*internal_1 + ego_f_2_object_1_slack*internal_2 + ego_r_2_agent_0_slack*internal_0 + ego_r_2_object_0_slack*internal_1 + ego_r_2_object_1_slack*internal_2 + ego_s_coarse_weight*internal_4*pow(ego_s - ego_s_coarse, 2) + ego_v_coarse_weight*internal_5*pow(ego_v - ego_v_coarse, 2) + 0.5*l_ref_weight*pow(ego_l - l_offset, 2)/pow(l_scale, 2) + 0.5*theta_ref_weight*pow(ego_theta - ego_theta_ref, 2)/pow(theta_scale, 2) + 0.5*steer_weight*pow(ego_steer - ego_steer_ref, 2)/pow(steer_scale, 2);

    return cost;
  }

  virtual void updateGradient(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::VectorXd& gx, Eigen::VectorXd& gu) const override {
    // Determine stage variables
    const auto& ego_s = states(0);
    const auto& ego_theta = states(3);
    const auto& ego_v = states(4);
    const auto& agent_0_s = states(5);
    const auto& agent_0_v = states(9);
    const auto& ego_a = ctrls(0);
    const auto& ego_steer = ctrls(1);
    const auto& agent_0_a = ctrls(2);
    const auto& ego_theta_ref = params(2);
    const auto& ego_steer_ref = params(4);
    const auto& ego_s_coarse = params(10);
    const auto& ego_v_coarse = params(11);
    const auto& agent_0_s_coarse = params(13);
    const auto& agent_0_v_coarse = params(14);
    const auto& l_offset = params(16);
    const auto& l_ref_weight = params(22);
    const auto& theta_ref_weight = params(23);
    const auto& steer_weight = params(24);
    const auto& ego_s_coarse_weight = params(25);
    const auto& ego_v_coarse_weight = params(26);
    const auto& ego_a_weight = params(27);
    const auto& agent_0_s_coarse_weight = params(28);
    const auto& agent_0_v_coarse_weight = params(29);
    const auto& agent_0_a_weight = params(30);
    const auto& agent_0_avoid_weight = params(31);
    const auto& obs_0_weight = params(32);
    const auto& obs_1_weight = params(33);
    const auto& s_scale = params(79);
    const auto& v_scale = params(80);
    const auto& a_scale = params(81);
    const auto& l_scale = params(83);
    const auto& theta_scale = params(84);
    const auto& steer_scale = params(85);

    // Determine global variables
    const auto& ego_l = globals(3);

    // Determine internal variables
    const double internal_4 = 0.5*agent_0_avoid_weight;
    const double internal_5 = 0.5*obs_0_weight;
    const double internal_6 = 0.5*obs_1_weight;
    const double internal_3 = pow(a_scale, -2);
    const double internal_0 = 0.5/pow(s_scale, 2);
    const double internal_2 = 0.5/pow(v_scale, 2);
    const double internal_1 = l_ref_weight*(ego_l - l_offset)/pow(l_scale, 2);

    // Evaluation of Vector gx
    gx(0) = ego_s_coarse_weight*internal_0*(2*ego_s - 2*ego_s_coarse);
    gx(1) = -internal_1*sin(ego_theta_ref);
    gx(2) = internal_1*cos(ego_theta_ref);
    gx(3) = 0.5*theta_ref_weight*(2*ego_theta - 2*ego_theta_ref)/pow(theta_scale, 2);
    gx(4) = ego_v_coarse_weight*internal_2*(2*ego_v - 2*ego_v_coarse);
    gx(5) = agent_0_s_coarse_weight*internal_0*(2*agent_0_s - 2*agent_0_s_coarse);
    gx(9) = agent_0_v_coarse_weight*internal_2*(2*agent_0_v - 2*agent_0_v_coarse);

    // Evaluation of Vector gu
    gu(0) = ego_a*ego_a_weight*internal_3;
    gu(1) = 0.5*steer_weight*(2*ego_steer - 2*ego_steer_ref)/pow(steer_scale, 2);
    gu(2) = agent_0_a*agent_0_a_weight*internal_3;
    gu(3) = internal_4;
    gu(4) = internal_4;
    gu(5) = internal_5;
    gu(6) = internal_5;
    gu(7) = internal_6;
    gu(8) = internal_6;

  }

  virtual void updateGradientDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::VectorXd& gx) const override {
    // Determine stage variables
    const auto& ego_s = states(0);
    const auto& ego_theta = states(3);
    const auto& ego_v = states(4);
    const auto& agent_0_s = states(5);
    const auto& agent_0_v = states(9);
    const auto& ego_theta_ref = params(2);
    const auto& ego_s_coarse = params(10);
    const auto& ego_v_coarse = params(11);
    const auto& agent_0_s_coarse = params(13);
    const auto& agent_0_v_coarse = params(14);
    const auto& l_offset = params(16);
    const auto& l_ref_weight = params(22);
    const auto& theta_ref_weight = params(23);
    const auto& ego_s_coarse_weight = params(25);
    const auto& ego_v_coarse_weight = params(26);
    const auto& agent_0_s_coarse_weight = params(28);
    const auto& agent_0_v_coarse_weight = params(29);
    const auto& s_scale = params(79);
    const auto& v_scale = params(80);
    const auto& l_scale = params(83);
    const auto& theta_scale = params(84);

    // Determine global variables
    const auto& ego_l = globals(3);

    // Determine internal variables
    const double internal_0 = 0.5/pow(s_scale, 2);
    const double internal_2 = 0.5/pow(v_scale, 2);
    const double internal_1 = l_ref_weight*(ego_l - l_offset)/pow(l_scale, 2);

    // Evaluation of Vector gx
    gx(0) = ego_s_coarse_weight*internal_0*(2*ego_s - 2*ego_s_coarse);
    gx(1) = -internal_1*sin(ego_theta_ref);
    gx(2) = internal_1*cos(ego_theta_ref);
    gx(3) = 0.5*theta_ref_weight*(2*ego_theta - 2*ego_theta_ref)/pow(theta_scale, 2);
    gx(4) = ego_v_coarse_weight*internal_2*(2*ego_v - 2*ego_v_coarse);
    gx(5) = agent_0_s_coarse_weight*internal_0*(2*agent_0_s - 2*agent_0_s_coarse);
    gx(9) = agent_0_v_coarse_weight*internal_2*(2*agent_0_v - 2*agent_0_v_coarse);

  }

  virtual void updateGradientDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::VectorXd& gu) const override {
    // Determine stage variables
    const auto& ego_a = ctrls(0);
    const auto& ego_steer = ctrls(1);
    const auto& agent_0_a = ctrls(2);
    const auto& ego_steer_ref = params(4);
    const auto& steer_weight = params(24);
    const auto& ego_a_weight = params(27);
    const auto& agent_0_a_weight = params(30);
    const auto& agent_0_avoid_weight = params(31);
    const auto& obs_0_weight = params(32);
    const auto& obs_1_weight = params(33);
    const auto& a_scale = params(81);
    const auto& steer_scale = params(85);

    // Determine internal variables
    const double internal_1 = 0.5*agent_0_avoid_weight;
    const double internal_2 = 0.5*obs_0_weight;
    const double internal_3 = 0.5*obs_1_weight;
    const double internal_0 = pow(a_scale, -2);

    // Evaluation of Vector gu
    gu(0) = ego_a*ego_a_weight*internal_0;
    gu(1) = 0.5*steer_weight*(2*ego_steer - 2*ego_steer_ref)/pow(steer_scale, 2);
    gu(2) = agent_0_a*agent_0_a_weight*internal_0;
    gu(3) = internal_1;
    gu(4) = internal_1;
    gu(5) = internal_2;
    gu(6) = internal_2;
    gu(7) = internal_3;
    gu(8) = internal_3;

  }

  virtual void updateHessian(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& dxdx, Eigen::MatrixXd& dxdu, Eigen::MatrixXd& dudu) const override {
    // Determine stage variables
    const auto& ego_theta_ref = params(2);
    const auto& l_ref_weight = params(22);
    const auto& theta_ref_weight = params(23);
    const auto& steer_weight = params(24);
    const auto& ego_s_coarse_weight = params(25);
    const auto& ego_v_coarse_weight = params(26);
    const auto& ego_a_weight = params(27);
    const auto& agent_0_s_coarse_weight = params(28);
    const auto& agent_0_v_coarse_weight = params(29);
    const auto& agent_0_a_weight = params(30);
    const auto& s_scale = params(79);
    const auto& v_scale = params(80);
    const auto& a_scale = params(81);
    const auto& l_scale = params(83);
    const auto& theta_scale = params(84);
    const auto& steer_scale = params(85);

    // Determine internal variables
    const double internal_1 = sin(ego_theta_ref);
    const double internal_3 = cos(ego_theta_ref);
    const double internal_0 = pow(s_scale, -2);
    const double internal_5 = pow(v_scale, -2);
    const double internal_6 = pow(a_scale, -2);
    const double internal_2 = l_ref_weight/pow(l_scale, 2);
    const double internal_4 = -internal_1*internal_2*internal_3;

    // Evaluation of SymmetricMatrix dxdx
    dxdx(0, 0) = ego_s_coarse_weight*internal_0;
    dxdx(1, 1) = pow(internal_1, 2)*internal_2;
    dxdx(1, 2) = internal_4;
    dxdx(2, 2) = internal_2*pow(internal_3, 2);
    dxdx(3, 3) = theta_ref_weight/pow(theta_scale, 2);
    dxdx(4, 4) = ego_v_coarse_weight*internal_5;
    dxdx(5, 5) = agent_0_s_coarse_weight*internal_0;
    dxdx(9, 9) = agent_0_v_coarse_weight*internal_5;
    dxdx(2, 1) = dxdx(1, 2);

    // Evaluation of SymmetricMatrix dudu
    dudu(0, 0) = ego_a_weight*internal_6;
    dudu(1, 1) = steer_weight/pow(steer_scale, 2);
    dudu(2, 2) = agent_0_a_weight*internal_6;

  }

  virtual void updateHessianDxDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& dxdx) const override {
    // Determine stage variables
    const auto& ego_theta_ref = params(2);
    const auto& l_ref_weight = params(22);
    const auto& theta_ref_weight = params(23);
    const auto& ego_s_coarse_weight = params(25);
    const auto& ego_v_coarse_weight = params(26);
    const auto& agent_0_s_coarse_weight = params(28);
    const auto& agent_0_v_coarse_weight = params(29);
    const auto& s_scale = params(79);
    const auto& v_scale = params(80);
    const auto& l_scale = params(83);
    const auto& theta_scale = params(84);

    // Determine internal variables
    const double internal_1 = sin(ego_theta_ref);
    const double internal_3 = cos(ego_theta_ref);
    const double internal_0 = pow(s_scale, -2);
    const double internal_5 = pow(v_scale, -2);
    const double internal_2 = l_ref_weight/pow(l_scale, 2);
    const double internal_4 = -internal_1*internal_2*internal_3;

    // Evaluation of SymmetricMatrix dxdx
    dxdx(0, 0) = ego_s_coarse_weight*internal_0;
    dxdx(1, 1) = pow(internal_1, 2)*internal_2;
    dxdx(1, 2) = internal_4;
    dxdx(2, 2) = internal_2*pow(internal_3, 2);
    dxdx(3, 3) = theta_ref_weight/pow(theta_scale, 2);
    dxdx(4, 4) = ego_v_coarse_weight*internal_5;
    dxdx(5, 5) = agent_0_s_coarse_weight*internal_0;
    dxdx(9, 9) = agent_0_v_coarse_weight*internal_5;
    dxdx(2, 1) = dxdx(1, 2);

  }

  virtual void updateHessianDxDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& dxdu) const override {

  }

  virtual void updateHessianDuDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& dudu) const override {
    // Determine stage variables
    const auto& steer_weight = params(24);
    const auto& ego_a_weight = params(27);
    const auto& agent_0_a_weight = params(30);
    const auto& a_scale = params(81);
    const auto& steer_scale = params(85);

    // Determine internal variables
    const double internal_0 = pow(a_scale, -2);

    // Evaluation of SymmetricMatrix dudu
    dudu(0, 0) = ego_a_weight*internal_0;
    dudu(1, 1) = steer_weight/pow(steer_scale, 2);
    dudu(2, 2) = agent_0_a_weight*internal_0;

  }

  virtual void updateGradient(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t, Eigen::VectorXd& gx, Eigen::VectorXd& gu) const override {
    // Determine stage variables
    const auto& ego_s = states(0);
    const auto& ego_theta = states(3);
    const auto& ego_v = states(4);
    const auto& agent_0_s = states(5);
    const auto& agent_0_v = states(9);
    const auto& ego_a = ctrls(0);
    const auto& ego_steer = ctrls(1);
    const auto& agent_0_a = ctrls(2);
    const auto& ego_theta_ref = params(2);
    const auto& ego_steer_ref = params(4);
    const auto& ego_s_coarse = params(10);
    const auto& ego_v_coarse = params(11);
    const auto& agent_0_s_coarse = params(13);
    const auto& agent_0_v_coarse = params(14);
    const auto& l_offset = params(16);
    const auto& l_ref_weight = params(22);
    const auto& theta_ref_weight = params(23);
    const auto& steer_weight = params(24);
    const auto& ego_s_coarse_weight = params(25);
    const auto& ego_v_coarse_weight = params(26);
    const auto& ego_a_weight = params(27);
    const auto& agent_0_s_coarse_weight = params(28);
    const auto& agent_0_v_coarse_weight = params(29);
    const auto& agent_0_a_weight = params(30);
    const auto& agent_0_avoid_weight = params(31);
    const auto& obs_0_weight = params(32);
    const auto& obs_1_weight = params(33);
    const auto& s_scale = params(79);
    const auto& v_scale = params(80);
    const auto& a_scale = params(81);
    const auto& l_scale = params(83);
    const auto& theta_scale = params(84);
    const auto& steer_scale = params(85);

    // Determine global variables
    const auto& ego_l = globals(3);

    // Determine internal variables
    const double internal_0 = 0.5*delta_t;
    const double internal_2 = delta_t;
    const double internal_6 = agent_0_avoid_weight*internal_0;
    const double internal_7 = internal_0*obs_0_weight;
    const double internal_8 = internal_0*obs_1_weight;
    const double internal_1 = internal_0/pow(s_scale, 2);
    const double internal_4 = internal_0/pow(v_scale, 2);
    const double internal_5 = internal_2/pow(a_scale, 2);
    const double internal_3 = internal_2*l_ref_weight*(ego_l - l_offset)/pow(l_scale, 2);

    // Evaluation of Vector gx
    gx(0) = ego_s_coarse_weight*internal_1*(2*ego_s - 2*ego_s_coarse);
    gx(1) = -internal_3*sin(ego_theta_ref);
    gx(2) = internal_3*cos(ego_theta_ref);
    gx(3) = internal_0*theta_ref_weight*(2*ego_theta - 2*ego_theta_ref)/pow(theta_scale, 2);
    gx(4) = ego_v_coarse_weight*internal_4*(2*ego_v - 2*ego_v_coarse);
    gx(5) = agent_0_s_coarse_weight*internal_1*(2*agent_0_s - 2*agent_0_s_coarse);
    gx(9) = agent_0_v_coarse_weight*internal_4*(2*agent_0_v - 2*agent_0_v_coarse);

    // Evaluation of Vector gu
    gu(0) = ego_a*ego_a_weight*internal_5;
    gu(1) = internal_0*steer_weight*(2*ego_steer - 2*ego_steer_ref)/pow(steer_scale, 2);
    gu(2) = agent_0_a*agent_0_a_weight*internal_5;
    gu(3) = internal_6;
    gu(4) = internal_6;
    gu(5) = internal_7;
    gu(6) = internal_7;
    gu(7) = internal_8;
    gu(8) = internal_8;

  }

  virtual void updateGradientDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t, Eigen::VectorXd& gx) const override {
    // Determine stage variables
    const auto& ego_s = states(0);
    const auto& ego_theta = states(3);
    const auto& ego_v = states(4);
    const auto& agent_0_s = states(5);
    const auto& agent_0_v = states(9);
    const auto& ego_theta_ref = params(2);
    const auto& ego_s_coarse = params(10);
    const auto& ego_v_coarse = params(11);
    const auto& agent_0_s_coarse = params(13);
    const auto& agent_0_v_coarse = params(14);
    const auto& l_offset = params(16);
    const auto& l_ref_weight = params(22);
    const auto& theta_ref_weight = params(23);
    const auto& ego_s_coarse_weight = params(25);
    const auto& ego_v_coarse_weight = params(26);
    const auto& agent_0_s_coarse_weight = params(28);
    const auto& agent_0_v_coarse_weight = params(29);
    const auto& s_scale = params(79);
    const auto& v_scale = params(80);
    const auto& l_scale = params(83);
    const auto& theta_scale = params(84);

    // Determine global variables
    const auto& ego_l = globals(3);

    // Determine internal variables
    const double internal_0 = 0.5*delta_t;
    const double internal_1 = internal_0/pow(s_scale, 2);
    const double internal_3 = internal_0/pow(v_scale, 2);
    const double internal_2 = delta_t*l_ref_weight*(ego_l - l_offset)/pow(l_scale, 2);

    // Evaluation of Vector gx
    gx(0) = ego_s_coarse_weight*internal_1*(2*ego_s - 2*ego_s_coarse);
    gx(1) = -internal_2*sin(ego_theta_ref);
    gx(2) = internal_2*cos(ego_theta_ref);
    gx(3) = internal_0*theta_ref_weight*(2*ego_theta - 2*ego_theta_ref)/pow(theta_scale, 2);
    gx(4) = ego_v_coarse_weight*internal_3*(2*ego_v - 2*ego_v_coarse);
    gx(5) = agent_0_s_coarse_weight*internal_1*(2*agent_0_s - 2*agent_0_s_coarse);
    gx(9) = agent_0_v_coarse_weight*internal_3*(2*agent_0_v - 2*agent_0_v_coarse);

  }

  virtual void updateGradientDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t, Eigen::VectorXd& gu) const override {
    // Determine stage variables
    const auto& ego_a = ctrls(0);
    const auto& ego_steer = ctrls(1);
    const auto& agent_0_a = ctrls(2);
    const auto& ego_steer_ref = params(4);
    const auto& steer_weight = params(24);
    const auto& ego_a_weight = params(27);
    const auto& agent_0_a_weight = params(30);
    const auto& agent_0_avoid_weight = params(31);
    const auto& obs_0_weight = params(32);
    const auto& obs_1_weight = params(33);
    const auto& a_scale = params(81);
    const auto& steer_scale = params(85);

    // Determine internal variables
    const double internal_1 = 0.5*delta_t;
    const double internal_2 = agent_0_avoid_weight*internal_1;
    const double internal_3 = internal_1*obs_0_weight;
    const double internal_4 = internal_1*obs_1_weight;
    const double internal_0 = delta_t/pow(a_scale, 2);

    // Evaluation of Vector gu
    gu(0) = ego_a*ego_a_weight*internal_0;
    gu(1) = internal_1*steer_weight*(2*ego_steer - 2*ego_steer_ref)/pow(steer_scale, 2);
    gu(2) = agent_0_a*agent_0_a_weight*internal_0;
    gu(3) = internal_2;
    gu(4) = internal_2;
    gu(5) = internal_3;
    gu(6) = internal_3;
    gu(7) = internal_4;
    gu(8) = internal_4;

  }

  virtual void updateHessian(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t, Eigen::MatrixXd& dxdx, Eigen::MatrixXd& dxdu, Eigen::MatrixXd& dudu) const override {
    // Determine stage variables
    const auto& ego_theta_ref = params(2);
    const auto& l_ref_weight = params(22);
    const auto& theta_ref_weight = params(23);
    const auto& steer_weight = params(24);
    const auto& ego_s_coarse_weight = params(25);
    const auto& ego_v_coarse_weight = params(26);
    const auto& ego_a_weight = params(27);
    const auto& agent_0_s_coarse_weight = params(28);
    const auto& agent_0_v_coarse_weight = params(29);
    const auto& agent_0_a_weight = params(30);
    const auto& s_scale = params(79);
    const auto& v_scale = params(80);
    const auto& a_scale = params(81);
    const auto& l_scale = params(83);
    const auto& theta_scale = params(84);
    const auto& steer_scale = params(85);

    // Determine internal variables
    const double internal_0 = delta_t;
    const double internal_2 = sin(ego_theta_ref);
    const double internal_4 = cos(ego_theta_ref);
    const double internal_1 = internal_0/pow(s_scale, 2);
    const double internal_6 = internal_0/pow(v_scale, 2);
    const double internal_7 = internal_0/pow(a_scale, 2);
    const double internal_3 = internal_0*l_ref_weight/pow(l_scale, 2);
    const double internal_5 = -internal_2*internal_3*internal_4;

    // Evaluation of SymmetricMatrix dxdx
    dxdx(0, 0) = ego_s_coarse_weight*internal_1;
    dxdx(1, 1) = pow(internal_2, 2)*internal_3;
    dxdx(1, 2) = internal_5;
    dxdx(2, 2) = internal_3*pow(internal_4, 2);
    dxdx(3, 3) = internal_0*theta_ref_weight/pow(theta_scale, 2);
    dxdx(4, 4) = ego_v_coarse_weight*internal_6;
    dxdx(5, 5) = agent_0_s_coarse_weight*internal_1;
    dxdx(9, 9) = agent_0_v_coarse_weight*internal_6;
    dxdx(2, 1) = dxdx(1, 2);

    // Evaluation of SymmetricMatrix dudu
    dudu(0, 0) = ego_a_weight*internal_7;
    dudu(1, 1) = internal_0*steer_weight/pow(steer_scale, 2);
    dudu(2, 2) = agent_0_a_weight*internal_7;

  }

  virtual void updateHessianDxDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t, Eigen::MatrixXd& dxdx) const override {
    // Determine stage variables
    const auto& ego_theta_ref = params(2);
    const auto& l_ref_weight = params(22);
    const auto& theta_ref_weight = params(23);
    const auto& ego_s_coarse_weight = params(25);
    const auto& ego_v_coarse_weight = params(26);
    const auto& agent_0_s_coarse_weight = params(28);
    const auto& agent_0_v_coarse_weight = params(29);
    const auto& s_scale = params(79);
    const auto& v_scale = params(80);
    const auto& l_scale = params(83);
    const auto& theta_scale = params(84);

    // Determine internal variables
    const double internal_0 = delta_t;
    const double internal_2 = sin(ego_theta_ref);
    const double internal_4 = cos(ego_theta_ref);
    const double internal_1 = internal_0/pow(s_scale, 2);
    const double internal_6 = internal_0/pow(v_scale, 2);
    const double internal_3 = internal_0*l_ref_weight/pow(l_scale, 2);
    const double internal_5 = -internal_2*internal_3*internal_4;

    // Evaluation of SymmetricMatrix dxdx
    dxdx(0, 0) = ego_s_coarse_weight*internal_1;
    dxdx(1, 1) = pow(internal_2, 2)*internal_3;
    dxdx(1, 2) = internal_5;
    dxdx(2, 2) = internal_3*pow(internal_4, 2);
    dxdx(3, 3) = internal_0*theta_ref_weight/pow(theta_scale, 2);
    dxdx(4, 4) = ego_v_coarse_weight*internal_6;
    dxdx(5, 5) = agent_0_s_coarse_weight*internal_1;
    dxdx(9, 9) = agent_0_v_coarse_weight*internal_6;
    dxdx(2, 1) = dxdx(1, 2);

  }

  virtual void updateHessianDxDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t, Eigen::MatrixXd& dxdu) const override {

  }

  virtual void updateHessianDuDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t, Eigen::MatrixXd& dudu) const override {
    // Determine stage variables
    const auto& steer_weight = params(24);
    const auto& ego_a_weight = params(27);
    const auto& agent_0_a_weight = params(30);
    const auto& a_scale = params(81);
    const auto& steer_scale = params(85);

    // Determine internal variables
    const double internal_0 = delta_t;
    const double internal_1 = internal_0/pow(a_scale, 2);

    // Evaluation of SymmetricMatrix dudu
    dudu(0, 0) = ego_a_weight*internal_1;
    dudu(1, 1) = internal_0*steer_weight/pow(steer_scale, 2);
    dudu(2, 2) = agent_0_a_weight*internal_1;

  }

};

}  //  namespace gpal::pnc::planning