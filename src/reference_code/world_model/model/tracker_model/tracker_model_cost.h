#pragma once

#include "ocp/cost_function.h"

namespace gpal::pnc::planning {

template <StageType Ttype>
class TrackerModelCost : public CostFunction {
 public:
  TrackerModelCost() : CostFunction(5, 2) {
  }

  virtual ~TrackerModelCost() = default;

  virtual double evaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls) const override {
    // Determine stage variables
    const auto& theta = states(3);
    const auto& steer = states(4);
    const auto& dsteer = ctrls(0);
    const auto& l_slack = ctrls(1);
    const auto& theta_ref = params(2);
    const auto& l_ref_weight = params(11);
    const auto& theta_ref_weight = params(12);
    const auto& steer_weight = params(13);
    const auto& dsteer_weight = params(14);
    const auto& l_slack_weight = params(15);
    const auto& l_scale = params(24);
    const auto& theta_scale = params(25);
    const auto& steer_scale = params(26);
    const auto& dsteer_scale = params(27);

    // Determine global variables
    const auto& l = globals(1);

    // Determine internal variables
    const double internal_0 = 0.5/pow(l_scale, 2);

    // Evaluation of Scalar cost
    double cost = 0.5*pow(dsteer, 2)*dsteer_weight/pow(dsteer_scale, 2) + internal_0*pow(l, 2)*l_ref_weight + internal_0*pow(l_slack, 2)*l_slack_weight + 0.5*pow(steer, 2)*steer_weight/pow(steer_scale, 2) + 0.5*theta_ref_weight*pow(theta - theta_ref, 2)/pow(theta_scale, 2);

    return cost;
  }

  virtual void updateGradient(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::VectorXd& gx, Eigen::VectorXd& gu) const override {
    // Determine stage variables
    const auto& theta = states(3);
    const auto& steer = states(4);
    const auto& dsteer = ctrls(0);
    const auto& l_slack = ctrls(1);
    const auto& theta_ref = params(2);
    const auto& l_ref_weight = params(11);
    const auto& theta_ref_weight = params(12);
    const auto& steer_weight = params(13);
    const auto& dsteer_weight = params(14);
    const auto& l_slack_weight = params(15);
    const auto& l_scale = params(24);
    const auto& theta_scale = params(25);
    const auto& steer_scale = params(26);
    const auto& dsteer_scale = params(27);

    // Determine global variables
    const auto& l = globals(1);

    // Determine internal variables
    const double internal_0 = pow(l_scale, -2);
    const double internal_1 = internal_0*l*l_ref_weight;

    // Evaluation of Vector gx
    gx(1) = -internal_1*sin(theta_ref);
    gx(2) = internal_1*cos(theta_ref);
    gx(3) = 0.5*theta_ref_weight*(2*theta - 2*theta_ref)/pow(theta_scale, 2);
    gx(4) = steer*steer_weight/pow(steer_scale, 2);

    // Evaluation of Vector gu
    gu(0) = dsteer*dsteer_weight/pow(dsteer_scale, 2);
    gu(1) = internal_0*l_slack*l_slack_weight;

  }

  virtual void updateGradientDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::VectorXd& gx) const override {
    // Determine stage variables
    const auto& theta = states(3);
    const auto& steer = states(4);
    const auto& theta_ref = params(2);
    const auto& l_ref_weight = params(11);
    const auto& theta_ref_weight = params(12);
    const auto& steer_weight = params(13);
    const auto& l_scale = params(24);
    const auto& theta_scale = params(25);
    const auto& steer_scale = params(26);

    // Determine global variables
    const auto& l = globals(1);

    // Determine internal variables
    const double internal_0 = l*l_ref_weight/pow(l_scale, 2);

    // Evaluation of Vector gx
    gx(1) = -internal_0*sin(theta_ref);
    gx(2) = internal_0*cos(theta_ref);
    gx(3) = 0.5*theta_ref_weight*(2*theta - 2*theta_ref)/pow(theta_scale, 2);
    gx(4) = steer*steer_weight/pow(steer_scale, 2);

  }

  virtual void updateGradientDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::VectorXd& gu) const override {
    // Determine stage variables
    const auto& dsteer = ctrls(0);
    const auto& l_slack = ctrls(1);
    const auto& dsteer_weight = params(14);
    const auto& l_slack_weight = params(15);
    const auto& l_scale = params(24);
    const auto& dsteer_scale = params(27);

    // Evaluation of Vector gu
    gu(0) = dsteer*dsteer_weight/pow(dsteer_scale, 2);
    gu(1) = l_slack*l_slack_weight/pow(l_scale, 2);

  }

  virtual void updateHessian(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& dxdx, Eigen::MatrixXd& dxdu, Eigen::MatrixXd& dudu) const override {
    // Determine stage variables
    const auto& theta_ref = params(2);
    const auto& l_ref_weight = params(11);
    const auto& theta_ref_weight = params(12);
    const auto& steer_weight = params(13);
    const auto& dsteer_weight = params(14);
    const auto& l_slack_weight = params(15);
    const auto& l_scale = params(24);
    const auto& theta_scale = params(25);
    const auto& steer_scale = params(26);
    const auto& dsteer_scale = params(27);

    // Determine internal variables
    const double internal_0 = sin(theta_ref);
    const double internal_3 = cos(theta_ref);
    const double internal_1 = pow(l_scale, -2);
    const double internal_2 = internal_1*l_ref_weight;
    const double internal_4 = -internal_0*internal_2*internal_3;

    // Evaluation of SymmetricMatrix dxdx
    dxdx(1, 1) = pow(internal_0, 2)*internal_2;
    dxdx(1, 2) = internal_4;
    dxdx(2, 2) = internal_2*pow(internal_3, 2);
    dxdx(3, 3) = theta_ref_weight/pow(theta_scale, 2);
    dxdx(4, 4) = steer_weight/pow(steer_scale, 2);
    dxdx(2, 1) = dxdx(1, 2);

    // Evaluation of SymmetricMatrix dudu
    dudu(0, 0) = dsteer_weight/pow(dsteer_scale, 2);
    dudu(1, 1) = internal_1*l_slack_weight;

  }

  virtual void updateHessianDxDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& dxdx) const override {
    // Determine stage variables
    const auto& theta_ref = params(2);
    const auto& l_ref_weight = params(11);
    const auto& theta_ref_weight = params(12);
    const auto& steer_weight = params(13);
    const auto& l_scale = params(24);
    const auto& theta_scale = params(25);
    const auto& steer_scale = params(26);

    // Determine internal variables
    const double internal_0 = sin(theta_ref);
    const double internal_2 = cos(theta_ref);
    const double internal_1 = l_ref_weight/pow(l_scale, 2);
    const double internal_3 = -internal_0*internal_1*internal_2;

    // Evaluation of SymmetricMatrix dxdx
    dxdx(1, 1) = pow(internal_0, 2)*internal_1;
    dxdx(1, 2) = internal_3;
    dxdx(2, 2) = internal_1*pow(internal_2, 2);
    dxdx(3, 3) = theta_ref_weight/pow(theta_scale, 2);
    dxdx(4, 4) = steer_weight/pow(steer_scale, 2);
    dxdx(2, 1) = dxdx(1, 2);

  }

  virtual void updateHessianDxDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& dxdu) const override {

  }

  virtual void updateHessianDuDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& dudu) const override {
    // Determine stage variables
    const auto& dsteer_weight = params(14);
    const auto& l_slack_weight = params(15);
    const auto& l_scale = params(24);
    const auto& dsteer_scale = params(27);

    // Evaluation of SymmetricMatrix dudu
    dudu(0, 0) = dsteer_weight/pow(dsteer_scale, 2);
    dudu(1, 1) = l_slack_weight/pow(l_scale, 2);

  }

  virtual void updateGradient(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t, Eigen::VectorXd& gx, Eigen::VectorXd& gu) const override {
    // Determine stage variables
    const auto& theta = states(3);
    const auto& steer = states(4);
    const auto& dsteer = ctrls(0);
    const auto& l_slack = ctrls(1);
    const auto& theta_ref = params(2);
    const auto& l_ref_weight = params(11);
    const auto& theta_ref_weight = params(12);
    const auto& steer_weight = params(13);
    const auto& dsteer_weight = params(14);
    const auto& l_slack_weight = params(15);
    const auto& l_scale = params(24);
    const auto& theta_scale = params(25);
    const auto& steer_scale = params(26);
    const auto& dsteer_scale = params(27);

    // Determine global variables
    const auto& l = globals(1);

    // Determine internal variables
    const double internal_0 = delta_t;
    const double internal_1 = internal_0/pow(l_scale, 2);
    const double internal_2 = internal_1*l*l_ref_weight;

    // Evaluation of Vector gx
    gx(1) = -internal_2*sin(theta_ref);
    gx(2) = internal_2*cos(theta_ref);
    gx(3) = 0.5*delta_t*theta_ref_weight*(2*theta - 2*theta_ref)/pow(theta_scale, 2);
    gx(4) = internal_0*steer*steer_weight/pow(steer_scale, 2);

    // Evaluation of Vector gu
    gu(0) = dsteer*dsteer_weight*internal_0/pow(dsteer_scale, 2);
    gu(1) = internal_1*l_slack*l_slack_weight;

  }

  virtual void updateGradientDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t, Eigen::VectorXd& gx) const override {
    // Determine stage variables
    const auto& theta = states(3);
    const auto& steer = states(4);
    const auto& theta_ref = params(2);
    const auto& l_ref_weight = params(11);
    const auto& theta_ref_weight = params(12);
    const auto& steer_weight = params(13);
    const auto& l_scale = params(24);
    const auto& theta_scale = params(25);
    const auto& steer_scale = params(26);

    // Determine global variables
    const auto& l = globals(1);

    // Determine internal variables
    const double internal_0 = delta_t;
    const double internal_1 = internal_0*l*l_ref_weight/pow(l_scale, 2);

    // Evaluation of Vector gx
    gx(1) = -internal_1*sin(theta_ref);
    gx(2) = internal_1*cos(theta_ref);
    gx(3) = 0.5*delta_t*theta_ref_weight*(2*theta - 2*theta_ref)/pow(theta_scale, 2);
    gx(4) = internal_0*steer*steer_weight/pow(steer_scale, 2);

  }

  virtual void updateGradientDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t, Eigen::VectorXd& gu) const override {
    // Determine stage variables
    const auto& dsteer = ctrls(0);
    const auto& l_slack = ctrls(1);
    const auto& dsteer_weight = params(14);
    const auto& l_slack_weight = params(15);
    const auto& l_scale = params(24);
    const auto& dsteer_scale = params(27);

    // Determine internal variables
    const double internal_0 = delta_t;

    // Evaluation of Vector gu
    gu(0) = dsteer*dsteer_weight*internal_0/pow(dsteer_scale, 2);
    gu(1) = internal_0*l_slack*l_slack_weight/pow(l_scale, 2);

  }

  virtual void updateHessian(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t, Eigen::MatrixXd& dxdx, Eigen::MatrixXd& dxdu, Eigen::MatrixXd& dudu) const override {
    // Determine stage variables
    const auto& theta_ref = params(2);
    const auto& l_ref_weight = params(11);
    const auto& theta_ref_weight = params(12);
    const auto& steer_weight = params(13);
    const auto& dsteer_weight = params(14);
    const auto& l_slack_weight = params(15);
    const auto& l_scale = params(24);
    const auto& theta_scale = params(25);
    const auto& steer_scale = params(26);
    const auto& dsteer_scale = params(27);

    // Determine internal variables
    const double internal_1 = delta_t;
    const double internal_0 = sin(theta_ref);
    const double internal_4 = cos(theta_ref);
    const double internal_2 = internal_1/pow(l_scale, 2);
    const double internal_3 = internal_2*l_ref_weight;
    const double internal_5 = -internal_0*internal_3*internal_4;

    // Evaluation of SymmetricMatrix dxdx
    dxdx(1, 1) = pow(internal_0, 2)*internal_3;
    dxdx(1, 2) = internal_5;
    dxdx(2, 2) = internal_3*pow(internal_4, 2);
    dxdx(3, 3) = internal_1*theta_ref_weight/pow(theta_scale, 2);
    dxdx(4, 4) = internal_1*steer_weight/pow(steer_scale, 2);
    dxdx(2, 1) = dxdx(1, 2);

    // Evaluation of SymmetricMatrix dudu
    dudu(0, 0) = dsteer_weight*internal_1/pow(dsteer_scale, 2);
    dudu(1, 1) = internal_2*l_slack_weight;

  }

  virtual void updateHessianDxDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t, Eigen::MatrixXd& dxdx) const override {
    // Determine stage variables
    const auto& theta_ref = params(2);
    const auto& l_ref_weight = params(11);
    const auto& theta_ref_weight = params(12);
    const auto& steer_weight = params(13);
    const auto& l_scale = params(24);
    const auto& theta_scale = params(25);
    const auto& steer_scale = params(26);

    // Determine internal variables
    const double internal_1 = delta_t;
    const double internal_0 = sin(theta_ref);
    const double internal_3 = cos(theta_ref);
    const double internal_2 = internal_1*l_ref_weight/pow(l_scale, 2);
    const double internal_4 = -internal_0*internal_2*internal_3;

    // Evaluation of SymmetricMatrix dxdx
    dxdx(1, 1) = pow(internal_0, 2)*internal_2;
    dxdx(1, 2) = internal_4;
    dxdx(2, 2) = internal_2*pow(internal_3, 2);
    dxdx(3, 3) = internal_1*theta_ref_weight/pow(theta_scale, 2);
    dxdx(4, 4) = internal_1*steer_weight/pow(steer_scale, 2);
    dxdx(2, 1) = dxdx(1, 2);

  }

  virtual void updateHessianDxDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t, Eigen::MatrixXd& dxdu) const override {

  }

  virtual void updateHessianDuDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t, Eigen::MatrixXd& dudu) const override {
    // Determine stage variables
    const auto& dsteer_weight = params(14);
    const auto& l_slack_weight = params(15);
    const auto& l_scale = params(24);
    const auto& dsteer_scale = params(27);

    // Determine internal variables
    const double internal_0 = delta_t;

    // Evaluation of SymmetricMatrix dudu
    dudu(0, 0) = dsteer_weight*internal_0/pow(dsteer_scale, 2);
    dudu(1, 1) = internal_0*l_slack_weight/pow(l_scale, 2);

  }

};

template <>
class TrackerModelCost<StageType::TERMINAL> : public CostFunction {
 public:
  TrackerModelCost() : CostFunction(5, 2) {}
  virtual ~TrackerModelCost() = default;

  virtual double evaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls) const override {
    // Determine stage variables
    const auto& theta = states(3);
    const auto& theta_ref = params(2);
    const auto& terminal_l_weight = params(16);
    const auto& terminal_theta_weight = params(17);
    const auto& l_scale = params(24);
    const auto& theta_scale = params(25);

    // Determine global variables
    const auto& l = globals(1);

    // Evaluation of Scalar cost
    double cost = 0.5*pow(l, 2)*terminal_l_weight/pow(l_scale, 2) + 0.5*terminal_theta_weight*pow(theta - theta_ref, 2)/pow(theta_scale, 2);

    return cost;
  }

  virtual void updateGradientDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::VectorXd& gx) const override {
    // Determine stage variables
    const auto& theta = states(3);
    const auto& theta_ref = params(2);
    const auto& terminal_l_weight = params(16);
    const auto& terminal_theta_weight = params(17);
    const auto& l_scale = params(24);
    const auto& theta_scale = params(25);

    // Determine global variables
    const auto& l = globals(1);

    // Determine internal variables
    const double internal_0 = l*terminal_l_weight/pow(l_scale, 2);

    // Evaluation of Vector gx
    gx(1) = -internal_0*sin(theta_ref);
    gx(2) = internal_0*cos(theta_ref);
    gx(3) = 0.5*terminal_theta_weight*(2*theta - 2*theta_ref)/pow(theta_scale, 2);

  }

  virtual void updateHessianDxDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& hxx) const override {
    // Determine stage variables
    const auto& theta_ref = params(2);
    const auto& terminal_l_weight = params(16);
    const auto& terminal_theta_weight = params(17);
    const auto& l_scale = params(24);
    const auto& theta_scale = params(25);

    // Determine internal variables
    const double internal_0 = sin(theta_ref);
    const double internal_2 = cos(theta_ref);
    const double internal_1 = terminal_l_weight/pow(l_scale, 2);
    const double internal_3 = -internal_0*internal_1*internal_2;

    // Evaluation of SymmetricMatrix hxx
    hxx(1, 1) = pow(internal_0, 2)*internal_1;
    hxx(1, 2) = internal_3;
    hxx(2, 2) = internal_1*pow(internal_2, 2);
    hxx(3, 3) = terminal_theta_weight/pow(theta_scale, 2);
    hxx(2, 1) = hxx(1, 2);

  }

};

}  //  namespace gpal::pnc::planning