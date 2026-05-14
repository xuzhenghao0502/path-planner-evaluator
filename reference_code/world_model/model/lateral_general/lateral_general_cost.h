#pragma once

#include "ocp/cost_function.h"

namespace gpal::pnc::planning {

template <StageType Ttype>
class LateralGeneralCost : public CostFunction {
 public:
  LateralGeneralCost() : CostFunction(5, 2) {
  }

  virtual ~LateralGeneralCost() = default;

  virtual double evaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls) const override {
    // Determine stage variables
    const auto& theta = states(3);
    const auto& steer = states(4);
    const auto& dsteer = ctrls(0);
    const auto& slack_offset = ctrls(1);
    const auto& thetar = params(3);
    const auto& l_weight = params(5);
    const auto& theta_ref_weight = params(6);
    const auto& steer_weight = params(7);
    const auto& dsteer_weight = params(8);
    const auto& weight_slack_offset = params(9);
    const auto& weight_prev_steer = params(36);
    const auto& prev_steer = params(38);
    const auto& l_offset = params(40);

    // Determine global variables
    const auto& l = globals(1);

    // Evaluation of Scalar cost
    double cost = 0.5*pow(dsteer, 2)*dsteer_weight + 0.5*l_weight*pow(l - l_offset, 2) + 0.5*pow(slack_offset, 2)*weight_slack_offset + 0.5*pow(steer, 2)*steer_weight + 0.5*theta_ref_weight*pow(theta - thetar, 2) + 0.5*weight_prev_steer*pow(-prev_steer + steer, 2);

    return cost;
  }

  virtual void updateGradient(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::VectorXd& gx, Eigen::VectorXd& gu) const override {
    // Determine stage variables
    const auto& theta = states(3);
    const auto& steer = states(4);
    const auto& dsteer = ctrls(0);
    const auto& slack_offset = ctrls(1);
    const auto& thetar = params(3);
    const auto& l_weight = params(5);
    const auto& theta_ref_weight = params(6);
    const auto& steer_weight = params(7);
    const auto& dsteer_weight = params(8);
    const auto& weight_slack_offset = params(9);
    const auto& weight_prev_steer = params(36);
    const auto& prev_steer = params(38);
    const auto& l_offset = params(40);

    // Determine global variables
    const auto& l = globals(1);

    // Determine internal variables
    const double internal_0 = l_weight*(l - l_offset);

    // Evaluation of Vector gx
    gx(1) = -internal_0*sin(thetar);
    gx(2) = internal_0*cos(thetar);
    gx(3) = 0.5*theta_ref_weight*(2*theta - 2*thetar);
    gx(4) = steer*steer_weight + 0.5*weight_prev_steer*(-2*prev_steer + 2*steer);

    // Evaluation of Vector gu
    gu(0) = dsteer*dsteer_weight;
    gu(1) = slack_offset*weight_slack_offset;

  }

  virtual void updateGradientDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::VectorXd& gx) const override {
    // Determine stage variables
    const auto& theta = states(3);
    const auto& steer = states(4);
    const auto& thetar = params(3);
    const auto& l_weight = params(5);
    const auto& theta_ref_weight = params(6);
    const auto& steer_weight = params(7);
    const auto& weight_prev_steer = params(36);
    const auto& prev_steer = params(38);
    const auto& l_offset = params(40);

    // Determine global variables
    const auto& l = globals(1);

    // Determine internal variables
    const double internal_0 = l_weight*(l - l_offset);

    // Evaluation of Vector gx
    gx(1) = -internal_0*sin(thetar);
    gx(2) = internal_0*cos(thetar);
    gx(3) = 0.5*theta_ref_weight*(2*theta - 2*thetar);
    gx(4) = steer*steer_weight + 0.5*weight_prev_steer*(-2*prev_steer + 2*steer);

  }

  virtual void updateGradientDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::VectorXd& gu) const override {
    // Determine stage variables
    const auto& dsteer = ctrls(0);
    const auto& slack_offset = ctrls(1);
    const auto& dsteer_weight = params(8);
    const auto& weight_slack_offset = params(9);

    // Evaluation of Vector gu
    gu(0) = dsteer*dsteer_weight;
    gu(1) = slack_offset*weight_slack_offset;

  }

  virtual void updateHessian(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& dxdx, Eigen::MatrixXd& dxdu, Eigen::MatrixXd& dudu) const override {
    // Determine stage variables
    const auto& thetar = params(3);
    const auto& l_weight = params(5);
    const auto& theta_ref_weight = params(6);
    const auto& steer_weight = params(7);
    const auto& dsteer_weight = params(8);
    const auto& weight_slack_offset = params(9);
    const auto& weight_prev_steer = params(36);

    // Determine internal variables
    const double internal_1 = l_weight;
    const double internal_0 = sin(thetar);
    const double internal_2 = cos(thetar);
    const double internal_3 = -internal_0*internal_1*internal_2;

    // Evaluation of SymmetricMatrix dxdx
    dxdx(1, 1) = pow(internal_0, 2)*internal_1;
    dxdx(1, 2) = internal_3;
    dxdx(2, 2) = internal_1*pow(internal_2, 2);
    dxdx(3, 3) = theta_ref_weight;
    dxdx(4, 4) = steer_weight + weight_prev_steer;
    dxdx(2, 1) = dxdx(1, 2);

    // Evaluation of SymmetricMatrix dudu
    dudu(0, 0) = dsteer_weight;
    dudu(1, 1) = weight_slack_offset;

  }

  virtual void updateHessianDxDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& dxdx) const override {
    // Determine stage variables
    const auto& thetar = params(3);
    const auto& l_weight = params(5);
    const auto& theta_ref_weight = params(6);
    const auto& steer_weight = params(7);
    const auto& weight_prev_steer = params(36);

    // Determine internal variables
    const double internal_1 = l_weight;
    const double internal_0 = sin(thetar);
    const double internal_2 = cos(thetar);
    const double internal_3 = -internal_0*internal_1*internal_2;

    // Evaluation of SymmetricMatrix dxdx
    dxdx(1, 1) = pow(internal_0, 2)*internal_1;
    dxdx(1, 2) = internal_3;
    dxdx(2, 2) = internal_1*pow(internal_2, 2);
    dxdx(3, 3) = theta_ref_weight;
    dxdx(4, 4) = steer_weight + weight_prev_steer;
    dxdx(2, 1) = dxdx(1, 2);

  }

  virtual void updateHessianDxDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& dxdu) const override {

  }

  virtual void updateHessianDuDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& dudu) const override {
    // Determine stage variables
    const auto& dsteer_weight = params(8);
    const auto& weight_slack_offset = params(9);

    // Evaluation of SymmetricMatrix dudu
    dudu(0, 0) = dsteer_weight;
    dudu(1, 1) = weight_slack_offset;

  }

  virtual void updateGradient(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t, Eigen::VectorXd& gx, Eigen::VectorXd& gu) const override {
    // Determine stage variables
    const auto& theta = states(3);
    const auto& steer = states(4);
    const auto& dsteer = ctrls(0);
    const auto& slack_offset = ctrls(1);
    const auto& thetar = params(3);
    const auto& l_weight = params(5);
    const auto& theta_ref_weight = params(6);
    const auto& steer_weight = params(7);
    const auto& dsteer_weight = params(8);
    const auto& weight_slack_offset = params(9);
    const auto& weight_prev_steer = params(36);
    const auto& prev_steer = params(38);
    const auto& l_offset = params(40);

    // Determine global variables
    const auto& l = globals(1);

    // Determine internal variables
    const double internal_0 = delta_t;
    const double internal_1 = internal_0*l_weight*(l - l_offset);

    // Evaluation of Vector gx
    gx(1) = -internal_1*sin(thetar);
    gx(2) = internal_1*cos(thetar);
    gx(3) = 0.5*delta_t*theta_ref_weight*(2*theta - 2*thetar);
    gx(4) = delta_t*(steer*steer_weight + 0.5*weight_prev_steer*(-2*prev_steer + 2*steer));

    // Evaluation of Vector gu
    gu(0) = dsteer*dsteer_weight*internal_0;
    gu(1) = internal_0*slack_offset*weight_slack_offset;

  }

  virtual void updateGradientDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t, Eigen::VectorXd& gx) const override {
    // Determine stage variables
    const auto& theta = states(3);
    const auto& steer = states(4);
    const auto& thetar = params(3);
    const auto& l_weight = params(5);
    const auto& theta_ref_weight = params(6);
    const auto& steer_weight = params(7);
    const auto& weight_prev_steer = params(36);
    const auto& prev_steer = params(38);
    const auto& l_offset = params(40);

    // Determine global variables
    const auto& l = globals(1);

    // Determine internal variables
    const double internal_0 = delta_t*l_weight*(l - l_offset);

    // Evaluation of Vector gx
    gx(1) = -internal_0*sin(thetar);
    gx(2) = internal_0*cos(thetar);
    gx(3) = 0.5*delta_t*theta_ref_weight*(2*theta - 2*thetar);
    gx(4) = delta_t*(steer*steer_weight + 0.5*weight_prev_steer*(-2*prev_steer + 2*steer));

  }

  virtual void updateGradientDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t, Eigen::VectorXd& gu) const override {
    // Determine stage variables
    const auto& dsteer = ctrls(0);
    const auto& slack_offset = ctrls(1);
    const auto& dsteer_weight = params(8);
    const auto& weight_slack_offset = params(9);

    // Determine internal variables
    const double internal_0 = delta_t;

    // Evaluation of Vector gu
    gu(0) = dsteer*dsteer_weight*internal_0;
    gu(1) = internal_0*slack_offset*weight_slack_offset;

  }

  virtual void updateHessian(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t, Eigen::MatrixXd& dxdx, Eigen::MatrixXd& dxdu, Eigen::MatrixXd& dudu) const override {
    // Determine stage variables
    const auto& thetar = params(3);
    const auto& l_weight = params(5);
    const auto& theta_ref_weight = params(6);
    const auto& steer_weight = params(7);
    const auto& dsteer_weight = params(8);
    const auto& weight_slack_offset = params(9);
    const auto& weight_prev_steer = params(36);

    // Determine internal variables
    const double internal_1 = delta_t;
    const double internal_0 = sin(thetar);
    const double internal_3 = cos(thetar);
    const double internal_2 = internal_1*l_weight;
    const double internal_4 = -internal_0*internal_2*internal_3;

    // Evaluation of SymmetricMatrix dxdx
    dxdx(1, 1) = pow(internal_0, 2)*internal_2;
    dxdx(1, 2) = internal_4;
    dxdx(2, 2) = internal_2*pow(internal_3, 2);
    dxdx(3, 3) = internal_1*theta_ref_weight;
    dxdx(4, 4) = delta_t*(steer_weight + weight_prev_steer);
    dxdx(2, 1) = dxdx(1, 2);

    // Evaluation of SymmetricMatrix dudu
    dudu(0, 0) = dsteer_weight*internal_1;
    dudu(1, 1) = internal_1*weight_slack_offset;

  }

  virtual void updateHessianDxDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t, Eigen::MatrixXd& dxdx) const override {
    // Determine stage variables
    const auto& thetar = params(3);
    const auto& l_weight = params(5);
    const auto& theta_ref_weight = params(6);
    const auto& steer_weight = params(7);
    const auto& weight_prev_steer = params(36);

    // Determine internal variables
    const double internal_1 = delta_t;
    const double internal_0 = sin(thetar);
    const double internal_3 = cos(thetar);
    const double internal_2 = internal_1*l_weight;
    const double internal_4 = -internal_0*internal_2*internal_3;

    // Evaluation of SymmetricMatrix dxdx
    dxdx(1, 1) = pow(internal_0, 2)*internal_2;
    dxdx(1, 2) = internal_4;
    dxdx(2, 2) = internal_2*pow(internal_3, 2);
    dxdx(3, 3) = internal_1*theta_ref_weight;
    dxdx(4, 4) = delta_t*(steer_weight + weight_prev_steer);
    dxdx(2, 1) = dxdx(1, 2);

  }

  virtual void updateHessianDxDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t, Eigen::MatrixXd& dxdu) const override {

  }

  virtual void updateHessianDuDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t, Eigen::MatrixXd& dudu) const override {
    // Determine stage variables
    const auto& dsteer_weight = params(8);
    const auto& weight_slack_offset = params(9);

    // Determine internal variables
    const double internal_0 = delta_t;

    // Evaluation of SymmetricMatrix dudu
    dudu(0, 0) = dsteer_weight*internal_0;
    dudu(1, 1) = internal_0*weight_slack_offset;

  }

};

template <>
class LateralGeneralCost<StageType::TERMINAL> : public CostFunction {
 public:
  LateralGeneralCost() : CostFunction(5, 2) {}
  virtual ~LateralGeneralCost() = default;

  virtual double evaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls) const override {
    // Determine stage variables
    const auto& theta = states(3);
    const auto& thetar = params(3);
    const auto& terminal_l_weight = params(10);
    const auto& terminal_heading_weight = params(11);

    // Determine global variables
    const auto& l = globals(1);

    // Evaluation of Scalar cost
    double cost = 0.5*pow(l, 2)*terminal_l_weight + 0.5*terminal_heading_weight*pow(theta - thetar, 2);

    return cost;
  }

  virtual void updateGradientDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::VectorXd& gx) const override {
    // Determine stage variables
    const auto& theta = states(3);
    const auto& thetar = params(3);
    const auto& terminal_l_weight = params(10);
    const auto& terminal_heading_weight = params(11);

    // Determine global variables
    const auto& l = globals(1);

    // Determine internal variables
    const double internal_0 = l*terminal_l_weight;

    // Evaluation of Vector gx
    gx(1) = -internal_0*sin(thetar);
    gx(2) = internal_0*cos(thetar);
    gx(3) = 0.5*terminal_heading_weight*(2*theta - 2*thetar);

  }

  virtual void updateHessianDxDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& hxx) const override {
    // Determine stage variables
    const auto& thetar = params(3);
    const auto& terminal_l_weight = params(10);
    const auto& terminal_heading_weight = params(11);

    // Determine internal variables
    const double internal_1 = terminal_l_weight;
    const double internal_0 = sin(thetar);
    const double internal_2 = cos(thetar);
    const double internal_3 = -internal_0*internal_1*internal_2;

    // Evaluation of SymmetricMatrix hxx
    hxx(1, 1) = pow(internal_0, 2)*internal_1;
    hxx(1, 2) = internal_3;
    hxx(2, 2) = internal_1*pow(internal_2, 2);
    hxx(3, 3) = terminal_heading_weight;
    hxx(2, 1) = hxx(1, 2);

  }

};

}  //  namespace gpal::pnc::planning