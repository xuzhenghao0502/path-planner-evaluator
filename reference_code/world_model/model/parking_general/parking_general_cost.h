#pragma once

#include "ocp/cost_function.h"

namespace gpal::pnc::planning {

template <StageType Ttype>
class ParkingGeneralCost : public CostFunction {
 public:
  ParkingGeneralCost() : CostFunction(6, 3) {
  }

  virtual ~ParkingGeneralCost() = default;

  virtual double evaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls) const override {
    // Determine stage variables
    const auto& steer = states(4);
    const auto& v = states(5);
    const auto& dsteer = ctrls(0);
    const auto& a = ctrls(1);
    const auto& slack_offset = ctrls(2);
    const auto& vr = params(5);
    const auto& ref_weight = params(6);
    const auto& steer_weight = params(7);
    const auto& dsteer_weight = params(8);
    const auto& v_weight = params(12);
    const auto& a_weight = params(13);
    const auto& weigth_slack_offset = params(14);

    // Determine global variables
    const auto& l = globals(1);

    // Evaluation of Scalar cost
    double cost = 0.5*pow(a, 2)*a_weight + 0.5*pow(dsteer, 2)*dsteer_weight + 0.5*pow(l, 2)*ref_weight + 0.5*pow(slack_offset, 2)*weigth_slack_offset + 0.5*pow(steer, 2)*steer_weight + 0.5*v_weight*pow(v - vr, 2);

    return cost;
  }

  virtual void updateGradient(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::VectorXd& gx, Eigen::VectorXd& gu) const override {
    // Determine stage variables
    const auto& steer = states(4);
    const auto& v = states(5);
    const auto& dsteer = ctrls(0);
    const auto& a = ctrls(1);
    const auto& slack_offset = ctrls(2);
    const auto& thetar = params(3);
    const auto& vr = params(5);
    const auto& ref_weight = params(6);
    const auto& steer_weight = params(7);
    const auto& dsteer_weight = params(8);
    const auto& v_weight = params(12);
    const auto& a_weight = params(13);
    const auto& weigth_slack_offset = params(14);

    // Determine global variables
    const auto& l = globals(1);

    // Determine internal variables
    const double internal_0 = l*ref_weight;

    // Evaluation of Vector gx
    gx(1) = -internal_0*sin(thetar);
    gx(2) = internal_0*cos(thetar);
    gx(4) = steer*steer_weight;
    gx(5) = 0.5*v_weight*(2*v - 2*vr);

    // Evaluation of Vector gu
    gu(0) = dsteer*dsteer_weight;
    gu(1) = a*a_weight;
    gu(2) = slack_offset*weigth_slack_offset;

  }

  virtual void updateGradientDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::VectorXd& gx) const override {
    // Determine stage variables
    const auto& steer = states(4);
    const auto& v = states(5);
    const auto& thetar = params(3);
    const auto& vr = params(5);
    const auto& ref_weight = params(6);
    const auto& steer_weight = params(7);
    const auto& v_weight = params(12);

    // Determine global variables
    const auto& l = globals(1);

    // Determine internal variables
    const double internal_0 = l*ref_weight;

    // Evaluation of Vector gx
    gx(1) = -internal_0*sin(thetar);
    gx(2) = internal_0*cos(thetar);
    gx(4) = steer*steer_weight;
    gx(5) = 0.5*v_weight*(2*v - 2*vr);

  }

  virtual void updateGradientDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::VectorXd& gu) const override {
    // Determine stage variables
    const auto& dsteer = ctrls(0);
    const auto& a = ctrls(1);
    const auto& slack_offset = ctrls(2);
    const auto& dsteer_weight = params(8);
    const auto& a_weight = params(13);
    const auto& weigth_slack_offset = params(14);

    // Evaluation of Vector gu
    gu(0) = dsteer*dsteer_weight;
    gu(1) = a*a_weight;
    gu(2) = slack_offset*weigth_slack_offset;

  }

  virtual void updateHessian(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& dxdx, Eigen::MatrixXd& dxdu, Eigen::MatrixXd& dudu) const override {
    // Determine stage variables
    const auto& thetar = params(3);
    const auto& ref_weight = params(6);
    const auto& steer_weight = params(7);
    const auto& dsteer_weight = params(8);
    const auto& v_weight = params(12);
    const auto& a_weight = params(13);
    const auto& weigth_slack_offset = params(14);

    // Determine internal variables
    const double internal_1 = ref_weight;
    const double internal_0 = sin(thetar);
    const double internal_2 = cos(thetar);
    const double internal_3 = -internal_0*internal_1*internal_2;

    // Evaluation of SymmetricMatrix dxdx
    dxdx(1, 1) = pow(internal_0, 2)*internal_1;
    dxdx(1, 2) = internal_3;
    dxdx(2, 2) = internal_1*pow(internal_2, 2);
    dxdx(4, 4) = steer_weight;
    dxdx(5, 5) = v_weight;
    dxdx(2, 1) = dxdx(1, 2);

    // Evaluation of SymmetricMatrix dudu
    dudu(0, 0) = dsteer_weight;
    dudu(1, 1) = a_weight;
    dudu(2, 2) = weigth_slack_offset;

  }

  virtual void updateHessianDxDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& dxdx) const override {
    // Determine stage variables
    const auto& thetar = params(3);
    const auto& ref_weight = params(6);
    const auto& steer_weight = params(7);
    const auto& v_weight = params(12);

    // Determine internal variables
    const double internal_1 = ref_weight;
    const double internal_0 = sin(thetar);
    const double internal_2 = cos(thetar);
    const double internal_3 = -internal_0*internal_1*internal_2;

    // Evaluation of SymmetricMatrix dxdx
    dxdx(1, 1) = pow(internal_0, 2)*internal_1;
    dxdx(1, 2) = internal_3;
    dxdx(2, 2) = internal_1*pow(internal_2, 2);
    dxdx(4, 4) = steer_weight;
    dxdx(5, 5) = v_weight;
    dxdx(2, 1) = dxdx(1, 2);

  }

  virtual void updateHessianDxDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& dxdu) const override {

  }

  virtual void updateHessianDuDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& dudu) const override {
    // Determine stage variables
    const auto& dsteer_weight = params(8);
    const auto& a_weight = params(13);
    const auto& weigth_slack_offset = params(14);

    // Evaluation of SymmetricMatrix dudu
    dudu(0, 0) = dsteer_weight;
    dudu(1, 1) = a_weight;
    dudu(2, 2) = weigth_slack_offset;

  }

  virtual void updateGradient(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t, Eigen::VectorXd& gx, Eigen::VectorXd& gu) const override {
    // Determine stage variables
    const auto& steer = states(4);
    const auto& v = states(5);
    const auto& dsteer = ctrls(0);
    const auto& a = ctrls(1);
    const auto& slack_offset = ctrls(2);
    const auto& thetar = params(3);
    const auto& vr = params(5);
    const auto& ref_weight = params(6);
    const auto& steer_weight = params(7);
    const auto& dsteer_weight = params(8);
    const auto& v_weight = params(12);
    const auto& a_weight = params(13);
    const auto& weigth_slack_offset = params(14);

    // Determine global variables
    const auto& l = globals(1);

    // Determine internal variables
    const double internal_0 = delta_t;
    const double internal_1 = internal_0*l*ref_weight;

    // Evaluation of Vector gx
    gx(1) = -internal_1*sin(thetar);
    gx(2) = internal_1*cos(thetar);
    gx(4) = internal_0*steer*steer_weight;
    gx(5) = 0.5*delta_t*v_weight*(2*v - 2*vr);

    // Evaluation of Vector gu
    gu(0) = dsteer*dsteer_weight*internal_0;
    gu(1) = a*a_weight*internal_0;
    gu(2) = internal_0*slack_offset*weigth_slack_offset;

  }

  virtual void updateGradientDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t, Eigen::VectorXd& gx) const override {
    // Determine stage variables
    const auto& steer = states(4);
    const auto& v = states(5);
    const auto& thetar = params(3);
    const auto& vr = params(5);
    const auto& ref_weight = params(6);
    const auto& steer_weight = params(7);
    const auto& v_weight = params(12);

    // Determine global variables
    const auto& l = globals(1);

    // Determine internal variables
    const double internal_0 = delta_t;
    const double internal_1 = internal_0*l*ref_weight;

    // Evaluation of Vector gx
    gx(1) = -internal_1*sin(thetar);
    gx(2) = internal_1*cos(thetar);
    gx(4) = internal_0*steer*steer_weight;
    gx(5) = 0.5*delta_t*v_weight*(2*v - 2*vr);

  }

  virtual void updateGradientDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t, Eigen::VectorXd& gu) const override {
    // Determine stage variables
    const auto& dsteer = ctrls(0);
    const auto& a = ctrls(1);
    const auto& slack_offset = ctrls(2);
    const auto& dsteer_weight = params(8);
    const auto& a_weight = params(13);
    const auto& weigth_slack_offset = params(14);

    // Determine internal variables
    const double internal_0 = delta_t;

    // Evaluation of Vector gu
    gu(0) = dsteer*dsteer_weight*internal_0;
    gu(1) = a*a_weight*internal_0;
    gu(2) = internal_0*slack_offset*weigth_slack_offset;

  }

  virtual void updateHessian(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t, Eigen::MatrixXd& dxdx, Eigen::MatrixXd& dxdu, Eigen::MatrixXd& dudu) const override {
    // Determine stage variables
    const auto& thetar = params(3);
    const auto& ref_weight = params(6);
    const auto& steer_weight = params(7);
    const auto& dsteer_weight = params(8);
    const auto& v_weight = params(12);
    const auto& a_weight = params(13);
    const auto& weigth_slack_offset = params(14);

    // Determine internal variables
    const double internal_1 = delta_t;
    const double internal_0 = sin(thetar);
    const double internal_3 = cos(thetar);
    const double internal_2 = internal_1*ref_weight;
    const double internal_4 = -internal_0*internal_2*internal_3;

    // Evaluation of SymmetricMatrix dxdx
    dxdx(1, 1) = pow(internal_0, 2)*internal_2;
    dxdx(1, 2) = internal_4;
    dxdx(2, 2) = internal_2*pow(internal_3, 2);
    dxdx(4, 4) = internal_1*steer_weight;
    dxdx(5, 5) = internal_1*v_weight;
    dxdx(2, 1) = dxdx(1, 2);

    // Evaluation of SymmetricMatrix dudu
    dudu(0, 0) = dsteer_weight*internal_1;
    dudu(1, 1) = a_weight*internal_1;
    dudu(2, 2) = internal_1*weigth_slack_offset;

  }

  virtual void updateHessianDxDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t, Eigen::MatrixXd& dxdx) const override {
    // Determine stage variables
    const auto& thetar = params(3);
    const auto& ref_weight = params(6);
    const auto& steer_weight = params(7);
    const auto& v_weight = params(12);

    // Determine internal variables
    const double internal_1 = delta_t;
    const double internal_0 = sin(thetar);
    const double internal_3 = cos(thetar);
    const double internal_2 = internal_1*ref_weight;
    const double internal_4 = -internal_0*internal_2*internal_3;

    // Evaluation of SymmetricMatrix dxdx
    dxdx(1, 1) = pow(internal_0, 2)*internal_2;
    dxdx(1, 2) = internal_4;
    dxdx(2, 2) = internal_2*pow(internal_3, 2);
    dxdx(4, 4) = internal_1*steer_weight;
    dxdx(5, 5) = internal_1*v_weight;
    dxdx(2, 1) = dxdx(1, 2);

  }

  virtual void updateHessianDxDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t, Eigen::MatrixXd& dxdu) const override {

  }

  virtual void updateHessianDuDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t, Eigen::MatrixXd& dudu) const override {
    // Determine stage variables
    const auto& dsteer_weight = params(8);
    const auto& a_weight = params(13);
    const auto& weigth_slack_offset = params(14);

    // Determine internal variables
    const double internal_0 = delta_t;

    // Evaluation of SymmetricMatrix dudu
    dudu(0, 0) = dsteer_weight*internal_0;
    dudu(1, 1) = a_weight*internal_0;
    dudu(2, 2) = internal_0*weigth_slack_offset;

  }

};

}  //  namespace gpal::pnc::planning