#pragma once

#include "ocp/cost_function.h"

namespace gpal::pnc::planning {

template <StageType type>
class SpeedRiskModelCost : public CostFunction {
 public:
  SpeedRiskModelCost() : CostFunction(3, 8) {}

  virtual ~SpeedRiskModelCost() = default;

  virtual double evaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls) const override {
    // Determine stage variables
    const auto& s = states(0);
    const auto& v = states(1);
    const auto& a = states(2);
    const auto& j = ctrls(0);
    const auto& slack_s_upper = ctrls(1);
    const auto& slack_s_lower = ctrls(2);
    const auto& slack_v_upper = ctrls(3);
    const auto& slack_v_lower = ctrls(4);
    const auto& slack_a_upper = ctrls(5);
    const auto& slack_a_lower = ctrls(6);
    const auto& slack_d_v = ctrls(7);
    const auto& SRef = params(0);
    const auto& VRef = params(1);
    const auto& K = params(2);
    const auto& SWeight = params(3);
    const auto& VWeight = params(4);
    const auto& AWeight = params(5);
    const auto& JWeight = params(6);
    const auto& SlackSUpperWeight = params(7);
    const auto& SlackSLowerWeight = params(8);
    const auto& SlackVUpperWeight = params(9);
    const auto& SlackVLowerWeight = params(10);
    const auto& SlackAUpperWeight = params(11);
    const auto& SlackALowerWeight = params(12);
    const auto& SlackDVWeight = params(13);

    // Determine internal variables
    const double a_p2 = a * a;
    const double j_p2 = j * j;
    const double slack_s_upper_p2 = slack_s_upper * slack_s_upper;
    const double slack_s_lower_p2 = slack_s_lower * slack_s_lower;
    const double slack_v_upper_p2 = slack_v_upper * slack_v_upper;
    const double slack_v_lower_p2 = slack_v_lower * slack_v_lower;
    const double slack_a_upper_p2 = slack_a_upper * slack_a_upper;
    const double slack_a_lower_p2 = slack_a_lower * slack_a_lower;
    const double slack_d_v_p2 = slack_d_v * slack_d_v;
    const double internal_1 = -VRef + v;
    const double internal_0 = K * v - SRef + s;
    const double internal_1_p2 = internal_1 * internal_1;
    const double internal_0_p2 = internal_0 * internal_0;

    // Evaluation of Scalar cost
    double cost = AWeight * a_p2 + JWeight * j_p2 + SWeight * internal_0_p2 + SlackALowerWeight * slack_a_lower_p2 +
                  SlackAUpperWeight * slack_a_upper_p2 + SlackDVWeight * slack_d_v_p2 +
                  SlackSLowerWeight * slack_s_lower_p2 + SlackSUpperWeight * slack_s_upper_p2 +
                  SlackVLowerWeight * slack_v_lower_p2 + SlackVUpperWeight * slack_v_upper_p2 + VWeight * internal_1_p2;

    return cost;
  }

  virtual void updateGradient(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::VectorXd& gx,
                              Eigen::VectorXd& gu) const override {
    // Determine stage variables
    const auto& s = states(0);
    const auto& v = states(1);
    const auto& a = states(2);
    const auto& j = ctrls(0);
    const auto& slack_s_upper = ctrls(1);
    const auto& slack_s_lower = ctrls(2);
    const auto& slack_v_upper = ctrls(3);
    const auto& slack_v_lower = ctrls(4);
    const auto& slack_a_upper = ctrls(5);
    const auto& slack_a_lower = ctrls(6);
    const auto& slack_d_v = ctrls(7);
    const auto& SRef = params(0);
    const auto& VRef = params(1);
    const auto& K = params(2);
    const auto& SWeight = params(3);
    const auto& VWeight = params(4);
    const auto& AWeight = params(5);
    const auto& JWeight = params(6);
    const auto& SlackSUpperWeight = params(7);
    const auto& SlackSLowerWeight = params(8);
    const auto& SlackVUpperWeight = params(9);
    const auto& SlackVLowerWeight = params(10);
    const auto& SlackAUpperWeight = params(11);
    const auto& SlackALowerWeight = params(12);
    const auto& SlackDVWeight = params(13);

    // Evaluation of Vector gx
    gx(0) = SWeight * (2 * K * v - 2 * SRef + 2 * s);
    gx(1) = 2 * K * SWeight * (K * v - SRef + s) + VWeight * (-2 * VRef + 2 * v);
    gx(2) = 2 * AWeight * a;

    // Evaluation of Vector gu
    gu(0) = 2 * JWeight * j;
    gu(1) = 2 * SlackSUpperWeight * slack_s_upper;
    gu(2) = 2 * SlackSLowerWeight * slack_s_lower;
    gu(3) = 2 * SlackVUpperWeight * slack_v_upper;
    gu(4) = 2 * SlackVLowerWeight * slack_v_lower;
    gu(5) = 2 * SlackAUpperWeight * slack_a_upper;
    gu(6) = 2 * SlackALowerWeight * slack_a_lower;
    gu(7) = 2 * SlackDVWeight * slack_d_v;
  }

  virtual void updateGradientDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                Eigen::VectorXd& gx) const override {
    // Determine stage variables
    const auto& s = states(0);
    const auto& v = states(1);
    const auto& a = states(2);
    const auto& SRef = params(0);
    const auto& VRef = params(1);
    const auto& K = params(2);
    const auto& SWeight = params(3);
    const auto& VWeight = params(4);
    const auto& AWeight = params(5);

    // Evaluation of Vector gx
    gx(0) = SWeight * (2 * K * v - 2 * SRef + 2 * s);
    gx(1) = 2 * K * SWeight * (K * v - SRef + s) + VWeight * (-2 * VRef + 2 * v);
    gx(2) = 2 * AWeight * a;
  }

  virtual void updateGradientDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                Eigen::VectorXd& gu) const override {
    // Determine stage variables
    const auto& j = ctrls(0);
    const auto& slack_s_upper = ctrls(1);
    const auto& slack_s_lower = ctrls(2);
    const auto& slack_v_upper = ctrls(3);
    const auto& slack_v_lower = ctrls(4);
    const auto& slack_a_upper = ctrls(5);
    const auto& slack_a_lower = ctrls(6);
    const auto& slack_d_v = ctrls(7);
    const auto& JWeight = params(6);
    const auto& SlackSUpperWeight = params(7);
    const auto& SlackSLowerWeight = params(8);
    const auto& SlackVUpperWeight = params(9);
    const auto& SlackVLowerWeight = params(10);
    const auto& SlackAUpperWeight = params(11);
    const auto& SlackALowerWeight = params(12);
    const auto& SlackDVWeight = params(13);

    // Evaluation of Vector gu
    gu(0) = 2 * JWeight * j;
    gu(1) = 2 * SlackSUpperWeight * slack_s_upper;
    gu(2) = 2 * SlackSLowerWeight * slack_s_lower;
    gu(3) = 2 * SlackVUpperWeight * slack_v_upper;
    gu(4) = 2 * SlackVLowerWeight * slack_v_lower;
    gu(5) = 2 * SlackAUpperWeight * slack_a_upper;
    gu(6) = 2 * SlackALowerWeight * slack_a_lower;
    gu(7) = 2 * SlackDVWeight * slack_d_v;
  }

  virtual void updateHessian(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& dxdx,
                             Eigen::MatrixXd& dxdu, Eigen::MatrixXd& dudu) const override {
    // Determine stage variables
    const auto& K = params(2);
    const auto& SWeight = params(3);
    const auto& VWeight = params(4);
    const auto& AWeight = params(5);
    const auto& JWeight = params(6);
    const auto& SlackSUpperWeight = params(7);
    const auto& SlackSLowerWeight = params(8);
    const auto& SlackVUpperWeight = params(9);
    const auto& SlackVLowerWeight = params(10);
    const auto& SlackAUpperWeight = params(11);
    const auto& SlackALowerWeight = params(12);
    const auto& SlackDVWeight = params(13);

    // Determine internal variables
    const double K_p2 = K * K;

    // Evaluation of SymmetricMatrix dxdx
    dxdx(0, 0) = 2 * SWeight;
    dxdx(0, 1) = K * dxdx(0, 0);
    dxdx(1, 1) = K_p2 * dxdx(0, 0) + 2 * VWeight;
    dxdx(2, 2) = 2 * AWeight;
    dxdx(1, 0) = dxdx(0, 1);

    // Evaluation of SymmetricMatrix dudu
    dudu(0, 0) = 2 * JWeight;
    dudu(1, 1) = 2 * SlackSUpperWeight;
    dudu(2, 2) = 2 * SlackSLowerWeight;
    dudu(3, 3) = 2 * SlackVUpperWeight;
    dudu(4, 4) = 2 * SlackVLowerWeight;
    dudu(5, 5) = 2 * SlackAUpperWeight;
    dudu(6, 6) = 2 * SlackALowerWeight;
    dudu(7, 7) = 2 * SlackDVWeight;
  }

  virtual void updateHessianDxDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                 Eigen::MatrixXd& dxdx) const override {
    // Determine stage variables
    const auto& K = params(2);
    const auto& SWeight = params(3);
    const auto& VWeight = params(4);
    const auto& AWeight = params(5);

    // Determine internal variables
    const double K_p2 = K * K;

    // Evaluation of SymmetricMatrix dxdx
    dxdx(0, 0) = 2 * SWeight;
    dxdx(0, 1) = K * dxdx(0, 0);
    dxdx(1, 1) = K_p2 * dxdx(0, 0) + 2 * VWeight;
    dxdx(2, 2) = 2 * AWeight;
    dxdx(1, 0) = dxdx(0, 1);
  }

  virtual void updateHessianDxDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                 Eigen::MatrixXd& dxdu) const override {}

  virtual void updateHessianDuDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                 Eigen::MatrixXd& dudu) const override {
    // Determine stage variables
    const auto& JWeight = params(6);
    const auto& SlackSUpperWeight = params(7);
    const auto& SlackSLowerWeight = params(8);
    const auto& SlackVUpperWeight = params(9);
    const auto& SlackVLowerWeight = params(10);
    const auto& SlackAUpperWeight = params(11);
    const auto& SlackALowerWeight = params(12);
    const auto& SlackDVWeight = params(13);

    // Evaluation of SymmetricMatrix dudu
    dudu(0, 0) = 2 * JWeight;
    dudu(1, 1) = 2 * SlackSUpperWeight;
    dudu(2, 2) = 2 * SlackSLowerWeight;
    dudu(3, 3) = 2 * SlackVUpperWeight;
    dudu(4, 4) = 2 * SlackVLowerWeight;
    dudu(5, 5) = 2 * SlackAUpperWeight;
    dudu(6, 6) = 2 * SlackALowerWeight;
    dudu(7, 7) = 2 * SlackDVWeight;
  }

  virtual void updateGradient(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t,
                              Eigen::VectorXd& gx, Eigen::VectorXd& gu) const override {
    // Determine stage variables
    const auto& s = states(0);
    const auto& v = states(1);
    const auto& a = states(2);
    const auto& j = ctrls(0);
    const auto& slack_s_upper = ctrls(1);
    const auto& slack_s_lower = ctrls(2);
    const auto& slack_v_upper = ctrls(3);
    const auto& slack_v_lower = ctrls(4);
    const auto& slack_a_upper = ctrls(5);
    const auto& slack_a_lower = ctrls(6);
    const auto& slack_d_v = ctrls(7);
    const auto& SRef = params(0);
    const auto& VRef = params(1);
    const auto& K = params(2);
    const auto& SWeight = params(3);
    const auto& VWeight = params(4);
    const auto& AWeight = params(5);
    const auto& JWeight = params(6);
    const auto& SlackSUpperWeight = params(7);
    const auto& SlackSLowerWeight = params(8);
    const auto& SlackVUpperWeight = params(9);
    const auto& SlackVLowerWeight = params(10);
    const auto& SlackAUpperWeight = params(11);
    const auto& SlackALowerWeight = params(12);
    const auto& SlackDVWeight = params(13);

    // Evaluation of Vector gx
    gx(0) = SWeight * delta_t * (2 * K * v - 2 * SRef + 2 * s);
    gx(1) = delta_t * (2 * K * SWeight * (K * v - SRef + s) + VWeight * (-2 * VRef + 2 * v));
    gx(2) = 2 * AWeight * a * delta_t;

    // Evaluation of Vector gu
    gu(0) = 2 * JWeight * delta_t * j;
    gu(1) = 2 * SlackSUpperWeight * delta_t * slack_s_upper;
    gu(2) = 2 * SlackSLowerWeight * delta_t * slack_s_lower;
    gu(3) = 2 * SlackVUpperWeight * delta_t * slack_v_upper;
    gu(4) = 2 * SlackVLowerWeight * delta_t * slack_v_lower;
    gu(5) = 2 * SlackAUpperWeight * delta_t * slack_a_upper;
    gu(6) = 2 * SlackALowerWeight * delta_t * slack_a_lower;
    gu(7) = 2 * SlackDVWeight * delta_t * slack_d_v;
  }

  virtual void updateGradientDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t,
                                Eigen::VectorXd& gx) const override {
    // Determine stage variables
    const auto& s = states(0);
    const auto& v = states(1);
    const auto& a = states(2);
    const auto& SRef = params(0);
    const auto& VRef = params(1);
    const auto& K = params(2);
    const auto& SWeight = params(3);
    const auto& VWeight = params(4);
    const auto& AWeight = params(5);

    // Evaluation of Vector gx
    gx(0) = SWeight * delta_t * (2 * K * v - 2 * SRef + 2 * s);
    gx(1) = delta_t * (2 * K * SWeight * (K * v - SRef + s) + VWeight * (-2 * VRef + 2 * v));
    gx(2) = 2 * AWeight * a * delta_t;
  }

  virtual void updateGradientDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t,
                                Eigen::VectorXd& gu) const override {
    // Determine stage variables
    const auto& j = ctrls(0);
    const auto& slack_s_upper = ctrls(1);
    const auto& slack_s_lower = ctrls(2);
    const auto& slack_v_upper = ctrls(3);
    const auto& slack_v_lower = ctrls(4);
    const auto& slack_a_upper = ctrls(5);
    const auto& slack_a_lower = ctrls(6);
    const auto& slack_d_v = ctrls(7);
    const auto& JWeight = params(6);
    const auto& SlackSUpperWeight = params(7);
    const auto& SlackSLowerWeight = params(8);
    const auto& SlackVUpperWeight = params(9);
    const auto& SlackVLowerWeight = params(10);
    const auto& SlackAUpperWeight = params(11);
    const auto& SlackALowerWeight = params(12);
    const auto& SlackDVWeight = params(13);

    // Evaluation of Vector gu
    gu(0) = 2 * JWeight * delta_t * j;
    gu(1) = 2 * SlackSUpperWeight * delta_t * slack_s_upper;
    gu(2) = 2 * SlackSLowerWeight * delta_t * slack_s_lower;
    gu(3) = 2 * SlackVUpperWeight * delta_t * slack_v_upper;
    gu(4) = 2 * SlackVLowerWeight * delta_t * slack_v_lower;
    gu(5) = 2 * SlackAUpperWeight * delta_t * slack_a_upper;
    gu(6) = 2 * SlackALowerWeight * delta_t * slack_a_lower;
    gu(7) = 2 * SlackDVWeight * delta_t * slack_d_v;
  }

  virtual void updateHessian(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t,
                             Eigen::MatrixXd& dxdx, Eigen::MatrixXd& dxdu, Eigen::MatrixXd& dudu) const override {
    // Determine stage variables
    const auto& K = params(2);
    const auto& SWeight = params(3);
    const auto& VWeight = params(4);
    const auto& AWeight = params(5);
    const auto& JWeight = params(6);
    const auto& SlackSUpperWeight = params(7);
    const auto& SlackSLowerWeight = params(8);
    const auto& SlackVUpperWeight = params(9);
    const auto& SlackVLowerWeight = params(10);
    const auto& SlackAUpperWeight = params(11);
    const auto& SlackALowerWeight = params(12);
    const auto& SlackDVWeight = params(13);

    // Determine internal variables
    const double K_p2 = K * K;

    // Evaluation of SymmetricMatrix dxdx
    dxdx(0, 0) = 2 * SWeight * delta_t;
    dxdx(0, 1) = K * dxdx(0, 0);
    dxdx(1, 1) = delta_t * (2 * K_p2 * SWeight + 2 * VWeight);
    dxdx(2, 2) = 2 * AWeight * delta_t;
    dxdx(1, 0) = dxdx(0, 1);

    // Evaluation of SymmetricMatrix dudu
    dudu(0, 0) = 2 * JWeight * delta_t;
    dudu(1, 1) = 2 * SlackSUpperWeight * delta_t;
    dudu(2, 2) = 2 * SlackSLowerWeight * delta_t;
    dudu(3, 3) = 2 * SlackVUpperWeight * delta_t;
    dudu(4, 4) = 2 * SlackVLowerWeight * delta_t;
    dudu(5, 5) = 2 * SlackAUpperWeight * delta_t;
    dudu(6, 6) = 2 * SlackALowerWeight * delta_t;
    dudu(7, 7) = 2 * SlackDVWeight * delta_t;
  }

  virtual void updateHessianDxDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t,
                                 Eigen::MatrixXd& dxdx) const override {
    // Determine stage variables
    const auto& K = params(2);
    const auto& SWeight = params(3);
    const auto& VWeight = params(4);
    const auto& AWeight = params(5);

    // Determine internal variables
    const double K_p2 = K * K;

    // Evaluation of SymmetricMatrix dxdx
    dxdx(0, 0) = 2 * SWeight * delta_t;
    dxdx(0, 1) = K * dxdx(0, 0);
    dxdx(1, 1) = delta_t * (2 * K_p2 * SWeight + 2 * VWeight);
    dxdx(2, 2) = 2 * AWeight * delta_t;
    dxdx(1, 0) = dxdx(0, 1);
  }

  virtual void updateHessianDxDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t,
                                 Eigen::MatrixXd& dxdu) const override {}

  virtual void updateHessianDuDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t,
                                 Eigen::MatrixXd& dudu) const override {
    // Determine stage variables
    const auto& JWeight = params(6);
    const auto& SlackSUpperWeight = params(7);
    const auto& SlackSLowerWeight = params(8);
    const auto& SlackVUpperWeight = params(9);
    const auto& SlackVLowerWeight = params(10);
    const auto& SlackAUpperWeight = params(11);
    const auto& SlackALowerWeight = params(12);
    const auto& SlackDVWeight = params(13);

    // Evaluation of SymmetricMatrix dudu
    dudu(0, 0) = 2 * JWeight * delta_t;
    dudu(1, 1) = 2 * SlackSUpperWeight * delta_t;
    dudu(2, 2) = 2 * SlackSLowerWeight * delta_t;
    dudu(3, 3) = 2 * SlackVUpperWeight * delta_t;
    dudu(4, 4) = 2 * SlackVLowerWeight * delta_t;
    dudu(5, 5) = 2 * SlackAUpperWeight * delta_t;
    dudu(6, 6) = 2 * SlackALowerWeight * delta_t;
    dudu(7, 7) = 2 * SlackDVWeight * delta_t;
  }
};

template <>
class SpeedRiskModelCost<StageType::TERMINAL> : public CostFunction {
 public:
  SpeedRiskModelCost() : CostFunction(3, 8) {}
  virtual ~SpeedRiskModelCost() = default;

  virtual double evaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls) const override {
    // Determine stage variables
    const auto& s = states(0);
    const auto& v = states(1);
    const auto& a = states(2);
    const auto& SRef = params(0);
    const auto& VRef = params(1);
    const auto& K = params(2);
    const auto& SWeight = params(3);
    const auto& VWeight = params(4);
    const auto& AWeight = params(5);

    // Determine internal variables
    const double a_p2 = a * a;
    const double internal_1 = -VRef + v;
    const double internal_0 = K * v - SRef + s;
    const double internal_1_p2 = internal_1 * internal_1;
    const double internal_0_p2 = internal_0 * internal_0;

    // Evaluation of Scalar cost
    double cost = AWeight * a_p2 + SWeight * internal_0_p2 + VWeight * internal_1_p2;

    return cost;
  }

  virtual void updateGradientDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                Eigen::VectorXd& gx) const override {
    // Determine stage variables
    const auto& s = states(0);
    const auto& v = states(1);
    const auto& a = states(2);
    const auto& SRef = params(0);
    const auto& VRef = params(1);
    const auto& K = params(2);
    const auto& SWeight = params(3);
    const auto& VWeight = params(4);
    const auto& AWeight = params(5);

    // Evaluation of Vector gx
    gx(0) = SWeight * (2 * K * v - 2 * SRef + 2 * s);
    gx(1) = 2 * K * SWeight * (K * v - SRef + s) + VWeight * (-2 * VRef + 2 * v);
    gx(2) = 2 * AWeight * a;
  }

  virtual void updateHessianDxDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                 Eigen::MatrixXd& hxx) const override {
    // Determine stage variables
    const auto& K = params(2);
    const auto& SWeight = params(3);
    const auto& VWeight = params(4);
    const auto& AWeight = params(5);

    // Determine internal variables
    const double K_p2 = K * K;

    // Evaluation of SymmetricMatrix hxx
    hxx(0, 0) = 2 * SWeight;
    hxx(0, 1) = K * hxx(0, 0);
    hxx(1, 1) = K_p2 * hxx(0, 0) + 2 * VWeight;
    hxx(2, 2) = 2 * AWeight;
    hxx(1, 0) = hxx(0, 1);
  }
};

}  //  namespace gpal::pnc::planning