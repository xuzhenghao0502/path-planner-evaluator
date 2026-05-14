#pragma once

#include "ocp/cost_function.h"

namespace gpal::pnc::planning {

template <StageType type>
class ReferenceLineModelCost : public CostFunction {
 public:
  ReferenceLineModelCost() : CostFunction(4, 2) {}

  virtual ~ReferenceLineModelCost() = default;

  virtual double evaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls) const override {
    // Determine stage variables
    const auto& kappa = states(3);
    const auto& dkappa = ctrls(0);
    const auto& slack_offset = ctrls(1);
    const auto& l_weight = params(3);
    const auto& kappa_weight = params(4);
    const auto& dkappa_weight = params(5);
    const auto& slack_offset_weight = params(7);

    // Determine global variables
    const auto& l = globals(3);

    // Determine internal variables
    const double kappa_p2 = kappa * kappa;
    const double dkappa_p2 = dkappa * dkappa;
    const double slack_offset_p2 = slack_offset * slack_offset;
    const double l_p2 = l * l;

    // Evaluation of Scalar cost
    double cost = 0.5 * dkappa_p2 * dkappa_weight + 0.5 * kappa_p2 * kappa_weight + 0.5 * l_p2 * l_weight +
                  0.5 * slack_offset_p2 * slack_offset_weight;

    return cost;
  }

  virtual void updateGradient(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::VectorXd& gx,
                              Eigen::VectorXd& gu) const override {
    // Determine stage variables
    const auto& kappa = states(3);
    const auto& dkappa = ctrls(0);
    const auto& slack_offset = ctrls(1);
    const auto& l_weight = params(3);
    const auto& kappa_weight = params(4);
    const auto& dkappa_weight = params(5);
    const auto& slack_offset_weight = params(7);

    // Determine global variables
    const auto& cos_thetar = globals(0);
    const auto& sin_thetar = globals(1);
    const auto& l = globals(3);

    // Evaluation of Vector gx
    gx(0) = -1.0 * l * l_weight * sin_thetar;
    gx(1) = cos_thetar * l * l_weight;
    gx(3) = kappa * kappa_weight;

    // Evaluation of Vector gu
    gu(0) = dkappa * dkappa_weight;
    gu(1) = slack_offset * slack_offset_weight;
  }

  virtual void updateGradientDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                Eigen::VectorXd& gx) const override {
    // Determine stage variables
    const auto& kappa = states(3);
    const auto& l_weight = params(3);
    const auto& kappa_weight = params(4);

    // Determine global variables
    const auto& cos_thetar = globals(0);
    const auto& sin_thetar = globals(1);
    const auto& l = globals(3);

    // Evaluation of Vector gx
    gx(0) = -1.0 * l * l_weight * sin_thetar;
    gx(1) = cos_thetar * l * l_weight;
    gx(3) = kappa * kappa_weight;
  }

  virtual void updateGradientDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                Eigen::VectorXd& gu) const override {
    // Determine stage variables
    const auto& dkappa = ctrls(0);
    const auto& slack_offset = ctrls(1);
    const auto& dkappa_weight = params(5);
    const auto& slack_offset_weight = params(7);

    // Evaluation of Vector gu
    gu(0) = dkappa * dkappa_weight;
    gu(1) = slack_offset * slack_offset_weight;
  }

  virtual void updateHessian(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& dxdx,
                             Eigen::MatrixXd& dxdu, Eigen::MatrixXd& dudu) const override {
    // Determine stage variables
    const auto& l_weight = params(3);
    const auto& kappa_weight = params(4);
    const auto& dkappa_weight = params(5);
    const auto& slack_offset_weight = params(7);

    // Determine global variables
    const auto& cos_thetar = globals(0);
    const auto& sin_thetar = globals(1);

    // Determine internal variables
    const double cos_thetar_p2 = cos_thetar * cos_thetar;
    const double sin_thetar_p2 = sin_thetar * sin_thetar;

    // Evaluation of SymmetricMatrix dxdx
    dxdx(0, 0) = l_weight * sin_thetar_p2;
    dxdx(0, 1) = -1.0 * cos_thetar * l_weight * sin_thetar;
    dxdx(1, 1) = cos_thetar_p2 * l_weight;
    dxdx(3, 3) = kappa_weight;
    dxdx(1, 0) = dxdx(0, 1);

    // Evaluation of SymmetricMatrix dudu
    dudu(0, 0) = dkappa_weight;
    dudu(1, 1) = slack_offset_weight;
  }

  virtual void updateHessianDxDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                 Eigen::MatrixXd& dxdx) const override {
    // Determine stage variables
    const auto& l_weight = params(3);
    const auto& kappa_weight = params(4);

    // Determine global variables
    const auto& cos_thetar = globals(0);
    const auto& sin_thetar = globals(1);

    // Determine internal variables
    const double cos_thetar_p2 = cos_thetar * cos_thetar;
    const double sin_thetar_p2 = sin_thetar * sin_thetar;

    // Evaluation of SymmetricMatrix dxdx
    dxdx(0, 0) = l_weight * sin_thetar_p2;
    dxdx(0, 1) = -1.0 * cos_thetar * l_weight * sin_thetar;
    dxdx(1, 1) = cos_thetar_p2 * l_weight;
    dxdx(3, 3) = kappa_weight;
    dxdx(1, 0) = dxdx(0, 1);
  }

  virtual void updateHessianDxDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                 Eigen::MatrixXd& dxdu) const override {}

  virtual void updateHessianDuDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                 Eigen::MatrixXd& dudu) const override {
    // Determine stage variables
    const auto& dkappa_weight = params(5);
    const auto& slack_offset_weight = params(7);

    // Evaluation of SymmetricMatrix dudu
    dudu(0, 0) = dkappa_weight;
    dudu(1, 1) = slack_offset_weight;
  }

  virtual void updateGradient(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t,
                              Eigen::VectorXd& gx, Eigen::VectorXd& gu) const override {
    // Determine stage variables
    const auto& kappa = states(3);
    const auto& dkappa = ctrls(0);
    const auto& slack_offset = ctrls(1);
    const auto& l_weight = params(3);
    const auto& kappa_weight = params(4);
    const auto& dkappa_weight = params(5);
    const auto& slack_offset_weight = params(7);

    // Determine global variables
    const auto& cos_thetar = globals(0);
    const auto& sin_thetar = globals(1);
    const auto& l = globals(3);

    // Evaluation of Vector gx
    gx(0) = -1.0 * delta_t * l * l_weight * sin_thetar;
    gx(1) = cos_thetar * delta_t * l * l_weight;
    gx(3) = delta_t * kappa * kappa_weight;

    // Evaluation of Vector gu
    gu(0) = delta_t * dkappa * dkappa_weight;
    gu(1) = delta_t * slack_offset * slack_offset_weight;
  }

  virtual void updateGradientDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t,
                                Eigen::VectorXd& gx) const override {
    // Determine stage variables
    const auto& kappa = states(3);
    const auto& l_weight = params(3);
    const auto& kappa_weight = params(4);

    // Determine global variables
    const auto& cos_thetar = globals(0);
    const auto& sin_thetar = globals(1);
    const auto& l = globals(3);

    // Evaluation of Vector gx
    gx(0) = -1.0 * delta_t * l * l_weight * sin_thetar;
    gx(1) = cos_thetar * delta_t * l * l_weight;
    gx(3) = delta_t * kappa * kappa_weight;
  }

  virtual void updateGradientDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t,
                                Eigen::VectorXd& gu) const override {
    // Determine stage variables
    const auto& dkappa = ctrls(0);
    const auto& slack_offset = ctrls(1);
    const auto& dkappa_weight = params(5);
    const auto& slack_offset_weight = params(7);

    // Evaluation of Vector gu
    gu(0) = delta_t * dkappa * dkappa_weight;
    gu(1) = delta_t * slack_offset * slack_offset_weight;
  }

  virtual void updateHessian(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t,
                             Eigen::MatrixXd& dxdx, Eigen::MatrixXd& dxdu, Eigen::MatrixXd& dudu) const override {
    // Determine stage variables
    const auto& l_weight = params(3);
    const auto& kappa_weight = params(4);
    const auto& dkappa_weight = params(5);
    const auto& slack_offset_weight = params(7);

    // Determine global variables
    const auto& cos_thetar = globals(0);
    const auto& sin_thetar = globals(1);

    // Determine internal variables
    const double cos_thetar_p2 = cos_thetar * cos_thetar;
    const double sin_thetar_p2 = sin_thetar * sin_thetar;

    // Evaluation of SymmetricMatrix dxdx
    dxdx(0, 0) = delta_t * l_weight * sin_thetar_p2;
    dxdx(0, 1) = -1.0 * cos_thetar * delta_t * l_weight * sin_thetar;
    dxdx(1, 1) = cos_thetar_p2 * delta_t * l_weight;
    dxdx(3, 3) = delta_t * kappa_weight;
    dxdx(1, 0) = dxdx(0, 1);

    // Evaluation of SymmetricMatrix dudu
    dudu(0, 0) = delta_t * dkappa_weight;
    dudu(1, 1) = delta_t * slack_offset_weight;
  }

  virtual void updateHessianDxDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t,
                                 Eigen::MatrixXd& dxdx) const override {
    // Determine stage variables
    const auto& l_weight = params(3);
    const auto& kappa_weight = params(4);

    // Determine global variables
    const auto& cos_thetar = globals(0);
    const auto& sin_thetar = globals(1);

    // Determine internal variables
    const double cos_thetar_p2 = cos_thetar * cos_thetar;
    const double sin_thetar_p2 = sin_thetar * sin_thetar;

    // Evaluation of SymmetricMatrix dxdx
    dxdx(0, 0) = delta_t * l_weight * sin_thetar_p2;
    dxdx(0, 1) = -1.0 * cos_thetar * delta_t * l_weight * sin_thetar;
    dxdx(1, 1) = cos_thetar_p2 * delta_t * l_weight;
    dxdx(3, 3) = delta_t * kappa_weight;
    dxdx(1, 0) = dxdx(0, 1);
  }

  virtual void updateHessianDxDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t,
                                 Eigen::MatrixXd& dxdu) const override {}

  virtual void updateHessianDuDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t,
                                 Eigen::MatrixXd& dudu) const override {
    // Determine stage variables
    const auto& dkappa_weight = params(5);
    const auto& slack_offset_weight = params(7);

    // Evaluation of SymmetricMatrix dudu
    dudu(0, 0) = delta_t * dkappa_weight;
    dudu(1, 1) = delta_t * slack_offset_weight;
  }
};

template <>
class ReferenceLineModelCost<StageType::TERMINAL> : public CostFunction {
 public:
  ReferenceLineModelCost() : CostFunction(4, 2) {}
  virtual ~ReferenceLineModelCost() = default;

  virtual double evaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls) const override {
    // Determine stage variables
    const auto& kappa = states(3);
    const auto& l_weight_terminal = params(8);
    const auto& kappa_weight_terminal = params(9);

    // Determine global variables
    const auto& l = globals(3);

    // Determine internal variables
    const double kappa_p2 = kappa * kappa;
    const double l_p2 = l * l;

    // Evaluation of Scalar cost
    double cost = 0.5 * kappa_p2 * kappa_weight_terminal + 0.5 * l_p2 * l_weight_terminal;

    return cost;
  }

  virtual void updateGradientDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                Eigen::VectorXd& gx) const override {
    // Determine stage variables
    const auto& kappa = states(3);
    const auto& l_weight_terminal = params(8);
    const auto& kappa_weight_terminal = params(9);

    // Determine global variables
    const auto& cos_thetar = globals(0);
    const auto& sin_thetar = globals(1);
    const auto& l = globals(3);

    // Evaluation of Vector gx
    gx(0) = -1.0 * l * l_weight_terminal * sin_thetar;
    gx(1) = cos_thetar * l * l_weight_terminal;
    gx(3) = kappa * kappa_weight_terminal;
  }

  virtual void updateHessianDxDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                 Eigen::MatrixXd& hxx) const override {
    // Determine stage variables
    const auto& l_weight_terminal = params(8);
    const auto& kappa_weight_terminal = params(9);

    // Determine global variables
    const auto& cos_thetar = globals(0);
    const auto& sin_thetar = globals(1);

    // Determine internal variables
    const double cos_thetar_p2 = cos_thetar * cos_thetar;
    const double sin_thetar_p2 = sin_thetar * sin_thetar;

    // Evaluation of SymmetricMatrix hxx
    hxx(0, 0) = l_weight_terminal * sin_thetar_p2;
    hxx(0, 1) = -1.0 * cos_thetar * l_weight_terminal * sin_thetar;
    hxx(1, 1) = cos_thetar_p2 * l_weight_terminal;
    hxx(3, 3) = kappa_weight_terminal;
    hxx(1, 0) = hxx(0, 1);
  }
};

}  //  namespace gpal::pnc::planning