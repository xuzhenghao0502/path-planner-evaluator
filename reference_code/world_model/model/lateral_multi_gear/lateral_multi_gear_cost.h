#pragma once

#include "ocp/cost_function.h"

namespace gpal::pnc::planning {

template <StageType type>
class LateralMultiGearCost : public CostFunction {
 public:
  LateralMultiGearCost() : CostFunction(4, 1) {}

  virtual ~LateralMultiGearCost() = default;

  virtual double evaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls) const override {
    // Determine stage variables
    const auto& x = states(0);
    const auto& y = states(1);
    const auto& theta = states(2);
    const auto& kappa = states(3);
    const auto& dkappa = ctrls(0);
    const auto& xr = params(0);
    const auto& yr = params(1);
    const auto& thetar = params(2);
    const auto& kr = params(3);
    const auto& weight_position = params(5);
    const auto& weight_heading = params(6);
    const auto& weight_kappa = params(7);
    const auto& weight_dkappa = params(8);

    // Determine internal variables
    const double dkappa_p2 = dkappa * dkappa;
    const double internal_3 = x - xr;
    const double internal_2 = y - yr;
    const double internal_1 = theta - thetar;
    const double internal_0 = kappa - kr;
    const double internal_3_p2 = internal_3 * internal_3;
    const double internal_2_p2 = internal_2 * internal_2;
    const double internal_1_p2 = internal_1 * internal_1;
    const double internal_0_p2 = internal_0 * internal_0;

    // Evaluation of Scalar cost
    double cost = 0.5 * dkappa_p2 * weight_dkappa + 0.5 * internal_0_p2 * weight_kappa +
                  0.5 * internal_1_p2 * weight_heading + 0.5 * weight_position * (internal_2_p2 + internal_3_p2);

    return cost;
  }

  virtual void updateGradient(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::VectorXd& gx,
                              Eigen::VectorXd& gu) const override {
    // Determine stage variables
    const auto& x = states(0);
    const auto& y = states(1);
    const auto& theta = states(2);
    const auto& kappa = states(3);
    const auto& dkappa = ctrls(0);
    const auto& xr = params(0);
    const auto& yr = params(1);
    const auto& thetar = params(2);
    const auto& kr = params(3);
    const auto& weight_position = params(5);
    const auto& weight_heading = params(6);
    const auto& weight_kappa = params(7);
    const auto& weight_dkappa = params(8);

    // Evaluation of Vector gx
    gx(0) = 0.5 * weight_position * (2 * x - 2 * xr);
    gx(1) = 0.5 * weight_position * (2 * y - 2 * yr);
    gx(2) = 0.5 * weight_heading * (2 * theta - 2 * thetar);
    gx(3) = 0.5 * weight_kappa * (2 * kappa - 2 * kr);

    // Evaluation of Vector gu
    gu(0) = dkappa * weight_dkappa;
  }

  virtual void updateGradientDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                Eigen::VectorXd& gx) const override {
    // Determine stage variables
    const auto& x = states(0);
    const auto& y = states(1);
    const auto& theta = states(2);
    const auto& kappa = states(3);
    const auto& xr = params(0);
    const auto& yr = params(1);
    const auto& thetar = params(2);
    const auto& kr = params(3);
    const auto& weight_position = params(5);
    const auto& weight_heading = params(6);
    const auto& weight_kappa = params(7);

    // Evaluation of Vector gx
    gx(0) = 0.5 * weight_position * (2 * x - 2 * xr);
    gx(1) = 0.5 * weight_position * (2 * y - 2 * yr);
    gx(2) = 0.5 * weight_heading * (2 * theta - 2 * thetar);
    gx(3) = 0.5 * weight_kappa * (2 * kappa - 2 * kr);
  }

  virtual void updateGradientDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                Eigen::VectorXd& gu) const override {
    // Determine stage variables
    const auto& dkappa = ctrls(0);
    const auto& weight_dkappa = params(8);

    // Evaluation of Vector gu
    gu(0) = dkappa * weight_dkappa;
  }

  virtual void updateHessian(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& dxdx,
                             Eigen::MatrixXd& dxdu, Eigen::MatrixXd& dudu) const override {
    // Determine stage variables
    const auto& weight_position = params(5);
    const auto& weight_heading = params(6);
    const auto& weight_kappa = params(7);
    const auto& weight_dkappa = params(8);

    // Evaluation of SymmetricMatrix dxdx
    dxdx(0, 0) = weight_position;
    dxdx(1, 1) = weight_position;
    dxdx(2, 2) = weight_heading;
    dxdx(3, 3) = weight_kappa;

    // Evaluation of SymmetricMatrix dudu
    dudu(0, 0) = weight_dkappa;
  }

  virtual void updateHessianDxDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                 Eigen::MatrixXd& dxdx) const override {
    // Determine stage variables
    const auto& weight_position = params(5);
    const auto& weight_heading = params(6);
    const auto& weight_kappa = params(7);

    // Evaluation of SymmetricMatrix dxdx
    dxdx(0, 0) = weight_position;
    dxdx(1, 1) = weight_position;
    dxdx(2, 2) = weight_heading;
    dxdx(3, 3) = weight_kappa;
  }

  virtual void updateHessianDxDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                 Eigen::MatrixXd& dxdu) const override {}

  virtual void updateHessianDuDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                 Eigen::MatrixXd& dudu) const override {
    // Determine stage variables
    const auto& weight_dkappa = params(8);

    // Evaluation of SymmetricMatrix dudu
    dudu(0, 0) = weight_dkappa;
  }

  virtual void updateGradient(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t,
                              Eigen::VectorXd& gx, Eigen::VectorXd& gu) const override {
    // Determine stage variables
    const auto& x = states(0);
    const auto& y = states(1);
    const auto& theta = states(2);
    const auto& kappa = states(3);
    const auto& dkappa = ctrls(0);
    const auto& xr = params(0);
    const auto& yr = params(1);
    const auto& thetar = params(2);
    const auto& kr = params(3);
    const auto& weight_position = params(5);
    const auto& weight_heading = params(6);
    const auto& weight_kappa = params(7);
    const auto& weight_dkappa = params(8);

    // Evaluation of Vector gx
    gx(0) = 0.5 * delta_t * weight_position * (2 * x - 2 * xr);
    gx(1) = 0.5 * delta_t * weight_position * (2 * y - 2 * yr);
    gx(2) = 0.5 * delta_t * weight_heading * (2 * theta - 2 * thetar);
    gx(3) = 0.5 * delta_t * weight_kappa * (2 * kappa - 2 * kr);

    // Evaluation of Vector gu
    gu(0) = delta_t * dkappa * weight_dkappa;
  }

  virtual void updateGradientDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t,
                                Eigen::VectorXd& gx) const override {
    // Determine stage variables
    const auto& x = states(0);
    const auto& y = states(1);
    const auto& theta = states(2);
    const auto& kappa = states(3);
    const auto& xr = params(0);
    const auto& yr = params(1);
    const auto& thetar = params(2);
    const auto& kr = params(3);
    const auto& weight_position = params(5);
    const auto& weight_heading = params(6);
    const auto& weight_kappa = params(7);

    // Evaluation of Vector gx
    gx(0) = 0.5 * delta_t * weight_position * (2 * x - 2 * xr);
    gx(1) = 0.5 * delta_t * weight_position * (2 * y - 2 * yr);
    gx(2) = 0.5 * delta_t * weight_heading * (2 * theta - 2 * thetar);
    gx(3) = 0.5 * delta_t * weight_kappa * (2 * kappa - 2 * kr);
  }

  virtual void updateGradientDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t,
                                Eigen::VectorXd& gu) const override {
    // Determine stage variables
    const auto& dkappa = ctrls(0);
    const auto& weight_dkappa = params(8);

    // Evaluation of Vector gu
    gu(0) = delta_t * dkappa * weight_dkappa;
  }

  virtual void updateHessian(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t,
                             Eigen::MatrixXd& dxdx, Eigen::MatrixXd& dxdu, Eigen::MatrixXd& dudu) const override {
    // Determine stage variables
    const auto& weight_position = params(5);
    const auto& weight_heading = params(6);
    const auto& weight_kappa = params(7);
    const auto& weight_dkappa = params(8);

    // Evaluation of SymmetricMatrix dxdx
    dxdx(0, 0) = delta_t * weight_position;
    dxdx(1, 1) = dxdx(0, 0);
    dxdx(2, 2) = delta_t * weight_heading;
    dxdx(3, 3) = delta_t * weight_kappa;

    // Evaluation of SymmetricMatrix dudu
    dudu(0, 0) = delta_t * weight_dkappa;
  }

  virtual void updateHessianDxDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t,
                                 Eigen::MatrixXd& dxdx) const override {
    // Determine stage variables
    const auto& weight_position = params(5);
    const auto& weight_heading = params(6);
    const auto& weight_kappa = params(7);

    // Evaluation of SymmetricMatrix dxdx
    dxdx(0, 0) = delta_t * weight_position;
    dxdx(1, 1) = dxdx(0, 0);
    dxdx(2, 2) = delta_t * weight_heading;
    dxdx(3, 3) = delta_t * weight_kappa;
  }

  virtual void updateHessianDxDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t,
                                 Eigen::MatrixXd& dxdu) const override {}

  virtual void updateHessianDuDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t,
                                 Eigen::MatrixXd& dudu) const override {
    // Determine stage variables
    const auto& weight_dkappa = params(8);

    // Evaluation of SymmetricMatrix dudu
    dudu(0, 0) = delta_t * weight_dkappa;
  }
};

template <>
class LateralMultiGearCost<StageType::TERMINAL> : public CostFunction {
 public:
  LateralMultiGearCost() : CostFunction(4, 1) {}
  virtual ~LateralMultiGearCost() = default;

  virtual double evaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls) const override {
    // Determine stage variables
    const auto& theta = states(2);
    const auto& thetar = params(2);
    const auto& terminal_position_weight = params(9);
    const auto& terminal_heading_weight = params(10);

    // Determine global variables
    const auto& l = globals(2);

    // Determine internal variables
    const double internal_0 = theta - thetar;
    const double l_p2 = l * l;
    const double internal_0_p2 = internal_0 * internal_0;

    // Evaluation of Scalar cost
    double cost = 0.5 * internal_0_p2 * terminal_heading_weight + 0.5 * l_p2 * terminal_position_weight;

    return cost;
  }

  virtual void updateGradientDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                Eigen::VectorXd& gx) const override {
    // Determine stage variables
    const auto& theta = states(2);
    const auto& thetar = params(2);
    const auto& terminal_position_weight = params(9);
    const auto& terminal_heading_weight = params(10);

    // Determine global variables
    const auto& cos_thetar = globals(0);
    const auto& sin_thetar = globals(1);
    const auto& l = globals(2);

    // Evaluation of Vector gx
    gx(0) = -1.0 * l * sin_thetar * terminal_position_weight;
    gx(1) = cos_thetar * l * terminal_position_weight;
    gx(2) = 0.5 * terminal_heading_weight * (2 * theta - 2 * thetar);
  }

  virtual void updateHessianDxDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                 Eigen::MatrixXd& hxx) const override {
    // Determine stage variables
    const auto& terminal_position_weight = params(9);
    const auto& terminal_heading_weight = params(10);

    // Determine global variables
    const auto& cos_thetar = globals(0);
    const auto& sin_thetar = globals(1);

    // Determine internal variables
    const double cos_thetar_p2 = cos_thetar * cos_thetar;
    const double sin_thetar_p2 = sin_thetar * sin_thetar;

    // Evaluation of SymmetricMatrix hxx
    hxx(0, 0) = sin_thetar_p2 * terminal_position_weight;
    hxx(0, 1) = -1.0 * cos_thetar * sin_thetar * terminal_position_weight;
    hxx(1, 1) = cos_thetar_p2 * terminal_position_weight;
    hxx(2, 2) = terminal_heading_weight;
    hxx(1, 0) = hxx(0, 1);
  }
};

}  //  namespace gpal::pnc::planning