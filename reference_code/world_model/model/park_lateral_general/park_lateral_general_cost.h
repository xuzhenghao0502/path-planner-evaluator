#pragma once

#include "ocp/cost_function.h"

namespace gpal::pnc::planning {

template <StageType type>
class ParkLateralGeneralCost : public CostFunction {
 public:
  ParkLateralGeneralCost() : CostFunction(5, 2) {}

  virtual ~ParkLateralGeneralCost() = default;

  virtual double evaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls) const override {
    // Determine stage variables
    const auto& kappa = states(4);
    const auto& dkappa = ctrls(0);
    const auto& slack_offset = ctrls(1);
    const auto& ref_weight = params(5);
    const auto& kappa_weight = params(6);
    const auto& dkappa_weight = params(7);
    const auto& weigth_slack_offset = params(8);
    const auto& weigth_prev_kappa = params(31);
    const auto& weight_prev_dkappa = params(32);
    const auto& prev_kappa = params(33);
    const auto& prev_dkappa = params(34);

    // Determine global variables
    const auto& l = globals(10);

    // Determine internal variables
    const double kappa_p2 = kappa * kappa;
    const double dkappa_p2 = dkappa * dkappa;
    const double slack_offset_p2 = slack_offset * slack_offset;
    const double internal_0 = kappa - prev_kappa;
    const double internal_1 = dkappa - prev_dkappa;
    const double l_p2 = l * l;
    const double internal_0_p2 = internal_0 * internal_0;
    const double internal_1_p2 = internal_1 * internal_1;

    // Evaluation of Scalar cost
    double cost = 0.5 * dkappa_p2 * dkappa_weight + 0.5 * internal_0_p2 * weigth_prev_kappa +
                  0.5 * internal_1_p2 * weight_prev_dkappa + 0.5 * kappa_p2 * kappa_weight + 0.5 * l_p2 * ref_weight +
                  0.5 * slack_offset_p2 * weigth_slack_offset;

    return cost;
  }

  virtual void updateGradient(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::VectorXd& gx,
                              Eigen::VectorXd& gu) const override {
    // Determine stage variables
    const auto& kappa = states(4);
    const auto& dkappa = ctrls(0);
    const auto& slack_offset = ctrls(1);
    const auto& ref_weight = params(5);
    const auto& kappa_weight = params(6);
    const auto& dkappa_weight = params(7);
    const auto& weigth_slack_offset = params(8);
    const auto& weigth_prev_kappa = params(31);
    const auto& weight_prev_dkappa = params(32);
    const auto& prev_kappa = params(33);
    const auto& prev_dkappa = params(34);

    // Determine global variables
    const auto& cos_thetar = globals(3);
    const auto& sin_thetar = globals(4);
    const auto& l = globals(10);

    // Evaluation of Vector gx
    gx(1) = -1.0 * l * ref_weight * sin_thetar;
    gx(2) = cos_thetar * l * ref_weight;
    gx(4) = kappa * kappa_weight + 0.5 * weigth_prev_kappa * (2 * kappa - 2 * prev_kappa);

    // Evaluation of Vector gu
    gu(0) = dkappa * dkappa_weight + 0.5 * weight_prev_dkappa * (2 * dkappa - 2 * prev_dkappa);
    gu(1) = slack_offset * weigth_slack_offset;
  }

  virtual void updateGradientDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                Eigen::VectorXd& gx) const override {
    // Determine stage variables
    const auto& kappa = states(4);
    const auto& ref_weight = params(5);
    const auto& kappa_weight = params(6);
    const auto& weigth_prev_kappa = params(31);
    const auto& prev_kappa = params(33);

    // Determine global variables
    const auto& cos_thetar = globals(3);
    const auto& sin_thetar = globals(4);
    const auto& l = globals(10);

    // Evaluation of Vector gx
    gx(1) = -1.0 * l * ref_weight * sin_thetar;
    gx(2) = cos_thetar * l * ref_weight;
    gx(4) = kappa * kappa_weight + 0.5 * weigth_prev_kappa * (2 * kappa - 2 * prev_kappa);
  }

  virtual void updateGradientDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                Eigen::VectorXd& gu) const override {
    // Determine stage variables
    const auto& dkappa = ctrls(0);
    const auto& slack_offset = ctrls(1);
    const auto& dkappa_weight = params(7);
    const auto& weigth_slack_offset = params(8);
    const auto& weight_prev_dkappa = params(32);
    const auto& prev_dkappa = params(34);

    // Evaluation of Vector gu
    gu(0) = dkappa * dkappa_weight + 0.5 * weight_prev_dkappa * (2 * dkappa - 2 * prev_dkappa);
    gu(1) = slack_offset * weigth_slack_offset;
  }

  virtual void updateHessian(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& dxdx,
                             Eigen::MatrixXd& dxdu, Eigen::MatrixXd& dudu) const override {
    // Determine stage variables
    const auto& ref_weight = params(5);
    const auto& kappa_weight = params(6);
    const auto& dkappa_weight = params(7);
    const auto& weigth_slack_offset = params(8);
    const auto& weigth_prev_kappa = params(31);
    const auto& weight_prev_dkappa = params(32);

    // Determine global variables
    const auto& cos_thetar = globals(3);
    const auto& sin_thetar = globals(4);

    // Determine internal variables
    const double cos_thetar_p2 = cos_thetar * cos_thetar;
    const double sin_thetar_p2 = sin_thetar * sin_thetar;

    // Evaluation of SymmetricMatrix dxdx
    dxdx(1, 1) = ref_weight * sin_thetar_p2;
    dxdx(1, 2) = -1.0 * cos_thetar * ref_weight * sin_thetar;
    dxdx(2, 2) = cos_thetar_p2 * ref_weight;
    dxdx(4, 4) = kappa_weight + weigth_prev_kappa;
    dxdx(2, 1) = dxdx(1, 2);

    // Evaluation of SymmetricMatrix dudu
    dudu(0, 0) = dkappa_weight + weight_prev_dkappa;
    dudu(1, 1) = weigth_slack_offset;
  }

  virtual void updateHessianDxDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                 Eigen::MatrixXd& dxdx) const override {
    // Determine stage variables
    const auto& ref_weight = params(5);
    const auto& kappa_weight = params(6);
    const auto& weigth_prev_kappa = params(31);

    // Determine global variables
    const auto& cos_thetar = globals(3);
    const auto& sin_thetar = globals(4);

    // Determine internal variables
    const double cos_thetar_p2 = cos_thetar * cos_thetar;
    const double sin_thetar_p2 = sin_thetar * sin_thetar;

    // Evaluation of SymmetricMatrix dxdx
    dxdx(1, 1) = ref_weight * sin_thetar_p2;
    dxdx(1, 2) = -1.0 * cos_thetar * ref_weight * sin_thetar;
    dxdx(2, 2) = cos_thetar_p2 * ref_weight;
    dxdx(4, 4) = kappa_weight + weigth_prev_kappa;
    dxdx(2, 1) = dxdx(1, 2);
  }

  virtual void updateHessianDxDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                 Eigen::MatrixXd& dxdu) const override {}

  virtual void updateHessianDuDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                 Eigen::MatrixXd& dudu) const override {
    // Determine stage variables
    const auto& dkappa_weight = params(7);
    const auto& weigth_slack_offset = params(8);
    const auto& weight_prev_dkappa = params(32);

    // Evaluation of SymmetricMatrix dudu
    dudu(0, 0) = dkappa_weight + weight_prev_dkappa;
    dudu(1, 1) = weigth_slack_offset;
  }

  virtual void updateGradient(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t,
                              Eigen::VectorXd& gx, Eigen::VectorXd& gu) const override {
    // Determine stage variables
    const auto& kappa = states(4);
    const auto& dkappa = ctrls(0);
    const auto& slack_offset = ctrls(1);
    const auto& ref_weight = params(5);
    const auto& kappa_weight = params(6);
    const auto& dkappa_weight = params(7);
    const auto& weigth_slack_offset = params(8);
    const auto& weigth_prev_kappa = params(31);
    const auto& weight_prev_dkappa = params(32);
    const auto& prev_kappa = params(33);
    const auto& prev_dkappa = params(34);

    // Determine global variables
    const auto& cos_thetar = globals(3);
    const auto& sin_thetar = globals(4);
    const auto& l = globals(10);

    // Evaluation of Vector gx
    gx(1) = -1.0 * delta_t * l * ref_weight * sin_thetar;
    gx(2) = cos_thetar * delta_t * l * ref_weight;
    gx(4) = delta_t * (kappa * kappa_weight + 0.5 * weigth_prev_kappa * (2 * kappa - 2 * prev_kappa));

    // Evaluation of Vector gu
    gu(0) = delta_t * (dkappa * dkappa_weight + 0.5 * weight_prev_dkappa * (2 * dkappa - 2 * prev_dkappa));
    gu(1) = delta_t * slack_offset * weigth_slack_offset;
  }

  virtual void updateGradientDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t,
                                Eigen::VectorXd& gx) const override {
    // Determine stage variables
    const auto& kappa = states(4);
    const auto& ref_weight = params(5);
    const auto& kappa_weight = params(6);
    const auto& weigth_prev_kappa = params(31);
    const auto& prev_kappa = params(33);

    // Determine global variables
    const auto& cos_thetar = globals(3);
    const auto& sin_thetar = globals(4);
    const auto& l = globals(10);

    // Evaluation of Vector gx
    gx(1) = -1.0 * delta_t * l * ref_weight * sin_thetar;
    gx(2) = cos_thetar * delta_t * l * ref_weight;
    gx(4) = delta_t * (kappa * kappa_weight + 0.5 * weigth_prev_kappa * (2 * kappa - 2 * prev_kappa));
  }

  virtual void updateGradientDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t,
                                Eigen::VectorXd& gu) const override {
    // Determine stage variables
    const auto& dkappa = ctrls(0);
    const auto& slack_offset = ctrls(1);
    const auto& dkappa_weight = params(7);
    const auto& weigth_slack_offset = params(8);
    const auto& weight_prev_dkappa = params(32);
    const auto& prev_dkappa = params(34);

    // Evaluation of Vector gu
    gu(0) = delta_t * (dkappa * dkappa_weight + 0.5 * weight_prev_dkappa * (2 * dkappa - 2 * prev_dkappa));
    gu(1) = delta_t * slack_offset * weigth_slack_offset;
  }

  virtual void updateHessian(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t,
                             Eigen::MatrixXd& dxdx, Eigen::MatrixXd& dxdu, Eigen::MatrixXd& dudu) const override {
    // Determine stage variables
    const auto& ref_weight = params(5);
    const auto& kappa_weight = params(6);
    const auto& dkappa_weight = params(7);
    const auto& weigth_slack_offset = params(8);
    const auto& weigth_prev_kappa = params(31);
    const auto& weight_prev_dkappa = params(32);

    // Determine global variables
    const auto& cos_thetar = globals(3);
    const auto& sin_thetar = globals(4);

    // Determine internal variables
    const double cos_thetar_p2 = cos_thetar * cos_thetar;
    const double sin_thetar_p2 = sin_thetar * sin_thetar;

    // Evaluation of SymmetricMatrix dxdx
    dxdx(1, 1) = delta_t * ref_weight * sin_thetar_p2;
    dxdx(1, 2) = -1.0 * cos_thetar * delta_t * ref_weight * sin_thetar;
    dxdx(2, 2) = cos_thetar_p2 * delta_t * ref_weight;
    dxdx(4, 4) = delta_t * (kappa_weight + weigth_prev_kappa);
    dxdx(2, 1) = dxdx(1, 2);

    // Evaluation of SymmetricMatrix dudu
    dudu(0, 0) = delta_t * (dkappa_weight + weight_prev_dkappa);
    dudu(1, 1) = delta_t * weigth_slack_offset;
  }

  virtual void updateHessianDxDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t,
                                 Eigen::MatrixXd& dxdx) const override {
    // Determine stage variables
    const auto& ref_weight = params(5);
    const auto& kappa_weight = params(6);
    const auto& weigth_prev_kappa = params(31);

    // Determine global variables
    const auto& cos_thetar = globals(3);
    const auto& sin_thetar = globals(4);

    // Determine internal variables
    const double cos_thetar_p2 = cos_thetar * cos_thetar;
    const double sin_thetar_p2 = sin_thetar * sin_thetar;

    // Evaluation of SymmetricMatrix dxdx
    dxdx(1, 1) = delta_t * ref_weight * sin_thetar_p2;
    dxdx(1, 2) = -1.0 * cos_thetar * delta_t * ref_weight * sin_thetar;
    dxdx(2, 2) = cos_thetar_p2 * delta_t * ref_weight;
    dxdx(4, 4) = delta_t * (kappa_weight + weigth_prev_kappa);
    dxdx(2, 1) = dxdx(1, 2);
  }

  virtual void updateHessianDxDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t,
                                 Eigen::MatrixXd& dxdu) const override {}

  virtual void updateHessianDuDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t,
                                 Eigen::MatrixXd& dudu) const override {
    // Determine stage variables
    const auto& dkappa_weight = params(7);
    const auto& weigth_slack_offset = params(8);
    const auto& weight_prev_dkappa = params(32);

    // Evaluation of SymmetricMatrix dudu
    dudu(0, 0) = delta_t * (dkappa_weight + weight_prev_dkappa);
    dudu(1, 1) = delta_t * weigth_slack_offset;
  }
};

template <>
class ParkLateralGeneralCost<StageType::TERMINAL> : public CostFunction {
 public:
  ParkLateralGeneralCost() : CostFunction(5, 2) {}
  virtual ~ParkLateralGeneralCost() = default;

  virtual double evaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls) const override {
    // Determine stage variables
    const auto& terminal_position_weight = params(9);
    const auto& terminal_heading_weight = params(10);

    // Determine global variables
    const auto& anonymous_0 = globals(2);
    const auto& l = globals(10);

    // Determine internal variables
    const double anonymous_0_p2 = anonymous_0 * anonymous_0;
    const double l_p2 = l * l;

    // Evaluation of Scalar cost
    double cost = 0.5 * anonymous_0_p2 * terminal_heading_weight + 0.5 * l_p2 * terminal_position_weight;

    return cost;
  }

  virtual void updateGradientDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                Eigen::VectorXd& gx) const override {
    // Determine stage variables
    const auto& theta = states(3);
    const auto& thetar = params(2);
    const auto& terminal_position_weight = params(9);
    const auto& terminal_heading_weight = params(10);

    // Determine global variables
    const auto& cos_thetar = globals(3);
    const auto& sin_thetar = globals(4);
    const auto& l = globals(10);

    // Evaluation of Vector gx
    gx(1) = -1.0 * l * sin_thetar * terminal_position_weight;
    gx(2) = cos_thetar * l * terminal_position_weight;
    gx(3) = 0.5 * terminal_heading_weight * (2 * theta - 2 * thetar);
  }

  virtual void updateHessianDxDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                 Eigen::MatrixXd& hxx) const override {
    // Determine stage variables
    const auto& terminal_position_weight = params(9);
    const auto& terminal_heading_weight = params(10);

    // Determine global variables
    const auto& cos_thetar = globals(3);
    const auto& sin_thetar = globals(4);

    // Determine internal variables
    const double cos_thetar_p2 = cos_thetar * cos_thetar;
    const double sin_thetar_p2 = sin_thetar * sin_thetar;

    // Evaluation of SymmetricMatrix hxx
    hxx(1, 1) = sin_thetar_p2 * terminal_position_weight;
    hxx(1, 2) = -1.0 * cos_thetar * sin_thetar * terminal_position_weight;
    hxx(2, 2) = cos_thetar_p2 * terminal_position_weight;
    hxx(3, 3) = terminal_heading_weight;
    hxx(2, 1) = hxx(1, 2);
  }
};

}  //  namespace gpal::pnc::planning