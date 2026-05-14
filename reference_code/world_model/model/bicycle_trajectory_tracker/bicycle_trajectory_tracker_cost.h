#pragma once

#include "ocp/cost_function.h"

namespace gpal::pnc::planning {

template <StageType type>
class BicycleTrajectoryTrackerCost : public CostFunction {
 public:
  BicycleTrajectoryTrackerCost() : CostFunction(5, 2) {}

  virtual ~BicycleTrajectoryTrackerCost() = default;

  virtual double evaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls) const override {
    // Determine stage variables
    const auto& x = states(0);
    const auto& y = states(1);
    const auto& theta = states(2);
    const auto& v = states(3);
    const auto& kappa = states(4);
    const auto& a = ctrls(0);
    const auto& dkappa = ctrls(1);
    const auto& xr = params(0);
    const auto& yr = params(1);
    const auto& thetar = params(2);
    const auto& vr = params(3);
    const auto& kr = params(4);
    const auto& weight_position = params(5);
    const auto& weight_heading = params(6);
    const auto& weight_v = params(7);
    const auto& weight_kappa = params(8);
    const auto& weight_a = params(9);
    const auto& weight_dkappa = params(10);

    // Determine internal variables
    const double a_p2 = a * a;
    const double dkappa_p2 = dkappa * dkappa;
    const double internal_3 = x - xr;
    const double internal_1 = y - yr;
    const double internal_4 = theta - thetar;
    const double internal_0 = v - vr;
    const double internal_2 = kappa - kr;
    const double internal_3_p2 = internal_3 * internal_3;
    const double internal_1_p2 = internal_1 * internal_1;
    const double internal_4_p2 = internal_4 * internal_4;
    const double internal_0_p2 = internal_0 * internal_0;
    const double internal_2_p2 = internal_2 * internal_2;

    // Evaluation of Scalar cost
    double cost = 0.5 * a_p2 * weight_a + 0.5 * dkappa_p2 * weight_dkappa + 0.5 * internal_0_p2 * weight_v +
                  0.5 * internal_2_p2 * weight_kappa + 0.5 * internal_4_p2 * weight_heading +
                  0.5 * weight_position * (internal_1_p2 + internal_3_p2);

    return cost;
  }

  virtual void updateGradient(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::VectorXd& gx,
                              Eigen::VectorXd& gu) const override {
    // Determine stage variables
    const auto& x = states(0);
    const auto& y = states(1);
    const auto& theta = states(2);
    const auto& v = states(3);
    const auto& kappa = states(4);
    const auto& a = ctrls(0);
    const auto& dkappa = ctrls(1);
    const auto& xr = params(0);
    const auto& yr = params(1);
    const auto& thetar = params(2);
    const auto& vr = params(3);
    const auto& kr = params(4);
    const auto& weight_position = params(5);
    const auto& weight_heading = params(6);
    const auto& weight_v = params(7);
    const auto& weight_kappa = params(8);
    const auto& weight_a = params(9);
    const auto& weight_dkappa = params(10);

    // Evaluation of Vector gx
    gx(0) = 0.5 * weight_position * (2 * x - 2 * xr);
    gx(1) = 0.5 * weight_position * (2 * y - 2 * yr);
    gx(2) = 0.5 * weight_heading * (2 * theta - 2 * thetar);
    gx(3) = 0.5 * weight_v * (2 * v - 2 * vr);
    gx(4) = 0.5 * weight_kappa * (2 * kappa - 2 * kr);

    // Evaluation of Vector gu
    gu(0) = a * weight_a;
    gu(1) = dkappa * weight_dkappa;
  }

  virtual void updateGradientDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                Eigen::VectorXd& gx) const override {
    // Determine stage variables
    const auto& x = states(0);
    const auto& y = states(1);
    const auto& theta = states(2);
    const auto& v = states(3);
    const auto& kappa = states(4);
    const auto& xr = params(0);
    const auto& yr = params(1);
    const auto& thetar = params(2);
    const auto& vr = params(3);
    const auto& kr = params(4);
    const auto& weight_position = params(5);
    const auto& weight_heading = params(6);
    const auto& weight_v = params(7);
    const auto& weight_kappa = params(8);

    // Evaluation of Vector gx
    gx(0) = 0.5 * weight_position * (2 * x - 2 * xr);
    gx(1) = 0.5 * weight_position * (2 * y - 2 * yr);
    gx(2) = 0.5 * weight_heading * (2 * theta - 2 * thetar);
    gx(3) = 0.5 * weight_v * (2 * v - 2 * vr);
    gx(4) = 0.5 * weight_kappa * (2 * kappa - 2 * kr);
  }

  virtual void updateGradientDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                Eigen::VectorXd& gu) const override {
    // Determine stage variables
    const auto& a = ctrls(0);
    const auto& dkappa = ctrls(1);
    const auto& weight_a = params(9);
    const auto& weight_dkappa = params(10);

    // Evaluation of Vector gu
    gu(0) = a * weight_a;
    gu(1) = dkappa * weight_dkappa;
  }

  virtual void updateHessian(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& dxdx,
                             Eigen::MatrixXd& dxdu, Eigen::MatrixXd& dudu) const override {
    // Determine stage variables
    const auto& weight_position = params(5);
    const auto& weight_heading = params(6);
    const auto& weight_v = params(7);
    const auto& weight_kappa = params(8);
    const auto& weight_a = params(9);
    const auto& weight_dkappa = params(10);

    // Evaluation of SymmetricMatrix dxdx
    dxdx(0, 0) = weight_position;
    dxdx(1, 1) = weight_position;
    dxdx(2, 2) = weight_heading;
    dxdx(3, 3) = weight_v;
    dxdx(4, 4) = weight_kappa;

    // Evaluation of SymmetricMatrix dudu
    dudu(0, 0) = weight_a;
    dudu(1, 1) = weight_dkappa;
  }

  virtual void updateHessianDxDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                 Eigen::MatrixXd& dxdx) const override {
    // Determine stage variables
    const auto& weight_position = params(5);
    const auto& weight_heading = params(6);
    const auto& weight_v = params(7);
    const auto& weight_kappa = params(8);

    // Evaluation of SymmetricMatrix dxdx
    dxdx(0, 0) = weight_position;
    dxdx(1, 1) = weight_position;
    dxdx(2, 2) = weight_heading;
    dxdx(3, 3) = weight_v;
    dxdx(4, 4) = weight_kappa;
  }

  virtual void updateHessianDxDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                 Eigen::MatrixXd& dxdu) const override {}

  virtual void updateHessianDuDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                 Eigen::MatrixXd& dudu) const override {
    // Determine stage variables
    const auto& weight_a = params(9);
    const auto& weight_dkappa = params(10);

    // Evaluation of SymmetricMatrix dudu
    dudu(0, 0) = weight_a;
    dudu(1, 1) = weight_dkappa;
  }

  virtual void updateGradient(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t,
                              Eigen::VectorXd& gx, Eigen::VectorXd& gu) const override {
    // Determine stage variables
    const auto& x = states(0);
    const auto& y = states(1);
    const auto& theta = states(2);
    const auto& v = states(3);
    const auto& kappa = states(4);
    const auto& a = ctrls(0);
    const auto& dkappa = ctrls(1);
    const auto& xr = params(0);
    const auto& yr = params(1);
    const auto& thetar = params(2);
    const auto& vr = params(3);
    const auto& kr = params(4);
    const auto& weight_position = params(5);
    const auto& weight_heading = params(6);
    const auto& weight_v = params(7);
    const auto& weight_kappa = params(8);
    const auto& weight_a = params(9);
    const auto& weight_dkappa = params(10);

    // Evaluation of Vector gx
    gx(0) = 0.5 * delta_t * weight_position * (2 * x - 2 * xr);
    gx(1) = 0.5 * delta_t * weight_position * (2 * y - 2 * yr);
    gx(2) = 0.5 * delta_t * weight_heading * (2 * theta - 2 * thetar);
    gx(3) = 0.5 * delta_t * weight_v * (2 * v - 2 * vr);
    gx(4) = 0.5 * delta_t * weight_kappa * (2 * kappa - 2 * kr);

    // Evaluation of Vector gu
    gu(0) = a * delta_t * weight_a;
    gu(1) = delta_t * dkappa * weight_dkappa;
  }

  virtual void updateGradientDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t,
                                Eigen::VectorXd& gx) const override {
    // Determine stage variables
    const auto& x = states(0);
    const auto& y = states(1);
    const auto& theta = states(2);
    const auto& v = states(3);
    const auto& kappa = states(4);
    const auto& xr = params(0);
    const auto& yr = params(1);
    const auto& thetar = params(2);
    const auto& vr = params(3);
    const auto& kr = params(4);
    const auto& weight_position = params(5);
    const auto& weight_heading = params(6);
    const auto& weight_v = params(7);
    const auto& weight_kappa = params(8);

    // Evaluation of Vector gx
    gx(0) = 0.5 * delta_t * weight_position * (2 * x - 2 * xr);
    gx(1) = 0.5 * delta_t * weight_position * (2 * y - 2 * yr);
    gx(2) = 0.5 * delta_t * weight_heading * (2 * theta - 2 * thetar);
    gx(3) = 0.5 * delta_t * weight_v * (2 * v - 2 * vr);
    gx(4) = 0.5 * delta_t * weight_kappa * (2 * kappa - 2 * kr);
  }

  virtual void updateGradientDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t,
                                Eigen::VectorXd& gu) const override {
    // Determine stage variables
    const auto& a = ctrls(0);
    const auto& dkappa = ctrls(1);
    const auto& weight_a = params(9);
    const auto& weight_dkappa = params(10);

    // Evaluation of Vector gu
    gu(0) = a * delta_t * weight_a;
    gu(1) = delta_t * dkappa * weight_dkappa;
  }

  virtual void updateHessian(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t,
                             Eigen::MatrixXd& dxdx, Eigen::MatrixXd& dxdu, Eigen::MatrixXd& dudu) const override {
    // Determine stage variables
    const auto& weight_position = params(5);
    const auto& weight_heading = params(6);
    const auto& weight_v = params(7);
    const auto& weight_kappa = params(8);
    const auto& weight_a = params(9);
    const auto& weight_dkappa = params(10);

    // Evaluation of SymmetricMatrix dxdx
    dxdx(0, 0) = delta_t * weight_position;
    dxdx(1, 1) = dxdx(0, 0);
    dxdx(2, 2) = delta_t * weight_heading;
    dxdx(3, 3) = delta_t * weight_v;
    dxdx(4, 4) = delta_t * weight_kappa;

    // Evaluation of SymmetricMatrix dudu
    dudu(0, 0) = delta_t * weight_a;
    dudu(1, 1) = delta_t * weight_dkappa;
  }

  virtual void updateHessianDxDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t,
                                 Eigen::MatrixXd& dxdx) const override {
    // Determine stage variables
    const auto& weight_position = params(5);
    const auto& weight_heading = params(6);
    const auto& weight_v = params(7);
    const auto& weight_kappa = params(8);

    // Evaluation of SymmetricMatrix dxdx
    dxdx(0, 0) = delta_t * weight_position;
    dxdx(1, 1) = dxdx(0, 0);
    dxdx(2, 2) = delta_t * weight_heading;
    dxdx(3, 3) = delta_t * weight_v;
    dxdx(4, 4) = delta_t * weight_kappa;
  }

  virtual void updateHessianDxDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t,
                                 Eigen::MatrixXd& dxdu) const override {}

  virtual void updateHessianDuDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, const double delta_t,
                                 Eigen::MatrixXd& dudu) const override {
    // Determine stage variables
    const auto& weight_a = params(9);
    const auto& weight_dkappa = params(10);

    // Evaluation of SymmetricMatrix dudu
    dudu(0, 0) = delta_t * weight_a;
    dudu(1, 1) = delta_t * weight_dkappa;
  }
};

}  //  namespace gpal::pnc::planning