#pragma once

#include "ocp/constraints.h"

namespace gpal::pnc::planning {

template <StageType Ttype>
class LateralMultiGearConstraints : public Constraints {
 public:
  LateralMultiGearConstraints() : Constraints(4, 1, 2, 2, 0) {
    // Evaluation of Matrix jacobian_.jx
    jacobian_.jx(0, 3) = -1;
    jacobian_.jx(1, 3) = 1;

    // Evaluation of Vector jacobian_.ju
    jacobian_.ju(2) = -1;
    jacobian_.ju(3) = 1;
    exprs_[0] = "kappa_lower <= kappa";
    exprs_[1] = "kappa <= kappa_upper";
    exprs_[2] = "dkappa_lower <= dkappa";
    exprs_[3] = "dkappa <= dkappa_upper";
  }

  virtual ~LateralMultiGearConstraints() = default;

  virtual Type type() const override { return Type::INEQUALITY; }
  virtual void updateEvaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                              Eigen::VectorXd& values) const override {
    // Determine stage variables
    const auto& kappa = states(3);
    const auto& dkappa = ctrls(0);
    const auto& kappa_lower = params(11);
    const auto& kappa_upper = params(12);
    const auto& dkappa_lower = params(13);
    const auto& dkappa_upper = params(14);

    // Evaluation of Vector values
    values(0) = -kappa + kappa_lower;
    values(1) = kappa - kappa_upper;
    values(2) = -dkappa + dkappa_lower;
    values(3) = dkappa - dkappa_upper;
  }

  virtual void updateJacobian(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& jx,
                              Eigen::MatrixXd& ju) const override {
    // Evaluation of Matrix jx
    jx(0, 3) = -1;
    jx(1, 3) = 1;

    // Evaluation of Matrix ju
    ju(2, 0) = -1;
    ju(3, 0) = 1;
  }

  virtual void updateStateJacobian(const Eigen::VectorXd& states, Eigen::MatrixXd& jx) const override {
    // Evaluation of Matrix jx
    jx(0, 3) = -1;
    jx(1, 3) = 1;
  }

  virtual void updateCtrlJacobian(const Eigen::VectorXd& ctrls, Eigen::MatrixXd& ju) const override {
    // Evaluation of Matrix ju
    ju(0, 0) = -1;
    ju(1, 0) = 1;
  }

  virtual void updateJacobianDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                Eigen::MatrixXd& jx) const override {
    // Evaluation of Matrix jx
    jx(0, 3) = -1;
    jx(1, 3) = 1;
  }

  virtual void updateJacobianDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                Eigen::MatrixXd& ju) const override {
    // Evaluation of Matrix ju
    ju(2, 0) = -1;
    ju(3, 0) = 1;
  }

  virtual void updateDeltaEvaluation(const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::VectorXd& dx,
                                     const Eigen::VectorXd& du, Eigen::VectorXd& dg) const override {
    // Evaluation of Vector dg
    dg(0) = dx(3) * g_x(0, 3);
    dg(1) = dx(3) * g_x(1, 3);
    dg(2) = du(0) * g_u(2, 0);
    dg(3) = du(0) * g_u(3, 0);
  }
};

template <>
class LateralMultiGearConstraints<StageType::INITIAL> : public Constraints {
 public:
  LateralMultiGearConstraints() : Constraints(4, 1, 0, 2, 0) {
    // Evaluation of Vector jacobian_.ju
    jacobian_.ju(0) = -1;
    jacobian_.ju(1) = 1;
    exprs_[0] = "dkappa_lower <= dkappa";
    exprs_[1] = "dkappa <= dkappa_upper";
  }

  virtual ~LateralMultiGearConstraints() = default;

  virtual Type type() const override { return Type::INEQUALITY; }
  virtual void updateEvaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                              Eigen::VectorXd& values) const override {
    // Determine stage variables
    const auto& dkappa = ctrls(0);
    const auto& dkappa_lower = params(13);
    const auto& dkappa_upper = params(14);

    // Evaluation of Vector values
    values(0) = -dkappa + dkappa_lower;
    values(1) = dkappa - dkappa_upper;
  }

  virtual void updateJacobian(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& jx,
                              Eigen::MatrixXd& ju) const override {
    // Evaluation of Matrix ju
    ju(0, 0) = -1;
    ju(1, 0) = 1;
  }

  virtual void updateCtrlJacobian(const Eigen::VectorXd& ctrls, Eigen::MatrixXd& ju) const override {
    // Evaluation of Matrix ju
    ju(0, 0) = -1;
    ju(1, 0) = 1;
  }

  virtual void updateJacobianDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                Eigen::MatrixXd& ju) const override {
    // Evaluation of Matrix ju
    ju(0, 0) = -1;
    ju(1, 0) = 1;
  }

  virtual void updateDeltaEvaluation(const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::VectorXd& dx,
                                     const Eigen::VectorXd& du, Eigen::VectorXd& dg) const override {
    // Evaluation of Vector dg
    dg(0) = du(0) * g_u(0, 0);
    dg(1) = du(0) * g_u(1, 0);
  }
};

template <>
class LateralMultiGearConstraints<StageType::TERMINAL> : public Constraints {
 public:
  LateralMultiGearConstraints() : Constraints(4, 1, 2, 0, 0) {
    // Evaluation of Matrix jacobian_.jx
    jacobian_.jx(0, 3) = -1;
    jacobian_.jx(1, 3) = 1;

    exprs_[0] = "kappa_lower <= kappa";
    exprs_[1] = "kappa <= kappa_upper";
  }

  virtual ~LateralMultiGearConstraints() = default;

  virtual Type type() const override { return Type::INEQUALITY; }
  virtual void updateEvaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                              Eigen::VectorXd& values) const override {
    // Determine stage variables
    const auto& kappa = states(3);
    const auto& kappa_lower = params(11);
    const auto& kappa_upper = params(12);

    // Evaluation of Vector values
    values(0) = -kappa + kappa_lower;
    values(1) = kappa - kappa_upper;
  }

  virtual void updateJacobian(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& jx,
                              Eigen::MatrixXd& ju) const override {
    // Evaluation of Matrix jx
    jx(0, 3) = -1;
    jx(1, 3) = 1;
  }

  virtual void updateStateJacobian(const Eigen::VectorXd& states, Eigen::MatrixXd& jx) const override {
    // Evaluation of Matrix jx
    jx(0, 3) = -1;
    jx(1, 3) = 1;
  }

  virtual void updateJacobianDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                Eigen::MatrixXd& jx) const override {
    // Evaluation of Matrix jx
    jx(0, 3) = -1;
    jx(1, 3) = 1;
  }

  virtual void updateDeltaEvaluation(const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::VectorXd& dx,
                                     const Eigen::VectorXd& du, Eigen::VectorXd& dg) const override {
    // Evaluation of Vector dg
    dg(0) = dx(3) * g_x(0, 3);
    dg(1) = dx(3) * g_x(1, 3);
  }
};

template <StageType Ttype>
class LateralMultiGearStateOnlyEqualities : public Constraints {
 public:
  LateralMultiGearStateOnlyEqualities() : Constraints(4, 1, 0, 0, 0) {}

  virtual ~LateralMultiGearStateOnlyEqualities() = default;

  virtual Type type() const override { return Type::STATE_ONLY_EQUALITY; }
};

template <>
class LateralMultiGearStateOnlyEqualities<StageType::INITIAL> : public Constraints {
 public:
  LateralMultiGearStateOnlyEqualities() : Constraints(4, 1, 4, 0, 0) {
    // Evaluation of SymmetricMatrix jacobian_.jx
    jacobian_.jx(0, 0) = 1;
    jacobian_.jx(1, 1) = 1;
    jacobian_.jx(2, 2) = 1;
    jacobian_.jx(3, 3) = 1;

    exprs_[0] = "x = 0";
    exprs_[1] = "y = 0";
    exprs_[2] = "theta = 0";
    exprs_[3] = "kappa = 0";
  }

  virtual ~LateralMultiGearStateOnlyEqualities() = default;

  virtual Type type() const override { return Type::STATE_ONLY_EQUALITY; }
  virtual void updateEvaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                              Eigen::VectorXd& values) const override {
    // Determine stage variables
    const auto& x = states(0);
    const auto& y = states(1);
    const auto& theta = states(2);
    const auto& kappa = states(3);

    // Evaluation of Vector values
    values(0) = x;
    values(1) = y;
    values(2) = theta;
    values(3) = kappa;
  }

  virtual void updateJacobian(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& jx,
                              Eigen::MatrixXd& ju) const override {
    // Evaluation of Matrix jx
    jx(0, 0) = 1;
    jx(1, 1) = 1;
    jx(2, 2) = 1;
    jx(3, 3) = 1;
  }

  virtual void updateStateJacobian(const Eigen::VectorXd& states, Eigen::MatrixXd& jx) const override {
    // Evaluation of Matrix jx
    jx(0, 0) = 1;
    jx(1, 1) = 1;
    jx(2, 2) = 1;
    jx(3, 3) = 1;
  }

  virtual void updateJacobianDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                Eigen::MatrixXd& jx) const override {
    // Evaluation of Matrix jx
    jx(0, 0) = 1;
    jx(1, 1) = 1;
    jx(2, 2) = 1;
    jx(3, 3) = 1;
  }

  virtual void updateDeltaEvaluation(const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::VectorXd& dx,
                                     const Eigen::VectorXd& du, Eigen::VectorXd& dg) const override {
    // Evaluation of Vector dg
    dg(0) = dx(0) * g_x(0, 0);
    dg(1) = dx(1) * g_x(1, 1);
    dg(2) = dx(2) * g_x(2, 2);
    dg(3) = dx(3) * g_x(3, 3);
  }
};

template <>
class LateralMultiGearStateOnlyEqualities<StageType::TERMINAL> : public Constraints {
 public:
  LateralMultiGearStateOnlyEqualities() : Constraints(4, 1, 0, 0, 0) {}

  virtual ~LateralMultiGearStateOnlyEqualities() = default;

  virtual Type type() const override { return Type::STATE_ONLY_EQUALITY; }
};

}  // namespace gpal::pnc::planning