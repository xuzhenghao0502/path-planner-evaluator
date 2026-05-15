#pragma once

#include "ocp/constraints.h"

namespace gpal::pnc::planning {

template <StageType Ttype>
class BicycleTrajectoryTrackerConstraints : public Constraints {
 public:
  BicycleTrajectoryTrackerConstraints() : Constraints(5, 2, 4, 4, 0) {
    // Evaluation of Matrix jacobian_.jx
    jacobian_.jx(0, 3) = -1;
    jacobian_.jx(1, 3) = 1;
    jacobian_.jx(2, 4) = -1;
    jacobian_.jx(3, 4) = 1;

    // Evaluation of Matrix jacobian_.ju
    jacobian_.ju(4, 0) = -1;
    jacobian_.ju(5, 0) = 1;
    jacobian_.ju(6, 1) = -1;
    jacobian_.ju(7, 1) = 1;
    exprs_[0] = "v_lower <= v";
    exprs_[1] = "v <= v_upper";
    exprs_[2] = "kappa_lower <= kappa";
    exprs_[3] = "kappa <= kappa_upper";
    exprs_[4] = "a_lower <= a";
    exprs_[5] = "a <= a_upper";
    exprs_[6] = "dkappa_lower <= dkappa";
    exprs_[7] = "dkappa <= dkappa_upper";
  }

  virtual ~BicycleTrajectoryTrackerConstraints() = default;

  virtual Type type() const override { return Type::INEQUALITY; }
  virtual void updateEvaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                              Eigen::VectorXd& values) const override {
    // Determine stage variables
    const auto& v = states(3);
    const auto& kappa = states(4);
    const auto& a = ctrls(0);
    const auto& dkappa = ctrls(1);
    const auto& v_lower = params(11);
    const auto& v_upper = params(12);
    const auto& a_lower = params(13);
    const auto& a_upper = params(14);
    const auto& kappa_lower = params(15);
    const auto& kappa_upper = params(16);
    const auto& dkappa_lower = params(17);
    const auto& dkappa_upper = params(18);

    // Evaluation of Vector values
    values(0) = -v + v_lower;
    values(1) = v - v_upper;
    values(2) = -kappa + kappa_lower;
    values(3) = kappa - kappa_upper;
    values(4) = -a + a_lower;
    values(5) = a - a_upper;
    values(6) = -dkappa + dkappa_lower;
    values(7) = dkappa - dkappa_upper;
  }

  virtual void updateJacobian(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& jx,
                              Eigen::MatrixXd& ju) const override {
    // Evaluation of Matrix jx
    jx(0, 3) = -1;
    jx(1, 3) = 1;
    jx(2, 4) = -1;
    jx(3, 4) = 1;

    // Evaluation of Matrix ju
    ju(4, 0) = -1;
    ju(5, 0) = 1;
    ju(6, 1) = -1;
    ju(7, 1) = 1;
  }

  virtual void updateStateJacobian(const Eigen::VectorXd& states, Eigen::MatrixXd& jx) const override {
    // Evaluation of Matrix jx
    jx(0, 3) = -1;
    jx(1, 3) = 1;
    jx(2, 4) = -1;
    jx(3, 4) = 1;
  }

  virtual void updateCtrlJacobian(const Eigen::VectorXd& ctrls, Eigen::MatrixXd& ju) const override {
    // Evaluation of Matrix ju
    ju(0, 0) = -1;
    ju(1, 0) = 1;
    ju(2, 1) = -1;
    ju(3, 1) = 1;
  }

  virtual void updateJacobianDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                Eigen::MatrixXd& jx) const override {
    // Evaluation of Matrix jx
    jx(0, 3) = -1;
    jx(1, 3) = 1;
    jx(2, 4) = -1;
    jx(3, 4) = 1;
  }

  virtual void updateJacobianDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                Eigen::MatrixXd& ju) const override {
    // Evaluation of Matrix ju
    ju(4, 0) = -1;
    ju(5, 0) = 1;
    ju(6, 1) = -1;
    ju(7, 1) = 1;
  }

  virtual void updateDeltaEvaluation(const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::VectorXd& dx,
                                     const Eigen::VectorXd& du, Eigen::VectorXd& dg) const override {
    // Evaluation of Vector dg
    dg(0) = dx(3) * g_x(0, 3);
    dg(1) = dx(3) * g_x(1, 3);
    dg(2) = dx(4) * g_x(2, 4);
    dg(3) = dx(4) * g_x(3, 4);
    dg(4) = du(0) * g_u(4, 0);
    dg(5) = du(0) * g_u(5, 0);
    dg(6) = du(1) * g_u(6, 1);
    dg(7) = du(1) * g_u(7, 1);
  }
};

template <>
class BicycleTrajectoryTrackerConstraints<StageType::INITIAL> : public Constraints {
 public:
  BicycleTrajectoryTrackerConstraints() : Constraints(5, 2, 0, 4, 0) {
    // Evaluation of Matrix jacobian_.ju
    jacobian_.ju(0, 0) = -1;
    jacobian_.ju(1, 0) = 1;
    jacobian_.ju(2, 1) = -1;
    jacobian_.ju(3, 1) = 1;
    exprs_[0] = "a_lower <= a";
    exprs_[1] = "a <= a_upper";
    exprs_[2] = "dkappa_lower <= dkappa";
    exprs_[3] = "dkappa <= dkappa_upper";
  }

  virtual ~BicycleTrajectoryTrackerConstraints() = default;

  virtual Type type() const override { return Type::INEQUALITY; }
  virtual void updateEvaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                              Eigen::VectorXd& values) const override {
    // Determine stage variables
    const auto& a = ctrls(0);
    const auto& dkappa = ctrls(1);
    const auto& a_lower = params(13);
    const auto& a_upper = params(14);
    const auto& dkappa_lower = params(17);
    const auto& dkappa_upper = params(18);

    // Evaluation of Vector values
    values(0) = -a + a_lower;
    values(1) = a - a_upper;
    values(2) = -dkappa + dkappa_lower;
    values(3) = dkappa - dkappa_upper;
  }

  virtual void updateJacobian(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& jx,
                              Eigen::MatrixXd& ju) const override {
    // Evaluation of Matrix ju
    ju(0, 0) = -1;
    ju(1, 0) = 1;
    ju(2, 1) = -1;
    ju(3, 1) = 1;
  }

  virtual void updateCtrlJacobian(const Eigen::VectorXd& ctrls, Eigen::MatrixXd& ju) const override {
    // Evaluation of Matrix ju
    ju(0, 0) = -1;
    ju(1, 0) = 1;
    ju(2, 1) = -1;
    ju(3, 1) = 1;
  }

  virtual void updateJacobianDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                Eigen::MatrixXd& ju) const override {
    // Evaluation of Matrix ju
    ju(0, 0) = -1;
    ju(1, 0) = 1;
    ju(2, 1) = -1;
    ju(3, 1) = 1;
  }

  virtual void updateDeltaEvaluation(const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::VectorXd& dx,
                                     const Eigen::VectorXd& du, Eigen::VectorXd& dg) const override {
    // Evaluation of Vector dg
    dg(0) = du(0) * g_u(0, 0);
    dg(1) = du(0) * g_u(1, 0);
    dg(2) = du(1) * g_u(2, 1);
    dg(3) = du(1) * g_u(3, 1);
  }
};

template <>
class BicycleTrajectoryTrackerConstraints<StageType::TERMINAL> : public Constraints {
 public:
  BicycleTrajectoryTrackerConstraints() : Constraints(5, 2, 4, 0, 0) {
    // Evaluation of Matrix jacobian_.jx
    jacobian_.jx(0, 3) = -1;
    jacobian_.jx(1, 3) = 1;
    jacobian_.jx(2, 4) = -1;
    jacobian_.jx(3, 4) = 1;

    exprs_[0] = "v_lower <= v";
    exprs_[1] = "v <= v_upper";
    exprs_[2] = "kappa_lower <= kappa";
    exprs_[3] = "kappa <= kappa_upper";
  }

  virtual ~BicycleTrajectoryTrackerConstraints() = default;

  virtual Type type() const override { return Type::INEQUALITY; }
  virtual void updateEvaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                              Eigen::VectorXd& values) const override {
    // Determine stage variables
    const auto& v = states(3);
    const auto& kappa = states(4);
    const auto& v_lower = params(11);
    const auto& v_upper = params(12);
    const auto& kappa_lower = params(15);
    const auto& kappa_upper = params(16);

    // Evaluation of Vector values
    values(0) = -v + v_lower;
    values(1) = v - v_upper;
    values(2) = -kappa + kappa_lower;
    values(3) = kappa - kappa_upper;
  }

  virtual void updateJacobian(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& jx,
                              Eigen::MatrixXd& ju) const override {
    // Evaluation of Matrix jx
    jx(0, 3) = -1;
    jx(1, 3) = 1;
    jx(2, 4) = -1;
    jx(3, 4) = 1;
  }

  virtual void updateStateJacobian(const Eigen::VectorXd& states, Eigen::MatrixXd& jx) const override {
    // Evaluation of Matrix jx
    jx(0, 3) = -1;
    jx(1, 3) = 1;
    jx(2, 4) = -1;
    jx(3, 4) = 1;
  }

  virtual void updateJacobianDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                Eigen::MatrixXd& jx) const override {
    // Evaluation of Matrix jx
    jx(0, 3) = -1;
    jx(1, 3) = 1;
    jx(2, 4) = -1;
    jx(3, 4) = 1;
  }

  virtual void updateDeltaEvaluation(const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::VectorXd& dx,
                                     const Eigen::VectorXd& du, Eigen::VectorXd& dg) const override {
    // Evaluation of Vector dg
    dg(0) = dx(3) * g_x(0, 3);
    dg(1) = dx(3) * g_x(1, 3);
    dg(2) = dx(4) * g_x(2, 4);
    dg(3) = dx(4) * g_x(3, 4);
  }
};

template <StageType Ttype>
class BicycleTrajectoryTrackerStateOnlyEqualities : public Constraints {
 public:
  BicycleTrajectoryTrackerStateOnlyEqualities() : Constraints(5, 2, 0, 0, 0) {}

  virtual ~BicycleTrajectoryTrackerStateOnlyEqualities() = default;

  virtual Type type() const override { return Type::STATE_ONLY_EQUALITY; }
};

template <>
class BicycleTrajectoryTrackerStateOnlyEqualities<StageType::INITIAL> : public Constraints {
 public:
  BicycleTrajectoryTrackerStateOnlyEqualities() : Constraints(5, 2, 5, 0, 0) {
    // Evaluation of SymmetricMatrix jacobian_.jx
    jacobian_.jx(0, 0) = 1;
    jacobian_.jx(1, 1) = 1;
    jacobian_.jx(2, 2) = 1;
    jacobian_.jx(3, 3) = 1;
    jacobian_.jx(4, 4) = 1;

    exprs_[0] = "x = 0";
    exprs_[1] = "y = 0";
    exprs_[2] = "theta = 0";
    exprs_[3] = "v = 0";
    exprs_[4] = "kappa = 0";
  }

  virtual ~BicycleTrajectoryTrackerStateOnlyEqualities() = default;

  virtual Type type() const override { return Type::STATE_ONLY_EQUALITY; }
  virtual void updateEvaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                              Eigen::VectorXd& values) const override {
    // Determine stage variables
    const auto& x = states(0);
    const auto& y = states(1);
    const auto& theta = states(2);
    const auto& v = states(3);
    const auto& kappa = states(4);

    // Evaluation of Vector values
    values(0) = x;
    values(1) = y;
    values(2) = theta;
    values(3) = v;
    values(4) = kappa;
  }

  virtual void updateJacobian(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& jx,
                              Eigen::MatrixXd& ju) const override {
    // Evaluation of Matrix jx
    jx(0, 0) = 1;
    jx(1, 1) = 1;
    jx(2, 2) = 1;
    jx(3, 3) = 1;
    jx(4, 4) = 1;
  }

  virtual void updateStateJacobian(const Eigen::VectorXd& states, Eigen::MatrixXd& jx) const override {
    // Evaluation of Matrix jx
    jx(0, 0) = 1;
    jx(1, 1) = 1;
    jx(2, 2) = 1;
    jx(3, 3) = 1;
    jx(4, 4) = 1;
  }

  virtual void updateJacobianDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                Eigen::MatrixXd& jx) const override {
    // Evaluation of Matrix jx
    jx(0, 0) = 1;
    jx(1, 1) = 1;
    jx(2, 2) = 1;
    jx(3, 3) = 1;
    jx(4, 4) = 1;
  }

  virtual void updateDeltaEvaluation(const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::VectorXd& dx,
                                     const Eigen::VectorXd& du, Eigen::VectorXd& dg) const override {
    // Evaluation of Vector dg
    dg(0) = dx(0) * g_x(0, 0);
    dg(1) = dx(1) * g_x(1, 1);
    dg(2) = dx(2) * g_x(2, 2);
    dg(3) = dx(3) * g_x(3, 3);
    dg(4) = dx(4) * g_x(4, 4);
  }
};

template <>
class BicycleTrajectoryTrackerStateOnlyEqualities<StageType::TERMINAL> : public Constraints {
 public:
  BicycleTrajectoryTrackerStateOnlyEqualities() : Constraints(5, 2, 0, 0, 0) {}

  virtual ~BicycleTrajectoryTrackerStateOnlyEqualities() = default;

  virtual Type type() const override { return Type::STATE_ONLY_EQUALITY; }
};

}  // namespace gpal::pnc::planning