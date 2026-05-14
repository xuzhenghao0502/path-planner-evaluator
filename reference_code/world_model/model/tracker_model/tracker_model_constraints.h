#pragma once

#include "ocp/constraints.h"

namespace gpal::pnc::planning {

template <StageType Ttype>
class TrackerModelConstraints : public Constraints {
 public:
  TrackerModelConstraints() : Constraints(5, 2, 3, 2, 2) {
    // Evaluation of Matrix jacobian_.jx
    jacobian_.jx(0, 4) = -1;
    jacobian_.jx(1, 4) = 1;

    // Evaluation of Matrix jacobian_.ju
    jacobian_.ju(3, 0) = -1;
    jacobian_.ju(4, 0) = 1;
    jacobian_.ju(5, 1) = -1;
    jacobian_.ju(6, 1) = -1;
    exprs_[0] = "SteerLowerBound <= steer";
    exprs_[1] = "steer <= SteerUpperBound";
    exprs_[2] = "cos(theta - theta_ref) >= 0.0";
    exprs_[3] = "DSteerLowerBound <= dsteer";
    exprs_[4] = "dsteer <= DSteerUpperBound";
    exprs_[5] = "LHardLowerBound <= l_slack - (x - x_ref)*sin(theta_ref) + (y - y_ref)*cos(theta_ref)";
    exprs_[6] = "-l_slack - (x - x_ref)*sin(theta_ref) + (y - y_ref)*cos(theta_ref) <= LHardUpperBound";

    idx_with_slack_.emplace(5);
    idx_with_slack_.emplace(6);
  }

  virtual ~TrackerModelConstraints() = default;

  virtual Type type() const override { return Type::INEQUALITY; }
  virtual void updateEvaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::VectorXd& values) const override {
    // Determine stage variables
    const auto& theta = states(3);
    const auto& steer = states(4);
    const auto& dsteer = ctrls(0);
    const auto& l_slack = ctrls(1);
    const auto& theta_ref = params(2);
    const auto& LHardLowerBound = params(18);
    const auto& LHardUpperBound = params(19);
    const auto& SteerLowerBound = params(20);
    const auto& SteerUpperBound = params(21);
    const auto& DSteerLowerBound = params(22);
    const auto& DSteerUpperBound = params(23);

    // Determine global variables
    const auto& l = globals(1);

    // Determine internal variables
    const double internal_0 = -steer;
    const double internal_1 = -dsteer;

    // Evaluation of Vector values
    values(0) = SteerLowerBound + internal_0;
    values(1) = -SteerUpperBound - internal_0;
    values(2) = -cos(theta - theta_ref);
    values(3) = DSteerLowerBound + internal_1;
    values(4) = -DSteerUpperBound - internal_1;
    values(5) = LHardLowerBound - l - l_slack;
    values(6) = -LHardUpperBound + l - l_slack;

  }

  virtual void updateJacobian(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& jx, Eigen::MatrixXd& ju) const override {
    // Determine stage variables
    const auto& theta = states(3);
    const auto& theta_ref = params(2);

    // Determine internal variables
    const double internal_0 = sin(theta_ref);
    const double internal_1 = cos(theta_ref);

    // Evaluation of Matrix jx
    jx(0, 4) = -1;
    jx(1, 4) = 1;
    jx(2, 3) = sin(theta - theta_ref);
    jx(5, 1) = internal_0;
    jx(5, 2) = -internal_1;
    jx(6, 1) = -internal_0;
    jx(6, 2) = internal_1;

    // Evaluation of Matrix ju
    ju(3, 0) = -1;
    ju(4, 0) = 1;
    ju(5, 1) = -1;
    ju(6, 1) = -1;

  }

  virtual void updateStateJacobian(const Eigen::VectorXd& states, Eigen::MatrixXd& jx) const override {
    // Determine stage variables
    const auto& theta = states(3);
    const auto& theta_ref = params(2);

    // Evaluation of Matrix jx
    jx(0, 4) = -1;
    jx(1, 4) = 1;
    jx(2, 3) = sin(theta - theta_ref);

  }

  virtual void updateCtrlJacobian(const Eigen::VectorXd& ctrls, Eigen::MatrixXd& ju) const override {
    // Evaluation of Matrix ju
    ju(0, 0) = -1;
    ju(1, 0) = 1;

  }

  virtual void updateJacobianDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& jx) const override {
    // Determine stage variables
    const auto& theta = states(3);
    const auto& theta_ref = params(2);

    // Determine internal variables
    const double internal_0 = sin(theta_ref);
    const double internal_1 = cos(theta_ref);

    // Evaluation of Matrix jx
    jx(0, 4) = -1;
    jx(1, 4) = 1;
    jx(2, 3) = sin(theta - theta_ref);
    jx(5, 1) = internal_0;
    jx(5, 2) = -internal_1;
    jx(6, 1) = -internal_0;
    jx(6, 2) = internal_1;

  }

  virtual void updateJacobianDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& ju) const override {
    // Evaluation of Matrix ju
    ju(3, 0) = -1;
    ju(4, 0) = 1;
    ju(5, 1) = -1;
    ju(6, 1) = -1;

  }

  virtual void updateDeltaEvaluation(const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::VectorXd& dx, const Eigen::VectorXd& du, Eigen::VectorXd& dg) const override {
    // Evaluation of Vector dg
    dg(0) = dx(4)*g_x(0, 4);
    dg(1) = dx(4)*g_x(1, 4);
    dg(2) = dx(3)*g_x(2, 3);
    dg(3) = du(0)*g_u(3, 0);
    dg(4) = du(0)*g_u(4, 0);
    dg(5) = du(1)*g_u(5, 1) + dx(1)*g_x(5, 1) + dx(2)*g_x(5, 2);
    dg(6) = du(1)*g_u(6, 1) + dx(1)*g_x(6, 1) + dx(2)*g_x(6, 2);

  }

};

template <>
class TrackerModelConstraints<StageType::INITIAL> : public Constraints {
 public:
  TrackerModelConstraints() : Constraints(5, 2, 0, 2, 0) {
    // Evaluation of Matrix jacobian_.ju
    jacobian_.ju(0, 0) = -1;
    jacobian_.ju(1, 0) = 1;
    exprs_[0] = "DSteerLowerBound <= dsteer";
    exprs_[1] = "dsteer <= DSteerUpperBound";

  }

  virtual ~TrackerModelConstraints() = default;

  virtual Type type() const override { return Type::INEQUALITY; }
  virtual void updateEvaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::VectorXd& values) const override {
    // Determine stage variables
    const auto& dsteer = ctrls(0);
    const auto& DSteerLowerBound = params(22);
    const auto& DSteerUpperBound = params(23);

    // Determine internal variables
    const double internal_0 = -dsteer;

    // Evaluation of Vector values
    values(0) = DSteerLowerBound + internal_0;
    values(1) = -DSteerUpperBound - internal_0;

  }

  virtual void updateJacobian(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& jx, Eigen::MatrixXd& ju) const override {
    // Evaluation of Matrix ju
    ju(0, 0) = -1;
    ju(1, 0) = 1;

  }

  virtual void updateCtrlJacobian(const Eigen::VectorXd& ctrls, Eigen::MatrixXd& ju) const override {
    // Evaluation of Matrix ju
    ju(0, 0) = -1;
    ju(1, 0) = 1;

  }

  virtual void updateJacobianDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& ju) const override {
    // Evaluation of Matrix ju
    ju(0, 0) = -1;
    ju(1, 0) = 1;

  }

  virtual void updateDeltaEvaluation(const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::VectorXd& dx, const Eigen::VectorXd& du, Eigen::VectorXd& dg) const override {
    // Evaluation of Vector dg
    dg(0) = du(0)*g_u(0, 0);
    dg(1) = du(0)*g_u(1, 0);

  }

};

template <>
class TrackerModelConstraints<StageType::TERMINAL> : public Constraints {
 public:
  TrackerModelConstraints() : Constraints(5, 2, 3, 0, 0) {
    // Evaluation of Matrix jacobian_.jx
    jacobian_.jx(0, 4) = -1;
    jacobian_.jx(1, 4) = 1;

    exprs_[0] = "SteerLowerBound <= steer";
    exprs_[1] = "steer <= SteerUpperBound";
    exprs_[2] = "cos(theta - theta_ref) >= 0.0";

  }

  virtual ~TrackerModelConstraints() = default;

  virtual Type type() const override { return Type::INEQUALITY; }
  virtual void updateEvaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::VectorXd& values) const override {
    // Determine stage variables
    const auto& theta = states(3);
    const auto& steer = states(4);
    const auto& theta_ref = params(2);
    const auto& SteerLowerBound = params(20);
    const auto& SteerUpperBound = params(21);

    // Determine internal variables
    const double internal_0 = -steer;

    // Evaluation of Vector values
    values(0) = SteerLowerBound + internal_0;
    values(1) = -SteerUpperBound - internal_0;
    values(2) = -cos(theta - theta_ref);

  }

  virtual void updateJacobian(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& jx, Eigen::MatrixXd& ju) const override {
    // Determine stage variables
    const auto& theta = states(3);
    const auto& theta_ref = params(2);

    // Evaluation of Matrix jx
    jx(0, 4) = -1;
    jx(1, 4) = 1;
    jx(2, 3) = sin(theta - theta_ref);


  }

  virtual void updateStateJacobian(const Eigen::VectorXd& states, Eigen::MatrixXd& jx) const override {
    // Determine stage variables
    const auto& theta = states(3);
    const auto& theta_ref = params(2);

    // Evaluation of Matrix jx
    jx(0, 4) = -1;
    jx(1, 4) = 1;
    jx(2, 3) = sin(theta - theta_ref);

  }

  virtual void updateJacobianDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& jx) const override {
    // Determine stage variables
    const auto& theta = states(3);
    const auto& theta_ref = params(2);

    // Evaluation of Matrix jx
    jx(0, 4) = -1;
    jx(1, 4) = 1;
    jx(2, 3) = sin(theta - theta_ref);

  }

  virtual void updateDeltaEvaluation(const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::VectorXd& dx, const Eigen::VectorXd& du, Eigen::VectorXd& dg) const override {
    // Evaluation of Vector dg
    dg(0) = dx(4)*g_x(0, 4);
    dg(1) = dx(4)*g_x(1, 4);
    dg(2) = dx(3)*g_x(2, 3);

  }

};

template <StageType Ttype>
class TrackerModelStateOnlyEqualities : public Constraints {
 public:
  TrackerModelStateOnlyEqualities() : Constraints(5, 2, 0, 0, 0) {
  }

  virtual ~TrackerModelStateOnlyEqualities() = default;

  virtual Type type() const override { return Type::STATE_ONLY_EQUALITY; }
};

template <>
class TrackerModelStateOnlyEqualities<StageType::INITIAL> : public Constraints {
 public:
  TrackerModelStateOnlyEqualities() : Constraints(5, 2, 5, 0, 0) {
    // Evaluation of SymmetricMatrix jacobian_.jx
    jacobian_.jx(0, 0) = 1;
    jacobian_.jx(1, 1) = 1;
    jacobian_.jx(2, 2) = 1;
    jacobian_.jx(3, 3) = 1;
    jacobian_.jx(4, 4) = 1;

    exprs_[0] = "s = 0";
    exprs_[1] = "x = 0";
    exprs_[2] = "y = 0";
    exprs_[3] = "theta = 0";
    exprs_[4] = "steer = 0";
  }

  virtual ~TrackerModelStateOnlyEqualities() = default;

  virtual Type type() const override { return Type::STATE_ONLY_EQUALITY; }
  virtual void updateEvaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::VectorXd& values) const override {
    // Determine stage variables
    const auto& s = states(0);
    const auto& x = states(1);
    const auto& y = states(2);
    const auto& theta = states(3);
    const auto& steer = states(4);

    // Evaluation of Vector values
    values(0) = s;
    values(1) = x;
    values(2) = y;
    values(3) = theta;
    values(4) = steer;

  }

  virtual void updateJacobian(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& jx, Eigen::MatrixXd& ju) const override {
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

  virtual void updateJacobianDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& jx) const override {
    // Evaluation of Matrix jx
    jx(0, 0) = 1;
    jx(1, 1) = 1;
    jx(2, 2) = 1;
    jx(3, 3) = 1;
    jx(4, 4) = 1;

  }

  virtual void updateDeltaEvaluation(const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::VectorXd& dx, const Eigen::VectorXd& du, Eigen::VectorXd& dg) const override {
    // Evaluation of Vector dg
    dg(0) = dx(0)*g_x(0, 0);
    dg(1) = dx(1)*g_x(1, 1);
    dg(2) = dx(2)*g_x(2, 2);
    dg(3) = dx(3)*g_x(3, 3);
    dg(4) = dx(4)*g_x(4, 4);

  }

};

template <>
class TrackerModelStateOnlyEqualities<StageType::TERMINAL> : public Constraints {
 public:
  TrackerModelStateOnlyEqualities() : Constraints(5, 2, 0, 0, 0) {
  }

  virtual ~TrackerModelStateOnlyEqualities() = default;

  virtual Type type() const override { return Type::STATE_ONLY_EQUALITY; }
};

}  // namespace gpal::pnc::planning