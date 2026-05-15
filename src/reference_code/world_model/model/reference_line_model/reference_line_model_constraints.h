#pragma once

#include "ocp/constraints.h"

namespace gpal::pnc::planning {

template <StageType Ttype>
class ReferenceLineModelConstraints : public Constraints {
 public:
  ReferenceLineModelConstraints() : Constraints(4, 2, 3, 2, 2) {
    // Evaluation of Matrix jacobian_.jx
    jacobian_.jx(1, 3) = -1;
    jacobian_.jx(2, 3) = 1;

    // Evaluation of Matrix jacobian_.ju
    jacobian_.ju(3, 0) = 1;
    jacobian_.ju(4, 0) = -1;
    jacobian_.ju(5, 1) = -1;
    jacobian_.ju(6, 1) = -1;
    exprs_[0] = "cos(theta - thetar) >= 0.5";
    exprs_[1] = "kappa >= KappaLowerBound";
    exprs_[2] = "kappa <= KappaUpperBound";
    exprs_[3] = "dkappa <= DKappaUpperBound";
    exprs_[4] = "dkappa >= DKappaLowerBound";
    exprs_[5] = "sll <= slack_offset - (x - xr)*sin(thetar) + (y - yr)*cos(thetar)";
    exprs_[6] = "-slack_offset - (x - xr)*sin(thetar) + (y - yr)*cos(thetar) <= slu";
  }

  virtual ~ReferenceLineModelConstraints() = default;

  virtual Type type() const override { return Type::INEQUALITY; }
  virtual void updateEvaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                              Eigen::VectorXd& values) const override {
    // Determine stage variables
    const auto& kappa = states(3);
    const auto& dkappa = ctrls(0);
    const auto& slack_offset = ctrls(1);
    const auto& sll = params(10);
    const auto& slu = params(11);
    const auto& KappaLowerBound = params(12);
    const auto& KappaUpperBound = params(13);
    const auto& DKappaLowerBound = params(14);
    const auto& DKappaUpperBound = params(15);

    // Determine global variables
    const auto& theta_offset = globals(2);
    const auto& l = globals(3);

    // Evaluation of Vector values
    values(0) = 0.5 - theta_offset;
    values(1) = KappaLowerBound - kappa;
    values(2) = -KappaUpperBound + kappa;
    values(3) = -DKappaUpperBound + dkappa;
    values(4) = DKappaLowerBound - dkappa;
    values(5) = -l - slack_offset + sll;
    values(6) = l - slack_offset - slu;
  }

  virtual void updateJacobian(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& jx,
                              Eigen::MatrixXd& ju) const override {
    // Determine stage variables
    const auto& theta = states(2);
    const auto& thetar = params(2);

    // Determine global variables
    const auto& cos_thetar = globals(0);
    const auto& sin_thetar = globals(1);

    // Determine internal variables
    const double internal_0 = theta - thetar;
    const double sin_internal_0 = sin(internal_0);

    // Evaluation of Matrix jx
    jx(0, 2) = sin_internal_0;
    jx(1, 3) = -1;
    jx(2, 3) = 1;
    jx(5, 0) = sin_thetar;
    jx(5, 1) = -cos_thetar;
    jx(6, 0) = -sin_thetar;
    jx(6, 1) = cos_thetar;

    // Evaluation of Matrix ju
    ju(3, 0) = 1;
    ju(4, 0) = -1;
    ju(5, 1) = -1;
    ju(6, 1) = -1;
  }

  virtual void updateStateJacobian(const Eigen::VectorXd& states, Eigen::MatrixXd& jx) const override {
    // Determine stage variables
    const auto& theta = states(2);
    const auto& thetar = params(2);

    // Determine internal variables
    const double internal_0 = theta - thetar;
    const double sin_internal_0 = sin(internal_0);

    // Evaluation of Matrix jx
    jx(0, 2) = sin_internal_0;
    jx(1, 3) = -1;
    jx(2, 3) = 1;
  }

  virtual void updateCtrlJacobian(const Eigen::VectorXd& ctrls, Eigen::MatrixXd& ju) const override {
    // Evaluation of Matrix ju
    ju(0, 0) = 1;
    ju(1, 0) = -1;
  }

  virtual void updateJacobianDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                Eigen::MatrixXd& jx) const override {
    // Determine stage variables
    const auto& theta = states(2);
    const auto& thetar = params(2);

    // Determine global variables
    const auto& cos_thetar = globals(0);
    const auto& sin_thetar = globals(1);

    // Determine internal variables
    const double internal_0 = theta - thetar;
    const double sin_internal_0 = sin(internal_0);

    // Evaluation of Matrix jx
    jx(0, 2) = sin_internal_0;
    jx(1, 3) = -1;
    jx(2, 3) = 1;
    jx(5, 0) = sin_thetar;
    jx(5, 1) = -cos_thetar;
    jx(6, 0) = -sin_thetar;
    jx(6, 1) = cos_thetar;
  }

  virtual void updateJacobianDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                Eigen::MatrixXd& ju) const override {
    // Evaluation of Matrix ju
    ju(3, 0) = 1;
    ju(4, 0) = -1;
    ju(5, 1) = -1;
    ju(6, 1) = -1;
  }

  virtual void updateDeltaEvaluation(const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::VectorXd& dx,
                                     const Eigen::VectorXd& du, Eigen::VectorXd& dg) const override {
    // Evaluation of Vector dg
    dg(0) = dx(2) * g_x(0, 2);
    dg(1) = dx(3) * g_x(1, 3);
    dg(2) = dx(3) * g_x(2, 3);
    dg(3) = du(0) * g_u(3, 0);
    dg(4) = du(0) * g_u(4, 0);
    dg(5) = du(1) * g_u(5, 1) + dx(0) * g_x(5, 0) + dx(1) * g_x(5, 1);
    dg(6) = du(1) * g_u(6, 1) + dx(0) * g_x(6, 0) + dx(1) * g_x(6, 1);
  }
};

template <>
class ReferenceLineModelConstraints<StageType::INITIAL> : public Constraints {
 public:
  ReferenceLineModelConstraints() : Constraints(4, 2, 0, 2, 0) {
    // Evaluation of Matrix jacobian_.ju
    jacobian_.ju(0, 0) = 1;
    jacobian_.ju(1, 0) = -1;
    exprs_[0] = "dkappa <= DKappaUpperBound";
    exprs_[1] = "dkappa >= DKappaLowerBound";
  }

  virtual ~ReferenceLineModelConstraints() = default;

  virtual Type type() const override { return Type::INEQUALITY; }
  virtual void updateEvaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                              Eigen::VectorXd& values) const override {
    // Determine stage variables
    const auto& dkappa = ctrls(0);
    const auto& DKappaLowerBound = params(14);
    const auto& DKappaUpperBound = params(15);

    // Evaluation of Vector values
    values(0) = -DKappaUpperBound + dkappa;
    values(1) = DKappaLowerBound - dkappa;
  }

  virtual void updateJacobian(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& jx,
                              Eigen::MatrixXd& ju) const override {
    // Evaluation of Matrix ju
    ju(0, 0) = 1;
    ju(1, 0) = -1;
  }

  virtual void updateCtrlJacobian(const Eigen::VectorXd& ctrls, Eigen::MatrixXd& ju) const override {
    // Evaluation of Matrix ju
    ju(0, 0) = 1;
    ju(1, 0) = -1;
  }

  virtual void updateJacobianDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                Eigen::MatrixXd& ju) const override {
    // Evaluation of Matrix ju
    ju(0, 0) = 1;
    ju(1, 0) = -1;
  }

  virtual void updateDeltaEvaluation(const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::VectorXd& dx,
                                     const Eigen::VectorXd& du, Eigen::VectorXd& dg) const override {
    // Evaluation of Vector dg
    dg(0) = du(0) * g_u(0, 0);
    dg(1) = du(0) * g_u(1, 0);
  }
};

template <>
class ReferenceLineModelConstraints<StageType::TERMINAL> : public Constraints {
 public:
  ReferenceLineModelConstraints() : Constraints(4, 2, 3, 0, 0) {
    // Evaluation of Matrix jacobian_.jx
    jacobian_.jx(1, 3) = -1;
    jacobian_.jx(2, 3) = 1;

    exprs_[0] = "cos(theta - thetar) >= 0.5";
    exprs_[1] = "kappa >= KappaLowerBound";
    exprs_[2] = "kappa <= KappaUpperBound";
  }

  virtual ~ReferenceLineModelConstraints() = default;

  virtual Type type() const override { return Type::INEQUALITY; }
  virtual void updateEvaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                              Eigen::VectorXd& values) const override {
    // Determine stage variables
    const auto& kappa = states(3);
    const auto& KappaLowerBound = params(12);
    const auto& KappaUpperBound = params(13);

    // Determine global variables
    const auto& theta_offset = globals(2);

    // Evaluation of Vector values
    values(0) = 0.5 - theta_offset;
    values(1) = KappaLowerBound - kappa;
    values(2) = -KappaUpperBound + kappa;
  }

  virtual void updateJacobian(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& jx,
                              Eigen::MatrixXd& ju) const override {
    // Determine stage variables
    const auto& theta = states(2);
    const auto& thetar = params(2);

    // Determine internal variables
    const double internal_0 = theta - thetar;
    const double sin_internal_0 = sin(internal_0);

    // Evaluation of Matrix jx
    jx(0, 2) = sin_internal_0;
    jx(1, 3) = -1;
    jx(2, 3) = 1;
  }

  virtual void updateStateJacobian(const Eigen::VectorXd& states, Eigen::MatrixXd& jx) const override {
    // Determine stage variables
    const auto& theta = states(2);
    const auto& thetar = params(2);

    // Determine internal variables
    const double internal_0 = theta - thetar;
    const double sin_internal_0 = sin(internal_0);

    // Evaluation of Matrix jx
    jx(0, 2) = sin_internal_0;
    jx(1, 3) = -1;
    jx(2, 3) = 1;
  }

  virtual void updateJacobianDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                Eigen::MatrixXd& jx) const override {
    // Determine stage variables
    const auto& theta = states(2);
    const auto& thetar = params(2);

    // Determine internal variables
    const double internal_0 = theta - thetar;
    const double sin_internal_0 = sin(internal_0);

    // Evaluation of Matrix jx
    jx(0, 2) = sin_internal_0;
    jx(1, 3) = -1;
    jx(2, 3) = 1;
  }

  virtual void updateDeltaEvaluation(const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::VectorXd& dx,
                                     const Eigen::VectorXd& du, Eigen::VectorXd& dg) const override {
    // Evaluation of Vector dg
    dg(0) = dx(2) * g_x(0, 2);
    dg(1) = dx(3) * g_x(1, 3);
    dg(2) = dx(3) * g_x(2, 3);
  }
};

template <StageType Ttype>
class ReferenceLineModelStateOnlyEqualities : public Constraints {
 public:
  ReferenceLineModelStateOnlyEqualities() : Constraints(4, 2, 0, 0, 0) {}

  virtual ~ReferenceLineModelStateOnlyEqualities() = default;

  virtual Type type() const override { return Type::STATE_ONLY_EQUALITY; }
};

template <>
class ReferenceLineModelStateOnlyEqualities<StageType::INITIAL> : public Constraints {
 public:
  ReferenceLineModelStateOnlyEqualities() : Constraints(4, 2, 4, 0, 0) {
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

  virtual ~ReferenceLineModelStateOnlyEqualities() = default;

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
class ReferenceLineModelStateOnlyEqualities<StageType::TERMINAL> : public Constraints {
 public:
  ReferenceLineModelStateOnlyEqualities() : Constraints(4, 2, 0, 0, 0) {}

  virtual ~ReferenceLineModelStateOnlyEqualities() = default;

  virtual Type type() const override { return Type::STATE_ONLY_EQUALITY; }
};

}  // namespace gpal::pnc::planning