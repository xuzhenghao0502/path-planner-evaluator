#pragma once

#include "ocp/constraints.h"

namespace gpal::pnc::planning {

template <StageType Ttype>
class SpeedOCPModelConstraints : public Constraints {
 public:
  SpeedOCPModelConstraints() : Constraints(3, 8, 6, 2, 7) {
    // Evaluation of Matrix jacobian_.jx
    jacobian_.jx(0, 0) = -1;
    jacobian_.jx(1, 0) = 1;
    jacobian_.jx(2, 1) = -1;
    jacobian_.jx(3, 1) = 1;
    jacobian_.jx(4, 2) = -1;
    jacobian_.jx(5, 2) = 1;
    jacobian_.jx(8, 0) = 1;
    jacobian_.jx(9, 0) = -1;
    jacobian_.jx(10, 1) = 1;
    jacobian_.jx(11, 1) = -1;
    jacobian_.jx(12, 2) = 1;
    jacobian_.jx(13, 2) = -1;
    jacobian_.jx(14, 0) = 1;

    // Evaluation of Matrix jacobian_.ju
    jacobian_.ju(6, 0) = -1;
    jacobian_.ju(7, 0) = 1;
    jacobian_.ju(8, 1) = -1;
    jacobian_.ju(9, 2) = -1;
    jacobian_.ju(10, 3) = -1;
    jacobian_.ju(11, 4) = -1;
    jacobian_.ju(12, 5) = -1;
    jacobian_.ju(13, 6) = -1;
    jacobian_.ju(14, 7) = -1;
    exprs_[0] = "SHardLowerBound <= s";
    exprs_[1] = "s <= SHardUpperBound";
    exprs_[2] = "VHardLowerBound <= v";
    exprs_[3] = "v <= VHardUpperBound";
    exprs_[4] = "AHardLowerBound <= a";
    exprs_[5] = "a <= AHardUpperBound";
    exprs_[6] = "JHardLowerBound <= j";
    exprs_[7] = "j <= JHardUpperBound";
    exprs_[8] = "s - slack_s_upper <= SSoftUpperBound";
    exprs_[9] = "SSoftLowerBound <= s + slack_s_lower";
    exprs_[10] = "-slack_v_upper + v <= VSoftUpperBound";
    exprs_[11] = "VSoftLowerBound <= slack_v_lower + v";
    exprs_[12] = "a - slack_a_upper <= ASoftUpperBound";
    exprs_[13] = "ASoftLowerBound <= a + slack_a_lower";
    exprs_[14] = "SafeDistForDVConstraint + k*v <= SUpperBoundForDVConstraint - s + slack_d_v";

    idx_with_slack_.emplace(8);
    idx_with_slack_.emplace(9);
    idx_with_slack_.emplace(10);
    idx_with_slack_.emplace(11);
    idx_with_slack_.emplace(12);
    idx_with_slack_.emplace(13);
    idx_with_slack_.emplace(14);
  }

  virtual ~SpeedOCPModelConstraints() = default;

  virtual Type type() const override { return Type::INEQUALITY; }
  virtual void updateEvaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                              Eigen::VectorXd& values) const override {
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
    const auto& SHardUpperBound = params(14);
    const auto& SHardLowerBound = params(15);
    const auto& SSoftUpperBound = params(16);
    const auto& SSoftLowerBound = params(17);
    const auto& SUpperBoundForDVConstraint = params(18);
    const auto& VHardUpperBound = params(19);
    const auto& VHardLowerBound = params(20);
    const auto& VSoftUpperBound = params(21);
    const auto& VSoftLowerBound = params(22);
    const auto& AHardUpperBound = params(23);
    const auto& AHardLowerBound = params(24);
    const auto& ASoftUpperBound = params(25);
    const auto& ASoftLowerBound = params(26);
    const auto& JHardUpperBound = params(27);
    const auto& JHardLowerBound = params(28);
    const auto& SafeDistForDVConstraint = params(29);
    const auto& k = params(30);

    // Evaluation of Vector values
    values(0) = SHardLowerBound - s;
    values(1) = -SHardUpperBound + s;
    values(2) = VHardLowerBound - v;
    values(3) = -VHardUpperBound + v;
    values(4) = AHardLowerBound - a;
    values(5) = -AHardUpperBound + a;
    values(6) = JHardLowerBound - j;
    values(7) = -JHardUpperBound + j;
    values(8) = -SSoftUpperBound + s - slack_s_upper;
    values(9) = SSoftLowerBound - s - slack_s_lower;
    values(10) = -VSoftUpperBound - slack_v_upper + v;
    values(11) = VSoftLowerBound - slack_v_lower - v;
    values(12) = -ASoftUpperBound + a - slack_a_upper;
    values(13) = ASoftLowerBound - a - slack_a_lower;
    values(14) = -SUpperBoundForDVConstraint + SafeDistForDVConstraint + k * v + s - slack_d_v;
  }

  virtual void updateJacobian(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& jx,
                              Eigen::MatrixXd& ju) const override {
    // Determine stage variables
    const auto& k = params(30);

    // Evaluation of Matrix jx
    jx(0, 0) = -1;
    jx(1, 0) = 1;
    jx(2, 1) = -1;
    jx(3, 1) = 1;
    jx(4, 2) = -1;
    jx(5, 2) = 1;
    jx(8, 0) = 1;
    jx(9, 0) = -1;
    jx(10, 1) = 1;
    jx(11, 1) = -1;
    jx(12, 2) = 1;
    jx(13, 2) = -1;
    jx(14, 0) = 1;
    jx(14, 1) = k;

    // Evaluation of Matrix ju
    ju(6, 0) = -1;
    ju(7, 0) = 1;
    ju(8, 1) = -1;
    ju(9, 2) = -1;
    ju(10, 3) = -1;
    ju(11, 4) = -1;
    ju(12, 5) = -1;
    ju(13, 6) = -1;
    ju(14, 7) = -1;
  }

  virtual void updateStateJacobian(const Eigen::VectorXd& states, Eigen::MatrixXd& jx) const override {
    // Evaluation of Matrix jx
    jx(0, 0) = -1;
    jx(1, 0) = 1;
    jx(2, 1) = -1;
    jx(3, 1) = 1;
    jx(4, 2) = -1;
    jx(5, 2) = 1;
  }

  virtual void updateCtrlJacobian(const Eigen::VectorXd& ctrls, Eigen::MatrixXd& ju) const override {
    // Evaluation of Matrix ju
    ju(0, 0) = -1;
    ju(1, 0) = 1;
  }

  virtual void updateJacobianDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                Eigen::MatrixXd& jx) const override {
    // Determine stage variables
    const auto& k = params(30);

    // Evaluation of Matrix jx
    jx(0, 0) = -1;
    jx(1, 0) = 1;
    jx(2, 1) = -1;
    jx(3, 1) = 1;
    jx(4, 2) = -1;
    jx(5, 2) = 1;
    jx(8, 0) = 1;
    jx(9, 0) = -1;
    jx(10, 1) = 1;
    jx(11, 1) = -1;
    jx(12, 2) = 1;
    jx(13, 2) = -1;
    jx(14, 0) = 1;
    jx(14, 1) = k;
  }

  virtual void updateJacobianDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                Eigen::MatrixXd& ju) const override {
    // Evaluation of Matrix ju
    ju(6, 0) = -1;
    ju(7, 0) = 1;
    ju(8, 1) = -1;
    ju(9, 2) = -1;
    ju(10, 3) = -1;
    ju(11, 4) = -1;
    ju(12, 5) = -1;
    ju(13, 6) = -1;
    ju(14, 7) = -1;
  }

  virtual void updateDeltaEvaluation(const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::VectorXd& dx,
                                     const Eigen::VectorXd& du, Eigen::VectorXd& dg) const override {
    // Evaluation of Vector dg
    dg(0) = dx(0) * g_x(0, 0);
    dg(1) = dx(0) * g_x(1, 0);
    dg(2) = dx(1) * g_x(2, 1);
    dg(3) = dx(1) * g_x(3, 1);
    dg(4) = dx(2) * g_x(4, 2);
    dg(5) = dx(2) * g_x(5, 2);
    dg(6) = du(0) * g_u(6, 0);
    dg(7) = du(0) * g_u(7, 0);
    dg(8) = du(1) * g_u(8, 1) + dx(0) * g_x(8, 0);
    dg(9) = du(2) * g_u(9, 2) + dx(0) * g_x(9, 0);
    dg(10) = du(3) * g_u(10, 3) + dx(1) * g_x(10, 1);
    dg(11) = du(4) * g_u(11, 4) + dx(1) * g_x(11, 1);
    dg(12) = du(5) * g_u(12, 5) + dx(2) * g_x(12, 2);
    dg(13) = du(6) * g_u(13, 6) + dx(2) * g_x(13, 2);
    dg(14) = du(7) * g_u(14, 7) + dx(0) * g_x(14, 0) + dx(1) * g_x(14, 1);
  }
};

template <>
class SpeedOCPModelConstraints<StageType::INITIAL> : public Constraints {
 public:
  SpeedOCPModelConstraints() : Constraints(3, 8, 0, 2, 0) {
    // Evaluation of Matrix jacobian_.ju
    jacobian_.ju(0, 0) = -1;
    jacobian_.ju(1, 0) = 1;
    exprs_[0] = "JHardLowerBound <= j";
    exprs_[1] = "j <= JHardUpperBound";
  }

  virtual ~SpeedOCPModelConstraints() = default;

  virtual Type type() const override { return Type::INEQUALITY; }
  virtual void updateEvaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                              Eigen::VectorXd& values) const override {
    // Determine stage variables
    const auto& j = ctrls(0);
    const auto& JHardUpperBound = params(27);
    const auto& JHardLowerBound = params(28);

    // Evaluation of Vector values
    values(0) = JHardLowerBound - j;
    values(1) = -JHardUpperBound + j;
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
class SpeedOCPModelConstraints<StageType::TERMINAL> : public Constraints {
 public:
  SpeedOCPModelConstraints() : Constraints(3, 8, 6, 0, 0) {
    // Evaluation of Matrix jacobian_.jx
    jacobian_.jx(0, 0) = -1;
    jacobian_.jx(1, 0) = 1;
    jacobian_.jx(2, 1) = -1;
    jacobian_.jx(3, 1) = 1;
    jacobian_.jx(4, 2) = -1;
    jacobian_.jx(5, 2) = 1;

    exprs_[0] = "SHardLowerBound <= s";
    exprs_[1] = "s <= SHardUpperBound";
    exprs_[2] = "VHardLowerBound <= v";
    exprs_[3] = "v <= VHardUpperBound";
    exprs_[4] = "AHardLowerBound <= a";
    exprs_[5] = "a <= AHardUpperBound";
  }

  virtual ~SpeedOCPModelConstraints() = default;

  virtual Type type() const override { return Type::INEQUALITY; }
  virtual void updateEvaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                              Eigen::VectorXd& values) const override {
    // Determine stage variables
    const auto& s = states(0);
    const auto& v = states(1);
    const auto& a = states(2);
    const auto& SHardUpperBound = params(14);
    const auto& SHardLowerBound = params(15);
    const auto& VHardUpperBound = params(19);
    const auto& VHardLowerBound = params(20);
    const auto& AHardUpperBound = params(23);
    const auto& AHardLowerBound = params(24);

    // Evaluation of Vector values
    values(0) = SHardLowerBound - s;
    values(1) = -SHardUpperBound + s;
    values(2) = VHardLowerBound - v;
    values(3) = -VHardUpperBound + v;
    values(4) = AHardLowerBound - a;
    values(5) = -AHardUpperBound + a;
  }

  virtual void updateJacobian(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& jx,
                              Eigen::MatrixXd& ju) const override {
    // Evaluation of Matrix jx
    jx(0, 0) = -1;
    jx(1, 0) = 1;
    jx(2, 1) = -1;
    jx(3, 1) = 1;
    jx(4, 2) = -1;
    jx(5, 2) = 1;
  }

  virtual void updateStateJacobian(const Eigen::VectorXd& states, Eigen::MatrixXd& jx) const override {
    // Evaluation of Matrix jx
    jx(0, 0) = -1;
    jx(1, 0) = 1;
    jx(2, 1) = -1;
    jx(3, 1) = 1;
    jx(4, 2) = -1;
    jx(5, 2) = 1;
  }

  virtual void updateJacobianDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                Eigen::MatrixXd& jx) const override {
    // Evaluation of Matrix jx
    jx(0, 0) = -1;
    jx(1, 0) = 1;
    jx(2, 1) = -1;
    jx(3, 1) = 1;
    jx(4, 2) = -1;
    jx(5, 2) = 1;
  }

  virtual void updateDeltaEvaluation(const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::VectorXd& dx,
                                     const Eigen::VectorXd& du, Eigen::VectorXd& dg) const override {
    // Evaluation of Vector dg
    dg(0) = dx(0) * g_x(0, 0);
    dg(1) = dx(0) * g_x(1, 0);
    dg(2) = dx(1) * g_x(2, 1);
    dg(3) = dx(1) * g_x(3, 1);
    dg(4) = dx(2) * g_x(4, 2);
    dg(5) = dx(2) * g_x(5, 2);
  }
};

template <StageType Ttype>
class SpeedOCPModelStateOnlyEqualities : public Constraints {
 public:
  SpeedOCPModelStateOnlyEqualities() : Constraints(3, 8, 0, 0, 0) {}

  virtual ~SpeedOCPModelStateOnlyEqualities() = default;

  virtual Type type() const override { return Type::STATE_ONLY_EQUALITY; }
};

template <>
class SpeedOCPModelStateOnlyEqualities<StageType::INITIAL> : public Constraints {
 public:
  SpeedOCPModelStateOnlyEqualities() : Constraints(3, 8, 3, 0, 0) {
    // Evaluation of SymmetricMatrix jacobian_.jx
    jacobian_.jx(0, 0) = 1;
    jacobian_.jx(1, 1) = 1;
    jacobian_.jx(2, 2) = 1;

    exprs_[0] = "s = 0";
    exprs_[1] = "v = 0";
    exprs_[2] = "a = 0";
  }

  virtual ~SpeedOCPModelStateOnlyEqualities() = default;

  virtual Type type() const override { return Type::STATE_ONLY_EQUALITY; }
  virtual void updateEvaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                              Eigen::VectorXd& values) const override {
    // Determine stage variables
    const auto& s = states(0);
    const auto& v = states(1);
    const auto& a = states(2);

    // Evaluation of Vector values
    values(0) = s;
    values(1) = v;
    values(2) = a;
  }

  virtual void updateJacobian(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& jx,
                              Eigen::MatrixXd& ju) const override {
    // Evaluation of Matrix jx
    jx(0, 0) = 1;
    jx(1, 1) = 1;
    jx(2, 2) = 1;
  }

  virtual void updateStateJacobian(const Eigen::VectorXd& states, Eigen::MatrixXd& jx) const override {
    // Evaluation of Matrix jx
    jx(0, 0) = 1;
    jx(1, 1) = 1;
    jx(2, 2) = 1;
  }

  virtual void updateJacobianDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                Eigen::MatrixXd& jx) const override {
    // Evaluation of Matrix jx
    jx(0, 0) = 1;
    jx(1, 1) = 1;
    jx(2, 2) = 1;
  }

  virtual void updateDeltaEvaluation(const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::VectorXd& dx,
                                     const Eigen::VectorXd& du, Eigen::VectorXd& dg) const override {
    // Evaluation of Vector dg
    dg(0) = dx(0) * g_x(0, 0);
    dg(1) = dx(1) * g_x(1, 1);
    dg(2) = dx(2) * g_x(2, 2);
  }
};

template <>
class SpeedOCPModelStateOnlyEqualities<StageType::TERMINAL> : public Constraints {
 public:
  SpeedOCPModelStateOnlyEqualities() : Constraints(3, 8, 0, 0, 0) {}

  virtual ~SpeedOCPModelStateOnlyEqualities() = default;

  virtual Type type() const override { return Type::STATE_ONLY_EQUALITY; }
};

}  // namespace gpal::pnc::planning