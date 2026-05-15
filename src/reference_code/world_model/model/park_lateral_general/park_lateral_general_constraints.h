#pragma once

#include "ocp/constraints.h"

namespace gpal::pnc::planning {

template <StageType Ttype>
class ParkLateralGeneralConstraints : public Constraints {
 public:
  ParkLateralGeneralConstraints() : Constraints(5, 2, 9, 2, 6) {
    // Evaluation of Matrix jacobian_.jx
    jacobian_.jx(6, 4) = -1;
    jacobian_.jx(7, 4) = 1;

    // Evaluation of Matrix jacobian_.ju
    jacobian_.ju(9, 0) = 1;
    jacobian_.ju(10, 0) = -1;
    jacobian_.ju(11, 1) = -1;
    jacobian_.ju(12, 1) = -1;
    jacobian_.ju(13, 1) = -1;
    jacobian_.ju(14, 1) = -1;
    jacobian_.ju(15, 1) = -1;
    jacobian_.ju(16, 1) = -1;
    exprs_[0] = "ll <= -(x - xr)*sin(thetar) + (y - yr)*cos(thetar)";
    exprs_[1] = "-(x - xr)*sin(thetar) + (y - yr)*cos(thetar) <= lu";
    exprs_[2] = "lfl <= (lf*sin(theta) + y - yrf)*cos(thetarf) - (lf*cos(theta) + x - xrf)*sin(thetarf)";
    exprs_[3] = "(lf*sin(theta) + y - yrf)*cos(thetarf) - (lf*cos(theta) + x - xrf)*sin(thetarf) <= lfu";
    exprs_[4] = "lrl <= (-lr*sin(theta) + y - yrr)*cos(thetarr) - (-lr*cos(theta) + x - xrr)*sin(thetarr)";
    exprs_[5] = "(-lr*sin(theta) + y - yrr)*cos(thetarr) - (-lr*cos(theta) + x - xrr)*sin(thetarr) <= lru";
    exprs_[6] = "kappa >= KappaLowerBound";
    exprs_[7] = "kappa <= KappaUpperBound";
    exprs_[8] = "cos(theta - thetar) >= 0.0";
    exprs_[9] = "dkappa <= DKappaUpperBound";
    exprs_[10] = "dkappa >= DKappaLowerBound";
    exprs_[11] = "sll <= slack_offset - (x - xr)*sin(thetar) + (y - yr)*cos(thetar)";
    exprs_[12] = "-slack_offset - (x - xr)*sin(thetar) + (y - yr)*cos(thetar) <= slu";
    exprs_[13] =
        "slfl <= slack_offset + (lf*sin(theta) + y - yrf)*cos(thetarf) - (lf*cos(theta) + x - xrf)*sin(thetarf)";
    exprs_[14] =
        "-slack_offset + (lf*sin(theta) + y - yrf)*cos(thetarf) - (lf*cos(theta) + x - xrf)*sin(thetarf) <= slfu";
    exprs_[15] =
        "slrl <= slack_offset + (-lr*sin(theta) + y - yrr)*cos(thetarr) - (-lr*cos(theta) + x - xrr)*sin(thetarr)";
    exprs_[16] =
        "-slack_offset + (-lr*sin(theta) + y - yrr)*cos(thetarr) - (-lr*cos(theta) + x - xrr)*sin(thetarr) <= slru";

    idx_with_slack_.emplace(11);
    idx_with_slack_.emplace(12);
    idx_with_slack_.emplace(13);
    idx_with_slack_.emplace(14);
    idx_with_slack_.emplace(15);
    idx_with_slack_.emplace(16);
  }

  virtual ~ParkLateralGeneralConstraints() = default;

  virtual Type type() const override { return Type::INEQUALITY; }
  virtual void updateEvaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                              Eigen::VectorXd& values) const override {
    // Determine stage variables
    const auto& kappa = states(4);
    const auto& dkappa = ctrls(0);
    const auto& slack_offset = ctrls(1);
    const auto& ll = params(19);
    const auto& lu = params(20);
    const auto& lfl = params(21);
    const auto& lfu = params(22);
    const auto& lrl = params(23);
    const auto& lru = params(24);
    const auto& sll = params(25);
    const auto& slu = params(26);
    const auto& KappaLowerBound = params(27);
    const auto& KappaUpperBound = params(28);
    const auto& DKappaLowerBound = params(29);
    const auto& DKappaUpperBound = params(30);
    const auto& slfl = params(35);
    const auto& slfu = params(36);
    const auto& slrl = params(37);
    const auto& slru = params(38);

    // Determine global variables
    const auto& cos_anonymous_0 = globals(9);
    const auto& l = globals(10);
    const auto& blf = globals(11);
    const auto& blr = globals(12);

    // Evaluation of Vector values
    values(0) = -l + ll;
    values(1) = l - lu;
    values(2) = -blf + lfl;
    values(3) = blf - lfu;
    values(4) = -blr + lrl;
    values(5) = blr - lru;
    values(6) = KappaLowerBound - kappa;
    values(7) = -KappaUpperBound + kappa;
    values(8) = -cos_anonymous_0;
    values(9) = -DKappaUpperBound + dkappa;
    values(10) = DKappaLowerBound - dkappa;
    values(11) = -l - slack_offset + sll;
    values(12) = l - slack_offset - slu;
    values(13) = -blf - slack_offset + slfl;
    values(14) = blf - slack_offset - slfu;
    values(15) = -blr - slack_offset + slrl;
    values(16) = blr - slack_offset - slru;
  }

  virtual void updateJacobian(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& jx,
                              Eigen::MatrixXd& ju) const override {
    // Determine stage variables
    const auto& lf = params(17);
    const auto& lr = params(18);

    // Determine global variables
    const auto& cos_theta = globals(0);
    const auto& sin_theta = globals(1);
    const auto& anonymous_0 = globals(2);
    const auto& cos_thetar = globals(3);
    const auto& sin_thetar = globals(4);
    const auto& cos_thetarf = globals(5);
    const auto& sin_thetarf = globals(6);
    const auto& cos_thetarr = globals(7);
    const auto& sin_thetarr = globals(8);

    // Determine internal variables
    const double sin_anonymous_0 = sin(anonymous_0);
    const double internal_0 = -cos_theta * cos_thetarf * lf;
    const double internal_1 = -lf * sin_theta * sin_thetarf;
    const double internal_2 = cos_theta * cos_thetarr * lr;
    const double internal_3 = lr * sin_theta * sin_thetarr;

    // Evaluation of Matrix jx
    jx(0, 1) = sin_thetar;
    jx(0, 2) = -cos_thetar;
    jx(1, 1) = -sin_thetar;
    jx(1, 2) = cos_thetar;
    jx(2, 1) = sin_thetarf;
    jx(2, 2) = -cos_thetarf;
    jx(2, 3) = internal_0 + internal_1;
    jx(3, 1) = -sin_thetarf;
    jx(3, 2) = cos_thetarf;
    jx(3, 3) = -jx(2, 3);
    jx(4, 1) = sin_thetarr;
    jx(4, 2) = -cos_thetarr;
    jx(4, 3) = internal_2 + internal_3;
    jx(5, 1) = -sin_thetarr;
    jx(5, 2) = cos_thetarr;
    jx(5, 3) = -jx(4, 3);
    jx(6, 4) = -1;
    jx(7, 4) = 1;
    jx(8, 3) = sin_anonymous_0;
    jx(11, 1) = sin_thetar;
    jx(11, 2) = -cos_thetar;
    jx(12, 1) = -sin_thetar;
    jx(12, 2) = cos_thetar;
    jx(13, 1) = sin_thetarf;
    jx(13, 2) = -cos_thetarf;
    jx(13, 3) = jx(2, 3);
    jx(14, 1) = -sin_thetarf;
    jx(14, 2) = cos_thetarf;
    jx(14, 3) = -jx(2, 3);
    jx(15, 1) = sin_thetarr;
    jx(15, 2) = -cos_thetarr;
    jx(15, 3) = jx(4, 3);
    jx(16, 1) = -sin_thetarr;
    jx(16, 2) = cos_thetarr;
    jx(16, 3) = -jx(4, 3);

    // Evaluation of Matrix ju
    ju(9, 0) = 1;
    ju(10, 0) = -1;
    ju(11, 1) = -1;
    ju(12, 1) = -1;
    ju(13, 1) = -1;
    ju(14, 1) = -1;
    ju(15, 1) = -1;
    ju(16, 1) = -1;
  }

  virtual void updateStateJacobian(const Eigen::VectorXd& states, Eigen::MatrixXd& jx) const override {
    // Determine stage variables
    const auto& lf = params(17);
    const auto& lr = params(18);

    // Determine global variables
    const auto& cos_theta = globals(0);
    const auto& sin_theta = globals(1);
    const auto& anonymous_0 = globals(2);
    const auto& cos_thetar = globals(3);
    const auto& sin_thetar = globals(4);
    const auto& cos_thetarf = globals(5);
    const auto& sin_thetarf = globals(6);
    const auto& cos_thetarr = globals(7);
    const auto& sin_thetarr = globals(8);

    // Determine internal variables
    const double sin_anonymous_0 = sin(anonymous_0);
    const double internal_0 = -cos_theta * cos_thetarf * lf;
    const double internal_1 = -lf * sin_theta * sin_thetarf;
    const double internal_2 = cos_theta * cos_thetarr * lr;
    const double internal_3 = lr * sin_theta * sin_thetarr;

    // Evaluation of Matrix jx
    jx(0, 1) = sin_thetar;
    jx(0, 2) = -cos_thetar;
    jx(1, 1) = -sin_thetar;
    jx(1, 2) = cos_thetar;
    jx(2, 1) = sin_thetarf;
    jx(2, 2) = -cos_thetarf;
    jx(2, 3) = internal_0 + internal_1;
    jx(3, 1) = -sin_thetarf;
    jx(3, 2) = cos_thetarf;
    jx(3, 3) = -jx(2, 3);
    jx(4, 1) = sin_thetarr;
    jx(4, 2) = -cos_thetarr;
    jx(4, 3) = internal_2 + internal_3;
    jx(5, 1) = -sin_thetarr;
    jx(5, 2) = cos_thetarr;
    jx(5, 3) = -jx(4, 3);
    jx(6, 4) = -1;
    jx(7, 4) = 1;
    jx(8, 3) = sin_anonymous_0;
  }

  virtual void updateCtrlJacobian(const Eigen::VectorXd& ctrls, Eigen::MatrixXd& ju) const override {
    // Evaluation of Matrix ju
    ju(0, 0) = 1;
    ju(1, 0) = -1;
  }

  virtual void updateJacobianDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                Eigen::MatrixXd& jx) const override {
    // Determine stage variables
    const auto& lf = params(17);
    const auto& lr = params(18);

    // Determine global variables
    const auto& cos_theta = globals(0);
    const auto& sin_theta = globals(1);
    const auto& anonymous_0 = globals(2);
    const auto& cos_thetar = globals(3);
    const auto& sin_thetar = globals(4);
    const auto& cos_thetarf = globals(5);
    const auto& sin_thetarf = globals(6);
    const auto& cos_thetarr = globals(7);
    const auto& sin_thetarr = globals(8);

    // Determine internal variables
    const double sin_anonymous_0 = sin(anonymous_0);
    const double internal_0 = -cos_theta * cos_thetarf * lf;
    const double internal_1 = -lf * sin_theta * sin_thetarf;
    const double internal_2 = cos_theta * cos_thetarr * lr;
    const double internal_3 = lr * sin_theta * sin_thetarr;

    // Evaluation of Matrix jx
    jx(0, 1) = sin_thetar;
    jx(0, 2) = -cos_thetar;
    jx(1, 1) = -sin_thetar;
    jx(1, 2) = cos_thetar;
    jx(2, 1) = sin_thetarf;
    jx(2, 2) = -cos_thetarf;
    jx(2, 3) = internal_0 + internal_1;
    jx(3, 1) = -sin_thetarf;
    jx(3, 2) = cos_thetarf;
    jx(3, 3) = -jx(2, 3);
    jx(4, 1) = sin_thetarr;
    jx(4, 2) = -cos_thetarr;
    jx(4, 3) = internal_2 + internal_3;
    jx(5, 1) = -sin_thetarr;
    jx(5, 2) = cos_thetarr;
    jx(5, 3) = -jx(4, 3);
    jx(6, 4) = -1;
    jx(7, 4) = 1;
    jx(8, 3) = sin_anonymous_0;
    jx(11, 1) = sin_thetar;
    jx(11, 2) = -cos_thetar;
    jx(12, 1) = -sin_thetar;
    jx(12, 2) = cos_thetar;
    jx(13, 1) = sin_thetarf;
    jx(13, 2) = -cos_thetarf;
    jx(13, 3) = jx(2, 3);
    jx(14, 1) = -sin_thetarf;
    jx(14, 2) = cos_thetarf;
    jx(14, 3) = -jx(2, 3);
    jx(15, 1) = sin_thetarr;
    jx(15, 2) = -cos_thetarr;
    jx(15, 3) = jx(4, 3);
    jx(16, 1) = -sin_thetarr;
    jx(16, 2) = cos_thetarr;
    jx(16, 3) = -jx(4, 3);
  }

  virtual void updateJacobianDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                Eigen::MatrixXd& ju) const override {
    // Evaluation of Matrix ju
    ju(9, 0) = 1;
    ju(10, 0) = -1;
    ju(11, 1) = -1;
    ju(12, 1) = -1;
    ju(13, 1) = -1;
    ju(14, 1) = -1;
    ju(15, 1) = -1;
    ju(16, 1) = -1;
  }

  virtual void updateDeltaEvaluation(const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::VectorXd& dx,
                                     const Eigen::VectorXd& du, Eigen::VectorXd& dg) const override {
    // Evaluation of Vector dg
    dg(0) = dx(1) * g_x(0, 1) + dx(2) * g_x(0, 2);
    dg(1) = dx(1) * g_x(1, 1) + dx(2) * g_x(1, 2);
    dg(2) = dx(1) * g_x(2, 1) + dx(2) * g_x(2, 2) + dx(3) * g_x(2, 3);
    dg(3) = dx(1) * g_x(3, 1) + dx(2) * g_x(3, 2) + dx(3) * g_x(3, 3);
    dg(4) = dx(1) * g_x(4, 1) + dx(2) * g_x(4, 2) + dx(3) * g_x(4, 3);
    dg(5) = dx(1) * g_x(5, 1) + dx(2) * g_x(5, 2) + dx(3) * g_x(5, 3);
    dg(6) = dx(4) * g_x(6, 4);
    dg(7) = dx(4) * g_x(7, 4);
    dg(8) = dx(3) * g_x(8, 3);
    dg(9) = du(0) * g_u(9, 0);
    dg(10) = du(0) * g_u(10, 0);
    dg(11) = du(1) * g_u(11, 1) + dx(1) * g_x(11, 1) + dx(2) * g_x(11, 2);
    dg(12) = du(1) * g_u(12, 1) + dx(1) * g_x(12, 1) + dx(2) * g_x(12, 2);
    dg(13) = du(1) * g_u(13, 1) + dx(1) * g_x(13, 1) + dx(2) * g_x(13, 2) + dx(3) * g_x(13, 3);
    dg(14) = du(1) * g_u(14, 1) + dx(1) * g_x(14, 1) + dx(2) * g_x(14, 2) + dx(3) * g_x(14, 3);
    dg(15) = du(1) * g_u(15, 1) + dx(1) * g_x(15, 1) + dx(2) * g_x(15, 2) + dx(3) * g_x(15, 3);
    dg(16) = du(1) * g_u(16, 1) + dx(1) * g_x(16, 1) + dx(2) * g_x(16, 2) + dx(3) * g_x(16, 3);
  }
};

template <>
class ParkLateralGeneralConstraints<StageType::INITIAL> : public Constraints {
 public:
  ParkLateralGeneralConstraints() : Constraints(5, 2, 0, 2, 0) {
    // Evaluation of Matrix jacobian_.ju
    jacobian_.ju(0, 0) = 1;
    jacobian_.ju(1, 0) = -1;
    exprs_[0] = "dkappa <= DKappaUpperBound";
    exprs_[1] = "dkappa >= DKappaLowerBound";
  }

  virtual ~ParkLateralGeneralConstraints() = default;

  virtual Type type() const override { return Type::INEQUALITY; }
  virtual void updateEvaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                              Eigen::VectorXd& values) const override {
    // Determine stage variables
    const auto& dkappa = ctrls(0);
    const auto& DKappaLowerBound = params(29);
    const auto& DKappaUpperBound = params(30);

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
class ParkLateralGeneralConstraints<StageType::TERMINAL> : public Constraints {
 public:
  ParkLateralGeneralConstraints() : Constraints(5, 2, 9, 0, 0) {
    // Evaluation of Matrix jacobian_.jx
    jacobian_.jx(6, 4) = -1;
    jacobian_.jx(7, 4) = 1;

    exprs_[0] = "ll <= -(x - xr)*sin(thetar) + (y - yr)*cos(thetar)";
    exprs_[1] = "-(x - xr)*sin(thetar) + (y - yr)*cos(thetar) <= lu";
    exprs_[2] = "lfl <= (lf*sin(theta) + y - yrf)*cos(thetarf) - (lf*cos(theta) + x - xrf)*sin(thetarf)";
    exprs_[3] = "(lf*sin(theta) + y - yrf)*cos(thetarf) - (lf*cos(theta) + x - xrf)*sin(thetarf) <= lfu";
    exprs_[4] = "lrl <= (-lr*sin(theta) + y - yrr)*cos(thetarr) - (-lr*cos(theta) + x - xrr)*sin(thetarr)";
    exprs_[5] = "(-lr*sin(theta) + y - yrr)*cos(thetarr) - (-lr*cos(theta) + x - xrr)*sin(thetarr) <= lru";
    exprs_[6] = "kappa >= KappaLowerBound";
    exprs_[7] = "kappa <= KappaUpperBound";
    exprs_[8] = "cos(theta - thetar) >= 0.0";
  }

  virtual ~ParkLateralGeneralConstraints() = default;

  virtual Type type() const override { return Type::INEQUALITY; }
  virtual void updateEvaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                              Eigen::VectorXd& values) const override {
    // Determine stage variables
    const auto& kappa = states(4);
    const auto& ll = params(19);
    const auto& lu = params(20);
    const auto& lfl = params(21);
    const auto& lfu = params(22);
    const auto& lrl = params(23);
    const auto& lru = params(24);
    const auto& KappaLowerBound = params(27);
    const auto& KappaUpperBound = params(28);

    // Determine global variables
    const auto& cos_anonymous_0 = globals(9);
    const auto& l = globals(10);
    const auto& blf = globals(11);
    const auto& blr = globals(12);

    // Evaluation of Vector values
    values(0) = -l + ll;
    values(1) = l - lu;
    values(2) = -blf + lfl;
    values(3) = blf - lfu;
    values(4) = -blr + lrl;
    values(5) = blr - lru;
    values(6) = KappaLowerBound - kappa;
    values(7) = -KappaUpperBound + kappa;
    values(8) = -cos_anonymous_0;
  }

  virtual void updateJacobian(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& jx,
                              Eigen::MatrixXd& ju) const override {
    // Determine stage variables
    const auto& lf = params(17);
    const auto& lr = params(18);

    // Determine global variables
    const auto& cos_theta = globals(0);
    const auto& sin_theta = globals(1);
    const auto& anonymous_0 = globals(2);
    const auto& cos_thetar = globals(3);
    const auto& sin_thetar = globals(4);
    const auto& cos_thetarf = globals(5);
    const auto& sin_thetarf = globals(6);
    const auto& cos_thetarr = globals(7);
    const auto& sin_thetarr = globals(8);

    // Determine internal variables
    const double sin_anonymous_0 = sin(anonymous_0);
    const double internal_0 = -cos_theta * cos_thetarf * lf;
    const double internal_1 = -lf * sin_theta * sin_thetarf;
    const double internal_2 = cos_theta * cos_thetarr * lr;
    const double internal_3 = lr * sin_theta * sin_thetarr;

    // Evaluation of Matrix jx
    jx(0, 1) = sin_thetar;
    jx(0, 2) = -cos_thetar;
    jx(1, 1) = -sin_thetar;
    jx(1, 2) = cos_thetar;
    jx(2, 1) = sin_thetarf;
    jx(2, 2) = -cos_thetarf;
    jx(2, 3) = internal_0 + internal_1;
    jx(3, 1) = -sin_thetarf;
    jx(3, 2) = cos_thetarf;
    jx(3, 3) = -jx(2, 3);
    jx(4, 1) = sin_thetarr;
    jx(4, 2) = -cos_thetarr;
    jx(4, 3) = internal_2 + internal_3;
    jx(5, 1) = -sin_thetarr;
    jx(5, 2) = cos_thetarr;
    jx(5, 3) = -jx(4, 3);
    jx(6, 4) = -1;
    jx(7, 4) = 1;
    jx(8, 3) = sin_anonymous_0;
  }

  virtual void updateStateJacobian(const Eigen::VectorXd& states, Eigen::MatrixXd& jx) const override {
    // Determine stage variables
    const auto& lf = params(17);
    const auto& lr = params(18);

    // Determine global variables
    const auto& cos_theta = globals(0);
    const auto& sin_theta = globals(1);
    const auto& anonymous_0 = globals(2);
    const auto& cos_thetar = globals(3);
    const auto& sin_thetar = globals(4);
    const auto& cos_thetarf = globals(5);
    const auto& sin_thetarf = globals(6);
    const auto& cos_thetarr = globals(7);
    const auto& sin_thetarr = globals(8);

    // Determine internal variables
    const double sin_anonymous_0 = sin(anonymous_0);
    const double internal_0 = -cos_theta * cos_thetarf * lf;
    const double internal_1 = -lf * sin_theta * sin_thetarf;
    const double internal_2 = cos_theta * cos_thetarr * lr;
    const double internal_3 = lr * sin_theta * sin_thetarr;

    // Evaluation of Matrix jx
    jx(0, 1) = sin_thetar;
    jx(0, 2) = -cos_thetar;
    jx(1, 1) = -sin_thetar;
    jx(1, 2) = cos_thetar;
    jx(2, 1) = sin_thetarf;
    jx(2, 2) = -cos_thetarf;
    jx(2, 3) = internal_0 + internal_1;
    jx(3, 1) = -sin_thetarf;
    jx(3, 2) = cos_thetarf;
    jx(3, 3) = -jx(2, 3);
    jx(4, 1) = sin_thetarr;
    jx(4, 2) = -cos_thetarr;
    jx(4, 3) = internal_2 + internal_3;
    jx(5, 1) = -sin_thetarr;
    jx(5, 2) = cos_thetarr;
    jx(5, 3) = -jx(4, 3);
    jx(6, 4) = -1;
    jx(7, 4) = 1;
    jx(8, 3) = sin_anonymous_0;
  }

  virtual void updateJacobianDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                                Eigen::MatrixXd& jx) const override {
    // Determine stage variables
    const auto& lf = params(17);
    const auto& lr = params(18);

    // Determine global variables
    const auto& cos_theta = globals(0);
    const auto& sin_theta = globals(1);
    const auto& anonymous_0 = globals(2);
    const auto& cos_thetar = globals(3);
    const auto& sin_thetar = globals(4);
    const auto& cos_thetarf = globals(5);
    const auto& sin_thetarf = globals(6);
    const auto& cos_thetarr = globals(7);
    const auto& sin_thetarr = globals(8);

    // Determine internal variables
    const double sin_anonymous_0 = sin(anonymous_0);
    const double internal_0 = -cos_theta * cos_thetarf * lf;
    const double internal_1 = -lf * sin_theta * sin_thetarf;
    const double internal_2 = cos_theta * cos_thetarr * lr;
    const double internal_3 = lr * sin_theta * sin_thetarr;

    // Evaluation of Matrix jx
    jx(0, 1) = sin_thetar;
    jx(0, 2) = -cos_thetar;
    jx(1, 1) = -sin_thetar;
    jx(1, 2) = cos_thetar;
    jx(2, 1) = sin_thetarf;
    jx(2, 2) = -cos_thetarf;
    jx(2, 3) = internal_0 + internal_1;
    jx(3, 1) = -sin_thetarf;
    jx(3, 2) = cos_thetarf;
    jx(3, 3) = -jx(2, 3);
    jx(4, 1) = sin_thetarr;
    jx(4, 2) = -cos_thetarr;
    jx(4, 3) = internal_2 + internal_3;
    jx(5, 1) = -sin_thetarr;
    jx(5, 2) = cos_thetarr;
    jx(5, 3) = -jx(4, 3);
    jx(6, 4) = -1;
    jx(7, 4) = 1;
    jx(8, 3) = sin_anonymous_0;
  }

  virtual void updateDeltaEvaluation(const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::VectorXd& dx,
                                     const Eigen::VectorXd& du, Eigen::VectorXd& dg) const override {
    // Evaluation of Vector dg
    dg(0) = dx(1) * g_x(0, 1) + dx(2) * g_x(0, 2);
    dg(1) = dx(1) * g_x(1, 1) + dx(2) * g_x(1, 2);
    dg(2) = dx(1) * g_x(2, 1) + dx(2) * g_x(2, 2) + dx(3) * g_x(2, 3);
    dg(3) = dx(1) * g_x(3, 1) + dx(2) * g_x(3, 2) + dx(3) * g_x(3, 3);
    dg(4) = dx(1) * g_x(4, 1) + dx(2) * g_x(4, 2) + dx(3) * g_x(4, 3);
    dg(5) = dx(1) * g_x(5, 1) + dx(2) * g_x(5, 2) + dx(3) * g_x(5, 3);
    dg(6) = dx(4) * g_x(6, 4);
    dg(7) = dx(4) * g_x(7, 4);
    dg(8) = dx(3) * g_x(8, 3);
  }
};

template <StageType Ttype>
class ParkLateralGeneralStateOnlyEqualities : public Constraints {
 public:
  ParkLateralGeneralStateOnlyEqualities() : Constraints(5, 2, 0, 0, 0) {}

  virtual ~ParkLateralGeneralStateOnlyEqualities() = default;

  virtual Type type() const override { return Type::STATE_ONLY_EQUALITY; }
};

template <>
class ParkLateralGeneralStateOnlyEqualities<StageType::INITIAL> : public Constraints {
 public:
  ParkLateralGeneralStateOnlyEqualities() : Constraints(5, 2, 5, 0, 0) {
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
    exprs_[4] = "kappa = 0";
  }

  virtual ~ParkLateralGeneralStateOnlyEqualities() = default;

  virtual Type type() const override { return Type::STATE_ONLY_EQUALITY; }
  virtual void updateEvaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls,
                              Eigen::VectorXd& values) const override {
    // Determine stage variables
    const auto& s = states(0);
    const auto& x = states(1);
    const auto& y = states(2);
    const auto& theta = states(3);
    const auto& kappa = states(4);

    // Evaluation of Vector values
    values(0) = s;
    values(1) = x;
    values(2) = y;
    values(3) = theta;
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
class ParkLateralGeneralStateOnlyEqualities<StageType::TERMINAL> : public Constraints {
 public:
  ParkLateralGeneralStateOnlyEqualities() : Constraints(5, 2, 0, 0, 0) {}

  virtual ~ParkLateralGeneralStateOnlyEqualities() = default;

  virtual Type type() const override { return Type::STATE_ONLY_EQUALITY; }
};

}  // namespace gpal::pnc::planning