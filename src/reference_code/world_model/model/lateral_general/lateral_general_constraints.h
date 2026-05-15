#pragma once

#include "ocp/constraints.h"

namespace gpal::pnc::planning {

template <StageType Ttype>
class LateralGeneralConstraints : public Constraints {
 public:
  LateralGeneralConstraints() : Constraints(5, 2, 9, 2, 6) {
    // Evaluation of Matrix jacobian_.jx
    jacobian_.jx(6, 4) = -1;
    jacobian_.jx(7, 4) = 1;

    // Evaluation of Matrix jacobian_.ju
    jacobian_.ju(9, 0) = -1;
    jacobian_.ju(10, 0) = 1;
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
    exprs_[6] = "steer >= SteerLowerBound";
    exprs_[7] = "steer <= SteerUpperBound";
    exprs_[8] = "cos(theta - thetar) >= 0.0";
    exprs_[9] = "dsteer >= DSteerLowerBound";
    exprs_[10] = "dsteer <= DSteerUpperBound";
    exprs_[11] = "sll <= slack_offset - (x - xr)*sin(thetar) + (y - yr)*cos(thetar)";
    exprs_[12] = "-slack_offset - (x - xr)*sin(thetar) + (y - yr)*cos(thetar) <= slu";
    exprs_[13] = "slfl <= slack_offset + (lf*sin(theta) + y - yrf)*cos(thetarf) - (lf*cos(theta) + x - xrf)*sin(thetarf)";
    exprs_[14] = "-slack_offset + (lf*sin(theta) + y - yrf)*cos(thetarf) - (lf*cos(theta) + x - xrf)*sin(thetarf) <= slfu";
    exprs_[15] = "slrl <= slack_offset + (-lr*sin(theta) + y - yrr)*cos(thetarr) - (-lr*cos(theta) + x - xrr)*sin(thetarr)";
    exprs_[16] = "-slack_offset + (-lr*sin(theta) + y - yrr)*cos(thetarr) - (-lr*cos(theta) + x - xrr)*sin(thetarr) <= slru";

    idx_with_slack_.emplace(11);
    idx_with_slack_.emplace(12);
    idx_with_slack_.emplace(13);
    idx_with_slack_.emplace(14);
    idx_with_slack_.emplace(15);
    idx_with_slack_.emplace(16);
  }

  virtual ~LateralGeneralConstraints() = default;

  virtual Type type() const override { return Type::INEQUALITY; }
  virtual void updateEvaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::VectorXd& values) const override {
    // Determine stage variables
    const auto& theta = states(3);
    const auto& steer = states(4);
    const auto& dsteer = ctrls(0);
    const auto& slack_offset = ctrls(1);
    const auto& thetar = params(3);
    const auto& ll = params(20);
    const auto& lu = params(21);
    const auto& sll = params(22);
    const auto& slu = params(23);
    const auto& lfl = params(24);
    const auto& lfu = params(25);
    const auto& slfl = params(26);
    const auto& slfu = params(27);
    const auto& lrl = params(28);
    const auto& lru = params(29);
    const auto& slrl = params(30);
    const auto& slru = params(31);
    const auto& SteerLowerBound = params(32);
    const auto& SteerUpperBound = params(33);
    const auto& DSteerLowerBound = params(34);
    const auto& DSteerUpperBound = params(35);

    // Determine global variables
    const auto& l = globals(1);
    const auto& blf = globals(2);
    const auto& blr = globals(3);

    // Determine internal variables
    const double internal_0 = -steer;
    const double internal_1 = -dsteer;

    // Evaluation of Vector values
    values(0) = -l + ll;
    values(1) = l - lu;
    values(2) = -blf + lfl;
    values(3) = blf - lfu;
    values(4) = -blr + lrl;
    values(5) = blr - lru;
    values(6) = SteerLowerBound + internal_0;
    values(7) = -SteerUpperBound - internal_0;
    values(8) = -cos(theta - thetar);
    values(9) = DSteerLowerBound + internal_1;
    values(10) = -DSteerUpperBound - internal_1;
    values(11) = -l - slack_offset + sll;
    values(12) = l - slack_offset - slu;
    values(13) = -blf - slack_offset + slfl;
    values(14) = blf - slack_offset - slfu;
    values(15) = -blr - slack_offset + slrl;
    values(16) = blr - slack_offset - slru;

  }

  virtual void updateJacobian(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& jx, Eigen::MatrixXd& ju) const override {
    // Determine stage variables
    const auto& theta = states(3);
    const auto& thetar = params(3);
    const auto& thetarf = params(14);
    const auto& thetarr = params(17);
    const auto& lf = params(18);
    const auto& lr = params(19);

    // Determine internal variables
    const double internal_7 = cos(theta);
    const double internal_8 = sin(theta);
    const double internal_0 = sin(thetar);
    const double internal_1 = cos(thetar);
    const double internal_4 = sin(thetarf);
    const double internal_5 = cos(thetarf);
    const double internal_12 = sin(thetarr);
    const double internal_13 = cos(thetarr);
    const double internal_2 = -internal_1;
    const double internal_3 = -internal_0;
    const double internal_11 = -internal_4;
    const double internal_6 = -internal_5;
    const double internal_14 = -internal_13;
    const double internal_16 = -internal_12;
    const double internal_9 = internal_4*internal_8*lf + internal_5*internal_7*lf;
    const double internal_15 = internal_12*internal_8*lr + internal_13*internal_7*lr;
    const double internal_10 = -internal_9;
    const double internal_17 = -internal_15;

    // Evaluation of Matrix jx
    jx(0, 1) = internal_0;
    jx(0, 2) = internal_2;
    jx(1, 1) = internal_3;
    jx(1, 2) = internal_1;
    jx(2, 1) = internal_4;
    jx(2, 2) = internal_6;
    jx(2, 3) = internal_10;
    jx(3, 1) = internal_11;
    jx(3, 2) = internal_5;
    jx(3, 3) = internal_9;
    jx(4, 1) = internal_12;
    jx(4, 2) = internal_14;
    jx(4, 3) = internal_15;
    jx(5, 1) = internal_16;
    jx(5, 2) = internal_13;
    jx(5, 3) = internal_17;
    jx(6, 4) = -1;
    jx(7, 4) = 1;
    jx(8, 3) = sin(theta - thetar);
    jx(11, 1) = internal_0;
    jx(11, 2) = internal_2;
    jx(12, 1) = internal_3;
    jx(12, 2) = internal_1;
    jx(13, 1) = internal_4;
    jx(13, 2) = internal_6;
    jx(13, 3) = internal_10;
    jx(14, 1) = internal_11;
    jx(14, 2) = internal_5;
    jx(14, 3) = internal_9;
    jx(15, 1) = internal_12;
    jx(15, 2) = internal_14;
    jx(15, 3) = internal_15;
    jx(16, 1) = internal_16;
    jx(16, 2) = internal_13;
    jx(16, 3) = internal_17;

    // Evaluation of Matrix ju
    ju(9, 0) = -1;
    ju(10, 0) = 1;
    ju(11, 1) = -1;
    ju(12, 1) = -1;
    ju(13, 1) = -1;
    ju(14, 1) = -1;
    ju(15, 1) = -1;
    ju(16, 1) = -1;

  }

  virtual void updateStateJacobian(const Eigen::VectorXd& states, Eigen::MatrixXd& jx) const override {
    // Determine stage variables
    const auto& theta = states(3);
    const auto& thetar = params(3);
    const auto& thetarf = params(14);
    const auto& thetarr = params(17);
    const auto& lf = params(18);
    const auto& lr = params(19);

    // Determine internal variables
    const double internal_4 = cos(theta);
    const double internal_5 = sin(theta);
    const double internal_0 = sin(thetar);
    const double internal_1 = cos(thetar);
    const double internal_2 = sin(thetarf);
    const double internal_3 = cos(thetarf);
    const double internal_7 = sin(thetarr);
    const double internal_8 = cos(thetarr);
    const double internal_6 = internal_2*internal_5*lf + internal_3*internal_4*lf;
    const double internal_9 = internal_4*internal_8*lr + internal_5*internal_7*lr;

    // Evaluation of Matrix jx
    jx(0, 1) = internal_0;
    jx(0, 2) = -internal_1;
    jx(1, 1) = -internal_0;
    jx(1, 2) = internal_1;
    jx(2, 1) = internal_2;
    jx(2, 2) = -internal_3;
    jx(2, 3) = -internal_6;
    jx(3, 1) = -internal_2;
    jx(3, 2) = internal_3;
    jx(3, 3) = internal_6;
    jx(4, 1) = internal_7;
    jx(4, 2) = -internal_8;
    jx(4, 3) = internal_9;
    jx(5, 1) = -internal_7;
    jx(5, 2) = internal_8;
    jx(5, 3) = -internal_9;
    jx(6, 4) = -1;
    jx(7, 4) = 1;
    jx(8, 3) = sin(theta - thetar);

  }

  virtual void updateCtrlJacobian(const Eigen::VectorXd& ctrls, Eigen::MatrixXd& ju) const override {
    // Evaluation of Matrix ju
    ju(0, 0) = -1;
    ju(1, 0) = 1;

  }

  virtual void updateJacobianDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& jx) const override {
    // Determine stage variables
    const auto& theta = states(3);
    const auto& thetar = params(3);
    const auto& thetarf = params(14);
    const auto& thetarr = params(17);
    const auto& lf = params(18);
    const auto& lr = params(19);

    // Determine internal variables
    const double internal_7 = cos(theta);
    const double internal_8 = sin(theta);
    const double internal_0 = sin(thetar);
    const double internal_1 = cos(thetar);
    const double internal_4 = sin(thetarf);
    const double internal_5 = cos(thetarf);
    const double internal_12 = sin(thetarr);
    const double internal_13 = cos(thetarr);
    const double internal_2 = -internal_1;
    const double internal_3 = -internal_0;
    const double internal_11 = -internal_4;
    const double internal_6 = -internal_5;
    const double internal_14 = -internal_13;
    const double internal_16 = -internal_12;
    const double internal_9 = internal_4*internal_8*lf + internal_5*internal_7*lf;
    const double internal_15 = internal_12*internal_8*lr + internal_13*internal_7*lr;
    const double internal_10 = -internal_9;
    const double internal_17 = -internal_15;

    // Evaluation of Matrix jx
    jx(0, 1) = internal_0;
    jx(0, 2) = internal_2;
    jx(1, 1) = internal_3;
    jx(1, 2) = internal_1;
    jx(2, 1) = internal_4;
    jx(2, 2) = internal_6;
    jx(2, 3) = internal_10;
    jx(3, 1) = internal_11;
    jx(3, 2) = internal_5;
    jx(3, 3) = internal_9;
    jx(4, 1) = internal_12;
    jx(4, 2) = internal_14;
    jx(4, 3) = internal_15;
    jx(5, 1) = internal_16;
    jx(5, 2) = internal_13;
    jx(5, 3) = internal_17;
    jx(6, 4) = -1;
    jx(7, 4) = 1;
    jx(8, 3) = sin(theta - thetar);
    jx(11, 1) = internal_0;
    jx(11, 2) = internal_2;
    jx(12, 1) = internal_3;
    jx(12, 2) = internal_1;
    jx(13, 1) = internal_4;
    jx(13, 2) = internal_6;
    jx(13, 3) = internal_10;
    jx(14, 1) = internal_11;
    jx(14, 2) = internal_5;
    jx(14, 3) = internal_9;
    jx(15, 1) = internal_12;
    jx(15, 2) = internal_14;
    jx(15, 3) = internal_15;
    jx(16, 1) = internal_16;
    jx(16, 2) = internal_13;
    jx(16, 3) = internal_17;

  }

  virtual void updateJacobianDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& ju) const override {
    // Evaluation of Matrix ju
    ju(9, 0) = -1;
    ju(10, 0) = 1;
    ju(11, 1) = -1;
    ju(12, 1) = -1;
    ju(13, 1) = -1;
    ju(14, 1) = -1;
    ju(15, 1) = -1;
    ju(16, 1) = -1;

  }

  virtual void updateDeltaEvaluation(const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::VectorXd& dx, const Eigen::VectorXd& du, Eigen::VectorXd& dg) const override {
    // Evaluation of Vector dg
    dg(0) = dx(1)*g_x(0, 1) + dx(2)*g_x(0, 2);
    dg(1) = dx(1)*g_x(1, 1) + dx(2)*g_x(1, 2);
    dg(2) = dx(1)*g_x(2, 1) + dx(2)*g_x(2, 2) + dx(3)*g_x(2, 3);
    dg(3) = dx(1)*g_x(3, 1) + dx(2)*g_x(3, 2) + dx(3)*g_x(3, 3);
    dg(4) = dx(1)*g_x(4, 1) + dx(2)*g_x(4, 2) + dx(3)*g_x(4, 3);
    dg(5) = dx(1)*g_x(5, 1) + dx(2)*g_x(5, 2) + dx(3)*g_x(5, 3);
    dg(6) = dx(4)*g_x(6, 4);
    dg(7) = dx(4)*g_x(7, 4);
    dg(8) = dx(3)*g_x(8, 3);
    dg(9) = du(0)*g_u(9, 0);
    dg(10) = du(0)*g_u(10, 0);
    dg(11) = du(1)*g_u(11, 1) + dx(1)*g_x(11, 1) + dx(2)*g_x(11, 2);
    dg(12) = du(1)*g_u(12, 1) + dx(1)*g_x(12, 1) + dx(2)*g_x(12, 2);
    dg(13) = du(1)*g_u(13, 1) + dx(1)*g_x(13, 1) + dx(2)*g_x(13, 2) + dx(3)*g_x(13, 3);
    dg(14) = du(1)*g_u(14, 1) + dx(1)*g_x(14, 1) + dx(2)*g_x(14, 2) + dx(3)*g_x(14, 3);
    dg(15) = du(1)*g_u(15, 1) + dx(1)*g_x(15, 1) + dx(2)*g_x(15, 2) + dx(3)*g_x(15, 3);
    dg(16) = du(1)*g_u(16, 1) + dx(1)*g_x(16, 1) + dx(2)*g_x(16, 2) + dx(3)*g_x(16, 3);

  }

};

template <>
class LateralGeneralConstraints<StageType::INITIAL> : public Constraints {
 public:
  LateralGeneralConstraints() : Constraints(5, 2, 0, 2, 0) {
    // Evaluation of Matrix jacobian_.ju
    jacobian_.ju(0, 0) = -1;
    jacobian_.ju(1, 0) = 1;
    exprs_[0] = "dsteer >= DSteerLowerBound";
    exprs_[1] = "dsteer <= DSteerUpperBound";

  }

  virtual ~LateralGeneralConstraints() = default;

  virtual Type type() const override { return Type::INEQUALITY; }
  virtual void updateEvaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::VectorXd& values) const override {
    // Determine stage variables
    const auto& dsteer = ctrls(0);
    const auto& DSteerLowerBound = params(34);
    const auto& DSteerUpperBound = params(35);

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
class LateralGeneralConstraints<StageType::TERMINAL> : public Constraints {
 public:
  LateralGeneralConstraints() : Constraints(5, 2, 9, 0, 0) {
    // Evaluation of Matrix jacobian_.jx
    jacobian_.jx(6, 4) = -1;
    jacobian_.jx(7, 4) = 1;

    exprs_[0] = "ll <= -(x - xr)*sin(thetar) + (y - yr)*cos(thetar)";
    exprs_[1] = "-(x - xr)*sin(thetar) + (y - yr)*cos(thetar) <= lu";
    exprs_[2] = "lfl <= (lf*sin(theta) + y - yrf)*cos(thetarf) - (lf*cos(theta) + x - xrf)*sin(thetarf)";
    exprs_[3] = "(lf*sin(theta) + y - yrf)*cos(thetarf) - (lf*cos(theta) + x - xrf)*sin(thetarf) <= lfu";
    exprs_[4] = "lrl <= (-lr*sin(theta) + y - yrr)*cos(thetarr) - (-lr*cos(theta) + x - xrr)*sin(thetarr)";
    exprs_[5] = "(-lr*sin(theta) + y - yrr)*cos(thetarr) - (-lr*cos(theta) + x - xrr)*sin(thetarr) <= lru";
    exprs_[6] = "steer >= SteerLowerBound";
    exprs_[7] = "steer <= SteerUpperBound";
    exprs_[8] = "cos(theta - thetar) >= 0.0";

  }

  virtual ~LateralGeneralConstraints() = default;

  virtual Type type() const override { return Type::INEQUALITY; }
  virtual void updateEvaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::VectorXd& values) const override {
    // Determine stage variables
    const auto& theta = states(3);
    const auto& steer = states(4);
    const auto& thetar = params(3);
    const auto& ll = params(20);
    const auto& lu = params(21);
    const auto& lfl = params(24);
    const auto& lfu = params(25);
    const auto& lrl = params(28);
    const auto& lru = params(29);
    const auto& SteerLowerBound = params(32);
    const auto& SteerUpperBound = params(33);

    // Determine global variables
    const auto& l = globals(1);
    const auto& blf = globals(2);
    const auto& blr = globals(3);

    // Determine internal variables
    const double internal_0 = -steer;

    // Evaluation of Vector values
    values(0) = -l + ll;
    values(1) = l - lu;
    values(2) = -blf + lfl;
    values(3) = blf - lfu;
    values(4) = -blr + lrl;
    values(5) = blr - lru;
    values(6) = SteerLowerBound + internal_0;
    values(7) = -SteerUpperBound - internal_0;
    values(8) = -cos(theta - thetar);

  }

  virtual void updateJacobian(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& jx, Eigen::MatrixXd& ju) const override {
    // Determine stage variables
    const auto& theta = states(3);
    const auto& thetar = params(3);
    const auto& thetarf = params(14);
    const auto& thetarr = params(17);
    const auto& lf = params(18);
    const auto& lr = params(19);

    // Determine internal variables
    const double internal_4 = cos(theta);
    const double internal_5 = sin(theta);
    const double internal_0 = sin(thetar);
    const double internal_1 = cos(thetar);
    const double internal_2 = sin(thetarf);
    const double internal_3 = cos(thetarf);
    const double internal_7 = sin(thetarr);
    const double internal_8 = cos(thetarr);
    const double internal_6 = internal_2*internal_5*lf + internal_3*internal_4*lf;
    const double internal_9 = internal_4*internal_8*lr + internal_5*internal_7*lr;

    // Evaluation of Matrix jx
    jx(0, 1) = internal_0;
    jx(0, 2) = -internal_1;
    jx(1, 1) = -internal_0;
    jx(1, 2) = internal_1;
    jx(2, 1) = internal_2;
    jx(2, 2) = -internal_3;
    jx(2, 3) = -internal_6;
    jx(3, 1) = -internal_2;
    jx(3, 2) = internal_3;
    jx(3, 3) = internal_6;
    jx(4, 1) = internal_7;
    jx(4, 2) = -internal_8;
    jx(4, 3) = internal_9;
    jx(5, 1) = -internal_7;
    jx(5, 2) = internal_8;
    jx(5, 3) = -internal_9;
    jx(6, 4) = -1;
    jx(7, 4) = 1;
    jx(8, 3) = sin(theta - thetar);


  }

  virtual void updateStateJacobian(const Eigen::VectorXd& states, Eigen::MatrixXd& jx) const override {
    // Determine stage variables
    const auto& theta = states(3);
    const auto& thetar = params(3);
    const auto& thetarf = params(14);
    const auto& thetarr = params(17);
    const auto& lf = params(18);
    const auto& lr = params(19);

    // Determine internal variables
    const double internal_4 = cos(theta);
    const double internal_5 = sin(theta);
    const double internal_0 = sin(thetar);
    const double internal_1 = cos(thetar);
    const double internal_2 = sin(thetarf);
    const double internal_3 = cos(thetarf);
    const double internal_7 = sin(thetarr);
    const double internal_8 = cos(thetarr);
    const double internal_6 = internal_2*internal_5*lf + internal_3*internal_4*lf;
    const double internal_9 = internal_4*internal_8*lr + internal_5*internal_7*lr;

    // Evaluation of Matrix jx
    jx(0, 1) = internal_0;
    jx(0, 2) = -internal_1;
    jx(1, 1) = -internal_0;
    jx(1, 2) = internal_1;
    jx(2, 1) = internal_2;
    jx(2, 2) = -internal_3;
    jx(2, 3) = -internal_6;
    jx(3, 1) = -internal_2;
    jx(3, 2) = internal_3;
    jx(3, 3) = internal_6;
    jx(4, 1) = internal_7;
    jx(4, 2) = -internal_8;
    jx(4, 3) = internal_9;
    jx(5, 1) = -internal_7;
    jx(5, 2) = internal_8;
    jx(5, 3) = -internal_9;
    jx(6, 4) = -1;
    jx(7, 4) = 1;
    jx(8, 3) = sin(theta - thetar);

  }

  virtual void updateJacobianDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& jx) const override {
    // Determine stage variables
    const auto& theta = states(3);
    const auto& thetar = params(3);
    const auto& thetarf = params(14);
    const auto& thetarr = params(17);
    const auto& lf = params(18);
    const auto& lr = params(19);

    // Determine internal variables
    const double internal_4 = cos(theta);
    const double internal_5 = sin(theta);
    const double internal_0 = sin(thetar);
    const double internal_1 = cos(thetar);
    const double internal_2 = sin(thetarf);
    const double internal_3 = cos(thetarf);
    const double internal_7 = sin(thetarr);
    const double internal_8 = cos(thetarr);
    const double internal_6 = internal_2*internal_5*lf + internal_3*internal_4*lf;
    const double internal_9 = internal_4*internal_8*lr + internal_5*internal_7*lr;

    // Evaluation of Matrix jx
    jx(0, 1) = internal_0;
    jx(0, 2) = -internal_1;
    jx(1, 1) = -internal_0;
    jx(1, 2) = internal_1;
    jx(2, 1) = internal_2;
    jx(2, 2) = -internal_3;
    jx(2, 3) = -internal_6;
    jx(3, 1) = -internal_2;
    jx(3, 2) = internal_3;
    jx(3, 3) = internal_6;
    jx(4, 1) = internal_7;
    jx(4, 2) = -internal_8;
    jx(4, 3) = internal_9;
    jx(5, 1) = -internal_7;
    jx(5, 2) = internal_8;
    jx(5, 3) = -internal_9;
    jx(6, 4) = -1;
    jx(7, 4) = 1;
    jx(8, 3) = sin(theta - thetar);

  }

  virtual void updateDeltaEvaluation(const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::VectorXd& dx, const Eigen::VectorXd& du, Eigen::VectorXd& dg) const override {
    // Evaluation of Vector dg
    dg(0) = dx(1)*g_x(0, 1) + dx(2)*g_x(0, 2);
    dg(1) = dx(1)*g_x(1, 1) + dx(2)*g_x(1, 2);
    dg(2) = dx(1)*g_x(2, 1) + dx(2)*g_x(2, 2) + dx(3)*g_x(2, 3);
    dg(3) = dx(1)*g_x(3, 1) + dx(2)*g_x(3, 2) + dx(3)*g_x(3, 3);
    dg(4) = dx(1)*g_x(4, 1) + dx(2)*g_x(4, 2) + dx(3)*g_x(4, 3);
    dg(5) = dx(1)*g_x(5, 1) + dx(2)*g_x(5, 2) + dx(3)*g_x(5, 3);
    dg(6) = dx(4)*g_x(6, 4);
    dg(7) = dx(4)*g_x(7, 4);
    dg(8) = dx(3)*g_x(8, 3);

  }

};

template <StageType Ttype>
class LateralGeneralStateOnlyEqualities : public Constraints {
 public:
  LateralGeneralStateOnlyEqualities() : Constraints(5, 2, 0, 0, 0) {
  }

  virtual ~LateralGeneralStateOnlyEqualities() = default;

  virtual Type type() const override { return Type::STATE_ONLY_EQUALITY; }
};

template <>
class LateralGeneralStateOnlyEqualities<StageType::INITIAL> : public Constraints {
 public:
  LateralGeneralStateOnlyEqualities() : Constraints(5, 2, 5, 0, 0) {
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

  virtual ~LateralGeneralStateOnlyEqualities() = default;

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
class LateralGeneralStateOnlyEqualities<StageType::TERMINAL> : public Constraints {
 public:
  LateralGeneralStateOnlyEqualities() : Constraints(5, 2, 0, 0, 0) {
  }

  virtual ~LateralGeneralStateOnlyEqualities() = default;

  virtual Type type() const override { return Type::STATE_ONLY_EQUALITY; }
};

}  // namespace gpal::pnc::planning