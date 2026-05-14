#pragma once

#include "ocp/constraints.h"

namespace gpal::pnc::planning {

template <StageType Ttype>
class ParkingGeneralConstraints : public Constraints {
 public:
  ParkingGeneralConstraints() : Constraints(6, 3, 11, 4, 6) {
    // Evaluation of Matrix jacobian_.jx
    jacobian_.jx(6, 4) = -1;
    jacobian_.jx(7, 4) = 1;
    jacobian_.jx(8, 5) = -1;
    jacobian_.jx(9, 5) = 1;

    // Evaluation of Matrix jacobian_.ju
    jacobian_.ju(11, 0) = -1;
    jacobian_.ju(12, 0) = 1;
    jacobian_.ju(13, 1) = -1;
    jacobian_.ju(14, 1) = 1;
    jacobian_.ju(15, 2) = -1;
    jacobian_.ju(16, 2) = -1;
    jacobian_.ju(17, 2) = -1;
    jacobian_.ju(18, 2) = -1;
    jacobian_.ju(19, 2) = -1;
    jacobian_.ju(20, 2) = -1;
    exprs_[0] = "ll <= -(x - xr)*sin(thetar) + (y - yr)*cos(thetar)";
    exprs_[1] = "-(x - xr)*sin(thetar) + (y - yr)*cos(thetar) <= lu";
    exprs_[2] = "lfl <= (lf*sin(theta) + y - yrf)*cos(thetarf) - (lf*cos(theta) + x - xrf)*sin(thetarf)";
    exprs_[3] = "(lf*sin(theta) + y - yrf)*cos(thetarf) - (lf*cos(theta) + x - xrf)*sin(thetarf) <= lfu";
    exprs_[4] = "lrl <= (-lr*sin(theta) + y - yrr)*cos(thetarr) - (-lr*cos(theta) + x - xrr)*sin(thetarr)";
    exprs_[5] = "(-lr*sin(theta) + y - yrr)*cos(thetarr) - (-lr*cos(theta) + x - xrr)*sin(thetarr) <= lru";
    exprs_[6] = "steer_lower <= steer";
    exprs_[7] = "steer <= steer_upper";
    exprs_[8] = "v_lower <= v";
    exprs_[9] = "v <= v_upper";
    exprs_[10] = "cos(theta - thetar) >= 0.0";
    exprs_[11] = "dsteer_lower <= dsteer";
    exprs_[12] = "dsteer <= dsteer_upper";
    exprs_[13] = "a_lower <= a";
    exprs_[14] = "a <= a_upper";
    exprs_[15] = "sll <= slack_offset - (x - xr)*sin(thetar) + (y - yr)*cos(thetar)";
    exprs_[16] = "-slack_offset - (x - xr)*sin(thetar) + (y - yr)*cos(thetar) <= slu";
    exprs_[17] = "slfl <= slack_offset + (lf*sin(theta) + y - yrf)*cos(thetarf) - (lf*cos(theta) + x - xrf)*sin(thetarf)";
    exprs_[18] = "-slack_offset + (lf*sin(theta) + y - yrf)*cos(thetarf) - (lf*cos(theta) + x - xrf)*sin(thetarf) <= slfu";
    exprs_[19] = "slrl <= slack_offset + (-lr*sin(theta) + y - yrr)*cos(thetarr) - (-lr*cos(theta) + x - xrr)*sin(thetarr)";
    exprs_[20] = "-slack_offset + (-lr*sin(theta) + y - yrr)*cos(thetarr) - (-lr*cos(theta) + x - xrr)*sin(thetarr) <= slru";

    idx_with_slack_.emplace(15);
    idx_with_slack_.emplace(16);
    idx_with_slack_.emplace(17);
    idx_with_slack_.emplace(18);
    idx_with_slack_.emplace(19);
    idx_with_slack_.emplace(20);
  }

  virtual ~ParkingGeneralConstraints() = default;

  virtual Type type() const override { return Type::INEQUALITY; }
  virtual void updateEvaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::VectorXd& values) const override {
    // Determine stage variables
    const auto& theta = states(3);
    const auto& steer = states(4);
    const auto& v = states(5);
    const auto& dsteer = ctrls(0);
    const auto& a = ctrls(1);
    const auto& slack_offset = ctrls(2);
    const auto& thetar = params(3);
    const auto& steer_lower = params(21);
    const auto& steer_upper = params(22);
    const auto& dsteer_lower = params(23);
    const auto& dsteer_upper = params(24);
    const auto& v_lower = params(25);
    const auto& v_upper = params(26);
    const auto& a_lower = params(27);
    const auto& a_upper = params(28);
    const auto& ll = params(31);
    const auto& lu = params(32);
    const auto& lfl = params(33);
    const auto& lfu = params(34);
    const auto& lrl = params(35);
    const auto& lru = params(36);
    const auto& sll = params(37);
    const auto& slu = params(38);
    const auto& slfl = params(39);
    const auto& slfu = params(40);
    const auto& slrl = params(41);
    const auto& slru = params(42);

    // Determine global variables
    const auto& l = globals(1);
    const auto& blf = globals(2);
    const auto& blr = globals(3);

    // Evaluation of Vector values
    values(0) = -l + ll;
    values(1) = l - lu;
    values(2) = -blf + lfl;
    values(3) = blf - lfu;
    values(4) = -blr + lrl;
    values(5) = blr - lru;
    values(6) = -steer + steer_lower;
    values(7) = steer - steer_upper;
    values(8) = -v + v_lower;
    values(9) = v - v_upper;
    values(10) = -cos(theta - thetar);
    values(11) = -dsteer + dsteer_lower;
    values(12) = dsteer - dsteer_upper;
    values(13) = -a + a_lower;
    values(14) = a - a_upper;
    values(15) = -l - slack_offset + sll;
    values(16) = l - slack_offset - slu;
    values(17) = -blf - slack_offset + slfl;
    values(18) = blf - slack_offset - slfu;
    values(19) = -blr - slack_offset + slrl;
    values(20) = blr - slack_offset - slru;

  }

  virtual void updateJacobian(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& jx, Eigen::MatrixXd& ju) const override {
    // Determine stage variables
    const auto& theta = states(3);
    const auto& thetar = params(3);
    const auto& thetarf = params(17);
    const auto& thetarr = params(20);
    const auto& lf = params(29);
    const auto& lr = params(30);

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
    jx(8, 5) = -1;
    jx(9, 5) = 1;
    jx(10, 3) = sin(theta - thetar);
    jx(15, 1) = internal_0;
    jx(15, 2) = internal_2;
    jx(16, 1) = internal_3;
    jx(16, 2) = internal_1;
    jx(17, 1) = internal_4;
    jx(17, 2) = internal_6;
    jx(17, 3) = internal_10;
    jx(18, 1) = internal_11;
    jx(18, 2) = internal_5;
    jx(18, 3) = internal_9;
    jx(19, 1) = internal_12;
    jx(19, 2) = internal_14;
    jx(19, 3) = internal_15;
    jx(20, 1) = internal_16;
    jx(20, 2) = internal_13;
    jx(20, 3) = internal_17;

    // Evaluation of Matrix ju
    ju(11, 0) = -1;
    ju(12, 0) = 1;
    ju(13, 1) = -1;
    ju(14, 1) = 1;
    ju(15, 2) = -1;
    ju(16, 2) = -1;
    ju(17, 2) = -1;
    ju(18, 2) = -1;
    ju(19, 2) = -1;
    ju(20, 2) = -1;

  }

  virtual void updateStateJacobian(const Eigen::VectorXd& states, Eigen::MatrixXd& jx) const override {
    // Determine stage variables
    const auto& theta = states(3);
    const auto& thetar = params(3);
    const auto& thetarf = params(17);
    const auto& thetarr = params(20);
    const auto& lf = params(29);
    const auto& lr = params(30);

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
    jx(8, 5) = -1;
    jx(9, 5) = 1;
    jx(10, 3) = sin(theta - thetar);

  }

  virtual void updateCtrlJacobian(const Eigen::VectorXd& ctrls, Eigen::MatrixXd& ju) const override {
    // Evaluation of Matrix ju
    ju(0, 0) = -1;
    ju(1, 0) = 1;
    ju(2, 1) = -1;
    ju(3, 1) = 1;

  }

  virtual void updateJacobianDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& jx) const override {
    // Determine stage variables
    const auto& theta = states(3);
    const auto& thetar = params(3);
    const auto& thetarf = params(17);
    const auto& thetarr = params(20);
    const auto& lf = params(29);
    const auto& lr = params(30);

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
    jx(8, 5) = -1;
    jx(9, 5) = 1;
    jx(10, 3) = sin(theta - thetar);
    jx(15, 1) = internal_0;
    jx(15, 2) = internal_2;
    jx(16, 1) = internal_3;
    jx(16, 2) = internal_1;
    jx(17, 1) = internal_4;
    jx(17, 2) = internal_6;
    jx(17, 3) = internal_10;
    jx(18, 1) = internal_11;
    jx(18, 2) = internal_5;
    jx(18, 3) = internal_9;
    jx(19, 1) = internal_12;
    jx(19, 2) = internal_14;
    jx(19, 3) = internal_15;
    jx(20, 1) = internal_16;
    jx(20, 2) = internal_13;
    jx(20, 3) = internal_17;

  }

  virtual void updateJacobianDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& ju) const override {
    // Evaluation of Matrix ju
    ju(11, 0) = -1;
    ju(12, 0) = 1;
    ju(13, 1) = -1;
    ju(14, 1) = 1;
    ju(15, 2) = -1;
    ju(16, 2) = -1;
    ju(17, 2) = -1;
    ju(18, 2) = -1;
    ju(19, 2) = -1;
    ju(20, 2) = -1;

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
    dg(8) = dx(5)*g_x(8, 5);
    dg(9) = dx(5)*g_x(9, 5);
    dg(10) = dx(3)*g_x(10, 3);
    dg(11) = du(0)*g_u(11, 0);
    dg(12) = du(0)*g_u(12, 0);
    dg(13) = du(1)*g_u(13, 1);
    dg(14) = du(1)*g_u(14, 1);
    dg(15) = du(2)*g_u(15, 2) + dx(1)*g_x(15, 1) + dx(2)*g_x(15, 2);
    dg(16) = du(2)*g_u(16, 2) + dx(1)*g_x(16, 1) + dx(2)*g_x(16, 2);
    dg(17) = du(2)*g_u(17, 2) + dx(1)*g_x(17, 1) + dx(2)*g_x(17, 2) + dx(3)*g_x(17, 3);
    dg(18) = du(2)*g_u(18, 2) + dx(1)*g_x(18, 1) + dx(2)*g_x(18, 2) + dx(3)*g_x(18, 3);
    dg(19) = du(2)*g_u(19, 2) + dx(1)*g_x(19, 1) + dx(2)*g_x(19, 2) + dx(3)*g_x(19, 3);
    dg(20) = du(2)*g_u(20, 2) + dx(1)*g_x(20, 1) + dx(2)*g_x(20, 2) + dx(3)*g_x(20, 3);

  }

};

template <>
class ParkingGeneralConstraints<StageType::INITIAL> : public Constraints {
 public:
  ParkingGeneralConstraints() : Constraints(6, 3, 0, 4, 0) {
    // Evaluation of Matrix jacobian_.ju
    jacobian_.ju(0, 0) = -1;
    jacobian_.ju(1, 0) = 1;
    jacobian_.ju(2, 1) = -1;
    jacobian_.ju(3, 1) = 1;
    exprs_[0] = "dsteer_lower <= dsteer";
    exprs_[1] = "dsteer <= dsteer_upper";
    exprs_[2] = "a_lower <= a";
    exprs_[3] = "a <= a_upper";

  }

  virtual ~ParkingGeneralConstraints() = default;

  virtual Type type() const override { return Type::INEQUALITY; }
  virtual void updateEvaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::VectorXd& values) const override {
    // Determine stage variables
    const auto& dsteer = ctrls(0);
    const auto& a = ctrls(1);
    const auto& dsteer_lower = params(23);
    const auto& dsteer_upper = params(24);
    const auto& a_lower = params(27);
    const auto& a_upper = params(28);

    // Evaluation of Vector values
    values(0) = -dsteer + dsteer_lower;
    values(1) = dsteer - dsteer_upper;
    values(2) = -a + a_lower;
    values(3) = a - a_upper;

  }

  virtual void updateJacobian(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& jx, Eigen::MatrixXd& ju) const override {
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

  virtual void updateJacobianDu(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& ju) const override {
    // Evaluation of Matrix ju
    ju(0, 0) = -1;
    ju(1, 0) = 1;
    ju(2, 1) = -1;
    ju(3, 1) = 1;

  }

  virtual void updateDeltaEvaluation(const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::VectorXd& dx, const Eigen::VectorXd& du, Eigen::VectorXd& dg) const override {
    // Evaluation of Vector dg
    dg(0) = du(0)*g_u(0, 0);
    dg(1) = du(0)*g_u(1, 0);
    dg(2) = du(1)*g_u(2, 1);
    dg(3) = du(1)*g_u(3, 1);

  }

};

template <>
class ParkingGeneralConstraints<StageType::TERMINAL> : public Constraints {
 public:
  ParkingGeneralConstraints() : Constraints(6, 3, 11, 0, 0) {
    // Evaluation of Matrix jacobian_.jx
    jacobian_.jx(6, 4) = -1;
    jacobian_.jx(7, 4) = 1;
    jacobian_.jx(8, 5) = -1;
    jacobian_.jx(9, 5) = 1;

    exprs_[0] = "ll <= -(x - xr)*sin(thetar) + (y - yr)*cos(thetar)";
    exprs_[1] = "-(x - xr)*sin(thetar) + (y - yr)*cos(thetar) <= lu";
    exprs_[2] = "lfl <= (lf*sin(theta) + y - yrf)*cos(thetarf) - (lf*cos(theta) + x - xrf)*sin(thetarf)";
    exprs_[3] = "(lf*sin(theta) + y - yrf)*cos(thetarf) - (lf*cos(theta) + x - xrf)*sin(thetarf) <= lfu";
    exprs_[4] = "lrl <= (-lr*sin(theta) + y - yrr)*cos(thetarr) - (-lr*cos(theta) + x - xrr)*sin(thetarr)";
    exprs_[5] = "(-lr*sin(theta) + y - yrr)*cos(thetarr) - (-lr*cos(theta) + x - xrr)*sin(thetarr) <= lru";
    exprs_[6] = "steer_lower <= steer";
    exprs_[7] = "steer <= steer_upper";
    exprs_[8] = "v_lower <= v";
    exprs_[9] = "v <= v_upper";
    exprs_[10] = "cos(theta - thetar) >= 0.0";

  }

  virtual ~ParkingGeneralConstraints() = default;

  virtual Type type() const override { return Type::INEQUALITY; }
  virtual void updateEvaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::VectorXd& values) const override {
    // Determine stage variables
    const auto& theta = states(3);
    const auto& steer = states(4);
    const auto& v = states(5);
    const auto& thetar = params(3);
    const auto& steer_lower = params(21);
    const auto& steer_upper = params(22);
    const auto& v_lower = params(25);
    const auto& v_upper = params(26);
    const auto& ll = params(31);
    const auto& lu = params(32);
    const auto& lfl = params(33);
    const auto& lfu = params(34);
    const auto& lrl = params(35);
    const auto& lru = params(36);

    // Determine global variables
    const auto& l = globals(1);
    const auto& blf = globals(2);
    const auto& blr = globals(3);

    // Evaluation of Vector values
    values(0) = -l + ll;
    values(1) = l - lu;
    values(2) = -blf + lfl;
    values(3) = blf - lfu;
    values(4) = -blr + lrl;
    values(5) = blr - lru;
    values(6) = -steer + steer_lower;
    values(7) = steer - steer_upper;
    values(8) = -v + v_lower;
    values(9) = v - v_upper;
    values(10) = -cos(theta - thetar);

  }

  virtual void updateJacobian(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& jx, Eigen::MatrixXd& ju) const override {
    // Determine stage variables
    const auto& theta = states(3);
    const auto& thetar = params(3);
    const auto& thetarf = params(17);
    const auto& thetarr = params(20);
    const auto& lf = params(29);
    const auto& lr = params(30);

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
    jx(8, 5) = -1;
    jx(9, 5) = 1;
    jx(10, 3) = sin(theta - thetar);


  }

  virtual void updateStateJacobian(const Eigen::VectorXd& states, Eigen::MatrixXd& jx) const override {
    // Determine stage variables
    const auto& theta = states(3);
    const auto& thetar = params(3);
    const auto& thetarf = params(17);
    const auto& thetarr = params(20);
    const auto& lf = params(29);
    const auto& lr = params(30);

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
    jx(8, 5) = -1;
    jx(9, 5) = 1;
    jx(10, 3) = sin(theta - thetar);

  }

  virtual void updateJacobianDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& jx) const override {
    // Determine stage variables
    const auto& theta = states(3);
    const auto& thetar = params(3);
    const auto& thetarf = params(17);
    const auto& thetarr = params(20);
    const auto& lf = params(29);
    const auto& lr = params(30);

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
    jx(8, 5) = -1;
    jx(9, 5) = 1;
    jx(10, 3) = sin(theta - thetar);

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
    dg(8) = dx(5)*g_x(8, 5);
    dg(9) = dx(5)*g_x(9, 5);
    dg(10) = dx(3)*g_x(10, 3);

  }

};

template <StageType Ttype>
class ParkingGeneralStateOnlyEqualities : public Constraints {
 public:
  ParkingGeneralStateOnlyEqualities() : Constraints(6, 3, 0, 0, 0) {
  }

  virtual ~ParkingGeneralStateOnlyEqualities() = default;

  virtual Type type() const override { return Type::STATE_ONLY_EQUALITY; }
};

template <>
class ParkingGeneralStateOnlyEqualities<StageType::INITIAL> : public Constraints {
 public:
  ParkingGeneralStateOnlyEqualities() : Constraints(6, 3, 6, 0, 0) {
    // Evaluation of SymmetricMatrix jacobian_.jx
    jacobian_.jx(0, 0) = 1;
    jacobian_.jx(1, 1) = 1;
    jacobian_.jx(2, 2) = 1;
    jacobian_.jx(3, 3) = 1;
    jacobian_.jx(4, 4) = 1;
    jacobian_.jx(5, 5) = 1;

    exprs_[0] = "s = 0";
    exprs_[1] = "x = 0";
    exprs_[2] = "y = 0";
    exprs_[3] = "theta = 0";
    exprs_[4] = "steer = 0";
    exprs_[5] = "v = 0";
  }

  virtual ~ParkingGeneralStateOnlyEqualities() = default;

  virtual Type type() const override { return Type::STATE_ONLY_EQUALITY; }
  virtual void updateEvaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::VectorXd& values) const override {
    // Determine stage variables
    const auto& s = states(0);
    const auto& x = states(1);
    const auto& y = states(2);
    const auto& theta = states(3);
    const auto& steer = states(4);
    const auto& v = states(5);

    // Evaluation of Vector values
    values(0) = s;
    values(1) = x;
    values(2) = y;
    values(3) = theta;
    values(4) = steer;
    values(5) = v;

  }

  virtual void updateJacobian(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& jx, Eigen::MatrixXd& ju) const override {
    // Evaluation of Matrix jx
    jx(0, 0) = 1;
    jx(1, 1) = 1;
    jx(2, 2) = 1;
    jx(3, 3) = 1;
    jx(4, 4) = 1;
    jx(5, 5) = 1;


  }

  virtual void updateStateJacobian(const Eigen::VectorXd& states, Eigen::MatrixXd& jx) const override {
    // Evaluation of Matrix jx
    jx(0, 0) = 1;
    jx(1, 1) = 1;
    jx(2, 2) = 1;
    jx(3, 3) = 1;
    jx(4, 4) = 1;
    jx(5, 5) = 1;

  }

  virtual void updateJacobianDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& jx) const override {
    // Evaluation of Matrix jx
    jx(0, 0) = 1;
    jx(1, 1) = 1;
    jx(2, 2) = 1;
    jx(3, 3) = 1;
    jx(4, 4) = 1;
    jx(5, 5) = 1;

  }

  virtual void updateDeltaEvaluation(const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::VectorXd& dx, const Eigen::VectorXd& du, Eigen::VectorXd& dg) const override {
    // Evaluation of Vector dg
    dg(0) = dx(0)*g_x(0, 0);
    dg(1) = dx(1)*g_x(1, 1);
    dg(2) = dx(2)*g_x(2, 2);
    dg(3) = dx(3)*g_x(3, 3);
    dg(4) = dx(4)*g_x(4, 4);
    dg(5) = dx(5)*g_x(5, 5);

  }

};

template <>
class ParkingGeneralStateOnlyEqualities<StageType::TERMINAL> : public Constraints {
 public:
  ParkingGeneralStateOnlyEqualities() : Constraints(6, 3, 3, 0, 0) {
    // Evaluation of Matrix jacobian_.jx
    jacobian_.jx(0, 1) = 1;
    jacobian_.jx(1, 2) = 1;
    jacobian_.jx(2, 3) = 1;

    exprs_[0] = "x - xr = 0";
    exprs_[1] = "y - yr = 0";
    exprs_[2] = "theta - thetar = 0";
  }

  virtual ~ParkingGeneralStateOnlyEqualities() = default;

  virtual Type type() const override { return Type::STATE_ONLY_EQUALITY; }
  virtual void updateEvaluate(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::VectorXd& values) const override {
    // Determine stage variables
    const auto& x = states(1);
    const auto& y = states(2);
    const auto& theta = states(3);
    const auto& xr = params(1);
    const auto& yr = params(2);
    const auto& thetar = params(3);

    // Evaluation of Vector values
    values(0) = x - xr;
    values(1) = y - yr;
    values(2) = theta - thetar;

  }

  virtual void updateJacobian(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& jx, Eigen::MatrixXd& ju) const override {
    // Evaluation of Matrix jx
    jx(0, 1) = 1;
    jx(1, 2) = 1;
    jx(2, 3) = 1;


  }

  virtual void updateStateJacobian(const Eigen::VectorXd& states, Eigen::MatrixXd& jx) const override {
    // Evaluation of Matrix jx
    jx(0, 1) = 1;
    jx(1, 2) = 1;
    jx(2, 3) = 1;

  }

  virtual void updateJacobianDx(const Eigen::VectorXd& states, const Eigen::VectorXd& ctrls, Eigen::MatrixXd& jx) const override {
    // Evaluation of Matrix jx
    jx(0, 1) = 1;
    jx(1, 2) = 1;
    jx(2, 3) = 1;

  }

  virtual void updateDeltaEvaluation(const Eigen::MatrixXd& g_x, const Eigen::MatrixXd& g_u, const Eigen::VectorXd& dx, const Eigen::VectorXd& du, Eigen::VectorXd& dg) const override {
    // Evaluation of Vector dg
    dg(0) = dx(1)*g_x(0, 1);
    dg(1) = dx(2)*g_x(1, 2);
    dg(2) = dx(3)*g_x(2, 3);

  }

};

}  // namespace gpal::pnc::planning