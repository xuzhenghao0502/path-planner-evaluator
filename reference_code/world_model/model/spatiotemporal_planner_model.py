import sys

import numpy as np
import sympy as sym
from sympy import *

sys.path.append("../../pnc_solver/scripts/")

from core.ocp_model import *

file_path = os.path.abspath(__file__)
dir_path = os.path.dirname(file_path)
solver_path = os.path.dirname(dir_path)
gen_dir = solver_path + "/model/spatiotemporal_planner_model/"
if len(sys.argv) > 1:

    gen_dir = sys.argv[1]

ocp_model = OptimalControlProblem("SpatiotemporalPlannerModel")

epsilon = 1e-4
# define state symbols
# s 为投影到参考线上的匹配点的local s信息
s, x, y, theta, steer, v, a = ocp_model.add("state", "s, x, y, theta, steer, v, a")

# define control symbols
dsteer, jerk = ocp_model.add("ctrl", "dsteer, jerk")

# define slack symbols
l_slack, s_slack, v_slack = ocp_model.add("slack", "l_slack, s_slack, v_slack")

(
    obs_0_slack,
    obs_1_slack,
    obs_2_slack,
    obs_3_slack,
    obs_4_slack,
    obs_5_slack,
    obs_6_slack,
    obs_7_slack,
) = ocp_model.add(
    "slack",
    "obs_0_slack, obs_1_slack, obs_2_slack, obs_3_slack, obs_4_slack, obs_5_slack, obs_6_slack, obs_7_slack",
)

# reference line state params
x_ref, y_ref, theta_ref, kappa_ref, steer_ref = ocp_model.add(
    "param", "x_ref, y_ref, theta_ref, kappa_ref, steer_ref"
)

#  coarse state params s_coarse 为决策轨迹在参考线上的投影点的local s信息
s_coarse, v_coarse, a_coarse = ocp_model.add("param", "s_coarse, v_coarse, a_coarse")

#  terminal state params
l_terminal, theta_terminal, s_terminal, v_terminal = ocp_model.add(
    "param", "l_terminal, theta_terminal, s_terminal, v_terminal"
)


l_offset, a_offset = ocp_model.add(
    "param", l_offset=0.0, a_offset=0.0
)

wheelbase, front_overhang, rear_overhang, length, width = ocp_model.add(
    "param",
    wheelbase=3.0,
    front_overhang=1.0,
    rear_overhang=1.0,
    length=5.0,
    width=2.0,
)

# cost params

weight_prev_steer, prev_steer = ocp_model.add(
    "param",
    weight_prev_steer=1.0,
    prev_steer=1.0,
)

l_ref_weight, theta_ref_weight, steer_weight, dsteer_weight = (
    ocp_model.add(
        "param",
        l_ref_weight=1.0,
        theta_ref_weight=1.0,
        steer_weight=1.0,
        dsteer_weight=1.0,
    )
)

s_coarse_weight, v_coarse_weight, a_weight, jerk_weight = ocp_model.add(
    "param", s_coarse_weight=0.0, v_coarse_weight=1.0, a_weight=1.0, jerk_weight=1.0
)

obs_0_weight, obs_1_weight, obs_2_weight, obs_3_weight, obs_4_weight, obs_5_weight, obs_6_weight, obs_7_weight = ocp_model.add(
    "param", obs_0_weight=1.0, obs_1_weight=1.0, obs_2_weight=1.0, obs_3_weight=1.0, obs_4_weight=1.0, obs_5_weight=1.0, obs_6_weight=1.0, obs_7_weight=1.0
)

l_slack_weight, s_slack_weight, v_slack_weight = ocp_model.add(
    "param", l_slack_weight=1.0, s_slack_weight=1.0, v_slack_weight=1.0
)

terminal_l_weight, terminal_theta_weight = ocp_model.add(
    "param", terminal_l_weight=1.0, terminal_theta_weight=1.0
)

terminal_s_weight, terminal_v_weight, terminal_a_weight = ocp_model.add(
    "param", terminal_s_weight=1.0, terminal_v_weight=1.0, terminal_a_weight=1.0
)


# constraints params
# 车辆前后端中心点相对于参考线的匹配点信息
xf_ref, yf_ref, thetaf_ref = ocp_model.add("param", "xf_ref, yf_ref, thetaf_ref")

xr_ref, yr_ref, thetar_ref = ocp_model.add("param", "xr_ref, yr_ref, thetar_ref")

Obs_0_AvoidedBound, Obs_1_AvoidedBound, Obs_2_AvoidedBound, Obs_3_AvoidedBound, Obs_4_AvoidedBound, Obs_5_AvoidedBound, Obs_6_AvoidedBound, Obs_7_AvoidedBound = (
    ocp_model.add(
        "param",
        Obs_0_AvoidedBound=1.0,
        Obs_1_AvoidedBound=1.0,
        Obs_2_AvoidedBound=1.0,
        Obs_3_AvoidedBound=1.0,
        Obs_4_AvoidedBound=1.0,
        Obs_5_AvoidedBound=1.0,
        Obs_6_AvoidedBound=1.0,
        Obs_7_AvoidedBound=1.0
    )
)


LHardLowerBound, LHardUpperBound = ocp_model.add(
    "param", LHardLowerBound=-20.0, LHardUpperBound=20.0
)

LFHardLowerBound, LFHardUpperBound = ocp_model.add(
    "param", LFHardLowerBound=-20.0, LFHardUpperBound=20.0
)

LRHardLowerBound, LRHardUpperBound = ocp_model.add(
    "param", LRHardLowerBound=-20.0, LRHardUpperBound=20.0
)

LSoftLowerBound, LSoftUpperBound = ocp_model.add(
    "param", LSoftLowerBound=-20.0, LSoftUpperBound=20.0
)

LFSoftLowerBound, LFSoftUpperBound = ocp_model.add(
    "param", LFSoftLowerBound=-20.0, LFSoftUpperBound=20.0
)

LRSoftLowerBound, LRSoftUpperBound = ocp_model.add(
    "param", LRSoftLowerBound=-20.0, LRSoftUpperBound=20.0
)

SteerLowerBound, SteerUpperBound = ocp_model.add(
    "param", SteerLowerBound=-0.436, SteerUpperBound=0.436
)  # 默认最大轮转角度为25度

DSteerLowerBound, DSteerUpperBound = ocp_model.add(
    "param", DSteerLowerBound=-0.436, DSteerUpperBound=0.436
)  # 默认最大轮转角速率为25度每秒


SHardLowerBound, SHardUpperBound = ocp_model.add(
    "param", SHardLowerBound=0.0, SHardUpperBound=300.0
)

VHardLowerBound, VHardUpperBound = ocp_model.add(
    "param", VHardLowerBound=0.0, VHardUpperBound=40.0
)

AHardLowerBound, AHardUpperBound = ocp_model.add(
    "param", AHardLowerBound=-6.0, AHardUpperBound=3.0
)

JHardLowerBound, JHardUpperBound = ocp_model.add(
    "param", JHardLowerBound=-8.0, JHardUpperBound=3.0
)

SSoftLowerBound, SSoftUpperBound = ocp_model.add(
    "param", SSoftLowerBound=0.0, SSoftUpperBound=300.0
)

VSoftLowerBound, VSoftUpperBound = ocp_model.add(
    "param", VSoftLowerBound=0.0, VSoftUpperBound=40.0
)

ASoftLowerBound, ASoftUpperBound = ocp_model.add(
    "param", ASoftLowerBound=-6.0, ASoftUpperBound=3.0
)

# 车辆前后端中心点相对于参考线的匹配点的横向偏差
blf = -sin(thetaf_ref) * (x + (front_overhang + wheelbase) * cos(theta) - xf_ref) + cos(
    thetaf_ref
) * (y + (front_overhang + wheelbase) * sin(theta) - yf_ref)

blr = -sin(thetar_ref) * (x - rear_overhang * cos(theta) - xr_ref) + cos(thetar_ref) * (
    y - rear_overhang * sin(theta) - yr_ref
)


# 车辆头部中心坐标
x_f_circle = x + (wheelbase + front_overhang) * cos(theta)

y_f_circle = y + (wheelbase + front_overhang) * sin(theta)

# 车辆尾部中心坐标
x_r_circle = x - rear_overhang * cos(theta)

y_r_circle = y - rear_overhang * sin(theta)


# 计算点到当前障碍物椭圆的归一化距离
def compute_ellipse_distance(
    point_x, point_y, a_square, b_square, x_obs, y_obs, cos_theta_obs, sin_theta_obs
):
    dx = point_x - x_obs
    dy = point_y - y_obs
    x_local = dx * cos_theta_obs + dy * sin_theta_obs
    y_local = -dx * sin_theta_obs + dy * cos_theta_obs
    return (x_local**2) / a_square + (y_local**2) / b_square


(
    a_0,
    a_0_square,
    b_0,
    b_0_square,
    x_0,
    y_0,
    theta_0,
    cos_theta_0,
    sin_theta_0,
    v_0,
    length_0,
    width_0,
) = ocp_model.add(
    "param",
    a_0=5.0,
    a_0_square=25.0,
    b_0=2.0,
    b_0_square=4.0,
    x_0=0.0,
    y_0=0.0,
    theta_0=0.0,
    cos_theta_0=1.0,
    sin_theta_0=0.0,
    v_0=0.0,
    length_0=5.0,
    width_0=2.0,
)

(
    a_1,
    a_1_square,
    b_1,
    b_1_square,
    x_1,
    y_1,
    theta_1,
    cos_theta_1,
    sin_theta_1,
    v_1,
    length_1,
    width_1,
) = ocp_model.add(
    "param",
    a_1=5.0,
    a_1_square=25.0,
    b_1=2.0,
    b_1_square=4.0,
    x_1=0.0,
    y_1=0.0,
    theta_1=0.0,
    cos_theta_1=1.0,
    sin_theta_1=0.0,
    v_1=0.0,
    length_1=5.0,
    width_1=2.0,
)

(
    a_2,
    a_2_square,
    b_2,
    b_2_square,
    x_2,
    y_2,
    theta_2,
    cos_theta_2,
    sin_theta_2,
    v_2,
    length_2,
    width_2,
) = ocp_model.add(
    "param",
    a_2=5.0,
    a_2_square=25.0,
    b_2=2.0,
    b_2_square=4.0,
    x_2=0.0,
    y_2=0.0,
    theta_2=0.0,
    cos_theta_2=1.0,
    sin_theta_2=0.0,
    v_2=0.0,
    length_2=5.0,
    width_2=2.0,
)

(
    a_3,
    a_3_square,
    b_3,
    b_3_square,
    x_3,
    y_3,
    theta_3,
    cos_theta_3,
    sin_theta_3,
    v_3,
    length_3,
    width_3,
) = ocp_model.add(
    "param",
    a_3=5.0,
    a_3_square=25.0,
    b_3=2.0,
    b_3_square=4.0,
    x_3=0.0,
    y_3=0.0,
    theta_3=0.0,
    cos_theta_3=1.0,
    sin_theta_3=0.0,
    v_3=0.0,
    length_3=5.0,
    width_3=2.0,
)

(
    a_4,
    a_4_square,
    b_4,
    b_4_square,
    x_4,
    y_4,
    theta_4,
    cos_theta_4,
    sin_theta_4,
    v_4,
    length_4,
    width_4,
) = ocp_model.add(
    "param",
    a_4=5.0,
    a_4_square=25.0,
    b_4=2.0,
    b_4_square=4.0,
    x_4=0.0,
    y_4=0.0,
    theta_4=0.0,
    cos_theta_4=1.0,
    sin_theta_4=0.0,
    v_4=0.0,
    length_4=5.0,
    width_4=2.0,
)

(
    a_5,
    a_5_square,
    b_5,
    b_5_square,
    x_5,
    y_5,
    theta_5,
    cos_theta_5,
    sin_theta_5,
    v_5,
    length_5,
    width_5,
) = ocp_model.add(
    "param",
    a_5=5.0,
    a_5_square=25.0,
    b_5=2.0,
    b_5_square=4.0,
    x_5=0.0,
    y_5=0.0,
    theta_5=0.0,
    cos_theta_5=1.0,
    sin_theta_5=0.0,
    v_5=0.0,
    length_5=5.0,
    width_5=2.0,
)

(
    a_6,
    a_6_square,
    b_6,
    b_6_square,
    x_6,
    y_6,
    theta_6,
    cos_theta_6,
    sin_theta_6,
    v_6,
    length_6,
    width_6,
) = ocp_model.add(
    "param",
    a_6=5.0,
    a_6_square=25.0,
    b_6=2.0,
    b_6_square=4.0,
    x_6=0.0,
    y_6=0.0,
    theta_6=0.0,
    cos_theta_6=1.0,
    sin_theta_6=0.0,
    v_6=0.0,
    length_6=5.0,
    width_6=2.0,
)

(
    a_7,
    a_7_square,
    b_7,
    b_7_square,
    x_7,
    y_7,
    theta_7,
    cos_theta_7,
    sin_theta_7,
    v_7,
    length_7,
    width_7,
) = ocp_model.add(
    "param",
    a_7=5.0,
    a_7_square=25.0,
    b_7=2.0,
    b_7_square=4.0,
    x_7=0.0,
    y_7=0.0,
    theta_7=0.0,
    cos_theta_7=1.0,
    sin_theta_7=0.0,
    v_7=0.0,
    length_7=5.0,
    width_7=2.0,
)



f_dis_2_obs_0 = compute_ellipse_distance(
    x_f_circle, y_f_circle, a_0_square, b_0_square, x_0, y_0, cos_theta_0, sin_theta_0
)
r_dis_2_obs_0 = compute_ellipse_distance(
    x_r_circle, y_r_circle, a_0_square, b_0_square, x_0, y_0, cos_theta_0, sin_theta_0
)

f_dis_2_obs_1 = compute_ellipse_distance(
    x_f_circle, y_f_circle, a_1_square, b_1_square, x_1, y_1, cos_theta_1, sin_theta_1
)
r_dis_2_obs_1 = compute_ellipse_distance(
    x_r_circle, y_r_circle, a_1_square, b_1_square, x_1, y_1, cos_theta_1, sin_theta_1
)


f_dis_2_obs_2 = compute_ellipse_distance(
    x_f_circle, y_f_circle, a_2_square, b_2_square, x_2, y_2, cos_theta_2, sin_theta_2
)
r_dis_2_obs_2 = compute_ellipse_distance(
    x_r_circle, y_r_circle, a_2_square, b_2_square, x_2, y_2, cos_theta_2, sin_theta_2
)


f_dis_2_obs_3 = compute_ellipse_distance(
    x_f_circle, y_f_circle, a_3_square, b_3_square, x_3, y_3, cos_theta_3, sin_theta_3
)
r_dis_2_obs_3 = compute_ellipse_distance(
    x_r_circle, y_r_circle, a_3_square, b_3_square, x_3, y_3, cos_theta_3, sin_theta_3
)

f_dis_2_obs_4 = compute_ellipse_distance(
    x_f_circle, y_f_circle, a_4_square, b_4_square, x_4, y_4, cos_theta_4, sin_theta_4
)
r_dis_2_obs_4 = compute_ellipse_distance(
    x_r_circle, y_r_circle, a_4_square, b_4_square, x_4, y_4, cos_theta_4, sin_theta_4
)

f_dis_2_obs_5 = compute_ellipse_distance(
    x_f_circle, y_f_circle, a_5_square, b_5_square, x_5, y_5, cos_theta_5, sin_theta_5
)
r_dis_2_obs_5 = compute_ellipse_distance(
    x_r_circle, y_r_circle, a_5_square, b_5_square, x_5, y_5, cos_theta_5, sin_theta_5
)

f_dis_2_obs_6 = compute_ellipse_distance(
    x_f_circle, y_f_circle, a_6_square, b_6_square, x_6, y_6, cos_theta_6, sin_theta_6
)
r_dis_2_obs_6 = compute_ellipse_distance(
    x_r_circle, y_r_circle, a_6_square, b_6_square, x_6, y_6, cos_theta_6, sin_theta_6
)

f_dis_2_obs_7 = compute_ellipse_distance(
    x_f_circle, y_f_circle, a_7_square, b_7_square, x_7, y_7, cos_theta_7, sin_theta_7
)
r_dis_2_obs_7 = compute_ellipse_distance(
    x_r_circle, y_r_circle, a_7_square, b_7_square, x_7, y_7, cos_theta_7, sin_theta_7
)


l = -sin(theta_ref) * (x - x_ref) + cos(theta_ref) * (y - y_ref)
ds = cos(theta - theta_ref) / (1 - kappa_ref * l)
kappa = tan(steer) / wheelbase

# define dynamics
ocp_model.subject(dot(s) == v * ds)
ocp_model.subject(dot(x) == v * cos(theta))
ocp_model.subject(dot(y) == v * sin(theta))
ocp_model.subject(dot(theta) == v * kappa)
ocp_model.subject(dot(steer) == dsteer)
ocp_model.subject(dot(v) == a)
ocp_model.subject(dot(a) == jerk)


(
    s_scale,
    v_scale,
    a_scale,
    jerk_scale,
    l_scale,
    theta_scale,
    steer_scale,
    dsteer_scale,
) = ocp_model.add(
    "param",
    s_scale=20.0,
    v_scale=10.0,
    a_scale=5.0,
    jerk_scale=5.0,
    l_scale=1.0,
    theta_scale=1.0,
    steer_scale=1.0,
    dsteer_scale=1.0,
)

# define cost
cost_x_lateral = (
    0.5 * l_ref_weight * ((l - l_offset) / l_scale) ** 2
    + 0.5 * steer_weight * ((steer - steer_ref) / steer_scale) ** 2
    + 0.5 * theta_ref_weight * ((theta - theta_ref) / theta_scale) ** 2
)
cost_x_longitudinal = (
    0.5 * s_coarse_weight * ((s - s_coarse) / s_scale) ** 2
    + 0.5 * v_coarse_weight * ((v - v_coarse) / v_scale) ** 2
    + 0.5 * a_weight * ((a - a_offset) / a_scale) ** 2
)

cost_u = (
    0.5 * dsteer_weight * (dsteer / dsteer_scale) ** 2
    + 0.5 * jerk_weight * (jerk / jerk_scale) ** 2
)
cost_prev = 0.5 * weight_prev_steer * (steer - prev_steer) ** 2

cost_s = (
    0.5 * l_slack_weight * l_slack**2
    + 0.5 * s_slack_weight * s_slack**2
    + 0.5 * v_slack_weight * v_slack**2
    + 0.5 * obs_0_weight * obs_0_slack**2
    + 0.5 * obs_1_weight * obs_1_slack**2
    + 0.5 * obs_2_weight * obs_2_slack**2
    + 0.5 * obs_3_weight * obs_3_slack**2
    + 0.5 * obs_4_weight * obs_4_slack**2
    + 0.5 * obs_5_weight * obs_5_slack**2
    + 0.5 * obs_6_weight * obs_6_slack**2
    + 0.5 * obs_7_weight * obs_7_slack**2
)
ocp_model.add(
    "cost", cost_x_lateral, cost_x_longitudinal, cost_u, cost_prev, cost_s
)

# define terminal cost
terminal_cost_x = (
    0.5 * terminal_l_weight * ((l - l_terminal) / l_scale) ** 2
    + 0.5 * terminal_theta_weight * ((theta - theta_terminal) / theta_scale) ** 2
    + 0.5 * terminal_s_weight * ((s - s_terminal) / s_scale) ** 2
    + 0.5 * terminal_v_weight * ((v - v_terminal) / v_scale) ** 2
)
ocp_model.add("cost", terminal_cost_x, type=StageType.TERMINAL)


# define constraints

ocp_model.subject(Obs_0_AvoidedBound <= f_dis_2_obs_0 + obs_0_slack)

ocp_model.subject(Obs_1_AvoidedBound <= f_dis_2_obs_1 + obs_1_slack)

ocp_model.subject(Obs_2_AvoidedBound <= f_dis_2_obs_2 + obs_2_slack)

ocp_model.subject(Obs_3_AvoidedBound <= f_dis_2_obs_3 + obs_3_slack)

ocp_model.subject(Obs_4_AvoidedBound <= f_dis_2_obs_4 + obs_4_slack)

ocp_model.subject(Obs_5_AvoidedBound <= f_dis_2_obs_5 + obs_5_slack)

ocp_model.subject(Obs_6_AvoidedBound <= f_dis_2_obs_6 + obs_6_slack)

ocp_model.subject(Obs_7_AvoidedBound <= f_dis_2_obs_7 + obs_7_slack)

ocp_model.subject(Obs_0_AvoidedBound <= r_dis_2_obs_0 + obs_0_slack)

ocp_model.subject(Obs_1_AvoidedBound <= r_dis_2_obs_1 + obs_1_slack)

ocp_model.subject(Obs_2_AvoidedBound <= r_dis_2_obs_2 + obs_2_slack)

ocp_model.subject(Obs_3_AvoidedBound <= r_dis_2_obs_3 + obs_3_slack)

ocp_model.subject(Obs_4_AvoidedBound <= r_dis_2_obs_4 + obs_4_slack)

ocp_model.subject(Obs_5_AvoidedBound <= r_dis_2_obs_5 + obs_5_slack)

ocp_model.subject(Obs_6_AvoidedBound <= r_dis_2_obs_6 + obs_6_slack)

ocp_model.subject(Obs_7_AvoidedBound <= r_dis_2_obs_7 + obs_7_slack)


ocp_model.subject(LHardLowerBound <= l, l <= LHardUpperBound)

ocp_model.subject(LFHardLowerBound <= blf, blf <= LFHardUpperBound)

ocp_model.subject(LRHardLowerBound <= blr, blr <= LRHardUpperBound)

ocp_model.subject(LSoftLowerBound <= l + l_slack, l - l_slack <= LSoftUpperBound)

ocp_model.subject(LFSoftLowerBound <= blf + l_slack, blf - l_slack <= LFSoftUpperBound)

ocp_model.subject(LRSoftLowerBound <= blr + l_slack, blr - l_slack <= LRSoftUpperBound)

ocp_model.subject(SteerLowerBound <= steer, steer <= SteerUpperBound)

ocp_model.subject(DSteerLowerBound <= dsteer, dsteer <= DSteerUpperBound)

ocp_model.subject(SHardLowerBound <= s, s <= SHardUpperBound)

ocp_model.subject(s - s_slack <= SSoftUpperBound)

ocp_model.subject(VHardLowerBound <= v, v <= VHardUpperBound)

ocp_model.subject(v - v_slack <= VSoftUpperBound)

ocp_model.subject(AHardLowerBound <= a, a <= AHardUpperBound)

ocp_model.subject(JHardLowerBound <= jerk, jerk <= JHardUpperBound)

ocp_model.subject(cos(theta - theta_ref) >= 0.0)

ocp_model.generate(gen_dir)
