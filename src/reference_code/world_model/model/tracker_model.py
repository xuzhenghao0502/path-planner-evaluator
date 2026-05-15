import sys

import numpy as np
import sympy as sym
from sympy import *

sys.path.append("../../pnc_solver/scripts/")

from core.ocp_model import *

file_path = os.path.abspath(__file__)
dir_path = os.path.dirname(file_path)
solver_path = os.path.dirname(dir_path)
gen_dir = solver_path + "/model/tracker_model/"
if len(sys.argv) > 1:

    gen_dir = sys.argv[1]

ocp_model = OptimalControlProblem("TrackerModel")

# define state symbols
# s 为投影到参考线上的匹配点的local s信息
s, x, y, theta, steer = ocp_model.add("state", "s, x, y, theta, steer")

# define control symbols
dsteer= ocp_model.add("ctrl", "dsteer")

# define slack symbols
l_slack = ocp_model.add("slack", "l_slack")

# reference line state params
x_ref, y_ref, theta_ref, kappa_ref = ocp_model.add(
    "param", "x_ref, y_ref, theta_ref, kappa_ref"
)

#  coarse state params s_coarse 为决策轨迹在参考线上的投影点的local s信息
v_ref = ocp_model.add("param", "v_ref")

#  terminal state params
l_terminal = ocp_model.add(
    "param", "l_terminal"
)


wheelbase, front_overhang, rear_overhang, length, width = ocp_model.add(
    "param",
    wheelbase=3.0,
    front_overhang=1.0,
    rear_overhang=1.0,
    length=5.0,
    width=2.0,
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

l_slack_weight = ocp_model.add("param", l_slack_weight=1.0)

terminal_l_weight, terminal_theta_weight = ocp_model.add(
    "param", terminal_l_weight=1.0, terminal_theta_weight=1.0
)

# constraints params
LHardLowerBound, LHardUpperBound = ocp_model.add(
    "param", LHardLowerBound=-20.0, LHardUpperBound=20.0
)

SteerLowerBound, SteerUpperBound = ocp_model.add(
    "param", SteerLowerBound=-0.436, SteerUpperBound=0.436
)  # 默认最大轮转角度为25度

DSteerLowerBound, DSteerUpperBound = ocp_model.add(
    "param", DSteerLowerBound=-0.436, DSteerUpperBound=0.436
)  # 默认最大轮转角速率为25度每秒



l = -sin(theta_ref) * (x - x_ref) + cos(theta_ref) * (y - y_ref)
ds = cos(theta - theta_ref) / (1 - kappa_ref * l)
kappa = tan(steer) / wheelbase

# define dynamics
ocp_model.subject(dot(s) == v_ref * ds)
ocp_model.subject(dot(x) == v_ref * cos(theta))
ocp_model.subject(dot(y) == v_ref * sin(theta))
ocp_model.subject(dot(theta) == v_ref * kappa)
ocp_model.subject(dot(steer) == dsteer)


(
    l_scale,
    theta_scale,
    steer_scale,
    dsteer_scale,
) = ocp_model.add(
    "param",
    l_scale=1.0,
    theta_scale=1.0,
    steer_scale=1.0,
    dsteer_scale=1.0,
)

# define cost
cost_x_lateral = (
    0.5 * l_ref_weight * (l / l_scale) ** 2
    + 0.5 * steer_weight * (steer / steer_scale) ** 2
    + 0.5 * theta_ref_weight * ((theta - theta_ref) / theta_scale) ** 2
)

cost_u = (
    0.5 * dsteer_weight * (dsteer / dsteer_scale) ** 2
)

cost_s = 0.5 * l_slack_weight * (l_slack / l_scale) ** 2 

ocp_model.add(
    "cost", cost_x_lateral, cost_u, cost_s
)

# define terminal cost
terminal_cost_x = (
    0.5 * terminal_l_weight * (l / l_scale) ** 2
    + 0.5 * terminal_theta_weight * ((theta - theta_ref) / theta_scale) ** 2
)
ocp_model.add("cost", terminal_cost_x, type=StageType.TERMINAL)

# define constraints

ocp_model.subject(LHardLowerBound <= l + l_slack, l - l_slack<= LHardUpperBound)

ocp_model.subject(SteerLowerBound <= steer, steer <= SteerUpperBound)

ocp_model.subject(DSteerLowerBound <= dsteer, dsteer <= DSteerUpperBound)

ocp_model.subject(cos(theta - theta_ref) >= 0.0)

ocp_model.generate(gen_dir)
