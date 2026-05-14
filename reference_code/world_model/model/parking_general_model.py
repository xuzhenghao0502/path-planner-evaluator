# -*- coding: utf-8 -*-
import sys
import sympy as sym
from sympy import *
import numpy as np

sys.path.append("../../pnc_solver/scripts/")
from core.ocp_model import *

file_path = os.path.abspath(__file__)
dir_path = os.path.dirname(file_path)
solver_path = os.path.dirname(dir_path)
gen_dir = solver_path + "/model/parking_general/"
if len(sys.argv) > 1:
    gen_dir = sys.argv[1]

ocp_model = OptimalControlProblem("ParkingGeneral")

# define state symbols
s, x, y, theta, steer, v = ocp_model.add("state", "s, x, y, theta, steer, v")

# define control symbols
dsteer, a = ocp_model.add("ctrl", "dsteer, a")

# define slack symbols
slack_offset = ocp_model.add("slack", "slack_offset")

# state params
wheelbase, xr, yr, thetar, kr, vr = ocp_model.add(
    "param", "wheelbase, xr, yr, thetar, kr, vr"
)

# cost params
ref_weight, steer_weight, dsteer_weight = ocp_model.add(
    "param",
    ref_weight=0.01,
    steer_weight=10.0,
    dsteer_weight=0.1,
)
terminal_ref_weight, terminal_heading_weight, terminal_v_weight = ocp_model.add(
    "param",
    terminal_ref_weight=1.0,
    terminal_heading_weight=1.0,
    terminal_v_weight=1.0,
)
v_weight = ocp_model.add("param", v_weight=0.0)
a_weight = ocp_model.add("param", a_weight=1.0)
weigth_slack_offset = ocp_model.add("param", weigth_slack_offset=1.0)
# constraints params
xrf, yrf, thetarf = ocp_model.add("param", "xrf, yrf, thetarf")  # front
xrr, yrr, thetarr = ocp_model.add("param", "xrr, yrr, thetarr")  # rear
steer_lower, steer_upper = ocp_model.add("param", steer_lower=-0.436, steer_upper=0.436)
dsteer_lower, dsteer_upper = ocp_model.add(
    "param", dsteer_lower=-0.436, dsteer_upper=0.436
)
v_lower, v_upper = ocp_model.add("param", v_lower=-3, v_upper=3)
a_lower, a_upper = ocp_model.add("param", a_lower=-6, a_upper=2)
lf, lr = ocp_model.add("param", lf=4.0, lr=1.0)
ll, lu = ocp_model.add("param", ll=-20, lu=20)
lfl, lfu = ocp_model.add("param", lfl=-20, lfu=20)
lrl, lru = ocp_model.add("param", lrl=-20, lru=20)
sll, slu = ocp_model.add("param", sll=-20, slu=20)
slfl, slfu = ocp_model.add("param", slfl=-20, slfu=20)
slrl, slru = ocp_model.add("param", slrl=-20, slru=20)

l = -sin(thetar) * (x - xr) + cos(thetar) * (y - yr)
ds = cos(theta - thetar) / (1 - kr * l)
kappa = tan(steer) / wheelbase

# define dynamics
ocp_model.subject(dot(s) == v * ds)
ocp_model.subject(dot(x) == v * cos(theta))
ocp_model.subject(dot(y) == v * sin(theta))
ocp_model.subject(dot(theta) == v * kappa)
ocp_model.subject(dot(steer) == dsteer)
ocp_model.subject(dot(v) == a)

# define norminal cost
cost_x = (
    0.5 * ref_weight * l**2
    + 0.5 * v_weight * (v - vr) ** 2
    + 0.5 * steer_weight * steer**2
)
cost_u = 0.5 * dsteer_weight * dsteer**2 + 0.5 * a_weight * a**2
cost_s = 0.5 * weigth_slack_offset * slack_offset**2
ocp_model.add("cost", cost_x, cost_u, cost_s)

# define terminal cost
# ternimal_cost_x = (
#     0.5 * terminal_ref_weight * l**2
#     + 0.5 * terminal_heading_weight * (theta - thetar) ** 2
#     + 0.5 * terminal_v_weight * (v - 0.0) ** 2
# )
# ocp_model.add("cost", ternimal_cost_x, type=StageType.TERMINAL)

# define constraints
blf = -sin(thetarf) * (x + lf * cos(theta) - xrf) + cos(thetarf) * (
    y + lf * sin(theta) - yrf
)
blr = -sin(thetarr) * (x - lr * cos(theta) - xrr) + cos(thetarr) * (
    y - lr * sin(theta) - yrr
)

ocp_model.subject(ll <= l, l <= lu)
ocp_model.subject(sll <= l + slack_offset, l - slack_offset <= slu)
ocp_model.subject(lfl <= blf, blf <= lfu)
ocp_model.subject(lrl <= blr, blr <= lru)
ocp_model.subject(slfl <= blf + slack_offset, blf - slack_offset <= slfu)
ocp_model.subject(slrl <= blr + slack_offset, blr - slack_offset <= slru)

ocp_model.subject(steer_lower <= steer, steer <= steer_upper)
ocp_model.subject(dsteer_lower <= dsteer, dsteer <= dsteer_upper)
ocp_model.subject(v_lower <= v, v <= v_upper)
ocp_model.subject(a_lower <= a, a <= a_upper)
ocp_model.subject(cos(theta - thetar) >= 0.0)

ocp_model.subject(Equality(x, xr), stage=StageType.TERMINAL)
ocp_model.subject(Equality(y, yr), stage=StageType.TERMINAL)
ocp_model.subject(Equality(theta, thetar), stage=StageType.TERMINAL)
# ocp_model.subject(Equality(v, v_ref), stage=StageType.TERMINAL)
# ocp_model.subject(Equality(l, 0), stage=StageType.TERMINAL)

ocp_model.generate(gen_dir)
