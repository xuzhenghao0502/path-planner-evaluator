import sys
import sympy as sym
from sympy import *

import numpy as np
sys.path.append("../../pnc_solver/scripts/") 

from core.ocp_model import *

file_path = os.path.abspath(__file__)
dir_path = os.path.dirname(file_path)
solver_path = os.path.dirname(dir_path)
gen_dir = solver_path + "/model/lateral_general/"
if len(sys.argv) > 1:
    gen_dir = sys.argv[1]

ocp_model = OptimalControlProblem("LateralGeneral")

# define state symbols
s, x, y, theta, steer = ocp_model.add("state", "s, x, y, theta, steer")

# define control symbols
dsteer = ocp_model.add("ctrl", "dsteer")

# define slack symbols
slack_offset = ocp_model.add("slack", "slack_offset")

# state params
wheelbase, xr, yr, thetar, kr = ocp_model.add("param", "wheelbase, xr, yr, thetar, kr")
# cost params
l_weight,theta_ref_weight, steer_weight, dsteer_weight = ocp_model.add("param", l_weight=0.01,theta_ref_weight=0.1, steer_weight=0.1, dsteer_weight = 1.0)
weight_slack_offset = ocp_model.add("param", weight_slack_offset=1.0)
terminal_l_weight, terminal_heading_weight = ocp_model.add("param", terminal_l_weight=1.0, terminal_heading_weight=1.0)

# constraints params
xrf, yrf, thetarf = ocp_model.add("param", "xrf, yrf, thetarf")
xrr, yrr, thetarr = ocp_model.add("param", "xrr, yrr, thetarr")
lf, lr = ocp_model.add("param", lf=3.701, lr=1.0)

ll, lu = ocp_model.add("param", ll=-20, lu=20)
sll, slu = ocp_model.add("param", sll=-20, slu=20)

lfl, lfu = ocp_model.add("param", lfl=-20, lfu=20)
slfl, slfu = ocp_model.add("param", slfl=-20, slfu=20)

lrl, lru = ocp_model.add("param", lrl=-20, lru=20)
slrl, slru = ocp_model.add("param", slrl=-20, slru=20)

SteerLowerBound, SteerUpperBound = ocp_model.add("param", SteerLowerBound=-0.436, SteerUpperBound=0.436)   #默认最大轮转角度为25度
DSteerLowerBound, DSteerUpperBound = ocp_model.add("param", DSteerLowerBound=-0.436, DSteerUpperBound=0.436)  #默认最大轮转角速率为25度每秒
weight_prev_steer, weight_prev_dsteer, prev_steer, prev_dsteer = ocp_model.add("param", weight_prev_steer=0.0, weight_prev_dsteer=0.0, prev_steer=0.0, prev_dsteer=0.0)

l_offset = ocp_model.add("param", l_offset = 0.0)

l = -sin(thetar) * (x - xr) + cos(thetar) * (y - yr)
ds = cos(theta - thetar) / (1 - kr * l)
kappa = tan(steer) / wheelbase
vr = ocp_model.add("param", vr=1)
# define dynamics
ocp_model.subject(dot(s) == vr * ds)
ocp_model.subject(dot(x) == vr * cos(theta))
ocp_model.subject(dot(y) == vr * sin(theta))
ocp_model.subject(dot(theta) == vr * kappa)
ocp_model.subject(dot(steer) == dsteer)


# define cost
cost_x = 0.5 * l_weight * (l - l_offset) ** 2 + 0.5 * steer_weight * steer**2 + 0.5 * theta_ref_weight * (theta - thetar) ** 2
cost_u = 0.5 * dsteer_weight * dsteer**2
cost_s = 0.5 * weight_slack_offset * slack_offset ** 2
cost_prev = 0.5 * weight_prev_steer * (steer - prev_steer) ** 2
ocp_model.add("cost", cost_x, cost_u, cost_s, cost_prev)

# define terminal cost
ternimal_cost_x = 0.5 * terminal_l_weight * l ** 2 +  0.5 * terminal_heading_weight * (theta - thetar) ** 2

ocp_model.add("cost", ternimal_cost_x, type=StageType.TERMINAL)

# define constraints
blf = -sin(thetarf) * (x + lf * cos(theta) - xrf) + cos(thetarf) * (y + lf * sin(theta) - yrf)
blr = -sin(thetarr) * (x - lr * cos(theta) - xrr) + cos(thetarr) * (y - lr * sin(theta) - yrr)

ocp_model.subject(ll <= l, l <= lu)
ocp_model.subject(sll <= l + slack_offset, l - slack_offset <= slu)

ocp_model.subject(lfl <= blf, blf <= lfu)
ocp_model.subject(slfl <= blf + slack_offset, blf - slack_offset <= slfu)
                  
ocp_model.subject(lrl <= blr, blr <= lru)
ocp_model.subject(slrl <= blr + slack_offset, blr - slack_offset <= slru)

ocp_model.subject(steer >= SteerLowerBound, steer <= SteerUpperBound)
ocp_model.subject(dsteer >= DSteerLowerBound, dsteer <= DSteerUpperBound)

ocp_model.subject(cos(theta - thetar) >= 0.0)

ocp_model.generate(gen_dir)
