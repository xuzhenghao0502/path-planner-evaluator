# -*- coding: utf-8 -*-
import sys
import sympy as sym
from sympy import *

import numpy as np

from core.ocp_model import *

file_path = os.path.abspath(__file__)
dir_path = os.path.dirname(file_path)
solver_path = os.path.dirname(dir_path)
gen_dir = solver_path + "/model/park_lateral_general/"
if len(sys.argv) > 1:
    gen_dir = sys.argv[1]

ocp_model = OptimalControlProblem("ParkLateralGeneral")

# define state symbols
s, x, y, theta, kappa = ocp_model.add("state", "s, x, y, theta, kappa")

# define control symbols
dkappa = ocp_model.add("ctrl", "dkappa")

# define slack symbols
slack_offset = ocp_model.add("slack", "slack_offset")

# state params
xr, yr, thetar, kr = ocp_model.add("param",
                                   "xr, yr, thetar, kr")

# other params
vr = ocp_model.add("param", "vr")

# cost params
ref_weight, kappa_weight, dkappa_weight = ocp_model.add("param",
                                                        ref_weight=0.01, kappa_weight=10.0, dkappa_weight=100.0)
weigth_slack_offset = ocp_model.add("param",
                                    weigth_slack_offset=1.0)
terminal_ref_weight, terminal_heading_weight = ocp_model.add("param",
                                                             terminal_position_weight=1.0, terminal_heading_weight=1.0)
# constraints params
xrf, yrf, thetarf = ocp_model.add("param", "xrf, yrf, thetarf") # front
xrr, yrr, thetarr = ocp_model.add("param", "xrr, yrr, thetarr") # rear
lf, lr = ocp_model.add("param", lf=4.0, lr=1.0)
ll, lu = ocp_model.add("param", ll=-20, lu=20)
lfl, lfu = ocp_model.add("param", lfl=-20, lfu=20)
lrl, lru = ocp_model.add("param", lrl=-20, lru=20)
sll, slu = ocp_model.add("param", sll=-20, slu=20)
KappaLowerBound, KappaUpperBound = ocp_model.add("param",
                                                 KappaLowerBound=-0.1, KappaUpperBound=0.1)
DKappaLowerBound, DKappaUpperBound = ocp_model.add("param",
                                                   DKappaLowerBound=-0.1, DKappaUpperBound=0.1)
weigth_prev_kappa, weight_prev_dkappa, prev_kappa, prev_dkappa = ocp_model.add("param",
                                                                               weigth_prev_kappa=0.0, weight_prev_dkappa=0.0, prev_kappa=0.0, prev_dkappa=0.0)
slfl, slfu = ocp_model.add("param", slfl=-20, slfu=20)
slrl, slru = ocp_model.add("param", slrl=-20, slru=20)

l = -sin(thetar) * (x - xr) + cos(thetar) * (y - yr)
ds = cos(theta - thetar) / (1 - kr * l)

# define dynamics
ocp_model.subject(dot(s) == vr * ds)
ocp_model.subject(dot(x) == vr * cos(theta))
ocp_model.subject(dot(y) == vr * sin(theta))
ocp_model.subject(dot(theta) == vr * kappa)
ocp_model.subject(dot(kappa) == dkappa)

# define norminal cost
cost_x = 0.5 * ref_weight * l ** 2 + 0.5 * kappa_weight * kappa**2
cost_u = 0.5 * dkappa_weight * dkappa**2
cost_s = 0.5 * weigth_slack_offset * slack_offset ** 2
cost_prev = 0.5 * weigth_prev_kappa * \
    (kappa - prev_kappa) ** 2 + 0.5 * \
    weight_prev_dkappa * (dkappa - prev_dkappa) ** 2
ocp_model.add("cost", cost_x, cost_u, cost_s, cost_prev)

# define terminal cost
ternimal_cost_x = 0.5 * terminal_ref_weight * l ** 2 + \
    0.5 * terminal_heading_weight * (theta - thetar) ** 2

ocp_model.add("cost", ternimal_cost_x, type=StageType.TERMINAL)

# define constraints
blf = -sin(thetarf) * (x + lf * cos(theta) - xrf) + \
    cos(thetarf) * (y + lf * sin(theta) - yrf)
blr = -sin(thetarr) * (x - lr * cos(theta) - xrr) + \
    cos(thetarr) * (y - lr * sin(theta) - yrr)

ocp_model.subject(ll <= l, l <= lu)
ocp_model.subject(sll <= l + slack_offset, l - slack_offset <= slu)
ocp_model.subject(lfl <= blf, blf <= lfu)
ocp_model.subject(lrl <= blr, blr <= lru)
ocp_model.subject(slfl <= blf + slack_offset,
                  blf - slack_offset <= slfu)
ocp_model.subject(slrl <= blr + slack_offset,
                  blr - slack_offset <= slru)

ocp_model.subject(kappa >= KappaLowerBound, kappa <= KappaUpperBound)
ocp_model.subject(dkappa <= DKappaUpperBound,
                  dkappa >= DKappaLowerBound)
ocp_model.subject(cos(theta - thetar) >= 0.0)


ocp_model.generate(gen_dir)
