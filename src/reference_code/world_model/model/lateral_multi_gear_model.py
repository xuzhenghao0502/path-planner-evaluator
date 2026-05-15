# -*- coding: utf-8 -*-
import sys
import sympy as sym
from sympy import *

import numpy as np

from core.ocp_model import *

file_path = os.path.abspath(__file__)
dir_path = os.path.dirname(file_path)
solver_path = os.path.dirname(dir_path)
gen_dir = solver_path + "/model/lateral_multi_gear/"
if len(sys.argv) > 1:
    gen_dir = sys.argv[1]

ocp_model = OptimalControlProblem("LateralMultiGear")

# define state symbols
x, y, theta, kappa = ocp_model.add("state", "x, y, theta, kappa")

# define control symbols
dkappa = ocp_model.add("ctrl", "dkappa")

# state params
xr, yr, thetar, kr = ocp_model.add("param",
                                   "xr, yr, thetar, kr")

# other params
vr = ocp_model.add("param", "vr")

# cost params
weight_position, weight_heading, weight_kappa, weight_dkappa = ocp_model.add("param",
                        weight_position=0.01, weight_heading=0.1, weight_kappa=1.0, weight_dkappa=10.0)
terminal_ref_weight, terminal_heading_weight = ocp_model.add("param",
                                                             terminal_position_weight=1.0, terminal_heading_weight=1.0)
# constraints params
kappa_lower, kappa_upper = ocp_model.add("param",
                                         kappa_lower=-0.1, kappa_upper=0.1)
dkappa_lower, dkappa_upper = ocp_model.add("param",
                                           dkappa_lower=-0.1, dkappa_upper=0.1)

l = -sin(thetar) * (x - xr) + cos(thetar) * (y - yr)
# define dynamics
ocp_model.subject(dot(x) == vr * cos(theta))
ocp_model.subject(dot(y) == vr * sin(theta))
ocp_model.subject(dot(theta) == vr * kappa)
ocp_model.subject(dot(kappa) == dkappa)

# define norminal cost
cost_x = 0.5 * weight_position * ((x-xr) ** 2 + (y-yr)**2) + 0.5 * weight_heading * (
    theta - thetar)**2 + 0.5 * weight_kappa * (kappa - kr)**2
cost_u = 0.5 * weight_dkappa * dkappa**2
ocp_model.add("cost", cost_x, cost_u)

# define terminal cost
ternimal_cost_x = 0.5 * terminal_ref_weight * l ** 2 + \
    0.5 * terminal_heading_weight * (theta - thetar) ** 2

ocp_model.add("cost", ternimal_cost_x, type=StageType.TERMINAL)

# define constraints
ocp_model.subject(kappa_lower <= kappa, kappa <= kappa_upper)
ocp_model.subject(dkappa_lower <= dkappa, dkappa <= dkappa_upper)

ocp_model.generate(gen_dir)
