import sys
import sympy as sym
from sympy import *

import numpy as np

from core.ocp_model import *

file_path = os.path.abspath(__file__)
dir_path = os.path.dirname(file_path)
solver_path = os.path.dirname(dir_path)
gen_dir = solver_path + "/model/bicycle_trajectory_tracker/"
if len(sys.argv) > 1:
    gen_dir = sys.argv[1]

ocp_model = OptimalControlProblem("BicycleTrajectoryTracker")

# define state symbols
x, y, theta, v, kappa = ocp_model.add("state", "x, y, theta, v, kappa")
# define control symbols
a, dkappa = ocp_model.add("ctrl", "a, dkappa")
# define parameters

# state params
xr, yr, thetar, vr, kr = ocp_model.add("param", "xr, yr, thetar, vr, kr")
# cost params
weight_position, weight_heading, weight_v, weight_kappa, weight_a, weight_dkappa = ocp_model.add("param",
                                                                                                 weight_position=0.01, weight_heading=0.1, weight_v=0.001, weight_kappa=1.0, weight_a=0.01, weight_dkappa=10.0)
# constraints params
v_lower, v_upper = ocp_model.add("param", v_lower=0.0, v_upper=40)
a_lower, a_upper = ocp_model.add("param", a_lower=-10.0, a_upper=5.0)
kappa_lower, kappa_upper = ocp_model.add("param", kappa_lower=-0.1, kappa_upper=0.1)
dkappa_lower, dkappa_upper = ocp_model.add("param", dkappa_lower=-0.1, dkappa_upper=0.1)

# define dynamics
ocp_model.subject(dot(x) == v * cos(theta))
ocp_model.subject(dot(y) == v * sin(theta))
ocp_model.subject(dot(theta) == v * kappa)
ocp_model.subject(dot(v) == a)
ocp_model.subject(dot(kappa) == dkappa)

# define norminal cost
cost_x = 0.5 * weight_position * ((x-xr) ** 2 + (y-yr)**2) + 0.5 * weight_heading * (theta - thetar)**2 + 0.5 * weight_v * (v - vr)**2 + 0.5 * weight_kappa * (kappa - kr)**2
cost_u = 0.5 * weight_a * a**2 + 0.5 * weight_dkappa * dkappa**2
ocp_model.add("cost", cost_x, cost_u)

# define constraints
ocp_model.subject(v_lower <= v, v <= v_upper)
ocp_model.subject(a_lower <= a, a <= a_upper)
ocp_model.subject(kappa_lower <= kappa, kappa <= kappa_upper)
ocp_model.subject(dkappa_lower <= dkappa, dkappa <= dkappa_upper)

ocp_model.generate(gen_dir)
