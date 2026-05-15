import sys
import sympy as sym
from sympy import *

import numpy as np

from core.ocp_model import *

file_path = os.path.abspath(__file__)
dir_path = os.path.dirname(file_path)
solver_path = os.path.dirname(dir_path)
gen_dir = solver_path + "/model/reference_line_model"
if len(sys.argv) > 1:
    gen_dir = sys.argv[1]

ocp_model = OptimalControlProblem("ReferenceLineModel")

# define state symbols
x, y, theta, kappa = ocp_model.add("state", "x, y, theta, kappa")

# define control symbols
dkappa, slack_offset = ocp_model.add("ctrl", "dkappa, slack_offset")
# dkappa = ocp_model.add("ctrl", "dkappa")

# define system equation
ocp_model.subject(dot(x) == cos(theta))
ocp_model.subject(dot(y) == sin(theta))
ocp_model.subject(dot(theta) == kappa)
ocp_model.subject(dot(kappa) == dkappa)

# define cost parameters
xr, yr, thetar = ocp_model.add("param", "xr, yr, thetar")
(
    l_weight,
    kappa_weight,
    dkappa_weight,
    v_weight,
    slack_offset_weight,
    l_weight_terminal,
    kappa_weight_terminal
) = ocp_model.add("param",
                  "l_weight, kappa_weight, dkappa_weight, v_weight, slack_offset_weight, l_weight_terminal, kappa_weight_terminal"
                  )

# cost
l = -sin(thetar) * (x - xr) + cos(thetar) * (y - yr)
l_x = 0.5 * l_weight * l**2 + 0.5 * kappa_weight * kappa**2
l_u = 0.5 * dkappa_weight * dkappa**2 + 0.5 * slack_offset_weight * slack_offset**2

# l_u = 0.5 * dkappa_weight * dkappa**2


ocp_model.add("cost", l_x, l_u)

# define terminal cost parameters
l_t = 0.5 * l_weight_terminal * l**2 + 0.5 * kappa_weight_terminal * kappa**2

ocp_model.add("cost", l_t, type=StageType.TERMINAL)

# define constraints
#ll, lu = ocp_model.add("param", "ll", "lu")
sll, slu = ocp_model.add("param", "sll", "slu")
KappaLowerBound, KappaUpperBound = ocp_model.add("param",
                                                 "KappaLowerBound", "KappaUpperBound"
                                                 )
DKappaLowerBound, DKappaUpperBound = ocp_model.add("param",
                                                   "DKappaLowerBound", "DKappaUpperBound"
                                                   )

theta_offset = cos(theta - thetar)
#ocp_model.subject(ll <= l, l <= lu)
ocp_model.subject(0.5 <= theta_offset)
ocp_model.subject(sll <= l + slack_offset, l - slack_offset <= slu)
ocp_model.subject(kappa >= KappaLowerBound, kappa <= KappaUpperBound)
ocp_model.subject(dkappa <= DKappaUpperBound, dkappa >= DKappaLowerBound)

ocp_model.generate(gen_dir)
