# -*- coding: utf-8 -*-
import sys
import sympy as sym
from sympy import *

import numpy as np

from core.ocp_model import *

file_path = os.path.abspath(__file__)
dir_path = os.path.dirname(file_path)
solver_path = os.path.dirname(dir_path)
gen_dir = solver_path + "/model/speed_ocp_model"
if len(sys.argv) > 1:
    gen_dir = sys.argv[1]

ocp_model = OptimalControlProblem("SpeedOCPModel")

# define state symbols
s, v, a = ocp_model.add("state", "s, v, a")

# define control symbols
j = ocp_model.add("ctrl", "j")

# define slack symbols
(
    slack_s_upper,
    slack_s_lower,
    slack_v_upper,
    slack_v_lower,
    slack_a_upper,
    slack_a_lower,
    slack_d_v,
) = ocp_model.add(
    "slack",
    "slack_s_upper, slack_s_lower, slack_v_upper, slack_v_lower, slack_a_upper, slack_a_lower, slack_d_v",
)

# define dynamic equation
ocp_model.subject(dot(s) == v)
ocp_model.subject(dot(v) == a)
ocp_model.subject(dot(a) == j)

# define cost parameters
SRef, VRef, K = ocp_model.add("param", SRef=285.0, VRef=30.0, K=1.5)
(
    SWeight,
    VWeight,
    AWeight,
    JWeight,
    SlackSUpperWeight,
    SlackSLowerWeight,
    SlackVUpperWeight,
    SlackVLowerWeight,
    SlackAUpperWeight,
    SlackALowerWeight,
    SlackDVWeight,
) = ocp_model.add(
    "param",
    SWeight=0.004,
    VWeight=0.001,
    AWeight=0.1,
    JWeight=10.0,
    SlackSUpperWeight=100.0,
    SlackSLowerWeight=0.0001,
    SlackVUpperWeight=100000.0,
    SlackVLowerWeight=0.0001,
    SlackAUpperWeight=0.1,
    SlackALowerWeight=0.1,
    SlackDVWeight=1.0,
)

# define constraint parameters
(
    SHardUpperBound,
    SHardLowerBound,
    SSoftUpperBound,
    SSoftLowerBound,
    SUpperBoundForDVConstraint,
    VHardUpperBound,
    VHardLowerBound,
    VSoftUpperBound,
    VSoftLowerBound,
    AHardUpperBound,
    AHardLowerBound,
    ASoftUpperBound,
    ASoftLowerBound,
    JHardUpperBound,
    JHardLowerBound,
    SafeDistForDVConstraint,
    k,
) = ocp_model.add(
    "param",
    SHardUpperBound=300.0,
    SHardLowerBound=0.0,
    SSoftUpperBound=200.0,
    SSoftLowerBound=0.0,
    SUpperBoundForDVConstraint=200.0,
    VHardUpperBound=40.0,
    VHardLowerBound=0.0,
    VSoftUpperBound=30.0,
    VSoftLowerBound=0.0,
    AHardUpperBound=2.0,
    AHardLowerBound=-6.0,
    ASoftUpperBound=1.99,
    ASoftLowerBound=-5.0,
    JHardUpperBound=2.0,
    JHardLowerBound=-10.0,
    SafeDistForDVConstraint=4.0,
    k=0.3,
)

# define cost
cost_x = (
    SWeight * (s - SRef + K * v) ** 2 + VWeight * (v - VRef) ** 2 + AWeight * a**2
)
cost_u = JWeight * j**2
cost_slack = (
    SlackSUpperWeight * slack_s_upper**2
    + SlackSLowerWeight * slack_s_lower**2
    + SlackVUpperWeight * slack_v_upper**2
    + SlackVLowerWeight * slack_v_lower**2
    + SlackAUpperWeight * slack_a_upper**2
    + SlackALowerWeight * slack_a_lower**2
    + SlackDVWeight * slack_d_v**2
)

ocp_model.add("cost", cost_x, cost_u, cost_slack)
ocp_model.add("cost", cost_x, type=StageType.TERMINAL)

# define constraints
ocp_model.subject(SHardLowerBound <= s, s <= SHardUpperBound)
ocp_model.subject(VHardLowerBound <= v, v <= VHardUpperBound)
ocp_model.subject(AHardLowerBound <= a, a <= AHardUpperBound)
ocp_model.subject(JHardLowerBound <= j, j <= JHardUpperBound)
ocp_model.subject(s - slack_s_upper <= SSoftUpperBound)
ocp_model.subject(SSoftLowerBound <= s + slack_s_lower)
ocp_model.subject(v - slack_v_upper <= VSoftUpperBound)
ocp_model.subject(VSoftLowerBound <= v + slack_v_lower)
ocp_model.subject(a - slack_a_upper <= ASoftUpperBound)
ocp_model.subject(ASoftLowerBound <= a + slack_a_lower)
ocp_model.subject(
    SafeDistForDVConstraint + k * v <= SUpperBoundForDVConstraint - s + slack_d_v
)


ocp_model.generate(gen_dir)
