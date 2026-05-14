import sys

from sympy import *

sys.path.append("../../pnc_solver/scripts/")

from core.ocp_model import *

file_path = os.path.abspath(__file__)
dir_path = os.path.dirname(file_path)
solver_path = os.path.dirname(dir_path)
gen_dir = solver_path + "/model/multi_agents_trajectory_model/"
if len(sys.argv) > 1:
    gen_dir = sys.argv[1]

ocp_model = OptimalControlProblem("MultiAgentsTrajectoryModel")

epsilon = 1e-4

# define state symbols
# s 为投影到参考线上的匹配点的local s信息
ego_s, ego_x, ego_y, ego_theta, ego_v = ocp_model.add(
    "state", "ego_s, ego_x, ego_y, ego_theta, ego_v"
)
agent_0_s, agent_0_x, agent_0_y, agent_0_theta, agent_0_v = ocp_model.add(
    "state", "agent_0_s, agent_0_x, agent_0_y, agent_0_theta, agent_0_v"
)

# define control symbols
ego_a, ego_steer = ocp_model.add("ctrl", "ego_a, ego_steer")
agent_0_a = ocp_model.add("ctrl", "agent_0_a")


# define slack symbols
(
    ego_f_2_agent_0_slack,
    ego_r_2_agent_0_slack,
    ego_f_2_object_0_slack,
    ego_r_2_object_0_slack,
    ego_f_2_object_1_slack,
    ego_r_2_object_1_slack,
) = ocp_model.add(
    "slack",
    "ego_f_2_agent_0_slack, ego_r_2_agent_0_slack, ego_f_2_object_0_slack, ego_r_2_object_0_slack, ego_f_2_object_1_slack, ego_r_2_object_1_slack",
)


# reference line state params
ego_x_ref, ego_y_ref, ego_theta_ref, ego_kappa_ref, ego_steer_ref = ocp_model.add(
    "param", "ego_x_ref, ego_y_ref, ego_theta_ref, ego_kappa_ref, ego_steer_ref"
)
(
    agent_0_x_ref,
    agent_0_y_ref,
    agent_0_theta_ref,
    agent_0_kappa_ref,
    agent_0_steer_ref,
) = ocp_model.add(
    "param",
    "agent_0_x_ref, agent_0_y_ref, agent_0_theta_ref, agent_0_kappa_ref, agent_0_steer_ref",
)

#  coarse state params s_coarse 为决策轨迹在参考线上的投影点的local s信息
ego_s_coarse, ego_v_coarse, ego_a_coarse = ocp_model.add(
    "param", "ego_s_coarse, ego_v_coarse, ego_a_coarse"
)
agent_0_s_coarse, agent_0_v_coarse, agent_0_a_coarse = ocp_model.add(
    "param", "agent_0_s_coarse, agent_0_v_coarse, agent_0_a_coarse"
)

l_offset = ocp_model.add("param", l_offset=0.0)


wheelbase, front_overhang, rear_overhang, length, width = ocp_model.add(
    "param",
    wheelbase=3.0,
    front_overhang=1.0,
    rear_overhang=1.0,
    length=5.0,
    width=2.0,
)

# cost params

l_ref_weight, theta_ref_weight, steer_weight = ocp_model.add(
    "param",
    l_ref_weight=1.0,
    theta_ref_weight=1.0,
    steer_weight=1.0,
)


ego_s_coarse_weight, ego_v_coarse_weight, ego_a_weight = ocp_model.add(
    "param", ego_s_coarse_weight=1.0, ego_v_coarse_weight=1.0, ego_a_weight=1.0
)
agent_0_s_coarse_weight, agent_0_v_coarse_weight, agent_0_a_weight = ocp_model.add(
    "param",
    agent_0_s_coarse_weight=1.0,
    agent_0_v_coarse_weight=1.0,
    agent_0_a_weight=1.0,
)


agent_0_avoid_weight = ocp_model.add("param", agent_0_avoid_weight=100.0)

obs_0_weight, obs_1_weight = ocp_model.add("param", obs_0_weight=1.0, obs_1_weight=1.0)

Agent_0_AvoidedBound = ocp_model.add("param", Agent_0_AvoidedBound=1.0)

Obs_0_AvoidedBound, Obs_1_AvoidedBound = ocp_model.add(
    "param",
    Obs_0_AvoidedBound=1.0,
    Obs_1_AvoidedBound=1.0,
)

LHardLowerBound, LHardUpperBound = ocp_model.add(
    "param", LHardLowerBound=-20.0, LHardUpperBound=20.0
)


SteerLowerBound, SteerUpperBound = ocp_model.add(
    "param", SteerLowerBound=-0.436, SteerUpperBound=0.436
)  # 默认最大轮转角度为25度


SHardLowerBound, SHardUpperBound = ocp_model.add(
    "param", SHardLowerBound=0.0, SHardUpperBound=300.0
)

VHardLowerBound, VHardUpperBound = ocp_model.add(
    "param", VHardLowerBound=0.0, VHardUpperBound=40.0
)

AHardLowerBound, AHardUpperBound = ocp_model.add(
    "param", AHardLowerBound=-6.0, AHardUpperBound=3.0
)

Agent_0_AHardLowerBound, Agent_0_AHardUpperBound = ocp_model.add(
    "param", Agent_0_AHardLowerBound=-3.0, Agent_0_AHardUpperBound=3.0
)


# 计算点到当前障碍物椭圆的归一化距离
def compute_ellipse_distance(
    point_x,
    point_y,
    a_square,
    b_square,
    x_agent,
    y_agent,
    cos_theta_agent,
    sin_theta_agent,
):
    dx = point_x - x_agent
    dy = point_y - y_agent
    x_local = dx * cos_theta_agent + dy * sin_theta_agent
    y_local = -dx * sin_theta_agent + dy * cos_theta_agent
    return (x_local**2) / a_square + (y_local**2) / b_square


(
    agent_a_0,
    agent_a_0_square,
    agent_b_0,
    agent_b_0_square,
    agent_length_0,
    agent_width_0,
) = ocp_model.add(
    "param",
    agent_a_0=5.0,
    agent_a_0_square=25.0,
    agent_b_0=2.0,
    agent_b_0_square=4.0,
    agent_length_0=5.0,
    agent_width_0=2.0,
)


(
    object_a_0,
    object_a_0_square,
    object_b_0,
    object_b_0_square,
    object_x_0,
    object_y_0,
    object_theta_0,
    object_cos_theta_0,
    object_sin_theta_0,
    object_v_0,
    object_length_0,
    object_width_0,
) = ocp_model.add(
    "param",
    object_a_0=5.0,
    object_a_0_square=25.0,
    object_b_0=2.0,
    object_b_0_square=4.0,
    object_x_0=0.0,
    object_y_0=0.0,
    object_theta_0=0.0,
    object_cos_theta_0=1.0,
    object_sin_theta_0=0.0,
    object_v_0=0.0,
    object_length_0=5.0,
    object_width_0=2.0,
)

(
    object_a_1,
    object_a_1_square,
    object_b_1,
    object_b_1_square,
    object_x_1,
    object_y_1,
    object_theta_1,
    object_cos_theta_1,
    object_sin_theta_1,
    object_v_1,
    object_length_1,
    object_width_1,
) = ocp_model.add(
    "param",
    object_a_1=5.0,
    object_a_1_square=25.0,
    object_b_1=2.0,
    object_b_1_square=4.0,
    object_x_1=0.0,
    object_y_1=0.0,
    object_theta_1=0.0,
    object_cos_theta_1=1.0,
    object_sin_theta_1=0.0,
    object_v_1=0.0,
    object_length_1=5.0,
    object_width_1=2.0,
)


# define dynamics
# for ego
ego_l = -sin(ego_theta_ref) * (ego_x - ego_x_ref) + cos(ego_theta_ref) * (
    ego_y - ego_y_ref
)
ego_ds = cos(ego_theta - ego_theta_ref) / (1 - ego_kappa_ref * ego_l)
ego_kappa = tan(ego_steer) / wheelbase

ocp_model.subject(dot(ego_s) == ego_v * ego_ds)
ocp_model.subject(dot(ego_x) == ego_v * cos(ego_theta))
ocp_model.subject(dot(ego_y) == ego_v * sin(ego_theta))
ocp_model.subject(dot(ego_theta) == ego_v * ego_kappa)
ocp_model.subject(dot(ego_v) == ego_a)


# for agent 0
ocp_model.subject(dot(agent_0_s) == agent_0_v)
ocp_model.subject(dot(agent_0_x) == agent_0_v * cos(agent_0_theta_ref))
ocp_model.subject(dot(agent_0_y) == agent_0_v * sin(agent_0_theta_ref))
ocp_model.subject(dot(agent_0_v) == agent_0_a)


# 车辆头部中心坐标
x_f_circle = ego_x + (wheelbase + front_overhang) * cos(ego_theta)

y_f_circle = ego_y + (wheelbase + front_overhang) * sin(ego_theta)

# 车辆尾部中心坐标
x_r_circle = ego_x - rear_overhang * cos(ego_theta)

y_r_circle = ego_y - rear_overhang * sin(ego_theta)


# for agent
f_dis_2_agent_0 = compute_ellipse_distance(
    x_f_circle,
    y_f_circle,
    agent_a_0_square,
    agent_b_0_square,
    agent_0_x,
    agent_0_y,
    cos(agent_0_theta_ref),
    sin(agent_0_theta_ref),
)
r_dis_2_agent_0 = compute_ellipse_distance(
    x_r_circle,
    y_r_circle,
    agent_a_0_square,
    agent_b_0_square,
    agent_0_x,
    agent_0_y,
    cos(agent_0_theta_ref),
    sin(agent_0_theta_ref),
)


# for object
f_dis_2_object_0 = compute_ellipse_distance(
    x_f_circle,
    y_f_circle,
    object_a_0_square,
    object_b_0_square,
    object_x_0,
    object_y_0,
    object_cos_theta_0,
    object_sin_theta_0,
)
r_dis_2_object_0 = compute_ellipse_distance(
    x_r_circle,
    y_r_circle,
    object_a_0_square,
    object_b_0_square,
    object_x_0,
    object_y_0,
    object_cos_theta_0,
    object_sin_theta_0,
)

f_dis_2_object_1 = compute_ellipse_distance(
    x_f_circle,
    y_f_circle,
    object_a_1_square,
    object_b_1_square,
    object_x_1,
    object_y_1,
    object_cos_theta_1,
    object_sin_theta_1,
)
r_dis_2_object_1 = compute_ellipse_distance(
    x_r_circle,
    y_r_circle,
    object_a_1_square,
    object_b_1_square,
    object_x_1,
    object_y_1,
    object_cos_theta_1,
    object_sin_theta_1,
)


(
    s_scale,
    v_scale,
    a_scale,
    jerk_scale,
    l_scale,
    theta_scale,
    steer_scale,
) = ocp_model.add(
    "param",
    s_scale=20.0,
    v_scale=10.0,
    a_scale=5.0,
    jerk_scale=5.0,
    l_scale=1.0,
    theta_scale=1.0,
    steer_scale=1.0,
)

# define cost
cost_x_lateral = (
    0.5 * l_ref_weight * ((ego_l - l_offset) / l_scale) ** 2
    + 0.5 * theta_ref_weight * ((ego_theta - ego_theta_ref) / theta_scale) ** 2
)

cost_x_longitudinal = (
    +0.5 * ego_v_coarse_weight * ((ego_v - ego_v_coarse) / v_scale) ** 2
    + 0.5 * agent_0_v_coarse_weight * ((agent_0_v - agent_0_v_coarse) / v_scale) ** 2
    + 0.5 * ego_s_coarse_weight * ((ego_s - ego_s_coarse) / s_scale) ** 2
    + 0.5 * agent_0_s_coarse_weight * ((agent_0_s - agent_0_s_coarse) / s_scale) ** 2
)

cost_u = (
    0.5 * steer_weight * ((ego_steer - ego_steer_ref) / steer_scale) ** 2
    + 0.5 * ego_a_weight * (ego_a / a_scale) ** 2
    + 0.5 * agent_0_a_weight * (agent_0_a / a_scale) ** 2
)

cost_s = (
    0.5 * agent_0_avoid_weight * ego_f_2_agent_0_slack
    + 0.5 * agent_0_avoid_weight * ego_r_2_agent_0_slack
    + 0.5 * obs_0_weight * ego_f_2_object_0_slack
    + 0.5 * obs_0_weight * ego_r_2_object_0_slack
    + 0.5 * obs_1_weight * ego_f_2_object_1_slack
    + 0.5 * obs_1_weight * ego_r_2_object_1_slack
)

ocp_model.add("cost", cost_x_lateral, cost_x_longitudinal, cost_u, cost_s)


# define constraints
# for agent
ocp_model.subject(Agent_0_AvoidedBound <= f_dis_2_agent_0 + ego_f_2_agent_0_slack)


ocp_model.subject(Agent_0_AvoidedBound <= r_dis_2_agent_0 + ego_r_2_agent_0_slack)


# for object
ocp_model.subject(Obs_0_AvoidedBound <= f_dis_2_object_0 + ego_f_2_object_0_slack)

ocp_model.subject(Obs_1_AvoidedBound <= f_dis_2_object_1 + ego_f_2_object_1_slack)

ocp_model.subject(Obs_0_AvoidedBound <= r_dis_2_object_0 + ego_r_2_object_0_slack)

ocp_model.subject(Obs_1_AvoidedBound <= r_dis_2_object_1 + ego_r_2_object_1_slack)

# for slack
ocp_model.subject(ego_f_2_agent_0_slack >= 0.0)
ocp_model.subject(ego_r_2_agent_0_slack >= 0.0)
ocp_model.subject(ego_f_2_object_0_slack >= 0.0)
ocp_model.subject(ego_r_2_object_0_slack >= 0.0)
ocp_model.subject(ego_f_2_object_1_slack >= 0.0)
ocp_model.subject(ego_r_2_object_1_slack >= 0.0)

ocp_model.subject(LHardLowerBound <= ego_l, ego_l <= LHardUpperBound)


ocp_model.subject(SteerLowerBound <= ego_steer, ego_steer <= SteerUpperBound)


ocp_model.subject(SHardLowerBound <= ego_s, ego_s <= SHardUpperBound)
ocp_model.subject(agent_0_s >= 0.0)

ocp_model.subject(VHardLowerBound <= ego_v, ego_v <= VHardUpperBound)
ocp_model.subject(agent_0_v >= 0.0)

ocp_model.subject(AHardLowerBound <= ego_a, ego_a <= AHardUpperBound)
ocp_model.subject(
    Agent_0_AHardLowerBound <= agent_0_a, agent_0_a <= Agent_0_AHardUpperBound
)

ocp_model.subject(1 - ego_kappa_ref * ego_l >= epsilon)
ocp_model.subject(cos(ego_theta - ego_theta_ref) >= 0.0)


ocp_model.generate(gen_dir)
