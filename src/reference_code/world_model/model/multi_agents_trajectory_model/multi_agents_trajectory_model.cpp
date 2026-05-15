#include "multi_agents_trajectory_model.h"

namespace gpal::pnc::planning {

MultiAgentsTrajectoryModel::MultiAgentsTrajectoryModel() : OptimalControlProblem("MultiAgentsTrajectoryModel", 10, 9, 6, 86, 13) {
  default_state_.emplace("ego_s");
  default_state_.emplace("ego_x");
  default_state_.emplace("ego_y");
  default_state_.emplace("ego_theta");
  default_state_.emplace("ego_v");
  default_state_.emplace("agent_0_s");
  default_state_.emplace("agent_0_x");
  default_state_.emplace("agent_0_y");
  default_state_.emplace("agent_0_theta");
  default_state_.emplace("agent_0_v");

  default_ctrl_.emplace("ego_a");
  default_ctrl_.emplace("ego_steer");
  default_ctrl_.emplace("agent_0_a");
  default_ctrl_.emplace("ego_f_2_agent_0_slack");
  default_ctrl_.emplace("ego_r_2_agent_0_slack");
  default_ctrl_.emplace("ego_f_2_object_0_slack");
  default_ctrl_.emplace("ego_r_2_object_0_slack");
  default_ctrl_.emplace("ego_f_2_object_1_slack");
  default_ctrl_.emplace("ego_r_2_object_1_slack");

  default_param_.emplace("ego_x_ref", 0.0);
  default_param_.emplace("ego_y_ref", 0.0);
  default_param_.emplace("ego_theta_ref", 0.0);
  default_param_.emplace("ego_kappa_ref", 0.0);
  default_param_.emplace("ego_steer_ref", 0.0);
  default_param_.emplace("agent_0_x_ref", 0.0);
  default_param_.emplace("agent_0_y_ref", 0.0);
  default_param_.emplace("agent_0_theta_ref", 0.0);
  default_param_.emplace("agent_0_kappa_ref", 0.0);
  default_param_.emplace("agent_0_steer_ref", 0.0);
  default_param_.emplace("ego_s_coarse", 0.0);
  default_param_.emplace("ego_v_coarse", 0.0);
  default_param_.emplace("ego_a_coarse", 0.0);
  default_param_.emplace("agent_0_s_coarse", 0.0);
  default_param_.emplace("agent_0_v_coarse", 0.0);
  default_param_.emplace("agent_0_a_coarse", 0.0);
  default_param_.emplace("l_offset", 0.0);
  default_param_.emplace("wheelbase", 3.0);
  default_param_.emplace("front_overhang", 1.0);
  default_param_.emplace("rear_overhang", 1.0);
  default_param_.emplace("length", 5.0);
  default_param_.emplace("width", 2.0);
  default_param_.emplace("l_ref_weight", 1.0);
  default_param_.emplace("theta_ref_weight", 1.0);
  default_param_.emplace("steer_weight", 1.0);
  default_param_.emplace("ego_s_coarse_weight", 1.0);
  default_param_.emplace("ego_v_coarse_weight", 1.0);
  default_param_.emplace("ego_a_weight", 1.0);
  default_param_.emplace("agent_0_s_coarse_weight", 1.0);
  default_param_.emplace("agent_0_v_coarse_weight", 1.0);
  default_param_.emplace("agent_0_a_weight", 1.0);
  default_param_.emplace("agent_0_avoid_weight", 100.0);
  default_param_.emplace("obs_0_weight", 1.0);
  default_param_.emplace("obs_1_weight", 1.0);
  default_param_.emplace("Agent_0_AvoidedBound", 1.0);
  default_param_.emplace("Obs_0_AvoidedBound", 1.0);
  default_param_.emplace("Obs_1_AvoidedBound", 1.0);
  default_param_.emplace("LHardLowerBound", -20.0);
  default_param_.emplace("LHardUpperBound", 20.0);
  default_param_.emplace("SteerLowerBound", -0.436);
  default_param_.emplace("SteerUpperBound", 0.436);
  default_param_.emplace("SHardLowerBound", 0.0);
  default_param_.emplace("SHardUpperBound", 300.0);
  default_param_.emplace("VHardLowerBound", 0.0);
  default_param_.emplace("VHardUpperBound", 40.0);
  default_param_.emplace("AHardLowerBound", -6.0);
  default_param_.emplace("AHardUpperBound", 3.0);
  default_param_.emplace("Agent_0_AHardLowerBound", -3.0);
  default_param_.emplace("Agent_0_AHardUpperBound", 3.0);
  default_param_.emplace("agent_a_0", 5.0);
  default_param_.emplace("agent_a_0_square", 25.0);
  default_param_.emplace("agent_b_0", 2.0);
  default_param_.emplace("agent_b_0_square", 4.0);
  default_param_.emplace("agent_length_0", 5.0);
  default_param_.emplace("agent_width_0", 2.0);
  default_param_.emplace("object_a_0", 5.0);
  default_param_.emplace("object_a_0_square", 25.0);
  default_param_.emplace("object_b_0", 2.0);
  default_param_.emplace("object_b_0_square", 4.0);
  default_param_.emplace("object_x_0", 0.0);
  default_param_.emplace("object_y_0", 0.0);
  default_param_.emplace("object_theta_0", 0.0);
  default_param_.emplace("object_cos_theta_0", 1.0);
  default_param_.emplace("object_sin_theta_0", 0.0);
  default_param_.emplace("object_v_0", 0.0);
  default_param_.emplace("object_length_0", 5.0);
  default_param_.emplace("object_width_0", 2.0);
  default_param_.emplace("object_a_1", 5.0);
  default_param_.emplace("object_a_1_square", 25.0);
  default_param_.emplace("object_b_1", 2.0);
  default_param_.emplace("object_b_1_square", 4.0);
  default_param_.emplace("object_x_1", 0.0);
  default_param_.emplace("object_y_1", 0.0);
  default_param_.emplace("object_theta_1", 0.0);
  default_param_.emplace("object_cos_theta_1", 1.0);
  default_param_.emplace("object_sin_theta_1", 0.0);
  default_param_.emplace("object_v_1", 0.0);
  default_param_.emplace("object_length_1", 5.0);
  default_param_.emplace("object_width_1", 2.0);
  default_param_.emplace("s_scale", 20.0);
  default_param_.emplace("v_scale", 10.0);
  default_param_.emplace("a_scale", 5.0);
  default_param_.emplace("jerk_scale", 5.0);
  default_param_.emplace("l_scale", 1.0);
  default_param_.emplace("theta_scale", 1.0);
  default_param_.emplace("steer_scale", 1.0);

  default_global_.emplace("ego_kappa");
  default_global_.emplace("x_r_circle");
  default_global_.emplace("y_r_circle");
  default_global_.emplace("ego_l");
  default_global_.emplace("x_f_circle");
  default_global_.emplace("y_f_circle");
  default_global_.emplace("ego_ds");
  default_global_.emplace("r_dis_2_agent_0");
  default_global_.emplace("r_dis_2_object_0");
  default_global_.emplace("r_dis_2_object_1");
  default_global_.emplace("f_dis_2_agent_0");
  default_global_.emplace("f_dis_2_object_0");
  default_global_.emplace("f_dis_2_object_1");
  default_global_.setUpdater(std::make_shared<OcpVariable::Updater>([](const OcpStage& stage, OcpVariable* ptr_global) {
    // Determine stage variables
    const auto& ego_x = stage.x(1);
    const auto& ego_y = stage.x(2);
    const auto& ego_theta = stage.x(3);
    const auto& agent_0_x = stage.x(6);
    const auto& agent_0_y = stage.x(7);
    const auto& ego_steer = stage.u(1);
    const auto& ego_x_ref = stage.params(0);
    const auto& ego_y_ref = stage.params(1);
    const auto& ego_theta_ref = stage.params(2);
    const auto& ego_kappa_ref = stage.params(3);
    const auto& agent_0_theta_ref = stage.params(7);
    const auto& wheelbase = stage.params(17);
    const auto& front_overhang = stage.params(18);
    const auto& rear_overhang = stage.params(19);
    const auto& agent_a_0_square = stage.params(50);
    const auto& agent_b_0_square = stage.params(52);
    const auto& object_a_0_square = stage.params(56);
    const auto& object_b_0_square = stage.params(58);
    const auto& object_x_0 = stage.params(59);
    const auto& object_y_0 = stage.params(60);
    const auto& object_cos_theta_0 = stage.params(62);
    const auto& object_sin_theta_0 = stage.params(63);
    const auto& object_a_1_square = stage.params(68);
    const auto& object_b_1_square = stage.params(70);
    const auto& object_x_1 = stage.params(71);
    const auto& object_y_1 = stage.params(72);
    const auto& object_cos_theta_1 = stage.params(74);
    const auto& object_sin_theta_1 = stage.params(75);

    (*ptr_global)(0) = tan(ego_steer)/wheelbase;
    (*ptr_global)(1) = ego_x - rear_overhang*cos(ego_theta);
    (*ptr_global)(2) = ego_y - rear_overhang*sin(ego_theta);
    (*ptr_global)(3) = -(ego_x - ego_x_ref)*sin(ego_theta_ref) + (ego_y - ego_y_ref)*cos(ego_theta_ref);
    (*ptr_global)(4) = ego_x + (front_overhang + wheelbase)*cos(ego_theta);
    (*ptr_global)(5) = ego_y + (front_overhang + wheelbase)*sin(ego_theta);
    (*ptr_global)(6) = cos(ego_theta - ego_theta_ref)/(-ego_kappa_ref*(-(ego_x - ego_x_ref)*sin(ego_theta_ref) + (ego_y - ego_y_ref)*cos(ego_theta_ref)) + 1);
    (*ptr_global)(7) = pow((agent_0_x - ego_x + rear_overhang*cos(ego_theta))*sin(agent_0_theta_ref) + (-agent_0_y + ego_y - rear_overhang*sin(ego_theta))*cos(agent_0_theta_ref), 2)/agent_b_0_square + pow((-agent_0_x + ego_x - rear_overhang*cos(ego_theta))*cos(agent_0_theta_ref) + (-agent_0_y + ego_y - rear_overhang*sin(ego_theta))*sin(agent_0_theta_ref), 2)/agent_a_0_square;
    (*ptr_global)(8) = pow(object_cos_theta_0*(ego_y - object_y_0 - rear_overhang*sin(ego_theta)) + object_sin_theta_0*(-ego_x + object_x_0 + rear_overhang*cos(ego_theta)), 2)/object_b_0_square + pow(object_cos_theta_0*(ego_x - object_x_0 - rear_overhang*cos(ego_theta)) + object_sin_theta_0*(ego_y - object_y_0 - rear_overhang*sin(ego_theta)), 2)/object_a_0_square;
    (*ptr_global)(9) = pow(object_cos_theta_1*(ego_y - object_y_1 - rear_overhang*sin(ego_theta)) + object_sin_theta_1*(-ego_x + object_x_1 + rear_overhang*cos(ego_theta)), 2)/object_b_1_square + pow(object_cos_theta_1*(ego_x - object_x_1 - rear_overhang*cos(ego_theta)) + object_sin_theta_1*(ego_y - object_y_1 - rear_overhang*sin(ego_theta)), 2)/object_a_1_square;
    (*ptr_global)(10) = pow((agent_0_x - ego_x - (front_overhang + wheelbase)*cos(ego_theta))*sin(agent_0_theta_ref) + (-agent_0_y + ego_y + (front_overhang + wheelbase)*sin(ego_theta))*cos(agent_0_theta_ref), 2)/agent_b_0_square + pow((-agent_0_x + ego_x + (front_overhang + wheelbase)*cos(ego_theta))*cos(agent_0_theta_ref) + (-agent_0_y + ego_y + (front_overhang + wheelbase)*sin(ego_theta))*sin(agent_0_theta_ref), 2)/agent_a_0_square;
    (*ptr_global)(11) = pow(object_cos_theta_0*(ego_y - object_y_0 + (front_overhang + wheelbase)*sin(ego_theta)) + object_sin_theta_0*(-ego_x + object_x_0 - (front_overhang + wheelbase)*cos(ego_theta)), 2)/object_b_0_square + pow(object_cos_theta_0*(ego_x - object_x_0 + (front_overhang + wheelbase)*cos(ego_theta)) + object_sin_theta_0*(ego_y - object_y_0 + (front_overhang + wheelbase)*sin(ego_theta)), 2)/object_a_0_square;
    (*ptr_global)(12) = pow(object_cos_theta_1*(ego_y - object_y_1 + (front_overhang + wheelbase)*sin(ego_theta)) + object_sin_theta_1*(-ego_x + object_x_1 - (front_overhang + wheelbase)*cos(ego_theta)), 2)/object_b_1_square + pow(object_cos_theta_1*(ego_x - object_x_1 + (front_overhang + wheelbase)*cos(ego_theta)) + object_sin_theta_1*(ego_y - object_y_1 + (front_overhang + wheelbase)*sin(ego_theta)), 2)/object_a_1_square;
  }));
}

std::shared_ptr<Dynamics> MultiAgentsTrajectoryModel::createDynamics(OcpConfig::IntegratorType type) const {
  return std::make_shared<MultiAgentsTrajectoryModelDynamics>(type);
}

std::shared_ptr<CostFunction> MultiAgentsTrajectoryModel::createCostFunction() const {
  return std::make_shared<MultiAgentsTrajectoryModelCost<StageType::NORMINAL>>();
}

std::shared_ptr<Constraints> MultiAgentsTrajectoryModel::createConstraint() const {
  return std::make_shared<MultiAgentsTrajectoryModelConstraints<StageType::NORMINAL>>();
}

std::shared_ptr<Constraints> MultiAgentsTrajectoryModel::createInitialConstraint() const {
  return std::make_shared<MultiAgentsTrajectoryModelConstraints<StageType::INITIAL>>();
}

std::shared_ptr<Constraints> MultiAgentsTrajectoryModel::createTerminalConstraint() const {
  return std::make_shared<MultiAgentsTrajectoryModelConstraints<StageType::TERMINAL>>();
}

std::shared_ptr<Constraints> MultiAgentsTrajectoryModel::createStateOnlyEqualities() const {
  return std::make_shared<MultiAgentsTrajectoryModelStateOnlyEqualities<StageType::NORMINAL>>();
}

std::shared_ptr<Constraints> MultiAgentsTrajectoryModel::createInitialStateOnlyEqualities() const {
  return std::make_shared<MultiAgentsTrajectoryModelStateOnlyEqualities<StageType::INITIAL>>();
}

std::shared_ptr<Constraints> MultiAgentsTrajectoryModel::createTerminalEqualities() const {
  return std::make_shared<MultiAgentsTrajectoryModelStateOnlyEqualities<StageType::TERMINAL>>();
}

std::shared_ptr<IPMHelper> MultiAgentsTrajectoryModel::createBaseIpmHelper() const {
  IPMHelper::Creater creater{
    {{StageType::NORMINAL, OcpConfig::FORWARD_EULER},[]() { return std::make_shared<MultiAgentsTrajectoryModelIpmEvaluator<StageType::NORMINAL, OcpConfig::FORWARD_EULER>>(); }},
    {{StageType::NORMINAL, OcpConfig::ERK4},[]() { return std::make_shared<MultiAgentsTrajectoryModelIpmEvaluator<StageType::NORMINAL, OcpConfig::ERK4>>(); }},
    {{StageType::INITIAL, OcpConfig::FORWARD_EULER},[]() { return std::make_shared<MultiAgentsTrajectoryModelIpmEvaluator<StageType::INITIAL, OcpConfig::FORWARD_EULER>>(); }},
    {{StageType::INITIAL, OcpConfig::ERK4},[]() { return std::make_shared<MultiAgentsTrajectoryModelIpmEvaluator<StageType::INITIAL, OcpConfig::ERK4>>(); }},
    {{StageType::TERMINAL, OcpConfig::FORWARD_EULER},[]() { return std::make_shared<MultiAgentsTrajectoryModelIpmEvaluatorTerminal>(); }},
    {{StageType::TERMINAL, OcpConfig::ERK4},[]() { return std::make_shared<MultiAgentsTrajectoryModelIpmEvaluatorTerminal>(); }}};
  return std::make_shared<IPMHelper>(std::move(creater));
}

REGIST_MODEL(MultiAgentsTrajectoryModel)
} // namespace gpal::pnc::planning