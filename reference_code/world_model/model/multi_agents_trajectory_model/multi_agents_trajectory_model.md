# MultiAgentsTrajectoryModel user manual

user manual for generated model MultiAgentsTrajectoryModel, code optimization has been applied for the best performance

## State variable map

| Index | Name          |
| ----- | ------------- |
| 0     | ego_s         |
| 1     | ego_x         |
| 2     | ego_y         |
| 3     | ego_theta     |
| 4     | ego_v         |
| 5     | agent_0_s     |
| 6     | agent_0_x     |
| 7     | agent_0_y     |
| 8     | agent_0_theta |
| 9     | agent_0_v     |

## Ctrl variable map

| Index | Name      |
| ----- | --------- |
| 0     | ego_a     |
| 1     | ego_steer |
| 2     | agent_0_a |

## Slack variable map

| Index | Name                   |
| ----- | ---------------------- |
| 0     | ego_f_2_agent_0_slack  |
| 1     | ego_r_2_agent_0_slack  |
| 2     | ego_f_2_object_0_slack |
| 3     | ego_r_2_object_0_slack |
| 4     | ego_f_2_object_1_slack |
| 5     | ego_r_2_object_1_slack |

## Paramter map

| Index | Name                    |
| ----- | ----------------------- |
| 0     | ego_x_ref               |
| 1     | ego_y_ref               |
| 2     | ego_theta_ref           |
| 3     | ego_kappa_ref           |
| 4     | ego_steer_ref           |
| 5     | agent_0_x_ref           |
| 6     | agent_0_y_ref           |
| 7     | agent_0_theta_ref       |
| 8     | agent_0_kappa_ref       |
| 9     | agent_0_steer_ref       |
| 10    | ego_s_coarse            |
| 11    | ego_v_coarse            |
| 12    | ego_a_coarse            |
| 13    | agent_0_s_coarse        |
| 14    | agent_0_v_coarse        |
| 15    | agent_0_a_coarse        |
| 16    | l_offset                |
| 17    | wheelbase               |
| 18    | front_overhang          |
| 19    | rear_overhang           |
| 20    | length                  |
| 21    | width                   |
| 22    | l_ref_weight            |
| 23    | theta_ref_weight        |
| 24    | steer_weight            |
| 25    | ego_s_coarse_weight     |
| 26    | ego_v_coarse_weight     |
| 27    | ego_a_weight            |
| 28    | agent_0_s_coarse_weight |
| 29    | agent_0_v_coarse_weight |
| 30    | agent_0_a_weight        |
| 31    | agent_0_avoid_weight    |
| 32    | obs_0_weight            |
| 33    | obs_1_weight            |
| 34    | Agent_0_AvoidedBound    |
| 35    | Obs_0_AvoidedBound      |
| 36    | Obs_1_AvoidedBound      |
| 37    | LHardLowerBound         |
| 38    | LHardUpperBound         |
| 39    | SteerLowerBound         |
| 40    | SteerUpperBound         |
| 41    | SHardLowerBound         |
| 42    | SHardUpperBound         |
| 43    | VHardLowerBound         |
| 44    | VHardUpperBound         |
| 45    | AHardLowerBound         |
| 46    | AHardUpperBound         |
| 47    | Agent_0_AHardLowerBound |
| 48    | Agent_0_AHardUpperBound |
| 49    | agent_a_0               |
| 50    | agent_a_0_square        |
| 51    | agent_b_0               |
| 52    | agent_b_0_square        |
| 53    | agent_length_0          |
| 54    | agent_width_0           |
| 55    | object_a_0              |
| 56    | object_a_0_square       |
| 57    | object_b_0              |
| 58    | object_b_0_square       |
| 59    | object_x_0              |
| 60    | object_y_0              |
| 61    | object_theta_0          |
| 62    | object_cos_theta_0      |
| 63    | object_sin_theta_0      |
| 64    | object_v_0              |
| 65    | object_length_0         |
| 66    | object_width_0          |
| 67    | object_a_1              |
| 68    | object_a_1_square       |
| 69    | object_b_1              |
| 70    | object_b_1_square       |
| 71    | object_x_1              |
| 72    | object_y_1              |
| 73    | object_theta_1          |
| 74    | object_cos_theta_1      |
| 75    | object_sin_theta_1      |
| 76    | object_v_1              |
| 77    | object_length_1         |
| 78    | object_width_1          |
| 79    | s_scale                 |
| 80    | v_scale                 |
| 81    | a_scale                 |
| 82    | jerk_scale              |
| 83    | l_scale                 |
| 84    | theta_scale             |
| 85    | steer_scale             |

## Constraint map

| Index | Name                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| ----- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| 0     | LHardLowerBound <= -(ego_x - ego_x_ref)*sin(ego_theta_ref) + (ego_y - ego_y_ref)*cos(ego_theta_ref)                                                                                                                                                                                                                                                                                                                                                                      |
| 1     | -(ego_x - ego_x_ref)*sin(ego_theta_ref) + (ego_y - ego_y_ref)*cos(ego_theta_ref) <= LHardUpperBound                                                                                                                                                                                                                                                                                                                                                                      |
| 2     | SHardLowerBound <= ego_s                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 3     | ego_s <= SHardUpperBound                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 4     | agent_0_s >= 0.0                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 5     | VHardLowerBound <= ego_v                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 6     | ego_v <= VHardUpperBound                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 7     | agent_0_v >= 0.0                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 8     | -ego_kappa_ref*(-(ego_x - ego_x_ref)*sin(ego_theta_ref) + (ego_y - ego_y_ref)*cos(ego_theta_ref)) + 1 >= 0.0001                                                                                                                                                                                                                                                                                                                                                          |
| 9     | cos(ego_theta - ego_theta_ref) >= 0.0                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| 10    | ego_f_2_agent_0_slack >= 0.0                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| 11    | ego_r_2_agent_0_slack >= 0.0                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| 12    | ego_f_2_object_0_slack >= 0.0                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 13    | ego_r_2_object_0_slack >= 0.0                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 14    | ego_f_2_object_1_slack >= 0.0                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 15    | ego_r_2_object_1_slack >= 0.0                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 16    | SteerLowerBound <= ego_steer                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| 17    | ego_steer <= SteerUpperBound                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| 18    | AHardLowerBound <= ego_a                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 19    | ego_a <= AHardUpperBound                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 20    | Agent_0_AHardLowerBound <= agent_0_a                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 21    | agent_0_a <= Agent_0_AHardUpperBound                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 22    | Agent_0_AvoidedBound <= ego_f_2_agent_0_slack + ((agent_0_x - ego_x - (front_overhang + wheelbase)*cos(ego_theta))*sin(agent_0_theta_ref) + (-agent_0_y + ego_y + (front_overhang + wheelbase)*sin(ego_theta))*cos(agent_0_theta_ref))**2/agent_b_0_square + ((-agent_0_x + ego_x + (front_overhang + wheelbase)*cos(ego_theta))*cos(agent_0_theta_ref) + (-agent_0_y + ego_y + (front_overhang + wheelbase)*sin(ego_theta))*sin(agent_0_theta_ref))**2/agent_a_0_square |
| 23    | Agent_0_AvoidedBound <= ego_r_2_agent_0_slack + ((agent_0_x - ego_x + rear_overhang*cos(ego_theta))*sin(agent_0_theta_ref) + (-agent_0_y + ego_y - rear_overhang*sin(ego_theta))*cos(agent_0_theta_ref))**2/agent_b_0_square + ((-agent_0_x + ego_x - rear_overhang*cos(ego_theta))*cos(agent_0_theta_ref) + (-agent_0_y + ego_y - rear_overhang*sin(ego_theta))*sin(agent_0_theta_ref))**2/agent_a_0_square                                                             |
| 24    | Obs_0_AvoidedBound <= ego_f_2_object_0_slack + (object_cos_theta_0*(ego_y - object_y_0 + (front_overhang + wheelbase)*sin(ego_theta)) + object_sin_theta_0*(-ego_x + object_x_0 - (front_overhang + wheelbase)*cos(ego_theta)))**2/object_b_0_square + (object_cos_theta_0*(ego_x - object_x_0 + (front_overhang + wheelbase)*cos(ego_theta)) + object_sin_theta_0*(ego_y - object_y_0 + (front_overhang + wheelbase)*sin(ego_theta)))**2/object_a_0_square              |
| 25    | Obs_1_AvoidedBound <= ego_f_2_object_1_slack + (object_cos_theta_1*(ego_y - object_y_1 + (front_overhang + wheelbase)*sin(ego_theta)) + object_sin_theta_1*(-ego_x + object_x_1 - (front_overhang + wheelbase)*cos(ego_theta)))**2/object_b_1_square + (object_cos_theta_1*(ego_x - object_x_1 + (front_overhang + wheelbase)*cos(ego_theta)) + object_sin_theta_1*(ego_y - object_y_1 + (front_overhang + wheelbase)*sin(ego_theta)))**2/object_a_1_square              |
| 26    | Obs_0_AvoidedBound <= ego_r_2_object_0_slack + (object_cos_theta_0*(ego_y - object_y_0 - rear_overhang*sin(ego_theta)) + object_sin_theta_0*(-ego_x + object_x_0 + rear_overhang*cos(ego_theta)))**2/object_b_0_square + (object_cos_theta_0*(ego_x - object_x_0 - rear_overhang*cos(ego_theta)) + object_sin_theta_0*(ego_y - object_y_0 - rear_overhang*sin(ego_theta)))**2/object_a_0_square                                                                          |
| 27    | Obs_1_AvoidedBound <= ego_r_2_object_1_slack + (object_cos_theta_1*(ego_y - object_y_1 - rear_overhang*sin(ego_theta)) + object_sin_theta_1*(-ego_x + object_x_1 + rear_overhang*cos(ego_theta)))**2/object_b_1_square + (object_cos_theta_1*(ego_x - object_x_1 - rear_overhang*cos(ego_theta)) + object_sin_theta_1*(ego_y - object_y_1 - rear_overhang*sin(ego_theta)))**2/object_a_1_square                                                                          |

## Inital Constraint map

| Index | Name                                 |
| ----- | ------------------------------------ |
| 0     | ego_f_2_agent_0_slack >= 0.0         |
| 1     | ego_r_2_agent_0_slack >= 0.0         |
| 2     | ego_f_2_object_0_slack >= 0.0        |
| 3     | ego_r_2_object_0_slack >= 0.0        |
| 4     | ego_f_2_object_1_slack >= 0.0        |
| 5     | ego_r_2_object_1_slack >= 0.0        |
| 6     | SteerLowerBound <= ego_steer         |
| 7     | ego_steer <= SteerUpperBound         |
| 8     | AHardLowerBound <= ego_a             |
| 9     | ego_a <= AHardUpperBound             |
| 10    | Agent_0_AHardLowerBound <= agent_0_a |
| 11    | agent_0_a <= Agent_0_AHardUpperBound |

## Terminal Constraint map

| Index | Name                                                                                                            |
| ----- | --------------------------------------------------------------------------------------------------------------- |
| 0     | LHardLowerBound <= -(ego_x - ego_x_ref)*sin(ego_theta_ref) + (ego_y - ego_y_ref)*cos(ego_theta_ref)             |
| 1     | -(ego_x - ego_x_ref)*sin(ego_theta_ref) + (ego_y - ego_y_ref)*cos(ego_theta_ref) <= LHardUpperBound             |
| 2     | SHardLowerBound <= ego_s                                                                                        |
| 3     | ego_s <= SHardUpperBound                                                                                        |
| 4     | agent_0_s >= 0.0                                                                                                |
| 5     | VHardLowerBound <= ego_v                                                                                        |
| 6     | ego_v <= VHardUpperBound                                                                                        |
| 7     | agent_0_v >= 0.0                                                                                                |
| 8     | -ego_kappa_ref*(-(ego_x - ego_x_ref)*sin(ego_theta_ref) + (ego_y - ego_y_ref)*cos(ego_theta_ref)) + 1 >= 0.0001 |
| 9     | cos(ego_theta - ego_theta_ref) >= 0.0                                                                           |
