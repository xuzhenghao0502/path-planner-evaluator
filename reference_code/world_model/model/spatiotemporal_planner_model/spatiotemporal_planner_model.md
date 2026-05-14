# SpatiotemporalPlannerModel user manual

user manual for generated model SpatiotemporalPlannerModel, code optimization has been applied for the best performance

## State variable map

| Index | Name  |
| ----- | ----- |
| 0     | s     |
| 1     | x     |
| 2     | y     |
| 3     | theta |
| 4     | steer |
| 5     | v     |
| 6     | a     |

## Ctrl variable map

| Index | Name   |
| ----- | ------ |
| 0     | dsteer |
| 1     | jerk   |

## Slack variable map

| Index | Name        |
| ----- | ----------- |
| 0     | l_slack     |
| 1     | s_slack     |
| 2     | v_slack     |
| 3     | obs_0_slack |
| 4     | obs_1_slack |
| 5     | obs_2_slack |
| 6     | obs_3_slack |
| 7     | obs_4_slack |
| 8     | obs_5_slack |
| 9     | obs_6_slack |
| 10    | obs_7_slack |

## Paramter map

| Index | Name                  |
| ----- | --------------------- |
| 0     | x_ref                 |
| 1     | y_ref                 |
| 2     | theta_ref             |
| 3     | kappa_ref             |
| 4     | steer_ref             |
| 5     | s_coarse              |
| 6     | v_coarse              |
| 7     | a_coarse              |
| 8     | l_terminal            |
| 9     | theta_terminal        |
| 10    | s_terminal            |
| 11    | v_terminal            |
| 12    | l_offset              |
| 13    | a_offset              |
| 14    | wheelbase             |
| 15    | front_overhang        |
| 16    | rear_overhang         |
| 17    | length                |
| 18    | width                 |
| 19    | weight_prev_steer     |
| 20    | prev_steer            |
| 21    | l_ref_weight          |
| 22    | theta_ref_weight      |
| 23    | steer_weight          |
| 24    | dsteer_weight         |
| 25    | s_coarse_weight       |
| 26    | v_coarse_weight       |
| 27    | a_weight              |
| 28    | jerk_weight           |
| 29    | obs_0_weight          |
| 30    | obs_1_weight          |
| 31    | obs_2_weight          |
| 32    | obs_3_weight          |
| 33    | obs_4_weight          |
| 34    | obs_5_weight          |
| 35    | obs_6_weight          |
| 36    | obs_7_weight          |
| 37    | l_slack_weight        |
| 38    | s_slack_weight        |
| 39    | v_slack_weight        |
| 40    | terminal_l_weight     |
| 41    | terminal_theta_weight |
| 42    | terminal_s_weight     |
| 43    | terminal_v_weight     |
| 44    | terminal_a_weight     |
| 45    | xf_ref                |
| 46    | yf_ref                |
| 47    | thetaf_ref            |
| 48    | xr_ref                |
| 49    | yr_ref                |
| 50    | thetar_ref            |
| 51    | Obs_0_AvoidedBound    |
| 52    | Obs_1_AvoidedBound    |
| 53    | Obs_2_AvoidedBound    |
| 54    | Obs_3_AvoidedBound    |
| 55    | Obs_4_AvoidedBound    |
| 56    | Obs_5_AvoidedBound    |
| 57    | Obs_6_AvoidedBound    |
| 58    | Obs_7_AvoidedBound    |
| 59    | LHardLowerBound       |
| 60    | LHardUpperBound       |
| 61    | LFHardLowerBound      |
| 62    | LFHardUpperBound      |
| 63    | LRHardLowerBound      |
| 64    | LRHardUpperBound      |
| 65    | LSoftLowerBound       |
| 66    | LSoftUpperBound       |
| 67    | LFSoftLowerBound      |
| 68    | LFSoftUpperBound      |
| 69    | LRSoftLowerBound      |
| 70    | LRSoftUpperBound      |
| 71    | SteerLowerBound       |
| 72    | SteerUpperBound       |
| 73    | DSteerLowerBound      |
| 74    | DSteerUpperBound      |
| 75    | SHardLowerBound       |
| 76    | SHardUpperBound       |
| 77    | VHardLowerBound       |
| 78    | VHardUpperBound       |
| 79    | AHardLowerBound       |
| 80    | AHardUpperBound       |
| 81    | JHardLowerBound       |
| 82    | JHardUpperBound       |
| 83    | SSoftLowerBound       |
| 84    | SSoftUpperBound       |
| 85    | VSoftLowerBound       |
| 86    | VSoftUpperBound       |
| 87    | ASoftLowerBound       |
| 88    | ASoftUpperBound       |
| 89    | a_0                   |
| 90    | a_0_square            |
| 91    | b_0                   |
| 92    | b_0_square            |
| 93    | x_0                   |
| 94    | y_0                   |
| 95    | theta_0               |
| 96    | cos_theta_0           |
| 97    | sin_theta_0           |
| 98    | v_0                   |
| 99    | length_0              |
| 100   | width_0               |
| 101   | a_1                   |
| 102   | a_1_square            |
| 103   | b_1                   |
| 104   | b_1_square            |
| 105   | x_1                   |
| 106   | y_1                   |
| 107   | theta_1               |
| 108   | cos_theta_1           |
| 109   | sin_theta_1           |
| 110   | v_1                   |
| 111   | length_1              |
| 112   | width_1               |
| 113   | a_2                   |
| 114   | a_2_square            |
| 115   | b_2                   |
| 116   | b_2_square            |
| 117   | x_2                   |
| 118   | y_2                   |
| 119   | theta_2               |
| 120   | cos_theta_2           |
| 121   | sin_theta_2           |
| 122   | v_2                   |
| 123   | length_2              |
| 124   | width_2               |
| 125   | a_3                   |
| 126   | a_3_square            |
| 127   | b_3                   |
| 128   | b_3_square            |
| 129   | x_3                   |
| 130   | y_3                   |
| 131   | theta_3               |
| 132   | cos_theta_3           |
| 133   | sin_theta_3           |
| 134   | v_3                   |
| 135   | length_3              |
| 136   | width_3               |
| 137   | a_4                   |
| 138   | a_4_square            |
| 139   | b_4                   |
| 140   | b_4_square            |
| 141   | x_4                   |
| 142   | y_4                   |
| 143   | theta_4               |
| 144   | cos_theta_4           |
| 145   | sin_theta_4           |
| 146   | v_4                   |
| 147   | length_4              |
| 148   | width_4               |
| 149   | a_5                   |
| 150   | a_5_square            |
| 151   | b_5                   |
| 152   | b_5_square            |
| 153   | x_5                   |
| 154   | y_5                   |
| 155   | theta_5               |
| 156   | cos_theta_5           |
| 157   | sin_theta_5           |
| 158   | v_5                   |
| 159   | length_5              |
| 160   | width_5               |
| 161   | a_6                   |
| 162   | a_6_square            |
| 163   | b_6                   |
| 164   | b_6_square            |
| 165   | x_6                   |
| 166   | y_6                   |
| 167   | theta_6               |
| 168   | cos_theta_6           |
| 169   | sin_theta_6           |
| 170   | v_6                   |
| 171   | length_6              |
| 172   | width_6               |
| 173   | a_7                   |
| 174   | a_7_square            |
| 175   | b_7                   |
| 176   | b_7_square            |
| 177   | x_7                   |
| 178   | y_7                   |
| 179   | theta_7               |
| 180   | cos_theta_7           |
| 181   | sin_theta_7           |
| 182   | v_7                   |
| 183   | length_7              |
| 184   | width_7               |
| 185   | s_scale               |
| 186   | v_scale               |
| 187   | a_scale               |
| 188   | jerk_scale            |
| 189   | l_scale               |
| 190   | theta_scale           |
| 191   | steer_scale           |
| 192   | dsteer_scale          |

## Constraint map

| Index | Name                                                                                                                                                                                                                                                                                                                                       |
| ----- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| 0     | LHardLowerBound <= -(x - x_ref)*sin(theta_ref) + (y - y_ref)*cos(theta_ref)                                                                                                                                                                                                                                                                |
| 1     | -(x - x_ref)*sin(theta_ref) + (y - y_ref)*cos(theta_ref) <= LHardUpperBound                                                                                                                                                                                                                                                                |
| 2     | LFHardLowerBound <= -(x - xf_ref + (front_overhang + wheelbase)*cos(theta))*sin(thetaf_ref) + (y - yf_ref + (front_overhang + wheelbase)*sin(theta))*cos(thetaf_ref)                                                                                                                                                                       |
| 3     | -(x - xf_ref + (front_overhang + wheelbase)*cos(theta))*sin(thetaf_ref) + (y - yf_ref + (front_overhang + wheelbase)*sin(theta))*cos(thetaf_ref) <= LFHardUpperBound                                                                                                                                                                       |
| 4     | LRHardLowerBound <= (-rear_overhang*sin(theta) + y - yr_ref)*cos(thetar_ref) - (-rear_overhang*cos(theta) + x - xr_ref)*sin(thetar_ref)                                                                                                                                                                                                    |
| 5     | (-rear_overhang*sin(theta) + y - yr_ref)*cos(thetar_ref) - (-rear_overhang*cos(theta) + x - xr_ref)*sin(thetar_ref) <= LRHardUpperBound                                                                                                                                                                                                    |
| 6     | SteerLowerBound <= steer                                                                                                                                                                                                                                                                                                                   |
| 7     | steer <= SteerUpperBound                                                                                                                                                                                                                                                                                                                   |
| 8     | SHardLowerBound <= s                                                                                                                                                                                                                                                                                                                       |
| 9     | s <= SHardUpperBound                                                                                                                                                                                                                                                                                                                       |
| 10    | VHardLowerBound <= v                                                                                                                                                                                                                                                                                                                       |
| 11    | v <= VHardUpperBound                                                                                                                                                                                                                                                                                                                       |
| 12    | AHardLowerBound <= a                                                                                                                                                                                                                                                                                                                       |
| 13    | a <= AHardUpperBound                                                                                                                                                                                                                                                                                                                       |
| 14    | cos(theta - theta_ref) >= 0.0                                                                                                                                                                                                                                                                                                              |
| 15    | DSteerLowerBound <= dsteer                                                                                                                                                                                                                                                                                                                 |
| 16    | dsteer <= DSteerUpperBound                                                                                                                                                                                                                                                                                                                 |
| 17    | JHardLowerBound <= jerk                                                                                                                                                                                                                                                                                                                    |
| 18    | jerk <= JHardUpperBound                                                                                                                                                                                                                                                                                                                    |
| 19    | Obs_0_AvoidedBound <= obs_0_slack + (cos_theta_0*(y - y_0 + (front_overhang + wheelbase)*sin(theta)) + sin_theta_0*(-x + x_0 - (front_overhang + wheelbase)*cos(theta)))**2/b_0_square + (cos_theta_0*(x - x_0 + (front_overhang + wheelbase)*cos(theta)) + sin_theta_0*(y - y_0 + (front_overhang + wheelbase)*sin(theta)))**2/a_0_square |
| 20    | Obs_1_AvoidedBound <= obs_1_slack + (cos_theta_1*(y - y_1 + (front_overhang + wheelbase)*sin(theta)) + sin_theta_1*(-x + x_1 - (front_overhang + wheelbase)*cos(theta)))**2/b_1_square + (cos_theta_1*(x - x_1 + (front_overhang + wheelbase)*cos(theta)) + sin_theta_1*(y - y_1 + (front_overhang + wheelbase)*sin(theta)))**2/a_1_square |
| 21    | Obs_2_AvoidedBound <= obs_2_slack + (cos_theta_2*(y - y_2 + (front_overhang + wheelbase)*sin(theta)) + sin_theta_2*(-x + x_2 - (front_overhang + wheelbase)*cos(theta)))**2/b_2_square + (cos_theta_2*(x - x_2 + (front_overhang + wheelbase)*cos(theta)) + sin_theta_2*(y - y_2 + (front_overhang + wheelbase)*sin(theta)))**2/a_2_square |
| 22    | Obs_3_AvoidedBound <= obs_3_slack + (cos_theta_3*(y - y_3 + (front_overhang + wheelbase)*sin(theta)) + sin_theta_3*(-x + x_3 - (front_overhang + wheelbase)*cos(theta)))**2/b_3_square + (cos_theta_3*(x - x_3 + (front_overhang + wheelbase)*cos(theta)) + sin_theta_3*(y - y_3 + (front_overhang + wheelbase)*sin(theta)))**2/a_3_square |
| 23    | Obs_4_AvoidedBound <= obs_4_slack + (cos_theta_4*(y - y_4 + (front_overhang + wheelbase)*sin(theta)) + sin_theta_4*(-x + x_4 - (front_overhang + wheelbase)*cos(theta)))**2/b_4_square + (cos_theta_4*(x - x_4 + (front_overhang + wheelbase)*cos(theta)) + sin_theta_4*(y - y_4 + (front_overhang + wheelbase)*sin(theta)))**2/a_4_square |
| 24    | Obs_5_AvoidedBound <= obs_5_slack + (cos_theta_5*(y - y_5 + (front_overhang + wheelbase)*sin(theta)) + sin_theta_5*(-x + x_5 - (front_overhang + wheelbase)*cos(theta)))**2/b_5_square + (cos_theta_5*(x - x_5 + (front_overhang + wheelbase)*cos(theta)) + sin_theta_5*(y - y_5 + (front_overhang + wheelbase)*sin(theta)))**2/a_5_square |
| 25    | Obs_6_AvoidedBound <= obs_6_slack + (cos_theta_6*(y - y_6 + (front_overhang + wheelbase)*sin(theta)) + sin_theta_6*(-x + x_6 - (front_overhang + wheelbase)*cos(theta)))**2/b_6_square + (cos_theta_6*(x - x_6 + (front_overhang + wheelbase)*cos(theta)) + sin_theta_6*(y - y_6 + (front_overhang + wheelbase)*sin(theta)))**2/a_6_square |
| 26    | Obs_7_AvoidedBound <= obs_7_slack + (cos_theta_7*(y - y_7 + (front_overhang + wheelbase)*sin(theta)) + sin_theta_7*(-x + x_7 - (front_overhang + wheelbase)*cos(theta)))**2/b_7_square + (cos_theta_7*(x - x_7 + (front_overhang + wheelbase)*cos(theta)) + sin_theta_7*(y - y_7 + (front_overhang + wheelbase)*sin(theta)))**2/a_7_square |
| 27    | Obs_0_AvoidedBound <= obs_0_slack + (cos_theta_0*(-rear_overhang*sin(theta) + y - y_0) + sin_theta_0*(rear_overhang*cos(theta) - x + x_0))**2/b_0_square + (cos_theta_0*(-rear_overhang*cos(theta) + x - x_0) + sin_theta_0*(-rear_overhang*sin(theta) + y - y_0))**2/a_0_square                                                           |
| 28    | Obs_1_AvoidedBound <= obs_1_slack + (cos_theta_1*(-rear_overhang*sin(theta) + y - y_1) + sin_theta_1*(rear_overhang*cos(theta) - x + x_1))**2/b_1_square + (cos_theta_1*(-rear_overhang*cos(theta) + x - x_1) + sin_theta_1*(-rear_overhang*sin(theta) + y - y_1))**2/a_1_square                                                           |
| 29    | Obs_2_AvoidedBound <= obs_2_slack + (cos_theta_2*(-rear_overhang*sin(theta) + y - y_2) + sin_theta_2*(rear_overhang*cos(theta) - x + x_2))**2/b_2_square + (cos_theta_2*(-rear_overhang*cos(theta) + x - x_2) + sin_theta_2*(-rear_overhang*sin(theta) + y - y_2))**2/a_2_square                                                           |
| 30    | Obs_3_AvoidedBound <= obs_3_slack + (cos_theta_3*(-rear_overhang*sin(theta) + y - y_3) + sin_theta_3*(rear_overhang*cos(theta) - x + x_3))**2/b_3_square + (cos_theta_3*(-rear_overhang*cos(theta) + x - x_3) + sin_theta_3*(-rear_overhang*sin(theta) + y - y_3))**2/a_3_square                                                           |
| 31    | Obs_4_AvoidedBound <= obs_4_slack + (cos_theta_4*(-rear_overhang*sin(theta) + y - y_4) + sin_theta_4*(rear_overhang*cos(theta) - x + x_4))**2/b_4_square + (cos_theta_4*(-rear_overhang*cos(theta) + x - x_4) + sin_theta_4*(-rear_overhang*sin(theta) + y - y_4))**2/a_4_square                                                           |
| 32    | Obs_5_AvoidedBound <= obs_5_slack + (cos_theta_5*(-rear_overhang*sin(theta) + y - y_5) + sin_theta_5*(rear_overhang*cos(theta) - x + x_5))**2/b_5_square + (cos_theta_5*(-rear_overhang*cos(theta) + x - x_5) + sin_theta_5*(-rear_overhang*sin(theta) + y - y_5))**2/a_5_square                                                           |
| 33    | Obs_6_AvoidedBound <= obs_6_slack + (cos_theta_6*(-rear_overhang*sin(theta) + y - y_6) + sin_theta_6*(rear_overhang*cos(theta) - x + x_6))**2/b_6_square + (cos_theta_6*(-rear_overhang*cos(theta) + x - x_6) + sin_theta_6*(-rear_overhang*sin(theta) + y - y_6))**2/a_6_square                                                           |
| 34    | Obs_7_AvoidedBound <= obs_7_slack + (cos_theta_7*(-rear_overhang*sin(theta) + y - y_7) + sin_theta_7*(rear_overhang*cos(theta) - x + x_7))**2/b_7_square + (cos_theta_7*(-rear_overhang*cos(theta) + x - x_7) + sin_theta_7*(-rear_overhang*sin(theta) + y - y_7))**2/a_7_square                                                           |
| 35    | LSoftLowerBound <= l_slack - (x - x_ref)*sin(theta_ref) + (y - y_ref)*cos(theta_ref)                                                                                                                                                                                                                                                       |
| 36    | -l_slack - (x - x_ref)*sin(theta_ref) + (y - y_ref)*cos(theta_ref) <= LSoftUpperBound                                                                                                                                                                                                                                                      |
| 37    | LFSoftLowerBound <= l_slack - (x - xf_ref + (front_overhang + wheelbase)*cos(theta))*sin(thetaf_ref) + (y - yf_ref + (front_overhang + wheelbase)*sin(theta))*cos(thetaf_ref)                                                                                                                                                              |
| 38    | -l_slack - (x - xf_ref + (front_overhang + wheelbase)*cos(theta))*sin(thetaf_ref) + (y - yf_ref + (front_overhang + wheelbase)*sin(theta))*cos(thetaf_ref) <= LFSoftUpperBound                                                                                                                                                             |
| 39    | LRSoftLowerBound <= l_slack + (-rear_overhang*sin(theta) + y - yr_ref)*cos(thetar_ref) - (-rear_overhang*cos(theta) + x - xr_ref)*sin(thetar_ref)                                                                                                                                                                                          |
| 40    | -l_slack + (-rear_overhang*sin(theta) + y - yr_ref)*cos(thetar_ref) - (-rear_overhang*cos(theta) + x - xr_ref)*sin(thetar_ref) <= LRSoftUpperBound                                                                                                                                                                                         |
| 41    | s - s_slack <= SSoftUpperBound                                                                                                                                                                                                                                                                                                             |
| 42    | v - v_slack <= VSoftUpperBound                                                                                                                                                                                                                                                                                                             |

## Inital Constraint map

| Index | Name                       |
| ----- | -------------------------- |
| 0     | DSteerLowerBound <= dsteer |
| 1     | dsteer <= DSteerUpperBound |
| 2     | JHardLowerBound <= jerk    |
| 3     | jerk <= JHardUpperBound    |

## Terminal Constraint map

| Index | Name                                                                                                                                                                 |
| ----- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 0     | LHardLowerBound <= -(x - x_ref)*sin(theta_ref) + (y - y_ref)*cos(theta_ref)                                                                                          |
| 1     | -(x - x_ref)*sin(theta_ref) + (y - y_ref)*cos(theta_ref) <= LHardUpperBound                                                                                          |
| 2     | LFHardLowerBound <= -(x - xf_ref + (front_overhang + wheelbase)*cos(theta))*sin(thetaf_ref) + (y - yf_ref + (front_overhang + wheelbase)*sin(theta))*cos(thetaf_ref) |
| 3     | -(x - xf_ref + (front_overhang + wheelbase)*cos(theta))*sin(thetaf_ref) + (y - yf_ref + (front_overhang + wheelbase)*sin(theta))*cos(thetaf_ref) <= LFHardUpperBound |
| 4     | LRHardLowerBound <= (-rear_overhang*sin(theta) + y - yr_ref)*cos(thetar_ref) - (-rear_overhang*cos(theta) + x - xr_ref)*sin(thetar_ref)                              |
| 5     | (-rear_overhang*sin(theta) + y - yr_ref)*cos(thetar_ref) - (-rear_overhang*cos(theta) + x - xr_ref)*sin(thetar_ref) <= LRHardUpperBound                              |
| 6     | SteerLowerBound <= steer                                                                                                                                             |
| 7     | steer <= SteerUpperBound                                                                                                                                             |
| 8     | SHardLowerBound <= s                                                                                                                                                 |
| 9     | s <= SHardUpperBound                                                                                                                                                 |
| 10    | VHardLowerBound <= v                                                                                                                                                 |
| 11    | v <= VHardUpperBound                                                                                                                                                 |
| 12    | AHardLowerBound <= a                                                                                                                                                 |
| 13    | a <= AHardUpperBound                                                                                                                                                 |
| 14    | cos(theta - theta_ref) >= 0.0                                                                                                                                        |
