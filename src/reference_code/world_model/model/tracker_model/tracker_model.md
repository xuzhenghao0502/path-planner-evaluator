# TrackerModel user manual

user manual for generated model TrackerModel, code optimization has been applied for the best performance

## State variable map

| Index | Name  |
| ----- | ----- |
| 0     | s     |
| 1     | x     |
| 2     | y     |
| 3     | theta |
| 4     | steer |

## Ctrl variable map

| Index | Name   |
| ----- | ------ |
| 0     | dsteer |

## Slack variable map

| Index | Name    |
| ----- | ------- |
| 0     | l_slack |

## Paramter map

| Index | Name                  |
| ----- | --------------------- |
| 0     | x_ref                 |
| 1     | y_ref                 |
| 2     | theta_ref             |
| 3     | kappa_ref             |
| 4     | v_ref                 |
| 5     | l_terminal            |
| 6     | wheelbase             |
| 7     | front_overhang        |
| 8     | rear_overhang         |
| 9     | length                |
| 10    | width                 |
| 11    | l_ref_weight          |
| 12    | theta_ref_weight      |
| 13    | steer_weight          |
| 14    | dsteer_weight         |
| 15    | l_slack_weight        |
| 16    | terminal_l_weight     |
| 17    | terminal_theta_weight |
| 18    | LHardLowerBound       |
| 19    | LHardUpperBound       |
| 20    | SteerLowerBound       |
| 21    | SteerUpperBound       |
| 22    | DSteerLowerBound      |
| 23    | DSteerUpperBound      |
| 24    | l_scale               |
| 25    | theta_scale           |
| 26    | steer_scale           |
| 27    | dsteer_scale          |

## Constraint map

| Index | Name                                                                                  |
| ----- | ------------------------------------------------------------------------------------- |
| 0     | SteerLowerBound <= steer                                                              |
| 1     | steer <= SteerUpperBound                                                              |
| 2     | cos(theta - theta_ref) >= 0.0                                                         |
| 3     | DSteerLowerBound <= dsteer                                                            |
| 4     | dsteer <= DSteerUpperBound                                                            |
| 5     | LHardLowerBound <= l_slack - (x - x_ref)*sin(theta_ref) + (y - y_ref)*cos(theta_ref)  |
| 6     | -l_slack - (x - x_ref)*sin(theta_ref) + (y - y_ref)*cos(theta_ref) <= LHardUpperBound |

## Inital Constraint map

| Index | Name                       |
| ----- | -------------------------- |
| 0     | DSteerLowerBound <= dsteer |
| 1     | dsteer <= DSteerUpperBound |

## Terminal Constraint map

| Index | Name                          |
| ----- | ----------------------------- |
| 0     | SteerLowerBound <= steer      |
| 1     | steer <= SteerUpperBound      |
| 2     | cos(theta - theta_ref) >= 0.0 |
