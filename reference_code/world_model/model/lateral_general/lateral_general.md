# LateralGeneral user manual

user manual for generated model LateralGeneral, code optimization has been applied for the best performance

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

| Index | Name         |
| ----- | ------------ |
| 0     | slack_offset |

## Paramter map

| Index | Name                    |
| ----- | ----------------------- |
| 0     | wheelbase               |
| 1     | xr                      |
| 2     | yr                      |
| 3     | thetar                  |
| 4     | kr                      |
| 5     | l_weight                |
| 6     | theta_ref_weight        |
| 7     | steer_weight            |
| 8     | dsteer_weight           |
| 9     | weight_slack_offset     |
| 10    | terminal_l_weight       |
| 11    | terminal_heading_weight |
| 12    | xrf                     |
| 13    | yrf                     |
| 14    | thetarf                 |
| 15    | xrr                     |
| 16    | yrr                     |
| 17    | thetarr                 |
| 18    | lf                      |
| 19    | lr                      |
| 20    | ll                      |
| 21    | lu                      |
| 22    | sll                     |
| 23    | slu                     |
| 24    | lfl                     |
| 25    | lfu                     |
| 26    | slfl                    |
| 27    | slfu                    |
| 28    | lrl                     |
| 29    | lru                     |
| 30    | slrl                    |
| 31    | slru                    |
| 32    | SteerLowerBound         |
| 33    | SteerUpperBound         |
| 34    | DSteerLowerBound        |
| 35    | DSteerUpperBound        |
| 36    | weight_prev_steer       |
| 37    | weight_prev_dsteer      |
| 38    | prev_steer              |
| 39    | prev_dsteer             |
| 40    | l_offset                |
| 41    | vr                      |

## Constraint map

| Index | Name                                                                                                      |
| ----- | --------------------------------------------------------------------------------------------------------- |
| 0     | ll <= -(x - xr)*sin(thetar) + (y - yr)*cos(thetar)                                                        |
| 1     | -(x - xr)*sin(thetar) + (y - yr)*cos(thetar) <= lu                                                        |
| 2     | lfl <= (lf*sin(theta) + y - yrf)*cos(thetarf) - (lf*cos(theta) + x - xrf)*sin(thetarf)                    |
| 3     | (lf*sin(theta) + y - yrf)*cos(thetarf) - (lf*cos(theta) + x - xrf)*sin(thetarf) <= lfu                    |
| 4     | lrl <= (-lr*sin(theta) + y - yrr)*cos(thetarr) - (-lr*cos(theta) + x - xrr)*sin(thetarr)                  |
| 5     | (-lr*sin(theta) + y - yrr)*cos(thetarr) - (-lr*cos(theta) + x - xrr)*sin(thetarr) <= lru                  |
| 6     | steer >= SteerLowerBound                                                                                  |
| 7     | steer <= SteerUpperBound                                                                                  |
| 8     | cos(theta - thetar) >= 0.0                                                                                |
| 9     | dsteer >= DSteerLowerBound                                                                                |
| 10    | dsteer <= DSteerUpperBound                                                                                |
| 11    | sll <= slack_offset - (x - xr)*sin(thetar) + (y - yr)*cos(thetar)                                         |
| 12    | -slack_offset - (x - xr)*sin(thetar) + (y - yr)*cos(thetar) <= slu                                        |
| 13    | slfl <= slack_offset + (lf*sin(theta) + y - yrf)*cos(thetarf) - (lf*cos(theta) + x - xrf)*sin(thetarf)    |
| 14    | -slack_offset + (lf*sin(theta) + y - yrf)*cos(thetarf) - (lf*cos(theta) + x - xrf)*sin(thetarf) <= slfu   |
| 15    | slrl <= slack_offset + (-lr*sin(theta) + y - yrr)*cos(thetarr) - (-lr*cos(theta) + x - xrr)*sin(thetarr)  |
| 16    | -slack_offset + (-lr*sin(theta) + y - yrr)*cos(thetarr) - (-lr*cos(theta) + x - xrr)*sin(thetarr) <= slru |

## Inital Constraint map

| Index | Name                       |
| ----- | -------------------------- |
| 0     | dsteer >= DSteerLowerBound |
| 1     | dsteer <= DSteerUpperBound |

## Terminal Constraint map

| Index | Name                                                                                     |
| ----- | ---------------------------------------------------------------------------------------- |
| 0     | ll <= -(x - xr)*sin(thetar) + (y - yr)*cos(thetar)                                       |
| 1     | -(x - xr)*sin(thetar) + (y - yr)*cos(thetar) <= lu                                       |
| 2     | lfl <= (lf*sin(theta) + y - yrf)*cos(thetarf) - (lf*cos(theta) + x - xrf)*sin(thetarf)   |
| 3     | (lf*sin(theta) + y - yrf)*cos(thetarf) - (lf*cos(theta) + x - xrf)*sin(thetarf) <= lfu   |
| 4     | lrl <= (-lr*sin(theta) + y - yrr)*cos(thetarr) - (-lr*cos(theta) + x - xrr)*sin(thetarr) |
| 5     | (-lr*sin(theta) + y - yrr)*cos(thetarr) - (-lr*cos(theta) + x - xrr)*sin(thetarr) <= lru |
| 6     | steer >= SteerLowerBound                                                                 |
| 7     | steer <= SteerUpperBound                                                                 |
| 8     | cos(theta - thetar) >= 0.0                                                               |
