# ParkingGeneral user manual

user manual for generated model ParkingGeneral, code optimization has been applied for the best performance

## State variable map

| Index | Name  |
| ----- | ----- |
| 0     | s     |
| 1     | x     |
| 2     | y     |
| 3     | theta |
| 4     | steer |
| 5     | v     |

## Ctrl variable map

| Index | Name   |
| ----- | ------ |
| 0     | dsteer |
| 1     | a      |

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
| 5     | vr                      |
| 6     | ref_weight              |
| 7     | steer_weight            |
| 8     | dsteer_weight           |
| 9     | terminal_ref_weight     |
| 10    | terminal_heading_weight |
| 11    | terminal_v_weight       |
| 12    | v_weight                |
| 13    | a_weight                |
| 14    | weigth_slack_offset     |
| 15    | xrf                     |
| 16    | yrf                     |
| 17    | thetarf                 |
| 18    | xrr                     |
| 19    | yrr                     |
| 20    | thetarr                 |
| 21    | steer_lower             |
| 22    | steer_upper             |
| 23    | dsteer_lower            |
| 24    | dsteer_upper            |
| 25    | v_lower                 |
| 26    | v_upper                 |
| 27    | a_lower                 |
| 28    | a_upper                 |
| 29    | lf                      |
| 30    | lr                      |
| 31    | ll                      |
| 32    | lu                      |
| 33    | lfl                     |
| 34    | lfu                     |
| 35    | lrl                     |
| 36    | lru                     |
| 37    | sll                     |
| 38    | slu                     |
| 39    | slfl                    |
| 40    | slfu                    |
| 41    | slrl                    |
| 42    | slru                    |

## Constraint map

| Index | Name                                                                                                      |
| ----- | --------------------------------------------------------------------------------------------------------- |
| 0     | ll <= -(x - xr)*sin(thetar) + (y - yr)*cos(thetar)                                                        |
| 1     | -(x - xr)*sin(thetar) + (y - yr)*cos(thetar) <= lu                                                        |
| 2     | lfl <= (lf*sin(theta) + y - yrf)*cos(thetarf) - (lf*cos(theta) + x - xrf)*sin(thetarf)                    |
| 3     | (lf*sin(theta) + y - yrf)*cos(thetarf) - (lf*cos(theta) + x - xrf)*sin(thetarf) <= lfu                    |
| 4     | lrl <= (-lr*sin(theta) + y - yrr)*cos(thetarr) - (-lr*cos(theta) + x - xrr)*sin(thetarr)                  |
| 5     | (-lr*sin(theta) + y - yrr)*cos(thetarr) - (-lr*cos(theta) + x - xrr)*sin(thetarr) <= lru                  |
| 6     | steer_lower <= steer                                                                                      |
| 7     | steer <= steer_upper                                                                                      |
| 8     | v_lower <= v                                                                                              |
| 9     | v <= v_upper                                                                                              |
| 10    | cos(theta - thetar) >= 0.0                                                                                |
| 11    | dsteer_lower <= dsteer                                                                                    |
| 12    | dsteer <= dsteer_upper                                                                                    |
| 13    | a_lower <= a                                                                                              |
| 14    | a <= a_upper                                                                                              |
| 15    | sll <= slack_offset - (x - xr)*sin(thetar) + (y - yr)*cos(thetar)                                         |
| 16    | -slack_offset - (x - xr)*sin(thetar) + (y - yr)*cos(thetar) <= slu                                        |
| 17    | slfl <= slack_offset + (lf*sin(theta) + y - yrf)*cos(thetarf) - (lf*cos(theta) + x - xrf)*sin(thetarf)    |
| 18    | -slack_offset + (lf*sin(theta) + y - yrf)*cos(thetarf) - (lf*cos(theta) + x - xrf)*sin(thetarf) <= slfu   |
| 19    | slrl <= slack_offset + (-lr*sin(theta) + y - yrr)*cos(thetarr) - (-lr*cos(theta) + x - xrr)*sin(thetarr)  |
| 20    | -slack_offset + (-lr*sin(theta) + y - yrr)*cos(thetarr) - (-lr*cos(theta) + x - xrr)*sin(thetarr) <= slru |

## Inital Constraint map

| Index | Name                   |
| ----- | ---------------------- |
| 0     | dsteer_lower <= dsteer |
| 1     | dsteer <= dsteer_upper |
| 2     | a_lower <= a           |
| 3     | a <= a_upper           |

## Terminal Constraint map

| Index | Name                                                                                     |
| ----- | ---------------------------------------------------------------------------------------- |
| 0     | ll <= -(x - xr)*sin(thetar) + (y - yr)*cos(thetar)                                       |
| 1     | -(x - xr)*sin(thetar) + (y - yr)*cos(thetar) <= lu                                       |
| 2     | lfl <= (lf*sin(theta) + y - yrf)*cos(thetarf) - (lf*cos(theta) + x - xrf)*sin(thetarf)   |
| 3     | (lf*sin(theta) + y - yrf)*cos(thetarf) - (lf*cos(theta) + x - xrf)*sin(thetarf) <= lfu   |
| 4     | lrl <= (-lr*sin(theta) + y - yrr)*cos(thetarr) - (-lr*cos(theta) + x - xrr)*sin(thetarr) |
| 5     | (-lr*sin(theta) + y - yrr)*cos(thetarr) - (-lr*cos(theta) + x - xrr)*sin(thetarr) <= lru |
| 6     | steer_lower <= steer                                                                     |
| 7     | steer <= steer_upper                                                                     |
| 8     | v_lower <= v                                                                             |
| 9     | v <= v_upper                                                                             |
| 10    | cos(theta - thetar) >= 0.0                                                               |

## Terminal Equality map

| Index | Name           |
| ----- | -------------- |
| 0     | x - xr         |
| 1     | y - yr         |
| 2     | theta - thetar |
