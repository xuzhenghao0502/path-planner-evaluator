# ParkLateralGeneral user manual

user manual for generated model ParkLateralGeneral, code optimization has been applied for the best performance

## State variable map

| Index | Name  |
| ----- | ----- |
| 0     | s     |
| 1     | x     |
| 2     | y     |
| 3     | theta |
| 4     | kappa |

## Ctrl variable map

| Index | Name   |
| ----- | ------ |
| 0     | dkappa |

## Slack variable map

| Index | Name         |
| ----- | ------------ |
| 0     | slack_offset |

## Paramter map

| Index | Name                     |
| ----- | ------------------------ |
| 0     | xr                       |
| 1     | yr                       |
| 2     | thetar                   |
| 3     | kr                       |
| 4     | vr                       |
| 5     | ref_weight               |
| 6     | kappa_weight             |
| 7     | dkappa_weight            |
| 8     | weigth_slack_offset      |
| 9     | terminal_position_weight |
| 10    | terminal_heading_weight  |
| 11    | xrf                      |
| 12    | yrf                      |
| 13    | thetarf                  |
| 14    | xrr                      |
| 15    | yrr                      |
| 16    | thetarr                  |
| 17    | lf                       |
| 18    | lr                       |
| 19    | ll                       |
| 20    | lu                       |
| 21    | lfl                      |
| 22    | lfu                      |
| 23    | lrl                      |
| 24    | lru                      |
| 25    | sll                      |
| 26    | slu                      |
| 27    | KappaLowerBound          |
| 28    | KappaUpperBound          |
| 29    | DKappaLowerBound         |
| 30    | DKappaUpperBound         |
| 31    | weigth_prev_kappa        |
| 32    | weight_prev_dkappa       |
| 33    | prev_kappa               |
| 34    | prev_dkappa              |
| 35    | slfl                     |
| 36    | slfu                     |
| 37    | slrl                     |
| 38    | slru                     |

## Constraint map

| Index | Name                                                                                                      |
| ----- | --------------------------------------------------------------------------------------------------------- |
| 0     | ll <= -(x - xr)*sin(thetar) + (y - yr)*cos(thetar)                                                        |
| 1     | -(x - xr)*sin(thetar) + (y - yr)*cos(thetar) <= lu                                                        |
| 2     | lfl <= (lf*sin(theta) + y - yrf)*cos(thetarf) - (lf*cos(theta) + x - xrf)*sin(thetarf)                    |
| 3     | (lf*sin(theta) + y - yrf)*cos(thetarf) - (lf*cos(theta) + x - xrf)*sin(thetarf) <= lfu                    |
| 4     | lrl <= (-lr*sin(theta) + y - yrr)*cos(thetarr) - (-lr*cos(theta) + x - xrr)*sin(thetarr)                  |
| 5     | (-lr*sin(theta) + y - yrr)*cos(thetarr) - (-lr*cos(theta) + x - xrr)*sin(thetarr) <= lru                  |
| 6     | kappa >= KappaLowerBound                                                                                  |
| 7     | kappa <= KappaUpperBound                                                                                  |
| 8     | cos(theta - thetar) >= 0.0                                                                                |
| 9     | dkappa <= DKappaUpperBound                                                                                |
| 10    | dkappa >= DKappaLowerBound                                                                                |
| 11    | sll <= slack_offset - (x - xr)*sin(thetar) + (y - yr)*cos(thetar)                                         |
| 12    | -slack_offset - (x - xr)*sin(thetar) + (y - yr)*cos(thetar) <= slu                                        |
| 13    | slfl <= slack_offset + (lf*sin(theta) + y - yrf)*cos(thetarf) - (lf*cos(theta) + x - xrf)*sin(thetarf)    |
| 14    | -slack_offset + (lf*sin(theta) + y - yrf)*cos(thetarf) - (lf*cos(theta) + x - xrf)*sin(thetarf) <= slfu   |
| 15    | slrl <= slack_offset + (-lr*sin(theta) + y - yrr)*cos(thetarr) - (-lr*cos(theta) + x - xrr)*sin(thetarr)  |
| 16    | -slack_offset + (-lr*sin(theta) + y - yrr)*cos(thetarr) - (-lr*cos(theta) + x - xrr)*sin(thetarr) <= slru |

## Inital Constraint map

| Index | Name                       |
| ----- | -------------------------- |
| 0     | dkappa <= DKappaUpperBound |
| 1     | dkappa >= DKappaLowerBound |

## Terminal Constraint map

| Index | Name                                                                                     |
| ----- | ---------------------------------------------------------------------------------------- |
| 0     | ll <= -(x - xr)*sin(thetar) + (y - yr)*cos(thetar)                                       |
| 1     | -(x - xr)*sin(thetar) + (y - yr)*cos(thetar) <= lu                                       |
| 2     | lfl <= (lf*sin(theta) + y - yrf)*cos(thetarf) - (lf*cos(theta) + x - xrf)*sin(thetarf)   |
| 3     | (lf*sin(theta) + y - yrf)*cos(thetarf) - (lf*cos(theta) + x - xrf)*sin(thetarf) <= lfu   |
| 4     | lrl <= (-lr*sin(theta) + y - yrr)*cos(thetarr) - (-lr*cos(theta) + x - xrr)*sin(thetarr) |
| 5     | (-lr*sin(theta) + y - yrr)*cos(thetarr) - (-lr*cos(theta) + x - xrr)*sin(thetarr) <= lru |
| 6     | kappa >= KappaLowerBound                                                                 |
| 7     | kappa <= KappaUpperBound                                                                 |
| 8     | cos(theta - thetar) >= 0.0                                                               |
