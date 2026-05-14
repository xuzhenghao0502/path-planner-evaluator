# ReferenceLineModel user manual

user manual for generated model ReferenceLineModel, code optimization has been applied for the best performance

## State variable map

| Index | Name  |
| ----- | ----- |
| 0     | x     |
| 1     | y     |
| 2     | theta |
| 3     | kappa |

## Ctrl variable map

| Index | Name         |
| ----- | ------------ |
| 0     | dkappa       |
| 1     | slack_offset |

## Slack variable map

| Index | Name |
| ----- | ---- |

## Paramter map

| Index | Name                  |
| ----- | --------------------- |
| 0     | xr                    |
| 1     | yr                    |
| 2     | thetar                |
| 3     | l_weight              |
| 4     | kappa_weight          |
| 5     | dkappa_weight         |
| 6     | v_weight              |
| 7     | slack_offset_weight   |
| 8     | l_weight_terminal     |
| 9     | kappa_weight_terminal |
| 10    | sll                   |
| 11    | slu                   |
| 12    | KappaLowerBound       |
| 13    | KappaUpperBound       |
| 14    | DKappaLowerBound      |
| 15    | DKappaUpperBound      |

## Constraint map

| Index | Name                                                               |
| ----- | ------------------------------------------------------------------ |
| 0     | cos(theta - thetar) >= 0.5                                         |
| 1     | kappa >= KappaLowerBound                                           |
| 2     | kappa <= KappaUpperBound                                           |
| 3     | dkappa <= DKappaUpperBound                                         |
| 4     | dkappa >= DKappaLowerBound                                         |
| 5     | sll <= slack_offset - (x - xr)*sin(thetar) + (y - yr)*cos(thetar)  |
| 6     | -slack_offset - (x - xr)*sin(thetar) + (y - yr)*cos(thetar) <= slu |

## Inital Constraint map

| Index | Name                       |
| ----- | -------------------------- |
| 0     | dkappa <= DKappaUpperBound |
| 1     | dkappa >= DKappaLowerBound |

## Terminal Constraint map

| Index | Name                       |
| ----- | -------------------------- |
| 0     | cos(theta - thetar) >= 0.5 |
| 1     | kappa >= KappaLowerBound   |
| 2     | kappa <= KappaUpperBound   |
