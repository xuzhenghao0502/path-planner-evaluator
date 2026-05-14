# LateralMultiGear user manual

user manual for generated model LateralMultiGear, code optimization has been applied for the best performance

## State variable map

| Index | Name  |
| ----- | ----- |
| 0     | x     |
| 1     | y     |
| 2     | theta |
| 3     | kappa |

## Ctrl variable map

| Index | Name   |
| ----- | ------ |
| 0     | dkappa |

## Slack variable map

| Index | Name |
| ----- | ---- |

## Paramter map

| Index | Name                     |
| ----- | ------------------------ |
| 0     | xr                       |
| 1     | yr                       |
| 2     | thetar                   |
| 3     | kr                       |
| 4     | vr                       |
| 5     | weight_position          |
| 6     | weight_heading           |
| 7     | weight_kappa             |
| 8     | weight_dkappa            |
| 9     | terminal_position_weight |
| 10    | terminal_heading_weight  |
| 11    | kappa_lower              |
| 12    | kappa_upper              |
| 13    | dkappa_lower             |
| 14    | dkappa_upper             |

## Constraint map

| Index | Name                   |
| ----- | ---------------------- |
| 0     | kappa_lower <= kappa   |
| 1     | kappa <= kappa_upper   |
| 2     | dkappa_lower <= dkappa |
| 3     | dkappa <= dkappa_upper |

## Inital Constraint map

| Index | Name                   |
| ----- | ---------------------- |
| 0     | dkappa_lower <= dkappa |
| 1     | dkappa <= dkappa_upper |

## Terminal Constraint map

| Index | Name                 |
| ----- | -------------------- |
| 0     | kappa_lower <= kappa |
| 1     | kappa <= kappa_upper |
