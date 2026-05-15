# BicycleTrajectoryTracker user manual

user manual for generated model BicycleTrajectoryTracker, code optimization has been applied for the best performance

## State variable map

| Index | Name  |
| ----- | ----- |
| 0     | x     |
| 1     | y     |
| 2     | theta |
| 3     | v     |
| 4     | kappa |

## Ctrl variable map

| Index | Name   |
| ----- | ------ |
| 0     | a      |
| 1     | dkappa |

## Slack variable map

| Index | Name |
| ----- | ---- |

## Paramter map

| Index | Name            |
| ----- | --------------- |
| 0     | xr              |
| 1     | yr              |
| 2     | thetar          |
| 3     | vr              |
| 4     | kr              |
| 5     | weight_position |
| 6     | weight_heading  |
| 7     | weight_v        |
| 8     | weight_kappa    |
| 9     | weight_a        |
| 10    | weight_dkappa   |
| 11    | v_lower         |
| 12    | v_upper         |
| 13    | a_lower         |
| 14    | a_upper         |
| 15    | kappa_lower     |
| 16    | kappa_upper     |
| 17    | dkappa_lower    |
| 18    | dkappa_upper    |

## Constraint map

| Index | Name                   |
| ----- | ---------------------- |
| 0     | v_lower <= v           |
| 1     | v <= v_upper           |
| 2     | kappa_lower <= kappa   |
| 3     | kappa <= kappa_upper   |
| 4     | a_lower <= a           |
| 5     | a <= a_upper           |
| 6     | dkappa_lower <= dkappa |
| 7     | dkappa <= dkappa_upper |

## Inital Constraint map

| Index | Name                   |
| ----- | ---------------------- |
| 0     | a_lower <= a           |
| 1     | a <= a_upper           |
| 2     | dkappa_lower <= dkappa |
| 3     | dkappa <= dkappa_upper |

## Terminal Constraint map

| Index | Name                 |
| ----- | -------------------- |
| 0     | v_lower <= v         |
| 1     | v <= v_upper         |
| 2     | kappa_lower <= kappa |
| 3     | kappa <= kappa_upper |
