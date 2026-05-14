# SpeedRiskModel user manual

user manual for generated model SpeedRiskModel, code optimization has been applied for the best performance

## State variable map

| Index | Name |
| ----- | ---- |
| 0     | s    |
| 1     | v    |
| 2     | a    |

## Ctrl variable map

| Index | Name |
| ----- | ---- |
| 0     | j    |

## Slack variable map

| Index | Name          |
| ----- | ------------- |
| 0     | slack_s_upper |
| 1     | slack_s_lower |
| 2     | slack_v_upper |
| 3     | slack_v_lower |
| 4     | slack_a_upper |
| 5     | slack_a_lower |
| 6     | slack_d_v     |

## Paramter map

| Index | Name                       |
| ----- | -------------------------- |
| 0     | SRef                       |
| 1     | VRef                       |
| 2     | K                          |
| 3     | SWeight                    |
| 4     | VWeight                    |
| 5     | AWeight                    |
| 6     | JWeight                    |
| 7     | SlackSUpperWeight          |
| 8     | SlackSLowerWeight          |
| 9     | SlackVUpperWeight          |
| 10    | SlackVLowerWeight          |
| 11    | SlackAUpperWeight          |
| 12    | SlackALowerWeight          |
| 13    | SlackDVWeight              |
| 14    | SHardUpperBound            |
| 15    | SHardLowerBound            |
| 16    | SSoftUpperBound            |
| 17    | SSoftLowerBound            |
| 18    | SUpperBoundForDVConstraint |
| 19    | VHardUpperBound            |
| 20    | VHardLowerBound            |
| 21    | VSoftUpperBound            |
| 22    | VSoftLowerBound            |
| 23    | AHardUpperBound            |
| 24    | AHardLowerBound            |
| 25    | ASoftUpperBound            |
| 26    | ASoftLowerBound            |
| 27    | JHardUpperBound            |
| 28    | JHardLowerBound            |
| 29    | SafeDistForDVConstraint    |
| 30    | k                          |
| 31    | residual                   |
| 32    | df                         |

## Constraint map

| Index | Name                                                                        |
| ----- | --------------------------------------------------------------------------- |
| 0     | SHardLowerBound <= s                                                        |
| 1     | s <= SHardUpperBound                                                        |
| 2     | VHardLowerBound <= v                                                        |
| 3     | v <= VHardUpperBound                                                        |
| 4     | AHardLowerBound <= a                                                        |
| 5     | a <= AHardUpperBound                                                        |
| 6     | JHardLowerBound <= j                                                        |
| 7     | j <= JHardUpperBound                                                        |
| 8     | s - slack_s_upper <= SSoftUpperBound                                        |
| 9     | SSoftLowerBound <= s + slack_s_lower                                        |
| 10    | -VSoftUpperBound - df*s + residual - slack_v_upper + v <= 0                 |
| 11    | VSoftLowerBound <= slack_v_lower + v                                        |
| 12    | a - slack_a_upper <= ASoftUpperBound                                        |
| 13    | ASoftLowerBound <= a + slack_a_lower                                        |
| 14    | SafeDistForDVConstraint + k*v <= SUpperBoundForDVConstraint - s + slack_d_v |

## Inital Constraint map

| Index | Name                 |
| ----- | -------------------- |
| 0     | JHardLowerBound <= j |
| 1     | j <= JHardUpperBound |

## Terminal Constraint map

| Index | Name                 |
| ----- | -------------------- |
| 0     | SHardLowerBound <= s |
| 1     | s <= SHardUpperBound |
| 2     | VHardLowerBound <= v |
| 3     | v <= VHardUpperBound |
| 4     | AHardLowerBound <= a |
| 5     | a <= AHardUpperBound |
