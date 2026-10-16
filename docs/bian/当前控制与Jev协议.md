# Jev API 调用说明

本文说明 Jev 接口接受什么、当前项目每次发送什么，以及回答怎样变成小船动作。Jev 支持三种题型；本项目目前固定使用 Choice。

## 1. 怎么调用 Jev

```http
POST https://api.typesafe.ai/v1/systemone
Authorization: Bearer <API_KEY>
Content-Type: application/json
```

请求体是 UTF-8 JSON，顶层包含三个字段：

| 字段 | 内容 |
| --- | --- |
| `model` | 模型名称，项目默认 `jev-latest` |
| `state` | 所有题目共同参考的信息，可以是字符串、对象或数组；项目使用对象 |
| `questions` | 以问题 ID 为键的问题字典；每题指定 `type`、`instructions`，按题型填写 `criteria` |

问题 ID 用来匹配返回答案，不参与模型推理。因此不能只用 `turn_3` 这个名字表达“第 4 个动作”，时间区间必须写进 `instructions`。同次请求的题目分别参考共享状态求答，不能把它们当成依次读取前题答案的对话。

接口字段依据：[官方 API](https://docs.typesafe.ai/api)。问题 ID 与并行提问说明见 [Choice 文档](https://docs.typesafe.ai/primitives/choice)。

项目在 [boat_client.py](../../jev_api/boat_client.py) 的 `call_api()` 中直接使用 Python 标准库发送 HTTP，不需要 SDK。文件顶部的 `API_KEY` 只用于请求头，不放进模型输入或日志。[route_ticket.py](../../jev_api/route_ticket.py) 是另一个独立 SDK 示例，正式客户端不读取它。

启动项目客户端：

```powershell
python jev_api/boat_client.py
```

Unity Play 后选择 Jev 模式，设置动作数量和持续时间，点击 `Start Jev`。

## 2. 三种问题类型

| `type` | 问什么 | `criteria` 的形式 | 回答 |
| --- | --- | --- | --- |
| `choice` | 从给定候选项中选一项 | 字典：选项名称 → 选项描述 | `choice` 为选中的字符串，另有 `probabilities` 和 `confidence` |
| `score` | 按有序等级打分 | 数组：从低到高的等级描述 | `score` 为等级的概率加权结果，可能介于两个整数之间；另有 `legend`、`probabilities`、`confidence` |
| `noul` | 判断一个命题是否成立 | 可省略，或用 `true`、`false` 描述两种情况 | `noul` 为 0～1 的肯定概率 |

Choice 接受最多 255 个选项；Score 接受 2～10 个等级。`instructions` 和候选描述可以写成字符串、对象或数组，以便提供结构化信息。[Choice](https://docs.typesafe.ai/primitives/choice)、[Score](https://docs.typesafe.ai/primitives/score)、[Noul](https://docs.typesafe.ai/primitives/noul)

三种提问的简例（用于解释官方能力，不是本项目发送的问题）：

```json
{
  "model": "jev-latest",
  "state": {"speed_mps": 1.2, "target_speed_mps": 1.5},
  "questions": {
    "input": {
      "type": "choice",
      "instructions": "Which longitudinal input should be applied?",
      "criteria": {"brake": "Reduce speed", "coast": "No input", "accelerate": "Increase speed"}
    },
    "speed_level": {
      "type": "score",
      "instructions": "Rate the current speed relative to the target.",
      "criteria": ["Below target", "At target", "Above target"]
    },
    "below_target": {
      "type": "noul",
      "instructions": "Is the current speed below the target?"
    }
  }
}
```

当前项目只生成 `type: choice`，也只解析 Choice 答案。响应中的 `choice: "4"` 表示选项编号 4，不是分数 4；`confidence` 和候选概率也不是小船执行的加速度。若要使用 Score 或 Noul，必须另改问题构建和答案解析，不能只改一个启动参数。

## 3. 每次调用的数据在哪里

没有单独的请求配置 JSON 文件。请求在运行时组装：

1. [ObservationBuilder.cs](../../Assets/Scripts/ObservationBuilder.cs) 的 `ForModel()` 生成当前状态对象 `observation`。
2. [SimulationRunner.cs](../../Assets/Scripts/SimulationRunner.cs) 把它放进本地 UDP JSON 的 `observation` 字段，发送给 Python。
3. Python 的 `build_payload()` 生成 `{model, state, questions}`。`call_api()` 把这个对象序列化为 HTTP 请求体。
4. `decode_actions()` 读取返回答案，配成动作数组；Python 再通过 UDP 把数组交给 Unity。

想查看每次真实发送和接收的完整 JSON，启动时指定日志：

```powershell
New-Item -ItemType Directory -Force Logs | Out-Null
python jev_api/boat_client.py --log Logs/jev.jsonl
```

这是 JSONL 文件，每行是一轮调用记录：

| 日志字段 | 保存什么 |
| --- | --- |
| `request` | Unity 发来的原始 UDP 请求，包括标识和 `observation` |
| `payload` | 实际发给 Jev 的完整 HTTP JSON 请求体 |
| `raw_response` | Jev 原始回答 |
| `result` | 本地解析后发给 Unity 的动作包或错误包 |
| `event`、`elapsed_s` | 记录类别与本轮处理耗时 |

日志是可选输出，既不是请求输入文件，也不决定下一轮动作。key 不记录在内。查看单行中的 `payload`，就能知道那次调用给了模型哪些完整信息。

## 4. 本项目实际发送哪些字段

### `state`：所有问题共享的上下文

| 字段 | 完整内容 |
| --- | --- |
| `units` | XY 世界坐标以米计、时间以秒计、角度以度计；航向 0° 朝东、90° 朝北，逆时针为正 |
| `planning` | `action_count`、`action_duration_s`、`decision_period_s`、`fixed_dt_s`、`simulation_paused_while_deciding` |
| `actions` | `acceleration_fractions`、`turn_fractions`、`semantics`；给出可选输入比例和方向含义 |
| `boat` | `x`、`y`、`heading_deg`、`speed_mps`、`yaw_rate_deg_s`、`actuator_acceleration_mps2`、`actuator_angular_acceleration_deg_s2` |
| `dynamics` | `max_forward_speed`、`max_reverse_speed`、`max_acceleration`、`max_angular_acceleration_deg`、`max_yaw_rate_deg`、`acceleration_time_constant`、`angular_acceleration_time_constant`、`linear_damping`、`angular_damping`、`integration` |
| `navigation` | `lookahead_xy`、`heading_error_deg`、`distance_to_next_target_m`、`speed_goal_mps`、`reference_distance_m`、`yaw_rate_goal_deg_s`、`meaning`、`goal_rule` |
| `mission` | `status`、`next_target`、`subsequent_targets`、`arrival_radius_m`、`end_stop_speed_mps`、`cruise_speed_mps`、`rules`；任务点包含 `id`、`role`、`x`、`y` |
| `reference_path_xy` | 当前附近的参考路径坐标数组，最多 13 个点，不是整张地图 |
| `objective` | 跟踪路线、按序经过任务点、保持巡航速度、在终点范围内停车；选择的是加速度输入 |
| `control_guidance` | 比较各轴预测的跟踪误差，选择误差最小的输入；考虑执行器滞后和阻尼，不能只按输入正负号或绝对值判断 |
| `plan_instructions` | 本次规划 N 对有序动作、每对持续 Δt 仿真秒；全部回答收齐后再顺序执行 |
| `forecast_horizon_s` | 单轴恒定输入预测的时长，目前等于 `N × Δt` |

`navigation` 是本地计算的几何跟踪目标，不是 Jev 的输出。Unity 的 `observation.input_forecasts` 包含预测数据；Python 会移除这整个字段，把每个档位的预测放到对应问题的 `criteria` 中，并将预测时长单独放进 `state.forecast_horizon_s`。因此 HTTP `state` 与原始 UDP `observation` 不完全相同。

### 提示词在哪

本项目没有 Chat Completions 风格的 `messages` 或独立 `system` 字段。控制要求分布在：

- `state.objective`：任务目标。
- `state.control_guidance`：如何比较输入预测。
- `state.plan_instructions`：动作数量、时长和执行顺序。
- `questions.*.instructions`：这一题对应哪个动作、哪个时间区间、选择哪个轴的输入。
- `questions.*.criteria`：每个选项的动作含义、实际输入值与预测结果。

前三项和每题文字由 `boat_client.py:build_payload()` 生成；候选项文字来自同文件的 `ACCEL_LABELS`、`TURN_LABELS`，状态、导航与物理预测来自 Unity。下方完整请求示例保留了实际英文提示词。

### `questions`：本轮的题目和候选项

动作数量 N 与动作持续时间 Δt 来自 Unity 游戏面板，分别对应 `SimulationRunner.actionCount`、`actionDuration`，并非固定为 10 和 0.1 秒。N 的范围是 1～10，Δt 按固定仿真步取整。

每轮只调用一次 HTTP API，请求内含 `2N` 道 Choice：

```text
acceleration_0 + turn_0 → 第 1 对动作，执行 [0, Δt)
acceleration_1 + turn_1 → 第 2 对动作，执行 [Δt, 2Δt)
...
acceleration_(N-1) + turn_(N-1) → 第 N 对动作
```

每题均带 `type: choice`、`instructions`、`criteria`。`instructions` 明确动作序号、总数量、单动作时长和半开时间区间，并询问该轴哪个输入的 `physical_prediction.tracking_error` 最小。

线加速度候选项：

| 选项字符串 | 比例 | 含义 |
| --- | --- | --- |
| `"0"` | -1 | 最大负向输入，制动前进或建立倒车速度 |
| `"1"` | -2/3 | 中等负向输入 |
| `"2"` | -1/3 | 较小负向输入 |
| `"3"` | 0 | 不施加线加速度输入 |
| `"4"` | 0.25 | 小幅正向输入 |
| `"5"` | 0.5 | 中等正向输入 |
| `"6"` | 0.75 | 较大正向输入 |
| `"7"` | 1 | 最大正向输入 |

角加速度候选项：

| 选项字符串 | 比例 | 含义 |
| --- | --- | --- |
| `"0"` | -1 | 强右转／顺时针角加速度 |
| `"1"` | -0.5 | 中等右转角加速度 |
| `"2"` | -0.25 | 较小右转角加速度 |
| `"3"` | 0 | 不施加角加速度输入 |
| `"4"` | 0.25 | 较小左转／逆时针角加速度 |
| `"5"` | 0.5 | 中等左转角加速度 |
| `"6"` | 1 | 强左转角加速度 |

比例分别乘以当前的最大线加速度、最大角加速度，得到实际输入。零输入不会立刻停止平移或转动；已有速度仍受惯性、执行器滞后和阻尼影响。反向角加速度可以用于制动已有转动。

每个候选项的描述对象包含：

| 字段 | 线加速度候选项 | 角加速度候选项 |
| --- | --- | --- |
| `meaning` | 档位含义和输入 m/s² | 档位含义和输入 deg/s² |
| `physical_prediction.index` | 0～7 | 0～6 |
| 实际输入 | `input_mps2` | `input_deg_s2` |
| 预测结果 | `predicted_speed_mps` | `predicted_yaw_rate_deg_s` |
| 跟踪误差 | `tracking_error` | `tracking_error` |

目前每个时段会复用同一份“输入持续整个 N×Δt”的单轴预测。它不是执行前面已选动作后重新预测的结果，所以序列可能重复选择同一档位。这是当前提示逻辑的行为，本文不改变它。

## 5. 一次实际调用的完整请求和返回

下面取自本次实际运行的一轮调用：N=1、Δt≈0.4 秒、Level 0 初始状态。数值是当时的快照，不是固定配置；Unity 浮点数可能显示为 0.39999998 等近似值。示例没有省略状态字段、问题或候选项，不含请求头中的 key。

<details>
<summary>完整 HTTP 请求体：payload</summary>

```json
{
  "model": "jev-latest",
  "state": {
    "units": "world XY metres; seconds; degrees; heading 0=E,90=N,counterclockwise positive",
    "planning": {
      "action_count": 1,
      "action_duration_s": 0.4,
      "decision_period_s": 0.4,
      "fixed_dt_s": 0.0199999921,
      "simulation_paused_while_deciding": true
    },
    "actions": {
      "acceleration_fractions": [
        -1.0,
        -0.6666667,
        -0.333333343,
        0.0,
        0.25,
        0.5,
        0.75,
        1.0
      ],
      "turn_fractions": [
        -1.0,
        -0.5,
        -0.25,
        0.0,
        0.25,
        0.5,
        1.0
      ],
      "semantics": "Signed longitudinal acceleration and angular acceleration COMMANDS. Positive turn=left/CCW; negative=right/CW. Zero does not stop motion. No heading servo, no automatic yaw braking."
    },
    "boat": {
      "x": 0.0,
      "y": 0.0,
      "heading_deg": 0.0,
      "speed_mps": 0.0,
      "yaw_rate_deg_s": 0.0,
      "actuator_acceleration_mps2": 0.0,
      "actuator_angular_acceleration_deg_s2": 0.0
    },
    "dynamics": {
      "max_forward_speed": 5.0,
      "max_reverse_speed": 1.0,
      "max_acceleration": 1.5,
      "max_angular_acceleration_deg": 60.0,
      "max_yaw_rate_deg": 45.0,
      "acceleration_time_constant": 0.25,
      "angular_acceleration_time_constant": 0.25,
      "linear_damping": 0.2,
      "angular_damping": 1.2,
      "integration": "Every fixed_dt: actuator += (command-actuator)*(1-exp(-dt/time_constant)); v += (actuator_acceleration-linear_damping*v)*dt; yaw_rate += (actuator_angular_acceleration-angular_damping*yaw_rate)*dt; clamp speeds; heading += yaw_rate*dt; position += signed_speed * heading_unit_vector * dt. Zero time_constant means no lag."
    },
    "navigation": {
      "lookahead_xy": [
        1.0,
        0.0
      ],
      "heading_error_deg": 0.0,
      "distance_to_next_target_m": 10.0,
      "speed_goal_mps": 1.5,
      "reference_distance_m": 0.0,
      "meaning": "Geometry/task goals only, not actuator commands. Positive heading error means target to left. Lookahead cannot skip an unpassed checkpoint. End speed goal decreases towards an inner stopping region.",
      "yaw_rate_goal_deg_s": 0.0,
      "goal_rule": "Yaw-rate goal = clamp(1.5 * lookahead heading error, +/- max yaw rate). This is a geometric tracking setpoint, not an angular acceleration command."
    },
    "mission": {
      "status": "Running",
      "next_target": {
        "id": "P1",
        "role": "checkpoint",
        "x": 10.0,
        "y": 0.0
      },
      "subsequent_targets": [
        {
          "id": "P2",
          "role": "checkpoint",
          "x": 20.0,
          "y": 5.0
        },
        {
          "id": "P3",
          "role": "checkpoint",
          "x": 30.0,
          "y": 5.0
        },
        {
          "id": "E",
          "role": "end",
          "x": 40.0,
          "y": 0.0
        }
      ],
      "arrival_radius_m": 1.0,
      "end_stop_speed_mps": 0.1,
      "cruise_speed_mps": 1.5,
      "rules": "Visit next_target then subsequent_targets in order. Intermediate points do not require stopping. End requires distance AND abs(speed) within limits simultaneously."
    },
    "reference_path_xy": [
      [
        0.0,
        0.0
      ],
      [
        1.0,
        0.0
      ],
      [
        2.0,
        0.0
      ],
      [
        3.0,
        0.0
      ],
      [
        4.0,
        0.0
      ],
      [
        5.0,
        0.0
      ],
      [
        6.0,
        0.0
      ],
      [
        7.0,
        0.0
      ],
      [
        8.0,
        0.0
      ],
      [
        9.0,
        0.0
      ],
      [
        10.0,
        0.0
      ],
      [
        11.0,
        0.0428
      ],
      [
        12.0,
        0.2896
      ]
    ],
    "objective": "Follow the reference path forward, visit mission targets in order, maintain cruise speed, and stop inside the end radius. Choose acceleration INPUTS, not velocity, position or heading. Turning right can be needed to stop an existing leftward yaw rate.",
    "control_guidance": "Navigation supplies speed and yaw-rate TRACKING GOALS. Each axis has independent physical forecasts. Select the input with the SMALLEST tracking_error for that axis. tracking_error already includes current actuator inertia and damping. Smaller is better. Do not choose by the input sign alone: an opposite angular input can reduce excessive existing yaw rate. Do not choose an input merely because its absolute magnitude is small. Compare the predicted goal errors.",
    "plan_instructions": "Plan 1 ordered paired actions in this single request. Each action lasts 0.4 simulation seconds. Each interval has one longitudinal acceleration question and one angular acceleration question. All answers are collected before simulation resumes; intervals execute consecutively in index order.",
    "forecast_horizon_s": 0.4
  },
  "questions": {
    "acceleration_0": {
      "type": "choice",
      "instructions": "Action 1 of 1; duration 0.4 s. Plan input for interval [0.000, 0.400) simulation seconds after resuming from this frozen state. Account for prior intervals, inertia, damping and the mission goals. Which input has the SMALLEST physical_prediction.tracking_error on this axis? Select the closest predicted tracking result; all inputs are allowed.",
      "criteria": {
        "0": {
          "meaning": "Strong negative acceleration; rapidly brake forward motion, or build reverse speed. Input is -1.5 m/s^2.",
          "physical_prediction": {
            "index": 0,
            "input_mps2": -1.5,
            "predicted_speed_mps": -0.3038,
            "tracking_error": 1.8038
          }
        },
        "1": {
          "meaning": "Medium negative acceleration; clearly reduce forward speed. Input is -1 m/s^2.",
          "physical_prediction": {
            "index": 1,
            "input_mps2": -1.0,
            "predicted_speed_mps": -0.2025,
            "tracking_error": 1.7025
          }
        },
        "2": {
          "meaning": "Mild negative acceleration; gently reduce forward speed. Input is -0.5 m/s^2.",
          "physical_prediction": {
            "index": 2,
            "input_mps2": -0.5,
            "predicted_speed_mps": -0.1013,
            "tracking_error": 1.6013
          }
        },
        "3": {
          "meaning": "No longitudinal acceleration input; coast with actuator lag and drag. Input is 0 m/s^2.",
          "physical_prediction": {
            "index": 3,
            "input_mps2": 0.0,
            "predicted_speed_mps": 0.0,
            "tracking_error": 1.5
          }
        },
        "4": {
          "meaning": "Small positive acceleration; gently increase forward speed or compensate drag. Input is 0.375 m/s^2.",
          "physical_prediction": {
            "index": 4,
            "input_mps2": 0.375,
            "predicted_speed_mps": 0.0759,
            "tracking_error": 1.4241
          }
        },
        "5": {
          "meaning": "Medium positive acceleration; build forward speed. Input is 0.75 m/s^2.",
          "physical_prediction": {
            "index": 5,
            "input_mps2": 0.75,
            "predicted_speed_mps": 0.1519,
            "tracking_error": 1.3481
          }
        },
        "6": {
          "meaning": "Large positive acceleration; increase forward speed quickly. Input is 1.125 m/s^2.",
          "physical_prediction": {
            "index": 6,
            "input_mps2": 1.125,
            "predicted_speed_mps": 0.2278,
            "tracking_error": 1.2722
          }
        },
        "7": {
          "meaning": "Maximum positive acceleration; strongest forward speed increase. Input is 1.5 m/s^2.",
          "physical_prediction": {
            "index": 7,
            "input_mps2": 1.5,
            "predicted_speed_mps": 0.3038,
            "tracking_error": 1.1962
          }
        }
      }
    },
    "turn_0": {
      "type": "choice",
      "instructions": "Action 1 of 1; duration 0.4 s. Plan input for interval [0.000, 0.400) simulation seconds after resuming from this frozen state. Account for prior intervals, inertia, damping and the mission goals. Which input has the SMALLEST physical_prediction.tracking_error on this axis? Select the closest predicted tracking result; all inputs are allowed.",
      "criteria": {
        "0": {
          "meaning": "Strong clockwise/right angular acceleration; build negative yaw rate or brake positive yaw rate strongly. Input is -60 deg/s^2.",
          "physical_prediction": {
            "index": 0,
            "input_deg_s2": -60.0,
            "predicted_yaw_rate_deg_s": -10.5846,
            "tracking_error": 10.5846
          }
        },
        "1": {
          "meaning": "Medium clockwise/right angular acceleration; increase rightward rotation or reduce leftward rotation. Input is -30 deg/s^2.",
          "physical_prediction": {
            "index": 1,
            "input_deg_s2": -30.0,
            "predicted_yaw_rate_deg_s": -5.2923,
            "tracking_error": 5.2923
          }
        },
        "2": {
          "meaning": "Mild clockwise/right angular acceleration; small adjustment toward rightward rotation. Input is -15 deg/s^2.",
          "physical_prediction": {
            "index": 2,
            "input_deg_s2": -15.0,
            "predicted_yaw_rate_deg_s": -2.6462,
            "tracking_error": 2.6462
          }
        },
        "3": {
          "meaning": "No angular acceleration input; yaw rate continues under actuator lag and damping. Input is 0 deg/s^2.",
          "physical_prediction": {
            "index": 3,
            "input_deg_s2": 0.0,
            "predicted_yaw_rate_deg_s": 0.0,
            "tracking_error": 0.0
          }
        },
        "4": {
          "meaning": "Mild counterclockwise/left angular acceleration; small adjustment toward leftward rotation. Input is 15 deg/s^2.",
          "physical_prediction": {
            "index": 4,
            "input_deg_s2": 15.0,
            "predicted_yaw_rate_deg_s": 2.6462,
            "tracking_error": 2.6462
          }
        },
        "5": {
          "meaning": "Medium counterclockwise/left angular acceleration; increase leftward rotation or reduce rightward rotation. Input is 30 deg/s^2.",
          "physical_prediction": {
            "index": 5,
            "input_deg_s2": 30.0,
            "predicted_yaw_rate_deg_s": 5.2923,
            "tracking_error": 5.2923
          }
        },
        "6": {
          "meaning": "Strong counterclockwise/left angular acceleration; build positive yaw rate or brake negative yaw rate strongly. Input is 60 deg/s^2.",
          "physical_prediction": {
            "index": 6,
            "input_deg_s2": 60.0,
            "predicted_yaw_rate_deg_s": 10.5846,
            "tracking_error": 10.5846
          }
        }
      }
    }
  }
}
```

</details>

<details>
<summary>对应的完整 Jev 回答：raw_response</summary>

```json
{
  "model": "jev-1.13.0",
  "answers": {
    "acceleration_0": {
      "type": "choice",
      "choice": "7",
      "confidence": 0.75,
      "probabilities": {
        "0": 0.01,
        "1": 0.01,
        "2": 0.01,
        "3": 0.04,
        "4": 0.07,
        "5": 0.04,
        "6": 0.04,
        "7": 0.78
      }
    },
    "turn_0": {
      "type": "choice",
      "choice": "3",
      "confidence": 0.98,
      "probabilities": {
        "0": 0.0,
        "1": 0.0,
        "2": 0.01,
        "3": 0.98,
        "4": 0.01,
        "5": 0.0,
        "6": 0.0
      }
    }
  },
  "usage": {
    "input_tokens": 3232,
    "output_tokens": 142
  }
}
```

</details>

响应顶层的 `model` 是实际处理请求的模型版本，可能与请求里的别名不同；`answers` 按问题 ID 返回各题结果，`usage` 记录输入、输出 token 用量。[官方响应结构](https://docs.typesafe.ai/api)

## 6. 回答怎样被本地接受并执行

Python 的 `decode_actions()` 要求恰好收到 `2N` 个答案，并逐个读取 `acceleration_i`、`turn_i`：

1. 每题必须是 `type: choice`。
2. 线输入的 `choice` 必须是字符串 `"0"`～`"7"`，角输入必须是 `"0"`～`"6"`。
3. 把字符串转为整数，按 i 配对成 `actions` 数组。概率、置信度和 token 用量不参与动作计算。
4. 整批校验通过后发给 Unity，不先执行其中一部分。

下面是对应样例的本地结果。标识已替换为说明性占位符，动作和时长取自真实回答：

```json
{
  "session_id": "<session_id: echo from Unity request>",
  "episode_id": "<episode_id: echo from Unity request>",
  "run_id": "<run_id: echo from Unity request>",
  "decision_id": "<decision_id: echo from Unity request>",
  "tick": 0,
  "version": 4,
  "type": "decision_result",
  "actions": [
    {
      "acceleration_index": 7,
      "turn_index": 3
    }
  ],
  "action_duration_s": 0.4
}
```

Unity 与 Python 的本地通信使用 UDP JSON，默认 Unity 接收 `127.0.0.1:5005`，Python 接收 `127.0.0.1:5006`，每包最多 8192 字节。HTTP 请求体不受这个本地包长限制；候选项文字在 Python 中组装，没有通过 UDP 逐项传输。

Unity 请求包的顶层为 `type: decision_request`、`version: 4`、`session_id`、`episode_id`、`run_id`、`decision_id`、`tick`、`observation`。这些传输标识不送给 Jev，只在 Python 返回结果时原样回传，用于避免旧回合或旧决策控制当前小船。

[BoatUdpBridge.cs](../../Assets/Scripts/BoatUdpBridge.cs) 接收数据，[SimulationRunner.cs](../../Assets/Scripts/SimulationRunner.cs) 校验标识、动作数量、索引和持续时间，再交给 [BoatActionSequence.cs](../../Assets/Scripts/BoatActionSequence.cs) 播放：

```text
线输入 = acceleration_fractions[acceleration_index] × maxAcceleration
角输入 = turn_fractions[turn_index] × maxAngularAccelerationDeg
```

例如 `acceleration_index=4` 是 0.25 倍最大线加速度，`turn_index=3` 是零角加速度输入。它们改变动力学输入，由 [BoatDynamics2D.cs](../../Assets/Scripts/BoatDynamics2D.cs) 积分出速度、航向和位置；API 不会直接修改坐标，也不直接返回目标航向。

每对动作保持 Δt 仿真秒，连续执行 N 对后，再取得新状态并发起下一次调用。等待 API 时冻结艇、执行器和任务时间，界面与通信继续。这里的周期 N×Δt 是仿真时间，现实等待时间还包含 API 耗时。

Unity 接受结果后发送 `decision_ack`；未确认的同一结果会重发，同一请求的重复 UDP 包复用 Python 缓存，不重复调用模型。重置、切换模式、任务或参数变化会作废旧结果。

HTTP 调用或答案解析失败时，Python 返回 `decision_error`。Unity 进入 Failed 并暂停，用户可点击 `Retry Jev`；没有本地备用自动驾驶，也没有模型决策超时。
