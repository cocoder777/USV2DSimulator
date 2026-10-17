# Jev API 调用说明

当前项目每个动作使用两道 Score，分别决定有符号线加速度和角加速度。Jev 只获得当前船状态、当前目标、动力学参数和动作配置。参考路线仅用于监控显示及轨迹导出对比。

## 1. 启动与接口

```powershell
python jev_api/boat_client.py --log Logs/jev.jsonl
```

Unity Play 后选择 Jev 模式，设置动作数量和持续时间，点击 `Start Jev`。默认一个动作、持续 0.4 秒；每批支持 1～10 个动作。等待回答期间仿真暂停。

```text
POST https://api.typesafe.ai/v1/systemone
Authorization: Bearer <API_KEY>
Content-Type: application/json
```

请求顶层为 `model`、`state` 和 `questions`。默认模型为 `jev-latest`，可通过 `--model` 指定。客户端使用 Python 标准库直接请求 HTTP；`route_ticket.py` 为独立 SDK 示例，不参与控制。Key 只用于请求头，不进入模型输入或日志。

官方资料：[API](https://docs.typesafe.ai/api)、[Score](https://docs.typesafe.ai/primitives/score)、[State](https://docs.typesafe.ai/concepts/state)、[问题独立求值](https://docs.typesafe.ai/primitives)。

## 2. 输入在哪里生成

1. `SimulationRunner.BeginDecision()` 调用 `ObservationBuilder.ForModel()`。
2. Unity 生成 `decision_request`，通过 `BoatUdpBridge` 发送给 Python。
3. `boat_client.py:build_payload()` 按字段白名单构造 HTTP 请求，并添加任务说明和问题。
4. `call_api()` 请求 Jev；`decode_actions()` 解析 Score；`decide()` 组装结果返回 Unity。

### HTTP state

| 字段 | 内容 |
| --- | --- |
| `coordinates` | 全局 XY 米制坐标；航向为度，东为 0、北为 90，逆时针为正 |
| `planning` | `action_count`、`action_duration_s`、`simulation_paused_while_deciding` |
| `boat` | `position_xy_m`、`heading_deg`、`speed_mps`、`yaw_rate_deg_s`、两轴实际执行器加速度 |
| `target` | 当前目标的 `position_xy_m`、`arrival_radius_m`、`requires_stop`、`stop_speed_mps` |
| `dynamics` | 速度及加速度限制、两轴执行器响应时间常数、阻尼 |
| `motion` | 有符号速度沿船头运动、执行器一阶滞后及阻尼的简短说明 |
| `task` | 到达当前目标；需要停车时，在目标半径内同时满足绝对速度阈值 |

UDP 观测另带 `actions`，包含共享档位比例。Python 用它生成 Score 等级的实际输入值，HTTP state 不重复携带比例数组。

模型输入不包含参考曲线、后续任务点、坐标变换、距离或方位辅助量、推荐速度或角速度，也不计算候选运动预测。地图中任务点列表仍由本地任务判断使用；每次只向模型发送当前目标。

## 3. 提示词与动作链

`build_payload()` 添加共同任务说明；`make_instructions()` 统一生成两轴问题，仅控制轴名称不同，没有额外的角输入提示分支。

单动作示例：

```text
What signed longitudinal acceleration command is appropriate to reach the current target for the next 0.4 seconds? The supplied boat state is the state at the start of this interval.
```

三个动作中的第二个角输入问题：

```text
What signed angular acceleration command is appropriate to reach the current target for action 2 of 3, during [0.4, 0.8) seconds after simulation resumes? The supplied boat state is measured at time zero. No updated state is supplied for later intervals. All actions are requested together and executed in order without intermediate feedback.
```

每个动作生成 `acceleration_i`、`turn_i`，问题均为 `type: score`，`criteria` 为有序字符串数组。问题 ID 仅用于关联响应，实际含义在 `instructions` 中完整表达。

所有问题独立求值：后一道题看不到前一道题的答案。动作链按序执行，但不保证模型完成了相互衔接的规划；提示词不会要求依赖未提供的前序答案。

## 4. 等级与加速度

线输入八级比例：

```text
[-1, -2/3, -1/3, 0, 0.25, 0.5, 0.75, 1]
```

角输入七级比例：

```text
[-1, -0.5, -0.25, 0, 0.25, 0.5, 1]
```

等级描述包含方向、强度和当前参数下的实际指令值。线输入正值增加有符号速度，角输入正值为逆时针角加速度；零输入不代表立即停止。

Score 返回等级位置：线输入范围 0～7，角输入范围 0～6，可为小数。它是等级位置的概率加权平均，不是直接输出的物理加速度。Python 采用 `floor(score + 0.5)` 取最近等级，恰好半级时向上取整。例如 4.2 映射线索引 4；2.5 映射角索引 3。之后按原有比例乘最大加速度，不对物理加速度插值。

`decode_actions()` 检查问题 ID 完全匹配、类型为 Score、分数为有限数且在范围内；布尔值、字符串、缺项、额外问题或越界分数均报错。概率及置信度记录在原始响应中，不用于再次选择动作。

## 5. 返回与执行

Unity/Python 外层协议仍为 version 4。请求身份字段为 `session_id`、`episode_id`、`run_id`、`decision_id`、`tick`；不发送给 Jev，返回时原样带回。

成功结果仍使用原有动作格式，例如：

```json
{
  "type": "decision_result",
  "version": 4,
  "session_id": "s",
  "episode_id": "e",
  "run_id": "r",
  "decision_id": "d",
  "tick": 0,
  "actions": [
    {"acceleration_index": 4, "turn_index": 2}
  ],
  "action_duration_s": 0.4
}
```

`SimulationRunner.ReceiveDecision()` 校验身份、动作数、索引和时长。`BoatActionSequence` 顺序执行每个动作，将索引转换为线/角加速度；`BoatDynamics2D.Step()` 推进物理。每批结束后以最新状态请求下一批，任务完成时提前结束。

UDP 重发、结果缓存及确认机制保留。周期性 `type: state` 广播仅作状态传输，不触发 API。HTTP 或解析失败时返回 `decision_error`，Unity 暂停并允许 `Retry Jev`；没有本地备用控制器，也没有决策超时。

## 6. 日志与验证

`--log` 每次记录原始 UDP `request`、实际 HTTP `payload`、`raw_response`、返回 Unity 的 `result` 和 `elapsed_s`。生成观测时已删除决策辅助信息，因此不会在原始请求日志中继续保存它们。已有历史文件不自动重写。

离线协议检查，不调用真实 API：

```powershell
python -m unittest discover -s jev_api -p "test_*.py" -v
```

历史实验结果见 [Jev仿真实验记录](Jev仿真实验记录.md)，不能作为当前简化 Score 输入方案的效果证明。
