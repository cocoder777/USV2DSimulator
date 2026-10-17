# Jev 单动作调用协议

每次决策只选择一个动作：Unity 暂停并提供最新状态，Python 请求 Jev，Unity 执行返回的线/角加速度组合，执行结束后再次暂停并请求下一次决策。默认每个动作持续 0.4 秒。

## 1. 启动与接口

```powershell
python jev_api/boat_client.py --log Logs/jev.jsonl
```

Unity Play 后选择 Jev 模式，在 `JEV CONTROL` 设置动作持续时间，点击 `Start Jev`。运行期间时长锁定，重置后可重新设置。等待 Jev 回答期间仿真暂停，界面继续响应。

```text
POST https://api.typesafe.ai/v1/systemone
Authorization: Bearer <API_KEY>
Content-Type: application/json
```

HTTP 请求顶层为 `model`、`state`、`questions`。默认模型 `jev-latest`，可用 `--model` 指定。客户端仅用 Python 标准库发起 HTTP 请求；Key 只进入请求头，不进入输入或日志。

官方资料：[API](https://docs.typesafe.ai/api)、[Choice](https://docs.typesafe.ai/primitives/choice)、[State](https://docs.typesafe.ai/concepts/state)。

## 2. 可用信息与坐标

1. `SimulationRunner.BeginDecision()` 调用 `ObservationBuilder.ForModel()`，采集船状态及当前目标。
2. `BoatUdpBridge` 将 `decision_request` 发给 Python。
3. `choice_control.py:build_payload()` 转换目标坐标，构建唯一一道 `action_0` Choice。
4. `call_api()` 请求 Jev；`decode_action()` 将选项名称映射为一个线/角加速度索引对。

| HTTP state 字段 | 内容 |
| --- | --- |
| `coordinates` | 决策时刻的船体坐标定义、方向及单位 |
| `planning` | `action_duration_s`、`simulation_paused_while_deciding` |
| `boat` | 有符号线速度 `speed_mps`、角速度 `yaw_rate_deg_s` |
| `target` | `forward_m`、`left_m`、到达半径；仅终点携带停车速度阈值 |
| `dynamics` | 速度和加速度限制、阻尼 |
| `motion` | 加速度、速度、角速度、航向和位置之间的关系 |

本地原始观测用全局坐标，并以 `action_space` 提供八档线输入及七档角输入比例。Python 根据目标与船的位置差 dx、dy，以及航向 h，转换为：

```text
forward_m = dx*cos(h) + dy*sin(h)
left_m    = -dx*sin(h) + dy*cos(h)
```

HTTP state 不包含全局位置与航向。船体坐标中船位于原点并朝向前方，forward 正数表示前方，left 正数表示左侧，角运动逆时针为正。`coordinates` 是自定义的说明文本，不是 API 内置坐标配置。目标固定在世界中，相对坐标随船移动和旋转而变化。

模型不接收参考路线、未来目标、指导速度、指导角速度、距离/方位误差辅助量、预测轨迹、实际执行器加速度或响应时间常数。全局状态与物理执行数据仍可供监控和轨迹对比使用。

## 3. 题目与选项

本地读取原始观测中的 `requires_stop`，选择 `CHECKPOINT_TASK` 或 `PARKING_TASK`，从 HTTP state 删除该标志。Jev 只看到当前适用的目标说明。

- 打卡点：进入到达圈即可，不要求停车；必要时可减速以避免错过目标圈。
- 终点：在到达圈内同时满足低速条件。停车阈值只约束完成时刻，远处应保持有效接近；已有动量可能导致越过停车区时提前制动，必要时边减速边转向。

`TASK` 保留现有运动规则、方向说明和状态示例，`make_instructions(duration, requires_stop)` 组合当前目标说明，并询问接下来指定时长内的动作。

每次只有一道 `action_0`，包含 56 个联合选项。线输入八档比例为：

```text
[-1, -2/3, -1/3, 0, 0.25, 0.5, 0.75, 1]
```

角输入七档比例为：

```text
[-1, -0.5, -0.25, 0, 0.25, 0.5, 1]
```

`criteria` 为“选项名称 → 适用情境与两个实际加速度数值”的对象。现有打卡/停车规则、运动示例及 56 个选项的导航含义保持不变。

完整示例：[打卡请求](../../jev_api/examples/choice56_request.json)、[终点停车请求](../../jev_api/examples/choice56_parking_request.json)。

供本地解码的 API 响应关键内容：

```json
{"answers":{"action_0":{"type":"choice","choice":"positive_gentle__clockwise_gentle"}}}
```

`decode_action()` 严格检查唯一的问题 ID、类型和选项名称，直接返回 `acceleration_index=4, turn_index=2`。概率和置信度保存在原始响应中，不在本地重新选择动作；非法响应返回错误并暂停。

## 4. Unity/Python 单动作协议

内部 UDP 使用 **version 5**，双方需要一起更新。旧版本数据不被接受。

```json
{
  "type":"decision_result",
  "version":5,
  "session_id":"s",
  "episode_id":"e",
  "run_id":"r",
  "decision_id":"d",
  "tick":0,
  "action":{"acceleration_index":4,"turn_index":2},
  "action_duration_s":0.4
}
```

`SimulationRunner.ReceiveDecision()` 校验身份、采样 tick、动作索引和持续时间。`BoatActionExecutor` 只持有一个动作及剩余执行步数，通过 `DiscreteAction` 的档位定义设置控制量，由 `BoatDynamics2D.Step()` 推进物理。

动作必须执行整数个物理步，默认 0.4 秒对应 20 步。动作结束后取消控制并用最新状态请求下一次决策；任务完成时提前停止剩余执行。等待回答时不推进物理。重复响应只确认、不重启动作，过期响应不影响当前决策；非法动作或时长不匹配会暂停并报告错误。

UDP 重发、缓存和确认机制保持原逻辑。API 失败时使用 `Retry Jev`，没有本地备用控制器或决策超时。

## 5. 日志与验证

`--log` 记录原始 UDP `request`、实际 HTTP `payload`、`raw_response`、单动作 `result` 及 API 耗时。状态广播保留 `action_duration_s`；每条决策只对应一个动作。

离线协议测试：

```powershell
python -m unittest discover -s jev_api -p "test_*.py" -v
```

全部历史改动、实验及测试统一记录在 [Jev 仿真实验记录](Jev仿真实验记录.md)，后续也只更新该文件，不另建实验或验证记录。历史协议字段仅反映当时配置。
