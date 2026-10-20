# 当前 Jev 控制与协议（v9）

Unity 采样并暂停 → Python 构造航线局部信息和 64 个 Choice → Jev 选择航速/航向 → Unity 平滑改变航向/角速度和航速，并执行固定时长 → 再次采样。

## 请求观测

`ObservationBuilder.ForModel` 提供 `planning/action_space/boat/task`。船和航线位置用模拟经纬度 `[lon,lat]`。任务同时提供完整地图虚线采样点和打卡点；航线编号只用于标识，不代替几何数据。

Python `RouteContext` 进行经纬度到米制转换，并结合当前打卡段、历史进度匹配虚线。内部保留船体数值坐标，发给模型时使用“前方/后方、左/右”的英文文字和明确数值。

交叉与折返时，当前段前视截止当前目标，后续航线另列 `after_checkpoint_preview`；`near_route_direction` 与沿线剩余距离帮助理解近处引导。候选预测未入当前圈时只匹配当前段及衔接尾段，入圈后才开放后续局部航线；投影的沿线范围由预测移动距离和初始偏差约束。模型被明确要求偏离时先回线，不能因结束时与航线平行而持续沿偏移位置前进。转弯角度提示版复跑失败，当前恢复为两次完成 Level4 的当前航段引导版。

HTTP 顶层仍为 `model/state/questions`。`state` 包含 `task/boat/route/target/execution`：任务背景、当前阶段、船况、附近虚线、当前目标和执行规律。瞬时响应时省略对应的惯性状态；全局位置、航线编号和手动加速度参数不进入模型输入。

`questions.action.instructions` 要求不漏点、正确停车、贴线和尽快完成。64 个 `criteria` 各自提供实际动作和几何结果：当前动作全过程偏差、沿线进度、通过当前圈的情况、结束航速/角速度、结束船头与局部虚线的夹角，以及若继续保持相同指令两段时长的假设预览。假设预览不改变实际动作时长，不代表预先安排下一次动作。

贴线软偏好由 Python `--tracking-preference-m` 设置，独立于 Unity 到达半径。Python 不做动作排名或代选，实际执行 Jev 返回的唯一 Choice。底层 Unity/Python 协议与执行语义保持 v9。

## 单动作结果

```json
{
  "type": "decision_result",
  "version": 9,
  "session_id": "s",
  "episode_id": "e",
  "run_id": "r",
  "decision_id": "d",
  "tick": 0,
  "action": {"speed_index": 5, "heading_index": 1},
  "action_duration_s": 0.4
}
```

速度比例 `[-1,-0.5,0,0.1,0.25,0.5,0.75,1]`，按符号使用最大后退或前进速度；航向偏移 `[0,45,90,135,180,-135,-90,-45]` 度，左正右负。示例动作表示半速前进、相对采样船头左偏 45 度。

`BoatActionExecutor.AcceptAction` 将偏移转换为固定世界航向，后续物理步不再累加偏移。`BoatDynamics2D` 在 Jev 模式收到新的目标速度后，以恒定加速度 `(target-current)/speedResponseTime` 立刻改变航速，在设定时间达到目标并保持，默认 0.5 s；0 时瞬时切换。相同目标继续剩余加速过程，不重置计时。航向以三次曲线从当前航向/角速度平滑到目标并归零角速度，独立转向时长默认0.5 s，0为瞬时转向。Unity 与 Python 均沿弯曲路径积分速度向量，并在速度或转向过程结束处拆分积分区间。不应用手动模式的加速度动力学或阻尼。动作持续整数个物理步；0.4 秒对应 20 步。

Unity 继续校验身份、采样 tick、动作索引及持续时间；重复、过期结果不会改变当前动作。非法结果或 API 失败会暂停报错，通过 Retry Jev 重新采样。请求和结果各只发一次，无 ACK、自动重发和备用控制器。

## 任务结束

中间点按序进入到达圈即可；终点须在圈内同时满足绝对速度不超过停车阈值。Unity 为唯一任务判定方，完成后停止剩余动作、导出轨迹，并发送一次 `mission_completed`，包含 version、session_id、episode_id、run_id 和完成 tick。

甲方组播帧、行为树状态回复、计划航速和转弯半径暂未接入。完整运行方式及信息字段见 [jev_api/README](../../jev_api/README.md)。

## 实际请求示例与参数来源

[完整 API 请求示例](../../jev_api/examples/choice64_request.json) 包含当前选定方案的全部 state、完整 instructions 与64个候选项。

所有速度、动作时长、响应时间、到达半径和停车阈值来自本次观测；贴线软偏好是独立且可配置的决策参数。前方虚线采样仍由 `route_context.py` 定义。当前地图参数之外的效果需要另行验证，不能把 Level 0 结果当作所有场景的性能保证。
