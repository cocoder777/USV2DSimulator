# 当前 Jev 控制与协议（v6）

Unity 采样并暂停 → Python 构造航线局部信息和 64 个 Choice → Jev 选择航速/航向 → Unity 瞬时设置并执行固定时长 → 再次采样。

## 请求观测

`ObservationBuilder.ForModel` 提供 `planning/action_space/boat/task`。船和航线位置用模拟经纬度 `[lon,lat]`。任务同时提供完整地图虚线采样点和打卡点；航线编号只用于标识，不代替几何数据。

Python `RouteContext` 进行经纬度到米制转换，并结合当前打卡段、历史进度匹配虚线。`build_payload` 只把当前投影点、前方虚线片段、当前打卡点及动作时长交给模型，所有点均采用 `[forward_m,left_m]` 船体坐标。当前航速、全局航向、加速度和阻尼不进入零惯性版本的模型输入。

## 单动作结果

```json
{
  "type": "decision_result",
  "version": 6,
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

`BoatActionExecutor.AcceptAction` 将偏移转换为固定世界航向，后续物理步不再累加偏移。`BoatDynamics2D` 在 Jev 模式直接保持速度和航向，不应用手动模式的加速度动力学。动作持续整数个物理步；0.4 秒对应 20 步。

Unity 继续校验身份、采样 tick、动作索引及持续时间；重复、过期结果不会改变当前动作。非法结果或 API 失败会暂停报错，通过 Retry Jev 重新采样。请求和结果各只发一次，无 ACK、自动重发和备用控制器。

## 任务结束

中间点按序进入到达圈即可；终点须在圈内同时满足绝对速度不超过停车阈值。Unity 为唯一任务判定方，完成后停止剩余动作、导出轨迹，并发送一次 `mission_completed`，包含 version、session_id、episode_id、run_id 和完成 tick。

甲方组播帧、行为树状态回复、计划航速和转弯半径暂未接入。完整运行方式及信息字段见 [jev_api/README](../../jev_api/README.md)。
