# Jev 航路点决策客户端

接收 Unity 模拟的甲方任务及兵力融合态势，请求 Jev 从 64 个离散候选中选择目标航速/航向。Unity 采样后暂停，收到匹配动作后执行指定时长，再次采样。启动及界面操作见[项目 README](../README.md)，完整字段、分片与状态语义见[当前协议](../docs/bian/当前控制与Jev协议.md)。

| 文件 | 职责 |
| --- | --- |
| `boat_client.py` | 点播/组播接收、任务适配、状态及控制发送、日志 |
| `tools/client_protocol.py` | 甲方二进制帧、特殊值、分片重组、Unity 快照适配 |
| `decision_worker.py` | 决策去重、后台调度、HTTPS 线程上下文 |
| `choice_control.py` | 导航/立即停止模型请求、64 Choice、动作解码 |
| `tools/waypoint_geometry.py` | 经纬度转固定东/北坐标、入圈与通过速度计算 |
| `tools/navigation_motion.py` | 简化运动预测，不选择动作 |
| `tools/obstacle_geometry.py` | 附近障碍物、当前位置快照净距及运动事实 |
| `tools/model_units.py` | Jev 速度表达单位转换，不改变计算与判断 |
| `tools/http_transport.py` | HTTPS 复用、代理兼容、预连接、计时 |

入口与核心流程位于本目录，辅助计算与传输位于 `tools/`，离线测试位于 `tests/`，示例位于 `examples/`。`jev_api_guoliang/` 是独立程序，本客户端不导入或依赖该目录。

## 报文与单位

默认 Unity `127.0.0.1:5005` ↔ Python `127.0.0.1:5006` 为 UDP 点播；态势组播 `239.255.0.10:18010`，默认网卡 `127.0.0.1`。参数 `--situation-group/--situation-port/--multicast-interface` 与 Unity Bridge 一致。采用甲方标识：任务 0x0660、态势 0x0060、停止 0x0666、控制 0x0662、状态 0x0664；JSON 算法报文支持最多十片，首片带总长度，续片不带。

任务按 `simId/id/code/params` 组织，列表只含目标，`currentRoute` 是已完成目标数量，0 对应第一目标。态势按 `platformId` 汇集，通过 `params.platCode` 找本艇。本艇及障碍物共用态势字段；障碍物 `type=0`（不明），进入 Jev 后明确描述为障碍物并提供当前速度和运动方向。`unity_only` 的半径表 key 兼作全部实体清单，按共同快照时戳汇齐才调用一次 Jev，不能用上一轮缓存补齐。消费后删除快照，缺失 3 秒失败；重复 `decision_id` 不重调，新任务撤销旧输出。

`unity_only` 是本项目扩展对象，包含 `decision_id/tick`、`event`、快照时戳、半径表、动作时长、`max_speed_knots`、响应配置和到达/停车阈值。运行身份和进度使用正式 `simId/id/currentRoute`，候选表由接收端按固定定义生成。STOP 按正式身份读取已接收任务上下文；未接收对应任务时拒绝。完整字段及必要性见当前协议。

线侧所有速度为节，位置为经纬度。地图与内部计算保留米和 m/s；适配器将任务节速度转换为内部 m/s，不改变圈内上限含义。Jev 默认速度为节、距离为米，state.units 说明换算；可加 `--jev-speed-unit mps` 回退模型表达，通信继续为节。

航速航向以 0.01 精度发送，态势采用 Excel 定义的 0.1 节/1 度及经纬度量化。控制与状态通过 `unity_only` 的 `decision_id/tick/control_timestamp` 配齐，正式 `simId/id` 隔离运行。Unity 执行量化控制，时长取匹配的本地请求，旧动作和旧状态均拒绝。

算法状态 1 为执行中，2 为 Unity 确认的任务/停车完成，3 为决策失败。碰撞是 Unity 自己判定的仿真失败，不转换成算法状态 3。立即停止仍请求 Jev 选择停车动作，保持原有 Stopped 行为。

## 模型请求

HTTP 正文为 `model/state/questions`，提供 8 档非负速度 × 8 个绝对航向，共 64 个候选。当前运动、目标和后续点使用固定东/北视角，无参考虚线、额外艏向、角速度或执行器进度。候选预测为近似计算，Python 不打分、排序、过滤或代选，Unity 不强制裁剪通过速度。

`state.obstacle_avoidance` 提供附近障碍物的当前相对坐标、半径、净距、速度及北起顺时针运动方向。候选净距预测冻结障碍物当前位置，不预测其未来轨迹。

完整模型请求样本见 [choice64_request.json](examples/choice64_request.json)。

API 默认 `POST https://api.typesafe.ai/v1/systemone`、模型 `jev-latest`。HTTPS 线程独立复用连接，预连接不发业务请求；失败后使用手动 Retry，不自动重发。JSONL 保存任务、观测、payload、响应、动作与计时；`wait_headers_s` 包含网络和服务端等待，非纯推理时间。

## 验证

```powershell
python -m unittest discover -s jev_api/tests -v
```

闭环实验过程与结果见[本地实验记录](../docs/bian_local/Jev仿真实验记录.md)，完整请求、响应及轨迹保存在 `Logs/`。
