# Jev 航路点决策客户端

接收 Unity 模拟的甲方任务及兵力融合态势，请求 Jev 从原有 64 Choice 选择目标航速/航向。启动及界面操作见[项目 README](../README.md)，完整字段、分片与状态语义见[当前协议](../docs/bian/当前控制与Jev协议.md)。

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

根目录保留入口与核心流程，辅助计算与传输放 tools，保留的既有离线测试放 tests，示例放 examples。本次新增的障碍物测试文件已清理。

## 报文与单位

默认 Unity `127.0.0.1:5005` ↔ Python `127.0.0.1:5006` 为 UDP 点播；态势组播 `239.255.0.10:18010`，默认网卡 `127.0.0.1`。参数 `--situation-group/--situation-port/--multicast-interface` 与 Unity Bridge 一致。采用甲方标识：任务 0x0660、态势 0x0060、停止 0x0666、控制 0x0662、状态 0x0664；JSON 算法报文支持最多十片，首片带总长度，续片不带。

任务按 `simId/id/code/params` 组织，态势按 platformId 汇集，通过 params.platCode 找本艇。Unity 的本艇及障碍物共用态势字段；红圈 type=0（不明），进入 Jev 后仍作为障碍物。碰撞半径等非甲方信息放 unity_only。其采样清单保证同一 tick 的全部实体到齐后才调用一次 Jev，不能用上一轮缓存补齐。

线侧所有速度为节，位置为经纬度。地图与内部计算保留米和 m/s；适配器将任务节速度转换为内部 m/s，不改变圈内上限含义。Jev 默认速度为节、距离为米，state.units 说明换算；可加 `--jev-speed-unit mps` 回退模型表达，通信继续为节。

航速航向以 0.01 精度发送，态势保留 Excel 的 0.1 节航速、1 度方向及经纬度量化。控制与状态通过 unity_only 关联决策编号、tick 和控制帧时间戳，Unity 等待配齐后执行。

算法状态 1 为执行中，2 为 Unity 确认的任务/停车完成，3 为决策失败。碰撞是 Unity 自己判定的仿真失败，不转换成算法状态 3。立即停止仍请求 Jev 选择停车动作，保持原有 Stopped 行为。

## 模型请求

HTTP 正文继续为 model/state/questions，64 Choice 名称、顺序、解码保持。当前运动、目标和后续点使用固定东/北视角，无参考虚线、额外艏向、角速度或执行器进度。候选预测保持近似，Python 不打分、排序、过滤或代选，Unity 不强制裁剪通过速度。

附近障碍物仍在 state.obstacle_avoidance 中，提供当前相对坐标、半径、净距，新增速度和北起顺时针运动方向。净距预测仍冻结当前位置；避障优先级和绕行策略保留，只修正未知运动信息的旧说明。

修改/复制请求功能需一并包含 choice_control.py、tools/obstacle_geometry.py、tools/model_units.py 及既有几何、运动模块。完整样本见 [choice64_request.json](examples/choice64_request.json)。

API 默认 `POST https://api.typesafe.ai/v1/systemone`、模型 jev-latest。HTTPS 线程独立复用连接，预连接不发业务请求；失败沿用手动 Retry，不自动重发。JSONL 保存任务、观测、payload、响应、动作与计时；wait_headers_s 包含网络和服务端等待，非纯推理时间。

## 验证

```powershell
python -m unittest discover -s jev_api/tests -v
```

实际模型表现和接口检查结果记录在既有实验记录中，不将历史成绩当作新协议验证结果。
