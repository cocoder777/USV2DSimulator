# Jev 航路点决策客户端

接收 Unity 模拟的甲方船况和航路点任务，整理全局运动观测，请求 Jev 选择目标航速/航向。启动和界面操作见[项目 README](../README.md)。

| 文件 | 职责 |
| --- | --- |
| `boat_client.py` | UDP 收发、观测验证、转发及日志 |
| `decision_worker.py` | 去重、后台调度、HTTPS 线程上下文 |
| `tools/waypoint_geometry.py` | 航路点经纬度转固定东/北米制坐标、入圈和通过速度几何计算 |
| `choice_control.py` | 构造 64 个全局动作候选，解码 Jev 唯一选择 |
| `tools/navigation_motion.py` | 仅根据当前运动观测作近似候选预测，不选择动作 |
| `tools/http_transport.py` | HTTPS 复用、代理兼容、预连接、计时 |
| `tests/test_navigation.py` | 离线接口、坐标、速度上限及运动测试 |

目录约定：`jev_api/` 根目录保留客户端入口和核心决策流程，不随意增加文件。几何计算、运动预测和传输等辅助模块放在 `tools/`，离线测试放在 `tests/`，示例放在 `examples/`。`tools/waypoint_geometry.py` 只提供几何计算，不负责选择动作。

## 请求和动作

Unity `boat` 使用 `longitude/latitude/targetSpeed/targetCourse`，分别为度、度、节、北起顺时针角度，表示实际状态。`targetCourse` 是速度向量方向，静止为 `null`；没有额外艏向、转速、执行器进度或参考虚线输入。Python 内部转 m/s，位置按固定东/北表达，不使用船头视角。

`planWaypointList` 包含 `routeLon/routeLat/routeSpeed`；本仿真明确 `routeSpeed` 单位为 m/s，只是通过圈内的速度上限，不要求达到。当前和后续点可供 Jev 提前规划，必须按序通过。地图虚线仅在 Unity 显示，不约束路径。

HTTP 正文为 `model/state/questions`，`state` 包含任务、当前运动、当前目标、后续航路点和已知响应时长。候选为 8 档非负速度 × 8 个全局方向，提供近似轨迹、入圈速度和超限事实、同指令两段时长的假设预览。实际只执行一段。由于缺少艏向、转速和执行进度，启动与急转预测不精确，模型请求中明确标记。Python 不打分、排序、过滤或代选。

完整当前请求样本见 [choice64_request.json](examples/choice64_request.json)。输出动作是 `{"speed": 航速节数, "course": 全局角度}`，不是相对船头偏移。执行、统计定义与接口限制见[当前协议](../docs/bian/当前控制与Jev协议.md)。

## 通信与记录

本机 Unity `127.0.0.1:5005` ↔ Python `127.0.0.1:5006` 使用 UDP，完整 JSON 单数据报，无应用层分包。暂停等待，每份请求及结果只发一次，不自动重发；失败通过 Retry Jev 重新采样。身份与采样 tick 校验保留。

通信报文不携带自定义 `version` 字段，也不维护递增的通信版本号；按消息类型、具体字段和运行身份校验。地图及轨迹文件的 `schema_version` 属于文件格式定义，继续保留。

API 为 `POST https://api.typesafe.ai/v1/systemone`，默认模型 `jev-latest`。每个工作线程独立复用 HTTPS 连接，预连接不发送业务请求。JSONL 保存原始观测、完整 payload、原始响应、结果和计时；`wait_headers_s` 包含网络及服务端等待，并非纯推理耗时。

完成后记录 `event=completed`；`--exit-on-complete` 可控制退出。甲方组播二进制接收和真实动力系统适配尚未实现。`examples/route_ticket.py` 是独立 SDK 示例。

## 验证

离线检查，不操作 Unity 或调用 API：

```powershell
python -m unittest discover -s jev_api/tests -v
```

实际运行通过 Unity 界面选择地图和启动 Jev，见[项目 README](../README.md)。后续事项见本地[代办清单](../docs/bian_local/Jev请求优化代办清单.md)（不纳入版本控制）。
