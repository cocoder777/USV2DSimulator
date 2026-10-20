# Jev 航速 / 航向客户端

## 运行

```powershell
python jev_api/boat_client.py --log Logs/jev.jsonl
```

Unity 进入 Play，选择 Level 0、Jev 模式、0.40 秒动作时长，点击 Start Jev。默认一个工作线程；Unity 暂停等待每份决策。API 失败或 UDP 丢包时通过 Retry Jev 重新请求，不自动重试 POST。

## 文件职责

| 文件 | 职责 |
| --- | --- |
| `boat_client.py` | 接收本机 UDP、验证请求、收取结果、日志、发送结果和接收完成通知 |
| `decision_worker.py` | 去重、后台 HTTPS 调度、按运行身份保存几何上下文 |
| `route_context.py` | 经纬度转换、匹配当前航段、维护沿线进度、截取前方虚线 |
| `choice_control.py` | 构造 64 个速度/航向 Choice，解释动作并解码选项 |
| `http_transport.py` | 原有 HTTPS 连接复用、代理兼容、预连接、阶段计时 |

处理顺序：`serve → submit → _execute → build_payload → transport.call → decode_action → collect_results`。

## 两层信息筛选

Unity 提供模拟经纬度：固定原点经度 120.5、纬度 30.2，球半径 6378137 米，局部 +X 东、+Y 北。八位小数的经纬度在当前地图范围内约有毫米级量化误差。它是仿真用小区域坐标映射，不是真实测绘转换。

每个请求自带当前完整任务，避免依赖可能丢失的独立初始化消息；Python 缓存转换后的航线。内部 UDP 上限 65000 字节，现有五张地图均验证可容纳。该协议仅用于本机，不是甲方组播协议；扩大地图时需要重新评估消息大小。

原始观测包括船位置/航向、航线编号、有序虚线采样点、有序打卡点、下一个打卡点索引、当前段、到达半径和停车阈值。索引从零开始，`currentRoute=next_waypoint_index-1`。

Python 限制在当前打卡段及其到达圈附近匹配航线，并结合上次进度限制搜索窗口。重置或换任务时清除历史。模型仅接收：

- 当前最近航线点的 `[forward,left]` 坐标及横向距离。
- 沿虚线前方 1、2、3、4、6 米处的坐标，尾端重复点去重。
- 下一个打卡点的坐标、到达半径。
- 动作持续时间；终点停车要求写入题目。

这些是几何事实，不计算或提供推荐动作。零惯性版本不向模型提供当前速度。

## 动作

八档速度比例为 `[-1,-0.5,0,0.1,0.25,0.5,0.75,1]`；正数乘最大前进速度，负数乘最大后退速度。八个相对船头方向为 `[0,45,90,135,180,-135,-90,-45]` 度，正角左转。

选项名如 `forward_half__ahead_left`，解码为 `speed_index=5, heading_index=1`。题目明确说明航向瞬间设置一次，保持整个动作时长，不是转速或角加速度。选项同时列出该动作的精确相对终点，帮助模型理解正负速度和方向；不对选项打分。

## 通信与日志

本机 Unity `127.0.0.1:5005` ↔ Python `127.0.0.1:5006`，使用 UDP v6。结果只含一个 `action`，保留身份、采样 tick 和动作时长。旧 v5 不接受；Unity 校验重复和过期响应。请求和结果各只发送一次，不发送 ACK。

HTTP 继续使用 `POST https://api.typesafe.ai/v1/systemone`，顶层 `model/state/questions`，默认 `jev-latest`。预连接只建立 HTTPS 连接，不发送业务请求；每个工作线程独立复用连接。并发基础设施保留，但本版 Unity 不产生并行请求。

JSONL 保留原始 `request`、实际模型 `payload`、`raw_response`、`result` 和计时。`wait_headers_s` 包含网络与服务端等待，不代表纯推理耗时。任务完成后记录 `event=completed`；`--exit-on-complete` 决定随后是否退出。

`examples/route_ticket.py` 是独立 SDK 示例，不参与无人艇决策。`examples/choice64_request.json` 是当前格式示例。
