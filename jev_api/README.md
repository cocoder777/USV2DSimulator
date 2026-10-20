# Jev 航速 / 航向客户端

负责接收 Unity 观测、整理模型输入、请求 Jev 并返回动作。启动和 Unity 操作见[项目 README](../README.md)。

## 文件职责

| 文件 | 职责 |
| --- | --- |
| `boat_client.py` | UDP 收发、观测验证、结果转发及日志 |
| `decision_worker.py` | 请求去重、后台调度、维护运行上下文 |
| `route_context.py` | 经纬度转局部坐标、匹配当前航段、维护进度、计算贴线与打卡几何关系 |
| `choice_control.py` | 构造任务说明和64个候选项，解码 Jev 的唯一选择 |
| `tools/navigation_motion.py` | 按 Unity 执行规律预测候选轨迹、结束航速、航向与转速；不选择动作 |
| `tools/http_transport.py` | HTTPS 连接复用、代理兼容、预连接及请求计时 |
| `tests/test_navigation.py` | 离线单元测试 |
| `tests/evaluate_jev.py` | 手动闭环回归与轨迹评价；会操作 Unity、调用真实 API，不属于离线单测 |

处理顺序：`serve → submit → _execute → build_payload → transport.call → decode_action → collect_results`。

## 信息筛选与请求

Unity 发送当前完整任务、船况及模拟经纬度；Python 缓存完整航线，转换成米制船体坐标，只把当前决策需要的信息送给模型。位置和转向采用英文“前/后、左/右、前进/倒退”描述。

HTTP 正文为 `model/state/questions`；`questions.action` 包含问题说明和全部64个候选项。完整样本见 [choice64_request.json](examples/choice64_request.json)。

| state 字段 | 内容 |
| --- | --- |
| `task` | 任务背景、当前阶段、打卡次序、完成条件、贴线软偏好 |
| `boat` | 当前航速和转速；瞬时响应时省略对应信息 |
| `route` | 当前偏差、近处航线方向、沿线剩余距离、当前段前视与通过目标后的预览 |
| `target` | 当前打卡点的位置、距离、到达半径、是否在圈内 |
| `execution` | 动作时长、坐标含义、响应规律、当前指令及剩余响应时间 |

当前段前视截止尚未通过的打卡点，后续预览单列。候选预测未入当前圈时不匹配未来段；入圈后开放后续局部航线，沿线匹配范围受预测移动距离与初始偏差约束，减少交叉与折返航线的干扰。

三个任务阶段为普通打卡、终点接近和终点圈内停车。候选项包含轨迹位置、贴线误差、沿线进度、入圈情况、结束航速/转速和船头夹角，以及保持同指令两段时长的假设预览。实际只执行一段时长。

`--tracking-preference-m` 设置贴线软偏好，默认 **0.25 m**，独立于打卡半径。Python 不对候选打分、排序、过滤或代选，Unity 执行 Jev 的实际选择。

## 动作与通信

动作是目标航速与相对采样船头的目标航向，转换为固定世界航向后执行。速度按固定时长匀加速，航向和角速度平滑过渡；相同世界目标继续剩余过程。动作档位、响应公式和停车规则见[当前协议](../docs/bian/当前控制与Jev协议.md)。

本机 Unity `127.0.0.1:5005` ↔ Python `127.0.0.1:5006`，使用 UDP v9。Unity 暂停等待，不产生并行决策。保留运行身份与采样 tick 校验，拒绝过期、重复或非法结果；请求和结果各发送一次，不自动重发，失败后通过 Retry Jev 重新采样。

API 使用 `POST https://api.typesafe.ai/v1/systemone`，默认模型 `jev-latest`。预连接仅建立 HTTPS 连接，不发送业务请求；每个工作线程独立复用连接。

JSONL 保存原始观测、实际请求 `payload`、原始响应、执行结果与计时。`wait_headers_s` 包含网络和服务端等待，不等于纯推理时间。完成后记录 `event=completed`；`--exit-on-complete` 可控制客户端退出。`examples/route_ticket.py` 是独立 SDK 示例，不参与仿真决策。

## 离线检查

在项目根目录执行，不连接 Unity 或调用 API：

```powershell
python -m unittest discover -s jev_api/tests -v
```

已做工作与后续计划见[优化代办清单](../docs/bian/Jev请求优化代办清单.md)；实验过程和结论集中记录在 `docs/bian/Jev仿真实验记录.md`。
