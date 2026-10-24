# USV2DSimulator

基于 Unity Universal 2D 的无人艇仿真平台，用于验证 Jev 按序通过航路点、遵守逐点通过速度上限和终点停车的能力。支持手动控制与 Jev 决策。

## 环境与启动

- Unity **6000.6.5f1**，打开 `Assets/Scenes/Main.unity` 并进入 Play。
- Python 3.10+；Jev 客户端仅依赖标准库，轨迹绘图依赖 Matplotlib。

在项目根目录执行：

```powershell
New-Item -ItemType Directory -Force Logs | Out-Null
python jev_api/boat_client.py --log Logs/jev.jsonl
```

等待终端显示 `Jev worker listening`，在 Unity 选择地图、切换 Jev 模式，点击 **Start Jev**。当前验证常用动作时长为 **0.40 s**。

## 运行规则与操作

Unity 采样后暂停，等待 Jev 从 **8 档非负航速 × 8 个全局航向**中选择一个动作；执行指定时长后再次采样。航向正北为 0°，顺时针增加。航速与转向均有渐变过程，两个切换时间独立可调，默认各 **0.5 s**，设为 0 可恢复对应的瞬时响应。

地图虚线仅供显示，不发送给 Python 或 Jev，也不约束点间走法。每点 `routeSpeed` 是通过圈内的实际航速上限（m/s），不是要求达到的参考速度。地图显示“计划航速（通过上限）”及实测通过最高航速。中间点按序入圈，终点还须低速停车；超限会记录和显示，不由执行器强制限速或阻止打卡。Unity 负责判定任务完成，并自动保存轨迹。

每张地图自动生成最多 3 个半径 1 m 的运动障碍物，以红色圆圈表示；障碍物与船共用仿真步进，等待 Jev 时一起暂停。船采用半径 0.5 m 的简化圆形碰撞范围，接触障碍物即任务失败并保存轨迹，碰撞优先于同一步的终点判定。失败后只能按 R 或重新加载地图开始新一轮，Retry Jev 和切换模式不能恢复；可使用原有 Open trajectory plot 打开轨迹图，图中包含障碍物运动轨迹和最终圆圈位置。

| 操作 | 功能 |
| --- | --- |
| W/S、A/D | Manual 模式加速/减速、左/右转 |
| R | 重置船、任务进度与轨迹 |
| 模式按钮 | Manual / Jev |
| Start Jev | 开始决策循环 |
| Retry Jev | 请求失败后重新采样决策 |
| Level 0～4 | 切换地图并重置 |

## 轨迹导出

Editor 导出到 `Logs/trajectories/`；独立程序导出到 `Application.persistentDataPath/Trajectories/`。首次使用绘图：

```powershell
python -m pip install -r Assets/StreamingAssets/Tools/requirements.txt
python Tools/configure_trajectory_plot.py
```

## 当前范围与文档

当前采用暂停决策，支持简化运动障碍物，不模拟风浪或真实动力系统。障碍物和本艇统一发送甲方兵力融合态势（经纬度、节、运动方向），Jev 请求仍明确描述障碍物，新增其速度和方向，候选净距继续使用当前位置快照估计。

Unity/Python 已按最新 `jev_api_guoliang/` 核对甲方 `0x0660/0x0060/0x0666/0x0662/0x0664` 大端帧、态势组播及JSON分片。任务列表只含目标，`currentRoute`为已完成数量；地图起点仍保留在Unity。`unity_only`保留采样关联、快照时戳、兼作实体清单的半径表、动作时长、速度上限、响应配置与到达/停车阈值；删除重复身份、进度、整份候选表和暂停常量，状态回包也不重复动作。这是本项目扩展对象，不是甲方报文类型。默认Unity/Python点播端口5005/5006，态势组播`239.255.0.10:18010`，回环网卡。

`jev_api_guoliang/`是面向甲方的独立程序，本项目运行不依赖它；其最新默认后端为Kev-4B，包含部署脚本和历史实测记录，但当前副本未附权重。本项目仍用真实Jev和原64 Choice。其持续控制、自动续发/重试及`routeRadius`作到达圈的行为不移植，保持Unity暂停等待、物理判定与当前避障策略。

线侧速度全部为节；地图、轨迹与内部物理仍用 m/s。Jev 默认用节表达速度、用米表达距离，说明 `1 节 = 1852/3600 m/s`；启动 Python 时加 `--jev-speed-unit mps` 可仅切回模型的 m/s 表达。既有 Python 客户端需重启加载本次协议改动。算法状态 1/2/3 分别表示执行中/完成/决策失败，碰撞仍由 Unity 单独判定。转弯半径暂不处理。

- [Python 客户端结构与请求内容](jev_api/README.md)
- [当前控制与通信协议](docs/bian/当前控制与Jev协议.md)
- 本地 [Jev 请求优化代办清单](docs/bian_local/Jev请求优化代办清单.md)（不纳入版本控制）
- [甲方对接说明](docs/甲方接口/对接单艇决策服务.md)
