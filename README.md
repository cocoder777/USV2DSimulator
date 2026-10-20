# USV2DSimulator

基于 Unity Universal 2D 的无人艇仿真平台，用于验证 Jev 沿地图虚线航行、按序打卡和终点停车的能力。支持手动控制与 Jev 决策。

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

Unity 采样后暂停，等待 Jev 从 **8 档速度 × 8 个航向**中选择一个动作；执行指定时长后再次采样。航速与转向均有渐变过程，两个切换时间独立可调，默认各 **0.5 s**，设为 0 可恢复对应的瞬时响应。

地图虚线是参考航线。中间点按序进入打卡圈即可，终点需要同时位于圈内且满足低速停车条件。Unity 负责判定任务完成，并自动保存轨迹。

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

当前采用暂停决策，不模拟风浪、障碍物或真实动力系统。借用甲方的航线任务思想，尚未接入其组播帧、执行状态回复、计划航速和转弯半径约束。当前保留 Level4 两次完整通过的请求版本，仍需跨地图及参数回归。

- [Python 客户端结构与请求内容](jev_api/README.md)
- [当前控制与通信协议](docs/bian/当前控制与Jev协议.md)
- [Jev 请求优化代办清单](docs/bian/Jev请求优化代办清单.md)
- [甲方对接说明](docs/对接单艇决策服务.md)
