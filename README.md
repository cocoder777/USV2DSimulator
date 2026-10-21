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

当前采用暂停决策，不模拟风浪、障碍物或真实动力系统。船况使用甲方态势字段的物理量：经纬度、实际航速（节）及运动方向。决策不使用额外艏向、转速或执行器进度；候选预测是简化估计。已加入逐点速度上限，甲方二进制组播、执行状态回复和转弯半径仍待对接。真实动力系统的输出坐标约定尚待确认，仿真使用全局目标航速/航向。

- [Python 客户端结构与请求内容](jev_api/README.md)
- [当前控制与通信协议](docs/bian/当前控制与Jev协议.md)
- [Jev 请求优化代办清单](docs/bian/Jev请求优化代办清单.md)
- [甲方对接说明](docs/甲方接口/对接单艇决策服务.md)
