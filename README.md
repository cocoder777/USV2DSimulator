# USV2DSimulator

基于 Unity Universal 2D 的无人艇仿真平台，用于验证 Jev 沿地图虚线航行、按序打卡和终点停车的能力。支持手动加速度控制与 Jev 航速/航向控制。

## 环境与启动

- Unity **6000.6.5f1**，打开 `Assets/Scenes/Main.unity` 并进入 Play。
- Python 3.10+；正式 Jev 客户端仅依赖标准库，轨迹绘图依赖 Matplotlib。

在项目根目录执行：

```powershell
New-Item -ItemType Directory -Force Logs | Out-Null
python jev_api/boat_client.py --log Logs/jev.jsonl
```

等待终端显示 `Jev worker listening`，在 Unity 选择 Level 0，切换 Jev 模式，设置 Action duration 为 **0.40 s**，点击 **Start Jev**。

Jev 每次从 **8 档有符号速度 × 8 个相对航向**的 64 个选项中选择一个。速度和航向瞬时生效，固定航向执行一段时间，再暂停采集观测。等待 API 时不推进仿真时间。没有并行决策、动力系统响应、风浪或障碍物。

速度比例为 `[-1, -0.5, 0, 0.1, 0.25, 0.5, 0.75, 1]`，正负档分别乘最大前进/后退速度。相对航向为 `[0, 45, 90, 135, 180, -135, -90, -45]` 度，左正右负；相对于采样时船头只设置一次。负速度沿新船头反方向移动。

## 航线与模型信息

地图预设虚线就是参考航线。Unity 发送虚线采样点、打卡点、当前位置和航向；位置以模拟经纬度表示，Python 转换为米制坐标，再转为船体坐标。

Python 保留完整航线，但模型只看当前航线投影点、前方 1/2/3/4/6 米处的虚线位置、下一个打卡点及其到达半径、动作时长。进度匹配结合打卡段和历史进度，避免在交叉处跳段。模型不接收船体编号、航线编号、经纬度、当前速度和加速度参数，不接收推荐速度或方向。

按序进入中间打卡圈即可；终点要求同时位于到达圈内且绝对速度不超过停车阈值。Level 0 默认半径 1 米、停车阈值 0.1 m/s。任务完成后仿真暂停并自动保存轨迹 JSON 和 PNG。

## 操作

| 操作 | 功能 |
| --- | --- |
| W/S、A/D | Manual 模式加速/减速、左/右转 |
| R | 重置船、进度与轨迹 |
| 左下角模式按钮 | Manual / Jev |
| Start Jev | 开始决策循环 |
| Retry Jev | API 或丢包导致停滞后重新采样请求 |
| Level 0～4 | 切换地图并重置 |

Jev 模式仅使用最大前进/后退速度；手动模式保留原有加速度、阻尼与执行器响应参数。

## 轨迹与验证

Editor 导出位置为 `Logs/trajectories/`；独立程序使用 `Application.persistentDataPath/Trajectories/`。首次配置绘图：

```powershell
python -m pip install -r Assets/StreamingAssets/Tools/requirements.txt
python Tools/configure_trajectory_plot.py
```

离线验证：

```powershell
python -m unittest discover -s jev_api/tests -v
```

参见 [客户端结构](jev_api/README.md)、[当前协议](docs/bian/当前控制与Jev协议.md)、[甲方对接说明](docs/对接单艇决策服务.md)。本版本借用甲方的航线任务思想，尚未接入甲方组播帧、执行状态回复、计划航速或转弯半径约束。
