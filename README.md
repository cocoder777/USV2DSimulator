# USV2DSimulator

基于 **Unity Universal 2D** 的无人艇（USV）运动仿真平台，提供二维动力学、参数调节、地图任务、状态和轨迹显示，支持手动控制与 Jev 决策控制。

## 功能

- **二维运动**：前进、减速、后退和转向，包含速度与加速度限制、阻尼和响应延迟。
- **手动 / Jev 控制**：WASD 操作，或由 Jev 返回动作批次并顺序执行。
- **实时参数面板**：调整动力学参数，选择地图、动作数量和单动作持续时间。
- **任务与轨迹**：显示参考路线和实际轨迹，按序打卡并判断终点停车；完成后自动导出轨迹数据和对比图。

## 环境

| 项目 | 配置 |
| --- | --- |
| Unity Editor | **6000.6.5f1**，以 `ProjectSettings/ProjectVersion.txt` 为准 |
| 渲染 | Universal Render Pipeline，2D 场景 |
| Python | 建议 3.10 或更新版本；Jev 客户端仅使用标准库，轨迹绘图需要 Matplotlib |
| 运行场景 | `Assets/Scenes/Main.unity` |
| 主要开发环境 | Windows / PowerShell |

项目使用简化二维运动模型，未包含严格船舶水动力学、独立横向漂移、风浪和障碍物。

## 快速开始

### 1. 获取并打开项目

```powershell
git clone https://github.com/cocoder777/USV2DSimulator.git
cd USV2DSimulator
```

在 Unity Hub 中添加项目，使用 Unity **6000.6.5f1** 打开。首次打开等待资源和依赖导入完成，然后打开 **`Assets/Scenes/Main.unity`**，点击 Play。

### 2. 手动控制

默认是 Manual 模式；点击 Game 窗口使键盘输入生效。

| 操作 | 功能 |
| --- | --- |
| W / S | 向前加速 / 减速并可继续后退 |
| A / D | 左转 / 右转 |
| R | 重置小船、任务进度和本局轨迹 |
| 左下角模式按钮 | 切换 Manual / Jev |
| 右侧参数面板 | 实时调节动力学参数 |

### 3. Jev 控制

按 [Jev API 调用说明](docs/bian/当前控制与Jev协议.md) ，使用`python jev_api/boat_client.py`启动 Python 客户端。

在右侧 `JEV ACTION PLAN` 中设置动作数量（1～10）和单动作持续时间，切换到 Jev 模式，点击 **`Start Jev`**。开始后这两个参数锁定；按 R 重置后可以重新调整。初始配置为 1 个动作、每个持续 0.4 秒。

每个动作通过两道 Score 决定线加速度和角加速度档位。Jev 仅接收全局坐标下的船状态、当前目标、动力学参数和动作配置；参考路线只用于显示和轨迹对比。多动作问题独立求值，整批按序执行。

等待回答时仿真暂停，界面仍可操作。调用失败时点击 `Retry Jev`，或切回 Manual。

## 地图与任务

右侧 `MAP LEVEL` 可选择 **Level 0～4**，包含基础路线、交替弯道、折返和密集交叉路线。切换地图会重置小船及任务；`Reload task` 重新加载当前地图。

按序通过任务点；中间点不要求停车，终点需要同时满足距离和速度要求。参考路线供监控和轨迹对比查看。完成任务后，本局仿真暂停。

地图数据位于 `Assets/StreamingAssets/fixed_route.json` 和 `Assets/StreamingAssets/Maps/`，Editor 与独立程序均从这里加载。

## 轨迹导出

首次使用轨迹绘图时，在项目根目录运行：

```powershell
python -m pip install -r Assets/StreamingAssets/Tools/requirements.txt
python Tools/configure_trajectory_plot.py
```

任务完成后自动保存 JSON 和 PNG，点击右侧 **`Open trajectory plot`** 查看。Editor 输出到 `Logs/trajectories/`；独立程序输出到 `Application.persistentDataPath/Trajectories/`。绘图失败时，面板会显示原因并提供重试按钮。

## 项目结构

```text
Assets/Scenes/Main.unity     主仿真场景
Assets/Scripts/             仿真、任务、界面和通信代码
Assets/StreamingAssets/     地图数据与运行时绘图工具
jev_api/                    Jev 客户端与独立 API 示例
Tools/                      地图生成与绘图配置工具
Packages/                   Unity 包依赖
ProjectSettings/            Unity 项目配置
docs/                       详细说明
```

## 更多资料

- [Jev API 调用说明](docs/bian/当前控制与Jev协议.md)：接口使用、请求和回答格式、本地解析与执行。
