# USV2DSimulator

基于 **Unity Universal 2D** 的无人艇（USV）运动仿真平台，提供二维动力学、参数调节、地图任务、状态和轨迹显示，支持手动控制与 Jev 决策控制。

## 功能

- **二维运动**：前进、减速、后退和转向，包含速度与加速度限制、阻尼；当前简化配置的加速度指令立即生效。
- **手动 / Jev 控制**：WASD 操作，或由 Jev 每次选择一个动作，执行后依据最新状态重新决策。
- **实时参数面板**：调整动力学参数，选择地图和动作持续时间。
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

首次使用按以下顺序操作：

1. 保持 Unity 已打开 `Main.unity` 并进入 Play；首次测试选择右侧 **Level 0**。
2. 在项目根目录打开 PowerShell，创建日志目录并启动客户端：

   ```powershell
   New-Item -ItemType Directory -Force Logs | Out-Null
   python jev_api/boat_client.py --log Logs/jev.jsonl
   ```

3. 终端出现 `Jev worker listening on 127.0.0.1:5006` 表示客户端已启动，保持这个终端运行。默认 Unity 使用本机 UDP 5005，Python 使用 5006。
4. 回到 Unity，点击左下角模式按钮切换为 **Jev**。在右侧 **`JEV CONTROL`** 设置 **Action duration = 0.40 s**，再点击 **`Start Jev`**。

只切换 Jev 模式还不会开始决策，必须点击 `Start Jev`。开始后时长锁定；按 R 重置后可重新设置。

**如何确认运行正常：**界面在 `Waiting` 和 `Executing` 间切换，船的位置随执行更新；终端持续输出 `decision_result`，`Logs/jev.jsonl` 记录实际请求、响应和动作。`Waiting` 期间船保持静止是当前设计：等待 API 不消耗仿真时间，收到动作后才执行下一段 0.4 秒。

## 当前 Jev 决策方式

每次请求一道 `action` Choice，从 8×7 个组合中选择线/角加速度，执行结束后用最新状态重新决策。Python 将当前目标转为船体坐标，提供线速度、角速度及必要物理参数，并按目标类型生成打卡或停车题目；参考路线仅供显示和轨迹对比。

当前两个执行器时间常数均为零，决策期间暂停物理仿真。Unity/Python 使用 version 5 协议，每次返回一个 `action` 对象。字段与处理过程见 [Jev API 调用说明](docs/bian/当前控制与Jev协议.md)。

## 地图与任务

右侧 `MAP LEVEL` 可选择 **Level 0～4**，包含基础路线、交替弯道、折返和密集交叉路线。切换地图会重置小船及任务；`Reload task` 重新加载当前地图。

按序通过任务点；中间点不要求停车，默认终点要求同时满足“距离不超过 1 米”和“绝对线速度不超过 0.1 m/s”，不要求角速度归零。参考路线供监控和轨迹对比查看。完成任务后，本局仿真暂停。

地图数据位于 `Assets/StreamingAssets/fixed_route.json` 和 `Assets/StreamingAssets/Maps/`，Editor 与独立程序均从这里加载。

## 轨迹自动导出

任务完成、状态变为 **Completed** 后，程序自动保存轨迹 JSON，并调用绘图程序生成 PNG 对比图，无需手动导出。图片生成后，点击右侧 **`Open trajectory plot`** 查看。

- Unity Editor：`Logs/trajectories/`
- 独立程序：`Application.persistentDataPath/Trajectories/`

<details>
<summary>首次使用时的绘图环境配置</summary>

PNG 生成依赖 Python 和 Matplotlib；环境已配置时无需重复操作。首次需要配置时，在项目根目录运行：

```powershell
python -m pip install -r Assets/StreamingAssets/Tools/requirements.txt
python Tools/configure_trajectory_plot.py
```

配置完成后，每局任务完成都会自动导出轨迹并生成图片。

</details>

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
