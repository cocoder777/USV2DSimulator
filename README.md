# USV2DSimulator

面向单艇决策算法开发的 Unity 2D 仿真项目。项目的核心是**优化甲方要求的决策模块，并方便将我们的决策模块快速接入甲方仿真系统**。Unity 提供可重复的本地验证环境，模拟任务下发、态势输入和控制执行；`jev_api/` 实现 Python 决策层，将任务与态势组织为 Jev 请求，再把模型选择转换为航速、航向控制。

决策模块面向三个场景：

| 场景 | 甲方要求 | 本地验证现状 |
| --- | --- | --- |
| 航线任务 | 按给定航路点执行航迹跟踪 | `Route`：按序进入航点圈，遵守圈内通过速度上限，终点低速停车 |
| 敌方目标跟随 | 沿目标航迹尾随，默认在目标后方保持间距 | `Distance`：目前只保持两艇直线距离，允许在目标侧方或前方；与甲方要求有差别，后续需改进 |
| 友方伴随 | 在目标体坐标系的纵、横位置带内保位，可要求航向同步 | `Accompany`：进入并保持指定位置带，支持仅保位和稳定后同步航向 |

## 项目由哪些部分组成

- **Unity 仿真端**：二维船舶运动、受限加减速与转向、航点和运动目标、动态障碍物、碰撞和任务统计、操作面板、轨迹导出。每次采样后暂停，等待决策结果，再推进一个动作周期。
- **`jev_api/` Python 决策层**：接收任务和当前态势，校验输入，计算候选动作的运动后果，调用外部 Jev 服务，解码选择并回传控制和状态。这里是决策方法与请求表达的主要迭代位置。
- **外部 Jev 服务**：根据 Python 提供的任务、态势与候选事实，选择一个动作。
- **`jev_api_guoliang/`**：面向甲方接入的独立程序副本，不是本地 Unity 游戏的运行依赖；不能用它的启动方式替代下文的客户端。

本地链路沿用甲方的任务、态势、控制和状态报文，同时增加 Unity 暂停采样所需的扩展信息。当前尚不能把本地客户端视为可直接接入甲方实时仿真的完整服务，具体差异见[通信协议](docs/bian/当前控制与Jev协议.md)。Unity 使用简化运动学，不模拟真实推进器、风浪和水动力；已核对实船加速日志，当前固定能力模型与观测响应存在差异，尚未完成实船校准，详见[核对记录](docs/bian_local/Jev仿真实验记录.md#2026-10-29实船数据与-unity-参数核对)。

## 启动 Jev 决策端

环境：Unity **6000.6.5f1**（版本以 `ProjectSettings/ProjectVersion.txt` 为准），Python **3.10+**。主决策客户端只使用 Python 标准库；`jev_api/requirements.txt` 中的 TypeSafe SDK 用于独立示例，不是启动客户端的必需依赖。

启动前确认 HTTPS 网络可达，且 `jev_api/boat_client.py` 中的 `API_KEY` 是可用凭据；当前入口直接读取该常量，没有读取环境变量的配置入口。

1. 在项目根目录打开 PowerShell，运行：

   ```powershell
   New-Item -ItemType Directory -Force Logs | Out-Null
   python jev_api/boat_client.py --log Logs/jev.jsonl
   ```

2. 等待终端显示 `Jev worker listening`，保持该终端运行。默认与本机 Unity 通信，无需启动 `jev_api_guoliang/`。
3. 用 Unity 打开本项目，打开 `Assets/Scenes/Main.unity`，点击 Editor 的 **Play**。

可用选项通过 `python jev_api/boat_client.py --help` 查看。默认网络地址、模型与单位约定统一见[通信协议](docs/bian/当前控制与Jev协议.md)。

## 在 Unity 中运行三个游戏

先在右侧可滚动面板的 **TASK MODE** 选择 `Route`、`Distance` 或 `Accompany`，再设置对应场景和参数。切换任务、地图、场景或应用任务参数会重置本轮实验。

运行方式：

1. 点击屏幕底部的 **Mode: Manual / Jev**，切换到 `Jev`。
2. 在右侧 **JEV CONTROL** 设置 **Action duration**，默认每次执行 1 秒仿真时间；开始后要重置才能修改该值。当前只支持 `Paused`，界面中的 `Continuous` 尚未实现。
3. 点击右侧或底部的 **Start Jev**。左上角显示当前任务、决策状态、航速等信息；`Waiting` 时船、目标和障碍物暂停，收到匹配结果后继续。
4. 观察游戏结果和统计。开始新一轮时按 **R** 或 **Reload task**，然后重新点击 **Start Jev**。

### Route：航点寻线

- 选择 **Route**，在 **MAP LEVEL** 中选择 `Level 0`～`Level 4`。地图分别包含 3、5、7、9、9 个中间检查点，各另有一个终点；通信中的目标点总数因此是 4、6、8、10、10。
- 目标是按顺序进入航点圈，中间点不要求停车，终点要求在圈内低速停车。虚线只是地图参考，不要求沿线行驶。
- 每个航点的计划航速在本地解释为**圈内实际通过速度上限**。超过上限会记录和显示，但不会自动限速或阻止打卡，因此显示完成也可能包含超速记录。
- 地图带运动障碍物。橙色安全圈用于统计安全距离不足，红色实体圈接触会导致碰撞失败。

### Distance：目标距离保持

- 选择 **Distance**，在 **FOLLOWING MAP** 中选择 `Straight`、`Gentle turn`、`Complex` 或 `Irregular`。
- 设置中心到中心的 **Min / Max** 距离，点击 **Apply distances and reset**；默认 100～200 米。最小距离必须大于两艇实体半径之和。
- 有障碍物的场景可切换 **Scenario obstacles**，切换也会重置。紫色显示目标和轨迹，红/绿圆显示距离边界，青色标记是两艇连线上的参考位置。
- 关注 **Range**、距离变化率及 **in band** 时间占比。进入距离区间后仍持续执行，实验按场景时长结束；结束不代表全程合格。

**此游戏当前不要求沿目标航迹或在其后方尾随，尚不能代表甲方敌方跟随任务已实现。**

### Accompany：友方伴随

- 选择 **Accompany**，在 **FRIENDLY ACCOMPANY** 中选择 `Straight`、`Gentle turn`、`Speed / stop` 或 `Irregular`。当前伴随场景不含普通障碍物。
- 设置 **Along min / max** 和 **Across min / max**：以友艇为原点，纵向正值在前、负值在后；横向正值在右舷、负值在左舷。默认带为后方 140～160 米、右侧 70～90 米。
- **Sync heading (straight / stopped)** 决定是否在稳定保位后同步航向；不勾选时只要求保位。**Task maxSpeed** 为任务航速上限，填 `0` 表示使用本艇物理上限。
- 可以设置目标初始航速和方向，再点击 **Apply accompany settings and reset**。绿色显示友艇，青色矩形显示允许位置带，本艇从带外开始。
- 关注纵横位置、**In position** 时间占比及航向误差。转弯时优先保位；实验到时结束不等于持续稳定伴随已经通过验收。

## 常用操作与异常恢复

| 操作 | 效果 |
| --- | --- |
| `W / S`、`A / D` | `Manual` 模式前进/减速或倒航、左/右转；手动体验不需要 Python |
| `R` / `Reload task` | 重置船、目标、任务进度及本轮记录；Jev 模式需重新开始 |
| `Retry Jev`（底部） | 决策失败后重新采样请求；碰撞后需先重置 |
| `Stop via Jev`（右侧） | 中断原目标，交由 Jev 选择停车动作；实际停稳后进入 `Stopped` |
| `Simulation speed` | 1～10 倍仿真速度；等待 Jev 时仍暂停，不能加速网络推理 |
| 鼠标滚轮 / `Map overview` | 调整镜头缩放 / 切换全图 |

右侧 **USV PARAMETERS** 可调整本艇运动和尺寸参数，手动与 Jev 模式共用。修改这些值会影响任务难度和动作预测。

若停在 `Waiting`，先查看 Python 终端和左上角 UDP/决策状态。确认客户端正在运行、端口无冲突且 Jev 请求可用；出现失败提示后用 **Retry Jev** 重新采样。当前没有自动重发和本地替代决策，漏包也可能需要手动 Retry。

更新 Python 代码后需要重启决策端。当前航线和距离保持各使用 64 个候选，友方伴随使用 128 个；详细配置见决策层说明。HTTP 400 的具体服务端原因会显示在终端错误中。

## 查看轨迹

任务完成、碰撞失败或 Jev 停车后自动保存轨迹 JSON。Editor 保存到 `Logs/trajectories/`，独立程序保存到 `Application.persistentDataPath/Trajectories/`。

首次使用绘图，在项目根目录执行：

```powershell
python -m pip install -r Assets/StreamingAssets/Tools/requirements.txt
python Tools/configure_trajectory_plot.py
```

图像生成后可点击 **Open trajectory plot**；绘图失败可点击 **Retry trajectory plot**，已保存的 JSON 仍可使用。Python 决策日志在启动命令指定的 `Logs/jev.jsonl`，可用来检查实际请求、模型响应和耗时。

## 文档入口

- [Python 决策层：结构、调用链和各任务实现](jev_api/README.md)
- [当前控制与 Jev 协议：通信、报文、单位和接入差异](docs/bian/当前控制与Jev协议.md)
- [甲方单艇决策服务说明](docs/甲方接口/对接单艇决策服务.md)
- [Jev 请求优化待办](docs/bian_local/Jev请求优化代办清单.md)（本地文档，目录被 Git 忽略）
