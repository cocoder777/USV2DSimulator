# USV2DSimulator

基于 **Unity Universal 2D** 的无人艇（USV）运动仿真平台，提供二维动力学、参数调节、地图任务、状态和轨迹显示，支持手动控制与 Jev 决策控制。

本分支包含 Kev / Laya 本地模型接入、切换按钮和可迁移部署脚本。新电脑请先阅读[Kev 与 Laya 使用指南](docs/Kev与Laya使用指南.md)：权重按固定版本另行下载，不随 Git 仓库上传；Python 环境在各自电脑上重建。当前两种模型尚未通过导航评估。

## 功能

- **二维运动**：前进、减速、后退和转向，包含速度与加速度限制、阻尼；当前简化配置的加速度指令立即生效。
- **手动 / Jev 控制**：WASD 操作，或由 Jev 每次选择一个动作，执行后依据最新状态重新决策。
- **实时参数面板**：调整动力学参数，选择地图和动作持续时间。
- **模型按钮切换**：Run Jev 明确调用云端 Jev；Run Laya / Run Kev 使用本地模型。从当前地图起点重新运行，显示实际模型和决策耗时，保存带模型名称的对比轨迹。
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
git clone --branch codex/kev-laya-integration https://github.com/cocoder777/USV2DSimulator.git
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

推荐直接使用按钮：

1. 打开 `Main.unity` 并进入 Play，选择右侧地图。
2. 在右侧 **MODEL** 点击 **Run Jev**（云端）、**Run Laya** 或 **Run Kev**（本地）。Jev 按钮优先读取环境变量 `JEV_API_KEY`，其次读取项目根目录 `.env.jev.json` 中的同名字段；两者均缺少时界面明确提示。克隆项目后需自行配置云端密钥，本地密钥文件不随仓库分发。
3. 等待模型加载，服务和 Python 客户端自动启动，随后自动进入 Jev 运行；无需另开终端。
4. 点击另一个模型会保存上一轮轨迹，保留当前地图，从相同起点重跑。Jev 按钮恢复 GitHub `42088e5` 配置：`jev-latest`、original 提示词、0.4 秒动作、0.02 秒固定步长和原九项动力学参数；Laya/Kev 保持当前参数。**Restart same model** 只重跑当前模型，无需重新加载权重。

模型加载期间暂停运动；面板显示模型、仿真时间、打卡数、最近决策耗时与动作。切换/停止 Play 时，未完成轨迹也会保存，文件和图片带模型标识，不会被标为成功。原始请求/回答在 `Logs/model-switch/`，轨迹在 `Logs/trajectories/`。比较时使用相同**仿真时间**，等待推理的墙钟时间不计入仿真时间。

按钮当前支持 Windows 本机 `D:\AI\Laya` 与 `D:\AI\Kev-0.8B` 部署。只会自动关闭能确认属于本项目的旧客户端；其它程序或归属不明确的相对路径客户端占用端口时，会提示先手动关闭。详情与测试范围见[模型按钮切换说明](docs/模型按钮切换.md)。

#### 手动启动（兼容原方式）

也可按以下顺序手动启动，使用按钮时不必重复执行：

项目客户端现在默认使用本地 **Laya-multilingual**。先启动模型服务：`powershell -ExecutionPolicy Bypass -File D:\AI\Laya\start-laya.ps1`。模型切换方式见下方说明。

1. 保持 Unity 已打开 `Main.unity` 并进入 Play；首次测试选择右侧 **Level 0**。
2. 在项目根目录打开 PowerShell，创建日志目录并启动客户端：

   ```powershell
   New-Item -ItemType Directory -Force Logs | Out-Null
   python jev_api/boat_client.py --log Logs/jev.jsonl
   ```

3. 终端出现 `Jev worker listening on 127.0.0.1:5006` 表示客户端已启动，保持这个终端运行。默认 Unity 使用本机 UDP 5005，Python 使用 5006。
4. 回到 Unity，点击左下角模式按钮切换为 **Jev**。在右侧 **`JEV CONTROL`** 设置 **Action duration = 0.40 s**，再点击 **`Start selected / external client`**，使用手动启动的客户端。点击 **Start Jev** 则明确选择云端 Jev，会接管可确认属于本项目的旧客户端。

只切换 Jev 模式还不会开始决策，必须启动所选客户端；模型按钮则会自动开始。开始后时长锁定；按 R 重置后可重新设置。

**如何确认运行正常：**界面在 `Waiting` 和 `Executing` 间切换，船的位置随执行更新；终端持续输出 `decision_result`，`Logs/jev.jsonl` 记录实际请求、响应和动作。`Waiting` 期间船保持静止是当前设计：等待 API 不消耗仿真时间，收到动作后才执行下一段 0.4 秒。

## 当前 Jev 决策方式

### 本机模型与后端切换

Laya 独立部署于 `D:\AI\Laya`，使用官方 Laya-multilingual 权重、独立 Python 3.12 和 CUDA 环境。Kev 的权重和环境保留在 `D:\AI\Kev-0.8B`。本次没有训练模型。

```powershell
# 启动后台模型服务，等待 ready
powershell -ExecutionPolicy Bypass -File D:\AI\Laya\start-laya.ps1
# 在另一个窗口启动客户端并保持运行
powershell -ExecutionPolicy Bypass -File D:\AI\Laya\start-boat.ps1
```

模型服务已启动时不用重复启动。手动启动客户端后，Unity 选择 **Jev** 模式并点击 **Start selected / external client**。**Start Jev** 现在明确启动云端 Jev。仅点击 Play 不会自动启动模型服务或 Python 客户端。

| 后端 | 客户端选项 | HTTP 地址 | 输入格式 |
| --- | --- | --- | --- |
| Laya-multilingual，默认 | `--backend laya` | `http://127.0.0.1:8010/v1/systemone` | compact，精简描述、保留全部 56 个动作 |
| Kev-0.8B | `--backend kev` | `http://127.0.0.1:8009/v1/systemone` | original，原输入 |
| 云端 Jev | `--backend cloud` | `https://api.typesafe.ai/v1/systemone` | original，原输入 |

切换前在客户端窗口 Ctrl+C 停止当前客户端，再启动目标模型服务和对应客户端。同一时间只保留一个项目客户端。云端需先设置 `$env:JEV_API_KEY`，不会在本地服务失败时自动调用云端。

```powershell
python jev_api/boat_client.py --backend kev --log Logs/kev.jsonl
python jev_api/boat_client.py --backend cloud --log Logs/cloud.jsonl
```

显式 `--endpoint` 和 `--model` 覆盖所选后端的地址及模型默认值；`--prompt-profile original|compact` 可单独选择输入格式作对照，默认按后端选择。Laya 使用 `max_len=4096`、`head_max_len=3072`；服务按实际分词和编码结果拒绝任何状态、指令或选项截断，不筛掉动作或替模型选动作。客户端日志包含后端、输入格式、token 使用量、原始响应及耗时。

检查或停止 Laya：

```powershell
powershell -ExecutionPolicy Bypass -File D:\AI\Laya\check-laya.ps1
powershell -ExecutionPolicy Bypass -File D:\AI\Laya\stop-laya.ps1
```

实际部署和验证结果见 `D:\AI\Laya\README.md` 与 `logs`。Laya 预热后接口实测约 0.10～0.13 秒，但在多种状态下仍重复选择不合理动作，**运行速度更快不代表已经能够正确导航**。Unity 的 UDP v5、5005/5006 端口、动力学参数和暂停等待机制保持不变。

独立 Unity 测试副本已执行 300 次决策、6000 个物理步，连续控制、模式切换、重置和服务故障暂停通过；Level 0 未完成，第一个中间点未到达，艇持续倒车转圈。未进行原编辑器窗口的视觉验收。

针对倒退问题，已补充正负指令验证、模型输入对照，以及两种模型各 12 次真实决策到 Unity 执行的逐步核对。链路正常，模型在相同状态下仍会因选项顺序改变动作；倒退尚未修复，默认控制输入未替换。复测工具为 `Tools/diagnose_model_control.py` 和 `Tools/verify_model_trace.py`，详见[模型控制诊断](docs/模型控制诊断.md)。

### 本机 Kev-0.8B

本机部署位于 `D:\AI\Kev-0.8B`，模型、Python 环境、缓存及日志独立于项目。Kev 是兼容 System One 接口的另一款决策模型，使用预训练权重，不需要先训练；决策质量需单独验证。

本机已通过 CUDA 推理、完整 56 选项接口、离线重启、12 项 Python 测试，以及独立 Unity 测试副本的连续控制、模式切换、重置和服务中断暂停检查。**初始目标在前方时模型选择了反向加速，尚不能认为具备有效导航能力**；完整任务完成和画面效果未验证。原项目场景、C# 与动力学参数没有修改。

1. 在 PowerShell 启动模型服务：

   ```powershell
   powershell -ExecutionPolicy Bypass -File D:\AI\Kev-0.8B\start-kev.ps1
   ```

2. 出现 ready 后，在另一个 PowerShell 启动客户端并保持运行：

   ```powershell
   powershell -ExecutionPolicy Bypass -File D:\AI\Kev-0.8B\start-boat.ps1
   ```

3. Unity 打开 Main 场景并进入 Play，选择 Level 0，切换 **Jev**，设置动作时长 **0.40 s**，点击 **Start selected / external client**，使用已启动的本机 Kev 客户端。也可直接点击 **Run Kev** 自动管理服务与客户端。

服务地址为 `http://127.0.0.1:8009/v1/systemone`，模型标识 `kev-latest`；Unity 与 Python 之间仍使用 UDP 5005/5006、version 5。模型默认离线加载，只监听本机。查看身份和状态：

```powershell
Invoke-RestMethod http://127.0.0.1:8009/v1/models
```

客户端窗口按 Ctrl+C 停止。停止模型服务：

```powershell
powershell -ExecutionPolicy Bypass -File D:\AI\Kev-0.8B\stop-kev.ps1
```

日志和实际部署验证结果见 `D:\AI\Kev-0.8B\logs` 及该目录的 README。调用本机服务不会发送云端 API 密钥。切回云端时，先停止本地客户端，在当前 PowerShell 设置 `$env:JEV_API_KEY`，再运行 `python jev_api/boat_client.py --backend cloud --log Logs/jev.jsonl`；密钥不再硬编码在该客户端中。勿同时启动两个客户端占用 5006 端口。

### 请求与动作

每次请求一道 `action_0` Choice，从 8×7 个组合中选择线/角加速度，执行结束后用最新状态重新决策。Python 将当前目标转为船体坐标，提供线速度、角速度及必要物理参数，并按目标类型生成打卡或停车题目；参考路线仅供显示和轨迹对比。

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

- [Kev 与 Laya 使用指南](docs/Kev与Laya使用指南.md)：新电脑安装、模型下载、按钮与手动操作、迁移和故障排查。
- [Jev API 调用说明](docs/bian/当前控制与Jev协议.md)：接口使用、请求和回答格式、本地解析与执行。
- [模型统一对照报告](docs/模型统一对照报告.md)：Classic、Laya、Kev 九场景真实 Unity 闭环、服务中断诊断和统一指标。
- [PD 监督数据采集方案](docs/PD监督数据采集方案.md)：数据字段、采样频率、规模规划与后续微调验收条件；尚未进行训练。
