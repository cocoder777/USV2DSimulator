# USV2DSimulator

基于 **Unity Universal 2D** 的无人艇（USV）仿真项目，用于二维运动、路径跟踪和模型决策控制实验。支持手动操作、云端 **Jev** 和本地 **Kev / Laya** 模型控制。

## 主要功能

- **运动与参数调节**：前进、后退、转向，可调整速度、加速度、阻尼和动作时长。
- **地图与任务**：内置 Level 0～4 地图，按顺序通过任务点，并在终点停车。
- **模型切换**：通过 Run Jev、Run Laya、Run Kev 按钮选择模型，查看决策耗时和执行状态。
- **轨迹记录**：显示参考路线与实际轨迹，保存带模型标识的轨迹数据，并支持生成对比图。

项目采用简化二维动力学，适合控制算法实验，暂未包含完整船舶水动力学、风浪和障碍物。

## 快速开始

### 打开项目

```powershell
git clone --branch kev-laya-integration https://github.com/cocoder777/USV2DSimulator.git
cd USV2DSimulator
```

使用 Unity Hub 添加项目，以 **Unity 6000.6.5f1** 打开，等待资源导入完成后，打开 `Assets/Scenes/Main.unity` 并点击 Play。

### 手动控制

- **W / S**：前进、减速和后退。
- **A / D**：左转、右转。
- **R**：重置小船、任务和轨迹。
- 右侧面板可选择地图、调整动力学参数。

### 模型控制

本地模型部署支持 **Windows + NVIDIA CUDA/BF16 GPU**。安装入口需要 Python 3.10 或更新版本，并会创建独立 Python 3.12 环境：

```powershell
python -m pip install uv
powershell -NoProfile -ExecutionPolicy Bypass -File .\Tools\LocalModels\setup.ps1 -Backend both
```

默认部署到 `D:\AI\Laya` 和 `D:\AI\Kev-0.8B`。**模型权重、虚拟环境和密钥不随仓库上传**，安装时下载固定版本；其他目录的配置见[使用指南](docs/Kev与Laya使用指南.md)。

Kev / Laya 在本机运行，无需云端 Jev API 或密钥。

进入 Play 后，在右侧 **MODEL** 选择：

| 按钮 | 功能 |
| --- | --- |
| Run Laya | 启动本地 Laya-multilingual 控制 |
| Run Kev | 启动本地 Kev-0.8B 控制 |
| Run Jev | 调用云端 Jev，自动读取已有密钥配置 |
| Restart same model | 复用当前模型，从当前地图起点重跑 |

Jev 按钮支持读取 `JEV_API_KEY` 环境变量或项目根目录的 `.env.jev.json`；已有配置时无需重复填写，换电脑后需自行配置。

模型按钮自动管理服务和客户端。等待决策时仿真暂停，收到动作后继续执行；切换模型会保存上一轮轨迹并重置任务。

**当前 Kev / Laya 已接入控制链路，但尚未通过导航评估，可能出现倒退、转圈或任务未完成。** 具体结果见[模型统一对照报告](docs/模型统一对照报告.md)。

## 轨迹与详细说明

Editor 的轨迹数据保存在 `Logs/trajectories/`，模型按钮的请求与回答保存在 `Logs/model-switch/`。生成 PNG 对比图需要配置 Python / Matplotlib，步骤见使用指南。

- [Kev 与 Laya 使用指南](docs/Kev与Laya使用指南.md)：安装部署、手动启动、迁移及故障排查。
- [模型按钮切换说明](docs/模型按钮切换.md)：按钮行为、参数与日志。
- [Jev API 与控制协议](docs/bian/当前控制与Jev协议.md)：请求格式、动作与 UDP 通信。
