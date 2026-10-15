# USV2DSimulator

基于 **Unity Universal 2D** 的简易无人艇（USV）运动仿真平台，提供二维动力学、参数调节、状态和轨迹显示，并通过本地 UDP 接入外部控制程序。项目主要负责仿真环境、数据接口和可视化，正式决策与路径跟踪算法由外部提供。

目前包含固定平滑测试路线和 Python 通信测试客户端。**测试客户端只执行加速、转向、零输入三个动作阶段，不会自动跟踪目标路线。**

## 功能

- **二维运动**：前进、减速、后退和转向，包含加速度限制、速度限制、阻尼及一阶惯性延迟。
- **手动 / 外部控制**：WASD 手动操作与 External 模式切换，共用动力学接口，R 重置艇状态。
- **实时参数面板**：调整速度、加速度、响应时间和阻尼，支持恢复默认参数。
- **状态与轨迹**：HUD 展示位置、速度、航向、角速度、控制指令及通信状态；青色线记录实际历史轨迹。
- **固定目标曲线**：从 JSON 读取 401 个曲线采样点，绘制黄色参考路线及起点、打卡点、终点和编号。
- **本地 UDP v2**：输出实时艇状态、接收加速度控制，包含来源校验、重复和乱序过滤、控制限幅及超时清零。
- **算法交付资料**：统一 JSON 汇总路线、模型参考参数、状态及控制字段、单位、派生量和通信规则。

代码和数据检查已完成，新增功能的 Unity 实际运行验证仍有限，详见[验证状态与限制](#验证状态与限制)。

## 环境

| 项目 | 配置 |
| --- | --- |
| Unity Editor | **6000.6.5f1**，以 `ProjectSettings/ProjectVersion.txt` 为准 |
| 渲染 | Universal Render Pipeline，2D 场景 |
| Unity 脚本 | C# |
| 外部测试程序 | Python，建议 3.10 或更新版本，仅使用标准库 |
| 当前运行场景 | `Assets/Scenes/Main.unity` |
| 当前主要开发环境 | Windows / PowerShell |

项目采用简化二维运动模型，不包含严格船舶水动力学、独立横向漂移、风浪、障碍物、推力或舵角模型。

## 快速开始

### 1. 获取并打开项目

```powershell
git clone https://github.com/cocoder777/USV2DSimulator.git
cd USV2DSimulator
```

在 Unity Hub 中添加这个项目目录，使用 Unity **6000.6.5f1** 打开。首次打开需要等待资源导入和 Packages 依赖解析；全新环境可能需要联网下载依赖。

在 Editor 中明确打开 **`Assets/Scenes/Main.unity`**，再进入 Play Mode。不要依赖当前默认场景或 Build Settings 自动选择 Main。

### 2. 手动控制

默认是 Manual 模式；点击 Game 窗口使键盘输入生效。

| 操作 | 功能 |
| --- | --- |
| W / S | 向前加速 / 减速并可继续后退 |
| A / D | 左转 / 右转 |
| R | 重置位置、航向、速度、执行器状态及实际轨迹 |
| 左下角模式按钮 | 切换 Manual / External |
| 右侧参数面板 | 实时调节动力学参数 |

External 模式下 WASD 不覆盖外部控制，R 仍有效。模式切换清空旧控制输入，但不瞬间清空速度和执行器滞后状态。

### 3. 运行 Python 通信测试

保持 Unity 处于 Play Mode，在项目根目录的另一终端运行：

```powershell
python jev_api/boat_client.py
```

点击游戏窗口左下角按钮切换到 **External**。客户端接收艇状态，依次执行默认各 3 秒的加速、转向和零输入阶段，然后退出。

可选状态日志：

```powershell
python jev_api/boat_client.py --phase-seconds 3 --log Logs/boat_states.jsonl
```

客户端默认 5 秒未收到新的有效状态会退出。若提示端口占用，先关闭其他客户端；若收不到状态，检查 Main 场景、Play Mode、Console 和双方端口。

本流程不需要安装 `jev_api/requirements.txt`，也不调用 Jev API。该目录原有客服 API 示例的说明与当前仿真客户端是不同用途，其示例脚本未纳入仓库。

## 目标路线与坐标

路线源为 [TestData/fixed_route.json](TestData/fixed_route.json)，格式版本为 **2**，路线 ID 为 `usv_fixed_curve_001`。

| 点 | 类型 | X（m） | Y（m） |
| --- | --- | ---: | ---: |
| S | 起点 | 0 | 0 |
| P1 | 中间打卡点 | 10 | 0 |
| P2 | 中间打卡点 | 20 | 5 |
| P3 | 中间打卡点 | 30 | 5 |
| E | 终点 | 40 | 0 |

- 世界 XY 平面，1 个 Unity 单位视为 1 米；0° 朝 +X，90° 朝 +Y，逆时针为正。
- 初始位置为 S，航向、线速度和角速度均为 0；建议巡航速度 1.5 m/s。
- `waypoints` 定义五个任务点；**`path.samples` 定义算法需要跟踪的参考曲线**，不能只直连五个任务点替代曲线。
- 采用分段五次平滑插值，穿过全部任务点，节点处切线和曲率连续；沿 X 每 0.1 m 采样，共 401 点，非等弧长采样。
- 任务约定为按序进入各点 1 m 半径，终点还需同时满足 `abs(speed) <= 0.1 m/s`。中间点不要求停车。

显示颜色：目标曲线黄色，实际轨迹青色，起点绿色，中间点橙色，终点红色。点颜色表示类型，**尚不表示通过状态**。摄像机仅跟随艇显示局部路线，没有全览镜头。

修改 `fixed_route.json` 的任务点后，在项目根目录运行：

```powershell
python TestData/generate_fixed_curve.py
python TestData/generate_fixed_curve.py --check
```

生成器要求任务点 X 严格递增，保留初始状态、巡航速度及容差，重建曲线并同步算法资料中的路线副本。`--check` 只校验，不修改文件。重新进入 Play Mode 后 Unity 加载新数据。

当前路线仅支持 **Unity Editor** 从项目根目录加载，独立构建包内的路线加载尚未实现。

## 决策算法对接

首先阅读 [TestData/README.md](TestData/README.md)，或直接读取统一资料 [TestData/algorithm_interface.json](TestData/algorithm_interface.json)：

| 字典 | 内容 |
| --- | --- |
| `route` | 完整路线副本、初始状态及任务容差 |
| `model_parameters` | 九项动力学参数和固定步长的参考值、单位及来源 |
| `state_fields` | 全部 19 个实时状态字段的类型、单位和含义 |
| `derived_fields` | 世界速度 `vx/vy` 的计算公式 |
| `control_fields`、`control_example` | 算法输出字段及示例 |
| `communication` | UDP 配置、频率、大小限制及超时规则 |
| `route_task_rules`、`route_generation` | 任务完成条件及曲线生成规则 |

这份文件是**静态资料和配置快照**；实时艇状态仍通过 UDP 获取。运行时修改参数不会自动更新资料，当前也不通过 UDP 实时输出动力学参数。配置变化后须同步维护或告知算法方。

### 通信约定

| 接收方 | 默认固定地址 | 内容 |
| --- | --- | --- |
| Unity | `127.0.0.1:5005` | 算法控制指令 |
| 算法程序 | `127.0.0.1:5006` | Unity 状态，默认约 10 Hz |

算法绑定 5006，并使用同一个 UDP 套接字向 5005 发指令。Unity 仅接受配置的来源地址和端口。每个数据报是一条完整 UTF-8 JSON，最大 8192 字节，不使用 TCP 换行分帧。

控制消息示例：

```json
{
  "type": "control",
  "version": 2,
  "run_id": "最新状态中的run_id",
  "control_epoch": "最新状态中的control_epoch",
  "sequence": 1,
  "acceleration": 0.3,
  "angular_acceleration_deg": 0.0
}
```

全部字段必填，数值必须有限。线加速度单位为 m/s²，角加速度为 °/s²，逆时针为正；算法输出的是**加速度，不是目标速度或目标角速度**。

`run_id` 和 `control_epoch` 须来自最新状态，`sequence` 在当前控制周期内严格递增。重复、倒序、旧标识或非法指令不刷新超时。External 下默认 0.5 秒无有效指令会清空输入并更换控制周期，艇继续受惯性和阻尼影响，不会瞬间停车。

`speed` 是沿船头方向的带符号速度，不是世界 X 速度；执行器 `actual_acceleration` 等输出尚未扣除阻尼，不是净运动加速度。

当前 UDP 仅交换状态和控制指令；路线由双方读取文件，不通过 UDP 发送。详细协议和恢复规则见 [算法接入使用说明.md](算法接入使用说明.md)。

## 项目结构

```text
Assets/
  Scenes/Main.unity              主仿真场景
  Scripts/
    BoatDynamics2D.cs            动力学、控制接口、模式与重置
    BoatControlInput.cs          键盘输入与模式按钮
    BoatParameterPanel.cs        实时参数面板
    BoatHUD.cs                   状态与通信显示
    BoatTrail.cs                 实际历史轨迹
    BoatCameraFollow.cs          摄像机跟随
    BoatStateSnapshot.cs         UDP 状态定义
    BoatUdpBridge.cs             UDP 收发、验证和超时处理
    TargetRouteData.cs           路线解析与校验
    TargetRouteDisplay.cs        目标曲线、圆环与标签
  Editor/                       Unity 批处理验证入口
Packages/                       Unity 包依赖及锁定文件
ProjectSettings/                Unity 项目配置
TestData/                       路线、统一算法资料、生成器与测试
jev_api/
  boat_client.py                Python UDP 动作测试客户端
  test_boat_client.py           本地模拟服务测试
Agent.md                        项目背景、进度及开发约定
算法接入使用说明.md              运行与通信细节
```

## 检查与测试

在项目根目录执行：

```powershell
python -m unittest discover -s jev_api -p test_boat_client.py -v
python -m unittest discover -s TestData -p test_generate_fixed_curve.py -v
python TestData/generate_fixed_curve.py --check
```

这些检查分别覆盖 UDP 客户端、曲线几何与数据一致性；Python 模拟服务测试不替代 Unity 真实闭环测试。

另有两个 Unity Editor 批处理入口：

- `BoatBridgeValidation.Run`：检查 UDP 状态/控制、限幅、超时、重置、客户端重启和端口处理。
- `TargetRouteValidation.Run`：检查场景挂载、401 个参考点、五个标签、颜色和资源清理。

可在关闭该项目的 Editor 后使用实际 Unity 安装路径运行，例如：

```powershell
& '<Unity安装目录>/Editor/Unity.exe' -batchmode -nographics -projectPath "$PWD" -executeMethod TargetRouteValidation.Run -logFile "$PWD/Logs/target-route-validation.log"
```

该命令中的安装目录必须替换。检查 UDP 时将入口改为 `BoatBridgeValidation.Run`。批处理成功应同时满足退出码为 0、日志包含对应的 `TARGET_ROUTE_VALIDATION_PASS` 或 `BOAT_BRIDGE_VALIDATION_PASS`；验证不会保存场景。

## 验证状态与限制

已完成 C# 编译器检查、Python UDP 客户端四项测试、曲线六项测试，以及路线解析器 v1/v2 兼容和 25 类非法输入检查。原有基础手动运动、参数面板和跟随功能此前已确认。

**新增功能尚未完成 Unity 实际运行验收。** 当前环境多次无界面启动在验证入口前退出，退出码为 1，原因尚未确定。因此，真实 Unity UDP 闭环、曲线视觉效果、标签遮挡、重置及清理行为仍需在 Editor 中验证。数据和编译检查通过不代表路径跟踪成功。

尚未完成：

- 正式决策/路径跟踪算法接入和实际循迹验收。
- 上层系统路线与必过点的网络接收和任务转发。
- 动力学参数的外部动态读写与 UI 同步。
- 平台内按序打卡统计、终点完成判定和跟踪误差展示。
- 独立构建包内的路线加载。

## 更多资料与版本管理

- [算法设计者资料及路线说明](TestData/README.md)
- [统一算法接口 JSON](TestData/algorithm_interface.json)
- [详细运行与 UDP 说明](算法接入使用说明.md)
- [项目背景和开发进度](Agent.md)
- [早期接入计划（历史 TCP 方案）](算法接入计划.md)：不作为当前协议依据。

仓库保留 `Assets`、`Packages`、`ProjectSettings` 和必要 `.meta`；`Library`、`Temp`、`Logs`、`UserSettings`、Python 缓存及本地凭据等由 `.gitignore` 排除。不要手动删除资源 `.meta` 或把生成缓存、API 密钥加入提交。
