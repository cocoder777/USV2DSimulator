# 固定路径测试任务：算法设计者交付说明

算法设计者优先读取本目录的 **`algorithm_interface.json`**：它集中整理了目标路线、模型参考参数、全部实时状态字段、派生速度、控制输出及通信约定。建议将整个 `TestData` 文件夹一并交付。

## 统一资料入口

| JSON 字典 | 包含什么 | 数据如何获取 |
| --- | --- | --- |
| `route` | 初始状态、坐标约定、五个任务点、401 个参考曲线采样点、建议航速及容差 | 直接读取字典；它是 `fixed_route.json` 的完整副本 |
| `route_field_definitions`、`route_task_rules` | 路线字段的类型、单位、通过顺序和完成条件 | 静态任务约定，尚未由 Unity 执行打卡统计 |
| `model_parameters` | 九项动力学参数和固定步长，含参考值、单位、含义及配置来源 | 静态参考，不经 UDP 实时发送 |
| `state_fields` | 全部 19 个 UDP 状态字段的定义 | 运行 Unity 后从 `state` 消息读取实际值 |
| `derived_fields` | 世界坐标速度 `vx`、`vy` 的公式及输入字段 | 算法根据实时速度及航向自行计算，UDP 没有这两个字段 |
| `control_fields`、`control_example` | 算法输出的 7 个控制字段及示例 | 算法计算后通过 UDP 发给 Unity |
| `communication` | 地址端口、协议版本、状态频率、超时和乱序处理 | 静态通信配置及协议约定 |

每项字段定义包含 `type`、`unit`、`description`、`access`；无物理单位时 `unit` 为 `null`，`access_definitions` 解释获取方式。参数项另有 `value` 和 `source`；`state_fields` 只有定义，不包含伪装成实时数据的示例数值。

**该 JSON 是接口资料及配置快照，不是实时状态文件。** 实时位置、速度和航向仍由 UDP 提供；运行时参数面板的改动不会自动写入此文件。`fixed_route.json` 保持为 Unity 绘图的路线源，路线、模型参数、固定步长或通信配置变化后，须同步更新统一资料中的对应内容。统一资料自身的 `document_version`、路线的 `schema_version` 和 UDP 的 `version` 是相互独立的版本。

Python 读取示例（在项目根目录运行）：

```python
import json
import math
from pathlib import Path

info = json.loads(Path("TestData/algorithm_interface.json").read_text(encoding="utf-8"))
route = info["route"]
parameters = info["model_parameters"]
state_fields = info["state_fields"]
derived_fields = info["derived_fields"]
control_fields = info["control_fields"]
communication = info["communication"]
waypoints = route["waypoints"]
reference_samples = route["path"]["samples"]  # 算法跟踪曲线，不能只直连五个任务点
max_speed_reference = parameters["maxForwardSpeed"]["value"]

# state 应来自实际收到的 UDP state 消息，而不是本资料文件。
def world_velocity(state):
    heading_rad = math.radians(state["heading_deg"])
    return (
        state["speed"] * math.cos(heading_rad),
        state["speed"] * math.sin(heading_rad),
    )
```

注意：`speed` 是沿船头方向的带符号速度，不是世界 X 方向速度；模型没有独立横向漂移速度状态。`actual_acceleration` 和 `actual_angular_acceleration_deg` 是未扣除阻尼的执行器输出。算法必须输出线加速度和角加速度，不能把目标速度或目标角速度直接填入控制字段。

Unity Main 场景现已配置路线显示组件，在 Editor 的 Play Mode 中读取 `fixed_route.json` 并绘制路线。现有 Python 三阶段测试客户端不会自动跟踪路线；路线跟踪控制及通过统计需由算法程序实现，或后续另行接入平台。

### 在 Unity 中查看

打开 `Assets/Scenes/Main.unity` 并进入 Play Mode。场景根对象 `TargetRoute` 会从项目根目录的 `TestData/fixed_route.json` 读取目标路径，用 401 个采样点显示黄色平滑参考曲线，以及绿色起点圆环、橙色中间点圆环、红色终点圆环和 S、P1、P2、P3、E 文字标签。实际运动轨迹仍为原有青色。

保留现有跟随镜头，因此一次只能看到路线局部，后续点会随艇移动进入视野；没有新增全览镜头。R 重置和控制模式切换不会删除路线，圆环颜色表示点类型，不表示是否已打卡。本阶段未实现打卡统计。

修改任务点后先运行下方生成命令，再重新进入 Play Mode 更新显示。若文件缺失或数据无效，Console 会给出明确错误，且不绘制不完整路线；艇控制不受影响。当前仅支持 Editor 从项目根目录加载，独立构建运行时会提示尚不支持此加载方式。

## 1. 路线与完成条件

路线 ID 为 `usv_fixed_curve_001`，五个任务点保持原位，按照 JSON 数组顺序执行：

| ID | role | X（m） | Y（m） |
| --- | --- | ---: | ---: |
| S | start（起点） | 0 | 0 |
| P1 | checkpoint（打卡点） | 10 | 0 |
| P2 | checkpoint（打卡点） | 20 | 5 |
| P3 | checkpoint（打卡点） | 30 | 5 |
| E | end（终点） | 40 | 0 |

`waypoints` 定义起点、必过点和终点，`path.samples` 定义算法应跟踪的参考曲线。不要将五个任务点直接相连代替参考曲线。

每两个任务点之间使用五次平滑插值：`t=(x-x0)/(x1-x0)`，`y=y0+(y1-y0)*(10*t^3-15*t^4+6*t^5)`。节点处斜率及二阶导数均为 0，切线沿 +X、曲率为 0，整条路线 C2 连续；原有水平段仍为直线，升降段平滑过渡，精确穿过五个任务点。此形式限定任务点 X 严格递增，不是通用任意二维曲线。

沿 X 每 0.1 m 采样（不是等弧长），得到 401 个点；所有任务点均精确包含在内。Unity 和算法使用同一个数组，相邻采样点连线仅是平滑曲线的细分近似，不再存在原先任务点之间的大拐角。

修改 `fixed_route.json` 的 `waypoints` 后，在项目根目录运行：

```powershell
python TestData/generate_fixed_curve.py
python TestData/generate_fixed_curve.py --check
```

生成器保留初始状态、巡航速度及容差，重新生成 `path`，并同步 `algorithm_interface.json` 的路线副本和曲线字段说明。`--check` 只验证，不写入文件；源路线和统一资料有任何一处未同步时返回非零状态。

- 初始状态：位置 (0, 0)，航向 0°，线速度 0 m/s，角速度 0°/s，与当前 Main 场景中 Boat 的初始状态一致。测试前在 Unity 中按 R 重置，再选择 External 模式，以实际 UDP 状态确认初值。
- 建议巡航速度：1.5 m/s，可按转弯和终点停车需要降低，不要求全程恒速。
- 打卡条件：艇中心到当前待通过点的欧氏距离 **≤ 1.0 m** 即视为通过；按 S → P1 → P2 → P3 → E 顺序累计，不提前计入后续点。起点满足条件后，首个待跟踪目标为 P1。
- 中间点不要求停车；允许平滑转弯，但仍须进入相应打卡半径。
- 完成条件：已依次通过 P1、P2、P3，且同一时刻满足距离 E **≤ 1.0 m**、`abs(speed) ≤ 0.1 m/s`。不要求终点航向，也不要求额外停留时间。
- 未规定严格的横向误差上限或总用时上限；如需评估跟踪精度，可另外记录到参考曲线采样点细分线段的距离和任务耗时。生成此文件不表示路径跟踪已经验证通过。

## 2. JSON 字段与坐标

| 字段 | 含义 |
| --- | --- |
| `schema_version` | 路线文件格式版本，当前为 2；Unity 兼容版本 1 的任务点折线，与 UDP 协议版本独立 |
| `route_id` | 固定测试任务标识 |
| `coordinate_system` | 坐标系、位置单位、航向单位及正方向 |
| `initial_state` | 约定初始位置、航向、速度和角速度 |
| `cruise_speed_mps` | 建议巡航速度，m/s |
| `arrival_radius_m` | 起点、打卡点及终点统一到达半径，m |
| `stop_speed_mps` | 终点停车速度容差，比较速度绝对值，m/s |
| `waypoints` | 有序路径点；每点包含唯一 `id`、`role`、`x`、`y` |
| `path.type` | sampled_curve，表示参考路线由采样点定义 |
| `path.interpolation` | piecewise_quintic_smoothstep，分段五次平滑插值 |
| `path.sampling_step_x_m` | 沿 X 方向的最大采样步长，当前为 0.1 m |
| `path.samples` | 有序参考曲线采样点，每点为 `x`、`y`（m）；用于路径跟踪，不是额外必过任务点 |

位置采用 Unity 世界 XY 平面，1 个 Unity 单位视为 1 米；+X 向右、+Y 向上。航向 0° 朝 +X，90° 朝 +Y，逆时针为正。航向误差计算时应归一化为最短角度差，例如 [-180°, 180°)。这不是经纬度或北向零度的导航坐标。

`initial_state.speed` 单位为 m/s，`heading_deg` 为度，`yaw_rate_deg` 为 °/s；负速度表示后退。路线文件仅作为算法输入，不会自动改变 Unity 的初始状态或控制模式。

Python 标准库读取示例（在项目根目录运行）：

```python
import json
from pathlib import Path

route_path = Path("TestData/fixed_route.json")
route = json.loads(route_path.read_text(encoding="utf-8"))
assert route["schema_version"] == 2
initial_state = route["initial_state"]
waypoints = route["waypoints"]
reference_samples = route["path"]["samples"]
cruise_speed = route["cruise_speed_mps"]
arrival_radius = route["arrival_radius_m"]
stop_speed = route["stop_speed_mps"]
for point in waypoints:
    print(point["id"], point["role"], point["x"], point["y"])
print("Reference curve samples:", len(reference_samples))
```

## 3. 当前模型参数参考

以下为交付时核对的 `Assets/Scenes/Main.unity` 中 Boat 参数及 `ProjectSettings/TimeManager.asset` 配置，不是由路线文件设置的值。运行期间若通过参数面板或 Inspector 调整，需要同步告知算法设计者；当前 UDP 没有输出这些模型参数。

| Unity 参数 | 值 | 单位或含义 |
| --- | ---: | --- |
| `maxForwardSpeed` | 5.0 | m/s，前进速度上限 |
| `maxReverseSpeed` | 1.0 | m/s，后退速度绝对值上限 |
| `maxAcceleration` | 1.5 | m/s²，线加速度指令绝对值上限 |
| `maxAngularAccelerationDeg` | 60.0 | °/s²，角加速度指令绝对值上限 |
| `maxYawRateDeg` | 45.0 | °/s，角速度绝对值上限 |
| `accelerationTimeConstant` | 0.25 | s，线加速度一阶响应时间常数 |
| `angularAccelerationTimeConstant` | 0.25 | s，角加速度一阶响应时间常数 |
| `linearDamping` | 0.2 | 1/s，线速度阻尼系数 |
| `angularDamping` | 1.2 | 1/s，角速度阻尼系数 |
| `Fixed Timestep` | 0.02 | s，物理更新频率为 50 Hz |

模型是简化二维运动模型，未包含严格船舶水动力学、横向漂移、风浪或障碍物。加速度指令先经过一阶滞后，再扣除速度阻尼后积分；因此零加速度指令不会保持恒速，也不会立即停车。状态中的 `actual_acceleration`、`actual_angular_acceleration_deg` 是执行器滞后输出，尚未扣除阻尼，不等于净运动加速度。

按当前 1.5 m/s 恒速沿解析曲线行驶估算，峰值角速度约 21.19°/s，净角加速度约 38.67°/s²，补偿当前角阻尼后的角加速度指令约 40.60°/s²，低于当前 45°/s 和 60°/s² 的对应限制。该估算不包含执行器一阶滞后及实际起停变速影响，不是跟踪成功或所有动力学约束满足的证明；算法仍应合理减速并进行仿真验证。

数据与曲线几何测试命令：`python -m unittest discover -s TestData -p test_generate_fixed_curve.py -v`。检查确定性、五点包含、节点切线/曲率连续、公式和当前理想角运动需求；实际 Unity 视觉与闭环跟踪需另行验收。

## 4. 与当前 UDP 接口配合

| 接收方 | 本机固定地址 | 内容 |
| --- | --- | --- |
| Unity | `127.0.0.1:5005` | 算法控制指令 |
| 算法程序 | `127.0.0.1:5006` | Unity 艇状态，默认 10 Hz |

算法绑定 5006 端口接收状态，并使用同一个 UDP 套接字向 5005 端口发送控制；不能同时运行另一个占用 5006 的测试客户端。Unity 只接受配置的来源地址和端口。

每个数据报为一条完整 UTF-8 JSON，最大 8192 字节，不使用 TCP 换行分帧。路线 JSON 由算法直接从文件读取，本次不通过 UDP 发送路线，也不修改现有 UDP v2 协议。

算法从 `type=state`、`version=2` 的消息中读取：

- 运动状态：`x`、`y`、`speed`、`heading_deg`、`yaw_rate_deg`。
- 指令和执行器输出：`acceleration`、`angular_acceleration_deg`、`actual_acceleration`、`actual_angular_acceleration_deg`。
- 运行标识：`run_id`、`control_epoch`、`state_sequence`、`mode`、`simulation_time`。
- 控制状态：`control_active`、`command_timed_out`、`command_age_seconds`。

输出必须是线加速度（m/s²）和角加速度（°/s²），不能直接发送目标速度或目标角速度。例如：

```json
{"type":"control","version":2,"run_id":"最新状态中的run_id","control_epoch":"最新状态中的control_epoch","sequence":1,"acceleration":0.3,"angular_acceleration_deg":0.0}
```

上述数值仅演示消息格式，不是完整路线控制策略。所有控制字段必填，数值必须有限，控制量会由 Unity 按当前限制夹紧。

- 建议每收到一条新的有效状态计算并发送一次最新指令，约 10 Hz；丢弃重复及倒序的 `state_sequence`。
- `sequence` 在当前 `control_epoch` 内递增；epoch 变化后可从 1 重新开始，使用最新状态中的 `run_id` 和 `control_epoch`。
- 仅 External 模式执行外部指令。模式切换、R 重置和控制超时会使旧控制标识失效，算法需要相应重置任务进度或控制器内部状态；R 重置后从起点重新进行本测试。
- 默认 0.5 秒无有效控制指令，Unity 输入归零并更新控制周期；旧周期报文不能恢复控制。UDP 无连接、无重传保障，不能依赖最终停止指令必达。
- 判断任务完成后，在停止算法前发送零控制指令。即使退出，仿真仍会按惯性及阻尼继续演化；完成条件应以观测到的位置和速度判定。

完整运行说明见项目根目录的 `算法接入使用说明.md`。目前实际 Unity UDP 闭环及本路线跟踪仍待运行验收；本文件只交付测试数据和接口约定。
