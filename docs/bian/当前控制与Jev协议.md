# 当前 Jev 控制与协议

Unity 采样并暂停 → 发送任务和统一态势 → Python 汇齐本次采样 → Jev 从 64 个 Choice 选航速/航向 → Unity 平滑执行固定时长 → 再采样。地图虚线只显示和保存在轨迹中，不进入模型输入。

## 甲方协议与通道

业务字段以 [对接单艇决策服务](../甲方接口/对接单艇决策服务.md) 为准，二进制布局、缩放和特殊值同时依据同目录四份 Excel；实现参考独立程序 `jev_api_guoliang/server/decision_service.py`。

| 方向 | 标识 | 内容 | 传输 |
| --- | --- | --- | --- |
| Unity → Python | `0x0660` | `simId/id/code/params` 算法任务，附 `unity_only` | UDP 点播 |
| Unity → Python | `0x0060` | 本艇和全部障碍物，各一帧兵力融合态势 | UDP 组播 |
| Unity → Python | `0x0666` | 原任务身份及 Unity 停车采样上下文 | UDP 点播 |
| Python → Unity | `0x0662` | `PlatCode/Speed/Course` 航速航向 | UDP 点播 |
| Python → Unity | `0x0664` | `simId/id/code/status/description/params` | UDP 点播 |

全部多字节整数大端。公共头为 12 字节：`infoUnitSeq:uint8 / infoUnitId:uint8 / infoUnitLength:uint16 / timestamp:uint32 / reserved:uint16 / secondaryInfoUnitId:uint16`。算法 JSON 报文再加 `totalPackets:uint8 / packetIndex:uint8`，组成 14 字节头；首片额外带 `contentLength:uint16`，续片直接接 UTF-8 JSON。首片头后最多 1450 字节（含长度），最多十片，JSON 实际上限 14498 字节。支持乱序、相同重复片；冲突片拒绝，未完成重组超过 3 秒清理。无自动重发。

算法任务、状态和停止的时戳为当天 UTC 起算的 0.1ms；态势和控制为当天北京时间起算的 ms，由本机模拟生成。帧时间戳不是仿真 tick。同一采样的各实体使用共同态势时戳；快速 Retry 时保证快照标记不重复。

默认 Unity 点播端口 `5005`，Python `5006`，态势组播 `239.255.0.10:18010`，组播网卡默认 `127.0.0.1`，启用组播回环。Python 的 `--host/--port` 对应 Unity 回包地址和端口，`--listen-host/--listen-port` 为任务监听端；双方组播地址、端口、网卡须一致。Python 只接收配置的 Unity 源端点。局域网使用时显式配置双方地址和网卡，不沿用 guoliang 的现场动力系统地址。

这是采用甲方帧格式的 Unity 决策验证入口，仍有下面明确标识的本地扩展，不等同于现场系统完整验收。

## 任务、统一态势与 unity_only

任务使用 `code=trackingMode`；`params` 包含 `platCode/planRouteNum/currentRoute/planWaypointNum/planWaypointList`。航路点为 `routeLon/routeLat/routeSpeed`，单位分别为度、度、节。`simId` 对应仿真轮次，`id` 对应本次算法运行，`code` 为算法编码。每次采样沿用任务身份，不把 decision ID 当作任务 ID。

本艇和障碍物均使用兵力融合态势字段：`direction/latitude/longitude/platformId/sign/targetCourse/height/targetSpeed/type/targetTypeConfid/threatLevel/batch`。本艇通过 `params.platCode == platformId` 选取，当前为 1001；障碍物从 2001 起。`type=0` 表示不明，不创造甲方未定义的障碍物枚举；Python 在 Jev 请求中仍明确将其他这些仿真实体描述为障碍物。

`longitude` 精度为 `180/2^31` 度，`latitude` 为 `90/2^31` 度；`targetSpeed` 为实际有符号航速，线值 0.1 节，`-1` 线值为未知，不能当作有效速度；`targetCourse` 为北起顺时针速度向量方向，精度 1°，`65535` 解码为 `null`。`direction` 是目标相对发现平台的方位，不能替代运动方向。静止不提供额外艏向。置信度和威胁等级的 `255` 解码为 `null`。

Unity 世界 +X 东、+Y 北，1 单位 1 米；经纬度基点为 120.5°E、30.2°N。线侧量化真实生效，不通过扩展绕开。负速度恰好量化成保留线值 -1 时采用邻近有效反向线值 -2。

`unity_only` 是本项目新增的 JSON 扩展对象，不是新报文标识，也不是甲方标准字段，不直接作为模型状态发送。它容纳：

- `session_id/episode_id/run_id/decision_id/tick`：现有运行身份及采样关联。
- `event`：决策请求、停车请求、任务完成、停车完成、仿真碰撞等本地事件。
- `situation_timestamp/situation_platform_ids`：本次采样期望的态势快照清单。
- `planning/action_space/execution`：动作时长、暂停标志、8 档节速度、8 个航向和已知响应时长。
- `next_waypoint_index/arrival_radius_m/stop_speed_knots/collision_radii_m`：Unity 进度、到达圈、停车阈值及按平台关联的碰撞半径。

每次采样组播全部实体，再发送带采样清单的任务/停车帧。Python 可以先收到任意一方，但只在全部对应态势到齐后发起一次决策；3 秒缺失则报告决策失败，留给用户 Retry，不使用上一轮缓存替代。同一任务的新决策编号触发下一次采样处理，重复编号不重复调用 API，不重建任务线程。内部组装为 guoliang 风格的 `simId/id/code/params/situation` 后再适配现有几何与模型请求。

甲方 `routeRadius` 仍是文档中的转弯半径，本轮不处理；guoliang 当前将其用于到达圈是已知实现差异。Unity 继续使用自身 `unity_only.arrival_radius_m`，不把它塞入转弯半径。

## 模型单位与候选

甲方态势、任务和控制对外速度统一为节（任务和控制采用 guoliang 当前三端为节的对接约定）。Unity 地图、轨迹、动力学和 Python 几何计算保持米、m/s。1 节 = 1852/3600 m/s ≈ 0.514444 m/s。

Jev 默认使用节表达当前速度、候选速度、预测末速度、圈内最高速度和停车阈值；固定东/北相对位置、距离、半径、净距仍用米。`state.units` 说明单位和换算关系，速度字段用 `_knots`。模型展示在预测完成后转换，不改变入圈、超限、碰撞风险等事实或布尔判断。`--jev-speed-unit mps` 可仅将 Jev 表达恢复为 m/s，线侧速度继续为节。

HTTP 正文保持 `model/state/questions`，只有一道 action Choice。8 档非负速度占最大速度的 `[0,.05,.1,.2,.35,.5,.75,1]`，8 个航向为 `[0,45,90,135,180,225,270,315]`。Choice 名称、顺序和解码不变，不在本地打分、排序、过滤或代选。模型仍看到当前目标、后续点、当前运动和近似轨迹及同指令两段预览，实际只执行一段。没有参考虚线输入。

预测只依据当前运动速度/方向及已知完整响应时长，缺少实际艏向、角速度和过渡进度；启动和急转可能不准，零速时不伪造测得艏向。导航保持原有按序通过、圈内限速、终点停车、直接接近及尽快完成的要求。

## 障碍物与仿真判定

五张地图最多生成三个半径 1 m 红圈，沿原有横向正弦轨迹运动；实际速度为原轨迹的导数，加入态势航速和运动方向，不改运动轨迹。等待 Jev 时与船一起暂停。船体简化半径 0.5 m。

Python 按当前边缘净距筛选附近全部障碍物，搜索范围仍为 `max(12, 最大速度×(2×动作时长+最大响应时长)+2)` 米，不筛选候选。Jev `state.obstacle_avoidance` 仍提供位置、半径、净距，并新增当前速度和运动方向。候选净距仍冻结障碍物当前位置，不做动态轨迹预测；仅修正原来“方向和速度未知”的文字。避障最高优先级、提前绕行、1 m 额外余量等既有策略保持。

Unity 每物理步以相对运动扫掠判碰，碰撞优先于同一步完成判定；碰撞终止、冻结、保存失败轨迹，Retry 或模式切换不能恢复。碰撞是 Unity 仿真结果，通过 `unity_only` 通知结束本轮，不发送算法失败状态 3。

每点计划速度仍是通过圈内实际速度的上限，不是要求达到的参考速度；圈外不施加下一点上限，已通过但尚未离开的圈继续限速。按物理步扫掠入出圈、插值速度统计圈内最高速度，记录 `passing_max_speeds_mps`。超限只记录，不裁剪指令、不阻止打卡。中间点按序入圈，终点须在圈内且低于地图停车阈值；不要求角速度归零。地图、显示和轨迹单位保持当前定义。

## 控制与状态

Jev 返回的目标速度/航向编码为 `0x0662`，平台编码 uint32，速度/航向 uint16、精度 0.01。Unity 执行实际解码后的控制值，不用 JSON 中未量化值替代；最大速度校验容纳半个控制量化步长。

`0x0662` 没有决策编号，配套 `0x0664.params.unity_only` 提供该控制帧时间戳及原运行/决策身份、tick、动作时长。接收顺序不限，Unity 两者配齐后才应用，并继续拒绝重复、过期和其他运行的结果。

状态定义：1 执行中，2 任务/停车完成，3 决策失败（API、响应解析、动作解码或采样数据等失败）。Unity 现有状态文本显示算法状态及描述。一次动作返回不是任务完成；完成由 Unity 实际判定后通知 Python，再返回状态 2。达到控制次数上限不作为完成条件。碰撞不映射为状态 3。

停车使用 `0x0666`，标准身份之外附 `unity_only` 停车快照，复用当前 Jev immediate_stop 请求及 64 Choice。减速到 Unity 的实际零速阈值后保持 Stopped，不继续航线。与 guoliang 直接持续下发零速的实现有意保留差异。

Unity 将节转换为 m/s，以 `90-course` 转内部角度；零速保持接收时的艏向制动。速度固定响应时长匀加速，航向三次 Hermite 过渡，相同目标继续原过渡。默认两个响应时长各 0.5 秒。

## 实现与记录

`ClientProtocol.cs` 与 `tools/client_protocol.py` 负责二进制和分片；`BoatUdpBridge.cs` 与 `boat_client.py` 负责通道；`ObservationBuilder.cs` 生成统一态势与扩展，Python `UnitySnapshotAdapter` 汇齐并适配观测。`choice_control.py`、`tools/obstacle_geometry.py` 和 `tools/model_units.py` 负责模型请求和单位展示。

不增加通信 version。地图和轨迹的 schema_version 继续保留。JSONL 记录任务、原始观测、完整模型 payload、响应、结果和计时，日志不含 Authorization。请求、回包各发一次，不自动重试。实验结果追加到既有实验记录，不以接口联通宣称模型导航或现场系统验收成功。
