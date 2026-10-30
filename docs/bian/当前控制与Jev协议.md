# 当前控制与 Jev 协议

本文描述主项目 **Unity ↔ `jev_api/` ↔ 外部 Jev 服务** 的通信约定。业务要求参照[甲方单艇决策服务说明](../甲方接口/对接单艇决策服务.md)，二进制布局核对同目录的任务、状态、兵力融合态势、航速航向四份 Excel；当前实现依据为 `ClientProtocol.cs`、`BoatUdpBridge.cs`、`ObservationBuilder.cs` 和 `jev_api/protocol/`。

启动和游戏操作见[项目 README](../../README.md)，代码分工、候选动作和各任务决策方法见[Python 决策层说明](../../jev_api/README.md)。`jev_api_guoliang/` 是独立接入程序，不是本地链路的一环；下文“当前实现”默认指主项目。

## 1. 通道与交互

| 方向 | 标识 | 内容 | 通道 |
| --- | --- | --- | --- |
| Unity → Python | `0x0660` | 算法任务：`simId/id/code/params`，附采样扩展 | UDP 点播 |
| Unity → Python | `0x0060` | 每个实体一帧态势：本艇、指定目标及普通障碍物 | UDP 组播 |
| Unity → Python | `0x0666` | 停止算法，附新的停车采样扩展 | UDP 点播 |
| Python → Unity | `0x0662` | 目标航速、目标航向控制 | UDP 点播 |
| Python → Unity | `0x0664` | 算法状态及业务明细 | UDP 点播 |
| Python ↔ Jev | — | `model/state/questions` 请求与 Choice 答案 | HTTPS JSON |

一轮为：Unity 采样并暂停 → 发送全部态势及任务 → Python 汇齐同次快照并校验 → 回复执行中 → 构造模型请求 → Jev 选择动作 → Python 发控制和关联状态 → Unity 配齐并执行 → 再次采样。UDP 到达顺序不作为正确性前提。

默认动作 1 秒仿真时间，物理步长 0.02 秒，即每动作 50 步。等待决策期间本艇、目标和障碍物共同暂停，墙钟间隔包含模型耗时，**不是实时 1 Hz 心跳**。状态在处理、控制结果、完成或失败时发送，一轮可有多条。

### 默认网络配置

| 用途 | 默认值 | Python 参数 / Unity Bridge 字段 |
| --- | --- | --- |
| Unity 收控制/状态，同时作为发包源端口 | `127.0.0.1:5005` | `--host/--port` / `port` |
| Python 收任务/停止 | `127.0.0.1:5006` | `--listen-host/--listen-port` / `targetAddress/targetPort` |
| 态势组播 | `239.255.0.10:18010` | `--situation-group/--situation-port` / `situationGroup/situationPort` |
| 组播网卡 | `127.0.0.1` | `--multicast-interface` / `multicastInterface` |

Unity 组播回环开启，TTL 为 1，点播与组播共用发包 socket。Python 只接受配置的 Unity 源 IP **和端口**；Unity 也只接受配置的 Python 源端点。跨机时需要同时调整地址、监听和网卡。

甲方行为树算法通道、动力控制通道和态势组播在部署上相互独立。主客户端将控制与状态发回同一个 Unity 端点，不能直接套用为甲方现场的分离端点配置。

## 2. 二进制帧

多字节整数使用**大端**，不使用裸 JSON、HEX 文本或额外 `version` 字段。

### 2.1 公共头：12 字节

| 偏移 | 字段 | 类型 | 当前含义 |
| --- | --- | --- | --- |
| 0 | `infoUnitSeq` | uint8 | 发送填 0 |
| 1 | `infoUnitId` | uint8 | 发送填 0 |
| 2 | `infoUnitLength` | uint16 | 本包总字节数，含头 |
| 4 | `timestamp` | uint32 | 按类别使用下表的时基 |
| 8 | `reserved` | uint16 | 0 |
| 10 | `secondaryInfoUnitId` | uint16 | 报文标识 |

| 报文 | 时戳时基 |
| --- | --- |
| 任务/停止/状态：`0x0660/0x0666/0x0664` | 当天 UTC 00:00 起算，0.1 ms |
| 态势/控制：`0x0060/0x0662` | 当天北京时间（UTC+8）00:00 起算，ms |

时戳不是仿真 tick。同次采样全部实体共用态势时戳；同 tick 的 Retry 也分配新决策 ID 和快照时戳。控制时戳用于本地配对，不以跨日大小比较代替任务身份校验。

### 2.2 算法 JSON 与分片

仅 `0x0660/0x0666/0x0664` 在公共头后增加 `totalPackets:uint8` 和 `packetIndex:uint8`，因此头长 14 字节。

```text
首片：[14B 头][contentLength:uint16][UTF-8 JSON 片段]
续片：[14B 头][UTF-8 JSON 片段]
```

- 总片数 1～10，片号从 1 起；同消息各片共用类型、时戳和总片数。
- 14 字节头之后的体最多 1450 字节，首片两字节长度也计入预算。
- `contentLength` 是整段 JSON 的 UTF-8 **字节数**；受十片预算约束，实际上限 **14498 字节**。
- 按来源、类型、时戳和总片数重组。允许乱序和相同重复片，冲突片或长度不符拒绝，未齐片 3 秒过期。
- JSON 必须是对象，发送端拒绝 NaN/Infinity。分片不提供可靠传输、确认重发或缺片补发。

### 2.3 态势 `0x0060`：40 字节

公共头 12 字节，体布局 `>hiiHHHhhHBBI`，字段顺序如下。

| 字段 | 类型 | 单位、特殊值与用途 |
| --- | --- | --- |
| `direction` | int16 | 目标相对发现平台的方位，北起顺时针，1°；不是运动方向 |
| `latitude` | int32 | 度；线值乘 `90/2^31` |
| `longitude` | int32 | 度；线值乘 `180/2^31`，东经为正 |
| `platformId` | uint16 | 用任务本艇/目标编码关联 |
| `sign` | uint16 | 0 不明、1 敌、2 我、3 友等；不替代指定目标编码 |
| `targetCourse` | uint16 | 实际速度向量方向，北起顺时针，1°；65535 在当前适配中解为未知 |
| `height` | int16 | 高度，负值为深度；本地填 0，不参与二维决策 |
| `targetSpeed` | int16 | 实际有符号航速，0.1 节；业务范围 -20～200 节，原始线值 -1 为特殊值 |
| `type` | uint16 | 0 不明、1 渔船、2 浮标、3 灯塔；本地统一填 0 |
| `targetTypeConfid` | uint8 | 0～100；255 为无法判断 |
| `threatLevel` | uint8 | 0 高、1 中、2 低、3 不明；255 无效 |
| `batch` | uint32 | 融合批号；本地用平台编码模拟，不作快照序号 |

Excel 将航速线值 -1、航向 65535 描述为“不限制”；当前决策输入将它们视为未知并解为 `None`。本艇速度未知、目标速度未知或运动目标方向未知时拒绝观测，不能当作有效零速/零方向。静止时无运动方向，本艇艏向由扩展提供，伴随目标另有静止参考方向。

本地本艇编码 1001、普通障碍物 2001 起、指定目标 3001。本艇和伴随目标 `sign=2`，距离目标及普通障碍物为 0；“敌方跟随”是任务用途，当前距离游戏并未将目标标成敌方。目标按任务平台编码选择。普通障碍物在模型中明确描述为障碍物，不创造甲方 `type` 枚举。

Unity 实际量化经纬度、0.1 节速度和 1°方向；微小负速度若量化到保留线值 -1，就改用 -2。扩展不另发精确位置/线速度来绕过量化。

### 2.4 控制 `0x0662`：20 字节

公共头 12 字节，体 `>IHH`：

| 字段 | 类型 | 解释 |
| --- | --- | --- |
| `PlatCode` | uint32 | 4 字节平台编码；任务及态势平台字段仍为 uint16 |
| `Speed` | uint16 | 目标航速，当前按 0.01 节解码 |
| `Course` | uint16 | 目标绝对航向，0.01°，北起顺时针，范围 [0,360) |

Excel 控制字段名大写开头，JSON 业务对象用 `platCode/speed/course`。本地发送非负航速，Unity 执行解码后的量化控制；时长取匹配请求，状态不重复发送未量化动作。

## 3. 算法 JSON 公共结构

### 任务、停止与状态

```json
{"simId":"episode","id":"run","code":"trackingMode","params":{}}
```

这里只示意外壳，实际任务参数见下一节。`simId + id` 关联任务，`code` 是算法编码。甲方行为树中 `id` 为节点 ID；本地 `simId=EpisodeId`、`id=RunId`，重置后换身份。没有正式 `requestId`。

停止 `0x0666` 正式正文只有 `simId/id/code`，不带 `params`。本地另带本轮 `unity_only`；Python 按身份恢复已接收的任务配置，找不到上下文就失败。

状态 `0x0664`：

```json
{
  "simId": "episode",
  "id": "run",
  "code": "trackingMode",
  "status": 1,
  "description": "tracking",
  "params": {
    "currentRoute": 0,
    "unity_only": {
      "decision_id": "decision",
      "tick": 0,
      "control_timestamp": 123456
    }
  }
}
```

| 状态 | 含义 |
| --- | --- |
| 0 | 发送方本地“已下发、未收到回复”，不是对端状态帧取值 |
| 1 | 执行中：包括 processing、动作结果和持续跟随/伴随 |
| 2 | 完成：本地由 Unity 判定任务/停车完成并通知后发送 |
| 3 | 决策失败：无效输入、API、答案解码或控制编码等错误 |

一次动作不是任务完成。实体碰撞属于 Unity 仿真失败，单独冻结、保存轨迹并通知 Python，不转换成算法状态 3。

## 4. 四个任务的业务接口

四个任务共用上述通道和控制帧，差别在编码、参数和状态明细。以下同时列出**甲方要求**与**当前实现**。

### 4.1 航线任务：`trackingMode`

甲方要求按给定航路点跟踪。调用 `params`：

| 字段 | JSON 值类型 | 说明 |
| --- | --- | --- |
| `platCode` | 整数（uint16 范围） | 本艇编码 |
| `planRouteNum` | 整数（uint16 范围） | 计划航线编号，本地为地图级别 |
| `currentRoute` | 整数（uint16 范围） | 当前实现为已完成目标点数，0 指向第一目标 |
| `planWaypointNum` | 整数（uint16 范围） | 必须等于列表长度 |
| `planWaypointList` | 对象数组 | 只含目标点，不含 Unity 地图起点 |
| 点内 `routeLon/routeLat` | 数值 | 实际经纬度，单位度 |
| 点内 `routeSpeed` | 数值 | 当前按节传输，本地为圈内实际通过速度上限 |
| 点内 `routeRadius` | 数值 | 甲方定义为转弯半径（米）；Unity 当前不生成、不使用 |

```json
{
  "simId": "episode", "id": "run", "code": "trackingMode",
  "params": {
    "platCode": 1001, "planRouteNum": 0, "currentRoute": 0,
    "planWaypointNum": 2,
    "planWaypointList": [
      {"routeLon": 120.501, "routeLat": 30.201, "routeSpeed": 12},
      {"routeLon": 120.502, "routeLat": 30.202, "routeSpeed": 6}
    ]
  }
}
```

此例省略本地必需的采样扩展。执行请求要求 `0 <= currentRoute < planWaypointNum`；完成通知可等于总点数。下一目标下标就是 `currentRoute`，不能再减一。

甲方说明列出 `nextWaypointIndex/nextWaypoint/remainingWaypointNum/remainingDistanceM/estimatedRemainingTimeS/completionRate` 等状态明细；主客户端目前仅回 `currentRoute` 和本地关联扩展，**尚未提供完整甲方航线进度明细**。独立 `jev_api_guoliang/` 的进度字段较全，其完成率为 0～100，不代表主客户端也支持。

本地使用 `unity_only.arrival_radius_m` 判定打卡，终点还须低于停车阈值，不使用 `routeRadius` 作为到达圈或转弯约束。甲方“计划航速”与本地“圈内通过上限”是正式接入时需确认的业务差别。

### 4.2 敌方跟随：`followMode`

甲方要求沿敌方/不明目标航迹尾随，默认在后方保持间距。正式 `params`：

| 字段 | JSON 值类型 | 说明 |
| --- | --- | --- |
| `platCode/targetPlatCode` | 整数（uint16 范围） | 本艇与目标，不传瞬时经纬度代替目标身份 |
| `followDistanceMinM` | 数值 | 间距下限，米，≥0 |
| `followDistanceMaxM` | 数值 | 上限，米，>0，且不小于下限 |
| `maxSpeed` | 数值 | 本艇航速上限，当前速度口径为节；0 表示无附加任务限制 |

甲方期望状态为 `targetPlatCode/relativeDistanceM/relativeBearingDeg/inRange/distanceOutM/targetSpeed/targetCourse`。相对方位以目标艏向为基准，0°正前、180°正后；越界量在距为 0、过近为负、过远为正。入距仍保持状态 1。甲方说明同时使用“沿航迹尾随”和“水平间距”，弯道中沿轨间距与直线距离的验收口径还需明确。

主客户端使用正式编码和参数：

```json
{
  "simId": "episode", "id": "run", "code": "followMode",
  "params": {
    "platCode": 1001, "targetPlatCode": 3001,
    "followDistanceMinM": 100, "followDistanceMaxM": 200, "maxSpeed": 0
  }
}
```

本地按用户指定论文的后方扇环定义执行：水平中心距在闭区间内，相对目标参考方向的方位在 90°～270° 内，区域开角 180°。参数校验为 `0 <= min <= max`、`max > 0`，最大距离须超过两艇实体半径之和；实体碰撞约束始终独立保留。窄带及低限速可能难以稳定保持，不自动扩大范围。`maxSpeed=0` 使用物理上限，否则取二者较小值生成 128 个候选。

执行请求根据当前完整快照分为保持/小幅调整与丢失追回。正式扇环内或附近为保持，保护距离内及明显离区为追回；附近进入/离开阈值为可用径向带宽的10%/20%，任务重置清空迟滞；明确丢失后须进入内部区域再切回保持。保持沿用V4软尾点跟踪：目标当前方向后方距离中点、带宽5%且最多5米容差，以当前速度平移，渐进位置修正形成参考方向。追回以区域余量为目标；内部径向带在保护半径与最大距离间各留四分之一带宽，角边界按相同物理余量内收，不改变正式合格范围。追回额外提供绕敌艇保护圈到后方内部区域的顺/逆时针切线—圆弧剩余距离，圈内先计径向分离；本地保留两侧几何事实，不选择路径或动作。候选给出动作结束和12倍动作时长的后果：保持为尾点误差/参考偏差，追回为转弯与直航两种假设的区域误差/双侧余距。追回区域与安全预览一致，按观测转动预测、加速度最多延续2秒；更长预览及目标转弯外推另用于实体安全。敌艇和障碍物统一汇总实体/保护圈净距，敌艇保护半径取最小跟随距离与联合实体半径的较大值，障碍物使用其避障半径；保留动作与长预览最小值及端点。几何余距不含其他障碍及转向惯性，须结合安全事实判断。Jev选择全部128候选之一，本地不排序、过滤或覆盖。目标转速/加速度从完整快照与物理tick估计，加速度最多外推2秒；停车冻结最后可靠方向，开局静止需参考方向。`inRange`仍仅表示距离合格，正式后方扇环由Unity独立统计。STOP独立构造并保留任务身份、限速和碰撞事实。

当前以运动方向暂代目标艏向，不处理侧滑；不逐点复刻目标历史轨迹，不采用沿轨弧长。论文扇环保距与甲方严格路径型尾随的验收差异需明确保留。旧 `targetFollowing` 不静默映射为正式任务，旧实验日志仍按距离保持解释。场景到时结束是本地实验终态，不是甲方持续尾随自然完成。

### 4.3 友方伴随：`accompanyMode`

甲方要求在目标体坐标的纵横带内保位，支持稳定后的航向同步。当前已实现同名编码和正式业务字段：

| 字段 | JSON 值类型 | 说明 |
| --- | --- | --- |
| `platCode/targetPlatCode` | 整数（uint16 范围） | 本艇与目标，不能相同 |
| `offsetAlongMinM/offsetAlongMaxM` | 数值 | 纵向闭区间，米；前正后负 |
| `offsetAcrossMinM/offsetAcrossMaxM` | 数值 | 横向闭区间，米；右舷正、左舷负 |
| `syncHeading` | 整数 0 或 1 | 0 仅保位，1 稳定后同步航向 |
| `maxSpeed` | 非负数值 | 节；0 使用物理上限，否则取任务与物理上限较小值 |

```json
{
  "simId": "episode", "id": "run", "code": "accompanyMode",
  "params": {
    "platCode": 1001, "targetPlatCode": 3001,
    "offsetAlongMinM": -160, "offsetAlongMaxM": -140,
    "offsetAcrossMinM": 70, "offsetAcrossMaxM": 90,
    "syncHeading": 1, "maxSpeed": 0
  }
}
```

两轴须有限且 min≤max，整个允许矩形不能与两艇合并实体包络重叠。当前快照只能含本艇和指定友艇，不支持普通障碍物。

状态回 `targetPlatCode/currentAlongM/currentAcrossM/alongOutM/acrossOutM/inPosition/targetSpeed/targetCourse`。纵横越界量带方向符号，带内为 0；两轴均在原闭区间内时 `inPosition=1`，仍保持状态 1。处理和动作结果取本轮观测，结束通知可携带 Unity 实际末态统计。

体坐标参考以态势运动方向暂代目标艏向，静止时用 `unity_only.target_reference_course_deg`；真实艏向与运动方向不一致时不能沿用此假设。同步策略先保位，阶段见决策层说明。实验到时或实际停车后发送本地完成状态，不表示通过长期稳定伴随验收。

### 4.4 特殊目标追踪：`SpecialTargetTrackMode`

正式参数为 `platCode/targetBatch/expectedAngleDeg/expectedSpeed/expectedDistance`。本地平台编码、控制编码及位宽均沿用现有实现，不为新任务改动。以当前完整快照的 `batch` 匹配目标，再用其 `platformId` 关联实体半径；不存在或不唯一则拒绝。本地目标 `platformId=3001`、`batch=30015`。

`expectedAngleDeg` 是目标船头起算、顺时针增加的位置方位：0° 前、90° 右、180° 后、270° 左。距离为中心距（米），速度为本艇实际航速（节）。目标艏向等于运动方向，停车仍用 `unity_only.target_reference_course_deg`。指定距离须超过两艇实体半径之和，期望速度不能超过当前物理上限。

新任务仅支持本艇与目标，使用组织接近 / 末段达成两类执行请求及原 128 候选。正常决策回状态 1 和 5.6 规定的九个业务字段；`angleErrorDeg` 仍是位置方位误差，船头指向误差等诊断放 `params.unity_only`，并与 `decision_id/tick/control_timestamp` 合并保留。

当前本地实验暂不定义四项容差：进入期望点周围一个本艇实体半径内即冻结，保留非零航速和四项实际误差，结果为 `position_reached`，不宣称四项严格达标。Unity 发 `mission_completed` 后回状态 2，description 为 `position_reached; inspect terminal errors`，扩展 `four_conditions_accepted=false`。碰撞冻结并走独立仿真失败事件，同一步碰撞优先；场景时长不触发完成。正式四项验收与本地到位终态须区分。

## 5. Unity 采样扩展与完整快照

`unity_only` 是本项目 JSON 扩展，**不是甲方正式报文类型**。主客户端依赖它调度；仅有甲方标准任务和独立实时态势，还不能驱动当前客户端。

任务/停止扩展位于顶层，状态关联扩展位于 `params.unity_only`。扩展先适配成内部观测，不原样全部发送给 Jev。

| 任务/停止扩展字段 | 用途 |
| --- | --- |
| `decision_id:string/tick:int` | 单次采样身份与非负物理 tick；Retry 生成新决策 ID |
| `event:string` | 正常采样、停止或终态事件 |
| `situation_timestamp:uint32` | 本次全部实体共同态势时戳 |
| `collision_radii_m:object` | key 为十进制平台 ID 字符串，value 为正实体半径（米）；也作本轮完整实体清单 |
| `action_duration_s:number` | 本次控制时长，正式控制没有此字段 |
| `max_speed_knots:number` | 本艇物理速度上限，用于生成动作空间 |
| `arrival_radius_m:number` | 只在航线及其 STOP 中提供，独立于转弯半径 |
| `stop_speed_knots:number` | 停车阈值；立即停止使用约等于 0 的独立阈值 |
| `target_reference_course_deg:number` | 伴随静止目标或跟随低速/开局静止目标的参考方向；跟随运动中通常不发送 |
| `execution:object` | 下列运动配置、艏向和保持目标状态 |

`execution` 字段：

- `motion_model="bounded_acceleration"`。
- `max_acceleration_mps2/max_deceleration_mps2/max_yaw_rate_deg_s/max_angular_acceleration_deg_s2`：运动能力。
- `fixed_dt_s/hull_length_m/hull_beam_m`：物理步长、长宽。
- `heading_course_deg/yaw_rate_deg_s`：实际艏向和角速度，北起/顺时针为正，静止仍有效。
- `active_target_course_deg/active_course_error_deg`：保持中的航向目标及未取模角差，可为空，用于转向分支预测。
- `obstacle_avoidance_radii_m`：按普通障碍物 ID 给安全半径，与实体半径分开。

接收规则：

1. Unity 先发各实体态势，再发任务/停止；Python 允许乱序到达。
2. 以快照时戳和半径表中的**全部平台 ID**汇齐；本艇和指定目标也参加屏障。
3. 缺实体不调用模型，超过 3 秒失败；态势缓存同样保留 3 秒，消费后删除，不从上一轮补齐。
4. 同任务重复决策 ID 不重复调用，更旧 tick 拒绝，同 tick 新 Retry 可接纳。新任务撤销旧等待与输出资格，退休身份不重新激活。
5. STOP 只复用任务配置，仍需新态势。Python 重启丢失上下文后直接停车会失败，需要先恢复任务通信。

## 6. 控制配对、结束与失败

`0x0662` 本身不含任务或决策 ID，本地用两层关联避免迟到控制作用于新实验：

- `0x0664.params.unity_only` 回显 `decision_id/tick`；动作结果再附 `control_timestamp`，等于对应控制头时戳。processing 不带控制时戳。
- Unity 等控制与状态配齐，再验证 `simId/id/decision_id/tick`、当前参数和本艇编码，才执行量化控制。旧运行/采样不执行，终态不被迟到执行中状态覆盖。

| 本地事件 | 通信与处理 |
| --- | --- |
| `decision_request` | `0x0660` + 当前采样，正常决策 |
| `stop_request` | `0x0666` + 当前采样，保留任务类型，暂停原目标，交给 Jev 制动 |
| `mission_completed` | 用 `0x0660` 扩展事件通知，Python 回状态 2 并退休任务 |
| `stop_completed` | 实际停稳后通知，回状态 2，Unity 进入 Stopped |
| `simulation_collision` | 通知碰撞，撤销输出并退休任务，不伪造算法完成或状态 3 |
| `simulation_cancelled` | Python 支持的取消事件，当前 Unity 主路径未主动发送 |

终态通知不调用模型，也不要求正常决策的完整扩展。跟随/伴随进入目标带内不完成，其本地实验由场景时长结束。STOP 期间暂停正常完成判定，到实际停稳才结束；实体碰撞始终优先。

API、答案解析、无效观测等错误有可关联身份时回状态 3。没有自动业务重发、丢包重传或本地备用动作，使用者通过 Retry 重新采样。当前两种 HTTP 传输均未设置有限请求超时，调用挂起可能一直等待；快照 3 秒超时不是 Jev 推理超时。

## 7. 单位、执行与模型接口

### 单位和坐标

| 数据 | 当前约定 |
| --- | --- |
| 任务/状态 JSON 速度、态势速度、控制速度 | 节；JSON 为实际数值，二进制按各自精度编码 |
| 任务 JSON 经纬度 | 实际度数，不再乘二进制经纬度比例 |
| 地图、轨迹、内部几何和物理 | 米、m/s |
| 航向/运动方向 | 北起顺时针，0°北、90°东 |
| Unity 世界位置 | +X 东、+Y 北，1 单位=1 米；基点 120.5°E、30.2°N |
| Jev 展示 | 默认速度节、距离米；`--jev-speed-unit mps` 只改模型表达，不改通信 |

`1 节 = 1852/3600 m/s`。Unity 内部 `HeadingDeg` 使用另一角度表示，由 `ObservationBuilder` 转成协议艏向，不要直接当北起航向。`targetSpeed` 是实际速度，控制 `Speed` 是目标速度；`direction` 是方位，不是 `targetCourse`。

### 物理执行

场景及默认配置：前进上限 50 节，加速 1.46 m/s²、减速 1.75 m/s²、角速度 12°/s、角加速度 4°/s²；长 11.6 米、宽 3.4 米。实体半径为半对角线，约 6.044 米，打卡半径默认其 5 倍，约 30.22 米。参数可调并经扩展传给 Python。

航速/航向受加减速和角加减速限制，不能瞬时跳变；相同目标延续转向分支，零速指令不能原地转艏。当前使用固定能力上限；已核对[真实船 CSV](../甲方接口/0-50KN.csv)，固定加速响应及角加速度限制与观测存在差异，尚未完成校准，见[核对记录](../bian_local/Jev仿真实验记录.md#2026-10-29实船数据与-unity-参数核对)。

普通障碍物默认小/大目标安全半径 50/90 米、实体半径 3/12 米；侵入安全圈只记录，实体接触才失败。Unity 按运动中的相对扫掠判碰，碰撞优先于同一步完成。模型对普通障碍物冻结当前位置计算净距，不能据此保证动态避障安全。

### Jev HTTPS 接口

默认 `POST https://api.typesafe.ai/v1/systemone`，模型 `jev-latest`，使用 Bearer 凭据。`--endpoint/--model` 可覆盖默认值；当前入口读取 `boat_client.py` 中的 `API_KEY` 常量。

请求正文为 `model/state/questions`，`questions.action.type="choice"`，`criteria` 为候选 ID 到描述字符串的映射。候选数、速度上限和各任务事实见[决策层各任务章节](../../jev_api/README.md)。客户端期待：

```json
{"answers":{"action":{"type":"choice","choice":"speed_0__north"}}}
```

客户端验证题型和候选 ID，再按本轮动作空间映射航速/航向，不接受任意生成的控制数值。请求不包含目标未来脚本或隐藏整段轨迹。

候选配置按任务选择：航线为 8 档速度 × 8 航向（64 个，45° 间隔），敌方跟随、友方伴随与特殊目标追踪为 8 档速度 × 16 航向（128 个，22.5° 间隔）；STOP 沿用所属任务配置。该差异属于 Python 决策配置，不增加 UDP 字段，Unity 继续执行实际航速/航向控制。

若返回 HTTP 400，终端错误及 JSONL 的 `timings.http_error_body` 会保留最多 4096 字节的服务端原因。例如 `max_tokens_exceeded` 表示输入超出模型预算；带障碍物、恢复阶段和停车题目应单独验证，不能仅以简单初始样本判断请求可用。

默认 `--http-transport persistent`，每线程独立复用 HTTPS 连接并预连接；`urllib` 是兼容路径。预连接不发决策请求，失败也不自动重发业务。默认单 worker，只保留一个最新待执行观测。

JSONL 保存身份、观测、payload、原始响应、动作及计时，不记录 Authorization；状态不承载完整模型响应。`wait_headers_s` 包含网络和服务端等待，不能直接当纯推理时间。

## 8. 甲方要求与接入差异

| 项目 | 甲方说明/定义 | 当前主项目与后续要求 |
| --- | --- | --- |
| 调度 | 行为树异步下发，tick 不阻塞等完成 | Unity 暂停等每次决策；实时接入需处理持续运动、旧指令保持和失效策略 |
| 输入 | 按平台编码读取持续态势 | 依赖扩展、同次时戳及实体清单，标准实时输入不能直接驱动 |
| 回传 | 状态回行为树，控制到动力系统 | 目前统一回 Unity，现场需分离端点 |
| 航线速度/半径 | 计划航速、转弯半径 | 本地圈内上限、独立打卡圈，未实现转弯半径约束 |
| 航线进度 | 下一点、剩余航程/时间、完成率等 | 主客户端仅 currentRoute，需补齐明细 |
| 敌方跟随 | followMode，路径型后方尾随，含限速 | 正式字段及限速已支持；本地采用论文后方 180° 扇环/水平中心距，严格沿航迹语义待确认 |
| 友方伴随 | accompanyMode，体坐标带和航向同步 | 字段已支持；无普通障碍物，运动方向代艏向，静止参考依赖扩展 |
| 持续任务终态 | 在距/到位仍执行，直到停止 | 本地到时结束并报告完成，不能直接作为甲方任务自然终态 |
| 平台能力 | 应反映实际平台 | 当前简化运动，实船校准仍待办 |

甲方业务依据见[甲方说明](../甲方接口/对接单艇决策服务.md)。其跟随要求保留，本地论文扇环定义不等于严格沿航迹验收已通过。改进项见[本地待办](../bian_local/Jev请求优化代办清单.md)。
