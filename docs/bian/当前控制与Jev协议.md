# 当前 Jev 控制与协议

本文描述 Unity 与主 `jev_api/` 的通信协议。`jev_api_guoliang/` 是面向甲方的独立程序，不是主项目运行依赖。态势输入使用 0x0060，没有独立的 tag 字段或报文类型。

Unity采样并暂停 → 发送任务和全部当前实体 → Python汇齐同次采样 → Jev从64 Choice选目标航速/航向 → Unity平滑执行固定时长 → 再采样。船与障碍物共同暂停，地图虚线不进入模型。

## 适用范围与接口差异

甲方业务定义见[接口说明及同目录四份 Excel](../甲方接口/对接单艇决策服务.md)。Unity 的暂停采样、动态障碍物及执行配置通过下文的扩展字段表达。

| 项目 | 当前约定与差异 |
| --- | --- |
| 帧与单位 | 使用甲方帧标识、字段大小写和二进制布局，速度为节，支持态势组播与算法JSON分片 |
| 航点/进度 | 列表只含目标，不含地图起点；currentRoute为已完成目标数量，0指向第一个目标 |
| 本艇与障碍物 | 共用0x0060，按platCode/platformId选本艇；障碍物type=0，不创造甲方枚举 |
| 任务调度 | guoliang每个有效新任务均接管旧任务，即使身份参数相同；Unity同任务每次采样新decision_id，重复采样不重调模型 |
| 态势缓存 | guoliang按platformId取最新新鲜本艇态势，默认2.5s；Unity按快照时戳和完整实体清单汇齐，3s缺失失败，不跨采样补齐 |
| 执行方式 | guoliang推理期间继续运动、续发旧指令、过保持时限发零速、可恢复错误连续3次失败；Unity暂停等待、执行单动作，失败后手动Retry，无本地代选 |
| 到达与停车 | guoliang从态势推进并主动零速停车；Unity按物理步判定到达，立即停止由Jev决策 |
| routeRadius | 甲方定义为转弯半径，guoliang用于到达圈，0/缺省回退20m；Unity使用独立arrival_radius_m |
| 模型 | guoliang默认Kev-4B，另有Jev/Julia；主项目使用Jev及64个速度/航向候选 |
| 状态详情/输入兼容 | guoliang另报nextWaypointIndex、剩余距离/时间、完成百分比等，Unity只需正式currentRoute及采样关联；主入口只接收正式二进制，不接其调试用裸JSON/HEX文本 |

甲方MD中的控制平台类型写为UINT16，Excel和实际0x0662编码为uint32；本协议采用下文20字节控制帧。任务、态势和控制的速度统一按节解释。

## 帧与通道

| 方向 | 标识 | 内容 | 通道 |
| --- | --- | --- | --- |
| Unity → Python | 0x0660 | simId/id/code/params任务，附unity_only | UDP点播 |
| Unity → Python | 0x0060 | 本艇和全部障碍物，各一帧态势 | UDP组播 |
| Unity → Python | 0x0666 | simId/id/code停止，附当前停车采样扩展 | UDP点播 |
| Python → Unity | 0x0662 | PlatCode/Speed/Course二进制控制 | UDP点播 |
| Python → Unity | 0x0664 | simId/id/code/status/description/params状态 | UDP点播 |

多字节整数大端。公共头12字节：`infoUnitSeq:uint8 / infoUnitId:uint8 / infoUnitLength:uint16 / timestamp:uint32 / reserved:uint16 / secondaryInfoUnitId:uint16`。seq/id/reserved填0，length含头。算法JSON再加totalPackets:uint8 / packetIndex:uint8，头共14字节；首片再带contentLength:uint16，续片无长度，后接UTF-8 JSON。头后最多1450字节（首片含长度），最多10片，JSON实际上限14498字节。支持乱序、相同重复片，冲突片拒绝，3秒过期，无自动重发。guoliang接收端支持分片，其build_json_packet目前只生成单片；本项目按1450字节预算发送。

算法任务/状态/停止时戳为当天UTC起算0.1ms；态势/控制为当天北京时间起算ms。时戳不等于仿真tick。同次采样所有实体共用态势时戳，快速同tick Retry也分配新快照时戳。

默认Unity 127.0.0.1:5005 ↔ Python 127.0.0.1:5006；态势239.255.0.10:18010，默认网卡127.0.0.1，组播回环开启。Python的--host/--port是Unity回包端，--listen-host/--listen-port是任务监听端；--situation-group/--situation-port/--multicast-interface与Bridge一致。只接受配置的Unity源端点。

guoliang现场默认任务0.0.0.0:18014、控制192.168.3.201:18123，Compose示例为192.168.3.126:18123，状态回任务源端口（行为树默认7612）；这些是部署地址差异，Unity使用本机配置。

## 态势字段（tag）

0x0060总长40字节，体为 `>hiiHHHhhHBBI`，按以下顺序：

| 解码字段 | 线类型 | 含义、量化及用途 |
| --- | --- | --- |
| direction | int16 | 相对发现平台、北起顺时针方位，1度；不是运动方向 |
| latitude | int32 | 度，90/2^31；进入位置计算 |
| longitude | int32 | 度，180/2^31；进入位置计算 |
| platformId | uint16 | 平台身份；Unity本艇1001，障碍物2001起，与任务platCode关联 |
| sign | uint16 | 敌我属性；本艇2（我），障碍物0（不明），不进入模型 |
| targetCourse | uint16 | 实际运动方向，北起顺时针1度；65535未知，解码null |
| height | int16 | 高度，Unity填0，不进入二维决策 |
| targetSpeed | int16 | 实际有符号航速，0.1节，范围-20～200节；原始-1未知，不当成-0.1节 |
| type | uint16 | 0不明/1渔船/2浮标/3灯塔；Unity填0，不创建障碍物枚举 |
| targetTypeConfid | uint8 | 0～100，255未知，Unity填未知 |
| threatLevel | uint8 | 威胁等级，255无效，Unity填无效，不进入模型 |
| batch | uint32 | 融合目标批号；Unity以platformId稳定模拟，不作采样编号 |

targetSpeed是实际速度，0x0662的Speed是目标速度，不能互换。Unity障碍物速度为正弦轨迹的导数；标准态势在静止时运动方向未知。本艇当前艏向和角速度通过 execution 补充，用于受限转向预测，静止时仍有效。负速度量化为保留线值-1时用相邻有效-2。经纬度、0.1节/1度运动方向量化实际生效；扩展不重复精确位置或线速度，艏向/角速度是新运动模型所需的独立状态。Unity +X东/+Y北、1单位1米，地理基点120.5°E/30.2°N。

Unity航向就近取整；guoliang模拟编码用int截断浮点航向。两者采用相同的1度线侧网格，舍入策略不同。

## 任务和unity_only

任务code=trackingMode；params为platCode/planRouteNum/currentRoute/planWaypointNum/planWaypointList。点为routeLon/routeLat/routeSpeed（度/度/节）。列表只含目标，点数一致；currentRoute是已完成数量，0指向第一目标，完成时等于点数。simId使用EpisodeId，id使用RunId，均为随机身份，重置/重启天然隔离；每次采样沿用任务身份。地图起点用于Unity显示、任务判定与轨迹，不进入正式目标列表。

unity_only是本项目JSON扩展对象，不是甲方标准报文类型，不直接作为模型状态发送。字段如下：

| 字段 | 含义及必要性 |
| --- | --- |
| action_duration_s:number | 单次动作执行时长，正式控制没有时长字段 |
| decision_id:string、tick:int | 关联当前采样、拦截旧动作；同tick Retry通过不同decision_id区分 |
| event:string | 区分采样、立即停止、完成、碰撞等本地事件 |
| situation_timestamp:uint32 | 精确标记本次快照，不能只取最新缓存 |
| collision_radii_m:对象 | key为全部平台ID，value为米；兼作完整性清单，标准态势没有半径和实体总数 |
| max_speed_knots:number | Unity可调速度上限，用于生成固定8档比例和8航向；不能从实际速度或航点限速推得 |
| execution:对象 | 保留受限运动配置与当前转向状态，见下文；移除恒为 0 且不参与模型的 speed_transition_time_s / course_transition_time_s |
| arrival_radius_m:number、stop_speed_knots:number | arrival_radius_m 仅寻线任务提供，表示打卡半径（本艇包络半径乘可调倍率，默认5倍）；跟踪及其 STOP 省略。停车阈值保留，立即停止使用独立低速阈值，不以转弯半径代替到点范围 |

execution 当前字段：`motion_model="bounded_acceleration"`、`max_acceleration_mps2`、`max_deceleration_mps2`、`max_yaw_rate_deg_s`、`max_angular_acceleration_deg_s2`、`fixed_dt_s`、`hull_length_m/hull_beam_m`；`heading_course_deg` 为北起顺时针实际艏向，`yaw_rate_deg_s` 为顺时针角速度，静止时仍有效。`active_target_course_deg/active_course_error_deg` 表示当前保持的航向目标及未取模角差，保证连续相同指令在过冲或跨 ±180° 时延续原转向分支。`obstacle_avoidance_radii_m` 以障碍物平台 ID 为 key，传递每目标安全半径；`collision_radii_m` 继续只表示实体碰撞半径并兼作完整实体清单。

运行身份与进度使用正式simId/id/currentRoute；Python内部worker envelope由这些字段派生。接收端固定采用暂停决策，候选表按上述速度上限生成。

STOP按simId/id读取已接收的正式任务上下文，正文不包含params；未接收对应任务时拒绝，需先恢复任务通信。缓存任务配置不等于复用旧态势。

有可关联身份的非法采样/未知STOP上下文返回状态3，避免Unity无限等待。STOP状态的currentRoute为最近已接收正式任务的数量，停车动作不使用其导航进度。

Unity先组播全部当前实体，再点播任务/停止；接收顺序不限。Python按半径表key与共同timestamp汇齐，缺一不可；3秒缺失报告失败，由用户Retry，绝不拿上轮补齐。消费后删除对应态势，缓存记录本身也有3秒有效期。同decision_id去重，较旧tick拒绝；新任务撤销旧任务等待和输出，退休身份不重新激活。只缓存正式任务配置供STOP，进入一次决策的态势全部来自当前采样。

## 目标连线距离跟随任务（项目新增）

`code="targetFollowing"` 区分运动目标距离跟随与原 `trackingMode` 航路点寻线。此编码和业务参数是本项目预研扩展，尚非甲方已定义算法；帧头、0x0660/0x0060/0x0666/0x0662/0x0664、单位和动作配对机制复用现有协议。

```json
{"simId":"episode","id":"run","code":"targetFollowing","params":{"platCode":1001,"targetPlatformId":3001,"minFollowingDistanceM":100,"maxFollowingDistanceM":200}}
```

跟随任务不提供 `planWaypointList/planWaypointNum/currentRoute`；本艇与目标身份必须不同，两实体半径之和 < 最小中心间距 < 最大中心间距，均为有限米制数值。目标半径位于 `unity_only.collision_radii_m`，目标必须参加当前快照屏障。目标 0x0060 使用原有位置、实际速度及运动方向，不新增类型枚举；direction 方位不能当作运动方向。目标速度未知、运动中方向未知或本轮目标缺失均失败暂停，不使用旧缓存补齐。`unity_only` 仍复用动作时长、执行状态、快照与停止阈值；跟踪省略不参与判定的 arrival_radius_m。客户端需重启加载该适配变化，旧格式中存在的多余字段仍可接收。

Python 内部 `observation.task.code="targetFollowing"` 保留原任务类型，`observation.operation="execute"/"stop"` 表示执行或停车，停车不覆盖原任务编码；这仅是内部表示，不增加甲方报文字段。目标放在独立 `target`，其他实体为 `obstacles`。HTTP 保留原有 `state.task.mode` 与 `state.task.guidance="line_of_sight_distance"` 表达；`line_reference` 是目标朝向当前本艇的一侧、距目标为区间中点的位置：pF=pT-dRef*(pT-pB)/|pT-pB|。该点不根据目标运动方向定义；不要求本艇在目标后方，不惩罚侧方或前方。距离处于区间内时不强求精确中点，参考点与本艇重合也不表示应停车。原目标方向缓存仅保留内部兼容，后方／横向关系仅由 Unity 作为历史对照指标记录，不再用作模型控制目标。

模型状态提供当前距离、距离变化率、目标与本艇速度在当前连线上的径向分量、目标是否正趋近本艇及直接远离方位；每候选仍提供联合时刻的最小／最大距离、末端距离／距离变化率、本艇末端运动、软参考误差、径向制动余量和长预览。过近或预测将逼近下限时要求主动转向／运动拉开距离，尤其关注迎面目标，必要时安全侧向绕开；本艇零速不会阻止目标继续逼近，不能反复停车等待。超距时仍应安全追近，不能仅因目标正在接近就逃离。直接远离方位仅为几何事实，不是本地代选指令；转向／加速惯性及障碍可能要求另一侧逃逸。

跟踪候选保留 Choice API 的字符串格式和原 64 个 ID；字符串内容改为 JSON 数值表，`state.candidate_schema` 一次解释数组顺序与单位，`state.warning_meanings` 一次解释显式警告码。`range_m/preview_range_m` 为最小、最大、末端距离；`target_hull_clearance_m` 为动作及长预览最小目标实体净距；障碍实体净距包含动作最小／末端及预览最小／末端，安全净距包含动作最小与预览最小／末端。`end_motion`、末端距离变化率、径向制动距离及余量、参考误差、转向误差全部保留。速度字段随模型单位变为 `_mps` 或 `_knots`，其他数值仍为米、度、度每秒。当前距离、当前参考误差、动作时长和预览时长由公共状态提供，候选不重复这些值；不再重复相同的规则、单位和假设段落。

跟踪仅移除与停车距离完全相同的 `braking.distance_to_ceiling_m`，保留 `distance_to_stop_m`；径向制动距离与本艇停车距离含义不同，仍保留。跟踪运动预测通过 `route_details=False` 跳过未使用的中点积分与路径速度数组，端点、逐段路径、速度／角运动及碰撞预测不变。寻线继续使用完整中点与速度数组计算入圈和通过速度。连线参考坐标及 Unity 纵横向诊断未删除。

目标仍按当前速度做匀速预测，未来转弯／急停未知；其他障碍物仍冻结当前位置计算净距。保持原 64 候选，不做本地排序、过滤或代选。当前动作与长时间保持指令的预览警告分开；当前已经过近时所有候选最小值均含初态，比较末端变化及长预览恢复。制动余量为近似同向、目标匀速时的径向估计，不是转弯安全保证。

超距时 HTTP `state.range_recovery` 表达当前恢复阶段、目标方位、带符号航向误差、当前运动是否背离及转向角速度。仅根据当前几何判断：本艇航向与目标方位的绝对误差超过 90° 为 `reorient_before_catchup`，否则为 `close_range`。前者给各候选增加动作末端航向误差与指令方位误差，标注零速不转向及背离指令。要求 Jev 持续执行有用的安全转向，不因掉头第一秒仍增距而反复取消，避免沿错误方向加速；碰撞与障碍安全仍优先。这是决策请求表达，无本地动作代选、排序、过滤或执行端强制限速；区间内只有预计将失距时提示提前恢复，静态相对方位本身不是误差；STOP 不附带恢复阶段。正式任务和态势报文未增加字段。

在区间外半段（中心距不小于区间中点）且正在增距时，若航向仍背离目标，还比较到达上限的时间 `(dMax-d)/range_rate` 与恢复到朝向目标半平面的估计时间。后者按超过 90° 的角度、角速度／角加速度限制及当前角速度估计，并加一个决策间隔；时间不足即提前启用同一恢复阶段。仅该情况补充 `trigger=opening_threatens_maximum_before_turn`、`time_to_maximum_at_current_range_rate_s` 和 `estimated_time_to_inward_heading_s`。这是当前距离变化率及方位的近似，不是未来目标轨迹或安全保证；最低非零速度不是强制限速。

未来目标脚本、整段轨迹及场景时长不进入决策请求。进入距离区间持续执行，超距记录并尝试恢复；Unity 到达实验时长后结束，不保证跟随合格或零速。状态 params.targetPlatformId 标记目标，不伪造 currentRoute。STOP 从已接收任务恢复配置并用当前完整快照，采用独立停车题目，暂停距离跟随与主动远离目标；即使制动期间超过场景时长，也继续到实际停稳后发 stop_completed，导出 stopped。实体碰撞仍优先于同一步结束判定。

## 友方伴随（甲方5.3）

`code="accompanyMode"`，正式params为`platCode/targetPlatCode/offsetAlongMinM/offsetAlongMaxM/offsetAcrossMinM/offsetAcrossMaxM/syncHeading/maxSpeed`，不带航点和currentRoute。纵向正为前方、横向正为右舷，两轴均要求有限且min≤max；首版要求整个允许矩形避开两艇实体包络。maxSpeed为节，0表示不附加任务限速；Python按任务/物理上限的较小值生成原八档比例速度及八绝对航向，不强制瞬间改变实际航速。

当前伴随仅包含本艇和指定友方目标，不启用普通障碍物；友方目标sign=2，仍通过targetPlatCode而非sign选择。目标按同一0x0060提供当前经纬度、实际速度和运动方向并参与快照屏障，运动方向暂作目标艏向。Unity额外提供`unity_only.target_reference_course_deg`，为本轮目标参考方向，静止时直接用于体坐标轴；它不重复位置/速度，也不包含未来路线或转向命令。运动方向有效时以正式态势方向为准，无效的静止方向直接取该扩展，不做推断或历史补帧。

Python内部task.code仍为accompanyMode，operation表示execute/stop；接收参数即按syncHeading选择position_only/heading_sync请求。当前伴随决策恢复初版：只提供单动作末端位置带误差和纵横相对速度，没有位置改善量、显式阶段字段、制动估计或额外警告码。同步分支再提供航向误差及独立同步提示词；暂不增加长预览、复杂恢复阶段或候选碰撞预测。Unity保留两艇实体碰撞检查。不排序、过滤或代选64候选，目标未来运动脚本不进模型。

0x0664.params使用甲方5.3的targetPlatCode、currentAlongM/currentAcrossM、alongOutM/acrossOutM、inPosition、targetSpeed和targetCourse，不伪造currentRoute。处理/控制回包的明细来自本次观测；场景结束通知可带Unity实际末态明细。inPosition=1仍持续状态1。仿真实验达到场景时长后由Unity通知结束并回状态2，这只是实验结束，不是稳定伴随验收；STOP使用当前完整采样并暂停位置/航向目标，直到实际停稳再通知结束。实验初始位置在允许带外，方向、速度和位置带可调；八档速度/航向的离散精度会限制精确保持。

当前初版不估计目标转速，友艇按当前方向匀速直线预测；没有恢复方位、转弯专门分支或独立静止对齐阶段。同步分支先进入并稳定保位，再对齐艏向。初始静止直接给参考方向，零速不能原地转艏。转弯和静止同步仍有已知不足。

## 控制与状态关联

0x0662总长20字节，体 `>IHH`，PlatCode:uint32 / Speed:uint16 / Course:uint16，后两项均0.01精度（节、度）。Unity执行量化解码控制，最大速度校验容纳半个量化步长；不以JSON未量化值替代。

0x0664.params.currentRoute为已完成目标数量；执行中附unity_only:{decision_id,tick,control_timestamp}，processing不带control_timestamp；失败用status=3和description表达。动作值来自0x0662，执行时长来自匹配的本地请求。Unity按正式simId/id、decision_id、tick校验，状态与控制时戳配齐才执行。旧任务/旧决策的动作和状态不应用，终态不由迟到执行中状态覆盖。

状态1/2/3是执行中/完成/决策失败。一次动作不是任务完成；Unity实际判定任务或停车完成后通知Python，再回状态2。API、解析、动作、数据缺失失败为3。Unity碰撞单独失败、冻结并保存轨迹，通知仿真碰撞，不转换成算法状态3。立即停止由Jev选停车动作，实际零速后进入Stopped，不继续航线。

## 单位、模型与仿真规则

线侧全部速度为节；地图、轨迹、内部几何与物理使用米/m/s。Jev默认速度节、距离米，state.units注明1节=1852/3600 m/s；--jev-speed-unit mps仅将模型速度表达设为m/s，通信单位为节。任务routeSpeed转换为内部m/s，表示圈内实际通过上限，不是要求达到的参考速度。

HTTP正文为model/state/questions，提供8档比例[0,.05,.1,.2,.35,.5,.75,1] × 北起45度八航向，共64个Choice。内部目标索引从0起，模型目标编号从1起；已通过但尚未离开的圈继续提供速度上限。Python不排序、过滤或代选动作。64 个候选共享一次任务/运动说明，各候选只列不同的运动、打卡与净距事实，避免重复说明超过模型输入长度限制。

障碍物进入Jev后明确为障碍物，提供当前运动方向、速度、相对位置、半径、净距。附近范围相对安全圈边界计算，覆盖最大速度的制动距离、取消角速度的运动距离及两个动作周期的余量；候选净距冻结障碍物当前位置，不预测其未来轨迹。

Unity障碍物沿正弦轨迹运动，每物理步使用相对扫掠判碰，碰撞优先于同一步的完成判定。航点按序入圈，通过后继续统计圈内速度直到离圈，终点要求圈内低速停车。超速只记录，不裁剪指令、不阻止打卡；完成不能代表没有超速。速度按目标差值受加速/减速上限约束；转向按最短航向差规划受限角加速、匀角速度和角制动过程，相同目标保持转向分支。默认上限为 50 节、+1.46/-1.75 m/s²、12°/s、4°/s²，首轮不建随航速变化的曲线；手动模式使用相同运动上限，无附加阻尼。默认船长 11.6 m、宽 3.4 m，碰撞包络半径为 sqrt(length²+beam²)/2（约 6.044 m），打卡半径为该半径乘 waypointRadiusMultiplier，默认 5 倍（约 30.22 m）；扩大打卡圈同时扩大该点的通过限速适用范围，终点停车条件保留。该倍率通过 Unity 配置及面板修改，已有 arrival_radius_m 继续传输最终生效半径。

小/大目标避碰安全半径默认 50/90 m，实体碰撞半径默认 3/12 m，各目标类别、半径与运动幅度可配。进入安全圈只记录安全距离不足，实体接触才触发碰撞失败。附近目标不限制个数，候选不做本地排序或过滤。

默认动作与正常决策采样周期均为 1 s 仿真时间，物理步长 0.02 s，每动作 50 步。0x0060/0x0660 每次采样发送；0x0664 在 processing、控制结果、完成或失败时即时发送，一轮可以多条。推理等待仍暂停，墙钟发送间隔包含 API 耗时；不是独立的实时 1 Hz 心跳。当前动作按物理步预测；长预览覆盖完整转向与当前制动所需时间，使用不小于 0.1 s 的采样步长，假设保持同一指令，不承诺未来执行或安全。

## 实现和验证

ClientProtocol.cs与protocol/frames.py负责编解码/分片；ObservationBuilder生成任务和实体；BoatUdpBridge与boat_client负责收发；protocol/unity_snapshot.py的UnitySnapshotAdapter负责完整快照屏障及公共观测组装，tasks/负责各任务参数、业务适配、请求构造与状态参数，protocol/status.py封装公共状态关联；geometry/负责共享数值计算，SimulationRunner负责暂停和动作关联。tasks/registry.py统一注册trackingMode、targetFollowing与accompanyMode，航线位于tasks/task1_waypoint.py、伴随位于tasks/task3_accompany.py；followMode尚未实现。主项目不导入guoliang目录，通信不设version字段。

离线测试：

```powershell
python -m unittest discover -s jev_api/tests -v
```

模型JSONL日志包含任务、观测、完整请求、响应、动作及计时，不保存Authorization。闭环实验过程与结果见[本地实验记录](../bian_local/Jev仿真实验记录.md)，请求、响应及轨迹保存在 `Logs/`。
