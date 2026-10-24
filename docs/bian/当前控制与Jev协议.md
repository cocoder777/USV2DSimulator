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
| type | uint16 | 0不明/1民船/2渔船/3商船；Unity填0，不创建障碍物枚举 |
| targetTypeConfid | uint8 | 0～100，255未知，Unity填未知 |
| threatLevel | uint8 | 威胁等级，255无效，Unity填无效，不进入模型 |
| batch | uint32 | 融合目标批号；Unity以platformId稳定模拟，不作采样编号 |

targetSpeed是实际速度，0x0662的Speed是目标速度，不能互换。Unity障碍物速度为正弦轨迹的导数；静止时不提供额外艏向。负速度量化为保留线值-1时用相邻有效-2。经纬度、0.1节/1度量化实际生效，扩展不传递绕过量化的精确值。Unity +X东/+Y北、1单位1米，地理基点120.5°E/30.2°N。

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
| execution:对象 | speed_transition_time_s/course_transition_time_s，预测需要的可调响应时长 |
| arrival_radius_m:number、stop_speed_knots:number | Unity到达圈与停车阈值；立即停止使用独立的低速阈值，不以转弯半径代替到达圈 |

运行身份与进度使用正式simId/id/currentRoute；Python内部worker envelope由这些字段派生。接收端固定采用暂停决策，候选表按上述速度上限生成。

STOP按simId/id读取已接收的正式任务上下文，正文不包含params；未接收对应任务时拒绝，需先恢复任务通信。缓存任务配置不等于复用旧态势。

有可关联身份的非法采样/未知STOP上下文返回状态3，避免Unity无限等待。STOP状态的currentRoute为最近已接收正式任务的数量，停车动作不使用其导航进度。

Unity先组播全部当前实体，再点播任务/停止；接收顺序不限。Python按半径表key与共同timestamp汇齐，缺一不可；3秒缺失报告失败，由用户Retry，绝不拿上轮补齐。消费后删除对应态势，缓存记录本身也有3秒有效期。同decision_id去重，较旧tick拒绝；新任务撤销旧任务等待和输出，退休身份不重新激活。只缓存正式任务配置供STOP，进入一次决策的态势全部来自当前采样。

## 控制与状态关联

0x0662总长20字节，体 `>IHH`，PlatCode:uint32 / Speed:uint16 / Course:uint16，后两项均0.01精度（节、度）。Unity执行量化解码控制，最大速度校验容纳半个量化步长；不以JSON未量化值替代。

0x0664.params.currentRoute为已完成目标数量；执行中附unity_only:{decision_id,tick,control_timestamp}，processing不带control_timestamp；失败用status=3和description表达。动作值来自0x0662，执行时长来自匹配的本地请求。Unity按正式simId/id、decision_id、tick校验，状态与控制时戳配齐才执行。旧任务/旧决策的动作和状态不应用，终态不由迟到执行中状态覆盖。

状态1/2/3是执行中/完成/决策失败。一次动作不是任务完成；Unity实际判定任务或停车完成后通知Python，再回状态2。API、解析、动作、数据缺失失败为3。Unity碰撞单独失败、冻结并保存轨迹，通知仿真碰撞，不转换成算法状态3。立即停止由Jev选停车动作，实际零速后进入Stopped，不继续航线。

## 单位、模型与仿真规则

线侧全部速度为节；地图、轨迹、内部几何与物理使用米/m/s。Jev默认速度节、距离米，state.units注明1节=1852/3600 m/s；--jev-speed-unit mps仅将模型速度表达设为m/s，通信单位为节。任务routeSpeed转换为内部m/s，表示圈内实际通过上限，不是要求达到的参考速度。

HTTP正文为model/state/questions，提供8档比例[0,.05,.1,.2,.35,.5,.75,1] × 北起45度八航向，共64个Choice。内部目标索引从0起，模型目标编号从1起；已通过但尚未离开的圈继续提供速度上限。Python不排序、过滤或代选动作。

障碍物进入Jev后明确为障碍物，提供当前运动方向、速度、相对位置、半径、净距。附近范围为max(12,最大速度×(2×动作时长+最大响应时长)+2)米；候选净距冻结障碍物当前位置，不预测其未来轨迹。

Unity障碍物沿正弦轨迹运动，每物理步使用相对扫掠判碰，碰撞优先于同一步的完成判定。航点按序入圈，通过后继续统计圈内速度直到离圈，终点要求圈内低速停车。超速只记录，不裁剪指令、不阻止打卡；完成不能代表没有超速。速度固定时长匀加速，航向三次Hermite过渡，相同目标延续当前过渡，默认响应各0.5s。

## 实现和验证

ClientProtocol.cs与tools/client_protocol.py负责编解码/分片；ObservationBuilder生成任务和实体；BoatUdpBridge与boat_client负责收发；UnitySnapshotAdapter负责屏障/适配；SimulationRunner负责暂停和动作关联。主项目不导入guoliang目录，通信不设version字段。

离线测试：

```powershell
python -m unittest discover -s jev_api/tests -v
```

模型JSONL日志包含任务、观测、完整请求、响应、动作及计时，不保存Authorization。闭环实验过程与结果见[本地实验记录](../bian_local/Jev仿真实验记录.md)，请求、响应及轨迹保存在 `Logs/`。
