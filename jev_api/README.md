# Jev 航线、目标距离跟随与友方伴随决策客户端

接收 Unity 模拟的甲方任务及兵力融合态势，请求 Jev 从 128 个离散候选中选择目标航速/航向。Unity 采样后暂停，收到匹配动作后执行指定时长，再次采样；默认 1 s 仿真时间，物理步长 0.02 s。启动及界面操作见[项目 README](../README.md)，完整字段、分片与状态语义见[当前协议](../docs/bian/当前控制与Jev协议.md)。

| 文件 | 职责 |
| --- | --- |
| `boat_client.py` | 命令行、点播/组播收发、事件循环、状态及控制发送、日志 |
| `protocol/frames.py` | 甲方二进制帧、特殊值、量化和分片重组 |
| `protocol/unity_snapshot.py` | 同次完整快照屏障、身份隔离、调用任务适配 |
| `protocol/status.py` | 状态报文身份、采样关联及公共封装 |
| `validation.py` | 内部请求、公共态势及受限动力学校验 |
| `decision_worker.py` | 决策去重、后台调度、HTTPS 线程上下文 |
| `action_space.py`、`choice_control.py` | 固定 128 Choice、请求总入口、模型表达处理及动作解码 |
| `tasks/registry.py` | 甲方/项目任务编码到任务模块的唯一注册表 |
| `tasks/task1_waypoint.py` | 航线任务参数、业务适配与校验、提示词、请求构造及进度参数 |
| `tasks/distance_following.py` | 当前连线距离跟随的参数、业务适配与校验、提示词、请求及状态参数 |
| `tasks/task3_accompany.py` | 甲方伴随参数、速度上限、两种同步模式的独立请求及状态参数 |
| `tasks/stop.py` | 公共停车操作分发、停车目标与提示词；保留跟随目标的运动/碰撞事实 |
| `tasks/presentation.py` | 模型请求共用的位置、障碍净距文字及避障说明 |
| `geometry/coordinates.py` | 经纬度转固定东/北坐标、公共单位常量 |
| `geometry/waypoint.py` | 航点入圈与通过速度计算 |
| `geometry/motion.py` | 与 Unity 同规则的受限加减速和角运动预测，不选择动作 |
| `geometry/obstacles.py` | 附近障碍物、当前位置快照净距及运动事实 |
| `geometry/relative_motion.py` | 两艇相对运动、连线参考与联合时刻距离扫掠 |
| `geometry/accompany.py` | 旋转目标体坐标投影、位置带误差、位置变化率及候选预测 |
| `tools/model_units.py` | Jev 速度表达单位转换和候选事实字符串序列化，不改变计算与判断 |
| `tools/http_transport.py` | HTTPS 复用、代理兼容、预连接、计时 |

入口与调度位于本目录；任务业务位于 `tasks/`，数值计算位于 `geometry/`，通信协议与快照适配位于 `protocol/`，HTTP 与模型单位表达位于 `tools/`。离线测试位于 `tests/`，示例位于 `examples/`。`jev_api_guoliang/` 是独立程序，本客户端不导入或依赖该目录。

内部观测以 `observation.task.code` 保留原任务编码，以 `observation.operation="execute"/"stop"` 区分执行和停车。停车不覆盖任务类型，仍读取已接收任务配置及新的完整快照。这两个内部字段不增加或改变甲方报文字段，也不直接加入 HTTP state；模型请求保持各任务原有表达。请求经过公共单位转换和 Choice 字符串序列化后，由 worker 统一调用 HTTP，任务模块不直接发网络请求。

新增任务时，在任务模块提供 `validate_params/validate_manifest/target_platform_id/adapt_observation/validate_observation/build_payload/status_params`，并在 `registry.py` 注册编码。`adapt_observation` 接收独立任务状态字典，重置或切换任务时由快照适配器清空；该状态不能补齐当前态势。`status_params(context, observation=None)` 可使用本轮观测生成业务明细，终态可采用Unity通知中的实际统计。需要目标专属停车事实时可提供 `build_stop_payload`，否则使用公共航线停车构造器。甲方 5.2 `followMode` 尚未实现；当前 `targetFollowing` 继续表示原连线距离任务。航线使用 `task1_waypoint.py`，友方伴随使用 `task3_accompany.py`。

## 友方伴随

`tasks/task3_accompany.py`负责甲方5.3参数、观测适配、分阶段请求和状态；`geometry/accompany.py`只计算位置带、旋转体坐标变化率、目标运动外推和候选后果，不选择动作。

正式编码 `accompanyMode`，参数为 `platCode/targetPlatCode/offsetAlongMinM/offsetAlongMaxM/offsetAcrossMinM/offsetAcrossMaxM/syncHeading/maxSpeed`。目标前向为纵轴正向、右舷为横轴正向，单位米。本艇和友艇必须来自当前完整快照；暂不接收普通障碍物，位置带须避开两艇实体包络。静止参考方向使用 `unity_only.target_reference_course_deg`。

基于连续完整快照及物理tick估计目标转速和加速度，不使用墙钟报文间隔，不补齐缺失态势。采用当前观测转速外推；加速度最多延续2秒，此后按所得速度预测，未来脚本不进入决策。稳定保位的指标是旋转体坐标下纵横位置变化率为零，而不是两艇世界速度相等。位置带中心及其运动只作为几何参考。以旋转运动前馈和控制区外位置误差构造LOS参考航向，并提供候选指令/末端艏向相对该参考的偏差；不生成推荐航速或本地执行指令。内部控制区使用0.05米观测容差，甲方外部到位判定仍严格使用原闭区间。

统一以甲方位置带的中间一半作为内部控制区域，外部到位判定不变。控制区外请求强调主动接近及单动作改善，控制区内请求强调减小位置变化率，给外边界留出余量；长时间保持同一指令的越界不能成为停在带外的理由。syncHeading=0始终仅保位；syncHeading=1在已进入控制区、体坐标变化率不超过1m/s且观测目标转速低于0.05°/s时加入艏向同步事实，其他时候优先位置。两种模式使用同一套预测和全部128个候选。

候选提供当前动作及两个更长时刻的纵横位置、甲方位置带及内部控制区外距离、内部控制区单动作改善量、末端体坐标变化率及长预览最小两艇实体净距。预览覆盖至少12秒，并随转向和制动能力延长；本艇沿用共享受限运动模型。后续保持指令只是预测假设，实际每个动作后仍由Jev重新选择；不是完整MPC或稳定性保证。Python不排序、过滤或代选，不计算并执行本地最优控制。

伴随请求只提供旋转体坐标位置变化率，省略另算的世界相对速度投影；艏向和角速度统一放在execution，不在boat重复。模型不重复接收固定物理步长、长宽及空障碍半径表，数值预测仍使用完整物理配置，实体半径与全预览碰撞净距保留。

maxSpeed=0使用物理上限，非零取任务/物理上限较小值，再乘八档比例 `[0,.05,.1,.2,.35,.5,.75,1]`，与16个绝对航向（北起22.5°间隔）组合成128候选。内部使用米及m/s，模型支持节或m/s显示，包括预览速度数组。

0x0664按甲方5.3返回目标编码、currentAlongM/currentAcrossM、alongOutM/acrossOutM、inPosition、targetSpeed（节）及targetCourse。到位仍为状态1；Unity场景结束或实际停车通知后回状态2。STOP保留任务身份，暂停伴随目标，由Jev选择停车动作。

四个伴随场景为Straight、Gentle turn、Speed / stop、Irregular。转弯场景至少360秒：前48秒直航，此后以2°/s持续转弯。Irregular替代静止场景，至少240秒，前40秒直航入位，再按固定种子反复变速及左右转向；默认友艇约6～8节、转速上限2°/s、转速变化约0.14°/s²，航速缓慢变化且不安排停车。参数随本艇物理能力、任务限速及位置带偏移收紧，给伴随位置的所需速度留余量；不加障碍物。Straight和Speed / stop仍默认120秒。实验结果和限制见本地实验记录，场景结束不等于稳定伴随验收。

## 目标距离跟随

`tasks/distance_following.py` 调用 `geometry/relative_motion.py`，按当前态势构造两艇连线上的距离参考位置、距离变化率、目标趋近本艇的径向分量和直接远离方位，并将目标匀速预测与本艇受限运动按同一未来时刻比较。每个候选提供动作期间最小／最大距离、结束距离／距离变化率、软参考误差及保持指令的长预览，仍由 Jev 从原 128 Choice 选动作；不排序、过滤或代选。长预测包含假设与误差限制，不等于完整 MPC 或安全保证。其他障碍物继续采用原快照预测。

新增项目任务编码 `targetFollowing`，`params` 为 `platCode/targetPlatformId/minFollowingDistanceM/maxFollowingDistanceM`，不含航路点或 `currentRoute`；非甲方已定义编码。指定目标通过 `0x0060` 提供当前态势并参与完整快照屏障，适配后独立放在 `observation.target`，不混入普通障碍物。目标缺失、运动数据未知或间距参数无效时失败并暂停。目标低速方向缓存仅保留内部兼容，不用于确定控制参考点，且不能补齐当前快照。模型 state.task.guidance 为 line_of_sight_distance，line_reference 由当前两艇连线计算；原后方位置和横向偏移不再是控制目标。过近或预测逼近下限时要求主动远离，尤其关注迎面目标；超距时仍应恢复间距，不能仅因目标正在接近就逃离。候选运动预测保留转向／加速惯性；零目标速度会忽略航向，不能原地转向或阻止目标继续接近。

超距时 HTTP state 增加 `range_recovery`，本艇航向与当前目标方位相差超过 90° 时为 `reorient_before_catchup`，其余为 `close_range`。区间外半段且正在增距时，若预计到达上限的时间不够转回有效航向，也提前触发恢复，提供该时间及转向时间估计；并非只因区间内朝向不同就强制追逐。恢复阶段补充候选末端航向误差与指令方位误差，要求用安全非零速度持续恢复航向，容许转弯初期增距，避免沿背离方向加速或反复取消转向，最低速度并非固定要求。这些仅是几何事实和 Jev 决策要求，不排序过滤候选，也不强制控制指令；STOP 不带恢复目标。

进入距离区间持续执行，不发完成。Unity 到达场景时长或实际停车后通知完成。跟踪 STOP 读取同一任务配置，并接收新的完整采样，仍通过 Jev 制动。状态上报 `targetPlatformId`，不伪造航路段进度。

## 报文与单位

默认 Unity `127.0.0.1:5005` ↔ Python `127.0.0.1:5006` 为 UDP 点播；态势组播 `239.255.0.10:18010`，默认网卡 `127.0.0.1`。参数 `--situation-group/--situation-port/--multicast-interface` 与 Unity Bridge 一致。采用甲方标识：任务 0x0660、态势 0x0060、停止 0x0666、控制 0x0662、状态 0x0664；JSON 算法报文支持最多十片，首片带总长度，续片不带。

任务按 `simId/id/code/params` 组织，列表只含目标，`currentRoute` 是已完成目标数量，0 对应第一目标。态势按 `platformId` 汇集，通过 `params.platCode` 找本艇。本艇及障碍物共用态势字段；障碍物 `type=0`（不明），进入 Jev 后明确描述为障碍物并提供当前速度和运动方向。`unity_only` 的半径表 key 兼作全部实体清单，按共同快照时戳汇齐才调用一次 Jev，不能用上一轮缓存补齐。消费后删除快照，缺失 3 秒失败；重复 `decision_id` 不重调，新任务撤销旧输出。

`unity_only` 是本项目扩展对象，包含 `decision_id/tick`、`event`、快照时戳、半径表、动作时长、`max_speed_knots`、受限动力学配置、当前艏向/角速度/保持目标状态、障碍物安全半径和到达/停车阈值。运行身份和进度使用正式 `simId/id/currentRoute`，候选表由接收端按固定定义生成。STOP 按正式身份读取已接收任务上下文；未接收对应任务时拒绝。完整字段及必要性见当前协议。

线侧所有速度为节，位置为经纬度。地图与内部计算保留米和 m/s；适配器将任务节速度转换为内部 m/s，不改变圈内上限含义。Jev 默认速度为节、距离为米，state.units 说明换算；可加 `--jev-speed-unit mps` 回退模型表达，通信继续为节。

航速航向以 0.01 精度发送，态势采用 Excel 定义的 0.1 节/1 度及经纬度量化。控制与状态通过 `unity_only` 的 `decision_id/tick/control_timestamp` 配齐，正式 `simId/id` 隔离运行。Unity 执行量化控制，时长取匹配的本地请求，旧动作和旧状态均拒绝。

算法状态 1 为执行中，2 为 Unity 确认的任务/停车完成，3 为决策失败。碰撞是 Unity 自己判定的仿真失败，不转换成算法状态 3。立即停止仍请求 Jev 选择停车动作，保持原有 Stopped 行为。

## 模型请求

HTTP 正文为 `model/state/questions`，提供 8 档非负速度 × 16 个绝对航向，共 128 个候选。寻线沿用运动、打卡及净距文字说明；跟踪使用 JSON 数值表字符串，公共 `state.candidate_schema/warning_meanings` 定义列、单位和显式警告码，减少 128 份重复规则。全部候选预测数值及碰撞／距离警告保留。 跟随候选的values按state.candidate_value_columns排列，warnings及turn_error_deg保持具名；航线重复策略段移至state.candidate_policies，各候选引用同一原文。当前运动、目标和后续点使用固定东/北视角，无参考虚线。`execution` 提供物理上限和当前转向状态，静止时也使用实际艏向预测起步；速度和位置仍取正式态势的量化值。恒为零的两项过渡时间不再发送。默认 50 节、加速 1.46 m/s²、减速 1.75 m/s²、角速度 12°/s、角加速度 4°/s²，全部从 Unity 配置读取。打卡半径与碰撞半径分开：默认分别约 30.22 m / 6.044 m，跟踪省略到达半径，终点仍须满足低速停车。已有客户端需重启加载精简后的适配。候选预测为近似计算，Python 不打分、排序、过滤或代选，Unity 不强制裁剪通过速度。

`state.obstacle_avoidance` 提供附近障碍物的当前相对坐标、实体/避碰半径、两种净距、速度及北起顺时针运动方向。小目标安全半径默认 50 m，大目标 90 m（可调 80～100 m），安全圈侵入与物理碰撞分别统计。候选净距预测冻结障碍物当前位置，不预测其未来轨迹。当前动作按物理步预测，长预览使用不小于 0.1 s 的采样步长；时长覆盖转向和当前制动所需时间，仅表达保持当前指令的假设。请求提供制动距离、转向制动角及长预览末端安全净距。

完整模型请求样本见 [choice64_request.json](examples/choice64_request.json)。

API 默认 `POST https://api.typesafe.ai/v1/systemone`、模型 `jev-latest`。HTTPS 线程独立复用连接，预连接不发业务请求；失败后使用手动 Retry，不自动重发。JSONL 保存任务、观测、payload、响应、动作与计时；`wait_headers_s` 包含网络和服务端等待，非纯推理时间。

## 验证

```powershell
python -m unittest discover -s jev_api/tests -v
```

闭环实验过程与结果见[本地实验记录](../docs/bian_local/Jev仿真实验记录.md)，完整请求、响应及轨迹保存在 `Logs/`。
