# Jev 寻线与目标尾随决策客户端

接收 Unity 模拟的甲方任务及兵力融合态势，请求 Jev 从 64 个离散候选中选择目标航速/航向。Unity 采样后暂停，收到匹配动作后执行指定时长，再次采样；默认 1 s 仿真时间，物理步长 0.02 s。启动及界面操作见[项目 README](../README.md)，完整字段、分片与状态语义见[当前协议](../docs/bian/当前控制与Jev协议.md)。

| 文件 | 职责 |
| --- | --- |
| `boat_client.py` | 点播/组播接收、任务适配、状态及控制发送、日志 |
| `tools/client_protocol.py` | 甲方二进制帧、特殊值、分片重组、Unity 快照适配 |
| `decision_worker.py` | 决策去重、后台调度、HTTPS 线程上下文 |
| `choice_control.py` | 导航/立即停止模型请求、64 Choice、动作解码 |
| `tools/waypoint_geometry.py` | 经纬度转固定东/北坐标、入圈与通过速度计算 |
| `tools/navigation_motion.py` | 与 Unity 同规则的受限加减速和角运动预测，不选择动作 |
| `tools/obstacle_geometry.py` | 附近障碍物、当前位置快照净距及运动事实 |
| `tools/model_units.py` | Jev 速度表达单位转换，不改变计算与判断 |
| `tools/http_transport.py` | HTTPS 复用、代理兼容、预连接、计时 |

入口与核心流程位于本目录，辅助计算与传输位于 `tools/`，离线测试位于 `tests/`，示例位于 `examples/`。`jev_api_guoliang/` 是独立程序，本客户端不导入或依赖该目录。

## 目标距离跟随

`tools/following_geometry.py` 按当前态势构造两艇连线上的距离参考位置、距离变化率、目标趋近本艇的径向分量和直接远离方位，并将目标匀速预测与本艇受限运动按同一未来时刻比较。每个候选提供动作期间最小／最大距离、结束距离／距离变化率、软参考误差及保持指令的长预览，仍由 Jev 从原 64 Choice 选动作；不排序、过滤或代选。长预测包含假设与误差限制，不等于完整 MPC 或安全保证。其他障碍物继续采用原快照预测。

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

HTTP 正文为 `model/state/questions`，提供 8 档非负速度 × 8 个绝对航向，共 64 个候选。寻线沿用运动、打卡及净距文字说明；跟踪使用 JSON 数值表字符串，公共 `state.candidate_schema/warning_meanings` 定义列、单位和显式警告码，减少 64 份重复规则。全部候选预测数值及碰撞／距离警告保留。当前运动、目标和后续点使用固定东/北视角，无参考虚线。`execution` 提供物理上限和当前转向状态，静止时也使用实际艏向预测起步；速度和位置仍取正式态势的量化值。恒为零的两项过渡时间不再发送。默认 50 节、加速 1.46 m/s²、减速 1.75 m/s²、角速度 12°/s、角加速度 4°/s²，全部从 Unity 配置读取。打卡半径与碰撞半径分开：默认分别约 30.22 m / 6.044 m，跟踪省略到达半径，终点仍须满足低速停车。已有客户端需重启加载精简后的适配。候选预测为近似计算，Python 不打分、排序、过滤或代选，Unity 不强制裁剪通过速度。

`state.obstacle_avoidance` 提供附近障碍物的当前相对坐标、实体/避碰半径、两种净距、速度及北起顺时针运动方向。小目标安全半径默认 50 m，大目标 90 m（可调 80～100 m），安全圈侵入与物理碰撞分别统计。候选净距预测冻结障碍物当前位置，不预测其未来轨迹。当前动作按物理步预测，长预览使用不小于 0.1 s 的采样步长；时长覆盖转向和当前制动所需时间，仅表达保持当前指令的假设。请求提供制动距离、转向制动角及长预览末端安全净距。

完整模型请求样本见 [choice64_request.json](examples/choice64_request.json)。

API 默认 `POST https://api.typesafe.ai/v1/systemone`、模型 `jev-latest`。HTTPS 线程独立复用连接，预连接不发业务请求；失败后使用手动 Retry，不自动重发。JSONL 保存任务、观测、payload、响应、动作与计时；`wait_headers_s` 包含网络和服务端等待，非纯推理时间。

## 验证

```powershell
python -m unittest discover -s jev_api/tests -v
```

闭环实验过程与结果见[本地实验记录](../docs/bian_local/Jev仿真实验记录.md)，完整请求、响应及轨迹保存在 `Logs/`。
