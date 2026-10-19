# PD 状态—动作监督数据采集方案

日期：2026-10-18。本文件是下一阶段方案，本轮没有训练、没有替换模型输出，也没有启动批量采集。

## 目标与边界

用已在真实 Unity 九场景通过的 `Tools/classic_control.py --method pd` 作为教师，采集“决策前观测 → 同一 56 动作中的教师选择 → 0.4 秒实际执行结果”。目的是让模型学习当前系统的基本转向、制动和按序到点行为；不能把模仿 PD 等同于最优控制或严格贴合参考曲线。

冻结当前动力学、动作顺序、0.02 秒固定步长、0.4 秒保持、任务判定、教师增益及输入构造器。所有采集在独立 Unity 测试副本进行，不更改原项目默认后端或控制配置。原始模型测试数据单独保存，不能把教师动作写回模型原始响应。

## 复用现有链路

复用 `control_benchmark.py`、`ControlBenchmarkValidation.cs`、`classic_control.py` 和现有 UDP v5，不另写仿真器。`worker.jsonl` 已有每个唯一请求、PD 动作和计算耗时；`unity-trace.jsonl` 有决策前状态、接收动作以及逐固定步状态。按完整 session/episode/run/decision/tick 标识关联，去除 UDP 重复包。

训练输入从请求的 `observation` 用现有 `choice_control.build_payload` 重建，分别保留 Laya compact 和 Kev original 的输入格式。只把 PD 动作编号转换为原候选键；保留全部 56 选项及原顺序，不新增“教师建议”输入。模型不得看到未来轨迹、完成结果或教师动作。

以下仅示意未来导出时的标签结构，`observation` 和 `teacher_action` 来自同一个已核验的 PD 决策；本轮不执行导出或训练：

```python
from jev_api.choice_control import ACTION_CHOICES, build_payload

choice = next(key for key, value in ACTION_CHOICES.items()
              if value == teacher_action)
laya_input = build_payload(observation, model="multilingual", profile="compact")
laya_row = {"state": laya_input["state"], "questions": laya_input["questions"],
            "expected": {"action_0": choice}}
kev_input = build_payload(observation, model="kev-latest", profile="original")
kev_row = {"state": kev_input["state"], "questions": kev_input["questions"]}
kev_row["questions"]["action_0"]["label"] = choice
assert len(laya_row["questions"]["action_0"]["criteria"]) == 56
assert len(kev_row["questions"]["action_0"]["criteria"]) == 56
```

训练标签不能混进正常推理请求。例如动作 `(7,3)` 的候选键是 `positive_full__no_angular_input`；标签是候选键而非文本描述，也不是连续的 `(a, alpha)` 数值。正式导出还需通过各官方训练数据读取器和完整 token 预算校验。

## 数据字段

建议存为 `manifest.json`、`episodes.jsonl`、`decisions.jsonl`、`steps.jsonl`，再导出各模型专用训练 JSONL。

| 分组 | 字段与单位 | 用途 |
|---|---|---|
| 版本与来源 | schema_version、数据集版本、Git 状态及文件哈希、场景/路线哈希、教师版本/配置哈希、随机种子 | 可复现；未提交代码也必须有内容哈希 |
| 标识 | route_family、scenario、episode_id、session_id、run_id、decision_id、tick | 关联、去重、按整局拆分 |
| 时间 | wall_time_utc、simulation_time_s、fixed_dt_s=0.02、action_duration_s=0.4、实际执行步数 | 区分暂停推理的墙钟时间和仿真时间 |
| 原始观测 | 完整 observation：boat、target、dynamics、action_space、planning 等 | 唯一输入事实来源，保留有符号线速度 |
| 艇状态 | 世界 x/y(m)、heading_deg、speed_mps、yaw_rate_deg_s | 单位和方向沿用当前协议 |
| 当前任务 | target x/y、arrival_radius_m、requires_stop、stop_speed_mps；任务点索引及路线 ID 作审计 | 模型只使用当前协议已有字段，不额外增加整条路线预览 |
| 派生量 | target_forward_m、target_left_m、distance_m、bearing_deg、nearest_path_deviation_m | 检查坐标变换与分桶；路径偏差仅作审计，不偷偷增加模型输入 |
| 教师标签 | acceleration_index、turn_index、原 choice 键、实际 a(m/s²)、alpha(deg/s²) | 硬标签，禁止把索引当连续加速度 |
| 教师信息 | method=pd、完整 ControllerConfig、计算耗时、采集类型 on_policy/recovery | 明确标签来源，区分补充状态 |
| 执行结果 | 实际收到动作、开始/结束 tick、实际执行时间、next_state、经过点、终止原因 | 验证标签确实被执行；末段可能提前完成 |
| 质量标志 | 状态有限、初始状态一致、56 选项完整、标识匹配、响应有效、执行完整性、是否成功局 | 筛选与质量统计 |
| 模型格式 | backend_profile、构造后 state/questions、token_count、选项键顺序/哈希 | 可重建推理输入并验证没有截断 |

`steps.jsonl` 额外保留每 0.02 秒的命令、执行器实际输出和运动状态。监督样本只在每个唯一决策的执行前取一次，名义频率 2.5 Hz；不要把 20 个保持步复制成 20 条相同标签，造成虚假数据量。提前完成的最后动作标记实际时长；服务异常、人工中断和身份不匹配记录不能伪装成成功执行样本。

## 采样覆盖与数量

1. 管线验收：现有 PD 九场景只用于核验日志关联、单位、标签和闭环回放，不将这九个固定测试场景加入正式训练集。其样本量仅百余条，远不足以证明泛化。
2. 小批试采：另外生成约 100 个独立任务，目标 3,000～5,000 个唯一决策。覆盖直行、左右转弯、不同距离、初始倒退、正负角速度、终点制动和多点任务；先检查动作覆盖、类别分布、日志体积及教师通过率。
3. 正式采集：约 1,000 个独立任务，按平均 30～100 次决策规划 30,000～100,000 条。它是预算区间，按实测平均局长调整，不通过重复相同初始状态凑数量。

保持动力学参数固定；在副本的任务初始状态与路线中变化距离、方向、初始有符号速度和角速度，范围不超出当前速度/角速度限制。使用种子生成、预先保存场景。每个新场景先让教师独立执行，保留成功和失败的原始记录；失败局不盲目全部当优质监督，分析后单独标记。

尽量让正常前进、左右转弯、倒退制动、正负过度转向、临近终点/停车、连续必过点都占有明确配额，并统计“动作对、线加速度、角加速度”三级分布。并非全部 56 动作都是教师常用动作，不能为填满类别而伪造标签。可在独立采集副本从有界偏航/倒退初始状态让 PD 恢复，增加真实恢复样本；未来若做 DAgger，必须另行开展并标识，不能改写本轮模型对照轨迹。

按 50 Hz 留存状态时，步日志行数约为决策数的 20 倍；完整 prompt 体积较大。先以试采实测每条大小，再按“条数 × 平均字节 × 2 倍余量”核算空间，压缩归档重复配置。当前磁盘余量需重新检查，不能按最早部署时的余量直接开大批采集。

本轮 2026-10-18 检查 D 盘剩余约 **9.0 GiB**，不是最早部署时约 30GB。先以小批试采验收体积；正式数万至十万条原始日志、导出副本、训练缓存和新 checkpoint 的合计空间必须另行核算，不能通过删除既有测试结果腾空间。

## 数据拆分与验收

按路线族/整局/种子拆分，例如训练 70%、验证 15%、测试 15%；同一路线族及其镜像、轻微初始扰动尽量放同一分组。严禁随机拆相邻时间步，避免同一条轨迹泄漏。原九场景另外作为冻结回归集，不能拿它反复挑增益、挑 checkpoint 后再宣称独立泛化。

采集前固化清单和场景分配；采集后检查 ID 唯一、数值有限、动作索引范围、标签与动作物理值一致、时间步连续、开始状态一致、UDP 身份一致、所有候选完整、教师配置无变化。训练导出只保留执行前可获取信息和标签；所有过滤都有数量与原因，不静默丢弃难样本。

离线评价包括动作准确率、各状态桶准确率、混淆矩阵、合法输出率和耗时；接近等价动作可能使准确率低估控制效果。最终必须在真实 Unity 用相同场景/时限/判据重新闭环，分别统计导航成功、技术中断、终点速度和路径偏差，不能只凭分类准确率交付。

## 监督微调可行性与后续门槛

本机已固定的官方源码具备带标签决策训练入口，但两个模型的训练格式不同，不能直接把通用聊天问答 JSON 或轨迹 CSV 交给任意 SFT 工具。

- Laya：本地 `source/docs/finetune.md` 和 `laya/train.py` 支持 state/questions/expected（或 gold）数据，`TrainConfig` 有 `freeze_encoder` 与 soft-ce 目标。后续可以先评估冻结编码器、训练决策相关部分的低显存实验；完整微调另测。需要把教师标签映射到 `expected.action_0` 的真实候选键，并验证原 multilingual 分词器下全部 56 选项保留。
- Kev：本地 `kev/data.py` 说明标签在 `questions.<id>.label`，`kev/train.py` 提供 LoRA 加 pointer head 的训练路径。后续需核实从现有 0.8B checkpoint 初始化时头部的处理方式，不能假设与 Laya 共用格式或默认保留旧 head；该脚本还导入 Unix `resource`，Windows 原生训练兼容性需要单独验证。

3060 Laptop 6GB 上推理成功并不保证训练能放下。冻结部分参数、小 micro-batch、累积梯度、检查点等只能作为后续评估选项；在原约 2k/7k token 输入和 56 选项条件下必须实际测量训练显存、速度和稳定性。训练输出应保存到新的 checkpoint 目录，原权重不可覆盖。先验收数据和训练格式，再另行决定训练方式、资源与预算；本轮未安装训练依赖、未启动训练、未修改部署配置或权重。
