# Kev 与 Laya 部署及使用指南

本分支提供 Unity 无人艇仿真、模型客户端、模型切换按钮，以及可重新部署的轻量脚本。**模型权重和 Python 环境不随仓库上传**；安装时从原作者仓库下载固定版本，每台电脑独立创建环境。

当前模型接入可以完成 HTTP 请求、UDP 通信和真实 Unity 动作执行，但不表示模型能够正确导航。已有统一闭环报告中，**Laya 0/9、Kev 0/9**；详见[模型统一对照报告](模型统一对照报告.md)。本指南不涉及训练或改善决策策略。

## 1. 获取项目与环境要求

```powershell
git clone --branch codex/kev-laya-integration https://github.com/cocoder777/USV2DSimulator.git
cd USV2DSimulator
```

- Windows 10/11 x64、Git、可用的 Python 3.10 或更新版本，用于执行安装入口。
- Unity **6000.6.5f1**，实际版本以 `ProjectSettings/ProjectVersion.txt` 为准。
- 支持 CUDA 和 BF16 的 NVIDIA GPU，以及兼容 CUDA 12.8 运行时的驱动。既有测试机器为 RTX 3060 Laptop 6GB；显存占用随 Unity、输入和后台负载变化，6GB 不作为所有场景的容量保证。
- 首次安装需要访问 GitHub、Python 包索引、PyTorch CUDA wheel 索引和 Hugging Face，并预留足够空间容纳两份环境、模型、Kev 基座和缓存。

安装入口使用 `uv` 下载独立 **Python 3.12.15**。先在运行安装入口的 Python 中安装 uv：

```powershell
python -m pip install uv
```

uv 不需要加入 PATH；入口默认通过 `python -m uv` 调用。已安装独立 uv.exe 时可传 `-UvPath '完整路径\uv.exe'`，Python 不在 PATH 时可传 `-Python '完整路径\python.exe'`。

## 2. 安装模型服务

在项目根目录执行：

```powershell
# 先预览下载版本和目标位置：不会写文件或下载
powershell -NoProfile -ExecutionPolicy Bypass -File .\Tools\LocalModels\setup.ps1 -Backend both -Plan

# 安装两种模型；也可只选 -Backend laya 或 -Backend kev
powershell -NoProfile -ExecutionPolicy Bypass -File .\Tools\LocalModels\setup.ps1 -Backend both
```

默认部署结构为：

```text
D:\AI\Laya\        Laya-multilingual 环境、源码、权重、启停脚本
D:\AI\Kev-0.8B\    Kev 环境、源码、适配器、决策头、Qwen 基座、启停脚本
```

入口依次创建独立环境、安装固定依赖、检查 CUDA/BF16、下载固定 revision 的权重，并生成当前电脑的 `models.json`（模型路径和逐文件 SHA-256）、`project.json`（项目位置）。安装不会启动模型服务。默认运行时离线加载，仅监听本机。

可自定义根目录，支持空格：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\Tools\LocalModels\setup.ps1 -Backend both -DeploymentRoot 'E:\AI Models'
```

自定义根目录下面仍使用 `Laya` 和 `Kev-0.8B` 两个子目录。Unity Play 后选择艇对象，在自动添加的 **LocalModelSwitcher** 组件中将 `deploymentRoot` 设为相同根目录。迁移项目后，手动客户端可传 `-ProjectPath '新的项目路径'`，避免使用旧的 `project.json`。

**已有部署保护：**目标目录存在时，只核对模型版本、逐文件校验值、源码提交、Python 和依赖版本及必要脚本，核对成功即复用，不覆盖已有文件。非本入口创建的旧部署也允许核对后复用。新入口创建但尚未完成的部署会明确报错并保留现场；请检查错误后选择新的空目录重新安装，不会自动删除或修复旧目录。

只核对已安装文件，不安装、不启动服务：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\Tools\LocalModels\setup.ps1 -Backend both -VerifyOnly
```

| 内容 | 固定版本 |
| --- | --- |
| Python / CUDA torch | 3.12.15 / 2.8.0+cu128 |
| Laya 上游源码 | `NandhaKishorM/laya` @ `d74e8be46ef5241e4ba2b16cd25e896f24f71373` |
| Laya 权重 | `convaiinnovations/laya-multilingual` @ `1720e3e3357cfe1e281542e223f8273b0890ca34` |
| Kev 上游源码 | `jaredpalmer/kev` @ `1f04cfa28e6a708485b28a9585010b9e436b776e` |
| Kev 权重 | `jaredpalmer/kev-0.8b` @ `bf75a6a8848ea6960ff2ed108d9ed44c2941174f` |
| Kev 基座 | `Qwen/Qwen3.5-0.8B-Base` @ `dc7cdfe2ee4154fa7e30f5b51ca41bfa40174e68` |

机器可读版本在 `Tools/LocalModels/pins.json`，两种环境的依赖各见对应子目录的 `requirements.lock.txt`。不会用浮动 latest 或同名 PyPI 包代替固定源码。下载的上游源码、权重保留其许可证文件；本仓库不分发第三方权重。

## 3. 在 Unity 使用模型按钮

1. Unity Hub 添加克隆的项目，等待导入后打开 `Assets/Scenes/Main.unity`，点击 Play。
2. 在右侧选择地图；首次可使用 Level 0。
3. 点击 **Run Laya** 或 **Run Kev**。按钮自动准备服务、启动 Python 客户端、切换到 Jev 通信控制模式，并从当前地图起点运行。首次加载可能需要几十秒。
4. 核对界面的实际 Model、决策耗时、仿真时间和动作。界面中的 Jev 模式是通信执行模式，实际本地模型以 Model 显示为准。
5. 点击另一个模型会保存上一轮轨迹并重置当前任务；**Restart same model** 复用已加载模型和客户端，从头重跑。

Laya/Kev 按钮沿用当前动力学参数与动作时长。比较模型时请统一地图、参数和**仿真时间**；等待推理的墙钟时间不计入仿真时间。按 R 或切换地图后不会自动开始下一轮，可用 Restart same model 重跑。服务失败时暂停，不会自动调用云端或更换模型。

**Run Jev / Start Jev (cloud)** 使用云端服务，需自行配置 `JEV_API_KEY`，与本地两种模型分开。不要把真实密钥写进源码。当前新版本清除了示例脚本中的硬编码密钥，但旧 Git 历史仍包含旧值，需在服务提供方更换该密钥；本分支不会重写历史。

Windows 构建包还需将组件 `projectRoot` 指向包含 `Tools` 和 `jev_api` 的项目目录；构建包不会自动包含 Python 或模型。

## 4. 手动启动方式

使用 Unity 模型按钮时无需重复执行本节。手动运行时，同一时间只保留一个客户端，切换模型前 Ctrl+C 停止旧客户端，并停止另一模型服务以释放显存。

```powershell
# 启动 Laya，等待 ready
powershell -NoProfile -ExecutionPolicy Bypass -File D:\AI\Laya\start-laya.ps1
# 检查版本、GPU、预算保护
powershell -NoProfile -ExecutionPolicy Bypass -File D:\AI\Laya\check-laya.ps1
# 另开终端，保持客户端运行
powershell -NoProfile -ExecutionPolicy Bypass -File D:\AI\Laya\start-boat.ps1 -ProjectPath '你的项目完整路径'
```

使用 Kev：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File D:\AI\Kev-0.8B\start-kev.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File D:\AI\Kev-0.8B\check-kev.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File D:\AI\Kev-0.8B\start-boat.ps1 -ProjectPath '你的项目完整路径'
```

然后在 Unity 中切换到 Jev 模式，设置动作时长（默认 0.40 s），点击 **Start selected / external client**。不要点击 Start Jev (cloud)，该按钮会启动云端客户端。

也可在项目根目录直接运行客户端：

```powershell
New-Item -ItemType Directory -Force Logs | Out-Null
python jev_api/boat_client.py --backend laya --log Logs/laya.jsonl
# 或：python jev_api/boat_client.py --backend kev --log Logs/kev.jsonl
```

| 后端 | HTTP 接口 | 模型标识 | 默认输入 |
| --- | --- | --- | --- |
| Laya | `http://127.0.0.1:8010/v1/systemone` | `multilingual` | compact，全部 56 个动作 |
| Kev | `http://127.0.0.1:8009/v1/systemone` | `kev-latest` | original，全部 56 个动作 |

Unity UDP 接收端口为 **5005**，Python 客户端为 **5006**，协议为 **version 5**。CLI 不指定后端时默认 Laya；`--endpoint`、`--model` 和 `--prompt-profile` 可显式覆盖默认值。本机回环请求无需云端密钥，客户端不会向本地模型发送云端密钥。

客户端 Ctrl+C 后，按需停止服务：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File D:\AI\Laya\stop-laya.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File D:\AI\Kev-0.8B\stop-kev.ps1
```

停止脚本核对 PID、进程启动时间、解释器和启动命令，只管理该部署记录的服务。已有服务正常时不要重复启动。

## 5. 日志、轨迹和排查

- 模型服务日志：部署目录的 `logs/service-*.out.log`、`logs/service-*.err.log`。
- 手动客户端日志：部署目录的 `logs/boat-*.jsonl`，或 CLI 的 `--log` 指定文件。
- Unity 按钮会话：项目 `Logs/model-switch/`，包括启停日志、状态及完整请求/回答。
- Editor 轨迹：`Logs/trajectories/`；构建程序轨迹：`Application.persistentDataPath/Trajectories/`。未完成的轨迹也会保存，并按模型标记。
- 绘图依赖单独安装：`python -m pip install -r Assets/StreamingAssets/Tools/requirements.txt`，再运行 `python Tools/configure_trajectory_plot.py`。缺少绘图环境不影响轨迹 JSON 保存。

| 问题 | 处理 |
| --- | --- |
| GitHub/Hugging Face/包索引连接失败 | 检查网络或代理；首次安装需要联网。不要把不完整目录当作部署成功，使用新根目录重试。 |
| 缺少权重、models.json 或校验不一致 | 运行 VerifyOnly 定位错误；重新安装到空目录，不要手改模型 revision 或路径冒充成功。Laya 官方 tokenizer 元数据规范化通过原始配置副本单独核对。 |
| CUDA/BF16 不可用、显存不足 | 检查 NVIDIA 驱动，停止另一模型及不必要的 GPU 程序。当前服务不支持静默降到 CPU。 |
| 8009/8010 已占用 | 先运行对应 check 脚本确认服务身份；正常服务可直接复用，不要重复启动。未知占用由使用者确认后关闭。 |
| UDP 5006 已占用 | 停止重复客户端；归属不明确的旧客户端不会被按钮自动关闭。 |
| Unity 提示找不到 Python 或管理脚本 | 检查部署根目录、两个子目录名和项目路径；在组件设置 deploymentRoot，构建程序另设 projectRoot。 |
| 等待模型时艇静止 | 正常设计：等待推理时暂停仿真，收到动作后执行下一段。 |
| 模型持续倒退、转圈或未通过任务 | 当前已观察到的决策质量问题；先查看原始回答与统一报告，不能把接口正常等同于导航达标。 |

回归测试（项目根目录）：

```powershell
python -m unittest discover -s jev_api -p 'test_*.py'
python -m unittest discover -s Tools -p 'test_*.py'
```

安装脚本的安全性测试使用小型临时文件和模拟结果，不下载真实模型、不停止实际服务。完整新机器安装需具备上述网络和 GPU 条件；具体本次发布验证范围见发布提交说明。
