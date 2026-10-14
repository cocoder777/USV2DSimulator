# Jev API 十分钟入门

用 TypeSafe 官方 Python SDK 调用 Jev，沿用[官方 quickstart](https://docs.typesafe.ai/introduction/quickstart) 的唯一一个客服工单示例：一次请求判断负责部门、客户不满程度和是否紧急。

项目只有三个文件：本说明、依赖列表 `requirements.txt`、可直接运行的示例 `route_ticket.py`。

## 1. 安装和运行

需要 Python 3.10 或更新版本。直接在当前默认 Python 环境安装依赖。请先在终端进入本项目目录。

**Windows PowerShell：以下命令可直接在 Windows 运行。**

```powershell
python --version
python -m pip install -r requirements.txt
python route_ticket.py
```

如果 Windows 使用 `py` 启动 Python，把上面的 `python` 全部替换为 `py`。

**WSL Bash：以下命令在 WSL 运行。**

```bash
python3 --version
python3 -m pip install -r requirements.txt
python3 route_ticket.py
```

Windows 与 WSL 使用各自的 Python 环境，需要在实际运行脚本的环境中安装依赖。如果 WSL 的系统 Python 提示 `externally-managed-environment`，请改用允许 pip 安装的个人 Python 环境，例如已有的 Conda 环境，再执行安装命令。

`route_ticket.py` 的 `API_KEY` 已填写本项目使用的 key；其他使用者可直接修改这一行。程序通过官方地址 `https://api.typesafe.ai/v1/systemone` 发起请求，运行需要联网，并消耗账户 API 额度。

## 2. 示例在问什么

工单内容：API 在约 20 分钟前开始持续返回 500 错误，导致客户订单无法处理。

| 代码里的名字 | 作用 |
| --- | --- |
| `api_key` | 官方 TypeSafe 账户的 API key，SDK 负责添加 Bearer 鉴权 |
| `base_url` | 官方服务地址，SDK 负责拼接接口路径 |
| `model` | 使用 `jev-latest` 模型别名 |
| `state` | 待判断的材料，这里是 `ticket` 文本 |
| `questions` | 问题集合，键是自己起的问题标识 |
| `instructions` | 明确告诉模型需要判断什么 |
| `criteria` | 选择题的选项，或评分题的有序等级 |

三道题共享同一份 `state`，每题分别返回结果：

| 问题 | 类型 | 选项或标准 | 返回值 |
| --- | --- | --- | --- |
| `department` | `Choice` 选择题 | `billing` 账单、`technical` 技术、`sales` 销售 | 一个部门标签 |
| `frustration` | `Score` 评分题 | 0 平静陈述、1 不满但礼貌、2 非常愤怒 | 0～2 的分数，可以有小数 |
| `is_urgent` | `Noul` 是非判断 | 消息是否表达紧迫性 | 0～1 的“是”概率 |

`Choice` 的 `criteria` 是“选项标签 → 描述”的字典；`Score` 的 `criteria` 是从低到高排列的描述列表；本例的 `Noul` 不需要 `criteria`。

## 3. 如何读取结果

程序先打印三项判断，再打印完整 JSON 响应，方便调试。下面仅为示意，实际数值可能变化：

```text
department: technical
frustration (0-2): 0.63
is_urgent (0-1): 0.99
```

对应读取方法已经写在脚本中：

```python
response.choices["department"].choice
response.scores["frustration"].score
response.nouls["is_urgent"].noul
```

这些是 Python 代码，Windows 和 WSL 共用。问题标识与 `questions` 中的键一一对应。

完整响应中的 `model` 是本次返回的模型版本；`answers` 是各题答案；`usage` 是 token 用量。`probabilities` 是各选项或等级的概率分布，`confidence` 是置信度，不要默认它等于最大选项概率。`noul` 接近 1 表示倾向“是”，接近 0 表示倾向“否”，0.99 不代表问题有多严重。

## 4. 如何改成自己的调用

修改 `ticket` 就能换一份输入材料。修改 `instructions` 就能换问题；修改 `criteria` 就能换分类选项或评分标准。更改问题标识后，也要同步更改下面读取结果的键。

需要接入其他项目时，复制 `TypeSafeClient` 和 `client.system_one(...)` 这段调用，再用返回值编写自己的分支逻辑即可。Jev 返回结构化判断，本例不会生成客服回复或文字解释。

运行报错时先看终端最后的异常：`ModuleNotFoundError` 通常表示当前 Python 未安装依赖；401 检查 key；网络或超时错误检查连接与服务状态。详细请求格式以[官方文档](https://docs.typesafe.ai/introduction/quickstart)为准。
