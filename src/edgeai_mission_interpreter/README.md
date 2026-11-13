# edgeai_mission_interpreter

Task 2B 任务解释器。现在只有**规则基线**：不用模型，把一条自然语言指令变成候选步骤（`MissionStep` 的五项）。它是整条链路的起点，以后模型出错时也用它兜底。

还没有 ROS 节点和服务（那是卡 04），也不做验证（验证器是卡 02）。

## 文件清单

| 文件 | 作用 |
|---|---|
| `edgeai_mission_interpreter/interpreter.py` | 规则解释器：`interpret(instruction, registry, policy)` |
| `edgeai_mission_interpreter/policy.py` | 读取和检查策略表 |
| `config/policy.yaml` | 策略表：规则书和接口没定下来的四种情形怎么处理 |
| `test/test_interpreter.py` | 规则书 6.8 的英文例句、`mission_plan.example.json` 的中文例句、步骤字段和 schema、看不懂就不猜 |
| `test/test_issue_codes.py` | 问题码、`action`、目标类型对照仓库里的 msg 和 schema 文件 |
| `test/test_policy.py` | 四种待定情形，每条指令在策略的两个取值下各跑一遍 |
| `test/test_testset_v1.py` | 60 条测试集 v1 全部跑一遍：输出格式、禁止内容扫描、副本校验和 |
| `test/data/testset_v1/` | 60 条测试集 v1 的副本（原件在 Ubuntu 的 `~/edgeai_llm_dev/testset/`，没改过内容） |
| `test/test_copyright.py`、`test_flake8.py`、`test_pep257.py` | 版权头、代码风格和文档字符串检查 |

## 用法

```python
from edgeai_mission_interpreter import interpret, registry_from_target_list

registry = registry_from_target_list(frozen_target_list)   # {target_id: 'normal' | 'cooperative'}
result = interpret('先救援T2；T2完成后救援T1和T3；两者都完成后，协同救援TC1。', registry)
result.ok        # True
result.steps     # [{'step_id': 'step-t2', 'action': 'rescue', 'target_id': 'T2', 'priority': 0,
                 #   'predecessor_step_ids': []}, ...]
result.issues    # 失败时一条 Issue(code=12 或 13, detail=...)，steps 为空
result.notes     # 用到策略表时写在这里
```

- `registry` 只用来展开「所有目标」「所有普通目标」这类说法；解释器不检查目标是否存在、有没有环，这些原样交给验证器（问题码 102、105）。
- 失败只用解释器阶段的两个码：12 指令里没有可结构化的任务内容（闲聊、注入、速度坐标这类底层运动），13 有歧义（没指明目标、条件或备选、「或」选目标）。数值从 `MissionValidationIssue.msg` 抄来，测试会对照原文件。
- 输出里不会出现机器人编号、坐标、路径、速度；`notes` 也不会照抄机器人名。

## 策略表

`config/policy.yaml` 的默认值就是「框架」第 7 节前四条（Yueer 2026-11-13 拍板）：

| 情形 | 默认 | 另一个取值 |
|---|---|---|
| `or_dependency`「T1 或 T3 完成后救 TC1」 | `tighten_to_all`：收紧成「都完成」，前提是 T1、T3 本来就在计划里要救，否则按 `reject` | `reject`：失败，码 13 |
| `time_limit`「3 分钟内救 T1」 | `raise_priority`：不管时限，这一步优先级调到最高，时限原文写进 `notes` | `reject`：失败，码 13 |
| `named_robot`「让 robot_2 救 T1」 | `drop_robot`：去掉机器人名，保留目标和顺序 | `reject`：失败，码 13 |
| `duplicate_target`「最后再救一次 T1」 | `merge`：合并成一步 | `keep_both`：保留两步，第二步排在第一步后面 |

改策略只改这个文件，不用改代码；取值写错时 `load_policy()` 抛 `PolicyError`。

## 测试

纯 Python，不需要先 build。在这个包的目录里跑（和 `pixi.toml` 里 `test-perception` 的做法一样）：

```bash
cd src/edgeai_mission_interpreter && pixi run python -m pytest -q test
```

也可以走 colcon，只测这个包：

```bash
pixi run bash -c 'colcon build --packages-select edgeai_mission_interpreter && colcon test --packages-select edgeai_mission_interpreter --event-handlers console_cohesion+ ; colcon test-result --verbose'
```

测试只检查每条输出都应该满足的性质。每条输出和测试集期望答案是否一致，由测试集自带的评分器判断。那个评分器在仓库外（`~/edgeai_llm_dev/edgeai_llm_eval`），没有放进仓库。

## 局限

- 规则是对着 60 条测试集 v1 写的，这 60 条是开发集，不能说明没见过的说法会怎样。
- 只认 `T<数字>`、`TC<数字>` 形式的目标 ID；官方目标 ID 格式还没公布。
- 句子结构只覆盖：先后链、「X 完成后」条件、「X 在 Y 之后」关系、「在 X 之前」、并列、按类别选择和排除、优先级档。别的说法大多会当成一组并列目标，或者失败。
