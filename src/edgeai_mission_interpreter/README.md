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
| `test/test_no_guess.py` | 卡 01b：不认识的约束词一律拒绝（14 条自写说法）、矛盾时拒绝、每条新规则一个例子、包里没有 dev 用例编号 |
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
- 失败只用解释器阶段的两个码：12 指令里没有可结构化的任务内容（闲聊、注入、速度坐标这类底层运动），13 有歧义（没指明目标、条件或备选、「或」选目标、有没用上的约束词，见下一节）。数值从 `MissionValidationIssue.msg` 抄来，测试会对照原文件。
- 输出里不会出现机器人编号、坐标、路径、速度；`notes` 也不会照抄机器人名。

## 不猜（卡 01b）

规则基线看不懂就拒绝，不给可能错的计划。错计划结构合法，验证器拦不住；拒绝了的指令以后交给本地模型。

**怎么判定**：

1. `interpreter.py` 里的 `_CONSTRAINT_WORDS` 是一张约束词表：表示先后（first、then、after、before、last、wait、order、先、后、再、等、最、顺序、同时…）、条件（only、if、require、只有、前提、才…）、优先级（priority、urgent、important、优先、紧急、重要…）、范围（all、only、both、each、these、所有、都、只、其他、两个、它们…）的词。
2. 每条规则在用上一个词时，把这个词在指令里的位置记下来（`_Parser.used`）。只有规则真的按这个词的意思处理了，才算用上：比如「after」只有在被当作条件或先后关系建了边时才算；「in any order」只有在检查过那几个目标之间确实没有先后时才算。
3. 解释完以后，指令里每一个约束词都必须被用上过。只要有一个没用上，就返回失败，问题码 13，`detail` 写出是哪个词，不给步骤。
4. 另外三种自相矛盾也拒绝（码 13）：说了「不分先后」的目标在计划里却有先后；说了「最后」的目标后面还有别的目标；「先 A，」后面跟一个没有先后词的分句（不知道是同一批还是下一批）。

**卡 01b 新增和改动的规则**（都按说法写，不按整句、不按用例编号）：

| 规则 | 对应说法 |
|---|---|
| 主语等别人 | 「X 要等 Y 完成后再救援」「但必须等 Y 救完之后」（没有主语时指前一个分句的目标）「X 要等 Y 和 Z 都救完之后才救援」 |
| 前提 | 「救援 X 的前提是 Y 和 Z 都已完成」 |
| 不带「在」的之前 | 「救援 X 之前必须先救援 Y」 |
| 等待 | 「X and Y (both) have to / must wait for (both) Z」 |
| 指代上一组 | 「两个都」「这三个全部」「全部完成后」「两者」「both」「all three」「those two」 |
| 完成在句末的条件 | 「B 和 C 都完成，救援 D」 |
| 句末条件归前一组 | 「Rescue X last, after A, B and C are done」 |
| 最后 | 「Rescue X last」：X 排在所有别的目标之后；和别的先后矛盾就拒绝 |
| 不分先后 | 「不分先后」「顺序随意」「先后都可以」「不要求顺序」「同时」「in any order」「no order is required」「between A and B, …」「A、B 之间 …」：只检查，不加边 |
| 也 | 「X 也要救援」：只把 X 加进计划，不加先后，不算重复 |
| 只 | 「只需要救援 X 这一个目标」「Only rescue X, nothing else」 |
| 顺序列表 | 「in exactly this order」「in this order: A, B, C」「按 A、B、C 的顺序依次救援」 |
| 开头词 | 「Initially」「after that」「然后再」「最后再」 |
| 列表逗号 | 「Rescue A, B and C …」「after A, B and C are finished」：逗号后面只是接着列目标时不拆开（后面的分句有动词、先后词或优先级词时照常拆） |
| 优先级逐个目标 | 一句里多个目标各有各的档：「A has the highest priority and B has a lower priority」「A 最紧急，B 没那么急」「C 优先级较低，A 和 B 一样重要」 |
| 优先级排序 | 「优先级从高到低是 A、B、C」「Priority from high to low: A, B, C」 |
| 重复 | 同一目标再次出现时，只有带「再…一次」「再次」「again」「twice」这类词才算救两次（交给策略表）；不带的就是同一步，后面的分句只给它加先后 |

优先级的数值：最低的一档是 0，往上每档加 1；没提到的目标和「一样重要」的目标在中间一档。没表达优先级时全部是 0。

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

- 规则是对着 60 条难例和 420 条 dev 写的，这两份都是开发集；dev 上的分数不能说明没见过的说法会怎样。held-out 由另一个对话去跑。
- 只认 `T<数字>`、`TC<数字>` 形式的目标 ID；官方目标 ID 格式还没公布。
- 「不猜」只管约束词表里的词。一句话如果用表外的词表达先后或优先级（比如「T1 更 pressing」），解释器看不出来，可能给出缺约束的计划。词表要随新说法补。
- 拒绝会比卡 01 多。拒绝了的指令以后由本地模型来接。
