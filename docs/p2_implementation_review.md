# P2 实现记录、要求对照与测试指南

核对日期：2026-11-01。仓库：`/home/zy/edgeai_robot_ws`。
基线提交：`ac9f321`；交接分支 `feature/p2-handoff`，用于开发检查点；是否上传请查看远端分支。

## 1. 结论与 P3 依赖

P2 核心软件已实现，并通过一次独立 Gazebo 世界的完整闭环演练；**尚不能认定项目中 P2 的全部要求均已完成验收**。
当前依据是 211 项全仓测试汇总通过（原有 181 项 + P2 30 项），以及一次有效的单机器人闭环。
没有用空目标 Mock 的成功替代真实感知、救援或实机证据。

P2 与 P3 是并行开发关系，不是“先完成 P3 才能做 P2”。

- P1 → P2：提供 LiDAR、里程计、TF、机器人状态、导航动作和重定位服务。P2 建图、探索主要依赖这些。
- P2 → P3：提供地图坐标系、地图标识/版本、比赛阶段。P3 需在正确观测时刻转换目标坐标，并处理地图回环修正。
- P3 → P2：提供 `/edge/targets` 权威目标注册表及冻结快照；P2 才能提交正式 Task 1 地图与目标结果。自然语言指令原文、模型版本、解析输出也由 P3 提供、P2 保存。
- P4 → P2：提供救援事件及验证后的任务计划；P2 记录证据，向 P4 提供阶段与冻结结果。

开发时使用 `mock_targets:=true`，模拟一份明确为空、版本匹配的目标注册表。正式联调必须关掉替身，连接真实 P3。P2 没有实现 ArUco 检测、LLM 推理或救援分配，这些不属于 P2。

## 2. 已新增的内容

### edgeai_exploration

- `frontier.py`：栅格坐标转换、面积统计、前沿聚类、可达性搜索、候选排序、黑名单和结束条件。
- `coordinator.py`：根据阶段自主下发 P1 导航动作；取消、重试、记录探索状态并请求 HANDOVER。不会直接发底盘速度。
- `ros_grid.py`：OccupancyGrid 转换和地图元数据校验。
- `map_manager.py`：唯一地图管理者、SLAM 子进程生命周期、地图版本、持久化、冻结及重新加载。
- `wait_mapping.py`：实际地图/TF 就绪门控，替代固定启动延时；冻结地图恢复使用 AMCL 启动路径。
- `launch/slam.launch.py`、`config/slam.yaml`：SLAM 生命周期与参数。
- 测试、安装文件、README、LICENSE。

### edgeai_system_manager

- `phases.py`：阶段跳转、请求方权限、急停与恢复规则。
- `manager.py`：阶段服务、心跳、匹配安全确认、累计计时、重置编排、重定位、Task 1 冻结和最终封存。
- `storage.py`：独占运行目录、原子文件写入、Schema 检查、事件去重、路径和损坏检查。
- `bootstrap.py`：参数、组件版本、依赖锁哈希、源文件哈希及源代码归档。代码变更后不能静默复用旧运行。
- `freeze.py`：PGM/YAML/目标文件生成、目标几何检查、原子提交、崩溃恢复、幂等与哈希核验。
- `event_logger.py`：JSONL、任务原文和原始模型输出、救援记录、MCAP 录制、健康状态及封存。
- `audit.py`：只读产物审计及运行包导出。
- `mock_targets.py`：development 专用空目标替身。
- `validate_sim.py`：隔离 DDS 域与 Gazebo 分区的自动闭环演练，保存成功/失败报告，清理自有进程组。
- `launch/task1.launch.py`：统一启动入口。
- 测试、安装文件、README、LICENSE。

未修改 P1/P3/P4 实现、共享消息、Schema 或依赖锁。更多逐文件说明见两个包的 README。

## 3. 对照 P2 的 15 项职责

依据：`docs/team_responsibilities.md` 第 3 节；`docs/development_and_integration_plan.md` 的 P2 部分；`docs/decisions/0001-task1-v1-strategy.md`。

以下“实现完成”仅指当前软件范围，不等于全场景或实机验收。

1. **SLAM 参数、生命周期和地图质量评估：部分完成。** 已实现启动/停止、配置、地图面积统计和仿真检查；缺正式场地、实机、重影/断裂的量化评价与数据集。
2. **Frontier 自主探索：实现并通过单机仿真。** 无人工中间路点。
3. **目标选择、不可达黑名单、重试、结束判断：实现并测试。** 支持无可达前沿、无地图增长、超时；缺更广泛的复杂场景统计。
4. **一台或两台机器人探索协调：单机完成。** 遵循 ADR-0001；robot_2 在线、状态与安全确认的双机真实仿真还未验收。
5. **双机建图对齐/融合：当前版本不适用。** ADR-0001 明确首版单主机器人；不是漏做强制双机融合。未来启用需要新决策及测试。
6. **Task 1 map 坐标系发布权：实现并单机验证。** SLAM 停止后才进入 AMCL 交接；尚未做所有 TF 冲突注入场景。
7. **地图保存、校验、重新加载、版本管理：实现并测试。** 冻结地图重载及晚加入订阅者已测；同轮冻结地图上 AMCL 导航已通过。完整进程冷重启后再次导航还缺独立实测。
8. **面积、未知区域、前沿数、结束原因：实现。** 最终面积保存至 `task1/map_quality.json`，前沿/原因发布到探索状态并记录。未知权威场地面积时覆盖率为 NaN/null；尚未实现覆盖率阈值触发结束和独立长期前沿统计报表。
9. **run_id、阶段、序号、权限：实现并测试。** 包括重复请求、急停、精确安全确认和持久化。
10. **运行模式和官方参数门禁：实现并测试。** 缺必要官方参数时拒绝运动；正式起点坐标与 SLAM 原点对齐待规则明确。
11. **重置及重定位编排：部分完成。** BEGIN/COMPLETE/ABORT、计数、计时、停稳、保留结果及重新定位已测。当前保守全局停车；不支持要求清除地图/目标/救援结果的跨模块重置事务，明确拒绝该策略。实机人工重摆未验收。
12. **Schema 事件/Manifest/版本文件：实现并测试。** 验证错误不会静默跳过。
13. **Task 1 原子冻结：实现并测试。** 地图/目标版本一致、幂等、崩溃恢复、篡改拒绝；真实 P3 冻结联调未完成。
14. **运行目录、版本、JSONL、Rosbag、哈希、导出：实现并实测导出。** 乱序任务证据、原文和版本不一致有回归测试；真实 P3/P4 业务全链路仍待验证。
15. **启动顺序、就绪汇总、冻结停止：实现并单机验证。** 地图/TF 门控、日志/SLAM 故障门控、安全确认已实现；多机完整启动、更多崩溃故障及正式模式全过程待验收。

## 4. 对照 8 条验收条件

1. **无人给中间路点的单机器人探索：通过一次有效独立仿真。**
2. **按配置自动结束：已实现并验证无增长/前沿/时限逻辑。** 实际成功轮次为 `no_map_growth`；覆盖率阈值结束尚未实现。
3. **外墙/障碍完整，无严重重影断裂：部分证据。** 有仿真地图与导航证据，不足以代替正式场地质量验收。
4. **地图重载、定位和导航：分项通过，冷重启整链未验。** 重载的 ROS 测试与同轮 AMCL 导航分别通过，不将二者说成一次冷启动实测。
5. **双机建图或明确第二机器人用途：方案符合 ADR-0001。** 第二机器人仅在线/安全/Task 2 准备；双机在线实测待做。
6. **缺图或错误元数据拒绝 FinalizeTask1：测试通过。**
7. **重复请求/重启不产生冲突身份或 Manifest：核心机制测试通过。** 采用独占锁、持久化和提交恢复；未冻结 SLAM 崩溃需显式新运行，不自动拼接旧地图。尚缺所有节点故障组合测试。
8. **冻结版本不可静默覆盖、在线机器人精确确认：实现并单机验证。** 多机真实确认还待测试。

### 尚未完成的交接

- P1：单机 TF/导航/AMCL 已验证；双机、实机、冷重启和重摆验收待做。
- P3：真实观测时间戳 TF、回环后目标更新、真实冻结快照、模型元数据联调待做。
- P4：任务证据接口有测试；真实救援/自然语言执行全链路待做。
- P5：运行包已可交付；第二人复核及独立回放尚未完成。
- ADR-0001 提到的“连续三次单机成功”尚未达到，不能用一次通过宣称满足。

## 5. 已验证证据与修复记录

有效成功轮次：`run-20261031T220820Z-c1f66909`，development，空目标替身。

- 自主探索 → HANDOVER → 原子冻结 → AMCL → 冻结图返航 → FINISHED → 审计通过。
- 16 个文件、8 条事件；已导出、解包，再次审计通过。
- 已知面积约 24.745 m²、自由面积约 23.495 m²。未计算官方覆盖率。
- 该轮 JSON 报告误将最后一条探索状态 `navigating` 当结束原因；权威阶段事件是 `no_map_growth`。驱动已修正，历史报告保留原样。
- 运行包：`runs/p2-validation/exports/run-20261031T220820Z-c1f66909.tar.gz`。
- SHA-256：`86d9566ae15730990534636aec0e5fb2a73914b8252009d24d41a2f592bdbe7a`。

主要修复：服务 busy 有界重试；导航目标使用最新 TF；冻结地图只发布一次并通过持久 QoS 供后加入节点获取；写盘失败不确认冻结；封存状态与实时安全状态分离；检测日志器/录包/SLAM 故障；地图/TF 就绪启动；Gazebo 独立分区与进程组清理；按 P1 非对称足迹修正净空（路线 0.45 m、目标 0.60 m）。

早期一轮闭环曾通过，但发现旧 Gazebo 服务器残留，已明确撤销其“全新世界验收”资格。
之后隔离演练发生返航阻塞，修正净空后才得到上述有效成功结果。失败日志仍保留。

## 6. 你如何测试

### A. 环境与自动测试

在仓库根目录：

```bash
cd /home/zy/edgeai_robot_ws
pixi install --locked
pixi run build
pixi run validate
pixi run check-env
pixi run test
```

本机已配置好环境，通常无需重新安装。Ubuntu 22.04 是宿主；Pixi 内使用 ROS 2 Jazzy，避免再 source 系统 Humble 环境。
预期：构建成功、契约检查通过、环境 0 error、测试 0 failure。新增测试后总数可能变化，以实际输出为准。

### B. 一键完整自动演练（建议先做）

```bash
cd /home/zy/edgeai_robot_ws
pixi run bash -c 'source install/setup.bash && ros2 run edgeai_system_manager validate_sim --timeout 660'
```

它自动启动独立世界、请求 Task 1、选择探索目标、冻结、切 AMCL、返航、封存并退出。
只跑一轮，不要同时启动另一个域 74 的演练。正常需要数分钟；不应只看机器人走动，应等 `P2 rehearsal passed` 且退出码 0。
`--timeout 660` 约束探索等待，不是整轮总时限。

报告与日志位于 `runs/p2-validation/`，每轮都有独立 run_id。检查报告 `success`、`frozen_map_navigation`、`last_phase=6` 和 audit。
失败时先读同名 `.launch.log` 与 `.validation.json`，不要删除失败证据。

### C. 一边自动测试一边看 RViz

先运行 B；另开终端：

```bash
cd /home/zy/edgeai_robot_ws
pixi shell
source install/setup.bash
export ROS_DOMAIN_ID=74
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
rviz2 -d "$(ros2 pkg prefix --share edgeai_robot_description)/rviz/realistic_robot.rviz" --ros-args -p use_sim_time:=true
```

在 RViz 左侧将 **Global Options → Fixed Frame** 从 `robot_1/odom` 改为 **map**。
勾选 **Map** 显示项（原配置默认关闭），确认 Topic=/map；检查 LaserScan=/robot_1/scan、RobotModel、TF。等待 SLAM 就绪前短暂缺 map 属正常启动过程。
若冻结后才打开 RViz，Map 订阅的 Durability 设为 **Transient Local**，Reliability 为 **Reliable**。
可添加 Path（选择实际存在的 `/robot_1/plan`）查看全局规划路径。
当前没有专门的前沿 Marker 可视化，不能期待 RViz 自动显示所有前沿点。

不要用 RViz 的 2D Goal Pose 工具替代 P2 探索，它不等于项目的导航动作接口。
本指南的 RViz 命令与显示配置已按仓库核对；本次没有人工打开 GUI 检查屏幕效果。

### D. 看 Gazebo 三维场景并手动开始 Task 1

这是一轮独立手动演练，**不要与 B 同时运行**。

```bash
cd /home/zy/edgeai_robot_ws
pixi shell
source install/setup.bash
export ROS_DOMAIN_ID=74
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 launch edgeai_system_manager task1.launch.py run_id:=new mock_targets:=true headless:=false rviz:=true
```

需要本机图形桌面。Gazebo 看真实场景、碰撞和机器人运动；RViz 看 SLAM/TF/导航数据。
第二终端同样进入 Pixi、source 并设置域 74，然后启动比赛：

```bash
P2_RUN_ID=$(cat runs/active_run.txt)
ros2 service call /competition/set_phase edgeai_robot_interfaces/srv/SetCompetitionPhase "{run_id: '$P2_RUN_ID', requested_phase: 1, requester_type: 1, requester_node: operator, reason: start}"
```

手动启动入口会自动探索到 HANDOVER，但不会替操作员完成 FinalizeTask1 和 Task 2 开始操作；完整自动验收用 B。
需要停止运动可在相同环境下请求 ESTOP：

```bash
ros2 service call /competition/set_phase edgeai_robot_interfaces/srv/SetCompetitionPhase "{run_id: '$P2_RUN_ID', requested_phase: 7, requester_type: 1, requester_node: operator, reason: manual_stop}"
```

停止后 Ctrl+C 退出启动器。自动驱动有进程组清理；手动入口退出后若发现 Gazebo 残留，应先识别该轮进程再关闭，不使用广泛 pkill。

### E. 审计、导出与回放

在激活并 source 的仓库终端，用实际 run_id 替换占位符：

```bash
ros2 run edgeai_system_manager audit_run runs/p2-validation/<run_id> --finished
ros2 run edgeai_system_manager audit_run runs/p2-validation/<run_id> --finished --export runs/p2-validation/exports/<run_id>.tar.gz
```

导出目标必须尚不存在。冻结图位于对应运行的 `task1/map.pgm`，可直接作为静态图查看；`map.yaml` 存分辨率和原点。

离线回放请停止实机/仿真控制，并使用单独 DDS 域，例如 76。在 `system/rosbag/segment-...` 找到含 metadata.yaml 的分段目录：

```bash
export ROS_DOMAIN_ID=76
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 bag info /绝对路径/segment-实际编号
ros2 bag play /绝对路径/segment-实际编号 --clock
```

另一个终端同样使用域 76 打开 RViz。回放仅用于查看记录，不同时启动 P2/P1 控制节点，因为录包含阶段和急停话题。
独立回放流程尚未由 P5 或第二人验收。

## 7. 如何将已复核的 P2 合入 main

当前远端：`https://github.com/hzc333/edgeai_robot_ws.git`，交接分支 `feature/p2-handoff`。
按本仓库 CONTRIBUTING.md：现阶段直接提交/push main，不默认走 PR；禁止 force push。
涉及安全、TF、地图、证据的修改，推送前需要至少一名其他成员看过。本次符合这个范围。

建议先让 P1 或 P5 查看代码和本报告，确认“开发基线”可入 main，而不是当作已完成比赛验收的发布版本。
以下为团队默认 main 流程，适用于尚未提交的改动；已在交接分支提交时，不要重复提交，应由复核者先确认分支差异，再将审核后的提交合入 main。远端分支上传不等于 main 合入。

```bash
cd /home/zy/edgeai_robot_ws
git status --short
git fetch origin
git log --oneline HEAD..origin/main

git add README_P2.md src/edgeai_exploration src/edgeai_system_manager docs/p2_development.md docs/p2_implementation_review.md
git diff --cached --stat
git diff --cached --check
git diff --cached
# 检查没有 runs、录包、密钥、.pixi、build、install 或 log

git commit -m "Implement P2 exploration, competition control and evidence persistence"
git pull --rebase origin main
# 若有冲突，逐文件处理后 git add <文件>，再 git rebase --continue；不要覆盖队友代码。
# 不确定如何解决时 git rebase --abort 回到 rebase 前。

pixi run build
pixi run test
pixi run validate
pixi run check-env
git diff --check origin/main
git push origin main
```

如 status 出现其他人的未提交改动，先辨认归属，不要全部 add、stash 或 reset。
认证失败时使用你已有的 GitHub 登录/凭据管理，不把 Token 放进代码或聊天。
远端再次前进导致拒绝 push 时，重新 fetch/rebase/检查，再普通 push，不使用 --force。
推送后查看 GitHub Actions，确认 CI 成功。运行包约 11 MB，已被 runs/ 忽略，不随源码提交；供队友审查时另行交付。

## 8. 下一阶段建议顺序

1. 使用本指南自己跑一轮，结合 RViz 观察并让第二人复核。
2. 连续三次独立单机演练，保留所有报告；补冷重启地图加载后导航和双机在线确认。
3. 接入真实 P3，验证回环目标修正、版本一致的冻结和非空目标产物。
4. 与 P4 演练救援/自然语言任务，检查原文、模型版本、事件和 Manifest。
5. 根据正式规则实现必要的起点对齐及跨模块重置清理，再做实机与 P5 独立验收。
