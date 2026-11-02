# P2 交接 README：自主探索、比赛控制与运行证据

更新日期：2026-11-01。

本文是接手 P2 的阅读入口。先阅读“当前状态”，然后按“首次运行”复现；需要修改实现时，再看代码导航和模块交接约定。
所有命令默认在仓库根目录执行。本机目录为 `/home/zy/edgeai_robot_ws`，其他机器使用自己的克隆路径。

## 1. 当前交付状态

**P2 核心软件已实现，单机器人开发仿真闭环已通过；正式比赛和实机验收尚未完成。**

已验证：

- 全仓测试汇总 **211 项通过，0 错误、0 失败**，其中 P2 新增 30 项。
- Pixi 构建、共享契约校验通过；环境检查 0 错误、0 警告。
- 一次有效的独立 Gazebo 世界演练：自主探索 → 停稳 → 地图/目标原子冻结 → AMCL 重定位 → 冻结地图返航 → FINISHED → 日志封存与审计。
- 运行包已导出，解包后重新审计通过。

上述演练使用 **development 空目标替身**，不是实际 ArUco 检测或救援验收。没有连续三次独立成功、多机完整联调、真实 P3/P4 全链路或实机成功的证据。

交接工作整理在 `feature/p2-handoff` 分支，作为开发检查点提交；是否已上传以远端分支为准。该分支不是正式比赛发布版本。开发基线为 `ac9f321`；代码未修改 P1/P3/P4 包、共享接口或 Pixi 锁文件。

## 2. P2 负责什么

新增两个 ROS 2 Python 包：

- `src/edgeai_exploration/`：SLAM、地图管理、Frontier 自主探索。
- `src/edgeai_system_manager/`：比赛阶段、重置/重定位编排、原子冻结、事件日志、录包、审计和导出。

P2 不实现底盘控制、ArUco 检测、LLM 推理或救援调度。

按 ADR-0001，**robot_1 是唯一建图机器人**；robot_2 只为状态、安全和 Task 2 做准备，不参与首版权威建图。双机地图融合不在当前决策范围内。

### 与其他模块的关系

- **P1 提供** scan、odom、TF、机器人状态、导航动作、重定位服务和底盘安全。P2 的建图/探索主要依赖 P1。
- **P3 提供**权威目标注册表及冻结快照，以及自然语言原文、模型元数据和解析输出。P2 与 P3 可以并行开发，但正式冻结和业务联调必须接入真实 P3。
- **P4 提供**验证后的任务计划、救援事件。P2 提供阶段与地图身份并统一记录证据。
- **P5 负责**第二人复核、发布检查与独立回放验收。

## 3. 首次运行：先复现自动闭环

### 环境

宿主为 Ubuntu 22.04；仓库锁定的 Pixi 环境内使用 **ROS 2 Jazzy + Gazebo Harmonic**。
不要在同一终端混入系统 ROS Humble 的 setup.bash。Pixi 已安装时执行：

```bash
cd /path/to/edgeai_robot_ws
pixi install --locked
pixi run build
pixi run validate
pixi run check-env
pixi run test
```

若本机环境已经部署好，通常可直接从 build 开始。测试以实际的失败数为准，后续新增测试会改变总数。

### 自动端到端演练

```bash
pixi run bash -c 'source install/setup.bash && ros2 run edgeai_system_manager validate_sim --timeout 660'
```

该命令会创建新 run_id，启动 Gazebo、SLAM、Nav2、P2 和空目标替身，自动开始 Task 1，并完成冻结、AMCL、返航与封存。不需要人工给中间路点。

成功标准：

1. 命令输出 `P2 rehearsal passed`，进程退出码为 0。
2. `runs/p2-validation/<run_id>.validation.json` 的 `success=true`、`frozen_map_navigation=true`、`last_phase=6`。
3. 同一报告内存在通过的 audit 结果。

默认 DDS 域为 **74**，只在本机发现；Gazebo 分区为 `p2-<run_id>`。不要同时运行另一轮域 74 演练。`--timeout 660` 是探索等待上限，不是整轮总时限。

失败报告和 `.launch.log` 位于 `runs/p2-validation/`。保留失败证据，不因机器人能走动就判定测试成功。

## 4. 可视化与手动操作

### 自动演练期间看 RViz

启动上一节自动演练后，另开终端：

```bash
cd /path/to/edgeai_robot_ws
pixi shell
source install/setup.bash
export ROS_DOMAIN_ID=74
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
rviz2 -d "$(ros2 pkg prefix --share edgeai_robot_description)/rviz/realistic_robot.rviz" --ros-args -p use_sim_time:=true
```

默认 RViz 配置需要手动调整：

- Global Options → Fixed Frame 改为 `map`。
- 勾选 Map，Topic 为 `/map`；默认配置中 Map 是关闭的。
- Map 的 Durability Policy 设为 **Transient Local**，Reliability 设为 **Reliable**，这样冻结后再打开也能收到地图。
- 检查 `/robot_1/scan`、RobotModel 和 TF；可按实际话题添加 `/robot_1/plan` 的 Path。

当前没有专用前沿 Marker 显示。不要用 RViz 的 2D Goal Pose 工具替代 P2 的导航动作接口。
GUI 操作步骤已核对仓库配置，本轮没有人工打开窗口验收显示效果。

### 单独启动可视化手动演练

**此方式不要与自动演练同时运行。** 在本机图形桌面、已激活 Pixi 和设置域 74 的终端中：

```bash
ros2 launch edgeai_system_manager task1.launch.py run_id:=new mock_targets:=true headless:=false rviz:=true
```

另一个同样配置环境的终端发起 Task 1：

```bash
P2_RUN_ID=$(cat runs/active_run.txt)
ros2 service call /competition/set_phase edgeai_robot_interfaces/srv/SetCompetitionPhase "{run_id: '$P2_RUN_ID', requested_phase: 1, requester_type: 1, requester_node: operator, reason: start}"
```

手动入口自动探索到 HANDOVER，但完整冻结提交及 Task 2 开始还需服务调用。首次接手建议先使用自动演练；手动调用字段见 `docs/p2_implementation_review.md` 及接口定义。

需要停车时，通过同一服务请求 ESTOP：

```bash
ros2 service call /competition/set_phase edgeai_robot_interfaces/srv/SetCompetitionPhase "{run_id: '$P2_RUN_ID', requested_phase: 7, requester_type: 1, requester_node: operator, reason: manual_stop}"
```

急停解除不会自动恢复运动。停稳后 Ctrl+C 关闭启动器；自动验收驱动会清理自己的进程组。手动启动若留下 Gazebo 子进程，应先确认该轮 PID 再关闭，不要广泛 pkill。

## 5. 运行流程与必须保留的约定

开发闭环大致如下：

```text
P1 scan / odom / TF
        ↓
SLAM → map_manager → /map → Frontier → P1 导航动作
                          ↓
TASK1 → HANDOVER → 停稳 / 停止 SLAM / 冻结地图
                          ↓
P3 匹配版本的冻结目标 → FinalizeTask1 → manifest + Task 1 文件
                          ↓
操作员请求 TASK2A → AMCL 重定位 → P4 任务
                          ↓
FINISHED → 封存 JSONL / MCAP / 任务证据 → 审计 / 导出
```

- `/competition/state` 和 `/competition/estop` 由 system_manager 发布，其他模块调用服务，不伪造权威状态。
- P2 不直接发布 `/robot_N/cmd_vel`；导航交给 P1。
- SLAM 与 AMCL 不能同时拥有 robot_1 的 `map → odom` 变换。
- 冻结需要 `run_id / map_id / map_revision / target_revision` 一致；目标数为零也必须有权威空快照。
- 所有配置机器人必须返回新鲜、匹配阶段序号的安全确认。
- 心跳默认 5 Hz，为 P1 当前 1 秒失联阈值保留抖动余量。
- 路线净空默认 0.45 m，目标再加 0.15 m。该设置考虑 P1 非对称足迹、约 0.429 m 的停车区旋转半径和到达误差；更换机器人尺寸时需要重新核对。
- 冻结图通过持久 QoS 发布一次；不要恢复成每秒重发冻结图，否则 AMCL 会反复处理地图。

## 6. 关键接口：联调前先核对

以共享 `.msg/.srv/.action` 定义和 `config/interface_runtime.yaml` 为准，不在消费方自行扩展字段。

P2 对外服务：

- `/competition/set_phase`：`SetCompetitionPhase`，操作员/系统阶段请求。
- `/competition/request_reset`：`RequestReset`，BEGIN / COMPLETE / ABORT。
- `/edge/finalize_task1`：`FinalizeTask1`，提交精确版本的地图和目标。

P2 消费：

- `/robot_N/state`、`/robot_N/phase_ack`；`/robot_1/scan` 供 SLAM 使用。
- `/edge/targets`：P3 的 `TargetArray`。
- `/edge/mission_intake`、`/edge/mission_plan`、`/edge/rescue_events`：业务证据。
- `/robot_1/execute_navigation_goal` 动作、`/robot_N/relocalize` 服务。

P2 发布：

- `/map`、`/competition/state`、`/competition/estop`、`/edge/exploration_status`。

内部协作端点：`/edge/slam_map`、`/edge/map_metadata`、`/edge/map_manager/freeze`、`/edge/logger_status`、`/edge/event_logger/seal`。其中 JSON String 状态只供 P2 内部使用，不替代公共接口。

接真实 P3 时设置 `mock_targets:=false`，确保没有替身与真实节点同时发布。P3 要确认观测时间戳 TF、回环后目标修正、地图版本切换和 `task1_frozen` 快照的生产时机。

## 7. 改代码时先看哪些文件

探索与地图：

- `frontier.py`：纯算法、栅格几何、候选、统计、黑名单与结束判断。
- `coordinator.py`：ROS 输入、导航动作和探索生命周期。
- `map_manager.py`：SLAM 进程、唯一地图、版本、冻结和恢复。
- `ros_grid.py`：地图消息与内部栅格转换。
- `wait_mapping.py`、`launch/slam.launch.py`、`config/slam.yaml`：启动就绪与 SLAM 配置。

比赛管理与证据：

- `phases.py`：阶段策略；`manager.py`：ROS 服务、就绪、计时、急停与重定位编排。
- `storage.py`、`bootstrap.py`：独占目录、原子写、Schema、参数与版本身份。
- `freeze.py`：地图/目标冻结事务与崩溃恢复。
- `event_logger.py`：JSONL、任务原文、模型输出、MCAP、健康状态与封存。
- `audit.py`：产物核验和导出。
- `validate_sim.py`、`mock_targets.py`：开发验收工具，不是真实感知实现。
- `launch/task1.launch.py`：集成启动入口。

两个包的 `test/` 包含算法、存储、阶段、ROS 服务、重置、地图重载、任务证据及风格测试。逐文件说明见各包 README。

## 8. 运行目录与恢复

手动启动默认 `runs/<run_id>/`，自动验收默认 `runs/p2-validation/<run_id>/`。

```text
<run_id>/
  manifest.json                文件清单、版本、哈希
  events.jsonl                 去重后的事件日志
  task1/                       map.pgm / map.yaml / targets.json / 元数据 / 面积
  task2b/                      指令原文、模型输出、任务计划和接收记录
  system/
    parameters.yaml            实际解析参数
    component_versions.json    组件与依赖版本
    source_hashes.json         源文件哈希
    source_snapshot.tar.gz     运行时源码
    map_snapshot.json          持久地图快照
    state.json                 当前控制状态
    final_state.json           不可变最终状态证据
    sealed.json                封存文件记录
    rosbag/                    MCAP 分段及日志
```

- `run_id:=new` 明确新建；省略 run_id 会尝试使用对应 runs_root 的 `active_run.txt`。
- 改过源码或参数后使用新运行，不能把新代码静默混入旧运行。
- 正在运动时管理器重启会进入 ESTOP；未冻结 SLAM 会话不自动从头覆盖旧地图，需要显式新运行。
- 冻结地图可以恢复，但冷重启后完整定位/导航尚未独立验收。
- 文件哈希不匹配时停止并查因，不手改 Manifest 让检查通过。

在已激活环境的仓库终端中审计成功轮次：

```bash
ros2 run edgeai_system_manager audit_run runs/p2-validation/run-20261031T220820Z-c1f66909 --finished
```

新机器需要另行取得运行包并解压到同名 run_id 目录，Git 克隆不含这些数据。对新的运行导出时：

```bash
ros2 run edgeai_system_manager audit_run runs/p2-validation/<run_id> --finished --export runs/p2-validation/exports/<run_id>.tar.gz
```

替换 `<run_id>`；目标文件必须不存在。回放请另用 DDS 域，例如 76，只运行 rosbag/RViz，不同时启动控制节点；录包中含阶段与急停消息。

## 9. 已有证据与排障入口

有效演练：`run-20261031T220820Z-c1f66909`。

- 16 个文件、8 条事件核验通过。
- 结束原因以最终阶段事件为准：`no_map_growth`。
- 已知面积约 24.745 m²，自由面积约 23.495 m²。没有权威总面积，覆盖率保留 NaN/null，不据此宣称覆盖率达到某百分比。
- 导出包 SHA-256：`86d9566ae15730990534636aec0e5fb2a73914b8252009d24d41a2f592bdbe7a`。
- 该轮旧 JSON 报告误采到 `navigating`；采样代码已修正，历史证据没有重写。

常见问题：

- **看到 PREPARATION 但不能开始**：查看响应中的缺失官方参数、机器人状态、地图或日志器就绪原因。
- **HANDOVER 卡住**：检查安全确认的 run_id/序号、地图冻结状态，以及 P3 是否提供匹配版本的冻结快照。
- **返回 busy**：长操作正在进行；等待并有界重试，不直接修改状态或重复启动管理器。
- **RViz 看不到地图**：检查 DDS 域、use_sim_time、Map 勾选、map 固定坐标系与 Transient Local。
- **Nav2 error 104 / 无进展**：结合地图、机器人姿态、碰撞保护区与导航日志检查。不要关闭碰撞监视器来让验收通过。
- **旧世界或重复时钟**：检查 GZ_PARTITION、对应启动日志及本轮 PID。早期因残留 Gazebo 产生的“成功”已撤销验收资格。
- **源代码变化或哈希冲突**：先保留旧运行证据，再显式新建运行。

## 10. 明确未完成的工作

接手后建议按顺序推进：

1. 自己复现自动演练，并与另一位成员观察 RViz；补 GUI 显示检查。
2. 连续三次独立单机成功；补全进程冷重启后地图加载、定位和导航；完成 robot_2 在线和匹配安全确认测试。
3. 连接真实 P3，验证非空目标、回环修正及原子冻结。
4. 连接 P4，验证真实救援和自然语言任务证据；交 P5 独立回放。
5. 用正式规则补齐场地/起点坐标对齐、重置策略，再进行实机重摆和地图质量验收。

当前限制：

- 不支持要求清除地图、目标、已完成救援的跨模块重置事务；遇到这种正式策略会拒绝。
- 重置保守暂停整组机器人，尚非独立单机器人停车。
- 覆盖率阈值触发结束尚未实现；当前按前沿、地图增长与时间判断。
- 尚无地图重影/断裂的系统量化评估；不保证任意未知场景成功。
- 双机地图融合当前不适用，不能在未更新 ADR 的情况下直接开启第二权威建图源。

完整要求逐条对照见 `docs/p2_implementation_review.md`；不要把本节未完成项当作已验收能力。

## 11. 提交与交接清单

本仓库当前规定直接提交/push `main`，不默认走 PR，禁止 force push。
涉及安全、TF、地图和证据的改动在推送前需至少一名其他成员看过（见 CONTRIBUTING.md）。

- [ ] 接手人读完本文和两个包的 README。
- [ ] 本地 build / test / validate / check-env 通过。
- [ ] 已复现独立自动演练并保存 run_id。
- [ ] 队友已复核代码、已知限制和证据。
- [ ] 确认暂存内容只有 P2 代码、README 与文档。
- [ ] 同步 origin/main，处理冲突后重新检查，再普通 push。
- [ ] 推送后确认 GitHub Actions 成功。

建议明确暂存：

```bash
git add README_P2.md src/edgeai_exploration src/edgeai_system_manager docs/p2_development.md docs/p2_implementation_review.md
git diff --cached --stat
git diff --cached --check
```

不要暂存 `.pixi/`、build/install/log、运行包或凭据；`runs/` 已被 Git 忽略。完整 commit/rebase/push 流程见 `docs/p2_implementation_review.md` 第 7 节。开发检查点可先在独立分支保存；远端上传及 main 合入应分别确认，不把分支检查点视为真人验收。

## 12. 延伸阅读

- `docs/p2_implementation_review.md`：15 项职责、8 条验收条件逐项核对，完整测试与 Git 步骤。
- `docs/p2_development.md`：演练记录、失败原因与修复证据。
- `src/edgeai_exploration/README.md`：算法和地图参数。
- `src/edgeai_system_manager/README.md`：控制面、持久化和服务约定。
- `docs/team_responsibilities.md`：团队职责及验收标准。
- `docs/decisions/0001-task1-v1-strategy.md`：单主机器人建图决策。
- `config/interface_runtime.yaml`、`src/edgeai_robot_interfaces/`、`schemas/`：联调契约。
- `CONTRIBUTING.md`：提交、第二人复核和发布规则。
