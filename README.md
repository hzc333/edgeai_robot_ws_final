# EdgeAI 多机器人自主探索与救援

面向 KTH EdgeAI Robotics Challenge 的 ROS 2 Jazzy 工作空间。两台差速驱动机器人在 Gazebo Harmonic 中完成一条全自主的中期检查流程：

> 自主探索建图 → 视觉检测 S1、C1 → robot_1 回起点 → 冻结地图和 Task 1 结果 → 两台机器人重新定位 → 双机自动救援 S1、C1 → 冻结 → 结束并审计

整个流程无需人工驾驶。操作员只负责切换比赛阶段，这一步也可以交给演练脚本自动完成。

```bash
pixi install --locked && pixi run build   # 安装依赖并构建
pixi run midterm-gui                      # 运行完整中期流程（打开 Gazebo 和 RViz）
```

---

## 目录

1. [项目简介](#1-项目简介)
2. [系统架构](#2-系统架构)
3. [系统要求](#3-系统要求)
4. [安装与构建](#4-安装与构建)
5. [运行中期全流程](#5-运行中期全流程)
6. [中期流程与场地说明](#6-中期流程与场地说明)
7. [其他运行方式](#7-其他运行方式)
8. [自动化测试](#8-自动化测试)
9. [复现与问题反馈](#9-复现与问题反馈)
10. [常见问题排查](#10-常见问题排查)
11. [已知限制](#11-已知限制)
12. [仓库结构](#12-仓库结构)
13. [文档索引](#13-文档索引)

---

## 1. 项目简介

### 1.1 中期检查任务

5 m × 5 m 场地里有两个 ArUco 目标：**S1**（ID 1，普通目标，单机救援）和 **C1**（ID 2，协同目标，双机救援）。要求在 13 分钟内完成：

- **Part 1**：robot_1 从起始区出发，自主探索并实时 SLAM 建图，检测到 S1 和 C1，并记录 ID、类型、相对位置和时间。这期间 robot_2 停在起始区。
- **Part 2**：两台机器人在地图上定位，按目标类型自动分配机器人。判定条件：S1 有 1 台、C1 有 2 台机器人同时位于目标半径内（S1 0.5 m，C1 0.7 m），连续停留 10 秒（仿真时间）。全程不能碰撞。

完整比赛规则草案见 [docs/reference/](docs/reference/)。

### 1.2 主要功能

| 模块 | 功能 | ROS 包 |
|---|---|---|
| 机器人平台 | 仿真模型（LiDAR、RGB-D 相机、IMU）、多机器人启动、`robot_agent`（状态、导航和救援动作）、速度安全看门狗 | `edgeai_robot_description`、`edgeai_robot_bringup` |
| 定位与导航 | 每台机器人的 Nav2（RPP 控制器、碰撞监视器）、AMCL 和重新定位服务、对方机器人足迹避让 | `edgeai_robot_navigation` |
| 探索与比赛控制 | SLAM Toolbox 地图管理、前沿探索加标记补看点、比赛阶段状态机、地图冻结、重新定位编排、事件日志、录包与审计 | `edgeai_exploration`、`edgeai_system_manager` |
| 视觉感知 | ArUco 检测与 RGB-D 位姿估计、多机器人目标融合、Task 1 目标快照、检测证据（带标注截图） | `edgeai_target_perception` |
| 救援协调 | 任务分配、双机协同、10 s 连续停留判定、Gazebo 物理接触检测插件、救援证据 | `edgeai_task_coordinator` |
| 公共设施 | 共享 msg/srv/action、配置解析、QoS 工具、Mock 节点 | `edgeai_robot_interfaces`、`edgeai_common`、`edgeai_system_tests` |

模块之间只通过 `edgeai_robot_interfaces` 中的话题、服务和动作通信。各包的详细说明写在各自的 `src/<包名>/README.md`。

### 1.3 当前状态（仿真）

- 最后连续 4 轮完整演练全部通过：每轮总时长 338～362 s（约 6 分钟，远低于 13 分钟），S1/C1 的检测位置误差 0.5～8 cm，审计通过，物理接触 0 次。
- 典型一轮：Task 1 约 156 s，交接约 11 s，Task 2A 约 171 s。
- 自动化测试：colcon 217 项，加上感知包的 77 项，全部通过。
- 尚未验证：真实机器人和真实传感器。

---

## 2. 系统架构

```text
                   ┌──────────────────────── 边缘端（/edge） ─────────────────────────┐
                   │  system_manager ── /competition/state（阶段、急停、地图版本）      │
                   │      │  relocalize / 冻结 / 审计                                  │
                   │  map_manager ── /map（SLAM 地图 → 冻结地图）                      │
                   │  exploration_coordinator ── 探索目标、补看点、回起点              │
                   │  target_fusion ── /edge/targets（Task 1 目标与冻结快照）          │
                   │  task_allocator / rescue_verifier ── S1、C1 救援派发与停留判定    │
                   └────────▲───────────────────────────────▲─────────────────────────┘
       execute_navigation_goal / execute_rescue_task       target_observations
                            │                               │
   ┌──────────── /robot_N（每台机器人） ──────────────────────┴──────────────┐
   │  robot_agent → Nav2（规划、RPP、碰撞监视器）→ safety_watchdog → cmd_vel │
   │  AMCL / localization（relocalize 服务）   peer_footprints（避让对方）    │
   │  target_detector（ArUco + 深度）          scan / odom / camera / imu    │
   └────────────────────────────────▲────────────────────────────────────────┘
                                    │ ros_gz_bridge
                      Gazebo Harmonic：中期场地、两台机器人、物理接触检测插件
```

- 每台机器人的话题和坐标系都带 `robot_N` 前缀（[ADR-0002](docs/decisions/0002-multi-robot-tf-and-naming.md)）。`map` 是共享的场地坐标系。
- `/robot_N/cmd_vel` 只由安全看门狗发布：阶段不允许、急停或输入超时时一律输出零速度（[ADR-0003](docs/decisions/0003-velocity-safety-chain.md)）。
- 接口、QoS、状态机和持久化格式的完整定义见 [docs/system_architecture_and_interfaces.md](docs/system_architecture_and_interfaces.md)。

---

## 3. 系统要求

| 项目 | 要求 |
|---|---|
| 操作系统 | **Ubuntu 24.04 x86-64**（在 24.04.5 上开发验证；Ubuntu 22.04 上的 Pixi 环境也运行过） |
| CPU / 内存 | 建议 8 核以上、16 GB 内存。两台机器人加 Gazebo 很吃 CPU，负载过高时仿真会变慢，部分墙钟超时可能误触急停 |
| 显卡 | 需要能正常运行 OpenGL 的桌面环境。即使无界面运行，Gazebo 的相机传感器也要渲染 |
| 磁盘 | Pixi 环境约 **6 GB**，构建产物约 70 MB，每轮演练输出约 80 MB（在 `runs/` 下） |
| 网络 | 首次 `pixi install` 需要下载 ROS 2 Jazzy、Gazebo Harmonic、Nav2 等 |

**不需要**事先安装系统 ROS 或 Gazebo：所有依赖（ROS 2 Jazzy、Gazebo Harmonic、Nav2、SLAM Toolbox、OpenCV 4.12、C++ 编译器等）都由 [Pixi](https://pixi.sh) 按 `pixi.lock` 安装到仓库内的 `.pixi/`，不影响系统的其他环境。

---

## 4. 安装与构建

### 4.1 安装 Pixi（每台电脑一次）

```bash
curl -fsSL https://pixi.sh/install.sh | bash
```

安装后重新打开终端（安装脚本会把 `~/.pixi/bin` 加进 `~/.bashrc`），确认可用：

```bash
pixi --version
```

开发使用的是 pixi 0.83.0；更新的版本一般也可以。

### 4.2 获取代码并安装依赖

```bash
git clone <仓库地址> edgeai_robot_ws
cd edgeai_robot_ws
pixi install --locked
```

`--locked` 会严格按 `pixi.lock` 安装，保证每台电脑的版本完全一致。首次安装需要几分钟到十几分钟。

### 4.3 构建

```bash
pixi run build
```

这会执行 `colcon build --symlink-install`，构建全部 10 个包，包括 Gazebo 物理接触检测插件（C++，用 Pixi 自带的编译器）。

### 4.4 检查环境（推荐）

```bash
pixi run check-env
pixi run validate
```

`check-env` 检查 ROS 包、OpenCV 版本、DDS 等是否和标准环境一致；`validate` 检查共享接口、Schema 和配置文件。两者都应该是 0 错误。

### 4.5 注意事项

- **所有命令都在仓库根目录执行。** 演练脚本用当前目录查找 `schemas/` 和 `config/`。
- **`~/.bashrc` 里有 `source /opt/ros/...` 也没关系。** Pixi 激活时 `tools/pixi_activate.sh` 会自动去掉系统 ROS 的路径；但不要在同一个终端里先 `pixi shell`、再手动 `source /opt/ros/jazzy/setup.bash`。
- `config/` 目录在运行时直接读取，修改后不需要重新构建；修改 `src/` 下的配置、launch 或世界文件后，要重新 `pixi run build`，否则运行时用的还是 `install/` 里的旧文件。
- 如果曾经用系统 ROS 构建过，先删除 `build/ install/ log/` 再用 Pixi 重新构建。

---

## 5. 运行中期全流程

### 5.1 一条命令运行

```bash
# 无界面（推荐用于计时和统计成功率）
pixi run midterm

# 打开 Gazebo 和 RViz（便于观察和展示）
pixi run midterm-gui
```

这两个任务等价于：

```bash
pixi run bash -c 'source install/setup.bash && python tools/midterm_rehearsal.py [--gui]'
```

需要额外参数时用这条完整命令：

| 参数 | 默认值 | 作用 |
|---|---|---|
| `--gui` | 关 | 打开 Gazebo 和 RViz |
| `--domain` | 77 | 隔离用的 `ROS_DOMAIN_ID` |
| `--runs-root` | `runs/midterm-rehearsal` | 输出目录 |
| `--task1-timeout` | 780 | Task 1 超时（墙钟秒） |
| `--rescue-timeout` | 420 | Task 2A 超时（墙钟秒） |
| `--debug-on-fail` | 关 | 失败时在关闭前抓取 robot_2 的 AMCL 和 TF 现场证据 |

**运行前请确认：**

1. 系统负载不高：`uptime` 显示的 1 分钟负载最好低于 6，并且关掉其他占 CPU 的程序。
2. 没有残留的仿真进程：`pgrep -af "gz sim"` 没有输出。有残留时见第 10 节。

一轮大约 6～12 分钟（取决于电脑性能）。脚本只扮演“操作员”：开始 TASK1、在 HANDOVER 后定稿 Task 1、进入 TASK2A、结束后进入 FINISHED 并审计。其余全部由系统自主完成。按 Ctrl+C 可以中止，脚本会自动关闭仿真。

### 5.2 终端输出

脚本每一步打印一行 JSON，正常的一轮大致是：

```text
{"step": "ready; starting TASK1 (autonomous exploration)"}
{"step": "TASK1 ended", "reason": "...", "seconds": 156.1}
{"step": "Task 1 targets", "targets": {"S1": {"xy": [1.043, 4.133], "error_m": 0.054}, "C1": {...}}}
{"step": "TASK2A: P4 rescue", "handover_seconds": 11.3}
{"step": "P4", "status": "...", "target": "S1"}
...
{"step": "FROZEN", "p4_done": true, "seconds": 170.7, "total_seconds": 338.1}
passed=True; report in .../runs/midterm-rehearsal/<run_id>.rehearsal.json
```

（输出中的 `P4` 指救援协调模块。）

**成功的标准：** 最后一行是 `passed=True`（退出码 0）。这要求同时满足：两个目标都检测到、救援协调报告 `DONE`、审计通过。

### 5.3 输出文件

都在 `runs/midterm-rehearsal/` 下（`runs/` 不进 git）：

| 路径 | 内容 |
|---|---|
| `<run_id>.rehearsal.json` | **本轮结果摘要**：`success`、各阶段耗时 `times_s`、检测结果和误差 `task1_targets`、救援状态序列 `p4_status_sequence`，失败时还有 `error`、`robots`、`p4_last_status` |
| `<run_id>.launch.log` | 所有节点的完整日志，排查失败时用 |
| `<run_id>/task1/` | 冻结的 SLAM 地图（`map.pgm`、`map.yaml`）和 Task 1 目标列表 `targets.json` |
| `<run_id>/detections/<run_id>/` | 检测证据：`detections.jsonl`（每次检测的 ID、类型、相机坐标系位置、时间）、带标注的截图 `*.png`、`summary.json` |
| `<run_id>/p4/<id>/` | 救援证据：`events.jsonl`、`distances.jsonl`（每台机器人到目标中心的距离历史）、`snapshot.json` |
| `<run_id>/events.jsonl`、`manifest.json` | 阶段事件和运行清单 |
| `<run_id>/system/` | 参数、源码快照、SLAM 日志、rosbag 录包等 |

---

## 6. 中期流程与场地说明

### 6.1 场地与坐标

- 场地 5 × 5 m，**原点在左下角**，x 朝东、y 朝北。世界文件是 `src/edgeai_task_coordinator/test_assets/midterm.sdf`，预建地图在同一目录。
- **S1**：中心 (1.0, 4.1)，半径 0.5 m，标记朝南。**C1**：中心 (4.1, 1.0)，半径 0.7 m，标记朝北。标记边长 0.16 m（DICT_4X4_50），中心离地约 0.35 m；仿真相机因此上仰 15°。
- **起点**：robot_1 (0.50, 0.5)，robot_2 (1.20, 0.5)，都朝北（`config/sim/robots_midterm.yaml`，与 `config/competition_parameters.yaml` 的 `midterm_robot_1/2` 一致）。修改起点时，还要同步修改 `src/edgeai_robot_navigation/tools/midterm_switch_check.py` 和 `src/edgeai_task_coordinator/tools/run_simulation.py` 里的副本。
- SLAM 地图从一开始就对齐场地坐标（`map` = 场地坐标系），所以 Part 1 的地图和 Part 2 的目标坐标可以直接对应。

### 6.2 阶段流程（`system_manager` 控制）

| 阶段 | 发生了什么 |
|---|---|
| PREPARATION | robot_1 为 SLAM 模式；robot_2 停放（它在地图中的位置照常发布，robot_1 能避开它） |
| TASK1 | 前沿探索 → 按 `config/sim/midterm_marker_viewpoints.yaml` 依次走过补看点，看还没检测到的标记 → robot_1 回起点 → 请求 HANDOVER |
| HANDOVER | 停止 SLAM 并冻结地图（清掉停放的 robot_2）；感知模块发布冻结的 Task 1 目标快照；操作员调用 `/edge/finalize_task1` |
| TASK2A | 两台机器人重新定位（AMCL）；救援协调取地图和目标，先救援 S1，再救援 C1，完成后请求 FROZEN |
| FINISHED | 封存证据并审计 |

系统入口是：

```bash
ros2 launch edgeai_system_manager task1.launch.py scenario:=midterm run_id:=new
```

演练脚本就是用它启动整个系统的。直接运行时需要自己调用 `/competition/set_phase` 服务切换阶段（见 `edgeai_system_manager` 的 README），所以一般用演练脚本。

### 6.3 救援停车点（`src/edgeai_task_coordinator/config/midterm.yaml`）

| 目标 | 停车点 | 接近点 | 进入方式 |
|---|---|---|---|
| S1 | (1.15, 4.48) | (1.60, 4.48)，朝东 | 在支架东侧掉头，倒车 0.45 m 进入；离开时直接向东开走 |
| C1 第 1 台 | (4.45, 1.45) | (4.45, 2.20)，朝南 | 直线前进 0.75 m |
| C1 第 2 台 | (3.90, 1.50) | (3.95, 2.30) | 直线前进 0.80 m |

几何约束：机器人尾部伸出 0.315 m，原地转身需要约 **0.41 m** 的空间（含碰撞监视器的停车区）。因此所有原地转身都安排在开阔处，靠近停车点只走直线。

---

## 7. 其他运行方式

下面的命令都需要先进入环境：`pixi shell`，然后 `source install/setup.bash`；也可以用 `pixi run bash -c 'source install/setup.bash && <命令>'`。

| 用途 | 命令 | 说明 |
|---|---|---|
| 只启动中期场地和两台机器人 | `ros2 launch edgeai_robot_bringup midterm_sim.launch.py part:=1`（或 `part:=2`） | Part 1：robot_1 建图、robot_2 停放；Part 2：预建地图加 AMCL |
| Part 1 → Part 2 切换检查 | `python src/edgeai_robot_navigation/tools/midterm_switch_check.py [--gui] [--rescue]` | 计时切换过程；`--rescue` 时接着完成完整救援 |
| 双机导航回归 | `python3 src/edgeai_robot_navigation/tools/midterm_nav_check.py --output runs/nav-cross --scenario cross --rounds 6` | 双机交叉或对向行驶，统计间隙和误差 |
| Part 2 单独演练 | `python src/edgeai_task_coordinator/tools/run_simulation.py --start-layout midterm --output runs/part2-check`，然后 `python src/edgeai_task_coordinator/tools/audit_simulation.py runs/part2-check` | 自带阶段控制和审计 |
| 检测证据记录 | `ros2 launch edgeai_target_perception midterm_detection.launch.py robot_id:=robot_1 output_dir:=runs/detections` | 现场展示：`ros2 topic echo /edge/detection_summary --once` |
| 开发仿真（空场地、手动驾驶） | `./start_sim.sh [机器人数量]` | 见 [docs/simulation_basics.md](docs/simulation_basics.md) |

注意：切换检查、导航回归和 Part 2 单独演练这几个工具使用平台自带的 SLAM 和预建地图，地图坐标和中期全流程的不同（这些工具的 SLAM 原点在 robot_1 起点），不要混用它们的地图。

---

## 8. 自动化测试

```bash
pixi run test             # 全部 colcon 测试（按顺序执行，使用隔离的 DDS 域 42）
pixi run test-perception  # 感知包测试（pytest）
pixi run validate         # 接口和配置契约检查
```

- `pixi run test` 必须按顺序执行（任务里已设置 `--executor sequential`），否则测试之间会通过 DDS 互相干扰。运行测试时不要同时运行仿真。
- 测试数量以失败数为准，新增测试会改变总数（当前 217 项加感知包的 77 项）。
- GitHub Actions（`.github/workflows/contracts.yml`）在推送到 `main` 和每个 PR 上运行契约检查、构建、环境检查和全部测试。

---

## 9. 复现与问题反馈

在新电脑上复现时，建议：

1. 完成第 4 节的安装和构建，`pixi run check-env` 0 错误。
2. 运行 `pixi run test` 和 `pixi run test-perception`。
3. 无界面连续运行 `pixi run midterm` 至少 3 轮（每轮之间检查 `pgrep -af "gz sim"`），记录每轮是否 `passed=True` 和 `times_s.total`。
4. 再用 `pixi run midterm-gui` 跑一轮，在 Gazebo 里观察有没有擦碰、卡住、来回转等现象。

报告问题时请附上：电脑配置（CPU 型号和核数、内存、显卡、Ubuntu 版本）、运行时的负载（`uptime`）、该轮的 `<run_id>.rehearsal.json` 和 `<run_id>.launch.log`（较大时压缩后发送）。

---

## 10. 常见问题排查

| 现象 | 原因与处理 |
|---|---|
| Gazebo 一启动就退出，或测试报 `PluginValidationError` | 系统 ROS 的路径混进了 Pixi 环境。确认是用 `pixi run` 或 `pixi shell` 启动的，并且之后没有再 `source /opt/ros/...` |
| 越跑越慢、偶发失败 | 有残留的 Gazebo 或 `ros2 daemon`。用 `pgrep -af "gz sim"` 查看，确认是自己的测试进程后结束它（`pkill -f "gz sim"`），必要时运行 `ros2 daemon stop` |
| `ESTOP: map manager unavailable` 等急停 | 机器负载太高，仿真变慢，墙钟超时被触发。关掉其他程序，等负载降下来再跑，或者用无界面模式 |
| `SLAM/Nav2 did not become ready` | 启动太慢或有残留进程。先清理残留再重试；仍然失败时看 `.launch.log` |
| 修改的配置没有生效 | 改了 `src/` 下的配置或 launch 文件后要重新 `pixi run build` |
| 失败后想定位原因 | 先看 `rehearsal.json` 里的 `error`、`p4_status_sequence`、`p4_last_status`（其中的 `dispatch_gate`、`failure`），再在 `.launch.log` 中搜索 `edge.coordinator`、`robot_N.robot_agent`、`collision_monitor` |
| 想同时跑两轮 | 给第二轮换一个 `--domain`（例如 78）。但一台电脑同时跑两轮通常会因为负载太高而失败，不建议 |

---

## 11. 已知限制

1. **样本量**：最后 4 轮全部通过，但更早的几批大约 3/4 通过；需要在更多电脑、更多轮次上确认成功率。
2. **前沿探索在中期场地里基本不动**：剩下的前沿都在窄通道后面，达不到 0.6 m 的路线净空要求。标记完全靠 `config/sim/midterm_marker_viewpoints.yaml` 里的补看点检测。如果官方场地布局不同，需要重新制作补看点。
3. **部分安全超时用墙钟时间**（地图 3 s、日志器、导航目标 90 s）：电脑过载时可能误触急停。
4. **余量较小的位置**：起点离南墙 0.45 m、S1 接近点离北墙 0.47 m、两个 S1 补看点 0.46 m，余量只有 4～6 cm。
5. **两台机器人在窄处对向行驶**时会互相等待（不会接触），目前靠任务调度避免。
6. 规则上尚待组织方确认：13 分钟按墙钟还是仿真时间计算；场地坐标原点是否在左下角；切换时能否直接把机器人放回起点。
7. 以上全部是 Gazebo 仿真结论，**真实机器人和传感器尚未验证**。

---

## 12. 仓库结构

```text
README.md                       项目入口（本文件）
pixi.toml / pixi.lock           Pixi 环境、依赖锁文件和常用任务
start_sim.sh                    开发仿真入口（空场地，手动驾驶）

config/                         共享配置（运行时直接读取）
  competition_parameters.yaml   官方参数（未确定为 null）与开发默认值；起点 midterm_robot_1/2
  interface_runtime.yaml        QoS、频率、超时、坐标系、阶段跳转表
  development_scenarios.yaml    开发用 Marker 和测试场景
  sim/robots.yaml               开发世界的出生位姿
  sim/robots_midterm.yaml       中期场地的出生位姿
  sim/midterm_marker_viewpoints.yaml   Task 1 补看标记的观察点和路线

src/
  edgeai_robot_interfaces/      共享 msg/srv/action（唯一来源）
  edgeai_common/                公共 Python 工具（配置解析、QoS、帧名）
  edgeai_robot_description/     机器人模型、开发世界、RViz 配置
  edgeai_robot_bringup/         多机仿真启动、robot_agent、速度看门狗、中期仿真入口
  edgeai_robot_navigation/      Nav2/AMCL、重新定位、对方足迹、中期检查工具
  edgeai_exploration/           SLAM 地图管理、前沿探索、补看点协调
  edgeai_system_manager/        阶段管理、冻结、事件日志、审计、系统入口 task1.launch.py
  edgeai_target_perception/     ArUco 检测、目标融合、检测证据
  edgeai_task_coordinator/      救援协调、中期场地与预建地图、物理接触检测插件
  edgeai_system_tests/          Mock 节点与系统测试

tools/
  midterm_rehearsal.py          中期全流程演练（pixi run midterm）
  probe_localized.py            失败时抓取 robot_2 定位现场（被演练脚本调用）
  check_environment.py          环境一致性检查（pixi run check-env）
  validate_project_contracts.py 契约检查（pixi run validate）
  pixi_activate.sh              Pixi 激活时去掉系统 ROS 路径
  install_system_ros_deps.sh    仅在使用系统 ROS 时需要

docs/                           设计文档、架构决策（ADR）和组织方参考资料（见第 13 节）
schemas/                        Task 1/Task 2B 和运行证据的 JSON Schema
examples/interfaces/            Schema 的有效样例
.github/workflows/              CI：契约检查、构建与测试
```

`build/`、`install/`、`log/`、`runs/`、`.pixi/` 都是本地生成的，不进 git。

---

## 13. 文档索引

- [docs/README.md](docs/README.md)：设计文档总目录。
- [docs/system_architecture_and_interfaces.md](docs/system_architecture_and_interfaces.md)：系统架构与接口协议。
- [docs/package_and_node_layout.md](docs/package_and_node_layout.md)：ROS 包与节点布局。
- [docs/marker_and_rescue_geometry.md](docs/marker_and_rescue_geometry.md)：Marker 坐标约定与救援区几何。
- [docs/rule_assumptions_and_tbd.md](docs/rule_assumptions_and_tbd.md)：比赛规则假设与 TBD 登记。
- [docs/decisions/](docs/decisions/)：架构决策记录 ADR-0001～0003。
- [docs/simulation_basics.md](docs/simulation_basics.md)：开发仿真、话题列表、手动驾驶、手动 SLAM、系统 ROS。
- [docs/reference/](docs/reference/)：组织方的比赛规则草案和初始仿真指南。
- 各包 README：`src/<包名>/README.md`，记录每个文件、参数、接口和验证结果。
