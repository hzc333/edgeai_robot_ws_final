# edgeai_task_coordinator

任务协调包：期中Part 2的任务分配、双机协作、连续停留判定、动作取消与释放，以及执行证据记录。使用预建地图和官方固定目标坐标，在TASK2A阶段自主执行S1、C1。

| 项目 | 内容 |
|---|---|
| 当前状态 | 期中功能已实现 |
| 已实现 | S1单机救援、C1双机协同、目标范围与连续停留验证、动作释放和异常处理 |
| 已实现（续） | 同一run内受控恢复、物理接触实时观测与历史记录；双机进场检查、停靠稳定确认和S1失败换机 |
| 适用范围 | Task 2A 使用中期 S1/C1 和两台机器人。通用 Task 2A/2B 在 Gazebo 使用冻结目标快照，见第 8 节。 |

## 1. 文件清单

以下路径相对于本包目录。

| 路径 | 作用 |
|---|---|
| `.gitignore` | 忽略本机编译的Gazebo插件二进制 |
| `README.md` | 包说明、启动、接口和参数 |
| `LICENSE` | Apache-2.0许可证 |
| `package.xml` | ROS依赖、包元数据、ament_python构建类型 |
| `setup.py` | Python/资源/文档安装与coordinator入口；tools按源码路径运行 |
| `setup.cfg` | ros2 run可执行文件安装位置 |
| `resource/edgeai_task_coordinator` | ament包索引标记，空文件需保留 |
| `edgeai_task_coordinator/__init__.py` | Python包标记 |
| `edgeai_task_coordinator/model.py` | 官方配置校验、数据类型、机器人健康/空闲检查 |
| `edgeai_task_coordinator/planning.py` | 膨胀静态栅格、A*/距离场代价、分配枚举；固定目标的距离场在后台预计算 |
| `edgeai_task_coordinator/scheduling.py` | 车体轮廓、槽位/入口几何检查和进场许可 |
| `edgeai_task_coordinator/verification.py` | 圆形范围与共同连续停留判定 |
| `edgeai_task_coordinator/engine.py` | 预留、进入、重试、取消和释放状态机 |
| `edgeai_task_coordinator/mission_execution.py` | 冻结目标分配、Task 2B 依赖调度、逐区停留验证、动作恢复和证据 |
| `edgeai_task_coordinator/evidence.py` | 文件锁、事件追加、原子快照和收据恢复 |
| `edgeai_task_coordinator/collision.py` | 碰撞观测有效性、接触事件去重及历史锁存 |
| `edgeai_task_coordinator/physics.py` | Gazebo数据接收队列、插件库定位及observer_plugin入口 |
| `gazebo_plugin/Contacts.cc` | 逐物理步采集机器人所有碰撞几何体，聚合接触并发布覆盖/存活心跳；构建时编译 |
| `edgeai_task_coordinator/coordinator.py` | ROS适配、动作/阶段客户端、事件/状态及退出清理 |
| `config/midterm.yaml` | 官方规则、机器人选择和工程参数 |
| `launch/midterm_rescue.launch.py` | 协调器启动入口 |
| `launch/development_sim.launch.py` | 测试世界、地图和机器人栈；不提供协调器或阶段管理替身 |
| `tools/generate_midterm_fixture.py` | 生成SDF、ArUco图案、地图和几何清单 |
| `tools/run_simulation.py` | 真实双机演练、阶段测试控制器、真值审计；可注入冻结及协调器进程SIGKILL |
| `tools/simulation_driver.py` | 从ROS状态观察独立协调器进程，管理启动、强制重启和日志 |
| `tools/check_collision_pipeline.py` | 独立物理接触测试：无接触、撞墙、机器人互撞和目标支架 |
| `tools/audit_simulation.py` | 不导入协调器判定器，独立审计原始轨迹 |
| `test/test_core.py` | 33项规则、时间、分配、故障/释放和日志测试 |
| `test/test_ros.py` | 14项机器人Mock上的DDS/Action/Service集成测试 |
| `test/test_scheduling.py` | 23项进场稳定性、入口占用、停靠漂移、同伴失效、换机重试及地图检查测试 |
| `test/test_collision.py` | 11项观测缺失/过期、双机覆盖、接触去重、分离后历史及崩溃重放测试 |
| `test/test_quality.py` | 2项ament版权/flake8检查 |
| `test_assets/midterm.sdf` | 场地、障碍、目标支架、ArUco及接触传感器开发世界 |
| `test_assets/midterm.pgm` | 270×270预建占据地图图像 |
| `test_assets/midterm.yaml` | Nav2地图图像引用、原点和阈值 |
| `test_assets/geometry.json` | 几何清单；仿真脚本实际读取生成器同源BOXES常量 |

## 2. midterm_rescue.launch.py

在仓库根目录执行，环境为仓库Pixi、ROS 2 Jazzy / Gazebo Harmonic：

```bash
# 构建本包及依赖；需要独立仿真时用 pixi run build 构建全部包
pixi run colcon build --symlink-install --packages-up-to edgeai_task_coordinator
# 先启动机器人、定位、地图、时钟和阶段管理，再启动协调器
pixi run bash -c 'source install/setup.bash && ros2 launch edgeai_task_coordinator midterm_rescue.launch.py'
# 观察任务状态
pixi run bash -c 'source install/setup.bash && ros2 topic echo /edge/midterm_rescue/status'
```

**场地接入：** 本包开发场地已加载物理接触观测器。使用其他Gazebo世界时，在`<world>`内加入下面命令输出的插件元素，保持机器人模型名为robot_1～robot_4、地面模型名为ground_plane，且Gazebo与协调器使用同一GZ_PARTITION：

```bash
pixi run bash -c 'source install/setup.bash && ros2 run edgeai_task_coordinator observer_plugin'
```

插件随本包自动编译，依赖Pixi中的C++17工具链及Gazebo Harmonic。打印的filename为本机绝对路径，换机器需重新构建并生成。未加载、观测过期或覆盖不完整时显示`UNKNOWN`，协调器不派发新动作；执行中遇到此情况则取消当前尝试。

启动一个协调进程，内含`/edge/task_allocator`、`/edge/robot_state_registry`、`/edge/rescue_verifier`三个节点，使用单线程执行器。

| 参数 | 默认值 | 含义 |
|---|---|---|
| `config_file` | 安装后本包的 `config/midterm.yaml` | 协调器配置文件路径 |
| `config_dir` | `''`，自动查找 | 共享契约目录路径，查找顺序见参数表 |
| `output_dir` | 启动工作目录下的 `runs/rescue` | 证据根目录路径 |

`use_sim_time`由launch固定为`true`。其他节点参数见2.1节，通过`ros2 run`设置，例如关闭自动冻结请求：

```bash
pixi run bash -c 'source install/setup.bash && ros2 run edgeai_task_coordinator coordinator --ros-args -p use_sim_time:=true -p request_finish:=false'
```

### 2.1 节点参数

以下参数属于`/edge/task_allocator`，启动时读取，不动态重载。路径与布尔量无物理单位，默认值来自节点或launch。

| ROS参数 | 默认值 | 含义 |
|---|---|---|
| `config_file` | 安装后的本包config/midterm.yaml | 协调器独立YAML路径，不是ros__parameters文件，不可直接用作--params-file |
| `config_dir` | `''` | 共享目录；依次检查显式目录、EDGEAI_CONFIG_DIR、当前至父目录的config，选首个含competition_parameters.yaml的目录，再读interface_runtime.yaml |
| `output_dir` | 工作目录下runs/rescue | 证据根目录 |
| `require_collision_monitor` | `true` | 要求有效物理接触观测；只有test_mode可关闭 |
| `require_map` | `true` | 要求有效地图，只有test_mode可关闭 |
| `test_mode` | `false` | 允许测试时关闭地图/接触观测要求或使用非仿真时钟；本身不自动关闭这些检查 |
| `request_finish` | `true` | 完成后请求进入FROZEN |
| `use_sim_time` | 正式launch传true；rclpy标准默认false | 正式模式须true；直接ros2 run时显式传入 |

## 3. 协调器节点

### 3.1 执行流程

1. 等待仿真时钟和运行身份（run_id、map_id非空，map_revision>0）建立证据目录；随后等待有效地图、物理接触观测和允许执行的TASK2A心跳。各输入就绪前不派发救援动作。
2. 两台机器人均健康、定位有效且空闲停稳后分配。后台距离场估算路程，枚举分配，先比较最长路程，再比较总路程；靠墙的原始空闲起点允许在有限范围内连接膨胀自由区，不穿墙或未知格。实际导航由机器人代理和Nav2执行。
3. 按配置顺序执行S1、C1。C1同时预留两台机器人，先进入远侧槽位；第一台须拥有当前任务、处于HOLDING、距槽位≤0.12m、速度满足停稳条件，并以新鲜样本连续确认0.5仿真秒，第二台才开始整段导航。
4. 每台先导航至接近点，动作成功且空闲后发送救援停靠动作，`hold_timeout=0`保持至协调器取消；已经在最终停靠容差内时直接发送救援动作。
5. 所有参与者HOLDING后开始共同停留判定；完成事件落盘后发布，随后取消保持动作。
6. 收到动作终态且RobotState确认空闲、停稳后释放机器人；全部完成显示`DONE`，请求进入FROZEN。

判定使用map中base_footprint到**目标圆心**的距离，导航停靠点与圆心分开。离区、样本过期或无效会重置计时；重复时间戳和仿真暂停不累计时间。默认额外等待0.3仿真秒采样余量，实际判定不少于官方10仿真秒，目标数值见第4节。

C1全程串行是当前实现的保守调度选择，官方只要求两台同时满足距离条件并连续保持10仿真秒，不要求串行出发。实际路径、转向和速度由机器人执行，进场顺序、接近点和停靠点由本包配置。

动作部分接受时撤销整组，迟到接受仍取消。普通释放超时进入`HALTED`；安全中断的释放超时进入`RECOVERING`，继续确认旧动作。`DONE`表示救援完成且动作已释放，`finish_request=ACCEPTED`才表示阶段服务已接受冻结请求；两台safe确认在阶段控制流程中核对。

**调度保护：** 启动时拒绝车体间距不足、后续入口穿过先到槽位的配置；收到地图后在后台检查停靠/接近姿态与占据格、未知格及地图边界的间隙。派发和接近点转入最终救援前，均检查名义直线入口与另一台机器人实际朝向下的车体轮廓。当前双机轮廓间距≤0.05m、已停靠机器人漂移/重新运动、任一机器人状态失效，或未预留机器人开始运动时，取消当前尝试，确认安全条件后恢复。速度信息仅用于确认停稳，不发布速度命令。

普通失败后至少等待1仿真秒，且必须确认动作结束和停稳，才重新分配；S1优先换另一台可达机器人，只有原机器人可达时仍允许重试。C1重新分配保留已占用槽位的零路程优势，停在容差内者直接重入保持。`dispatch_gate`显示等待停稳、入口占用、重试退避或无可达分配的具体原因。

查看`/edge/midterm_rescue/status`时，先看`status`，再看对应的等待或错误字段：

| status | 含义与查看位置 |
|---|---|
| `WAITING_FOR_PHASE_MAP_AND_CLOCK` | 尚未建立运行身份或仿真时钟；检查阶段消息、run/map字段和clock |
| `WAITING` | 等待下一次分配；检查阶段、地图、collision及`dispatch_gate`，后者只描述已检查到的分配条件 |
| `ACTIVE` | 已分配；`task_state`区分导航、等待伙伴和停留，`dispatch_gate`说明进场等待原因 |
| `RELEASING` | 正在取消动作并等待终态、空闲和停稳 |
| `RECOVERING` | 正在安全恢复，查看`recovery_gate` |
| `HALTED` | 自动执行已停止，查看`failure`、`error`并排查原因 |
| `DONE` | S1/C1完成且动作释放；继续核对`finish_request`和阶段冻结确认 |

### 3.2 ROS接口

共享定义以 [edgeai_robot_interfaces](../edgeai_robot_interfaces/README.md) 为准，QoS来自 [config/interface_runtime.yaml](../../config/interface_runtime.yaml)。`robot_N`为配置选定的两台机器人。

以下QoS均KeepLast：`robot_state`=Reliable/Volatile/1；`snapshot`、`control_state`=Reliable/TransientLocal/1；`event_stream`=Reliable/Volatile/1000。全局坐标map，位置m、朝向rad，机器人位姿参考robot_N/base_footprint；协调器不查TF。仿真时间来自clock，10Hz调度及心跳看门狗用墙钟/单调时钟。

| 方向 | 话题 | 类型 | QoS / 频率 | 坐标、时间及用途 |
|---|---|---|---|---|
| 订阅 | `/robot_N/state` | `edgeai_robot_interfaces/msg/RobotState` | robot_state / 默认10Hz | map位姿，header为机器人仿真采样时间；校验身份、定位、健康、模式及active_task_id |
| 订阅 | `/competition/state` | `edgeai_robot_interfaces/msg/CompetitionState` | control_state / 契约1Hz，开发替身5Hz墙钟 | 非几何；header为system_manager仿真时间，同时检查墙钟接收间隔、阶段和run/map身份 |
| 订阅 | `/competition/estop` | `edgeai_robot_interfaces/msg/EmergencyStop` | control_state / 变化及重发 | 非几何；按更大generation更新active，不靠header计时 |
| 订阅 | `/map` | `nav_msgs/msg/OccupancyGrid` | snapshot / 加载或变更 | frame=map；stamp由地图提供方填写，不参与停留计时；用于分配估算及停靠检查，未知格视为障碍 |
| 隐式订阅 | `/clock` | `rosgraph_msgs/msg/Clock` | TimeSource默认BestEffort/Volatile/KeepLast1 / 仿真器决定 | use_sim_time启用，驱动仿真时钟 |
| 发布 | `/edge/robot_states` | `edgeai_robot_interfaces/msg/RobotStateArray` | robot_state / 约10Hz墙钟 | registry发布，数组header=map/当前仿真时间，子状态保留原stamp；消费者仍须判新鲜度 |
| 发布 | `/edge/rescue_events` | `edgeai_robot_interfaces/msg/RescueEvent` | event_stream / 事件触发 | verifier发布，header=map/事件sim_time，位姿来自最新状态；落盘后发布 |
| 发布 | `/edge/midterm_rescue/collision` | `std_msgs/msg/String`，JSON | snapshot / 约10Hz墙钟 | 当前CLEAR/CONTACT/UNKNOWN、覆盖几何体数量、物理存活证明、历史接触次数/对象/仿真时间；无Header，数据帧时间来自Gazebo |
| 发布 | `/edge/midterm_rescue/status` | `std_msgs/msg/String`，JSON | snapshot / 约10Hz墙钟 | allocator发布；无Header，正常执行含sim_time、identity、圆心/半径、距离、failure/error、finish_request；等待/错误快照字段可简化 |

本包无业务服务/动作服务器，调用以下端点：

| 端点 | 类型 | 用法 |
|---|---|---|
| `/robot_N/execute_navigation_goal` | `edgeai_robot_interfaces/action/ExecuteNavigationGoal` | 有接近点且未停在最终容差内时调用；purpose=TRANSIT，request_id=task_id；map目标stamp为发送时仿真时间，容差0.10m/0.25rad，timeout默认240墙钟秒 |
| `/robot_N/execute_rescue_task` | `edgeai_robot_interfaces/action/ExecuteRescueTask` | 接近完成且空闲后发送map停靠目标，stamp为发送时仿真时间；C1共用task_id、槽位0/1；hold_timeout=0，直到协调器取消 |
| 两种动作的 `/_action/cancel_goal` | `action_msgs/srv/CancelGoal` | 仅取消记录的UUID；GoalInfo stamp=0；同目标至少间隔0.5墙钟秒重试，无坐标 |
| 两种动作的 `/_action/get_result` | 各动作自动生成的 `ExecuteNavigationGoal_GetResult` / `ExecuteRescueTask_GetResult` 服务 | 重启或响应丢失时按持久化UUID查询旧动作终态；挂起查询超过2墙钟秒后重新查询 |
| `/competition/set_phase` | `edgeai_robot_interfaces/srv/SetCompetitionPhase` | TASK2A中全完成且释放后，或记录物理接触时请求FROZEN，REQUESTER_SYSTEM；非几何、不含采样stamp |

动作服务/普通服务用rclpy默认Reliable/Volatile/KeepLast10；动作feedback相同，status为Reliable/TransientLocal/KeepLast1。客户端不处理feedback，状态依据RobotState与动作接受/终态；反馈频率由机器人代理决定。动作run/map身份取自阶段消息，救援类型S1=NORMAL、C1=COOPERATIVE。

RescueEvent的rescue_zone_indices沿用槽位；官方判定圆写入JSON和磁盘证据。`collision_status`为`CLEAR`、`CONTACT`或`UNKNOWN`，`collision.ever_contact`锁存本run是否曾发生接触。当前接触解除不清除历史。数据超过0.5仿真秒或1墙钟秒未更新、缺任一机器人的几何覆盖或地面支撑接触证明时，状态为UNKNOWN。

Gazebo输入为`/edgeai/physics_contacts`（`gz.msgs.Contacts`，约20Hz仿真调度）；插件逐物理步读取所监测几何体的接触并聚合发布，Gazebo侧保留最近256个带序号事件，协调器重启后按实例与序号补录、去重。排除地面支撑和同一机器人内部接触，覆盖机器人互撞及与墙、障碍、支架的接触。物理接触会取消当前任务、发布BLOCKED事件并请求FROZEN，历史锁存后不自动恢复运动。该观测反映已经发生的接触；碰撞前的保护由调度检查与机器人导航安全链共同提供。

### 3.3 接入约定

| 模块 | 接入约定 |
|---|---|
| **机器人代理** | 默认两台 `robot_1/robot_2`；RobotState提供map中的base_footprint位姿、仿真时间戳、定位/导航/健康和动作状态；提供导航与救援动作。救援 `hold_timeout=0`，由协调器取消；释放需动作终态和空闲停稳共同确认 |
| **system_manager / map_manager** | 提供共享 `/map`、仿真 `/clock`及阶段/急停接入，保持run_id/map_id/正map_revision一致；编排重新定位。就绪后进入TASK2A，完成后处理FROZEN请求并收集两台safe确认；记录比赛总计时与全局日志 |
| **目标感知** | 官方marker 1=S1、2=C1；中期本包用官方固定坐标，不要求目标先经Part 1观察/冻结，不读取 `/edge/targets` |
| **证据核查** | 检查RescueEvent、本包JSON状态与磁盘证据；共享RescueTask的zone_tolerance_m围绕停靠点，不能直接当作官方目标半径，接入时区分停靠点与完成区域 |

联调顺序：启动地图、机器人和HANDOVER心跳 → 调用relocalize确认定位/导航就绪 → 启动协调器 → 进入TASK2A → S1/C1完成并释放 → 接受FROZEN请求，核对两台同phase_sequence的safe确认。

阶段心跳需满足机器人约1 s失联停车阈值，开发控制器以5 Hz墙钟发布。开发演练用development栈；正式模式的共享`required_for_motion`还会检查Task 2B等配置，启动时需确认中期配置。

### 3.4 中断恢复

发生冻结、急停、阶段/心跳中断、物理观测失联或进程重启后，未完成任务进入`RECOVERING`。已完成目标从持久化事件重建，保留在同一run内，不重复计分；未完成目标的连续停留重新计时。

恢复前依次检查：

1. run_id、map_id、map_revision和配置哈希一致；时钟、地图及物理观测有效，没有历史物理接触。
2. 按持久化UUID取消旧动作，并查询终态。对于崩溃前只写入UUID而未发出的目标，间隔至少1墙钟秒两次确认服务器返回UNKNOWN，再结合新鲜空闲状态确认清理。
3. 两台机器人状态新鲜、定位/导航可用、没有活动任务且停稳，当前车体轮廓间隙大于`entry_clearance_m`；动作服务器可用。
4. 当前阶段允许TASK2A。冻结或急停后，必须由system_manager发布更高phase_sequence的TASK2A；解除急停本身不恢复运动。单纯心跳/观测短暂失联，在同阶段输入恢复后可继续。
5. 上述条件连续稳定`recovery_stable_wall_s`后，分配新的尝试ID并重做未完成目标；不重置run或比赛总计时。

`recovery_gate`会显示等待阶段授权、旧动作终态、新鲜空闲状态、车体间隙、服务器或物理观测。`max_attempts`限制每目标累计普通失败次数，达到上限停止；安全中断恢复使用新尝试ID，不消耗普通故障重试预算。

### 3.5 证据文件

证据写入`output_dir/<run_id的SHA256前24位>/`，收到有效运行身份与仿真时钟后创建：

| 文件 | 作用 |
|---|---|
| `identity.json` | run/map版本、规则来源与配置哈希；同run不同配置/地图拒绝复用 |
| `events.jsonl` | 救援专用事件，追加并fsync后发布；序号与完成收据防重复 |
| `distances.jsonl` | 仿真时间/任务变化时保存距离、计时与状态 |
| `snapshot.json` | 原子快照，含活动goal UUID及结束请求状态 |
| `owner.lock` | 防同目录重复写入者的文件锁 |

恢复时从事件重建完成收据及碰撞历史，按3.4节检查后继续；地图或运行身份改变时保留停止状态。全局事件日志按RescueEvent接入；ROS事件不回放离线历史，可查本包磁盘证据。结束服务拒绝/通信错误显示原因，不自行发布阶段或自动重试请求。

演练目录还含 `report.json`、`audited_report.json`、`ground_truth.jsonl`、`source_manifest.json`、环境/进程日志、独立 `robot-config/` 和 `evidence/`；工具用独立DDS域/Gazebo分区，结束清理自己的进程组。NVIDIA可用时仅为当前测试选择EGL驱动。

## 4. 主要参数（`config/midterm.yaml`）

`config/midterm.yaml`保存目标定义和工程参数，默认按S1、C1顺序执行。官方字段不匹配时拒绝启动。

| 字段 | 默认值与单位 | 来源/含义 |
|---|---|---|
| `schema_version` | 1 | 配置格式版本 |
| `rule_profile` | kth-midterm-2026 | 规则标识 |
| `frame_id` | map | 坐标约定 |
| `robots` | [robot_1,robot_2] | 开发选择；robot_1至robot_4中两台不同ID |
| `targets[].id` | S1 / C1 | 官方名称，数组顺序决定执行顺序 |
| `targets[].marker_id` | 1 / 2 | 官方marker ID |
| `targets[].center` | [1.0,4.1] / [4.1,1.0]，m | 官方圆心 |
| `targets[].radius_m` | 0.5 / 0.7，m | 官方半径 |
| `targets[].required_robots` | 1 / 2，台 | 官方最低人数对应的分配数量 |
| `targets[].dwell_s` | 10.0，仿真秒 | 官方连续停留 |
| `targets[].slots` | 见下表，m/m/rad | 工程停靠点；须在官方圆内留出arrival容差 |
| `targets[].approaches` | 见下表，m/m/rad | 工程接近点；省略则直接救援，当前验证配置使用接近点 |

| 目标/槽位 | slots `[x,y,yaw]` | approaches `[x,y,yaw]` |
|---|---|---|
| S1/0 | [1.15,4.48,0.0] | [1.60,4.48,0.0]（在接近点掉头后倒车进入停车点） |
| C1/0，先进入远侧 | [4.45,1.45,-1.5707963267948966] | [4.45,2.2,-1.5707963267948966] |
| C1/1 | [3.90,1.5,-1.5707963267948966] | [3.95,2.3,-1.6332151367908543] |

S1从西向东接近以方便安全离开；机器人救援动作可能不使用最终yaw。以下`engineering.*`均为开发值，来源为`config/midterm.yaml`及同值的`model.Settings`默认：

| 字段 | 默认值 / 单位 | 作用 |
|---|---|---|
| `arrival_tolerance_m` | 0.06m | 最终停靠容差 |
| `state_stale_s` | 0.5仿真秒 | 状态过期和最大连续采样间断 |
| `maximum_state_skew_s` | 0.25仿真秒 | 双机采样最大时间差 |
| `phase_heartbeat_timeout_s` | 2.0墙钟秒 | 阶段心跳接收超时 |
| `action_response_timeout_s` | 5.0仿真秒 | 动作接受响应超时 |
| `navigation_timeout_s` | 150.0仿真秒 | 动作导航阶段上限，HOLDING不按此超时 |
| `partner_wait_timeout_s` | 180.0仿真秒 | 每次目标尝试总预算，含伙伴进入和停留 |
| `release_timeout_s` | 5.0仿真秒 | 释放确认预算；普通失败超时HALTED，安全中断则进入RECOVERING继续确认旧动作 |
| `max_attempts` | 2次 | 每目标普通失败次数上限；达到后停止，安全中断不消耗此预算 |
| `parking_map_clearance_m` | 0.005m | 停车足迹与地图占用/未知格的最小间距（仅判重叠；SLAM地图上支架可能偏一格） |
| `allocation_clearance_m` | 0.20m | 仅分配代价地图膨胀，不配置Nav2足迹（默认0.26 m在0.05 m SLAM地图上会挡住S1停车点） |
| `completion_guard_s` | 0.3仿真秒 | 官方10秒之外的采样余量 |
| `recovery_stable_wall_s` | 1.0墙钟秒 | 恢复条件连续稳定的最短时间 |
| `executor_timeout_wall_s` | 240.0墙钟秒 | 接近点导航动作预算，当前仅使用整数秒部分 |
| `footprint_front_m` / `footprint_rear_m` / `footprint_half_width_m` | 0.115 / 0.315 / 0.175m | `nav2_sim.yaml`车体包络，参考base_footprint；更换模型须同步 |
| `entry_clearance_m` | 0.05m | 调度几何检查的最小间隙；不是制动距离 |
| `holding_position_tolerance_m` | 0.12m | 第二台放行及停靠后监控所用的槽位位置容差，不改变官方判定圆 |
| `holding_stable_s` | 0.5仿真秒 | 放行伙伴前的连续新鲜停稳确认 |
| `retry_backoff_s` | 1.0仿真秒 | 普通失败后再次分配的最小等待 |

实现常量：调度0.1墙钟秒；阶段stamp相对now差限[-0.5,2.0]仿真秒；样本最多超前0.02仿真秒；释放需线/角速度≤0.01m/s、0.02rad/s；取消重试0.5墙钟秒；Ctrl+C清理最多3墙钟秒。它们不是ROS参数。

## 5. development_sim.launch.py

```bash
pixi run bash -c 'source install/setup.bash && ros2 launch edgeai_task_coordinator development_sim.launch.py headless:=false'
```

启动开发世界、预建地图和两台机器人栈。完整自动演练使用6.3节的脚本，它另行启动协调器、阶段控制器并准备重定位配置。

| 参数 | 默认值 | 说明 |
|---|---|---|
| `config_dir` | `EDGEAI_CONFIG_DIR`或空字符串 | 共享配置目录路径 |
| `headless` | `true` | 是否关闭Gazebo GUI，布尔值 |
| `robot_1_start` | `0.55,0.5,0.0` | 开发出生姿态x,y,yaw，逗号分隔，m/m/rad |
| `robot_2_start` | `1.20,0.5,0.0` | 同上；重定位配置须使用对应姿态 |

内部固定`use_sim_time=true`、`run_mode=development`、`require_competition_state=true`、`nav_mode=amcl`。开发起点robot_1=(0.55,0.5,0)、robot_2=(1.20,0.5,0)，单位m、m、rad。

### 5.1 工具参数

以下工具按源码路径调用，支持--help；`simulation_driver.py`由演练脚本导入，不是命令行入口：

| 工具 | 全部参数及默认值 |
|---|---|
| `tools/audit_simulation.py` | directory位置参数必填，演练目录路径；写audited_report.json，退出0为通过 |
| `tools/generate_midterm_fixture.py` | output位置参数必填，生成目录路径，会覆盖同名文件；需OpenCV ArUco/PyYAML |
| `tools/check_collision_pipeline.py` | --output必填，必须是新目录；运行物理接触正向测试，结果见6.2节 |

`tools/run_simulation.py`的全部参数：

| 参数 | 默认值 | 含义 |
|---|---|---|
| `--output` | 必填 | 新演练目录路径，不能复用已有目录 |
| `--domain` | 73 | 独立DDS域ID，无单位 |
| `--timeout-wall` | 900 | 演练退出预算，墙钟秒，不是官方任务时限 |
| `--gui` | 关闭 | 传入时打开Gazebo GUI |
| `--exercise-recovery` | 关闭 | S1停留时冻结，C1停留时SIGKILL并重启协调器 |
| `--start-layout` | nominal | nominal为默认起点；swapped交换两台起点；staggered错开位置且第一台朝北；midterm与中期出生点一致（并排朝北）。自动同步出生与重定位配置 |

生成临时场景：`pixi run python src/edgeai_task_coordinator/tools/generate_midterm_fixture.py /tmp/midterm-fixture`。

地图配置test_assets/midterm.yaml全部字段：image=midterm.pgm（相对路径）、mode=trinary、resolution=0.02m/像素、origin=[-0.2,-0.2,0.0]（m/m/rad）、negate=0、occupied_thresh=0.65、free_thresh=0.196（阈值无单位）。均为开发生成值。

测试SDF物理步长0.002 s，配置real_time_factor=1.0；墙厚/支架几何为开发值。ArUco使用4x4_50字典，边长0.16 m、中心高0.35 m。

## 6. 验证

### 6.1 自动测试

```bash
pixi run test      # 全工作区测试
pixi run validate  # 共享契约检查
pixi run check-env
```

只运行本包，使用空闲的独立DDS域：

```bash
pixi run bash -c 'source install/setup.bash && export ROS_DOMAIN_ID=83 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST && colcon test --packages-select edgeai_task_coordinator --event-handlers console_cohesion+ && colcon test-result --verbose'
```

`test_core.py`覆盖规则校验、计时/暂停/回退、伙伴离区、过期/偏时样本、A*及分配、动作拒绝/迟到回调、释放和日志恢复。`test_ros.py`使用机器人Mock及真实DDS/Action/Service，覆盖完成/冻结请求、阶段/急停授权、心跳/观测失联、同run重启、历史锁存、迟到回调和协调器主动退出。质量检查为ament版权与flake8。


### 6.2 物理接触链路测试

```bash
pixi run bash -c 'source install/setup.bash && python src/edgeai_task_coordinator/tools/check_collision_pipeline.py --output runs/contact-check'
```

`--output`必填，必须是新目录。测试使用独立Gazebo分区，分别制造撞墙、机器人互撞、支架接触；预期三类均产生接触记录，移开后恢复CLEAR且历史计数保留；还会在接收器离线期间制造接触，重连后须补录历史。消息未到或覆盖不足时应为UNKNOWN。结果写入report.json和contacts.jsonl。

### 6.3 双机仿真

```bash
# 运行前 pixi run build；output必须是新目录
pixi run bash -c 'source install/setup.bash && python src/edgeai_task_coordinator/tools/run_simulation.py --output runs/part2-check'
pixi run python src/edgeai_task_coordinator/tools/audit_simulation.py runs/part2-check
```

预期仿真/审计退出0且`passed=true`。审计直接检查Gazebo轨迹：S1/C1各一次完成，完成前最后10秒全部在圆内，C1两台不同机器人，采样覆盖两端误差≤0.05秒、最大间隔≤0.1秒，足迹重叠为0、冻结safe确认通过。


**计时口径：** 官方10秒停留使用仿真时间，建议的Part 2约7分钟不是独立硬限时；Part 1、切换、任务初始化、导航和停留共同受13分钟总预算约束，提前完成的软件启动不计入。串行调度的等待、接近点/朝向、导航减速和仿真速度都会影响耗时。

故障注入演练：

```bash
pixi run bash -c 'source install/setup.bash && python src/edgeai_task_coordinator/tools/run_simulation.py --output runs/recovery-check --exercise-recovery'
pixi run python src/edgeai_task_coordinator/tools/audit_simulation.py runs/recovery-check
```

协调器作为独立进程运行。预期S1冻结后等待新的TASK2A授权并重新计时；C1停留中协调器被强制终止，重启后清理旧UUID、保留S1收据、重新完成C1；最终每个目标只有一次COMPLETED、物理接触历史为0、冻结safe确认通过。

### 6.4 手动联调检查

按3.3节接入，确认S1单机、C1两台不同机器人各完成一次，无需手动输入导航点；离区/暂停/旧样本不会错误累计停留；动作释放后两台均停车，system_manager收到冻结请求和safe确认。另建测试run检查冻结、急停、心跳中断及进程重启：动作应取消，恢复须满足3.4节条件；发生物理接触后应保留历史并停止执行。保存任务事件、原始真值与总计时记录。

## 7. 已知限制

| 情况 | 当前行为与处理方式 |
|---|---|
| 目标或几何配置改变 | 当前只接受期中S1/C1和两台机器人。停靠点可配置；改变地图、车体或接近方式后，需同步配置并重跑接触测试及双机真值审计 |
| 输入失联与恢复 | 状态/心跳/观测短暂失联后，按3.4节条件恢复；旧动作终态或停稳无法确认时保持等待。普通释放超时则HALTED，安全中断释放超时则RECOVERING，详见`failure`和`recovery_gate` |
| 身份变化、证据异常或重试耗尽 | 同run更换地图/配置、不可恢复的日志损坏、普通失败达到上限时不自动续跑。先排查`error`/`failure`，保留已有证据；更换运行身份或配置时使用新的run |
| 实际发生物理接触 | 停止当前执行并锁存历史。分离后的CLEAR仅表示当前无接触，仍须查看`ever_contact`；重启协调器不会清除本run的碰撞记录 |
| 观测器接入或历史不完整 | 需加载插件并使用约定模型名。缺失、过期、覆盖不足时UNKNOWN；观测器实例变化、接触历史缺号或接收队列溢出会锁存错误，输入恢复也不会自动清除此类错误 |
| 入口检查与持续堵路 | 入口检查使用名义直线包络，不验证实际全局路径和转向扫掠。持续堵路最多按普通失败预算重试，之后停止；没有自动撤退策略 |
| 仿真长暂停及整队时间 | 停留/动作调度按仿真秒，接近点导航动作默认还有240墙钟秒预算，心跳与恢复稳定期也看墙钟。长暂停可能取消尝试；整队13分钟需按实际任务初始化、切换和执行流程核验 |

## 8. 按冻结目标表执行 Task 2A 和 Task 2B

新入口使用 Task 1 存档的 `task1/targets.json` 和系统管理器生成的
`system/parameters.yaml`。只有冻结快照的 run、地图和目标版本与比赛状态一致，且参数
文件的哈希与比赛状态一致，才会使用该输入。决赛参数未定时，开发模式使用对应的开发
规则。中期固定 S1/C1 入口继续供原演练回归。

- Task 2A 只分配冻结时有基础救援资格的目标。已完成目标有持久化收据，单次尝试失败后
  释放动作，暂停 `rescue_retry_backoff_s`（默认 5 秒），再优先分配给之前未失败的机器人。
  未使用队友堵路等待额度的导航失败可额外重分配 3 次。阶段切换和急停取消不计入失败预算。
  全部目标完成且动作释放后，请求系统管理器冻结。
- Task 2B 订阅 `/edge/mission_plan`，调用 `DependencyScheduler` 并发布 `/edge/mission_state`。
  2A 中救过的目标在 2B 中重新执行，分别记录阶段、计划、步骤和任务 ID。
- 两阶段共用机器人预留、停车位冲突检查，以及 `/edge/rescue_verifier` 的连续停留判定。
  协作目标的两台机器人必须各占自己的救援区，位置、样本新鲜度或任务归属失效会重置计时。
  只使用实际状态样本推进停留计算。仿真接触监测异常时停止派发并释放动作。
- 协同任务先预留两台车和两个区。首车在分配区内稳定保持，且队友未占用后车的入口后，
  才放行后车。如果首车的实测车身占用名义停车位的安全余量，优先检查原救援区内的替代
  停车点。替代点仍须满足地图可达和车辆间隙检查，停留判定继续使用冻结区中心。
  已经停在目标救援区内的车辆优先保留该区，减少两车互换停车区的动作。
  若 Nav2 返回 104、208，或机器人动作报告导航受阻，并且有队友可能堵路的位姿证据，
  每隔 5 墙钟秒在同一个任务 ID 下重试该动作，最多等待 45 墙钟秒且受步骤超时限制。
  已派出的车都停稳、剩下的车却都找不到无冲突的停车点时，同样最多等 45 墙钟秒，然后
  结束这次尝试、释放机器人重新分配，而不是一直占着到步骤超时。
- 分配器记住每台车导航失败时还没离开的位置（离出发点 0.15 m 以内）。这台车之后又停在
  那里、需要出发时排在其他车后面；它从那里成功离开后忘掉这个位置。Task 2B 的步骤不能
  中途换车，例如 2A 里没能从协同救援区出发的车，2B 再从同一处出发多半同样失败。
- 开发或演练中，若 Task 2A 还有失败目标，但所有尝试均已结束且动作已释放，会记录失败
  清单并申请冻结。演练可继续验证 Task 2B，同时仍将不完整的 Task 2A 计为整场未通过。
  比赛模式中，失败目标保留在状态里，由系统管理器按比赛时限或裁判授权结束。
- 阶段切换和急停会终止活动任务。分配器在派发前保存动作 UUID，重启后取消并核对旧动作；
  运行中的 2B 计划中止，已确认完成的步骤保留。 `rescue_step_timeout_s` 是每次派发的墙钟
  工程超时，默认 240 秒，阶段总时限由系统管理器负责。
- `/edge/rescue/status` 提供通用救援进度。证据在该轮 `rescue/frozen/` 下，系统事件记录
  也接收 `/edge/rescue_events`。Gazebo 路径要求有效的接触观测器，实机需要单独验证。

若比赛系统已启动，请在 `task1.launch.py` 使用 `rescue:=false`，然后另启以下入口，
避免同时运行两个分配器。机器人列表、ROS 域和运行目录要与系统管理器一致：

```bash
ros2 launch edgeai_task_coordinator frozen_rescue.launch.py \
  run_directory:=/path/to/run robot_ids:=robot_1,robot_2,robot_3,robot_4
```

从仓库根目录构建后，可在独立 loopback 网络里跑 Mock 回归：

```bash
unshare --user --map-root-user --net bash --noprofile --norc -c '
  ip link set lo up &&
  source tools/task2b/env.sh &&
  python -m pytest -q src/edgeai_task_coordinator/test
'
```

本包的 Gazebo 演练脚本调用现有系统入口和操作员服务。它从 Task 1 开始生成冻结产物，
让同一分配器跑两阶段，默认以原地交接进入 Task 2B。练习场真值仅用于验收发现的目标 ID，
不传给分配器。下面以四台机器人和开放的练习场为例：

```bash
pixi run practice-arena runs/frozen-rescue-arenas/seed-7 --seed 7 --depth 0 --obstacles 0 0
pixi run bash -c 'source install/setup.bash &&
  python src/edgeai_task_coordinator/tools/rehearse_frozen_rescue.py \
    --arena runs/frozen-rescue-arenas/seed-7 --robots 4 --domain 78'
```

演练报告和进程日志写入 `runs/frozen-rehearsal/`。官方几何、时限和重摆流程按组织方
确认后的参数及队伍联调方案执行。
