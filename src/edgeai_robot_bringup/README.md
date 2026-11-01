# edgeai_robot_bringup

P1负责的机器人启动包：多机器人仿真启动、每台机器人的软件栈启动，以及速度安全看门狗`safety_watchdog`。

| 项目 | 内容 |
|---|---|
| 负责人 | P1（hzc333） |
| 已完成 | 多机器人仿真启动；`safety_watchdog`（ADR-0003）及其测试；`robot_stack.launch.py`（看门狗 + 可选Nav2），2026-10-24验证 |
| 已完成 | `robot_agent`：RobotState、阶段确认、`execute_navigation_goal`（阶段2验收，2026-10-24）和`execute_rescue_task`“到达并保持”（计划3-04，双机协同场景验收，2026-10-25） |
| 未实现 | 实机启动 |

## 1. 文件清单

| 文件 | 作用 |
|---|---|
| `edgeai_robot_bringup/safety_watchdog.py` | 看门狗节点，入口`ros2 run edgeai_robot_bringup safety_watchdog`；门控逻辑是纯函数`decide()`，便于测试 |
| `edgeai_robot_bringup/robot_agent.py` | 机器人代理节点，入口`ros2 run edgeai_robot_bringup robot_agent`：10 Hz发布`state`、2 Hz发布`phase_ack`，提供`execute_navigation_goal`（内部调用Nav2 `navigate_to_pose`，到达后检查目标自身容差，不满足时再逼近一次；结束时取消Nav2并等待里程计确认停车，超时则报告`CODE_COMMAND_TIMEOUT`）。Nav2错误码104～106映射为BLOCKED，其余为FAILED；碰撞监视器持续停车超过5 s也判BLOCKED |
| `test/test_robot_agent.py` | 代理测试：纯逻辑（模式优先级、错误码映射、容差和协方差判定）和使用模拟Nav2/TF/里程计/AMCL的18项行为测试 |
| `launch/robot_stack.launch.py` | 在`/robot_N`命名空间启动一台机器人的软件栈：看门狗、机器人代理，以及`nav_mode`不为`none`时的Nav2；按运行模式解析参数 |
| `launch/sim_multi_robot.launch.py` | 启动Gazebo世界、1～4台仿真机器人以及各自的软件栈，可选RViz；`robots_file`选出生位姿文件，`nav_modes`、`parked`、`parked_map_origin`按机器人覆盖导航模式和停放 |
| `launch/midterm_sim.launch.py` | 中期检查仿真入口：P4的中期场地、两台机器人在起始区；`part:=1`建图＋停放，`part:=2`预建地图＋AMCL。见第5.1节 |
| `test/test_safety_watchdog.py` | 看门狗测试：纯逻辑测试和ROS通信测试（见第5节） |
| `test/test_copyright.py`、`test_flake8.py`、`test_pep257.py` | 版权头、代码风格和文档字符串检查 |
| `config/.gitkeep` | 占位，将来放机器人代理的参数 |
| `edgeai_robot_bringup/__init__.py` | Python包入口 |
| `setup.py`、`setup.cfg`、`package.xml`、`resource/` | `ament_python`包配置；`launch/*.py`和`config/*.yaml`安装到包共享目录 |

## 2. safety_watchdog

`/robot_N/cmd_vel`的**唯一发布者**（ADR-0003）。以固定频率重发最新的速度输入；只要任一门控条件成立，就输出零速度。

### 2.1 话题

| 话题（相对命名空间） | 类型 | 方向 | QoS | 说明 |
|---|---|---|---|---|
| `cmd_vel_safe_in` | `geometry_msgs/Twist` | 订阅 | Reliable，深度10 | 自动输入，来自Nav2碰撞监视器 |
| `cmd_vel_manual` | `geometry_msgs/Twist` | 订阅 | Reliable，深度10 | 手动输入，只在`run_mode=development`时生效，生效时优先于自动输入 |
| `/competition/state` | `CompetitionState` | 订阅 | `control_state`（Reliable，Transient Local，深度1） | 阶段和边缘心跳 |
| `/competition/estop` | `EmergencyStop` | 订阅 | 同上 | 急停 |
| `cmd_vel` | `geometry_msgs/Twist` | 发布 | Reliable，深度10 | 底盘速度，`publish_rate_hz`频率 |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 发布 | 默认 | 1 Hz；状态名`robot_N/safety_watchdog`；门控关闭时为WARN，消息中列出原因 |

只使用`Twist`的`linear.x`和`angular.z`，其他分量一律输出0。

### 2.2 门控规则

| 条件 | 诊断原因 |
|---|---|
| 尚未收到`CompetitionState` | `waiting_for_competition_state` |
| 阶段不是TASK1、TASK2A、TASK2B | `phase_<名称>`，如`phase_frozen` |
| 急停生效（只认`generation`比已收到最大值更大的消息） | `estop` |
| 超过`edge_timeout_s`没有新的`CompetitionState` | `edge_connection_lost` |
| 超过`input_timeout_s`没有速度输入 | `input_timeout`（正常现象，不算故障） |

`require_competition_state:=false`时不检查前两项和边缘心跳，但急停照常生效。

速度超过上限时，线速度和角速度**按同一比例**缩小，保持转弯曲率，例如(0.3 m/s, 0.3 rad/s)在上限(0.15, 0.30)下变为(0.15, 0.15)。

### 2.3 参数

| 参数 | 默认值 | 单位 | 来源（由`robot_stack.launch.py`传入） |
|---|---|---|---|
| `robot_id` | `robot_1` | — | 启动参数 |
| `run_mode` | `development` | — | 启动参数：`development`/`rehearsal`/`competition` |
| `max_linear_mps` | 0.15 | m/s | 解析后的`motion.max_linear_speed_mps` |
| `max_angular_radps` | 0.30 | rad/s | 解析后的`motion.max_angular_speed_radps` |
| `input_timeout_s` | 0.5 | s | `interface_runtime.yaml`的`timeouts_s.velocity_command` |
| `edge_timeout_s` | 1.0 | s | `timeouts_s.edge_connection` |
| `require_competition_state` | `true` | — | 启动参数；只有`run_mode=development`允许设为`false`，否则节点报错退出 |
| `publish_rate_hz` | 20.0 | Hz | 固定 |
| `config_dir` | 搜索 | — | 用于读取QoS定义 |

节点退出时会再发布一次零速度。

## 3. robot_agent

机器人与边缘之间的唯一接口（P1计划2-10～2-14、3-04）。边缘节点只调用它，不直接调用Nav2。代理**不发布速度**：结束目标时取消Nav2，由看门狗输出零速度，再用里程计确认停稳。

### 3.1 接口

| 名称（相对命名空间） | 类型 | 方向 | 说明 |
|---|---|---|---|
| `state` | `RobotState` | 发布，10 Hz，`robot_state` QoS | 见3.3节 |
| `phase_ack` | `RobotPhaseAck` | 发布，2 Hz | 对应当前`phase_sequence` |
| `execute_navigation_goal` | `ExecuteNavigationGoal`动作 | 服务端 | 见3.2节 |
| `execute_rescue_task` | `ExecuteRescueTask`动作 | 服务端 | 见3.3节 |
| `navigate_to_pose` | `nav2_msgs/NavigateToPose`动作 | 客户端 | Nav2 |
| `lifecycle_manager_navigation/is_active` | `std_srvs/Trigger` | 客户端，1 Hz轮询 | 得到`navigation_available` |
| `/competition/state`、`/competition/estop` | 共享接口 | 订阅，`control_state` QoS | 阶段、心跳、急停 |
| `odom`、`scan`、`amcl_pose`、`collision_monitor_state` | 标准消息 | 订阅 | 速度/停车确认、传感器健康、定位协方差、受阻判断 |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 订阅 | 读取`robot_N/localization`的最近一次重新定位结果 |
| TF `map -> robot_N/base_footprint` | — | 查询 | 机器人位姿 |

### 3.2 execute_navigation_goal

- **同一时刻只允许一个活动目标**，有活动目标时新目标直接拒绝。
- **接受条件**：已收到`CompetitionState`；`run_id`、`map_id`、`map_revision`一致；阶段为TASK1/TASK2A/TASK2B、无急停、边缘心跳未超时；已定位；Nav2为active；目标在`map`坐标系中。
- **执行**：调用Nav2 `navigate_to_pose`；成功后用当前位姿检查该目标自己的容差（`xy_tolerance_m`为0时用0.10 m；`yaw_tolerance_rad`为0时不检查朝向）。不满足时再发一次做精确逼近，仍不满足返回FAILED。Nav2自身的目标容差为0.05 m，必须小于任何请求的容差。
- **结束**：取消Nav2（如仍在运行），等待里程计|v| ≤ 0.01 m/s且|ω| ≤ 0.02 rad/s，最长`safe_stop_ack`（2 s）；超时仍返回结果，并在状态中报告`CODE_COMMAND_TIMEOUT`。

| 情况 | `terminal_state` | `reason.code` |
|---|---|---|
| 到达且满足容差 | SUCCEEDED | `NONE` |
| Nav2中止，错误码104～106（无进展、控制器超时、无可行控制） | BLOCKED | `NAVIGATION_FAILED`（detail含`blocked`） |
| 碰撞监视器持续停车超过5 s | BLOCKED | 同上 |
| Nav2中止，其他错误（规划失败等） | FAILED | `NAVIGATION_FAILED` |
| 精确逼近后仍超出容差 | FAILED | `NAVIGATION_FAILED` |
| 超过`timeout`（为0时用60 s） | FAILED | `NAVIGATION_TIMEOUT` |
| 定位丢失 | FAILED | `LOCALIZATION_LOST` |
| 调用方取消 | CANCELLED | `USER_REQUEST` |
| 阶段切到非运动阶段 | CANCELLED | `PHASE_CHANGED` |
| 急停（更新的`generation`） | CANCELLED | `SAFETY_STOP` |

代理不会自动重试失败的探索目标（前沿黑名单属于P2）。

### 3.3 execute_rescue_task（到达并保持）

语义是“导航到救援位姿并保持静止”。**机器人不计时、不判断救援是否完成**：救援区占用、停留计时和完成只由边缘的`rescue_verifier`判定，完成或改派后由协调器取消本动作以释放机器人。

- **接受条件**：与3.2节相同，另外`rescue_pose`必须在`map`中，`arrival_xy_tolerance_m > 0`。同一时刻只允许一个活动目标（导航或救援）。
- **逼近**：用Nav2导航到`rescue_pose`；到达判据只看位置（规则3.3），距离 ≤ `arrival_xy_tolerance_m`（开发值0.08 m，必须小于验证器的救援区容差0.15 m）；不满足时再逼近一次，仍不满足返回FAILED。`rescue_ignore_orientation`为真（默认）时，发给Nav2的目标朝向换成“当前位置指向目标”的方向，避免到达后再原地转向。
- **直线回位**：目标在车头或车尾方向上（距离 ≤ `rescue_straight_max_m`，横向偏差 ≤ 0.75倍到达容差）时，先用Nav2的`drive_on_heading`或`backup`行为直线开过去，不转向；到达容差内即完成，否则再用Nav2逼近。原因：机器人大部分车身在轮轴后方，靠墙原地掉头时车尾会扫进碰撞监视器的停车区，机器人会卡住（阶段3实测）。
- **保持**：`mode=MODE_HOLDING`，反馈`STATE_HOLDING`和`distance_to_rescue_pose_m`（2 Hz）。Nav2空闲，看门狗输出零速度。
- **被推离**：距离超过“到达容差 + 0.03 m”（滞回）、阶段允许运动且机器人已停稳（推动结束）时，退出保持并重新逼近，回到后再次保持。被沿车身方向推开时通常走直线回位。
- **结束**（只有以下几种）：

| 情况 | `terminal_state` | `reason.code` |
|---|---|---|
| 被取消，且当时阶段允许运动、无急停（协调器释放或改派） | CANCELLED | `RELEASED_BY_COORDINATOR` |
| 被取消，阶段不允许运动 | CANCELLED | `PHASE_CHANGED` |
| 被取消，急停生效 | CANCELLED | `SAFETY_STOP` |
| 阶段切到非运动阶段 / 急停 | CANCELLED | `PHASE_CHANGED` / `SAFETY_STOP` |
| 首次到达后超过`hold_timeout`（为0则永不超时） | HOLD_TIMEOUT | `HOLD_TIMEOUT` |
| 逼近时Nav2失败、受阻或超时 | FAILED / BLOCKED | 同3.2节 |
| 保持中定位丢失 | FAILED | `LOCALIZATION_LOST` |

ROS 2动作被取消时无法携带原因，以上“被取消”的三种原因是按取消发生时的状态推断的（与`mock_robot_agent`一致，需P4确认）。

### 3.4 RobotState

| 字段 | 规则 |
|---|---|
| `pose` | TF `map -> robot_N/base_footprint`；`header.frame_id`恒为`map` |
| `pose.covariance` | AMCL模式取`amcl_pose`；SLAM模式用固定方差`slam_pose_variance` |
| `localized` | TF不超过`tf_stale_s`；收到过`amcl_pose`后（AMCL激活后，包括SLAM到AMCL交接之后）还要求x、y标准差 ≤ 0.5 m、航向标准差 ≤ 0.6 rad；之前在SLAM模式下只看TF |
| `navigation_available` | Nav2生命周期管理器报告active |
| `mode` | 优先级：ESTOP > SAFE（阶段不允许运动或边缘失联）> ERROR（FATAL故障）> HOLDING > EXPLORING（导航目标`purpose=EXPLORATION`）> NAVIGATING > STANDBY |
| `faults` | `TF_STALE`、`LOCALIZATION_LOST`、`NAVIGATION_ABORTED`（Nav2未active）、`COMMAND_TIMEOUT`、`RELOCALIZATION_FAILED`（`/diagnostics`中`robot_N/localization`为ERROR）、`ESTOP_ACTIVE`、`EDGE_CONNECTION_LOST`、`SENSOR_UNAVAILABLE`（激光1 s无数据） |
| `battery_percentage` | NaN（仿真） |

`localized`的阈值是“仍处于定位状态”的判据，比重新定位收敛的判据（0.10 m，计划3-02）宽松。仿真实测：一次4.7 m的行驶中，AMCL报告的标准差从0.09 m涨到约0.31 m、航向约0.33 rad，而真实误差约0.1 m。

### 3.5 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `robot_id`、`run_mode`、`config_dir` | — | 同看门狗 |
| `require_competition_state` | `true` | 只有development允许`false` |
| `localization_mode` | `amcl` | `amcl`或`slam`；`robot_stack`按`nav_mode`设置。只影响尚未收到`amcl_pose`时的判断 |
| `localized_max_xy_std_m`、`localized_max_yaw_std_rad` | 0.5、0.6 | 见3.3节 |
| `slam_pose_variance` | 0.0025 | SLAM模式下报告的位姿方差（待与P2确认） |
| `tf_stale_s` | 0.5 | TF过期判定[s] |
| `sensor_timeout_s` | 1.0 | 激光无数据判定[s] |
| `default_xy_tolerance_m` | 0.10 | 目标未给容差时使用[m] |
| `blocked_after_s` | 5.0 | 碰撞监视器持续停车多久判为受阻[s]。监视器只在状态变化时发布，目标开始前就已停车的，从目标开始时计时 |
| `rescue_ignore_orientation` | `true` | 救援逼近不要求朝向，见3.3节 |
| `rescue_straight_max_m`、`rescue_straight_speed_mps` | 0.6、0.08 | 直线回位的最大距离[m]和速度[m/s] |
| `stop_linear_eps_mps`、`stop_angular_eps_radps` | 0.01、0.02 | 停稳判定 |

超时（`navigation_default`、`safe_stop_ack`、`tf_lookup`、`edge_connection`）和频率读自`interface_runtime.yaml`。

## 4. robot_stack.launch.py

```bash
ros2 launch edgeai_robot_bringup robot_stack.launch.py robot_id:=robot_1 run_mode:=development
```

| 参数 | 默认值 | 说明 |
|---|---|---|
| `robot_id` | `robot_1` | 命名空间 |
| `run_mode` | `development` | 用`edgeai_common.resolve_parameters`解析比赛参数。`competition`模式下官方必需参数仍为`null`时**启动失败**（当前就是这种情况） |
| `config_dir` | 搜索 | 仓库根目录`config/` |
| `use_sim_time` | `true` | 实机设为`false` |
| `require_competition_state` | `true` | 见2.3节 |
| `nav_mode` | `none` | `none`：只启动看门狗；`amcl`或`slam`：同时包含`edgeai_robot_navigation`的`nav_robot.launch.py`（AMCL模式下导航需在定位后激活，见该包README第2.2节） |

## 5. sim_multi_robot.launch.py

```bash
export EDGEAI_CONFIG_DIR=$PWD/config      # start_sim.sh会自动设置
ros2 launch edgeai_robot_bringup sim_multi_robot.launch.py robots:=robot_1,robot_2
ros2 launch edgeai_robot_bringup sim_multi_robot.launch.py robots:=robot_1,robot_2,robot_3,robot_4 headless:=true
```

`./start_sim.sh N`（N为1～4）等价于`robots:=robot_1,...,robot_N rviz:=true run_mode:=development require_competition_state:=false`，因此可以直接用`/robot_N/cmd_vel_manual`手动驾驶。

| 参数 | 默认值 | 说明 |
|---|---|---|
| `robots` | `robot_1,robot_2` | 逗号分隔的机器人ID，不能为空或重复 |
| `config_dir` | 环境变量`EDGEAI_CONFIG_DIR` | 为空时从当前目录向上查找`config/` |
| `world` | `edgeai_robot_description`的`worlds/empty.sdf` | 世界文件 |
| `headless` | `false` | 只运行Gazebo服务器 |
| `rviz` | `false` | 1台用`realistic_robot.rviz`，多台用`multi_robot.rviz` |
| `stack` | `true` | 为每台机器人包含`robot_stack.launch.py` |
| `run_mode` | `development` | 传给每台机器人的软件栈 |
| `require_competition_state` | `true` | 同上 |
| `nav_mode` | `none` | 同上，作用于每台机器人 |

工作过程：读取`config/sim/robots.yaml`的出生位姿 → 包含一次`sim_world.launch.py` → 每台机器人包含`sim_robot.launch.py`和（`stack:=true`时）`robot_stack.launch.py`。`robots.yaml`中没有某台机器人的位姿时启动报错退出。每台机器人的话题和坐标系见`edgeai_robot_description`的README。

### 5.1 midterm_sim.launch.py（中期检查，2026-11-12）

```bash
ros2 launch edgeai_robot_bringup midterm_sim.launch.py part:=1 headless:=true   # Part 1
ros2 launch edgeai_robot_bringup midterm_sim.launch.py part:=2 headless:=true   # Part 2
```

| 项目 | 内容 |
|---|---|
| 场地 | P4生成的`edgeai_task_coordinator/test_assets/midterm.sdf`（唯一来源，场地坐标，原点在左下角）；P4碰撞观测插件按本机构建路径加载，未构建时去掉插件并提示 |
| 出生位姿 | `config/sim/robots_midterm.yaml`：robot_1 (0.50, 0.50)、robot_2 (1.20, 0.50)，都朝北，与组织方图1一致；两台都能在起点原地转向；起点`midterm_robot_1/2`同值 |
| `part:=1` | robot_1 `nav_mode:=slam`；robot_2 `nav_mode:=amcl`并停放在`midterm_robot_2`（发布`map -> robot_2/odom`，robot_1能看到它的车身）。SLAM和探索归P2；`dev_slam:=true`只供P1测试 |
| `part:=2` | 两台`nav_mode:=amcl`，加载预建地图（P4 `test_assets/midterm.yaml`，可用`map:=`替换）；分别用起点`midterm_robot_N`调用`relocalize` |
| 其他参数 | `headless`、`rviz`、`run_mode`（默认development）、`require_competition_state`（默认true）、`slam_map_origin`（默认`midterm_robot_1`：SLAM地图锚定在robot_1起点） |

Part 1 → Part 2切换（P2编排）时，P1侧需要的顺序已用`edgeai_robot_navigation/tools/midterm_switch_check.py`验证：robot_1回到起点 → `HANDOVER` → 停止SLAM → 加载预建地图并**等新地图到达** → 两台用已知起点`relocalize` → `TASK2A` → **等两台的`phase_ack`确认新阶段**后再派发。实测切换12 s，切换后两台导航真值误差0.034～0.065 m（见导航包README 6.1节）。

## 6. 验证

```bash
pixi run test     # 或只测本包：colcon test --packages-select edgeai_robot_bringup
```

`test/test_safety_watchdog.py`覆盖：

- 纯逻辑：各门控条件（无比赛状态、FROZEN/HANDOVER/ESTOP阶段、急停、边缘失联、输入超时）都输出零速度；不要求比赛状态时急停仍生效；手动输入只在允许时生效且优先；按比例限幅保持曲率。
- ROS通信（P1计划2-04）：`cmd_vel`只有一个发布者；输入停止、急停、阶段切到FROZEN、心跳停止时1 s内输出零速度；旧`generation`的急停释放被忽略、新的释放后恢复；超速被按比例限幅；非开发模式忽略手动输入；非开发模式下禁止关闭比赛状态要求。

2026-10-24仿真实测（无界面Gazebo，robot_1）：

| 情况 | 结果 |
|---|---|
| `require_competition_state:=false`，手动指令0.3 m/s持续2 s | 被限到0.15 m/s，移动约0.22 m；停止发送后1 s内停车，之后仅移动约2 mm |
| `require_competition_state:=true`，无比赛状态 | 不移动 |
| 两种情况下`/robot_1/cmd_vel`的发布者 | 1个（看门狗） |
| `run_mode:=competition` | 启动失败，提示`aruco.dictionary`等官方参数未设置 |

`test/test_robot_agent.py`覆盖救援动作：到达后保持直到被释放（`RELEASED_BY_COORDINATOR`）、保持超时、被推离后重新逼近（沿车身方向被推时直线倒车、不发Nav2目标，横向被推时用Nav2）、保持中阶段变化、到达容差为0被拒、逼近失败（BLOCKED）和两次逼近仍超差（FAILED）；以及纯逻辑：模式优先级、Nav2错误码映射、容差和协方差判定、直线回位判定；以及使用模拟Nav2/TF/里程计/AMCL的行为测试：状态内容、导航成功与NAVIGATING模式、EXPLORING模式、各类拒绝（`run_id`、地图版本、未定位、Nav2未active、FROZEN）、Nav2错误105→BLOCKED和208→FAILED、精确逼近两次后仍超差→FAILED、取消、阶段变化、急停及旧`generation`被忽略、超时、停不下来时报告`COMMAND_TIMEOUT`、激光中断报告`SENSOR_UNAVAILABLE`。

阶段2验收（2026-10-24，无界面Gazebo，robot_1 + 真实Nav2/AMCL，测试脚本扮演边缘节点发布比赛状态）：

| 项目 | 结果 |
|---|---|
| 精确目标，容差0.08 m | SUCCEEDED；与Gazebo真实位置误差0.14 m（含AMCL定位误差） |
| 不可达目标（障碍物内部） | FAILED，但等到60 s默认超时才返回（见已知限制） |
| 行驶中取消 | CANCELLED（`USER_REQUEST`），返回时已停稳 |
| 3 s超时 | FAILED（`NAVIGATION_TIMEOUT`），返回时已停稳 |
| 行驶中切到FROZEN | CANCELLED（`PHASE_CHANGED`），0.30 s后停车 |
| 行驶中边缘心跳中断 | 1.00 s后停车（等于失联判定时间），模式变为SAFE |
| 全程`/robot_1/cmd_vel`发布者 | 1个（看门狗） |

双机协同救援（2026-10-25，无界面Gazebo，robot_1和robot_2 + 真实Nav2/AMCL，测试脚本扮演边缘节点）：

| 步骤 | 结果 |
|---|---|
| 重新定位 | robot_1已知起点、robot_2位姿提示，都成功（约6 s，匹配度0.98～0.99） |
| 同时前往相距0.6 m的两个救援区（到达容差0.08 m） | 56 s内两台都进入HOLDING；与Gazebo真实位置误差0.082 m和0.089 m |
| 保持5 s | 两台漂移都是0.000 m |
| 用开发手动指令把robot_2向前推约0.34 m | 离开HOLDING，自动回到救援区并重新HOLDING（约53 s），真实误差0.125 m |
| 释放 | 两台都以`RELEASED_BY_COORDINATOR`结束，回到STANDBY |
| `/robot_N/cmd_vel`发布者 | 每台1个（看门狗） |

## 7. 已知限制

- 目前只有仿真启动；实机启动等组织方技术教程后实现（P1计划阶段5）。
- 机器人ID只能是`robot_1`～`robot_4`。
- **定位误差占用救援区余量**：代理按AMCL估计判断到达（0.08 m内），但真实误差实测到过0.125 m，接近0.15 m的开发救援区容差。裁判按真实位置判定，P4和P1需在官方容差公布后复核到达容差。
- 重新逼近：沿车身方向被推开时约12 s回位（阶段3实测）；被横向推开时仍要原地转向，靠墙时可能触发停车区而BLOCKED。
- 不可达目标在阶段3调参后24～37 s失败（之前60 s）。P2探索遇到不可达前沿时，可再配合更短的探索超时。
- 长距离目标要给足`timeout`：0.15 m/s下穿越场地加上减速区和转向常常超过60 s的默认超时（阶段3实测约50～60 s）。
- 边缘失联时代理不主动取消目标：看门狗已在1 s内停车，Nav2随后会因无进展判为受阻。
- 在P2的阶段管理器完成前，需要运动的开发测试只能用`run_mode:=development require_competition_state:=false`，或自行发布`/competition/state`。
