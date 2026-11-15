# 系统架构与接口协议

状态：基础协议 v0.2（接口包0.2.0，变更见`src/edgeai_robot_interfaces/CHANGELOG.md`）  
范围：仿真和实机、Task 1、Task 2A及Task 2B

## 1. 设计原则

1. 仿真和实机使用同一套算法代码，只允许驱动、标定和启动配置不同。
2. 每台机器人使用独立命名空间。机器人本地节点不得发布未加命名空间的控制、里程计、传感器或动作端点。
3. `map`是共享比赛坐标系。原始观测只有在使用正确时间戳成功完成TF转换后，才能成为全局目标。
4. LLM只负责生成任务语义。机器人选择、导航、避障、停留验证和运动控制由确定性模块完成。
5. 官方标为TBD的数值只能进入配置，不能成为代码常量。
6. 安全指令优先于任务执行。`ESTOP`、阶段冻结和通信中断处理必须可重复调用且结果一致。
7. 与得分有关的状态变化使用带时间戳的只追加事件；当前状态应能从事件流重建。

## 2. 部署边界

### 每台机器人上运行

- 实机驱动或Gazebo驱动。
- `robot_state_publisher`和机器人局部TF树。
- 传感器预处理。
- 本地里程计和定位客户端。
- Nav2与局部避障。
- 发布原始`TargetObservation`的目标检测器。
- 提供`ExecuteRescueTask`的机器人执行器。
- 本地看门狗和急停。

### 边缘节点上运行

- SLAM/地图汇总和探索协调器。
- 目标融合和权威目标注册表。
- 多机器人状态注册表。
- 任务分配器和救援协调器。
- 协同停留验证器。
- 本地LLM任务解释器和确定性计划验证器。
- 比赛阶段管理器。
- 事件日志器、文件导出器、监控界面和回放工具。

所有影响得分的关键功能优先采用本地执行。当前规则暂定禁止正式计时期间使用外部云端AI，因此比赛流程不能依赖互联网。

## 3. 命名与身份标识

### 机器人ID和命名空间

- 标准机器人ID为`robot_1`、`robot_2`、`robot_3`和`robot_4`。
- 命名空间与ID完全一致，例如`/robot_2`。
- 节点名称描述功能，不重复命名空间，例如`/robot_2/target_detector`。
- `run_id`、`mission_id`、`task_id`、`step_id`、`event_id`和`target_id`均为不透明字符串。生产方只生成一次，消费方不得解析其内部格式。
- 从准备阶段到Task 2B和最终导出，一次比赛运行使用同一个不可变`run_id`。
- `/edge/system_manager`在进入`PREPARATION`时生成唯一`run_id`，其他节点只能读取和传递，不能自行生成替代值。
- `/edge/map_manager`在新建图会话开始时生成`map_id`。`map_revision`只在地图被重置、替换或生成权威持久化快照时递增；`target_revision`在权威目标注册表每次接受修改时递增。

### Topic与端点规则

- 机器人本地端点位于`/<robot_id>/...`。
- 边缘节点权威端点位于`/edge/...`。
- 裁判/操作员端点位于`/competition/...`。
- 可复用机器人节点内部禁止写死绝对Topic名；命名空间和重映射由启动文件提供。

## 4. 坐标系

每台机器人使用以下逻辑TF链：

```text
map -> robot_N/odom -> robot_N/base_footprint -> robot_N/base_link
                                              -> robot_N/lidar_link
                                              -> robot_N/camera_link
                                              -> robot_N/camera_optical_frame
```

规则如下：

- `map`为共享坐标系，不增加机器人前缀。
- `odom`及所有车体、传感器坐标系必须带机器人前缀。
- 遵循REP-103：机器人坐标为`x`向前、`y`向左、`z`向上；光学坐标为`x`向右、`y`向下、`z`向前。
- 提交的所有目标和救援区位姿必须位于`map`。
- 所有导航目标使用`map`坐标系的`geometry_msgs/PoseStamped`。
- TF查询必须使用观测时间戳，不能直接使用最新变换代替。
- TF缺失、发生外推或已经过期时，应拒绝观测或在有限时间内重试；不得使用当前机器人位姿静默替代。
- 任一时刻只能有一个全局坐标系发布源。建图阶段由SLAM发布`map -> odom`；静态地图阶段由选定的定位模块发布。

当前单机器人基线使用未加前缀的坐标系。在同一ROS Domain运行两台机器人之前，必须先完成TF前缀改造。

## 5. 时间与单位

- ROS时间戳是权威事件时间。
- 仿真节点使用`use_sim_time=true`；实机使用同步后的主机时间。一次运行中不得混用两种时间源。
- 持久化时间使用UTC RFC 3339格式；条件允许时保留纳秒精度。
- 所有数据使用SI单位：米、秒、弧度、米每秒。
- Yaw归一化到`[-pi, pi)`。
- 停留计时使用ROS/单调时钟Duration，不能通过解析墙上时钟字符串计算。

## 6. 共享ROS 2接口

`edgeai_robot_interfaces`是共享接口的唯一权威来源。主要数据流为：

```text
TargetObservation -> 目标融合 -> TargetArray -> Task 1导出
RobotState --------> 状态注册表 ------------------------+
TargetArray --------------------------------------------+-> 协调器
SubmitMission -> 解释器 -ValidateMission-> 验证器 -MissionPlan-+      |
                   |                                          v
                   +-> MissionIntakeResult          RescueTask/Event、MissionExecutionState
                                                              |
                                                 ExecuteRescueTask（到达并保持）
```

Marker、目标和救援区的几何约定见[Marker坐标约定与救援区几何](marker_and_rescue_geometry.md)。

### Topics

| 名称 | 类型 | 发布方 | 消费方 | QoS |
|---|---|---|---|---|
| `/<robot_id>/target_observations` | `TargetObservation` | 检测器 | 目标融合 | 传感器数据配置 |
| `/<robot_id>/state` | `RobotState` | 机器人代理 | 状态注册表/协调器 | Reliable，Keep Last 1 |
| `/<robot_id>/phase_ack` | `RobotPhaseAck` | 机器人代理 | 阶段管理器/日志器 | Reliable，Keep Last 1 |
| `/edge/robot_states` | `RobotStateArray` | 状态注册表 | 协调器/界面/日志器 | Reliable，Keep Last 1 |
| `/edge/exploration_status` | `ExplorationStatus` | 探索协调器 | 阶段管理器/界面/日志器 | Reliable，Transient Local |
| `/edge/targets` | `TargetArray` | 目标融合 | 协调器/界面/导出器 | Reliable，Transient Local，Keep Last 1 |
| `/edge/rescue_tasks` | `RescueTask` | 协调器 | 界面/日志器 | Reliable，Transient Local |
| `/edge/rescue_events` | `RescueEvent` | 协调器/验证器 | 日志器/界面 | Reliable，有限深度Keep All |
| `/edge/mission_plan` | `MissionPlan` | 任务验证器（唯一发布方） | 协调器/日志器/界面 | Reliable，Transient Local |
| `/edge/mission_intake` | `MissionIntakeResult` | 任务解释器 | 日志器/界面 | Reliable，Transient Local |
| `/edge/mission_state` | `MissionExecutionState` | 任务分配器 | 日志器/界面 | Reliable，Transient Local，Keep Last 1 |
| `/competition/state` | `CompetitionState` | 阶段管理器 | 所有节点 | Reliable，Transient Local |
| `/competition/estop` | `EmergencyStop` | 安全管理器 | 所有机器人/协调器 | Reliable，Transient Local |

相机和LiDAR使用标准Sensor Data QoS。命令、计划、状态快照和审计事件使用Reliable。需要新订阅者立即获得当前值的快照使用Transient Local。审计记录不能只依赖DDS历史，日志器必须将每个接受事件写入磁盘。

### Actions与Services

| 名称 | 类型 | 用途 |
|---|---|---|
| `/<robot_id>/execute_rescue_task` | `ExecuteRescueTask`动作 | 边缘节点向机器人下发救援并接收进度 |
| `/<robot_id>/execute_navigation_goal` | `ExecuteNavigationGoal`动作 | 探索/协调模块向机器人代理发送通用导航目标 |
| `/<robot_id>/relocalize` | `Relocalize`服务 | 在已保存地图上（重新）定位，由机器人定位封装提供 |
| `/competition/set_phase` | `SetCompetitionPhase`服务 | 操作员或自主模块按跳转表受控切换比赛阶段 |
| `/competition/request_reset` | `RequestReset`服务 | 裁判批准的两步式重置 |
| `/edge/finalize_task1` | `FinalizeTask1`服务 | 冻结并原子化导出Task 1结果 |
| `/edge/submit_mission` | `SubmitMission`服务 | 提交组织方提供的Task 2B自然语言指令，异步返回结果 |
| `/edge/validate_mission` | `ValidateMission`服务 | 解释器把候选计划交给验证器，验证器接受后发布计划 |

Nav2标准`NavigateToPose`动作只在机器人代理内部使用；边缘协调器不直接调用Nav2。这样，恢复、安全和机器人特有导航细节都封装在机器人执行器内部。

具体QoS深度、发布频率、数据过期时间、ID生成者、权威生产节点、阶段跳转表和安全确认时限以`config/interface_runtime.yaml`为准。`/diagnostics`使用标准`diagnostic_msgs/DiagnosticArray`，供启动就绪检查汇总，避免重复定义通用诊断消息。

## 7. 持久化文件

每次运行保存在独立目录：

```text
runs/<run_id>/
  manifest.json                      # schemas/manifest.schema.json
  events.jsonl                       # schemas/event_record.schema.json，全运行唯一事件流
  task1/map.yaml
  task1/map.pgm
  task1/targets.json                 # schemas/target_list.schema.json
  task2b/original_instruction.txt
  task2b/mission_intake.json         # schemas/mission_intake.schema.json，含模型原始输出与拒绝原因
  task2b/mission_plan.json           # schemas/mission_plan.schema.json，仅验证通过时存在
  system/parameters.yaml             # 已解析参数，含rehearsal/development回退项
  system/component_versions.json     # schemas/component_versions.schema.json
  system/rosbag/
```

阶段、急停、重置和救援事件跨越多个阶段，因此只保留一个按`seq`全序的`events.jsonl`；每条记录带`phase`和`scoring_stage`，提交材料可按阶段筛选导出。多条Task 2B指令时，`mission_intake.json`按`mission_id`拆分为`task2b/<mission_id>/`子目录。

`manifest.json`记录所有得分相关文件的哈希和相对路径。只有地图、元数据、目标列表和Manifest全部写盘并验证成功后，`FinalizeTask1`才可以返回成功。阶段冻结后不能修改文件；任何修正都必须建立新版本并记录审计事件。

JSON格式全部由`schemas/`下的Schema定义，样例位于`examples/interfaces/`。

## 8. 目标融合协议

1. 检测器对每次Marker检测发布一条相机光学坐标系观测，其中包含图像时间戳、协方差和置信度。
2. 融合节点在观测时间将其转换到`map`。
3. 未知Marker ID可以作为诊断观测保留，但除非官方ID映射能够识别，否则不能成为有效救援目标。
4. 使用官方目标ID进行数据关联；与现有目标估计空间上明显冲突的观测应被拒绝。
5. 以确定性方式更新融合位置/朝向、协方差、置信度、观察次数和观察机器人ID。
6. Task 1冻结时，权威列表设置`eligible_for_basic_rescue=true`。Task 2中的新观察可以修正已知目标，但新发现目标不能获得基本救援资格。
7. 救援区由Marker位姿和官方几何规则推导，检测器不得猜测救援区位置。几何约定、目标坐标系和yaw定义见`marker_and_rescue_geometry.md`。

所有阈值从配置加载。在官方Marker及救援几何参数公布前，对应官方字段保持`null`，比赛模式必须拒绝进入可运动状态。

## 9. 任务计划协议

LLM输出属于不可信输入。确定性验证器必须检查：

- Schema版本及所有ID是否唯一。
- 每个引用目标是否存在于冻结目标注册表。
- 目标类型是否与注册表一致。
- 每个前置步骤是否存在。
- 依赖图是否无环。
- 是否错误要求同一台机器人占据两个协同救援区。
- 当前可用机器人能力能否完成该计划。
- LLM输出中是否包含不允许的速度、路径、位姿或机器人分配。

通道与权威归属：

1. 操作员通过`/edge/submit_mission`提交指令，解释器立即返回`mission_id`。
2. 解释器调用模型。模型不可用、超时或输出无法结构化时，直接发布`OUTCOME_INTERPRETATION_FAILED`的`MissionIntakeResult`。
3. 否则解释器以`valid=false`的候选`MissionPlan`调用`/edge/validate_mission`。验证器不得修改步骤，只能接受或给出`MissionValidationIssue`列表。
4. 验证器接受时先以`valid=true`发布到`/edge/mission_plan`，再返回响应；验证器是该Topic唯一发布方。
5. 解释器根据响应发布最终`MissionIntakeResult`（接受或被拒绝），日志器据此写`mission_intake.json`。

无效计划不会出现在`/edge/mission_plan`上，系统保持安全。协调器负责把已经满足前置条件的任务步骤转换为带`mission_id/step_id`、`scoring_stage=STAGE_TASK2B`的`RescueTask`，在运行时分配机器人，并通过`/edge/mission_state`发布每个步骤的执行状态。

## 10. 状态机

### 比赛阶段状态机

```text
PREPARATION -> TASK1 -> HANDOVER -> TASK2A -> FROZEN -> TASK2B -> FINISHED
      \_____________________________________________________________/
                               ESTOP
```

每个阶段都可以进入`ESTOP`。合法跳转及允许的请求方（操作员`OPERATOR`或自主模块`SYSTEM`）以`config/interface_runtime.yaml`的`phase_transitions`为准，非法阶段跳转必须拒绝。自主模块只能请求结束当前阶段：探索协调器在探索完成时请求`TASK1 -> HANDOVER`，任务分配器在全部有效目标完成时请求`TASK2A -> FROZEN`，任务分配器在已验证计划全部完成时请求`TASK2B -> FINISHED`。

当前规则对Task 2A到Task 2B是否重摆机器人存在冲突，因此阶段管理器应在不预设结果的情况下支持可配置的交接和重新定位过程：进入`HANDOVER`和`FROZEN -> TASK2B`之前，阶段管理器对每台在线机器人调用`relocalize`，全部成功（或失败机器人被标记不可用）后才允许跳转。

### 运行模式

`CompetitionState.run_mode`取`development`、`rehearsal`或`competition`，由启动参数显式指定，并在整个运行中不变。参数解析规则见`config/competition_parameters.yaml`头部注释：比赛模式只使用`official`值，`run_modes.required_for_motion`中任一项为`null`时阶段管理器拒绝离开`PREPARATION`并报告`CODE_OFFICIAL_PARAMETER_MISSING`；演练和开发模式允许回退到`development`值，但回退项必须写入`system/parameters.yaml`。节点只读取启动时生成的已解析参数，不得各自实现回退。

### 重置流程

1. 裁判批准后，操作员以`ACTION_BEGIN`调用`/competition/request_reset`。比赛模式下`official.reset`未确定时拒绝。
2. 阶段管理器递增`reset_count`，置`reset_in_progress`，指定机器人取消动作并进入`MODE_SAFE`；协调器把其任务标记为`RESET_REQUESTED`后重新分配。
3. 人工放回机器人后，以`ACTION_COMPLETE`调用；阶段管理器调用`relocalize`，成功后机器人才重新成为分配候选。
4. 每一步都写入`category=reset`事件；地图、目标和已完成救援是否保留按`official.reset`执行，重置不能补全缺失信息或直接完成救援。

### 救援任务状态机

```text
PENDING -> READY -> ASSIGNED -> NAVIGATING
                                  |      \
                                  |       -> FAILED/BLOCKED
                                  v
                     WAITING_FOR_PARTNER（协同任务）
                                  |
                                  v
                              DWELLING -> COMPLETED
```

`/edge/rescue_verifier`是救援区占用、停留计时和完成判定的唯一权威：它使用`/edge/robot_states`中的`map`位姿和`zone_tolerance_m`判断占用，使用ROS时间计时。机器人执行器只负责“到达救援位姿并保持静止”（`MODE_HOLDING`），不计时、不判断是否完成；验证器发布`EVENT_COMPLETED`后，协调器取消对应动作以释放机器人。机器人动作结果不是得分证据。

机器人离开有效救援区后，停留进度清零。对于协同救援，只有两台不同的已分配机器人同时占据各自救援区时，边缘验证器才开始计时。完成事件只能产生一次，重复消息不会重复计分；同一目标在`STAGE_TASK2A`和`STAGE_TASK2B`中各自独立计数。

## 11. 安全协议

- 速度命令超时后，本地看门狗发送零速度。
- 边缘连接过期后，机器人停车并进入`MODE_SAFE`。
- 阶段冻结会取消探索和导航，发送零速度，并等待所有机器人确认安全状态。
- 急停始终优先于导航或任务动作。
- 恢复行为不能为了赶任务时间而穿越未知或占据区域。
- 对已知的碰撞里程计漂移，应通过受阻/接触检测、停车和恢复处理，不能继续积分指令运动。

### 阶段确认协议

1. 阶段管理器递增`phase_sequence`并发布新的`CompetitionState`。
2. 每台在线机器人取消与新阶段不兼容的动作，并在需要时发送零速度。
3. 机器人持续发布相同`phase_sequence`的`RobotPhaseAck`，直到阶段管理器确认。
4. 冻结阶段只有在全部在线机器人报告`safe=true`后才完成。
5. 超过`config/interface_runtime.yaml`中的确认时限仍未响应的机器人被标记为故障；系统不得假装其已经停车。

### 急停协议

1. `/competition/estop`独立于任务协调器，由安全/阶段管理器发布并使用Transient Local保存最后状态。
2. 每次改变急停状态时，`generation`必须递增。机器人忽略更旧的generation。
3. `active=true`后，机器人立即取消动作、发送零速度并进入`MODE_ESTOP`。
4. 释放急停必须发布`active=false`和新的generation。释放不会直接恢复运动：若被中断阶段是`PREPARATION`、`FROZEN`或`FINISHED`则回到该阶段，否则进入`FROZEN`并在`interrupted_phase`中保留被中断阶段；是否继续由裁判决定，操作员再请求恢复。
5. 软件急停不能替代实机物理急停；物理急停接入方式在机器人代理的实机配置中定义。

## 12. 接口验收检查

只有满足以下条件，接口集成才可验收：

1. 两台仿真机器人同时运行时没有Topic或TF冲突。
2. 所有得分相关消息包含相同`run_id`。
3. 带时间戳观测能够转换到`map`；不能转换时记录明确拒绝原因。
4. 订阅节点重启后可以恢复当前目标、任务和任务计划快照。
5. 重复完成消息或事件不会造成重复得分。
6. 状态过期机器人会从分配候选中移除并进入安全停车。
7. Task 1导出结果通过JSON Schema验证。
8. 无效或循环依赖任务计划被拒绝，且不会触发机器人运动。
9. 冻结和急停能够在不依赖协调器当前任务状态的情况下停止所有机器人。
10. 每次阶段切换都收到所有在线机器人的匹配`phase_sequence`确认。
11. 地图、目标注册表、任务和LLM计划使用一致的`map_id/map_revision/target_revision`。
12. 每条Task 2B指令都有`MissionIntakeResult`记录；被拒绝的计划不出现在`/edge/mission_plan`上。
13. 所有持久化文件和样例通过`tools/validate_project_contracts.py`的Schema及语义检查。
14. 比赛模式下必需的官方参数缺失时，系统拒绝离开`PREPARATION`。
