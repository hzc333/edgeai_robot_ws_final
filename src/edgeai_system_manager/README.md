# edgeai_system_manager

比赛阶段、结果冻结、重置编排和运行证据。所有代码均使用仓库共享 msg/srv/action；
不实现底盘控制、视觉识别或救援调度。

## 文件与职责

- `storage.py`：运行根目录独占锁、安全路径、fsync 原子写、Schema 验证、唯一 JSONL
  写者与事件 ID 去重；损坏的日志尾部会拒绝恢复，不静默丢弃证据。
- `phases.py`：运行时契约的合法跳转、请求方权限、急停释放和精确阶段确认规则。
- `bootstrap.py`：统一参数解析、配置哈希、组件版本与初始 Manifest；比赛官方参数
  缺失时仍允许观察 PREPARATION，但不能运动。
- `manager.py`：system_manager 节点；持久化阶段、心跳、地图版本确认、Finalizetask1、
  重新定位、重置与超时。长服务等待不阻塞心跳；急停可抢占正在执行的操作。
- `freeze.py`：PGM/YAML 编码、目标几何检查、冻结事务、文件哈希和崩溃恢复。
- `event_logger.py`：events.jsonl 唯一写者，保存控制/救援/任务证据、LLM 原始输出、
  rosbag MCAP 分段录制；FINISHED 后封存，登记最终文件。
- `audit.py`：只读检查哈希、Schema、事件序列及地图元数据；导出经验证的运行包。
- `mock_targets.py`：明确启用的 development 空目标注册表，仅供 P2 联调。不能与
  P3 target_fusion 并存，不能据此宣称视觉感知通过。
- `validate_sim.py`：隔离 DDS 域的真实 Gazebo 自动验收驱动，保留日志和采样报告。
- `launch/task1.launch.py`：运行身份、传感器、SLAM、导航、P2 节点和可选 Mock 的入口。
- `test/test_core.py`：持久化、损坏检测、阶段规则、地图编码、冻结失败恢复。
- `test/test_manager_ros.py`：真实 ROS 服务通信与 P1 Mock 配合的控制面集成测试。
- `test/test_quality.py`：风格、版权。
- `setup.py`、`setup.cfg`、`package.xml`、`resource/`：ament_python 安装配置。

## 快速使用

```bash
pixi run build
pixi shell
source install/setup.bash
export ROS_DOMAIN_ID=74
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 launch edgeai_system_manager task1.launch.py run_id:=new mock_targets:=true
```

`mock_targets:=true` 仅限没有真实 P3 的开发演练。正式联调保持默认 false。
`run_id:=new` 明确创建新运行；省略 run_id 时沿用 `runs/active_run.txt`，防止重启
静默生成新身份。每次正式比赛使用独立运行。默认 headless=true，可设 rviz=true。
`robots:=robot_1,robot_2` 让第二台接入但不建图；`simulation:=false` 时外部 P1 驱动
必须先准备好同名真实传感器和导航端点。实机启动仍需 P1 的驱动配置。

另一终端同样激活环境和 DDS 域后，读取 runs/active_run.txt 的真实 ID，通过服务
发送 OPERATOR 请求进入 TASK1；不能直接伪造 `/competition/state`。

```bash
ros2 service call /competition/set_phase edgeai_robot_interfaces/srv/SetCompetitionPhase   "{run_id: '<实际run_id>', requested_phase: 1, requester_type: 1, requester_node: operator, reason: start}"
```

自主探索结束会请求 HANDOVER。冻结地图和 P3 的 task1_frozen 目标快照必须在
run_id/map_id/map_revision/target_revision 上一致，随后调用 `/edge/finalize_task1`。
该服务拒绝缺失目标快照；即使目标数量是零，也需要显式的权威空快照。
操作员再请求 TASK2A，管理器用最终 SLAM 位姿提示激活 robot_1 的 AMCL，其他机器人
使用全局定位。重定位失败会停留在安全阶段，不恢复运动。

## 运行与安全语义

- `/competition/state`、`/competition/estop` 的唯一生产者是 system_manager。
- 心跳默认 5 Hz（可配置 heartbeat_hz，最低为契约的 1 Hz）。额外频率给 P1 当前
  1 秒失联阈值留出抖动余量，不改变消息或其他节点的超时配置。
- 安全阶段等待所有配置机器人新鲜、匹配 run_id/phase_sequence 的安全确认；不会把
  空机器人集合视为成功。地图版本更新也递增确认序号，再交给定位/导航消费。
- 急停释放只进入安全阶段；操作员必须显式请求恢复。阶段计时累计保留，不会因
  急停恢复重新获得完整任务时间。重置是否继续计时使用统一参数。
- 为适配当前 P1 不按 resetting_robot_ids 门控的实现，重置保守地停止整组机器人。
  BEGIN 等待停稳，COMPLETE 在冻结地图上重新定位，ABORT 仍保持停车。
- 比赛重置参数缺失时拒绝；要求清空地图/目标/救援结果的官方策略仍需跨模块清理
  协议，当前明确拒绝，不能冒充已实现。
- 运行中重启进入 ESTOP；未冻结 SLAM 会话不会自动从头覆盖旧地图。冻结地图可恢复。
- Task 1 的冻结文件不可变；FINISHED 才封存全运行 events.jsonl 与 MCAP/Task 2B 文件。

## 产物与核查

运行目录：`runs/<run_id>/`（中期演练为 `runs/midterm-rehearsal/<run_id>/`，P2 自动验收为
`runs/p2-validation/<run_id>/`）：

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

控制事件与状态在同一个原子快照持久化，日志器恢复后去重补写。Task 1 事务采用临时
目录和提交意图；崩溃后验证全部哈希再恢复 Manifest。目标位姿使用 map、米、xyzw。

```bash
ros2 run edgeai_system_manager audit_run runs/<run_id>
ros2 run edgeai_system_manager audit_run runs/<run_id> --finished --export /tmp/p2-run.tar.gz
ros2 run edgeai_system_manager validate_sim --timeout 660
```

验证驱动只发出开始请求，探索点由算法自主选择；之后测试冻结、AMCL 交接、回到起点
导航和日志封存。结果保存在 runs/p2-validation/，失败时记录错误和日志，不声称通过。

当前不保证任意未知场景或真机一次通过。P3 真正的冻结联调、正式规则和实体机器人
验收必须单独完成。首次实现阶段禁止据现有代码数量代替这些验收证据。


## 本轮可靠性补充

启动不依赖固定秒数：wait_mapping 确认地图与 TF 后才启动导航。冻结地图恢复则先
加载地图，再交给 AMCL 建立定位 TF。每轮仿真使用独立 Gazebo 分区。
日志器退出、MCAP 录制进程死亡、证据校验失败或 SLAM 地图超时均阻止继续运动。
最终产物包含源文件哈希和源代码快照，便于复现尚未提交的开发版本。
地图质量面积保存在 task1/map_quality.json；未提供权威场地面积时覆盖率为 null。
system/final_state.json 是不可变终态证据，system/state.json 保留实时安全控制状态。

## 用 RViz 查看运行

自动演练运行时，另开终端（DDS 域与演练一致，中期演练默认 77）：

```bash
pixi shell
source install/setup.bash
export ROS_DOMAIN_ID=77
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
rviz2 -d "$(ros2 pkg prefix --share edgeai_robot_description)/rviz/multi_robot.rviz" --ros-args -p use_sim_time:=true
```

Map 显示项的 Durability Policy 设为 **Transient Local**、Reliability 设为 **Reliable**，
这样冻结后再打开也能收到地图。不要用 RViz 的 2D Goal Pose 代替导航动作接口。

需要手动急停时，通过同一服务请求 ESTOP（急停解除不会自动恢复运动）：

```bash
ros2 service call /competition/set_phase edgeai_robot_interfaces/srv/SetCompetitionPhase "{run_id: '<实际run_id>', requested_phase: 7, requester_type: 1, requester_node: operator, reason: manual_stop}"
```

## 常见问题

- **停在 PREPARATION、不能开始**：查看服务响应中的缺失官方参数、机器人状态、地图或日志器就绪原因。
- **HANDOVER 卡住**：检查安全确认的 run_id/序号、地图冻结状态，以及是否有匹配版本的冻结目标快照。
- **返回 busy**：长操作正在进行；等待并有界重试，不直接修改状态或重复启动管理器。
- **RViz 看不到地图**：检查 DDS 域、use_sim_time、Map 勾选、`map` 固定坐标系与 Transient Local。
- **Nav2 error 104 / 无进展**：结合地图、机器人姿态、碰撞监视器停车区与导航日志检查；不要关闭碰撞监视器。
- **旧世界或重复时钟**：检查 GZ_PARTITION、启动日志及本轮 PID，清理残留的 Gazebo。
- **源代码变化或哈希冲突**：保留旧运行证据，显式新建运行（`run_id:=new`）。
