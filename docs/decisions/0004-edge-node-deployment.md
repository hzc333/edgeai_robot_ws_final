# ADR-0004：机器人与边缘节点的部署和通信

- 状态：已采用（2026-11-20），待在组织方的边缘节点和 5G 网络上验证。
适用范围：机器人和边缘节点分处不同机器时（实机、组织方的 L4 测试容器、12 月的 5G 场地）

## 背景

规则 2.2、2.3 节：边缘节点由组织方提供，用于地图和状态汇总、任务协调和本地 LLM 推理；机器人、边缘节点和操作员通过组织方的本地网络通信。比赛不会故意制造丢包或限速，但通信中断时机器人必须执行预定义的安全行为。每台机器人是 Intel NUC11（带 GT 1030）。

组织方的测试边缘节点是 R1 worker-05 上的一个 GPU 容器（NVIDIA L4），只能通过 SSH 访问，多个队伍共用；正式比赛计划在 R1 反应堆大厅通过 5G 连接。

到目前为止，所有仿真都在一台机器上运行，ROS 2 用 CycloneDDS，只在本机发现（`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`）。跨机器部署从来没有测过。

## 测量

`tools/edge_traffic.py` 在中期全流程运行时读取 ROS 图，把 `/robot_N/...` 的节点（以及仿真的 `/clock` 桥）算作机器人侧，其余算作边缘侧，测量两侧之间每个话题的流量。

- 除图像外，跨两侧的数据量级是每秒几百 KiB，主要是 `/tf`、各机器人的 `odom`、`scan` 和 `state`。
- 中期的检测证据记录器 `detection_recorder` 订阅每台机器人的原始相机图像（640×480 RGB），每台约 23 MiB/s。放在边缘节点上，就要把所有图像传过网络。

`tools/edge_link.sh` 在本机用两个网络命名空间（不需要 root）模拟机器人和边缘节点，中间是一条可设延迟、抖动、丢包和带宽的链路；`tools/midterm_rehearsal.py --split` 让边缘侧和机器人侧分开启动（`task1.launch.py side:=edge|robots`）。在这条链路上比较了几种中间件配置：

| 配置 | 跨链路的流量 | 说明 |
|---|---|---|
| CycloneDDS 默认（组播） | 每个相机流约 16 MiB/s 外泄 | 一个话题在其他进程里有多个读者时，Cyclone 用组播发数据；组播路由把它送上网络，即使所有读者都在机器人本机 |
| CycloneDDS，组播只用于发现（`AllowMulticast spdp`） | 中期全流程约 1.3 MiB/s（两个方向合计） | 数据走单播，每个边缘进程各收一份 |
| CycloneDDS，完全不用组播（静态对端） | 发现开销最大 | 每个参与者要向对端的每个可能端口发发现报文 |
| Zenoh（`rmw_zenoh_cpp`，每台机器一个路由器） | 中期全流程约 0.34 MiB/s | 跨机器只走路由器之间的一条 TCP 连接，每份数据只过一次链路，在本机再分发 |

Zenoh 和组播只用于发现的 CycloneDDS 在模拟的 5G 类链路（单向 10 ms ± 3 ms、0.2% 丢包、50 Mbit/s）上都能跑完中期全流程。

链路更差时（单向 25 ms ± 10 ms、1% 丢包、20 Mbit/s），SLAM 原先在边缘节点上，Task 1 后半段 robot_1 的每个导航目标都被 Nav2 中止（控制器报 `Unable to transform robot pose into global plan's frame`）：Task 1 期间 robot_1 的 `map -> robot_1/odom` 由 SLAM 发布，这个变换晚到，robot_1 的控制闭环就断了。SLAM 移到 robot_1 后（见决定 1），同样的链路上 Task 1 完整完成（地图、两个目标、回到起始区）。

这条差链路上 Task 2A 的救援没有完成，原因是仿真本身：救援协调器的碰撞裁判直接订阅 Gazebo 的物理接触（gz-transport，不经 ROS），分开运行时这条数据也要过模拟链路，延迟时状态变为 UNKNOWN，协调器按设计停下。仿真的 `/clock`（约 430 Hz）同样要过链路。实机上没有这两样，所以差链路下的 Task 2A 结论要在真实部署上验证。

## 决定

### 1. 节点放在哪里

| 机器人（每台 NUC） | 边缘节点 |
|---|---|
| 底盘驱动、传感器、`robot_state_publisher` | `system_manager`、`event_logger`（运行记录和 rosbag） |
| Nav2、AMCL、`localization`、碰撞监视器 | `map_manager`（地图冻结、导出和地图版本；与 `system_manager` 共用运行目录） |
| SLAM Toolbox（Task 1，robot_1，由 `slam_runner` 启停） | |
| `robot_agent`、`safety_watchdog` | `exploration_coordinator` |
| `target_detector`（原始图像不出机器人） | `target_fusion` |
| 检测证据记录（需要原始图像；实机上每台机器人各记自己的） | 救援协调器、任务分配、救援验证 |
| | Task 2B 的 LLM 任务解释器和计划验证器 |

原则：用原始传感器数据（图像、深度、点云）的节点放在机器人上，只把观测结果（`TargetObservation`）、状态和位姿发给边缘节点。控制闭环全部在机器人上，不依赖链路：Nav2、碰撞监视、看门狗，以及给机器人提供 `map -> odom` 的定位（Task 2 的 AMCL，Task 1 中 robot_1 的 SLAM）。

**SLAM 在 robot_1 上。** `map_manager` 管地图权威（冻结、导出、地图版本），要和 `system_manager` 共用运行目录，所以留在边缘节点；SLAM 进程由 robot_1 上的 `/robot_1/slam_runner` 启动和结束（`map_manager slam_runner:=/robot_1/slam_runner`，服务 `start`/`stop`，状态心跳 `status`）。冻结时 `map_manager` 要等到 `slam_runner` 确认 SLAM 已结束，才把地图标为冻结；确认不了就拒绝冻结，避免 SLAM 和 AMCL 同时发布 `map -> odom`。地图仍经 `/edge/slam_map` 送到边缘节点，每次更新约 10 KB。`task1.launch.py side:=robots|edge` 按上表拆分；所有进程在一台机器上时（`side:=all`），SLAM 仍是 `map_manager` 的子进程。

### 2. 中间件：rmw_zenoh

跨机器部署使用 `rmw_zenoh_cpp`（已加入 `pixi.toml`）：

- 每台机器运行一个路由器 `rmw_zenohd`；本机的 ROS 进程连本机路由器（默认 `tcp/localhost:7447`）。
- 机器人上的路由器主动连接边缘节点的路由器：

  ```bash
  export RMW_IMPLEMENTATION=rmw_zenoh_cpp
  # 边缘节点
  ros2 run rmw_zenoh_cpp rmw_zenohd
  # 每台机器人
  ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/<边缘节点地址>:7447"]' ros2 run rmw_zenoh_cpp rmw_zenohd
  ```

理由：

- 不需要组播；5G 网络和容器网络通常不转发组播。
- 机器人到边缘节点只需要一条出站 TCP 连接。容器只开放 SSH 时，可以用 `ssh -L 7447:localhost:7447 <边缘节点>` 转发，机器人路由器连 `tcp/localhost:7447`。DDS 需要双向 UDP，做不到这一点。
- 机器人本机的大流量（图像、扫描、代价地图）留在本机，不会外泄。
- 不会和同一网络上的其他队伍互相发现：路由器只连配置里的地址，组播搜索默认关闭。

备选：网络允许组播且不能用 Zenoh 时，用 CycloneDDS 并设置 `<General><AllowMulticast>spdp</AllowMulticast></General>`，同时给每个队伍一个独立的 `ROS_DOMAIN_ID`。**不要用 CycloneDDS 的默认配置跨机器运行**：相机图像会以组播发到网络上。

单机仿真保持现状（CycloneDDS、只在本机发现）。

### 3. 时钟同步

实机没有 `/clock`，各机器用自己的系统时间。部分检查比较不同机器上的时间戳：救援协调器只接受时间戳与本机时钟相差 -0.5～2.0 s 的机器人位姿；边缘节点上的 SLAM 按时间戳查机器人的 TF。机器人和边缘节点都要用 chrony 同步到同一个时间源，偏差应在几毫秒以内。安全看门狗判断边缘心跳时用本机收到消息的时间，不受时钟偏差影响。

### 4. 通信中断

沿用 ADR-0003：`safety_watchdog` 超过 1.0 s（`edge_timeout_s`）没有收到 `CompetitionState` 就输出零速度（`edge_connection_lost`），心跳恢复后解除。Nav2 和定位继续运行，机器人停在原地。

**边缘一侧的宽限期（`link_loss_grace_s`，20 s）。** `system_manager` 在运动阶段每 0.5 s 检查一次。原来任何机器人的 `RobotState` 超过 1.0 s（`robot_state_stale`）没更新就锁存急停；急停解除后也不会恢复运动，计时赛就此结束，所以实机上一次超过约 1 s 的断网就会结束整场比赛，而机器人自己早已安全停车。现在：

- 状态晚到 1.0 s 到 1.0 s + 宽限期之间，只记一条警告（机器人已经被看门狗停住）；超过宽限期才锁存急停。从没收到过状态的机器人仍立即急停。
- SLAM 在 robot_1 上时，它的状态心跳（3 s）和地图更新（10 s）的超时同样加上宽限期。
- 这些检查改用墙钟定时器：分开运行的仿真里 `/clock` 从机器人一侧经网络过来，断网时会一起停住，检查就恰好在断网期间暂停（本机模拟中 5 s 断网曾因此“没出事”）。
- 宽限期要比断网本身长得多：Zenoh 路由器之间是一条 TCP 连接，断网期间 TCP 重发的间隔逐次加倍（8 s 的断网里，下一次重发在 12.6 s），链路恢复后还要等几秒数据才重新到达。模拟中宽限期 10 s 时 8 s 的断网仍结束了比赛，20 s 时 5 s 和 8 s 的断网都能恢复。
- Task 2A 中断网时，中期救援协调器（`edgeai_task_coordinator`）按设计中断正在停留的任务，但随后取消机器人动作的确认在断网期间到不了，协调器停在 `RELEASE_NOT_CONFIRMED`，链路恢复后也不再继续（模拟中 8 s 的断网落在 S1 停留期间时出现）。最终比赛的救援协调器要在链路恢复后重新确认释放并继续任务。
- 超过 Zenoh 默认 10 s 连接租约的断网（模拟中 15 s）会让路由器之间断开重连、重新声明话题，恢复时间超过宽限期，仍会急停。这样的长断网要人来处理，机器人在此之前早已停车。

## 待验证（在组织方的边缘节点和网络上）

1. 机器人能否直接连到边缘节点的 TCP 7447，还是只能经 SSH 转发；容器里能否运行 `rmw_zenohd`。
2. 实际链路的延迟、抖动和带宽；用 `tools/edge_traffic.py` 在真实部署上重新测量流量。
3. 机器人和边缘节点的时钟同步方式（容器内能否运行 chrony，或使用宿主机时间）。
4. 链路短暂中断（几秒）后，机器人停车、恢复，任务能否继续。
5. 链路较差时 Task 2A 的表现（本机模拟中被仿真专用的数据流掩盖，见“测量”）。

在组织方的容器上按 [`docs/edge_node_test_plan.md`](../edge_node_test_plan.md) 依次验证上面几项。

## 本地复现

```bash
pixi run bash -c 'source install/setup.bash && tools/edge_link.sh --rmw zenoh \
    --delay-ms 10 --jitter-ms 3 --loss-pct 0.2 --rate-mbit 50 -- \
    python tools/midterm_rehearsal.py --split'
```

`--rmw` 可选 `cyclone`、`cyclone-spdp`、`cyclone-unicast`、`zenoh`；结束时打印两个方向跨链路的字节数和包数。机器人侧的启动日志在 `runs/midterm-rehearsal/<run_id>.robots.launch.log`。
