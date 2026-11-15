# ADR-0003：速度安全链与运动门控

- 状态：已采用（2026-10-23）。
适用范围：仿真和实机的全部机器人

## 背景

当前任何节点都能直接向底盘速度话题发布命令，而Gazebo会一直执行最后一条速度命令，所以“停止发布”不等于“停车”。规则要求系统必须有有效的停止机制（7.4），通信中断时执行预定义的安全行为（2.3），计时期间不得遥控（7.1）。架构文档第11节要求急停优先于一切导航和任务动作。

## 决定

### 1. 速度链

```text
Nav2 controller_server ─cmd_vel_nav─> velocity_smoother ─cmd_vel_smoothed─> collision_monitor ─cmd_vel_safe_in─┐
                                                                                                                ├─> safety_watchdog ─cmd_vel─> 底盘
开发手动控制（仅development模式）────────────────────────────────────────────────────────cmd_vel_manual────────┘
```

- `/robot_N/safety_watchdog`是`/robot_N/cmd_vel`的**唯一**发布者，以固定20 Hz发布；门控生效时持续发布零速度。
- 其他任何节点（包括边缘端节点和调试脚本）都不能发布`/robot_N/cmd_vel`。
- `robot_agent`不直接发布速度；它通过取消Nav2目标让上游停止输出，再由看门狗的输入超时兜底。

### 2. 门控规则

满足任一条件时，看门狗输出零速度：

| 条件 | 判定 | 诊断信息 |
|---|---|---|
| 尚未收到`CompetitionState` | 启动后的默认状态 | `waiting_for_competition_state` |
| 阶段不允许运动 | 阶段不是`TASK1`、`TASK2A`、`TASK2B` | `phase_<名称>` |
| 急停生效 | 最新（`generation`最大）的`EmergencyStop.active=true` | `estop` |
| 边缘失联 | 超过`timeouts_s.edge_connection`（1.0 s）没有收到新的`CompetitionState` | `edge_connection_lost` |
| 输入超时 | 超过`timeouts_s.velocity_command`（0.5 s）没有收到速度输入 | `input_timeout`（正常现象） |

补充规则：

- `generation`不大于已收到最大值的`EmergencyStop`一律忽略。
- 速度上限取已解析参数中的`motion.max_linear_speed_mps`和`motion.max_angular_speed_radps`。超限时线速度和角速度按同一比例缩小，保持曲率不变。
- `cmd_vel_manual`只在`run_mode=development`时生效，有手动输入时优先于自动输入。
- 参数`require_competition_state:=false`（不需要阶段管理器即可运动）只在`run_mode=development`下允许，用于单独调试导航；其他模式下设置该参数，节点报错退出。

### 3. 碰撞监视器

`nav2_collision_monitor`以`/robot_N/scan`和`/robot_N/peer_footprints`（其他机器人的足迹，见`edgeai_robot_navigation`的README 4.1节；其他机器人的底盘低于LiDAR扫描平面）为数据源，设置停车多边形（足迹外扩0.05 m）和减速多边形（外扩0.20 m，减速比0.5）。它位于看门狗之前，只能降低或清零速度，不能绕过看门狗。

## 原因

- 唯一出口让“是否停车”只取决于一个节点的一组明确规则，可以用测试完整覆盖。
- 门控不依赖任务协调器或Nav2是否正常，满足架构文档验收条件9（冻结和急停不依赖协调器当前任务状态）。
- 输入超时兜底覆盖了上游节点崩溃、网络中断和发布者被关闭的情况。

## 替代方案

- **只依赖Nav2的停止行为**：Nav2崩溃或网络中断时无法保证停车。不采用。
- **使用`twist_mux`**：能做多输入优先级和超时，但不理解比赛阶段和急停`generation`语义，仍需额外节点。不采用。

## 影响

| 对象 | 影响 |
|---|---|
| 阶段管理 | `/competition/state`需以`interface_runtime.yaml`规定的1 Hz心跳发布，否则机器人会判定边缘失联并停车 |
| 救援协调 | 救援“保持”期间的零速度由看门狗保证，协调器不需要也不能发布速度 |
| 所有模块 | 调试时不能直接向`/robot_N/cmd_vel`发布，应在开发模式下使用`/robot_N/cmd_vel_manual` |

## 验收

1. `ros2 topic info -v /robot_N/cmd_vel`只显示一个发布者。
2. 以下每种情况，`cmd_vel`都在1 s内变为零：输入停止发布；急停生效；阶段切到`FROZEN`；`CompetitionState`心跳停止。
3. 旧`generation`的急停释放消息被忽略。
4. 超速输入被按比例限幅。
5. 非开发模式下，手动输入被忽略。
