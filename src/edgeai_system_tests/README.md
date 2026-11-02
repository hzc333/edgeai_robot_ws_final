# edgeai_system_tests

系统测试、Mock节点和回放工具。

Mock必须使用`edgeai_robot_interfaces`的真实消息、服务和动作，语义与正式模块一致。**正式比赛启动文件禁止包含任何Mock。**

## 文件清单

| 文件 | 作用 |
|---|---|
| `edgeai_system_tests/mocks/mock_robot_agent.py` | 模拟机器人代理节点，入口`ros2 run edgeai_system_tests mock_robot_agent` |
| `launch/mock_two_robots.launch.py` | 同时启动robot_1（-1.5, -1.5）和robot_2（-1.5, -0.8）两个Mock |
| `test/test_mock_robot_agent.py` | 11项Mock行为测试（见下文） |
| `edgeai_system_tests/__init__.py`、`mocks/__init__.py` | 包入口 |
| `test/test_copyright.py`、`test_flake8.py`、`test_pep257.py` | 版权头、代码风格和文档字符串检查 |
| `setup.py`、`setup.cfg`、`package.xml`、`resource/` | `ament_python`包配置 |

## mock_robot_agent

代替P1的机器人代理，提供与正式代理相同的接口：

| 接口 | 行为 |
|---|---|
| `/robot_N/state` | `RobotState`，10 Hz，`map`坐标系，电池为NaN |
| `/robot_N/phase_ack` | `RobotPhaseAck`，2 Hz，对应当前`phase_sequence` |
| `/robot_N/execute_navigation_goal` | 先转向、再直行；到达`xy_tolerance_m`（为0时用0.10）及可选朝向容差即成功 |
| `/robot_N/execute_rescue_task` | 到达`arrival_xy_tolerance_m`后保持（`MODE_HOLDING`），直到被取消或`hold_timeout`到期；**不计时、不判定完成** |
| `/robot_N/relocalize` | 三种模式；`MODE_KNOWN_START_POSE`从已解析参数`localization.start_poses`取位姿 |

与正式代理相同的约定：

- 同一时刻只允许一个活动目标，有活动目标时新目标被拒绝。
- 接受条件：已收到`CompetitionState`，`run_id`、`map_id`、`map_revision`一致，阶段为TASK1/TASK2A/TASK2B，无急停，边缘心跳未超时，已定位，目标在`map`坐标系中。
- 阶段切到非运动阶段时，活动目标以`TERMINAL_CANCELLED`和`PHASE_CHANGED`结束；急停时原因为`SAFETY_STOP`；只接受`generation`更大的急停消息。
- 救援动作被取消时：运动允许则原因为`RELEASED_BY_COORDINATOR`，否则为`PHASE_CHANGED`或`SAFETY_STOP`。
- `CompetitionState`心跳超时后`mode=MODE_SAFE`并停止移动，故障列表出现`CODE_EDGE_CONNECTION_LOST`。

### 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `robot_id` | `robot_1` | 也应作为启动命名空间 |
| `initial_x`、`initial_y`、`initial_yaw` | 0 | `map`坐标系初始位姿 |
| `linear_speed`、`angular_speed` | 0.15、0.30 | 运动学速度 |
| `localized` | `true` | 为`false`时拒绝目标，直到`relocalize`成功 |
| `fail_navigation_after_s` | 0 | >0时，目标运行到该时间返回`NAVIGATION_FAILED` |
| `blocked_region` | 空 | 多边形顶点平铺列表`[x1, y1, x2, y2, ...]`，目标在其中时返回`TERMINAL_BLOCKED` |
| `relocalize_succeeds` | `true` | 为`false`时`relocalize`返回`RELOCALIZATION_FAILED` |
| `go_stale_after_s` | 0 | >0时，到时间后停止发布状态，模拟掉线 |
| `publish_tf` | `false` | 发布`map -> robot_N/base_footprint`，供目标融合测试 |
| `config_dir`、`run_mode` | 搜索、`development` | 读取运行时契约和起点位姿 |

### 启动

```bash
export EDGEAI_CONFIG_DIR=$PWD/config
ros2 launch edgeai_system_tests mock_two_robots.launch.py
```

故障注入示例：

```bash
ros2 run edgeai_system_tests mock_robot_agent --ros-args -r __ns:=/robot_2 \
  -p robot_id:=robot_2 -p fail_navigation_after_s:=3.0
```

## 测试

`test/test_mock_robot_agent.py`在同一进程中启动Mock和一个模拟边缘节点（发布`CompetitionState`和急停、调用动作和服务），覆盖：导航成功；非运动阶段拒绝目标；已有活动目标时拒绝第二个；注入导航失败；取消导航；急停打断导航且旧`generation`被忽略；救援保持直到被释放；保持超时；阶段变化取消目标并发送匹配`phase_sequence`的确认；心跳丢失进入`MODE_SAFE`；重新定位成功和失败。

2026-10-23：包内全部测试通过，连续运行3次结果一致。

### 已知限制

- Mock按“先转向、再直行”的运动学积分位姿，不考虑障碍物，也不模拟传感器。
- `MODE_GLOBAL`重新定位时，Mock直接认为找到当前位姿。
- 阶段确认以2 Hz持续发布当前`phase_sequence`；“阶段管理器确认后停止发送”的握手尚无对应接口。

### 运行

```bash
colcon test --packages-select edgeai_system_tests && colcon test-result --verbose
```
