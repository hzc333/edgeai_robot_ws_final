# ROS包与节点布局

本文固定包名、公开节点名和依赖方向。包名和公开节点名发生变化时，必须按照接口修改规则审核。

## 1. 依赖方向

```text
edgeai_robot_interfaces
        ^
        |
bringup/navigation/exploration/perception/fusion/coordinator/mission/system
```

`edgeai_robot_interfaces`不得依赖任何业务包。感知、探索和协调模块不得直接依赖仿真模型；通过标准ROS接口与机器人代理通信。

## 2. 包

| 包 | 状态 | 主要节点/内容 | 禁止包含 |
|---|---|---|---|
| `edgeai_robot_interfaces` | 已实现 | 共享msg/srv/action | 算法和运行节点 |
| `edgeai_common` | 已实现 | 运行模式参数解析、QoS、帧名工具；无运行节点 | 业务逻辑和运行节点 |
| `edgeai_robot_description` | 已实现 | URDF、世界、传感器、基础仿真 | 任务逻辑 |
| `edgeai_robot_bringup` | 已实现 | `robot_agent`、多机器人启动、驱动配置、安全看门狗 | 探索和任务分配 |
| `edgeai_robot_navigation` | 已实现 | Nav2参数、定位、恢复、动作适配器 | 全局机器人分配 |
| `edgeai_exploration` | 已实现 | `exploration_coordinator`、Frontier、地图管理 | 电机控制、目标识别 |
| `edgeai_system_manager` | 已实现 | `system_manager`、阶段管理、冻结、日志、Manifest | 感知/导航算法 |
| `edgeai_target_perception` | 已实现 | `target_detector`、相机标定、观测发布 | 全局任务状态 |
| `edgeai_target_fusion` | 已并入`edgeai_target_perception`（`target_fusion_node.py`） | `target_fusion`、目标注册表、救援区生成、Task 1目标导出 | 机器人分配 |
| `edgeai_mission_interpreter` | 规划中（Task 2B） | `mission_interpreter`、本地模型适配、候选计划生成 | DAG调度、Nav2和机器人选择 |
| `edgeai_rescue_coordinator` | 规划中（Task 2B）；中期Part 2由`edgeai_task_coordinator`实现 | `mission_validator`、`robot_state_registry`、`task_allocator`、`rescue_verifier` | LLM推理、底层速度控制 |
| `edgeai_task_coordinator` | 已实现 | 中期Part 2：`task_allocator`、`robot_state_registry`、`rescue_verifier`、碰撞检测插件、中期场地 | LLM推理、底层速度控制 |
| `edgeai_system_tests` | 已实现 | Mock节点、场景、端到端测试和回放 | 正式比赛算法 |

`/robot_N/localization`提供`relocalize`服务；`/edge/system_manager`提供`/competition/set_phase`和`/competition/request_reset`；`/edge/mission_validator`提供`/edge/validate_mission`并且是`/edge/mission_plan`的唯一发布方。各权威数据的唯一生产节点见`config/interface_runtime.yaml`的`authority`。

## 3. 公开节点名

机器人节点由Launch命名空间包裹：

```text
/robot_N/robot_agent
/robot_N/target_detector
/robot_N/localization
/robot_N/navigator
/robot_N/safety_watchdog
```

边缘节点使用固定名称：

```text
/edge/map_manager
/edge/exploration_coordinator
/edge/target_fusion
/edge/robot_state_registry
/edge/task_allocator
/edge/rescue_verifier
/edge/mission_interpreter
/edge/mission_validator
/edge/system_manager
/edge/event_logger
```

节点内部不得重复写`robot_N`。机器人ID和命名空间全部由Launch参数传入。

## 4. 配置目录

```text
config/
  competition_parameters.yaml   # 官方值和开发值
  interface_runtime.yaml        # QoS、频率、超时、身份规则
  sim/                           # 仿真驱动和传感器参数
  hardware/                      # 实机驱动和标定参数
  nav2/                          # 导航与定位参数
  perception/                    # ArUco和融合参数
```

实机序列号、IP和设备路径不能提交到通用默认配置；应使用每台机器人独立配置和受控的现场部署文件。

## 5. 包规则

- 不允许业务包复制共享消息定义或维护私有同义字段。
- Mock节点必须使用真实接口包，不得另建简化消息类型。
