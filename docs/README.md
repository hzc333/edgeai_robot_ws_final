# 设计文档

安装、运行和测试请先看[根目录README](../README.md)。本目录记录系统设计和约定。

## 系统设计

1. [系统架构与接口协议](system_architecture_and_interfaces.md)：命名空间、TF树、ROS 2端点、QoS、状态机、安全协议和持久化文件格式。
2. [ROS包与节点布局](package_and_node_layout.md)：包、公开节点名、依赖方向和配置目录。
3. [Marker坐标约定与救援区几何](marker_and_rescue_geometry.md)：从ArUco检测到`map`坐标系目标位姿和救援区。
4. [比赛规则假设与TBD登记表](rule_assumptions_and_tbd.md)：规则草案中的假设、冲突和尚未确定的官方参数。

## 架构决策（ADR）

- [ADR-0001 首版Task 1策略](decisions/0001-task1-v1-strategy.md)
- [ADR-0002 多机器人TF、命名和机器人参考点](decisions/0002-multi-robot-tf-and-naming.md)
- [ADR-0003 速度安全链与运动门控](decisions/0003-velocity-safety-chain.md)

## 使用说明

- [基础仿真与手动操作](simulation_basics.md)：开发仿真、话题、手动驾驶、手动SLAM、系统ROS。
- 各包的文件、参数、接口和验证记录见`src/<包名>/README.md`。

## 参考资料（组织方）

- [比赛规则草案 V0](reference/Edge_AI_Robotics_Challenge_Rulebook_V0.md)
- [初始机器人仿真与SLAM指南](reference/Initial_Robot_Simulation_and_SLAM_Guide.md)（部分数值已被ADR-0002修正）

## 机器可读文件

- `schemas/target_list.schema.json`：持久化Task 1目标列表格式。
- `schemas/mission_plan.schema.json`：验证通过的Task 2B任务计划格式。
- `schemas/mission_intake.schema.json`：每条Task 2B指令的接收、解析和验证记录。
- `schemas/event_record.schema.json`：`events.jsonl`中每行事件的格式。
- `schemas/manifest.schema.json`、`schemas/component_versions.schema.json`：运行Manifest和组件版本记录。
- `config/competition_parameters.yaml`：官方参数、开发默认值和运行模式解析规则。官方字段为`null`表示组织方尚未确定，代码不得静默猜测。
- `config/interface_runtime.yaml`：统一QoS、频率、超时、坐标系、ID生成规则、权威生产节点和阶段跳转表。
- `examples/interfaces/`：目标列表和任务计划格式的有效样例。
- `src/edgeai_robot_interfaces`：共享ROS 2消息、服务和动作定义。

## 修改规则

修改接口时，数据生产方、消费方和系统集成都要审核。破坏兼容性的修改必须增加`schema_version`并提供迁移说明。比赛规则更新应先记录到TBD登记表，再同步到配置和代码。修改后运行`pixi run validate`检查契约。
