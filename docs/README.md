# 项目前期准备文档

这些文档定义了比赛系统的共享工程协议。除非经过审核的新版本明确替代，否则团队开发应以这些内容为准。

1. [系统架构与接口协议](system_architecture_and_interfaces.md)
2. [五人分工、职责边界与交接协议](team_responsibilities.md)
3. [开发与集成计划](development_and_integration_plan.md)
4. [规则假设与TBD登记表](rule_assumptions_and_tbd.md)
5. [ROS包、节点与所有权规划](package_and_node_layout.md)
6. [首版Task 1策略决策](decisions/0001-task1-v1-strategy.md)（ADR-0001）
   - [多机器人TF、命名和机器人参考点](decisions/0002-multi-robot-tf-and-naming.md)（ADR-0002）
   - [速度安全链与运动门控](decisions/0003-velocity-safety-chain.md)（ADR-0003）
7. [Mock与集成测试计划](mock_and_test_plan.md)
8. [Marker坐标约定与救援区几何](marker_and_rescue_geometry.md)
9. [小组GitHub协作流程](team_collaboration.md)

运行与开发说明：

- [根目录README](../README.md)：安装、中期全流程运行、测试和排查（组员测试从这里开始）。
- [基础仿真与手动操作](simulation_basics.md)：开发仿真、话题、手动驾驶、手动SLAM、系统ROS。
- [P2开发记录](p2_development.md)、[P2实现核对](p2_implementation_review.md)、[P2交接README](../README_P2.md)。

配套的机器可读文件：

- `schemas/target_list.schema.json`：持久化Task 1目标列表格式。
- `schemas/mission_plan.schema.json`：验证通过的Task 2B任务计划格式。
- `schemas/mission_intake.schema.json`：每条Task 2B指令的接收、解析和验证记录。
- `schemas/event_record.schema.json`：`events.jsonl`中每行事件的格式。
- `schemas/manifest.schema.json`、`schemas/component_versions.schema.json`：运行Manifest和组件版本记录。
- `config/competition_parameters.yaml`：官方参数、保守的开发默认值和运行模式解析规则。官方字段为`null`表示组织方尚未确定，代码不得静默猜测。
- `config/team_assignments.yaml`：P1～P5主负责人和备份人员记录。
- `config/interface_runtime.yaml`：统一QoS、频率、超时、坐标系、ID生成规则、权威生产节点和阶段跳转表。
- `examples/interfaces/`：目标列表和任务计划格式的有效样例。
- `src/edgeai_robot_interfaces`：共享ROS 2消息、服务和动作定义。

## 修改规则

修改接口时，必须由数据生产方负责人、消费方负责人以及系统集成负责人共同审核。破坏兼容性的修改必须增加`schema_version`，并提供迁移说明。比赛规则更新应先记录到TBD登记表，再同步到配置和代码。

在仓库根目录执行以下命令，可以检查前期准备协议：

```bash
python3 tools/validate_project_contracts.py
```
