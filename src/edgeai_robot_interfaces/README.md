# edgeai_robot_interfaces

该包是比赛系统共享ROS 2接口的唯一权威来源，不包含业务算法。

## 接口分类

- 感知与目标：`TargetObservation`、`Target`、`TargetArray`。
- 机器人状态与定位：`RobotState`、`RobotStateArray`、`RobotPhaseAck`、`Relocalize`服务。
- 比赛与探索：`CompetitionState`、`EmergencyStop`、`ExplorationStatus`、`SetCompetitionPhase`和`RequestReset`服务。
- 救援与任务：`RescueTask`、`RescueEvent`、`MissionStep`、`MissionPlan`。
- Task 2B指令处理：`MissionValidationIssue`、`MissionIntakeResult`、`MissionStepState`、`MissionExecutionState`，`SubmitMission`和`ValidateMission`服务。
- 统一原因：`SystemFault`、`TaskReason`。
- 动作：`ExecuteNavigationGoal`、`ExecuteRescueTask`。
- 服务：`FinalizeTask1`、`SetCompetitionPhase`、`RequestReset`、`SubmitMission`、`ValidateMission`、`Relocalize`。

当前版本0.2.0，变更记录见`CHANGELOG.md`。

## 修改规则

- 不得在业务包中复制这些定义。
- 破坏兼容性的修改需要所有生产方、消费方和P5审核。
- 修改后必须重新构建全部消费包，并更新架构文档和样例。
- 自由文本只供人阅读；程序判断必须使用枚举、状态码或版本字段。

## 构建

项目锁定环境中的构建命令：

```bash
pixi run colcon build --packages-select edgeai_robot_interfaces
```

如果使用系统安装的ROS 2 Jazzy，也可以执行：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select edgeai_robot_interfaces --symlink-install
source install/setup.bash
ros2 interface list | grep edgeai_robot_interfaces
```

构建后`ros2 interface list`应列出全部消息、服务和动作，生成的Python类型可以正常导入。
