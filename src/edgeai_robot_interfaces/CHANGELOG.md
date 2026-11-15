# edgeai_robot_interfaces变更记录

## 0.2.0（2026-10-22）

破坏兼容性的修改。当前没有已实现的消费包，无需数据迁移；所有持久化格式同步升级`schema_version`。

### 新增

- `ValidateMission.srv`、`MissionValidationIssue.msg`：任务解释器到任务验证器的候选计划通道和机器可读拒绝原因。
- `MissionIntakeResult.msg`（`/edge/mission_intake`）：每条自然语言指令的原始输入、模型/Prompt版本、模型原始输出、候选计划和最终结论。
- `MissionStepState.msg`、`MissionExecutionState.msg`（`/edge/mission_state`）：Task 2B步骤执行状态。
- `RequestReset.srv`（`/competition/request_reset`）：两步式重置流程。
- `Relocalize.srv`（`/<robot_id>/relocalize`）：全局定位、已知起点和位姿提示三种重新定位模式。
- `SystemFault`：`CODE_RELOCALIZATION_FAILED`、`CODE_CONFIGURATION_INVALID`、`CODE_OFFICIAL_PARAMETER_MISSING`。
- `TaskReason`：`RESET_REQUESTED`、`RELEASED_BY_COORDINATOR`、`HOLD_TIMEOUT`、`RELOCALIZATION_FAILED`、`CONFIGURATION_INVALID`。

### 修改

- `RescueTask`：增加`mission_id`、`step_id`、`scoring_stage`和`zone_tolerance_m`；删除`llm_mission_action`（由`scoring_stage == STAGE_TASK2B`代替）。
- `RescueEvent`：增加`mission_id`、`step_id`、`scoring_stage`、`rescue_zone_indices`、`robot_poses`和`EVENT_ZONE_EXITED`。
- `ExecuteRescueTask`：语义改为“到达并保持”。删除`required_dwell`、结果中的`success`/`TERMINAL_COMPLETED`以及反馈中的`inside_zone`/`dwell_elapsed`；增加`arrival_xy_tolerance_m`、`hold_timeout`、`STATE_HOLDING`和`TERMINAL_HOLD_TIMEOUT`。停留与完成只由边缘救援验证器判定。
- `RobotState`：`MODE_WAITING_FOR_PARTNER`/`MODE_DWELLING`合并为`MODE_HOLDING=4`，取值5保留。
- `MissionPlan`：增加`prompt_version`；删除`validation_error`（改由`MissionIntakeResult.issues`记录）。
- `MissionStep`：明确`priority`数值越大优先级越高。
- `CompetitionState`：增加`run_mode`、`parameters_sha256`、`interrupted_phase`、`PHASE_NONE`和重置状态字段。
- `SetCompetitionPhase`：增加`requester_type`和`requester_node`，允许自主模块按跳转表请求阶段结束。
- `SubmitMission`：明确响应只表示已接收，结果异步发布。

### 持久化格式

- `target_list`升级到`schema_version: 2`：完整地图元数据、`rescue_geometry`、`marker_yaw_rad`，允许空救援区和空目标列表。
- `mission_plan`升级到`schema_version: 2`：`model.prompt_version`、`validated_at`，`target_revision`允许为0。
- 新增`mission_intake`、`event_record`、`manifest`、`component_versions` Schema（均为v1）。

## 0.1.0

初始接口集合。
