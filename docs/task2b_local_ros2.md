# 卡 04：本地 ROS 2 规则解释器接入

基于已核实的 04a `a98fd8c`。使用旧目录已有 Pixi 环境，只借 Python/ROS 运行时；接口、配置、规则书和 Mock 全部来自当前仓库。无需下载模型或安装依赖。

## 构建

```bash
cd ~/edgeai_ws_task2b
source tools/task2b/env.sh
colcon build --packages-select edgeai_common edgeai_robot_interfaces edgeai_system_tests edgeai_mission_interpreter edgeai_rescue_coordinator --executor sequential
source install/setup.bash
```

## 一次命令跑两次验收（推荐）

```bash
cd ~/edgeai_ws_task2b
unshare --user --map-root-user --net bash --noprofile --norc -c 'ip link set lo up; tools/task2b/launch_mock.sh --test'
```

该命令创建只有 loopback 的网络环境，在其中启动两个仓库 MockRobotAgent、解释器、验证器和明确标识的测试替身。观察器先订阅，再用真实 `ros2 service call` 提交规则书 6.8 原句和 `Rescue T999.`。按 mission_id 校验依赖、终态、发布者和拒绝原因；非法提交后观察 6 秒，不能把有限观察说成永远没有消息。

完整消息存入 `log/task2b/observations.json`，节点日志在同目录。每次运行覆盖这些调试日志，保存证据请先复制。运行结束自动关闭本次节点。

## 手动启动、提交和查看

在一个终端进入断网测试 shell，后面的命令全部在此 shell 内执行：

```bash
cd ~/edgeai_ws_task2b
unshare --user --map-root-user --net bash --noprofile --norc
ip link set lo up
source tools/task2b/env.sh
tools/task2b/launch_mock.sh &
stack_pid=$!
mkdir -p log/task2b/manual
ros2 topic echo /edge/mission_intake > log/task2b/manual/intake.yaml &
intake_pid=$!
ros2 topic echo /edge/mission_plan --qos-durability transient_local > log/task2b/manual/plan.yaml &
plan_pid=$!
ros2 topic echo /edge/mission_state --qos-durability transient_local > log/task2b/manual/state.yaml &
state_pid=$!
```

等待服务和订阅者发现后提交（intake 是 volatile，必须先启动观察器）：

```bash
ros2 service call /edge/submit_mission edgeai_robot_interfaces/srv/SubmitMission '{run_id: "run-20261022T120000Z-1a2b3c4d", original_instruction: "First rescue T2. After T2 is completed, rescue T1 and T3. Finally, after both T1 and T3 are completed, use two robots to cooperatively rescue TC1."}'
```

记下返回 ID，等对应 state=2（COMPLETED），再提交非法例句：

```bash
ros2 service call /edge/submit_mission edgeai_robot_interfaces/srv/SubmitMission '{run_id: "run-20261022T120000Z-1a2b3c4d", original_instruction: "Rescue T999."}'
ros2 topic info /edge/mission_state --verbose
cat log/task2b/manual/intake.yaml
cat log/task2b/manual/plan.yaml
cat log/task2b/manual/state.yaml
```

按两个 mission_id 分开检查。第二次 accepted=true 仅代表收到；intake outcome=2、code=102 才是验证拒绝。plan/state 的 transient-local 旧快照可能仍属于第一次任务。收工：

```bash
kill "$intake_pid" "$plan_pid" "$state_pid"
kill "$stack_pid"
wait "$stack_pid"
exit
```

## 实现和边界

- `ros_node.py`：规则 `rescue` 映射 `MissionStep.ACTION_RESCUE`；每次提交新建 UUID；单线程 timer 驱动解释与异步验证；只发布 intake，不发布 plan 或 state。
- `validator_node.py`：加载显式 frozen_targets 文件，订阅比赛状态和机器人聚合状态，检查上下文新鲜度，复用 `validate_mission`；候选 steps 不改动，成功时只把 valid 改成 true 并先 publish 后 return。
- notes 与 steps/issues 一起序列化到 raw_model_output。rule-baseline 没有模型调用；不把 notes 冒充验证错误，不新增共享字段。
- 验证服务超时使用 CODE_UNKNOWN=0 并说明远端结果未知；现有接口没有取消验证事务的能力。此时不能推断远端一定没有发布计划。超时/迟到响应端到端测试未运行。
- `task2b_mock_allocator_TEST_ONLY` 是测试替身，兼任测试比赛状态与机器人状态聚合。只支持一个活动任务；重放同 ID 不重复调度。依赖库仅在单线程 executor 内调用，start 先于 complete，task_id 和 robot_ids 保持一致。步骤超时 60 秒注入失败。
- 替身调用仓库原始 ExecuteRescueTask 动作，普通目标顺序执行；T1、T3 同时 READY，但本次没有并行执行。TC1 同时派两台机器人。Mock 只报告到达/保持；替身在参与者都 HOLDING 时注入**测试验证完成事件**并取消动作释放机器人，不把动作取消当成救援成功。
- 这不是正式 rescue_verifier 的区占用/停留判定，也没有碰撞规划。测试速度为 2 m/s 和 3 rad/s，仅用于运动学 Mock。真机、Gazebo、Ye 节点联调均未运行。
- 当前起点的 `edgeai_task_coordinator` 未接入 MissionPlan/DependencyScheduler/MissionExecutionState。正式 mission_state 权威仍是 `/edge/task_allocator`；不要与此测试替身同时启动。
- 共享 msg/srv/action/schema/config 和 Ye 包零改动。问题码 103 的字段限制仍待团队澄清。
