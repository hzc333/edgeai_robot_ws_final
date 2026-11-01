# P2 开发与验收记录

环境：Ubuntu 22.04 + 仓库 pixi.lock（ROS 2 Jazzy / Gazebo Harmonic）。
基线：ac9f321。代码位于 edgeai_exploration、edgeai_system_manager；没有修改 P1/P3/P4
实现或共享接口。ADR-0001：仅 robot_1 建图，robot_2 为 Task 2 准备，不融合地图。

## 已实现的软件能力

- [x] Pixi 构建、环境检查和共享契约校验。
- [x] 运行目录独占、稳定 run_id、解析后参数、版本与源代码归档、事件去重。
- [x] 阶段权限、心跳、匹配确认、急停抢占、累计计时、官方参数门禁。
- [x] SLAM 生命周期、地图版本、持久化和重新加载、map/TF 启动门控。
- [x] Frontier 安全候选、距离/信息增益排序、失败黑名单、取消与自动结束。
- [x] FinalizeTask1 版本一致性、原子提交、哈希、崩溃恢复和幂等。
- [x] HANDOVER 停稳、停止 SLAM、冻结地图、AMCL relocalize 编排。
- [x] 救援事件、任务输入/输出、原文、模型版本、MCAP、审计和导出。
- [x] 日志器/SLAM 故障门控、独立最终状态快照、冻结地图晚加入订阅者。
- [x] 每包 README、开发用空目标替身、自动验收驱动。

这些勾选表示代码和相关自动化检查已实现，不等同正式比赛/实机验收通过。

## 可复现检查

在仓库根目录执行：

```bash
pixi run build
pixi run validate
pixi run check-env
pixi run test
pixi run bash -c 'source install/setup.bash && ros2 run edgeai_system_manager validate_sim --timeout 660'
```

全仓原有 181 项测试通过；P2 另有算法、ROS 服务、地图重载、任务证据和质量检查。
具体最新计数以 colcon test-result 为准。validate 通过，check-env 为 0 错误、0 警告。
各测试使用独立 DDS 域，验收默认域 74。不要与其他域 74 演练并行运行。
每次仿真使用 GZ_PARTITION=p2-<run_id>，验收退出时清理自己的进程组。

## 仿真证据与纠正记录

- run-20261031T164655Z-f96c74c7：自主探索结束，验收驱动过早请求下一阶段而失败；
  已增加 busy 响应的有界重试。
- run-20261031T215328Z-93212b44：闭环和哈希审计通过，但后来发现旧 Gazebo
  服务器残留，因此不能作为全新世界启动的有效验收证据。
- run-20261031T215916Z-35a847c4：发现同一隔离问题后主动中断，不计为通过。
- run-20261031T220407Z-9cd6a7a6：独立分区，探索/冻结/AMCL 通过；返航被
  Nav2 拒绝（error 104）。核对 P1 足迹后发现原 0.35 m 净空小于后角 StopZone
  约 0.429 m，已改为路线 0.45 m、目标 0.60 m，并新增目标余量测试。
- run-20261031T220820Z-c1f66909：独立 Gazebo 分区完整通过自主探索、冻结、AMCL、
  返航、FINISHED 与审计（16 个文件、8 条事件）。最终控制事件记录结束原因
  no_map_growth；验收 JSON 的旧采样逻辑误填 navigating，已在驱动中修正。
  已知面积 24.745 m²，自由面积 23.495 m²；无权威总面积，覆盖率保持 null。
  运行包已导出至 runs/p2-validation/exports/，解包后再次审计通过。
  导出 SHA-256：86d9566ae15730990534636aec0e5fb2a73914b8252009d24d41a2f592bdbe7a。

当前测试汇总：211 tests，0 errors，0 failures，0 skipped。

报告和 launch 日志：runs/p2-validation/。每个运行保存参数、源代码 tar.gz、
源文件哈希、组件版本、地图、targets、Manifest、JSONL、MCAP。源码变更后应显式
run_id:=new，禁止在旧运行中悄悄更换代码。终态的 final_state.json 与持续更新的
控制 state.json 分开，赛后急停不会改变已封存证据。

## 尚需完成的联合验收及限制

- [ ] 真实 P3 冻结目标、P4 救援和自然语言任务的联合比赛演练。
- [ ] 正式场地/传感器的地图质量、重摆定位、TF 与真实运动验收。
- [ ] 根据官方规则确认起点坐标系、场地总面积、时限与重置策略。
- [ ] 多次独立场景和故障注入的 Gazebo 回归，记录成功率而非只报一次通过。

P3 空目标替身必须显式开启且仅限 development，不代表感知成功。
没有权威场地面积时只保存已知/自由/未知面积，coverage_fraction 不伪造百分比。
Frontier 为保守单机基线；未实现双机地图融合。

重置实现 BEGIN/COMPLETE/ABORT、全局停稳、计数、计时和重定位；P1 尚未按
resetting_robot_ids 单机门控，因此保守暂停所有机器人。官方规则缺失时 competition
拒绝重置；若正式规则要求清除地图/目标/已完成救援，当前拒绝执行，需要团队约定
跨模块清理事务，不能仅改 P2 本地文件冒充完成。未冻结 SLAM 崩溃不自动拼接旧图，
需要明确新运行；冻结地图可恢复，恢复运动仍需操作员请求。
