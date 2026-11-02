# edgeai_exploration

单机器人 SLAM 地图管理与 Frontier 自主探索。首版只允许 robot_1 建图；其他机器人
可在线确认阶段、准备 Task 2，不向权威地图写入扫描。开发环境为 Ubuntu 22.04 + Pixi。

## 文件与职责

- `frontier.py`：不依赖 ROS 的 Grid、FrontierPlanner、ExplorationProgress。支持旋转地图
  原点、未知/障碍膨胀、四连通可达性、前沿聚类、候选评分、重试黑名单及结束判断。
- `ros_grid.py`：OccupancyGrid 与 Grid 转换。长度单位米，角度弧度，四元数 xyzw；
  原点变换是从栅格局部坐标到 map 的主动平面变换。
- `map_manager.py`：唯一 `/map` 发布方，拥有 map_id、revision、SLAM 子进程和冻结快照。
  SLAM 输出重映射到 `/edge/slam_map`；SLAM 仍独占发布建图期间的 map→robot_1/odom。
- `coordinator.py`：读取地图/机器人状态，通过 P1 的 ExecuteNavigationGoal 发送目标，
  处理结果、取消和阶段变化；不发布任何速度。
- `launch/slam.launch.py`：SLAM Toolbox 配置/激活及地图话题重映射。
- `config/slam.yaml`：P2 拥有的 SLAM 参数；1 秒地图刷新、0.1 米/弧度更新门限。
- `test/test_frontier.py`：坐标、障碍与未知空间、断开区域、重试及结束条件。
- `test/test_exploration_quality.py`：代码风格与版权。
- `setup.py`、`setup.cfg`、`package.xml`、`resource/`：ament_python 安装配置。

## 启动

在工作空间根目录：

```bash
pixi run build
pixi shell
source install/setup.bash
ros2 launch edgeai_system_manager task1.launch.py run_id:=new
```

该入口在 PREPARATION 启动传感器、SLAM 和 Nav2。操作员通过 `/competition/set_phase`
进入 TASK1 后，探索自动执行。不要同时启动原始 start_sim.sh 或另一套 SLAM。
已有运行重启不会自动继续运动；未冻结 SLAM 会话不自动重建，需明确创建新运行。

## 接口

- 输入：`/map`、`/competition/state`、`/robot_1/state`。
- 输出：`/edge/exploration_status`（共享接口）、`/map`（OccupancyGrid）。
- 调用：`/robot_1/execute_navigation_goal`、`/competition/set_phase`。
- P2 内部：`/edge/map_metadata`（String JSON，run_id/map_id/revision/frozen/has_map/
  slam_running）；`/edge/map_manager/freeze`（Trigger）。这些端点不替代团队公共消息。
- 冻结仅允许 HANDOVER。先停止 SLAM 及其 TF，再持久化最终地图、递增版本并发布冻结图。
  system_manager 负责取得机器人安全确认，并与目标快照一起完成 FinalizeTask1。

## 参数

`config_dir`、`run_mode`、`use_sim_time` 由统一入口传入。探索参数：

- `robot_id=robot_1`：首版不能改成第二台建图机器人。
- `clearance_m=0.45`：候选中心周围必须为已知自由空间的距离。
- `goal_margin_m=0.15`：目标额外留出到达误差与栅格余量；默认目标净空 0.60 m。
- `minimum_frontier_cells=3`、`approach_m=0.8`：前沿过滤与目标投影距离。
- `no_frontiers_s=15`：持续无可达前沿才结束。
- `goal_timeout_s=90`：单次动作时限；P1 仍负责局部规划与恢复。
- `visited_radius_m=0.25`：已到达目标邻域避免反复下发。
- `reference_area_m2=0`：未知真实区域面积时 coverage_fraction 为 NaN，不伪造覆盖率。
- 总时限、无地图增长时限和失败黑名单半径来自统一 resolved competition 参数。

map_manager 参数：`run_directory` 必须含初始化身份文件；`slam_params` 指向配置；
`start_slam=false` 仅供注入地图测试。运行中 map_revision 不随每次 SLAM 栅格更新递增；
初次持久化为 1，最终冻结快照递增。运行目录有独占锁，防止重复地图管理器。

## 验证与限制

```bash
pixi run bash -c 'source install/setup.bash && python -m pytest src/edgeai_exploration/test -q'
```

真实仿真验收由
`ros2 run edgeai_system_manager validate_sim` 执行，记录实际失败与成功，不能用 Mock
测试替代真机验收。

Frontier 是保守基线，不是全局最优探索；四连通规划可能拒绝狭窄区域。Nav2 仍会按完整
机器人轮廓验证路径。SLAM 重影、真实打滑与回环目标修正需要实际数据评估；地图覆盖率
不能仅凭矩形栅格比例判定。首版未实现双机地图融合。正式起点坐标系要求待规则确定。


P1 的 StopZone 最远角距 base_footprint 约 0.429 m。探索路线采用 0.45 m
净空，目标再加 0.15 m（含 0.10 m 到达误差及栅格余量），避免目标虽可到达，
但机器人转向时后角进入安全停车区。实机更换足迹时必须重新核对这些数值。
