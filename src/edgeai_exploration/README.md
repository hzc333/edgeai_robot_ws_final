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
  `center_karto_cells` 修正 SLAM 地图半格的整体偏移，`complete_max_edges` 补回 SLAM
  可能丢掉的地图右边一列和上边一行（见“参数”）。
- `coordinator.py`：读取地图/机器人状态，通过机器人代理的 ExecuteNavigationGoal 发送目标，
  处理结果、取消和阶段变化；不发布任何速度。
- `slam_runner.py`：机器人和边缘节点分开部署时，在 robot_1 上启停 SLAM（服务 `~/start`、`~/stop`，状态心跳 `~/status`）。`map_manager` 设 `slam_runner:=/robot_1/slam_runner` 时不再自己启动 SLAM，冻结前等它确认 SLAM 已结束（ADR-0004）。
- `launch/slam.launch.py`：SLAM Toolbox 配置/激活及地图话题重映射。
- `config/slam.yaml`：SLAM 参数；1 秒地图刷新、0.1 米/弧度更新门限。
- `test/test_frontier.py`：坐标、障碍与未知空间、断开区域、重试及结束条件。
- `test/test_map_manager.py`：清除停放机器人、半格原点修正、补回被丢掉的地图边缘、冻结图只发布一次、
  快照写入失败不确认冻结、重启后重新发布冻结图。
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
- 内部：`/edge/map_metadata`（String JSON，run_id/map_id/revision/frozen/has_map/
  slam_running）；`/edge/map_manager/freeze`（Trigger）。这些端点不替代团队公共消息。
- 冻结仅允许 HANDOVER。先停止 SLAM 及其 TF，再持久化最终地图、递增版本并发布冻结图。
  system_manager 负责取得机器人安全确认，并与目标快照一起完成 FinalizeTask1。

## 参数

`config_dir`、`run_mode`、`use_sim_time` 由统一入口传入。探索参数：

- `robot_id=robot_1`：首版不能改成第二台建图机器人。
- `clearance_m=0.30`：通往目标的路径离障碍格（已占据）的最小距离。只判断能否到达，
  实际路径由 Nav2 按完整轮廓规划；0.30 m 让 0.8 m 宽的门也算可通过（原来的 0.45 m
  要求两侧共 0.9 m，中期场地 O2 和 W2 之间 0.85 m 的通道被判为不通）。
- `unknown_clearance_m=0.15`：路径离未知格的最小距离。第一帧扫描在激光束之间和其他
  机器人的遮挡里留有未知格，若像墙一样要求，起点附近找不到任何可达前沿。靠近后激光会
  揭示未知格里的东西。
- `goal_margin_m=0.30`：目标在路径净空之外再留的余量，即目标离障碍 0.60 m、离未知
  0.45 m。机器人在目标处可能原地转向，碰撞监视器 StopZone 的最远角距 base_footprint
  约 0.429 m，再加到达误差和栅格余量。
- `fallback_goal_margin_m=0.20`：按上面的余量找不到任何前沿目标时，改用这个余量再找一次
  （目标离障碍 0.50 m，仍大于 0.429 m）。SLAM Toolbox 只在机器人移动 0.1 m 后才加入新
  扫描（原地转向不算），且一个格要被两条激光穿过才标为自由，所以起点处的第一张地图只是
  一帧稀疏扫描；起始区夹在墙和停着的 robot_2 之间，按 0.60 m 找不到目标。先走一小步，
  地图就会补全。
- `escape_m=0.30`：机器人当前位置周围这一范围内的自由格不要求路径净空，可以从这里出发。
  机器人本来就站在那里，可能离墙比 `clearance_m` 近。
- `minimum_frontier_cells=3`、`approach_m=0.8`：前沿过滤与目标投影距离。
- `no_frontiers_s=15`：持续无可达前沿才结束。
- `goal_timeout_s=90`：单次动作时限；机器人代理和 Nav2 负责局部规划与恢复。
- `visited_radius_m=0.25`：已到达目标邻域避免反复下发。
- `reference_area_m2=0`：未知真实区域面积时 coverage_fraction 为 NaN，不伪造覆盖率。
- 总时限、无地图增长时限和失败黑名单半径来自统一 resolved competition 参数。
- 摄像头覆盖（`camera_coverage=true`）：前沿探索只保证激光地图完整，不保证摄像头看过每面墙。
  Task 1 期间以 5 Hz 记录摄像头看过的表面格：在 `coverage_fov_deg=50`（相机实际 60°，留余量）
  和 `coverage_range_m=2.0` 内被射线打到、且与墙面法线夹角不超过 60°（标记要大致正对才能读出）。
  前沿探索结束后（时间用完时除外），在可达的目标格上按 0.3 m 间隔找观察位姿，贪心地选能看到最多
  未看过表面的位置和朝向，直到新增少于 `coverage_min_cells=3` 格或达到 `coverage_max_views=10`
  个，按就近顺序逐个前往并停留 `viewpoint_dwell_s`，然后再回起点。Task 1 剩余时间少于
  `coverage_reserve_s=90` 时跳过剩下的观察点，留时间回起点。实现见 `coverage.py`。
- 探索结束后：`viewpoints_file`（默认为空，不启用）可给出未检测标记的补看点和路线，
  `viewpoint_dwell_s=2` 是每个补看点的停留时间；`return_pose_id` 非空时最后开回该起点，
  之后才请求 HANDOVER。Part 2 要求机器人从起始区出发，所以回起点失败（例如途中短暂
  定位丢失）时会等定位恢复后重发，最多 `return_retries=3` 次；主动取消（阶段切换、急停）
  不重试。

map_manager 参数：`run_directory` 必须含初始化身份文件；`slam_params` 指向配置；
`start_slam=false` 仅供注入地图测试。运行中 map_revision 不随每次 SLAM 栅格更新递增；
初次持久化为 1，最终冻结快照递增。运行目录有独占锁，防止重复地图管理器。

SLAM 地图修正（`task1.launch.py` 打开，map_manager 默认关闭）：

- 半格原点修正（`center_slam_cells`）：Karto 把点舍入到最近格，格 k 的中心在
  原点 + k×分辨率；SLAM Toolbox 却把这个原点当作 ROS 地图第 0 格的角发布，地图内容因此
  整体向 +x/+y 偏半格（0.05 m 分辨率时 2.5 cm），墙边的空闲格会被标成障碍。修正把原点
  沿地图自身坐标轴移回半格。
- 边缘补全（`edge_scan_topic`、`edge_min_hits=3`）：Karto 的栅格宽高为
  round(端点范围 / 分辨率)，最右一列（+x）或最上一行（+y）的墙点可能被舍入到栅格外而
  丢掉。`edge_scan_topic` 非空时，map_manager
  在冻结前把建图机器人每帧扫描的端点经 TF 变换到 map、按 1 cm 网格计数；某一侧栅格外
  一列/一行的端点多于栅格最后一列/一行时（墙在外面），把栅格扩一格，命中不少于
  `edge_min_hits` 次的新格标为障碍，其余为未知。只用实测端点，不凭空补墙；地图完整时
  不改变（实时 TF 下少量端点越过已画出的墙，不触发扩展）。
- 回环搜索范围（`config/slam.yaml`，`edgeai_robot_description/config/slam_toolbox.yaml`
  保持一致）：`loop_search_maximum_distance=1.0`、`loop_search_space_dimension=2.0`，
  只在当前位姿附近 1 m（2 m 窗口）找回环。默认的 3 m / 8 m 窗口覆盖整个 5 m 场地，
  房间长得相似时会匹配到错误的位置：一次更新里 robot_1 位姿跳 1～3 m，地图上的墙出现
  重影。这类场地里里程计漂移远小于 1 m。不要改回默认值。
- 两项地图修正都作用于实时 `/map` 和冻结图。离线验证方法：用运行包里的 rosbag 把 Task 1 的
  激光、里程计和 TF 重放给同参数的 SLAM Toolbox，再与场地真实几何逐格比较。

## 录包和离线建图

调 SLAM 参数（尤其是换成实机激光雷达后）可以对同一份录包反复建图比较。录一台机器人
的输入，录制时只运行驱动、里程计 TF 和静态外参，不运行 SLAM 或 AMCL，免得旧的
`map -> odom` 混进回放输入：

```bash
ros2 bag record --storage mcap -o runs/robot1-slam-input \
  /robot_1/scan /robot_1/odom /tf /tf_static
ros2 bag info runs/robot1-slam-input      # 核对四个话题和消息数
```

在单独的 ROS 域里回放，两个终端都激活环境并 `source install/setup.bash`：

```bash
# 终端 1
export ROS_DOMAIN_ID=81 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 launch edgeai_exploration slam.launch.py \
  slam_params_file:="$PWD/src/edgeai_exploration/config/slam.yaml" \
  use_sim_time:=true odom_frame:=robot_1/odom
# 终端 2
export ROS_DOMAIN_ID=81 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 bag play runs/robot1-slam-input --clock
```

RViz 的固定帧用 `map`，地图话题 `/edge/slam_map`，同样用仿真时间。看重影、墙是否对齐、
闭环前后的位置和已知尺寸的误差；改参数时保留原录包和参数副本，用同一份录包对比。
比赛运行的录包（`event_logger` 录的 MCAP）里已经有 SLAM 发布的 `map -> odom`，要先
过滤掉这条 TF 才能作为建图输入。

## 验证与限制

```bash
pixi run bash -c 'source install/setup.bash && python -m pytest src/edgeai_exploration/test -q'
```

真实仿真验收由 `ros2 run edgeai_system_manager validate_sim` 执行；Mock 测试不能替代
真机验收。

改探索、覆盖或 SLAM 参数后，用随机练习场地衡量效果（根目录 README 5.5 节）：
`pixi run practice --arena runs/practice-arenas/seed-N` 跑 Task 1 并打分，要在多个种子
上比较目标是否找到、Task 1 用时和地图占据准确率，不能只看一个场地。只在中期场地上
通过不说明探索可用：最终场地事先不公开，也不允许按已知标记位置安排补看点。

Frontier 是保守基线，不是全局最优探索；四连通规划可能拒绝狭窄区域。Nav2 仍会按完整
机器人轮廓验证路径。SLAM 重影、真实打滑与回环目标修正需要实际数据评估；地图覆盖率
不能仅凭矩形栅格比例判定。首版未实现双机地图融合。正式起点坐标系要求待规则确定。


碰撞监视器 StopZone 的最远角距 base_footprint 约 0.429 m。探索目标离障碍 0.60 m
（含 0.10 m 到达误差及栅格余量），避免目标虽可到达，但机器人转向时后角进入安全停车区；
路径只要求 0.30 m，转向发生在目标处而不是途中。前沿格按八连通分组：斜着的前沿（例如
穿过门口的一束激光）只有角点相接，四连通分组会把它拆成单格，全部低于
`minimum_frontier_cells`。实机更换足迹时必须重新核对这些数值。
