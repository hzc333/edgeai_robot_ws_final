# 实机接入清单

本仓库目前只在 Gazebo 里运行过。这份清单列出仿真里默认成立、换到实机必须测量、标定或决定的事项，以及第一次上机的顺序。实机驱动、话题和标定方法以组织方的 Technical Tutorial 为准；它公布后先对照第 1 节，再逐项处理其余各节。

硬件（规则书 2.1）：差速底盘，Phidgets 4A DC 电机控制器和 VINT Hub，Pololu 50:1 电机带 64 CPR 编码器（输出轴每转 3200 计数），BaneBots 3 7/8 英寸轮，Intel RealSense D435i，SLAMTEC RPLIDAR（A1），Phidgets IMU，Intel NUC11 加 GT 1030。

## 1. 驱动必须提供的接口

机器人上的软件栈（`robot_stack.launch.py`：安全看门狗、`robot_agent`、Nav2、定位、目标检测）只依赖下面这些话题和坐标系。驱动给出的名字不同时，用重映射对齐，不改软件栈。

| 话题 | 类型 | 坐标系 | 仿真中的值 | 要求 |
|---|---|---|---|---|
| `/robot_N/scan` | `sensor_msgs/LaserScan` | `robot_N/lidar_link` | 10 Hz，360 点，0.15–12 m | 至少 5 Hz，覆盖 360° |
| `/robot_N/odom` | `nav_msgs/Odometry` | `robot_N/odom` → `robot_N/base_footprint` | 50 Hz | 至少 20 Hz；同时发布同名 TF |
| `/robot_N/camera/image` | `sensor_msgs/Image` | `robot_N/camera_optical_frame` | 30 Hz，640×480 RGB | 至少 10 Hz；光学坐标系（x 右、y 下、z 前） |
| `/robot_N/camera/camera_info` | `sensor_msgs/CameraInfo` | 同上 | 与图像同步 | 内参非零，尺寸与图像一致 |
| `/robot_N/cmd_vel` | `geometry_msgs/Twist` | — | 驱动订阅 | 只有 `safety_watchdog` 发布（ADR-0003） |

- **TF**：`robot_N/base_footprint` 是两个驱动轮轴中点在地面上的投影（ADR-0002）。`robot_state_publisher` 按 URDF 发布 `base_footprint` 到 `lidar_link`、`camera_optical_frame` 的静态变换，帧名都带 `robot_N/` 前缀。
- **驱动自己要有命令超时**：看门狗门控时以 20 Hz 发零速度；但看门狗进程本身挂掉时没有人再发零速度，驱动必须在一段时间（建议 ≤0.5 s）收不到 `cmd_vel` 时自行停车。Gazebo 会一直执行最后一条命令，仿真测不到这一点。
- **深度图和点云**：目前软件栈不用（`use_depth: false`）。IMU 话题也没有节点使用。
- 检查方法：`tools/robot_preflight.py --robot robot_N`（第 8 节）。

## 2. 要测量和标定的硬件参数

| 项目 | 现在的值（文件） | 怎么做 |
|---|---|---|
| 轮半径、轮距 | 0.049 m、0.300 m（`edgeai_robot_real.urdf.xacro`） | 直行 2 m 量实际距离修正轮半径；原地转 10 圈量角度修正轮距。编码器按输出轴每转 3200 计数换算，最终用整车标定值（规则书 2.1） |
| 车体轮廓 | 轴前 0.115 m、轴后 0.315 m、半宽 0.175 m | 实测外形（含线缆、凸出件）。轮廓在多个文件里各有一份，要一起改，见下表 |
| LiDAR 安装 | `lidar_link` (0.0125, 0.080, 0.155)，朝前 | 量位置；放在长直墙前看扫描是否和墙平行，校正朝向。检查车体是否挡住部分扫描角度（需要时设角度范围或用激光滤波） |
| 相机安装 | `camera_link` (0.0875, 0, 0.100)，上仰 15° | 量位置和俯仰角；最终比赛的标记高度公布后复核俯仰角（`edgeai_robot_description/README.md`） |
| 相机内参 | 仿真的理想针孔 | 用 RealSense 出厂内参（`camera_info`），检测前确认分辨率和 `camera_info` 一致；必要时自己标定 |
| 电机极限 | 0.30 m/s、1.00 rad/s，加速度 0.5、1.5（DiffDrive 插件，开发值） | 测实际最大速度和加速度；比赛限速由看门狗按 `motion.*` 执行 |
| 质量、惯量、IMU 位置 | 占位值 | 称重、实测（只影响仿真，软件栈不用） |

车体轮廓的所有副本（改轮廓时全部同步）：

| 文件 | 用途 |
|---|---|
| `src/edgeai_robot_navigation/config/nav2_sim.yaml` | 局部、全局代价地图的 `footprint`，碰撞监视器的 StopZone/SlowZone 多边形 |
| `src/edgeai_robot_navigation/edgeai_robot_navigation/peer_footprints.py` | 其他机器人轮廓的默认值（启动时由代价地图参数覆盖） |
| `src/edgeai_robot_bringup/edgeai_robot_bringup/robot_agent.py` | `FOOTPRINT_REAR_X` 等，救援直行和脱困判断 |
| `src/edgeai_task_coordinator/edgeai_task_coordinator/model.py` | 救援协调的车体尺寸 |
| `src/edgeai_task_coordinator/tools/run_simulation.py`、`tools/localization_check.py` | 演练和检查工具 |

## 3. 按仿真调过、实机要重标的软件参数

仿真里的传感器没有噪声，里程计几乎不打滑，下面这些值都是对着仿真调的：

| 参数 | 仿真值 | 位置 | 说明 |
|---|---|---|---|
| AMCL 激光模型 `sigma_hit`、`z_hit`、`z_rand` | 0.08、0.9、0.1 | `nav2_sim.yaml` | 无噪声 LiDAR 才能这么紧；RPLIDAR A1 有测距噪声和反光材料的异常点，先回到 Nav2 默认（0.2、0.5、0.5）再收紧，用定位误差和 AMCL 自报的标准差判断 |
| AMCL 运动噪声 `alpha1`–`alpha5` | 0.05 … 0.2 | `nav2_sim.yaml` | 按实测里程计误差设定（直行和转圈测试的残差） |
| 重定位扫描匹配门槛 `scan_match_min_ratio` | 0.85 | `nav2_sim.yaml` 的 `localization` | 仿真中正确位姿 0.96–1.00、错误位姿 0.31–0.66。实机先在已知正确位姿上测匹配度分布，再定门槛；太高会拒绝正确位姿，太低会接受错位 |
| 收敛标准差 `convergence_max_xy_std_m`、`_yaw_std_rad` | 0.10、0.10 | `localization.py` 参数 | 实机 AMCL 标准差更大时要放宽 |
| `robot_agent` 的 `tf_stale_s`、`sensor_timeout_s` | 0.5 s、1.0 s | `robot_agent.py` 参数 | 实机 LiDAR 频率更低（A1 约 5.5–10 Hz） |
| 救援直行速度 `rescue_straight_speed_mps` | 0.08 | `robot_agent.py` 参数 | 低速时电机死区可能让车不动 |
| 碰撞监视器 StopZone/SlowZone | 见 `nav2_sim.yaml` | `collision_monitor` | 按实测轮廓和刹车距离重定 |
| SLAM 参数 | `src/edgeai_exploration/config/slam.yaml` | — | 换成 A1 的扫描频率、点数和量程后重新验证建图 |
| 摄像头覆盖模型 | 视场 50°、距离 2.0 m | `edgeai_exploration/coverage.py` | 按实际能稳定识别标记的距离和视场设定 |
| 检测参数 | ArUco 字典、标记边长来自配置 | `config/competition_parameters.yaml` | 官方公布后填入；在比赛照明下测识别距离和误检 |

导航参数文件叫 `nav2_sim.yaml`；实机建议复制成单独的参数文件，用 `nav_robot.launch.py params_file:=...` 选择，仿真的值保持不动。

## 4. 官方参数（`config/competition_parameters.yaml` 的 `official`）

`run_mode:=competition` 只用 `official` 的值；下面这些为空时系统拒绝离开 PREPARATION：

- `aruco.dictionary`、`aruco.marker_size_m`、`aruco.id_mapping`
- `rescue.geometry_version`、`rescue.position_tolerance_m`、`rescue.normal_dwell_s`、`rescue.cooperative_dwell_s`
- `motion.max_linear_speed_mps`、`motion.max_angular_speed_radps`
- `timing.task1_s`、`timing.task2a_s`、`timing.task2b_s`

还要填但不阻止运动的：场地尺寸和起始区坐标系（`arena.*`）、起点（`localization.start_poses`）和 Task 2B 起点（`localization.task2b_start_poses`）、是否允许操作员给位姿提示、标记安装方式和高度、救援区规则、各阶段时长、重置规则、Task 2B 的语言和模型资源限制。规则冲突和待定项见 `rule_assumptions_and_tbd.md`。

## 5. 网络、时钟和边缘节点（ADR-0004）

- 跨机器用 Zenoh：每台机器运行 `rmw_zenohd`，机器人上的路由器连接边缘节点的路由器（TCP 7447）；所有机器设 `RMW_IMPLEMENTATION=rmw_zenoh_cpp`。不要用 CycloneDDS 的默认配置跨机器运行。
- 每台机器人和边缘节点用 chrony 同步到同一个时间源（偏差几毫秒以内）；`tools/robot_preflight.py` 在边缘节点上运行时会检查消息时间戳和本机时钟的差。
- 实机上 `use_sim_time:=false`（各启动文件和节点参数）。
- 启动分两侧：边缘节点 `task1.launch.py side:=edge`，机器人 `side:=robots`，两边用同一个 `run_id`。Task 1 的 SLAM 在 robot_1 上运行（`slam_runner`）。
- 通信中断时，看门狗 1 s 收不到边缘心跳就停车（ADR-0003）；实机上要试一次拔网线。

## 6. 安全

- 物理急停按钮先于一切软件检查可用（规则书 7.4）。
- 第一次运动时把 `motion.max_linear_speed_mps` 和 `max_angular_speed_radps` 设低（例如 0.1 m/s、0.3 rad/s），确认看门狗限速有效再逐步提高。
- 确认 `cmd_vel` 只有看门狗一个发布者；开发时的手动控制走 `cmd_vel_manual`，只在 `development` 模式下生效。
- 确认停掉看门狗进程后驱动在超时内停车（第 1 节）。
- 软件急停：`/competition/estop` 触发后所有机器人停车，且不会自动恢复运动。

## 7. 第一次上机的顺序

每一步通过后再做下一步；单台机器人做完前不要上多台。

1. 驱动和 `robot_state_publisher` 起来，`tools/robot_preflight.py --robot robot_1` 全部通过（机器人静止）。
2. 架空车轮，低速限速下用开发模式手动控制，确认转向、方向和停车；停掉看门狗，确认驱动超时停车。
3. 落地标定轮半径和轮距（第 2 节），更新 URDF 后重新运行 preflight。
4. 检查 LiDAR 朝向和遮挡、相机内参和俯仰角；在 1–2 m 处识别一个标记，核对检测位置。
5. 单机 SLAM：手动开一圈，保存地图，看墙是否直、闭环是否正确。同时录一份输入，之后离线反复调参（`src/edgeai_exploration/README.md` 的“录包和离线建图”）。
6. AMCL：在已知起点重定位，测扫描匹配度分布，定 `scan_match_min_ratio`；开到几个点，对比实测位置，调激光模型和运动噪声。
7. 单机导航：低速开到目标点，检查碰撞监视器停车距离和原地转向余量。
8. 边缘节点分开部署（第 5 节）：preflight 的时钟检查通过，再跑单机 Task 1。
9. 第二台机器人：分别 preflight，再跑双机 Task 1、交接和 Task 2A。

## 8. 检查工具

```bash
pixi run bash -c 'source install/setup.bash && python tools/robot_preflight.py --robot robot_1'
```

检查话题和类型、频率、坐标系名、`base_footprint` 到 LiDAR 和相机的 TF、里程计 TF 是否在更新、LiDAR 是否覆盖一周且大部分光束有返回、`camera_info` 内参和图像尺寸、`cmd_vel` 有驱动订阅且只有看门狗发布、消息时间戳和本机时钟的差（在边缘节点上运行时即时钟同步检查）。有失败项时退出码为 1；`--json` 另存报告，仿真里加 `--use-sim-time`。
