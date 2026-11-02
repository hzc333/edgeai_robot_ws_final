# 基础仿真与手动操作

本文是原根目录README中“开发仿真”部分的整理版，适合单独调试机器人模型、传感器、手动驾驶和手动SLAM建图。
**中期全流程不需要这些步骤**，直接按根目录[README](../README.md)运行`pixi run midterm`即可。

所有命令都在仓库根目录执行，并且需要先完成`pixi install --locked`和`pixi run build`。

## 1. 启动开发仿真

```bash
./start_sim.sh        # 1台机器人robot_1
./start_sim.sh 2      # robot_1和robot_2，最多4台
```

该脚本会根据自身位置确定工作空间路径，优先激活Pixi环境；若Pixi不可用，则加载系统ROS 2 Jazzy。随后它加载`install/setup.bash`，导出`EDGEAI_CONFIG_DIR`，启动`edgeai_robot_bringup`的`sim_multi_robot.launch.py`并打开RViz（1台时用`realistic_robot.rviz`，多台时用`multi_robot.rviz`）。在脚本所在终端按**Ctrl+C**即可停止整个会话。

这里用的是开发世界`src/edgeai_robot_description/worlds/empty.sdf`，出生位姿在`config/sim/robots.yaml`（原点在场地中心，例如robot_1在(-1.5, -1.5)）。它**不是**中期场地；中期场地见根目录README。

多机器人约定见[ADR-0002](decisions/0002-multi-robot-tf-and-naming.md)：每台机器人的话题位于`/robot_N/`下，坐标系带`robot_N/`前缀。不带图形界面运行：

```bash
ros2 launch edgeai_robot_bringup sim_multi_robot.launch.py robots:=robot_1,robot_2 headless:=true
```

单独的启动文件：`edgeai_robot_description`中的`sim_world.launch.py`（世界和`/clock`，只启动一次）和`sim_robot.launch.py`（一台机器人，参数`robot_id`、`x`、`y`、`yaw`）。`simulation_real.launch.py`保留为“世界加robot_1”的兼容入口。

## 2. 在其他终端中使用ROS命令

每打开一个额外终端，都需要进入仓库并激活项目环境：

```bash
cd /path/to/edgeai_robot_ws
pixi shell
source install/setup.bash
```

使用系统ROS时改为：

```bash
cd /path/to/edgeai_robot_ws
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source install/setup.bash
```

### 主要话题

下表以`robot_1`为例，其他机器人把前缀换成`robot_N`。

| 话题 | ROS消息类型 | 用途 |
| --- | --- | --- |
| `/robot_1/cmd_vel` | `geometry_msgs/msg/Twist` | 底盘速度指令，只由`safety_watchdog`发布 |
| `/robot_1/cmd_vel_manual` | `geometry_msgs/msg/Twist` | 开发模式下的手动速度指令 |
| `/robot_1/odom` | `nav_msgs/msg/Odometry` | Gazebo差速驱动里程计 |
| `/robot_1/joint_states` | `sensor_msgs/msg/JointState` | 仿真关节状态 |
| `/robot_1/scan` | `sensor_msgs/msg/LaserScan` | LiDAR扫描数据（Best Effort） |
| `/robot_1/imu` | `sensor_msgs/msg/Imu` | IMU（100 Hz） |
| `/robot_1/camera/image` | `sensor_msgs/msg/Image` | RGB图像，帧`robot_1/camera_optical_frame` |
| `/robot_1/camera/depth_image` | `sensor_msgs/msg/Image` | 深度图像 |
| `/robot_1/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | 相机标定信息 |
| `/robot_1/robot_description` | `std_msgs/msg/String` | 机器人模型 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 全局动态TF，包括`robot_1/odom` → `robot_1/base_footprint` |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 全局静态TF |
| `/clock` | `rosgraph_msgs/msg/Clock` | 仿真时间 |
| `/map` | `nav_msgs/msg/OccupancyGrid` | 占据栅格地图（SLAM或地图服务器） |

单机器人RViz配置使用`robot_1/odom`作为固定坐标系，多机器人配置使用`map`（需要SLAM或定位运行后才有）。

```bash
ros2 topic list
ros2 topic echo /robot_1/odom --once
```

## 3. 手动移动与停止

每台机器人的`/robot_N/cmd_vel`只由`safety_watchdog`发布（[ADR-0003](decisions/0003-velocity-safety-chain.md)），**不要直接向它发布**。`./start_sim.sh`以开发模式启动，并关闭“必须有比赛阶段管理器”的要求，所以可以向`/robot_N/cmd_vel_manual`发送手动指令。看门狗把速度限制在`config/competition_parameters.yaml`的开发上限（目前0.30 m/s、0.80 rad/s），输入停止0.5 s后自动输出零速度。

确认机器人前方没有障碍后，发送低速前进指令：

```bash
ros2 topic pub --rate 10 /robot_1/cmd_vel_manual geometry_msgs/msg/Twist \
  '{linear: {x: 0.1}, angular: {z: 0.0}}'
```

按**Ctrl+C**停止发布器，机器人会在0.5 s内停车。手动控制只在`run_mode:=development`下有效；演练和比赛模式中看门狗忽略手动指令（规则7.1禁止遥控）。

## 4. 手动SLAM建图

保持仿真运行。在另一个已准备好的终端中执行：

```bash
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:="$PWD/src/edgeai_robot_description/config/slam_toolbox.yaml" \
  use_sim_time:=true
```

该配置为`robot_1`建图：使用`/robot_1/scan`、`robot_1/base_footprint`、`robot_1/odom`和`map`，分辨率0.05 m。仿真启动脚本不会自动启动SLAM。（中期流程里SLAM由P2的`map_manager`启动，参数在`src/edgeai_exploration/config/slam.yaml`。）

SLAM启动后，在RViz中将**Fixed Frame**设为`map`，添加订阅`/map`的**Map**显示项。让机器人在空旷区域缓慢行驶，并适当回到已经观察过的区域。

保存占据栅格地图：

```bash
mkdir -p maps
ros2 run nav2_map_server map_saver_cli -f "$PWD/maps/my_map" \
  --ros-args -p use_sim_time:=true
```

生成`maps/my_map.yaml`及对应图像。这是占据栅格导出结果，不是SLAM Toolbox的序列化位姿图。包内自带地图`src/edgeai_robot_description/maps/edgeai_arena.yaml`不会被覆盖。

## 5. 使用系统ROS（不推荐）

Pixi是唯一的标准环境（见根目录README）。只做非感知模块的本地开发时，也可以用系统ROS 2 Jazzy：

```bash
./tools/install_system_ros_deps.sh  # 每台机器首次准备时执行
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
colcon build --symlink-install
python3 tools/check_environment.py --allow-system-opencv
```

Ubuntu 24.04自带OpenCV 4.6，与Pixi固定的4.12系列不同，ArUco行为也不同，系统ROS下的感知结果不能作为验收证据。在Pixi和系统ROS之间切换前，必须先删除`build/`、`install/`和`log/`再重新构建。

## 6. 与组织方指南的差异（ADR-0002）

本仓库对组织方基线做了几处几何修正，`Initial_Robot_Simulation_and_SLAM_Guide.md`中的部分数值因此不再适用：

- **`base_footprint`位于驱动轮轴中点。**原基线的`base_footprint`在`base_link`正下方，比实际旋转中心靠后0.0875 m，原地旋转约180°会产生约0.18 m的里程计误差。修正后，`lidar_link`相对`base_footprint`约为(0.0125, 0.080, 0.155) m。
- **相机图像使用光学坐标系。**`/camera/image`、`/camera/depth_image`和`/camera/camera_info`的`frame_id`为`camera_optical_frame`（x右、y下、z前）。
- **相机上仰15°**（中期需要，S1标记中心离地约0.35 m），见`edgeai_robot_description`的README。
- **仿真不再桥接`/camera/points`。**需要点云时，请用`depth_image_proc`由深度图和CameraInfo生成。

## 7. 已知限制：碰撞时里程计漂移

机器人接触墙壁或障碍物后，Gazebo中的实体运动会停止，而`/odom`可能继续变化，RViz中的机器人也会继续移动。`/odom`来自Gazebo DiffDrive插件，应视为估计值。机器人受阻后应立即停止；建图时避免主动撞击障碍物。
