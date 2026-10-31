# edgeai_robot_description

仿真机器人模型、Gazebo世界和基础仿真启动文件。

| 项目 | 内容 |
|---|---|
| 负责人 | P1（hzc333） |
| 来源 | 组织方基线（dongjw-hit/edgeai_robot_ws）+ 团队修改（ADR-0002） |
| 状态 | 1～4台仿真机器人可同时运行；几何、传感器和命名已在无界面Gazebo中验证（2026-10-23） |
| 未验证 | 带图形界面的RViz显示效果 |

多机器人约定见[ADR-0002](../../docs/decisions/0002-multi-robot-tf-and-naming.md)。一次启动多台机器人请使用`edgeai_robot_bringup`的`sim_multi_robot.launch.py`。

## 1. 文件清单

| 文件 | 作用 |
|---|---|
| `urdf/edgeai_robot_real.urdf.xacro` | 机器人模型：几何、质量、关节、传感器和Gazebo插件。参数`robot_id` |
| `worlds/empty.sdf` | Gazebo世界：约5 m × 5 m围墙、3个障碍物、物理和传感器系统。世界名为`empty` |
| `launch/sim_world.launch.py` | 启动Gazebo世界和`/clock`桥，整个仿真只启动一次 |
| `launch/sim_robot.launch.py` | 在`/robot_N`命名空间生成一台机器人：`robot_state_publisher`、spawn和ROS–Gazebo桥 |
| `launch/simulation_real.launch.py` | 兼容入口：世界 + `robot_1`（出生在(-1.5, -1.5)） |
| `rviz/realistic_robot.rviz` | 单机器人RViz配置，固定坐标系`robot_1/odom` |
| `rviz/multi_robot.rviz` | 多机器人RViz配置（robot_1、robot_2），固定坐标系`map`，需要SLAM或定位运行 |
| `config/slam_toolbox.yaml` | SLAM Toolbox参数（**内容由P2负责**），当前为robot_1建图 |
| `maps/edgeai_arena.yaml` + `.pgm` | 组织方提供的参考地图，101 × 101像素，0.05 m/像素，原点(-1.023, -1.030) |
| `maps/edgeai_arena_world.yaml` | 同一张图像，原点平移为(-2.523, -2.530)，使`map`坐标系与Gazebo世界坐标系重合；`edgeai_robot_navigation`的开发地图服务器默认使用它 |
| `CMakeLists.txt` | 把`launch`、`urdf`、`worlds`、`rviz`、`config`、`maps`安装到包共享目录 |
| `package.xml` | 包信息和运行依赖 |

## 2. 使用

```bash
# 世界 + 一台机器人（兼容入口）
ros2 launch edgeai_robot_description simulation_real.launch.py headless:=false

# 分开启动：世界一次，机器人逐台添加
ros2 launch edgeai_robot_description sim_world.launch.py headless:=true
ros2 launch edgeai_robot_description sim_robot.launch.py robot_id:=robot_2 x:=-1.5 y:=-0.8 yaw:=0.0
```

### 启动参数

| 启动文件 | 参数 | 默认值 | 说明 |
|---|---|---|---|
| `sim_world.launch.py` | `world` | 包内`worlds/empty.sdf` | 世界文件路径 |
| | `headless` | `false` | `true`时只运行Gazebo服务器（`-s`），无图形界面 |
| `sim_robot.launch.py` | `robot_id` | `robot_1` | 只能是`robot_1`～`robot_4`，同时作为命名空间、模型名和帧前缀 |
| | `x`、`y` | -1.5、-1.5 | 出生位置[m]，即`base_footprint`在世界坐标系中的位置 |
| | `yaw` | 0.0 | 出生朝向[rad] |
| | `publish_odom_tf` | `true` | 是否把DiffDrive的`robot_N/odom -> robot_N/base_footprint`桥接到`/tf`。由EKF负责该变换时设为`false` |
| `simulation_real.launch.py` | `headless` | `false` | 同上 |

## 3. 机器人模型

### 3.1 坐标系

`base_footprint`位于两个驱动轮轴中点的地面上（ADR-0002）；`base_link`在它后方0.0875 m、上方0.049 m。机器人坐标系x向前、y向左、z向上。

| 坐标系（相对`base_footprint`） | 位置[m] | 说明 |
|---|---|---|
| `base_link` | (-0.0875, 0, 0.049) | 车体中心 |
| `lidar_link` | (0.0125, 0.080, 0.155) | LiDAR中心 |
| `camera_link` | (0.0875, 0, 0.100)，上仰0.26 rad（15°） | 相机本体，x朝前。上仰是为中期标记（中心离地约0.35 m）：水平安装时0.75 m以内看不全标记，上仰后0.45～2.5 m都能看全（见`config/sim/midterm_marker_viewpoints.yaml`）。实机安装角在阶段5实测 |
| `camera_optical_frame` | 同`camera_link` | 光学约定：x右、y下、z前 |
| `imu_link` | (-0.0875, 0, 0.049) | **占位**：放在`base_link`原点，待实机测量 |

URDF中的link名不带前缀；`robot_state_publisher`的`frame_prefix`（`sim_robot.launch.py`设为`robot_N/`）给全部帧加前缀。Gazebo插件自己产生的帧名在Xacro中显式加了前缀。

### 3.2 几何与质量

| 参数 | 值 | 说明 |
|---|---|---|
| 下底盘 | 0.300 × 0.265 × 0.070 m | 相对`base_link`前移0.0375 m |
| 上板 | 0.375 × 0.265 × 0.003 m | |
| 驱动轮 | 半径0.049 m，宽0.020 m，轮距0.300 m | 位于`base_link`前方0.0875 m |
| 后脚轮 | 轮半径0.025 m，转向轴在`base_link`(-0.1675, ±0.1125)，拖尾0.025 m | 被动转向和滚动 |
| 车体质量 | 5.0 kg，惯量为临时值 | **占位**，待实机称重 |
| 外形范围（相对`base_footprint`） | x约-0.305～+0.103 m，y约±0.163 m | 脚轮转到最后方时；Nav2足迹以此为准 |

### 3.3 传感器与插件

| 名称 | 类型 | 参数 | gz话题 → ROS话题 | 帧 |
|---|---|---|---|---|
| LiDAR | `gpu_lidar` | 10 Hz，360点，-π～π，0.15～12 m，分辨率0.01 m | `/robot_N/scan` | `robot_N/lidar_link` |
| RGB-D相机 | `rgbd_camera` | 30 Hz，640 × 480，R8G8B8，水平视场1.047 rad，0.10～10 m | `/robot_N/camera/image`、`depth_image`、`camera_info` | `robot_N/camera_optical_frame` |
| IMU | `imu` | 100 Hz，角速度噪声标准差0.002 rad/s，加速度噪声标准差0.02 m/s²（**占位**，非数据手册值） | `/robot_N/imu` | `robot_N/imu_link` |
| 差速驱动 | `DiffDrive` | 里程计50 Hz；限幅：线速度0.30 m/s、角速度1.00 rad/s、线加速度0.50 m/s²、角加速度1.50 rad/s² | `/robot_N/cmd_vel`（输入）、`/robot_N/odom`、TF | `robot_N/odom -> robot_N/base_footprint` |
| 关节状态 | `JointStatePublisher` | 两个驱动轮和四个脚轮关节 | `/robot_N/joint_states` | — |

差速驱动的限幅是物理极限的开发值，不是比赛速度上限；比赛速度上限由`safety_watchdog`执行（ADR-0003）。

## 4. 接口（每台机器人）

`sim_robot.launch.py`按机器人生成桥接配置，全部话题位于`/robot_N/`下：

| ROS话题 | 类型 | 方向 | QoS（ROS侧） |
|---|---|---|---|
| `/robot_N/cmd_vel` | `geometry_msgs/msg/Twist` | ROS → Gazebo | Reliable；只由`edgeai_robot_bringup`的`safety_watchdog`发布 |
| `/robot_N/odom` | `nav_msgs/msg/Odometry` | Gazebo → ROS | Reliable |
| `/robot_N/joint_states` | `sensor_msgs/msg/JointState` | Gazebo → ROS | Reliable |
| `/robot_N/scan` | `sensor_msgs/msg/LaserScan` | Gazebo → ROS | 订阅时用Best Effort兼容 |
| `/robot_N/imu` | `sensor_msgs/msg/Imu` | Gazebo → ROS | 同上 |
| `/robot_N/camera/image`、`/robot_N/camera/depth_image` | `sensor_msgs/msg/Image` | Gazebo → ROS | 同上 |
| `/robot_N/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Gazebo → ROS | 同上 |
| `/robot_N/robot_description` | `std_msgs/msg/String` | `robot_state_publisher` | Reliable，Transient Local |
| `/tf` | `tf2_msgs/msg/TFMessage` | Gazebo DiffDrive → ROS | `robot_N/odom -> robot_N/base_footprint` |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | `robot_state_publisher` | 机器人固定变换 |

全局只有一个`/clock`桥（`sim_world.launch.py`）。所有节点使用`use_sim_time: true`。

## 5. 世界

世界坐标系原点在场地中心，x向东、y向北。

| 模型 | 中心位置[m] | 尺寸[m] | 说明 |
|---|---|---|---|
| 四面围墙 | (0, ±2.55)、(±2.55, 0) | 5.2 × 0.1 × 0.5 | 内表面在±2.5 m，场地内部5.0 m × 5.0 m |
| `obstacle_1` | (-0.8, 0.9) | 0.8 × 0.3 × 0.5 | |
| `obstacle_2` | (1.1, 0.5) | 0.3 × 1.0 × 0.5 | |
| `obstacle_3` | (0.8, -1.2)，旋转30° | 0.8 × 0.3 × 0.5 | |

系统插件：Physics（步长1 ms，目标实时速率1.0）、UserCommands（支持`gz service .../set_pose`移动模型）、SceneBroadcaster、IMU、Sensors（ogre2渲染，LiDAR和相机需要）。

仿真起点在仓库根目录`config/sim/robots.yaml`中，4台机器人位于西南角空地。

## 6. 验证

```bash
# 模型可展开并通过检查
xacro urdf/edgeai_robot_real.urdf.xacro robot_id:=robot_2 > /tmp/r2.urdf && check_urdf /tmp/r2.urdf

# 两台机器人无界面运行，检查话题、帧和独立运动
ros2 launch edgeai_robot_bringup sim_multi_robot.launch.py robots:=robot_1,robot_2 headless:=true \
  require_competition_state:=false
ros2 topic list | grep -vE '^/(robot_[1-4]/|clock$|tf$|tf_static$|parameter_events$|rosout$)'   # 应为空
ros2 topic echo /robot_2/scan --once --field header.frame_id --qos-reliability best_effort         # robot_2/lidar_link
ros2 topic echo /robot_2/camera/image --once --field header.frame_id --qos-reliability best_effort # robot_2/camera_optical_frame
timeout 2 ros2 topic pub -r 10 /robot_2/cmd_vel_manual geometry_msgs/msg/Twist '{linear: {x: 0.1}}'
gz model -m robot_1 -p    # robot_1不应移动
```

2026-10-23实测结果：

| 项目 | 结果 |
|---|---|
| 两台机器人 | 无命名空间外的机器人话题；28个TF坐标系全部带前缀；只驱动robot_2时robot_1不动 |
| 原地旋转约180° | 真实位置偏移约0.02 m（修正参考点前约0.18 m），`/odom`位置不变 |
| 四台机器人 | 实时速率0.98；LiDAR约7～8 Hz/台，相机约23 Hz；整机CPU约22%（16核） |
| SLAM（robot_1） | `/map`正常发布，`map -> robot_1/base_footprint`可查询 |

## 7. 已知限制

- **仿真不提供点云。**Gazebo的RGB-D点云数据使用`camera_link`轴向（x朝前），却以`camera_optical_frame`作为帧名，数据与帧名不一致，所以不桥接`/robot_N/camera/points`。需要时用`depth_image_proc`从深度图和CameraInfo生成。实机RealSense驱动不受影响。
- **碰撞时里程计漂移**：撞墙后Gazebo中机器人停下，但`/odom`继续累积（组织方README已知问题）。
- **原地旋转残余偏移约0.02 m**：来自脚轮拖拽和轮子打滑。
- **占位值**：车体质量和惯量、IMU安装位置和噪声，待实机数据替换。
- **与组织方指南的差异**：`base_footprint`位置、相机坐标系和话题命名均已改变（见仓库根目录README“与组织方指南的差异”）。
- **参考地图较旧**：`maps/edgeai_arena`由组织方在旧几何下建成，描述的是同一个世界，可用于开发测试。
