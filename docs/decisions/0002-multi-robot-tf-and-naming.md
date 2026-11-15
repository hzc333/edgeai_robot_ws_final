# ADR-0002：多机器人TF、命名和机器人参考点

- 状态：已采用（2026-10-23）。
适用范围：仿真和实机的全部机器人本地节点

## 背景

当前基线只支持一台机器人：Gazebo插件、桥接和启动文件写死了`/cmd_vel`、`/odom`、`/scan`、`/tf`和不带前缀的坐标系。第二台机器人一启动，话题和TF就会冲突。

另外，2026-10-23的仿真实测发现两个几何问题：

1. **相机图像坐标系错误**：`/camera/image`和`/camera/camera_info`的`frame_id`是`edgeai_robot_real/base_footprint/rgbd_camera`，而启动文件把它作为与`camera_link`同向的坐标系（零旋转静态TF）。图像数据实际是光学方向（z朝前），所以由图像算出的位姿经TF转换后坐标轴会错位。原因是Xacro把`<optical_frame_id>`写在`<sensor>`下，SDF忽略了它。
2. **`base_footprint`不在驱动轮轴中点**：驱动轮在`base_link`前方0.0875 m，而`base_footprint`在`base_link`正下方。DiffDrive按旋转中心积分里程计，却把结果作为`base_footprint`发布。实测原地旋转约171°后，真实位置移动约0.18 m，而`/odom`位置不变。

## 决定

### 1. TF树

- 所有机器人共用全局`/tf`和`/tf_static`。
- 除`map`外，所有坐标系带`robot_N/`前缀，例如`robot_2/odom`、`robot_2/base_footprint`、`robot_2/lidar_link`、`robot_2/camera_optical_frame`、`robot_2/imu_link`。
- `map`不带前缀，是唯一共享的全局坐标系。任一时刻每个`map -> robot_N/odom`只能有一个发布者。
- URDF中的link和joint名称不加前缀，前缀由`robot_state_publisher`的`frame_prefix`参数添加；Gazebo插件和传感器自己产生的帧名（DiffDrive的`frame_id`/`child_frame_id`、传感器的`gz_frame_id`和相机的`optical_frame_id`）在Xacro中显式加前缀。
- Nav2自带的`navigation_launch.py`会把`/tf`重映射为命名空间内的`tf`。本项目的Nav2启动文件**不做这个重映射**。

### 2. 话题和服务

每台机器人的全部端点位于`/robot_N/`下：

| 名称 | 类型 | 说明 |
|---|---|---|
| `/robot_N/cmd_vel` | `geometry_msgs/Twist` | 底盘速度，唯一发布者为`safety_watchdog`（ADR-0003） |
| `/robot_N/odom` | `nav_msgs/Odometry` | |
| `/robot_N/joint_states` | `sensor_msgs/JointState` | |
| `/robot_N/imu` | `sensor_msgs/Imu` | |
| `/robot_N/scan` | `sensor_msgs/LaserScan` | Sensor Data QoS |
| `/robot_N/camera/image`、`/robot_N/camera/depth_image` | `sensor_msgs/Image` | Sensor Data QoS，`frame_id`为`robot_N/camera_optical_frame` |
| `/robot_N/camera/camera_info` | `sensor_msgs/CameraInfo` | |
| `/robot_N/camera/points` | `sensor_msgs/PointCloud2` | 仅实机；仿真不提供（见第4节） |
| `/robot_N/robot_description` | `std_msgs/String` | |
| `/robot_N/state`、`/robot_N/phase_ack` | 共享接口 | |
| `/robot_N/execute_navigation_goal`、`/robot_N/execute_rescue_task` | 共享接口（动作） | |
| `/robot_N/relocalize` | 共享接口（服务） | |

全局话题只有：`/clock`、`/tf`、`/tf_static`、`/map`、`/competition/*`、`/edge/*`、`/diagnostics`、`/rosout`、`/parameter_events`。

速度消息使用不带时间戳的`geometry_msgs/Twist`，与Jazzy版Nav2默认设置和Gazebo桥接一致。

### 3. 机器人参考点

`base_footprint`位于两个驱动轮轴的中点，在地面上。`base_link`相对`base_footprint`的位置为`(-0.0875, 0, 0.049)`。所有传感器仍相对`base_link`定义，几何不变，但相对`base_footprint`的数值改变：

| 坐标系 | 修改前（相对`base_footprint`） | 修改后 |
|---|---|---|
| `lidar_link` | (0.100, 0.080, 0.155) | (0.0125, 0.080, 0.155) |
| `camera_link` | (0.175, 0, 0.100) | (0.0875, 0, 0.100) |

实机的轮式里程计同样以轴中点为参考。

### 4. 相机坐标系

相机传感器在`<camera>`内设置`<optical_frame_id>robot_N/camera_optical_frame</optical_frame_id>`，在`<sensor>`下设置`<gz_frame_id>robot_N/camera_link</gz_frame_id>`。图像、深度图和CameraInfo的`frame_id`都是`robot_N/camera_optical_frame`（光学约定：x右、y下、z前）。

仿真不提供`/robot_N/camera/points`。实测Gazebo的RGB-D点云数据使用`camera_link`轴向（x朝前），但消息头标的是光学坐标系，二者不一致；Gazebo也不能给图像和点云分别设置坐标系。需要点云时，用`depth_image_proc`从深度图和CameraInfo生成。实机RealSense驱动的点云坐标系正确，照常发布。

## 原因

- 目标融合和救援验证器需要在同一棵TF树中查询`map -> robot_N/base_footprint`和`map -> robot_N/camera_optical_frame`，每台机器人独立TF树做不到这一点。
- 统一命名使边缘端各模块只需替换机器人ID即可使用任意机器人，不需要了解底层驱动。
- 参考点放在轴中点后，里程计、Nav2足迹和原地旋转的几何关系才一致。

## 替代方案

- **每台机器人独立TF树（`/robot_N/tf`）**：Nav2默认做法，但边缘节点需要为每台机器人维护独立TF缓冲，并自行拼接`map`，复杂且容易出错。不采用。
- **保持`base_footprint`位置，另加旋转中心坐标系**：DiffDrive里程计仍会把轴心轨迹作为`base_footprint`发布，问题依旧。不采用。

## 影响

| 对象 | 影响 |
|---|---|
| SLAM/探索 | `slam_toolbox.yaml`改为`odom_frame: robot_1/odom`、`base_frame: robot_1/base_footprint`、`scan_topic: /robot_1/scan` |
| 感知 | 检测器订阅`/robot_N/camera/*`；观测的`frame_id`为`robot_N/camera_optical_frame`；旧的零旋转兼容TF不再存在 |
| 救援协调 | 只通过`/robot_N/state`和两个动作使用机器人 |
| 所有模块 | 启动命令改为多机器人启动文件；README中的TF偏移数值更新 |

## 验收

1. 两台机器人同时运行时，`ros2 topic list`中没有未加命名空间的机器人话题。
2. `view_frames`显示两棵以`robot_N/odom`为根的子树，没有不带前缀的机器人坐标系。
3. 图像`frame_id`为`robot_N/camera_optical_frame`。
4. 原地旋转约180°后，Gazebo真实位置变化和`/odom`位置变化都小于0.01 m。

## 附录：Task 1到Task 2的定位交接顺序

为保证`map -> robot_1/odom`始终只有一个发布者：

1. `system_manager`进入`HANDOVER`，所有机器人停车（ADR-0003的门控）。
2. `system_manager`记录SLAM给出的最后位姿（TF `map -> robot_1/base_footprint`）。
3. `map_manager`停止SLAM发布`map -> robot_1/odom`，由`map_manager`发布冻结的`/map`。
4. `system_manager`以`MODE_POSE_HINT`和第2步的位姿调用`/robot_1/relocalize`（系统内部交接，不是操作员注入）。
5. 机器人的`localization`节点启动AMCL并确认收敛后返回成功。
6. 交接前后用`ros2 run tf2_ros tf2_monitor map robot_1/odom`确认只有一个发布者。

实现：`nav_mode:=slam`时AMCL和`localization`节点已启动但未激活，第4步的一次`relocalize`调用即完成切换；`slam`模式下禁止自动激活AMCL。`HANDOVER`阶段内不带协方差的位姿提示使用0.03 m / 0.03 rad的默认标准差（详见`src/edgeai_robot_navigation/README.md`）。
