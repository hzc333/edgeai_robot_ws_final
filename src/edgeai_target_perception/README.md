# P3 视觉感知：目标检测、多机器人融合与 Task 1 目标导出（Phase 1–5）

本目录实现相机光学坐标系下的 ArUco 检测/位姿估计、RGB-D 深度校验、
反投影和刚体坐标变换，以及单机器人 ROS 2 检测节点。节点只发布原始
`TargetObservation`；集中式融合节点按观测时间查询 TF，并发布 map 坐标系下的目标注册表。
Phase 5 增加开发模式救援区生成，以及供比赛控制层调用的 Task 1 不可变快照、
Schema v2 校验和原子写出组件。竞赛规则未公布的官方救援几何不会被推测或替代。

## 文件

- `edgeai_target_perception/__init__.py`：独立算法包入口。
- `edgeai_target_perception/aruco_core.py`：配置适配、ArUco 检测、IPPE 候选筛选、RGB-D 一致性和二义性诊断。
- `edgeai_target_perception/geometry.py`：深度单位转换、带畸变的像素反投影、刚体变换及 M→T 转换。
- `edgeai_target_perception/target_detector.py`：ROS 2 单机器人节点及观察消息转换。
- `edgeai_target_perception/fusion_core.py`：独立于 ROS 的身份校验、去重、离群拒绝和保守融合。
- `edgeai_target_perception/target_fusion_node.py`：时间戳 TF 查询、比赛状态隔离和 `/edge/targets` 注册表发布。
- `edgeai_target_perception/rescue_geometry.py`：Marker→目标平面位姿投影和版本化救援区计算。
- `edgeai_target_perception/task1_export.py`：Finalize 请求版本匹配、运行内幂等快照、Schema v2 构造与原子文件写入。
- `launch/target_detector.launch.py`：在 `robot_N` 命名空间启动节点。
- `launch/target_fusion.launch.py`：启动边缘侧集中式多机器人融合节点。
- `test/`：几何、融合算法和 ROS 消息适配合成测试。

## 依赖与测试

项目标准环境使用 OpenCV 4.12、NumPy、PyYAML、`jsonschema`、ROS 2 Jazzy 和 `cv_bridge`。
算法要求 `cv2.aruco.ArucoDetector`；配置入口复用 `edgeai_common.config` 的统一解析，
比赛模式继续执行 official 参数门禁。本包不引入新依赖、不更改 Pixi 配置或共享消息。

从仓库根目录在现有 Pixi 环境构建并测试：

```bash
pixi run colcon build --packages-up-to edgeai_target_perception --symlink-install
pixi run colcon test --packages-select edgeai_target_perception --event-handlers console_direct+
pixi run colcon test-result --verbose
```

启动单机器人节点（topic 按 `robot_id` 解析为机器人前缀路径）：

```bash
ros2 launch edgeai_target_perception target_detector.launch.py \
  robot_id:=robot_1 run_mode:=development use_depth:=false
```

输入为 `camera/image`、`camera/camera_info` 和可选的 `camera/depth_image`；
输出为 `target_observations`，比赛状态从全局 `/competition/state` 读取。
启用深度可设置 `use_depth:=true`。16UC1 深度按 ROS 约定解释为毫米，32FC1 按米解释；
深度帧必须属于同一光学坐标系、尺寸匹配，时间戳差不得超过 `depth_sync_tolerance_s`
（默认 0.05 秒）。深度暂缺或过期时以纯 PnP 继续，但歧义位姿不会发布。

## 使用

把 `src/edgeai_target_perception` 和 `src/edgeai_common` 加入 `PYTHONPATH`。

```python
from edgeai_target_perception.aruco_core import ArucoCore

core = ArucoCore.from_repository_config('development', config_dir='config')
# image: uint8 灰度或 BGR；K: 3x3 内参；D: OpenCV 畸变系数或 None。
results = core.detect(image, K, D)
for result in results:
    if result.valid:
        print(result.identity.target_id, result.rvec, result.tvec,
              result.reprojection_error_px)
    else:
        print(result.marker_id, result.rejection_reason)
```

也可用 `from_resolved_parameters(parameters)` 消费公共解析器结果；不重复实现 official/development
回退。competition 保留公共解析器的全部官方参数门禁。正式官方参数尚未公布时不能静默回退。

算法入口 `estimate_pose(marker_id, corners, K, D)` 支持直接测试已知角点。
角点遵守 `docs/marker_and_rescue_geometry.md`：TL/TR/BR/BL，对应
`(-L/2,+L/2,0)`、`(+L/2,+L/2,0)`、`(+L/2,-L/2,0)`、`(-L/2,-L/2,0)`。
物理尺寸是黑框外沿边长（米）。用 `SOLVEPNP_IPPE_SQUARE` 求候选解，要求所有角点在
相机前方，按重投影 RMS 最小选解。`rvec/tvec` 表示 M→相机光学帧，满足
`p_camera = Rodrigues(rvec) @ p_marker + tvec`，不进行 M→T 或 map 变换。

重投影误差定义为四角点二维欧氏残差的 RMS，单位像素。
可显式传入 `max_reprojection_error_px` 拒绝高误差解；现有配置没有该阈值，默认仅计算，
不擅自写入比赛参数。`ambiguity` diagnostics 比较正深度 IPPE 候选的重投影误差间距和姿态差异；
默认 1 px 内标记为 ambiguous，但诊断本身不拒绝位姿，且可由构造参数调整。未知 ID
返回 `unknown_marker_id`，identity/位姿为空，valid=false。
调用者必须检查 valid，不能仅凭有 tvec 就接受结果。PnP 失败、退化角点、无正深度解均返回
拒绝原因；无检测返回空列表；非法输入或配置抛出 ValueError。

RGB-D 调用可传入 `depth_image`、`depth_scale_m` 和 `depth_tolerance_m`。深度图必须已与彩色图
配准且尺寸一致；提供深度图时必须显式提供尺度（例如 uint16 毫米深度用 `0.001`，浮点米深度用 `1.0`），
因为仓库当前没有规定传感器深度编码。
零、负值和非有限样本转换为 NaN；单位转换结果以米表示。PnP 候选在 Marker 内部的九个位置
采样局部有效深度，至少三个位置可用时计算深度误差中位数，超容差候选被拒绝；没有足够有效
采样返回 `depth_unavailable`，没有一致候选返回 `depth_inconsistent`。深度只用于已配准图像，
不执行时间同步或重配准。

`back_project_pixels` 将像素及米制深度反投影到 `camera_optical_frame`，畸变使用 OpenCV 的
`undistortPoints`。`RigidTransform` 提供右手刚体变换、逆和组合；组合 `A.compose(B)` 先应用
B 再应用 A。`marker_to_target_pose` 按几何协议右乘固定 `R_M_T`，并保持 Marker 中心平移不变。

ROS 节点验证图像和 CameraInfo 的 frame 为 `robot_N/camera_optical_frame`，拒绝无效标定、
不匹配分辨率和不支持的图像编码。仅当 `/competition/state` 在 3 秒内更新、含非空 `run_id`，
且处于 Task 1/Task 2A/Task 2B 非冻结、非急停阶段时发布。消息 Header 使用 RGB 图像的时间戳
和光学 frame；位姿为 Marker→相机光学帧，四元数由 Rodrigues 旋转生成。
未知 Marker ID、PnP 失败、深度不一致和二义性结果不发布。confidence 是由重投影 RMS 导出的
启发式分数，不是校准概率；没有经过测量验证的协方差时，消息以高对角协方差表示不确定性。

## 验证范围与限制

测试覆盖已知位姿合成图像、不同距离/倾角、轻微模糊、普通/协同/未知 ID、畸变参数、
重投影误差、双解选择、负深度/角点穿越相机平面、空图像和错误输入。
合成图像测试容差为距离的 2.5%、姿态 0.12 rad、重投影 1 px，只用于回归，非比赛验收标准。

输入图像必须与标定分辨率一致；RGB 输入应先转 BGR/灰度，鱼眼模型不属于此入口。
平面 PnP 存在二义性，最小重投影误差不能保证真实解；二义性诊断不等于概率置信度，
本阶段不输出未经标定的置信度或协方差。尚无真实相机数据集、遮挡/强模糊性能证明、
时间同步或深度与图像配准验证。
本阶段不修改现有 ROS 消息、Schema、配置或几何协议。

Phase 1 和 Phase 2 的合成单元测试应在项目 Linux Pixi 环境中运行；不增加依赖。

## 单机器人 Gazebo 集成测试

集成 fixture 位于 `test/fixtures/aruco_test_world.sdf`，不会改动正式竞赛世界。
Marker 是 DICT_4X4_50、ID 1，黑框外沿边长 0.15 m；模型用黑白几何格和白色底板构造，
外侧留有 quiet zone。生成的 PNG 是参考图像，不依赖 Gazebo 的纹理渲染。
它位于 robot_1 相机前方约 0.9 m，
正面法线朝向相机并绕竖直轴偏转约 0.4 rad。位姿按 robot_1 出生位置及 xacro 中
`camera_link`/`camera_optical_frame` 变换计算，而不是将 Gazebo 世界轴误认为相机光学轴。

在仓库根目录用已有 Pixi 环境构建并生成模型和 PNG 参考图：

```bash
pixi run colcon build --packages-up-to edgeai_robot_bringup edgeai_robot_description edgeai_target_perception --symlink-install
pixi run python src/edgeai_target_perception/test/fixtures/generate_aruco_marker.py
```

**终端 A：** 启动独立测试世界及 robot_1。`GZ_SIM_RESOURCE_PATH` 让 Gazebo 找到 Marker
模型。该命令不运行 `start_sim.sh`，因此不启动正式世界、RViz 或 robot stack。

```bash
cd /home/xiaomin/projects/edgeai_robot_ws
pixi shell
source install/setup.bash
export EDGEAI_CONFIG_DIR="$PWD/config"
export GZ_SIM_RESOURCE_PATH="$PWD/src/edgeai_target_perception/test/fixtures${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export ROS_DOMAIN_ID=67
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 launch edgeai_robot_bringup sim_multi_robot.launch.py \
  robots:=robot_1 \
  world:="$PWD/src/edgeai_target_perception/test/fixtures/aruco_test_world.sdf" \
  headless:=true rviz:=false stack:=false config_dir:="$PWD/config"
```

**终端 B：** 持续发布临时 Task 1 状态，使用与 detector `control_state` 订阅匹配的可靠、
Transient Local QoS。持续发布满足 3 秒状态新鲜度限制：

```bash
cd /home/xiaomin/projects/edgeai_robot_ws
pixi shell
source install/setup.bash
export ROS_DOMAIN_ID=67
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 topic pub -r 1 --qos-reliability reliable --qos-durability transient_local \
  /competition/state edgeai_robot_interfaces/msg/CompetitionState \
  "{run_id: 'run-p3-gazebo-test', phase: 1, frozen: false, estop_active: false}"
```

**终端 C：** 先检查真实 ROS 图像、标定、深度和时钟，然后启动 detector：

```bash
cd /home/xiaomin/projects/edgeai_robot_ws
pixi shell
source install/setup.bash
export EDGEAI_CONFIG_DIR="$PWD/config"
export ROS_DOMAIN_ID=67
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 topic echo /clock --once
ros2 topic echo /robot_1/camera/image --once --field header.frame_id --qos-profile sensor_data
ros2 topic echo /robot_1/camera/image --once --field encoding --qos-profile sensor_data
ros2 topic echo /robot_1/camera/camera_info --once --field header.frame_id --qos-profile sensor_data
ros2 topic echo /robot_1/camera/camera_info --once --field k --qos-profile sensor_data
ros2 topic echo /robot_1/camera/depth_image --once --field encoding --qos-profile sensor_data
ros2 run tf2_ros tf2_echo robot_1/camera_link robot_1/camera_optical_frame
ros2 launch edgeai_target_perception target_detector.launch.py \
  robot_id:=robot_1 run_mode:=development config_dir:="$PWD/config" use_depth:=true
```

**终端 D：** 等待观察消息：

```bash
cd /home/xiaomin/projects/edgeai_robot_ws
pixi shell
source install/setup.bash
export ROS_DOMAIN_ID=67
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 topic echo /robot_1/target_observations \
  edgeai_robot_interfaces/msg/TargetObservation --once --timeout 30 \
  --qos-reliability best_effort
```

验收字段：`marker_id=1`、`target_id=T1`、`robot_id=robot_1`、
`run_id=run-p3-gazebo-test`，frame 为 `robot_1/camera_optical_frame`，Header stamp 与触发
该检测的 RGB 图像时间戳相同。核对 RGB 和深度的 frame/尺寸/时间戳及 depth encoding；
深度同步误差应不超过默认 0.05 秒。测试后在各终端 Ctrl+C 停止模拟器、detector 和状态发布。

### WSL2 实测结果

- Gazebo Harmonic 8.10.0 成功加载独立 `aruco_test` 世界并生成 robot_1；ROS `/clock`、
  RGB、CameraInfo、深度和 TF 均有数据。
- RGB 为 `640x480 rgb8`，深度为 `640x480 32FC1`；相机图像、深度和 CameraInfo 的 frame
  均为 `robot_1/camera_optical_frame`。
- `camera_link` 到 `camera_optical_frame` 的静态变换可查到；平移为零，RPY 约为
  `(-1.571, 0, -1.571)`。
- 开启深度模式时收到有效观察：`marker_id=1`、`target_id=T1`、`robot_id=robot_1`、
  `run_id=run-p3-gazebo-test`。观察 Header stamp 与对应 RGB、深度及 CameraInfo stamp
  完全相同，光学 frame 正确；PnP 深度约 `0.897 m`。
- 深度路径通过，本次无需 RGB-only 对照。Gazebo 日志仍出现 `/dev/dri` EGL 权限警告，
  但本次传感器渲染和端到端检测成功；硬件加速状态未单独确认。

若深度模式没有观察，先看 detector 是否报告 `depth_inconsistent`、无效深度或时间同步问题，
随后以 `use_depth:=false` 重启 detector 重测纯 RGB PnP。不要为通过测试而更改 PnP 歧义或深度
拒绝阈值。若 RGB-only 仍未发布，检查 Gazebo 是否找到 Marker 模型、Marker 是否在图像中可见，以及
日志中的 unknown ID、PnP 或 ambiguity 诊断。

## Phase 4：多机器人目标融合

在每台参与机器人的 `target_detector` 输出 `/<robot_id>/target_observations` 后，
边缘侧启动一个集中式融合节点：

```bash
ros2 launch edgeai_target_perception target_fusion.launch.py \
  robot_ids:=robot_1,robot_2,robot_3,robot_4 \
  run_mode:=development config_dir:="$PWD/config" use_sim_time:=true
```

`robot_ids` 应只列出本次运行中实际启用的机器人。节点订阅这些机器人的原始观察和
`/competition/state`，并以 `TargetArray` 发布权威快照 `/edge/targets`。输入观察必须是
`robot_N/camera_optical_frame`，且 `robot_id` 必须与订阅话题命名空间一致。每条观察在
其 `header.stamp` 对应的时刻查询 `map -> robot_N/camera_optical_frame`；TF 暂不可用时只在
配置的有限窗口内重试，过期、未来时间、未知身份、低置信度和空间离群观察会被拒绝。
不会用最新 TF、当前机器人位姿或其他时间的变换替代。

身份由解析后的 `aruco.id_mapping` 严格校验，marker ID、target ID 和类型必须与配置一致。
注册表每个 target 每台机器人只保留最新一条有效观测；新帧替换该机器人旧样本，不增加
`observation_count`，避免把同一相机连续帧当成独立证据。不同机器人当前样本在配置的
`perception.fusion_position_gate_m` 内才参与融合；融合权重结合检测置信度和位置协方差。
合并协方差保留输入不确定性并增加跨机器人位置散布，不因样本数而虚假缩小。目标身份映射
保持稳定，`TargetArray.revision` 在注册表接受/删除样本时递增。每次 `run_id`、`map_id`、
`map_revision` 或 `reset_count` 改变，以及 reset 开始或 ROS 时间倒退，都会清空注册表。
观察过期时间、TF 查询时限及重试窗口来自 `config/interface_runtime.yaml`。开发/演练模式下，
融合节点使用版本匹配的救援几何发布 `rescue_poses`；官方规则缺失时不会生成救援区。在线
`eligible_for_basic_rescue` 和 `TargetArray.task1_frozen` 在 P2 finalize 协调完成前保持 false；
Task 1 导出快照只在冻结副本中将目标标记为 `eligible_for_basic_rescue=true`。

## Phase 5：救援区、目标快照与 Schema v2 导出

`target_fusion` 通过 `R_M_T` 将地图系 Marker 姿态转换成目标系，再按几何约定投影到
`map` 平面；目标面法线接近竖直（相对竖直轴夹角小于 45°）时拒绝该观测并报告
`marker_not_vertical`。开发/演练模式只有在已解析的 `rescue.geometry_version` 与
`config/development_scenarios.yaml` 一致时才生成救援位姿：普通目标一个区，协同目标两个区，
偏移量和顺序直接取配置。没有可用的带版本几何时不生成救援区。当前 `official` 救援几何为
TBD；若将来比赛配置声明 official 几何，本节点会显式失败，而不是使用 development 规则冒充。

`task1_export.py` 提供与 ROS 无关的 `Task1SnapshotFreezer`、`build_target_list` 和
`write_target_list_atomic`。冻结函数要求 `CompetitionState`、Finalize 请求的 run/map 版本、
`TargetArray` map/target revision 全部完全一致，且状态仍处于 Task 1、未冻结、未急停、未重置。
同一 run 的重复请求返回首次保存的不可变序列化副本；同一 run 若 map 或 target revision 改变则
报冲突，不会替换已冻结目标；新 run 可独立冻结。导出只接受与快照相同的地图元数据，目标按
`target_id` 排序，观察机器人排序稳定，并以现有 `schemas/target_list.schema.json` Draft 2020-12
Schema 和救援几何语义共同校验。写入使用同目录临时文件、`fsync` 和 `os.replace`，失败时保留
已有目标文件。

**Finalize 集成仍有 P2 前置阻塞，当前不能声称端到端冻结/导出已接通：**

- `/edge/finalize_task1` 的接口明确要求同时协调 map snapshot、target list、Manifest 并整体原子化；
  P2 对阶段控制、Manifest 和该服务拥有运行时权威。仓库当前没有该服务的 server，也没有地图
  快照/元数据提供者；P3 不应另建同名服务或只写 targets.json 后错误返回整体 finalize 成功。
- 当前 `target_fusion` 的 `TargetArray.revision` 是其本地 registry revision，`CompetitionState.target_revision`
  由 P2 管理，但融合节点未同步或更新它。严格快照组件会拒绝两者不一致；需先由 P2/P3 确定
  revision 单一权威及同步协议，并由 P2 在 finalize 操作中冻结/锁定一致快照，再传入地图元数据、
  UTC 冻结时间和运行目录路径。服务所有权或 revision 对齐尚未确认前，不会更改 P2 生命周期状态。
- 已冻结快照是不可变副本，不再依赖实时 TF 或融合 registry；但服务层需确保先完成该精确版本捕获，
  再改变阶段/地图版本。上游融合继续按既有规则拒收过期观察及 run/map/reset 变更；它们不能改写
  已捕获副本。当前 freezer 的幂等记录仅存在于进程内存；P2 服务必须在成功响应前持久化冻结副本/
  Manifest，并在重启后从运行产物恢复同一结果。`CompetitionState`新鲜度也必须由服务/状态管理层
  在冻结事务中确认。

Phase 5 合成测试：

```bash
pixi run colcon build --packages-up-to edgeai_target_perception --symlink-install
pixi run colcon test --packages-select edgeai_target_perception --event-handlers console_direct+
pixi run colcon test-result --verbose
```

### P2 地图/TF 前置条件与假设

`map` 是唯一共享全局 frame，ADR-0002 要求全局 TF 树包含每台机器人的
`map -> robot_N/odom -> robot_N/base_footprint -> ... -> robot_N/camera_optical_frame`。
当前仓库的 SLAM Toolbox 配置只将 `robot_1/odom` 配到共享 `map`；现有导航说明也是单一
主 SLAM 机器人，其他机器人不能仅凭各自的 `odom` 或相同的 `map` 字符串就视作坐标已对齐。
P2/map manager 或定位系统必须为每台参与融合的机器人提供有效、同一地图版本下的
`map -> robot_N/odom` 变换，并在 Task 1→Task 2 交接时单一发布者地维护它。若缺少变换，
融合节点按时间戳有限重试后丢弃观察；若变换存在但地图实际未对齐，当前接口没有地图对齐
不确定度可供补偿，空间门限只能拒绝部分错误关联，无法修复系统性坐标偏差。

`TargetArray.revision` 是本融合节点的注册表版本；节点不篡改 `/competition/state` 中由
system manager 拥有的 `target_revision`。若下游要求两者严格相等，需由系统管理接口明确
版本同步协议，当前共享接口没有该写入服务。

### 双机器人 Gazebo 端到端融合测试

测试使用独立世界 `test/fixtures/aruco_two_robot_test_world.sdf`，不修改正式竞赛世界。
Marker 是 DICT_4X4_50 ID 1（T1），边长 0.15 m，中心位于 Gazebo 测试世界坐标
`(-0.45, -1.15, 0.15) m`。两个机器人使用 `config/sim/robots.yaml` 中的出生位姿
`robot_1=(-1.5, -1.5, 0)`、`robot_2=(-1.5, -0.8, 0)`；结合 xacro 的相机安装位置，
两台相机均能看到 Marker。

测试中的 `map` 明确等同于 Gazebo 世界坐标系。DiffDrive odometry 在出生位置从零开始，
所以通过静态变换把已知出生位姿映射到 `map`。这是仅供仿真的 ground-truth TF，不是
P2 SLAM、地图管理器或生产定位方案。生产部署仍需提供同一地图版本下对齐的定位 TF。

先构建并生成 marker 模型：

```bash
cd /home/xiaomin/projects/edgeai_robot_ws
pixi run colcon build --packages-up-to edgeai_robot_bringup edgeai_target_perception --symlink-install
pixi run python src/edgeai_target_perception/test/fixtures/generate_aruco_marker.py
```

在每个终端中从仓库根目录加载 Pixi 和工作区环境，并使用同一 ROS Domain：

```bash
cd /home/xiaomin/projects/edgeai_robot_ws
pixi shell
source install/setup.bash
export EDGEAI_CONFIG_DIR="$PWD/config"
export ROS_DOMAIN_ID=96
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```

**终端 A — 双机器人 Gazebo：**

```bash
export GZ_SIM_RESOURCE_PATH="$PWD/src/edgeai_target_perception/test/fixtures:${GZ_SIM_RESOURCE_PATH:-}"
ros2 launch edgeai_robot_bringup sim_multi_robot.launch.py \
  robots:=robot_1,robot_2 \
  world:="$PWD/src/edgeai_target_perception/test/fixtures/aruco_two_robot_test_world.sdf" \
  headless:=true rviz:=false stack:=false config_dir:="$PWD/config"
```

**两个 TF 终端 — 测试专用 `map -> robot_N/odom` ground-truth：**

```bash
ros2 run tf2_ros static_transform_publisher \
  --x -1.5 --y -1.5 --frame-id map --child-frame-id robot_1/odom \
  --ros-args -p use_sim_time:=true
```

```bash
ros2 run tf2_ros static_transform_publisher \
  --x -1.5 --y -0.8 --frame-id map --child-frame-id robot_2/odom \
  --ros-args -p use_sim_time:=true
```

**状态终端 — 发布临时有效 Task 1 状态：**

```bash
ros2 topic pub -r 1 --qos-reliability reliable --qos-durability transient_local \
  /competition/state edgeai_robot_interfaces/msg/CompetitionState \
  "{run_id: run-p3-two-robot-20261025, map_id: test-two-robot-ground-truth, map_revision: 1, phase: 1, run_mode: 0, frozen: false, estop_active: false}"
```

**两个 detector 终端 — 分别启动机器人检测器（默认先用 RGB-D）：**

```bash
ros2 launch edgeai_target_perception target_detector.launch.py \
  robot_id:=robot_1 run_mode:=development config_dir:="$PWD/config" use_depth:=true
```

```bash
ros2 launch edgeai_target_perception target_detector.launch.py \
  robot_id:=robot_2 run_mode:=development config_dir:="$PWD/config" use_depth:=true
```

**融合终端 — 启动节点并验证输出：**

```bash
ros2 launch edgeai_target_perception target_fusion.launch.py \
  robot_ids:=robot_1,robot_2 run_mode:=development \
  config_dir:="$PWD/config" use_sim_time:=true
```

```bash
python src/edgeai_target_perception/test/fixtures/verify_two_robot_fusion.py \
  --mode verify --run-id run-p3-two-robot-20261025
```

验证器核对两路 RGB、CameraInfo、深度编码/尺寸/frame/同步时间，且每台机器人 observation 的
robot ID、run ID、Marker/T1 身份、光学 frame 和 RGB 时间戳正确。融合快照必须只有一个 T1、
包含两台机器人作为来源、`observation_count=2`，注册表 revision 至少为 2，融合位置误差小于
0.15 m；输出 target 时间戳必须等于一条输入 observation 时间戳。融合节点确实使用原始观测时间
查询 TF 的行为由 `test_target_fusion_node.py` 直接断言。由于本集成场景的 TF 是静态 ground-truth，
它不验证动态 TF 历史插值。

检查陈旧数据前，先 Ctrl+C 停止两个 detector，保留 Gazebo、状态发布器、ground-truth TF 和融合
节点；然后运行：

```bash
python src/edgeai_target_perception/test/fixtures/verify_two_robot_fusion.py \
  --mode stale --run-id run-p3-two-robot-20261025 --timeout-s 15
```

验证器等待旧注册表过期且观察流静默，注入一条超过配置过期门限的有效 T1 observation；目标必须
仍为空，revision 不得因这条陈旧消息改变。完成后停止融合、状态发布器、两个 TF 发布器和 Gazebo。

#### WSL2 首次实测结果（backing 修正前）

- 两台机器人均成功生成 RGB、CameraInfo 和深度数据：RGB `640x480 rgb8`、深度
  `640x480 32FC1`；对应 optical frame 正确，RGB 与深度时间戳一致。
- 深度模式下两台 detector 均检测到 Marker 1，但都按现有规则报告 `depth_inconsistent`。
  PnP 候选深度约 `0.860 m`，与深度样本的残差约 `0.101 m`，超过既有 `0.05 m` 容差；
  因此未发布 RGB-D observation。没有修改拒绝门限。
- 随后 RGB-only 检测成功：两台机器人都发布 run ID
  `run-p3-two-robot-20261025` 的 T1/Marker 1 observation，时间戳均为 `91972000000 ns`，
  与各自 RGB 输入一致。`/edge/targets` 只有一个 T1，来源为两台机器人，count=2、
  revision=3；位置约 `[-0.552, -1.148, 0.145] m`，相对测试 Marker 中心误差
  `0.1022 m`。
- 停止 detector 并等待样本过期后，陈旧 observation 未重建目标，revision 保持 `2465` 不变。
- 所有测试进程最终均已退出。关闭 Gazebo 时 launch 在 SIGINT 后等待 5 秒并升级至 SIGTERM；
  Gazebo 随后退出且无残留进程，ROS bridges 和 robot state publishers 正常退出。

#### RGB-D 深度偏差根因与修正后复测

首次 RGB-D 失败不是 RGB/深度错位或背景污染。两路 CameraInfo 的内参相同
（`fx=fy=554.3827`、`cx=320`、`cy=240`、畸变为零），RGB 与深度均为 `640x480`，
使用同一个 optical frame，配对时间戳差为零；PnP 内部九点采样均落在 marker 可见平面，
深度为 `0.9615–0.9625 m`。问题是 fixture 的白色底板为 `0.20x0.20 m`，比配置的黑色
ArUco 外沿 `0.15x0.15 m` 大；底板外轮廓干扰 marker 角点定位，使角点超出理论 marker 边界
约 8–10 px，PnP 因错误角点得到约 `0.860 m` 深度，而相机实际看到的 marker 平面约为
`0.9625 m`，残差约 `0.101 m`。

最小修正是在 `test/fixtures/generate_aruco_marker.py` 将独立测试模型的白色底板改为
`0.15x0.15 m`，与黑色 marker 外沿一致；未修改官方世界、接口、PnP 歧义逻辑或 `0.05 m`
深度一致性门限。重新生成后，两个相机实测角点都回到理论投影边缘附近：robot_1
`[[75,253],[75,168],[161,168],[161,253]]`，robot_2
`[[478,253],[478,168],[564,168],[564,253]]`。PnP 中心深度约 `0.9780 m`，九点预测深度约
`0.9771–0.9790 m`，对应原始深度仍为 `0.9615–0.9625 m`，残差约 `0.016 m`，低于原门限。
诊断命令为：

```bash
python src/edgeai_target_perception/test/fixtures/inspect_two_robot_rgbd.py
```

双机器人 `use_depth:=true` 复测通过。验证器确认两台机器人都发布 Marker 1 / T1 observation，
robot ID、run ID、RGB 时间戳和 optical frame 均正确；`/edge/targets` 只有一个 T1，
`observation_count=2`、来源为 robot_1 和 robot_2。该次融合位置为
`[-0.4443,-1.1491,0.1515] m`，与测试 marker ground truth 的误差为 `0.0059 m`。
复测运行 ID 为 `run-p3-depth-fixture-backing-01`；版本号随持续接收的实时观测递增，不作为
固定断言。测试验证器同时修正了等待条件：只有在两台相机元数据/图像及与 RGB 时间戳匹配的
observation 都到齐后才开始断言，避免启动时的订阅调度竞态。

## Phase 4 验证范围与限制

合成测试覆盖刚体坐标和协方差变换、时间戳 TF 查询、短时 TF 不可用重试、同机器人多帧去重、
跨机器人加权融合、空间离群点、未知/冲突身份、陈旧数据、run/map/reset 隔离和目标消息字段。
当前 TF 查询在 ROS 多线程执行器中有限阻塞；仿真老化和重试使用 ROS 时钟，暂停的仿真时间
也暂停这些时限。输入 pose covariance 来自检测器的保守高方差占位值，融合输出因此也保持较大
不确定度；这不是已标定的检测概率或地图不确定度模型。协方差传播采用姿态小角度块旋转近似，
不含 SLAM 地图间相关性。写入 `Target` 的协方差按Marker到目标平面姿态的yaw投影传播；尚未标定的
PnP方差仍保守保留，不能解释为测量精度保证。本集成验证使用已知测试世界 TF，不代表 P2 多机器人地图对齐已完成；
动态 TF 历史插值也尚未通过 Gazebo 集成场景验证。
