# Marker坐标约定与救援区几何

状态：约定 v0.2；修改需感知、救援协调和接口负责人共同审核

本文档定义从图像中的ArUco Marker到`map`坐标系目标位姿、再到救援区位姿的全部几何约定。`TargetObservation`、`Target`、`targets.json`、救援区生成和救援验证都必须遵循同一约定，不得依赖OpenCV版本的默认行为。

## 1. OpenCV Marker坐标系（检测器内部）

OpenCV 4.6的`estimatePoseSingleMarkers`与4.7+的`ArucoDetector + solvePnP`默认角点约定不同。检测器必须显式传入以下物体角点，并使用`SOLVEPNP_IPPE_SQUARE`，不能依赖库默认值：

```text
L = marker_size_m（Marker黑色边框外沿边长）
角点顺序与detectMarkers返回顺序一致：左上、右上、右下、左下
左上 (-L/2, +L/2, 0)
右上 (+L/2, +L/2, 0)
右下 (+L/2, -L/2, 0)
左下 (-L/2, -L/2, 0)
```

由此得到Marker坐标系`M`：原点在Marker中心，`x_M`沿Marker上边缘指向右，`y_M`指向Marker上方，`z_M`垂直于Marker表面指向观察者（Marker正面法线）。

`TargetObservation.marker_pose`中的姿态就是`M`在相机光学坐标系（`header.frame_id`为`robot_N/camera_optical_frame`）中的姿态，不做任何轴变换。

## 2. 目标坐标系（融合、导出和救援使用）

融合节点在观测时间戳把`M`变换到`map`后，构造目标坐标系`T`：

```text
x_T = z_M   （Marker正面法线，从Marker指向场地内部）
y_T = x_M
z_T = y_M   （Marker竖直安装时为竖直向上）
```

即在`M`坐标中`R_M_T = [[0,1,0],[0,0,1],[1,0,0]]`（三列依次是`x_T`、`y_T`、`z_T`）。该变换右手且行列式为1。

当前假设Marker竖直安装在墙面或立柱上。进入`map`后，`T`进一步投影为平面位姿：

- `position`：Marker中心在`map`中的三维坐标，保留`z`作为安装高度。
- `yaw = atan2(x_T.y, x_T.x)`，归一化到`[-pi, pi)`。
- `orientation`：只含yaw的四元数`(0, 0, sin(yaw/2), cos(yaw/2))`；滚转和俯仰被丢弃。
- 若`x_T`与`map`竖直轴夹角小于45°（Marker近似水平安装），yaw不可定义，融合节点必须拒绝该观测，并在拒绝统计中记录原因`marker_not_vertical`；出现该情况说明安装假设不成立，应在TBD登记表中更新安装方式问题。

`Target.marker_pose`和`targets.json`中的`marker_pose`都使用该平面目标位姿；`targets.json`另写`marker_yaw_rad`，必须与四元数一致（容差1e-3 rad）。规则5.4表格中的yaw即该值。

## 3. 救援区位姿

救援区只能由带版本的几何规则从目标位姿计算。官方规则公布前，`official.rescue.geometry_version`为`null`：

- 比赛模式：不得生成救援区，`Target.rescue_poses`和`targets.json`中的`rescue_zones`为空。
- 开发/演练模式：使用`config/development_scenarios.yaml`中的`rescue_geometry`，并把其`version`写入`targets.json.rescue_geometry`。

开发几何`development-v1`的定义（偏移均在目标坐标系`T`的水平面内）：

```text
普通目标：zone_0 = p + forward_offset * x_T
协同目标：zone_i = p + forward_offset * x_T + lateral_offsets[i] * y_T，i = 0, 1
救援区z = 0
救援区yaw = normalize(marker_yaw + pi)   # 面向Marker，便于相机持续观测
```

其中`p`是Marker中心在`map`中的水平位置。协同目标的区编号`i`就是`RescueTask.goal_poses`、`ExecuteRescueTask.rescue_zone_index`和`RescueEvent.rescue_zone_indices`中的编号。

规则3.3说明救援完成不要求特定朝向，因此救援区yaw只用于导航，不参与占用判定。占用判定只比较机器人`base_footprint`在`map`中的水平位置与救援区中心的距离是否不超过`zone_tolerance_m`。

## 4. 示例

`examples/interfaces/task1_targets.example.json`：

| 目标 | Marker位置 | marker_yaw | 救援区 |
|---|---|---:|---|
| T1（普通） | (1.30, 2.10) | +pi/2 | zone_0 (1.30, 2.60)，yaw -pi/2 |
| TC1（协同） | (3.00, 0.40) | 0 | zone_0 (3.50, 0.10)，zone_1 (3.50, 0.70)，yaw -pi |

`tools/validate_project_contracts.py`会按本节公式重新计算样例中的救援区并比较。

## 5. 待组织方确认

- Marker安装方式（竖直/水平）、安装高度和朝向。
- 官方救援区几何是否以Marker中心、法线方向定义；正式规则公布后，以新版本几何替换`development-v1`，本文档同步升级版本。
