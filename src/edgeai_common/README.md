# edgeai_common

比赛系统各节点共用的Python工具，不包含任何运行节点。

## 文件清单

| 文件 | 作用 |
|---|---|
| `edgeai_common/config.py` | 查找`config/`目录，按运行模式解析`competition_parameters.yaml`，读取`interface_runtime.yaml` |
| `edgeai_common/qos.py` | 按运行时契约生成`QoSProfile` |
| `edgeai_common/frames.py` | 机器人ID校验、带前缀的帧名和话题名 |
| `edgeai_common/__init__.py` | 包入口 |
| `test/test_config.py` | 解析规则测试：官方值优先、回退记录、比赛模式拒绝缺失参数、开发模式允许缺失、未知模式报错，以及对仓库真实配置的解析 |
| `test/test_qos_and_frames.py` | QoS映射、帧名和非法机器人ID测试 |
| `test/test_copyright.py`、`test_flake8.py`、`test_pep257.py` | 版权头、代码风格和文档字符串检查 |
| `setup.py`、`setup.cfg`、`package.xml`、`resource/` | `ament_python`包配置 |

## 模块

- `edgeai_common.config`：查找仓库根目录`config/`，并按运行模式解析`competition_parameters.yaml`。
  - 查找顺序：显式参数`config_dir` → 环境变量`EDGEAI_CONFIG_DIR` → 当前目录及其上级中的`config/`。
  - `resolve_parameters(run_mode)`返回`ResolvedParameters`：`values`（解析后的参数树）、`fallbacks`（回退到`development`的参数路径）、`missing_for_motion`（仍未设置的必需参数）、`sha256`（参数集合哈希，可写入`CompetitionState.parameters_sha256`）。
  - 规则与`competition_parameters.yaml`头部注释一致：`competition`只用`official`，必需参数缺失时抛出`MissingOfficialParameters`；`rehearsal`允许回退但必需参数必须有值；`development`允许必需参数缺失。空映射（如未公布的`aruco.id_mapping: {}`）视为未设置。
- `edgeai_common.qos`：`qos_from_contract(contract, name)`按`interface_runtime.yaml`的`qos`段生成`rclpy`的`QoSProfile`。
- `edgeai_common.frames`：`frame('robot_2', 'odom') -> 'robot_2/odom'`、`topic('robot_2', 'scan') -> '/robot_2/scan'`，以及机器人ID校验（ADR-0002）。

## 使用约定

节点不要自己读YAML，也不要自己实现回退逻辑：由启动文件调用`resolve_parameters`，把需要的值作为ROS参数传给节点。统一的参数解析服务完成后（P2的`system_manager`生成`system/parameters.yaml`），只需修改启动文件。

## 已知限制

- 参数解析目前由各启动文件调用；P2的`system_manager`生成统一的`system/parameters.yaml`后，启动文件应改为读取该文件。
- 解析结果中，`aruco.id_mapping`、`localization.start_poses`、`localization.task2b_start_poses`作为整体处理（不逐项合并）。

## 测试

```bash
colcon build --symlink-install --packages-select edgeai_common
colcon test --packages-select edgeai_common && colcon test-result --verbose
```

2026-10-23：16项测试全部通过。
