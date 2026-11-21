# 决赛运行与 Task 1 验证

本页记录 Zhang 模块的操作入口和仿真复现步骤。Task 1 使用 ADR-0001 的单机建图，robot_2 等待交接。系统管理器控制比赛阶段，Deng 的目标融合节点发布目标快照，操作员在地图和目标快照版本一致时定稿。

## 本地仿真

在仓库根目录安装锁定环境，然后构建和检查。

```bash
pixi install --locked
pixi run build
pixi run check-env
pixi run validate
pixi run test
```

用已有中期场地一键演练 Task 1、Task 2A 和封存。如果冻结清单缺少该场地要求的目标，演练保留 Task 1 结果并报告缺失目标。同一台电脑一次运行一轮仿真。`practice` 不传 `--arena` 时使用中期场地，并且会转发后续参数。

```bash
pixi run practice --domain 79 --runs-root runs/zhang/midterm
```

以下命令使用中期场地做两台机器人的 Task 2A 和 Task 2B 定位检查。它会测试阶段切换及证据封存，Task 2B 的指令解析和救援执行需用集成后的模块另行验证。

```bash
pixi run practice --domain 79 --runs-root runs/zhang/handover \
  --check-localization 1 --check-seed 7 --task2b stay
```

## 操作员

在已启动的系统上使用同一 ROS 域、发现范围和中间件。读取该次启动的 `<runs-root>/active_run.txt`，每条命令显式带上该运行 ID。先用 `status` 核对运行和阶段。下列示例以本地域 79 为例。

```bash
pixi shell
source install/setup.bash
export ROS_DOMAIN_ID=79 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
run_id=$(cat runs/zhang/manual/active_run.txt)
python tools/competition_operator.py status --run-id "$run_id"
```

要手动启动上述本地练习场，可先在另一终端使用相同环境和 ROS 域运行：

```bash
ros2 launch edgeai_system_manager task1.launch.py scenario:=midterm \
  run_id:=new runs_root:=runs/zhang/manual
```

收到比赛操作指令后，将 `status` 换为对应动作。常规顺序为 `start`、等待系统进入 HANDOVER、`finalize`、`task2a`、`freeze`、`task2b`、`finish`。中期场地的救援节点可以自动进入 FROZEN，此时从下一步继续。若要请求提前结束探索，用 `handover`，服务会检查当前阶段及停车确认。需停车时可显式发 `estop`。所有命令仍由系统管理器执行权限和阶段检查。不要让自动演练脚本与手动操作同时控制同一个运行。

`finish` 额外核对本机证据目录。使用自定义路径时传入与启动一致的 `--runs-root`，例如：

```bash
python tools/competition_operator.py finish --run-id "$run_id" --runs-root runs/zhang/manual
ros2 run edgeai_system_manager audit_run "runs/zhang/manual/$run_id" --finished
```

## 随机场地对照

固定 Git 提交、Pixi 锁、种子和命令。每个种子重复运行，记录失败轮次和结束原因。开发用评分采用仓库现有阈值。正式时间和得分以组织方确认的规则为准。

```bash
for seed in 7 3 11; do
  pixi run practice-arena --seed "$seed"
  pixi run practice --domain 79 --arena "runs/practice-arenas/seed-$seed" \
    --runs-root "runs/zhang/practice/seed-$seed"
done
```

- `*.rehearsal.json` 记录运行 ID、墙钟耗时、阶段结果和审计。补看和回起点的首次状态转换时间用仿真秒，缺失键表示未观察到该状态。
- `*.task1_score.json` 记录种子、目标误差、误检、地图覆盖、占据准确率。仅跑 Task 1 时，演练退出码同时要求评分和 Task 1 证据审计通过。整场封存需到 FINISHED。
- 复现漏检时加 `--camera-diagnostics`。它只用于本地诊断，会增加一条 RGB 订阅。报告会记录收到的帧数、最大帧时间间隔和检测节点发布的原始观测数。MCAP 同时保留原始目标观测与融合后的目标快照。对比分数和耗时的常规基准应采用同样的诊断设置。
- 每轮的 `system/component_versions.json`、`source_hashes.json`、启动日志和评分报告共同用于复现；运行数据放在 `runs/`。

## 已有对照结果

在 `4067f55` 上，种子 7、3、11 的 Task 1 墙钟分别为 369.1、317.8、272.8 秒，正确目标分别为 0/2、2/2、0/2。仅调整补看访问顺序的实验版本分别为 466.4、348.5、342.0 秒，正确目标为 0/2、0/2、2/2。六轮地图覆盖率与占据准确率均为 1.0。每个组合仅运行一次，路线排序版本未达到提速门槛。

四轮未得分的实验未留下检测节点发布的目标观测。冻结结果为有效的空快照。以录包位姿、练习场几何和估算视场作离线筛查，两块标记都曾进入可能可见范围；该筛查无法证明图像内容或检测器当时处理了相同帧。下一轮定位需同时检查 RGB 连续性、原始观测和融合快照。

基线的中期场地复跑完成了探索、地图冻结和双机重定位，但冻结清单为空。原脚本仍进入 Task 2A，按中期固定配置派发救援，日志器发现事件目标不在权威清单后触发急停。改进后的脚本在进入 Task 2A 前检查目标清单。遇到缺失的检测文件时，查验工具继续输出失败项和已经保存的地图。

## 双机评估边界

当前第二台机器人在 Task 1 停放，无法用已有结果估算双机提速。双机候选需要统一地图坐标系、明确每台 SLAM TF 的唯一发布方、分配互不阻塞的目标，并验证两台机器人都能从 SLAM 切换到 AMCL。与 Huang 和 Deng 在相同场地和计时设定下对比得分、地图质量、超时率、碰撞和交接失败率，再按团队约定决定是否修改 ADR-0001。真机离线建图需要实际激光、里程计、TF 和时间戳一致的录包。
