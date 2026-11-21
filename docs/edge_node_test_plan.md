# 边缘节点测试步骤（组织方的 GPU 容器）

组织方的测试边缘节点是一个多队共用的 GPU 容器（NVIDIA L4，24 GB），只能通过 SSH 登录，内容可能被清空。这份步骤用来在订到的时段里依次验证：我们的环境能在容器里运行；本机仿真的机器人和容器里的边缘节点之间能按 ADR-0004 通信；实际链路的延迟、流量和时钟偏差。模型推理的测试由 Task 2B 负责人另行安排，可以接在第 5 步之后。

登录地址、账号和密码只在组织方的邮件里，不要写进仓库（仓库是公开的）。下文用 `<边缘节点>` 代替 `root@地址`。

## 0. 订时段之前

- 在共享表格上订时段；用完停掉自己启动的所有进程（组织方要求）。
- 用密钥登录，省得每次输密码：`ssh-copy-id -i ~/.ssh/id_ed25519.pub <边缘节点>`（输入一次邮件里的密码）。容器被清空后要重做。
- 本机先 `pixi run build`，确认分开部署的本地模拟能通过（`tools/edge_link.sh --rmw zenoh ... -- python tools/midterm_rehearsal.py --split`，见 ADR-0004）。
- 不做系统级改动（不用 `apt`）：容器里的环境全部用 Pixi 装在用户目录下。

## 1. 登录后的检查（约 5 分钟）

```bash
ssh <边缘节点>
nvidia-smi                      # GPU 是否可用、显存是否被别的队占用
/root/check_gpu_python.sh       # 组织方提供的检查脚本
nproc; free -h; df -h ~         # CPU、内存、磁盘（Pixi 环境约需 5 GB）
curl -sI https://prefix.dev | head -1   # 能否上网下载 Pixi 包
date -u +%s.%N                  # 和本机同时执行，粗看时钟偏差
```

不能上网时，Pixi 环境要在本机打包后复制进去（`pixi-pack`），这一步要另约时间。

## 2. 在容器里装环境（约 15–30 分钟，取决于下载速度）

容器的 root 账号和 `/root` 由所有队伍共用（里面还有 R1 团队的 5G 系统），所以全部东西放在我们队自己的目录里：Pixi 本体、下载缓存、仓库和环境。不用会改 `~/.bashrc` 的安装脚本；用完删掉这个目录即可。

```bash
T=/root/edgeai_<队名>; mkdir -p $T/bin $T/cache && cd $T
V=v0.83.0; F=pixi-x86_64-unknown-linux-musl.tar.gz
curl -fsSL -O https://github.com/prefix-dev/pixi/releases/download/$V/$F
curl -fsSL -O https://github.com/prefix-dev/pixi/releases/download/$V/$F.sha256
sha256sum -c $F.sha256 && tar -xzf $F -C bin && rm $F $F.sha256
echo "export PATH=$T/bin:\$PATH PIXI_CACHE_DIR=$T/cache RATTLER_CACHE_DIR=$T/cache" > env.sh
source env.sh
git clone https://github.com/hzc333/edgeai_robot_ws_final.git repo && cd repo
# 下载和构建放进 tmux，SSH 断开也不中断（tmux attach -t edgeai 查看）
tmux new-session -d -s edgeai "source $T/env.sh && pixi install && pixi run build && pixi run practice-arena --seed 3"
```

之后每次登录先 `source /root/edgeai_<队名>/env.sh`，在 `repo` 目录里工作。`pixi run practice-arena --seed 3` 与本机同一个种子，场地文件两边一致。

## 3. 打通 Zenoh（约 5 分钟）

容器只开放 SSH，所以用 SSH 端口转发把容器的 Zenoh 路由器端口 7447 映射到本机。本机自己的路由器也监听 7447，转发要用另一个本地端口（这里 17447）。

```bash
# 容器里（一个 SSH 会话，保持运行）
source /root/edgeai_<队名>/env.sh && cd /root/edgeai_<队名>/repo
pixi run bash -c 'source install/setup.bash && ros2 run rmw_zenoh_cpp rmw_zenohd'

# 本机终端 1：端口转发（保持运行）
ssh -N -L 17447:localhost:7447 <边缘节点>

# 本机终端 2：本机路由器，连到转发端口（保持运行）
pixi run bash -c 'source install/setup.bash && \
  ZENOH_CONFIG_OVERRIDE='"'"'connect/endpoints=["tcp/localhost:17447"]'"'"' \
  ros2 run rmw_zenoh_cpp rmw_zenohd'
```

两边所有 ROS 进程都设 `RMW_IMPLEMENTATION=rmw_zenoh_cpp` 和同一个 `ROS_DOMAIN_ID`（演练脚本用 77）。先做冒烟测试：

```bash
# 容器里
pixi run bash -c 'source install/setup.bash && export RMW_IMPLEMENTATION=rmw_zenoh_cpp ROS_DOMAIN_ID=77 && \
  ros2 topic pub -r 2 /ping std_msgs/msg/String "{data: edge}"'
# 本机
pixi run bash -c 'source install/setup.bash && export RMW_IMPLEMENTATION=rmw_zenoh_cpp ROS_DOMAIN_ID=77 && \
  ros2 topic echo --once /ping std_msgs/msg/String'
```

## 4. 跨机器跑 Task 1（约 15 分钟）

边缘节点在容器里，机器人和 Gazebo 在本机。用练习场地只跑 Task 1：中期救援的碰撞裁判直接订阅 Gazebo 的物理接触（gz-transport），这条数据不走 Zenoh，过不了 SSH 转发。

```bash
# 容器里：只启动边缘一侧，打印本机要执行的命令
pixi run bash -c 'source install/setup.bash && export RMW_IMPLEMENTATION=rmw_zenoh_cpp && \
  python tools/midterm_rehearsal.py --arena runs/practice-arenas/seed-3 --robots-elsewhere'

# 本机：15 分钟内执行上面打印的 command（在仓库根目录）
pixi run bash -c 'source install/setup.bash && export RMW_IMPLEMENTATION=rmw_zenoh_cpp ROS_DOMAIN_ID=77 && \
  <打印的 command>'
```

运行期间在容器里另开一个会话测流量和接口（同样的环境变量）：

```bash
python tools/edge_traffic.py --seconds 60 --out runs/edge_traffic.json
python tools/robot_preflight.py --robot robot_1 --use-sim-time
```

## 5. 记录

- 本机到容器的往返延迟和抖动：`ping -c 50 <地址>`（容器不回 ping 时看 SSH 冒烟测试的延迟）。
- `edge_traffic.py` 的两个方向总流量和最大的几个话题。
- Task 1 的结果（`runs/practice/<run_id>.rehearsal.json`、`task1_score.json`），出错时两边的启动日志。
- 时钟：容器能否运行 chrony 或使用宿主机时间；`date` 的偏差。
- 实机部署要用的结论：机器人能否直接连容器的 7447，还是必须经 SSH 转发。

## 6. 结束（约 5 分钟）

```bash
# 本机：把容器里的结果取回（容器可能被清空）
scp -r <边缘节点>:/root/edgeai_<队名>/repo/runs/practice runs/edge-node-$(date +%Y%m%d)
```

停掉容器里自己启动的进程（路由器、演练脚本、tmux 会话；按 Ctrl+C、`tmux kill-session -t edgeai` 或 `kill <进程号>`，不要按名字批量结束别的队的进程），关掉本机的端口转发和路由器，在共享表格上结束时段。不再测试时删掉整个 `/root/edgeai_<队名>`。
