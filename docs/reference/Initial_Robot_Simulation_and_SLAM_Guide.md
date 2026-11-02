From environment setup and robot construction to 2D mapping and map management.

Ubuntu 24.04 • ROS 2 Jazzy (Pixi) • Gazebo Harmonic • SLAM Toolbox

Public code repository: https://github.com/dongjw-hit/edgeai_robot_ws

This consolidated guide is organized into three progressive parts. The public GitHub baseline is the runnable source of truth; the construction walk through explains its implementation without requiring source files to be recreated by hand.

# Contents

## Part I — Getting Started

Install, clone, build and verify the simulator

> 1.1 Overview
>
> 1.2 Prerequisites
>
> 1.3 Install the host tools
>
> 1.4 Clone and build the workspace
>
> 1.5 Launch the simulation
>
> 1.6 Verify the ROS interfaces
>
> 1.7 Move slowly and stop explicitly

## Part II — Robot Simulation Construction

Explore the implementation and source files

> 2.1 Repository orientation and working convention
>
> 2.2 Project and ROS 2 package construction
>
> 2.3 Constructing the Gazebo world
>
> 2.4 Constructing the realistic robot (URDF/Xacro)
>
> 2.5 Adding motion and joint-state plugins
>
> 2.6 Launch assembly and ROS–Gazebo bridge
>
> 2.7 Integrating the 2D LiDAR
>
> 2.8 Integrating the RGB-D camera and its frames
>
> 2.9 RViz configuration and start-up entry point
>
> 2.10 Rebuild and validate the integrated source

## Part III — SLAM Mapping and Map Management

Create, save and reload a 2D occupancy map

> 3.1 Overview
>
> 3.2 Prerequisites and terminal layout
>
> 3.3 SLAM inputs and system architecture
>
> 3.4 Project SLAM Toolbox configuration
>
> 3.5 Start SLAM Toolbox and display the map
>
> 3.6 Build a complete occupancy map
>
> 3.7 Save the occupancy map
>
> 3.8 Reload the map without SLAM

Word navigation: headings can be expanded in the Navigation Pane. Section labels correspond to the three original tutorials.

# Part I — Getting Started

Environment setup and first simulation run

## 1.1 Overview

Set up and run the baseline robot simulation for the KTH Edge AI & Robotics Challenge. The repository provides a realistic differential-drive robot, a Gazebo arena, a 2D LiDAR, an RGB-D camera, ROS 2 interfaces, and RViz visualization.

By the end of this tutorial, you will have installed the project environment, built the workspace, launched the simulation, checked sensor data, and tested a slow movement followed by an explicit stop. The robot model and launch configuration are already provided; no URDF or launch files need to be written.

The baseline has passed a clean-clone, dependency-installation, build, and simulation test. This tutorial covers simulation and basic interfaces. AMCL localization and autonomous navigation are outside its validated scope.

## 1.2 Prerequisites

| **Component**        | **Required setup**                                        |
|----------------------|-----------------------------------------------------------|
| **Operating system** | Ubuntu 24.04 LTS, x86_64                                  |
| **Simulator**        | Gazebo Harmonic / Gazebo Sim 8.x on the host              |
| **ROS environment**  | ROS 2 Jazzy, managed by the project’s Pixi environment    |
| **Tools and access** | Git, Pixi on PATH, internet access, and repository access |
| **Graphics**         | A graphical desktop capable of running Gazebo and RViz    |

### 1.2.1 Check the host environment

```bash
lsb_release -ds
uname -m
command -v pixi
pixi --version
gz sim --version
```

Expect Ubuntu 24.04, x86_64, an available Pixi executable, and Gazebo Sim 8.x. If a tool is missing, install it on the next page.

Use a fresh Bash terminal. Avoid mixing this project with another active Pixi or Conda environment or a sourced system ROS installation. ROS 2 is supplied by Pixi; sourcing /opt/ros/jazzy/setup.bash is not required.

## 1.3 Install the host tools

### 1.3.1 Basic Ubuntu packages

```bash
sudo apt update
sudo apt install curl git lsb-release gnupg build-essential
```

### 1.3.2 Pixi

If Pixi is not installed, run the installer and reload your Bash configuration:

```bash
curl -fsSL https://pixi.sh/install.sh | sh
source ~/.bashrc
pixi --version
```

### 1.3.3 Gazebo Harmonic

Skip this step if gz sim --version already reports Sim 8.x. On Ubuntu 24.04, add the Gazebo package repository and install Harmonic:

```bash
sudo curl https://packages.osrfoundation.org/gazebo.gpg \
--output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
GZ_ARCH=$(dpkg --print-architecture)
GZ_CODENAME=$(lsb_release -cs)
GZ_KEY=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
GZ_REPO=https://packages.osrfoundation.org/gazebo/ubuntu-stable
echo "deb [arch=$GZ_ARCH signed-by=$GZ_KEY] $GZ_REPO $GZ_CODENAME main" | \
sudo tee /etc/apt/sources.list.d/gazebo-stable.list
sudo apt update
sudo apt install gz-harmonic
gz sim --version
```

For Ubuntu 24.04, the detected codename must be noble. Do not substitute package sources for another Ubuntu release.

### 1.3.4 Check graphics before continuing

```bash
gz sim shapes.sdf
```

Confirm that the example scene opens and can run, then close it before launching the challenge simulation. The LiDAR and RGB-D sensors use rendering, so a working ROS command line alone does not verify graphics support.

Official Pixi installation guide

Official Gazebo Harmonic installation guide

## 1.4 Clone and build the workspace

Choose a working directory, then run the README commands below. If an edgeai_robot_ws directory already exists, use another parent directory to keep your existing work intact.

```bash
git clone https://github.com/dongjw-hit/edgeai_robot_ws.git
cd edgeai_robot_ws
pixi install
pixi run colcon build --symlink-install
```

The baseline simulation is publicly available on GitHub. You can clone the repository without requesting access or configuring GitHub authentication.

pixi install creates the project environment from pixi.toml and pixi.lock. The build installs the supplied ROS package and its resources. Continue only after the build completes without errors. Keep the manifest and lockfile supplied with the repository.

## 1.5 Launch the simulation

In terminal A, from the repository root, run the single simulation entry point:

```bash
./start_sim.sh
```

The script locates the workspace, activates Pixi, sources install/setup.bash, launches the realistic robot simulation, and opens the saved RViz configuration. Keep terminal A open and run only one instance of this simulation.

### 1.5.1 What you should see

Gazebo shows the arena, walls and obstacles, and the realistic robot with its wheels and sensors. Ensure that simulation time is advancing; use the play control if the simulation is paused.

RViz opens with the supplied realistic_robot.rviz configuration. The fixed frame is odom. Check the robot model, transforms, LiDAR scan, RGB image, depth image, and point cloud. Sensors should reflect the surrounding scene once startup has completed.

Gazebo displays simulated physical motion; RViz displays ROS data and estimated transforms. Their roles differ, particularly after a collision. A map frame is not required at this stage. The startup script does not start SLAM automatically.

### 1.5.2 Prepare terminal B for ROS commands

Open a second terminal. Replace the path below with your actual clone location:

```bash
cd /path/to/edgeai_robot_ws
pixi shell
```

Wait for the Pixi shell prompt, then run:

```bash
source install/setup.bash
```

Repeat this setup in each additional terminal used for ROS checks or control.

## 1.6 Verify the ROS interfaces

Keep the simulation running in terminal A. Run these checks in the prepared terminal B:

```bash
ros2 topic list
ros2 topic echo /odom --once
ros2 topic echo /clock --once
```

| **Topic**               | **Expected ROS message type** |
|-------------------------|-------------------------------|
| **/cmd_vel**            | geometry_msgs/msg/Twist       |
| **/odom**               | nav_msgs/msg/Odometry         |
| **/tf and /tf_static**  | tf2_msgs/msg/TFMessage        |
| **/joint_states**       | sensor_msgs/msg/JointState    |
| **/scan**               | sensor_msgs/msg/LaserScan     |
| **/camera/image**       | sensor_msgs/msg/Image         |
| **/camera/depth_image** | sensor_msgs/msg/Image         |
| **/camera/camera_info** | sensor_msgs/msg/CameraInfo    |
| **/camera/points**      | sensor_msgs/msg/PointCloud2   |
| **/clock**              | rosgraph_msgs/msg/Clock       |

### 1.6.1 Confirm sensor messages

```bash
ros2 topic type /scan
ros2 topic echo /scan --once --qos-reliability best_effort
ros2 topic echo /camera/image --once --field header \
--qos-reliability best_effort
ros2 topic echo /camera/depth_image --once --field encoding \
--qos-reliability best_effort
ros2 topic echo /camera/camera_info --once \
--qos-reliability best_effort
ros2 topic echo /camera/points --once --field header \
--qos-reliability best_effort
```

Expect scan ranges and camera messages with populated headers or fields. Some LiDAR ranges may be infinite where no surface is detected. Listing a topic alone does not prove that messages are arriving.

### 1.6.2 Check the robot transform

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
```

Expect a transform to be reported. This command continues running; press Ctrl+C after inspection. If a one-message sensor check waits indefinitely, cancel it and use the troubleshooting steps on the last page.

## 1.7 Move slowly and stop explicitly

Use this test only in the simulation. Check Gazebo for clear space ahead, keep the test short, and be ready to stop before reaching a wall or obstacle. Do not run another velocity publisher at the same time.

### 1.7.1 Send a slow forward command

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
'{linear: {x: 0.1}, angular: {z: 0.0}}'
```

The command requests 0.1 m/s forward motion and zero angular velocity. Watch Gazebo and RViz while the robot moves briefly through free space.

### 1.7.2 Stop the robot

Press Ctrl+C in terminal B to stop the forward-command publisher. Then immediately send an explicit zero-velocity command:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
'{linear: {x: 0.0}, angular: {z: 0.0}}'
```

Confirm that the robot stops in Gazebo. Ending the publisher alone must not be treated as a stop command. If the robot does not stop, end any other velocity publishers, send zero again, and stop the simulation from terminal A if needed.

### 1.7.3 Understand the odometry limitation

In the reported tests, normal straight driving stayed synchronized between Gazebo and RViz, and the explicit zero command stopped the robot. During contact with a wall or obstacle, the robot stopped physically in Gazebo while /odom continued changing and RViz continued showing motion.

The current odometry comes from Gazebo’s DiffDrive plugin. Treat it as an estimate, not collision-corrected ground truth: wheel motion under contact or slip can diverge from actual displacement. This collision-related drift remains unresolved. Stop when blocked and avoid sustained contact during later mapping work. SLAM does not guarantee that this behavior is corrected.

### 1.7.4 Close the session

After sending zero velocity, press Ctrl+C in terminal A to stop the launched session. Wait for shutdown before starting another instance.

### 1.7.5 Completion check

The workspace installs and builds without errors.

Gazebo and RViz open with the realistic robot.

Odometry, clock, LiDAR, and camera messages are available.

The robot moves briefly in clear space and stops after the zero command.

# Part II — How the Robot Simulation Is Built

Source-guided architecture and construction of the realistic robot

## Construction roadmap

| **Stage** | **What is constructed**               | **Main source file**                     |
|-----------|---------------------------------------|------------------------------------------|
| **1**     | Project / ROS 2 package               | pixi.toml; CMakeLists.txt; package.xml   |
| **2**     | Gazebo arena and sensors system       | worlds/empty.sdf                         |
| **3**     | Robot geometry and joints             | urdf/edgeai_robot_real.urdf.xacro        |
| **4**     | Motion plugins and odometry           | same Xacro                               |
| **5**     | Launch, spawn and ROS–Gazebo bridge   | launch/simulation_real.launch.py         |
| **6**     | LiDAR, camera and transforms          | Xacro + launch                           |
| **7**     | Visualization and one-command startup | rviz/realistic_robot.rviz + start_sim.sh |

## 2.1 Repository orientation and working convention

The GitHub repository is already cloned, installed and built in Tutorial 0. Here the task is to inspect and understand how the source files are assembled into a running simulation. The sequence mirrors the original complete construction guide: create the environment, define the world, construct the robot, attach plugins and sensors, bridge messages, and verify the integrated system.

```text
edgeai_robot_ws/
pixi.toml / pixi.lock
start_sim.sh
src/edgeai_robot_description/
CMakeLists.txt / package.xml
worlds/empty.sdf
urdf/edgeai_robot_real.urdf.xacro
launch/simulation_real.launch.py
rviz/realistic_robot.rviz
config/slam_toolbox.yaml
maps/edgeai_arena.yaml (+ image)
```

### 2.1.1 Terminal A — Start the existing baseline

In a regular Ubuntu terminal (no need to enter Pixi first):

```bash
cd ~/edgeai_robot_ws
./start_sim.sh
```

### 2.1.2 Terminal B — Inspect source and ROS 2 interfaces

Open another terminal. Enter the same Pixi environment and source the built workspace:

```bash
cd ~/edgeai_robot_ws
pixi shell
source install/setup.bash
```

File paths Examples assume the repository lives at ~/edgeai_robot_ws. If you cloned elsewhere, substitute your own path. Keep Terminal A running for live checks; commands that inspect source can be run without a live simulation.

## 2.2 Project and ROS 2 package construction

### 2.2.1 The dependency environment

pixi.toml specifies the ROS 2 Jazzy environment and associated development dependencies; pixi.lock fixes the resolved dependency set. The current baseline relies on Pixi/RoboStack and Gazebo Harmonic rather than a separately sourced system ROS installation.

```bash
sed -n '1,95p' pixi.toml
pixi run ros2 pkg list | grep -E 'ros_gz|robot_state_publisher|xacro'
```

Expected: dependencies for ros_gz, xacro and ROS 2 tools are available in the Pixi environment. Do not overwrite the repository manifest with the shorter historical example from the original construction notes.

### 2.2.2 Package metadata and resource installation

The ament_cmake package edgeai_robot_description is described by package.xml. CMakeLists.txt installs launch, urdf, worlds, rviz, config and maps into the package share directory. That installation step is essential because launch files locate resources through the ROS package index.

```bash
cat src/edgeai_robot_description/CMakeLists.txt
ros2 pkg prefix edgeai_robot_description
```

Expected: the package prefix points into the current workspace install directory. The repository no longer requires a separate empty meshes/ directory for installation.

## 2.3 Constructing the Gazebo world

The arena is specified in src/edgeai_robot_description/worlds/empty.sdf. Despite the filename, it is not empty: it contains the ground plane, four walls forming roughly a 5 × 5 m usable interior, and three rectangular obstacles. The SDF world is named empty because the launch and spawn operations refer to that world name.

| **SDF component**                    | **Purpose**                       |
|--------------------------------------|-----------------------------------|
| **Physics system**                   | Gravity and collision simulation  |
| **UserCommands / SceneBroadcaster**  | Entity creation and GUI updates   |
| **Sensors system using Ogre2**       | Rendering for GPU LiDAR and RGB-D |
| **Ground, wall and obstacle models** | Collision and visual geometry     |

Representative SDF structure (abridged; inspect the file for the exact models and poses):

```xml
<world name="empty">
<plugin filename="gz-sim-physics-system"
name="gz::sim::systems::Physics"/>
<plugin filename="gz-sim-sensors-system"
name="gz::sim::systems::Sensors">
<render_engine>ogre2</render_engine>
</plugin>
<!-- ground_plane, four walls, three obstacles -->
</world>
grep -nE 'world name|model name|gz-sim-sensors-system' \
src/edgeai_robot_description/worlds/empty.sdf
```

Expected: named world, GPU sensors system, ground and obstacle models. When simulation starts, Gazebo should show four outer walls and three internal obstacles. The robot spawns near (-1.5, -1.5), facing +x.

## 2.4 Constructing the realistic robot (URDF/Xacro)

The primary model source is src/edgeai_robot_description/urdf/edgeai_robot_real.urdf.xacro. Xacro expresses repetitive wheel and caster geometry through reusable macros, then expands to URDF. The body comprises a lower chassis, an upper plate, two front drive wheels and two passive rear casters, each with swivel and rolling joints.

| **Component**      | **Nominal configuration**                     |
|--------------------|-----------------------------------------------|
| **Lower chassis**  | 0.300 × 0.265 × 0.070 m                       |
| **Upper plate**    | 0.375 × 0.265 × 0.003 m                       |
| **Driven wheels**  | radius 0.049 m; separation 0.300 m            |
| **Rear casters**   | wheel radius 0.025 m, passive swivel and roll |
| **Base reference** | base_footprint → base_link: z = 0.049 m       |

### 2.4.1 Links, joints and macros

The Xacro defines base_footprint, base_link, drive-wheel links, caster forks and wheels, then sensor links. Left and right drive wheel axes use +y in their joint definitions; the positive wheel rotation convention drives the robot forward. The robot coordinate system uses x forward, y left, z upward.

```bash
grep -nE 'xacro:macro name=|wheel_radius|wheel_separation|base_footprint_joint|drive_wheel|caster' \
src/edgeai_robot_description/urdf/edgeai_robot_real.urdf.xacro
```

### 2.4.2 Expand and validate Xacro

```bash
ros2 run xacro xacro \
src/edgeai_robot_description/urdf/edgeai_robot_real.urdf.xacro \
-o /tmp/edgeai_robot_real.urdf
gz sdf -p /tmp/edgeai_robot_real.urdf > /tmp/edgeai_robot_real.sdf
```

Expected: both conversions finish without XML parsing errors or invalid link/joint definitions. Gazebo may merge fixed links during SDF conversion; TF is still published from the original robot description.

## 2.5 Adding motion and joint-state plugins

The same Xacro contains Gazebo plugins. DiffDrive connects left_wheel_joint and right_wheel_joint to velocity input and publishes odometry. JointStatePublisher exposes the active drive-wheel and caster joint states. No separate hand-written simple model is required.

```xml
<plugin filename="gz-sim-diff-drive-system"
name="gz::sim::systems::DiffDrive">
<left_joint>left_wheel_joint</left_joint>
<right_joint>right_wheel_joint</right_joint>
<wheel_separation>${wheel_separation}</wheel_separation>
<wheel_radius>${wheel_radius}</wheel_radius>
<topic>/cmd_vel</topic>
<odom_topic>/odom</odom_topic>
<frame_id>odom</frame_id>
<child_frame_id>base_footprint</child_frame_id>
</plugin>
```

A simplified differential-drive model uses v = r(ωR + ωL)/2 and yaw rate Ω = r(ωR − ωL)/b, where positive wheel speeds move the robot forward. The real simulation can deviate under collision or wheel slip.

```bash
ros2 topic type /cmd_vel
ros2 topic echo /odom --once
ros2 topic echo /joint_states --once
```

Expected: Twist, Odometry and JointState messages respectively. The six articulated joints include two drive wheels and four caster joints.

## 2.6 Launch assembly and ROS–Gazebo bridge

The integration file src/edgeai_robot_description/launch/simulation_real.launch.py locates the installed package resources, processes Xacro, launches Gazebo, publishes the robot description, spawns edgeai_robot_real, configures bridges and adds a camera point-cloud compatibility transform.

| **ROS 2 interface**                     | **Direction** | **Function**                |
|-----------------------------------------|---------------|-----------------------------|
| **/cmd_vel**                            | ROS → Gazebo  | Drive command               |
| **/odom; /tf; /joint_states**           | Gazebo → ROS  | Motion and articulation     |
| **/scan**                               | Gazebo → ROS  | LaserScan                   |
| **/camera/image; /camera/depth_image**  | Gazebo → ROS  | RGB and depth images        |
| **/camera/camera_info; /camera/points** | Gazebo → ROS  | Calibration and point cloud |
| **/clock**                              | Gazebo → ROS  | Simulation clock            |

Bridge entries use the ros_gz_bridge parameter_bridge syntax. The following is an abbreviated excerpt from the intended configuration:

```text
'/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist
/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry
/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan
/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock
grep -nE 'robot_description|empty.sdf|parameter_bridge|/clock|/scan|/camera/' \
src/edgeai_robot_description/launch/simulation_real.launch.py
ros2 topic list
```

Expected: the launch contains the listed interfaces; they become visible as ROS topics when Terminal A is running. The presence of /clock does not by itself prove that every node uses simulated time—inspect a node’s use_sim_time parameter when timestamps matter.

## 2.7 Integrating the 2D LiDAR

The LiDAR is attached through lidar_support_link and lidar_link in the Xacro. The simulated sensor is gpu_lidar, published on /scan, nominally 10 Hz, 360 angular samples and a valid range from 0.15 to 12 m. The lidar_link origin is approximately \[0.100, 0.080, 0.155\] m relative to base_footprint.

```bash
grep -nE 'lidar_support_link|lidar_link|gpu_lidar|<topic>/scan' \
src/edgeai_robot_description/urdf/edgeai_robot_real.urdf.xacro
ros2 topic echo /scan --once --qos-reliability best_effort
ros2 run tf2_ros tf2_echo base_footprint lidar_link
```

Expected: frame_id lidar_link, around 360 ranges, and the nominal offset above; press Ctrl+C after observing tf2_echo. An observed ROS message rate lower than 10 Hz can occur if Gazebo runs slower than real time.

## 2.8 Integrating the RGB-D camera and its frames

The Xacro attaches camera_link to the body, defines camera_optical_frame with the standard optical-axis convention, and configures the RGB-D sensor. The intended output is 640 × 480 RGB/depth with a nominal 30 Hz update rate. ROS topics include /camera/image, /camera/depth_image, /camera/camera_info and /camera/points.

```bash
ros2 topic list | grep '^/camera/'
ros2 topic echo /camera/image --once --field header --qos-reliability best_effort
ros2 topic echo /camera/points --once --field header --qos-reliability best_effort
ros2 run tf2_ros tf2_echo base_footprint camera_link
```

Expected: camera_link near \[0.175, 0, 0.100\] m from base_footprint, and four camera topics. Camera optical axes are x right, y down, z forward, which differs from the body frame.

Point-cloud frame compatibility In the tested baseline, /camera/points can use the Gazebo-scoped frame edgeai_robot_real/base_footprint/rgbd_camera. The launch adds a zero-translation, zero-rotation static transform from camera_link to that observed frame. This is a model-specific compatibility measure, not a general rule for every RGB-D sensor. Inspect header.frame_id and verify floor/wall orientation before changing transforms.

```bash
ros2 run tf2_ros tf2_echo camera_link \
edgeai_robot_real/base_footprint/rgbd_camera
```

## 2.9 RViz configuration and start-up entry point

The source file src/edgeai_robot_description/rviz/realistic_robot.rviz stores the visualizer configuration. It is not a ROS simulation itself: RViz displays robot-description/TF and sensor topics. The current entry point, start_sim.sh, resolves the repository location, activates Pixi, sources the built workspace, starts simulation_real.launch.py and opens RViz.

```bash
sed -n '1,110p' start_sim.sh
ls src/edgeai_robot_description/rviz/
ros2 topic info /scan --verbose
```

Use odom as the RViz Fixed Frame for standalone simulation. Displays of RobotModel, TF, LaserScan, RGB, Depth and PointCloud2 should be checked in the GUI. The checked-in RViz configuration may use Reliable QoS for sensor displays; if a display is blank, compare its subscription reliability with the published topic using ros2 topic info --verbose, and adjust only as needed. Do not assume Best Effort is always the saved default.

## 2.10 Rebuild and validate the integrated source

When source files are modified, terminate the old running simulation first; rebuild in the project Pixi environment, source the workspace and restart. Students who only inspect the files need not rebuild.

```bash
# In a Pixi-enabled shell:
cd ~/edgeai_robot_ws
python -m py_compile src/edgeai_robot_description/launch/simulation_real.launch.py
colcon build --packages-select edgeai_robot_description --symlink-install
source install/setup.bash
# In a regular terminal after stopping the previous instance:
./start_sim.sh
```

Confirm: Gazebo arena and realistic chassis visible; /odom, /joint_states, /scan and /camera topics present; RViz can display robot and sensors. Test motion only in clear space and explicitly send a zero velocity command after manual control.

```bash
timeout 2s ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
'{linear: {x: 0.15}, angular: {z: 0.0}}'
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
'{linear: {x: 0.0}, angular: {z: 0.0}}'
```

Known limitation During a collision, the physical robot in Gazebo can stop while wheel-based /odom continues accumulating displacement, causing the RViz pose to drift. Do not treat odometry as ground truth or intentionally drive into obstacles to test it.

# Part III — SLAM Mapping and Map Management

SLAM Toolbox, occupancy mapping, map saving and static map reload

## 3.1 Overview

This tutorial describes how the SLAM mapping pipeline was assembled in the EdgeAI robot workspace and how to operate it. Tutorial 0 covers cloning and building the repository; Tutorial 1 explains how the robot simulation and ROS interfaces were constructed. Here, we connect the already available LiDAR, odometry, TF and simulation clock to SLAM Toolbox, build an occupancy grid, export the map, and reload it independently with Nav2 Map Server.

This guide follows the two original development tutorials, but removes intermediate experiments and debugging history. It uses the configuration files and commands in the published baseline. AMCL localization and autonomous navigation are outside its scope.

| **Source file**                                                   | **Role in this tutorial**                              |
|-------------------------------------------------------------------|--------------------------------------------------------|
| **src/edgeai_robot_description/config/slam_toolbox.yaml**         | Mapping parameters: frames, laser range and resolution |
| **src/edgeai_robot_description/launch/simulation_real.launch.py** | Gazebo + ROS bridges for /scan, /odom, /tf and /clock  |
| **src/edgeai_robot_description/maps/edgeai_arena.yaml + .pgm**    | Example occupancy map included in the repository       |
| **src/edgeai_robot_description/CMakeLists.txt**                   | Installs config and maps resources in the ROS package  |
| **start_sim.sh**                                                  | Starts the realistic robot simulation and RViz         |

## 3.2 Prerequisites and terminal layout

Complete Tutorial 0 first. The repository should already be cloned and built, and ./start_sim.sh should launch Gazebo and RViz. The Pixi dependencies in the current repository include SLAM Toolbox, Nav2 Map Server and the required ROS command-line utilities. No separate pixi add step is required for users of this baseline.

| **Terminal**   | **Purpose**                                                  | **Keep running?**                 |
|----------------|--------------------------------------------------------------|-----------------------------------|
| **Terminal 1** | Gazebo, robot simulation and RViz                            | Yes, during mapping and reloading |
| **Terminal 2** | SLAM Toolbox, later Nav2 Map Server                          | Yes, for the selected process     |
| **Terminal 3** | Topic inspection, commands, map saving and lifecycle control | Use as needed                     |

### 3.2.1 Launch the simulation — Terminal 1

Open a normal Ubuntu terminal, not an already activated ROS environment.

```bash
cd ~/edgeai_robot_ws
./start_sim.sh
```

Expected: Gazebo displays the realistic mobile robot inside the 5 × 5 m arena; RViz opens with the stored visualization configuration. Keep Terminal 1 running.

### 3.2.2 Prepare a ROS 2 command terminal — Terminal 3

Open a second terminal. Wait until pixi shell finishes before executing the source command.

```bash
cd ~/edgeai_robot_ws
pixi shell
source install/setup.bash
```

NOTE In subsequent steps, “prepared terminal” means the above Pixi environment with install/setup.bash sourced. If you cloned elsewhere, use the actual repository path.

## 3.3 SLAM inputs and system architecture

SLAM Toolbox consumes a 2D scan and robot motion information to estimate a map and a consistent mapping frame. The relevant pipeline is:

Gazebo LiDAR /scan + DiffDrive /odom + TF + /clock  
↓ ros_gz_bridge  
ROS 2 + SLAM Toolbox  
↓  
/map (OccupancyGrid) + map → odom TF  
↓  
RViz

| **Interface** | **Expected configuration**                                 |
|---------------|------------------------------------------------------------|
| **/scan**     | LaserScan with frame_id lidar_link                         |
| **/odom**     | Odometry with frame_id odom, child_frame_id base_footprint |
| **TF**        | odom → base_footprint → base_link → lidar_link             |
| **/clock**    | Gazebo simulation time bridged to ROS 2                    |
| **/map**      | Published by SLAM Toolbox while mapping                    |

### 3.3.1 Check the input interfaces — Terminal 3

With Terminal 1 running, verify the required topics and inspect a small part of the scan and odometry messages.

```bash
ros2 topic list | grep -E '^/(scan|odom|tf|tf_static|clock)$'
ros2 topic echo /scan --once --field header --qos-reliability best_effort
ros2 topic echo /odom --once --field header
ros2 topic info /clock
```

Expected: scan frame lidar_link; odometry in odom; /clock has a publisher. The frame chain can be checked with the following continuous command; press Ctrl+C after observing the output.

ros2 run tf2_ros tf2_echo base_footprint lidar_link

Expected translation is approximately \[0.100, 0.080, 0.155\] m. This step checks interface availability; it does not establish that wheel odometry is ground truth.

## 3.4 Project SLAM Toolbox configuration

The first development trial used SLAM Toolbox’s official asynchronous mapping defaults. The published baseline now includes a project-specific copy at src/edgeai_robot_description/config/slam_toolbox.yaml; therefore, participants should use this repository file rather than copy and edit the system-wide example.

| **Parameter**       | **Baseline value** | **Reason**                            |
|---------------------|--------------------|---------------------------------------|
| **odom_frame**      | odom               | Matches differential-drive odometry   |
| **map_frame**       | map                | SLAM global mapping frame             |
| **base_frame**      | base_footprint     | Robot pose reference                  |
| **scan_topic**      | /scan              | Simulated LiDAR data                  |
| **mode**            | mapping            | Build an occupancy map                |
| **resolution**      | 0.05               | Grid cell size, metres                |
| **max_laser_range** | 12.0               | Matches simulated LiDAR maximum range |

### 3.4.1 Inspect the supplied configuration — Terminal 3

Open or inspect the parameters already committed in the repository.

```bash
grep -E 'odom_frame:|map_frame:|base_frame:|scan_topic:|mode:|resolution:|max_laser_range:' \
src/edgeai_robot_description/config/slam_toolbox.yaml
```

Expected: the seven settings above appear. Other solver, scan-matching and loop-closure parameters remain in the supplied configuration; this tutorial does not tune them.

NOTE The published CMakeLists.txt installs config/ and maps/ as package resources. If modifying these source files, rebuild the workspace before testing installed package paths.

## 3.5 Start SLAM Toolbox and display the map

### 3.5.1 Launch online asynchronous mapping — Terminal 2

Keep Terminal 1 running. Open Terminal 2 and enter the same Pixi workspace:

```bash
cd ~/edgeai_robot_ws
pixi shell
source install/setup.bash
```

Start the mapping node using the project-owned configuration (run from the repository root):

```bash
ros2 launch slam_toolbox online_async_launch.py \
slam_params_file:="$PWD/src/edgeai_robot_description/config/slam_toolbox.yaml" \
use_sim_time:=true
```

Expected: SLAM Toolbox configures and activates successfully and registers the laser sensor. Leave Terminal 2 running throughout mapping.

### 3.5.2 Inspect mapping topics — Terminal 3

In the prepared command terminal, run:

```bash
ros2 topic list | grep -E "^/(map|map_metadata)$"
ros2 topic info /map
ros2 topic echo /map --once --field info
```

Expected: /map is published as nav_msgs/msg/OccupancyGrid and its info includes a 0.05 m resolution. Map dimensions and origin depend on explored coverage and are not fixed outcomes for every run.

### 3.5.3 Configure RViz

In the RViz instance launched in Terminal 1, set Global Options → Fixed Frame to map, then Add → Map and select Topic /map. Keep RobotModel, TF and LaserScan (/scan) if their transforms are available.

Expected: an occupancy grid appears around the robot, with wall and obstacle returns accumulating as measurements arrive. If the map is visible but the robot is not, inspect TF rather than treating it as a mapping failure.

## 3.6 Build a complete occupancy map

The arena has four outer walls and three internal obstacles, including one rotated obstacle. The aim is to observe walls, obstacle faces and free space from several positions and headings. You do not need to physically drive into every corner, and the LiDAR cannot observe through walls or obstacles.

### 3.6.1 Drive slowly — Terminal 3

Publish a short, low-speed command in open space. The explicit zero-speed command must always be sent afterward, including when the publisher is interrupted.

```bash
timeout 2s ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
'{linear: {x: 0.15}, angular: {z: 0.0}}'
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
'{linear: {x: 0.0}, angular: {z: 0.0}}'
```

### 3.6.2 Change viewing direction — Terminal 3

When there is clearance for rotation, perform a short turn and stop.

```bash
timeout 2s ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
'{linear: {x: 0.0}, angular: {z: 0.2}}'
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
'{linear: {x: 0.0}, angular: {z: 0.0}}'
```

NOTE Never rely on Ctrl+C or timeout alone to stop the robot. A timeout exit status of 124 is normal; do not join the velocity and stop commands using &&. Avoid wall contact: during previous tests Gazebo stopped physically while /odom continued to accumulate motion.

### 3.6.3 Assess map coverage

In RViz, inspect whether the four outer walls, three obstacles, principal free-space regions and the rotated obstacle are represented. Small unknown patches behind obstacles are expected; they need not be filled by driving into obstacles.

## 3.7 Save the occupancy map

The development notes originally saved maps using the SLAM Toolbox save_map service. The current published repository README uses Nav2 Map Server’s map_saver_cli. This tutorial adopts that documented workflow. Map export produces a YAML file and an associated map image, not a SLAM Toolbox serialized pose graph.

### 3.7.1 Export the current map — Terminal 3

Leave Gazebo and SLAM Toolbox running; stop the robot before saving. From the repository root:

```bash
cd ~/edgeai_robot_ws
mkdir -p maps
ros2 run nav2_map_server map_saver_cli -f "$PWD/maps/my_map" \
--ros-args -p use_sim_time:=true
```

Expected: files such as maps/my_map.yaml and maps/my_map.pgm are generated. The image format may depend on map-saving settings. The packaged reference map in src/edgeai_robot_description/maps/ is not overwritten.

### 3.7.2 Check the saved files

Inspect the exported YAML metadata and associated image file.

```bash
ls -lh maps/my_map*
cat maps/my_map.yaml
file maps/my_map.pgm
```

Expected YAML metadata typically includes an image filename, resolution, origin, mode and occupancy thresholds. The original development run produced a 101 × 101 map at 0.05 m/cell, but the exported map size and origin may differ after a new mapping run.

## 3.8 Reload the map without SLAM

This optional closing step tests that the exported image and YAML can be loaded independently as a static occupancy map. It does not perform localization. A map_server lifecycle node must be configured and activated before it publishes the map.

### 3.8.1 Stop mapping — Terminal 2

Press Ctrl+C in the terminal running SLAM Toolbox. Leave Gazebo and RViz in Terminal 1 running.

### 3.8.2 Start Map Server — Terminal 2

Use the same Pixi workspace (if the shell remains active, only source setup as needed). Start the server with the YAML just exported:

```bash
ros2 run nav2_map_server map_server --ros-args \
-p yaml_filename:="$PWD/maps/my_map.yaml" \
-p use_sim_time:=true
```

Expected: the lifecycle node starts and waits for lifecycle transitions. Keep this terminal running.

### 3.8.3 Configure and activate — Terminal 3

In a prepared Pixi terminal:

```bash
ros2 lifecycle get /map_server
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
ros2 lifecycle get /map_server
```

Expected final lifecycle status: active \[3\]. The configure transition reads the YAML and image and the active server provides the occupancy map.

### 3.8.4 Verify the static map

Run:

```bash
ros2 topic info /map
ros2 topic echo /map --once --field info
```

Expected: /map is published as nav_msgs/msg/OccupancyGrid. In RViz, display /map to inspect the saved walls and obstacles. If RViz uses Fixed Frame = map while SLAM is stopped, RobotModel or LaserScan may show a TF error because no localization node is currently publishing map → odom. This does not mean the static map failed to load.

## 3.9 Project files and next steps

| **File / component**                                           | **Purpose**                                                        |
|----------------------------------------------------------------|--------------------------------------------------------------------|
| **config/slam_toolbox.yaml**                                   | Source-controlled SLAM parameters and frame/topic alignment        |
| **simulation_real.launch.py**                                  | Simulation plus ROS/Gazebo scan, odometry, TF and clock bridges    |
| **maps/my_map.yaml and image (working tree)**                  | Fresh local map export; not included until intentionally committed |
| **src/edgeai_robot_description/maps/edgeai_arena.yaml + .pgm** | Reference map shipped in the package                               |
| **nav2_map_server**                                            | Exports/reloads occupancy maps; lifecycle required for map_server  |

The complete flow is: start realistic simulation → check SLAM inputs → launch SLAM Toolbox → inspect /map in RViz → explore the arena → export YAML/image → stop SLAM → activate Nav2 Map Server → verify static /map.

Localization (AMCL) and Nav2 autonomous navigation are separate future stages and are not claimed as completed by this tutorial.
