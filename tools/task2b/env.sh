# Source from the repository root; reuse the existing Pixi environment.
export CONDA_PREFIX=/home/yueer/edgeai_robot_ws/.pixi/envs/default
export PATH="$CONDA_PREFIX/bin:$PATH"
export ROS_DISTRO=jazzy
source tools/pixi_activate.sh
source "$CONDA_PREFIX/setup.bash"
source install/setup.bash
export EDGEAI_CONFIG_DIR="$PWD/config"
export ROS_DOMAIN_ID=104
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export PYTHONDONTWRITEBYTECODE=1
