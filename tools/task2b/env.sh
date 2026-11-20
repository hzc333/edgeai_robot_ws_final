# Source from the repository root: the repository's Pixi environment (or the one already
# active, e.g. inside `pixi run`) and its build.
export CONDA_PREFIX="${CONDA_PREFIX:-$PWD/.pixi/envs/default}"
export PATH="$CONDA_PREFIX/bin:$PATH"
export ROS_DISTRO=jazzy
# pixi.toml's middleware; without it ROS falls back to FastDDS, whose participant
# teardown sometimes busy-waits forever when the nodes are stopped.
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
source tools/pixi_activate.sh
source "$CONDA_PREFIX/setup.bash"
source install/setup.bash
export EDGEAI_CONFIG_DIR="$PWD/config"
export ROS_DOMAIN_ID=104
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export PYTHONDONTWRITEBYTECODE=1
