# Sourced by pixi on activation (pixi.toml [activation] scripts).
# A shell that already sourced /opt/ros/jazzy/setup.bash (e.g. from ~/.bashrc)
# leaks system ROS paths into pixi: pytest then loads the system launch_ros
# plugin against pixi's launch_testing (PluginValidationError in every package)
# and gz loads system Gazebo pieces. Drop every /opt/ros entry; pixi provides ROS.
for _edgeai_var in PATH PYTHONPATH LD_LIBRARY_PATH CMAKE_PREFIX_PATH AMENT_PREFIX_PATH \
        COLCON_PREFIX_PATH PKG_CONFIG_PATH GZ_SIM_RESOURCE_PATH GZ_SIM_SYSTEM_PLUGIN_PATH \
        GZ_GUI_PLUGIN_PATH; do
    _edgeai_kept=""
    IFS=: read -r -a _edgeai_parts <<< "${!_edgeai_var-}"
    for _edgeai_part in "${_edgeai_parts[@]}"; do
        case "$_edgeai_part" in
            /opt/ros/*|"") ;;
            *) _edgeai_kept="${_edgeai_kept:+$_edgeai_kept:}$_edgeai_part" ;;
        esac
    done
    # Export even when empty: pixi does not carry an unset into the shell.
    if [ -z "$_edgeai_kept" ] && [ "$_edgeai_var" = CMAKE_PREFIX_PATH ]; then
        _edgeai_kept="$CONDA_PREFIX"
    fi
    export "$_edgeai_var=$_edgeai_kept"
done
unset _edgeai_var _edgeai_kept _edgeai_parts _edgeai_part
