#!/usr/bin/env python3

import os
import shutil
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def _source_cmd() -> str:
    scripts = [
        '/opt/ros/humble/setup.bash',
        os.path.expanduser('~/IICC26_ws/install/setup.bash'),
        os.path.expanduser('~/SynologyDrive/INCSL/devel/INCSL/IICC26_ws/install/setup.bash'),
        os.path.expanduser('~/gz_ros2_aruco_ws/install/setup.bash'),
    ]
    lines = ['set +e']
    for script in scripts:
        lines.append(f"[ -f '{script}' ] && source '{script}' >/dev/null 2>&1")
    lines.append('set -e')
    return '; '.join(lines)


def _resolve_rviz_config(autolanding_root: str, explicit_config: str) -> str:
    candidates = []
    if explicit_config:
        candidates.append(Path(explicit_config).expanduser())

    env_config = os.environ.get('AUTOLANDING_RVIZ_CONFIG', '').strip()
    if env_config:
        candidates.append(Path(env_config).expanduser())

    root_path = Path(autolanding_root).expanduser()
    candidates.extend(
        [
            root_path / 'simulation' / 'configs' / 'unified_system_monitor.rviz',
            root_path / 'simulation' / 'configs' / 'autolanding_monitor.rviz',
        ]
    )

    for cfg in candidates:
        if cfg.is_file():
            return str(cfg.resolve())
    return ''


def _resolve_rviz_binary() -> str:
    """Prefer ROS-installed RViz binary to avoid snap runtime conflicts."""
    preferred = Path('/opt/ros/humble/bin/rviz2')
    if preferred.is_file():
        return str(preferred)

    which_rviz = shutil.which('rviz2')
    if which_rviz:
        return which_rviz

    return 'rviz2'


def _resolve_bridge_config(autolanding_root: str, explicit_config: str) -> str:
    candidates = []
    if explicit_config:
        candidates.append(Path(explicit_config).expanduser())

    env_config = os.environ.get('AUTOLANDING_ROS_GZ_BRIDGE_CONFIG', '').strip()
    if env_config:
        candidates.append(Path(env_config).expanduser())

    root = Path(autolanding_root).expanduser()
    candidates.extend(
        [
            root / 'simulation' / 'configs' / 'ros_gz_bridge.yaml',
            root / 'simulation' / 'configs' / 'bridge.yaml',
            Path(os.path.expanduser('~/gz_ros2_aruco_ws/src/ros2-gazebo-aruco/config/bridge.yaml')),
        ]
    )

    for cfg in candidates:
        if cfg.is_file():
            return str(cfg.resolve())
    return ''


def _build_actions(context):
    autolanding_root = LaunchConfiguration('autolanding_root').perform(context)
    workers = LaunchConfiguration('workers').perform(context)
    rviz_config = LaunchConfiguration('rviz_config').perform(context)
    bridge_config_override = LaunchConfiguration('bridge_config').perform(context)
    aruco_image_topic = LaunchConfiguration('aruco_image_topic').perform(context)
    aruco_camera_info_topic = LaunchConfiguration('aruco_camera_info_topic').perform(context)

    bridge_config = _resolve_bridge_config(autolanding_root, bridge_config_override)
    odom_script = str(Path(autolanding_root).expanduser() / 'scripts' / 'publish_multi_drone_odom.py')
    resolved_rviz = _resolve_rviz_config(autolanding_root, rviz_config)
    rviz_binary = _resolve_rviz_binary()
    source = _source_cmd()
    env_clean = (
        'unset LD_LIBRARY_PATH QT_PLUGIN_PATH QML2_IMPORT_PATH QT_QPA_PLATFORMTHEME PYTHONUSERBASE '
        'SNAP SNAP_NAME SNAP_REVISION SNAP_ARCH SNAP_LIBRARY_PATH SNAP_INSTANCE_NAME'
    )

    rviz_cmd = f"'{rviz_binary}'"
    if resolved_rviz:
        rviz_cmd += f" -d '{resolved_rviz}'"

    rviz_cmd_isolated = (
        "env -i "
        "HOME=\"$HOME\" USER=\"${USER:-j}\" LOGNAME=\"${LOGNAME:-$USER}\" SHELL=/bin/bash "
        "PATH=/opt/ros/humble/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin "
        "LD_LIBRARY_PATH=/opt/ros/humble/opt/rviz_ogre_vendor/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib "
        "AMENT_PREFIX_PATH=\"${AMENT_PREFIX_PATH:-/opt/ros/humble}\" "
        "COLCON_PREFIX_PATH=\"${COLCON_PREFIX_PATH:-/opt/ros/humble}\" "
        "CMAKE_PREFIX_PATH=\"${CMAKE_PREFIX_PATH:-/opt/ros/humble}\" "
        "ROS_DISTRO=\"${ROS_DISTRO:-humble}\" ROS_VERSION=\"${ROS_VERSION:-2}\" "
        "ROS_PYTHON_VERSION=\"${ROS_PYTHON_VERSION:-3}\" "
        "XDG_RUNTIME_DIR=\"${XDG_RUNTIME_DIR:-/tmp/runtime-${USER:-j}}\" "
        "DISPLAY=\"${DISPLAY:-:0}\" ROS_DOMAIN_ID=\"${ROS_DOMAIN_ID:-0}\" "
        f"{rviz_cmd}"
    )

    actions = []

    actions.append(
        ExecuteProcess(
            cmd=[
                'bash',
                '-lc',
                f"{env_clean}; {source}; ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world map",
            ],
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_world_map_tf')),
        )
    )

    if bridge_config:
        actions.append(
            ExecuteProcess(
                cmd=[
                    'bash',
                    '-lc',
                    (
                        f"{env_clean}; {source}; "
                        f"ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:={bridge_config}"
                    ),
                ],
                output='screen',
                condition=IfCondition(LaunchConfiguration('use_ros_gz_bridge')),
            )
        )
    else:
        actions.append(
            LogInfo(
                msg='[autolanding_unified_monitor] ros_gz_bridge skipped: no valid bridge_config found '
                    '(set bridge_config:=/abs/path/to/bridge.yaml)'
            )
        )

    actions.extend([
        ExecuteProcess(
            cmd=[
                'bash',
                '-lc',
                (
                    f"{env_clean}; {source}; "
                    'export PYTHONNOUSERSITE=1; '
                    'ros2 run ros2_aruco aruco_node --ros-args '
                    f"-p image_topic:={aruco_image_topic} "
                    f"-p camera_info_topic:={aruco_camera_info_topic} "
                    '-p marker_size:=0.4 -p aruco_dictionary_id:=DICT_ARUCO_ORIGINAL'
                ),
            ],
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_ros2_aruco')),
        ),
        ExecuteProcess(
            cmd=[
                'bash',
                '-lc',
                f"{env_clean}; {source}; python3 '{odom_script}' --workers {workers}",
            ],
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_multi_drone_odom')),
        ),
        ExecuteProcess(
            cmd=['bash', '-lc', rviz_cmd_isolated],
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ])

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'autolanding_root',
                default_value=os.path.expanduser('~/SynologyDrive/INCSL/devel/INCSL/autolanding_ws'),
                description='Path to autolanding_ws root',
            ),
            DeclareLaunchArgument('ros_domain_id', default_value='0', description='ROS_DOMAIN_ID value'),
            DeclareLaunchArgument('workers', default_value='1', description='Worker count for odom publisher'),
            DeclareLaunchArgument('rviz_config', default_value='', description='Optional explicit RViz config path'),
            DeclareLaunchArgument('bridge_config', default_value='', description='Optional ros_gz_bridge YAML config path'),
            DeclareLaunchArgument('aruco_image_topic', default_value='/autolanding/drone1/camera', description='Input image topic for ros2_aruco'),
            DeclareLaunchArgument('aruco_camera_info_topic', default_value='/autolanding/drone1/camera_info', description='Input camera_info topic for ros2_aruco'),
            DeclareLaunchArgument('use_world_map_tf', default_value='true', choices=['true', 'false']),
            DeclareLaunchArgument('use_ros_gz_bridge', default_value='true', choices=['true', 'false']),
            DeclareLaunchArgument('use_ros2_aruco', default_value='true', choices=['true', 'false']),
            DeclareLaunchArgument('use_multi_drone_odom', default_value='true', choices=['true', 'false']),
            DeclareLaunchArgument('use_rviz', default_value='true', choices=['true', 'false']),
            SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('ros_domain_id')),
            OpaqueFunction(function=_build_actions),
        ]
    )
