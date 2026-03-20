#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _build_parallel_actions(context):
    count = int(LaunchConfiguration('instance_count').perform(context))
    domain_base = int(LaunchConfiguration('domain_id_base').perform(context))
    port_base = int(LaunchConfiguration('gazebo_port_base').perform(context))
    stagger = float(LaunchConfiguration('startup_stagger_sec').perform(context))

    use_gui_first_only = LaunchConfiguration('use_gui_first_only').perform(context).lower() == 'true'

    bringup_share = get_package_share_directory('sjtu_drone_bringup')
    bringup_launch = os.path.join(bringup_share, 'launch', 'sjtu_drone_bringup.launch.py')

    actions = []
    for i in range(count):
        domain_id = domain_base + i
        gazebo_port = port_base + i
        use_gui = 'true' if (use_gui_first_only and i == 0) else 'false'
        drone_namespace = f'/drone_w{i + 1:02d}'

        group = GroupAction(
            scoped=True,
            actions=[
                SetEnvironmentVariable('ROS_DOMAIN_ID', str(domain_id)),
                SetEnvironmentVariable('GAZEBO_MASTER_URI', f'http://127.0.0.1:{gazebo_port}'),
                LogInfo(msg=f'[parallel] instance={i} ns={drone_namespace} domain={domain_id} gazebo_port={gazebo_port} gui={use_gui}'),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(bringup_launch),
                    launch_arguments={
                        'drone_namespace': drone_namespace,
                        'use_gui': use_gui,
                        'use_rviz': LaunchConfiguration('use_rviz').perform(context),
                        'use_teleop': LaunchConfiguration('use_teleop').perform(context),
                        'use_apriltag': LaunchConfiguration('use_apriltag').perform(context),
                        'takeoff_hover_height': LaunchConfiguration('takeoff_hover_height').perform(context),
                        'takeoff_vertical_speed': LaunchConfiguration('takeoff_vertical_speed').perform(context),
                    }.items(),
                ),
            ],
        )

        if stagger > 0.0 and i > 0:
            actions.append(TimerAction(period=stagger * i, actions=[group]))
        else:
            actions.append(group)

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('instance_count', default_value='4', description='Number of parallel Gazebo+ROS instances'),
        DeclareLaunchArgument('domain_id_base', default_value='40', description='First ROS_DOMAIN_ID value'),
        DeclareLaunchArgument('gazebo_port_base', default_value='12045', description='First Gazebo master port'),
        DeclareLaunchArgument('startup_stagger_sec', default_value='2.0', description='Delay between starting each instance'),
        DeclareLaunchArgument('use_gui_first_only', default_value='false', choices=['true', 'false'], description='Enable GUI only for instance 0'),
        DeclareLaunchArgument('use_rviz', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('use_teleop', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('use_apriltag', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('takeoff_hover_height', default_value='1.0'),
        DeclareLaunchArgument('takeoff_vertical_speed', default_value='1.0'),
        OpaqueFunction(function=_build_parallel_actions),
    ])
