#!/usr/bin/env python3
# Copyright 2023 Georg Novotny
#
# Licensed under the GNU GENERAL PUBLIC LICENSE, Version 3.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.gnu.org/licenses/gpl-3.0.en.html
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

import xacro


def _normalize_namespace(ns: str) -> str:
    ns = (ns or '').strip()
    if not ns:
        return '/drone'
    if not ns.startswith('/'):
        ns = '/' + ns
    return ns


def setup_drone_nodes(context, use_sim_time, xacro_file, yaml_file_path):
    takeoff_hover_height = LaunchConfiguration("takeoff_hover_height").perform(context)
    takeoff_vertical_speed = LaunchConfiguration("takeoff_vertical_speed").perform(context)

    robot_description_config = xacro.process_file(
        xacro_file,
        mappings={
            "params_path": yaml_file_path,
            "takeoff_hover_height": takeoff_hover_height,
            "takeoff_vertical_speed": takeoff_vertical_speed,
        },
    )
    robot_desc = robot_description_config.toxml()

    model_ns = _normalize_namespace(LaunchConfiguration('drone_namespace').perform(context))

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            namespace=model_ns,
            output="screen",
            parameters=[{"use_sim_time": use_sim_time, "robot_description": robot_desc, "frame_prefix": model_ns + "/"}],
            arguments=[robot_desc]
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace=model_ns,
            output='screen',
        ),
        Node(
            package="sjtu_drone_bringup",
            executable="spawn_drone",
            arguments=[robot_desc, model_ns],
            output="screen"
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "world", f"{model_ns}/odom"],
            output="screen"
        ),
    ]


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    yaml_file_path = os.path.join(
        get_package_share_directory('sjtu_drone_bringup'),
        'config', 'drone.yaml'
    )

    with open(yaml_file_path, 'r') as f:
        yaml_dict = yaml.load(f, Loader=yaml.FullLoader)
        default_ns = _normalize_namespace(yaml_dict["namespace"])

    drone_namespace = DeclareLaunchArgument(
        "drone_namespace",
        default_value=default_ns,
        description="Drone ROS namespace (e.g. /drone, /drone_w01)",
    )
    use_gui = DeclareLaunchArgument("use_gui", default_value="true", choices=["true", "false"],
                                    description="Whether to execute gzclient")
    takeoff_hover_height = DeclareLaunchArgument(
        "takeoff_hover_height",
        default_value="1.0",
        description="Target altitude increase (m) after takeoff",
    )
    takeoff_vertical_speed = DeclareLaunchArgument(
        "takeoff_vertical_speed",
        default_value="1.0",
        description="Vertical climb command used during takeoff",
    )
    xacro_file_name = "sjtu_drone.urdf.xacro"
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    xacro_file = os.path.join(
        get_package_share_directory("sjtu_drone_description"),
        "urdf", xacro_file_name
    )

    world_file_default = os.path.join(
        get_package_share_directory("sjtu_drone_description"),
        "worlds", "landingPad.world"
    )

    world_file = LaunchConfiguration('world', default=world_file_default)

    world = DeclareLaunchArgument(
        name='world',
        default_value=world_file_default,
        description='Full path to world file to load'
    )

    def launch_gzclient(context, *args, **kwargs):
        if context.launch_configurations.get('use_gui') == 'true':
            return [IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
                ),
                launch_arguments={'verbose': 'true'}.items()
            )]
        return []

    return LaunchDescription([
        world,
        use_gui,
        drone_namespace,
        takeoff_hover_height,
        takeoff_vertical_speed,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world_file,
                              'verbose': "true",
                              'extra_gazebo_args': 'verbose'}.items()
        ),

        OpaqueFunction(function=launch_gzclient),
        OpaqueFunction(
            function=setup_drone_nodes,
            kwargs={
                "use_sim_time": use_sim_time,
                "xacro_file": xacro_file,
                "yaml_file_path": yaml_file_path,
            },
        ),
    ])
