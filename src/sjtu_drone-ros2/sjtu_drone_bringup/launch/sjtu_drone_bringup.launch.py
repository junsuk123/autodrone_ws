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
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, ExecuteProcess, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def get_teleop_controller(context, *_, **kwargs) -> Node:
    controller = context.launch_configurations["controller"]
    namespace = kwargs["model_ns"]

    if controller == "joystick":
        node = Node(
            package="sjtu_drone_control",
            executable="teleop_joystick",
            namespace=namespace,
            output="screen",
        )

    else:
        node = Node(
            package="sjtu_drone_control",
            executable="teleop",
            namespace=namespace,
            output="screen",
            prefix="xterm -e",
        )

    return [node]

def rviz_node_generator(context, rviz_path):
    """Return a Node action for RViz, omitting --fixed-frame if empty."""
    if LaunchConfiguration('use_rviz').perform(context) != 'true':
        return []

    fixed_frame_value = LaunchConfiguration('fixed_frame').perform(context)

    rviz_arguments = ['-d', rviz_path]

    if fixed_frame_value:
        rviz_arguments.extend(['--fixed-frame', fixed_frame_value])

    return [
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=rviz_arguments,
            output='screen',
        )
    ]


def get_apriltag_nodes(context, *_, **__):
    use_apriltag = LaunchConfiguration("use_apriltag").perform(context)
    if use_apriltag != "true":
        return []

    try:
        apriltag_share = get_package_share_directory("apriltag_detector")
    except Exception:
        print("[bringup] apriltag_detector package not found. Skipping AprilTag detector launch.")
        return []

    camera_ns = LaunchConfiguration("apriltag_camera").perform(context)
    image_topic = LaunchConfiguration("apriltag_image").perform(context)
    tags_topic = LaunchConfiguration("apriltag_tags").perform(context)
    detector_type = LaunchConfiguration("apriltag_type").perform(context)
    bridge_topic = LaunchConfiguration("apriltag_bridge_topic").perform(context)
    bridge_use_target_id = LaunchConfiguration("apriltag_bridge_use_target_id").perform(context)
    bridge_target_id = LaunchConfiguration("apriltag_bridge_target_id").perform(context)
    use_standalone_detector = LaunchConfiguration("apriltag_use_standalone_detector").perform(context)
    bridge_exe = os.path.join(get_package_prefix('sjtu_drone_bringup'), 'bin', 'apriltag_state_bridge')

    tags_full_topic = f'{camera_ns}/{tags_topic}'
    image_full_topic = f'{camera_ns}/{image_topic}'

    actions = [
        LogInfo(msg=f'[bringup] Apriltag config: camera={camera_ns}, image={image_full_topic}, tags={tags_full_topic}, bridge={bridge_topic}, standalone={use_standalone_detector}, use_target_id={bridge_use_target_id}, target_id={bridge_target_id}')
    ]

    bridge_action = None
    if os.path.exists(bridge_exe):
        bridge_action = ExecuteProcess(
            cmd=[
                bridge_exe,
                '--ros-args',
                '-p', f'input_topic:={tags_full_topic}',
                '-p', f'output_topic:={bridge_topic}',
                '-p', f'target_id:={bridge_target_id}',
                '-p', f'use_target_id:={bridge_use_target_id}',
            ],
            output='screen',
            condition=IfCondition(LaunchConfiguration("use_apriltag")),
        )
    else:
        print(f"[bringup] apriltag_state_bridge executable not found at {bridge_exe}. Skipping bridge node.")

    if use_standalone_detector == 'true':
        actions.append(
            Node(
                package='apriltag_detector',
                executable='apriltag_detector_node',
                name='apriltag_detector_node',
                output='screen',
                parameters=[
                    {
                        'type': detector_type,
                    }
                ],
                remappings=[
                    ('image', image_full_topic),
                    ('tags', tags_full_topic),
                ],
                condition=IfCondition(LaunchConfiguration("use_apriltag")),
            )
        )
    else:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(apriltag_share, "launch", "detect.launch.py")
                ),
                launch_arguments={
                    "camera": camera_ns,
                    "image": image_topic,
                    "tags": tags_topic,
                    "type": detector_type,
                }.items(),
                condition=IfCondition(LaunchConfiguration("use_apriltag")),
            )
        )

    if bridge_action is not None:
        actions.append(bridge_action)

    return actions


def generate_launch_description():
    sjtu_drone_bringup_path = get_package_share_directory('sjtu_drone_bringup')

    rviz_path = os.path.join(
        sjtu_drone_bringup_path, "rviz", "rviz.rviz"
    )

    yaml_file_path = os.path.join(
        get_package_share_directory('sjtu_drone_bringup'),
        'config', 'drone.yaml'
    )

    model_ns = "drone"

    with open(yaml_file_path, 'r') as f:
        yaml_dict = yaml.load(f, Loader=yaml.FullLoader)
        model_ns = yaml_dict["namespace"]

    return LaunchDescription([
        DeclareLaunchArgument(
            "controller",
            default_value="keyboard",
            description="Type of controller: keyboard (default) or joystick",
        ),

        DeclareLaunchArgument(
            "use_apriltag",
            default_value="true",
            choices=["true", "false"],
            description="Whether to start apriltag_detector for landing zone tag tracking",
        ),

        DeclareLaunchArgument(
            "apriltag_camera",
            default_value="/drone/bottom",
            description="Camera namespace for apriltag detector",
        ),

        DeclareLaunchArgument(
            "apriltag_image",
            default_value="image_raw",
            description="Image topic name under camera namespace",
        ),

        DeclareLaunchArgument(
            "apriltag_tags",
            default_value="tags",
            description="Output detections topic name under camera namespace",
        ),

        DeclareLaunchArgument(
            "apriltag_type",
            default_value="umich",
            choices=["umich", "mit"],
            description="Apriltag detector backend type",
        ),

        DeclareLaunchArgument(
            "apriltag_bridge_topic",
            default_value="/landing_tag_state",
            description="Bridge topic to publish tag state as Float32MultiArray",
        ),

        DeclareLaunchArgument(
            "apriltag_bridge_use_target_id",
            default_value="false",
            choices=["true", "false"],
            description="Whether apriltag_state_bridge filters by target_id",
        ),

        DeclareLaunchArgument(
            "apriltag_bridge_target_id",
            default_value="0",
            description="Target tag id used by apriltag_state_bridge when filtering is enabled",
        ),

        DeclareLaunchArgument(
            "apriltag_use_standalone_detector",
            default_value="true",
            choices=["true", "false"],
            description="Use apriltag_detector_node directly instead of detect.launch composable container",
        ),

        DeclareLaunchArgument(
            "takeoff_hover_height",
            default_value="1.0",
            description="Target altitude increase (m) after takeoff",
        ),

        DeclareLaunchArgument(
            "takeoff_vertical_speed",
            default_value="1.0",
            description="Vertical climb command used during takeoff",
        ),

        DeclareLaunchArgument(
            'fixed_frame',
            default_value='',
            description='If provided, sets the fixed frame in RViz.'
        ),

        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            choices=['true', 'false'],
            description='Whether to launch RViz2',
        ),

        DeclareLaunchArgument(
            'use_teleop',
            default_value='true',
            choices=['true', 'false'],
            description='Whether to launch joystick and teleop nodes',
        ),

        OpaqueFunction(
            function=rviz_node_generator,
            kwargs={'rviz_path': rviz_path},
        ),


        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sjtu_drone_bringup_path, 'launch', 'sjtu_drone_gazebo.launch.py')
            ),
            launch_arguments={
                'takeoff_hover_height': LaunchConfiguration('takeoff_hover_height'),
                'takeoff_vertical_speed': LaunchConfiguration('takeoff_vertical_speed'),
            }.items(),
        ),

        Node(
            package='joy',
            executable='joy_node',
            name='joy',
            namespace=model_ns,
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_teleop')),
        ),

        OpaqueFunction(
            function=get_teleop_controller,
            kwargs={'model_ns': model_ns},
            condition=IfCondition(LaunchConfiguration('use_teleop')),
        ),

        OpaqueFunction(
            function=get_apriltag_nodes,
        ),
    ])
