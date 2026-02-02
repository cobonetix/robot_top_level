# Copyright 2023 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start Rviz2 and Joint State Publisher gui automatically \
        with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "joint_service",
            default_value="true",
            description="Start the joint service server.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "pose_service",
            default_value="true",
            description="Start the pose service server.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gpio_service",
            default_value="true",
            description="Start the GPIO command service server.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gpio_status_service",
            default_value="true",
            description="Start the GPIO status service server.",
        )
    )
    # Initialize Arguments
    gui = LaunchConfiguration("gui")
    joint_service = LaunchConfiguration("joint_service")
    pose_service = LaunchConfiguration("pose_service")
    gpio_service = LaunchConfiguration("gpio_service")
    gpio_status_service = LaunchConfiguration("gpio_status_service")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("cobo_arm_control"),
                    "urdf",
                    "cobo.urdf.xacro",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("cobo_arm_control"), "rviz", "view_robot.rviz"]
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(gui),
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    # Path to cobonetix services directory
    # Use absolute path since launch file may run from install directory
    services_dir = os.path.expanduser(
        "~/ros2_ws/src/ros-controls/cobonetix/services"
    )

    # Service server processes
    joint_service_server = ExecuteProcess(
        cmd=["python3", os.path.join(services_dir, "joint_service_server.py")],
        name="joint_service_server",
        output="screen",
        condition=IfCondition(joint_service),
    )

    pose_service_server = ExecuteProcess(
        cmd=["python3", os.path.join(services_dir, "pose_service_server.py")],
        name="pose_service_server",
        output="screen",
        condition=IfCondition(pose_service),
    )

    gpio_service_server = ExecuteProcess(
        cmd=["python3", os.path.join(services_dir, "gpio_service_server.py")],
        name="gpio_service_server",
        output="screen",
        condition=IfCondition(gpio_service),
    )

    gpio_status_server = ExecuteProcess(
        cmd=["python3", os.path.join(services_dir, "gpio_status_server.py")],
        name="gpio_status_server",
        output="screen",
        condition=IfCondition(gpio_status_service),
    )

    #
    nodes_to_start = [
        joint_state_publisher_node,
        robot_state_publisher_node,
        joint_service_server,
        pose_service_server,
        gpio_service_server,
        gpio_status_server,
#rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
