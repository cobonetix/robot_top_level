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
from launch.actions import RegisterEventHandler, DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
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
            description="Start RViz2 automatically with this launch file.",
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
    declared_arguments.append(
        DeclareLaunchArgument(
            "trajectory_service",
            default_value="true",
            description="Start the trajectory select service server.",
        )
    )
    # Initialize Arguments
    gui = LaunchConfiguration("gui")
    joint_service = LaunchConfiguration("joint_service")
    pose_service = LaunchConfiguration("pose_service")
    gpio_service = LaunchConfiguration("gpio_service")
    gpio_status_service = LaunchConfiguration("gpio_status_service")
    trajectory_service = LaunchConfiguration("trajectory_service")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("cobo_arm_control"),
                    "urdf",
                    "bob4.urdf.xacro",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}
     
    
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("cobo_arm_control"),
            "config",
            "cobo_controller.yaml",
        ]
    )
         
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("cobo_arm_control"), "description/rviz", "view_robot.rviz"]
    )
    
    
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
    )
    

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ], 
        output="both",
     )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file]
  )
        
    usbcam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cama',
        parameters=[{
            'video_device': '/dev/video0',
            'framerate': 10.0,
        }]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

 
    cobo_controller= Node(
        package="controller_manager",
        executable="spawner",
        arguments=["cobo_controller", "-c", "/controller_manager","-t","cobo_arm_control::RobotController"]
    )

    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=control_node,
            on_exit=[joint_state_broadcaster_spawner]
        )
    )

    cobo_basex_xbox_controller = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            output="screen",
            parameters=[{
            "joy_config": "xbox"}]    
    ),

    cobo_base_xbox_controller = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy_node",
        output="screen",
        parameters=[{
            "joy_config": "xbox"}]
    )
     
     
    cobo_arm_xbox_controller = Node(
        package="arm_pose_joy",
        executable="arm_node",
        name="arm_pose_joy",
        output="screen",
        parameters=[{
            "joy_config": "xbox"}]
    )
     
    cobo_joy_controller = Node(
     
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
                'dev': '/dev/input/js0',
                'deadzone': 0.3,
                'autorepeat_rate': 20.0,
                'coalesce_interval_ms': 100
            }]
    )

    gpio_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gpio_controller", "-c", "/controller_manager"]
    )

    # Path to cobonetix services directory
    # Use absolute path since launch file may run from install directory
    services_dir = os.path.expanduser(
        "~/ros2_ws/src/robot_top_level/services"
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

    trajectory_service_server = ExecuteProcess(
        cmd=["python3", os.path.join(services_dir, "trajectory_service_server.py")],
        name="trajectory_service_server",
        output="screen",
        condition=IfCondition(trajectory_service),
    )

      # Load controllers
    load_controllers = []
    for controller in [
        "cobo_controller",  
        "gpio_controller"

        ] :
        
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]

    return LaunchDescription(
        declared_arguments + [
#usbcam_node,
           # rviz_node,
            robot_state_publisher,
            control_node,
       #     cobo_joy_controller,
#            gpio_controller,
            joint_state_broadcaster_spawner,
        #    cobo_arm_xbox_controller,
#cobo_base_xbox_controller,
            joint_service_server,
            pose_service_server,
            gpio_service_server,
            gpio_status_server,
            trajectory_service_server,
        ]  + load_controllers
        )
    
    
    
    
    
    
    
    
    

