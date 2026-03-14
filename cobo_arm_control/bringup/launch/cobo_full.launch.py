# cobo_full.launch.py
#
# Merged launch file for the complete cobo robot:
#   - Diff drive base  (diffdrive_arduino hardware plugin)
#   - Cobo arm + GPIO  (cobo_arm_control hardware plugin)
#
# Replaces running diffbot.launch.py and cobo_controller.launch.py at the
# same time, which caused conflicts because each file spawned its own
# ros2_control_node, robot_state_publisher, and joint_state_broadcaster.
#
# Uses:
#   URDF   : cobo_arm_control/description/urdf/cobo_full.urdf.xacro
#   Config : cobo_arm_control/bringup/config/cobo_full_controllers.yaml

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Arguments ────────────────────────────────────────────────────────────
    declared_arguments = [
        DeclareLaunchArgument(
            "gui",
            default_value="false",
            description="Start RViz2 automatically with this launch file.",
        ),
        DeclareLaunchArgument(
            "joint_service",
            default_value="true",
            description="Start the joint service server.",
        ),
        DeclareLaunchArgument(
            "pose_service",
            default_value="true",
            description="Start the pose service server.",
        ),
        DeclareLaunchArgument(
            "gpio_service",
            default_value="true",
            description="Start the GPIO command service server.",
        ),
        DeclareLaunchArgument(
            "gpio_status_service",
            default_value="true",
            description="Start the GPIO status service server.",
        ),
        DeclareLaunchArgument(
            "trajectory_service",
            default_value="true",
            description="Start the trajectory select service server.",
        ),
    ]

    gui               = LaunchConfiguration("gui")
    joint_service     = LaunchConfiguration("joint_service")
    pose_service      = LaunchConfiguration("pose_service")
    gpio_service      = LaunchConfiguration("gpio_service")
    gpio_status_service = LaunchConfiguration("gpio_status_service")
    trajectory_service  = LaunchConfiguration("trajectory_service")

    # ── Combined URDF ─────────────────────────────────────────────────────────
    robot_description_content = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [FindPackageShare("cobo_arm_control"), "urdf", "cobo_full.urdf.xacro"]
                ),
            ]
        ),
        value_type=str,
    )
    robot_description = {"robot_description": robot_description_content}

    # ── Combined controller config ────────────────────────────────────────────
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("cobo_arm_control"), "config", "cobo_full_controllers.yaml"]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("cobo_arm_control"), "rviz", "view_robot.rviz"]
    )

    # ── Single ros2_control_node (controller manager) ─────────────────────────
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        remappings=[
            ("/diff_drive_controller/odom", "/odom"),
            ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
        ],
        output="both",
    )

    # ── Single robot_state_publisher ──────────────────────────────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # ── RViz (optional) ───────────────────────────────────────────────────────
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    # ── Controller spawners ───────────────────────────────────────────────────
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    # Spawn diffbot_base_controller after joint_state_broadcaster is active
    diffbot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "-c", "/controller_manager"],
    )

    cobo_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "cobo_controller", "-c", "/controller_manager",
            "-t", "cobo_arm_control/RobotController",
        ],
    )

    gpio_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gpio_controller", "-c", "/controller_manager"],
    )

    # Delay arm/drive/gpio controllers until joint_state_broadcaster is running
    delay_controllers_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                diffbot_controller_spawner,
                cobo_controller_spawner,
                gpio_controller_spawner,
            ],
        )
    )

    # ── Service servers ───────────────────────────────────────────────────────
    services_dir = os.path.expanduser(
        "~/ros2_ws/src/robot_top_level/services"
    )

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

    # ── Launch description ────────────────────────────────────────────────────
    return LaunchDescription(
        declared_arguments + [
            robot_state_publisher,
          
            control_node,
            joint_state_broadcaster_spawner,
            delay_controllers_after_jsb,
          #  rviz_node,
            joint_service_server,
            pose_service_server,
            gpio_service_server,
            gpio_status_server,
            trajectory_service_server,
        ]
    )
