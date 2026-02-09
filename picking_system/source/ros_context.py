#!/usr/bin/env python3
"""
Global ROS2 context for the picking system.

Creates the node, service clients, and subscriptions once,
then exposes them as module-level globals for all routines.
"""

from pathlib import Path

import rclpy
from rclpy.node import Node
from cobonetix_interfaces.srv import ArmJoint, GpioCommand, GpioStatus
from sensor_msgs.msg import JointState

# Path to the shared data directory (robot_top_level/data/)
DATA_DIR = Path(__file__).resolve().parent.parent.parent / 'data'

# Module-level globals
node: Node = None
joint_client = None
arm_cmd_client = None
arm_status_client = None
latest_joint_state: JointState = None


def _joint_state_callback(msg: JointState):
    """Store the latest joint state message."""
    global latest_joint_state
    latest_joint_state = msg


def init(args=None):
    """
    Initialize ROS2 and create the picking system node with all
    service clients and subscriptions.

    Call this once at startup before any other picking routines.
    """
    global node, joint_client, arm_cmd_client, arm_status_client

    rclpy.init(args=args)
    node = Node('picking_system')

    # Create service clients
    joint_client = node.create_client(ArmJoint, '/arm_joint')
    arm_cmd_client = node.create_client(GpioCommand, '/gpio_command')
    arm_status_client = node.create_client(GpioStatus, '/gpio_status')

    # Subscribe to joint states
    node.create_subscription(JointState, '/joint_states', _joint_state_callback, 10)

    # Wait for services to be available
    node.get_logger().info('Waiting for arm_joint service...')
    while not joint_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('arm_joint service not available, waiting...')
    node.get_logger().info('Connected to arm_joint service')

    node.get_logger().info('Waiting for gpio_command service...')
    while not arm_cmd_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('gpio_command service not available, waiting...')
    node.get_logger().info('Connected to gpio_command service')

    node.get_logger().info('Waiting for gpio_status service...')
    while not arm_status_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('gpio_status service not available, waiting...')
    node.get_logger().info('Connected to gpio_status service')

    node.get_logger().info('Picking system node initialized')


def shutdown():
    """Destroy the node and shut down ROS2."""
    global node
    if node is not None:
        node.destroy_node()
        node = None
    rclpy.shutdown()
