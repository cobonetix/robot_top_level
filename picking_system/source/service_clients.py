#!/usr/bin/env python3
"""
Service client creation utilities for the picking system.
"""

from rclpy.node import Node
from cobonetix_interfaces.srv import ArmJoint, GpioCommand


def create_service_clients(node: Node):
    """
    Create and wait for service clients.

    Args:
        node: The ROS2 node to create clients on

    Returns:
        Tuple of (joint_client, gpio_client)
    """
    # Create service clients
    joint_client = node.create_client(ArmJoint, 'arm_joint')
    gpio_client = node.create_client(GpioCommand, 'gpio_command')

    # Wait for services to be available
    node.get_logger().info('Waiting for arm_joint service...')
    while not joint_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('arm_joint service not available, waiting...')
    node.get_logger().info('Connected to arm_joint service')

    node.get_logger().info('Waiting for gpio_command service...')
    while not gpio_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('gpio_command service not available, waiting...')
    node.get_logger().info('Connected to gpio_command service')

    return joint_client, gpio_client
