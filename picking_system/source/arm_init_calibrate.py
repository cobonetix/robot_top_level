#!/usr/bin/env python3
"""
Procedures to initialize arm joints and calibrate the tower.

All functions use the global node and service clients from ros_context.
"""

import time
import rclpy
import random
from cobonetix_interfaces.srv import GpioCommand

import ros_context
from picking_utils import send_joint_request, send_trajectory_request, wait_for_arm_idle


def set_all_joints(j1_position: float = 1.0, other_position: float = 3.14) -> bool:
    """Set all joints of both arms and wait until they stop moving.

    Args:
        j1_position: Joint 1 position in radians (default 1.0)
        other_position: Position for joints 2, 3, 4 in radians (default 3.14)

    Returns:
        True if successful and arms reached idle.
    """
    left = (j1_position, other_position, other_position, other_position)
    right = (j1_position, other_position, other_position, other_position)
    return send_joint_request(left, right)


def set_joint1_both_arms(position: float) -> bool:
    """Set joint 1 of both arms and wait until they stop moving.

    Args:
        position: Joint position in radians

    Returns:
        True if successful and arms reached idle.
    """
    left = (position, 0.0, 0.0, 0.0)
    right = (position, 0.0, 0.0, 0.0)
    return send_joint_request(left, right)


def display_actual_positions():
    """Display actual joint positions from /joint_states topic."""
    timeout = time.time() + 1.0
    while ros_context.latest_joint_state is None and time.time() < timeout:
        rclpy.spin_once(ros_context.node, timeout_sec=0.1)

    if ros_context.latest_joint_state is None:
        ros_context.node.get_logger().warn('No joint state received')
        return

    ros_context.node.get_logger().info('=== Actual Joint Positions ===')
    msg = ros_context.latest_joint_state
    for name, pos in zip(msg.name, msg.position):
        ros_context.node.get_logger().info(f'  {name}: {pos:.4f}')


def calibrate_tower() -> bool:
    """
    Trigger tower calibration via GPIO command.

    Returns:
        True if successful, False otherwise
    """
    request = GpioCommand.Request()
    request.command = 'c'
    request.value = 1

    ros_context.node.get_logger().info('Triggering tower calibration...')

    future = ros_context.arm_cmd_client.call_async(request)
    rclpy.spin_until_future_complete(ros_context.node, future)

    result = future.result()
    if result.success:
        ros_context.node.get_logger().info(f'Calibration successful: {result.message}')
    else:
        ros_context.node.get_logger().error(f'Calibration failed: {result.message}')

    return result.success


def lock_arms() -> bool:
    """
    Lock the arms using GPIO command.

    Returns:
        True if successful, False otherwise
    """
    request = GpioCommand.Request()
    request.command = 'l'
    request.value = 1

    ros_context.node.get_logger().info('Locking arms...')

    future = ros_context.arm_cmd_client.call_async(request)
    rclpy.spin_until_future_complete(ros_context.node, future)

    result = future.result()
    if result.success:
        ros_context.node.get_logger().info(f'Arms locked: {result.message}')
    else:
        ros_context.node.get_logger().error(f'Failed to lock arms: {result.message}')

    return result.success


def run_init_sequence() -> bool:
    """
    Run the full initialization sequence:
    1. Set all joints (j1=0, j2-j4=3.14 radians)
    2. Wait for arms to become idle
    3. Calibrate the tower
    4. Lock the arms
    5. Set joint 1 on both arms to 1.0 radians
    6. Display actual positions

    Returns:
        True if all steps successful, False otherwise
    """
    logger = ros_context.node.get_logger()
    logger.info('Starting arm initialization sequence...')

    # Step 1: Set joint 1 to 0 and other joints to 3.14 radians
    logger.info('Step 1: Setting joint 1 to 0, other joints to 3.14 radians')
    if not send_trajectory_request("START_ARMS", 0.0):
        logger.error('Failed to set initial joint positions.')
        return False
    
    # Step 2: Calibrate the tower
    logger.info('Step 2: Calibrating tower')
    if not calibrate_tower():
        logger.error('Failed to calibrate tower.')
        return False

    if not wait_for_arm_idle('tower', 't_s_moving'):
        logger.error('Tower did not become idle after calibration.')
        return False

    # Steo 3: Lock the arms after calibration
    logger.info('Locking arms after calibration')
    if not lock_arms():
        logger.error('Failed to lock arms.')
        return False

    if not wait_for_arm_idle('right_arm', 'r_s_moving'):
        logger.error('Right arm did not become idle after locking.')
        return False
    if not wait_for_arm_idle('left_arm', 'l_s_moving'):
        logger.error('Left arm did not become idle after locking.')
        return False

    # Step 4: Set joint 1 on both arms to 0.7 meters
    logger.info('Step 4: Setting joint 1 on both arms to 0.7 meters')
    if not send_trajectory_request("START_TOWERS", 0.0):
        logger.error('Failed to set initial joint positions.')
        return False

    if not wait_for_arm_idle('tower', 't_s_moving'):
        logger.error('Tower did not become idle after setting joint 1.')
        return False

    # Step 5: tuck the arms
    logger.info('Step 5: Tucking arms')
    if not send_trajectory_request("TUCK", 0.0):
        logger.error('Failed to tuck arms.')
        return False

    if not wait_for_arm_idle('tower', 't_s_moving'):
        logger.error('Tower did not become idle after tucking.')
        return False

    # Display actual joint positions
    display_actual_positions()

    logger.info('Initialization sequence completed successfully!')
    return True
