#!/usr/bin/env python3
"""
ROS2 routine to initialize arm joints to 3.14 radians and then calibrate.
"""

import time
import rclpy
from rclpy.node import Node
from cobonetix_interfaces.srv import ArmJoint, GpioCommand
from sensor_msgs.msg import JointState


class ArmInitCalibrate(Node):
    """Node that sets arm joints and triggers calibration."""

    def __init__(self):
        super().__init__('arm_init_calibrate')

        # Create service clients
        self.joint_client = self.create_client(ArmJoint, 'arm_joint')
        self.gpio_client = self.create_client(GpioCommand, 'gpio_command')

        # Wait for services to be available
        self.get_logger().info('Waiting for arm_joint service...')
        while not self.joint_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('arm_joint service not available, waiting...')
        self.get_logger().info('Connected to arm_joint service')

        self.get_logger().info('Waiting for gpio_command service...')
        while not self.gpio_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('gpio_command service not available, waiting...')
        self.get_logger().info('Connected to gpio_command service')

        # Subscribe to joint states
        self._latest_joint_state = None
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10
        )

    def _joint_state_callback(self, msg: JointState):
        """Store the latest joint state message."""
        self._latest_joint_state = msg

    def set_all_joints(self, position: float):
        """
        Set all joints of both arms to the specified position.

        Args:
            position: Joint position in radians

        Returns:
            Result object with success status and actual positions
        """
        request = ArmJoint.Request()
        # Left arm joints
        request.l_j1 = position
        request.l_j2 = position
        request.l_j3 = position
        request.l_j4 = position
        # Right arm joints
        request.r_j1 = position
        request.r_j2 = position
        request.r_j3 = position
        request.r_j4 = position

        self.get_logger().info(
            f'Setting all joints to {position} rad: '
            f'left=[{position}, {position}, {position}, {position}], '
            f'right=[{position}, {position}, {position}, {position}]'
        )

        future = self.joint_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        if result.success:
            self.get_logger().info(f'Joint positioning successful: {result.message}')
        else:
            self.get_logger().error(f'Joint positioning failed: {result.message}')

        return result

    def set_joint1_both_arms(self, position: float):
        """
        Set joint 1 of both arms to the specified position.

        Args:
            position: Joint position in radians

        Returns:
            Result object with success status and actual positions
        """
        request = ArmJoint.Request()
        # Set joint 1 on both arms
        request.l_j1 = position
        request.r_j1 = position
        # Keep other joints at current position (3.14)
        request.l_j2 = 3.14
        request.l_j3 = 3.14
        request.l_j4 = 3.14
        request.r_j2 = 3.14
        request.r_j3 = 3.14
        request.r_j4 = 3.14

        self.get_logger().info(f'Setting joint 1 on both arms to {position} rad')

        future = self.joint_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        if result.success:
            self.get_logger().info(f'Joint 1 positioning successful: {result.message}')
        else:
            self.get_logger().error(f'Joint 1 positioning failed: {result.message}')

        return result

    def display_actual_positions(self):
        """Display actual joint positions from /joint_states topic."""
        # Spin briefly to receive latest joint state
        timeout = time.time() + 1.0
        while self._latest_joint_state is None and time.time() < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self._latest_joint_state is None:
            self.get_logger().warn('No joint state received')
            return

        self.get_logger().info('=== Actual Joint Positions ===')
        msg = self._latest_joint_state
        for name, pos in zip(msg.name, msg.position):
            self.get_logger().info(f'  {name}: {pos:.4f}')

    def calibrate_tower(self) -> bool:
        """
        Trigger tower calibration via GPIO command.

        Returns:
            True if successful, False otherwise
        """
        request = GpioCommand.Request()
        request.command = 'c'  # t_c_calibrate command
        request.value = 1      # Enable calibration

        self.get_logger().info('Triggering tower calibration...')

        future = self.gpio_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        if result.success:
            self.get_logger().info(f'Calibration successful: {result.message}')
        else:
            self.get_logger().error(f'Calibration failed: {result.message}')

        return result.success

    def run_init_sequence(self) -> bool:
        """
        Run the full initialization sequence:
        1. Set all joints to 3.14 radians
        2. Calibrate the tower
        3. Set joint 1 on both arms to 1.0 radians

        Returns:
            True if all steps successful, False otherwise
        """
        self.get_logger().info('Starting arm initialization sequence...')

        # Step 1: Set all joints to 3.14 radians
        self.get_logger().info('Step 1: Setting all joints to 3.14 radians')
        result = self.set_all_joints(3.14)
        if not result.success:
            self.get_logger().error('Failed to set joint positions. Aborting.')
            return False

        # Step 2: Calibrate the tower
        self.get_logger().info('Step 2: Calibrating tower')
        if not self.calibrate_tower():
            self.get_logger().error('Failed to calibrate tower.')
            return False

        # Step 3: Set joint 1 on both arms to 1.0 radians
        self.get_logger().info('Step 3: Setting joint 1 on both arms to 1.0 radians')
        result = self.set_joint1_both_arms(1.0)
        if not result.success:
            self.get_logger().error('Failed to set joint 1 positions.')
            return False

        # Display actual joint positions
        self.display_actual_positions()

        self.get_logger().info('Initialization sequence completed successfully!')
        return True


def main(args=None):
    rclpy.init(args=args)

    node = ArmInitCalibrate()

    try:
        success = node.run_init_sequence()
        if not success:
            node.get_logger().error('Initialization sequence failed')
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0 if success else 1


if __name__ == '__main__':
    exit(main())
