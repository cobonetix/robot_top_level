#!/usr/bin/env python3
"""
ROS2 routine to initialize arm joints to 3.14 radians and then calibrate.
"""

import time
import rclpy
from rclpy.node import Node
from cobonetix_interfaces.srv import ArmJoint, GpioCommand, GpioStatus
from sensor_msgs.msg import JointState

class ArmInitCalibrate(Node):
    """Node that sets arm joints and triggers calibration."""

    def __init__(self):
        super().__init__('arm_init_calibrate')

        # Create service clients
        self.joint_client = self.create_client(ArmJoint, '/arm_joint')
        self.gpio_client = self.create_client(GpioCommand, '/gpio_command')
        self.gpio_status_client = self.create_client(GpioStatus, '/gpio_status')

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

    def set_all_joints(self, j1_position: float = 1.0, other_position: float = 3.14):
        """
        Set all joints of both arms.

        Args:
            j1_position: Joint 1 position in radians (default 1.0)
            other_position: Position for joints 2, 3, 4 in radians (default 3.14)

        Returns:
            Result object with success status and actual positions
        """
        request = ArmJoint.Request()
        # Left arm joints
        request.l_j1 = j1_position
        request.l_j2 = other_position
        request.l_j3 = other_position
        request.l_j4 = other_position
        # Right arm joints
        request.r_j1 = j1_position
        request.r_j2 = other_position
        request.r_j3 = other_position
        request.r_j4 = other_position

        self.get_logger().info(
            f'Setting joints: j1={j1_position} rad, j2-j4={other_position} rad: '
            f'left=[{j1_position}, {other_position}, {other_position}, {other_position}], '
            f'right=[{j1_position}, {other_position}, {other_position}, {other_position}]'
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
        # Set other joints to zero (no change to their last settings)
        request.l_j2 = 0.0
        request.l_j3 = 0.0
        request.l_j4 = 0.0
        request.r_j2 = 0.0
        request.r_j3 = 0.0
        request.r_j4 = 0.0

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

    def wait_for_gpio_idle(self, group: str = 'tower', field: str = 't_s_moving',
                          timeout: float = 30.0, poll_interval: float = 0.5) -> bool:
        """
        Poll the gpio_status service until the specified field reports idle (0.0).

        Args:
            group: GPIO group to query (e.g. 'tower', 'right_arm', 'left_arm')
            field: Status field to check (e.g. 't_s_moving')
            timeout: Maximum time to wait in seconds
            poll_interval: Time between polls in seconds

        Returns:
            True if idle was reached, False on timeout or error
        """
        self.get_logger().info(f'Waiting for {group}.{field} to become idle...')
        start = time.time()

        while (time.time() - start) < timeout:
            request = GpioStatus.Request()
            request.group = group
            request.field = field

            future = self.gpio_status_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)

            result = future.result()
            if result is None:
                self.get_logger().warn('gpio_status call returned None')
        #    elif not result.success:
        #        self.get_logger().warn(f'gpio_status query failed: {result.message}')
            elif result.value == 0.0:
                self.get_logger().info(f'{group}.{field} is idle')
                return True
            else:
                self.get_logger().debug(f'{group}.{field} = {result.value}, still busy...')

            time.sleep(poll_interval)

        self.get_logger().error(f'Timeout waiting for {group}.{field} to become idle')
        return False

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

    def lock_arms(self) -> bool:
        """
        Lock the arms using GPIO command.

        Returns:
            True if successful, False otherwise
        """
        request = GpioCommand.Request()
        request.command = 'l'  # lock command
        request.value = 1      # Enable lock

        self.get_logger().info('Locking arms...')

        future = self.gpio_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        if result.success:
            self.get_logger().info(f'Arms locked: {result.message}')
        else:
            self.get_logger().error(f'Failed to lock arms: {result.message}')

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

        # Step 1: Set joint 1 to 0 and other joints to 3.14 radians
        self.get_logger().info('Step 1: Setting joint 1 to 0, other joints to 3.14 radians')
        result = self.set_all_joints(0.0, 3.14)
        if not result.success:
            self.get_logger().error('Failed to set joint positions. Aborting.')
            return False

        if not self.wait_for_gpio_idle('right_arm', 'r_s_moving'):
            self.get_logger().error('Right arm did not become idle after joint positioning.')
            return False
        
        if not self.wait_for_gpio_idle('left_arm', 'l_s_moving'):
            self.get_logger().error('Left arm did not become idle after joint positioning.')
            return False

        # Step 2: Calibrate the tower
        self.get_logger().info('Step 2: Calibrating tower')
        if not self.calibrate_tower():
            self.get_logger().error('Failed to calibrate tower.')
            return False

        if not self.wait_for_gpio_idle('tower', 't_s_moving'):
            self.get_logger().error('Tower did not become idle after calibration.')
            return False

        # Lock the arms after calibration
        self.get_logger().info('Locking arms after calibration')
        if not self.lock_arms():
            self.get_logger().error('Failed to lock arms.')
            return False

        if not self.wait_for_gpio_idle('right_arm', 'r_s_moving'):
            self.get_logger().error('Right arm did not become idle after locking.')
            return False
        if not self.wait_for_gpio_idle('left_arm', 'l_s_moving'):
            self.get_logger().error('Left arm did not become idle after locking.')
            return False

        # Step 3: Set joint 1 on both arms to 1.0 radians
        self.get_logger().info('Step 3: Setting joint 1 on both arms to 1.0 radians')
        result = self.set_joint1_both_arms(1.0)
        if not result.success:
            self.get_logger().error('Failed to set joint 1 positions.')
            return False

        if not self.wait_for_gpio_idle('tower', 't_s_moving'):
            self.get_logger().error('Tower did not become idle after setting joint 1.')
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
