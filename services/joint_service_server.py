import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32MultiArray
from control_msgs.msg import DynamicInterfaceGroupValues
from cobonetix_interfaces.srv import ArmJoint


class JointServiceServer(Node):
    def __init__(self):
        super().__init__('joint_service_server')

        # Create separate callback groups to allow concurrent execution
        self.service_cb_group = MutuallyExclusiveCallbackGroup()
        self.subscription_cb_group = MutuallyExclusiveCallbackGroup()

        self.publisher_l = self.create_publisher(Float32MultiArray, '/cobo_left_arm_command/joint', 10)
        self.publisher_r = self.create_publisher(Float32MultiArray, '/cobo_right_arm_command/joint', 10)
        self.srv = self.create_service(
            ArmJoint, 'arm_joint', self.arm_joint_callback,
            callback_group=self.service_cb_group)

        # Store latest gpio states
        self.gpio_states = {
            'tower': {},
            'right_arm': {},
            'left_arm': {}
        }

        # Flag to track if we received gpio_states response
        self.gpio_response_received = False

        # Subscribe to gpio_states topic once
        self.subscription = self.create_subscription(
            DynamicInterfaceGroupValues,
            '/gpio_controller/gpio_states',
            self.listener_callback,
            10,
            callback_group=self.subscription_cb_group)

        self.get_logger().info('Joint service server is ready')

    def arm_joint_callback(self, request, response):
        # Check which arms should be moved (skip if all joints are zero)
        left_joints = [request.l_j1, request.l_j2, request.l_j3, request.l_j4]
        right_joints = [request.r_j1, request.r_j2, request.r_j3, request.r_j4]
        move_left = not all(j == 0.0 for j in left_joints)
        move_right = not all(j == 0.0 for j in right_joints)

        # Publish joint commands only to arms that should move
        if move_left:
            msg_left = Float32MultiArray()
            msg_left.data = [float(j) for j in left_joints]
            self.publisher_l.publish(msg_left)
            self.get_logger().info(
                f'Publishing left arm Joint: j1={request.l_j1}, j2={request.l_j2}, '
                f'j3={request.l_j3}, j4={request.l_j4}'
            )
        else:
            self.get_logger().info('Skipping left arm (all joints are zero)')

        if move_right:
            msg_right = Float32MultiArray()
            msg_right.data = [float(j) for j in right_joints]
            self.publisher_r.publish(msg_right)
            self.get_logger().info(
                f'Publishing right arm Joint: j1={request.r_j1}, j2={request.r_j2}, '
                f'j3={request.r_j3}, j4={request.r_j4}'
            )
        else:
            self.get_logger().info('Skipping right arm (all joints are zero)')

        # If no arms are moving, return immediately
        if not move_left and not move_right:
            response.success = True
            response.message = 'No arms commanded to move (all joints zero)'
            return response

        # Wait for movement to complete (tower and commanded arms stopped)
        timeout_sec = 30.0
        poll_interval = 0.1
        start_time = self.get_clock().now()
        movement_complete = False

        while not movement_complete:
            # Sleep and let the executor handle subscription callbacks
            time.sleep(poll_interval)

            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed > timeout_sec:
                self.get_logger().warn('Timeout waiting for movement to complete')
                break

            # Check if tower has stopped moving
            tower_moving = self.gpio_states.get('tower', {}).get('t_s_moving', 1.0)

            # Only check arms that were commanded to move
            left_arm_moving = self.gpio_states.get('left_arm', {}).get('l_s_moving', 1.0) if move_left else 0.0
            right_arm_moving = self.gpio_states.get('right_arm', {}).get('r_s_moving', 1.0) if move_right else 0.0

            # Values are floats; 0.0 means not moving
            if tower_moving == 0.0 and left_arm_moving == 0.0 and right_arm_moving == 0.0:
                movement_complete = True
                arms_moved = []
                if move_left:
                    arms_moved.append('left')
                if move_right:
                    arms_moved.append('right')
                self.get_logger().info(
                    f'Movement complete - tower and {" and ".join(arms_moved)} arm(s) stopped'
                )

        # Log parsed states (moving and error only)
        if 'tower' in self.gpio_states and self.gpio_states['tower']:
            tower = self.gpio_states['tower']
            self.get_logger().info(
                f"Tower - moving: {tower.get('t_s_moving', 'N/A')}, "
                f"error: {tower.get('error', 'N/A')}"
            )

        if 'right_arm' in self.gpio_states and self.gpio_states['right_arm']:
            right_arm = self.gpio_states['right_arm']
            self.get_logger().info(
                f"Right Arm - moving: {right_arm.get('r_s_moving', 'N/A')}, "
                f"error: {right_arm.get('error', 'N/A')}"
            )

        if 'left_arm' in self.gpio_states and self.gpio_states['left_arm']:
            left_arm = self.gpio_states['left_arm']
            self.get_logger().info(
                f"Left Arm - moving: {left_arm.get('l_s_moving', 'N/A')}, "
                f"error: {left_arm.get('error', 'N/A')}"
            )

        response.success = True
        arms_moved = []
        if move_left:
            arms_moved.append('left')
        if move_right:
            arms_moved.append('right')
        response.message = f'Joint command published to {" and ".join(arms_moved)} arm(s), gpio_states: {self.gpio_states}'
        return response

    def listener_callback(self, msg):
        # Parse gpio_states message - only extract moving and error fields
        # Field names are prefixed per component: t_s_moving, r_s_moving, l_s_moving, etc.
        for i, group in enumerate(msg.interface_groups):
            if i < len(msg.interface_values):
                interface_value = msg.interface_values[i]
                state_dict = {}
                for j, name in enumerate(interface_value.interface_names):
                    if j < len(interface_value.values) and ('moving' in name or 'error' in name):
                        state_dict[name] = interface_value.values[j]
                self.gpio_states[group] = state_dict

        # Mark that we received the response
        self.gpio_response_received = True

    def get_gpio_states(self):
        """Return the latest gpio states for external access."""
        return self.gpio_states


def main(args=None):
    rclpy.init(args=args)
    server = JointServiceServer()
    executor = MultiThreadedExecutor()
    executor.add_node(server)
    try:
        executor.spin()
    finally:
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
