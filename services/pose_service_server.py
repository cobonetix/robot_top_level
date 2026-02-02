import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Pose
from control_msgs.msg import DynamicInterfaceGroupValues
from cobonetix_interfaces.srv import ArmPose


class PoseServiceServer(Node):
    def __init__(self):
        super().__init__('pose_service_server')

        # Create separate callback groups to allow concurrent execution
        self.service_cb_group = MutuallyExclusiveCallbackGroup()
        self.subscription_cb_group = MutuallyExclusiveCallbackGroup()

        self.publisher_r = self.create_publisher(Pose, '/cobo_right_arm_command/pose', 10)
        self.publisher_l = self.create_publisher(Pose, '/cobo_left_arm_command/pose', 10)
        self.srv = self.create_service(
            ArmPose, 'arm_pose', self.arm_pose_callback,
            callback_group=self.service_cb_group
        )

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
            callback_group=self.subscription_cb_group
        )

        self.get_logger().info('Pose service server is ready')

    def arm_pose_callback(self, request, response):
        msg = Pose()
        msg.position.x = request.x
        msg.position.y = request.y
        msg.position.z = request.z
        msg.orientation.z = request.rotation

        # Publish the pose command
        if request.arm == 'l':
            self.publisher_l.publish(msg)
            self.get_logger().info(
                f'Publishing left arm Pose: x={msg.position.x}, y={msg.position.y}, '
                f'z={msg.position.z}, rotation={msg.orientation.z}'
            )
        else:
            self.publisher_r.publish(msg)
            self.get_logger().info(
                f'Publishing right arm Pose: x={msg.position.x}, y={msg.position.y}, '
                f'z={msg.position.z}, rotation={msg.orientation.z}'
            )

        # Determine which arm moving flag to check
        arm_moving_key = 'l_s_moving' if request.arm == 'l' else 'r_s_moving'
        arm_name = 'left_arm' if request.arm == 'l' else 'right_arm'

        # Wait for movement to complete (tower and arm both stopped)
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

            # Check if tower and arm have stopped moving
            tower_moving = self.gpio_states.get('tower', {}).get('t_s_moving', 1.0)
            arm_moving = self.gpio_states.get(arm_name, {}).get(arm_moving_key, 1.0)

            # Values are floats; 0.0 means not moving
            if tower_moving == 0.0 and arm_moving == 0.0:
                movement_complete = True
                self.get_logger().info(
                    f'Movement complete - tower and {arm_name} stopped'
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
        response.message = f'Pose published to {request.arm} arm, gpio_states: {self.gpio_states}'
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
    server = PoseServiceServer()
    executor = MultiThreadedExecutor()
    executor.add_node(server)
    try:
        executor.spin()
    finally:
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
