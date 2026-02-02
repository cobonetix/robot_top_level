import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from control_msgs.msg import DynamicInterfaceGroupValues
from cobonetix_interfaces.srv import GpioStatus


class GpioStatusServer(Node):
    def __init__(self):
        super().__init__('gpio_status_server')

        # Use ReentrantCallbackGroup to allow concurrent callback execution
        self.callback_group = ReentrantCallbackGroup()

        # Create the service
        self.srv = self.create_service(
            GpioStatus,
            'gpio_status',
            self.gpio_status_callback,
            callback_group=self.callback_group
        )

        # QoS profile to not buffer messages - only keep latest
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Flag and storage for received message
        self.message_received = False
        self.latest_msg = None

        # Create subscription once at startup
        self.subscription = self.create_subscription(
            DynamicInterfaceGroupValues,
            '/gpio_controller/gpio_states',
            self.status_callback,
            qos,
            callback_group=self.callback_group
        )

        # Valid groups and their expected fields
        self.valid_groups = ['tower', 'right_arm', 'left_arm']

        self.get_logger().info('GPIO status server is ready')
        self.get_logger().info(f'Valid groups: {self.valid_groups}')

    def status_callback(self, msg):
        """Callback for gpio_states subscription."""
        self.latest_msg = msg
        self.message_received = True
 
    def gpio_status_callback(self, request, response):
        group = request.group
        field = request.field
        

        # Validate group
        if group not in self.valid_groups:
            response.success = False
            response.value = 0.0
            response.message = f"Invalid group '{group}'. Valid: {self.valid_groups}"
            return response

        # Reset flag to wait for fresh message
        self.message_received = False

        # Wait for a fresh message with timeout
        timeout_seconds = 5.0
        start_time = self.get_clock().now()

        self.get_logger().info(f'Waiting for gpio_states message...')

        while not self.message_received:
            time.sleep(0.1)
            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed > timeout_seconds:
                response.success = False
                response.value = 0.0
                response.message = 'Timeout waiting for gpio_states message'
                self.get_logger().warn('Timeout waiting for gpio_states')
                return response

        # Parse the message and find the requested field
        msg = self.latest_msg

        # Find the group index
        group_idx = -1
        for i, grp in enumerate(msg.interface_groups):
            if grp == group:
                group_idx = i
                break

        if group_idx == -1 or group_idx >= len(msg.interface_values):
            response.success = False
            response.value = 0.0
            response.message = f"Group '{group}' not found in message"
            return response

        # Find the field in the interface values
        interface_value = msg.interface_values[group_idx]
        field_idx = -1
        for j, name in enumerate(interface_value.interface_names):
            if name == field:
                field_idx = j
                break

        if field_idx == -1 or field_idx >= len(interface_value.values):
            available_fields = list(interface_value.interface_names)
            response.success = False
            response.value = 0.0
            response.message = f"Field '{field}' not found in '{group}'. Available: {available_fields}"
            return response

        # Return the value
        value = interface_value.values[field_idx]
        response.success = True
        response.value = value
        response.message = f"{group}.{field} = {value}"

        self.get_logger().info(f'Returning {group}.{field} = {value}')
        return response


def main(args=None):
    rclpy.init(args=args)
    server = GpioStatusServer()
    executor = MultiThreadedExecutor()
    executor.add_node(server)
    try:
        executor.spin()
    finally:
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
