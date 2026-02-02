import rclpy
from rclpy.node import Node
from cobonetix_interfaces.srv import GpioStatus


class GpioStatusClient(Node):
    def __init__(self):
        super().__init__('gpio_status_client')
        self.client = self.create_client(GpioStatus, 'gpio_status')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for gpio_status service...')

        self.get_logger().info('Connected to gpio_status service')

        # Available status fields by group
        self.status_fields = {
            'tower': ['t_s_moving', 't_s_error', 't_s_pump_on', 't_s_auto_mode', 't_s_valve_open', 't_s_pressure'],
            'right_arm': ['r_s_moving', 'r_s_error', 'r_s_servo', 'r_s_vacuum_on', 'r_s_attach_mode', 'r_s_attach_status'],
            'left_arm': ['l_s_moving', 'l_s_error', 'l_s_door_position']
        }

    def send_request(self, group, field):
        request = GpioStatus.Request()
        request.group = group
        request.field = field
        future = self.client.call_async(request)
        return future

    def print_help(self):
        print("\nGPIO Status Query")
        print("=" * 40)
        print("Available groups and fields:")
        for group, fields in self.status_fields.items():
            print(f"\n  {group}:")
            for field in fields:
                print(f"    - {field}")
        print("\nUsage: <group> <field>")
        print("Example: tower moving")
        print("         right_arm error")
        print("Type 'q' to quit\n")


def main(args=None):
    rclpy.init(args=args)
    client = GpioStatusClient()

    client.print_help()

    while True:
        try:
            user_input = input("Enter group and field (or 'q' to quit): ").strip()

            if user_input.lower() == 'q':
                print("Exiting...")
                break

            parts = user_input.split()
            if len(parts) != 2:
                print("Invalid input. Enter: <group> <field>")
                client.print_help()
                continue

            group = parts[0]
            field = parts[1]

            print(f"Querying: {group}.{field} ...")

            future = client.send_request(group, field)
            rclpy.spin_until_future_complete(client, future)

            result = future.result()
            if result.success:
                print(f"Result: {result.value}")
                is_set = result.value != 0.0
                print(f"Status: {'SET' if is_set else 'NOT SET'}")
            else:
                print(f"Failed: {result.message}")
            print()

        except KeyboardInterrupt:
            print("\nExiting...")
            break

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
