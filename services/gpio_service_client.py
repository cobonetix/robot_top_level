import rclpy
from rclpy.node import Node
from cobonetix_interfaces.srv import GpioCommand


class GpioServiceClient(Node):
    def __init__(self):
        super().__init__('gpio_service_client')
        self.client = self.create_client(GpioCommand, 'gpio_command')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for gpio_command service...')

        self.get_logger().info('Connected to gpio_command service')

        # Command help info
        self.commands = {
            'o': 't_c_valve_open (0/1)',
            'p': 't_c_pump_on (0/1)',
            'a': 't_c_auto_mode (0/1)',
            'c': 't_c_calibrate (0/1)',
            'r': 't_c_reset (0/1)',
            's': 'r_c_servo (0-180)',
            'm': 'r_c_attach_mode (0/1)',
            'v': 'r_c_vacuum_on (0/1)',
            'l': 'r_c_lock (0/1)',
            'e': 'r_c_reset (0/1)',
            'd': 'l_c_door_position (0/1)',
            'n': 'l_c_lock (0/1)',
            'f': 'l_c_reset (0/1)'
        }

    def send_request(self, command, value):
        request = GpioCommand.Request()
        request.command = command
        request.value = value
        future = self.client.call_async(request)
        return future

    def print_help(self):
        print("\nAvailable commands:")
        for cmd, desc in self.commands.items():
            print(f"  {cmd} - {desc}")
        print("  q - quit")
        print("\nUsage: <command> <value>")
        print("Example: p 1  (turn pump on)\n")


def main(args=None):
    rclpy.init(args=args)
    client = GpioServiceClient()

    client.print_help()

    while True:
        try:
            user_input = input("Enter command and value (or 'q' to quit): ").strip()

            if user_input.lower() == 'q':
                print("Exiting...")
                break

            parts = user_input.split()
            if len(parts) != 2:
                print("Invalid input. Enter: <command> <value>")
                client.print_help()
                continue

            command = parts[0]
            try:
                value = int(parts[1])
            except ValueError:
                print("Value must be an integer")
                continue

            print(f"Sending: command='{command}', value={value}")

            future = client.send_request(command, value)
            rclpy.spin_until_future_complete(client, future)

            result = future.result()
            print(f"Response: success={result.success}")
            print(f"Message: {result.message}")
            print(f"Current states: {result.current_states}\n")

        except KeyboardInterrupt:
            print("\nExiting...")
            break

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
