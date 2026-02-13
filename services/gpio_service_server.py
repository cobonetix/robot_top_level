import rclpy
from rclpy.node import Node
from control_msgs.msg import DynamicInterfaceGroupValues, InterfaceValue
from std_msgs.msg import Header
from cobonetix_interfaces.srv import GpioCommand
import random

class GpioServiceServer(Node):
    def __init__(self):
        super().__init__('gpio_service_server')

        # Publisher for GPIO commands
        self.publisher_ = self.create_publisher(
            DynamicInterfaceGroupValues,
            "/gpio_controller/commands",
            1
        )

        # Subscriber for GPIO states
        self.subscription = self.create_subscription(
            DynamicInterfaceGroupValues,
            '/gpio_controller/gpio_states',
            self.gpio_states_callback,
            1
        )

        # Create the service
        self.srv = self.create_service(
            GpioCommand,
            'gpio_command',
            self.gpio_command_callback
        )

        # Command mapping (same as cmd_kb.py)
        self.cmdToVariable = {
            "o": "t_c_valve_open",
            "p": "t_c_pump_on",
            "a": "t_c_auto_mode",
            "c": "t_c_calibrate",
            "r": "t_c_reset",

            "s": "r_c_servo",
            "m": "r_c_attach_mode",
            "v": "r_c_vacuum_on",
            "l": "r_c_lock",
            "e": "r_c_reset",

            "d": "l_c_door_position",
            "n": "l_c_lock",
            "f": "l_c_reset"
        }

        # Variable values
        self.variableToValue = {
            "t_c_pump_on": 0,
            "t_c_valve_open": 0,
            "t_c_auto_mode": 0,
            "t_c_calibrate": 0,
            "t_c_reset": 0,

            "r_c_servo": 0,
            "r_c_attach_mode": 0,
            "r_c_vacuum_on": 0,
            "r_c_lock": 0,
            "r_c_reset": 0,
            
            "l_c_door_position": 0,
            "l_c_lock": 0,
            "l_c_reset": 0
        }

        # Store latest GPIO states from subscription
        self.gpio_states = {}

        self.get_logger().info('GPIO service server is ready')
        self.get_logger().info(f'Available commands: {list(self.cmdToVariable.keys())}')

    def gpio_command_callback(self, request, response):
        cmd = request.command
        value = request.value

        # Validate command
        if cmd not in self.cmdToVariable:
            response.success = False
            response.message = f"Unknown command '{cmd}'. Valid: {list(self.cmdToVariable.keys())}"
            response.current_states = str(self.variableToValue)
            return response

        # Validate value (servo can be 0-180, others are 0 or 1)
        if cmd != 's' and value not in [0, 1]:
            response.success = False
            response.message = f"Value must be 0 or 1 for command '{cmd}'"
            response.current_states = str(self.variableToValue)
            return response

        if cmd == 's' and (value < 0 or value > 180):
            response.success = False
            response.message = "Servo value must be between 0 and 180"
            response.current_states = str(self.variableToValue)
            return response

        if cmd == 'c' or cmd == 'r':
            value = random.random()
        # Update value
        var = self.cmdToVariable[cmd]
        self.variableToValue[var] = value
        self.get_logger().info(f'Setting {var} to {value}')

        # Publish the command
        self.publish_gpio_values()

        response.success = True
        response.message = f"Set {var} to {value}"
        response.current_states = str(self.variableToValue)
        return response

    def gpio_states_callback(self, msg):
        # Parse and store GPIO states
        for i, group in enumerate(msg.interface_groups):
            if i < len(msg.interface_values):
                interface_value = msg.interface_values[i]
                state_dict = {}
                for j, name in enumerate(interface_value.interface_names):
                    if j < len(interface_value.values):
                        state_dict[name] = interface_value.values[j]
                self.gpio_states[group] = state_dict

    def publish_gpio_values(self):
        msg = DynamicInterfaceGroupValues()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.interface_groups = ["tower", "right_arm", "left_arm"]

        tower_value = InterfaceValue(
            interface_names=["t_c_pump_on", "t_c_auto_mode", "t_c_valve_open", "t_c_calibrate", "t_c_reset"],
            values=[
                float(self.variableToValue["t_c_pump_on"]),
                float(self.variableToValue["t_c_auto_mode"]),
                float(self.variableToValue["t_c_valve_open"]),
                float(self.variableToValue["t_c_calibrate"]),
                float(self.variableToValue["t_c_reset"])
]
        )

        right_arm_value = InterfaceValue(
            interface_names=["r_c_servo", "r_c_vacuum_on", "r_c_attach_mode", "r_c_lock", "r_c_reset"],
            values=[
                float(self.variableToValue["r_c_servo"]),
                float(self.variableToValue["r_c_vacuum_on"]),
                float(self.variableToValue["r_c_attach_mode"]),
                float(self.variableToValue["r_c_lock"]),
                float(self.variableToValue["r_c_reset"])
            ]
        )

        left_arm_value = InterfaceValue(
            interface_names=["l_c_door_position", "l_c_lock", "l_c_reset"],
            values=[
                float(self.variableToValue["l_c_door_position"])
              , float(self.variableToValue["l_c_lock"])
              , float(self.variableToValue["l_c_reset"])
            ]
        )

        msg.interface_values = [tower_value, right_arm_value, left_arm_value]

        self.publisher_.publish(msg)
        self.get_logger().info(f'Published GPIO values: {msg.interface_groups}')


def main(args=None):
    rclpy.init(args=args)
    server = GpioServiceServer()
    rclpy.spin(server)
    server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
