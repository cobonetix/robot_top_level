import rclpy,random
from rclpy.node import Node
from control_msgs.msg import DynamicInterfaceGroupValues, InterfaceValue,DynamicJointState
from std_msgs.msg import Header
import h5py

class DynamicInterfacePublisher(Node):
    def __init__(self):
        super().__init__('gpioControllerTest')
        self.publisher_ = self.create_publisher(DynamicInterfaceGroupValues, "/gpio_controller/commands", 10)
        self.timer = self.create_timer(3.0, self.publish_dynamic_values)

        self.subscription = self.create_subscription(
            DynamicInterfaceGroupValues,
            '/gpio_controller/gpio_states',
            self.listener_callback,
            10)
        
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
         
        print(msg.interface_groups)
        #print(msg.interface_values)
        print(msg.interface_values[0].interface_names[0], msg.interface_values[0].values[0])   # tower
        print(msg.interface_values[0].interface_names[1], msg.interface_values[0].values[1])
        print(msg.interface_values[0].interface_names[2], msg.interface_values[0].values[2])
        print(msg.interface_values[0].interface_names[3], msg.interface_values[0].values[3])

        print(msg.interface_values[1].interface_names[0], msg.interface_values[0].values[0])   # right_arm
        
        print(msg.interface_values[2].interface_names[0], msg.interface_values[0].values[0])   # left_arm
        print(msg.interface_values[2].interface_names[1], msg.interface_values[0].values[1])
        print(msg.interface_values[2].interface_names[2], msg.interface_values[0].values[2])

         

    def publish_dynamic_values(self):
        msg = DynamicInterfaceGroupValues()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.interface_groups = ["tower", "right_arm", "left_arm"]
        
        tower_value_0 = InterfaceValue(interface_names=["c_valve_open","c_pump_on","c_pump_on"], values=[random.randint(0,1),random.randint(0,1),random.randint(0,1)] )
        right_arm_value_0 = InterfaceValue(interface_names=["c_servo","c_vacuum_on","c_attach_mode"], values=[random.randint(0,1),random.randint(0,1),random.randint(0,1)] )
        left_arm_value_0 = InterfaceValue(interface_names=["c_door_position"], values=[random.randint(0,1)] )
                                     
        msg.interface_values = [tower_value_0, right_arm_value_0, left_arm_value_0]
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published dynamic interface values: {msg.interface_groups}')

def main(args=None):
    rclpy.init(args=args)
    node = DynamicInterfacePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()