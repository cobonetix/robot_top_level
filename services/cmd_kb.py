import rclpy,random
from rclpy.node import Node
from control_msgs.msg import DynamicInterfaceGroupValues, InterfaceValue,DynamicJointState
from std_msgs.msg import Header,String
import h5py

class KeyboardInterfacePublisher(Node):
    def __init__(self):
        super().__init__('gpioControllerTest')
        self.publisher_ = self.create_publisher(DynamicInterfaceGroupValues, "/gpio_controller/commands", 10)
       
        self.subscription = self.create_subscription(
            DynamicInterfaceGroupValues,
            '/gpio_controller/gpio_states',
            self.listener_callback,
            10)
        
        self.subscription = self.create_subscription(
            String,
            '/cobo/command_input',
            self.command_callback,
            10)
        

        self.cmdToVariable = my_dict = {"o": "t_c_valve_open", "p": "t_c_pump_on", "a": "t_c_auto_mode", "c": "t_c_calibrate",
                                   "s": "r_c_servo", "m":"r_c_attach_mode",  "v":"r_c_vacuum_on",  \
                                   "d": "l_c_door_position",  "q": "q"}

        self.variableToValue = {"t_c_pump_on":0,"t_c_valve_open":0,  "t_c_auto_mode":0, "t_c_calibrate":0, 
                                "r_c_servo":0, "r_c_attach_mode":0, "r_c_vacuum_on":0,    
                                "l_c_door_position":0}


        self.subscription  # prevent unused variable warning

    def handleCommand(self, cmd_input):
          
          cmd_items  = [s.strip() for s in cmd_input.split(" ")]      
  
          if not cmd_items[0] in  self.cmdToVariable:
              print("unknown command")
              return 0
                   
          
          if cmd_items[0] == "q":
            print ("exiting")
            return 1
          
          if len(cmd_items) != 2:
              print("need 2 items")
              return 0
           
          if (cmd_items[0] != 's') and (not cmd_items[1] in ["0","1"]):
              print("2nd item must be 0 or 1")
              return 0
                   
          var = self.cmdToVariable[cmd_items[0]]
          print ("setting ", var, " to ", cmd_items[1] )
          self.variableToValue[var] = int(cmd_items[1])
          #print(self.variableToValue)
          self.publish_keyboard_values()
     
    def getCommands(self):

      while True:
          cmd_input = input("enter ( cmd, state ) or q: ")  
          if self.handleCommand(cmd_input) == 1:
              break
    
    def command_callback(self, msg):    
        print("command input" ,msg.data)
        self.handleCommand(msg.data)
    
    def listener_callback(self, msg):
        return
         
        print(msg.interface_groups)
        print(msg.interface_values)
        print(msg.interface_values[0].interface_names[0], msg.interface_values[0].values[0])   # tower moving
        print(msg.interface_values[0].interface_names[1], msg.interface_values[0].values[1])   # tower error
        print(msg.interface_values[0].interface_names[2], msg.interface_values[0].values[2])   # tower pump on
        print(msg.interface_values[0].interface_names[3], msg.interface_values[0].values[3])   # tower auto mode
        print(msg.interface_values[0].interface_names[4], msg.interface_values[0].values[4])   # tower valve open
        print(msg.interface_values[0].interface_names[5], msg.interface_values[0].values[5])   # tower pressure

        print(msg.interface_values[1].interface_names[0], msg.interface_values[0].values[0])   # right arm  moving
        print(msg.interface_values[1].interface_names[1], msg.interface_values[0].values[1])   # right arm  error
        print(msg.interface_values[1].interface_names[2], msg.interface_values[0].values[2])   # right_arm servo
        print(msg.interface_values[1].interface_names[3], msg.interface_values[0].values[3])   # right_arm v acuum on
        print(msg.interface_values[1].interface_names[4], msg.interface_values[0].values[4])   # right_arm attach mode
        print(msg.interface_values[1].interface_names[5], msg.interface_values[0].values[5])   # right_arm  attach status
        
        print(msg.interface_values[2].interface_names[0], msg.interface_values[0].values[0])   # left arm  moving
        print(msg.interface_values[2].interface_names[1], msg.interface_values[0].values[1])   # left arm  error
        print(msg.interface_values[2].interface_names[2], msg.interface_values[0].values[2])   # left_arm  door position
         

         

    def publish_keyboard_values(self):
        msg = DynamicInterfaceGroupValues()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.interface_groups = ["tower", "right_arm", "left_arm"]
        
        tower_value_0 = InterfaceValue(interface_names=["t_c_pump_on","t_c_auto_mode","t_c_valve_open", "t_c_calibrate"], \
                                        values=[self.variableToValue["t_c_pump_on"],self.variableToValue["t_c_auto_mode"],\
                                                self.variableToValue["t_c_valve_open"], self.variableToValue["t_c_calibrate"]] )

        right_arm_value_0 = InterfaceValue(interface_names=["r_c_servo","r_c_vacuum_on","r_c_attach_mode"],  \
                                        values=[self.variableToValue["r_c_servo"],self.variableToValue["r_c_vacuum_on"],self.variableToValue["r_c_attach_mode"]] )

        left_arm_value_0 = InterfaceValue(interface_names=["l_c_door_position"], values=[self.variableToValue["l_c_door_position"] ])
                                     
        msg.interface_values = [tower_value_0, right_arm_value_0, left_arm_value_0]
        
        print (msg)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published keyboard interface values: {msg.interface_groups}')

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardInterfacePublisher()
    node.getCommands()
    #rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()