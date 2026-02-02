import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class SimpleJointPublisher(Node):
    def __init__(self):
        super().__init__('simple_joint_publisher')
        self.publisher_l = self.create_publisher(Float32MultiArray, '/cobo_left_arm_command/joint', 10)
        self.publisher_r = self.create_publisher(Float32MultiArray, '/cobo_right_arm_command/joint', 10)
         

    def send_msg(self,lr,j1,j2,j3,j4):
        msg = Float32MultiArray()
        msg.data= [j1, j2, j3, j4]

        if lr == 'l':
          self.publisher_l.publish(msg)
        else:
          self.publisher_r.publish(msg)

        self.get_logger().info(f'Publishing Post: {lr} msg.data[0]: {msg.data[0]}, msg.data[1]: {msg.data[1]}, msg.data[2]: {msg.data[2]}, msg.data[3]: {msg.data[3]}')
        
    def getAndSendInput(self):
        while True:
            try:
                lr,j1, j2, j3, j4 = input("Enter l/r j1 j2 j3 j4: ").split()
                j1 = float (j1)
                j2 = float (j2)
                j3 = float (j3)
                j4 = float (j4)
 
                print("input: ",j1,j2,j3,j4)

                self.send_msg(lr,j1,j2,j3,j4)
            except ValueError:
                print("Invalid input. Please enter numeric values.")

def main(args=None):
    rclpy.init(args=args)
    sim_joint_pub = SimpleJointPublisher()
       
    sim_joint_pub.getAndSendInput()
    simple_twist_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()