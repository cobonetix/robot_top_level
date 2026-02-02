import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

class SimplePosePublisher(Node):
    def __init__(self):
        super().__init__('simple_twist_publisher')
        self.publisher_r = self.create_publisher(Pose, '/cobo_right_arm_command/pose', 10)
        self.publisher_l = self.create_publisher(Pose, '/cobo_left_arm_command/pose', 10)
         

    def send_msg(self,lr,x,y,z,roll,pitch,yaw):
        msg = Pose()
        msg.position.x = x; # Position in x-direction
        msg.position.y = y; # Position in x-direction
        msg.position.z = z;  # Position in x-direction
        msg.orientation.z = roll  # Angular velocity around z-axis (rad/s
        if lr == 'l':
            self.publisher_l.publish(msg)
            self.get_logger().info(f'Publishing l Pose: msg.position.x: {msg.position.x}, msg.position.y: {msg.position.y}, msg.position.z: {msg.position.z}, msg.orientation.z: {msg.orientation.z}')
        else:
            self.publisher_r.publish(msg)
            self.get_logger().info(f'Publishing r Pose: msg.position.x: {msg.position.x}, msg.position.y: {msg.position.y}, msg.position.z: {msg.position.z}, msg.orientation.z: {msg.orientation.z}')  

    def getAndSendInput(self):
        while True:
            try:
                lr,x,y,z,r = input("Enter lr,x, y, z, r: ").split()
                x = float(x)
                y = float(y)
                z = float(z)
                roll = float(r)
                print("input: ",lr, x,y,z,roll)

                self.send_msg(lr,x, y, z, roll, 0, 0)
            except ValueError:
                print("Invalid input. Please enter numeric values.")

def main(args=None):
    rclpy.init(args=args)
    sim_pose_pub = SimplePosePublisher()
       
    sim_pose_pub.getAndSendInput()
    simple_twist_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()