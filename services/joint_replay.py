import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import h5py
from cv_bridge import CvBridge
import cv2
import time
import numpy as np
from datetime import datetime
class SimpleJointPublisher(Node):
    def __init__(self):
        super().__init__('joint_replay_publisher')
  
        self.publisher_dual = self.create_publisher(Float32MultiArray, '/cobo_dual_arm_command/joint', 10)
         

    def send_msg(self,lj1,lj2,lj3,lj4,rj1,rj2,rj3,rj4):
        msg = Float32MultiArray()
        l1 =  [lj1,lj2,lj3,lj4,rj1,rj2,rj3,rj4]
        msg.data = [float(item) for item in l1]
        print(msg.data)
        self.publisher_dual.publish(msg)

        self.get_logger().info(f'Publishing Post:  lj1: {msg.data[0]}, lj2: {msg.data[1]}, lj3: {msg.data[2]}, lj4: {msg.data[3]}, rj1: {msg.data[4]}, rj2: {msg.data[5]}, rj3: {msg.data[6]}, rj4: {msg.data[7]}')
        
    def getAndSendInput(self):
        while True:
          filename = input("enter name of h5 file: ")
          
          if len(filename) == 0: return
          
          fd =  h5py.File(filename + '.h5', 'r')
          grps = list(fd.keys())
          print("Groups in the file: ", grps)

          while True:
            grp = input("enter group: ")
            if len(grp) == 0:
              break
          
            group = fd[grp]
            datasets = group.keys() 
            dslist = list(datasets)

            print("Datasets in the group: ", dslist)

            dset0 = group[dslist[0]]
            dset1 = group[dslist[1]]
            print(dset0)
            print(dset1)
        
            data = dset1[:]

            skip= input("enter skip factor: ")  
            if len(skip) == 0: 
               skip = 1
            else:
               skip = int(skip)
                        #
            for i in range(0,len(data),skip):
              time.sleep(1)
              self.send_msg(data[i][0],data[i][1],data[i][2],data[i][3],data[i][4],data[i][5],data[i][6],data[i][7] )
              #print(data[i])

          fd.close()
          print("File closed.")


def main(args=None):
    rclpy.init(args=args)
    sim_joint_replay_pub = SimpleJointPublisher()
       
    sim_joint_replay_pub.getAndSendInput()
    sim_joint_replay_pub = SimpleJointPublisher()
    sim_joint_replay_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()