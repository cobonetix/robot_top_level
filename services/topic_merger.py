import rclpy,random
from rclpy.node import Node
from control_msgs.msg import DynamicInterfaceGroupValues, InterfaceValue,DynamicJointState
from std_msgs.msg import Header,Int8
from sensor_msgs.msg import Image,JointState
import h5py
from cv_bridge import CvBridge
import cv2
import time
import numpy as np
from datetime import datetime

CAPTURE_STOP_SAVE   = 1
CAPTURE_STOP_NOSAVE = 2
CAPTURE_START = 3

CAPTURE_IDLE = 0
CAPTURE_ACTIVE = 1
CAPTURE_SAVE  = 2

captureState = CAPTURE_IDLE # 0 no capture, 1 capture, 2 save and stop

daIndx = 0
dataSetIndex = 100


class DynamicInterfacePublisher(Node):
    def __init__(self):
        super().__init__('gpioControllerTest')

        self.stateSize = 500
         
        self.cmdStates = np.empty([self.stateSize, 3],   # 3 16 for status and cmds, 8 float for joints
         dtype = np.uint16,order='f')

        self.jointStates = np.empty([self.stateSize, 8],   # 3 16 for status and cmds, 8 float for joints
         dtype = np.float32,order='f')

        #daIndx = 0
        self.bridge = CvBridge()

        self.hf = h5py.File('training_data.h5', 'w')
 

        self.gpioStates = self.create_subscription(
            DynamicInterfaceGroupValues,
            '/gpio_controller/gpio_states',
            self.gpio_states_callback,
            1)
        
        self.gpioCommands = self.create_subscription(
            DynamicInterfaceGroupValues,
            '/gpio_controller/commands',
            self.gpio_command_callback,
            1)
        
        self.joints = self.create_subscription(
            JointState,
            '/joint_states',
            self.joints_callback,
            1)
        
        self.video = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            1)
       
        self.video = self.create_subscription(
            Int8,
            '/cobo_capture_mode',
            self.capture_mode_callback,
            1)
        
        self.last_gpio_state_msg = None
        self.last_gpio_cmd_msg = None
        self.last_image_msg = None 
        self.imageList = []
        self.saveImages = False
        self.jointValues = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]


        self.jointList = [
          "left_T_J_S0_T_TO_U_SL",
          "left_A_J_S1_U_SL_TO_A_L0",
          "left_A_J_S1_U_A_L0_TO_L1",
          "left_A_J_S1_U_A_L1_TO_L2_B1",
          "right_T_J_S0_T_TO_U_SL",
          "right_A_J_S1_U_SL_TO_A_L0",
          "right_A_J_S1_U_A_L0_TO_L1",
          "right_A_J_S1_U_A_L1_TO_L2_B1"]
         
    def addToFile (self,msg, gpio_cmd_msg, gpio_state_msg, image_msg):
        global captureState,  daIndx, dataSetIndex
        
        s1 = 0
        s2 = 0
        c = 0
      
        # print out gpio state msgs
        # 
        if not gpio_state_msg is None:
          '''
          print("state")         
          print(gpio_state_msg.interface_values[0].interface_names[0], gpio_state_msg.interface_values[0].values[0])   # tower
          print(gpio_state_msg.interface_values[0].interface_names[1], gpio_state_msg.interface_values[0].values[1])
          print(gpio_state_msg.interface_values[0].interface_names[2], gpio_state_msg.interface_values[0].values[2])
          print(gpio_state_msg.interface_values[0].interface_names[3], gpio_state_msg.interface_values[0].values[3])

          print(gpio_state_msg.interface_values[1].interface_names[0], gpio_state_msg.interface_values[0].values[0])   # left_arm

          print(gpio_state_msg.interface_values[2].interface_names[0], gpio_state_msg.interface_values[2].values[0])   # right_arm
          print(gpio_state_msg.interface_values[2].interface_names[1], gpio_state_msg.interface_values[2].values[1])
          print(gpio_state_msg.interface_values[2].interface_names[2], gpio_state_msg.interface_values[2].values[2]) 
          print(gpio_state_msg.interface_values[2].interface_names[3], gpio_state_msg.interface_values[2].values[3])
          print(gpio_state_msg.interface_values[2].interface_names[4], gpio_state_msg.interface_values[2].values[4])
          '''

          # these status items are  single bit and are in the lower byte with servo position in upper byte
          s1 =  ((int(gpio_state_msg.interface_values[0].values[0]) << 0) |    #ST_TOWER_PUMP_ON
                 (int(gpio_state_msg.interface_values[0].values[1]) << 1) |    #ST_TOWER_AUTO_MODE
                 (int(gpio_state_msg.interface_values[0].values[2]) << 2) |    #ST_TOWER_VALVE_OPEN
                 (int(gpio_state_msg.interface_values[2].values[0]) << 3) |    #ST_RIGHT_ARM_VACUUM_ON
                 (int(gpio_state_msg.interface_values[2].values[1]) << 4) |    #ST_RIGHT_ARM_ATTACH_MODE
                 (int(gpio_state_msg.interface_values[2].values[3]) << 5) |    #ST_RIGHT_ARM_ATTACH_STATUS 
                 (int(gpio_state_msg.interface_values[1].values[0]) << 6) |    #ST_LEFT_DOOR_POSITION
                 (int(gpio_state_msg.interface_values[2].values[0]) << 8) )     #ST_RIGHT_ARM_SERVO (8 bit)
          
          
          # these two status items are pressure values and analog. pack into two bytes
          s2 = ((int(gpio_state_msg.interface_values[0].values[3] )  )    |    #ST_TOWER_TANK_PRESSURE
                (int(gpio_state_msg.interface_values[2].values[4] ) << 8 ))     #ST_RIGHT_ARM_VACUUM_LEVEL
          
          #print("s1 ", hex(s1))
          # print("s2 ", hex(s2)) 
       
        if not gpio_cmd_msg is None:
          #print("cmds")
          '''
          print(gpio_cmd_msg.interface_values[0].interface_names[0], gpio_cmd_msg.interface_values[0].values[0])   # tower
          print(gpio_cmd_msg.interface_values[0].interface_names[1], gpio_cmd_msg.interface_values[0].values[1])
          print(gpio_cmd_msg.interface_values[0].interface_names[2], gpio_cmd_msg.interface_values[0].values[2])
          
          print(gpio_cmd_msg.interface_values[1].interface_names[0], gpio_cmd_msg.interface_values[1].values[0])   # right_arm
          print(gpio_cmd_msg.interface_values[1].interface_names[1], gpio_cmd_msg.interface_values[1].values[1])
          print(gpio_cmd_msg.interface_values[1].interface_names[2], gpio_cmd_msg.interface_values[1].values[2]) 
          
          print(gpio_cmd_msg.interface_values[2].interface_names[0], gpio_cmd_msg.interface_values[2].values[0])   # left_arm
          '''

          c = ( (int(gpio_cmd_msg.interface_values[0].values[0]) << 0) |    #CMD_TOWER_PUMP_ON    
                (int(gpio_cmd_msg.interface_values[0].values[1]) << 1) |    #CMD_TOWER_AUTO_MODE
                (int(gpio_cmd_msg.interface_values[0].values[2]) << 2) |    #CMD_TOWER_VALVE_OPEN               
                (int(gpio_cmd_msg.interface_values[1].values[1]) << 3) |    #CMD_RIGHT_ARM_VACUUM_ON
                (int(gpio_cmd_msg.interface_values[1].values[2]) << 4) |    #CMD_RIGHT_ARM_ATTACH_MODE
                (int(gpio_cmd_msg.interface_values[2].values[0]) << 5) |    #CMD_LEFT_DOOR_POSITION
                (int(gpio_cmd_msg.interface_values[1].values[0]) << 8) )    #CMD_RIGHT_ARM_SERVO (8 bit)
          #print("c ", hex(c))

        #print("joints")
        #print (msg)
        
        try:
          for i in range(len(msg.name)):
            index = self.jointList.index(msg.name[i])
            #print(i,index,msg.name[i])
            self.jointValues[index] = msg.position[i]     
            
        except ValueError:
          print("unknown joint " ,msg.name[i])
                
        #print (self.jointValues)

        #print("seq: ", daIndx) 

        if daIndx < self.stateSize:

          self.cmdStates[daIndx][0] = s1
          self.cmdStates[daIndx][1] = s2
          self.cmdStates[daIndx][2] = c  
          self.jointStates[daIndx][0] = (self.jointValues[0]) 
          self.jointStates[daIndx][1] = (self.jointValues[1])
          self.jointStates[daIndx][2] = (self.jointValues[2])
          self.jointStates[daIndx][3] = (self.jointValues[3])
          self.jointStates[daIndx][4] = (self.jointValues[4])
          self.jointStates[daIndx][5] = (self.jointValues[5])
          self.jointStates[daIndx][6] = (self.jointValues[6] )
          self.jointStates[daIndx][7] = (self.jointValues[7] ) 
  
          if not image_msg is None:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
            resized_image = cv2.resize(cv_image, (640 , 480), interpolation=cv2.INTER_AREA)
            flipped_vertical = cv2.flip(resized_image, 0)
            self.imageList.append(flipped_vertical)
            #print("image count ", len(self.imageList))


          #print(daIndx, self.cmdStates[daIndx])
          #print(self.jointStates[daIndx])

          daIndx += 1
        
        if captureState == CAPTURE_SAVE:
       
        #  print (self.cmdStates )
        #  print (self.jointstates )

          grpName = str(dataSetIndex)
          dataSetIndex += 1

          grp = self.hf.create_group(grpName)
          
          grp.create_dataset("cmds_and_states",  data=self.cmdStates[0:daIndx-1])
          grp.create_dataset("joint_states",     data=self.jointStates[0:daIndx-1])
          
          if self.saveImages:
             grp.create_dataset("images",      data=np.array(self.imageList))

          self.hf.flush()
          print("created group ", grpName, " with ", daIndx, " entries")
          captureState = CAPTURE_IDLE
          self.imageList = []
          daIndx = 0


           

    def gpio_states_callback(self, msg):
         
        #print(msg.interface_groups)
        #print(msg.interface_values)
        
        self.last_gpio_state_msg = msg  
        
    def gpio_command_callback(self, msg): 
        print(msg)
        self.last_gpio_cmd_msg = msg
        
    def joints_callback(self, msg):
        global captureState, daIndx
        #print(msg)
        if captureState != CAPTURE_IDLE:
          self.addToFile (msg, self.last_gpio_cmd_msg, self.last_gpio_state_msg, self.last_image_msg)
         
    def image_callback(self, msg):
        #print("got image")
        if self.saveImages:
           self.last_image_msg = msg
        
            
    def capture_mode_callback(self, msg):
      global captureState, daIndx

      if (msg.data ==   CAPTURE_START) and (captureState == CAPTURE_IDLE ):
        print("start capture")
        captureState = CAPTURE_ACTIVE  # start capture   
        daIndx = 0
        self.imageList = []

      elif (msg.data == CAPTURE_STOP_NOSAVE) and (captureState == CAPTURE_ACTIVE):
        print("stop capture no save")
        daIndx = 0
        self.imageList = []
        captureState = CAPTURE_IDLE # stop capture and toss

      elif (msg.data == CAPTURE_STOP_SAVE) and (captureState == CAPTURE_ACTIVE):
        print("stop capture and  save")
        captureState = CAPTURE_SAVE  # stop  and save capture



def main(args=None):
    rclpy.init(args=args)
    node = DynamicInterfacePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()