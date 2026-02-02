std_msgs/Header header

# for all arrays, left arm is 0, right arm is 1
# command bitmap
uint16 left_command_map 
uint16 right_command_map 

#status bitmap
uint16 left_status_map 
uint16 right_status_map 

# three arm angles
uint16[3] left_arm_joints 
uint16[3] right_arm_joints 

# image

#sensor_msgs/Image last_image
