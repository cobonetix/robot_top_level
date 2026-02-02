/**
Software License Agreement (BSD)

\authors   Mike Purvis <mpurvis@clearpathrobotics.com>
\copyright Copyright (c) 2014, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <cinttypes>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>

#include "rcutils/logging_macros.h"

#include "arm_pose_joy/arm_twist_joy.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/int8.hpp>
#include "std_msgs/msg/float32_multi_array.hpp"

//#include "/home/bob/ros2_ws/install/control_msgs/msg/dynamic_joint_state.hpp"



#define ROS_INFO_NAMED RCUTILS_LOG_INFO_NAMED
#define ROS_INFO_COND_NAMED RCUTILS_LOG_INFO_EXPRESSION_NAMED

double deg2rad(double deg)
{
  return deg * M_PI / 180.0;
}

double rad2deg(double rad)
{
  return rad * 180.0 / M_PI;
}
namespace arm_pose_joy
{


   enum CONTROLLING {LEFT_ARM, RIGHT_ARM, END_EFFECTOR, NONE};
  float axes_scale_map[4] = {0.1, 0.1, 0.1, 0.1}; // scale for each axes

/**
 * Internal members of class. This is the pimpl idiom, and allows more flexibility in adding
 * parameters later without breaking ABI compatibility, for robots which link ArmPoseJoy
 * directly into base nodes.
 */
struct ArmPoseJoy::Impl
{
  void poseCallback(const sensor_msgs::msg::JointState::SharedPtr joint_state);
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy);
  double getPosAdj(double currentPos, const sensor_msgs::msg::Joy::SharedPtr joy_msg, int64_t axejoy_msg_idx,int64_t axes);
 
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr pose_sub;

  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr cmd_pos_pub_left;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr cmd_pos_pub_right;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr cmd_pos_pub_ee;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr captureMode;
  //::msg::Int8 captureMsg;

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr left_joint_command_joint_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr right_joint_command_joint_pub_;
 
  
  rclcpp::Clock::SharedPtr clock;
  geometry_msgs::msg::Pose lastPoseLeft;   
  geometry_msgs::msg::Pose lastPoseRight;
  geometry_msgs::msg::Pose targetPoseLeft;   
  geometry_msgs::msg::Pose targetPoseRight;
  geometry_msgs::msg::Pose pose_temp;

  
  int64_t x_move;
  int64_t y_move;
  int64_t z_move;
  int64_t ee_move;
  int64_t ee_rot;
 
  int64_t right_arm_select;
  int64_t left_arm_select;
  int64_t ee_select;
  int64_t drive_mode;
  int64_t arm_stop;

  int64_t captureStart;
  int64_t captureStopSave;
  int64_t captureStopNoSave;

  int64_t leftArmUpDown;
  int64_t rightArmUpDown;

  
  float lastEEPos = 0.0; // last end effector position

  float L1_SIZE = 0.1905;  // length of first arm segment
  float L2_SIZE = 0.1905;  // length of second arm segment

  CONTROLLING controlling = NONE; 
  int leftArmBusy;
  int rightArmBusy;

};


 
 void ArmPoseJoy::Impl::poseCallback(const sensor_msgs::msg::JointState::SharedPtr js)
 {
    // calculates pose from the pose message. Note that we may get callbacks for a variety of sources
    // and so make sure we are getting only the right or left arm ones. 
    double x1, y1;


    if (js->name.size() != 4)
    {
        return;
    } 
    RCLCPP_INFO(
       rclcpp::get_logger("arm_pose_joy"),  "js: %f %f %f : %f",
       js->position[0], rad2deg(js->position[1]), rad2deg(js->position[2]), rad2deg(js->position[3])  
      );

    if (js->name[0][0] == 'l' || js->name[0][0] == 'L')
    {
      x1 = L1_SIZE * cos(js->position[1]);
      y1 = L1_SIZE * sin(js->position[1]);

      // calculate left arm current pose
      lastPoseLeft.position.z = js->position[0];
      lastPoseLeft.position.x = x1 + L2_SIZE * cos(js->position[1] + js->position[2] -1.57);
      lastPoseLeft.position.y = y1 + L2_SIZE * sin(js->position[1] + js->position[2] - 1.57);
      lastPoseLeft.orientation.z = js->position[3];
      lastPoseLeft.orientation.y = 0;
      lastPoseLeft.orientation.x = 0;

      RCLCPP_INFO(
       rclcpp::get_logger("arm_pose_joy"),  "left pose: %f %f %f : %f",
       lastPoseLeft.position.x, lastPoseLeft.position.y, lastPoseLeft.position.z,  (lastPoseLeft.orientation.z)  
      );
      
      if (leftArmBusy)
      {
        // see if we are at the target position
        if (fabs(lastPoseLeft.position.x - targetPoseLeft.position.x) < 0.01 &&
            fabs(lastPoseLeft.position.y - targetPoseLeft.position.y) < 0.01 &&
            fabs(lastPoseLeft.position.z - targetPoseLeft.position.z) < 0.01 )
        {
          leftArmBusy = 0;
          RCLCPP_INFO(
            rclcpp::get_logger("arm_pose_joy"), "left arm reached target position");
        }
        else
        {
          RCLCPP_INFO(
            rclcpp::get_logger("arm_pose_joy"), "left arm not at target position, %f", fabs(lastPoseLeft.position.z - targetPoseLeft.position.z));
        } 
      }
    }
    else if (js->name[0][0] == 'r' || js->name[0][0] == 'R')
    {
      // calculate right arm current pose
      lastPoseRight.position.z = js->position[0];
      lastPoseRight.position.x = L1_SIZE * cos(js->position[1]) + L2_SIZE * cos(js->position[1] + js->position[2]);
      lastPoseRight.position.y = L1_SIZE * sin(js->position[1]) + L2_SIZE * sin(js->position[1] + js->position[2]);
      lastPoseRight.orientation.z = js->position[3];
      lastPoseRight.orientation.y = 0;
      lastPoseRight.orientation.x = 0;     
    
      RCLCPP_INFO(
       rclcpp::get_logger("arm_pose_joy"),  "right pose: %f %f %f : %f",
       lastPoseRight.position.x, lastPoseRight.position.y, lastPoseRight.position.z,  lastPoseRight.orientation.z  
      );

      if (rightArmBusy)
      {

        if (/*fabs(lastPoseLeft.position.x - targetPoseLeft.position.x) < 0.01 &&
            fabs(lastPoseLeft.position.y - targetPoseLeft.position.y) < 0.01 && */
            fabs(lastPoseRight.position.z - targetPoseRight.position.z) < 0.01 )
        {
          rightArmBusy = 0;
          RCLCPP_INFO(
            rclcpp::get_logger("arm_pose_joy"), "right arm reached target position");
        }
        else
        {
          RCLCPP_INFO(
            rclcpp::get_logger("arm_pose_joy"), "right arm not at target position, %f", fabs(lastPoseRight.position.z - targetPoseRight.position.z));
        }
      }  
       
    }
  }

/**
 * Constructs ArmPoseJoy.
 */
ArmPoseJoy::ArmPoseJoy(const rclcpp::NodeOptions & options)
: rclcpp::Node("arm_pose_joy_node", options)
{
  pimpl_ = new Impl;

  pimpl_->clock = this->get_clock();

 // pimpl_->frame_id = this->declare_parameter("frame", "arm_pose_joy");

  pimpl_->cmd_pos_pub_left = this->create_publisher<geometry_msgs::msg::Pose>( "/left_cobo_controller/cobo_left_arm_command", 10);
  pimpl_->cmd_pos_pub_right = this->create_publisher<geometry_msgs::msg::Pose>( "/left_cobo_controller/cobo_right_arm_command", 10);

  pimpl_->captureMode = this->create_publisher<std_msgs::msg::Int8>( "/cobo_capture_mode", 1);
  
  pimpl_->cmd_pos_pub_ee = this->create_publisher<geometry_msgs::msg::Pose>( "/cobo_ee_command", 10); 

  pimpl_->joy_sub = this->create_subscription<sensor_msgs::msg::Joy>("joy", rclcpp::QoS(10), std::bind(&ArmPoseJoy::Impl::joyCallback, this->pimpl_, std::placeholders::_1));
  pimpl_->pose_sub = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", rclcpp::QoS(10),std::bind(&ArmPoseJoy::Impl::poseCallback, this->pimpl_, std::placeholders::_1));


  pimpl_->left_joint_command_joint_pub_ =  this->create_publisher<std_msgs::msg::Float32MultiArray>("/cobo_left_arm_command/joint", 10);

  pimpl_->right_joint_command_joint_pub_ =  this->create_publisher<std_msgs::msg::Float32MultiArray>("/cobo_right_arm_command/joint", 10);
   
    //void poseCallback(const sensor_msgs::msg::JointState::SharedPtr joint_state);
 
    // default mappings - can be changed by parameters later

  pimpl_->left_arm_select = 12;   // left button
  pimpl_->right_arm_select = 13;  // right button
  pimpl_->ee_select = 14;   // axes but treated like a button - right trigger
  
  pimpl_->arm_stop = 15;    // X button
  pimpl_->drive_mode = 5;   // this is trigger button that indicates the unit is driving 

  pimpl_->x_move =  0;  // axes l/r - left side joystick
  pimpl_->y_move =  1;  // axes  u/d
  pimpl_->z_move =  4;  // axes u/d  -- right side joystick
  pimpl_->ee_rot =  3;   // 
  pimpl_->ee_move = 4; 

  // capture mode buttons/controls
  pimpl_->captureStart = 0;      // A button
  pimpl_->captureStopSave = 2;   // x button
  pimpl_->captureStopNoSave= 3 ;    // Y button 
  pimpl_->leftArmUpDown = 1;
  pimpl_->rightArmUpDown = 1;

  pimpl_->leftArmBusy = 0;
  pimpl_->rightArmBusy = 0; 
}


ArmPoseJoy::~ArmPoseJoy()
{
  delete pimpl_;
}

double ArmPoseJoy::Impl::getPosAdj(double currentPos,const sensor_msgs::msg::Joy::SharedPtr joy_msg, int64_t joy_msg_indx,int64_t axes)
{
  double t =  currentPos  + (currentPos * (joy_msg->axes[joy_msg_indx] * axes_scale_map[axes]));
      RCLCPP_INFO(
       rclcpp::get_logger("arm_pose_joy"),  "posadj: %li %f %f %f",
       axes, currentPos, joy_msg->axes[joy_msg_indx], axes_scale_map[axes]  );


  return t;  
}



void ArmPoseJoy::Impl::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
{

  // if the drive button is pressed, then we exit as the unit is being used to contorl the drive wheels

  if (joy_msg->axes[drive_mode] < 0)
  {
   //RCLCPP_INFO( rclcpp::get_logger("arm_pose_joy"), "Drive mode is active, ignoring arm commands");
     return;
  }

  
  if (!joy_msg->buttons[left_arm_select]  && !joy_msg->buttons[right_arm_select])
  {
    if (joy_msg->axes[ee_select] > 0.0)
    {
      controlling = NONE;
      
    if ( joy_msg->buttons[captureStart])
    {
       auto msg = std_msgs::msg::Int8();
       msg.data = 3;
       captureMode->publish(msg);
     //  RCLCPP_INFO( rclcpp::get_logger("arm_pose_joy"), "capture start ");
    }; 
    
    if ( joy_msg->buttons[captureStopSave])
    {
       auto msg = std_msgs::msg::Int8();
       msg.data = 1;
       captureMode->publish(msg);
//RCLCPP_INFO( rclcpp::get_logger("arm_pose_joy"), "capture stop, save");
    }; 

    if ( joy_msg->buttons[captureStopNoSave])
    {
       auto msg = std_msgs::msg::Int8();
       msg.data = 2;
       captureMode->publish(msg);
      // RCLCPP_INFO( rclcpp::get_logger("arm_pose_joy"), "capture stop, no save ");
    }; 
    
    return;
    }
  }

  // no, see what are we are controlling
  
  if (joy_msg->buttons[left_arm_select] )
  { 
     if (leftArmBusy == 1)
       return;
     
     controlling = LEFT_ARM;
     RCLCPP_INFO( rclcpp::get_logger("arm_pose_joy"), "Controlling left arm");
   }
   else if (joy_msg->buttons[right_arm_select])
   {
     controlling= RIGHT_ARM;
      RCLCPP_INFO( rclcpp::get_logger("arm_pose_joy"), "Controlling right arm");
   }
  else if (joy_msg->axes[ee_select] < 0.0)
  {
    controlling= END_EFFECTOR;
    RCLCPP_INFO(  rclcpp::get_logger("arm_pose_joy"), "controlling ee");
  }

  if ((controlling != END_EFFECTOR) &&
     ((joy_msg->axes[x_move] != 0.0) ||
      (joy_msg->axes[y_move] != 0.0 ) ||
      (joy_msg->axes[z_move] != 0.0 ) ))
  {

    pose_temp.position.x = 0; //getPosAdj(lastPoseLeft.position.x, joy_msg, x_move,0);
    pose_temp.position.y = 0; //getPosAdj(lastPoseLeft.position.y, joy_msg, y_move,1);
    pose_temp.position.z = getPosAdj(lastPoseLeft.position.z, joy_msg, z_move,2);
    pose_temp.orientation.z = 0; //getPosAdj(lastPoseLeft.orientation.z, joy_msg, ee_rot,3);

    RCLCPP_INFO(
      rclcpp::get_logger("arm_pose_joy"), "new position: %f %f %f : %f",
      pose_temp.position.x, pose_temp.position.y, pose_temp.position.z, pose_temp.orientation.z);

        
    switch (controlling)
    {
      case LEFT_ARM:  
         leftArmBusy = 1;
         cmd_pos_pub_left->publish(pose_temp);
         targetPoseLeft = pose_temp;
         break;

      case RIGHT_ARM:
         rightArmBusy = 1;
         cmd_pos_pub_right->publish(pose_temp);
         targetPoseLeft = pose_temp;
         break;
   
      default:
        RCLCPP_WARN(
          rclcpp::get_logger("arm_pose_joy"),
          "No arm selected for command");
        break;
    }
  }
  else if (controlling == END_EFFECTOR)
  {
    // we are moving the EE in or out. we reuse the z axis for this
    if  (joy_msg->axes[z_move] != 0.0 )
    {
      pose_temp.position.x = 0.0;
      pose_temp.position.y = 0.0;
      pose_temp.orientation.x = 0.0;
      pose_temp.orientation.y = 0.0;
      pose_temp.orientation.z = 0.0;

      lastEEPos += (joy_msg->axes[z_move] > 0 ? 1 : -1);
      pose_temp.position.z = lastEEPos ;
      
      RCLCPP_INFO(rclcpp::get_logger("arm_pose_joy"), "new ee position: %f", pose_temp.position.z);

     // pimpl_->cmd_pos_pub_ee->publish(pose_temp);

    }
 
  }
}

}  // namespace arm_pose_joy

RCLCPP_COMPONENTS_REGISTER_NODE(arm_pose_joy::ArmPoseJoy)
