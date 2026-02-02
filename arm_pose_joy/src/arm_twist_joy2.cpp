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

#include <geometry_msgs/msg/twist.hpp>
 #include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "arm_twist_joy/arm_twist_joy.hpp"

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
namespace arm_twist_joy
{


   enum CONTROLLING {LEFT_ARM, RIGHT_ARM, END_EFFECTOR, NONE};
  float axes_scale_map[4] = {0.1, 0.1, 0.1, 0.1}; // scale for each axes

/**
 * Internal members of class. This is the pimpl idiom, and allows more flexibility in adding
 * parameters later without breaking ABI compatibility, for robots which link ArmTwistJoy
 * directly into base nodes.
 */
struct ArmTwistJoy::Impl
{
  void poseCallback(const sensor_msgs::msg::JointState::SharedPtr joint_state);
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy);
  double getPosAdj(double currentPos, const sensor_msgs::msg::Joy::SharedPtr joy_msg, int64_t axejoy_msg_idx,int64_t axes);
 
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr pose_sub;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pos_pub_left;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pos_pub_right;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pos_pub_ee;
  
  rclcpp::Clock::SharedPtr clock;
  geometry_msgs::msg::Twist lastPoseLeft;   
  geometry_msgs::msg::Twist lastPoseRight;
  geometry_msgs::msg::Twist targetPoseLeft;   
  geometry_msgs::msg::Twist targetPoseRight;
  geometry_msgs::msg::Twist twist_temp;

  
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
 
  float lastEEPos = 0.0; // last end effector position

  float L1_SIZE = 0.1905;  // length of first arm segment
  float L2_SIZE = 0.1905;  // length of second arm segment

  CONTROLLING controlling = NONE; 
  int leftArmBusy;
  int rightArmBusy;

};

 void ArmTwistJoy::Impl::poseCallback(const sensor_msgs::msg::JointState::SharedPtr js)
 {
    // calculates pose from the twist message. Note that we may get callbacks for a variety of sources
    // and so make sure we are getting only the right or left arm ones. 
    double x1, y1;


    if (js->name.size() != 4)
    {
        return;
    } 
    RCLCPP_INFO(
       rclcpp::get_logger("arm_twist_joy"),  "js: %f %f %f : %f",
       js->position[0], rad2deg(js->position[1]), rad2deg(js->position[2]), rad2deg(js->position[3])  
      );

    if (js->name[0][0] == 'l' || js->name[0][0] == 'L')
    {
      x1 = L1_SIZE * cos(js->position[1]);
      y1 = L1_SIZE * sin(js->position[1]);

      // calculate left arm current pose
      lastPoseLeft.linear.z = js->position[0];
      lastPoseLeft.linear.x = x1 + L2_SIZE * cos(js->position[1] + js->position[2] -1.57);
      lastPoseLeft.linear.y = y1 + L2_SIZE * sin(js->position[1] + js->position[2] - 1.57);
      lastPoseLeft.angular.z = js->position[3];
      lastPoseLeft.angular.y = 0;
      lastPoseLeft.angular.x = 0;

      RCLCPP_INFO(
       rclcpp::get_logger("arm_twist_joy"),  "left pose: %f %f %f : %f",
       lastPoseLeft.linear.x, lastPoseLeft.linear.y, lastPoseLeft.linear.z,  (lastPoseLeft.angular.z)  
      );
      
      if (leftArmBusy)
      {
        // see if we are at the target position
        if (fabs(lastPoseLeft.linear.x - targetPoseLeft.linear.x) < 0.01 &&
            fabs(lastPoseLeft.linear.y - targetPoseLeft.linear.y) < 0.01 &&
            fabs(lastPoseLeft.linear.z - targetPoseLeft.linear.z) < 0.01 )
        {
          leftArmBusy = 0;
          RCLCPP_INFO(
            rclcpp::get_logger("arm_twist_joy"), "left arm reached target position");
        }
        else
        {
          RCLCPP_INFO(
            rclcpp::get_logger("arm_twist_joy"), "left arm not at target position, %f", fabs(lastPoseLeft.linear.z - targetPoseLeft.linear.z));
        } 
      }
    }
    else if (js->name[0][0] == 'r' || js->name[0][0] == 'R')
    {
      // calculate right arm current pose
      lastPoseRight.linear.z = js->position[0];
      lastPoseRight.linear.x = L1_SIZE * cos(js->position[1]) + L2_SIZE * cos(js->position[1] + js->position[2]);
      lastPoseRight.linear.y = L1_SIZE * sin(js->position[1]) + L2_SIZE * sin(js->position[1] + js->position[2]);
      lastPoseRight.angular.z = js->position[3];
      lastPoseRight.angular.y = 0;
      lastPoseRight.angular.x = 0;     
    
      RCLCPP_INFO(
       rclcpp::get_logger("arm_twist_joy"),  "right pose: %f %f %f : %f",
       lastPoseRight.linear.x, lastPoseRight.linear.y, lastPoseRight.linear.z,  lastPoseRight.angular.z  
      );
    }
  }

/**
 * Constructs ArmTwistJoy.
 */
ArmTwistJoy::ArmTwistJoy(const rclcpp::NodeOptions & options)
: rclcpp::Node("arm_twist_joy_node", options)
{
  pimpl_ = new Impl;

  pimpl_->clock = this->get_clock();

 // pimpl_->frame_id = this->declare_parameter("frame", "arm_twist_joy");

  pimpl_->cmd_pos_pub_left = this->create_publisher<geometry_msgs::msg::Twist>( "/left_cobo_controller/cobo_left_arm_command", 10);
  pimpl_->cmd_pos_pub_right = this->create_publisher<geometry_msgs::msg::Twist>( "/left_cobo_controller/cobo_right_arm_command", 10);

   
  pimpl_->cmd_pos_pub_ee = this->create_publisher<geometry_msgs::msg::Twist>( "/cobo_ee_command", 10);
   

  pimpl_->joy_sub = this->create_subscription<sensor_msgs::msg::Joy>("joy", rclcpp::QoS(10), std::bind(&ArmTwistJoy::Impl::joyCallback, this->pimpl_, std::placeholders::_1));
  pimpl_->pose_sub = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", rclcpp::QoS(10),std::bind(&ArmTwistJoy::Impl::poseCallback, this->pimpl_, std::placeholders::_1));

    //void poseCallback(const sensor_msgs::msg::JointState::SharedPtr joint_state);
 
  pimpl_->left_arm_select = 4;   // left button
  pimpl_->right_arm_select = 5;  // right button
  pimpl_->ee_select = 2;   // axes but treated like a button - right trigger
  
  pimpl_->arm_stop = 2;    // X button
  pimpl_->drive_mode = 5;   // this is trigger button that indicates the unit is driving 

  pimpl_->x_move =  0;  // axes l/r - left side joystick
  pimpl_->y_move =  1;  // axes  u/d
  pimpl_->z_move =  4;  // axes u/d  -- right side joystick
  pimpl_->ee_rot =  3;   // 

  pimpl_->ee_move = 4; 

  pimpl_->leftArmBusy = 0;
  pimpl_->rightArmBusy = 0; 
}

ArmTwistJoy::~ArmTwistJoy()
{
  delete pimpl_;
}

double ArmTwistJoy::Impl::getPosAdj(double currentPos,const sensor_msgs::msg::Joy::SharedPtr joy_msg, int64_t joy_msg_indx,int64_t axes)
{
  double t =  currentPos  + (currentPos * (joy_msg->axes[joy_msg_indx] * axes_scale_map[axes]));
      RCLCPP_INFO(
       rclcpp::get_logger("arm_twist_joy"),  "posadj: %li %f %f %f",
       axes, currentPos, joy_msg->axes[joy_msg_indx], axes_scale_map[axes]  );


  return t;  
}



void ArmTwistJoy::Impl::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
{

  // if the drive button is pressed, then we exit as the unit is being used to contorl the drive wheels

  if (joy_msg->axes[drive_mode] < 0)
  {
   //RCLCPP_INFO( rclcpp::get_logger("arm_twist_joy"), "Drive mode is active, ignoring arm commands");
     return;
  }

  // see if we should stop everything
  
  if (joy_msg->buttons[arm_stop])
  {
    RCLCPP_INFO( rclcpp::get_logger("arm_twist_joy"), "stop all arm movements");

    // do something
    return;
  }  
  
  // no, see what are we are controlling
  
  if (joy_msg->buttons[left_arm_select] )
  { 
     if (leftArmBusy == 1)
       return;
     
     controlling = LEFT_ARM;
     RCLCPP_INFO( rclcpp::get_logger("arm_twist_joy"), "Controlling left arm");
   }
   else if (joy_msg->buttons[right_arm_select])
   {
     controlling= RIGHT_ARM;
      RCLCPP_INFO( rclcpp::get_logger("arm_twist_joy"), "Controlling right arm");
   }
  else if (joy_msg->axes[ee_select] < 0.0)
  {
    controlling= END_EFFECTOR;
    RCLCPP_INFO(  rclcpp::get_logger("arm_twist_joy"), "controlling ee");
  }

  if ((controlling != END_EFFECTOR) &&
     ((joy_msg->axes[x_move] != 0.0) ||
      (joy_msg->axes[y_move] != 0.0 ) ||
      (joy_msg->axes[z_move] != 0.0 ) ))
  {

    twist_temp.linear.x = getPosAdj(lastPoseLeft.linear.x, joy_msg, x_move,0);
    twist_temp.linear.y = getPosAdj(lastPoseLeft.linear.y, joy_msg, y_move,1);
    twist_temp.linear.z = getPosAdj(lastPoseLeft.linear.z, joy_msg, z_move,2);
    twist_temp.angular.z = getPosAdj(lastPoseLeft.angular.z, joy_msg, ee_rot,3);

    RCLCPP_INFO(
      rclcpp::get_logger("arm_twist_joy"), "new position: %f %f %f : %f",
      twist_temp.linear.x, twist_temp.linear.y, twist_temp.linear.z, twist_temp.angular.z);

        
    switch (controlling)
    {
      case LEFT_ARM:  
         leftArmBusy = 1;
         cmd_pos_pub_left->publish(twist_temp);
         targetPoseLeft = twist_temp;
         break;

      case RIGHT_ARM:
         //cmd_pos_pub_right->publish(twist_temp);
         break;
   
      default:
        RCLCPP_WARN(
          rclcpp::get_logger("arm_twist_joy"),
          "No arm selected for command");
        break;
    }
  }
  else if (controlling == END_EFFECTOR)
  {
    // we are moving the EE in or out. we reuse the z axis for this
    if  (joy_msg->axes[z_move] != 0.0 )
    {
      twist_temp.linear.x = 0.0;
      twist_temp.linear.y = 0.0;
      twist_temp.angular.x = 0.0;
      twist_temp.angular.y = 0.0;
      twist_temp.angular.z = 0.0;

      lastEEPos += (joy_msg->axes[z_move] > 0 ? 1 : -1);
      twist_temp.linear.z = lastEEPos ;
      
      RCLCPP_INFO(rclcpp::get_logger("arm_twist_joy"), "new ee position: %f", twist_temp.linear.z);

     // pimpl_->cmd_pos_pub_ee->publish(twist_temp);

    }
 
  }
}

}  // namespace arm_twist_joy

RCLCPP_COMPONENTS_REGISTER_NODE(arm_twist_joy::ArmTwistJoy)
