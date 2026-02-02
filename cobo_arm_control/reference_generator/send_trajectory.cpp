// Copyright 2023 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <kdl/frames.hpp>

int rToD (double radians)
{
  return int ((180/3.1416) * radians);  
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<rclcpp::Node>("send_trajectory");
  auto pub = node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/left_cobo_controller/joint_trajectory", 10);
 
  // get robot description
  auto robot_param = rclcpp::Parameter();
  node->declare_parameter("robot_description", rclcpp::ParameterType::PARAMETER_STRING);
  node->get_parameter("robot_description", robot_param);
  auto robot_description = robot_param.as_string();

  std::cout << robot_description << std::endl;
  
  // create kinematic chain
  KDL::Tree robot_tree;
  KDL::Chain chain;
  kdl_parser::treeFromString(robot_description, robot_tree);
  robot_tree.getChain("base_link", "left_S1_U_A_L2_B1", chain);

 
  auto joint_positions_next = KDL::JntArray(chain.getNrOfJoints());
  auto joint_positions_current = KDL::JntArray(chain.getNrOfJoints());
  auto twist = KDL::Frame();

  // create KDL solvers
  auto ik_pos_solver_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(chain);

  trajectory_msgs::msg::JointTrajectory trajectory_msg;
  trajectory_msg.header.stamp = node->now();
    std::cout << "3"  << std::endl; ;
 
  for (size_t i = 0; i < chain.getNrOfSegments(); i++)
  {
    auto joint = chain.getSegment(i).getJoint();
    if (joint.getType() != KDL::Joint::Fixed)
    {
      trajectory_msg.joint_names.push_back(joint.getName());
      
      std::cout << "joint " << joint.getName() << std::endl;
      
    }
  }

   
  trajectory_msgs::msg::JointTrajectoryPoint trajectory_point_msg;
  trajectory_point_msg.positions.resize(chain.getNrOfJoints());
      
  joint_positions_current(0) = 1.0; 
  joint_positions_current(1) = 3.14;
  joint_positions_current(2) = 1.57;
  joint_positions_current(3) = 3.0;
 
  double total_time = 20.0;
  int trajectory_len = 200;
  int loop_rate = trajectory_len / total_time;
  double dt = 1.0 / loop_rate;

  #define TRAJECT_POINTS 6
  
  float traj[TRAJECT_POINTS][3] = {
    {0.49, 0.0, 1.0},
    {0.49, 0.29,1.0},
   // {0.15, 0.0, 1.0},
   // {0.32, 0.0, 0.8},
  //  {0.49, 0.29,0.8},
  //  {0.15, 0.0, 0.8}
    
    };
    

  
  for (int i = 0; i < TRAJECT_POINTS; i++)
  {
    // set endpoint twist
    double t = i;

    const KDL::Frame p_in(KDL::Vector(traj[i][0],traj[i][1],traj[i][2]));   
     
    //std::cout << "twist " << i << " " << p_in.V.x;
    
    
    //std::cout << "twist " << i << " " << twist.p.x << ":" << twist.p.y << ":" << twist.p.z << std::endl;
 
    std::cout << "cure " << i << " " << joint_positions_current(0) << ":" << 360-rToD(joint_positions_current(1)) << ":" << 360-rToD(joint_positions_current(2));    
    std::cout << ":" << 360-rToD(joint_positions_current(3)) << std::endl;

   ik_pos_solver_->CartToJnt(joint_positions_current, p_in,joint_positions_next);


    std::cout << "next " << i << " " << joint_positions_next(0) << ":" << 360-rToD(joint_positions_next(1)) << ":" << 360-rToD(joint_positions_next(2));    
    std::cout << ":" << 360-rToD(joint_positions_next(3)) << std::endl;
 
    // copy to trajectory_point_msg
    std::memcpy(
      trajectory_point_msg.positions.data(), joint_positions_next.data.data(),
      trajectory_point_msg.positions.size() * sizeof(double));
 
 //   std::memcpy(
 //     trajectory_point_msg.velocities.data(), joint_velocities.data.data(),
//      trajectory_point_msg.velocities.size() * sizeof(double));

  //  // integrate joint velocities
    //joint_positions_next.data += joint_velocities_next.data * dt;

    // set timing information
    trajectory_point_msg.time_from_start.sec = i / loop_rate;
    trajectory_point_msg.time_from_start.nanosec = static_cast<int>(
      1E9 / loop_rate *
      static_cast<double>(t - loop_rate * (i / loop_rate)));  // implicit integer division

   /// std::cout << trajectory_point_msg << "\n";
    
    trajectory_msg.points.push_back(trajectory_point_msg);
    
    joint_positions_current = joint_positions_next;
    
  }

  pub->publish(trajectory_msg);
  while (rclcpp::ok())
  {
  }

  return 0;
}
