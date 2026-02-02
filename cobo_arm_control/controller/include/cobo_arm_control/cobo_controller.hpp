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

#ifndef cobo_arm_control__cobo_CONTROLLER_HPP_
#define cobo_arm_control__cobo_CONTROLLER_HPP_

#include <array>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "cobo_arm_control/ik_solver.hpp"
namespace cobo_arm_control
{
class RobotController : public controller_interface::ControllerInterface
{
public:
  RobotController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

protected:
  // Constants
  static constexpr size_t NUMBER_OF_JOINTS = 4;
  static constexpr size_t RIGHT_JOINT_OFFSET = 0;
  static constexpr size_t LEFT_JOINT_OFFSET = NUMBER_OF_JOINTS;
  static constexpr size_t TOTAL_JOINTS = NUMBER_OF_JOINTS * 2;

  // Configuration
  std::vector<std::string> joint_names_;
  std::vector<std::string> command_interface_types_;
  std::vector<std::string> state_interface_types_;

  // Joint state (thread-safe)
  std::array<double, TOTAL_JOINTS> joint_angles_;
  mutable std::mutex joint_angles_mutex_;

  // IK Solver
  std::unique_ptr<IKSolver> ik_solver_;

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr left_joint_command_pose_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr right_joint_command_pose_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr left_joint_command_joint_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr right_joint_command_joint_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr dual_joint_command_joint_subscriber_;

  // Realtime buffers
  realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::Pose>> left_pose_msg_external_point_ptr_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::Pose>> right_pose_msg_external_point_ptr_;

  // State flags
  bool left_new_pose_msg_ = false;
  bool right_new_pose_msg_ = false;
  rclcpp::Time start_time_;

  std::shared_ptr<geometry_msgs::msg::Pose> pose_msg_;

  // Hardware interfaces
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> joint_position_command_interface_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_position_state_interface_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_velocity_state_interface_;

  std::unordered_map<
    std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> *>
    command_interface_map_ = {
      {"position", &joint_position_command_interface_}};

  std::unordered_map<
    std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> *>
    state_interface_map_ = {
      {"position", &joint_position_state_interface_},
      {"velocity", &joint_velocity_state_interface_}};

private:
  // Helper methods
  void updateJointCommands(const std::array<double, TOTAL_JOINTS> & angles);
  bool processIKRequest(
    const geometry_msgs::msg::Pose & pose,
    ArmSide arm_side,
    std::array<double, NUMBER_OF_JOINTS> & joint_values);
  bool validatePoseMessage(const geometry_msgs::msg::Pose & pose) const;
};

}  // namespace cobo_arm_control

#endif  // cobo_arm_control__cobo_CONTROLLER_HPP_
