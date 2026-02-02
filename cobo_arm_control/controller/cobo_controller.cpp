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

#include "cobo_arm_control/cobo_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

using config_type = controller_interface::interface_configuration_type;

namespace
{
// Configuration constants
constexpr double DEFAULT_L1_LENGTH = 0.20;  // Upper arm length in meters
constexpr double DEFAULT_L2_LENGTH = 0.20;  // Lower arm length in meters
constexpr double POSE_POSITION_MAX = 0.40;   // Maximum position value
constexpr double POSE_POSITION_MIN = -0.40;  // Minimum position value
constexpr double Z_POSE_POSITION_MAX = 1.8;   // Maximum Z position value
constexpr double Z_POSE_POSITION_MIN = 0.370;  // Minimum Z position value
}  // namespace

namespace cobo_arm_control
{

RobotController::RobotController()
: controller_interface::ControllerInterface(),
  joint_angles_{0.0},
  ik_solver_(nullptr)
{
}

bool RobotController::validatePoseMessage(const geometry_msgs::msg::Pose & pose) const
{
  // Validate position bounds
  if (std::abs(pose.position.x) > POSE_POSITION_MAX   ||
      std::abs(pose.position.y) > POSE_POSITION_MAX   ||
      std::abs(pose.position.z) > Z_POSE_POSITION_MAX ||
      std::abs(pose.position.z) < Z_POSE_POSITION_MIN)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Pose position out of bounds: (%.3f, %.3f, %.3f)",
      pose.position.x, pose.position.y, pose.position.z);
    return false;
  }

  // Check for NaN or infinity
  if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
      !std::isfinite(pose.position.z) || !std::isfinite(pose.orientation.z))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Pose contains invalid values (NaN or Inf)");
    return false;
  }

  return true;
}

bool RobotController::processIKRequest(
  const geometry_msgs::msg::Pose & pose,
  ArmSide arm_side,
  std::array<double, NUMBER_OF_JOINTS> & joint_values)
{
  if (!ik_solver_) {
    RCLCPP_ERROR(get_node()->get_logger(), "IK solver not initialized");
    return false;
  }

  // Prepare IK input
  Pose2D target_pose;
  target_pose.x = pose.position.x;
  target_pose.y = pose.position.y;
  target_pose.rotation = pose.orientation.z;

  RCLCPP_DEBUG(
    get_node()->get_logger(),
    "IK request for %s arm: pos(%.3f, %.3f, %.3f) rot(%.3f)",
    arm_side == ArmSide::LEFT ? "left" : "right",
    pose.position.x, pose.position.y, pose.position.z, pose.orientation.z);

  // Solve IK
  auto solution = ik_solver_->solve(target_pose, arm_side);

  if (!solution.has_value()) {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "IK failed: target (%.3f, %.3f) unreachable or out of limits",
      target_pose.x, target_pose.y);
    return false;
  }

  // Joint 0 is the Z-axis (linear), directly from pose
  joint_values[0] = pose.position.z;
  // Joints 1-3 from IK solution
  joint_values[1] = solution->joint_1;
  joint_values[2] = solution->joint_2;
  joint_values[3] = solution->joint_3;

  RCLCPP_INFO(
    get_node()->get_logger(),
    "IK solution (%s elbow): j0=%.3f j1=%.3f j2=%.3f j3=%.3f",
    solution->elbow_config == ElbowConfig::UP ? "up" : "down",
    joint_values[0], joint_values[1], joint_values[2], joint_values[3]);

  return true;
}

void RobotController::updateJointCommands(const std::array<double, TOTAL_JOINTS> & angles)
{
  if (joint_position_command_interface_.size() != TOTAL_JOINTS) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Command interface size mismatch: expected %zu, got %zu",
      TOTAL_JOINTS, joint_position_command_interface_.size());
    return;
  }

  for (size_t i = 0; i < TOTAL_JOINTS; ++i) {
    joint_position_command_interface_[i].get().set_value(angles[i]);
  }
}

controller_interface::CallbackReturn RobotController::on_init()
{
  try {
    joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
    command_interface_types_ = auto_declare<std::vector<std::string>>(
      "command_interfaces", command_interface_types_);
    state_interface_types_ = auto_declare<std::vector<std::string>>(
      "state_interfaces", state_interface_types_);

    // Declare arm length parameters
    auto_declare<double>("l1_length", DEFAULT_L1_LENGTH);
    auto_declare<double>("l2_length", DEFAULT_L2_LENGTH);

    RCLCPP_INFO(get_node()->get_logger(), "Controller initialized successfully");
    return CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error during initialization: %s", e.what());
    return CallbackReturn::ERROR;
  }
}

controller_interface::InterfaceConfiguration RobotController::command_interface_configuration() const
{
  RCLCPP_DEBUG(get_node()->get_logger(), "Configuring command interfaces");

  controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

  conf.names.reserve(joint_names_.size() * command_interface_types_.size());
  for (const auto & joint_name : joint_names_) {
    for (const auto & interface_type : command_interface_types_) {
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }

  RCLCPP_DEBUG(
    get_node()->get_logger(),
    "Configured %zu command interfaces", conf.names.size());

  return conf;
}

controller_interface::InterfaceConfiguration RobotController::state_interface_configuration() const
{
  RCLCPP_DEBUG(get_node()->get_logger(), "Configuring state interfaces");

  controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

  conf.names.reserve(joint_names_.size() * state_interface_types_.size());
  for (const auto & joint_name : joint_names_) {
    for (const auto & interface_type : state_interface_types_) {
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }

  RCLCPP_DEBUG(
    get_node()->get_logger(),
    "Configured %zu state interfaces", conf.names.size());

  return conf;
}

controller_interface::CallbackReturn RobotController::on_configure(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_node()->get_logger(), "Configuring controller");

  // Initialize IK solver with configured parameters
  const double l1_length = get_node()->get_parameter("l1_length").as_double();
  const double l2_length = get_node()->get_parameter("l2_length").as_double();
  ik_solver_ = std::make_unique<IKSolver>(l1_length, l2_length);

  RCLCPP_INFO(
    get_node()->get_logger(),
    "IK solver initialized with L1=%.3f, L2=%.3f", l1_length, l2_length);

  // Left pose callback
  auto left_pose_callback =
    [this](const std::shared_ptr<geometry_msgs::msg::Pose> pose_msg) -> void
  {
    RCLCPP_DEBUG(get_node()->get_logger(), "Received left pose command");
    left_pose_msg_external_point_ptr_.writeFromNonRT(pose_msg);
    left_new_pose_msg_ = true;
  };

  // Right pose callback
  auto right_pose_callback =
    [this](const std::shared_ptr<geometry_msgs::msg::Pose> pose_msg) -> void
  {
    RCLCPP_DEBUG(get_node()->get_logger(), "Received right pose command");
    right_pose_msg_external_point_ptr_.writeFromNonRT(pose_msg);
    right_new_pose_msg_ = true;
  };

  // Dual joint callback
  auto dual_joint_callback =
    [this](const std::shared_ptr<std_msgs::msg::Float32MultiArray> joint_msg) -> void
  {
    RCLCPP_DEBUG(get_node()->get_logger(), "Received dual joint command");

    if (joint_msg->data.size() != TOTAL_JOINTS) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Invalid dual joint message size: expected %zu, got %zu",
        TOTAL_JOINTS, joint_msg->data.size());
      return;
    }

    std::lock_guard<std::mutex> lock(joint_angles_mutex_);
    for (size_t i = 0; i < TOTAL_JOINTS; ++i) {
      joint_angles_[i] = joint_msg->data[i];
    }

    updateJointCommands(joint_angles_);
  };

  // Left joint callback
  auto left_joint_callback =
    [this](const std::shared_ptr<std_msgs::msg::Float32MultiArray> joint_msg) -> void
  {
    RCLCPP_DEBUG(get_node()->get_logger(), "Received left joint command");

    if (joint_msg->data.size() != NUMBER_OF_JOINTS) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Invalid left joint message size: expected %zu, got %zu",
        NUMBER_OF_JOINTS, joint_msg->data.size());
      return;
    }

    std::lock_guard<std::mutex> lock(joint_angles_mutex_);
    for (size_t i = 0; i < NUMBER_OF_JOINTS; ++i) {
      joint_angles_[i + LEFT_JOINT_OFFSET] = joint_msg->data[i];
    }

    updateJointCommands(joint_angles_);
  };

  // Right joint callback
  auto right_joint_callback =
    [this](const std::shared_ptr<std_msgs::msg::Float32MultiArray> joint_msg) -> void
  {
    RCLCPP_DEBUG(get_node()->get_logger(), "Received right joint command");

    if (joint_msg->data.size() != NUMBER_OF_JOINTS) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Invalid right joint message size: expected %zu, got %zu",
        NUMBER_OF_JOINTS, joint_msg->data.size());
      return;
    }

    std::lock_guard<std::mutex> lock(joint_angles_mutex_);
    for (size_t i = 0; i < NUMBER_OF_JOINTS; ++i) {
      joint_angles_[i + RIGHT_JOINT_OFFSET] = joint_msg->data[i];
    }

    updateJointCommands(joint_angles_);
  };

  // Create subscriptions
  left_joint_command_pose_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::Pose>(
    "/cobo_left_arm_command/pose", rclcpp::SystemDefaultsQoS(), left_pose_callback);

  right_joint_command_pose_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::Pose>(
    "/cobo_right_arm_command/pose", rclcpp::SystemDefaultsQoS(), right_pose_callback);

  left_joint_command_joint_subscriber_ =
    get_node()->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/cobo_left_arm_command/joint", rclcpp::SystemDefaultsQoS(), left_joint_callback);

  right_joint_command_joint_subscriber_ =
    get_node()->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/cobo_right_arm_command/joint", rclcpp::SystemDefaultsQoS(), right_joint_callback);

  dual_joint_command_joint_subscriber_ =
    get_node()->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/cobo_dual_arm_command/joint", rclcpp::SystemDefaultsQoS(), dual_joint_callback);

  RCLCPP_INFO(get_node()->get_logger(), "Controller configured successfully");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RobotController::on_activate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_node()->get_logger(), "Activating controller");

  // Clear out vectors in case of restart
  joint_position_command_interface_.clear();
  joint_position_state_interface_.clear();
  joint_velocity_state_interface_.clear();

  // Assign command interfaces
  for (auto & interface : command_interfaces_) {
    command_interface_map_[interface.get_interface_name()]->push_back(interface);
    RCLCPP_DEBUG(
      get_node()->get_logger(),
      "Registered command interface: %s", interface.get_interface_name().c_str());
  }

  // Assign state interfaces
  for (auto & interface : state_interfaces_) {
    state_interface_map_[interface.get_interface_name()]->push_back(interface);
    RCLCPP_DEBUG(
      get_node()->get_logger(),
      "Registered state interface: %s", interface.get_interface_name().c_str());
  }

  RCLCPP_INFO(get_node()->get_logger(), "Controller activated successfully");
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type RobotController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  // Process pose messages if available
  if (left_new_pose_msg_ || right_new_pose_msg_) {
    ArmSide arm_side;
    std::shared_ptr<geometry_msgs::msg::Pose> pose;

    if (left_new_pose_msg_) {
      pose = *left_pose_msg_external_point_ptr_.readFromRT();
      left_new_pose_msg_ = false;
      arm_side = ArmSide::LEFT;
      RCLCPP_DEBUG(get_node()->get_logger(), "Processing left arm pose");
    } else {
      pose = *right_pose_msg_external_point_ptr_.readFromRT();
      right_new_pose_msg_ = false;
      arm_side = ArmSide::RIGHT;
      RCLCPP_DEBUG(get_node()->get_logger(), "Processing right arm pose");
    }

    start_time_ = time;

    if (pose != nullptr) {
      // Validate pose message
      if (!validatePoseMessage(*pose)) {
        RCLCPP_ERROR(get_node()->get_logger(), "Invalid pose message received");
        return controller_interface::return_type::OK;
      }

      RCLCPP_DEBUG(
        get_node()->get_logger(),
        "Pose: pos(%.3f, %.3f, %.3f) orient(%.3f, %.3f, %.3f)",
        pose->position.x, pose->position.y, pose->position.z,
        pose->orientation.x, pose->orientation.y, pose->orientation.z);

      // Compute IK
      std::array<double, NUMBER_OF_JOINTS> joint_values;
      if (!processIKRequest(*pose, arm_side, joint_values)) {
        RCLCPP_ERROR(get_node()->get_logger(), "IK calculation failed");
        return controller_interface::return_type::OK;
      }

      // Update joint angles with thread safety
      {
        std::lock_guard<std::mutex> lock(joint_angles_mutex_);
        const size_t offset = (arm_side == ArmSide::RIGHT) ? RIGHT_JOINT_OFFSET : LEFT_JOINT_OFFSET;

        for (size_t i = 0; i < NUMBER_OF_JOINTS; ++i) {
          joint_angles_[i + offset] = joint_values[i];
          RCLCPP_DEBUG(
            get_node()->get_logger(),
            "Joint[%zu] = %.3f", i + offset, joint_values[i]);
        }

        // Send commands to hardware
        updateJointCommands(joint_angles_);
      }
    }
  }

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn RobotController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_node()->get_logger(), "Deactivating controller");
  release_interfaces();
  return CallbackReturn::SUCCESS;
}

}  // namespace cobo_arm_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  cobo_arm_control::RobotController, controller_interface::ControllerInterface)
