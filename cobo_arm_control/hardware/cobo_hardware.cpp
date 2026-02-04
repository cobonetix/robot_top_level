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

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <string>
#include <iostream>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "cobo_arm_control/arduino_comms.hpp"
#include "cobo_arm_control/cobo_hardware.hpp"

namespace cobo_arm_control
{

// Constants for initialization
constexpr double INITIAL_JOINT_POSITION = 0.157;
constexpr double INITIAL_JOINT_VELOCITY = 0.5;
constexpr double DEFAULT_TOWER_RIGHT_HEIGHT = 0.8;
constexpr double DEFAULT_TOWER_LEFT_HEIGHT = 1.0;
constexpr int INITIAL_ARM_ANGLE_DEGREES = 180;
constexpr std::chrono::seconds INIT_DELAY{2};
constexpr std::chrono::seconds ARM_MOVEMENT_DELAY{3};
constexpr std::chrono::seconds CALIBRATION_DELAY{2};
constexpr std::chrono::seconds FINAL_INIT_DELAY{5};

CallbackReturn RobotSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  logger_ = std::make_shared<rclcpp::Logger>(
    rclcpp::get_logger("controller_manager.resource_manager.hardware_component.system.cobo"));

  clock_ = std::make_shared<rclcpp::Clock>(rclcpp::Clock());

  RCLCPP_INFO(rclcpp::get_logger("cobo"), "Init ...please wait...");

  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // Robot has 8 joints (4 per arm)
  joint_position_.assign(8, 0);
  joint_velocities_.assign(8, 0);
  joint_position_command_.assign(8, 0);
  joint_velocities_command_.assign(8, 0);

  hw_commands_.assign(CMD_LEFT_ARM_RESET + 1, 0.0);
  hw_states_.assign(ST_LEFT_DOOR_POSITION + 1, 0.0);

  cfg_.towerPort = info_.hardware_parameters["tower_port"];
  cfg_.armPort[RIGHT_ARM_PORT] = info_.hardware_parameters["right_arm_port"];
  cfg_.armPort[LEFT_ARM_PORT] = info_.hardware_parameters["left_arm_port"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.initial_rotation = std::stof(info_.hardware_parameters["initial_rotation"]);
  cfg_.initial_z = std::stof(info_.hardware_parameters["initial_z"]);
  cfg_.fakeMode = (std::stoi(info_.hardware_parameters["fake_mode"]) != 0);

  for (const auto & joint : info_.joints)
  {
    for (const auto & interface : joint.state_interfaces)
    {
      joint_interfaces[interface.name].push_back(joint.name);
      RCLCPP_INFO(rclcpp::get_logger("cobo"), "interface %s segment %s",
                  interface.name.c_str(), joint.name.c_str());
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("cobo"), "Init done");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RobotSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  int ind = 0;
  int oddeven = 0;

  for (const auto & joint_name : joint_interfaces["position"])
  {
    if (oddeven++ & 1)
    {
      RCLCPP_INFO(rclcpp::get_logger("cobo"), "state pos interfaces %s %d",
                  joint_name.c_str(), ind);

      state_interfaces.emplace_back(joint_name, "position", &joint_position_[ind]);
      joint_position_[ind] = INITIAL_JOINT_POSITION;
      ind++;
    }
  }

  ind = 0;
  for (const auto & joint_name : joint_interfaces["velocity"])
  {
    RCLCPP_INFO(rclcpp::get_logger("cobo"), "state vel interfaces %s %d",
                joint_name.c_str(), ind);
    state_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_[ind]);
    joint_velocities_[ind] = INITIAL_JOINT_VELOCITY;
    ind++;
  }

  size_t ct = 0;
  for (size_t i = 0; i < info_.gpios.size(); i++)
  {
    for (auto state_if : info_.gpios.at(i).state_interfaces)
    {
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(info_.gpios.at(i).name, state_if.name, &hw_states_[ct++]));

        RCLCPP_INFO(get_logger(), "Added %s/%s", info_.gpios.at(i).name.c_str(),state_if.name.c_str());
    }
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RobotSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  int ind = 0;
  int oddeven = 0;

  for (const auto & joint_name : joint_interfaces["position"])
  {
    if (oddeven++ & 1)
    {
      command_interfaces.emplace_back(joint_name, "position", &joint_position_command_[ind]);
      RCLCPP_INFO(rclcpp::get_logger("cobo"), "cmd interfaces %s %d",
                  joint_name.c_str(), ind);
      ind++;
    }
  }

  size_t ct = 0;
  for (size_t i = 0; i < info_.gpios.size(); i++)
  {
    for (auto command_if : info_.gpios.at(i).command_interfaces)
    {
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          info_.gpios.at(i).name, command_if.name, &hw_commands_[ct++]));
      RCLCPP_INFO(get_logger(), "Added %s/%s", info_.gpios.at(i).name.c_str(),
                  command_if.name.c_str());
    }
  }

  return command_interfaces;
}

CallbackReturn RobotSystem::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("cobo"), "deactivate ...please wait...");
  return CallbackReturn::SUCCESS;
}

CallbackReturn RobotSystem::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("cobo"), "activate ...please wait...");

  // Set loggers for communication objects
  TowerComms.set_logger(rclcpp::get_logger("cobo.tower"));
  ArmComms[LEFT_ARM_PORT].set_logger(rclcpp::get_logger("cobo.left_arm"));
  ArmComms[RIGHT_ARM_PORT].set_logger(rclcpp::get_logger("cobo.right_arm"));

  // Disconnect if already connected
  if (TowerComms.connected())
  {
    TowerComms.disconnect();
    RCLCPP_INFO(rclcpp::get_logger("cobo"), "tower disconnect");
  }

  if (ArmComms[LEFT_ARM_PORT].connected())
  {
    ArmComms[LEFT_ARM_PORT].disconnect();
    RCLCPP_INFO(rclcpp::get_logger("cobo"), "left arm disconnect");
  }

  if (ArmComms[RIGHT_ARM_PORT].connected())
  {
    ArmComms[RIGHT_ARM_PORT].disconnect();
    RCLCPP_INFO(rclcpp::get_logger("cobo"), "right arm disconnect");
  }

  RCLCPP_INFO(rclcpp::get_logger("cobo"), "ready to connect to ports");

  TowerComms.connect(cfg_.towerPort, cfg_.baud_rate, cfg_.timeout_ms);

  if (!TowerComms.connected())
  {
    RCLCPP_ERROR(rclcpp::get_logger("cobo"), "tower connect failed");
    ArmComms[LEFT_ARM_PORT].disconnect();
    ArmComms[RIGHT_ARM_PORT].disconnect();
    return CallbackReturn::FAILURE;
  }

  ArmComms[LEFT_ARM_PORT].connect(cfg_.armPort[LEFT_ARM_PORT], cfg_.baud_rate, cfg_.timeout_ms);

  if (!ArmComms[LEFT_ARM_PORT].connected())
  {
    RCLCPP_ERROR(rclcpp::get_logger("cobo"), "left arm connect failed");
    TowerComms.disconnect();
    return CallbackReturn::FAILURE;
  }

  std::this_thread::sleep_for(INIT_DELAY);

  ArmComms[LEFT_ARM_PORT].set_left_arm();

  ArmComms[RIGHT_ARM_PORT].connect(cfg_.armPort[RIGHT_ARM_PORT], cfg_.baud_rate, cfg_.timeout_ms);

  if (!ArmComms[RIGHT_ARM_PORT].connected())
  {
    RCLCPP_ERROR(rclcpp::get_logger("cobo"), "right arm connect failed");
    TowerComms.disconnect();
    ArmComms[LEFT_ARM_PORT].disconnect();
    return CallbackReturn::FAILURE;
  }
 
  RCLCPP_INFO(rclcpp::get_logger("cobo"), "All ports connected, calibrating tower");



  if (!cfg_.fakeMode)
  {
    // Lock arms
    ArmComms[LEFT_ARM_PORT].arm_lock(1);
    ArmComms[RIGHT_ARM_PORT].arm_lock(1);
  }

    // Set fake mode if needed
  ArmComms[LEFT_ARM_PORT].arm_set_fake_mode(cfg_.fakeMode ? 1 : 0);
  ArmComms[RIGHT_ARM_PORT].arm_set_fake_mode(cfg_.fakeMode ? 1 : 0);
  TowerComms.tower_set_fake_mode(cfg_.fakeMode ? 1 : 0);


  const double initial_angle_rad = degrees_to_radians(INITIAL_ARM_ANGLE_DEGREES);
  //ArmComms[LEFT_ARM_PORT].set_arm_position(initial_angle_rad, initial_angle_rad, initial_angle_rad);
  //[RIGHT_ARM_PORT].set_arm_position(initial_angle_rad, initial_angle_rad, initial_angle_rad);

 // std::this_thread::sleep_for(ARM_MOVEMENT_DELAY);

  //TowerComms.calibrate_tower();
  //std::this_thread::sleep_for(CALIBRATION_DELAY);
//TowerComms.set_tower_position(DEFAULT_TOWER_RIGHT_HEIGHT, DEFAULT_TOWER_LEFT_HEIGHT);
  //TowerComms.set_tower_parameters(hw_commands_);

//std::this_thread::sleep_for(FINAL_INIT_DELAY);

  RCLCPP_INFO(rclcpp::get_logger("cobo"), "activate ...done");

  return CallbackReturn::SUCCESS;
}

return_type RobotSystem::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  double val1, val2;

  TowerComms.read_tower_info(val1, val2, hw_states_);

  joint_position_[LEFT_ARM_JOINT_OFFSET + 0] = val1;
  joint_position_[RIGHT_ARM_JOINT_OFFSET + 0] = val2;

  ArmComms[LEFT_ARM_PORT].read_arm_info(
    joint_position_[LEFT_ARM_JOINT_OFFSET + 1],
    joint_position_[LEFT_ARM_JOINT_OFFSET + 2],
    joint_position_[LEFT_ARM_JOINT_OFFSET + 3],
    hw_states_);

  ArmComms[RIGHT_ARM_PORT].read_arm_info(
    joint_position_[RIGHT_ARM_JOINT_OFFSET + 1],
    joint_position_[RIGHT_ARM_JOINT_OFFSET + 2],
    joint_position_[RIGHT_ARM_JOINT_OFFSET + 3],
    hw_states_);

  RCLCPP_DEBUG(rclcpp::get_logger("cobo_rd"),
               "left arm %.3f %d %d %d",
               joint_position_[LEFT_ARM_JOINT_OFFSET + 0],
               radians_to_degrees(joint_position_[LEFT_ARM_JOINT_OFFSET + 1]),
               radians_to_degrees(joint_position_[LEFT_ARM_JOINT_OFFSET + 2]),
               radians_to_degrees(joint_position_[LEFT_ARM_JOINT_OFFSET + 3]));

  RCLCPP_DEBUG(rclcpp::get_logger("cobo_rd"),
               "right arm %.3f %d %d %d",
               joint_position_[RIGHT_ARM_JOINT_OFFSET + 0],
               radians_to_degrees(joint_position_[RIGHT_ARM_JOINT_OFFSET + 1]),
               radians_to_degrees(joint_position_[RIGHT_ARM_JOINT_OFFSET + 2]),
               radians_to_degrees(joint_position_[RIGHT_ARM_JOINT_OFFSET + 3]));

  return return_type::OK;
}

return_type RobotSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  TowerComms.set_tower_position(
    joint_position_command_[LEFT_ARM_JOINT_OFFSET + 0],
    joint_position_command_[RIGHT_ARM_JOINT_OFFSET + 0]);

  ArmComms[LEFT_ARM_PORT].set_arm_position(
    joint_position_command_[LEFT_ARM_JOINT_OFFSET + 1],
    joint_position_command_[LEFT_ARM_JOINT_OFFSET + 2],
    joint_position_command_[LEFT_ARM_JOINT_OFFSET + 3]);

  ArmComms[RIGHT_ARM_PORT].set_arm_position(
    joint_position_command_[RIGHT_ARM_JOINT_OFFSET + 1],
    joint_position_command_[RIGHT_ARM_JOINT_OFFSET + 2],
    joint_position_command_[RIGHT_ARM_JOINT_OFFSET + 3]);
  
  TowerComms.set_tower_parameters(hw_commands_);
  ArmComms[RIGHT_ARM_PORT].set_arm_parameters(hw_commands_);
  ArmComms[LEFT_ARM_PORT].set_arm_parameters(hw_commands_);

  return return_type::OK;
}

}  // namespace cobo_arm_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  cobo_arm_control::RobotSystem, hardware_interface::SystemInterface)
