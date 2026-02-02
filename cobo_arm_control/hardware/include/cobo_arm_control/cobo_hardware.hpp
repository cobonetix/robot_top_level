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

#ifndef cobo_arm_control__cobo_HARDWARE_HPP_
#define cobo_arm_control__cobo_HARDWARE_HPP_

#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "cobo_arm_control/arduino_comms.hpp"


using hardware_interface::return_type;

namespace cobo_arm_control
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class  RobotSystem : public hardware_interface::SystemInterface
{


public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RobotSystem);
  
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

   return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

/// Get the logger of the ActuatorInterface.
  /**
   * \return logger of the ActuatorInterface.
   */
  rclcpp::Logger get_logger() const { return *logger_; }

  /// Get the clock of the ActuatorInterface.
  /**
   * \return clock of the ActuatorInterface.
   */
  rclcpp::Clock::SharedPtr get_clock() const { return clock_; }
  
  
static constexpr size_t LEFT_ARM_JOINT_OFFSET = 0;
static constexpr size_t RIGHT_ARM_JOINT_OFFSET = 4;


  	
struct Config
{
  float loop_rate = 0.0;
  std::string armPort[2] = {"", ""};
  std::string towerPort = "";

  uint32_t baud_rate = 0;
  uint32_t timeout_ms = 0;

  float initial_rotation = 1.57f;
  float initial_z = 0.7f;
  bool fakeMode = true;
};

protected:
 ArduinoComms TowerComms ;
 ArduinoComms ArmComms [2]; // 0 = left arm, 1 = right arm 
 
 Config cfg_;
 

  /// The size of this vector is (standard_interfaces_.size() x nr_joints)
  std::vector<double> joint_position_command_;
  std::vector<double> joint_velocities_command_;
  std::vector<double> joint_position_;
  std::vector<double> joint_velocities_;
  
 // Store the command and state interfaces for the simulated robot
 // hw commands: tower (pump_on(0), auto_mode(1) valve_open(2)), right_arm (servo(3), vacuum_on(4), attach_mode(5)), left_arm (door_position(6))
// hw states: tower (pump_on, auto_mode, valve_open,tank pressure), right_arm (servo, vacuum_on, attach_mode, vacuum_level, attach_status), left_arm (door_position)

  std::vector<double> hw_commands_;
  //std::vector<double> last_hw_commands_;
  std::vector<double> hw_states_;
  

 // Objects for logging
  std::shared_ptr<rclcpp::Logger> logger_;
  rclcpp::Clock::SharedPtr clock_;

  std::unordered_map<std::string, std::vector<std::string>> joint_interfaces = {
    {"position", {}}, {"velocity", {}}};
};

}  // namespace cobo_arm_control

#endif  // cobo_arm_control__cobo_HARDWARE_HPP_
