#ifndef COBO_ARM_CONTROL_ARDUINO_COMMS_HPP
#define COBO_ARM_CONTROL_ARDUINO_COMMS_HPP

#include <cmath>
#include <string>
#include <vector>
#include <libserial/SerialPort.h>
#include "rclcpp/rclcpp.hpp"

namespace cobo_arm_control
{

// Hardware command indices
enum CommandIndices {
  CMD_TOWER_PUMP_ON = 0,
  CMD_TOWER_AUTO_MODE = 1,
  CMD_TOWER_VALVE_OPEN = 2,
  CMD_TOWER_CALIBRATE = 3,
  CMD_TOWER_RESET = 4,
  CMD_RIGHT_ARM_SERVO = 5,
  CMD_RIGHT_ARM_VACUUM_ON = 6,
  CMD_RIGHT_ARM_ATTACH_MODE = 7,
  CMD_RIGHT_ARM_LOCK = 8,
  CMD_RIGHT_ARM_RESET = 9,
  CMD_LEFT_DOOR_POSITION = 10,
  CMD_LEFT_ARM_LOCK = 11,
  CMD_LEFT_ARM_RESET = 12

};

// Hardware state indices
enum StateIndices {
  ST_TOWER_MOVING = 0,
  ST_TOWER_ERROR = 1,
  ST_TOWER_PUMP_ON = 2,
  ST_TOWER_AUTO_MODE = 3,
  ST_TOWER_VALVE_OPEN = 4,
  ST_TOWER_TANK_PRESSURE = 5,
  ST_RIGHT_ARM_MOVING = 6,
  ST_RIGHT_ARM_ERROR = 7,
  ST_RIGHT_ARM_SERVO = 8,
  ST_RIGHT_ARM_VACUUM_ON = 9,
  ST_RIGHT_ARM_ATTACH_MODE = 10,
  ST_RIGHT_ARM_ATTACH_STATUS = 11,
  ST_RIGHT_ARM_VACUUM_LEVEL = 12,
  ST_LEFT_ARM_MOVING = 13,
  ST_LEFT_ARM_ERROR = 14,
  ST_LEFT_DOOR_POSITION = 15,
};

// Constants
constexpr double EPSILON = 1e-6;
constexpr size_t LEFT_ARM_PORT = 0;
constexpr size_t RIGHT_ARM_PORT = 1;

// Utility functions
LibSerial::BaudRate convert_baud_rate(int baud_rate);
int radians_to_degrees(double radians);
double degrees_to_radians(int degrees);
int adjust_degrees(int degrees);
bool floats_equal(double a, double b, double epsilon = EPSILON);

class ArduinoComms
{
public:
  ArduinoComms();
  ~ArduinoComms() = default;

  void set_logger(rclcpp::Logger logger);
  void connect(const std::string &serial_device, uint32_t baud_rate, uint32_t timeout_ms);
  void disconnect();
  bool connected() const;

  std::string send_msg(const std::string &msg_to_send, bool print_output = false, uint32_t t_out = 0);
  void send_empty_msg();

  // Tower operations
  void read_tower_info(double &val_1, double &val_2, std::vector<double> &hw_states);
  void set_tower_position(double val_1, double val_2);
  void set_tower_parameters(std::vector<double> &hw_commands);
  void calibrate_tower();
  void tower_set_pump_on(int on_off);
  void tower_set_fake_mode(int on_off);
  void tower_set_air_on(int on_off);
  void tower_set_auto_mode(int on_off);
  void tower_reset();
  

  // Arm operations
  void read_arm_info(double &val_1, double &val_2, double &val_3, std::vector<double> &hw_states);
  void set_arm_position(double val_1, double val_2, double val_3);
  void set_arm_parameters(std::vector<double> &hw_commands);
  void arm_set_attach_mode(int on_off);
  void arm_set_vacuum_on(int on_off);
  void arm_set_fake_mode(int on_off);
  void arm_set_servo(int extension);
  void door_set_position(int open_close);
  void set_left_arm();
  void arm_reset();
  void arm_lock(int onOff);


private:
  LibSerial::SerialPort serial_conn_;
  rclcpp::Logger logger_;
  bool logger_initialized_;

  double previous_arm_cmd_1_;
  double previous_arm_cmd_2_;
  double previous_arm_cmd_3_;
  double previous_tower_cmd_1_;
  double previous_tower_cmd_2_;

  std::vector<double> last_hw_commands_;

  uint32_t timeout_ms_;
  bool rightArm_;
};

}  // namespace cobo_arm_control

#endif  // COBO_ARM_CONTROL_ARDUINO_COMMS_HPP
