#include "cobo_arm_control/arduino_comms.hpp"
#include <sstream>
#include <iostream>

namespace cobo_arm_control
{

// Utility functions
LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  switch (baud_rate)
  {
    case 1200: return LibSerial::BaudRate::BAUD_1200;
    case 1800: return LibSerial::BaudRate::BAUD_1800;
    case 2400: return LibSerial::BaudRate::BAUD_2400;
    case 4800: return LibSerial::BaudRate::BAUD_4800;
    case 9600: return LibSerial::BaudRate::BAUD_9600;
    case 19200: return LibSerial::BaudRate::BAUD_19200;
    case 38400: return LibSerial::BaudRate::BAUD_38400;
    case 57600: return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;
    default:
      std::cerr << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
      return LibSerial::BaudRate::BAUD_57600;
  }
}

int radians_to_degrees(double radians)
{
  return static_cast<int>((180.0 / M_PI) * radians);
}

double degrees_to_radians(int degrees)
{
  return (M_PI / 180.0) * static_cast<double>(degrees);
}

int adjust_degrees(int degrees)
{
  if (degrees < 0)
    return degrees + 360;
  return degrees;
}

bool floats_equal(double a, double b, double epsilon)
{
  return std::fabs(a - b) < epsilon;
}

// ArduinoComms class implementation
ArduinoComms::ArduinoComms()
  : logger_(rclcpp::get_logger("ArduinoComms")),
    logger_initialized_(true),
    previous_arm_cmd_1_(-1.0),
    previous_arm_cmd_2_(-1.0),
    previous_arm_cmd_3_(-1.0),
    previous_tower_cmd_1_(-1.0),
    previous_tower_cmd_2_(-1.0),
    timeout_ms_(0),
    rightArm_(true)
{
  last_hw_commands_.assign(CMD_LEFT_DOOR_POSITION + 1, 0.0);
}

void ArduinoComms::set_logger(rclcpp::Logger logger)
{
  logger_ = logger;
  logger_initialized_ = true;
}

void ArduinoComms::connect(const std::string &serial_device, uint32_t baud_rate, uint32_t timeout_ms)
{
  if (logger_initialized_)
  {
    RCLCPP_INFO(logger_, "Connecting to %s", serial_device.c_str());
  }

  timeout_ms_ = timeout_ms;
  serial_conn_.Open(serial_device);
  serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
  RCLCPP_INFO(logger_, "Connected to %s", serial_device.c_str());
}

void ArduinoComms::disconnect()
{
  serial_conn_.Close();
}

bool ArduinoComms::connected() const
{
  return serial_conn_.IsOpen();
}

std::string ArduinoComms::send_msg(const std::string &msg_to_send, bool print_output, uint32_t t_out)
{
  serial_conn_.FlushIOBuffers();
  serial_conn_.Write(msg_to_send + "\r");

  std::string response = "";
  try
  {
    serial_conn_.ReadLine(response, '\n', t_out);
  }
  catch (const LibSerial::ReadTimeout&)
  {
    if (logger_initialized_)
    {
      RCLCPP_WARN(logger_, "ReadLine timed out for message: %s", msg_to_send.c_str());
    }
    else
    {
      std::cerr << "The ReadLine() call has timed out for message: " << msg_to_send << std::endl;
    }
  }

  if (print_output)
  {
    if (logger_initialized_)
    {
      RCLCPP_INFO(logger_, "Sent: %s Recv: %s", msg_to_send.c_str(), response.c_str());
    }
    else
    {
      std::cout << "Sent: " << msg_to_send << " Recv: " << response << std::endl;
    }
  }

  return response;
}

void ArduinoComms::send_empty_msg()
{
  send_msg("\r");
}

void ArduinoComms::read_tower_info(double &val_1, double &val_2, std::vector<double> &hw_states)
{
  int rT_pos, lT_pos, p1, p2, p3, p4,p5,p6;

  std::string response = send_msg("s");
  const int ret = std::sscanf(response.c_str(), "%d%d%d%d%d%d%d%d", &rT_pos, &lT_pos, &p1, &p2, &p3, &p4,&p5, &p6);

  if (ret == 8)
  {
    hw_states[ST_TOWER_MOVING] = p1;
    hw_states[ST_TOWER_ERROR] = p2;
    hw_states[ST_TOWER_PUMP_ON] = p3;
    hw_states[ST_TOWER_AUTO_MODE] = p4;
    hw_states[ST_TOWER_VALVE_OPEN] = p5;
    hw_states[ST_TOWER_TANK_PRESSURE] = p6;

    val_1 = static_cast<double>(rT_pos) / 1000.0;  // tower uses mm, convert to meters
    val_2 = static_cast<double>(lT_pos) / 1000.0;
  }
  else if (logger_initialized_)
  {
    RCLCPP_WARN(logger_, "Failed to parse tower info response: %s", response.c_str());
  }
}

void ArduinoComms::set_tower_position(double val_1, double val_2)
{
  val_1 *= 1000.0;  // convert from m to mm
  val_2 *= 1000.0;

  if (floats_equal(val_1, previous_tower_cmd_1_) && floats_equal(val_2, previous_tower_cmd_2_))
  {
    return;
  }

  previous_tower_cmd_1_ = val_1;
  previous_tower_cmd_2_ = val_2;

  if (logger_initialized_)
  {
    RCLCPP_DEBUG(logger_, "Set tower pos: %d %d", static_cast<int>(val_1), static_cast<int>(val_2));
  }

  std::stringstream ss;
  ss << "p " << static_cast<int>(val_1) << " " << static_cast<int>(val_2);
  send_msg(ss.str(), true);
}

void ArduinoComms::set_tower_parameters(std::vector<double> &hw_commands)
{
  if (!floats_equal(hw_commands[CMD_TOWER_CALIBRATE], last_hw_commands_[CMD_TOWER_CALIBRATE]))
  {
  
    std::cout << "TC " << hw_commands[CMD_TOWER_CALIBRATE] << " " << last_hw_commands_[CMD_TOWER_CALIBRATE] << std::endl;
  
    calibrate_tower();
    last_hw_commands_[CMD_TOWER_CALIBRATE] =hw_commands[CMD_TOWER_CALIBRATE]; //
  }

  if (!floats_equal(hw_commands[CMD_TOWER_PUMP_ON], last_hw_commands_[CMD_TOWER_PUMP_ON]))
  {
    tower_set_pump_on(static_cast<int>(hw_commands[CMD_TOWER_PUMP_ON]));
    last_hw_commands_[CMD_TOWER_PUMP_ON] = hw_commands[CMD_TOWER_PUMP_ON];
  }

  if (!floats_equal(hw_commands[CMD_TOWER_AUTO_MODE], last_hw_commands_[CMD_TOWER_AUTO_MODE]))
  {
    tower_set_auto_mode(static_cast<int>(hw_commands[CMD_TOWER_AUTO_MODE]));
    last_hw_commands_[CMD_TOWER_AUTO_MODE] = hw_commands[CMD_TOWER_AUTO_MODE];
  }

  if (!floats_equal(hw_commands[CMD_TOWER_VALVE_OPEN], last_hw_commands_[CMD_TOWER_VALVE_OPEN]))
  {
    tower_set_air_on(static_cast<int>(hw_commands[CMD_TOWER_VALVE_OPEN]));
    last_hw_commands_[CMD_TOWER_VALVE_OPEN] = hw_commands[CMD_TOWER_VALVE_OPEN];
  }

  if (!floats_equal(hw_commands[CMD_TOWER_RESET], last_hw_commands_[CMD_TOWER_RESET ]))
  {
    tower_reset();
    last_hw_commands_[CMD_TOWER_RESET] = hw_commands[CMD_TOWER_RESET];
    
  }

}

void ArduinoComms::calibrate_tower()
{
  if (logger_initialized_)
  {
    RCLCPP_INFO(logger_, "Calibrating tower");
  }

  std::stringstream ss;
  ss << "c";
  send_msg(ss.str(), false, 20000);  // long timeout
}

void ArduinoComms::tower_reset()
{
  if (1) //logger_initialized_)
  {
    RCLCPP_INFO(logger_, "Resetting tower");
  }

  std::stringstream ss;
  ss << "c";
  send_msg(ss.str(), false, 20000);  // long timeout
}

void ArduinoComms::set_arm_position(double val_1, double val_2, double val_3)
{

  if (floats_equal(val_1, previous_arm_cmd_1_) &&
      floats_equal(val_2, previous_arm_cmd_2_) &&
      floats_equal(val_3, previous_arm_cmd_3_))
  {
    return;
  }
 // RCLCPP_INFO(logger_, "AWT %f %f %f", val_1, val_2, val_3);
    
  previous_arm_cmd_1_ = val_1;
  previous_arm_cmd_2_ = val_2;
  previous_arm_cmd_3_ = val_3;

  if (1) //logger_initialized_)
  {
    const char* arm_name = rightArm_ ? "right" : "left";
    RCLCPP_INFO(logger_, "Set %s arm pos: %.3f %.3f %.3f", arm_name, val_1, val_2, val_3);
  }

  int d_val_1 = adjust_degrees(radians_to_degrees(val_1));
  int d_val_2 = adjust_degrees(radians_to_degrees(val_2));
  int d_val_3 = adjust_degrees(radians_to_degrees(val_3));

  if (1) // logger_initialized_)
  {
    RCLCPP_INFO(logger_, "Degrees: %d %d %d", d_val_1, d_val_2, d_val_3);
  }

  std::stringstream ss;
  ss << "p " << d_val_1 << " " << d_val_2 << " " << d_val_3;
  send_msg(ss.str(), false);
}

void ArduinoComms::set_arm_parameters(std::vector<double> &hw_commands)
{
  
  
  if (rightArm_)
  {
    if (!floats_equal(hw_commands[CMD_RIGHT_ARM_SERVO], last_hw_commands_[CMD_RIGHT_ARM_SERVO]))
    {
      arm_set_servo(static_cast<int>(hw_commands[CMD_RIGHT_ARM_SERVO]));
      last_hw_commands_[CMD_RIGHT_ARM_SERVO] = hw_commands[CMD_RIGHT_ARM_SERVO];
    }

    if (!floats_equal(hw_commands[CMD_RIGHT_ARM_VACUUM_ON], last_hw_commands_[CMD_RIGHT_ARM_VACUUM_ON]))
    {
      arm_set_vacuum_on(static_cast<int>(hw_commands[CMD_RIGHT_ARM_VACUUM_ON]));
      last_hw_commands_[CMD_RIGHT_ARM_VACUUM_ON] = hw_commands[CMD_RIGHT_ARM_VACUUM_ON];
    }

    if (!floats_equal(hw_commands[CMD_RIGHT_ARM_ATTACH_MODE], last_hw_commands_[CMD_RIGHT_ARM_ATTACH_MODE]))
    {
      arm_set_attach_mode(static_cast<int>(hw_commands[CMD_RIGHT_ARM_ATTACH_MODE]));
      last_hw_commands_[CMD_RIGHT_ARM_ATTACH_MODE] = hw_commands[CMD_RIGHT_ARM_ATTACH_MODE];
    }

    if (!floats_equal(hw_commands[CMD_RIGHT_ARM_RESET], last_hw_commands_[CMD_RIGHT_ARM_RESET]))
    {
      arm_reset();
      last_hw_commands_[CMD_RIGHT_ARM_RESET] = hw_commands[CMD_RIGHT_ARM_RESET];
    }

    if (!floats_equal(hw_commands[CMD_RIGHT_ARM_LOCK], last_hw_commands_[CMD_RIGHT_ARM_LOCK]))
    {
      arm_lock(static_cast<int>(hw_commands[CMD_RIGHT_ARM_LOCK]));
      last_hw_commands_[CMD_RIGHT_ARM_LOCK] = hw_commands[CMD_RIGHT_ARM_LOCK];
    }

  }
  else
  {
    if (!floats_equal(hw_commands[CMD_LEFT_DOOR_POSITION], last_hw_commands_[CMD_LEFT_DOOR_POSITION]))
    {
      door_set_position(static_cast<int>(hw_commands[CMD_LEFT_DOOR_POSITION]));
      last_hw_commands_[CMD_LEFT_DOOR_POSITION] = hw_commands[CMD_LEFT_DOOR_POSITION];
    }

    if (!floats_equal(hw_commands[CMD_LEFT_ARM_RESET], last_hw_commands_[CMD_LEFT_ARM_RESET]))
    {
      arm_reset();
      last_hw_commands_[CMD_LEFT_ARM_RESET] = hw_commands[CMD_LEFT_ARM_RESET];
    }

    if (!floats_equal(hw_commands[CMD_LEFT_ARM_LOCK], last_hw_commands_[CMD_LEFT_ARM_LOCK]))
    {
      arm_lock(static_cast<int>(hw_commands[CMD_LEFT_ARM_LOCK]));
      last_hw_commands_[CMD_LEFT_ARM_LOCK] = hw_commands[CMD_LEFT_ARM_LOCK];
    }
  }
}

void ArduinoComms::read_arm_info(double &val_1, double &val_2, double &val_3, std::vector<double> &hw_states)
{
  int j1, j2, j3, p1, p2, p3, p4, p5,p6,p7;
  
  std::string response = send_msg("s\r");
  const int ret = std::sscanf(response.c_str(), "%d%d%d%d%d%d%d%d%d%d", &j1, &j2, &j3, &p1, &p2, &p3, &p4, &p5, &p6, &p7);

  val_1 = degrees_to_radians(j1);
  val_2 = degrees_to_radians(j2);
  val_3 = degrees_to_radians(j3);


  if (rightArm_ && (ret == 10)) 
  {
    hw_states[ST_RIGHT_ARM_MOVING] = p1;
    hw_states[ST_RIGHT_ARM_ERROR] = p2;
    hw_states[ST_RIGHT_ARM_SERVO] = p3;
    hw_states[ST_RIGHT_ARM_VACUUM_ON] = p4;
    hw_states[ST_RIGHT_ARM_ATTACH_MODE] = p5;
    hw_states[ST_RIGHT_ARM_VACUUM_LEVEL] = p6;
    hw_states[ST_RIGHT_ARM_ATTACH_STATUS] = p7;

    if (logger_initialized_)
    {
      RCLCPP_DEBUG(logger_, "Right arm pos: %.3f %.3f %.3f", val_1, val_2, val_3);
    }
    return;
  } 

  if (!rightArm_ && (ret == 6)) 
  {
    hw_states[ST_LEFT_ARM_MOVING] = p1;
    hw_states[ST_LEFT_ARM_ERROR] = p2;
    hw_states[ST_LEFT_DOOR_POSITION] = p3;

      if (logger_initialized_)
      {
        RCLCPP_DEBUG(logger_, "Left arm pos: %.3f %.3f %.3f", val_1, val_2, val_3);
      }
      return;
  }
  
  if (logger_initialized_)
  {
    RCLCPP_WARN(logger_, "Failed to parse arm info response: %s", response.c_str());
  }
}

void ArduinoComms::tower_set_pump_on(int on_off)
{
  if (logger_initialized_)
  {
    RCLCPP_INFO(logger_, "Set tower pump: %d", on_off);
  }

  std::stringstream ss;
  ss << "k " << on_off;
  send_msg(ss.str(), true);
}

void ArduinoComms::tower_set_fake_mode(int on_off)
{
  if (logger_initialized_)
  {
    RCLCPP_INFO(logger_, "Set tower fake mode: %d", on_off);
  }

  std::stringstream ss;
  ss << "f " << on_off;
  send_msg(ss.str(), true);
}

void ArduinoComms::tower_set_air_on(int on_off)
{
  if (logger_initialized_)
  {
    RCLCPP_INFO(logger_, "Set tower air on: %d", on_off);
  }

  std::stringstream ss;
  ss << "o " << on_off;
  send_msg(ss.str(), true);
}

void ArduinoComms::tower_set_auto_mode(int on_off)
{
  if (logger_initialized_)
  {
    RCLCPP_INFO(logger_, "Set tower auto mode: %d", on_off);
  }

  std::stringstream ss;
  ss << "m " << on_off;
  send_msg(ss.str(), true);
}



void ArduinoComms::arm_set_attach_mode(int on_off)
{
  if (logger_initialized_)
  {
    RCLCPP_INFO(logger_, "Set arm attach mode: %d", on_off);
  }

  std::stringstream ss;
  ss << "g " << on_off;
  send_msg(ss.str(), true);
}

void ArduinoComms::arm_set_vacuum_on(int on_off)
{
  if (logger_initialized_)
  {
    RCLCPP_INFO(logger_, "Set arm vacuum: %d", on_off);
  }

  std::stringstream ss;
  ss << "v " << on_off;
  send_msg(ss.str(), true);
}

void ArduinoComms::arm_set_fake_mode(int on_off)
{
  
 if (logger_initialized_)
  {
    RCLCPP_INFO(logger_, "Set arm fake mode: %d", on_off);
  }

  std::stringstream ss;
  ss << "f " << on_off;
  send_msg(ss.str(), true);
}

void ArduinoComms::arm_reset()
{
  
 if (1) // logger_initialized_)
  {
    RCLCPP_INFO(logger_, "Reset arm");
  }

  std::stringstream ss;
  ss << "r " ;
  send_msg(ss.str(), true);
}

void ArduinoComms::arm_lock(int onOff)
{
  
 if (1) //dlogger_initialized_)
  {
    RCLCPP_INFO(logger_, "Lock arms %d" , onOff);
  }

  std::stringstream ss;
  ss << "k " << onOff;
  send_msg(ss.str(), true);
  
}

void ArduinoComms::door_set_position(int open_close)
{
  if (logger_initialized_)
  {
    RCLCPP_INFO(logger_, "Set door position: %d", open_close);
  }

  std::stringstream ss;
  ss << "j " << open_close;
  send_msg(ss.str(), true);
}

void ArduinoComms::arm_set_servo(int extension)
{
  if (logger_initialized_)
  {
    RCLCPP_INFO(logger_, "Set arm servo: %d", extension);
  }

  std::stringstream ss;
  ss << "m " << extension;
  send_msg(ss.str(), true);
}

void ArduinoComms::set_left_arm()
{

if (logger_initialized_)
  {
    RCLCPP_INFO(logger_, "Setting left arm mode");
  }

  std::stringstream ss;
  ss << "a l";
  send_msg(ss.str(), true);
  rightArm_ = false;
}

}  // namespace cobo_arm_control
