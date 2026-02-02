cmake_minimum_required(VERSION 3.16)
project(cobo_arm_control LANGUAGES C CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# set the same behavior for windows as it is on linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# find dependencies
find_package(ament_cmake REQUIRED)

set(HW_IF_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  hardware_interface
)
set(REF_GEN_INCLUDE_DEPENDS
  kdl_parser
  rclcpp
  trajectory_msgs
  std_msgs
)
set(CONTROLLER_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  rclcpp
  controller_interface
  realtime_tools
  trajectory_msgs
  std_msgs
)

# find dependencies
find_package(backward_ros REQUIRED)
#find_package(serial REQUIRED)
foreach(Dependency IN ITEMS ${HW_IF_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
foreach(Dependency IN ITEMS ${REF_GEN_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
foreach(Dependency IN ITEMS ${CONTROLLER_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


## COMPILE
add_executable(send_trajectory reference_generator/send_trajectory.cpp)

ament_target_dependencies(
  send_trajectory PUBLIC
  ${REF_GEN_INCLUDE_DEPENDS}
)

add_library(
  cobo_arm_control_lib
  SHARED
  hardware/cobo_hardware.cpp
  hardware/arduino_comms.cpp
  controller/cobo_controller.cpp
  controller/ik_solver.cpp
)

target_compile_features(cobo_arm_control_lib PUBLIC cxx_std_17)
target_include_directories(cobo_arm_control_lib PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/controller/include>
$<INSTALL_INTERFACE:include/cobo_arm_control>
)
ament_target_dependencies(
  cobo_arm_control_lib PUBLIC
  ${HW_IF_INCLUDE_DEPENDS}
  ${CONTROLLER_INCLUDE_DEPENDS}
)

## Link serial library
target_link_libraries(cobo_arm_control_lib PUBLIC serial)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface cobo_arm_control.xml)
# Export controller plugins
pluginlib_export_plugin_description_file(controller_interface cobo_arm_control.xml)

# INSTALL
install(
  DIRECTORY hardware/include/ controller/include/
  DESTINATION include/cobo_arm_control
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf
  DESTINATION share/cobo_arm_control
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/cobo_arm_control
)
install(
    TARGETS send_trajectory
    RUNTIME DESTINATION lib/cobo_arm_control
)

install(TARGETS cobo_arm_control_lib
  EXPORT export_cobo_arm_control_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)



## EXPORTS
ament_export_targets(export_cobo_arm_control_lib HAS_LIBRARY_TARGET)
ament_export_dependencies(${HW_IF_INCLUDE_DEPENDS} ${REF_GEN_INCLUDE_DEPENDS} ${CONTROLLER_INCLUDE_DEPENDS})
ament_package()
