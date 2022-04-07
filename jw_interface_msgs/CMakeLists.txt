cmake_minimum_required(VERSION 3.5)
project(jw_interface_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

## Generate messages in the 'msg' folder
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ModeCommand.msg"
  "msg/MotorRPMCommand.msg"
  "msg/Command.msg"
  "msg/CommandStamped.msg"
  "msg/TimeStampStatus.msg"
  "msg/MotorRPMStatus.msg"
  "msg/BatteryStatus.msg"
  "msg/SpeedSettingStatus.msg"
# spell-checker:disable
  "msg/JSADCommand.msg"
  "msg/JSADStatus.msg"
# spell-checker:enable
  "msg/Status.msg"
  "msg/StatusStamped.msg"
  DEPENDENCIES std_msgs
)

ament_package()
