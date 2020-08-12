cmake_minimum_required(VERSION 2.8.3)
project(jw_interface_msgs)

find_package(
  catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
)


## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
  FILES
    ModeCommand.msg
    JSADCommand.msg
    MotorRPMCommand.msg

    Command.msg
    CommandStamped.msg

    TimeStampStatus.msg
    MotorRPMStatus.msg
    BatteryStatus.msg
    SpeedSettingStatus.msg
    JSADStatus.msg

    Status.msg
    StatusStamped.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
)
