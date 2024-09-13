cmake_minimum_required(VERSION 3.5)
project(jw_interface_awiv_adapter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(autoware_vehicle_info_utils REQUIRED)
include_directories(${autoware_vehicle_info_utils_INCLUDE_DIRS})
message(SergioTest="${autoware_vehicle_info_utils_INCLUDE_DIRS}")
# sender
ament_auto_add_library(jw_interface_awiv_adapt_sender SHARED
  node/jw_interface_awiv_adapt_sender/jw_interface_awiv_adapt_sender.cpp
)

rclcpp_components_register_node(jw_interface_awiv_adapt_sender
  PLUGIN "JwInterfaceAWIVAdaptSender"
  EXECUTABLE jw_interface_awiv_adapt_sender_node
)

# receiver
ament_auto_add_library(jw_interface_awiv_adapt_receiver SHARED
  node/jw_interface_awiv_adapt_receiver/jw_interface_awiv_adapt_receiver.cpp
)

rclcpp_components_register_node(jw_interface_awiv_adapt_receiver
  PLUGIN "JwInterfaceAWIVAdaptReceiver"
  EXECUTABLE jw_interface_awiv_adapt_receiver_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
