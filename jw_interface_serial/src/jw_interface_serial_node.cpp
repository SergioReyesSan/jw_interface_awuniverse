#include "jw_interface_serial/jw_interface_serial_core.h"


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<JwInterfaceSerial>());
  rclcpp::shutdown();

  return 0;
}
