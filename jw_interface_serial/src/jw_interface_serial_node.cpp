#include <ros/ros.h>

#include "jw_interface_serial/jw_interface_serial_core.h"


int main(int argc, char *argv[]) {
  ros::init(argc, argv, "jw_interface_serial");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  JwInterfaceSerial node(nh, private_nh);

  node.run();

  return 0;
}
