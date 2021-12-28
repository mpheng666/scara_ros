#include "ros/ros.h"
#include "scara_joy.hpp"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "scara_teleop_node");

  ros::NodeHandle nh(""), nh_param("~");
  scara_joy::ScaraJoy scara_joy(&nh, &nh_param);

  ros::spin();
}
