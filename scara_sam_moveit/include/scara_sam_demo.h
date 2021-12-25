#include <ros/ros.h>
#include <iostream>
#include <sstream>
#include <unistd.h>

#include <stdio.h>
#include <string.h>

#include <std_msgs/Int32.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

using std::cin;
using std::cout;
using std::endl;
using std::string;
using std::stringstream;

ros::Publisher joint_pub;
ros::Publisher mission_pub;
ros::Publisher demo_mode_pub;