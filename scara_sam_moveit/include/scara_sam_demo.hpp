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
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PoseStamped.h>

#include <scara_planner/scara.h>

using std::cin;
using std::cout;
using std::endl;
using std::string;
using std::stringstream;

int reset_ = 0, record_ = 0, playback_ = 0, enable_ = 0;
double x_ = 0.0, y_ = 0.0;
double z_ = 0.0;
double joint_1_ = 0.0, joint_2_ = 0.0;

ros::Publisher joint_list_pub;
ros::Subscriber scara_sub;
ros::Subscriber cartesian_control_sub;
ros::Subscriber joint_control_sub;

void scaraCB(const scara_planner::scara::ConstPtr &msg)
{
    reset_ = msg->reset;
    record_ = msg->record;
    playback_ = msg->playback;
    enable_ = msg->enable;
    joint_1_ = msg->joint_1;
    joint_2_ = msg->joint_2;
    x_ = msg->x;
    y_ = msg->y;
    z_ = msg->z;
}

void cartesianCB(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
}

void jointCB(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
}