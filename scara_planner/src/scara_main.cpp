#include <ros/ros.h>
#include "scara.hpp"

const std::string ROS_NODE = "scara_node";
const std::string PUB_TOPIC_ODOM = "scara_odom";
const std::string SUB_TOPIC_CMDVEL = "scara_cmd_vel";
const std::string SUB_TOPIC_INITPOSE = "scara_initpose";

int main(int argc, char **argv)
{
    ros::init(argc, argv, ROS_NODE);

    scara_ns::Scara scara_node(PUB_TOPIC_ODOM, SUB_TOPIC_CMDVEL, SUB_TOPIC_INITPOSE);
    scara_node.startProcess();

    ros::spin();
    return 0;
}