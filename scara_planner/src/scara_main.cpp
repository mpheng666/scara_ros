#include <ros/ros.h>
#include "scara_planner.hpp"

const std::string ROS_NODE = "scara_planner_node";

int main(int argc, char **argv)
{
    ros::init(argc, argv, ROS_NODE);

    scara_ns::ScaraPlanner scara_planner();
    scara_planner.startProcess();

    ros::spin();
    return 0;
}