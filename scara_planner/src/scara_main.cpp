#include <ros/ros.h>
#include "scara_planner.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scara_planner_node");

    ScaraPlanner scara_planner_node;
    scara_planner_node.startProcess();

    ros::spin();
    return 0;
}