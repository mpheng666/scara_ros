#include "scara_planner.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scara_planner_node");

    scara_ns::ScaraPlanner scara_planner_node;
    scara_planner_node.startProcess();

    return 0;
}