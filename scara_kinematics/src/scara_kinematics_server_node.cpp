#include <ros/ros.h>
#include "scara_kinematics/scara_kinematics.hpp"

int main(int argc, char** argv){
    ros::init(argc, argv, "scara_kinematics_server_node");
    ros::NodeHandle nh("~");

    scara_kinematics_ns::ScaraKinematics scara_kinematics(&nh);
    scara_kinematics.start();

    return 0;
}