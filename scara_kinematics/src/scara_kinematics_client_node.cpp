#include "ros/ros.h"
#include "scara_kinematics/ScaraFk.h"
#include "scara_kinematics/ScaraIk.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scara_kinematics_client_node");

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<scara_kinematics::ScaraFk>("/scara_forward_kinematics_server");
    scara_kinematics::ScaraFk scarafk_srv;
    scarafk_srv.request.header.frame_id = "end_effector_frame";
    scarafk_srv.request.header.stamp = ros::Time::now();
    for(int i=0; i<3; ++i)
    {
        scarafk_srv.request.joint_name.emplace_back("joint " + std::to_string(i));
        scarafk_srv.request.joint_position.emplace_back(0.1);
    }

    if (client.call(scarafk_srv))
    {
        ROS_INFO("Calling scarafk_srv");
    }
    else
    {
        ROS_ERROR("Failed to call service scarafk_srv");
        return 1;
    }

    return 0;
}