#include "ros/ros.h"
#include "scara_kinematics/ScaraFk.h"
#include "scara_kinematics/ScaraIk.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scara_kinematics_client_node");

    ros::NodeHandle n;
    ros::ServiceClient fk_client = n.serviceClient<scara_kinematics::ScaraFk>("/scara_forward_kinematics_server");
    ros::ServiceClient ik_client = n.serviceClient<scara_kinematics::ScaraIk>("/scara_inverse_kinematics_server");
    scara_kinematics::ScaraFk scarafk_srv;
    scara_kinematics::ScaraIk scaraik_srv;

    scarafk_srv.request.header.frame_id = "end_effector_frame";
    scarafk_srv.request.header.stamp = ros::Time::now();
    for(int i=0; i<3; ++i)
    {
        scarafk_srv.request.joint_name.emplace_back("joint " + std::to_string(i));
        scarafk_srv.request.joint_position.emplace_back(0.1);
    }

    if (fk_client.call(scarafk_srv))
    {
        ROS_INFO("Calling scarafk_srv");
    }
    else
    {
        ROS_ERROR("Failed to call service scarafk_srv");
        return 1;
    }

    geometry_msgs::Pose target_pose;
    target_pose.position.x = 0.3;
    target_pose.position.y = 0.2;
    target_pose.position.z = -0.3;
    target_pose.orientation.w = 1.0;

    scaraik_srv.request.end_effector_pose.header.frame_id = "end_effector_frame";
    scaraik_srv.request.end_effector_pose.header.stamp = ros::Time::now();
    scaraik_srv.request.end_effector_pose.pose = target_pose;

    if (ik_client.call(scaraik_srv))
    {
        ROS_INFO("Calling scaraik_srv");
    }
    else
    {
        ROS_ERROR("Failed to call service scaraik_srv");
        return 1;
    }

    return 0;
}