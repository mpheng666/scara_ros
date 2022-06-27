#include "scara_kinematics/scara_kinematics.hpp"

using namespace scara_kinematics_ns;

    
ScaraKinematics::ScaraKinematics(ros::NodeHandle *nh):
ik_service_(nh->advertiseService("/scara_inverse_kinematics_server", &ScaraKinematics::ikServiceCb, this)),
fk_service_(nh->advertiseService("/scara_forward_kinematics_server", &ScaraKinematics::fkServiceCb, this))
{
}

ScaraKinematics::~ScaraKinematics()
{
}

void ScaraKinematics::loadParams()
{
    if (!ros::param::param<double>("/scara_kinematics_node/link_0_height", link_0_height_, DEFAULT_LINK_0_HEIGHT))
    {
        ROS_WARN("Link 0 height is not set. Using %f as default", DEFAULT_LINK_0_HEIGHT);
    }
    if (!ros::param::param<double>("/scara_kinematics_node/link_1_length", link_1_length_, DEFAULT_LINK_1_LENGTH))
    {
        ROS_WARN("Link 1 length is not set. Using %f as default", DEFAULT_LINK_1_LENGTH);
    }
    if (!ros::param::param<double>("/scara_kinematics_node/link_2_length", link_2_length_, DEFAULT_LINK_2_LENGTH))
    {
        ROS_WARN("Link 2 length is not set. Using %f as default", DEFAULT_LINK_2_LENGTH);
    }
}

bool ScaraKinematics::forwardKinematics(const std::vector<double>& joints, geometry_msgs::PoseStamped& target_pose)
{
    target_pose.header.frame_id = "end_effector_frame";
    target_pose.header.stamp = ros::Time::now();
    target_pose.pose.position.x = (-link_1_length_*sin(joints.at(0))) + (-link_2_length_*sin(joints.at(0)+joints.at(1)));
    target_pose.pose.position.y = (link_1_length_*cos(joints.at(0))) + (link_2_length_*cos(joints.at(0)+joints.at(1)));
    target_pose.pose.position.z = link_0_height_ - joints.at(2);
    ROS_INFO("Target_pose_frame: %s", target_pose.header.frame_id.c_str());
    ROS_INFO("Target_pose_x: %f", target_pose.pose.position.x);
    ROS_INFO("Target_pose_y: %f", target_pose.pose.position.y);
    ROS_INFO("Target_pose_z: %f", target_pose.pose.position.z);
    return true;
}

bool ScaraKinematics::inverseKinematics(const geometry_msgs::PoseStamped &pose, std::vector<double>& target_joints)
{

    ROS_INFO("inverse kinematics");
    return true;
}

bool ScaraKinematics::fkServiceCb(scara_kinematics::ScaraFk::Request& request, scara_kinematics::ScaraFk::Response& response)
{
    ROS_INFO("Request scara forward kinematics with joints:");
    for(int i=0; i<request.joint_position.size(); ++i)
    {
        ROS_INFO("%s: %f", request.joint_name.at(i).c_str(), request.joint_position.at(i));
    }
    geometry_msgs::PoseStamped target_pose;
    bool success = this->forwardKinematics(request.joint_position, target_pose);
    response.end_effector_pose = target_pose;
    ROS_INFO("success: %i", success);
    return success;
}

bool ScaraKinematics::ikServiceCb(scara_kinematics::ScaraIk::Request& request, scara_kinematics::ScaraIk::Response& response)
{
    ROS_INFO("Service ik called!");
    // this->inverseKinematics();
    return true;
}

void ScaraKinematics::start()
{
    ros::Rate r(LOOP_RATE);
    this->loadParams();

    while(ros::ok())
    {
        ros::spin();
        r.sleep();
    }
}