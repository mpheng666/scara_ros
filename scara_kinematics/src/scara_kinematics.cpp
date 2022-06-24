#include "scara_kinematics/scara_kinematics.hpp"

using namespace scara_kinematics_ns;

    
ScaraKinematics::ScaraKinematics(ros::NodeHandle *nh)
{
    ik_service_ = nh->advertiseService("/scara_inverse_kinematics_server", &ScaraKinematics::ikServiceCb, this);
    fk_service_ = nh->advertiseService("/scara_forward_kinematics_server", &ScaraKinematics::fkServiceCb, this);
}

ScaraKinematics::~ScaraKinematics()
{
}

void ScaraKinematics::loadParams()
{
    if (!ros::param::param<double>("/scara_kinematics_node/link_1_length", link_1_length_, DEFAULT_LINK_1_LENGTH))
    {
        ROS_WARN("Link 1 length is not set. Using %f as default", DEFAULT_LINK_1_LENGTH);
    }
    if (!ros::param::param<double>("/scara_kinematics_node/link_2_length", link_2_length_, DEFAULT_LINK_2_LENGTH))
    {
        ROS_WARN("Link 2 length is not set. Using %f as default", DEFAULT_LINK_2_LENGTH);
    }
}

bool ScaraKinematics::forwardKinematics(std::vector<double> joints, geometry_msgs::PoseStamped &target_pose)
{
    ROS_INFO("forward kinematics");
    return true;
}

bool ScaraKinematics::inverseKinematics(geometry_msgs::PoseStamped &pose, std::vector<double> &target_joints)
{
    ROS_INFO("inverse kinematics");
    return true;
}

bool ScaraKinematics::fkServiceCb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO("Service fk called!");
    // this->forwardKinematics();
    return true;
}

bool ScaraKinematics::ikServiceCb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO("Service ik called!");
    // this->inverseKinematics();
    return true;
}

void ScaraKinematics::start()
{
    ros::Rate r(LOOP_RATE);

    while(ros::ok())
    {
        ros::spin();
        r.sleep();
    }
}