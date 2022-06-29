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

bool ScaraKinematics::forwardKinematics(const std::vector<double>& target_joints, geometry_msgs::PoseStamped& pose)
{
    pose.header.frame_id = "end_effector_frame";
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x = (-link_1_length_*sin(target_joints.at(0))) + (-link_2_length_*sin(target_joints.at(0)+target_joints.at(1)));
    pose.pose.position.y = (link_1_length_*cos(target_joints.at(0))) + (link_2_length_*cos(target_joints.at(0)+target_joints.at(1)));
    pose.pose.position.z = link_0_height_ - target_joints.at(2);
    ROS_INFO("Result pose_frame: %s", pose.header.frame_id.c_str());
    ROS_INFO("Result pose_x: %f", pose.pose.position.x);
    ROS_INFO("Result pose_y: %f", pose.pose.position.y);
    ROS_INFO("Result pose_z: %f", pose.pose.position.z);
    return true;
}

bool ScaraKinematics::inverseKinematics(const geometry_msgs::PoseStamped &target_pose, std::vector<double>& joints)
{
    double theta_zx = atan(target_pose.pose.position.z/target_pose.pose.position.x);
    double resultant_length = sqrt(target_pose.pose.position.z*target_pose.pose.position.z + target_pose.pose.position.x*target_pose.pose.position.x);
    double theta = acos((-resultant_length*resultant_length + link_1_length_*link_1_length_ + link_2_length_*link_2_length_)/(2*link_1_length_*link_2_length_));
    double alpha = (M_1_PI-theta)/2;

    joints.at(0) = theta_zx - alpha;
    joints.at(1) = M_1_PI-theta;
    joints.at(2) = target_pose.pose.position.z + link_0_height_;

    ROS_INFO("Target pose_frame: %s", target_pose.header.frame_id.c_str());
    for(int i=0; i<joints.size(); ++i)
    {
        ROS_INFO("Result joint %d: %f", i, joints.at(i));
    }
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
    geometry_msgs::PoseStamped pose;
    bool success = this->forwardKinematics(request.joint_position, pose);
    response.end_effector_pose = pose;
    if(success)
    {
        ROS_INFO("Forward kinematics calculation completed");
    }
    else
    {
        ROS_INFO("Forward kinematics calculation failed");
    }
    return success;
}

bool ScaraKinematics::ikServiceCb(scara_kinematics::ScaraIk::Request& request, scara_kinematics::ScaraIk::Response& response)
{
    ROS_INFO("Request scara inverse kinematics with pose:");
    ROS_INFO("Target pose_frame: %s", request.end_effector_pose.header.frame_id.c_str());
    ROS_INFO("Target pose_x: %f", request.end_effector_pose.pose.position.x);
    ROS_INFO("Target pose_y: %f", request.end_effector_pose.pose.position.y);
    ROS_INFO("Target pose_z: %f", request.end_effector_pose.pose.position.z);

    std::vector<double> joints;
    joints.resize(3);

    bool success = this->inverseKinematics(request.end_effector_pose, joints);
    
    if(success)
    {
        ROS_INFO("Inverse kinematics calculation completed");
    }
    else
    {
        ROS_INFO("Inverse kinematics calculation failed");
    }
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