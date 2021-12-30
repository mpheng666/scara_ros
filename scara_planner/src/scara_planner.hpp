#ifndef SCARA_PLANNER_HPP
#define SCARA_PLANNER_HPP

#define DEBUG_

#include <unistd.h>
#include <iostream>
#include <string>
#include <map>
#include <math.h>
#include <time.h>
#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

// #include <boost/thread.hpp>

#include <std_msgs/Float32.h>
#include <geometry_msgs/Point.h>
#include <scara_planner/TrajectoryJoints.h>

class ScaraPlanner
{
public:
    ScaraPlanner();
    ~ScaraPlanner();
    void startProcess();
    ros::NodeHandle _nh;

private:

    // ros::AsyncSpinner spinner(1);
    // spinner.start();
    const double LOOP_RATE = 10.0f;

    ros::Publisher traj_list_pub_;
    ros::Publisher traj_goal_pub_;
    ros::Publisher current_pose_pub_;
    ros::Subscriber current_joint_sub_;

    geometry_msgs::Point currentPose_;
    geometry_msgs::Point previousPose_;

    scara_planner::TrajectoryJoints currentJoint_;
    scara_planner::TrajectoryJoints prevJoint_;

    geometry_msgs::Point prevGoal_;
    geometry_msgs::Point currentGoal_;

    void joints_callback(const scara_planner::TrajectoryJoints &msg);
    void startMoveGroupInterface();
    void loadParam();
    void getCurrentJoint();
    void getCurrentPose();
    void getInverseKinematics();
    void getForwardKinematics();
    std::vector<float> getTraj(std::vector<float> &A, std::vector<float> &B);
};

#endif
