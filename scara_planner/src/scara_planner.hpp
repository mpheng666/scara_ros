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

#include <boost/thread.hpp>
#include <Eigen/Geometry>

#include <std_msgs/Float32.h>
#include <geometry_msgs/Point.h>
#include "scara_planner/TrajectoryJoints.h"

namespace rvt = rviz_visual_tools;

namespace scara_ns
{
    class ScaraPlanner
    {
    public:
        ScaraPlanner();
        ~ScaraPlanner();
        void startProcess();
        ros::NodeHandle nh;
        ros::NodeHandle private_nh_;

    private:
        const double LOOP_RATE = 10.0f;
        ros::AsyncSpinner spinner;

        // Moveit CONSTANT
        const std::string PLANNING_GROUP = "whole_arm";

        // Visualization tools
        moveit_visual_tools::MoveItVisualTools visual_tools;
        moveit::planning_interface::MoveGroupInterface move_group;
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

        ros::Publisher traj_list_pub_;
        ros::Publisher traj_goal_pub_;
        ros::Publisher current_pose_pub_;
        ros::Subscriber current_joint_sub_;

        // ros::ServiceServer PlannerService = private_nh_.advertiseService("Get pose plan:", this->getPosePlan);

        geometry_msgs::Point currentPose_;
        geometry_msgs::Point previousPose_;

        scara_planner::TrajectoryJoints currentJoint_;
        scara_planner::TrajectoryJoints prevJoint_;

        geometry_msgs::Point prevGoal_;
        geometry_msgs::Point currentGoal_;

        void loadParam();

        void joints_callback(const scara_planner::TrajectoryJoints &msg);
        const robot_state::JointModelGroup* startMoveGroupInterface();
        void startVisualization();
        void visualizePlan(const geometry_msgs::Pose &, const moveit::planning_interface::MoveGroupInterface::Plan &, const robot_state::JointModelGroup*);

        // position goal
        geometry_msgs::Pose getPoseTarget();
        moveit::planning_interface::MoveGroupInterface::Plan getPosePlan();
        void setPoseTarget(const geometry_msgs::Pose &);
        void getGoalPosTolerance();
        void setGoalPosTolerance(const double&);
        void getGoalOrientTolerance();
        void setGoalOrientTolerance(const double&);
        void moveGoalTarget();

        // joint goal
        void getJointTarget();
        void getJointPlan();
        void setJointTarget();
        void moveJointTarget();

        void getCurrentJoint();
        geometry_msgs::Pose getCurrentPose();

        void getRobotState();

        void getInverseKinematics();
        void getForwardKinematics();
    };

} // scara_ns
#endif
