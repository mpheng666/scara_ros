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
#include <sensor_msgs/Joy.h>
#include "scara_planner/TrajectoryJoints.h"
#include "tf/transform_broadcaster.h"
#include "visualization_msgs/Marker.h"

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

        enum class Mode
        {
            cartesian_mode,
            joint_mode,
            execution_mode
        };

    private:
        const double LOOP_RATE = 20.0f;
        std::vector<double> joint_group_positions = {0, -0.785, 0, -2.356, 0, 1.571, 0.785, 0.035, 0.035};

        geometry_msgs::Pose target_pose;
        geometry_msgs::Pose current_pose;

        Mode planner_mode;
        ros::AsyncSpinner spinner;

        // Moveit CONSTANT
        const std::string PLANNING_GROUP = "whole_arm";
        // const std::string PLANNING_GROUP = "panda_arm";

        // Visualization tools
        moveit_visual_tools::MoveItVisualTools visual_tools;
        moveit::planning_interface::MoveGroupInterface move_group;
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

        ros::Publisher traj_list_pub_;
        ros::Publisher traj_goal_pub_;
        ros::Publisher current_pose_pub_;
        ros::Publisher marker_pub;
        ros::Subscriber current_joint_sub_;
        ros::Subscriber joy_sub_;

        bool execution_completion_flag_;

        moveit::planning_interface::MoveGroupInterface::Plan plan;

        // ros::ServiceServer PlannerService = private_nh_.advertiseService("Get pose plan:", this->getPosePlan);

        geometry_msgs::Point currentPose_;
        geometry_msgs::Point previousPose_;

        scara_planner::TrajectoryJoints currentJoint_;
        scara_planner::TrajectoryJoints prevJoint_;

        geometry_msgs::Point prevGoal_;
        geometry_msgs::Point currentGoal_;

        bool planning_flag = true;

        visualization_msgs::Marker marker;

        void loadParam();

        void joints_callback(const scara_planner::TrajectoryJoints &msg);
        void joyCb(const sensor_msgs::Joy::ConstPtr &msg);
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
        // moveit::planning_interface::MoveItErrorCode ScaraPlanner::executePlan(const moveit::planning_interface::MoveGroupInterface::Plan &plan)

        // joint goal
        void getJointTarget(const robot_state::JointModelGroup *joint_model_group);
        const moveit::planning_interface::MoveGroupInterface::Plan setJointTarget(const robot_state::JointModelGroup *joint_model_group);
        void moveJointTarget();

        void getCurrentJoint();
        geometry_msgs::Pose getCurrentPose();

        void getRobotState();

        void getInverseKinematics();
        void getForwardKinematics();
    };

} // scara_ns
#endif
