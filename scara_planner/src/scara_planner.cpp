#include "scara_planner.hpp"

namespace scara_ns
{

    ScaraPlanner::ScaraPlanner()
    {
        ROS_INFO("scara_node initiated!");
        traj_list_pub_ = relative_nh_.advertise<scara_planner::TrajectoryJoints>("trajectory_joints", 10);
        traj_goal_pub_ = relative_nh_.advertise<geometry_msgs::Point>("trajectory_goal", 10);
        current_pose_pub_ = relative_nh_.advertise<geometry_msgs::Point>("current_pose", 10);
        // current_joint_sub_ = relative_nh_.subscribe<scara_planner::TrajectoryJoints>("current_joints", 20, & ScaraPlanner::joints_callback, this);

        loadParam();

    }

    // ScaraPlanner::ScaraPlanner()
    // {
    //     ROS_INFO("scara node default initiated!");
    // }

    ScaraPlanner::~ScaraPlanner()
    {
        ;
    }

    void ScaraPlanner::loadParam()
    {
        // if (!ros::param::param<int>("/ks114_sonar/num_of_sonar", num_of_sonar_, DEFAULT_NUM))
        // {
        //     ROS_WARN("Number of sonar is not set. Using %u as default", DEFAULT_NUM);
        // }
    }

    void ScaraPlanner::startProcess()
    {
        ros::Rate r(LOOP_RATE);

        // this->startMoveGroupInterface();

        while (relative_nh_.ok())
        {
            ros::spinOnce();
        }
    }

    // void ScaraPlanner::joints_callback(const scara_planner::TrajectoryJoints &msg)
    // {
    // }

    void startMoveGroupInterface()
    {
        static const std::string PLANNING_GROUP = "whole_arm";
        moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        const robot_state::JointModelGroup *joint_model_group =
            move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

        ROS_INFO_NAMED("Scara", "Planning frame: %s", move_group.getPlanningFrame().c_str());
        ROS_INFO_NAMED("scara", "Available Planning Groups:");

        std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
                  std::ostream_iterator<std::string>(std::cout, ", "));

        std::vector<double> current_joint = move_group.getCurrentJointValues();
        ROS_INFO("Current joint 1: %f", current_joint[1]);
        ROS_INFO("Current joint 2: %f", current_joint[2]);
        ROS_INFO("Current joint 3: %f", current_joint[3]);
        ROS_INFO("Current joint 4: %f", current_joint[4]);

        geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();
        ROS_INFO("Current x: %f", current_pose.pose.position.x);
        ROS_INFO("Current y: %f", current_pose.pose.position.y);
        ROS_INFO("Current z: %f", current_pose.pose.position.z);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        ROS_INFO_NAMED("scara", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

        ROS_INFO_NAMED("scara", "Visualizing plan 1 as trajectory line");

    }

    void getCurrentJoint()
    {
        // std::vector<double> current_joint = move_group.getCurrentJointValues();
        // ROS_INFO("Current joint 1: %f", current_joint[1]);
        // ROS_INFO("Current joint 2: %f", current_joint[2]);
        // ROS_INFO("Current joint 3: %f", current_joint[3]);
        // ROS_INFO("Current joint 4: %f", current_joint[4]);
    }

    void getCurrentPose()
    {
        // geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();
        // ROS_INFO("Current x: %f", current_pose.pose.position.x);
        // ROS_INFO("Current y: %f", current_pose.pose.position.y);
        // ROS_INFO("Current z: %f", current_pose.pose.position.z);
    }

    std::vector<float> getTraj(std::vector<float> &A, std::vector<float> &B)
    {
        // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        // bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        // ROS_INFO_NAMED("scara", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

        // ROS_INFO_NAMED("scara", "Visualizing plan 1 as trajectory line");
    }

}; // scara_ns
