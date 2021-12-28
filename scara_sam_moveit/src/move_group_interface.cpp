#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <scara_sam_demo.hpp>

geometry_msgs::Pose home_pose;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_interface");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  joint_list_pub = nh.advertise<std_msgs::Float64MultiArray>("joint_msg", 1);
  scara_sub = nh.subscribe<scara_planner::scara>("scara_cmd", 1000, &scaraCB);
  cartesian_control_sub = nh.subscribe<std_msgs::Float64MultiArray>("cartesian_msg", 1000, &cartesianCB);
  joint_control_sub = nh.subscribe<std_msgs::Float64MultiArray>("joint_sub_msg", 1000, &jointCB);
  spinner.start();

  static const std::string PLANNING_GROUP = "whole_arm";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const robot_state::JointModelGroup *joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // namespace rvt = rviz_visual_tools;

  ROS_INFO_NAMED("Scara", "Planning frame: %s", move_group.getPlanningFrame().c_str());
  ROS_INFO_NAMED("scara", "Available Planning Groups:");

  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

  while (nh.ok())
  {

    home_pose.orientation.w = 1.0;
    // home_pose.position.x += x_;
    home_pose.position.x = 0.330071;
    ROS_INFO("X: %f", home_pose.position.x);
    // home_pose.position.y += y_;
    home_pose.position.y = 0.320370;
    ROS_INFO("Y: %f", home_pose.position.y);
    home_pose.position.z = 0.200974;
    ROS_INFO("Z: %f", home_pose.position.z);
    move_group.setPoseTarget(home_pose);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("scara", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    ROS_INFO_NAMED("scara", "Visualizing plan 1 as trajectory line");

    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();
    
    ROS_INFO("Current x: %f", current_pose.pose.position.x);
    ROS_INFO("Current y: %f", current_pose.pose.position.y);
    ROS_INFO("Current z: %f", current_pose.pose.position.z);

  }

  ros::shutdown();
  return 0;
}