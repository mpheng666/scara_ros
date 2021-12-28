#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "scara_joy.hpp"
#include "scara_planner/scara.h"

#include <map>
#include <string>


namespace scara_joy
{
struct ScaraJoy::Impl
{
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void sendCmdMsg(const sensor_msgs::Joy::ConstPtr& joy_msg, const std::string& which_map);

  ros::Subscriber joy_sub;
  ros::Publisher cmd_pub;

  int reset_button; // x
  int record_button; // y
  int playback_button; // B
  int enable_button; // A 
  double z_button;

  std::map<std::string, int> axis_x_cartesian_map;
  std::map< std::string, std::map<std::string, double> > scale_x_cartesian_map;
  std::map<std::string, int> axis_y_cartesian_map;
  std::map< std::string, std::map<std::string, double> > scale_y_cartesian_map;

  std::map<std::string, int> axis_1_joint_map;
  std::map< std::string, std::map<std::string, double> > scale_1_joint_map;
  std::map<std::string, int> axis_2_joint_map;
  std::map< std::string, std::map<std::string, double> > scale_2_joint_map;

  bool sent_disable_msg;
};

/**
 * Constructs ScaraJoy.
 * \param nh NodeHandle to use for setting up the publisher and subscriber.
 * \param nh_param NodeHandle to use for searching for configuration parameters.
 */
ScaraJoy::ScaraJoy(ros::NodeHandle* nh, ros::NodeHandle* nh_param)
{
  pimpl_ = new Impl;

  pimpl_->cmd_pub = nh->advertise<scara_planner::scara>("scara_cmd", 1, true);
  pimpl_->joy_sub = nh->subscribe<sensor_msgs::Joy>("joy", 1, &ScaraJoy::Impl::joyCallback, pimpl_);

  nh_param->param<double>("z_button", pimpl_->z_button, 7);
  nh_param->param<int>("reset_button", pimpl_->reset_button, 2);
  nh_param->param<int>("record_button", pimpl_->record_button, 3);
  nh_param->param<int>("playback_button", pimpl_->playback_button, 1);
  nh_param->param<int>("enable_button", pimpl_->enable_button, 0);
  nh_param->param<int>("axis_x_cartesian", pimpl_->axis_x_cartesian_map["x"], 0);
  nh_param->param<double>("scale_x_cartesian", pimpl_->scale_x_cartesian_map["normal"]["x"], 1.0);
  nh_param->param<int>("axis_y_cartesian", pimpl_->axis_y_cartesian_map["y"], 1);
  nh_param->param<double>("scale_y_cartesian", pimpl_->scale_y_cartesian_map["normal"]["y"], 1.0);
  nh_param->param<int>("axis_1_joint", pimpl_->axis_1_joint_map["joint_1"], 3);
  nh_param->param<double>("scale_1_joint", pimpl_->scale_1_joint_map["normal"]["joint_1"], 0.5);
  nh_param->param<int>("axis_2_joint", pimpl_->axis_2_joint_map["joint_2"], 4);
  nh_param->param<double>("scale_2_joint", pimpl_->scale_2_joint_map["normal"]["joint_2"], 0.5);

  ROS_INFO_NAMED("ScaraJoy", "Teleop reset button %i.", pimpl_->reset_button);
  ROS_INFO_NAMED("ScaraJoy", "Teleop record button %i.", pimpl_->record_button);
  ROS_INFO_NAMED("ScaraJoy", "Teleop playback button %i.", pimpl_->playback_button);
  ROS_INFO_NAMED("ScaraJoy", "Teleop enable button %i.", pimpl_->enable_button);
  ROS_INFO_NAMED("ScaraJoy", "Teleop z button %f.", pimpl_->z_button);

  for (std::map<std::string, int>::iterator it = pimpl_->axis_x_cartesian_map.begin();
      it != pimpl_->axis_x_cartesian_map.end(); ++it)
  {
    ROS_INFO_NAMED("ScaraJoy", "Cartesian axis %s on %i at scale %f.",
    it->first.c_str(), it->second, pimpl_->scale_x_cartesian_map["normal"][it->first]);
  }

  for (std::map<std::string, int>::iterator it = pimpl_->axis_y_cartesian_map.begin();
      it != pimpl_->axis_y_cartesian_map.end(); ++it)
  {
    ROS_INFO_NAMED("ScaraJoy", "Cartesian axis %s on %i at scale %f.",
    it->first.c_str(), it->second, pimpl_->scale_y_cartesian_map["normal"][it->first]);
  }

  for (std::map<std::string, int>::iterator it = pimpl_->axis_1_joint_map.begin();
      it != pimpl_->axis_1_joint_map.end(); ++it)
  {
    ROS_INFO_NAMED("ScaraJoy", "Joint axis %s on %i at scale %f.",
    it->first.c_str(), it->second, pimpl_->scale_1_joint_map["normal"][it->first]);
  }

  for (std::map<std::string, int>::iterator it = pimpl_->axis_2_joint_map.begin();
      it != pimpl_->axis_2_joint_map.end(); ++it)
  {
    ROS_INFO_NAMED("ScaraJoy", "Joint axis %s on %i at scale %f.",
    it->first.c_str(), it->second, pimpl_->scale_2_joint_map["normal"][it->first]);
  }

  pimpl_->sent_disable_msg = false;
}

double getVal(const sensor_msgs::Joy::ConstPtr& joy_msg, const std::map<std::string, int>& axis_map,
          const std::map<std::string, double>& scale_map, const std::string& fieldname)
{
  if (axis_map.find(fieldname) == axis_map.end() ||
      scale_map.find(fieldname) == scale_map.end() ||
      joy_msg->axes.size() <= axis_map.at(fieldname))
  {
    return 0.0;
  }

  return joy_msg->axes[axis_map.at(fieldname)] * scale_map.at(fieldname);
}

void ScaraJoy::Impl::sendCmdMsg(const sensor_msgs::Joy::ConstPtr& joy_msg,
                                         const std::string& which_map)
{
  // Initializes with zeros by default.
  scara_planner::scara cmd_msg;

  cmd_msg.x = getVal(joy_msg, axis_x_cartesian_map, scale_x_cartesian_map[which_map], "x");
  cmd_msg.y = getVal(joy_msg, axis_y_cartesian_map, scale_y_cartesian_map[which_map], "y");
  cmd_msg.joint_1 = getVal(joy_msg, axis_1_joint_map, scale_1_joint_map[which_map], "joint_1");
  cmd_msg.joint_2 = getVal(joy_msg, axis_2_joint_map, scale_2_joint_map[which_map], "joint_2");
  cmd_msg.reset = joy_msg->buttons[reset_button];
  cmd_msg.record = joy_msg->buttons[record_button];
  cmd_msg.playback = joy_msg->buttons[playback_button];
  cmd_msg.enable = joy_msg->buttons[enable_button];
  cmd_msg.z = joy_msg->axes[z_button];

  cmd_pub.publish(cmd_msg);
  sent_disable_msg = false;
}

void ScaraJoy::Impl::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  if (joy_msg->buttons.size() > enable_button &&
           joy_msg->buttons[enable_button])
  {
    sendCmdMsg(joy_msg, "normal");
  }
  else
  {
    // When enable button is released, immediately send a single no-motion command
    // in order to stop the robot.
    if (!sent_disable_msg)
    {
      // Initializes with zeros by default.
      scara_planner::scara cmd_msg;
      cmd_pub.publish(cmd_msg);
      sent_disable_msg = true;
    }
  }
}

}  // namespace scarajoy
