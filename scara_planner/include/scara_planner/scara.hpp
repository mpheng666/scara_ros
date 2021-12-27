#ifndef SCARA_HPP
#define SCARA_HPP

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

#include "std_msgs/Float32.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"

namespace scara_ns
{
    class Scara
    {
    public:
        Scara(const std::string, const std::string, const std::string);
        ~Scara();
        void startProcess();

    private:
        ros::NodeHandle *prive_nh_;
        ros::NodeHandle *relative_nh_;
        ros::Publisher scara_odom_pub;
        ros::Subscriber scara_cmd_sub;
        ros::Subscriber scara_initpose_sub;

        enum ModeState
        {
            CartesainMode,
            JointMode,
            TeachMode,
            RandomMode
        };

        ModeState _currState = CartesainMode;

        const double LOOP_RATE = 10;

        // double last_t_;
        // double freq_, publish_freq_;

        string base_frame_id;
        string odom_frame_id;
        string global_frame_id;

        tf::TransformBroadcaster broadcaster;

        // boost::recursive_mutex state_lock_;
        // boost::recursive_mutex vw_lock_;
        // string name;

        const std_msgs::Float32 DEFAULT_LINK_1;
        const std_msgs::Float32 DEFAULT_LINK_2;
        const std_msgs::Float32 DEFAULT_LINK_3;
        double x, y, th, v, w;
        double x_gt, y_gt, th_gt;
        double 1rot_lim_, 2rot_lim_, 3pris_lim_;
        double 1rot_, 2rot_, 3pris_;

        void loadParam();
        void emergencyStop();
        void pauseMove();
        void kinematicCal();
        void modeChange();
        void lineSegmentation();
        void moveScara();
        void zeroScara();
    }

}; //scara_ns

#endif
