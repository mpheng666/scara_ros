#ifndef _SCARA_KINEMATICS_SCARA_KINEMATICS_HPP_
#define _SCARA_KINEMATICS_SCARA_KINEMATICS_HPP_

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Empty.h>

// #include "scara_kinematics/ScaraFk.h"
// #include "scara_kinematics/ScaraIk.h"


namespace scara_kinematics_ns
{
    class ScaraKinematics
    {
        private:
            ros::NodeHandle private_nh_;
            static constexpr double LOOP_RATE {20.0};
            const double DEFAULT_LINK_1_LENGTH {0.2};
            const double DEFAULT_LINK_2_LENGTH {0.2};
            double link_1_length_ {DEFAULT_LINK_1_LENGTH};
            double link_2_length_ {DEFAULT_LINK_2_LENGTH};

            void loadParams();
            ros::ServiceServer ik_service_;
            ros::ServiceServer fk_service_;

        public:
            ScaraKinematics(ros::NodeHandle *nh);
            ~ScaraKinematics();
            void start();
            bool forwardKinematics(std::vector<double> joints, geometry_msgs::PoseStamped &target_pose);
            bool inverseKinematics(geometry_msgs::PoseStamped &pose, std::vector<double> &target_joints);
            bool fkServiceCb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
            bool ikServiceCb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

    }; // ScaraKinematics
    

}; // scara_kinematics_ns

#endif