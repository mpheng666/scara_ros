#ifndef _SCARA_KINEMATICS_SCARA_KINEMATICS_HPP_
#define _SCARA_KINEMATICS_SCARA_KINEMATICS_HPP_

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Empty.h>

#include "scara_kinematics/ScaraFk.h"
#include "scara_kinematics/ScaraIk.h"


namespace scara_kinematics_ns
{
    class ScaraKinematics
    {
        private:
            ros::NodeHandle private_nh_;
            static constexpr double LOOP_RATE {20.0};
            static constexpr double DEFAULT_LINK_0_HEIGHT {0.1};
            static constexpr double DEFAULT_LINK_1_LENGTH {0.2};
            static constexpr double DEFAULT_LINK_2_LENGTH {0.2};
            double link_0_height_ {DEFAULT_LINK_0_HEIGHT};
            double link_1_length_ {DEFAULT_LINK_1_LENGTH};
            double link_2_length_ {DEFAULT_LINK_2_LENGTH};

            void loadParams();
            ros::ServiceServer ik_service_;
            ros::ServiceServer fk_service_;

        public:
            ScaraKinematics(ros::NodeHandle *nh);
            ~ScaraKinematics();
            void start();
            bool forwardKinematics(const std::vector<double>& joints, geometry_msgs::PoseStamped& target_pose);
            bool inverseKinematics(const geometry_msgs::PoseStamped& pose, std::vector<double>& target_joints);
            bool fkServiceCb(scara_kinematics::ScaraFk::Request& request, scara_kinematics::ScaraFk::Response& response);
            bool ikServiceCb(scara_kinematics::ScaraIk::Request& request, scara_kinematics::ScaraIk::Response& response);

    }; // ScaraKinematics
    

}; // scara_kinematics_ns

#endif