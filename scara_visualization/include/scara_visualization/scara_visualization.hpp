#ifndef _SCARA_VISUALIZATION_SCARA_VISUALIZATION_HPP_
#define _SCARA_VISUALIZATION_SCARA_VISUALIZATION_HPP_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <vector>

namespace scara_visualization_ns
{
    class ScaraVisualization
    {
        private:
            static constexpr double LOOP_RATE {20.0};

        public:
            ScaraVisualization(ros::NodeHandle *nh);
            ~ScaraVisualization();
            ros::Publisher marker_pub_;

            visualization_msgs::Marker createMarker(std::string frame_id, std::string ns, int id, int type, geometry_msgs::Pose& pose, double scale, std_msgs::ColorRGBA& colour);
            void publishMarker(const visualization_msgs::Marker& marker);
            void publishMarker(const std::vector<visualization_msgs::Marker>& markers);
            void updateMarker(visualization_msgs::Marker& marker, geometry_msgs::Pose pose);

    };

}; // scara_visualization_ns

#endif //_SCARA_VISUALIZATION_SCARA_VISUALIZATION_HPP_