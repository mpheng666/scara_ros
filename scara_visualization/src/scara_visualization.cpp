#include "scara_visualization/scara_visualization.hpp"

using namespace scara_visualization_ns;
    
ScaraVisualization::ScaraVisualization(ros::NodeHandle *nh):
marker_pub_(nh->advertise<visualization_msgs::Marker>("marker_pub", 10))
{
}

ScaraVisualization::~ScaraVisualization()
{
}

visualization_msgs::Marker ScaraVisualization::createMarker(std::string frame_id, std::string ns, int id, int type, geometry_msgs::Pose& pose, double scale, std_msgs::ColorRGBA& colour)
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();

    marker.ns = ns;
    marker.id = id;
    marker.type = type;

    marker.action = visualization_msgs::Marker::ADD;

    marker.pose = pose;
    if(marker.type == 0)
    {
        marker.scale.x = scale;
        marker.scale.y = 0.25;
        marker.scale.z = 0.25;
    }
    else
    {
        marker.scale.x = scale;
        marker.scale.y = scale;
        marker.scale.z = scale;
    }

    marker.color = colour;

    marker.lifetime = ros::Duration();

    return marker;
}

void ScaraVisualization::publishMarker(const visualization_msgs::Marker& marker)
{
    marker_pub_.publish(marker);
}

void ScaraVisualization::publishMarker(const std::vector<visualization_msgs::Marker>& markers)
{

    for(auto i=0; i<markers.size(); ++i)
    {
        marker_pub_.publish(markers.at(i));
    }
}

void ScaraVisualization::updateMarker(visualization_msgs::Marker& marker, geometry_msgs::Pose pose)
{
    marker.pose = pose;
    this->publishMarker(marker);
}