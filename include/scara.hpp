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

#include <boost/thread.hpp>

#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"

namespace scara_ns
{
    class Scara

}