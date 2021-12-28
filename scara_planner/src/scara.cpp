#include <scara.hpp>

namespace scara_ns
{

    Scara::Scara(std::string PUB_TOPIC)
        : private_nh_("~"),
          scara_odom_pub_(relative_nh_.advertise<nav_msgs::Odometry>(PUB_TOPIC_ODOM, 10)),
          scara_cmd_vel_sub_(relative_nh._)subscribe<geomety_msgs::Twist>(SUB_TOPIC_CMDVEL, 20, &Scara::cmd_vel_callback, this))
    {
        loadParam();
        ROS_INFO("scara_node initiated!");
    }

    Scara::Scara()
    {
        ROS_INFO("scara node default initiated!");
    }

    Scara::~Scara()
    {
        ;
    }

    void Scara::loadParam()
    {
        // if (!ros::param::param<int>("/ks114_sonar/num_of_sonar", num_of_sonar_, DEFAULT_NUM))
        // {
        //     ROS_WARN("Number of sonar is not set. Using %u as default", DEFAULT_NUM);
        // }
        // if (!ros::param::param<std::string>("/ks114_sonar/serial_port", port_, DEFAULT_PORT))
        // {
        //     ROS_WARN("Sonar serial port is not set. Using %s as default", DEFAULT_PORT.c_str());
        // }
        // if (!ros::param::param<int>("/ks114_sonar/serial_baud_rate", baud_rate_, DEFAULT_BAUDRATE))
        // {
        //     ROS_WARN("Sonar baud rate is not set. Using %u as default", DEFAULT_BAUDRATE);
        // }
    }

    void Scara::startProcess()
    {
        ros::Rate r(LOOP_RATE);

        while (private_nh_.ok())
        {

            ros::spinOnce();
        }
    }

    void loadParam();
    void emergencyStop();
    void pauseMove();
    void kinematicCal();
    void modeChange();
    void lineSegmentation();
    void moveScara();

}; //scara_ns
