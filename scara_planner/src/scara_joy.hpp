#ifndef SCARA_JOY_H
#define SCARA_JOY_H

namespace ros { class NodeHandle; }

namespace scara_joy
{

class ScaraJoy
{
public:
  ScaraJoy(ros::NodeHandle* nh, ros::NodeHandle* nh_param);

private:
  struct Impl;
  Impl* pimpl_;
};

}  // namespace scara_joy

#endif  
