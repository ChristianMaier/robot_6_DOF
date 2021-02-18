#ifndef _ROS_SERVICE_pos_service_h
#define _ROS_SERVICE_pos_service_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Float32MultiArray.h"

namespace robot_hardware_interface
{

static const char POS_SERVICE[] = "robot_hardware_interface/pos_service";

  class pos_serviceRequest : public ros::Msg
  {
    public:

    pos_serviceRequest()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
     return offset;
    }

    virtual const char * getType() override { return POS_SERVICE; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class pos_serviceResponse : public ros::Msg
  {
    public:
      typedef std_msgs::Float32MultiArray _joint_pos_type;
      _joint_pos_type joint_pos;

    pos_serviceResponse():
      joint_pos()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->joint_pos.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->joint_pos.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return POS_SERVICE; };
    virtual const char * getMD5() override { return "c605bcd1060e2ab3fdb6258d8cd3effc"; };

  };

  class pos_service {
    public:
    typedef pos_serviceRequest Request;
    typedef pos_serviceResponse Response;
  };

}
#endif
