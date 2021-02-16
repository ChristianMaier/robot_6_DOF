#ifndef _ROS_SERVICE_pos_service_h
#define _ROS_SERVICE_pos_service_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

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
      uint32_t joint_pos_length;
      typedef float _joint_pos_type;
      _joint_pos_type st_joint_pos;
      _joint_pos_type * joint_pos;

    pos_serviceResponse():
      joint_pos_length(0), st_joint_pos(), joint_pos(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->joint_pos_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->joint_pos_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->joint_pos_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->joint_pos_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_pos_length);
      for( uint32_t i = 0; i < joint_pos_length; i++){
      union {
        float real;
        uint32_t base;
      } u_joint_posi;
      u_joint_posi.real = this->joint_pos[i];
      *(outbuffer + offset + 0) = (u_joint_posi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_joint_posi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_joint_posi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_joint_posi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_pos[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t joint_pos_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      joint_pos_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      joint_pos_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      joint_pos_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->joint_pos_length);
      if(joint_pos_lengthT > joint_pos_length)
        this->joint_pos = (float*)realloc(this->joint_pos, joint_pos_lengthT * sizeof(float));
      joint_pos_length = joint_pos_lengthT;
      for( uint32_t i = 0; i < joint_pos_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_joint_pos;
      u_st_joint_pos.base = 0;
      u_st_joint_pos.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_joint_pos.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_joint_pos.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_joint_pos.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_joint_pos = u_st_joint_pos.real;
      offset += sizeof(this->st_joint_pos);
        memcpy( &(this->joint_pos[i]), &(this->st_joint_pos), sizeof(float));
      }
     return offset;
    }

    virtual const char * getType() override { return POS_SERVICE; };
    virtual const char * getMD5() override { return "b7a5b192aa496cd0bd74aa738e2d0265"; };

  };

  class pos_service {
    public:
    typedef pos_serviceRequest Request;
    typedef pos_serviceResponse Response;
  };

}
#endif
