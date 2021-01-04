#ifndef _ROS_kobuki_msgs_DigitalInputEvent_h
#define _ROS_kobuki_msgs_DigitalInputEvent_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace kobuki_msgs
{

  class DigitalInputEvent : public ros::Msg
  {
    public:
      bool values[4];

    DigitalInputEvent():
      values()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 4; i++){
      union {
        bool real;
        uint8_t base;
      } u_valuesi;
      u_valuesi.real = this->values[i];
      *(outbuffer + offset + 0) = (u_valuesi.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->values[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 4; i++){
      union {
        bool real;
        uint8_t base;
      } u_valuesi;
      u_valuesi.base = 0;
      u_valuesi.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->values[i] = u_valuesi.real;
      offset += sizeof(this->values[i]);
      }
     return offset;
    }

    virtual const char * getType() override { return "kobuki_msgs/DigitalInputEvent"; };
    virtual const char * getMD5() override { return "93da823c8b121f8a3940ef3830c58e44"; };

  };

}
#endif
