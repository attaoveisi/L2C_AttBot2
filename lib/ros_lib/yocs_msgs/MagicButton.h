#ifndef _ROS_yocs_msgs_MagicButton_h
#define _ROS_yocs_msgs_MagicButton_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace yocs_msgs
{

  class MagicButton : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef bool _pressed_type;
      _pressed_type pressed;

    MagicButton():
      header(),
      pressed(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_pressed;
      u_pressed.real = this->pressed;
      *(outbuffer + offset + 0) = (u_pressed.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->pressed);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_pressed;
      u_pressed.base = 0;
      u_pressed.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->pressed = u_pressed.real;
      offset += sizeof(this->pressed);
     return offset;
    }

    virtual const char * getType() override { return "yocs_msgs/MagicButton"; };
    virtual const char * getMD5() override { return "bfc1e2424321b1d3dcc226b473f78588"; };

  };

}
#endif
