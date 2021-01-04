#ifndef _ROS_yocs_msgs_WallList_h
#define _ROS_yocs_msgs_WallList_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "yocs_msgs/Wall.h"

namespace yocs_msgs
{

  class WallList : public ros::Msg
  {
    public:
      uint32_t obstacles_length;
      typedef yocs_msgs::Wall _obstacles_type;
      _obstacles_type st_obstacles;
      _obstacles_type * obstacles;

    WallList():
      obstacles_length(0), st_obstacles(), obstacles(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->obstacles_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->obstacles_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->obstacles_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->obstacles_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->obstacles_length);
      for( uint32_t i = 0; i < obstacles_length; i++){
      offset += this->obstacles[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t obstacles_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      obstacles_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      obstacles_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      obstacles_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->obstacles_length);
      if(obstacles_lengthT > obstacles_length)
        this->obstacles = (yocs_msgs::Wall*)realloc(this->obstacles, obstacles_lengthT * sizeof(yocs_msgs::Wall));
      obstacles_length = obstacles_lengthT;
      for( uint32_t i = 0; i < obstacles_length; i++){
      offset += this->st_obstacles.deserialize(inbuffer + offset);
        memcpy( &(this->obstacles[i]), &(this->st_obstacles), sizeof(yocs_msgs::Wall));
      }
     return offset;
    }

    virtual const char * getType() override { return "yocs_msgs/WallList"; };
    virtual const char * getMD5() override { return "2879979aabb372fc7f6d782228e53412"; };

  };

}
#endif
