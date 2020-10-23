/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle  nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

std_msgs::String str_msg2;
ros::Publisher chatter2("chatter2", &str_msg2);

char hello[13] = "hello world!";
char hello2[13] = "hello Atta!!";

void setup()
{
  nh.initNode();
  nh.advertise(chatter);
  nh.advertise(chatter2);
}

void loop()
{
  str_msg.data = hello;
  str_msg2.data = hello2;
  Serial.print(hello2);
  Serial.print("\n");
  chatter2.publish( &str_msg2 );
  nh.spinOnce();
  //delay(1000);
}
