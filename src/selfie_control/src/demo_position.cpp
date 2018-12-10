#include "ros/ros.h"
#include "std_msgs/Float64.h"

float target;

void positionCallback(const std_msgs::Float64::ConstPtr& msg);

int main(int argc, char **argv)
{

  ros::init(argc, argv, "demo_position");

  ros::NodeHandle n;
  ros::Publisher offset_publisher = n.advertise<std_msgs::Float64>("position_offset", 100);
  ros::Subscriber target_subscriber = n.subscribe("target_offset", 1, positionCallback);
  std_msgs::Float64 offset_msg;
  offset_msg.data = 0.f;
  while (ros::ok())
  {
    if(target!=0){
        offset_msg.data +=target/100;
    }
    ros::Duration(0.1).sleep();
    ROS_INFO("Jade %f",offset_msg.data);
    //publishing msg
    offset_publisher.publish(offset_msg);
    ros::spinOnce();
  }
}

void positionCallback(const std_msgs::Float64::ConstPtr& msg)
{
  target = msg->data;
}
