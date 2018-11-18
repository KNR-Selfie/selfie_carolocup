#include "ros/ros.h"
#include "std_msgs/String.h"
#include "selfie_msgs/RoadMarkings.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;

    ros::Publisher chatter_pub = nh.advertise<selfie_msgs::RoadMarkings>("road_markings", 1000);

    ros::Rate loop_rate(1);
    geometry_msgs::Point point;
    point.x = 3;
    point.y = 2;
    point.z = 0;
    selfie_msgs::RoadMarkings msg;

    for(int i =0;i<7;i++)
    {
        point.y+=0.2;
        msg.center_line.push_back(point);
    }
    msg.left_line.push_back(point);
    msg.right_line.push_back(point);

    while (ros::ok())
    {
        chatter_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
