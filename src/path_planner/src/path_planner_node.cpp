#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "selfie_msgs/RoadMarkings.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"
#include "nav_msgs/Path.h"

#define PREVIEW_MODE 1

ros::Publisher cloud_pub;
ros::Publisher path_pub;

void road_markingsCallback(const selfie_msgs::RoadMarkings::ConstPtr& msg)
{
    ROS_INFO("position: x: %f y: %f", msg->center_line[0].x, msg->center_line[0].y);

#if PREVIEW_MODE
    sensor_msgs::PointCloud points_preview;
    geometry_msgs::Point32 point;
    points_preview.header.frame_id = "my_frame";

    for(int i = 0;i<msg->center_line.size();i++)
    {
        point.x = msg->center_line[i].x;
        point.y = msg->center_line[i].y;
        point.z = msg->center_line[i].z;
        points_preview.points.push_back(point);
    }

    cloud_pub.publish(points_preview);
#endif
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_planner_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("road_markings", 1000, road_markingsCallback);
#if PREVIEW_MODE
    cloud_pub = nh.advertise<sensor_msgs::PointCloud>("cloud", 1000);
    path_pub  = nh.advertise<nav_msgs::Path>("path",1000);
#endif
    ros::Publisher position_offset_pub = nh.advertise<std_msgs::Float64>("position_offset",1000);
    ros::Publisher heading_offset_pub = nh.advertise<std_msgs::Float64>("heading_offset",1000);

    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
