#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "selfie_msgs/RoadMarkings.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Path.h"
#include <path_planner/polyfit.hpp>
#include <path_planner/path_planner.h>




#define PREVIEW_MODE 1

#if PREVIEW_MODE
    ros::Publisher cloud_pub;
    ros::Publisher path_pub;
#endif

poly right_line;
poly left_line;
poly center_line;
poly middle_path;
tangent path_tangent;

void road_markingsCallback(const selfie_msgs::RoadMarkings::ConstPtr& msg)
{

    center_line.get_row_pts(msg->center_line);
    center_line.polyfit(7); //todo parametrize degree

    right_line.get_row_pts(msg->right_line);
    right_line.polyfit(7);

    center_line.polyval();
    right_line.polyval();

    path_tangent.calc(right_line,2);

    middle_path.find_middle(right_line,center_line);

#if PREVIEW_MODE
    sensor_msgs::PointCloud points_preview;
    nav_msgs::Path calc_path;

    RoadMarkings_to_cloud(msg,points_preview);
    poly_to_path(middle_path,calc_path);

    //publish to rviz
    points_preview.header.frame_id = "my_frame";
    cloud_pub.publish(points_preview);

    calc_path.header.frame_id = "my_frame";
    path_pub.publish(calc_path);
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
