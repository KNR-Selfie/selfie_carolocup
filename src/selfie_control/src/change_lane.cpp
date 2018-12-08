#include "ChangeLane.h"

void execute(const selfie_control::ChangeLaneGoalConstPtr& goal, Server* as, ChangeLane *changelane);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "change_lane");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    ChangeLane changelane(nh, pnh);
    pnh.setParam("lane_width",10);
    pnh.setParam("error_margin",2);
    changelane.init();
    changelane.target_position = 2.f;
    Server server(nh, "change_lane", boost::bind(&execute, _1, &server,&changelane), false);
    server.start();
    ros::spin();

    
    return 0;

}

void execute(const selfie_control::ChangeLaneGoalConstPtr& goal, Server* as, ChangeLane* changelane)
{
    changelane->send_target_position();
    as->setSucceeded();
}