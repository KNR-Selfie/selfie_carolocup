#include "ChangeLane.h"

//function used in Action Server for change lane
void execute(const selfie_control::ChangeLaneGoalConstPtr& goal, Server* as, ChangeLane *changelane);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "change_lane");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    //Constructor for changelane class
    ChangeLane changelane(nh, pnh);

    //parameters set
    pnh.setParam("lane_width",10);
    pnh.setParam("error_margin",2);
    changelane.init();

    changelane.target_position = 2.f;
    
    //Action server creation
    Server server(nh, "change_lane", boost::bind(&execute, _1, &server,&changelane), false);
    server.start();
    ros::spin();

    return 0;

}

void execute(const selfie_control::ChangeLaneGoalConstPtr& goal, Server* as, ChangeLane* changelane)
{
    //compute output values
    //TODO: implement inside
    changelane->compute_target_position();
    
    //sending msgs to topics
    changelane->send_publishers_msgs();

    //confirm success
    as->setSucceeded();
}