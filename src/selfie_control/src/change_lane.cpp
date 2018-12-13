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

    changelane.init();
    
    //Action server creation
    Server server(nh, "change_lane", boost::bind(&execute, _1, &server,&changelane), false);
    server.start();
    changelane.maneuver_done = true;

    while (ros::ok())
    {
        if(changelane.maneuver_done ==false){
            //compute output values
            changelane.compute_target_position();
    
            //sending msgs to topics
            changelane.send_publishers_msgs();
        }
        ros::spinOnce();
    }

    return 0;

}

void execute(const selfie_control::ChangeLaneGoalConstPtr& goal, Server* as, ChangeLane* changelane)
{
    ROS_INFO("Chce zmienic pas");
    changelane->be_on_left_lane = goal->left_lane;
    changelane->maneuver_done = false;
    
    //confirm success
    as->setSucceeded();
}