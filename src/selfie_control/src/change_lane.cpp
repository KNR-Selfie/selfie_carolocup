#include "ChangeLane.h"

bool maneuver_done = false;

//function used in Action Server for change lane
void execute(const selfie_control::ChangeLaneGoalConstPtr& goal, Server* as, ChangeLane *changelane);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "change_lane");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    //geting paramets 
    float lane_width=1, error_margin = 0.1;
    pnh.getParam("lane_width",lane_width);
    pnh.getParam("error_margin",error_margin);
    //evaluation
    ROS_INFO("Width %f error %f", lane_width, error_margin);

    //Constructor for changelane class
    ChangeLane changelane(nh, pnh, lane_width, error_margin);

    changelane.init();
    
    
    //Action server creation
    Server server(nh, "change_lane", boost::bind(&execute, _1, &server,&changelane), false);
    server.start();
    bool maneuver_status = false;
    while (ros::ok())
    {
        if(maneuver_done ==false){
            //process output values
            maneuver_status = changelane.process_target_position();
            if (maneuver_status == true){
                maneuver_done = true;
                ROS_INFO("Skonczylem zmiane pasa");
            }
            //sending msgs to topics
            changelane.send_publishers_msgs();
        }
        ros::spinOnce();
    }

    return 0;

}

void execute(const selfie_control::ChangeLaneGoalConstPtr& goal, Server* as, ChangeLane* changelane)
{
    changelane->be_on_left_lane = goal->left_lane;
    
    bool maneuver_status = changelane->process_target_position();
    if (maneuver_status==true){
        ROS_INFO("Juz na nim jestes");
    }
    else{
        maneuver_done = false;
        ROS_INFO("Zmieniam pas");
    }
    //confirm success
    as->setSucceeded();
}