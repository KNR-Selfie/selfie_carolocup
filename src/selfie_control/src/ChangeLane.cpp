#include "ChangeLane.h"

ChangeLane::ChangeLane(const ros::NodeHandle& nh, const ros::NodeHandle& pnh):
    nh_(nh),
    pnh_(pnh),
    target_position(0.f),
    visualize_(true)
{
}


ChangeLane::~ChangeLane()
{
}

bool ChangeLane::init(void){

    position_offset_subscriber = nh_.subscribe("position_offset", 10, &ChangeLane::position_offset_callback, this);
 
    target_publisher = nh_.advertise<std_msgs::Float64>("target_offset", 100);

    left_turn_indicator_publisher = nh_.advertise<std_msgs::Bool>("left_turn_indicator", 100);

    right_turn_indicator_publisher = nh_.advertise<std_msgs::Bool>("right_turn_indicator", 100);


    pnh_.getParam("lane_width",lane_width);
    pnh_.getParam("error_margin",error_margin);
    ROS_INFO("Width %f error %f", lane_width, error_margin);

    return true;
}


void ChangeLane::position_offset_callback(const std_msgs::Float64::ConstPtr& msg)
{
    position_offset = msg->data;
}

void ChangeLane::send_target_position(void){
    

    target_msg.data = target_position;
    target_publisher.publish(target_msg);
}

void ChangeLane::execute(const selfie_control::ChangeLaneGoalConstPtr& goal, Server* as)
{
    send_target_position();
    as->setSucceeded();
}
