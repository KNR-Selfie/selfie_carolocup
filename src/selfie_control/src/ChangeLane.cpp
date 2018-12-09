#include "ChangeLane.h"

ChangeLane::ChangeLane(const ros::NodeHandle& nh, const ros::NodeHandle& pnh):
    nh_(nh),
    pnh_(pnh),
    target_position(0.f)
{
}

ChangeLane::~ChangeLane()
{
}

bool ChangeLane::init(void){
    //subscriber creation
    position_offset_subscriber = nh_.subscribe("position_offset", 10, &ChangeLane::position_offset_callback, this);
    //publishers creation
    target_publisher = nh_.advertise<std_msgs::Float64>("target_offset", 100);
    left_turn_indicator_publisher = nh_.advertise<std_msgs::Bool>("left_turn_indicator", 100);
    right_turn_indicator_publisher = nh_.advertise<std_msgs::Bool>("right_turn_indicator", 100);

    //geting paramets 
    pnh_.getParam("lane_width",lane_width);
    pnh_.getParam("error_margin",error_margin);
    //evaluation
    ROS_INFO("Width %f error %f", lane_width, error_margin);

    return true;
}

//callback from subscriber with information about position goal
void ChangeLane::position_offset_callback(const std_msgs::Float64::ConstPtr& msg)
{
    position_offset = msg->data;
}

//sending target postion to publisher
void ChangeLane::send_target_position(void){
    target_msg.data = target_position;
    target_publisher.publish(target_msg);
}

//sending indicator msgs to topics
void ChangeLane::send_indicator_msgs(void){
    left_turn_indicator_msg.data = left_turn_indicator;
    right_turn_indicator_msg.data = right_turn_indicator;
    left_turn_indicator_publisher.publish(left_turn_indicator_msg);
    right_turn_indicator_publisher.publish(right_turn_indicator_msg);
}

//send all msgs to publishers
void ChangeLane::send_publishers_msgs(void){
    send_target_position();
    send_indicator_msgs();
}

//main fuction for computing value for target position
//input lane_width, error_margin, position_offset
//output taget_positions, left_turn_indicator, right_turn_indicator
//left +, right -
void ChangeLane::compute_target_position(void){
}