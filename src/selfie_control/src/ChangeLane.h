#pragma once
#ifndef CHANGELANE_H
#define CHANGELANE_H

#include <ros/ros.h>
#include <iostream>
#include <string>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <selfie_control/ChangeLaneAction.h>  
#include <actionlib/server/simple_action_server.h>

#define PI 3.1415926

typedef actionlib::SimpleActionServer<selfie_control::ChangeLaneAction> Server;

class ChangeLane
{
  public:
	ChangeLane(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);
	~ChangeLane();
	bool init(void);
    void send_target_position(void);
    void execute(const selfie_control::ChangeLaneGoalConstPtr& goal, Server* as);
    float target_position;

  private:
	ros::NodeHandle nh_;
	ros::NodeHandle pnh_;
	
    //publishers
	ros::Publisher target_publisher;
    ros::Publisher left_turn_indicator_publisher;
    ros::Publisher right_turn_indicator_publisher;
    std_msgs::Float64 target_msg;
    std_msgs::Bool left_turn_indicator_msg;
    std_msgs::Bool right_turn_indicator_msg;
    //subscriber
    ros::Subscriber position_offset_subscriber;
    void position_offset_callback(const std_msgs::Float64::ConstPtr& msg);


    float error_margin;
    float lane_width;
    float position_offset;

    float visualize_;

};


#endif