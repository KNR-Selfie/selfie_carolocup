#pragma once
#ifndef CHANGELANE_H
#define CHANGELANE_H

#include <ros/ros.h>
#include <iostream>
#include <string>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <selfie_control/ChangeLaneAction.h>  
#include <actionlib/server/action_server.h>
#include <boost/bind.hpp>

#define PI 3.1415926

typedef actionlib::ActionServer<selfie_control::ChangeLaneAction> Server;

class ChangeLane
{
  public:
	ChangeLane(float lane_width_param, float error_margin_param);
	~ChangeLane();
    void send_publishers_msgs(void);
    bool process_target_position(void);
    float target_position;
    bool be_on_left_lane;
    bool maneuver_done;
    bool left_turn_indicator;
    bool right_turn_indicator;
    float error_margin;
    float lane_width;
    float position_offset;
  private:	
    
   
    int change_lane_status;

};


#endif