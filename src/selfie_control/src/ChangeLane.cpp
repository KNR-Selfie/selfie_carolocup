#include "ChangeLane.h"

ChangeLane::ChangeLane(float lane_width_param, float error_margin_param):
    lane_width(lane_width_param),
    error_margin(error_margin_param),
    target_position(lane_width_param/2)
{
}

ChangeLane::~ChangeLane()
{
}

//main fuction for computing value for target position
//input lane_width, error_margin, position_offset
//output taget_positions, left_turn_indicator, right_turn_indicator
//left +, right -
bool ChangeLane::process_target_position(void){
    if (be_on_left_lane == true){
        if (position_offset>lane_width/2-error_margin && position_offset < lane_width/2 + error_margin){
            ROS_INFO("Juz jestem na lewym pasie");
            maneuver_done = true;
            target_position = 0;
            left_turn_indicator = false;
            right_turn_indicator = false;
        }
        else{
            ROS_INFO("Jade na lewy pas. Pos_off: %f, Lane_width: %f, Margin: %f ", position_offset, lane_width, error_margin);
            maneuver_done = false;
            target_position = lane_width/2;
            left_turn_indicator = true;
            right_turn_indicator = false; 
        }
    }
    else if (be_on_left_lane == false){
        if (position_offset>-lane_width/2-error_margin && position_offset < -lane_width/2 + error_margin){
            ROS_INFO("Juz jestem na prawym");
            maneuver_done = true;
            target_position = 0;
            left_turn_indicator = false;
            right_turn_indicator = false;
        }
        else{
            ROS_INFO("Jade na prawy pas. Pos_off: %f, Lane_width: %f, Margin: %f ", position_offset, lane_width, error_margin);
            maneuver_done = false;
            target_position = -lane_width/2;
            left_turn_indicator = true;
            right_turn_indicator = false; 
        }
    }
    return maneuver_done;

}