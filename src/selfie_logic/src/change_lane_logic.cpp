#include "change_lane_logic.h"


void ChangeLaneLogic::check_line_direction(void){
    ROS_INFO("%f %f %f", line_coef.left[x], line_coef.center[x], line_coef.right[x]);
}