#include "change_lane_logic.h"


void ChangeLaneLogic::check_lane_status(void){
    if (line_coef.right.size())
        ROS_INFO("%f %f %f %d", line_coef.left[0], line_coef.center[0], line_coef.right[0], line_coef.right.size());
}

