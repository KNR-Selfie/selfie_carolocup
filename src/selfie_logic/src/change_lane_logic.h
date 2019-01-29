#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include <selfie_msgs/RoadMarkings.h>
#include <selfie_msgs/PolygonArray.h>
#include <geometry_msgs/Point32.h>
#include <vector>

struct line_s{
    std::vector<float> center;
    std::vector<float> left;
    std::vector<float> right;
};

enum lane_status_e{
    l_left,
    l_right,
    l_straight,
};

enum car_lane_e{
    c_left,
    c_right,
};

class ChangeLaneLogic{
public:
    line_s line_coef;
    lane_status_e lane_status;
    car_lane_e car_lane;
    void check_line_direction(void);
};