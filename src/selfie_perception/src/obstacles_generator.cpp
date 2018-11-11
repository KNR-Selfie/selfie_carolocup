#include <selfie_perception/obstacles_generator.h>
#include <math.h>

ObstaclesGenerator::ObstaclesGenerator(const ros::NodeHandle& nh, const ros::NodeHandle& pnh):
    nh_(nh),
    pnh_(pnh),
    max_distance_(1.5),
    min_distance_(0.03),
    line_max_range_difference_(0.04),
    line_max_slope_difference_(0.8),
    line_min_slope_difference_(0.12),
    line_slope_difference_ratio_(20),
    line_min_points_(10),
    visualize_(true)
{
    //obstacles_pub_ = nh_.advertise<std_msgs::Float32[8]>("obstacles", 10);
}

ObstaclesGenerator::~ObstaclesGenerator()
{
    obstacle_array_.clear();
}

bool ObstaclesGenerator::init()
{
    scan_sub_ = nh_.subscribe("/scan", 10, &ObstaclesGenerator::laserScanCallback, this);
    pnh_.getParam("max_distance",max_distance_);
    pnh_.getParam("min_distance",min_distance_);
    pnh_.getParam("line_max_range_difference",line_max_range_difference_);
    pnh_.getParam("line_max_slope_difference",line_max_slope_difference_);
    pnh_.getParam("line_min_slope_difference",line_min_slope_difference_);
    pnh_.getParam("line_slope_difference_ratio",line_slope_difference_ratio_);
    pnh_.getParam("line_min_points",line_min_points_);
    pnh_.getParam("visualize",visualize_);

    if(visualize_)
    {
        visualization_lines_pub_ = nh_.advertise<visualization_msgs::Marker>( "visualization_lines", 1 );
    }
    return true;
}

void ObstaclesGenerator::laserScanCallback(const sensor_msgs::LaserScan& msg)
{
    scan_ = msg;
    generateLines();
    ROS_INFO("Lines found: %d", line_array_.size());
    if(visualize_)
        visualizeLines();
}

void ObstaclesGenerator::generateLines()
{
    line_array_.clear();
    Point start_point = getXY(scan_.angle_min, scan_.ranges[0]);
    Point last_fitted_point;
    int points_in_line = 1;
    float act_line_slope_sum = 0;
    float avg_act_line_slope;
    int act_angle_index = 1;
    int start_point_index = 0;
    bool reset_params = false;
    Point act_point;
    float line_slope_difference = line_max_slope_difference_;

    for(float act_angle = scan_.angle_min + scan_.angle_increment; act_angle <= scan_.angle_max; act_angle += scan_.angle_increment)
    {
        act_point = getXY(act_angle, scan_.ranges[act_angle_index]);
        if(scan_.ranges[act_angle_index] <= max_distance_ && scan_.ranges[act_angle_index] >= min_distance_)
        {
            if(std::abs(scan_.ranges[act_angle_index] - scan_.ranges[act_angle_index-1]) <= line_max_range_difference_)
            {
                if(points_in_line < 2)
                {
                    act_line_slope_sum += getSlope(start_point, act_point);
                    avg_act_line_slope = act_line_slope_sum;
                    last_fitted_point = act_point;
                }
                else 
                {
                    if(line_slope_difference > line_min_slope_difference_)
                        line_slope_difference = (line_min_slope_difference_ - line_max_slope_difference_) / line_slope_difference_ratio_ * points_in_line + line_max_slope_difference_;
                    if(std::abs(avg_act_line_slope - getSlope(start_point, act_point)) <= line_slope_difference)
                    {
                        act_line_slope_sum += getSlope(start_point, act_point);
                        avg_act_line_slope = act_line_slope_sum / points_in_line;
                        last_fitted_point = act_point;
                    }
                    else
                        reset_params = true;
                }
            }
            else
                reset_params = true;
        }
        else
            reset_params = true;
        if(reset_params || act_angle == scan_.angle_max)
        {
            if (points_in_line >= line_min_points_)
            {
                Line l;
                l.start_point = start_point;
                l.end_point = last_fitted_point;
                l.slope = getSlope(start_point, last_fitted_point);
                line_array_.push_back(l);
            }
            do
            {
                act_angle_index ++;
                act_angle += scan_.angle_increment;
            }while(std::isnan(scan_.ranges[act_angle_index]) && act_angle < scan_.angle_max);
            start_point = getXY(act_angle, scan_.ranges[act_angle_index]);
            start_point_index = act_angle_index;
            points_in_line = 0;
            act_line_slope_sum = 0;
            line_slope_difference = line_max_slope_difference_;
            reset_params = false;
        }
        act_angle_index ++;
        points_in_line ++;
    }
}

Point ObstaclesGenerator::getXY(float &angle, float &range)
{
    Point p;
    p.x = range * cos(angle);
    p.y = range * sin(angle);
    return p;
}

float ObstaclesGenerator::getSlope(Point &p1, Point &p2)
{
        return atan((p2.x - p1.x) / (p2.y - p1.y));
}

void ObstaclesGenerator::visualizeLines()
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = "laser";
    marker.header.stamp = ros::Time::now();
    marker.ns = "line";
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = 0;
    marker.lifetime = ros::Duration();

    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    marker.scale.x = 0.01;
    marker.scale.y = 0.01;

    geometry_msgs::Point marker_point;
    marker_point.z = 0;

    for(int i = 0; i < line_array_.size(); i++)
    {
        marker_point.x = line_array_[i].start_point.x;
        marker_point.y = line_array_[i].start_point.y;
        marker.points.push_back(marker_point);

        marker_point.x = line_array_[i].end_point.x;
        marker_point.y = line_array_[i].end_point.y;
        marker.points.push_back(marker_point);
    }
    visualization_lines_pub_.publish(marker);
}