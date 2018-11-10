#include <selfie_perception/obstacles_generator.h>

ObstaclesGenerator::ObstaclesGenerator(const ros::NodeHandle& nh, const ros::NodeHandle& pnh):
    nh_(nh),
    pnh_(pnh),
    max_distance_(2.0),
    line_max_range_difference_(0.01),
    line_max_slope_difference_(0.2),
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
    pnh_.getParam("line_max_range_difference",line_max_range_difference_);
    pnh_.getParam("line_max_slope_difference",line_max_slope_difference_);
    pnh_.getParam("line_min_points",line_min_points_);
    pnh_.getParam("visualize",visualize_);

    if(visualize_)
    {
        visualizeLines_pub_ = nh_.advertise<visualization_msgs::Marker>( "visualization_lines", 1 );
    }
    return true;
}

void ObstaclesGenerator::laserScanCallback(const sensor_msgs::LaserScan& msg)
{
    scan_ = msg;
    generateLines();
    ROS_INFO("Lines found: %d", line_array_.size());
    visualizeLines();
}

void ObstaclesGenerator::generateLines()
{
    line_array_.clear();
    Point start_point = getXY(scan_.angle_min, scan_.ranges[0]);
    Point last_fitted_point;
    int start_point_index = 0;
    int points_in_line = 1;
    float act_line_slope_sum = 0;
    float avg_act_line_slope;
    int act_angle_index = 1;
    bool reset_params = false;
    Point act_point;

    for(float act_angle = scan_.angle_min + scan_.angle_increment; act_angle < scan_.angle_max; act_angle += scan_.angle_increment)
    {
        if(scan_.ranges[act_angle_index] <= max_distance_)
        {
            if(abs(scan_.ranges[act_angle_index] - scan_.ranges[act_angle_index-1]) <= line_max_range_difference_)
            {
                act_point = getXY(act_angle, scan_.ranges[act_angle_index]);
                if(points_in_line < 2)
                {
                    act_line_slope_sum += getSlope(start_point, act_point);
                    avg_act_line_slope = act_line_slope_sum;
                    last_fitted_point = act_point;
                    points_in_line ++;
                }
                else 
                {
                    if(abs(avg_act_line_slope - getSlope(start_point, act_point)) <= line_max_slope_difference_)
                    {
                        act_line_slope_sum += getSlope(start_point, act_point);
                        avg_act_line_slope = act_line_slope_sum / points_in_line;
                        last_fitted_point = act_point;
                        points_in_line ++;
                    }
                    else
                    {
                        if(points_in_line >= line_min_points_)
                        {
                            Line l;
                            l.start_point = start_point;
                            l.end_point = last_fitted_point;
                            l.slope = getSlope(start_point, last_fitted_point);
                            line_array_.push_back(l);
                        }
                        reset_params = true;
                    }
                }
            }
            else
                reset_params = true;
        }
        else
            reset_params = true;
        if(reset_params)
        {
            start_point = act_point;
            start_point_index = act_angle_index;
            points_in_line = 1;
            act_line_slope_sum = 0;
            reset_params = false;
        }
        act_angle_index ++;
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
    if((p2.x - p1.x) == 0)
        return 999;
    else
        return (p2.y - p1.y) / (p2.x - p1.x);
}

void ObstaclesGenerator::visualizeLines()
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = "markers";
    marker.header.stamp = ros::Time::now();
    marker.ns = "line";
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = 0;

    geometry_msgs::Point marker_point;
    marker_point.z = 0;

    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    marker.lifetime = ros::Duration();

    for(int i = 0; i < line_array_.size(); i++)
    {
        marker_point.x = line_array_[i].start_point.x;
        marker_point.y = line_array_[i].start_point.y;
        marker.points.push_back(marker_point);

        marker_point.x = line_array_[i].end_point.x;
        marker_point.y = line_array_[i].end_point.y;
        marker.points.push_back(marker_point);
    }
    visualizeLines_pub_.publish(marker);
}