#pragma once
#ifndef PACKAGE_PATH_FILE_H
#define PACKAGE_PATH_FILE_H

#include <ros/ros.h>
#include <iostream>
#include <vector>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

struct Point
{
    float x;
    float y;
};

struct Line
{
    Point start_point;
    Point end_point;
    float slope;
};

struct Obstacle
{
    std::vector <Line> obstacle;
    std::vector <Point> apex;
};

class ObstaclesGenerator
{
public:
    ObstaclesGenerator(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
    ~ObstaclesGenerator();
    bool init();
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber scan_sub_;
    ros::Publisher obstacles_pub_;
    ros::Publisher visualization_lines_pub_;

    std::vector <Line> line_array_;
    std::vector <Obstacle> obstacle_array_;
    sensor_msgs::LaserScan scan_;
    void laserScanCallback(const sensor_msgs::LaserScan& msg);
    void generateLines();
    Point getXY(float &angle, float &range);
    float getSlope(Point &p1, Point &p2);
    void visualizeLines();

    float max_distance_;
    float min_distance_;
    float line_max_range_difference_;
    float line_max_slope_difference_;
    float line_min_slope_difference_;
    float line_slope_difference_ratio_;
    int line_min_points_;
    bool visualize_;
    
};

    
#endif
