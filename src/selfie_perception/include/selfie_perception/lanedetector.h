#pragma once

#include <ros/ros.h>
#include <iostream>
#include <string>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define IDS_WIDTH 752
#define IDS_HEIGHT 360

#define PI 3.1415926

class LaneDetector
{    
public:
    LaneDetector(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
	~LaneDetector();
	bool init();

private:
	ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
	image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Publisher lines_pub_;

	cv::Mat kernel_v_;
	cv::Mat cam2world_;
	cv::Mat current_frame_;
	cv::Mat gray_frame_;
	cv::Mat binary_frame_;
	cv::Mat mask_;
	cv::Mat canny_frame_;
	cv::Mat visualization_frame_;

	std::vector<std::vector<cv::Point> > points_vector_;

	void imageCallback(const sensor_msgs::ImageConstPtr& msg);
	void openCVVisualization();

	void detectLine_both(cv::Mat &input_white, std::vector<std::vector<cv::Point> > &output_white);
    void drawPoints_both(std::vector<std::vector<cv::Point> > &input_white, cv::Mat &output_white);
	void Dev_Lines(std::vector<std::vector<cv::Point> > &input_white, std::vector<cv::Point> &left_points, std::vector<cv::Point> &right_points, std::vector<cv::Point> &middle_points, cv::Point A, cv::Point B);
	double Alfa_Val(cv::Point A, cv::Point B, cv::Point C);
	void Draw_Points(cv::Mat &frame, std::vector<cv::Point> left_points, std::vector<cv::Point> right_points, std::vector<cv::Point> middle_points);
	void AvgSlope(std::vector<std::vector<cv::Point> > &input_white, cv::Mat &output);

	int binary_treshold_;
	bool mask_initialized_;
	bool visualize_;


	std::vector<cv::Point> Left_points;
	std::vector<cv::Point> Right_points;
	std::vector<cv::Point> Middle_points;

	double slope;
	double act_slope;
	double last_slope;
	int count;

    std::vector<cv::Vec4i> hierarchy_detectline;
};
