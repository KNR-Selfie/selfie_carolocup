#pragma once

#include <iostream>
#include <string>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define IDS_WIDTH 752
#define IDS_HEIGHT 360

#define PI 3.1415926

class LaneDetector
{    
public:
    LaneDetector();

    void detectLine_both(cv::Mat &input_white, std::vector<std::vector<cv::Point> > &output_white);
    void drawPoints_both(std::vector<std::vector<cv::Point> > &input_white, cv::Mat &output_white);
	void Dev_Lines(std::vector<std::vector<cv::Point> > &input_white, std::vector<cv::Point> &left_points, std::vector<cv::Point> &right_points, std::vector<cv::Point> &middle_points, cv::Point A, cv::Point B);
	double Alfa_Val(cv::Point A, cv::Point B, cv::Point C);
	void Draw_Points(cv::Mat &frame, std::vector<cv::Point> left_points, std::vector<cv::Point> right_points, std::vector<cv::Point> middle_points);
	void AvgSlope(std::vector<std::vector<cv::Point> > &input_white, cv::Mat &output);

	std::vector<cv::Point> Left_points;
	std::vector<cv::Point> Right_points;
	std::vector<cv::Point> Middle_points;

private:
	double slope;
	double act_slope;
	double last_slope;
	int count;

    std::vector<cv::Vec4i> hierarchy_detectline;
};
