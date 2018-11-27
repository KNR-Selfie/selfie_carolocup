#include "lanedetector.hpp"

static int Acc_slider = 1;
static int Acc_value = 1;
static int Acc_filt = 15;
static int Acc_filt_slider = 40;

cv::Mat tmp_mat(IDS_HEIGHT, IDS_WIDTH, CV_8UC3);

LaneDetector::LaneDetector()
{
}

void LaneDetector::detectLine_both(cv::Mat &input_white, std::vector<std::vector<cv::Point> > &output_white)
{
    output_white.clear();
    cv::findContours(input_white, output_white, hierarchy_detectline, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    for (std::vector<std::vector<cv::Point> >::iterator filt =output_white.begin();filt !=output_white.end();){
        if (filt->size()<Acc_filt)
            filt = output_white.erase(filt);
        else
            ++filt;
    }
    for (int i = 0; i < output_white.size(); i++)
    {
        cv::approxPolyDP(cv::Mat(output_white[i]),output_white[i], Acc_value, false);
    }
}

void LaneDetector::drawPoints_both(std::vector<std::vector<cv::Point> > &input_white, cv::Mat &output_white)
{
	double sum = 0, avg = 0;
	slope = 0;
	count = 0;

    output_white = cv::Mat::zeros(output_white.size(), CV_8UC3);
    for (int i = 0; i < input_white.size(); i++)
    {
        for (unsigned int j = 0; j < input_white[i].size(); j++)
        {
			cv::circle(output_white, input_white[i][j], 3, cv::Scalar(0, 255, 255), CV_FILLED, cv::LINE_AA);

			if (j > 0)
			{
				if ((input_white[i][j].x - input_white[i][j - 1].x) != 0)
					sum = sum + ((input_white[i][j].y - input_white[i][j - 1].y) / (input_white[i][j].x - input_white[i][j - 1].x));
				count++;
			}

        }
    }

	if(count != 0)
		avg = sum / count;

	slope = avg;
}

void LaneDetector::AvgSlope(std::vector<std::vector<cv::Point> > &input_white, cv::Mat &output)
{
	int x1 = output.cols / 2;
	int y1 = output.rows;
	double b = y1 - slope * x1;
	int y2 = output.rows - 50;
	int x2 = (y2 - b) / slope;
	cv::Point center1(x1, y1);
	cv::Point center2(x2, y2);

	cv::line(output, center1, center2, cv::Scalar(100, 100, 255), 4);
	Dev_Lines(input_white, Left_points, Right_points, Middle_points, center1, center2);
	Draw_Points(output, Left_points, Right_points, Middle_points);
}

 void LaneDetector::Dev_Lines(std::vector<std::vector<cv::Point> > &input_white, std::vector<cv::Point> &left_points, std::vector<cv::Point> &right_points, std::vector<cv::Point> &middle_points, cv::Point A, cv::Point B)
 {
	 left_points.clear();
	 right_points.clear();
	 middle_points.clear();

	 int sum = 0;
	 std::vector<cv::Point> tmp_points;

	 for (int i = 0; i < input_white.size(); i++)
	 {
		 for (unsigned int j = 0; j < input_white[i].size(); j++)
		 {
			 if (Alfa_Val(A, B, input_white[i][j]) > 0)
				 tmp_points.push_back(input_white[i][j]);
			 else if (Alfa_Val(A, B, input_white[i][j]) < 0)
				 right_points.push_back(input_white[i][j]);				
		 }
	 }

	 for (int i = 0; i < tmp_points.size(); i++)
	 {
		 if (tmp_points[i].x > sum / tmp_points.size())
			 middle_points.push_back(tmp_points[i]);
		 else if (tmp_points[i].x < sum / tmp_points.size())
			 left_points.push_back(tmp_points[i]);
	 }
 }

 double LaneDetector::Alfa_Val(cv::Point A, cv::Point B, cv::Point C)
 {
	 double alfa = (B.x - A.x) * (C.x - A.y) - (B.y - A.y) * (C.x - A.x);

	 return alfa;
 }

 void LaneDetector::Draw_Points(cv::Mat &frame, std::vector<cv::Point> left_points, std::vector<cv::Point> right_points, std::vector<cv::Point> middle_points)
 {
	 for (int i = 0; i < left_points.size(); i++)
	 {
		 cv::circle(frame, left_points[i], 3, cv::Scalar(0, 255, 0), CV_FILLED, cv::LINE_AA);
	 }
	 for (int i = 0; i < right_points.size(); i++)
	 {
		 cv::circle(frame, right_points[i], 3, cv::Scalar(255, 0, 0), CV_FILLED, cv::LINE_AA);
	 }
	 for (int i = 0; i < middle_points.size(); i++)
	 {
		 cv::circle(frame, middle_points[i], 3, cv::Scalar(0, 0, 255), CV_FILLED, cv::LINE_AA);
	 }
 }
