#include <selfie_perception/lanedetector.h>

static int Acc_slider = 1;
static int Acc_value = 1;
static int Acc_filt = 15;
static int Acc_filt_slider = 40;

cv::Mat tmp_mat(IDS_HEIGHT, IDS_WIDTH, CV_8UC3);

LaneDetector::LaneDetector(const ros::NodeHandle &nh, const ros::NodeHandle &pnh) : 
	nh_(nh),
	pnh_(pnh),
	it_(nh),
	binary_treshold_(180),
	mask_initialized_(false),
	visualize_(true),

	kernel_v_(),
	current_frame_(),
	gray_frame_(),
	binary_frame_(),
	mask_(),
	canny_frame_(),
	visualization_frame_()
{
	//lines_pub_ =
}

LaneDetector::~LaneDetector()
{
}

bool LaneDetector::init()
{
	kernel_v_ = cv::Mat(1, 3, CV_32F);
	kernel_v_.at<float>(0, 0) = -1;
	kernel_v_.at<float>(0, 1) = 0;
	kernel_v_.at<float>(0, 2) = 1;

	image_sub_ = it_.subscribe("/image_raw", 1, &LaneDetector::imageCallback, this);

	return true;
}

void LaneDetector::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
	try
	{
		current_frame_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
	}
	catch (cv_bridge::Exception &e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	if (!mask_initialized_)
	{
		mask_ = cv::Mat::zeros(cv::Size(current_frame_.cols, current_frame_.rows), CV_8UC1);
		cv::Point points[4] =
			{
				cv::Point(0, current_frame_.rows),
				cv::Point(current_frame_.cols, current_frame_.rows),
				cv::Point(current_frame_.cols - 60, current_frame_.rows / 3),
				cv::Point(60, current_frame_.rows / 3)};
		cv::fillConvexPoly(mask_, points, 4, cv::Scalar(255, 0, 0));
		mask_initialized_ = true;
	}

	cv::cvtColor(current_frame_, gray_frame_, cv::COLOR_BGR2GRAY);
	cv::threshold(gray_frame_, binary_frame_, binary_treshold_, 255, cv::THRESH_BINARY);
	cv::bitwise_and(binary_frame_, mask_, canny_frame_);
	cv::medianBlur(canny_frame_, canny_frame_, 5);
	cv::filter2D(canny_frame_, canny_frame_, -1, kernel_v_, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);

	points_vector_.clear();
	visualization_frame_.rows = current_frame_.rows;
	visualization_frame_.cols = current_frame_.cols;
	detectLine_both(canny_frame_, points_vector_);
	drawPoints_both(points_vector_, visualization_frame_);
	AvgSlope(points_vector_, visualization_frame_);

	if (visualize_)
	{
		openCVVisualization();
	}
}

void LaneDetector::detectLine_both(cv::Mat &input_white, std::vector<std::vector<cv::Point> > &output_white)
{
	output_white.clear();
	cv::findContours(input_white, output_white, hierarchy_detectline, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

	for (std::vector<std::vector<cv::Point> >::iterator filt = output_white.begin(); filt != output_white.end();)
	{
		if (filt->size() < Acc_filt)
			filt = output_white.erase(filt);
		else
			++filt;
	}
	for (int i = 0; i < output_white.size(); i++)
	{
		cv::approxPolyDP(cv::Mat(output_white[i]), output_white[i], Acc_value, false);
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

	if (count != 0)
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

void LaneDetector::openCVVisualization()
{
	cv::imshow("Raw image", current_frame_);
	cv::imshow("Grayscale", gray_frame_);
	cv::imshow("Canny", canny_frame_);
	cv::imshow("Output", visualization_frame_);
	cv::waitKey(1);
}
