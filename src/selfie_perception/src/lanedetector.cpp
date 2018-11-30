#include <selfie_perception/lanedetector.h>

#define PI 3.1415926

static int Acc_slider = 1;
static int Acc_value = 1;
static int Acc_filt = 5;
static int Acc_filt_slider = 40;
static int Alpha_ = 12;
static int F_ = 500, Dist_ = 500;

cv::Mat tmp_mat(IDS_HEIGHT, IDS_WIDTH, CV_8UC3);

LaneDetector::LaneDetector(const ros::NodeHandle &nh, const ros::NodeHandle &pnh) : 
	nh_(nh),
	pnh_(pnh),
	it_(nh),
	binary_treshold_(180),
	mask_initialized_(false),
	visualize_(true),
	max_mid_line_gap_(90),
	left_points_index_(-1),
	right_points_index_(-1),
	middle_points_index_(-1),

	kernel_v_(),
	current_frame_(),
	gray_frame_(),
	binary_frame_(),
	mask_(),
	canny_frame_(),
	visualization_frame_(),
	homography_frame_()
{
	lines_pub_ =  nh_.advertise<selfie_msgs::RoadMarkings>("road_markings", 10);
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

	cv::namedWindow("Homography", 1);
	cv::createTrackbar("Alpha", "Homography", &Alpha_, 180);

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
				cv::Point(60, current_frame_.rows),
				cv::Point(current_frame_.cols - 60, current_frame_.rows),
				cv::Point(current_frame_.cols - 60, current_frame_.rows / 3),
				cv::Point(60, current_frame_.rows / 3)};
		cv::fillConvexPoly(mask_, points, 4, cv::Scalar(255, 0, 0));
		mask_initialized_ = true;
	}
	Homography(current_frame_, homography_frame_);
	cv::cvtColor(homography_frame_, gray_frame_, cv::COLOR_BGR2GRAY);
	cv::threshold(gray_frame_, binary_frame_, binary_treshold_, 255, cv::THRESH_BINARY);
	cv::bitwise_and(binary_frame_, mask_, canny_frame_);
	cv::medianBlur(canny_frame_, canny_frame_, 5);
	cv::filter2D(canny_frame_, canny_frame_, -1, kernel_v_, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);

	detectLine_both(canny_frame_, points_vector_);
	mergeMiddleLane();
	recognizeLines();

	if (visualize_)
	{
		visualization_frame_.rows = current_frame_.rows;
		visualization_frame_.cols = current_frame_.cols;
		Draw_Points(visualization_frame_);
		openCVVisualization();
	}
	publish_markings();
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
				cv::line(output_white, cv::Point(input_white[i][j - 1]), cv::Point(input_white[i][j]), cv::Scalar(0, 0, 255), 2);
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
	//Dev_Lines(input_white, Left_points, Right_points, Middle_points, center1, center2);
	//Draw_Points(output, Left_points, Right_points, Middle_points);
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

void LaneDetector::Draw_Points(cv::Mat &frame)
{
	frame = cv::Mat::zeros(frame.size(), CV_8UC3);
	if(right_points_index_ >= 0)
		for (int i = 0; i < points_vector_[right_points_index_].size(); i++)
		{
			cv::circle(frame, points_vector_[right_points_index_][i], 3, cv::Scalar(255, 0, 0), CV_FILLED, cv::LINE_AA);
		}
	if(left_points_index_ >= 0)
		for (int i = 0; i < points_vector_[left_points_index_].size(); i++)
		{
			cv::circle(frame, points_vector_[left_points_index_][i], 3, cv::Scalar(0, 0, 255), CV_FILLED, cv::LINE_AA);
		}
	if(middle_points_index_ >= 0)
		for (int i = 0; i < points_vector_[middle_points_index_].size(); i++)
		{
			cv::circle(frame, points_vector_[middle_points_index_][i], 3, cv::Scalar(0, 255, 0), CV_FILLED, cv::LINE_AA);
		}
}

void LaneDetector::Homography(cv::Mat input_frame, cv::Mat &homography_frame)
{
	resize(input_frame, input_frame, cv::Size(640, 480));

	double focal_lenth, dist, alpha;

	alpha = ((double)Alpha_ - 90) * PI / 180;
	focal_lenth = (double)F_;
	dist = (double)Dist_;

	cv::Size image_size = input_frame.size();
	double w = (double)image_size.width, h = (double)image_size.height;

	// Matrix 2D -> 3D
	cv::Mat A1 = (cv::Mat_<float>(4, 3) <<
		1, 0, -w / 2,
		0, 1, -h / 2,
		0, 0, 0,
		0, 0, 1);
	
	// Rotation matrix
	cv::Mat RX = (cv::Mat_<float>(4, 4) <<
		1, 0, 0, 0,
		0, cos(alpha), -sin(alpha), 0,
		0, sin(alpha), cos(alpha), 0,
		0, 0, 0, 1);
	
	cv::Mat R = RX;

	// Translation matrix
	cv::Mat T = (cv::Mat_<float>(4, 4) <<
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, dist,
		0, 0, 0, 1);

	// Intrinsic matrix
	cv::Mat K = (cv::Mat_<float>(3, 4) <<
		focal_lenth, 0, w / 2, 0,
		0, focal_lenth, h / 2, 0,
		0, 0, 1, 0
		);

	cv::Mat transformationMat = K * (T * (R * A1));

	cv::warpPerspective(input_frame, homography_frame, transformationMat, image_size, cv::INTER_CUBIC | cv::WARP_INVERSE_MAP);
}

void LaneDetector::openCVVisualization()
{
	//cv::imshow("Raw image", current_frame_);
	//cv::imshow("Binarization", binary_frame_);
	//cv::imshow("Canny", canny_frame_);
	cv::imshow("Homography", homography_frame_);
	cv::imshow("Output", visualization_frame_);
	cv::waitKey(1);
}

void LaneDetector::quickSortLinesY(int left, int right)
{
	int i = left;
	int j = right;
	int y = points_vector_[(left + right) / 2][0].y;
	do
	{
		while (points_vector_[i][0].y > y)
			i++;

		while (points_vector_[j][0].y < y)
			j--;

		if (i <= j)
		{
			points_vector_[i].swap(points_vector_[j]);

			i++;
			j--;
		}
	} while (i <= j);

	if (left < j)
		LaneDetector::quickSortLinesY(left, j);

	if (right > i)
		LaneDetector::quickSortLinesY(i, right);
}

void LaneDetector::quickSortPointsY(std::vector<cv::Point> &vector_in, int left, int right)
{
	int i = left;
	int j = right;
	int y = vector_in[(left + right) / 2].y;
	do
	{
		while (vector_in[i].y > y)
			i++;

		while (vector_in[j].y < y)
			j--;

		if (i <= j)
		{
			cv::Point temp = vector_in[i];
			vector_in[i] = vector_in[j];
			vector_in[j] = temp;

			i++;
			j--;
		}
	} while (i <= j);

	if (left < j)
		LaneDetector::quickSortPointsY(vector_in, left, j);

	if (right > i)
		LaneDetector::quickSortPointsY(vector_in, i, right);
}

void LaneDetector::mergeMiddleLane()
{
	for (int i = 0; i < points_vector_.size(); i++)
	{
		quickSortPointsY(points_vector_[i], 0, points_vector_[i].size() - 1);
	}
	quickSortLinesY(0, points_vector_.size() - 1);

	for (int i = 0; i < points_vector_.size(); i++)
	{
		for (int j = i + 1; j < points_vector_.size(); j++)
		{
			float distance = getDistance(points_vector_[j][0], points_vector_[i][points_vector_[i].size() - 1]);

			if (distance < max_mid_line_gap_)
			{
				points_vector_[i].insert(points_vector_[i].end(), points_vector_[j].begin(), points_vector_[j].end());
				points_vector_.erase(points_vector_.begin() + j);
				j--;
			}
		}
	}
}

float LaneDetector::getDistance(cv::Point p1, cv::Point p2)
{
	float dx = float(p1.x - p2.x);
	float dy = float(p1.y - p2.y);
	return sqrtf(dx * dx + dy * dy);
}

void LaneDetector::recognizeLines()
{
	switch (points_vector_.size())
	{
	case 1:
		right_points_index_ = 0;
		middle_points_index_ = -1;
		left_points_index_ = -1;
		break;
	case 2:
		if (points_vector_[0][0].x > points_vector_[1][0].x)
		{
			right_points_index_ = 0;
			middle_points_index_ = 1;
			left_points_index_ = -1;
		}
		else
		{
			right_points_index_ = 1;
			middle_points_index_= 0;
			left_points_index_ = -1;
		}
		break;
	case 3:
		if (points_vector_[1][0].x > points_vector_[0][0].x)
		{
			if (points_vector_[0][0].x > points_vector_[2][0].x)
			{
				right_points_index_ = 1;
				middle_points_index_ = 0;
				left_points_index_ = 2;
			}
			else if (points_vector_[2][0].x > points_vector_[1][0].x)
			{
				right_points_index_ = 2;
				middle_points_index_ = 1;
				left_points_index_ = 0;
			}
			else
			{
				right_points_index_ = 1;
				middle_points_index_ = 2;
				left_points_index_ = 0;
			}
		}
		else if (points_vector_[1][0].x > points_vector_[2][0].x)
		{
			right_points_index_ = 0;
			middle_points_index_ = 1;
			left_points_index_ = 2;
		}
		else if (points_vector_[0][0].x > points_vector_[2][0].x)
		{
			right_points_index_ = 0;
			middle_points_index_ = 2;
			left_points_index_ = 1;
		}
		else
		{
			right_points_index_ = 2;
			middle_points_index_ = 0;
			left_points_index_ = 1;
		}
		break;
	default:
		right_points_index_ = -1;
		middle_points_index_ = -1;
		left_points_index_ = -1;
		break;
	}
}

void LaneDetector::publish_markings()
{
	selfie_msgs::RoadMarkings road_markings;
	road_markings.header.stamp = ros::Time::now();
	road_markings.header.frame_id = "road_markings";
	geometry_msgs::Point p;
	p.z = 0;
	if(right_points_index_ >= 0)
		for (int i = 0; i < points_vector_[right_points_index_].size(); i++)
		{
			p.x = points_vector_[right_points_index_][i].x;
			p.y = points_vector_[right_points_index_][i].y;
			road_markings.right_line.push_back(p);
		}
	if(left_points_index_ >= 0)
		for (int i = 0; i < points_vector_[left_points_index_].size(); i++)
		{
			p.x = points_vector_[left_points_index_][i].x;
			p.y = points_vector_[left_points_index_][i].y;
			road_markings.left_line.push_back(p);
		}
	if(middle_points_index_ >= 0)
		for (int i = 0; i < points_vector_[middle_points_index_].size(); i++)
		{
			p.x = points_vector_[middle_points_index_][i].x;
			p.y = points_vector_[middle_points_index_][i].y;
			road_markings.center_line.push_back(p);
		}
	lines_pub_.publish(road_markings);
}