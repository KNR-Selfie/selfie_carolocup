
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <selfie_perception/lanedetector.h>

static const std::string OPENCV_WINDOW = "Image window";

LaneDetector laneDetector;

cv::Mat kernel_v;
cv::Point anchor = cv::Point(-1, -1);

class ImageConverter
{
public:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  std::vector<std::vector<cv::Point> > yellow_vector;
  std::vector<std::vector<cv::Point> > white_vector;
  std::vector<cv::Point> left_points;
  std::vector<cv::Point> right_points;
  std::vector<cv::Point> middle_points;

  cv::Mat Masked_frame;
  cv::Mat Wrapped_frame;
  cv::Mat Frame;
  cv::Mat Frame_canny;
  cv::Mat birdeye_frame, distCoeffs;
  cv::Mat HSV_frame;
  cv::Mat cone_frame_out;
  cv::Mat frame_out_yellow, frame_out_white, frame_out_edge_yellow, frame_out_edge_white;
  cv::Mat yellow_vector_frame;
  cv::Mat white_vector_frame;
  cv::Mat Matrix;

  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed

    cv::FileStorage fs("/home/maciej/catkin_ws/src/opencv_vision/src/homography_unrectified.yaml", cv::FileStorage::READ);
    fs["cam2world"] >> Matrix;

    std::cout << Matrix.cols << Matrix.rows << Matrix.type() << std::endl;
    std::cout << Matrix <<std::endl;

    image_sub_ = it_.subscribe("/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

	  kernel_v = cv::Mat(1, 3, CV_32F);
	  kernel_v.at<float>(0, 0) = -1;
	  kernel_v.at<float>(0, 1) = 0;
	  kernel_v.at<float>(0, 2) = 1;

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
	  cv_bridge::CvImagePtr cv_gray;
	  cv_bridge::CvImagePtr cv_binarize;

    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	    cv_gray = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

  cv::Mat Camera_matrix (cv_ptr->image.cols, cv_ptr->image.rows, CV_8UC3);

	white_vector_frame.rows = cv_ptr->image.rows;
	white_vector_frame.cols = cv_ptr->image.cols;

	Masked_frame.rows = cv_ptr->image.rows;
	Masked_frame.cols = cv_ptr->image.cols;

	cv::Mat mask = cv::Mat::zeros(cv::Size(cv_ptr->image.cols, cv_ptr->image.rows), CV_8UC1);
	cv::Point points[4] =
	{
		cv::Point(0, cv_ptr->image.rows),
		cv::Point(cv_ptr->image.cols, cv_ptr->image.rows),
		cv::Point(cv_ptr->image.cols - 60, cv_ptr->image.rows / 3),
		cv::Point(60, cv_ptr->image.rows / 3)
	};
	cv::fillConvexPoly(mask, points, 4, cv::Scalar(255, 0, 0));

  Camera_matrix = cv_ptr->image;

  cv::warpPerspective(Camera_matrix, Wrapped_frame, Matrix, Wrapped_frame.size(), cv::INTER_LINEAR + cv::WARP_FILL_OUTLIERS);
	cv::cvtColor(Camera_matrix, cv_gray->image, cv::COLOR_BGR2GRAY);
	cv::threshold(cv_gray->image, Frame, 180, 255, cv::THRESH_BINARY);
	cv::bitwise_and(Frame, mask, Masked_frame);
	cv::medianBlur(Masked_frame, Masked_frame, 5);
	cv::filter2D(Masked_frame, Frame_canny, -1, kernel_v, anchor, 0, cv::BORDER_DEFAULT);
	//laneDetector.Hsv_both(HSV_frame, frame_out_yellow, frame_out_white, frame_out_edge_yellow, frame_out_edge_white);

	// Detect lines
	laneDetector.detectLine_both(Frame_canny, white_vector);

	laneDetector.drawPoints_both(white_vector, white_vector_frame);
	laneDetector.AvgSlope(white_vector, white_vector_frame);


    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);

	cv::imshow("Grayscale", cv_gray->image);
  cv::imshow("Binarize frame", Frame_canny);
	cv::imshow("Vector frame", white_vector_frame);
  cv::imshow("Warped frame", Wrapped_frame);

	yellow_vector_frame = cv::Mat::zeros(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC3);
	white_vector_frame = cv::Mat::zeros(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC3);

    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
