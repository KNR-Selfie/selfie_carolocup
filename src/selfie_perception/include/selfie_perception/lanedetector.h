#pragma once

#include <ros/ros.h>
#include <iostream>
#include <string>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <selfie_msgs/RoadMarkings.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud.h>
#include <math.h>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/lu.hpp>
#include <vector>
#include <stdexcept>

#include <visualization_msgs/Marker.h>

class LaneDetector
{
  public:
	LaneDetector(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);
	~LaneDetector();
	bool init();

  private:
	ros::NodeHandle nh_;
	ros::NodeHandle pnh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	ros::Publisher lanes_pub_;

	cv::Size topview_size_;
	cv::Mat world2cam_;
	cv::Mat topview2world_;
	cv::Mat topview2cam_;
  cv::Mat world2topview_;

	cv::Mat kernel_v_;
	cv::Mat current_frame_;
	cv::Mat gray_frame_;
	cv::Mat binary_frame_;
	cv::Mat dynamic_mask_;
	cv::Mat masked_frame_;
	cv::Mat crossing_ROI_;
	cv::Mat crossing_frame_;
	cv::Mat canny_frame_;
	cv::Mat visualization_frame_;
	cv::Mat homography_frame_;
	cv::Mat testCrossing;
	cv::Mat debug_frame_;

	std::vector<std::vector<cv::Point> > lanes_vector_;
	std::vector<std::vector<cv::Point2f> > lanes_vector_converted_;
	std::vector<std::vector<cv::Point2f> > aprox_lines_frame_coordinate_;

	std::vector<float> last_left_coeff_;
	std::vector<float> last_middle_coeff_;
	std::vector<float> last_right_coeff_;
	std::vector<float> left_coeff_;
	std::vector<float> middle_coeff_;
	std::vector<float> right_coeff_;

	int left_line_index_;
	int center_line_index_;
	int right_line_index_;

	void imageCallback(const sensor_msgs::ImageConstPtr &msg);
	void computeTopView();
	void openCVVisualization();
	void mergeMiddleLane();
	void quickSortLinesY(int left, int right);
	void quickSortPointsY(std::vector<cv::Point> &vector_in, int left, int right);
	float getDistance(cv::Point2f p1, cv::Point2f p2);
	void recognizeLines();
	void publishMarkings();
	void detectLines(cv::Mat &input_frame, std::vector<std::vector<cv::Point> > &output_lanes);
	void drawPoints(cv::Mat &frame);
	void homography(cv::Mat input_frame, cv::Mat &homography_frame);
	void printInfoParams();
	void dynamicMask(cv::Mat &input_frame, cv::Mat &output_frame);
	void crossingLane(cv::Mat &input_frame, cv::Mat &output_frame, std::vector<std::vector<cv::Point2f> > lanes_vector);
	void crossingLaneLeft(cv::Mat &input_frame, cv::Mat &output_frame, std::vector<std::vector<cv::Point2f> > lanes_vector);
	void filterSmallLines();
	void convertCoordinates();
	float getAproxY(std::vector<float> coeff, float x);
	void convertApproxToFrameCoordinate();
	void initRecognizeLines();
	void linesApproximation(std::vector<std::vector<cv::Point2f> > lanes_vector);

	void pointsRVIZVisualization();
	void aproxVisualization();
	sensor_msgs::PointCloud points_cloud_;
	ros::Publisher points_cloud_pub_;
	ros::Publisher aprox_visualization_pub_;
	void filterPoints();
	void lanesVectorVisualization(cv::Mat &visualization_frame);
	void removeCar(cv::Mat &frame);
	void addBottomPoint();
	bool polyfit(int nDegree, std::vector<cv::Point2f> line, std::vector<float> &coeff);
	std::vector<float> adjust(std::vector<float> good_poly_coeff, std::vector<cv::Point2f> line);
	void calcRoadWidth();
	void generatePoints();
	void removeHorizontalLines();
	std::vector<cv::Point2f> createOffsetLine(std::vector<float> coeff, float offset);

	float min_length_search_line_;
	float min_length_lane_;
	float max_delta_y_lane_;
	float min_length_to_aprox_;
	float lane_width_;

	std::string config_file_;
	float binary_treshold_;
	bool debug_mode_;
	float max_mid_line_distance_;
	float max_mid_line_gap_;
	bool init_imageCallback_;
	float nominal_center_line_Y_;
	float points_density_;
	int poly_nDegree_;
};
