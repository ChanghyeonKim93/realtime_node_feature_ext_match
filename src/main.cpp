#include <ros/ros.h>

#include "MyAlgorithm.h"

#include <iostream>
#include <sys/time.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry> // for Lie algeb

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/image_encodings.h>

// for IMU
#include <sensor_msgs/Imu.h>

// for VICON
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

// threading
#include <thread>
#include "Communicator.h"

typedef Eigen::Matrix<double,9,1> ImuVector;
ImuVector current_imu = ImuVector::Zero();

bool image_updated     = false;
bool imu_updated       = false;
bool algorithm_updated = false;
typedef std::string TopicTimeStr;
typedef double TopicTime;
TopicTimeStr image_time_str, imu_time_str;
TopicTime    image_time,     imu_time;

cv::Mat current_image;

inline std::string dtos(double x);
void image_callback(const sensor_msgs::ImageConstPtr& msg);
void imu_callback(const sensor_msgs::ImuConstPtr& msg);


int main(int argc, char **argv){

	ros::init(argc, argv, "realtime_node");
	ros::NodeHandle nh("~");

	std::string image_topic_name, imu_topic_name;
	bool debug_flag;
	ros::param::get("~image_topic_name", image_topic_name);
	ros::param::get("~imu_topic_name", imu_topic_name);
	ros::param::get("~debug_image",debug_flag);

	ros::Subscriber image_sub,imu_sub;
	image_sub = nh.subscribe<sensor_msgs::Image>(image_topic_name,10,&image_callback);
	imu_sub   = nh.subscribe<sensor_msgs::Imu>(imu_topic_name,10,&imu_callback);

	MyAlgorithm::Parameters parameters;
	parameters.debug_flag.print_image = debug_flag;

	Communicator communicator;
	MyAlgorithm *my_algorithm = new MyAlgorithm(parameters);

	while(ros::ok()) {
		ros::spinOnce(); // so fast
		if(image_updated == true) {
			// read imu queues.
			my_algorithm->image_acquisition(current_image, image_time); // read image.
			my_algorithm->run(); // run main loop of the algorithm.
			image_updated = false;
		}
	}

	delete my_algorithm;
	return 0;
}

//========================================

inline std::string dtos(double x){
	std::stringstream s;
	s<<std::setprecision(6) << std::fixed << x;
	return s.str();
}

void image_callback(const sensor_msgs::ImageConstPtr& msg){
	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
	current_image = cv_ptr->image;
	image_time = (double)(msg->header.stamp.sec*1e6+msg->header.stamp.nsec/1000)/1000000.0;
	image_time_str = dtos(image_time);
	image_updated = true;
	ROS_INFO_STREAM("SUBSCRIBER : Image updated");
}

void imu_callback(const sensor_msgs::ImuConstPtr& msg){
	current_imu(0,0) = msg->linear_acceleration.x;
	current_imu(1,0) = msg->linear_acceleration.y;
	current_imu(2,0) = msg->linear_acceleration.z;
	current_imu(3,0) = msg->angular_velocity.x;
	current_imu(4,0) = msg->angular_velocity.y;
	current_imu(5,0) = msg->angular_velocity.z;
	imu_time = (double)(msg->header.stamp.sec*1e6+msg->header.stamp.nsec/1000)/1000000.0;
	imu_time_str = dtos(imu_time);
	imu_updated  = true;
	ROS_INFO_STREAM("SUBSCRIBER : Imu updated");
}
