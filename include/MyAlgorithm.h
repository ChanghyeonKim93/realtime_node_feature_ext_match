#ifndef _MY_ALGORITHM_H_
#define _MY_ALGORITHM_H_

#define PI 3.141592

#include <ros/ros.h>

#include <iostream>
#include <sys/time.h>
#include <Eigen/Dense>
#include <ctime>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <opencv2/core/eigen.hpp> // opencv with Eigen .

// standard STL
#include <list>
#include <vector>

// c++ 11 for random
#include <algorithm>
#include <random>

// threading
#include <thread>
#include "Communicator.h"


// my libraries
//#include "Bucketing.h"

typedef std::string TopicTimeStr;
typedef double TopicTime;

class MyAlgorithm{

public:

  struct Calibration {
    double fx;  //  focal length x
    double fy;  //  focal length y
    double cx;  //  principal point (u-coordinate)
    double cy;  //  principal point (v-coordinate)
    int width;
    int height;
    Calibration () {
    //  fx     = 620.608832234754;
    //  fy     = 619.113993685335;
    //  cx     = 323.902900972212;
    //  cy     = 212.418428046497; // for ocam
      fx     = 614.2090454101562;
      fy     = 619.4898071289062;
      cx     = 302.99072265625;
      cy     = 222.56651306152344; // for realsense R200
      width  = 640;
      height = 480;
    }
  };

  // debug parameters
  struct DebugFlag {
    bool print_image;
    DebugFlag () {
      print_image = false;
    }
  };

  struct FeatureDetector {
    int max_features;
    int block_size;
    int bucket_size;
    int fast_intensity_gap;
    double quality_level;
    double min_distance;
    double k;
    bool use_Harris_detector;
    FeatureDetector () {
      max_features  = 300;
      quality_level = 0.06;
      min_distance  = 5;
      block_size    = 5;
      bucket_size   = 2;
      fast_intensity_gap  = 35;
      use_Harris_detector = false;
      k = 0.03;
    }
  };

  struct FeatureTracker {
    int win_size;
    int max_level;
    int flags_R;
    FeatureTracker () {
      win_size = 35;
      max_level = 5;
      flags_R = 1;
    }
  };

  struct Parameters {
    MyAlgorithm::Calibration calibration;
    MyAlgorithm::DebugFlag debug_flag;
    MyAlgorithm::FeatureDetector feature_detector;
    MyAlgorithm::FeatureTracker feature_tracker;
  };

  bool complete_flag;

  MyAlgorithm(Parameters parameters);
  ~MyAlgorithm();
  void image_acquisition(const cv::Mat& img,const TopicTime& curr_time);
  void point_plot(cv::Mat& result_image, const std::vector<cv::KeyPoint>& input_features, cv::Scalar& line_color);

  void run();

  // scripts
  void bucketing_feature_detection();
  void initialize();
  void tracking_current_features();
  void changed_prev_image();
  void debug_plotting();
  std::thread runThread(Communicator* communicator);

private:
  Parameters my_param;
  TopicTime curr_time;

  cv::Mat curr_image;
  cv::Mat prev_image;
  cv::Mat debug_image;
  std::vector<cv::Mat> prev_imgpyr, curr_imgpyr;
  std::vector<cv::Mat> mask_img;

  std::vector<cv::KeyPoint> curr_features;
  std::vector<cv::KeyPoint> prev_features;

  std::vector<cv::KeyPoint> curr_features_valid;
  std::vector<cv::KeyPoint> prev_features_valid;

  std::vector<cv::Point2f> prev_features_2f;
  std::vector<cv::Point2f> curr_features_2f;

  int num_features_valid;
  int total_num_images;
  int instant_num_images;
  bool initial_loop;

  cv::Ptr<cv::Feature2D> fts_detector;

  // For R, T, E, K ... etc.
  cv::Mat K;
  Eigen::MatrixXd K_eigen; // 정의 할 때, Eigen은 공간의 크기를 지정해놓으면 안된다. class 내부에서.
  Eigen::MatrixXd R_eigen;
  Eigen::MatrixXd T_eigen;
  Eigen::MatrixXd g_eigen;
};
#endif
