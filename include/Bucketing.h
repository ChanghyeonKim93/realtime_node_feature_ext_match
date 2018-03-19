#ifndef _BUCKETING_H_
#define _BUCKETING_H_

#include <iostream>
#include <ros/ros.h>
#include <sys/time.h>
#include <Eigen/Dense>
#include <ctime>
#include <string>

// STL
#include <list>
#include <vector>

// c++ 11 for random
#include <algorithm>
#include <random>

#include <opencv2/core.hpp>

namespace Bucketing {

  void randsample(const int& num_pts, const int& N_sample, std::vector<int>& sub_idx);
  void bucketing(const std::vector<cv::Point2f>& key_features, const int& max_num_features, int bucket_size, int num_rows, int num_cols);
};


#endif
