#ifndef _COMMUNICATOR_H_
#define _COMMUNICATOR_H_

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <list>
#include <vector>
#include <queue>

#include <thread>


class Communicator{
public:

private:
  cv::Mat current_image;
  std::queue<double> a;
};

#endif
