#include "MyAlgorithm.h"

#include <string>
#include <iostream>
#include <stdio.h>
#include <sys/time.h>

MyAlgorithm::MyAlgorithm(Parameters parameters):my_param(parameters){
  if(my_param.debug_flag.print_image == true) cv::namedWindow("current image");
  this->complete_flag = true;
  this->initial_loop  = true;
  this->total_num_images = 1;
  this->instant_num_images = 1;
  // create bucketing mask vector.
  int den_rows, den_cols;
  den_rows = (int)(my_param.calibration.width/my_param.feature_detector.bucket_size);
  den_cols = (int)(my_param.calibration.height/my_param.feature_detector.bucket_size);

  for(int v=0;v<my_param.feature_detector.bucket_size;v++){
    for(int u=0;u<my_param.feature_detector.bucket_size;u++){
      cv::Mat temp = cv::Mat::zeros(my_param.calibration.height,my_param.calibration.width,CV_8U);
      std::cout<<cv::Rect(v*den_cols,u*den_rows,den_cols,den_rows)<<std::endl;
      temp(cv::Rect(u*den_rows,v*den_cols,den_rows,den_cols)).setTo(cv::Scalar(255));
      mask_img.push_back(temp);
    }
  }
}

MyAlgorithm::~MyAlgorithm(){
  ROS_INFO_STREAM("Realtime node is terminated.\n");
}


void MyAlgorithm::point_plot(cv::Mat& result_image, const std::vector<cv::Point2f>& input_features, cv::Scalar& line_color){
  int width = my_param.calibration.width;
  int height = my_param.calibration.height;
  // cv::Scalar line_color = CV_RGB(0,255,255);
  for(int i=0; i < (int)input_features.size(); i++){
    cv::Point2f p;
    p = input_features[i];
    cv::circle(result_image, p, 2,line_color);
  }
}

void MyAlgorithm::image_acquisition(const cv::Mat& img,const TopicTime& curr_time_input){
  this->curr_image = img.clone();
  this->curr_time  = curr_time_input;
  ROS_INFO_STREAM("In algorithm.");
}

void MyAlgorithm::run(){
  std::vector<cv::KeyPoint> prev_features2;

  if(initial_loop == true){
    cv::goodFeaturesToTrack(curr_image,prev_features,my_param.feature_detector.max_features, my_param.feature_detector.quality_level, my_param.feature_detector.min_distance, cv::Mat(), my_param.feature_detector.block_size, my_param.feature_detector.use_Harris_detector, my_param.feature_detector.k);
    //cv::FAST(curr_image,prev_features2,5);
    //if(curr_features.size()>0) cv::cornerSubPix(curr_image, prev_features, cv::Size(7,7), cv::Size(-1,-1), cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03));
    prev_image = curr_image.clone();
    cv::buildOpticalFlowPyramid(prev_image, prev_imgpyr, cv::Size(my_param.feature_tracker.win_size, my_param.feature_tracker.win_size), my_param.feature_tracker.max_level, true);
    this->initial_loop = false;
  }

  //cv::Size patternsize(6,4);
  //cv::findChessboardCorners(curr_image,patternsize, curr_features);
  std::vector<uchar> status_features;
  std::vector<float> error;

  curr_features.clear();
  cv::buildOpticalFlowPyramid(curr_image, curr_imgpyr, cv::Size(my_param.feature_tracker.win_size, my_param.feature_tracker.win_size), my_param.feature_tracker.max_level, true);
  cv::calcOpticalFlowPyrLK(prev_imgpyr, curr_imgpyr, prev_features, curr_features, status_features, error, cv::Size(my_param.feature_tracker.win_size, my_param.feature_tracker.win_size), my_param.feature_tracker.max_level);
  //flags_R |= CV_LKFLOW_PYR_A_READY;
  // extract only valid features from prev_features_ and curr_features_
  prev_features_valid.clear();
  curr_features_valid.clear();

  num_features_valid = 0;
  for(int i = 0; i < (int)prev_features.size(); i++) {
    if ( status_features[i] != 0 ) {    // just skip if it is not valid feature
      prev_features_valid.push_back(prev_features[i]);
      curr_features_valid.push_back(curr_features[i]);
      num_features_valid += 1;
    }
  }
  std::cout<<"num of valid features : "<<num_features_valid<<std::endl;

  // save valid features only
  if(this->instant_num_images>=1){
    prev_features.clear();
    cv::cvtColor(curr_image, debug_image, cv::COLOR_GRAY2RGB);

    if(curr_features_valid.size()<=50) this->bucketing_feature_detection();

    for(int i = 0; i < (int)curr_features_valid.size(); i++) {
      prev_features.push_back(curr_features_valid[i]);
    }
    prev_image = curr_image.clone();
    cv::buildOpticalFlowPyramid(prev_image, prev_imgpyr, cv::Size(my_param.feature_tracker.win_size, my_param.feature_tracker.win_size), my_param.feature_tracker.max_level, true);
    this->instant_num_images=0;
  }
  //
  this->total_num_images++;
  this->instant_num_images++;

  if(my_param.debug_flag.print_image == true){
    //cv::cvtColor(curr_image, debug_image, cv::COLOR_GRAY2RGB);
    if(prev_features.size()>0){
      cv::Scalar prev_color = CV_RGB(255,0,0);
      cv::Scalar curr_color = CV_RGB(0,255,0);
      this->point_plot(debug_image,prev_features,prev_color);
      this->point_plot(debug_image,curr_features_valid,curr_color);
      for(int i=0;i<prev_features_valid.size();i++)  cv::line(debug_image, prev_features_valid[i], curr_features_valid[i], prev_color, 2, CV_AA);
    }
    cv::imshow("current image",debug_image);
    cv::waitKey(1);
    debug_image.release();
  }
  // algorithm execution in this function.
  ROS_INFO_STREAM("   algorithm is operated.");

  this->complete_flag = true;
}

void MyAlgorithm::bucketing_feature_detection(){
  int feature_bucket[my_param.feature_detector.bucket_size*my_param.feature_detector.bucket_size]={0};
  double inverse_bucket[my_param.feature_detector.bucket_size*my_param.feature_detector.bucket_size]={0.0};
  int residue_num_features = my_param.feature_detector.max_features-curr_features_valid.size();
  int den_rows, den_cols;
  den_rows = (int)(my_param.calibration.width/my_param.feature_detector.bucket_size);
  den_cols = (int)(my_param.calibration.height/my_param.feature_detector.bucket_size);

  int ind_rows,ind_cols;
  for(int i=0; i<curr_features_valid.size(); i++){
     ind_rows=(int)(curr_features_valid[i].x/den_rows);
     ind_cols=(int)(curr_features_valid[i].y/den_cols);
     feature_bucket[ind_cols+ind_rows*my_param.feature_detector.bucket_size]+=1;
  }
  // inverse ratio
  double inv_ratio_sum = 0.0;
  for(int i=0; i<my_param.feature_detector.bucket_size*my_param.feature_detector.bucket_size;i++){
    if(feature_bucket[i]==0) inverse_bucket[i]=0;
    else inverse_bucket[i] = 1.0/(double)feature_bucket[i];
    inv_ratio_sum += inverse_bucket[i];
  }
  inv_ratio_sum = 1.0/inv_ratio_sum;
  for(int i=0; i<my_param.feature_detector.bucket_size*my_param.feature_detector.bucket_size;i++) inverse_bucket[i]*=inv_ratio_sum;
  for(int i=0;i<my_param.feature_detector.bucket_size*my_param.feature_detector.bucket_size;i++){
    int num_extract = (int)(inverse_bucket[i]*residue_num_features);
    std::vector<cv::Point2f> temp_features;
    cv::goodFeaturesToTrack(curr_image, temp_features, num_extract, my_param.feature_detector.quality_level, my_param.feature_detector.min_distance, mask_img[i], my_param.feature_detector.block_size, my_param.feature_detector.use_Harris_detector, my_param.feature_detector.k);
    if(temp_features.size()>0){
       cv::cornerSubPix(curr_image, temp_features, cv::Size(5,5), cv::Size(-1,-1), cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.08));
       //for(int i=0;temp_features.size();i++) curr_features_valid.push_back(temp_features[i]);
       cv::Scalar curr_color = CV_RGB(255,255,0);
       this->point_plot(debug_image,temp_features,curr_color);
       //for(int i=0;i<temp_features.size();i++) std::cout<<temp_features[i]<<std::endl;

    }
    for(int i=0;i<curr_features_valid.size();i++) std::cout<<curr_features_valid[i]<<std::endl;

  }
}
