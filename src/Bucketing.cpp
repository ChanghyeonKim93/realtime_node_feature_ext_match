#include "Bucketing.h"

void Bucketing::randsample(const int& num_pts, const int& N_sample, std::vector<int>& sub_idx){
  std::vector<int> full_idx(num_pts,0);
  for(int i=0;i<num_pts;i++) full_idx[i] = i;

  std::random_shuffle(full_idx.begin(),full_idx.end());
  std::vector<int> null_vec;
  sub_idx.swap(null_vec);
  sub_idx.resize(N_sample);

  for(int i=0;i<N_sample;i++){  //sampling specific numbers.
    sub_idx[i] = full_idx[i];
  }
};


void Bucketing::bucketing(const std::vector<cv::Point2f>& key_features, const int& max_num_features, int bucket_size, int num_rows, int num_cols){

  int feature_bucket[bucket_size*bucket_size]={0};
  double inverse_bucket[bucket_size*bucket_size]={0.0};
  int den_rows, den_cols;
  int residue_num_features = max_num_features-key_features.size();
  den_rows = (int)(num_rows/bucket_size);
  den_cols = (int)(num_cols/bucket_size);

  int ind_rows,ind_cols;
  for(int i=0; i<key_features.size(); i++){
     ind_rows=(int)(key_features[i].x/den_rows);
     ind_cols=(int)(key_features[i].y/den_cols);
     feature_bucket[ind_rows+ind_cols*bucket_size]+=1;
  }

  // inverse ratio
  double inv_ratio_sum = 0.0;
  for(int i=0; i<bucket_size*bucket_size;i++){
    if(feature_bucket[i]==0) inverse_bucket[i]=0;
    else inverse_bucket[i] = 1.0/(double)feature_bucket[i];
    inv_ratio_sum+=inverse_bucket[i];
  }
  inv_ratio_sum = 1.0/inv_ratio_sum;
  for(int i=0; i<bucket_size*bucket_size;i++) inverse_bucket[i]*=inv_ratio_sum;

  

};
