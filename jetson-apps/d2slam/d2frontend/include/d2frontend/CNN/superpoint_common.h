#pragma once
#include <ATen/ATen.h>
#include <torch/csrc/api/include/torch/types.h>

#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>

#define SP_DESC_RAW_LEN 256

namespace D2FrontEnd {
void getKeyPoints(const cv::Mat &prob, float threshold, int nms_dist,
                  std::vector<cv::Point2f> &keypoints,
                  std::vector<float> &scores, int width, int height,
                  int max_num);
void computeDescriptors(const torch::Tensor &mProb, const torch::Tensor &desc,
                        const std::vector<cv::Point2f> &keypoints,
                        std::vector<float> &local_descriptors, int width,
                        int height, const Eigen::MatrixXf &pca_comp_T,
                        const Eigen::RowVectorXf &pca_mean);
}  // namespace D2FrontEnd
