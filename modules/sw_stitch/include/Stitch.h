#ifndef STITCH_HPP
#define STITCH_HPP

#include <string>
#include <vector>
#include <opencv2/core.hpp>

int InitStitch(std::string intrinsic_yml, std::string stitch_yml);

cv::Mat Stitch(std::vector<cv::Mat>& inputs);

void ReleaseStitch();

#endif

