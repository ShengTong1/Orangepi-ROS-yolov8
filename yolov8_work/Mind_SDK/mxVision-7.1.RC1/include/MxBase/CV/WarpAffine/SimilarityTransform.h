/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
 * Description: Compute affine transformation matrices.
 * Author: MindX SDK
 * Create: 2020
 * History: NA
 */

#ifndef SIMILARITY_TRANSFORM_H
#define SIMILARITY_TRANSFORM_H

#include <vector>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include "MxBase/ErrorCode/ErrorCode.h"

namespace MxBase {
class SimilarityTransform {
public:
    // Constructor
    SimilarityTransform();
    // Destructor
    ~SimilarityTransform();
    // Calculate the affine transformation matrix
    cv::Mat Transform(const std::vector<cv::Point2f> &srcPoint, const std::vector<cv::Point2f> &dstPoint) const;

private:
    // Calculate the mean value
    cv::Point2f GetMean(const std::vector<cv::Point2f> &srcPoint) const;
    // Calculate the variance
    double GetSumVars(const cv::Mat &array) const;
};
}  // namespace MxBase
#endif