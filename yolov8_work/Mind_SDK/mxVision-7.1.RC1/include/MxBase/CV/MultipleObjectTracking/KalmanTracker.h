/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
 * Description: Blocking Queue Function Implementation.
 * Author: MindX SDK
 * Create: 2020
 * History: NA
 */

#ifndef KALMANTRACKER_H
#define KALMANTRACKER_H

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include "MxBase/CV/Core/DataType.h"
#include "MxBase/Log/Log.h"
#include <cmath>

namespace MxBase {
const int OFFSET = 2;
const int MULTIPLE = 2;

class KalmanTracker {
public:
    KalmanTracker() {}

    ~KalmanTracker() {}

    void CvKalmanInit(const MxBase::DetectBox &initRect);

    MxBase::DetectBox Predict();

    void Update(const MxBase::DetectBox &stateMat);

private:
    cv::KalmanFilter cvkalmanfilter_ = {};
    cv::Mat measurement_ = {};
    bool isInitialized_ = false;
};
}
#endif
