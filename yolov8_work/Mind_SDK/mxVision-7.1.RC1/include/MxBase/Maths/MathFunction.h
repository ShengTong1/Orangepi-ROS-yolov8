/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
 * Description: linear fitting function.
 * Author: MindX SDK
 * Create: 2020
 * History: NA
 */

#ifndef MATHFUNCTION_H
#define MATHFUNCTION_H

#include <vector>
#include <cmath>
#include "MxBase/Log/Log.h"

class LineRegressionFit {
public:
    LineRegressionFit() {};
    ~LineRegressionFit() {};

    double LRFunction(const float &x) const
    {
        return alpha_ * x + beta_;
    }

    void SetAlphaAndBeta(const std::vector<float> &xVec, const std::vector<float> &yVec)
    {
        double douX = 0;
        double sumX = 0;
        double xMulY = 0;
        double douY = 0;
        const double EPSILON_DBL = 1e-15;
        if ((xVec.size() == 0) || (yVec.size() != xVec.size())) {
            LogWarn << "Size of xVec(" << xVec.size() << ") or yVec(" << yVec.size() << ") is invalid";
            return;
        }
        for (size_t i = 0; i < xVec.size(); ++i) {
            douX += xVec[i] * xVec[i];
            sumX += xVec[i];
            xMulY += xVec[i] * yVec[i];
            douY += yVec[i];
        }
        double divideNum = douX * xVec.size() - sumX * sumX;
        if (std::fabs(divideNum) < EPSILON_DBL) {
            LogWarn << "In line regression fit function, the divisor is " << divideNum;
            return;
        }
        alpha_ = (xMulY * xVec.size() - sumX * douY) / (douX * xVec.size() - sumX * sumX);
        beta_ = (douX * douY - sumX * xMulY) / (douX * xVec.size() - sumX * sumX);
    }

public:
    double alpha_ = 0.0;
    double beta_ = 0.0;
};

#endif // MATHFUNCTION_H
