/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
 * Description: Initialize the matching matrix of the Hungarian algorithm.
 * Author: MindX SDK
 * Create: 2020
 * History: NA
 */

#ifndef HUANGARIAN_H
#define HUANGARIAN_H

#include <vector>
#include <memory>
#include "MxBase/ErrorCode/ErrorCode.h"
#include "MxBase/Log/Log.h"

namespace MxBase {
const int INF = 0x3f3f3f3f;
const int VISITED = 1;
const int HUNGARIAN_CONTENT = 7;

const int X_MATCH_OFFSET = 0;
const int Y_MATCH_OFFSET = 1;
const int X_VALUE_OFFSET = 2;
const int Y_VALUE_OFFSET = 3;
const int SLACK_OFFSET = 4;
const int X_VISIT_OFFSET = 5;
const int Y_VISIT_OFFSET = 6;

struct HungarianHandle {
    int rows;
    int cols;
    int max;
    int* resX;
    int* resY;
    bool transpose;
    std::shared_ptr<int> adjMat;
    std::shared_ptr<int> xMatch;
    std::shared_ptr<int> yMatch;
    std::shared_ptr<int> xValue;
    std::shared_ptr<int> yValue;
    std::shared_ptr<int> slack;
    std::shared_ptr<int> xVisit;
    std::shared_ptr<int> yVisit;
};

/**
 * @description init Hungarian Method param
 */
APP_ERROR HungarianHandleInit(HungarianHandle &handle, int row, int cols);

/**
 * @description this function integrates the Hungarian Method,
 *              you should invoke HungarianHandleInit function first
 */
APP_ERROR HungarianSolve(HungarianHandle &handle, const std::vector<std::vector<int>> &cost,
                         const int rows, const int cols);
}
#endif
