/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
 * Description: Classification and Target Detection Related Data.
 * Author: MindX SDK
 * Create: 2020
 * History: NA
 */

#ifndef DATATYPE_H
#define DATATYPE_H

#include <string>

namespace MxBase {
// Box information
struct DetectBox {
    float prob;
    int classID;
    float x;
    float y;
    float width;
    float height;
    std::string className;
    void *maskPtr;
};

// RoiBox
struct RoiBox {
    float x0;
    float y0;
    float x1;
    float y1;
};

enum IOUMethod {
    MAX = 0,
    MIN = 1,
    UNION = 2,
    DIOU = 3
};

enum TrackFlag {
    NEW_OBJECT = 0,
    TRACKED_OBJECT = 1,
    LOST_OBJECT = 2
};
}  // namespace MxBase
#endif
