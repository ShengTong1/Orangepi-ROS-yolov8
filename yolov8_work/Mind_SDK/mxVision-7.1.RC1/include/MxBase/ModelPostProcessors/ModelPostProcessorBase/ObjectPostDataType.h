/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
 * Description: Data Structure of Inference Post-processing for Target Detection.
 * Author: MindX SDK
 * Create: 2020
 * History: NA
 */

#ifndef OBJECTPOSTDATATYPE_H
#define OBJECTPOSTDATATYPE_H

#include <algorithm>
#include <vector>
#include <memory>

const float COORDINATE_PARAM = 2.0;
const int BOX_DIM = 4;
const int VECTOR_FIRST_INDEX = 0;
const int VECTOR_SECOND_INDEX = 1;
const int VECTOR_THIRD_INDEX = 2;
const int VECTOR_FOURTH_INDEX = 3;
const int VECTOR_FIFTH_INDEX = 4;
const int FPN_BOX_DIM = 5;
const int ANCHOR_NUM = 6;

struct OutputLayer {
    int layerIdx;
    int width;
    int height;
    float anchors[ANCHOR_NUM];
};

struct NetInfo {
    int anchorDim;
    int classNum;
    int bboxDim;
    int netWidth;
    int netHeight;
    std::vector<OutputLayer> outputLayers;
};

struct ImageInfo {
    int modelWidth;
    int modelHeight;
    int imgWidth;
    int imgHeight;
};

// Detect Info which could be transformed by DetectBox
struct ObjDetectInfo {
    float x0 = 0;
    float y0 = 0;
    float x1 = 0;
    float y1 = 0;
    float confidence = 0;
    int classId = 0;
    void *maskPtr;
};

#endif
