/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
 * Description: DataStruct of the Model Inference Function.
 * Author: MindX SDK
 * Create: 2022
 * History: NA
 */

#ifndef MODEL_DATA_TYPE_H
#define MODEL_DATA_TYPE_H

#include <memory>
#include <vector>

namespace MxBase {
struct TensorDesc {
    size_t tensorSize;
    std::string tensorName;
    std::vector<int64_t> tensorDims;
};

struct ImageSize {
    size_t height;
    size_t width;

    ImageSize() = default;

    ImageSize(size_t height, size_t width)
    {
        this->width = width;
        this->height = height;
    }
};

enum DynamicType {
    STATIC_BATCH = 0,
    DYNAMIC_BATCH = 1,
    DYNAMIC_HW = 2,
    DYNAMIC_DIMS = 3,
    DYNAMIC_SHAPE = 4
};

enum DataFormat {
    NCHW = 0,
    NHWC = 1
};

struct DynamicInfo {
    DynamicType dynamicType = DYNAMIC_BATCH;
    size_t batchSize;
    ImageSize imageSize = {};
    std::vector<std::vector<uint32_t>> shape = {};
};

struct ModelDesc {
    std::vector<TensorDesc> inputTensors;
    std::vector<TensorDesc> outputTensors;
    std::vector<size_t> batchSizes;
    bool dynamicBatch;
};

struct ModelDataset {
    void* mdlDataPtr;
    size_t dynamicBatchSize;
};

struct PostImageInfo {
    uint32_t widthOriginal = 0;
    uint32_t heightOriginal = 0;
    uint32_t widthResize = 0;
    uint32_t heightResize = 0;
    float x0 = 0;
    float y0 = 0;
    float x1 = 0;
    float y1 = 0;
};
}
#endif