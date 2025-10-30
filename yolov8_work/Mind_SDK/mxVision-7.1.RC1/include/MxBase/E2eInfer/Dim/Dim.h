/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
 * Description: Constructing Rect Class.
 * Author: MindX SDK
 * Create: 2022
 * History: NA
 */

#ifndef MX_DIM_H
#define MX_DIM_H

namespace MxBase {
    struct Dim {
        Dim() : left(0), right(0), top(0), bottom(0) {};

        Dim(const uint32_t inputDim) : left(inputDim), right(inputDim), top(inputDim), bottom(inputDim) {};

        Dim(const uint32_t inputLeft, const uint32_t inputRight, const uint32_t inputTop, const uint32_t inputBottom)
            : left(inputLeft), right(inputRight), top(inputTop), bottom(inputBottom) {};
        uint32_t left;
        uint32_t right;
        uint32_t top;
        uint32_t bottom;
    };
}

#endif
