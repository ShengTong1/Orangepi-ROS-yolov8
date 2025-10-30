/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
 * Description: Constructing Size Class.
 * Author: MindX SDK
 * Create: 2022
 * History: NA
 */

#ifndef MX_SIZE_H
#define MX_SIZE_H

namespace MxBase {
struct Size {
    Size()
        : width(0), height(0) {};
    Size(const uint32_t inputWidth, const uint32_t inputHeight)
        : width(inputWidth), height(inputHeight) {};

    uint32_t width = 0;
    uint32_t height = 0;
};
}
#endif