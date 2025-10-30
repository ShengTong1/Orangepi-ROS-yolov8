/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
 * Description: Constructing Rect Class.
 * Author: MindX SDK
 * Create: 2022
 * History: NA
 */

#ifndef MX_RECT_H
#define MX_RECT_H

#include "MxBase/E2eInfer/Point/Point.h"

namespace MxBase {
struct Rect {
    Rect()
        : x0(0), y0(0), x1(0), y1(0) {};
    Rect(const uint32_t leftTopX, const uint32_t leftTopY, const uint32_t rightBottomX, const uint32_t rightBottomY)
        : x0(leftTopX), y0(leftTopY), x1(rightBottomX), y1(rightBottomY) {};
    Rect(const Point leftTop, const Point rightBottom)
        : x0(leftTop.x), y0(leftTop.y), x1(rightBottom.x), y1(rightBottom.y) {};

    uint32_t x0 = 0;
    uint32_t y0 = 0;
    uint32_t x1 = 0;
    uint32_t y1 = 0;
};
}
#endif