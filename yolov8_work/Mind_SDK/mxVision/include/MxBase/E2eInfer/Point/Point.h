/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
 * Description: Constructing Point Class.
 * Author: MindX SDK
 * Create: 2022
 * History: NA
 */

#ifndef MX_POINT_H
#define MX_POINT_H

namespace MxBase {
struct Point {
    Point()
        : x(0), y(0) {};
    Point(const uint32_t inputX, const uint32_t inputY)
        : x(inputX), y(inputY) {};

    uint32_t x = 0;
    uint32_t y = 0;
};
}
#endif