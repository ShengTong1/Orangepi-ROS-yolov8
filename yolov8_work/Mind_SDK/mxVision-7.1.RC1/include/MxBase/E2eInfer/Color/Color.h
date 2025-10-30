/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
 * Description: Constructing Rect Class.
 * Author: MindX SDK
 * Create: 2022
 * History: NA
 */

#ifndef MX_COLOR_H
#define MX_COLOR_H

namespace MxBase {
    struct Color {
        Color() : channel_zero(0), channel_one(0), channel_two(0) {};

        Color(const uint32_t inputRed, const uint32_t inputGreen, const uint32_t inputBlue)
            : channel_zero(inputRed), channel_one(inputGreen), channel_two(inputBlue) {};
        uint32_t channel_zero;
        uint32_t channel_one;
        uint32_t channel_two;
    };
}

#endif
