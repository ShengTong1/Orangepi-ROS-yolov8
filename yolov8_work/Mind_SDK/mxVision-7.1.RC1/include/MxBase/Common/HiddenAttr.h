/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
 * Description: Define macros for other files.
 * Author: MindX SDK
 * Create: 2020
 * History: NA
 */

#ifndef MINDX_SDK_HIDDENATTR_H
#define MINDX_SDK_HIDDENATTR_H

#define SDK_AVAILABLE_FOR_OUT __attribute__((visibility("default")))

#define SDK_AVAILABLE_FOR_IN __attribute__((visibility("default")))

#define SDK_UNAVAILABLE_FOR_OTHER __attribute__((visibility("hidden")))

#define SDK_DEPRECATED_FOR(f) __attribute__((__deprecated__("Use '" #f "' instead")))

#endif