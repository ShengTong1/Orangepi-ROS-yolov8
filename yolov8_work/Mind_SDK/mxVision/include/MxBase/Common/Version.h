/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
 * Description: Obtaining the SDK Version.
 * Author: MindX SDK
 * Create: 2020
 * History: NA
 */

#ifndef MINDX_SDK_VERSION_H
#define MINDX_SDK_VERSION_H

#define MINDX_SDK_VERSION_MAJOR 5
#define MINDX_SDK_VERSION_MINOR 0
#define MINDX_SDK_VERSION_MICRO 0
#define VERSION_INTERVAL 1000

#include <string>
#include "MxBase/Common/HiddenAttr.h"
namespace MxBase {
/**
 * Get MindX SDK mxVision version
 *
 * @return Version number: string.
 *
 */
std::string GetSDKVersion();
}

/**
 * Get MindX SDK version
 *
 * @return Version number calculated from major, minor, and micro version, [0, UINT64_MAX].
 *
 */
#define MINDX_SDK_VERSION ((uint64_t)(MINDX_SDK_VERSION_MAJOR) * (VERSION_INTERVAL) * (VERSION_INTERVAL) + \
    (uint64_t)(MINDX_SDK_VERSION_MINOR) * (VERSION_INTERVAL) + (uint64_t)(MINDX_SDK_VERSION_MICRO))

/**
 * Get MindX SDK major version.
 *
 * @return Major version, [0, UINT32_MAX].
 */
#define MINDX_SDK_MAJOR_VERSION MINDX_SDK_VERSION_MAJOR

/**
 * Get MindX SDK major version.
 *
 * @return Minor version, [0, UINT32_MAX].
 */
#define MINDX_SDK_MINOR_VERSION MINDX_SDK_VERSION_MINOR

/**
 * Get MindX SDK major version.
 *
 * @return Micro version, [0, UINT32_MAX].
 */
#define MINDX_SDK_MICRO_VERSION MINDX_SDK_VERSION_MICRO

#endif
