/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
 * Description: data structure helper.
 * Author: MindX SDK
 * Create: 2021
 * History: NA
 */

#ifndef MXSTREAM_DATAHELPER_H
#define MXSTREAM_DATAHELPER_H

#include "MxStream/StreamManager/MxsmDataType.h"

namespace MxStream {
class DataHelper {
public:
    static MxstBufferInput ReadImage(const std::string& imagePath);
    static std::string ReadFile(const std::string& filePath);
};
}
#endif // MXSTREAM_DATAHELPER_H
