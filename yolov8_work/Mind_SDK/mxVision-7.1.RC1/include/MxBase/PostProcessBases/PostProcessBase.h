/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
 * Description: Used to initialize the model post-processing.
 * Author: MindX SDK
 * Create: 2021
 * History: NA
 */

#ifndef POST_PROCESS_BASE_H
#define POST_PROCESS_BASE_H

#include <vector>
#include <map>
#include <string>

#include "MxBase/PostProcessBases/PostProcessDataType.h"
#include "MxBase/Tensor/TensorBase/TensorBase.h"
#include "MxBase/ConfigUtil/ConfigUtil.h"
#include "MxBase/ErrorCode/ErrorCode.h"
#include "MxBase/Common/Version.h"

namespace MxBase {
class PostProcessBase {
public:
    PostProcessBase() = default;

    PostProcessBase(const PostProcessBase &other) = default;

    virtual ~PostProcessBase() = default;

    PostProcessBase& operator=(const PostProcessBase &other);

    virtual APP_ERROR Init(const std::map<std::string, std::string> &postConfig);

    virtual APP_ERROR DeInit();

    virtual uint64_t GetCurrentVersion();

protected:
    APP_ERROR LoadConfigData(const std::map<std::string, std::string> &postConfig);

    APP_ERROR CheckAndMoveTensors(std::vector<TensorBase> &tensors);

    void* GetBuffer(const TensorBase& tensor, uint32_t index) const;

    bool JudgeResizeType(const ResizedImageInfo& resizedImageInfo);

protected:
    MxBase::ConfigData configData_;
    bool checkModelFlag_ = true;
    bool isInitConfig_ = false;

    const uint32_t ZERO_BYTE = 0;
    const uint32_t ONE_BYTE = 1;
    const uint32_t TWO_BYTE = 2;
    const uint32_t FOUR_BYTE = 4;
    const uint32_t EIGHT_BYTE = 8;
    const uint32_t MAX_TENSOR_VEC_SIZE = 128;
};
}


#endif