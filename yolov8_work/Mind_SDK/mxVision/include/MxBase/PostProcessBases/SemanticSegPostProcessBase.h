/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
 * Description: Base class for semantic segmentation task post-processing.
 * Author: MindX SDK
 * Create: 2020
 * History: NA
 */

#ifndef SEGMENT_POST_PROCESS_H
#define SEGMENT_POST_PROCESS_H
#include "MxBase/PostProcessBases/ImagePostProcessBase.h"

namespace MxBase {
class SemanticSegPostProcessBase : public ImagePostProcessBase {
public:
    SemanticSegPostProcessBase() = default;

    SemanticSegPostProcessBase(const SemanticSegPostProcessBase &other) = default;

    virtual ~SemanticSegPostProcessBase() = default;

    SemanticSegPostProcessBase& operator=(const SemanticSegPostProcessBase &other);

    APP_ERROR Init(const std::map<std::string, std::string> &postConfig) override;

    APP_ERROR DeInit() override;

    virtual APP_ERROR Process(const std::vector<TensorBase>& tensors, std::vector<SemanticSegInfo>& semanticSegInfos,
                              const std::vector<ResizedImageInfo>& resizedImageInfos = {},
                              const std::map<std::string, std::shared_ptr<void>> &configParamMap = {});

protected:
    void CoordinatesReduction(const ResizedImageInfo& resizedImageInfo,
                              SemanticSegInfo& semanticSegInfos);
                        
    APP_ERROR GetSemanticSegConfigData();

protected:
    uint32_t classNum_ = 0;
    int modelType_ = TYPE_NHWC;
    std::vector<std::string> labelMap_;
};

using GetSemanticSegInstanceFunc = std::shared_ptr<SemanticSegPostProcessBase>(*)();
}


#endif