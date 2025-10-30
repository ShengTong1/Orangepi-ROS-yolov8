/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
 * Description: Accepts the model inference output tensor and outputs the text generation result.
 * Author: MindX SDK
 * Create: 2020
 * History: NA
 */

#ifndef TEXTGENERATION_POST_PROCESS_H
#define TEXTGENERATION_POST_PROCESS_H
#include "MxBase/PostProcessBases/PostProcessBase.h"

namespace MxBase {
class TextGenerationPostProcessBase : public PostProcessBase {
public:
    TextGenerationPostProcessBase() = default;

    TextGenerationPostProcessBase(const TextGenerationPostProcessBase &other) = default;

    virtual ~TextGenerationPostProcessBase() = default;

    TextGenerationPostProcessBase& operator=(const TextGenerationPostProcessBase &other);

    APP_ERROR Init(const std::map<std::string, std::string> &postConfig) override;

    APP_ERROR DeInit() override;

    virtual APP_ERROR Process(const std::vector<TensorBase>& tensors,
                              std::vector<TextsInfo>& textsInfos,
                              const std::map<std::string, std::shared_ptr<void>> &configParamMap = {});

protected:
    uint32_t classNum_ = 0;
};

using GetTextGenerationInstanceFunc = std::shared_ptr<TextGenerationPostProcessBase>(*)();
}


#endif