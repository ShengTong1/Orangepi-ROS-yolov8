/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
 * Description: Classification detection post-processing.
 * Author: MindX SDK
 * Create: 2020
 * History: NA
 */

#ifndef CLASS_POST_PROCESS_H
#define CLASS_POST_PROCESS_H
#include "MxBase/PostProcessBases/PostProcessBase.h"

namespace MxBase {
class ClassPostProcessBase : public PostProcessBase {
public:
    ClassPostProcessBase() = default;

    ClassPostProcessBase(const ClassPostProcessBase &other) = default;

    virtual ~ClassPostProcessBase() = default;

    ClassPostProcessBase& operator=(const ClassPostProcessBase &other);

    APP_ERROR Init(const std::map<std::string, std::string> &postConfig) override;

    APP_ERROR DeInit() override;
    
    APP_ERROR virtual Process(const std::vector<TensorBase>& tensors, std::vector<std::vector<ClassInfo>>& classInfos,
                              const std::map<std::string, std::shared_ptr<void>> &configParamMap = {});

protected:
    uint32_t classNum_ = 0;
    uint32_t topK_ = 1;
};

using GetClassInstanceFunc = std::shared_ptr<ClassPostProcessBase>(*)();
}


#endif