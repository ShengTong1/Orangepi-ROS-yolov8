/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
 * Description: UNetMindSpore model post-processing.
 * Author: MindX SDK
 * Create: 2021
 * History: NA
 */

#ifndef UNETMINDSPORE_POST_PROCESS_H
#define UNETMINDSPORE_POST_PROCESS_H
#include "MxBase/PostProcessBases/SemanticSegPostProcessBase.h"
#include "MxBase/Common/HiddenAttr.h"

namespace MxBase {
class UNetMindSporePostProcessDptr;
class SDK_AVAILABLE_FOR_OUT UNetMindSporePostProcess : public SemanticSegPostProcessBase {
public:
    UNetMindSporePostProcess();

    ~UNetMindSporePostProcess() = default;

    UNetMindSporePostProcess(const UNetMindSporePostProcess &other);

    APP_ERROR Init(const std::map<std::string, std::string> &postConfig) override;

    APP_ERROR DeInit() override;

    APP_ERROR Process(const std::vector<TensorBase> &tensors, std::vector<SemanticSegInfo> &semanticSegInfos,
                      const std::vector<ResizedImageInfo> &resizedImageInfos = {},
                      const std::map<std::string, std::shared_ptr<void>> &configParamMap = {});

    UNetMindSporePostProcess &operator=(const UNetMindSporePostProcess &other);

    uint64_t GetCurrentVersion() override;

private:
    friend class UNetMindSporePostProcessDptr;
    std::shared_ptr<MxBase::UNetMindSporePostProcessDptr> dPtr_;
    APP_ERROR CheckDptr();
};
#ifdef ENABLE_POST_PROCESS_INSTANCE
extern "C" {
std::shared_ptr<MxBase::UNetMindSporePostProcess> GetSemanticSegInstance();
}
#endif
}
#endif