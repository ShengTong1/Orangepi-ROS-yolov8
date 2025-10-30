/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
 * Description: Transformer model post-processing.
 * Author: MindX SDK
 * Create: 2021
 * History: NA
 */

#ifndef TRANSFORMER_POST_PROCESS_H
#define TRANSFORMER_POST_PROCESS_H
#include "MxBase/PostProcessBases/TextGenerationPostProcessBase.h"
#include "MxBase/Common/HiddenAttr.h"

namespace MxBase {
class TransformerPostProcessDptr;
class SDK_AVAILABLE_FOR_OUT TransformerPostProcess : public TextGenerationPostProcessBase {
public:
    TransformerPostProcess();

    ~TransformerPostProcess() = default;

    TransformerPostProcess(const TransformerPostProcess &other);

    APP_ERROR Init(const std::map<std::string, std::string> &postConfig) override;

    APP_ERROR DeInit() override;

    APP_ERROR Process(const std::vector<TensorBase>& tensors, std::vector<TextsInfo>& translationInfos,
                      const std::map<std::string, std::shared_ptr<void>> &configParamMap = {});

    TransformerPostProcess &operator=(const TransformerPostProcess &other);

    uint64_t GetCurrentVersion() override;

private:
    friend class TransformerPostProcessDptr;
    std::shared_ptr<MxBase::TransformerPostProcessDptr> dPtr_;
    APP_ERROR CheckDptr();
};
#ifdef ENABLE_POST_PROCESS_INSTANCE
extern "C" {
    std::shared_ptr<MxBase::TransformerPostProcess> GetTextGenerationInstance();
}
#endif
}
#endif