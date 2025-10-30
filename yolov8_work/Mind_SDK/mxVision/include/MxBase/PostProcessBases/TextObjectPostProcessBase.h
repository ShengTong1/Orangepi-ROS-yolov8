/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
 * Description: Accepts the model inference output tensor and outputs the target detection result.
 * Author: MindX SDK
 * Create: 2020
 * History: NA
 */

#ifndef TEXTDETECTPOSTPROCESSBASE_H
#define TEXTDETECTPOSTPROCESSBASE_H

#include "MxBase/PostProcessBases/ImagePostProcessBase.h"

namespace MxBase {
class TextObjectPostProcessBase : public ImagePostProcessBase {
public:
    TextObjectPostProcessBase() = default;

    TextObjectPostProcessBase(const TextObjectPostProcessBase &other) = default;

    virtual ~TextObjectPostProcessBase() = default;

    TextObjectPostProcessBase &operator = (const TextObjectPostProcessBase &other);

    APP_ERROR Init(const std::map<std::string, std::string> &postConfig) override;

    APP_ERROR DeInit() override;

    virtual APP_ERROR Process(const std::vector<TensorBase> &tensors,
        std::vector<std::vector<TextObjectInfo>> &textObjectInfos,
        const std::vector<ResizedImageInfo> &resizedImageInfos = {},
        const std::map<std::string, std::shared_ptr<void>> &configParamMap = {});

protected:
    void ResizeReduction(const ResizedImageInfo &resizedImageInfo, TextObjectInfo &textObjInfo);

    void FixCoords(uint32_t scrData, float &desData);

protected:
    bool checkModelFlag_ = true;
};
using GetTextObjectInstanceFunc = std::shared_ptr<TextObjectPostProcessBase>(*)();
}

#endif // TEXTDETECTPOSTPROCESSBASE_H
