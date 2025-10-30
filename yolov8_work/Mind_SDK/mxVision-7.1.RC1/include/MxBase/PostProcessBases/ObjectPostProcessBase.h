/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
 * Description: Target detection post-processing.
 * Author: MindX SDK
 * Create: 2020
 * History: NA
 */

#ifndef OBJECT_POST_PROCESS_H
#define OBJECT_POST_PROCESS_H
#include "MxBase/PostProcessBases/ImagePostProcessBase.h"

namespace MxBase {
class ObjectPostProcessBase : public ImagePostProcessBase {
public:
    ObjectPostProcessBase() = default;

    ObjectPostProcessBase(const ObjectPostProcessBase &other) = default;

    virtual ~ObjectPostProcessBase() = default;

    ObjectPostProcessBase& operator=(const ObjectPostProcessBase &other);

    APP_ERROR Init(const std::map<std::string, std::string> &postConfig) override;

    APP_ERROR DeInit() override;

    virtual APP_ERROR Process(const std::vector<TensorBase> &tensors,
                              std::vector<std::vector<ObjectInfo>> &objectInfos,
                              const std::vector<ResizedImageInfo> &resizedImageInfos = {},
                              const std::map<std::string, std::shared_ptr<void>> &configParamMap = {});

protected:
    void CoordinatesReduction(const uint32_t index, const ResizedImageInfo& resizedImageInfo,
                              std::vector<ObjectInfo>& objInfos, bool normalizedFlag = true);

    void ResizeReduction(const ResizedImageInfo& resizedImageInfo, const int imgWidth, const int imgHeight,
                         std::vector<ObjectInfo>& objInfos);

    void CoordinatesReduction(std::vector<std::vector<ObjectInfo>> &objInfos,
        const std::vector<MxBase::ImagePreProcessInfo> &imagePreProcessInfos,
        bool normalizedFlag = true);

    void LogObjectInfos(const std::vector<std::vector<ObjectInfo>>& objectInfos);

    APP_ERROR GetSeparateScoreThresh(std::string& strSeparateScoreThresh);

    APP_ERROR GetObjectConfigData();

protected:
    std::vector<float> separateScoreThresh_ = {};
    float scoreThresh_ = 0.0;
    uint32_t classNum_ = 0;
};

using GetObjectInstanceFunc = std::shared_ptr<ObjectPostProcessBase>(*)();
}


#endif