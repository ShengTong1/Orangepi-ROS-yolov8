/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
 * Description: Keypoint detection post-processing.
 * Author: MindX SDK
 * Create: 2021
 * History: NA
 */

#ifndef KEYPOINT_POST_PROCESS_H
#define KEYPOINT_POST_PROCESS_H
#include "MxBase/PostProcessBases/ImagePostProcessBase.h"

namespace MxBase {
class KeypointPostProcessBase : public ImagePostProcessBase {
public:
    KeypointPostProcessBase() = default;

    KeypointPostProcessBase(const KeypointPostProcessBase &other) = default;

    virtual ~KeypointPostProcessBase() = default;

    KeypointPostProcessBase& operator=(const KeypointPostProcessBase &other);

    APP_ERROR Init(const std::map<std::string, std::string> &postConfig) override;

    APP_ERROR DeInit() override;

    virtual APP_ERROR Process(const std::vector<TensorBase>& tensors,
                            std::vector<std::vector<KeyPointDetectionInfo>>& keyPointInfos,
                            const std::vector<ResizedImageInfo>& resizedImageInfos = {},
                            const std::map<std::string, std::shared_ptr<void>> &configParamMap = {});

protected:
    void LogKeyPointInfos(const std::vector<std::vector<KeyPointDetectionInfo>>& keyPointInfos);

    APP_ERROR GetSeparateScoreThresh(std::string& strSeparateScoreThresh);

protected:
    std::vector<float> separateScoreThresh_ = {};
    float scoreThresh_ = 0.0;
    uint32_t classNum_ = 0;
};

using GetKeypointInstanceFunc = std::shared_ptr<KeypointPostProcessBase>(*)();
}


#endif