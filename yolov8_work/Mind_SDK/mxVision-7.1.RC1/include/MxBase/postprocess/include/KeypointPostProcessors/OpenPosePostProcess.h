/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
 * Description: OpenPose model post-processing.
 * Author: MindX SDK
 * Create: 2021
 * History: NA
 */

#ifndef OPENPOSE_POST_PROCESS_H
#define OPENPOSE_POST_PROCESS_H

#include "MxBase/Common/HiddenAttr.h"
#include "MxBase/PostProcessBases/KeypointPostProcessBase.h"

namespace MxBase {

class OpenPosePostProcessDptr;
class SDK_AVAILABLE_FOR_OUT OpenPosePostProcess : public KeypointPostProcessBase {
public:
    OpenPosePostProcess();

    ~OpenPosePostProcess() = default;

    OpenPosePostProcess(const OpenPosePostProcess &other);

    OpenPosePostProcess &operator=(const OpenPosePostProcess &other);

    APP_ERROR Init(const std::map<std::string, std::string> &postConfig) override;

    APP_ERROR DeInit() override;

    APP_ERROR Process(const std::vector<TensorBase>& tensors,
                      std::vector<std::vector<KeyPointDetectionInfo>>& keyPointInfos,
                      const std::vector<ResizedImageInfo>& resizedImageInfos,
                      const std::map<std::string, std::shared_ptr<void>> &paramMap = {}) override;

    uint64_t GetCurrentVersion() override;

private:
    friend class OpenPosePostProcessDptr;
    APP_ERROR CheckDptr();
    std::shared_ptr<MxBase::OpenPosePostProcessDptr> dPtr_;
};

#ifdef ENABLE_POST_PROCESS_INSTANCE
extern "C" {
std::shared_ptr<MxBase::OpenPosePostProcess> GetKeypointInstance();
}
#endif
}
#endif