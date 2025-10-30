/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
 * Description: FasterRcnn model post-processing.
 * Author: MindX SDK
 * Create: 2020
 * History: NA
 */

#ifndef FASTER_RCNN_POST_PROCESS_H
#define FASTER_RCNN_POST_PROCESS_H
#include "MxBase/PostProcessBases/ObjectPostProcessBase.h"
#include "MxBase/Common/HiddenAttr.h"

namespace MxBase {
class FasterRcnnPostProcessDptr;
class SDK_AVAILABLE_FOR_OUT FasterRcnnPostProcess : public ObjectPostProcessBase {
public:
    FasterRcnnPostProcess();

    ~FasterRcnnPostProcess() = default;

    FasterRcnnPostProcess(const FasterRcnnPostProcess &other);

    FasterRcnnPostProcess &operator=(const FasterRcnnPostProcess &other);

    APP_ERROR Init(const std::map<std::string, std::string> &postConfig) override;

    APP_ERROR DeInit() override;

    APP_ERROR Process(const std::vector<TensorBase> &tensors, std::vector<std::vector<ObjectInfo>> &objectInfos,
                      const std::vector<ResizedImageInfo> &resizedImageInfos = {},
                      const std::map<std::string, std::shared_ptr<void>> &configParamMap = {}) override;
    uint64_t GetCurrentVersion() override;

private:
    friend class FasterRcnnPostProcessDptr;
    std::shared_ptr<MxBase::FasterRcnnPostProcessDptr> dPtr_;
    APP_ERROR CheckDptr();
};

#ifdef ENABLE_POST_PROCESS_INSTANCE
extern "C" {
std::shared_ptr<MxBase::FasterRcnnPostProcess> GetObjectInstance();
}
#endif
}
#endif