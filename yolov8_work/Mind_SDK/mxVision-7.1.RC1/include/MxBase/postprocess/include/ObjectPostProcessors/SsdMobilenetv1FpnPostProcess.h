/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
 * Description: SsdMobilenetv1Fpn model post-processing.
 * Author: MindX SDK
 * Create: 2020
 * History: NA
 */

#ifndef SSDMOBILENETV1FPN_POST_PROCESS_H
#define SSDMOBILENETV1FPN_POST_PROCESS_H
#include "MxBase/PostProcessBases/ObjectPostProcessBase.h"
#include "MxBase/Common/HiddenAttr.h"

namespace MxBase {
class SsdMobilenetv1FpnPostProcessDptr;
class SDK_AVAILABLE_FOR_OUT SsdMobilenetv1FpnPostProcess : public ObjectPostProcessBase {
public:
    SsdMobilenetv1FpnPostProcess();

    ~SsdMobilenetv1FpnPostProcess() = default;

    SsdMobilenetv1FpnPostProcess(const SsdMobilenetv1FpnPostProcess &other);

    SsdMobilenetv1FpnPostProcess &operator=(const SsdMobilenetv1FpnPostProcess &other);

    APP_ERROR Init(const std::map<std::string, std::string> &postConfig) override;

    APP_ERROR DeInit() override;

    APP_ERROR Process(const std::vector<TensorBase> &tensors, std::vector<std::vector<ObjectInfo>> &objectInfos,
                      const std::vector<ResizedImageInfo> &resizedImageInfos = {},
                      const std::map<std::string, std::shared_ptr<void>> &configParamMap = {}) override;

    uint64_t GetCurrentVersion() override;

private:
    friend class SsdMobilenetv1FpnPostProcessDptr;
    std::shared_ptr<MxBase::SsdMobilenetv1FpnPostProcessDptr> dPtr_;
    APP_ERROR CheckDptr();
};

#ifdef ENABLE_POST_PROCESS_INSTANCE
extern "C" {
std::shared_ptr<MxBase::SsdMobilenetv1FpnPostProcess> GetObjectInstance();
}
#endif
}
#endif