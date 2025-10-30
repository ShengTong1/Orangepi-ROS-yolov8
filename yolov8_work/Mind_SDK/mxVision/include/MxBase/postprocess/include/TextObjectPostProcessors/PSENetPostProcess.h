/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
 * Description: PSENet model post-processing.
 * Author: MindX SDK
 * Create: 2020
 * History: NA
 */

#ifndef PSENetPostProcess_H
#define PSENetPostProcess_H

#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/core/types.hpp>
#include <queue>
#include "MxBase/PostProcessBases/TextObjectPostProcessBase.h"

namespace {
const float MIN_KERNEL_AREA = 5.0;
const float PSE_SCALE = 1.0;
const float MIN_SCORE = 0.9;
const float MIN_AREA = 600;
const int KERNEL_NUM = 7;
}

namespace {
const int DEFAULT_OBJECT_NUM_TENSOR = 0;
}

namespace MxBase {
class PSENetPostProcessDptr;
class PSENetPostProcess : public TextObjectPostProcessBase {
public:
    PSENetPostProcess &operator = (const PSENetPostProcess &other);

    PSENetPostProcess();

    virtual ~PSENetPostProcess();

    /*
     * @description Load the configs and labels from the file.
     * @param labelPath config path and label path.
     * @return APP_ERROR error code.
     */

    APP_ERROR Init(const std::map<std::string, std::string> &postConfig) override;

    /*
     * @description: Do nothing temporarily.
     * @return APP_ERROR error code.
     */
    APP_ERROR DeInit() override;

    /*
     * @description: Get the info of detected object from output and resize to original coordinates.
     * @param featLayerData  Vector of output feature data.
     * @param objInfos  Address of output object infos.
     * @param useMpPictureCrop  if true, offsets of coordinates will be given.
     * @param postImageInfo  Info of model/image width and height, offsets of coordinates.
     * @return: ErrorCode.
     */
    APP_ERROR Process(const std::vector<TensorBase> &tensors, std::vector<std::vector<TextObjectInfo>> &textObjInfos,
        const std::vector<ResizedImageInfo> &resizedImageInfos = {},
        const std::map<std::string, std::shared_ptr<void>> &configParamMap = {});

    uint64_t GetCurrentVersion() override
    {
        return MINDX_SDK_VERSION;
    }

private:
    friend class PSENetPostProcessDptr;
    std::shared_ptr<MxBase::PSENetPostProcessDptr> dPtr_;
    APP_ERROR CheckDptr();
};
#ifdef ENABLE_POST_PROCESS_INSTANCE
extern "C" {
std::shared_ptr<MxBase::PSENetPostProcess> GetTextObjectInstance();
}
#endif
}

#endif // MXVISION_PSENetPostProcess_H
