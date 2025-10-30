/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
 * Description: Post-processing of image tasks.
 * Author: MindX SDK
 * Create: 2020
 * History: NA
 */

#ifndef IMAGE_POST_PROCESS_H
#define IMAGE_POST_PROCESS_H
#include "MxBase/PostProcessBases/PostProcessBase.h"

namespace MxBase {
class ImagePostProcessBase : public PostProcessBase {
public:
    void SetCropRoiBoxes(std::vector<MxBase::CropRoiBox> cropRoiBoxes)
    {
        cropRoiBoxes_ = cropRoiBoxes;
    }

protected:
    std::vector<MxBase::CropRoiBox> cropRoiBoxes_ = {};
};
}

#endif