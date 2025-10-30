/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
 * Description: Converts the data structure used by model post-processing to the protobuf data structure.
 * Author: MindX SDK
 * Create: 2020
 * History: NA
 */

#ifndef MXPI_DATATYPECONVERTERV2_H
#define MXPI_DATATYPECONVERTERV2_H

#include "MxBase/Tensor/TensorBase/TensorBase.h"
#include "MxBase/DvppWrapper/DvppWrapper.h"
#include "MxBase/PostProcessBases/PostProcessDataType.h"
#include "MxTools/Proto/MxpiDataType.pb.h"

namespace MxTools {
std::shared_ptr<MxpiObjectList> ConstructProtobuf(
    const std::vector<std::vector<MxBase::ObjectInfo>> &objectInfos, std::string dataSource);

std::shared_ptr<MxpiClassList> ConstructProtobuf(const std::vector<std::vector<MxBase::ClassInfo>> &classInfos,
                                                 std::string dataSource);

std::shared_ptr<MxpiImageMaskList> ConstructProtobuf(
    const std::vector<MxBase::SemanticSegInfo> &semanticSegInfos, std::string dataSource);

std::shared_ptr<MxpiTextsInfoList> ConstructProtobuf(const std::vector<MxBase::TextsInfo> &textsInfo,
                                                     std::string dataSource);

std::shared_ptr<MxpiTextObjectList> ConstructProtobuf(
    const std::vector<std::vector<MxBase::TextObjectInfo>> &textObjectInfos, std::string dataSource);

std::shared_ptr<MxpiPoseList> ConstructProtobuf(
    const std::vector<std::vector<MxBase::KeyPointDetectionInfo>>& keyPointInfos, std::string dataSource);

void StackMxpiVisionPreProcess(MxpiVisionInfo &dstMxpiVisionInfo,
                               const MxpiVisionInfo &srcMxpiVisionInfo,
                               const MxBase::CropResizePasteConfig &cropResizePasteConfig,
                               const std::string& elementName = "");

APP_ERROR ConstrutImagePreProcessInfo(MxTools::MxpiVisionInfo& visionInfo, MxBase::ImagePreProcessInfo& positionInfo);
}
#endif // MXPI_DATATYPECONVERTERV2_H
