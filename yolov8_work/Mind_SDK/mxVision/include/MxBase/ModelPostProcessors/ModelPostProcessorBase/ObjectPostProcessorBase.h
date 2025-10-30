/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
 * Description: Inference post-processing of target detection.
 * Author: MindX SDK
 * Create: 2020
 * History: NA
 */

#ifndef OBJECT_POSTPROCESSOR_H
#define OBJECT_POSTPROCESSOR_H

#include "MxBase/Maths/FastMath.h"
#include "MxBase/CV/ObjectDetection/Nms/Nms.h"
#include "ModelPostProcessorBase.h"
#include "ObjectPostDataType.h"

namespace {
const int DEFAULT_CLASS_NUM = 2;
const float DEFAULT_SCORE_THRESH = 0.5;
}

namespace MxBase {
class SDK_DEPRECATED_FOR() ObjectPostProcessorBase : public MxBase::ModelPostProcessorBase {
public:

    /*
     * @description Load the configs and labels from the file.
     * @param labelPath config path and label path.
     * @return APP_ERROR error code.
     */
    APP_ERROR Init(const std::string& configPath, const std::string& labelPath, MxBase::ModelDesc modelDesc) override;

    /*
     * @description Do nothing temporarily.
     * @return APP_ERROR error code.
     */
    APP_ERROR DeInit() override;

    /*
     * @description Get the info of detected object from output and resize to original coordinates.
     * @param featLayerData  Vector of output feature data.
     * @param objInfos  Address of output object infos.
     * @param useMpPictureCrop  if true, offsets of coordinates will be given.
     * @param postImageInfo  Info of model/image width and height, offsets of coordinates.
     * @return: ErrorCode.
     */
    virtual APP_ERROR Process(std::vector<std::shared_ptr<void>>& featLayerData, std::vector<ObjDetectInfo>& objInfos,
        const bool useMpPictureCrop, MxBase::PostImageInfo postImageInfo);

    /*
    * @description: set post processor image information.
    */
    void SetAspectRatioImageInfo(const MxBase::AspectRatioPostImageInfo& postProcessorImageInfo);

protected:
    virtual void ObjectDetectionOutput(std::vector<std::shared_ptr<void>>& featLayerData,
                                       std::vector<ObjDetectInfo>& objInfos, ImageInfo& imgInfo);
    void CoordinatesReduction(ImageInfo& imgInfo, std::vector<ObjDetectInfo>& objInfos);
    APP_ERROR GetSeparateScoreThresh(std::string& strSeparateScoreThresh);

    void NmsSort(std::vector<ObjDetectInfo>& objInfos, float iouThresh, IOUMethod method = UNION);

protected:
    MxBase::AspectRatioPostImageInfo postProcessorImageInfo_;
    std::vector<float> separateScoreThresh_ = {};
    float scoreThresh_ = DEFAULT_SCORE_THRESH;
    int classNum_ = DEFAULT_CLASS_NUM;
};
}
#endif  // OBJECT_POSTPROCESSOR_H
