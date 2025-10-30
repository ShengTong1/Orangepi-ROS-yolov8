/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
 * Description: Processing of the Model Inference Function.
 * Author: MindX SDK
 * Create: 2020
 * History: NA
 */

#ifndef MODLE_INFERENCEPROCESSOR_H
#define MODLE_INFERENCEPROCESSOR_H

#include "ModelDataType.h"
#include "MxBase/ErrorCode/ErrorCode.h"
#include "MxBase/Tensor/TensorBase/TensorBase.h"
#include "MxBase/Common/HiddenAttr.h"

namespace MxBase {
struct AspectRatioPostImageInfo : PostImageInfo {
    enum ResizeType {
        RESIZER_STRETCHING = 0,
        RESIZER_TF_KEEP_ASPECT_RATIO,
        RESIZER_MS_KEEP_ASPECT_RATIO,
        RESIZER_ONLY_PADDING,
        RESIZER_KEEP_ASPECT_RATIO_LONG,
        RESIZER_KEEP_ASPECT_RATIO_SHORT,
        RESIZER_RESCALE,
        RESIZER_RESCALE_DOUBLE,
        RESIZER_MS_YOLOV4,
    };
    float keepAspectRatioScaling = 0;
    ResizeType resizeType;
};

struct PostProcessorImageInfo {
    std::vector<MxBase::PostImageInfo> postImageInfoVec;
    bool useMpPictureCrop = false;
};

class ModelInferenceProcessorDptr;

class SDK_AVAILABLE_FOR_OUT ModelInferenceProcessor {
public:
    ModelInferenceProcessor();

    ~ModelInferenceProcessor() = default;

    /**
     * @description Init
     * 1.Loading  Model
     * 2.Get input sizes and output sizes
     * @return APP_ERROR error code
     */
    APP_ERROR Init(std::string modelPath, ModelDesc& modelDesc);

    /**
     * @description Init
     * 1.Loading  Model
     * 2.Get input sizes and output sizes
     * @return APP_ERROR error code
     */
    APP_ERROR Init(const std::string& modelPath);

    /**
     * @description Unload Model
     * @return APP_ERROR error code
     */
    APP_ERROR DeInit(void);

    /**
     * @description ModelInference
     * 1.Get model description
     * 2.Execute model infer
     * @return APP_ERROR error code
     */
    APP_ERROR ModelInference(
        std::vector<BaseTensor>& inputTensors, std::vector<BaseTensor>& outputTensors, size_t dynamicBatchSize = 0);

    /**
     * @description ModelInference
     * 1.Get model description
     * 2.Execute model infer
     * @return APP_ERROR error code
     */
    APP_ERROR ModelInference(
        std::vector<BaseTensor>& inputTensors, std::vector<BaseTensor>& outputTensors, DynamicInfo dynamicInfo);

    /**
     * @description ModelInference
     * 1.Get model description
     * 2.Execute model infer
     * @return APP_ERROR error code
     */
    APP_ERROR ModelInference(
       const std::vector<TensorBase>& inputTensors, std::vector<TensorBase>& outputTensors, DynamicInfo dynamicInfo);

    /**
     * @description ModelInference
     */
    const std::vector<ImageSize>& GetDynamicImageSizes() const;

    /**
     * @description get model dynamic type
     */
    DynamicType GetDynamicType() const;

    /**
     * @description get modelDesc
     */
    const ModelDesc& GetModelDesc() const;

    /**
     * @description get data format
     */
    DataFormat GetDataFormat() const;

    /**
     * @description get input data type
     */
    std::vector<TensorDataType> GetInputDataType() const;

    /**
     * @description get input data type
     */
    std::vector<TensorDataType> GetOutputDataType() const;

    /**
     * @description get input tensor shape
     */
    std::vector<std::vector<int64_t>> GetInputShape() const;

    /**
     * @description get output tensor shape
     */
    std::vector<std::vector<int64_t>> GetOutputShape() const;

    /**
     * @description get output tensor shape
     */
    std::vector<int64_t> GetDynamicBatch() const;

    /**
     * @description get input tensor fomat
     */
    std::vector<size_t> GetInputFormat() const;

    /**
     * @description get output tensor fomat
     */
    std::vector<size_t> GetOutputFormat() const;

    /**
     * @description: get dynamic gear info of dynamic model
     */
    std::vector<std::vector<uint64_t>> GetDynamicGearInfo();

private:
    std::shared_ptr<MxBase::ModelInferenceProcessorDptr> dPtr_;
};
}  // namespace MxBase
#endif
