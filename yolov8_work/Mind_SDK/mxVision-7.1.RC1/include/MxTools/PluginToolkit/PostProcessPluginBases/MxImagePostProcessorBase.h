/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
 * Description: Interface of the base class of the image post-processing plug-in.
 * Author: MindX SDK
 * Create: 2020
 * History: NA
 */

#ifndef MXIMAGEPOSTPROCESSOR_PLUGINBASE
#define MXIMAGEPOSTPROCESSOR_PLUGINBASE

#include "MxTools/PluginToolkit/PostProcessPluginBases/MxModelPostProcessorBase.h"
#include "MxBase/PostProcessBases/PostProcessDataType.h"

namespace MxTools {
class MxImagePostProcessorBase : public MxTools::MxModelPostProcessorBase {
public:

    /**
    * @description: Init configs, get postProcess instance.
    * @param configParamMap: config.
    * @return: Error code.
    */
    APP_ERROR Init(std::map<std::string, std::shared_ptr<void>> &configParamMap) override;

    /**
    * @description: MxImagePostProcessorBase plugin process.
    * @param mxpiBuffer: data receive from the previous.
    * @return: Error code.
    */
    APP_ERROR Process(std::vector<MxTools::MxpiBuffer *> &mxpiBuffer) override;

    /**
    * @description: DeInit postProcess instance.
    * @return: Error code.
    */
    APP_ERROR DeInit() override;

    /**
    * @description: MxImagePostProcessorBase plugin define properties.
    * @return: properties.
    */
    static std::vector<std::shared_ptr<void>> DefineProperties();

    /**
    * @api
    * @brief Define the number and data type of input ports.
    * @return MxTools::MxpiPortInfo.
    */
    static MxpiPortInfo DefineInputPorts();

protected:
    std::string dataSourceRoiBoxes_ = ""; // key of crop datasource.
    std::string dataSourceResize_ = ""; // key of resize datasource.
    std::string dataSourceImage_ = "";
    std::vector<MxBase::ResizedImageInfo> resizedImageInfos_ = {};
    std::vector<MxBase::ImagePreProcessInfo> imagePreProcessInfos_ = {};

private:
    APP_ERROR ConstructPostImageInfo(std::vector<MxBase::ResizedImageInfo> &resizedImageInfos,
                                     std::vector<MxBase::CropRoiBox> &cropRoiBoxes,
                                     std::vector<MxTools::MxpiBuffer *> &mxpiBuffer,
                                     std::shared_ptr<MxTools::MxpiVisionList> mxpiVisionList);

    APP_ERROR ConstructImagePreProcessInfo(std::vector<MxTools::MxpiBuffer *> &mxpiBuffer);

    APP_ERROR ConstructWithDataSources(std::vector<MxTools::MxpiBuffer *> &mxpiBuffer,
                                       std::shared_ptr<MxTools::MxpiTensorPackageList> tensorPackageList);
};
}
#endif // MXIMAGEPOSTPROCESSOR_PLUGINBASE