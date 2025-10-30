/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
 * Description: Ctpn model post-processing.
 * Author: MindX SDK
 * Create: 2020
 * History: NA
 */

#ifndef CTPNPOSTPROCESSOR_H
#define CTPNPOSTPROCESSOR_H

#include <unordered_set>
#include <vector>
#include <queue>
#include "MxBase/PostProcessBases/TextObjectPostProcessBase.h"
#include "MxBase/ErrorCode/ErrorCode.h"
#include "MxBase/Maths/NpySort.h"
#include "MxBase/CV/ObjectDetection/Nms/Nms.h"
#include "MxBase/Common/HiddenAttr.h"

namespace {
const bool IS_ORIENTED = false;
const int MIN_SIZE = 8;
const int FEAT_STRIDE = 16;
const int ANCHOR_SCALES = 16;
const int FEAT_BOX_LAYER = 4;
const int FEAT_CONF_LAYER = 2;
const int ANCHORNUM = 10;
const int MAX_HORIZONTAL_GAP = 60;
const float MIN_OVER_LAPS = 0.7;
const float MIN_SIZE_SIM = 0.7;
const float BOX_IOU_THRESH = 0.7;
const float TEXT_IOU_THRESH = 0.2;
const float MIN_RATIO = 0.5;
const float TEXT_PROPOSALS_MIN_SCORE = 0.7;
const float LINE_MIN_SCORE = 0.9;
const int TEXT_PROPOSALS_WIDTH = 16;
const int MIN_NUM_PROPOSALS = 2;
const int RPN_PRE_NMS_TOP_N = 12000;
const int RPN_POST_NMS_TOP_N = 1000;
const float ZERO_POINT_FIVE = 0.5;
const float HEIGHT_OFFSIZE = 2.5;
const bool IS_MINDSPORE = true;
const bool ORDER_BY_PY = true;
}

namespace {
struct CoorDim {
    int i;
    int j;
    int k;
    int index;
    CoorDim(int i, int j, int k, int idx) : i(i), j(j), k(k), index(idx) {}
};
}

namespace MxBase {
class CtpnPostProcessDptr;
class SDK_AVAILABLE_FOR_OUT CtpnPostProcess : public TextObjectPostProcessBase {
public:
    CtpnPostProcess();

    virtual ~CtpnPostProcess();

    CtpnPostProcess(const CtpnPostProcess &other);

    CtpnPostProcess &operator = (const CtpnPostProcess &other);
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

    uint64_t GetCurrentVersion() override;

private:
    friend class CtpnPostProcessDptr;
    APP_ERROR CheckDptr();
    std::shared_ptr<MxBase::CtpnPostProcessDptr> dPtr_;
};
#ifdef ENABLE_POST_PROCESS_INSTANCE
extern "C" {
std::shared_ptr<MxBase::CtpnPostProcess> GetTextObjectInstance();
}
#endif
}


#endif // CTPNPOSTPROCESSOR_H
