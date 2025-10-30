/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
 * Description: Filter duplicate targets based on area size.
 * Author: MindX SDK
 * Create: 2020
 * History: NA
 */

#ifndef NMS_H
#define NMS_H

#include <algorithm>
#include <vector>
#include <map>
#include "MxBase/ErrorCode/ErrorCode.h"
#include "MxBase/CV/Core/DataType.h"

namespace MxBase {
float CalcIou(DetectBox a, DetectBox b, IOUMethod method = UNION);

void FilterByIou(std::vector<DetectBox> dets,
                 std::vector<DetectBox>& sortBoxes, float iouThresh, IOUMethod method = UNION);

void NmsSort(std::vector<DetectBox>& detBoxes, float iouThresh, IOUMethod method = UNION);
void NmsSortByArea(std::vector<DetectBox>& detBoxes, const float iouThresh, const IOUMethod method = UNION);

class ObjectInfo;
float CalcIou(ObjectInfo a, ObjectInfo b, IOUMethod method = UNION);

void FilterByIou(std::vector<ObjectInfo> dets,
                 std::vector<ObjectInfo>& sortBoxes, float iouThresh, IOUMethod method = UNION);

void NmsSort(std::vector<ObjectInfo>& detBoxes, float iouThresh, IOUMethod method = UNION);
void NmsSortByArea(std::vector<ObjectInfo>& detBoxes, const float iouThresh, const IOUMethod method = UNION);
}  // namespace MxBase
#endif