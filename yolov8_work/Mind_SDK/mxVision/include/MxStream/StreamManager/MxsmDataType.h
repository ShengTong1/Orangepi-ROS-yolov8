/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
 * Description: Data, buffer and metadata input and output structures.
 * Author: MindX SDK
 * Create: 2020
 * History: NA
 */

#ifndef MXSM_DATA_TYPE_H
#define MXSM_DATA_TYPE_H

#include <iostream>
#include <vector>
#include <google/protobuf/message.h>
#include "MxBase/ErrorCode/ErrorCode.h"
#include "MxTools/Proto/MxpiDataType.pb.h"

namespace MxStream {
struct MxstDataOutput {
    APP_ERROR errorCode = APP_ERR_OK;
    int dataSize = 0;
    uint32_t *dataPtr = nullptr;

    MxstDataOutput() = default;

    MxstDataOutput(const MxstDataOutput&) = delete;

    MxstDataOutput& operator=(const MxstDataOutput&) = delete;

    ~MxstDataOutput()
    {
        if (dataPtr != nullptr) {
            free(dataPtr);
            dataPtr = nullptr;
        }
    }
};

struct MxstBufferOutput {
    APP_ERROR errorCode = APP_ERR_OK;
    std::string errorMsg;
    int dataSize = 0;
    uint32_t *dataPtr = nullptr;
    void *reservedPtr = nullptr;
    MxTools::MxpiFrameInfo mxpiFrameInfo;

    MxstBufferOutput() = default;

    explicit MxstBufferOutput(APP_ERROR errorCode, const std::string& errorMsg = "")
        : errorCode(errorCode), errorMsg(std::move(errorMsg)) {}

    MxstBufferOutput(const MxstBufferOutput&) = delete;

    MxstBufferOutput& operator=(const MxstBufferOutput&) = delete;

    void SetErrorInfo(APP_ERROR errorCodeIn, const std::string& errorMsgIn)
    {
        errorCode = errorCodeIn;
        errorMsg = errorMsgIn;
    }

    ~MxstBufferOutput()
    {
        if (dataPtr != nullptr) {
            free(dataPtr);
            dataPtr = nullptr;
        }
    }
};

struct CropRoiBox {
    float x0;
    float y0;
    float x1;
    float y1;
};

struct MxstFrameExternalInfo {
    uint64_t uniqueId;
    int fragmentId;
    std::string customParam;
};

struct MxstServiceInfo {
    int fragmentId;
    std::string customParam;
    std::vector<CropRoiBox> roiBoxs;
};
struct MxstDataInput {
    MxstServiceInfo serviceInfo;
    int dataSize = 0;
    uint32_t *dataPtr = nullptr;
};

struct MxstBufferInput {
    MxTools::MxpiFrameInfo mxpiFrameInfo;
    MxTools::MxpiVisionInfo mxpiVisionInfo;
    int dataSize = 0;
    uint32_t *dataPtr = nullptr;
    void *reservedPtr = nullptr;
};

struct MxstProtobufIn {
    std::string key;
    std::shared_ptr<google::protobuf::Message> messagePtr;
};

struct MxstMetadataInput {
    std::string dataSource;
    std::shared_ptr<google::protobuf::Message> messagePtr;
    void *reservedPtr = nullptr;
};

struct MxstProtobufAndBuffer {
    std::map<std::string, std::shared_ptr<google::protobuf::Message>> mxpiProtobufMap;
    MxstDataOutput *dataOutput;
};

struct MxstBufferAndMetadata {
    std::map<std::string, std::shared_ptr<google::protobuf::Message>> mxpiProtobufMap;
    MxstBufferOutput *bufferOutput;
    void *reservedPtr = nullptr;
};

struct MxstProtobufOut {
    APP_ERROR errorCode = APP_ERR_OK;
    std::string messageName;
    std::shared_ptr<google::protobuf::Message> messagePtr;
    MxstProtobufOut() = default;

    explicit MxstProtobufOut(APP_ERROR errorCode, const std::string& messageName = "")
        : errorCode(errorCode), messageName(std::move(messageName)) {}
};

struct MxstMetadataOutput {
    APP_ERROR errorCode = APP_ERR_OK;
    std::string errorMsg;
    std::string dataType;
    std::shared_ptr<google::protobuf::Message> dataPtr;
    void *reservedPtr = nullptr;

    MxstMetadataOutput() = default;

    explicit MxstMetadataOutput(const std::string& dataType)
        : dataType(std::move(dataType)) {}

    explicit MxstMetadataOutput(APP_ERROR errorCode, const std::string& errorMsg = "")
        : errorCode(errorCode), errorMsg(std::move(errorMsg)) {}

    void SetErrorInfo(APP_ERROR errorCodeIn, const std::string& errorMsgIn)
    {
        errorCode = errorCodeIn;
        errorMsg = errorMsgIn;
    }
};

struct MxstBufferAndMetadataOutput {
    APP_ERROR errorCode = APP_ERR_OK;
    std::string errorMsg;
    std::shared_ptr<MxstBufferOutput> bufferOutput;
    std::vector<MxstMetadataOutput> metadataVec;
    void *reservedPtr = nullptr;

    MxstBufferAndMetadataOutput() = default;

    explicit MxstBufferAndMetadataOutput(APP_ERROR errorCode, const std::string& errorMsg = "")
        : errorCode(errorCode), errorMsg(std::move(errorMsg)) {}

    void SetErrorInfo(APP_ERROR errorCodeIn, const std::string& errorMsgIn)
    {
        errorCode = errorCodeIn;
        errorMsg = errorMsgIn;
    }
};

const std::string STREAM_CONFIG = "stream_config";
const std::string DEVICEID_KEY_NAME = "deviceId";
const int DELAY_TIME = 3000;
}

#endif // MXSM_DATA_TYPE_H
