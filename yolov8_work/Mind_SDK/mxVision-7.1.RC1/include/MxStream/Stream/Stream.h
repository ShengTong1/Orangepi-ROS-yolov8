/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
 * Description: Determines the plug-in set in the pipeline and creates an inference flow.
 * Author: MindX SDK
 * Create: 2021
 * History: NA
 */

#ifndef MXSTREAM_STREAM_H
#define MXSTREAM_STREAM_H

#include <memory>
#include <vector>
#include "MxBase/ErrorCode/ErrorCode.h"
#include "MxStream/DataType/DataType.h"
#include "MxStream/DataType/StateInfo.h"
#include "MxStream/Stream/PluginNode.h"
#include "MxStream/StreamManager/MxsmDataType.h"

namespace MxStream {
class MxsmStream;
class StreamDptr;
class MxsmDescription;
class Stream {
public:
    explicit Stream(const std::string& pipelinePath);
    Stream(const std::string& pipelinePath, const std::string& streamName);
    virtual ~Stream();

    APP_ERROR SendData(const std::string& elementName, std::vector<MxstMetadataInput>& metadataVec,
        MxstBufferInput& dataBuffer);
    MxstBufferAndMetadataOutput GetResult(const std::string& elementName,
        const std::vector<std::string>& dataSourceVec, const uint32_t& msTimeOut = DELAY_TIME);
    APP_ERROR SendMultiDataWithUniqueId(std::vector<int> inPluginIdVec,
        std::vector<MxstDataInput>& dataInputVec,
        uint64_t& uniqueId);
    std::vector<MxstDataOutput*> GetMultiResultWithUniqueId(uint64_t uniqueId, uint32_t timeOutMs = DELAY_TIME);
    APP_ERROR SetElementProperty(const std::string& elementName, const std::string& propertyName,
        const std::string& propertyValue);
    void SetDeviceId(const std::string& deviceId);
    std::string ToJson() const;

    virtual APP_ERROR Build();
    APP_ERROR Start();
    APP_ERROR Stop();

protected:
    std::shared_ptr<StreamDptr> dPtr_ = nullptr;
    std::shared_ptr<MxsmDescription> mxsmDescription_ = nullptr;

private:
    Stream() = delete;
    Stream(const Stream &) = delete;
    Stream(const Stream &&) = delete;
    Stream& operator=(const Stream &) = delete;
    Stream& operator=(const Stream &&) = delete;
    friend class StreamDptr;
};
}
#endif // MXSTREAM_STREAM_H