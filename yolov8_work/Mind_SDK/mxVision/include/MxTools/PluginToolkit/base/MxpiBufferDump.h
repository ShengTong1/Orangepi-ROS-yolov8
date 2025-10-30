/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
 * Description: Complete plug-in data download.
 * Author: MindX SDK
 * Create: 2020
 * History: NA
 */

#ifndef MX_MXPIBUFFERDUMP_H
#define MX_MXPIBUFFERDUMP_H

#include "MxBase/ErrorCode/ErrorCode.h"
#include "MxBase/MemoryHelper/MemoryHelper.h"
#include "MxTools/Proto/MxpiDataType.pb.h"
#include "MxTools/Proto/MxpiDumpData.pb.h"

namespace MxTools {
class MxpiBuffer;

class MxpiBufferDump {
public:
    static std::string DoDump(MxTools::MxpiBuffer& mxpiBuffer,
        const std::vector<std::string>& filterKeys = std::vector<std::string>(),
        const std::vector<std::string>& requiredKeys = std::vector<std::string>());

    static MxTools::MxpiBuffer* DoLoad(MxTools::MxpiBuffer& mxpiBuffer, int deviceId = 0);
    static MxTools::MxpiBuffer* DoLoad(const std::string& filePath, int deviceId = 0);

private:
    static void BuildBufferData(MxpiBuffer& mxpiBuffer, MxTools::MxpiDumpData& dumpData);

    static APP_ERROR BuildMxpiVisionData(std::pair<const std::string, std::shared_ptr<void>>& item);

    static bool IsMetaDataKeyFilter(const std::vector<std::string>& filterKeys,
        const std::vector<std::string>& requiredKeys, const std::string& metaKey);

    static std::string GetHostMemoryData(void* dataPtr, int dataSize, const MxpiMemoryType& memoryType,
        unsigned int deviceId);

    static APP_ERROR HandleProtoDataType(std::pair<const std::string, std::shared_ptr<void>>& item,
                                         const std::string& metaKey, MetaData* dumpMetaData,
                                         bool isDumpMemoryData);

    static void BuildTensorPackageList(const std::pair<const std::string, std::shared_ptr<void>>& item);

    static APP_ERROR BuildStringData(void* mxpiMetadataManager, const MxTools::MetaData& metaData);

    static APP_ERROR BuildMxpiBuffer(MxTools::MxpiDumpData& dumpData, MxTools::MxpiBuffer& mxpiBuffer);

    static APP_ERROR BuildMetaData(MxTools::MxpiBuffer& mxpiBuffer, MxTools::MxpiDumpData& dumpData, int deviceId);

    static APP_ERROR BuildVisionData(void* mxpiMetadataManager, const MxTools::MetaData& metaData, int deviceId);

    static APP_ERROR BuildTensorPackage(void* mxpiMetadataManager, const MxTools::MetaData& metaData, int deviceId);

    static APP_ERROR LoadDataToMemory(const std::string& content,
                                      MxTools::MxpiMemoryType memoryType,
                                      void* &dstPtrData,
                                      int deviceId);

    static google::protobuf::Message* CreateMessage(const std::string& typeName);
};
}
#endif
