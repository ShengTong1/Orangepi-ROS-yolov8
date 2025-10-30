/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
 * Description: Device-side management.
 * Author: MindX SDK
 * Create: 2020
 * History: NA
 */

#ifndef DEVICE_MANAGER_H
#define DEVICE_MANAGER_H

#include <map>
#include <string>
#include <mutex>
#include <memory>
#include "MxBase/ErrorCode/ErrorCode.h"
namespace MxBase {
const unsigned int DEFAULT_VALUE = 0;
struct DeviceContext {
    enum DeviceStatus {
        IDLE = 0,  // idle status
        USING      // running status
    } devStatus = IDLE;
    int32_t devId = DEFAULT_VALUE;
};

class DeviceManager {
public:
    virtual ~DeviceManager();
    static DeviceManager *GetInstance();
    // initailze all devices
    APP_ERROR InitDevices(std::string configFilePath = "");
    // get all devices count
    APP_ERROR GetDevicesCount(uint32_t& deviceCount);
    // get current running device
    APP_ERROR GetCurrentDevice(DeviceContext& device);
    // set one device for running
    APP_ERROR SetDevice(DeviceContext device);
    // release all devices
    APP_ERROR DestroyDevices();
    bool IsInitDevices() const;
    APP_ERROR CheckDeviceId(int32_t deviceId);
    static bool IsAscend310();
    static bool IsAscend310B();
    static bool IsAscend310P();
    static bool IsAtlas800IA2();
    static std::string GetSocName();
private:
    DeviceManager() = default;
    std::mutex mtx_ = {};
    std::map<int32_t, std::shared_ptr<void>> contexts_ = {};
    uint32_t deviceCount_ = 0;
    uint32_t initCounter_ = 0;
    bool destroyFlag_ = false;
};
}  // namespace MxBase

#endif  // DEVICE_MANAGER_H
