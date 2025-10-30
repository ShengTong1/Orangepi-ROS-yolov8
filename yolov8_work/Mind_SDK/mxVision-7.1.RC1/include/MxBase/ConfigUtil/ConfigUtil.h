/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
 * Description: Obtains key-value pairs in a file.
 * Author: MindX SDK
 * Create: 2020
 * History: NA
 */

#ifndef CONFIG_UTIL_H
#define CONFIG_UTIL_H

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <map>
#include <typeinfo>
#include <nlohmann/json.hpp>
#include "boost/algorithm/string.hpp"
#include "MxBase/ErrorCode/ErrorCode.h"
#include "MxBase/Log/Log.h"

namespace MxBase {
enum ConfigMode {
    CONFIGJSON = 0,    // Json file
    CONFIGFILE,        // Normal file
    CONFIGPM,          // Pm file
    CONFIGCONTENT      // Json content
};

class ConfigData {
public:
    // Constructor
    ConfigData();
    ConfigData(const ConfigData &other);
    ConfigData &operator = (const ConfigData &other);
    ~ConfigData() {};

public:
    // Set value by key
    APP_ERROR SetJsonValue(const std::string &key, const std::string &value, int pos = -1);
    // Load the class name
    APP_ERROR LoadLabels(const std::string &labelPath);
    // Get the class name by class id
    std::string GetClassName(const size_t classId);
    // Get file value by key
    template<typename T> APP_ERROR GetFileValue(const std::string &key, T &value) const
    {
        if (cfgFile_.count(key) == 0) {
            return APP_ERR_COMM_NO_EXIST;
        }
        T valueBak = value;
        if (*(typeid(T).name()) == 'b') {
            std::string str = cfgFile_.find(key)->second;
            boost::algorithm::to_lower(str);
            if (!(std::stringstream(str) >> std::boolalpha >> value)) {
                value = valueBak;
                return APP_ERR_COMM_INVALID_PARAM;
            }
        } else {
            std::string str = cfgFile_.find(key)->second;
            if (!(std::stringstream(str) >> value)) {
                value = valueBak;
                return APP_ERR_COMM_INVALID_PARAM;
            }
        }
        return APP_ERR_OK;
    }

    // Get file value by key
    template<typename T> APP_ERROR GetFileValue(const std::string &key, T &value, const T &min, const T &max) const
    {
        if (cfgFile_.count(key) == 0) {
            return APP_ERR_COMM_NO_EXIST;
        }
        T newValue;
        if (*(typeid(T).name()) == 'b') {
            std::string str = cfgFile_.find(key)->second;
            boost::algorithm::to_lower(str);
            if (!(std::stringstream(str) >> std::boolalpha >> newValue)) {
                return APP_ERR_COMM_INVALID_PARAM;
            }
        } else {
            std::string str = cfgFile_.find(key)->second;
            if (!(std::stringstream(str) >> newValue)) {
                return APP_ERR_COMM_INVALID_PARAM;
            }
        }
        if (newValue < min) {
            value = min;
            LogInfo << "The value of the key from config is lower than the limit"
                    << ", therefore it is set as min value instead.";
        } else if (newValue > max) {
            value = max;
            LogInfo << "The value of the key from config is more than the limit"
                    << ", therefore it is set as max value instead.";
        } else {
            value = newValue;
        }
        return APP_ERR_OK;
    }

    template<typename T> void GetFileValueWarn(const std::string &key, T &value) const
    {
        auto ret = GetFileValue<T>(key, value);
        if (ret != APP_ERR_OK) {
            LogWarn << GetErrorInfo(ret) << "Fail to read key from config.";
        }
    }

    template<typename T> void GetFileValueWarn(const std::string &key, T &value, const T &min, const T &max) const
    {
        auto ret = GetFileValue<T>(key, value, min, max);
        if (ret != APP_ERR_OK) {
            LogWarn << GetErrorInfo(ret) << "Fail to read key from config.";
        }
    }

    // Set file value by key
    template<typename T> APP_ERROR SetFileValue(const std::string &key, const T &value)
    {
        if (key.empty()) {
            LogError << "The key cannot be empty." << GetErrorInfo(APP_ERR_COMM_INVALID_PARAM);
            return APP_ERR_COMM_INVALID_PARAM;
        }
        if (*(typeid(T).name()) == 'b') {
            saveBuf_ << key << " = " << std::boolalpha << value << std::endl;
        } else {
            saveBuf_ << key << " = " << value << std::endl;
        }
        return APP_ERR_OK;
    }

    // Init file
    APP_ERROR InitFile(std::ifstream &inFile);

    APP_ERROR InitJson(std::ifstream &inFile);
    // Init content
    APP_ERROR InitContent(const std::string &content);
    // Get json
    std::string GetCfgJson();

private:
    nlohmann::json cfgJson_ = {};                   // Save json value
    std::map<std::string, std::string> cfgFile_ = {}; // Save file value
    std::stringstream saveBuf_;              // Save stream value
    std::vector<std::string> labelVec_ = {}; // labels info
};

class ConfigUtil {
public:
    // Load file
    APP_ERROR LoadConfiguration(const std::string &config, ConfigData &data, ConfigMode mode = CONFIGJSON);

private:
    std::ofstream outfile_;
};
} // namespace MxBase
#endif
