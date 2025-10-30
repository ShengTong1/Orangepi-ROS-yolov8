/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
 * Description: PluginNode structure.
 * Author: MindX SDK
 * Create: 2021
 * History: NA
 */

#ifndef MXSTREAM_PLUGIN_NODE_H
#define MXSTREAM_PLUGIN_NODE_H

#include <map>
#include <nlohmann/json.hpp>

namespace MxStream {
class PluginNodeDptr;

class PluginNode {
public:
    PluginNode(const std::string& factory,
        const std::map<std::string, std::string>& props = std::map<std::string, std::string>(),
        const std::string& name = "");
    PluginNode(const PluginNode &);
    PluginNode(const PluginNode &&);
    ~PluginNode();

    PluginNode& operator()(PluginNode& preNode);

    template<typename... Args>
    PluginNode& operator()(PluginNode& preNode, Args& ... args)
    {
        operator()(preNode);
        operator()(args...);
        return *this;
    }

    PluginNode& operator()(std::vector<PluginNode>& preNodeList);

    int PluginId() const;
    std::string PluginName() const;
    std::string Factory() const;
    std::map<std::string, std::string> Properties() const;
    std::string ToJson() const;

private:
    void SetPluginName(const std::string& name);
    void SetNextNode(const std::string& name);
    void SetNextNodes();
    std::vector<PluginNode>& NextNodes() const;

private:
    std::shared_ptr<PluginNodeDptr> dPtr_ = nullptr;
    static int pluginCount_;

private:
    PluginNode() = delete;
    PluginNode& operator=(const PluginNode &) = delete;
    PluginNode& operator=(const PluginNode &&) = delete;
    friend class PluginNodeDptr;
    friend class Stream;
    friend class StreamDptr;
    friend class SequentialStream;
    friend class FunctionalStream;
    friend class MxsmDescription;
};
}
#endif // MXSTREAM_PLUGIN_NODE_H