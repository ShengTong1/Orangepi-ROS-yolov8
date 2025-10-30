/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
 * Description: Determines and parses the plug-in attributes set in the pipeline.
 * Author: MindX SDK
 * Create: 2021
 * History: NA
 */

#ifndef MXSTREAM_FUNCTIONAL_STREAM_H
#define MXSTREAM_FUNCTIONAL_STREAM_H

#include <list>
#include <vector>
#include "MxStream/Stream/Stream.h"
#include "MxStream/Stream/PluginNode.h"

namespace MxStream {
class FunctionalStream : public Stream {
public:
    FunctionalStream(const std::string& name, const std::vector<PluginNode>& inputs,
        const std::vector<PluginNode>& outputs);
    FunctionalStream(const std::string& name);
    ~FunctionalStream();

    APP_ERROR Build();

private:
    std::vector<PluginNode> inputs_;
    std::vector<PluginNode> outputs_;

private:
    FunctionalStream() = delete;
    FunctionalStream(const FunctionalStream &) = delete;
    FunctionalStream(const FunctionalStream &&) = delete;
    FunctionalStream& operator=(const FunctionalStream &) = delete;
    FunctionalStream& operator=(const FunctionalStream &&) = delete;
};
}
#endif // MXSTREAM_FUNCTIONAL_STREAM_H