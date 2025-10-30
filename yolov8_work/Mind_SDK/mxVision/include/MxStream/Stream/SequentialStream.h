/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
 * Description: a implementation of Stream.
 * Author: MindX SDK
 * Create: 2021
 * History: NA
 */

#ifndef MXSTREAM_SEQUENTIAL_STREAM_H
#define MXSTREAM_SEQUENTIAL_STREAM_H

#include "MxStream/Stream/Stream.h"
#include "MxStream/Stream/PluginNode.h"

namespace MxStream {
class SequentialStream : public Stream {
public:
    SequentialStream(const std::string &name);
    ~SequentialStream();

    APP_ERROR Add(const PluginNode &pluginNode);

    APP_ERROR Build();

private:
    std::vector<PluginNode> pluginNodeVec_;

private:
    SequentialStream() = delete;
    SequentialStream(const SequentialStream &) = delete;
    SequentialStream(const SequentialStream &&) = delete;
    SequentialStream& operator=(const SequentialStream &) = delete;
    SequentialStream& operator=(const SequentialStream &&) = delete;
};
}
#endif // MXSTREAM_SEQUENTIAL_STREAM_H