/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
 * Description: StateInfo structure.
 * Author: MindX SDK
 * Create: 2021
 * History: NA
 */

#ifndef MXSTREAM_STATE_INFO_H
#define MXSTREAM_STATE_INFO_H

namespace MxStream {
enum StreamState {
    STREAM_STATE_NORMAL = 0,
    STREAM_STATE_NEW,
    STREAM_STATE_BUILD_INPROGRESS,
    STREAM_STATE_BUILD_FAILED,
    STREAM_STATE_DESTROY,
};
}
#endif // MXSTREAM_STATE_INFO_H