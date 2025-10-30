/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
 * Description: Implementation of framework functions such as plug-in initialization and plug-in class registration.
 * Author: MindX SDK
 * Create: 2020
 * History: NA
 */

#ifndef MX_GST_BASE_H_
#define MX_GST_BASE_H_

#include "MxTools/PluginToolkit/base/MxPluginBase.h"
#include <iostream>
#include <vector>
#include <map>
#include "gst/gst.h"
#include <thread>
#include <mutex>
#include <condition_variable>


namespace MxTools {
const unsigned int MAX_PAD_NUM = 256;
G_BEGIN_DECLS

#define GST_TYPE_MXBASE (MxGstBaseGetType())
#define GST_MXBASE(obj) (G_TYPE_CHECK_INSTANCE_CAST((obj), GST_TYPE_MXBASE, MxGstBase))
#define GST_MXBASE_CLASS(klass) (G_TYPE_CHECK_CLASS_CAST((klass), GST_TYPE_MXBASE, MxGstBaseClass))
#define GST_MXBASE_GET_CLASS(obj) (G_TYPE_INSTANCE_GET_CLASS((obj), GST_TYPE_MXBASE, MxGstBaseClass))
#define GST_IS_GST_MXBASE(obj) (G_TYPE_CHECK_INSTANCE_TYPE((obj), GST_TYPE_MXBASE))
#define GST_IS_GST_MXBASE_CLASS(klass) (G_TYPE_CHECK_CLASS_TYPE((klass), GST_TYPE_MXBASE))
#define GST_GST_MXBASE_CAST(obj) ((MxGstBase *)(obj))

struct MxGstBase {
    GstElement element;
    guint padIdx;   // request index
    guint flushStartNum;
    guint flushStopNum;
    std::vector<GstPad *> sinkPadVec;
    std::vector<GstPad *> srcPadVec;
    MxPluginBase* pluginInstance;
    std::unique_ptr<std::map<std::string, std::shared_ptr<void>>> configParam;
    std::vector<MxpiBuffer *> input;
    std::vector<MxpiBuffer *> inputQueue;
    std::mutex inputMutex_;
    std::mutex eventMutex_;
    std::condition_variable condition_;
};

struct MxGstBaseClass {
    GstElementClass parentClass;

    MxPluginBase* (* CreatePluginInstance)();
};

enum {
    ASYNC = 0,
    SYNC = 1,
};

GType MxGstBaseGetType(void);
G_END_DECLS
}

#endif
