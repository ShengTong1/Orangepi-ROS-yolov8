#!/bin/bash

# Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
# Description: Set and find the path of the runtime.
# Author: MindX SDK
# Create: 2020
# History: NA

# Simple log helper functions
info() { echo -e "\033[1;34m[INFO ][MxStream] $1\033[1;37m" ; }
error() { echo >&2 -e "\033[1;31m[ERROR ][MxStream] $1\033[1;37m" ; }

CUR_PATH=$(cd "$(dirname "$0")" || { error "Failed to check path/to/run.sh" ; exit 1; } ; pwd)

export MX_SDK_HOME=${CUR_PATH}/../..

export LD_LIBRARY_PATH=${MX_SDK_HOME}/lib:${MX_SDK_HOME}/opensource/lib:${MX_SDK_HOME}/opensource/lib64:/usr/local/Ascend/ascend-toolkit/latest/acllib/lib64:${LD_LIBRARY_PATH}
export GST_PLUGIN_SCANNER=${MX_SDK_HOME}/opensource/libexec/gstreamer-1.0/gst-plugin-scanner
export GST_PLUGIN_PATH=${MX_SDK_HOME}/opensource/lib/gstreamer-1.0:${MX_SDK_HOME}/lib/plugins

#to set PYTHONPATH, import the StreamManagerApi.py
export PYTHONPATH=$PYTHONPATH:${MX_SDK_HOME}/python

cd "${CUR_PATH}" || { error "Failed to change directory. " ; exit 1; }
python3 main.py $@
