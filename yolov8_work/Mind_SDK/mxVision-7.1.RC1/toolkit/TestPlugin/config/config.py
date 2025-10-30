#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
Description: Single-plug-in test tool pipeline.
Author: MindX SDK
Create: 2020
History: NA
"""

PipelineConfig = {
    "stream_config": {
        "deviceId": "0"
    },
    "factory": "mxpi_imageresize",       # 插件类型
    "plugin_name": "mxpi_imageresize0",  # 插件名称
    "props": {                           # 插件属性
        "dataSource": "mxpi_imagedecoder0",
        "resizeHeight": "416",
        "resizeWidth": "416"
    },
    "load": [                            # load插件的输入文件名，一个输入文件对应一个load插件
        "input/imageresize0.json"
    ],
    "dump": [                            # dump插件的输出文件名，一个输出文件对应一个dump插件
        "imageresize0-output.json"
    ]
}
