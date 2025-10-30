#!/bin/bash

# 设置环境变量
if [ -f /usr/local/Ascend/ascend-toolkit/set_env.sh ]; then
    source /usr/local/Ascend/ascend-toolkit/set_env.sh
fi

if [ -f ${SDK_INSTALL_PATH}/mxVision/set_env.sh ]; then
    source ${SDK_INSTALL_PATH}/mxVision/set_env.sh
fi

# 运行YOLOv8检测
python3 yolov8_detect.py



