#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
YOLOv8 目标检测示例
参考代码结构
"""

from mindx.sdk import base
import numpy as np
import cv2
import yaml
from det_utils import *  # 模型前后处理相关函数


if __name__ == '__main__':
    base.mx_init()  # 初始化 mxVision 资源
    DEVICE_ID = 0  # 设备id

    # 从 yaml 文件读取类别
    with open('data.yaml', 'r', encoding='utf-8') as f:
        yaml_data = yaml.safe_load(f)
    class_names = yaml_data.get('names', [])
    class_list = class_names
    print(f"类别列表: {class_list}")

    std_h, std_w = 640, 640  # 标准输入尺寸
    
    model_path = 'yolov8.om'
    img_path = 'test.jpg'

    # 读取图片
    img = cv2.imread(img_path)
    if img is None or img.size == 0:
        print("路径有误！")
        exit(1)
    
    print(f"读取图片成功，尺寸: {img.shape}")
    
    # 前处理
    img_after, scale, dh, dw = resize_image(img, (std_w, std_h), True)
    letterbox_info = (scale, dw, dh)
    
    # 将图像处理成输入的格式
    data = img2input(img_after)

    # 模型推理
    print("开始推理...")
    model = base.model(modelPath=model_path, deviceId=DEVICE_ID)
    output = model.infer([data])[0]  # 执行推理

    # 后处理
    output.to_host()  # 将Tensor数据转移到内存
    output = np.array(output)  # 将数据转为 numpy array 类型
    print(f"模型输出形状: {output.shape}")
    
    # 标准化输出
    pred = std_output(output)
    
    # 置信度过滤+nms
    result = nms(pred, 0.25, 0.45)  # [x,y,w,h,conf(最大类别概率),class]
    
    # 坐标变换
    result = cod_trf(result, img, img_after, letterbox_info)
    
    # 绘制结果
    image = draw(result, img.copy(), class_list)

    # 保存输出图像
    result_path = "yolov8_result.jpg"
    cv2.imwrite(result_path, image)
    print('保存推理结果成功: {}'.format(result_path))
    
    # 去初始化
    base.mx_deinit()
