#!/usr/bin/env python
# coding=utf-8

"""
Copyright (c) Huawei Technologies Co., Ltd. 2022-2022. All rights reserved.
Description: Check and import the config file.
Author: MindX SDK
Create: 2022
History: NA
"""

import os

size = os.path.getsize('config/config.py')
if size > 100 * 1024 * 1024:
    raise Exception("Import config failed, config size is bigger than max permitted size.")