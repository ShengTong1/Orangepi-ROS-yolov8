#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Copyright (c) Huawei Technologies Co., Ltd. 2024-2025. All rights reserved.
"""
Copyright (c) Huawei Technologies Co., Ltd. 2024-2025. All rights reserved.
Description: error code with message.
Author: Vision SDK
Create: 2024
History: NA
"""


class ErrorCode:
    SUCCESS = 0
    INFER_FAILED = 1
    INVALID_URI = 2
    TARGET_NOT_EXISTS = 3
    INVALID_BODY = 4
    CACHE_FULL = 5
    MODEL_INPUT_FAILED = 6
    MODEL_OUTPUT_FAILED = 7
    RATE_EXCEEDED = 8
    ILLEGAL_CONTENT = 9
    LARGE_CONTENT = 10
    INTERNAL_ERROR = 11
    UNSUPPORTED_METHOD = 12


ErrorMessage = {
    ErrorCode.SUCCESS: "Succeeded!",
    ErrorCode.INFER_FAILED: "Failed!",
    ErrorCode.INVALID_URI: "Invalid uri, format is incorrect!",
    ErrorCode.TARGET_NOT_EXISTS: "Illegal request, requested stream or model does not exist!",
    ErrorCode.INVALID_BODY: "Illegal request, the request body is invalid!",
    ErrorCode.CACHE_FULL: "Request Cache is full!",
    ErrorCode.MODEL_INPUT_FAILED: "Send input to model failed!",
    ErrorCode.MODEL_OUTPUT_FAILED: "Get output from model failed!",
    ErrorCode.RATE_EXCEEDED: "Maximum request rate exceeded!",
    ErrorCode.ILLEGAL_CONTENT: "Illegal content type! expected application/json",
    ErrorCode.LARGE_CONTENT: "The length of the content with its request head has exceeded!",
    ErrorCode.INTERNAL_ERROR: "The server has encountered an internal error!",
    ErrorCode.UNSUPPORTED_METHOD: "Options request or head request is not allowed!",
}
