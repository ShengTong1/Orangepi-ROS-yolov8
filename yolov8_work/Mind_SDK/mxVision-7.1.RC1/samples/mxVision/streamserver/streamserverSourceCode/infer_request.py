#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
# Copyright (c) Huawei Technologies Co., Ltd. 2024-2025. All rights reserved.
"""
Copyright (c) Huawei Technologies Co., Ltd. 2024-2025. All rights reserved.
Description: Infer request.
Author: Vision SDK
Create: 2024
History: NA
"""
from server_options_and_logger import logger as logging


class InferRequest:
    def __init__(self, infer_config, http_request=None):
        self.infer_config = infer_config
        self.request = http_request
        self.inputs = dict()
        self.outputs = dict()
        self.is_processed = False
        self.is_error = False

    def add_input(self, index, input_tensor):
        if index in self.inputs:
            logging.error("Input %s already exists in request.", index)
            return False
        self.inputs[index] = input_tensor
        return True

    def get_input(self, index):
        if index in self.inputs:
            return self.inputs.get(index)
        logging.error("Input %s not exists in request.", index)
        raise Exception("Input %s not exists in request.", index)

    def add_output(self, index, output_tensor):
        if index in self.outputs:
            logging.error("Output %s already exists in request.", index)
            raise Exception("Output %s already exists in request.", index)
        self.outputs[index] = output_tensor

    def get_output(self, index):
        if index in self.outputs:
            return self.outputs[index]
        logging.error("Output %s not exists in request.", index)
        raise Exception("Output %s not exists in request.", index)

    def get_request(self):
        return self.request

    def get_infer_config(self):
        return self.infer_config

    def get_input_ids(self):
        return list(self.inputs.keys())

    def get_output_ids(self):
        return list(self.outputs.keys())

    def get_output_json(self):
        return [item.get_tensor_json() for _, item in self.outputs.items()]
