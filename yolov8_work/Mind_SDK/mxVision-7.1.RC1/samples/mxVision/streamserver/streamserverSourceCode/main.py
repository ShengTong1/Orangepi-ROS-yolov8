#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Copyright (c) Huawei Technologies Co., Ltd. 2024-2025. All rights reserved.
"""
Copyright (c) Huawei Technologies Co., Ltd. 2024-2025. All rights reserved.
Description: Start restful service with flask.
Author: Vision SDK
Create: 2024
History: NA
"""

from http.server import BaseHTTPRequestHandler
from utils import get_sys_info
from server_options_and_logger import server_option_instance
from server_options_and_logger import logger as logging
from kmc_and_ssl_context import ssl_context
from create_app import StreamServerServiceApp
from infer_server_manager import InferServerManager


def version_string(self):
    return 'server'


BaseHTTPRequestHandler.version_string = version_string


def start_service(infer_server_manager):
    curr_user, hostname, user_ip = get_sys_info()
    logging.info("starting stream server: [%s], [%s], [%s]", user_ip, curr_user, hostname)
    infer_server_manager.init_infer_server()
    infer_server_manager.start_infer_server()
    stream_service = StreamServerServiceApp(infer_server_manager, server_option_instance)
    app = stream_service.create_app()
    app.run(host="127.0.0.1", port=server_option_instance.port, ssl_context=ssl_context, debug=False)


def main():
    infer_server_manager = InferServerManager(server_option_instance)
    try:
        start_service(infer_server_manager)
    except Exception as err_message:
        logging.error(err_message)
        logging.error('stream_server startup failed.')
    finally:
        infer_server_manager.stop_service()


if __name__ == '__main__':
    main()