#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Copyright (c) Huawei Technologies Co., Ltd. 2024-2025. All rights reserved.
"""
Copyright (c) Huawei Technologies Co., Ltd. 2024-2025. All rights reserved.
Description: Token bucket algorithm.
Author: Vision SDK
Create: 2024
History: NA
"""
import time
from threading import Lock

from server_options_and_logger import logger as logging
from server_options_and_logger import server_option_instance


class TokenBucket:
    """Token bucket algorithm class

    Attributes：
        bucket_capacity（int）：Total capacity of the token bucket
        generate_rate（int）：Token generation rate
        current_token_amount（int）：The number of tokens in the current token bucket
        last_consume_time （int）：Time of last consumption
        token_bucket_lock（obj：threading.Lock）：Token bucket thread lock

    """
    def __init__(self, capacity: int = 20, rate: int = 10):
        """The capacity is the total number of tokens in the bucket. Rate is the rate of token generation.

        Attributes：
            capacity（int）：Total capacity of the token bucket. The default value is 20.
            rate（int）：Token generation rate of the token bucket. The default value is 10/s.

        """

        self.bucket_capacity = capacity
        self.generate_rate = rate
        self.current_token_amount = capacity
        self.last_consume_time = int(time.perf_counter())
        self.token_bucket_lock = Lock()

    def consume_token(self, consume_token_amount: int) -> bool:
        if self.token_bucket_lock.locked():
            logging.warning("Token bucket lock is locked. Please wait for it to release.")
            return False

        with self.token_bucket_lock:
            cur_consume_time = int(time.perf_counter())
            token_increment = (cur_consume_time - self.last_consume_time) * self.generate_rate
            self.current_token_amount = min(self.bucket_capacity, token_increment + self.current_token_amount)
            if consume_token_amount > self.current_token_amount:
                logging.warning("Consume tokens amount be more current tokens amount. "
                                "Please wait for it to be released.")
                self.last_consume_time = cur_consume_time
                return False

            self.current_token_amount -= consume_token_amount
            self.last_consume_time = cur_consume_time
            return True


token_bucket_instance = TokenBucket(server_option_instance.max_request_rate * 2,
                                    server_option_instance.max_request_rate)
