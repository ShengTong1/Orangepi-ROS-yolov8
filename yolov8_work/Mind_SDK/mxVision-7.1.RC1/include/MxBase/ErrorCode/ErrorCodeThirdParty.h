/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
 * Description: Definition of Returned Error Codes.
 * Author: MindX SDK
 * Create: 2020
 * History: NA
 */

#ifndef ERROR_CODE_THIRD_PARTY_H
#define ERROR_CODE_THIRD_PARTY_H

enum {
    APP_ERR_FLOW_CUSTOM_SUCCESS_2 = 102,
    APP_ERR_FLOW_CUSTOM_SUCCESS_1 = 101,
    APP_ERR_FLOW_CUSTOM_SUCCESS = 100,
    APP_ERR_FLOW_OK		  =  0,
    APP_ERR_FLOW_NOT_LINKED     = -1,
    APP_ERR_FLOW_FLUSHING       = -2,
    APP_ERR_FLOW_EOS            = -3,
    APP_ERR_FLOW_NOT_NEGOTIATED = -4,
    APP_ERR_FLOW_ERROR	  = -5,
    APP_ERR_FLOW_NOT_SUPPORTED  = -6,
    APP_ERR_FLOW_CUSTOM_ERROR   = -100,
    APP_ERR_FLOW_CUSTOM_ERROR_1 = -101,
    APP_ERR_FLOW_CUSTOM_ERROR_2 = -102
};

#endif  // ERROR_CODE_H_