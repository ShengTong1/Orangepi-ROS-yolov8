#!/bin/bash

# Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
# Description: run.sh.
# Author: MindX SDK
# Create: 2021
# History: NA

set -e

# Simple log helper functions
info() { echo -e "\033[1;34m[INFO ][MxStream] $1\033[1;37m" ; }
error() { echo >&2 -e "\033[1;31m[ERROR ][MxStream] $1\033[1;37m" ; }

CUR_PATH=$(cd "$(dirname "$0")" || { error "Failed to check path/to/run.sh" ; exit 1; } ; pwd)

if [ ! -O "$0" ]; then
    error "Error: run.sh is not belong to the current user."
    exit 1
fi

if [ $(id -u) -eq 0 ]; then
    info "Info: This shell is executed by root."
fi

cd "${CUR_PATH}" || { error "Failed to change directory. " ; exit 1; }
if [ ! -f "../../../set_env.sh" ]; then
    error "Error: set_env.sh is not found."
    exit 1
fi

if [ ! -O "../../../set_env.sh" ]; then
    error "Error: set_env.sh is not belong to the current user."
    exit 1
fi

. /usr/local/Ascend/ascend-toolkit/set_env.sh
. ../../../set_env.sh

# import certificate
if [ -n "$4" ]; then
    bash certImport.sh -c $1 -s $2 -k $3 -r $4
else
    bash certImport.sh -c $1 -s $2 -k $3
fi

if [ $? -ne 0 ]; then
    warn "Failed to import certificate."
    exit 0
fi

# run
python3 streamserverSourceCode/main.py
