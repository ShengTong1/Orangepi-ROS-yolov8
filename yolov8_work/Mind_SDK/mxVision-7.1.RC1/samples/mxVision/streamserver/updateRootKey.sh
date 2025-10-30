#!/bin/bash

# Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
# Description: updateRootKey.sh.
# Author: MindX SDK
# Create: 2021
# History: NA

CUR_PATH=$(cd "$(dirname "$0")" || { warn "Failed to check path/to/updateRootKey.sh" ; exit ; } ; pwd)

info_record_path="${HOME}/log/mindxsdk"
info_record_file="deployment.log"
log_size_threshold=1024000

ms_deployment_log_rotate() {
  if [ -L "$info_record_path" ]; then
    echo "The directory path of deployment.log cannot be a symlink." >&2
    exit 1
  fi
  if [[ ! -d "$info_record_path" ]];then
    mkdir -p "$info_record_path"
    chmod 750 "$info_record_path"
  fi
  record_file_path="$info_record_path"/"$info_record_file"
  if [ -L "$record_file_path" ]; then
    echo "The deployment.log cannot be a symlink." >&2
    exit 1
  fi
  if [[ ! -f "$record_file_path" ]];then
    touch "$record_file_path" 2>/dev/null
  fi
  record_file_path_bk="$info_record_path"/"$info_record_file".bk
  if [ -L "$record_file_path_bk" ]; then
    echo "The deployment.log.bk cannot be a symlink." >&2
    exit 1
  fi
  log_size=$(find "$record_file_path" -exec ls -l {} \; | awk '{ print $5 }')
  if [[ "${log_size}" -ge "${log_size_threshold}" ]];then
    mv -f "$record_file_path" "$record_file_path_bk"
    touch "$record_file_path" 2>/dev/null
    chmod 400 "$record_file_path_bk"
  fi
  chmod 600 "$record_file_path"
}

record_info() {
  ms_deployment_log_rotate
  deployment_path="$info_record_path"/"$info_record_file"
  find "$deployment_path" -type f -exec chmod 750 {} +
  user_ip=$(who am i | awk '{print $NF}' | sed 's/(//g' | sed 's/)//g')
  if [[ -z "${user_ip}" ]]; then
    user_ip=localhost
  fi
  user_name=$(whoami)
  host_name=$(hostname)
  append_text="[$(date "+%Y-%m-%d %H:%M:%S")][$user_ip][$user_name][$host_name] :"
  echo "$append_text" >> "$deployment_path"
  echo "$1" >> "$deployment_path"
  echo "$1"
  find "$deployment_path" -type f -exec chmod 440 {} +
}

if [ ! -O "$0" ]; then
    record_info "Error: updateRootKey.sh is not belong to the current user. Failed to update rootkey."
    exit 1
fi

if [ $(id -u) -eq 0 ]; then
    record_info "Warning: updateRootKey.sh is executed by root."
fi

# Simple log helper functions
info() { echo -e "\033[1;34m[INFO ][MxStream] $1\033[1;37m" ; }
warn() { echo >&2 -e "\033[1;31m[WARN ][MxStream] $1\033[1;37m" ; }

if [ -d "./keys" ]; then
    if [ -O "./keys" ]; then
        echo "Info: keys directory exists."
    else
        record_info "Error: The destination directory \"keys\" is not belong to the current user. Failed to update rootkey."
        exit 1
    fi
fi

if [ ! -f "../../../set_env.sh" ]; then
    record_info "Error: set_env.sh is not found."
    exit 1
fi

if [ ! -O "../../../set_env.sh" ]; then
    record_info "Error: set_env.sh is not belong to the current user. Failed to update rootkey."
    exit 1
fi

. ../../../set_env.sh

log_file="../../../config/logging.conf"
if [ -L "$log_file" ] || [ ! -f "$log_file" ] || [ ! -O "$log_file" ]; then
    record_info "Error: cannot find the logging.conf. Failed to update rootkey."
    exit 1
fi
max_size=$((1024 * 10))
file_size=$(ls -l "$log_file" | awk '{ print $5 }')
if [ "$file_size" -gt "$max_size" ]; then
    record_info "Error: invalid logging.conf, the file size is too large. Failed to update rootkey."
    exit 1
fi
levelc=$(cat ../../../config/logging.conf | grep console_level= | wc -l)
if [ "$levelc" -ne 1 ]; then
    record_info "Error: console_level key should be exact one in config file. Failed to update rootkey."
    exit 1
fi

old_level=$(cat ../../../config/logging.conf | grep console_level=)
pattern="^console_level=(-1|0|1|2|3)$"
if ! [[ "${old_level}" =~ ${pattern} ]]; then
    record_info "Error: console_level in logging.conf is not set or invalid."
    exit 1
fi
sed -i "s/"${old_level}"/console_level=0/" ../../../config/logging.conf
record_info "Change the console_level to 0 in updateRootKey."
# run
../../../bin/UpdateRootKey
sed -i "s/console_level=0/"${old_level}"/" ../../../config/logging.conf
record_info "Change the console_level to its original level in updateRootKey."
record_info "updateRootKey execute finished, see result in sdk log."
exit 0