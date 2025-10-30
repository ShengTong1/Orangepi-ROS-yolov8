#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Copyright (c) Huawei Technologies Co., Ltd. 2024-2025. All rights reserved.
"""
Copyright (c) Huawei Technologies Co., Ltd. 2024-2025. All rights reserved.
Description: KMC encrypt lib to create ssl context.
Author: Vision SDK
Create: 2024
History: NA
"""
import ctypes
import os
import signal
import ssl
from ctypes.util import find_library
from enum import Enum
from functools import wraps
import secrets
from server_options_and_logger import logger as logging
from server_options_and_logger import server_option_instance
from utils import file_base_check

KMC_DLL: ctypes.CDLL = None
LIBC_DLL: ctypes.CDLL = None

# update the key within the period set by advanceDay
ADVANCE_DAY = 3
OK = 0
MAX_BLOCK_SIZE = 1024 * 1024


class SdpAlgorithm(Enum):
    AES128_GCM = 8
    AES256_GCM = 9


class HmacAlgorithm:
    HMAC_SHA256 = 2052
    HMAC_SHA384 = 2053
    HMAC_SHA512 = 2054


class KmcRole:
    AGENT = 0
    MASTER = 1


class LogLevel:
    LOG_DISABLE = 0
    LOG_ERROR = 1
    LOG_WARN = 2
    LOG_INFO = 3
    LOG_DEBUG = 4
    LOG_TRACE = 5


class KmcConfig(ctypes.Structure):
    _fields_ = [
        ("primary_key_store_file", ctypes.c_char * 4096),
        ("standby_key_store_file", ctypes.c_char * 4096),
        ("domain_count", ctypes.c_int),
        ("role", ctypes.c_int),
        ("proc_lock_perm", ctypes.c_int),
        ("sdp_alg_id", ctypes.c_int),
        ("hmac_alg_id", ctypes.c_int),
        ("sem_key", ctypes.c_int),
        ("inner_symm_alg_id", ctypes.c_int),
        ("inner_hash_alg_id", ctypes.c_int),
        ("inner_hmac_alg_id", ctypes.c_int),
        ("inner_kdf_alg_id", ctypes.c_int),
        ("work_key_iter", ctypes.c_int),
        ("root_key_iter", ctypes.c_int)
    ]


@ctypes.CFUNCTYPE(None, ctypes.c_int, ctypes.c_char_p)
def _logger_callback(level: ctypes.c_int, msg: ctypes.c_char_p):
    logging.info("level:%d, msg:%s", level, str(msg))
    if 'errno = 13' in str(msg):
        logging.error('No permission to modify KMC Key file lock semaphore,'
              ' different user should use different kmc_config_sem_key, '
              'you can modify the "kmc_config_sem_key" in config/config.yaml, '
              'kmc_config_sem_key scope: [0x20161112, 0x20169999]')


def _load_dll(kmc_dll_path: str) -> None:
    global KMC_DLL
    if KMC_DLL:
        return

    global LIBC_DLL
    if LIBC_DLL:
        return

    LIBC_DLL = ctypes.CDLL(find_library("c"))

    KMC_DLL = ctypes.CDLL(kmc_dll_path)

    # KeInitialize
    KMC_DLL.KeInitialize.restype = ctypes.c_int
    KMC_DLL.KeInitialize.argtypes = [ctypes.POINTER(KmcConfig)]

    # KeFinalize
    KMC_DLL.KeFinalize.restype = ctypes.c_int
    KMC_DLL.KeFinalize.argtypes = []

    # KeGetCipherDataLen
    KMC_DLL.KeGetCipherDataLen.restype = ctypes.c_int
    KMC_DLL.KeGetCipherDataLen.argtypes = [ctypes.c_int, ctypes.POINTER(ctypes.c_int)]

    # KeSetLoggerLevel
    KMC_DLL.KeSetLoggerLevel.restype = None
    KMC_DLL.KeSetLoggerLevel.argtypes = [ctypes.c_int]

    # KeSetLoggerCallback
    KMC_DLL.KeSetLoggerCallback.restype = None
    KMC_DLL.KeSetLoggerCallback.argtypes = [ctypes.CFUNCTYPE(None, ctypes.c_int, ctypes.c_char_p)]

    # KeEncryptByDomain
    KMC_DLL.KeEncryptByDomain.restype = ctypes.c_int
    KMC_DLL.KeEncryptByDomain.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.POINTER(ctypes.c_char_p),
                                           ctypes.POINTER(ctypes.c_int)]
    # KeDecryptByDomain
    KMC_DLL.KeDecryptByDomain.restype = ctypes.c_int
    KMC_DLL.KeDecryptByDomain.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.POINTER(ctypes.c_char_p),
                                           ctypes.POINTER(ctypes.c_int)]
    # KeHmacByDomain
    KMC_DLL.KeHmacByDomain.restype = ctypes.c_int
    KMC_DLL.KeHmacByDomain.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.POINTER(ctypes.c_char_p),
                                        ctypes.POINTER(ctypes.c_int)]
    # KeHmacVerifyByDomain
    KMC_DLL.KeHmacVerifyByDomain.restype = ctypes.c_int
    KMC_DLL.KeHmacVerifyByDomain.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p,
                                              ctypes.c_int]

    # KeGetMaxMkID
    KMC_DLL.KeGetMaxMkID.restype = ctypes.c_int
    KMC_DLL.KeGetMaxMkID.argtypes = [ctypes.c_uint, ctypes.POINTER(ctypes.c_uint)]

    # KeGetMaxMkID
    KMC_DLL.KeActiveNewKey.restype = ctypes.c_int
    KMC_DLL.KeActiveNewKey.argtypes = [ctypes.c_uint]


def hmac(domain_id: int, plain_text: str) -> str:
    p_char = ctypes.c_char_p()
    hmac_len = ctypes.c_int(0)
    c_plain_text = ctypes.create_string_buffer(plain_text.encode())
    ret = KMC_DLL.KeHmacByDomain(domain_id, c_plain_text, len(plain_text), ctypes.byref(p_char),
                                  ctypes.pointer(hmac_len))
    logging.info("KeHmacByDomain ret=%d" % ret)

    if ret != OK:
        logging.error("KeHmacByDomain failed.")
        return ""
    value = p_char.value.decode()
    ret = LIBC_DLL.free(p_char)
    logging.info("free memory ret=%d" % ret)

    return value


def hmac_verify(domain_id: int, plain_text: str, hmac_text: str) -> bool:
    c_plain_text = ctypes.create_string_buffer(plain_text.encode())
    c_hmac_text = ctypes.create_string_buffer(hmac_text.encode())
    ret = KMC_DLL.KeHmacVerifyByDomain(domain_id, c_plain_text, len(plain_text), c_hmac_text, len(c_hmac_text))
    logging.info("KeHmacVerifyByDomain ret=%d" % ret)
    return ret


def encrypt(domain_id: int, plain_text: str) -> str:
    p_char = ctypes.c_char_p()
    cipher_len = ctypes.c_int(0)
    c_plain_text = ctypes.create_string_buffer(plain_text.encode())

    ret = KMC_DLL.KeEncryptByDomain(domain_id, c_plain_text, len(plain_text), ctypes.byref(p_char),
                                     ctypes.pointer(cipher_len))
    logging.info("KeEncryptByDomain ret=%d" % ret)
    if ret != OK:
        logging.error("KeEncryptByDomain failed.")
        return ""

    value = p_char.value.decode()
    ret = LIBC_DLL.free(p_char)
    if ret != OK:
        logging.error("free memory error. ret=%d" % ret)
    return value


def decrypt(domain_id: int, cipher_text: str):
    p_char = ctypes.c_char_p()
    plain_len = ctypes.c_int(0)
    c_cipher_text = ctypes.create_string_buffer(cipher_text.encode())
    ret = KMC_DLL.KeDecryptByDomain(domain_id, c_cipher_text, len(cipher_text), ctypes.byref(p_char),
                                     ctypes.pointer(plain_len))
    logging.info("KeDecryptByDomain ret=%d" % ret)
    if ret != OK:
        logging.error("KeDecryptByDomain failed.")
        return ""

    value = p_char.value.decode()
    ret = LIBC_DLL.free(p_char)
    if ret != OK:
        logging.error("free memory error. ret=%d" % ret)

    return value


def check_and_update_mk(domain_id: int, advance_day: int) -> None:
    ret = KMC_DLL.KeCheckAndUpdateMk(domain_id, advance_day)
    if ret != OK:
        logging.error("KeCheckAndUpdateMk ret=%d" % ret)
        return
    return


def update_root_key() -> bool:
    ret = KMC_DLL.KeUpdateRootKey()
    if ret != OK:
        logging.error("KeUpdateRootKey ret=%d" % ret)
        return False
    return True


def finalize() -> None:
    KMC_DLL.KeFinalize()


def set_timeout(num):
    def wrap(func):
        def handle(signum, frame):
            raise RuntimeError

        @wraps(func)
        def to_do(*args, **kwargs):
            signal.signal(signal.SIGALRM, handle)
            signal.alarm(num)
            logging.info('start alarm signal.')
            r = func(*args, **kwargs)
            logging.info('close alarm signal.')
            signal.alarm(0)
            return r
        return to_do
    return wrap


@set_timeout(5)
def init(primary_key_store_file: str, standby_key_store_file: str, alg_id: int,
         domain_count=3) -> bool:
    def _check_alg_id() -> bool:
        for alg in SdpAlgorithm:
            if alg_id == alg.value:
                return True
        return False

    if _check_alg_id() is False:
        logging.error("alg_id is not legal. alg_id=%s" % alg_id)
        return False
    env_value = os.environ.get('MX_SDK_HOME')
    if not env_value:
        logging.error("Environment variable MX_SDK_HOME is not set!")
        raise Exception("Environment variable MX_SDK_HOME is not set!")
    _load_dll(os.path.join(env_value.split("=")[-1].strip("\""), 'lib', 'libkmcext.so'))
    KMC_DLL.KeSetLoggerCallback(_logger_callback)
    KMC_DLL.KeSetLoggerLevel(LogLevel.LOG_WARN)
    primary_key_store_file = os.path.realpath(primary_key_store_file)
    standby_key_store_file = os.path.realpath(standby_key_store_file)
    kmc_config = KmcConfig(
        primary_key_store_file=primary_key_store_file.encode(),
        standby_key_store_file=standby_key_store_file.encode(),
        domain_count=domain_count,
        role=KmcRole.MASTER,
        proc_lock_perm=0o0600,
        sdp_alg_id=alg_id,
        hmac_alg_id=HmacAlgorithm.HMAC_SHA256,
        sem_key=0x20161516
    )
    ret = KMC_DLL.KeInitialize(ctypes.byref(kmc_config))
    logging.info("KeInitialize ret=%d" % ret)
    if ret != OK:
        logging.error("KeInitialized Failed. ret=%d" % ret)
        return False

    return True


def _write_data(file_path: str, value: int, is_random: bool):
    if not is_random and (value < 0 or value > 255):
        logging.error("the input value not in byte range!")
        raise Exception("the input value not in byte range!")
    os.chmod(file_path, 0o600)
    with os.fdopen(os.open(file_path, os.O_RDWR | os.O_CREAT, 0o600), 'a+b') as file:
        block_size = 1024
        file_length = file.tell()
        file.seek(0, 0)
        block_count = file_length // block_size
        if block_count > MAX_BLOCK_SIZE:
            logging.error("the file size exceed the limit!")
            raise Exception("the file size exceed the limit!")
        data_res = file_length % block_size
        for _ in range(block_count):
            if is_random:
                file.write(secrets.token_bytes(nbytes=block_size))
            else:
                file.write(bytes(bytearray([value for _ in range(block_size)])))
        if data_res > 0:
            if is_random:
                file.write(secrets.token_bytes(nbytes=data_res))
            else:
                file.write(bytes(bytearray([value for _ in range(data_res)])))

        file.flush()


def delete_dir_files(dir_path):
    filename_list = os.listdir(dir_path)
    for filename in filename_list:
        file_path = os.path.join(dir_path, filename)
        if os.path.isfile(file_path) and file_path.endswith(('.key', '.ks')):
            _write_data(file_path, 0, False)
            _write_data(file_path, 1, False)
            _write_data(file_path, 0, True)
        os.remove(file_path)


def create_context(ssl_options):
    ca_path = ssl_options.ca_crt_file_path
    crt_path = ssl_options.server_crt_file_path
    key_path = ssl_options.server_key_file_path
    key_mm = ssl_options.server_key_mm
    crl = ssl_options.ca_crl_file_path
    sdp_alg_id = ssl_options.sdp_algorithm_id
    ciphers = ssl_options.cipher_list
    # Make sure ssl certificate file exist
    ca_file_list = (ca_path, crt_path, key_path)
    for file in ca_file_list:
        if file and os.path.exists(file):
            continue
        else:
            logging.error(f"SSL Certificate files does not exist! Please check config.yaml and cert file.")
            raise FileNotFoundError
    context = ssl.SSLContext(ssl.PROTOCOL_TLSv1_2)
    context.verify_mode = ssl.CERT_REQUIRED
    context.set_ciphers(ciphers)
    primary_key_store_file = "./keys/kmc.ks"
    standby_key_store_file = "./keys/StandbyKsf.ks"
    file_base_check(primary_key_store_file)
    file_base_check(standby_key_store_file)
    ret = init(primary_key_store_file, standby_key_store_file, sdp_alg_id)
    if ret is False:
        logging.error("kmc_bak init error.")
        raise Exception('ERROR: kmc_bak init failed!')

    domain_id = 0
    decrypt_mm = decrypt(domain_id, key_mm)
    if decrypt_mm == "":
        logging.error("kmc_bak decrypt error.")
        raise Exception('ERROR: kmc_bak decrypt failed!')

    check_and_update_mk(domain_id, ADVANCE_DAY)
    finalize()
    context.load_cert_chain(crt_path, key_path, password=decrypt_mm)
    context.load_verify_locations(ca_path)
    if crl:
        context.verify_flags = ssl.VERIFY_CRL_CHECK_LEAF
        context.load_verify_locations(crl)
    delete_dir_files("./keys/")
    return context

ssl_context = None
if "HTTPS" in server_option_instance.ip_addr_port:
    ssl_context = create_context(server_option_instance)
