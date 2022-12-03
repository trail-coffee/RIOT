/*
 * Copyright (C) 2017 OTA keys S.A.
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 *
 */

/**
 * @ingroup     drivers_lsm6ds3
 * @{
 *
 * @file
 * @brief       Default configuration for LSM6DS3 devices
 *
 * @author      Vincent Dupont <vincent@otakeys.com>
 *
 */

#ifndef LSM6DS3_PARAMS_H
#define LSM6DS3_PARAMS_H

#include "board.h"
#include "lsm6ds3.h"
#include "saul_reg.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    Set default configuration parameters
 * @{
 */
#ifndef LSM6DS3_PARAM_I2C
#define LSM6DS3_PARAM_I2C            I2C_DEV(0)
#endif
#ifndef LSM6DS3_PARAM_ADDR
#define LSM6DS3_PARAM_ADDR           (0x6A)
#endif
#ifndef LSM6DS3_PARAM_ACC_ODR
#define LSM6DS3_PARAM_ACC_ODR        (LSM6DS3_DATA_RATE_52HZ)
#endif
#ifndef LSM6DS3_PARAM_GYRO_ODR
#define LSM6DS3_PARAM_GYRO_ODR       (LSM6DS3_DATA_RATE_52HZ)
#endif
#ifndef LSM6DS3_PARAM_ACC_FS
#define LSM6DS3_PARAM_ACC_FS         (LSM6DS3_ACC_FS_2G)
#endif
#ifndef LSM6DS3_PARAM_GYRO_FS
#define LSM6DS3_PARAM_GYRO_FS        (LSM6DS3_GYRO_FS_245DPS)
#endif
#ifndef LSM6DS3_PARAM_ACC_FIFO_DEC
#define LSM6DS3_PARAM_ACC_FIFO_DEC   (LSM6DS3_DECIMATION_NO)
#endif
#ifndef LSM6DS3_PARAM_GYRO_FIFO_DEC
#define LSM6DS3_PARAM_GYRO_FIFO_DEC  (LSM6DS3_DECIMATION_NO)
#endif

#ifndef LSM6DS3_PARAMS
#define LSM6DS3_PARAMS               { .i2c             = LSM6DS3_PARAM_I2C,          \
                                       .addr            = LSM6DS3_PARAM_ADDR,         \
                                       .acc_odr         = LSM6DS3_PARAM_ACC_ODR,      \
                                       .gyro_odr        = LSM6DS3_PARAM_GYRO_ODR,     \
                                       .acc_fs          = LSM6DS3_PARAM_ACC_FS,       \
                                       .gyro_fs         = LSM6DS3_PARAM_GYRO_FS,      \
                                       .acc_decimation  = LSM6DS3_PARAM_ACC_FIFO_DEC, \
                                       .gyro_decimation = LSM6DS3_PARAM_GYRO_FIFO_DEC }
#endif
#ifndef LSM6DS3_SAUL_INFO
#define LSM6DS3_SAUL_INFO            { .name = "lsm6ds3" }
#endif
/** @} */

/**
 * @brief   Allocate some memory to store the actual configuration
 */
static const lsm6ds3_params_t lsm6ds3_params[] =
{
    LSM6DS3_PARAMS
};

/**
 * @brief   Additional meta information to keep in the SAUL registry
 */
static const saul_reg_info_t lsm6ds3_saul_info[] =
{
    LSM6DS3_SAUL_INFO
};

#ifdef __cplusplus
}
#endif

#endif /* LSM6DS3_PARAMS_H */
/** @} */
