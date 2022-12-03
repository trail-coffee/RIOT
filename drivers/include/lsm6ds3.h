/*
 * Copyright (C) 2017 OTA keys S.A.
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 *
 */

/**
 * @defgroup    drivers_lsm6ds3 LSM6DS3 3D accelerometer/gyroscope
 * @ingroup     drivers_sensors
 * @ingroup     drivers_saul
 * @brief       Device driver for the LSM6DS3 3D accelerometer/gyroscope
 *
 * This driver provides @ref drivers_saul capabilities.
 * @{
 *
 * @file
 * @brief       Device driver interface for the LSM6DS3 3D accelerometer/gyroscope.
 *
 * @author      Vincent Dupont <vincent@otakeys.com>
 * @author      Sebastian Meiling <s@mlng.net>
 *
 */

#ifndef LSM6DS3_H
#define LSM6DS3_H

#ifdef __cplusplus
extern "C" {
#endif

#include "periph/i2c.h"

/**
 * @brief   Data rate settings
 */
enum {
    LSM6DS3_DATA_RATE_POWER_DOWN = 0x0,
    LSM6DS3_DATA_RATE_1_6HZ      = 0xB,
    LSM6DS3_DATA_RATE_12_5HZ     = 0x1,
    LSM6DS3_DATA_RATE_26HZ       = 0x2,
    LSM6DS3_DATA_RATE_52HZ       = 0x3,
    LSM6DS3_DATA_RATE_104HZ      = 0x4,
    LSM6DS3_DATA_RATE_208HZ      = 0x5,
    LSM6DS3_DATA_RATE_416HZ      = 0x6,
    LSM6DS3_DATA_RATE_833HZ      = 0x7,
    LSM6DS3_DATA_RATE_1_66KHZ    = 0x8,
    LSM6DS3_DATA_RATE_3_33KHZ    = 0x9,
    LSM6DS3_DATA_RATE_6_66KHZ    = 0xa,
};

/**
 * @brief   Decimation settings
 */
enum {
    LSM6DS3_DECIMATION_NOT_IN_FIFO = 0,
    LSM6DS3_DECIMATION_NO,
    LSM6DS3_DECIMATION_2,
    LSM6DS3_DECIMATION_3,
    LSM6DS3_DECIMATION_4,
    LSM6DS3_DECIMATION_8,
    LSM6DS3_DECIMATION_16,
    LSM6DS3_DECIMATION_32,
};

/**
 * @brief   Accelerometer full scale
 */
enum {
    LSM6DS3_ACC_FS_2G  = 0,
    LSM6DS3_ACC_FS_16G,
    LSM6DS3_ACC_FS_4G,
    LSM6DS3_ACC_FS_8G,
    LSM6DS3_ACC_FS_MAX,
};

/**
 * @brief   Gyroscope full scale
 */
enum {
    LSM6DS3_GYRO_FS_245DPS    = 0,
    LSM6DS3_GYRO_FS_500DPS,
    LSM6DS3_GYRO_FS_1000DPS,
    LSM6DS3_GYRO_FS_2000DPS,
    LSM6DS3_GYRO_FS_MAX,
};

/**
 * @brief   LSM6DS3 driver parameters
 */
typedef struct {
    i2c_t i2c;                  /**< i2c bus */
    uint8_t addr;               /**< i2c address */
    uint8_t acc_odr;            /**< accelerometer output data rate */
    uint8_t gyro_odr;           /**< gyroscope output data rate */
    uint8_t acc_fs;             /**< accelerometer full scale */
    uint8_t gyro_fs;            /**< gyroscope full scale */
    uint8_t acc_decimation;     /**< accelerometer decimation */
    uint8_t gyro_decimation;    /**< gyroscope decimation */
} lsm6ds3_params_t;

/**
 * @brief   LSM6DS3 device descriptor
 */
typedef struct {
    lsm6ds3_params_t params; /**< driver parameters */
} lsm6ds3_t;

/**
 * @brief   3D output data
 */
typedef struct {
    int16_t x;  /**< X axis */
    int16_t y;  /**< Y axis */
    int16_t z;  /**< Z axis */
} lsm6ds3_3d_data_t;

/**
 * @brief   Named return values
 */
enum {
    LSM6DS3_OK = 0,             /**< all good */
    LSM6DS3_ERROR_BUS,          /**< I2C bus error */
    LSM6DS3_ERROR_CNF,          /**< Config error */
    LSM6DS3_ERROR_DEV,          /**< device error */
};

/**
 * @brief   Initialize a LSM6DS3 device
 *
 * @param[out] dev     device to initialize
 * @param[in] params  driver parameters
 *
 * @return LSM6DS3_OK on success
 * @return < 0 on error
 */
int lsm6ds3_init(lsm6ds3_t *dev, const lsm6ds3_params_t *params);

/**
 * @brief   Read accelerometer data
 *
 * @param[in] dev    device to read
 * @param[out] data  accelerometer values
 *
 * @return LSM6DS3_OK on success
 * @return < 0 on error
 */
int lsm6ds3_read_acc(const lsm6ds3_t *dev, lsm6ds3_3d_data_t *data);

/**
 * @brief   Read gyroscope data
 *
 * @param[in] dev    device to read
 * @param[out] data  gyroscope values
 *
 * @return LSM6DS3_OK on success
 * @return < 0 on error
 */
int lsm6ds3_read_gyro(const lsm6ds3_t *dev, lsm6ds3_3d_data_t *data);

/**
 * @brief   Read temperature data
 *
 * @note To avoid floating point data types but still provide high resolution
 *       for temperature readings, resulting values are scale by factor 100.
 *
 *
 * @param[in] dev    device to read
 * @param[out] data  temperature value, in °C x 100
 *
 * @return LSM6DS3_OK on success
 * @return < 0 on error
 */
int lsm6ds3_read_temp(const lsm6ds3_t *dev, int16_t *data);

/**
 * @brief   Power down accelerometer
 *
 * @param[in] dev    device to power down
 *
 * @return LSM6DS3_OK on success
 * @return < 0 on error
 */
int lsm6ds3_acc_power_down(const lsm6ds3_t *dev);

/**
 * @brief   Power down gyroscope
 *
 * @param[in] dev    device to power down
 *
 * @return LSM6DS3_OK on success
 * @return < 0 on error
 */
int lsm6ds3_gyro_power_down(const lsm6ds3_t *dev);

/**
 * @brief   Power up accelerometer
 *
 * @param[in] dev    device to power up
 *
 * @return LSM6DS3_OK on success
 * @return < 0 on error
 */
int lsm6ds3_acc_power_up(const lsm6ds3_t *dev);

/**
 * @brief   Power up gyroscope
 *
 * @param[in] dev    device to power up
 *
 * @return LSM6DS3_OK on success
 * @return < 0 on error
 */
int lsm6ds3_gyro_power_up(const lsm6ds3_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* LSM6DS3_H */
/** @} */
