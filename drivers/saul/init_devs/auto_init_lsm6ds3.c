/*
 * Copyright (C) 2017 OTA keys S.A.
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 *
 */

/*
 * @ingroup     sys_auto_init_saul
 * @{
 *
 * @file
 * @brief       Auto initialization of LSM6DS3 accelerometer/gyroscope sensors
 *
 * @author      Vincent Dupont <vincent@otakeys.com
 *
 * @}
 */

#include "assert.h"
#include "log.h"
#include "saul_reg.h"
#include "lsm6ds3.h"
#include "lsm6ds3_params.h"

/**
 * @brief   Define the number of configured sensors
 */
#define LSM6DS3_NUM    ARRAY_SIZE(lsm6ds3_params)

/**
 * @brief   Allocate memory for the device descriptors
 */
static lsm6ds3_t lsm6ds3_devs[LSM6DS3_NUM];

/**
 * @brief   Memory for the SAUL registry entries
 */
static saul_reg_t saul_entries[LSM6DS3_NUM * 3];

/**
 * @brief   Define the number of saul info
 */
#define LSM6DS3_INFO_NUM    ARRAY_SIZE(lsm6ds3_saul_info)

/**
 * @name    Reference the driver structs
 * @{
 */
extern saul_driver_t lsm6ds3_saul_acc_driver;
extern saul_driver_t lsm6ds3_saul_gyro_driver;
extern saul_driver_t lsm6ds3_saul_temp_driver;
/** @} */

void auto_init_lsm6ds3(void)
{
    assert(LSM6DS3_NUM == LSM6DS3_INFO_NUM);

    for (unsigned int i = 0; i < LSM6DS3_NUM; i++) {
        LOG_DEBUG("[auto_init_saul] initializing lsm6ds3 #%u\n", i);

        if (lsm6ds3_init(&lsm6ds3_devs[i], &lsm6ds3_params[i]) < 0) {
            LOG_ERROR("[auto_init_saul] error initializing lsm6ds3 #%u\n", i);
            continue;
        }

        saul_entries[(i * 3)].dev = &(lsm6ds3_devs[i]);
        saul_entries[(i * 3)].name = lsm6ds3_saul_info[i].name;
        saul_entries[(i * 3)].driver = &lsm6ds3_saul_acc_driver;
        saul_reg_add(&(saul_entries[(i * 3)]));
        saul_entries[(i * 3) + 1].dev = &(lsm6ds3_devs[i]);
        saul_entries[(i * 3) + 1].name = lsm6ds3_saul_info[i+1].name;
        saul_entries[(i * 3) + 1].driver = &lsm6ds3_saul_gyro_driver;
        saul_reg_add(&(saul_entries[(i * 3) + 1]));
        saul_entries[(i * 3) + 2].dev = &(lsm6ds3_devs[i]);
        saul_entries[(i * 3) + 2].name = lsm6ds3_saul_info[i+2].name;
        saul_entries[(i * 3) + 2].driver = &lsm6ds3_saul_temp_driver;
        saul_reg_add(&(saul_entries[(i * 3) + 2]));
    }
}
