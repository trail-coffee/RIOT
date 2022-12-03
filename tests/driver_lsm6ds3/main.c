/*
 * Copyright (C) 2017 OTA keys S.A.
 *               2017 HAW Hamburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup tests
 * @{
 *
 * @file
 * @brief       Test application for the LSM6DS3 accelerometer/gyroscope driver.
 *
 * @author      Vincent Dupont <vincent@otakeys.com>
 * @author      Sebastian Meiling <s@mlng.net>
 *
 * @}
 */

#include <stdio.h>

#include "ztimer.h"
#include "lsm6ds3.h"
#include "lsm6ds3_params.h"

#define SLEEP_MSEC  (500UL)

int main(void)
{
    lsm6ds3_t dev;
    int16_t temp_value;
    lsm6ds3_3d_data_t mag_value;
    lsm6ds3_3d_data_t acc_value;

    puts("LSM6DS3 test application");
    printf("Initializing LSM6DS3 sensor at I2C_%i... ", lsm6ds3_params->i2c);

    if (lsm6ds3_init(&dev, lsm6ds3_params) != LSM6DS3_OK) {
        puts("[ERROR]");
        return 1;
    }
    puts("[SUCCESS]\n");

    puts("Powering down LSM6DS3 sensor...");
    if (lsm6ds3_acc_power_down(&dev) != LSM6DS3_OK) {
        puts("[ERROR]");
        return 1;
    }
    if (lsm6ds3_gyro_power_down(&dev) != LSM6DS3_OK) {
        puts("[ERROR]");
        return 1;
    }
    puts("[SUCCESS]\n");

    ztimer_sleep(ZTIMER_MSEC, 1 * 1000);

    puts("Powering up LSM6DS3 sensor...");
    if (lsm6ds3_acc_power_up(&dev) != LSM6DS3_OK) {
        puts("[ERROR]");
        return 1;
    }
    if (lsm6ds3_gyro_power_up(&dev) != LSM6DS3_OK) {
        puts("[ERROR]");
        return 1;
    }
    puts("[SUCCESS]\n");

    while (1) {
        if (lsm6ds3_read_acc(&dev, &acc_value) == LSM6DS3_OK) {
            printf("Accelerometer x: %i y: %i z: %i\n", acc_value.x,
                                                        acc_value.y,
                                                        acc_value.z);
        }
        else {
            puts("[ERROR] reading accelerometer!\n");
        }

        if (lsm6ds3_read_gyro(&dev, &mag_value) == LSM6DS3_OK) {
            printf("Gyroscope x: %i y: %i z: %i\n", mag_value.x,
                                                    mag_value.y,
                                                    mag_value.z);
        }
        else {
            puts("[ERROR] reading gyroscope!\n");
        }

        if (lsm6ds3_read_temp(&dev, &temp_value) == LSM6DS3_OK) {
            printf("Temperature [in °C x 100]: %i \n", temp_value);
        }
        else {
            puts("[ERROR] reading temperature!\n");
        }

        puts("");
        ztimer_sleep(ZTIMER_MSEC, SLEEP_MSEC);
    }

    return 0;
}
