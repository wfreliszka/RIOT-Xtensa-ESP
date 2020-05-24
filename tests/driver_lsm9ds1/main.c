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
 * @brief       Test application for the LSM9DS1 accelerometer/gyroscope driver.
 *
 * @author      Vincent Dupont <vincent@otakeys.com>
 * @author      Sebastian Meiling <s@mlng.net>
 *
 * @}
 */

#include <stdio.h>

#include "xtimer.h"
#include "lsm9ds1.h"
#include "lsm9ds1_params.h"

#define SLEEP_USEC  (500UL * US_PER_MS)

int main(void)
{
    lsm9ds1_t dev;
    int16_t temp_value;
    lsm9ds1_3d_data_t mag_value;
    lsm9ds1_3d_data_t acc_value;
    xtimer_sleep(1);

    puts("LSM9DS1 test application");
    printf("Initializing LSM9DS1 sensor at I2C_%i... ", lsm9ds1_params->i2c);

    if (lsm9ds1_init(&dev, lsm9ds1_params) != LSM9DS1_OK) {
        puts("[ERROR]");
        return 1;
    }
    puts("[SUCCESS]\n");

    puts("Powering down LSM9DS1 sensor...");
    if (lsm9ds1_acc_power_down(&dev) != LSM9DS1_OK) {
        puts("[ERROR]");
        return 1;
    }
    if (lsm9ds1_gyro_power_down(&dev) != LSM9DS1_OK) {
        puts("[ERROR]");
        return 1;
    }
    puts("[SUCCESS]\n");

    xtimer_sleep(1);

    puts("Powering up LSM9DS1 sensor...");
    if (lsm9ds1_acc_power_up(&dev) != LSM9DS1_OK) {
        puts("[ERROR]");
        return 1;
    }
    if (lsm9ds1_gyro_power_up(&dev) != LSM9DS1_OK) {
        puts("[ERROR]");
        return 1;
    }
    puts("[SUCCESS]\n");

    while (1) {
        if (lsm9ds1_read_gyro(&dev, &mag_value) == LSM9DS1_OK) {
            printf("Gyroscope x: %i y: %i z: %i\n", mag_value.x,
                                                    mag_value.y,
                                                    mag_value.z);
        }
        else {
            puts("[ERROR] reading gyroscope!\n");
        }

        if (lsm9ds1_read_acc(&dev, &acc_value) == LSM9DS1_OK) {
            printf("Accelerometer x: %i y: %i z: %i\n", acc_value.x,
                                                        acc_value.y,
                                                        acc_value.z);
        }
        else {
            puts("[ERROR] reading accelerometer!\n");
        }

        

        if (lsm9ds1_read_temp(&dev, &temp_value) == LSM9DS1_OK) {
            printf("Temperature [in °C x 100]: %i \n", temp_value);
        }
        else {
            puts("[ERROR] reading temperature!\n");
        }

        puts("");
        xtimer_sleep(1);
    }

    return 0;
}

