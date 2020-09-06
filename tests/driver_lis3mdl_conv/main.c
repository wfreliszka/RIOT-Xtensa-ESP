
/*
 * Copyright (C) 2015 HAW Hamburg
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
 * @brief       Test application for the LIS3MDL 3-axis magnetometer
 *
 * @author      René Herthel <rene-herthel@outlook.de>
 *
 * @}
 */

#include <stdio.h>

#include "xtimer.h"
#include "lis3mdl.h"
#include "lis3mdl_params.h"
#include <math.h>
#define SLEEP_USEC  (800 * 800U)
#define M_PI 3.14159265358979323846
//const float 0.48828125f;

int main(void)
{
    lis3mdl_t dev;

    xtimer_sleep(1);

    puts("LIS3MDL test application");
    puts("Initializing LIS3MDL sensor");

    if (lis3mdl_init(&dev, &lis3mdl_params[0]) == 0) {
        puts("[ OK ]\n");
    }
    else {
        puts("[ FAIL ]\n");
        return 1;
    }

    while(1) {
        lis3mdl_3d_data_t mag_value;
        lis3mdl_read_mag(&dev, &mag_value);
        printf("Magnetometer [G]:\tX: %2d\tY: %2d\tZ: %2d\n",
               mag_value.x_axis,
               mag_value.y_axis,
               mag_value.z_axis);

        int16_t temp_value;
        lis3mdl_read_temp(&dev, &temp_value);
        printf("Temperature:\t\t%i°C\n", temp_value);


        float heading = atan2(mag_value.y_axis, mag_value.x_axis) * 180.f / M_PI;

        printf("Heading: %d °\n", (int)heading);

        xtimer_usleep(SLEEP_USEC);
    }

    return 0;
}
