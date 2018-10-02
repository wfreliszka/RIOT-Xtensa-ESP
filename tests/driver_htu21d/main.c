/*
 * Copyright (C) 2016 Bas Stottelaar
 * Copyright (C) 2018 Gunar Schorcht
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     tests
 * @brief       Test application for HTU21D humidity and temperature sensor
 * @author      Bas Stottelaar <basstottelaar@gmail.com>
 * @author      Gunar Schorcht <gunar@schorcht.net>
 * @file
 *
 * @note The HTU21D driver is just a wrapper for the driver for the compatible
 * SI7021 Humidity and Temperature Sensor. Please refer @ref drivers_si70xx
 * for details about the driver.
 */

#include <stdio.h>

#include "xtimer.h"

#include "htu21d_params.h"
#include "htu21d.h"

int main(void)
{
    htu21d_t dev;

    puts("HTU21D temperature and humidity sensor test application");

    /* initialize the sensor */
    printf("Initializing sensor... ");

    if (htu21d_init(&dev, &htu21d_params[0]) == 0) {
        puts("[OK]");
    }
    else {
        puts("[Failed]");
        return 1;
    }

    /* read temperature and humidity every 1 seconds */
    bool both = false;

    int16_t temperature;
    uint16_t humidity;

    while (1) {
        /* rotate the way of getting the data */
        if (both) {
            htu21d_get_both(&dev, &humidity, &temperature);
        }
        else {
            temperature = htu21d_get_temperature(&dev);
            humidity = htu21d_get_relative_humidity(&dev);
        }

        both = !both;

        /* display results */
        printf("relative humidity: %d.%02d\n", humidity / 100, humidity % 100);
        printf("temperature: %d.%02d C\n", temperature / 100, temperature % 100);

        /* sleep between measurements */
        xtimer_usleep(1000 * US_PER_MS);
    }

    return 0;
}
