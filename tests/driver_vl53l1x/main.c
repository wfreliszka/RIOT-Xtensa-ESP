/*
 * Copyright (C) 2018 Gunar Schorcht
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     tests
 * @brief       Test application for ST VL53L1X Time-of-Flight distance sensor
 * @author      Gunar Schorcht <gunar@schorcht.net>
 * @file
 *
 * The test application demonstrates the usage of different functions
 * dependent on the used driver variant:
 *
 * - module vl53l1x         standard driver with most functionality is used
 * - module vl53l1x_st_api  ST API driver with complete functionality is used
 * - module vl53l1x_basic   basic driver with only very basic functionality is
 *                          used
 *
 * What driver variant is used can be defined at make command line. By default,
 * the standard driver variant vl53l1x is used:
 *
 *     make flash -C tests/driver_vl53l1x BOARD=...
 *
 * To use other driver variants, module vl53l1x_st_api or module vl53l1x_basic
 * have to be specified at make command line
 *
 *     USEMODULE=vl53l1x_st_api make flash -C tests/driver_vl53l1x BOARD=...
 *
 * or
 *
 *     USEMODULE=vl53l1x_basic make flash -C tests/driver_vl53l1x BOARD=...
 *
 * If the configuration parameter VL53L1X_PARAM_PIN_INT is defined, interrupts
 * are used to get data instead of polling for new data. In the case of driver
 * variant vl53l1x_st_api, threshold interrupts are configured. Otherwise
 * only data interrupts are used.
 *
 * In all cases, the sensor is configured with following default parameters:
 *
 * - timing budget of 50 ms
 * - intermeasurement period of 100 ms
 */

#include <stdio.h>

#include "thread.h"
#include "xtimer.h"

#include "vl53l1x.h"
#include "vl53l1x_params.h"

/* reference to main thread */
kernel_pid_t p_main;

static void isr (void *arg)
{
    (void)arg;
    /*
     * The ISR function is executed in the interrupt context. It must not be
     * blocking or time-consuming and must not access the sensor directly
     * via I2C.
     *
     * Therefore, the ISR function only indicates to the waiting thread that
     * an interrupt has occurred which needs to be handled in the thread
     * context.
     */
    msg_t msg;
    msg_send(&msg, p_main);
}

int main(void)
{
    /* Initialize the sensor */
    vl53l1x_t dev;

    /* initialize the sensor  */
    puts("VL53L1X Time-of-Flight distance sensor\n");
    puts("Initializing VL53L1X sensor");

    if (vl53l1x_init(&dev, &vl53l1x_params[0]) == VL53L1X_OK) {
        printf("[OK]\n");
    }
    else {
        printf("[Failed]\n");
        return 1;
    }

    #if !MODULE_VL53L1X_BASIC
    /* optional read and print the ROI */
    vl53l1x_roi_t roi;
    if (vl53l1x_get_roi(&dev, &roi) == VL53L1X_OK) {
        printf("old ROI top left [%d, %d], ROI bottom right [%d, %d]\n",
               roi.x_tl, roi.y_tl, roi.x_br, roi.y_br);
    }
    roi.x_tl = 4;
    roi.y_tl = 12;
    roi.x_br = 12;
    roi.y_br = 4;
    if (vl53l1x_set_roi(&dev, &roi) == VL53L1X_OK &&
        vl53l1x_get_roi(&dev, &roi) == VL53L1X_OK) {
        printf("new ROI top left [%d, %d], ROI bottom right [%d, %d]\n",
               roi.x_tl, roi.y_tl, roi.x_br, roi.y_br);
    }
    #endif /* !MODULE_VL53L1X_BASIC */

    /* save the main thread pid */
    p_main = sched_active_pid;

    /* if interrupt pin is defined, enable the interrupt */
    if (vl53l1x_params[0].pin_int != GPIO_UNDEF) {
        #if MODULE_VL53L1X_ST_API
        /* generate interrrupts when distance is between 200 mm and 400 mm */
        vl53l1x_int_config_t cfg = {
            .mode = VL53L1X_INT_DIST,
            .dist_mode = VL53L1X_THRESH_IN,
            .dist_high = 400,
            .dist_low  = 200,
        };
        vl53l1x_int_config(&dev, &cfg, isr, 0);
        #else
        vl53l1x_int_config(&dev, 0, isr, 0);
        #endif /* MODULE_VL53L1X_ST_API */
    }

    while (1) {
        /* if interrupt pin is defined, wait for the interrupt */
        if (vl53l1x_params[0].pin_int != GPIO_UNDEF) {
            msg_t msg;
            msg_receive(&msg);
        }
        else {
            /* otherwise wait 100 ms. */
            xtimer_usleep(100 * US_PER_MS);
        }

        #if MODULE_VL53L1X_BASIC
        int16_t mm;

        if (vl53l1x_data_ready(&dev) == VL53L1X_OK &&
            vl53l1x_read_mm(&dev, &mm) == VL53L1X_OK) {

            printf("distance=%d [mm]\n", mm);
        }
        #else
        vl53l1x_data_t data;

        if (vl53l1x_data_ready(&dev) == VL53L1X_OK &&
            vl53l1x_read_data(&dev, &data) == VL53L1X_OK) {

            printf("distance=%d [mm] status=%u "
                   "signal=%u.%03u [mcps] ambient=%u.%03u [mcps]\n",
                   data.distance, data.status,
                   data.signal_rate >> 16,
                   (unsigned)((data.signal_rate & 0xffff)/65.535),
                   data.ambient_rate >> 16,
                   (unsigned)((data.ambient_rate & 0xffff)/65.535));
        }
        #endif /* MODULE_VL53L1X_BASIC */
    }

    return 0;
}
