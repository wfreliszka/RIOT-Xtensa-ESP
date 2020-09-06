/*
 * Copyright (C) 2020 Wojtek Freliszka
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 *
 */

/**
 * @ingroup     drivers_hcsr0x
 * @{
 *
 * @file
 * @brief       Device driver implementation for the HCSR0X sonic distance meter
 *
 * @author      Wojtek Freliszka
 *
 * @}
 */

#ifndef HCSR0X_H
#define HCSR0X_H

#ifdef __cplusplus
extern "C" {
#endif

#include "periph/gpio.h"

struct hcsr0x_t;



/**
 * @brief   Signature of event callback functions triggered from interrupts
 *
 * @param[in] dev       optional context for the callback
 * @param[in] milimeters_distance       distance in milimeters or error code if negative
 */
typedef void (*hcsr0x_cb_t)(const void *dev,int16_t milimeters_distance,int32_t payload);

/**
 * @brief   HCSR0X driver parameters
 */
typedef struct {
    hcsr0x_cb_t callback;
    uint32_t start_time;
    gpio_t trigger_pin;
    gpio_t echo_pin;
    int32_t payload;

} hcsr0x_params_t;

/**
 * @brief   HCSR0X device descriptor
 */
typedef struct {
    hcsr0x_params_t params; /**< driver parameters */
} hcsr0x_t;

/**
 * @brief   Named return values
 */
enum {
    HCSR0X_OK = 0,             /**< all good */
    HCSR0X_TRIGGER_PIN_ERR = -1,          /**< cannot use trigger pin as output */
    HCSR0X_ECHO_PIN_ERR = -2,          /**< cannot use echo pin as inpit */
    HCSR0X_ECHO_DURATION_INVALID = -3,          /**< cannot use echo pin as inpit */
};

/**
 * @brief   Initialize a HCSR0X device
 *
 * @param[out] dev     device to initialize
 * @param[in] cb    our callback with results
 * @param[in] trigger_pin   a gpio pin to trigger measurement
 * @param[in] echo_pin  a gpio pin for echo
 *
 * @return HCSR0X_OK on success
 * @return < 0 on error
 */
int hcsr0x_init(hcsr0x_t *dev, hcsr0x_cb_t cb, gpio_t trigger_pin,gpio_t echo_pin,bool trigger_reused,int32_t payload);


/**
 * @brief   Initiates measure action on a HCSR0X device
 *
 * @param[out] dev     device to initialize
 *
 * @return HCSR0X_OK on success, in general always returns success
 * @return < 0 on error
 */
int hcsr0x_measure(hcsr0x_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* HCSR0X_H */
/** @} */
