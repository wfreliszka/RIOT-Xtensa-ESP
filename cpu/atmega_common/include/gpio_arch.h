/*
 * Copyright (C) 2015 HAW Hamburg
 *               2016 Freie Universität Berlin
 *               2016 INRIA
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup         cpu_atmega_common
 * @{
 *
 * @file
 * @brief           CPU specific definitions for GPIOs
 *
 * @author          René Herthel <rene-herthel@outlook.de>
 * @author          Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author          Francisco Acosta <francisco.acosta@inria.fr>
 */

#ifndef GPIO_ARCH_H
#define GPIO_ARCH_H

#include "cpu.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef DOXYGEN
/**
 * @brief   Overwrite the default gpio_t type definition
 * @{
 */
#define HAVE_GPIO_PIN_T
typedef uint8_t gpio_pin_t;
/** @} */
#endif

/**
 * @brief   Definition of a fitting UNDEF value
 */
#define GPIO_UNDEF          (0xff)

#if 0
/**
 * @brief   Define a CPU specific GPIO pin generator macro
 */
#define GPIO_CPU_PIN(x, y)          ((x << 4) | y)
#endif

/**
 * @brief   Override the GPIO flanks
 *
 * This device has an additional mode in which the interrupt is triggered
 * when the pin is low.
 *
 * Enumeration order is important, do not modify.
 * @{
 */
#define HAVE_GPIO_FLANK_T
typedef enum {
    GPIO_LOW,          /**< emit interrupt when pin low */
    GPIO_BOTH,         /**< emit interrupt on both flanks */
    GPIO_FALLING,      /**< emit interrupt on falling flank */
    GPIO_RISING,       /**< emit interrupt on rising flank */
} gpio_flank_t;
/** @} */


#ifdef __cplusplus
}
#endif

#endif /* GPIO_ARCH_H */
/** @} */
