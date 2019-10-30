/*
 * Copyright (C) 2018 Acutam Automation, LLC
 *               2019 Gunar Schorcht
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_extend_gpio GPIO extension
 * @ingroup     drivers_extend
 * @brief       GPIO peripheral extension handling
 *
 * The GPIO extension interface makes handling of non-CPU GPIO devices invisible
 * to code using the GPIO peripheral API (periph/gpio.h).
 *
 * The following requirements have to be met in order to use the GPIO
 * extension interface
 *
 * - The board has to provide a list of GPIO extension devices
 *   of type @ref gpio_dev_t as configuration in file
 *   `boards/.../include/gpio_ext_conf.h`. Please refer gpio_ext_conf.h
 *   for more information and @ref gpio_ext as an example.
 *
 * - GPIO extension device drivers for GPIO extender hardware modules have to
 *   implement a set of functions that are compatible with the GPIO driver
 *   interface. Please refer @ref gpio_driver_t for more information.
 *
 * @{
 *
 * @file
 * @brief       GPIO extension interface definitions
 *
 * @author      Matthew Blue <matthew.blue.neuro@gmail.com>
 * @author      Gunar Schorcht <gunar@schorcht.net>
 */

#ifndef EXTEND_GPIO_H
#define EXTEND_GPIO_H

#include <limits.h>
#include <stdint.h>

#include "assert.h"
#include "periph_conf.h"
#include "periph/gpio.h"

#include "gpio_ext_conf.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Convert the (dev, num) tuple of a GPIO extension pin to
 *          corresponding @ref gpio_t value
 *
 * @param   dev     GPIO extension device number (index in the GPIO extension device list)
 * @param   num     Pin of the GPIO extension device
 */
#define GPIO_EXT_PIN(x, y)  ((gpio_t){ .dev = &gpio_ext_dev[x], .num = y } )

/**
 * @brief   Number of configured GPIO extension devices
 */
#define GPIO_EXT_DEV_NUMOF  (ARRAY_SIZE(gpio_ext_dev))

#ifdef __cplusplus
}
#endif

/** @} */

#endif /* EXTEND_GPIO_H */
/** @} */
