/*
 * Copyright (C) 2019 Gunar Schorcht
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_periph_gpio
 * @{
 *
 * @file
 * @brief       Common GPIO driver functions/definitions
 *
 * @author      Gunar Schorcht <gunar@schorcht.net>
 * @}
 */

#ifdef MODULE_PERIPH_GPIO

#include "periph/gpio.h"

/* the CPU low level GPIO driver */
static const gpio_driver_t gpio_cpu_driver = {
                .init = gpio_cpu_init,
#ifdef MODULE_PERIPH_GPIO_IRQ
                .init_int = gpio_cpu_init_int,
                .irq_enable = gpio_cpu_irq_enable,
                .irq_disable = gpio_cpu_irq_disable,
#endif /* MODULE_PERIPH_GPIO_IRQ */
                .read = gpio_cpu_read,
                .set = gpio_cpu_set,
                .clear = gpio_cpu_clear,
                .toggle = gpio_cpu_toggle,
                .write = gpio_cpu_write,
};

/* the CPU low level GPIO device */
const gpio_dev_t gpio_cpu_dev = {
    .driver = &gpio_cpu_driver,
    .dev = NULL,
};

#endif /* MODULE_PERIPH_GPIO */
