/*
 * Copyright (C) 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     boards_stm32f0discovery
 * @{
 *
 * @file
 * @brief       Board specific definitions for the STM32F0Discovery evaluation board.
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Sebastian Meiling <s@mlng.net>
 */

#ifndef BOARD_H
#define BOARD_H

#include "cpu.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name Macros for controlling the on-board LEDs.
 * @{
 */
#define LED0_PIN            GPIO_PIN(PORT_C, 9)
#define LED1_PIN            GPIO_PIN(PORT_C, 8)

#define LED_PORT            GPIOC
#define LED0_MASK           (1 << 9)
#define LED1_MASK           (1 << 8)

#define LED0_ON             (LED_PORT->BSRR = LED0_MASK)
#define LED0_OFF            (LED_PORT->BRR  = LED0_MASK)
#define LED0_TOGGLE         (LED_PORT->ODR ^= LED0_MASK)

#define LED1_ON             (LED_PORT->BSRR = LED1_MASK)
#define LED1_OFF            (LED_PORT->BRR  = LED1_MASK)
#define LED1_TOGGLE         (LED_PORT->ODR ^= LED1_MASK)
/** @} */

/**
 * @name User button
 * @{
 */
#define BTN0_PIN            GPIO_PIN(PORT_A, 0)
#define BTN0_MODE           GPIO_IN
/** @} */

/**
 * @brief Initialize board specific hardware, including clock, LEDs and std-IO
 */
void board_init(void);


#include "motor_driver.h"


#include "periph_cpu.h"

/**
 * @name Describe DC motors with PWM channel and GPIOs
 * @{
 */
static const motor_driver_config_t motor_driver_config[] = {
    {
        .pwm_dev         = 0,
        .mode            = MOTOR_DRIVER_2_DIRS,
        .mode_brake      = MOTOR_BRAKE_HIGH,
        .pwm_mode        = PWM_LEFT,
        .pwm_frequency   = 20000U,
        .pwm_resolution  = 2250U,
        .nb_motors       = 2,
        .motors          = {
            {
                .pwm_channel            = 0,
                .gpio_enable            = GPIO_UNDEF,
                .gpio_dir0              = GPIO_PIN(PORT_C, 12),
                .gpio_dir1_or_brake     = GPIO_PIN(PORT_C, 13),
                .gpio_dir_reverse       = 0,
                .gpio_enable_invert     = 0,
                .gpio_brake_invert      = 1,
            },
            {
                .pwm_channel            = 1,
                .gpio_enable            = GPIO_UNDEF,
                .gpio_dir0              = GPIO_PIN(PORT_C, 11),
                .gpio_dir1_or_brake     = GPIO_PIN(PORT_C, 10),
                .gpio_dir_reverse       = 0,
                .gpio_enable_invert     = 0,
                .gpio_brake_invert      = 1,
            },
        },
        .cb = NULL,
    },
};

#define MOTOR_DRIVER_NUMOF           ARRAY_SIZE(motor_driver_config)


#ifdef __cplusplus
}
#endif

#endif /* BOARD_H */
/** @} */
