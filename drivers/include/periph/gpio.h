/*
 * Copyright (C) 2015 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_periph_gpio GPIO
 * @ingroup     drivers_periph
 * @brief       Low-level GPIO peripheral driver
 *
 * This is a basic GPIO (General-purpose input/output) interface to allow
 * platform independent access to a MCU's input/output pins. This interface is
 * intentionally designed to be as simple as possible, to allow for easy
 * implementation and maximum portability.
 *
 * The interface provides capabilities to initialize a pin as output-,
 * input- and interrupt pin. With the API you can basically set/clear/toggle the
 * digital signal at the hardware pin when in output mode. Configured as input you can
 * read a digital value that is being applied to the pin externally. When initializing
 * an external interrupt pin, you can register a callback function that is executed
 * in interrupt context once the interrupt condition applies to the pin. Usually you
 * can react to rising or falling signal flanks (or both).
 *
 * In addition the API provides to set standard input/output circuit modes such as
 * e.g. internal push-pull configurations.
 *
 * All modern micro controllers organize their GPIOs in some form of ports,
 * often named 'PA', 'PB', 'PC'..., or 'P0', 'P1', 'P2'..., or similar. Each of
 * these ports is then assigned a number of pins, often 8, 16, or 32. A hardware
 * pin can thus be described by its port/pin tuple. To access a pin, the
 * @p GPIO_PIN(port, pin) macro should be used. For example: If your platform has
 * a pin PB22, it will be port=1 and pin=22. The @p GPIO_PIN macro should be
 * overridden by a MCU, to allow for efficient encoding of the the port/pin tuple.
 * For example, on many platforms it is possible to `OR` the pin number with the
 * corresponding ports base register address. This allows for efficient decoding
 * of pin number and base address without the need of any address lookup.
 *
 * In case the driver does not define it, the below macro definition is used to
 * simply map the port/pin tuple to the pin value. In that case, predefined GPIO
 * definitions in `RIOT/boards/ * /include/periph_conf.h` will define the selected
 * GPIO pin.
 *
 * # (Low-) Power Implications
 *
 * On almost all platforms, we can only control the peripheral power state of
 * full ports (i.e. groups of pins), but not for single GPIO pins. Together with
 * CPU specific alternate function handling for pins used by other peripheral
 * drivers, this can make it quite complex to keep track of pins that are
 * currently used at a certain moment. To simplify the implementations (and ease
 * the memory consumption), we expect ports to be powered on (e.g. through
 * peripheral clock gating) when first used and never be powered off again.
 *
 * GPIO driver implementations **should** power on the corresponding port during
 * gpio_init() and gpio_init_int().
 *
 * For external interrupts to work, some platforms may need to block certain
 * power modes (although this is not very likely). This should be done during
 * gpio_init_int().
 *
 * @{
 * @file
 * @brief       Low-level GPIO peripheral driver interface definitions
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 */

#ifndef PERIPH_GPIO_H
#define PERIPH_GPIO_H

#include <limits.h>

#include "gpio_arch.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef HAVE_GPIO_PIN_T
/**
 * @brief   GPIO pin type identifier
 */
typedef unsigned int gpio_pin_t;
#endif

/**
 * @brief   Convert (port, pin) tuple to @c gpio_cpu_t value
 */
/* Default GPIO macro maps port-pin tuples to the CPU pin value */
#ifndef GPIO_CPU_PIN
#define GPIO_CPU_PIN(x,y)   ((gpio_t){ .dev = &gpio_cpu_dev[x], .num = y } )
#endif

/**
 * @brief   GPIO pin not defined for CPU pins
 */
#ifndef GPIO_CPU_UNDEF
#define GPIO_CPU_UNDEF      ((gpio_pin_t)(UINT_MAX))
#endif

/**
 * @brief   GPIO pin not defined for all types of pins
 *
 * In case the GPIO extension API is used, it defines the GPIO pin as not
 * defined.
 */
#ifndef GPIO_UNDEF
#define GPIO_UNDEF          { .dev = NULL, .num = GPIO_CPU_UNDEF }
#endif

/**
 * @brief   Convert (port, pin) tuple to gpio_t structure
 *
 * @note At the moment it simply maps to a CPU GPIO.
 */
#ifndef GPIO_PIN
#define GPIO_PIN(x, y)      (GPIO_CPU_PIN(x,y))
#endif

/**
 * @brief   Available pin modes
 *
 * Generally, a pin can be configured to be input or output. In output mode, a
 * pin can further be put into push-pull or open drain configuration. Though
 * this is supported by most platforms, this is not always the case, so driver
 * implementations may return an error code if a mode is not supported.
 */
#ifndef HAVE_GPIO_MODE_T
typedef enum {
    GPIO_IN ,               /**< configure as input without pull resistor */
    GPIO_IN_PD,             /**< configure as input with pull-down resistor */
    GPIO_IN_PU,             /**< configure as input with pull-up resistor */
    GPIO_OUT,               /**< configure as output in push-pull mode */
    GPIO_OD,                /**< configure as output in open-drain mode without
                             *   pull resistor */
    GPIO_OD_PU              /**< configure as output in open-drain mode with
                             *   pull resistor enabled */
} gpio_mode_t;
#endif

/**
 * @brief   Definition of possible active flanks for external interrupt mode
 */
#ifndef HAVE_GPIO_FLANK_T
typedef enum {
    GPIO_FALLING = 0,       /**< emit interrupt on falling flank */
    GPIO_RISING = 1,        /**< emit interrupt on rising flank */
    GPIO_BOTH = 2           /**< emit interrupt on both flanks */
} gpio_flank_t;
#endif

/**
 * @brief   Signature of event callback functions triggered from interrupts
 *
 * @param[in] arg       optional context for the callback
 */
typedef void (*gpio_cb_t)(void *arg);

/**
 * @brief   Default interrupt context for GPIO pins
 */
#ifndef HAVE_GPIO_ISR_CTX_T
typedef struct {
    gpio_cb_t cb;           /**< interrupt callback */
    void *arg;              /**< optional argument */
} gpio_isr_ctx_t;
#endif

/**
 * @brief   GPIO device driver type
 *
 * Defines a structure to hold the driver interface to function mapping.
 *
 * TODO Document
 */
typedef struct gpio_driver {
    /**
     * @brief   Callback typedef for gpio_init
     *
     * @see @ref #gpio_init
     */
    int (*init)(void *dev, gpio_pin_t pin, gpio_mode_t mode);

#ifdef MODULE_PERIPH_GPIO_IRQ

    /**
     * @brief   Callback typedef for gpio_init_int
     *
     * @see @ref #gpio_init_int
     */
    int (*init_int)(void *dev, gpio_pin_t pin, gpio_mode_t mode,
                    gpio_flank_t flank, gpio_cb_t cb, void *arg);

    /**
     * @brief   Callback typedef for gpio_irq_enable
     *
     * @see @ref #gpio_irq_enable
     */
    void (*irq_enable)(void *dev, gpio_pin_t pin);

    /**
     * @brief   Callback typedef for gpio_irq_disable
     *
     * @see @ref #gpio_irq_disable
     */
    void (*irq_disable)(void *dev, gpio_pin_t pin);

#endif /* MODULE_PERIPH_GPIO_IRQ */

    /**
     * @brief   Callback typedef for gpio_read
     *
     * @see @ref #gpio_read
     */
    int (*read)(void *dev, gpio_pin_t pin);

    /**
     * @brief   Callback typedef for gpio_set
     *
     * @see @ref #gpio_set
     */
    void (*set)(void *dev, gpio_pin_t pin);

    /**
     * @brief   Callback typedef for gpio_clear
     *
     * @see @ref #gpio_clear
     */
    void (*clear)(void *dev, gpio_pin_t pin);

    /**
     * @brief   Callback typedef for gpio_toggle
     *
     * @see @ref #gpio_toggle
     */
    void (*toggle)(void *dev, gpio_pin_t pin);

    /**
     * @brief   Callback typedef for gpio_write
     *
     * @see @ref #gpio_write
     */
    void (*write)(void *dev, gpio_pin_t pin, int value);

} gpio_driver_t;

/**
 * @brief   GPIO device type
 * TODO Document
 */
typedef struct gpio_dev {
    const gpio_driver_t *driver;    /**< pointer to the GPIO device driver */
    void *dev;                      /**< pointer to the GPIO device descriptor, NULL in case of MCU */
} gpio_dev_t;

/**
 * @brief   GPIO type definition
 * TODO Document
 * If GPIO extension API is enabled by module `extend_gpio`, the GPIO type
 * definition is a structure that is used for CPU GPIO pins as well
 * as GPIO extension pins.
 */
typedef struct {
    const gpio_dev_t *dev;  /**< pointer to the device of the GPIO */
    gpio_pin_t num;         /**< pin number */
} gpio_t;

/** TODO Document */
extern const gpio_driver_t gpio_cpu_driver;

#ifndef DOXYGEN
/** TODO Document */
/**
 * @name    Low-level versions of the GPIO functions
 *
 * These are for CPU implementation in `cpu/.../periph/gpio.c` and should not
 * be called directly.
 * @{
 */
int  gpio_cpu_init(void *dev, gpio_pin_t pin, gpio_mode_t mode);
int  gpio_cpu_init_int(void *dev, gpio_pin_t pin, gpio_mode_t mode,
                       gpio_flank_t flank, gpio_cb_t cb, void *arg);
void gpio_cpu_irq_enable(void *dev, gpio_pin_t pin);
void gpio_cpu_irq_disable(void *dev, gpio_pin_t pin);
int  gpio_cpu_read(void *dev, gpio_pin_t pin);
void gpio_cpu_set(void *dev, gpio_pin_t pin);
void gpio_cpu_clear(void *dev, gpio_pin_t pin);
void gpio_cpu_toggle(void *dev, gpio_pin_t pin);
void gpio_cpu_write(void *dev, gpio_pin_t pin, int value);
/** @} */
#endif /* DOXYGEN */

/**
 * @brief   Initialize the given pin as general purpose input or output
 *
 * When configured as output, the pin state after initialization is undefined.
 * The output pin's state **should** be untouched during the initialization.
 * This behavior can however **not be guaranteed** by every platform.
 *
 * @param[in] pin       pin to initialize
 * @param[in] mode      mode of the pin, see @c gpio_mode_t
 *
 * @return              0 on success
 * @return              -1 on error
 */
static inline int gpio_init(gpio_t pin, gpio_mode_t mode)
{
    return pin.dev->driver->init(pin.dev->dev, pin.num, mode);
}

#if defined(MODULE_PERIPH_GPIO_IRQ) || defined(DOXYGEN)
/**
 * @brief   Initialize a GPIO pin for external interrupt usage
 *
 * The registered callback function will be called in interrupt context every
 * time the defined flank(s) are detected.
 *
 * The interrupt is activated automatically after the initialization.
 *
 * @note    You have to add the module `periph_gpio_irq` to your project to
 *          enable this function
 *
 * @param[in] pin       pin to initialize
 * @param[in] mode      mode of the pin, see @c gpio_mode_t
 * @param[in] flank     define the active flank(s)
 * @param[in] cb        callback that is called from interrupt context
 * @param[in] arg       optional argument passed to the callback
 *
 * @return              0 on success
 * @return              -1 on error
 */
static inline int gpio_init_int(gpio_t pin, gpio_mode_t mode, gpio_flank_t flank,
                                gpio_cb_t cb, void *arg)
{
    return pin.dev->driver->init_int(pin.dev->dev, pin.num, mode, flank, cb, arg);
}

/**
 * @brief   Enable pin interrupt if configured as interrupt source
 *
 * @note    You have to add the module `periph_gpio_irq` to your project to
 *          enable this function
 *
 * @param[in] pin       the pin to enable the interrupt for
 */
static inline void gpio_irq_enable(gpio_t pin)
{
    pin.dev->driver->irq_enable(pin.dev->dev, pin.num);
}

/**
 * @brief   Disable the pin interrupt if configured as interrupt source
 *
 * @note    You have to add the module `periph_gpio_irq` to your project to
 *          enable this function
 *
 * @param[in] pin       the pin to disable the interrupt for
 */
static inline void gpio_irq_disable(gpio_t pin)
{
    pin.dev->driver->irq_disable(pin.dev->dev, pin.num);
}

#endif /* defined(MODULE_PERIPH_GPIO_IRQ) || defined(DOXYGEN) */

/**
 * @brief   Get the current value of the given pin
 *
 * @param[in] pin       the pin to read
 *
 * @return              0 when pin is LOW
 * @return              >0 for HIGH
 */
static inline int gpio_read(gpio_t pin)
{
    return pin.dev->driver->read(pin.dev->dev, pin.num);
}

/**
 * @brief   Set the given pin to HIGH
 *
 * @param[in] pin       the pin to set
 */
static inline void gpio_set(gpio_t pin)
{
    pin.dev->driver->set(pin.dev->dev, pin.num);
}

/**
 * @brief   Set the given pin to LOW
 *
 * @param[in] pin       the pin to clear
 */
static inline void gpio_clear(gpio_t pin)
{
    pin.dev->driver->clear(pin.dev->dev, pin.num);
}

/**
 * @brief   Toggle the value of the given pin
 *
 * @param[in] pin       the pin to toggle
 */
static inline void gpio_toggle(gpio_t pin)
{
    pin.dev->driver->toggle(pin.dev->dev, pin.num);
}

/**
 * @brief   Set the given pin to the given value
 *
 * @param[in] pin       the pin to set
 * @param[in] value     value to set the pin to, 0 for LOW, HIGH otherwise
 */
static inline void gpio_write(gpio_t pin, int value)
{
    pin.dev->driver->write(pin.dev->dev, pin.num, value);
}

#ifdef __cplusplus
}
#endif

#endif /* PERIPH_GPIO_H */
/** @} */
