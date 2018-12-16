/*
 * Copyright (C) 2018 Gunar Schorcht
 * Copyright (C) 2018 Acutam Automation, LLC
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_extend_adc
 * @brief       ADC extension configuration file example
 *
 * The configuration of ADC extension devices is defined in the `adc_ext_conf.h`
 * file as part of the board definition in `$RIOTBASE/boards/<board>/include`.
 * This default configuration can be overridden by an application-specific
 * ADC extension configuration file in `$APPDIR/include`.
 *
 * The ADC extension configuration file has to define
 *
 * - the list of ADC extension devices of type #adc_ext_t in #adc_ext_list,
 * - the total number of ADC extension lines #ADC_EXT_NUMOF, and
 * - the #ADC_EXT_LINE macro for mapping the n-th ADC extension line to the tupel
 *   (device, channel) where n can be in the range from 0 to #ADC_EXT_NUMOF-1.
 *
 * The configuration for three extension devices could then look like the
 * following:
 *
 * ```C
 * #define ADC_EXT_DEV0_NUMOF  (4)
 * #define ADC_EXT_DEV1_NUMOF  (2)
 * #define ADC_EXT_DEV2_NUMOF  (8)
 *
 * #define ADC_EXT_DEV0_OFFSET (0)
 * #define ADC_EXT_DEV1_OFFSET (ADC_EXT_DEV0_OFFSET + ADC_EXT_DEV0_NUMOF)
 * #define ADC_EXT_DEV2_OFFSET (ADC_EXT_DEV1_OFFSET + ADC_EXT_DEV1_NUMOF)
 *
 * #define ADC_EXT_NUMOF       (ADC_EXT_DEV0_NUMOF + \
 *                              ADC_EXT_DEV1_NUMOF + \
 *                              ADC_EXT_DEV2_NUMOF)
 *
 * #define ADC_EXT_LINE(x)     (x < ADC_EXT_DEV1_OFFSET \
 *                              ? ADC_EXT_LINE_REV(0, (x - ADC_EXT_DEV0_OFFSET)) \
 *                              : (x < ADC_EXT_DEV2_OFFSET \
 *                                 ? ADC_EXT_LINE_REV(1, (x - ADC_EXT_DEV1_OFFSET)) \
 *                                 : ADC_EXT_LINE_REV(2, (x - ADC_EXT_DEV2_OFFSET))))
 *
 * ```
 *
 * @author      Gunar Schorcht <gunar@schorcht.net>
 * @author      Matthew Blue <matthew.blue.neuro@gmail.com>
 * @file
 *
 * @{
 */

#ifndef ADC_EXT_CONF_H
#define ADC_EXT_CONF_H

#include <stddef.h>

#include "extend/adc.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Reference the driver struct
 */
extern adc_ext_driver_t adc_ext_notsup_driver;

/**
 * @brief   ADC expansion default list if not defined
 */
static const adc_ext_t adc_ext_list[] =
{
    {
        .driver = &adc_ext_notsup_driver,
        .dev = NULL,
    },
};

/**
 * @brief   Number of channels of the first ADC extension device
 *
 * Declare the number of ADC channels for each ADC extension device.
 */
#define ADC_EXT_DEV0_NUMOF  (0)

/**
 * @brief   Default offset of channels of the first ADC extension device
 */
#define ADC_EXT_DEV0_OFFSET (0)

/**
 * @brief   Default number of ADC extension channels
 */
#define ADC_EXT_NUMOF       (ADC_EXT_DEV0_NUMOF)

/**
 * @brief   Default ADC line access macro for ADC extension channels
 */
#define ADC_EXT_LINE(x)     (ADC_EXT_LINE_REV(0, x - ADC_EXT_DEV0_OFFSET))


#ifdef __cplusplus
}
#endif

#endif /* ADC_EXT_CONF_H */
/** @} */
