/*
 * Copyright (C) 2018 Gunar Schorcht
 * Copyright (C) 2018 Acutam Automation, LLC
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     tests
 *
 * @{
 *
 * @file
 * @brief       ADC extension test list
 *
 * @author      Gunar Schorcht <gunar@schorcht.net>
 * @author      Matthew Blue <matthew.blue.neuro@gmail.com>
 *
 * @}
 */

#ifndef ADC_EXT_CONF_H
#define ADC_EXT_CONF_H

#include <stddef.h>

#include "extend/adc.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Reference the driver structs
 *
 * @{
 */
extern const adc_ext_driver_t adc_ext_driver_dev0;
extern const adc_ext_driver_t adc_ext_driver_dev1;
extern const adc_ext_driver_t adc_ext_driver_dev2;
extern const adc_ext_driver_t adc_ext_notsup_driver;
/** @} */

/**
 * @brief   ADC extension test list
 */
static const adc_ext_t adc_ext_list[] =
{
    {
        .driver = &adc_ext_driver_dev0,
        .dev = (void *)0xc001,
    },
    {
        .driver = &adc_ext_driver_dev1,
        .dev = (void *)0xbeef,
    },
    {
        .driver = &adc_ext_driver_dev2,
        .dev = (void *)0x0f00,
    },
    {
        .driver = &adc_ext_notsup_driver,
        .dev = NULL,
    },
};

#define ADC_EXT_DEV0_NUMOF  (4)
#define ADC_EXT_DEV1_NUMOF  (2)
#define ADC_EXT_DEV2_NUMOF  (8)
#define ADC_EXT_DEV3_NUMOF  (0)

#define ADC_EXT_DEV0_OFFSET (0)
#define ADC_EXT_DEV1_OFFSET (ADC_EXT_DEV0_OFFSET + ADC_EXT_DEV0_NUMOF)
#define ADC_EXT_DEV2_OFFSET (ADC_EXT_DEV1_OFFSET + ADC_EXT_DEV1_NUMOF)
#define ADC_EXT_DEV3_OFFSET (ADC_EXT_DEV2_OFFSET + ADC_EXT_DEV2_NUMOF)

#define ADC_EXT_NUMOF       (ADC_EXT_DEV0_NUMOF + \
                             ADC_EXT_DEV1_NUMOF + \
                             ADC_EXT_DEV2_NUMOF + \
                             ADC_EXT_DEV3_NUMOF)

#define ADC_EXT_LINE(x)     (x < ADC_EXT_DEV1_OFFSET \
                             ? ADC_EXT_LINE_REV(0, (x - ADC_EXT_DEV0_OFFSET)) \
                             : (x < ADC_EXT_DEV2_OFFSET \
                                ? ADC_EXT_LINE_REV(1, (x - ADC_EXT_DEV1_OFFSET)) \
                                : (x < ADC_EXT_DEV3_OFFSET \
                                   ? ADC_EXT_LINE_REV(2, (x - ADC_EXT_DEV2_OFFSET)) \
                                   : ADC_EXT_LINE_REV(3, (x - ADC_EXT_DEV3_OFFSET)))))

#ifdef __cplusplus
}
#endif

#endif /* ADC_EXT_CONF_H */
/** @} */
