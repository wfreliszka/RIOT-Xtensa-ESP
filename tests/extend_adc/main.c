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
 * @{
 *
 * @file
 * @brief       ADC extension test routine
 *
 * @author      Gunar Schorcht <gunar@schorcht.net>
 * @author      Matthew Blue <matthew.blue.neuro@gmail.com>
 * @}
 */

#include <stdio.h>

#include "adc_ext_conf.h"
#include "extend/adc.h"
#include "periph/adc.h"

/* ADC extension test device functions */
int adc_ext_init(void *dev, adc_t chn);
int adc_ext_sample(void *dev, adc_t chn, adc_res_t res);
unsigned int adc_ext_channels_dev0(void *dev);
unsigned int adc_ext_channels_dev1(void *dev);
unsigned int adc_ext_channels_dev2(void *dev);

/* ADC extension device 0 driver */
const adc_ext_driver_t adc_ext_driver_dev0 = {
    .init = adc_ext_init,
    .sample = adc_ext_sample,
    .channels = adc_ext_channels_dev0,
};

/* ADC extension device 1 driver */
const adc_ext_driver_t adc_ext_driver_dev1 = {
    .init = adc_ext_init,
    .sample = adc_ext_sample,
    .channels = adc_ext_channels_dev1,
};

/* ADC extension device 2 driver */
const adc_ext_driver_t adc_ext_driver_dev2 = {
    .init = adc_ext_init,
    .sample = adc_ext_sample,
    .channels = adc_ext_channels_dev2,
};

int adc_ext_init(void *dev, adc_t chn)
{
    printf("\t%s dev=0x%04x chn=%u\n", __func__, (uint16_t)(uintptr_t)dev, chn);
    return 0;
}

int adc_ext_sample(void *dev, adc_t chn, adc_res_t res)
{
    printf("\t%s dev=0x%04x chn=%u res=%u\n",
           __func__, (uint16_t)(uintptr_t)dev, chn, res);
    return 128 * chn;
}

unsigned int adc_ext_channels_dev0(void *dev)
{
    printf("\t%s dev=0x%04x\n", __func__, (uint16_t)(uintptr_t)dev);
    return ADC_EXT_DEV0_NUMOF;
}

unsigned int adc_ext_channels_dev1(void *dev)
{
    printf("\t%s dev=0x%04x\n", __func__, (uint16_t)(uintptr_t)dev);
    return ADC_EXT_DEV1_NUMOF;
}

unsigned int adc_ext_channels_dev2(void *dev)
{
    printf("\t%s dev=0x%04x\n", __func__, (uint16_t)(uintptr_t)dev);
    return ADC_EXT_DEV2_NUMOF;
}

int main(void)
{
    adc_t line;

    puts("ADC extension test routine");
    printf("Number of ADC channels: %d\n", ADC_NUMOF);
    printf("    low-level channels: %d\n", ADC_NUMOF_LL);
    printf("    extension channels: %d\n", ADC_EXT_NUMOF);

    for (line = 0; line < ADC_NUMOF; line++) {
        printf("Init ADC channel %u\n", line);
        if (adc_init(ADC_LINE(line))) {
            puts("[FAILED]");
            return 1;
        }

        int sample = adc_sample(ADC_LINE(line), ADC_RES_10BIT);
        if (sample < 0) {
            puts("[FAILED]");
            return 1;
        }
        printf("Sample of ADC channel %u: %d\n", line, sample);
    }

    puts("[SUCCESS]");

    return 0;
}
