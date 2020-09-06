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

#include <stdio.h>

#include "xtimer.h"
#include "hcsr0x.h"

//gpio_t trigger_pin = GPIO_PIN(PORT_C, 10);
gpio_t trigger_pin = GPIO_PIN(PORT_D, 11);
//GPIO_PIN(0, 2);
gpio_t echo_pin = GPIO_PIN(PORT_A, 1);
//GPIO_PIN(0, 5);
 

//gpio_t trigger_pin2 = GPIO_PIN(PORT_C, 11);
//gpio_t trigger_pin2 = GPIO_PIN(PORT_B, 2);
//gpio_t echo_pin2 = GPIO_PIN(PORT_C, 1);

    void * dev1=NULL;
int out_milimeters_distance=0;
void output_callback(const void *dev,int16_t milimeters_distance,int32_t payload)
{
	out_milimeters_distance=milimeters_distance;
    payload+=1;
    dev1=(void *)dev;
}


int main(void)
{
    hcsr0x_t dev;
    //hcsr0x_t dev2;
    xtimer_sleep(1);

    printf("HCSR0X test application %d\n",(int)dev1);
   

    if (hcsr0x_init(&dev, &output_callback,trigger_pin,echo_pin,false,111) != HCSR0X_OK) {
        printf("[ERROR] in init");
        return 1;
    }
    // if (hcsr0x_init(&dev2, &output_callback,trigger_pin2,echo_pin2,true) != HCSR0X_OK) {
    //     printf("[ERROR] in init");
    //     return 1;
    // }
    printf("[SUCCESS]\n");

    //xtimer_sleep(1);
	printf("Initiating measurements:\n");
    while (1) {
		//printf("Measure!\n");
		xtimer_usleep(100*1000);
        //xtimer_sleep(1);
        if (hcsr0x_measure(&dev) != HCSR0X_OK)
		{
            puts("\n[ERROR] invoking echo!\n");
        }
		printf("Distance is %d - %d cm\n",(int)dev1,(int)out_milimeters_distance);

        // xtimer_usleep(100*1000);
        // if (hcsr0x_measure(&dev2) != HCSR0X_OK)
		// {
        //     puts("\n[ERROR] invoking echo!\n");
        // }
		// printf("Distance is %d - %d cm\n",(int)dev1,(int)out_milimeters_distance);
		
    }

    return 0;
}
