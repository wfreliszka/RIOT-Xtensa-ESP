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
#include "xtimer.h"

#include "hcsr0x.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

void hcsr0x_echo_cb(void* arg){
    assert(arg);

    DEBUG("hcsr0x_echo_cb\n"); 
	hcsr0x_t *dev=(hcsr0x_t *)arg;
    int value = gpio_read(dev->params.echo_pin);

    DEBUG("hcsr0x_echo_cb value %d\n",value);

    if(value)
    {   //this is a rising edge. We start measuring
        dev->params.start_time = xtimer_now_usec();
    }
    else
    {   //this is a falling edge. We conclude measurement
        uint32_t end_time = xtimer_now_usec();
	    uint32_t echo_time = end_time - dev->params.start_time;
        DEBUG("echo_time %ld\n",echo_time);
        if(echo_time>0xE000)
            dev->params.callback(dev,HCSR0X_ECHO_DURATION_INVALID);
        else{
            int16_t distance = echo_time/58;
            dev->params.callback(dev,distance);
        }
    }
    

}

int hcsr0x_init(hcsr0x_t *dev, hcsr0x_cb_t cb, gpio_t trigger_pin,gpio_t echo_pin,bool trigger_reused)
{
    int res;
    assert(dev);

    dev->params.callback=cb;
    dev->params.trigger_pin=trigger_pin;
    dev->params.echo_pin=echo_pin;

    if(trigger_reused==false)
    {
        res=gpio_init(trigger_pin, GPIO_OUT);
        if(res<0){
            DEBUG("[ERROR] gpio_init failed for trigger_pin %d!\n",res); 
            return HCSR0X_TRIGGER_PIN_ERR;
        }
        gpio_clear(trigger_pin);
    }


	res=gpio_init_int(echo_pin, GPIO_IN, GPIO_BOTH, &hcsr0x_echo_cb, dev);
    if(res<0){
        DEBUG("[ERROR] gpio_init_int failed for echo_pin %d!\n",res); 
        return HCSR0X_ECHO_PIN_ERR;
    }

DEBUG("hcsr0x_echo_cb\n"); 
    return HCSR0X_OK;
}

    int hcsr0x_measure(hcsr0x_t *dev)
{
    DEBUG("measure\n"); 
    gpio_clear(dev->params.trigger_pin);
    xtimer_usleep(50);
    gpio_set(dev->params.trigger_pin);
    return HCSR0X_OK;
}