/*
 * Copyright (C) 2018 Gunar Schorcht
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_htu21d HTU21D humidity and temperature
 * @ingroup     drivers_sensors
 * @ingroup     drivers_saul
 * @brief       Device driver for HTU21D humidity and temperature sensor
 *
 * @note The HTU21D driver is just a wrapper for the driver for the compatible
 * SI7021 Humidity and Temperature Sensor. Please refer @ref drivers_si70xx
 * for details about the driver.
 * 
 * @{
 *
 * @author      Gunar Schorcht <gunar@schorcht.net>
 * @file
 */

#ifndef HTU21D_H
#define HTU21D_H

#include "si70xx.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @name Symbol redefinitions
 */
#define HTU21D_OK                       SI70XX_OK
#define HTU21D_ERR_NODEV                SI70XX_ERR_NODEV
#define HTU21D_ERR_I2C                  SI70XX_ERR_I2C

#define htu21d_params_t                 si70xx_params_t
#define htu21d_t                        si70xx_t
#define htu21d_init                     si70xx_init
#define htu21d_get_both                 si70xx_get_both
#define htu21d_get_temperature          si70xx_get_temperature
#define htu21d_get_relative_humidity    si70xx_get_relative_humidity
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* HTU21D_H */
/** @} */
