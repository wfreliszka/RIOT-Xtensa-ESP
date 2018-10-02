/*
 * Copyright (C) 2018 Gunar Schorcht
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_htu21d
 * @brief       Default configuration for HTU21D humidity and temperature sensor
 * @author      Gunar Schorcht <gunar@schorcht.net>
 * @file
 *
 * @note The HTU21D driver is just a wrapper for the driver for the compatible
 * SI7021 Humidity and Temperature Sensor. Please refer @ref drivers_si70xx
 * for details about the driver.
 *
 * @{
 */

#ifndef HTU21D_PARAMS_H
#define HTU21D_PARAMS_H

#include "si70xx_params.h"

#ifdef __cplusplus
extern "C" {
#endif

#define htu21d_params   si70xx_params

#ifdef __cplusplus
}
#endif

#endif /* HTU21D_PARAMS_H */
/** @} */
