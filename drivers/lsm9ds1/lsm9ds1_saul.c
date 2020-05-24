/*
 * Copyright (C) 2020 Wojtek Freliszka
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 *
 */

/**
 * @ingroup     drivers_lsm9ds1
 * @{
 *
 * @file
 * @brief       Device driver implementation for the LSM9DS1 3D accelerometer/gyroscope. Based on lsm6dso.
 *
 * @author      Wojtek Freliszka
 * @author      Vincent Dupont <vincent@otakeys.com>
 * @author      Sebastian Meiling <s@mlng.net>
 *
 * @}
 */

#include "lsm9ds1.h"
#include "saul.h"

static int read_acc(const void *dev, phydat_t *res)
{
    int ret = lsm9ds1_read_acc((const lsm9ds1_t *)dev, (lsm9ds1_3d_data_t *)res->val);
    if (ret < 0) {
        return -ECANCELED;
    }

    res->scale = -3;
    res->unit = UNIT_G;

    return 3;
}

static int read_gyro(const void *dev, phydat_t *res)
{
    int ret = lsm9ds1_read_gyro((const lsm9ds1_t *)dev, (lsm9ds1_3d_data_t *)res->val);
    if (ret < 0) {
        return -ECANCELED;
    }

    res->scale = -1;
    res->unit = UNIT_DPS;

    return 3;
}

const saul_driver_t lsm9ds1_saul_acc_driver = {
    .read = read_acc,
    .write = saul_notsup,
    .type = SAUL_SENSE_ACCEL,
};

const saul_driver_t lsm9ds1_saul_gyro_driver = {
    .read = read_gyro,
    .write = saul_notsup,
    .type = SAUL_SENSE_GYRO,
};

