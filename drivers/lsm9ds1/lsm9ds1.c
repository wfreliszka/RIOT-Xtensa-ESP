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

#include "xtimer.h"

#include "lsm9ds1.h"
#include "lsm9ds1_internal.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

#define MAX(a, b) ((a) > (b) ? (a) : (b))

#define BUS         (dev->params.i2c)
#define ADDR        (dev->params.addr)
#define I2C_AUTO_INCREMENT    (0x80)
/**
 * order in array [0, 1, 2, 3] is
 * LSM9DS1_ACC_FS_2G, LSM9DS1_ACC_FS_16G, LSM9DS1_ACC_FS_4G, LSM9DS1_ACC_FS_8G
 */
static const int16_t range_acc[] = { 2000, 16000, 4000, 8000 };

/**
 * order in array [0, 1, 2, 3] is
 * LSM9DS1_GYRO_FS_245DPS, LSM9DS1_GYRO_FS_500DPS,
 * LSM9DS1_GYRO_FS_1000DPS, LSM9DS1_GYRO_FS_2000DPS
 */
static const int16_t range_gyro[] = { 2450, 5000, 10000, 20000 };

int lsm9ds1_init(lsm9ds1_t *dev, const lsm9ds1_params_t *params)
{
    uint8_t tmp;
    int res;

    assert(dev && params);

    dev->params = *params;

    i2c_acquire(BUS);

    /* Reboot */
    i2c_write_reg(BUS, ADDR, LSM9DS1_REG_CTRL3_C, LSM9DS1_CTRL3_C_BOOT, 0);

    xtimer_usleep(LSM9DS1_BOOT_WAIT);

    if (i2c_read_reg(BUS, ADDR, LSM9DS1_REG_WHO_AM_I, &tmp, 0) < 0) {
        i2c_release(BUS);
        DEBUG("[ERROR] lsm9ds1_init: i2c_read_reg LSM9DS1_REG_WHO_AM_I!\n");
        return -LSM9DS1_ERROR_BUS;
    }

    if (tmp != LSM9DS1_WHO_AM_I) {
        DEBUG("[ERROR] lsm9ds1_init: WHO_AM_I, wrong chip id %02x, "
                           "should be %02x\n",tmp,LSM9DS1_WHO_AM_I);
        return -LSM9DS1_ERROR_DEV;
    }

    /* Set acc odr / full scale */
    tmp = (dev->params.acc_odr << LSM9DS1_CTRL_ODR_SHIFT) |
          (dev->params.acc_fs << LSM9DS1_CTRL_FS_SHIFT);
    res = i2c_write_reg(BUS, ADDR, LSM9DS1_REG_CTRL1_XL, tmp, 0);
    /* Set gyro odr / full scale */
    tmp = (dev->params.gyro_odr << LSM9DS1_CTRL_ODR_SHIFT) |
          (dev->params.gyro_fs << LSM9DS1_CTRL_FS_SHIFT);
    res += i2c_write_reg(BUS, ADDR, LSM9DS1_REG_CTRL2_G, tmp, 0);
    /* Set continuous mode */
    uint8_t fifo_odr = MAX(dev->params.acc_odr, dev->params.gyro_odr);
    tmp = (fifo_odr << LSM9DS1_FIFO_CTRL5_FIFO_ODR_SHIFT) |
          LSM9DS1_FIFO_CTRL5_CONTINUOUS_MODE;
    res += i2c_write_reg(BUS, ADDR, LSM9DS1_REG_FIFO_CTRL5, tmp, 0);
    tmp = (dev->params.gyro_decimation << LSM9DS1_FIFO_CTRL3_GYRO_DEC_SHIFT) |
          dev->params.acc_decimation;
    res += i2c_write_reg(BUS, ADDR, LSM9DS1_REG_FIFO_CTRL3, tmp, 0);

    i2c_release(BUS);

    if (res < 0) {
        DEBUG("[ERROR] lsm9ds1_init: config\n");
        return -LSM9DS1_ERROR_CNF;
    }
    return LSM9DS1_OK;
}


int lsm9ds1_read_acc(const lsm9ds1_t *dev, lsm9ds1_3d_data_t *data)
{
    int res;
    uint8_t tmp;

    i2c_acquire(BUS);
    i2c_read_reg(BUS, ADDR, LSM9DS1_REG_STATUS_REG, &tmp, 0);
    DEBUG("lsm9ds1 status: %x\n", tmp);

    uint8_t in[6];
    int reg=LSM9DS1_REG_OUTX_L_XL;

    reg |= I2C_AUTO_INCREMENT;

    res = i2c_read_regs(BUS, ADDR, reg, in, 6, 0);

    i2c_release(BUS);


    if (res < 0) {
        DEBUG("[ERROR] lsm9ds1_read_acc\n");
        return -LSM9DS1_ERROR_BUS;
    }

     /* L3GD20H_REG_CTRL4.BLE = 0, Data LSB @ lower address */
    data->x = (in[1] << 8) | in[0];
    data->y = (in[3] << 8) | in[2];
    data->z = (in[5] << 8) | in[4];

    assert(dev->params.acc_fs < LSM9DS1_ACC_FS_MAX);
    data->x = ((int32_t)data->x * range_acc[dev->params.acc_fs]) / INT16_MAX;
    data->y = ((int32_t)data->y * range_acc[dev->params.acc_fs]) / INT16_MAX;
    data->z = ((int32_t)data->z * range_acc[dev->params.acc_fs]) / INT16_MAX;

    return LSM9DS1_OK;
}

int lsm9ds1_read_gyro(const lsm9ds1_t *dev, lsm9ds1_3d_data_t *data)
{
    int res;
    uint8_t tmp;

    i2c_acquire(BUS);
    i2c_read_reg(BUS, ADDR, LSM9DS1_REG_STATUS_REG, &tmp, 0);
    DEBUG("lsm9ds1 status: %x\n", tmp);

    uint8_t in[6];
    int reg=LSM9DS1_REG_OUTX_L_G;

    reg |= I2C_AUTO_INCREMENT;

    res = i2c_read_regs(BUS, ADDR, reg, in, 6, 0);

    i2c_release(BUS);

    if (res < 0) {
        DEBUG("[ERROR] lsm9ds1_read_gyro\n");
        return -LSM9DS1_ERROR_BUS;
    }

    data->x = (in[1] << 8) | in[0];
    data->y = (in[3] << 8) | in[2];
    data->z = (in[5] << 8) | in[4];

    assert(dev->params.gyro_fs < LSM9DS1_GYRO_FS_MAX);
    data->x = ((int32_t)data->x * range_gyro[dev->params.gyro_fs]) ;// INT16_MAX;
    data->y = ((int32_t)data->y * range_gyro[dev->params.gyro_fs]) ;// INT16_MAX;
    data->z = ((int32_t)data->z * range_gyro[dev->params.gyro_fs]) ;// INT16_MAX;

    return LSM9DS1_OK;
}

int lsm9ds1_read_temp(const lsm9ds1_t *dev, int16_t *data)
{
    uint8_t tmp;
    uint16_t traw;
    /* read raw temperature */
    i2c_acquire(BUS);
    if (i2c_read_reg(BUS, ADDR, LSM9DS1_REG_OUT_TEMP_L, &tmp, 0) < 0) {
        i2c_release(BUS);
        return -LSM9DS1_ERROR_BUS;
    }
    traw = tmp;
    if (i2c_read_reg(BUS, ADDR, LSM9DS1_REG_OUT_TEMP_H, &tmp, 0) < 0) {
        i2c_release(BUS);
        return -LSM9DS1_ERROR_BUS;
    }
    traw |= (uint16_t)tmp << 8;
    i2c_release(BUS);
    /* convert temperature to degC x 100 */
    traw += LSM9DS1_TEMP_OFFSET;
    *data = (int16_t)(((int32_t)traw * 100) / 256);

    return LSM9DS1_OK;
}

int lsm9ds1_acc_power_down(const lsm9ds1_t *dev)
{
    int res;
    uint8_t tmp;

    i2c_acquire(BUS);
    res = i2c_read_reg(BUS, ADDR, LSM9DS1_REG_CTRL1_XL, &tmp, 0);
    if (res < 0) {
        i2c_release(BUS);
        DEBUG("[ERROR] lsm9ds1_acc_power_down\n");
        return -LSM9DS1_ERROR_BUS;
    }

    tmp &= ~(LSM9DS1_CTRL_ODR_MASK);
    res = i2c_write_reg(BUS, ADDR, LSM9DS1_REG_CTRL1_XL, tmp, 0);

    i2c_release(BUS);

    if (res < 0) {
        DEBUG("[ERROR] lsm9ds1_acc_power_down\n");
        return -LSM9DS1_ERROR_BUS;
    }

    return LSM9DS1_OK;
}

int lsm9ds1_gyro_power_down(const lsm9ds1_t *dev)
{
    int res;
    uint8_t tmp;

    i2c_acquire(BUS);
    res = i2c_read_reg(BUS, ADDR, LSM9DS1_REG_CTRL2_G, &tmp, 0);
    if (res < 0) {
        i2c_release(BUS);
        DEBUG("[ERROR] lsm9ds1_gyro_power_down\n");
        return -LSM9DS1_ERROR_BUS;
    }

    tmp &= ~(LSM9DS1_CTRL_ODR_MASK);
    res = i2c_write_reg(BUS, ADDR, LSM9DS1_REG_CTRL2_G, tmp, 0);

    i2c_release(BUS);

    if (res < 0) {
        DEBUG("[ERROR] lsm9ds1_gyro_power_down\n");
        return -LSM9DS1_ERROR_BUS;
    }

    return LSM9DS1_OK;
}

int lsm9ds1_acc_power_up(const lsm9ds1_t *dev)
{
    int res;
    uint8_t tmp;

    i2c_acquire(BUS);
    res = i2c_read_reg(BUS, ADDR, LSM9DS1_REG_CTRL1_XL, &tmp, 0);
    if (res < 0) {
        i2c_release(BUS);
        DEBUG("[ERROR] lsm9ds1_acc_power_up\n");
        return -LSM9DS1_ERROR_BUS;
    }

    tmp &= ~(LSM9DS1_CTRL_ODR_MASK);
    tmp |= dev->params.acc_odr << LSM9DS1_CTRL_ODR_SHIFT;
    res = i2c_write_reg(BUS, ADDR, LSM9DS1_REG_CTRL1_XL, tmp, 0);

    i2c_release(BUS);

    if (res < 0) {
        DEBUG("[ERROR] lsm9ds1_acc_power_up\n");
        return -LSM9DS1_ERROR_BUS;
    }

    return LSM9DS1_OK;
}

int lsm9ds1_gyro_power_up(const lsm9ds1_t *dev)
{
    int res;
    uint8_t tmp;

    i2c_acquire(BUS);
    res = i2c_read_reg(BUS, ADDR, LSM9DS1_REG_CTRL2_G, &tmp, 0);
    if (res < 0) {
        i2c_release(BUS);
        DEBUG("[ERROR] lsm9ds1_gyro_power_up\n");
        return -LSM9DS1_ERROR_BUS;
    }

    tmp &= ~(LSM9DS1_CTRL_ODR_MASK);
    tmp |= dev->params.gyro_odr << LSM9DS1_CTRL_ODR_SHIFT;
    res = i2c_write_reg(BUS, ADDR, LSM9DS1_REG_CTRL2_G, tmp, 0);

    i2c_release(BUS);

    if (res < 0) {
        DEBUG("[ERROR] lsm9ds1_gyro_power_up\n");
        return -LSM9DS1_ERROR_BUS;
    }

    return LSM9DS1_OK;
}
