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

#ifndef LSM9DS1_INTERNAL_H
#define LSM9DS1_INTERNAL_H

#include "xtimer.h"

#ifdef __cplusplus
extern "C" {
#endif

// LSM9DS1 Accel/Gyro (XL/G) Registers //
/////////////////////////////////////////
#define LSM9DS1_ACT_THS             0x04
#define LSM9DS1_ACT_DUR             0x05
#define LSM9DS1_INT_GEN_CFG_XL      0x06
#define LSM9DS1_INT_GEN_THS_X_XL    0x07
#define LSM9DS1_INT_GEN_THS_Y_XL    0x08
#define LSM9DS1_INT_GEN_THS_Z_XL    0x09
#define LSM9DS1_INT_GEN_DUR_XL      0x0A
#define LSM9DS1_REFERENCE_G         0x0B
#define LSM9DS1_INT1_CTRL           0x0C
#define LSM9DS1_INT2_CTRL           0x0D
#define LSM9DS1_WHO_AM_I_XG         0x0F
#define LSM9DS1_CTRL_REG1_G         0x10
#define LSM9DS1_CTRL_REG2_G         0x11
#define LSM9DS1_CTRL_REG3_G         0x12
#define LSM9DS1_ORIENT_CFG_G        0x13
#define LSM9DS1_INT_GEN_SRC_G       0x14
#define LSM9DS1_OUT_TEMP_L          0x15
#define LSM9DS1_OUT_TEMP_H          0x16
#define LSM9DS1_STATUS_REG_0        0x17

#define LSM9DS1_CTRL_REG4           0x1E
#define LSM9DS1_CTRL_REG5_XL        0x1F
#define LSM9DS1_CTRL_REG6_XL        0x20
#define LSM9DS1_CTRL_REG7_XL        0x21
#define LSM9DS1_CTRL_REG8           0x22
#define LSM9DS1_CTRL_REG9           0x23
#define LSM9DS1_CTRL_REG10          0x24
#define LSM9DS1_INT_GEN_SRC_XL      0x26
#define LSM9DS1_STATUS_REG_1        0x27

#define LSM9DS1_FIFO_CTRL           0x2E
#define LSM9DS1_FIFO_SRC            0x2F
#define LSM9DS1_INT_GEN_CFG_G       0x30
#define LSM9DS1_INT_GEN_THS_XH_G    0x31
#define LSM9DS1_INT_GEN_THS_XL_G    0x32
#define LSM9DS1_INT_GEN_THS_YH_G    0x33
#define LSM9DS1_INT_GEN_THS_YL_G    0x34
#define LSM9DS1_INT_GEN_THS_ZH_G    0x35
#define LSM9DS1_INT_GEN_THS_ZL_G    0x36
#define LSM9DS1_INT_GEN_DUR_G       0x37
 
/**
 * @name    LSM9DS1 registers
 * @{
 */
#define LSM9DS1_REG_FUNC_CFG_ACCESS         (0x01)
#define LSM9DS1_REG_SENSOR_SYNC_TIME_FRAME  (0x04)
#define LSM9DS1_REG_SENSOR_SYC_RES_RATIO    (0x05)
#define LSM9DS1_REG_FIFO_CTRL1              (0x06)
#define LSM9DS1_REG_FIFO_CTRL2              (0x07)
#define LSM9DS1_REG_FIFO_CTRL3              (0x08)
#define LSM9DS1_REG_FIFO_CTRL4              (0x09)
#define LSM9DS1_REG_FIFO_CTRL5              (0x0A)
#define LSM9DS1_REG_DRDY_PULSE_CFG_G        (0x0B)
#define LSM9DS1_REG_INT1_CTRL               (0x0D)
#define LSM9DS1_REG_INT2_CTRL               (0x0E)
#define LSM9DS1_REG_WHO_AM_I                (0x0F)
#define LSM9DS1_REG_CTRL1_XL                (0x10)
#define LSM9DS1_REG_CTRL2_G                 (0x11)
#define LSM9DS1_REG_CTRL3_C                 (0x12)
#define LSM9DS1_REG_CTRL4_C                 (0x13)
#define LSM9DS1_REG_CTRL5_C                 (0x14)
#define LSM9DS1_REG_CTRL6_C                 (0x15)
#define LSM9DS1_REG_CTRL7_G                 (0x16)
#define LSM9DS1_REG_CTRL8_XL                (0x17)
#define LSM9DS1_REG_CTRL9_XL                (0x18)
#define LSM9DS1_REG_CTRL10_C                (0x19)
#define LSM9DS1_REG_MASTER_CONFIG           (0x1A)
#define LSM9DS1_REG_WAKE_UP_SRC             (0x1B)
#define LSM9DS1_REG_TAP_SRC                 (0x1C)
#define LSM9DS1_REG_D6D_SRC                 (0x1D)
#define LSM9DS1_REG_STATUS_REG              (0x1E)
#define LSM9DS1_REG_OUT_TEMP_L              (0x15)
#define LSM9DS1_REG_OUT_TEMP_H              (0x16)


//ACC and gryro out regs
#define LSM9DS1_REG_OUTX_L_G                (0x18)
#define LSM9DS1_REG_OUTX_H_G                (0x19)
#define LSM9DS1_REG_OUTY_L_G                (0x1A)
#define LSM9DS1_REG_OUTY_H_G                (0x1B)
#define LSM9DS1_REG_OUTZ_L_G                (0x1C)
#define LSM9DS1_REG_OUTZ_H_G                (0x1D)
#define LSM9DS1_REG_OUTX_L_XL               (0x28)
#define LSM9DS1_REG_OUTX_H_XL               (0x29)
#define LSM9DS1_REG_OUTY_L_XL               (0x2A)
#define LSM9DS1_REG_OUTY_H_XL               (0x2B)
#define LSM9DS1_REG_OUTZ_L_XL               (0x2C)
#define LSM9DS1_REG_OUTZ_H_XL               (0x2D)




#define LSM9DS1_REG_SENSORHUB1_REG          (0x2E)
#define LSM9DS1_REG_SENSORHUB2_REG          (0x2F)
#define LSM9DS1_REG_SENSORHUB3_REG          (0x30)
#define LSM9DS1_REG_SENSORHUB4_REG          (0x31)
#define LSM9DS1_REG_SENSORHUB5_REG          (0x32)
#define LSM9DS1_REG_SENSORHUB6_REG          (0x33)
#define LSM9DS1_REG_SENSORHUB7_REG          (0x34)
#define LSM9DS1_REG_SENSORHUB8_REG          (0x35)
#define LSM9DS1_REG_SENSORHUB9_REG          (0x36)
#define LSM9DS1_REG_SENSORHUB10_REG         (0x37)
#define LSM9DS1_REG_SENSORHUB11_REG         (0x38)
#define LSM9DS1_REG_SENSORHUB12_REG         (0x39)
#define LSM9DS1_REG_FIFO_STATUS1            (0x3A)
#define LSM9DS1_REG_FIFO_STATUS2            (0x3B)
#define LSM9DS1_REG_FIFO_STATUS3            (0x3C)
#define LSM9DS1_REG_FIFO_STATUS4            (0x3D)
#define LSM9DS1_REG_FIFO_DATA_OUT_L         (0x3E)
#define LSM9DS1_REG_FIFO_DATA_OUT_H         (0x3F)
#define LSM9DS1_REG_TIMESTAMP0_REG          (0x40)
#define LSM9DS1_REG_TIMESTAMP1_REG          (0x41)
#define LSM9DS1_REG_TIMESTAMP2_REG          (0x42)
#define LSM9DS1_REG_STEP_TIMESTAMP_L        (0x49)
#define LSM9DS1_REG_STEP_TIMESTAMP_H        (0x4A)
#define LSM9DS1_REG_STEP_COUNTER_L          (0x4B)
#define LSM9DS1_REG_STEP_COUNTER_H          (0x4C)
#define LSM9DS1_REG_SENSORHUB13_REG         (0x4D)
#define LSM9DS1_REG_SENSORHUB14_REG         (0x4E)
#define LSM9DS1_REG_SENSORHUB15_REG         (0x4F)
#define LSM9DS1_REG_SENSORHUB16_REG         (0x50)
#define LSM9DS1_REG_SENSORHUB17_REG         (0x51)
#define LSM9DS1_REG_SENSORHUB18_REG         (0x52)
#define LSM9DS1_REG_FUNC_SRC_1              (0x53)
#define LSM9DS1_REG_FUNC_SRC_2              (0x54)
#define LSM9DS1_REG_WRIST_TILT_IA           (0x55)
#define LSM9DS1_REG_TAP_CFG                 (0x58)
#define LSM9DS1_REG_TAP_THS_6D              (0x59)
#define LSM9DS1_REG_INT_DUR2                (0x5A)
#define LSM9DS1_REG_WAKE_UP_THS             (0x5B)
#define LSM9DS1_REG_WAKE_UP_DUR             (0x5C)
#define LSM9DS1_REG_FREE_FALL               (0x5D)
#define LSM9DS1_REG_MD1_CFG                 (0x5E)
#define LSM9DS1_REG_MD2_CFG                 (0x5F)
#define LSM9DS1_REG_MASTER_CMD_CODE         (0x60)
#define LSM9DS1_REG_SENS_SYNC_SPI_ERR_CODE  (0x61)
#define LSM9DS1_REG_OUT_MAG_RAW_X_L         (0x66)
#define LSM9DS1_REG_OUT_MAG_RAW_X_H         (0x67)
#define LSM9DS1_REG_OUT_MAG_RAW_Y_L         (0x68)
#define LSM9DS1_REG_OUT_MAG_RAW_Y_H         (0x69)
#define LSM9DS1_REG_OUT_MAG_RAW_Z_L         (0x6A)
#define LSM9DS1_REG_OUT_MAG_RAW_Z_H         (0x6B)
#define LSM9DS1_REG_X_OFS_USR               (0x73)
#define LSM9DS1_REG_Y_OFS_USR               (0x74)
#define LSM9DS1_REG_Z_OFS_USR               (0x75)
/** @} */

/** WHO_AM_I value */
#define LSM9DS1_WHO_AM_I                    0x68

/**
 * @name    CTRL_x registers
 * @{
 */
#define LSM9DS1_CTRL_ODR_SHIFT              (4)
#define LSM9DS1_CTRL_ODR_MASK               (0xF0)
#define LSM9DS1_CTRL_FS_SHIFT               (2)
#define LSM9DS1_CTRL_FS_MASK                (0x0C)

#define LSM9DS1_CTRL3_C_BOOT                (0x80)
/** @} */

/**
 * @name    FIFO_CTRL_x registers
 * @{
 */
#define LSM9DS1_FIFO_CTRL5_CONTINUOUS_MODE  (0x6)
#define LSM9DS1_FIFO_CTRL5_FIFO_ODR_SHIFT   (3)

#define LSM9DS1_FIFO_CTRL3_GYRO_DEC_SHIFT   (3)
/** @} */

/**
 * @brief	Offset for temperature calculation
 */
#define LSM9DS1_TEMP_OFFSET                 (0x1900)

/**
 * @brief 	Reboot wait interval in us (15ms)
 */
#define LSM9DS1_BOOT_WAIT                   (15 * US_PER_MS)

#ifdef __cplusplus
}
#endif

#endif /* LSM9DS1_INTERNAL_H */
/** @} */
