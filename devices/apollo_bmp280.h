//*****************************************************************************
//
//! @file bmp280.h
//!
//! @brief Functions for controlling bmp280(baromater sensor)
//!
//! @addtogroup devices External Device Control Library
//! @addtogroup IIC Device Control for bmp280.
//! @ingroup devices
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2020, Thundercomm, Inc.
// All rights reserved.
//
//*****************************************************************************
#ifndef APOLLO_BMP280_H
#define APOLLO_BMP280_H

#include <stdint.h>
#include "apollo3_init.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define BMP280_IOM_MODULE  4
#define BMP280_ADDR        0x76

// sensor ID
#define BMP280_CHIP_ID3                      UINT8_C(0x58)

/*! @name Return codes */
/*! @name Success code*/
#define BMP280_OK                            INT8_C(0)
#define BMP280_BOND_WIRE_OK                  INT8_C(0)

/*! @name Error codes */
#define BMP280_E_NULL_PTR                    INT8_C(-1)
#define BMP280_E_DEV_NOT_FOUND               INT8_C(-2)
#define BMP280_E_INVALID_LEN                 INT8_C(-3)
#define BMP280_E_COMM_FAIL                   INT8_C(-4)
#define BMP280_E_INVALID_MODE                INT8_C(-5)
#define BMP280_E_BOND_WIRE                   INT8_C(-6)
#define BMP280_E_IMPLAUS_TEMP                INT8_C(-7)
#define BMP280_E_IMPLAUS_PRESS               INT8_C(-8)
#define BMP280_E_CAL_PARAM_RANGE             INT8_C(-9)
#define BMP280_E_UNCOMP_TEMP_RANGE           INT8_C(-10)
#define BMP280_E_UNCOMP_PRES_RANGE           INT8_C(-11)
#define BMP280_E_UNCOMP_TEMP_AND_PRESS_RANGE INT8_C(-12)
#define BMP280_E_UNCOMP_DATA_CALC            INT8_C(-13)
#define BMP280_E_32BIT_COMP_TEMP             INT8_C(-14)
#define BMP280_E_32BIT_COMP_PRESS            INT8_C(-15)
#define BMP280_E_64BIT_COMP_PRESS            INT8_C(-16)
#define BMP280_E_DOUBLE_COMP_TEMP            INT8_C(-17)
#define BMP280_E_DOUBLE_COMP_PRESS           INT8_C(-18)

/*! @name ODR options */
#define BMP280_ODR_0_5_MS                    UINT8_C(0x00)
#define BMP280_ODR_62_5_MS                   UINT8_C(0x01)
#define BMP280_ODR_125_MS                    UINT8_C(0x02)
#define BMP280_ODR_250_MS                    UINT8_C(0x03)
#define BMP280_ODR_500_MS                    UINT8_C(0x04)
#define BMP280_ODR_1000_MS                   UINT8_C(0x05)
#define BMP280_ODR_2000_MS                   UINT8_C(0x06)
#define BMP280_ODR_4000_MS                   UINT8_C(0x07)

/*! @name Over-sampling macros */
#define BMP280_OS_NONE                       UINT8_C(0x00)
#define BMP280_OS_1X                         UINT8_C(0x01)
#define BMP280_OS_2X                         UINT8_C(0x02)
#define BMP280_OS_4X                         UINT8_C(0x03)
#define BMP280_OS_8X                         UINT8_C(0x04)
#define BMP280_OS_16X                        UINT8_C(0x05)

/*! @name Filter coefficient macros */
#define BMP280_FILTER_OFF                    UINT8_C(0x00)
#define BMP280_FILTER_COEFF_2                UINT8_C(0x01)
#define BMP280_FILTER_COEFF_4                UINT8_C(0x02)
#define BMP280_FILTER_COEFF_8                UINT8_C(0x03)
#define BMP280_FILTER_COEFF_16               UINT8_C(0x04)

/*! @name SPI 3-Wire macros */
#define BMP280_SPI3_WIRE_ENABLE              UINT8_C(1)
#define BMP280_SPI3_WIRE_DISABLE             UINT8_C(0)

// Register offset address
/*! @name Calibration parameter register addresses*/
#define BMP280_DIG_T1_LSB_ADDR               UINT8_C(0x88)
#define BMP280_DIG_T1_MSB_ADDR               UINT8_C(0x89)
#define BMP280_DIG_T2_LSB_ADDR               UINT8_C(0x8A)
#define BMP280_DIG_T2_MSB_ADDR               UINT8_C(0x8B)
#define BMP280_DIG_T3_LSB_ADDR               UINT8_C(0x8C)
#define BMP280_DIG_T3_MSB_ADDR               UINT8_C(0x8D)
#define BMP280_DIG_P1_LSB_ADDR               UINT8_C(0x8E)
#define BMP280_DIG_P1_MSB_ADDR               UINT8_C(0x8F)
#define BMP280_DIG_P2_LSB_ADDR               UINT8_C(0x90)
#define BMP280_DIG_P2_MSB_ADDR               UINT8_C(0x91)
#define BMP280_DIG_P3_LSB_ADDR               UINT8_C(0x92)
#define BMP280_DIG_P3_MSB_ADDR               UINT8_C(0x93)
#define BMP280_DIG_P4_LSB_ADDR               UINT8_C(0x94)
#define BMP280_DIG_P4_MSB_ADDR               UINT8_C(0x95)
#define BMP280_DIG_P5_LSB_ADDR               UINT8_C(0x96)
#define BMP280_DIG_P5_MSB_ADDR               UINT8_C(0x97)
#define BMP280_DIG_P6_LSB_ADDR               UINT8_C(0x98)
#define BMP280_DIG_P6_MSB_ADDR               UINT8_C(0x99)
#define BMP280_DIG_P7_LSB_ADDR               UINT8_C(0x9A)
#define BMP280_DIG_P7_MSB_ADDR               UINT8_C(0x9B)
#define BMP280_DIG_P8_LSB_ADDR               UINT8_C(0x9C)
#define BMP280_DIG_P8_MSB_ADDR               UINT8_C(0x9D)
#define BMP280_DIG_P9_LSB_ADDR               UINT8_C(0x9E)
#define BMP280_DIG_P9_MSB_ADDR               UINT8_C(0x9F)

/*! @name Other registers */
#define BMP280_CHIP_ID_ADDR                  UINT8_C(0xD0)
#define BMP280_SOFT_RESET_ADDR               UINT8_C(0xE0)
#define BMP280_STATUS_ADDR                   UINT8_C(0xF3)
#define BMP280_CTRL_MEAS_ADDR                UINT8_C(0xF4)
#define BMP280_CONFIG_ADDR                   UINT8_C(0xF5)
#define BMP280_PRES_MSB_ADDR                 UINT8_C(0xF7)
#define BMP280_PRES_LSB_ADDR                 UINT8_C(0xF8)
#define BMP280_PRES_XLSB_ADDR                UINT8_C(0xF9)
#define BMP280_TEMP_MSB_ADDR                 UINT8_C(0xFA)
#define BMP280_TEMP_LSB_ADDR                 UINT8_C(0xFB)
#define BMP280_TEMP_XLSB_ADDR                UINT8_C(0xFC)

/*! @name Power modes */
#define BMP280_SLEEP_MODE                    UINT8_C(0x00)
#define BMP280_FORCED_MODE                   UINT8_C(0x01)
#define BMP280_NORMAL_MODE                   UINT8_C(0x03)

/*! @name Soft reset command */
#define BMP280_SOFT_RESET_CMD                UINT8_C(0xB6)

/*! @name Position and mask macros */
#define BMP280_STATUS_IM_UPDATE_POS          UINT8_C(0)
#define BMP280_STATUS_IM_UPDATE_MASK         UINT8_C(0x01)
#define BMP280_STATUS_MEAS_POS               UINT8_C(3)
#define BMP280_STATUS_MEAS_MASK              UINT8_C(0x08)
#define BMP280_OS_TEMP_POS                   UINT8_C(5)
#define BMP280_OS_TEMP_MASK                  UINT8_C(0xE0)
#define BMP280_OS_PRES_POS                   UINT8_C(2)
#define BMP280_OS_PRES_MASK                  UINT8_C(0x1C)
#define BMP280_POWER_MODE_POS                UINT8_C(0)
#define BMP280_POWER_MODE_MASK               UINT8_C(0x03)
#define BMP280_STANDBY_DURN_POS              UINT8_C(5)
#define BMP280_STANDBY_DURN_MASK             UINT8_C(0xE0)
#define BMP280_FILTER_POS                    UINT8_C(2)
#define BMP280_FILTER_MASK                   UINT8_C(0x1C)
#define BMP280_SPI3_ENABLE_POS               UINT8_C(0)
#define BMP280_SPI3_ENABLE_MASK              UINT8_C(0x01)

/*! @name Calibration parameters' relative position */
#define BMP280_DIG_T1_LSB_POS                UINT8_C(0)
#define BMP280_DIG_T1_MSB_POS                UINT8_C(1)
#define BMP280_DIG_T2_LSB_POS                UINT8_C(2)
#define BMP280_DIG_T2_MSB_POS                UINT8_C(3)
#define BMP280_DIG_T3_LSB_POS                UINT8_C(4)
#define BMP280_DIG_T3_MSB_POS                UINT8_C(5)
#define BMP280_DIG_P1_LSB_POS                UINT8_C(6)
#define BMP280_DIG_P1_MSB_POS                UINT8_C(7)
#define BMP280_DIG_P2_LSB_POS                UINT8_C(8)
#define BMP280_DIG_P2_MSB_POS                UINT8_C(9)
#define BMP280_DIG_P3_LSB_POS                UINT8_C(10)
#define BMP280_DIG_P3_MSB_POS                UINT8_C(11)
#define BMP280_DIG_P4_LSB_POS                UINT8_C(12)
#define BMP280_DIG_P4_MSB_POS                UINT8_C(13)
#define BMP280_DIG_P5_LSB_POS                UINT8_C(14)
#define BMP280_DIG_P5_MSB_POS                UINT8_C(15)
#define BMP280_DIG_P6_LSB_POS                UINT8_C(16)
#define BMP280_DIG_P6_MSB_POS                UINT8_C(17)
#define BMP280_DIG_P7_LSB_POS                UINT8_C(18)
#define BMP280_DIG_P7_MSB_POS                UINT8_C(19)
#define BMP280_DIG_P8_LSB_POS                UINT8_C(20)
#define BMP280_DIG_P8_MSB_POS                UINT8_C(21)
#define BMP280_DIG_P9_LSB_POS                UINT8_C(22)
#define BMP280_DIG_P9_MSB_POS                UINT8_C(23)
#define BMP280_CALIB_DATA_SIZE               UINT8_C(24)

/*! @name Bit-slicing macros */
#define BMP280_GET_BITS(bitname, x)                    ((x & bitname##_MASK) \
                                                        >> bitname##_POS)
#define BMP280_SET_BITS(regvar, bitname, val)          ((regvar & \
                                                         ~bitname##_MASK) | ((val << bitname##_POS) & bitname##_MASK))
#define BMP280_SET_BITS_POS_0(reg_data, bitname, data) ((reg_data & \
                                                         ~(bitname##_MASK)) | (data & bitname##_MASK))
#define BMP280_GET_BITS_POS_0(bitname, reg_data)       (reg_data & \
                                                        (bitname##_MASK))

/*! @brief Macros holding the minimum and maximum for trimming values */

#define BMP280_ST_DIG_T1_MIN UINT16_C(19000)
#define BMP280_ST_DIG_T1_MAX UINT16_C(35000)
#define BMP280_ST_DIG_T2_MIN UINT16_C(22000)
#define BMP280_ST_DIG_T2_MAX UINT16_C(30000)
#define BMP280_ST_DIG_T3_MIN INT16_C(-3000)
#define BMP280_ST_DIG_T3_MAX INT16_C(-1000)
#define BMP280_ST_DIG_P1_MIN UINT16_C(30000)
#define BMP280_ST_DIG_P1_MAX UINT16_C(42000)
#define BMP280_ST_DIG_P2_MIN INT16_C(-12970)
#define BMP280_ST_DIG_P2_MAX INT16_C(-8000)
#define BMP280_ST_DIG_P3_MIN INT16_C(-5000)
#define BMP280_ST_DIG_P3_MAX UINT16_C(8000)
#define BMP280_ST_DIG_P4_MIN INT16_C(-10000)
#define BMP280_ST_DIG_P4_MAX UINT16_C(18000)
#define BMP280_ST_DIG_P5_MIN INT16_C(-500)
#define BMP280_ST_DIG_P5_MAX UINT16_C(1100)
#define BMP280_ST_DIG_P6_MIN INT16_C(-1000)
#define BMP280_ST_DIG_P6_MAX UINT16_C(1000)
#define BMP280_ST_DIG_P7_MIN INT16_C(-32768)
#define BMP280_ST_DIG_P7_MAX UINT16_C(32767)
#define BMP280_ST_DIG_P8_MIN INT16_C(-30000)
#define BMP280_ST_DIG_P8_MAX UINT16_C(10000)
#define BMP280_ST_DIG_P9_MIN INT16_C(-10000)
#define BMP280_ST_DIG_P9_MAX UINT16_C(30000)

#define BMP280_GET_BITSLICE(regvar, bitname) \
    ((regvar & bitname##__MSK) >> bitname##__POS)

/*! @brief Macros to read out API revision number */
/*Register holding custom trimming values */
#define BMP280_ST_TRIMCUSTOM_REG             UINT8_C(0x87)
#define BMP280_ST_TRIMCUSTOM_REG_APIREV__POS UINT8_C(1)
#define BMP280_ST_TRIMCUSTOM_REG_APIREV__MSK UINT8_C(0x06)
#define BMP280_ST_TRIMCUSTOM_REG_APIREV__LEN UINT8_C(2)
#define BMP280_ST_TRIMCUSTOM_REG_APIREV__REG BMP280_ST_TRIMCUSTOM_REG

/* highest API revision supported is revision 0. */
#define BMP280_ST_MAX_APIREVISION            UINT8_C(0x00)

/*! @brief Macros holding the minimum and maximum for trimming values */
/* 0x00000 is minimum output value */
#define BMP280_ST_ADC_T_MIN                  INT32_C(0x00000)

/* 0xFFFF0 is maximum 20-bit output value without over sampling */
#define BMP280_ST_ADC_T_MAX                  INT32_C(0xFFFF0)

/* 0x00000 is minimum output value */
#define BMP280_ST_ADC_P_MIN                  INT32_C(0x00000)

/* 0xFFFF0 is maximum 20-bit output value without over sampling */
#define BMP280_ST_ADC_P_MAX                  INT32_C(0xFFFF0)

/*! @name Calibration parameters' structure */
struct bmp280_calib_param
{
    uint16_t dig_t1;
    int16_t dig_t2;
    int16_t dig_t3;
    uint16_t dig_p1;
    int16_t dig_p2;
    int16_t dig_p3;
    int16_t dig_p4;
    int16_t dig_p5;
    int16_t dig_p6;
    int16_t dig_p7;
    int16_t dig_p8;
    int16_t dig_p9;
    int32_t t_fine;
};

/*! @name Sensor configuration structure */
struct bmp280_config
{
    uint8_t os_temp;
    uint8_t os_pres;
    uint8_t odr;
    uint8_t filter;
    uint8_t spi3w_en;
};

/*! @name Sensor status structure */
struct bmp280_status
{
    uint8_t measuring;
    uint8_t im_update;
};

/*! @name Uncompensated data structure */
struct bmp280_uncomp_data
{
    int32_t uncomp_temp;
    uint32_t uncomp_press;
};

//*****************************************************************************
//
// External function definitions
//
//*****************************************************************************
extern void bmp280_init(void);
void bmp280_get_config(void);
void bmp280_set_config(void);
void bmp280_get_status(struct bmp280_status *status);
void bmp280_get_power_mode(uint8_t *mode);
void bmp280_get_uncomp_data(struct bmp280_uncomp_data *uncomp_data);
void bmp280_get_comp_temp_32bit(int32_t *comp_temp, int32_t uncomp_temp);
void bmp280_get_comp_pres_32bit(uint32_t *comp_pres, uint32_t uncomp_pres);
void bmp280_set_power_mode(uint8_t mode);

#ifdef __cplusplus
}
#endif

#endif // APOLLO_BMP280_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
