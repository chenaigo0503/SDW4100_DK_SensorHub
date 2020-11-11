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

//*****************************************************************************
//
// External function definitions
//
//*****************************************************************************
extern void bmp280_init(void);

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
