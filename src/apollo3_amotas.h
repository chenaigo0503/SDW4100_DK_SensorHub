//*****************************************************************************
//
//! @file apollo3_amotas.h
//!
//! @brief This file provides the main api and macro for the AMOTA service.
//!
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2020, Thundercomm
// All rights reserved.
//
//*****************************************************************************
#ifndef APOLLO3_AMOTAS_H
#define APOLLO3_AMOTAS_H

// Macro definitions at amota_profile_config.h

// Note: Internal flash area to be used for OTA temporary storage
// The address must be aligned to flash page
// This should be customized to the desired memory map of the design
#define AMOTA_INT_FLASH_OTA_ADDRESS         0x00050000

// User specified maximum size of OTA storage area.
// Make sure the size is flash page multiple
// (Default value is determined based on rest of flash from the start)
//
#define AMOTA_INT_FLASH_OTA_MAX_SIZE        (AM_HAL_FLASH_LARGEST_VALID_ADDR - AMOTA_INT_FLASH_OTA_ADDRESS + 1)

// OTA Descriptor address by reserving 256K bytes app image size
// OTA Descriptor only need one page which is 8K bytes
#define OTA_POINTER_LOCATION                0x4C000

// end

//*****************************************************************************
//
// Macro definitions
//
//*****************************************************************************
#define AMOTA_PACKET_SIZE           (512 + 16)    // Bytes

#define AMOTA_FW_HEADER_SIZE        44
#define AMOTA_FW_HEADER_PACKAGE     0x30
#define AMOTA_FW_DATA_PACKAGE       0x200

#define AMOTA_FW_STORAGE_INTERNAL   0
#define AMOTA_FW_STORAGE_EXTERNAL   1

//*****************************************************************************
//
// Handle the various packet types from the Client
//
//*****************************************************************************
void amotas_init(void);
void dump_ota_status(void);
void amotas_cback(uint8_t cmd, uint16_t len, uint8_t *buf);
void distribute_pack(uint8_t len, uint8_t *buf, uint8_t isEndPack);

#endif // APOLLO3_AMOTAS_H
