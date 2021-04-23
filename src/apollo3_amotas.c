//*****************************************************************************
//
//! @file apollo3_amotas.c
//!
//! @brief This file provides the main application for the AMOTA service.
//!
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2020, Thundercomm
// All rights reserved.
//
//*****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "bstream.h"
#include "am_mcu_apollo.h"

#include "am_multi_boot.h"
#include "apollo_tracelog.h"
#include "apollo3_amotas.h"

#define AMOTAS_TEMP_BUFSIZE     AM_HAL_FLASH_PAGE_SIZE

// Protection against NULL pointer
#define FLASH_OPERATE(pFlash, func) ((pFlash)->func ? (pFlash)->func() : 0)

static am_multiboot_flash_info_t *g_pFlash = &g_intFlash;
uint8_t amotaStart = 0;

// Data structure for flash operation
typedef struct
{
    uint8_t     writeBuffer[AMOTAS_TEMP_BUFSIZE]   __attribute__((aligned(4)));   // needs to be 32-bit word aligned.
    uint16_t    bufferIndex;
}amotasFlashOp_t;

amotasFlashOp_t amotasFlash = {
    .bufferIndex = 0,
};

// Temporary scratch buffer used to read from flash
uint32_t amotasTmpBuf[AMOTA_PACKET_SIZE / 4];

// amota states
typedef enum
{
    AMOTA_STATE_INIT,
    AMOTA_STATE_GETTING_FW,
    AMOTA_STATE_MAX
}eAmotaState;

// amota commands
typedef enum
{
    AMOTA_CMD_UNKNOWN,
    AMOTA_CMD_FW_HEADER,
    AMOTA_CMD_FW_DATA,
    AMOTA_CMD_FW_VERIFY,
    AMOTA_CMD_FW_RESET,
    AMOTA_CMD_MAX
}eAmotaCommand;

// amota status
typedef enum
{
    AMOTA_STATUS_SUCCESS,
    AMOTA_STATUS_CRC_ERROR,
    AMOTA_STATUS_INVALID_HEADER_INFO,
    AMOTA_STATUS_INVALID_PKT_LENGTH,
    AMOTA_STATUS_INSUFFICIENT_BUFFER,
    AMOTA_STATUS_INSUFFICIENT_FLASH,
    AMOTA_STATUS_UNKNOWN_ERROR,
    AMOTA_STATUS_FLASH_WRITE_ERROR,
    AMOTA_STATUS_MAX
}eAmotaStatus;

// FW header information
typedef struct
{
    uint32_t    encrypted;
    uint32_t    fwStartAddr;            // Address to install the image
    uint32_t    fwLength;
    uint32_t    fwCrc;
    uint32_t    secInfoLen;
    uint32_t    resvd1;
    uint32_t    resvd2;
    uint32_t    resvd3;
    uint32_t    version;
    uint32_t    fwDataType;             //binary type
    uint32_t    storageType;
    uint32_t    resvd4;

}
amotaHeaderInfo_t;

// Firmware Address
typedef struct
{
    uint32_t    addr;
    uint32_t    offset;
}
amotasNewFwFlashInfo_t;

/* Control block */
static struct
{
    eAmotaState             state;
    amotaHeaderInfo_t       fwHeader;
    amotasNewFwFlashInfo_t  newFwFlashInfo;
}
amotasCb;

// Erases the flash based on ui32Addr & ui32NumBytes
void erase_flash(uint32_t ui32Addr, uint32_t ui32NumBytes)
{
    // Erase the image
    while ( ui32NumBytes )
    {
        g_pFlash->flash_erase_sector(ui32Addr);
        if ( ui32NumBytes > g_pFlash->flashSectorSize )
        {
            ui32NumBytes -= g_pFlash->flashSectorSize;
            ui32Addr += g_pFlash->flashSectorSize;
        }
        else
        {
            break;
        }
    }
}

//*****************************************************************************
//
// Set Firmware Address
//
// return true if success, otherwise false
//
//*****************************************************************************
static bool amotas_set_fw_addr(void)
{
    bool bResult = false;

    amotasCb.newFwFlashInfo.addr = 0;
    amotasCb.newFwFlashInfo.offset = 0;

    // Check storage type
    if ( amotasCb.fwHeader.storageType == AMOTA_FW_STORAGE_INTERNAL )
    {
        // storage in internal flash
        uint32_t storeAddr = (AMOTA_INT_FLASH_OTA_ADDRESS + AM_HAL_FLASH_PAGE_SIZE - 1) & ~(AM_HAL_FLASH_PAGE_SIZE - 1);
        uint32_t maxSize = AMOTA_INT_FLASH_OTA_MAX_SIZE & ~(AM_HAL_FLASH_PAGE_SIZE - 1);

        // Check to make sure the incoming image will fit in the space allocated for OTA
        if (amotasCb.fwHeader.fwLength > maxSize)
        {
            PR_INFO("not enough OTA space allocated = %d bytes, Desired = %d bytes",
                maxSize, amotasCb.fwHeader.fwLength);
            return false;
        }

        g_pFlash = &g_intFlash;
        amotasCb.newFwFlashInfo.addr = storeAddr;
        bResult = true;
    }
    else if ( amotasCb.fwHeader.storageType == AMOTA_FW_STORAGE_EXTERNAL )
    {
        //storage in external flash

        {
            bResult = false;
        }
    }
    else
    {
        // reserved state
        bResult = false;
    }
    if (bResult == true)
    {
        // Initialize the flash device.
        if (FLASH_OPERATE(g_pFlash, flash_init) == 0)
        {
            if (FLASH_OPERATE(g_pFlash, flash_enable) != 0)
            {
                FLASH_OPERATE(g_pFlash, flash_deinit);
                bResult = false;
            }
            // Erase necessary sectors in the flash according to length of the image.
            erase_flash(amotasCb.newFwFlashInfo.addr, amotasCb.fwHeader.fwLength);

            FLASH_OPERATE(g_pFlash, flash_disable);
        }
        else
        {
            bResult = false;
        }
    }
    return bResult;
}

static int verify_flash_content(uint32_t flashAddr, uint32_t *pSram, uint32_t len, am_multiboot_flash_info_t *pFlash)
{
    // read back and check
    uint32_t  offset = 0;
    uint32_t  remaining = len;
    int       ret = 0;
    while (remaining)
    {
        uint32_t tmpSize =
            (remaining > AMOTA_PACKET_SIZE) ? AMOTA_PACKET_SIZE : remaining;
        pFlash->flash_read_page((uint32_t)amotasTmpBuf, (uint32_t *)(flashAddr + offset), tmpSize);

        ret = memcmp(amotasTmpBuf, (uint8_t*)((uint32_t)pSram + offset), tmpSize);

        if ( ret != 0 )
        {
            // there is write failure happened.
            PR_INFO("flash write verify failed. address 0x%x. length %d", flashAddr, len);
            break;
        }
        offset += tmpSize;
        remaining -= tmpSize;
    }
    return ret;
}

//*****************************************************************************
//
// Write to Flash
//
// return true if success, otherwise false
//
//*****************************************************************************
static bool amotas_write2flash(uint16_t len, uint8_t *buf, uint32_t addr, bool lastPktFlag)
{
    uint16_t ui16BytesRemaining = len;
    uint32_t ui32TargetAddress = 0;
    uint8_t ui8PageCount = 0;
    bool bResult = true;
    uint16_t i;

    addr -= amotasFlash.bufferIndex;

    // Check the target flash address to ensure we do not operation the wrong address
    // make sure to write to page boundary
    if (((uint32_t)amotasCb.newFwFlashInfo.addr > addr) ||
        (addr & (g_pFlash->flashPageSize - 1)))
    {
        // application is trying to write to wrong address
        return false;
    }

    FLASH_OPERATE(g_pFlash, flash_enable);
    while (ui16BytesRemaining)
    {
        uint16_t ui16Bytes2write = g_pFlash->flashPageSize - amotasFlash.bufferIndex;
        if (ui16Bytes2write > ui16BytesRemaining)
        {
            ui16Bytes2write = ui16BytesRemaining;
        }
        // move data into buffer
        for ( i = 0; i < ui16Bytes2write; i++ )
        {
            // avoid using memcpy
            amotasFlash.writeBuffer[amotasFlash.bufferIndex++] = buf[i];
        }
        ui16BytesRemaining -= ui16Bytes2write;
        buf += ui16Bytes2write;

        // Write to flash when there is data more than 1 page size
        // For last fragment write even if it is less than one page
        if (lastPktFlag || (amotasFlash.bufferIndex == g_pFlash->flashPageSize))
        {
            ui32TargetAddress = (addr + ui8PageCount*g_pFlash->flashPageSize);

            // Always write whole pages
            if ((g_pFlash->flash_write_page(ui32TargetAddress, (uint32_t *)amotasFlash.writeBuffer, g_pFlash->flashPageSize) != 0)
                || (verify_flash_content(ui32TargetAddress, (uint32_t *)amotasFlash.writeBuffer, amotasFlash.bufferIndex, g_pFlash) != 0))
            {
                bResult = false;
                break;
            }

            PR_INFO("Flash write succeeded to address 0x%x. length %d", ui32TargetAddress, amotasFlash.bufferIndex);
            ui8PageCount++;
            amotasFlash.bufferIndex = 0;
            bResult = true;
        }
    }
    FLASH_OPERATE(g_pFlash, flash_disable);

    // If we get here, operations are done correctly
    return bResult;
}

//*****************************************************************************
//
// Verify Firmware Image CRC
//
//return true if success, otherwise false
//*****************************************************************************
static bool amotas_verify_firmware_crc(void)
{
    // read back the whole firmware image from flash and calculate CRC
    uint32_t ui32CRC = 0;

    //
    // Check crc in external flash
    //
    FLASH_OPERATE(g_pFlash, flash_enable);

    // read from spi flash and calculate CRC32
    for ( uint16_t i = 0; i < (amotasCb.fwHeader.fwLength / AMOTA_PACKET_SIZE); i++ )
    {
        g_pFlash->flash_read_page((uint32_t)amotasTmpBuf,
            (uint32_t *)(amotasCb.newFwFlashInfo.addr + i*AMOTA_PACKET_SIZE),
            AMOTA_PACKET_SIZE);

        am_bootloader_partial_crc32(amotasTmpBuf, AMOTA_PACKET_SIZE, &ui32CRC);
    }

    uint32_t ui32Remainder = amotasCb.fwHeader.fwLength % AMOTA_PACKET_SIZE;
    if ( ui32Remainder )
    {
        g_pFlash->flash_read_page((uint32_t)amotasTmpBuf,
            (uint32_t *)(amotasCb.newFwFlashInfo.addr + amotasCb.fwHeader.fwLength - ui32Remainder),
            ui32Remainder);

        am_bootloader_partial_crc32(amotasTmpBuf, ui32Remainder, &ui32CRC);
    }


    FLASH_OPERATE(g_pFlash, flash_disable);

    return (ui32CRC == amotasCb.fwHeader.fwCrc);
}

//*****************************************************************************
//
// Update OTA information with Firmware Information.
//
//*****************************************************************************
static void amotas_update_ota(void)
{
    uint8_t  magic = *((uint8_t *)(amotasCb.newFwFlashInfo.addr + 3));
    PR_ERR("amotas_update_ota: magic: %d, addr: %x", magic, amotasCb.newFwFlashInfo.addr);

    // Set OTAPOINTER
    am_hal_ota_add(AM_HAL_FLASH_PROGRAM_KEY, magic, (uint32_t *)amotasCb.newFwFlashInfo.addr);
}

static void amotas_init_ota(void)
{
    uint32_t *pOtaDesc = (uint32_t *)(OTA_POINTER_LOCATION & ~(AM_HAL_FLASH_PAGE_SIZE - 1));
    // Initialize OTA descriptor - This should ideally be initiated through a separate command
    // to facilitate multiple image upgrade in a single reboot
    // Will need change in the AMOTA app to do so
    am_hal_ota_init(AM_HAL_FLASH_PROGRAM_KEY, pOtaDesc);
}

//*****************************************************************************
//
// Handle the various packet types from the Client
//
//*****************************************************************************
void amotas_packet_handler(eAmotaCommand cmd, uint16_t len, uint8_t *buf)
{
    eAmotaStatus status = AMOTA_STATUS_SUCCESS;
    bool resumeTransfer = false;
    bool bResult = false;
    uint8_t data[4] = {0};
    status = status;
    resumeTransfer = resumeTransfer;
    data[0] = data[1];
    
    switch(cmd)
    {
        case AMOTA_CMD_FW_HEADER:
            if (len < AMOTA_FW_HEADER_SIZE)
            {
                status = AMOTA_STATUS_INVALID_HEADER_INFO;
                // replay to client
                break;
            }

            BYTES_TO_UINT32(amotasCb.fwHeader.encrypted, buf);
            BYTES_TO_UINT32(amotasCb.fwHeader.fwStartAddr, buf + 4);
            BYTES_TO_UINT32(amotasCb.fwHeader.fwLength, buf + 8);
            BYTES_TO_UINT32(amotasCb.fwHeader.fwCrc, buf + 12);
            BYTES_TO_UINT32(amotasCb.fwHeader.secInfoLen, buf + 16);
            BYTES_TO_UINT32(amotasCb.fwHeader.version, buf + 32);
            BYTES_TO_UINT32(amotasCb.fwHeader.fwDataType, buf + 36);
            BYTES_TO_UINT32(amotasCb.fwHeader.storageType, buf + 40);

            PR_DBG("OTA process start from beginning");
            amotasFlash.bufferIndex = 0;
            bResult = amotas_set_fw_addr();

            if ( bResult == false )
            {
                PR_DBG("Header FALSE");
                // amotas_reply_to_client(cmd, AMOTA_STATUS_INSUFFICIENT_FLASH, NULL, 0);
                amotasCb.state = AMOTA_STATE_INIT;
                return;
            }

            data[0] = ((amotasCb.newFwFlashInfo.offset) & 0xff);
            data[1] = ((amotasCb.newFwFlashInfo.offset >> 8) & 0xff);
            data[2] = ((amotasCb.newFwFlashInfo.offset >> 16) & 0xff);
            data[3] = ((amotasCb.newFwFlashInfo.offset >> 24) & 0xff);
            // amotas_reply_to_client(cmd, AMOTA_STATUS_SUCCESS, data, sizeof(data));
            break;

        case AMOTA_CMD_FW_DATA:
            bResult = amotas_write2flash(len, buf, amotasCb.newFwFlashInfo.addr + amotasCb.newFwFlashInfo.offset,
                        ((amotasCb.newFwFlashInfo.offset + len) == amotasCb.fwHeader.fwLength));

            if ( bResult == false )
            {
                PR_DBG("Data FALSE");
                data[0] = ((amotasCb.newFwFlashInfo.offset) & 0xff);
                data[1] = ((amotasCb.newFwFlashInfo.offset >> 8) & 0xff);
                data[2] = ((amotasCb.newFwFlashInfo.offset >> 16) & 0xff);
                data[3] = ((amotasCb.newFwFlashInfo.offset >> 24) & 0xff);
                // amotas_reply_to_client(cmd, AMOTA_STATUS_FLASH_WRITE_ERROR, data, sizeof(data));
            }
            else
            {
                amotasCb.newFwFlashInfo.offset += len;

                data[0] = ((amotasCb.newFwFlashInfo.offset) & 0xff);
                data[1] = ((amotasCb.newFwFlashInfo.offset >> 8) & 0xff);
                data[2] = ((amotasCb.newFwFlashInfo.offset >> 16) & 0xff);
                data[3] = ((amotasCb.newFwFlashInfo.offset >> 24) & 0xff);
                // amotas_reply_to_client(cmd, AMOTA_STATUS_SUCCESS, data, sizeof(data));
            }
            break;

        case AMOTA_CMD_FW_VERIFY:
            if (amotas_verify_firmware_crc())
            {
                PR_INFO("crc verify success");

                // amotas_reply_to_client(cmd, AMOTA_STATUS_SUCCESS, NULL, 0);

                //
                // Update flash flag page here
                //
                amotas_update_ota();
            }
            else
            {
                PR_INFO("crc verify failed");
                // amotas_reply_to_client(cmd, AMOTA_STATUS_CRC_ERROR, NULL, 0);
            }
            FLASH_OPERATE(g_pFlash, flash_deinit);
            amotasCb.state = AMOTA_STATE_INIT;
            g_pFlash = &g_intFlash;
            break;

        case AMOTA_CMD_FW_RESET:
            PR_INFO("Apollo will reset in 500ms.");
            am_hal_reset_control(AM_HAL_RESET_CONTROL_SWPOI, 0);
            // amotas_reply_to_client(cmd, AMOTA_STATUS_SUCCESS, NULL, 0);

            // Delay here to let packet go through the RF before we disconnect
            // WsfTimerStartMs(&amotasCb.disconnectTimer, 500);
        break;

        default:
            break;
    }
}

//*****************************************************************************
//
//! @brief initialize amota service
//!
//! @param handlerId - connection handle
//! @param pCfg - configuration parameters
//!
//! @return None
//
//*****************************************************************************
void amotas_init(void)
{
    memset(&amotasCb, 0, sizeof(amotasCb));
    amotasCb.state = AMOTA_STATE_INIT;
    amotas_init_ota();
    amotaStart = 1;
}

static char *otaStatusMessage[] =
    {
        "Success",
        "Error",
        "Failure",
        "Pending"
    };
void dump_ota_status(void)
{
    uint32_t *pOtaDesc = (uint32_t *)(OTA_POINTER_LOCATION & ~(AM_HAL_FLASH_PAGE_SIZE - 1));
    uint32_t i;

    // Check if the current content at OTA descriptor is valid
    for (i = 0; i < AM_HAL_SECURE_OTA_MAX_OTA + 1; i++)
    {
        // Make sure the image address looks okay
        if (pOtaDesc[i] == 0xFFFFFFFF)
        {
            break;
        }
        if (((pOtaDesc[i] & 0x3) == AM_HAL_OTA_STATUS_ERROR) || ((pOtaDesc[i] & ~0x3) >= 0x100000))
        {
            break;
        }
    }
    if (pOtaDesc[i] == 0xFFFFFFFF)
    {
        PR_INFO("Valid Previous OTA state,i=%d", i);
        // It seems in last boot this was used as OTA descriptor
        // Dump previous OTA information
        am_hal_ota_status_t otaStatus[AM_HAL_SECURE_OTA_MAX_OTA];
        am_hal_get_ota_status(pOtaDesc, AM_HAL_SECURE_OTA_MAX_OTA, otaStatus);
        for ( uint32_t i = 0; i < AM_HAL_SECURE_OTA_MAX_OTA; i++ )
        {
            if ((uint32_t)otaStatus[i].pImage == 0xFFFFFFFF)
            {
                break;
            }
            {
                PR_INFO("Previous OTA: Blob Addr: 0x%x - Result %s",
                                     otaStatus[i].pImage, otaStatusMessage[otaStatus[i].status]);
            }
        }
    }
    else
    {
        PR_ERR("No Previous OTA state");
    }
}

void amotas_cback(uint8_t cmd, uint16_t len, uint8_t *buf)
{
    amotas_packet_handler((eAmotaCommand)cmd, len, buf);
}

void distribute_pack(uint8_t len, uint8_t *buf, uint8_t isEndPack)
{
    static uint8_t firstPack = 0;
    static uint16_t otaBufLen = 0;
    static uint8_t packBuf[0x200] = {0};

    while(len)
    {
        if (firstPack)
        {
            // Data packet
            if ((len + otaBufLen) >= AMOTA_FW_DATA_PACKAGE)
            {
                memcpy(&packBuf[otaBufLen], buf, AMOTA_FW_DATA_PACKAGE - otaBufLen);
                amotas_packet_handler(AMOTA_CMD_FW_DATA, AMOTA_FW_DATA_PACKAGE, packBuf);
                len -= (AMOTA_FW_DATA_PACKAGE - otaBufLen);
                buf += (AMOTA_FW_DATA_PACKAGE - otaBufLen);
                otaBufLen = 0;
            }
            else
            {
                memcpy(&packBuf[otaBufLen], buf, len);
                otaBufLen += len;
                len = 0;
            }
        }
        else
        {
            if((len + otaBufLen) >= AMOTA_FW_HEADER_PACKAGE)
            {
                memcpy(&packBuf[otaBufLen], buf, AMOTA_FW_HEADER_PACKAGE - otaBufLen);
                amotas_packet_handler(AMOTA_CMD_FW_HEADER, AMOTA_FW_HEADER_PACKAGE, packBuf);
                firstPack = 1;
                len -= (AMOTA_FW_HEADER_PACKAGE - otaBufLen);
                buf += (AMOTA_FW_HEADER_PACKAGE - otaBufLen);
                otaBufLen = 0;
            }
            else
            {
                memcpy(&packBuf[otaBufLen], buf, len);
                otaBufLen += len;
                len = 0;
            }
        }
    }

    if(isEndPack)
    {
        if (otaBufLen)
            amotas_packet_handler(AMOTA_CMD_FW_DATA, otaBufLen, packBuf);

        amotas_packet_handler(AMOTA_CMD_FW_VERIFY, 0, NULL);
        amotas_packet_handler(AMOTA_CMD_FW_RESET, 0, NULL);

        while(1);
    }
}
