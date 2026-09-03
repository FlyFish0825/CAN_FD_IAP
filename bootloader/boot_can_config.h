#ifndef __BOOT_CAN_CONFIG_H
#define __BOOT_CAN_CONFIG_H

#include <stdint.h>

/* ========================= Device / Flash layout ========================= */
#define BOOT_FLASH_BASE_ADDR          0x08000000UL
#define BOOT_FLASH_SIZE_BYTES         (128UL * 1024UL)
#define BOOT_FLASH_END_ADDR           (BOOT_FLASH_BASE_ADDR + BOOT_FLASH_SIZE_BYTES - 1UL)

#define BOOT_FLASH_PAGE_SIZE          2048UL
#define BOOT_FLASH_PAGE_COUNT         (BOOT_FLASH_SIZE_BYTES / BOOT_FLASH_PAGE_SIZE)

#define BOOT_BOOTLOADER_START_ADDR    0x08000000UL
#define BOOT_BOOTLOADER_SIZE_BYTES    (20UL * 1024UL)
#define BOOT_BOOTLOADER_END_ADDR      (BOOT_BOOTLOADER_START_ADDR + BOOT_BOOTLOADER_SIZE_BYTES - 1UL)

#define BOOT_APP_START_ADDR           0x08005000UL
#define BOOT_CONFIG_PAGE_ADDR         0x0801F800UL
#define BOOT_CONFIG_PAGE_SIZE         2048UL
#define BOOT_CONFIG_PAGE_END_ADDR     (BOOT_CONFIG_PAGE_ADDR + BOOT_CONFIG_PAGE_SIZE - 1UL)
#define BOOT_APP_END_ADDR             (BOOT_CONFIG_PAGE_ADDR - 1UL)
#define BOOT_APP_MAX_SIZE             (BOOT_CONFIG_PAGE_ADDR - BOOT_APP_START_ADDR)

#define BOOT_APP_FIRST_PAGE           ((BOOT_APP_START_ADDR - BOOT_FLASH_BASE_ADDR) / BOOT_FLASH_PAGE_SIZE)
#define BOOT_APP_PAGE_COUNT           (BOOT_APP_MAX_SIZE / BOOT_FLASH_PAGE_SIZE)
#define BOOT_CONFIG_PAGE_INDEX        ((BOOT_CONFIG_PAGE_ADDR - BOOT_FLASH_BASE_ADDR) / BOOT_FLASH_PAGE_SIZE)

/* STM32G431: 32 KiB SRAM is addressable contiguously through the 0x20000000 alias. */
#define BOOT_SRAM_ALIAS_START_ADDR    0x20000000UL
#define BOOT_SRAM_ALIAS_END_ADDR      0x20008000UL
#define BOOT_CCM_START_ADDR           0x10000000UL
#define BOOT_CCM_END_ADDR             0x10002800UL

/* ========================= CAN identifiers =============================== */
#define BOOT_CAN_CMD_ID               0x000U
#define BOOT_CAN_FD_DATA_ID           0x100U
#define BOOT_CAN_RESP_BASE_ID         0x500U

#define BOOT_MAX_NODE_NUM             8U
#define BOOT_DEFAULT_NODE_ID          1U
#define BOOT_BROADCAST_ID             0xFFU

#define BOOT_CAN_CTRL_DLC             8U
#define BOOT_CAN_FD_DLC               64U
#define BOOT_CAN_FD_PAYLOAD_SIZE      56U

/* STM32G4 HAL uses a fixed 3-element Tx FIFO/Queue in Message RAM. */
#define BOOT_FDCAN_TX_FIFO_DEPTH      3U

/* ========================= Protocol version ============================== */
#define BOOT_VERSION_MAJOR            1U
#define BOOT_VERSION_MINOR            0U
#define BOOT_VERSION_PATCH            0U
#define BOOT_VERSION_BUILD            0U

/* ========================= CRC =========================================== */
/* Control frame Byte7: CRC-8/ATM (poly 0x07, init 0x00, refin=false, xorout=0). */
#define BOOT_CTRL_CRC8_POLY           0x07U
#define BOOT_CTRL_CRC8_INIT           0x00U

/* APP/config CRC: STM32 hardware CRC-32 (poly 0x04C11DB7, init 0xFFFFFFFF,
 * refin=false, refout=false, xorout=0), equivalent to CRC-32/MPEG-2 byte stream.
 */
#define BOOT_CRC32_POLY               0x04C11DB7UL
#define BOOT_CRC32_INIT               0xFFFFFFFFUL

/* ========================= Persistent config ============================= */
#define BOOT_CONFIG_MAGIC             0x31474643UL /* "CFG1" little-endian */
#define BOOT_CONFIG_VERSION           1U

/* ========================= APP entry request ============================= */
#define BOOT_REQUEST_MAGIC            0x544F4F42UL /* "BOOT" little-endian */

/* ========================= Update/data plane ============================= */
#define BOOT_WRITE_REGION_APP         0x00U
#define BOOT_WRITE_REGION_CONFIG      0x01U
#define BOOT_WRITE_REGION_BOOTLOADER  0x02U /* reserved; rejected in v1 */

#define BOOT_FD_CMD_WRITE_DATA        0x01U

#define BOOT_MAX_PACKET_COUNT         ((BOOT_APP_MAX_SIZE + BOOT_CAN_FD_PAYLOAD_SIZE - 1UL) / BOOT_CAN_FD_PAYLOAD_SIZE)
#define BOOT_BITMAP_SIZE_BYTES        ((BOOT_MAX_PACKET_COUNT + 7UL) / 8UL)

#endif /* __BOOT_CAN_CONFIG_H */
