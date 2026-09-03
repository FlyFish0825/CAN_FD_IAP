#ifndef __BOOTLOADER_H
#define __BOOTLOADER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * STM32G431 Bootloader - transport independent core
 *
 * The upper Bootloader and the lower communication driver exchange ONLY
 * Boot_Message_t.  The SAME structure is used for RX and TX.
 *
 * The core does not include fdcan.h/usart.h/spi.h/i2c.h and does not know
 * CAN IDs, UART framing, SPI chip-selects, etc.
 * ======================================================================== */

/* -------------------------- Flash layout -------------------------------- */
#define BOOT_FLASH_BASE_ADDR          0x08000000UL
#define BOOT_FLASH_SIZE_BYTES         (128UL * 1024UL)
#define BOOT_FLASH_END_ADDR           (BOOT_FLASH_BASE_ADDR + BOOT_FLASH_SIZE_BYTES - 1UL)
#define BOOT_FLASH_PAGE_SIZE          2048UL

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

#define BOOT_SRAM_ALIAS_START_ADDR    0x20000000UL
#define BOOT_SRAM_ALIAS_END_ADDR      0x20008000UL
#define BOOT_CCM_START_ADDR           0x10000000UL
#define BOOT_CCM_END_ADDR             0x10002800UL

/* -------------------------- Protocol ------------------------------------ */
#define BOOT_MAX_NODE_NUM             8U
#define BOOT_DEFAULT_NODE_ID          1U
#define BOOT_BROADCAST_ID             0xFFU

#define BOOT_CONTROL_SIZE             8U
#define BOOT_DATA_SIZE                64U
#define BOOT_DATA_PAYLOAD_SIZE        56U
#define BOOT_DATA_CMD_WRITE           0x01U

#define BOOT_MAX_PACKET_COUNT         ((BOOT_APP_MAX_SIZE + BOOT_DATA_PAYLOAD_SIZE - 1UL) / BOOT_DATA_PAYLOAD_SIZE)
#define BOOT_BITMAP_SIZE_BYTES        ((BOOT_MAX_PACKET_COUNT + 7UL) / 8UL)

#define BOOT_VERSION_MAJOR            1U
#define BOOT_VERSION_MINOR            0U
#define BOOT_VERSION_PATCH            0U
#define BOOT_VERSION_BUILD            0U

#define BOOT_CTRL_CRC8_POLY           0x07U
#define BOOT_CTRL_CRC8_INIT           0x00U
#define BOOT_CRC32_POLY               0x04C11DB7UL
#define BOOT_CRC32_INIT               0xFFFFFFFFUL

#define BOOT_CONFIG_MAGIC             0x31474643UL /* CFG1 */
#define BOOT_CONFIG_VERSION           1U
#define BOOT_REQUEST_MAGIC            0x544F4F42UL /* BOOT */

#define BOOT_WRITE_REGION_APP         0x00U
#define BOOT_WRITE_REGION_CONFIG      0x01U
#define BOOT_WRITE_REGION_BOOTLOADER  0x02U /* reserved / protected */

/* RX queue stores complete logical messages. Boot_Input() is intentionally
 * lightweight so a transport ISR may call it after it has assembled a full
 * message. Heavy protocol/Flash work remains in Boot_Task().
 */
#define BOOT_RX_QUEUE_SIZE            16U

/* -------------------- The ONLY link-layer exchange object --------------- */
typedef enum
{
    BOOT_MESSAGE_CONTROL = 0U,  /* 8-byte command / response */
    BOOT_MESSAGE_DATA    = 1U,  /* 64-byte firmware data packet */
} Boot_MessageType_t;

typedef struct
{
    uint8_t type;               /* Boot_MessageType_t */
    uint16_t len;
    uint8_t data[BOOT_DATA_SIZE];
} Boot_Message_t;

/* Lower-layer TX callback. Return 1 when accepted for transmission, 0 when
 * busy/error.  The exact transport mapping is implemented outside bootloader/.
 */
typedef uint8_t (*Boot_SendCallback_t)(const Boot_Message_t *message,
                                       void *user);

/* Optional: wait until the physical transport has finished queued TX before
 * RESET/JUMP_APP.  May be NULL if the lower layer does not need it.
 */
typedef void (*Boot_FlushCallback_t)(void *user, uint32_t timeout_ms);

/* -------------------------- Commands ------------------------------------ */
typedef enum
{
    BOOT_CMD_GET_VERSION      = 0x01,
    BOOT_CMD_GET_DEVICE_ID    = 0x02,
    BOOT_CMD_GET_INFO         = 0x03,
    BOOT_CMD_ENTER_BOOT       = 0x04,
    BOOT_CMD_SET_GUARD        = 0x05,
    BOOT_CMD_RELEASE_GUARD    = 0x06,

    BOOT_CMD_ERASE            = 0x10,
    BOOT_CMD_WRITE            = 0x11,
    BOOT_CMD_READ             = 0x12,
    BOOT_CMD_VERIFY           = 0x13,
    BOOT_CMD_WRITE_END        = 0x14,
    BOOT_CMD_MISSING_COUNT    = 0x15,
    BOOT_CMD_MISSING_ITEM     = 0x16,
    BOOT_CMD_PROVIDER_GRANT   = 0x17,
    BOOT_CMD_ABORT            = 0x18,

    BOOT_CMD_JUMP_APP         = 0x20,
    BOOT_CMD_RESET            = 0x21,
    BOOT_CMD_GET_STATUS       = 0x30,
} Boot_Command_t;

typedef enum
{
    BOOT_STATUS_IDLE          = 0x00,
    BOOT_STATUS_ERASE         = 0x01,
    BOOT_STATUS_WRITE         = 0x02,
    BOOT_STATUS_VERIFY        = 0x03,
    BOOT_STATUS_READY         = 0x04,
    BOOT_STATUS_ERROR         = 0x05,
    BOOT_STATUS_REPAIR        = 0x06,
    BOOT_STATUS_GUARD         = 0x07,
} Boot_Status_t;

typedef enum
{
    BOOT_ERR_NONE             = 0x00,
    BOOT_ERR_BAD_CRC          = 0x01,
    BOOT_ERR_BAD_LENGTH       = 0x02,
    BOOT_ERR_BAD_ADDRESS      = 0x03,
    BOOT_ERR_BAD_STATE        = 0x04,
    BOOT_ERR_FLASH_ERASE      = 0x05,
    BOOT_ERR_FLASH_WRITE      = 0x06,
    BOOT_ERR_CONFIG           = 0x07,
    BOOT_ERR_APP_INVALID      = 0x08,
    BOOT_ERR_SIZE             = 0x09,
    BOOT_ERR_GUARD_PROTECTED  = 0x0A,
    BOOT_ERR_SEQUENCE         = 0x0B,
    BOOT_ERR_CRC_MISMATCH     = 0x0C,
    BOOT_ERR_PROVIDER_SOURCE  = 0x0D,
    BOOT_ERR_BUSY             = 0x0E,
    BOOT_ERR_PROTECTED_REGION = 0x0F,
    BOOT_ERR_ABORTED          = 0x10,
    BOOT_ERR_RX_OVERFLOW      = 0x11,
} Boot_Error_t;

/* Persistent PCB calibration + APP metadata. */
typedef struct
{
    uint32_t magic;
    uint16_t config_version;
    uint16_t length;

    uint8_t  node_id;
    uint8_t  hardware_version;
    uint16_t reserved0;

    uint16_t current_offset_a;
    uint16_t current_offset_b;
    uint16_t current_offset_c;
    uint16_t vbus_offset;

    float current_gain_a;
    float current_gain_b;
    float current_gain_c;
    float vbus_gain;

    uint32_t app_size;
    uint32_t app_crc32;
    uint8_t  app_valid;
    uint8_t  reserved1[3];

    uint32_t reserved[8];
    uint32_t crc32;
} Boot_Config_t;

/* -------------------------- Public API ---------------------------------- */

/* Call once after HAL/clock init. Communication peripheral may be started
 * before or after this call.  send_cb receives every outgoing Boot_Message_t.
 */
void Boot_Init(uint8_t default_node_id,
               Boot_SendCallback_t send_cb,
               Boot_FlushCallback_t flush_cb,
               void *transport_user);

/* Lower layer -> Bootloader.
 * This only copies one complete logical message into the internal queue.
 * It is safe to call from a short RX callback/ISR in the single-producer case.
 */
uint8_t Boot_Input(const Boot_Message_t *message);

/* Call continuously in main loop.  It processes queued RX messages and all
 * non-blocking READ / missing report / provider tasks.
 */
void Boot_Task(void);

/* APP -> Bootloader request: set backup-register magic, then reset. */
void Boot_RequestBootloader(void);

uint8_t Boot_ShouldJumpApp(void);
void Boot_JumpApp(void);

/* Useful for APP/calibration code sharing the same persistent page layout. */
uint8_t Boot_ConfigLoad(Boot_Config_t *cfg);
uint8_t Boot_ConfigSave(const Boot_Config_t *cfg);

uint8_t  Boot_GetNodeId(void);
uint8_t  Boot_GetStatus(void);
uint8_t  Boot_GetLastError(void);
uint8_t  Boot_GetProgress(void);
uint32_t Boot_GetRxOverflowCount(void);

#ifdef __cplusplus
}
#endif

#endif /* __BOOTLOADER_H */
