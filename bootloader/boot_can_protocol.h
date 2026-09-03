#ifndef __BOOT_CAN_PROTOCOL_H
#define __BOOT_CAN_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

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
    BOOT_CMD_MISSING_COUNT    = 0x15, /* node -> master report */
    BOOT_CMD_MISSING_ITEM     = 0x16, /* node -> master report */
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
} Boot_Error_t;

typedef struct
{
    uint8_t target;
    uint8_t cmd;
    uint8_t seq;
    uint8_t param[4];
    uint8_t crc;
} Boot_CAN_Frame_t;

typedef struct
{
    uint8_t node;
    uint8_t cmd;
    uint8_t status;
    uint8_t data[4];
    uint8_t crc;
} Boot_CAN_Response_t;

void BootCAN_Init(uint8_t default_node_id);
uint8_t BootCAN_ShouldJumpApp(void);
void BootCAN_JumpApp(void);

/* Backward-compatible name for the 8-byte Classic CAN command processor. */
void BootCAN_Process(const uint8_t *data, uint8_t len);
void BootCAN_ProcessControl(const uint8_t *data, uint8_t len);
void BootCAN_ProcessFDData(const uint8_t *data, uint8_t len);

/* Call continuously from main loop. Handles READ streaming, missing-list
 * reporting and provider range transmission without blocking the RX callback.
 */
void BootCAN_Task(void);

uint8_t BootCAN_SendResponse(uint8_t cmd, uint8_t status, const uint8_t *data);

uint8_t BootCAN_GetNodeId(void);
uint8_t BootCAN_GetStatus(void);
uint8_t BootCAN_GetLastError(void);
uint8_t BootCAN_GetProgress(void);

#ifdef __cplusplus
}
#endif

#endif /* __BOOT_CAN_PROTOCOL_H */
