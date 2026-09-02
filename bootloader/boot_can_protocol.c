#include "boot_can_protocol.h"
#include "boot_can_config.h"
#include "boot_can_port.h"

#include <stddef.h>
#include <string.h>

static uint8_t g_node_id = 0U;

void BootCAN_Init(uint8_t node_id)
{
    g_node_id = node_id;
}

void BootCAN_Process(const uint8_t *data, uint8_t len)
{
    Boot_CAN_Frame_t frame;

    if (data == NULL)
    {
        return;
    }

    if (len != BOOT_CAN_DLC)
    {
        return;
    }

    memcpy(&frame, data, sizeof(frame));

    if ((frame.target != g_node_id) &&
        (frame.target != BOOT_BROADCAST_ID))
    {
        return;
    }

    switch (frame.cmd)
    {
    case BOOT_CMD_GET_VERSION:
        BootCAN_SendResponse(frame.cmd, BOOT_STATUS_READY, NULL);
        break;

    case BOOT_CMD_ERASE:
        /* TODO: BootFlash_Erase(...) */
        BootCAN_SendResponse(frame.cmd, BOOT_STATUS_ERASE, NULL);
        break;

    case BOOT_CMD_WRITE:
        /* TODO: BootFlash_Write(...) */
        BootCAN_SendResponse(frame.cmd, BOOT_STATUS_WRITE, NULL);
        break;

    case BOOT_CMD_VERIFY:
        /* TODO: BootCRC_Verify(...) */
        BootCAN_SendResponse(frame.cmd, BOOT_STATUS_VERIFY, NULL);
        break;

    case BOOT_CMD_JUMP_APP:
        /* TODO: Boot_JumpApplication(); */
        break;

    case BOOT_CMD_RESET:
        /* TODO: NVIC_SystemReset(); */
        break;

    case BOOT_CMD_GET_DEVICE_ID:
    case BOOT_CMD_GET_INFO:
    case BOOT_CMD_ENTER_BOOT:
    case BOOT_CMD_READ:
    case BOOT_CMD_GET_STATUS:
    default:
        BootCAN_SendResponse(frame.cmd, BOOT_STATUS_ERROR, NULL);
        break;
    }
}

void BootCAN_SendResponse(uint8_t cmd, uint8_t status, const uint8_t *data)
{
    Boot_CAN_Response_t response = {0};

    response.node = g_node_id;
    response.cmd = cmd;
    response.status = status;

    if (data != NULL)
    {
        memcpy(response.data, data, sizeof(response.data));
    }

    response.crc = 0U;

    BootCAN_HW_Send(
        BOOT_CAN_RESP_BASE_ID + g_node_id,
        (const uint8_t *)&response,
        BOOT_CAN_DLC
    );
}
