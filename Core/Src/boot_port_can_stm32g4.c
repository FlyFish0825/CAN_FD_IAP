/*
 * STM32G4 FDCAN adapter example for transport-independent bootloader core.
 * Keep this file outside bootloader/ if you want the core directory to stay clean.
 */
#include "bootloader.h"
#include "fdcan.h"
#include "stm32g4xx_hal.h"
#include <string.h>

#define BOOT_CAN_CONTROL_RX_ID     0x000U
#define BOOT_CAN_DATA_ID           0x100U
#define BOOT_CAN_RESPONSE_BASE_ID  0x500U
#define BOOT_CAN_TX_FIFO_DEPTH     3U

static uint8_t CanDlcToBytes(uint32_t dlc)
{
    switch (dlc)
    {
    case FDCAN_DLC_BYTES_0:  return 0U;
    case FDCAN_DLC_BYTES_1:  return 1U;
    case FDCAN_DLC_BYTES_2:  return 2U;
    case FDCAN_DLC_BYTES_3:  return 3U;
    case FDCAN_DLC_BYTES_4:  return 4U;
    case FDCAN_DLC_BYTES_5:  return 5U;
    case FDCAN_DLC_BYTES_6:  return 6U;
    case FDCAN_DLC_BYTES_7:  return 7U;
    case FDCAN_DLC_BYTES_8:  return 8U;
    case FDCAN_DLC_BYTES_12: return 12U;
    case FDCAN_DLC_BYTES_16: return 16U;
    case FDCAN_DLC_BYTES_20: return 20U;
    case FDCAN_DLC_BYTES_24: return 24U;
    case FDCAN_DLC_BYTES_32: return 32U;
    case FDCAN_DLC_BYTES_48: return 48U;
    case FDCAN_DLC_BYTES_64: return 64U;
    default: return 0U;
    }
}

static uint32_t CanBytesToDlc(uint16_t len)
{
    switch (len)
    {
    case 0U:  return FDCAN_DLC_BYTES_0;
    case 1U:  return FDCAN_DLC_BYTES_1;
    case 2U:  return FDCAN_DLC_BYTES_2;
    case 3U:  return FDCAN_DLC_BYTES_3;
    case 4U:  return FDCAN_DLC_BYTES_4;
    case 5U:  return FDCAN_DLC_BYTES_5;
    case 6U:  return FDCAN_DLC_BYTES_6;
    case 7U:  return FDCAN_DLC_BYTES_7;
    case 8U:  return FDCAN_DLC_BYTES_8;
    case 12U: return FDCAN_DLC_BYTES_12;
    case 16U: return FDCAN_DLC_BYTES_16;
    case 20U: return FDCAN_DLC_BYTES_20;
    case 24U: return FDCAN_DLC_BYTES_24;
    case 32U: return FDCAN_DLC_BYTES_32;
    case 48U: return FDCAN_DLC_BYTES_48;
    case 64U: return FDCAN_DLC_BYTES_64;
    default: return FDCAN_DLC_BYTES_0;
    }
}

/* Bootloader -> FDCAN. SAME Boot_Message_t used for TX. */
uint8_t BootPort_CAN_Send(const Boot_Message_t *message, void *user)
{
    FDCAN_HandleTypeDef *hfdcan = (FDCAN_HandleTypeDef *)user;
    FDCAN_TxHeaderTypeDef tx = {0};
    uint32_t dlc;

    if ((message == NULL) || (hfdcan == NULL)) return 0U;
    dlc = CanBytesToDlc(message->len);
    if ((message->len != 0U) && (dlc == FDCAN_DLC_BYTES_0)) return 0U;

    tx.IdType = FDCAN_STANDARD_ID;
    tx.TxFrameType = FDCAN_DATA_FRAME;
    tx.DataLength = dlc;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker = 0U;

    if (message->type == (uint8_t)BOOT_MESSAGE_CONTROL)
    {
        if (message->len != BOOT_CONTROL_SIZE) return 0U;
        /* Response payload Byte0 is Node ID. */
        tx.Identifier = BOOT_CAN_RESPONSE_BASE_ID + message->data[0];
        tx.BitRateSwitch = FDCAN_BRS_OFF;
        tx.FDFormat = FDCAN_CLASSIC_CAN;
    }
    else if (message->type == (uint8_t)BOOT_MESSAGE_DATA)
    {
        if (message->len != BOOT_DATA_SIZE) return 0U;
        tx.Identifier = BOOT_CAN_DATA_ID;
        tx.BitRateSwitch = FDCAN_BRS_ON;
        tx.FDFormat = FDCAN_FD_CAN;
    }
    else
    {
        return 0U;
    }

    return (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx,
                                           (uint8_t *)message->data) == HAL_OK) ? 1U : 0U;
}

void BootPort_CAN_Flush(void *user, uint32_t timeout_ms)
{
    FDCAN_HandleTypeDef *hfdcan = (FDCAN_HandleTypeDef *)user;
    uint32_t start = HAL_GetTick();
    if (hfdcan == NULL) return;

    while (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) < BOOT_CAN_TX_FIFO_DEPTH)
    {
        if ((HAL_GetTick() - start) >= timeout_ms) break;
    }
}

/* FDCAN -> Bootloader. The ISR only maps a hardware frame into Boot_Message_t
 * and queues it through Boot_Input(); Flash/protocol work stays in Boot_Task().
 */
void BootPort_CAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                                  uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef rx = {0};
    Boot_Message_t message;
    uint8_t len;

    if ((hfdcan == NULL) ||
        ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)) return;

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U)
    {
        memset(&message, 0, sizeof(message));
        memset(&rx, 0, sizeof(rx));

        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0,
                                   &rx, message.data) != HAL_OK) break;

        if ((rx.IdType != FDCAN_STANDARD_ID) ||
            (rx.RxFrameType != FDCAN_DATA_FRAME)) continue;

        len = CanDlcToBytes(rx.DataLength);

        if ((rx.Identifier == BOOT_CAN_CONTROL_RX_ID) &&
            (rx.FDFormat == FDCAN_CLASSIC_CAN) &&
            (len == BOOT_CONTROL_SIZE))
        {
            message.type = (uint8_t)BOOT_MESSAGE_CONTROL;
            message.len = BOOT_CONTROL_SIZE;
            (void)Boot_Input(&message);
        }
        else if ((rx.Identifier == BOOT_CAN_DATA_ID) &&
                 (rx.FDFormat == FDCAN_FD_CAN) &&
                 (len == BOOT_DATA_SIZE))
        {
            message.type = (uint8_t)BOOT_MESSAGE_DATA;
            message.len = BOOT_DATA_SIZE;
            (void)Boot_Input(&message);
        }
    }
}
