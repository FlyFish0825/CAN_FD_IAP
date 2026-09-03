#include "can_rx_buffer.h"
#include <string.h>

static uint8_t CAN_RX_DlcToBytes(uint32_t dlc)
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
    default:                 return 0U;
    }
}

static void CAN_RX_Write(CAN_RX_Handle_t *rx, const CAN_RX_Message_t *msg)
{
    uint16_t next;

    if ((rx == NULL) || (msg == NULL))
    {
        return;
    }

    next = (uint16_t)((rx->write_index + 1U) % CAN_RX_BUFFER_SIZE);

    if (next == rx->read_index)
    {
        rx->overflow_count++;
        return;
    }

    rx->buffer[rx->write_index] = *msg;
    rx->write_index = next;
    rx->count++;
}

void CAN_RX_Init(CAN_RX_Handle_t *rx, FDCAN_HandleTypeDef *hfdcan)
{
    if (rx == NULL)
    {
        return;
    }

    memset(rx, 0, sizeof(*rx));
    rx->hfdcan = hfdcan;
}

uint8_t CAN_RX_Read(CAN_RX_Handle_t *rx, CAN_RX_Message_t *msg)
{
    if ((rx == NULL) || (msg == NULL))
    {
        return 0U;
    }

    if (rx->read_index == rx->write_index)
    {
        return 0U;
    }

    *msg = rx->buffer[rx->read_index];
    rx->read_index = (uint16_t)((rx->read_index + 1U) % CAN_RX_BUFFER_SIZE);

    if (rx->count > 0U)
    {
        rx->count--;
    }

    return 1U;
}

void CAN_RX_Fifo0Callback(CAN_RX_Handle_t *rx,
                          FDCAN_HandleTypeDef *hfdcan,
                          uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef header;
    CAN_RX_Message_t msg;

    if ((rx == NULL) || (hfdcan == NULL) || (hfdcan != rx->hfdcan))
    {
        return;
    }

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)
    {
        return;
    }

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U)
    {
        memset(&msg, 0, sizeof(msg));
        memset(&header, 0, sizeof(header));

        if (HAL_FDCAN_GetRxMessage(hfdcan,
                                   FDCAN_RX_FIFO0,
                                   &header,
                                   msg.data) != HAL_OK)
        {
            break;
        }

        msg.id = header.Identifier;
        msg.ide = (header.IdType == FDCAN_EXTENDED_ID) ? 1U : 0U;
        msg.rtr = (header.RxFrameType == FDCAN_REMOTE_FRAME) ? 1U : 0U;
        msg.fd = (header.FDFormat == FDCAN_FD_CAN) ? 1U : 0U;
        msg.brs = (header.BitRateSwitch == FDCAN_BRS_ON) ? 1U : 0U;
        msg.dlc = CAN_RX_DlcToBytes(header.DataLength);

        CAN_RX_Write(rx, &msg);
    }
}
