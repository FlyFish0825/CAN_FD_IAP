#include "boot_can_port.h"
#include "boot_can_config.h"
#include "fdcan.h"
#include "stm32g4xx_hal.h"

static uint32_t BootCAN_LengthToDLC(uint8_t len)
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
    default:  return FDCAN_DLC_BYTES_0;
    }
}

uint8_t BootCAN_HW_SendClassic(uint32_t id, const uint8_t *data, uint8_t len)
{
    FDCAN_TxHeaderTypeDef tx_header = {0};

    if ((data == NULL) || (len > 8U))
    {
        return 0U;
    }

    tx_header.Identifier = id;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = BootCAN_LengthToDLC(len);
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0U;

    return (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, (uint8_t *)data) == HAL_OK) ? 1U : 0U;
}

uint8_t BootCAN_HW_SendFD(uint32_t id, const uint8_t *data, uint8_t len)
{
    FDCAN_TxHeaderTypeDef tx_header = {0};

    if ((data == NULL) || (BootCAN_LengthToDLC(len) == FDCAN_DLC_BYTES_0))
    {
        return 0U;
    }

    tx_header.Identifier = id;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = BootCAN_LengthToDLC(len);
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_ON;
    tx_header.FDFormat = FDCAN_FD_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0U;

    return (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, (uint8_t *)data) == HAL_OK) ? 1U : 0U;
}

uint32_t BootCAN_HW_TxFreeLevel(void)
{
    return HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1);
}

uint8_t BootCAN_HW_WaitTxIdle(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    /* STM32G4 HAL allocates a fixed three-element Tx FIFO/Queue in Message RAM.
     * When all three entries are free, no frame remains queued for transmission.
     */
    while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) < BOOT_FDCAN_TX_FIFO_DEPTH)
    {
        if ((HAL_GetTick() - start) >= timeout_ms)
        {
            return 0U;
        }
    }

    return 1U;
}
