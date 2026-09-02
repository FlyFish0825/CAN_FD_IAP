#include "boot_can_port.h"
#include "fdcan.h"

static uint32_t BootCAN_LengthToDLC(uint8_t len)
{
    switch (len)
    {
    case 0U: return FDCAN_DLC_BYTES_0;
    case 1U: return FDCAN_DLC_BYTES_1;
    case 2U: return FDCAN_DLC_BYTES_2;
    case 3U: return FDCAN_DLC_BYTES_3;
    case 4U: return FDCAN_DLC_BYTES_4;
    case 5U: return FDCAN_DLC_BYTES_5;
    case 6U: return FDCAN_DLC_BYTES_6;
    case 7U: return FDCAN_DLC_BYTES_7;
    case 8U: return FDCAN_DLC_BYTES_8;
    default: return FDCAN_DLC_BYTES_0;
    }
}

void BootCAN_HW_Send(uint32_t id, const uint8_t *data, uint8_t len)
{
    FDCAN_TxHeaderTypeDef tx_header = {0};

    if ((data == NULL) || (len > 8U))
    {
        return;
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

    (void)HAL_FDCAN_AddMessageToTxFifoQ(
        &hfdcan1,
        &tx_header,
        (uint8_t *)data
    );
}
