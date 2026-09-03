/*
 * STM32G4 FDCAN adapter example for transport-independent bootloader core.
 * Keep this file outside bootloader/ if you want the core directory to stay clean.
 */
#include "bootloader.h"
#include "fdcan.h"
#include "stm32g4xx_hal.h"
#include <string.h>
#include "boot_port_can_stm32g4.h"


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


static uint8_t g_classic_data_buffer[64];

static uint8_t g_classic_data_active = 0U;
static uint8_t g_classic_expected_fragment = 0U;




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





static void BootPort_CAN_ProcessClassicDataFragment(
    uint32_t can_id,
    const uint8_t data[8])
{
    Boot_Message_t message;

    uint8_t fragment;

    if ((can_id < 0x100U) || (can_id > 0x107U))
    {
        return;
    }

    fragment = (uint8_t)(can_id - 0x100U);

    /*
     * Fragment 0 represents the start of one new
     * 64-byte logical DATA packet.
     */
    if (fragment == 0U)
    {
        g_classic_data_active = 1U;
        g_classic_expected_fragment = 1U;

        memcpy(
            &g_classic_data_buffer[0],
            data,
            8U);

        return;
    }

    /*
     * Require strict order:
     *
     * 0x100
     * 0x101
     * ...
     * 0x107
     *
     * If one Classic CAN frame is lost, discard
     * this complete logical packet.
     *
     * The upper Sequence Bitmap will later
     * identify the firmware packet as missing.
     */
    if ((g_classic_data_active == 0U) ||
        (fragment != g_classic_expected_fragment))
    {
        g_classic_data_active = 0U;
        g_classic_expected_fragment = 0U;

        return;
    }

    memcpy(
        &g_classic_data_buffer[(uint32_t)fragment * 8U],
        data,
        8U);

    if (fragment < 7U)
    {
        g_classic_expected_fragment++;

        return;
    }

    /* 8 Classic CAN frames -> one logical DATA message */
    message.type = BOOT_MESSAGE_DATA;
    message.len  = 64U;

    memcpy(
        message.data,
        g_classic_data_buffer,
        64U);

    g_classic_data_active = 0U;
    g_classic_expected_fragment = 0U;

    Boot_Input(&message);
}








/* FDCAN -> Bootloader. The ISR only maps a hardware frame into Boot_Message_t
 * and queues it through Boot_Input(); Flash/protocol work stays in Boot_Task().
 */
 void BootPort_CAN_RxFifo0Callback(
    FDCAN_HandleTypeDef *hfdcan,
    uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t data[64];
    Boot_Message_t message;
    uint8_t len;

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)
    {
        return;
    }

    while (HAL_FDCAN_GetRxFifoFillLevel(
               hfdcan,
               FDCAN_RX_FIFO0) > 0U)
    {
        if (HAL_FDCAN_GetRxMessage(
                hfdcan,
                FDCAN_RX_FIFO0,
                &rx_header,
                data) != HAL_OK)
        {
            return;
        }

        /* =========================
         * CONTROL
         * ========================= */

        if ((rx_header.FDFormat == FDCAN_CLASSIC_CAN) &&
            (rx_header.Identifier == 0x000U) &&
            (rx_header.DataLength == FDCAN_DLC_BYTES_8))
        {
            message.type = BOOT_MESSAGE_CONTROL;
            message.len  = 8U;

            memcpy(message.data, data, 8U);

            Boot_Input(&message);

            continue;
        }

        /* =========================
         * DATA - native CAN FD
         * ========================= */

        if ((rx_header.FDFormat == FDCAN_FD_CAN) &&
            (rx_header.Identifier == 0x100U) &&
            (rx_header.DataLength == FDCAN_DLC_BYTES_64))
        {
            message.type = BOOT_MESSAGE_DATA;
            message.len  = 64U;

            memcpy(message.data, data, 64U);

            Boot_Input(&message);

            continue;
        }

        /* =========================
         * DATA - Classic CAN fallback
         * ========================= */

        if ((rx_header.FDFormat == FDCAN_CLASSIC_CAN) &&
            (rx_header.Identifier >= 0x100U) &&
            (rx_header.Identifier <= 0x107U) &&
            (rx_header.DataLength == FDCAN_DLC_BYTES_8))
        {
            BootPort_CAN_ProcessClassicDataFragment(
                rx_header.Identifier,
                data);

            continue;
        }
    }
}
void BootPort_CAN_Filter_Init() {
  FDCAN_FilterTypeDef filter = {0};

  /* 0x000：控制面 */
  filter.IdType = FDCAN_STANDARD_ID;
  filter.FilterIndex = 0;
  filter.FilterType = FDCAN_FILTER_MASK;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID1 = 0x000;
  filter.FilterID2 = 0x7FF;

  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK) {
    Error_Handler();
  }

  /* 0x100：数据面 */
  filter.FilterIndex = 1;
  filter.FilterID1 = 0x100;
  filter.FilterID2 = 0x7F8;

  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_FDCAN_ConfigGlobalFilter(
        &hfdcan1,
        FDCAN_REJECT,
        FDCAN_REJECT,
        FDCAN_REJECT_REMOTE,
        FDCAN_REJECT_REMOTE) != HAL_OK)
{
    Error_Handler();
}
}

