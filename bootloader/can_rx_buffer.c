#include "can_rx_buffer.h"

#include <string.h>

static CAN_RX_Handle_t *g_can_rx = NULL;

static void CAN_RX_Write(CAN_RX_Handle_t *rx, const CAN_RX_Message_t *msg);
static void CAN_RX_UpdateCount(CAN_RX_Handle_t *rx);
static uint8_t CAN_RX_DlcToBytes(uint32_t dlc);

void CAN_RX_Init(CAN_RX_Handle_t *rx, FDCAN_HandleTypeDef *hfdcan) {
  if ((rx == NULL) || (hfdcan == NULL)) {
    return;
  }

  memset(rx, 0, sizeof(*rx));
  rx->hfdcan = hfdcan;
  g_can_rx = rx;
}

static void CAN_RX_Write(CAN_RX_Handle_t *rx, const CAN_RX_Message_t *msg) {
  uint16_t next = (uint16_t)(rx->write_index + 1U);

  if (next >= CAN_RX_BUFFER_SIZE) {
    next = 0U;
  }

  if (next == rx->read_index) {
    rx->overflow_count++;
    return;
  }

  rx->buffer[rx->write_index] = *msg;
  rx->write_index = next;
  CAN_RX_UpdateCount(rx);
}

static void CAN_RX_UpdateCount(CAN_RX_Handle_t *rx) {
  if (rx->write_index >= rx->read_index) {
    rx->count = (uint16_t)(rx->write_index - rx->read_index);
  } else {
    rx->count =
        (uint16_t)(CAN_RX_BUFFER_SIZE - rx->read_index + rx->write_index);
  }
}

uint8_t CAN_RX_Read(CAN_RX_Handle_t *rx, CAN_RX_Message_t *msg) {
  if ((rx == NULL) || (msg == NULL)) {
    return 0U;
  }

  if (rx->read_index == rx->write_index) {
    return 0U;
  }

  *msg = rx->buffer[rx->read_index];

  rx->read_index++;
  if (rx->read_index >= CAN_RX_BUFFER_SIZE) {
    rx->read_index = 0U;
  }

  CAN_RX_UpdateCount(rx);
  return 1U;
}

void CAN_RX_Fifo0Callback(CAN_RX_Handle_t *rx, FDCAN_HandleTypeDef *hfdcan,
                          uint32_t RxFifo0ITs) {
  FDCAN_RxHeaderTypeDef header;
  uint8_t data[8] = {0};

  if (rx == NULL) {
    return;
  }

  if (hfdcan != rx->hfdcan) {
    return;
  }

  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U) {
    return;
  }

  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U) {
    CAN_RX_Message_t msg = {0};

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, data) !=
        HAL_OK) {
      break;
    }

    /*
     * CAN ID
     */
    msg.id = header.Identifier;

    /*
     * 标准帧 / 扩展帧
     */
    if (header.IdType == FDCAN_STANDARD_ID) {
      msg.ide = 0U;
    } else {
      msg.ide = 1U;
    }

    /*
     * 数据帧 / 远程帧
     */
    if (header.RxFrameType == FDCAN_REMOTE_FRAME) {
      msg.rtr = 1U;
    } else {
      msg.rtr = 0U;
    }

    /*
     * 当前只支持 Classic CAN 0~8 Byte
     */
    switch (header.DataLength) {
    case FDCAN_DLC_BYTES_0:
      msg.dlc = 0U;
      break;

    case FDCAN_DLC_BYTES_1:
      msg.dlc = 1U;
      break;

    case FDCAN_DLC_BYTES_2:
      msg.dlc = 2U;
      break;

    case FDCAN_DLC_BYTES_3:
      msg.dlc = 3U;
      break;

    case FDCAN_DLC_BYTES_4:
      msg.dlc = 4U;
      break;

    case FDCAN_DLC_BYTES_5:
      msg.dlc = 5U;
      break;

    case FDCAN_DLC_BYTES_6:
      msg.dlc = 6U;
      break;

    case FDCAN_DLC_BYTES_7:
      msg.dlc = 7U;
      break;

    case FDCAN_DLC_BYTES_8:
      msg.dlc = 8U;
      break;

    default:
      /*
       * 当前版本不接受 CAN FD > 8 Byte
       */
      continue;
    }

    /*
     * 只复制实际有效数据
     */
    if (msg.dlc > 0U) {
      memcpy(msg.data, data, msg.dlc);
    }

    /*
     * 写入软件环形缓冲
     */
    CAN_RX_Write(rx, &msg);
  }
}
static uint8_t CAN_RX_DlcToBytes(uint32_t dlc) {
  switch (dlc) {
  case FDCAN_DLC_BYTES_0:
    return 0U;
  case FDCAN_DLC_BYTES_1:
    return 1U;
  case FDCAN_DLC_BYTES_2:
    return 2U;
  case FDCAN_DLC_BYTES_3:
    return 3U;
  case FDCAN_DLC_BYTES_4:
    return 4U;
  case FDCAN_DLC_BYTES_5:
    return 5U;
  case FDCAN_DLC_BYTES_6:
    return 6U;
  case FDCAN_DLC_BYTES_7:
    return 7U;
  case FDCAN_DLC_BYTES_8:
    return 8U;
  default:
    return 0U;
  }
}
