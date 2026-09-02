#include "boot_can_protocol.h"
#include "boot_can_config.h"
#include "boot_can_port.h"
#include "fdcan.h"



static uint8_t g_node_id;

void BootCAN_Init(uint8_t node_id) { g_node_id = node_id; }

void BootCAN_Process(uint8_t *data, uint8_t len) {

  if (len != 8)
    return;

  Boot_CAN_Frame_t *frame;

  frame = (Boot_CAN_Frame_t *)data;

  /*
   * 判断目标节点
   */

  if (frame->target != g_node_id && frame->target != BOOT_BROADCAST_ID) {
    return;
  }

  switch (frame->cmd) {

  case BOOT_CMD_GET_VERSION:

    BootCAN_SendResponse(frame->cmd, BOOT_STATUS_READY, NULL);

    break;

  case BOOT_CMD_ERASE:

    /*
     * 后续调用flash擦除
     */

    BootCAN_SendResponse(frame->cmd, BOOT_STATUS_ERASE, NULL);

    break;

  case BOOT_CMD_WRITE:

    /*
     * 后续写Flash
     */

    BootCAN_SendResponse(frame->cmd, BOOT_STATUS_WRITE, NULL);

    break;

  case BOOT_CMD_VERIFY:

    BootCAN_SendResponse(frame->cmd, BOOT_STATUS_VERIFY, NULL);

    break;

  case BOOT_CMD_JUMP_APP:

    /*
     * jump_app();
     */

    break;

  case BOOT_CMD_RESET:

    /*
     * NVIC_SystemReset()
     */

    break;

  default:

    BootCAN_SendResponse(frame->cmd, BOOT_STATUS_ERROR, NULL);

    break;
  }
}

void BootCAN_SendResponse(uint8_t cmd, uint8_t status, uint8_t *data) {

  uint8_t tx[8] = {0};

  tx[0] = g_node_id;

  tx[1] = cmd;

  tx[2] = status;

  if (data != NULL) {
    tx[3] = data[0];
    tx[4] = data[1];
    tx[5] = data[2];
    tx[6] = data[3];
  }

  tx[7] = 0; // 暂时CRC不用

  FDCAN_TxHeaderTypeDef txHeader;

  txHeader.Identifier = BOOT_CAN_RESP_BASE_ID + g_node_id;

  txHeader.IdType = FDCAN_STANDARD_ID;

  txHeader.TxFrameType = FDCAN_DATA_FRAME;

  txHeader.DataLength = FDCAN_DLC_BYTES_8;

  txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;

  txHeader.BitRateSwitch = FDCAN_BRS_OFF;

  txHeader.FDFormat = FDCAN_CLASSIC_CAN;

  txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

  txHeader.MessageMarker = 0;

  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, tx);
}
