#include "boot_can_protocol.h"
#include "boot_can_config.h"
#include "boot_can_port.h"

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