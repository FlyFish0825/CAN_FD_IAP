#include "boot_can_rx.h"
#include "boot_can_config.h"
#include "boot_can_protocol.h"

void BootCAN_RX_Process(CAN_RX_Handle_t *rx)
{
    CAN_RX_Message_t msg;

    while (CAN_RX_Read(rx, &msg) != 0U)
    {
        if ((msg.ide != 0U) || (msg.rtr != 0U))
        {
            continue;
        }

        if ((msg.id == BOOT_CAN_CMD_ID) &&
            (msg.fd == 0U) &&
            (msg.dlc == BOOT_CAN_CTRL_DLC))
        {
            BootCAN_ProcessControl(msg.data, msg.dlc);
            continue;
        }

        if ((msg.id == BOOT_CAN_FD_DATA_ID) &&
            (msg.fd != 0U) &&
            (msg.dlc == BOOT_CAN_FD_DLC))
        {
            BootCAN_ProcessFDData(msg.data, msg.dlc);
            continue;
        }
    }
}
