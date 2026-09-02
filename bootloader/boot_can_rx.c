#include "boot_can_rx.h"
#include "boot_can_config.h"
#include "boot_can_protocol.h"

void BootCAN_RX_Process(CAN_RX_Handle_t *rx)
{
    CAN_RX_Message_t msg;

    while (CAN_RX_Read(rx, &msg))
    {
        if (msg.ide != 0U)
        {
            continue;
        }

        if (msg.rtr != 0U)
        {
            continue;
        }

        if (msg.id != BOOT_CAN_CMD_ID)
        {
            continue;
        }

        if (msg.dlc != BOOT_CAN_DLC)
        {
            continue;
        }

        BootCAN_Process(msg.data, msg.dlc);
    }
}
