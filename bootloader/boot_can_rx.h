#ifndef __BOOT_CAN_RX_H
#define __BOOT_CAN_RX_H

#include "can_rx_buffer.h"

#ifdef __cplusplus
extern "C" {
#endif

void BootCAN_RX_Process(CAN_RX_Handle_t *rx);

#ifdef __cplusplus
}
#endif

#endif /* __BOOT_CAN_RX_H */
