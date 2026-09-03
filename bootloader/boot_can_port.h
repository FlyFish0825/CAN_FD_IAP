#ifndef __BOOT_CAN_PORT_H
#define __BOOT_CAN_PORT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint8_t  BootCAN_HW_SendClassic(uint32_t id, const uint8_t *data, uint8_t len);
uint8_t  BootCAN_HW_SendFD(uint32_t id, const uint8_t *data, uint8_t len);
uint32_t BootCAN_HW_TxFreeLevel(void);
uint8_t  BootCAN_HW_WaitTxIdle(uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* __BOOT_CAN_PORT_H */
