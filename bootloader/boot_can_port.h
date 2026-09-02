#ifndef __BOOT_CAN_PORT_H
#define __BOOT_CAN_PORT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void BootCAN_HW_Send(uint32_t id, const uint8_t *data, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* __BOOT_CAN_PORT_H */
