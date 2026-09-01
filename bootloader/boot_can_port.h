#ifndef __BOOT_CAN_PORT_H
#define __BOOT_CAN_PORT_H

#include <stdint.h>

void BootCAN_HW_Send(uint32_t id, uint8_t *data, uint8_t len);

#endif