#ifndef __BOOT_PORT_CAN_STM32G4_H
#define __BOOT_PORT_CAN_STM32G4_H

#include "bootloader.h"
#include "fdcan.h"

#ifdef __cplusplus
extern "C" {
#endif

uint8_t BootPort_CAN_Send(const Boot_Message_t *message, void *user);

void BootPort_CAN_Flush(void *user, uint32_t timeout_ms);

void BootPort_CAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                                  uint32_t RxFifo0ITs);
                        
void BootPort_CAN_Filter_Init();

#ifdef __cplusplus
}
#endif

#endif