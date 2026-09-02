#ifndef __CAN_RX_BUFFER_H
#define __CAN_RX_BUFFER_H

#include "fdcan.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CAN_RX_MAX_DATA_LEN 8U
#define CAN_RX_BUFFER_SIZE  32U

typedef struct
{
    uint32_t id;

    union
    {
        uint8_t flags;
        struct
        {
            uint8_t ide : 1;
            uint8_t rtr : 1;
            uint8_t reserve : 6;
        };
    };

    uint8_t dlc;
    uint8_t data[CAN_RX_MAX_DATA_LEN];
} CAN_RX_Message_t;

typedef struct
{
    FDCAN_HandleTypeDef *hfdcan;
    CAN_RX_Message_t buffer[CAN_RX_BUFFER_SIZE];
    volatile uint16_t write_index;
    volatile uint16_t read_index;
    volatile uint32_t overflow_count;
    volatile uint16_t count;
} CAN_RX_Handle_t;

void CAN_RX_Init(CAN_RX_Handle_t *rx, FDCAN_HandleTypeDef *hfdcan);
uint8_t CAN_RX_Read(CAN_RX_Handle_t *rx, CAN_RX_Message_t *msg);

void HAL_FDCAN_RxFifo0Callback(
    FDCAN_HandleTypeDef *hfdcan,
    uint32_t RxFifo0ITs
);

#ifdef __cplusplus
}
#endif

#endif /* __CAN_RX_BUFFER_H */
