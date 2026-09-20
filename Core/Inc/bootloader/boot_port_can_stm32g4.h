#ifndef __BOOT_PORT_CAN_STM32G4_H
#define __BOOT_PORT_CAN_STM32G4_H /* 防止 FDCAN 适配层接口重复包含。 */

#include "bootloader.h"
#include "fdcan.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 通过 STM32G4 FDCAN 外设发送一条逻辑 Boot_Message_t 消息。
 * @param message 待发送的逻辑消息，包含消息类型、长度和有效载荷。
 * @param user    底层传输上下文，约定为 FDCAN_HandleTypeDef 指针。
 * @return 1 表示已接受到发送队列，0 表示参数、长度或硬件队列不可用。
 */
uint8_t BootPort_CAN_Send(const Boot_Message_t *message, void *user);

/**
 * @brief 等待 FDCAN 发送队列和经典 CAN 分片发送缓冲区排空。
 * @param user       底层传输上下文，约定为 FDCAN_HandleTypeDef 指针。
 * @param timeout_ms 最长等待时间，单位为毫秒。
 */
void BootPort_CAN_Flush(void *user, uint32_t timeout_ms);

/* Drain a staged 64-byte DATA message as 8 Classic CAN fragments when the
 * FDCAN peripheral is configured in Classic-only mode. Call from main loop. */
void BootPort_CAN_Task(void *user);

/**
 * @brief 处理 FDCAN FIFO0 新报文中断，并转换为 Bootloader 输入消息。
 * @param hfdcan     产生中断的 FDCAN 句柄。
 * @param RxFifo0ITs 已使能且实际触发的 FIFO0 中断标志集合。
 */
void BootPort_CAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                                  uint32_t RxFifo0ITs);
                        
/**
 * @brief 配置控制、数据和节点间通信使用的 FDCAN 标准帧过滤器。
 */
void BootPort_CAN_Filter_Init(void);

#ifdef __cplusplus
}
#endif

#endif
