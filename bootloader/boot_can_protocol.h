#ifndef __BOOT_CAN_PROTOCOL_H
#define __BOOT_CAN_PROTOCOL_H

#include <stdint.h>

/*
 * Bootloader命令
 */

typedef enum {

  BOOT_CMD_GET_VERSION = 0x01,

  BOOT_CMD_GET_DEVICE_ID = 0x02,

  BOOT_CMD_GET_INFO = 0x03,

  BOOT_CMD_ENTER_BOOT = 0x04,

  BOOT_CMD_ERASE = 0x10,

  BOOT_CMD_WRITE = 0x11,

  BOOT_CMD_READ = 0x12,

  BOOT_CMD_VERIFY = 0x13,

  BOOT_CMD_JUMP_APP = 0x20,

  BOOT_CMD_RESET = 0x21,

  BOOT_CMD_GET_STATUS = 0x30,

} Boot_Command_t;

/*
 * Boot状态
 */

typedef enum {

  BOOT_STATUS_IDLE = 0,

  BOOT_STATUS_ERASE,

  BOOT_STATUS_WRITE,

  BOOT_STATUS_VERIFY,

  BOOT_STATUS_READY,

  BOOT_STATUS_ERROR

} Boot_Status_t;

/*
 * CAN数据帧
 */

typedef struct {

  uint8_t target;

  uint8_t cmd;

  uint8_t seq;

  uint8_t param[4];

  uint8_t crc;

} Boot_CAN_Frame_t;

/*
 * 回复帧
 */

typedef struct {

  uint8_t node;

  uint8_t cmd;

  uint8_t status;

  uint8_t data[4];

  uint8_t crc;

} Boot_CAN_Response_t;

/*
 * 协议接口
 */

void BootCAN_Init(uint8_t node_id);

void BootCAN_Process(uint8_t *data, uint8_t len);

void BootCAN_SendResponse(uint8_t cmd, uint8_t status, uint8_t *data);

#endif