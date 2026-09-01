#ifndef __BOOT_CAN_CONFIG_H
#define __BOOT_CAN_CONFIG_H

/*==========================
 * CAN ID
 *==========================*/

#define BOOT_CAN_CMD_ID 0x000

#define BOOT_CAN_RESP_BASE_ID 0x500

/* 最大节点 */

#define BOOT_MAX_NODE_NUM 8

/* 广播地址 */

#define BOOT_BROADCAST_ID 0xFF

/* 数据长度 */

#define BOOT_CAN_DLC 8

#endif