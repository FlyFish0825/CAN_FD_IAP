# CAN Bootloader Protocol

这是当前 Bootloader 项目的 CAN 协议层说明文档。

当前目录包含：

```text
bootloader/
├── boot_can_config.h
├── boot_can_port.h
├── boot_can_protocol.h
├── boot_can_protocol.c
└── README.md
```

当前代码主要完成：

- Bootloader CAN ID 定义
- 节点地址定义
- Bootloader 命令定义
- 8 Byte Classic CAN 数据帧解析
- 节点目标地址判断
- 命令分发
- 节点响应帧发送

> 注意：当前版本主要完成“协议框架”。Flash 擦写、CRC 校验、APP 跳转、设备信息读取等实际功能还没有全部接入。

---

## 1. 文件作用

### `boot_can_config.h`

保存 CAN Bootloader 的固定配置参数。

当前定义：

```c
#define BOOT_CAN_CMD_ID        0x000
#define BOOT_CAN_RESP_BASE_ID  0x500

#define BOOT_MAX_NODE_NUM      8
#define BOOT_BROADCAST_ID      0xFF

#define BOOT_CAN_DLC           8
```

含义：

| 宏 | 当前值 | 作用 |
| --- | ---: | --- |
| `BOOT_CAN_CMD_ID` | `0x000` | 主机发送 Bootloader 命令使用的 CAN ID |
| `BOOT_CAN_RESP_BASE_ID` | `0x500` | 节点回复 ID 的基础值 |
| `BOOT_MAX_NODE_NUM` | `8` | 当前计划支持的最大节点数量 |
| `BOOT_BROADCAST_ID` | `0xFF` | 广播目标地址 |
| `BOOT_CAN_DLC` | `8` | Classic CAN 数据长度 |

节点回复 ID 的计算方式：

```text
Response CAN ID = 0x500 + Node_ID
```

例如：

| Node ID | Response CAN ID |
| ---: | ---: |
| `0x01` | `0x501` |
| `0x02` | `0x502` |
| `0x03` | `0x503` |
| `0x08` | `0x508` |

---

### `boot_can_protocol.h`

定义 Bootloader 协议本身，包括：

- Bootloader 命令
- Bootloader 状态
- 接收帧结构体
- 回复帧结构体
- 协议接口函数

---

### `boot_can_protocol.c`

实现：

- 当前节点 ID 保存
- 8 Byte CAN 数据解析
- Target Node 判断
- Broadcast 判断
- Command 分发
- Bootloader 回复帧发送

核心入口：

```c
void BootCAN_Process(uint8_t *data, uint8_t len);
```

CAN 接收完成后，将 8 Byte 数据交给该函数即可。

---

### `boot_can_port.h`

当前定义了一个底层 CAN 发送抽象接口：

```c
void BootCAN_HW_Send(uint32_t id, uint8_t *data, uint8_t len);
```

这个接口的设计目的是让协议层以后可以和具体 MCU / HAL 驱动解耦。

但是当前 `boot_can_protocol.c` 中的发送函数仍然直接使用：

```c
HAL_FDCAN_AddMessageToTxFifoQ(...)
```

因此当前版本的发送部分实际上仍然和 STM32 HAL FDCAN 耦合。

后续如果要进一步提高可移植性，可以把 `BootCAN_SendResponse()` 内部的 HAL 发送代码移动到 `BootCAN_HW_Send()` 中。

---

## 2. CAN 数据格式

当前使用 Classic CAN 标准数据帧，DLC 固定为 8 Byte。

### 主机 -> 节点

```text
Byte0 : Target
Byte1 : Command
Byte2 : Sequence
Byte3 : Parameter[0]
Byte4 : Parameter[1]
Byte5 : Parameter[2]
Byte6 : Parameter[3]
Byte7 : CRC
```

对应结构体：

```c
typedef struct {

    uint8_t target;
    uint8_t cmd;
    uint8_t seq;
    uint8_t param[4];
    uint8_t crc;

} Boot_CAN_Frame_t;
```

字段说明：

| 字段 | 长度 | 作用 | 当前状态 |
| --- | ---: | --- | --- |
| `target` | 1 Byte | 指定目标节点 | 已使用 |
| `cmd` | 1 Byte | Bootloader 命令 | 已使用 |
| `seq` | 1 Byte | 分包序号 / 命令序号 | 已预留，暂未使用 |
| `param` | 4 Byte | 命令参数 | 已预留，当前多数命令未使用 |
| `crc` | 1 Byte | 协议层 CRC | 已预留，暂未校验 |

---

### 节点 -> 主机

```text
Byte0 : Node ID
Byte1 : Command
Byte2 : Status
Byte3 : Response Data[0]
Byte4 : Response Data[1]
Byte5 : Response Data[2]
Byte6 : Response Data[3]
Byte7 : CRC
```

对应结构体：

```c
typedef struct {

    uint8_t node;
    uint8_t cmd;
    uint8_t status;
    uint8_t data[4];
    uint8_t crc;

} Boot_CAN_Response_t;
```

---

## 3. 节点寻址

### 指定节点

例如：

```text
Target = 0x03
```

表示只有 Node 3 处理该命令。

代码中的判断：

```c
if (frame->target != g_node_id &&
    frame->target != BOOT_BROADCAST_ID) {
    return;
}
```

所以某个节点只有在以下两种情况下会继续处理命令：

1. `Target == 本节点 Node ID`
2. `Target == 0xFF`

---

### 广播

```text
Target = 0xFF
```

表示所有 Bootloader 节点都可以处理该命令。

当前代码收到广播命令后，各节点仍然会按照自己的：

```text
0x500 + Node_ID
```

回复。

例如 Node 1 ~ Node 3 同时收到广播：

```text
0x501
0x502
0x503
```

都会产生回复。

后续在做多节点升级时，需要根据具体命令决定：

- 广播命令是否允许所有节点回复
- 是否采用分时回复
- 是否只由指定节点回复
- 是否使用 ACK Bitmap / 汇总状态

当前版本暂未处理这些高级策略。

---

## 4. Bootloader 状态

当前状态定义：

```c
typedef enum {

    BOOT_STATUS_IDLE = 0,
    BOOT_STATUS_ERASE,
    BOOT_STATUS_WRITE,
    BOOT_STATUS_VERIFY,
    BOOT_STATUS_READY,
    BOOT_STATUS_ERROR

} Boot_Status_t;
```

对应数值：

| 状态 | 数值 | 含义 |
| --- | ---: | --- |
| `BOOT_STATUS_IDLE` | `0x00` | 空闲 |
| `BOOT_STATUS_ERASE` | `0x01` | 擦除状态 |
| `BOOT_STATUS_WRITE` | `0x02` | 写入状态 |
| `BOOT_STATUS_VERIFY` | `0x03` | 校验状态 |
| `BOOT_STATUS_READY` | `0x04` | 已准备好 / 命令处理正常 |
| `BOOT_STATUS_ERROR` | `0x05` | 命令错误或暂不支持 |

当前这些状态主要用于响应帧。

---

# 5. Bootloader 命令说明

当前命令定义在：

```c
typedef enum {
    BOOT_CMD_GET_VERSION   = 0x01,
    BOOT_CMD_GET_DEVICE_ID = 0x02,
    BOOT_CMD_GET_INFO      = 0x03,
    BOOT_CMD_ENTER_BOOT    = 0x04,

    BOOT_CMD_ERASE         = 0x10,
    BOOT_CMD_WRITE         = 0x11,
    BOOT_CMD_READ          = 0x12,
    BOOT_CMD_VERIFY        = 0x13,

    BOOT_CMD_JUMP_APP      = 0x20,
    BOOT_CMD_RESET         = 0x21,

    BOOT_CMD_GET_STATUS    = 0x30,
} Boot_Command_t;
```

命令总览：

| CMD | 名称 | 作用 | 当前代码状态 |
| ---: | --- | --- | --- |
| `0x01` | `GET_VERSION` | 查询 Bootloader 版本 | 已进入命令分支，但暂未返回真实版本号 |
| `0x02` | `GET_DEVICE_ID` | 查询设备 / MCU 标识 | 仅定义，尚未实现 |
| `0x03` | `GET_INFO` | 查询节点综合信息 | 仅定义，尚未实现 |
| `0x04` | `ENTER_BOOT` | 请求进入 / 保持 Bootloader 模式 | 仅定义，尚未实现 |
| `0x10` | `ERASE` | 擦除待升级 Flash 区域 | 已进入命令分支，但尚未真正擦除 Flash |
| `0x11` | `WRITE` | 向 Flash 写入固件数据 | 已进入命令分支，但尚未真正写 Flash |
| `0x12` | `READ` | 读取指定存储区域 | 仅定义，尚未实现 |
| `0x13` | `VERIFY` | 校验已写入固件 | 已进入命令分支，但尚未真正执行 CRC / Hash 校验 |
| `0x20` | `JUMP_APP` | 跳转到 Application | 已进入命令分支，但实际跳转代码尚未实现 |
| `0x21` | `RESET` | 软件复位 MCU | 已进入命令分支，但实际复位代码尚未实现 |
| `0x30` | `GET_STATUS` | 查询 Bootloader 当前状态 | 仅定义，尚未实现 |

## 5.1 `GET_VERSION` - `0x01`

作用：查询 Bootloader 自身的软件版本。

典型用途：

- 上位机确认 Bootloader 是否在线
- 判断协议版本
- 判断上位机与 Bootloader 是否兼容
- 作为最基础的通信测试命令

发送示例：

```text
CAN ID : 0x000
Data   : 01 01 00 00 00 00 00 00
```

当前回复：

```text
CAN ID : 0x501
Data   : 01 01 04 00 00 00 00 00
```

当前实现：

```c
case BOOT_CMD_GET_VERSION:

    BootCAN_SendResponse(
        frame->cmd,
        BOOT_STATUS_READY,
        NULL
    );

    break;
```

所以目前只是证明：

```text
接收 -> 解析 -> 分发 -> 回复
```

还没有真正返回版本号。

后续可以把 `Byte3 ~ Byte6` 定义为：

```text
Major
Minor
Patch
Build
```

---

## 5.2 `GET_DEVICE_ID` - `0x02`

作用：读取设备标识。

后续可以返回：

- MCU Device ID
- 芯片型号编码
- 板卡类型
- 硬件版本
- 自定义 Device Type

典型用途：

```text
升级前确认当前固件是否适用于该硬件
```

当前状态：

`boot_can_protocol.c` 中没有对应 `case`，因此当前发送该命令会进入 `default` 并返回：

```text
BOOT_STATUS_ERROR
```

---

## 5.3 `GET_INFO` - `0x03`

作用：获取节点的综合 Bootloader 信息。

后续可以用于返回：

- Bootloader Version
- APP Version
- Flash Size
- Application Address
- Hardware Version
- Node ID
- 当前升级状态

与 `GET_VERSION` 的区别：

```text
GET_VERSION -> 只查询 Bootloader 版本
GET_INFO    -> 查询节点综合信息
```

当前状态：

仅定义命令，没有实际处理，因此当前会返回：

```text
BOOT_STATUS_ERROR
```

---

## 5.4 `ENTER_BOOT` - `0x04`

作用：请求节点进入或保持 Bootloader 模式。

后续典型启动流程：

```text
MCU Reset
   |
   v
Bootloader
   |
   +---- 未收到升级请求 ----> Jump APP
   |
   +---- 收到 ENTER_BOOT ---> 保持 Bootloader
```

典型用途：

上位机准备升级时先发送：

```text
ENTER_BOOT
```

告诉节点不要跳转 APP。

当前状态：

仅定义命令，没有实际处理，因此当前会返回：

```text
BOOT_STATUS_ERROR
```

---

## 5.5 `ERASE` - `0x10`

作用：擦除准备写入固件的 Flash 区域。

典型升级顺序：

```text
ENTER_BOOT
    |
    v
ERASE
    |
    v
WRITE
```

`param[4]` 后续可以定义为：

- Flash 起始地址
- Page Index
- Sector Index
- 擦除长度
- 擦除模式

当前实现：

```c
case BOOT_CMD_ERASE:

    BootCAN_SendResponse(
        frame->cmd,
        BOOT_STATUS_ERASE,
        NULL
    );

    break;
```

所以当前实际行为只是：

```text
收到 ERASE
    |
    v
回复 BOOT_STATUS_ERASE
```

不会真正擦除 Flash。

---

## 5.6 `WRITE` - `0x11`

作用：向 Application Flash 区写入固件数据。

完整固件通常需要分包：

```text
Firmware
   |
   +--> Block 0
   +--> Block 1
   +--> Block 2
   +--> ...
```

当前协议中的 `Sequence` 字段就是为分包序号等用途预留的。

当前实现：

```c
case BOOT_CMD_WRITE:

    BootCAN_SendResponse(
        frame->cmd,
        BOOT_STATUS_WRITE,
        NULL
    );

    break;
```

所以当前只表示：

```text
WRITE 命令已经被识别
```

不会真正写 Flash。

---

## 5.7 `READ` - `0x12`

作用：读取指定数据或存储区域。

后续可以用于：

- Flash 调试读取
- Metadata 读取
- 固件信息读取
- 指定地址检查

对于只需要：

```text
下载固件 -> 校验 -> 跳转
```

的最小 Bootloader，`READ` 不是必须命令，可以保留用于调试或后续扩展。

当前状态：

仅定义命令，没有实际处理，因此当前会返回：

```text
BOOT_STATUS_ERROR
```

---

## 5.8 `VERIFY` - `0x13`

作用：升级完成后验证 Flash 中的固件是否正确。

典型流程：

```text
WRITE COMPLETE
     |
     v
VERIFY
     |
     +---- PASS ---> Ready / Jump APP
     |
     +---- FAIL ---> Error
```

后续可以实现：

- CRC32
- 固件长度检查
- 固件 Header 检查
- Hash 校验

当前实现：

```c
case BOOT_CMD_VERIFY:

    BootCAN_SendResponse(
        frame->cmd,
        BOOT_STATUS_VERIFY,
        NULL
    );

    break;
```

当前没有执行真实 CRC / Hash 校验。

因此：

```text
BOOT_STATUS_VERIFY
```

只代表命令进入了 VERIFY 分支，不代表固件已经通过校验。

---

## 5.9 `JUMP_APP` - `0x20`

作用：Bootloader 完成工作后跳转到 Application。

典型流程：

```text
VERIFY PASS
    |
    v
JUMP_APP
    |
    v
Application
```

真正实现时通常需要：

- 检查 APP 地址
- 检查 MSP
- 检查 Reset Handler
- 关闭 Bootloader 使用的中断 / 外设
- 设置向量表
- 设置 MSP
- 跳转 Application Reset Handler

当前实现：

```c
case BOOT_CMD_JUMP_APP:

    /*
     * jump_app();
     */

    break;
```

所以当前：

```text
不会跳转
不会回复
```

这里只是预留了命令入口。

---

## 5.10 `RESET` - `0x21`

作用：让 MCU 软件复位。

后续可以用于：

- 升级完成后重新启动
- 从 Bootloader 重启
- 重新执行启动判断流程

STM32 上后续可以使用：

```c
NVIC_SystemReset();
```

当前实现：

```c
case BOOT_CMD_RESET:

    /*
     * NVIC_SystemReset()
     */

    break;
```

所以当前：

```text
不会复位
不会回复
```

只是预留命令。

---

## 5.11 `GET_STATUS` - `0x30`

作用：查询 Bootloader 当前工作状态。

后续主机可以查询：

```text
IDLE
ERASE
WRITE
VERIFY
READY
ERROR
```

例如：

```text
Host -> ERASE
Host -> GET_STATUS
Node -> ERASE
Host -> GET_STATUS
Node -> READY
```

这样主机不必写死擦除等待时间。

当前状态：

仅定义命令，没有对应 `case`，因此当前会进入 `default` 并返回：

```text
BOOT_STATUS_ERROR
```

---

## 6. 当前命令实际可测试情况

按照当前 `boot_can_protocol.c`，存在实际 `case` 分支的命令：

```text
GET_VERSION
ERASE
WRITE
VERIFY
JUMP_APP
RESET
```

当前实际行为：

| 命令 | 能否识别 | 是否执行真实功能 | 是否回复 |
| --- | --- | --- | --- |
| `GET_VERSION` | 是 | 否，只返回 READY | 是 |
| `ERASE` | 是 | 否，没有擦 Flash | 是 |
| `WRITE` | 是 | 否，没有写 Flash | 是 |
| `VERIFY` | 是 | 否，没有 CRC 校验 | 是 |
| `JUMP_APP` | 是 | 否 | 否 |
| `RESET` | 是 | 否 | 否 |

以下命令目前会进入 `default`：

```text
GET_DEVICE_ID
GET_INFO
ENTER_BOOT
READ
GET_STATUS
```

并返回：

```text
BOOT_STATUS_ERROR
```

---

## 7. 使用方法

### 7.1 初始化节点

Node 1：

```c
BootCAN_Init(1);
```

Node 8：

```c
BootCAN_Init(8);
```

内部保存到：

```c
static uint8_t g_node_id;
```

---

### 7.2 CAN 接收后调用协议解析

收到 Bootloader CAN 数据后：

```c
BootCAN_Process(rx_data, 8);
```

内部流程：

```text
收到 8 Byte CAN Data
        |
        v
检查 len == 8
        |
        v
映射 Boot_CAN_Frame_t
        |
        v
检查 Target
        |
        v
检查 Broadcast
        |
        v
switch(Command)
        |
        v
执行对应处理
        |
        v
发送 Response
```

---

### 7.3 CANPro 测试

当前可以直接测试 Node 1 的 `GET_VERSION`。

发送：

```text
CAN ID : 0x000
DLC    : 8
Data   : 01 01 00 00 00 00 00 00
```

预期收到：

```text
CAN ID : 0x501
DLC    : 8
Data   : 01 01 04 00 00 00 00 00
```

其中：

```text
01 = Node 1
01 = GET_VERSION
04 = BOOT_STATUS_READY
```

---

## 8. 如何扩展一个新命令

例如增加：

```text
GET_SERIAL_NUMBER
```

### 第一步：定义命令值

在 `boot_can_protocol.h`：

```c
typedef enum {

    ...

    BOOT_CMD_GET_SERIAL_NUMBER = 0x40,

} Boot_Command_t;
```

命令值不要和现有命令重复。

### 第二步：增加命令处理

在 `boot_can_protocol.c`：

```c
case BOOT_CMD_GET_SERIAL_NUMBER:
{
    uint8_t response[4];

    response[0] = 0x12;
    response[1] = 0x34;
    response[2] = 0x56;
    response[3] = 0x78;

    BootCAN_SendResponse(
        frame->cmd,
        BOOT_STATUS_READY,
        response
    );

    break;
}
```

对应回复格式：

```text
Byte0 Node ID
Byte1 GET_SERIAL_NUMBER
Byte2 READY
Byte3 0x12
Byte4 0x34
Byte5 0x56
Byte6 0x78
Byte7 CRC
```

---

## 9. 如何扩展现有命令

建议协议层只负责：

```text
接收命令
    |
    v
解析参数
    |
    v
调用功能模块
    |
    v
根据结果回复
```

例如以后实现真正的 `ERASE`：

```c
case BOOT_CMD_ERASE:
{
    BootFlash_Result_t result;

    result = BootFlash_Erase(...);

    if (result == BOOT_FLASH_OK) {

        BootCAN_SendResponse(
            frame->cmd,
            BOOT_STATUS_READY,
            NULL
        );

    } else {

        BootCAN_SendResponse(
            frame->cmd,
            BOOT_STATUS_ERROR,
            NULL
        );
    }

    break;
}
```

这样不要把所有 Flash 细节直接写进协议解析函数。

---

## 10. 建议的模块扩展方式

后续可以逐步增加：

```text
bootloader/
├── boot_can_config.h
├── boot_can_port.h
├── boot_can_protocol.h
├── boot_can_protocol.c
├── boot_flash.h
├── boot_flash.c
├── boot_crc.h
├── boot_crc.c
├── boot_jump.h
├── boot_jump.c
└── README.md
```

职责建议：

```text
boot_can_protocol
        |
        +---- boot_flash
        |
        +---- boot_crc
        |
        +---- boot_jump
```

协议层只负责命令解析、调用和响应。

---

## 11. 底层 CAN 发送接口的进一步解耦

当前 `boot_can_port.h` 已经定义：

```c
void BootCAN_HW_Send(
    uint32_t id,
    uint8_t *data,
    uint8_t len
);
```

但当前 `BootCAN_SendResponse()` 仍直接创建：

```c
FDCAN_TxHeaderTypeDef
```

并调用：

```c
HAL_FDCAN_AddMessageToTxFifoQ(...)
```

如果以后需要支持：

- 不同 STM32
- bxCAN
- FDCAN
- HAL
- LL

推荐把 HAL 相关代码移动到独立的：

```text
boot_can_port.c
```

协议层只保留：

```c
BootCAN_HW_Send(
    BOOT_CAN_RESP_BASE_ID + g_node_id,
    tx,
    BOOT_CAN_DLC
);
```

这样 `boot_can_protocol.c` 就不需要：

```c
#include "fdcan.h"
```

协议层会更加独立。

---

## 12. 当前协议扩展时需要保持的原则

### 命令号唯一

不要让两个命令使用同一个 CMD。

### 协议层只做协议

协议层负责：

```text
解析
分发
响应
```

不要承担大量 Flash / CRC / APP 逻辑。

### 未实现功能不要返回成功

例如当前 `ERASE` 只是占位。

真正实现以后，应该根据 Flash 擦除结果决定返回：

```text
READY
```

或者：

```text
ERROR
```

### 保留 Sequence

当前 `seq` 没有使用，但后续固件分包时非常重要。

### 保留 CRC

当前：

```c
tx[7] = 0;
```

因此 CRC 还没有启用。

真正启用 CRC 后，接收和回复都应使用相同 CRC 算法。

---

## 13. 当前代码的最小调用关系

```text
          FDCAN Receive
                |
                v
       BootCAN_Process()
                |
                v
      Target / CMD Decode
                |
                v
        switch(frame->cmd)
                |
        +-------+-------+
        |       |       |
        v       v       v
      INFO    ERASE   WRITE ...
                |
                v
     BootCAN_SendResponse()
                |
                v
 HAL_FDCAN_AddMessageToTxFifoQ()
```

当前阶段的重点是先把：

```text
CAN 通信
+
节点寻址
+
命令解析
+
响应机制
```

稳定下来，再逐步把具体 Bootloader 功能接入对应命令。
