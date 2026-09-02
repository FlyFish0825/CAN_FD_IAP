# CAN Bootloader Protocol

轻量级 CAN Bootloader 协议框架。

当前版本实现：

-   CAN 数据格式定义
-   节点地址管理
-   命令定义
-   CAN 数据解析
-   命令分发
-   回复帧生成

当前代码不包含：

-   Flash 擦写
-   固件下载
-   CRC 校验
-   APP 跳转

------------------------------------------------------------------------

# 1. 文件结构

    bootloader
    │
    ├── boot_can_config.h
    ├── boot_can_port.h
    ├── boot_can_protocol.h
    ├── boot_can_protocol.c
    └── README.md

------------------------------------------------------------------------

# 2. 软件结构

    CAN Hardware
          |
          |
    boot_can_port.h
          |
          |
    boot_can_protocol.c/h
          |
          |
    Application

## boot_can_protocol

负责：

-   CAN 数据解析
-   命令处理
-   状态回复

## boot_can_port

负责：

-   CAN 底层发送接口

## boot_can_config

负责：

-   协议参数配置

------------------------------------------------------------------------

# 3. CAN 协议格式

Classic CAN:

    8 Byte

## 主机发送

    Byte0 : Target Node ID
    Byte1 : Command
    Byte2 : Sequence
    Byte3~Byte6 : Parameter
    Byte7 : CRC

## 节点回复

    Byte0 : Node ID
    Byte1 : Command
    Byte2 : Status
    Byte3~Byte6 : Response Data
    Byte7 : CRC

------------------------------------------------------------------------

# 4. CAN ID

## 主机发送 ID

    0x000

所有节点监听。

## 节点回复 ID

    0x500 + Node_ID

例如：

  节点    ID
  ------- -------
  Node1   0x501
  Node2   0x502
  Node8   0x508

------------------------------------------------------------------------

# 5. 节点地址

单节点：

    0x01 ~ 0x08

广播：

    0xFF

------------------------------------------------------------------------

# 6. 命令定义

  Command         Value   功能
  --------------- ------- --------------
  GET_VERSION     0x01    获取版本
  GET_DEVICE_ID   0x02    获取设备ID
  GET_INFO        0x03    获取设备信息
  ENTER_BOOT      0x04    进入Boot
  ERASE           0x10    擦除
  WRITE           0x11    写数据
  READ            0x12    读取
  VERIFY          0x13    校验
  JUMP_APP        0x20    跳转APP
  RESET           0x21    复位
  GET_STATUS      0x30    获取状态

------------------------------------------------------------------------

# 7. 使用方法

初始化：

``` c
BootCAN_Init(1);
```

表示当前节点：

    Node ID = 1

CAN 接收后调用：

``` c
BootCAN_Process(rx_data, 8);
```

调用流程：

    CAN RX Interrupt

            |

    BootCAN_Process()

            |

    Command Decode

            |

    Execute Command

            |

    Send Response

------------------------------------------------------------------------

# 8. 测试示例

查询版本：

CAN ID:

    0x000

Data:

    01 01 00 00 00 00 00 00

含义：

    Target = Node1

    CMD = GET_VERSION

回复：

ID:

    0x501

Data:

    01 01 04 00 00 00 00 00

------------------------------------------------------------------------

# 9. 扩展命令

增加命令：

在 boot_can_protocol.h：

``` c
BOOT_CMD_NEW = 0x40
```

在 boot_can_protocol.c：

``` c
case BOOT_CMD_NEW:

    BootCAN_SendResponse(
        frame->cmd,
        BOOT_STATUS_READY,
        data
    );

break;
```

------------------------------------------------------------------------

# 10. 底层 CAN 移植

协议层不直接调用 HAL。

通过：

    boot_can_port.h

提供：

``` c
void BootCAN_HW_Send(
    uint32_t id,
    uint8_t *data,
    uint8_t len
);
```

移植时只需要实现：

    BootCAN_HW_Send()

即可。

------------------------------------------------------------------------

# 11. 扩展结构

推荐：

    bootloader

    ├── boot_can_protocol.c
    ├── boot_can_port.c
    ├── boot_flash.c
    ├── boot_crc.c
    └── boot_jump.c

协议层：

    收到命令

    ↓

    调用对应模块

    ↓

    返回结果

------------------------------------------------------------------------

# 12. 设计原则

## 低耦合

协议与 CAN 驱动分离。

## 可移植

支持：

-   STM32 HAL
-   STM32 LL
-   其他 MCU

## 可扩展

预留：

-   Node ID
-   Sequence
-   Parameter
-   Command ID
