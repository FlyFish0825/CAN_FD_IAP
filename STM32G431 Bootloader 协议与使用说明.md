# STM32G431 Bootloader 协议与使用说明

> 适用版本：当前 `bootloader_refactor_v2`  
> 核心文件：`bootloader.c / bootloader.h`  
> 设计原则：**Bootloader 核心与 CAN/UART/SPI/I2C 等具体通信接口解耦，上下层只交换 `Boot_Message_t`。**

---

# 1. 文档目的

本文档统一说明当前 Bootloader 的：

- 软件架构；
- Flash 分区；
- 上下层数据交换方式；
- 当前 CAN/CAN FD 映射；
- 控制帧和数据帧格式；
- CRC 算法；
- 状态码和错误码；
- 所有命令的参数、用途和可能返回值；
- APP 升级完整流程；
- Guard / 缺包 / Provider 的工作方式；
- 如何接入 `main.c`；
- 如何使用 CAN Pro 进行测试。

Bootloader 核心本身不依赖 CAN。当前项目只是使用 STM32G4 FDCAN 作为一种底层 Transport。

---

# 2. 软件架构

整体分层：

```text
                    Bootloader Core
                 bootloader.c/.h
                        │
                        │ Boot_Message_t
                        ▼
                  Transport Adapter
                        │
          ┌─────────────┼─────────────┐
          ▼             ▼             ▼
       CAN/CAN FD      UART         SPI/I2C/USB
```

核心原则：

```text
Bootloader 只负责：
- 协议
- Flash
- CRC
- APP 校验
- Guard
- Bitmap
- Missing Report
- Provider
- Jump APP

Transport 只负责：
- 数据怎么收
- 数据怎么发
- 如何把物理链路转换成 Boot_Message_t
```

因此 `bootloader.c` 不需要知道：

```text
CAN ID
DLC
BRS
UART DMA
SPI CS
I2C Address
```

---

# 3. 唯一上下层交换结构

接收和发送都只使用：

```c
typedef struct
{
    uint8_t type;
    uint16_t len;
    uint8_t data[64];
} Boot_Message_t;
```

`type`：

```c
BOOT_MESSAGE_CONTROL = 0
BOOT_MESSAGE_DATA    = 1
```

## 3.1 CONTROL

用于：

- 查询；
- 控制；
- 状态；
- ACK；
- 缺包报告；
- Provider 授权。

当前固定：

```text
len = 8 Byte
```

## 3.2 DATA

用于：

- APP 固件数据；
- Config 数据；
- Provider 补包数据。

当前固定：

```text
len = 64 Byte
```

---

# 4. Bootloader 对外 API

初始化：

```c
Boot_Init(default_node_id,
          send_callback,
          flush_callback,
          transport_user);
```

Transport 收到一帧完整逻辑消息后：

```c
Boot_Input(&message);
```

主循环持续调用：

```c
Boot_Task();
```

APP 请求进入 Bootloader：

```c
Boot_RequestBootloader();
NVIC_SystemReset();
```

启动时判断是否可以直接进入 APP：

```c
if (Boot_ShouldJumpApp())
{
    Boot_JumpApp();
}
```

---

# 5. 当前 CAN / CAN FD 映射

虽然核心与 CAN 解耦，但当前 FDCAN Adapter 约定：

| 方向 | 类型 | CAN ID | 帧类型 |
|---|---|---:|---|
| 主控 → 节点 | CONTROL | `0x000` | Classic CAN，8 Byte |
| 节点 → 主控 | CONTROL | `0x500 + Node_ID` | Classic CAN，8 Byte |
| 主控/Provider → 节点 | DATA | `0x100` | CAN FD + BRS，64 Byte |

例如：

```text
Node1 Response ID = 0x501
Node8 Response ID = 0x508
```

建议 FDCAN Standard Filter：

```text
Filter 0 : 0x000 exact match -> FIFO0
Filter 1 : 0x100 exact match -> FIFO0
```

其他帧通过 Global Filter 拒绝。

---

# 6. Flash 分区

STM32G431 当前按照 128 KiB Flash：

| 区域 | 地址 | 大小 |
|---|---|---:|
| Bootloader | `0x08000000 ~ 0x08004FFF` | 20 KiB |
| APP | `0x08005000 ~ 0x0801F7FF` | 106 KiB |
| Config + Metadata | `0x0801F800 ~ 0x0801FFFF` | 2 KiB |

STM32G431 Flash Page = 2 KiB：

```text
Page 0 ~ 9    Bootloader
Page 10 ~ 62  APP
Page 63       Config + Metadata
```

权限：

```text
READ:
    允许读取整个内部 Flash
    0x08000000 ~ 0x0801FFFF

WRITE:
    APP区       允许
    Config区    允许
    Bootloader  禁止
```

---

# 7. 持久配置区

最后一页保存：

```c
typedef struct
{
    uint32_t magic;
    uint16_t config_version;
    uint16_t length;

    uint8_t  node_id;
    uint8_t  hardware_version;
    uint16_t reserved0;

    uint16_t current_offset_a;
    uint16_t current_offset_b;
    uint16_t current_offset_c;
    uint16_t vbus_offset;

    float current_gain_a;
    float current_gain_b;
    float current_gain_c;
    float vbus_gain;

    uint32_t app_size;
    uint32_t app_crc32;
    uint8_t  app_valid;
    uint8_t  reserved1[3];

    uint32_t reserved[8];

    uint32_t crc32;
} Boot_Config_t;
```

主要保存：

```text
PCB Node ID
硬件版本
三相 ADC 零点
三相增益校准
VBUS 零点
VBUS 增益
APP Size
APP CRC32
APP Valid
Config CRC32
```

---

# 8. Classic CAN CONTROL 请求帧格式

固定 8 Byte：

```text
Byte0      Target
Byte1      Command
Byte2      Seq / Command-specific Parameter
Byte3      Param0
Byte4      Param1
Byte5      Param2
Byte6      Param3
Byte7      CRC8(Byte0 ~ Byte6)
```

Target：

```text
0x01 ~ 0x08   单节点
0xFF          广播
```

所有 16/32-bit 参数默认使用：

```text
Little Endian
```

例如：

```text
0x08005000
```

发送：

```text
00 50 00 08
```

---

# 9. CONTROL 响应帧格式

节点固定返回 8 Byte：

```text
Byte0      Node ID
Byte1      Command
Byte2      Status
Byte3      Data0
Byte4      Data1
Byte5      Data2
Byte6      Data3
Byte7      CRC8(Byte0 ~ Byte6)
```

注意：

> `Byte2` 是响应的当前状态；错误详情通常放在 `Byte3`。

通用错误响应：

```text
Byte0      Node ID
Byte1      原命令
Byte2      BOOT_STATUS_ERROR = 0x05
Byte3      Error Code
Byte4~6    0
Byte7      CRC
```

---

# 10. CRC

## 10.1 CONTROL CRC8

CONTROL 请求和响应使用：

```text
CRC-8
Polynomial = 0x07
Init       = 0x00
RefIn      = false
RefOut     = false
XorOut     = 0
```

计算范围：

```text
Byte0 ~ Byte6
```

结果：

```text
Byte7
```

当前由 STM32 CRC 外设计算。

如果收到错误 CRC：

```text
节点不回复
g_last_error = BOOT_ERR_BAD_CRC
```

原因是 CRC 错误时不再信任 Target 和 Command。

---

## 10.2 APP CRC32

APP 和 Config 使用：

```text
Polynomial = 0x04C11DB7
Init       = 0xFFFFFFFF
RefIn      = false
RefOut     = false
XorOut     = 0
```

用于：

- Config 完整性；
- APP VERIFY；
- 上电 APP 校验。

---

# 11. DATA 数据帧格式

固定 64 Byte：

```text
Byte0       Target
Byte1       Data Command
Byte2~3     Sequence，uint16_t，小端
Byte4~7     Reserved
Byte8~63    Payload = 56 Byte
```

当前 Data Command：

```text
0x01 = WRITE_DATA
```

Sequence：

```text
0, 1, 2, 3, ...
```

对于 APP：

```c
FlashAddress = 0x08005000 + Sequence * 56;
```

每包 Payload = 56 Byte，正好：

```text
56 = 7 × 8
```

适合 STM32G4 64-bit Double Word Flash 编程。

最后一包不足 56 Byte：

```text
有效数据按 firmware_size 判断
未使用部分填 0xFF
```

---

# 12. 状态码 Status

| 值 | 名称 | 含义 |
|---:|---|---|
| `0x00` | `IDLE` | 空闲 |
| `0x01` | `ERASE` | 擦除中 |
| `0x02` | `WRITE` | 写入/发送数据中 |
| `0x03` | `VERIFY` | 数据完整，等待或正在校验 |
| `0x04` | `READY` | 当前操作完成 |
| `0x05` | `ERROR` | 错误 |
| `0x06` | `REPAIR` | 存在缺包，等待恢复 |
| `0x07` | `GUARD` | 当前节点是 Guard |

---

# 13. 错误码 Error

| 值 | 名称 | 含义 |
|---:|---|---|
| `0x00` | `NONE` | 无错误 |
| `0x01` | `BAD_CRC` | CONTROL CRC 错误 |
| `0x02` | `BAD_LENGTH` | 长度错误 |
| `0x03` | `BAD_ADDRESS` | 地址/参数非法 |
| `0x04` | `BAD_STATE` | 当前状态不允许该命令 |
| `0x05` | `FLASH_ERASE` | Flash 擦除失败 |
| `0x06` | `FLASH_WRITE` | Flash 写入或读回失败 |
| `0x07` | `CONFIG` | Config 保存/读取失败 |
| `0x08` | `APP_INVALID` | APP 无效 |
| `0x09` | `SIZE` | 固件/配置长度非法 |
| `0x0A` | `GUARD_PROTECTED` | Guard 拒绝擦写 |
| `0x0B` | `SEQUENCE` | Sequence 非法 |
| `0x0C` | `CRC_MISMATCH` | APP CRC32 不一致 |
| `0x0D` | `PROVIDER_SOURCE` | Provider 没有所需数据 |
| `0x0E` | `BUSY` | 当前异步任务忙 |
| `0x0F` | `PROTECTED_REGION` | 尝试写受保护 Bootloader |
| `0x10` | `ABORTED` | 升级被中止 |
| `0x11` | `RX_OVERFLOW` | Boot_Message RX Queue 溢出 |

---

# 14. 命令总表

| CMD | 名称 | 方向 | 作用 |
|---:|---|---|---|
| `0x01` | GET_VERSION | Master → Node | 查询 Bootloader 版本 |
| `0x02` | GET_DEVICE_ID | Master → Node | 查询 STM32 `DBGMCU->IDCODE` |
| `0x03` | GET_INFO | Master → Node | 查询节点/APP/Config 概览 |
| `0x04` | ENTER_BOOT | Master → Node | 保持在 Bootloader |
| `0x05` | SET_GUARD | Master → Nodes | 指定 Guard |
| `0x06` | RELEASE_GUARD | Master → Nodes | 解除 Guard |
| `0x10` | ERASE | Master → Node | 擦除整个 APP |
| `0x11` | WRITE | Master → Node | 建立 APP/Config 写入 Session |
| `0x12` | READ | Master → Node | 连续读取 Flash |
| `0x13` | VERIFY | Master → Node | 校验 APP CRC32 |
| `0x14` | WRITE_END | Master → Node | 本轮发送结束并检查缺包 |
| `0x15` | MISSING_COUNT | Node → Master | 缺包总数报告 |
| `0x16` | MISSING_ITEM | Node → Master | 缺失 Sequence 报告 |
| `0x17` | PROVIDER_GRANT | Master → Provider | 授权节点提供单包/Range |
| `0x18` | ABORT | Master → Node | 中止当前升级 |
| `0x20` | JUMP_APP | Master → Node | 校验后跳 APP |
| `0x21` | RESET | Master → Node | ACK 后软件复位 |
| `0x30` | GET_STATUS | Master → Node | 查询状态/错误/进度 |

---

# 15. 命令详细说明

---

## 15.1 GET_VERSION `0x01`

### 请求

```text
Byte0   Target
Byte1   0x01
Byte2   0
Byte3~6 0
Byte7   CRC
```

Node1 示例：

```text
01 01 00 00 00 00 00 F6
```

### 成功响应

```text
Byte2   READY = 0x04
Byte3   Major
Byte4   Minor
Byte5   Patch
Byte6   Build
```

当前版本：

```text
1.0.0.0
```

Node1：

```text
01 01 04 01 00 00 00 6F
```

### 可能结果

```text
READY
```

CRC 错误时：

```text
无响应
```

---

## 15.2 GET_DEVICE_ID `0x02`

读取：

```c
DBGMCU->IDCODE
```

### 请求

```text
Target 02 00 00 00 00 00 CRC
```

Node1 已实测：

```text
01 02 00 00 00 00 00 8D
```

### 成功响应

```text
Byte3~6 = DBGMCU->IDCODE，32-bit Little Endian
```

例如当前某块板：

```text
01 02 04 68 64 03 20 CRC
```

### 可能结果

```text
READY
```

---

## 15.3 GET_INFO `0x03`

### 请求

```text
Target 03 00 00 00 00 00 CRC
```

### 响应 Data

```text
Byte3 = Node ID
Byte4 = Hardware Version
Byte5 = APP Valid
Byte6 = Config Valid
```

例如：

```text
01 03 04 01 00 00 00 3D
```

表示：

```text
Node ID       = 1
Hardware Ver  = 0
APP Valid     = 0
Config Valid  = 0
```

### APP Valid 的实际含义

不是简单检查 Flash 非空，而是：

```text
Config合法
+
app_valid == 1
+
app_size合法
+
MSP合法
+
Reset_Handler合法
+
CRC32(APP) == app_crc32
```

### 可能结果

```text
READY
```

---

## 15.4 ENTER_BOOT `0x04`

### Bootloader 内收到

作用：

```text
g_boot_requested = 1
清除错误
保持在 Bootloader
```

### 请求

```text
Target 04 00 00 00 00 00 CRC
```

### 成功响应

```text
Status = READY
```

### APP 中进入 Bootloader

APP 侧推荐不是直接复用 Bootloader 状态变量，而是：

```c
Boot_RequestBootloader();
NVIC_SystemReset();
```

`Boot_RequestBootloader()` 会写：

```text
TAMP->BKP0R = BOOT_REQUEST_MAGIC
```

复位后 Bootloader 消费该 Magic 并留在 Bootloader。

---

## 15.5 SET_GUARD `0x05`

主控指定本轮 Guard Node。

### 请求格式

```text
Byte0   Target，通常建议 0xFF 广播
Byte1   0x05
Byte2   Guard Node ID
Byte3~6 0
Byte7   CRC
```

例如：

```text
FF 05 01 00 00 00 00 AB
```

表示 Node1 是 Guard。

### Guard 节点响应

```text
Status = GUARD = 0x07
Byte3  = Guard Node ID
```

实测：

```text
01 05 07 01 00 00 00 6D
```

### 非 Guard 节点响应

```text
Status = READY
Byte3  = Guard Node ID
```

### 错误

Guard ID 非 `1~8`：

```text
ERROR
Byte3 = BAD_ADDRESS = 0x03
```

---

## 15.6 RELEASE_GUARD `0x06`

解除当前 Guard。

### 请求

```text
Byte0   Target，通常广播
Byte1   0x06
Byte2   当前 Guard ID
Byte3~6 0
Byte7   CRC
```

例如：

```text
FF 06 01 00 00 00 00 D0
```

### 成功响应

```text
Status = READY
Byte3  = 被解除的 Guard ID
```

实测：

```text
01 06 04 01 00 00 00 B0
```

### 可能错误

没有 Guard，或者 Guard ID 不匹配：

```text
ERROR
Byte3 = BAD_STATE = 0x04
```

---

## 15.7 ERASE `0x10`

擦除整个 APP：

```text
0x08005000 ~ 0x0801F7FF
```

### 请求

```text
Target 10 00 00 00 00 00 CRC
```

### 实际执行顺序

```text
收到 ERASE
↓
如果是 Guard -> 拒绝
↓
先保存 app_valid = 0
↓
擦除整个 APP
↓
成功后 READY
```

### 成功响应

```text
Status = READY
```

表示：

> APP 已经真正擦完，不是“开始擦除”。

### Guard 状态

返回：

```text
Status = GUARD
```

并：

```text
Last Error = GUARD_PROTECTED
```

实测：

```text
01 10 07 00 00 00 00 68
```

### 可能错误

```text
GUARD              Guard保护
ERROR + CONFIG     app_valid=0 写入失败
ERROR + FLASH_ERASE Flash擦除失败
```

---

## 15.8 WRITE `0x11`

`WRITE` 本身不携带固件。

它只建立一个后续 DATA Session。

### 请求格式

```text
Byte0      Target
Byte1      0x11
Byte2      Region
Byte3~6    Size，uint32_t，小端
Byte7      CRC
```

Region：

```text
0x00 APP
0x01 CONFIG
0x02 BOOTLOADER，目前禁止
```

---

### APP WRITE

要求：

```text
之前已经成功 ERASE
```

否则：

```text
ERROR + BAD_STATE
```

Size：

```text
1 ~ 106 KiB
```

总包数：

```c
packet_count = (size + 55) / 56;
```

### 成功响应

```text
Status = WRITE

Byte3 = Region
Byte4~5 = Total Packet Count，uint16 LE
Byte6 = 0
```

---

### CONFIG WRITE

Size：

```text
1 ~ 2048 Byte
```

Config 写入先写 RAM Stage Buffer。

真正擦 Config Page 并提交发生在：

```text
WRITE_END
```

---

### 可能返回

```text
GUARD
ERROR + GUARD_PROTECTED

ERROR + PROTECTED_REGION
    Region = BOOTLOADER

ERROR + BAD_STATE
    APP没有先ERASE

ERROR + SIZE
    Size=0 或超范围

ERROR + CONFIG
    Config Stage初始化失败

ERROR + BAD_ADDRESS
    未知Region

WRITE
    Session成功建立
```

---

## 15.9 READ `0x12`

连续读取内部 Flash。

### 请求格式

```text
Byte0      Target
Byte1      0x12
Byte2      Length，1~255 Byte
Byte3~6    Start Address
Byte7      CRC
```

允许地址：

```text
0x08000000 ~ 0x0801FFFF
```

不允许：

```text
SRAM
外设
System ROM
任意非法地址
```

### 工作方式

READ 命令本身不会立即返回一个独立 ACK。

之后 `Boot_Task()` 连续发送多个：

```text
CMD = READ
Status = READY
```

每帧 Data 最多：

```text
4 Byte
```

直到 Length 全部读完。

例如：

```text
Length = 255
```

返回：

```text
64个响应帧
```

前 63 帧 4 Byte，最后一帧只有 3 Byte 有效。

### 已实测

读取：

```text
0x08000000
```

返回数据和 ST-LINK Utility 完全一致。

### 可能错误

```text
ERROR + BAD_LENGTH
    Length = 0

ERROR + BUSY
    上一个READ仍未结束

ERROR + BAD_ADDRESS
    地址越界
```

---

## 15.10 VERIFY `0x13`

对完整 APP 做 CRC32。

### 前提

必须满足：

```text
WRITE Session存在
Region = APP
Missing Count = 0
```

### 请求

```text
Byte0      Target
Byte1      0x13
Byte2      0
Byte3~6    Expected APP CRC32
Byte7      CRC8
```

### 成功流程

```text
计算 CRC32(0x08005000, firmware_size)
↓
等于 Expected CRC
↓
保存 app_size
↓
保存 app_crc32
↓
app_valid = 1
↓
READY
```

### 成功响应

```text
Status = READY
Byte3~6 = Actual CRC32
```

### CRC 不一致

```text
Status = ERROR
Byte3~6 = Bootloader实际计算出的 CRC32
Last Error = CRC_MISMATCH
```

注意：

> CRC_MISMATCH 这一种错误响应的 Data[4] 返回 Actual CRC，而不是错误码。

### 其他可能错误

```text
GUARD

ERROR + BAD_STATE
    没有APP WRITE Session
    或仍有缺包

ERROR + CONFIG
    APP Metadata保存失败
```

---

## 15.11 WRITE_END `0x14`

表示：

> 当前这一轮 DATA 已经发完，请节点检查 Bitmap。

### 请求

```text
Target 14 00 00 00 00 00 CRC
```

### 有缺包

节点计算：

```text
missing = total_packets - received_packets
```

响应：

```text
Status = REPAIR
Byte3~4 = Missing Count，uint16 LE
```

随后 `Boot_Task()` 自动发送：

```text
MISSING_COUNT
MISSING_ITEM
MISSING_ITEM
...
```

---

### APP 无缺包

响应：

```text
Status = VERIFY
Missing Count = 0
```

表示：

> 所有包已收到，可以发送 VERIFY。

同时仍会异步发送：

```text
MISSING_COUNT = 0
```

---

### CONFIG 无缺包

执行：

```text
Config Stage Commit
↓
擦最后一页
↓
整页写回
↓
重新加载Config
```

成功：

```text
Status = READY
```

并异步报告：

```text
MISSING_COUNT = 0
```

---

### 可能错误

```text
GUARD

ERROR + BAD_STATE
    没有 WRITE Session

ERROR + CONFIG
    Config Commit失败
```

---

## 15.12 MISSING_COUNT `0x15`

这是：

```text
Node -> Master
```

自动报告命令。

主控不应主动向节点发送这个命令。

### 响应格式

```text
Byte0      Node ID
Byte1      0x15
Byte2      READY 或 REPAIR
Byte3~4    Missing Count
Byte5~6    Total Packet Count
Byte7      CRC
```

如果：

```text
Missing Count = 0
```

Status：

```text
READY
```

否则：

```text
REPAIR
```

### 如果主控错误地发送 0x15 给节点

节点返回：

```text
ERROR + BAD_STATE
```

---

## 15.13 MISSING_ITEM `0x16`

也是：

```text
Node -> Master
```

自动报告。

每帧只报告一个缺失 Sequence。

### 格式

```text
Byte0      Node ID
Byte1      0x16
Byte2      REPAIR
Byte3~4    Missing Sequence
Byte5~6    Missing Item Index
Byte7      CRC
```

例如：

```text
Node3 缺包：
105
382
901
```

先：

```text
MISSING_COUNT = 3
```

再依次发送：

```text
MISSING_ITEM seq=105 index=0
MISSING_ITEM seq=382 index=1
MISSING_ITEM seq=901 index=2
```

### 如果主控主动发送 0x16

节点返回：

```text
ERROR + BAD_STATE
```

---

## 15.14 PROVIDER_GRANT `0x17`

主控指定某一个节点成为唯一 Provider。

### 非常重要

Provider Grant：

```text
必须单播
```

禁止：

```text
Target = 0xFF
```

因为广播授权可能导致多个节点一起发送。

当前代码对广播 `PROVIDER_GRANT`：

```text
直接忽略，不回复
```

---

### 请求格式

```text
Byte0      Provider Node ID
Byte1      0x17
Byte2      Data Target
Byte3~4    Start Sequence
Byte5~6    Count
Byte7      CRC
```

Data Target 可以是：

```text
1~8   指定节点
0xFF  广播补包
```

Count：

```text
1       单包
>1      连续Range
```

例如：

```text
Provider = Node1
Target   = Node3
StartSeq = 105
Count    = 1
```

---

### 授权成功响应

```text
Status = WRITE
Byte3  = Data Target
Byte4~5 = Start Sequence
Byte6  = 0
```

然后 Provider 在 `Boot_Task()` 中开始发送：

```text
BOOT_MESSAGE_DATA
```

---

### Provider DATA 发送完成

节点会再异步发送一帧：

```text
CMD    = PROVIDER_GRANT
Status = READY
Byte3  = Data Target
```

表示 Range 已发送完成。

---

### Provider 数据来源

某 Sequence 可提供，只要满足其中一种：

#### 当前升级 Session 中该包可信

```text
bitmap[seq] == 1
```

也就是：

```text
已经成功写Flash
+
Read-back正确
```

#### 或者节点已有完整持久 APP

```text
app_valid == 1
```

且 Sequence 在旧/当前 APP 范围内。

因此 Provider 是：

> **包级可信**

不要求节点自己的整个新 APP 已经 VERIFY。

---

### 可能错误

```text
ERROR + BAD_ADDRESS
    Data Target非法

ERROR + BAD_LENGTH
    Count = 0

ERROR + SEQUENCE
    start + count 超出 uint16 范围

ERROR + PROVIDER_SOURCE
    Provider没有Start Sequence数据

异步 ERROR + PROVIDER_SOURCE
    Range发送过程中某个后续包不存在
```

---

## 15.15 ABORT `0x18`

中止当前升级操作。

### 请求

```text
Target 18 00 00 00 00 00 CRC
```

### 节点动作

```text
取消 READ
取消 Missing Report
取消 Provider
取消 Config Stage
结束 WRITE Session
```

状态：

```text
ERROR
Last Error = ABORTED
```

### 响应

```text
Status = ERROR
```

注意：

当前 ABORT 响应 Data 全 0，具体原因通过：

```text
GET_STATUS
```

读取：

```text
Last Error = ABORTED
```

---

## 15.16 JUMP_APP `0x20`

### 前提

必须：

```text
g_app_valid == 1
```

并重新验证：

```text
Config CRC
APP Size
Vector Table
APP CRC32
```

### 成功

先发送：

```text
READY
```

然后：

```text
Flush TX
↓
关闭中断/SysTick
↓
VTOR = 0x08005000
↓
设置 MSP
↓
跳 APP Reset_Handler
```

### 失败

```text
ERROR
Byte3 = APP_INVALID
```

不会跳转。

---

## 15.17 RESET `0x21`

### 请求

```text
Target 21 00 00 00 00 00 CRC
```

### 行为

```text
先回复 READY
↓
Flush TX
↓
HAL_Delay(1)
↓
NVIC_SystemReset()
```

### 成功返回

```text
READY
```

然后 MCU 立即复位。

---

## 15.18 GET_STATUS `0x30`

查询当前状态。

### 请求

```text
Target 30 00 00 00 00 00 CRC
```

### 响应

```text
Byte2 = 当前 Status

Byte3 = 当前 Status
Byte4 = Last Error
Byte5 = Progress，0~100
Byte6 = Reserved
```

例如 Guard 保护后实测：

```text
01 30 07 07 0A 00 00 B6
```

含义：

```text
Status     = GUARD
Last Error = GUARD_PROTECTED
Progress   = 0
```

---

# 16. DATA 包的处理规则

收到 DATA 时节点依次检查：

```text
Target
↓
Data Command
↓
是否Guard
↓
是否存在WRITE Session
↓
Sequence范围
↓
Bitmap是否已收到
↓
目标Region
↓
写Flash/RAM Stage
↓
Read-back
↓
Bitmap置1
```

---

## 16.1 Wrong Target

不是：

```text
本节点ID
或
0xFF
```

则：

```text
直接忽略
```

---

## 16.2 Guard

Guard 在第一阶段收到 DATA：

```text
直接忽略
```

不会修改 APP/Config。

---

## 16.3 Duplicate Packet

如果：

```text
bitmap[seq] == 1
```

说明此包已经成功写入并读回。

重复包：

```text
直接忽略
```

不会再次编程 Flash。

---

## 16.4 单包写失败

如果某个包 Flash 编程/读回失败：

```text
Last Error = FLASH_WRITE
bitmap[seq] 保持 0
```

但：

> 不终止整个传输。

节点继续接收后续包。

最后通过：

```text
WRITE_END
```

统一发现缺包并修复。

---

# 17. 单节点 APP 升级完整流程

推荐主控流程：

```text
1. ENTER_BOOT / 节点已经在Bootloader

2. ERASE
   等 READY

3. WRITE
   Region = APP
   Size = firmware_size
   等 WRITE

4. 连续发送 DATA
   Seq = 0 ~ N-1

5. WRITE_END

6. 如果 REPAIR
   收 MISSING_COUNT
   收全部 MISSING_ITEM
   补包
   再 WRITE_END
   最多由中控决定执行3轮

7. Missing = 0
   节点状态进入 VERIFY

8. VERIFY(expected_crc32)

9. READY

10. JUMP_APP
```

---

# 18. 多节点广播升级

同构节点可共用一份 BIN：

```text
Master
  │
  ├─ Broadcast DATA Packet0
  ├─ Broadcast DATA Packet1
  ├─ Broadcast DATA Packet2
  └─ ...
```

各节点独立维护 Bitmap。

某节点缺一个包不会要求全网停止。

---

# 19. 全局缺包汇总

首轮完成后：

```text
Master -> WRITE_END
```

各节点：

```text
MISSING_COUNT
MISSING_ITEM...
```

主控先收集所有节点全部缺包。

例如：

```text
Node2 = {10,25}
Node3 = {25,100}
Node5 = {25,200}
```

主控得到：

```text
Union = {10,25,100,200}
```

对于：

```text
Sequence 25
```

可以只补一次广播：

```text
Target = 0xFF
Seq = 25
```

Node2/3/5 一起修复。

---

# 20. Provider 机制

PC 只发送一次 BIN。

中控可以只做：

```text
流式转发
```

不永久保存完整 BIN。

如果首轮后需要补包：

```text
主控统一选择一个拥有该包的节点
↓
PROVIDER_GRANT
↓
该Provider从自己的Flash读取对应56 Byte
↓
发送DATA
```

严格原则：

```text
同时只允许一个Provider发送
```

---

# 21. 7+1 Guard

升级 8 个电机时：

```text
主控动态指定一个 Guard
```

例如：

```text
Node8 = Guard
```

第一阶段：

```text
Node1~7 更新新版本
Node8 只听，不擦、不写
```

这样 Node8 始终保存上一版本。

---

# 22. Guard 正常升级流程

```text
SET_GUARD(Node8)

Node1~7:
    ERASE
    WRITE
    DATA
    Repair
    VERIFY

Node1~7全部成功
↓
RELEASE_GUARD(Node8)
↓
选择一个拥有完整新APP的节点
↓
PROVIDER_GRANT Range
↓
完整发送新APP给Node8
↓
Node8 VERIFY
↓
8节点全部READY
↓
广播JUMP_APP
```

---

# 23. 三轮恢复与回滚

恢复轮次数由：

```text
中控
```

决定，不是 Bootloader 节点自己决定。

建议：

```text
Initial Broadcast
↓
Repair Round 1
↓
Repair Round 2
↓
Repair Round 3
```

如果仍失败：

```text
停止新版本更新
↓
Guard提供旧版本
↓
自动Rollback
```

Rollback 也最多三轮。

如果回滚仍失败：

```text
ABORT
↓
停在Bootloader
↓
不JUMP_APP
↓
等待人工/总线恢复后重新更新
```

---

# 24. Config 更新流程

如果要通过协议修改 PCB 参数：

```text
WRITE
Region = CONFIG
Size = 实际要发送的Config数据长度
↓
DATA
↓
WRITE_END
```

注意：

Config Data 的 offset 也是：

```text
Sequence × 56
```

写入的是最后一页对应偏移。

`WRITE_END` 时：

```text
自动修正：
Magic
Config Version
Length
Node ID合法性
Config CRC32
```

然后整页擦除并重新写入。

---

# 25. Bootloader 上电行为

`Boot_Init()`：

```text
读取Config
↓
Config有效？
  ├─ 是 -> 使用Config Node ID
  └─ 否 -> 使用烧录时传入的default_node_id
↓
验证APP
↓
读取并清除Backup Boot Magic
```

然后：

```c
if (Boot_ShouldJumpApp())
{
    Boot_JumpApp();
}
```

只有：

```text
没有 Boot Request
+
APP有效
```

才跳 APP。

否则留在 Bootloader。

---

# 26. main.c 使用方法

包含：

```c
#include "bootloader.h"
#include "fdcan.h"
```

声明 CAN Adapter：

```c
uint8_t BootPort_CAN_Send(const Boot_Message_t *message, void *user);

void BootPort_CAN_Flush(void *user, uint32_t timeout_ms);

void BootPort_CAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                                  uint32_t RxFifo0ITs);
```

初始化：

```c
Boot_Init(
    1U,
    BootPort_CAN_Send,
    BootPort_CAN_Flush,
    &hfdcan1
);
```

判断 APP：

```c
if (Boot_ShouldJumpApp() != 0U)
{
    Boot_JumpApp();
}
```

启动 FDCAN：

```c
HAL_FDCAN_Start(&hfdcan1);

HAL_FDCAN_ActivateNotification(
    &hfdcan1,
    FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
    0U
);
```

主循环：

```c
while (1)
{
    Boot_Task();
}
```

---

# 27. FDCAN RX 回调

```c
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0ITs)
{
    BootPort_CAN_RxFifo0Callback(hfdcan, RxFifo0ITs);
}
```

CAN Adapter 内部只做：

```text
FDCAN Frame
↓
Boot_Message_t
↓
Boot_Input()
```

不会在 ISR 里：

```text
擦Flash
写Flash
算APP CRC
协议重任务
```

这些全部由：

```c
Boot_Task();
```

执行。

---

# 28. CAN Pro 基础测试

当前已实测通过：

---

## GET_VERSION

发送：

```text
ID = 0x000
01 01 00 00 00 00 00 F6
```

收到：

```text
ID = 0x501
01 01 04 01 00 00 00 6F
```

---

## GET_DEVICE_ID

发送：

```text
01 02 00 00 00 00 00 8D
```

成功返回 `DBGMCU->IDCODE`。

---

## GET_INFO

发送：

```text
01 03 00 00 00 00 00 A4
```

成功返回 Node / HW / APP Valid / Config Valid。

---

## GET_STATUS

发送：

```text
01 30 00 00 00 00 00 7A
```

成功。

---

## READ

已测试：

```text
Address = 0x08000000
Length  = 255 Byte
```

CAN 返回内容和 ST-LINK Utility 逐字节一致。

---

## SET_GUARD

发送：

```text
FF 05 01 00 00 00 00 AB
```

收到：

```text
01 05 07 01 00 00 00 6D
```

---

## Guard ERASE Protection

发送：

```text
01 10 00 00 00 00 00 41
```

收到：

```text
01 10 07 00 00 00 00 68
```

APP 没有被擦除。

---

## Guard Status

收到：

```text
01 30 07 07 0A 00 00 B6
```

表示：

```text
GUARD
GUARD_PROTECTED
```

---

## RELEASE_GUARD

发送：

```text
FF 06 01 00 00 00 00 D0
```

收到：

```text
01 06 04 01 00 00 00 B0
```

---

# 29. 推荐测试顺序

重构后的 Bootloader 建议按：

```text
1. GET_VERSION
2. GET_DEVICE_ID
3. GET_INFO
4. GET_STATUS
5. READ
6. SET_GUARD
7. Guard状态下ERASE，应拒绝
8. RELEASE_GUARD
9. 未ERASE直接WRITE，应BAD_STATE

10. 真正ERASE
11. WRITE(APP,size)
12. CAN FD单包DATA
13. READ回读
14. 多包DATA
15. 故意漏Sequence
16. WRITE_END
17. MISSING_COUNT / ITEM
18. 补包
19. WRITE_END
20. VERIFY
21. RESET后自动APP校验
22. JUMP_APP

23. Provider单包
24. Provider Range
25. 多节点广播
26. 7+1 Guard
27. 3轮失败
28. Rollback
```

---

# 30. 使用时必须注意的几点

## 30.1 CONTROL CRC 错误不会回复

这是有意设计。

---

## 30.2 READ 是异步多响应

所以必须持续：

```c
Boot_Task();
```

---

## 30.3 WRITE 不等于写数据

`WRITE` 只是建立 Session。

真正数据通过：

```text
BOOT_MESSAGE_DATA
```

发送。

---

## 30.4 APP 写之前必须 ERASE

否则：

```text
BAD_STATE
```

---

## 30.5 APP VERIFY 前必须没有缺包

否则：

```text
BAD_STATE
```

---

## 30.6 JUMP_APP 会重新验证 APP

即使以前 VERIFY 成功，也不会只相信内存状态。

---

## 30.7 Provider Grant 必须单播

广播 Provider Grant 当前：

```text
无响应
```

这是为了避免多个 Provider 同时发送。

---

## 30.8 Guard 第一阶段绝不写

Guard：

```text
ERASE -> 拒绝
WRITE -> 拒绝
DATA  -> 忽略
```

直到：

```text
RELEASE_GUARD
```

---

## 30.9 3轮恢复不是节点自己执行

三轮策略、回滚策略、Provider 选择都属于：

```text
Master Coordinator
```

Bootloader 节点只负责：

```text
执行
报告
提供数据
验证
等待Commit
```

---

# 31. 当前协议的一句话总结

当前协议可以概括为：

> **以 `Boot_Message_t` 作为唯一上下层交换对象，使用控制面完成状态和升级管理，使用数据面完成 56 Byte 分包传输，并通过 Bitmap、缺包汇总、单 Provider 协作、7+1 Guard、CRC32 验证和统一 JUMP_APP 实现多节点可靠固件升级。**

