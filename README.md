# STM32G431 多节点 CAN / CAN FD Bootloader V1

本版本按当前已经确认的协议实现，目标芯片为 **STM32G431，128 KiB Flash**。

当前阶段只实现 **固定 Bootloader + APP 在线升级**，暂不实现 Bootloader 自升级和 Recovery Stub。

---

## 1. Flash 分区

| 区域 | 地址 | 大小 | Bootloader 写权限 |
|---|---|---:|---|
| Bootloader | `0x08000000 ~ 0x08004FFF` | 20 KiB | 禁止 |
| APP | `0x08005000 ~ 0x0801F7FF` | 106 KiB | 允许 |
| Config + APP Metadata | `0x0801F800 ~ 0x0801FFFF` | 2 KiB | 允许 |

STM32G431 Flash Page = 2 KiB，因此：

- Page 0~9：Bootloader
- Page 10~62：APP
- Page 63：Config + Metadata

`ERASE` 只擦 APP Page 10~62，永远不擦 Bootloader。

---

## 2. 总线分层

### Classic CAN：控制面

标准 ID：

```text
0x000
```

负责：

- 查询版本/设备信息
- ENTER_BOOT
- Guard 设置
- ERASE
- WRITE 准备
- READ
- WRITE_END
- VERIFY
- Missing Report
- Provider 调度
- JUMP_APP / RESET
- GET_STATUS

### CAN FD：数据面

标准 ID：

```text
0x100
```

固定 64 Byte，BRS 开启，只负责固件/配置数据。

节点响应：

```text
0x500 + Node_ID
```

例如 Node 1：`0x501`，Node 8：`0x508`。

---

## 3. Classic CAN 控制帧

请求固定 8 Byte：

```text
Byte0      Target
Byte1      Command
Byte2      Command-specific field
Byte3~6    Parameter[4]
Byte7      CRC8(Byte0~Byte6)
```

Target：

```text
0x01 ~ 0x08   指定电机节点
0xFF          广播
```

CRC 使用 STM32G431 自带 CRC 外设：

```text
CRC-8/ATM
Polynomial = 0x07
Init       = 0x00
RefIn      = false
RefOut     = false
XorOut     = 0x00
```

示例：Node1 GET_VERSION：

```text
01 01 00 00 00 00 00 F6
```

---

## 4. Classic CAN 响应帧

```text
Byte0      Node ID
Byte1      Command
Byte2      Status
Byte3~6    Data[4]
Byte7      CRC8(Byte0~Byte6)
```

状态：

| 值 | 状态 |
|---:|---|
| `0x00` | IDLE |
| `0x01` | ERASE |
| `0x02` | WRITE |
| `0x03` | VERIFY / 等待 VERIFY |
| `0x04` | READY |
| `0x05` | ERROR |
| `0x06` | REPAIR |
| `0x07` | GUARD |

---

## 5. 命令表

| CMD | 名称 | 作用 |
|---:|---|---|
| `0x01` | GET_VERSION | 返回 Bootloader 版本 |
| `0x02` | GET_DEVICE_ID | 返回 `DBGMCU->IDCODE` |
| `0x03` | GET_INFO | Node/HW/APP/Config 状态 |
| `0x04` | ENTER_BOOT | 保持在 Bootloader |
| `0x05` | SET_GUARD | 主控指定本轮 Guard 节点 |
| `0x06` | RELEASE_GUARD | 解除 Guard 写保护 |
| `0x10` | ERASE | 擦整个 APP 区 |
| `0x11` | WRITE | 建立 APP/Config 写入会话 |
| `0x12` | READ | 连续读取内部 Flash |
| `0x13` | VERIFY | APP CRC32 完整校验 |
| `0x14` | WRITE_END | 一轮传输结束，统计全部缺包 |
| `0x15` | MISSING_COUNT | 节点向主控报告缺包总数 |
| `0x16` | MISSING_ITEM | 节点逐项报告缺失 Sequence |
| `0x17` | PROVIDER_GRANT | 主控授权唯一 Provider |
| `0x18` | ABORT | 停止本轮更新，留在 Bootloader |
| `0x20` | JUMP_APP | 验证后跳 APP |
| `0x21` | RESET | ACK 后系统复位 |
| `0x30` | GET_STATUS | 查询状态/错误/进度 |

---

## 6. GET_VERSION

返回 4 Byte：

```text
Major Minor Patch Build
```

当前：

```text
1.0.0.0
```

---

## 7. GET_DEVICE_ID

响应 Data[0..3] 为 `DBGMCU->IDCODE`，小端。

该 ID 用于确认 MCU 器件/Revision，不是每块板唯一序列号。

---

## 8. GET_INFO

响应：

```text
Byte3 = Node ID
Byte4 = Hardware Version
Byte5 = APP Valid
Byte6 = Config Valid
```

`APP Valid` 不是简单看向量表：Bootloader 上电时会根据保存的 `app_size + app_crc32` 使用硬件 CRC32 重新校验 APP。

---

## 9. 上电启动逻辑

正常上电 **不等待 500 ms**。

```text
Reset
  ↓
Bootloader
  ↓
检查 TAMP Backup Register BOOT Magic
  ↓
有 BOOT 请求？ ── 是 ─→ 清 Magic，停在 Bootloader
  │
  否
  ↓
Config CRC 正确？
  ↓
app_valid == 1？
  ↓
MSP / Reset_Handler 合法？
  ↓
CRC32(APP, app_size) == app_crc32？
  ↓
立即 Jump APP
```

APP 收到 `ENTER_BOOT` 时应：

```text
ACK
↓
TAMP->BKP0R = BOOT_REQUEST_MAGIC
↓
NVIC_SystemReset()
```

系统复位不会清这个 Backup Register，Bootloader 启动后读取并立即清除。

---

## 10. SET_GUARD / RELEASE_GUARD

### SET_GUARD

建议由主控广播：

```text
Byte0 = 0xFF
Byte1 = 0x05
Byte2 = Guard Node ID
Byte3~6 = 0
Byte7 = CRC8
```

例如 Guard = Node8。

Guard 第一阶段：

- 可以监听控制帧和 CAN FD 数据；
- 可以 READ；
- 可以作为 Provider；
- **绝不执行 ERASE**；
- **绝不执行 WRITE**；
- **绝不执行 WRITE_DATA**；
- WRITE_END / VERIFY 返回 GUARD 状态。

### RELEASE_GUARD

广播：

```text
Byte0 = 0xFF
Byte1 = 0x06
Byte2 = 原 Guard Node ID
```

Node1~7 新 APP 全部验证成功后，再解除 Node8 Guard，然后最后升级 Node8。

---

## 11. ERASE

```text
Byte0 = Target / 0xFF
Byte1 = 0x10
Byte2~6 = 0
Byte7 = CRC8
```

执行顺序：

```text
先把 app_valid = 0 写入 Config
↓
擦除 APP 0x08005000 ~ 0x0801F7FF
↓
全部擦完
↓
回复 READY
```

主控必须等目标节点全部完成 ERASE，才能进入 WRITE。

这样升级中途掉电时不会启动残缺 APP。

---

## 12. WRITE

```text
Byte0      Target
Byte1      0x11
Byte2      Region
Byte3~6    Length，Byte，小端 uint32
Byte7      CRC8
```

Region：

```text
0x00 = APP
0x01 = CONFIG
0x02 = BOOTLOADER（V1 明确拒绝）
```

### APP WRITE

起始地址固定：

```text
0x08005000
```

不从主控接收写地址，避免错误地址覆盖 Bootloader。

### CONFIG WRITE

起始地址固定：

```text
0x0801F800
```

Config 只有 2 KiB，因此实现为：

```text
先把整页复制到 RAM
↓
CAN FD 数据修改 staging buffer
↓
WRITE_END 后统一更新 Config CRC
↓
擦 Page63
↓
整页重新写回
```

因此未修改的字段和页面剩余数据可以保留。

---

## 13. CAN FD WRITE_DATA

标准 ID：

```text
0x100
```

固定 64 Byte：

```text
Byte0      Target
Byte1      WRITE_DATA = 0x01
Byte2~3    Sequence，uint16，小端，从 0 开始
Byte4~7    Reserved = 0
Byte8~63   Firmware Data = 56 Byte
```

APP 地址：

```text
FlashAddress = 0x08005000 + Sequence * 56
```

56 Byte = `7 × 8 Byte`，天然符合 STM32G431 Flash Double Word 8 Byte 编程粒度。

最后一个包不足 56 Byte：

- CAN FD 剩余区域填 `0xFF`；
- Flash 最后不足 8 Byte 的 Double Word 填 `0xFF`；
- APP CRC32 只计算真实 `firmware_size`，不计算填充字节。

---

## 14. Packet Bitmap

最大 APP：106 KiB。

```text
packet_count = ceil(firmware_size / 56)
```

每个节点维护接收 Bitmap。

一个包只有在：

```text
CAN FD 正常接收
↓
Flash Program 成功
↓
Flash Read-back == 本次接收数据
↓
bitmap[sequence] = 1
```

才认为本地拥有这个包。

某个包写失败：

- 不停止后面的数据；
- bitmap 保持 0；
- 后续继续接收；
- WRITE_END 后统一报告缺失。

重复 Sequence：

```text
bitmap[seq] == 1
→ 直接忽略
→ 不重复编程 Flash
```

---

## 15. WRITE_END 和缺包汇总

```text
Byte1 = 0x14
```

主控在一轮数据传输完全结束后发送。

节点扫描 Bitmap，然后先报告：

### MISSING_COUNT `0x15`

Data：

```text
Data0~1 = Missing Count
Data2~3 = Total Packet Count
```

然后每个缺失包发：

### MISSING_ITEM `0x16`

```text
Data0~1 = Missing Sequence
Data2~3 = Missing Item Index
```

例如：

```text
Node2 = {10, 25}
Node3 = {25, 100}
Node5 = {25, 200}
```

主控统一汇总：

```text
Union = {10, 25, 100, 200}
```

`Sequence 25` 只需要选择性广播补一次，所有缺 25 的节点一起修复。

---

## 16. Provider：单包 + Range

`PROVIDER_GRANT = 0x17` **必须单播给 Provider**，绝不能广播授权。

格式：

```text
Byte0      Provider Node ID
Byte1      0x17
Byte2      Data Target
Byte3~4    Start Sequence
Byte5~6    Count
Byte7      CRC8
```

`Count = 1`：单包补传。

`Count > 1`：连续 Range。

例如：

```text
Provider = Node1
Target   = Node8
Start    = 0
Count    = 全部包数
```

可用于最后升级 Guard。

回滚：

```text
Provider = Guard
Target   = 0xFF
Start    = 0
Count    = 旧 APP 全部包数
```

Provider 不是收到授权后一次性把 1900 多个包塞进 TX FIFO，而是由 `BootCAN_Task()` 根据 FDCAN TX FIFO 空间逐包发送。

同一时刻只允许一个 Provider，这是主控状态机必须保证的规则。

---

## 17. 包级 Provider 可信规则

Provider **不要求整个 APP 已经 VERIFY OK**。

如果某节点：

```text
bitmap[105] = 1
```

则说明该节点的 Packet105 已经成功写 Flash 并通过本地 Read-back，因此可在主控授权下提供 Packet105。

因此可以出现：

```text
Node1 不完整
Node2 不完整
Node3 不完整
```

但：

```text
Bitmap1 ∪ Bitmap2 ∪ Bitmap3 ... = Full Firmware
```

主控仍然可以从不同节点为不同 Sequence 选择 Provider，最终拼出完整固件。

这正是后续论文中“分布式包级固件冗余恢复”的核心。

---

## 18. VERIFY

```text
Byte0      Target
Byte1      0x13
Byte2      0
Byte3~6    Expected APP CRC32
Byte7      CRC8
```

CRC32：

```text
Polynomial = 0x04C11DB7
Init       = 0xFFFFFFFF
RefIn      = false
RefOut     = false
XorOut     = 0
```

即 CRC-32/MPEG-2 参数形式，使用 STM32 硬件 CRC 外设计算。

Bootloader：

```text
CRC32(0x08005000, firmware_size)
↓
与 Expected CRC32 比较
```

成功后永久保存：

```text
app_size
app_crc32
app_valid = 1
```

到 Page63 的 Config/Metadata 中。

VERIFY 成功后节点只进入 READY，**绝不自行跳 APP**。

---

## 19. JUMP_APP

`0x20`。

跳转前再次检查：

- Config 合法；
- `app_valid == 1`；
- `app_size` 合法；
- APP MSP 合法；
- Reset_Handler 在 APP Flash 内且为 Thumb 地址；
- 整个 APP CRC32 正确。

只有全部满足才 ACK 并 Jump。

多节点升级中，最终应由中控：

```text
所有目标节点 READY
↓
广播 JUMP_APP
```

实现统一提交。

---

## 20. RESET

`0x21`。

实现为：

```text
先发送 READY ACK
↓
等待 FDCAN TX Queue 尽量发送完
↓
NVIC_SystemReset()
```

---

## 21. GET_STATUS

响应：

```text
Byte3 = 当前 Bootloader Status
Byte4 = 最近一次 Error Code
Byte5 = Progress 0~100
Byte6 = Reserved
```

---

## 22. PCB Config 结构

当前结构固定 84 Byte，位于 Page63 开头：

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
} Boot_PersistConfig_t;
```

保存的是 PCB 个体差异，而不是所有电机都相同的控制参数：

- Node ID；
- Hardware Version；
- A/B/C 三相 ADC 原始零电流点；
- A/B/C 三相 float 增益修正；
- VBUS ADC 原始零点；
- VBUS float 增益修正；
- APP Metadata；
- Config CRC32。

---

## 23. 7+1 Guard 升级流程

主控在升级开始时动态指定 Guard，例如 Node8。

### 新版本第一阶段

```text
PC 只发送一次 BIN
        ↓
中控只做流式转发，不保存完整 BIN
        ↓
Node1~7 ERASE + WRITE
Node8 = Guard，只听，保留旧 APP
        ↓
首轮 CAN FD 广播
        ↓
汇总所有 Missing Bitmap
        ↓
Recovery Round 1
Recovery Round 2
Recovery Round 3
```

每轮均为：

```text
WRITE_END
↓
所有节点完整上报 Missing Set
↓
主控求集合并集
↓
选择唯一 Provider
↓
单包/Range 选择性补传
↓
再次 WRITE_END
```

首轮正常广播 **不计入 3 个恢复轮**。

### Node1~7 成功

```text
Node1~7 VERIFY OK
↓
选一个新固件 Provider
↓
RELEASE_GUARD(Node8)
↓
ERASE + WRITE Node8
↓
Provider Range 发送完整新 APP 给 Node8
↓
Node8 VERIFY OK
↓
8节点 READY
↓
Broadcast JUMP_APP
```

---

## 24. 三轮失败自动回滚

如果新版本经过 3 个补包恢复轮仍然无法完成：

```text
停止新版本更新
↓
Node8 仍保存完整旧 APP
↓
Guard 作为旧固件 Provider
↓
擦除 Node1~7 当前不完整 APP
↓
WRITE(old_app_size)
↓
Guard Range 广播旧 APP
↓
最多 3 个回滚恢复轮
↓
VERIFY old CRC32
↓
全部 READY
↓
统一 JUMP_APP
```

如果旧版本回滚也连续 3 个恢复轮失败：

```text
ABORT
↓
停在 Bootloader
↓
禁止 JUMP_APP
↓
不再无限占用故障总线
↓
等待干扰/总线异常解决后重新更新
```

**三轮计数和自动回滚由中控决定，电机 Bootloader 只负责执行和上报。**

---

## 25. 为什么中控不保存完整 BIN 时节点恢复有意义

系统角色：

```text
PC      = Firmware Origin，一次发送 BIN
中控    = Coordinator + Streaming Gateway，不持久化完整固件
电机节点 = Receiver + Distributed Firmware Source
```

如果 PC 流已经结束，而不同节点分别漏了不同包，中控手里没有完整 BIN 可以随意补发。

但节点 Flash 中已经存在大量成功写入并 Read-back 通过的数据，因此主控可以根据 Missing Set 统一选择 Provider。

即使没有任何一个节点完整，只要所有非 Guard 节点已有数据的并集覆盖完整新固件，也可以互相拼齐。

如果某个 Sequence 在所有升级节点中都缺失，则无法从新版本节点恢复；3轮后自动使用 Guard 的旧版本回滚。

---

## 26. FDCAN 配置必须修改

CubeMX / `fdcan.c` 至少确认：

1. `Mode = FDCAN_MODE_NORMAL`，不要 External Loopback。
2. FDCAN 外设允许 FD+BRS。
3. `StdFiltersNbr >= 2`。
4. Filter0 精确接收 `0x000`。
5. Filter1 精确接收 `0x100`。
6. STM32G4 HAL 的 Message RAM 元素大小由驱动固定配置，RX/TX 元素本身可容纳 64 Byte FD 数据；CubeMX 中无需寻找 Rx/Tx element-size 配置项。
7. `FrameFormat` 必须允许 CAN FD + BRS；单帧是否为 Classic CAN 或 CAN FD 由发送 Header 的 `FDFormat/BitRateSwitch` 决定。
8. 启动 RX FIFO0 NEW MESSAGE 中断。

当前 Ring Buffer 已扩展到 64 Byte 数据，并采用：

```text
FDCAN IRQ
↓
快速取硬件 FIFO
↓
32项 Ring Buffer
↓
主循环 BootCAN_RX_Process()
↓
BootCAN_Task()
```

Flash 编程、CRC、协议状态机都不放在 FDCAN 中断里。

---

## 27. main.c 使用

参考：

```text
integration/main_integration_example.c
```

主循环：

```c
while (1)
{
    BootCAN_RX_Process(&boot_rx);
    BootCAN_Task();
}
```

`BootCAN_Task()` 很重要，它负责：

- READ 连续返回；
- Missing List 分帧上报；
- Provider 单包/Range 非阻塞发送。

---

## 28. APP 工程必须改两件事

### APP Flash 地址

Linker：

```text
ORIGIN = 0x08005000
LENGTH = 106K
```

### APP Vector Table

APP 自己的 `SystemInit()` 运行后，VTOR 必须仍然指向：

```text
0x08005000
```

否则 APP 虽然能进入 `main()`，中断可能跳回 Bootloader 的向量表。

详见：

```text
integration/linker_and_vector_notes.txt
```

---

## 29. PC 端 CRC / 打包参考

`tools/boot_protocol_reference.py` 提供：

- CRC8/ATM；
- STM32 CRC32；
- Classic CAN 控制帧生成；
- WRITE / READ / VERIFY；
- SET/RELEASE Guard；
- PROVIDER_GRANT；
- 56 Byte 固件分包；
- Missing Set 合并；
- 响应 CRC 校验。

可先直接运行：

```bash
python boot_protocol_reference.py
```

参考校验值：

```text
CRC8("123456789")  = 0xF4
CRC32("123456789") = 0x0376E6E7
```

---

## 30. 当前 V1 的边界

已经实现节点侧：

- 固定 20 KiB Bootloader；
- 直接启动 APP；
- Backup Register ENTER_BOOT；
- Classic CAN 控制面；
- CAN FD 64 Byte 数据面；
- APP 边收边写；
- 56 Byte packet；
- Bitmap；
- 不中断丢包；
- Missing Count / Item；
- 包级 Provider；
- 单包 / Range Provider；
- Guard 保护；
- APP CRC32；
- Config + Metadata；
- Flash READ；
- 统一 JUMP_APP 所需节点行为。

暂未实现：

- Bootloader 自升级；
- Recovery Stub；
- 中控侧完整 7+1 / 3-round / rollback 状态机代码。

后两项不会影响当前 APP OTA 主链路验证；中控状态机可以直接基于本协议继续实现。
