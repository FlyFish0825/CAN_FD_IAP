# STM32G431 多节点 CAN / CAN FD Bootloader V1.3

目标芯片：**STM32G431，128 KiB Flash**。Bootloader 固定，不支持 Bootloader 自升级。

V1.3 是基于 V1.2 协议的 **Flash 体积优化版本**。命令、CAN ID、CRC、升级流程和持久化数据格式均保持兼容，`GET_VERSION` 更新为 `1.3.0.0`。本版本主要变化：

- 删除未使用的 UART 初始化和驱动构建依赖；
- Release 保持 `-Oz + LTO + --gc-sections`，增加跨过程指针分析并抑制小函数过度内联；
- 纯 C 固件不再链接无用的 GCC CRT/C++ 构造器启动框架；
- Bootloader 中断向量表裁剪到当前最高使用的 FDCAN1 IT0；
- FDCAN 接收中断只处理 FIFO0 新消息，不再链接 HAL 通用中断分派器；
- Release Flash 从 19,324 B 降到 16,488 B，20 KiB 分区剩余 3,992 B。

V1.2 在 V1.1 的广播升级、Bitmap、Missing、Provider、Session、Coordinator 基础上补齐：

- 首次广播结束后才进行 Coordinator Claim；
- 确定性 Provider 选择；
- 最多 3 轮节点自治选择性修复；
- 新固件分布式 VERIFY；
- 7+1 Guard 更新与旧版本自动回滚；
- Prepare / Commit 协调提交；
- Classic CAN Provider 64B→8×8B 分片发送；
- CAN FD 原生 64B DATA 保持不变。

> `Session ID = 0` 为 Legacy 兼容模式；不启用自治流程时，原来的单节点/CANPro 手动 ERASE→WRITE→DATA→WRITE_END→VERIFY→JUMP 流程仍可使用。

## 1. Flash 分区

| 区域 | 地址 | 大小 |
|---|---|---:|
| Bootloader | `0x08000000 ~ 0x08004FFF` | 20 KiB |
| APP | `0x08005000 ~ 0x0801F7FF` | 106 KiB |
| Config + APP Metadata | `0x0801F800 ~ 0x0801FFFF` | 2 KiB |

APP 必须链接到 `0x08005000`，并设置 `SCB->VTOR = 0x08005000`。

### Bootloader 与 APP 共用 SRAM

Bootloader 和 APP 不会同时执行，因此二者可以使用同一整块 SRAM，不需要在 linker 中各自划分固定 RAM 分区。Bootloader 跳转前清理中断和外设，设置 APP 的 MSP/VTOR；APP 的 Reset_Handler 随后重新复制自己的 `.data`、清零自己的 `.bss`，Bootloader 原有的普通 RAM 内容自然被复用。

若确实需要从 Bootloader 向 APP 传递少量状态，不应依赖普通 `.bss`。可使用备份寄存器、现有 Config/Metadata Flash 页，或由两个 linker script 共同约定的 `NOLOAD` RAM 小区域。

把函数放到 SRAM 执行通常不会减少 Flash，因为函数初始机器码仍需保存在 Flash 再复制到 RAM。当前 V1.3 使用 SRAM 的重点是允许队列和工作缓冲保持较大，而不为节省 RAM 压缩协议状态。

### Bootloader → APP 安全跳转

Bootloader 与 APP 可以使用不同 PLL/系统时钟配置。当前实机验证中，Bootloader 为 170 MHz，而 `Observer_Motor` APP 为 168 MHz；如果直接跳转，APP 的 `HAL_RCC_OscConfig()` 会因为 PLL 仍是当前 SYSCLK 且参数不同而返回 `HAL_ERROR`。

因此 `Boot_RuntimeJumpToApp()` 在设置 MSP/VTOR 和执行 APP Reset_Handler 前，会先执行：

```c
HAL_DeInit();
HAL_RCC_DeInit();
```

并关闭 SysTick、清 NVIC enable/pending、清 PendSV/SysTick pending，使 APP 从接近上电复位的时钟/外设状态重新执行自己的 `HAL_Init()` 和 `SystemClock_Config()`。这个步骤是 APP 可独立配置时钟的必要条件，不应删除。

## 2. CAN 映射

| 方向 | 类型 | 标准 ID | 帧 |
|---|---|---:|---|
| Host/PC → Nodes | CONTROL | `0x000` | Classic CAN，8 Byte |
| Node → Host/PC | RESPONSE | `0x500 + Node_ID` | Classic CAN，8 Byte |
| Node → Nodes | PEER CONTROL | `0x600 + Node_ID` | Classic CAN，8 Byte |
| Host/Provider → Nodes | DATA | `0x100` | CAN FD+BRS，64 Byte |
| Classic 兼容 DATA | Fragments | `0x100 ~ 0x107` | 8×Classic CAN |

Peer Control 使用 `0x601 ~ 0x608`。低 Node ID 同时具有更低 CAN ID 和更早 Claim 时隙。

当前过滤器：

```text
Filter0  0x000        Host CONTROL
Filter1  0x100~0x107  DATA / Classic fragments
Filter2  0x600~0x60F  Peer Control（Adapter 再限制到 0x601~0x608）
```

`StdFiltersNbr >= 3`。

## 3. Core / Transport 解耦

Core 只交换 `Boot_Message_t`，不依赖 FDCAN/UART/SPI/I2C：

```c
typedef struct {
    uint8_t type;
    uint16_t len;
    uint8_t data[64];
} Boot_Message_t;
```
类型：

```text
BOOT_MESSAGE_CONTROL       Host请求 / Node响应
BOOT_MESSAGE_DATA          固件DATA
BOOT_MESSAGE_PEER_CONTROL  节点间协调控制
```

主循环：

```c
while (1)
{
    BootPort_CAN_Task(&hfdcan1);  /* Classic Provider TX 分片 */
    Boot_Task();
}
```

RX 中断只完成物理帧→`Boot_Message_t`映射和入队，Flash/CRC/恢复状态机都在 `Boot_Task()` 执行。

## 4. Host CONTROL / RESPONSE

Host 请求固定 8 Byte：

```text
Byte0      Target：0x01~0x08 / 0xFF
Byte1      Command
Byte2      Command-specific
Byte3~6    Parameter[4]
Byte7      CRC8(Byte0~Byte6)
```

响应：`[Node, Command, Status, Data0..3, CRC8]`。

CRC8：Poly=`0x07`，Init=`0x00`，无反射，XorOut=`0`。
APP CRC32：Poly=`0x04C11DB7`，Init=`0xFFFFFFFF`，无反射，XorOut=`0`。
## 5. Session 与 DATA

`SESSION_BEGIN = 0x07`：

```text
Byte0      Target，自治模式通常 0xFF
Byte1      0x07
Byte2      Flags
Byte3~4    Session ID，uint16 LE，非0
Byte5~6    Reserved
Byte7      CRC8
```

Flags：

```text
bit0  PEER_RECOVERY   节点自治修复
bit1  GUARD_ROLLBACK  新版本失败允许由Guard回滚
bit2  COORD_COMMIT    启用分布式VERIFY + Prepare/Commit
```

`SESSION_CRC32 = 0x08` 用于在自治提交前保存本次新 APP 的 Expected CRC32。

DATA 64 Byte：

```text
Byte0       Target
Byte1       WRITE_DATA = 0x01
Byte2~3     Sequence，uint16 LE
Byte4~5     Session ID，uint16 LE
Byte6~7     Reserved = 0
Byte8~63    Firmware Payload = 56 Byte
```

地址=`0x08005000 + Sequence*56`。最后一包补 `0xFF`，CRC32 只覆盖真实 firmware_size。
## 6. 命令分类

为便于 CANPro 调试和源码定位，V1.2 不再把所有命令只按数值排列，而是按用途分为 7 类。完整逐命令测试帧、可能接收、失败响应和判断方法见 [`COMMAND_TEST_GUIDE.md`](COMMAND_TEST_GUIDE.md)。

### A. 基础查询 / Boot 控制

| CMD | 名称 | 方向 | 直接回复 | 作用 |
|---:|---|---|---|---|
| `0x01` | GET_VERSION | Host→Node | RESPONSE | 查询 Bootloader 版本 |
| `0x02` | GET_DEVICE_ID | Host→Node | RESPONSE | 读取 `DBGMCU->IDCODE` |
| `0x03` | GET_INFO | Host→Node | RESPONSE | Node/HW/APP/Config 摘要 |
| `0x04` | ENTER_BOOT | Host→Node | RESPONSE | 本次运行保持 Bootloader |
| `0x18` | ABORT | Host→Node | RESPONSE | 取消异步任务并中止当前写会话 |
| `0x20` | JUMP_APP | Host→Node | READY 后跳转 | 验证持久化 APP 后跳转 |
| `0x21` | RESET | Host→Node | READY 后复位 | Flush TX 后系统复位 |
| `0x30` | GET_STATUS | Host→Node | RESPONSE | 查询 Status/Error/Progress |

### B. Session / Guard 配置

| CMD | 名称 | 方向 | 直接回复 | 作用 |
|---:|---|---|---|---|
| `0x05` | SET_GUARD | Host→Nodes | 每节点 RESPONSE | 指定本次升级 Guard |
| `0x06` | RELEASE_GUARD | Host→Nodes | 每节点 RESPONSE | 解除 Guard 角色 |
| `0x07` | SESSION_BEGIN | Host→Nodes | 每节点 RESPONSE | 建立新升级事务并清理旧 RAM 状态 |
| `0x08` | SESSION_CRC32 | Host→Nodes | 每节点 RESPONSE | 设置本次新 APP Expected CRC32 |

### C. Flash 下载 / 读取 / Legacy 校验

| CMD | 名称 | 方向 | 直接回复 | 作用 |
|---:|---|---|---|---|
| `0x10` | ERASE | Host→Node(s) | `ERASE` 后 `READY` | 擦除 APP，擦除前先使 metadata 失效 |
| `0x11` | WRITE | Host→Node(s) | `WRITE` | 建立 APP/Config 写会话，返回总包数 |
| `DATA 0x01` | WRITE_DATA | Host/Provider→Node(s) | 无逐包 ACK | 每个逻辑包携带 56B 固件数据 |
| `0x12` | READ | Host→Node | 异步多帧 RESPONSE | 每帧返回最多 4B Flash 数据 |
| `0x14` | WRITE_END | Host→Node(s) | `VERIFY/REPAIR/READY` | 结束首次广播并扫描 Bitmap |
| `0x13` | VERIFY | Host→Node | RESPONSE | 仅 Legacy；CRC/向量通过后置 `app_valid=1` |

### D. Legacy 缺包 / 人工 Provider

| CMD | 名称 | 方向 | 直接回复 | 作用 |
|---:|---|---|---|---|
| `0x15` | MISSING_COUNT | Node→Host | 报告帧 | 缺包数量 + 总包数 |
| `0x16` | MISSING_ITEM | Node→Host | 报告帧 | 逐个报告缺失 Sequence |
| `0x17` | PROVIDER_GRANT | Host→Provider | `WRITE`，结束再 `READY` | Host 人工指定唯一 Provider 发送 Seq/Range |

### E. 自治选主 / Provider / Repair

| CMD | 名称 | 方向 | 直接回复 | 作用 |
|---:|---|---|---|---|
| `0x19` | COORDINATOR_CLAIM | Node→Nodes | 无 ACK | 最低有效 Node ID 声明 Coordinator |
| `0x1A` | PROVIDER_ASSIGN | Coordinator→Provider | 无 ACK | 指派某一 Sequence 的 Provider |
| `0x1B` | PROVIDER_DONE | Provider→Coordinator | 无 ACK | DATA 已 Flush，Provider 本次发送完成 |
| `0x1C` | REPAIR_ROUND_END | Coordinator→Nodes | 无 ACK | 进入下一轮 Missing 重报 |
| `0x1D` | RECOVERY_READY | Coordinator→Nodes | 无 ACK | 自治恢复阶段完成通知 |
| `0x1E` | RECOVERY_FAILED | Coordinator→Nodes | 无 ACK | 自治恢复最终失败通知 |

### F. 分布式 VERIFY / Guard / Rollback

| CMD | 名称 | 方向 | 直接回复 | 作用 |
|---:|---|---|---|---|
| `0x22` | VERIFY_REQUEST | Coordinator→成员 | `VERIFY_RESULT` | 请求本地 CRC + 向量表校验 |
| `0x23` | VERIFY_RESULT | 成员→Coordinator | 无二次 ACK | 返回 context + 成功/失败 |
| `0x24` | GUARD_UPDATE_BEGIN | Coordinator→Guard | `GUARD_UPDATE_READY` | Guard 解除保护、擦除并准备接收新 APP |
| `0x25` | GUARD_UPDATE_READY | Guard→Coordinator | 无二次 ACK | Guard 返回准备结果 |
| `0x26` | ROLLBACK_REQUEST | Coordinator→Guard | 后续 4 个 metadata 帧 | 请求旧 APP size/CRC |
| `0x27` | ROLLBACK_SIZE_LO | Guard→Nodes | 无 ACK | 旧 APP size 低 16 bit |
| `0x28` | ROLLBACK_SIZE_HI | Guard→Nodes | 无 ACK | 旧 APP size 高 16 bit |
| `0x29` | ROLLBACK_CRC_LO | Guard→Nodes | 无 ACK | 旧 APP CRC32 低 16 bit |
| `0x2A` | ROLLBACK_CRC_HI | Guard→Nodes | 无 ACK | 旧 APP CRC32 高 16 bit |
| `0x2B` | ROLLBACK_BEGIN | Coordinator→成员 | `ROLLBACK_PREPARED` | 普通节点擦除并准备接收旧镜像 |
| `0x2C` | ROLLBACK_PREPARED | 成员→Coordinator | 无二次 ACK | 返回回滚准备结果 |
| `0x2D` | FULL_STREAM | Coordinator→Provider | 无 ACK；随后 DATA | Provider 广播完整镜像 |

### G. Prepare / Commit

| CMD | 名称 | 方向 | 直接回复 | 作用 |
|---:|---|---|---|---|
| `0x2E` | COMMIT_PREPARE | Coordinator→成员 | `COMMIT_ACK` | 成员再次确认镜像可提交 |
| `0x2F` | COMMIT_ACK | 成员→Coordinator | 无二次 ACK | 返回本地 Commit arm 结果 |
| `0x31` | COMMIT_EXECUTE | Coordinator→成员 | 无 ACK | 重复广播 3 次；置 `app_valid=1` 后延时跳 APP |

> `0x15/0x16` 在 Legacy 模式走 `Node→Host RESPONSE (0x50x)`；自治模式复用相同命令值，但走 `Node→Nodes PEER CONTROL (0x60x)`。`PROVIDER_ASSIGN/FULL_STREAM` 成功时不要等待 ACK，应观察后续 DATA 和最终 `PROVIDER_DONE`。

## 7. 自治升级流程

第一次固件广播阶段不设置 Coordinator。Host 负责：

```text
SESSION_BEGIN
SET_GUARD（可选）
ERASE / WRITE
DATA Sequence 0..N
WRITE_END
```

`WRITE_END` 后各非Guard节点自动发送 Missing 信息，并进入 Coordinator Claim。
确定性规则：

```text
Coordinator = 参与本次升级且成功Claim的最低 Node ID
Provider    = 拥有目标Sequence、非Guard、优先级最高（最低Node ID）的节点
```

Claim 使用 ID 时隙：Node1 最早，随后 Node2…Node8。较低 ID 未在线/未Claim时，后续节点自然接管。

Coordinator 收集所有节点 Missing 集合后，对缺失 Sequence 做并集修复。同一个 Sequence 多节点缺失时只广播一次。

每轮修复结束后重新报告 Missing，初始广播不计入 Repair Round；最多 3 轮。新版本修复失败时：若启用了 Guard Rollback，则进入回滚；否则 `RECOVERY_FAILED`。

## 8. 7+1 Guard 与自动回滚

Guard 在第一阶段保持旧APP：

```text
SET_GUARD
↓
7个普通节点擦除/写新APP
Guard只监听，不擦除、不写入
```

若普通节点新版本在修复/VERIFY阶段失败：

```text
Coordinator → ROLLBACK_REQUEST
Guard → 返回旧APP size + CRC32
普通节点 → 擦APP并进入ROLLBACK_PREP
Guard → FULL_STREAM广播旧APP
缺包 → 最多3轮选择性修复
所有节点 → CRC32 VERIFY旧APP
```

回滚本身仍失败时进入 `RECOVERY_FAILED`，节点保持Bootloader，不跳APP。
普通节点新版本全部 VERIFY 成功后才更新 Guard：

```text
Coordinator → GUARD_UPDATE_BEGIN
Guard解除保护、擦除旧APP
最低 Node ID 的合格新版本 Provider → FULL_STREAM到Guard
Guard缺包则进入选择性修复
Guard VERIFY新APP
↓
Prepare / Commit
```

因此旧版本回滚能力只覆盖“7个普通节点的新版本阶段”；一旦进入 Guard 更新阶段，旧Guard镜像已被释放。

## 9. Prepare / Commit

自治模式不会在单个节点 VERIFY 成功后立即设置 `app_valid=1`。

VERIFY 成功时先保存：

```text
app_size
app_crc32
app_valid = 0
```

即“镜像已验证但尚未提交”。Coordinator 确认所有目标节点准备完成后：

```text
COMMIT_PREPARE
↓
所有节点验证本地镜像可启动
↓
COMMIT_ACK
↓
Coordinator收齐ACK
↓
COMMIT_EXECUTE（重复3次）
↓
各节点持久化 app_valid=1
↓
短延时后统一 Jump APP
```

任何节点未准备好时不执行全局 Commit。
## 10. 状态 / 错误 / Recovery Phase

Status：`IDLE=0x00, ERASE=0x01, WRITE=0x02, VERIFY=0x03, READY=0x04, ERROR=0x05, REPAIR=0x06, GUARD=0x07`。

新增错误：

```text
0x12 SESSION
0x13 COORDINATOR
0x14 RECOVERY_FAILED
0x15 COMMIT
```

Recovery Phase：

```text
0x00 IDLE
0x01 NEW_REPAIR
0x02 NEW_VERIFY
0x03 GUARD_UPDATE
0x04 GUARD_REPAIR
0x05 ROLLBACK_META
0x06 ROLLBACK_PREP
0x07 ROLLBACK_REPAIR
0x08 ROLLBACK_VERIFY
0x09 COMMIT
0x0A FAILED
```

调试 API：`Boot_GetSessionId()`、`Boot_GetCoordinatorId()`、`Boot_IsCoordinator()`、`Boot_GetRepairRound()`、`Boot_GetRecoveryPhase()`。

## 11. Classic CAN / CAN FD

正式 CAN FD：一个逻辑 DATA 就是一帧 `ID=0x100, FD=1, BRS=1, DLC=64`。

Classic 兼容：同一逻辑 64B DATA 严格拆为 `0x100~0x107` 八帧，每帧8B；丢片/乱序时整包丢弃，最终由 Sequence Bitmap 判为 Missing。
## 12. bin_to_boot_frames.py

Legacy 单节点/手动模式：

```bash
python bin_to_boot_frames.py app_boot.bin --mode classic --target 1
```

完整 8 节点自治模式示例：

```bash
python bin_to_boot_frames.py app_boot.bin \
  --mode classic \
  --target 0xFF \
  --session 0x1234 \
  --session-flags 0x07 \
  --guard 8
```

脚本打印：Firmware size、Packet count、APP CRC32、SESSION_BEGIN、SESSION_CRC32、SET_GUARD、ERASE、WRITE、WRITE_END，以及控制帧；COORD_COMMIT 模式会省略 Legacy VERIFY，并生成 Classic CANPro 列表或 CAN FD 帧列表。

正式 CAN FD 使用 `--mode fd`。

## 13. Release 编译

```bash
cmake --build --preset Release
```

当前 V1.3 Release 已 clean build 通过。Linker 已锁定 `FLASH ORIGIN=0x08000000, LENGTH=20K`，超过 `0x08004FFF` 会直接链接失败。

```text
FLASH 16488 B / 20 KiB  80.51%  （剩余 3992 B）
RAM    7448 B / 32 KiB  22.73%
0 compiler/linker error
```

体积变化记录：

| 阶段 | Flash | 相对最初版本减少 |
|---|---:|---:|
| V1.2 安全 Jump 完成后 | 19,324 B | — |
| 删除未使用 UART | 17,688 B | 1,636 B |
| Release 细化优化、移除空 GPIO 初始化 | 17,344 B | 1,980 B |
| V1.3 精简启动、向量表和 FDCAN ISR | **16,488 B** | **2,836 B** |

Flash 数值按 ELF 的 `text + data` 统计，不是 `.elf` 文件在电脑上的文件大小。必须使用 Release；Debug 的 `-O0/-g3` 用于调试，不满足 20 KiB 体积约束。

V1.3 的启动文件是纯 C 固件专用配置，不调用静态构造器。向量表目前只覆盖到外部 IRQ21（FDCAN1 IT0）；以后若 Bootloader 新增 IRQ22 或更高编号的外设中断，必须同步恢复向量表至对应表项。
## 14. 自治模式总线流量约束

自 V1.2 起，`WRITE_END` 后的 Missing 流量有两点限制：

- Legacy/单播流程仍通过 `0x50x` 向 Host 输出完整 Missing Report；
- 自治广播流程不再重复向 Host 流式发送 `MISSING_ITEM`，详细缺包只走 `0x60x` Peer Control；Host 仍能收到各节点即时 `WRITE_END` 状态。

Peer Missing Report 先发送 `MISSING_COUNT`，随后等待 `BOOT_PEER_MISSING_ITEM_DELAY_MS=20 ms` 再发送明细，使 Node1~Node8 有机会先完成 Count/在线声明，避免低 CAN ID 节点连续 Missing Item 长时间压住高 ID 节点。

`SESSION_BEGIN` 是新的事务边界：会取消旧异步任务/旧写会话、清 Bitmap、清 RAM 中旧 Guard 角色；因此完整自治流程应按 `SESSION_BEGIN → SESSION_CRC32 → SET_GUARD → ERASE → WRITE` 的顺序开始。

Host 在首次广播 DATA 前仍应确认目标节点已经成功接收关键控制面命令（尤其是 SESSION/ERASE/WRITE）；自治恢复针对的是 DATA 缺包，不把“节点连 WRITE 会话都没建立”当作普通 Sequence 缺包处理。
