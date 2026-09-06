# STM32G431 多节点 CAN / CAN FD Bootloader 协议与使用说明 V1.2

> 目标：STM32G431，128 KiB Flash，最多 8 个节点。
> 核心：`bootloader.c / bootloader.h`。
> CAN 适配：`boot_port_can_stm32g4.c/.h`。
> 帧生成：`bin_to_boot_frames.py`。
> 本文档以当前源码实际实现为准。
>
> CANPro/总线逐命令实测帧、可能接收、失败响应与判断方法见 [`COMMAND_TEST_GUIDE.md`](COMMAND_TEST_GUIDE.md)。

## 1. V1.2 目标

V1.2 同时保留两种工作方式：

- Legacy：Host 直接完成 ERASE→WRITE→DATA→WRITE_END→VERIFY→JUMP；
- Autonomous：Host 只负责首次固件注入，随后 8 个 Bootloader 节点自治完成缺包修复、校验、Guard 更新/回滚和协调提交。

自治模式新增：Session ID、Coordinator Claim、确定性 Provider、最多 3 轮 Repair、分布式 VERIFY、7+1 Guard、Rollback、Prepare/Commit。

Bootloader 本身不支持自升级，Bootloader 区始终受保护。

## 2. Flash 布局

| 区域 | 地址 | 大小 |
|---|---|---:|
| Bootloader | `0x08000000~0x08004FFF` | 20 KiB |
| APP | `0x08005000~0x0801F7FF` | 106 KiB |
| Config + APP Metadata | `0x0801F800~0x0801FFFF` | 2 KiB |
STM32G431 Flash Page = 2 KiB：Page 0~9 为 Bootloader，Page 10~62 为 APP，Page 63 为 Config/Metadata。

APP 必须链接到 `0x08005000`，启动后设置 `SCB->VTOR = 0x08005000`。

Bootloader linker 已锁定：

```text
FLASH ORIGIN = 0x08000000
FLASH LENGTH = 20K
```

因此 Bootloader 超过 `0x08004FFF` 会在链接阶段直接失败。

## 3. Core / Transport 解耦

Core 唯一交换对象：

```c
typedef struct {
    uint8_t type;
    uint16_t len;
    uint8_t data[64];
} Boot_Message_t;
```

`type` 当前有三类：`BOOT_MESSAGE_CONTROL`、`BOOT_MESSAGE_DATA`、`BOOT_MESSAGE_PEER_CONTROL`。

RX ISR 只做物理帧解析和入队，Flash、CRC、状态机都在 `Boot_Task()` 中执行。
主循环：

```c
while (1)
{
    BootPort_CAN_Task(&hfdcan1); /* Classic Provider TX分片 */
    Boot_Task();
}
```

## 4. CAN / CAN FD 映射

| 方向 | 逻辑类型 | 标准 ID | 物理帧 |
|---|---|---:|---|
| Host→Nodes | CONTROL | `0x000` | Classic CAN, 8B |
| Node→Host | RESPONSE | `0x500+Node_ID` | Classic CAN, 8B |
| Node→Nodes | PEER CONTROL | `0x600+Node_ID` | Classic CAN, 8B |
| Host/Provider→Nodes | DATA | `0x100` | CAN FD+BRS, 64B |
| Classic fallback | DATA fragments | `0x100~0x107` | 8×8B |

Peer Control 精确范围为 `0x601~0x608`。

FDCAN 标准过滤器数量至少为 3：

```text
Filter0: 0x000
Filter1: 0x100~0x107
Filter2: 0x600~0x60F，Adapter 再限制 0x601~0x608
```
## 5. CRC

CONTROL / RESPONSE / PEER CONTROL 使用 CRC8：Poly=`0x07`，Init=`0x00`，无反射，XorOut=`0`；计算 Byte0~Byte6，Byte7 放 CRC。

APP / Config 使用 CRC32：Poly=`0x04C11DB7`，Init=`0xFFFFFFFF`，无反射，XorOut=`0`，即当前代码的 CRC-32/MPEG-2 字节馈送方式。

测试向量：`"123456789"` 的 CRC8=`0xF4`，CRC32=`0x0376E6E7`。

## 6. Host CONTROL / RESPONSE

Host CONTROL 固定 8B：

```text
Byte0      Target：1~8 或 0xFF
Byte1      Command
Byte2      Command-specific
Byte3~6    Parameter[4]
Byte7      CRC8
```

Node RESPONSE 固定 8B：

```text
Byte0      Node ID
Byte1      Command
Byte2      Status
Byte3~6    Data[4]
Byte7      CRC8
```

响应 CAN ID=`0x500+Node_ID`。
## 7. Session

`SESSION_BEGIN = 0x07`：

```text
Byte0      Target
Byte1      0x07
Byte2      Flags
Byte3~4    Session ID，uint16 LE，必须非0
Byte5~6    Reserved
Byte7      CRC8
```

Flags：

```text
bit0  0x01  PEER_RECOVERY
bit1  0x02  GUARD_ROLLBACK
bit2  0x04  COORD_COMMIT
```

完整自治模式通常使用 `Flags=0x07`。

`SESSION_CRC32 = 0x08`：Byte3~6 直接携带新 APP Expected CRC32（uint32 LE）。自治分布式 VERIFY / Commit 依赖这个值。

Session ID 表示“哪一次升级事务”，CRC32 表示“是什么固件”。即使连续发送相同 BIN，也建议更换 Session ID。

Session=0 保留给 Legacy 模式；旧 DATA 不带 Session 时 Byte4~5 为 0。
## 8. DATA 64B 格式

```text
Byte0       Target
Byte1       WRITE_DATA = 0x01
Byte2~3     Sequence，uint16 LE
Byte4~5     Session ID，uint16 LE
Byte6~7     Reserved = 0
Byte8~63    Firmware Payload = 56B
```

APP 地址：`0x08005000 + Sequence*56`。

最后一包不足 56B 时发送端补 `0xFF`，但 APP CRC32 只覆盖真实 `firmware_size`。

节点每包执行：写 Flash → read-back 验证 → `bitmap[seq]=1`。重复 Sequence 已有 bit=1 时直接忽略。

单个 DATA 包写失败不会终止整次广播；bit 保持 0，后续由 Missing/Repair 修复。

### Classic fallback

一个逻辑 64B DATA 严格拆成：

```text
0x100 = bytes 0..7
0x101 = bytes 8..15
...
0x107 = bytes 56..63
```

必须按顺序到达；缺片/乱序则丢弃整逻辑包。
## 9. 命令表（Host / Legacy）

| CMD | 名称 | 说明 |
|---:|---|---|
| `0x01` | GET_VERSION | Bootloader 版本 |
| `0x02` | GET_DEVICE_ID | `DBGMCU->IDCODE` |
| `0x03` | GET_INFO | Node/HW/APP/Config |
| `0x04` | ENTER_BOOT | 保持 Bootloader |
| `0x05` | SET_GUARD | 指定 Guard 节点 |
| `0x06` | RELEASE_GUARD | 解除 Guard |
| `0x07` | SESSION_BEGIN | 建立 Session |
| `0x08` | SESSION_CRC32 | 设置新 APP Expected CRC32 |
| `0x10` | ERASE | 擦除 APP |
| `0x11` | WRITE | 建立 APP/Config 写会话 |
| `0x12` | READ | 读内部 Flash |
| `0x13` | VERIFY | Legacy CRC32 校验并持久化 app_valid |
| `0x14` | WRITE_END | 扫描 Bitmap；广播时启动自治阶段 |
| `0x15` | MISSING_COUNT | 缺包数 |
| `0x16` | MISSING_ITEM | 缺失 Sequence |
| `0x17` | PROVIDER_GRANT | Legacy Host 指定 Provider 单包/Range |
| `0x18` | ABORT | 中止 |
| `0x20` | JUMP_APP | Legacy 跳 APP |
| `0x21` | RESET | ACK 后复位 |
| `0x30` | GET_STATUS | Status/Error/Progress |
## 10. 节点间 Peer Control

Peer Control CAN ID=`0x600+SourceNode`，8B 通用格式：

```text
Byte0      Target：Node ID 或 0xFF
Byte1      Peer Command
Byte2      Source Node ID
Byte3~4    Session ID，uint16 LE
Byte5~6    Value，uint16 LE
Byte7      CRC8
```

Adapter 会检查 `Byte2 == CAN_ID-0x600`，Core 再检查 Session ID。

| CMD | 名称 | 作用 |
|---:|---|---|
| `0x19` | COORDINATOR_CLAIM | Coordinator 声明 |
| `0x1A` | PROVIDER_ASSIGN | 指派某 Sequence Provider |
| `0x1B` | PROVIDER_DONE | Provider 完成通知 |
| `0x1C` | REPAIR_ROUND_END | 请求重新上报 Missing |
| `0x1D` | RECOVERY_READY | 自治恢复完成 |
| `0x1E` | RECOVERY_FAILED | 自治恢复失败 |
| `0x22` | VERIFY_REQUEST | 分布式 VERIFY 请求 |
| `0x23` | VERIFY_RESULT | 分布式 VERIFY 结果 |
| `0x24` | GUARD_UPDATE_BEGIN | Guard 开始更新 |
| `0x25` | GUARD_UPDATE_READY | Guard 擦除/准备完成 |
| `0x26` | ROLLBACK_REQUEST | 请求 Guard 旧镜像元数据 |
| `0x27` | ROLLBACK_SIZE_LO | 旧 APP size 低16位 |
| `0x28` | ROLLBACK_SIZE_HI | 旧 APP size 高16位 |
| `0x29` | ROLLBACK_CRC_LO | 旧 APP CRC 低16位 |
| `0x2A` | ROLLBACK_CRC_HI | 旧 APP CRC 高16位 |
| `0x2B` | ROLLBACK_BEGIN | 普通节点准备接收旧镜像 |
| `0x2C` | ROLLBACK_PREPARED | 回滚擦除/准备完成 |
| `0x2D` | FULL_STREAM | Provider 发送完整镜像 |
| `0x2E` | COMMIT_PREPARE | Prepare 阶段 |
| `0x2F` | COMMIT_ACK | 节点提交条件确认 |
| `0x31` | COMMIT_EXECUTE | Commit 执行 |

`MISSING_COUNT/MISSING_ITEM` 在自治模式也通过 Peer Control 发送，命令值仍为 `0x15/0x16`。

## 11. Coordinator 产生规则

第一次 DATA 广播期间没有固定 Coordinator。只有收到广播 `WRITE_END` 后，非 Guard 节点才启动 Peer Missing Report 和选举。

当前 Claim 时序：

```text
基础等待 200 ms
Node1: 200 ms
Node2: 210 ms
...
Node8: 270 ms
```

较低 ID 先 Claim；如果低 ID 节点未能 Claim，后续 ID 在自己的时隙接管。收到更低 ID 的有效 Claim 时会接受更低 ID。
Coordinator 的 `primary_members_bitmap` 来自首次 Missing Report 阶段已经出现的普通节点；Guard 不属于 primary members。

## 12. Provider 选择

对每一个缺失 Sequence，Coordinator 根据各节点 Missing Bitmap 选择 Provider。

当前确定性规则：

1. 正常新固件 Repair：只在 primary members 中选择；
2. Guard 节点在正常新固件阶段不作为 Provider；
3. 优先使用已通过分布式 VERIFY 的节点；否则选择明确“不缺该 Sequence”的节点；
4. 从 Node1 向 Node8 扫描，因此满足条件的最低 Node ID 优先；
5. Rollback Repair 阶段固定优先由 Guard 提供旧固件。

Coordinator 与 Provider 是两个角色：Coordinator 自己缺某 Sequence 时，仍可让另一个节点提供；Coordinator 对它拥有的其他 Sequence 也可以成为 Provider。

Provider DATA 仍使用统一 64B DATA 格式，因此同一个 Sequence 多个节点缺失时只需广播一次。

## 13. Repair Round

首次完整广播不算 Repair Round。

每轮选择性修复后，Coordinator 发送 `REPAIR_ROUND_END`，目标节点重新计算并报告 Missing。

`BOOT_MAX_REPAIR_ROUNDS = 3`。达到限制仍不能恢复时进入阶段失败处理：新版本阶段可转 Rollback；其他阶段进入 `RECOVERY_FAILED`。

## 14. 分布式 VERIFY

当 `COORD_COMMIT` flag 开启时，新固件 Repair 完成后不会直接宣布可启动，而是进入 `NEW_VERIFY`。

Coordinator 必须已经从 `SESSION_CRC32` 得到 Expected CRC32，否则本阶段失败。

每个成员执行：

```text
MissingCount == 0
↓
计算 g_write_size 范围 CRC32
↓
检查 CRC32 == Expected
↓
检查 MSP / Reset_Handler 向量合法
↓
保存 app_size/app_crc32，但 app_valid=0
↓
VERIFY_RESULT
```

`app_valid=0` 是 Prepare/Commit 的关键：单个节点提前验证通过也不能自行启动新 APP。

Coordinator 收齐所有成员成功结果后，才进入 Guard 更新或 Commit。

分布式阶段超时常量：`BOOT_PEER_PHASE_TIMEOUT_MS = 5000 ms`。

Provider 数据发送/完成通知单独使用 `BOOT_PROVIDER_TIMEOUT_MS = 10000 ms`。Provider 超时后 Coordinator 将该 Provider 临时排除，并尝试同一 Sequence 的下一个合法 Provider；若本轮仍无法恢复，则重新收集 Missing，最多 3 轮。
## 15. 7+1 Guard

Guard 通过 `SET_GUARD` 指定。当前代码不是自动选择 Guard；Host 应在首次广播前明确设置。

第一阶段 Guard 行为：

```text
保留旧 APP
不擦除 APP
不写新 DATA
不参加普通节点 Missing/Provider
```

因此普通 7 节点的新固件阶段失败时，Guard 仍持有可验证的旧 APP。

### 新固件成功后的 Guard 更新

普通节点 Repair + VERIFY 全部成功后：

```text
Coordinator → GUARD_UPDATE_BEGIN
Guard 解除保护并擦除 APP
Coordinator 选择已验证的新版本 Provider
Provider → FULL_STREAM 到 Guard
Guard Missing → 最多3轮 Repair
Guard VERIFY 新 APP
```

Guard 更新成功后进入全局 Commit。

注意：进入 `GUARD_UPDATE_BEGIN` 后旧 Guard APP 会被擦除；因此旧版本回滚保护覆盖的是“普通节点新版本 Repair/VERIFY 阶段”，不是 Guard 自身更新失败后的再次回滚。
## 16. 自动 Rollback

只有 `GUARD_ROLLBACK` flag 开启且 Guard 有有效旧 APP 时才允许自动回滚。

新固件普通节点 Repair/VERIFY 失败后：

```text
Coordinator → ROLLBACK_REQUEST
Guard → SIZE_LO / SIZE_HI / CRC_LO / CRC_HI
Coordinator 获得旧 APP size + CRC32
↓
primary members 擦除当前 APP
↓
ROLLBACK_PREPARED
↓
Guard → FULL_STREAM 广播旧 APP
↓
缺包继续最多3轮选择性修复
↓
ROLLBACK_VERIFY
```

Rollback VERIFY 同样检查完整 CRC32 和向量表。

回滚成功后不会直接跳 APP，而是走同一个 Prepare/Commit；回滚也失败则进入 `RECOVERY_FAILED`，保持 Bootloader。

Guard 发送完整旧镜像时的数据来源是其持久化的、当前 CRC 和向量检查仍然有效的 APP。

## 17. Prepare / Commit

自治模式把“镜像准备完成”和“允许启动”分开。
VERIFY 成功但尚未 Commit：

```text
app_size  = prepared size
app_crc32 = prepared crc
app_valid = 0
```

Coordinator 发：

```text
COMMIT_PREPARE
↓
各目标节点再次确认本地镜像可启动
↓
COMMIT_ACK
↓
Coordinator 收齐 ACK
↓
COMMIT_EXECUTE，重复 3 次
```

节点收到 `COMMIT_EXECUTE` 后持久化 `app_valid=1`，当前 `BOOT_COMMIT_DELAY_MS=50 ms`，随后 Flush TX 并 Jump APP。

重复 3 次用于降低 Commit 控制帧丢失概率；当前实现属于轻量级 CAN 协调提交，并不是具有日志复制/事务恢复能力的强一致分布式数据库协议。

如果某节点在 Commit 前复位，因为 `app_valid=0`，它会留在 Bootloader，而不是启动半完成的新镜像。

## 18. Recovery Phase

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

调试 API：

```c
Boot_GetSessionId();
Boot_GetCoordinatorId();
Boot_IsCoordinator();
Boot_GetRepairRound();
Boot_GetRecoveryPhase();
```

## 19. Status / Error

Status：

```text
0x00 IDLE   0x01 ERASE   0x02 WRITE   0x03 VERIFY
0x04 READY  0x05 ERROR   0x06 REPAIR  0x07 GUARD
```

新增自治错误：`0x12 SESSION`、`0x13 COORDINATOR`、`0x14 RECOVERY_FAILED`、`0x15 COMMIT`。
完整错误码保持源码定义：

```text
0x00 NONE              0x01 BAD_CRC
0x02 BAD_LENGTH        0x03 BAD_ADDRESS
0x04 BAD_STATE         0x05 FLASH_ERASE
0x06 FLASH_WRITE       0x07 CONFIG
0x08 APP_INVALID       0x09 SIZE
0x0A GUARD_PROTECTED   0x0B SEQUENCE
0x0C CRC_MISMATCH      0x0D PROVIDER_SOURCE
0x0E BUSY              0x0F PROTECTED_REGION
0x10 ABORTED           0x11 RX_OVERFLOW
0x12 SESSION           0x13 COORDINATOR
0x14 RECOVERY_FAILED   0x15 COMMIT
```

## 20. Legacy 单节点完整流程

```text
ERASE
↓ READY
WRITE(size)
↓ DATA Seq0..N
WRITE_END
↓ Missing=0
VERIFY(expected_crc32)
↓ READY/app_valid=1
JUMP_APP
```

Legacy 模式不发送 `SESSION_BEGIN`，DATA Byte4~5 应为 `0x0000`。

该流程已经用于 CANPro Classic fallback 实测。
## 21. 完整自治升级流程

Host 建议顺序：

```text
1. SESSION_BEGIN(session, flags=0x07)
2. SESSION_CRC32(new_app_crc32)
3. SET_GUARD(guard_node)          可选，但启用Rollback时需要
4. ERASE                          广播
5. WRITE(new_app_size)            广播
6. DATA Seq0..N                   广播一次
7. WRITE_END                      广播
```

从第 7 步开始，不再要求专用中控保存完整 BIN：

```text
各普通节点上报 Missing
↓
最低 Node ID 时隙 Claim Coordinator
↓
Coordinator 合并缺失集合
↓
按 Sequence 选择最低 ID 合法 Provider
↓
广播选择性修复，最多3轮
↓
分布式 VERIFY
↓
成功：更新 Guard → VERIFY → Commit
失败：Guard Rollback → VERIFY → Commit
```

最终只有 Commit 成功的节点设置 `app_valid=1` 并跳 APP。
## 22. `bin_to_boot_frames.py`

Legacy Classic：

```bash
python bin_to_boot_frames.py app_boot.bin --mode classic --target 1
```

完整自治 Classic：

```bash
python bin_to_boot_frames.py app_boot.bin \
  --mode classic \
  --target 0xFF \
  --session 0x1234 \
  --session-flags 0x07 \
  --guard 8
```

原生 CAN FD：把 `--mode classic` 改为 `--mode fd`。

脚本会打印：Firmware size、Packet count、APP CRC32、SESSION_BEGIN、SESSION_CRC32、SET_GUARD、ERASE、WRITE、WRITE_END 和 Legacy VERIFY。

Classic 输出为 CANPro SendList；FD 输出为一行一个 64B DATA 的文本列表。

脚本读取 BIN 的真实文件长度，不应根据 map 地址人工猜测 `firmware_size`。
## 23. Release 编译

必须使用 Release preset：

```bash
cmake --build --preset Release --clean-first
```

当前 V1.2 最近一次 clean build：

```text
RAM   7624 B / 32 KiB   23.27%
FLASH 19324 B / 20 KiB  94.36%
0 warning / 0 error
```

Bootloader 只剩约 2 KiB Flash 余量，后续继续增加功能前应优先检查体积。

## 24. 推荐测试顺序

1. Legacy 单节点 ERASE→WRITE→DATA→WRITE_END→VERIFY→JUMP；
2. Session=非0，但只开 bit0，验证 Coordinator/Provider；
3. 两节点人为丢不同 Sequence，验证联合恢复；
4. 多节点同时缺同一 Sequence，确认 Provider 只广播一次；
5. 断开最低 ID 节点，确认后续 ID Claim；
6. 开启 bit2，验证全部节点 VERIFY 后才 Commit；
7. 设置 Guard，故意制造新版本 Repair/VERIFY 失败，验证自动 Rollback；
8. Guard 更新阶段测试 FULL_STREAM + Missing Repair；
9. 最后做断电/复位边界测试。
## 25. 当前实现边界

- Bootloader 自升级：未实现；
- Guard：由 Host 通过 `SET_GUARD` 指定，不是当前源码自动选最大 ID；
- Coordinator：首次广播 `WRITE_END` 后按 Node ID Claim，不是固定主节点；
- Provider：按当前节点状态确定性选择，不要求 Coordinator 持有完整固件；
- Session ID：用于隔离升级事务，不等同于固件版本；
- Guard Rollback：保护普通节点新版本 Repair/VERIFY 阶段；Guard 开始更新后旧镜像会被擦除；
- Commit：采用 Prepare/ACK + 多次 COMMIT_EXECUTE 的轻量实现，不包含掉电日志或重新选主后的事务恢复；
- 当前代码面向固定最多 8 节点，成员集合使用 `uint8_t bitmap`；
- 当前 Classic fallback 主要用于现有 CANPro 测试，正式数据面仍推荐 CAN FD。

## 26. APP 启动条件

Bootloader 只允许有效 APP 启动：Config/Metadata 有效、`app_valid=1`、size 合法、MSP 合法、Reset_Handler 位于 APP 区且为 Thumb 地址、整 APP CRC32 匹配。

APP 偏移工程必须保证：

```text
FLASH ORIGIN = 0x08005000
SCB->VTOR    = 0x08005000
```

`.bin` 应直接由链接在 `0x08005000` 的 APP ELF 导出，不要人工在前面填 `0x5000` 个字节。

### 26.1 Jump 前的时钟/外设清理

Bootloader 与 APP 可以采用不同 PLL 参数。当前实机中 Bootloader 使用 170 MHz，`Observer_Motor` APP 使用 168 MHz。若 Bootloader 仍以 PLL 作为 SYSCLK 时直接进入 APP，APP 再调用 `HAL_RCC_OscConfig()` 修改 PLL 参数，STM32G4 HAL 会返回 `HAL_ERROR`，APP 会卡在 `Error_Handler()`。

当前 `Boot_RuntimeJumpToApp()` 在真正设置 APP MSP 并执行 Reset_Handler 前执行：

```c
HAL_DeInit();
HAL_RCC_DeInit();
```

随后关闭 SysTick、清 NVIC enable/pending、清 PendSV/SysTick pending，并把 VTOR 指向 `0x08005000`。这样 APP 从接近上电复位的 HSI/reset-like 状态重新执行 `HAL_Init()` / `SystemClock_Config()`。

本修复已经通过单节点 CANPro 下载 + VERIFY + JUMP 实机验证。若以后 APP 与 Bootloader 使用完全相同的时钟参数，也仍建议保留该清理逻辑，以避免外设/中断残留跨镜像传播。

---

本文档与 `README.md`、`bootloader.h`、`bootloader.c`、`boot_port_can_stm32g4.c` 和 `bin_to_boot_frames.py` 对齐到 V1.2。
## 27. 自治模式的控制面前提与总线流量

自治恢复主要解决首次 DATA 广播后的 Sequence 缺失，不把关键控制命令丢失当作普通数据缺包。因此 Host 在发送整份 BIN 前，应确认目标节点已经进入同一 Session，并成功完成 ERASE/WRITE 会话建立。

`SESSION_BEGIN` 会主动清理上一事务残留的异步任务、写状态、Bitmap 和 RAM Guard 角色；新 Session 需要重新 `SET_GUARD`。

广播 `WRITE_END` 后：

- 每个普通节点仍立即返回一次 WRITE_END 状态；
- 自治模式的详细 Missing Report 只走 Peer Control，不再同时通过 `0x50x` 重复流式发送；
- Peer 节点先发送 `MISSING_COUNT`，再等待 20 ms 才发送 `MISSING_ITEM`；
- 这样可以先让 8 个节点完成在线/Count 声明，再进入可能较长的缺包明细传输。

该 20 ms 保护窗口由 `BOOT_PEER_MISSING_ITEM_DELAY_MS` 定义，可根据最终 CAN nominal bitrate 和节点数量再做实测调整。
