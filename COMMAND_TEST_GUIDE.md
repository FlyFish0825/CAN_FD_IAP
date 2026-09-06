# STM32G431 Bootloader V1.3 命令测试手册

本文档用于 **CANPro / 标准 CAN / CAN FD** 实机调试。所有示例均与当前 `bootloader.c` 实现对应，不把“协议设想”当作实际返回。V1.3 与 V1.2 的命令和帧格式兼容，主要变化是 Flash 体积及底层启动/FDCAN 中断实现。

## 1. 固定测试上下文

除特别说明外，后续示例使用：

```text
测试节点          Node1
Host CONTROL ID   0x000
Node1 RESPONSE ID 0x501
Guard             Node8
Session           0x1234
Session Flags     0x07
APP size          56120 B = 0x0000DB38
Packet count      1003 = 0x03EB
APP CRC32         0xFE565BD9
Coordinator       Node1
普通成员 bitmap   0x7F = Node1~Node7
```

Host CONTROL：`[Target, Cmd, Seq/Arg, Param0..3, CRC8]`。Node RESPONSE：`[Node, Cmd, Status, Data0..3, CRC8]`。

Peer Control：`[Target, Cmd, Source, SessionLo, SessionHi, ValueLo, ValueHi, CRC8]`，CAN ID=`0x600+Source`。

CRC8：Poly=`0x07`，Init=`0x00`，计算 Byte0~Byte6。
## 2. 通用响应规则

正常 Host 响应 CAN ID=`0x500+NodeID`。`Boot_SendError()` 的错误响应格式固定为：

```text
Byte0 NodeID
Byte1 原 Command
Byte2 0x05 = ERROR
Byte3 ErrorCode
Byte4~6 0
Byte7 CRC8
```

例如 VERIFY 在错误状态执行，`BAD_STATE=0x04`：

```text
0x501  01 13 05 04 00 00 00 8F
```

注意：`ABORT` 是主动中止命令，本身返回 `ERROR` 状态，但 Data0 不放 `ABORTED=0x10`；应再用 `GET_STATUS` 查看 `last_error=0x10`。

Peer Control 默认**不使用 Host RESPONSE ACK**。某些命令有配对 Peer 返回，某些则靠后续 DATA 或状态通知确认，后文逐项注明。

## 3. A 类：基础查询 / Boot 控制

### 3.1 GET_VERSION `0x01`

用途：确认 Bootloader 在线和协议版本。无特殊前置条件。

```text
TX  ID=0x000  01 01 00 00 00 00 00 F6
RX  ID=0x501  01 01 04 01 03 00 00 D2
```

`Data0..3 = 1.3.0.0`。收到 `Status=0x04 READY` 即成功。
### 3.2 GET_DEVICE_ID `0x02`

用途：读取当前 MCU 的 `DBGMCU->IDCODE`。

```text
TX  ID=0x000  01 02 00 00 00 00 00 8D
RX  ID=0x501  01 02 04 ID0 ID1 ID2 ID3 CRC8
```

`ID0..ID3` 为运行时芯片 IDCODE 的 little-endian 字节，因此不同芯片/Revision 可能不同，最后 CRC8 也随之变化。`Status=READY` 即成功。

### 3.3 GET_INFO `0x03`

用途：快速查看 Node/HW/APP/Config 状态。

```text
TX  ID=0x000  01 03 00 00 00 00 00 A4
RX  示例      01 03 04 01 00 01 01 2F
```

响应 `Data0=NodeID`，`Data1=hardware_version`，`Data2=app_valid`，`Data3=config_valid`。上例表示 Node1、HW版本0、APP有效、Config有效。实际值以节点为准。

### 3.4 ENTER_BOOT `0x04`

用途：本次运行强制留在 Bootloader，不立即跳 APP。

```text
TX  ID=0x000  01 04 00 00 00 00 00 7B
RX  ID=0x501  01 04 04 00 00 00 00 F4
```

成功后 `g_boot_requested=1`，同时清除当前 `last_error`。它不会擦 APP。
### 3.5 ABORT `0x18`

用途：取消 READ/Missing/Provider 等异步任务并结束当前写会话。

```text
TX  ID=0x000  01 18 00 00 00 00 00 0E
RX  ID=0x501  01 18 05 00 00 00 00 E3
```

这里 `Status=0x05 ERROR` 是协议设计上的“已中止”状态，不代表 ABORT 命令本身失败。随后 `GET_STATUS` 应看到 `last_error=0x10 ABORTED`。

### 3.6 JUMP_APP `0x20`

前置：持久化 metadata 有效、`app_valid=1`、size/CRC/向量表全部合法。

```text
TX  ID=0x000  01 20 00 00 00 00 00 E4
RX  ID=0x501  01 20 04 00 00 00 00 6B
```

收到 READY 后 Bootloader 先 Flush TX，再恢复 reset-like RCC/外设状态并跳 `0x08005000`。成功后 Bootloader 不再响应 GET_VERSION。

若 APP 无效，可能收到 `ERROR + APP_INVALID(0x08)`；此时绝不能跳转。

### 3.7 RESET `0x21`

```text
TX  ID=0x000  01 21 00 00 00 00 00 CD
RX  ID=0x501  01 21 04 00 00 00 00 42
```

节点 Flush TX 后调用 `NVIC_SystemReset()`。ACK 后 CAN 通信会短暂中断，随后重新启动。
### 3.8 GET_STATUS `0x30`

```text
TX  ID=0x000  01 30 00 00 00 00 00 7A
RX  示例      01 30 04 04 00 64 00 0C
```

响应含义：`Byte2=current status`，`Data0=current status`，`Data1=last_error`，`Data2=progress(0~100)`，`Data3=reserved`。上例表示 READY、无错误、100%。

若节点当前处于 ERROR，例如 CRC mismatch，`Byte2` 和 `Data0` 都可能为 `0x05`，`Data1` 给出具体错误码。

## 4. B 类：Session / Guard 配置

### 4.1 SESSION_BEGIN `0x07`

完整自治模式建议广播，Session 必须非 0：

```text
TX  ID=0x000  FF 07 07 34 12 00 00 37
RX  Node1      01 07 04 34 12 07 00 61
```

`Flags=0x07` 表示 Peer Recovery + Guard Rollback + Coord Commit 全开。每个在线节点都应从自己的 `0x50x` 返回 READY。

可能失败：Session=0 → `ERROR + SESSION(0x12)`。成功时会清旧异步任务、旧 Bitmap、旧 RAM Guard 角色，因此之后必须重新 `SET_GUARD`。

### 4.2 SESSION_CRC32 `0x08`

前置：已经建立非 0 Session。

```text
TX  ID=0x000  FF 08 00 D9 5B 56 FE 2E
RX  Node1      01 08 04 D9 5B 56 FE 3A
```

`D9 5B 56 FE` = `0xFE565BD9` little-endian。无活动 Session 时返回 `ERROR + SESSION(0x12)`。
### 4.3 SET_GUARD `0x05`

示例指定 Node8 为 Guard，通常广播给所有节点：

```text
TX  ID=0x000  FF 05 08 00 00 00 00 D0
RX  Node1      01 05 04 08 00 00 00 6D
RX  Node8      08 05 07 08 00 00 00 FE
```

普通节点返回 `READY`，Guard 自己返回 `GUARD(0x07)`；`Data0=8`。Guard ID 不在 1~8 时返回 `ERROR + BAD_ADDRESS(0x03)`。

### 4.4 RELEASE_GUARD `0x06`

```text
TX  ID=0x000  FF 06 08 00 00 00 00 AB
RX  Node1      01 06 04 08 00 00 00 16
```

所有节点解除本次 RAM Guard 角色。若当前没有 Guard，或参数 ID 与当前 Guard 不一致，返回 `ERROR + BAD_STATE(0x04)`。

## 5. C 类：Flash 下载 / 读取 / Legacy 校验

### 5.1 ERASE `0x10`

Node1 单节点测试：

```text
TX  ID=0x000  01 10 00 00 00 00 00 41
RX1 ID=0x501  01 10 01 00 00 00 00 23
RX2 ID=0x501  01 10 04 00 00 00 00 CE
```

`RX1=ERASE` 表示已受理；`RX2=READY` 才表示真正擦除完成。必须等 RX2 后再发 WRITE。

Guard 节点会返回 `GUARD` 而不擦除；Flash/metadata 失败分别可能返回 `CONFIG(0x07)` 或 `FLASH_ERASE(0x05)`。
### 5.2 WRITE `0x11`

当前实测 APP：56120 B=`0xDB38`，region=APP(0)：

```text
TX  ID=0x000  01 11 00 38 DB 00 00 B2
RX  ID=0x501  01 11 02 00 EB 03 00 B1
```

响应 `Data0=region=0`，`Data1~2=0x03EB=1003 packets`。APP 未先 ERASE → `BAD_STATE`；size=0/超范围 → `SIZE`；region=Bootloader → `PROTECTED_REGION`。

### 5.3 WRITE_DATA `0x01`（64B 逻辑 DATA）

该命令不走 8B CONTROL，不逐包 ACK。Seq0 逻辑头：

```text
01 01 00 00 00 00 00 00 | 后续 56B firmware
```

Legacy Session=0；自治模式例如 Session `0x1234` 时头为：

```text
FF 01 00 00 34 12 00 00 | 后续 56B firmware
```

Classic CAN 把同一 64B 严格拆为 ID `0x100~0x107` 八帧。包写 Flash + read-back 成功后才置 Bitmap bit；无逐包 RESPONSE，应最终通过 WRITE_END/MISSING 判断。

### 5.4 READ `0x12`

读取 `0x08005000` 起始 8 Byte：

```text
TX  ID=0x000  01 12 08 00 50 00 08 16
RX1 ID=0x501  01 12 04 00 80 00 20 77
RX2 ID=0x501  01 12 04 71 D8 00 08 FE
```

当前 `Observer_boot.bin` 前 8B 正是 `00 80 00 20 71 D8 00 08`：MSP=`0x20008000`，Reset=`0x0800D871`。READ 每次 RESPONSE 最多 4B，因此 8B 会返回两帧。
READ 可能失败：length=0 → `BAD_LENGTH`；上一次 READ 尚未完成 → `BUSY`；地址/长度越过 Flash → `BAD_ADDRESS`。READ 命令本身不先回“已受理”，第一帧就是实际数据。

### 5.5 WRITE_END `0x14`

1003 包全部收到时：

```text
TX  ID=0x000  01 14 00 00 00 00 00 E5
RX  ID=0x501  01 14 03 00 00 00 00 43
```

`Status=VERIFY(0x03)`、Missing=0，随后 Legacy 模式还会异步收到：

```text
RX  ID=0x501  01 15 04 00 00 EB 03 9E
```

若缺 2 包，WRITE_END 会返回 `REPAIR(0x06)`，`Data0~1=2`，随后继续 MISSING_COUNT/MISSING_ITEM。自治广播模式下详细 Missing 改走 Peer Control。

### 5.6 VERIFY `0x13`

当前 APP Expected CRC32=`0xFE565BD9`：

```text
TX  ID=0x000  01 13 00 D9 5B 56 FE 1F
RX  ID=0x501  01 13 04 D9 5B 56 FE 90
```

成功条件：写会话为 APP、Missing=0、CRC 一致、MSP/Reset vector 合法、metadata 保存成功。成功后 `app_valid=1`。

若 CRC 不一致，`Status=ERROR` 且 `Data0~3` 返回**实际 Flash CRC32**；若处于 `COORD_COMMIT` Session，则 Legacy VERIFY 被拒绝，返回 `ERROR + BAD_STATE(0x04)`。

## 6. D 类：Legacy 缺包 / 人工 Provider
### 6.1 MISSING_COUNT `0x15`

它通常不是 Host 主动发送，而是 `WRITE_END` 后节点异步上报。缺包为 0：

```text
RX  ID=0x501  01 15 04 00 00 EB 03 9E
```

缺 2 包：

```text
RX  ID=0x501  01 15 06 02 00 EB 03 76
```

`Data0~1=missing_count`，`Data2~3=total_packets`。`Status=READY` 表示 0 缺包，`REPAIR` 表示存在缺包。

### 6.2 MISSING_ITEM `0x16`

假设缺 Seq5 和 Seq100，则在 MISSING_COUNT 后依次可能收到：

```text
RX  ID=0x501  01 16 06 05 00 00 00 B2
RX  ID=0x501  01 16 06 64 00 01 00 E4
```

`Data0~1=missing sequence`，`Data2~3=item_index`，index 从 0 递增。收到的 MISSING_ITEM 数应与 MISSING_COUNT 一致。

### 6.3 PROVIDER_GRANT `0x17`

示例：Host 让 **Node1** 向 **Node2** 提供 Seq5，共 1 包。命令必须单播给 Provider：

```text
TX  ID=0x000  01 17 02 05 00 01 00 01
RX1 ID=0x501  01 17 02 02 05 00 00 B6
```

`RX1 Status=WRITE` 表示 Node1 接受 Provider 任务；随后 Node1 发送 Target=2, Seq=5 的 DATA。物理 DATA Flush 完成后：

```text
RX2 ID=0x501  01 17 04 02 00 00 00 3D
```

广播 `PROVIDER_GRANT` 会被直接忽略且**不回复**，防止多个节点同时发送。Seq 不存在时可能返回 `PROVIDER_SOURCE(0x0D)`。
## 7. E 类：自治 Missing / Coordinator / Provider / Repair

Peer 示例统一使用 Session=`0x1234`。CAN ID 必须与 Source 一致：Node1=`0x601`，Node2=`0x602`，…，Node8=`0x608`。

### 7.1 Peer MISSING_COUNT `0x15`

假设 Node2 缺 2 包：

```text
RX/模拟  ID=0x602  FF 15 02 34 12 02 00 3C
```

`Target=FF`，`Source=2`，`Value=2`。该帧既是 Missing 数量，也是在线/成员发现信息。**无 ACK**。

### 7.2 Peer MISSING_ITEM `0x16`

Node2 报告缺 Seq5：

```text
RX/模拟  ID=0x602  FF 16 02 34 12 05 00 2C
```

`Value=5`。实际实现会先发 MISSING_COUNT，然后等待约 20ms 再发明细。**无 ACK**。

### 7.3 COORDINATOR_CLAIM `0x19`

Node1 声明自己为 Coordinator，成员 bitmap=`0x7F`：

```text
RX/模拟  ID=0x601  FF 19 01 34 12 7F 00 3A
```

其它节点接受后更新 `coordinator_id=1` 和成员 bitmap，**不会回 ACK**。较低 ID 节点在自己的选举时隙尚未结束时可忽略更高 ID 的抢先 Claim。

成功判断：后续 `PROVIDER_ASSIGN/VERIFY_REQUEST/...` 应由同一 Coordinator 的 `0x601` 发出。当前实现尚无“已成功 Claim 后 Coordinator 死亡”的 heartbeat 重新选主。
### 7.4 PROVIDER_ASSIGN `0x1A`

Coordinator Node1 指派 Node2 提供 Seq5：

```text
TX/RX  ID=0x601  02 1A 01 34 12 05 00 9C
```

Node2 仅在 `Target=2`、`Source=当前 Coordinator`、本地确实拥有 Seq5、且当前没有 Provider 任务时接受。

**没有 ACK。** 成功后应观察 Node2 发送 Target=`0xFF`、Seq5 的 DATA，最后再看到 Node2→Coordinator 的 `PROVIDER_DONE`。

### 7.5 PROVIDER_DONE `0x1B`

Node2 完成 Seq5，并且 DATA 已 Flush：

```text
RX  ID=0x602  01 1B 02 34 12 05 00 75
```

`Target=1`，`Source=2`，`Value=5`。Coordinator 收到后继续扫描下一缺失 Sequence。**无二次 ACK**。

若 Provider 超过 `BOOT_PROVIDER_TIMEOUT_MS=10000ms` 未 DONE，Coordinator 会临时排除该 Provider，并尝试同 Seq 的下一个合法 Provider。

### 7.6 REPAIR_ROUND_END `0x1C`

Coordinator 宣布第 1 轮 Repair 结束，请成员重新上报 Missing：

```text
RX  ID=0x601  FF 1C 01 34 12 01 00 C3
```

`Value=repair_round=1`。成员更新本地 repair round，并重新执行 Peer Missing Report。**无 ACK**；后续 `MISSING_COUNT/MISSING_ITEM` 就是响应效果。

### 7.7 RECOVERY_READY `0x1D`

Coordinator 广播自治恢复完成：

```text
RX  ID=0x601  FF 1D 01 34 12 01 00 EA
```

成员记录 repair round，并将自治恢复置为 terminal。**无 ACK**。若本地 Missing=0，状态会进入 VERIFY 方向，后续通常进入分布式 VERIFY/Commit。
### 7.8 RECOVERY_FAILED `0x1E`

示例：第 3 轮后仍失败：

```text
RX  ID=0x601  FF 1E 01 34 12 03 00 BB
```

成员进入 `RECOVERY_PHASE_FAILED`，`Status=ERROR`，`last_error=RECOVERY_FAILED(0x14)`，保持 Bootloader。**无 ACK**。

## 8. F 类：分布式 VERIFY / Guard / Rollback

### 8.1 VERIFY_REQUEST `0x22`

Coordinator Node1 请求所有普通成员执行 NEW_VERIFY，context=`0x02`：

```text
RX  ID=0x601  FF 22 01 34 12 02 00 E0
```

成员收到后检查 Missing=0、CRC32、MSP/Reset vector，并保存“prepared image”，此时 `app_valid=0`。

该命令不回 Host ACK，而是每个成员通过 `VERIFY_RESULT` 回 Coordinator。

### 8.2 VERIFY_RESULT `0x23`

Node2 对 NEW_VERIFY 返回成功。Value 高字节=context=`0x02`，低字节=ok=`0x01`：

```text
RX  ID=0x602  01 23 02 34 12 01 02 C5
```

`Target=Node1`。失败时低字节为 `0x00`；Coordinator 收到任一失败结果会进入阶段失败处理（允许时转 Guard Rollback）。**无二次 ACK**。

### 8.3 GUARD_UPDATE_BEGIN `0x24`

普通成员的新 APP 全部验证成功后，Coordinator 通知 Node8 Guard 开始更新：

```text
RX  ID=0x601  08 24 01 34 12 00 00 92
```

Guard 只有在自身角色、Session CRC、image size 都有效时才接受。接受后解除 Guard 保护并擦除 APP。
### 8.4 GUARD_UPDATE_READY `0x25`

Node8 Guard 擦除/准备成功后返回：

```text
RX  ID=0x608  01 25 08 34 12 01 00 E0
```

`Target=Node1`，`Value=1` 表示成功，`Value=0` 表示失败。Coordinator 成功收到后选择已验证的新版本 Provider，并触发 FULL_STREAM。**无二次 ACK**。

### 8.5 ROLLBACK_REQUEST `0x26`

新版本阶段失败、且允许 Guard Rollback 时，Coordinator 请求 Node8 提供旧 APP metadata：

```text
RX  ID=0x601  08 26 01 34 12 00 00 C0
```

Guard 若仍是有效 Guard 且旧 APP 通过持久化校验，则不会回单个 ACK，而是依次广播四个 metadata 帧：`SIZE_LO / SIZE_HI / CRC_LO / CRC_HI`。

### 8.6 ROLLBACK_SIZE_LO `0x27`

以下四帧用示例旧镜像 `size=0x0000C000`、`CRC32=0x12345678`：

```text
RX  ID=0x608  FF 27 08 34 12 00 C0 72
```

Value=`0xC000`，为旧 APP size 低 16bit。**无 ACK**。

### 8.7 ROLLBACK_SIZE_HI `0x28`

```text
RX  ID=0x608  FF 28 08 34 12 00 00 AC
```

Value=`0x0000`，为旧 APP size 高 16bit。**无 ACK**。

### 8.8 ROLLBACK_CRC_LO `0x29`

```text
RX  ID=0x608  FF 29 08 34 12 78 56 2A
```

Value=`0x5678`，为旧 APP CRC32 低 16bit。**无 ACK**。
### 8.9 ROLLBACK_CRC_HI `0x2A`

```text
RX  ID=0x608  FF 2A 08 34 12 34 12 2D
```

Value=`0x1234`，为旧 APP CRC32 高 16bit。四个 metadata frame 全部收到后，Coordinator 才认为旧镜像元数据完整。**无 ACK**。

### 8.10 ROLLBACK_BEGIN `0x2B`

Coordinator 让普通成员擦除当前 APP 并准备接收旧镜像：

```text
RX  ID=0x601  FF 2B 01 34 12 00 00 AC
```

仅 primary members 且已经收齐 4 个 rollback metadata 的节点处理。每个成员处理后通过 `ROLLBACK_PREPARED` 返回结果。

### 8.11 ROLLBACK_PREPARED `0x2C`

Node2 准备成功：

```text
RX  ID=0x602  01 2C 02 34 12 01 00 5B
```

`Value=1` 成功，`Value=0` 失败。Coordinator 收到任一失败会进入最终 recovery failure。**无二次 ACK**。

### 8.12 FULL_STREAM `0x2D`

示例：Coordinator Node1 指派 Node2 将完整镜像发送给 Guard Node8：

```text
RX  ID=0x601  02 2D 01 34 12 08 00 0F
```

`Target=2` 是 Provider，`Value低8bit=8` 是 DATA target。Provider 成功接受后**不回复 ACK**，而是从 Seq0 开始连续发送整镜像 DATA；全部 Flush 后发送 `PROVIDER_DONE`，其中 `Value=0xFFFF` 表示 full stream。

若 Provider 没有有效镜像或正在执行其它 Provider 任务，该命令可被静默忽略；Coordinator 依靠 provider timeout 判断失败。

## 9. G 类：Prepare / Commit
### 9.1 COMMIT_PREPARE `0x2E`

Coordinator Node1 要求普通成员 bitmap=`0x7F` 进入 Prepare：

```text
RX  ID=0x601  FF 2E 01 34 12 7F 00 40
```

成员先检查自己是否属于 bitmap，再执行本地 `Boot_ArmCommitLocal()`。该命令的配对返回是 `COMMIT_ACK`。

### 9.2 COMMIT_ACK `0x2F`

Node2 arm 成功：

```text
RX  ID=0x602  01 2F 02 34 12 01 00 20
```

`Value=1` 表示本地镜像具备提交条件；`Value=0` 表示失败。Coordinator 必须收齐 expected bitmap 中所有节点 ACK 才能进入 Execute。**无二次 ACK**。

### 9.3 COMMIT_EXECUTE `0x31`

Coordinator 收齐 ACK 后广播：

```text
RX  ID=0x601  FF 31 01 34 12 7F 00 4E
```

当前实现重复发送 3 次。成员只有在 `Source=当前 Coordinator`、bitmap 与本地 expected bitmap 一致、且自己属于 bitmap 时处理。

首次有效 Execute 会持久化 `app_valid=1`，随后等待 `BOOT_COMMIT_DELAY_MS=50ms`、Flush TX 并跳 APP。**没有 ACK**；成功判断是节点随后进入 APP，Bootloader 不再响应。

若本地 commit 写 metadata 失败，则节点留在 Bootloader，`Status=ERROR`，`last_error=COMMIT(0x15)`。

## 10. 单节点 CANPro 推荐实测顺序

当前已经实测通过的 Legacy 路径：

```text
GET_VERSION
ERASE                  等 ERASE + READY 两帧
WRITE 56120B           确认返回 1003 packets
DATA Seq0..1002        Classic = 1003*8 = 8024帧
WRITE_END              确认 Missing=0
VERIFY 0xFE565BD9      确认实际 CRC 相同
READ 0x08005000, 8B    可选，核对 MSP/Reset vector
JUMP_APP               READY 后观察 APP 行为
```

这条路径已验证标准 CAN / CANPro 下载、Flash read-back、CRC32、metadata、Bootloader→APP 时钟清理与跳转。

## 11. V1.3 体积优化专项回归

V1.3 改动了启动框架、Bootloader 向量表和 FDCAN FIFO0 中断入口，但没有改协议状态机。烧录 V1.3 后至少执行：

1. 上电后发送 GET_VERSION，确认返回 `1.3.0.0`；
2. 连续发送多帧 CONTROL，确认 FIFO0 接收和 RESPONSE 正常；
3. 完整执行一次 ERASE→WRITE→DATA→WRITE_END→VERIFY；
4. 执行 JUMP_APP，确认 APP 的 MSP、VTOR、时钟和 SysTick 正常；
5. 复位后确认有效 APP 可以自动启动；
6. 人为制造一个 DATA 缺包，确认 Missing/补包流程未受影响。

当前 Bootloader 只启用 FDCAN1 IT0（外部 IRQ21），向量表也只保留到 IRQ21。以后若在 Bootloader 中启用 IRQ22 或更高编号中断，必须先恢复启动文件中的对应向量表项。APP 拥有自己位于 `0x08005000` 的向量表，不受此限制。
