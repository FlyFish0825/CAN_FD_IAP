# Codex 任务书：最小改动实现 APP Trial Jump / 返回 Bootloader 验证

## 1. 目标

在现有 CAN_FD_IAP 基础上增加一个“APP 试运行并验证能否返回 Bootloader”的最小机制。

核心判定标准只有一个：

> 新 APP 在 Bootloader 跳转后，必须能够启动到 CAN 通信层，收到 ENTER_BOOT 指令，并通过软件复位重新进入 Bootloader。

如果能回来，认为 APP 符合系统要求；如果不能回来，认为 APP 异常，记录失败原因，并禁止以后自动再次跳入该 APP。

该机制主要防止：

- APP CRC/Vector 都正确，但 VTOR、时钟、中断、初始化逻辑等存在运行期错误；
- APP 可以被完整下载和 VERIFY，但实际启动后死机；
- 坏 APP 启动后无法再进入 Bootloader，导致节点“软变砖”。

## 2. 修改原则：必须严格遵守

这是一个“小改动任务”，不要重构现有 Bootloader。

禁止修改或重写以下已有主流程：

- DATA 下载、Bitmap、Missing Report；
- Peer Repair；
- Coordinator election；- Provider / FULL_STREAM；
- Guard / Rollback；
- Distributed Verify；
- COMMIT_PREPARE / COMMIT_ACK / COMMIT_EXECUTE；
- 现有 CAN ID 与控制帧基本格式；
- `Boot_RuntimeJumpToApp()` 里已经验证过的 `HAL_DeInit()`、`HAL_RCC_DeInit()`、SysTick/NVIC 清理、VTOR/MSP 跳转逻辑。

不要为了实现 Trial 引入大状态机、双 Bank、复杂健康监控或新的长期 APP Watchdog 逻辑。

必须尽量复用现有机制、现有字段和现有命令。

## 3. 当前已有关键实现

Bootloader 工程：

`F:/file/BaiduSyncdisk/Project/CAN_FD_IAP/CAN_FD_IAP`

APP 工程：

`F:/file/BaiduSyncdisk/Project/Observer_Motor`

Bootloader 当前关键函数位于 `Core/Src/bootloader/bootloader.c`：

- `Boot_RuntimeJumpToApp()`
- `Boot_HandleJumpApp()`
- `Boot_CoreInit()`
- `Boot_ShouldJumpApp()`
- `Boot_JumpApp()`
- `Boot_HandleVerify()`
现有 Boot Request 机制必须复用，不要再造第二套：

```c
#define BOOT_REQUEST_MAGIC 0x544F4F42UL /* BOOT */
```

现有函数：

```c
void Boot_RequestBootloader(void)
{
    Boot_RuntimeEnableBackupWrite();
    TAMP->BKP0R = BOOT_REQUEST_MAGIC;
    __DSB();
}
```

Bootloader 启动时 `Boot_RuntimeConsumeBootRequest()` 会读取并清除 `TAMP->BKP0R`。

APP 返回 Bootloader 时应使用同一个 BKP0R / BOOT_REQUEST_MAGIC 语义。

## 4. 单节点 Trial 目标流程

现有下载/校验仍保持：

```text
ERASE -> WRITE -> DATA -> WRITE_END -> Missing=0 -> VERIFY
```

VERIFY 仍负责 CRC32 和 Vector 合法性判断，但 Trial APP 在第一次试运行前不能被视为长期可信 APP。
### 4.1 Trial 触发方式

不要新增新的 CAN Command ID。复用现有 `BOOT_CMD_JUMP_APP (0x20)`，只给它增加一个很小的模式参数：

```text
JUMP_APP Byte2/seq = 0x00 : 原有普通 Jump，行为必须保持兼容
JUMP_APP Byte2/seq = 0x01 : Trial Jump
其它值                  : BAD_STATE 或 BAD_ADDRESS
```

因此需要把当前只接收 `cmd` 的 `Boot_HandleJumpApp()` 局部改为能看到当前 `Boot_ControlFrame_t`，不要改变其它命令格式。

### 4.2 Trial 状态不要改 Boot_Config_t 布局

优先使用 Backup Register 保存短期 Trial 状态，不增加新的 Flash metadata 结构版本。

建议：

```text
TAMP->BKP0R : 已有 BOOT_REQUEST_MAGIC，不改
TAMP->BKP1R : 新增 BOOT_TRIAL_MAGIC，仅表示“刚刚进行过一次 Trial Jump”
```

例如定义独立的 `BOOT_TRIAL_MAGIC`，值只需与 BOOT_REQUEST_MAGIC 不同。

Trial 状态是一次性状态：Bootloader 每次启动读取后应清除 BKP1R，避免重复判定。

不要使用 BKP0R 同时承担 Trial 和 Boot Request 两种语义。

### 4.3 Trial Jump 前的处理

第一版 Trial 只用于 Host 控制的 Canary 预检，推荐在正式多节点 Autonomous Session 之前执行。

若当前 `g_session_active != 0` 且启用了 `BOOT_SESSION_FLAG_COORD_COMMIT`，`JUMP_APP seq=1` 应直接拒绝并返回 `BOOT_ERR_BAD_STATE`，不要尝试跨复位保存/恢复 Coordinator Session。

正常第一版来源是 Legacy VERIFY 已通过、metadata 当前 `app_valid=1` 的 Canary APP。

Trial 模式必须验证：

```text
app_size 合法
app_crc32 与 Flash 实际 CRC 一致
MSP / Reset_Handler 合法
```

第一版先按现有 Legacy `app_valid=1` 镜像完成完整校验，再执行 Trial invalidation；不要为了支持任意 `app_valid=0` 镜像额外扩展状态机。

真正跳转前必须保证持久化 metadata 的 `app_valid=0`：用现有 metadata 保存接口把相同 size/crc 改成 `app_valid=0`，再跳 APP。

这样即使 APP 死机后被复位，下一次 Bootloader 启动也不会走 `Boot_ShouldJumpApp()` 再次自动进入坏 APP。

然后：

```text
写 BKP1R = BOOT_TRIAL_MAGIC
启动 IWDG Trial timeout
Flush CAN TX
调用现有 Boot_RuntimeJumpToApp()
```

Trial timeout 建议约 3 秒（可在 2~5 秒范围内选择一个当前 STM32G431/LSI 可稳定实现的值），并写清实际配置值。

IWDG 只由 Bootloader 在 Trial Jump 前启动。APP 不增加任何喂狗逻辑。

不要修改 `Boot_RuntimeJumpToApp()` 已验证的时钟、NVIC、VTOR、MSP 清理顺序。

## 5. APP 侧只做最小改动
APP 工程只增加“收到 ENTER_BOOT 后返回 Bootloader”的能力，不增加 Trial 状态机和 Watchdog 任务。

必须在 APP 现有 CAN 接收/协议分发路径里做最小插入，不要再创建第二套 CAN 接收架构。

APP 需要接受 Boot 控制 ID `0x000` 的 `ENTER_BOOT (0x04)`，至少支持单播到本 Node。若现有 APP 已有 Node ID/CRC8 工具，应直接复用。

收到合法 ENTER_BOOT 后：

```text
1. 不需要先回复“支持 Bootloader”的 ACK；
2. 写 TAMP->BKP0R = BOOT_REQUEST_MAGIC；
3. __DSB();
4. NVIC_SystemReset();
```

真正重新进入 Bootloader 本身就是成功响应。

APP 侧可以写一个极小的 `App_RequestBootloader()` helper，逻辑与 Bootloader 的 `Boot_RequestBootloader()` 一致；不要把整个 Bootloader 模块链接进 APP。

APP 必须使用同一个：

```c
#define BOOT_REQUEST_MAGIC 0x544F4F42UL /* BOOT */
```

需要时仅做 Backup Domain 写权限初始化，例如 PWR clock / backup access；不要因此重构 APP 初始化。

APP **不要初始化或喂 IWDG**。正常 APP 应在 Trial timeout 之前收到 ENTER_BOOT 并主动 `NVIC_SystemReset()`。

## 6. Bootloader 重新启动后的 Trial 判定
Bootloader 启动时，在现有 `Boot_CoreInit()` 附近增加很小的 Trial 结果处理，不要重构初始化流程。

读取/消费：

```text
BKP1R == BOOT_TRIAL_MAGIC ? trial_pending = 1 : 0
BKP0R == BOOT_REQUEST_MAGIC ? boot_requested = 1 : 0
```

### 成功条件

```text
trial_pending == 1
AND
boot_requested == 1
```

这表示 APP 确实启动、收到 ENTER_BOOT、写入 Boot Request，并主动软件复位回来。

成功时：

- 再次校验 metadata 中的 `app_size/app_crc32` 与 Flash image；
- 使用现有 metadata 保存函数把 `app_valid` 恢复为 1；
- `g_app_valid = 1`；
- `g_last_error = BOOT_ERR_NONE`；
- 保持当前这次启动停留在 Bootloader，不要马上再次自动 Jump；
- Host 可用 `GET_INFO` 看到 `app_valid=1`，然后再发普通 `JUMP_APP seq=0` 正式启动。

因为 `Boot_RuntimeConsumeBootRequest()` 本来就会令 `g_boot_requested=1`，应尽量利用这一点阻止本次启动自动跳 APP。

### 失败条件
如果：

```text
trial_pending == 1
AND
boot_requested == 0
```

则认为 Trial 失败。典型情况就是 APP 没有成功收到/执行 ENTER_BOOT，最终由 IWDG 复位回来。

失败时：

- 保持 metadata `app_valid=0`；
- `g_app_valid=0`；
- 强制本次启动停留在 Bootloader；
- 增加一个最小错误码，例如 `BOOT_ERR_APP_NO_RETURN` 或 `BOOT_ERR_APP_TRIAL_TIMEOUT`；
- `GET_STATUS` 能看到该错误；
- 不要自动再次 Jump 这个 APP。

如容易实现，可读取 RCC reset flag 区分 IWDG reset；但不要为了细分 reset cause 引入大模块。第一版只要能稳定区分“主动 ENTER_BOOT 返回”和“没有 Boot Request 的异常返回”即可。

已有 CRC mismatch、APP_INVALID 等错误继续使用原有错误码，不要重复定义。

处理 Trial 结果后清除 BKP1R Trial Magic，避免下次启动重复判定。

注意 `Boot_CoreInit()` 当前最后会把 `g_last_error` 清成 NONE；新增 Trial 错误的赋值位置不能被后续初始化覆盖。

## 7. IWDG 实现要求

当前任务只需要一个 Trial timeout。优先采用代码体积最小、改动最少的实现。
如果项目当前没有生成 IWDG HAL 模块，不要为了 Trial 大范围修改 `.ioc` 和大量自动生成文件。

可以：

- 优先复用现有 IWDG HAL（如果项目已经包含）；
- 否则写一个很小的 STM32G431 寄存器级 helper，只负责配置一次性 Trial timeout；
- 不要增加周期任务、喂狗线程或定时器回调。

Trial watchdog 的语义只有：

```text
Bootloader 开狗 -> Jump APP -> APP若正常就很快软复位回来
                           -> APP若异常则 IWDG 超时复位回来
```

请确认软件复位/IWDG 复位后 Bootloader 能正常重新初始化 FDCAN，并且 Backup Register 仍可用于本次 Trial 判定。

## 8. 多节点策略：只做一个 Canary，不逐节点 Trial

不要实现 Node1、Node2、Node3...逐个 Trial 的状态机。

最终设计是：

```text
先选 1 个非 Guard 节点作为 Canary
-> 对 Canary 做一次 Trial Jump / ENTER_BOOT 返回验证
-> Canary PASS：认为该 APP 软件满足本系统启动和回 Bootloader 要求
-> 然后其它节点继续走项目现有的多节点升级、Guard、Commit、统一 Jump 流程
-> Canary FAIL：新 APP 判失败，不继续正式切换；保留/使用旧版本 Guard 和现有 Rollback 能力
```

节点将来即使扩展到 32 个，也仍然只试一个 Canary，不要按节点数量线性增加 Trial 时间。

### 非常重要：本任务不要把 Canary Trial 塞进现有 Coordinator 状态机
原因：Canary APP 会执行 `NVIC_SystemReset()`，如果它正处于活跃的 Autonomous Session 中，会破坏 RAM 中的 Session/Coordinator/Repair 状态；为了“最小改动”，第一版不要处理 Session 跨复位恢复。

第一版由 Host 编排 Canary 预检，推荐放在正式多节点自治升级之前：

```text
Host 单独给 Canary 下载/VERIFY 新 APP
-> Trial Jump
-> Host 向 Canary APP 发 ENTER_BOOT
-> Canary 返回 Bootloader
-> PASS 后再开始原有 8/32 节点广播升级流程
```

这样现有 Peer Recovery / Guard / Commit 代码完全不需要改。

如果未来需要“所有节点已经完成 Distributed Verify 后再挑一个 Canary”，那需要单独设计 Session 恢复/重加入机制，不属于本任务。

## 9. Host/CANPro 测试顺序

单节点 Trial 建议测试：

```text
1. ERASE
2. WRITE
3. DATA
4. WRITE_END，确认 Missing=0
5. VERIFY，确认 CRC/Vector OK
6. JUMP_APP，Byte2/seq=1（Trial）
7. 等 APP CAN 初始化完成，例如 300~500 ms
8. 连续发送几次 ENTER_BOOT 到该 Node
9. 等待该节点 Bootloader 的 0x50x 响应重新出现
10. GET_INFO / GET_STATUS 检查 Trial 结果
```

正常 PASS：`app_valid=1`、last_error=NONE，节点停留 Bootloader；Host 再发普通 `JUMP_APP seq=0` 正式进入 APP。
异常 FAIL 测试至少做一种：临时让 APP 不处理 ENTER_BOOT，或让 APP 在 CAN 初始化前停住。

预期：

```text
Trial Jump
-> APP 不主动返回
-> IWDG 超时
-> Bootloader 重新启动
-> app_valid 仍为 0
-> GET_STATUS 显示 APP_NO_RETURN / APP_TRIAL_TIMEOUT
-> 之后普通上电不再自动跳入该 APP
```

## 10. 必须保持兼容的原行为

以下行为不能被 Trial 功能破坏：

- 旧 Host 发送 `JUMP_APP` 且 Byte2=0 时，行为与修改前一致；
- `Boot_ShouldJumpApp()` 对正常 `app_valid=1` APP 的自动启动逻辑保持；
- `ENTER_BOOT` 在 Bootloader 内的现有意义保持；
- Legacy ERASE/WRITE/VERIFY 流程保持；
- Autonomous Session 的全部命令和状态保持；
- Guard 节点保护逻辑保持；
- Rollback 和 FULL_STREAM 保持；
- Commit 成功后统一 Jump 保持；
- Flash 布局、APP 起始地址 `0x08005000`、Config Page 地址全部不改。

不要修改 CAN ID 分配。

不要增加新的 Peer Command。

不要增加新的 Session Flag。

## 11. 建议改动文件范围
Bootloader 侧预计只需要：

- `Core/Inc/bootloader/bootloader.h`
- `Core/Src/bootloader/bootloader.c`
- 如 IWDG helper 必须独立，可增加一个很小的 boot runtime/port 文件；若无必要不要新增文件。

APP 侧只改现有 CAN 接收/命令分发相关文件，以及必要的一个小 helper/头文件。

不要修改与此功能无关的电机控制、FOC、Observer、ADC、PWM、传感器、控制算法代码。

不要改 `.ioc`，除非确实无法实现；如果必须改，先说明原因，并确保生成代码 diff 最小。

仓库中如果存在用户当前未提交的 README/协议文档/测试列表改动，不要覆盖、格式化或清理这些无关文件。

## 12. 错误码与状态输出

尽量只新增一个 Trial 失败错误码，例如：

```c
BOOT_ERR_APP_NO_RETURN
```

或：

```c
BOOT_ERR_APP_TRIAL_TIMEOUT
```

二选一即可，不需要为了第一版细分多个失败类型。

已有：

```text
CRC错误       -> BOOT_ERR_CRC_MISMATCH
Vector非法    -> BOOT_ERR_APP_INVALID
Trial无返回   -> 新增 Trial error
```
`GET_STATUS` 必须能看到 Trial 失败错误；`GET_INFO` 可继续使用现有 `app_valid` 字段判断该 APP 是否被正式接受。

不要为了展示 Trial 结果扩展控制帧长度。

## 13. 验收标准

Codex 修改完成后必须逐项自检：

1. `JUMP_APP seq=0` 与旧版本完全兼容；
2. 合法新 APP 可以执行 `JUMP_APP seq=1` Trial；
3. Trial 前 Flash metadata 确认 `app_valid=0`；
4. APP 收到 ENTER_BOOT 后写同一个 `BOOT_REQUEST_MAGIC` 并 `NVIC_SystemReset()`；
5. 正常返回后 Bootloader 恢复 `app_valid=1`，但本次先停留 Bootloader；
6. APP 不处理 ENTER_BOOT 时，IWDG 能把 MCU 拉回 Bootloader；
7. Trial FAIL 后 `app_valid=0`，普通复位/上电不会再次自动进入坏 APP；
8. `GET_STATUS` 能区分 Trial 无返回与普通 CRC/Vector 错误；
9. Observer_Motor APP 不存在周期喂狗代码；
10. 现有 Repair/Guard/Rollback/Coordinator/Commit 无逻辑变化；
11. CAN_FD_IAP 使用 **Release** clean build 成功；
12. Bootloader 仍严格小于 20 KiB，并报告修改前后 Flash/RAM 差值；
13. Observer_Motor Boot/Release 构建成功；
14. `git diff --check` 通过。

## 14. 完成后输出给用户的报告

不要直接 commit/push，除非用户明确要求。

完成后只报告：

- 实际修改了哪些文件；
- Trial Jump 具体使用哪个 Byte/字段；
- IWDG 实际 timeout；
- APP ENTER_BOOT 插入到哪个现有接收路径；
- PASS/FAIL 判定逻辑；
- Release Flash/RAM 修改前后大小；
- 编译/静态检查结果；
- 是否存在尚未实机验证的风险。

核心原则再次强调：**只补 APP Trial Jump / 返回 Bootloader 防砖闭环，其他协议与自治升级主流程不要动。**
