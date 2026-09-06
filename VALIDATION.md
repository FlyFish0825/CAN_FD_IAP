# V1.3 Validation Notes

## 1. Bootloader Release 编译

当前工程已使用 **CMake Release preset** 做 STM32G431 真实交叉编译：

```bash
cmake --build --preset Release --clean-first
```

V1.3 完成 UART、启动框架、向量表和 FDCAN ISR 体积优化后的 clean build 结果：

```text
RAM    7448 B / 32 KiB  22.73%
FLASH 16488 B / 20 KiB  80.51%
剩余   3992 B
0 compiler/linker error
```

Linker 固定 `FLASH ORIGIN=0x08000000, LENGTH=20K`，Bootloader 超过 `0x08004FFF` 会直接链接失败。

体积优化前后对比：

| 构建节点 | Flash |
|---|---:|
| V1.2 安全 Jump 完成后 | 19,324 B |
| 删除 UART 后 | 17,688 B |
| Release 编译细化后 | 17,344 B |
| V1.3 最终 clean build | **16,488 B** |

V1.3 静态检查结果：

- ELF 类型为 ARM executable，入口指向 Thumb `Reset_Handler`；
- 无未解析符号；
- Bootloader 向量表为 152 B，覆盖内核异常和外部 IRQ0~IRQ21；
- 当前最高且唯一启用的外部中断为 FDCAN1 IT0（IRQ21）；
- 仅激活 `FDCAN_IT_RX_FIFO0_NEW_MESSAGE`；
- 通用 `HAL_FDCAN_IRQHandler()` 已不进入最终 ELF；
- 协议命令、CRC、Config/Metadata 布局及 APP 起始地址均未改变。

## 2. 协议 / 工具验证

- CRC8/ATM(`"123456789"`) = `0xF4`；
- CRC32/MPEG-2(`"123456789"`) = `0x0376E6E7`；
- `Observer_boot.bin` size = `56120 B = 0xDB38`；
- Packet count = `1003 = 0x03EB`；
- 当前实测 APP CRC32 = `0xFE565BD9`；
- 最后一包 Seq=`1002 = 0x03EA`；
- Classic fallback DATA = `1003*8 = 8024` 个标准 CAN 帧。
## 3. 单节点 CANPro 标准 CAN 实测

已使用 Node1 完成 Legacy 闭环：

```text
ERASE
→ WRITE 56120 B
→ 1003 个逻辑 DATA / 8024 个 Classic CAN fragments
→ WRITE_END
→ Missing Count = 0
→ VERIFY
→ JUMP_APP
```

关键实测结果：

```text
Flash Missing = 0
Expected CRC32 = 0xFE565BD9
Actual CRC32   = 0xFE565BD9
APP vector[0]  = 0x20008000
APP vector[1]  = 0x0800D871
```

因此已经验证：CANPro 标准 CAN 分片、逻辑 64B 重组、Flash program + read-back、Bitmap、整镜像 CRC32、metadata 持久化均工作正常。

## 4. Bootloader → APP Jump 实机问题与修复

首次 JUMP 时 APP 未闪烁。定位后确认不是下载/CRC/VTOR 问题，而是 **Bootloader 与 APP 使用不同 PLL 参数**：

```text
Bootloader SYSCLK = PLL 170 MHz
Observer APP      = PLL 168 MHz
```

STM32G4 `HAL_RCC_OscConfig()` 在 PLL 正作为 SYSCLK 时，不允许直接改成另一组 PLL 参数，因此 APP 在 `SystemClock_Config()` 返回 HAL_ERROR 并进入 `Error_Handler()`。
修复位于 `Boot_RuntimeJumpToApp()`：

```c
HAL_DeInit();
HAL_RCC_DeInit();
```

随后关闭 SysTick、清 NVIC enable/pending、清 PendSV/SysTick pending、设置 APP VTOR/MSP 后再进入 APP Reset_Handler。

修复后同一份 `Observer_boot.bin` 无需重新 OTA 下载，重新烧写新版 Bootloader 后即可正常跳 APP，PC4/PC6 500ms 闪烁测试通过。

## 5. 当前已验证 / 未验证边界

已验证：

- 单节点 Legacy 下载完整闭环；
- 标准 CAN Classic fallback 8024 帧；
- Flash read-back / Missing Bitmap / CRC32；
- APP 向量表与 `0x08005000` 偏移；
- Bootloader→不同 PLL APP 的安全 Jump。

以上实机结果来自 V1.2 完整功能版本，协议核心在 V1.3 中未改变。V1.3 已完成 clean build 和 ELF 静态检查，但由于启动框架、向量表和 FDCAN 中断入口经过精简，仍需按下面第一项重新做上板回归后，才能把 V1.3 标记为完整实机验证通过。

仍需上板验证：

1. V1.3 上电、GET_VERSION=1.3.0.0、FIFO0 连续接收、ERASE/WRITE/VERIFY/JUMP 基础回归；
2. 8 节点 `WRITE_END` 后 Coordinator Claim 时序；
3. 不同节点丢不同 Sequence 的联合恢复；
4. 同一 Sequence 多节点缺失只广播一次；
5. Provider 中途掉线与 10s timeout 换 Provider；
6. Guard 新版本失败后的自动 Rollback；
7. Guard FULL_STREAM 更新 + Repair；
8. Prepare/Commit 与掉电/复位边界；
9. 正式 CAN FD+BRS 数据面的吞吐与持续接收。

当前没有实现“Coordinator 已经成功 Claim 后又掉线”的 heartbeat/重新选主，测试和文档均不得把该能力视为已实现。
