# V1 Validation Notes

已完成的离线检查：

- 所有 `bootloader/*.c` 使用 C11 + `-Wall -Wextra -Werror` 做语法/告警检查通过（使用与 STM32G4 HAL 关键类型/宏一致的最小桩头文件）。
- `boot_protocol_reference.py` 自检通过：
  - CRC-8/ATM("123456789") = `0xF4`
  - STM32 CRC32 / CRC-32-MPEG-2("123456789") = `0x0376E6E7`
  - Node1 GET_VERSION 请求 = `01 01 00 00 00 00 00 F6`
- 已按 STM32G4 FDCAN HAL 的固定 Message RAM 结构修正发送空闲判断：Tx FIFO/Queue 深度按 3 个元素处理；不访问不存在的 `TxFifoQueueElmtsNbr` 初始化字段。

上板前仍必须在你的真实 STM32G431 工程中完成：

1. 实际交叉编译，确认最终 Bootloader `.text + .rodata + data load image` 不超过 20 KiB。
2. `FDCAN_MODE_NORMAL`、FD+BRS、两个标准 ID Filter (`0x000` / `0x100`) 的真实总线测试。
3. 在实际 CAN FD 数据速率下测试“边接收边 Flash 编程”的持续吞吐；如硬件 FIFO/Flash 编程导致大量系统性缺包，中控侧需要增加发送节拍/块级流控，而不是单纯提高恢复轮数。
4. 实测掉电点：Config 失效、ERASE 中断、WRITE 中断、VERIFY 前后复位。
5. APP linker origin 与 VTOR 均确认在 `0x08005000`。
