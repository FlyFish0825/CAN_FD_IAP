# V1.2 Validation Notes

## 已完成的本地验证

当前工程已使用 **CMake Release preset** 做真实 STM32G431 交叉编译，不再只是桩头文件语法检查。

最终命令：

```bash
cmake --build --preset Release --clean-first
```

最近一次结果：

```text
RAM   7608 B / 32 KiB   23.22%
FLASH 19132 B / 20 KiB  93.42%
0 warning / 0 error
```

Linker 已固定 `FLASH ORIGIN=0x08000000, LENGTH=20K`，因此超过 Bootloader 区会直接链接失败。

## 协议/工具验证

`bin_to_boot_frames.py` 已通过 Python `py_compile` 和实际 BIN 生成测试。
已验证：

- CRC8/ATM(`"123456789"`) = `0xF4`；
- CRC32/MPEG-2(`"123456789"`) = `0x0376E6E7`；
- `Observer_boot.bin` size = `56120` B；
- Packet count = `1003`；
- APP CRC32 = `0x8D6ECDAB`；
- Session `0x1234` 的最后一包 Seq=`0x03EA` 头 = `FF 01 EA 03 34 12 00 00`；
- Classic 输出总计 8024 个 DATA fragments + SendList 首尾标签，结构正确。

## V1.2 静态一致性检查

- Header 中 41 个 `BOOT_CMD_*` 均能在 Core 的 switch 中找到处理 case；
- `Boot_GetRecoveryPhase()` 等新增调试 API 已有声明和定义；
- `git diff --check` 仅发现文档行尾空格，代码本身未发现 whitespace error；
- Release clean build 0 warning / 0 error。

## 仍需上板验证

1. 8 节点同时 `WRITE_END` 后 Coordinator Claim 时序；
2. 不同节点丢不同 Sequence 的联合恢复；
3. 同一 Sequence 多节点同时缺失，只广播一次修复包；
4. 最低 ID 节点掉线后的后续 ID 接管；
5. Guard 新版本失败后的自动 Rollback；
6. Guard FULL_STREAM 更新 + Repair；
7. Prepare/Commit 与掉电/复位边界；
8. 正式 CAN FD+BRS 数据面吞吐和 Flash 编程期间的持续接收能力。
