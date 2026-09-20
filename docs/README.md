# 项目文档目录

本目录集中存放 CAN_FD_IAP 项目的协议、测试、验证和实施说明。项目入口及快速概览请先阅读根目录 [`README.md`](../README.md)。

## 协议与使用

- [`STM32G431 Bootloader 协议与使用说明.md`](STM32G431%20Bootloader%20协议与使用说明.md)：完整协议、Flash 布局、升级状态机和现场使用说明。
- [`COMMAND_TEST_GUIDE.md`](COMMAND_TEST_GUIDE.md)：CANPro 控制帧、响应帧和逐命令测试步骤。

## 验证与适配

- [`VALIDATION.md`](VALIDATION.md)：构建、静态检查和端到端验证记录。
- [`HOST_WINDOW_FLOW_CONTROL_REPORT.md`](HOST_WINDOW_FLOW_CONTROL_REPORT.md)：Host 窗口流控设计、边界条件和验证结论。

## 实施任务

- [`CODEX_APP_TRIAL_JUMP_TASK.md`](CODEX_APP_TRIAL_JUMP_TASK.md)：APP 试运行跳转任务记录与验收项。

根目录 `README.md` 保留同一份文档索引，便于从项目首页进入；本文件用于在 `docs/` 内快速定位资料。

帧生成工具 `bin_to_boot_frames.py` 保留在仓库根目录，命令默认从仓库根目录执行。
