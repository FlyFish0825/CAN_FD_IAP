# FDCAN 环形接收缓冲接入说明

本次接入保持现有 Bootloader 协议接口不变：

- `boot_can_config.h`：未修改；
- `boot_can_port.h`：未修改；
- `boot_can_protocol.h`：未修改；
- `boot_can_protocol.c`：未修改。

只新增：

- `can_rx_buffer.h`；
- `can_rx_buffer.c`。

并对 `main.c`、`fdcan.c` 和顶层 `CMakeLists.txt` 做了最小接入修改。

## 1. 接收流程

```text
FDCAN RX FIFO0
       |
       | 中断
       v
HAL_FDCAN_RxFifo0Callback()
       |
       v
CAN_RX_Fifo0Callback()
       |
       | 只读取 FIFO0 并写入环形缓冲
       v
CAN_RX_Process()，在 main() 的 while(1) 中运行
       |
       | 筛选 ID / 标准帧 / 数据帧 / DLC=8
       v
BootCAN_Process(msg.data, msg.dlc)
```

中断中不再直接执行协议解析、回复发送或后续 Flash 操作，因此连续接收时中断占用时间更短。

## 2. 唯一 HAL 回调

工程中只能有一个：

```c
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0ITs)
```

本工程原来已经在 `Core/Src/stm32g4xx_it.c` 中定义了该回调。本次保留这个原位置，只把回调体由“直接读取并解析协议”改成“转交给环形缓冲”。没有在 `main.c` 中再定义第二个回调。

以后如果其他 `.c` 文件又添加了同名函数，必须合并到 `stm32g4xx_it.c` 这一处，不能保留两个定义。原回调里存在的：

```c
HAL_FDCAN_GetRxMessage(...);
BootCAN_Process(...);
```

已经删除，并改为只调用：

```c
CAN_RX_Fifo0Callback(&boot_rx, hfdcan, RxFifo0ITs);
```

否则同一个硬件 FIFO 可能被重复读取，并且链接时会出现重复定义错误。

## 3. DLC 与 memcpy 修正

STM32 HAL 的 `header.DataLength` 是 `FDCAN_DLC_BYTES_x` 编码，不是可以直接使用的普通字节数。

`CAN_RX_DlcToBytes()` 会把它转换成真实长度：

- 0～8 字节；
- 12、16、20、24、32、48、64 字节。

随后只执行：

```c
memcpy(msg.data, raw_data, msg.dlc);
```

因此不会再像旧 bxCAN 版本一样无论实际 DLC 是多少都固定复制 8 字节。

## 4. 缓冲区大小

旧版本使用 256 个 Classic CAN 槽位。现在为了后续兼容 CAN FD，每个槽位的数据区扩展为 64 字节；如果仍保留 256 帧，预计会占用约 18 KiB RAM。

本版本使用：

```c
#define CAN_RX_BUFFER_SIZE 32U
```

环形缓冲保留一个槽位区分“满”和“空”，所以实际最多缓存 31 帧，RAM 占用约 2.3 KiB。

调试时重点观察：

- `boot_rx.count`：当前积压帧数，仅用于观察；
- `boot_rx.overflow_count`：软件缓冲溢出次数，正常应始终为 0。

如果 `overflow_count` 增加，应先检查主循环是否被阻塞，并在后续下载协议中增加 ACK/窗口流控，而不是直接把缓存继续放大。

## 5. main.c 调用位置

初始化顺序为：

```c
MX_FDCAN1_Init();
HAL_FDCAN_ConfigFilter(...);
HAL_FDCAN_ConfigGlobalFilter(...);
CAN_RX_Init(&boot_rx, &hfdcan1);
BootCAN_Init(1);
HAL_FDCAN_Start(&hfdcan1);
HAL_FDCAN_ActivateNotification(...);
```

主循环持续调用：

```c
while (1)
{
    CAN_RX_Process(&boot_rx);
}
```

原来的 `HAL_Delay(1000)` 已删除，因为它会导致接收帧最多每秒才处理一次，连续通信时很容易把缓冲区填满。

## 6. FDCAN 配置变化

`fdcan.c` 当前已经是：

```c
hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
```

适合与外部 CANPro 通信。

本次仅把：

```c
hfdcan1.Init.StdFiltersNbr = 0;
```

改为：

```c
hfdcan1.Init.StdFiltersNbr = 1;
```

否则 `main.c` 配置的 `FilterIndex = 0` 没有对应的标准过滤器空间。

本次已经把 `CAN_FD_IAP.ioc` 中的 Standard Filters Nbr 同步设置为 1，避免以后使用 CubeMX 重新生成代码时把 `fdcan.c` 覆盖回 0。
