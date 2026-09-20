# Bootloader 单节点窗口下载：上位机适配报告

## 1. 适用版本

- Bootloader 协议版本：`1.3.1`
- MCU：STM32G431
- APP 起始地址：`0x08005000`
- CAN FD：仲裁段 500 kbit/s，BRS 数据段 5 Mbit/s
- 本报告只描述 Host 对单节点 Legacy 下载的新增窗口流控要求。

## 2. 修改原因

旧实现每收到一个 64B 逻辑 DATA，就立即把其中 56B 固件数据拆成 7 个
Double Word 写入 Flash。Host 连续高速发送时，Flash 平均写入速度可能低于
CAN FD 输入速度，有限的 FDCAN FIFO 和软件队列会溢出。

新实现把 64 个逻辑 DATA 组成一个 SRAM 窗口：

```text
64 packets × 56 firmware bytes = 3584 bytes/window
```

Bootloader 内部有两个 3584B Flash staging buffer，并把完整消息接收队列扩大
到 64 项。窗口完整后才批量写 Flash、整窗口回读、设置 Sequence Bitmap，最后
通知 Host 继续发送。

## 3. DATA 格式不变

CAN FD 仍使用：

```text
ID=0x100, Standard, FD=1, BRS=1, DLC=64

Byte0       Target Node
Byte1       0x01 (WRITE_DATA)
Byte2~3     Sequence，uint16 little-endian
Byte4~5     Session ID；单节点Legacy固定为0
Byte6~7     Reserved，固定为0
Byte8~63    56B固件数据
```

最后一包不足 56B 时仍用 `0xFF` 补齐。Classic CAN兼容拆帧规则也不变：
一个64B逻辑DATA仍拆成ID `0x100~0x107`八个8B物理帧。

## 4. WRITE响应新增窗口大小

Host发送原有 `WRITE (0x11)` 后，节点响应格式为：

```text
Byte0       Node ID
Byte1       0x11
Byte2       BOOT_STATUS_WRITE = 0x02
Byte3       Region；APP=0
Byte4~5     Total packet count，uint16 LE
Byte6       Window packet count = 64
Byte7       CRC8
```

上位机必须读取 Byte6。Byte6非0表示启用窗口流控；不要再连续发送完整固件。

## 5. 新增 WINDOW_READY / WINDOW_STATUS：0x32

### 5.1 节点主动通知

每个窗口成功写入并回读后，节点在自身响应ID `0x500 + NodeID` 主动发送：

```text
Byte0       Node ID
Byte1       0x32
Byte2       BOOT_STATUS_WRITE = 0x02
Byte3~4     Next Sequence，uint16 LE
Byte5       Window packet count = 64
Byte6       Available window credits，至少为1
Byte7       CRC8
```

`Next Sequence` 是当前 Flash Bitmap 中第一个尚未成功写入的 Sequence。

例如节点已经可靠提交 `Seq 0~63`：

```text
Next Sequence = 64
```

最后一个窗口提交完成时：

```text
Next Sequence = Total packet count
```

只有收到这个条件，上位机才能发送 `WRITE_END`。

### 5.2 Host主动查询

如果 WINDOW_READY 超时或响应丢失，Host发送控制命令：

```text
CAN ID=0x000, Standard Classic CAN, DLC=8

Byte0       Target Node
Byte1       0x32
Byte2~6     0
Byte7       CRC8(Byte0~6)
```

节点用与5.1完全相同的格式返回当前窗口状态。因此ACK丢失时不要直接假定
Flash未写入，也不要盲目进入下一个窗口。

Node1 查询帧示例：

```text
CAN ID 0x000
01 32 00 00 00 00 00 28
```

## 6. 上位机必须采用的发送算法

推荐严格单窗口模式，最容易保证不丢包：

```text
1. ERASE，等待READY
2. WRITE，读取TotalPackets和WindowPackets(64)
3. next = 0
4. 发送Seq next开始、最多64个逻辑DATA
5. 停止发送DATA，等待0x32响应
6. 如果响应NextSequence前进：next = NextSequence，继续下一窗口
7. 如果超时：发送0x32查询
8. 查询仍停在当前窗口：重发当前窗口
9. NextSequence == TotalPackets后发送WRITE_END
10. 确认Missing=0后执行VERIFY
```

伪代码：

```text
while next_seq < total_packets:
    window_start = next_seq
    window_end = min(window_start + 64, total_packets)
    send DATA[window_start .. window_end-1]

    response = wait WINDOW_READY(timeout)
    if timeout:
        response = query WINDOW_STATUS(0x32)

    if response.next_seq >= window_end:
        next_seq = response.next_seq
    else:
        next_seq = window_start       # 重发本窗口，已缓存包会被去重

send WRITE_END
```

上位机可以保留每个窗口的64个逻辑DATA，收到确认后再释放这部分发送缓存。

## 7. WRITE_END忙响应

如果Host在最后一个SRAM窗口尚未提交时提前发送 `WRITE_END`，节点不会进入
Missing/Verify，而是回复：

```text
Command = 0x14
Status  = BOOT_STATUS_WRITE (0x02)
Data0~1 = 当前已经写入Flash的包数，uint16 LE
Data2   = 64
Data3   = 0
```

Host应继续等待/查询 `0x32`，确认 `NextSequence == TotalPackets` 后重发
`WRITE_END`。

## 8. 超时与重发建议

- 窗口确认超时建议初始使用 `500 ms`，实测后可以缩短；
- 超时先查询 `0x32`，不要立刻ERASE或重新开始整个固件；
- 查询显示NextSequence未前进时，重发整个当前窗口即可；
- 窗口内重复Sequence会被SRAM mask去重；
- 已经写入Flash并置位的Sequence会被全局Bitmap去重；
- 连续多次查询仍无进展或收到ERROR，再中止升级并重新ERASE。

## 9. CRC与最终校验保持不变

窗口ACK只表示该窗口已经成功写入并回读，不代表整个APP最终有效。完整升级仍须：

```text
All WINDOW_READY
-> WRITE_END
-> Missing = 0
-> VERIFY整镜像CRC32
-> APP Vector合法
-> app_valid = 1
```

之后才允许普通Jump或Trial Jump。

## 10. 兼容性提醒

- 新版单节点Legacy下载要求Host支持窗口流控；
- 旧版“一次连续发送整个 `.list`”方式不再可靠；
- CANPro若不能在64个逻辑包后等待节点响应，需要把发送列表按窗口拆分；
- Classic CAN模式一个窗口是 `64 × 8 = 512` 个物理帧；
- CAN FD模式一个窗口是64个物理帧；
- 多节点Autonomous Session、Peer Repair、Guard、Rollback和Commit仍走原有路径，
  本次没有把Host单节点窗口协议塞入自治状态机。

## 11. CRC8

所有8字节控制帧继续使用：

```text
CRC-8/ATM
Poly   = 0x07
Init   = 0x00
RefIn  = false
RefOut = false
XorOut = 0x00
```

CRC覆盖Byte0~Byte6，结果放入Byte7。
