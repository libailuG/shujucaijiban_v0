# 左右采集链路

| 身体侧别 | 主板图纸接口 | 串口与引脚 | DMA RX |
|---|---|---|---|
| 左侧 | 右侧IMU2 / JP6 | UART5，PB13 TX / PB12 RX，AF14；PB14 DE | DMA1 Stream3 / DMAMUX1 Channel3 / request65 |
| 右侧 | 左侧IMU2 / JP5 | USART1，PA9 TX / PA10 RX，AF7；PA11 DE | DMA1 Stream4 / DMAMUX1 Channel4 / request41 |

两路均为2M、8N1、半双工RS485，方向低接收、高发送。
USART1使用与USART6/UART9相同的PCLK2时钟源，不更改时钟树。
主板JP5/JP6均为1 GND、2 A、3 B、4 +5V；大腿JP2为1 +5V、2 A、3 B、4 GND，按信号连接。

## 实现与接口

`cascade_port.inc` 是共享接收/握手实现，由 `cascade_left.c`、`cascade_right.c` 分别实例化。
两路拥有独立的静态状态、解析器、UID绑定、超时和诊断，UART中断和DMA资源互不复用。
每路4096字节循环缓冲，分别位于AXI SRAM 0x24000100、0x24001100；主循环每轮每路处理至多512字节。
`cascade_receiver_init_side()` 分别配置0x100（左脚）或0x200（右脚），不接受对侧/双侧脚底标志。
共用协议CRC、记录长度和状态检查，详细启动/恢复与年龄规则见 CASCADE_LEFT.md。

```c
#include "cascade_left.h"
#include "cascade_right.h"
CascadeSnapshot left, right;
cascade_left_get_snapshot(&left);
cascade_right_get_snapshot(&right);
```

每侧 records[0/1/2] 对应本侧脚底/小腿/大腿；检查 online、present_mask、valid_mask、attitude_mask 和年龄后使用。
`g_cascade_left_diag` / `g_cascade_right_diag` 分别提供收帧、错误、绑定和新样本计数。
每侧掉线、接错侧别或UID变化只影响该侧快照。
新数据已接入预留串口2调试上行（模式13..18），见 RESERVED_UART2_PROTOCOL.md；未接入现有策略观测或电机控制。

## 构建与验证

Keil工程与 build.ps1 同时包含双路模块；独立命令行输出位于 ObjectsCli，Keil输出位于 Objects。
两种构建的符号地址可能不同，DAP测试必须使用与实际烧录HEX对应的AXF。
离线测试：`work/cascade_test/test.ps1`，覆盖左右侧别隔离、掉线恢复和既有协议边界。
本次DAP测试/烧录日志位于 `work/cascade_dual_20261116`，测试结论以该目录报告为准。

