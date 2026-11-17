# 左侧 IMU / 压力级联接入

链路：左脚压力板 → 左小腿板 → 左大腿板 JP2 → 主板右侧 IMU2 JP6。
接口图纸左右名称不表示传感器的身体侧别。当前模块只接受左脚标志。

## 接线与串口

主板 UART5：PB13 TX / PB12 RX，均 AF14；PB14 为 RS485 方向，低接收、高发送。
2,000,000 bit/s、8N1，BRR=69，实际约 1,992,754 bit/s。

主板 JP6：1 GND、2 A、3 B、4 +5V。大腿 JP2：1 +5V、2 A、3 B、4 GND。
按信号接线，不能按四芯端子编号直接直通；供电按系统方案处理。

## 软件入口

- `App/cascade_protocol.c/.h`：从 cascade_release/mainboard_protocol 复制的共享线上协议。
- `App/cascade_receiver.c/.h`：纯 C 三源语义校验、UID 绑定、年龄/状态掩码。
- `App/cascade_left.c/.h`：UART5、DMA、启动握手、恢复、快照 API。
- 双路共享传输实现已移至 `App/cascade_port.inc`；右侧链路配置见 `CASCADE_DUAL.md`。
- `main.c`：初始化后在每轮主循环最先执行 `cascade_left_poll()`。
- Keil 工程与 build.ps1 均包含以上源文件，使用同一 board.sct。

DMA1 Stream3 / DMAMUX1 Channel3 / request65 为 UART5 RX 专用。
4096 字节循环缓冲在 AXI SRAM 0x24000100，隔离完整 cache line；读取前失效缓存。
不使用 RXNE 逐字节中断，不依赖 IDLE 定界。主循环每轮最多解析512字节。
轮询间隔达到20ms或积压接近覆盖边界时，丢弃受损流并重新同步；UART/DMA错误亦如此。
DMA停止与重启分两轮完成，不在主循环等待 DMA 停止。

握手先监听50ms；已有推送通过连续三帧不同推送序号及一致来源身份绑定。
无推送且总线静默50ms时请求 INFO，应答事务序号匹配且 role=3、支持流后请求2000us推送。
握手超时80ms，失败至少间隔100ms重试；START前至少静默1ms。
PB14发送建立至少2us；TXE/TC中断发送，TC中断立即释放方向，正常运行仅接收。
100ms无接受的新推送进入恢复监听。持续噪声/错误来源不会触发向忙碌总线强行发命令。
UID绑定跨掉线保留；更换采集板需要重启主板。源重启后的倒退序号在离线后可重新接受。

## 读取数据

```c
CascadeSnapshot left;
cascade_left_get_snapshot(&left);
/* role-1: records[0]脚底，records[1]小腿，records[2]大腿 */
if (left.online && (left.valid_mask & 1U)) {
    float front_voltage = left.records[0].values[0];
    (void)front_voltage;
}
```

快照复制由短 IRQ 屏蔽保护，可在主循环或中断中读取。
present_mask / valid_mask / attitude_mask 位0/1/2依次对应脚底/小腿/大腿。
缺失记录在本帧清零，不沿用上一帧冒充新数据。
校验 CRC、长度、版本、role去重、UID、有限浮点数、脚底左侧标志、状态和0..3.6V电压范围。
3.6V为3.3V ADC的容差检查，可按实际硬件收紧。
驱动错误或超过50ms的数据不能通过有效掩码；100ms无新推送时离线。
年龄保守加入DMA排队/组帧停留上界、实际串行时间及读取时本地停留时间。
应用可按业务设置更严格的年龄阈值；本模块的50ms阈值不代表适合闭环控制。

脚底前三个 values 为V；IMU依次为加速度m/s²（含重力）、角速度rad/s、姿态rad。
本次提供采集快照，没有将新传感器接入策略观测、改变现有电机/遥控协议或坐标变换。
`g_cascade_left_diag` 可在调试器查看状态、收帧/错误计数和每源新样本计数。
状态0监听、1等待身份、2等待启动、3流接收、4重试等待。

## 验证与边界

离线原生 C 测试：`work/cascade_test/test_receiver.c`，覆盖协议CRC向量、逐字节解析、
CRC恢复、三帧绑定、重复、错误侧别/UID/角色/NaN/长度、缺记录、过期及计数回绕。
工具为 work/cascade_test/tcc/tcc.exe，编译命令见 test.ps1。
固件通过 build.ps1 编译链接（-Wall -Wextra -Werror），未进行实板烧录和端到端验收。
仍需板测：A/B极性与方向时序、500Hz持续吞吐、UART/DMA错误及拔线恢复、
实际三源UID/左脚标志/年龄、当前控制负载下的最大主循环间隔。
