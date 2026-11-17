# 双路采集主板实现与 DAP 验证（2026-11-16）

已完成右侧采集链路，并烧录最终双路固件。HEX 全镜像 verify_image 校验通过，测试结束 CPU 为 running。

## 最终接口

- 左侧身体链路 → 主板右侧 IMU2 / JP6 / UART5，PB13 TX、PB12 RX、PB14方向。
- 右侧身体链路 → 主板左侧 IMU2 / JP5 / USART1，PA9 TX、PA10 RX、PA11方向。
- 均为2M、8N1；两路各4096字节DMA缓冲、独立握手、解析器、UID绑定和快照。
- 左脚flags检查0x101，右脚flags检查0x201；新数据未接入现有电机控制观测或遥控上行。

## 问题定位与修复

此前接线核对表将JP5方向引脚误记为PA12。第一版据此配置时JP5接收为0字节。
交换两条已上电链路后，右侧链路接JP6能够收到完整协议帧，已知正常的左侧链路接JP5仍为0字节。
DAP输入采样：PA10在500次采样中低电平0次，PB12为182次。
重新放大主控原理图确认RE#DE_1位于U7第103脚/PA11（PA12为104脚，未连接该网）。
已更正固件、双路说明和历史接线表。恢复原接线后，修正版USART1握手与数据接收成功。
原理图核对源：firmware/data/数据采集板.pdf，第2页；放大参考tmp/pdfs/zhuban/pins2.png。

## 最终持续测试

通过CMSIS-DAP/SWD 500kHz，按Keil实际烧录AXF符号读RAM，持续30.574593秒，31个诊断采样点。
持续观察期间不暂停、不复位主板；烧录阶段已按流程复位。

| 指标 | 左侧 | 右侧 |
|---|---:|---:|
| 接受帧数增量 | 14469 | 11505 |
| 实测接收帧率 | 473.24Hz | 376.29Hz |
| UART/DMA/CRC错误增量 | 0/0/0 | 0/0/0 |
| 长度/语义拒绝增量 | 0/0 | 0/0 |
| 重复/推送序号跳号增量 | 0/0 | 0/0 |
| 接收覆盖保护计数 | 1→1 | 1→1 |
| 握手请求计数 | 2→2 | 2→2 |
| 流接收状态 | 全程3 | 全程3 |
| UID绑定 | 成功 | 成功 |

六个来源均已识别，末次采样topology=1、present_mask=7、online=1；记录status均0，左右脚标志正确，IMU数据/姿态有效位均为1。
末次记录年龄左侧脚底/小腿/大腿约6.019/5.636/4.318ms，右侧约5.635/6.060/4.774ms。
左右三源UID和新样本频率详见final_result.json。
DAP实时读取跨越更新，不是多源原子同步快照；年龄为诊断样本，不是精确同步测得的端到端时延。

**双路通信与数据更新通过；500Hz端到端性能尚未达标。** 两侧速率不同、此前也有波动，当前未观察到主板推送序号缺口，不能仅据此把原因归到主板或上游。
未执行人为拔线恢复、传感器标定、压力加载或闭环控制验收。

## 构建与产物

- 原生C离线测试282项通过（含左右隔离、绑定、CRC恢复、缺记录、过期、回绕）。
- Keil 0错误、0警告；命令行构建也通过-Wall -Wextra -Werror。
- 实际烧录：firmware/Objects/zhuban_caiji.hex。
- SHA256：DBC48B69572691C750B304A387C391BD340FEFDF7B7F3C8E3879855105EB7183。
- 最终烧录日志：flash_fixed.log；持续测试：final_observe.log；统计：final_result.json。
- 初版与修正前的测试保留于initial_result.json、swapped.log、pins.log，不代表最终版本结果。
- 原单路Keil HEX/AXF及修改前代码保存在before目录。

## 代码入口

firmware/App/cascade_right.c/.h：USART1/JP5右侧实例。
firmware/App/cascade_left.c/.h：UART5/JP6左侧实例。
firmware/App/cascade_port.inc：共用传输实现，每个编译单元拥有独立静态状态。
firmware/App/cascade_receiver.c/.h：根据配置的脚侧校验数据。
firmware/CASCADE_DUAL.md：接线、DMA资源和双路快照API。
