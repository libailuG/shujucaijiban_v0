控制模式1扭矩门控（2026-11-13）：data2=3时，data1.bit0默认0禁止策略扭矩输出，1允许输出；关闭时电机仍为mode1全零/KP0，策略计算及监测继续。按最后有效遥控事件保持开关，不设消息超时；退出模式清开关，bit3强制普通静止优先。模式1的bit0仍是置零请求。

按钮4：接收data1.bit3=1时优先于data2，直接进入既有普通静止模式，回执data2=0，清除调试扭矩和策略状态，发送mode1零扭矩/KP0。不触发新置零；此前置零按既有退出路径安全完成。无新指令则保持普通模式；bit3=0后按该帧data2选择，不自动恢复旧模式。

控制模式1接入（2026-11-12，仅编译/离线验证，未烧录）：
IMU最大年龄10ms；左右电机反馈最大年龄30ms、两侧时间差最大15ms；周期20ms±1ms。
诊断：g_control_mode1_status/ticks/last_compute_us/selftest_error。状态0成功、1暖机、2禁用、负值错误。
编译后的符号地址与旧固件不同，不能用于解释当前板端RAM；板端耗时尚未实测。

模式模块入口（本次只拆分代码，协议不变）：
App/normal_mode.c/.h：普通模式反馈。
App/motor_debug_mode.c/.h：电机调试目标接受、反馈；置零状态机/底层安全发送仍在motor_bus.c。
App/imu_debug_mode.c/.h：IMU调试上行，保留最终竖装映射。
App/control_mode1.c/.h：控制模式1，50Hz运行sim2real_v0策略；方向、零点和单位适配仅在本模块。
App/control1_policy/s2_deploy.c/.h：原部署包逐字复制，包含权重、推理和自检。
App/reserved_uart2.c：UART9解析、实际模式接受/回执与模块分发。所有poll均不应阻塞。

UART9新增控制模式1：data2整个uint32值恰好为3进入，实际接受回执data2=3。control_mode1_poll按20ms实测周期运行策略；进入时清除电机调试目标，先10次零输出暖机，再输出策略扭矩，初始限幅6Nm，扭矩帧KP=10.0。退出、IMU或左右反馈过期、错误及置零期间清空策略并保持mode1零扭矩/KP0；不触发新置零，已开始置零继续安全完成。控制模式1上行data4/5为左右电机原始符号角度[度]（反馈过期/无效时为0），data8/9为左右输出扭矩命令Nm（驱动符号、限幅和方向转换后，非实测扭矩/非发送ACK）；暖机或异常为0，其余上行字段沿用普通模式。预留串口2为JP7/UART9，921600波特，20Hz上行，无静默超时退出。

IMU调试模式最新扩展（用户确认2026-11-12）：
  UART9接收data2=0普通，data2=1电机调试，data2=2 IMU调试；其他数值回普通。
  上行data2返回主控实际已接受的模式0/1/2。按整个uint32数值判断，不是bit判断。
  IMU调试(data2=2)的data3..data10，先角度，再角速度，再加速度，不发送yaw：
    data3 Roll，data4 Pitch，单位度（从内部rad乘180/pi转换）。
    data5/6/7 角速度X/Y/Z，单位rad/s。
    data8/9/10 加速度X/Y/Z，单位m/s^2，包含重力。
  本模式8个float全部用于IMU，不携带电压，无磁力计数据。板载ICM42688P为六轴。
  进入IMU调试时退出电机扭矩调试，电机目标清零、KP0，仍发送mode1静止帧。
  UART9的data2=2不等于电机协议mode2，不触发新的电机置零！
  先前已启动的电机一次性置零流程不强行中止，仍按原状态机静止查询恢复。
  data2=1仍按现有左右电机反馈映射；data2=0普通data3电压、data4/5 Roll/Pitch仍按原rad单位。
  IMU调试退出时清理data10等残留IMU量。data1.bit0仍只用于电机调试中的置零请求/状态。
  遥控器事件发送，无静默自动退出；2Mbps电机链路与UART9 921600/20Hz保持。

遥控字段最新映射（2026-11-12，覆盖此前开关位定义）：
  接收data2为uint32，整个数值等于1才进入电机调试；0/2/3等其他数值均退出。
  上行data2=1表示主控实际已进入调试，=0表示未进入。
  接收data1.bit0为置零开关；原data1.bit1不再触发置零，data1.bit0不再控制调试入口。
  上行data1.bit0为置零状态：执行中每250ms翻转，左右静止查询恢复后保持1，关闭后0。
  其他流程不变：左mode2一次，左TC后至少10ms右mode2一次；mode1零扭矩查询，不等mode2 ACK。
  置零开关保持打开时一直零扭矩/KP0；关闭且两路已恢复后才恢复调试扭矩控制。
  不使用mode0。遥控器事件发送，不增加无消息超时退出。

调试开关2位置置零（用户确认2026-11-12）：
  置零开关为接收data1.bit0，仅在调试模式下0->1触发。持续为1绝不重复置零。
  等待现有发送完成，左ID126发送mode2一次；从左发送完成TC起至少10ms后，右ID127发送mode2一次。
  此后两路周期下发mode1静止帧：扭矩/速度/位置/KP/KD全0。不等待mode2 ACK，也不重发mode2。
  两路各收到静止查询后的新mode1有效反馈才标记通信恢复，不使用旧缓存；并非Flash内容校验。
  执行中上行data1.bit0每250ms翻转；两路均恢复且开关2仍开时稳态为1；关后为0。
  开关2保持打开期间，即使置零完成，所有下发扭矩始终0、KP0。打开期间的扭矩指令不缓存。
  关闭开关2后才允许恢复控制；若尚在置零/等待恢复，先保持零，完成后才应用关闭帧中的新控制量。
  提前关闭不取消已开始的左右一次性置零流程。不做超时重发，也不自动宣告完成。
  重复置零需要前一次完成后重新0->1；执行过程中额外按下不排队。
  遥控器必须区分用户请求位和主控回传闪烁，不能把闪烁回传形成新按键指令。
  Watch: g_motor_zero_state(0空闲1排队2左发3等10ms4右发5等恢复6完成)，
         g_motor_zero_switch，g_motor_zero_left_sent/right_sent(累计完成发送数)，g_motor_zero_recovered(bit0左bit1右)。

预留串口2电机调试模式（用户确认，2026-11-12）
  接收data2整个数值恰好等于1进入调试，其他数值退出；上电默认退出。
  遥控器按事件发送、可能长时间静默；保持最后一条有效模式/扭矩，无静默超时退出。
  打开时接收data8/data9为左/右目标扭矩Nm；两电机均mode1，KP固定50，其余控制字段0。
  上行data2=1/0回传主控实际接受的模式状态，不等同电机执行ACK。
  调试上行data4/5为左/右角度[度]，data6/7为左/右角速度rad/s，data8/9为左/右实际扭矩Nm。
  data3仍为电压V；data10上传右电机反馈扭矩Nm（与data9相同，保留原始符号），data2用于模式状态。
  关闭时恢复下方长期静止约定：mode1零扭矩且KP0，禁止mode0；上行data4/5恢复Roll/Pitch，6..9清0。
  非有限目标扭矩(NaN/Inf)拒绝整条打开命令并计数；关闭命令始终允许，忽略扭矩字段。
  Watch: g_motor_debug_mode，g_motor_debug_torque_left/right，g_motor_debug_reject_count。
  主控UART9仍921600、50ms上行，电机两路仍2Mbps、10ms下行。

当前电机通信与静止约定（用户确认，2026-11-12；优先于下方历史说明）
  左ID126/JP3/USART3，右ID127/JP2/USART6；两路2Mbps，周期10ms。
  以后“静止/停止运动/待机”统一使用：mode1扭矩模式，目标扭矩0。
  速度、位置、KP、KD及其余控制字段全0，保持使能；不使用mode0，也不自动回退mode0。
  仅接收反馈时同样发送上述零扭矩帧。这不是断电或PWM封波指令。
  当前主控实际固件已使用此设定，两台电机均返回mode1有效反馈。
  蓝灯PG14暂时预留并保持熄灭。ESP32-C6 UART9配置不变。
  后续修改、排查、烧录均保留这项静止约定，除非用户另行明确更改。

STM32H723ZGT6 主板最小点灯程序

运行效果：绿色 LED6（PG9）亮 500ms、灭 500ms，循环。
从开机开始闪烁，由 TIM6 定时器中断独立驱动。
红色 LED5（PG11）、蓝色 LED4（PG14）保持熄灭。LED 低电平点亮。
使用外部 25MHz 晶振和 PLL1，主频 550MHz；初始化板载 ICM-42688-P。

文件与编译
  App/main.c：GPIO 和闪灯逻辑。
  App/imu.h、imu.c：IMU 数据结构和每 1ms 的采集服务。
  App/imu_bus.c：I2C4 总线及 TIM2 微秒时钟。
  App/icm_imu_i2c.c：复用此前子板的传感器驱动（未引入 FIFO 功能）。
  App/mahony.c：复用此前子板的六轴姿态解算。
  Startup/system_stm32h7xx.c：550MHz 时钟、内核 LDO 和向量表初始化。
  zhuban_caiji.uvprojx：Keil MDK 工程，选择 Arm Compiler 6。
  build.ps1：命令行编译脚本，已在本机编译通过。
  Objects/zhuban_caiji.hex：烧录文件。
  Objects/zhuban_caiji.bin：二进制，起始地址 0x08000000。
  Objects/zhuban_caiji.axf：带符号的调试文件。

在本目录执行：powershell -ExecutionPolicy Bypass -File .\build.ps1
默认编译器位于 %LOCALAPPDATA%/Keil_v5/ARM/ARMCLANG/bin。
可使用 -Toolchain 参数指定其他 Arm Compiler 6 的 bin 目录。
Keil 工程选型 STM32H723ZGTx，需 STM32H7xx_DFP 4.1.3。
首次下载前在 Debug 选择实际的 ST-Link/J-Link，接口选择 SW。
下载算法选择 STM32H72x-73x_1024（1MB Flash）。

烧录
  可使用 STM32CubeProgrammer 或 Keil，通过 SWD 下载 HEX，然后复位。
  JP1：1=GND，2=3.3V参考，3=NRST，4=SWDIO，5=SWCLK。
  连接前核对实板排针方向。固件从 Flash 起始地址运行，无 Bootloader 偏移。
  2026-11-10 已连接实板并完成 OpenOCD 和 Keil 命令行烧录、校验与运行检查。

硬件前提
  先确认主控 3.3V 正常。原理图 U4 的 EN 画为接地，若实板一致，
  需要先修正使能接法；软件无法开启该 LDO。
  原理图 LED 串联电阻为 10k，实际亮度可能较低。

第三方来源
  Device 头文件及启动汇编来自 ST 官方 cmsis-device-h7：
  https://github.com/STMicroelectronics/cmsis-device-h7
  Core 头文件复用相邻工程的 Arm CMSIS 版本；未使用附带的 F1 设备头文件。
  许可证见 Drivers/CMSIS 下的 LICENSE 文件及源文件头。

550MHz 时钟配置
  HSE=25MHz（晶体模式，不是 BYPASS），PLL1 M=5、N=110、P=1。
  VCO 输入 5MHz，VCO 输出 550MHz，CPU/SYSCLK=550MHz。
  AHB/AXI=275MHz，APB1/2/3/4=137.5MHz。
  内核 LDO + VOS0；Flash LATENCY=3，WRHIGHFREQ=3。
  SysTick 按实际 SystemCoreClock 配置 1ms；绿灯仍亮灭各 500ms。
  SystemCoreClockUpdate 仅适用于本工程的 HSI/PLL1 两种固定配置。

550MHz 必要条件
  CPU_FREQ_BOOST 选项字节必须启用，否则此芯片的上限为 520MHz。
  烧录前可在 STM32CubeProgrammer 的 Option Bytes 中检查并启用
  CPU_FREQ_BOOST（某些版本显示 CPUFREQ_BOOST），应用后复位或重新上电。
  此程序只检查该位，不自动改写选项字节。
  成功：绿色灯按 1 秒周期闪烁。
  失败：红色灯常亮，初始化函数返回错误码：
    1=CPU_FREQ_BOOST 未启用；2=VOS0 未就绪；3=外部晶振未启动；
    4=PLL1 无法关闭；5=Flash 配置失败；6=PLL1 未锁定；7=时钟切换失败。
  超时等待不会继续切换到目标高频。程序不自动回退到 520MHz。
  已在实板确认晶振和 PLL 启动：CPU=550MHz、AHB=275MHz。
  GPIOG ODR 的 PG9 位随时间翻转。尚未进行长时间稳定性或温升测试。

时钟参考
  https://www.st.com/resource/en/datasheet/stm32h723zg.pdf
  https://www.st.com/resource/en/reference_manual/dm00603761.pdf
  https://github.com/STMicroelectronics/stm32h7xx-hal-driver/blob/master/Inc/stm32h7xx_hal_rcc.h

Keil 下载设置
  ST-Link、SW、Normal、Autodetect，SWD 时钟 100kHz。
  勾选 Program、Verify、Reset and Run。

按键初始化
  SW1=PC12，SW2=PC11，SW3=PB4，低电平按下，上拉输入。
  每 1ms 扫描，连续 20 个样本一致后确认按下或松开；支持同时按下。
  当前只记录消抖后的状态，不绑定任何灯光或模式切换动作。
  Keil Watch 可查看 g_button_state（消抖后的按下位图）。
  位图 bit0=SW1、bit1=SW2、bit2=SW3；1=按下，0=松开。
  按键 GPIO 及代码在 App/main.c。未改变 PA13/PA14 的 SWD 调试功能。

绿灯定时器
  TIM6 时钟 275MHz，PSC=27499，ARR=4999，每 500ms 触发更新中断。
  TIM6_DAC_IRQHandler 中翻转 PG9，完整闪烁周期为 1 秒。
  SysTick 仅用于每 1ms 扫描按键，不再控制绿灯。

板载 IMU
  型号 ICM-42688-P，WHO_AM_I=0x47，自动探测 0x69 和 0x68。
  实板已识别地址 0x69。I2C4：PD12=SCL、PD13=SDA，AF4，开漏。
  I2C4 使用 HSI 64MHz 内核时钟，SCL 低于 400kHz；需要外部上拉。
  TIM2 为 1MHz、32 位自由运行计数器，负责 1ms 采集调度与总线超时。
  TIM7 每 1ms 发起六轴 12 字节寄存器读取，PSC=274，ARR=999。
  I2C4_EV / I2C4_ER 中断负责寄存器地址发送、重复 START、接收和错误处理。
  中断中没有总线等待循环或姿态解算；主循环 imu_poll() 消费完整数据并解算。
  TIM2 保留微秒时间戳和超时计时；TIM6 绿灯、SysTick 按键扫描保持独立。
  只保留一个完成槽，未消费或上一笔未完成时跳过当前时隙，不覆盖数据、不积压补读。
  TIM7 / I2C4 中断优先级均为 5，彼此不抢占；绿灯 TIM6 优先级为 3。
  卡住的传输由下一个 TIM7 中断检查并终止，超时检查阈值为 800us，检查间隔 1ms。
  初始化和故障后的重初始化仍在主循环使用有超时的同步 I2C。
  当前采用中断传输，未使用 DMA。不是 FIFO，不保证保留每个内部 ODR 样本。
  量程：加速度 +/-16g，角速度 +/-2000度/秒。传感器内部 ODR 沿用原驱动 32kHz。

数据结构
  全局变量 g_imu_data，类型 ImuData，定义在 App/imu.h。
    angle_rad.roll / pitch / yaw：横滚、俯仰、相对偏航，单位 rad。
    gyro_rad_s.x / y / z：角速度，单位 rad/s。
    accel_m_s2.x / y / z：包含重力的加速度，单位 m/s²。
    timestamp_us：TIM7 发起读取的时刻，TIM2 微秒时间戳，约 71.6 分钟回绕。
    sample_count：成功读取的累计次数。
    valid：最近一次读取是否成功，1=成功；错误时旧数值不代表新样本。
    angle_valid：姿态是否已通过静态重力初始化；初始剧烈运动时可能为 0。
    last_error：0=正常；负数表示总线或传感器错误。
  六轴 Mahony 解算没有磁力计，不提供绝对航向，yaw 会随零偏漂移。
  业务代码建议使用 imu_get_data(&local_data) 获取一致快照；
  例如 ImuData local_data; imu_get_data(&local_data); 再检查 valid / angle_valid。
  连续 5 次读取失败后，每 500ms 重新尝试初始化；没有数据时不会伪造有效样本。

验证
  修改前的同步采集版本已实板验证 1000Hz。
  TIM7 + I2C4 中断采集版本已通过 Keil 编译、烧录及校验。
  实板样本数差 1021、时间戳差 1021000us，对应 1000Hz；valid=1、last_error=0。
  短暂错误地址测试产生 NACK（last_error=-11、valid=0），随后自动恢复到
  0x69 地址，valid=1、angle_valid=1、last_error=0。测试未改写 Flash 选项字节。
  不包含长期稳定性、温漂和姿态精度标定。

Battery monitoring (4S Li-ion/LiPo, full 16.8V):
PF11/ADC1_INP2, schematic R18=100k, R19=10k, gain=11.
16-bit ADC, conversion clock 8MHz, 810.5-cycle sampling, 100Hz nonblocking service.
EMA 1/16 (~160ms); 0.10V upward hysteresis, immediate downward band transition.
Level 4 >=15.6V: green 0.5Hz (toggle 1000ms).
Level 3 >=14.8V: green 1Hz (toggle 500ms).
Level 2 >=14.0V: green 2Hz (toggle 250ms).
Level 1 <14.0V: green 4Hz (toggle 125ms).
ADC fault: level=0, valid=0, last_error negative, green 4Hz.
Watch: g_battery_data; voltage_v / filtered_voltage_v in volts, level=1..4.
Nominal VREF+=3.3V. Adjust BATTERY_REFERENCE_V or BATTERY_CALIBRATION_GAIN
in App/battery.h against a multimeter if needed. Bands indicate voltage,
not accurate state of charge. TIM6 LED and TIM7 IMU remain independent.


JP7 reserved serial port 2 (UART9, not USART2):
JP7 pin1 GND, pin2 RX=PD14/AF11, pin3 TX=PD15/AF11, pin4 +3.3V.
921600 baud, 8N1, no flow control. PCLK2=137.5MHz, BRR=149.
Binary protocol: App/mcu_uart_protocol.h copied from the supplied C6 protocol.
Exactly 48 bytes, little endian, start=0xAAAAAAAA, end=0xEEEEEEEE.
No newline, string terminator, extra commands or CRC. hello_world removed.
TX period: 50ms (20Hz), RESERVED_UART2_TX_PERIOD_US in reserved_uart2.h.
g_uart2_tx_frame / g_uart2_rx_frame are both td_mcu_frame_t (48 bytes).
TX mapping:
  data3 = g_battery_data.filtered_voltage_v, volts.
  data4 = g_imu_data.angle_rad.roll, radians.
  data5 = g_imu_data.angle_rad.pitch, radians.
  data1/data2/data6..data10 start at zero and are otherwise left unchanged.
Check g_battery_data.valid and g_imu_data.angle_valid separately in Watch;
no extra status semantics or error values are added to the wire protocol.
TX buffer remains immutable until the UART TC interrupt completes the frame.
RX interrupt -> 1024-byte ring -> main-loop parser -> latest received frame.
Supports fragmented/concatenated frames, junk prefixes, overlapping headers,
payload marker bytes and bad-tail resynchronization. UART byte loss/overflow
resets partial parsing. At most 256 RX bytes are consumed per main-loop pass.
Received frames are saved only: no automatic echo or hardware control.
Use reserved_uart2_get_rx() for a snapshot; use rx_frame_count for updates.
reserved_uart2_set_tx() is a main-loop API. data3..5 are refreshed each TX.
Watch:
  g_uart2_tx_count: completed local TX frames, not a remote acknowledgment.
  g_uart2_rx_count: received UART bytes; g_uart2_rx_frame_count: parsed frames.
  g_uart2_rx_valid: at least one frame received (not a freshness indicator).
  g_uart2_rx_timestamp_us: last valid frame time (TIM2, wraps ~71.6 minutes).
  g_uart2_error_count / g_uart2_rx_overflow_count / g_uart2_rx_bad_tail_count.
## 控制模式2：v10 SVD r64（2026-11-17）

- UART9接收data2整个uint32值等于2进入模式2；IMU调试仍为12，模式1仍为1。
- 模式2的data1.bit0为扭矩许可：0继续推理/监测但发送mode1零扭矩/KP0，1允许输出；不触发置零。事件指令保持，不增加消息超时。
- data1.bit3优先进入普通静止；模式切换清除策略。已有置零流程及反馈异常均抑制输出并清空策略。
- 固定100Hz，10ms步长；实测间隔超过11ms清空状态并计数，不补跑。主机验证不代表板端实时能力。
- 输入关节端rad、rad/s，左右顺序，符号[+1,-1]；输出转驱动符号同样[+1,-1]。不依赖IMU数据。
- 策略及两层振荡保护源码从参考工程逐字复制至App/control2_policy。800维历史、SVD r64、脉冲和内置guard不改。
- 与参考app_config.h一致的八档：0.6、0.714285714、0.828571429、0.942857143、1.057142857、1.171428571、1.285714286、1.4。按用户要求启动默认倍率1.2（自定义倍率，output_level=UINT32_MAX），最终限幅正负15Nm；无额外6Nm限幅。历史保留缩放前策略力矩。
- control_mode2_set_output_level(0..7)选择档位；g_control_mode2_output_level/scale可观察。当前遥控未分配档位字段，启动保持默认档；切换模式不重置档位。生产代码应调用setter使档位和比例一致。
- 电机发送继续使用本板接口，模式2输出有效时KP=20（电流系数，用户更新），模式1仍为10，无有效输出时KP=0；不复制原F405电机驱动或其KP字段语义。
- 两侧反馈最大年龄30ms、时间差15ms；检查mode1、错误标志、角度和速度有限值。外部振荡触发后停助约0.2s，再重建历史；内置保护锁存到策略停止/重置。
- 上行data2=2，data1.bit0回显许可；data4/5为左右原始符号角度[度]，data8/9为最终驱动符号扭矩命令Nm，不是实测扭矩或执行ACK。data6上传整轮控制计算平均耗时ms，data7保留0，其他字段沿用普通反馈。
- Watch：g_control_mode2_monitor含status、ticks、resets、计算时间、周期、deadline_misses及两层保护状态。status：0成功、1初始化、2禁用、3外部抑制，-1反馈/置零、-2周期异常、-3数值异常。
- 测试：work/control2_v10_20261117/test.ps1；覆盖八档参考一致性、方向、门控、异常、置零和两层保护。使用附带32位TCC运行主机测试；固件由Arm Compiler 6构建。
- 本次仅编译和离线验证，未烧录。Keil与CLI均构建成功；上板后需测计算耗时、实际周期、deadline_misses及其他采集链路是否受影响。

模式2计算耗时上行（2026-11-17）：预留串口2/UART9的data6=float整轮控制计算平均耗时，单位ms，Watch为g_control_mode2_monitor.average_compute_ms。从读反馈前到输出命令准备结束计时，包含反馈检查、历史/归一化/推理、脉冲与保护、命令转换及门控，并包含期间的中断抢占时间；不含等待下一周期、串口物理发送或其他模块poll。自最近一次reset统计完成周期，含外部保护抑制周期；无样本或reset后为0。仍按10ms调度，调用间隔平均值保留在average_period_ms，但不再通过data6上传。沿用20Hz上行及48字节帧格式。板载IMU采样保持原逻辑，暂停采样改动已撤回。

控制模式2首轮运行时间优化（2026-11-17）：
- SystemInit在设置VTOR后调用SCB_EnableICache，使用CMSIS先失效再开启指令缓存；不启用D-Cache，不改DMA数据缓存行为。
- Keil的Mode_Control2组单独使用-O3，命令行对control2_policy两个C文件使用-O3；保留-ffp-contract=off，不启用fast-math。其他控制模式编译选项保持原样。
- 原模型源码、权重、ELU、脉冲/保护、默认倍率1.2、15Nm限幅及10ms调度不变。
- 两套构建通过；反汇编确认I-Cache启用位设置、推理循环展开及硬件浮点运算。离线功能测试通过；尚未在硬件比较优化前后数值和耗时。
- 优化前Keil HEX/策略目标文件及修改前配置备份在work/control2_opt_20261117/before；反汇编在同级目录。新固件为Objects/zhuban_caiji.hex。
- 上板复位后，在相同工况下比较UART9 data6（整轮控制平均计算耗时ms）、max_compute_us及deadline_misses；暂停/断点会污染计时。实际提速和最坏耗时需实测，不由编译成功推断。

控制模式2预留串口1遥测（2026-11-17）：data2=2自动上传ControlMode2Telemetry，80字节/100Hz，921600/8N1。bit0只影响扭矩许可，不关闭上传。六项为最新有效左右反馈角度/速度（模型符号[+1,-1]）及本周期缩放和两层保护后的计算力矩（许可前）；另含最终驱动命令、状态/有效标志、设备时间、序号和耗时。复制及UART发送位于计算耗时计时之外。串口忙则非阻塞丢帧，g_uart1_tx_drop_count计数；发布序号仍递增。异常也上传，必须检查flags，不把无效0视为真实测量。协议与上位机入口见host/control_mode2_monitor/README.md。I-Cache、O3、原网络源码保留。仅构建/离线验证，未烧录。


控制模式3（2026-11-17，当前协议，覆盖历史data2=3映射）：
  JP7/UART9 921600：data2整个uint32=3进入；data1.bit0=1允许，0禁止；bit3强制普通静止并清许可。
  按用户最新要求右侧取反：原始驱动坐标 T左=8*sin(pi*t)，T右=-8*sin(pi*t) Nm，幅值8Nm、周期2秒。
  模块App/control_mode3.c/.h独立，无网络/策略推理，不修改模式2符号。主循环按微秒时间更新，电机发送沿用10ms。
  初次许可、禁止后恢复、退出后重入、反馈故障恢复、置零结束均从零相位起步。
  反馈年龄<=30ms、左右时间差<=15ms，两路有效mode1、error_flag=0、角度/速度有限；置零时抑制。
  出力沿用当前板控制模式KP=10电流系数，其余控制字段0；禁止/故障返回mode1全零/KP0，不发送mode0。
  bit0在本模式仅为许可，不触发置零；模式11既有一次性置零流程保留并可在切换后继续安全完成。
  串口2上行20Hz：data2=3，bit0回执许可；data3电压V，data4当前状态位掩码，data5历史故障OR，
  data6当前周期内相位时间秒[0,2)，data7运行清零累计数，data8/9最终左右驱动命令Nm，data10=0。
  状态0正常；1禁止；故障位2置零、4/8左/右无效、16/32左/右过期、64时间差、128/256左/右mode错误、
  512/1024左/右error、2048/4096左/右非有限。历史故障保留至重启，便于定位短脉冲原因。
  模式2串口1的80字节遥测协议不变；模式3现以version=3发送同长度帧，详见后续扩展说明。
  本次离线验证和构建，未烧录，硬件实际出力尚待验证。


模式3参数与串口1扩展（2026-11-17，覆盖此前模式3不上串口1的说明）：
  串口2 data2=3，data1.bit2=1时应用data8=幅值Nm、data9=周期秒；bit0许可、bit3普通静止优先。
  幅值0..15Nm，周期0.1..60秒，默认8Nm/2秒。无bit2忽略data8/9；NaN/Inf/越界拒绝整组并保留旧值，不阻止停止。
  参数变更从零相位重启，重复相同参数不重启；保持至下次修改或重启，不写Flash。
  C接口control_mode3_set_parameters返回1成功/0拒绝；g_control_mode3_parameter_rejects为拒绝计数。请通过接口设置。
  JP4/UART4 921600自动100Hz上传，包括禁止/故障状态。使用start_control2.bat，自动识别模式2/3，六曲线及CSV复盘。
  沿用80字节LE帧、头尾和flags：模式2 version=1保持不变，模式3 version=3，status为模式3故障位掩码。
  角度/速度/计算力矩为左原始、右取反坐标；最终驱动为左sin、右-sin，mode1、KP电流系数语义不变。
  version3最后两个float为幅值Nm、周期秒（复用output_scale/average_compute_ms）；period_us上传间隔，compute_us上传准备耗时。
  CSV新增control_mode/amplitude_nm/sine_period_s；模式3旧output_scale/average_compute_ms列为幅值/周期别名。
  串口1仅上传，调参仍走串口2。串口2其余模式3反馈字段沿用上节。未烧录。


模式3左侧单独测试（2026-11-17，最新要求覆盖上述双侧输出）：仅左侧输出可调正弦，右侧扭矩固定0，仍发送mode1全零/KP0静止帧。右侧计算及最终驱动扭矩上传为0；左右反馈保护保持，参数接口及上传频率不变。仅影响模式3，未烧录。

模式3默认幅值更新（2026-11-17）：5Nm，默认周期仍2秒，仅左侧输出；串口2有效调参仍可覆盖默认值。未烧录。

模式3默认幅值更新（2026-11-17，覆盖此前5Nm）：3Nm，默认周期2秒，仅左侧输出，右侧mode1零扭矩/KP0。串口2有效调参仍可覆盖默认值。未烧录。

模式3右侧KP更新（2026-11-17，覆盖此前右侧KP0）：允许出力且反馈正常时，左右KP均为10；左侧正弦默认3Nm/2秒，右侧扭矩仍为0。禁止、异常及置零时左右仍沿用mode1全零/KP0。未烧录。

模式2反馈停机判定临时关闭（用户要求，2026-11-17，覆盖此前30ms/15ms等规则）：read_feedback仅因置零开关或置零流程未完成返回失败。反馈年龄、左右时间差、反馈mode和error_flag不再触发停机/策略重置；暂未收到反馈、反馈丢失或角度/速度为NaN/Inf时保留最近一份有限角度/速度，不把NaN/Inf送入网络。尚无历史反馈时缓存默认零。UART9角度上行采用同一缓存。此改动意味着反馈丢失不会自动停止扭矩，限用于当前排查。data1.bit0输出许可、退出、周期和数值异常处理、两层振荡保护及置零抑制仍保持原逻辑。KP有效输出为20，禁用/置零仍0。模式1/3不变。

遥测bit0在本排查固件中表示可用的反馈缓存，不保证反馈新鲜；status=-1在模式2中仅由置零抑制触发。CSV协议字节布局不变。之前CSV中的-1仍按当时固件解释，不得重解释为置零。

模式2默认系数更新（用户要求，覆盖此前1.2）：默认output_level=0，output_scale=0.6；KP=20、15Nm最终限幅及当前反馈检查旁路逻辑保持不变。未烧录。

模式2下发系数（2026-11-17，覆盖此前默认0.6与固定档位入口）：上电g_control_mode2_output_scale=0，output_level=UINT32_MAX。每次接受预留串口2/UART9 data2=2帧时，将float data3作为输出倍率。bit0仍独立控制扭矩许可；data3=0则输出零，0.6表示0.6倍。仅在模式2接受系数，其他模式data3含义不变；遥控事件静默时保持最后收到的系数，不增加超时；再次进入模式2时采用该帧data3。负数、NaN或Inf系数按0处理，不使用负系数反转方向。有效非负值不再量化到八档，最终仍有±15Nm力矩限幅，KP=20及当前反馈旁路逻辑不变。系数改变时立即清除旧倍率的缓存命令，下个有效控制周期生成新命令，保留未缩放策略历史。旧档位setter仅保留为内部兼容接口。串口1遥测output_scale上传当前倍率；串口2上行data3仍为原电压反馈，不是系数回显。未烧录。

模式1反馈错误处理临时关闭（用户要求，2026-11-17，覆盖此前IMU10ms/电机30ms/左右15ms规则）：IMU valid/angle_valid/last_error、IMU与电机反馈年龄、左右反馈时间差、电机反馈mode/error_flag或暂时没有反馈，不再触发模式1清零/重置。采用最近有限的IMU欧拉角/陀螺及左右电机角度/速度；新值NaN/Inf则保留缓存，首次无数据缓存为零。传入策略的enabled/sensors_valid按可用缓存置1。只有置零开关/置零流程仍走原输入失败清零路径；模式退出、输出许可、10周期暖机、自检、20ms±1ms周期和策略数值检查保持原样。调试器monitor的反馈新鲜度有效位仍按真实年龄记录；52字节串口1观测可能来自缓存，不保证新鲜。断反馈不再自动停止出力，本版用于当前排查。模式2和模式3代码未改；本次构建未烧录。

模式3反馈保护取消（2026-11-17，覆盖此前反馈故障清零说明）：反馈无效/过期、左右时间差、mode错误、error_flag及非有限角度/速度仅记录当前状态和累计故障，不停止正弦、不重置相位；反馈曲线无效时仍不显示无效数值。手动禁止、退出、bit3普通静止及既有置零抑制保持。当前左侧默认3Nm/2秒，右扭矩0，允许出力时两侧KP10；仅模式3生效。未烧录。
