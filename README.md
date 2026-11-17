# zhuban_caiji · 主板采集与控制项目

与 `jiaodiban_caiji`、`imuban_caiji` 使用相同的 firmware / host 分层。

## 常用入口

- `firmware/zhuban_caiji.uvprojx`：Keil 工程，目标名 Zhuban_Caiji。
- `firmware/Objects/zhuban_caiji.hex`：Keil 重新构建后的烧录文件。
- `firmware/build.ps1`：独立命令行构建，输出到 ObjectsCli/zhuban_caiji.hex 和 .bin。
- `start.bat`：启动控制模式 1 监测上位机。
- `start_simulation.bat`：启动监测上位机的无硬件演示。
- `start_motor_simulator.bat`：启动独立电机模拟器。

## 目录

- `firmware`：原 App、Drivers、Startup、RTE、data、链接脚本、Keil 配置及构建输出。
- `host/control_mode1_monitor`：控制模式 1 监测上位机，具体串口、接线及协议见其 README.md。
- `host/motor_simulator`：电机模拟器，使用前阅读其 README.md 并确认接线和模拟用途。
- `backups`：原备份；本次重排前配置保存在 before_directory_layout_20261113。
- `zhuban_troubleshooting_archive`、`tmp`、`work`：保留的排障归档、实验和历史整理脚本，历史路径可能仍指向重排前目录。
- `ziliao`：参考资料。

本次仅整理目录、工程名和输出名，保留固件功能、编译参数、调试设置和硬件配置；未烧录或切换设备运行模式。
原 README.txt 已移至 firmware，保留此前功能与验证记录。运行程序请使用以上新入口。

## 左右级联采集

左侧链路接右侧IMU2（JP6/UART5），右侧链路接左侧IMU2（JP5/USART1）。两路2M、独立DMA/握手/快照；详细接线与接口见 [firmware/CASCADE_DUAL.md](firmware/CASCADE_DUAL.md)。JP5方向脚已按原理图更正为PA11。

控制模式2已加入：data2=2启动，data1.bit0=1允许输出扭矩、0禁止输出但继续推理；data1.bit3强制普通静止。IMU调试仍为data2=12。使用v10 SVD r64，100Hz，默认倍率1.2、保留参考八档0.6～1.4，最终限幅±15Nm。档位通过control_mode2_set_output_level选择，未新增遥控档位字段。详细接口、诊断及验证说明见firmware/README.txt。未烧录。

控制模式2上位机（2026-11-17）：新增 start_control2.bat 和 start_control2_simulation.bat。预留串口1/JP4/UART4 921600，模式2自动100Hz上传80字节帧。同图六条主曲线：左右模型坐标角度rad、角速度rad/s、倍率及保护后/许可前计算力矩Nm；另显示最终驱动命令、许可、状态、耗时和序号。支持CSV连续记录及离线二进制日志。说明见host/control_mode2_monitor/README.md。需要烧录新增遥测固件，原模式1上位机和启动入口保留。

模式2上位机CSV记录已按C:/play/pyqtgraph改为：开始保存后自动在host/control_mode2_monitor/data创建北京时间命名的CSV，持续记录所有通道，每秒刷新，结束/断开/关闭时关闭文件；新增独立CSV复盘标签页。暂停绘图不暂停接收和记录。本次不涉及固件。


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

模式3反馈保护取消（2026-11-17，覆盖此前反馈故障清零说明）：反馈无效/过期、左右时间差、mode错误、error_flag及非有限角度/速度仅记录当前状态和累计故障，不停止正弦、不重置相位；反馈曲线无效时仍不显示无效数值。手动禁止、退出、bit3普通静止及既有置零抑制保持。当前左侧默认3Nm/2秒，右扭矩0，允许出力时两侧KP10；仅模式3生效。未烧录。
