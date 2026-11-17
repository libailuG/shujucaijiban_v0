# 修改与验证记录

已实现模式0普通、1控制模式1、11电机、12机身IMU、13/14/15左大腿/小腿/压力、16/17/18右大腿/小腿/压力。
上行data1.bit6为所选传感器数据有效、bit7为姿态有效；压力无姿态位。协议详见 firmware/RESERVED_UART2_PROTOCOL.md。
电机层移除旧硬编码mode_value==1，以统一常量11判断，避免新控制模式1误入电机调试或置零。
上行bit0按模式复用：模式1回显策略许可，11为置零状态；其他模式清零。

验证：
- firmware/build.ps1：编译、链接、HEX/BIN生成通过，-Wall -Wextra -Werror。
- work/uart2_modes_20261116/test.ps1：新模式编号、bit6/7掩码、六路来源、角度换算、压力字段、过期/离线/缺失/无效、姿态独立无效、机身IMU及NaN检查通过。
- work/cascade_test/test.ps1：原有282项检查通过。

本次未烧录、未连接或操作设备，未做硬件联调。遥控端需使用新编号同步升级。
CLI产物：firmware/ObjectsCli/zhuban_caiji.hex、bin、axf。Keil Objects目录未更新，不能将其旧产物视为本次固件。
修改前源码备份在本目录before。
