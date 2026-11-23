# S2 v30 独立 sim2real

本包参考 hip_assist_lab/sim2real 的 Python/C99 双实现与软件对照方式。
只部署 S2 外骨骼 v30 策略，不部署 H170 本体控制器；没有 CAN/UART/HAL、
电流环或实机标定数据，未进行实机验证。不得使用其它 hip_assist 权重/观察布局。

## 模型与时序

来源：`S2_H170_DS_v30_sim2sim/policies/S2_v30_fp32.pt`。
SHA256：`25981d75a0ee2f00ca352532d40fdbfc8f0131d56687a61d1034dcc52039d8a0`。
FP32，65→256→128→64→2，隐藏ReLU，最后一层线性；
标准化 `(obs-mean)/(std+epsilon)`，epsilon保留原值约0.01。
输出 `clip(raw,-1,1)*12 Nm`，不额外限速，每20ms推理并保持主动命令。
reset清空10帧历史和上一主动命令，前10次有效帧更新历史/计算网络但主动输出零。
第11次开始允许非零命令。每帧恰好调用一次，不能按1kHz推进历史。

所有左右均为穿戴者左右。外骨骼关节后伸正、前抬负；腰带B坐标X右/Y前/Z上。
完整65维索引、人体方向、单位、IMU四元数、编码器/电机转换均见 `c/s2_deploy.h`。
网络历史的扭矩固定除40，不得改为除12。
使用当前瞬时体坐标陀螺仪，不能以四元数差分替代当前策略语义。

本包输出主动命令，不添加仿真中的 `-0.05*dq` 被动阻尼。
仿真每1ms执行 `clip(active-0.05*dq,-12,12)`；这与20ms主动命令有区别。
硬件主动命令成功发送后的模型坐标值才能进入历史；外部限幅后须commit。
发送失败需停驱动并reset，不能将实测电机扭矩/电流/含阻尼总扭矩反馈进历史。

## Python

```python
from controller import Controller
controller = Controller()
# q/dq为外骨骼模型坐标rad、rad/s；quat_wb为wxyz，B→Z向上世界。
# gyro为当前B坐标rad/s；全部传感器需要同步、有效、新鲜。
status, active_model_nm = controller.step(q, dq, quat_wb, gyro_body_rad_s,
                                         measured_dt_s=0.02)
# status=0正常，1暖机，2禁用，负数故障。必须显式发送返回命令（含零）。
# motor_sign需每侧实测；driver_joint_nm=motor_sign*active_model_nm。
# 若发送前又限幅，发送成功后：controller.commit_command(limited_model_nm)
```

`map_hardware()` 提供安装旋转、编码器符号/零点转换；默认不确认标定。
运行 `python demo.py` 为无硬件示例。Python仅需NumPy，不依赖训练项目或MuJoCo。

## C99 / STM32

硬件只需 `c/s2_deploy.c`、`c/s2_deploy.h`，参考 `c/main.c` 接入。
权重已内嵌，无堆分配。50Hz接口 `s2_app_tick_50hz()`，高级接口
`s2_controller_step()` 使用调用方独立状态。模型接口和硬件接口不可混用。
本包ABI版本30，不能与旧v17对象文件混链接；必须重新编译。

```bash
make -C c
./build/example
```

默认示例仅自检后退出；`board_*` 是未实现的硬件占位。
示例标定未确认，额外主动限幅2Nm用于展示接入；完整策略默认12Nm。
故障/禁用会输出零并清空状态，下一次有效使能重新暖机。
故障锁存、重新使能、数据新鲜度/时间戳/通信看门狗由硬件层实现。
不得忽略错误后继续发送上一非零命令。
可选限幅/限速会改变模型包装，报告和实测时应记录实际参数。

编译：C99、libm、`-ffp-contract=off`；不要`-ffast-math`。
M4F可用 `-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard`，
需匹配工程浮点ABI/启动FPU。Flash权重233236字节，每次约57728乘加；
PC/交叉编译不代表板端50Hz最坏耗时通过，需板端测量。
本机M4F交叉编译通过：text/rodata 238220字节，单例BSS 2412字节；
`bash build_cortex_m4.sh` 可重新生成静态库（需ARM GCC/Newlib）。
`python package.py` 生成相邻的 s2_v30.zip，不包含主机二进制/缓存。

## 复现验证

在 env_isaaclab_0 环境：

```bash
make -C c
OPENBLAS_NUM_THREADS=1 python tests/validate.py /home/libai/12_x1/x1_dege/S2_H170_DS_v30_sim2sim
```

验证内容：FP32权重逐字节一致、1000随机观测、2000顺序帧、100组安装转换、
非法输入/时序/配置/禁用、20.2秒原仿真观测对照，以及独立C控制器闭环20.2秒。
验证入口对原仿真只注入ROOT路径模块，避免原服务器私有路径探测；不改源码。
C串行浮点归约与PyTorch/NumPy矩阵乘法存在小量舍入差，不宣称逐位相同。
结果见 `validation/validation.json`。实际传感器、延迟、硬件方向与助力效果尚未验证。
