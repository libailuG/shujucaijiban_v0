# RTOS phase 1

Build with `powershell -ExecutionPolicy Bypass -File .\build.ps1 -RtosPhase1`.
The output is `ObjectsRtosPhase1/zhuban_rtos_phase1.hex` and `.bin`.
The Keil project is `zhuban_rtos_phase1.uvprojx`. Its HEX output is
`ObjectsKeilRtosPhase1/zhuban_rtos_phase1.hex`.
The original Keil target and the default `build.ps1` target remain the normal firmware.

This image uses FreeRTOS V11.2.0 with static tasks. TIM3 releases the motor task
every 5 ms. The task alternates left and right, targeting one mode 1
zero-torque frame per bus every 10 ms. TIM2 compare interrupts complete the RS-485 DE
setup before DMA starts. TIM7 still schedules 1 ms IMU transfers, and IMU
initialization/retry delays suspend only the IMU task. Cascade, battery and
UART polling run in a lower-priority I/O task.

This phase-1 image supports ordinary, IMU, cascade and acquisition telemetry.
The control modes, manual motor torque and motor-position zeroing are absent
from this image. Incoming requests for those modes select ordinary feedback.
No control-mode command can produce a nonzero motor frame in this build.

For on-board validation, inspect `g_rtos_motor_slot_count`,
`g_rtos_motor_coalesced_slots`, `g_rtos_motor_max_lateness_us`,
`g_motor_left.tx_count`, `g_motor_right.tx_count`, `g_motor_left.busy_skips`
and `g_motor_right.busy_skips`. The lateness value measures task dispatch
after a TIM3 update in microseconds, when one slot is pending. Verify DE-to-TX
and each bus's 10 ms frame interval with a logic analyzer. Repeat with the IMU
unplugged and the UART9 and cascade links active.
