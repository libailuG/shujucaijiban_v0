# RTOS control migration

This directory is a self-contained snapshot of the RTOS control firmware.
Its `App`, `Drivers`, `Startup`, FreeRTOS kernel and linker script are local
copies. Changes to the original firmware tree do not update this snapshot.

Build from this directory with `powershell -ExecutionPolicy Bypass -File .\build.ps1`.
The image is `ObjectsCli/zhuban_rtos_controls.hex` and `.bin`.
The Keil project is `zhuban_rtos_controls.uvprojx`. Its HEX output is
`ObjectsKeilRtosControls/zhuban_rtos_controls.hex`.
The original build and the phase-1 zero-torque image live in `../firmware`.

The motor task remains the highest-priority task and retains the TIM3 5 ms
alternating schedule and TIM2 RS-485 DE compare. The IMU task has the next
priority. TIM7 checks IMU transfer timeouts every 1 ms and starts a sample
every 10 ms. Motor slots are released independently by TIM3.
A control task handles UART9 commands, runs control modes 1, 2 and 3,
and publishes command snapshots. The I/O task handles cascade and battery
service. UART4 telemetry remains owned by the control task.

| Device | Board connector | UART | TX / RX / DE |
| --- | --- | --- | --- |
| Left motor (ID 126) | JP6, RIGHT IMU2 | UART5 | PB13 / PB12 / PB14 |
| Right motor (ID 127) | JP2, LEFT IMU1 | USART6 | PC6 / PC7 / PC8 |
| Left acquisition chain | JP9, RIGHT IMU3 | UART7 | PE8 / PE7 / PG0 |
| Right acquisition chain | JP5, LEFT IMU2 | USART1 | PA9 / PA10 / PA11 |

JP3 (RIGHT IMU1) is unused in this build.

The red LED (PG10) is solid when both acquisition chains report fresh, valid
records from their foot, shank and thigh boards. It blinks with a full period
of 0.3 s when only the left chain is healthy, 0.6 s when only the right chain
is healthy, and 1 s when neither chain is healthy. The on and off times are
equal. Each chain requires all three of its boards.

Each accepted UART9 command clears the pending motor command before changing
mode or permissions. The control task then publishes a new command snapshot.
The motor task checks the mode and command age before constructing a frame.
Mode 1 and 2 freshness limits are 21 ms and 11 ms. Mode 3 uses 25 ms. Manual
motor debug commands retain their event semantics. Motor-position zeroing runs
in the motor task and suppresses torque while its sequence is active.

Mode 2 uses the `hip_assist_caiji_v0` encoder policy at 100 Hz with grade 5
and precontact profile 2. UART9 `data3` still sets the output multiplier and
`data1.bit0` grants torque permission. The drive limit is ±15 N·m per side.
Right-side feedback and commands are each sign-converted once between drive
and model coordinates. UART4 telemetry uses the existing version 1 layout.
The diagnostic feedback cache still accepts the last finite encoder sample.
The internal guard invalidates the published motor command immediately and
stays latched until Mode 2 is reset. The external oscillation monitor checks
the larger absolute command and retains its 200 ms recovery.

In control mode 3, both motors use the same sinusoidal torque command in drive
coordinates. UART9 feedback reports these commands in
`data8` (left) and `data9` (right).
In mode 3 only, UART9 `data10` counts normal commands from both motors that
missed the 2 ms reply deadline, including zero-torque polling. The count
starts at mode entry and saturates at 16,777,216. The motor task can retry
once on the same bus if it is idle and the next regular slot is at least
1 ms away. A valid reply before the retry cancels it. The retry uses the
current command and does not open a second timeout window.

Before motion testing, inspect `g_rtos_motor_coalesced_slots`,
`g_rtos_motor_max_lateness_us`, `g_control_mode2_monitor.deadline_misses`,
`g_motor_left.busy_skips` and `g_motor_right.busy_skips`. Check UART9 stop,
mode changes, permission changes, manual mode and motor-position zeroing on
the board. Measure each motor's 10 ms transmission interval and DE-to-TX
setup with a logic analyzer, including during IMU reconnect and UART load.
