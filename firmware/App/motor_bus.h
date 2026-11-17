#ifndef MOTOR_BUS_H
#define MOTOR_BUS_H
#include "motor_protocol.h"
#include "motor_debug_mode.h"
#define MOTOR_PERIOD_US 10000U
#define MOTOR_BAUD 2000000UL
/* Logical left=JP3/USART3/126, right=JP2/USART6/127.
 * KP is a current coefficient; mode1004 torque is magnetic encoder position.
 * Permanent stationary policy: enabled mode1 TORQUE, all control fields zero.
 * 静止/待机必须使用mode1零扭矩；禁止发送mode0。2Mbps，每10ms发送。
 * 调试例外：UART9 data2数值恰好等于11时，按接收data8/data9发送左右扭矩，KP=50。
 * 控制模式1可通过模块命令接口提供扭矩，仍受置零流程抑制，方向和单位适配不在此层。
 * Debugger edits to tx_frame do not alter the transmitted command. */
typedef struct {
    MotorFrame tx_frame, rx_frame;
    uint32_t tx_count, rx_frame_count, rx_byte_count, rx_timestamp_us, rx_valid;
    uint32_t uart_errors, overflow_count, wrong_id_count, bad_tail_count, busy_skips;
    /* Milliseconds since any RX byte (including malformed data), not link health.
     * Counts from init; reset immediately by RX IRQ; saturates at UINT32_MAX. */
    uint32_t rx_idle_ms;
    uint32_t tx_dma_errors; /* Aborted TX frames; excluded from tx_count. */
} MotorData;
extern volatile MotorData g_motor_left, g_motor_right;
/* Diagnostic observations only; pin sampling is not a logic-analyzer capture. */
typedef struct {
    uint32_t rx_pin_samples, rx_pin_low_samples;
    uint32_t last_tx_us, max_tx_us, error_flags_or, last_rx_byte, rx_during_tx;
} MotorDiagnostics;
extern volatile MotorDiagnostics g_motor_diag_left, g_motor_diag_right;
#define MOTOR_RX_RECENT_SIZE 128U
/* JP3/right-side IMU1 raw RDR history, including UART-error bytes.
 * write_index points to the NEXT slot; valid_count saturates at 128.
 * Oldest byte: (write_index + 128 - valid_count) % 128.
 * Latest byte: (write_index + 127) % 128, when valid_count != 0.
 * Halt the target for a coherent debugger view of bytes and metadata. */
typedef struct {
    uint8_t bytes[MOTOR_RX_RECENT_SIZE];
    uint32_t write_index, valid_count, total_count;
} MotorRxRecent;
extern volatile MotorRxRecent g_motor_left_rx_recent;
/* 状态：0空闲、1等待空闲、2左置零发送中、3间隔10ms、4右置零发送中、
 * 5发送静止帧等待两路恢复、6已恢复。开关2为1时仍强制零扭矩。 */
enum { MOTOR_ZERO_IDLE=0, MOTOR_ZERO_QUEUED, MOTOR_ZERO_LEFT_TX, MOTOR_ZERO_GAP,
       MOTOR_ZERO_RIGHT_TX, MOTOR_ZERO_WAIT_FEEDBACK, MOTOR_ZERO_DONE };
extern volatile uint32_t g_motor_zero_state, g_motor_zero_switch;
extern volatile uint32_t g_motor_zero_left_sent, g_motor_zero_right_sent, g_motor_zero_recovered;
int motor_debug_receive_command(uint32_t mode_value, uint32_t switches, float left_torque, float right_torque);
uint32_t motor_zero_status_bit(uint32_t now);
void motor_bus_init(void);
void motor_bus_poll(void);
/* 电机安装朝向（用户确认）：左侧电机正转时下落，右侧电机正转时上升。
 * 左侧=JP3/USART3/ID126，右侧=JP2/USART6/ID127。
 * 本接口保留电机原始反馈符号，不在读取时做方向取反。
 */
int motor_get_feedback(unsigned right, MotorFrame *out);
#endif

