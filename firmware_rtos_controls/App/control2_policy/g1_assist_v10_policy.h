/* Generated fixed-weight v10 assist policy; C99, no heap or file I/O. */
#ifndef G1_ASSIST_V10_POLICY_H
#define G1_ASSIST_V10_POLICY_H
#ifdef __cplusplus
extern "C" {
#endif
#define G1_ASSIST_V10_INPUT_SIZE 800
#define G1_ASSIST_V10_OUTPUT_SIZE 2
#define G1_ASSIST_V10_HISTORY_LENGTH 25
#define G1_ASSIST_V10_POLICY_RATE_HZ 100
/* Left/right order; radians, rad/s, Nm. Paired torque [T,-T], +/-10 Nm, latched peak/duration, displacement-qualified and cadence-adaptive timing. */
typedef struct { float hidden_256[256]; float secondary_800[800]; } G1AssistV10Workspace;
typedef struct {
    float position_rad[25][2], velocity_rad_s[25][2], smoothed_torque_nm[25][2], pulse_state[25][26];
    unsigned int next_index;
} G1AssistV10History;
typedef struct {
    float positions[150][2], reversals[150][2], torques[150];
    float direction[2], extreme[2], peak_to_peak[2], reversal_count[2], previous;
    unsigned int index, count, latched, tripped;
} G1AssistV10Guard;
typedef struct {
    G1AssistV10History history;
    G1AssistV10Workspace workspace;
    float observation[800], previous[2], filtered_velocity[2], pulse[18], displacement[2];
    unsigned int ready, first;
    G1AssistV10Guard guard;
} G1AssistV10Controller;
void g1_assist_v10_history_reset(G1AssistV10History *, const float[2], const float[2]);
void g1_assist_v10_history_append(G1AssistV10History *, const float[2], const float[2], const float[2], const float[26]);
void g1_assist_v10_history_build_observation(const G1AssistV10History *, float[800]);
/* Low-level forward requires finite inputs; returns raw network actions. */
void g1_assist_v10_policy_forward(const float[800], float[2], G1AssistV10Workspace *);
/* reset: fills history. Return 0 success, -1 invalid inputs. */
int g1_assist_v10_reset(G1AssistV10Controller *, const float[2], const float[2]);
/* stop zeroes output and latches disabled until reset; does not write hardware. */
void g1_assist_v10_stop(G1AssistV10Controller *, float[2]);
/* Exactly once per 10 ms. No vx input. Returns 0 success, 1 disabled, -1 fault.
 * On fault/disabled, output is zero and controller is latched stopped.
 * Caller sends output to hardware, handles stale samples and deadlines.
 * State/workspace must be unique to a controller, not shared across threads.
 */
int g1_assist_v10_step(G1AssistV10Controller *, const float[2], const float[2], int, float[2]);
#ifdef __cplusplus
}
#endif
#endif
