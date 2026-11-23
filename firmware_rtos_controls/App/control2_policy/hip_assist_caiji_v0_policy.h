/* Generated fixed-weight hip_assist_caiji_v0 assist policy; C99, no heap or file I/O. */
#ifndef HIP_ASSIST_CAIJI_V0_POLICY_H
#define HIP_ASSIST_CAIJI_V0_POLICY_H
#ifdef __cplusplus
extern "C" {
#endif
#define HIP_ASSIST_CAIJI_V0_INPUT_SIZE 151
#define HIP_ASSIST_CAIJI_V0_OUTPUT_SIZE 4
#define HIP_ASSIST_CAIJI_V0_HISTORY_LENGTH 25
#define HIP_ASSIST_CAIJI_V0_POLICY_RATE_HZ 100
/* Left/right order; radians, rad/s, Nm. Independent left/right torque, +/-18 Nm, four peak/duration actions, displacement-qualified and cadence-adaptive timing. */
typedef struct { float hidden_100[100]; float secondary_151[151]; } HipAssistCaijiV0Workspace;
typedef struct {
    float position_rad[25][2], velocity_rad_s[25][2], smoothed_torque_nm[2], pulse_state[44];
    unsigned int next_index;
} HipAssistCaijiV0History;
typedef struct {
    float positions[150][2], reversals[150][2], torques[150][2];
    float direction[2], extreme[2], peak_to_peak[2], reversal_count[2];
    unsigned int index, count, latched, tripped;
} HipAssistCaijiV0Guard;
typedef struct { float state[18], displacement[2], previous; } HipAssistCaijiV0Pulse;
typedef struct {
    HipAssistCaijiV0History history;
    HipAssistCaijiV0Workspace workspace;
    float observation[151], previous[2], filtered_velocity[2], raw_action[4];
    HipAssistCaijiV0Pulse pulses[2], motion_tracker;
    unsigned int consecutive_steps, walking_stage, stage_changed, quiet_ticks, standing;
    int last_direction;
    unsigned int precontact_profile;
    double last_timestamp;
    unsigned int timestamp_valid;
    unsigned int current_grade, requested_grade, zero_releasing, grade_changed;
    unsigned int ready, first;
    HipAssistCaijiV0Guard guard;
} HipAssistCaijiV0Controller;
void hip_assist_caiji_v0_history_reset(HipAssistCaijiV0History *, const float[2], const float[2]);
void hip_assist_caiji_v0_history_append(HipAssistCaijiV0History *, const float[2], const float[2], const float[2], const float[44]);
void hip_assist_caiji_v0_history_build_observation(const HipAssistCaijiV0History *, float[151]);
/* history_build zeros command slots 146..150. Low-level users must set effective
 * strength, requested strength, zero-release, stage and desired profile before inference.
 * Low-level forward requires finite inputs; returns four raw network actions. */
void hip_assist_caiji_v0_policy_forward(const float[151], float[4], HipAssistCaijiV0Workspace *);
/* reset: fills history. Return 0 success, -1 invalid inputs. */
int hip_assist_caiji_v0_reset(HipAssistCaijiV0Controller *, const float[2], const float[2], int grade, int precontact_profile);
/* Ordinary grades wait for bilateral idle/zero; zero cancels and ramps down. */
int hip_assist_caiji_v0_set_grade(HipAssistCaijiV0Controller *, int grade);
/* Profile 1..3 is the desired command; does not wait for torque zero. */
int hip_assist_caiji_v0_set_precontact_profile(HipAssistCaijiV0Controller *, int profile);
int hip_assist_caiji_v0_reward_precontact_profile(const HipAssistCaijiV0Controller *);
/* Timestamped entry checks monotonic intervals in [5,15] ms. Do not mix with step. */
int hip_assist_caiji_v0_step_timed(HipAssistCaijiV0Controller *, const float[2], const float[2], int, double, float[2]);
/* stop zeroes output and latches disabled until reset; does not write hardware. */
void hip_assist_caiji_v0_stop(HipAssistCaijiV0Controller *, float[2]);
/* Exactly once per 10 ms. No vx input. Returns 0 success, 1 disabled, -1 fault.
 * On fault/disabled, output is zero and controller is latched stopped.
 * Caller sends output to hardware, handles stale samples and deadlines.
 * State/workspace must be unique to a controller, not shared across threads.
 */
int hip_assist_caiji_v0_step(HipAssistCaijiV0Controller *, const float[2], const float[2], int, float[2]);
#ifdef __cplusplus
}
#endif
#endif
