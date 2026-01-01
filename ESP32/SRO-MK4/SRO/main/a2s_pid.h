/**
 ******************************************************************************
 * @file           : a2s_pid.h
 * @brief          : Reusable PID Controller Library Header
 ******************************************************************************
 *
 * Generic PID Controller with Auto-Tuning
 * Copyright (c) 2024 Intector Inc.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

// Define to prevent recursive inclusion --------------------------------------
#ifndef __A2S_PID_H
#define __A2S_PID_H

#ifdef __cplusplus
extern "C" {
#endif

// Includes -------------------------------------------------------------------
#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

// ----------------------------------------------------------------------------
// Configuration Constants
// ----------------------------------------------------------------------------

#define A2S_PID_HISTORY_SIZE           3000  // Sample history for auto-tune (1 sample/sec = 50 min)
#define A2S_AUTOTUNE_COAST_DURATION_MS 30000 // 30 seconds coast phase

// ----------------------------------------------------------------------------
// Type Definitions
// ----------------------------------------------------------------------------

typedef enum
{
    A2S_PID_MODE_MANUAL = 0,
    A2S_PID_MODE_AUTOMATIC
} a2s_pid_mode_t;

typedef enum
{
    A2S_PID_DIRECTION_DIRECT = 0, // Output increases when error is positive
    A2S_PID_DIRECTION_REVERSE     // Output decreases when error is positive
} a2s_pid_direction_t;

// Simplified: Only one auto-tune method now
typedef enum
{
    A2S_PID_TUNE_NONE = 0,
    A2S_PID_TUNE_STEP_RESPONSE // Keep for API compatibility, but it's the only method
} a2s_pid_tune_method_t;

// NEW: Auto-tune phase state machine
typedef enum
{
    A2S_AUTOTUNE_PHASE_IDLE = 0,
    A2S_AUTOTUNE_PHASE_HEAT,   // 100% heater, door closed, until target temp
    A2S_AUTOTUNE_PHASE_COAST,  // 0% heater, door closed, 30 seconds (measure overshoot)
    A2S_AUTOTUNE_PHASE_COOL,   // 0% heater, door 100% open, until ambient+15°C
    A2S_AUTOTUNE_PHASE_ANALYZE // Analyze data and calculate PID
} a2s_autotune_phase_t;

typedef struct
{
    // PID gains
    float kp;
    float ki;
    float kd;

    // Output limits
    float output_min;
    float output_max;

    // Sample time
    uint32_t sample_time_ms;

    // Operating mode
    a2s_pid_mode_t mode;
    a2s_pid_direction_t direction;

    // Anti-windup
    bool enable_anti_windup;

    // Derivative filtering (0.0 = no filter, 1.0 = heavy filter)
    float derivative_filter;

} a2s_pid_params_t;

typedef struct
{
    // Current state
    float setpoint;
    float input;
    float output;

    // PID terms
    float p_term;
    float i_term;
    float d_term;

    // Error tracking
    float error;
    float last_error;
    float error_sum;

    // Timing
    uint32_t last_time_ms;

    // Filtered derivative
    float last_derivative;

    // Parameters
    a2s_pid_params_t params;

} a2s_pid_controller_t;

// NEW: Simplified autotune structure for 3-phase test
typedef struct
{
    // State
    bool tuning_active;
    bool tuning_complete;
    a2s_autotune_phase_t phase;
    uint32_t start_time;
    uint32_t phase_start_time;

    // Test parameters
    float ambient_temp; // Recorded at test start
    float target_temp;  // User-specified target

    // Phase-specific data
    float peak_temp;        // Max temp during coast phase
    uint32_t heat_end_time; // When heating phase ended (target reached)

    // Door control output (for SRO_TemperatureControl to read)
    uint8_t door_target; // 0-100% door position
    float heater_output; // 0-100% heater duty

    // Sample history (1 sample per second during test)
    float temp_history[A2S_PID_HISTORY_SIZE];
    uint32_t time_history[A2S_PID_HISTORY_SIZE];
    uint16_t history_count;
    uint32_t last_sample_time;

    // Calculated system characteristics
    float dead_time;     // θ (seconds) - delay before temp starts rising
    float time_constant; // τ (seconds) - time to 63.2% of final value
    float process_gain;  // K (°C per % heater output)
    float overshoot;     // Peak - Target (°C)

    // Calculated PID gains
    float tuned_kp;
    float tuned_ki;
    float tuned_kd;

    // Legacy field for API compatibility
    a2s_pid_tune_method_t method;

} a2s_pid_autotune_t;

// NEW: Status info for WebSocket reporting
typedef struct
{
    a2s_autotune_phase_t phase;
    float progress_percent; // 0-100 estimated progress
    float current_temp;
    float target_temp;
    float ambient_temp;
    float peak_temp;
    const char *message;
} a2s_autotune_status_t;

// ----------------------------------------------------------------------------
// Public Function Prototypes - Core PID Functions
// ----------------------------------------------------------------------------

esp_err_t a2s_pid_init(void);
esp_err_t a2s_pid_create(a2s_pid_controller_t *pid, a2s_pid_params_t *params);
esp_err_t a2s_pid_set_tunings(a2s_pid_controller_t *pid, float kp, float ki, float kd);
esp_err_t a2s_pid_set_sample_time(a2s_pid_controller_t *pid, uint32_t sample_time_ms);
esp_err_t a2s_pid_set_output_limits(a2s_pid_controller_t *pid, float min, float max);
esp_err_t a2s_pid_set_mode(a2s_pid_controller_t *pid, a2s_pid_mode_t mode);
esp_err_t a2s_pid_set_direction(a2s_pid_controller_t *pid, a2s_pid_direction_t direction);
esp_err_t a2s_pid_set_setpoint(a2s_pid_controller_t *pid, float setpoint);
float a2s_pid_compute(a2s_pid_controller_t *pid, float input);
esp_err_t a2s_pid_reset(a2s_pid_controller_t *pid);
esp_err_t a2s_pid_reset_bumpless(a2s_pid_controller_t *pid, float current_temp, float setpoint);

// ----------------------------------------------------------------------------
// Public Function Prototypes - Auto-Tune Functions (Simplified API)
// ----------------------------------------------------------------------------

esp_err_t a2s_pid_autotune_init(a2s_pid_autotune_t *tuner);

// Start autotune - only needs target temperature now
esp_err_t a2s_pid_autotune_start(a2s_pid_autotune_t *tuner, a2s_pid_tune_method_t method, float target_temp);

// Update autotune state machine - call every 100ms from temperature control task
// Sets tuner->heater_output and tuner->door_target for caller to apply
esp_err_t a2s_pid_autotune_update(a2s_pid_autotune_t *tuner, float current_temp, float *heater_output);

esp_err_t a2s_pid_autotune_stop(a2s_pid_autotune_t *tuner);
bool a2s_pid_autotune_is_complete(a2s_pid_autotune_t *tuner);

esp_err_t a2s_pid_autotune_get_results(a2s_pid_autotune_t *tuner, float *kp, float *ki, float *kd);

// NEW: Get current autotune status for WebSocket reporting
esp_err_t a2s_pid_autotune_get_status(a2s_pid_autotune_t *tuner, a2s_autotune_status_t *status);

// NEW: Get door position target (caller applies to servo)
uint8_t a2s_pid_autotune_get_door_target(a2s_pid_autotune_t *tuner);

// ----------------------------------------------------------------------------
// Public Function Prototypes - Utility Functions
// ----------------------------------------------------------------------------

esp_err_t a2s_pid_get_status(a2s_pid_controller_t *pid, float *p_term, float *i_term, float *d_term);

const char *a2s_pid_get_mode_string(a2s_pid_mode_t mode);
const char *a2s_pid_get_direction_string(a2s_pid_direction_t direction);
const char *a2s_pid_get_tune_method_string(a2s_pid_tune_method_t method);
const char *a2s_pid_get_autotune_phase_string(a2s_autotune_phase_t phase);

// ----------------------------------------------------------------------------
// Public Function Prototypes - Preset Tuning (kept for manual use)
// ----------------------------------------------------------------------------

esp_err_t a2s_pid_apply_ziegler_nichols(a2s_pid_controller_t *pid, float ku, float tu);
esp_err_t a2s_pid_apply_cohen_coon(a2s_pid_controller_t *pid, float process_gain, float time_constant, float dead_time);
esp_err_t a2s_pid_apply_pessen_integral(a2s_pid_controller_t *pid, float ku, float tu);
esp_err_t a2s_pid_apply_some_overshoot(a2s_pid_controller_t *pid, float ku, float tu);
esp_err_t a2s_pid_apply_no_overshoot(a2s_pid_controller_t *pid, float ku, float tu);



#ifdef __cplusplus
}
#endif

#endif // __A2S_PID_H
