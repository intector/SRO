/**
 ******************************************************************************
 * @file           : a2s_pid.c
 * @brief          : Reusable PID Controller Library Implementation
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
// Includes -------------------------------------------------------------------
#include "a2s_pid.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>
#include <string.h>

static const char *TAG = "A2S_PID";

// Private variables ----------------------------------------------------------
static bool g_initialized = false;

// static functions prototypes ------------------------------------------------
static uint32_t get_time_ms(void);
static float constrain_float(float value, float min, float max);
static void calculate_ziegler_nichols_pid(float ku, float tu, float *kp, float *ki, float *kd);
static void calculate_cohen_coon_pid(float kp_process, float tau, float theta, float *kp, float *ki, float *kd);
static esp_err_t analyze_autotune_data(a2s_pid_autotune_t *tuner);

// ----------------------------------------------------------------------------
// Core PID Functions
// ----------------------------------------------------------------------------

esp_err_t a2s_pid_init(void)
{
    if (g_initialized) {
        return ESP_OK;
    }

    g_initialized = true;
    ESP_LOGI(TAG, "PID library initialized");

    return ESP_OK;
}

esp_err_t a2s_pid_create(a2s_pid_controller_t *pid, a2s_pid_params_t *params)
{
    if (pid == NULL || params == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(pid, 0, sizeof(a2s_pid_controller_t));
    memcpy(&pid->params, params, sizeof(a2s_pid_params_t));

    pid->last_time_ms = get_time_ms();

    return ESP_OK;
}

esp_err_t a2s_pid_set_tunings(a2s_pid_controller_t *pid, float kp, float ki, float kd)
{
    if (pid == NULL || kp < 0.0f || ki < 0.0f || kd < 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    pid->params.kp = kp;
    pid->params.ki = ki;
    pid->params.kd = kd;

    if (pid->params.direction == A2S_PID_DIRECTION_REVERSE) {
        pid->params.kp = -pid->params.kp;
        pid->params.ki = -pid->params.ki;
        pid->params.kd = -pid->params.kd;
    }

    return ESP_OK;
}

esp_err_t a2s_pid_set_sample_time(a2s_pid_controller_t *pid, uint32_t sample_time_ms)
{
    if (pid == NULL || sample_time_ms == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    float ratio = (float)sample_time_ms / (float)pid->params.sample_time_ms;
    pid->params.ki *= ratio;
    pid->params.kd /= ratio;
    pid->params.sample_time_ms = sample_time_ms;

    return ESP_OK;
}

esp_err_t a2s_pid_set_output_limits(a2s_pid_controller_t *pid, float min, float max)
{
    if (pid == NULL || min >= max) {
        return ESP_ERR_INVALID_ARG;
    }

    pid->params.output_min = min;
    pid->params.output_max = max;

    // Clamp current output
    pid->output = constrain_float(pid->output, min, max);

    // Clamp integral term if anti-windup is enabled
    if (pid->params.enable_anti_windup) {
        pid->i_term = constrain_float(pid->i_term, min, max);
    }

    return ESP_OK;
}

esp_err_t a2s_pid_set_mode(a2s_pid_controller_t *pid, a2s_pid_mode_t mode)
{
    if (pid == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    bool mode_change = (mode != pid->params.mode);

    if (mode_change && mode == A2S_PID_MODE_AUTOMATIC) {
        // Initialize on transition to automatic
        pid->i_term     = pid->output;
        pid->last_error = pid->error;

        // Clamp integral term
        pid->i_term = constrain_float(pid->i_term,
                                      pid->params.output_min,
                                      pid->params.output_max);
    }

    pid->params.mode = mode;

    return ESP_OK;
}

esp_err_t a2s_pid_set_direction(a2s_pid_controller_t *pid, a2s_pid_direction_t direction)
{
    if (pid == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (direction != pid->params.direction) {
        pid->params.kp = -pid->params.kp;
        pid->params.ki = -pid->params.ki;
        pid->params.kd = -pid->params.kd;
    }

    pid->params.direction = direction;

    return ESP_OK;
}

esp_err_t a2s_pid_set_setpoint(a2s_pid_controller_t *pid, float setpoint)
{
    if (pid == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    pid->setpoint = setpoint;

    return ESP_OK;
}

float a2s_pid_compute(a2s_pid_controller_t *pid, float input)
{
    if (pid == NULL) {
        return 0.0f;
    }

    pid->input = input;

    if (pid->params.mode != A2S_PID_MODE_AUTOMATIC) {
        return pid->output;
    }

    uint32_t current_time = get_time_ms();
    uint32_t time_change  = current_time - pid->last_time_ms;

    if (time_change >= pid->params.sample_time_ms) {
        // Calculate error
        pid->error = pid->setpoint - pid->input;

        // Calculate proportional term
        pid->p_term = pid->params.kp * pid->error;

        // Calculate integral term
        pid->error_sum += pid->error;
        pid->i_term = pid->params.ki * pid->error_sum;

        // Apply anti-windup
        if (pid->params.enable_anti_windup) {
            pid->i_term = constrain_float(pid->i_term,
                                          pid->params.output_min,
                                          pid->params.output_max);
        }

        // Calculate derivative term with filtering
        float derivative = pid->error - pid->last_error;

        if (pid->params.derivative_filter > 0.0f) {
            // First-order low-pass filter
            pid->last_derivative = pid->last_derivative * pid->params.derivative_filter +
                                   derivative * (1.0f - pid->params.derivative_filter);
            pid->d_term = pid->params.kd * pid->last_derivative;
        }
        else {
            pid->d_term = pid->params.kd * derivative;
        }

        // Calculate total output
        pid->output = pid->p_term + pid->i_term + pid->d_term;

        // Clamp output
        pid->output = constrain_float(pid->output,
                                      pid->params.output_min,
                                      pid->params.output_max);

        // Save for next iteration
        pid->last_error   = pid->error;
        pid->last_time_ms = current_time;
    }

    return pid->output;
}

esp_err_t a2s_pid_reset(a2s_pid_controller_t *pid)
{
    if (pid == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    pid->error           = 0.0f;
    pid->last_error      = 0.0f;
    pid->error_sum       = 0.0f;
    pid->i_term          = 0.0f;
    pid->d_term          = 0.0f;
    pid->p_term          = 0.0f;
    pid->output          = 0.0f;
    pid->last_derivative = 0.0f;
    pid->last_time_ms    = get_time_ms();

    return ESP_OK;
}

esp_err_t a2s_pid_reset_bumpless(a2s_pid_controller_t *pid, float current_temp, float setpoint)
{
    if (pid == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    float current_error = setpoint - current_temp;

    // Reset integral and output - prevents windup carryover
    pid->error_sum = 0.0f;
    pid->i_term    = 0.0f;
    pid->output    = 0.0f;
    pid->p_term    = 0.0f;
    pid->d_term    = 0.0f;

    // BUMPLESS: Initialize last_error to current error
    // This makes (error - last_error) = 0 on first compute
    pid->error           = current_error;
    pid->last_error      = current_error;
    pid->last_derivative = 0.0f;
    pid->last_time_ms    = get_time_ms();

    return ESP_OK;
}

// ----------------------------------------------------------------------------
// Auto-Tune Functions (3-Phase: HEAT -> COAST -> COOL -> ANALYZE)
// ----------------------------------------------------------------------------

esp_err_t a2s_pid_autotune_init(a2s_pid_autotune_t *tuner)
{
    if (tuner == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(tuner, 0, sizeof(a2s_pid_autotune_t));
    tuner->tuning_active   = false;
    tuner->tuning_complete = false;
    tuner->phase           = A2S_AUTOTUNE_PHASE_IDLE;

    return ESP_OK;
}

esp_err_t a2s_pid_autotune_start(a2s_pid_autotune_t *tuner, a2s_pid_tune_method_t method, float target_temp)
{
    if (tuner == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (target_temp < 50.0f || target_temp > 280.0f) {
        ESP_LOGE(TAG, "Target temp %.1f out of range (50-280)", target_temp);
        return ESP_ERR_INVALID_ARG;
    }

    // Initialize state
    tuner->method           = method; // Kept for API compatibility, ignored
    tuner->tuning_active    = true;
    tuner->tuning_complete  = false;
    tuner->phase            = A2S_AUTOTUNE_PHASE_HEAT;
    tuner->start_time       = get_time_ms();
    tuner->phase_start_time = tuner->start_time;
    tuner->history_count    = 0;
    tuner->last_sample_time = 0;

    // Store test parameters
    tuner->target_temp   = target_temp;
    tuner->ambient_temp  = 0.0f; // Will be set on first update
    tuner->peak_temp     = 0.0f;
    tuner->heat_end_time = 0;

    // Initial outputs: 100% heater, door closed
    tuner->heater_output = 100.0f;
    tuner->door_target   = 0;

    // Clear calculated values
    tuner->dead_time     = 0.0f;
    tuner->time_constant = 0.0f;
    tuner->process_gain  = 0.0f;
    tuner->overshoot     = 0.0f;
    tuner->tuned_kp      = 0.0f;
    tuner->tuned_ki      = 0.0f;
    tuner->tuned_kd      = 0.0f;

    ESP_LOGI(TAG, "Auto-tune started: target=%.1f°C", target_temp);

    return ESP_OK;
}

esp_err_t a2s_pid_autotune_update(a2s_pid_autotune_t *tuner, float current_temp, float *heater_output)
{
    if (tuner == NULL || heater_output == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!tuner->tuning_active || tuner->tuning_complete) {
        return ESP_ERR_INVALID_STATE;
    }

    uint32_t current_time = get_time_ms();

    // Record ambient temperature on first call
    if (tuner->ambient_temp == 0.0f) {
        tuner->ambient_temp = current_temp;
        ESP_LOGI(TAG, "Ambient temperature recorded: %.1f°C", tuner->ambient_temp);
    }

    // Sample at 1 Hz (1 sample per second)
    if (current_time - tuner->last_sample_time >= 1000) {
        if (tuner->history_count < A2S_PID_HISTORY_SIZE) {
            tuner->temp_history[tuner->history_count] = current_temp;
            tuner->time_history[tuner->history_count] = current_time - tuner->start_time;
            tuner->history_count++;
        }
        tuner->last_sample_time = current_time;
    }

    // State machine
    switch (tuner->phase) {
        case A2S_AUTOTUNE_PHASE_HEAT:
            // 100% heater, door closed
            tuner->heater_output = 100.0f;
            tuner->door_target   = 0;

            // Transition when target reached
            if (current_temp >= tuner->target_temp) {
                tuner->heat_end_time    = current_time;
                tuner->phase            = A2S_AUTOTUNE_PHASE_COAST;
                tuner->phase_start_time = current_time;
                tuner->peak_temp        = current_temp;
                ESP_LOGI(TAG, "HEAT complete: target %.1f°C reached in %lu ms, entering COAST",
                         tuner->target_temp, current_time - tuner->start_time);
            }
            break;

        case A2S_AUTOTUNE_PHASE_COAST:
            // 0% heater, door closed - measure thermal overshoot
            tuner->heater_output = 0.0f;
            tuner->door_target   = 0;

            // Track peak temperature
            if (current_temp > tuner->peak_temp) {
                tuner->peak_temp = current_temp;
            }

            // Transition after coast duration
            if (current_time - tuner->phase_start_time >= A2S_AUTOTUNE_COAST_DURATION_MS) {
                tuner->overshoot        = tuner->peak_temp - tuner->target_temp;
                tuner->phase            = A2S_AUTOTUNE_PHASE_COOL;
                tuner->phase_start_time = current_time;
                ESP_LOGI(TAG, "COAST complete: peak=%.1f°C, overshoot=%.1f°C, entering COOL",
                         tuner->peak_temp, tuner->overshoot);
            }
            break;

        case A2S_AUTOTUNE_PHASE_COOL:
            // 0% heater, door 100% open
            tuner->heater_output = 0.0f;
            tuner->door_target   = 100;

            // Transition when cooled to ambient + 15°C
            // if (current_temp <= tuner->ambient_temp + 15.0f) {
            if (current_temp < 50.0f) {
                tuner->phase = A2S_AUTOTUNE_PHASE_ANALYZE;
                ESP_LOGI(TAG, "COOL complete: temp=%.1f°C, analyzing %d samples",
                         current_temp, tuner->history_count);
            }
            break;

        case A2S_AUTOTUNE_PHASE_ANALYZE:
            // Analyze collected data and calculate PID
            tuner->heater_output = 0.0f;
            tuner->door_target   = 100; // Close door after cooling
            analyze_autotune_data(tuner);
            break;

        default:
            return ESP_ERR_INVALID_STATE;
    }

    *heater_output = tuner->heater_output;
    return ESP_OK;
}

esp_err_t a2s_pid_autotune_stop(a2s_pid_autotune_t *tuner)
{
    if (tuner == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    tuner->tuning_active = false;
    tuner->phase         = A2S_AUTOTUNE_PHASE_IDLE;

    // Safety: ensure heater off
    tuner->heater_output = 0.0f;
    tuner->door_target   = 100; // Open door for safety on abort

    ESP_LOGI(TAG, "Auto-tune stopped");

    return ESP_OK;
}

bool a2s_pid_autotune_is_complete(a2s_pid_autotune_t *tuner)
{
    if (tuner == NULL) {
        return false;
    }

    return tuner->tuning_complete;
}

esp_err_t a2s_pid_autotune_get_results(a2s_pid_autotune_t *tuner, float *kp, float *ki, float *kd)
{
    if (tuner == NULL || kp == NULL || ki == NULL || kd == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!tuner->tuning_complete) {
        return ESP_ERR_INVALID_STATE;
    }

    *kp = tuner->tuned_kp;
    *ki = tuner->tuned_ki;
    *kd = tuner->tuned_kd;

    return ESP_OK;
}

esp_err_t a2s_pid_autotune_get_status(a2s_pid_autotune_t *tuner, a2s_autotune_status_t *status)
{
    if (tuner == NULL || status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    status->phase        = tuner->phase;
    status->target_temp  = tuner->target_temp;
    status->ambient_temp = tuner->ambient_temp;
    status->peak_temp    = tuner->peak_temp;

    // Get current temp from most recent sample
    if (tuner->history_count > 0) {
        status->current_temp = tuner->temp_history[tuner->history_count - 1];
    }
    else {
        status->current_temp = tuner->ambient_temp;
    }

    // Estimate progress based on phase
    switch (tuner->phase) {
        case A2S_AUTOTUNE_PHASE_IDLE:
            status->progress_percent = 0.0f;
            status->message          = "Idle";
            break;

        case A2S_AUTOTUNE_PHASE_HEAT:
            // Progress based on temperature rise
            if (tuner->target_temp > tuner->ambient_temp) {
                float rise               = status->current_temp - tuner->ambient_temp;
                float total              = tuner->target_temp - tuner->ambient_temp;
                status->progress_percent = (rise / total) * 50.0f; // 0-50%
            }
            else {
                status->progress_percent = 0.0f;
            }
            status->message = "Heating to target temperature";
            break;

        case A2S_AUTOTUNE_PHASE_COAST: {
            uint32_t elapsed         = get_time_ms() - tuner->phase_start_time;
            float coast_progress     = (float)elapsed / (float)A2S_AUTOTUNE_COAST_DURATION_MS;
            status->progress_percent = 50.0f + (coast_progress * 20.0f); // 50-70%
        }
            status->message = "Measuring thermal overshoot";
            break;

        case A2S_AUTOTUNE_PHASE_COOL:
            // Progress based on cooling
            if (tuner->peak_temp > tuner->ambient_temp + 15.0f) {
                float cooled             = tuner->peak_temp - status->current_temp;
                float total              = tuner->peak_temp - (tuner->ambient_temp + 15.0f);
                status->progress_percent = 70.0f + (cooled / total) * 25.0f; // 70-95%
            }
            else {
                status->progress_percent = 95.0f;
            }
            status->message = "Cooling down";
            break;

        case A2S_AUTOTUNE_PHASE_ANALYZE:
            status->progress_percent = 100.0f;
            status->message          = tuner->tuning_complete ? "Complete" : "Analyzing data";
            break;

        default:
            status->progress_percent = 0.0f;
            status->message          = "Unknown state";
            break;
    }

    return ESP_OK;
}

uint8_t a2s_pid_autotune_get_door_target(a2s_pid_autotune_t *tuner)
{
    if (tuner == NULL) {
        return 0;
    }
    return tuner->door_target;
}

// ----------------------------------------------------------------------------
// Utility Functions
// ----------------------------------------------------------------------------

esp_err_t a2s_pid_get_status(a2s_pid_controller_t *pid, float *p_term, float *i_term, float *d_term)
{
    if (pid == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (p_term != NULL)
        *p_term = pid->p_term;
    if (i_term != NULL)
        *i_term = pid->i_term;
    if (d_term != NULL)
        *d_term = pid->d_term;

    return ESP_OK;
}

const char *a2s_pid_get_mode_string(a2s_pid_mode_t mode)
{
    switch (mode) {
        case A2S_PID_MODE_MANUAL:
            return "MANUAL";
        case A2S_PID_MODE_AUTOMATIC:
            return "AUTOMATIC";
        default:
            return "UNKNOWN";
    }
}

const char *a2s_pid_get_direction_string(a2s_pid_direction_t direction)
{
    switch (direction) {
        case A2S_PID_DIRECTION_DIRECT:
            return "DIRECT";
        case A2S_PID_DIRECTION_REVERSE:
            return "REVERSE";
        default:
            return "UNKNOWN";
    }
}

const char *a2s_pid_get_tune_method_string(a2s_pid_tune_method_t method)
{
    switch (method) {
        case A2S_PID_TUNE_NONE:
            return "NONE";
        case A2S_PID_TUNE_STEP_RESPONSE:
            return "STEP_RESPONSE";
        default:
            return "UNKNOWN";
    }
}

const char *a2s_pid_get_autotune_phase_string(a2s_autotune_phase_t phase)
{
    switch (phase) {
        case A2S_AUTOTUNE_PHASE_IDLE:
            return "idle";
        case A2S_AUTOTUNE_PHASE_HEAT:
            return "heat";
        case A2S_AUTOTUNE_PHASE_COAST:
            return "coast";
        case A2S_AUTOTUNE_PHASE_COOL:
            return "cool";
        case A2S_AUTOTUNE_PHASE_ANALYZE:
            return "analyze";
        default:
            return "unknown";
    }
}

// ----------------------------------------------------------------------------
// Preset Tuning Methods
// ----------------------------------------------------------------------------

esp_err_t a2s_pid_apply_ziegler_nichols(a2s_pid_controller_t *pid, float ku, float tu)
{
    if (pid == NULL || ku <= 0.0f || tu <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    float kp, ki, kd;
    calculate_ziegler_nichols_pid(ku, tu, &kp, &ki, &kd);

    return a2s_pid_set_tunings(pid, kp, ki, kd);
}

esp_err_t a2s_pid_apply_cohen_coon(a2s_pid_controller_t *pid, float process_gain, float time_constant, float dead_time)
{
    if (pid == NULL || process_gain <= 0.0f || time_constant <= 0.0f || dead_time < 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    float kp, ki, kd;
    calculate_cohen_coon_pid(process_gain, time_constant, dead_time, &kp, &ki, &kd);

    return a2s_pid_set_tunings(pid, kp, ki, kd);
}

esp_err_t a2s_pid_apply_pessen_integral(a2s_pid_controller_t *pid, float ku, float tu)
{
    if (pid == NULL || ku <= 0.0f || tu <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    float kp = 0.7f * ku;
    float ki = (1.75f * ku) / tu;
    float kd = (0.105f * ku * tu);

    return a2s_pid_set_tunings(pid, kp, ki, kd);
}

esp_err_t a2s_pid_apply_some_overshoot(a2s_pid_controller_t *pid, float ku, float tu)
{
    if (pid == NULL || ku <= 0.0f || tu <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    float kp = 0.33f * ku;
    float ki = (0.66f * ku) / tu;
    float kd = (0.11f * ku * tu);

    return a2s_pid_set_tunings(pid, kp, ki, kd);
}

esp_err_t a2s_pid_apply_no_overshoot(a2s_pid_controller_t *pid, float ku, float tu)
{
    if (pid == NULL || ku <= 0.0f || tu <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    float kp = 0.2f * ku;
    float ki = (0.4f * ku) / tu;
    float kd = (0.066f * ku * tu);

    return a2s_pid_set_tunings(pid, kp, ki, kd);
}

// ----------------------------------------------------------------------------
// Static Helper Functions
// ----------------------------------------------------------------------------

static uint32_t get_time_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static float constrain_float(float value, float min, float max)
{
    if (value < min)
        return min;
    if (value > max)
        return max;
    return value;
}

static void calculate_ziegler_nichols_pid(float ku, float tu, float *kp, float *ki, float *kd)
{
    // Classic Ziegler-Nichols tuning
    *kp = 0.6f * ku;
    *ki = (1.2f * ku) / tu;
    *kd = (0.075f * ku * tu);
}

static void calculate_cohen_coon_pid(float kp_process, float tau, float theta, float *kp, float *ki, float *kd)
{
    // Cohen-Coon tuning formulas for PID controller
    // kp_process = process gain (K)
    // tau = time constant
    // theta = dead time

    float ratio = theta / tau;

    // Cohen-Coon formulas
    *kp      = (1.0f / kp_process) * (1.35f + (0.25f * ratio));

    float ti = theta * (2.5f - 2.0f * ratio) / (1.0f - 0.39f * ratio);
    *ki      = *kp / ti;

    float td = theta * 0.37f * (1.0f - ratio) / (1.0f - 0.81f * ratio);
    *kd      = *kp * td;

    ESP_LOGI(TAG, "Cohen-Coon: K=%.4f, tau=%.1f, theta=%.1f -> Kp=%.4f, Ki=%.6f, Kd=%.2f",
             kp_process, tau, theta, *kp, *ki, *kd);
}

static esp_err_t analyze_autotune_data(a2s_pid_autotune_t *tuner)
{
    if (tuner->history_count < 30) {
        ESP_LOGE(TAG, "Insufficient data: only %d samples", tuner->history_count);
        tuner->tuning_complete = true;
        tuner->tuning_active   = false;
        return ESP_ERR_INVALID_STATE;
    }

    float initial_temp = tuner->ambient_temp;
    float final_temp   = tuner->target_temp;
    float delta_temp   = final_temp - initial_temp;

    if (delta_temp < 10.0f) {
        ESP_LOGE(TAG, "Temperature rise too small: %.1f°C", delta_temp);
        tuner->tuning_complete = true;
        tuner->tuning_active   = false;
        return ESP_ERR_INVALID_STATE;
    }

    // Calculate process gain: K = ΔT / Δ(heater %)
    // We applied 100% heater, so K = delta_temp / 100
    tuner->process_gain = delta_temp / 100.0f;

    // Find dead time: time to reach 5% of final temperature rise
    float target_5pct = initial_temp + 0.05f * delta_temp;
    tuner->dead_time  = 0.0f;

    for (uint16_t i = 0; i < tuner->history_count; i++) {
        if (tuner->temp_history[i] >= target_5pct) {
            tuner->dead_time = (float)tuner->time_history[i] / 1000.0f;
            break;
        }
    }

    // Find time constant: time to reach 63.2% of final value
    float target_63pct   = initial_temp + 0.632f * delta_temp;
    tuner->time_constant = 0.0f;

    for (uint16_t i = 0; i < tuner->history_count; i++) {
        if (tuner->temp_history[i] >= target_63pct) {
            tuner->time_constant = (float)tuner->time_history[i] / 1000.0f;
            break;
        }
    }

    // Adjust time constant to be relative to dead time
    if (tuner->time_constant > tuner->dead_time) {
        tuner->time_constant -= tuner->dead_time;
    }

    ESP_LOGI(TAG, "System characteristics: K=%.4f, dead_time=%.1fs, tau=%.1fs, overshoot=%.1f°C",
             tuner->process_gain, tuner->dead_time, tuner->time_constant, tuner->overshoot);

    // Validate measurements
    if (tuner->dead_time < 1.0f) {
        ESP_LOGW(TAG, "Dead time very short (%.1fs), using minimum of 5s", tuner->dead_time);
        tuner->dead_time = 5.0f;
    }

    if (tuner->time_constant < 10.0f) {
        ESP_LOGW(TAG, "Time constant very short (%.1fs), using minimum of 30s", tuner->time_constant);
        tuner->time_constant = 30.0f;
    }

    // Calculate PID gains using Cohen-Coon
    calculate_cohen_coon_pid(tuner->process_gain,
                             tuner->time_constant,
                             tuner->dead_time,
                             &tuner->tuned_kp,
                             &tuner->tuned_ki,
                             &tuner->tuned_kd);

    // Sanity check and clamp PID values
    if (tuner->tuned_kp < 0.1f)
        tuner->tuned_kp = 0.1f;
    if (tuner->tuned_kp > 10.0f)
        tuner->tuned_kp = 10.0f;

    if (tuner->tuned_ki < 0.001f)
        tuner->tuned_ki = 0.001f;
    if (tuner->tuned_ki > 1.0f)
        tuner->tuned_ki = 1.0f;

    if (tuner->tuned_kd < 0.0f)
        tuner->tuned_kd = 0.0f;
    if (tuner->tuned_kd > 100.0f)
        tuner->tuned_kd = 100.0f;

    ESP_LOGI(TAG, "Auto-tune complete: Kp=%.4f, Ki=%.6f, Kd=%.2f",
             tuner->tuned_kp, tuner->tuned_ki, tuner->tuned_kd);

    tuner->tuning_complete = true;
    tuner->tuning_active   = false;
    tuner->phase           = A2S_AUTOTUNE_PHASE_IDLE;

    return ESP_OK;
}

// -------------------------------------------------------------------------
// end of a2s_pid.c
// -------------------------------------------------------------------------
