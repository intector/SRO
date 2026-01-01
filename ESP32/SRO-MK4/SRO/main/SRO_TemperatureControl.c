/**
 ******************************************************************************
 * @file           : SRO_TemperatureControl.c
 * @brief          : Temperature Control Task Implementation
 ******************************************************************************
 *
 * SRO - Soldering Reflow Oven
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
#include "SRO_TemperatureControl.h"
#include "a2s_max6675.h"
#include "a2s_ssr.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "a2s_ssr.h"

static const char *TAG = "SRO_TEMP_CONTROL";

// ----------------------------------------------------------------------------
// global duty variable (atomic access via volatile)
// ----------------------------------------------------------------------------
static volatile float g_heater_duty_percent = 0.0f;

// ----------------------------------------------------------------------------
// Private variables
// ----------------------------------------------------------------------------

// Temperature control state
static sro_temp_control_state_t g_temp_state = {0};
// PID controllers (dual: heat and cool) and autotuner
static a2s_pid_controller_t g_pid_heat;
static a2s_pid_controller_t g_pid_cool;
static a2s_pid_autotune_t g_autotune;

static bool g_boost_mode = false;

// Initialization flag
static bool g_initialized = false;

// ----------------------------------------------------------------------------
// static functions prototypes
// ----------------------------------------------------------------------------

static esp_err_t load_pid_params_from_nvs(float *kp, float *ki, float *kd);
static esp_err_t save_pid_params_to_nvs(float kp, float ki, float kd);
static void read_temperature_sensor(void);
static void pid_loop(void);
static float pid_loop_heating(void);
static float pid_loop_cooling(void);
static void process_tuning_mode(void);
// ----------------------------------------------------------------------------
// functions
// ----------------------------------------------------------------------------

esp_err_t SRO_TemperatureControl_Init(void)
{
    if (g_initialized) {
        return ESP_OK;
    }

    // Initialize temperature control state
    memset(&g_temp_state, 0, sizeof(sro_temp_control_state_t));
    g_temp_state.current_temp     = 0.0f;
    g_temp_state.target_temp      = 0.0f;
    g_temp_state.pid_output       = 0.0f;
    g_temp_state.door_output      = 0.0f;
    g_temp_state.control_mode     = SRO_CONTROL_MODE_IDLE;
    g_temp_state.heater_on        = false;
    g_temp_state.pid_active       = false;
    g_temp_state.mode             = SRO_TEMP_MODE_IDLE;
    g_temp_state.tuning_active    = false;
    g_temp_state.tuning_complete  = false;
    g_temp_state.last_update_time = esp_timer_get_time() / 1000;
    g_temp_state.feedforward      = 0.0f;

    // Try to load saved PID parameters from NVS
    float saved_kp = SRO_PID_DEFAULT_KP;
    float saved_ki = SRO_PID_DEFAULT_KI;
    float saved_kd = SRO_PID_DEFAULT_KD;

    load_pid_params_from_nvs(&saved_kp, &saved_ki, &saved_kd);

    // Create PID controller with loaded/default parameters
    a2s_pid_params_t pid_params = {
        .kp                 = saved_kp,
        .ki                 = saved_ki,
        .kd                 = saved_kd,
        .output_min         = SRO_PID_OUTPUT_MIN,
        .output_max         = SRO_PID_OUTPUT_MAX,
        .sample_time_ms     = SRO_PID_SAMPLE_TIME_MS,
        .mode               = A2S_PID_MODE_AUTOMATIC,
        .direction          = A2S_PID_DIRECTION_DIRECT,
        .enable_anti_windup = true,
        .derivative_filter  = SRO_PID_DERIVATIVE_FILTER};

    esp_err_t err = a2s_pid_create(&g_pid_heat, &pid_params);
    if (err != ESP_OK) {
        return err;
    }

    // Create cooling PID with scaled parameters
    a2s_pid_params_t cool_params = {
        .kp                 = saved_kp * SRO_COOL_PID_KP_SCALE,
        .ki                 = saved_ki * SRO_COOL_PID_KI_SCALE,
        .kd                 = saved_kd * SRO_COOL_PID_KD_SCALE,
        .output_min         = SRO_PID_OUTPUT_MIN,
        .output_max         = SRO_PID_OUTPUT_MAX,
        .sample_time_ms     = SRO_PID_SAMPLE_TIME_MS,
        .mode               = A2S_PID_MODE_AUTOMATIC,
        .direction          = A2S_PID_DIRECTION_DIRECT, // We'll negate error instead
        .enable_anti_windup = true,
        .derivative_filter  = SRO_PID_DERIVATIVE_FILTER};

    err = a2s_pid_create(&g_pid_cool, &cool_params);
    if (err != ESP_OK) {
        return err;
    }
    
    // Store PID parameters in state
    g_temp_state.kp = saved_kp;
    g_temp_state.ki = saved_ki;
    g_temp_state.kd = saved_kd;

    // Initialize autotune structure
    err = a2s_pid_autotune_init(&g_autotune);
    if (err != ESP_OK) {
        return err;
    }

    g_initialized = true;

    return ESP_OK;
}

void temperature_control_task(void *pvParameters)
{
    const char *TAG = "TEMP_CONTROL_TASK";

    // -------------------------------------------------------------------------
    // STARTUP GATE - Wait for system initialization
    // -------------------------------------------------------------------------
    ESP_LOGI(TAG, "Task created, waiting for initialization...");

    xEventGroupWaitBits(sro_ctrl1_events,
                        SRO_CE1_TASKS_RELEASE,
                        pdFALSE, // Don't clear the bit
                        pdTRUE,
                        portMAX_DELAY);

    ESP_LOGI(TAG, "Released! Starting operation...");

    // -------------------------------------------------------------------------
    // MAIN TASK LOOP - Time-based, runs every 100ms
    // -------------------------------------------------------------------------
    TickType_t last_wake_time = xTaskGetTickCount();
    
    while (1) {

        // Check for suspend command FIRST
        EventBits_t bits = xEventGroupGetBits(sro_ctrl2_events);
        if (bits & SRO_CE2_SUSPEND_TEMP) {
            ESP_LOGI("TEMP_CTRL", "SUSPENDING");

            a2s_ssr_set_heater(false);
            xEventGroupSetBits(sro_status2_events, SRO_SE2_TEMP_SUSPENDED);

            // Wait for resume (flag to be CLEARED)
            while (xEventGroupGetBits(sro_ctrl2_events) & SRO_CE2_SUSPEND_TEMP) {
                vTaskDelay(pdMS_TO_TICKS(100));
            }

            xEventGroupClearBits(sro_status2_events, SRO_SE2_TEMP_SUSPENDED);
            ESP_LOGI("TEMP_CTRL", "RESUMED");
            continue;
        }
        
        // ALWAYS read temperature sensor - no dependencies
        read_temperature_sensor();

        // Process based on current mode
        switch (g_temp_state.mode) {
            case SRO_TEMP_MODE_NORMAL:
                pid_loop();
                break;

            case SRO_TEMP_MODE_TUNING:
                process_tuning_mode();
                break;

            case SRO_TEMP_MODE_IDLE:
            default:
                // Ensure heater is off when idle
                if (g_temp_state.heater_on) {
                    a2s_ssr_set_heater(false);
                    g_temp_state.heater_on = false;
                    xEventGroupClearBits(sro_status1_events, SRO_SE1_HEATER_ON);
                }
                break;
        }

        if (g_temp_state.heater_on || g_temp_state.current_temp > 50.0f) {
            a2s_ssr_set_fan(A2S_SSR_STATE_ON); // Forced ON, ignores request
        }
        else {
            // Only here is the request applied
            a2s_ssr_set_fan(g_temp_state.manual_fan_request ? A2S_SSR_STATE_ON : A2S_SSR_STATE_OFF);
        } 
        
        // Signal temperature update complete (for tasks that need notification)
        xEventGroupSetBits(sro_ctrl1_events, SRO_CE1_TEMP_UPDATE_DONE);

        a2s_apa102_color_t color = a2s_apa102_temperature_to_color(30.0f, 100.0f, g_temp_state.current_temp);
        a2s_apa102_set_effect(4, A2S_APA102_STATUS_SOLID, color, 0.20f);
        
        // Wait exactly 100ms - temperature reading is ALWAYS 10Hz
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(100));
    }
}

void SRO_Heater_SetDuty(float percent)
{
    g_heater_duty_percent = fminf(fmaxf(percent, 0.0f), 100.0f);
}

void heater_pwm_task(void *pvParameters)
{
    const char *TAG = "HEATER_PWM";
    uint8_t cycle_counter = 0;

    // -------------------------------------------------------------------------
    // STARTUP GATE - Wait for system initialization
    // -------------------------------------------------------------------------
    ESP_LOGI(TAG, "Task created, waiting for initialization...");

    xEventGroupWaitBits(sro_ctrl1_events,
                        SRO_CE1_TASKS_RELEASE,
                        pdFALSE, // Don't clear the bit
                        pdTRUE,
                        portMAX_DELAY);

    ESP_LOGI(TAG, "Released! Starting operation...");

    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        cycle_counter++;
        if (cycle_counter >= SRO_HEATER_PWM_PERIOD_TICKS) {
            cycle_counter = 0;
        }

        uint8_t duty_ticks = (uint8_t)(g_heater_duty_percent * SRO_HEATER_PWM_PERIOD_TICKS / 100.0f);
        bool heater_on     = (cycle_counter < duty_ticks);
        a2s_ssr_set_heater(heater_on);

        // Update state for status reporting
        if (heater_on != g_temp_state.heater_on) {
            g_temp_state.heater_on = heater_on;
            if (heater_on) {
                xEventGroupSetBits(sro_status1_events, SRO_SE1_HEATER_ON);
            }
            else {
                xEventGroupClearBits(sro_status1_events, SRO_SE1_HEATER_ON);
            }
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SRO_HEATER_PWM_TICK_MS));
    }
}

// ----------------------------------------------------------------------------
// Public API Functions - Control
// ----------------------------------------------------------------------------

esp_err_t SRO_TemperatureControl_SetTarget(float target_temp)
{
    if (!g_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    g_temp_state.target_temp = target_temp;
    a2s_pid_set_setpoint(&g_pid_heat, target_temp);
    a2s_pid_set_setpoint(&g_pid_cool, target_temp);

    return ESP_OK;
}

esp_err_t SRO_TemperatureControl_EnablePID(bool enable)
{
    if (!g_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Don't allow enabling if we're tuning
    if (enable && g_temp_state.mode == SRO_TEMP_MODE_TUNING) {
        return ESP_ERR_INVALID_STATE;
    }

    g_temp_state.pid_active = enable;

    if (enable) {
        g_temp_state.mode = SRO_TEMP_MODE_NORMAL;
        xEventGroupSetBits(sro_status1_events, SRO_SE1_PID_ACTIVE);
    }
    else {
        g_temp_state.mode = SRO_TEMP_MODE_IDLE;
        xEventGroupClearBits(sro_status1_events, SRO_SE1_PID_ACTIVE);

        // Reset heater duty cycle
        g_heater_duty_percent = 0.0f;

        // Turn off heater when PID disabled
        if (g_temp_state.heater_on) {
            a2s_ssr_set_heater(false);
            g_temp_state.heater_on = false;
            xEventGroupClearBits(sro_status1_events, SRO_SE1_HEATER_ON);
        }
    }

    return ESP_OK;
}

esp_err_t SRO_TemperatureControl_GetState(sro_temp_control_state_t *state)
{
    if (!g_initialized || state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memcpy(state, &g_temp_state, sizeof(sro_temp_control_state_t));
    return ESP_OK;
}

// ----------------------------------------------------------------------------
// Public API Functions - PID Tuning
// ----------------------------------------------------------------------------

esp_err_t SRO_TemperatureControl_SetPIDParams(float kp, float ki, float kd)
{
    if (!g_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Don't allow changing parameters while tuning
    if (g_temp_state.mode == SRO_TEMP_MODE_TUNING) {
        return ESP_ERR_INVALID_STATE;
    }

    // Update heating PID
    esp_err_t err = a2s_pid_set_tunings(&g_pid_heat, kp, ki, kd);
    if (err != ESP_OK) {
        return err;
    }

    // Update cooling PID with scaled values
    err = a2s_pid_set_tunings(&g_pid_cool,
                              kp * SRO_COOL_PID_KP_SCALE,
                              ki * SRO_COOL_PID_KI_SCALE,
                              kd * SRO_COOL_PID_KD_SCALE);
    if (err != ESP_OK) {
        return err;
    }

    // Update state
    g_temp_state.kp = kp;
    g_temp_state.ki = ki;
    g_temp_state.kd = kd;

    save_pid_params_to_nvs(kp, ki, kd);

    return ESP_OK;
}

esp_err_t SRO_TemperatureControl_GetPIDParams(float *kp, float *ki, float *kd)
{
    if (!g_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (kp == NULL || ki == NULL || kd == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *kp = g_temp_state.kp;
    *ki = g_temp_state.ki;
    *kd = g_temp_state.kd;

    return ESP_OK;
}

esp_err_t SRO_TemperatureControl_StartTuning(a2s_pid_tune_method_t method, float setpoint)
{
    if (!g_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Can't tune while PID is active
    if (g_temp_state.pid_active) {
        return ESP_ERR_INVALID_STATE;
    }

    // Initialize autotune
    esp_err_t err = a2s_pid_autotune_start(&g_autotune, method, setpoint);
    if (err != ESP_OK) {
        return err;
    }

    // Update state
    g_temp_state.mode            = SRO_TEMP_MODE_TUNING;
    g_temp_state.tuning_active   = true;
    g_temp_state.tuning_complete = false;
    g_temp_state.tune_method     = method;
    g_temp_state.target_temp     = setpoint;

    // Clear PID active bit, we're in tuning mode now
    xEventGroupClearBits(sro_status1_events, SRO_SE1_PID_ACTIVE);

    return ESP_OK;
}

esp_err_t SRO_TemperatureControl_StopTuning(void)
{
    if (!g_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (g_temp_state.mode != SRO_TEMP_MODE_TUNING) {
        return ESP_ERR_INVALID_STATE;
    }

    // Stop autotune (this sets heater_output=0, door_target=100 for safety)
    a2s_pid_autotune_stop(&g_autotune);

    // Turn off heater
    SRO_Heater_SetDuty(0);
    a2s_ssr_set_heater(false);
    g_temp_state.heater_on = false;
    xEventGroupClearBits(sro_status1_events, SRO_SE1_HEATER_ON);

    // Open door for safety when aborting
    a2s_servo_move_to(100, A2S_SERVO_SMOOTH_FAST);

    // Update state
    g_temp_state.mode          = SRO_TEMP_MODE_IDLE;
    g_temp_state.tuning_active = false;

    ESP_LOGI(TAG, "Auto-tune aborted");

    return ESP_OK;
}

esp_err_t SRO_TemperatureControl_GetTuningResults(bool *complete, float *kp, float *ki, float *kd)
{
    if (!g_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (complete == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *complete = a2s_pid_autotune_is_complete(&g_autotune);

    if (*complete && kp != NULL && ki != NULL && kd != NULL) {
        return a2s_pid_autotune_get_results(&g_autotune, kp, ki, kd);
    }

    return ESP_OK;
}

esp_err_t SRO_TemperatureControl_SetManualFanRequest(bool on)
{
    g_temp_state.manual_fan_request = on;
    return ESP_OK;
}

esp_err_t SRO_TemperatureControl_GetTuningStatus(a2s_autotune_status_t *status)
{
    if (!g_initialized || status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    return a2s_pid_autotune_get_status(&g_autotune, status);
}

esp_err_t SRO_TemperatureControl_SetFeedforward(float feedforward)
{
    if (!g_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Clamp to reasonable range
    if (feedforward < 0.0f)
        feedforward = 0.0f;
    if (feedforward > 50.0f)
        feedforward = 50.0f;

    g_temp_state.feedforward = feedforward;
    return ESP_OK;
}

void SRO_TemperatureControl_SetBoostMode(bool enable)
{
    if (g_boost_mode && !enable) {
        // Exiting boost → bumpless reset to prevent derivative spikes
        a2s_pid_reset_bumpless(&g_pid_heat, g_temp_state.current_temp, g_temp_state.target_temp);
        a2s_pid_reset_bumpless(&g_pid_cool, g_temp_state.current_temp, g_temp_state.target_temp);
        ESP_LOGI(TAG, "BOOST OFF: PID reset");
    }
    else if (!g_boost_mode && enable) {
        ESP_LOGI(TAG, "BOOST ON: Heater 100%%");
    }
    g_boost_mode = enable;
}

bool SRO_TemperatureControl_GetBoostMode(void)
{
    return g_boost_mode;
}

// ----------------------------------------------------------------------------
// static functions
// ----------------------------------------------------------------------------

static esp_err_t load_pid_params_from_nvs(float *kp, float *ki, float *kd)
{
    nvs_handle_t nvs_handle;
    esp_err_t err;

    err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        return err;
    }

    uint32_t temp_u32;
    bool all_loaded = true;

    err             = nvs_get_u32(nvs_handle, NVS_KEY_KP, &temp_u32);
    if (err == ESP_OK) {
        memcpy(kp, &temp_u32, sizeof(float));
    }
    else {
        all_loaded = false;
    }

    err = nvs_get_u32(nvs_handle, NVS_KEY_KI, &temp_u32);
    if (err == ESP_OK) {
        memcpy(ki, &temp_u32, sizeof(float));
    }
    else {
        all_loaded = false;
    }

    err = nvs_get_u32(nvs_handle, NVS_KEY_KD, &temp_u32);
    if (err == ESP_OK) {
        memcpy(kd, &temp_u32, sizeof(float));
    }
    else {
        all_loaded = false;
    }

    nvs_close(nvs_handle);

    if (!all_loaded) {
        return ESP_ERR_NOT_FOUND;
    }

    if (*kp > 0.0f && *ki >= 0.0f && *kd >= 0.0f) {
        return ESP_OK;
    }
    else {
        return ESP_ERR_INVALID_STATE;
    }
}

static esp_err_t save_pid_params_to_nvs(float kp, float ki, float kd)
{
    nvs_handle_t nvs_handle;
    esp_err_t err;

    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        return err;
    }

    uint32_t temp_u32;

    memcpy(&temp_u32, &kp, sizeof(float));
    err = nvs_set_u32(nvs_handle, NVS_KEY_KP, temp_u32);
    if (err != ESP_OK) {
        goto cleanup;
    }

    memcpy(&temp_u32, &ki, sizeof(float));
    err = nvs_set_u32(nvs_handle, NVS_KEY_KI, temp_u32);
    if (err != ESP_OK) {
        goto cleanup;
    }

    memcpy(&temp_u32, &kd, sizeof(float));
    err = nvs_set_u32(nvs_handle, NVS_KEY_KD, temp_u32);
    if (err != ESP_OK) {
        goto cleanup;
    }

    err = nvs_commit(nvs_handle);

cleanup:
    nvs_close(nvs_handle);
    return err;
}

static void read_temperature_sensor(void)
{
    float temperature = 0.0f;
    esp_err_t err     = a2s_max6675_read_temperature(&temperature);

    if (err == ESP_OK) {
        g_temp_state.current_temp     = temperature;
        g_temp_state.last_update_time = esp_timer_get_time() / 1000;

        // Update status bits
        xEventGroupSetBits(sro_status1_events, SRO_SE1_TEMP_VALID);
        xEventGroupClearBits(sro_status1_events, SRO_SE1_TEMP_SENSOR_FAULT);
    }
    else {
        // Sensor fault detected
        xEventGroupClearBits(sro_status1_events, SRO_SE1_TEMP_VALID);
        xEventGroupSetBits(sro_status1_events, SRO_SE1_TEMP_SENSOR_FAULT);
    }
}

static void pid_loop(void)
{
    static uint8_t log_counter = 0;

    // Check if PID should be active
    EventBits_t status_bits = xEventGroupGetBits(sro_status1_events);
    bool profile_running    = (status_bits & SRO_SE1_PROFILE_RUNNING) != 0;

    if (!g_temp_state.pid_active && !profile_running) {
        // Nothing to do - ensure outputs are off
        if (g_temp_state.heater_on) {
            SRO_Heater_SetDuty(0);
            g_temp_state.heater_on  = false;
            g_temp_state.pid_output = 0.0f;
            xEventGroupClearBits(sro_status1_events, SRO_SE1_HEATER_ON);
        }
        g_temp_state.control_mode = SRO_CONTROL_MODE_IDLE;
        return;
    }

    // === BOOST MODE CHECK ===
    if (g_boost_mode) {
        SRO_Heater_SetDuty(100.0f);
        g_temp_state.pid_output   = 100.0f;
        g_temp_state.heater_on    = true;
        g_temp_state.door_output  = 0.0f;
        g_temp_state.control_mode = SRO_CONTROL_MODE_HEATING;
        a2s_servo_move_to(0, A2S_SERVO_SMOOTH_FAST);
        xEventGroupSetBits(sro_status1_events, SRO_SE1_HEATER_ON);
        return;
    }
    // === END BOOST MODE ===

    float error         = g_temp_state.target_temp - g_temp_state.current_temp;
    float heater_output = 0.0f;
    float door_output   = 0.0f;

    // Mode switching with hysteresis
    if (error > SRO_TEMP_HYSTERESIS) {
        g_temp_state.control_mode = SRO_CONTROL_MODE_HEATING;
    }
    else if (error < -SRO_TEMP_HYSTERESIS) {
        g_temp_state.control_mode = SRO_CONTROL_MODE_COOLING;
    }
    else {
        // In dead band - but don't stay in wrong mode
        if (g_temp_state.control_mode == SRO_CONTROL_MODE_COOLING && error > 0) {
            // Below setpoint but in cooling mode - switch to heating
            g_temp_state.control_mode = SRO_CONTROL_MODE_HEATING;
        }
        else if (g_temp_state.control_mode == SRO_CONTROL_MODE_HEATING && error < 0) {
            // Above setpoint but in heating mode - switch to cooling
            g_temp_state.control_mode = SRO_CONTROL_MODE_COOLING;
        }
    }

    // Execute appropriate control loop
    switch (g_temp_state.control_mode) {
        case SRO_CONTROL_MODE_HEATING:
            heater_output = pid_loop_heating();
            door_output   = 0.0f;
            break;

        case SRO_CONTROL_MODE_COOLING:
            heater_output = 0.0f;
            door_output   = pid_loop_cooling();
            break;

        case SRO_CONTROL_MODE_IDLE:
        default:
            heater_output = 0.0f;
            door_output   = 0.0f;
            break;
    }

    // Apply heater output
    SRO_Heater_SetDuty(heater_output);
    g_temp_state.pid_output = heater_output;

    if (heater_output > 0.0f) {
        g_temp_state.heater_on = true;
        xEventGroupSetBits(sro_status1_events, SRO_SE1_HEATER_ON);
    }
    else {
        g_temp_state.heater_on = false;
        xEventGroupClearBits(sro_status1_events, SRO_SE1_HEATER_ON);
    }

    // Apply door output
    g_temp_state.door_output = door_output;
    a2s_servo_move_to((uint8_t)door_output, A2S_SERVO_SMOOTH_NORMAL);

    // Log every 10th call (once per second)
    log_counter++;
    if (log_counter >= 10) {
        log_counter          = 0;
        const char *mode_str = (g_temp_state.control_mode == SRO_CONTROL_MODE_HEATING) ? "HEAT" : (g_temp_state.control_mode == SRO_CONTROL_MODE_COOLING) ? "COOL"
                                                                                                                                                          : "IDLE";
        ESP_LOGI(TAG, "PID: mode=%s, sp=%.1f, pv=%.1f, heater=%.1f%%, door=%.1f%%",
                 mode_str, g_temp_state.target_temp, g_temp_state.current_temp,
                 heater_output, door_output);
    }
}

static float pid_loop_heating(void)
{
    // Bumpless reset of cooling PID - prevents derivative kick on mode switch
    a2s_pid_reset_bumpless(&g_pid_cool, g_temp_state.current_temp, g_temp_state.target_temp);
    
    float error = g_temp_state.target_temp - g_temp_state.current_temp;

    // Suppress integral accumulation when far from setpoint
    // This prevents windup during initial heat-up
    if (error > 20.0f) {
        // Far from target: use P and D only, reset integral
        g_pid_heat.i_term    = 0.0f;
        g_pid_heat.error_sum = 0.0f;
    }

    // Compute heating PID
    float output = a2s_pid_compute(&g_pid_heat, g_temp_state.current_temp);

    // Add feedforward for ramp tracking
    output += g_temp_state.feedforward;

    // Clamp total output
    // Clamp total output
    if (output < 0.0f)
        output = 0.0f;
    if (output > 100.0f)
        output = 100.0f;

    return output;
}

static float pid_loop_cooling(void)
{
    // Bumpless reset of heating PID - prevents derivative kick on mode switch
    a2s_pid_reset_bumpless(&g_pid_heat, g_temp_state.current_temp, g_temp_state.target_temp);
    
    // If already at or below setpoint, no cooling needed
    if (g_temp_state.current_temp <= g_temp_state.target_temp) {
        a2s_pid_reset_bumpless(&g_pid_cool, g_temp_state.current_temp, g_temp_state.target_temp);
        return 0.0f;
    }
    
    // For cooling: we need to invert the logic
    // When temp > setpoint, error is negative, but we want positive door output
    // Solution: temporarily invert the setpoint relationship
    float cooling_setpoint = g_temp_state.target_temp;
    float cooling_input    = 2.0f * cooling_setpoint - g_temp_state.current_temp;

    // This makes: when actual > setpoint, cooling_input < setpoint,
    // so PID sees positive error and outputs positive value
    a2s_pid_set_setpoint(&g_pid_cool, cooling_setpoint);
    float output = a2s_pid_compute(&g_pid_cool, cooling_input);

    return output;
}

static void process_tuning_mode(void)
{
    // Check if tuning is complete
    if (a2s_pid_autotune_is_complete(&g_autotune)) {
        // Get tuning results
        float kp, ki, kd;
        esp_err_t err = a2s_pid_autotune_get_results(&g_autotune, &kp, &ki, &kd);

        if (err == ESP_OK) {
            // Apply tuned parameters to heating PID
            a2s_pid_set_tunings(&g_pid_heat, kp, ki, kd);

            // Apply scaled parameters to cooling PID
            a2s_pid_set_tunings(&g_pid_cool,
                                kp * SRO_COOL_PID_KP_SCALE,
                                ki * SRO_COOL_PID_KI_SCALE,
                                kd * SRO_COOL_PID_KD_SCALE);
            
            // Update state
            g_temp_state.kp = kp;
            g_temp_state.ki = ki;
            g_temp_state.kd = kd;

            save_pid_params_to_nvs(kp, ki, kd);

            g_temp_state.tuning_complete = true;

            ESP_LOGI(TAG, "Auto-tune complete: Kp=%.4f, Ki=%.6f, Kd=%.2f", kp, ki, kd);
        }

        // Turn off heater
        SRO_Heater_SetDuty(0);
        a2s_ssr_set_heater(false);
        g_temp_state.heater_on = false;
        xEventGroupClearBits(sro_status1_events, SRO_SE1_HEATER_ON);

        // Signal tuning complete
        xEventGroupSetBits(sro_ctrl2_events, SRO_CE2_TUNING_COMPLETE);

        // Close door when tuning complete
        a2s_servo_move_to(100, A2S_SERVO_SMOOTH_FAST);

        // Return to idle mode
        g_temp_state.mode          = SRO_TEMP_MODE_IDLE;
        g_temp_state.tuning_active = false;
        g_temp_state.target_temp   = 0.0f;
        
        return;
    }

    // Continue tuning - update autotune state machine
    float heater_output;
    esp_err_t err = a2s_pid_autotune_update(&g_autotune, g_temp_state.current_temp, &heater_output);

    if (err == ESP_OK) {
        // Apply heater output from autotune
        g_temp_state.pid_output = heater_output;
        SRO_Heater_SetDuty(heater_output);

        // Apply door position from autotune
        uint8_t door_target  = a2s_pid_autotune_get_door_target(&g_autotune);
        a2s_servo_move_to(door_target, A2S_SERVO_SMOOTH_FAST);
    }
    else if (err == ESP_ERR_INVALID_STATE) {
        // Tuning not active or already complete - shouldn't happen, but handle gracefully
        ESP_LOGW(TAG, "Autotune update returned invalid state");
        g_temp_state.mode          = SRO_TEMP_MODE_IDLE;
        g_temp_state.tuning_active = false;
    }
}

// ----------------------------------------------------------------------------
// end of SRO_TemperatureControl.c
// ----------------------------------------------------------------------------
