/**
 ******************************************************************************
 * @file           : SRO_ProfileManager.c
 * @brief          : Profile Manager implementation file.
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
#include "SRO_ProfileManager.h"
#include <math.h>

static const char *TAG = "SRO_PROFILE_MANAGER";

// Private variables ----------------------------------------------------------
static sro_profile_t g_current_profile       = {0};
static sro_profile_status_t g_profile_status = {0};
static bool g_initialized                    = false;
static float g_phase_start_temp              = 0.0f;
static bool g_reflow_timer_started           = false;
static uint32_t g_profile_start_time_ms      = 0;
static uint32_t g_phase_start_time_ms        = 0;
static uint32_t g_reflow_timer_start_ms      = 0;

// static functions prototypes ------------------------------------------------
static void update_profile_phase(void);
static void advance_to_next_phase(void);
static void complete_profile(void);
static const char *get_phase_name(sro_profile_phase_t phase);

// functions ------------------------------------------------------------------
esp_err_t SRO_ProfileManager_Init(void)
{
    if (g_initialized) {
        return ESP_OK;
    }

    // Initialize profile status
    memset(&g_profile_status, 0, sizeof(sro_profile_status_t));
    g_profile_status.state             = SRO_PROFILE_STATE_IDLE;
    g_profile_status.current_phase     = 0;
    g_profile_status.phase_elapsed_sec = 0;
    g_profile_status.total_elapsed_sec = 0;

    g_initialized                      = true;

    return ESP_OK;
}

void profile_execution_task(void *pvParameters)
{
    const char *TAG = "PROFILE_EXEC_TASK";
    TickType_t last_wake_time;

    // -------------------------------------------------------------------------
    // STARTUP GATE
    // -------------------------------------------------------------------------
    ESP_LOGI(TAG, "Task created, waiting for initialization...");

    xEventGroupWaitBits(sro_ctrl1_events,
                        SRO_CE1_TASKS_RELEASE,
                        pdFALSE,
                        pdTRUE,
                        portMAX_DELAY);

    ESP_LOGI(TAG, "Released! Starting operation...");

    // -------------------------------------------------------------------------
    // TASK-SPECIFIC INITIALIZATION
    // -------------------------------------------------------------------------
    last_wake_time = xTaskGetTickCount();

    // -------------------------------------------------------------------------
    // MAIN TASK LOOP - Event-driven with 1-second updates
    // -------------------------------------------------------------------------
    while (1) {
        // Wait for profile update trigger or timeout after 1 second
        EventBits_t bits = xEventGroupWaitBits(sro_ctrl1_events,
                                               SRO_CE1_PROFILE_UPDATE,
                                               pdTRUE, // Clear bit
                                               pdFALSE,
                                               pdMS_TO_TICKS(SRO_INTERVAL_PROFILE_UPDATE));

        // Only process if profile is running
        if (g_profile_status.state == SRO_PROFILE_STATE_RUNNING) {
            update_profile_phase();

            // Trigger WebSocket broadcast
            xEventGroupSetBits(sro_ctrl2_events, SRO_CE2_WEBSOCKET_BROADCAST);
        }

        // Use delay until for consistent 100-ms intervals
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(SRO_INTERVAL_PROFILE_UPDATE));
    }
}

esp_err_t SRO_ProfileManager_Start(const sro_profile_t *profile)
{
    if (!g_initialized || profile == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (g_profile_status.state == SRO_PROFILE_STATE_RUNNING) {
        return ESP_ERR_INVALID_STATE;
    }

    // Copy profile
    memcpy(&g_current_profile, profile, sizeof(sro_profile_t));

    // Get current temperature as starting point
    float current_temp = 0.0f;
    a2s_max6675_read_temperature(&current_temp);

    // Initialize profile execution
    g_profile_status.state              = SRO_PROFILE_STATE_RUNNING;
    g_profile_status.current_phase      = SRO_PROFILE_PHASE_PREHEAT;
    g_profile_status.current_profile_id = profile->id;
    g_profile_status.phase_elapsed_sec  = 0;
    g_profile_status.total_elapsed_sec  = 0;
    g_profile_status.reflow_timer_sec   = 0;
    g_profile_status.current_temp       = current_temp;
    g_profile_status.target_temp        = current_temp;
    g_profile_status.progress_percent   = 0;

    g_profile_start_time_ms             = esp_timer_get_time() / 1000;
    g_phase_start_time_ms               = g_profile_start_time_ms;
    g_phase_start_temp                  = current_temp;
    g_reflow_timer_started              = false;
    g_reflow_timer_start_ms             = 0;
    
    // Set status bits
    xEventGroupSetBits(sro_status1_events, SRO_SE1_PROFILE_RUNNING);

    // Set initial PID setpoint
    SRO_TemperatureControl_SetTarget(current_temp);

    // Enable PID control
    SRO_TemperatureControl_EnablePID(true);

    ESP_LOGI(TAG, "Profile '%s' started at %.1f°C", profile->name, current_temp);

    return ESP_OK;
}

esp_err_t SRO_ProfileManager_Stop(void)
{
    if (!g_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    g_profile_status.state = SRO_PROFILE_STATE_ABORTED;

    // Clear running status
    xEventGroupClearBits(sro_status1_events, SRO_SE1_PROFILE_RUNNING);

    // Disable feedforward
    SRO_TemperatureControl_SetFeedforward(0.0f);
    
    // Reset setpoint to 0
    g_profile_status.target_temp = 0.0f;
    SRO_TemperatureControl_SetTarget(0.0f);

    // Disable PID
    SRO_TemperatureControl_EnablePID(false);

    // Door fully open for cooling
    a2s_servo_move_to(100, A2S_SERVO_SMOOTH_NORMAL);

    ESP_LOGI(TAG, "Profile aborted");

    return ESP_OK;
}

esp_err_t SRO_ProfileManager_GetStatus(sro_profile_status_t *status)
{
    if (!g_initialized || status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memcpy(status, &g_profile_status, sizeof(sro_profile_status_t));
    return ESP_OK;
}

// static functions -----------------------------------------------------------
static void update_profile_phase(void)
{
    uint32_t current_time_ms = esp_timer_get_time() / 1000; // milliseconds

    // Calculate elapsed as float seconds for smooth ramps
    float total_elapsed = (float)(current_time_ms - g_profile_start_time_ms) / 1000.0f;
    float phase_elapsed = (float)(current_time_ms - g_phase_start_time_ms) / 1000.0f;

    // Update status (keep as integers for display)
    g_profile_status.total_elapsed_sec = (uint32_t)total_elapsed;
    g_profile_status.phase_elapsed_sec = (uint32_t)phase_elapsed;

    // Get current temperature
    float current_temp = 0.0f;
    a2s_max6675_read_temperature(&current_temp);
    g_profile_status.current_temp = current_temp;

    // Safety override: absolute peak temperature
    if (current_temp >= g_current_profile.absolute_peak_temp) {
        ESP_LOGW(TAG, "SAFETY: Temp %.1f >= peak %.1f - heater OFF",
                 current_temp, g_current_profile.absolute_peak_temp);
        SRO_TemperatureControl_EnablePID(false);
        SRO_TemperatureControl_SetTarget(0);
        a2s_servo_move_to(100, A2S_SERVO_SMOOTH_FAST);
        return;
    }

    float setpoint      = 0.0f;
    bool should_advance = false;

    switch (g_profile_status.current_phase) {
        // ---------------------------------------------------------------------
        // PREHEAT: Fixed target, heat only (door stays closed), exit on temp
        // ---------------------------------------------------------------------
        case SRO_PROFILE_PHASE_PREHEAT: {
            float max_temp = g_current_profile.phases[SRO_PROFILE_PHASE_PREHEAT].max_temp;
            float margin   = 15.0f; // Hand over to PID 15°C before target

            setpoint       = max_temp;

            if (current_temp < (max_temp - margin)) {
                SRO_TemperatureControl_SetBoostMode(true);
            }
            else {
                SRO_TemperatureControl_SetBoostMode(false);
            }

            // Exit when target reached
            if (current_temp >= max_temp) {
                ESP_LOGI(TAG, "PREHEAT complete - temp: %.1f°C, time: %.1f sec", current_temp, phase_elapsed);
                should_advance = true;
            }
            break;
        }

            // ---------------------------------------------------------------------
            // SOAK: Smooth linear ramp with feedforward, full dual-PID, exit on time
            // Box: preheat.max_temp <= actual <= soak.max_temp
            // ---------------------------------------------------------------------
            case SRO_PROFILE_PHASE_SOAK: {
            SRO_TemperatureControl_SetBoostMode(false);
            float start_temp = g_current_profile.phases[SRO_PROFILE_PHASE_PREHEAT].max_temp;
            float end_temp   = g_current_profile.phases[SRO_PROFILE_PHASE_SOAK].max_temp;
            float duration   = (float)g_current_profile.phases[SRO_PROFILE_PHASE_SOAK].time_sec;

            // Calculate ramp rate and feedforward
            float ramp_rate   = (end_temp - start_temp) / duration; // °C/sec
            float feedforward = ramp_rate * SRO_FEEDFORWARD_GAIN;
            SRO_TemperatureControl_SetFeedforward(feedforward);

            // Smooth linear ramp based on elapsed time
            float progress = phase_elapsed / duration;
            if (progress > 1.0f) {
                progress = 1.0f;
            }
            setpoint = start_temp + (progress * (end_temp - start_temp));
            setpoint = roundf(setpoint * 10.0f) / 10.0f;

            // Exit when time elapsed (regardless of temperature)
            if (phase_elapsed >= duration) {
                ESP_LOGI(TAG, "SOAK complete - temp: %.1f°C, time: %.1f sec", current_temp, phase_elapsed);
                SRO_TemperatureControl_SetFeedforward(0.0f); // Disable feedforward
                should_advance = true;
            }
            break;
        }
        
        // ---------------------------------------------------------------------
        // REFLOW: Fast ramp to min_temp, then hold at center of box
        // Box: reflow.min_temp <= actual <= reflow.max_temp
        // Timer starts when min_temp reached, exit when timer expires
        // ---------------------------------------------------------------------
        case SRO_PROFILE_PHASE_REFLOW: {
            float min_temp  = g_current_profile.phases[SRO_PROFILE_PHASE_REFLOW].min_temp;
            float max_temp  = g_current_profile.phases[SRO_PROFILE_PHASE_REFLOW].max_temp;
            float hold_time = (float)g_current_profile.phases[SRO_PROFILE_PHASE_REFLOW].time_sec;
            float margin    = 5.0f; // Hand over to PID 5°C before reflow zone

            setpoint        = min_temp + ((max_temp - min_temp) / 2.0f);

            // Boost until close to reflow zone
            if (current_temp < (min_temp - margin)) {
                SRO_TemperatureControl_SetBoostMode(true);
            }
            else {
                SRO_TemperatureControl_SetBoostMode(false);
            }

            // Start timer when min_temp reached
            if (!g_reflow_timer_started && current_temp >= min_temp) {
                g_reflow_timer_started  = true;
                g_reflow_timer_start_ms = current_time_ms;
                ESP_LOGI(TAG, "REFLOW timer started at %.1f°C", current_temp);
            }

            if (g_reflow_timer_started) {
                float reflow_elapsed              = (float)(current_time_ms - g_reflow_timer_start_ms) / 1000.0f;
                g_profile_status.reflow_timer_sec = (uint32_t)reflow_elapsed;

                if (reflow_elapsed >= hold_time) {
                    ESP_LOGI(TAG, "REFLOW complete - temp: %.1f°C, time: %.1f sec", current_temp, phase_elapsed);
                    should_advance = true;
                }
            }
            break;
        }
        
        // ---------------------------------------------------------------------
        // COOLING: Passive - PID off, heater off, door 100% open
        // Exit when temp < 50°C
        // ---------------------------------------------------------------------
        case SRO_PROFILE_PHASE_COOLING: {
            SRO_TemperatureControl_SetBoostMode(false);
            // Disable PID - no active temperature control
            SRO_TemperatureControl_EnablePID(false);

            // Heater off, target 50°C
            setpoint = (float)SRO_PROFILE_COOLING_END_TEMPERATURE;
            g_profile_status.target_temp = setpoint;
            SRO_TemperatureControl_SetTarget(g_profile_status.target_temp);

            a2s_servo_move_to(100, A2S_SERVO_SMOOTH_FAST);

            // Exit when cool enough
            if (current_temp <= (float)SRO_PROFILE_COOLING_END_TEMPERATURE) {
                a2s_servo_move_to(100, A2S_SERVO_SMOOTH_FAST);
                ESP_LOGI(TAG, "COOLING complete - temp: %.1f°C, time: %.1f sec", current_temp, phase_elapsed);
                should_advance = true;
            }
            break;
        }
    }

    // Apply setpoint (except cooling which handles it directly)
    if (g_profile_status.current_phase != SRO_PROFILE_PHASE_COOLING) {
        g_profile_status.target_temp = setpoint;
        SRO_TemperatureControl_SetTarget(setpoint);
    }

    // Calculate progress
    uint32_t phase_time               = g_current_profile.phases[g_profile_status.current_phase].time_sec;
    g_profile_status.progress_percent = (uint8_t)((g_profile_status.current_phase * 25) +
                                                  (g_profile_status.phase_elapsed_sec * 25 / (phase_time + 1)));
    if (g_profile_status.progress_percent > 100)
        g_profile_status.progress_percent = 100;

    if (should_advance) {
        advance_to_next_phase();
    }
}

static void advance_to_next_phase(void)
{
    g_profile_status.current_phase++;

    if (g_profile_status.current_phase > SRO_PROFILE_PHASE_COOLING) {
        complete_profile();
        return;
    }

    // Reset phase tracking (use milliseconds)
    g_phase_start_time_ms              = esp_timer_get_time() / 1000;
    g_profile_status.phase_elapsed_sec = 0;
    g_phase_start_temp                 = g_profile_status.current_temp;

    // Reset reflow timer
    g_reflow_timer_started            = false;
    g_reflow_timer_start_ms           = 0;
    g_profile_status.reflow_timer_sec = 0;

    ESP_LOGI(TAG, "Advanced to %s phase, start temp: %.1f°C",
             get_phase_name(g_profile_status.current_phase), g_phase_start_temp);
}

static void complete_profile(void)
{
    g_profile_status.state = SRO_PROFILE_STATE_COMPLETE;

    // Clear running status
    xEventGroupClearBits(sro_status1_events, SRO_SE1_PROFILE_RUNNING);

    // Disable feedforward
    SRO_TemperatureControl_SetFeedforward(0.0f);
    
    // Disable PID
    SRO_TemperatureControl_EnablePID(false);

    // Reset setpoint
    g_profile_status.target_temp = 0.0f;
    SRO_TemperatureControl_SetTarget(0.0f);

    // Open door fully for rapid cooling after profile complete
    a2s_servo_move_to(100, A2S_SERVO_SMOOTH_FAST);

    // Notify web interface
    xEventGroupSetBits(sro_ctrl2_events, SRO_CE2_SOLDERING_COMPLETE);

    ESP_LOGI(TAG, "Profile completed");
}

static const char *get_phase_name(sro_profile_phase_t phase)
{
    switch (phase) {
        case SRO_PROFILE_PHASE_PREHEAT:
            return "PREHEAT";
        case SRO_PROFILE_PHASE_SOAK:
            return "SOAK";
        case SRO_PROFILE_PHASE_REFLOW:
            return "REFLOW";
        case SRO_PROFILE_PHASE_COOLING:
            return "COOLING";
        default:
            return "UNKNOWN";
    }
}

// -------------------------------------------------------------------------
// end of SRO_ProfileManager.c
// -------------------------------------------------------------------------
