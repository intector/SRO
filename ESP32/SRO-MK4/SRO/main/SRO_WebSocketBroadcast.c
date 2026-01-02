/**
 ******************************************************************************
 * @file           : SRO_WebSocketBroadcast.c
 * @brief          : WebSocket Broadcast Tasks - Data and Timer
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
#include "SRO_WebSocketBroadcast.h"
#include "SRO_ProfileManager.h"
#include "SRO_TemperatureControl.h"
#include "SRO_WebSocketServer.h"
#include "a2s_max6675.h"
#include "a2s_servo.h"
#include "a2s_ssr.h"
#include "cJSON.h"
#include "a2s_pid.h"
#include "esp_wifi.h"

static const char *TAG = "SRO_WEBSOCKET_BROADCAST";

// Private variables ----------------------------------------------------------
static bool g_initialized = false;

// static functions prototypes ------------------------------------------------
static char *build_status_json(void);
static void broadcast_to_clients(const char *json_data);

// functions ------------------------------------------------------------------
esp_err_t SRO_WebSocketBroadcast_Init(void)
{
    if (g_initialized) {
        return ESP_OK;
    }

    g_initialized = true;
    return ESP_OK;
}

// ============================================================================
// TASK 1: WebSocket Broadcast Task (Priority 8)
// Waits for SRO_CE2_WEBSOCKET_BROADCAST event, builds JSON, sends to clients
// ============================================================================

void websocket_broadcast_task(void *pvParameters)
{
    const char *TAG = "WEBSOCKET_BCAST_TASK";

    // -------------------------------------------------------------------------
    // STARTUP GATE
    // -------------------------------------------------------------------------

    xEventGroupWaitBits(sro_ctrl1_events,
                        SRO_CE1_TASKS_RELEASE,
                        pdFALSE,
                        pdTRUE,
                        portMAX_DELAY);

    ESP_LOGI(TAG, "Released! Starting operation...");

    // -------------------------------------------------------------------------
    // MAIN TASK LOOP - Event-driven
    // -------------------------------------------------------------------------
    while (1) {
        // Wait for broadcast trigger event
        EventBits_t bits = xEventGroupWaitBits(sro_ctrl2_events,
                                               SRO_CE2_WEBSOCKET_BROADCAST | SRO_CE2_TUNING_COMPLETE | SRO_CE2_SOLDERING_COMPLETE,
                                               pdTRUE, // Clear bit after reading
                                               pdFALSE,
                                               portMAX_DELAY);

        if (bits & SRO_CE2_TUNING_COMPLETE) {
            cJSON *root = cJSON_CreateObject();
            cJSON_AddStringToObject(root, "event", "tuningComplete");

            cJSON *autotune = cJSON_CreateObject();
            cJSON_AddStringToObject(autotune, "phase", "idle");
            cJSON_AddStringToObject(autotune, "message", "Auto-tuning complete");
            cJSON_AddItemToObject(root, "autotune", autotune);

            char *json_data = cJSON_Print(root);
            if (json_data) {
                broadcast_to_clients(json_data);
                free(json_data);
            }
            cJSON_Delete(root);
        }

        if (bits & SRO_CE2_SOLDERING_COMPLETE) {
            cJSON *root = cJSON_CreateObject();
            cJSON_AddStringToObject(root, "event", "profileCompleted");
            cJSON_AddBoolToObject(root, "success", true);
            cJSON_AddStringToObject(root, "message", "Soldering profile completed");

            char *json_data = cJSON_Print(root);
            if (json_data) {
                broadcast_to_clients(json_data);
                free(json_data);
            }
            cJSON_Delete(root);
        }

        if (bits & SRO_CE2_WEBSOCKET_BROADCAST) {
            // Build JSON status message
            char *json_data = build_status_json();

            if (json_data != NULL) {
                // Broadcast to all connected clients
                broadcast_to_clients(json_data);

                // CRITICAL: Free JSON memory to prevent leak
                free(json_data);
                json_data = NULL;
            }
        }
    }
}

// ============================================================================
// TASK 2: WebSocket Broadcast Timer Task (Priority 6)
// Triggers periodic broadcasts (1 second) when clients are connected
// Controlled by SRO_CE2_BROADCAST_ENABLE bit
// ============================================================================

void websocket_broadcast_timer_task(void *pvParameters)
{
    const char *TAG = "WS_TIMER_TASK";

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
    // MAIN TASK LOOP
    // -------------------------------------------------------------------------
    while (1) {
        // Wait for broadcast enable bit (clients connected)
        ESP_LOGI(TAG, "Waiting for clients to connect...");

        xEventGroupWaitBits(sro_ctrl2_events,
                            SRO_CE2_BROADCAST_ENABLE,
                            pdFALSE, // Don't clear the bit
                            pdTRUE,
                            portMAX_DELAY);

        ESP_LOGI(TAG, "Clients connected - starting periodic broadcasts");

        TickType_t last_wake_time = xTaskGetTickCount();

        // Broadcast loop - runs while clients are connected
        while (1) {
            // Check if we should still be broadcasting
            EventBits_t bits = xEventGroupGetBits(sro_ctrl2_events);
            if (!(bits & SRO_CE2_BROADCAST_ENABLE)) {
                ESP_LOGI(TAG, "No clients connected - stopping broadcasts");
                break; // Exit inner loop, go back to waiting
            }

            // Trigger broadcast
            xEventGroupSetBits(sro_ctrl2_events, SRO_CE2_WEBSOCKET_BROADCAST);

            // Wait 1 second
            vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1000));
        }
    }
}

// ============================================================================
// STATIC FUNCTIONS
// ============================================================================

static char *build_status_json(void)
{
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        return NULL;
    }

    // Get temperature control state
    sro_temp_control_state_t temp_state;
    esp_err_t temp_ret = SRO_TemperatureControl_GetState(&temp_state);

    // Get profile status
    sro_profile_status_t profile_status;
    esp_err_t profile_ret = SRO_ProfileManager_GetStatus(&profile_status);

    // Get hardware states
    a2s_ssr_state_t heater_state = a2s_ssr_get_heater_state();
    a2s_ssr_state_t fan_state    = a2s_ssr_get_fan_state();
    // uint8_t door_position        = a2s_servo_get_door_percent();

    // Get event group status bits
    EventBits_t status1_bits = xEventGroupGetBits(sro_status1_events);
    EventBits_t ctrl1_bits   = xEventGroupGetBits(sro_ctrl1_events);

    // **ADD DIAGNOSTICS OBJECT**
    cJSON *diagnostics = cJSON_CreateObject();
    if (diagnostics) {
        cJSON_AddNumberToObject(diagnostics, "uptime_sec", esp_timer_get_time() / 1000000);
        cJSON_AddNumberToObject(diagnostics, "free_heap", esp_get_free_heap_size());
        // cJSON_AddStringToObject(diagnostics, "temp_get_result", esp_err_to_name(temp_ret));
        // Check for sensor fault from status bits
        const char *sensor_status;
        if (status1_bits & SRO_SE1_TEMP_SENSOR_FAULT) {
            sensor_status = "ESP_ERR_INVALID_RESPONSE"; // Sensor error
        }
        else if (status1_bits & SRO_SE1_TEMP_VALID) {
            sensor_status = "ESP_OK"; // Valid reading
        }
        else {
            sensor_status = "ESP_ERR_TIMEOUT"; // No recent reading
        }
        cJSON_AddStringToObject(diagnostics, "temp_get_result", sensor_status);
        
        cJSON_AddStringToObject(diagnostics, "profile_get_result", esp_err_to_name(profile_ret));
        cJSON_AddNumberToObject(diagnostics, "status1_bits", status1_bits);
        cJSON_AddNumberToObject(diagnostics, "ctrl1_bits", ctrl1_bits);
        cJSON_AddNumberToObject(diagnostics, "temp_mode", temp_state.mode);
        cJSON_AddNumberToObject(diagnostics, "temp_last_update", temp_state.last_update_time);
        cJSON_AddBoolToObject(diagnostics, "system_initialized",
                              (xEventGroupGetBits(sro_status2_events) & SRO_SE2_SYSTEM_INITIALIZED) != 0);
        cJSON_AddBoolToObject(diagnostics, "tasks_released",
                              (ctrl1_bits & SRO_CE1_TASKS_RELEASE) != 0);
        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            cJSON_AddNumberToObject(diagnostics, "rssi", ap_info.rssi);
        }
        else {
            cJSON_AddNumberToObject(diagnostics, "rssi", 0);
        }

        cJSON_AddItemToObject(root, "diagnostics", diagnostics);
    }

    // Determine target temperature
    float target_temperature = 0.0f;
    if (profile_ret == ESP_OK && profile_status.state == SRO_PROFILE_STATE_RUNNING) {
        target_temperature = profile_status.target_temp;
    }
    else if (temp_state.tuning_active) {
        // Only show target during HEAT phase of tuning
        a2s_autotune_status_t tune_status;
        if (SRO_TemperatureControl_GetTuningStatus(&tune_status) == ESP_OK &&
            tune_status.phase == A2S_AUTOTUNE_PHASE_HEAT) {
            target_temperature = tune_status.target_temp;
        }
    }
    else {
        target_temperature = temp_state.target_temp;
    }

    float current_temp = temp_state.current_temp;

    // Basic temperature data
    cJSON_AddNumberToObject(root, "temp", current_temp);
    cJSON_AddNumberToObject(root, "target", target_temperature);

    // Hardware status
    cJSON_AddBoolToObject(root, "heater", (heater_state == A2S_SSR_STATE_ON));
    cJSON_AddBoolToObject(root, "fan", (fan_state == A2S_SSR_STATE_ON));

    // Door status (enhanced with state info)
    cJSON *door = cJSON_CreateObject();
    cJSON_AddNumberToObject(door, "position", a2s_servo_get_position());
    cJSON_AddBoolToObject(door, "moving", a2s_servo_is_moving());

    // Tuning status (if active)
    if (temp_state.tuning_active) {
        a2s_autotune_status_t tune_status;
        if (SRO_TemperatureControl_GetTuningStatus(&tune_status) == ESP_OK) {
            cJSON *autotune = cJSON_CreateObject();
            cJSON_AddStringToObject(autotune, "phase", a2s_pid_get_autotune_phase_string(tune_status.phase));
            cJSON_AddNumberToObject(autotune, "progress", tune_status.progress_percent);
            cJSON_AddStringToObject(autotune, "message", tune_status.message);
            cJSON_AddNumberToObject(autotune, "currentTemp", tune_status.current_temp);
            cJSON_AddNumberToObject(autotune, "targetTemp", tune_status.target_temp);
            cJSON_AddNumberToObject(autotune, "ambientTemp", tune_status.ambient_temp);
            cJSON_AddNumberToObject(autotune, "peakTemp", tune_status.peak_temp);
            cJSON_AddItemToObject(root, "autotune", autotune);
        }
    }    
    
    a2s_servo_state_t door_state = a2s_servo_get_state();
    const char *state_str = (door_state == A2S_SERVO_STATE_IDLE) ? "idle" :
                            (door_state == A2S_SERVO_STATE_MOVING) ? "moving" :
                            (door_state == A2S_SERVO_STATE_ERROR)    ? "error" : "unknown";
    cJSON_AddStringToObject(door, "state", state_str);

    cJSON_AddItemToObject(root, "door", door);

    // PID status
    cJSON *pid = cJSON_CreateObject();
    cJSON_AddBoolToObject(pid, "active", temp_state.pid_active);
    cJSON_AddNumberToObject(pid, "output", temp_state.pid_output);
    cJSON_AddNumberToObject(pid, "setpoint", temp_state.target_temp);
    cJSON_AddNumberToObject(pid, "kp", temp_state.kp);
    cJSON_AddNumberToObject(pid, "ki", temp_state.ki);
    cJSON_AddNumberToObject(pid, "kd", temp_state.kd);

    // Control mode and door output
    const char *mode_str = (temp_state.control_mode == SRO_CONTROL_MODE_HEATING) ? "heating" : (temp_state.control_mode == SRO_CONTROL_MODE_COOLING) ? "cooling"
                                                                                                                                                     : "idle";
    cJSON_AddStringToObject(pid, "controlMode", mode_str);
    cJSON_AddNumberToObject(pid, "doorOutput", temp_state.door_output);

    cJSON_AddItemToObject(root, "pid", pid);
    
    // Profile data (if running)
    if (profile_ret == ESP_OK && profile_status.state == SRO_PROFILE_STATE_RUNNING) {
        // ... your existing profile JSON code ...
    }

    char *json_string = cJSON_Print(root);
    cJSON_Delete(root);

    return json_string;
}

static void broadcast_to_clients(const char *json_data)
{
    if (json_data == NULL) {
        return;
    }

    // Send via WebSocket event system
    esp_err_t result = SRO_WebSocketServer_SendEvent(SRO_WS_EVENT_TEMP_UPDATE, json_data);

    if (result != ESP_OK) {
        ESP_LOGW(TAG, "Broadcast failed: %s", esp_err_to_name(result));
    }
}

// ----------------------------------------------------------------------------
// end of SRO_WebSocketBroadcast.c
// ----------------------------------------------------------------------------
