/**
 ******************************************************************************
 * @file           : SRO_SystemCoordinator.c
 * @brief          : System Coordinator implementation file.
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
#include "SRO_SystemCoordinator.h"

static const char *TAG = "SRO_SYSTEM_COORDINATOR";

// Private variables ----------------------------------------------------------
static bool g_initialized             = false;
static uint32_t g_last_heartbeat_time = 0;

// static functions prototypes ------------------------------------------------
static void handle_command_events(EventBits_t bits);
static void send_system_heartbeat(void);
static void handle_emergency_stop(void);

// functions ------------------------------------------------------------------
esp_err_t SRO_SystemCoordinator_Init(void)
{
    if (g_initialized) {
        return ESP_OK;
    }

    g_last_heartbeat_time = esp_timer_get_time() / 1000;
    g_initialized         = true;

    return ESP_OK;
}

void system_coordinator_task(void *pvParameters)
{
    const char *TAG = "SYS_COORDINATOR_TASK";
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
    // MAIN TASK LOOP - Event-driven with periodic tasks
    // -------------------------------------------------------------------------
    while (1) {
        uint32_t current_time = esp_timer_get_time() / 1000;

        // Wait for command events with timeout
        EventBits_t bits = xEventGroupWaitBits(sro_ctrl1_events,
                                               SRO_CE1_HEATER_ON_CMD | SRO_CE1_HEATER_OFF_CMD |
                                                   SRO_CE1_FAN_ON_CMD | SRO_CE1_FAN_OFF_CMD |
                                                   SRO_CE1_DOOR_MOVE_CMD | SRO_CE1_EMERGENCY_STOP,
                                               pdTRUE,  // Clear bits on exit
                                               pdFALSE, // Wait for ANY bit
                                               pdMS_TO_TICKS(100));

        // Handle any command events
        if (bits != 0) {
            handle_command_events(bits);
        }

        // Periodic heartbeat every 5 seconds
        if ((current_time - g_last_heartbeat_time) >= SRO_INTERVAL_HEARTBEAT) {
            send_system_heartbeat();
            g_last_heartbeat_time = current_time;
        }

        // Small delay to prevent task starvation
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// static functions -----------------------------------------------------------
static void handle_command_events(EventBits_t bits)
{
    // Handle emergency stop first (highest priority)
    if (bits & SRO_CE1_EMERGENCY_STOP) {
        handle_emergency_stop();
        return;
    }

    // Handle heater commands
    if (bits & SRO_CE1_HEATER_ON_CMD) {
        EventBits_t status = xEventGroupGetBits(sro_status1_events);
        a2s_ssr_set_heater(true);
        xEventGroupSetBits(sro_status1_events, SRO_SE1_HEATER_ON);
    }

    if (bits & SRO_CE1_HEATER_OFF_CMD) {
        a2s_ssr_set_heater(false);
        xEventGroupClearBits(sro_status1_events, SRO_SE1_HEATER_ON);
    }

    // Handle fan commands
    if (bits & SRO_CE1_FAN_ON_CMD) {
        a2s_ssr_set_fan(true);
        xEventGroupSetBits(sro_status1_events, SRO_SE1_FAN_ON);
    }

    if (bits & SRO_CE1_FAN_OFF_CMD) {
        a2s_ssr_set_fan(false);
        xEventGroupClearBits(sro_status1_events, SRO_SE1_FAN_ON);
    }
    
    // Handle door movement commands
    if (bits & SRO_CE1_DOOR_MOVE_CMD) {
        // Door movement is handled by profile manager
        // This is a placeholder for manual door control
    }
}

static void send_system_heartbeat(void)
{
    // Trigger memory check every 6th heartbeat (30 seconds)
    static uint8_t heartbeat_counter = 0;
    heartbeat_counter++;

    if (heartbeat_counter >= 6) {
        xEventGroupSetBits(sro_ctrl2_events, SRO_CE2_MEMORY_CHECK);
        heartbeat_counter = 0;
    }

    // Could add other periodic tasks here
}

static void handle_emergency_stop(void)
{
    ESP_LOGE(TAG, "EMERGENCY STOP activated!");

    // Turn off all actuators immediately
    a2s_ssr_set_heater(false);
    a2s_ssr_set_fan(false);

    // Update status bits
    xEventGroupClearBits(sro_status1_events, SRO_SE1_HEATER_ON);
    xEventGroupClearBits(sro_status1_events, SRO_SE1_FAN_ON);

    // Stop any running profile
    SRO_ProfileManager_Stop();

    // Open door fully for cooling
    a2s_servo_move_to(100, A2S_SERVO_SMOOTH_INSTANT);
}

// -------------------------------------------------------------------------
// end of SRO_SystemCoordinator.c
// -------------------------------------------------------------------------
