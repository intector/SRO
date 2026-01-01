/**
 ******************************************************************************
 * @file           : SRO_TemperatureControl.h
 * @brief          : Header for SRO_TemperatureControl.c file.
 *                   This file contains the common defines of the application.
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

// Define to prevent recursive inclusion --------------------------------------
#ifndef __SRO_TEMPERATURE_CONTROL_H
#define __SRO_TEMPERATURE_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

// Includes -------------------------------------------------------------------
#include "a2s_pid.h"
#include "main.h"

// ----------------------------------------------------------------------------
// LEDC PWM Configuration for Heater
// ----------------------------------------------------------------------------

#define SRO_HEATER_PWM_TICK_MS      20 // Tick interval in milliseconds
#define SRO_HEATER_PWM_PERIOD_TICKS 25 // 25 × 20ms = 500ms period

// ----------------------------------------------------------------------------
// SRO_TemperatureControl Configuration and Constants
// ----------------------------------------------------------------------------

// Default PID parameters (will be replaced by tuned values)
#define SRO_PID_DEFAULT_KP        2.5f
#define SRO_PID_DEFAULT_KI        0.015f
#define SRO_PID_DEFAULT_KD        20.0f
#define SRO_PID_OUTPUT_MIN        0.0f
#define SRO_PID_OUTPUT_MAX        100.0f
#define SRO_PID_SAMPLE_TIME_MS    100 // Match safety monitor interval
#define SRO_PID_DERIVATIVE_FILTER 0.1f

// Feedforward gain: heater % per °C/sec ramp rate
// Tune this value based on testing (start with 120-150)
#define SRO_FEEDFORWARD_GAIN      140.0f

// ----------------------------------------------------------------------------
// Dual PID (Heat/Cool) Configuration
// ----------------------------------------------------------------------------
#define SRO_TEMP_HYSTERESIS   1.0f // ±°C dead band between modes
#define SRO_COOL_PID_KP_SCALE 0.5f // Cooling Kp = Heating Kp × scale
#define SRO_COOL_PID_KI_SCALE 0.3f // Cooling Ki = Heating Ki × scale
#define SRO_COOL_PID_KD_SCALE 0.5f // Cooling Kd = Heating Kd × scale

// ----------------------------------------------------------------------------
// NVS storage keys for PID parameters
// ----------------------------------------------------------------------------

#define NVS_NAMESPACE "pid_params"
#define NVS_KEY_KP    "kp"
#define NVS_KEY_KI    "ki"
#define NVS_KEY_KD    "kd"

// ----------------------------------------------------------------------------
// Type Definitions
// ----------------------------------------------------------------------------

typedef enum
{
    SRO_TEMP_MODE_IDLE,   // Not controlling temperature
    SRO_TEMP_MODE_NORMAL, // Normal PID control
    SRO_TEMP_MODE_TUNING  // Auto-tuning mode
} sro_temp_control_mode_t;

// Active control mode (heating vs cooling)
typedef enum
{
    SRO_CONTROL_MODE_IDLE = 0,
    SRO_CONTROL_MODE_HEATING,
    SRO_CONTROL_MODE_COOLING
} sro_control_mode_t;

typedef struct
{
    // Temperature readings
    float current_temp;
    float target_temp;

    // PID output and control
    float pid_output;
    bool heater_on;
    float door_output;               // Door position (0-100%)
    sro_control_mode_t control_mode; // Current heating/cooling mode
    
    // Operating mode
    sro_temp_control_mode_t mode;
    bool pid_active;

    // PID tuning parameters
    float kp;
    float ki;
    float kd;

    // Tuning status
    bool tuning_active;
    bool tuning_complete;
    a2s_pid_tune_method_t tune_method;

    // Manual fan control
    bool manual_fan_request;

    // Timing
    uint32_t last_update_time;

    // Feedforward term for ramp tracking
    float feedforward; 

} sro_temp_control_state_t;

// ----------------------------------------------------------------------------
// Public Function Prototypes - Core Functions
// ----------------------------------------------------------------------------

esp_err_t SRO_TemperatureControl_Init(void);
void temperature_control_task(void *pvParameters);
void SRO_Heater_SetDuty(float percent);
void heater_pwm_task(void *pvParameters);

// ----------------------------------------------------------------------------
// Public Function Prototypes - Control Functions
// ----------------------------------------------------------------------------

esp_err_t SRO_TemperatureControl_SetTarget(float target_temp);
esp_err_t SRO_TemperatureControl_EnablePID(bool enable);
esp_err_t SRO_TemperatureControl_GetState(sro_temp_control_state_t *state);

// ----------------------------------------------------------------------------
// Public Function Prototypes - PID Tuning Functions
// ----------------------------------------------------------------------------

esp_err_t SRO_TemperatureControl_SetPIDParams(float kp, float ki, float kd);
esp_err_t SRO_TemperatureControl_GetPIDParams(float *kp, float *ki, float *kd);
esp_err_t SRO_TemperatureControl_StartTuning(a2s_pid_tune_method_t method, float setpoint);
esp_err_t SRO_TemperatureControl_StopTuning(void);
esp_err_t SRO_TemperatureControl_GetTuningResults(bool *complete, float *kp, float *ki, float *kd);
esp_err_t SRO_TemperatureControl_SetManualFanRequest(bool on);
esp_err_t SRO_TemperatureControl_GetTuningStatus(a2s_autotune_status_t *status);

// Set feedforward term for ramp tracking (0.0 to disable)
esp_err_t SRO_TemperatureControl_SetFeedforward(float feedforward);

// Boost mode for rapid heating (bypasses PID)
void SRO_TemperatureControl_SetBoostMode(bool enable);
bool SRO_TemperatureControl_GetBoostMode(void);

// ----------------------------------------------------------------------------
// end of file
// ----------------------------------------------------------------------------
#ifdef __cplusplus
}
#endif

#endif // __SRO_TEMPERATURE_CONTROL_H
