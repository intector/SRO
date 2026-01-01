/**
 ******************************************************************************
 * @file           : SRO_ProfileManager.h
 * @brief          : Header for SRO_ProfileManager.c file.
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
#ifndef __SRO_PROFILE_MANAGER_H
#define __SRO_PROFILE_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

// Includes -------------------------------------------------------------------
#include "main.h"

// ----------------------------------------------------------------------------
// SRO_ProfileManager Configuration and Constants
// ----------------------------------------------------------------------------

#define SRO_PROFILE_MAX_PHASES              4
#define SRO_PROFILE_COOLING_END_TEMPERATURE 50
#define SRO_PROFILE_SOAK_STEPS              10

// ----------------------------------------------------------------------------
// Type Definitions
// ----------------------------------------------------------------------------

// possible profile states
typedef enum
{
    SRO_PROFILE_STATE_IDLE = 0,
    SRO_PROFILE_STATE_RUNNING,
    SRO_PROFILE_STATE_COMPLETE,
    SRO_PROFILE_STATE_ABORTED
} sro_profile_state_t;

// Profile phases
typedef enum
{
    SRO_PROFILE_PHASE_PREHEAT = 0,
    SRO_PROFILE_PHASE_SOAK,
    SRO_PROFILE_PHASE_REFLOW,
    SRO_PROFILE_PHASE_COOLING
} sro_profile_phase_t;

// Phase data
typedef struct
{
    float min_temp;    // Minimum temp (0 if not used, e.g., preheat/soak)
    float max_temp;    // Maximum/target temp for phase
    uint32_t time_sec; // Duration in seconds
} sro_profile_phase_data_t;

// Profile definition (loaded from JSON)
typedef struct
{
    uint8_t id;                                              // Profile ID (informational)
    char name[32];                                           // Profile name (informational)
    float solder_melting_point;                              // Melting point (informational)
    float absolute_peak_temp;                                // Safety limit - heater OFF above this
    float cooling_rate;                                      // Cooling rate in °C/sec
    uint32_t estimated_duration_sec;                         // Calculated duration (informational)
    sro_profile_phase_data_t phases[SRO_PROFILE_MAX_PHASES]; // Phase parameters
} sro_profile_t;

// Runtime execution status
typedef struct
{
    sro_profile_state_t state;         // Current state
    sro_profile_phase_t current_phase; // Current phase (named enum)
    uint8_t current_profile_id;        // Which profile is running

    // Timing
    uint32_t phase_elapsed_sec; // Time in current phase
    uint32_t total_elapsed_sec; // Total execution time
    uint32_t reflow_timer_sec;  // Time since reflow min_temp reached (0 if not started)

    // Temperature
    float current_temp; // Actual temperature
    float target_temp;  // Current setpoint

    // Progress (for UI)
    uint8_t progress_percent; // Overall progress 0-100%
} sro_profile_status_t;

// ----------------------------------------------------------------------------
// Public Function Prototypes
// ----------------------------------------------------------------------------
esp_err_t SRO_ProfileManager_Init(void);
void profile_execution_task(void *pvParameters);
esp_err_t SRO_ProfileManager_Start(const sro_profile_t *profile);
esp_err_t SRO_ProfileManager_Stop(void);
esp_err_t SRO_ProfileManager_GetStatus(sro_profile_status_t *status);

#ifdef __cplusplus
}
#endif

#endif // __SRO_PROFILE_MANAGER_H
