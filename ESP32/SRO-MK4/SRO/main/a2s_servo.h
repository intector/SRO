/**
 ******************************************************************************
 * @file           : a2s_servo.h
 * @brief          : A2S Servo Motor Driver - Standalone Reusable Component
 ******************************************************************************
 *
 * Copyright (c) 2024-2030 Intector Inc.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#ifndef A2S_SERVO_H
#define A2S_SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

// Includes -------------------------------------------------------------------
#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

// ----------------------------------------------------------------------------
// Hardware Configuration - Adjust for your servo hardware
// ----------------------------------------------------------------------------

// Default GPIO pin for PWM output
#define A2S_SERVO_DEFAULT_GPIO 10

// PWM pulse width range (microseconds)
// Standard RC servo: 500us = 0°, 2500us = 180°
#define A2S_SERVO_MIN_PULSE_US 500
#define A2S_SERVO_MAX_PULSE_US 2500

// PWM frequency (Hz) - standard for RC servos
#define A2S_SERVO_PWM_FREQUENCY_HZ 50

// ----------------------------------------------------------------------------
// Position Definitions - Physical Meaning
// ----------------------------------------------------------------------------

// Define what 0% and 100% position mean for your application
// Examples:
//   Door application: 0% = fully closed, 100% = fully open
//   Valve application: 0% = fully closed, 100% = fully open
//   Generic angle: 0% = minimum angle, 100% = maximum angle

#define A2S_SERVO_POSITION_0_PERCENT_MEANS   "Fully Closed / Minimum Angle"
#define A2S_SERVO_POSITION_100_PERCENT_MEANS "Fully Open / Maximum Angle"

// Position limits
#define A2S_SERVO_POSITION_MIN 0
#define A2S_SERVO_POSITION_MAX 100

// Position tolerance for "movement complete" detection
#define A2S_SERVO_POSITION_TOLERANCE 0.5f // ±0.5%

// ----------------------------------------------------------------------------
// Smoothness Levels - Configure Speed and Acceleration
// ----------------------------------------------------------------------------

// Level 1: Instant (no smoothing) - used for SMOOTH_INSTANT
#define A2S_SERVO_SMOOTH_INSTANT_SPEED 1000.0f // Effectively instant
#define A2S_SERVO_SMOOTH_INSTANT_ACCEL 1000.0f

/*
// Level 2: Fast movement
#define A2S_SERVO_SMOOTH_FAST_SPEED 50.0f // %/second
#define A2S_SERVO_SMOOTH_FAST_ACCEL 80.0f // %/second²

// Level 3: Normal movement (balanced)
#define A2S_SERVO_SMOOTH_NORMAL_SPEED 30.0f // %/second
#define A2S_SERVO_SMOOTH_NORMAL_ACCEL 50.0f // %/second²

// Level 4: Slow movement
#define A2S_SERVO_SMOOTH_SLOW_SPEED 20.0f // %/second
#define A2S_SERVO_SMOOTH_SLOW_ACCEL 30.0f // %/second²

// Level 5: Very slow movement (maximum smoothness)
#define A2S_SERVO_SMOOTH_VERY_SLOW_SPEED 10.0f // %/second
#define A2S_SERVO_SMOOTH_VERY_SLOW_ACCEL 15.0f // %/second²
*/

// Level 2: Fast movement
#define A2S_SERVO_SMOOTH_FAST_SPEED 400.0f // %/second
#define A2S_SERVO_SMOOTH_FAST_ACCEL 400.0f // %/second²

// Level 3: Normal movement (balanced)
#define A2S_SERVO_SMOOTH_NORMAL_SPEED 200.0f // %/second
#define A2S_SERVO_SMOOTH_NORMAL_ACCEL 400.0f // %/second²

// Level 4: Slow movement
#define A2S_SERVO_SMOOTH_SLOW_SPEED 100.0f // %/second
#define A2S_SERVO_SMOOTH_SLOW_ACCEL 200.0f // %/second²

// Level 5: Very slow movement (maximum smoothness)
#define A2S_SERVO_SMOOTH_VERY_SLOW_SPEED 50.0f // %/second
#define A2S_SERVO_SMOOTH_VERY_SLOW_ACCEL 100.0f // %/second²

// ----------------------------------------------------------------------------
// Task Configuration
// ----------------------------------------------------------------------------

#define A2S_SERVO_TASK_PRIORITY       12    // Medium priority
#define A2S_SERVO_TASK_STACK_SIZE     4096  // 4KB stack
#define A2S_SERVO_UPDATE_INTERVAL_MS  50    // 50ms update rate (20Hz)
#define A2S_SERVO_MOVEMENT_TIMEOUT_MS 10000 // 10 second timeout

// ----------------------------------------------------------------------------
// Type Definitions
// ----------------------------------------------------------------------------

// Smoothness level enumeration
typedef enum
{
    A2S_SERVO_SMOOTH_INSTANT = 0, // Immediate movement (no smoothing)
    A2S_SERVO_SMOOTH_FAST,        // Fast movement
    A2S_SERVO_SMOOTH_NORMAL,      // Normal movement (default)
    A2S_SERVO_SMOOTH_SLOW,        // Slow movement
    A2S_SERVO_SMOOTH_VERY_SLOW    // Very slow movement
} a2s_servo_smoothness_t;

// Event types for callback notifications
typedef enum
{
    A2S_SERVO_EVENT_MOVE_COMPLETE = 0, // Movement successfully completed
    A2S_SERVO_EVENT_ERROR,             // General error occurred
    A2S_SERVO_EVENT_TIMEOUT,           // Movement timeout
    A2S_SERVO_EVENT_STOPPED            // Movement stopped by user
} a2s_servo_event_t;

// Servo state enumeration
typedef enum
{
    A2S_SERVO_STATE_UNINITIALIZED = 0, // Not initialized
    A2S_SERVO_STATE_IDLE,              // Initialized, at position
    A2S_SERVO_STATE_MOVING,            // Movement in progress
    A2S_SERVO_STATE_ERROR              // Error state
} a2s_servo_state_t;

// Event callback function type
// Parameters:
//   event: Type of event that occurred
//   position: Current position when event occurred (0-100%)
//   user_data: User-provided data pointer from config
typedef void (*a2s_servo_callback_t)(a2s_servo_event_t event,
                                     uint8_t position,
                                     void *user_data);

// Configuration structure
typedef struct
{
    uint8_t gpio_pin;                    // PWM output GPIO pin
    uint16_t min_pulse_us;               // Minimum pulse width (microseconds)
    uint16_t max_pulse_us;               // Maximum pulse width (microseconds)
    uint8_t task_priority;               // Update task priority (0-24)
    uint16_t task_stack_size;            // Update task stack size (bytes)
    uint8_t update_interval_ms;          // Physics update interval (milliseconds)
    a2s_servo_callback_t event_callback; // Event notification callback (optional)
    void *callback_user_data;            // User data for callback (optional)
} a2s_servo_config_t;

// ----------------------------------------------------------------------------
// Public Function Prototypes
// ----------------------------------------------------------------------------

// Configuration helper - returns default configuration
// Parameters:
//   gpio_pin: GPIO pin number for PWM output
// Returns: Default configuration structure
a2s_servo_config_t a2s_servo_default_config(uint8_t gpio_pin);

// Initialize servo driver
// Parameters:
//   config: Pointer to configuration structure
// Returns: ESP_OK on success, error code otherwise
esp_err_t a2s_servo_init(const a2s_servo_config_t *config);

// Deinitialize servo driver and free resources
// Returns: ESP_OK on success
esp_err_t a2s_servo_deinit(void);

// Move servo to target position with specified smoothness
// Parameters:
//   position: Target position (0-100%)
//   smoothness: Movement smoothness level
// Returns: ESP_OK on success, error code otherwise
esp_err_t a2s_servo_move_to(uint8_t position, a2s_servo_smoothness_t smoothness);

// Get current servo position
// Returns: Current position (0-100%)
uint8_t a2s_servo_get_position(void);

// Check if servo is currently moving
// Returns: true if movement in progress, false if at target
bool a2s_servo_is_moving(void);

// Stop current movement immediately
// Returns: ESP_OK on success
esp_err_t a2s_servo_stop(void);

// Get current servo state
// Returns: Current state
a2s_servo_state_t a2s_servo_get_state(void);

// Set event callback (can be changed after init)
// Parameters:
//   callback: Callback function pointer (NULL to disable)
//   user_data: User data pointer for callback
// Returns: ESP_OK on success
esp_err_t a2s_servo_set_callback(a2s_servo_callback_t callback, void *user_data);

#ifdef __cplusplus
}
#endif

#endif // A2S_SERVO_H
